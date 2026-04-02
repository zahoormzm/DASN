/*
 * DASN Sentinel Type B — Camera Sentinel
 * Board: ESP32-CAM-MB with OV3660
 *
 * Components:
 *   OV3660 camera, HC-SR501 PIR, INMP441 mic (I2S),
 *   Buzzer (PWM), MicroSD card
 *
 * Features:
 *   - PIR-triggered camera capture
 *   - MJPEG streaming on port 80 when active
 *   - Audio capture via I2S
 *   - ESP-NOW for PIR/audio events to gateway
 *   - MicroSD buffered recording if WiFi drops
 */

#include <WiFi.h>
#include <esp_now.h>
#include <esp_wifi.h>
#include <esp_camera.h>
#include <WebServer.h>
#include <driver/i2s.h>
#include <SD_MMC.h>
#include <FS.h>

#include "../shared/espnow_protocol.h"
#include "../shared/config.h"

// ─── Zone identity ──────────────────────────────────────────────────────────
static const char ZONE_ID[] = "CAM_FRONT";

// ─── WiFi credentials for streaming (AP mode) ─────────────────────────────
#define WIFI_AP_SSID    "DASN_CAM"
#define WIFI_AP_PASS    "dasn1234"

// ─── ESP32-CAM-MB pin mapping (OV3660) ─────────────────────────────────────
#define CAM_PIN_PWDN    32
#define CAM_PIN_RESET   -1
#define CAM_PIN_XCLK     0
#define CAM_PIN_SIOD    26
#define CAM_PIN_SIOC    27
#define CAM_PIN_D7      35
#define CAM_PIN_D6      34
#define CAM_PIN_D5      39
#define CAM_PIN_D4      36
#define CAM_PIN_D3      21
#define CAM_PIN_D2      19
#define CAM_PIN_D1      18
#define CAM_PIN_D0       5
#define CAM_PIN_VSYNC   25
#define CAM_PIN_HREF    23
#define CAM_PIN_PCLK    22

// ─── PIR pin (GPIO 13 on ESP32-CAM) ────────────────────────────────────────
#define PIN_PIR_CAM     13

// ─── Buzzer (GPIO 2 — onboard LED doubled as buzzer pin) ───────────────────
#define PIN_BUZZER_CAM  2
#define BUZZER_CH       0

// ─── I2S mic pins (INMP441) ────────────────────────────────────────────────
// Note: ESP32-CAM has limited free GPIOs. Using GPIO 12,14,15 carefully.
#define PIN_I2S_SCK_CAM  14
#define PIN_I2S_WS_CAM   15
#define PIN_I2S_SD_CAM   12

#define I2S_PORT         I2S_NUM_0
#define I2S_SAMPLE_RATE  16000
#define I2S_BUF_LEN      512

// ─── Globals ────────────────────────────────────────────────────────────────
static volatile bool pirTriggered = false;
static unsigned long pirLastTrigger = 0;
static bool cameraActive    = false;
static bool sdAvailable      = false;
static bool streamingActive  = false;
static unsigned long lastEventSend = 0;

static SentinelData_t sensorData;
static esp_now_peer_info_t gatewayPeer;
static int16_t i2sBuf[I2S_BUF_LEN];

WebServer server(80);

// ─── PIR ISR ────────────────────────────────────────────────────────────────
static void IRAM_ATTR pirISR() {
    unsigned long now = millis();
    if (now - pirLastTrigger > PIR_DEBOUNCE_MS) {
        pirTriggered = true;
        pirLastTrigger = now;
    }
}

// ─── Forward declarations ───────────────────────────────────────────────────
void initCamera();
void initI2S();
void initESPNOW();
void initSD();
void startStreaming();
void handleMJPEGStream();
void handleRoot();
float readMicLevel();
void sendPIREvent();
void saveFrameToSD(camera_fb_t *fb);
void playAlert();

// ═══════════════════════════════════════════════════════════════════════════
void setup() {
    Serial.begin(SERIAL_BAUD_USB);
    Serial.println("[TypeB] Camera Sentinel starting...");

    // Buzzer
    ledcSetup(BUZZER_CH, 2000, 8);
    ledcAttachPin(PIN_BUZZER_CAM, BUZZER_CH);
    ledcWriteTone(BUZZER_CH, 0);

    // PIR
    pinMode(PIN_PIR_CAM, INPUT);
    attachInterrupt(digitalPinToInterrupt(PIN_PIR_CAM), pirISR, RISING);

    // Camera
    initCamera();

    // MicroSD
    initSD();

    // I2S mic
    initI2S();

    // ESP-NOW (uses WiFi STA internally)
    initESPNOW();

    // Start AP for streaming alongside ESP-NOW
    // ESP-NOW works on STA; we set up a SoftAP for MJPEG
    WiFi.softAP(WIFI_AP_SSID, WIFI_AP_PASS);
    Serial.printf("[TypeB] AP started: %s  IP: %s\n",
                  WIFI_AP_SSID, WiFi.softAPIP().toString().c_str());

    startStreaming();

    // Prepare static sensor data fields
    memset(&sensorData, 0, sizeof(sensorData));
    strncpy(sensorData.zone_id, ZONE_ID, sizeof(sensorData.zone_id) - 1);
    sensorData.comm_mode = COMM_MODE_ESPNOW;

    Serial.println("[TypeB] Setup complete");
}

// ═══════════════════════════════════════════════════════════════════════════
void loop() {
    server.handleClient();

    // PIR trigger handling
    if (pirTriggered) {
        pirTriggered = false;
        cameraActive = true;
        Serial.println("[TypeB] PIR triggered — camera active");

        // Capture frame
        camera_fb_t *fb = esp_camera_fb_get();
        if (fb) {
            if (sdAvailable) {
                saveFrameToSD(fb);
            }
            esp_camera_fb_return(fb);
        }

        // Play alert
        playAlert();
    }

    // Periodically send sensor data via ESP-NOW
    unsigned long now = millis();
    if (now - lastEventSend >= SENSOR_READ_INTERVAL_MS) {
        lastEventSend = now;

        // Read mic
        float db = readMicLevel();
        sensorData.sound_level_db  = db;
        sensorData.sound_transient = (db > SOUND_TRANSIENT_DB);
        sensorData.pir_triggered   = (now - pirLastTrigger < 3000);
        sensorData.timestamp       = now;

        // Simple threat: PIR + sound
        float threat = 0.0f;
        if (sensorData.pir_triggered) threat += 0.50f;
        if (sensorData.sound_transient) threat += 0.30f;
        if (db > SOUND_BASELINE_DB) {
            float sn = (db - SOUND_BASELINE_DB) / (SOUND_TRANSIENT_DB - SOUND_BASELINE_DB);
            if (sn > 1.0f) sn = 1.0f;
            threat += 0.20f * sn;
        }
        if (threat > 1.0f) threat = 1.0f;
        sensorData.local_threat_score = threat;

        sendPIREvent();

        // Buzzer for threat
        if (threat >= THREAT_HIGH) {
            playAlert();
        } else {
            ledcWriteTone(BUZZER_CH, 0);
        }
    }

    // Deactivate camera after 30s of no PIR
    if (cameraActive && (now - pirLastTrigger > 30000)) {
        cameraActive = false;
        Serial.println("[TypeB] Camera idle — deactivated");
    }
}

// ─── Camera init ────────────────────────────────────────────────────────────
void initCamera() {
    camera_config_t config = {};
    config.ledc_channel = LEDC_CHANNEL_1;
    config.ledc_timer   = LEDC_TIMER_1;
    config.pin_d0       = CAM_PIN_D0;
    config.pin_d1       = CAM_PIN_D1;
    config.pin_d2       = CAM_PIN_D2;
    config.pin_d3       = CAM_PIN_D3;
    config.pin_d4       = CAM_PIN_D4;
    config.pin_d5       = CAM_PIN_D5;
    config.pin_d6       = CAM_PIN_D6;
    config.pin_d7       = CAM_PIN_D7;
    config.pin_xclk     = CAM_PIN_XCLK;
    config.pin_pclk     = CAM_PIN_PCLK;
    config.pin_vsync    = CAM_PIN_VSYNC;
    config.pin_href     = CAM_PIN_HREF;
    config.pin_sscb_sda = CAM_PIN_SIOD;
    config.pin_sscb_scl = CAM_PIN_SIOC;
    config.pin_pwdn     = CAM_PIN_PWDN;
    config.pin_reset    = CAM_PIN_RESET;
    config.xclk_freq_hz = 20000000;
    config.pixel_format = PIXFORMAT_JPEG;

    // OV3660 supports higher resolution
    config.frame_size   = FRAMESIZE_QVGA;  // 320x240 for streaming
    config.jpeg_quality = 12;
    config.fb_count     = 2;
    config.fb_location  = CAMERA_FB_IN_PSRAM;
    config.grab_mode    = CAMERA_GRAB_LATEST;

    esp_err_t err = esp_camera_init(&config);
    if (err != ESP_OK) {
        Serial.printf("[TypeB] Camera init failed: 0x%x\n", err);
        return;
    }

    // OV3660 specific settings
    sensor_t *s = esp_camera_sensor_get();
    if (s) {
        s->set_brightness(s, 1);
        s->set_contrast(s, 1);
        s->set_saturation(s, 0);
        // OV3660 PID check
        if (s->id.PID == OV3660_PID) {
            Serial.println("[TypeB] OV3660 detected");
            s->set_vflip(s, 1);
        }
    }
    Serial.println("[TypeB] Camera ready");
}

// ─── I2S mic ────────────────────────────────────────────────────────────────
void initI2S() {
    i2s_config_t i2sCfg = {};
    i2sCfg.mode                 = (i2s_mode_t)(I2S_MODE_MASTER | I2S_MODE_RX);
    i2sCfg.sample_rate          = I2S_SAMPLE_RATE;
    i2sCfg.bits_per_sample      = I2S_BITS_PER_SAMPLE_16BIT;
    i2sCfg.channel_format       = I2S_CHANNEL_FMT_ONLY_LEFT;
    i2sCfg.communication_format = I2S_COMM_FORMAT_STAND_I2S;
    i2sCfg.intr_alloc_flags     = ESP_INTR_FLAG_LEVEL1;
    i2sCfg.dma_buf_count        = 4;
    i2sCfg.dma_buf_len          = I2S_BUF_LEN;
    i2sCfg.use_apll             = false;

    i2s_driver_install(I2S_PORT, &i2sCfg, 0, NULL);

    i2s_pin_config_t pinCfg = {};
    pinCfg.bck_io_num   = PIN_I2S_SCK_CAM;
    pinCfg.ws_io_num    = PIN_I2S_WS_CAM;
    pinCfg.data_in_num  = PIN_I2S_SD_CAM;
    pinCfg.data_out_num = I2S_PIN_NO_CHANGE;

    i2s_set_pin(I2S_PORT, &pinCfg);
    Serial.println("[TypeB] I2S mic ready");
}

float readMicLevel() {
    size_t bytesRead = 0;
    i2s_read(I2S_PORT, i2sBuf, sizeof(i2sBuf), &bytesRead, pdMS_TO_TICKS(50));
    int samples = bytesRead / sizeof(int16_t);
    if (samples == 0) return 0.0f;

    float sumSq = 0.0f;
    for (int i = 0; i < samples; i++) {
        float s = (float)i2sBuf[i];
        sumSq += s * s;
    }
    float rms = sqrtf(sumSq / (float)samples);
    return 20.0f * log10f(rms + 1.0f);
}

// ─── ESP-NOW ────────────────────────────────────────────────────────────────
static void onDataSent(const uint8_t *mac, esp_now_send_status_t status) {
    if (status != ESP_NOW_SEND_SUCCESS) {
        Serial.println("[TypeB] ESP-NOW send failed");
    }
}

void initESPNOW() {
    WiFi.mode(WIFI_AP_STA);
    esp_wifi_set_channel(ESPNOW_CHANNEL, WIFI_SECOND_CHAN_NONE);

    if (esp_now_init() != ESP_OK) {
        Serial.println("[TypeB] ESP-NOW init failed");
        return;
    }
    esp_now_register_send_cb(onDataSent);

    memset(&gatewayPeer, 0, sizeof(gatewayPeer));
    memcpy(gatewayPeer.peer_addr, GATEWAY_MAC, 6);
    gatewayPeer.channel = ESPNOW_CHANNEL;
    gatewayPeer.encrypt = ESPNOW_ENCRYPT;

    if (esp_now_add_peer(&gatewayPeer) != ESP_OK) {
        Serial.println("[TypeB] Failed to add gateway peer");
    }
    Serial.println("[TypeB] ESP-NOW ready");
}

void sendPIREvent() {
    esp_err_t result = esp_now_send(GATEWAY_MAC,
                                    (uint8_t *)&sensorData,
                                    sizeof(SentinelData_t));
    if (result != ESP_OK) {
        Serial.printf("[TypeB] ESP-NOW error: %d\n", result);
    }
}

// ─── MicroSD ────────────────────────────────────────────────────────────────
void initSD() {
    if (SD_MMC.begin("/sdcard", true)) { // 1-bit mode
        sdAvailable = true;
        Serial.printf("[TypeB] SD card: %lluMB\n",
                      SD_MMC.totalBytes() / (1024 * 1024));
    } else {
        sdAvailable = false;
        Serial.println("[TypeB] SD card not available");
    }
}

void saveFrameToSD(camera_fb_t *fb) {
    if (!sdAvailable || !fb) return;

    char filename[40];
    snprintf(filename, sizeof(filename), "/capture_%lu.jpg", millis());

    File file = SD_MMC.open(filename, FILE_WRITE);
    if (file) {
        file.write(fb->buf, fb->len);
        file.close();
        Serial.printf("[TypeB] Saved %s (%u bytes)\n", filename, fb->len);
    } else {
        Serial.println("[TypeB] SD write failed");
    }
}

// ─── MJPEG streaming ───────────────────────────────────────────────────────
#define PART_BOUNDARY "123456789000000000000987654321"
static const char *STREAM_CONTENT_TYPE =
    "multipart/x-mixed-replace;boundary=" PART_BOUNDARY;
static const char *STREAM_BOUNDARY =
    "\r\n--" PART_BOUNDARY "\r\n";
static const char *STREAM_PART =
    "Content-Type: image/jpeg\r\nContent-Length: %u\r\n\r\n";

void handleRoot() {
    String html = "<!DOCTYPE html><html><body>"
                  "<h1>DASN Camera Sentinel</h1>"
                  "<p>Zone: " + String(ZONE_ID) + "</p>"
                  "<img src=\"/stream\" width=\"640\">"
                  "</body></html>";
    server.send(200, "text/html", html);
}

void handleMJPEGStream() {
    WiFiClient client = server.client();
    if (!client.connected()) return;

    client.println("HTTP/1.1 200 OK");
    client.printf("Content-Type: %s\r\n", STREAM_CONTENT_TYPE);
    client.println("Connection: close");
    client.println();

    while (client.connected()) {
        camera_fb_t *fb = esp_camera_fb_get();
        if (!fb) {
            Serial.println("[TypeB] Frame capture failed");
            break;
        }

        client.print(STREAM_BOUNDARY);
        char partBuf[64];
        snprintf(partBuf, sizeof(partBuf), STREAM_PART, fb->len);
        client.print(partBuf);
        client.write(fb->buf, fb->len);

        esp_camera_fb_return(fb);

        if (!client.connected()) break;
        delay(33); // ~30 fps cap
    }
}

void startStreaming() {
    server.on("/", HTTP_GET, handleRoot);
    server.on("/stream", HTTP_GET, handleMJPEGStream);
    server.begin();
    streamingActive = true;
    Serial.println("[TypeB] MJPEG server started on port 80");
}

// ─── Buzzer ─────────────────────────────────────────────────────────────────
void playAlert() {
    ledcWriteTone(BUZZER_CH, BUZZER_FREQ_HIGH);
    delay(150);
    ledcWriteTone(BUZZER_CH, 0);
    delay(50);
    ledcWriteTone(BUZZER_CH, BUZZER_FREQ_HIGH);
    delay(150);
    ledcWriteTone(BUZZER_CH, 0);
}
