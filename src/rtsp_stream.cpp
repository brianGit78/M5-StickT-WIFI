#include "rtsp_stream.h"
#include "thermal_state.h"
#include "lepton.h"

#include <WiFi.h>
#include <JPEGENC.h>

extern uint16_t smallBuffer[FLIR_X * FLIR_Y];

#define MJPEG_PORT       8080
#define STREAM_FPS       9
#define STREAM_PERIOD_MS (1000 / STREAM_FPS)
#define JPEG_QUALITY     JPEGE_Q_MED
#define STREAM_W         FLIR_X          // 160 — full sensor width
// 112 = 7×16: must be a multiple of 16 for clean 4:2:0 MCU rows.
// Covers rows 0–111 of the 120-row sensor (drops the last 8 rows, ~7%).
#define STREAM_H         112
#define JPEG_BUF_BYTES   12000

static uint8_t*  s_jpegBuf = nullptr;
static uint16_t* s_rgbBuf  = nullptr;

void rtspAllocBuffers() {
    s_jpegBuf = (uint8_t*)malloc(JPEG_BUF_BYTES);
    s_rgbBuf  = (uint16_t*)malloc(STREAM_W * STREAM_H * sizeof(uint16_t));
    if (!s_jpegBuf || !s_rgbBuf)
        Serial.println("[MJPEG] ERROR: buffer pre-alloc failed");
    else
        Serial.printf("[MJPEG] Buffers OK, heap free: %u\n", ESP.getFreeHeap());
}

static size_t encodeFrame() {
    if (!s_jpegBuf || !s_rgbBuf) return 0;

    // Hold the mutex only for the pixel copy so we get a coherent snapshot of
    // smallBuffer. Without this, the Lepton write (Core 1) races the MJPEG read
    // (Core 0) mid-frame, producing horizontal black bands in the stream.
    xSemaphoreTake(smallBufMutex, portMAX_DELAY);
    const uint16_t* palette = currentPalette();
    for (int sy = 0; sy < STREAM_H; sy++)
        for (int sx = 0; sx < STREAM_W; sx++)
            s_rgbBuf[sy * STREAM_W + sx] =
                palette[thermalToIndex(smallBuffer[sy * FLIR_X + sx])];
    xSemaphoreGive(smallBufMutex);

    JPEGENC    enc;
    JPEGENCODE jpe;
    if (enc.open(s_jpegBuf, JPEG_BUF_BYTES) != JPEGE_SUCCESS)
        return 0;
    if (enc.encodeBegin(&jpe, STREAM_W, STREAM_H,
                         JPEGE_PIXEL_RGB565,
                         JPEGE_SUBSAMPLE_420,
                         JPEG_QUALITY) != JPEGE_SUCCESS) {
        enc.close();
        return 0;
    }
    enc.addFrame(&jpe, (uint8_t*)s_rgbBuf, STREAM_W * sizeof(uint16_t));
    return (size_t)enc.close();
}

static void mjpegTask(void* /*param*/) {
    WiFiServer server(MJPEG_PORT);
    server.begin();
    Serial.printf("[MJPEG] Stream: http://%s:%d/\n",
                  WiFi.localIP().toString().c_str(), MJPEG_PORT);

    for (;;) {
        WiFiClient client = server.available();
        if (!client) {
            vTaskDelay(10 / portTICK_PERIOD_MS);
            continue;
        }
        Serial.printf("[MJPEG] Client from %s\n",
                      client.remoteIP().toString().c_str());

        // Consume HTTP request headers (read until blank line).
        while (client.connected()) {
            if (!client.available()) {
                vTaskDelay(1 / portTICK_PERIOD_MS);
                continue;
            }
            String line = client.readStringUntil('\n');
            if (line == "\r" || line.length() == 0) break;
        }

        client.print("HTTP/1.1 200 OK\r\n");
        client.print("Content-Type: multipart/x-mixed-replace;boundary=jpgboundary\r\n");
        client.print("Connection: close\r\n");
        client.print("\r\n");

        uint32_t lastMs  = 0;
        uint32_t framesSent = 0;
        while (client.connected()) {
            uint32_t now = millis();
            if (now - lastMs < STREAM_PERIOD_MS) {
                vTaskDelay(5 / portTICK_PERIOD_MS);
                continue;
            }
            lastMs = now;

            size_t jpegLen = encodeFrame();
            if (jpegLen == 0) {
                Serial.println("[MJPEG] encode failed");
                continue;
            }

            client.print("--jpgboundary\r\n");
            client.print("Content-Type: image/jpeg\r\n");
            client.printf("Content-Length: %u\r\n\r\n", (unsigned)jpegLen);
            client.write(s_jpegBuf, (int)jpegLen);
            client.print("\r\n");

            framesSent++;
            if (framesSent % 40 == 0)
                Serial.printf("[MJPEG] frames=%u jpegLen=%u heap=%u\n",
                              framesSent, (unsigned)jpegLen, ESP.getFreeHeap());
        }
        client.stop();
        Serial.println("[MJPEG] Client disconnected");
    }
}

void rtspBegin() {
    xTaskCreatePinnedToCore(
        mjpegTask,
        "mjpeg",
        8192,   // actual usage ~5 KB (JPEGENC struct 3.3 KB + WiFiClient + overhead)
        nullptr,
        1,
        nullptr,
        0
    );
}
