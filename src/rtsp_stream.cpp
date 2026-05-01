#include "rtsp_stream.h"
#include "thermal_state.h"
#include "lepton.h"
// palette access goes through currentPalette() defined in main.cpp (thermal_state.h)

#include <WiFi.h>
#include <CRtspSession.h>
#include <CStreamer.h>
#include <JPEGENC.h>

// smallBuffer, raw_max, raw_min, fpa_temp are defined in lepton.cpp
extern uint16_t smallBuffer[FLIR_X * FLIR_Y];
extern uint16_t raw_max, raw_min, fpa_temp;

#define RTSP_PORT        554
#define STREAM_FPS       8
#define STREAM_PERIOD_MS (1000 / STREAM_FPS)
#define JPEG_QUALITY     60      // 1-100; 60 is a good tradeoff for 160x120
#define JPEG_BUF_BYTES   25000  // well above worst-case for 160x120 @ Q60

// ---------------------------------------------------------------------------
// LeptonStreamer — renders current frame with the active palette and JPEG-encodes it
// ---------------------------------------------------------------------------
class LeptonStreamer : public CStreamer {
    uint8_t* _jpegBuf;
    uint32_t _lastFrameMs;

    // Build a JPEG image from smallBuffer using the current palette + encoder range.
    // Writes into _jpegBuf; returns encoded byte count (0 = failed).
    size_t encodeCurrentFrame() {
        const uint16_t* palette = currentPalette();

        JPEGENC    jpegEnc;
        JPEGENCODE jpe;

        if (jpegEnc.open(_jpegBuf, JPEG_BUF_BYTES) != JPEGE_SUCCESS)
            return 0;

        // JPEG_SUBSAMPLE_422 keeps MCU height = 8 rows; 120 / 8 = 15 (exact)
        if (jpegEnc.encodeBegin(&jpe, FLIR_X, FLIR_Y,
                                 JPEG_PIXEL_RGB565,
                                 JPEG_SUBSAMPLE_422,
                                 JPEG_QUALITY) != JPEGE_SUCCESS) {
            jpegEnc.close();
            return 0;
        }

        // Feed one row at a time; thermalToIndex() mirrors Update_Flir() mapping
        uint16_t row[FLIR_X];
        for (int y = 0; y < FLIR_Y; y++) {
            for (int x = 0; x < FLIR_X; x++) {
                uint8_t idx = thermalToIndex(smallBuffer[y * FLIR_X + x]);
                row[x] = palette[idx];
            }
            if (jpegEnc.addMCURow(&jpe, (uint8_t*)row) != JPEGE_SUCCESS)
                break;
        }

        return (size_t)jpegEnc.close();
    }

public:
    LeptonStreamer(WiFiClient& client)
        : CStreamer(client, /*UDP=*/false), _lastFrameMs(0)
    {
        _jpegBuf = (uint8_t*)malloc(JPEG_BUF_BYTES);
    }

    ~LeptonStreamer() { free(_jpegBuf); }

    void streamImage(uint32_t curmsec) override {
        if (curmsec - _lastFrameMs < STREAM_PERIOD_MS)
            return;
        _lastFrameMs = curmsec;

        if (!_jpegBuf) return;

        size_t jpegLen = encodeCurrentFrame();
        if (jpegLen > 0)
            streamFrame(_jpegBuf, _jpegBuf + jpegLen, curmsec);
    }
};

// ---------------------------------------------------------------------------
// FreeRTOS task — single-client RTSP server
// ---------------------------------------------------------------------------
static void rtspTask(void* /*param*/) {
    WiFiServer server(RTSP_PORT);
    server.begin();
    Serial.printf("[RTSP] Listening on port %d\n", RTSP_PORT);

    for (;;) {
        WiFiClient client = server.accept();
        if (client) {
            Serial.printf("[RTSP] Client connected from %s\n",
                          client.remoteIP().toString().c_str());

            LeptonStreamer streamer(client);
            CRtspSession   session(client, &streamer);

            while (client.connected() && !session.m_stopped) {
                if (client.available())
                    session.handleRequests(0);

                if (session.m_streaming)
                    streamer.streamImage(millis());

                vTaskDelay(1 / portTICK_PERIOD_MS);
            }
            client.stop();
            Serial.println("[RTSP] Client disconnected");
        }
        vTaskDelay(10 / portTICK_PERIOD_MS);
    }
}

void rtspBegin() {
    // Run on Core 0 alongside the startup animation task
    xTaskCreatePinnedToCore(
        rtspTask,
        "rtsp",
        8192,   // 8 KB stack — JPEGENC needs headroom
        nullptr,
        1,
        nullptr,
        0       // Core 0
    );
}
