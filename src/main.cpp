#include <Arduino.h>
#include <M5StickT.h>
#include <Wire.h>
#include <SPI.h>
#include "lepton.h"
#include "img_table.h"
#include "esp_attr.h"
#include "esp_timer.h"
#include "esp_task_wdt.h"
#include "esp_system.h"
#include "img/ColorT.h"
#include "wifi_manager.h"
#include "thermal_state.h"
#include "mjpeg_stream.h"

// Fired by FreeRTOS when any task overflows its stack — prints the offending
// task name and forces a clean restart so the cause is visible in the next boot log.
extern "C" void vApplicationStackOverflowHook(TaskHandle_t, char* name) {
    Serial.printf("\n[CRASH] Stack overflow in task: %s — restarting\n", name);
    Serial.flush();
    esp_restart();
}

#define ENCODER_ADDR 0x30

#define SCREEN_X 240
#define SCREEN_Y 135
#define FLIR_X 160
#define FLIR_Y 120
#define MAX_FLIR_RAW_BUFFER (FLIR_X * FLIR_Y - 1)

#define FLIR_WINDOW_X1 5
#define FLIR_WINDOW_Y1 4
#define FLIR_WINDOW_X2 (FLIR_WINDOW_X1 + FLIR_X)
#define FLIR_WINDOW_Y2 (FLIR_WINDOW_Y1 + FLIR_Y)
#define FLIR_WINDOW_CENTER_X (FLIR_WINDOW_X1 + FLIR_X / 2)
#define FLIR_WINDOW_CENTER_Y (FLIR_WINDOW_Y1 + FLIR_Y / 2)

#define HIST_HEIGHT 70
#define HIST_WINDOWS_X1 171
#define HIST_WINDOWS_Y1 34
#define HIST_WINDOWS_X2 (HIST_WINDOWS_X1 + 64)
#define HIST_WINDOWS_Y2 (HIST_WINDOWS_Y1 + HIST_HEIGHT)

#define MES_MODE_X 171
#define MES_MODE_Y 8

#define DISP_MODE_X 171
#define DISP_MODE_Y 25

//SDA SCL CS VSYNC
Lepton lepton(21, 22, 0, 38);
extern uint16_t fpa_temp, aux_temp;
extern const uint16_t camColors[];
extern const uint16_t GrayLevel[];
extern uint16_t smallBuffer[FLIR_X * FLIR_Y];
extern uint16_t raw_max, raw_min;
extern uint16_t max_x, max_y, min_x, min_y;
TFT_eSprite img_buffer = TFT_eSprite(&M5.Lcd);

enum modes
{
    MES_AUTO_MAX = 0,
    MES_AUTO_MIN,
    MES_CENTER,
    DISP_MODE_CAM = 0,
    DISP_MODE_GRAY,
    DISP_MODE_GOLDEN,
    DISP_MODE_RAINBOW,
    DISP_MODE_IRONBLACK,
};

// encoder_t is defined in thermal_state.h
encoder_t encoder = {0};
uint16_t hist_buffer[64] = {0};
uint8_t mes_mode = MES_AUTO_MAX;
uint8_t disp_mode = DISP_MODE_RAINBOW;

// ---------------------------------------------------------------------------
// thermal_state.h implementations — used by mjpeg_stream.cpp
// ---------------------------------------------------------------------------

const uint16_t* currentPalette() {
    switch (disp_mode) {
        case DISP_MODE_CAM:       return colormap_cam;
        case DISP_MODE_GRAY:      return colormap_grayscale;
        case DISP_MODE_GOLDEN:    return colormap_golden;
        case DISP_MODE_RAINBOW:   return colormap_rainbow;
        case DISP_MODE_IRONBLACK: return colormap_ironblack;
        default:                  return colormap_cam;
    }
}

// Mirrors the Update_Flir() range calculation exactly so RTSP colors match the LCD.
uint8_t thermalToIndex(uint16_t raw) {
    if (raw_max == raw_min) return 0;

    float fpa_f     = fpa_temp / 100.0f - 273.15f;
    float max_temp  = 0.0217f * raw_max + fpa_f - 177.77f;
    float min_temp  = 0.0217f * raw_min + fpa_f - 177.77f;

    float enc = encoder.data;
    bool  dir_flag = (enc >= 0.0f);
    if (enc >  0.95f) enc =  0.95f;
    if (enc < -0.95f) enc = -0.95f;

    uint16_t raw_cursor;
    float    raw_diff;

    if (dir_flag) {
        float ct  = min_temp + (max_temp - min_temp) * enc;
        raw_cursor = (uint16_t)((ct + 177.77f - fpa_f) / 0.0217f);
        if (raw_max <= raw_cursor) return 0;
        raw_diff = 256.0f / (raw_max - raw_cursor);
        if (raw < raw_cursor) return 0;
        uint32_t idx = (uint32_t)((raw - raw_cursor) * raw_diff);
        return (uint8_t)(idx > 255 ? 255 : idx);
    } else {
        float ct  = max_temp - (max_temp - min_temp) * (-enc);
        raw_cursor = (uint16_t)((ct + 177.77f - fpa_f) / 0.0217f);
        if (raw_cursor <= raw_min) return 255;
        raw_diff = 256.0f / (raw_cursor - raw_min);
        if (raw > raw_cursor) return 255;
        uint32_t idx = (uint32_t)((raw - raw_min) * raw_diff);
        return (uint8_t)(idx > 255 ? 255 : idx);
    }
}

// ---------------------------------------------------------------------------

uint8_t I2CSetReadReg(uint8_t reg_addr)
{
    Wire1.beginTransmission(ENCODER_ADDR);
    Wire1.write(reg_addr);
    uint8_t err = Wire1.endTransmission();
    return err;
}

uint8_t I2CWriteReg(uint8_t reg_addr, uint8_t value)
{
    Wire1.beginTransmission(ENCODER_ADDR);
    Wire1.write(reg_addr);
    Wire1.write(value);
    uint8_t err = Wire1.endTransmission();
    return err;
}

const float kEncoderTempStep = 0.05f;
void UpdateEncoder(encoder_t *encoder)
{
    I2CSetReadReg(0x10);
    Wire1.requestFrom(ENCODER_ADDR, 1);
    encoder->sw = Wire1.read();

    if (encoder->sw != 0)
    {
        encoder->data = 0;
        I2CWriteReg(0x20, 0xFF);
    }
    else
    {
        I2CSetReadReg(0x00);
        Wire1.requestFrom(ENCODER_ADDR, 2);
        encoder->increment = Wire1.read() << 8;
        encoder->increment |= Wire1.read();

        if (encoder->increment != 0)
        {
            encoder->data += ((encoder->increment) * kEncoderTempStep);
            I2CWriteReg(0x20, 0xFF);
        }
    }
}

void IRAM_ATTR Display_Cursor(uint16_t x, uint16_t y)
{
    img_buffer.drawCircle(x, y, 6, TFT_WHITE);
    img_buffer.drawLine(x, y - 10, x, y + 10, TFT_WHITE);
    img_buffer.drawLine(x - 10, y, x + 10, y, TFT_WHITE);
}

void DisplayImage(float raw_diff, uint16_t raw_cursor, const uint16_t *palette, bool dir_flag)
{
    uint16_t x, y, i = 0;
    uint16_t index = 0;

    if (dir_flag)
    {
        for (y = FLIR_WINDOW_Y1; y < FLIR_WINDOW_Y2; y++)
        {
            for (x = FLIR_WINDOW_X1; x < FLIR_WINDOW_X2; x++)
            {
                if (smallBuffer[i] < raw_cursor)
                {
                    index = 0;
                }
                else
                {
                    index = (smallBuffer[i] - raw_cursor) * raw_diff;
                }
                if (index > 255)
                    index = 255;
                hist_buffer[(index >> 2)]++;
                img_buffer.drawPixel(x, y, *(palette + index));
                i++;
            }
        }
    }
    else
    {
        for (y = FLIR_WINDOW_Y1; y < FLIR_WINDOW_Y2; y++)
        {
            for (x = FLIR_WINDOW_X1; x < FLIR_WINDOW_X2; x++)
            {
                if (smallBuffer[i] > raw_cursor)
                {
                    index = 255;
                }
                else
                {
                    index = (smallBuffer[i] - raw_min) * raw_diff;
                }
                if (index > 255)
                    index = 255;
                hist_buffer[(index >> 2)]++;
                img_buffer.drawPixel(x, y, *(palette + index));
                i++;
            }
        }
    }
}

void DrawBattery(uint16_t x, uint16_t y, float vol)
{
    const uint8_t w = 18;
    const uint8_t h = 7;
    img_buffer.drawLine(x + 1, y, x + w, y, TFT_WHITE);
    img_buffer.drawLine(x, y + 1, x, y + h, TFT_WHITE);
    img_buffer.drawLine(x + 1, y + h + 1, x + w, y + h + 1, TFT_WHITE);
    img_buffer.drawLine(x + w + 1, y + 1, x + w + 1, y + h, TFT_WHITE);
    img_buffer.drawLine(x + w + 3, y + 4, x + w + 3, y + h - 3, TFT_WHITE);
    img_buffer.drawPixel(x + w + 2, y + 3, TFT_WHITE);
    img_buffer.drawPixel(x + w + 2, y + h - 2, TFT_WHITE);

    float rate = (vol - 3.4) / (4.1 - 3.4);
    if (rate > 1.0)
    {
        img_buffer.fillRect(x + 2, y + 2, w - 2, h - 2, TFT_GREEN);
    }
    else if (rate <= 0.05)
    {
        img_buffer.drawLine(x + 2, y + 2, x + 2, y + h - 1, TFT_GREEN);
    }
    else
    {
        img_buffer.fillRect(x + 2, y + 2, uint16_t(rate * (w - 2)), h - 2, TFT_GREEN);
    }
}

void IRAM_ATTR Update_Flir()
{
    UpdateEncoder(&encoder);
    uint16_t i, raw_cursor = raw_max;
    int32_t x, y;
    float raw_diff = 0;
    img_buffer.fillRect(0, 0, SCREEN_X, SCREEN_Y, TFT_BLACK);

    float fpa_temp_f = fpa_temp / 100.0f - 273.15;
    float max_temp = 0.0217f * raw_max + fpa_temp_f - 177.77f;
    float min_temp = 0.0217f * raw_min + fpa_temp_f - 177.77f;
    float center_temp = 0.0217f * smallBuffer[9519] + fpa_temp_f - 177.77f;

    float cursor_temp = max_temp;
    bool dir_flag = encoder.data >= 0;
    if (dir_flag)
    {
        if (encoder.data > 0.95)
        {
            encoder.data = 0.95;
        }

        cursor_temp = min_temp + (max_temp - min_temp) * encoder.data;
        raw_cursor = (cursor_temp + 177.77 - fpa_temp_f) / 0.0217f;
        raw_diff = 256.0f / (raw_max - raw_cursor);
    }
    else
    {
        if (encoder.data < -0.95)
        {
            encoder.data = -0.95;
        }

        cursor_temp = max_temp - ((max_temp - min_temp) * (-encoder.data));
        raw_cursor = (cursor_temp + 177.77 - fpa_temp_f) / 0.0217f;
        raw_diff = 256.0f / (raw_cursor - raw_min);
    }

    max_x += FLIR_WINDOW_X1;
    max_y += FLIR_WINDOW_Y1;
    min_x += FLIR_WINDOW_X1;
    min_y += FLIR_WINDOW_Y1;

    i = 0;
    switch (disp_mode)
    {
    case DISP_MODE_CAM:
        DisplayImage(raw_diff, raw_cursor, colormap_cam, dir_flag);
        break;

    case DISP_MODE_GRAY:
        DisplayImage(raw_diff, raw_cursor, colormap_grayscale, dir_flag);
        break;

    case DISP_MODE_GOLDEN:
        DisplayImage(raw_diff, raw_cursor, colormap_golden, dir_flag);
        break;

    case DISP_MODE_RAINBOW:
        DisplayImage(raw_diff, raw_cursor, colormap_rainbow, dir_flag);
        break;

    case DISP_MODE_IRONBLACK:
        DisplayImage(raw_diff, raw_cursor, colormap_ironblack, dir_flag);
        break;
    }

    switch (mes_mode)
    {
    case MES_AUTO_MAX:
        Display_Cursor(max_x, max_y);
        x = max_x + 5;
        y = max_y + 5;
        if (max_x > FLIR_WINDOW_X2 - 35)
            x = max_x - 35;
        if (max_y > FLIR_WINDOW_Y2 - 15)
            y = max_y - 15;
        img_buffer.setCursor(x, y);
        img_buffer.printf("%.2f", max_temp);
        break;

    case MES_AUTO_MIN:
        Display_Cursor(min_x, min_y);
        x = min_x + 5;
        y = min_y + 5;
        if (min_x > FLIR_WINDOW_X2 - 35)
            x = min_x - 35;
        if (min_y > FLIR_WINDOW_Y2 - 15)
            y = min_y - 15;
        img_buffer.setCursor(x, y);
        img_buffer.printf("%.2f", min_temp);
        break;

    case MES_CENTER:
        Display_Cursor(FLIR_WINDOW_CENTER_X, FLIR_WINDOW_CENTER_Y);
        img_buffer.setCursor(FLIR_WINDOW_CENTER_X + 5, FLIR_WINDOW_CENTER_Y + 5);
        img_buffer.printf("%.2f", center_temp);
        break;
    }

    // Histogram
    uint16_t max_hist = 0;
    for (i = 0; i < 64; i++)
    {
        if (hist_buffer[i] > max_hist)
        {
            max_hist = hist_buffer[i];
        }
    }

    uint16_t hist_div = max_hist / HIST_HEIGHT;

    i = 0;
    switch (disp_mode)
    {
    case DISP_MODE_CAM:
        for (x = HIST_WINDOWS_X1; x < HIST_WINDOWS_X2; x++)
        {
            img_buffer.drawLine(x, HIST_WINDOWS_Y2, x, HIST_WINDOWS_Y2 - hist_buffer[i] / hist_div, colormap_cam[i * 4]);
            hist_buffer[i] = 0;
            i++;
        }
        break;

    case DISP_MODE_GRAY:
        for (x = HIST_WINDOWS_X1; x < HIST_WINDOWS_X2; x++)
        {
            img_buffer.drawLine(x, HIST_WINDOWS_Y2, x, HIST_WINDOWS_Y2 - hist_buffer[i] / hist_div, colormap_grayscale[i * 4]);
            hist_buffer[i] = 0;
            i++;
        }
        break;

    case DISP_MODE_GOLDEN:
        for (x = HIST_WINDOWS_X1; x < HIST_WINDOWS_X2; x++)
        {
            img_buffer.drawLine(x, HIST_WINDOWS_Y2, x, HIST_WINDOWS_Y2 - hist_buffer[i] / hist_div, colormap_golden[i * 4]);
            hist_buffer[i] = 0;
            i++;
        }
        break;

    case DISP_MODE_RAINBOW:
        for (x = HIST_WINDOWS_X1; x < HIST_WINDOWS_X2; x++)
        {
            img_buffer.drawLine(x, HIST_WINDOWS_Y2, x, HIST_WINDOWS_Y2 - hist_buffer[i] / hist_div, colormap_rainbow[i * 4]);
            hist_buffer[i] = 0;
            i++;
        }
        break;

    case DISP_MODE_IRONBLACK:
        for (x = HIST_WINDOWS_X1; x < HIST_WINDOWS_X2; x++)
        {
            img_buffer.drawLine(x, HIST_WINDOWS_Y2, x, HIST_WINDOWS_Y2 - hist_buffer[i] / hist_div, colormap_ironblack[i * 4]);
            hist_buffer[i] = 0;
            i++;
        }
        break;
    }

    double bar_percentage = (double)(raw_cursor - raw_min) / (double)(raw_max - raw_min);
    uint8_t bar_len = bar_percentage * 64;
    img_buffer.drawRect(HIST_WINDOWS_X1, HIST_WINDOWS_Y2 + 5, 64, 4, TFT_WHITE);
    if (dir_flag)
    {
        img_buffer.fillRect(HIST_WINDOWS_X1 + bar_len, HIST_WINDOWS_Y2 + 5, 64 - bar_len, 4, TFT_WHITE);
    }
    else
    {
        img_buffer.fillRect(HIST_WINDOWS_X1, HIST_WINDOWS_Y2 + 5, bar_len, 4, TFT_WHITE);
    }

    img_buffer.setCursor(HIST_WINDOWS_X1, HIST_WINDOWS_Y2 + 12);
    img_buffer.printf("%.0f", min_temp);
    img_buffer.setCursor(HIST_WINDOWS_X2 - 12, HIST_WINDOWS_Y2 + 12);
    img_buffer.printf("%.0f", max_temp);

    uint8_t axp_button = M5.Axp.GetBtnPress();
    M5.update();

    if (axp_button == 0x01)
    {
        M5.Axp.Write1Byte(0x32, 0x80);
    }
    else if (axp_button == 0x02 || M5.BtnB.wasReleased())
    {
        mes_mode++;
        if (mes_mode > MES_CENTER)
        {
            mes_mode = MES_AUTO_MAX;
        }
    }

    if (M5.BtnA.wasReleased())
    {
        disp_mode++;
        if (disp_mode > DISP_MODE_IRONBLACK)
        {
            disp_mode = DISP_MODE_CAM;
        }
    }

    img_buffer.setTextDatum(TC_DATUM);
    float bat_voltage = M5.Axp.GetBatVoltage();
    DrawBattery(214, 4, bat_voltage);

    switch (disp_mode)
    {
    case DISP_MODE_CAM:
        img_buffer.drawString("RGB", HIST_WINDOWS_X1 + 20, 5);
        break;

    case DISP_MODE_GRAY:
        img_buffer.drawString("GRAY", HIST_WINDOWS_X1 + 20, 5);
        break;

    case DISP_MODE_GOLDEN:
        img_buffer.drawString("GOLDEN", HIST_WINDOWS_X1 + 20, 5);
        break;

    case DISP_MODE_RAINBOW:
        img_buffer.drawString("RAINBOW", HIST_WINDOWS_X1 + 20, 5);
        break;

    case DISP_MODE_IRONBLACK:
        img_buffer.drawString("IRON", HIST_WINDOWS_X1 + 20, 5);
        break;
    }

    img_buffer.pushSprite(0, 0);

    static uint32_t lastPushLog = 0;
    uint32_t _now = millis();
    if (_now - lastPushLog > 5000) {
        lastPushLog = _now;
        Serial.printf("[UFl] pushed @ %ums raw_diff=%.2f cursor=%u\n", _now, raw_diff, raw_cursor);
    }
}

#define COLORT_Y 27
#define COLORT_X 160
bool start_anime_flag = true;
void Progress_Bar(void *pvParameters)
{
    uint8_t frame = 0;
    uint8_t round = 0;
    while (1)
    {
        switch (frame)
        {
        case 0:  M5.Lcd.drawBitmap(COLORT_X, COLORT_Y, 50, 73, (uint16_t *)ColorT_0000_16); break;
        case 1:  M5.Lcd.drawBitmap(COLORT_X, COLORT_Y, 50, 73, (uint16_t *)ColorT_0001_15); break;
        case 2:  M5.Lcd.drawBitmap(COLORT_X, COLORT_Y, 50, 73, (uint16_t *)ColorT_0002_14); break;
        case 3:  M5.Lcd.drawBitmap(COLORT_X, COLORT_Y, 50, 73, (uint16_t *)ColorT_0003_13); break;
        case 4:  M5.Lcd.drawBitmap(COLORT_X, COLORT_Y, 50, 73, (uint16_t *)ColorT_0004_12); break;
        case 5:  M5.Lcd.drawBitmap(COLORT_X, COLORT_Y, 50, 73, (uint16_t *)ColorT_0005_11); break;
        case 6:  M5.Lcd.drawBitmap(COLORT_X, COLORT_Y, 50, 73, (uint16_t *)ColorT_0006_10); break;
        case 7:  M5.Lcd.drawBitmap(COLORT_X, COLORT_Y, 50, 73, (uint16_t *)ColorT_0007_9);  break;
        case 8:  M5.Lcd.drawBitmap(COLORT_X, COLORT_Y, 50, 73, (uint16_t *)ColorT_0008_8);  break;
        case 9:  M5.Lcd.drawBitmap(COLORT_X, COLORT_Y, 50, 73, (uint16_t *)ColorT_0009_7);  break;
        case 10: M5.Lcd.drawBitmap(COLORT_X, COLORT_Y, 50, 73, (uint16_t *)ColorT_0010_6);  break;
        case 11: M5.Lcd.drawBitmap(COLORT_X, COLORT_Y, 50, 73, (uint16_t *)ColorT_0011_5);  break;
        case 12: M5.Lcd.drawBitmap(COLORT_X, COLORT_Y, 50, 73, (uint16_t *)ColorT_0012_4);  break;
        case 13: M5.Lcd.drawBitmap(COLORT_X, COLORT_Y, 50, 73, (uint16_t *)ColorT_0013_3);  break;
        case 14: M5.Lcd.drawBitmap(COLORT_X, COLORT_Y, 50, 73, (uint16_t *)ColorT_0014_2);  break;
        case 15: M5.Lcd.drawBitmap(COLORT_X, COLORT_Y, 50, 73, (uint16_t *)ColorT_0015_1);  break;
        case 16: M5.Lcd.drawBitmap(COLORT_X, COLORT_Y, 50, 73, (uint16_t *)ColorT_0016_0);  break;
        }

        frame++;
        if (frame > 16)
        {
            frame = 0;
            round++;
            if (round > 4)
            {
                delay(1500);
                start_anime_flag = false;
                break;
            }
        }
        delay(45);
    }
    vTaskDelete(NULL);
}

SemaphoreHandle_t smallBufMutex;

void setup()
{
    smallBufMutex = xSemaphoreCreateMutex();
    if (!smallBufMutex) {
        // Without this mutex the Lepton/MJPEG data sharing is unsafe.
        // Restart and let setup() try again rather than run corrupted.
        Serial.println("[FATAL] smallBufMutex alloc failed — restarting");
        Serial.flush();
        esp_restart();
    }

    esp_timer_init();
    delay(100);
    M5.begin();

    // --- Crash diagnostics: print why we restarted before anything else runs ---
    static const char* resetNames[] = {
        "unknown", "power-on", "external", "software",
        "panic/exception", "interrupt WDT", "task WDT",
        "other WDT", "deep-sleep", "brownout", "SDIO"
    };
    esp_reset_reason_t reason = esp_reset_reason();
    int ri = (int)reason;
    Serial.printf("\n[BOOT] reset reason: %s (%d)\n",
                  (ri >= 0 && ri <= 10) ? resetNames[ri] : "?", ri);
    if (reason == ESP_RST_TASK_WDT || reason == ESP_RST_INT_WDT) {
        Serial.println("[BOOT] *** Watchdog reset — check for tasks blocking >5 s ***");
    }

    // Extend the Task WDT to 15 s to cover worst-case lepton operations:
    //   4 segments × 500ms VSYNC timeout + 2 × 2s I2C waitIdle = ~6 s typical max.
    // The yield added to WaitForVsync() should keep us well under this, but 15 s
    // gives headroom for FFC bursts or I2C retries without a spurious reset.
    esp_task_wdt_init(15, true);  // 15-second timeout, panic=true (prints backtrace)

    M5.Lcd.setRotation(1);
    M5.Lcd.fillScreen(TFT_WHITE);
    M5.Lcd.drawBitmap(29, 39, 132, 57, (uint16_t *)title);

    disableCore0WDT();
    xTaskCreatePinnedToCore(
        Progress_Bar,
        "Progress_Bar",
        4096,
        NULL,
        1,
        NULL,
        0);

    void* spr = img_buffer.createSprite(SCREEN_X, SCREEN_Y);
    Serial.printf("[setup] Sprite: %s heap=%u\n", spr ? "OK" : "FAILED", (unsigned)ESP.getFreeHeap());
    img_buffer.setTextSize(1);
    img_buffer.setTextColor(TFT_WHITE, TFT_BLACK);

    // Init Lepton BEFORE WiFi: Lepton delays are on Core 1, so they don't touch Core 0
    // where Progress_Bar runs. WiFi tasks run on Core 0 at priority 23 and would starve
    // Progress_Bar (priority 1), freezing the animation.
    Serial.println("[setup] lepton.begin()");
    lepton.begin();
    Serial.println("[setup] lepton.syncFrame()");
    lepton.syncFrame();
    Serial.println("[setup] doSetCommand 0x4854");
    uint16_t SYNC = 5, DELAY = 3;
    lepton.doSetCommand(0x4854, &SYNC, 1);
    Serial.println("[setup] doSetCommand 0x4858");
    lepton.doSetCommand(0x4858, &DELAY, 1);
    Serial.println("[setup] lepton.end()");
    lepton.end();
    Serial.println("[setup] Lepton init done");

    while (start_anime_flag)
    {
        delay(1);
    }
    Serial.printf("[setup] animation done @ %ums heap=%u\n", millis(), (unsigned)ESP.getFreeHeap());

    // After ~85 pushImage() calls from the animation task, the VSPI hardware
    // state can be corrupted. hardReinit() calls spi.end() first (sets _spi=NULL)
    // so the subsequent spi.begin() inside init() actually reconfigures VSPI,
    // then sends RST + full ST7789 init sequence so DISPON reaches the display.
    M5.Lcd.hardReinit();

    // Allocate stream buffers BEFORE WiFi so they land in contiguous pre-fragmentation heap.
    // WiFi takes ~100 KB in many small chunks; a 36 KB contiguous alloc won't fit after that.
    mjpegAllocBuffers();

    // Start WiFi only AFTER the animation finishes — avoids Core 0 starvation.
    wifiBegin();
    {
        uint32_t deadline = millis() + 5000;
        while (millis() < deadline && !wifiIsConnected()) {
            delay(100);
        }
    }

    // Start MJPEG server and mDNS if WiFi is up
    if (wifiIsConnected()) {
        Serial.printf("[setup] WiFi OK ip=%s\n", wifiGetIP().c_str());
        wifiSetupMDNS();
        mjpegBegin();
    } else {
        Serial.println("[setup] WiFi FAILED");
    }
}

void loop()
{
    static uint32_t lastHeartbeatMs = 0;
    uint32_t now = millis();

    if (now - lastHeartbeatMs > 3000) {
        lastHeartbeatMs = now;
        Serial.printf("[loop] t=%ums raw_max=%u raw_min=%u fpa=%.1fC heap=%u minHeap=%u tasks=%u stackHWM=%u\n",
                      now, raw_max, raw_min,
                      raw_max ? (fpa_temp / 100.0f - 273.15f) : 0.0f,
                      (unsigned)ESP.getFreeHeap(),
                      (unsigned)ESP.getMinFreeHeap(),
                      (unsigned)uxTaskGetNumberOfTasks(),
                      (unsigned)uxTaskGetStackHighWaterMark(NULL));
    }

    // Validate handle before use: a non-null but out-of-DRAM value (e.g. 0x1ded1ded)
    // causes a LoadProhibited fault inside xSemaphoreTake at Queue_t offset +8.
    {
        uintptr_t a = (uintptr_t)smallBufMutex;
        if (a < 0x3FFA0000UL || a >= 0x40000000UL) {
            Serial.printf("[loop] PANIC: smallBufMutex corrupted (%p) — restarting\n",
                          smallBufMutex);
            Serial.flush();
            esp_restart();
        }
    }
    xSemaphoreTake(smallBufMutex, portMAX_DELAY);
    lepton.getRawValues();
    xSemaphoreGive(smallBufMutex);
    mjpegNotifyFrame();   // signal MJPEG task: fresh lepton data is ready
    Update_Flir();
}
