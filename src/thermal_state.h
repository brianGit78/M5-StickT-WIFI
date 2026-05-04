#pragma once
#include <stdint.h>
#include <freertos/FreeRTOS.h>
#include <freertos/semphr.h>

// Mutex that must be held while reading OR writing smallBuffer.
// Created in main.cpp setup(); used by loop() around getRawValues()
// and by encodeFrame() around the pixel copy.
extern SemaphoreHandle_t smallBufMutex;

// Encoder knob state — defined in main.cpp
typedef struct {
    float    data;       // -0.95..+0.95: controls which end of the temp range to clip
    int16_t  increment;
    uint8_t  sw;
} encoder_t;

// Both defined in main.cpp; read by rtsp_stream.cpp
extern uint8_t   disp_mode;
extern encoder_t encoder;

// Returns the 256-entry RGB565 palette currently selected by disp_mode.
// Defined in main.cpp so palette tables don't have to be linked twice.
const uint16_t* currentPalette();

// Maps a single raw Lepton value to a 0-255 palette index using the same
// formula as DisplayImage() / Update_Flir() so the RTSP frame matches the LCD.
// Requires raw_max, raw_min, fpa_temp, encoder (all visible via lepton.h / externs).
uint8_t thermalToIndex(uint16_t raw);
