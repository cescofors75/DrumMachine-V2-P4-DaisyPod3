// =============================================================================
// DrumMachine V2 P4 — Guition ESP32-P4 JC1060P470C (7" MIPI-DSI)
// =============================================================================
#pragma once

#include <Arduino.h>

// =============================================================================
// DEBUG LOGGING
// =============================================================================
#ifndef P4_ENABLE_DEBUG_LOG
#define P4_ENABLE_DEBUG_LOG 0
#endif

#if P4_ENABLE_DEBUG_LOG
#define P4_LOG_PRINT(...)   Serial.print(__VA_ARGS__)
#define P4_LOG_PRINTLN(...) Serial.println(__VA_ARGS__)
#define P4_LOG_PRINTF(...)  Serial.printf(__VA_ARGS__)
#else
#define P4_LOG_PRINT(...)   ((void)0)
#define P4_LOG_PRINTLN(...) ((void)0)
#define P4_LOG_PRINTF(...)  ((void)0)
#endif

#ifndef P4_ENABLE_FX_SYNC_LOG
#define P4_ENABLE_FX_SYNC_LOG 0
#endif

#if P4_ENABLE_FX_SYNC_LOG
#define P4_FX_LOG_PRINTF(...) Serial.printf(__VA_ARGS__)
#else
#define P4_FX_LOG_PRINTF(...) ((void)0)
#endif

// Periodic Serial report of render FPS, LVGL task rate, touch poll rate and
// free heap (internal/PSRAM) — every PERF_LOG_INTERVAL_MS. Zero-cost when
// disabled (counters + prints compile out entirely). Enable with
// -DP4_ENABLE_PERF_LOG=1 to get real numbers before tuning anything else.
#ifndef P4_ENABLE_PERF_LOG
#define P4_ENABLE_PERF_LOG 0
#endif

// =============================================================================
// DISPLAY — Guition JC1060P470C (7" MIPI-DSI, JD9165BA)
// =============================================================================
#ifndef LCD_H_RES
#define LCD_H_RES   1024
#endif
#ifndef LCD_V_RES
#define LCD_V_RES   600
#endif

// Native panel mode: Guition P4 is landscape 1024x600.
// Keep this at 0 unless the display port implements explicit LVGL rotation.
#ifndef PORTRAIT_MODE
#define PORTRAIT_MODE 0
#endif

#if PORTRAIT_MODE
#define UI_W  600
#define UI_H  1024
#else
#define UI_W  LCD_H_RES
#define UI_H  LCD_V_RES
#endif

// MIPI-DSI configuration
#ifndef MIPI_DSI_LANES
#define MIPI_DSI_LANES          2
#endif
#ifndef MIPI_DSI_LANE_BITRATE_MBPS
#define MIPI_DSI_LANE_BITRATE_MBPS 550
#endif

// JD9165BA timing (1024×600 @ 60Hz)
#define LCD_HSYNC_PULSE     24
#define LCD_HSYNC_BACK      136
#define LCD_HSYNC_FRONT     160
#define LCD_VSYNC_PULSE     2
#define LCD_VSYNC_BACK      21
#define LCD_VSYNC_FRONT     12

// GPIO pins
#ifndef LCD_BL_GPIO
#define LCD_BL_GPIO     23
#endif
#ifndef LCD_RST_GPIO
#define LCD_RST_GPIO    27
#endif

// =============================================================================
// TOUCH — GT911 (I2C)
// =============================================================================
#ifndef TOUCH_I2C_SDA
#define TOUCH_I2C_SDA   7
#endif
#ifndef TOUCH_I2C_SCL
#define TOUCH_I2C_SCL   8
#endif
#ifndef TOUCH_I2C_ADDR
#define TOUCH_I2C_ADDR  0x5D
#endif

// External controls use the second ESP32-P4 I2C controller. GPIO7/8 remain
// reserved for GT911; a faulty external transaction therefore cannot stall
// touch/rendering or destabilize the complete instrument.
#ifndef ROTARY_I2C_SDA
#define ROTARY_I2C_SDA  3
#endif
#ifndef ROTARY_I2C_SCL
#define ROTARY_I2C_SCL  4
#endif
#ifndef TOUCH_RST_GPIO
#define TOUCH_RST_GPIO  22
#endif
#ifndef TOUCH_INT_GPIO
#define TOUCH_INT_GPIO  21
#endif

#ifndef TOUCH_CAL_X_MIN
#define TOUCH_CAL_X_MIN  0
#endif
#ifndef TOUCH_CAL_X_MAX
#define TOUCH_CAL_X_MAX  (LCD_V_RES - 1)
#endif
#ifndef TOUCH_CAL_Y_MIN
#define TOUCH_CAL_Y_MIN  0
#endif
#ifndef TOUCH_CAL_Y_MAX
#define TOUCH_CAL_Y_MAX  (LCD_H_RES - 1)
#endif

// =============================================================================
// USB-C — P4 is the USB host; DaisyPod3 is the USB CDC device.
// This is the only connection between the two subprojects.
// =============================================================================
#define DAISYPOD_USB_VID 0x0483
#define DAISYPOD_USB_PID 0x5740

// =============================================================================
// SEQUENCER (mirror of S3 constants for UI rendering)
// =============================================================================
namespace Config {
    constexpr int MAX_STEPS     = 16;
    constexpr int MAX_TRACKS    = 16;
    constexpr int TRACKS_PER_PAGE = 8;
    // The P4 UI keeps 128 local slots; DaisyPod3 maps the active program bank.
    constexpr int MAX_PATTERNS  = 128;
    constexpr int MAX_SAMPLES   = 16;
    constexpr int MAX_VOLUME    = 150;
    constexpr int DEFAULT_BPM   = 120;

    // UI timing. Updates run inside the LVGL task, not from Core1 loop(), so
    // 60 Hz state refresh runs in the dedicated LVGL task.
    constexpr uint32_t SCREEN_UPDATE_MS = 16;
    constexpr uint32_t HEARTBEAT_TIMEOUT_MS = 3000;
}
