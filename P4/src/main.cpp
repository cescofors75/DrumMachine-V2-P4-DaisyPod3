// =============================================================================
// DrumMachine V2 — P4 controller
// ESP32-P4 + MIPI-DSI 7" display + USB host to DaisyPod3.
// There is no ESP32-S3 and no network or serial control transport.
// =============================================================================

#include <Arduino.h>
#include <SPIFFS.h>
#include <esp_task_wdt.h>
#include "../include/config.h"
#include "drivers/display_init.h"
#include "drivers/lvgl_port.h"
#include "control_api.h"
#include "ui/ui_screens.h"
#include "ui/ui_theme.h"
#include "dsp_task.h"
#include "settings_store.h"

#include "usb_cdc_handler.h"

void setup() {
    // 1. Debug serial (only waits if debug logging is enabled)
    Serial.begin(115200);
#if P4_ENABLE_DEBUG_LOG
    delay(500);
    Serial.println("\n=== DrumMachine V2 — P4 ===");
    Serial.println("Guition JC1060P470C | USB-C Host -> DaisyPod3");
#elif P4_ENABLE_FX_SYNC_LOG
    delay(500);
    Serial.println("\n[FX][DBG] P4 FX sync log enabled @115200");
    Serial.println("[FX][DBG] DaisyPod3 USB-C control trace enabled");
#endif

    // 1b. Task watchdog: the prebuilt Arduino core ships with the IDF
    // defaults baked in (5 s timeout + panic reset) — sdkconfig.defaults is
    // NOT applied when framework = arduino uses precompiled libs. Reconfigure
    // at runtime instead: 30 s absorbs transient bursts (screen transitions,
    // MIDI load) and no panic keeps a genuine stall from hard-resetting the
    // instrument mid-session (it logs the culprit task instead).
    {
        esp_task_wdt_config_t wdt_cfg = {};
        wdt_cfg.timeout_ms = 30000;
        wdt_cfg.idle_core_mask = (1 << 0);   // keep watching Core 0 idle task
        wdt_cfg.trigger_panic = false;
        esp_task_wdt_reconfigure(&wdt_cfg);
    }

    // 2. Initialize MIPI-DSI display + backlight
    P4_LOG_PRINTLN("[INIT] Display...");
    display_init();
    P4_LOG_PRINTLN("[INIT] Display OK");

    // 3. Initialize LVGL (display driver + GT911 touch)
    P4_LOG_PRINTLN("[INIT] LVGL port...");
    lvgl_port_init();
    P4_LOG_PRINTLN("[INIT] LVGL OK");

    // 4. Load persisted settings (NVS) and apply the saved theme
    settings_load();
    ui_theme_apply((VisualTheme)settings_boot_theme());

    // 5. Mount SPIFFS BEFORE creating screens: create_live_screen() restores
    // the persisted XTRA pad state from SPIFFS, so the FS must be up first.
    // Also used for MEM MIDI storage (/mid/*.mid). Non-fatal if absent.
    P4_LOG_PRINTLN("[INIT] Mounting SPIFFS...");
    if (SPIFFS.begin(false, "/spiffs", 10)) {
#if P4_ENABLE_DEBUG_LOG
        Serial.printf("[INIT] SPIFFS OK — %u / %u bytes used\n",
                      (unsigned)SPIFFS.usedBytes(), (unsigned)SPIFFS.totalBytes());
#endif
    } else {
        P4_LOG_PRINTLN("[INIT] SPIFFS mount failed (run uploadfs once)");
    }

    // 5b. Create UI screens (boot → live → seq → fx → vol → settings → perf)
    P4_LOG_PRINTLN("[INIT] UI screens...");
    ui_create_all_screens();
    P4_LOG_PRINTLN("[INIT] UI screens OK");

    // 6. Initialize the only inter-board link: P4 USB host to DaisyPod3 CDC.
    P4_LOG_PRINTLN("[INIT] USB-C Host for DaisyPod3...");
    usb_cdc_init();
    control_init();

    // 7. Restore the P4 settings. control_process() pushes the resulting state
    // to DaisyPod3 as soon as the USB command handshake becomes active.
    settings_apply();

    // 9. Start DSP processing task (Core 0)
    P4_LOG_PRINTLN("[INIT] DSP task...");
    dsp_task_init();

    P4_LOG_PRINTLN("=== P4 Ready — Waiting for DaisyPod3 over USB-C ===");

    // 10. Start LVGL rendering task (must be AFTER UI creation)
    lvgl_port_task_start();
}

void loop() {
    // Enumerate USB first, then exchange the binary P4/Daisy command stream.
    usb_cdc_process();
    control_process();

    // Drain pad event queue FIRST — lowest latency pad→Master path (Core 1, no mutex)
    ui_process_pad_queue();

    // Send deferred UI control commands outside the LVGL task so button
    // feedback paints immediately while sample transfers are in progress.
    ui_process_control_queue();

    // Debounced NVS persistence of theme/volumes/BPM/pattern.
    settings_tick();

    // LVGL screen updates and rendering are handled by the dedicated LVGL task.
}
