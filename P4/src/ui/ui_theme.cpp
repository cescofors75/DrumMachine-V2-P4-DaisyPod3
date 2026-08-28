// =============================================================================
// ui_theme.cpp — RED808 theme presets (mirror of S3)
// =============================================================================
#include "ui_theme.h"
#include "config.h"

VisualTheme currentTheme = THEME_OCEAN;

const ThemeColors theme_presets[THEME_COUNT] = {
    // THEME_RED808
    { .bg=0x090806, .panel=0x14110D, .surface=0x211B14, .border=0x3A3024,
      .text=0xF7EAD7, .text_dim=0xA99A86,
      .accent=0xE23D22, .accent2=0xF87925,
      .success=0xF7EAD7, .warning=0xF5BC31, .error=0xC9271B, .info=0xFF8A24, .cyan=0xF7EAD7,
      .track_colors={0xC9271B,0xD93421,0xE74428,0xF2552F,
                     0xD85A1A,0xE86820,0xF87925,0xFF8C2A,
                     0xC99522,0xE0AA2A,0xF5BC31,0xFFD052,
                     0xDCC9A8,0xEAD9BA,0xF7EAD7,0xFFF7E8},
      .name="RED808" },
    // THEME_OCEAN
    { .bg=0x0A1628, .panel=0x0F2035, .surface=0x162D4A, .border=0x1E3F66,
      .text=0xD4E4F7, .text_dim=0x7BA3CC,
      .accent=0x4A9EFF, .accent2=0x6BB3FF,
      .success=0x40C4AA, .warning=0xF0B860, .error=0xF07070, .info=0x4A9EFF, .cyan=0x40C4FF,
      .track_colors={0x1144CC,0x2255DD,0x3366EE,0x4477FF,0x1166DD,0x2277EE,0x3388FF,0x0055BB,
                     0x1177CC,0x2288DD,0x3399EE,0x44AAFF,0x0044AA,0x1155BB,0x2266CC,0x3377DD},
      .name="OCEAN" },
    // THEME_NEON
    { .bg=0x0A1A0A, .panel=0x0F260F, .surface=0x183318, .border=0x264D26,
      .text=0xD0F5D0, .text_dim=0x7BBF7B,
      .accent=0x39FF14, .accent2=0x66FF44,
      .success=0x39FF14, .warning=0xFFFF00, .error=0xFF3333, .info=0x00FFAA, .cyan=0x00FFCC,
      .track_colors={0x00CC00,0x11DD11,0x22EE22,0x33FF33,0x00BB44,0x11CC55,0x22DD66,0x00AA00,
                     0x33EE33,0x44FF44,0x00DD88,0x11EE99,0x00CC66,0x22DD77,0x33EE88,0x44FF99},
      .name="NEON" },
    // THEME_SUNSET
    { .bg=0x1A0A14, .panel=0x26101E, .surface=0x381828, .border=0x552244,
      .text=0xF5D0E0, .text_dim=0xD099B8,
      .accent=0xFF6B35, .accent2=0xFF8855,
      .success=0xFFAA33, .warning=0xFF6B35, .error=0xFF3366, .info=0xCC66FF, .cyan=0xFF66AA,
      .track_colors={0xFF4400,0xFF5511,0xFF6622,0xFF7733,0xFF8844,0xCC44FF,0xBB33EE,0xAA22DD,
                     0xFF3366,0xFF5588,0xFF77AA,0xDD55CC,0xFF9944,0xCC55EE,0xFF6655,0xEE44BB},
      .name="SUNSET" },
    // THEME_RAINBOW
    { .bg=0x0D0D18, .panel=0x141428, .surface=0x1E1E38, .border=0x3A2A5A,
      .text=0xF0F0FF, .text_dim=0x9999CC,
      .accent=0xFF00AA, .accent2=0xAA00FF,
      .success=0x00FF66, .warning=0xFFCC00, .error=0xFF2255, .info=0x00CCFF, .cyan=0x00FFDD,
      .track_colors={0xFF0000,0xFF5500,0xFFAA00,0xFFFF00,0x88FF00,0x00FF00,0x00FF88,0x00FFFF,
                     0x0088FF,0x0000FF,0x5500FF,0xAA00FF,0xFF00FF,0xFF0088,0xFF4444,0x44FFAA},
      .name="RAINBOW" },
    // THEME_GREYSCALE — warning/error luminance lifted so accent-colored
    // outline buttons (border + label over the dark surface) stay legible.
    { .bg=0x101010, .panel=0x1A1A1A, .surface=0x252525, .border=0x404040,
      .text=0xE0E0E0, .text_dim=0x808080,
      .accent=0xCCCCCC, .accent2=0xAAAAAA,
      .success=0xBBBBBB, .warning=0xC8C8C8, .error=0xA0A0A0, .info=0xDDDDDD, .cyan=0xC0C0C0,
      .track_colors={0xFFFFFF,0xEEEEEE,0xDDDDDD,0xCCCCCC,0xBBBBBB,0xAAAAAA,0x999999,0x888888,
                     0x777777,0x666666,0x555555,0x444444,0x333333,0x222222,0xF0F0F0,0xD0D0D0},
      .name="GREYSCALE" },
    // THEME_MPD218 — matte black chassis, AKAI-red pad borders, white
    // labels: the AKAI MPD218 controller's own look. Deliberately just
    // black/red/white/grey (no orange or gold, unlike THEME_RED808) so it
    // reads as a distinct hardware skin rather than a RED808 variant.
    // success/error/warning lean on brightness (white=good, dark red=bad)
    // instead of hue, the same trick THEME_GREYSCALE uses, since red is
    // the only hue in the palette and can't do double duty as both the
    // brand accent and an error color without losing the distinction.
    { .bg=0x0A0A0A, .panel=0x141414, .surface=0x1E1A1A, .border=0x3D1418,
      .text=0xFFFFFF, .text_dim=0x9A9A9A,
      .accent=0xE2222C, .accent2=0xFF4D4D,
      .success=0xFFFFFF, .warning=0xFF8A3D, .error=0xB0121C, .info=0xE6E6E6, .cyan=0xFFFFFF,
      .track_colors={0x8C0F16,0xA31620,0xBB1E29,0xD22532,
                     0xE2222C,0xEB3B44,0xF25560,0xFA6E77,
                     0xFF8A90,0xFFA8AC,0xFFC6C8,0xFFE0E1,
                     0xFFFFFF,0xF0F0F0,0xD8D8D8,0xC0C0C0},
      .name="MPD218" },
};

uint8_t ui_theme_index() {
  const uint8_t index = static_cast<uint8_t>(currentTheme);
  if (index < THEME_COUNT) return index;
  P4_THEME_LOG_PRINTF("[THEME] invalid current=%u; repairing to OCEAN\n",
            static_cast<unsigned>(index));
  currentTheme = THEME_OCEAN;
  return static_cast<uint8_t>(currentTheme);
}

// Theme accessors
lv_color_t theme_bg()       { return lv_color_hex(theme_presets[ui_theme_index()].bg); }
lv_color_t theme_panel()    { return lv_color_hex(theme_presets[ui_theme_index()].panel); }
lv_color_t theme_surface()  { return lv_color_hex(theme_presets[ui_theme_index()].surface); }
lv_color_t theme_border()   { return lv_color_hex(theme_presets[ui_theme_index()].border); }
lv_color_t theme_text()     { return lv_color_hex(theme_presets[ui_theme_index()].text); }
lv_color_t theme_text_dim() { return lv_color_hex(theme_presets[ui_theme_index()].text_dim); }
lv_color_t theme_accent()   { return lv_color_hex(theme_presets[ui_theme_index()].accent); }
lv_color_t theme_accent2()  { return lv_color_hex(theme_presets[ui_theme_index()].accent2); }
lv_color_t theme_success()  { return lv_color_hex(theme_presets[ui_theme_index()].success); }
lv_color_t theme_warning()  { return lv_color_hex(theme_presets[ui_theme_index()].warning); }
lv_color_t theme_error()    { return lv_color_hex(theme_presets[ui_theme_index()].error); }
lv_color_t theme_info()     { return lv_color_hex(theme_presets[ui_theme_index()].info); }
lv_color_t theme_cyan()     { return lv_color_hex(theme_presets[ui_theme_index()].cyan); }

void ui_theme_apply(VisualTheme theme) {
  const uint8_t requested = static_cast<uint8_t>(theme);
  if (requested >= THEME_COUNT) {
    P4_THEME_LOG_PRINTF("[THEME] rejected apply=%u current=%u\n",
              static_cast<unsigned>(requested),
              static_cast<unsigned>(ui_theme_index()));
    return;
  }
  const uint8_t previous = ui_theme_index();
    currentTheme = theme;
  P4_THEME_LOG_PRINTF("[THEME] apply %u->%u (%s)\n",
            static_cast<unsigned>(previous),
            static_cast<unsigned>(requested),
            theme_presets[requested].name);
}
