# DrumMachine V2 — DaisyPod3
# RED808 audio engine + native Daisy Pod controls + USB CDC device.

TARGET = DrumMachineV2_DaisyPod3
CPP_SOURCES = main.cpp syscalls.cpp

OPT = -O3
APP_TYPE = BOOT_SRAM

# Dependencies are pinned as repository submodules.
# Override these variables to point at another libDaisy/DaisySP checkout.
LIBDAISY_DIR ?= ../third_party/libDaisy
DAISYSP_DIR  ?= ../third_party/DaisySP

C_INCLUDES += -I.
C_INCLUDES += -I$(DAISYSP_DIR)/DaisySP-LGPL/Source

LIBS += -ldaisysp-lgpl
LIBDIR += -L$(DAISYSP_DIR)/DaisySP-LGPL/build
LDFLAGS += -Wl,--no-warn-rwx-segments
LDFLAGS += -Wl,--allow-multiple-definition

# USB-C CDC is the only control transport in DaisyPod3.
C_DEFS += -DRED808_BOOT_DIAG_MINIMAL=0
C_DEFS += -DRED808_AUDIO_DIAG_MINIMAL=0
C_DEFS += -DRED808_BOOT_PROGRESS_DIAG=0
C_DEFS += -DRED808_ENABLE_INIT_FX=1
C_DEFS += -DRED808_STARTUP_TONE_TEST=0
C_DEFS += -DRED808_STARTUP_808_SELF_TEST=0
C_DEFS += -DRED808_STARTUP_SHOWCASE_DEMO=0
C_DEFS += -DRED808_STARTUP_TONE_SECONDS=3
C_DEFS += -DRED808_STARTUP_STRESS_REPORT=0
C_DEFS += -DRED808_STARTUP_STRESS_SECONDS=18
C_DEFS += -DRED808_DSP_BLOCK_PROFILE=1
C_DEFS += -DRED808_MEM_AUDIT=0
C_DEFS += -DRED808_ENABLE_SD_BOOTLOAD=1

SYSTEM_FILES_DIR = $(LIBDAISY_DIR)/core
include $(SYSTEM_FILES_DIR)/Makefile
