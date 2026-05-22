#pragma once

// Emulate ESP-IDF sdkconfig for Bluepad32 when building with Pico-SDK.
#define CONFIG_BLUEPAD32_MAX_DEVICES 4
#define CONFIG_BLUEPAD32_MAX_ALLOWLIST 4
#define CONFIG_BLUEPAD32_GAP_SECURITY 1
#define CONFIG_BLUEPAD32_ENABLE_BLE_BY_DEFAULT 1

// Use your own custom platform file: lib/Controller/xbox_bluepad32_platform.c
#define CONFIG_BLUEPAD32_PLATFORM_CUSTOM 1

// Pico 2 W is still part of Bluepad32's Pico W platform family.
#define CONFIG_TARGET_PICO_W 1

// 0=None, 1=Error, 2=Info, 3=Debug, 4=Verbose
#define CONFIG_BLUEPAD32_LOG_LEVEL 2
