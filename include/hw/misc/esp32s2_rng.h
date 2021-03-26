#pragma once

#include "hw/hw.h"
#include "hw/sysbus.h"
#include "hw/misc/esp32_reg.h"


#define TYPE_ESP32S2_RNG "misc.esp32s2.rng"
#define ESP32S2_RNG(obj) OBJECT_CHECK(Esp32S2RngState, (obj), TYPE_ESP32S2_RNG)

typedef struct Esp32S2RngState {
    SysBusDevice parent_obj;
    MemoryRegion iomem;
} Esp32S2RngState;

#define ESP32_RNG_BASE (DR_REG_WDEV_BASE + 0x144)

