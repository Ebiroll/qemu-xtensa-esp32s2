#pragma once

#include "hw/sysbus.h"
#include "hw/hw.h"
#include "hw/registerfields.h"

#define TYPE_ESP32S2_GPIO "esp32s2.gpio"
#define ESP32_GPIO(obj) OBJECT_CHECK(Esp32s2GpioState, (obj), TYPE_ESP32S2_GPIO)

REG32(GPIO_STRAP, 0x0038)
// 0x12
#define ESP32S2_STRAP_MODE_FLASH_BOOT 0x0a
#define ESP32S2_STRAP_MODE_UART_BOOT  0x0f

typedef struct Esp32s2GpioState {
    SysBusDevice parent_obj;

    MemoryRegion iomem;
    qemu_irq irq;
    uint32_t strap_mode;
} Esp32s2GpioState;

