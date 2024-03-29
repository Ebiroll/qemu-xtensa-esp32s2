/*
 * ESP32 Interrupt Matrix
 *
 * Copyright (c) 2019 Espressif Systems (Shanghai) Co. Ltd.
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 or
 * (at your option) any later version.
 */

#pragma once

#include "qemu/osdep.h"
#include "qemu/log.h"
#include "qemu/error-report.h"
#include "qapi/error.h"
#include "hw/hw.h"
#include "hw/sysbus.h"
#include "hw/irq.h"
#include "hw/qdev-properties.h"
#include "target/xtensa/cpu.h"
#include "target/xtensa/cpu-qom.h"

#define ESP32S2_CPU_COUNT 1
#define ESP32S2_INT_MATRIX_INPUTS 69

#define TYPE_ESP32S2_INTMATRIX "misc.esp32S2.intmatrix"
#define ESP32S2_INTMATRIX(obj) OBJECT_CHECK(Esp32S2IntMatrixState, (obj), TYPE_ESP32_INTMATRIX)


typedef struct Esp32S2IntMatrixState {
    SysBusDevice parent_obj;

    MemoryRegion iomem;
    qemu_irq *outputs[ESP32S2_CPU_COUNT];
    uint8_t irq_map[ESP32S2_CPU_COUNT][ESP32S2_INT_MATRIX_INPUTS];

    /* properties */
    XtensaCPU *cpu[ESP32S2_CPU_COUNT];
} Esp32IntMatrixState;

