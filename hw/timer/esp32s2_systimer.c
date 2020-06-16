/*
 * ESP32S2 Systimer emulation
 *
 * Copyright (c) 2020 Espressif Systems (Shanghai) Co. Ltd.
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 or
 * (at your option) any later version.
 */

#include "qemu/osdep.h"
#include "qemu/log.h"
#include "qemu/error-report.h"
#include "qapi/error.h"
#include "hw/hw.h"
#include "hw/sysbus.h"
#include "hw/registerfields.h"
#include "hw/irq.h"
#include "hw/qdev-properties.h"
#include "hw/timer/esp32s2_systimer.h"



static uint64_t esp32s2_systimer_read(void *opaque, hwaddr addr, unsigned int size)
{
    //Esp32S2SysTimerState *s = ESP32_SYSTIMER(opaque);
    uint64_t r = 0;
    switch (addr) {
    case A_SYSTIMER_CONF:
        r = 0;
        break;

    case A_SYSTIMER_UPDATE:
        r=BIT(30);
        break;


    default:
        break;
    }
    printf("systimer read  %08X,%08X\n",(unsigned int)addr,(unsigned int)r);

    return r;
}

static void esp32s2_systimer_write(void *opaque, hwaddr addr,
                       uint64_t value, unsigned int size)
{
    printf("systimer write  %08X,%08X\n",(unsigned int)addr,(unsigned int)value);

}

static const MemoryRegionOps uart_ops = {
    .read =  esp32s2_systimer_read,
    .write = esp32s2_systimer_write,
    .endianness = DEVICE_LITTLE_ENDIAN,
};

static void esp32s2_systimer_reset(DeviceState *dev)
{
}

static void esp32s2_systimer_realize(DeviceState *dev, Error **errp)
{
}

static void esp32s2_systimer_init(Object *obj)
{
    Esp32S2SysTimerState *s = ESP32_SYSTIMER(obj);
    SysBusDevice *sbd = SYS_BUS_DEVICE(obj);

    memory_region_init_io(&s->iomem, obj, &uart_ops, s,
                          TYPE_ESP32S2_SYSTIMER, 0x100);
    sysbus_init_mmio(sbd, &s->iomem);
    sysbus_init_irq(sbd, &s->irq);
}

static Property esp32s2_systimer_properties[] = {
    DEFINE_PROP_UINT32("systimer_mode", Esp32S2SysTimerState, systimer_mode, 0),
    DEFINE_PROP_END_OF_LIST(),
};

static void esp32s2_systimer_class_init(ObjectClass *klass, void *data)
{
    DeviceClass *dc = DEVICE_CLASS(klass);

    dc->reset = esp32s2_systimer_reset;
    dc->realize = esp32s2_systimer_realize;
    device_class_set_props(dc, esp32s2_systimer_properties);
}

static const TypeInfo esp32s2_systimer_info = {
    .name = TYPE_ESP32S2_SYSTIMER,
    .parent = TYPE_SYS_BUS_DEVICE,
    .instance_size = sizeof(Esp32S2SysTimerState),
    .instance_init = esp32s2_systimer_init,
    .class_init = esp32s2_systimer_class_init
};

static void esp32s2_systimer_register_types(void)
{
    type_register_static(&esp32s2_systimer_info);
}

type_init(esp32s2_systimer_register_types)
