/*
 * ESP32 eFuse emulation
 *
 * Copyright (c) 2019 Espressif Systems (Shanghai) Co. Ltd.
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 or
 * (at your option) any later version.
 */

#include "qemu/osdep.h"
#include "qemu/log.h"
#include "qemu/module.h"
#include "qapi/error.h"
#include "qemu/error-report.h"
#include "sysemu/sysemu.h"
#include "chardev/char-fe.h"
#include "hw/registerfields.h"
#include "hw/sysbus.h"
#include "hw/irq.h"
#include "hw/qdev-properties.h"
#include "hw/nvram/esp32s2_efuse.h"

static void esp32s2_efuse_read_op(Esp32S2EfuseState *s);
static void esp32s2_efuse_program_op(Esp32S2EfuseState *s);
static void esp32s2_efuse_update_irq(Esp32S2EfuseState *s);
static void esp32s2_efuse_op_timer_start(Esp32S2EfuseState *s);

static uint64_t esp32s2_efuse_read(void *opaque, hwaddr addr, unsigned int size)
{
    Esp32S2EfuseState *s = ESP32S2_EFUSE(opaque);
    uint64_t r = 0;
    int idx;
    switch (addr) {
    case A_EFUSE_BLK0_RDATA0 ... A_EFUSE_BLK1_RDATA0 - 4:
        idx = (addr - A_EFUSE_BLK0_RDATA0) / 4;
        r = s->efuse_rd.blk0[idx] & ~(s->efuse_rd_dis.blk0[idx]);
        break;
//    case A_EFUSE_BLK0_WDATA0 ... A_EFUSE_BLK1_RDATA0 - 4:
//        r = s->efuse_wr.blk0[(addr - A_EFUSE_BLK0_WDATA0) / 4];
//        break;
    case A_EFUSE_BLK1_RDATA0 ... A_EFUSE_BLK2_RDATA0 - 4:
        idx = (addr - A_EFUSE_BLK1_RDATA0) / 4;
        r = s->efuse_rd.blk1[idx] & ~(s->efuse_rd_dis.blk1[idx]);
        break;
    case A_EFUSE_BLK2_RDATA0 ... A_EFUSE_BLK3_RDATA0 - 4:
        idx = (addr - A_EFUSE_BLK2_RDATA0) / 4;
        r = s->efuse_rd.blk2[idx] & ~(s->efuse_rd_dis.blk2[idx]);
        break;
    case A_EFUSE_BLK3_RDATA0 ... A_EFUSE_BLK4_RDATA0 - 4:
        idx = (addr - A_EFUSE_BLK3_RDATA0) / 4;
        r = s->efuse_rd.blk3[idx] & ~(s->efuse_rd_dis.blk3[idx]);
        break;
//    case A_EFUSE_BLK1_WDATA0 ... A_EFUSE_BLK2_WDATA0 - 4:
//        r = s->efuse_wr.blk1[(addr - A_EFUSE_BLK1_WDATA0) / 4];
//        break;
//    case A_EFUSE_BLK2_WDATA0 ... A_EFUSE_BLK3_WDATA0 - 4:
//        r = s->efuse_wr.blk2[(addr - A_EFUSE_BLK2_WDATA0) / 4];
//        break;
//    case A_EFUSE_BLK3_WDATA0 ... A_EFUSE_CLK - 4:
//        r = s->efuse_wr.blk3[(addr - A_EFUSE_BLK3_WDATA0) / 4];
//        break;
    case A_EFUSE_CLK:
        r = s->clk_reg;
        break;
    case A_EFUSE_CONF:
        r = s->conf_reg;
        break;
    case A_EFUSE_CMD:
        r = s->cmd_reg;
        break;
    case A_EFUSE_STATUS:
        r = 0;
        break;
    case A_EFUSE_DAC_CONF:
        r = s->dac_conf_reg;
        break;
    case A_EFUSE_INT_RAW:
        r = s->int_raw_reg;
        break;
    case A_EFUSE_INT_ST:
        r = s->int_st_reg;
        break;
    case A_EFUSE_INT_ENA:
        r = s->int_ena_reg;
        break;
//    case A_EFUSE_DEC_STATUS:
//       r = 0;
//        break;
    case A_EFUSE_DATE:
        r = 0x16042600;
    }

    printf("efuse read  %08X,%08X\n",(unsigned int)addr,(unsigned int)r);

    return r;
}

static void esp32s2_efuse_write(void *opaque, hwaddr addr,
                       uint64_t value, unsigned int size)
{
    Esp32S2EfuseState *s = ESP32S2_EFUSE(opaque);
    switch (addr) {
    case A_EFUSE_CLK:
        s->clk_reg = value;
        break;
    case A_EFUSE_CONF:
        s->conf_reg = value;
        break;
    case A_EFUSE_CMD:
        if ((value & EFUSE_READ) && s->conf_reg == EFUSE_READ_OP_CODE) {
            esp32s2_efuse_read_op(s);
        }
        if ((value & EFUSE_PGM) && s->conf_reg == EFUSE_PGM_OP_CODE) {
            esp32s2_efuse_program_op(s);
        }
        break;
    case A_EFUSE_DAC_CONF:
        s->dac_conf_reg = value;
        break;
    case A_EFUSE_INT_ENA:
        s->int_ena_reg = value;
        esp32s2_efuse_update_irq(s);
        break;
    case A_EFUSE_INT_CLR:
        s->int_raw_reg &= ~value;
        esp32s2_efuse_update_irq(s);
        break;
//    case A_EFUSE_BLK0_WDATA0 ... A_EFUSE_BLK1_RDATA0 - 4:
//        s->efuse_wr.blk0[(addr - A_EFUSE_BLK0_WDATA0) / 4] = value;
//        break;
//    case A_EFUSE_BLK1_WDATA0 ... A_EFUSE_BLK2_WDATA0 - 4:
//        s->efuse_wr.blk1[(addr - A_EFUSE_BLK1_WDATA0) / 4] = value;
//        break;
//    case A_EFUSE_BLK2_WDATA0 ... A_EFUSE_BLK3_WDATA0 - 4:
//        s->efuse_wr.blk2[(addr - A_EFUSE_BLK2_WDATA0) / 4] = value;
//        break;
//    case A_EFUSE_BLK3_WDATA0 ... A_EFUSE_CLK - 4:
//        s->efuse_wr.blk3[(addr - A_EFUSE_BLK3_WDATA0) / 4] = value;
//        break;
    }

       printf("efuse write  %08X,%08X\n",(unsigned int)addr,(unsigned int)value);
}

#define APPLY_DIS(rdwr_, ctrl_field_, dest_field_) \
    if (s->efuse_ ## rdwr_ .blk0_d0.ctrl_field_) { \
        memset(&s->efuse_ ## rdwr_ ## _dis.dest_field_, 0xff, sizeof(s->efuse_ ## rdwr_ ## _dis.dest_field_)); \
    }

#define APPLY_DIS_FIELD(rdwr_, ctrl_field_, dest_field_) \
    if (s->efuse_ ## rdwr_ .blk0_d0.ctrl_field_) { \
        s->efuse_ ## rdwr_ ## _dis.dest_field_ = 0; \
        s->efuse_ ## rdwr_ ## _dis.dest_field_ -= 1; \
    }


static void esp32s2_efuse_read_op(Esp32S2EfuseState *s)
{
    s->cmd_reg = EFUSE_READ;
    if (s->blk != NULL) {
        uint64_t perm = BLK_PERM_CONSISTENT_READ |
                                (blk_is_read_only(s->blk) ? 0 : BLK_PERM_WRITE);
        int ret = blk_set_perm(s->blk, perm, BLK_PERM_ALL, NULL);
        if (ret != 0) {
            fprintf(stderr, "%s: failed to set permission (%d)\n", __func__, ret);
        }
        ret = blk_pread(s->blk, 0, &s->efuse_rd, sizeof(s->efuse_rd));
        if (ret != sizeof(s->efuse_rd)) {
            fprintf(stderr, "%s: failed to read the block device (%d)\n", __func__, ret);
        }
    }

    memset(&s->efuse_rd_dis, 0, sizeof(s->efuse_rd_dis));
    memset(&s->efuse_wr_dis, 0, sizeof(s->efuse_wr_dis));

    APPLY_DIS(rd, rd_dis_blk1, blk1);
    APPLY_DIS(rd, rd_dis_blk2, blk2);
    APPLY_DIS(rd, rd_dis_blk3, blk3);
    APPLY_DIS_FIELD(rd, rd_dis_blk0_partial, blk0_d5.flash_crypt_config);
    APPLY_DIS_FIELD(rd, rd_dis_blk0_partial, blk0_d6.coding_scheme);
    APPLY_DIS_FIELD(rd, rd_dis_blk0_partial, blk0_d6.key_status);

    APPLY_DIS(wr, wr_dis_blk1, blk1);
    APPLY_DIS(wr, wr_dis_blk2, blk2);
    APPLY_DIS(wr, wr_dis_blk3, blk3);

    /* Other wr_dis bits are not emulated, but can be handled here if necessary */

    esp32s2_efuse_op_timer_start(s);
}

static void esp32s2_efuse_program_op(Esp32S2EfuseState *s)
{
    s->cmd_reg = EFUSE_PGM;

    Esp32EfuseRegs result;
    uint32_t* dst = (uint32_t*) &result;
    uint32_t* rd = (uint32_t*) &s->efuse_rd;
    uint32_t* wr = (uint32_t*) &s->efuse_wr;
    uint32_t* wr_dis = (uint32_t*) &s->efuse_wr_dis;
    for (int i = 0; i < sizeof(result) / sizeof(uint32_t); ++i) {
        uint32_t wr_word = wr[i];
        uint32_t wr_dis_word = wr_dis[i];
        uint32_t rd_word = rd[i];
        dst[i] = (wr_word & (~wr_dis_word)) | (rd_word & wr_dis_word);
    }

    if (s->blk != NULL) {
        int ret = blk_pwrite(s->blk, 0, &result, sizeof(result), 0);
        if (ret != sizeof(result)) {
            fprintf(stderr, "%s: failed to write to block device (%d)\n", __func__, ret);
        }
    }

    esp32s2_efuse_op_timer_start(s);
}

static void esp32s2_efuse_update_irq(Esp32S2EfuseState *s)
{
    s->int_st_reg = s->int_ena_reg & s->int_raw_reg;
    int level = s->int_st_reg != 0;
    qemu_set_irq(s->irq, level);
}

static void esp32s2_efuse_op_timer_start(Esp32S2EfuseState *s)
{
    uint64_t ns_now = qemu_clock_get_ns(QEMU_CLOCK_VIRTUAL);
    uint64_t interval_ns = 100000000; /* 10 ms, make this depend on EFUSE_CLK register */
    timer_mod_anticipate_ns(&s->op_timer, ns_now + interval_ns);
}

static void esp32s2_efuse_timer_cb(void *opaque)
{
    Esp32S2EfuseState *s = ESP32S2_EFUSE(opaque);
    uint32_t cmd = s->cmd_reg;
    s->cmd_reg = 0;
    s->int_raw_reg |= cmd;
    esp32s2_efuse_update_irq(s);
}

static const MemoryRegionOps esp32s2_efuse_ops = {
    .read =  esp32s2_efuse_read,
    .write = esp32s2_efuse_write,
    .endianness = DEVICE_LITTLE_ENDIAN,
};

static void esp32s2_efuse_reset(DeviceState *dev)
{
    Esp32S2EfuseState *s = ESP32S2_EFUSE(dev);
    esp32s2_efuse_read_op(s);
}

static void esp32s2_efuse_realize(DeviceState *dev, Error **errp)
{
}

static void esp32s2_efuse_init(Object *obj)
{
    Esp32S2EfuseState *s = ESP32S2_EFUSE(obj);
    SysBusDevice *sbd = SYS_BUS_DEVICE(obj);

    memory_region_init_io(&s->iomem, obj, &esp32s2_efuse_ops, s,
                          TYPE_ESP32S2_EFUSE, A_EFUSE_DATE + 4);
    sysbus_init_mmio(sbd, &s->iomem);
    sysbus_init_irq(sbd, &s->irq);

    timer_init_ns(&s->op_timer, QEMU_CLOCK_VIRTUAL, esp32s2_efuse_timer_cb, s);

    memset(&s->efuse_rd, 0, sizeof(s->efuse_rd));
    memset(&s->efuse_wr, 0, sizeof(s->efuse_wr));
}

static Property esp32s2_efuse_properties[] = {
    DEFINE_PROP_DRIVE("drive", Esp32S2EfuseState, blk),
    DEFINE_PROP_END_OF_LIST(),
};

static void esp32s2_efuse_class_init(ObjectClass *klass, void *data)
{
    DeviceClass *dc = DEVICE_CLASS(klass);

    dc->reset = esp32s2_efuse_reset;
    dc->realize = esp32s2_efuse_realize;
    device_class_set_props(dc, esp32s2_efuse_properties);
}

static const TypeInfo esp32s2_efuse_info = {
    .name = TYPE_ESP32S2_EFUSE,
    .parent = TYPE_SYS_BUS_DEVICE,
    .instance_size = sizeof(Esp32S2EfuseState),
    .instance_init = esp32s2_efuse_init,
    .class_init = esp32s2_efuse_class_init
};

static void esp32s2_efuse_register_types(void)
{
    type_register_static(&esp32s2_efuse_info);
}

type_init(esp32s2_efuse_register_types)
