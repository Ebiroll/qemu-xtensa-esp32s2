/*
 * ESP32 SoC and machine
 *
 * Copyright (c) 2019 Espressif Systems (Shanghai) Co. Ltd.
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 or
 * (at your option) any later version.
 */

#include "qemu/osdep.h"
#include "qemu/log.h"
#include "qemu/error-report.h"
#include "qapi/error.h"
#include "qemu-common.h"
#include "hw/hw.h"
#include "hw/boards.h"
#include "hw/loader.h"
#include "hw/sysbus.h"
#include "hw/i2c/esp32_i2c.h"
#include "hw/i2c/i2c.h"
#include "target/xtensa/cpu.h"
#include "hw/misc/esp32s2_reg.h"
#include "hw/char/esp32s2_uart.h"
#include "hw/gpio/esp32s2_gpio.h"
#include "hw/misc/esp32s2_dport.h"
#include "hw/misc/esp32_rtc_cntl.h"
#include "hw/misc/esp32_rng.h"
#include "hw/misc/esp32s2_sha.h"
#include "hw/timer/esp32_frc_timer.h"
#include "hw/timer/esp32_timg.h"
#include "hw/timer/esp32s2_systimer.h"
#include "hw/ssi/esp32s2_spi.h"
#include "hw/nvram/esp32s2_efuse.h"
#include "hw/xtensa/xtensa_memory.h"
#include "hw/misc/unimp.h"
#include "hw/irq.h"
#include "hw/qdev-properties.h"
#include "sysemu/sysemu.h"
#include "sysemu/reset.h"
#include "sysemu/cpus.h"
#include "sysemu/runstate.h"
#include "sysemu/blockdev.h"
#include "sysemu/block-backend.h"
#include "exec/exec-all.h"
#include "net/net.h"
#include "elf.h"
//#include "exec/memory.h"
#include "migration/vmstate.h"


// TODO, map flash when Cache_Ibus_MMU_Set is called

#define TYPE_ESP32S2_SOC "xtensa.esp32s2"
#define ESP32S2_SOC(obj) OBJECT_CHECK(Esp32S2SocState, (obj), TYPE_ESP32S2_SOC)

#define TYPE_ESP32S2_CPU XTENSA_CPU_TYPE_NAME("esp32s2")

typedef struct XtensaCPU XtensaCPU;


enum {
    ESP32S2_MEMREGION_IROM,
    ESP32S2_MEMREGION_DROM1,
    ESP32S2_MEMREGION_DROM,
    ESP32S2_MEMREGION_DRAM,
    ESP32S2_MEMREGION_IRAM,
    ESP32S2_MEMREGION_ICACHE0,
    ESP32S2_MEMREGION_ICACHE1,
    ESP32S2_MEMREGION_RTCSLOW,
    ESP32S2_MEMREGION_RTCFAST_D,
    ESP32S2_MEMREGION_RTCFAST_I,
};


// Overall memory map 
// drom0 low address for icache
#define SOC_DROM_LOW    0x3F000000 
// dram0 high address for dcache
#define SOC_DROM_HIGH   0x3FF80000 
#define SOC_IROM_LOW    0x40080000
#define SOC_IROM_HIGH   0x40800000

#define SOC_IROM_MASK_LOW  0x40000000
#define SOC_IROM_MASK_HIGH 0x4001A100

#define SOC_IRAM_LOW    0x40020000
#define SOC_IRAM_HIGH   0x40070000
#define SOC_DRAM_LOW    0x3FFB0000
#define SOC_DRAM_HIGH   0x40000000
#define SOC_RTC_IRAM_LOW  0x40070000
#define SOC_RTC_IRAM_HIGH 0x40072000
#define SOC_RTC_DRAM_LOW  0x3ff9e000
#define SOC_RTC_DRAM_HIGH 0x3ffa0000
#define SOC_RTC_DATA_LOW  0x50000000
#define SOC_RTC_DATA_HIGH 0x50002000
#define SOC_EXTRAM_DATA_LOW 0x3F500000
#define SOC_EXTRAM_DATA_HIGH 0x3FF80000






static const struct MemmapEntry {
    hwaddr base;
    hwaddr size;
} ESP32S2_memmap[] = {
    [ESP32S2_MEMREGION_DROM] = { SOC_DROM_LOW, SOC_DROM_HIGH-SOC_DROM_LOW},
    [ESP32S2_MEMREGION_IROM] = { 0x40000000, 0x80000 },
    [ESP32S2_MEMREGION_DROM1] = { 0x3ffa0000, 0x10000 },
    [ESP32S2_MEMREGION_DRAM] = {  0x3FFB0000 /*SOC_DRAM_LOW  */, 0x50000 /* 32K + 288KB */},
    [ESP32S2_MEMREGION_IRAM] = {  0x40020000 /* SOC_IRAM_LOW */,0x50000 /* 32K + 288KB */},
    [ESP32S2_MEMREGION_ICACHE0] = { SOC_DROM_LOW ,0xF80000 },  // 4MB, 0x8000 on esp32, emulation where cache is handled correctly
    [ESP32S2_MEMREGION_ICACHE1] = { SOC_IROM_LOW ,0xF80000 },  // 4MB, 0x8000 on esp32, emulation where cache is handled correctly
    [ESP32S2_MEMREGION_RTCSLOW] = { SOC_RTC_DATA_LOW, SOC_RTC_DATA_HIGH-SOC_RTC_DATA_LOW },
    [ESP32S2_MEMREGION_RTCFAST_I] = {SOC_RTC_IRAM_LOW, SOC_RTC_IRAM_HIGH-SOC_RTC_IRAM_LOW},
    [ESP32S2_MEMREGION_RTCFAST_D] = { SOC_RTC_DRAM_LOW, SOC_RTC_DRAM_HIGH-SOC_RTC_DRAM_LOW},
};


#define ESP32S2_SOC_RESET_PROCPU    0x1
#define ESP32S2_SOC_RESET_APPCPU    0x2
#define ESP32S2_SOC_RESET_PERIPH    0x4
#define ESP32S2_SOC_RESET_DIG       (ESP32S2_SOC_RESET_PROCPU | ESP32S2_SOC_RESET_APPCPU | ESP32S2_SOC_RESET_PERIPH)
#define ESP32S2_SOC_RESET_RTC       0x8
#define ESP32S2_SOC_RESET_ALL       (ESP32S2_SOC_RESET_RTC | ESP32S2_SOC_RESET_DIG)

#define TYPE_ESP32S2_MYUNIMP     "esp32s2.myunimp"


typedef struct Esp32UnimpState {
    SysBusDevice parent_obj;

    BlockBackend *flash_blk;
    uint32_t mmu_table[0x1000];
    uint32_t cache_value;
    MemoryRegion iomem;
    qemu_irq irq;
} Esp32UnimpState;


typedef struct Esp32S2SocState {
    /*< private >*/
    SysBusDevice parent_obj;

    /*< public >*/
    XtensaCPU cpu[ESP32S2_CPU_COUNT];
    Esp32DportState dport;
    Esp32IntMatrixState intmatrix;
    ESP32S2UARTState uart[ESP32S2_UART_COUNT];
    Esp32s2GpioState gpio;
    Esp32RngState rng;
    Esp32RtcCntlState rtc_cntl;
    Esp32FrcTimerState frc_timer[ESP32S2_FRC_COUNT];
    Esp32TimgState timg[ESP32S2_TIMG_COUNT];
    Esp32S2SysTimerState systimer;
    Esp32SpiState spi[ESP32S2_SPI_COUNT];
    Esp32I2CState i2c[ESP32_I2C_COUNT];
    Esp32S2ShaState sha;
    Esp32S2EfuseState efuse;
    Esp32UnimpState myunimp;
    DeviceState *eth;

    BusState rtc_bus;
    BusState periph_bus;

    MemoryRegion cpu_specific_mem[ESP32S2_CPU_COUNT];

    uint32_t requested_reset;
} Esp32S2SocState;

static void ESP32S2_remove_cpu_watchpoints(XtensaCPU* xcs)
{
    for (int i = 0; i < MAX_NDBREAK; ++i) {
        if (xcs->env.cpu_watchpoint[i]) {
            cpu_watchpoint_remove_by_ref(CPU(xcs), xcs->env.cpu_watchpoint[i]);
            xcs->env.cpu_watchpoint[i] = NULL;
        }
    }
}

static void ESP32S2_dig_reset(void *opaque, int n, int level)
{
    Esp32S2SocState *s = ESP32S2_SOC(opaque);
    if (level) {
        s->requested_reset = ESP32S2_SOC_RESET_DIG;
        qemu_system_reset_request(SHUTDOWN_CAUSE_GUEST_RESET);
    }
}

static void ESP32S2_cpu_reset(void* opaque, int n, int level)
{
    Esp32S2SocState *s = ESP32S2_SOC(opaque);
    if (level) {
        s->requested_reset = (n == 0) ? ESP32S2_SOC_RESET_PROCPU : ESP32S2_SOC_RESET_APPCPU;
        qemu_system_reset_request(SHUTDOWN_CAUSE_GUEST_RESET);
    }
}


static void ESP32S2_soc_reset(DeviceState *dev)
{
    Esp32S2SocState *s = ESP32S2_SOC(dev);

    uint32_t strap_mode = s->gpio.strap_mode;
    bool flash_boot_mode = ((strap_mode & 0x10) || (strap_mode & 0x1f) == 0x0c);

    if (s->requested_reset == 0) {
        s->requested_reset = ESP32S2_SOC_RESET_ALL;
    }
    if (s->requested_reset & ESP32S2_SOC_RESET_RTC) {
        device_cold_reset(DEVICE(&s->rtc_cntl));
    }
    if (s->requested_reset & ESP32S2_SOC_RESET_PERIPH) {
        device_cold_reset(DEVICE(&s->dport));
        device_cold_reset(DEVICE(&s->intmatrix));
        device_cold_reset(DEVICE(&s->sha));
        //device_cold_reset(DEVICE(&s->rsa));
        device_cold_reset(DEVICE(&s->gpio));
        for (int i = 0; i < ESP32S2_UART_COUNT; ++i) {
            device_cold_reset(DEVICE(&s->uart));
        }
        for (int i = 0; i < ESP32S2_FRC_COUNT; ++i) {
            device_cold_reset(DEVICE(&s->frc_timer[i]));
        }
        for (int i = 0; i < ESP32S2_TIMG_COUNT; ++i) {
            device_cold_reset(DEVICE(&s->timg[i]));
        }
        s->timg[0].flash_boot_mode = flash_boot_mode;
        device_cold_reset(DEVICE(&s->systimer));

        for (int i = 0; i < ESP32S2_SPI_COUNT; ++i) {
            device_cold_reset(DEVICE(&s->spi[i]));
        }
        for (int i = 0; i < ESP32_I2C_COUNT; i++) {
            device_cold_reset(DEVICE(&s->i2c[i]));
        }
        device_cold_reset(DEVICE(&s->efuse));
        if (s->eth) {
            device_cold_reset(s->eth);
        }
    }
    if (s->requested_reset & ESP32S2_SOC_RESET_PROCPU) {
        xtensa_select_static_vectors(&s->cpu[0].env, s->rtc_cntl.stat_vector_sel[0]);
        ESP32S2_remove_cpu_watchpoints(&s->cpu[0]);
        cpu_reset(CPU(&s->cpu[0]));
    }
    /*
    if (s->requested_reset & ESP32S2_SOC_RESET_APPCPU) {
        xtensa_select_static_vectors(&s->cpu[1].env, s->rtc_cntl.stat_vector_sel[1]);
        remove_cpu_watchpoints(&s->cpu[0]);
        cpu_reset(CPU(&s->cpu[1]));
    }
    */
    s->requested_reset = 0;
}

static void ESP32S2_cpu_stall(void* opaque, int n, int level)
{
    Esp32S2SocState *s = ESP32S2_SOC(opaque);

    bool stall;
    if (n == 0) {
        stall = s->rtc_cntl.cpu_stall_state[0];
    } else {
        stall = s->rtc_cntl.cpu_stall_state[1] && s->dport.appcpu_stall_state;
    }

    xtensa_runstall(&s->cpu[n].env, stall);
}

static void ESP32S2_clk_update(void* opaque, int n, int level)
{
    Esp32S2SocState *s = ESP32S2_SOC(opaque);
    if (!level) {
        return;
    }

    /* APB clock */
    uint32_t apb_clk_freq, cpu_clk_freq;
    if (s->rtc_cntl.soc_clk == ESP32_SOC_CLK_PLL) {
        const uint32_t cpu_clk_mul[] = {1, 2, 3};
        apb_clk_freq = s->rtc_cntl.pll_apb_freq;
        cpu_clk_freq = cpu_clk_mul[s->dport.cpuperiod_sel] * apb_clk_freq;
    } else {
        apb_clk_freq = s->rtc_cntl.xtal_apb_freq;
        cpu_clk_freq = apb_clk_freq;
    }
    qdev_prop_set_int32(DEVICE(&s->frc_timer), "apb_freq", apb_clk_freq);
    qdev_prop_set_int32(DEVICE(&s->timg[0]), "apb_freq", apb_clk_freq);
    qdev_prop_set_int32(DEVICE(&s->timg[1]), "apb_freq", apb_clk_freq);
    *(uint32_t*)(&s->cpu[0].env.config->clock_freq_khz) = cpu_clk_freq / 1000;
}

static void ESP32S2_soc_add_periph_device(MemoryRegion *dest, void* dev, hwaddr dport_base_addr)
{
    MemoryRegion *mr = sysbus_mmio_get_region(SYS_BUS_DEVICE(dev), 0);
    memory_region_add_subregion_overlap(dest, dport_base_addr, mr, 0);
    MemoryRegion *mr_apb = g_new(MemoryRegion, 1);
    char *name = g_strdup_printf("mr-apb-0x%08x", (uint32_t) dport_base_addr);
    memory_region_init_alias(mr_apb, OBJECT(dev), name, mr, 0, memory_region_size(mr));
    memory_region_add_subregion_overlap(dest, dport_base_addr - S2_DR_REG_UART_BASE + APB_REG_BASE, mr_apb, 0);
    g_free(name);
}

static void ESP32S2_soc_add_unimp_device(MemoryRegion *dest, const char* name, hwaddr dport_base_addr, size_t size)
{
    create_unimplemented_device(name, dport_base_addr, size);
    char * name_apb = g_strdup_printf("%s-apb", name);
    create_unimplemented_device(name_apb, dport_base_addr - S2_DR_REG_UART_BASE + APB_REG_BASE, size);
    g_free(name_apb);
}

void  stopme(void);

void  stopme(void) {
    printf("stop\n");
}


static void ESP32S2_soc_realize(DeviceState *dev, Error **errp)
{
    Esp32S2SocState *s = ESP32S2_SOC(dev);
    MachineState *ms = MACHINE(qdev_get_machine());

    const struct MemmapEntry *memmap = ESP32S2_memmap;
    MemoryRegion *sys_mem = get_system_memory();

    MemoryRegion *dram = g_new(MemoryRegion, 1);
    MemoryRegion *iram = g_new(MemoryRegion, 1);
    MemoryRegion *drom = g_new(MemoryRegion, 1);
    MemoryRegion *irom = g_new(MemoryRegion, 1);
    // irom1 is acually data rom...
    //MemoryRegion *irom1 = g_new(MemoryRegion, 1);

    MemoryRegion *icache0 = g_new(MemoryRegion, 1);
    MemoryRegion *icache1 = g_new(MemoryRegion, 1);
    MemoryRegion *rtcslow = g_new(MemoryRegion, 1);
    MemoryRegion *rtcfast_i = g_new(MemoryRegion, 1);
    MemoryRegion *rtcfast_d = g_new(MemoryRegion, 1);

    memory_region_init_rom(irom, NULL, "esp32s2.irom",
                           memmap[ESP32S2_MEMREGION_IROM].size, &error_fatal);
    memory_region_add_subregion(sys_mem, memmap[ESP32S2_MEMREGION_IROM].base, irom);

    //memory_region_init_rom(irom1, NULL, "esp32s2.irom1",
    //                       memmap[ESP32S2_MEMREGION_IROM1].size, &error_fatal);
    //memory_region_add_subregion(sys_mem, memmap[ESP32S2_MEMREGION_IROM1].base, irom1);


    memory_region_init_alias(drom, NULL, "esp32s2.drom", irom, 0xffff, memmap[ESP32S2_MEMREGION_DROM].size);
    memory_region_add_subregion(sys_mem, memmap[ESP32S2_MEMREGION_DROM1].base, drom);

    memory_region_init_ram(dram, NULL, "esp32s2.dram",
                           memmap[ESP32S2_MEMREGION_DRAM].size, &error_fatal);
    memory_region_add_subregion(sys_mem, memmap[ESP32S2_MEMREGION_DRAM].base, dram);

    memory_region_init_alias(iram, NULL, "esp32s2.iram", dram, 0x0, memmap[ESP32S2_MEMREGION_DRAM].size);
    memory_region_add_subregion(sys_mem, memmap[ESP32S2_MEMREGION_IRAM].base, iram);

    //memory_region_init_ram(iram, NULL, "esp32s2.iram",
    //                       memmap[ESP32S2_MEMREGION_IRAM].size, &error_fatal);
    //memory_region_add_subregion(sys_mem, memmap[ESP32S2_MEMREGION_IRAM].base, iram);

    memory_region_init_ram(icache0, NULL, "esp32s2.icache0",
                           memmap[ESP32S2_MEMREGION_ICACHE0].size, &error_fatal);
    memory_region_add_subregion(sys_mem, memmap[ESP32S2_MEMREGION_ICACHE0].base, icache0);

    memory_region_init_ram(icache1, NULL, "esp32s2.icache1",
                           memmap[ESP32S2_MEMREGION_ICACHE1].size, &error_fatal);
    memory_region_add_subregion(sys_mem, memmap[ESP32S2_MEMREGION_ICACHE1].base, icache1);

    memory_region_init_ram(rtcslow, NULL, "esp32s2.rtcslow",
                           memmap[ESP32S2_MEMREGION_RTCSLOW].size, &error_fatal);
    memory_region_add_subregion(sys_mem, memmap[ESP32S2_MEMREGION_RTCSLOW].base, rtcslow);

    /* RTC Fast memory is only accessible by the PRO CPU */

    memory_region_init_ram(rtcfast_i, NULL, "esp32s2.rtcfast_i",
                           memmap[ESP32S2_MEMREGION_RTCSLOW].size, &error_fatal);
    memory_region_add_subregion(&s->cpu_specific_mem[0], memmap[ESP32S2_MEMREGION_RTCFAST_I].base, rtcfast_i);

    memory_region_init_alias(rtcfast_d, NULL, "esp32s2.rtcfast_d", rtcfast_i, 0, memmap[ESP32S2_MEMREGION_RTCFAST_D].size);
    memory_region_add_subregion(&s->cpu_specific_mem[0], memmap[ESP32S2_MEMREGION_RTCFAST_D].base, rtcfast_d);

    for (int i = 0; i < ms->smp.cpus; ++i) {
        qdev_realize(DEVICE(&s->cpu[i]), NULL, &error_fatal);
    }

    qdev_realize(DEVICE(&s->dport), &s->periph_bus, &error_fatal);
// S2_DR_REG_SYSTEM_BASE
    memory_region_add_subregion(sys_mem,0x3F4C1000 /*S2_DR_REG_SYSTEM_BASE*/,
                                sysbus_mmio_get_region(SYS_BUS_DEVICE(&s->dport), 0));
    //qdev_connect_gpio_out_named(DEVICE(&s->dport), ESP32S2_DPORT_APPCPU_RESET_GPIO, 0,
    //                            qdev_get_gpio_in_named(dev, ESP32S2_RTC_CPU_RESET_GPIO, 1));
    //qdev_connect_gpio_out_named(DEVICE(&s->dport), ESP32S2_DPORT_APPCPU_STALL_GPIO, 0,
    //                            qdev_get_gpio_in_named(dev, ESP32S2_RTC_CPU_STALL_GPIO, 1));
    //qdev_connect_gpio_out_named(DEVICE(&s->rtc_cntl), ESP32S2_DPORT_CLK_UPDATE_GPIO, 0,
    //                            qdev_get_gpio_in_named(dev, ESP32S2_RTC_CLK_UPDATE_GPIO, 0));
    DeviceState* intmatrix_dev = DEVICE(&s->dport.intmatrix);

    if (s->dport.flash_blk) {
        for (int i = 0; i < ESP32S2_CPU_COUNT; ++i) {
            Esp32CacheRegionState *drom0 = &s->dport.cache_state[i].drom0;
            memory_region_add_subregion_overlap(&s->cpu_specific_mem[i], drom0->base, &drom0->mem, -1);
            Esp32CacheRegionState *iram0 = &s->dport.cache_state[i].iram0;
            memory_region_add_subregion_overlap(&s->cpu_specific_mem[i], iram0->base, &iram0->mem, -1);
        }
    }

    qdev_realize(DEVICE(&s->sha), &s->periph_bus, &error_fatal);
    ESP32S2_soc_add_periph_device(sys_mem, &s->sha, S2_DR_REG_SHA_BASE);

    qdev_realize(DEVICE(&s->rtc_cntl), &s->periph_bus, &error_abort);
    ESP32S2_soc_add_periph_device(sys_mem,  &s->rtc_cntl, S2_DR_REG_RTCCNTL_BASE);

    qdev_connect_gpio_out_named(DEVICE(&s->rtc_cntl), ESP32_RTC_DIG_RESET_GPIO, 0,
                                qdev_get_gpio_in_named(dev, ESP32_RTC_DIG_RESET_GPIO, 0));
    qdev_connect_gpio_out_named(DEVICE(&s->rtc_cntl), ESP32_RTC_CLK_UPDATE_GPIO, 0,
                                qdev_get_gpio_in_named(dev, ESP32_RTC_CLK_UPDATE_GPIO, 0));
    for (int i = 0; i < ms->smp.cpus; ++i) {
        qdev_connect_gpio_out_named(DEVICE(&s->rtc_cntl), ESP32_RTC_CPU_RESET_GPIO, i,
                                    qdev_get_gpio_in_named(dev, ESP32_RTC_CPU_RESET_GPIO, i));
        qdev_connect_gpio_out_named(DEVICE(&s->rtc_cntl), ESP32_RTC_CPU_STALL_GPIO, i,
                                    qdev_get_gpio_in_named(dev, ESP32_RTC_CPU_STALL_GPIO, i));
    }

    qdev_realize(DEVICE(&s->gpio), &s->periph_bus, &error_fatal);
    ESP32S2_soc_add_periph_device(sys_mem, &s->gpio, S2_DR_REG_GPIO_BASE);

    stopme();

    for (int i = 0; i < ESP32S2_UART_COUNT; ++i) {
        const hwaddr uart_base[] = {S2_DR_REG_UART_BASE, S2_DR_REG_UART1_BASE};
        qdev_realize(DEVICE(&s->uart[i]), &s->periph_bus, &error_fatal);
        ESP32S2_soc_add_periph_device(sys_mem, &s->uart[i], uart_base[i]);
        sysbus_connect_irq(SYS_BUS_DEVICE(&s->uart[i]), 0,
                           qdev_get_gpio_in(intmatrix_dev, ETS_UART0_INTR_SOURCE + i));
    }

    for (int i = 0; i < ESP32S2_FRC_COUNT; ++i) {
        qdev_realize(DEVICE(&s->frc_timer[i]), &s->periph_bus, &error_fatal);

        ESP32S2_soc_add_periph_device(sys_mem, &s->frc_timer[i], S2_DR_REG_FRC_TIMER_BASE + i * ESP32_FRC_TIMER_STRIDE);

        sysbus_connect_irq(SYS_BUS_DEVICE(&s->frc_timer[i]), 0,
                           qdev_get_gpio_in(intmatrix_dev, ETS_TIMER1_INTR_SOURCE + i));
    }

    for (int i = 0; i < ESP32S2_TIMG_COUNT; ++i) {
        const hwaddr timg_base[] = {S2_DR_REG_TIMERGROUP0_BASE, S2_DR_REG_TIMERGROUP1_BASE};
        qdev_realize(DEVICE(&s->timg[i]), &s->periph_bus, &error_fatal);

        ESP32S2_soc_add_periph_device(sys_mem, &s->timg[i], timg_base[i]);

        int timg_level_int[] = { ETS_TG0_T0_LEVEL_INTR_SOURCE, ETS_TG1_T0_LEVEL_INTR_SOURCE };
        int timg_edge_int[] = { ETS_TG0_T0_EDGE_INTR_SOURCE, ETS_TG1_T0_EDGE_INTR_SOURCE };
        for (Esp32TimgInterruptType it = TIMG_T0_INT; it < TIMG_INT_MAX; ++it) {
            sysbus_connect_irq(SYS_BUS_DEVICE(&s->timg[i]), it, qdev_get_gpio_in(intmatrix_dev, timg_level_int[i] + it));
            sysbus_connect_irq(SYS_BUS_DEVICE(&s->timg[i]), TIMG_INT_MAX + it, qdev_get_gpio_in(intmatrix_dev, timg_edge_int[i] + it));
        }

        //qdev_connect_gpio_out_named(DEVICE(&s->timg[i]), ESP32_TIMG_WDT_CPU_RESET_GPIO, 0,
        //                            qdev_get_gpio_in_named(dev, ESP32_TIMG_WDT_CPU_RESET_GPIO, i));
        //qdev_connect_gpio_out_named(DEVICE(&s->timg[i]), ESP32_TIMG_WDT_SYS_RESET_GPIO, 0,
        //                            qdev_get_gpio_in_named(dev, ESP32_TIMG_WDT_SYS_RESET_GPIO, i));


    }
    qdev_realize(DEVICE(&s->systimer), &s->periph_bus, &error_abort);
    ESP32S2_soc_add_periph_device(sys_mem, &s->systimer, S2_DR_REG_SYSTIMER_BASE);


    for (int i = 0; i < ESP32S2_SPI_COUNT; ++i) {
        const hwaddr spi_base[] = {
            S2_DR_REG_SPI0_BASE, S2_DR_REG_SPI1_BASE, S2_DR_REG_SPI2_BASE, S2_DR_REG_SPI3_BASE
        };
         qdev_realize(DEVICE(&s->spi[i]), &s->periph_bus, &error_fatal);

        ESP32S2_soc_add_periph_device(sys_mem, &s->spi[i], spi_base[i]);

        sysbus_connect_irq(SYS_BUS_DEVICE(&s->spi[i]), 0,
                           qdev_get_gpio_in(intmatrix_dev, ETS_SPI0_INTR_SOURCE + i));
    }

    for (int i = 0; i < ESP32_I2C_COUNT; i++) {
        const hwaddr i2c_base[] = {
            DR_REG_I2C_EXT_BASE, DR_REG_I2C1_EXT_BASE
        };
        qdev_realize(DEVICE(&s->i2c[i]), &s->periph_bus, &error_fatal);

        ESP32S2_soc_add_periph_device(sys_mem, &s->i2c[i], i2c_base[i]);

        sysbus_connect_irq(SYS_BUS_DEVICE(&s->i2c[i]), 0,
                           qdev_get_gpio_in(intmatrix_dev, ETS_I2C_EXT0_INTR_SOURCE + i));
    }



//OLAS random
    qdev_realize(DEVICE(&s->rng), &s->periph_bus, &error_abort);
    ESP32S2_soc_add_periph_device(sys_mem, &s->rng, 0x60035110);

    qdev_realize(DEVICE(&s->efuse),  &s->periph_bus, &error_abort);
    ESP32S2_soc_add_periph_device(sys_mem, &s->efuse, S2_DR_REG_EFUSE_BASE);
    sysbus_connect_irq(SYS_BUS_DEVICE(&s->efuse), 0,
                       qdev_get_gpio_in(intmatrix_dev, ETS_EFUSE_INTR_SOURCE));




    ESP32S2_soc_add_unimp_device(sys_mem, "esp32s2.extmem", S2_DR_REG_EXTMEM_BASE, 0x1000);
//  0x6000E044
    ESP32S2_soc_add_unimp_device(sys_mem, "esp32s2.analog", 0x6000E000, 0x1000);
    ESP32S2_soc_add_unimp_device(sys_mem, "esp32s2.rtcio", S2_DR_REG_RTCIO_BASE, 0x400);
    ESP32S2_soc_add_unimp_device(sys_mem, "esp32s2.rtcio", S2_DR_REG_SENS_BASE, 0x400);
    ESP32S2_soc_add_unimp_device(sys_mem, "esp32s2.iomux", S2_DR_REG_IO_MUX_BASE, 0x2000);
    ESP32S2_soc_add_unimp_device(sys_mem, "esp32s2.hinf", S2_DR_REG_HINF_BASE, 0x1000);
    ESP32S2_soc_add_unimp_device(sys_mem, "esp32s2.slc", S2_DR_REG_SLC_BASE, 0x1000);
    ESP32S2_soc_add_unimp_device(sys_mem, "esp32s2.slchost", S2_DR_REG_SLCHOST_BASE, 0x1000);
    ESP32S2_soc_add_unimp_device(sys_mem, "esp32s2.apbctrl", S2_DR_REG_APB_CTRL_BASE, 0x1000);
    ESP32S2_soc_add_unimp_device(sys_mem, "esp32s2.i2s0", S2_DR_REG_I2S_BASE, 0x1000);
    //ESP32S2_soc_add_unimp_device(sys_mem, "esp32s2.i2s1", S2_DR_REG_I2S1_BASE, 0x1000);
    ESP32S2_soc_add_unimp_device(sys_mem, "esp32s2.i2c0", S2_DR_REG_I2C_EXT_BASE, 0x1000);
    ESP32S2_soc_add_unimp_device(sys_mem, "esp32s2.i2c1", S2_DR_REG_I2C1_EXT_BASE, 0x1000);
    ESP32S2_soc_add_unimp_device(sys_mem, "esp32s2.usb", S2_DR_REG_USB_BASE, 0x1000);
    //ESP32S2_soc_add_unimp_device(sys_mem, "esp32s2.rndreg", 0x60035000, 0x1000);

    

    //ESP32S2_soc_add_unimp_device(sys_mem, "esp32s2.unknown", 0x61801000, 0x1000);

    qdev_realize(DEVICE(&s->myunimp), &s->periph_bus , &error_abort);
    ESP32S2_soc_add_periph_device(sys_mem, &s->myunimp, 0x61800000);

/*
$2 = 0x61800040
(gdb) p/x $a9
$3 = 0x0
(gdb) ni
0x400181fa in Cache_Invalidate_ICache_Items ()
(gdb) p/x $a3
$4 = 0x200
*/
    


    qemu_register_reset((QEMUResetHandler*) ESP32S2_soc_reset, dev);
}

static void esp32s2_soc_init(Object *obj)
{
    Esp32S2SocState *s = ESP32S2_SOC(obj);
    MachineState *ms = MACHINE(qdev_get_machine());
    char name[16];

    MemoryRegion *system_memory = get_system_memory();

    for (int i = 0; i < ms->smp.cpus; ++i) {
        snprintf(name, sizeof(name), "cpu%d", i);
        object_initialize_child(obj, name, &s->cpu[i], TYPE_ESP32S2_CPU);

        const uint32_t cpuid[ESP32S2_CPU_COUNT] = { 0xcdcd /*, 0xabab */};
        s->cpu[i].env.sregs[PRID] = cpuid[i];

        snprintf(name, sizeof(name), "cpu%d-mem", i);
        memory_region_init(&s->cpu_specific_mem[i], NULL, name, UINT32_MAX);

        CPUState* cs = CPU(&s->cpu[i]);
        cs->num_ases = 1;
        cpu_address_space_init(cs, 0, "cpu-memory", &s->cpu_specific_mem[i]);

        MemoryRegion *cpu_view_sysmem = g_new(MemoryRegion, 1);
        snprintf(name, sizeof(name), "cpu%d-sysmem", i);
        memory_region_init_alias(cpu_view_sysmem, NULL, name, system_memory, 0, UINT32_MAX);
        memory_region_add_subregion_overlap(&s->cpu_specific_mem[i], 0, cpu_view_sysmem, 0);
        cs->memory = &s->cpu_specific_mem[i];
    }

    for (int i = 0; i < ESP32S2_UART_COUNT; ++i) {
        snprintf(name, sizeof(name), "uart%d", i);
        object_initialize_child(obj, name, &s->uart[i], TYPE_ESP32S2_UART);
    }

    object_property_add_alias(obj, "serial0", OBJECT(&s->uart[0]), "chardev");
    //object_property_add_alias(obj, "serial1", OBJECT(&s->uart[1]), "chardev");
    //object_property_add_alias(obj, "serial2", OBJECT(&s->uart[2]), "chardev");

    object_initialize_child(obj, "gpio", &s->gpio, TYPE_ESP32S2_GPIO);

    object_initialize_child(obj, "dport", &s->dport, TYPE_ESP32S2_DPORT);

    object_initialize_child(obj, "intmatrix", &s->intmatrix, TYPE_ESP32S2_INTMATRIX);

    object_initialize_child(obj, "rtc_cntl", &s->rtc_cntl, TYPE_ESP32_RTC_CNTL);

    object_initialize_child(obj, "myunimp", &s->rtc_cntl, TYPE_ESP32S2_MYUNIMP);

    for (int i = 0; i < ESP32_FRC_COUNT; ++i) {
        snprintf(name, sizeof(name), "frc%d", i);
        object_initialize_child(obj, name, &s->frc_timer[i], TYPE_ESP32_FRC_TIMER);
    }

    for (int i = 0; i < ESP32_TIMG_COUNT; ++i) {
        snprintf(name, sizeof(name), "timg%d", i);
        object_initialize_child(obj, name, &s->timg[i], TYPE_ESP32_TIMG);
    }


    object_initialize_child(obj, "systimer", &s->systimer, TYPE_ESP32S2_MYUNIMP);


    for (int i = 0; i < ESP32_SPI_COUNT; ++i) {
        snprintf(name, sizeof(name), "spi%d", i);
        object_initialize_child(obj, name, &s->spi[i], TYPE_ESP32S2_SPI);
    }

    for (int i = 0; i < ESP32_I2C_COUNT; ++i) {
        snprintf(name, sizeof(name), "i2c%d", i);
        object_initialize_child(obj, name, &s->i2c[i], TYPE_ESP32_I2C);
    }

    object_initialize_child(obj, "rng", &s->rng, TYPE_ESP32_RNG);

    object_initialize_child(obj, "sha", &s->sha, TYPE_ESP32S2_SHA);

    // object_initialize_child(obj, "rsa", &s->rsa, TYPE_ESP32_RSA);

    object_initialize_child(obj, "efuse", &s->efuse, TYPE_ESP32S2_EFUSE);

    // object_initialize_child(obj, "flash_enc", &s->flash_enc, TYPE_ESP32_FLASH_ENCRYPTION);


    // qdev_init_gpio_in_named(DEVICE(s), esp32_timg_sys_reset, ESP32_TIMG_WDT_SYS_RESET_GPIO, 2);


    qdev_init_gpio_in_named(DEVICE(s), ESP32S2_dig_reset, ESP32_RTC_DIG_RESET_GPIO, 1);
    qdev_init_gpio_in_named(DEVICE(s), ESP32S2_cpu_reset, ESP32_RTC_CPU_RESET_GPIO, ESP32S2_CPU_COUNT);
    qdev_init_gpio_in_named(DEVICE(s), ESP32S2_cpu_stall, ESP32_RTC_CPU_STALL_GPIO, ESP32S2_CPU_COUNT);
    qdev_init_gpio_in_named(DEVICE(s), ESP32S2_clk_update, ESP32_RTC_CLK_UPDATE_GPIO, 1);

    const char *rom_filename = "s2rom.bin";
    //const char *irom_filename = "irom1.bin";

    //irom_filename = qemu_find_file(QEMU_FILE_TYPE_BIOS, irom_filename);
    //if (!irom_filename ||
    //    load_image_targphys(irom_filename, 0x3ffa0000, 64*1024) < 0) { 
    //    error_report("unable to load ROM image '%s'\n", irom_filename);
    //    exit(EXIT_FAILURE);
    //}
/* 0x3ffa0000 and 0x40010000 maps to same data, Data and Instrucions
    if (!irom_filename ||
        load_image_targphys(irom_filename, 0x40010000, 64*1024) < 0) { 
        error_report("unable to load ROM image '%s'\n", irom_filename);
        exit(EXIT_FAILURE);
    }
*/
    
    rom_filename = qemu_find_file(QEMU_FILE_TYPE_BIOS, rom_filename);
    if (!rom_filename ||
        load_image_targphys(rom_filename, 0x40000000, 128*1024) < 0) { 
        error_report("unable to load ROM image '%s'\n", rom_filename);
        exit(EXIT_FAILURE);
    }
/*
//MemTxResult address_space_write_rom(AddressSpace *as, hwaddr addr,
//                                    MemTxAttrs attrs,
//                                    const uint8_t *buf, hwaddr len);

            CPUState* cst = CPU(&s->cpu[0]);
            dump_mmu(&s->cpu[0]);

            
            // Patch rom, ets_unpack_flash_code
             
            //AddressSpace *as=cpu_get_address_space(cst , 1);

            //address_space_write_rom(as,0x40010f58,MEMTXATTRS_UNSPECIFIED, patch_ret, sizeof(patch_ret));
*/


}

static Property ESP32S2_soc_properties[] = {
    DEFINE_PROP_END_OF_LIST(),
};

static void ESP32S2_soc_class_init(ObjectClass *klass, void *data)
{
    DeviceClass *dc = DEVICE_CLASS(klass);

    dc->reset = ESP32S2_soc_reset;
    dc->realize = ESP32S2_soc_realize;
     device_class_set_props(dc, ESP32S2_soc_properties);
}

static const TypeInfo ESP32S2_soc_info = {
    .name = TYPE_ESP32S2_SOC,
    .parent = TYPE_DEVICE,
    .instance_size = sizeof(Esp32S2SocState),
    .instance_init = esp32s2_soc_init,
    .class_init = ESP32S2_soc_class_init
};

static void ESP32S2_soc_register_types(void)
{
    type_register_static(&ESP32S2_soc_info);
}

type_init(ESP32S2_soc_register_types)


static uint64_t translate_phys_addr(void *opaque, uint64_t addr)
{
    XtensaCPU *cpu = opaque;

    return cpu_get_phys_page_debug(CPU(cpu), addr);
}


struct Esp32S2MachineState {
    MachineState parent;

    Esp32S2SocState esp32S2;
    DeviceState *flash_dev;
};

#define TYPE_ESP32S2_MACHINE MACHINE_TYPE_NAME("esp32S2")
OBJECT_DECLARE_SIMPLE_TYPE(Esp32S2MachineState, ESP32S2_MACHINE)



static void ESP32S2_machine_init_spi_flash(MachineState *machine, Esp32S2SocState *s, BlockBackend* blk)
{
   /* "main" flash chip is attached to SPI1 */
    DeviceState *spi_master = DEVICE(&s->spi[1]);
    BusState* spi_bus = qdev_get_child_bus(spi_master, "spi");
    DeviceState *flash_dev = qdev_new("gd25q32");
    qdev_prop_set_drive(flash_dev, "drive", blk);
    qdev_realize_and_unref(flash_dev, spi_bus, &error_fatal);
    qdev_connect_gpio_out_named(spi_master, SSI_GPIO_CS, 0,
                                qdev_get_gpio_in_named(flash_dev, SSI_GPIO_CS, 0));
}


static void ESP32S2_machine_init_openeth(Esp32S2SocState *ss)
{
    SysBusDevice *sbd;
    NICInfo *nd = &nd_table[0];
    MemoryRegion* sys_mem = get_system_memory();
    hwaddr reg_base = ADDR_FIFO_USB_0;
    hwaddr desc_base = reg_base + 0x400;
    qemu_irq irq = qdev_get_gpio_in(DEVICE(&ss->intmatrix), ETS_ETH_MAC_INTR_SOURCE);

    const char* type_openeth = "open_eth";
    if (nd->used && nd->model && strcmp(nd->model, type_openeth) == 0) {
        DeviceState* open_eth_dev = qdev_new(type_openeth);
        ss->eth = open_eth_dev;
        qdev_set_nic_properties(open_eth_dev, nd);
        sbd = SYS_BUS_DEVICE(open_eth_dev);
        sysbus_realize_and_unref(sbd, &error_fatal);
        sysbus_connect_irq(sbd, 0, irq);
        memory_region_add_subregion(sys_mem, reg_base, sysbus_mmio_get_region(sbd, 0));
        memory_region_add_subregion(sys_mem, desc_base, sysbus_mmio_get_region(sbd, 1));
    }
}
/**************/

#define ESP32S2_UNIMP(obj) OBJECT_CHECK(Esp32UnimpState, (obj), TYPE_ESP32S2_MYUNIMP)

#define ESP32S2_UNIMP_VAL 0x0a 

static uint64_t ESP32S2_unimp_read(void *opaque, hwaddr addr, unsigned int size)
{
    Esp32UnimpState *s = ESP32S2_UNIMP(opaque);
    //printf("unimp read  %08X\n",(unsigned int)addr);

    uint64_t r = 0;
    switch (addr) {
                // 0x618000000 
    case 0x00:
        // Cache invaligate dcache items
       r=0x200;
       break;


    case 0x40:
    case 0x44:
    case 0x114:
    case 0x11C:
    case 0x60:
        {
            r=s->cache_value;
            if (s->cache_value==0x1) {
                printf(".\n");
                s->cache_value=0x80000;    
            } else if (s->cache_value==0x80000) {
                printf("o\n");
                s->cache_value=0x200000;
            } else if (s->cache_value==0x200000) {
                printf("x\n");
                s->cache_value=0x200;
            } else {
                s->cache_value=0x1;
            }
        }
       break;
        //r = 0x200;
        //break;

    case 0x1200 ... 0x1400: 
    {
        r=s->mmu_table[addr-0x1200];
    }
    break;

    default:
        break;
    }
    return r;
}

//#define SOC_DROM_LOW 0x3f000000
/* Use first 63 blocks in MMU for bootloader_mmap,
   63th block for bootloader_flash_read
*/
#define MMU_BLOCK0_VADDR  SOC_DROM_LOW
// 4MB
#define MMU_SIZE          (0x3f0000)
#define MMU_BLOCK63_VADDR (MMU_BLOCK0_VADDR + MMU_SIZE)
#define FLASH_READ_VADDR MMU_BLOCK63_VADDR

#define MMU_FREE_PAGES    (MMU_SIZE / FLASH_BLOCK_SIZE)

#if 0
                    
/**
 * @brief Set ICache mmu mapping.
 *        Please do not call this function in your SDK application.
 *
 * @param  uint32_t ext_ram : DPORT_MMU_ACCESS_FLASH for flash,
 DPORT_MMU_ACCESS_SPIRAM for spiram, DPORT_MMU_INVALID for invalid.
    *
    * @param  uint32_t vaddr : virtual address in CPU address space.
    *                              Can be Iram0,Iram1,Irom0,Drom0 and AHB buses
address.
    *                              Should be aligned by psize.
    *
    * @param  uint32_t paddr : physical address in external memory.
    *                              Should be aligned by psize.
    *
    * @param  uint32_t psize : page size of ICache, in kilobytes. Should be 64
here.
    *
    * @param  uint32_t num : pages to be set.
    *
    * @param  uint32_t fixed : 0 for physical pages grow with virtual pages,
other for virtual pages map to same physical page.
    *
    * @return uint32_t: error status
    *                   0 : mmu set success
    *                   2 : vaddr or paddr is not aligned
    *                   3 : psize error
    *                   4 : vaddr is out of range
    */ 

uint Cache_Ibus_MMU_Set(uint ext_ram,uint vaddr,uint paddr,uint psize,uint num,int fixed)
{
  uint ret;
  uint uVar2;
  uint *puVar3;
  uint uVar4;
  int iVar5;
  uint uVar6;
  
  ret = 2;
  if (((0xffff >> ((char)(0x40 / psize) - 1U & 0x1f) & (vaddr | paddr)) == 0) &&
     (ret = 3, psize == 0x40)) {
    do {
      if (num == 0) {
        return 0;
      }
      uVar4 = vaddr + 0x400000 & 0xffc00000;
      iVar5 = uVar4 - vaddr;
      uVar6 = iVar5 >> 0x10;
      if (num <= uVar6) {
        uVar6 = num;
      }
      if (vaddr + 0xc1000000 < 0x400000) {
        uVar2 = (vaddr >> 0x10 & 0x3f) + 0x80;
      } else {
        if (vaddr + 0xc0000000 < 0x400000) {
          uVar2 = vaddr >> 0x10 & 0x3f;
        } else {
          if (0x3fffff < vaddr + 0xbfc00000) {
              return 4;  // vaddr is out of range
          }
          uVar2 = (vaddr >> 0x10 & 0x3f) + 0x40;
        }
      }
      puVar3 = (uint *)(&DAT_61801000 + uVar2 * 4);
      uVar2 = 0;
      while (uVar2 != uVar6) {
        if (fixed == 0) {
          *puVar3 = uVar2 + (paddr >> 0x10) | ext_ram;
        }
        else {
          *puVar3 = paddr >> 0x10 | ext_ram;
        }
        uVar2 = uVar2 + 1;
        puVar3 = puVar3 + 1;
      }
      num = num - uVar2;
      vaddr = uVar4;
      if (fixed == 0) {
        paddr = paddr + iVar5;
      }
    } while( true );
  }
  return ret;
}

// The fun stops at verify_image_header()

read RTC_CNTL_RESET_STATE_REG
unimp write  000012FC,00008001




int Cache_Dbus_MMU_Set(uint32_t ext_ram,uint32_t vaddr,uint32_t paddr,uint32_t psize_64,
                      uint32_t num_pages,uint32_t fixed)

{
  int iVar1;
  int iVar2;
  uint *puVar3;
  uint uVar4;
  uint uVar5;
  uint uVar6;
  
                    /* 
                         * @brief Set DCache mmu mapping.
                         *        Please do not call this function in your SDK application.
                         *
                         * @param  uint32_t ext_ram : DPORT_MMU_ACCESS_FLASH for flash,
                       DPORT_MMU_ACCESS_SPIRAM for spiram, DPORT_MMU_INVALID for invalid.
                         *
                         * @param  uint32_t vaddr : virtual address in CPU address space.
                         *                              Can be DRam0, DRam1, DRom0, DPort and AHB
                       buses address.
                         *                              Should be aligned by psize.
                         *
                         * @param  uint32_t paddr : physical address in external memory.
                         *                              Should be aligned by psize.
                         *
                         * @param  uint32_t psize : page size of DCache, in kilobytes. Should be 64
                       here.
                         *
                         * @param  uint32_t num : pages to be set.
                       
                         * @param  uint32_t fixed : 0 for physical pages grow with virtual pages,
                       other for virtual pages map to same physical page.
                         *
                         * @return uint32_t: error status
                         *                   0 : mmu set success
                         *                   2 : vaddr or paddr is not aligned
                         *                   3 : psize error
                         *                   4 : vaddr is out of range
                         */
  iVar1 = 2;
  if (((0xffff >> ((char)(0x40 / psize_64) - 1U & 0x1f) & (vaddr | paddr)) == 0) &&
     (iVar1 = 3, psize_64 == 0x40)) {
    do {
      if (num_pages == 0) {
        return 0;
      }
      uVar5 = vaddr + 0x400000 & 0xffc00000;
      iVar1 = uVar5 - vaddr;  // 0x400000 
      uVar6 = iVar1 >> 0x10;   // 1
      if (num_pages <= uVar6) {
        uVar6 = num_pages;
      }
      if (vaddr + 0xc0400000 < 0x400000) {
        iVar2 = (vaddr >> 0x10 & 0x3f) + 0xc0;
      } else {
        if (vaddr + 0xc0800000 < 0x400000) {
          iVar2 = (vaddr >> 0x10 & 0x3f) + 0x100;
        } else {
          if (0x3fffff < vaddr + 0xc0c00000) {
            return 4;
          }
          iVar2 = (vaddr >> 0x10 & 0x3f) + 0x140;
        }
      }
      puVar3 = (uint *)(&DAT_ram_61801000 + iVar2 * 4);
      uVar4 = 0;
      while (uVar4 != uVar6) {
        if (fixed == 0) {
          *puVar3 = uVar4 + (paddr >> 0x10) | ext_ram;
        }
        else {
          *puVar3 = paddr >> 0x10 | ext_ram;
        }
        uVar4 = uVar4 + 1;
        puVar3 = puVar3 + 1;
      }
      num_pages = num_pages - uVar4;
      vaddr = uVar5;
      if (fixed == 0) {
        paddr = paddr + iVar1;
      }
    } while( true );
  }
  return iVar1;
}

void Cache_MMU_Init(void)

{
  undefined4 *puVar1;
  
  puVar1 = (undefined4 *)&DAT_ram_61801000;
  do {
    *puVar1 = 0x4000;
    puVar1 = puVar1 + 1;
  } while (puVar1 != (undefined4 *)0x61801600);
  return;
}
#endif

//#define ICACHE_MMU_SIZE                 0x300
//#define DCACHE_MMU_SIZE                 0x300
// 0x180 descripttors => 0x600 


//#define DR_REG_EXTMEM_BASE                      0x61800000


// Icache, Databus 0x3f0_0000 - 0x3F3F_FFFF 4MB   63 descriptors (0x3F)
// 0x3f*4 = FC

// Icache MMU      0x4008_000 - 0x407F_FFFF  7.5M  119 descriptors (0x77)
// 0x77*4 = 1DC
//#define DR_REG_MMU_TABLE                        0x61801000

// Dcache          0x3F50_0000 - 0x3FF7_FFFF  10.5MB 167 descriptors (0xa7) 
// 0xa7*4 = 29C

//#define DR_REG_ITAG_TABLE                       0x61802000
//#define DR_REG_DTAG_TABLE                       0x61803000





static void ESP32S2_unimp_write(void *opaque, hwaddr addr,
                       uint64_t value, unsigned int size)
{
    Esp32UnimpState *s = ESP32S2_UNIMP(opaque);
    //printf("unimp write  %08X,%08X\n",(unsigned int)addr,(unsigned int)value);
    //if (value!=0x4000) printf("unimp write  %08X,%08X\n",(unsigned int)addr,(unsigned int)value);




    uint32_t tmp_flash_cache[ESP32S2_CACHE_PAGE_SIZE*4];
    if ((value & 0x8000) == 0x8000) {
            printf("** Unimp write  %08X,%08X\n",(unsigned int)addr,(unsigned int)value);
    }

    switch (addr) {
        case 0x40:
        {
            if (value==0x100) {
                s->cache_value=0x80000;
            }
            else {
                s->cache_value=value;
            }
        }
        break;


        // 0x61801000
        case 0x1000 ... 0x1100: {
            // FLASH_MMU_TABLE
            if ((value & 0x8000) == 0x8000) {
                uint32_t esp_addr = 0x40080000 + ((addr-0x1020)/4) * ESP32S2_CACHE_PAGE_SIZE;
                uint32_t flash_addr =(value & 0x3FFF) << 16;
                printf("i write  %08X,%08X,%08X\n",(unsigned int)addr,(unsigned int)value,esp_addr);


                blk_pread(s->flash_blk, flash_addr, /*cache_page*/tmp_flash_cache, ESP32S2_CACHE_PAGE_SIZE);
                printf("%08X,%08X,b %08X,p %08X\n",*(uint32_t *)tmp_flash_cache,*(uint32_t *)&tmp_flash_cache[4],*(uint32_t *)&tmp_flash_cache[0x1000],*(uint32_t *)&tmp_flash_cache[0x8000]);
                cpu_physical_memory_write(esp_addr, tmp_flash_cache, ESP32S2_CACHE_PAGE_SIZE );                
            }
        } 
        break;
 /*
    Page 26 in TRM

1. System and Memory1.3.2.5RTC FAST MemoryRTC FAST Memory is an 8-KB, read-and-write SRAM, addressed by the CPU on the data or instruction bus, inthe same order, via range(s) described in Table2.1.3.2.6RTC SLOW MemoryRTC SLOW Memory is an 8-KB, read-and-write SRAM, addressed by the CPU via range(s) shared by the databus and the instruction bus, as described in Table2.RTC SLOW Memory can also be used as a peripheral addressable to the CPU via either 0x3F42_1000~0x3F42_2FFF or 0x6002_1000~0x6002_2FFF on the data bus.1.3.3External MemoryESP32-S2 supports multiple QSPI/OSPI flash and RAM chips. It also supports hardware encryption/decryptionbased on XTS-AES to protect user programs and data in the flash and external RAM.1.3.3.1External Memory Address MappingThe CPU accesses the external flash and RAM via the cache. According to the MMU settings, the cache mapsthe CPU’s address to the external physical memory address. Due to this address mapping, the ESP32-S2 canaddress up to 1 GB external flash and 1 GB external RAM.Using the cache, ESP32-S2 can support the following address space mappings at the same time.•Up to 7.5 MB instruction bus address space can be mapped into the external flash or RAM as individual 64KB blocks, via the instruction cache (ICache). Byte (8-bit), half-word (16-bit) and word (32-bit) reads aresupported.•Up to 4 MB read-only data bus address space can be mapped into the external flash or RAM as individual64 KB blocks, via ICache. Byte (8-bit), half-word (16-bit) and word (32-bit) reads are supported.•Up to 10.5 MB data bus address space can be mapped into the external RAM as individual 64 KB blocks,via DCache. Byte (8-bit), half-word (16-bit) or word (32-bit) reads and writes are supported. Blocks fromthis 10.5 MB space can also be mapped into the external flash or RAM, for read operations only.Table3lists the mapping between the cache and the corresponding address ranges on the data bus andinstruction bus.Table 3: External Memory Address MappingBoundary AddressBus TypeLow AddressHigh AddressSizeTargetPermission ControlData bus0x3F00_00000x3F3F_FFFF4 MBICacheYESData bus0x3F50_00000x3FF7_FFFF10.5 MBDCacheYESInstruction bus0x4008_00000x407F_FFFF7.5 MBICacheYES

*/
// int e = Cache_Ibus_MMU_Set(MMU_ACCESS_FLASH, MMU_BLOCK63_VADDR, map_at, 64, 1, 0);
/*
        case 0x12fc:
        // 3F3F0000
        {                        // 0x3f3f1000
                uint32_t esp_addr = 3F3F0000; // 0x3ffb2000 +  ((addr-0x1200)/4) * ESP32S2_CACHE_PAGE_SIZE;
                uint32_t flash_addr =(value & 0x3FFF) << 16;
                printf("12fc unimp write  %08X,%08X,%08X\n",(unsigned int)addr,(unsigned int)value,esp_addr);
                blk_pread(s->flash_blk, flash_addr, tmp_flash_cache, 4*ESP32S2_CACHE_PAGE_SIZE);
                printf("%08X,%08X,b %08X,p %08X\n",*(uint32_t *)tmp_flash_cache,*(uint32_t *)&tmp_flash_cache[4],*(uint32_t *)&tmp_flash_cache[0x1000],*(uint32_t *)&tmp_flash_cache[0x8000]);
                cpu_physical_memory_write(esp_addr, tmp_flash_cache, ESP32S2_CACHE_PAGE_SIZE );                

        }
        break;
*/

        case 0x1200 ... 0x12fc:
        {
            uint32_t phys_addr = 0x3f000000 + ((addr-0x1200)/4) * ESP32S2_CACHE_PAGE_SIZE;
            uint32_t flash_addr =(value & 0x3FFF) << 16;
            if ((value & 0x8000) == 0x8000) {
                printf("d write  %08X,%08X,%08X\n",(unsigned int)addr,(unsigned int)value,phys_addr);
                printf("MMU Map flash %08X to %08X\n",flash_addr,phys_addr);
                blk_pread(s->flash_blk, flash_addr, /*cache_page*/tmp_flash_cache, ESP32S2_CACHE_PAGE_SIZE);
                printf("%08X,%08X,b %08X,p %08X\n",*(uint32_t *)tmp_flash_cache,*(uint32_t *)&tmp_flash_cache[4],*(uint32_t *)&tmp_flash_cache[0x1000],*(uint32_t *)&tmp_flash_cache[0x8000]);
                cpu_physical_memory_write(phys_addr, tmp_flash_cache, ESP32S2_CACHE_PAGE_SIZE );
            }
        }
        break;


        case 0x1300 ... 0x1400: 
        {
            s->mmu_table[addr-0x1200]=value;
            // /sizeof(uint32_t)
            //uint32_t mmu_entry = value;
            //uint8_t* cache_data = (uint8_t*) memory_region_get_ram_ptr(&crs->mem);
            // 0x3f008000

           // (const esp_partition_info_t *) 0x3f008000
            uint32_t phys_addr = 0x3ffb2000 + ((addr-0x1200)/4) * ESP32S2_CACHE_PAGE_SIZE;
            uint32_t flash_addr =(value & 0x3FFF) << 16;
            //uint32_t* cache_page = (uint32_t*) (cache_data + i * ESP32S2_CACHE_PAGE_SIZE);
#if 0
            if ((s->mmu_table[addr-0x1200]!=0x4000) && value==0x4000) {
                for(int reset=0;reset<ESP32S2_CACHE_PAGE_SIZE;reset++) {
                    tmp_flash_cache[reset]=0xbaadbaad ;
                }
                cpu_physical_memory_write(phys_addr, (char *)tmp_flash_cache, ESP32S2_CACHE_PAGE_SIZE );
            }
#endif

            if ((value & 0x8000) == 0x8000) {
                printf("unimp write  %08X,%08X,%08X\n",(unsigned int)addr,(unsigned int)value,phys_addr);
                printf("MMU Map flash %08X to %08X\n",flash_addr,phys_addr);
                blk_pread(s->flash_blk, flash_addr, /*cache_page*/tmp_flash_cache, ESP32S2_CACHE_PAGE_SIZE);
                printf("%08X,%08X,b %08X,p %08X\n",*(uint32_t *)tmp_flash_cache,*(uint32_t *)&tmp_flash_cache[4],*(uint32_t *)&tmp_flash_cache[0x1000],*(uint32_t *)&tmp_flash_cache[0x8000]);
                cpu_physical_memory_write(phys_addr, tmp_flash_cache, ESP32S2_CACHE_PAGE_SIZE );
            }
        }
        break;
    }

}

static const MemoryRegionOps unimp_ops = {
    .read =  ESP32S2_unimp_read,
    .write = ESP32S2_unimp_write,
    .endianness = DEVICE_LITTLE_ENDIAN,
};

static void ESP32S2_unimp_reset(DeviceState *dev)
{ 
     Esp32UnimpState *s = ESP32S2_UNIMP(dev);

    for (int i = 0; i < ESP32S2_CACHE_PAGES_PER_REGION; ++i) {
        s->mmu_table[i] = ESP32S2_CACHE_MMU_ENTRY_CHANGED;
    }

}

static void ESP32S2_unimp_realize(DeviceState *dev, Error **errp)
{
}



static void ESP32S2_unimp_init(Object *obj)
{
    Esp32UnimpState *s = ESP32S2_UNIMP(obj);
    SysBusDevice *sbd = SYS_BUS_DEVICE(obj);

    memory_region_init_io(&s->iomem, obj, &unimp_ops, s,
                          TYPE_ESP32S2_MYUNIMP, 0x2000);
    sysbus_init_mmio(sbd, &s->iomem);
    sysbus_init_irq(sbd, &s->irq);
}

static Property ESP32S2_gpio_properties[] = {
    DEFINE_PROP_DRIVE("flash", Esp32DportState, flash_blk),
    DEFINE_PROP_END_OF_LIST(),
};

static void ESP32S2_unimp_class_init(ObjectClass *klass, void *data)
{
    DeviceClass *dc = DEVICE_CLASS(klass);

    dc->reset = ESP32S2_unimp_reset;
    dc->realize = ESP32S2_unimp_realize;
    device_class_set_props(dc,ESP32S2_gpio_properties);
}

static const TypeInfo ESP32S2_gpio_info = {
    .name = TYPE_ESP32S2_MYUNIMP,
    .parent = TYPE_SYS_BUS_DEVICE,
    .instance_size = sizeof(Esp32UnimpState),
    .instance_init = ESP32S2_unimp_init,
    .class_init = ESP32S2_unimp_class_init
};

static void ESP32S2_gpio_register_types(void)
{
    type_register_static(&ESP32S2_gpio_info);
}

type_init(ESP32S2_gpio_register_types)



/*************/


static void ESP32S2_machine_init_i2c(Esp32S2SocState *s)
{
    /* It should be possible to create an I2C device from the command line,
     * however for this to work the I2C bus must be reachable from sysbus-default.
     * At the moment the peripherals are added to an unrelated bus, to avoid being
     * reset on CPU reset.
     * If we find a way to decouple peripheral reset from sysbus reset,
     * we can move them to the sysbus and thus enable creation of i2c devices.
     */
    DeviceState *i2c_master = DEVICE(&s->i2c[0]);
    I2CBus* i2c_bus = I2C_BUS(qdev_get_child_bus(i2c_master, "i2c"));
    I2CSlave* tmp105 = i2c_slave_create_simple(i2c_bus, "tmp105", 0x48);
    object_property_set_int(OBJECT(tmp105), "temperature", 25 * 1000, &error_fatal);
}



static void ESP32S2_machine_init(MachineState *machine)
{
    Esp32S2SocState *s = g_new0(Esp32S2SocState, 1);

    BlockBackend* blk = NULL;
    DriveInfo *dinfo = drive_get_next(IF_MTD);
    if (dinfo) {
        qemu_log("Adding SPI flash device\n");
        blk = blk_by_legacy_dinfo(dinfo);
    } else {
        qemu_log("Not initializing SPI Flash\n");
    }

    qemu_log("Init SOC\n");

    Esp32S2MachineState *ms = ESP32S2_MACHINE(machine);
    object_initialize_child(OBJECT(ms), "soc", &ms->esp32S2, TYPE_ESP32S2_SOC);
    Esp32S2SocState *ss = ESP32S2_SOC(&ms->esp32S2);

    qemu_log("Done\n");

    if (blk) {
        s->myunimp.flash_blk = blk;
    }

    if (blk) {
        s->dport.flash_blk = blk;
    }
    qdev_prop_set_chr(DEVICE(s), "serial0", serial_hd(0));

    qdev_realize(DEVICE(ss),NULL, &error_fatal);

    if (blk) {
        ESP32S2_machine_init_spi_flash(machine, s, blk);
    }
    ESP32S2_machine_init_i2c(s);

    ESP32S2_machine_init_openeth(s);

    /* Need MMU initialized prior to ELF loading,
     * so that ELF gets loaded into virtual addresses
     */
    cpu_reset(CPU(&s->cpu[0]));
 
   /*
     MemoryRegion *system_memory = get_system_memory();
    // TODO, Figure out MMU mapping and related finetuning
      MemoryRegion *ram;
      ram = g_malloc(sizeof(*ram));
      memory_region_init_ram(ram, NULL, "esp32s2.iram0", 0x20000000,  // 00000
                           &error_abort);

      vmstate_register_ram_global(ram);
      memory_region_add_subregion(system_memory, 0x20000000, ram);
    */

    const char *load_elf_filename = NULL;
    if (machine->firmware) {
        load_elf_filename = machine->firmware;
    }
    if (machine->kernel_filename) {
        qemu_log("Loading elf.\n");
        load_elf_filename = machine->kernel_filename;
    }

    if (load_elf_filename) {
        uint64_t elf_entry;
        uint64_t elf_lowaddr;
        int success = load_elf(load_elf_filename, NULL,
                               translate_phys_addr, &s->cpu[0],
                               &elf_entry, &elf_lowaddr,
                               NULL, NULL, 0, EM_XTENSA, 0, 0);
        if (success > 0) {
            //s->cpu[0].env.pc = elf_entry;

            printf("Elf entry %08X\n",(unsigned int)elf_entry);
            static const uint8_t jx_a0[] = {
                0xa0, 0, 0,
            };            
            s->cpu[0].env.regs[0] = elf_entry;

            cpu_physical_memory_write(s->cpu[0].env.pc, jx_a0, sizeof(jx_a0));

        } else {
            qemu_log("Failed loading elf.\n"); 
            s->cpu[0].env.pc = elf_entry;
        }
    } else {
        /*
        char *rom_binary = qemu_find_file(QEMU_FILE_TYPE_BIOS, "esp32-v3-rom.bin");
        if (rom_binary == NULL) {
            error_report("Error: -bios argument not set, and ROM code binary not found");
            exit(1);
        }

        int size = load_image_targphys(rom_binary, esp32_memmap[ESP32_MEMREGION_IROM].base, esp32_memmap[ESP32_MEMREGION_IROM].size);
        if (size < 0) {
            error_report("Error: could not load ROM binary '%s'", rom_binary);
            exit(1);
        }
        g_free(rom_binary);
                */

    }
}

/* Initialize machine type */
//static void ESP32S2_machine_init(MachineClass *mc)
//{
//    mc->desc = "Espressif ESP32S2 machine";
//    mc->init = ESP32S2_machine_inst_init;
//    mc->max_cpus = 1;
//    mc->default_cpus = 1;
//}
//DEFINE_MACHINE("esp32s2", ESP32S2_machine_init)


/* Initialize machine type */
static void esp32s2_machine_class_init(ObjectClass *oc, void *data)
{
    MachineClass *mc = MACHINE_CLASS(oc);
    mc->desc = "Espressif ESP32S2 machine";
    mc->init = ESP32S2_machine_init;
    mc->max_cpus = 2;
    mc->default_cpus = 2;
}

static const TypeInfo esp32s2_info = {
    .name = TYPE_ESP32S2_MACHINE,
    .parent = TYPE_MACHINE,
    .instance_size = sizeof(Esp32S2MachineState),
    .class_init = esp32s2_machine_class_init,
};

static void esp32s2_machine_type_init(void)
{
    type_register_static(&esp32s2_info);
}

type_init(esp32s2_machine_type_init);