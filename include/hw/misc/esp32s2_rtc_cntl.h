#pragma once

#include "hw/hw.h"
#include "hw/sysbus.h"
#include "hw/registerfields.h"
#include "target/xtensa/cpu-qom.h"
#include "hw/misc/esp32s2_reg.h"
#define TYPE_ESP32S2_RTC_CNTL "misc.esp32s2.rtc_cntl"
#define ESP32S2_RTC_CNTL(obj) OBJECT_CHECK(Esp32S2RtcCntlState, (obj), TYPE_ESP32S2_RTC_CNTL)

#define ESP32S2_RTC_DIG_RESET_GPIO    "dig-reset"
#define ESP32S2_RTC_CPU_RESET_GPIO    "cpu-reset"
#define ESP32S2_RTC_CPU_STALL_GPIO    "cpu-stall"
#define ESP32S2_RTC_CLK_UPDATE_GPIO   "clk-update"

typedef enum Esp32S2ResetCause {
    ESP32S2_POWERON_RESET = 1,
    ESP32S2_SW_SYS_RESET = 3,
    ESP32S2_OWDT_RESET = 4,
    ESP32S2_DEEPSLEEP_RESET = 5,
    ESP32S2_SDIO_RESET = 6,
    ESP32S2_TG0WDT_SYS_RESET = 7,
    ESP32S2_TG1WDT_SYS_RESET = 8,
    ESP32S2_RTCWDT_SYS_RESET = 9,
    ESP32S2_TGWDT_CPU_RESET = 11,
    ESP32S2_SW_CPU_RESET = 12,
    ESP32S2_RTCWDT_CPU_RESET = 13,
    ESP32S2_EXT_CPU_RESET = 14,
    ESP32S2_RTCWDT_BROWN_OUT_RESET = 15,
    ESP32S2_RTCWDT_RTC_RESET = 16
} Esp32S2ResetCause;

typedef enum Esp32S2SocClkSel {
    ESP32S2_SOC_CLK_XTAL = 0,
    ESP32S2_SOC_CLK_PLL = 1,
    ESP32S2_SOC_CLK_8M = 2,
    ESP32S2_SOC_CLK_APLL = 3
} Esp32S2SocClkSel;

typedef enum Esp32S2FastClkSel {
    ESP32S2_FAST_CLK_XTALD4 = 0,
    ESP32S2_FAST_CLK_8M = 1
} Esp32S2FastClkSel;

typedef enum Esp32S2SlowClkSel {
    ESP32S2_SLOW_CLK_RC = 0,
    ESP32S2_SLOW_CLK_32KXTAL = 1,
    ESP32S2_SLOW_CLK_8MD256 = 2
} Esp32S2SlowClkSel;

typedef struct Esp32S2RtcCntlState {
    SysBusDevice parent_obj;

    MemoryRegion iomem;
    qemu_irq irq;
    qemu_irq dig_reset_req;
    qemu_irq cpu_reset_req[ESP32S2_CPU_COUNT];
    qemu_irq cpu_stall_req[ESP32S2_CPU_COUNT];
    qemu_irq clk_update;
    bool cpu_stall_state[ESP32S2_CPU_COUNT];

    uint32_t xtal_apb_freq;
    uint32_t pll_apb_freq;
    Esp32S2SocClkSel soc_clk;
    Esp32S2FastClkSel rtc_fastclk;
    uint32_t rtc_fastclk_freq;
    Esp32S2SlowClkSel rtc_slowclk;
    uint32_t rtc_slowclk_freq;
    int64_t time_base_ns;

    uint32_t options0_reg;
    uint64_t time_reg;
    uint32_t sw_cpu_stall_reg;
    uint32_t scratch_reg[ESP32S2_RTC_CNTL_SCRATCH_REG_COUNT];
    Esp32S2ResetCause reset_cause[ESP32S2_CPU_COUNT];
    bool stat_vector_sel[ESP32S2_CPU_COUNT];
} Esp32S2RtcCntlState;

REG32(RTC_CNTL_OPTIONS0, 0x00)
    FIELD(RTC_CNTL_OPTIONS0, SW_SYS_RESET, 31, 1)
    FIELD(RTC_CNTL_OPTIONS0, SW_PROCPU_RESET, 5, 1)
    FIELD(RTC_CNTL_OPTIONS0, SW_APPCPU_RESET, 4, 1)
    FIELD(RTC_CNTL_OPTIONS0, SW_STALL_PROCPU_C0, 2, 2)
    FIELD(RTC_CNTL_OPTIONS0, SW_STALL_APPCPU_C0, 0, 2)

REG32(RTC_CNTL_TIME_UPDATE, 0xc)
    FIELD(RTC_CNTL_TIME_UPDATE, UPDATE, 31, 1)
    FIELD(RTC_CNTL_TIME_UPDATE, VALID, 30, 1)
REG32(RTC_CNTL_TIME0, 0x10)
REG32(RTC_CNTL_TIME1, 0x14)

REG32(RTC_CNTL_RESET_STATE, 0x34)
    FIELD(RTC_CNTL_RESET_STATE, PROCPU_STAT_VECTOR_SEL, 13, 1)
    FIELD(RTC_CNTL_RESET_STATE, APPCPU_STAT_VECTOR_SEL, 12, 1)
    FIELD(RTC_CNTL_RESET_STATE, RESET_CAUSE_APPCPU, 6, 6)
    FIELD(RTC_CNTL_RESET_STATE, RESET_CAUSE_PROCPU, 0, 6)

REG32(RTC_CNTL_STORE0, 0x4c)
REG32(RTC_CNTL_STORE1, 0x50)
REG32(RTC_CNTL_STORE2, 0x54)
REG32(RTC_CNTL_STORE3, 0x58)

REG32(RTC_CNTL_CLK_CONF, 0x70)
    FIELD(RTC_CNTL_CLK_CONF, ANA_CLK_RTC_SEL, 30, 2)
    FIELD(RTC_CNTL_CLK_CONF, FAST_CLK_RTC_SEL, 29, 1)
    FIELD(RTC_CNTL_CLK_CONF, SOC_CLK_SEL, 27, 2)

REG32(RTC_CNTL_SW_CPU_STALL, 0xac)
    FIELD(RTC_CNTL_SW_CPU_STALL, PROCPU_C1, 26, 6)
    FIELD(RTC_CNTL_SW_CPU_STALL, APPCPU_C1, 20, 6)

REG32(RTC_CNTL_STORE4, 0xb0)
REG32(RTC_CNTL_STORE5, 0xb4)
REG32(RTC_CNTL_STORE6, 0xb8)
REG32(RTC_CNTL_STORE7, 0xbc)
REG32(RTC_CNTL_DATE,   0x13c)

#define ESP32S2_RTC_CNTL_SIZE (A_RTC_CNTL_DATE + 4)
