#pragma once

#include "hw/sysbus.h"
#include "hw/hw.h"
#include "hw/registerfields.h"

// <baseAddress>0x3f423000</baseAddress>
//    <baseAddress>0x60023000</baseAddress>

#define TYPE_ESP32S2_SYSTIMER "esp32s2.systimer"
#define ESP32_SYSTIMER(obj) OBJECT_CHECK(Esp32S2SysTimerState, (obj), TYPE_ESP32S2_SYSTIMER)

REG32(SYSTIMER_CONF, 0x0000)
REG32(SYSTIMER_LOAD, 0x0004)
REG32(SYSTIMER_LOAD_HI, 0x0008)
REG32(SYSTIMER_LOAD_LO, 0x000c)
REG32(SYSTIMER_STEP, 0x0010)
REG32(SYSTIMER_TARGETO_HI, 0x0014)
REG32(SYSTIMER_INT_CLR, 0x004c)
REG32(SYSTIMER_UPDATE, 0x0038)

/*
          <name></name>
          <addressOffset>0x00000014</addressOffset>
          <description>
          </description>
          <size>32</size>
          <resetValue>0x00000000</resetValue>
          <fields />
        </register>
        <register>
          <name>SYSTIMER_TARGETO_LO</name>
          <addressOffset>0x00000018</addressOffset>
          <description>
          </description>
          <size>32</size>
          <resetValue>0x00000000</resetValue>
          <fields />
        </register>
        <register>
          <name>SYSTIMER_TARGET1_HI</name>
          <addressOffset>0x0000001c</addressOffset>
          <description>
          </description>
          <size>32</size>
          <resetValue>0x00000000</resetValue>
          <fields />
        </register>
        <register>
          <name>SYSTIMER_TARGET1_LO</name>
          <addressOffset>0x00000020</addressOffset>
          <description>
          </description>
          <size>32</size>
          <resetValue>0x00000000</resetValue>
          <fields />
        </register>
        <register>
          <name>SYSTIMER_TARGET2_HI</name>
          <addressOffset>0x00000024</addressOffset>
          <description>
          </description>
          <size>32</size>
          <resetValue>0x00000000</resetValue>
          <fields />
        </register>
        <register>
          <name>SYSTIMER_TARGET2_LO</name>
          <addressOffset>0x00000028</addressOffset>
          <description>
          </description>
          <size>32</size>
          <resetValue>0x00000000</resetValue>
          <fields />
        </register>
        <register>
          <name>SYSTIMER_TARGETO_CONF</name>
          <addressOffset>0x0000002c</addressOffset>
          <description>
          </description>
          <size>32</size>
          <resetValue>0x00000000</resetValue>
          <fields />
        </register>
        <register>
          <name>SYSTIMER_TARGET1_CONF</name>
          <addressOffset>0x00000030</addressOffset>
          <description>
          </description>
          <size>32</size>
          <resetValue>0x00000000</resetValue>
          <fields />
        </register>
        <register>
          <name>SYSTIMER_TARGET2_CONF</name>
          <addressOffset>0x00000034</addressOffset>
          <description>
          </description>
          <size>32</size>
          <resetValue>0x00000000</resetValue>
          <fields />
        </register>
        <register>
          <name>SYSTIMER_UPDATE</name>
          <addressOffset>0x00000038</addressOffset>
          <description>
          </description>
          <size>32</size>
          <resetValue>0x00000000</resetValue>
          <fields />
        </register>
        <register>
          <name>SYSTIMER_VALUE_HI</name>
          <addressOffset>0x0000003c</addressOffset>
          <description>
          </description>
          <size>32</size>
          <resetValue>0x00000000</resetValue>
          <fields />
        </register>
        <register>
          <name>SYSTIMER_VALUE_LO</name>
          <addressOffset>0x00000040</addressOffset>
          <description>
          </description>
          <size>32</size>
          <resetValue>0x00000000</resetValue>
          <fields />
        </register>
        <register>
          <name>SYSTIMER_INT_ENA</name>
          <addressOffset>0x00000044</addressOffset>
          <description>
          </description>
          <size>32</size>
          <resetValue>0x00000000</resetValue>
          <fields />
        </register>
        <register>
          <name>SYSTIMER_INT_RAW</name>
          <addressOffset>0x00000048</addressOffset>
          <description>
          </description>
          <size>32</size>
          <resetValue>0x00000000</resetValue>
          <fields />
        </register>
        <register>
          <name>SYSTIMER_INT_CLR</name>
          <addressOffset>0x0000004c</addressOffset>
          <description>
          </description>
          <size>32</size>
          <resetValue>0x00000000</resetValue>
          <fields />
        </register>
      </registers>
    </peripheral>
    <peripheral>
      <name>SYSTIMER_B</name>
*/


// 0x12

typedef struct Esp32S2SysTimerState {
    SysBusDevice parent_obj;

    MemoryRegion iomem;
    qemu_irq irq;
    uint32_t systimer_mode;
} Esp32S2SysTimerState;

