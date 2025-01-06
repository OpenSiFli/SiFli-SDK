

#ifndef __BLE_RF_RFC_H
#define __BLE_RF_RFC_H

//#include "rtdef.h"

#ifdef SF32LB55X
    #define RD(n)    (0x400+n)
    #define WR(n)    (0x800+n)
    #define AND(n)   (0xC00+n)
    #define OR(n)    (0x1000+n)
    #define WAIT(n)  (0x1400+n)
    #define END      (0x1C00)
#else
    #define RD(n)       (0x1800 + n)
    #define WR(n)       (0x2800 + n)
    #define AND(n)      (0x3000 + n)
    #define OR(n)       (0x4000 + n)
    #define WAIT(n)     (0x5000 + n)
    #define RD_FULCAL   (0x6000)
    #define RD_DCCAL1   (0x7000)
    #define RD_DCCAL2   (0x8000)
    #define END         (0xF000)
#endif

#define read_memory(addr)        (*(volatile unsigned int *)((addr)))
#define write_memory(memory, reg) *((volatile uint32_t *)(memory)) = (reg);
#define write_hmemory(addr,value) (*(volatile uint16_t *)((addr))) = (value)

extern void bt_rf_cal(void);

#endif
