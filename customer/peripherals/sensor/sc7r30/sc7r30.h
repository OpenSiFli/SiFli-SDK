#ifndef __SC7R30_H
#define __SC7R30_H

#ifdef __cplusplus
extern "C" {
#endif
#include "board.h"


void sc7r30_write_reg(uint8_t reg, uint8_t data);
void sc7r30_read_reg(uint8_t reg, uint8_t *buf, uint8_t len);
int sc7r30_init(void);
void *sc7r30_get_bus(void);
uint8_t sc7r30_get_dev_addr(void);
uint8_t sc7r30_get_id(void);

#ifdef __cplusplus
}
#endif

#endif

