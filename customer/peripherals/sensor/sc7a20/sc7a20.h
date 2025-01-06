#ifndef __SC7A20_H
#define __SC7A20_H

#ifdef __cplusplus
extern "C" {
#endif
#include "board.h"

void sc7a20_write_reg(uint8_t reg, uint8_t data);
void sc7a20_read_reg(uint8_t reg, uint8_t *buf, uint8_t len);
int sc7a20_init(void);
void *sc7a20_get_bus(void);
uint8_t sc7a20_get_dev_addr(void);
uint8_t sc7a20_get_id(void);


#ifdef __cplusplus
}
#endif

#endif

