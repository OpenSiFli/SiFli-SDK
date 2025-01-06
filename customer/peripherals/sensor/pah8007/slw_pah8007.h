/**
  ******************************************************************************
  * @file   slw_sc7a20.h
  * @author wk software development team
  ******************************************************************************
*/


#ifndef SLW_SC7A20_H__
#define SLW_SC7A20_H__

#include "board.h"
#include "sensor.h"


int pah8007_init(void);
void *pah8007_get_bus(void);
uint8_t pah8007_get_dev_addr(void);


unsigned char SL_PAH8007_I2c_Spi_Write(unsigned char reg, unsigned char data);
unsigned char SL_PAH8007_I2c_Spi_Read(unsigned char reg, unsigned char len, unsigned char *buf);

#endif  // SLW_SC7A20_H__
/************************ (C) COPYRIGHT Sifli Technology *******END OF FILE****/
