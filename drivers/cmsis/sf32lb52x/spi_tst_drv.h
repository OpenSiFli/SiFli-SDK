#include <string.h>             // for memcpy
//#include <rtthread.h>
//#include <rtdevice.h>
#include <stdlib.h>
#include <board.h>

void enable_spi3_trx() ;

void disable_spi3_trx() ;

void set_spi3_tdata(uint32_t tdata) ;

uint32_t get_spi3_rdata() ;

void set_spi3_frm_width(uint8_t frm_width) ;

void set_spi3_sph() ;

void clear_spi3_sph();

void set_spi3_spo();

void clear_spi3_spo();

void set_spi3_clkdiv(uint8_t div);

void enable_spi4_trx() ;

void disable_spi4_trx() ;

void set_spi4_tdata(uint32_t tdata) ;

uint32_t get_spi4_rdata() ;

void set_spi4_frm_width(uint8_t frm_width) ;

void set_spi4_sph() ;

void clear_spi4_sph();

void set_spi4_spo();

void clear_spi4_spo();

void set_spi4_clkdiv(uint8_t div);
