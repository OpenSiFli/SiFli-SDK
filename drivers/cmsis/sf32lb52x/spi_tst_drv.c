//#include "cpu_tst_drv.h"
#include "spi_tst_drv.h"
#include "register.h"
#include "board.h"
void enable_spi3_trx()
{
    hwp_spi1->TOP_CTRL |= SPI_TOP_CTRL_SSE;
    set_spi3_clkdiv(0x4);
    hwp_spi1->CLK_CTRL |= 0x01 << SPI_CLK_CTRL_CLK_SSP_EN_Pos;
}

void disable_spi3_trx()
{
    hwp_spi1->TOP_CTRL &= ~SPI_TOP_CTRL_SSE_Msk;
}

void set_spi3_tdata(uint32_t tdata)
{
    hwp_spi1->DATA = tdata ;
}

uint32_t get_spi3_rdata()
{
    return hwp_spi1->DATA;
}

void set_spi3_frm_width(uint8_t frm_width)
{
    hwp_spi1->TOP_CTRL &= ~SPI_TOP_CTRL_DSS_Msk;
    hwp_spi1->TOP_CTRL |= ((frm_width - 1) << SPI_TOP_CTRL_DSS_Pos);
}

void set_spi3_sph()
{
    hwp_spi1->TOP_CTRL |= SPI_TOP_CTRL_SPH;
}

void clear_spi3_sph()
{
    hwp_spi1->TOP_CTRL &= ~SPI_TOP_CTRL_SPH_Msk;
}

void set_spi3_spo()
{
    hwp_spi1->TOP_CTRL |= SPI_TOP_CTRL_SPO;

}

void clear_spi3_spo()
{
    hwp_spi1->TOP_CTRL &= ~SPI_TOP_CTRL_SPO_Msk;

}

void set_spi3_clkdiv(uint8_t div)
{
    hwp_spi1->CLK_CTRL &= (~SPI_CLK_CTRL_CLK_DIV_Msk);
    hwp_spi1->CLK_CTRL |= (div << SPI_CLK_CTRL_CLK_DIV_Pos);
}

#if 0
void enable_spi4_trx()
{
    hwp_spi4->TOP_CTRL |= SPI_TOP_CTRL_SSE;
}

void disable_spi4_trx()
{
    hwp_spi4->TOP_CTRL &= ~SPI_TOP_CTRL_SSE_Msk;
}

void set_spi4_tdata(uint32_t tdata)
{
    hwp_spi4->DATA = tdata ;
}

uint32_t get_spi4_rdata()
{
    return hwp_spi4->DATA;
}

void set_spi4_frm_width(uint8_t frm_width)
{
    hwp_spi4->TOP_CTRL &= ~SPI_TOP_CTRL_DSS_Msk;
    hwp_spi4->TOP_CTRL |= ((frm_width - 1) << SPI_TOP_CTRL_DSS_Pos);
}

void set_spi4_sph()
{
    hwp_spi4->TOP_CTRL |= SPI_TOP_CTRL_SPH;
}

void clear_spi4_sph()
{
    hwp_spi4->TOP_CTRL &= ~SPI_TOP_CTRL_SPH_Msk;
}

void set_spi4_spo()
{
    hwp_spi4->TOP_CTRL |= SPI_TOP_CTRL_SPO;

}

void clear_spi4_spo()
{
    hwp_spi4->TOP_CTRL &= ~SPI_TOP_CTRL_SPO_Msk;

}

void set_spi4_clkdiv(uint8_t div)
{
    hwp_spi4->CLK_CTRL &= (~SPI_CLK_CTRL_CLK_DIV_Msk);
    hwp_spi4->CLK_CTRL |= (div << SPI_CLK_CTRL_CLK_DIV_Pos);
}
#endif
