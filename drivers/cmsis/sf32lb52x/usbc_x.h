/**
  ******************************************************************************
  * @file   usbc_x.h
  * @author Sifli software development team
  ******************************************************************************
*/
/**
 * Copyright (c) 2019 - 2022,  Sifli Technology
 *
 * All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without modification,
 * are permitted provided that the following conditions are met:
 *
 * 1. Redistributions of source code must retain the above copyright notice, this
 *    list of conditions and the following disclaimer.
 *
 * 2. Redistributions in binary form, except as embedded into a Sifli integrated circuit
 *    in a product or a software update for such product, must reproduce the above
 *    copyright notice, this list of conditions and the following disclaimer in the
 *    documentation and/or other materials provided with the distribution.
 *
 * 3. Neither the name of Sifli nor the names of its contributors may be used to endorse
 *    or promote products derived from this software without specific prior written permission.
 *
 * 4. This software, with or without modification, must only be used with a
 *    Sifli integrated circuit.
 *
 * 5. Any software provided in binary form under this license must not be reverse
 *    engineered, decompiled, modified and/or disassembled.
 *
 * THIS SOFTWARE IS PROVIDED BY SIFLI TECHNOLOGY "AS IS" AND ANY EXPRESS
 * OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED WARRANTIES
 * OF MERCHANTABILITY, NONINFRINGEMENT, AND FITNESS FOR A PARTICULAR PURPOSE ARE
 * DISCLAIMED. IN NO EVENT SHALL SIFLI TECHNOLOGY OR CONTRIBUTORS BE
 * LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
 * CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE
 * GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION)
 * HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
 * LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT
 * OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 *
 */

#ifndef _USBC_X_H_
#define _USBC_X_H_

#define REG8   __IO uint8_t
#define REG16  __IO uint16_t
#define REG32  __IO uint32_t

// =============================================================================
//  MACROS
// =============================================================================
#define SPFIFORAM_SIZE                               (264)
#define DIEP_NUM                                     (3)
#define DOEP_NUM                                     (2)

// =============================================================================
//  TYPES
// =============================================================================

// ============================================================================
// USBC_T
// -----------------------------------------------------------------------------
///
// =============================================================================

/* EP0 */
struct musb_ep0_regs
{
    REG16   reserved4;
    REG16   csr0;
    REG16   reserved5;
    REG16   reserved6;
    REG16   count0;
    REG8    host_type0;
    REG8    host_naklimit0;
    REG8    reserved7;
    REG8    reserved8;
    REG8    reserved9;
    REG8    configdata;
};


/* EP 1-15 */
struct musb_epN_regs
{
    REG16   txmaxp;
    REG16   txcsr;
    REG16   rxmaxp;
    REG16   rxcsr;
    REG16   rxcount;
    REG8    txtype;
    REG8    txinterval;
    REG8    rxtype;
    REG8    rxinterval;
    REG8    reserved0;
    REG8    fifosize;
};

typedef volatile struct
{
    /* common registers */
    REG8    faddr;                          //0x0000
    REG8    power;                          //0x0001
    REG16   intrtx;                         //0x0002
    REG16   intrrx;                         //0x0004
    REG16   intrtxe;                        //0x0006
    REG16   intrrxe;                        //0x0008
    REG8    intrusb;                        //0x000a
    REG8    intrusbe;                       //0x000b
    REG16   frame;                          //0x000c
    REG8    index;                          //0x000e
    REG8    testmode;                       //0x000f
    /* indexed registers */
    REG16   txmaxp;                         //0x0010
    REG16   csr0_txcsr;                     //0x0012
    REG16   rxmaxp;                         //0x0014
    REG16   rxcsr;                          //0x0016
    REG16   rxcount;                        //0x0018
    REG8    txtype;                         //0x001a
    REG8    txinterval;                     //0x001b
    REG8    rxtype;                         //0x001c
    REG8    rxinterval;                     //0x001d
    REG8    reserved0;                      //0x001e
    REG8    cfdt_fifosz;                    //0x001f
    /* fifo */
    REG32    fifox[0x10];                   //0x0020
    /* OTG, dynamic FIFO, version & vendor registers */
    REG8    devctl;                         //0x0060
    REG8    reserved1;                      //0x0061
    REG8    txfifosz;                       //0x0062
    REG8    rxfifosz;                       //0x0063
    REG16    txfifoadd;                     //0x0064
    REG16    rxfifoadd;                     //0x0066
    REG32    vcontrol;                      //0x0068
    REG16    hwvers;                        //0x006c
    REG16    reserved2a[1];                 //0x006e
    REG8    ulpi_busctl;                    //0x0070
    REG8    reserved2b[1];                  //0x0071
    REG16    reserved2[3];                  //0x0072
    REG8    epinfo;                         //0x0078
    REG8    raminfo;                        //0x0079
    REG8    linkinfo;                       //0x007a
    REG8    vplen;                          //0x007b
    REG8    hseof1;                         //0x007c
    REG8    fseof1;                         //0x007d
    REG8    lseof1;                         //0x007e
    REG8    soft_rst;                       //0x007f
    /* target address registers */          //0x0080
    struct musb_tar_regs
    {
        REG8    txfuncaddr;
        REG8    reserved0;
        REG8    txhubaddr;
        REG8    txhubport;
        REG8    rxfuncaddr;
        REG8    reserved1;
        REG8    rxhubaddr;
        REG8    rxhubport;
    } tar[0x10];
    /*
     * endpoint registers
     * ep0 elements are valid when array index is 0
     * otherwise epN is valid
     */
    union musb_ep_regs                      //0x0100
    {
        struct musb_ep0_regs ep0;
        struct musb_epN_regs epN;
    } ep[0x10];

    //REG32 reserved1fe;                    //0x0200
    REG32 dmaintr;

    struct musb_dma_regs
    {
        REG32    cntl;                      //0x0204
        REG32    addr;                      //0x0208
        REG32    count;                     //0x020c
        REG32    rsvd;                      //0x0210
    } dma[0x10];


    struct musb_reserved_1
    {
        REG32  reserved_1;                  //0x0304
    } rsvd_1[0xf];

    REG8 dpbrxdisl;                         //0x0340
    REG8 dpbrxdish;                         //0x0341
    REG8 dpbtxdisl;                         //0x0342
    REG8 dpbtxdish;                         //0x0343

    struct musb_reserved_2
    {
        REG32 reserved_2;                      //0x0344
    } rsvd_2[0x9];

    REG8    dbgl;                           //0x0368
    REG8    dbgh;                           //0x0369
    REG16   reserved_0;                     //0x036a
    REG32   reserved_1;                     //0x036c
    REG8    usbcfg;                         //0x0370

    struct musb_reserved_3
    {
        REG8 reserved_3;                    //0x0371~0x3bb
    } rsvd_3[0x4b];

    REG8     dbgcntl0;                     //0x3bc
    REG8     dbgcntl1;                     //0x3bd
} __attribute__((packed, aligned(32))) USBC_X_Typedef;



#define BIT0                                (0x0001)
#define BIT1                                (0x0002)
#define BIT2                                (0x0004)
#define BIT3                                (0x0008)
#define BIT4                                (0x0010)
#define BIT5                                (0x0020)
#define BIT6                                (0x0040)
#define BIT7                                (0x0080)


/* CSR0 */
#define USB_CSR0_FLUSHFIFO                  (0x0100)
#define USB_CSR0_TXPKTRDY                   (0x0002)
#define USB_CSR0_RXPKTRDY                   (0x0001)

/* CSR0 in Peripheral mode */
#define USB_CSR0_P_SVDSETUPEND              (0x0080)
#define USB_CSR0_P_SVDRXPKTRDY              (0x0040)
#define USB_CSR0_P_SENDSTALL                (0x0020)
#define USB_CSR0_P_SETUPEND                 (0x0010)
#define USB_CSR0_P_DATAEND                  (0x0008)
#define USB_CSR0_P_SENTSTALL                (0x0004)

/* CSR0 in Host mode */
#define USB_CSR0_H_DIS_PING                 (0x0800)
#define USB_CSR0_H_WR_DATATOGGLE            (0x0400)  /* Set to allow setting: */
#define USB_CSR0_H_DATATOGGLE               (0x0200)  /* Data toggle control */
#define USB_CSR0_H_NAKTIMEOUT               (0x0080)
#define USB_CSR0_H_STATUSPKT                (0x0040)
#define USB_CSR0_H_REQPKT                   (0x0020)
#define USB_CSR0_H_ERROR                    (0x0010)
#define USB_CSR0_H_SETUPPKT                 (0x0008)
#define USB_CSR0_H_RXSTALL                  (0x0004)


/* DEVCTL */
#define USB_DEVCTL_BDEVICE                  0x80
#define USB_DEVCTL_FSDEV                    0x40
#define USB_DEVCTL_LSDEV                    0x20
#define USB_DEVCTL_VBUS                     0x18
#define USB_DEVCTL_VBUS_SHIFT               3
#define USB_DEVCTL_HM                       0x04
#define USB_DEVCTL_HR                       0x02
#define USB_DEVCTL_SESSION                  0x01

/* ULPI VBUSCONTROL */
#define ULPI_USE_EXTVBUS                    0x01
#define ULPI_USE_EXTVBUSIND                 0x02

/* TESTMODE */
#define USB_TEST_FORCE_HOST                 0x80
#define USB_TEST_FIFO_ACCESS                0x40
#define USB_TEST_FORCE_FS                   0x20
#define USB_TEST_FORCE_HS                   0x10
#define USB_TEST_PACKET                     0x08
#define USB_TEST_K                          0x04
#define USB_TEST_J                          0x02
#define USB_TEST_SE0_NAK                    0x01

/* Allocate for double-packet buffering (effectively doubles assigned _SIZE) */
#define USB_FIFOSZ_DPB                      0x10
/* Allocation size (8, 16, 32, ... 4096) */
#define USB_FIFOSZ_SIZE                     0x0f




/* TxType/RxType */
#define USB_TYPE_SPEED                      0xc0
#define USB_TYPE_SPEED_SHIFT                6
#define USB_TYPE_SPEED_HIGH                 1
#define USB_TYPE_SPEED_FULL                 2
#define USB_TYPE_SPEED_LOW                  3
#define USB_TYPE_PROTO                      0x30    /* Implicitly zero for ep0 */
#define USB_TYPE_PROTO_SHIFT                4
#define USB_TYPE_PROTO_CTRL                 0
#define USB_TYPE_PROTO_ISO                  1
#define USB_TYPE_PROTO_BULK                 2
#define USB_TYPE_PROTO_INTR                 3
#define USB_TYPE_REMOTE_END                 0xf     /* Implicitly zero for ep0 */

/* CONFIGDATA */
#define USB_CONFIGDATA_MPRXE                0x80    /* Auto bulk pkt combining */
#define USB_CONFIGDATA_MPTXE                0x40    /* Auto bulk pkt splitting */
#define USB_CONFIGDATA_BIGENDIAN            0x20    /* Always 0 indicate Little Endian ordering.*/
#define USB_CONFIGDATA_HBRXE                0x10    /* HB-ISO for RX */
#define USB_CONFIGDATA_HBTXE                0x08    /* HB-ISO for TX */
#define USB_CONFIGDATA_DYNFIFO              0x04    /* Dynamic FIFO sizing */
#define USB_CONFIGDATA_SOFTCONE             0x02    /* SoftConnect */
#define USB_CONFIGDATA_UTMIDW               0x01    /* Data width 0/1 => 8/16bits */

#define USB_POWER_ISOUPDATE                 (BIT7)
#define USB_POWER_SOFTCONN                  (BIT6)
#define USB_POWER_HSENAB                    (BIT5)
#define USB_POWER_HSMODE                    (BIT4)
#define USB_POWER_RESET                     (BIT3)
#define USB_POWER_RESUME                    (BIT2)
#define USB_POWER_SUSPENDM                  (BIT1)
#define USB_POWER_ENSUSPEND                 (BIT0)

#define USB_INTR_VBUSERROR                  (BIT7)
#define USB_INTR_SESSREQ                    (BIT6)
#define USB_INTR_DISCONNECT                 (BIT5)
#define USB_INTR_CONNECT                    (BIT4)
#define USB_INTR_SOF                        (BIT3)
#define USB_INTR_BABBLE                     (BIT2)
#define USB_INTR_RESET                      (BIT2)
#define USB_INTR_RESUME                     (BIT1)
#define USB_INTR_SUSPEND                    (BIT0)

/* TXCSR in Peripheral and Host mode */
#define USB_TXCSR_AUTOSET                   (0x8000)
#define USB_TXCSR_MODE                      (0x2000)
#define USB_TXCSR_DMAENAB                   (0x1000)
#define USB_TXCSR_FRCDATATOG                (0x0800)
#define USB_TXCSR_DMAMODE                   (0x0400)
#define USB_TXCSR_CLRDATATOG                (0x0040)
#define USB_TXCSR_FLUSHFIFO                 (0x0008)
#define USB_TXCSR_FIFONOTEMPTY              (0x0002)
#define USB_TXCSR_TXPKTRDY                  (0x0001)


/* TXCSR in Peripheral mode */
#define USB_TXCSR_P_ISO                     (0x4000)
#define USB_TXCSR_P_INCOMPTX                (0x0080)
#define USB_TXCSR_P_SENTSTALL               (0x0020)
#define USB_TXCSR_P_SENDSTALL               (0x0010)
#define USB_TXCSR_P_UNDERRUN                (0x0004)

/* TXCSR in Host mode */
#define USB_TXCSR_H_WR_DATATOGGLE           (0x0200)
#define USB_TXCSR_H_DATATOGGLE              (0x0100)
#define USB_TXCSR_H_NAKTIMEOUT              (0x0080)
#define USB_TXCSR_H_RXSTALL                 (0x0020)
#define USB_TXCSR_H_ERROR                   (0x0004)


#define USB_MAX_EP_NUM                      (16)

#define USB_MAX_EP_USED_NUM                 (2)

/* HUBADDR */
#define USB_HUBADDR_MULTI_TT                0x80

//Usb intr mask
#define USBC_EP0_INTR_MASK                  (1<<0)

/* RXCSR in Peripheral and Host mode */
#define USB_RXCSR_AUTOCLEAR                 (0x8000)
#define USB_RXCSR_DMAENAB                   (0x2000)
#define USB_RXCSR_DISNYET                   (0x1000)
#define USB_RXCSR_PID_ERR                   (0x1000)
#define USB_RXCSR_DMAMODE                   (0x0800)
#define USB_RXCSR_INCOMPRX                  (0x0100)
#define USB_RXCSR_CLRDATATOG                (0x0080)
#define USB_RXCSR_FLUSHFIFO                 (0x0010)
#define USB_RXCSR_DATAERROR                 (0x0008)
#define USB_RXCSR_FIFOFULL                  (0x0002)
#define USB_RXCSR_RXPKTRDY                  (0x0001)

/* RXCSR in Peripheral mode */
#define USB_RXCSR_P_ISO                     (0x4000)
#define USB_RXCSR_P_SENTSTALL               (0x0040)
#define USB_RXCSR_P_SENDSTALL               (0x0020)
#define USB_RXCSR_P_OVERRUN                 (0x0004)

/* RXCSR in Host mode */
#define USB_RXCSR_H_AUTOREQ                 (0x4000)
#define USB_RXCSR_H_WR_DATATOGGLE           (0x0400)
#define USB_RXCSR_H_DATATOGGLE              (0x0200)
#define USB_RXCSR_H_RXSTALL                 (0x0040)
#define USB_RXCSR_H_REQPKT                  (0x0020)
#define USB_RXCSR_H_ERROR                   (0x0004)




/* DMA channel number            0 ~ 8    */
#define USB_DMA_CHANNEL1                    1
#define USB_DMA_CHANNEL2                    2
#define USB_DMA_CHANNEL3                    3
#define USB_DMA_CHANNEL4                    4
#define USB_DMA_CHANNEL5                    5
#define USB_DMA_CHANNEL6                    6
#define USB_DMA_CHANNEL7                    7
#define USB_DMA_CHANNEL8                    8

/* Control register (16-bit) */
#define USB_DMACTRL_ENABLE_SHIFT            (0)
#define USB_DMACTRL_TRANSMIT_SHIFT          (1)
#define USB_DMACTRL_MODE1_SHIFT             (2)
#define USB_DMACTRL_IRQENABLE_SHIFT         (3)
#define USB_DMACTRL_ENDPOINT_SHIFT          (4)
#define USB_DMACTRL_BUSERROR_SHIFT          (8)
#define USB_DMACTRL_BURSTMODE_SHIFT         (9)
#define USB_DMACTRL_BURSTMODE               (3 << USB_DMACTRL_BURSTMODE_SHIFT)
#define USB_DMACTRL_BURSTMODE_UNSPEC        (0)
#define USB_DMACTRL_BURSTMODE_INCR4         (1)
#define USB_DMACTRL_BURSTMODE_INCR8         (2)
#define USB_DMACTRL_BURSTMODE_INCR16        (3)

/*Indicates pending DMA interrupts, one bit per DMA channel implemented*/
#define USB_DMAINTR_MASK_CHAN1              (1<<0)
#define USB_DMAINTR_MASK_CHAN2              (1<<1)
#define USB_DMAINTR_MASK_CHAN3              (1<<2)
#define USB_DMAINTR_MASK_CHAN4              (1<<3)
#define USB_DMAINTR_MASK_CHAN5              (1<<4)
#define USB_DMAINTR_MASK_CHAN6              (1<<5)
#define USB_DMAINTR_MASK_CHAN7              (1<<6)
#define USB_DMAINTR_MASK_CHAN8              (1<<7)



#define USB_DMAADDR(n)                      (((n)&0x0FFFFFFF)<<0)

/* CSR0 bits to avoid zeroing (write zero clears, write 1 ignored) */
#define USB_CSR0_P_WZC_BITS    \
    (USB_CSR0_P_SENTSTALL)

#define USB_CSR0_H_WZC_BITS    \
    (USB_CSR0_H_NAKTIMEOUT | USB_CSR0_H_RXSTALL \
    | USB_CSR0_RXPKTRDY)


/* TXCSR bits to avoid zeroing (write zero clears, write 1 ignored) */
#define USB_TXCSR_P_WZC_BITS    \
    (USB_TXCSR_P_INCOMPTX | USB_TXCSR_P_SENTSTALL \
    | USB_TXCSR_P_UNDERRUN | USB_TXCSR_FIFONOTEMPTY)
#define USB_TXCSR_H_WZC_BITS    \
    (USB_TXCSR_H_NAKTIMEOUT | USB_TXCSR_H_RXSTALL \
    | USB_TXCSR_H_ERROR | USB_TXCSR_FIFONOTEMPTY)


/* RXCSR bits to avoid zeroing (write zero clears, write 1 ignored) */
#define USB_RXCSR_P_WZC_BITS    \
    (USB_RXCSR_P_SENTSTALL | USB_RXCSR_P_OVERRUN \
    | USB_RXCSR_RXPKTRDY)
#define USB_RXCSR_H_WZC_BITS    \
    (USB_RXCSR_H_RXSTALL | USB_RXCSR_H_ERROR \
    | USB_RXCSR_DATAERROR | USB_RXCSR_RXPKTRDY)


/* RXCSR bits to avoid zeroing (write zero clears, write 1 ignored) */
#define USB_RXCSR_P_WZC_BITS    \
    (USB_RXCSR_P_SENTSTALL | USB_RXCSR_P_OVERRUN \
    | USB_RXCSR_RXPKTRDY)

#define USB_RXCSR_H_WZC_BITS    \
    (USB_RXCSR_H_RXSTALL | USB_RXCSR_H_ERROR \
    | USB_RXCSR_DATAERROR | USB_RXCSR_RXPKTRDY)

/* TXCSR bits to avoid zeroing (write zero clears, write 1 ignored) */
#define USB_TXCSR_P_WZC_BITS   \
    (USB_TXCSR_P_INCOMPTX | USB_TXCSR_P_SENTSTALL \
    | USB_TXCSR_P_UNDERRUN | USB_TXCSR_FIFONOTEMPTY)

#define USB_TXCSR_H_WZC_BITS   \
    (USB_TXCSR_H_NAKTIMEOUT | USB_TXCSR_H_RXSTALL \
    | USB_TXCSR_H_ERROR | USB_TXCSR_FIFONOTEMPTY)


#endif
