
#ifndef SL_SC7R30_BPM_DRV__H__
#define SL_SC7R30_BPM_DRV__H__


/************由原厂修改后编译*************/
/***使用驱动前请根据实际IIC情况进行配置***/
/**SC7R30的IIC 接口地址类型 7bits：  0****/
/**SC7R30的IIC 接口地址类型 8bits：  1****/
#define SL_SC7R30_IIC_7BITS_8BITS    0
/*****************************************/
#define  SL_SC7R30_IIC_7BITS_Address        (0x38)
#define  SL_SC7R30_IIC_8BITS_Address        (0x38<<1)

#if SL_SC7R30_IIC_7BITS_8BITS==0
    #define SL_SC7R30_IIC_Address  SL_SC7R30_IIC_7BITS_Address
#else
    #define SL_SC7R30_IIC_Address  SL_SC7R30_IIC_8BITS_Address
#endif

#define  SL_SC7R30_CHIP_ID_ADDR     0x0F
#define  SL_SC7R30_CHIP_ID_VALUE    0x71
#define  SL_SC7R30_SCHEME_NUM       0x01

/*****SL_SC7R30_SCHEME_NUM************************************/
/*****1:SC7R30，ONE  INSIDE GREEN LED  *******************************/
/*****2:SC7R30，TWO  INSIDE GREEN LED+ OUTSIDE GREEN LED DRIVER3******/
/*****3:SC7R30，TWO  INSIDE GREEN LED+ OUTSIDE GREEN LED DRIVER4******/
/*****4:SC7R30，TWO  INSIDE GREEN LED+ OUTSIDE IR    LED DRIVER3******/
/*****5:SC7R30，TWO  INSIDE GREEN LED+ OUTSIDE IR    LED DRIVER4******/
/*****6:SC7R30，THREE  INSIDE GREEN LED+ OUTSIDE GREEN LED DRIVER3 + OUTSIDE IR LED DRIVER4**/
/*****7:SC7R30，THREE  INSIDE GREEN LED+ OUTSIDE GREEN LED DRIVER4 + OUTSIDE IR LED DRIVER3**/
/*****8:SC7R30，THREE  INSIDE GREEN LED+ OUTSIDE RED   LED DRIVER3 + OUTSIDE IR LED DRIVER4**/
/*****9:SC7R30，THREE  INSIDE GREEN LED+ OUTSIDE RED   LED DRIVER4 + OUTSIDE IR LED DRIVER3**/

/*****SL_SC7R30_SCHEME_NUM************/
/*****0A:SC7R40，FOUR  INSIDE LED*************/

/*****SL_SC7R30_ADOM_LED**************/
/*****1:DRIVER1 LED**************************/
/*****2:DRIVER2 LED**************************/
/*****3:DRIVER3 LED**************************/
/*****4:DRIVER4 LED**************************/

/*****SL_SC7R30_HR_LED****************/
/*****1:SC7R30 DRIVER1 LED*******************/
/*****2:SC7R30 DRIVER1 LED + DRIVER3 LED*****/
/*****3:SC7R30 DRIVER1 LED + DRIVER4 LED*****/
/*****4:SC7R40 DRIVER1 LED + DRIVER3 LED*****/

/*****SL_SC7R30_OXG_LED**********************/
/*****1:SC7R30 DRIVER3 RED LED + DRIVER4 IR  LED****/
/*****2:SC7R30 DRIVER4 IR  LED + DRIVER3 RED LED****/
/*****3:SC7R40 DRIVER2 LED + DRIVER4 LED************/

#if(SL_SC7R30_SCHEME_NUM==0x01)  /*****WORK_Mode*******/
    #define  SL_SC7R30_ADOM_LED        0x01
    #define  SL_SC7R30_HR_LED          0x01
    #define  SL_SC7R30_OXG_LED         0x01
    #define  SL_SC7R30_OXG_ENABLE      0x00
    #elif(SL_SC7R30_SCHEME_NUM==0x02)/*****WORK_Mode*******/
    #define  SL_SC7R30_ADOM_LED        0x01
    #define  SL_SC7R30_HR_LED          0x02
    #define  SL_SC7R30_OXG_LED         0x01
    #define  SL_SC7R30_OXG_ENABLE      0x00
    #elif(SL_SC7R30_SCHEME_NUM==0x03)/*****WORK_Mode*******/
    #define  SL_SC7R30_ADOM_LED        0x01
    #define  SL_SC7R30_HR_LED          0x03
    #define  SL_SC7R30_OXG_LED         0x01
    #define  SL_SC7R30_OXG_ENABLE      0x00
    #elif(SL_SC7R30_SCHEME_NUM==0x04)/*****WORK_Mode*******/
    #define  SL_SC7R30_ADOM_LED        0x03
    #define  SL_SC7R30_HR_LED          0x01
    #define  SL_SC7R30_OXG_LED         0x01
    #define  SL_SC7R30_OXG_ENABLE      0x00
    #elif(SL_SC7R30_SCHEME_NUM==0x05)/*****WORK_Mode*******/
    #define  SL_SC7R30_ADOM_LED        0x04
    #define  SL_SC7R30_HR_LED          0x01
    #define  SL_SC7R30_OXG_LED         0x01
    #define  SL_SC7R30_OXG_ENABLE      0x00
    #elif(SL_SC7R30_SCHEME_NUM==0x06)/*****WORK_Mode*******/
    #define  SL_SC7R30_ADOM_LED        0x04
    #define  SL_SC7R30_HR_LED          0x02
    #define  SL_SC7R30_OXG_LED         0x01
    #define  SL_SC7R30_OXG_ENABLE      0x00
    #elif(SL_SC7R30_SCHEME_NUM==0x07)/*****WORK_Mode*******/
    #define  SL_SC7R30_ADOM_LED        0x03
    #define  SL_SC7R30_HR_LED          0x03
    #define  SL_SC7R30_OXG_LED         0x01
    #define  SL_SC7R30_OXG_ENABLE      0x00
    #elif(SL_SC7R30_SCHEME_NUM==0x08)/*****WORK_Mode*******/
    #define  SL_SC7R30_ADOM_LED        0x04
    #define  SL_SC7R30_HR_LED          0x01
    #define  SL_SC7R30_OXG_LED         0x01
    #define  SL_SC7R30_OXG_ENABLE      0x01
    #elif(SL_SC7R30_SCHEME_NUM==0x09)/*****WORK_Mode*******/
    #define  SL_SC7R30_ADOM_LED        0x03
    #define  SL_SC7R30_HR_LED          0x01
    #define  SL_SC7R30_OXG_LED         0x02
    #define  SL_SC7R30_OXG_ENABLE      0x01
    #elif(SL_SC7R30_SCHEME_NUM==0x0A)/*****WORK_Mode*******/
    #define  SL_SC7R30_ADOM_LED        0x04
    #define  SL_SC7R30_HR_LED          0x04
    #define  SL_SC7R30_OXG_LED         0x03
    #define  SL_SC7R30_OXG_ENABLE      0x01
#endif

#define SL_ALGO_MCU_SEL               0//0x01
/*****0x00: any mcu********/
/*****0x01：stm32f103******/
/*****0x02：da14585********/
/*****0x03：da14580********/
/*****0x04：nrf51822*******/

/*****0x00: any mcu********/
/*****0x01:stm32f103*******/
/*****0x02:da14585*********/
/*****0x03:da14580*********/
/*****0x04:nrf51822********/
#if   SL_ALGO_MCU_SEL==0
#elif SL_ALGO_MCU_SEL==1
    #include "stm32f10x_gpio.h"
    #define SL_STM32F10X_DEBUG        1  //DEBUG
#elif SL_ALGO_MCU_SEL==2
    #include "gpio.h"
    #define SL_DA14585_DEBUG          1  //DEBUG
#elif SL_ALGO_MCU_SEL==3
#else
#endif

/*****Driver Test******/
signed char SL_SC7R30_Driver_Test(unsigned char led_on_time);
/*****led_on_time:  turn on time for test***********/
/*****return：  1  IIC Success**********************/
/*****return： -1  IIC Continuous Read Fail*********/
/*****return： -2  IIC Write      Read Fail*********/

/***current & Skin  Grade Set******/
/*   adom:*0-255 0:easy    255:hard***default:50  */
/*no_adom:*0-255 0:hard    255:easy***default:60  */
/*Signal  *1-15    1:small   15:big  ***default:8 */
/*skin    *Current Set  default:  00+0+0x1F =0x1F**/
/*skin    *Current Set  default:  01+0+0x1F =0x5F**/
/*skin    *Current Set  default:  10+0+0x1F =0x9F**/
/*skin    *Current Set  default:  11+0+0x1F =0xDF**/
void SL_SC7R30_Signal_Skin_Grade(unsigned char SL_ADOM_TH, unsigned char SL_NO_ADOM_TH, unsigned char SL_SC7R30_Signal_Grade, unsigned char SL_Skin_Colour_Grade);

/*min:*1-200 mA****/
/*max:*1-200 mA****/
void SL_SC7R30_Current_SET(unsigned char SL_Current_Min, unsigned char SL_Current_Max);
/*the smallest current   1 -200    default value:3**/
/*the biggest current    1 -200    default value:60*/


/********0x10-0xF0********/
void SL_SC7R30_LED_SEL_SET(unsigned char Adom_Led, unsigned char HR_Led);
/********0x10: LDR1*******/
/********0x20: LDR2*******/
/********0x40: LDR3*******/
/********0x80: LDR4*******/
/********0x90: LDR4 LDR1*******/
/********0xC0: LDR4 LDR3*******/


/***************Open The Function***********/
void SL_SC7R30_ALGO_START(unsigned char SL_ALGO_MODE);
/*****SL_ALGO_MODE=1 Set Adom************/
/*****SL_ALGO_MODE=2 Set HR   ***********/

/***********Close The Function******************/
void          SL_SC7R30_ALGO_END(void);

/******Return The Opened Function Mode**********/
unsigned char SL_SC7R30_ALGO_MODE(void);

/********Get Adom Status Right Now**************/
unsigned char SL_SC7R30_GET_ADOM_STATUS(void);
/**return： 0  no adom，Continue to test *******/
/**return： 1  adom，Perform other algorithms***/

/********Get SL_ADOM_TH Compare Value***********/
signed short SL_SC7R30_GET_ADOM_PS(void);
/**return： 0~32767   *************************/
/**if return value < SL_ADOM_TH*(100) adom*****/
/**if return value > SL_ADOM_TH*(100) no adom**/


/*****Get the current heart rate value**********/
unsigned char SL_SC7R30_Get_Heart_Rate(void);
/*****bpm value :40-180*************************/

/********客户需要进行的IIC接口封包函数****************/
extern unsigned char SL_SC7R30_I2c_Spi_Write(unsigned char reg, unsigned char dat);
extern unsigned char SL_SC7R30_I2c_Spi_Read(unsigned char reg, unsigned char len, unsigned char *buf);
/**SL_SC7R30_i2c_spi_Write 函数中， Reg：寄存器地址   data：寄存器的配置值********************************/
/**SL_SC7R30_i2c_spi_Write 函数 是一个单次写的函数********************************************************/
/**SL_SC7R30_i2c_spi_Read 函数中， Reg 同上，len:读取数据长度，buf:存储数据首地址（指针）*****************/
/**SL_SC7R30_i2c_spi_Read 函数 是可以进行单次读或多次连续读取的函数***************************************/

#define SC7R30_ODR_SEL                    0x01  //10Hz
#if   SC7R30_ODR_SEL==0x01
    #define SC7R30_ODR                        10 //10Hz  //1s
#elif SC7R30_ODR_SEL==0x02
    #define SC7R30_ODR                        15 //15Hz  //1s
#endif

#define SL_SC7R30_DEBUG                   0x00  //Debug
#define SL_SC7R30_BP_SPO2_Enable          0x00  //BP SPO2
#define SL_SC7R30_Desk_Paper_Enable       0x00  //desk paper monitor
#define SL_SC7R30_Dynamic_Enable          0x00  //Dynamic Algo Enable
#define SL_SC7R30_GET_DATA_Enable         0x01  //ENABLE
#define SL_LED_AUTO_ADJUST_ENABLE         0x01  //ENABLE


#if SL_SC7R30_BP_SPO2_Enable            ==0x01
    /*************初始化个人参数*************/
    /**参数初始化，用于血压、血氧计算********/
    void SL_BP_SPO2_Person_Inf_Init(unsigned char *Person_Inf_Init);
    /*********输入指针参数分别是:身高 体重 年龄 性别***举例:178,60,26 1*********/
    /**身高范围:  30cm ~ 250cm  *************/
    /**体重范围:  10Kg ~ 200Kg  *************/
    /**年龄范围:  3岁  ~ 150岁  *************/
    /**性别范围:  0 ~ 1    0:女 1:男   ******/

    void SL_Get_BP_Value(unsigned char *SBP_Value, unsigned char *DBP_Value);
    /****SBP_Value:收缩压           80-160**/
    /****DBP_Value:舒张压           60-100**/

    void SL_Get_SPO2_Value(unsigned char *SPO2_Value);
    /****SPO2_Value:血氧饱和浓度    80-100**/
#endif

#if SL_SC7R30_Desk_Paper_Enable         ==0x01  //desk paper monitor
    void SL_Desk_Paper_Degree_Set(unsigned char v1_level, unsigned short v2_level, unsigned char t2_level);
    /**********v1_level:0-255  推荐范围：10-200    默认值：80****************/
    /**********v1_level:设置越大，越容易触发 检测到桌子**********************/
    /**********v1_level:设置越小，越困难触发 检测到桌子**********************/
    /**********v2_level:0-65535  推荐范围：50-5000  默认值：300**************/
    /**********v2_level:设置越大，越容易触发 检测到桌子**********************/
    /**********v2_level:设置越小，越困难触发 检测到桌子**********************/
    /**********t2_level:0-255  推荐范围：10-200    默认值：55****************/
    /**********t2_level:设置越大，越困难触发 检测到桌子**********************/
    /**********t2_level:设置越小，越容易触发 检测到桌子**********************/
    /*************选择方法：选择一个戴在手上永远不会关闭的最大值*************/

    void SL_SC7R30_GET_Desk_Para(signed short *para1, unsigned char *para2);
    /***len=1****para1:内部测量值，和v_level有对应关系***********************/
    /***len=1****para2:内部测量值，和t_level有对应关系***********************/

#endif


#if SL_SC7R30_Dynamic_Enable            ==0x01
    void SL_IMPORT_ACC_DATA(signed char *SL_SC7A20_X_Buf, signed char *SL_SC7A20_Y_Buf, signed char *SL_SC7A20_Z_Buf, unsigned char len, unsigned int SL_STEP_NUM);
    //为了改善运动过程中，心率信号的稳定性，增加加速度计数据进行补偿心率值
    /*************输入XYZ三轴数组首地址，同时传入数组有效长度*********/
    /*************SL_SC7A20_X_Buf[0]:    X轴数据**********************/
    /*************SL_SC7A20_Y_Buf[0]:    Y轴数据**********************/
    /*************SL_SC7A20_Z_Buf[0]:    Z轴数据**********************/
    /*************len:                   数组长度*********************/
    /*************step:                  当前步数值*******************/
    //input xyz value    //8BITS  ±4G   1G≈32LSB**********************/


    void SL_Dynamic_Para_Set(unsigned char speed, unsigned char vth);
    /*****动态心率响应速度设置*******/
    /*****speed=1-20  默认值：5******/
    /*****vth  =1-50  默认值：5******/

    void SL_Get_Dynamic_Para(signed short *para1, signed short *para2, signed short *para3);
    //需要打印的三个参数

#endif

#if      SL_SC7R30_GET_DATA_Enable       ==0x01
    /***signal_flag:  1:orignal signal    6:last signal***SL_SC7R30_Data_Buf: the buffer length is 15*********/
    unsigned char SL_SC7R30_Get_FIFO_Data(unsigned char signal_flag, signed short *SL_SC7R30_Data_Buf);
    /*****return：  FIFO length********/
    /*****1:  orignal signal***********/
    /*****2:  HPF           ***********/
    /*****3:  HPF           ***********/
    /*****4:  LPF           ***********/
    /*****5:  HPF           ***********/
    /*****6:  LIM_OUT       ***********/
    /*****SL_SC7R30_Data_Buf[20]:Buffer for data***********/

#endif


#endif
