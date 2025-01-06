#include <rtthread.h>
#if defined(__CC_ARM) || defined(__CLANG_ARM)
#include "SL_SC7R30_BPM_Algo_Driver.h"

#if SL_SC7R30_DEBUG==0x01

#endif
#define SL_PRINT_ENABLE   0x00

static signed char sl_init_status = 0x00;
signed char SL_SC7R30_BPM_INIT(void)
{
#if SL_SC7R30_BP_SPO2_Enable            ==0x01
    unsigned char  sl_person_para[4] = {178, 60, 29, 1};
#endif

    /*sensor online detect:  1  -1  -2*/
    sl_init_status = SL_SC7R30_Driver_Test(1);
    /*-2:IIC Write  Read Error  -1:IIC Continuous Read Error*/

    /*Adom Para&No Adom Para&LED Degree&Skin Color  */
    SL_SC7R30_Signal_Skin_Grade(50, 60, 8, 0x5F);

    /*LED Current SET: min value &max value    mA   */
    SL_SC7R30_Current_SET(1, 50);
    /*SET ADOM LED AND HR LED                       */
    /*0x10,0x10=LDR1&LDR1, 0x40,0x90=LDR3& LDR4 LDR1*/
    SL_SC7R30_LED_SEL_SET(0x10, 0x10); // 0x40 0x90
#if SL_SC7R30_Desk_Paper_Enable         ==0x01
    /**Desk & Paper dectect**/
    SL_Desk_Paper_Degree_Set(80, 300, 55);
#endif

#if SL_SC7R30_BP_SPO2_Enable            ==0x01
    /**********set motion para**********/
    SL_BP_SPO2_Person_Inf_Init(&sl_person_para[0]);
#endif
#if SL_SC7R30_Dynamic_Enable            ==0x01
    /**********set Dynamic para*********/
    SL_Dynamic_Para_Set(5, 5);
#endif

    return sl_init_status;
}

#define  SL_HR_MONITOR_WAIT_TIME    10 //s
static unsigned char  SL_ADOM_WAIT_TIME_NUM     = 0;
static unsigned char  SL_ADOM                   = 0;
#if SL_SC7R30_Desk_Paper_Enable==1
    extern unsigned char  sl_no_human_cnt;
#endif
#if SL_SC7R30_Dynamic_Enable            ==0x01
    extern signed char   SL_ACC_X[15];//±4G,高8bits  +1g=+32
    extern signed char   SL_ACC_Y[15];//±4G,高8bits  +1g=+32
    extern signed char   SL_ACC_Z[15];//±4G,高8bits  +1g=+32
    extern unsigned char SL_ACC_LEN;  //10-15Hz=>1s=10-15个数据
    extern unsigned int  SL_STEP_NUM; //the last step value
#endif
unsigned char SL_SC7R30_BPM_ALGO(void)
{
#if SL_PRINT_ENABLE==0x01
    unsigned char sl_i          = 0;
    unsigned char sl_fifo_len   = 0;
    signed short  fifo_buf[30]  = {0};
    signed short  sl_para[3]    = {0};
#endif
    unsigned char sl_bpm        = 0;
#if SL_SC7R30_BP_SPO2_Enable==0x01
    unsigned char sl_sbp        = 0;
    unsigned char sl_dbp        = 0;
    unsigned char sl_spo2       = 0;
#endif
    unsigned char sc7r30_mode   = 0;
#if SL_SC7R30_DEBUG==0x01
    signed short  SC7R30_Adom_Ps = 0;
#if SL_SC7R30_Desk_Paper_Enable==0x01
    signed short  SC7R30_Desk_P1 = 0;
    unsigned char SC7R30_Desk_P2 = 0;
#endif
#endif

    if (sl_init_status != 0x01)    return 1; //initial fail

    sc7r30_mode = SL_SC7R30_ALGO_MODE();

    if (sc7r30_mode == 0x02)
    {
#if SL_SC7R30_Dynamic_Enable            ==0x01
        SL_IMPORT_ACC_DATA(&SL_ACC_X[0], &SL_ACC_Y[0], &SL_ACC_Z[0], SL_ACC_LEN, SL_STEP_NUM); //INPUT ACCEL DATA
#endif
        sl_bpm = SL_SC7R30_Get_Heart_Rate(); //SL_FREQ_HEART_RATE

#if SL_PRINT_ENABLE==0x01
#if SL_SC7R30_Dynamic_Enable            ==0x01
        SL_Get_Dynamic_Para(&sl_para[0], &sl_para[1], &sl_para[2]);
        USART_printf(USART1, "para1=%d para2=%d para3=%d \r\n", sl_para[0], sl_para[1], sl_para[2]);
#endif

//      sl_fifo_len = SL_SC7R30_Get_FIFO_Data(1,&fifo_buf[0]);
//      for(sl_i=0;sl_i<sl_fifo_len;sl_i++)
//      {
//          USART_printf( USART1, "ps_value=%d\r\n",fifo_buf[sl_i]);
//      }
#endif
#if SL_SC7R30_BP_SPO2_Enable            ==0x01
        SL_Get_BP_Value(&sl_sbp, &sl_dbp);
        SL_Get_SPO2_Value(&sl_spo2);
#endif

#if SL_SC7R30_Desk_Paper_Enable==0x01
#if SL_SC7R30_DEBUG==0x01
        //打印一下桌面检测的两个参数值，用于适配初始化桌面检测的参数调节
        SL_SC7R30_GET_Desk_Para(&SC7R30_Desk_P1, &SC7R30_Desk_P2);
        USART_printf(USART1, "Desk_P1:%d Desk_P2:%d!\r\n", SC7R30_Desk_P1, SC7R30_Desk_P2);
#endif
        //add here for user
#endif

        if (SL_ADOM_WAIT_TIME_NUM < SL_HR_MONITOR_WAIT_TIME)
        {
            SL_ADOM_WAIT_TIME_NUM++;
            sl_bpm = 0;
        }
    }

    if (sc7r30_mode != 0x00)
    {
        SL_ADOM = SL_SC7R30_GET_ADOM_STATUS();
        if (SL_ADOM == 1) //adom
        {
        }
        else if (SL_ADOM == 2) //adom
        {
            SL_SC7R30_ALGO_START(0x02);//open hr monitor
            SL_ADOM_WAIT_TIME_NUM = 0;
        }
        else if (SL_ADOM == 3) //no adom
        {
            sc7r30_mode = SL_SC7R30_ALGO_MODE();
        }
    }

#if SL_SC7R30_DEBUG==0x01
    //打印一下佩戴检测时，PS输出值，用于适配初始化佩戴参数调节
    SC7R30_Adom_Ps = SL_SC7R30_GET_ADOM_PS();
    USART_printf(USART1, "SC7R30_Adom_Ps=%d!\r\n", SC7R30_Adom_Ps);
#endif
    //add here for user

#if SL_SC7R30_Desk_Paper_Enable==0x01
    if (sl_no_human_cnt < 3) //test three times about no humam
#endif
        if (sc7r30_mode == 0x00) //reset  twice not human test and close it
        {
            SL_SC7R30_ALGO_START(0x01);
            SL_ADOM_WAIT_TIME_NUM = 0;
        }

#if SL_SC7R30_Desk_Paper_Enable==0x01
#if SL_SC7R30_DEBUG==0x01
    USART_printf(USART1, "sl_no_human_cnt:%d!\r\n", sl_no_human_cnt);
#endif
    //add here for user
#endif

    return sl_bpm;
}

#if SL_SC7R30_Desk_Paper_Enable==0x01
/*when start measure hr,and sl_no_human_cnt >=3  ,sl_no_human_cnt variable must clean it */
void SL_SC7R30_DESK_Clean(void)
{
    sl_no_human_cnt = 0;
}
#endif

#endif