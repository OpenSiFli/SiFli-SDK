#include <rtthread.h>
#include "board.h"
#include "sc7a20.h"
#include "sc7a20_driver.h"
#include "SL_Watch_Algorithm.h"

#define LOG_TAG              "drv.7a20c"
#include <drv_log.h>

#define SC7A20_CHIP_ID              (0x11)

#ifdef BSP_PORT_SIFLI_TO_PERSIM
    extern void rt_sc7a20_pm_control(rt_uint8_t flag);
#endif /* BSP_PORT_SIFLI_TO_PERSIM */

static signed short sc7a20_xyz[3][16];      //传感器x,y,z坐标值

unsigned char SL_MEMS_i2cWrite(unsigned char reg, unsigned char data)
{
#ifdef BSP_PORT_SIFLI_TO_PERSIM
    rt_sc7a20_pm_control(RT_TRUE);
#endif /* BSP_PORT_SIFLI_TO_PERSIM */

    sc7a20_write_reg(reg, data);

#ifdef BSP_PORT_SIFLI_TO_PERSIM
    rt_sc7a20_pm_control(RT_FALSE);
#endif /* BSP_PORT_SIFLI_TO_PERSIM */

    return 0;
}

unsigned char SL_MEMS_i2cRead(unsigned char reg, unsigned char len, unsigned char *buf)
{
#ifdef BSP_PORT_SIFLI_TO_PERSIM
    rt_sc7a20_pm_control(RT_TRUE);
#endif /* BSP_PORT_SIFLI_TO_PERSIM */

    sc7a20_read_reg(reg, buf, len);

#ifdef BSP_PORT_SIFLI_TO_PERSIM
    rt_sc7a20_pm_control(RT_FALSE);
#endif /* BSP_PORT_SIFLI_TO_PERSIM */

    return 0;
}


unsigned int Gsensor_Read_Data(void)
{
    unsigned int step;
    unsigned int fifodepth;

#if defined(__CC_ARM) || defined(__CLANG_ARM)
#ifdef BSP_PORT_SIFLI_TO_PERSIM
    rt_sc7a20_pm_control(RT_TRUE);
#endif /* BSP_PORT_SIFLI_TO_PERSIM */

    // SL_SC7A20_Watch_Algo_Exe(0);            //保证算法执行到位
    fifodepth = SL_SC7A20_GET_DATA(sc7a20_xyz[0], sc7a20_xyz[1], sc7a20_xyz[2]);    //获取x y z坐标值

#ifdef BSP_PORT_SIFLI_TO_PERSIM
    rt_sc7a20_pm_control(RT_FALSE);
#endif /* BSP_PORT_SIFLI_TO_PERSIM */

    for (unsigned int i = 0; i < fifodepth; i++)
    {
        rt_kprintf("%s: X:%d,Y:%d,Z:%d\r\n", __func__, sc7a20_xyz[0][i], sc7a20_xyz[1][i], sc7a20_xyz[2][i]);
    }
#endif
    step = Gsensor_Get_Step();
    rt_kprintf("step:%d\r\n", step);
    return step;
#if 0
#if 1
    if (SharkPhoto && SL_Get_Phone_Answer_Status(1, 1))                   //参数2写1是不能计步
    {
        SharkPhotograp_handler();                                        //摇一摇拍照
    }
    else
    {
        step = SL_Pedo_GetStepCount();                                    //获取当前步数
        Set_Step(step);
        //if(old_step_counter!=step_counter){                               //计步有变动
        //    old_step_counter=step_counter;
        //    update_database_0xFEA1();
        //}
    }
    if (!charge_status && Get_Gesture_Statue())  //APP抬手亮屏开关
    {
        wrist_status = SL_Turn_Wrist_Get_Status();//获取抬手状态
        if (wrist_status == 1)
        {
            sensor_wake_shakecount();
            Gesture_Screen_On();
        }
        else if (wrist_status == 2)
        {
            Gesture_Screen_Off();
        }
    }
    Sleep_CountPro(sc7a20_xyz[0][0], sc7a20_xyz[1][0], sc7a20_xyz[2][0]);
    //SEGGER_RTT_printf(0, "step_counter=%d,wrist_status=%d\n", step,wrist_status);
#else
    if (RemindSub_type == RemindSub_Shark_Photograp) //APP摇一摇开关
    {
        if (SL_Get_Phone_Answer_Status(1, 1)) //参数2写1是不能计步
        {
            SharkPhotograp_handler();        //摇一摇拍照
            sensor_wake_shakecount();
        }
    }
    else
    {
        d = SL_Pedo_GetStepCount();
        step_counter = (d > 99999) ? 99999 : d;
        step_counter += SC7A20_Step_Buffer;
        if (old_step_counter != step_counter) //计步有变动
        {
            stepcount = 60;
            old_step_counter = step_counter;
            update_database_0xFEA1();
        }
    }
    if (Gesture_parameter_get(GESTURE_ENABLE))  //APP抬手亮屏开关
    {
        SL_status = SL_Turn_Wrist_Get_Status();
        if (SL_status == 1) //获取抬手状态
        {
            stepcount = 60;
            sensor_wake_shakecount();
            Gesture_Screen_On();
        }
        else if (SL_status == 2)
        {
            //Gesture_Screen_Off();
        }
    }
    Sleep_CountPro(a, b, c);
#endif
#endif
}

unsigned int Gsensor_Set_Step(void)
{
    unsigned char person_info[4] = {178, 60, 26, 1};

#ifdef BSP_PORT_SIFLI_TO_PERSIM
    rt_sc7a20_pm_control(RT_TRUE);
#endif /* BSP_PORT_SIFLI_TO_PERSIM */

    SL_PEDO_InitAlgo();         //复位计步值,初始化算法
    SL_PEDO_Degree_Init(3);     //初始计步难度设置
    SL_Pedo_Person_Inf_Init(person_info);       //初始化个人参数，用于热量，距离计算

#ifdef BSP_PORT_SIFLI_TO_PERSIM
    rt_sc7a20_pm_control(RT_FALSE);
#endif /* BSP_PORT_SIFLI_TO_PERSIM */

    return 0;
}

unsigned int Gsensor_Get_Step(void)
{
    unsigned int step;

#ifdef BSP_PORT_SIFLI_TO_PERSIM
    rt_sc7a20_pm_control(RT_TRUE);
#endif /* BSP_PORT_SIFLI_TO_PERSIM */

#if defined(__CC_ARM) || defined(__CLANG_ARM)
    SL_SC7A20_Watch_Algo_Exe(0);             //保证算法执行到位
    step = SL_Pedo_GetStepCount();
#endif

#ifdef BSP_PORT_SIFLI_TO_PERSIM
    rt_sc7a20_pm_control(RT_FALSE);
#endif /* BSP_PORT_SIFLI_TO_PERSIM */

    return step;
}

unsigned char Gsensor_Get_Motion_Status(void)
{
#ifdef BSP_PORT_SIFLI_TO_PERSIM
    unsigned char status;

    rt_sc7a20_pm_control(RT_TRUE);

    status =  SL_Pedo_GetMotion_Status();

    rt_sc7a20_pm_control(RT_FALSE);

    return status;
#else
    return SL_Pedo_GetMotion_Status();
#endif /* BSP_PORT_SIFLI_TO_PERSIM */

}

unsigned int Gsensor_Get_Step_Distance(void)
{
#ifdef BSP_PORT_SIFLI_TO_PERSIM
    unsigned int distance;

    rt_sc7a20_pm_control(RT_TRUE);

    distance =  SL_Pedo_Step_Get_Distance();

    rt_sc7a20_pm_control(RT_FALSE);

    return distance;
#else
    return SL_Pedo_Step_Get_Distance();
#endif /* BSP_PORT_SIFLI_TO_PERSIM */
}

unsigned int Gsensor_Get_Step_KCal(void)
{
#ifdef BSP_PORT_SIFLI_TO_PERSIM
    unsigned int step_kcal;

    rt_sc7a20_pm_control(RT_TRUE);

    step_kcal =  SL_Pedo_Step_Get_KCal();

    rt_sc7a20_pm_control(RT_FALSE);

    return step_kcal;
#else
    return SL_Pedo_Step_Get_KCal();
#endif /* BSP_PORT_SIFLI_TO_PERSIM */
}

unsigned int Gsensor_Set_Gesture_Init(void)        //初始化抬腕亮屏手势
{
    unsigned char sl_para[4] = {1, 0, 2, 2};
    unsigned char sl_turn_wrist_init_para[4] = {6, 3, 0, 30};

#ifdef BSP_PORT_SIFLI_TO_PERSIM
    rt_sc7a20_pm_control(RT_TRUE);
#endif /* BSP_PORT_SIFLI_TO_PERSIM */

#if defined(__CC_ARM) || defined(__CLANG_ARM)
    SL_Turn_Wrist_Init(&sl_turn_wrist_init_para[0]);    //抬手亮屏
    SL_Sc7a20_Int_Config_Init(&sl_para[0]);             //中断配置参数初始化
#endif

#ifdef BSP_PORT_SIFLI_TO_PERSIM
    rt_sc7a20_pm_control(RT_FALSE);
#endif /* BSP_PORT_SIFLI_TO_PERSIM */

    return 0;
}

signed char Gsensor_Get_Gesture_status(void)        //获取抬腕亮屏手势
{
#ifdef BSP_PORT_SIFLI_TO_PERSIM
    signed char status;

    rt_sc7a20_pm_control(RT_TRUE);

    status =  SL_Turn_Wrist_Get_Status();//获取抬手状态

    rt_sc7a20_pm_control(RT_FALSE);

    return status;
#else
    return SL_Turn_Wrist_Get_Status();//获取抬手状态
#endif /* BSP_PORT_SIFLI_TO_PERSIM */
}

void Triaxial_Sensor_Int_Handler(void)
{


}

void Triaxial_Sensor_Enable(void)
{
    unsigned char  sl_para[4] = {1, 0, 2, 2};

#ifdef BSP_PORT_SIFLI_TO_PERSIM
    rt_sc7a20_pm_control(RT_TRUE);
#endif /* BSP_PORT_SIFLI_TO_PERSIM */

    SL_SC7A20_Driver_Init();
    SL_Sc7a20_Int_Config_Init(&sl_para[0]);            //中断配置参数初始化

#ifdef BSP_PORT_SIFLI_TO_PERSIM
    rt_sc7a20_pm_control(RT_FALSE);
#endif /* BSP_PORT_SIFLI_TO_PERSIM */

    rt_kprintf("Triaxial Sensor Enable!!!\r\n");
}

void Triaxial_Sensor_Disable(void)
{
#ifdef BSP_PORT_SIFLI_TO_PERSIM
    rt_sc7a20_pm_control(RT_TRUE);
#endif /* BSP_PORT_SIFLI_TO_PERSIM */

    SL_MEMS_i2cWrite(0x20, 0x00);

#ifdef BSP_PORT_SIFLI_TO_PERSIM
    rt_sc7a20_pm_control(RT_FALSE);
#endif /* BSP_PORT_SIFLI_TO_PERSIM */

    rt_kprintf("Triaxial Sensor Disable!!!\r\n");
}

int Triaxial_Sensor_CheckID(void)
{
    uint8_t id;

    SL_MEMS_i2cRead(0x0f, 1, &id);
    if (id == SC7A20_CHIP_ID)
    {
        rt_kprintf("sc7a20_id = [0x%x]\r\n", id);
        return 0;
    }
    else
    {
        return -1;
    }
}

int Triaxial_Sensor_Init(void)
{
    int ret = 0;
    /* Initial I2C BUS */
    ret = sc7a20_init();
    if (ret < 0)
    {
        return ret;
    }
    ret = Triaxial_Sensor_CheckID();
    if (ret == 0)
    {
        // Triaxial_Sensor_Disable();          //初始化后默认关闭传感器
        Triaxial_Sensor_Enable();           //初始化后默认打开传感器
        Gsensor_Set_Gesture_Init();         //初始化抬腕亮屏手势
        rt_kprintf("Triaxial Sensor Init Succeed!!!\r\n");
    }
    else
    {
        rt_kprintf("Triaxial Sensor Init Error!!!\r\n");
        return -1;
    }

    return 0;
}

