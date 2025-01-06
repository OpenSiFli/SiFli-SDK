
/*********************************************************************************************************************
*               Copyright(c) 2020, Vcare Corporation. All rights reserved.
**********************************************************************************************************************
* @file     module_heart_vc.c
* @brief    Application example.
* @details
              The resources needed to use this chip include an RTC or a general timer, an external interrupt line,
            and an I2C interface.
              First of all, confirm the count frequency of the RTC or the general timer, assign it to 'mcuOscData',
            if the count frequency is 30kHz, mcuOscData = 30000; then confirm the frequency of the I2C clock, assign
            it to 'mcuI2cClock', if the frequency of the I2C clock is 400kHz, mcuI2cClock = 400.
              Add the code to get the RTC or the general timer count value in function 'vcHr02GetRtcCountFromMCU()',
            add I2C read and write function code in 'vcHr02ReadRegisters(uint8_t startAddress, uint8_t *pRegisters,
            uint8_t len)' and 'vcHr02ReadRegisters(uint8_t startAddress, uint8_t *pRegisters, uint8_t len)', but do
            not change the name of the function.
               How To Use The Chip: Call the program to initialize the peripherals of the main control chip first,
            these peripherals include I2C, an external interrupt, RTC or general timer. Then the chip ID can be read
            to verify whether the chip is working properly or whether I2C communication is normal, the register add-
            ress is 0x00, if your chip is VC31B/VC32S/VC51B/VC52S, the chip ID is 0x21, but also need to verify whe-
            ther your I2C supports continuous multi byte read and write, because the driver code involves continuous
            multi byte read and write. After initializing the peripherals, call the chip initialization function
            'vcHr02Init(&vcHr02,vcMode)', the parameter 'vcHr02' is a defined heart rate data structure, 'vcMode'
            is also defined variable which can be used to switch the chip working mode. After the initialization of
            the chip is completed, the peripheral lights will be on, and the lights will be dimmed when the fingers
            are close to the chip from far to near, the interrupt pin of the chip will have interrupt level.
              About the function 'vcHr02_process(AlgoSportMode_t vcSportMode)': It can be used as an interrupt serv-
            ice function or as an independent task, which is added to the task queue by the interrupt trigger, but
            we strongly recommend the second one. The parameter 'vcSportMode' is a defined variable which can be us-
            ed to switch the sport mode in heart rate mode.
*---------------------------------------------------------------------------------------------------------------------
*---------------------------------------------------------------------------------------------------------------------
           使用芯片需要的外设资源包括一个实时时钟源或者一个通用定时器(计数频率不得低于10K，最佳是在30K左右)，一个外部
        中断，一路I2C。
           首先，确认实时时钟源或者通用定时器的计数频率，并将这个频率赋值给变量‘mcuOscData’，如果计数频率是30KHz，则
        mcuOscData = 30000；然后确认I2C的时钟频率，并将这个频率赋值给变量‘mcuI2cClock’，如果I2C的时钟频率是400KHz，则
        mcuI2cClock = 400。
           在函数‘vcHr02GetRtcCountFromMCU()'’里添加获取实时时钟源或通用定时器的计数值的代码，注意不是返回差值（与VC31
        和VC51不同），在函数‘vcHr02ReadRegisters(uint8_t startAddress, uint8_t *pRegisters, uint8_t len)’添加I2C的读函
        数的代码，在函数‘vcHr02ReadRegisters(uint8_t startAddress, uint8_t *pRegisters, uint8_t len)’添加I2C的写函数的
        代码，但不要改变函数名称。读函数中三个变量的意义分别是：寄存器地址、指向寄存器内容存储位置的指针（存寄存器里读
        回来的数据的变量地址）、读的长度，写函数中三个变量的意义分别是：寄存器地址、指向要写入寄存器的内容的指针、写的
        长度。
           芯片使用：先调用初始化主控芯片外设的代码，初始化实时时钟源或通用定时器，I2C，外部中断，然后可以通过读芯片的
        ID来判断芯片是否正常工作或者I2C通讯是否正常，寄存器地址是0x00，如果使用的芯片是VC31B/VC32S/VC51B/VC52S，芯片的
        ID是0x21，但要确认I2C是否支持连续读写多个字节，因为驱动里涉及连续读写多个字节的操作。外设初始化完成之后，调用
        ‘vcHr02Init(&vcHr02,vcMode)’来初始化芯片，变量‘vcHr02’是已经定义的心率数据结构体，变量‘vcMode’也是已经定义的，
        可以通过改变这个变量的值，切换芯片的工作模式，初始化芯片之后，外围的等会亮，并且手指由远到近靠近时，灯会由亮到暗
        调整，芯片的中断引脚也会有中断电平。
           关于‘vcHr02_process(AlgoSportMode_t vcSportMode)’函数：可以作为中断服务函数，也可以作为独立任务，通过中断触发
        将它添加到任务队列，但我们强烈推荐第二种方式。‘vcSportMode’是一个已经定义的在心率模式下切换运动模式的变量。

       *若用户是用定时读取方式读取Gsensor数据，用于计步等功能，为了更好的动态心率性能，我们推荐按照我们的中断的时间间隔，
       在‘vcHr02_process(AlgoSportMode_t vcSportMode)’函数中对应位置，读取Gsensor数据，中断的时间间隔（佩戴状态下）可以
       根据FIFO的长度可调（40ms整数倍）。如果用户心率非24小时连续测量方式，即进入心率测量功能界面测量心率，退出界面停止
       心率测量或定时唤醒心率芯片测量心率方式，则调用‘vcHr02StopSample(vcHr02_t *pvcHr02)’函数让芯片进入休眠时，可以重新
       使能定时器，定时读取Gsensor数据，因为调用上述函数使心率芯片休眠时，中断脚将不会有中断电平，‘vcHr02_process(AlgoS-
       portMode_t vcSportMode)’函数将不会执行，如果用户是24小时连续心率测量，即不会休眠，可以忽略。若用户是按照Gsensor中
       断触发读取Gsensor数据的方式，请将Gsensor读取的数据保存到buffer中，Gsensor每读一次数据就将buffer中的上一次数据用这
       一次的数据覆盖，在‘vcHr02_process(AlgoSportMode_t vcSportMode)’函数中传入算法函数中。

* @author
* @date
* @version  V1.9.2
**********************************************************************************************************************
*/
#include <rtthread.h>
#include "drv_i2c.h"
#include "drv_io.h"
#include "drv_gpio.h"
#include "vc32s.h"
#include "vcHr02Hci.h"
#ifdef BSP_USING_PM
    #include "bf0_pm.h"
#endif
#ifdef PMIC_CONTROL_SERVICE
    #include "pmic_service.h"
#endif

/* Include your INT,I2C,Timer header file */
//#include "INT.h"
//#include "I2C.h"
//#include "Timer.h"

static struct rt_semaphore vc32_int_sem;
static rt_thread_t vc32_thread = NULL;
#define VC32S_THRED_STACK_SIZE      2048    // 6 * 1024
/*
 * @brief  MCU peripherals initialization.
 *
 *
 **/
void vcHr02PhyInit(void)
{
    /*****************************************************************************
    * 1.If Reset Pin of the chip is connected to MCU, initialize the corresponding
    *   pin on the MCU.
    *
    * 2.Initialize the I2C of MCU.
    *
    * 3.Initialize the external interrupt of MCU.
    *
    *
    ******************************************************************************/

    /*------------------INSERT YOUR CODE HERE-----------------*/

}


/*
 * @brief  Interrupt service function.
 *
 *
 **/
extern void vc32s_fetch_data(void);
static void vc32s_irq_handler(void *arg)
{
    rt_pin_detach_irq(VC32S_INT_BIT);
    rt_sem_release(&vc32_int_sem);
}


static void vc32s_sensor_task(void *params)
{
    int32_t ret;
    uint8_t status = 0;

    while (1)
    {
        rt_sem_take(&vc32_int_sem, RT_WAITING_FOREVER);
        rt_pin_irq_enable(VC32S_INT_BIT, 0);

        //vcHr02ReadRegisters(0x00, &status, 1);
        vc32s_fetch_data();
        rt_pin_attach_irq(VC32S_INT_BIT, PIN_IRQ_MODE_RISING, vc32s_irq_handler, (void *)(rt_uint32_t)VC32S_INT_BIT);
        rt_pin_irq_enable(VC32S_INT_BIT, 1);
    }
}


static int vc32s_thread_init(void)
{
    rt_sem_init(&vc32_int_sem, "hr_vc32s_sem", 0, RT_IPC_FLAG_FIFO);

    vc32_thread = rt_thread_create("hr_vc32s_init", vc32s_sensor_task, NULL, VC32S_THRED_STACK_SIZE, 15, 10);   //VC32S_THRED_STACK_SIZE
    RT_ASSERT(vc32_thread);

    return 0;
}

static void vc32s_thread_deinit(void)
{
    if (vc32_thread)
    {
        rt_thread_delete(vc32_thread);
        vc32_thread = NULL;
    }
    rt_sem_detach(&vc32_int_sem);
    rt_kprintf("vc32s_thread_deinit\n");
}

static void vc32s_irq_init(void)
{
    rt_pin_mode(VC32S_INT_BIT, PIN_MODE_INPUT);
    rt_pin_attach_irq(VC32S_INT_BIT, PIN_IRQ_MODE_RISING, vc32s_irq_handler, (void *)(rt_uint32_t)VC32S_INT_BIT);



#ifdef BSP_USING_PM // add v32s INT WAKE
    GPIO_TypeDef *gpio = GET_GPIO_INSTANCE(VC32S_INT_BIT);
    uint16_t gpio_pin = GET_GPIOx_PIN(VC32S_INT_BIT);

    int8_t wakeup_pin = HAL_LPAON_QueryWakeupPin(gpio, gpio_pin);
    RT_ASSERT(wakeup_pin >= 0);
    pm_enable_pin_wakeup(wakeup_pin, AON_PIN_MODE_POS_EDGE);
#endif /* BSP_USING_PM */
}

static void vc32s_irq_deinit(void)
{
#ifdef BSP_USING_PM
    GPIO_TypeDef *gpio = GET_GPIO_INSTANCE(VC32S_INT_BIT);
    int8_t gpio_pin = GET_GPIOx_PIN(VC32S_INT_BIT);

    int8_t wakeup_pin = HAL_LPAON_QueryWakeupPin(gpio, gpio_pin);
    RT_ASSERT(wakeup_pin >= 0);
    pm_disable_pin_wakeup(wakeup_pin);

#endif /* BSP_USING_PM */
    rt_pin_irq_enable(VC32S_INT_BIT, 0);
    rt_pin_detach_irq(VC32S_INT_BIT);
}

rt_err_t vc32s_int_open(void)
{
    vc32s_thread_init();
    vc32s_irq_init();

    rt_kprintf("vc32s_int_open \n");
    return 0;
}

void vc32s_int_close(void)
{
    vc32s_irq_deinit();
    vc32s_thread_deinit();

    rt_kprintf("vc32s_int_close! \n");
}

void vc32sStart(vcHr02_t *pVcHr02, vcHr02Mode_t vcHr02WorkMode)
{
    pVcHr02->workMode = vcHr02WorkMode;
    vcHr02StartSample(pVcHr02);
    rt_thread_startup(vc32_thread);
    rt_pin_irq_enable(VC32S_INT_BIT, 1);
}


