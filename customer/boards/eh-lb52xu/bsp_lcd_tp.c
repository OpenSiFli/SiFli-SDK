#include "bsp_board.h"
#if LCD_TP_SHRAE_POWER
    #include "rtthread.h"
#endif

#ifdef BSP_USING_LCD
//#define LCD_VCC_EN                      // NO pin
//#define LCD_VIO_EN                        // NO pin

#define LCD_RESET_PIN           (0)         // GPIO_A00

//#define TP_VCC_EN                     // NO pin
//#define TP_VIO_EN                     // NO pin

#define TP_RESET            (44)            // GPIO_A01

#define LCD_TP_COM_POWER    (30)   //HDK V1.3.0 changto A30 
#define LCD_TP_COM_POWER1    (9)    //HDK V1.2.0 is GPIO_A09
#define PIN_LCD_VCI_EN      (1)     //GPIO_A01
#if LCD_TP_SHRAE_POWER

static struct rt_mutex              lcd_tp_power_mutex;

int lcd_tp_power_mutex_init()
{
    rt_err_t err = rt_mutex_init(&lcd_tp_power_mutex, "lcd_tp_power_mutex", RT_IPC_FLAG_FIFO);
    return err;
}
INIT_COMPONENT_EXPORT(lcd_tp_power_mutex_init);
typedef enum
{

    LCD_TP_NO_OPENED   = 0x00,
    LCD_OPENED = 0x01,
    TP_OPENED = 0x02,
} open_state_t;
enum
{
    POWER_OFF,
    POWER_ON,
};
volatile static uint8_t lcd_tp_opened = LCD_TP_NO_OPENED;
static void lcd_tp_set_state(uint8_t state, uint8_t mode)
{
    if (POWER_ON == state)
    {
        lcd_tp_opened |= mode;
    }
    else
    {
        lcd_tp_opened &= (~mode);
    }
}
static uint8_t lcd_tp_get_state(uint8_t mode)
{
    return (lcd_tp_opened & mode) ? POWER_ON : POWER_OFF;
}

#endif
extern void BSP_GPIO_Set(int pin, int val, int is_porta);

__WEAK void BSP_LCD_TP_PowerDown_Config(void)
{
}

void BSP_LCD_TP_PowerUp_Config()
{
    //LCDC1
    //chang function to digi_io
#if 1
    HAL_PIN_SetMode(PAD_PA04, 1, PIN_DIGITAL_IO_NORMAL);
    HAL_PIN_SetMode(PAD_PA03, 1, PIN_DIGITAL_IO_NORMAL);
    HAL_PIN_SetMode(PAD_PA05, 1, PIN_DIGITAL_IO_NORMAL);
    HAL_PIN_SetMode(PAD_PA06, 1, PIN_DIGITAL_IO_NORMAL);
    HAL_PIN_SetMode(PAD_PA07, 1, PIN_DIGITAL_IO_NORMAL);
    HAL_PIN_SetMode(PAD_PA08, 1, PIN_DIGITAL_IO_NORMAL);
    HAL_PIN_SetMode(PAD_PA02, 1, PIN_DIGITAL_IO_NORMAL);
    HAL_PIN_SetMode(PAD_PA01, 1, PIN_DIGITAL_IO_NORMAL);
#endif


    HAL_PIN_Set(PAD_PA04, LCDC1_SPI_CLK, PIN_NOPULL, 1);
    HAL_PIN_Set(PAD_PA03, LCDC1_SPI_CS, PIN_NOPULL, 1);
    HAL_PIN_Set(PAD_PA05, LCDC1_SPI_DIO0, PIN_PULLDOWN, 1);
    HAL_PIN_Set(PAD_PA06, LCDC1_SPI_DIO1, PIN_NOPULL, 1);
    HAL_PIN_Set(PAD_PA07, LCDC1_SPI_DIO2, PIN_NOPULL, 1);
    HAL_PIN_Set(PAD_PA08, LCDC1_SPI_DIO3, PIN_NOPULL, 1);
    HAL_PIN_Set(PAD_PA02, LCDC1_SPI_TE, PIN_NOPULL, 1);

#if defined(LCD_USING_PWM_AS_BACKLIGHT)
    HAL_PIN_Set(PAD_PA01, GPTIM2_CH4, PIN_NOPULL, 1);   // LCDC1_BL_PWM_CTRL, LCD backlight PWM
#else
    HAL_PIN_Set(PAD_PA01, GPIO_A1, PIN_NOPULL, 1);     // LCDC1_BL_PWM_CTRL, LCD backlight PWM
#endif

    //Touch
#if 1
    HAL_PIN_SetMode(PAD_PA42, 1, PIN_DIGITAL_IO_NORMAL);
    HAL_PIN_SetMode(PAD_PA41, 1, PIN_DIGITAL_IO_NORMAL);
    HAL_PIN_SetMode(PAD_PA43, 1, PIN_DIGITAL_IO_NORMAL);

    HAL_PIN_Set(PAD_PA42, I2C1_SCL, PIN_NOPULL, 1);     //SCL
    HAL_PIN_Set(PAD_PA41, I2C1_SDA, PIN_NOPULL, 1);     //SDA
    HAL_PIN_Set(PAD_PA43, GPIO_A43, PIN_NOPULL, 1);     // CTP_WKUP_INT
#endif
    HAL_PIN_SetMode(PAD_PA00, 1, PIN_DIGITAL_IO_NORMAL);
    HAL_PIN_Set(PAD_PA00, GPIO_A0, PIN_NOPULL, 1);

    HAL_PIN_SetMode(PAD_PA44, 1, PIN_DIGITAL_IO_NORMAL);
    HAL_PIN_Set(PAD_PA44, GPIO_A44, PIN_NOPULL, 1);

}

void BSP_LCD_Reset(uint8_t high1_low0)
{
    BSP_GPIO_Set(LCD_RESET_PIN, high1_low0, 1);
}


void BSP_LCD_PowerDown(void)
{
#if LCD_TP_SHRAE_POWER
    rt_kprintf("lcd_tp_opened = 0x%x, in BSP_LCD_PowerDown;\n", lcd_tp_opened);
    rt_mutex_take(&lcd_tp_power_mutex, RT_WAITING_FOREVER);
    lcd_tp_set_state(POWER_OFF, LCD_OPENED);
    if (lcd_tp_get_state(LCD_OPENED | TP_OPENED) == POWER_OFF)
    {

        BSP_GPIO_Set(LCD_TP_COM_POWER, 0, 1);// HDK V1.3.0 changto A30
        BSP_GPIO_Set(LCD_TP_COM_POWER1, 0, 1);  //HDK V1.2.0 is GPIO_A09
        BSP_LCD_TP_PowerDown_Config();
        rt_kprintf("lcd_tp_power close by lcd! 0x%x\n", lcd_tp_opened);
    }
    else
    {
        rt_kprintf("BSP_LCD_PowerDown is not_power_close !\n");
    }
    rt_mutex_release(&lcd_tp_power_mutex);
#else
    BSP_GPIO_Set(LCD_TP_COM_POWER, 0, 1);// HDK V1.3.0 changto A30
    BSP_GPIO_Set(LCD_TP_COM_POWER1, 0, 1);  //HDK V1.2.0 is GPIO_A09
    BSP_LCD_TP_PowerDown_Config();
#endif

    BSP_GPIO_Set(PIN_LCD_VCI_EN, 0, 1);
    BSP_LCD_Reset(0);
}

void BSP_LCD_PowerUp(void)
{
#if LCD_TP_SHRAE_POWER
    rt_kprintf("lcd_tp_opened = 0x%x, in BSP_LCD_PowerUp;\n", lcd_tp_opened);

    rt_mutex_take(&lcd_tp_power_mutex, RT_WAITING_FOREVER);
    if (lcd_tp_get_state(LCD_OPENED | TP_OPENED) == POWER_OFF)
    {
        BSP_GPIO_Set(LCD_TP_COM_POWER, 1, 1);//HDK V1.3.0 changto A30
        BSP_GPIO_Set(LCD_TP_COM_POWER1, 1, 1);//HDK V1.2.0 is GPIO_A09
        rt_kprintf("lcd_tp_power open by lcd! 0x%x;\n", lcd_tp_opened);
    }
    else
    {
        rt_kprintf("BSP_LCD_PowerUp is not run power up !\n");

    }
    lcd_tp_set_state(POWER_ON, LCD_OPENED);
    rt_mutex_release(&lcd_tp_power_mutex);
#else
    BSP_GPIO_Set(LCD_TP_COM_POWER, 1, 1);//HDK V1.3.0 changto A30
    BSP_GPIO_Set(LCD_TP_COM_POWER1, 1, 1);//HDK V1.2.0 is GPIO_A09
#endif
    BSP_GPIO_Set(PIN_LCD_VCI_EN, 1, 1);
    HAL_Delay_us(500);      // lcd power on finish ,need 500us
    BSP_LCD_TP_PowerUp_Config();   //if lcd display on ,lcd should reconfig pinmux
}
void BSP_TP_PowerUp_Config()
{

    //Touch
#if 1
    HAL_PIN_Set(PAD_PA42, I2C1_SCL, PIN_PULLUP, 1);     //SCL
    HAL_PIN_Set(PAD_PA41, I2C1_SDA, PIN_PULLUP, 1);     //SDA
    HAL_PIN_Set(PAD_PA43, GPIO_A43, PIN_NOPULL, 1);     // CTP_WKUP_INT
#endif
}

void BSP_TP_PowerUp(void)
{
#if LCD_TP_SHRAE_POWER
    rt_kprintf("lcd_tp_opened = 0x%x, in BSP_TP_PowerUp;\n", lcd_tp_opened);
    rt_mutex_take(&lcd_tp_power_mutex, RT_WAITING_FOREVER);
    if (lcd_tp_get_state(LCD_OPENED | TP_OPENED) == POWER_OFF)
    {
        BSP_GPIO_Set(LCD_TP_COM_POWER, 1, 1);//HDK V1.3.0 changto A30
        BSP_GPIO_Set(LCD_TP_COM_POWER1, 1, 1);//HDK V1.2.0 is GPIO_A09
        rt_kprintf("lcd_tp_power open by tp! 0x%x; \n", lcd_tp_opened);
    }
    else
    {
        rt_kprintf("BSP_TP_PowerUp is not run power open !\n");
    }
    lcd_tp_set_state(POWER_ON, TP_OPENED);
    rt_mutex_release(&lcd_tp_power_mutex);
#else
    BSP_GPIO_Set(LCD_TP_COM_POWER, 1, 1);//HDK V1.3.0 changto A30
    BSP_GPIO_Set(LCD_TP_COM_POWER1, 1, 1);//HDK V1.2.0 is GPIO_A09
#endif

    BSP_GPIO_Set(TP_RESET,  1, 1);
}

void BSP_TP_PowerDown(void)
{
#if LCD_TP_SHRAE_POWER
    rt_kprintf("lcd_tp_opened = 0x%x, in BSP_TP_PowerDown;\n", lcd_tp_opened);
    rt_mutex_take(&lcd_tp_power_mutex, RT_WAITING_FOREVER);
    lcd_tp_set_state(POWER_OFF, TP_OPENED);
    if (lcd_tp_get_state(LCD_OPENED | TP_OPENED) == POWER_OFF)
    {
        BSP_GPIO_Set(LCD_TP_COM_POWER, 0, 1);// HDK V1.3.0 changto A30
        BSP_GPIO_Set(LCD_TP_COM_POWER1, 0, 1);  //HDK V1.2.0 is GPIO_A09
        BSP_LCD_TP_PowerDown_Config();
        rt_kprintf("lcd_tp_power close by tp, 0x%x!\n", lcd_tp_opened);
    }
    else
    {
        rt_kprintf("BSP_TP_PowerDown is not_power_close !\n");
    }
    rt_mutex_release(&lcd_tp_power_mutex);

#endif
    BSP_GPIO_Set(TP_RESET,  0, 1);
}

void BSP_TP_Reset(uint8_t high1_low0)
{
    BSP_GPIO_Set(TP_RESET, high1_low0, 1);
}

#endif