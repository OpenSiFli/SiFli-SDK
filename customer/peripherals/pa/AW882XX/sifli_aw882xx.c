/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
  ******************************************************************************
  * @attention
  *
  * <h2><center>&copy; Copyright (c) 2021 STMicroelectronics.
  * All rights reserved.</center></h2>
  *
  * This software component is licensed by ST under BSD 3-Clause license,
  * the "License"; You may not use this file except in compliance with the
  * License. You may obtain a copy of the License at:
  *                        opensource.org/licenses/BSD-3-Clause
  *
  ******************************************************************************
  */
/* USER CODE END Header */
/* Includes ------------------------------------------------------------------*/

//#include "main.h"
//#include "i2c.h"
//#include "tim.h"
//#include "usart.h"
//#include "gpio.h"
#include <rtthread.h>
#include "board.h"
#include "log.h"
#include "pmic_controller.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include "stdio.h"
#include "stdint.h"
#include "aw_audio_common.h"
#include "aw882xx.h"
#include "aw_params.h"

/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */
#if 0
int fputc(int ch, FILE *f)
{
    uint8_t temp[1] = { ch };
    HAL_UART_Transmit(&huart1, temp, 1, 10);
    return 0;
}
#endif
/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */

/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/

/* USER CODE BEGIN PV */

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
/* USER CODE BEGIN PFP */

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */
static struct rt_i2c_bus_device *aw_i2c_bus = NULL;
#ifdef SOC_SF32LB58X
    #define RESET_GPIO_PIN   88
#else
    #define RESET_GPIO_PIN   65
#endif
#define AW882XX_I2C_NAME  "i2c3"

int aw_dev0_i2c_write_func(uint16_t dev_addr, uint8_t reg_addr,
                           uint8_t *pdata, uint16_t len)
{
    if (rt_i2c_mem_write(aw_i2c_bus, dev_addr, reg_addr, 8, pdata, len) <= 0)
    {
        return -1;
    }
    else
    {
        return 0;
    }
}

int aw_dev0_i2c_read_func(uint16_t dev_addr, uint8_t reg_addr,
                          uint8_t *pdata, uint16_t len)
{
    if (rt_i2c_mem_read(aw_i2c_bus, dev_addr, reg_addr, 8, pdata, len) <= 0)
    {
        return -1;
    }
    else
    {
        return 0;
    }
}

void aw_dev0_reset_gpio_ctl(bool State)
{
    GPIO_PinState PinState = (GPIO_PinState)State;
    HAL_GPIO_WritePin(hwp_gpio1, RESET_GPIO_PIN, PinState);
}


struct aw_fill_info fill_info[] =
{
    {
        .dev_index = AW_DEV_0,
        .i2c_addr = 0x35,
        .mix_chip_count = AW_DEV0_MIX_CHIP_NUM,
        .prof_info = g_dev0_prof_info,
        .i2c_read_func = aw_dev0_i2c_read_func,
        .i2c_write_func = aw_dev0_i2c_write_func,
        .reset_gpio_ctl = aw_dev0_reset_gpio_ctl,
    },

};
void MX_GPIO_Init()
{
    GPIO_InitTypeDef GPIO_InitStruct;

    // set sensor pin to output mode
    GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT;
    GPIO_InitStruct.Pin = RESET_GPIO_PIN;
    GPIO_InitStruct.Pull = GPIO_NOPULL;
    HAL_GPIO_Init(hwp_gpio1, &GPIO_InitStruct);
}
void MX_I2C1_Init()
{
    /* get i2c bus device */
    aw_i2c_bus = rt_i2c_bus_device_find(AW882XX_I2C_NAME);
    if (aw_i2c_bus)
    {
        LOG_D("Find i2c bus device %s\n", AW882XX_I2C_NAME);
        //if (0 != rt_device_open(&(aw_i2c_bus->parent), RT_DEVICE_FLAG_RDWR | RT_DEVICE_FLAG_DMA_RX | RT_DEVICE_FLAG_DMA_TX))
        if (0 != rt_device_open(&(aw_i2c_bus->parent), RT_DEVICE_FLAG_RDWR | RT_DEVICE_FLAG_INT_RX | RT_DEVICE_FLAG_INT_TX))
        {
            LOG_E("Can not open i2c bus %s!!\n", AW882XX_I2C_NAME);
        }
        {
            struct rt_i2c_configuration configuration =
            {
                .mode = 0,
                .addr = 0,
                .timeout = 5000,
                .max_hz  = 400000,
            };

            rt_i2c_configure(aw_i2c_bus, &configuration);
        }

    }
    else
    {
        LOG_E("Can not found i2c bus %s, init fail\n", AW882XX_I2C_NAME);
        //return -1;
    }
}

// chn_sel: 1: left channel 2: right channel 3: mono
void sifli_aw882xx_start(uint32_t samplerate, uint8_t chn_sel)
{
    rt_kprintf("aw882xx start\n");
    if (samplerate == 44100)
    {
        aw88xx_hal_iface_fops->set_fs(AW_DEV_0, 44000, chn_sel);
    }
    else
    {
        aw88xx_hal_iface_fops->set_fs(AW_DEV_0, samplerate, chn_sel);
    }

    /*awinic:The platform needs to output I2s first, and then ctl start*/
    aw88xx_hal_iface_fops->ctrl_state(AW_DEV_0, START);
}

void sifli_aw882xx_stop()
{
    /*awinic:The platform needs to control the stop first and then stop I2S*/
    aw88xx_hal_iface_fops->ctrl_state(AW_DEV_0, STOP);
    rt_kprintf("aw882xx stop\n");
}

int rt_aw882xx_init()
{
    pmic_device_control(PMIC_OUT_1V8_LVSW100_1, 1, 1);
    AW_MS_DELAY(AW_10_MS);
    /* Initialize all configured peripherals */
    MX_GPIO_Init();
    MX_I2C1_Init();
    /*awinic:initialization*/
    aw88xx_hal_iface_fops->init((void *)&fill_info[AW_DEV_0]);
    /*awinic:set mode id 0, "aw_params.h" contains id information*/
    aw88xx_hal_iface_fops->set_profile_byname(AW_DEV_0, "Music");

    /*awinic:The platform needs to output I2s first, and then ctl start*/
    //aw88xx_hal_iface_fops->ctrl_state(AW_DEV_0, START);
    rt_kprintf("aw882xx init\n");
    return 0;
}

//INIT_COMPONENT_EXPORT(rt_aw882xx_init);

/* USER CODE END 0 */

/**
  * @brief  The application entry point.
  * @retval int
  */
#if 0
int main(void)
{
    /* USER CODE BEGIN 1 */

    /* USER CODE END 1 */

    /* MCU Configuration--------------------------------------------------------*/

    /* Reset of all peripherals, Initializes the Flash interface and the Systick. */
    //HAL_Init();

    /* USER CODE BEGIN Init */

    /* USER CODE END Init */

    /* Configure the system clock */
    //SystemClock_Config();

    /* USER CODE BEGIN SysInit */

    /* USER CODE END SysInit */

    /* Initialize all configured peripherals */
    MX_GPIO_Init();
    MX_I2C1_Init();
    //MX_I2C2_Init();
    //MX_USART1_UART_Init();
    //MX_TIM3_Init();
    /* USER CODE BEGIN 2 */


    /*awinic:Control interface of dev0*/
    /*awinic:initialization*/
    aw88xx_hal_iface_fops->init((void *)&fill_info[AW_DEV_0]);
    /*awinic:set mode id 0, "aw_params.h" contains id information*/
    aw88xx_hal_iface_fops->set_profile_byname(AW_DEV_0, "Music");

    /*awinic:The platform needs to output I2s first, and then ctl start*/
    aw88xx_hal_iface_fops->ctrl_state(AW_DEV_0, START);

    /*awinic:The platform needs to control the stop first and then stop I2S*/
    //aw88xx_hal_iface_fops->ctrl_state(AW_DEV_0, STOP);
    //aw88xx_hal_iface_fops->deinit(AW_DEV_0);


    /*awinic:Control interface of dev1*/
    //aw88xx_hal_iface_fops->init((void *)&fill_info[AW_DEV_1]);
    //aw88xx_hal_iface_fops->set_profile_byname(AW_DEV_1, "Music");
    //aw88xx_hal_iface_fops->ctrl_state(AW_DEV_1, START);

    /*awinic:If enable the monitor function, palese enable the timer*/
    //HAL_TIM_Base_Start_IT(&htim3);


    /* USER CODE END 2 */

    /* Infinite loop */
    /* USER CODE BEGIN WHILE */
    while (1)
    {
        /*awinic:If enable the monitor function, palese enable monitor work*/
        //aw88xx_hal_iface_fops->monitor_work(AW_DEV_0);
        //aw88xx_hal_iface_fops->monitor_work(AW_DEV_1);

        /*awinic:If enable the irq function, palese enable irq_handler*/
        //aw88xx_hal_iface_fops->irq_handler(AW_DEV_0);

        /* USER CODE END WHILE */

        /* USER CODE BEGIN 3 */
    }

    /* USER CODE END 3 */
}
#endif
#if 0
/**
  * @brief System Clock Configuration
  * @retval None
  */
void SystemClock_Config(void)
{
    RCC_OscInitTypeDef RCC_OscInitStruct = {0};
    RCC_ClkInitTypeDef RCC_ClkInitStruct = {0};
    RCC_PeriphCLKInitTypeDef PeriphClkInitStruct = {0};

    /** Supply configuration update enable
    */
    HAL_PWREx_ConfigSupply(PWR_LDO_SUPPLY);
    /** Configure the main internal regulator output voltage
    */
    __HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE1);

    while (!__HAL_PWR_GET_FLAG(PWR_FLAG_VOSRDY)) {}
    /** Initializes the RCC Oscillators according to the specified parameters
    * in the RCC_OscInitTypeDef structure.
    */
    RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSE;
    RCC_OscInitStruct.HSEState = RCC_HSE_ON;
    RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
    RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSE;
    RCC_OscInitStruct.PLL.PLLM = 2;
    RCC_OscInitStruct.PLL.PLLN = 200;
    RCC_OscInitStruct.PLL.PLLP = 2;
    RCC_OscInitStruct.PLL.PLLQ = 2;
    RCC_OscInitStruct.PLL.PLLR = 2;
    RCC_OscInitStruct.PLL.PLLRGE = RCC_PLL1VCIRANGE_2;
    RCC_OscInitStruct.PLL.PLLVCOSEL = RCC_PLL1VCOWIDE;
    RCC_OscInitStruct.PLL.PLLFRACN = 0;
    if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
    {
        Error_Handler();
    }
    /** Initializes the CPU, AHB and APB buses clocks
    */
    RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK | RCC_CLOCKTYPE_SYSCLK
                                  | RCC_CLOCKTYPE_PCLK1 | RCC_CLOCKTYPE_PCLK2
                                  | RCC_CLOCKTYPE_D3PCLK1 | RCC_CLOCKTYPE_D1PCLK1;
    RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
    RCC_ClkInitStruct.SYSCLKDivider = RCC_SYSCLK_DIV1;
    RCC_ClkInitStruct.AHBCLKDivider = RCC_HCLK_DIV2;
    RCC_ClkInitStruct.APB3CLKDivider = RCC_APB3_DIV2;
    RCC_ClkInitStruct.APB1CLKDivider = RCC_APB1_DIV2;
    RCC_ClkInitStruct.APB2CLKDivider = RCC_APB2_DIV2;
    RCC_ClkInitStruct.APB4CLKDivider = RCC_APB4_DIV2;

    if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_2) != HAL_OK)
    {
        Error_Handler();
    }
    PeriphClkInitStruct.PeriphClockSelection = RCC_PERIPHCLK_USART1 | RCC_PERIPHCLK_I2C2
            | RCC_PERIPHCLK_I2C1;
    PeriphClkInitStruct.Usart16ClockSelection = RCC_USART16CLKSOURCE_D2PCLK2;
    PeriphClkInitStruct.I2c123ClockSelection = RCC_I2C123CLKSOURCE_D2PCLK1;
    if (HAL_RCCEx_PeriphCLKConfig(&PeriphClkInitStruct) != HAL_OK)
    {
        Error_Handler();
    }
}

/* USER CODE BEGIN 4 */

/*awinic:call aw_irq interface in EXTI*/
void HAL_GPIO_EXTI_Callback(uint16_t GPIO_Pin)
{
    if (GPIO_Pin == GPIO_PIN_2)
    {
        aw88xx_hal_iface_fops->irq_trigger(AW_DEV_0);
    }
}

/*awinic:If enable the monitor function, palese set monitor status in TIM_PeriodElapsedCallback*/
void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim)
{
    //if(htim ==(&htim3))
    {
        //aw88xx_hal_iface_fops->monitor_set_status(AW_DEV_0);
        //aw88xx_hal_iface_fops->monitor_set_status(AW_DEV_1);
    }
}

/* USER CODE END 4 */

/**
  * @brief  This function is executed in case of error occurrence.
  * @retval None
  */
void Error_Handler(void)
{
    /* USER CODE BEGIN Error_Handler_Debug */
    /* User can add his own implementation to report the HAL error return state */

    /* USER CODE END Error_Handler_Debug */
}

#ifdef  USE_FULL_ASSERT
/**
  * @brief  Reports the name of the source file and the source line number
  *         where the assert_param error has occurred.
  * @param  file: pointer to the source file name
  * @param  line: assert_param error line source number
  * @retval None
  */
void assert_failed(uint8_t *file, uint32_t line)
{
    /* USER CODE BEGIN 6 */
    /* User can add his own implementation to report the file name and line number,
       tex: printf("Wrong parameters value: file %s on line %d\r\n", file, line) */
    /* USER CODE END 6 */
}
#endif

#endif /* USE_FULL_ASSERT */

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
