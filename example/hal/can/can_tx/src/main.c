/*
 * SPDX-FileCopyrightText: 2026 SiFli Technologies(Nanjing) Co., Ltd
 *
 * SPDX-License-Identifier: Apache-2.0
 */
#include "rtthread.h"
#include "bf0_hal.h"
#include "drv_io.h"
#include "stdio.h"
#include "string.h"
#include "bf0_hal_can.h"

#define CAN1 hwp_can1;

CAN_HandleTypeDef hcan;


static void Error_Handler(void)
{
    rt_kprintf("Error occurred\r\n");

    HAL_ASSERT(0);

}

/**
  * @brief  CAN Send test
  * @param  None
  * @retval None
  */
static void CAN_Send_Test(void)
{
    uint32_t tx_data[2] = {0x11223344, 0x55667788};
    CAN_TxHeaderTypeDef tx_header;

    /* Configure transmission header */
    tx_header.StdId = 0x123;
    tx_header.ExtId = 0x00;
    tx_header.IDE = CAN_ID_STD;//
    tx_header.RTR = CAN_RTR_DATA;//
    tx_header.DLC = 8;

    /* Send data */
    if (HAL_CAN_AddTxMessage(&hcan, &tx_header, tx_data) != HAL_OK)
    {
        rt_kprintf("Send failed\r\n");
        return;
    }
    else
    {
        rt_kprintf("Send successful: ID=0x%03X, Data=", tx_header.StdId);
        for (int i = 0; i < 2; i++)
        {
            rt_kprintf("0x%08X ", tx_data[i]);
        }
        rt_kprintf("\r\n");
    }
}
/**
  * @brief  CAN receiving function
  * @param  None
  * @retval None
  */
static void CAN_Receive_Test(void)
{
    CAN_RxHeaderTypeDef rx_header;
    uint8_t rx_data[8];
    uint32_t original_rx_data[2];

    rt_kprintf("Switch to receiving mode and wait for data...\r\n");

    /* Check and receive the data */
    if (HAL_CAN_GetRxMessage(&hcan, &rx_header, rx_data) == HAL_OK)
    {
        rt_kprintf("Received data: ID=0x%03X, DLC=%d, ", rx_header.StdId, rx_header.DLC);
    }
    original_rx_data[0] = *(uint32_t *)&rx_data[0];
    original_rx_data[1] = *(uint32_t *)&rx_data[4];
    rt_kprintf("Data: 0x%08X, 0x%08X\r\n", original_rx_data[0], original_rx_data[1]);

}

/**
  * @brief  CAN Configuration Function
  * @param  None
  * @retval None
  */
static void CAN_Config(void)
{
    /* Initialization parameters */
    hcan.Instance = CAN1;
    hcan.Init.Prescaler = 0x0b0b;        //Baud rate prescaler,1/48MHz/（11+1）=0.25us。1/(0.25us*10) = 400kbps

    /* Initialize CAN peripheral */
    if (HAL_CAN_Init(&hcan) != HAL_OK)
    {
        rt_kprintf("CAN initialization failed.\n");
        Error_Handler();
    }

    /* Configure CAN filter */
    CAN_FilterTypeDef can_filter_config;

    // Configure standard frame filter (ID=0x234)
    can_filter_config.FilterId = 0x234;
    can_filter_config.FilterMask = 0x7FF; // Exact match for standard ID
    can_filter_config.FilterBank = 0;
    can_filter_config.FilterActivation = ENABLE;
    can_filter_config.IDECheckEnable = ENABLE;
    can_filter_config.IDEValue = CAN_ID_STD; // Only accept standard frames

    if (HAL_CAN_ConfigFilter(&hcan, &can_filter_config) != HAL_OK)//Configure filter
    {
        rt_kprintf("Failed to configure CAN filter.\n");
        Error_Handler();
    }
}

int main(void)
{
    HAL_RCC_HCPU_enable2(HPSYS_RCC_ENR2_CAN1, 1);

    CAN_Config();

    /* Start CAN */
    if (HAL_CAN_Start(&hcan) != HAL_OK)
    {
        rt_kprintf("Failed to start CAN.\n");
        Error_Handler();
    }

    rt_kprintf("CAN 2.0 start test.\r\n");


    rt_kprintf("This board operates as the sending end.\r\n");

    while (1)
    {
        CAN_Send_Test();
        HAL_Delay(500);
        CAN_Receive_Test();
        HAL_Delay(500);
    }


    /* Infinite loop */
    while (1)
    {
        // Delay for 1000 ms to yield CPU time to other threads
        rt_thread_mdelay(1000);
    }

    return 0;
}
