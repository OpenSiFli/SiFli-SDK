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

/* ── Compile-time switch ──────────────────────────────────── */
/*   0 = polling mode, 1 = interrupt mode                       */
#define USE_CAN_IT   1

#define CAN1 hwp_can1;

CAN_HandleTypeDef hcan;

/* ── RX synchronisation semaphore (IT mode) ───────────────── */
static rt_sem_t can_rx_sem = RT_NULL;

/* ── RX filter IDs (match what can_tx sends) ───────────────── */
#define RX_FILTER_PTB   0x123   /* PTB frame from TX board       */
#define RX_FILTER_STB   0x103   /* STB frame 2 from TX board     */


static void Error_Handler(void)
{
    rt_kprintf("Error occurred\r\n");
    while (1);
}

#if USE_CAN_IT

/* ================================================================
 *  IT branch: ISR bridge (pure RX — no send logic)
 * ================================================================ */

void CAN1_IRQHandler(void)
{
    HAL_CAN_IRQHandler(&hcan);
}

void HAL_CAN_MspInit(CAN_HandleTypeDef *hcan)
{
    if (hcan->Instance == hwp_can1)
    {
        NVIC_EnableIRQ(CAN1_IRQn);
    }
}

void HAL_CAN_RxMsgPendingCallback(CAN_HandleTypeDef *hcan)
{
    /* Notify main loop that a new CAN frame has arrived */
    rt_sem_release(can_rx_sem);
}

#endif /* USE_CAN_IT */

/**
  * @brief  CAN receive test — polling mode (direct hardware read)
  */
static void CAN_Receive_Test(void)
{
    CAN_RxHeaderTypeDef rx_header;
    uint8_t rx_data[8];

    if (HAL_CAN_GetRxMessage(&hcan, &rx_header, rx_data) == HAL_OK)
    {
        if (rx_header.DLC > 0)
        {
            const char *type = (rx_header.StdId == RX_FILTER_PTB) ? "PTB" : "STB";
            rt_kprintf("[RECV %s] ID=0x%03X, DLC=%d, Data=0x%08X 0x%08X\r\n",
                       type, rx_header.StdId, rx_header.DLC,
                       *(uint32_t *)&rx_data[0], *(uint32_t *)&rx_data[4]);
        }
    }
}

/**
  * @brief  CAN Configuration Function
  */
static void CAN_Config(void)
{
    hcan.Instance = CAN1;
    hcan.Init.Prescaler = 0x0b0b;

    if (HAL_CAN_Init(&hcan) != HAL_OK)
    {
        rt_kprintf("CAN initialization failed.\n");
        Error_Handler();
    }

    CAN_FilterTypeDef can_filter_config;

    /* Filter 0: PTB frames from TX board (ID=0x123) */
    can_filter_config.FilterId = RX_FILTER_PTB;
    can_filter_config.FilterMask = 0x7FF;
    can_filter_config.FilterBank = 0;
    can_filter_config.FilterActivation = ENABLE;
    can_filter_config.IDECheckEnable = ENABLE;
    can_filter_config.IDEValue = CAN_ID_STD;

    if (HAL_CAN_ConfigFilter(&hcan, &can_filter_config) != HAL_OK)
    {
        rt_kprintf("Failed to configure CAN filter 0.\n");
        Error_Handler();
    }

    /* Filter 1: STB frame from TX board (ID=0x103) */
    can_filter_config.FilterId = RX_FILTER_STB;
    can_filter_config.FilterMask = 0x7FF;
    can_filter_config.FilterBank = 1;
    can_filter_config.FilterActivation = ENABLE;
    can_filter_config.IDECheckEnable = ENABLE;
    can_filter_config.IDEValue = CAN_ID_STD;

    if (HAL_CAN_ConfigFilter(&hcan, &can_filter_config) != HAL_OK)
    {
        rt_kprintf("Failed to configure CAN filter 1.\n");
        Error_Handler();
    }
}

int main(void)
{
    HAL_PIN_Set(PAD_PA03, CAN1_TXD,  PIN_PULLUP, 1);
    HAL_PIN_Set(PAD_PA04, CAN1_RXD,  PIN_PULLUP, 1);

    HAL_RCC_HCPU_enable2(HPSYS_RCC_ENR2_CAN1, 1);

    CAN_Config();

    if (HAL_CAN_Start(&hcan) != HAL_OK)
    {
        rt_kprintf("Failed to start CAN.\n");
        Error_Handler();
    }

#if USE_CAN_IT
    /* Create semaphore for ISR-to-task synchronisation */
    can_rx_sem = rt_sem_create("can_rx", 0, RT_IPC_FLAG_FIFO);
    if (can_rx_sem == RT_NULL)
    {
        rt_kprintf("Failed to create CAN RX semaphore.\n");
        Error_Handler();
    }

    /* Enable receive interrupt — HAL_CAN_RxMsgPendingCallback fires on each frame */
    HAL_CAN_ActivateNotification(&hcan, CAN_IT_RX_MSG_PENDING);

    rt_kprintf("CAN IT mode: RX-only board (semaphore + GetRxMessage).\r\n");

    while (1)
    {
        /* Wait for ISR to signal that a frame has arrived */
        if (rt_sem_take(can_rx_sem, RT_WAITING_FOREVER) == RT_EOK)
        {
            /* Read frame directly from hardware RBUF */
            CAN_RxHeaderTypeDef rx_header;
            uint8_t rx_data[8];
            while (HAL_CAN_GetRxFifoFillLevel(&hcan) > 0)
            {
                if (HAL_CAN_GetRxMessage(&hcan, &rx_header, rx_data) == HAL_OK)
                {
                    if (rx_header.DLC > 0)
                    {
                        const char *type = (rx_header.StdId == RX_FILTER_PTB) ? "PTB" : "STB";
                        uint32_t data0, data1;
                        memcpy(&data0, &rx_data[0], sizeof(data0));
                        memcpy(&data1, &rx_data[4], sizeof(data1));
                        rt_kprintf("[RECV %s] ID=0x%03X, DLC=%d, Data=0x%08X 0x%08X\r\n",
                                   type, rx_header.StdId, rx_header.DLC, data0, data1);
                    }
                }
            }

        }
    }
#else
    rt_kprintf("CAN polling mode: RX-only board.\r\n");
    rt_kprintf("Filters: PTB=0x%03X, STB=0x%03X\r\n\r\n",
               RX_FILTER_PTB, RX_FILTER_STB);

    while (1)
    {
        CAN_Receive_Test();
        rt_thread_mdelay(100);
    }
#endif

    return 0;
}
