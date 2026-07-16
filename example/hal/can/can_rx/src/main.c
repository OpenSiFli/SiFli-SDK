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

/* ── Target CAN bitrate ───────────────────────────────────── */
#define CAN_BITRATE  400000

#define CAN1 hwp_can1

CAN_HandleTypeDef hcan;

/* ── RX synchronisation semaphore (IT mode) ───────────────── */
static rt_sem_t can_rx_sem = RT_NULL;

/* ── RX filter IDs (match what can_tx sends) ───────────────── */
#define RX_FILTER_PRIMARY   0x123   /* frame sent via primary tx buffer    */
#define RX_FILTER_SECONDARY 0x103   /* frame sent via secondary tx buffer  */


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
    rt_interrupt_enter();

    HAL_CAN_IRQHandler(&hcan);

    rt_interrupt_leave();
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
            const char *type = (rx_header.StdId == RX_FILTER_PRIMARY) ? "Primary" : "Secondary";
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

    /* Derive the prescaler and bit-timing segments from the target bitrate */
    if (HAL_CAN_CalcTiming(&hcan, CAN_BITRATE) < 0)
    {
        rt_kprintf("Failed to calculate CAN timing.\n");
        Error_Handler();
    }

    if (HAL_CAN_Init(&hcan) != HAL_OK)
    {
        rt_kprintf("CAN initialization failed.\n");
        Error_Handler();
    }

    CAN_FilterTypeDef can_filter_config;

    /* Filter 0: frame sent via primary tx buffer of TX board (ID=0x123) */
    can_filter_config.FilterId = RX_FILTER_PRIMARY;
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

    /* Filter 1: frame sent via secondary tx buffer of TX board (ID=0x103) */
    can_filter_config.FilterId = RX_FILTER_SECONDARY;
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
#if defined(SF32LB56X)
    HAL_PIN_Set(PAD_PA03, CAN1_TXD,  PIN_PULLUP, 1);
    HAL_PIN_Set(PAD_PA04, CAN1_RXD,  PIN_PULLUP, 1);
#elif defined(SF32LB58X)
    HAL_PIN_Set(PAD_PA00, CAN1_TXD,  PIN_PULLUP, 1);
    HAL_PIN_Set(PAD_PA03, CAN1_RXD,  PIN_PULLUP, 1);
#endif

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
                        const char *type = (rx_header.StdId == RX_FILTER_PRIMARY) ? "Primary" : "Secondary";
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
    rt_kprintf("Filters: Primary=0x%03X, Secondary=0x%03X\r\n\r\n",
               RX_FILTER_PRIMARY, RX_FILTER_SECONDARY);

    while (1)
    {
        CAN_Receive_Test();
        rt_thread_mdelay(100);
    }
#endif

    return 0;
}
