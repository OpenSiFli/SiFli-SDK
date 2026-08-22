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
/*   0 = polling mode (original), 1 = interrupt mode            */
#define USE_CAN_IT   1

#define CAN1 hwp_can1;

CAN_HandleTypeDef hcan;

/* ── PTB IDs (original test) ───────────────────────────────── */
#define PTB_SEND_ID       0x123
#define PTB_RECV_FILTER   0x234

/* ── STB IDs (new test) ────────────────────────────────────── */
/* This board sends 3 STB frames, peer only receives the 3rd one */
#define STB_SEND_ID_0     0x101
#define STB_SEND_ID_1     0x102
#define STB_SEND_ID_2     0x103
/* Peer sends 3 STB frames, this board only receives the 3rd one */
#define STB_RECV_FILTER   0x203


static void Error_Handler(void)
{
    rt_kprintf("Error occurred\r\n");
    HAL_ASSERT(0);
}

#if USE_CAN_IT

/* ================================================================
 *  IT branch: ISR bridge + callbacks + IT send functions
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

void HAL_CAN_TxPtbCompleteCallback(CAN_HandleTypeDef *hcan)
{
    HAL_CAN_DeactivateNotification(hcan, CAN_IT_TX_PTB_EMPTY);
}

void HAL_CAN_TxStbCompleteCallback(CAN_HandleTypeDef *hcan)
{
    HAL_CAN_DeactivateNotification(hcan, CAN_IT_TX_STB_EMPTY);
}

static void CAN_PTB_Transmit_IT_Test(void)
{
    CAN_TxHeaderTypeDef tx_header =
    {
        .StdId = PTB_SEND_ID, .IDE = CAN_ID_STD, .RTR = CAN_RTR_DATA, .DLC = 8,
    };
    uint32_t tx_data[2] = {0x11223344, 0x55667788};
    HAL_StatusTypeDef status;

    HAL_CAN_ActivateNotification(&hcan, CAN_IT_TX_PTB_EMPTY);

    /* Fill PTB and start — CAN controller does the rest */
    status = HAL_CAN_AddPrimaryTxMessage(&hcan, &tx_header, (uint8_t *)tx_data);
    if (status != HAL_OK)
    {
        rt_kprintf("[TX-IT] PTB transmit failed\r\n");
    }

}

static void CAN_STB_Transmit_IT_Test(void)
{
    CAN_TxHeaderTypeDef tx_header =
    {
        .IDE = CAN_ID_STD, .RTR = CAN_RTR_DATA, .DLC = 8,
    };
    uint32_t tx_data[2];
    HAL_StatusTypeDef status;

    HAL_CAN_ActivateNotification(&hcan, CAN_IT_TX_STB_EMPTY);
    tx_header.StdId = STB_SEND_ID_0;
    tx_data[0] = 0xAAAA0001;
    tx_data[1] = 0xBBBB0001;
    status = HAL_CAN_AddSecondaryTxMessage(&hcan, &tx_header, (uint8_t *)tx_data);
    if (status != HAL_OK)
    {
        rt_kprintf("[TX1-IT] STB transmit failed\r\n");
    }
    tx_header.StdId = STB_SEND_ID_1;
    tx_data[0] = 0xAAAA0002;
    tx_data[1] = 0xBBBB0002;
    status = HAL_CAN_AddSecondaryTxMessage(&hcan, &tx_header, (uint8_t *)tx_data);
    if (status != HAL_OK)
    {
        rt_kprintf("[TX2-IT] STB transmit failed\r\n");
    }
    tx_header.StdId = STB_SEND_ID_2;
    tx_data[0] = 0xAAAA0003;
    tx_data[1] = 0xBBBB0003;
    status = HAL_CAN_AddSecondaryTxMessage(&hcan, &tx_header, (uint8_t *)tx_data);
    if (status != HAL_OK)
    {
        rt_kprintf("[TX3-IT] STB transmit failed\r\n");
    }
}

#endif /* USE_CAN_IT */

/**
  * @brief  Original PTB send test
  */
static void CAN_PTB_Send_Test(void)
{
    uint32_t tx_data[2] = {0x11223344, 0x55667788};
    CAN_TxHeaderTypeDef tx_header;

    tx_header.StdId = PTB_SEND_ID;
    tx_header.ExtId = 0x00;
    tx_header.IDE = CAN_ID_STD;
    tx_header.RTR = CAN_RTR_DATA;
    tx_header.DLC = 8;

    if (HAL_CAN_AddPrimaryTxMessage(&hcan, &tx_header, (uint8_t *)tx_data) != HAL_OK)
    {
        rt_kprintf("PTB send failed\r\n");
        return;
    }

    rt_kprintf("[PTB SEND] ID=0x%03X, Data=0x%08X 0x%08X\r\n",
               tx_header.StdId, tx_data[0], tx_data[1]);
}

/**
  * @brief  STB send test: queue 3 frames with different IDs, then send all
  */
static void CAN_STB_Send_Test(void)
{
    CAN_TxHeaderTypeDef tx_header;
    uint32_t tx_data[2];

    tx_header.IDE = CAN_ID_STD;
    tx_header.RTR = CAN_RTR_DATA;
    tx_header.DLC = 8;

    /* Frame 0 ── will be filtered out by peer */
    tx_header.StdId = STB_SEND_ID_0;
    tx_data[0] = 0xAAAA0001;
    tx_data[1] = 0xBBBB0001;
    if (HAL_CAN_AddSecondaryTxMessage(&hcan, &tx_header, (uint8_t *)tx_data) != HAL_OK)
    {
        rt_kprintf("STB queue frame 0 failed\r\n");
        return;
    }

    /* Frame 1 ── will be filtered out by peer */
    tx_header.StdId = STB_SEND_ID_1;
    tx_data[0] = 0xAAAA0002;
    tx_data[1] = 0xBBBB0002;
    if (HAL_CAN_AddSecondaryTxMessage(&hcan, &tx_header, (uint8_t *)tx_data) != HAL_OK)
    {
        rt_kprintf("STB queue frame 1 failed\r\n");
        return;
    }

    /* Frame 2 ── will be accepted by peer (filter match) */
    tx_header.StdId = STB_SEND_ID_2;
    tx_data[0] = 0xAAAA0003;
    tx_data[1] = 0xBBBB0003;
    if (HAL_CAN_AddSecondaryTxMessage(&hcan, &tx_header, (uint8_t *)tx_data) != HAL_OK)
    {
        rt_kprintf("STB queue frame 2 failed\r\n");
        return;
    }

    while (HAL_CAN_IsTxMessagePending(&hcan))
    {
        /* Wait for all STB frames to be sent */
    }

    rt_kprintf("[STB SEND] 3 frames: 0x%03X, 0x%03X, 0x%03X\r\n",
               STB_SEND_ID_0, STB_SEND_ID_1, STB_SEND_ID_2);
}
/**
  * @brief  CAN receive test: handles both PTB and STB filtered frames
  */
static void CAN_Receive_Test(void)
{
    CAN_RxHeaderTypeDef rx_header;
    uint8_t rx_data[8];

    /* Read all available frames in FIFO (up to 4 per cycle: 1 PTB + 3 STB) */
    for (int i = 0; i < 4; i++)
    {
        if (HAL_CAN_GetRxMessage(&hcan, &rx_header, rx_data) == HAL_OK)
        {
            if (rx_header.DLC > 0)
            {
                const char *type = (rx_header.StdId == PTB_RECV_FILTER) ? "PTB" : "STB";
                rt_kprintf("[RECV %s] ID=0x%03X, DLC=%d, Data=0x%08X 0x%08X\r\n",
                           type, rx_header.StdId, rx_header.DLC,
                           *(uint32_t *)&rx_data[0], *(uint32_t *)&rx_data[4]);
            }
        }
        else
        {
            break;  /* FIFO empty */
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

    /* Filter 0: PTB frames from peer (exact match) */
    can_filter_config.FilterId = PTB_RECV_FILTER;
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

    /* Filter 1: STB filtered frame from peer (exact match) */
    can_filter_config.FilterId = STB_RECV_FILTER;
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
    rt_kprintf("CAN IT mode: Board A (send-first).\r\n");

    while (1)
    {
        CAN_PTB_Transmit_IT_Test();
        rt_thread_mdelay(500);
        CAN_STB_Transmit_IT_Test();
        rt_thread_mdelay(2000);
    }
#else
    rt_kprintf("CAN PTB+STB test start (Board A: send-first).\r\n");
    rt_kprintf("PTB filter: 0x%03X, STB filter: 0x%03X\r\n\r\n",
               PTB_RECV_FILTER, STB_RECV_FILTER);

    while (1)
    {
        /* Phase 1: PTB send + STB send 3 frames */
        CAN_PTB_Send_Test();
        CAN_STB_Send_Test();
        rt_thread_mdelay(1000);

        /* Phase 2: receive PTB + STB filtered frame from peer */
        CAN_Receive_Test();
        rt_thread_mdelay(1000);
    }
#endif

    return 0;
}
