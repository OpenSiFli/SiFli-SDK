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

/* ── Target CAN bitrate ───────────────────────────────────── */
#define CAN_BITRATE  400000

#define CAN1 hwp_can1

CAN_HandleTypeDef hcan;

/* ── Primary tx IDs (original test) ───────────────────────── */
#define PRIMARY_SEND_ID       0x123
#define PRIMARY_RECV_FILTER   0x234

/* ── Secondary tx IDs (new test) ──────────────────────────── */
/* This board sends 3 secondary frames, peer only receives the 3rd one */
#define SECONDARY_SEND_ID_0     0x101
#define SECONDARY_SEND_ID_1     0x102
#define SECONDARY_SEND_ID_2     0x103
/* Peer sends 3 secondary frames, this board only receives the 3rd one */
#define SECONDARY_RECV_FILTER   0x203


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

void HAL_CAN_TxPrimaryCompleteCallback(CAN_HandleTypeDef *hcan)
{
    HAL_CAN_DeactivateNotification(hcan, CAN_IT_TX_PTB_EMPTY);
}

void HAL_CAN_TxSecondaryCompleteCallback(CAN_HandleTypeDef *hcan)
{
    HAL_CAN_DeactivateNotification(hcan, CAN_IT_TX_STB_EMPTY);
}

static void CAN_Primary_Transmit_IT_Test(void)
{
    CAN_TxHeaderTypeDef tx_header =
    {
        .StdId = PRIMARY_SEND_ID, .IDE = CAN_ID_STD, .RTR = CAN_RTR_DATA, .DLC = 8,
    };
    uint32_t tx_data[2] = {0x11223344, 0x55667788};
    HAL_StatusTypeDef status;

    HAL_CAN_ActivateNotification(&hcan, CAN_IT_TX_PTB_EMPTY);

    /* Fill the primary tx buffer and start — CAN controller does the rest */
    status = HAL_CAN_AddPrimaryTxMessage(&hcan, &tx_header, (uint8_t *)tx_data);
    if (status != HAL_OK)
    {
        rt_kprintf("[TX-IT] Primary transmit failed\r\n");
    }
    else
    {
        rt_kprintf("[TX-IT] Primary submit: ID=0x%03X, Data=0x%08X 0x%08X\r\n",
                   tx_header.StdId, tx_data[0], tx_data[1]);
    }
}

static void CAN_Secondary_Transmit_IT_Test(void)
{
    CAN_TxHeaderTypeDef tx_header =
    {
        .IDE = CAN_ID_STD, .RTR = CAN_RTR_DATA, .DLC = 8,
    };
    uint32_t tx_data[2];
    HAL_StatusTypeDef status;

    HAL_CAN_ActivateNotification(&hcan, CAN_IT_TX_STB_EMPTY);
    tx_header.StdId = SECONDARY_SEND_ID_0;
    tx_data[0] = 0xAAAA0001;
    tx_data[1] = 0xBBBB0001;
    status = HAL_CAN_AddSecondaryTxMessage(&hcan, &tx_header, (uint8_t *)tx_data);
    if (status != HAL_OK)
    {
        rt_kprintf("[TX1-IT] Secondary transmit failed\r\n");
        return;
    }
    tx_header.StdId = SECONDARY_SEND_ID_1;
    tx_data[0] = 0xAAAA0002;
    tx_data[1] = 0xBBBB0002;
    status = HAL_CAN_AddSecondaryTxMessage(&hcan, &tx_header, (uint8_t *)tx_data);
    if (status != HAL_OK)
    {
        rt_kprintf("[TX2-IT] Secondary transmit failed\r\n");
        return;
    }
    tx_header.StdId = SECONDARY_SEND_ID_2;
    tx_data[0] = 0xAAAA0003;
    tx_data[1] = 0xBBBB0003;
    status = HAL_CAN_AddSecondaryTxMessage(&hcan, &tx_header, (uint8_t *)tx_data);
    if (status != HAL_OK)
    {
        rt_kprintf("[TX3-IT] Secondary transmit failed\r\n");
        return;
    }

    rt_kprintf("[TX-IT] Secondary submit: 3 frames: 0x%03X, 0x%03X, 0x%03X\r\n",
               SECONDARY_SEND_ID_0, SECONDARY_SEND_ID_1, SECONDARY_SEND_ID_2);
}

#endif /* USE_CAN_IT */

/**
  * @brief  Original primary tx buffer send test
  */
static void CAN_Primary_Send_Test(void)
{
    uint32_t tx_data[2] = {0x11223344, 0x55667788};
    CAN_TxHeaderTypeDef tx_header;

    tx_header.StdId = PRIMARY_SEND_ID;
    tx_header.ExtId = 0x00;
    tx_header.IDE = CAN_ID_STD;
    tx_header.RTR = CAN_RTR_DATA;
    tx_header.DLC = 8;

    if (HAL_CAN_AddPrimaryTxMessage(&hcan, &tx_header, (uint8_t *)tx_data) != HAL_OK)
    {
        rt_kprintf("Primary send failed\r\n");
        return;
    }

    rt_kprintf("[Primary SEND] ID=0x%03X, Data=0x%08X 0x%08X\r\n",
               tx_header.StdId, tx_data[0], tx_data[1]);
}

/**
  * @brief  Secondary tx send test: queue 3 frames with different IDs, then send all
  */
static void CAN_Secondary_Send_Test(void)
{
    CAN_TxHeaderTypeDef tx_header;
    uint32_t tx_data[2];

    tx_header.IDE = CAN_ID_STD;
    tx_header.RTR = CAN_RTR_DATA;
    tx_header.DLC = 8;

    /* Frame 0 ── will be filtered out by peer */
    tx_header.StdId = SECONDARY_SEND_ID_0;
    tx_data[0] = 0xAAAA0001;
    tx_data[1] = 0xBBBB0001;
    if (HAL_CAN_AddSecondaryTxMessage(&hcan, &tx_header, (uint8_t *)tx_data) != HAL_OK)
    {
        rt_kprintf("Secondary queue frame 0 failed\r\n");
        return;
    }

    /* Frame 1 ── will be filtered out by peer */
    tx_header.StdId = SECONDARY_SEND_ID_1;
    tx_data[0] = 0xAAAA0002;
    tx_data[1] = 0xBBBB0002;
    if (HAL_CAN_AddSecondaryTxMessage(&hcan, &tx_header, (uint8_t *)tx_data) != HAL_OK)
    {
        rt_kprintf("Secondary queue frame 1 failed\r\n");
        return;
    }

    /* Frame 2 ── will be accepted by peer (filter match) */
    tx_header.StdId = SECONDARY_SEND_ID_2;
    tx_data[0] = 0xAAAA0003;
    tx_data[1] = 0xBBBB0003;
    if (HAL_CAN_AddSecondaryTxMessage(&hcan, &tx_header, (uint8_t *)tx_data) != HAL_OK)
    {
        rt_kprintf("Secondary queue frame 2 failed\r\n");
        return;
    }

    while (HAL_CAN_IsTxMessagePending(&hcan))
    {
        /* Wait for all secondary frames to be sent */
    }

    rt_kprintf("[Secondary SEND] 3 frames: 0x%03X, 0x%03X, 0x%03X\r\n",
               SECONDARY_SEND_ID_0, SECONDARY_SEND_ID_1, SECONDARY_SEND_ID_2);
}
/**
  * @brief  CAN receive test: handles both filtered frame IDs
  */
static void CAN_Receive_Test(void)
{
    CAN_RxHeaderTypeDef rx_header;
    uint8_t rx_data[8];

    /* Read all available frames in FIFO (up to 4 per cycle) */
    for (int i = 0; i < 4; i++)
    {
        if (HAL_CAN_GetRxMessage(&hcan, &rx_header, rx_data) == HAL_OK)
        {
            if (rx_header.DLC > 0)
            {
                const char *type = (rx_header.StdId == PRIMARY_RECV_FILTER) ? "Primary" : "Secondary";
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

    /* Filter 0: primary tx frame from peer (exact match) */
    can_filter_config.FilterId = PRIMARY_RECV_FILTER;
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

    /* Filter 1: secondary tx filtered frame from peer (exact match) */
    can_filter_config.FilterId = SECONDARY_RECV_FILTER;
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
    rt_kprintf("CAN IT mode: Board A (send-first).\r\n");

    while (1)
    {
        CAN_Primary_Transmit_IT_Test();
        rt_thread_mdelay(500);
        CAN_Secondary_Transmit_IT_Test();
        rt_thread_mdelay(2000);
    }
#else
    rt_kprintf("CAN Primary+Secondary test start (Board A: send-first).\r\n");
    rt_kprintf("Primary filter: 0x%03X, Secondary filter: 0x%03X\r\n\r\n",
               PRIMARY_RECV_FILTER, SECONDARY_RECV_FILTER);

    while (1)
    {
        /* Phase 1: primary send + secondary send 3 frames */
        CAN_Primary_Send_Test();
        CAN_Secondary_Send_Test();
        rt_thread_mdelay(1000);

        /* Phase 2: receive filtered frames from peer */
        CAN_Receive_Test();
        rt_thread_mdelay(1000);
    }
#endif

    return 0;
}
