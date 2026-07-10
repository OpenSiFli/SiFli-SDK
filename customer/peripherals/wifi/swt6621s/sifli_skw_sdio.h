/*
 * SPDX-FileCopyrightText: 2026 SiFli Technologies(Nanjing) Co., Ltd
 *
 * SPDX-License-Identifier: Apache-2.0
 */
#ifndef __SIFLI_SKW_SDIO_H__
#define __SIFLI_SKW_SDIO_H__
#include <rtthread.h>
#include <stdbool.h>

/**
 * @brief Close an SDIO port by ID.
 * @param id Port identifier.
 * @return 0 on success, non-zero on failure.
 */
int skw_close_sdio_port(int id);

/**
 * @brief Send data through an SDIO port.
 * @param portno Port number.
 * @param buffer Data buffer to send.
 * @param size   Number of bytes to send.
 * @return Number of bytes sent, or negative on failure.
 */
int send_data(int portno, char *buffer, int size);

/**
 * @brief Open an SDIO port with receive callback.
 * @param id       Port identifier.
 * @param callback Function called when data is received on this port.
 * @param data     User context passed to the callback.
 * @return 0 on success, non-zero on failure.
 */
int skw_open_sdio_port(int id, void *callback, void *data);

/**
 * @brief Send WiFi data using scatter-gather list.
 * @param portno Port number.
 * @param sg     Scatter-gather buffer list.
 * @param sg_num Number of scatter-gather entries.
 * @param total  Total bytes to send.
 * @return Number of bytes sent, or negative on failure.
 */
int wifi_send_data(int portno, char *sg, int sg_num, int total);

/**
 * @brief Receive data from an SDIO port.
 * @param portno Port number.
 * @param buf    Buffer to store received data.
 * @param size   Buffer size in bytes.
 * @return Number of bytes received, or negative on failure.
 */
int recv_data(int portno, char *buf, int size);


#endif /* SKW_SDIO_H */
