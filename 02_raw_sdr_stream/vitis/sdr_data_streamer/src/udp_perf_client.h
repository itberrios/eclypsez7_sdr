/*
 * Copyright (C) 2017 - 2019 Xilinx, Inc.
 * Copyright (C) 2022 - 2024 Advanced Micro Devices, Inc.
 * All rights reserved.
 * SPDX-License-Identifier: MIT
 */

#ifndef __UDP_PERF_CLIENT_H_
#define __UDP_PERF_CLIENT_H_

#ifdef __cplusplus
extern "C" {
#endif

#include "lwipopts.h"
#include "lwip/ip_addr.h"
#include "lwip/err.h"
#include "lwip/udp.h"
#include "lwip/inet.h"
#include "lwip/sockets.h"
#include "lwip/sys.h"
#include "errno.h"
#include <sleep.h>
#include "xil_printf.h"
#include "xlwipconfig.h"

#include <stdint.h>
#include "xiltimer.h"
#include "xgpio.h"
#include "xaxidma.h"
#include "xparameters.h"

/* Used as indices into kLabel[] */
enum {
    KCONV_UNIT,
    KCONV_KILO,
    KCONV_MEGA,
    KCONV_GIGA,
};

/* Use static const in header to prevent multiple-definition linker errors */
static const char kLabel[] = { ' ', 'K', 'M', 'G' };

/* Used as type of print */
enum measure_t {
    BYTES,
    SPEED
};

/* Report Type */
enum report_type {
    INTER_REPORT,
    UDP_DONE_CLIENT,
    UDP_ABORTED_REMOTE
};

struct interim_report {
    u64_t start_time;
    u64_t last_report_time;
    u32_t total_bytes;
};

struct perf_stats {
    u8_t client_id;
    u64_t start_time;
    u64_t total_bytes;
    u64_t cnt_datagrams;
    struct interim_report i_report;
};

/* Seconds between periodic bandwidth reports */
#define INTERIM_REPORT_INTERVAL     5

/* Client port to connect */
#define UDP_CONN_PORT               5001

/* Time in seconds to transmit packets (set to 0 for infinite streaming) */
#define UDP_TIME_INTERVAL           300

/* Server to connect with */
#define UDP_SERVER_IP_ADDRESS       "192.168.1.100"

/* Hardware Address Definitions */
#define DMA_BASEADDR                XPAR_XAXIDMA_0_BASEADDR
#define CFG_GPIO_BASEADDR           XPAR_XGPIO_0_BASEADDR
#define CFG_GPIO_SDR_ENABLE         1U
#define CFG_GPIO_SET_PKT_SZ         2U

/* Packet Header Layout (16 bytes total) */
#define PACKET_ID_SZ                4U
#define SYNC_WORD_SZ                4U
#define TIMESTAMP_SZ                8U

#define PACKET_ID_OFFSET            0U
#define SYNC_WORD_OFFSET            (PACKET_ID_OFFSET + PACKET_ID_SZ)   // 4
#define TIMESTAMP_OFFSET            (SYNC_WORD_OFFSET + SYNC_WORD_SZ)   // 8
#define DATA_OFFSET                 (TIMESTAMP_OFFSET + TIMESTAMP_SZ)   // 16

/* Data Burst & Packet Frame Definitions */
#define MAX_UDP_BUFFER_SZ           1440U
#define MAX_DMA_PKT_LEN             (MAX_UDP_BUFFER_SZ - DATA_OFFSET)

#define DATAWIDTH                   64U /* SET THIS BASED ON THE DATAWIDTH */

#define BYTES_PER_BEAT              (DATAWIDTH / 8U)  // 8 bytes (2x float32)
#define NUM_DATABEATS               (MAX_DMA_PKT_LEN / BYTES_PER_BEAT) // 178U                        // 178 beats = 1424 bytes
#define DMA_PKT_LEN                 (NUM_DATABEATS * BYTES_PER_BEAT) // 1424 bytes

/* Total UDP payload = 16-byte header + 1424-byte IQ payload = 1440 bytes */
#define UDP_SEND_BUFSIZE            (DATA_OFFSET + DMA_PKT_LEN)

#define SYNC_WORD                   0xDEADBEEFU

/* Set to 1 to byte-swap 32-bit words to Big-Endian, or 0 for fast native Little-Endian memcpy */
#define SWAP_PAYLOAD_ENDIANNESS     0 // 1

/* MAX UDP send retries */
#define MAX_SEND_RETRY              10U
#define NUM_OF_PARALLEL_CLIENTS     1U
#define ERROR_SLEEP                 100U

#ifdef __cplusplus
}
#endif

#endif /* __UDP_PERF_CLIENT_H_ */