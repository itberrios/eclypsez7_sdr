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
#include <stdint.h>
#include <math.h>
#include "xil_printf.h"
#include "xil_io.h"
#include "xlwipconfig.h"
#include "xiltimer.h"
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
#define UDP_TIME_INTERVAL           60

/* Server to connect with */
#define UDP_SERVER_IP_ADDRESS       "192.168.1.100"

/* ========================================================================= */
/* Hardware Base Addresses & sdr_ctrl_regs Register Map                      */
/* ========================================================================= */
#define DMA_BASEADDR                XPAR_XAXIDMA_0_BASEADDR

#if defined(XPAR_SDR_CONTROLLER_BASEADDR)
    #define SDR_CTRL_BASEADDR       XPAR_SDR_CONTROLLER_BASEADDR
#elif defined(XPAR_SDR_CTRL_REGS_0_BASEADDR)
    #define SDR_CTRL_BASEADDR       XPAR_SDR_CTRL_REGS_0_BASEADDR
#else
    #define SDR_CTRL_BASEADDR       0x40000000U
#endif

/* Register Offsets inside sdr_ctrl_regs (AXI4-Lite Slave) */
#define SDR_REG_DDS1_PINC           0x00U  /* [31:0]  Ch1 NCO Phase Increment        */
#define SDR_REG_DDS2_PINC           0x04U  /* [31:0]  Ch2 NCO Phase Increment        */
#define SDR_REG_CIC_RATE            0x08U  /* [15:0]  CIC Decimation Rate (4..2048)  */
#define SDR_REG_PKT_CTRL            0x0CU  /* [16]    sw_enable, [15:0] pkt_len      */

#define SDR_PKT_CTRL_ENABLE_MASK    (1U << 16)
#define SDR_PKT_CTRL_LEN_MASK       0x0000FFFFU

/* ========================================================================= */
/* NCO / DDS & CIC Decimation Default Configuration                          */
/* ========================================================================= */
#define SAMPLING_FREQ_HZ            122880000.0  /* 122.88 MSPS ADC Master Clock     */
#define ACCUMULATOR_BITS            32           /* 32-bit DDS Phase Accumulator     */
#define NCO_CH1_FREQ                90000000.0 // 29780000.0   /* Default Ch1 NCO: 29.78 MHz       */
#define NCO_CH2_FREQ                100000000.0 // 95500000.0   /* Default Ch2 NCO: 95.50 MHz       */

/*
 * Default CIC Decimation Factor (valid hardware range: 4 to 2048).
 * Total Decimation = SDR_DEFAULT_CIC_RATE * (6 / 5).
 * Example: R_CIC = 32 -> FIR Output Rate = 122.88 MSPS / 32 * (5/6) = 3.20 MSPS/ch
 */
#define SDR_DEFAULT_CIC_RATE       16U

/* ========================================================================= */
/* Packet Header Layout (16 bytes total)                                     */
/* ========================================================================= */
#define PACKET_ID_SZ                4U
#define SYNC_WORD_SZ                4U
#define TIMESTAMP_SZ                8U

#define PACKET_ID_OFFSET            0U
#define SYNC_WORD_OFFSET            (PACKET_ID_OFFSET + PACKET_ID_SZ)   /* 4  */
#define TIMESTAMP_OFFSET            (SYNC_WORD_OFFSET + SYNC_WORD_SZ)   /* 8  */
#define DATA_OFFSET                 (TIMESTAMP_OFFSET + TIMESTAMP_SZ)   /* 16 */

/* ========================================================================= */
/* Data Burst, Beats, and Multi-Channel Frame Definitions                    */
/* ========================================================================= */
#define MAX_UDP_BUFFER_SZ           1440U
#define MAX_DMA_PKT_LEN             (MAX_UDP_BUFFER_SZ - DATA_OFFSET)   /* 1424 bytes */

#define DATAWIDTH                   64U   /* 64-bit AXI4-Stream bus ([Q_f32, I_f32]) */
#define NUM_SDR_CHANNELS            2U    /* 2 Interleaved Channels (Ch1, Ch2)       */

#define BYTES_PER_BEAT              (DATAWIDTH / 8U)                    /* 8 bytes/beat  */
#define BYTES_PER_FRAME             (BYTES_PER_BEAT * NUM_SDR_CHANNELS) /* 16 bytes/frame */

/* NOTE: a BEAT is defined as a single burst of data on the DMA
 *       a FRAME is defined as a complete pair of ch1 and ch2 I/Q data, since ch1 and 2 are interleaved
 *
 * NUM_FRAMES_PER_PKT: Number of complete dual-channel sample frames per packet.
 *   1424 bytes / 16 bytes_per_frame = 89 dual-channel frames.
 *   Programmed into sdr_ctrl_regs Offset 0x0C (cfg_packet_len).
 *
 * NUM_DATABEATS: Total 64-bit AXI4-Stream beats per packet.
 *   89 frames * 2 beats/frame = 178 beats.
 *
 * DMA_PKT_LEN: Total byte length per AXI DMA S2MM Buffer Descriptor.
 *   178 beats * 8 bytes/beat = 1424 bytes.
 */
#define NUM_FRAMES_PER_PKT          (MAX_DMA_PKT_LEN / BYTES_PER_FRAME) /* 89 frames  */
#define NUM_DATABEATS               (NUM_FRAMES_PER_PKT * NUM_SDR_CHANNELS) /* 178 beats */
#define DMA_PKT_LEN                 (NUM_DATABEATS * BYTES_PER_BEAT)    /* 1424 bytes */

/* Total UDP payload = 16-byte header + 1424-byte IQ payload = 1440 bytes */
#define UDP_SEND_BUFSIZE            (DATA_OFFSET + DMA_PKT_LEN)

#define SYNC_WORD                   0xDEADBEEFU

/* Set to 1 to byte-swap 32-bit words to Big-Endian, or 0 for fast native Little-Endian memcpy */
#define SWAP_PAYLOAD_ENDIANNESS     0 // Keep this at zero for real time operation

/* MAX UDP send retries */
#define MAX_SEND_RETRY              200U
#define NUM_OF_PARALLEL_CLIENTS     1U
#define ERROR_SLEEP                 2U 

/* Enable TURBO mode
 * Enabling this option speeds up the UDP Bandwidth by ~50 Mbps by
 * Not invalidating the cache before reading from the rx buffer pool. 
 * This can result in stale data from the previous buffer rings
 * instead of the new data from the DMA.
 * Since we are using a buffer ring, the stale data naturally gets moved
 * out of the buffers as we cycle through accodring to gemini:
 * "background lwIP network activity and FreeRTOS task switches 
 *  eventually evict and overwrite portions of the L2 cache."
 * This means that as the CPU 'does stuff' it uses the same addresses
 * as some of our rx_buffer_pool slots. When it 'does stuff' it inherently
 * invalidates the range!
 */

 /*
 * Enable TURBO Mode (Skip Software L1/L2 D-Cache Invalidation)
 * ----------------------------------------------------------------------------
 * Setting ENABLE_TURBO to 1 bypasses Xil_DCacheInvalidateRange() prior to
 * reading rx_buffer_pool in transfer_data(), recovering ~38-50 Mbps of CPU
 * throughput by eliminating 3.23 million PL310 L2 MMIO register writes/sec
 * at R_CIC = 16 (828.4 Mbps line rate).
 *
 * Why it mostly works with NUM_BDS = 256:
 *   - The AXI DMA writes new packets directly to physical DDR3 via S_AXI_HP0
 *     without notifying the CPU caches.
 *   - Because rx_buffer_pool is 384 KB (256 * 1536 B), it is 12x larger than
 *     the 32 KB L1 D-Cache, guaranteeing 100% natural L1 eviction before the
 *     256-BD ring wraps around.
 *   - Combined with lwIP's pbuf pool (~384 KB) and FreeRTOS stacks, the total
 *     working set (~800 KB) exceeds the 512 KB L2 cache, causing lwIP pbuf
 *     allocations to naturally evict most rx_buffer_pool lines from L2.
 *
 * Known Failure Modes when ENABLE_TURBO == 1 (on S_AXI_HP0):
 *   1. Partial L2 Stale Lines (Audio Distortion): Because L2 cache-set
 *      eviction is non-uniform, any 32-byte cache line in rx_buffer_pool not
 *      evicted during a 3.55 ms ring cycle will return stale IQ samples from
 *      256 packets prior, causing periodic phase jumps / slight audio distortion.
 *   2. Ring Size Dependency: If NUM_BDS is reduced (e.g., <= 128 BDs), the
 *      working set fits entirely inside the 512 KB L2 cache, halting natural
 *      L2 eviction and causing repeated stale packets.
 */
#define ENABLE_TURBO                0


/* Function Prototypes for SDR Hardware Control */
uint32_t calculate_dds_pinc(double target_rf_freq_hz);
void set_nco_frequency(u32 reg_offset, double target_freq_hz);
void set_cic_decimation(u16 decimation_rate);
void sdr_stream_enable(u8_t enable);

#ifdef __cplusplus
}
#endif

#endif /* __UDP_PERF_CLIENT_H_ */