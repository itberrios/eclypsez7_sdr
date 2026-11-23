/*
 * Copyright (C) 2017 - 2019 Xilinx, Inc.
 * All rights reserved.
 * SPDX-License-Identifier: MIT
 */

#include "udp_perf_client.h"

#include "xil_exception.h"
#include "xil_cache.h"
#include "xpseudo_asm.h"
#include "xscugic.h"
#include "xaxidma.h"
#include "xaxidma_bdring.h"
#include "FreeRTOS.h"
#include "task.h"
#include "semphr.h"

extern struct netif server_netif;
static XScuGic xInterruptController;

static struct perf_stats client;
static char send_buf[UDP_SEND_BUFSIZE] __attribute__((aligned(32)));
static struct sockaddr_in addr;
static int sock[NUM_OF_PARALLEL_CLIENTS];

#define FINISH                  1
#define REPORT_INTERVAL_TIME    (INTERIM_REPORT_INTERVAL * 1000)
#define END_TIME                (UDP_TIME_INTERVAL * 1000)

/* Convert relative Fabric SPI ID to Absolute GIC Vector ID (+32) */
#define AXI_DMA_GIC_VECTOR_ID   (XPAR_FABRIC_AXI_DMA_0_INTR + 32U)

/*
 * Scatter-Gather (SG) DMA Ring & Buffer Pool Configuration
 * ----------------------------------------------------------------------------
 * Why NUM_BDS is set to 512 (512 * 1,536 bytes = 768 KB total ring buffer):
 *
 * 1. Prevents Buffer Overflows During UART Prints & Network Bursts:
 *    At 828 Mbps (~72,000 packets/sec), 512 buffers provide ~7.1 ms of
 *    hardware storage in DDR3 RAM. This gives the FPGA plenty of room to keep
 *    writing incoming ADC samples without dropping data while the CPU is busy
 *    printing 5-second UART bandwidth reports (~4.8 ms) or handling lwIP bursts.
 *
 * 2. Enables "TURBO Mode" (Zero-Overhead Cache Refresh):
 *    The FPGA writes new ADC data directly into DDR3 RAM (via S_AXI_HP0),
 *    bypassing the CPU's internal caches (32 KB L1 and 512 KB L2). Normally,
 *    software must manually invalidate (clear) the cache before reading each
 *    packet so the CPU doesn't read old, cached data—but doing that 72,000
 *    times a second slows the CPU down by ~40 Mbps.
 *
 *    By sizing the buffer ring to 768 KB—which is larger than the entire
 *    512 KB L2 cache—simply cycling through all 512 buffers (plus normal
 *    lwIP and FreeRTOS memory activity) naturally pushes old packets out of
 *    the cache before the ring wraps around. When slot 0 is reused 512 packets
 *    later, the old cached copy is already gone, forcing the CPU to fetch the
 *    brand-new ADC samples straight from DDR3 RAM with zero software overhead!
 */
#define PKT_ALLOC_BYTES         1536U   // Buffer slot size (multiple of 32-byte Cortex-A9 cache line)
#define NUM_BDS                 512U    // 512 Buffer Descriptors (BDs) * 1536 B = 768 KB total ring
#define BD_ALIGNMENT            XAXIDMA_BD_MINIMUM_ALIGNMENT // 64-byte hardware alignment
#define DMA_COALESCE_COUNT      16U     // Fire 1 interrupt every 16 packets (~222 us)
#define DMA_COALESCE_TIMER      16U     // Fallback timeout if packet stream slows down

/* Peripherals & RTOS Handles */
static XAxiDma dma;
static XAxiDma_BdRing *RxRingPtr = NULL;
static SemaphoreHandle_t xDmaDoneSemaphore = NULL;
static volatile u32 dma_error_flag = 0;

/* BD Ring Space and Aligned Data Buffers in DDR3 memory */
static u8 bd_space[XAxiDma_BdRingMemCalc(BD_ALIGNMENT, NUM_BDS)] __attribute__((aligned(64)));
static u32_t rx_buffer_pool[NUM_BDS][PKT_ALLOC_BYTES / sizeof(u32_t)] __attribute__((aligned(32)));

/* ========================================================================= */
/* SDR Control Register Driver Functions (sdr_ctrl_regs)                     */
/* ========================================================================= */

/**
 * @brief Calculates the 32-bit Phase Increment (PINC) for the Xilinx DDS Compiler.
 *
 * Maps any target RF frequency (including undersampled signals in higher Nyquist
 * zones) into the 1st Nyquist zone (0 to Fs/2 = 61.44 MHz), accounting for
 * spectral inversion in even-numbered Nyquist zones. Then computes the 32-bit
 * unsigned phase step using:
 *     PINC = round((f_alias / Fs) * 2^32)
 *
 * @param  target_rf_freq_hz  Desired RF center frequency in Hz (e.g., 95500000.0).
 * @return uint32_t           32-bit PINC value formatted for S_AXIS_CONFIG.
 */
uint32_t calculate_dds_pinc(double target_rf_freq_hz)
{
    /* 1. Fold target frequency into the 1st Nyquist zone (0 to 61.44 MHz) */
    double nyquist_zone_width = SAMPLING_FREQ_HZ / 2.0;
    int zone = (int)(target_rf_freq_hz / nyquist_zone_width) + 1;

    double f_alias;
    if (zone % 2 == 0) {
        /* Even Nyquist zones (e.g., Zone 2: 61.44 - 122.88 MHz) fold inverted */
        f_alias = (zone * nyquist_zone_width) - target_rf_freq_hz;
    } else {
        /* Odd Nyquist zones fold non-inverted */
        f_alias = target_rf_freq_hz - ((zone - 1) * nyquist_zone_width);
    }

    /* 2. Compute 32-bit integer Phase Increment (PINC) */
    double pinc_double = (f_alias / SAMPLING_FREQ_HZ) * pow(2.0, ACCUMULATOR_BITS);
    uint32_t pinc = (uint32_t)round(pinc_double);

    /* Integer formatting for xil_printf (avoids float printf stack overhead) */
    u32 rf_mhz_int    = (u32)(target_rf_freq_hz / 1e6);
    u32 rf_mhz_frac   = (u32)(fmod(target_rf_freq_hz, 1e6) / 1e4);
    u32 alias_mhz_int = (u32)(f_alias / 1e6);
    u32 alias_mhz_frc = (u32)(fmod(f_alias, 1e6) / 1e4);

    xil_printf("[DDS Setup] Target RF: %u.%02u MHz | Zone: %d | Alias: %u.%02u MHz | PINC: 0x%08X\r\n",
               rf_mhz_int, rf_mhz_frac, zone, alias_mhz_int, alias_mhz_frc, pinc);

    return pinc;
}

/**
 * @brief Programs the NCO frequency for Channel 1 or Channel 2 via sdr_ctrl_regs.
 *
 * Writes the computed 32-bit PINC value to SDR_REG_DDS1_PINC (Offset 0x00) or
 * SDR_REG_DDS2_PINC (Offset 0x04). Inside sdr_ctrl_regs.v, this AXI4-Lite write
 * latches the PINC data and asserts m_axis_ddsX_tvalid for 1 clock cycle to
 * update the corresponding DDS Compiler IP over S_AXIS_CONFIG.
 *
 * @param  reg_offset      Register offset (SDR_REG_DDS1_PINC or SDR_REG_DDS2_PINC).
 * @param  target_freq_hz  Target RF tuning frequency in Hz.
 */
void set_nco_frequency(u32 reg_offset, double target_freq_hz)
{
    uint32_t pinc = calculate_dds_pinc(target_freq_hz);
    Xil_Out32(SDR_CTRL_BASEADDR + reg_offset, pinc);

    u32 ch_num  = (reg_offset == SDR_REG_DDS1_PINC) ? 1U : 2U;
    u32 mhz_int = (u32)(target_freq_hz / 1e6);
    u32 mhz_frc = (u32)(fmod(target_freq_hz, 1e6) / 1e4);

    xil_printf("[CH%u NCO] Reg 0x%02X <= PINC 0x%08X (%u.%02u MHz)\r\n",
               ch_num, reg_offset, pinc, mhz_int, mhz_frc);
}

/**
 * @brief Configures the shared CIC decimation factor across all 4 CIC filters.
 *
 * Clamps the requested decimation ratio to the valid hardware range [4, 2048]
 * and writes it to SDR_REG_CIC_RATE (Offset 0x08). In hardware, sdr_ctrl_regs
 * drives a 16-bit S_AXIS_CONFIG transaction into the 4-way AXI4-Stream
 * Broadcaster, updating ch1_i_cic, ch1_q_cic, ch2_i_cic, and ch2_q_cic in
 * lockstep.
 *
 * @param  decimation_rate  Integer CIC decimation factor R_CIC (4 to 2048).
 */
void set_cic_decimation(u16 decimation_rate)
{
    if (decimation_rate < 4U) {
        decimation_rate = 4U;
    } else if (decimation_rate > 2048U) {
        decimation_rate = 2048U;
    }

    Xil_Out32(SDR_CTRL_BASEADDR + SDR_REG_CIC_RATE, (u32)decimation_rate);

    /* Output sample rate = (122.88 MSPS / R_CIC) * (5 / 6) */
    u32 fs_out_khz = (u32)((122880.0 / (double)decimation_rate) * (5.0 / 6.0));
    xil_printf("[CIC Config] R_CIC = %u | Post-FIR Sample Rate = %u kSPS/ch\r\n",
               decimation_rate, fs_out_khz);
}

/**
 * @brief Enables or disables the SDR acquisition stream and hardware packetizer.
 *
 * Controls SDR_REG_PKT_CTRL (Offset 0x0C) in sdr_ctrl_regs:
 *   - Bits [15:0]: cfg_packet_len (always preserved as NUM_FRAMES_PER_PKT = 89).
 *   - Bit  [16]  : sw_enable (hardware-ANDed with dma_s2mm_rstn in Verilog to
 *                  drive pkt_enable to both ZmodSDRController_0 and
 *                  axis_interleaved_adc_packetizer).
 *
 * @param  enable  1U to enable acquisition and packet framing; 0U to disable.
 */
void sdr_stream_enable(u8_t enable)
{
    u32 reg_val = ((u32)NUM_FRAMES_PER_PKT & SDR_PKT_CTRL_LEN_MASK);
    if (enable) {
        reg_val |= SDR_PKT_CTRL_ENABLE_MASK;
    }
    Xil_Out32(SDR_CTRL_BASEADDR + SDR_REG_PKT_CTRL, reg_val);
}

/* ========================================================================= */
/* Scatter-Gather AXI DMA & Interrupt Setup                                  */
/* ========================================================================= */

/**
 * @brief Initializes the AXI DMA S2MM channel in Scatter-Gather (SG) mode.
 *
 * Creates a 64-descriptor BD ring in DDR3, attaches each descriptor to a
 * 32-byte cache-aligned receive buffer in rx_buffer_pool, sets each BD transfer
 * length to DMA_PKT_LEN (1424 bytes = 89 dual-channel frames = 178 beats),
 * invalidates the data cache over the buffer pool, and starts the S2MM ring.
 *
 * @param  AxiDmaInstPtr  Pointer to the XAxiDma driver instance.
 * @return int            XST_SUCCESS if successful, XST_FAILURE otherwise.
 */

 static int RxSetup(XAxiDma *AxiDmaInstPtr)
{
    int status;
    XAxiDma_Config *config;
    XAxiDma_Bd BdTemplate;
    XAxiDma_Bd *BdPtr = NULL;
    XAxiDma_Bd *BdCurPtr = NULL;

    config = XAxiDma_LookupConfig(DMA_BASEADDR);
    if (!config) {
        xil_printf("[SG DMA] LookupConfig failed for BaseAddr: 0x%08X\r\n", DMA_BASEADDR);
        return XST_FAILURE;
    }

    status = XAxiDma_CfgInitialize(AxiDmaInstPtr, config);
    if (status != XST_SUCCESS) {
        return XST_FAILURE;
    }

    if (!XAxiDma_HasSg(AxiDmaInstPtr)) {
        xil_printf("[SG DMA] Error: DMA is not configured for Scatter-Gather mode!\r\n");
        return XST_FAILURE;
    }

    /* Get pointer to rx ring */
    RxRingPtr = XAxiDma_GetRxRing(AxiDmaInstPtr);

    /* Disable RX interrupts during ring setup */
    XAxiDma_BdRingIntDisable(RxRingPtr, XAXIDMA_IRQ_ALL_MASK);

    /* Coalesce N BDs per interrupt to slash GIC/FreeRTOS context-switch overhead */
    XAxiDma_BdRingSetCoalesce(RxRingPtr, DMA_COALESCE_COUNT, DMA_COALESCE_TIMER);

    /* Create BD Ring in DDR */
    status = XAxiDma_BdRingCreate(RxRingPtr,
                                  (UINTPTR)bd_space,
                                  (UINTPTR)bd_space,
                                  BD_ALIGNMENT,
                                  NUM_BDS);
    if (status != XST_SUCCESS) {
        xil_printf("[SG DMA] BdRingCreate failed: %d\r\n", status);
        return XST_FAILURE;
    }

    /* clone BdTemplate into the every BD in the ring so we start fresh with everything zerod out */
    XAxiDma_BdClear(&BdTemplate);
    status = XAxiDma_BdRingClone(RxRingPtr, &BdTemplate);
    if (status != XST_SUCCESS) {
        return XST_FAILURE;
    }

    /* Allocate all NUM_BDS descriptors, BdPtr points to the first BD */
    status = XAxiDma_BdRingAlloc(RxRingPtr, NUM_BDS, &BdPtr);
    if (status != XST_SUCCESS) {
        return XST_FAILURE;
    }

    BdCurPtr = BdPtr;
    for (u32 i = 0; i < NUM_BDS; i++) {
        /*
         * CACHE-LINE ALIGNED ZERO-COPY OFFSET:
         * Start AXI DMA S2MM at word [8] (+32 bytes = exact start of Cache Line 1).
         * Words [4..7] (bytes 16..31 in Cache Line 0) hold the 16-byte UDP header!
         */
        UINTPTR payload_dma_addr = (UINTPTR)&rx_buffer_pool[i][8];

        // set size of each buffer in DDR3 RAM
        status = XAxiDma_BdSetBufAddr(BdCurPtr, payload_dma_addr);
        if (status != XST_SUCCESS) return XST_FAILURE;

        status = XAxiDma_BdSetLength(BdCurPtr, DMA_PKT_LEN, RxRingPtr->MaxTransferLen);
        if (status != XST_SUCCESS) return XST_FAILURE;

        XAxiDma_BdSetCtrl(BdCurPtr, 0);
        XAxiDma_BdSetId(BdCurPtr, i);

        Xil_DCacheFlushRange((UINTPTR)rx_buffer_pool[i], PKT_ALLOC_BYTES);

        BdCurPtr = (XAxiDma_Bd *)XAxiDma_BdRingNext(RxRingPtr, BdCurPtr);
    }
    
    /* push the configured BDs to hardware, once thid fuction completes
     * the BDs will be controlled by hardware. And the h/w will be able to
     * write transfers directly to the configured memory addresses
     */
    status = XAxiDma_BdRingToHw(RxRingPtr, NUM_BDS, BdPtr);
    if (status != XST_SUCCESS) {
        return XST_FAILURE;
    }

    /* Set the interupts for the BDs */
    XAxiDma_BdRingIntEnable(RxRingPtr, XAXIDMA_IRQ_IOC_MASK | XAXIDMA_IRQ_ERROR_MASK);

    /* allows DMA transactions to commence, DMA won't enable until this completes */
    status = XAxiDma_BdRingStart(RxRingPtr);
    if (status != XST_SUCCESS) {
        xil_printf("[SG DMA] BdRingStart failed!\r\n");
        return XST_FAILURE;
    }

    dma_error_flag = 0;
    xil_printf("[SG DMA] %u-Buffer Zero-Copy SG Ring started!\r\n", NUM_BDS);
    return XST_SUCCESS;
}

/**
 * @brief FreeRTOS-safe Interrupt Service Routine for AXI DMA S2MM Scatter-Gather.
 *
 * Acknowledges hardware interrupts on the S2MM BD ring, records a 64-bit ARM
 * Global Timer timestamp for each completed packet, and wakes the blocked
 * transfer_data() task via xDmaDoneSemaphore.
 *
 * @param  Callback  Pointer to the S2MM XAxiDma_BdRing instance.
 */
static void RxSgIntrHandler(void *Callback)
{
    XAxiDma_BdRing *ring_ptr = (XAxiDma_BdRing *)Callback;
    BaseType_t xHigherPriorityTaskWoken = pdFALSE;

	/* Read and acknowledge pending interrupts */
    u32 IrqStatus = XAxiDma_BdRingGetIrq(ring_ptr);
    XAxiDma_BdRingAckIrq(ring_ptr, IrqStatus);

    if (!(IrqStatus & XAXIDMA_IRQ_ALL_MASK)) {
        return;
    }

    if (IrqStatus & XAXIDMA_IRQ_ERROR_MASK) {
        dma_error_flag = 1;
    }

    xSemaphoreGiveFromISR(xDmaDoneSemaphore, &xHigherPriorityTaskWoken);
    portYIELD_FROM_ISR(xHigherPriorityTaskWoken);
}


/**
 * @brief Registers the AXI DMA S2MM interrupt with the Zynq GIC under FreeRTOS.
 *
 * Creates the Binary semaphore used to signal completed BDs and attaches
 * RxSgIntrHandler to AXI_DMA_GIC_VECTOR_ID at priority 0xA0 (safe for
 * FreeRTOS FromISR API calls).
 *
 * @return int  XST_SUCCESS on success, XST_FAILURE otherwise.
 */
int SetupFreeRTOSDmaInterrupts(void)
{
    int Status;

    if (xDmaDoneSemaphore == NULL) {
        xDmaDoneSemaphore = xSemaphoreCreateBinary();
        if (xDmaDoneSemaphore == NULL) {
            return XST_FAILURE;
        }
    }

    XScuGic_Config *IntcConfig = XScuGic_LookupConfig(XPAR_SCUGIC_SINGLE_DEVICE_ID);
    if (IntcConfig == NULL) {
        return XST_FAILURE;
    }

    xInterruptController.Config = IntcConfig;
    xInterruptController.IsReady = XIL_COMPONENT_IS_READY;

    /*
     * Priority: 0xA0 (must be numerically >= configMAX_API_CALL_INTERRUPT_PRIORITY)
     * Trigger:  0x1  (Active-High Level-Sensitive for AXI DMA s2mm_introut)
     */
    XScuGic_SetPriorityTriggerType(&xInterruptController, AXI_DMA_GIC_VECTOR_ID, 0xA0, 0x1);

    Status = XScuGic_Connect(&xInterruptController,
                             AXI_DMA_GIC_VECTOR_ID,
                             (Xil_InterruptHandler)RxSgIntrHandler,
                             (void *)RxRingPtr);
    if (Status != XST_SUCCESS) {
        return Status;
    }

    XScuGic_Enable(&xInterruptController, AXI_DMA_GIC_VECTOR_ID);

    xil_printf("[INTR SUCCESS] SG DMA Vector %u attached cleanly!\r\n", AXI_DMA_GIC_VECTOR_ID);
    return XST_SUCCESS;
}

/* ========================================================================= */
/* Telemetry Reporting & UDP Socket Transmission                             */
/* ========================================================================= */

void print_app_header(void)
{
    xil_printf("UDP client connecting to %s on port %d\r\n",
               UDP_SERVER_IP_ADDRESS, UDP_CONN_PORT);
    xil_printf("On Host: Run $iperf -s -i %d -u\r\n\r\n",
               INTERIM_REPORT_INTERVAL);
}

static void print_udp_conn_stats(void)
{
    xil_printf("[%3d] local %s port %d connected with ",
               client.client_id, inet_ntoa(server_netif.ip_addr),
               UDP_CONN_PORT);
    xil_printf("%s port %d\r\n", UDP_SERVER_IP_ADDRESS, UDP_CONN_PORT);
    xil_printf("[ ID] Interval\t\tTransfer   Bandwidth\n\r");
}

static void stats_buffer(char *outString, double data, enum measure_t type)
{
    int conv = KCONV_UNIT;
    const char *format;
    double unit = (type == SPEED) ? 1000.0 : 1024.0;

    while (data >= unit && conv <= KCONV_GIGA) {
        data /= unit;
        conv++;
    }

    if (data < 9.995) {
        format = "%4.2f %c";
    } else if (data < 99.95) {
        format = "%4.1f %c";
    } else {
        format = "%4.0f %c";
    }
    sprintf(outString, format, data, kLabel[conv]);
}

static void udp_conn_report(u64_t diff, enum report_type report_type)
{
    u64_t total_len;
    double duration, bandwidth = 0;
    char data[16], perf[16], time[64];

    if (report_type == INTER_REPORT) {
        total_len = client.i_report.total_bytes;
    } else {
        client.i_report.last_report_time = 0;
        total_len = client.total_bytes;
    }

    duration = diff / 1000.0;
    if (duration) {
        bandwidth = (total_len / duration) * 8.0;
    }

    stats_buffer(data, total_len, BYTES);
    stats_buffer(perf, bandwidth, SPEED);

    sprintf(time, "%4.1f-%4.1f sec",
            (double)client.i_report.last_report_time,
            (double)(client.i_report.last_report_time + duration));
    xil_printf("[%3d] %s  %sBytes  %sbits/sec\n\r",
               client.client_id, time, data, perf);

    if (report_type == INTER_REPORT) {
        client.i_report.last_report_time += duration;
    } else {
        xil_printf("[%3d] sent %llu datagrams\n\r",
                   client.client_id, client.cnt_datagrams);
    }
}

static void reset_stats(void)
{
    client.client_id++;
    print_udp_conn_stats();

    client.start_time = sys_now();
    client.total_bytes = 0;
    client.cnt_datagrams = 0;

    client.i_report.start_time = 0;
    client.i_report.total_bytes = 0;
    client.i_report.last_report_time = 0;
}


static int udp_packet_send_buf(u32_t *pkt_buf, u8_t finished)
{
    static int packet_id = 0;
    int count;
    socklen_t len = sizeof(addr);

    if (finished == FINISH) {
        packet_id = -1;
    }
    pkt_buf[0] = htonl((u32_t)packet_id);
    packet_id++;

    for (u32 i = 0; i < NUM_OF_PARALLEL_CLIENTS; i++) {
        u32 retries = MAX_SEND_RETRY;
        while (retries) {
#if LWIP_UDP_OPT_BLOCK_TX_TILL_COMPLETE
            count = lwip_sendto_blocking(sock[i], (char *)pkt_buf, UDP_SEND_BUFSIZE, 0,
                                         (struct sockaddr *)&addr, len);
#else
            count = lwip_sendto(sock[i], (char *)pkt_buf, UDP_SEND_BUFSIZE, 0,
                                (struct sockaddr *)&addr, len);
#endif
            if (count <= 0) {
                retries--;
                usleep(ERROR_SLEEP);
            } else {
                client.total_bytes += count;
                client.cnt_datagrams++;
                client.i_report.total_bytes += count;
                break;
            }
        }

        if (!retries) {
            u64_t now = sys_now();
            u64_t diff_ms = now - client.start_time;
            xil_printf("Too many udp_send() retries, Terminating application\n\r");
            udp_conn_report(diff_ms, UDP_DONE_CLIENT);
            xil_printf("UDP test failed\n\r");
            return -1;
        }
    }
    return 0;
}

/**
 * @brief Main real-time DMA harvesting and UDP packet transmission loop body.
 *
 * Waits on xDmaDoneSemaphore for completed S2MM Buffer Descriptors, invalidates
 * D-Cache over each received 1424-byte interleaved float IQ payload, prepends
 * the 16-byte packet header (Packet ID, 0xDEADBEEF sync word, 64-bit timestamp),
 * transmits the 1440-byte UDP datagram, and recycles the BDs back to hardware.
 *
 * @return int  0 to continue streaming, FINISH (1) when test duration completes.
 */
 int transfer_data(void)
{
    if (END_TIME || REPORT_INTERVAL_TIME) {
        u64_t now = sys_now();
        if (REPORT_INTERVAL_TIME) {
            if (client.i_report.start_time) {
                u64_t diff_ms = now - client.i_report.start_time;
                if (diff_ms >= REPORT_INTERVAL_TIME) {
                    udp_conn_report(diff_ms, INTER_REPORT);
                    client.i_report.start_time = 0;
                    client.i_report.total_bytes = 0;
                }
            } else {
                client.i_report.start_time = now;
            }
        }

        if (END_TIME) {
            u64_t diff_ms = now - client.start_time;
            if (diff_ms >= END_TIME) {
                sdr_stream_enable(0U);
                udp_packet_send_buf((u32_t *)send_buf, FINISH);
                udp_conn_report(diff_ms, UDP_DONE_CLIENT);
                xil_printf("UDP test passed Successfully\n\r");
                return FINISH;
            }
        }
    }

    static int pl_enabled = 0;
    if (!pl_enabled) {
        sdr_stream_enable(1U);
        pl_enabled = 1;
    }

    /* wait for the semaphore to signal DMA transfer complete */
    if (xSemaphoreTake(xDmaDoneSemaphore, pdMS_TO_TICKS(1000)) == pdTRUE) {

        if (dma_error_flag) {
            xil_printf("[SG DMA ERROR] Hardware error interrupt detected! Re-initializing DMA...\r\n");
            sdr_stream_enable(0U);
            XAxiDma_Reset(&dma);
            while (!XAxiDma_ResetIsDone(&dma));
            RxSetup(&dma);
            sdr_stream_enable(1U);
            return 0;
        }

        /* retrtieve all processed BDs from hardware */
        XAxiDma_Bd *bd_first_ptr = NULL;
        int processed_bds = XAxiDma_BdRingFromHw(RxRingPtr, XAXIDMA_ALL_BDS, &bd_first_ptr);
        if (processed_bds <= 0) {
            return 0;
        }

        XTime t_batch;
        XTime_GetTime(&t_batch);
        u32_t ts_hi = htonl((u32_t)(t_batch >> 32));
        u32_t ts_lo = htonl((u32_t)(t_batch & 0xFFFFFFFFU));
        u32_t sync  = htonl(SYNC_WORD);
		
        XAxiDma_Bd *bd_cur = bd_first_ptr;

        for (int b = 0; b < processed_bds; b++) {
            u32 buf_idx = (u32)XAxiDma_BdGetId(bd_cur);
            u32_t *active_buf = rx_buffer_pool[buf_idx];

            /*
             * Invalidate starting at Cache Line 1 (&active_buf[8], 32-byte aligned).
             * When ENABLE_TURBO == 1 and NUM_BDS == 512 (768 KB > 512 KB L2),
             * both L1 and L2 self-evict naturally before ring wraparound!
             */
            #if !ENABLE_TURBO
                Xil_DCacheInvalidateRange((UINTPTR)&active_buf[8], DMA_PKT_LEN);
            #endif
            /*
             * Write 16-byte UDP header at words [4..7] (bytes 16..31),
             * immediately preceding the DMA payload at word [8] (byte 32).
             */
            u32_t *udp_pkt_ptr = &active_buf[4];
            udp_pkt_ptr[1] = sync;
            udp_pkt_ptr[2] = ts_hi;
            udp_pkt_ptr[3] = ts_lo;

            /* Clear BD status register right now while bd_cur is already in a CPU register
             * (eliminates the second pointer-chasing loop during BdRingAlloc below!) */
            XAxiDma_BdWrite(bd_cur, XAXIDMA_BD_STS_OFFSET, 0);

            /* Transmit 1440 bytes starting at &active_buf[4] */
            if (udp_packet_send_buf(udp_pkt_ptr, !FINISH) < 0) {
                return FINISH;
            }

            bd_cur = (XAxiDma_Bd *)XAxiDma_BdRingNext(RxRingPtr, bd_cur);
        }

        /*
         * CRITICAL: Free the processed BDs and immediately re-submit them
         * to hardware. Because we already cleared XAXIDMA_BD_STS_OFFSET in
         * the loop above, we skip the second pointer-traversal loop here!
         */
        int status = XAxiDma_BdRingFree(RxRingPtr, processed_bds, bd_first_ptr);
        if (status == XST_SUCCESS) {
            XAxiDma_Bd *bd_realloc_ptr = NULL;
            status = XAxiDma_BdRingAlloc(RxRingPtr, processed_bds, &bd_realloc_ptr);
            if (status == XST_SUCCESS) {
                XAxiDma_BdRingToHw(RxRingPtr, processed_bds, bd_realloc_ptr);
            }
        }

    } else {
        xil_printf("[DMA TIMEOUT] Re-initializing SG DMA core...\r\n");
        sdr_stream_enable(0U);
        XAxiDma_Reset(&dma);
        while (!XAxiDma_ResetIsDone(&dma));
        RxSetup(&dma);
        sdr_stream_enable(1U);
    }

    return 0;
}

/**
 * @brief Application entry point called by the main FreeRTOS network thread.
 *
 * Sequences hardware initialization to prevent partial packets:
 *   1. Holds SDR stream disabled while programming NUM_FRAMES_PER_PKT (89).
 *   2. Programs Channel 1 NCO, Channel 2 NCO, and shared CIC decimation rate.
 *   3. Initializes the 64-descriptor AXI DMA S2MM Scatter-Gather ring and GIC ISR.
 *   4. Opens the UDP socket and enters the real-time transfer_data() loop.
 */
void start_application(void)
{
    err_t err;

    xil_printf("Application program init (sdr_ctrl_regs @ 0x%08X)\r\n", SDR_CTRL_BASEADDR);

    /* 1. Hold SDR stream disabled while setting packet frame count (89 frames) */
    sdr_stream_enable(0U);

    /* 2. Program NCO tuning frequencies and CIC decimation rate */
    set_nco_frequency(SDR_REG_DDS1_PINC, NCO_CH1_FREQ);
    set_nco_frequency(SDR_REG_DDS2_PINC, NCO_CH2_FREQ);
    set_cic_decimation(SDR_DEFAULT_CIC_RATE);

    /* 3. Initialize AXI DMA S2MM Scatter-Gather Ring & Interrupts */
    if (RxSetup(&dma) != XST_SUCCESS) {
        xil_printf("AXI DMA Failed to initialize\r\n");
        return;
    }

    if (SetupFreeRTOSDmaInterrupts() != XST_SUCCESS) {
        xil_printf("DMA Interrupts Failed to configure\r\n");
        return;
    }

    xil_printf("SDR Controller, SG DMA, and Interrupts successfully configured!\r\n");

    memset(&addr, 0, sizeof(struct sockaddr_in));
    addr.sin_family = AF_INET;
    addr.sin_port = htons(UDP_CONN_PORT);
    addr.sin_addr.s_addr = inet_addr(UDP_SERVER_IP_ADDRESS);

    for (u32 i = 0; i < NUM_OF_PARALLEL_CLIENTS; i++) {
        if ((sock[i] = socket(AF_INET, SOCK_DGRAM, 0)) < 0) {
            xil_printf("UDP client: Error creating Socket\r\n");
            return;
        }

        err = connect(sock[i], (struct sockaddr *)&addr, sizeof(addr));
        if (err != ERR_OK) {
            xil_printf("UDP client: Error on connect: %d\r\n", err);
            close(sock[i]);
            return;
        }
    }

    usleep(10);
    reset_stats();

    while (!transfer_data());
}