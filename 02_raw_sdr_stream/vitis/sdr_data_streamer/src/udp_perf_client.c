/*
 * Copyright (C) 2017 - 2019 Xilinx, Inc.
 * All rights reserved.
 * SPDX-License-Identifier: MIT
 */

#include "udp_perf_client.h"

#include "xil_exception.h"
#include "xil_cache.h"
#include "xscugic.h"
#include "xaxidma.h"
#include "xaxidma_bdring.h"
#include "FreeRTOS.h"
#include "task.h"
#include "semphr.h"

extern struct netif server_netif;
/* Use the global FreeRTOS GIC instance initialized by the kernel port */
// extern XScuGic xInterruptController;
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
 * Use 64 BDs (64 * 1536 = 96 KB ring) so lwIP bursts never cause
 * DMA ring starvation or overwrite in-flight buffers.
 */
#define PKT_ALLOC_BYTES         1536U   // Multiple of 32-byte Cortex-A9 cache line
#define NUM_BDS                 64U
#define BD_ALIGNMENT            XAXIDMA_BD_MINIMUM_ALIGNMENT // 64-byte alignment

/* Peripherals & RTOS Handles */
static XGpio cfg_gpio;
static XAxiDma dma;
static XAxiDma_BdRing *RxRingPtr = NULL;
static SemaphoreHandle_t xDmaDoneSemaphore = NULL;
static volatile u32 dma_error_flag = 0;

/* BD Ring Space and Aligned Data Buffers in DDR */
static u8 bd_space[XAxiDma_BdRingMemCalc(BD_ALIGNMENT, NUM_BDS)] __attribute__((aligned(64)));
static u32_t rx_buffer_pool[NUM_BDS][PKT_ALLOC_BYTES / sizeof(u32_t)] __attribute__((aligned(32)));

/* Timestamp per BD slot (indexed by BD ID 0 .. NUM_BDS-1) */
static volatile XTime rx_timestamps[NUM_BDS];
static volatile u32 isr_bd_write_idx = 0;

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

    RxRingPtr = XAxiDma_GetRxRing(AxiDmaInstPtr);

    /* Disable RX interrupts during ring setup */
    XAxiDma_BdRingIntDisable(RxRingPtr, XAXIDMA_IRQ_ALL_MASK);

    /* Set interrupt coalescing: fire every 1 BD with 0 delay timer */
    XAxiDma_BdRingSetCoalesce(RxRingPtr, 1, 0);

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

    XAxiDma_BdClear(&BdTemplate);
    status = XAxiDma_BdRingClone(RxRingPtr, &BdTemplate);
    if (status != XST_SUCCESS) {
        return XST_FAILURE;
    }

    /* Allocate all NUM_BDS descriptors */
    status = XAxiDma_BdRingAlloc(RxRingPtr, NUM_BDS, &BdPtr);
    if (status != XST_SUCCESS) {
        return XST_FAILURE;
    }

    BdCurPtr = BdPtr;
    for (u32 i = 0; i < NUM_BDS; i++) {
        UINTPTR buf_addr = (UINTPTR)rx_buffer_pool[i];

        status = XAxiDma_BdSetBufAddr(BdCurPtr, buf_addr);
        if (status != XST_SUCCESS) return XST_FAILURE;

        status = XAxiDma_BdSetLength(BdCurPtr, DMA_PKT_LEN, RxRingPtr->MaxTransferLen);
        if (status != XST_SUCCESS) return XST_FAILURE;

        XAxiDma_BdSetCtrl(BdCurPtr, 0);
        /* Store the buffer slot index (0 .. NUM_BDS-1) as the BD ID */
        XAxiDma_BdSetId(BdCurPtr, i);

        /* Invalidate cache line range prior to DMA S2MM write */
        Xil_DCacheInvalidateRange(buf_addr, PKT_ALLOC_BYTES);

        BdCurPtr = (XAxiDma_Bd *)XAxiDma_BdRingNext(RxRingPtr, BdCurPtr);
    }

    /* Enqueue all BDs to hardware */
    status = XAxiDma_BdRingToHw(RxRingPtr, NUM_BDS, BdPtr);
    if (status != XST_SUCCESS) {
        return XST_FAILURE;
    }

    /* Enable Completion (IOC) and Error interrupts */
    XAxiDma_BdRingIntEnable(RxRingPtr, XAXIDMA_IRQ_IOC_MASK | XAXIDMA_IRQ_ERROR_MASK);

    /* Start S2MM SG DMA Channel */
    status = XAxiDma_BdRingStart(RxRingPtr);
    if (status != XST_SUCCESS) {
        xil_printf("[SG DMA] BdRingStart failed!\r\n");
        return XST_FAILURE;
    }

    isr_bd_write_idx = 0;
    dma_error_flag = 0;
    xil_printf("[SG DMA] %u-Buffer SG Ring started successfully!\r\n", NUM_BDS);
    return XST_SUCCESS;
}

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
        xSemaphoreGiveFromISR(xDmaDoneSemaphore, &xHigherPriorityTaskWoken);
        portYIELD_FROM_ISR(xHigherPriorityTaskWoken);
        return;
    }

    if (IrqStatus & XAXIDMA_IRQ_IOC_MASK) {
        XTime t_now;
        XTime_GetTime(&t_now);

        u32 idx = isr_bd_write_idx;
        rx_timestamps[idx] = t_now;
        isr_bd_write_idx = (idx + 1U) % NUM_BDS;

        xSemaphoreGiveFromISR(xDmaDoneSemaphore, &xHigherPriorityTaskWoken);
        portYIELD_FROM_ISR(xHigherPriorityTaskWoken);
    }
}


int SetupFreeRTOSDmaInterrupts(void)
{
    int Status;

    if (xDmaDoneSemaphore == NULL) {
        xDmaDoneSemaphore = xSemaphoreCreateCounting(NUM_BDS, 0);
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

static int udp_packet_send(u8_t finished)
{
    static int packet_id = 0;
    int count;
    socklen_t len = sizeof(addr);

    u32_t *header_words = (u32_t *)send_buf;
    if (finished == FINISH) {
        packet_id = -1;
    }
    header_words[0] = htonl((u32_t)packet_id);
    packet_id++;

    for (u32 i = 0; i < NUM_OF_PARALLEL_CLIENTS; i++) {
        u8_t retries = MAX_SEND_RETRY;
        while (retries) {
#if LWIP_UDP_OPT_BLOCK_TX_TILL_COMPLETE
            count = lwip_sendto_blocking(sock[i], send_buf, sizeof(send_buf), 0,
                                         (struct sockaddr *)&addr, len);
#else
            count = lwip_sendto(sock[i], send_buf, sizeof(send_buf), 0,
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
                XGpio_DiscreteWrite(&cfg_gpio, CFG_GPIO_SDR_ENABLE, 0x0);
                udp_packet_send(FINISH);
                udp_conn_report(diff_ms, UDP_DONE_CLIENT);
                xil_printf("UDP test passed Successfully\n\r");
                return FINISH;
            }
        }
    }

    /* Enable PL SDR stream on first invocation */
    static int pl_enabled = 0;
    if (!pl_enabled) {
        XGpio_DiscreteWrite(&cfg_gpio, CFG_GPIO_SDR_ENABLE, 0x1);
        pl_enabled = 1;
    }

    /* Sleep until ISR signals at least one completed BD */
    if (xSemaphoreTake(xDmaDoneSemaphore, pdMS_TO_TICKS(1000)) == pdTRUE) {

        if (dma_error_flag) {
            xil_printf("[SG DMA ERROR] Hardware error interrupt detected! Re-initializing DMA...\r\n");
            XGpio_DiscreteWrite(&cfg_gpio, CFG_GPIO_SDR_ENABLE, 0x0);
            XAxiDma_Reset(&dma);
            while (!XAxiDma_ResetIsDone(&dma));
            RxSetup(&dma);
            XGpio_DiscreteWrite(&cfg_gpio, CFG_GPIO_SDR_ENABLE, 0x1);
            return 0;
        }

        XAxiDma_Bd *bd_first_ptr = NULL;
        int processed_bds = XAxiDma_BdRingFromHw(RxRingPtr, XAXIDMA_ALL_BDS, &bd_first_ptr);
        if (processed_bds <= 0) {
            return 0;
        }

        /* Drain extra semaphore counts if coalesced/multiple BDs were harvested at once */
        for (int extra = 1; extra < processed_bds; extra++) {
            xSemaphoreTake(xDmaDoneSemaphore, 0);
        }

        XAxiDma_Bd *bd_cur = bd_first_ptr;
        u32_t *send_words = (u32_t *)send_buf;

        for (int b = 0; b < processed_bds; b++) {
            u32 buf_idx = (u32)XAxiDma_BdGetId(bd_cur);
            u32_t *active_buf = rx_buffer_pool[buf_idx];

            /* Invalidate D-Cache for this specific buffer so CPU reads fresh DDR data */
            Xil_DCacheInvalidateRange((UINTPTR)active_buf, PKT_ALLOC_BYTES);

            /* Fetch timestamp associated with this BD slot */
            XTime t_dma_done = rx_timestamps[buf_idx];

            /* Write Header: Sync Word (offset 4B) and 64-bit Timestamp (offset 8B) */
            send_words[1] = htonl(SYNC_WORD);
            send_words[2] = htonl((u32_t)(t_dma_done >> 32));
            send_words[3] = htonl((u32_t)(t_dma_done & 0xFFFFFFFFU));

            /* Copy Payload (offset 16B = word index 4) */
			#if SWAP_PAYLOAD_ENDIANNESS
				u32_t *dst_payload = &send_words[4];
				const u32_t num_words = DMA_PKT_LEN / sizeof(u32_t);
				for (u32_t i = 0; i < num_words; i++) {
					dst_payload[i] = __builtin_bswap32(active_buf[i]);
				}
			#else
            	memcpy(&send_words[4], active_buf, DMA_PKT_LEN);
			#endif

            /* Transmit UDP datagram */
            if (udp_packet_send(!FINISH) < 0) {
                return FINISH;
            }

            bd_cur = (XAxiDma_Bd *)XAxiDma_BdRingNext(RxRingPtr, bd_cur);
        }

        /*
         * CRITICAL: Free the processed BDs and immediately re-submit them
         * to hardware so the SG ring never runs out of descriptors.
         */
        int status = XAxiDma_BdRingFree(RxRingPtr, processed_bds, bd_first_ptr);
        if (status == XST_SUCCESS) {
            XAxiDma_Bd *bd_realloc_ptr = NULL;
            status = XAxiDma_BdRingAlloc(RxRingPtr, processed_bds, &bd_realloc_ptr);
            if (status == XST_SUCCESS) {
                XAxiDma_Bd *bd_reinit = bd_realloc_ptr;
                for (int b = 0; b < processed_bds; b++) {
                    /* Clear the COMPLETE status bit from previous transfer */
                    XAxiDma_BdWrite(bd_reinit, XAXIDMA_BD_STS_OFFSET, 0);
                    bd_reinit = (XAxiDma_Bd *)XAxiDma_BdRingNext(RxRingPtr, bd_reinit);
                }
                XAxiDma_BdRingToHw(RxRingPtr, processed_bds, bd_realloc_ptr);
            }
        }

    } else {
        xil_printf("[DMA TIMEOUT] Re-initializing SG DMA core...\r\n");
        XGpio_DiscreteWrite(&cfg_gpio, CFG_GPIO_SDR_ENABLE, 0x0);
        XAxiDma_Reset(&dma);
        while (!XAxiDma_ResetIsDone(&dma));
        RxSetup(&dma);
        XGpio_DiscreteWrite(&cfg_gpio, CFG_GPIO_SDR_ENABLE, 0x1);
    }

    return 0;
}

static int init_output_gpio(XGpio *InstancePtr, UINTPTR BaseAddress)
{
    XGpio_Config *gpio_config = XGpio_LookupConfig(BaseAddress);
    if (!gpio_config) {
        return XST_FAILURE;
    }

    int gpio_status = XGpio_CfgInitialize(InstancePtr, gpio_config, gpio_config->BaseAddress);
    if (gpio_status != XST_SUCCESS) {
        return gpio_status;
    }

    XGpio_SetDataDirection(InstancePtr, CFG_GPIO_SDR_ENABLE, 0x0);
    XGpio_SetDataDirection(InstancePtr, CFG_GPIO_SET_PKT_SZ, 0x0);
    return XST_SUCCESS;
}

void start_application(void)
{
    err_t err;

    xil_printf("Application program init\r\n");

    if (init_output_gpio(&cfg_gpio, CFG_GPIO_BASEADDR) != XST_SUCCESS) {
        xil_printf("GPIO Failed to initialize\r\n");
        return;
    }

    /* Hold SDR stream disabled while setting packet size and arming DMA */
    XGpio_DiscreteWrite(&cfg_gpio, CFG_GPIO_SDR_ENABLE, 0x0);
    XGpio_DiscreteWrite(&cfg_gpio, CFG_GPIO_SET_PKT_SZ, NUM_DATABEATS);

    if (RxSetup(&dma) != XST_SUCCESS) {
        xil_printf("AXI DMA Failed to initialize\r\n");
        return;
    }

    if (SetupFreeRTOSDmaInterrupts() != XST_SUCCESS) {
        xil_printf("DMA Interrupts Failed to configure\r\n");
        return;
    }

    xil_printf("GPIO, SG DMA, and Interrupts successfully configured!\r\n");

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