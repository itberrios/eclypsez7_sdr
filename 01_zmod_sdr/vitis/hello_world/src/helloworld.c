/******************************************************************************
 * Eclypse Z7 + Zmod SDR AXI DMA Capture & Verification
 ******************************************************************************/

#include <stdio.h>
#include <stdint.h>
#include <string.h>
#include "platform.h"
#include "xil_printf.h"
#include "xil_cache.h"
#include "xstatus.h"
#include "sleep.h"
#include "xgpio.h"
#include "xaxidma.h"
#include "xparameters.h"

/* Hardware Base Addresses */
#define DMA_BASEADDR                XPAR_XAXIDMA_0_BASEADDR
#define GPIO_BASEADDR               XPAR_XGPIO_0_BASEADDR

/* AXI GPIO Channels */
#define GPIO_CHAN_ACQ_ENABLE        1U  // Ch 1: Enable Zmod SDR + Packetizer
#define GPIO_CHAN_SET_MAX_PKT_LEN   2U  // Ch 2: Packetizer TLAST count (in beats)

/* Capture Configuration */
#define BYTES_PER_BEAT              8U          // size of 32-bit AXI-Stream (Ch0 + Ch1)
#define NUM_DATABEATS               128U        // Samples per DMA burst
#define PKT_LEN_BYTES               (NUM_DATABEATS * BYTES_PER_BEAT)
#define N_CAPS                      5U          // Number of bursts to capture
#define TOTAL_SAMPLES               (N_CAPS * NUM_DATABEATS)
#define DMA_TIMEOUT_LOOP            10000000U

/* Peripheral Instances */
static XGpio   gpio;
static XAxiDma dma;

/*
 * Allocate the DMA receive buffer directly in DDR and align it to a 32-byte
 * Cortex-A9 L1/L2 cache line boundary so Xil_DCacheInvalidateRange is safe
 * and never corrupts adjacent variables or program code.
 */
static u32 data_rx[TOTAL_SAMPLES] __attribute__((aligned(32)));

/* Function Prototypes */
static int init_dma(XAxiDma *InstancePtr, UINTPTR BaseAddress);
static int init_gpio(XGpio *InstancePtr, UINTPTR BaseAddress);

int main(void)
{
    int status;

    init_platform();
    xil_printf("\r\n=== Eclypse Z7 Zmod SDR DMA Capture ===\r\n");

    /* 1. Initialize AXI GPIO */
    status = init_gpio(&gpio, GPIO_BASEADDR);
    if (status != XST_SUCCESS) {
        xil_printf("[ERROR] AXI GPIO initialization failed (%d)\r\n", status);
        cleanup_platform();
        return XST_FAILURE;
    }

    /* Hold acquisition disabled and program packet length (in beats) */
    XGpio_DiscreteWrite(&gpio, GPIO_CHAN_ACQ_ENABLE, 0x0U);
    XGpio_DiscreteWrite(&gpio, GPIO_CHAN_SET_MAX_PKT_LEN, NUM_DATABEATS);

    /* 2. Initialize AXI DMA */
    status = init_dma(&dma, DMA_BASEADDR);
    if (status != XST_SUCCESS) {
        xil_printf("[ERROR] AXI DMA initialization failed (%d)\r\n", status);
        cleanup_platform();
        return XST_FAILURE;
    }

    /* 3. Zero out the receive buffer and flush cache to physical DDR */
    memset(data_rx, 0, sizeof(data_rx));
    Xil_DCacheFlushRange((UINTPTR)data_rx, sizeof(data_rx));
    xil_printf("Receive buffer cleared (%u bytes at 0x%08lx).\r\n",
               (unsigned int)sizeof(data_rx), (UINTPTR)data_rx);

    /**************************************************************************
     * Continuous Multi-Burst DMA Capture Loop
     * Note: Avoid xil_printf inside this loop! UART printing takes ~1.5 ms,
     * which will overflow the 8192-sample PL FIFOs at 122.88 MSPS (~66 us).
     **************************************************************************/
    u32 completed_cycles = 0;

    for (u32 cycle = 0; cycle < N_CAPS; cycle++) {
        UINTPTR burst_dest_addr = (UINTPTR)&data_rx[cycle * NUM_DATABEATS];

        /* Arm S2MM DMA channel for this burst */
        status = XAxiDma_SimpleTransfer(&dma,
                                        burst_dest_addr,
                                        PKT_LEN_BYTES,
                                        XAXIDMA_DEVICE_TO_DMA);
        if (status != XST_SUCCESS) {
            xil_printf("[FAIL] SimpleTransfer failed on cycle %u\r\n", cycle);
            break;
        }

        /* Enable acquisition stream on the very first armed transfer
         * This starts the AXI handshake that kicks off the Data Stream!
         */
        if (cycle == 0U) {
            XGpio_DiscreteWrite(&gpio, GPIO_CHAN_ACQ_ENABLE, 0x1U);
        }

        /* Poll for DMA completion with timeout */
        u32 timeout = DMA_TIMEOUT_LOOP;
        while (XAxiDma_Busy(&dma, XAXIDMA_DEVICE_TO_DMA) && (timeout > 0U)) {
            timeout--;
        }

        if (timeout == 0U) {
            xil_printf("[FAIL] DMA timed out on cycle %u!\r\n", cycle);
            break;
        }

        completed_cycles++;
    }

    /* 4. Stop acquisition stream once all bursts finish */
    XGpio_DiscreteWrite(&gpio, GPIO_CHAN_ACQ_ENABLE, 0x0U);

    /* 5. Invalidate D-Cache across the entire buffer so CPU reads fresh DDR */
    Xil_DCacheInvalidateRange((UINTPTR)data_rx, sizeof(data_rx));

    xil_printf("Captured %u / %u bursts successfully. Dumping samples:\r\n\r\n",
               completed_cycles, N_CAPS);

    /* 6. Unpack and print captured samples */
    u32 sample_idx = 0;
    for (u32 cycle = 0; cycle < completed_cycles; cycle++) {
        xil_printf("--- Burst %u ---\r\n", cycle);
        for (u32 n = 0; n < NUM_DATABEATS; n++) {
            u32 raw_word = data_rx[cycle * NUM_DATABEATS + n];

            /* Extract raw 16-bit half-words */
            u16 ch0_hex = (u16)((raw_word >> 16) & 0xFFFFU);
            u16 ch1_hex = (u16)(raw_word & 0xFFFFU);

            /*
             * Zmod SDRController left-aligns 14-bit signed two's complement
             * ADC samples into bits [15:2] of each 16-bit channel.
             * Arithmetic right-shift by 2 recovers signed 14-bit (-8192..+8191).
             */
            int16_t ch0_signed = ((int16_t)ch0_hex) >> 2;
            int16_t ch1_signed = ((int16_t)ch1_hex) >> 2;

            xil_printf("Sample %4u | Ch1: 0x%04x (%6d) | Ch2: 0x%04x (%6d)\r\n",
                       sample_idx, ch0_hex, ch0_signed, ch1_hex, ch1_signed);
            sample_idx++;
        }
    }

    xil_printf("\r\nDMA Capture Complete!\r\n");
    cleanup_platform();
    return 0;
}

static int init_dma(XAxiDma *InstancePtr, UINTPTR BaseAddress)
{
    XAxiDma_Config *cfg = XAxiDma_LookupConfig(BaseAddress);
    if (cfg == NULL) {
        xil_printf("[ERROR] No AXI DMA config found for 0x%08lx\r\n", BaseAddress);
        return XST_FAILURE;
    }

    int status = XAxiDma_CfgInitialize(InstancePtr, cfg);
    if (status != XST_SUCCESS) {
        return status;
    }

    if (XAxiDma_HasSg(InstancePtr)) {
        xil_printf("[ERROR] DMA is configured in Scatter-Gather mode\r\n");
        return XST_FAILURE;
    }

    /* Disable interrupts since we are using polled mode */
    XAxiDma_IntrDisable(InstancePtr, XAXIDMA_IRQ_ALL_MASK, XAXIDMA_DEVICE_TO_DMA);

    status = XAxiDma_Selftest(InstancePtr);
    if (status == XST_SUCCESS) {
        xil_printf("AXI DMA initialized and passed self-test.\r\n");
    }
    return status;
}

static int init_gpio(XGpio *InstancePtr, UINTPTR BaseAddress)
{
    XGpio_Config *cfg = XGpio_LookupConfig(BaseAddress);
    if (cfg == NULL) {
        xil_printf("[ERROR] No GPIO config found for 0x%08lx\r\n", BaseAddress);
        return XST_FAILURE;
    }

    int status = XGpio_CfgInitialize(InstancePtr, cfg, cfg->BaseAddress);
    if (status != XST_SUCCESS) {
        return status;
    }

    /* Set both channels to all outputs (0 = output, 1 = input) */
    XGpio_SetDataDirection(InstancePtr, GPIO_CHAN_ACQ_ENABLE, 0x0U);
    XGpio_SetDataDirection(InstancePtr, GPIO_CHAN_SET_MAX_PKT_LEN, 0x0U);

    xil_printf("AXI GPIO initialized.\r\n");
    return XST_SUCCESS;
}