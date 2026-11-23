# Eclypse Z7 Dual-Channel Real-Time IQ SDR Receiver (`04_real_time_iq_mixer`)

## 1. Overview

This project implements a real-time, dual-channel Digital Downconverter (DDC) and Gigabit Ethernet UDP streaming receiver on the **Digilent Eclypse Z7** (Xilinx Zynq-7020 SoC) paired with the **Zmod SDR (AD9648)** dual-channel 14-bit, 122.88 MSPS ADC.

All FPGA Digital Signal Processing (DSP), AXI4-Stream packetization, Scatter-Gather AXI DMA transfers, and AXI4-Lite control logic operate synchronously on a single phase-locked **122.88 MHz** clock domain derived from the Zmod SDR ADC Data Clock Output (`DcoClkIn_0`).

### Key Capabilities
* **Dual Independent NCOs:** 32-bit phase accumulators (`0.0286 Hz` tuning resolution) supporting direct baseband and undersampled higher-Nyquist-zone RF reception.
* **Runtime Programmable Decimation:** Shared CIC decimation factor ($R_{\text{CIC}} \in [4, 2048]$) cascaded with a $5/6$ rational-rate compensation FIR filter.
* **True Non-Blocking Real-Time DSP:** Upstream DSP blocks run at 100% duty cycle (`TREADY = 1`) with frame-synchronized backpressure protection in `axis_interleaved_adc_packetizer`.
* **828.4 Mbps Sustained UDP Streaming:** Zero-copy cache-line aligned buffer offsets, 512-descriptor (`768 KB`) Scatter-Gather DMA ring with `ENABLE_TURBO` natural L1/L2 cache eviction, 16-packet interrupt coalescing, and FreeRTOS/lwIP integration achieve full theoretical line rate at $R_{\text{CIC}} = 16$ ($6.40\text{ MSPS/ch}$).

---

## 2. Hardware DSP Pipeline Architecture

### 2.1 Clock Domains
1. **`100.00 MHz` (`FCLK_CLK0` from Zynq PS7):** Used strictly by `ZmodSDRController_0` (`SysClk100` and `ClockGenPriRefClk`) to initialize the AD9648 ADC over SPI and the CDCE925 clock synthesizer over I2C.
2. **`122.88 MHz` (`clk_wiz_0/clk_out1`):** Locked to `ZmodDcoClkOut`. Drives the entire DSP datapath, `sdr_ctrl_regs`, `axi_smc`, `axi_dma_0`, `axi_mem_intercon`, `M_AXI_GP0_ACLK`, and `S_AXI_HP0_ACLK`.

### 2.2 End-to-End Signal Flow
1. **ADC Capture & CDC (`ZmodSDRController_0` $\rightarrow$ `zmod_cdc_fifo`):** Outputs a 32-bit packed word `{Ch2[15:0], Ch1[15:0]}` at $122.88\text{ MSPS}$.
2. **Channel Splitter (`ch_1_2_splitter`):** Splits the 32-bit stream into two sign-extended/zero-padded 32-bit streams (`M00_AXIS` = Ch1, `M01_AXIS` = Ch2).
3. **Complex Mixing (`ch1_nco`/`ch2_nco` + `ch1_mixer`/`ch2_mixer`):** Multiplies each 16-bit real ADC channel by an 18-bit complex exponential ($e^{-j\omega_0 n} = \cos(\omega_0 n) - j\sin(\omega_0 n)$), producing a 64-bit complex output `{Q[31:0], I[31:0]}`.
4. **Programmable CIC Decimators (`ch1_i_cic`, `ch1_q_cic`, `ch2_i_cic`, `ch2_q_cic`):** 5-stage CIC decimation filters with runtime rate $R_{\text{CIC}} \in [4, 2048]$.
5. **4-Path Rational FIR Filter (`axis_combiner_1` $\rightarrow$ `fir_compiler_0`):** Combines `{Ch2_Q, Ch2_I, Ch1_Q, Ch1_I}` into a 128-bit super-frame and applies a $5/6$ rational interpolation/decimation compensation filter (`Maximize_Dynamic_Range`).
6. **Channel Interleaver (`ch1_ch2_interleaver`):** An `axis_dwidth_converter` ($16\text{ bytes} \rightarrow 8\text{ bytes}$) that serializes each 128-bit 4-path FIR output into two sequential 64-bit AXI4-Stream beats:
   * **Even Beat (`Beat 0`):** `{Ch1_Q[31:0], Ch1_I[31:0]}` (8 bytes)
   * **Odd Beat (`Beat 1`):** `{Ch2_Q[31:0], Ch2_I[31:0]}` (8 bytes)
7. **IEEE-754 Single-Precision Float Conversion (`i_float_converter` & `q_float_converter`):** Converts signed 32-bit fixed-point (`Fix32_31`, range $[-1.0, +1.0)$) into 32-bit IEEE-754 `float32` (`[Q_float32, I_float32]`).
8. **Frame-Safe Packetizer (`axis_interleaved_adc_packetizer`):** Holds `s_axis_tready = 1'b1` permanently and asserts `m_axis_tlast` every `cfg_packet_len` (`89`) dual-channel frames (`178` beats = `1,424` bytes).
9. **Pre-DMA FIFO & Scatter-Gather DMA (`axis_data_fifo_0` $\rightarrow$ `axi_dma_0`):** Buffers up to 8,192 beats (`64 KB`, ~45 packets) in PL BRAM and bursts 1,424-byte packets into DDR3 RAM via `S_AXI_HP0`.

### 2.3 Order of Data Transfers (PL to Host PC)
1. **FPGA to DDR3 RAM (`AXI DMA S2MM via S_AXI_HP0`):** The AXI DMA engine writes each 1,424-byte interleaved IQ payload directly into physical DDR3 RAM at `&rx_buffer_pool[i][8]` (offset `+32 bytes`, Cache Line 1).
2. **In-Place CPU Header Insertion (`rx_buffer_pool` Cache Line 0):** Upon waking from the DMA interrupt, the FreeRTOS task writes the 16-byte UDP telemetry header directly into `&rx_buffer_pool[i][4..7]` (bytes `16..31`, immediately preceding the DMA payload).
3. **UDP Socket Transmission (`lwip_sendto` $\rightarrow$ Gigabit Ethernet DMA):** The contiguous 1,440-byte buffer starting at `&rx_buffer_pool[i][4]` is passed to lwIP and transmitted over the physical 1G Ethernet link to the host PC.

---

## 3. Sample Rate & Bandwidth Math

The post-FIR output sample rate per channel ($f_{s,\text{out}}$) is determined by the ADC clock ($122.88\text{ MSPS}$), the programmable CIC rate ($R_{\text{CIC}}$), and the fixed $5/6$ FIR rate:

$$f_{s,\text{out}} = \frac{122.88\text{ MSPS}}{R_{\text{CIC}}} \times \frac{5}{6} = \frac{102.4\text{ MSPS}}{R_{\text{CIC}}}$$

Each time instant produces **1 Dual-Channel Frame = 2 AXI Beats = 16 Bytes** (`Ch1_I`, `Ch1_Q`, `Ch2_I`, `Ch2_Q`). Each UDP packet carries **89 Frames (`1,424` payload bytes)** plus a **16-byte UDP application header (`1,440` total UDP payload bytes)**:

| $R_{\text{CIC}}$ | Total Decimation ($R_{\text{CIC}} \times \frac{6}{5}$) | Sample Rate / Channel ($f_{s,\text{out}}$) | IQ Payload Rate ($16\text{ B/frame}$) | Total UDP Payload Rate (w/ 16B Header) | Packet Rate ($\text{pkts/sec}$) | 1G Ethernet Compatibility |
| :---: | :---: | :---: | :---: | :---: | :---: | :---: |
| **4** | $4.8\times$ | $25.60\text{ MSPS}$ | $3,276.8\text{ Mbps}$ | $3,313.6\text{ Mbps}$ | $287,640\text{ pkt/s}$ | Exceeds 1G Line Rate |
| **8** | $9.6\times$ | $12.80\text{ MSPS}$ | $1,638.4\text{ Mbps}$ | $1,656.8\text{ Mbps}$ | $143,820\text{ pkt/s}$ | Exceeds 1G Line Rate |
| **16** | $19.2\times$ | **$6.40\text{ MSPS}$** | **$819.2\text{ Mbps}$** | **$828.4\text{ Mbps}$** | **$71,910\text{ pkt/s}$** | **Max Real-Time 1G Rate** |
| **32** | $38.4\times$ | **$3.20\text{ MSPS}$** | **$409.6\text{ Mbps}$** | **$414.2\text{ Mbps}$** | **$35,955\text{ pkt/s}$** | **Optimal Narrowband / FM** |
| **64** | $76.8\times$ | $1.60\text{ MSPS}$ | $204.8\text{ Mbps}$ | $207.1\text{ Mbps}$ | $17,978\text{ pkt/s}$ | Low Bandwidth |

---

## 4. Memory-Mapped Register Interface (`sdr_ctrl_regs`)

The `sdr_controller` block (`sdr_ctrl_regs.v`) is mapped on the Zynq `M_AXI_GP0` bus at base address **`0x40000000`** (`XPAR_SDR_CONTROLLER_BASEADDR`).

| Offset | Macro Name | Bits | Access | Description |
| :---: | :--- | :---: | :---: | :--- |
| **`0x00`** | `SDR_REG_DDS1_PINC` | `[31:0]` | R/W | **Channel 1 NCO Phase Increment ($\text{PINC}_1$).** Writing this register latches the 32-bit PINC and pulses `m_axis_dds1_tvalid` for 1 clock cycle. |
| **`0x04`** | `SDR_REG_DDS2_PINC` | `[31:0]` | R/W | **Channel 2 NCO Phase Increment ($\text{PINC}_2$).** Writing this register latches the 32-bit PINC and pulses `m_axis_dds2_tvalid` for 1 clock cycle. |
| **`0x08`** | `SDR_REG_CIC_RATE` | `[15:0]` | R/W | **Shared CIC Decimation Factor ($R_{\text{CIC}}$).** Valid range: `4` to `2048`. Writing pulses `m_axis_cic_tvalid` to all 4 CIC filters via `axis_broadcaster_0`. |
| **`0x0C`** | `SDR_REG_PKT_CTRL` | `[16]`<br>`[15:0]` | R/W | **Bit `[16]` (`sw_enable`):** Master stream enable (hardware-ANDed with `dma_s2mm_rstn` to drive `pkt_enable`).<br>**Bits `[15:0]` (`pkt_len`):** Dual-channel frames per DMA packet (`89`). |

### NCO Phase Increment Calculation
For a target RF frequency $f_{\text{RF}}$ sampled at $f_s = 122.88\text{ MHz}$, `calculate_dds_pinc()` folds $f_{\text{RF}}$ into the first Nyquist zone ($[0, 61.44\text{ MHz}]$) to support both direct and undersampled Nyquist zones:

$$z = \left\lfloor \frac{f_{\text{RF}}}{f_s / 2} \right\rfloor + 1, \quad f_{\text{alias}} = \begin{cases} z \cdot \frac{f_s}{2} - f_{\text{RF}} & \text{if } z \text{ is even} \\ f_{\text{RF}} - (z - 1)\frac{f_s}{2} & \text{if } z \text{ is odd} \end{cases}$$

$$\text{PINC} = \text{round}\left( \frac{f_{\text{alias}}}{122.88 \times 10^6} \times 2^{32} \right)$$

---

## 5. FreeRTOS Zero-Copy Scatter-Gather DMA & Ring Architecture

### 5.1 How Ring Count vs. Individual Buffer Size Are Configured
1. **Number of Buffers in the Ring (`NUM_BDS = 512U`):**
   Specified globally when creating the Scatter-Gather descriptor ring in DDR3 via `XAxiDma_BdRingCreate(..., NUM_BDS)`.
2. **Physical Allocation Size per Buffer (`PKT_ALLOC_BYTES = 1536U`):**
   Statically allocated in DDR3 RAM via `static u32_t rx_buffer_pool[NUM_BDS][PKT_ALLOC_BYTES / sizeof(u32_t)] __attribute__((aligned(32)))`. This reserves 1,536 bytes (48 Cortex-A9 cache lines) per slot, giving a 768 KB total pool.
3. **Hardware DMA Transfer Length per Descriptor (`DMA_PKT_LEN = 1424U`):**
   Programmed per Buffer Descriptor via `XAxiDma_BdSetLength(BdCurPtr, DMA_PKT_LEN, ...)`. This instructs the AXI DMA S2MM engine to expect exactly 1,424 bytes from the PL packetizer (closed by `TLAST`) before marking the descriptor complete and advancing to the next BD.

### 5.2 Cache-Line Aligned Buffer Pool Layout (`rx_buffer_pool`)

```text
rx_buffer_pool[buf_idx] (1,536 Bytes Total, 32-Byte Aligned):
+-----------------------------------------------------------------------+
| Words [0..3]   (Bytes 0..15)    : Unused Padding (16 Bytes)           |  \
| Words [4..7]   (Bytes 16..31)   : 16-Byte UDP Packet Header           |   } Cache Line 0 (Bytes 0..31)
+-----------------------------------------------------------------------+  /
| Words [8..363] (Bytes 32..1455) : 1,424-Byte AXI DMA S2MM IQ Payload  |  --> Cache Lines 1..45
+-----------------------------------------------------------------------+
| Words [364..383] (Bytes 1456..1535) : Tail Padding (80 Bytes)         |
+-----------------------------------------------------------------------+
```

* **Why DMA Starts at `&rx_buffer_pool[i][8]` (Byte 32):**
  Pointing the AXI DMA Buffer Descriptor (`XAxiDma_BdSetBufAddr`) to `&rx_buffer_pool[i][8]` places the 1,424-byte DMA write on **Cache Line 1 (`Byte 32`)**, leaving **Words `[4..7]` (`Bytes 16..31`)** in Cache Line 0 immediately ahead of the payload.
* When a BD completes, `transfer_data()` writes the 16-byte UDP header directly into `&active_buf[4..7]` and passes `&active_buf[4]` (`1,440` contiguous bytes) straight to `lwip_sendto()`—eliminating any intermediate `send_buf` copy.

### 5.3 Wire Packet Format (`1,440 Bytes` per UDP Datagram)
* **Header (`Bytes 0..15`, Big-Endian Network Byte Order):**
  * `Bytes 0..3` (`int32`): `packet_id` (increments by 1 per packet; `-1` indicates end-of-stream).
  * `Bytes 4..7` (`uint32`): `0xDEADBEEF` (`SYNC_WORD`).
  * `Bytes 8..15` (`uint64`): 64-bit Zynq ARM Global Timer timestamp (`XTime`).
* **Payload (`Bytes 16..1439`, Little-Endian IEEE-754 `float32`):**
  * `89` dual-channel frames (`178` beats = `356` floats):
    `[Ch1_I0, Ch1_Q0, Ch2_I0, Ch2_Q0, Ch1_I1, Ch1_Q1, Ch2_I1, Ch2_Q1, ...]`

### 5.4 Real-Time Loop & RTOS Optimizations
* **Binary Semaphore Signaling (`xSemaphoreCreateBinary`):** Replaces counting-semaphore token-draining loops with a single atomic wakeup per coalesced batch (`DMA_COALESCE_COUNT = 16`).
* **Single-Pass BD Status Clearing:** `XAxiDma_BdWrite(bd_cur, XAXIDMA_BD_STS_OFFSET, 0)` is executed inline inside the primary packet transmission loop while `bd_cur` is already loaded in a CPU register, eliminating a redundant pointer-chasing loop during `XAxiDma_BdRingAlloc()`.
* **32-Bit Retry Counter (`u32 retries`):** Prevents 8-bit truncation when tuning `MAX_SEND_RETRY` with microsecond `ERROR_SLEEP` intervals (`2 us`).
* **7.11 ms UART Print Cushion (`NUM_BDS = 512`):** Printing the periodic 5-second bandwidth report over polled UART at 115,200 baud blocks the CPU for `~4.8 ms` (~345 packets). The 512-BD ring provides `7.11 ms` of hardware DMA buffering so UART telemetry prints never overflow the ring.

---

## 6. Deep Dive: `ENABLE_TURBO`, Cache Coherency, and `S_AXI_HP0` vs. `S_AXI_ACP`

### 6.1 Why Software Cache Invalidation (`Xil_DCacheInvalidateRange`) Costs ~40 Mbps
In Xilinx's BSP, calling `Xil_DCacheInvalidateRange((UINTPTR)&active_buf[8], DMA_PKT_LEN)` on every packet executes two operations across 45 cache lines (`1,424 bytes`):
1. `Xil_L1DCacheInvalidateRange()`: Fast internal CP15 register operations.
2. `Xil_L2CacheInvalidateRange()`: Slow Memory-Mapped I/O (MMIO) writes over the internal bus to the PL310 L2 Cache Controller (`0xF8F02770`) plus hardware spinlock polling on every cache line.

At $71,910\text{ packets/sec}$ ($R_{\text{CIC}} = 16$), that translates to **3.23 million slow L2 MMIO register writes and DDR3 cache-miss stalls per second**, capping throughput at `~785–790 Mbps`.

### 6.2 How `ENABLE_TURBO` (`#define ENABLE_TURBO 1`) Achieves 828.4 Mbps with `NUM_BDS = 512`
Setting `ENABLE_TURBO` to `1` bypasses `Xil_DCacheInvalidateRange()` in `transfer_data()`, immediately unlocking the full **828.4 Mbps** line rate.

Why does `ENABLE_TURBO` work reliably when `NUM_BDS = 512` (`768 KB` ring)?
1. **Physical DDR3 Isolation:** `rx_buffer_pool` occupies 768 KB of dedicated physical DDR3 RAM (`512 * 1,536 bytes`). The AXI DMA writes new ADC samples directly into DDR3 via `S_AXI_HP0`, bypassing the CPU's internal caches (**32 KB L1** and **512 KB L2**).
2. **100% Natural L1 Cache Eviction (`768 KB` Pool $\gg$ `32 KB` L1):** Because the 768 KB buffer ring is $24\times$ larger than the 32 KB L1 data cache, reading through the ring naturally flushes old packets out of L1 every ~20 packets.
3. **Natural L2 Cache Self-Eviction (`768 KB` Pool $>$ `512 KB` L2):** With `NUM_BDS = 512`, the `768 KB` buffer ring alone is **$1.5\times$ larger than the entire 512 KB L2 cache**. Combined with lwIP's `pbuf` pool (`~384 KB`) and FreeRTOS task stacks, the active working set per ring revolution exceeds **1.15 MB** ($>2.2\times$ the L2 cache size). As the CPU cycles through all 512 buffers and allocates lwIP `pbufs` that share modulo L2 cache set indices (bits `[15:5]`), old cached lines from pass $N$ are naturally evicted from L2 before slot `0` is reused 512 packets later (`7.11 ms` at $R_{\text{CIC}} = 16$), forcing the CPU to fetch fresh ADC samples from DDR3.
   *(Note: If `NUM_BDS` is reduced to `<= 256` (`384 KB`), the pool fits inside the 512 KB L2 cache, and un-evicted L2 lines can return stale samples from the previous ring revolution.)*

### 6.3 Architectural Comparison: `S_AXI_HP0` vs. `S_AXI_ACP`
On the Zynq-7000 SoC, PL masters can write to DDR3 via two types of 64-bit AXI ports:
* **`S_AXI_HP0` (High-Performance Port 0 — Used in This Design):**
  * Connects directly to the DDR3 memory controller, bypassing the Cortex-A9 Snoop Control Unit (SCU) and L2 cache.
  * **Why `S_AXI_HP0` is Superior for Real-Time ADC Streaming:** Provides deterministic, fixed-latency hardware burst writes straight into DDR3 without polluting or thrashing the CPU's L1/L2 caches with continuous high-rate ADC writes while the CPU is executing lwIP and FreeRTOS code.
* **`S_AXI_ACP` (Accelerator Coherency Port):**
  * Routes DMA writes through the Cortex-A9 SCU to automatically invalidate/update L1/L2 cache tags in hardware.
  * While it eliminates software cache invalidation for small shared control structures, streaming 828 Mbps of unidirectional ADC data through the ACP causes severe cache thrashing and SCU snoop arbitration bottlenecks.

---

## 7. Building & Running the Project

### 7.1 Linux Host Network Tuning (Required for 828 Mbps Capture)
Before running the Python receiver at $R_{\text{CIC}} = 16$, increase the Linux kernel UDP receive buffer limit from the 208 KB default to **32 MB** so OS scheduling pauses never drop packets:

```bash
sudo sysctl -w net.core.rmem_max=33554432
sudo sysctl -w net.core.rmem_default=33554432
```

### 7.2 Vitis Firmware Configuration (`udp_perf_client.h`)
* Configure target RF frequencies (`NCO_CH1_FREQ`, `NCO_CH2_FREQ`), decimation rate (`SDR_DEFAULT_CIC_RATE`), `ENABLE_TURBO`, and host IP (`UDP_SERVER_IP_ADDRESS`):
  ```c
  #define UDP_SERVER_IP_ADDRESS       "192.168.1.100"
  #define NCO_CH1_FREQ                90000000.0   /* 90.0 MHz */
  #define NCO_CH2_FREQ                100000000.0  /* 100.0 MHz */
  #define SDR_DEFAULT_CIC_RATE        16U          /* 6.40 MSPS/ch (828.4 Mbps) */
  #define SWAP_PAYLOAD_ENDIANNESS     0            /* Native Little-Endian float32 */
  #define ENABLE_TURBO                1            /* 1 = Skip SW cache invalidate (828.4 Mbps) */
  ```

### 7.3 Capturing & Processing Streams in Python / GNU Radio
1. Start the Python receiver (`PAYLOAD_BIG_ENDIAN = False` to match `SWAP_PAYLOAD_ENDIANNESS = 0`):
   ```bash
   python3 receive_sdr_iq.py
   ```
2. Load the recorded `.bin` files directly into **GNU Radio** (`File Source` $\rightarrow$ `Output Type: complex`) or **NumPy**:
   ```python
   import numpy as np
   ch1_iq = np.fromfile("data/sdr_ch1_iq_YYYYMMDD_HHMMSS.bin", dtype=np.complex64)
   ch2_iq = np.fromfile("data/sdr_ch2_iq_YYYYMMDD_HHMMSS.bin", dtype=np.complex64)
   ```