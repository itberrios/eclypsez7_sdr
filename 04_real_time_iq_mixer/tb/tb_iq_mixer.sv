`timescale 1ns / 1ps
//////////////////////////////////////////////////////////////////////////////////
// Company: 
// Engineer: 
// 
// Create Date: 11/07/2026 04:47:53 PM
// Design Name: 
// Module Name: tb_iq_mixer
// Project Name: 
// Target Devices: 
// Tool Versions: 
// Description: 
// 
// Dependencies: 
// 
// Revision:
// Revision 0.01 - File Created
// Additional Comments:
// 
//////////////////////////////////////////////////////////////////////////////////


`timescale 1ns / 1ps

module tb_iq_mixer ();

    // =========================================================================
    // 1. Clocks, Test Frequencies, and Constants
    // =========================================================================
    localparam real FS_ADC_HZ      = 122880000.0;
    localparam real CLK_122_88_PER = 8.138020833; // 122.88 MHz period (ns)
    localparam real CLK_100_00_PER = 10.00000000; // 100.00 MHz period (ns)
    localparam real PI             = 3.14159265358979323846;

    // Test Tones:
    //   Ch1: ADC = 10.25 MHz, DDS1 = 10.00 MHz -> Expected Baseband = +250 kHz
    //   Ch2: ADC = 24.50 MHz, DDS2 = 25.00 MHz -> Expected Baseband = -500 kHz
    localparam real F_ADC_CH1_HZ   = 10250000.0;
    localparam real F_DDS_CH1_HZ   = 10000000.0;

    localparam real F_ADC_CH2_HZ   = 24500000.0;
    localparam real F_DDS_CH2_HZ   = 25000000.0;

    // PINC = round(f_dds * 2^32 / 122.88e6)
    localparam [31:0] PINC_CH1     = 32'd349525333; // 10.00 MHz
    localparam [31:0] PINC_CH2     = 32'd873813333; // 25.00 MHz
    localparam [15:0] TEST_CIC_R   = 16'd4;         // R=4 -> Fs_out = 25.6 MSPS
    localparam [15:0] TEST_PKT_LEN = 16'd64;        // 64 frames (128 beats) per packet

    // =========================================================================
    // 2. DUT Instantiation (design_1_wrapper)
    // =========================================================================
    reg clk_122_88 = 1'b0;
    reg clk_100_00 = 1'b0;
    reg rst_n      = 1'b0;
    
    always #(CLK_122_88_PER / 2.0) clk_122_88 = ~clk_122_88;
    always #(CLK_100_00_PER / 2.0) clk_100_00 = ~clk_100_00;

    reg  [13:0] diZmodSDR_Data_0 = 14'd0;
    reg  [0:0]  GPIO1            = 1'b1;
    reg  [0:0]  GPIO4            = 1'b1;
    wire        sZmodSDR_SDIO_0;

    design_1_wrapper uut (
        .CDCE_IIC_0_scl_io (),
        .CDCE_IIC_0_sda_io (),
        .CG_InputClk_n_0   (),
        .CG_InputClk_p_0   (),
        .DDR_addr          (),
        .DDR_ba            (),
        .DDR_cas_n         (),
        .DDR_ck_n          (),
        .DDR_ck_p          (),
        .DDR_cke           (),
        .DDR_cs_n          (),
        .DDR_dm            (),
        .DDR_dq            (),
        .DDR_dqs_n         (),
        .DDR_dqs_p         (),
        .DDR_odt           (),
        .DDR_ras_n         (),
        .DDR_reset_n       (),
        .DDR_we_n          (),
        .DcoClkIn_0        (clk_122_88),
        .FIXED_IO_ddr_vrn  (),
        .FIXED_IO_ddr_vrp  (),
        .FIXED_IO_mio      (),
        .FIXED_IO_ps_clk   (), // (clk_100_00),
        .FIXED_IO_ps_porb  (), // (rst_n),
        .FIXED_IO_ps_srstb (), // (rst_n),
        .GPIO1             (GPIO1),
        .GPIO4             (GPIO4),
        .aHW_SW_CTRL_0     (),
        .aREFSEL_0         (),
        .aZmodSync_0       (),
        .diZmodSDR_Data_0  (diZmodSDR_Data_0),
        .sPDNout_n_0       (),
        .sZmodSDR_CS_0     (),
        .sZmodSDR_SDIO_0   (sZmodSDR_SDIO_0),
        .sZmodSDR_Sclk_0   ()
    );

    // =========================================================================
    // 3. Hierarchical Signal Binding into design_1_i
    // =========================================================================
    reg  [3:0]  tb_awaddr     = 4'd0;
    reg         tb_awvalid    = 1'b0;
    reg  [31:0] tb_wdata      = 32'd0;
    reg  [3:0]  tb_wstrb      = 4'hF;
    reg         tb_wvalid     = 1'b0;
    reg         tb_bready     = 1'b1;

    reg  [31:0] tb_adc_tdata  = 32'd0;
    reg         tb_adc_tvalid = 1'b0;
    wire        tb_adc_tready;
    reg         tb_dma_tready = 1'b1;

    initial begin
        // Drive 122.88 MHz & 100 MHz clocks and active-low resets
        force uut.design_1_i.clk_wiz_0_clk_out1                 = clk_122_88;
        force uut.design_1_i.proc_sys_reset_0_peripheral_aresetn = rst_n;
        force uut.design_1_i.processing_system7_0_FCLK_CLK0     = clk_100_00;
        force uut.design_1_i.rst_ps7_0_100M_peripheral_aresetn  = rst_n;

        // Release DMA S2MM reset interlock
        force uut.design_1_i.axi_dma_0_s2mm_prmry_reset_out_n   = 1'b1;

        // Drive sdr_controller AXI4-Lite slave interface
        force uut.design_1_i.sdr_controller.s_axi_awaddr        = tb_awaddr;
        force uut.design_1_i.sdr_controller.s_axi_awvalid       = tb_awvalid;
        force uut.design_1_i.sdr_controller.s_axi_wdata         = tb_wdata;
        force uut.design_1_i.sdr_controller.s_axi_wstrb         = tb_wstrb;
        force uut.design_1_i.sdr_controller.s_axi_wvalid        = tb_wvalid;
        force uut.design_1_i.sdr_controller.s_axi_bready        = tb_bready;

        // Drive 32-bit Zmod stream into ch_1_2_splitter
        force uut.design_1_i.ch_1_2_splitter.s_axis_tdata       = tb_adc_tdata;
        force uut.design_1_i.ch_1_2_splitter.s_axis_tvalid      = tb_adc_tvalid;

        // Drain axis_data_fifo_0 M_AXIS port (emulating AXI DMA S2MM TREADY)
        force uut.design_1_i.axis_data_fifo_0.m_axis_tready     = tb_dma_tready;
    end

    assign tb_adc_tready = uut.design_1_i.ch_1_2_splitter.s_axis_tready;

    // =========================================================================
    // 4. AXI4-Lite Register Write Task
    // =========================================================================
    task automatic axi_write(input [3:0] addr, input [31:0] data);
        begin
            @(posedge clk_122_88);
            tb_awaddr  <= addr;
            tb_awvalid <= 1'b1;
            tb_wdata   <= data;
            tb_wstrb   <= 4'hF;
            tb_wvalid  <= 1'b1;

            wait (uut.design_1_i.sdr_controller.s_axi_awready &&
                  uut.design_1_i.sdr_controller.s_axi_wready);
            @(posedge clk_122_88);
            tb_awvalid <= 1'b0;
            tb_wvalid  <= 1'b0;

            wait (uut.design_1_i.sdr_controller.s_axi_bvalid);
            @(posedge clk_122_88);
            $display("[%0t ns] AXI_WRITE: Offset 0x%02h <= 0x%08h (%0d)",
                     $time, addr, data, data);
        end
    endtask

    // =========================================================================
    // 5. 122.88 MSPS Dual-Channel Sine Generator ({Ch2[15:0], Ch1[15:0]})
    // =========================================================================
    integer sample_idx = 0;
    real    val_ch1_r, val_ch2_r;
    reg signed [15:0] adc_ch1_s16, adc_ch2_s16;

    always @(posedge clk_122_88) begin
        if (!rst_n || !uut.design_1_i.sdr_controller_pkt_enable) begin
            tb_adc_tvalid <= 1'b0;
            tb_adc_tdata  <= 32'd0;
            sample_idx    <= 0;
        end else begin
            tb_adc_tvalid <= 1'b1;
            if (!tb_adc_tvalid || tb_adc_tready) begin
                val_ch1_r   = 0.85 * 32767.0 * $sin(2.0 * PI * F_ADC_CH1_HZ * real'(sample_idx) / FS_ADC_HZ);
                val_ch2_r   = 0.85 * 32767.0 * $sin(2.0 * PI * F_ADC_CH2_HZ * real'(sample_idx) / FS_ADC_HZ);

                adc_ch1_s16 = 16'(int'(val_ch1_r));
                adc_ch2_s16 = 16'(int'(val_ch2_r));

                tb_adc_tdata <= {adc_ch2_s16, adc_ch1_s16};
                sample_idx   <= sample_idx + 1;
            end
        end
    end

    // =========================================================================
    // 6. Output Packetizer Monitor, Phase/Freq Estimator & CSV Logger
    // =========================================================================
    wire [63:0] pkt_tdata  = uut.design_1_i.interleaved_packetizer.m_axis_tdata;
    wire        pkt_tvalid = uut.design_1_i.interleaved_packetizer.m_axis_tvalid;
    wire        pkt_tready = uut.design_1_i.interleaved_packetizer.m_axis_tready;
    wire        pkt_tlast  = uut.design_1_i.interleaved_packetizer.m_axis_tlast;

    shortreal   float_i, float_q;
    shortreal   ch1_i_hold, ch1_q_hold;
    integer     beat_in_pkt = 0;
    integer     frame_count = 0;
    integer     pkt_count   = 0;
    integer     csv_fd;

    always @(posedge clk_122_88) begin
        if (rst_n && pkt_tvalid && pkt_tready) begin
            float_i = $bitstoshortreal(pkt_tdata[31:0]);
            float_q = $bitstoshortreal(pkt_tdata[63:32]);

            if (float_i > 1.05 || float_i < -1.05 || float_q > 1.05 || float_q < -1.05) begin
                $error("[%0t ns] FLOAT RANGE ERROR: I=%0.6f, Q=%0.6f", $time, float_i, float_q);
            end

            if ((beat_in_pkt % 2) == 0) begin
                ch1_i_hold = float_i;
                ch1_q_hold = float_q;
                if (pkt_tlast) begin
                    $error("[%0t ns] ALIGNMENT ERROR: TLAST asserted on EVEN beat (%0d)!",
                           $time, beat_in_pkt);
                end
            end else begin
                $fwrite(csv_fd, "%0d,%0.8f,%0.8f,%0.8f,%0.8f\n",
                        frame_count, ch1_i_hold, ch1_q_hold, float_i, float_q);
                frame_count = frame_count + 1;
            end

            if (pkt_tlast) begin
                $display("[%0t ns] PACKET #%0d COMPLETE: %0d beats (%0d frames). Ch1=(%0.4f, %0.4f), Ch2=(%0.4f, %0.4f)",
                         $time, pkt_count, beat_in_pkt + 1, (beat_in_pkt + 1) / 2,
                         ch1_i_hold, ch1_q_hold, float_i, float_q);
                beat_in_pkt = 0;
                pkt_count   = pkt_count + 1;
            end else begin
                beat_in_pkt = beat_in_pkt + 1;
            end
        end
    end

    // =========================================================================
    // 7. Main Test Sequence
    // =========================================================================
    initial begin
        csv_fd = $fopen("sim_iq_out.csv", "w");
        $fwrite(csv_fd, "frame,ch1_i,ch1_q,ch2_i,ch2_q\n");

        $display("==========================================================");
        $display("  Starting 03_iq_mixer Full 122.88 MHz Pipeline Simulation");
        $display("==========================================================");

        rst_n = 1'b0;
        repeat (40) @(posedge clk_122_88);
        rst_n = 1'b1;
        repeat (40) @(posedge clk_122_88);

        // Configure sdr_controller registers over AXI4-Lite
        axi_write(4'h0, PINC_CH1);                          // DDS1 = 10.00 MHz
        axi_write(4'h4, PINC_CH2);                          // DDS2 = 25.00 MHz
        axi_write(4'h8, {16'd0, TEST_CIC_R});               // CIC R = 4
        repeat (20) @(posedge clk_122_88);
        axi_write(4'hC, (32'd1 << 16) | {16'd0, TEST_PKT_LEN}); // Enable = 1, PktLen = 64

        // Wait for first 2 packets
        wait (pkt_count == 2);

        // Inject random backpressure stalls on axis_data_fifo_0 output
        $display("[%0t ns] Injecting random TREADY stalls...", $time);
        repeat (80) begin
            @(posedge clk_122_88);
            tb_dma_tready <= $urandom_range(0, 1);
        end
        tb_dma_tready <= 1'b1;

        // Wait for 2 more packets after backpressure recovery
        wait (pkt_count == 4);

        $fclose(csv_fd);
        $display("==========================================================");
        $display("  SIMULATION PASSED: %0d dual-channel frames saved to sim_iq_out.csv", frame_count);
        $display("==========================================================");
        $finish;
    end

endmodule
