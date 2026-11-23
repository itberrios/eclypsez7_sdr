`timescale 1ns / 1ps
//////////////////////////////////////////////////////////////////////////////////
// Company: 
// Engineer: 
// 
// Create Date: 11/05/2026 07:58:12 PM
// Design Name: 
// Module Name: sdr_ctrl_regs
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

module sdr_ctrl_regs (
    input  wire        aclk,               // 122.88 MHz DSP/ADC Clock
    input  wire        aresetn,            // Active-low synchronous reset

    // Hardware Interlock Input from AXI DMA (s2mm_prmry_reset_out_n)
    input  wire        dma_s2mm_rstn,

    // AXI4-Lite Slave Interface (Connect to Zynq M_AXI_GP0 Interconnect)
    input  wire [3:0]  s_axi_awaddr,
    input  wire        s_axi_awvalid,
    output reg         s_axi_awready,
    input  wire [31:0] s_axi_wdata,
    input  wire [3:0]  s_axi_wstrb,
    input  wire        s_axi_wvalid,
    output reg         s_axi_wready,
    output reg  [1:0]  s_axi_bresp,
    output reg         s_axi_bvalid,
    input  wire        s_axi_bready,
    input  wire [3:0]  s_axi_araddr,
    input  wire        s_axi_arvalid,
    output reg         s_axi_arready,
    output reg  [31:0] s_axi_rdata,
    output reg  [1:0]  s_axi_rresp,
    output reg         s_axi_rvalid,
    input  wire        s_axi_rready,

    // Offset 0x00: DDS Ch1 S_AXIS_CONFIG
    output reg  [31:0] m_axis_dds1_tdata,
    output reg         m_axis_dds1_tvalid,
    input  wire        m_axis_dds1_tready,

    // Offset 0x04: DDS Ch2 S_AXIS_CONFIG
    output reg  [31:0] m_axis_dds2_tdata,
    output reg         m_axis_dds2_tvalid,
    input  wire        m_axis_dds2_tready,

    // Offset 0x08: Shared CIC Decimation Rate S_AXIS_CONFIG (to 4-Way Broadcaster)
    output reg  [15:0] m_axis_cic_tdata,
    output reg         m_axis_cic_tvalid,
    input  wire        m_axis_cic_tready,

    // Offset 0x0C: Packetizer & Zmod SDR Control
    //   bits [15:0] = pkt_len (frames per DMA packet)
    //   bit  [16]   = sw_enable (ANDed with dma_s2mm_rstn -> pkt_enable)
    output wire        pkt_enable,
    output reg  [15:0] pkt_len
    
);

    reg sw_enable;

    // Gate software enable with AXI DMA S2MM active-low reset out
    assign pkt_enable = sw_enable & dma_s2mm_rstn;

    always @(posedge aclk) begin
        if (!aresetn) begin
            s_axi_awready      <= 1'b0;
            s_axi_wready       <= 1'b0;
            s_axi_bvalid       <= 1'b0;
            s_axi_bresp        <= 2'b00;
            s_axi_arready      <= 1'b0;
            s_axi_rvalid       <= 1'b0;
            s_axi_rresp        <= 2'b00;
            s_axi_rdata        <= 32'd0;

            m_axis_dds1_tdata  <= 32'd0;
            m_axis_dds1_tvalid <= 1'b0;
            m_axis_dds2_tdata  <= 32'd0;
            m_axis_dds2_tvalid <= 1'b0;
            m_axis_cic_tdata   <= 16'd32;   // Default R = 32
            m_axis_cic_tvalid  <= 1'b0;

            pkt_len            <= 16'd1024; // Default 1024 frames per packet
            sw_enable          <= 1'b0;     // Default disabled until C app enables
        end else begin
            // Clear AXI-Stream config strobes when downstream completes handshake
            if (m_axis_dds1_tvalid && m_axis_dds1_tready) m_axis_dds1_tvalid <= 1'b0;
            if (m_axis_dds2_tvalid && m_axis_dds2_tready) m_axis_dds2_tvalid <= 1'b0;
            if (m_axis_cic_tvalid  && m_axis_cic_tready)  m_axis_cic_tvalid  <= 1'b0;

            // Write Address & Data Handshake
            if (~s_axi_awready && s_axi_awvalid && s_axi_wvalid) begin
                s_axi_awready <= 1'b1;
                s_axi_wready  <= 1'b1;
                s_axi_bvalid  <= 1'b1;

                case (s_axi_awaddr[3:2])
                    2'b00: begin // Offset 0x00: DDS Channel 1 Phase Increment
                        m_axis_dds1_tdata  <= s_axi_wdata;
                        m_axis_dds1_tvalid <= 1'b1;
                    end
                    2'b01: begin // Offset 0x04: DDS Channel 2 Phase Increment
                        m_axis_dds2_tdata  <= s_axi_wdata;
                        m_axis_dds2_tvalid <= 1'b1;
                    end
                    2'b10: begin // Offset 0x08: CIC Decimation Factor (4..2048)
                        m_axis_cic_tdata   <= s_axi_wdata[15:0];
                        m_axis_cic_tvalid  <= 1'b1;
                    end
                    2'b11: begin // Offset 0x0C: {15'b0, sw_enable, pkt_len[15:0]}
                        if (s_axi_wstrb[0]) pkt_len[7:0]  <= s_axi_wdata[7:0];
                        if (s_axi_wstrb[1]) pkt_len[15:8] <= s_axi_wdata[15:8];
                        if (s_axi_wstrb[2]) sw_enable     <= s_axi_wdata[16];
                    end
                endcase
            end else begin
                s_axi_awready <= 1'b0;
                s_axi_wready  <= 1'b0;
                if (s_axi_bvalid && s_axi_bready)
                    s_axi_bvalid <= 1'b0;
            end

            // Read Address & Data Handshake
            if (~s_axi_arready && s_axi_arvalid) begin
                s_axi_arready <= 1'b1;
                s_axi_rvalid  <= 1'b1;
                case (s_axi_araddr[3:2])
                    2'b00: s_axi_rdata <= m_axis_dds1_tdata;
                    2'b01: s_axi_rdata <= m_axis_dds2_tdata;
                    2'b10: s_axi_rdata <= {16'd0, m_axis_cic_tdata};
                    2'b11: s_axi_rdata <= {14'd0, pkt_enable, sw_enable, pkt_len};
                endcase
            end else begin
                s_axi_arready <= 1'b0;
                if (s_axi_rvalid && s_axi_rready)
                    s_axi_rvalid <= 1'b0;
            end
        end
    end

endmodule



//module sdr_ctrl_regs (
//    input  wire        aclk,          // Connect to 122.88 MHz DSP clock
//    input  wire        aresetn,

//    // AXI4-Lite Slave Interface (Connect to Zynq M_AXI_GP0 Interconnect)
//    input  wire [3:0]  s_axi_awaddr,
//    input  wire        s_axi_awvalid,
//    output reg         s_axi_awready,
//    input  wire [31:0] s_axi_wdata,
//    input  wire [3:0]  s_axi_wstrb,
//    input  wire        s_axi_wvalid,
//    output reg         s_axi_wready,
//    output reg  [1:0]  s_axi_bresp,
//    output reg         s_axi_bvalid,
//    input  wire        s_axi_bready,
//    input  wire [3:0]  s_axi_araddr,
//    input  wire        s_axi_arvalid,
//    output reg         s_axi_arready,
//    output reg  [31:0] s_axi_rdata,
//    output reg  [1:0]  s_axi_rresp,
//    output reg         s_axi_rvalid,
//    input  wire        s_axi_rready,

//    // DDS Ch1 S_AXIS_CONFIG (Offset 0x00)
//    output reg  [31:0] m_axis_dds1_tdata,
//    output reg         m_axis_dds1_tvalid,
//    input  wire        m_axis_dds1_tready,

//    // DDS Ch2 S_AXIS_CONFIG (Offset 0x04)
//    output reg  [31:0] m_axis_dds2_tdata,
//    output reg         m_axis_dds2_tvalid,
//    input  wire        m_axis_dds2_tready,

//    // Shared CIC Decimation Rate S_AXIS_CONFIG (Offset 0x08)
//    output reg  [15:0] m_axis_cic_tdata,
//    output reg         m_axis_cic_tvalid,
//    input  wire        m_axis_cic_tready
//);

//    always @(posedge aclk) begin
//        if (!aresetn) begin
//            s_axi_awready      <= 1'b0;
//            s_axi_wready       <= 1'b0;
//            s_axi_bvalid       <= 1'b0;
//            s_axi_bresp        <= 2'b00;
//            s_axi_arready      <= 1'b0;
//            s_axi_rvalid       <= 1'b0;
//            s_axi_rresp        <= 2'b00;
//            s_axi_rdata        <= 32'd0;
//            m_axis_dds1_tdata  <= 32'd0;
//            m_axis_dds1_tvalid <= 1'b0;
//            m_axis_dds2_tdata  <= 32'd0;
//            m_axis_dds2_tvalid <= 1'b0;
//            m_axis_cic_tdata   <= 16'd32;
//            m_axis_cic_tvalid  <= 1'b0;
//        end else begin
////            // Default: Deassert single-cycle AXI-Stream tvalid strobes
////            m_axis_dds1_tvalid <= 1'b0;
////            m_axis_dds2_tvalid <= 1'b0;
////            m_axis_cic_tvalid  <= 1'b0;
            
//            // Clear tvalid only when the downstream slave (or Broadcaster) accepts the transfer
//            if (m_axis_dds1_tvalid && m_axis_dds1_tready) m_axis_dds1_tvalid <= 1'b0;
//            if (m_axis_dds2_tvalid && m_axis_dds2_tready) m_axis_dds2_tvalid <= 1'b0;
//            if (m_axis_cic_tvalid  && m_axis_cic_tready)  m_axis_cic_tvalid  <= 1'b0;

//            // Accept write address & data simultaneously
//            if (~s_axi_awready && s_axi_awvalid && s_axi_wvalid) begin
//                s_axi_awready <= 1'b1;
//                s_axi_wready  <= 1'b1;
//                s_axi_bvalid  <= 1'b1;

//                case (s_axi_awaddr[3:2])
//                    2'b00: begin // Offset 0x00: DDS Channel 1 Phase Increment
//                        m_axis_dds1_tdata  <= s_axi_wdata;
//                        m_axis_dds1_tvalid <= 1'b1;
//                    end
//                    2'b01: begin // Offset 0x04: DDS Channel 2 Phase Increment
//                        m_axis_dds2_tdata  <= s_axi_wdata;
//                        m_axis_dds2_tvalid <= 1'b1;
//                    end
//                    2'b10: begin // Offset 0x08: CIC Decimation Factor (16..128)
//                        m_axis_cic_tdata   <= s_axi_wdata[15:0];
//                        m_axis_cic_tvalid  <= 1'b1;
//                    end
//                    default: ;
//                endcase
//            end else begin
//                s_axi_awready <= 1'b0;
//                s_axi_wready  <= 1'b0;
//                if (s_axi_bvalid && s_axi_bready)
//                    s_axi_bvalid <= 1'b0;
//            end

//            // Simple Readback
//            if (~s_axi_arready && s_axi_arvalid) begin
//                s_axi_arready <= 1'b1;
//                s_axi_rvalid  <= 1'b1;
//                case (s_axi_araddr[3:2])
//                    2'b00: s_axi_rdata <= m_axis_dds1_tdata;
//                    2'b01: s_axi_rdata <= m_axis_dds2_tdata;
//                    2'b10: s_axi_rdata <= {16'd0, m_axis_cic_tdata};
//                    default: s_axi_rdata <= 32'd0;
//                endcase
//            end else begin
//                s_axi_arready <= 1'b0;
//                if (s_axi_rvalid && s_axi_rready)
//                    s_axi_rvalid <= 1'b0;
//            end
//        end
//    end
//endmodule