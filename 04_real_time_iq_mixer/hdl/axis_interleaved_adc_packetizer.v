`timescale 1ns / 1ps
//////////////////////////////////////////////////////////////////////////////////
// Company: 
// Engineer: 
// 
// Create Date: 11/07/2026 10:00:24 AM
// Design Name: 
// Module Name: axis_interleaved_adc_packetizer
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

module axis_interleaved_adc_packetizer #(
    parameter integer DATA_WIDTH   = 64, // 64-bit for [Q_float32, I_float32]
    parameter integer NUM_CHANNELS = 2   // 2 for Ch1/Ch2, or 3, 4, etc.
)(
    input  wire                    clk,
    input  wire                    rst_n,
    
    // Runtime control
    input  wire                    enable,         // Enable packetizer framing
    input  wire [15:0]             cfg_packet_len, // Target MULTI-CHANNEL FRAMES per DMA packet

    // Slave Interface (from Fixed-to-Float Combiner)
    input  wire [DATA_WIDTH-1:0]   s_axis_tdata,
    input  wire                    s_axis_tvalid,
    output wire                    s_axis_tready,

    // Master Interface (to AXI Data FIFO / DMA)
    output wire [DATA_WIDTH-1:0]   m_axis_tdata,
    output wire                    m_axis_tvalid,
    output wire                    m_axis_tlast,
    output wire [DATA_WIDTH/8-1:0] m_axis_tkeep,
    input  wire                    m_axis_tready
);

    localparam integer CHAN_BITS = (NUM_CHANNELS > 1) ? $clog2(NUM_CHANNELS) : 1;

    reg [CHAN_BITS-1:0] chan_cnt;
    reg [15:0]          frame_cnt;
    reg                 active_frame_en;

    // Guard against cfg_packet_len == 0 at startup before C initializes registers
    wire [15:0] safe_pkt_frames = (cfg_packet_len == 16'd0) ? 16'd1 : cfg_packet_len;

    // Frame & Packet boundary flags
    wire last_chan_in_frame = (chan_cnt == (NUM_CHANNELS - 1));
    wire last_frame_in_pkt  = (frame_cnt >= (safe_pkt_frames - 16'd1));

    // Frame-aligned enable: turns ON immediately, but only turns OFF at a frame boundary
    // so we never strand Channel 2 inside the upstream 128-to-64 Width Converter!
    wire gate_en = enable | active_frame_en;

    // Pass-through data, valid, ready, and byte-keep
    assign m_axis_tdata  = s_axis_tdata;
    assign m_axis_tvalid = s_axis_tvalid && gate_en;
    assign s_axis_tready = m_axis_tready && gate_en;
    assign m_axis_tkeep  = {(DATA_WIDTH/8){1'b1}};

    // Assert TLAST strictly when BOTH the channel counter and frame counter finish
    assign m_axis_tlast  = m_axis_tvalid && last_chan_in_frame && last_frame_in_pkt;

    always @(posedge clk) begin
        if (!rst_n) begin
            chan_cnt        <= {CHAN_BITS{1'b0}};
            frame_cnt       <= 16'd0;
            active_frame_en <= 1'b0;
        end else begin
            if (!gate_en) begin
                chan_cnt        <= {CHAN_BITS{1'b0}};
                frame_cnt       <= 16'd0;
                active_frame_en <= 1'b0;
            end else if (m_axis_tvalid && m_axis_tready) begin
                if (last_chan_in_frame) begin
                    chan_cnt <= {CHAN_BITS{1'b0}};

                    // Only allow the stream to close on a completed multi-channel frame
                    active_frame_en <= enable;

                    if (last_frame_in_pkt || !enable) begin
                        frame_cnt <= 16'd0;
                    end else begin
                        frame_cnt <= frame_cnt + 16'd1;
                    end
                end else begin
                    chan_cnt        <= chan_cnt + 1'b1;
                    active_frame_en <= 1'b1; // Lock gate open until all N channels pass!
                end
            end
        end
    end

endmodule