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
    parameter integer NUM_CHANNELS = 2   // 2 for Ch1/Ch2 interleaved
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

    // 1. Upstream Channel Tracking (Free-running at 100% real-time rate)
    reg [CHAN_BITS-1:0] in_chan_cnt;

    // 2. Downstream FIFO/DMA Frame Tracking (Only increments on accepted beats)
    reg [CHAN_BITS-1:0] out_chan_cnt;
    reg [15:0]          frame_cnt;
    reg                 active_frame_en;
    reg                 drop_rest_of_frame;

    // Guard against cfg_packet_len == 0 at startup before C initializes registers
    wire [15:0] safe_pkt_frames = (cfg_packet_len == 16'd0) ? 16'd1 : cfg_packet_len;

    wire in_last_chan     = (in_chan_cnt  == (NUM_CHANNELS - 1));
    wire out_last_chan    = (out_chan_cnt == (NUM_CHANNELS - 1));
    wire last_frame_in_pkt = (frame_cnt   >= (safe_pkt_frames - 16'd1));

    // Frame-aligned enable: turns ON on Ch1 (in_chan_cnt == 0), turns OFF after a full frame
    wire gate_en = (enable && (in_chan_cnt == {CHAN_BITS{1'b0}})) || active_frame_en;

    // Only forward a beat to the FIFO if:
    //   1. Stream is enabled (gate_en)
    //   2. We didn't drop Ch1 of this frame (drop_rest_of_frame == 0)
    //   3. The incoming channel index matches the next channel slot expected by the FIFO!
    wire chan_aligned = (in_chan_cnt == out_chan_cnt) && !drop_rest_of_frame;

    // PERMANENT NON-BLOCKING UPSTREAM: DSP pipeline never stalls!
    assign s_axis_tready = 1'b1;

    assign m_axis_tdata  = s_axis_tdata;
    assign m_axis_tvalid = s_axis_tvalid && gate_en && chan_aligned;
    assign m_axis_tkeep  = {(DATA_WIDTH/8){1'b1}};

    // Assert TLAST strictly when the FIFO accepts the final channel of the final frame
    assign m_axis_tlast  = m_axis_tvalid && out_last_chan && last_frame_in_pkt;

    // -------------------------------------------------------------------------
    // Upstream Input Channel Counter (Tracks Ch1=0, Ch2=1 coming out of DSP)
    // -------------------------------------------------------------------------
    always @(posedge clk) begin
        if (!rst_n) begin
            in_chan_cnt <= {CHAN_BITS{1'b0}};
        end else if (s_axis_tvalid) begin
            if (in_last_chan)
                in_chan_cnt <= {CHAN_BITS{1'b0}};
            else
                in_chan_cnt <= in_chan_cnt + 1'b1;
        end
    end

    // -------------------------------------------------------------------------
    // Downstream Output Packet & Frame State Machine (Tracks beats entering FIFO)
    // -------------------------------------------------------------------------
    always @(posedge clk) begin
        if (!rst_n) begin
            out_chan_cnt       <= {CHAN_BITS{1'b0}};
            frame_cnt          <= 16'd0;
            active_frame_en    <= 1'b0;
            drop_rest_of_frame <= 1'b0;
        end else begin
            // Clear drop_rest_of_frame at the end of every incoming upstream frame
            if (s_axis_tvalid && in_last_chan) begin
                drop_rest_of_frame <= 1'b0;
            end

            if (!gate_en) begin
                out_chan_cnt       <= {CHAN_BITS{1'b0}};
                frame_cnt          <= 16'd0;
                active_frame_en    <= 1'b0;
                drop_rest_of_frame <= 1'b0;
            end else if (s_axis_tvalid) begin
                if (chan_aligned && m_axis_tready) begin
                    // Beat successfully accepted into Pre-DMA FIFO!
                    if (out_last_chan) begin
                        out_chan_cnt    <= {CHAN_BITS{1'b0}};
                        active_frame_en <= enable;

                        if (last_frame_in_pkt || !enable) begin
                            frame_cnt <= 16'd0;
                        end else begin
                            frame_cnt <= frame_cnt + 16'd1;
                        end
                    end else begin
                        out_chan_cnt    <= out_chan_cnt + 1'b1;
                        active_frame_en <= 1'b1;
                    end
                end else if (chan_aligned && !m_axis_tready) begin
                    // FIFO is FULL! Drop this beat cleanly.
                    // If we just dropped Ch1 (out_chan_cnt == 0), also drop Ch2 of this frame
                    // so we never write an orphaned Ch2 into the Ch1 slot!
                    if (out_chan_cnt == {CHAN_BITS{1'b0}} && !in_last_chan) begin
                        drop_rest_of_frame <= 1'b1;
                    end
                end
            end
        end
    end

endmodule