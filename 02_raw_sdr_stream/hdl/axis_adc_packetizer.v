`timescale 1ns / 1ps
//////////////////////////////////////////////////////////////////////////////////
// Company: 
// Engineer: 
// 
// Create Date: 10/31/2026 09:14:14 AM
// Design Name: 
// Module Name: axis_adc_packetizer
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


module axis_adc_packetizer #(
    parameter DATA_WIDTH = 32
)(
    input  wire                  clk,
    input  wire                  rst_n,
    
    // Runtime control
    input  wire                  enable,          // Enable packetizer framing
    input  wire [15:0]           cfg_packet_len,  // Target beats per DMA packet

    // Slave Interface (from continuous ADC / generator)
    input  wire [DATA_WIDTH-1:0] s_axis_tdata,
    input  wire                  s_axis_tvalid,
    output wire                  s_axis_tready,

    // Master Interface (to AXI Data FIFO / DMA)
    output wire [DATA_WIDTH-1:0] m_axis_tdata,
    output wire                  m_axis_tvalid,
    output wire                  m_axis_tlast,
    input  wire                  m_axis_tready
);

    reg [15:0] packet_beat_counter;

    // Pass-through data and valid when enabled
    assign m_axis_tdata  = s_axis_tdata;
    assign m_axis_tvalid = s_axis_tvalid && enable;
    
    // Backpressure pass-through
    assign s_axis_tready = m_axis_tready && enable;

    // Assert TLAST on the exact final beat of the configured frame size
    assign m_axis_tlast  = m_axis_tvalid && (packet_beat_counter == (cfg_packet_len - 1'b1));

    always @(posedge clk) begin
        if (!rst_n) begin
            packet_beat_counter <= 16'd0;
        end else begin
            if (!enable) begin
                packet_beat_counter <= 16'd0;
            end else if (m_axis_tvalid && m_axis_tready) begin
                if (packet_beat_counter == (cfg_packet_len - 1'b1)) begin
                    packet_beat_counter <= 16'd0; // Wrap around for next DMA packet
                end else begin
                    packet_beat_counter <= packet_beat_counter + 1'b1;
                end
            end
        end
    end

endmodule
