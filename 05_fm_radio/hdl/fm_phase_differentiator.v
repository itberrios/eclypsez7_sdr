`timescale 1ns / 1ps
//////////////////////////////////////////////////////////////////////////////////
// Module Name: fm_phase_differentiator
// Description: Multi-channel interleaved AXI4-Stream FM phase differentiator.
//              Uses an N-stage shift register so interleaved channels
//              (e.g., [Ch1, Ch2, Ch1, Ch2, ...]) differentiate against their
//              own previous phase sample.
//////////////////////////////////////////////////////////////////////////////////

/*
    Note: The first NUM_CHANNELS output beats will equal the raw initial phase
    of each channel (since phase_prev resets to 0). The downstream audio FIR
    filter will smooth out this single-sample startup transient.
*/

module fm_phase_differentiator #(
    parameter DATA_WIDTH   = 32,
    parameter NUM_CHANNELS = 2
)(
    input  wire                  clk,
    input  wire                  resetn,

    // Input from CORDIC (Interleaved Phase θ_ch[n])
    input  wire [DATA_WIDTH-1:0] s_axis_phase_tdata,
    input  wire                  s_axis_phase_tvalid,
    output wire                  s_axis_phase_tready,

    // Output Differentiated Phase (Interleaved Instantaneous Frequency / Audio)
    output reg  [DATA_WIDTH-1:0] m_axis_diff_tdata,
    output reg                   m_axis_diff_tvalid,
    input  wire                  m_axis_diff_tready
);

    reg [DATA_WIDTH-1:0] phase_prev [0:NUM_CHANNELS-1];
    integer i;

    // Ready whenever downstream is ready or our output register is currently empty
    assign s_axis_phase_tready = !m_axis_diff_tvalid || m_axis_diff_tready;

    always @(posedge clk or negedge resetn) begin
        if (!resetn) begin
            for (i = 0; i < NUM_CHANNELS; i = i + 1) begin
                phase_prev[i] <= {DATA_WIDTH{1'b0}};
            end
            m_axis_diff_tdata  <= {DATA_WIDTH{1'b0}};
            m_axis_diff_tvalid <= 1'b0;
        end else begin
            if (s_axis_phase_tvalid && s_axis_phase_tready) begin
                // Subtract the sample from NUM_CHANNELS beats ago (same channel).
                // Standard 2's complement subtraction handles [-π, +π] wrapping
                // automatically when CORDIC Phase Format is set to "Scaled Radians".
                m_axis_diff_tdata  <= $signed(s_axis_phase_tdata) - $signed(phase_prev[NUM_CHANNELS-1]);
                m_axis_diff_tvalid <= 1'b1;

                // Shift history pipeline forward by 1 beat
                phase_prev[0] <= s_axis_phase_tdata;
                for (i = 1; i < NUM_CHANNELS; i = i + 1) begin
                    phase_prev[i] <= phase_prev[i-1];
                end
            end else if (m_axis_diff_tready) begin
                m_axis_diff_tvalid <= 1'b0;
            end
        end
    end

endmodule