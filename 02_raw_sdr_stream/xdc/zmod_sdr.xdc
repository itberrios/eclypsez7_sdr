# 1. Isolate asynchronous Zynq PS 100 MHz clock from Zmod SDR 122.88 MHz ADC clock domain
set_clock_groups -asynchronous \
    -group [get_clocks clk_fpga_0] \
    -group [get_clocks -include_generated_clocks [get_clocks *DcoClkIn*]]

# 2. Shift ZmodSDRController internal MMCM capture clock (Clk0 -> 'I') phase from 127.5 deg to 150.0 deg
#    This moves ~0.51 ns of excess Hold margin (+0.664 ns) into Setup (-0.922 ns) to center the data eye.
set_property CLKOUT0_PHASE 150.000 [get_cells -hier -filter {NAME =~ *ZmodSDRController*InstDataPath/InstMMCM_SDR}]

# 3. Override over-pessimistic (-40C to +85C) AD9648 input delay constraints with lab-temperature (25C) skew
#    AD9648 typical DCO-to-Data skew at 25C is +/- 0.30 ns (instead of -0.72/+0.60 ns)
set tskew_max_typ 0.300
set tskew_min_typ -0.400
set OutputDelay 1.120
set net_delay_dcoclk 0.169
set net_skew_ecl 0.007
set net_delay_typ 0.167

set_input_delay -clock [get_clocks *DcoClkIn*] -clock_fall -min [expr $tskew_min_typ + $net_delay_typ - $OutputDelay - $net_delay_dcoclk - $net_skew_ecl] [get_ports {diZmodSDR_Data_0[*]}]
set_input_delay -clock [get_clocks *DcoClkIn*] -clock_fall -max [expr $tskew_max_typ + $net_delay_typ - $OutputDelay - $net_delay_dcoclk + $net_skew_ecl] [get_ports {diZmodSDR_Data_0[*]}]
set_input_delay -clock [get_clocks *DcoClkIn*] -min -add_delay [expr $tskew_min_typ + $net_delay_typ - $OutputDelay - $net_delay_dcoclk - $net_skew_ecl] [get_ports {diZmodSDR_Data_0[*]}]
set_input_delay -clock [get_clocks *DcoClkIn*] -max -add_delay [expr $tskew_max_typ + $net_delay_typ - $OutputDelay - $net_delay_dcoclk + $net_skew_ecl] [get_ports {diZmodSDR_Data_0[*]}]


# inputs
set_property PACKAGE_PIN N22 [get_ports {diZmodSDR_Data_0[0]}]
set_property PACKAGE_PIN L21 [get_ports {diZmodSDR_Data_0[1]}]
set_property PACKAGE_PIN R16 [get_ports {diZmodSDR_Data_0[2]}]
set_property PACKAGE_PIN J18 [get_ports {diZmodSDR_Data_0[3]}]
set_property PACKAGE_PIN K18 [get_ports {diZmodSDR_Data_0[4]}]
set_property PACKAGE_PIN L19 [get_ports {diZmodSDR_Data_0[5]}]
set_property PACKAGE_PIN L18 [get_ports {diZmodSDR_Data_0[6]}]
set_property PACKAGE_PIN L22 [get_ports {diZmodSDR_Data_0[7]}]
set_property PACKAGE_PIN K20 [get_ports {diZmodSDR_Data_0[8]}]
set_property PACKAGE_PIN P16 [get_ports {diZmodSDR_Data_0[9]}]
set_property PACKAGE_PIN K19 [get_ports {diZmodSDR_Data_0[10]}]
set_property PACKAGE_PIN J22 [get_ports {diZmodSDR_Data_0[11]}]
set_property PACKAGE_PIN J21 [get_ports {diZmodSDR_Data_0[12]}]
set_property PACKAGE_PIN P22 [get_ports {diZmodSDR_Data_0[13]}]
set_property IOSTANDARD LVCMOS18 [get_ports {diZmodSDR_Data_0[*]}]

set_property PACKAGE_PIN M19 [get_ports DcoClkIn_0]
set_property IOSTANDARD LVCMOS18 [get_ports DcoClkIn_0]

set_property PACKAGE_PIN T16 [get_ports GPIO1]
set_property PACKAGE_PIN T19 [get_ports GPIO4]
set_property IOSTANDARD LVCMOS18 [get_ports GPIO*]

# outputs 
set_property PACKAGE_PIN T17 [get_ports CDCE_IIC_0_scl_io] 
set_property PACKAGE_PIN R19 [get_ports CDCE_IIC_0_sda_io]
set_property IOSTANDARD LVCMOS18 [get_ports CDCE_IIC_*]

set_property PACKAGE_PIN M22 [get_ports aZmodSync_0]
set_property IOSTANDARD LVCMOS18 [get_ports aZmodSync_0]
set_property DRIVE 4 [get_ports aZmodSync_0]

set_property PACKAGE_PIN R18 [get_ports sZmodSDR_SDIO_0]
set_property IOSTANDARD LVCMOS18 [get_ports sZmodSDR_SDIO_0]
set_property DRIVE 4 [get_ports sZmodSDR_SDIO_0]

set_property PACKAGE_PIN M21 [get_ports sZmodSDR_CS_0]
set_property IOSTANDARD LVCMOS18 [get_ports sZmodSDR_CS_0]
set_property DRIVE 4 [get_ports sZmodSDR_CS_0]

set_property PACKAGE_PIN T18 [get_ports sZmodSDR_Sclk_0]
set_property IOSTANDARD LVCMOS18 [get_ports sZmodSDR_Sclk_0]
set_property DRIVE 4 [get_ports sZmodSDR_Sclk_0]

set_property IOSTANDARD DIFF_SSTL18_I [get_ports -filter { name =~ CG_InputClk* }]
set_property PACKAGE_PIN N19 [get_ports CG_InputClk_p_0]
set_property PACKAGE_PIN N20 [get_ports CG_InputClk_n_0]
set_property SLEW SLOW [get_ports -filter { name =~ CG_InputClk* }]

set_property PACKAGE_PIN P18 [get_ports aREFSEL_0]
set_property IOSTANDARD LVCMOS18 [get_ports aREFSEL_0]

set_property PACKAGE_PIN N15 [get_ports aHW_SW_CTRL_0]
set_property IOSTANDARD LVCMOS18 [get_ports aHW_SW_CTRL_0]

set_property PACKAGE_PIN P17 [get_ports sPDNout_n_0]
set_property IOSTANDARD LVCMOS18 [get_ports sPDNout_n_0]
