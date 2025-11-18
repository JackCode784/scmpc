
log_wave -r /
set designtopgroup [add_wave_group "Design Top Signals"]
set coutputgroup [add_wave_group "C Outputs" -into $designtopgroup]
set uOpt_group [add_wave_group uOpt(wire) -into $coutputgroup]
add_wave /apatb_MPCwithMADS_top/AESL_inst_MPCwithMADS/uOpt_ap_vld -into $uOpt_group -color #ffff00 -radix hex
add_wave /apatb_MPCwithMADS_top/AESL_inst_MPCwithMADS/uOpt -into $uOpt_group -radix hex
set cinputgroup [add_wave_group "C Inputs" -into $designtopgroup]
set ref_group [add_wave_group ref(wire) -into $cinputgroup]
add_wave /apatb_MPCwithMADS_top/AESL_inst_MPCwithMADS/ref -into $ref_group -radix hex
set par_1_group [add_wave_group par_1(wire) -into $cinputgroup]
add_wave /apatb_MPCwithMADS_top/AESL_inst_MPCwithMADS/par_1 -into $par_1_group -radix hex
set par_0_group [add_wave_group par_0(wire) -into $cinputgroup]
add_wave /apatb_MPCwithMADS_top/AESL_inst_MPCwithMADS/par_0 -into $par_0_group -radix hex
set x_1_group [add_wave_group x_1(wire) -into $cinputgroup]
add_wave /apatb_MPCwithMADS_top/AESL_inst_MPCwithMADS/x_1 -into $x_1_group -radix hex
set x_0_group [add_wave_group x_0(wire) -into $cinputgroup]
add_wave /apatb_MPCwithMADS_top/AESL_inst_MPCwithMADS/x_0 -into $x_0_group -radix hex
set blocksiggroup [add_wave_group "Block-level IO Handshake" -into $designtopgroup]
add_wave /apatb_MPCwithMADS_top/AESL_inst_MPCwithMADS/ap_start -into $blocksiggroup
add_wave /apatb_MPCwithMADS_top/AESL_inst_MPCwithMADS/ap_done -into $blocksiggroup
add_wave /apatb_MPCwithMADS_top/AESL_inst_MPCwithMADS/ap_idle -into $blocksiggroup
add_wave /apatb_MPCwithMADS_top/AESL_inst_MPCwithMADS/ap_ready -into $blocksiggroup
set resetgroup [add_wave_group "Reset" -into $designtopgroup]
add_wave /apatb_MPCwithMADS_top/AESL_inst_MPCwithMADS/ap_rst -into $resetgroup
set clockgroup [add_wave_group "Clock" -into $designtopgroup]
add_wave /apatb_MPCwithMADS_top/AESL_inst_MPCwithMADS/ap_clk -into $clockgroup
set testbenchgroup [add_wave_group "Test Bench Signals"]
set tbinternalsiggroup [add_wave_group "Internal Signals" -into $testbenchgroup]
set tb_simstatus_group [add_wave_group "Simulation Status" -into $tbinternalsiggroup]
set tb_portdepth_group [add_wave_group "Port Depth" -into $tbinternalsiggroup]
add_wave /apatb_MPCwithMADS_top/AUTOTB_TRANSACTION_NUM -into $tb_simstatus_group -radix hex
add_wave /apatb_MPCwithMADS_top/ready_cnt -into $tb_simstatus_group -radix hex
add_wave /apatb_MPCwithMADS_top/done_cnt -into $tb_simstatus_group -radix hex
add_wave /apatb_MPCwithMADS_top/LENGTH_x_0 -into $tb_portdepth_group -radix hex
add_wave /apatb_MPCwithMADS_top/LENGTH_x_1 -into $tb_portdepth_group -radix hex
add_wave /apatb_MPCwithMADS_top/LENGTH_par_0 -into $tb_portdepth_group -radix hex
add_wave /apatb_MPCwithMADS_top/LENGTH_par_1 -into $tb_portdepth_group -radix hex
add_wave /apatb_MPCwithMADS_top/LENGTH_ref -into $tb_portdepth_group -radix hex
add_wave /apatb_MPCwithMADS_top/LENGTH_uOpt -into $tb_portdepth_group -radix hex
set tbcoutputgroup [add_wave_group "C Outputs" -into $testbenchgroup]
set tb_uOpt_group [add_wave_group uOpt(wire) -into $tbcoutputgroup]
add_wave /apatb_MPCwithMADS_top/uOpt_ap_vld -into $tb_uOpt_group -color #ffff00 -radix hex
add_wave /apatb_MPCwithMADS_top/uOpt -into $tb_uOpt_group -radix hex
set tbcinputgroup [add_wave_group "C Inputs" -into $testbenchgroup]
set tb_ref_group [add_wave_group ref(wire) -into $tbcinputgroup]
add_wave /apatb_MPCwithMADS_top/ref -into $tb_ref_group -radix hex
set tb_par_1_group [add_wave_group par_1(wire) -into $tbcinputgroup]
add_wave /apatb_MPCwithMADS_top/par_1 -into $tb_par_1_group -radix hex
set tb_par_0_group [add_wave_group par_0(wire) -into $tbcinputgroup]
add_wave /apatb_MPCwithMADS_top/par_0 -into $tb_par_0_group -radix hex
set tb_x_1_group [add_wave_group x_1(wire) -into $tbcinputgroup]
add_wave /apatb_MPCwithMADS_top/x_1 -into $tb_x_1_group -radix hex
set tb_x_0_group [add_wave_group x_0(wire) -into $tbcinputgroup]
add_wave /apatb_MPCwithMADS_top/x_0 -into $tb_x_0_group -radix hex
save_wave_config MPCwithMADS.wcfg
run all
quit

