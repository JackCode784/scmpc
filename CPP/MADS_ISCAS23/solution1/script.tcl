############################################################
## This file is generated automatically by Vitis HLS.
## Please DO NOT edit it.
## Copyright 1986-2022 Xilinx, Inc. All Rights Reserved.
## Copyright 2022-2023 Advanced Micro Devices, Inc. All Rights Reserved.
############################################################
open_project MPCwithMADS_fxd
set_top MPCwithMADS
add_files MPCwithMADS_fxd/shiftOpt.cpp
add_files MPCwithMADS_fxd/pseudoRand.cpp
add_files MPCwithMADS_fxd/progressiveBarrierPolling.cpp
add_files MPCwithMADS_fxd/myLib.h
add_files MPCwithMADS_fxd/generatePollMatrix.cpp
add_files MPCwithMADS_fxd/generatePollDirections.cpp
add_files MPCwithMADS_fxd/costFunction.cpp
add_files MPCwithMADS_fxd/boostParams.h
add_files MPCwithMADS_fxd/MPCwithMADS.cpp
add_files MPCwithMADS_fxd/MADS.cpp
add_files -tb MPCwithMADS_fxd/testMain.cpp -cflags "-Wno-unknown-pragmas" -csimflags "-Wno-unknown-pragmas"
open_solution "solution1" -flow_target vivado
set_part {xc7z020-clg484-1}
create_clock -period 10 -name default
source "./MPCwithMADS_fxd/solution1/directives.tcl"
csim_design
csynth_design
cosim_design -trace_level all -rtl vhdl
export_design -format ip_catalog
