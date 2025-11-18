set moduleName MPCwithMADS
set isTopModule 1
set isCombinational 0
set isDatapathOnly 0
set isPipelined 0
set pipeline_type none
set FunctionProtocol ap_ctrl_hs
set isOneStateSeq 0
set ProfileFlag 0
set StallSigGenFlag 0
set isEnableWaveformDebug 1
set C_modelName {MPCwithMADS}
set C_modelType { void 0 }
set C_modelArgList {
	{ x_0 int 12 regular {pointer 0}  }
	{ x_1 int 12 regular {pointer 0}  }
	{ par_0 int 12 regular {pointer 0}  }
	{ par_1 int 12 regular {pointer 0}  }
	{ ref int 12 regular {pointer 0}  }
	{ uOpt int 12 regular {pointer 1}  }
}
set C_modelArgMapList {[ 
	{ "Name" : "x_0", "interface" : "wire", "bitwidth" : 12, "direction" : "READONLY", "bitSlice":[{"low":0,"up":0,"cElement": [{"cName": "x_0","cData": "int12","bit_use": { "low": 0,"up": 0},"cArray": [{"low" : 0,"up" : 0,"step" : 0}]}]}]} , 
 	{ "Name" : "x_1", "interface" : "wire", "bitwidth" : 12, "direction" : "READONLY", "bitSlice":[{"low":0,"up":0,"cElement": [{"cName": "x_1","cData": "int12","bit_use": { "low": 0,"up": 0},"cArray": [{"low" : 0,"up" : 0,"step" : 0}]}]}]} , 
 	{ "Name" : "par_0", "interface" : "wire", "bitwidth" : 12, "direction" : "READONLY", "bitSlice":[{"low":0,"up":0,"cElement": [{"cName": "par_0","cData": "int12","bit_use": { "low": 0,"up": 0},"cArray": [{"low" : 0,"up" : 0,"step" : 0}]}]}]} , 
 	{ "Name" : "par_1", "interface" : "wire", "bitwidth" : 12, "direction" : "READONLY", "bitSlice":[{"low":0,"up":0,"cElement": [{"cName": "par_1","cData": "int12","bit_use": { "low": 0,"up": 0},"cArray": [{"low" : 0,"up" : 0,"step" : 0}]}]}]} , 
 	{ "Name" : "ref", "interface" : "wire", "bitwidth" : 12, "direction" : "READONLY", "bitSlice":[{"low":0,"up":0,"cElement": [{"cName": "ref","cData": "int12","bit_use": { "low": 0,"up": 0},"cArray": [{"low" : 0,"up" : 0,"step" : 0}]}]}]} , 
 	{ "Name" : "uOpt", "interface" : "wire", "bitwidth" : 12, "direction" : "WRITEONLY", "bitSlice":[{"low":0,"up":0,"cElement": [{"cName": "uOpt","cData": "int12","bit_use": { "low": 0,"up": 0},"cArray": [{"low" : 0,"up" : 0,"step" : 0}]}]}]} ]}
# RTL Port declarations: 
set portNum 13
set portList { 
	{ ap_clk sc_in sc_logic 1 clock -1 } 
	{ ap_rst sc_in sc_logic 1 reset -1 active_high_sync } 
	{ ap_start sc_in sc_logic 1 start -1 } 
	{ ap_done sc_out sc_logic 1 predone -1 } 
	{ ap_idle sc_out sc_logic 1 done -1 } 
	{ ap_ready sc_out sc_logic 1 ready -1 } 
	{ x_0 sc_in sc_lv 12 signal 0 } 
	{ x_1 sc_in sc_lv 12 signal 1 } 
	{ par_0 sc_in sc_lv 12 signal 2 } 
	{ par_1 sc_in sc_lv 12 signal 3 } 
	{ ref sc_in sc_lv 12 signal 4 } 
	{ uOpt sc_out sc_lv 12 signal 5 } 
	{ uOpt_ap_vld sc_out sc_logic 1 outvld 5 } 
}
set NewPortList {[ 
	{ "name": "ap_clk", "direction": "in", "datatype": "sc_logic", "bitwidth":1, "type": "clock", "bundle":{"name": "ap_clk", "role": "default" }} , 
 	{ "name": "ap_rst", "direction": "in", "datatype": "sc_logic", "bitwidth":1, "type": "reset", "bundle":{"name": "ap_rst", "role": "default" }} , 
 	{ "name": "ap_start", "direction": "in", "datatype": "sc_logic", "bitwidth":1, "type": "start", "bundle":{"name": "ap_start", "role": "default" }} , 
 	{ "name": "ap_done", "direction": "out", "datatype": "sc_logic", "bitwidth":1, "type": "predone", "bundle":{"name": "ap_done", "role": "default" }} , 
 	{ "name": "ap_idle", "direction": "out", "datatype": "sc_logic", "bitwidth":1, "type": "done", "bundle":{"name": "ap_idle", "role": "default" }} , 
 	{ "name": "ap_ready", "direction": "out", "datatype": "sc_logic", "bitwidth":1, "type": "ready", "bundle":{"name": "ap_ready", "role": "default" }} , 
 	{ "name": "x_0", "direction": "in", "datatype": "sc_lv", "bitwidth":12, "type": "signal", "bundle":{"name": "x_0", "role": "default" }} , 
 	{ "name": "x_1", "direction": "in", "datatype": "sc_lv", "bitwidth":12, "type": "signal", "bundle":{"name": "x_1", "role": "default" }} , 
 	{ "name": "par_0", "direction": "in", "datatype": "sc_lv", "bitwidth":12, "type": "signal", "bundle":{"name": "par_0", "role": "default" }} , 
 	{ "name": "par_1", "direction": "in", "datatype": "sc_lv", "bitwidth":12, "type": "signal", "bundle":{"name": "par_1", "role": "default" }} , 
 	{ "name": "ref", "direction": "in", "datatype": "sc_lv", "bitwidth":12, "type": "signal", "bundle":{"name": "ref", "role": "default" }} , 
 	{ "name": "uOpt", "direction": "out", "datatype": "sc_lv", "bitwidth":12, "type": "signal", "bundle":{"name": "uOpt", "role": "default" }} , 
 	{ "name": "uOpt_ap_vld", "direction": "out", "datatype": "sc_logic", "bitwidth":1, "type": "outvld", "bundle":{"name": "uOpt", "role": "ap_vld" }}  ]}

set RtlHierarchyInfo {[
	{"ID" : "0", "Level" : "0", "Path" : "`AUTOTB_DUT_INST", "Parent" : "", "Child" : ["1", "2", "54"],
		"CDFG" : "MPCwithMADS",
		"Protocol" : "ap_ctrl_hs",
		"ControlExist" : "1", "ap_start" : "1", "ap_ready" : "1", "ap_done" : "1", "ap_continue" : "0", "ap_idle" : "1", "real_start" : "0",
		"Pipeline" : "None", "UnalignedPipeline" : "0", "RewindPipeline" : "0", "ProcessNetwork" : "0",
		"II" : "0",
		"VariableLatency" : "1", "ExactLatency" : "-1", "EstimateLatencyMin" : "1488", "EstimateLatencyMax" : "1498",
		"Combinational" : "0",
		"Datapath" : "0",
		"ClockEnable" : "0",
		"HasSubDataflow" : "0",
		"InDataflowNetwork" : "0",
		"HasNonBlockingOperation" : "0",
		"Port" : [
			{"Name" : "x_0", "Type" : "None", "Direction" : "I"},
			{"Name" : "x_1", "Type" : "None", "Direction" : "I"},
			{"Name" : "par_0", "Type" : "None", "Direction" : "I"},
			{"Name" : "par_1", "Type" : "None", "Direction" : "I"},
			{"Name" : "ref", "Type" : "None", "Direction" : "I"},
			{"Name" : "uOpt", "Type" : "Vld", "Direction" : "O"},
			{"Name" : "firstCall", "Type" : "OVld", "Direction" : "IO"},
			{"Name" : "optimum_V_0", "Type" : "OVld", "Direction" : "IO",
				"SubConnect" : [
					{"ID" : "54", "SubInstance" : "grp_generatePollMatrix_fu_341", "Port" : "optimum_V_0"}]},
			{"Name" : "optimum_V_1", "Type" : "OVld", "Direction" : "IO",
				"SubConnect" : [
					{"ID" : "54", "SubInstance" : "grp_generatePollMatrix_fu_341", "Port" : "optimum_V_1"}]},
			{"Name" : "R1_V", "Type" : "OVld", "Direction" : "IO",
				"SubConnect" : [
					{"ID" : "54", "SubInstance" : "grp_generatePollMatrix_fu_341", "Port" : "R1_V"}]},
			{"Name" : "R2_V", "Type" : "OVld", "Direction" : "IO",
				"SubConnect" : [
					{"ID" : "54", "SubInstance" : "grp_generatePollMatrix_fu_341", "Port" : "R2_V"}]},
			{"Name" : "success", "Type" : "OVld", "Direction" : "IO"}]},
	{"ID" : "1", "Level" : "1", "Path" : "`AUTOTB_DUT_INST.pollMatrix_i_U", "Parent" : "0"},
	{"ID" : "2", "Level" : "1", "Path" : "`AUTOTB_DUT_INST.grp_costFunction_fu_326", "Parent" : "0", "Child" : ["3", "4", "5", "6", "7", "8", "9", "10", "11", "12", "13", "14", "15", "16", "17", "18", "19", "20", "21", "22", "23", "24", "25", "26", "27", "28", "29", "30", "31", "32", "33", "34", "35", "36", "37", "38", "39", "40", "41", "42", "43", "44", "45", "46", "47", "48", "49", "50", "51", "52", "53"],
		"CDFG" : "costFunction",
		"Protocol" : "ap_ctrl_hs",
		"ControlExist" : "0", "ap_start" : "0", "ap_ready" : "0", "ap_done" : "0", "ap_continue" : "0", "ap_idle" : "0", "real_start" : "0",
		"Pipeline" : "Aligned", "UnalignedPipeline" : "0", "RewindPipeline" : "0", "ProcessNetwork" : "0",
		"II" : "1",
		"VariableLatency" : "0", "ExactLatency" : "70", "EstimateLatencyMin" : "70", "EstimateLatencyMax" : "70",
		"Combinational" : "0",
		"Datapath" : "1",
		"ClockEnable" : "1",
		"HasSubDataflow" : "0",
		"InDataflowNetwork" : "0",
		"HasNonBlockingOperation" : "0",
		"Port" : [
			{"Name" : "p_read2", "Type" : "None", "Direction" : "I"},
			{"Name" : "p_read7", "Type" : "None", "Direction" : "I"},
			{"Name" : "p_read8", "Type" : "None", "Direction" : "I"},
			{"Name" : "p_read9", "Type" : "None", "Direction" : "I"},
			{"Name" : "p_read10", "Type" : "None", "Direction" : "I"},
			{"Name" : "p_read11", "Type" : "None", "Direction" : "I"},
			{"Name" : "p_read12", "Type" : "None", "Direction" : "I"}]},
	{"ID" : "3", "Level" : "2", "Path" : "`AUTOTB_DUT_INST.grp_costFunction_fu_326.sdiv_42s_18s_42_46_0_U1", "Parent" : "2"},
	{"ID" : "4", "Level" : "2", "Path" : "`AUTOTB_DUT_INST.grp_costFunction_fu_326.sdiv_47ns_18s_47_51_0_U2", "Parent" : "2"},
	{"ID" : "5", "Level" : "2", "Path" : "`AUTOTB_DUT_INST.grp_costFunction_fu_326.sdiv_29ns_18s_13_33_0_U3", "Parent" : "2"},
	{"ID" : "6", "Level" : "2", "Path" : "`AUTOTB_DUT_INST.grp_costFunction_fu_326.mul_34s_15ns_44_2_0_U4", "Parent" : "2"},
	{"ID" : "7", "Level" : "2", "Path" : "`AUTOTB_DUT_INST.grp_costFunction_fu_326.mul_36s_11ns_47_2_0_U5", "Parent" : "2"},
	{"ID" : "8", "Level" : "2", "Path" : "`AUTOTB_DUT_INST.grp_costFunction_fu_326.mul_36s_11ns_47_2_0_U6", "Parent" : "2"},
	{"ID" : "9", "Level" : "2", "Path" : "`AUTOTB_DUT_INST.grp_costFunction_fu_326.mul_mul_18s_18s_29_4_0_U7", "Parent" : "2"},
	{"ID" : "10", "Level" : "2", "Path" : "`AUTOTB_DUT_INST.grp_costFunction_fu_326.mul_mul_18s_18s_36_4_0_U8", "Parent" : "2"},
	{"ID" : "11", "Level" : "2", "Path" : "`AUTOTB_DUT_INST.grp_costFunction_fu_326.mul_mul_18s_16ns_34_4_0_U9", "Parent" : "2"},
	{"ID" : "12", "Level" : "2", "Path" : "`AUTOTB_DUT_INST.grp_costFunction_fu_326.mul_mul_18s_18s_34_4_0_U10", "Parent" : "2"},
	{"ID" : "13", "Level" : "2", "Path" : "`AUTOTB_DUT_INST.grp_costFunction_fu_326.mul_mul_18s_18s_34_4_0_U11", "Parent" : "2"},
	{"ID" : "14", "Level" : "2", "Path" : "`AUTOTB_DUT_INST.grp_costFunction_fu_326.mul_mul_18s_18s_34_4_0_U12", "Parent" : "2"},
	{"ID" : "15", "Level" : "2", "Path" : "`AUTOTB_DUT_INST.grp_costFunction_fu_326.mul_mul_18s_14s_32_4_0_U13", "Parent" : "2"},
	{"ID" : "16", "Level" : "2", "Path" : "`AUTOTB_DUT_INST.grp_costFunction_fu_326.mac_muladd_18s_18s_33ns_34_4_0_U14", "Parent" : "2"},
	{"ID" : "17", "Level" : "2", "Path" : "`AUTOTB_DUT_INST.grp_costFunction_fu_326.mac_muladd_18s_18s_34s_34_4_0_U15", "Parent" : "2"},
	{"ID" : "18", "Level" : "2", "Path" : "`AUTOTB_DUT_INST.grp_costFunction_fu_326.mul_mul_18s_18s_34_4_0_U16", "Parent" : "2"},
	{"ID" : "19", "Level" : "2", "Path" : "`AUTOTB_DUT_INST.grp_costFunction_fu_326.mac_muladd_18s_18s_33ns_34_4_0_U17", "Parent" : "2"},
	{"ID" : "20", "Level" : "2", "Path" : "`AUTOTB_DUT_INST.grp_costFunction_fu_326.mul_mul_18s_18s_34_4_0_U18", "Parent" : "2"},
	{"ID" : "21", "Level" : "2", "Path" : "`AUTOTB_DUT_INST.grp_costFunction_fu_326.mac_muladd_18s_18s_33ns_34_4_0_U19", "Parent" : "2"},
	{"ID" : "22", "Level" : "2", "Path" : "`AUTOTB_DUT_INST.grp_costFunction_fu_326.mul_mul_18s_18s_34_4_0_U20", "Parent" : "2"},
	{"ID" : "23", "Level" : "2", "Path" : "`AUTOTB_DUT_INST.grp_costFunction_fu_326.mac_muladd_18s_18s_34s_34_4_0_U21", "Parent" : "2"},
	{"ID" : "24", "Level" : "2", "Path" : "`AUTOTB_DUT_INST.grp_costFunction_fu_326.mac_muladd_18s_18s_33ns_34_4_0_U22", "Parent" : "2"},
	{"ID" : "25", "Level" : "2", "Path" : "`AUTOTB_DUT_INST.grp_costFunction_fu_326.mul_mul_18s_18s_34_4_0_U23", "Parent" : "2"},
	{"ID" : "26", "Level" : "2", "Path" : "`AUTOTB_DUT_INST.grp_costFunction_fu_326.mul_mul_18s_13s_31_4_0_U24", "Parent" : "2"},
	{"ID" : "27", "Level" : "2", "Path" : "`AUTOTB_DUT_INST.grp_costFunction_fu_326.mul_mul_18s_16s_34_4_0_U25", "Parent" : "2"},
	{"ID" : "28", "Level" : "2", "Path" : "`AUTOTB_DUT_INST.grp_costFunction_fu_326.mac_muladd_18s_18s_34ns_34_4_0_U26", "Parent" : "2"},
	{"ID" : "29", "Level" : "2", "Path" : "`AUTOTB_DUT_INST.grp_costFunction_fu_326.mac_muladd_18s_18s_34s_34_4_0_U27", "Parent" : "2"},
	{"ID" : "30", "Level" : "2", "Path" : "`AUTOTB_DUT_INST.grp_costFunction_fu_326.mul_mul_18s_18s_34_4_0_U28", "Parent" : "2"},
	{"ID" : "31", "Level" : "2", "Path" : "`AUTOTB_DUT_INST.grp_costFunction_fu_326.mul_mul_18s_13s_31_4_0_U29", "Parent" : "2"},
	{"ID" : "32", "Level" : "2", "Path" : "`AUTOTB_DUT_INST.grp_costFunction_fu_326.mul_mul_18s_16s_34_4_0_U30", "Parent" : "2"},
	{"ID" : "33", "Level" : "2", "Path" : "`AUTOTB_DUT_INST.grp_costFunction_fu_326.mac_muladd_18s_18s_34ns_34_4_0_U31", "Parent" : "2"},
	{"ID" : "34", "Level" : "2", "Path" : "`AUTOTB_DUT_INST.grp_costFunction_fu_326.mac_muladd_18s_18s_34s_34_4_0_U32", "Parent" : "2"},
	{"ID" : "35", "Level" : "2", "Path" : "`AUTOTB_DUT_INST.grp_costFunction_fu_326.mul_mul_18s_18s_34_4_0_U33", "Parent" : "2"},
	{"ID" : "36", "Level" : "2", "Path" : "`AUTOTB_DUT_INST.grp_costFunction_fu_326.mac_muladd_18s_14s_34s_34_4_0_U34", "Parent" : "2"},
	{"ID" : "37", "Level" : "2", "Path" : "`AUTOTB_DUT_INST.grp_costFunction_fu_326.mul_mul_18s_18s_33_4_0_U35", "Parent" : "2"},
	{"ID" : "38", "Level" : "2", "Path" : "`AUTOTB_DUT_INST.grp_costFunction_fu_326.mac_muladd_18s_14s_34s_34_4_0_U36", "Parent" : "2"},
	{"ID" : "39", "Level" : "2", "Path" : "`AUTOTB_DUT_INST.grp_costFunction_fu_326.mul_mul_18s_18s_33_4_0_U37", "Parent" : "2"},
	{"ID" : "40", "Level" : "2", "Path" : "`AUTOTB_DUT_INST.grp_costFunction_fu_326.mul_mul_18s_18s_33_4_0_U38", "Parent" : "2"},
	{"ID" : "41", "Level" : "2", "Path" : "`AUTOTB_DUT_INST.grp_costFunction_fu_326.mac_muladd_16ns_18s_33s_33_4_0_U39", "Parent" : "2"},
	{"ID" : "42", "Level" : "2", "Path" : "`AUTOTB_DUT_INST.grp_costFunction_fu_326.mac_muladd_18s_18s_33s_33_4_0_U40", "Parent" : "2"},
	{"ID" : "43", "Level" : "2", "Path" : "`AUTOTB_DUT_INST.grp_costFunction_fu_326.mul_mul_18s_18s_36_4_0_U41", "Parent" : "2"},
	{"ID" : "44", "Level" : "2", "Path" : "`AUTOTB_DUT_INST.grp_costFunction_fu_326.mul_mul_18s_18s_33_4_0_U42", "Parent" : "2"},
	{"ID" : "45", "Level" : "2", "Path" : "`AUTOTB_DUT_INST.grp_costFunction_fu_326.mul_mul_18s_18s_31_4_0_U43", "Parent" : "2"},
	{"ID" : "46", "Level" : "2", "Path" : "`AUTOTB_DUT_INST.grp_costFunction_fu_326.ama_addmuladd_21s_19s_18s_32ns_32_4_0_U44", "Parent" : "2"},
	{"ID" : "47", "Level" : "2", "Path" : "`AUTOTB_DUT_INST.grp_costFunction_fu_326.mul_mul_18s_18s_33_4_0_U45", "Parent" : "2"},
	{"ID" : "48", "Level" : "2", "Path" : "`AUTOTB_DUT_INST.grp_costFunction_fu_326.mul_mul_18s_18s_33_4_0_U46", "Parent" : "2"},
	{"ID" : "49", "Level" : "2", "Path" : "`AUTOTB_DUT_INST.grp_costFunction_fu_326.mac_muladd_16ns_18s_33s_33_4_0_U47", "Parent" : "2"},
	{"ID" : "50", "Level" : "2", "Path" : "`AUTOTB_DUT_INST.grp_costFunction_fu_326.mac_muladd_18s_18s_33s_33_4_0_U48", "Parent" : "2"},
	{"ID" : "51", "Level" : "2", "Path" : "`AUTOTB_DUT_INST.grp_costFunction_fu_326.mul_mul_18s_18s_36_4_0_U49", "Parent" : "2"},
	{"ID" : "52", "Level" : "2", "Path" : "`AUTOTB_DUT_INST.grp_costFunction_fu_326.mul_mul_18s_18s_31_4_0_U50", "Parent" : "2"},
	{"ID" : "53", "Level" : "2", "Path" : "`AUTOTB_DUT_INST.grp_costFunction_fu_326.ama_addmuladd_21s_19s_18s_32ns_32_4_0_U51", "Parent" : "2"},
	{"ID" : "54", "Level" : "1", "Path" : "`AUTOTB_DUT_INST.grp_generatePollMatrix_fu_341", "Parent" : "0", "Child" : ["55", "56", "60", "61", "62"],
		"CDFG" : "generatePollMatrix",
		"Protocol" : "ap_ctrl_hs",
		"ControlExist" : "1", "ap_start" : "1", "ap_ready" : "1", "ap_done" : "1", "ap_continue" : "0", "ap_idle" : "1", "real_start" : "0",
		"Pipeline" : "None", "UnalignedPipeline" : "0", "RewindPipeline" : "0", "ProcessNetwork" : "0",
		"II" : "0",
		"VariableLatency" : "1", "ExactLatency" : "-1", "EstimateLatencyMin" : "117", "EstimateLatencyMax" : "117",
		"Combinational" : "0",
		"Datapath" : "0",
		"ClockEnable" : "0",
		"HasSubDataflow" : "0",
		"InDataflowNetwork" : "0",
		"HasNonBlockingOperation" : "0",
		"Port" : [
			{"Name" : "p_read", "Type" : "None", "Direction" : "I"},
			{"Name" : "p_read1", "Type" : "None", "Direction" : "I"},
			{"Name" : "p_read2", "Type" : "None", "Direction" : "I"},
			{"Name" : "p_read3", "Type" : "None", "Direction" : "I"},
			{"Name" : "pollMatrix", "Type" : "Memory", "Direction" : "O"},
			{"Name" : "optimum_V_0", "Type" : "None", "Direction" : "I"},
			{"Name" : "optimum_V_1", "Type" : "None", "Direction" : "I"},
			{"Name" : "R1_V", "Type" : "OVld", "Direction" : "IO"},
			{"Name" : "R2_V", "Type" : "OVld", "Direction" : "IO"}]},
	{"ID" : "55", "Level" : "2", "Path" : "`AUTOTB_DUT_INST.grp_generatePollMatrix_fu_341.directions_U", "Parent" : "54"},
	{"ID" : "56", "Level" : "2", "Path" : "`AUTOTB_DUT_INST.grp_generatePollMatrix_fu_341.grp_generatePollDirections_fu_256", "Parent" : "54", "Child" : ["57", "58", "59"],
		"CDFG" : "generatePollDirections",
		"Protocol" : "ap_ctrl_hs",
		"ControlExist" : "1", "ap_start" : "1", "ap_ready" : "1", "ap_done" : "1", "ap_continue" : "0", "ap_idle" : "1", "real_start" : "0",
		"Pipeline" : "None", "UnalignedPipeline" : "0", "RewindPipeline" : "0", "ProcessNetwork" : "0",
		"II" : "0",
		"VariableLatency" : "1", "ExactLatency" : "-1", "EstimateLatencyMin" : "68", "EstimateLatencyMax" : "68",
		"Combinational" : "0",
		"Datapath" : "0",
		"ClockEnable" : "0",
		"HasSubDataflow" : "0",
		"InDataflowNetwork" : "0",
		"HasNonBlockingOperation" : "0",
		"Port" : [
			{"Name" : "p_read", "Type" : "None", "Direction" : "I"},
			{"Name" : "p_read1", "Type" : "None", "Direction" : "I"},
			{"Name" : "p_read2", "Type" : "None", "Direction" : "I"},
			{"Name" : "p_read3", "Type" : "None", "Direction" : "I"},
			{"Name" : "p_read4", "Type" : "None", "Direction" : "I"},
			{"Name" : "p_read5", "Type" : "None", "Direction" : "I"},
			{"Name" : "directions", "Type" : "Memory", "Direction" : "O"}]},
	{"ID" : "57", "Level" : "3", "Path" : "`AUTOTB_DUT_INST.grp_generatePollMatrix_fu_341.grp_generatePollDirections_fu_256.grp_round_fixed_54_15_s_fu_318", "Parent" : "56",
		"CDFG" : "round_fixed_54_15_s",
		"Protocol" : "ap_ctrl_hs",
		"ControlExist" : "0", "ap_start" : "0", "ap_ready" : "0", "ap_done" : "0", "ap_continue" : "0", "ap_idle" : "0", "real_start" : "0",
		"Pipeline" : "Aligned", "UnalignedPipeline" : "0", "RewindPipeline" : "0", "ProcessNetwork" : "0",
		"II" : "1",
		"VariableLatency" : "0", "ExactLatency" : "1", "EstimateLatencyMin" : "1", "EstimateLatencyMax" : "1",
		"Combinational" : "0",
		"Datapath" : "1",
		"ClockEnable" : "0",
		"HasSubDataflow" : "0",
		"InDataflowNetwork" : "0",
		"HasNonBlockingOperation" : "0",
		"Port" : [
			{"Name" : "x", "Type" : "None", "Direction" : "I"}]},
	{"ID" : "58", "Level" : "3", "Path" : "`AUTOTB_DUT_INST.grp_generatePollMatrix_fu_341.grp_generatePollDirections_fu_256.mul_36ns_16s_51_2_1_U81", "Parent" : "56"},
	{"ID" : "59", "Level" : "3", "Path" : "`AUTOTB_DUT_INST.grp_generatePollMatrix_fu_341.grp_generatePollDirections_fu_256.mul_mul_20s_18s_33_4_1_U82", "Parent" : "56"},
	{"ID" : "60", "Level" : "2", "Path" : "`AUTOTB_DUT_INST.grp_generatePollMatrix_fu_341.mul_26s_12s_26_1_1_U92", "Parent" : "54"},
	{"ID" : "61", "Level" : "2", "Path" : "`AUTOTB_DUT_INST.grp_generatePollMatrix_fu_341.mul_mul_20s_6ns_20_4_1_U93", "Parent" : "54"},
	{"ID" : "62", "Level" : "2", "Path" : "`AUTOTB_DUT_INST.grp_generatePollMatrix_fu_341.mul_mul_20s_6ns_20_4_1_U94", "Parent" : "54"}]}


set ArgLastReadFirstWriteLatency {
	MPCwithMADS {
		x_0 {Type I LastRead 0 FirstWrite -1}
		x_1 {Type I LastRead 0 FirstWrite -1}
		par_0 {Type I LastRead 2 FirstWrite -1}
		par_1 {Type I LastRead 2 FirstWrite -1}
		ref {Type I LastRead 4 FirstWrite -1}
		uOpt {Type O LastRead -1 FirstWrite 78}
		firstCall {Type IO LastRead -1 FirstWrite -1}
		optimum_V_0 {Type IO LastRead -1 FirstWrite -1}
		optimum_V_1 {Type IO LastRead -1 FirstWrite -1}
		R1_V {Type IO LastRead -1 FirstWrite -1}
		R2_V {Type IO LastRead -1 FirstWrite -1}
		success {Type IO LastRead -1 FirstWrite -1}}
	costFunction {
		p_read2 {Type I LastRead 0 FirstWrite -1}
		p_read7 {Type I LastRead 0 FirstWrite -1}
		p_read8 {Type I LastRead 0 FirstWrite -1}
		p_read9 {Type I LastRead 0 FirstWrite -1}
		p_read10 {Type I LastRead 0 FirstWrite -1}
		p_read11 {Type I LastRead 0 FirstWrite -1}
		p_read12 {Type I LastRead 0 FirstWrite -1}}
	generatePollMatrix {
		p_read {Type I LastRead 0 FirstWrite -1}
		p_read1 {Type I LastRead 0 FirstWrite -1}
		p_read2 {Type I LastRead 0 FirstWrite -1}
		p_read3 {Type I LastRead 0 FirstWrite -1}
		pollMatrix {Type O LastRead -1 FirstWrite 11}
		optimum_V_0 {Type I LastRead 0 FirstWrite -1}
		optimum_V_1 {Type I LastRead 0 FirstWrite -1}
		R1_V {Type IO LastRead -1 FirstWrite -1}
		R2_V {Type IO LastRead -1 FirstWrite -1}}
	generatePollDirections {
		p_read {Type I LastRead 0 FirstWrite -1}
		p_read1 {Type I LastRead 0 FirstWrite -1}
		p_read2 {Type I LastRead 0 FirstWrite -1}
		p_read3 {Type I LastRead 0 FirstWrite -1}
		p_read4 {Type I LastRead 0 FirstWrite -1}
		p_read5 {Type I LastRead 0 FirstWrite -1}
		directions {Type O LastRead -1 FirstWrite 5}}
	round_fixed_54_15_s {
		x {Type I LastRead 0 FirstWrite -1}}}

set hasDtUnsupportedChannel 0

set PerformanceInfo {[
	{"Name" : "Latency", "Min" : "1488", "Max" : "1498"}
	, {"Name" : "Interval", "Min" : "1489", "Max" : "1499"}
]}

set PipelineEnableSignalInfo {[
	{"Pipeline" : "4", "EnableSignal" : "ap_enable_pp4"}
]}

set Spec2ImplPortList { 
	x_0 { ap_none {  { x_0 in_data 0 12 } } }
	x_1 { ap_none {  { x_1 in_data 0 12 } } }
	par_0 { ap_none {  { par_0 in_data 0 12 } } }
	par_1 { ap_none {  { par_1 in_data 0 12 } } }
	ref { ap_none {  { ref in_data 0 12 } } }
	uOpt { ap_vld {  { uOpt out_data 1 12 }  { uOpt_ap_vld out_vld 1 1 } } }
}

set busDeadlockParameterList { 
}

# RTL port scheduling information:
set fifoSchedulingInfoList { 
}

# RTL bus port read request latency information:
set busReadReqLatencyList { 
}

# RTL bus port write response latency information:
set busWriteResLatencyList { 
}

# RTL array port load latency information:
set memoryLoadLatencyList { 
}
