#include <systemc>
#include <iostream>
#include <cstdlib>
#include <cstddef>
#include <stdint.h>
#include "SysCFileHandler.h"
#include "ap_int.h"
#include "ap_fixed.h"
#include <complex>
#include <stdbool.h>
#include "autopilot_cbe.h"
#include "hls_stream.h"
#include "hls_half.h"
#include "hls_signal_handler.h"

using namespace std;
using namespace sc_core;
using namespace sc_dt;

// wrapc file define:
#define AUTOTB_TVIN_x_0 "../tv/cdatafile/c.MPCwithMADS.autotvin_x_0.dat"
#define AUTOTB_TVOUT_x_0 "../tv/cdatafile/c.MPCwithMADS.autotvout_x_0.dat"
// wrapc file define:
#define AUTOTB_TVIN_x_1 "../tv/cdatafile/c.MPCwithMADS.autotvin_x_1.dat"
#define AUTOTB_TVOUT_x_1 "../tv/cdatafile/c.MPCwithMADS.autotvout_x_1.dat"
// wrapc file define:
#define AUTOTB_TVIN_par_0 "../tv/cdatafile/c.MPCwithMADS.autotvin_par_0.dat"
#define AUTOTB_TVOUT_par_0 "../tv/cdatafile/c.MPCwithMADS.autotvout_par_0.dat"
// wrapc file define:
#define AUTOTB_TVIN_par_1 "../tv/cdatafile/c.MPCwithMADS.autotvin_par_1.dat"
#define AUTOTB_TVOUT_par_1 "../tv/cdatafile/c.MPCwithMADS.autotvout_par_1.dat"
// wrapc file define:
#define AUTOTB_TVIN_ref "../tv/cdatafile/c.MPCwithMADS.autotvin_ref.dat"
#define AUTOTB_TVOUT_ref "../tv/cdatafile/c.MPCwithMADS.autotvout_ref.dat"
// wrapc file define:
#define AUTOTB_TVIN_uOpt "../tv/cdatafile/c.MPCwithMADS.autotvin_uOpt.dat"
#define AUTOTB_TVOUT_uOpt "../tv/cdatafile/c.MPCwithMADS.autotvout_uOpt.dat"

#define INTER_TCL "../tv/cdatafile/ref.tcl"

// tvout file define:
#define AUTOTB_TVOUT_PC_x_0 "../tv/rtldatafile/rtl.MPCwithMADS.autotvout_x_0.dat"
// tvout file define:
#define AUTOTB_TVOUT_PC_x_1 "../tv/rtldatafile/rtl.MPCwithMADS.autotvout_x_1.dat"
// tvout file define:
#define AUTOTB_TVOUT_PC_par_0 "../tv/rtldatafile/rtl.MPCwithMADS.autotvout_par_0.dat"
// tvout file define:
#define AUTOTB_TVOUT_PC_par_1 "../tv/rtldatafile/rtl.MPCwithMADS.autotvout_par_1.dat"
// tvout file define:
#define AUTOTB_TVOUT_PC_ref "../tv/rtldatafile/rtl.MPCwithMADS.autotvout_ref.dat"
// tvout file define:
#define AUTOTB_TVOUT_PC_uOpt "../tv/rtldatafile/rtl.MPCwithMADS.autotvout_uOpt.dat"

class INTER_TCL_FILE {
  public:
INTER_TCL_FILE(const char* name) {
  mName = name; 
  x_0_depth = 0;
  x_1_depth = 0;
  par_0_depth = 0;
  par_1_depth = 0;
  ref_depth = 0;
  uOpt_depth = 0;
  trans_num =0;
}
~INTER_TCL_FILE() {
  mFile.open(mName);
  if (!mFile.good()) {
    cout << "Failed to open file ref.tcl" << endl;
    exit (1); 
  }
  string total_list = get_depth_list();
  mFile << "set depth_list {\n";
  mFile << total_list;
  mFile << "}\n";
  mFile << "set trans_num "<<trans_num<<endl;
  mFile.close();
}
string get_depth_list () {
  stringstream total_list;
  total_list << "{x_0 " << x_0_depth << "}\n";
  total_list << "{x_1 " << x_1_depth << "}\n";
  total_list << "{par_0 " << par_0_depth << "}\n";
  total_list << "{par_1 " << par_1_depth << "}\n";
  total_list << "{ref " << ref_depth << "}\n";
  total_list << "{uOpt " << uOpt_depth << "}\n";
  return total_list.str();
}
void set_num (int num , int* class_num) {
  (*class_num) = (*class_num) > num ? (*class_num) : num;
}
void set_string(std::string list, std::string* class_list) {
  (*class_list) = list;
}
  public:
    int x_0_depth;
    int x_1_depth;
    int par_0_depth;
    int par_1_depth;
    int ref_depth;
    int uOpt_depth;
    int trans_num;
  private:
    ofstream mFile;
    const char* mName;
};

static void RTLOutputCheckAndReplacement(std::string &AESL_token, std::string PortName) {
  bool no_x = false;
  bool err = false;

  no_x = false;
  // search and replace 'X' with '0' from the 3rd char of token
  while (!no_x) {
    size_t x_found = AESL_token.find('X', 0);
    if (x_found != string::npos) {
      if (!err) { 
        cerr << "WARNING: [SIM 212-201] RTL produces unknown value 'X' on port" 
             << PortName << ", possible cause: There are uninitialized variables in the C design."
             << endl; 
        err = true;
      }
      AESL_token.replace(x_found, 1, "0");
    } else
      no_x = true;
  }
  no_x = false;
  // search and replace 'x' with '0' from the 3rd char of token
  while (!no_x) {
    size_t x_found = AESL_token.find('x', 2);
    if (x_found != string::npos) {
      if (!err) { 
        cerr << "WARNING: [SIM 212-201] RTL produces unknown value 'x' on port" 
             << PortName << ", possible cause: There are uninitialized variables in the C design."
             << endl; 
        err = true;
      }
      AESL_token.replace(x_found, 1, "0");
    } else
      no_x = true;
  }
}
extern "C" void MPCwithMADS_hw_stub_wrapper(volatile void *, volatile void *, volatile void *, volatile void *, volatile void *, volatile void *);

extern "C" void apatb_MPCwithMADS_hw(volatile void * __xlx_apatb_param_x_0, volatile void * __xlx_apatb_param_x_1, volatile void * __xlx_apatb_param_par_0, volatile void * __xlx_apatb_param_par_1, volatile void * __xlx_apatb_param_ref, volatile void * __xlx_apatb_param_uOpt) {
  refine_signal_handler();
  fstream wrapc_switch_file_token;
  wrapc_switch_file_token.open(".hls_cosim_wrapc_switch.log");
  int AESL_i;
  if (wrapc_switch_file_token.good())
  {

    CodeState = ENTER_WRAPC_PC;
    static unsigned AESL_transaction_pc = 0;
    string AESL_token;
    string AESL_num;{
      static ifstream rtl_tv_out_file;
      if (!rtl_tv_out_file.is_open()) {
        rtl_tv_out_file.open(AUTOTB_TVOUT_PC_uOpt);
        if (rtl_tv_out_file.good()) {
          rtl_tv_out_file >> AESL_token;
          if (AESL_token != "[[[runtime]]]")
            exit(1);
        }
      }
  
      if (rtl_tv_out_file.good()) {
        rtl_tv_out_file >> AESL_token; 
        rtl_tv_out_file >> AESL_num;  // transaction number
        if (AESL_token != "[[transaction]]") {
          cerr << "Unexpected token: " << AESL_token << endl;
          exit(1);
        }
        if (atoi(AESL_num.c_str()) == AESL_transaction_pc) {
          std::vector<sc_bv<12> > uOpt_pc_buffer(1);
          int i = 0;

          rtl_tv_out_file >> AESL_token; //data
          while (AESL_token != "[[/transaction]]"){

            RTLOutputCheckAndReplacement(AESL_token, "uOpt");
  
            // push token into output port buffer
            if (AESL_token != "") {
              uOpt_pc_buffer[i] = AESL_token.c_str();;
              i++;
            }
  
            rtl_tv_out_file >> AESL_token; //data or [[/transaction]]
            if (AESL_token == "[[[/runtime]]]" || rtl_tv_out_file.eof())
              exit(1);
          }
          if (i > 0) {
            ((short*)__xlx_apatb_param_uOpt)[0] = uOpt_pc_buffer[0].to_int64();
          }
        } // end transaction
      } // end file is good
    } // end post check logic bolck
  
    AESL_transaction_pc++;
    return ;
  }
static unsigned AESL_transaction;
static AESL_FILE_HANDLER aesl_fh;
static INTER_TCL_FILE tcl_file(INTER_TCL);
std::vector<char> __xlx_sprintf_buffer(1024);
CodeState = ENTER_WRAPC;
//x_0
aesl_fh.touch(AUTOTB_TVIN_x_0);
aesl_fh.touch(AUTOTB_TVOUT_x_0);
//x_1
aesl_fh.touch(AUTOTB_TVIN_x_1);
aesl_fh.touch(AUTOTB_TVOUT_x_1);
//par_0
aesl_fh.touch(AUTOTB_TVIN_par_0);
aesl_fh.touch(AUTOTB_TVOUT_par_0);
//par_1
aesl_fh.touch(AUTOTB_TVIN_par_1);
aesl_fh.touch(AUTOTB_TVOUT_par_1);
//ref
aesl_fh.touch(AUTOTB_TVIN_ref);
aesl_fh.touch(AUTOTB_TVOUT_ref);
//uOpt
aesl_fh.touch(AUTOTB_TVIN_uOpt);
aesl_fh.touch(AUTOTB_TVOUT_uOpt);
CodeState = DUMP_INPUTS;
// print x_0 Transactions
{
  sprintf(__xlx_sprintf_buffer.data(), "[[transaction]] %d\n", AESL_transaction);
  aesl_fh.write(AUTOTB_TVIN_x_0, __xlx_sprintf_buffer.data());
  {
    sc_bv<12> __xlx_tmp_lv = *((short*)__xlx_apatb_param_x_0);

    sprintf(__xlx_sprintf_buffer.data(), "%s\n", __xlx_tmp_lv.to_string(SC_HEX).c_str());
    aesl_fh.write(AUTOTB_TVIN_x_0, __xlx_sprintf_buffer.data()); 
  }
  tcl_file.set_num(1, &tcl_file.x_0_depth);
  sprintf(__xlx_sprintf_buffer.data(), "[[/transaction]] \n");
  aesl_fh.write(AUTOTB_TVIN_x_0, __xlx_sprintf_buffer.data());
}
// print x_1 Transactions
{
  sprintf(__xlx_sprintf_buffer.data(), "[[transaction]] %d\n", AESL_transaction);
  aesl_fh.write(AUTOTB_TVIN_x_1, __xlx_sprintf_buffer.data());
  {
    sc_bv<12> __xlx_tmp_lv = *((short*)__xlx_apatb_param_x_1);

    sprintf(__xlx_sprintf_buffer.data(), "%s\n", __xlx_tmp_lv.to_string(SC_HEX).c_str());
    aesl_fh.write(AUTOTB_TVIN_x_1, __xlx_sprintf_buffer.data()); 
  }
  tcl_file.set_num(1, &tcl_file.x_1_depth);
  sprintf(__xlx_sprintf_buffer.data(), "[[/transaction]] \n");
  aesl_fh.write(AUTOTB_TVIN_x_1, __xlx_sprintf_buffer.data());
}
// print par_0 Transactions
{
  sprintf(__xlx_sprintf_buffer.data(), "[[transaction]] %d\n", AESL_transaction);
  aesl_fh.write(AUTOTB_TVIN_par_0, __xlx_sprintf_buffer.data());
  {
    sc_bv<12> __xlx_tmp_lv = *((short*)__xlx_apatb_param_par_0);

    sprintf(__xlx_sprintf_buffer.data(), "%s\n", __xlx_tmp_lv.to_string(SC_HEX).c_str());
    aesl_fh.write(AUTOTB_TVIN_par_0, __xlx_sprintf_buffer.data()); 
  }
  tcl_file.set_num(1, &tcl_file.par_0_depth);
  sprintf(__xlx_sprintf_buffer.data(), "[[/transaction]] \n");
  aesl_fh.write(AUTOTB_TVIN_par_0, __xlx_sprintf_buffer.data());
}
// print par_1 Transactions
{
  sprintf(__xlx_sprintf_buffer.data(), "[[transaction]] %d\n", AESL_transaction);
  aesl_fh.write(AUTOTB_TVIN_par_1, __xlx_sprintf_buffer.data());
  {
    sc_bv<12> __xlx_tmp_lv = *((short*)__xlx_apatb_param_par_1);

    sprintf(__xlx_sprintf_buffer.data(), "%s\n", __xlx_tmp_lv.to_string(SC_HEX).c_str());
    aesl_fh.write(AUTOTB_TVIN_par_1, __xlx_sprintf_buffer.data()); 
  }
  tcl_file.set_num(1, &tcl_file.par_1_depth);
  sprintf(__xlx_sprintf_buffer.data(), "[[/transaction]] \n");
  aesl_fh.write(AUTOTB_TVIN_par_1, __xlx_sprintf_buffer.data());
}
// print ref Transactions
{
  sprintf(__xlx_sprintf_buffer.data(), "[[transaction]] %d\n", AESL_transaction);
  aesl_fh.write(AUTOTB_TVIN_ref, __xlx_sprintf_buffer.data());
  {
    sc_bv<12> __xlx_tmp_lv = *((short*)__xlx_apatb_param_ref);

    sprintf(__xlx_sprintf_buffer.data(), "%s\n", __xlx_tmp_lv.to_string(SC_HEX).c_str());
    aesl_fh.write(AUTOTB_TVIN_ref, __xlx_sprintf_buffer.data()); 
  }
  tcl_file.set_num(1, &tcl_file.ref_depth);
  sprintf(__xlx_sprintf_buffer.data(), "[[/transaction]] \n");
  aesl_fh.write(AUTOTB_TVIN_ref, __xlx_sprintf_buffer.data());
}
// print uOpt Transactions
{
  sprintf(__xlx_sprintf_buffer.data(), "[[transaction]] %d\n", AESL_transaction);
  aesl_fh.write(AUTOTB_TVIN_uOpt, __xlx_sprintf_buffer.data());
  {
    sc_bv<12> __xlx_tmp_lv = *((short*)__xlx_apatb_param_uOpt);

    sprintf(__xlx_sprintf_buffer.data(), "%s\n", __xlx_tmp_lv.to_string(SC_HEX).c_str());
    aesl_fh.write(AUTOTB_TVIN_uOpt, __xlx_sprintf_buffer.data()); 
  }
  tcl_file.set_num(1, &tcl_file.uOpt_depth);
  sprintf(__xlx_sprintf_buffer.data(), "[[/transaction]] \n");
  aesl_fh.write(AUTOTB_TVIN_uOpt, __xlx_sprintf_buffer.data());
}
CodeState = CALL_C_DUT;
MPCwithMADS_hw_stub_wrapper(__xlx_apatb_param_x_0, __xlx_apatb_param_x_1, __xlx_apatb_param_par_0, __xlx_apatb_param_par_1, __xlx_apatb_param_ref, __xlx_apatb_param_uOpt);
CodeState = DUMP_OUTPUTS;
// print uOpt Transactions
{
  sprintf(__xlx_sprintf_buffer.data(), "[[transaction]] %d\n", AESL_transaction);
  aesl_fh.write(AUTOTB_TVOUT_uOpt, __xlx_sprintf_buffer.data());
  {
    sc_bv<12> __xlx_tmp_lv = *((short*)__xlx_apatb_param_uOpt);

    sprintf(__xlx_sprintf_buffer.data(), "%s\n", __xlx_tmp_lv.to_string(SC_HEX).c_str());
    aesl_fh.write(AUTOTB_TVOUT_uOpt, __xlx_sprintf_buffer.data()); 
  }
  tcl_file.set_num(1, &tcl_file.uOpt_depth);
  sprintf(__xlx_sprintf_buffer.data(), "[[/transaction]] \n");
  aesl_fh.write(AUTOTB_TVOUT_uOpt, __xlx_sprintf_buffer.data());
}
CodeState = DELETE_CHAR_BUFFERS;
AESL_transaction++;
tcl_file.set_num(AESL_transaction , &tcl_file.trans_num);
}
