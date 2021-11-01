/* Include files */

#include <stddef.h>
#include "blas.h"
#include "mirageTrackingRobot_sfun.h"
#include "c2_mirageTrackingRobot.h"
#include "mwmathutil.h"
#define CHARTINSTANCE_CHARTNUMBER      (chartInstance->chartNumber)
#define CHARTINSTANCE_INSTANCENUMBER   (chartInstance->instanceNumber)
#include "mirageTrackingRobot_sfun_debug_macros.h"
#define _SF_MEX_LISTEN_FOR_CTRL_C(S)   sf_mex_listen_for_ctrl_c_with_debugger(S, sfGlobalDebugInstanceStruct);

static void chart_debug_initialization(SimStruct *S, unsigned int
  fullDebuggerInitialization);
static void chart_debug_initialize_data_addresses(SimStruct *S);

/* Type Definitions */

/* Named Constants */
#define CALL_EVENT                     (-1)

/* Variable Declarations */

/* Variable Definitions */
static real_T _sfTime_;
static const char * c2_debug_family_names[22] = { "m", "n", "c", "l_gray",
  "r_gray", "l_rgb1", "r_rgb1", "l_rgb2", "r_rgb2", "l_bw", "r_bw", "l_center",
  "r_center", "ml", "i", "mr", "nargin", "nargout", "l_rgb", "r_rgb", "L_p",
  "R_p" };

static const char * c2_b_debug_family_names[7] = { "minL", "imgTmp", "maxL",
  "imgIn", "nargin", "nargout", "imgOut" };

static const char * c2_c_debug_family_names[10] = { "indx", "indy", "ind4",
  "ind3", "ind1", "ind2", "nargin", "nargout", "p", "new_p" };

/* Function Declarations */
static void initialize_c2_mirageTrackingRobot
  (SFc2_mirageTrackingRobotInstanceStruct *chartInstance);
static void initialize_params_c2_mirageTrackingRobot
  (SFc2_mirageTrackingRobotInstanceStruct *chartInstance);
static void enable_c2_mirageTrackingRobot(SFc2_mirageTrackingRobotInstanceStruct
  *chartInstance);
static void disable_c2_mirageTrackingRobot
  (SFc2_mirageTrackingRobotInstanceStruct *chartInstance);
static void c2_update_debugger_state_c2_mirageTrackingRobot
  (SFc2_mirageTrackingRobotInstanceStruct *chartInstance);
static const mxArray *get_sim_state_c2_mirageTrackingRobot
  (SFc2_mirageTrackingRobotInstanceStruct *chartInstance);
static void set_sim_state_c2_mirageTrackingRobot
  (SFc2_mirageTrackingRobotInstanceStruct *chartInstance, const mxArray *c2_st);
static void finalize_c2_mirageTrackingRobot
  (SFc2_mirageTrackingRobotInstanceStruct *chartInstance);
static void sf_gateway_c2_mirageTrackingRobot
  (SFc2_mirageTrackingRobotInstanceStruct *chartInstance);
static void mdl_start_c2_mirageTrackingRobot
  (SFc2_mirageTrackingRobotInstanceStruct *chartInstance);
static void c2_chartstep_c2_mirageTrackingRobot
  (SFc2_mirageTrackingRobotInstanceStruct *chartInstance);
static void initSimStructsc2_mirageTrackingRobot
  (SFc2_mirageTrackingRobotInstanceStruct *chartInstance);
static void c2_orderPoints(SFc2_mirageTrackingRobotInstanceStruct *chartInstance,
  real_T c2_p[8], real_T c2_new_p[8]);
static void init_script_number_translation(uint32_T c2_machineNumber, uint32_T
  c2_chartNumber, uint32_T c2_instanceNumber);
static void c2_Outputs(SFc2_mirageTrackingRobotInstanceStruct *chartInstance,
  c2_vision_BlobAnalysis_1 *c2_obj, boolean_T c2_U0[1228800], int32_T
  c2_Y0_data[], int32_T c2_Y0_sizes[2], real_T c2_Y1_data[], int32_T
  c2_Y1_sizes[2], int32_T c2_Y2_data[], int32_T c2_Y2_sizes[2]);
static const mxArray *c2_sf_marshallOut(void *chartInstanceVoid, void *c2_inData);
static void c2_emlrt_marshallIn(SFc2_mirageTrackingRobotInstanceStruct
  *chartInstance, const mxArray *c2_b_R_p, const char_T *c2_identifier, real_T
  c2_f_y[8]);
static void c2_b_emlrt_marshallIn(SFc2_mirageTrackingRobotInstanceStruct
  *chartInstance, const mxArray *c2_f_u, const emlrtMsgIdentifier *c2_parentId,
  real_T c2_f_y[8]);
static void c2_sf_marshallIn(void *chartInstanceVoid, const mxArray
  *c2_mxArrayInData, const char_T *c2_varName, void *c2_outData);
static const mxArray *c2_b_sf_marshallOut(void *chartInstanceVoid, void
  *c2_inData);
static const mxArray *c2_c_sf_marshallOut(void *chartInstanceVoid, void
  *c2_inData);
static real_T c2_c_emlrt_marshallIn(SFc2_mirageTrackingRobotInstanceStruct
  *chartInstance, const mxArray *c2_f_u, const emlrtMsgIdentifier *c2_parentId);
static void c2_b_sf_marshallIn(void *chartInstanceVoid, const mxArray
  *c2_mxArrayInData, const char_T *c2_varName, void *c2_outData);
static const mxArray *c2_d_sf_marshallOut(void *chartInstanceVoid, void
  *c2_inData);
static void c2_d_emlrt_marshallIn(SFc2_mirageTrackingRobotInstanceStruct
  *chartInstance, const mxArray *c2_f_u, const emlrtMsgIdentifier *c2_parentId,
  uint8_T c2_f_y[1228800]);
static void c2_c_sf_marshallIn(void *chartInstanceVoid, const mxArray
  *c2_mxArrayInData, const char_T *c2_varName, void *c2_outData);
static void c2_e_emlrt_marshallIn(SFc2_mirageTrackingRobotInstanceStruct
  *chartInstance, const mxArray *c2_f_u, const emlrtMsgIdentifier *c2_parentId,
  uint8_T c2_f_y[3686400]);
static void c2_d_sf_marshallIn(void *chartInstanceVoid, const mxArray
  *c2_mxArrayInData, const char_T *c2_varName, void *c2_outData);
static const mxArray *c2_e_sf_marshallOut(void *chartInstanceVoid, real_T
  c2_inData_data[], int32_T c2_inData_sizes[2]);
static void c2_f_emlrt_marshallIn(SFc2_mirageTrackingRobotInstanceStruct
  *chartInstance, const mxArray *c2_f_u, const emlrtMsgIdentifier *c2_parentId,
  real_T c2_y_data[], int32_T c2_y_sizes[2]);
static void c2_e_sf_marshallIn(void *chartInstanceVoid, const mxArray
  *c2_mxArrayInData, const char_T *c2_varName, real_T c2_outData_data[], int32_T
  c2_outData_sizes[2]);
static const mxArray *c2_f_sf_marshallOut(void *chartInstanceVoid, void
  *c2_inData);
static void c2_g_emlrt_marshallIn(SFc2_mirageTrackingRobotInstanceStruct
  *chartInstance, const mxArray *c2_f_u, const emlrtMsgIdentifier *c2_parentId,
  boolean_T c2_f_y[1228800]);
static void c2_f_sf_marshallIn(void *chartInstanceVoid, const mxArray
  *c2_mxArrayInData, const char_T *c2_varName, void *c2_outData);
static const mxArray *c2_g_sf_marshallOut(void *chartInstanceVoid, void
  *c2_inData);
static void c2_h_emlrt_marshallIn(SFc2_mirageTrackingRobotInstanceStruct
  *chartInstance, const mxArray *c2_f_u, const emlrtMsgIdentifier *c2_parentId,
  real_T c2_f_y[1228800]);
static void c2_g_sf_marshallIn(void *chartInstanceVoid, const mxArray
  *c2_mxArrayInData, const char_T *c2_varName, void *c2_outData);
static const mxArray *c2_h_sf_marshallOut(void *chartInstanceVoid, void
  *c2_inData);
static void c2_i_emlrt_marshallIn(SFc2_mirageTrackingRobotInstanceStruct
  *chartInstance, const mxArray *c2_f_u, const emlrtMsgIdentifier *c2_parentId,
  real_T c2_f_y[3686400]);
static void c2_h_sf_marshallIn(void *chartInstanceVoid, const mxArray
  *c2_mxArrayInData, const char_T *c2_varName, void *c2_outData);
static const mxArray *c2_i_sf_marshallOut(void *chartInstanceVoid, void
  *c2_inData);
static void c2_j_emlrt_marshallIn(SFc2_mirageTrackingRobotInstanceStruct
  *chartInstance, const mxArray *c2_f_u, const emlrtMsgIdentifier *c2_parentId,
  real_T c2_f_y[4]);
static void c2_i_sf_marshallIn(void *chartInstanceVoid, const mxArray
  *c2_mxArrayInData, const char_T *c2_varName, void *c2_outData);
static void c2_info_helper(const mxArray **c2_info);
static const mxArray *c2_emlrt_marshallOut(const char * c2_f_u);
static const mxArray *c2_b_emlrt_marshallOut(const uint32_T c2_f_u);
static void c2_imcomplement(SFc2_mirageTrackingRobotInstanceStruct
  *chartInstance, uint8_T c2_im[3686400], uint8_T c2_b_im[3686400]);
static void c2_medfilt2(SFc2_mirageTrackingRobotInstanceStruct *chartInstance,
  uint8_T c2_g_varargin_1[1228800], uint8_T c2_d_b[1228800]);
static void c2_padarray(SFc2_mirageTrackingRobotInstanceStruct *chartInstance,
  uint8_T c2_g_varargin_1[1228800], uint8_T c2_d_b[1233284]);
static c2_coder_internal_cell_4 c2_cell_cell
  (SFc2_mirageTrackingRobotInstanceStruct *chartInstance);
static c2_coder_internal_cell_5 c2_b_cell_cell
  (SFc2_mirageTrackingRobotInstanceStruct *chartInstance);
static c2_coder_internal_cell_6 c2_c_cell_cell
  (SFc2_mirageTrackingRobotInstanceStruct *chartInstance);
static c2_coder_internal_cell_7 c2_d_cell_cell
  (SFc2_mirageTrackingRobotInstanceStruct *chartInstance);
static void c2_myMat2gray(SFc2_mirageTrackingRobotInstanceStruct *chartInstance,
  uint8_T c2_b_imgIn[1228800], real_T c2_imgOut[1228800]);
static void c2_check_forloop_overflow_error
  (SFc2_mirageTrackingRobotInstanceStruct *chartInstance, boolean_T c2_overflow);
static void c2_System_System(SFc2_mirageTrackingRobotInstanceStruct
  *chartInstance, c2_visioncodegen_BlobAnalysis *c2_obj);
static c2_visioncodegen_BlobAnalysis *c2_BlobAnalysis_BlobAnalysis
  (SFc2_mirageTrackingRobotInstanceStruct *chartInstance,
   c2_visioncodegen_BlobAnalysis *c2_obj);
static void c2_SystemCore_step(SFc2_mirageTrackingRobotInstanceStruct
  *chartInstance, c2_visioncodegen_BlobAnalysis *c2_obj, boolean_T
  c2_g_varargin_1[1228800], int32_T c2_varargout_1_data[], int32_T
  c2_varargout_1_sizes[2], real_T c2_varargout_2_data[], int32_T
  c2_varargout_2_sizes[2], int32_T c2_varargout_3_data[], int32_T
  c2_varargout_3_sizes[2]);
static void c2_Nondirect_stepImpl(SFc2_mirageTrackingRobotInstanceStruct
  *chartInstance, c2_visioncodegen_BlobAnalysis *c2_obj, boolean_T
  c2_g_varargin_1[1228800], int32_T c2_varargout_1_data[], int32_T
  c2_varargout_1_sizes[2], real_T c2_varargout_2_data[], int32_T
  c2_varargout_2_sizes[2], int32_T c2_varargout_3_data[], int32_T
  c2_varargout_3_sizes[2]);
static void c2_b_SystemCore_step(SFc2_mirageTrackingRobotInstanceStruct
  *chartInstance, c2_visioncodegen_BlobAnalysis *c2_obj, boolean_T
  c2_g_varargin_1[1228800], int32_T c2_varargout_1_data[], int32_T
  c2_varargout_1_sizes[2], real_T c2_varargout_2_data[], int32_T
  c2_varargout_2_sizes[2], int32_T c2_varargout_3_data[], int32_T
  c2_varargout_3_sizes[2]);
static void c2_b_Nondirect_stepImpl(SFc2_mirageTrackingRobotInstanceStruct
  *chartInstance, c2_visioncodegen_BlobAnalysis *c2_obj, boolean_T
  c2_g_varargin_1[1228800], int32_T c2_varargout_1_data[], int32_T
  c2_varargout_1_sizes[2], real_T c2_varargout_2_data[], int32_T
  c2_varargout_2_sizes[2], int32_T c2_varargout_3_data[], int32_T
  c2_varargout_3_sizes[2]);
static void c2_eml_sort(SFc2_mirageTrackingRobotInstanceStruct *chartInstance,
  real_T c2_x[4], real_T c2_b_x[4], int32_T c2_idx[4]);
static void c2_eml_sort_idx(SFc2_mirageTrackingRobotInstanceStruct
  *chartInstance, real_T c2_x[4], int32_T c2_idx[4], real_T c2_b_x[4]);
static void c2_merge(SFc2_mirageTrackingRobotInstanceStruct *chartInstance,
                     int32_T c2_idx[4], real_T c2_x[4], int32_T c2_offset,
                     int32_T c2_np, int32_T c2_nq, int32_T c2_b_idx[4], real_T
                     c2_b_x[4]);
static const mxArray *c2_j_sf_marshallOut(void *chartInstanceVoid, void
  *c2_inData);
static int32_T c2_k_emlrt_marshallIn(SFc2_mirageTrackingRobotInstanceStruct
  *chartInstance, const mxArray *c2_f_u, const emlrtMsgIdentifier *c2_parentId);
static void c2_j_sf_marshallIn(void *chartInstanceVoid, const mxArray
  *c2_mxArrayInData, const char_T *c2_varName, void *c2_outData);
static boolean_T c2_l_emlrt_marshallIn(SFc2_mirageTrackingRobotInstanceStruct
  *chartInstance, const mxArray *c2_b_isInitialized, const char_T *c2_identifier);
static boolean_T c2_m_emlrt_marshallIn(SFc2_mirageTrackingRobotInstanceStruct
  *chartInstance, const mxArray *c2_f_u, const emlrtMsgIdentifier *c2_parentId);
static uint8_T c2_n_emlrt_marshallIn(SFc2_mirageTrackingRobotInstanceStruct
  *chartInstance, const mxArray *c2_b_is_active_c2_mirageTrackingRobot, const
  char_T *c2_identifier);
static uint8_T c2_o_emlrt_marshallIn(SFc2_mirageTrackingRobotInstanceStruct
  *chartInstance, const mxArray *c2_f_u, const emlrtMsgIdentifier *c2_parentId);
static void c2_b_imcomplement(SFc2_mirageTrackingRobotInstanceStruct
  *chartInstance, uint8_T c2_im[3686400]);
static void c2_b_eml_sort(SFc2_mirageTrackingRobotInstanceStruct *chartInstance,
  real_T c2_x[4], int32_T c2_idx[4]);
static void c2_b_eml_sort_idx(SFc2_mirageTrackingRobotInstanceStruct
  *chartInstance, real_T c2_x[4], int32_T c2_idx[4]);
static void c2_b_merge(SFc2_mirageTrackingRobotInstanceStruct *chartInstance,
  int32_T c2_idx[4], real_T c2_x[4], int32_T c2_offset, int32_T c2_np, int32_T
  c2_nq);
static void init_dsm_address_info(SFc2_mirageTrackingRobotInstanceStruct
  *chartInstance);
static void init_simulink_io_address(SFc2_mirageTrackingRobotInstanceStruct
  *chartInstance);

/* Function Definitions */
static void initialize_c2_mirageTrackingRobot
  (SFc2_mirageTrackingRobotInstanceStruct *chartInstance)
{
  if (sf_is_first_init_cond(chartInstance->S)) {
    initSimStructsc2_mirageTrackingRobot(chartInstance);
    chart_debug_initialize_data_addresses(chartInstance->S);
  }

  chartInstance->c2_sfEvent = CALL_EVENT;
  _sfTime_ = sf_get_time(chartInstance->S);
  chartInstance->c2_is_active_c2_mirageTrackingRobot = 0U;
}

static void initialize_params_c2_mirageTrackingRobot
  (SFc2_mirageTrackingRobotInstanceStruct *chartInstance)
{
  (void)chartInstance;
}

static void enable_c2_mirageTrackingRobot(SFc2_mirageTrackingRobotInstanceStruct
  *chartInstance)
{
  _sfTime_ = sf_get_time(chartInstance->S);
}

static void disable_c2_mirageTrackingRobot
  (SFc2_mirageTrackingRobotInstanceStruct *chartInstance)
{
  _sfTime_ = sf_get_time(chartInstance->S);
}

static void c2_update_debugger_state_c2_mirageTrackingRobot
  (SFc2_mirageTrackingRobotInstanceStruct *chartInstance)
{
  (void)chartInstance;
}

static const mxArray *get_sim_state_c2_mirageTrackingRobot
  (SFc2_mirageTrackingRobotInstanceStruct *chartInstance)
{
  const mxArray *c2_st;
  const mxArray *c2_f_y = NULL;
  const mxArray *c2_g_y = NULL;
  const mxArray *c2_h_y = NULL;
  boolean_T c2_hoistedGlobal;
  boolean_T c2_f_u;
  const mxArray *c2_i_y = NULL;
  uint8_T c2_b_hoistedGlobal;
  uint8_T c2_g_u;
  const mxArray *c2_j_y = NULL;
  c2_st = NULL;
  c2_st = NULL;
  c2_f_y = NULL;
  sf_mex_assign(&c2_f_y, sf_mex_createcellmatrix(4, 1), false);
  c2_g_y = NULL;
  sf_mex_assign(&c2_g_y, sf_mex_create("y", *chartInstance->c2_L_p, 0, 0U, 1U,
    0U, 2, 2, 4), false);
  sf_mex_setcell(c2_f_y, 0, c2_g_y);
  c2_h_y = NULL;
  sf_mex_assign(&c2_h_y, sf_mex_create("y", *chartInstance->c2_R_p, 0, 0U, 1U,
    0U, 2, 2, 4), false);
  sf_mex_setcell(c2_f_y, 1, c2_h_y);
  c2_hoistedGlobal = chartInstance->c2_isInitialized;
  c2_f_u = c2_hoistedGlobal;
  c2_i_y = NULL;
  sf_mex_assign(&c2_i_y, sf_mex_create("y", &c2_f_u, 11, 0U, 0U, 0U, 0), false);
  sf_mex_setcell(c2_f_y, 2, c2_i_y);
  c2_b_hoistedGlobal = chartInstance->c2_is_active_c2_mirageTrackingRobot;
  c2_g_u = c2_b_hoistedGlobal;
  c2_j_y = NULL;
  sf_mex_assign(&c2_j_y, sf_mex_create("y", &c2_g_u, 3, 0U, 0U, 0U, 0), false);
  sf_mex_setcell(c2_f_y, 3, c2_j_y);
  sf_mex_assign(&c2_st, c2_f_y, false);
  return c2_st;
}

static void set_sim_state_c2_mirageTrackingRobot
  (SFc2_mirageTrackingRobotInstanceStruct *chartInstance, const mxArray *c2_st)
{
  const mxArray *c2_f_u;
  real_T c2_dv0[8];
  int32_T c2_i0;
  real_T c2_dv1[8];
  int32_T c2_i1;
  chartInstance->c2_doneDoubleBufferReInit = true;
  c2_f_u = sf_mex_dup(c2_st);
  c2_emlrt_marshallIn(chartInstance, sf_mex_dup(sf_mex_getcell("L_p", c2_f_u, 0)),
                      "L_p", c2_dv0);
  for (c2_i0 = 0; c2_i0 < 8; c2_i0++) {
    (*chartInstance->c2_L_p)[c2_i0] = c2_dv0[c2_i0];
  }

  c2_emlrt_marshallIn(chartInstance, sf_mex_dup(sf_mex_getcell("R_p", c2_f_u, 1)),
                      "R_p", c2_dv1);
  for (c2_i1 = 0; c2_i1 < 8; c2_i1++) {
    (*chartInstance->c2_R_p)[c2_i1] = c2_dv1[c2_i1];
  }

  chartInstance->c2_isInitialized = c2_l_emlrt_marshallIn(chartInstance,
    sf_mex_dup(sf_mex_getcell("isInitialized", c2_f_u, 2)), "isInitialized");
  chartInstance->c2_is_active_c2_mirageTrackingRobot = c2_n_emlrt_marshallIn
    (chartInstance, sf_mex_dup(sf_mex_getcell("is_active_c2_mirageTrackingRobot",
       c2_f_u, 3)), "is_active_c2_mirageTrackingRobot");
  sf_mex_destroy(&c2_f_u);
  c2_update_debugger_state_c2_mirageTrackingRobot(chartInstance);
  sf_mex_destroy(&c2_st);
}

static void finalize_c2_mirageTrackingRobot
  (SFc2_mirageTrackingRobotInstanceStruct *chartInstance)
{
  (void)chartInstance;
}

static void sf_gateway_c2_mirageTrackingRobot
  (SFc2_mirageTrackingRobotInstanceStruct *chartInstance)
{
  int32_T c2_i2;
  int32_T c2_i3;
  int32_T c2_i4;
  int32_T c2_i5;
  _SFD_SYMBOL_SCOPE_PUSH(0U, 0U);
  _sfTime_ = sf_get_time(chartInstance->S);
  _SFD_CC_CALL(CHART_ENTER_SFUNCTION_TAG, 1U, chartInstance->c2_sfEvent);
  for (c2_i2 = 0; c2_i2 < 3686400; c2_i2++) {
    _SFD_DATA_RANGE_CHECK((real_T)(*chartInstance->c2_b_r_rgb)[c2_i2], 1U, 1U,
                          0U, chartInstance->c2_sfEvent, false);
  }

  for (c2_i3 = 0; c2_i3 < 3686400; c2_i3++) {
    _SFD_DATA_RANGE_CHECK((real_T)(*chartInstance->c2_b_l_rgb)[c2_i3], 0U, 1U,
                          0U, chartInstance->c2_sfEvent, false);
  }

  chartInstance->c2_sfEvent = CALL_EVENT;
  c2_chartstep_c2_mirageTrackingRobot(chartInstance);
  _SFD_SYMBOL_SCOPE_POP();
  _SFD_CHECK_FOR_STATE_INCONSISTENCY(_mirageTrackingRobotMachineNumber_,
    chartInstance->chartNumber, chartInstance->instanceNumber);
  for (c2_i4 = 0; c2_i4 < 8; c2_i4++) {
    _SFD_DATA_RANGE_CHECK((*chartInstance->c2_L_p)[c2_i4], 2U, 1U, 0U,
                          chartInstance->c2_sfEvent, false);
  }

  for (c2_i5 = 0; c2_i5 < 8; c2_i5++) {
    _SFD_DATA_RANGE_CHECK((*chartInstance->c2_R_p)[c2_i5], 3U, 1U, 0U,
                          chartInstance->c2_sfEvent, false);
  }
}

static void mdl_start_c2_mirageTrackingRobot
  (SFc2_mirageTrackingRobotInstanceStruct *chartInstance)
{
  (void)chartInstance;
}

static void c2_chartstep_c2_mirageTrackingRobot
  (SFc2_mirageTrackingRobotInstanceStruct *chartInstance)
{
  int32_T c2_i6;
  int32_T c2_i7;
  uint32_T c2_debug_family_var_map[22];
  real_T c2_m;
  real_T c2_n;
  real_T c2_c;
  int32_T c2_l_center_sizes[2];
  real_T c2_l_center_data[100];
  int32_T c2_r_center_sizes[2];
  real_T c2_r_center_data[100];
  real_T c2_ml;
  real_T c2_i;
  real_T c2_mr;
  real_T c2_nargin = 2.0;
  real_T c2_nargout = 2.0;
  real_T c2_b_L_p[8];
  real_T c2_b_R_p[8];
  int32_T c2_i8;
  int32_T c2_i9;
  int32_T c2_i10;
  int32_T c2_i11;
  int32_T c2_i12;
  int32_T c2_i13;
  int32_T c2_i14;
  int32_T c2_i15;
  int32_T c2_i16;
  int32_T c2_i17;
  int32_T c2_i18;
  int32_T c2_i19;
  int32_T c2_i20;
  int32_T c2_i21;
  int32_T c2_i22;
  int32_T c2_i23;
  int32_T c2_i24;
  int32_T c2_i25;
  real_T c2_d0;
  uint8_T c2_u0;
  int32_T c2_i26;
  int32_T c2_i27;
  int32_T c2_i28;
  int32_T c2_i29;
  real_T c2_d1;
  uint8_T c2_u1;
  int32_T c2_i30;
  int32_T c2_i31;
  int32_T c2_i32;
  int32_T c2_i33;
  real_T c2_d2;
  uint8_T c2_u2;
  int32_T c2_i34;
  uint32_T c2_u3;
  uint32_T c2_u4;
  int32_T c2_i35;
  int32_T c2_i36;
  int32_T c2_i37;
  int32_T c2_i38;
  real_T c2_d3;
  uint8_T c2_u5;
  int32_T c2_i39;
  int32_T c2_i40;
  int32_T c2_i41;
  int32_T c2_i42;
  real_T c2_d4;
  uint8_T c2_u6;
  int32_T c2_i43;
  int32_T c2_i44;
  int32_T c2_i45;
  int32_T c2_i46;
  real_T c2_d5;
  uint8_T c2_u7;
  int32_T c2_i47;
  uint32_T c2_u8;
  uint32_T c2_u9;
  int32_T c2_i48;
  int32_T c2_i49;
  int32_T c2_i50;
  uint32_T c2_q0;
  uint32_T c2_qY;
  uint32_T c2_u10;
  int32_T c2_i51;
  int32_T c2_i52;
  int32_T c2_i53;
  uint32_T c2_b_q0;
  uint32_T c2_b_qY;
  uint32_T c2_u11;
  int32_T c2_i54;
  int32_T c2_i55;
  int32_T c2_i56;
  int32_T c2_i57;
  int32_T c2_i58;
  int32_T c2_i59;
  int32_T c2_i60;
  int32_T c2_i61;
  int32_T c2_i62;
  int32_T c2_i63;
  int32_T c2_i64;
  int32_T c2_unusedU1_sizes[2];
  int32_T c2_unusedU1_data[200];
  int32_T c2_b_l_center_sizes[2];
  real_T c2_b_l_center_data[100];
  int32_T c2_unusedU0_sizes[2];
  int32_T c2_unusedU0_data[50];
  int32_T c2_l_center;
  int32_T c2_b_l_center;
  int32_T c2_loop_ub;
  int32_T c2_i65;
  int32_T c2_i66;
  int32_T c2_r_center;
  int32_T c2_b_r_center;
  int32_T c2_b_loop_ub;
  int32_T c2_i67;
  real_T c2_b_ml;
  int32_T c2_i68;
  int32_T c2_b_i;
  int32_T c2_i69;
  int32_T c2_i70;
  int32_T c2_c_i;
  int32_T c2_tmp_sizes[2];
  int32_T c2_c_loop_ub;
  int32_T c2_i71;
  real_T c2_tmp_data[2];
  static int32_T c2_iv0[1] = { 2 };

  int32_T c2_i72;
  int32_T c2_i73;
  real_T c2_b_mr;
  int32_T c2_i74;
  int32_T c2_d_i;
  int32_T c2_i75;
  int32_T c2_i76;
  int32_T c2_e_i;
  int32_T c2_d_loop_ub;
  int32_T c2_i77;
  int32_T c2_i78;
  int32_T c2_i79;
  int32_T c2_i80;
  real_T c2_c_L_p[8];
  real_T c2_dv4[8];
  int32_T c2_i81;
  int32_T c2_i82;
  real_T c2_c_R_p[8];
  real_T c2_dv5[8];
  int32_T c2_i83;
  int32_T c2_i84;
  int32_T c2_i85;
  int32_T c2_i86;
  int32_T c2_i87;
  int32_T c2_i88;
  int32_T c2_i89;
  _SFD_CC_CALL(CHART_ENTER_DURING_FUNCTION_TAG, 1U, chartInstance->c2_sfEvent);
  for (c2_i6 = 0; c2_i6 < 3686400; c2_i6++) {
    chartInstance->c2_l_rgb[c2_i6] = (*chartInstance->c2_b_l_rgb)[c2_i6];
  }

  for (c2_i7 = 0; c2_i7 < 3686400; c2_i7++) {
    chartInstance->c2_r_rgb[c2_i7] = (*chartInstance->c2_b_r_rgb)[c2_i7];
  }

  _SFD_SYMBOL_SCOPE_PUSH_EML(0U, 22U, 28U, c2_debug_family_names,
    c2_debug_family_var_map);
  _SFD_SYMBOL_SCOPE_ADD_EML(&c2_m, 0U, c2_c_sf_marshallOut);
  _SFD_SYMBOL_SCOPE_ADD_EML(&c2_n, 1U, c2_c_sf_marshallOut);
  _SFD_SYMBOL_SCOPE_ADD_EML(&c2_c, 2U, c2_c_sf_marshallOut);
  _SFD_SYMBOL_SCOPE_ADD_EML_IMPORTABLE(chartInstance->c2_l_gray, MAX_uint32_T,
    c2_g_sf_marshallOut, c2_g_sf_marshallIn);
  _SFD_SYMBOL_SCOPE_ADD_EML_IMPORTABLE(chartInstance->c2_r_gray, MAX_uint32_T,
    c2_g_sf_marshallOut, c2_g_sf_marshallIn);
  _SFD_SYMBOL_SCOPE_ADD_EML_IMPORTABLE(chartInstance->c2_l_rgb1, MAX_uint32_T,
    c2_h_sf_marshallOut, c2_h_sf_marshallIn);
  _SFD_SYMBOL_SCOPE_ADD_EML_IMPORTABLE(chartInstance->c2_r_rgb1, MAX_uint32_T,
    c2_h_sf_marshallOut, c2_h_sf_marshallIn);
  _SFD_SYMBOL_SCOPE_ADD_EML_IMPORTABLE(chartInstance->c2_l_rgb2, MAX_uint32_T,
    c2_g_sf_marshallOut, c2_g_sf_marshallIn);
  _SFD_SYMBOL_SCOPE_ADD_EML_IMPORTABLE(chartInstance->c2_r_rgb2, MAX_uint32_T,
    c2_g_sf_marshallOut, c2_g_sf_marshallIn);
  _SFD_SYMBOL_SCOPE_ADD_EML_IMPORTABLE(chartInstance->c2_l_bw, 9U,
    c2_f_sf_marshallOut, c2_f_sf_marshallIn);
  _SFD_SYMBOL_SCOPE_ADD_EML_IMPORTABLE(chartInstance->c2_r_bw, 10U,
    c2_f_sf_marshallOut, c2_f_sf_marshallIn);
  _SFD_SYMBOL_SCOPE_ADD_EML_DYN_IMPORTABLE(c2_l_center_data, (const int32_T *)
    &c2_l_center_sizes, NULL, 0, 11, (void *)c2_e_sf_marshallOut, (void *)
    c2_e_sf_marshallIn);
  _SFD_SYMBOL_SCOPE_ADD_EML_DYN_IMPORTABLE(c2_r_center_data, (const int32_T *)
    &c2_r_center_sizes, NULL, 0, 12, (void *)c2_e_sf_marshallOut, (void *)
    c2_e_sf_marshallIn);
  _SFD_SYMBOL_SCOPE_ADD_EML_IMPORTABLE(&c2_ml, 13U, c2_c_sf_marshallOut,
    c2_b_sf_marshallIn);
  _SFD_SYMBOL_SCOPE_ADD_EML_IMPORTABLE(&c2_i, 14U, c2_c_sf_marshallOut,
    c2_b_sf_marshallIn);
  _SFD_SYMBOL_SCOPE_ADD_EML_IMPORTABLE(&c2_mr, 15U, c2_c_sf_marshallOut,
    c2_b_sf_marshallIn);
  _SFD_SYMBOL_SCOPE_ADD_EML_IMPORTABLE(chartInstance->c2_b_l_rgb1, MAX_uint32_T,
    c2_b_sf_marshallOut, c2_d_sf_marshallIn);
  _SFD_SYMBOL_SCOPE_ADD_EML_IMPORTABLE(chartInstance->c2_b_r_rgb1, MAX_uint32_T,
    c2_b_sf_marshallOut, c2_d_sf_marshallIn);
  _SFD_SYMBOL_SCOPE_ADD_EML_IMPORTABLE(chartInstance->c2_b_l_gray, MAX_uint32_T,
    c2_d_sf_marshallOut, c2_c_sf_marshallIn);
  _SFD_SYMBOL_SCOPE_ADD_EML_IMPORTABLE(chartInstance->c2_b_r_gray, MAX_uint32_T,
    c2_d_sf_marshallOut, c2_c_sf_marshallIn);
  _SFD_SYMBOL_SCOPE_ADD_EML_IMPORTABLE(chartInstance->c2_b_l_rgb2, MAX_uint32_T,
    c2_d_sf_marshallOut, c2_c_sf_marshallIn);
  _SFD_SYMBOL_SCOPE_ADD_EML_IMPORTABLE(chartInstance->c2_b_r_rgb2, MAX_uint32_T,
    c2_d_sf_marshallOut, c2_c_sf_marshallIn);
  _SFD_SYMBOL_SCOPE_ADD_EML_IMPORTABLE(&c2_nargin, 16U, c2_c_sf_marshallOut,
    c2_b_sf_marshallIn);
  _SFD_SYMBOL_SCOPE_ADD_EML_IMPORTABLE(&c2_nargout, 17U, c2_c_sf_marshallOut,
    c2_b_sf_marshallIn);
  _SFD_SYMBOL_SCOPE_ADD_EML(chartInstance->c2_l_rgb, 18U, c2_b_sf_marshallOut);
  _SFD_SYMBOL_SCOPE_ADD_EML(chartInstance->c2_r_rgb, 19U, c2_b_sf_marshallOut);
  _SFD_SYMBOL_SCOPE_ADD_EML_IMPORTABLE(c2_b_L_p, 20U, c2_sf_marshallOut,
    c2_sf_marshallIn);
  _SFD_SYMBOL_SCOPE_ADD_EML_IMPORTABLE(c2_b_R_p, 21U, c2_sf_marshallOut,
    c2_sf_marshallIn);
  CV_EML_FCN(0, 0);
  _SFD_EML_CALL(0U, chartInstance->c2_sfEvent, 4);
  for (c2_i8 = 0; c2_i8 < 8; c2_i8++) {
    c2_b_L_p[c2_i8] = 0.0;
  }

  _SFD_EML_CALL(0U, chartInstance->c2_sfEvent, 5);
  for (c2_i9 = 0; c2_i9 < 8; c2_i9++) {
    c2_b_R_p[c2_i9] = 0.0;
  }

  _SFD_EML_CALL(0U, chartInstance->c2_sfEvent, 8);
  c2_m = 960.0;
  c2_n = 1280.0;
  c2_c = 3.0;
  _SFD_EML_CALL(0U, chartInstance->c2_sfEvent, 11);
  for (c2_i10 = 0; c2_i10 < 1228800; c2_i10++) {
    chartInstance->c2_l_gray[c2_i10] = 0.0;
  }

  _SFD_SYMBOL_SWITCH(3U, 4U);
  _SFD_EML_CALL(0U, chartInstance->c2_sfEvent, 11);
  for (c2_i11 = 0; c2_i11 < 1228800; c2_i11++) {
    chartInstance->c2_r_gray[c2_i11] = 0.0;
  }

  _SFD_SYMBOL_SWITCH(4U, 5U);
  _SFD_EML_CALL(0U, chartInstance->c2_sfEvent, 12);
  for (c2_i12 = 0; c2_i12 < 3686400; c2_i12++) {
    chartInstance->c2_l_rgb1[c2_i12] = 0.0;
  }

  _SFD_SYMBOL_SWITCH(5U, 6U);
  _SFD_EML_CALL(0U, chartInstance->c2_sfEvent, 12);
  for (c2_i13 = 0; c2_i13 < 3686400; c2_i13++) {
    chartInstance->c2_r_rgb1[c2_i13] = 0.0;
  }

  _SFD_SYMBOL_SWITCH(6U, 7U);
  _SFD_EML_CALL(0U, chartInstance->c2_sfEvent, 13);
  for (c2_i14 = 0; c2_i14 < 1228800; c2_i14++) {
    chartInstance->c2_l_rgb2[c2_i14] = 0.0;
  }

  _SFD_SYMBOL_SWITCH(7U, 8U);
  _SFD_EML_CALL(0U, chartInstance->c2_sfEvent, 13);
  for (c2_i15 = 0; c2_i15 < 1228800; c2_i15++) {
    chartInstance->c2_r_rgb2[c2_i15] = 0.0;
  }

  _SFD_SYMBOL_SWITCH(8U, 9U);
  _SFD_EML_CALL(0U, chartInstance->c2_sfEvent, 15);
  for (c2_i16 = 0; c2_i16 < 1228800; c2_i16++) {
    chartInstance->c2_l_bw[c2_i16] = false;
  }

  _SFD_EML_CALL(0U, chartInstance->c2_sfEvent, 15);
  for (c2_i17 = 0; c2_i17 < 1228800; c2_i17++) {
    chartInstance->c2_r_bw[c2_i17] = false;
  }

  _SFD_EML_CALL(0U, chartInstance->c2_sfEvent, 17);
  for (c2_i18 = 0; c2_i18 < 3686400; c2_i18++) {
    chartInstance->c2_uv0[c2_i18] = chartInstance->c2_l_rgb[c2_i18];
  }

  c2_b_imcomplement(chartInstance, chartInstance->c2_uv0);
  for (c2_i19 = 0; c2_i19 < 3686400; c2_i19++) {
    chartInstance->c2_b_l_rgb1[c2_i19] = chartInstance->c2_uv0[c2_i19];
  }

  _SFD_SYMBOL_SWITCH(5U, 17U);
  _SFD_EML_CALL(0U, chartInstance->c2_sfEvent, 18);
  for (c2_i20 = 0; c2_i20 < 3686400; c2_i20++) {
    chartInstance->c2_uv0[c2_i20] = chartInstance->c2_r_rgb[c2_i20];
  }

  c2_b_imcomplement(chartInstance, chartInstance->c2_uv0);
  for (c2_i21 = 0; c2_i21 < 3686400; c2_i21++) {
    chartInstance->c2_b_r_rgb1[c2_i21] = chartInstance->c2_uv0[c2_i21];
  }

  _SFD_SYMBOL_SWITCH(6U, 18U);
  _SFD_EML_CALL(0U, chartInstance->c2_sfEvent, 20);
  c2_i22 = 0;
  for (c2_i23 = 0; c2_i23 < 1280; c2_i23++) {
    for (c2_i24 = 0; c2_i24 < 960; c2_i24++) {
      chartInstance->c2_b[c2_i24 + c2_i22] = chartInstance->c2_b_l_rgb1[c2_i24 +
        c2_i22];
    }

    c2_i22 += 960;
  }

  for (c2_i25 = 0; c2_i25 < 1228800; c2_i25++) {
    c2_d0 = muDoubleScalarRound(0.2989 * (real_T)chartInstance->c2_b[c2_i25]);
    if (c2_d0 < 256.0) {
      if (CV_SATURATION_EVAL(4, 0, 0, 1, c2_d0 >= 0.0)) {
        c2_u0 = (uint8_T)c2_d0;
      } else {
        c2_u0 = 0U;
      }
    } else if (CV_SATURATION_EVAL(4, 0, 0, 0, c2_d0 >= 256.0)) {
      c2_u0 = MAX_uint8_T;
    } else {
      c2_u0 = 0U;
    }

    chartInstance->c2_b[c2_i25] = c2_u0;
  }

  c2_i26 = 0;
  for (c2_i27 = 0; c2_i27 < 1280; c2_i27++) {
    for (c2_i28 = 0; c2_i28 < 960; c2_i28++) {
      chartInstance->c2_b_b[c2_i28 + c2_i26] = chartInstance->c2_b_l_rgb1
        [(c2_i28 + c2_i26) + 1228800];
    }

    c2_i26 += 960;
  }

  for (c2_i29 = 0; c2_i29 < 1228800; c2_i29++) {
    c2_d1 = muDoubleScalarRound(0.587 * (real_T)chartInstance->c2_b_b[c2_i29]);
    if (c2_d1 < 256.0) {
      if (CV_SATURATION_EVAL(4, 0, 3, 1, c2_d1 >= 0.0)) {
        c2_u1 = (uint8_T)c2_d1;
      } else {
        c2_u1 = 0U;
      }
    } else if (CV_SATURATION_EVAL(4, 0, 3, 0, c2_d1 >= 256.0)) {
      c2_u1 = MAX_uint8_T;
    } else {
      c2_u1 = 0U;
    }

    chartInstance->c2_b_b[c2_i29] = c2_u1;
  }

  c2_i30 = 0;
  for (c2_i31 = 0; c2_i31 < 1280; c2_i31++) {
    for (c2_i32 = 0; c2_i32 < 960; c2_i32++) {
      chartInstance->c2_c_b[c2_i32 + c2_i30] = chartInstance->c2_b_l_rgb1
        [(c2_i32 + c2_i30) + 2457600];
    }

    c2_i30 += 960;
  }

  for (c2_i33 = 0; c2_i33 < 1228800; c2_i33++) {
    c2_d2 = muDoubleScalarRound(0.114 * (real_T)chartInstance->c2_c_b[c2_i33]);
    if (c2_d2 < 256.0) {
      if (CV_SATURATION_EVAL(4, 0, 4, 1, c2_d2 >= 0.0)) {
        c2_u2 = (uint8_T)c2_d2;
      } else {
        c2_u2 = 0U;
      }
    } else if (CV_SATURATION_EVAL(4, 0, 4, 0, c2_d2 >= 256.0)) {
      c2_u2 = MAX_uint8_T;
    } else {
      c2_u2 = 0U;
    }

    chartInstance->c2_c_b[c2_i33] = c2_u2;
  }

  for (c2_i34 = 0; c2_i34 < 1228800; c2_i34++) {
    c2_u3 = (uint32_T)chartInstance->c2_b[c2_i34] + (uint32_T)
      chartInstance->c2_b_b[c2_i34];
    if (CV_SATURATION_EVAL(4, 0, 1, 0, c2_u3 > 255U)) {
      c2_u3 = 255U;
    }

    c2_u4 = (uint32_T)(uint8_T)c2_u3 + (uint32_T)chartInstance->c2_c_b[c2_i34];
    if (CV_SATURATION_EVAL(4, 0, 2, 0, c2_u4 > 255U)) {
      c2_u4 = 255U;
    }

    chartInstance->c2_b_l_gray[c2_i34] = (uint8_T)c2_u4;
  }

  _SFD_SYMBOL_SWITCH(3U, 19U);
  _SFD_EML_CALL(0U, chartInstance->c2_sfEvent, 21);
  c2_i35 = 0;
  for (c2_i36 = 0; c2_i36 < 1280; c2_i36++) {
    for (c2_i37 = 0; c2_i37 < 960; c2_i37++) {
      chartInstance->c2_b[c2_i37 + c2_i35] = chartInstance->c2_b_r_rgb1[c2_i37 +
        c2_i35];
    }

    c2_i35 += 960;
  }

  for (c2_i38 = 0; c2_i38 < 1228800; c2_i38++) {
    c2_d3 = muDoubleScalarRound(0.2989 * (real_T)chartInstance->c2_b[c2_i38]);
    if (c2_d3 < 256.0) {
      if (CV_SATURATION_EVAL(4, 0, 5, 1, c2_d3 >= 0.0)) {
        c2_u5 = (uint8_T)c2_d3;
      } else {
        c2_u5 = 0U;
      }
    } else if (CV_SATURATION_EVAL(4, 0, 5, 0, c2_d3 >= 256.0)) {
      c2_u5 = MAX_uint8_T;
    } else {
      c2_u5 = 0U;
    }

    chartInstance->c2_b[c2_i38] = c2_u5;
  }

  c2_i39 = 0;
  for (c2_i40 = 0; c2_i40 < 1280; c2_i40++) {
    for (c2_i41 = 0; c2_i41 < 960; c2_i41++) {
      chartInstance->c2_b_b[c2_i41 + c2_i39] = chartInstance->c2_b_r_rgb1
        [(c2_i41 + c2_i39) + 1228800];
    }

    c2_i39 += 960;
  }

  for (c2_i42 = 0; c2_i42 < 1228800; c2_i42++) {
    c2_d4 = muDoubleScalarRound(0.587 * (real_T)chartInstance->c2_b_b[c2_i42]);
    if (c2_d4 < 256.0) {
      if (CV_SATURATION_EVAL(4, 0, 8, 1, c2_d4 >= 0.0)) {
        c2_u6 = (uint8_T)c2_d4;
      } else {
        c2_u6 = 0U;
      }
    } else if (CV_SATURATION_EVAL(4, 0, 8, 0, c2_d4 >= 256.0)) {
      c2_u6 = MAX_uint8_T;
    } else {
      c2_u6 = 0U;
    }

    chartInstance->c2_b_b[c2_i42] = c2_u6;
  }

  c2_i43 = 0;
  for (c2_i44 = 0; c2_i44 < 1280; c2_i44++) {
    for (c2_i45 = 0; c2_i45 < 960; c2_i45++) {
      chartInstance->c2_c_b[c2_i45 + c2_i43] = chartInstance->c2_b_r_rgb1
        [(c2_i45 + c2_i43) + 2457600];
    }

    c2_i43 += 960;
  }

  for (c2_i46 = 0; c2_i46 < 1228800; c2_i46++) {
    c2_d5 = muDoubleScalarRound(0.114 * (real_T)chartInstance->c2_c_b[c2_i46]);
    if (c2_d5 < 256.0) {
      if (CV_SATURATION_EVAL(4, 0, 9, 1, c2_d5 >= 0.0)) {
        c2_u7 = (uint8_T)c2_d5;
      } else {
        c2_u7 = 0U;
      }
    } else if (CV_SATURATION_EVAL(4, 0, 9, 0, c2_d5 >= 256.0)) {
      c2_u7 = MAX_uint8_T;
    } else {
      c2_u7 = 0U;
    }

    chartInstance->c2_c_b[c2_i46] = c2_u7;
  }

  for (c2_i47 = 0; c2_i47 < 1228800; c2_i47++) {
    c2_u8 = (uint32_T)chartInstance->c2_b[c2_i47] + (uint32_T)
      chartInstance->c2_b_b[c2_i47];
    if (CV_SATURATION_EVAL(4, 0, 6, 0, c2_u8 > 255U)) {
      c2_u8 = 255U;
    }

    c2_u9 = (uint32_T)(uint8_T)c2_u8 + (uint32_T)chartInstance->c2_c_b[c2_i47];
    if (CV_SATURATION_EVAL(4, 0, 7, 0, c2_u9 > 255U)) {
      c2_u9 = 255U;
    }

    chartInstance->c2_b_r_gray[c2_i47] = (uint8_T)c2_u9;
  }

  _SFD_SYMBOL_SWITCH(4U, 20U);
  _SFD_EML_CALL(0U, chartInstance->c2_sfEvent, 23);
  c2_i48 = 0;
  for (c2_i49 = 0; c2_i49 < 1280; c2_i49++) {
    for (c2_i50 = 0; c2_i50 < 960; c2_i50++) {
      c2_q0 = chartInstance->c2_b_l_rgb1[(c2_i50 + c2_i48) + 2457600];
      c2_qY = c2_q0 - (uint32_T)chartInstance->c2_b_l_gray[c2_i50 + c2_i48];
      if (CV_SATURATION_EVAL(4, 0, 10, 0, c2_qY > c2_q0)) {
        c2_qY = 0U;
      }

      c2_u10 = c2_qY;
      if (CV_SATURATION_EVAL(4, 0, 10, 0, c2_u10 > 255U)) {
        c2_u10 = 255U;
      }

      chartInstance->c2_b_l_rgb2[c2_i50 + c2_i48] = (uint8_T)c2_u10;
    }

    c2_i48 += 960;
  }

  _SFD_SYMBOL_SWITCH(7U, 21U);
  _SFD_EML_CALL(0U, chartInstance->c2_sfEvent, 24);
  c2_i51 = 0;
  for (c2_i52 = 0; c2_i52 < 1280; c2_i52++) {
    for (c2_i53 = 0; c2_i53 < 960; c2_i53++) {
      c2_b_q0 = chartInstance->c2_b_r_rgb1[(c2_i53 + c2_i51) + 2457600];
      c2_b_qY = c2_b_q0 - (uint32_T)chartInstance->c2_b_r_gray[c2_i53 + c2_i51];
      if (CV_SATURATION_EVAL(4, 0, 11, 0, c2_b_qY > c2_b_q0)) {
        c2_b_qY = 0U;
      }

      c2_u11 = c2_b_qY;
      if (CV_SATURATION_EVAL(4, 0, 11, 0, c2_u11 > 255U)) {
        c2_u11 = 255U;
      }

      chartInstance->c2_b_r_rgb2[c2_i53 + c2_i51] = (uint8_T)c2_u11;
    }

    c2_i51 += 960;
  }

  _SFD_SYMBOL_SWITCH(8U, 22U);
  _SFD_EML_CALL(0U, chartInstance->c2_sfEvent, 26);
  for (c2_i54 = 0; c2_i54 < 1228800; c2_i54++) {
    chartInstance->c2_c_l_rgb2[c2_i54] = chartInstance->c2_b_l_rgb2[c2_i54];
  }

  c2_medfilt2(chartInstance, chartInstance->c2_c_l_rgb2, chartInstance->c2_uv1);
  for (c2_i55 = 0; c2_i55 < 1228800; c2_i55++) {
    chartInstance->c2_b_l_rgb2[c2_i55] = chartInstance->c2_uv1[c2_i55];
  }

  _SFD_SYMBOL_SWITCH(7U, 21U);
  _SFD_EML_CALL(0U, chartInstance->c2_sfEvent, 27);
  for (c2_i56 = 0; c2_i56 < 1228800; c2_i56++) {
    chartInstance->c2_c_r_rgb2[c2_i56] = chartInstance->c2_b_r_rgb2[c2_i56];
  }

  c2_medfilt2(chartInstance, chartInstance->c2_c_r_rgb2, chartInstance->c2_uv2);
  for (c2_i57 = 0; c2_i57 < 1228800; c2_i57++) {
    chartInstance->c2_b_r_rgb2[c2_i57] = chartInstance->c2_uv2[c2_i57];
  }

  _SFD_SYMBOL_SWITCH(8U, 22U);
  _SFD_EML_CALL(0U, chartInstance->c2_sfEvent, 29);
  for (c2_i58 = 0; c2_i58 < 1228800; c2_i58++) {
    chartInstance->c2_d_l_rgb2[c2_i58] = chartInstance->c2_b_l_rgb2[c2_i58];
  }

  c2_myMat2gray(chartInstance, chartInstance->c2_d_l_rgb2, chartInstance->c2_dv2);
  for (c2_i59 = 0; c2_i59 < 1228800; c2_i59++) {
    chartInstance->c2_l_rgb2[c2_i59] = chartInstance->c2_dv2[c2_i59];
  }

  _SFD_SYMBOL_SWITCH(7U, 8U);
  _SFD_EML_CALL(0U, chartInstance->c2_sfEvent, 30);
  for (c2_i60 = 0; c2_i60 < 1228800; c2_i60++) {
    chartInstance->c2_d_r_rgb2[c2_i60] = chartInstance->c2_b_r_rgb2[c2_i60];
  }

  c2_myMat2gray(chartInstance, chartInstance->c2_d_r_rgb2, chartInstance->c2_dv3);
  for (c2_i61 = 0; c2_i61 < 1228800; c2_i61++) {
    chartInstance->c2_r_rgb2[c2_i61] = chartInstance->c2_dv3[c2_i61];
  }

  _SFD_SYMBOL_SWITCH(8U, 9U);
  _SFD_EML_CALL(0U, chartInstance->c2_sfEvent, 32);
  for (c2_i62 = 0; c2_i62 < 1228800; c2_i62++) {
    chartInstance->c2_l_bw[c2_i62] = (chartInstance->c2_l_rgb2[c2_i62] > 0.6);
  }

  _SFD_EML_CALL(0U, chartInstance->c2_sfEvent, 33);
  for (c2_i63 = 0; c2_i63 < 1228800; c2_i63++) {
    chartInstance->c2_r_bw[c2_i63] = (chartInstance->c2_r_rgb2[c2_i63] > 0.6);
  }

  _SFD_EML_CALL(0U, chartInstance->c2_sfEvent, 36);
  c2_BlobAnalysis_BlobAnalysis(chartInstance, &chartInstance->c2_hblob);
  _SFD_EML_CALL(0U, chartInstance->c2_sfEvent, 38);
  for (c2_i64 = 0; c2_i64 < 1228800; c2_i64++) {
    chartInstance->c2_b_l_bw[c2_i64] = chartInstance->c2_l_bw[c2_i64];
  }

  c2_SystemCore_step(chartInstance, &chartInstance->c2_hblob,
                     chartInstance->c2_b_l_bw, c2_unusedU0_data,
                     c2_unusedU0_sizes, c2_b_l_center_data, c2_b_l_center_sizes,
                     c2_unusedU1_data, c2_unusedU1_sizes);
  c2_l_center_sizes[0] = c2_b_l_center_sizes[0];
  c2_l_center_sizes[1] = c2_b_l_center_sizes[1];
  c2_l_center = c2_l_center_sizes[0];
  c2_b_l_center = c2_l_center_sizes[1];
  c2_loop_ub = c2_b_l_center_sizes[0] * c2_b_l_center_sizes[1] - 1;
  for (c2_i65 = 0; c2_i65 <= c2_loop_ub; c2_i65++) {
    c2_l_center_data[c2_i65] = c2_b_l_center_data[c2_i65];
  }

  _SFD_EML_CALL(0U, chartInstance->c2_sfEvent, 38);
  _SFD_EML_CALL(0U, chartInstance->c2_sfEvent, 38);
  _SFD_EML_CALL(0U, chartInstance->c2_sfEvent, 39);
  for (c2_i66 = 0; c2_i66 < 1228800; c2_i66++) {
    chartInstance->c2_b_r_bw[c2_i66] = chartInstance->c2_r_bw[c2_i66];
  }

  c2_b_SystemCore_step(chartInstance, &chartInstance->c2_hblob,
                       chartInstance->c2_b_r_bw, c2_unusedU0_data,
                       c2_unusedU0_sizes, c2_b_l_center_data,
                       c2_b_l_center_sizes, c2_unusedU1_data, c2_unusedU1_sizes);
  c2_r_center_sizes[0] = c2_b_l_center_sizes[0];
  c2_r_center_sizes[1] = c2_b_l_center_sizes[1];
  c2_r_center = c2_r_center_sizes[0];
  c2_b_r_center = c2_r_center_sizes[1];
  c2_b_loop_ub = c2_b_l_center_sizes[0] * c2_b_l_center_sizes[1] - 1;
  for (c2_i67 = 0; c2_i67 <= c2_b_loop_ub; c2_i67++) {
    c2_r_center_data[c2_i67] = c2_b_l_center_data[c2_i67];
  }

  _SFD_EML_CALL(0U, chartInstance->c2_sfEvent, 39);
  _SFD_EML_CALL(0U, chartInstance->c2_sfEvent, 39);
  _SFD_EML_CALL(0U, chartInstance->c2_sfEvent, 42);
  c2_ml = (real_T)c2_l_center_sizes[0];
  _SFD_EML_CALL(0U, chartInstance->c2_sfEvent, 43);
  if (CV_EML_IF(0, 1, 0, CV_RELATIONAL_EVAL(4U, 0U, 0, c2_ml, 4.0, -1, 4U, c2_ml
        > 4.0))) {
    _SFD_EML_CALL(0U, chartInstance->c2_sfEvent, 44);
    c2_ml = 4.0;
  }

  _SFD_EML_CALL(0U, chartInstance->c2_sfEvent, 46);
  c2_b_ml = c2_ml;
  c2_i68 = (int32_T)c2_b_ml - 1;
  c2_i = 1.0;
  c2_b_i = 0;
  while (c2_b_i <= c2_i68) {
    c2_i = 1.0 + (real_T)c2_b_i;
    CV_EML_FOR(0, 1, 0, 1);
    _SFD_EML_CALL(0U, chartInstance->c2_sfEvent, 47);
    c2_i69 = (int32_T)c2_i;
    c2_i70 = c2_l_center_sizes[1];
    c2_c_i = _SFD_EML_ARRAY_BOUNDS_CHECK("l_center", (int32_T)c2_i, 1,
      c2_l_center_sizes[0], 1, 0) - 1;
    c2_tmp_sizes[0] = 1;
    c2_tmp_sizes[1] = c2_i70;
    c2_c_loop_ub = c2_i70 - 1;
    for (c2_i71 = 0; c2_i71 <= c2_c_loop_ub; c2_i71++) {
      c2_tmp_data[c2_tmp_sizes[0] * c2_i71] = c2_l_center_data[c2_c_i +
        c2_l_center_sizes[0] * c2_i71];
    }

    _SFD_SUB_ASSIGN_SIZE_CHECK_ND(c2_iv0, 1, c2_tmp_sizes, 2);
    c2_i72 = c2_i69 - 1;
    for (c2_i73 = 0; c2_i73 < 2; c2_i73++) {
      c2_b_L_p[c2_i73 + (c2_i72 << 1)] = c2_tmp_data[c2_i73];
    }

    c2_b_i++;
    _SF_MEX_LISTEN_FOR_CTRL_C(chartInstance->S);
  }

  CV_EML_FOR(0, 1, 0, 0);
  _SFD_EML_CALL(0U, chartInstance->c2_sfEvent, 51);
  c2_mr = (real_T)c2_r_center_sizes[0];
  _SFD_EML_CALL(0U, chartInstance->c2_sfEvent, 52);
  if (CV_EML_IF(0, 1, 1, CV_RELATIONAL_EVAL(4U, 0U, 1, c2_mr, 4.0, -1, 4U, c2_mr
        > 4.0))) {
    _SFD_EML_CALL(0U, chartInstance->c2_sfEvent, 53);
    c2_mr = 4.0;
  }

  _SFD_EML_CALL(0U, chartInstance->c2_sfEvent, 55);
  c2_b_mr = c2_mr;
  c2_i74 = (int32_T)c2_b_mr - 1;
  c2_i = 1.0;
  c2_d_i = 0;
  while (c2_d_i <= c2_i74) {
    c2_i = 1.0 + (real_T)c2_d_i;
    CV_EML_FOR(0, 1, 1, 1);
    _SFD_EML_CALL(0U, chartInstance->c2_sfEvent, 56);
    c2_i75 = (int32_T)c2_i;
    c2_i76 = c2_r_center_sizes[1];
    c2_e_i = _SFD_EML_ARRAY_BOUNDS_CHECK("r_center", (int32_T)c2_i, 1,
      c2_r_center_sizes[0], 1, 0) - 1;
    c2_tmp_sizes[0] = 1;
    c2_tmp_sizes[1] = c2_i76;
    c2_d_loop_ub = c2_i76 - 1;
    for (c2_i77 = 0; c2_i77 <= c2_d_loop_ub; c2_i77++) {
      c2_tmp_data[c2_tmp_sizes[0] * c2_i77] = c2_r_center_data[c2_e_i +
        c2_r_center_sizes[0] * c2_i77];
    }

    _SFD_SUB_ASSIGN_SIZE_CHECK_ND(c2_iv0, 1, c2_tmp_sizes, 2);
    c2_i78 = c2_i75 - 1;
    for (c2_i79 = 0; c2_i79 < 2; c2_i79++) {
      c2_b_R_p[c2_i79 + (c2_i78 << 1)] = c2_tmp_data[c2_i79];
    }

    c2_d_i++;
    _SF_MEX_LISTEN_FOR_CTRL_C(chartInstance->S);
  }

  CV_EML_FOR(0, 1, 1, 0);
  _SFD_EML_CALL(0U, chartInstance->c2_sfEvent, 59);
  for (c2_i80 = 0; c2_i80 < 8; c2_i80++) {
    c2_c_L_p[c2_i80] = c2_b_L_p[c2_i80];
  }

  c2_orderPoints(chartInstance, c2_c_L_p, c2_dv4);
  for (c2_i81 = 0; c2_i81 < 8; c2_i81++) {
    c2_b_L_p[c2_i81] = c2_dv4[c2_i81];
  }

  _SFD_EML_CALL(0U, chartInstance->c2_sfEvent, 60);
  for (c2_i82 = 0; c2_i82 < 8; c2_i82++) {
    c2_c_R_p[c2_i82] = c2_b_R_p[c2_i82];
  }

  c2_orderPoints(chartInstance, c2_c_R_p, c2_dv5);
  for (c2_i83 = 0; c2_i83 < 8; c2_i83++) {
    c2_b_R_p[c2_i83] = c2_dv5[c2_i83];
  }

  _SFD_EML_CALL(0U, chartInstance->c2_sfEvent, 66);
  c2_i84 = 0;
  for (c2_i85 = 0; c2_i85 < 4; c2_i85++) {
    c2_b_L_p[c2_i84 + 1] = 960.0 - c2_b_L_p[c2_i84 + 1];
    c2_i84 += 2;
  }

  _SFD_EML_CALL(0U, chartInstance->c2_sfEvent, 67);
  c2_i86 = 0;
  for (c2_i87 = 0; c2_i87 < 4; c2_i87++) {
    c2_b_R_p[c2_i86 + 1] = 960.0 - c2_b_R_p[c2_i86 + 1];
    c2_i86 += 2;
  }

  _SFD_EML_CALL(0U, chartInstance->c2_sfEvent, -67);
  _SFD_SYMBOL_SCOPE_POP();
  for (c2_i88 = 0; c2_i88 < 8; c2_i88++) {
    (*chartInstance->c2_L_p)[c2_i88] = c2_b_L_p[c2_i88];
  }

  for (c2_i89 = 0; c2_i89 < 8; c2_i89++) {
    (*chartInstance->c2_R_p)[c2_i89] = c2_b_R_p[c2_i89];
  }

  _SFD_CC_CALL(EXIT_OUT_OF_FUNCTION_TAG, 1U, chartInstance->c2_sfEvent);
}

static void initSimStructsc2_mirageTrackingRobot
  (SFc2_mirageTrackingRobotInstanceStruct *chartInstance)
{
  (void)chartInstance;
}

static void c2_orderPoints(SFc2_mirageTrackingRobotInstanceStruct *chartInstance,
  real_T c2_p[8], real_T c2_new_p[8])
{
  uint32_T c2_debug_family_var_map[10];
  real_T c2_indx[4];
  real_T c2_indy[4];
  real_T c2_ind4;
  real_T c2_ind3;
  real_T c2_ind1;
  real_T c2_ind2;
  real_T c2_nargin = 1.0;
  real_T c2_nargout = 1.0;
  int32_T c2_i90;
  int32_T c2_i91;
  real_T c2_x[4];
  int32_T c2_iidx[4];
  int32_T c2_i92;
  int32_T c2_i93;
  int32_T c2_i94;
  int32_T c2_i95;
  int32_T c2_i96;
  int32_T c2_i97;
  real_T c2_b_ind1[4];
  int32_T c2_i98;
  int32_T c2_i99;
  _SFD_SYMBOL_SCOPE_PUSH_EML(0U, 10U, 10U, c2_c_debug_family_names,
    c2_debug_family_var_map);
  _SFD_SYMBOL_SCOPE_ADD_EML_IMPORTABLE(c2_indx, 0U, c2_i_sf_marshallOut,
    c2_i_sf_marshallIn);
  _SFD_SYMBOL_SCOPE_ADD_EML_IMPORTABLE(c2_indy, 1U, c2_i_sf_marshallOut,
    c2_i_sf_marshallIn);
  _SFD_SYMBOL_SCOPE_ADD_EML_IMPORTABLE(&c2_ind4, 2U, c2_c_sf_marshallOut,
    c2_b_sf_marshallIn);
  _SFD_SYMBOL_SCOPE_ADD_EML_IMPORTABLE(&c2_ind3, 3U, c2_c_sf_marshallOut,
    c2_b_sf_marshallIn);
  _SFD_SYMBOL_SCOPE_ADD_EML_IMPORTABLE(&c2_ind1, 4U, c2_c_sf_marshallOut,
    c2_b_sf_marshallIn);
  _SFD_SYMBOL_SCOPE_ADD_EML_IMPORTABLE(&c2_ind2, 5U, c2_c_sf_marshallOut,
    c2_b_sf_marshallIn);
  _SFD_SYMBOL_SCOPE_ADD_EML_IMPORTABLE(&c2_nargin, 6U, c2_c_sf_marshallOut,
    c2_b_sf_marshallIn);
  _SFD_SYMBOL_SCOPE_ADD_EML_IMPORTABLE(&c2_nargout, 7U, c2_c_sf_marshallOut,
    c2_b_sf_marshallIn);
  _SFD_SYMBOL_SCOPE_ADD_EML_IMPORTABLE(c2_p, 8U, c2_sf_marshallOut,
    c2_sf_marshallIn);
  _SFD_SYMBOL_SCOPE_ADD_EML_IMPORTABLE(c2_new_p, 9U, c2_sf_marshallOut,
    c2_sf_marshallIn);
  CV_SCRIPT_FCN(0, 0);
  _SFD_SCRIPT_CALL(0U, chartInstance->c2_sfEvent, 3);
  c2_i90 = 0;
  for (c2_i91 = 0; c2_i91 < 4; c2_i91++) {
    c2_x[c2_i91] = c2_p[c2_i90];
    c2_i90 += 2;
  }

  c2_b_eml_sort(chartInstance, c2_x, c2_iidx);
  for (c2_i92 = 0; c2_i92 < 4; c2_i92++) {
    c2_x[c2_i92] = (real_T)c2_iidx[c2_i92];
  }

  for (c2_i93 = 0; c2_i93 < 4; c2_i93++) {
    c2_indx[c2_i93] = c2_x[c2_i93];
  }

  _SFD_SCRIPT_CALL(0U, chartInstance->c2_sfEvent, 3);
  _SFD_SCRIPT_CALL(0U, chartInstance->c2_sfEvent, 4);
  c2_i94 = 0;
  for (c2_i95 = 0; c2_i95 < 4; c2_i95++) {
    c2_x[c2_i95] = 480.0 - c2_p[c2_i94 + 1];
    c2_i94 += 2;
  }

  c2_b_eml_sort(chartInstance, c2_x, c2_iidx);
  for (c2_i96 = 0; c2_i96 < 4; c2_i96++) {
    c2_x[c2_i96] = (real_T)c2_iidx[c2_i96];
  }

  for (c2_i97 = 0; c2_i97 < 4; c2_i97++) {
    c2_indy[c2_i97] = c2_x[c2_i97];
  }

  _SFD_SCRIPT_CALL(0U, chartInstance->c2_sfEvent, 4);
  _SFD_SCRIPT_CALL(0U, chartInstance->c2_sfEvent, 6);
  c2_ind4 = c2_indy[3];
  _SFD_SCRIPT_CALL(0U, chartInstance->c2_sfEvent, 7);
  c2_ind3 = c2_indy[2];
  _SFD_SCRIPT_CALL(0U, chartInstance->c2_sfEvent, 9);
  c2_ind1 = c2_indx[0];
  _SFD_SCRIPT_CALL(0U, chartInstance->c2_sfEvent, 10);
  c2_ind2 = c2_indx[3];
  _SFD_SCRIPT_CALL(0U, chartInstance->c2_sfEvent, 12);
  c2_b_ind1[0] = c2_ind1;
  c2_b_ind1[1] = c2_ind2;
  c2_b_ind1[2] = c2_ind3;
  c2_b_ind1[3] = c2_ind4;
  for (c2_i98 = 0; c2_i98 < 4; c2_i98++) {
    for (c2_i99 = 0; c2_i99 < 2; c2_i99++) {
      c2_new_p[c2_i99 + (c2_i98 << 1)] = c2_p[c2_i99 +
        ((_SFD_EML_ARRAY_BOUNDS_CHECK("p", (int32_T)c2_b_ind1[c2_i98], 1, 4, 2,
           0) - 1) << 1)];
    }
  }

  _SFD_SCRIPT_CALL(0U, chartInstance->c2_sfEvent, -12);
  _SFD_SYMBOL_SCOPE_POP();
}

static void init_script_number_translation(uint32_T c2_machineNumber, uint32_T
  c2_chartNumber, uint32_T c2_instanceNumber)
{
  (void)c2_machineNumber;
  _SFD_SCRIPT_TRANSLATION(c2_chartNumber, c2_instanceNumber, 0U,
    sf_debug_get_script_id(
    "C:\\Users\\sd0016\\Desktop\\Tracking Target Machine\\orderPoints.m"));
}

static void c2_Outputs(SFc2_mirageTrackingRobotInstanceStruct *chartInstance,
  c2_vision_BlobAnalysis_1 *c2_obj, boolean_T c2_U0[1228800], int32_T
  c2_Y0_data[], int32_T c2_Y0_sizes[2], real_T c2_Y1_data[], int32_T
  c2_Y1_sizes[2], int32_T c2_Y2_data[], int32_T c2_Y2_sizes[2])
{
  boolean_T c2_maxNumBlobsReached;
  int32_T c2_loop;
  uint8_T c2_currentLabel;
  int32_T c2_i;
  int32_T c2_idx;
  int32_T c2_n;
  int32_T c2_m;
  int32_T c2_b_loop;
  int32_T c2_nn;
  uint32_T c2_stackIdx;
  uint32_T c2_pixIdx;
  int32_T c2_b_n;
  int32_T c2_mm;
  int32_T c2_nnPadRows;
  int32_T c2_b_m;
  uint32_T c2_padIdx;
  uint32_T c2_start_pixIdx;
  uint32_T c2_centerIdx;
  int32_T c2_b_i;
  uint32_T c2_walkerIdx;
  uint32_T c2_u12;
  uint32_T c2_u13;
  uint32_T c2_numBlobs;
  int32_T c2_pixListMinc;
  int32_T c2_pixListNinc;
  int32_T c2_c_i;
  int32_T c2_ns;
  int32_T c2_ms;
  int32_T c2_j;
  real_T c2_centroid[2];
  uint32_T c2_colOffset;
  int32_T c2_minc;
  int32_T c2_minr;
  int32_T c2_maxc;
  int32_T c2_maxr;
  int32_T c2_j_pixListNinc;
  int32_T c2_j_pixListMinc;
  int32_T c2_bbox[4];
  uint32_T c2_b_colOffset;
  (void)chartInstance;

  /* System object Outputs function: vision.BlobAnalysis */
  c2_Y0_sizes[0] = 50;
  c2_Y0_sizes[1] = 1;
  c2_Y1_sizes[0] = 50;
  c2_Y1_sizes[1] = 2;
  c2_Y2_sizes[0] = 50;
  c2_Y2_sizes[1] = 4;
  c2_maxNumBlobsReached = false;
  for (c2_loop = 0; c2_loop < 963; c2_loop++) {
    c2_obj->W3_PAD_DW[c2_loop] = 0U;
  }

  c2_currentLabel = 1U;
  c2_i = 0;
  c2_idx = 963;
  for (c2_n = 0; c2_n < 1280; c2_n++) {
    for (c2_m = 0; c2_m < 960; c2_m++) {
      if (c2_U0[c2_i]) {
        c2_obj->W3_PAD_DW[c2_idx] = MAX_uint8_T;
      } else {
        c2_obj->W3_PAD_DW[c2_idx] = 0U;
      }

      c2_i++;
      c2_idx++;
    }

    c2_obj->W3_PAD_DW[c2_idx] = 0U;
    c2_idx;
    c2_obj->W3_PAD_DW[c2_idx + 1] = 0U;
    c2_idx += 2;
  }

  for (c2_b_loop = 0; c2_b_loop < 961; c2_b_loop++) {
    c2_obj->W3_PAD_DW[c2_b_loop + c2_idx] = 0U;
  }

  c2_nn = 0;
  c2_stackIdx = 0U;
  c2_pixIdx = 0U;
  c2_b_n = 0;
  while (c2_b_n < 1280) {
    c2_mm = 0;
    c2_nnPadRows = (c2_nn + 1) * 962;
    c2_b_m = 0;
    while (c2_b_m < 960) {
      c2_padIdx = (uint32_T)((c2_nnPadRows + c2_mm) + 1);
      c2_start_pixIdx = c2_pixIdx;
      if (c2_obj->W3_PAD_DW[c2_padIdx] == 255) {
        c2_obj->W3_PAD_DW[c2_padIdx] = c2_currentLabel;
        c2_obj->W0_N_PIXLIST_DW[c2_pixIdx] = (int16_T)c2_nn;
        c2_obj->W1_M_PIXLIST_DW[c2_pixIdx] = (int16_T)c2_mm;
        c2_pixIdx++;
        c2_obj->W2_NUM_PIX_DW[c2_currentLabel - 1] = 1U;
        c2_obj->W4_STACK_DW[c2_stackIdx] = c2_padIdx;
        c2_stackIdx++;
        while (c2_stackIdx != 0U) {
          c2_stackIdx--;
          c2_centerIdx = c2_obj->W4_STACK_DW[c2_stackIdx];
          for (c2_b_i = 0; c2_b_i < 8; c2_b_i++) {
            c2_walkerIdx = c2_centerIdx + (uint32_T)c2_obj->P0_WALKER_RTP[c2_b_i];
            if (c2_obj->W3_PAD_DW[c2_walkerIdx] == 255) {
              c2_obj->W3_PAD_DW[c2_walkerIdx] = c2_currentLabel;
              c2_u12 = 962U;
              if (c2_u12 == 0U) {
                c2_u13 = MAX_uint32_T;
              } else {
                c2_u13 = c2_walkerIdx / c2_u12;
              }

              c2_obj->W0_N_PIXLIST_DW[c2_pixIdx] = (int16_T)((int16_T)c2_u13 - 1);
              c2_obj->W1_M_PIXLIST_DW[c2_pixIdx] = (int16_T)(c2_walkerIdx % 962U
                - 1U);
              c2_pixIdx++;
              c2_obj->W2_NUM_PIX_DW[c2_currentLabel - 1]++;
              c2_obj->W4_STACK_DW[c2_stackIdx] = c2_walkerIdx;
              c2_stackIdx++;
            }
          }
        }

        if ((c2_obj->W2_NUM_PIX_DW[c2_currentLabel - 1] < c2_obj->P1_MINAREA_RTP)
            || (c2_obj->W2_NUM_PIX_DW[c2_currentLabel - 1] >
                c2_obj->P2_MAXAREA_RTP)) {
          c2_currentLabel--;
          c2_pixIdx = c2_start_pixIdx;
        }

        if (c2_currentLabel == 50) {
          c2_maxNumBlobsReached = true;
          c2_b_n = 1280;
          c2_b_m = 960;
        }

        if (c2_b_m < 960) {
          c2_currentLabel++;
        }
      }

      c2_mm++;
      c2_b_m++;
    }

    c2_nn++;
    c2_b_n++;
  }

  if (c2_maxNumBlobsReached) {
    c2_numBlobs = c2_currentLabel;
  } else {
    c2_numBlobs = (uint8_T)((uint32_T)c2_currentLabel - 1U);
  }

  c2_pixListMinc = 0;
  c2_pixListNinc = 0;
  for (c2_c_i = 0; c2_c_i < (int32_T)c2_numBlobs; c2_c_i++) {
    c2_Y0_data[c2_c_i] = (int32_T)c2_obj->W2_NUM_PIX_DW[c2_c_i];
    c2_ns = 0;
    c2_ms = 0;
    for (c2_j = 0; c2_j < (int32_T)c2_obj->W2_NUM_PIX_DW[c2_c_i]; c2_j++) {
      c2_ns += c2_obj->W0_N_PIXLIST_DW[c2_j + c2_pixListNinc];
      c2_ms += c2_obj->W1_M_PIXLIST_DW[c2_j + c2_pixListMinc];
    }

    c2_centroid[0] = (real_T)c2_ms / (real_T)c2_obj->W2_NUM_PIX_DW[c2_c_i];
    c2_centroid[1] = (real_T)c2_ns / (real_T)c2_obj->W2_NUM_PIX_DW[c2_c_i];
    c2_colOffset = c2_numBlobs;
    c2_Y1_data[c2_c_i] = c2_centroid[1] + 1.0;
    c2_Y1_data[c2_colOffset + (uint32_T)c2_c_i] = c2_centroid[0] + 1.0;
    c2_minc = 1280;
    c2_minr = 960;
    c2_maxc = 0;
    c2_maxr = 0;
    for (c2_j = 0; c2_j < (int32_T)c2_obj->W2_NUM_PIX_DW[c2_c_i]; c2_j++) {
      c2_j_pixListNinc = c2_j + c2_pixListNinc;
      if (c2_obj->W0_N_PIXLIST_DW[c2_j_pixListNinc] < c2_minc) {
        c2_minc = c2_obj->W0_N_PIXLIST_DW[c2_j_pixListNinc];
      }

      if (c2_obj->W0_N_PIXLIST_DW[c2_j_pixListNinc] > c2_maxc) {
        c2_maxc = c2_obj->W0_N_PIXLIST_DW[c2_j_pixListNinc];
      }

      c2_j_pixListMinc = c2_j + c2_pixListMinc;
      if (c2_obj->W1_M_PIXLIST_DW[c2_j_pixListMinc] < c2_minr) {
        c2_minr = c2_obj->W1_M_PIXLIST_DW[c2_j_pixListMinc];
      }

      if (c2_obj->W1_M_PIXLIST_DW[c2_j_pixListMinc] > c2_maxr) {
        c2_maxr = c2_obj->W1_M_PIXLIST_DW[c2_j_pixListMinc];
      }
    }

    c2_bbox[0] = c2_minr;
    c2_bbox[1] = c2_minc;
    c2_bbox[2] = (c2_maxr - c2_minr) + 1;
    c2_bbox[3] = (c2_maxc - c2_minc) + 1;
    c2_b_colOffset = c2_numBlobs;
    c2_Y2_data[c2_c_i] = c2_bbox[1] + 1;
    c2_Y2_data[c2_b_colOffset + (uint32_T)c2_c_i] = c2_bbox[0] + 1;
    c2_Y2_data[((int32_T)c2_b_colOffset << 1) + c2_c_i] = c2_bbox[3];
    c2_Y2_data[3 * (int32_T)c2_b_colOffset + c2_c_i] = c2_bbox[2];
    c2_pixListMinc += (int32_T)c2_obj->W2_NUM_PIX_DW[c2_c_i];
    c2_pixListNinc += (int32_T)c2_obj->W2_NUM_PIX_DW[c2_c_i];
  }

  c2_Y0_sizes[0] = (int32_T)c2_numBlobs;
  c2_Y0_sizes[1] = 1;
  c2_Y1_sizes[0] = (int32_T)c2_numBlobs;
  c2_Y1_sizes[1] = 2;
  c2_Y2_sizes[0] = (int32_T)c2_numBlobs;
  c2_Y2_sizes[1] = 4;
}

static const mxArray *c2_sf_marshallOut(void *chartInstanceVoid, void *c2_inData)
{
  const mxArray *c2_mxArrayOutData = NULL;
  int32_T c2_i100;
  int32_T c2_i101;
  int32_T c2_i102;
  real_T c2_f_u[8];
  const mxArray *c2_f_y = NULL;
  SFc2_mirageTrackingRobotInstanceStruct *chartInstance;
  chartInstance = (SFc2_mirageTrackingRobotInstanceStruct *)chartInstanceVoid;
  c2_mxArrayOutData = NULL;
  c2_i100 = 0;
  for (c2_i101 = 0; c2_i101 < 4; c2_i101++) {
    for (c2_i102 = 0; c2_i102 < 2; c2_i102++) {
      c2_f_u[c2_i102 + c2_i100] = (*(real_T (*)[8])c2_inData)[c2_i102 + c2_i100];
    }

    c2_i100 += 2;
  }

  c2_f_y = NULL;
  sf_mex_assign(&c2_f_y, sf_mex_create("y", c2_f_u, 0, 0U, 1U, 0U, 2, 2, 4),
                false);
  sf_mex_assign(&c2_mxArrayOutData, c2_f_y, false);
  return c2_mxArrayOutData;
}

static void c2_emlrt_marshallIn(SFc2_mirageTrackingRobotInstanceStruct
  *chartInstance, const mxArray *c2_b_R_p, const char_T *c2_identifier, real_T
  c2_f_y[8])
{
  emlrtMsgIdentifier c2_thisId;
  c2_thisId.fIdentifier = c2_identifier;
  c2_thisId.fParent = NULL;
  c2_b_emlrt_marshallIn(chartInstance, sf_mex_dup(c2_b_R_p), &c2_thisId, c2_f_y);
  sf_mex_destroy(&c2_b_R_p);
}

static void c2_b_emlrt_marshallIn(SFc2_mirageTrackingRobotInstanceStruct
  *chartInstance, const mxArray *c2_f_u, const emlrtMsgIdentifier *c2_parentId,
  real_T c2_f_y[8])
{
  real_T c2_dv6[8];
  int32_T c2_i103;
  (void)chartInstance;
  sf_mex_import(c2_parentId, sf_mex_dup(c2_f_u), c2_dv6, 1, 0, 0U, 1, 0U, 2, 2,
                4);
  for (c2_i103 = 0; c2_i103 < 8; c2_i103++) {
    c2_f_y[c2_i103] = c2_dv6[c2_i103];
  }

  sf_mex_destroy(&c2_f_u);
}

static void c2_sf_marshallIn(void *chartInstanceVoid, const mxArray
  *c2_mxArrayInData, const char_T *c2_varName, void *c2_outData)
{
  const mxArray *c2_b_R_p;
  const char_T *c2_identifier;
  emlrtMsgIdentifier c2_thisId;
  real_T c2_f_y[8];
  int32_T c2_i104;
  int32_T c2_i105;
  int32_T c2_i106;
  SFc2_mirageTrackingRobotInstanceStruct *chartInstance;
  chartInstance = (SFc2_mirageTrackingRobotInstanceStruct *)chartInstanceVoid;
  c2_b_R_p = sf_mex_dup(c2_mxArrayInData);
  c2_identifier = c2_varName;
  c2_thisId.fIdentifier = c2_identifier;
  c2_thisId.fParent = NULL;
  c2_b_emlrt_marshallIn(chartInstance, sf_mex_dup(c2_b_R_p), &c2_thisId, c2_f_y);
  sf_mex_destroy(&c2_b_R_p);
  c2_i104 = 0;
  for (c2_i105 = 0; c2_i105 < 4; c2_i105++) {
    for (c2_i106 = 0; c2_i106 < 2; c2_i106++) {
      (*(real_T (*)[8])c2_outData)[c2_i106 + c2_i104] = c2_f_y[c2_i106 + c2_i104];
    }

    c2_i104 += 2;
  }

  sf_mex_destroy(&c2_mxArrayInData);
}

static const mxArray *c2_b_sf_marshallOut(void *chartInstanceVoid, void
  *c2_inData)
{
  const mxArray *c2_mxArrayOutData = NULL;
  int32_T c2_i107;
  int32_T c2_i108;
  int32_T c2_i109;
  int32_T c2_i110;
  int32_T c2_i111;
  const mxArray *c2_f_y = NULL;
  SFc2_mirageTrackingRobotInstanceStruct *chartInstance;
  chartInstance = (SFc2_mirageTrackingRobotInstanceStruct *)chartInstanceVoid;
  c2_mxArrayOutData = NULL;
  c2_i107 = 0;
  for (c2_i108 = 0; c2_i108 < 3; c2_i108++) {
    c2_i109 = 0;
    for (c2_i110 = 0; c2_i110 < 1280; c2_i110++) {
      for (c2_i111 = 0; c2_i111 < 960; c2_i111++) {
        chartInstance->c2_c_u[(c2_i111 + c2_i109) + c2_i107] = (*(uint8_T (*)
          [3686400])c2_inData)[(c2_i111 + c2_i109) + c2_i107];
      }

      c2_i109 += 960;
    }

    c2_i107 += 1228800;
  }

  c2_f_y = NULL;
  sf_mex_assign(&c2_f_y, sf_mex_create("y", chartInstance->c2_c_u, 3, 0U, 1U, 0U,
    3, 960, 1280, 3), false);
  sf_mex_assign(&c2_mxArrayOutData, c2_f_y, false);
  return c2_mxArrayOutData;
}

static const mxArray *c2_c_sf_marshallOut(void *chartInstanceVoid, void
  *c2_inData)
{
  const mxArray *c2_mxArrayOutData = NULL;
  real_T c2_f_u;
  const mxArray *c2_f_y = NULL;
  SFc2_mirageTrackingRobotInstanceStruct *chartInstance;
  chartInstance = (SFc2_mirageTrackingRobotInstanceStruct *)chartInstanceVoid;
  c2_mxArrayOutData = NULL;
  c2_f_u = *(real_T *)c2_inData;
  c2_f_y = NULL;
  sf_mex_assign(&c2_f_y, sf_mex_create("y", &c2_f_u, 0, 0U, 0U, 0U, 0), false);
  sf_mex_assign(&c2_mxArrayOutData, c2_f_y, false);
  return c2_mxArrayOutData;
}

static real_T c2_c_emlrt_marshallIn(SFc2_mirageTrackingRobotInstanceStruct
  *chartInstance, const mxArray *c2_f_u, const emlrtMsgIdentifier *c2_parentId)
{
  real_T c2_f_y;
  real_T c2_d6;
  (void)chartInstance;
  sf_mex_import(c2_parentId, sf_mex_dup(c2_f_u), &c2_d6, 1, 0, 0U, 0, 0U, 0);
  c2_f_y = c2_d6;
  sf_mex_destroy(&c2_f_u);
  return c2_f_y;
}

static void c2_b_sf_marshallIn(void *chartInstanceVoid, const mxArray
  *c2_mxArrayInData, const char_T *c2_varName, void *c2_outData)
{
  const mxArray *c2_nargout;
  const char_T *c2_identifier;
  emlrtMsgIdentifier c2_thisId;
  real_T c2_f_y;
  SFc2_mirageTrackingRobotInstanceStruct *chartInstance;
  chartInstance = (SFc2_mirageTrackingRobotInstanceStruct *)chartInstanceVoid;
  c2_nargout = sf_mex_dup(c2_mxArrayInData);
  c2_identifier = c2_varName;
  c2_thisId.fIdentifier = c2_identifier;
  c2_thisId.fParent = NULL;
  c2_f_y = c2_c_emlrt_marshallIn(chartInstance, sf_mex_dup(c2_nargout),
    &c2_thisId);
  sf_mex_destroy(&c2_nargout);
  *(real_T *)c2_outData = c2_f_y;
  sf_mex_destroy(&c2_mxArrayInData);
}

static const mxArray *c2_d_sf_marshallOut(void *chartInstanceVoid, void
  *c2_inData)
{
  const mxArray *c2_mxArrayOutData = NULL;
  int32_T c2_i112;
  int32_T c2_i113;
  int32_T c2_i114;
  const mxArray *c2_f_y = NULL;
  SFc2_mirageTrackingRobotInstanceStruct *chartInstance;
  chartInstance = (SFc2_mirageTrackingRobotInstanceStruct *)chartInstanceVoid;
  c2_mxArrayOutData = NULL;
  c2_i112 = 0;
  for (c2_i113 = 0; c2_i113 < 1280; c2_i113++) {
    for (c2_i114 = 0; c2_i114 < 960; c2_i114++) {
      chartInstance->c2_e_u[c2_i114 + c2_i112] = (*(uint8_T (*)[1228800])
        c2_inData)[c2_i114 + c2_i112];
    }

    c2_i112 += 960;
  }

  c2_f_y = NULL;
  sf_mex_assign(&c2_f_y, sf_mex_create("y", chartInstance->c2_e_u, 3, 0U, 1U, 0U,
    2, 960, 1280), false);
  sf_mex_assign(&c2_mxArrayOutData, c2_f_y, false);
  return c2_mxArrayOutData;
}

static void c2_d_emlrt_marshallIn(SFc2_mirageTrackingRobotInstanceStruct
  *chartInstance, const mxArray *c2_f_u, const emlrtMsgIdentifier *c2_parentId,
  uint8_T c2_f_y[1228800])
{
  int32_T c2_i115;
  sf_mex_import(c2_parentId, sf_mex_dup(c2_f_u), chartInstance->c2_uv3, 1, 3, 0U,
                1, 0U, 2, 960, 1280);
  for (c2_i115 = 0; c2_i115 < 1228800; c2_i115++) {
    c2_f_y[c2_i115] = chartInstance->c2_uv3[c2_i115];
  }

  sf_mex_destroy(&c2_f_u);
}

static void c2_c_sf_marshallIn(void *chartInstanceVoid, const mxArray
  *c2_mxArrayInData, const char_T *c2_varName, void *c2_outData)
{
  const mxArray *c2_e_r_rgb2;
  const char_T *c2_identifier;
  emlrtMsgIdentifier c2_thisId;
  int32_T c2_i116;
  int32_T c2_i117;
  int32_T c2_i118;
  SFc2_mirageTrackingRobotInstanceStruct *chartInstance;
  chartInstance = (SFc2_mirageTrackingRobotInstanceStruct *)chartInstanceVoid;
  c2_e_r_rgb2 = sf_mex_dup(c2_mxArrayInData);
  c2_identifier = c2_varName;
  c2_thisId.fIdentifier = c2_identifier;
  c2_thisId.fParent = NULL;
  c2_d_emlrt_marshallIn(chartInstance, sf_mex_dup(c2_e_r_rgb2), &c2_thisId,
                        chartInstance->c2_e_y);
  sf_mex_destroy(&c2_e_r_rgb2);
  c2_i116 = 0;
  for (c2_i117 = 0; c2_i117 < 1280; c2_i117++) {
    for (c2_i118 = 0; c2_i118 < 960; c2_i118++) {
      (*(uint8_T (*)[1228800])c2_outData)[c2_i118 + c2_i116] =
        chartInstance->c2_e_y[c2_i118 + c2_i116];
    }

    c2_i116 += 960;
  }

  sf_mex_destroy(&c2_mxArrayInData);
}

static void c2_e_emlrt_marshallIn(SFc2_mirageTrackingRobotInstanceStruct
  *chartInstance, const mxArray *c2_f_u, const emlrtMsgIdentifier *c2_parentId,
  uint8_T c2_f_y[3686400])
{
  int32_T c2_i119;
  sf_mex_import(c2_parentId, sf_mex_dup(c2_f_u), chartInstance->c2_uv4, 1, 3, 0U,
                1, 0U, 3, 960, 1280, 3);
  for (c2_i119 = 0; c2_i119 < 3686400; c2_i119++) {
    c2_f_y[c2_i119] = chartInstance->c2_uv4[c2_i119];
  }

  sf_mex_destroy(&c2_f_u);
}

static void c2_d_sf_marshallIn(void *chartInstanceVoid, const mxArray
  *c2_mxArrayInData, const char_T *c2_varName, void *c2_outData)
{
  const mxArray *c2_c_r_rgb1;
  const char_T *c2_identifier;
  emlrtMsgIdentifier c2_thisId;
  int32_T c2_i120;
  int32_T c2_i121;
  int32_T c2_i122;
  int32_T c2_i123;
  int32_T c2_i124;
  SFc2_mirageTrackingRobotInstanceStruct *chartInstance;
  chartInstance = (SFc2_mirageTrackingRobotInstanceStruct *)chartInstanceVoid;
  c2_c_r_rgb1 = sf_mex_dup(c2_mxArrayInData);
  c2_identifier = c2_varName;
  c2_thisId.fIdentifier = c2_identifier;
  c2_thisId.fParent = NULL;
  c2_e_emlrt_marshallIn(chartInstance, sf_mex_dup(c2_c_r_rgb1), &c2_thisId,
                        chartInstance->c2_c_y);
  sf_mex_destroy(&c2_c_r_rgb1);
  c2_i120 = 0;
  for (c2_i121 = 0; c2_i121 < 3; c2_i121++) {
    c2_i122 = 0;
    for (c2_i123 = 0; c2_i123 < 1280; c2_i123++) {
      for (c2_i124 = 0; c2_i124 < 960; c2_i124++) {
        (*(uint8_T (*)[3686400])c2_outData)[(c2_i124 + c2_i122) + c2_i120] =
          chartInstance->c2_c_y[(c2_i124 + c2_i122) + c2_i120];
      }

      c2_i122 += 960;
    }

    c2_i120 += 1228800;
  }

  sf_mex_destroy(&c2_mxArrayInData);
}

static const mxArray *c2_e_sf_marshallOut(void *chartInstanceVoid, real_T
  c2_inData_data[], int32_T c2_inData_sizes[2])
{
  const mxArray *c2_mxArrayOutData = NULL;
  int32_T c2_u_sizes[2];
  int32_T c2_f_u;
  int32_T c2_g_u;
  int32_T c2_loop_ub;
  int32_T c2_i125;
  real_T c2_u_data[100];
  const mxArray *c2_f_y = NULL;
  SFc2_mirageTrackingRobotInstanceStruct *chartInstance;
  chartInstance = (SFc2_mirageTrackingRobotInstanceStruct *)chartInstanceVoid;
  c2_mxArrayOutData = NULL;
  c2_u_sizes[0] = c2_inData_sizes[0];
  c2_u_sizes[1] = c2_inData_sizes[1];
  c2_f_u = c2_u_sizes[0];
  c2_g_u = c2_u_sizes[1];
  c2_loop_ub = c2_inData_sizes[0] * c2_inData_sizes[1] - 1;
  for (c2_i125 = 0; c2_i125 <= c2_loop_ub; c2_i125++) {
    c2_u_data[c2_i125] = c2_inData_data[c2_i125];
  }

  c2_f_y = NULL;
  sf_mex_assign(&c2_f_y, sf_mex_create("y", c2_u_data, 0, 0U, 1U, 0U, 2,
    c2_u_sizes[0], c2_u_sizes[1]), false);
  sf_mex_assign(&c2_mxArrayOutData, c2_f_y, false);
  return c2_mxArrayOutData;
}

static void c2_f_emlrt_marshallIn(SFc2_mirageTrackingRobotInstanceStruct
  *chartInstance, const mxArray *c2_f_u, const emlrtMsgIdentifier *c2_parentId,
  real_T c2_y_data[], int32_T c2_y_sizes[2])
{
  int32_T c2_i126;
  uint32_T c2_uv5[2];
  int32_T c2_i127;
  boolean_T c2_bv0[2];
  int32_T c2_tmp_sizes[2];
  real_T c2_tmp_data[100];
  int32_T c2_f_y;
  int32_T c2_g_y;
  int32_T c2_loop_ub;
  int32_T c2_i128;
  (void)chartInstance;
  for (c2_i126 = 0; c2_i126 < 2; c2_i126++) {
    c2_uv5[c2_i126] = 50U + (uint32_T)(-48 * c2_i126);
  }

  for (c2_i127 = 0; c2_i127 < 2; c2_i127++) {
    c2_bv0[c2_i127] = true;
  }

  sf_mex_import_vs(c2_parentId, sf_mex_dup(c2_f_u), c2_tmp_data, 1, 0, 0U, 1, 0U,
                   2, c2_bv0, c2_uv5, c2_tmp_sizes);
  c2_y_sizes[0] = c2_tmp_sizes[0];
  c2_y_sizes[1] = c2_tmp_sizes[1];
  c2_f_y = c2_y_sizes[0];
  c2_g_y = c2_y_sizes[1];
  c2_loop_ub = c2_tmp_sizes[0] * c2_tmp_sizes[1] - 1;
  for (c2_i128 = 0; c2_i128 <= c2_loop_ub; c2_i128++) {
    c2_y_data[c2_i128] = c2_tmp_data[c2_i128];
  }

  sf_mex_destroy(&c2_f_u);
}

static void c2_e_sf_marshallIn(void *chartInstanceVoid, const mxArray
  *c2_mxArrayInData, const char_T *c2_varName, real_T c2_outData_data[], int32_T
  c2_outData_sizes[2])
{
  const mxArray *c2_r_center;
  const char_T *c2_identifier;
  emlrtMsgIdentifier c2_thisId;
  int32_T c2_y_sizes[2];
  real_T c2_y_data[100];
  int32_T c2_loop_ub;
  int32_T c2_i129;
  int32_T c2_b_loop_ub;
  int32_T c2_i130;
  SFc2_mirageTrackingRobotInstanceStruct *chartInstance;
  chartInstance = (SFc2_mirageTrackingRobotInstanceStruct *)chartInstanceVoid;
  c2_r_center = sf_mex_dup(c2_mxArrayInData);
  c2_identifier = c2_varName;
  c2_thisId.fIdentifier = c2_identifier;
  c2_thisId.fParent = NULL;
  c2_f_emlrt_marshallIn(chartInstance, sf_mex_dup(c2_r_center), &c2_thisId,
                        c2_y_data, c2_y_sizes);
  sf_mex_destroy(&c2_r_center);
  c2_outData_sizes[0] = c2_y_sizes[0];
  c2_outData_sizes[1] = c2_y_sizes[1];
  c2_loop_ub = c2_y_sizes[1] - 1;
  for (c2_i129 = 0; c2_i129 <= c2_loop_ub; c2_i129++) {
    c2_b_loop_ub = c2_y_sizes[0] - 1;
    for (c2_i130 = 0; c2_i130 <= c2_b_loop_ub; c2_i130++) {
      c2_outData_data[c2_i130 + c2_outData_sizes[0] * c2_i129] =
        c2_y_data[c2_i130 + c2_y_sizes[0] * c2_i129];
    }
  }

  sf_mex_destroy(&c2_mxArrayInData);
}

static const mxArray *c2_f_sf_marshallOut(void *chartInstanceVoid, void
  *c2_inData)
{
  const mxArray *c2_mxArrayOutData = NULL;
  int32_T c2_i131;
  int32_T c2_i132;
  int32_T c2_i133;
  const mxArray *c2_f_y = NULL;
  SFc2_mirageTrackingRobotInstanceStruct *chartInstance;
  chartInstance = (SFc2_mirageTrackingRobotInstanceStruct *)chartInstanceVoid;
  c2_mxArrayOutData = NULL;
  c2_i131 = 0;
  for (c2_i132 = 0; c2_i132 < 1280; c2_i132++) {
    for (c2_i133 = 0; c2_i133 < 960; c2_i133++) {
      chartInstance->c2_d_u[c2_i133 + c2_i131] = (*(boolean_T (*)[1228800])
        c2_inData)[c2_i133 + c2_i131];
    }

    c2_i131 += 960;
  }

  c2_f_y = NULL;
  sf_mex_assign(&c2_f_y, sf_mex_create("y", chartInstance->c2_d_u, 11, 0U, 1U,
    0U, 2, 960, 1280), false);
  sf_mex_assign(&c2_mxArrayOutData, c2_f_y, false);
  return c2_mxArrayOutData;
}

static void c2_g_emlrt_marshallIn(SFc2_mirageTrackingRobotInstanceStruct
  *chartInstance, const mxArray *c2_f_u, const emlrtMsgIdentifier *c2_parentId,
  boolean_T c2_f_y[1228800])
{
  int32_T c2_i134;
  sf_mex_import(c2_parentId, sf_mex_dup(c2_f_u), chartInstance->c2_bv1, 1, 11,
                0U, 1, 0U, 2, 960, 1280);
  for (c2_i134 = 0; c2_i134 < 1228800; c2_i134++) {
    c2_f_y[c2_i134] = chartInstance->c2_bv1[c2_i134];
  }

  sf_mex_destroy(&c2_f_u);
}

static void c2_f_sf_marshallIn(void *chartInstanceVoid, const mxArray
  *c2_mxArrayInData, const char_T *c2_varName, void *c2_outData)
{
  const mxArray *c2_c_r_bw;
  const char_T *c2_identifier;
  emlrtMsgIdentifier c2_thisId;
  int32_T c2_i135;
  int32_T c2_i136;
  int32_T c2_i137;
  SFc2_mirageTrackingRobotInstanceStruct *chartInstance;
  chartInstance = (SFc2_mirageTrackingRobotInstanceStruct *)chartInstanceVoid;
  c2_c_r_bw = sf_mex_dup(c2_mxArrayInData);
  c2_identifier = c2_varName;
  c2_thisId.fIdentifier = c2_identifier;
  c2_thisId.fParent = NULL;
  c2_g_emlrt_marshallIn(chartInstance, sf_mex_dup(c2_c_r_bw), &c2_thisId,
                        chartInstance->c2_d_y);
  sf_mex_destroy(&c2_c_r_bw);
  c2_i135 = 0;
  for (c2_i136 = 0; c2_i136 < 1280; c2_i136++) {
    for (c2_i137 = 0; c2_i137 < 960; c2_i137++) {
      (*(boolean_T (*)[1228800])c2_outData)[c2_i137 + c2_i135] =
        chartInstance->c2_d_y[c2_i137 + c2_i135];
    }

    c2_i135 += 960;
  }

  sf_mex_destroy(&c2_mxArrayInData);
}

static const mxArray *c2_g_sf_marshallOut(void *chartInstanceVoid, void
  *c2_inData)
{
  const mxArray *c2_mxArrayOutData = NULL;
  int32_T c2_i138;
  int32_T c2_i139;
  int32_T c2_i140;
  const mxArray *c2_f_y = NULL;
  SFc2_mirageTrackingRobotInstanceStruct *chartInstance;
  chartInstance = (SFc2_mirageTrackingRobotInstanceStruct *)chartInstanceVoid;
  c2_mxArrayOutData = NULL;
  c2_i138 = 0;
  for (c2_i139 = 0; c2_i139 < 1280; c2_i139++) {
    for (c2_i140 = 0; c2_i140 < 960; c2_i140++) {
      chartInstance->c2_b_u[c2_i140 + c2_i138] = (*(real_T (*)[1228800])
        c2_inData)[c2_i140 + c2_i138];
    }

    c2_i138 += 960;
  }

  c2_f_y = NULL;
  sf_mex_assign(&c2_f_y, sf_mex_create("y", chartInstance->c2_b_u, 0, 0U, 1U, 0U,
    2, 960, 1280), false);
  sf_mex_assign(&c2_mxArrayOutData, c2_f_y, false);
  return c2_mxArrayOutData;
}

static void c2_h_emlrt_marshallIn(SFc2_mirageTrackingRobotInstanceStruct
  *chartInstance, const mxArray *c2_f_u, const emlrtMsgIdentifier *c2_parentId,
  real_T c2_f_y[1228800])
{
  int32_T c2_i141;
  sf_mex_import(c2_parentId, sf_mex_dup(c2_f_u), chartInstance->c2_dv7, 1, 0, 0U,
                1, 0U, 2, 960, 1280);
  for (c2_i141 = 0; c2_i141 < 1228800; c2_i141++) {
    c2_f_y[c2_i141] = chartInstance->c2_dv7[c2_i141];
  }

  sf_mex_destroy(&c2_f_u);
}

static void c2_g_sf_marshallIn(void *chartInstanceVoid, const mxArray
  *c2_mxArrayInData, const char_T *c2_varName, void *c2_outData)
{
  const mxArray *c2_e_r_rgb2;
  const char_T *c2_identifier;
  emlrtMsgIdentifier c2_thisId;
  int32_T c2_i142;
  int32_T c2_i143;
  int32_T c2_i144;
  SFc2_mirageTrackingRobotInstanceStruct *chartInstance;
  chartInstance = (SFc2_mirageTrackingRobotInstanceStruct *)chartInstanceVoid;
  c2_e_r_rgb2 = sf_mex_dup(c2_mxArrayInData);
  c2_identifier = c2_varName;
  c2_thisId.fIdentifier = c2_identifier;
  c2_thisId.fParent = NULL;
  c2_h_emlrt_marshallIn(chartInstance, sf_mex_dup(c2_e_r_rgb2), &c2_thisId,
                        chartInstance->c2_b_y);
  sf_mex_destroy(&c2_e_r_rgb2);
  c2_i142 = 0;
  for (c2_i143 = 0; c2_i143 < 1280; c2_i143++) {
    for (c2_i144 = 0; c2_i144 < 960; c2_i144++) {
      (*(real_T (*)[1228800])c2_outData)[c2_i144 + c2_i142] =
        chartInstance->c2_b_y[c2_i144 + c2_i142];
    }

    c2_i142 += 960;
  }

  sf_mex_destroy(&c2_mxArrayInData);
}

static const mxArray *c2_h_sf_marshallOut(void *chartInstanceVoid, void
  *c2_inData)
{
  const mxArray *c2_mxArrayOutData = NULL;
  int32_T c2_i145;
  int32_T c2_i146;
  int32_T c2_i147;
  int32_T c2_i148;
  int32_T c2_i149;
  const mxArray *c2_f_y = NULL;
  SFc2_mirageTrackingRobotInstanceStruct *chartInstance;
  chartInstance = (SFc2_mirageTrackingRobotInstanceStruct *)chartInstanceVoid;
  c2_mxArrayOutData = NULL;
  c2_i145 = 0;
  for (c2_i146 = 0; c2_i146 < 3; c2_i146++) {
    c2_i147 = 0;
    for (c2_i148 = 0; c2_i148 < 1280; c2_i148++) {
      for (c2_i149 = 0; c2_i149 < 960; c2_i149++) {
        chartInstance->c2_u[(c2_i149 + c2_i147) + c2_i145] = (*(real_T (*)
          [3686400])c2_inData)[(c2_i149 + c2_i147) + c2_i145];
      }

      c2_i147 += 960;
    }

    c2_i145 += 1228800;
  }

  c2_f_y = NULL;
  sf_mex_assign(&c2_f_y, sf_mex_create("y", chartInstance->c2_u, 0, 0U, 1U, 0U,
    3, 960, 1280, 3), false);
  sf_mex_assign(&c2_mxArrayOutData, c2_f_y, false);
  return c2_mxArrayOutData;
}

static void c2_i_emlrt_marshallIn(SFc2_mirageTrackingRobotInstanceStruct
  *chartInstance, const mxArray *c2_f_u, const emlrtMsgIdentifier *c2_parentId,
  real_T c2_f_y[3686400])
{
  int32_T c2_i150;
  sf_mex_import(c2_parentId, sf_mex_dup(c2_f_u), chartInstance->c2_dv8, 1, 0, 0U,
                1, 0U, 3, 960, 1280, 3);
  for (c2_i150 = 0; c2_i150 < 3686400; c2_i150++) {
    c2_f_y[c2_i150] = chartInstance->c2_dv8[c2_i150];
  }

  sf_mex_destroy(&c2_f_u);
}

static void c2_h_sf_marshallIn(void *chartInstanceVoid, const mxArray
  *c2_mxArrayInData, const char_T *c2_varName, void *c2_outData)
{
  const mxArray *c2_c_r_rgb1;
  const char_T *c2_identifier;
  emlrtMsgIdentifier c2_thisId;
  int32_T c2_i151;
  int32_T c2_i152;
  int32_T c2_i153;
  int32_T c2_i154;
  int32_T c2_i155;
  SFc2_mirageTrackingRobotInstanceStruct *chartInstance;
  chartInstance = (SFc2_mirageTrackingRobotInstanceStruct *)chartInstanceVoid;
  c2_c_r_rgb1 = sf_mex_dup(c2_mxArrayInData);
  c2_identifier = c2_varName;
  c2_thisId.fIdentifier = c2_identifier;
  c2_thisId.fParent = NULL;
  c2_i_emlrt_marshallIn(chartInstance, sf_mex_dup(c2_c_r_rgb1), &c2_thisId,
                        chartInstance->c2_y);
  sf_mex_destroy(&c2_c_r_rgb1);
  c2_i151 = 0;
  for (c2_i152 = 0; c2_i152 < 3; c2_i152++) {
    c2_i153 = 0;
    for (c2_i154 = 0; c2_i154 < 1280; c2_i154++) {
      for (c2_i155 = 0; c2_i155 < 960; c2_i155++) {
        (*(real_T (*)[3686400])c2_outData)[(c2_i155 + c2_i153) + c2_i151] =
          chartInstance->c2_y[(c2_i155 + c2_i153) + c2_i151];
      }

      c2_i153 += 960;
    }

    c2_i151 += 1228800;
  }

  sf_mex_destroy(&c2_mxArrayInData);
}

static const mxArray *c2_i_sf_marshallOut(void *chartInstanceVoid, void
  *c2_inData)
{
  const mxArray *c2_mxArrayOutData = NULL;
  int32_T c2_i156;
  real_T c2_f_u[4];
  const mxArray *c2_f_y = NULL;
  SFc2_mirageTrackingRobotInstanceStruct *chartInstance;
  chartInstance = (SFc2_mirageTrackingRobotInstanceStruct *)chartInstanceVoid;
  c2_mxArrayOutData = NULL;
  for (c2_i156 = 0; c2_i156 < 4; c2_i156++) {
    c2_f_u[c2_i156] = (*(real_T (*)[4])c2_inData)[c2_i156];
  }

  c2_f_y = NULL;
  sf_mex_assign(&c2_f_y, sf_mex_create("y", c2_f_u, 0, 0U, 1U, 0U, 2, 1, 4),
                false);
  sf_mex_assign(&c2_mxArrayOutData, c2_f_y, false);
  return c2_mxArrayOutData;
}

static void c2_j_emlrt_marshallIn(SFc2_mirageTrackingRobotInstanceStruct
  *chartInstance, const mxArray *c2_f_u, const emlrtMsgIdentifier *c2_parentId,
  real_T c2_f_y[4])
{
  real_T c2_dv9[4];
  int32_T c2_i157;
  (void)chartInstance;
  sf_mex_import(c2_parentId, sf_mex_dup(c2_f_u), c2_dv9, 1, 0, 0U, 1, 0U, 2, 1,
                4);
  for (c2_i157 = 0; c2_i157 < 4; c2_i157++) {
    c2_f_y[c2_i157] = c2_dv9[c2_i157];
  }

  sf_mex_destroy(&c2_f_u);
}

static void c2_i_sf_marshallIn(void *chartInstanceVoid, const mxArray
  *c2_mxArrayInData, const char_T *c2_varName, void *c2_outData)
{
  const mxArray *c2_indy;
  const char_T *c2_identifier;
  emlrtMsgIdentifier c2_thisId;
  real_T c2_f_y[4];
  int32_T c2_i158;
  SFc2_mirageTrackingRobotInstanceStruct *chartInstance;
  chartInstance = (SFc2_mirageTrackingRobotInstanceStruct *)chartInstanceVoid;
  c2_indy = sf_mex_dup(c2_mxArrayInData);
  c2_identifier = c2_varName;
  c2_thisId.fIdentifier = c2_identifier;
  c2_thisId.fParent = NULL;
  c2_j_emlrt_marshallIn(chartInstance, sf_mex_dup(c2_indy), &c2_thisId, c2_f_y);
  sf_mex_destroy(&c2_indy);
  for (c2_i158 = 0; c2_i158 < 4; c2_i158++) {
    (*(real_T (*)[4])c2_outData)[c2_i158] = c2_f_y[c2_i158];
  }

  sf_mex_destroy(&c2_mxArrayInData);
}

const mxArray *sf_c2_mirageTrackingRobot_get_eml_resolved_functions_info(void)
{
  const mxArray *c2_nameCaptureInfo = NULL;
  c2_nameCaptureInfo = NULL;
  sf_mex_assign(&c2_nameCaptureInfo, sf_mex_createstruct("structure", 2, 1, 1),
                false);
  c2_info_helper(&c2_nameCaptureInfo);
  sf_mex_emlrtNameCapturePostProcessR2012a(&c2_nameCaptureInfo);
  return c2_nameCaptureInfo;
}

static void c2_info_helper(const mxArray **c2_info)
{
  const mxArray *c2_rhs0 = NULL;
  const mxArray *c2_lhs0 = NULL;
  sf_mex_addfield(*c2_info, c2_emlrt_marshallOut(""), "context", "context", 0);
  sf_mex_addfield(*c2_info, c2_emlrt_marshallOut("orderPoints"), "name", "name",
                  0);
  sf_mex_addfield(*c2_info, c2_emlrt_marshallOut("double"), "dominantType",
                  "dominantType", 0);
  sf_mex_addfield(*c2_info, c2_emlrt_marshallOut(
    "[E]C:/Users/sd0016/Desktop/Tracking Target Machine/orderPoints.m"),
                  "resolved", "resolved", 0);
  sf_mex_addfield(*c2_info, c2_b_emlrt_marshallOut(1440081773U), "fileTimeLo",
                  "fileTimeLo", 0);
  sf_mex_addfield(*c2_info, c2_b_emlrt_marshallOut(0U), "fileTimeHi",
                  "fileTimeHi", 0);
  sf_mex_addfield(*c2_info, c2_b_emlrt_marshallOut(0U), "mFileTimeLo",
                  "mFileTimeLo", 0);
  sf_mex_addfield(*c2_info, c2_b_emlrt_marshallOut(0U), "mFileTimeHi",
                  "mFileTimeHi", 0);
  sf_mex_assign(&c2_rhs0, sf_mex_createcellmatrix(0, 1), false);
  sf_mex_assign(&c2_lhs0, sf_mex_createcellmatrix(0, 1), false);
  sf_mex_addfield(*c2_info, sf_mex_duplicatearraysafe(&c2_rhs0), "rhs", "rhs", 0);
  sf_mex_addfield(*c2_info, sf_mex_duplicatearraysafe(&c2_lhs0), "lhs", "lhs", 0);
  sf_mex_destroy(&c2_rhs0);
  sf_mex_destroy(&c2_lhs0);
}

static const mxArray *c2_emlrt_marshallOut(const char * c2_f_u)
{
  const mxArray *c2_f_y = NULL;
  c2_f_y = NULL;
  sf_mex_assign(&c2_f_y, sf_mex_create("y", c2_f_u, 15, 0U, 0U, 0U, 2, 1, strlen
    (c2_f_u)), false);
  return c2_f_y;
}

static const mxArray *c2_b_emlrt_marshallOut(const uint32_T c2_f_u)
{
  const mxArray *c2_f_y = NULL;
  c2_f_y = NULL;
  sf_mex_assign(&c2_f_y, sf_mex_create("y", &c2_f_u, 7, 0U, 0U, 0U, 0), false);
  return c2_f_y;
}

static void c2_imcomplement(SFc2_mirageTrackingRobotInstanceStruct
  *chartInstance, uint8_T c2_im[3686400], uint8_T c2_b_im[3686400])
{
  int32_T c2_i159;
  for (c2_i159 = 0; c2_i159 < 3686400; c2_i159++) {
    c2_b_im[c2_i159] = c2_im[c2_i159];
  }

  c2_b_imcomplement(chartInstance, c2_b_im);
}

static void c2_medfilt2(SFc2_mirageTrackingRobotInstanceStruct *chartInstance,
  uint8_T c2_g_varargin_1[1228800], uint8_T c2_d_b[1228800])
{
  int32_T c2_i160;
  int32_T c2_i161;
  real_T c2_inputSize[2];
  int32_T c2_i162;
  real_T c2_maskSize[2];
  for (c2_i160 = 0; c2_i160 < 1228800; c2_i160++) {
    chartInstance->c2_b_varargin_1[c2_i160] = c2_g_varargin_1[c2_i160];
  }

  c2_padarray(chartInstance, chartInstance->c2_b_varargin_1, chartInstance->c2_a);
  for (c2_i161 = 0; c2_i161 < 2; c2_i161++) {
    c2_inputSize[c2_i161] = 962.0 + 320.0 * (real_T)c2_i161;
  }

  for (c2_i162 = 0; c2_i162 < 2; c2_i162++) {
    c2_maskSize[c2_i162] = 3.0;
  }

  ippMedianFilter_uint8(chartInstance->c2_a, c2_inputSize, c2_maskSize, c2_d_b);
}

static void c2_padarray(SFc2_mirageTrackingRobotInstanceStruct *chartInstance,
  uint8_T c2_g_varargin_1[1228800], uint8_T c2_d_b[1233284])
{
  int32_T c2_i;
  real_T c2_b_i;
  int32_T c2_c_i;
  int32_T c2_j;
  real_T c2_b_j;
  real_T c2_b_a;
  int32_T c2_c;
  int32_T c2_c_j;
  real_T c2_c_a;
  int32_T c2_b_c;
  int32_T c2_d_j;
  int32_T c2_d_i;
  real_T c2_d_a;
  int32_T c2_c_c;
  real_T c2_e_a;
  int32_T c2_d_c;
  c2_cell_cell(chartInstance);
  c2_b_cell_cell(chartInstance);
  c2_c_cell_cell(chartInstance);
  c2_d_cell_cell(chartInstance);
  for (c2_i = 0; c2_i < 962; c2_i++) {
    c2_b_i = 1.0 + (real_T)c2_i;
    c2_d_b[(int32_T)c2_b_i - 1] = 0U;
  }

  for (c2_c_i = 0; c2_c_i < 962; c2_c_i++) {
    c2_b_i = 1.0 + (real_T)c2_c_i;
    c2_d_b[(int32_T)c2_b_i + 1232321] = 0U;
  }

  for (c2_j = 0; c2_j < 1280; c2_j++) {
    c2_b_j = 1.0 + (real_T)c2_j;
    c2_b_a = c2_b_j;
    c2_c = (int32_T)c2_b_a;
    c2_d_b[962 * c2_c] = 0U;
  }

  for (c2_c_j = 0; c2_c_j < 1280; c2_c_j++) {
    c2_b_j = 1.0 + (real_T)c2_c_j;
    c2_c_a = c2_b_j;
    c2_b_c = (int32_T)c2_c_a;
    c2_d_b[961 + 962 * c2_b_c] = 0U;
  }

  for (c2_d_j = 0; c2_d_j < 1280; c2_d_j++) {
    c2_b_j = 1.0 + (real_T)c2_d_j;
    for (c2_d_i = 0; c2_d_i < 960; c2_d_i++) {
      c2_b_i = 1.0 + (real_T)c2_d_i;
      c2_d_a = c2_b_i;
      c2_c_c = (int32_T)c2_d_a;
      c2_e_a = c2_b_j;
      c2_d_c = (int32_T)c2_e_a;
      c2_d_b[(_SFD_EML_ARRAY_BOUNDS_CHECK("", c2_c_c + 1, 1, 962, 1, 0) + 962 *
              c2_d_c) - 1] = c2_g_varargin_1[((int32_T)c2_b_i + 960 * ((int32_T)
        c2_b_j - 1)) - 1];
    }
  }
}

static c2_coder_internal_cell_4 c2_cell_cell
  (SFc2_mirageTrackingRobotInstanceStruct *chartInstance)
{
  c2_coder_internal_cell_4 c2_this;
  (void)chartInstance;
  c2_this.dummy = 0;
  return c2_this;
}

static c2_coder_internal_cell_5 c2_b_cell_cell
  (SFc2_mirageTrackingRobotInstanceStruct *chartInstance)
{
  c2_coder_internal_cell_5 c2_this;
  (void)chartInstance;
  c2_this.dummy = 0;
  return c2_this;
}

static c2_coder_internal_cell_6 c2_c_cell_cell
  (SFc2_mirageTrackingRobotInstanceStruct *chartInstance)
{
  c2_coder_internal_cell_6 c2_this;
  (void)chartInstance;
  c2_this.dummy = 0;
  return c2_this;
}

static c2_coder_internal_cell_7 c2_d_cell_cell
  (SFc2_mirageTrackingRobotInstanceStruct *chartInstance)
{
  c2_coder_internal_cell_7 c2_this;
  (void)chartInstance;
  c2_this.dummy = 0;
  return c2_this;
}

static void c2_myMat2gray(SFc2_mirageTrackingRobotInstanceStruct *chartInstance,
  uint8_T c2_b_imgIn[1228800], real_T c2_imgOut[1228800])
{
  uint32_T c2_debug_family_var_map[7];
  real_T c2_minL;
  real_T c2_maxL;
  real_T c2_nargin = 1.0;
  real_T c2_nargout = 1.0;
  int32_T c2_i163;
  int32_T c2_i164;
  real_T c2_mtmp;
  int32_T c2_ix;
  int32_T c2_b_ix;
  real_T c2_b_a;
  real_T c2_d_b;
  boolean_T c2_p;
  real_T c2_b_mtmp;
  int32_T c2_i165;
  int32_T c2_i166;
  real_T c2_c_mtmp;
  int32_T c2_c_ix;
  int32_T c2_d_ix;
  real_T c2_c_a;
  real_T c2_e_b;
  boolean_T c2_b_p;
  real_T c2_d_mtmp;
  int32_T c2_i167;
  real_T c2_B;
  real_T c2_f_y;
  real_T c2_g_y;
  real_T c2_h_y;
  int32_T c2_i168;
  _SFD_SYMBOL_SCOPE_PUSH_EML(0U, 7U, 8U, c2_b_debug_family_names,
    c2_debug_family_var_map);
  _SFD_SYMBOL_SCOPE_ADD_EML_IMPORTABLE(&c2_minL, 0U, c2_c_sf_marshallOut,
    c2_b_sf_marshallIn);
  _SFD_SYMBOL_SCOPE_ADD_EML_IMPORTABLE(chartInstance->c2_imgTmp, 1U,
    c2_g_sf_marshallOut, c2_g_sf_marshallIn);
  _SFD_SYMBOL_SCOPE_ADD_EML_IMPORTABLE(&c2_maxL, 2U, c2_c_sf_marshallOut,
    c2_b_sf_marshallIn);
  _SFD_SYMBOL_SCOPE_ADD_EML_IMPORTABLE(chartInstance->c2_imgIn, MAX_uint32_T,
    c2_g_sf_marshallOut, c2_g_sf_marshallIn);
  _SFD_SYMBOL_SCOPE_ADD_EML_IMPORTABLE(&c2_nargin, 4U, c2_c_sf_marshallOut,
    c2_b_sf_marshallIn);
  _SFD_SYMBOL_SCOPE_ADD_EML_IMPORTABLE(&c2_nargout, 5U, c2_c_sf_marshallOut,
    c2_b_sf_marshallIn);
  _SFD_SYMBOL_SCOPE_ADD_EML_IMPORTABLE(c2_b_imgIn, 3U, c2_d_sf_marshallOut,
    c2_c_sf_marshallIn);
  _SFD_SYMBOL_SCOPE_ADD_EML_IMPORTABLE(c2_imgOut, 6U, c2_g_sf_marshallOut,
    c2_g_sf_marshallIn);
  CV_EML_FCN(0, 1);
  _SFD_EML_CALL(0U, chartInstance->c2_sfEvent, 72);
  for (c2_i163 = 0; c2_i163 < 1228800; c2_i163++) {
    chartInstance->c2_imgIn[c2_i163] = (real_T)c2_b_imgIn[c2_i163];
  }

  _SFD_SYMBOL_SWITCH(3U, 3U);
  _SFD_EML_CALL(0U, chartInstance->c2_sfEvent, 74);
  for (c2_i164 = 0; c2_i164 < 1228800; c2_i164++) {
    chartInstance->c2_varargin_1[c2_i164] = chartInstance->c2_imgIn[c2_i164];
  }

  c2_mtmp = chartInstance->c2_varargin_1[0];
  for (c2_ix = 2; c2_ix < 1228801; c2_ix++) {
    c2_b_ix = c2_ix - 1;
    c2_b_a = chartInstance->c2_varargin_1[c2_b_ix];
    c2_d_b = c2_mtmp;
    c2_p = (c2_b_a < c2_d_b);
    if (c2_p) {
      c2_mtmp = chartInstance->c2_varargin_1[c2_b_ix];
    }
  }

  c2_b_mtmp = c2_mtmp;
  c2_minL = c2_b_mtmp;
  _SFD_EML_CALL(0U, chartInstance->c2_sfEvent, 76);
  for (c2_i165 = 0; c2_i165 < 1228800; c2_i165++) {
    chartInstance->c2_imgTmp[c2_i165] = chartInstance->c2_imgIn[c2_i165] -
      c2_minL;
  }

  _SFD_EML_CALL(0U, chartInstance->c2_sfEvent, 78);
  for (c2_i166 = 0; c2_i166 < 1228800; c2_i166++) {
    chartInstance->c2_varargin_1[c2_i166] = chartInstance->c2_imgTmp[c2_i166];
  }

  c2_c_mtmp = chartInstance->c2_varargin_1[0];
  for (c2_c_ix = 2; c2_c_ix < 1228801; c2_c_ix++) {
    c2_d_ix = c2_c_ix - 1;
    c2_c_a = chartInstance->c2_varargin_1[c2_d_ix];
    c2_e_b = c2_c_mtmp;
    c2_b_p = (c2_c_a > c2_e_b);
    if (c2_b_p) {
      c2_c_mtmp = chartInstance->c2_varargin_1[c2_d_ix];
    }
  }

  c2_d_mtmp = c2_c_mtmp;
  c2_maxL = c2_d_mtmp;
  _SFD_EML_CALL(0U, chartInstance->c2_sfEvent, 80);
  for (c2_i167 = 0; c2_i167 < 1228800; c2_i167++) {
    chartInstance->c2_A[c2_i167] = chartInstance->c2_imgTmp[c2_i167];
  }

  c2_B = c2_maxL;
  c2_f_y = c2_B;
  c2_g_y = c2_f_y;
  c2_h_y = c2_g_y;
  for (c2_i168 = 0; c2_i168 < 1228800; c2_i168++) {
    c2_imgOut[c2_i168] = chartInstance->c2_A[c2_i168] / c2_h_y;
  }

  _SFD_EML_CALL(0U, chartInstance->c2_sfEvent, -80);
  _SFD_SYMBOL_SCOPE_POP();
}

static void c2_check_forloop_overflow_error
  (SFc2_mirageTrackingRobotInstanceStruct *chartInstance, boolean_T c2_overflow)
{
  const mxArray *c2_f_y = NULL;
  static char_T c2_f_u[34] = { 'C', 'o', 'd', 'e', 'r', ':', 't', 'o', 'o', 'l',
    'b', 'o', 'x', ':', 'i', 'n', 't', '_', 'f', 'o', 'r', 'l', 'o', 'o', 'p',
    '_', 'o', 'v', 'e', 'r', 'f', 'l', 'o', 'w' };

  const mxArray *c2_g_y = NULL;
  static char_T c2_g_u[5] = { 'i', 'n', 't', '3', '2' };

  (void)chartInstance;
  (void)c2_overflow;
  c2_f_y = NULL;
  sf_mex_assign(&c2_f_y, sf_mex_create("y", c2_f_u, 10, 0U, 1U, 0U, 2, 1, 34),
                false);
  c2_g_y = NULL;
  sf_mex_assign(&c2_g_y, sf_mex_create("y", c2_g_u, 10, 0U, 1U, 0U, 2, 1, 5),
                false);
  sf_mex_call_debug(sfGlobalDebugInstanceStruct, "error", 0U, 1U, 14,
                    sf_mex_call_debug(sfGlobalDebugInstanceStruct, "message", 1U,
    2U, 14, c2_f_y, 14, c2_g_y));
}

static void c2_System_System(SFc2_mirageTrackingRobotInstanceStruct
  *chartInstance, c2_visioncodegen_BlobAnalysis *c2_obj)
{
  c2_visioncodegen_BlobAnalysis *c2_b_obj;
  c2_visioncodegen_BlobAnalysis *c2_c_obj;
  c2_visioncodegen_BlobAnalysis *c2_d_obj;
  const mxArray *c2_unusedU1 = NULL;
  const mxArray *c2_unusedU2 = NULL;
  const mxArray *c2_defaultValidateInputsImpl = NULL;
  const mxArray *c2_unusedU3 = NULL;
  const mxArray *c2_unusedU4 = NULL;
  const mxArray *c2_defaultProcessInputSizeChangeImpl = NULL;
  (void)chartInstance;
  c2_b_obj = c2_obj;
  c2_c_obj = c2_b_obj;
  c2_d_obj = c2_c_obj;
  c2_d_obj->isInitialized = 0;
  sf_mex_destroy(&c2_unusedU1);
  sf_mex_destroy(&c2_unusedU2);
  sf_mex_destroy(&c2_defaultValidateInputsImpl);
  sf_mex_destroy(&c2_unusedU3);
  sf_mex_destroy(&c2_unusedU4);
  sf_mex_destroy(&c2_defaultProcessInputSizeChangeImpl);
}

static c2_visioncodegen_BlobAnalysis *c2_BlobAnalysis_BlobAnalysis
  (SFc2_mirageTrackingRobotInstanceStruct *chartInstance,
   c2_visioncodegen_BlobAnalysis *c2_obj)
{
  c2_visioncodegen_BlobAnalysis *c2_b_obj;
  c2_vision_BlobAnalysis_1 *c2_c_obj;
  c2_vision_BlobAnalysis_1 *c2_d_obj;
  int32_T c2_i169;
  static int32_T c2_iv1[8] = { -1, 961, 962, 963, 1, -961, -962, -963 };

  c2_visioncodegen_BlobAnalysis *c2_e_obj;
  const mxArray *c2_f_y = NULL;
  static char_T c2_f_u[42] = { 'M', 'A', 'T', 'L', 'A', 'B', ':', 's', 'y', 's',
    't', 'e', 'm', ':', 'n', 'o', 'T', 'u', 'n', 'i', 'n', 'g', 'B', 'e', 'f',
    'o', 'r', 'e', 'L', 'o', 'c', 'k', 'i', 'n', 'g', 'C', 'o', 'd', 'e', 'G',
    'e', 'n' };

  c2_vision_BlobAnalysis_1 *c2_f_obj;
  c2_visioncodegen_BlobAnalysis *c2_g_obj;
  const mxArray *c2_g_y = NULL;
  c2_vision_BlobAnalysis_1 *c2_h_obj;
  c2_b_obj = c2_obj;
  c2_System_System(chartInstance, c2_b_obj);
  c2_b_obj->NoTuningBeforeLockingCodeGenError = true;
  c2_c_obj = &c2_b_obj->cSFunObject;

  /* System object Constructor function: vision.BlobAnalysis */
  c2_d_obj = c2_c_obj;
  c2_d_obj->S0_isInitialized = false;
  for (c2_i169 = 0; c2_i169 < 8; c2_i169++) {
    c2_d_obj->P0_WALKER_RTP[c2_i169] = c2_iv1[c2_i169];
  }

  c2_d_obj->P1_MINAREA_RTP = 0U;
  c2_d_obj->P2_MAXAREA_RTP = MAX_uint32_T;
  c2_e_obj = c2_b_obj;
  if (c2_e_obj->NoTuningBeforeLockingCodeGenError) {
  } else {
    c2_f_y = NULL;
    sf_mex_assign(&c2_f_y, sf_mex_create("y", c2_f_u, 10, 0U, 1U, 0U, 2, 1, 42),
                  false);
    sf_mex_call_debug(sfGlobalDebugInstanceStruct, "error", 0U, 1U, 14,
                      sf_mex_call_debug(sfGlobalDebugInstanceStruct, "message",
      1U, 1U, 14, c2_f_y));
  }

  c2_f_obj = &c2_e_obj->cSFunObject;
  c2_f_obj->P1_MINAREA_RTP = 0U;
  c2_g_obj = c2_b_obj;
  if (c2_g_obj->NoTuningBeforeLockingCodeGenError) {
  } else {
    c2_g_y = NULL;
    sf_mex_assign(&c2_g_y, sf_mex_create("y", c2_f_u, 10, 0U, 1U, 0U, 2, 1, 42),
                  false);
    sf_mex_call_debug(sfGlobalDebugInstanceStruct, "error", 0U, 1U, 14,
                      sf_mex_call_debug(sfGlobalDebugInstanceStruct, "message",
      1U, 1U, 14, c2_g_y));
  }

  c2_h_obj = &c2_g_obj->cSFunObject;
  c2_h_obj->P2_MAXAREA_RTP = MAX_uint32_T;
  c2_b_obj->NoTuningBeforeLockingCodeGenError = false;
  return c2_b_obj;
}

static void c2_SystemCore_step(SFc2_mirageTrackingRobotInstanceStruct
  *chartInstance, c2_visioncodegen_BlobAnalysis *c2_obj, boolean_T
  c2_g_varargin_1[1228800], int32_T c2_varargout_1_data[], int32_T
  c2_varargout_1_sizes[2], real_T c2_varargout_2_data[], int32_T
  c2_varargout_2_sizes[2], int32_T c2_varargout_3_data[], int32_T
  c2_varargout_3_sizes[2])
{
  const mxArray *c2_f_y = NULL;
  static char_T c2_f_u[45] = { 'M', 'A', 'T', 'L', 'A', 'B', ':', 's', 'y', 's',
    't', 'e', 'm', ':', 'm', 'e', 't', 'h', 'o', 'd', 'C', 'a', 'l', 'l', 'e',
    'd', 'W', 'h', 'e', 'n', 'R', 'e', 'l', 'e', 'a', 's', 'e', 'd', 'C', 'o',
    'd', 'e', 'g', 'e', 'n' };

  const mxArray *c2_g_y = NULL;
  static char_T c2_g_u[4] = { 's', 't', 'e', 'p' };

  c2_visioncodegen_BlobAnalysis *c2_b_obj;
  c2_visioncodegen_BlobAnalysis *c2_c_obj;
  const mxArray *c2_h_y = NULL;
  static char_T c2_h_u[51] = { 'M', 'A', 'T', 'L', 'A', 'B', ':', 's', 'y', 's',
    't', 'e', 'm', ':', 'm', 'e', 't', 'h', 'o', 'd', 'C', 'a', 'l', 'l', 'e',
    'd', 'W', 'h', 'e', 'n', 'L', 'o', 'c', 'k', 'e', 'd', 'R', 'e', 'l', 'e',
    'a', 's', 'e', 'd', 'C', 'o', 'd', 'e', 'g', 'e', 'n' };

  const mxArray *c2_i_y = NULL;
  static char_T c2_i_u[5] = { 's', 'e', 't', 'u', 'p' };

  const mxArray *c2_unusedU1f = NULL;
  const mxArray *c2_unusedU20 = NULL;
  const mxArray *c2_isDefNum = NULL;
  c2_visioncodegen_BlobAnalysis *c2_d_obj;
  const mxArray *c2_in = NULL;
  const mxArray *c2_unusedU9 = NULL;
  const mxArray *c2_isDefImpl = NULL;
  int32_T c2_i170;
  const mxArray *c2_unusedU5 = NULL;
  const mxArray *c2_unusedU6 = NULL;
  const mxArray *c2_isDefNumOut = NULL;
  if (c2_obj->isInitialized != 2) {
  } else {
    c2_f_y = NULL;
    sf_mex_assign(&c2_f_y, sf_mex_create("y", c2_f_u, 10, 0U, 1U, 0U, 2, 1, 45),
                  false);
    c2_g_y = NULL;
    sf_mex_assign(&c2_g_y, sf_mex_create("y", c2_g_u, 10, 0U, 1U, 0U, 2, 1, 4),
                  false);
    sf_mex_call_debug(sfGlobalDebugInstanceStruct, "error", 0U, 1U, 14,
                      sf_mex_call_debug(sfGlobalDebugInstanceStruct, "message",
      1U, 2U, 14, c2_f_y, 14, c2_g_y));
  }

  if (c2_obj->isInitialized != 1) {
    c2_b_obj = c2_obj;
    c2_c_obj = c2_b_obj;
    if (c2_c_obj->isInitialized == 0) {
    } else {
      c2_h_y = NULL;
      sf_mex_assign(&c2_h_y, sf_mex_create("y", c2_h_u, 10, 0U, 1U, 0U, 2, 1, 51),
                    false);
      c2_i_y = NULL;
      sf_mex_assign(&c2_i_y, sf_mex_create("y", c2_i_u, 10, 0U, 1U, 0U, 2, 1, 5),
                    false);
      sf_mex_call_debug(sfGlobalDebugInstanceStruct, "error", 0U, 1U, 14,
                        sf_mex_call_debug(sfGlobalDebugInstanceStruct, "message",
        1U, 2U, 14, c2_h_y, 14, c2_i_y));
    }

    c2_c_obj->isInitialized = 1;
    sf_mex_destroy(&c2_unusedU1f);
    sf_mex_destroy(&c2_unusedU20);
    sf_mex_destroy(&c2_isDefNum);
    c2_d_obj = c2_c_obj;
    c2_d_obj->NoTuningBeforeLockingCodeGenError = true;
    sf_mex_destroy(&c2_in);
    sf_mex_destroy(&c2_unusedU9);
    sf_mex_destroy(&c2_isDefImpl);
  }

  for (c2_i170 = 0; c2_i170 < 1228800; c2_i170++) {
    chartInstance->c2_e_varargin_1[c2_i170] = c2_g_varargin_1[c2_i170];
  }

  c2_Nondirect_stepImpl(chartInstance, c2_obj, chartInstance->c2_e_varargin_1,
                        c2_varargout_1_data, c2_varargout_1_sizes,
                        c2_varargout_2_data, c2_varargout_2_sizes,
                        c2_varargout_3_data, c2_varargout_3_sizes);
  sf_mex_destroy(&c2_unusedU5);
  sf_mex_destroy(&c2_unusedU6);
  sf_mex_destroy(&c2_isDefNumOut);
}

static void c2_Nondirect_stepImpl(SFc2_mirageTrackingRobotInstanceStruct
  *chartInstance, c2_visioncodegen_BlobAnalysis *c2_obj, boolean_T
  c2_g_varargin_1[1228800], int32_T c2_varargout_1_data[], int32_T
  c2_varargout_1_sizes[2], real_T c2_varargout_2_data[], int32_T
  c2_varargout_2_sizes[2], int32_T c2_varargout_3_data[], int32_T
  c2_varargout_3_sizes[2])
{
  c2_visioncodegen_BlobAnalysis *c2_b_obj;
  c2_vision_BlobAnalysis_1 *c2_c_obj;
  int32_T c2_i171;
  const mxArray *c2_errId = NULL;
  const mxArray *c2_errMsg = NULL;
  const mxArray *c2_numinputs = NULL;
  const mxArray *c2_b_errId = NULL;
  const mxArray *c2_b_errMsg = NULL;
  const mxArray *c2_numoutputs = NULL;
  c2_b_obj = c2_obj;
  c2_c_obj = &c2_b_obj->cSFunObject;
  for (c2_i171 = 0; c2_i171 < 1228800; c2_i171++) {
    chartInstance->c2_f_varargin_1[c2_i171] = c2_g_varargin_1[c2_i171];
  }

  c2_Outputs(chartInstance, c2_c_obj, chartInstance->c2_f_varargin_1,
             c2_varargout_1_data, c2_varargout_1_sizes, c2_varargout_2_data,
             c2_varargout_2_sizes, c2_varargout_3_data, c2_varargout_3_sizes);
  sf_mex_destroy(&c2_errId);
  sf_mex_destroy(&c2_errMsg);
  sf_mex_destroy(&c2_numinputs);
  sf_mex_destroy(&c2_b_errId);
  sf_mex_destroy(&c2_b_errMsg);
  sf_mex_destroy(&c2_numoutputs);
}

static void c2_b_SystemCore_step(SFc2_mirageTrackingRobotInstanceStruct
  *chartInstance, c2_visioncodegen_BlobAnalysis *c2_obj, boolean_T
  c2_g_varargin_1[1228800], int32_T c2_varargout_1_data[], int32_T
  c2_varargout_1_sizes[2], real_T c2_varargout_2_data[], int32_T
  c2_varargout_2_sizes[2], int32_T c2_varargout_3_data[], int32_T
  c2_varargout_3_sizes[2])
{
  const mxArray *c2_f_y = NULL;
  static char_T c2_f_u[45] = { 'M', 'A', 'T', 'L', 'A', 'B', ':', 's', 'y', 's',
    't', 'e', 'm', ':', 'm', 'e', 't', 'h', 'o', 'd', 'C', 'a', 'l', 'l', 'e',
    'd', 'W', 'h', 'e', 'n', 'R', 'e', 'l', 'e', 'a', 's', 'e', 'd', 'C', 'o',
    'd', 'e', 'g', 'e', 'n' };

  const mxArray *c2_g_y = NULL;
  static char_T c2_g_u[4] = { 's', 't', 'e', 'p' };

  c2_visioncodegen_BlobAnalysis *c2_b_obj;
  c2_visioncodegen_BlobAnalysis *c2_c_obj;
  const mxArray *c2_h_y = NULL;
  static char_T c2_h_u[51] = { 'M', 'A', 'T', 'L', 'A', 'B', ':', 's', 'y', 's',
    't', 'e', 'm', ':', 'm', 'e', 't', 'h', 'o', 'd', 'C', 'a', 'l', 'l', 'e',
    'd', 'W', 'h', 'e', 'n', 'L', 'o', 'c', 'k', 'e', 'd', 'R', 'e', 'l', 'e',
    'a', 's', 'e', 'd', 'C', 'o', 'd', 'e', 'g', 'e', 'n' };

  const mxArray *c2_i_y = NULL;
  static char_T c2_i_u[5] = { 's', 'e', 't', 'u', 'p' };

  const mxArray *c2_unusedU1f = NULL;
  const mxArray *c2_unusedU20 = NULL;
  const mxArray *c2_isDefNum = NULL;
  c2_visioncodegen_BlobAnalysis *c2_d_obj;
  const mxArray *c2_in = NULL;
  const mxArray *c2_unusedU9 = NULL;
  const mxArray *c2_isDefImpl = NULL;
  int32_T c2_i172;
  const mxArray *c2_unusedU5 = NULL;
  const mxArray *c2_unusedU6 = NULL;
  const mxArray *c2_isDefNumOut = NULL;
  if (c2_obj->isInitialized != 2) {
  } else {
    c2_f_y = NULL;
    sf_mex_assign(&c2_f_y, sf_mex_create("y", c2_f_u, 10, 0U, 1U, 0U, 2, 1, 45),
                  false);
    c2_g_y = NULL;
    sf_mex_assign(&c2_g_y, sf_mex_create("y", c2_g_u, 10, 0U, 1U, 0U, 2, 1, 4),
                  false);
    sf_mex_call_debug(sfGlobalDebugInstanceStruct, "error", 0U, 1U, 14,
                      sf_mex_call_debug(sfGlobalDebugInstanceStruct, "message",
      1U, 2U, 14, c2_f_y, 14, c2_g_y));
  }

  if (c2_obj->isInitialized != 1) {
    c2_b_obj = c2_obj;
    c2_c_obj = c2_b_obj;
    if (c2_c_obj->isInitialized == 0) {
    } else {
      c2_h_y = NULL;
      sf_mex_assign(&c2_h_y, sf_mex_create("y", c2_h_u, 10, 0U, 1U, 0U, 2, 1, 51),
                    false);
      c2_i_y = NULL;
      sf_mex_assign(&c2_i_y, sf_mex_create("y", c2_i_u, 10, 0U, 1U, 0U, 2, 1, 5),
                    false);
      sf_mex_call_debug(sfGlobalDebugInstanceStruct, "error", 0U, 1U, 14,
                        sf_mex_call_debug(sfGlobalDebugInstanceStruct, "message",
        1U, 2U, 14, c2_h_y, 14, c2_i_y));
    }

    c2_c_obj->isInitialized = 1;
    sf_mex_destroy(&c2_unusedU1f);
    sf_mex_destroy(&c2_unusedU20);
    sf_mex_destroy(&c2_isDefNum);
    c2_d_obj = c2_c_obj;
    c2_d_obj->NoTuningBeforeLockingCodeGenError = true;
    sf_mex_destroy(&c2_in);
    sf_mex_destroy(&c2_unusedU9);
    sf_mex_destroy(&c2_isDefImpl);
  }

  for (c2_i172 = 0; c2_i172 < 1228800; c2_i172++) {
    chartInstance->c2_c_varargin_1[c2_i172] = c2_g_varargin_1[c2_i172];
  }

  c2_b_Nondirect_stepImpl(chartInstance, c2_obj, chartInstance->c2_c_varargin_1,
    c2_varargout_1_data, c2_varargout_1_sizes, c2_varargout_2_data,
    c2_varargout_2_sizes, c2_varargout_3_data, c2_varargout_3_sizes);
  sf_mex_destroy(&c2_unusedU5);
  sf_mex_destroy(&c2_unusedU6);
  sf_mex_destroy(&c2_isDefNumOut);
}

static void c2_b_Nondirect_stepImpl(SFc2_mirageTrackingRobotInstanceStruct
  *chartInstance, c2_visioncodegen_BlobAnalysis *c2_obj, boolean_T
  c2_g_varargin_1[1228800], int32_T c2_varargout_1_data[], int32_T
  c2_varargout_1_sizes[2], real_T c2_varargout_2_data[], int32_T
  c2_varargout_2_sizes[2], int32_T c2_varargout_3_data[], int32_T
  c2_varargout_3_sizes[2])
{
  c2_visioncodegen_BlobAnalysis *c2_b_obj;
  c2_vision_BlobAnalysis_1 *c2_c_obj;
  int32_T c2_i173;
  const mxArray *c2_errId = NULL;
  const mxArray *c2_errMsg = NULL;
  const mxArray *c2_numinputs = NULL;
  const mxArray *c2_b_errId = NULL;
  const mxArray *c2_b_errMsg = NULL;
  const mxArray *c2_numoutputs = NULL;
  c2_b_obj = c2_obj;
  c2_c_obj = &c2_b_obj->cSFunObject;
  for (c2_i173 = 0; c2_i173 < 1228800; c2_i173++) {
    chartInstance->c2_d_varargin_1[c2_i173] = c2_g_varargin_1[c2_i173];
  }

  c2_Outputs(chartInstance, c2_c_obj, chartInstance->c2_d_varargin_1,
             c2_varargout_1_data, c2_varargout_1_sizes, c2_varargout_2_data,
             c2_varargout_2_sizes, c2_varargout_3_data, c2_varargout_3_sizes);
  sf_mex_destroy(&c2_errId);
  sf_mex_destroy(&c2_errMsg);
  sf_mex_destroy(&c2_numinputs);
  sf_mex_destroy(&c2_b_errId);
  sf_mex_destroy(&c2_b_errMsg);
  sf_mex_destroy(&c2_numoutputs);
}

static void c2_eml_sort(SFc2_mirageTrackingRobotInstanceStruct *chartInstance,
  real_T c2_x[4], real_T c2_b_x[4], int32_T c2_idx[4])
{
  int32_T c2_i174;
  for (c2_i174 = 0; c2_i174 < 4; c2_i174++) {
    c2_b_x[c2_i174] = c2_x[c2_i174];
  }

  c2_b_eml_sort(chartInstance, c2_b_x, c2_idx);
}

static void c2_eml_sort_idx(SFc2_mirageTrackingRobotInstanceStruct
  *chartInstance, real_T c2_x[4], int32_T c2_idx[4], real_T c2_b_x[4])
{
  int32_T c2_i175;
  for (c2_i175 = 0; c2_i175 < 4; c2_i175++) {
    c2_b_x[c2_i175] = c2_x[c2_i175];
  }

  c2_b_eml_sort_idx(chartInstance, c2_b_x, c2_idx);
}

static void c2_merge(SFc2_mirageTrackingRobotInstanceStruct *chartInstance,
                     int32_T c2_idx[4], real_T c2_x[4], int32_T c2_offset,
                     int32_T c2_np, int32_T c2_nq, int32_T c2_b_idx[4], real_T
                     c2_b_x[4])
{
  int32_T c2_i176;
  int32_T c2_i177;
  for (c2_i176 = 0; c2_i176 < 4; c2_i176++) {
    c2_b_idx[c2_i176] = c2_idx[c2_i176];
  }

  for (c2_i177 = 0; c2_i177 < 4; c2_i177++) {
    c2_b_x[c2_i177] = c2_x[c2_i177];
  }

  c2_b_merge(chartInstance, c2_b_idx, c2_b_x, c2_offset, c2_np, c2_nq);
}

static const mxArray *c2_j_sf_marshallOut(void *chartInstanceVoid, void
  *c2_inData)
{
  const mxArray *c2_mxArrayOutData = NULL;
  int32_T c2_f_u;
  const mxArray *c2_f_y = NULL;
  SFc2_mirageTrackingRobotInstanceStruct *chartInstance;
  chartInstance = (SFc2_mirageTrackingRobotInstanceStruct *)chartInstanceVoid;
  c2_mxArrayOutData = NULL;
  c2_f_u = *(int32_T *)c2_inData;
  c2_f_y = NULL;
  sf_mex_assign(&c2_f_y, sf_mex_create("y", &c2_f_u, 6, 0U, 0U, 0U, 0), false);
  sf_mex_assign(&c2_mxArrayOutData, c2_f_y, false);
  return c2_mxArrayOutData;
}

static int32_T c2_k_emlrt_marshallIn(SFc2_mirageTrackingRobotInstanceStruct
  *chartInstance, const mxArray *c2_f_u, const emlrtMsgIdentifier *c2_parentId)
{
  int32_T c2_f_y;
  int32_T c2_i178;
  (void)chartInstance;
  sf_mex_import(c2_parentId, sf_mex_dup(c2_f_u), &c2_i178, 1, 6, 0U, 0, 0U, 0);
  c2_f_y = c2_i178;
  sf_mex_destroy(&c2_f_u);
  return c2_f_y;
}

static void c2_j_sf_marshallIn(void *chartInstanceVoid, const mxArray
  *c2_mxArrayInData, const char_T *c2_varName, void *c2_outData)
{
  const mxArray *c2_b_sfEvent;
  const char_T *c2_identifier;
  emlrtMsgIdentifier c2_thisId;
  int32_T c2_f_y;
  SFc2_mirageTrackingRobotInstanceStruct *chartInstance;
  chartInstance = (SFc2_mirageTrackingRobotInstanceStruct *)chartInstanceVoid;
  c2_b_sfEvent = sf_mex_dup(c2_mxArrayInData);
  c2_identifier = c2_varName;
  c2_thisId.fIdentifier = c2_identifier;
  c2_thisId.fParent = NULL;
  c2_f_y = c2_k_emlrt_marshallIn(chartInstance, sf_mex_dup(c2_b_sfEvent),
    &c2_thisId);
  sf_mex_destroy(&c2_b_sfEvent);
  *(int32_T *)c2_outData = c2_f_y;
  sf_mex_destroy(&c2_mxArrayInData);
}

static boolean_T c2_l_emlrt_marshallIn(SFc2_mirageTrackingRobotInstanceStruct
  *chartInstance, const mxArray *c2_b_isInitialized, const char_T *c2_identifier)
{
  boolean_T c2_f_y;
  emlrtMsgIdentifier c2_thisId;
  c2_thisId.fIdentifier = c2_identifier;
  c2_thisId.fParent = NULL;
  c2_f_y = c2_m_emlrt_marshallIn(chartInstance, sf_mex_dup(c2_b_isInitialized),
    &c2_thisId);
  sf_mex_destroy(&c2_b_isInitialized);
  return c2_f_y;
}

static boolean_T c2_m_emlrt_marshallIn(SFc2_mirageTrackingRobotInstanceStruct
  *chartInstance, const mxArray *c2_f_u, const emlrtMsgIdentifier *c2_parentId)
{
  boolean_T c2_f_y;
  boolean_T c2_b0;
  (void)chartInstance;
  sf_mex_import(c2_parentId, sf_mex_dup(c2_f_u), &c2_b0, 1, 11, 0U, 0, 0U, 0);
  c2_f_y = c2_b0;
  sf_mex_destroy(&c2_f_u);
  return c2_f_y;
}

static uint8_T c2_n_emlrt_marshallIn(SFc2_mirageTrackingRobotInstanceStruct
  *chartInstance, const mxArray *c2_b_is_active_c2_mirageTrackingRobot, const
  char_T *c2_identifier)
{
  uint8_T c2_f_y;
  emlrtMsgIdentifier c2_thisId;
  c2_thisId.fIdentifier = c2_identifier;
  c2_thisId.fParent = NULL;
  c2_f_y = c2_o_emlrt_marshallIn(chartInstance, sf_mex_dup
    (c2_b_is_active_c2_mirageTrackingRobot), &c2_thisId);
  sf_mex_destroy(&c2_b_is_active_c2_mirageTrackingRobot);
  return c2_f_y;
}

static uint8_T c2_o_emlrt_marshallIn(SFc2_mirageTrackingRobotInstanceStruct
  *chartInstance, const mxArray *c2_f_u, const emlrtMsgIdentifier *c2_parentId)
{
  uint8_T c2_f_y;
  uint8_T c2_u14;
  (void)chartInstance;
  sf_mex_import(c2_parentId, sf_mex_dup(c2_f_u), &c2_u14, 1, 3, 0U, 0, 0U, 0);
  c2_f_y = c2_u14;
  sf_mex_destroy(&c2_f_u);
  return c2_f_y;
}

static void c2_b_imcomplement(SFc2_mirageTrackingRobotInstanceStruct
  *chartInstance, uint8_T c2_im[3686400])
{
  int32_T c2_i179;
  uint32_T c2_q0;
  uint32_T c2_qY;
  uint32_T c2_u15;
  (void)chartInstance;
  for (c2_i179 = 0; c2_i179 < 3686400; c2_i179++) {
    c2_q0 = 255U;
    c2_qY = c2_q0 - (uint32_T)c2_im[c2_i179];
    if (c2_qY > c2_q0) {
      c2_qY = 0U;
    }

    c2_u15 = c2_qY;
    if (c2_u15 > 255U) {
      c2_u15 = 255U;
    }

    c2_im[c2_i179] = (uint8_T)c2_u15;
  }
}

static void c2_b_eml_sort(SFc2_mirageTrackingRobotInstanceStruct *chartInstance,
  real_T c2_x[4], int32_T c2_idx[4])
{
  c2_b_eml_sort_idx(chartInstance, c2_x, c2_idx);
}

static void c2_b_eml_sort_idx(SFc2_mirageTrackingRobotInstanceStruct
  *chartInstance, real_T c2_x[4], int32_T c2_idx[4])
{
  int32_T c2_i180;
  int32_T c2_i181;
  real_T c2_x4[4];
  int32_T c2_i182;
  int32_T c2_idx4[4];
  int32_T c2_nNaNs;
  int32_T c2_ib;
  int32_T c2_k;
  int32_T c2_b_k;
  real_T c2_b_x;
  boolean_T c2_d_b;
  real_T c2_xwork[4];
  int32_T c2_quartetOffset;
  int32_T c2_i1;
  int32_T c2_i2;
  int32_T c2_i3;
  int32_T c2_i4;
  int32_T c2_perm[4];
  int32_T c2_wOffset;
  int32_T c2_tOffset;
  int32_T c2_n;
  int32_T c2_i183;
  int32_T c2_b_ib;
  int32_T c2_e_b;
  int32_T c2_f_b;
  boolean_T c2_overflow;
  int32_T c2_c_k;
  int32_T c2_m;
  int32_T c2_b_m;
  int32_T c2_g_b;
  int32_T c2_h_b;
  boolean_T c2_b_overflow;
  int32_T c2_d_k;
  int32_T c2_itmp;
  int32_T c2_b_nNaNs;
  int32_T c2_nNonNaN;
  int32_T c2_b_n;
  int32_T c2_nBlocks;
  int32_T c2_bLen;
  int32_T c2_tailOffset;
  int32_T c2_nTail;
  int32_T c2_bLen2;
  int32_T c2_nPairs;
  int32_T c2_b_nPairs;
  int32_T c2_i_b;
  int32_T c2_j_b;
  boolean_T c2_c_overflow;
  int32_T c2_e_k;
  int32_T c2_f_k;
  for (c2_i180 = 0; c2_i180 < 4; c2_i180++) {
    c2_idx[c2_i180] = 0;
  }

  for (c2_i181 = 0; c2_i181 < 4; c2_i181++) {
    c2_x4[c2_i181] = 0.0;
  }

  for (c2_i182 = 0; c2_i182 < 4; c2_i182++) {
    c2_idx4[c2_i182] = 0;
  }

  c2_nNaNs = 0;
  c2_ib = 0;
  for (c2_k = 1; c2_k < 5; c2_k++) {
    c2_b_k = c2_k - 1;
    c2_b_x = c2_x[c2_b_k];
    c2_d_b = muDoubleScalarIsNaN(c2_b_x);
    if (c2_d_b) {
      c2_idx[_SFD_EML_ARRAY_BOUNDS_CHECK("", 4 - c2_nNaNs, 1, 4, 1, 0) - 1] =
        c2_b_k + 1;
      c2_xwork[_SFD_EML_ARRAY_BOUNDS_CHECK("", 4 - c2_nNaNs, 1, 4, 1, 0) - 1] =
        c2_x[c2_b_k];
      c2_nNaNs++;
    } else {
      c2_ib++;
      c2_idx4[c2_ib - 1] = c2_b_k + 1;
      c2_x4[c2_ib - 1] = c2_x[c2_b_k];
      if (c2_ib == 4) {
        c2_quartetOffset = c2_b_k - c2_nNaNs;
        if (c2_x4[0] <= c2_x4[1]) {
          c2_i1 = 1;
          c2_i2 = 2;
        } else {
          c2_i1 = 2;
          c2_i2 = 1;
        }

        if (c2_x4[2] <= c2_x4[3]) {
          c2_i3 = 3;
          c2_i4 = 4;
        } else {
          c2_i3 = 4;
          c2_i4 = 3;
        }

        if (c2_x4[c2_i1 - 1] <= c2_x4[c2_i3 - 1]) {
          if (c2_x4[c2_i2 - 1] <= c2_x4[c2_i3 - 1]) {
            c2_perm[0] = c2_i1;
            c2_perm[1] = c2_i2;
            c2_perm[2] = c2_i3;
            c2_perm[3] = c2_i4;
          } else if (c2_x4[c2_i2 - 1] <= c2_x4[c2_i4 - 1]) {
            c2_perm[0] = c2_i1;
            c2_perm[1] = c2_i3;
            c2_perm[2] = c2_i2;
            c2_perm[3] = c2_i4;
          } else {
            c2_perm[0] = c2_i1;
            c2_perm[1] = c2_i3;
            c2_perm[2] = c2_i4;
            c2_perm[3] = c2_i2;
          }
        } else if (c2_x4[c2_i1 - 1] <= c2_x4[c2_i4 - 1]) {
          if (c2_x4[c2_i2 - 1] <= c2_x4[c2_i4 - 1]) {
            c2_perm[0] = c2_i3;
            c2_perm[1] = c2_i1;
            c2_perm[2] = c2_i2;
            c2_perm[3] = c2_i4;
          } else {
            c2_perm[0] = c2_i3;
            c2_perm[1] = c2_i1;
            c2_perm[2] = c2_i4;
            c2_perm[3] = c2_i2;
          }
        } else {
          c2_perm[0] = c2_i3;
          c2_perm[1] = c2_i4;
          c2_perm[2] = c2_i1;
          c2_perm[3] = c2_i2;
        }

        c2_idx[_SFD_EML_ARRAY_BOUNDS_CHECK("", c2_quartetOffset - 2, 1, 4, 1, 0)
          - 1] = c2_idx4[c2_perm[0] - 1];
        c2_idx[_SFD_EML_ARRAY_BOUNDS_CHECK("", c2_quartetOffset - 1, 1, 4, 1, 0)
          - 1] = c2_idx4[c2_perm[1] - 1];
        c2_idx[_SFD_EML_ARRAY_BOUNDS_CHECK("", c2_quartetOffset, 1, 4, 1, 0) - 1]
          = c2_idx4[c2_perm[2] - 1];
        c2_idx[_SFD_EML_ARRAY_BOUNDS_CHECK("", c2_quartetOffset + 1, 1, 4, 1, 0)
          - 1] = c2_idx4[c2_perm[3] - 1];
        c2_x[_SFD_EML_ARRAY_BOUNDS_CHECK("", c2_quartetOffset - 2, 1, 4, 1, 0) -
          1] = c2_x4[c2_perm[0] - 1];
        c2_x[_SFD_EML_ARRAY_BOUNDS_CHECK("", c2_quartetOffset - 1, 1, 4, 1, 0) -
          1] = c2_x4[c2_perm[1] - 1];
        c2_x[_SFD_EML_ARRAY_BOUNDS_CHECK("", c2_quartetOffset, 1, 4, 1, 0) - 1] =
          c2_x4[c2_perm[2] - 1];
        c2_x[_SFD_EML_ARRAY_BOUNDS_CHECK("", c2_quartetOffset + 1, 1, 4, 1, 0) -
          1] = c2_x4[c2_perm[3] - 1];
        c2_ib = 0;
      }
    }
  }

  c2_wOffset = 4 - c2_nNaNs;
  c2_tOffset = c2_wOffset;
  if (c2_ib > 0) {
    c2_n = c2_ib;
    for (c2_i183 = 0; c2_i183 < 4; c2_i183++) {
      c2_perm[c2_i183] = 0;
    }

    if (c2_n == 1) {
      c2_perm[0] = 1;
    } else if (c2_n == 2) {
      if (c2_x4[0] <= c2_x4[1]) {
        c2_perm[0] = 1;
        c2_perm[1] = 2;
      } else {
        c2_perm[0] = 2;
        c2_perm[1] = 1;
      }
    } else if (c2_x4[0] <= c2_x4[1]) {
      if (c2_x4[1] <= c2_x4[2]) {
        c2_perm[0] = 1;
        c2_perm[1] = 2;
        c2_perm[2] = 3;
      } else if (c2_x4[0] <= c2_x4[2]) {
        c2_perm[0] = 1;
        c2_perm[1] = 3;
        c2_perm[2] = 2;
      } else {
        c2_perm[0] = 3;
        c2_perm[1] = 1;
        c2_perm[2] = 2;
      }
    } else if (c2_x4[0] <= c2_x4[2]) {
      c2_perm[0] = 2;
      c2_perm[1] = 1;
      c2_perm[2] = 3;
    } else if (c2_x4[1] <= c2_x4[2]) {
      c2_perm[0] = 2;
      c2_perm[1] = 3;
      c2_perm[2] = 1;
    } else {
      c2_perm[0] = 3;
      c2_perm[1] = 2;
      c2_perm[2] = 1;
    }

    c2_b_ib = c2_ib;
    c2_e_b = c2_b_ib;
    c2_f_b = c2_e_b;
    if (1 > c2_f_b) {
      c2_overflow = false;
    } else {
      c2_overflow = (c2_f_b > 2147483646);
    }

    if (c2_overflow) {
      c2_check_forloop_overflow_error(chartInstance, true);
    }

    for (c2_c_k = 1; c2_c_k <= c2_b_ib; c2_c_k++) {
      c2_b_k = c2_c_k;
      c2_idx[_SFD_EML_ARRAY_BOUNDS_CHECK("", (c2_tOffset - c2_ib) + c2_b_k, 1, 4,
        1, 0) - 1] = c2_idx4[_SFD_EML_ARRAY_BOUNDS_CHECK("", c2_perm[c2_b_k - 1],
        1, 4, 1, 0) - 1];
      c2_x[_SFD_EML_ARRAY_BOUNDS_CHECK("", (c2_tOffset - c2_ib) + c2_b_k, 1, 4,
        1, 0) - 1] = c2_x4[_SFD_EML_ARRAY_BOUNDS_CHECK("", c2_perm[c2_b_k - 1],
        1, 4, 1, 0) - 1];
    }
  }

  c2_m = (c2_nNaNs >> 1) + 1;
  c2_b_m = c2_m - 1;
  c2_g_b = c2_b_m;
  c2_h_b = c2_g_b;
  if (1 > c2_h_b) {
    c2_b_overflow = false;
  } else {
    c2_b_overflow = (c2_h_b > 2147483646);
  }

  if (c2_b_overflow) {
    c2_check_forloop_overflow_error(chartInstance, true);
  }

  for (c2_d_k = 1; c2_d_k <= c2_b_m; c2_d_k++) {
    c2_b_k = c2_d_k;
    c2_itmp = c2_idx[_SFD_EML_ARRAY_BOUNDS_CHECK("", c2_tOffset + c2_b_k, 1, 4,
      1, 0) - 1];
    c2_idx[_SFD_EML_ARRAY_BOUNDS_CHECK("", c2_tOffset + c2_b_k, 1, 4, 1, 0) - 1]
      = c2_idx[_SFD_EML_ARRAY_BOUNDS_CHECK("", 5 - c2_b_k, 1, 4, 1, 0) - 1];
    c2_idx[_SFD_EML_ARRAY_BOUNDS_CHECK("", 5 - c2_b_k, 1, 4, 1, 0) - 1] =
      c2_itmp;
    c2_x[_SFD_EML_ARRAY_BOUNDS_CHECK("", c2_tOffset + c2_b_k, 1, 4, 1, 0) - 1] =
      c2_xwork[_SFD_EML_ARRAY_BOUNDS_CHECK("", 5 - c2_b_k, 1, 4, 1, 0) - 1];
    c2_x[_SFD_EML_ARRAY_BOUNDS_CHECK("", 5 - c2_b_k, 1, 4, 1, 0) - 1] =
      c2_xwork[_SFD_EML_ARRAY_BOUNDS_CHECK("", c2_wOffset + c2_b_k, 1, 4, 1, 0)
      - 1];
  }

  if ((c2_nNaNs & 1) != 0) {
    c2_x[_SFD_EML_ARRAY_BOUNDS_CHECK("", c2_tOffset + c2_m, 1, 4, 1, 0) - 1] =
      c2_xwork[_SFD_EML_ARRAY_BOUNDS_CHECK("", c2_wOffset + c2_m, 1, 4, 1, 0) -
      1];
  }

  c2_b_nNaNs = c2_nNaNs;
  c2_nNonNaN = 4 - c2_b_nNaNs;
  if (c2_nNonNaN > 1) {
    c2_b_n = c2_nNonNaN;
    c2_nBlocks = c2_b_n >> 2;
    c2_bLen = 4;
    while (c2_nBlocks > 1) {
      if ((c2_nBlocks & 1) != 0) {
        c2_nBlocks--;
        c2_tailOffset = c2_bLen * c2_nBlocks;
        c2_nTail = c2_b_n - c2_tailOffset;
        if (c2_nTail > c2_bLen) {
          c2_b_merge(chartInstance, c2_idx, c2_x, c2_tailOffset, c2_bLen,
                     c2_nTail - c2_bLen);
        }
      }

      c2_bLen2 = c2_bLen << 1;
      c2_nPairs = c2_nBlocks >> 1;
      c2_b_nPairs = c2_nPairs;
      c2_i_b = c2_b_nPairs;
      c2_j_b = c2_i_b;
      if (1 > c2_j_b) {
        c2_c_overflow = false;
      } else {
        c2_c_overflow = (c2_j_b > 2147483646);
      }

      if (c2_c_overflow) {
        c2_check_forloop_overflow_error(chartInstance, true);
      }

      for (c2_e_k = 1; c2_e_k <= c2_b_nPairs; c2_e_k++) {
        c2_f_k = c2_e_k - 1;
        c2_b_merge(chartInstance, c2_idx, c2_x, c2_f_k * c2_bLen2, c2_bLen,
                   c2_bLen);
      }

      c2_bLen = c2_bLen2;
      c2_nBlocks = c2_nPairs;
    }

    if (c2_b_n > c2_bLen) {
      c2_b_merge(chartInstance, c2_idx, c2_x, 0, c2_bLen, c2_b_n - c2_bLen);
    }
  }
}

static void c2_b_merge(SFc2_mirageTrackingRobotInstanceStruct *chartInstance,
  int32_T c2_idx[4], real_T c2_x[4], int32_T c2_offset, int32_T c2_np, int32_T
  c2_nq)
{
  int32_T c2_n;
  int32_T c2_b_n;
  int32_T c2_d_b;
  int32_T c2_e_b;
  boolean_T c2_overflow;
  int32_T c2_j;
  int32_T c2_b_j;
  int32_T c2_iwork[4];
  real_T c2_xwork[4];
  int32_T c2_p;
  int32_T c2_pend;
  int32_T c2_q;
  int32_T c2_qend;
  int32_T c2_iout;
  int32_T c2_offset1;
  int32_T c2_b_p;
  int32_T c2_b_pend;
  int32_T c2_f_b;
  int32_T c2_g_b;
  boolean_T c2_b_overflow;
  int32_T c2_c_j;
  int32_T exitg1;
  if (c2_nq == 0) {
  } else {
    c2_n = c2_np + c2_nq;
    c2_b_n = c2_n;
    c2_d_b = c2_b_n;
    c2_e_b = c2_d_b;
    if (1 > c2_e_b) {
      c2_overflow = false;
    } else {
      c2_overflow = (c2_e_b > 2147483646);
    }

    if (c2_overflow) {
      c2_check_forloop_overflow_error(chartInstance, true);
    }

    for (c2_j = 1; c2_j <= c2_b_n; c2_j++) {
      c2_b_j = c2_j;
      c2_iwork[_SFD_EML_ARRAY_BOUNDS_CHECK("", c2_b_j, 1, 4, 1, 0) - 1] =
        c2_idx[_SFD_EML_ARRAY_BOUNDS_CHECK("", c2_offset + c2_b_j, 1, 4, 1, 0) -
        1];
      c2_xwork[c2_b_j - 1] = c2_x[_SFD_EML_ARRAY_BOUNDS_CHECK("", c2_offset +
        c2_b_j, 1, 4, 1, 0) - 1];
    }

    c2_p = 0;
    c2_pend = c2_np;
    c2_q = c2_pend;
    c2_qend = c2_pend + c2_nq;
    c2_iout = c2_offset;
    do {
      exitg1 = 0;
      c2_iout++;
      if (c2_xwork[_SFD_EML_ARRAY_BOUNDS_CHECK("", c2_p + 1, 1, 4, 1, 0) - 1] <=
          c2_xwork[_SFD_EML_ARRAY_BOUNDS_CHECK("", c2_q + 1, 1, 4, 1, 0) - 1]) {
        c2_idx[_SFD_EML_ARRAY_BOUNDS_CHECK("", c2_iout, 1, 4, 1, 0) - 1] =
          c2_iwork[c2_p];
        c2_x[c2_iout - 1] = c2_xwork[c2_p];
        if (c2_p + 1 < c2_pend) {
          c2_p++;
        } else {
          exitg1 = 1;
        }
      } else {
        c2_idx[_SFD_EML_ARRAY_BOUNDS_CHECK("", c2_iout, 1, 4, 1, 0) - 1] =
          c2_iwork[c2_q];
        c2_x[c2_iout - 1] = c2_xwork[c2_q];
        if (c2_q + 1 < c2_qend) {
          c2_q++;
        } else {
          c2_offset1 = c2_iout - c2_p;
          c2_b_p = c2_p + 1;
          c2_b_pend = c2_pend;
          c2_f_b = c2_b_pend;
          c2_g_b = c2_f_b;
          c2_b_overflow = (c2_g_b > 2147483646);
          if (c2_b_overflow) {
            c2_check_forloop_overflow_error(chartInstance, true);
          }

          for (c2_c_j = c2_b_p; c2_c_j <= c2_b_pend; c2_c_j++) {
            c2_b_j = c2_c_j;
            c2_idx[_SFD_EML_ARRAY_BOUNDS_CHECK("", c2_offset1 + c2_b_j, 1, 4, 1,
              0) - 1] = c2_iwork[_SFD_EML_ARRAY_BOUNDS_CHECK("", c2_b_j, 1, 4, 1,
              0) - 1];
            c2_x[_SFD_EML_ARRAY_BOUNDS_CHECK("", c2_offset1 + c2_b_j, 1, 4, 1, 0)
              - 1] = c2_xwork[c2_b_j - 1];
          }

          exitg1 = 1;
        }
      }
    } while (exitg1 == 0);
  }
}

static void init_dsm_address_info(SFc2_mirageTrackingRobotInstanceStruct
  *chartInstance)
{
  (void)chartInstance;
}

static void init_simulink_io_address(SFc2_mirageTrackingRobotInstanceStruct
  *chartInstance)
{
  chartInstance->c2_b_l_rgb = (uint8_T (*)[3686400])ssGetInputPortSignal_wrapper
    (chartInstance->S, 0);
  chartInstance->c2_L_p = (real_T (*)[8])ssGetOutputPortSignal_wrapper
    (chartInstance->S, 1);
  chartInstance->c2_b_r_rgb = (uint8_T (*)[3686400])ssGetInputPortSignal_wrapper
    (chartInstance->S, 1);
  chartInstance->c2_R_p = (real_T (*)[8])ssGetOutputPortSignal_wrapper
    (chartInstance->S, 2);
}

/* SFunction Glue Code */
#ifdef utFree
#undef utFree
#endif

#ifdef utMalloc
#undef utMalloc
#endif

#ifdef __cplusplus

extern "C" void *utMalloc(size_t size);
extern "C" void utFree(void*);

#else

extern void *utMalloc(size_t size);
extern void utFree(void*);

#endif

void sf_c2_mirageTrackingRobot_get_check_sum(mxArray *plhs[])
{
  ((real_T *)mxGetPr((plhs[0])))[0] = (real_T)(3468817813U);
  ((real_T *)mxGetPr((plhs[0])))[1] = (real_T)(870992185U);
  ((real_T *)mxGetPr((plhs[0])))[2] = (real_T)(157567654U);
  ((real_T *)mxGetPr((plhs[0])))[3] = (real_T)(3457998211U);
}

mxArray* sf_c2_mirageTrackingRobot_get_post_codegen_info(void);
mxArray *sf_c2_mirageTrackingRobot_get_autoinheritance_info(void)
{
  const char *autoinheritanceFields[] = { "checksum", "inputs", "parameters",
    "outputs", "locals", "postCodegenInfo" };

  mxArray *mxAutoinheritanceInfo = mxCreateStructMatrix(1, 1, sizeof
    (autoinheritanceFields)/sizeof(autoinheritanceFields[0]),
    autoinheritanceFields);

  {
    mxArray *mxChecksum = mxCreateString("dILYnXyxJy5PN1yemTJTsE");
    mxSetField(mxAutoinheritanceInfo,0,"checksum",mxChecksum);
  }

  {
    const char *dataFields[] = { "size", "type", "complexity" };

    mxArray *mxData = mxCreateStructMatrix(1,2,3,dataFields);

    {
      mxArray *mxSize = mxCreateDoubleMatrix(1,3,mxREAL);
      double *pr = mxGetPr(mxSize);
      pr[0] = (double)(960);
      pr[1] = (double)(1280);
      pr[2] = (double)(3);
      mxSetField(mxData,0,"size",mxSize);
    }

    {
      const char *typeFields[] = { "base", "fixpt", "isFixedPointType" };

      mxArray *mxType = mxCreateStructMatrix(1,1,sizeof(typeFields)/sizeof
        (typeFields[0]),typeFields);
      mxSetField(mxType,0,"base",mxCreateDoubleScalar(3));
      mxSetField(mxType,0,"fixpt",mxCreateDoubleMatrix(0,0,mxREAL));
      mxSetField(mxType,0,"isFixedPointType",mxCreateDoubleScalar(0));
      mxSetField(mxData,0,"type",mxType);
    }

    mxSetField(mxData,0,"complexity",mxCreateDoubleScalar(0));

    {
      mxArray *mxSize = mxCreateDoubleMatrix(1,3,mxREAL);
      double *pr = mxGetPr(mxSize);
      pr[0] = (double)(960);
      pr[1] = (double)(1280);
      pr[2] = (double)(3);
      mxSetField(mxData,1,"size",mxSize);
    }

    {
      const char *typeFields[] = { "base", "fixpt", "isFixedPointType" };

      mxArray *mxType = mxCreateStructMatrix(1,1,sizeof(typeFields)/sizeof
        (typeFields[0]),typeFields);
      mxSetField(mxType,0,"base",mxCreateDoubleScalar(3));
      mxSetField(mxType,0,"fixpt",mxCreateDoubleMatrix(0,0,mxREAL));
      mxSetField(mxType,0,"isFixedPointType",mxCreateDoubleScalar(0));
      mxSetField(mxData,1,"type",mxType);
    }

    mxSetField(mxData,1,"complexity",mxCreateDoubleScalar(0));
    mxSetField(mxAutoinheritanceInfo,0,"inputs",mxData);
  }

  {
    mxSetField(mxAutoinheritanceInfo,0,"parameters",mxCreateDoubleMatrix(0,0,
                mxREAL));
  }

  {
    const char *dataFields[] = { "size", "type", "complexity" };

    mxArray *mxData = mxCreateStructMatrix(1,2,3,dataFields);

    {
      mxArray *mxSize = mxCreateDoubleMatrix(1,2,mxREAL);
      double *pr = mxGetPr(mxSize);
      pr[0] = (double)(2);
      pr[1] = (double)(4);
      mxSetField(mxData,0,"size",mxSize);
    }

    {
      const char *typeFields[] = { "base", "fixpt", "isFixedPointType" };

      mxArray *mxType = mxCreateStructMatrix(1,1,sizeof(typeFields)/sizeof
        (typeFields[0]),typeFields);
      mxSetField(mxType,0,"base",mxCreateDoubleScalar(10));
      mxSetField(mxType,0,"fixpt",mxCreateDoubleMatrix(0,0,mxREAL));
      mxSetField(mxType,0,"isFixedPointType",mxCreateDoubleScalar(0));
      mxSetField(mxData,0,"type",mxType);
    }

    mxSetField(mxData,0,"complexity",mxCreateDoubleScalar(0));

    {
      mxArray *mxSize = mxCreateDoubleMatrix(1,2,mxREAL);
      double *pr = mxGetPr(mxSize);
      pr[0] = (double)(2);
      pr[1] = (double)(4);
      mxSetField(mxData,1,"size",mxSize);
    }

    {
      const char *typeFields[] = { "base", "fixpt", "isFixedPointType" };

      mxArray *mxType = mxCreateStructMatrix(1,1,sizeof(typeFields)/sizeof
        (typeFields[0]),typeFields);
      mxSetField(mxType,0,"base",mxCreateDoubleScalar(10));
      mxSetField(mxType,0,"fixpt",mxCreateDoubleMatrix(0,0,mxREAL));
      mxSetField(mxType,0,"isFixedPointType",mxCreateDoubleScalar(0));
      mxSetField(mxData,1,"type",mxType);
    }

    mxSetField(mxData,1,"complexity",mxCreateDoubleScalar(0));
    mxSetField(mxAutoinheritanceInfo,0,"outputs",mxData);
  }

  {
    mxSetField(mxAutoinheritanceInfo,0,"locals",mxCreateDoubleMatrix(0,0,mxREAL));
  }

  {
    mxArray* mxPostCodegenInfo = sf_c2_mirageTrackingRobot_get_post_codegen_info
      ();
    mxSetField(mxAutoinheritanceInfo,0,"postCodegenInfo",mxPostCodegenInfo);
  }

  return(mxAutoinheritanceInfo);
}

mxArray *sf_c2_mirageTrackingRobot_third_party_uses_info(void)
{
  mxArray * mxcell3p = mxCreateCellMatrix(1,1);
  mxSetCell(mxcell3p, 0, mxCreateString(
             "images.internal.coder.buildable.Medianfilter_ippBuildable"));
  return(mxcell3p);
}

mxArray *sf_c2_mirageTrackingRobot_jit_fallback_info(void)
{
  const char *infoFields[] = { "fallbackType", "fallbackReason",
    "hiddenFallbackType", "hiddenFallbackReason", "incompatibleSymbol" };

  mxArray *mxInfo = mxCreateStructMatrix(1, 1, 5, infoFields);
  mxArray *fallbackType = mxCreateString("late");
  mxArray *fallbackReason = mxCreateString("ir_function_calls");
  mxArray *hiddenFallbackType = mxCreateString("");
  mxArray *hiddenFallbackReason = mxCreateString("");
  mxArray *incompatibleSymbol = mxCreateString("ippMedianFilter_uint8");
  mxSetField(mxInfo, 0, infoFields[0], fallbackType);
  mxSetField(mxInfo, 0, infoFields[1], fallbackReason);
  mxSetField(mxInfo, 0, infoFields[2], hiddenFallbackType);
  mxSetField(mxInfo, 0, infoFields[3], hiddenFallbackReason);
  mxSetField(mxInfo, 0, infoFields[4], incompatibleSymbol);
  return mxInfo;
}

mxArray *sf_c2_mirageTrackingRobot_updateBuildInfo_args_info(void)
{
  mxArray *mxBIArgs = mxCreateCellMatrix(1,0);
  return mxBIArgs;
}

mxArray* sf_c2_mirageTrackingRobot_get_post_codegen_info(void)
{
  const char* fieldNames[] = { "exportedFunctionsUsedByThisChart",
    "exportedFunctionsChecksum" };

  mwSize dims[2] = { 1, 1 };

  mxArray* mxPostCodegenInfo = mxCreateStructArray(2, dims, sizeof(fieldNames)/
    sizeof(fieldNames[0]), fieldNames);

  {
    mxArray* mxExportedFunctionsChecksum = mxCreateString("");
    mwSize exp_dims[2] = { 0, 1 };

    mxArray* mxExportedFunctionsUsedByThisChart = mxCreateCellArray(2, exp_dims);
    mxSetField(mxPostCodegenInfo, 0, "exportedFunctionsUsedByThisChart",
               mxExportedFunctionsUsedByThisChart);
    mxSetField(mxPostCodegenInfo, 0, "exportedFunctionsChecksum",
               mxExportedFunctionsChecksum);
  }

  return mxPostCodegenInfo;
}

static const mxArray *sf_get_sim_state_info_c2_mirageTrackingRobot(void)
{
  const char *infoFields[] = { "chartChecksum", "varInfo" };

  mxArray *mxInfo = mxCreateStructMatrix(1, 1, 2, infoFields);
  const char *infoEncStr[] = {
    "100 S1x4'type','srcId','name','auxInfo'{{M[1],M[5],T\"L_p\",},{M[1],M[7],T\"R_p\",},{M[4],M[0],T\"isInitialized\",},{M[8],M[0],T\"is_active_c2_mirageTrackingRobot\",}}"
  };

  mxArray *mxVarInfo = sf_mex_decode_encoded_mx_struct_array(infoEncStr, 4, 10);
  mxArray *mxChecksum = mxCreateDoubleMatrix(1, 4, mxREAL);
  sf_c2_mirageTrackingRobot_get_check_sum(&mxChecksum);
  mxSetField(mxInfo, 0, infoFields[0], mxChecksum);
  mxSetField(mxInfo, 0, infoFields[1], mxVarInfo);
  return mxInfo;
}

static void chart_debug_initialization(SimStruct *S, unsigned int
  fullDebuggerInitialization)
{
  if (!sim_mode_is_rtw_gen(S)) {
    SFc2_mirageTrackingRobotInstanceStruct *chartInstance;
    ChartRunTimeInfo * crtInfo = (ChartRunTimeInfo *)(ssGetUserData(S));
    ChartInfoStruct * chartInfo = (ChartInfoStruct *)(crtInfo->instanceInfo);
    chartInstance = (SFc2_mirageTrackingRobotInstanceStruct *)
      chartInfo->chartInstance;
    if (ssIsFirstInitCond(S) && fullDebuggerInitialization==1) {
      /* do this only if simulation is starting */
      {
        unsigned int chartAlreadyPresent;
        chartAlreadyPresent = sf_debug_initialize_chart
          (sfGlobalDebugInstanceStruct,
           _mirageTrackingRobotMachineNumber_,
           2,
           1,
           1,
           0,
           4,
           0,
           0,
           0,
           0,
           1,
           &(chartInstance->chartNumber),
           &(chartInstance->instanceNumber),
           (void *)S);

        /* Each instance must initialize its own list of scripts */
        init_script_number_translation(_mirageTrackingRobotMachineNumber_,
          chartInstance->chartNumber,chartInstance->instanceNumber);
        if (chartAlreadyPresent==0) {
          /* this is the first instance */
          sf_debug_set_chart_disable_implicit_casting
            (sfGlobalDebugInstanceStruct,_mirageTrackingRobotMachineNumber_,
             chartInstance->chartNumber,1);
          sf_debug_set_chart_event_thresholds(sfGlobalDebugInstanceStruct,
            _mirageTrackingRobotMachineNumber_,
            chartInstance->chartNumber,
            0,
            0,
            0);
          _SFD_SET_DATA_PROPS(0,1,1,0,"l_rgb");
          _SFD_SET_DATA_PROPS(1,1,1,0,"r_rgb");
          _SFD_SET_DATA_PROPS(2,2,0,1,"L_p");
          _SFD_SET_DATA_PROPS(3,2,0,1,"R_p");
          _SFD_STATE_INFO(0,0,2);
          _SFD_CH_SUBSTATE_COUNT(0);
          _SFD_CH_SUBSTATE_DECOMP(0);
        }

        _SFD_CV_INIT_CHART(0,0,0,0);

        {
          _SFD_CV_INIT_STATE(0,0,0,0,0,0,NULL,NULL);
        }

        _SFD_CV_INIT_TRANS(0,0,NULL,NULL,0,NULL);

        /* Initialization of MATLAB Function Model Coverage */
        _SFD_CV_INIT_EML(0,1,2,0,2,0,12,0,2,0,0,0);
        _SFD_CV_INIT_EML_FCN(0,0,"eML_blk_kernel",0,-1,2045);
        _SFD_CV_INIT_EML_FCN(0,1,"myMat2gray",2047,-1,2255);
        _SFD_CV_INIT_EML_SATURATION(0,1,0,610,-1,632);
        _SFD_CV_INIT_EML_SATURATION(0,1,1,610,-1,657);
        _SFD_CV_INIT_EML_SATURATION(0,1,2,610,-1,682);
        _SFD_CV_INIT_EML_SATURATION(0,1,3,635,-1,657);
        _SFD_CV_INIT_EML_SATURATION(0,1,4,660,-1,682);
        _SFD_CV_INIT_EML_SATURATION(0,1,5,740,-1,762);
        _SFD_CV_INIT_EML_SATURATION(0,1,6,740,-1,787);
        _SFD_CV_INIT_EML_SATURATION(0,1,7,740,-1,812);
        _SFD_CV_INIT_EML_SATURATION(0,1,8,765,-1,787);
        _SFD_CV_INIT_EML_SATURATION(0,1,9,790,-1,812);
        _SFD_CV_INIT_EML_SATURATION(0,1,10,874,-1,894);
        _SFD_CV_INIT_EML_SATURATION(0,1,11,946,-1,966);
        _SFD_CV_INIT_EML_IF(0,1,0,1562,1571,-1,1595);
        _SFD_CV_INIT_EML_IF(0,1,1,1717,1726,-1,1750);
        _SFD_CV_INIT_EML_FOR(0,1,0,1600,1611,1650);
        _SFD_CV_INIT_EML_FOR(0,1,1,1755,1766,1805);
        _SFD_CV_INIT_EML_RELATIONAL(0,1,0,1565,1571,-1,4);
        _SFD_CV_INIT_EML_RELATIONAL(0,1,1,1720,1726,-1,4);
        _SFD_CV_INIT_SCRIPT(0,1,0,0,0,0,0,0,0,0,0);
        _SFD_CV_INIT_SCRIPT_FCN(0,0,"orderPoints",0,-1,232);

        {
          unsigned int dimVector[3];
          dimVector[0]= 960;
          dimVector[1]= 1280;
          dimVector[2]= 3;
          _SFD_SET_DATA_COMPILED_PROPS(0,SF_UINT8,3,&(dimVector[0]),0,0,0,0.0,
            1.0,0,0,(MexFcnForType)c2_b_sf_marshallOut,(MexInFcnForType)NULL);
        }

        {
          unsigned int dimVector[3];
          dimVector[0]= 960;
          dimVector[1]= 1280;
          dimVector[2]= 3;
          _SFD_SET_DATA_COMPILED_PROPS(1,SF_UINT8,3,&(dimVector[0]),0,0,0,0.0,
            1.0,0,0,(MexFcnForType)c2_b_sf_marshallOut,(MexInFcnForType)NULL);
        }

        {
          unsigned int dimVector[2];
          dimVector[0]= 2;
          dimVector[1]= 4;
          _SFD_SET_DATA_COMPILED_PROPS(2,SF_DOUBLE,2,&(dimVector[0]),0,0,0,0.0,
            1.0,0,0,(MexFcnForType)c2_sf_marshallOut,(MexInFcnForType)
            c2_sf_marshallIn);
        }

        {
          unsigned int dimVector[2];
          dimVector[0]= 2;
          dimVector[1]= 4;
          _SFD_SET_DATA_COMPILED_PROPS(3,SF_DOUBLE,2,&(dimVector[0]),0,0,0,0.0,
            1.0,0,0,(MexFcnForType)c2_sf_marshallOut,(MexInFcnForType)
            c2_sf_marshallIn);
        }
      }
    } else {
      sf_debug_reset_current_state_configuration(sfGlobalDebugInstanceStruct,
        _mirageTrackingRobotMachineNumber_,chartInstance->chartNumber,
        chartInstance->instanceNumber);
    }
  }
}

static void chart_debug_initialize_data_addresses(SimStruct *S)
{
  if (!sim_mode_is_rtw_gen(S)) {
    SFc2_mirageTrackingRobotInstanceStruct *chartInstance;
    ChartRunTimeInfo * crtInfo = (ChartRunTimeInfo *)(ssGetUserData(S));
    ChartInfoStruct * chartInfo = (ChartInfoStruct *)(crtInfo->instanceInfo);
    chartInstance = (SFc2_mirageTrackingRobotInstanceStruct *)
      chartInfo->chartInstance;
    if (ssIsFirstInitCond(S)) {
      /* do this only if simulation is starting and after we know the addresses of all data */
      {
        _SFD_SET_DATA_VALUE_PTR(0U, *chartInstance->c2_b_l_rgb);
        _SFD_SET_DATA_VALUE_PTR(2U, *chartInstance->c2_L_p);
        _SFD_SET_DATA_VALUE_PTR(1U, *chartInstance->c2_b_r_rgb);
        _SFD_SET_DATA_VALUE_PTR(3U, *chartInstance->c2_R_p);
      }
    }
  }
}

static const char* sf_get_instance_specialization(void)
{
  return "sXMQT00SwubGcrRZ9wKCWDF";
}

static void sf_opaque_initialize_c2_mirageTrackingRobot(void *chartInstanceVar)
{
  chart_debug_initialization(((SFc2_mirageTrackingRobotInstanceStruct*)
    chartInstanceVar)->S,0);
  initialize_params_c2_mirageTrackingRobot
    ((SFc2_mirageTrackingRobotInstanceStruct*) chartInstanceVar);
  initialize_c2_mirageTrackingRobot((SFc2_mirageTrackingRobotInstanceStruct*)
    chartInstanceVar);
}

static void sf_opaque_enable_c2_mirageTrackingRobot(void *chartInstanceVar)
{
  enable_c2_mirageTrackingRobot((SFc2_mirageTrackingRobotInstanceStruct*)
    chartInstanceVar);
}

static void sf_opaque_disable_c2_mirageTrackingRobot(void *chartInstanceVar)
{
  disable_c2_mirageTrackingRobot((SFc2_mirageTrackingRobotInstanceStruct*)
    chartInstanceVar);
}

static void sf_opaque_gateway_c2_mirageTrackingRobot(void *chartInstanceVar)
{
  sf_gateway_c2_mirageTrackingRobot((SFc2_mirageTrackingRobotInstanceStruct*)
    chartInstanceVar);
}

static const mxArray* sf_opaque_get_sim_state_c2_mirageTrackingRobot(SimStruct*
  S)
{
  ChartRunTimeInfo * crtInfo = (ChartRunTimeInfo *)(ssGetUserData(S));
  ChartInfoStruct * chartInfo = (ChartInfoStruct *)(crtInfo->instanceInfo);
  return get_sim_state_c2_mirageTrackingRobot
    ((SFc2_mirageTrackingRobotInstanceStruct*)chartInfo->chartInstance);/* raw sim ctx */
}

static void sf_opaque_set_sim_state_c2_mirageTrackingRobot(SimStruct* S, const
  mxArray *st)
{
  ChartRunTimeInfo * crtInfo = (ChartRunTimeInfo *)(ssGetUserData(S));
  ChartInfoStruct * chartInfo = (ChartInfoStruct *)(crtInfo->instanceInfo);
  set_sim_state_c2_mirageTrackingRobot((SFc2_mirageTrackingRobotInstanceStruct*)
    chartInfo->chartInstance, st);
}

static void sf_opaque_terminate_c2_mirageTrackingRobot(void *chartInstanceVar)
{
  if (chartInstanceVar!=NULL) {
    SimStruct *S = ((SFc2_mirageTrackingRobotInstanceStruct*) chartInstanceVar
      )->S;
    ChartRunTimeInfo * crtInfo = (ChartRunTimeInfo *)(ssGetUserData(S));
    if (sim_mode_is_rtw_gen(S) || sim_mode_is_external(S)) {
      sf_clear_rtw_identifier(S);
      unload_mirageTrackingRobot_optimization_info();
    }

    finalize_c2_mirageTrackingRobot((SFc2_mirageTrackingRobotInstanceStruct*)
      chartInstanceVar);
    utFree(chartInstanceVar);
    if (crtInfo != NULL) {
      utFree(crtInfo);
    }

    ssSetUserData(S,NULL);
  }
}

static void sf_opaque_init_subchart_simstructs(void *chartInstanceVar)
{
  initSimStructsc2_mirageTrackingRobot((SFc2_mirageTrackingRobotInstanceStruct*)
    chartInstanceVar);
}

extern unsigned int sf_machine_global_initializer_called(void);
static void mdlProcessParameters_c2_mirageTrackingRobot(SimStruct *S)
{
  int i;
  for (i=0;i<ssGetNumRunTimeParams(S);i++) {
    if (ssGetSFcnParamTunable(S,i)) {
      ssUpdateDlgParamAsRunTimeParam(S,i);
    }
  }

  if (sf_machine_global_initializer_called()) {
    ChartRunTimeInfo * crtInfo = (ChartRunTimeInfo *)(ssGetUserData(S));
    ChartInfoStruct * chartInfo = (ChartInfoStruct *)(crtInfo->instanceInfo);
    initialize_params_c2_mirageTrackingRobot
      ((SFc2_mirageTrackingRobotInstanceStruct*)(chartInfo->chartInstance));
  }
}

static void mdlSetWorkWidths_c2_mirageTrackingRobot(SimStruct *S)
{
  ssMdlUpdateIsEmpty(S, 1);
  if (sim_mode_is_rtw_gen(S) || sim_mode_is_external(S)) {
    mxArray *infoStruct = load_mirageTrackingRobot_optimization_info();
    int_T chartIsInlinable =
      (int_T)sf_is_chart_inlinable(sf_get_instance_specialization(),infoStruct,2);
    ssSetStateflowIsInlinable(S,chartIsInlinable);
    ssSetRTWCG(S,1);
    ssSetEnableFcnIsTrivial(S,1);
    ssSetDisableFcnIsTrivial(S,1);
    ssSetNotMultipleInlinable(S,sf_rtw_info_uint_prop
      (sf_get_instance_specialization(),infoStruct,2,
       "gatewayCannotBeInlinedMultipleTimes"));
    sf_update_buildInfo(sf_get_instance_specialization(),infoStruct,2);
    if (chartIsInlinable) {
      ssSetInputPortOptimOpts(S, 0, SS_REUSABLE_AND_LOCAL);
      ssSetInputPortOptimOpts(S, 1, SS_REUSABLE_AND_LOCAL);
      sf_mark_chart_expressionable_inputs(S,sf_get_instance_specialization(),
        infoStruct,2,2);
      sf_mark_chart_reusable_outputs(S,sf_get_instance_specialization(),
        infoStruct,2,2);
    }

    {
      unsigned int outPortIdx;
      for (outPortIdx=1; outPortIdx<=2; ++outPortIdx) {
        ssSetOutputPortOptimizeInIR(S, outPortIdx, 1U);
      }
    }

    {
      unsigned int inPortIdx;
      for (inPortIdx=0; inPortIdx < 2; ++inPortIdx) {
        ssSetInputPortOptimizeInIR(S, inPortIdx, 1U);
      }
    }

    sf_set_rtw_dwork_info(S,sf_get_instance_specialization(),infoStruct,2);
    ssSetHasSubFunctions(S,!(chartIsInlinable));
  } else {
  }

  ssSetOptions(S,ssGetOptions(S)|SS_OPTION_WORKS_WITH_CODE_REUSE);
  ssSetChecksum0(S,(4077403510U));
  ssSetChecksum1(S,(2196850246U));
  ssSetChecksum2(S,(1221727109U));
  ssSetChecksum3(S,(946337809U));
  ssSetmdlDerivatives(S, NULL);
  ssSetExplicitFCSSCtrl(S,1);
  ssSupportsMultipleExecInstances(S,1);
}

static void mdlRTW_c2_mirageTrackingRobot(SimStruct *S)
{
  if (sim_mode_is_rtw_gen(S)) {
    ssWriteRTWStrParam(S, "StateflowChartType", "Embedded MATLAB");
  }
}

static void mdlStart_c2_mirageTrackingRobot(SimStruct *S)
{
  SFc2_mirageTrackingRobotInstanceStruct *chartInstance;
  ChartRunTimeInfo * crtInfo = (ChartRunTimeInfo *)utMalloc(sizeof
    (ChartRunTimeInfo));
  chartInstance = (SFc2_mirageTrackingRobotInstanceStruct *)utMalloc(sizeof
    (SFc2_mirageTrackingRobotInstanceStruct));
  memset(chartInstance, 0, sizeof(SFc2_mirageTrackingRobotInstanceStruct));
  if (chartInstance==NULL) {
    sf_mex_error_message("Could not allocate memory for chart instance.");
  }

  chartInstance->chartInfo.chartInstance = chartInstance;
  chartInstance->chartInfo.isEMLChart = 1;
  chartInstance->chartInfo.chartInitialized = 0;
  chartInstance->chartInfo.sFunctionGateway =
    sf_opaque_gateway_c2_mirageTrackingRobot;
  chartInstance->chartInfo.initializeChart =
    sf_opaque_initialize_c2_mirageTrackingRobot;
  chartInstance->chartInfo.terminateChart =
    sf_opaque_terminate_c2_mirageTrackingRobot;
  chartInstance->chartInfo.enableChart = sf_opaque_enable_c2_mirageTrackingRobot;
  chartInstance->chartInfo.disableChart =
    sf_opaque_disable_c2_mirageTrackingRobot;
  chartInstance->chartInfo.getSimState =
    sf_opaque_get_sim_state_c2_mirageTrackingRobot;
  chartInstance->chartInfo.setSimState =
    sf_opaque_set_sim_state_c2_mirageTrackingRobot;
  chartInstance->chartInfo.getSimStateInfo =
    sf_get_sim_state_info_c2_mirageTrackingRobot;
  chartInstance->chartInfo.zeroCrossings = NULL;
  chartInstance->chartInfo.outputs = NULL;
  chartInstance->chartInfo.derivatives = NULL;
  chartInstance->chartInfo.mdlRTW = mdlRTW_c2_mirageTrackingRobot;
  chartInstance->chartInfo.mdlStart = mdlStart_c2_mirageTrackingRobot;
  chartInstance->chartInfo.mdlSetWorkWidths =
    mdlSetWorkWidths_c2_mirageTrackingRobot;
  chartInstance->chartInfo.extModeExec = NULL;
  chartInstance->chartInfo.restoreLastMajorStepConfiguration = NULL;
  chartInstance->chartInfo.restoreBeforeLastMajorStepConfiguration = NULL;
  chartInstance->chartInfo.storeCurrentConfiguration = NULL;
  chartInstance->chartInfo.callAtomicSubchartUserFcn = NULL;
  chartInstance->chartInfo.callAtomicSubchartAutoFcn = NULL;
  chartInstance->chartInfo.debugInstance = sfGlobalDebugInstanceStruct;
  chartInstance->S = S;
  crtInfo->isEnhancedMooreMachine = 0;
  crtInfo->checksum = SF_RUNTIME_INFO_CHECKSUM;
  crtInfo->fCheckOverflow = sf_runtime_overflow_check_is_on(S);
  crtInfo->instanceInfo = (&(chartInstance->chartInfo));
  crtInfo->isJITEnabled = false;
  crtInfo->compiledInfo = NULL;
  ssSetUserData(S,(void *)(crtInfo));  /* register the chart instance with simstruct */
  init_dsm_address_info(chartInstance);
  init_simulink_io_address(chartInstance);
  if (!sim_mode_is_rtw_gen(S)) {
  }

  chart_debug_initialization(S,1);
}

void c2_mirageTrackingRobot_method_dispatcher(SimStruct *S, int_T method, void
  *data)
{
  switch (method) {
   case SS_CALL_MDL_START:
    mdlStart_c2_mirageTrackingRobot(S);
    break;

   case SS_CALL_MDL_SET_WORK_WIDTHS:
    mdlSetWorkWidths_c2_mirageTrackingRobot(S);
    break;

   case SS_CALL_MDL_PROCESS_PARAMETERS:
    mdlProcessParameters_c2_mirageTrackingRobot(S);
    break;

   default:
    /* Unhandled method */
    sf_mex_error_message("Stateflow Internal Error:\n"
                         "Error calling c2_mirageTrackingRobot_method_dispatcher.\n"
                         "Can't handle method %d.\n", method);
    break;
  }
}
