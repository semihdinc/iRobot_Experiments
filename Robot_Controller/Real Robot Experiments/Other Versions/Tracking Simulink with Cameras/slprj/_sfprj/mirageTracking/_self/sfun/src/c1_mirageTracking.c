/* Include files */

#include <stddef.h>
#include "blas.h"
#include "mirageTracking_sfun.h"
#include "c1_mirageTracking.h"
#include "mwmathutil.h"
#define CHARTINSTANCE_CHARTNUMBER      (chartInstance->chartNumber)
#define CHARTINSTANCE_INSTANCENUMBER   (chartInstance->instanceNumber)
#include "mirageTracking_sfun_debug_macros.h"
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
static const char * c1_debug_family_names[22] = { "m", "n", "c", "l_gray",
  "r_gray", "l_rgb1", "r_rgb1", "l_rgb2", "r_rgb2", "l_bw", "r_bw", "l_center",
  "r_center", "ml", "i", "mr", "nargin", "nargout", "l_rgb", "r_rgb", "L_p",
  "R_p" };

static const char * c1_b_debug_family_names[7] = { "minL", "imgTmp", "maxL",
  "imgIn", "nargin", "nargout", "imgOut" };

static const char * c1_c_debug_family_names[10] = { "indx", "indy", "ind4",
  "ind3", "ind1", "ind2", "nargin", "nargout", "p", "new_p" };

/* Function Declarations */
static void initialize_c1_mirageTracking(SFc1_mirageTrackingInstanceStruct
  *chartInstance);
static void initialize_params_c1_mirageTracking
  (SFc1_mirageTrackingInstanceStruct *chartInstance);
static void enable_c1_mirageTracking(SFc1_mirageTrackingInstanceStruct
  *chartInstance);
static void disable_c1_mirageTracking(SFc1_mirageTrackingInstanceStruct
  *chartInstance);
static void c1_update_debugger_state_c1_mirageTracking
  (SFc1_mirageTrackingInstanceStruct *chartInstance);
static const mxArray *get_sim_state_c1_mirageTracking
  (SFc1_mirageTrackingInstanceStruct *chartInstance);
static void set_sim_state_c1_mirageTracking(SFc1_mirageTrackingInstanceStruct
  *chartInstance, const mxArray *c1_st);
static void finalize_c1_mirageTracking(SFc1_mirageTrackingInstanceStruct
  *chartInstance);
static void sf_gateway_c1_mirageTracking(SFc1_mirageTrackingInstanceStruct
  *chartInstance);
static void mdl_start_c1_mirageTracking(SFc1_mirageTrackingInstanceStruct
  *chartInstance);
static void c1_chartstep_c1_mirageTracking(SFc1_mirageTrackingInstanceStruct
  *chartInstance);
static void initSimStructsc1_mirageTracking(SFc1_mirageTrackingInstanceStruct
  *chartInstance);
static void c1_orderPoints(SFc1_mirageTrackingInstanceStruct *chartInstance,
  real_T c1_p[8], real_T c1_new_p[8]);
static void init_script_number_translation(uint32_T c1_machineNumber, uint32_T
  c1_chartNumber, uint32_T c1_instanceNumber);
static void c1_Outputs(SFc1_mirageTrackingInstanceStruct *chartInstance,
  c1_vision_BlobAnalysis_1 *c1_obj, boolean_T c1_U0[1228800], int32_T
  c1_Y0_data[], int32_T c1_Y0_sizes[2], real_T c1_Y1_data[], int32_T
  c1_Y1_sizes[2], int32_T c1_Y2_data[], int32_T c1_Y2_sizes[2]);
static const mxArray *c1_emlrt_marshallOut(SFc1_mirageTrackingInstanceStruct
  *chartInstance, const real_T c1_f_u[1228800]);
static const mxArray *c1_b_emlrt_marshallOut(SFc1_mirageTrackingInstanceStruct
  *chartInstance, const char_T c1_f_u[2]);
static const mxArray *c1_c_emlrt_marshallOut(SFc1_mirageTrackingInstanceStruct
  *chartInstance, const real_T c1_f_u[4]);
static const mxArray *c1_sf_marshallOut(void *chartInstanceVoid, void *c1_inData);
static void c1_emlrt_marshallIn(SFc1_mirageTrackingInstanceStruct *chartInstance,
  const mxArray *c1_b_R_p, const char_T *c1_identifier, real_T c1_f_y[8]);
static void c1_b_emlrt_marshallIn(SFc1_mirageTrackingInstanceStruct
  *chartInstance, const mxArray *c1_f_u, const emlrtMsgIdentifier *c1_parentId,
  real_T c1_f_y[8]);
static void c1_sf_marshallIn(void *chartInstanceVoid, const mxArray
  *c1_mxArrayInData, const char_T *c1_varName, void *c1_outData);
static const mxArray *c1_b_sf_marshallOut(void *chartInstanceVoid, void
  *c1_inData);
static const mxArray *c1_c_sf_marshallOut(void *chartInstanceVoid, void
  *c1_inData);
static real_T c1_c_emlrt_marshallIn(SFc1_mirageTrackingInstanceStruct
  *chartInstance, const mxArray *c1_f_u, const emlrtMsgIdentifier *c1_parentId);
static void c1_b_sf_marshallIn(void *chartInstanceVoid, const mxArray
  *c1_mxArrayInData, const char_T *c1_varName, void *c1_outData);
static const mxArray *c1_d_sf_marshallOut(void *chartInstanceVoid, void
  *c1_inData);
static void c1_d_emlrt_marshallIn(SFc1_mirageTrackingInstanceStruct
  *chartInstance, const mxArray *c1_f_u, const emlrtMsgIdentifier *c1_parentId,
  real32_T c1_f_y[1228800]);
static void c1_c_sf_marshallIn(void *chartInstanceVoid, const mxArray
  *c1_mxArrayInData, const char_T *c1_varName, void *c1_outData);
static void c1_e_emlrt_marshallIn(SFc1_mirageTrackingInstanceStruct
  *chartInstance, const mxArray *c1_f_u, const emlrtMsgIdentifier *c1_parentId,
  real32_T c1_f_y[3686400]);
static void c1_d_sf_marshallIn(void *chartInstanceVoid, const mxArray
  *c1_mxArrayInData, const char_T *c1_varName, void *c1_outData);
static const mxArray *c1_e_sf_marshallOut(void *chartInstanceVoid, real_T
  c1_inData_data[], int32_T c1_inData_sizes[2]);
static void c1_f_emlrt_marshallIn(SFc1_mirageTrackingInstanceStruct
  *chartInstance, const mxArray *c1_f_u, const emlrtMsgIdentifier *c1_parentId,
  real_T c1_y_data[], int32_T c1_y_sizes[2]);
static void c1_e_sf_marshallIn(void *chartInstanceVoid, const mxArray
  *c1_mxArrayInData, const char_T *c1_varName, real_T c1_outData_data[], int32_T
  c1_outData_sizes[2]);
static const mxArray *c1_f_sf_marshallOut(void *chartInstanceVoid, void
  *c1_inData);
static void c1_g_emlrt_marshallIn(SFc1_mirageTrackingInstanceStruct
  *chartInstance, const mxArray *c1_f_u, const emlrtMsgIdentifier *c1_parentId,
  boolean_T c1_f_y[1228800]);
static void c1_f_sf_marshallIn(void *chartInstanceVoid, const mxArray
  *c1_mxArrayInData, const char_T *c1_varName, void *c1_outData);
static const mxArray *c1_g_sf_marshallOut(void *chartInstanceVoid, void
  *c1_inData);
static void c1_h_emlrt_marshallIn(SFc1_mirageTrackingInstanceStruct
  *chartInstance, const mxArray *c1_f_u, const emlrtMsgIdentifier *c1_parentId,
  real_T c1_f_y[1228800]);
static void c1_g_sf_marshallIn(void *chartInstanceVoid, const mxArray
  *c1_mxArrayInData, const char_T *c1_varName, void *c1_outData);
static const mxArray *c1_h_sf_marshallOut(void *chartInstanceVoid, void
  *c1_inData);
static void c1_i_emlrt_marshallIn(SFc1_mirageTrackingInstanceStruct
  *chartInstance, const mxArray *c1_f_u, const emlrtMsgIdentifier *c1_parentId,
  real_T c1_f_y[3686400]);
static void c1_h_sf_marshallIn(void *chartInstanceVoid, const mxArray
  *c1_mxArrayInData, const char_T *c1_varName, void *c1_outData);
static const mxArray *c1_i_sf_marshallOut(void *chartInstanceVoid, void
  *c1_inData);
static void c1_j_emlrt_marshallIn(SFc1_mirageTrackingInstanceStruct
  *chartInstance, const mxArray *c1_f_u, const emlrtMsgIdentifier *c1_parentId,
  real_T c1_f_y[4]);
static void c1_i_sf_marshallIn(void *chartInstanceVoid, const mxArray
  *c1_mxArrayInData, const char_T *c1_varName, void *c1_outData);
static void c1_info_helper(const mxArray **c1_info);
static const mxArray *c1_d_emlrt_marshallOut(const char * c1_f_u);
static const mxArray *c1_e_emlrt_marshallOut(const uint32_T c1_f_u);
static void c1_imcomplement(SFc1_mirageTrackingInstanceStruct *chartInstance,
  real32_T c1_im[3686400], real32_T c1_b_im[3686400]);
static void c1_medfilt2(SFc1_mirageTrackingInstanceStruct *chartInstance,
  real32_T c1_g_varargin_1[1228800], real32_T c1_d_b[1228800]);
static void c1_padarray(SFc1_mirageTrackingInstanceStruct *chartInstance,
  real32_T c1_g_varargin_1[1228800], real32_T c1_d_b[1233284]);
static c1_coder_internal_cell_4 c1_cell_cell(SFc1_mirageTrackingInstanceStruct
  *chartInstance);
static c1_coder_internal_cell_5 c1_b_cell_cell(SFc1_mirageTrackingInstanceStruct
  *chartInstance);
static c1_coder_internal_cell_6 c1_c_cell_cell(SFc1_mirageTrackingInstanceStruct
  *chartInstance);
static c1_coder_internal_cell_7 c1_d_cell_cell(SFc1_mirageTrackingInstanceStruct
  *chartInstance);
static void c1_myMat2gray(SFc1_mirageTrackingInstanceStruct *chartInstance,
  real32_T c1_b_imgIn[1228800], real_T c1_imgOut[1228800]);
static void c1_check_forloop_overflow_error(SFc1_mirageTrackingInstanceStruct
  *chartInstance, boolean_T c1_overflow);
static void c1_System_System(SFc1_mirageTrackingInstanceStruct *chartInstance,
  c1_visioncodegen_BlobAnalysis *c1_obj);
static c1_visioncodegen_BlobAnalysis *c1_BlobAnalysis_BlobAnalysis
  (SFc1_mirageTrackingInstanceStruct *chartInstance,
   c1_visioncodegen_BlobAnalysis *c1_obj);
static void c1_SystemCore_step(SFc1_mirageTrackingInstanceStruct *chartInstance,
  c1_visioncodegen_BlobAnalysis *c1_obj, boolean_T c1_g_varargin_1[1228800],
  int32_T c1_varargout_1_data[], int32_T c1_varargout_1_sizes[2], real_T
  c1_varargout_2_data[], int32_T c1_varargout_2_sizes[2], int32_T
  c1_varargout_3_data[], int32_T c1_varargout_3_sizes[2]);
static void c1_Nondirect_stepImpl(SFc1_mirageTrackingInstanceStruct
  *chartInstance, c1_visioncodegen_BlobAnalysis *c1_obj, boolean_T
  c1_g_varargin_1[1228800], int32_T c1_varargout_1_data[], int32_T
  c1_varargout_1_sizes[2], real_T c1_varargout_2_data[], int32_T
  c1_varargout_2_sizes[2], int32_T c1_varargout_3_data[], int32_T
  c1_varargout_3_sizes[2]);
static void c1_b_SystemCore_step(SFc1_mirageTrackingInstanceStruct
  *chartInstance, c1_visioncodegen_BlobAnalysis *c1_obj, boolean_T
  c1_g_varargin_1[1228800], int32_T c1_varargout_1_data[], int32_T
  c1_varargout_1_sizes[2], real_T c1_varargout_2_data[], int32_T
  c1_varargout_2_sizes[2], int32_T c1_varargout_3_data[], int32_T
  c1_varargout_3_sizes[2]);
static void c1_b_Nondirect_stepImpl(SFc1_mirageTrackingInstanceStruct
  *chartInstance, c1_visioncodegen_BlobAnalysis *c1_obj, boolean_T
  c1_g_varargin_1[1228800], int32_T c1_varargout_1_data[], int32_T
  c1_varargout_1_sizes[2], real_T c1_varargout_2_data[], int32_T
  c1_varargout_2_sizes[2], int32_T c1_varargout_3_data[], int32_T
  c1_varargout_3_sizes[2]);
static void c1_eml_sort(SFc1_mirageTrackingInstanceStruct *chartInstance, real_T
  c1_x[4], real_T c1_b_x[4], int32_T c1_idx[4]);
static void c1_eml_sort_idx(SFc1_mirageTrackingInstanceStruct *chartInstance,
  real_T c1_x[4], int32_T c1_idx[4], real_T c1_b_x[4]);
static void c1_merge(SFc1_mirageTrackingInstanceStruct *chartInstance, int32_T
                     c1_idx[4], real_T c1_x[4], int32_T c1_offset, int32_T c1_np,
                     int32_T c1_nq, int32_T c1_b_idx[4], real_T c1_b_x[4]);
static const mxArray *c1_j_sf_marshallOut(void *chartInstanceVoid, void
  *c1_inData);
static int32_T c1_k_emlrt_marshallIn(SFc1_mirageTrackingInstanceStruct
  *chartInstance, const mxArray *c1_f_u, const emlrtMsgIdentifier *c1_parentId);
static void c1_j_sf_marshallIn(void *chartInstanceVoid, const mxArray
  *c1_mxArrayInData, const char_T *c1_varName, void *c1_outData);
static boolean_T c1_l_emlrt_marshallIn(SFc1_mirageTrackingInstanceStruct
  *chartInstance, const mxArray *c1_b_isInitialized, const char_T *c1_identifier);
static boolean_T c1_m_emlrt_marshallIn(SFc1_mirageTrackingInstanceStruct
  *chartInstance, const mxArray *c1_f_u, const emlrtMsgIdentifier *c1_parentId);
static uint8_T c1_n_emlrt_marshallIn(SFc1_mirageTrackingInstanceStruct
  *chartInstance, const mxArray *c1_b_is_active_c1_mirageTracking, const char_T *
  c1_identifier);
static uint8_T c1_o_emlrt_marshallIn(SFc1_mirageTrackingInstanceStruct
  *chartInstance, const mxArray *c1_f_u, const emlrtMsgIdentifier *c1_parentId);
static void c1_b_imcomplement(SFc1_mirageTrackingInstanceStruct *chartInstance,
  real32_T c1_im[3686400]);
static void c1_b_eml_sort(SFc1_mirageTrackingInstanceStruct *chartInstance,
  real_T c1_x[4], int32_T c1_idx[4]);
static void c1_b_eml_sort_idx(SFc1_mirageTrackingInstanceStruct *chartInstance,
  real_T c1_x[4], int32_T c1_idx[4]);
static void c1_b_merge(SFc1_mirageTrackingInstanceStruct *chartInstance, int32_T
  c1_idx[4], real_T c1_x[4], int32_T c1_offset, int32_T c1_np, int32_T c1_nq);
static void init_dsm_address_info(SFc1_mirageTrackingInstanceStruct
  *chartInstance);
static void init_simulink_io_address(SFc1_mirageTrackingInstanceStruct
  *chartInstance);

/* Function Definitions */
static void initialize_c1_mirageTracking(SFc1_mirageTrackingInstanceStruct
  *chartInstance)
{
  if (sf_is_first_init_cond(chartInstance->S)) {
    initSimStructsc1_mirageTracking(chartInstance);
    chart_debug_initialize_data_addresses(chartInstance->S);
  }

  chartInstance->c1_sfEvent = CALL_EVENT;
  _sfTime_ = sf_get_time(chartInstance->S);
  chartInstance->c1_is_active_c1_mirageTracking = 0U;
}

static void initialize_params_c1_mirageTracking
  (SFc1_mirageTrackingInstanceStruct *chartInstance)
{
  (void)chartInstance;
}

static void enable_c1_mirageTracking(SFc1_mirageTrackingInstanceStruct
  *chartInstance)
{
  _sfTime_ = sf_get_time(chartInstance->S);
}

static void disable_c1_mirageTracking(SFc1_mirageTrackingInstanceStruct
  *chartInstance)
{
  _sfTime_ = sf_get_time(chartInstance->S);
}

static void c1_update_debugger_state_c1_mirageTracking
  (SFc1_mirageTrackingInstanceStruct *chartInstance)
{
  (void)chartInstance;
}

static const mxArray *get_sim_state_c1_mirageTracking
  (SFc1_mirageTrackingInstanceStruct *chartInstance)
{
  const mxArray *c1_st;
  const mxArray *c1_f_y = NULL;
  const mxArray *c1_g_y = NULL;
  const mxArray *c1_h_y = NULL;
  boolean_T c1_hoistedGlobal;
  boolean_T c1_f_u;
  const mxArray *c1_i_y = NULL;
  uint8_T c1_b_hoistedGlobal;
  uint8_T c1_g_u;
  const mxArray *c1_j_y = NULL;
  c1_st = NULL;
  c1_st = NULL;
  c1_f_y = NULL;
  sf_mex_assign(&c1_f_y, sf_mex_createcellmatrix(4, 1), false);
  c1_g_y = NULL;
  sf_mex_assign(&c1_g_y, sf_mex_create("y", *chartInstance->c1_L_p, 0, 0U, 1U,
    0U, 2, 2, 4), false);
  sf_mex_setcell(c1_f_y, 0, c1_g_y);
  c1_h_y = NULL;
  sf_mex_assign(&c1_h_y, sf_mex_create("y", *chartInstance->c1_R_p, 0, 0U, 1U,
    0U, 2, 2, 4), false);
  sf_mex_setcell(c1_f_y, 1, c1_h_y);
  c1_hoistedGlobal = chartInstance->c1_isInitialized;
  c1_f_u = c1_hoistedGlobal;
  c1_i_y = NULL;
  sf_mex_assign(&c1_i_y, sf_mex_create("y", &c1_f_u, 11, 0U, 0U, 0U, 0), false);
  sf_mex_setcell(c1_f_y, 2, c1_i_y);
  c1_b_hoistedGlobal = chartInstance->c1_is_active_c1_mirageTracking;
  c1_g_u = c1_b_hoistedGlobal;
  c1_j_y = NULL;
  sf_mex_assign(&c1_j_y, sf_mex_create("y", &c1_g_u, 3, 0U, 0U, 0U, 0), false);
  sf_mex_setcell(c1_f_y, 3, c1_j_y);
  sf_mex_assign(&c1_st, c1_f_y, false);
  return c1_st;
}

static void set_sim_state_c1_mirageTracking(SFc1_mirageTrackingInstanceStruct
  *chartInstance, const mxArray *c1_st)
{
  const mxArray *c1_f_u;
  real_T c1_dv0[8];
  int32_T c1_i0;
  real_T c1_dv1[8];
  int32_T c1_i1;
  chartInstance->c1_doneDoubleBufferReInit = true;
  c1_f_u = sf_mex_dup(c1_st);
  c1_emlrt_marshallIn(chartInstance, sf_mex_dup(sf_mex_getcell("L_p", c1_f_u, 0)),
                      "L_p", c1_dv0);
  for (c1_i0 = 0; c1_i0 < 8; c1_i0++) {
    (*chartInstance->c1_L_p)[c1_i0] = c1_dv0[c1_i0];
  }

  c1_emlrt_marshallIn(chartInstance, sf_mex_dup(sf_mex_getcell("R_p", c1_f_u, 1)),
                      "R_p", c1_dv1);
  for (c1_i1 = 0; c1_i1 < 8; c1_i1++) {
    (*chartInstance->c1_R_p)[c1_i1] = c1_dv1[c1_i1];
  }

  chartInstance->c1_isInitialized = c1_l_emlrt_marshallIn(chartInstance,
    sf_mex_dup(sf_mex_getcell("isInitialized", c1_f_u, 2)), "isInitialized");
  chartInstance->c1_is_active_c1_mirageTracking = c1_n_emlrt_marshallIn
    (chartInstance, sf_mex_dup(sf_mex_getcell("is_active_c1_mirageTracking",
       c1_f_u, 3)), "is_active_c1_mirageTracking");
  sf_mex_destroy(&c1_f_u);
  c1_update_debugger_state_c1_mirageTracking(chartInstance);
  sf_mex_destroy(&c1_st);
}

static void finalize_c1_mirageTracking(SFc1_mirageTrackingInstanceStruct
  *chartInstance)
{
  (void)chartInstance;
}

static void sf_gateway_c1_mirageTracking(SFc1_mirageTrackingInstanceStruct
  *chartInstance)
{
  int32_T c1_i2;
  int32_T c1_i3;
  int32_T c1_i4;
  int32_T c1_i5;
  _SFD_SYMBOL_SCOPE_PUSH(0U, 0U);
  _sfTime_ = sf_get_time(chartInstance->S);
  _SFD_CC_CALL(CHART_ENTER_SFUNCTION_TAG, 0U, chartInstance->c1_sfEvent);
  for (c1_i2 = 0; c1_i2 < 3686400; c1_i2++) {
    _SFD_DATA_RANGE_CHECK((real_T)(*chartInstance->c1_b_r_rgb)[c1_i2], 1U, 1U,
                          0U, chartInstance->c1_sfEvent, false);
  }

  for (c1_i3 = 0; c1_i3 < 3686400; c1_i3++) {
    _SFD_DATA_RANGE_CHECK((real_T)(*chartInstance->c1_b_l_rgb)[c1_i3], 0U, 1U,
                          0U, chartInstance->c1_sfEvent, false);
  }

  chartInstance->c1_sfEvent = CALL_EVENT;
  c1_chartstep_c1_mirageTracking(chartInstance);
  _SFD_SYMBOL_SCOPE_POP();
  _SFD_CHECK_FOR_STATE_INCONSISTENCY(_mirageTrackingMachineNumber_,
    chartInstance->chartNumber, chartInstance->instanceNumber);
  for (c1_i4 = 0; c1_i4 < 8; c1_i4++) {
    _SFD_DATA_RANGE_CHECK((*chartInstance->c1_L_p)[c1_i4], 2U, 1U, 0U,
                          chartInstance->c1_sfEvent, false);
  }

  for (c1_i5 = 0; c1_i5 < 8; c1_i5++) {
    _SFD_DATA_RANGE_CHECK((*chartInstance->c1_R_p)[c1_i5], 3U, 1U, 0U,
                          chartInstance->c1_sfEvent, false);
  }
}

static void mdl_start_c1_mirageTracking(SFc1_mirageTrackingInstanceStruct
  *chartInstance)
{
  (void)chartInstance;
}

static void c1_chartstep_c1_mirageTracking(SFc1_mirageTrackingInstanceStruct
  *chartInstance)
{
  int32_T c1_i6;
  int32_T c1_i7;
  uint32_T c1_debug_family_var_map[22];
  real_T c1_m;
  real_T c1_n;
  real_T c1_c;
  int32_T c1_l_center_sizes[2];
  real_T c1_l_center_data[100];
  int32_T c1_r_center_sizes[2];
  real_T c1_r_center_data[100];
  real_T c1_ml;
  real_T c1_i;
  real_T c1_mr;
  real_T c1_nargin = 2.0;
  real_T c1_nargout = 2.0;
  real_T c1_b_L_p[8];
  real_T c1_b_R_p[8];
  int32_T c1_i8;
  int32_T c1_i9;
  int32_T c1_i10;
  int32_T c1_i11;
  int32_T c1_i12;
  int32_T c1_i13;
  int32_T c1_i14;
  int32_T c1_i15;
  int32_T c1_i16;
  int32_T c1_i17;
  int32_T c1_i18;
  int32_T c1_i19;
  int32_T c1_i20;
  int32_T c1_i21;
  int32_T c1_i22;
  int32_T c1_i23;
  int32_T c1_i24;
  int32_T c1_i25;
  int32_T c1_i26;
  int32_T c1_i27;
  int32_T c1_i28;
  int32_T c1_i29;
  int32_T c1_i30;
  int32_T c1_i31;
  int32_T c1_i32;
  int32_T c1_i33;
  int32_T c1_i34;
  int32_T c1_i35;
  int32_T c1_i36;
  int32_T c1_i37;
  int32_T c1_i38;
  int32_T c1_i39;
  int32_T c1_i40;
  int32_T c1_i41;
  int32_T c1_i42;
  int32_T c1_i43;
  int32_T c1_i44;
  int32_T c1_i45;
  int32_T c1_i46;
  int32_T c1_i47;
  int32_T c1_i48;
  int32_T c1_i49;
  int32_T c1_i50;
  int32_T c1_i51;
  int32_T c1_i52;
  int32_T c1_i53;
  int32_T c1_i54;
  int32_T c1_i55;
  int32_T c1_i56;
  int32_T c1_i57;
  int32_T c1_i58;
  int32_T c1_i59;
  int32_T c1_i60;
  int32_T c1_i61;
  int32_T c1_i62;
  int32_T c1_i63;
  int32_T c1_i64;
  int32_T c1_unusedU1_sizes[2];
  int32_T c1_unusedU1_data[200];
  int32_T c1_b_l_center_sizes[2];
  real_T c1_b_l_center_data[100];
  int32_T c1_unusedU0_sizes[2];
  int32_T c1_unusedU0_data[50];
  int32_T c1_l_center;
  int32_T c1_b_l_center;
  int32_T c1_loop_ub;
  int32_T c1_i65;
  int32_T c1_i66;
  int32_T c1_r_center;
  int32_T c1_b_r_center;
  int32_T c1_b_loop_ub;
  int32_T c1_i67;
  real_T c1_b_ml;
  int32_T c1_i68;
  int32_T c1_b_i;
  int32_T c1_i69;
  int32_T c1_i70;
  int32_T c1_c_i;
  int32_T c1_tmp_sizes[2];
  int32_T c1_c_loop_ub;
  int32_T c1_i71;
  real_T c1_tmp_data[2];
  static int32_T c1_iv0[1] = { 2 };

  int32_T c1_i72;
  int32_T c1_i73;
  real_T c1_b_mr;
  int32_T c1_i74;
  int32_T c1_d_i;
  int32_T c1_i75;
  int32_T c1_i76;
  int32_T c1_e_i;
  int32_T c1_d_loop_ub;
  int32_T c1_i77;
  int32_T c1_i78;
  int32_T c1_i79;
  int32_T c1_i80;
  real_T c1_c_L_p[8];
  real_T c1_dv4[8];
  int32_T c1_i81;
  int32_T c1_i82;
  real_T c1_c_R_p[8];
  real_T c1_dv5[8];
  int32_T c1_i83;
  static char_T c1_cv0[2] = { 'o', 'n' };

  int32_T c1_i84;
  int32_T c1_i85;
  real_T c1_d_L_p[4];
  int32_T c1_i86;
  int32_T c1_i87;
  real_T c1_e_L_p[4];
  static char_T c1_cv1[2] = { 'r', 'o' };

  int32_T c1_i88;
  int32_T c1_i89;
  int32_T c1_i90;
  int32_T c1_i91;
  int32_T c1_i92;
  int32_T c1_i93;
  _SFD_CC_CALL(CHART_ENTER_DURING_FUNCTION_TAG, 0U, chartInstance->c1_sfEvent);
  for (c1_i6 = 0; c1_i6 < 3686400; c1_i6++) {
    chartInstance->c1_l_rgb[c1_i6] = (*chartInstance->c1_b_l_rgb)[c1_i6];
  }

  for (c1_i7 = 0; c1_i7 < 3686400; c1_i7++) {
    chartInstance->c1_r_rgb[c1_i7] = (*chartInstance->c1_b_r_rgb)[c1_i7];
  }

  _SFD_SYMBOL_SCOPE_PUSH_EML(0U, 22U, 28U, c1_debug_family_names,
    c1_debug_family_var_map);
  _SFD_SYMBOL_SCOPE_ADD_EML(&c1_m, 0U, c1_c_sf_marshallOut);
  _SFD_SYMBOL_SCOPE_ADD_EML(&c1_n, 1U, c1_c_sf_marshallOut);
  _SFD_SYMBOL_SCOPE_ADD_EML(&c1_c, 2U, c1_c_sf_marshallOut);
  _SFD_SYMBOL_SCOPE_ADD_EML_IMPORTABLE(chartInstance->c1_l_gray, MAX_uint32_T,
    c1_g_sf_marshallOut, c1_g_sf_marshallIn);
  _SFD_SYMBOL_SCOPE_ADD_EML_IMPORTABLE(chartInstance->c1_r_gray, MAX_uint32_T,
    c1_g_sf_marshallOut, c1_g_sf_marshallIn);
  _SFD_SYMBOL_SCOPE_ADD_EML_IMPORTABLE(chartInstance->c1_l_rgb1, MAX_uint32_T,
    c1_h_sf_marshallOut, c1_h_sf_marshallIn);
  _SFD_SYMBOL_SCOPE_ADD_EML_IMPORTABLE(chartInstance->c1_r_rgb1, MAX_uint32_T,
    c1_h_sf_marshallOut, c1_h_sf_marshallIn);
  _SFD_SYMBOL_SCOPE_ADD_EML_IMPORTABLE(chartInstance->c1_l_rgb2, MAX_uint32_T,
    c1_g_sf_marshallOut, c1_g_sf_marshallIn);
  _SFD_SYMBOL_SCOPE_ADD_EML_IMPORTABLE(chartInstance->c1_r_rgb2, MAX_uint32_T,
    c1_g_sf_marshallOut, c1_g_sf_marshallIn);
  _SFD_SYMBOL_SCOPE_ADD_EML_IMPORTABLE(chartInstance->c1_l_bw, 9U,
    c1_f_sf_marshallOut, c1_f_sf_marshallIn);
  _SFD_SYMBOL_SCOPE_ADD_EML_IMPORTABLE(chartInstance->c1_r_bw, 10U,
    c1_f_sf_marshallOut, c1_f_sf_marshallIn);
  _SFD_SYMBOL_SCOPE_ADD_EML_DYN_IMPORTABLE(c1_l_center_data, (const int32_T *)
    &c1_l_center_sizes, NULL, 0, 11, (void *)c1_e_sf_marshallOut, (void *)
    c1_e_sf_marshallIn);
  _SFD_SYMBOL_SCOPE_ADD_EML_DYN_IMPORTABLE(c1_r_center_data, (const int32_T *)
    &c1_r_center_sizes, NULL, 0, 12, (void *)c1_e_sf_marshallOut, (void *)
    c1_e_sf_marshallIn);
  _SFD_SYMBOL_SCOPE_ADD_EML_IMPORTABLE(&c1_ml, 13U, c1_c_sf_marshallOut,
    c1_b_sf_marshallIn);
  _SFD_SYMBOL_SCOPE_ADD_EML_IMPORTABLE(&c1_i, 14U, c1_c_sf_marshallOut,
    c1_b_sf_marshallIn);
  _SFD_SYMBOL_SCOPE_ADD_EML_IMPORTABLE(&c1_mr, 15U, c1_c_sf_marshallOut,
    c1_b_sf_marshallIn);
  _SFD_SYMBOL_SCOPE_ADD_EML_IMPORTABLE(chartInstance->c1_b_l_rgb1, MAX_uint32_T,
    c1_b_sf_marshallOut, c1_d_sf_marshallIn);
  _SFD_SYMBOL_SCOPE_ADD_EML_IMPORTABLE(chartInstance->c1_b_r_rgb1, MAX_uint32_T,
    c1_b_sf_marshallOut, c1_d_sf_marshallIn);
  _SFD_SYMBOL_SCOPE_ADD_EML_IMPORTABLE(chartInstance->c1_b_l_gray, MAX_uint32_T,
    c1_d_sf_marshallOut, c1_c_sf_marshallIn);
  _SFD_SYMBOL_SCOPE_ADD_EML_IMPORTABLE(chartInstance->c1_b_r_gray, MAX_uint32_T,
    c1_d_sf_marshallOut, c1_c_sf_marshallIn);
  _SFD_SYMBOL_SCOPE_ADD_EML_IMPORTABLE(chartInstance->c1_b_l_rgb2, MAX_uint32_T,
    c1_d_sf_marshallOut, c1_c_sf_marshallIn);
  _SFD_SYMBOL_SCOPE_ADD_EML_IMPORTABLE(chartInstance->c1_b_r_rgb2, MAX_uint32_T,
    c1_d_sf_marshallOut, c1_c_sf_marshallIn);
  _SFD_SYMBOL_SCOPE_ADD_EML_IMPORTABLE(&c1_nargin, 16U, c1_c_sf_marshallOut,
    c1_b_sf_marshallIn);
  _SFD_SYMBOL_SCOPE_ADD_EML_IMPORTABLE(&c1_nargout, 17U, c1_c_sf_marshallOut,
    c1_b_sf_marshallIn);
  _SFD_SYMBOL_SCOPE_ADD_EML(chartInstance->c1_l_rgb, 18U, c1_b_sf_marshallOut);
  _SFD_SYMBOL_SCOPE_ADD_EML(chartInstance->c1_r_rgb, 19U, c1_b_sf_marshallOut);
  _SFD_SYMBOL_SCOPE_ADD_EML_IMPORTABLE(c1_b_L_p, 20U, c1_sf_marshallOut,
    c1_sf_marshallIn);
  _SFD_SYMBOL_SCOPE_ADD_EML_IMPORTABLE(c1_b_R_p, 21U, c1_sf_marshallOut,
    c1_sf_marshallIn);
  CV_EML_FCN(0, 0);
  _SFD_EML_CALL(0U, chartInstance->c1_sfEvent, 4);
  for (c1_i8 = 0; c1_i8 < 8; c1_i8++) {
    c1_b_L_p[c1_i8] = 0.0;
  }

  _SFD_EML_CALL(0U, chartInstance->c1_sfEvent, 5);
  for (c1_i9 = 0; c1_i9 < 8; c1_i9++) {
    c1_b_R_p[c1_i9] = 0.0;
  }

  _SFD_EML_CALL(0U, chartInstance->c1_sfEvent, 8);
  c1_m = 960.0;
  c1_n = 1280.0;
  c1_c = 3.0;
  _SFD_EML_CALL(0U, chartInstance->c1_sfEvent, 11);
  for (c1_i10 = 0; c1_i10 < 1228800; c1_i10++) {
    chartInstance->c1_l_gray[c1_i10] = 0.0;
  }

  _SFD_SYMBOL_SWITCH(3U, 4U);
  _SFD_EML_CALL(0U, chartInstance->c1_sfEvent, 11);
  for (c1_i11 = 0; c1_i11 < 1228800; c1_i11++) {
    chartInstance->c1_r_gray[c1_i11] = 0.0;
  }

  _SFD_SYMBOL_SWITCH(4U, 5U);
  _SFD_EML_CALL(0U, chartInstance->c1_sfEvent, 12);
  for (c1_i12 = 0; c1_i12 < 3686400; c1_i12++) {
    chartInstance->c1_l_rgb1[c1_i12] = 0.0;
  }

  _SFD_SYMBOL_SWITCH(5U, 6U);
  _SFD_EML_CALL(0U, chartInstance->c1_sfEvent, 12);
  for (c1_i13 = 0; c1_i13 < 3686400; c1_i13++) {
    chartInstance->c1_r_rgb1[c1_i13] = 0.0;
  }

  _SFD_SYMBOL_SWITCH(6U, 7U);
  _SFD_EML_CALL(0U, chartInstance->c1_sfEvent, 13);
  for (c1_i14 = 0; c1_i14 < 1228800; c1_i14++) {
    chartInstance->c1_l_rgb2[c1_i14] = 0.0;
  }

  _SFD_SYMBOL_SWITCH(7U, 8U);
  _SFD_EML_CALL(0U, chartInstance->c1_sfEvent, 13);
  for (c1_i15 = 0; c1_i15 < 1228800; c1_i15++) {
    chartInstance->c1_r_rgb2[c1_i15] = 0.0;
  }

  _SFD_SYMBOL_SWITCH(8U, 9U);
  _SFD_EML_CALL(0U, chartInstance->c1_sfEvent, 15);
  for (c1_i16 = 0; c1_i16 < 1228800; c1_i16++) {
    chartInstance->c1_l_bw[c1_i16] = false;
  }

  _SFD_EML_CALL(0U, chartInstance->c1_sfEvent, 15);
  for (c1_i17 = 0; c1_i17 < 1228800; c1_i17++) {
    chartInstance->c1_r_bw[c1_i17] = false;
  }

  _SFD_EML_CALL(0U, chartInstance->c1_sfEvent, 17);
  for (c1_i18 = 0; c1_i18 < 3686400; c1_i18++) {
    chartInstance->c1_fv0[c1_i18] = chartInstance->c1_l_rgb[c1_i18];
  }

  c1_b_imcomplement(chartInstance, chartInstance->c1_fv0);
  for (c1_i19 = 0; c1_i19 < 3686400; c1_i19++) {
    chartInstance->c1_b_l_rgb1[c1_i19] = chartInstance->c1_fv0[c1_i19];
  }

  _SFD_SYMBOL_SWITCH(5U, 17U);
  _SFD_EML_CALL(0U, chartInstance->c1_sfEvent, 18);
  for (c1_i20 = 0; c1_i20 < 3686400; c1_i20++) {
    chartInstance->c1_fv0[c1_i20] = chartInstance->c1_r_rgb[c1_i20];
  }

  c1_b_imcomplement(chartInstance, chartInstance->c1_fv0);
  for (c1_i21 = 0; c1_i21 < 3686400; c1_i21++) {
    chartInstance->c1_b_r_rgb1[c1_i21] = chartInstance->c1_fv0[c1_i21];
  }

  _SFD_SYMBOL_SWITCH(6U, 18U);
  _SFD_EML_CALL(0U, chartInstance->c1_sfEvent, 20);
  c1_i22 = 0;
  for (c1_i23 = 0; c1_i23 < 1280; c1_i23++) {
    for (c1_i24 = 0; c1_i24 < 960; c1_i24++) {
      chartInstance->c1_b[c1_i24 + c1_i22] = chartInstance->c1_b_l_rgb1[c1_i24 +
        c1_i22];
    }

    c1_i22 += 960;
  }

  for (c1_i25 = 0; c1_i25 < 1228800; c1_i25++) {
    chartInstance->c1_b[c1_i25] *= 0.2989F;
  }

  c1_i26 = 0;
  for (c1_i27 = 0; c1_i27 < 1280; c1_i27++) {
    for (c1_i28 = 0; c1_i28 < 960; c1_i28++) {
      chartInstance->c1_b_b[c1_i28 + c1_i26] = chartInstance->c1_b_l_rgb1
        [(c1_i28 + c1_i26) + 1228800];
    }

    c1_i26 += 960;
  }

  for (c1_i29 = 0; c1_i29 < 1228800; c1_i29++) {
    chartInstance->c1_b_b[c1_i29] *= 0.587F;
  }

  c1_i30 = 0;
  for (c1_i31 = 0; c1_i31 < 1280; c1_i31++) {
    for (c1_i32 = 0; c1_i32 < 960; c1_i32++) {
      chartInstance->c1_c_b[c1_i32 + c1_i30] = chartInstance->c1_b_l_rgb1
        [(c1_i32 + c1_i30) + 2457600];
    }

    c1_i30 += 960;
  }

  for (c1_i33 = 0; c1_i33 < 1228800; c1_i33++) {
    chartInstance->c1_c_b[c1_i33] *= 0.114F;
  }

  for (c1_i34 = 0; c1_i34 < 1228800; c1_i34++) {
    chartInstance->c1_b_l_gray[c1_i34] = (chartInstance->c1_b[c1_i34] +
      chartInstance->c1_b_b[c1_i34]) + chartInstance->c1_c_b[c1_i34];
  }

  _SFD_SYMBOL_SWITCH(3U, 19U);
  _SFD_EML_CALL(0U, chartInstance->c1_sfEvent, 21);
  c1_i35 = 0;
  for (c1_i36 = 0; c1_i36 < 1280; c1_i36++) {
    for (c1_i37 = 0; c1_i37 < 960; c1_i37++) {
      chartInstance->c1_b[c1_i37 + c1_i35] = chartInstance->c1_b_r_rgb1[c1_i37 +
        c1_i35];
    }

    c1_i35 += 960;
  }

  for (c1_i38 = 0; c1_i38 < 1228800; c1_i38++) {
    chartInstance->c1_b[c1_i38] *= 0.2989F;
  }

  c1_i39 = 0;
  for (c1_i40 = 0; c1_i40 < 1280; c1_i40++) {
    for (c1_i41 = 0; c1_i41 < 960; c1_i41++) {
      chartInstance->c1_b_b[c1_i41 + c1_i39] = chartInstance->c1_b_r_rgb1
        [(c1_i41 + c1_i39) + 1228800];
    }

    c1_i39 += 960;
  }

  for (c1_i42 = 0; c1_i42 < 1228800; c1_i42++) {
    chartInstance->c1_b_b[c1_i42] *= 0.587F;
  }

  c1_i43 = 0;
  for (c1_i44 = 0; c1_i44 < 1280; c1_i44++) {
    for (c1_i45 = 0; c1_i45 < 960; c1_i45++) {
      chartInstance->c1_c_b[c1_i45 + c1_i43] = chartInstance->c1_b_r_rgb1
        [(c1_i45 + c1_i43) + 2457600];
    }

    c1_i43 += 960;
  }

  for (c1_i46 = 0; c1_i46 < 1228800; c1_i46++) {
    chartInstance->c1_c_b[c1_i46] *= 0.114F;
  }

  for (c1_i47 = 0; c1_i47 < 1228800; c1_i47++) {
    chartInstance->c1_b_r_gray[c1_i47] = (chartInstance->c1_b[c1_i47] +
      chartInstance->c1_b_b[c1_i47]) + chartInstance->c1_c_b[c1_i47];
  }

  _SFD_SYMBOL_SWITCH(4U, 20U);
  _SFD_EML_CALL(0U, chartInstance->c1_sfEvent, 23);
  c1_i48 = 0;
  for (c1_i49 = 0; c1_i49 < 1280; c1_i49++) {
    for (c1_i50 = 0; c1_i50 < 960; c1_i50++) {
      chartInstance->c1_b_l_rgb2[c1_i50 + c1_i48] = chartInstance->c1_b_l_rgb1
        [(c1_i50 + c1_i48) + 2457600] - chartInstance->c1_b_l_gray[c1_i50 +
        c1_i48];
    }

    c1_i48 += 960;
  }

  _SFD_SYMBOL_SWITCH(7U, 21U);
  _SFD_EML_CALL(0U, chartInstance->c1_sfEvent, 24);
  c1_i51 = 0;
  for (c1_i52 = 0; c1_i52 < 1280; c1_i52++) {
    for (c1_i53 = 0; c1_i53 < 960; c1_i53++) {
      chartInstance->c1_b_r_rgb2[c1_i53 + c1_i51] = chartInstance->c1_b_r_rgb1
        [(c1_i53 + c1_i51) + 2457600] - chartInstance->c1_b_r_gray[c1_i53 +
        c1_i51];
    }

    c1_i51 += 960;
  }

  _SFD_SYMBOL_SWITCH(8U, 22U);
  _SFD_EML_CALL(0U, chartInstance->c1_sfEvent, 26);
  for (c1_i54 = 0; c1_i54 < 1228800; c1_i54++) {
    chartInstance->c1_c_l_rgb2[c1_i54] = chartInstance->c1_b_l_rgb2[c1_i54];
  }

  c1_medfilt2(chartInstance, chartInstance->c1_c_l_rgb2, chartInstance->c1_fv1);
  for (c1_i55 = 0; c1_i55 < 1228800; c1_i55++) {
    chartInstance->c1_b_l_rgb2[c1_i55] = chartInstance->c1_fv1[c1_i55];
  }

  _SFD_SYMBOL_SWITCH(7U, 21U);
  _SFD_EML_CALL(0U, chartInstance->c1_sfEvent, 27);
  for (c1_i56 = 0; c1_i56 < 1228800; c1_i56++) {
    chartInstance->c1_c_r_rgb2[c1_i56] = chartInstance->c1_b_r_rgb2[c1_i56];
  }

  c1_medfilt2(chartInstance, chartInstance->c1_c_r_rgb2, chartInstance->c1_fv2);
  for (c1_i57 = 0; c1_i57 < 1228800; c1_i57++) {
    chartInstance->c1_b_r_rgb2[c1_i57] = chartInstance->c1_fv2[c1_i57];
  }

  _SFD_SYMBOL_SWITCH(8U, 22U);
  _SFD_EML_CALL(0U, chartInstance->c1_sfEvent, 29);
  for (c1_i58 = 0; c1_i58 < 1228800; c1_i58++) {
    chartInstance->c1_d_l_rgb2[c1_i58] = chartInstance->c1_b_l_rgb2[c1_i58];
  }

  c1_myMat2gray(chartInstance, chartInstance->c1_d_l_rgb2, chartInstance->c1_dv2);
  for (c1_i59 = 0; c1_i59 < 1228800; c1_i59++) {
    chartInstance->c1_l_rgb2[c1_i59] = chartInstance->c1_dv2[c1_i59];
  }

  _SFD_SYMBOL_SWITCH(7U, 8U);
  _SFD_EML_CALL(0U, chartInstance->c1_sfEvent, 30);
  for (c1_i60 = 0; c1_i60 < 1228800; c1_i60++) {
    chartInstance->c1_d_r_rgb2[c1_i60] = chartInstance->c1_b_r_rgb2[c1_i60];
  }

  c1_myMat2gray(chartInstance, chartInstance->c1_d_r_rgb2, chartInstance->c1_dv3);
  for (c1_i61 = 0; c1_i61 < 1228800; c1_i61++) {
    chartInstance->c1_r_rgb2[c1_i61] = chartInstance->c1_dv3[c1_i61];
  }

  _SFD_SYMBOL_SWITCH(8U, 9U);
  _SFD_EML_CALL(0U, chartInstance->c1_sfEvent, 32);
  for (c1_i62 = 0; c1_i62 < 1228800; c1_i62++) {
    chartInstance->c1_l_bw[c1_i62] = (chartInstance->c1_l_rgb2[c1_i62] > 0.6);
  }

  _SFD_EML_CALL(0U, chartInstance->c1_sfEvent, 33);
  for (c1_i63 = 0; c1_i63 < 1228800; c1_i63++) {
    chartInstance->c1_r_bw[c1_i63] = (chartInstance->c1_r_rgb2[c1_i63] > 0.6);
  }

  _SFD_EML_CALL(0U, chartInstance->c1_sfEvent, 36);
  c1_BlobAnalysis_BlobAnalysis(chartInstance, &chartInstance->c1_hblob);
  _SFD_EML_CALL(0U, chartInstance->c1_sfEvent, 38);
  for (c1_i64 = 0; c1_i64 < 1228800; c1_i64++) {
    chartInstance->c1_b_l_bw[c1_i64] = chartInstance->c1_l_bw[c1_i64];
  }

  c1_SystemCore_step(chartInstance, &chartInstance->c1_hblob,
                     chartInstance->c1_b_l_bw, c1_unusedU0_data,
                     c1_unusedU0_sizes, c1_b_l_center_data, c1_b_l_center_sizes,
                     c1_unusedU1_data, c1_unusedU1_sizes);
  c1_l_center_sizes[0] = c1_b_l_center_sizes[0];
  c1_l_center_sizes[1] = c1_b_l_center_sizes[1];
  c1_l_center = c1_l_center_sizes[0];
  c1_b_l_center = c1_l_center_sizes[1];
  c1_loop_ub = c1_b_l_center_sizes[0] * c1_b_l_center_sizes[1] - 1;
  for (c1_i65 = 0; c1_i65 <= c1_loop_ub; c1_i65++) {
    c1_l_center_data[c1_i65] = c1_b_l_center_data[c1_i65];
  }

  _SFD_EML_CALL(0U, chartInstance->c1_sfEvent, 38);
  _SFD_EML_CALL(0U, chartInstance->c1_sfEvent, 38);
  _SFD_EML_CALL(0U, chartInstance->c1_sfEvent, 39);
  for (c1_i66 = 0; c1_i66 < 1228800; c1_i66++) {
    chartInstance->c1_b_r_bw[c1_i66] = chartInstance->c1_r_bw[c1_i66];
  }

  c1_b_SystemCore_step(chartInstance, &chartInstance->c1_hblob,
                       chartInstance->c1_b_r_bw, c1_unusedU0_data,
                       c1_unusedU0_sizes, c1_b_l_center_data,
                       c1_b_l_center_sizes, c1_unusedU1_data, c1_unusedU1_sizes);
  c1_r_center_sizes[0] = c1_b_l_center_sizes[0];
  c1_r_center_sizes[1] = c1_b_l_center_sizes[1];
  c1_r_center = c1_r_center_sizes[0];
  c1_b_r_center = c1_r_center_sizes[1];
  c1_b_loop_ub = c1_b_l_center_sizes[0] * c1_b_l_center_sizes[1] - 1;
  for (c1_i67 = 0; c1_i67 <= c1_b_loop_ub; c1_i67++) {
    c1_r_center_data[c1_i67] = c1_b_l_center_data[c1_i67];
  }

  _SFD_EML_CALL(0U, chartInstance->c1_sfEvent, 39);
  _SFD_EML_CALL(0U, chartInstance->c1_sfEvent, 39);
  _SFD_EML_CALL(0U, chartInstance->c1_sfEvent, 42);
  c1_ml = (real_T)c1_l_center_sizes[0];
  _SFD_EML_CALL(0U, chartInstance->c1_sfEvent, 43);
  if (CV_EML_IF(0, 1, 0, CV_RELATIONAL_EVAL(4U, 0U, 0, c1_ml, 4.0, -1, 4U, c1_ml
        > 4.0))) {
    _SFD_EML_CALL(0U, chartInstance->c1_sfEvent, 44);
    c1_ml = 4.0;
  }

  _SFD_EML_CALL(0U, chartInstance->c1_sfEvent, 46);
  c1_b_ml = c1_ml;
  c1_i68 = (int32_T)c1_b_ml - 1;
  c1_i = 1.0;
  c1_b_i = 0;
  while (c1_b_i <= c1_i68) {
    c1_i = 1.0 + (real_T)c1_b_i;
    CV_EML_FOR(0, 1, 0, 1);
    _SFD_EML_CALL(0U, chartInstance->c1_sfEvent, 47);
    c1_i69 = (int32_T)c1_i;
    c1_i70 = c1_l_center_sizes[1];
    c1_c_i = _SFD_EML_ARRAY_BOUNDS_CHECK("l_center", (int32_T)c1_i, 1,
      c1_l_center_sizes[0], 1, 0) - 1;
    c1_tmp_sizes[0] = 1;
    c1_tmp_sizes[1] = c1_i70;
    c1_c_loop_ub = c1_i70 - 1;
    for (c1_i71 = 0; c1_i71 <= c1_c_loop_ub; c1_i71++) {
      c1_tmp_data[c1_tmp_sizes[0] * c1_i71] = c1_l_center_data[c1_c_i +
        c1_l_center_sizes[0] * c1_i71];
    }

    _SFD_SUB_ASSIGN_SIZE_CHECK_ND(c1_iv0, 1, c1_tmp_sizes, 2);
    c1_i72 = c1_i69 - 1;
    for (c1_i73 = 0; c1_i73 < 2; c1_i73++) {
      c1_b_L_p[c1_i73 + (c1_i72 << 1)] = c1_tmp_data[c1_i73];
    }

    c1_b_i++;
    _SF_MEX_LISTEN_FOR_CTRL_C(chartInstance->S);
  }

  CV_EML_FOR(0, 1, 0, 0);
  _SFD_EML_CALL(0U, chartInstance->c1_sfEvent, 51);
  c1_mr = (real_T)c1_r_center_sizes[0];
  _SFD_EML_CALL(0U, chartInstance->c1_sfEvent, 52);
  if (CV_EML_IF(0, 1, 1, CV_RELATIONAL_EVAL(4U, 0U, 1, c1_mr, 4.0, -1, 4U, c1_mr
        > 4.0))) {
    _SFD_EML_CALL(0U, chartInstance->c1_sfEvent, 53);
    c1_mr = 4.0;
  }

  _SFD_EML_CALL(0U, chartInstance->c1_sfEvent, 55);
  c1_b_mr = c1_mr;
  c1_i74 = (int32_T)c1_b_mr - 1;
  c1_i = 1.0;
  c1_d_i = 0;
  while (c1_d_i <= c1_i74) {
    c1_i = 1.0 + (real_T)c1_d_i;
    CV_EML_FOR(0, 1, 1, 1);
    _SFD_EML_CALL(0U, chartInstance->c1_sfEvent, 56);
    c1_i75 = (int32_T)c1_i;
    c1_i76 = c1_r_center_sizes[1];
    c1_e_i = _SFD_EML_ARRAY_BOUNDS_CHECK("r_center", (int32_T)c1_i, 1,
      c1_r_center_sizes[0], 1, 0) - 1;
    c1_tmp_sizes[0] = 1;
    c1_tmp_sizes[1] = c1_i76;
    c1_d_loop_ub = c1_i76 - 1;
    for (c1_i77 = 0; c1_i77 <= c1_d_loop_ub; c1_i77++) {
      c1_tmp_data[c1_tmp_sizes[0] * c1_i77] = c1_r_center_data[c1_e_i +
        c1_r_center_sizes[0] * c1_i77];
    }

    _SFD_SUB_ASSIGN_SIZE_CHECK_ND(c1_iv0, 1, c1_tmp_sizes, 2);
    c1_i78 = c1_i75 - 1;
    for (c1_i79 = 0; c1_i79 < 2; c1_i79++) {
      c1_b_R_p[c1_i79 + (c1_i78 << 1)] = c1_tmp_data[c1_i79];
    }

    c1_d_i++;
    _SF_MEX_LISTEN_FOR_CTRL_C(chartInstance->S);
  }

  CV_EML_FOR(0, 1, 1, 0);
  _SFD_EML_CALL(0U, chartInstance->c1_sfEvent, 59);
  for (c1_i80 = 0; c1_i80 < 8; c1_i80++) {
    c1_c_L_p[c1_i80] = c1_b_L_p[c1_i80];
  }

  c1_orderPoints(chartInstance, c1_c_L_p, c1_dv4);
  for (c1_i81 = 0; c1_i81 < 8; c1_i81++) {
    c1_b_L_p[c1_i81] = c1_dv4[c1_i81];
  }

  _SFD_EML_CALL(0U, chartInstance->c1_sfEvent, 60);
  for (c1_i82 = 0; c1_i82 < 8; c1_i82++) {
    c1_c_R_p[c1_i82] = c1_b_R_p[c1_i82];
  }

  c1_orderPoints(chartInstance, c1_c_R_p, c1_dv5);
  for (c1_i83 = 0; c1_i83 < 8; c1_i83++) {
    c1_b_R_p[c1_i83] = c1_dv5[c1_i83];
  }

  _SFD_EML_CALL(0U, chartInstance->c1_sfEvent, 63);
  _SFD_EML_CALL(0U, chartInstance->c1_sfEvent, 64);
  sf_mex_call_debug(sfGlobalDebugInstanceStruct, "imshow", 0U, 1U, 14,
                    c1_emlrt_marshallOut(chartInstance, chartInstance->c1_l_rgb2));
  _SFD_EML_CALL(0U, chartInstance->c1_sfEvent, 64);
  sf_mex_call_debug(sfGlobalDebugInstanceStruct, "hold", 0U, 1U, 14,
                    c1_b_emlrt_marshallOut(chartInstance, c1_cv0));
  _SFD_EML_CALL(0U, chartInstance->c1_sfEvent, 65);
  c1_i84 = 0;
  for (c1_i85 = 0; c1_i85 < 4; c1_i85++) {
    c1_d_L_p[c1_i85] = c1_b_L_p[c1_i84];
    c1_i84 += 2;
  }

  c1_i86 = 0;
  for (c1_i87 = 0; c1_i87 < 4; c1_i87++) {
    c1_e_L_p[c1_i87] = c1_b_L_p[c1_i86 + 1];
    c1_i86 += 2;
  }

  sf_mex_call_debug(sfGlobalDebugInstanceStruct, "plot", 0U, 3U, 14,
                    c1_c_emlrt_marshallOut(chartInstance, c1_d_L_p), 14,
                    c1_c_emlrt_marshallOut(chartInstance, c1_e_L_p), 14,
                    c1_b_emlrt_marshallOut(chartInstance, c1_cv1));
  _SFD_EML_CALL(0U, chartInstance->c1_sfEvent, 67);
  c1_i88 = 0;
  for (c1_i89 = 0; c1_i89 < 4; c1_i89++) {
    c1_b_L_p[c1_i88 + 1] = 960.0 - c1_b_L_p[c1_i88 + 1];
    c1_i88 += 2;
  }

  _SFD_EML_CALL(0U, chartInstance->c1_sfEvent, 68);
  c1_i90 = 0;
  for (c1_i91 = 0; c1_i91 < 4; c1_i91++) {
    c1_b_R_p[c1_i90 + 1] = 960.0 - c1_b_R_p[c1_i90 + 1];
    c1_i90 += 2;
  }

  _SFD_EML_CALL(0U, chartInstance->c1_sfEvent, -68);
  _SFD_SYMBOL_SCOPE_POP();
  for (c1_i92 = 0; c1_i92 < 8; c1_i92++) {
    (*chartInstance->c1_L_p)[c1_i92] = c1_b_L_p[c1_i92];
  }

  for (c1_i93 = 0; c1_i93 < 8; c1_i93++) {
    (*chartInstance->c1_R_p)[c1_i93] = c1_b_R_p[c1_i93];
  }

  _SFD_CC_CALL(EXIT_OUT_OF_FUNCTION_TAG, 0U, chartInstance->c1_sfEvent);
}

static void initSimStructsc1_mirageTracking(SFc1_mirageTrackingInstanceStruct
  *chartInstance)
{
  (void)chartInstance;
}

static void c1_orderPoints(SFc1_mirageTrackingInstanceStruct *chartInstance,
  real_T c1_p[8], real_T c1_new_p[8])
{
  uint32_T c1_debug_family_var_map[10];
  real_T c1_indx[4];
  real_T c1_indy[4];
  real_T c1_ind4;
  real_T c1_ind3;
  real_T c1_ind1;
  real_T c1_ind2;
  real_T c1_nargin = 1.0;
  real_T c1_nargout = 1.0;
  int32_T c1_i94;
  int32_T c1_i95;
  real_T c1_x[4];
  int32_T c1_iidx[4];
  int32_T c1_i96;
  int32_T c1_i97;
  int32_T c1_i98;
  int32_T c1_i99;
  int32_T c1_i100;
  int32_T c1_i101;
  real_T c1_b_ind1[4];
  int32_T c1_i102;
  int32_T c1_i103;
  _SFD_SYMBOL_SCOPE_PUSH_EML(0U, 10U, 10U, c1_c_debug_family_names,
    c1_debug_family_var_map);
  _SFD_SYMBOL_SCOPE_ADD_EML_IMPORTABLE(c1_indx, 0U, c1_i_sf_marshallOut,
    c1_i_sf_marshallIn);
  _SFD_SYMBOL_SCOPE_ADD_EML_IMPORTABLE(c1_indy, 1U, c1_i_sf_marshallOut,
    c1_i_sf_marshallIn);
  _SFD_SYMBOL_SCOPE_ADD_EML_IMPORTABLE(&c1_ind4, 2U, c1_c_sf_marshallOut,
    c1_b_sf_marshallIn);
  _SFD_SYMBOL_SCOPE_ADD_EML_IMPORTABLE(&c1_ind3, 3U, c1_c_sf_marshallOut,
    c1_b_sf_marshallIn);
  _SFD_SYMBOL_SCOPE_ADD_EML_IMPORTABLE(&c1_ind1, 4U, c1_c_sf_marshallOut,
    c1_b_sf_marshallIn);
  _SFD_SYMBOL_SCOPE_ADD_EML_IMPORTABLE(&c1_ind2, 5U, c1_c_sf_marshallOut,
    c1_b_sf_marshallIn);
  _SFD_SYMBOL_SCOPE_ADD_EML_IMPORTABLE(&c1_nargin, 6U, c1_c_sf_marshallOut,
    c1_b_sf_marshallIn);
  _SFD_SYMBOL_SCOPE_ADD_EML_IMPORTABLE(&c1_nargout, 7U, c1_c_sf_marshallOut,
    c1_b_sf_marshallIn);
  _SFD_SYMBOL_SCOPE_ADD_EML_IMPORTABLE(c1_p, 8U, c1_sf_marshallOut,
    c1_sf_marshallIn);
  _SFD_SYMBOL_SCOPE_ADD_EML_IMPORTABLE(c1_new_p, 9U, c1_sf_marshallOut,
    c1_sf_marshallIn);
  CV_SCRIPT_FCN(0, 0);
  _SFD_SCRIPT_CALL(0U, chartInstance->c1_sfEvent, 3);
  c1_i94 = 0;
  for (c1_i95 = 0; c1_i95 < 4; c1_i95++) {
    c1_x[c1_i95] = c1_p[c1_i94];
    c1_i94 += 2;
  }

  c1_b_eml_sort(chartInstance, c1_x, c1_iidx);
  for (c1_i96 = 0; c1_i96 < 4; c1_i96++) {
    c1_x[c1_i96] = (real_T)c1_iidx[c1_i96];
  }

  for (c1_i97 = 0; c1_i97 < 4; c1_i97++) {
    c1_indx[c1_i97] = c1_x[c1_i97];
  }

  _SFD_SCRIPT_CALL(0U, chartInstance->c1_sfEvent, 3);
  _SFD_SCRIPT_CALL(0U, chartInstance->c1_sfEvent, 4);
  c1_i98 = 0;
  for (c1_i99 = 0; c1_i99 < 4; c1_i99++) {
    c1_x[c1_i99] = 480.0 - c1_p[c1_i98 + 1];
    c1_i98 += 2;
  }

  c1_b_eml_sort(chartInstance, c1_x, c1_iidx);
  for (c1_i100 = 0; c1_i100 < 4; c1_i100++) {
    c1_x[c1_i100] = (real_T)c1_iidx[c1_i100];
  }

  for (c1_i101 = 0; c1_i101 < 4; c1_i101++) {
    c1_indy[c1_i101] = c1_x[c1_i101];
  }

  _SFD_SCRIPT_CALL(0U, chartInstance->c1_sfEvent, 4);
  _SFD_SCRIPT_CALL(0U, chartInstance->c1_sfEvent, 6);
  c1_ind4 = c1_indy[3];
  _SFD_SCRIPT_CALL(0U, chartInstance->c1_sfEvent, 7);
  c1_ind3 = c1_indy[2];
  _SFD_SCRIPT_CALL(0U, chartInstance->c1_sfEvent, 9);
  c1_ind1 = c1_indx[0];
  _SFD_SCRIPT_CALL(0U, chartInstance->c1_sfEvent, 10);
  c1_ind2 = c1_indx[3];
  _SFD_SCRIPT_CALL(0U, chartInstance->c1_sfEvent, 12);
  c1_b_ind1[0] = c1_ind1;
  c1_b_ind1[1] = c1_ind2;
  c1_b_ind1[2] = c1_ind3;
  c1_b_ind1[3] = c1_ind4;
  for (c1_i102 = 0; c1_i102 < 4; c1_i102++) {
    for (c1_i103 = 0; c1_i103 < 2; c1_i103++) {
      c1_new_p[c1_i103 + (c1_i102 << 1)] = c1_p[c1_i103 +
        ((_SFD_EML_ARRAY_BOUNDS_CHECK("p", (int32_T)c1_b_ind1[c1_i102], 1, 4, 2,
           0) - 1) << 1)];
    }
  }

  _SFD_SCRIPT_CALL(0U, chartInstance->c1_sfEvent, -12);
  _SFD_SYMBOL_SCOPE_POP();
}

static void init_script_number_translation(uint32_T c1_machineNumber, uint32_T
  c1_chartNumber, uint32_T c1_instanceNumber)
{
  (void)c1_machineNumber;
  _SFD_SCRIPT_TRANSLATION(c1_chartNumber, c1_instanceNumber, 0U,
    sf_debug_get_script_id(
    "C:\\Users\\sd0016\\Desktop\\Tracking Simulink with Cameras\\orderPoints.m"));
}

static void c1_Outputs(SFc1_mirageTrackingInstanceStruct *chartInstance,
  c1_vision_BlobAnalysis_1 *c1_obj, boolean_T c1_U0[1228800], int32_T
  c1_Y0_data[], int32_T c1_Y0_sizes[2], real_T c1_Y1_data[], int32_T
  c1_Y1_sizes[2], int32_T c1_Y2_data[], int32_T c1_Y2_sizes[2])
{
  boolean_T c1_maxNumBlobsReached;
  int32_T c1_loop;
  uint8_T c1_currentLabel;
  int32_T c1_i;
  int32_T c1_idx;
  int32_T c1_n;
  int32_T c1_m;
  int32_T c1_b_loop;
  int32_T c1_nn;
  uint32_T c1_stackIdx;
  uint32_T c1_pixIdx;
  int32_T c1_b_n;
  int32_T c1_mm;
  int32_T c1_nnPadRows;
  int32_T c1_b_m;
  uint32_T c1_padIdx;
  uint32_T c1_start_pixIdx;
  uint32_T c1_centerIdx;
  int32_T c1_b_i;
  uint32_T c1_walkerIdx;
  uint32_T c1_u0;
  uint32_T c1_u1;
  uint32_T c1_numBlobs;
  int32_T c1_pixListMinc;
  int32_T c1_pixListNinc;
  int32_T c1_c_i;
  int32_T c1_ns;
  int32_T c1_ms;
  int32_T c1_j;
  real_T c1_centroid[2];
  uint32_T c1_colOffset;
  int32_T c1_minc;
  int32_T c1_minr;
  int32_T c1_maxc;
  int32_T c1_maxr;
  int32_T c1_j_pixListNinc;
  int32_T c1_j_pixListMinc;
  int32_T c1_bbox[4];
  uint32_T c1_b_colOffset;
  (void)chartInstance;

  /* System object Outputs function: vision.BlobAnalysis */
  c1_Y0_sizes[0] = 50;
  c1_Y0_sizes[1] = 1;
  c1_Y1_sizes[0] = 50;
  c1_Y1_sizes[1] = 2;
  c1_Y2_sizes[0] = 50;
  c1_Y2_sizes[1] = 4;
  c1_maxNumBlobsReached = false;
  for (c1_loop = 0; c1_loop < 963; c1_loop++) {
    c1_obj->W3_PAD_DW[c1_loop] = 0U;
  }

  c1_currentLabel = 1U;
  c1_i = 0;
  c1_idx = 963;
  for (c1_n = 0; c1_n < 1280; c1_n++) {
    for (c1_m = 0; c1_m < 960; c1_m++) {
      if (c1_U0[c1_i]) {
        c1_obj->W3_PAD_DW[c1_idx] = MAX_uint8_T;
      } else {
        c1_obj->W3_PAD_DW[c1_idx] = 0U;
      }

      c1_i++;
      c1_idx++;
    }

    c1_obj->W3_PAD_DW[c1_idx] = 0U;
    c1_idx;
    c1_obj->W3_PAD_DW[c1_idx + 1] = 0U;
    c1_idx += 2;
  }

  for (c1_b_loop = 0; c1_b_loop < 961; c1_b_loop++) {
    c1_obj->W3_PAD_DW[c1_b_loop + c1_idx] = 0U;
  }

  c1_nn = 0;
  c1_stackIdx = 0U;
  c1_pixIdx = 0U;
  c1_b_n = 0;
  while (c1_b_n < 1280) {
    c1_mm = 0;
    c1_nnPadRows = (c1_nn + 1) * 962;
    c1_b_m = 0;
    while (c1_b_m < 960) {
      c1_padIdx = (uint32_T)((c1_nnPadRows + c1_mm) + 1);
      c1_start_pixIdx = c1_pixIdx;
      if (c1_obj->W3_PAD_DW[c1_padIdx] == 255) {
        c1_obj->W3_PAD_DW[c1_padIdx] = c1_currentLabel;
        c1_obj->W0_N_PIXLIST_DW[c1_pixIdx] = (int16_T)c1_nn;
        c1_obj->W1_M_PIXLIST_DW[c1_pixIdx] = (int16_T)c1_mm;
        c1_pixIdx++;
        c1_obj->W2_NUM_PIX_DW[c1_currentLabel - 1] = 1U;
        c1_obj->W4_STACK_DW[c1_stackIdx] = c1_padIdx;
        c1_stackIdx++;
        while (c1_stackIdx != 0U) {
          c1_stackIdx--;
          c1_centerIdx = c1_obj->W4_STACK_DW[c1_stackIdx];
          for (c1_b_i = 0; c1_b_i < 8; c1_b_i++) {
            c1_walkerIdx = c1_centerIdx + (uint32_T)c1_obj->P0_WALKER_RTP[c1_b_i];
            if (c1_obj->W3_PAD_DW[c1_walkerIdx] == 255) {
              c1_obj->W3_PAD_DW[c1_walkerIdx] = c1_currentLabel;
              c1_u0 = 962U;
              if (c1_u0 == 0U) {
                c1_u1 = MAX_uint32_T;
              } else {
                c1_u1 = c1_walkerIdx / c1_u0;
              }

              c1_obj->W0_N_PIXLIST_DW[c1_pixIdx] = (int16_T)((int16_T)c1_u1 - 1);
              c1_obj->W1_M_PIXLIST_DW[c1_pixIdx] = (int16_T)(c1_walkerIdx % 962U
                - 1U);
              c1_pixIdx++;
              c1_obj->W2_NUM_PIX_DW[c1_currentLabel - 1]++;
              c1_obj->W4_STACK_DW[c1_stackIdx] = c1_walkerIdx;
              c1_stackIdx++;
            }
          }
        }

        if ((c1_obj->W2_NUM_PIX_DW[c1_currentLabel - 1] < c1_obj->P1_MINAREA_RTP)
            || (c1_obj->W2_NUM_PIX_DW[c1_currentLabel - 1] >
                c1_obj->P2_MAXAREA_RTP)) {
          c1_currentLabel--;
          c1_pixIdx = c1_start_pixIdx;
        }

        if (c1_currentLabel == 50) {
          c1_maxNumBlobsReached = true;
          c1_b_n = 1280;
          c1_b_m = 960;
        }

        if (c1_b_m < 960) {
          c1_currentLabel++;
        }
      }

      c1_mm++;
      c1_b_m++;
    }

    c1_nn++;
    c1_b_n++;
  }

  if (c1_maxNumBlobsReached) {
    c1_numBlobs = c1_currentLabel;
  } else {
    c1_numBlobs = (uint8_T)((uint32_T)c1_currentLabel - 1U);
  }

  c1_pixListMinc = 0;
  c1_pixListNinc = 0;
  for (c1_c_i = 0; c1_c_i < (int32_T)c1_numBlobs; c1_c_i++) {
    c1_Y0_data[c1_c_i] = (int32_T)c1_obj->W2_NUM_PIX_DW[c1_c_i];
    c1_ns = 0;
    c1_ms = 0;
    for (c1_j = 0; c1_j < (int32_T)c1_obj->W2_NUM_PIX_DW[c1_c_i]; c1_j++) {
      c1_ns += c1_obj->W0_N_PIXLIST_DW[c1_j + c1_pixListNinc];
      c1_ms += c1_obj->W1_M_PIXLIST_DW[c1_j + c1_pixListMinc];
    }

    c1_centroid[0] = (real_T)c1_ms / (real_T)c1_obj->W2_NUM_PIX_DW[c1_c_i];
    c1_centroid[1] = (real_T)c1_ns / (real_T)c1_obj->W2_NUM_PIX_DW[c1_c_i];
    c1_colOffset = c1_numBlobs;
    c1_Y1_data[c1_c_i] = c1_centroid[1] + 1.0;
    c1_Y1_data[c1_colOffset + (uint32_T)c1_c_i] = c1_centroid[0] + 1.0;
    c1_minc = 1280;
    c1_minr = 960;
    c1_maxc = 0;
    c1_maxr = 0;
    for (c1_j = 0; c1_j < (int32_T)c1_obj->W2_NUM_PIX_DW[c1_c_i]; c1_j++) {
      c1_j_pixListNinc = c1_j + c1_pixListNinc;
      if (c1_obj->W0_N_PIXLIST_DW[c1_j_pixListNinc] < c1_minc) {
        c1_minc = c1_obj->W0_N_PIXLIST_DW[c1_j_pixListNinc];
      }

      if (c1_obj->W0_N_PIXLIST_DW[c1_j_pixListNinc] > c1_maxc) {
        c1_maxc = c1_obj->W0_N_PIXLIST_DW[c1_j_pixListNinc];
      }

      c1_j_pixListMinc = c1_j + c1_pixListMinc;
      if (c1_obj->W1_M_PIXLIST_DW[c1_j_pixListMinc] < c1_minr) {
        c1_minr = c1_obj->W1_M_PIXLIST_DW[c1_j_pixListMinc];
      }

      if (c1_obj->W1_M_PIXLIST_DW[c1_j_pixListMinc] > c1_maxr) {
        c1_maxr = c1_obj->W1_M_PIXLIST_DW[c1_j_pixListMinc];
      }
    }

    c1_bbox[0] = c1_minr;
    c1_bbox[1] = c1_minc;
    c1_bbox[2] = (c1_maxr - c1_minr) + 1;
    c1_bbox[3] = (c1_maxc - c1_minc) + 1;
    c1_b_colOffset = c1_numBlobs;
    c1_Y2_data[c1_c_i] = c1_bbox[1] + 1;
    c1_Y2_data[c1_b_colOffset + (uint32_T)c1_c_i] = c1_bbox[0] + 1;
    c1_Y2_data[((int32_T)c1_b_colOffset << 1) + c1_c_i] = c1_bbox[3];
    c1_Y2_data[3 * (int32_T)c1_b_colOffset + c1_c_i] = c1_bbox[2];
    c1_pixListMinc += (int32_T)c1_obj->W2_NUM_PIX_DW[c1_c_i];
    c1_pixListNinc += (int32_T)c1_obj->W2_NUM_PIX_DW[c1_c_i];
  }

  c1_Y0_sizes[0] = (int32_T)c1_numBlobs;
  c1_Y0_sizes[1] = 1;
  c1_Y1_sizes[0] = (int32_T)c1_numBlobs;
  c1_Y1_sizes[1] = 2;
  c1_Y2_sizes[0] = (int32_T)c1_numBlobs;
  c1_Y2_sizes[1] = 4;
}

static const mxArray *c1_emlrt_marshallOut(SFc1_mirageTrackingInstanceStruct
  *chartInstance, const real_T c1_f_u[1228800])
{
  const mxArray *c1_f_y = NULL;
  (void)chartInstance;
  c1_f_y = NULL;
  sf_mex_assign(&c1_f_y, sf_mex_create("y", c1_f_u, 0, 0U, 1U, 0U, 2, 960, 1280),
                false);
  return c1_f_y;
}

static const mxArray *c1_b_emlrt_marshallOut(SFc1_mirageTrackingInstanceStruct
  *chartInstance, const char_T c1_f_u[2])
{
  const mxArray *c1_f_y = NULL;
  (void)chartInstance;
  c1_f_y = NULL;
  sf_mex_assign(&c1_f_y, sf_mex_create("y", c1_f_u, 10, 0U, 1U, 0U, 2, 1, 2),
                false);
  return c1_f_y;
}

static const mxArray *c1_c_emlrt_marshallOut(SFc1_mirageTrackingInstanceStruct
  *chartInstance, const real_T c1_f_u[4])
{
  const mxArray *c1_f_y = NULL;
  (void)chartInstance;
  c1_f_y = NULL;
  sf_mex_assign(&c1_f_y, sf_mex_create("y", c1_f_u, 0, 0U, 1U, 0U, 2, 1, 4),
                false);
  return c1_f_y;
}

static const mxArray *c1_sf_marshallOut(void *chartInstanceVoid, void *c1_inData)
{
  const mxArray *c1_mxArrayOutData = NULL;
  int32_T c1_i104;
  int32_T c1_i105;
  int32_T c1_i106;
  real_T c1_f_u[8];
  const mxArray *c1_f_y = NULL;
  SFc1_mirageTrackingInstanceStruct *chartInstance;
  chartInstance = (SFc1_mirageTrackingInstanceStruct *)chartInstanceVoid;
  c1_mxArrayOutData = NULL;
  c1_i104 = 0;
  for (c1_i105 = 0; c1_i105 < 4; c1_i105++) {
    for (c1_i106 = 0; c1_i106 < 2; c1_i106++) {
      c1_f_u[c1_i106 + c1_i104] = (*(real_T (*)[8])c1_inData)[c1_i106 + c1_i104];
    }

    c1_i104 += 2;
  }

  c1_f_y = NULL;
  sf_mex_assign(&c1_f_y, sf_mex_create("y", c1_f_u, 0, 0U, 1U, 0U, 2, 2, 4),
                false);
  sf_mex_assign(&c1_mxArrayOutData, c1_f_y, false);
  return c1_mxArrayOutData;
}

static void c1_emlrt_marshallIn(SFc1_mirageTrackingInstanceStruct *chartInstance,
  const mxArray *c1_b_R_p, const char_T *c1_identifier, real_T c1_f_y[8])
{
  emlrtMsgIdentifier c1_thisId;
  c1_thisId.fIdentifier = c1_identifier;
  c1_thisId.fParent = NULL;
  c1_b_emlrt_marshallIn(chartInstance, sf_mex_dup(c1_b_R_p), &c1_thisId, c1_f_y);
  sf_mex_destroy(&c1_b_R_p);
}

static void c1_b_emlrt_marshallIn(SFc1_mirageTrackingInstanceStruct
  *chartInstance, const mxArray *c1_f_u, const emlrtMsgIdentifier *c1_parentId,
  real_T c1_f_y[8])
{
  real_T c1_dv6[8];
  int32_T c1_i107;
  (void)chartInstance;
  sf_mex_import(c1_parentId, sf_mex_dup(c1_f_u), c1_dv6, 1, 0, 0U, 1, 0U, 2, 2,
                4);
  for (c1_i107 = 0; c1_i107 < 8; c1_i107++) {
    c1_f_y[c1_i107] = c1_dv6[c1_i107];
  }

  sf_mex_destroy(&c1_f_u);
}

static void c1_sf_marshallIn(void *chartInstanceVoid, const mxArray
  *c1_mxArrayInData, const char_T *c1_varName, void *c1_outData)
{
  const mxArray *c1_b_R_p;
  const char_T *c1_identifier;
  emlrtMsgIdentifier c1_thisId;
  real_T c1_f_y[8];
  int32_T c1_i108;
  int32_T c1_i109;
  int32_T c1_i110;
  SFc1_mirageTrackingInstanceStruct *chartInstance;
  chartInstance = (SFc1_mirageTrackingInstanceStruct *)chartInstanceVoid;
  c1_b_R_p = sf_mex_dup(c1_mxArrayInData);
  c1_identifier = c1_varName;
  c1_thisId.fIdentifier = c1_identifier;
  c1_thisId.fParent = NULL;
  c1_b_emlrt_marshallIn(chartInstance, sf_mex_dup(c1_b_R_p), &c1_thisId, c1_f_y);
  sf_mex_destroy(&c1_b_R_p);
  c1_i108 = 0;
  for (c1_i109 = 0; c1_i109 < 4; c1_i109++) {
    for (c1_i110 = 0; c1_i110 < 2; c1_i110++) {
      (*(real_T (*)[8])c1_outData)[c1_i110 + c1_i108] = c1_f_y[c1_i110 + c1_i108];
    }

    c1_i108 += 2;
  }

  sf_mex_destroy(&c1_mxArrayInData);
}

static const mxArray *c1_b_sf_marshallOut(void *chartInstanceVoid, void
  *c1_inData)
{
  const mxArray *c1_mxArrayOutData = NULL;
  int32_T c1_i111;
  int32_T c1_i112;
  int32_T c1_i113;
  int32_T c1_i114;
  int32_T c1_i115;
  const mxArray *c1_f_y = NULL;
  SFc1_mirageTrackingInstanceStruct *chartInstance;
  chartInstance = (SFc1_mirageTrackingInstanceStruct *)chartInstanceVoid;
  c1_mxArrayOutData = NULL;
  c1_i111 = 0;
  for (c1_i112 = 0; c1_i112 < 3; c1_i112++) {
    c1_i113 = 0;
    for (c1_i114 = 0; c1_i114 < 1280; c1_i114++) {
      for (c1_i115 = 0; c1_i115 < 960; c1_i115++) {
        chartInstance->c1_b_u[(c1_i115 + c1_i113) + c1_i111] = (*(real32_T (*)
          [3686400])c1_inData)[(c1_i115 + c1_i113) + c1_i111];
      }

      c1_i113 += 960;
    }

    c1_i111 += 1228800;
  }

  c1_f_y = NULL;
  sf_mex_assign(&c1_f_y, sf_mex_create("y", chartInstance->c1_b_u, 1, 0U, 1U, 0U,
    3, 960, 1280, 3), false);
  sf_mex_assign(&c1_mxArrayOutData, c1_f_y, false);
  return c1_mxArrayOutData;
}

static const mxArray *c1_c_sf_marshallOut(void *chartInstanceVoid, void
  *c1_inData)
{
  const mxArray *c1_mxArrayOutData = NULL;
  real_T c1_f_u;
  const mxArray *c1_f_y = NULL;
  SFc1_mirageTrackingInstanceStruct *chartInstance;
  chartInstance = (SFc1_mirageTrackingInstanceStruct *)chartInstanceVoid;
  c1_mxArrayOutData = NULL;
  c1_f_u = *(real_T *)c1_inData;
  c1_f_y = NULL;
  sf_mex_assign(&c1_f_y, sf_mex_create("y", &c1_f_u, 0, 0U, 0U, 0U, 0), false);
  sf_mex_assign(&c1_mxArrayOutData, c1_f_y, false);
  return c1_mxArrayOutData;
}

static real_T c1_c_emlrt_marshallIn(SFc1_mirageTrackingInstanceStruct
  *chartInstance, const mxArray *c1_f_u, const emlrtMsgIdentifier *c1_parentId)
{
  real_T c1_f_y;
  real_T c1_d0;
  (void)chartInstance;
  sf_mex_import(c1_parentId, sf_mex_dup(c1_f_u), &c1_d0, 1, 0, 0U, 0, 0U, 0);
  c1_f_y = c1_d0;
  sf_mex_destroy(&c1_f_u);
  return c1_f_y;
}

static void c1_b_sf_marshallIn(void *chartInstanceVoid, const mxArray
  *c1_mxArrayInData, const char_T *c1_varName, void *c1_outData)
{
  const mxArray *c1_nargout;
  const char_T *c1_identifier;
  emlrtMsgIdentifier c1_thisId;
  real_T c1_f_y;
  SFc1_mirageTrackingInstanceStruct *chartInstance;
  chartInstance = (SFc1_mirageTrackingInstanceStruct *)chartInstanceVoid;
  c1_nargout = sf_mex_dup(c1_mxArrayInData);
  c1_identifier = c1_varName;
  c1_thisId.fIdentifier = c1_identifier;
  c1_thisId.fParent = NULL;
  c1_f_y = c1_c_emlrt_marshallIn(chartInstance, sf_mex_dup(c1_nargout),
    &c1_thisId);
  sf_mex_destroy(&c1_nargout);
  *(real_T *)c1_outData = c1_f_y;
  sf_mex_destroy(&c1_mxArrayInData);
}

static const mxArray *c1_d_sf_marshallOut(void *chartInstanceVoid, void
  *c1_inData)
{
  const mxArray *c1_mxArrayOutData = NULL;
  int32_T c1_i116;
  int32_T c1_i117;
  int32_T c1_i118;
  const mxArray *c1_f_y = NULL;
  SFc1_mirageTrackingInstanceStruct *chartInstance;
  chartInstance = (SFc1_mirageTrackingInstanceStruct *)chartInstanceVoid;
  c1_mxArrayOutData = NULL;
  c1_i116 = 0;
  for (c1_i117 = 0; c1_i117 < 1280; c1_i117++) {
    for (c1_i118 = 0; c1_i118 < 960; c1_i118++) {
      chartInstance->c1_d_u[c1_i118 + c1_i116] = (*(real32_T (*)[1228800])
        c1_inData)[c1_i118 + c1_i116];
    }

    c1_i116 += 960;
  }

  c1_f_y = NULL;
  sf_mex_assign(&c1_f_y, sf_mex_create("y", chartInstance->c1_d_u, 1, 0U, 1U, 0U,
    2, 960, 1280), false);
  sf_mex_assign(&c1_mxArrayOutData, c1_f_y, false);
  return c1_mxArrayOutData;
}

static void c1_d_emlrt_marshallIn(SFc1_mirageTrackingInstanceStruct
  *chartInstance, const mxArray *c1_f_u, const emlrtMsgIdentifier *c1_parentId,
  real32_T c1_f_y[1228800])
{
  int32_T c1_i119;
  sf_mex_import(c1_parentId, sf_mex_dup(c1_f_u), chartInstance->c1_fv3, 1, 1, 0U,
                1, 0U, 2, 960, 1280);
  for (c1_i119 = 0; c1_i119 < 1228800; c1_i119++) {
    c1_f_y[c1_i119] = chartInstance->c1_fv3[c1_i119];
  }

  sf_mex_destroy(&c1_f_u);
}

static void c1_c_sf_marshallIn(void *chartInstanceVoid, const mxArray
  *c1_mxArrayInData, const char_T *c1_varName, void *c1_outData)
{
  const mxArray *c1_e_r_rgb2;
  const char_T *c1_identifier;
  emlrtMsgIdentifier c1_thisId;
  int32_T c1_i120;
  int32_T c1_i121;
  int32_T c1_i122;
  SFc1_mirageTrackingInstanceStruct *chartInstance;
  chartInstance = (SFc1_mirageTrackingInstanceStruct *)chartInstanceVoid;
  c1_e_r_rgb2 = sf_mex_dup(c1_mxArrayInData);
  c1_identifier = c1_varName;
  c1_thisId.fIdentifier = c1_identifier;
  c1_thisId.fParent = NULL;
  c1_d_emlrt_marshallIn(chartInstance, sf_mex_dup(c1_e_r_rgb2), &c1_thisId,
                        chartInstance->c1_d_y);
  sf_mex_destroy(&c1_e_r_rgb2);
  c1_i120 = 0;
  for (c1_i121 = 0; c1_i121 < 1280; c1_i121++) {
    for (c1_i122 = 0; c1_i122 < 960; c1_i122++) {
      (*(real32_T (*)[1228800])c1_outData)[c1_i122 + c1_i120] =
        chartInstance->c1_d_y[c1_i122 + c1_i120];
    }

    c1_i120 += 960;
  }

  sf_mex_destroy(&c1_mxArrayInData);
}

static void c1_e_emlrt_marshallIn(SFc1_mirageTrackingInstanceStruct
  *chartInstance, const mxArray *c1_f_u, const emlrtMsgIdentifier *c1_parentId,
  real32_T c1_f_y[3686400])
{
  int32_T c1_i123;
  sf_mex_import(c1_parentId, sf_mex_dup(c1_f_u), chartInstance->c1_fv4, 1, 1, 0U,
                1, 0U, 3, 960, 1280, 3);
  for (c1_i123 = 0; c1_i123 < 3686400; c1_i123++) {
    c1_f_y[c1_i123] = chartInstance->c1_fv4[c1_i123];
  }

  sf_mex_destroy(&c1_f_u);
}

static void c1_d_sf_marshallIn(void *chartInstanceVoid, const mxArray
  *c1_mxArrayInData, const char_T *c1_varName, void *c1_outData)
{
  const mxArray *c1_c_r_rgb1;
  const char_T *c1_identifier;
  emlrtMsgIdentifier c1_thisId;
  int32_T c1_i124;
  int32_T c1_i125;
  int32_T c1_i126;
  int32_T c1_i127;
  int32_T c1_i128;
  SFc1_mirageTrackingInstanceStruct *chartInstance;
  chartInstance = (SFc1_mirageTrackingInstanceStruct *)chartInstanceVoid;
  c1_c_r_rgb1 = sf_mex_dup(c1_mxArrayInData);
  c1_identifier = c1_varName;
  c1_thisId.fIdentifier = c1_identifier;
  c1_thisId.fParent = NULL;
  c1_e_emlrt_marshallIn(chartInstance, sf_mex_dup(c1_c_r_rgb1), &c1_thisId,
                        chartInstance->c1_b_y);
  sf_mex_destroy(&c1_c_r_rgb1);
  c1_i124 = 0;
  for (c1_i125 = 0; c1_i125 < 3; c1_i125++) {
    c1_i126 = 0;
    for (c1_i127 = 0; c1_i127 < 1280; c1_i127++) {
      for (c1_i128 = 0; c1_i128 < 960; c1_i128++) {
        (*(real32_T (*)[3686400])c1_outData)[(c1_i128 + c1_i126) + c1_i124] =
          chartInstance->c1_b_y[(c1_i128 + c1_i126) + c1_i124];
      }

      c1_i126 += 960;
    }

    c1_i124 += 1228800;
  }

  sf_mex_destroy(&c1_mxArrayInData);
}

static const mxArray *c1_e_sf_marshallOut(void *chartInstanceVoid, real_T
  c1_inData_data[], int32_T c1_inData_sizes[2])
{
  const mxArray *c1_mxArrayOutData = NULL;
  int32_T c1_u_sizes[2];
  int32_T c1_f_u;
  int32_T c1_g_u;
  int32_T c1_loop_ub;
  int32_T c1_i129;
  real_T c1_u_data[100];
  const mxArray *c1_f_y = NULL;
  SFc1_mirageTrackingInstanceStruct *chartInstance;
  chartInstance = (SFc1_mirageTrackingInstanceStruct *)chartInstanceVoid;
  c1_mxArrayOutData = NULL;
  c1_u_sizes[0] = c1_inData_sizes[0];
  c1_u_sizes[1] = c1_inData_sizes[1];
  c1_f_u = c1_u_sizes[0];
  c1_g_u = c1_u_sizes[1];
  c1_loop_ub = c1_inData_sizes[0] * c1_inData_sizes[1] - 1;
  for (c1_i129 = 0; c1_i129 <= c1_loop_ub; c1_i129++) {
    c1_u_data[c1_i129] = c1_inData_data[c1_i129];
  }

  c1_f_y = NULL;
  sf_mex_assign(&c1_f_y, sf_mex_create("y", c1_u_data, 0, 0U, 1U, 0U, 2,
    c1_u_sizes[0], c1_u_sizes[1]), false);
  sf_mex_assign(&c1_mxArrayOutData, c1_f_y, false);
  return c1_mxArrayOutData;
}

static void c1_f_emlrt_marshallIn(SFc1_mirageTrackingInstanceStruct
  *chartInstance, const mxArray *c1_f_u, const emlrtMsgIdentifier *c1_parentId,
  real_T c1_y_data[], int32_T c1_y_sizes[2])
{
  int32_T c1_i130;
  uint32_T c1_uv0[2];
  int32_T c1_i131;
  boolean_T c1_bv0[2];
  int32_T c1_tmp_sizes[2];
  real_T c1_tmp_data[100];
  int32_T c1_f_y;
  int32_T c1_g_y;
  int32_T c1_loop_ub;
  int32_T c1_i132;
  (void)chartInstance;
  for (c1_i130 = 0; c1_i130 < 2; c1_i130++) {
    c1_uv0[c1_i130] = 50U + (uint32_T)(-48 * c1_i130);
  }

  for (c1_i131 = 0; c1_i131 < 2; c1_i131++) {
    c1_bv0[c1_i131] = true;
  }

  sf_mex_import_vs(c1_parentId, sf_mex_dup(c1_f_u), c1_tmp_data, 1, 0, 0U, 1, 0U,
                   2, c1_bv0, c1_uv0, c1_tmp_sizes);
  c1_y_sizes[0] = c1_tmp_sizes[0];
  c1_y_sizes[1] = c1_tmp_sizes[1];
  c1_f_y = c1_y_sizes[0];
  c1_g_y = c1_y_sizes[1];
  c1_loop_ub = c1_tmp_sizes[0] * c1_tmp_sizes[1] - 1;
  for (c1_i132 = 0; c1_i132 <= c1_loop_ub; c1_i132++) {
    c1_y_data[c1_i132] = c1_tmp_data[c1_i132];
  }

  sf_mex_destroy(&c1_f_u);
}

static void c1_e_sf_marshallIn(void *chartInstanceVoid, const mxArray
  *c1_mxArrayInData, const char_T *c1_varName, real_T c1_outData_data[], int32_T
  c1_outData_sizes[2])
{
  const mxArray *c1_r_center;
  const char_T *c1_identifier;
  emlrtMsgIdentifier c1_thisId;
  int32_T c1_y_sizes[2];
  real_T c1_y_data[100];
  int32_T c1_loop_ub;
  int32_T c1_i133;
  int32_T c1_b_loop_ub;
  int32_T c1_i134;
  SFc1_mirageTrackingInstanceStruct *chartInstance;
  chartInstance = (SFc1_mirageTrackingInstanceStruct *)chartInstanceVoid;
  c1_r_center = sf_mex_dup(c1_mxArrayInData);
  c1_identifier = c1_varName;
  c1_thisId.fIdentifier = c1_identifier;
  c1_thisId.fParent = NULL;
  c1_f_emlrt_marshallIn(chartInstance, sf_mex_dup(c1_r_center), &c1_thisId,
                        c1_y_data, c1_y_sizes);
  sf_mex_destroy(&c1_r_center);
  c1_outData_sizes[0] = c1_y_sizes[0];
  c1_outData_sizes[1] = c1_y_sizes[1];
  c1_loop_ub = c1_y_sizes[1] - 1;
  for (c1_i133 = 0; c1_i133 <= c1_loop_ub; c1_i133++) {
    c1_b_loop_ub = c1_y_sizes[0] - 1;
    for (c1_i134 = 0; c1_i134 <= c1_b_loop_ub; c1_i134++) {
      c1_outData_data[c1_i134 + c1_outData_sizes[0] * c1_i133] =
        c1_y_data[c1_i134 + c1_y_sizes[0] * c1_i133];
    }
  }

  sf_mex_destroy(&c1_mxArrayInData);
}

static const mxArray *c1_f_sf_marshallOut(void *chartInstanceVoid, void
  *c1_inData)
{
  const mxArray *c1_mxArrayOutData = NULL;
  int32_T c1_i135;
  int32_T c1_i136;
  int32_T c1_i137;
  const mxArray *c1_f_y = NULL;
  SFc1_mirageTrackingInstanceStruct *chartInstance;
  chartInstance = (SFc1_mirageTrackingInstanceStruct *)chartInstanceVoid;
  c1_mxArrayOutData = NULL;
  c1_i135 = 0;
  for (c1_i136 = 0; c1_i136 < 1280; c1_i136++) {
    for (c1_i137 = 0; c1_i137 < 960; c1_i137++) {
      chartInstance->c1_e_u[c1_i137 + c1_i135] = (*(boolean_T (*)[1228800])
        c1_inData)[c1_i137 + c1_i135];
    }

    c1_i135 += 960;
  }

  c1_f_y = NULL;
  sf_mex_assign(&c1_f_y, sf_mex_create("y", chartInstance->c1_e_u, 11, 0U, 1U,
    0U, 2, 960, 1280), false);
  sf_mex_assign(&c1_mxArrayOutData, c1_f_y, false);
  return c1_mxArrayOutData;
}

static void c1_g_emlrt_marshallIn(SFc1_mirageTrackingInstanceStruct
  *chartInstance, const mxArray *c1_f_u, const emlrtMsgIdentifier *c1_parentId,
  boolean_T c1_f_y[1228800])
{
  int32_T c1_i138;
  sf_mex_import(c1_parentId, sf_mex_dup(c1_f_u), chartInstance->c1_bv1, 1, 11,
                0U, 1, 0U, 2, 960, 1280);
  for (c1_i138 = 0; c1_i138 < 1228800; c1_i138++) {
    c1_f_y[c1_i138] = chartInstance->c1_bv1[c1_i138];
  }

  sf_mex_destroy(&c1_f_u);
}

static void c1_f_sf_marshallIn(void *chartInstanceVoid, const mxArray
  *c1_mxArrayInData, const char_T *c1_varName, void *c1_outData)
{
  const mxArray *c1_c_r_bw;
  const char_T *c1_identifier;
  emlrtMsgIdentifier c1_thisId;
  int32_T c1_i139;
  int32_T c1_i140;
  int32_T c1_i141;
  SFc1_mirageTrackingInstanceStruct *chartInstance;
  chartInstance = (SFc1_mirageTrackingInstanceStruct *)chartInstanceVoid;
  c1_c_r_bw = sf_mex_dup(c1_mxArrayInData);
  c1_identifier = c1_varName;
  c1_thisId.fIdentifier = c1_identifier;
  c1_thisId.fParent = NULL;
  c1_g_emlrt_marshallIn(chartInstance, sf_mex_dup(c1_c_r_bw), &c1_thisId,
                        chartInstance->c1_e_y);
  sf_mex_destroy(&c1_c_r_bw);
  c1_i139 = 0;
  for (c1_i140 = 0; c1_i140 < 1280; c1_i140++) {
    for (c1_i141 = 0; c1_i141 < 960; c1_i141++) {
      (*(boolean_T (*)[1228800])c1_outData)[c1_i141 + c1_i139] =
        chartInstance->c1_e_y[c1_i141 + c1_i139];
    }

    c1_i139 += 960;
  }

  sf_mex_destroy(&c1_mxArrayInData);
}

static const mxArray *c1_g_sf_marshallOut(void *chartInstanceVoid, void
  *c1_inData)
{
  const mxArray *c1_mxArrayOutData = NULL;
  int32_T c1_i142;
  int32_T c1_i143;
  int32_T c1_i144;
  const mxArray *c1_f_y = NULL;
  SFc1_mirageTrackingInstanceStruct *chartInstance;
  chartInstance = (SFc1_mirageTrackingInstanceStruct *)chartInstanceVoid;
  c1_mxArrayOutData = NULL;
  c1_i142 = 0;
  for (c1_i143 = 0; c1_i143 < 1280; c1_i143++) {
    for (c1_i144 = 0; c1_i144 < 960; c1_i144++) {
      chartInstance->c1_c_u[c1_i144 + c1_i142] = (*(real_T (*)[1228800])
        c1_inData)[c1_i144 + c1_i142];
    }

    c1_i142 += 960;
  }

  c1_f_y = NULL;
  sf_mex_assign(&c1_f_y, sf_mex_create("y", chartInstance->c1_c_u, 0, 0U, 1U, 0U,
    2, 960, 1280), false);
  sf_mex_assign(&c1_mxArrayOutData, c1_f_y, false);
  return c1_mxArrayOutData;
}

static void c1_h_emlrt_marshallIn(SFc1_mirageTrackingInstanceStruct
  *chartInstance, const mxArray *c1_f_u, const emlrtMsgIdentifier *c1_parentId,
  real_T c1_f_y[1228800])
{
  int32_T c1_i145;
  sf_mex_import(c1_parentId, sf_mex_dup(c1_f_u), chartInstance->c1_dv7, 1, 0, 0U,
                1, 0U, 2, 960, 1280);
  for (c1_i145 = 0; c1_i145 < 1228800; c1_i145++) {
    c1_f_y[c1_i145] = chartInstance->c1_dv7[c1_i145];
  }

  sf_mex_destroy(&c1_f_u);
}

static void c1_g_sf_marshallIn(void *chartInstanceVoid, const mxArray
  *c1_mxArrayInData, const char_T *c1_varName, void *c1_outData)
{
  const mxArray *c1_e_r_rgb2;
  const char_T *c1_identifier;
  emlrtMsgIdentifier c1_thisId;
  int32_T c1_i146;
  int32_T c1_i147;
  int32_T c1_i148;
  SFc1_mirageTrackingInstanceStruct *chartInstance;
  chartInstance = (SFc1_mirageTrackingInstanceStruct *)chartInstanceVoid;
  c1_e_r_rgb2 = sf_mex_dup(c1_mxArrayInData);
  c1_identifier = c1_varName;
  c1_thisId.fIdentifier = c1_identifier;
  c1_thisId.fParent = NULL;
  c1_h_emlrt_marshallIn(chartInstance, sf_mex_dup(c1_e_r_rgb2), &c1_thisId,
                        chartInstance->c1_c_y);
  sf_mex_destroy(&c1_e_r_rgb2);
  c1_i146 = 0;
  for (c1_i147 = 0; c1_i147 < 1280; c1_i147++) {
    for (c1_i148 = 0; c1_i148 < 960; c1_i148++) {
      (*(real_T (*)[1228800])c1_outData)[c1_i148 + c1_i146] =
        chartInstance->c1_c_y[c1_i148 + c1_i146];
    }

    c1_i146 += 960;
  }

  sf_mex_destroy(&c1_mxArrayInData);
}

static const mxArray *c1_h_sf_marshallOut(void *chartInstanceVoid, void
  *c1_inData)
{
  const mxArray *c1_mxArrayOutData = NULL;
  int32_T c1_i149;
  int32_T c1_i150;
  int32_T c1_i151;
  int32_T c1_i152;
  int32_T c1_i153;
  const mxArray *c1_f_y = NULL;
  SFc1_mirageTrackingInstanceStruct *chartInstance;
  chartInstance = (SFc1_mirageTrackingInstanceStruct *)chartInstanceVoid;
  c1_mxArrayOutData = NULL;
  c1_i149 = 0;
  for (c1_i150 = 0; c1_i150 < 3; c1_i150++) {
    c1_i151 = 0;
    for (c1_i152 = 0; c1_i152 < 1280; c1_i152++) {
      for (c1_i153 = 0; c1_i153 < 960; c1_i153++) {
        chartInstance->c1_u[(c1_i153 + c1_i151) + c1_i149] = (*(real_T (*)
          [3686400])c1_inData)[(c1_i153 + c1_i151) + c1_i149];
      }

      c1_i151 += 960;
    }

    c1_i149 += 1228800;
  }

  c1_f_y = NULL;
  sf_mex_assign(&c1_f_y, sf_mex_create("y", chartInstance->c1_u, 0, 0U, 1U, 0U,
    3, 960, 1280, 3), false);
  sf_mex_assign(&c1_mxArrayOutData, c1_f_y, false);
  return c1_mxArrayOutData;
}

static void c1_i_emlrt_marshallIn(SFc1_mirageTrackingInstanceStruct
  *chartInstance, const mxArray *c1_f_u, const emlrtMsgIdentifier *c1_parentId,
  real_T c1_f_y[3686400])
{
  int32_T c1_i154;
  sf_mex_import(c1_parentId, sf_mex_dup(c1_f_u), chartInstance->c1_dv8, 1, 0, 0U,
                1, 0U, 3, 960, 1280, 3);
  for (c1_i154 = 0; c1_i154 < 3686400; c1_i154++) {
    c1_f_y[c1_i154] = chartInstance->c1_dv8[c1_i154];
  }

  sf_mex_destroy(&c1_f_u);
}

static void c1_h_sf_marshallIn(void *chartInstanceVoid, const mxArray
  *c1_mxArrayInData, const char_T *c1_varName, void *c1_outData)
{
  const mxArray *c1_c_r_rgb1;
  const char_T *c1_identifier;
  emlrtMsgIdentifier c1_thisId;
  int32_T c1_i155;
  int32_T c1_i156;
  int32_T c1_i157;
  int32_T c1_i158;
  int32_T c1_i159;
  SFc1_mirageTrackingInstanceStruct *chartInstance;
  chartInstance = (SFc1_mirageTrackingInstanceStruct *)chartInstanceVoid;
  c1_c_r_rgb1 = sf_mex_dup(c1_mxArrayInData);
  c1_identifier = c1_varName;
  c1_thisId.fIdentifier = c1_identifier;
  c1_thisId.fParent = NULL;
  c1_i_emlrt_marshallIn(chartInstance, sf_mex_dup(c1_c_r_rgb1), &c1_thisId,
                        chartInstance->c1_y);
  sf_mex_destroy(&c1_c_r_rgb1);
  c1_i155 = 0;
  for (c1_i156 = 0; c1_i156 < 3; c1_i156++) {
    c1_i157 = 0;
    for (c1_i158 = 0; c1_i158 < 1280; c1_i158++) {
      for (c1_i159 = 0; c1_i159 < 960; c1_i159++) {
        (*(real_T (*)[3686400])c1_outData)[(c1_i159 + c1_i157) + c1_i155] =
          chartInstance->c1_y[(c1_i159 + c1_i157) + c1_i155];
      }

      c1_i157 += 960;
    }

    c1_i155 += 1228800;
  }

  sf_mex_destroy(&c1_mxArrayInData);
}

static const mxArray *c1_i_sf_marshallOut(void *chartInstanceVoid, void
  *c1_inData)
{
  const mxArray *c1_mxArrayOutData = NULL;
  int32_T c1_i160;
  real_T c1_f_u[4];
  const mxArray *c1_f_y = NULL;
  SFc1_mirageTrackingInstanceStruct *chartInstance;
  chartInstance = (SFc1_mirageTrackingInstanceStruct *)chartInstanceVoid;
  c1_mxArrayOutData = NULL;
  for (c1_i160 = 0; c1_i160 < 4; c1_i160++) {
    c1_f_u[c1_i160] = (*(real_T (*)[4])c1_inData)[c1_i160];
  }

  c1_f_y = NULL;
  sf_mex_assign(&c1_f_y, sf_mex_create("y", c1_f_u, 0, 0U, 1U, 0U, 2, 1, 4),
                false);
  sf_mex_assign(&c1_mxArrayOutData, c1_f_y, false);
  return c1_mxArrayOutData;
}

static void c1_j_emlrt_marshallIn(SFc1_mirageTrackingInstanceStruct
  *chartInstance, const mxArray *c1_f_u, const emlrtMsgIdentifier *c1_parentId,
  real_T c1_f_y[4])
{
  real_T c1_dv9[4];
  int32_T c1_i161;
  (void)chartInstance;
  sf_mex_import(c1_parentId, sf_mex_dup(c1_f_u), c1_dv9, 1, 0, 0U, 1, 0U, 2, 1,
                4);
  for (c1_i161 = 0; c1_i161 < 4; c1_i161++) {
    c1_f_y[c1_i161] = c1_dv9[c1_i161];
  }

  sf_mex_destroy(&c1_f_u);
}

static void c1_i_sf_marshallIn(void *chartInstanceVoid, const mxArray
  *c1_mxArrayInData, const char_T *c1_varName, void *c1_outData)
{
  const mxArray *c1_indy;
  const char_T *c1_identifier;
  emlrtMsgIdentifier c1_thisId;
  real_T c1_f_y[4];
  int32_T c1_i162;
  SFc1_mirageTrackingInstanceStruct *chartInstance;
  chartInstance = (SFc1_mirageTrackingInstanceStruct *)chartInstanceVoid;
  c1_indy = sf_mex_dup(c1_mxArrayInData);
  c1_identifier = c1_varName;
  c1_thisId.fIdentifier = c1_identifier;
  c1_thisId.fParent = NULL;
  c1_j_emlrt_marshallIn(chartInstance, sf_mex_dup(c1_indy), &c1_thisId, c1_f_y);
  sf_mex_destroy(&c1_indy);
  for (c1_i162 = 0; c1_i162 < 4; c1_i162++) {
    (*(real_T (*)[4])c1_outData)[c1_i162] = c1_f_y[c1_i162];
  }

  sf_mex_destroy(&c1_mxArrayInData);
}

const mxArray *sf_c1_mirageTracking_get_eml_resolved_functions_info(void)
{
  const mxArray *c1_nameCaptureInfo = NULL;
  c1_nameCaptureInfo = NULL;
  sf_mex_assign(&c1_nameCaptureInfo, sf_mex_createstruct("structure", 2, 1, 1),
                false);
  c1_info_helper(&c1_nameCaptureInfo);
  sf_mex_emlrtNameCapturePostProcessR2012a(&c1_nameCaptureInfo);
  return c1_nameCaptureInfo;
}

static void c1_info_helper(const mxArray **c1_info)
{
  const mxArray *c1_rhs0 = NULL;
  const mxArray *c1_lhs0 = NULL;
  sf_mex_addfield(*c1_info, c1_d_emlrt_marshallOut(""), "context", "context", 0);
  sf_mex_addfield(*c1_info, c1_d_emlrt_marshallOut("orderPoints"), "name",
                  "name", 0);
  sf_mex_addfield(*c1_info, c1_d_emlrt_marshallOut("double"), "dominantType",
                  "dominantType", 0);
  sf_mex_addfield(*c1_info, c1_d_emlrt_marshallOut(
    "[E]C:/Users/sd0016/Desktop/Tracking Simulink with Cameras/orderPoints.m"),
                  "resolved", "resolved", 0);
  sf_mex_addfield(*c1_info, c1_e_emlrt_marshallOut(1440081773U), "fileTimeLo",
                  "fileTimeLo", 0);
  sf_mex_addfield(*c1_info, c1_e_emlrt_marshallOut(0U), "fileTimeHi",
                  "fileTimeHi", 0);
  sf_mex_addfield(*c1_info, c1_e_emlrt_marshallOut(0U), "mFileTimeLo",
                  "mFileTimeLo", 0);
  sf_mex_addfield(*c1_info, c1_e_emlrt_marshallOut(0U), "mFileTimeHi",
                  "mFileTimeHi", 0);
  sf_mex_assign(&c1_rhs0, sf_mex_createcellmatrix(0, 1), false);
  sf_mex_assign(&c1_lhs0, sf_mex_createcellmatrix(0, 1), false);
  sf_mex_addfield(*c1_info, sf_mex_duplicatearraysafe(&c1_rhs0), "rhs", "rhs", 0);
  sf_mex_addfield(*c1_info, sf_mex_duplicatearraysafe(&c1_lhs0), "lhs", "lhs", 0);
  sf_mex_destroy(&c1_rhs0);
  sf_mex_destroy(&c1_lhs0);
}

static const mxArray *c1_d_emlrt_marshallOut(const char * c1_f_u)
{
  const mxArray *c1_f_y = NULL;
  c1_f_y = NULL;
  sf_mex_assign(&c1_f_y, sf_mex_create("y", c1_f_u, 15, 0U, 0U, 0U, 2, 1, strlen
    (c1_f_u)), false);
  return c1_f_y;
}

static const mxArray *c1_e_emlrt_marshallOut(const uint32_T c1_f_u)
{
  const mxArray *c1_f_y = NULL;
  c1_f_y = NULL;
  sf_mex_assign(&c1_f_y, sf_mex_create("y", &c1_f_u, 7, 0U, 0U, 0U, 0), false);
  return c1_f_y;
}

static void c1_imcomplement(SFc1_mirageTrackingInstanceStruct *chartInstance,
  real32_T c1_im[3686400], real32_T c1_b_im[3686400])
{
  int32_T c1_i163;
  for (c1_i163 = 0; c1_i163 < 3686400; c1_i163++) {
    c1_b_im[c1_i163] = c1_im[c1_i163];
  }

  c1_b_imcomplement(chartInstance, c1_b_im);
}

static void c1_medfilt2(SFc1_mirageTrackingInstanceStruct *chartInstance,
  real32_T c1_g_varargin_1[1228800], real32_T c1_d_b[1228800])
{
  int32_T c1_i164;
  int32_T c1_i165;
  real_T c1_inputSize[2];
  int32_T c1_i166;
  real_T c1_maskSize[2];
  for (c1_i164 = 0; c1_i164 < 1228800; c1_i164++) {
    chartInstance->c1_b_varargin_1[c1_i164] = c1_g_varargin_1[c1_i164];
  }

  c1_padarray(chartInstance, chartInstance->c1_b_varargin_1, chartInstance->c1_a);
  for (c1_i165 = 0; c1_i165 < 2; c1_i165++) {
    c1_inputSize[c1_i165] = 962.0 + 320.0 * (real_T)c1_i165;
  }

  for (c1_i166 = 0; c1_i166 < 2; c1_i166++) {
    c1_maskSize[c1_i166] = 3.0;
  }

  ippMedianFilter_real32(chartInstance->c1_a, c1_inputSize, c1_maskSize, c1_d_b);
}

static void c1_padarray(SFc1_mirageTrackingInstanceStruct *chartInstance,
  real32_T c1_g_varargin_1[1228800], real32_T c1_d_b[1233284])
{
  int32_T c1_i;
  real_T c1_b_i;
  int32_T c1_c_i;
  int32_T c1_j;
  real_T c1_b_j;
  real_T c1_b_a;
  int32_T c1_c;
  int32_T c1_c_j;
  real_T c1_c_a;
  int32_T c1_b_c;
  int32_T c1_d_j;
  int32_T c1_d_i;
  real_T c1_d_a;
  int32_T c1_c_c;
  real_T c1_e_a;
  int32_T c1_d_c;
  c1_cell_cell(chartInstance);
  c1_b_cell_cell(chartInstance);
  c1_c_cell_cell(chartInstance);
  c1_d_cell_cell(chartInstance);
  for (c1_i = 0; c1_i < 962; c1_i++) {
    c1_b_i = 1.0 + (real_T)c1_i;
    c1_d_b[(int32_T)c1_b_i - 1] = 0.0F;
  }

  for (c1_c_i = 0; c1_c_i < 962; c1_c_i++) {
    c1_b_i = 1.0 + (real_T)c1_c_i;
    c1_d_b[(int32_T)c1_b_i + 1232321] = 0.0F;
  }

  for (c1_j = 0; c1_j < 1280; c1_j++) {
    c1_b_j = 1.0 + (real_T)c1_j;
    c1_b_a = c1_b_j;
    c1_c = (int32_T)c1_b_a;
    c1_d_b[962 * c1_c] = 0.0F;
  }

  for (c1_c_j = 0; c1_c_j < 1280; c1_c_j++) {
    c1_b_j = 1.0 + (real_T)c1_c_j;
    c1_c_a = c1_b_j;
    c1_b_c = (int32_T)c1_c_a;
    c1_d_b[961 + 962 * c1_b_c] = 0.0F;
  }

  for (c1_d_j = 0; c1_d_j < 1280; c1_d_j++) {
    c1_b_j = 1.0 + (real_T)c1_d_j;
    for (c1_d_i = 0; c1_d_i < 960; c1_d_i++) {
      c1_b_i = 1.0 + (real_T)c1_d_i;
      c1_d_a = c1_b_i;
      c1_c_c = (int32_T)c1_d_a;
      c1_e_a = c1_b_j;
      c1_d_c = (int32_T)c1_e_a;
      c1_d_b[(_SFD_EML_ARRAY_BOUNDS_CHECK("", c1_c_c + 1, 1, 962, 1, 0) + 962 *
              c1_d_c) - 1] = c1_g_varargin_1[((int32_T)c1_b_i + 960 * ((int32_T)
        c1_b_j - 1)) - 1];
    }
  }
}

static c1_coder_internal_cell_4 c1_cell_cell(SFc1_mirageTrackingInstanceStruct
  *chartInstance)
{
  c1_coder_internal_cell_4 c1_this;
  (void)chartInstance;
  c1_this.dummy = 0;
  return c1_this;
}

static c1_coder_internal_cell_5 c1_b_cell_cell(SFc1_mirageTrackingInstanceStruct
  *chartInstance)
{
  c1_coder_internal_cell_5 c1_this;
  (void)chartInstance;
  c1_this.dummy = 0;
  return c1_this;
}

static c1_coder_internal_cell_6 c1_c_cell_cell(SFc1_mirageTrackingInstanceStruct
  *chartInstance)
{
  c1_coder_internal_cell_6 c1_this;
  (void)chartInstance;
  c1_this.dummy = 0;
  return c1_this;
}

static c1_coder_internal_cell_7 c1_d_cell_cell(SFc1_mirageTrackingInstanceStruct
  *chartInstance)
{
  c1_coder_internal_cell_7 c1_this;
  (void)chartInstance;
  c1_this.dummy = 0;
  return c1_this;
}

static void c1_myMat2gray(SFc1_mirageTrackingInstanceStruct *chartInstance,
  real32_T c1_b_imgIn[1228800], real_T c1_imgOut[1228800])
{
  uint32_T c1_debug_family_var_map[7];
  real_T c1_minL;
  real_T c1_maxL;
  real_T c1_nargin = 1.0;
  real_T c1_nargout = 1.0;
  int32_T c1_i167;
  int32_T c1_i168;
  int32_T c1_ixstart;
  real_T c1_mtmp;
  real_T c1_x;
  boolean_T c1_d_b;
  int32_T c1_ix;
  int32_T c1_b_ix;
  real_T c1_b_x;
  boolean_T c1_e_b;
  int32_T c1_b_a;
  int32_T c1_c_a;
  int32_T c1_i169;
  int32_T c1_c_ix;
  real_T c1_d_a;
  real_T c1_f_b;
  boolean_T c1_p;
  real_T c1_b_mtmp;
  int32_T c1_i170;
  int32_T c1_i171;
  int32_T c1_b_ixstart;
  real_T c1_c_mtmp;
  real_T c1_c_x;
  boolean_T c1_g_b;
  int32_T c1_d_ix;
  int32_T c1_e_ix;
  real_T c1_d_x;
  boolean_T c1_h_b;
  int32_T c1_e_a;
  int32_T c1_f_a;
  int32_T c1_i172;
  int32_T c1_f_ix;
  real_T c1_g_a;
  real_T c1_i_b;
  boolean_T c1_b_p;
  real_T c1_d_mtmp;
  int32_T c1_i173;
  real_T c1_B;
  real_T c1_f_y;
  real_T c1_g_y;
  real_T c1_h_y;
  int32_T c1_i174;
  boolean_T exitg1;
  boolean_T exitg2;
  _SFD_SYMBOL_SCOPE_PUSH_EML(0U, 7U, 8U, c1_b_debug_family_names,
    c1_debug_family_var_map);
  _SFD_SYMBOL_SCOPE_ADD_EML_IMPORTABLE(&c1_minL, 0U, c1_c_sf_marshallOut,
    c1_b_sf_marshallIn);
  _SFD_SYMBOL_SCOPE_ADD_EML_IMPORTABLE(chartInstance->c1_imgTmp, 1U,
    c1_g_sf_marshallOut, c1_g_sf_marshallIn);
  _SFD_SYMBOL_SCOPE_ADD_EML_IMPORTABLE(&c1_maxL, 2U, c1_c_sf_marshallOut,
    c1_b_sf_marshallIn);
  _SFD_SYMBOL_SCOPE_ADD_EML_IMPORTABLE(chartInstance->c1_imgIn, MAX_uint32_T,
    c1_g_sf_marshallOut, c1_g_sf_marshallIn);
  _SFD_SYMBOL_SCOPE_ADD_EML_IMPORTABLE(&c1_nargin, 4U, c1_c_sf_marshallOut,
    c1_b_sf_marshallIn);
  _SFD_SYMBOL_SCOPE_ADD_EML_IMPORTABLE(&c1_nargout, 5U, c1_c_sf_marshallOut,
    c1_b_sf_marshallIn);
  _SFD_SYMBOL_SCOPE_ADD_EML_IMPORTABLE(c1_b_imgIn, 3U, c1_d_sf_marshallOut,
    c1_c_sf_marshallIn);
  _SFD_SYMBOL_SCOPE_ADD_EML_IMPORTABLE(c1_imgOut, 6U, c1_g_sf_marshallOut,
    c1_g_sf_marshallIn);
  CV_EML_FCN(0, 1);
  _SFD_EML_CALL(0U, chartInstance->c1_sfEvent, 73);
  for (c1_i167 = 0; c1_i167 < 1228800; c1_i167++) {
    chartInstance->c1_imgIn[c1_i167] = c1_b_imgIn[c1_i167];
  }

  _SFD_SYMBOL_SWITCH(3U, 3U);
  _SFD_EML_CALL(0U, chartInstance->c1_sfEvent, 75);
  for (c1_i168 = 0; c1_i168 < 1228800; c1_i168++) {
    chartInstance->c1_varargin_1[c1_i168] = chartInstance->c1_imgIn[c1_i168];
  }

  c1_ixstart = 1;
  c1_mtmp = chartInstance->c1_varargin_1[0];
  c1_x = c1_mtmp;
  c1_d_b = muDoubleScalarIsNaN(c1_x);
  if (c1_d_b) {
    c1_ix = 2;
    exitg2 = false;
    while ((exitg2 == false) && (c1_ix < 1228801)) {
      c1_b_ix = c1_ix - 1;
      c1_ixstart = c1_b_ix + 1;
      c1_b_x = chartInstance->c1_varargin_1[c1_b_ix];
      c1_e_b = muDoubleScalarIsNaN(c1_b_x);
      if (!c1_e_b) {
        c1_mtmp = chartInstance->c1_varargin_1[c1_b_ix];
        exitg2 = true;
      } else {
        c1_ix++;
      }
    }
  }

  if (c1_ixstart < 1228800) {
    c1_b_a = c1_ixstart;
    c1_c_a = c1_b_a + 1;
    c1_i169 = c1_c_a;
    for (c1_c_ix = c1_i169; c1_c_ix < 1228801; c1_c_ix++) {
      c1_b_ix = c1_c_ix - 1;
      c1_d_a = chartInstance->c1_varargin_1[c1_b_ix];
      c1_f_b = c1_mtmp;
      c1_p = (c1_d_a < c1_f_b);
      if (c1_p) {
        c1_mtmp = chartInstance->c1_varargin_1[c1_b_ix];
      }
    }
  }

  c1_b_mtmp = c1_mtmp;
  c1_minL = c1_b_mtmp;
  _SFD_EML_CALL(0U, chartInstance->c1_sfEvent, 77);
  for (c1_i170 = 0; c1_i170 < 1228800; c1_i170++) {
    chartInstance->c1_imgTmp[c1_i170] = chartInstance->c1_imgIn[c1_i170] -
      c1_minL;
  }

  _SFD_EML_CALL(0U, chartInstance->c1_sfEvent, 79);
  for (c1_i171 = 0; c1_i171 < 1228800; c1_i171++) {
    chartInstance->c1_varargin_1[c1_i171] = chartInstance->c1_imgTmp[c1_i171];
  }

  c1_b_ixstart = 1;
  c1_c_mtmp = chartInstance->c1_varargin_1[0];
  c1_c_x = c1_c_mtmp;
  c1_g_b = muDoubleScalarIsNaN(c1_c_x);
  if (c1_g_b) {
    c1_d_ix = 2;
    exitg1 = false;
    while ((exitg1 == false) && (c1_d_ix < 1228801)) {
      c1_e_ix = c1_d_ix - 1;
      c1_b_ixstart = c1_e_ix + 1;
      c1_d_x = chartInstance->c1_varargin_1[c1_e_ix];
      c1_h_b = muDoubleScalarIsNaN(c1_d_x);
      if (!c1_h_b) {
        c1_c_mtmp = chartInstance->c1_varargin_1[c1_e_ix];
        exitg1 = true;
      } else {
        c1_d_ix++;
      }
    }
  }

  if (c1_b_ixstart < 1228800) {
    c1_e_a = c1_b_ixstart;
    c1_f_a = c1_e_a + 1;
    c1_i172 = c1_f_a;
    for (c1_f_ix = c1_i172; c1_f_ix < 1228801; c1_f_ix++) {
      c1_e_ix = c1_f_ix - 1;
      c1_g_a = chartInstance->c1_varargin_1[c1_e_ix];
      c1_i_b = c1_c_mtmp;
      c1_b_p = (c1_g_a > c1_i_b);
      if (c1_b_p) {
        c1_c_mtmp = chartInstance->c1_varargin_1[c1_e_ix];
      }
    }
  }

  c1_d_mtmp = c1_c_mtmp;
  c1_maxL = c1_d_mtmp;
  _SFD_EML_CALL(0U, chartInstance->c1_sfEvent, 81);
  for (c1_i173 = 0; c1_i173 < 1228800; c1_i173++) {
    chartInstance->c1_A[c1_i173] = chartInstance->c1_imgTmp[c1_i173];
  }

  c1_B = c1_maxL;
  c1_f_y = c1_B;
  c1_g_y = c1_f_y;
  c1_h_y = c1_g_y;
  for (c1_i174 = 0; c1_i174 < 1228800; c1_i174++) {
    c1_imgOut[c1_i174] = chartInstance->c1_A[c1_i174] / c1_h_y;
  }

  _SFD_EML_CALL(0U, chartInstance->c1_sfEvent, -81);
  _SFD_SYMBOL_SCOPE_POP();
}

static void c1_check_forloop_overflow_error(SFc1_mirageTrackingInstanceStruct
  *chartInstance, boolean_T c1_overflow)
{
  const mxArray *c1_f_y = NULL;
  static char_T c1_f_u[34] = { 'C', 'o', 'd', 'e', 'r', ':', 't', 'o', 'o', 'l',
    'b', 'o', 'x', ':', 'i', 'n', 't', '_', 'f', 'o', 'r', 'l', 'o', 'o', 'p',
    '_', 'o', 'v', 'e', 'r', 'f', 'l', 'o', 'w' };

  const mxArray *c1_g_y = NULL;
  static char_T c1_g_u[5] = { 'i', 'n', 't', '3', '2' };

  (void)chartInstance;
  (void)c1_overflow;
  c1_f_y = NULL;
  sf_mex_assign(&c1_f_y, sf_mex_create("y", c1_f_u, 10, 0U, 1U, 0U, 2, 1, 34),
                false);
  c1_g_y = NULL;
  sf_mex_assign(&c1_g_y, sf_mex_create("y", c1_g_u, 10, 0U, 1U, 0U, 2, 1, 5),
                false);
  sf_mex_call_debug(sfGlobalDebugInstanceStruct, "error", 0U, 1U, 14,
                    sf_mex_call_debug(sfGlobalDebugInstanceStruct, "message", 1U,
    2U, 14, c1_f_y, 14, c1_g_y));
}

static void c1_System_System(SFc1_mirageTrackingInstanceStruct *chartInstance,
  c1_visioncodegen_BlobAnalysis *c1_obj)
{
  c1_visioncodegen_BlobAnalysis *c1_b_obj;
  c1_visioncodegen_BlobAnalysis *c1_c_obj;
  c1_visioncodegen_BlobAnalysis *c1_d_obj;
  const mxArray *c1_unusedU1 = NULL;
  const mxArray *c1_unusedU2 = NULL;
  const mxArray *c1_defaultValidateInputsImpl = NULL;
  const mxArray *c1_unusedU3 = NULL;
  const mxArray *c1_unusedU4 = NULL;
  const mxArray *c1_defaultProcessInputSizeChangeImpl = NULL;
  (void)chartInstance;
  c1_b_obj = c1_obj;
  c1_c_obj = c1_b_obj;
  c1_d_obj = c1_c_obj;
  c1_d_obj->isInitialized = 0;
  sf_mex_destroy(&c1_unusedU1);
  sf_mex_destroy(&c1_unusedU2);
  sf_mex_destroy(&c1_defaultValidateInputsImpl);
  sf_mex_destroy(&c1_unusedU3);
  sf_mex_destroy(&c1_unusedU4);
  sf_mex_destroy(&c1_defaultProcessInputSizeChangeImpl);
}

static c1_visioncodegen_BlobAnalysis *c1_BlobAnalysis_BlobAnalysis
  (SFc1_mirageTrackingInstanceStruct *chartInstance,
   c1_visioncodegen_BlobAnalysis *c1_obj)
{
  c1_visioncodegen_BlobAnalysis *c1_b_obj;
  c1_vision_BlobAnalysis_1 *c1_c_obj;
  c1_vision_BlobAnalysis_1 *c1_d_obj;
  int32_T c1_i175;
  static int32_T c1_iv1[8] = { -1, 961, 962, 963, 1, -961, -962, -963 };

  c1_visioncodegen_BlobAnalysis *c1_e_obj;
  const mxArray *c1_f_y = NULL;
  static char_T c1_f_u[42] = { 'M', 'A', 'T', 'L', 'A', 'B', ':', 's', 'y', 's',
    't', 'e', 'm', ':', 'n', 'o', 'T', 'u', 'n', 'i', 'n', 'g', 'B', 'e', 'f',
    'o', 'r', 'e', 'L', 'o', 'c', 'k', 'i', 'n', 'g', 'C', 'o', 'd', 'e', 'G',
    'e', 'n' };

  c1_vision_BlobAnalysis_1 *c1_f_obj;
  c1_visioncodegen_BlobAnalysis *c1_g_obj;
  const mxArray *c1_g_y = NULL;
  c1_vision_BlobAnalysis_1 *c1_h_obj;
  c1_b_obj = c1_obj;
  c1_System_System(chartInstance, c1_b_obj);
  c1_b_obj->NoTuningBeforeLockingCodeGenError = true;
  c1_c_obj = &c1_b_obj->cSFunObject;

  /* System object Constructor function: vision.BlobAnalysis */
  c1_d_obj = c1_c_obj;
  c1_d_obj->S0_isInitialized = false;
  for (c1_i175 = 0; c1_i175 < 8; c1_i175++) {
    c1_d_obj->P0_WALKER_RTP[c1_i175] = c1_iv1[c1_i175];
  }

  c1_d_obj->P1_MINAREA_RTP = 0U;
  c1_d_obj->P2_MAXAREA_RTP = MAX_uint32_T;
  c1_e_obj = c1_b_obj;
  if (c1_e_obj->NoTuningBeforeLockingCodeGenError) {
  } else {
    c1_f_y = NULL;
    sf_mex_assign(&c1_f_y, sf_mex_create("y", c1_f_u, 10, 0U, 1U, 0U, 2, 1, 42),
                  false);
    sf_mex_call_debug(sfGlobalDebugInstanceStruct, "error", 0U, 1U, 14,
                      sf_mex_call_debug(sfGlobalDebugInstanceStruct, "message",
      1U, 1U, 14, c1_f_y));
  }

  c1_f_obj = &c1_e_obj->cSFunObject;
  c1_f_obj->P1_MINAREA_RTP = 0U;
  c1_g_obj = c1_b_obj;
  if (c1_g_obj->NoTuningBeforeLockingCodeGenError) {
  } else {
    c1_g_y = NULL;
    sf_mex_assign(&c1_g_y, sf_mex_create("y", c1_f_u, 10, 0U, 1U, 0U, 2, 1, 42),
                  false);
    sf_mex_call_debug(sfGlobalDebugInstanceStruct, "error", 0U, 1U, 14,
                      sf_mex_call_debug(sfGlobalDebugInstanceStruct, "message",
      1U, 1U, 14, c1_g_y));
  }

  c1_h_obj = &c1_g_obj->cSFunObject;
  c1_h_obj->P2_MAXAREA_RTP = MAX_uint32_T;
  c1_b_obj->NoTuningBeforeLockingCodeGenError = false;
  return c1_b_obj;
}

static void c1_SystemCore_step(SFc1_mirageTrackingInstanceStruct *chartInstance,
  c1_visioncodegen_BlobAnalysis *c1_obj, boolean_T c1_g_varargin_1[1228800],
  int32_T c1_varargout_1_data[], int32_T c1_varargout_1_sizes[2], real_T
  c1_varargout_2_data[], int32_T c1_varargout_2_sizes[2], int32_T
  c1_varargout_3_data[], int32_T c1_varargout_3_sizes[2])
{
  const mxArray *c1_f_y = NULL;
  static char_T c1_f_u[45] = { 'M', 'A', 'T', 'L', 'A', 'B', ':', 's', 'y', 's',
    't', 'e', 'm', ':', 'm', 'e', 't', 'h', 'o', 'd', 'C', 'a', 'l', 'l', 'e',
    'd', 'W', 'h', 'e', 'n', 'R', 'e', 'l', 'e', 'a', 's', 'e', 'd', 'C', 'o',
    'd', 'e', 'g', 'e', 'n' };

  const mxArray *c1_g_y = NULL;
  static char_T c1_g_u[4] = { 's', 't', 'e', 'p' };

  c1_visioncodegen_BlobAnalysis *c1_b_obj;
  c1_visioncodegen_BlobAnalysis *c1_c_obj;
  const mxArray *c1_h_y = NULL;
  static char_T c1_h_u[51] = { 'M', 'A', 'T', 'L', 'A', 'B', ':', 's', 'y', 's',
    't', 'e', 'm', ':', 'm', 'e', 't', 'h', 'o', 'd', 'C', 'a', 'l', 'l', 'e',
    'd', 'W', 'h', 'e', 'n', 'L', 'o', 'c', 'k', 'e', 'd', 'R', 'e', 'l', 'e',
    'a', 's', 'e', 'd', 'C', 'o', 'd', 'e', 'g', 'e', 'n' };

  const mxArray *c1_i_y = NULL;
  static char_T c1_i_u[5] = { 's', 'e', 't', 'u', 'p' };

  const mxArray *c1_unusedU1f = NULL;
  const mxArray *c1_unusedU20 = NULL;
  const mxArray *c1_isDefNum = NULL;
  c1_visioncodegen_BlobAnalysis *c1_d_obj;
  const mxArray *c1_in = NULL;
  const mxArray *c1_unusedU9 = NULL;
  const mxArray *c1_isDefImpl = NULL;
  int32_T c1_i176;
  const mxArray *c1_unusedU5 = NULL;
  const mxArray *c1_unusedU6 = NULL;
  const mxArray *c1_isDefNumOut = NULL;
  if (c1_obj->isInitialized != 2) {
  } else {
    c1_f_y = NULL;
    sf_mex_assign(&c1_f_y, sf_mex_create("y", c1_f_u, 10, 0U, 1U, 0U, 2, 1, 45),
                  false);
    c1_g_y = NULL;
    sf_mex_assign(&c1_g_y, sf_mex_create("y", c1_g_u, 10, 0U, 1U, 0U, 2, 1, 4),
                  false);
    sf_mex_call_debug(sfGlobalDebugInstanceStruct, "error", 0U, 1U, 14,
                      sf_mex_call_debug(sfGlobalDebugInstanceStruct, "message",
      1U, 2U, 14, c1_f_y, 14, c1_g_y));
  }

  if (c1_obj->isInitialized != 1) {
    c1_b_obj = c1_obj;
    c1_c_obj = c1_b_obj;
    if (c1_c_obj->isInitialized == 0) {
    } else {
      c1_h_y = NULL;
      sf_mex_assign(&c1_h_y, sf_mex_create("y", c1_h_u, 10, 0U, 1U, 0U, 2, 1, 51),
                    false);
      c1_i_y = NULL;
      sf_mex_assign(&c1_i_y, sf_mex_create("y", c1_i_u, 10, 0U, 1U, 0U, 2, 1, 5),
                    false);
      sf_mex_call_debug(sfGlobalDebugInstanceStruct, "error", 0U, 1U, 14,
                        sf_mex_call_debug(sfGlobalDebugInstanceStruct, "message",
        1U, 2U, 14, c1_h_y, 14, c1_i_y));
    }

    c1_c_obj->isInitialized = 1;
    sf_mex_destroy(&c1_unusedU1f);
    sf_mex_destroy(&c1_unusedU20);
    sf_mex_destroy(&c1_isDefNum);
    c1_d_obj = c1_c_obj;
    c1_d_obj->NoTuningBeforeLockingCodeGenError = true;
    sf_mex_destroy(&c1_in);
    sf_mex_destroy(&c1_unusedU9);
    sf_mex_destroy(&c1_isDefImpl);
  }

  for (c1_i176 = 0; c1_i176 < 1228800; c1_i176++) {
    chartInstance->c1_e_varargin_1[c1_i176] = c1_g_varargin_1[c1_i176];
  }

  c1_Nondirect_stepImpl(chartInstance, c1_obj, chartInstance->c1_e_varargin_1,
                        c1_varargout_1_data, c1_varargout_1_sizes,
                        c1_varargout_2_data, c1_varargout_2_sizes,
                        c1_varargout_3_data, c1_varargout_3_sizes);
  sf_mex_destroy(&c1_unusedU5);
  sf_mex_destroy(&c1_unusedU6);
  sf_mex_destroy(&c1_isDefNumOut);
}

static void c1_Nondirect_stepImpl(SFc1_mirageTrackingInstanceStruct
  *chartInstance, c1_visioncodegen_BlobAnalysis *c1_obj, boolean_T
  c1_g_varargin_1[1228800], int32_T c1_varargout_1_data[], int32_T
  c1_varargout_1_sizes[2], real_T c1_varargout_2_data[], int32_T
  c1_varargout_2_sizes[2], int32_T c1_varargout_3_data[], int32_T
  c1_varargout_3_sizes[2])
{
  c1_visioncodegen_BlobAnalysis *c1_b_obj;
  c1_vision_BlobAnalysis_1 *c1_c_obj;
  int32_T c1_i177;
  const mxArray *c1_errId = NULL;
  const mxArray *c1_errMsg = NULL;
  const mxArray *c1_numinputs = NULL;
  const mxArray *c1_b_errId = NULL;
  const mxArray *c1_b_errMsg = NULL;
  const mxArray *c1_numoutputs = NULL;
  c1_b_obj = c1_obj;
  c1_c_obj = &c1_b_obj->cSFunObject;
  for (c1_i177 = 0; c1_i177 < 1228800; c1_i177++) {
    chartInstance->c1_f_varargin_1[c1_i177] = c1_g_varargin_1[c1_i177];
  }

  c1_Outputs(chartInstance, c1_c_obj, chartInstance->c1_f_varargin_1,
             c1_varargout_1_data, c1_varargout_1_sizes, c1_varargout_2_data,
             c1_varargout_2_sizes, c1_varargout_3_data, c1_varargout_3_sizes);
  sf_mex_destroy(&c1_errId);
  sf_mex_destroy(&c1_errMsg);
  sf_mex_destroy(&c1_numinputs);
  sf_mex_destroy(&c1_b_errId);
  sf_mex_destroy(&c1_b_errMsg);
  sf_mex_destroy(&c1_numoutputs);
}

static void c1_b_SystemCore_step(SFc1_mirageTrackingInstanceStruct
  *chartInstance, c1_visioncodegen_BlobAnalysis *c1_obj, boolean_T
  c1_g_varargin_1[1228800], int32_T c1_varargout_1_data[], int32_T
  c1_varargout_1_sizes[2], real_T c1_varargout_2_data[], int32_T
  c1_varargout_2_sizes[2], int32_T c1_varargout_3_data[], int32_T
  c1_varargout_3_sizes[2])
{
  const mxArray *c1_f_y = NULL;
  static char_T c1_f_u[45] = { 'M', 'A', 'T', 'L', 'A', 'B', ':', 's', 'y', 's',
    't', 'e', 'm', ':', 'm', 'e', 't', 'h', 'o', 'd', 'C', 'a', 'l', 'l', 'e',
    'd', 'W', 'h', 'e', 'n', 'R', 'e', 'l', 'e', 'a', 's', 'e', 'd', 'C', 'o',
    'd', 'e', 'g', 'e', 'n' };

  const mxArray *c1_g_y = NULL;
  static char_T c1_g_u[4] = { 's', 't', 'e', 'p' };

  c1_visioncodegen_BlobAnalysis *c1_b_obj;
  c1_visioncodegen_BlobAnalysis *c1_c_obj;
  const mxArray *c1_h_y = NULL;
  static char_T c1_h_u[51] = { 'M', 'A', 'T', 'L', 'A', 'B', ':', 's', 'y', 's',
    't', 'e', 'm', ':', 'm', 'e', 't', 'h', 'o', 'd', 'C', 'a', 'l', 'l', 'e',
    'd', 'W', 'h', 'e', 'n', 'L', 'o', 'c', 'k', 'e', 'd', 'R', 'e', 'l', 'e',
    'a', 's', 'e', 'd', 'C', 'o', 'd', 'e', 'g', 'e', 'n' };

  const mxArray *c1_i_y = NULL;
  static char_T c1_i_u[5] = { 's', 'e', 't', 'u', 'p' };

  const mxArray *c1_unusedU1f = NULL;
  const mxArray *c1_unusedU20 = NULL;
  const mxArray *c1_isDefNum = NULL;
  c1_visioncodegen_BlobAnalysis *c1_d_obj;
  const mxArray *c1_in = NULL;
  const mxArray *c1_unusedU9 = NULL;
  const mxArray *c1_isDefImpl = NULL;
  int32_T c1_i178;
  const mxArray *c1_unusedU5 = NULL;
  const mxArray *c1_unusedU6 = NULL;
  const mxArray *c1_isDefNumOut = NULL;
  if (c1_obj->isInitialized != 2) {
  } else {
    c1_f_y = NULL;
    sf_mex_assign(&c1_f_y, sf_mex_create("y", c1_f_u, 10, 0U, 1U, 0U, 2, 1, 45),
                  false);
    c1_g_y = NULL;
    sf_mex_assign(&c1_g_y, sf_mex_create("y", c1_g_u, 10, 0U, 1U, 0U, 2, 1, 4),
                  false);
    sf_mex_call_debug(sfGlobalDebugInstanceStruct, "error", 0U, 1U, 14,
                      sf_mex_call_debug(sfGlobalDebugInstanceStruct, "message",
      1U, 2U, 14, c1_f_y, 14, c1_g_y));
  }

  if (c1_obj->isInitialized != 1) {
    c1_b_obj = c1_obj;
    c1_c_obj = c1_b_obj;
    if (c1_c_obj->isInitialized == 0) {
    } else {
      c1_h_y = NULL;
      sf_mex_assign(&c1_h_y, sf_mex_create("y", c1_h_u, 10, 0U, 1U, 0U, 2, 1, 51),
                    false);
      c1_i_y = NULL;
      sf_mex_assign(&c1_i_y, sf_mex_create("y", c1_i_u, 10, 0U, 1U, 0U, 2, 1, 5),
                    false);
      sf_mex_call_debug(sfGlobalDebugInstanceStruct, "error", 0U, 1U, 14,
                        sf_mex_call_debug(sfGlobalDebugInstanceStruct, "message",
        1U, 2U, 14, c1_h_y, 14, c1_i_y));
    }

    c1_c_obj->isInitialized = 1;
    sf_mex_destroy(&c1_unusedU1f);
    sf_mex_destroy(&c1_unusedU20);
    sf_mex_destroy(&c1_isDefNum);
    c1_d_obj = c1_c_obj;
    c1_d_obj->NoTuningBeforeLockingCodeGenError = true;
    sf_mex_destroy(&c1_in);
    sf_mex_destroy(&c1_unusedU9);
    sf_mex_destroy(&c1_isDefImpl);
  }

  for (c1_i178 = 0; c1_i178 < 1228800; c1_i178++) {
    chartInstance->c1_c_varargin_1[c1_i178] = c1_g_varargin_1[c1_i178];
  }

  c1_b_Nondirect_stepImpl(chartInstance, c1_obj, chartInstance->c1_c_varargin_1,
    c1_varargout_1_data, c1_varargout_1_sizes, c1_varargout_2_data,
    c1_varargout_2_sizes, c1_varargout_3_data, c1_varargout_3_sizes);
  sf_mex_destroy(&c1_unusedU5);
  sf_mex_destroy(&c1_unusedU6);
  sf_mex_destroy(&c1_isDefNumOut);
}

static void c1_b_Nondirect_stepImpl(SFc1_mirageTrackingInstanceStruct
  *chartInstance, c1_visioncodegen_BlobAnalysis *c1_obj, boolean_T
  c1_g_varargin_1[1228800], int32_T c1_varargout_1_data[], int32_T
  c1_varargout_1_sizes[2], real_T c1_varargout_2_data[], int32_T
  c1_varargout_2_sizes[2], int32_T c1_varargout_3_data[], int32_T
  c1_varargout_3_sizes[2])
{
  c1_visioncodegen_BlobAnalysis *c1_b_obj;
  c1_vision_BlobAnalysis_1 *c1_c_obj;
  int32_T c1_i179;
  const mxArray *c1_errId = NULL;
  const mxArray *c1_errMsg = NULL;
  const mxArray *c1_numinputs = NULL;
  const mxArray *c1_b_errId = NULL;
  const mxArray *c1_b_errMsg = NULL;
  const mxArray *c1_numoutputs = NULL;
  c1_b_obj = c1_obj;
  c1_c_obj = &c1_b_obj->cSFunObject;
  for (c1_i179 = 0; c1_i179 < 1228800; c1_i179++) {
    chartInstance->c1_d_varargin_1[c1_i179] = c1_g_varargin_1[c1_i179];
  }

  c1_Outputs(chartInstance, c1_c_obj, chartInstance->c1_d_varargin_1,
             c1_varargout_1_data, c1_varargout_1_sizes, c1_varargout_2_data,
             c1_varargout_2_sizes, c1_varargout_3_data, c1_varargout_3_sizes);
  sf_mex_destroy(&c1_errId);
  sf_mex_destroy(&c1_errMsg);
  sf_mex_destroy(&c1_numinputs);
  sf_mex_destroy(&c1_b_errId);
  sf_mex_destroy(&c1_b_errMsg);
  sf_mex_destroy(&c1_numoutputs);
}

static void c1_eml_sort(SFc1_mirageTrackingInstanceStruct *chartInstance, real_T
  c1_x[4], real_T c1_b_x[4], int32_T c1_idx[4])
{
  int32_T c1_i180;
  for (c1_i180 = 0; c1_i180 < 4; c1_i180++) {
    c1_b_x[c1_i180] = c1_x[c1_i180];
  }

  c1_b_eml_sort(chartInstance, c1_b_x, c1_idx);
}

static void c1_eml_sort_idx(SFc1_mirageTrackingInstanceStruct *chartInstance,
  real_T c1_x[4], int32_T c1_idx[4], real_T c1_b_x[4])
{
  int32_T c1_i181;
  for (c1_i181 = 0; c1_i181 < 4; c1_i181++) {
    c1_b_x[c1_i181] = c1_x[c1_i181];
  }

  c1_b_eml_sort_idx(chartInstance, c1_b_x, c1_idx);
}

static void c1_merge(SFc1_mirageTrackingInstanceStruct *chartInstance, int32_T
                     c1_idx[4], real_T c1_x[4], int32_T c1_offset, int32_T c1_np,
                     int32_T c1_nq, int32_T c1_b_idx[4], real_T c1_b_x[4])
{
  int32_T c1_i182;
  int32_T c1_i183;
  for (c1_i182 = 0; c1_i182 < 4; c1_i182++) {
    c1_b_idx[c1_i182] = c1_idx[c1_i182];
  }

  for (c1_i183 = 0; c1_i183 < 4; c1_i183++) {
    c1_b_x[c1_i183] = c1_x[c1_i183];
  }

  c1_b_merge(chartInstance, c1_b_idx, c1_b_x, c1_offset, c1_np, c1_nq);
}

static const mxArray *c1_j_sf_marshallOut(void *chartInstanceVoid, void
  *c1_inData)
{
  const mxArray *c1_mxArrayOutData = NULL;
  int32_T c1_f_u;
  const mxArray *c1_f_y = NULL;
  SFc1_mirageTrackingInstanceStruct *chartInstance;
  chartInstance = (SFc1_mirageTrackingInstanceStruct *)chartInstanceVoid;
  c1_mxArrayOutData = NULL;
  c1_f_u = *(int32_T *)c1_inData;
  c1_f_y = NULL;
  sf_mex_assign(&c1_f_y, sf_mex_create("y", &c1_f_u, 6, 0U, 0U, 0U, 0), false);
  sf_mex_assign(&c1_mxArrayOutData, c1_f_y, false);
  return c1_mxArrayOutData;
}

static int32_T c1_k_emlrt_marshallIn(SFc1_mirageTrackingInstanceStruct
  *chartInstance, const mxArray *c1_f_u, const emlrtMsgIdentifier *c1_parentId)
{
  int32_T c1_f_y;
  int32_T c1_i184;
  (void)chartInstance;
  sf_mex_import(c1_parentId, sf_mex_dup(c1_f_u), &c1_i184, 1, 6, 0U, 0, 0U, 0);
  c1_f_y = c1_i184;
  sf_mex_destroy(&c1_f_u);
  return c1_f_y;
}

static void c1_j_sf_marshallIn(void *chartInstanceVoid, const mxArray
  *c1_mxArrayInData, const char_T *c1_varName, void *c1_outData)
{
  const mxArray *c1_b_sfEvent;
  const char_T *c1_identifier;
  emlrtMsgIdentifier c1_thisId;
  int32_T c1_f_y;
  SFc1_mirageTrackingInstanceStruct *chartInstance;
  chartInstance = (SFc1_mirageTrackingInstanceStruct *)chartInstanceVoid;
  c1_b_sfEvent = sf_mex_dup(c1_mxArrayInData);
  c1_identifier = c1_varName;
  c1_thisId.fIdentifier = c1_identifier;
  c1_thisId.fParent = NULL;
  c1_f_y = c1_k_emlrt_marshallIn(chartInstance, sf_mex_dup(c1_b_sfEvent),
    &c1_thisId);
  sf_mex_destroy(&c1_b_sfEvent);
  *(int32_T *)c1_outData = c1_f_y;
  sf_mex_destroy(&c1_mxArrayInData);
}

static boolean_T c1_l_emlrt_marshallIn(SFc1_mirageTrackingInstanceStruct
  *chartInstance, const mxArray *c1_b_isInitialized, const char_T *c1_identifier)
{
  boolean_T c1_f_y;
  emlrtMsgIdentifier c1_thisId;
  c1_thisId.fIdentifier = c1_identifier;
  c1_thisId.fParent = NULL;
  c1_f_y = c1_m_emlrt_marshallIn(chartInstance, sf_mex_dup(c1_b_isInitialized),
    &c1_thisId);
  sf_mex_destroy(&c1_b_isInitialized);
  return c1_f_y;
}

static boolean_T c1_m_emlrt_marshallIn(SFc1_mirageTrackingInstanceStruct
  *chartInstance, const mxArray *c1_f_u, const emlrtMsgIdentifier *c1_parentId)
{
  boolean_T c1_f_y;
  boolean_T c1_b0;
  (void)chartInstance;
  sf_mex_import(c1_parentId, sf_mex_dup(c1_f_u), &c1_b0, 1, 11, 0U, 0, 0U, 0);
  c1_f_y = c1_b0;
  sf_mex_destroy(&c1_f_u);
  return c1_f_y;
}

static uint8_T c1_n_emlrt_marshallIn(SFc1_mirageTrackingInstanceStruct
  *chartInstance, const mxArray *c1_b_is_active_c1_mirageTracking, const char_T *
  c1_identifier)
{
  uint8_T c1_f_y;
  emlrtMsgIdentifier c1_thisId;
  c1_thisId.fIdentifier = c1_identifier;
  c1_thisId.fParent = NULL;
  c1_f_y = c1_o_emlrt_marshallIn(chartInstance, sf_mex_dup
    (c1_b_is_active_c1_mirageTracking), &c1_thisId);
  sf_mex_destroy(&c1_b_is_active_c1_mirageTracking);
  return c1_f_y;
}

static uint8_T c1_o_emlrt_marshallIn(SFc1_mirageTrackingInstanceStruct
  *chartInstance, const mxArray *c1_f_u, const emlrtMsgIdentifier *c1_parentId)
{
  uint8_T c1_f_y;
  uint8_T c1_u2;
  (void)chartInstance;
  sf_mex_import(c1_parentId, sf_mex_dup(c1_f_u), &c1_u2, 1, 3, 0U, 0, 0U, 0);
  c1_f_y = c1_u2;
  sf_mex_destroy(&c1_f_u);
  return c1_f_y;
}

static void c1_b_imcomplement(SFc1_mirageTrackingInstanceStruct *chartInstance,
  real32_T c1_im[3686400])
{
  int32_T c1_i185;
  (void)chartInstance;
  for (c1_i185 = 0; c1_i185 < 3686400; c1_i185++) {
    c1_im[c1_i185] = 1.0F - c1_im[c1_i185];
  }
}

static void c1_b_eml_sort(SFc1_mirageTrackingInstanceStruct *chartInstance,
  real_T c1_x[4], int32_T c1_idx[4])
{
  c1_b_eml_sort_idx(chartInstance, c1_x, c1_idx);
}

static void c1_b_eml_sort_idx(SFc1_mirageTrackingInstanceStruct *chartInstance,
  real_T c1_x[4], int32_T c1_idx[4])
{
  int32_T c1_i186;
  int32_T c1_i187;
  real_T c1_x4[4];
  int32_T c1_i188;
  int32_T c1_idx4[4];
  int32_T c1_nNaNs;
  int32_T c1_ib;
  int32_T c1_k;
  int32_T c1_b_k;
  real_T c1_b_x;
  boolean_T c1_d_b;
  real_T c1_xwork[4];
  int32_T c1_quartetOffset;
  int32_T c1_i1;
  int32_T c1_i2;
  int32_T c1_i3;
  int32_T c1_i4;
  int32_T c1_perm[4];
  int32_T c1_wOffset;
  int32_T c1_tOffset;
  int32_T c1_n;
  int32_T c1_i189;
  int32_T c1_b_ib;
  int32_T c1_e_b;
  int32_T c1_f_b;
  boolean_T c1_overflow;
  int32_T c1_c_k;
  int32_T c1_m;
  int32_T c1_b_m;
  int32_T c1_g_b;
  int32_T c1_h_b;
  boolean_T c1_b_overflow;
  int32_T c1_d_k;
  int32_T c1_itmp;
  int32_T c1_b_nNaNs;
  int32_T c1_nNonNaN;
  int32_T c1_b_n;
  int32_T c1_nBlocks;
  int32_T c1_bLen;
  int32_T c1_tailOffset;
  int32_T c1_nTail;
  int32_T c1_bLen2;
  int32_T c1_nPairs;
  int32_T c1_b_nPairs;
  int32_T c1_i_b;
  int32_T c1_j_b;
  boolean_T c1_c_overflow;
  int32_T c1_e_k;
  int32_T c1_f_k;
  for (c1_i186 = 0; c1_i186 < 4; c1_i186++) {
    c1_idx[c1_i186] = 0;
  }

  for (c1_i187 = 0; c1_i187 < 4; c1_i187++) {
    c1_x4[c1_i187] = 0.0;
  }

  for (c1_i188 = 0; c1_i188 < 4; c1_i188++) {
    c1_idx4[c1_i188] = 0;
  }

  c1_nNaNs = 0;
  c1_ib = 0;
  for (c1_k = 1; c1_k < 5; c1_k++) {
    c1_b_k = c1_k - 1;
    c1_b_x = c1_x[c1_b_k];
    c1_d_b = muDoubleScalarIsNaN(c1_b_x);
    if (c1_d_b) {
      c1_idx[_SFD_EML_ARRAY_BOUNDS_CHECK("", 4 - c1_nNaNs, 1, 4, 1, 0) - 1] =
        c1_b_k + 1;
      c1_xwork[_SFD_EML_ARRAY_BOUNDS_CHECK("", 4 - c1_nNaNs, 1, 4, 1, 0) - 1] =
        c1_x[c1_b_k];
      c1_nNaNs++;
    } else {
      c1_ib++;
      c1_idx4[c1_ib - 1] = c1_b_k + 1;
      c1_x4[c1_ib - 1] = c1_x[c1_b_k];
      if (c1_ib == 4) {
        c1_quartetOffset = c1_b_k - c1_nNaNs;
        if (c1_x4[0] <= c1_x4[1]) {
          c1_i1 = 1;
          c1_i2 = 2;
        } else {
          c1_i1 = 2;
          c1_i2 = 1;
        }

        if (c1_x4[2] <= c1_x4[3]) {
          c1_i3 = 3;
          c1_i4 = 4;
        } else {
          c1_i3 = 4;
          c1_i4 = 3;
        }

        if (c1_x4[c1_i1 - 1] <= c1_x4[c1_i3 - 1]) {
          if (c1_x4[c1_i2 - 1] <= c1_x4[c1_i3 - 1]) {
            c1_perm[0] = c1_i1;
            c1_perm[1] = c1_i2;
            c1_perm[2] = c1_i3;
            c1_perm[3] = c1_i4;
          } else if (c1_x4[c1_i2 - 1] <= c1_x4[c1_i4 - 1]) {
            c1_perm[0] = c1_i1;
            c1_perm[1] = c1_i3;
            c1_perm[2] = c1_i2;
            c1_perm[3] = c1_i4;
          } else {
            c1_perm[0] = c1_i1;
            c1_perm[1] = c1_i3;
            c1_perm[2] = c1_i4;
            c1_perm[3] = c1_i2;
          }
        } else if (c1_x4[c1_i1 - 1] <= c1_x4[c1_i4 - 1]) {
          if (c1_x4[c1_i2 - 1] <= c1_x4[c1_i4 - 1]) {
            c1_perm[0] = c1_i3;
            c1_perm[1] = c1_i1;
            c1_perm[2] = c1_i2;
            c1_perm[3] = c1_i4;
          } else {
            c1_perm[0] = c1_i3;
            c1_perm[1] = c1_i1;
            c1_perm[2] = c1_i4;
            c1_perm[3] = c1_i2;
          }
        } else {
          c1_perm[0] = c1_i3;
          c1_perm[1] = c1_i4;
          c1_perm[2] = c1_i1;
          c1_perm[3] = c1_i2;
        }

        c1_idx[_SFD_EML_ARRAY_BOUNDS_CHECK("", c1_quartetOffset - 2, 1, 4, 1, 0)
          - 1] = c1_idx4[c1_perm[0] - 1];
        c1_idx[_SFD_EML_ARRAY_BOUNDS_CHECK("", c1_quartetOffset - 1, 1, 4, 1, 0)
          - 1] = c1_idx4[c1_perm[1] - 1];
        c1_idx[_SFD_EML_ARRAY_BOUNDS_CHECK("", c1_quartetOffset, 1, 4, 1, 0) - 1]
          = c1_idx4[c1_perm[2] - 1];
        c1_idx[_SFD_EML_ARRAY_BOUNDS_CHECK("", c1_quartetOffset + 1, 1, 4, 1, 0)
          - 1] = c1_idx4[c1_perm[3] - 1];
        c1_x[_SFD_EML_ARRAY_BOUNDS_CHECK("", c1_quartetOffset - 2, 1, 4, 1, 0) -
          1] = c1_x4[c1_perm[0] - 1];
        c1_x[_SFD_EML_ARRAY_BOUNDS_CHECK("", c1_quartetOffset - 1, 1, 4, 1, 0) -
          1] = c1_x4[c1_perm[1] - 1];
        c1_x[_SFD_EML_ARRAY_BOUNDS_CHECK("", c1_quartetOffset, 1, 4, 1, 0) - 1] =
          c1_x4[c1_perm[2] - 1];
        c1_x[_SFD_EML_ARRAY_BOUNDS_CHECK("", c1_quartetOffset + 1, 1, 4, 1, 0) -
          1] = c1_x4[c1_perm[3] - 1];
        c1_ib = 0;
      }
    }
  }

  c1_wOffset = 4 - c1_nNaNs;
  c1_tOffset = c1_wOffset;
  if (c1_ib > 0) {
    c1_n = c1_ib;
    for (c1_i189 = 0; c1_i189 < 4; c1_i189++) {
      c1_perm[c1_i189] = 0;
    }

    if (c1_n == 1) {
      c1_perm[0] = 1;
    } else if (c1_n == 2) {
      if (c1_x4[0] <= c1_x4[1]) {
        c1_perm[0] = 1;
        c1_perm[1] = 2;
      } else {
        c1_perm[0] = 2;
        c1_perm[1] = 1;
      }
    } else if (c1_x4[0] <= c1_x4[1]) {
      if (c1_x4[1] <= c1_x4[2]) {
        c1_perm[0] = 1;
        c1_perm[1] = 2;
        c1_perm[2] = 3;
      } else if (c1_x4[0] <= c1_x4[2]) {
        c1_perm[0] = 1;
        c1_perm[1] = 3;
        c1_perm[2] = 2;
      } else {
        c1_perm[0] = 3;
        c1_perm[1] = 1;
        c1_perm[2] = 2;
      }
    } else if (c1_x4[0] <= c1_x4[2]) {
      c1_perm[0] = 2;
      c1_perm[1] = 1;
      c1_perm[2] = 3;
    } else if (c1_x4[1] <= c1_x4[2]) {
      c1_perm[0] = 2;
      c1_perm[1] = 3;
      c1_perm[2] = 1;
    } else {
      c1_perm[0] = 3;
      c1_perm[1] = 2;
      c1_perm[2] = 1;
    }

    c1_b_ib = c1_ib;
    c1_e_b = c1_b_ib;
    c1_f_b = c1_e_b;
    if (1 > c1_f_b) {
      c1_overflow = false;
    } else {
      c1_overflow = (c1_f_b > 2147483646);
    }

    if (c1_overflow) {
      c1_check_forloop_overflow_error(chartInstance, true);
    }

    for (c1_c_k = 1; c1_c_k <= c1_b_ib; c1_c_k++) {
      c1_b_k = c1_c_k;
      c1_idx[_SFD_EML_ARRAY_BOUNDS_CHECK("", (c1_tOffset - c1_ib) + c1_b_k, 1, 4,
        1, 0) - 1] = c1_idx4[_SFD_EML_ARRAY_BOUNDS_CHECK("", c1_perm[c1_b_k - 1],
        1, 4, 1, 0) - 1];
      c1_x[_SFD_EML_ARRAY_BOUNDS_CHECK("", (c1_tOffset - c1_ib) + c1_b_k, 1, 4,
        1, 0) - 1] = c1_x4[_SFD_EML_ARRAY_BOUNDS_CHECK("", c1_perm[c1_b_k - 1],
        1, 4, 1, 0) - 1];
    }
  }

  c1_m = (c1_nNaNs >> 1) + 1;
  c1_b_m = c1_m - 1;
  c1_g_b = c1_b_m;
  c1_h_b = c1_g_b;
  if (1 > c1_h_b) {
    c1_b_overflow = false;
  } else {
    c1_b_overflow = (c1_h_b > 2147483646);
  }

  if (c1_b_overflow) {
    c1_check_forloop_overflow_error(chartInstance, true);
  }

  for (c1_d_k = 1; c1_d_k <= c1_b_m; c1_d_k++) {
    c1_b_k = c1_d_k;
    c1_itmp = c1_idx[_SFD_EML_ARRAY_BOUNDS_CHECK("", c1_tOffset + c1_b_k, 1, 4,
      1, 0) - 1];
    c1_idx[_SFD_EML_ARRAY_BOUNDS_CHECK("", c1_tOffset + c1_b_k, 1, 4, 1, 0) - 1]
      = c1_idx[_SFD_EML_ARRAY_BOUNDS_CHECK("", 5 - c1_b_k, 1, 4, 1, 0) - 1];
    c1_idx[_SFD_EML_ARRAY_BOUNDS_CHECK("", 5 - c1_b_k, 1, 4, 1, 0) - 1] =
      c1_itmp;
    c1_x[_SFD_EML_ARRAY_BOUNDS_CHECK("", c1_tOffset + c1_b_k, 1, 4, 1, 0) - 1] =
      c1_xwork[_SFD_EML_ARRAY_BOUNDS_CHECK("", 5 - c1_b_k, 1, 4, 1, 0) - 1];
    c1_x[_SFD_EML_ARRAY_BOUNDS_CHECK("", 5 - c1_b_k, 1, 4, 1, 0) - 1] =
      c1_xwork[_SFD_EML_ARRAY_BOUNDS_CHECK("", c1_wOffset + c1_b_k, 1, 4, 1, 0)
      - 1];
  }

  if ((c1_nNaNs & 1) != 0) {
    c1_x[_SFD_EML_ARRAY_BOUNDS_CHECK("", c1_tOffset + c1_m, 1, 4, 1, 0) - 1] =
      c1_xwork[_SFD_EML_ARRAY_BOUNDS_CHECK("", c1_wOffset + c1_m, 1, 4, 1, 0) -
      1];
  }

  c1_b_nNaNs = c1_nNaNs;
  c1_nNonNaN = 4 - c1_b_nNaNs;
  if (c1_nNonNaN > 1) {
    c1_b_n = c1_nNonNaN;
    c1_nBlocks = c1_b_n >> 2;
    c1_bLen = 4;
    while (c1_nBlocks > 1) {
      if ((c1_nBlocks & 1) != 0) {
        c1_nBlocks--;
        c1_tailOffset = c1_bLen * c1_nBlocks;
        c1_nTail = c1_b_n - c1_tailOffset;
        if (c1_nTail > c1_bLen) {
          c1_b_merge(chartInstance, c1_idx, c1_x, c1_tailOffset, c1_bLen,
                     c1_nTail - c1_bLen);
        }
      }

      c1_bLen2 = c1_bLen << 1;
      c1_nPairs = c1_nBlocks >> 1;
      c1_b_nPairs = c1_nPairs;
      c1_i_b = c1_b_nPairs;
      c1_j_b = c1_i_b;
      if (1 > c1_j_b) {
        c1_c_overflow = false;
      } else {
        c1_c_overflow = (c1_j_b > 2147483646);
      }

      if (c1_c_overflow) {
        c1_check_forloop_overflow_error(chartInstance, true);
      }

      for (c1_e_k = 1; c1_e_k <= c1_b_nPairs; c1_e_k++) {
        c1_f_k = c1_e_k - 1;
        c1_b_merge(chartInstance, c1_idx, c1_x, c1_f_k * c1_bLen2, c1_bLen,
                   c1_bLen);
      }

      c1_bLen = c1_bLen2;
      c1_nBlocks = c1_nPairs;
    }

    if (c1_b_n > c1_bLen) {
      c1_b_merge(chartInstance, c1_idx, c1_x, 0, c1_bLen, c1_b_n - c1_bLen);
    }
  }
}

static void c1_b_merge(SFc1_mirageTrackingInstanceStruct *chartInstance, int32_T
  c1_idx[4], real_T c1_x[4], int32_T c1_offset, int32_T c1_np, int32_T c1_nq)
{
  int32_T c1_n;
  int32_T c1_b_n;
  int32_T c1_d_b;
  int32_T c1_e_b;
  boolean_T c1_overflow;
  int32_T c1_j;
  int32_T c1_b_j;
  int32_T c1_iwork[4];
  real_T c1_xwork[4];
  int32_T c1_p;
  int32_T c1_pend;
  int32_T c1_q;
  int32_T c1_qend;
  int32_T c1_iout;
  int32_T c1_offset1;
  int32_T c1_b_p;
  int32_T c1_b_pend;
  int32_T c1_f_b;
  int32_T c1_g_b;
  boolean_T c1_b_overflow;
  int32_T c1_c_j;
  int32_T exitg1;
  if (c1_nq == 0) {
  } else {
    c1_n = c1_np + c1_nq;
    c1_b_n = c1_n;
    c1_d_b = c1_b_n;
    c1_e_b = c1_d_b;
    if (1 > c1_e_b) {
      c1_overflow = false;
    } else {
      c1_overflow = (c1_e_b > 2147483646);
    }

    if (c1_overflow) {
      c1_check_forloop_overflow_error(chartInstance, true);
    }

    for (c1_j = 1; c1_j <= c1_b_n; c1_j++) {
      c1_b_j = c1_j;
      c1_iwork[_SFD_EML_ARRAY_BOUNDS_CHECK("", c1_b_j, 1, 4, 1, 0) - 1] =
        c1_idx[_SFD_EML_ARRAY_BOUNDS_CHECK("", c1_offset + c1_b_j, 1, 4, 1, 0) -
        1];
      c1_xwork[c1_b_j - 1] = c1_x[_SFD_EML_ARRAY_BOUNDS_CHECK("", c1_offset +
        c1_b_j, 1, 4, 1, 0) - 1];
    }

    c1_p = 0;
    c1_pend = c1_np;
    c1_q = c1_pend;
    c1_qend = c1_pend + c1_nq;
    c1_iout = c1_offset;
    do {
      exitg1 = 0;
      c1_iout++;
      if (c1_xwork[_SFD_EML_ARRAY_BOUNDS_CHECK("", c1_p + 1, 1, 4, 1, 0) - 1] <=
          c1_xwork[_SFD_EML_ARRAY_BOUNDS_CHECK("", c1_q + 1, 1, 4, 1, 0) - 1]) {
        c1_idx[_SFD_EML_ARRAY_BOUNDS_CHECK("", c1_iout, 1, 4, 1, 0) - 1] =
          c1_iwork[c1_p];
        c1_x[c1_iout - 1] = c1_xwork[c1_p];
        if (c1_p + 1 < c1_pend) {
          c1_p++;
        } else {
          exitg1 = 1;
        }
      } else {
        c1_idx[_SFD_EML_ARRAY_BOUNDS_CHECK("", c1_iout, 1, 4, 1, 0) - 1] =
          c1_iwork[c1_q];
        c1_x[c1_iout - 1] = c1_xwork[c1_q];
        if (c1_q + 1 < c1_qend) {
          c1_q++;
        } else {
          c1_offset1 = c1_iout - c1_p;
          c1_b_p = c1_p + 1;
          c1_b_pend = c1_pend;
          c1_f_b = c1_b_pend;
          c1_g_b = c1_f_b;
          c1_b_overflow = (c1_g_b > 2147483646);
          if (c1_b_overflow) {
            c1_check_forloop_overflow_error(chartInstance, true);
          }

          for (c1_c_j = c1_b_p; c1_c_j <= c1_b_pend; c1_c_j++) {
            c1_b_j = c1_c_j;
            c1_idx[_SFD_EML_ARRAY_BOUNDS_CHECK("", c1_offset1 + c1_b_j, 1, 4, 1,
              0) - 1] = c1_iwork[_SFD_EML_ARRAY_BOUNDS_CHECK("", c1_b_j, 1, 4, 1,
              0) - 1];
            c1_x[_SFD_EML_ARRAY_BOUNDS_CHECK("", c1_offset1 + c1_b_j, 1, 4, 1, 0)
              - 1] = c1_xwork[c1_b_j - 1];
          }

          exitg1 = 1;
        }
      }
    } while (exitg1 == 0);
  }
}

static void init_dsm_address_info(SFc1_mirageTrackingInstanceStruct
  *chartInstance)
{
  (void)chartInstance;
}

static void init_simulink_io_address(SFc1_mirageTrackingInstanceStruct
  *chartInstance)
{
  chartInstance->c1_b_l_rgb = (real32_T (*)[3686400])
    ssGetInputPortSignal_wrapper(chartInstance->S, 0);
  chartInstance->c1_L_p = (real_T (*)[8])ssGetOutputPortSignal_wrapper
    (chartInstance->S, 1);
  chartInstance->c1_b_r_rgb = (real32_T (*)[3686400])
    ssGetInputPortSignal_wrapper(chartInstance->S, 1);
  chartInstance->c1_R_p = (real_T (*)[8])ssGetOutputPortSignal_wrapper
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

void sf_c1_mirageTracking_get_check_sum(mxArray *plhs[])
{
  ((real_T *)mxGetPr((plhs[0])))[0] = (real_T)(2659894683U);
  ((real_T *)mxGetPr((plhs[0])))[1] = (real_T)(3186459922U);
  ((real_T *)mxGetPr((plhs[0])))[2] = (real_T)(3342282034U);
  ((real_T *)mxGetPr((plhs[0])))[3] = (real_T)(859440723U);
}

mxArray* sf_c1_mirageTracking_get_post_codegen_info(void);
mxArray *sf_c1_mirageTracking_get_autoinheritance_info(void)
{
  const char *autoinheritanceFields[] = { "checksum", "inputs", "parameters",
    "outputs", "locals", "postCodegenInfo" };

  mxArray *mxAutoinheritanceInfo = mxCreateStructMatrix(1, 1, sizeof
    (autoinheritanceFields)/sizeof(autoinheritanceFields[0]),
    autoinheritanceFields);

  {
    mxArray *mxChecksum = mxCreateString("ehhQYyxbI5HquenABu2k6D");
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
      mxSetField(mxType,0,"base",mxCreateDoubleScalar(9));
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
      mxSetField(mxType,0,"base",mxCreateDoubleScalar(9));
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
    mxArray* mxPostCodegenInfo = sf_c1_mirageTracking_get_post_codegen_info();
    mxSetField(mxAutoinheritanceInfo,0,"postCodegenInfo",mxPostCodegenInfo);
  }

  return(mxAutoinheritanceInfo);
}

mxArray *sf_c1_mirageTracking_third_party_uses_info(void)
{
  mxArray * mxcell3p = mxCreateCellMatrix(1,1);
  mxSetCell(mxcell3p, 0, mxCreateString(
             "images.internal.coder.buildable.Medianfilter_ippBuildable"));
  return(mxcell3p);
}

mxArray *sf_c1_mirageTracking_jit_fallback_info(void)
{
  const char *infoFields[] = { "fallbackType", "fallbackReason",
    "hiddenFallbackType", "hiddenFallbackReason", "incompatibleSymbol" };

  mxArray *mxInfo = mxCreateStructMatrix(1, 1, 5, infoFields);
  mxArray *fallbackType = mxCreateString("late");
  mxArray *fallbackReason = mxCreateString("ir_function_calls");
  mxArray *hiddenFallbackType = mxCreateString("");
  mxArray *hiddenFallbackReason = mxCreateString("");
  mxArray *incompatibleSymbol = mxCreateString("ippMedianFilter_real32");
  mxSetField(mxInfo, 0, infoFields[0], fallbackType);
  mxSetField(mxInfo, 0, infoFields[1], fallbackReason);
  mxSetField(mxInfo, 0, infoFields[2], hiddenFallbackType);
  mxSetField(mxInfo, 0, infoFields[3], hiddenFallbackReason);
  mxSetField(mxInfo, 0, infoFields[4], incompatibleSymbol);
  return mxInfo;
}

mxArray *sf_c1_mirageTracking_updateBuildInfo_args_info(void)
{
  mxArray *mxBIArgs = mxCreateCellMatrix(1,0);
  return mxBIArgs;
}

mxArray* sf_c1_mirageTracking_get_post_codegen_info(void)
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

static const mxArray *sf_get_sim_state_info_c1_mirageTracking(void)
{
  const char *infoFields[] = { "chartChecksum", "varInfo" };

  mxArray *mxInfo = mxCreateStructMatrix(1, 1, 2, infoFields);
  const char *infoEncStr[] = {
    "100 S1x4'type','srcId','name','auxInfo'{{M[1],M[5],T\"L_p\",},{M[1],M[7],T\"R_p\",},{M[4],M[0],T\"isInitialized\",},{M[8],M[0],T\"is_active_c1_mirageTracking\",}}"
  };

  mxArray *mxVarInfo = sf_mex_decode_encoded_mx_struct_array(infoEncStr, 4, 10);
  mxArray *mxChecksum = mxCreateDoubleMatrix(1, 4, mxREAL);
  sf_c1_mirageTracking_get_check_sum(&mxChecksum);
  mxSetField(mxInfo, 0, infoFields[0], mxChecksum);
  mxSetField(mxInfo, 0, infoFields[1], mxVarInfo);
  return mxInfo;
}

static void chart_debug_initialization(SimStruct *S, unsigned int
  fullDebuggerInitialization)
{
  if (!sim_mode_is_rtw_gen(S)) {
    SFc1_mirageTrackingInstanceStruct *chartInstance;
    ChartRunTimeInfo * crtInfo = (ChartRunTimeInfo *)(ssGetUserData(S));
    ChartInfoStruct * chartInfo = (ChartInfoStruct *)(crtInfo->instanceInfo);
    chartInstance = (SFc1_mirageTrackingInstanceStruct *)
      chartInfo->chartInstance;
    if (ssIsFirstInitCond(S) && fullDebuggerInitialization==1) {
      /* do this only if simulation is starting */
      {
        unsigned int chartAlreadyPresent;
        chartAlreadyPresent = sf_debug_initialize_chart
          (sfGlobalDebugInstanceStruct,
           _mirageTrackingMachineNumber_,
           1,
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
        init_script_number_translation(_mirageTrackingMachineNumber_,
          chartInstance->chartNumber,chartInstance->instanceNumber);
        if (chartAlreadyPresent==0) {
          /* this is the first instance */
          sf_debug_set_chart_disable_implicit_casting
            (sfGlobalDebugInstanceStruct,_mirageTrackingMachineNumber_,
             chartInstance->chartNumber,1);
          sf_debug_set_chart_event_thresholds(sfGlobalDebugInstanceStruct,
            _mirageTrackingMachineNumber_,
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
        _SFD_CV_INIT_EML(0,1,2,0,2,0,0,0,2,0,0,0);
        _SFD_CV_INIT_EML_FCN(0,0,"eML_blk_kernel",0,-1,2044);
        _SFD_CV_INIT_EML_FCN(0,1,"myMat2gray",2046,-1,2254);
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
          _SFD_SET_DATA_COMPILED_PROPS(0,SF_SINGLE,3,&(dimVector[0]),0,0,0,0.0,
            1.0,0,0,(MexFcnForType)c1_b_sf_marshallOut,(MexInFcnForType)NULL);
        }

        {
          unsigned int dimVector[3];
          dimVector[0]= 960;
          dimVector[1]= 1280;
          dimVector[2]= 3;
          _SFD_SET_DATA_COMPILED_PROPS(1,SF_SINGLE,3,&(dimVector[0]),0,0,0,0.0,
            1.0,0,0,(MexFcnForType)c1_b_sf_marshallOut,(MexInFcnForType)NULL);
        }

        {
          unsigned int dimVector[2];
          dimVector[0]= 2;
          dimVector[1]= 4;
          _SFD_SET_DATA_COMPILED_PROPS(2,SF_DOUBLE,2,&(dimVector[0]),0,0,0,0.0,
            1.0,0,0,(MexFcnForType)c1_sf_marshallOut,(MexInFcnForType)
            c1_sf_marshallIn);
        }

        {
          unsigned int dimVector[2];
          dimVector[0]= 2;
          dimVector[1]= 4;
          _SFD_SET_DATA_COMPILED_PROPS(3,SF_DOUBLE,2,&(dimVector[0]),0,0,0,0.0,
            1.0,0,0,(MexFcnForType)c1_sf_marshallOut,(MexInFcnForType)
            c1_sf_marshallIn);
        }
      }
    } else {
      sf_debug_reset_current_state_configuration(sfGlobalDebugInstanceStruct,
        _mirageTrackingMachineNumber_,chartInstance->chartNumber,
        chartInstance->instanceNumber);
    }
  }
}

static void chart_debug_initialize_data_addresses(SimStruct *S)
{
  if (!sim_mode_is_rtw_gen(S)) {
    SFc1_mirageTrackingInstanceStruct *chartInstance;
    ChartRunTimeInfo * crtInfo = (ChartRunTimeInfo *)(ssGetUserData(S));
    ChartInfoStruct * chartInfo = (ChartInfoStruct *)(crtInfo->instanceInfo);
    chartInstance = (SFc1_mirageTrackingInstanceStruct *)
      chartInfo->chartInstance;
    if (ssIsFirstInitCond(S)) {
      /* do this only if simulation is starting and after we know the addresses of all data */
      {
        _SFD_SET_DATA_VALUE_PTR(0U, *chartInstance->c1_b_l_rgb);
        _SFD_SET_DATA_VALUE_PTR(2U, *chartInstance->c1_L_p);
        _SFD_SET_DATA_VALUE_PTR(1U, *chartInstance->c1_b_r_rgb);
        _SFD_SET_DATA_VALUE_PTR(3U, *chartInstance->c1_R_p);
      }
    }
  }
}

static const char* sf_get_instance_specialization(void)
{
  return "sTAgcHd5RspX8KxNG3MW2FG";
}

static void sf_opaque_initialize_c1_mirageTracking(void *chartInstanceVar)
{
  chart_debug_initialization(((SFc1_mirageTrackingInstanceStruct*)
    chartInstanceVar)->S,0);
  initialize_params_c1_mirageTracking((SFc1_mirageTrackingInstanceStruct*)
    chartInstanceVar);
  initialize_c1_mirageTracking((SFc1_mirageTrackingInstanceStruct*)
    chartInstanceVar);
}

static void sf_opaque_enable_c1_mirageTracking(void *chartInstanceVar)
{
  enable_c1_mirageTracking((SFc1_mirageTrackingInstanceStruct*) chartInstanceVar);
}

static void sf_opaque_disable_c1_mirageTracking(void *chartInstanceVar)
{
  disable_c1_mirageTracking((SFc1_mirageTrackingInstanceStruct*)
    chartInstanceVar);
}

static void sf_opaque_gateway_c1_mirageTracking(void *chartInstanceVar)
{
  sf_gateway_c1_mirageTracking((SFc1_mirageTrackingInstanceStruct*)
    chartInstanceVar);
}

static const mxArray* sf_opaque_get_sim_state_c1_mirageTracking(SimStruct* S)
{
  ChartRunTimeInfo * crtInfo = (ChartRunTimeInfo *)(ssGetUserData(S));
  ChartInfoStruct * chartInfo = (ChartInfoStruct *)(crtInfo->instanceInfo);
  return get_sim_state_c1_mirageTracking((SFc1_mirageTrackingInstanceStruct*)
    chartInfo->chartInstance);         /* raw sim ctx */
}

static void sf_opaque_set_sim_state_c1_mirageTracking(SimStruct* S, const
  mxArray *st)
{
  ChartRunTimeInfo * crtInfo = (ChartRunTimeInfo *)(ssGetUserData(S));
  ChartInfoStruct * chartInfo = (ChartInfoStruct *)(crtInfo->instanceInfo);
  set_sim_state_c1_mirageTracking((SFc1_mirageTrackingInstanceStruct*)
    chartInfo->chartInstance, st);
}

static void sf_opaque_terminate_c1_mirageTracking(void *chartInstanceVar)
{
  if (chartInstanceVar!=NULL) {
    SimStruct *S = ((SFc1_mirageTrackingInstanceStruct*) chartInstanceVar)->S;
    ChartRunTimeInfo * crtInfo = (ChartRunTimeInfo *)(ssGetUserData(S));
    if (sim_mode_is_rtw_gen(S) || sim_mode_is_external(S)) {
      sf_clear_rtw_identifier(S);
      unload_mirageTracking_optimization_info();
    }

    finalize_c1_mirageTracking((SFc1_mirageTrackingInstanceStruct*)
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
  initSimStructsc1_mirageTracking((SFc1_mirageTrackingInstanceStruct*)
    chartInstanceVar);
}

extern unsigned int sf_machine_global_initializer_called(void);
static void mdlProcessParameters_c1_mirageTracking(SimStruct *S)
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
    initialize_params_c1_mirageTracking((SFc1_mirageTrackingInstanceStruct*)
      (chartInfo->chartInstance));
  }
}

static void mdlSetWorkWidths_c1_mirageTracking(SimStruct *S)
{
  ssMdlUpdateIsEmpty(S, 1);
  if (sim_mode_is_rtw_gen(S) || sim_mode_is_external(S)) {
    mxArray *infoStruct = load_mirageTracking_optimization_info();
    int_T chartIsInlinable =
      (int_T)sf_is_chart_inlinable(sf_get_instance_specialization(),infoStruct,1);
    ssSetStateflowIsInlinable(S,chartIsInlinable);
    ssSetRTWCG(S,1);
    ssSetEnableFcnIsTrivial(S,1);
    ssSetDisableFcnIsTrivial(S,1);
    ssSetNotMultipleInlinable(S,sf_rtw_info_uint_prop
      (sf_get_instance_specialization(),infoStruct,1,
       "gatewayCannotBeInlinedMultipleTimes"));
    sf_update_buildInfo(sf_get_instance_specialization(),infoStruct,1);
    if (chartIsInlinable) {
      ssSetInputPortOptimOpts(S, 0, SS_REUSABLE_AND_LOCAL);
      ssSetInputPortOptimOpts(S, 1, SS_REUSABLE_AND_LOCAL);
      sf_mark_chart_expressionable_inputs(S,sf_get_instance_specialization(),
        infoStruct,1,2);
      sf_mark_chart_reusable_outputs(S,sf_get_instance_specialization(),
        infoStruct,1,2);
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

    sf_set_rtw_dwork_info(S,sf_get_instance_specialization(),infoStruct,1);
    ssSetHasSubFunctions(S,!(chartIsInlinable));
  } else {
  }

  ssSetOptions(S,ssGetOptions(S)|SS_OPTION_WORKS_WITH_CODE_REUSE);
  ssSetChecksum0(S,(1128336388U));
  ssSetChecksum1(S,(843039240U));
  ssSetChecksum2(S,(2213297731U));
  ssSetChecksum3(S,(3107023038U));
  ssSetmdlDerivatives(S, NULL);
  ssSetExplicitFCSSCtrl(S,1);
  ssSupportsMultipleExecInstances(S,1);
}

static void mdlRTW_c1_mirageTracking(SimStruct *S)
{
  if (sim_mode_is_rtw_gen(S)) {
    ssWriteRTWStrParam(S, "StateflowChartType", "Embedded MATLAB");
  }
}

static void mdlStart_c1_mirageTracking(SimStruct *S)
{
  SFc1_mirageTrackingInstanceStruct *chartInstance;
  ChartRunTimeInfo * crtInfo = (ChartRunTimeInfo *)utMalloc(sizeof
    (ChartRunTimeInfo));
  chartInstance = (SFc1_mirageTrackingInstanceStruct *)utMalloc(sizeof
    (SFc1_mirageTrackingInstanceStruct));
  memset(chartInstance, 0, sizeof(SFc1_mirageTrackingInstanceStruct));
  if (chartInstance==NULL) {
    sf_mex_error_message("Could not allocate memory for chart instance.");
  }

  chartInstance->chartInfo.chartInstance = chartInstance;
  chartInstance->chartInfo.isEMLChart = 1;
  chartInstance->chartInfo.chartInitialized = 0;
  chartInstance->chartInfo.sFunctionGateway =
    sf_opaque_gateway_c1_mirageTracking;
  chartInstance->chartInfo.initializeChart =
    sf_opaque_initialize_c1_mirageTracking;
  chartInstance->chartInfo.terminateChart =
    sf_opaque_terminate_c1_mirageTracking;
  chartInstance->chartInfo.enableChart = sf_opaque_enable_c1_mirageTracking;
  chartInstance->chartInfo.disableChart = sf_opaque_disable_c1_mirageTracking;
  chartInstance->chartInfo.getSimState =
    sf_opaque_get_sim_state_c1_mirageTracking;
  chartInstance->chartInfo.setSimState =
    sf_opaque_set_sim_state_c1_mirageTracking;
  chartInstance->chartInfo.getSimStateInfo =
    sf_get_sim_state_info_c1_mirageTracking;
  chartInstance->chartInfo.zeroCrossings = NULL;
  chartInstance->chartInfo.outputs = NULL;
  chartInstance->chartInfo.derivatives = NULL;
  chartInstance->chartInfo.mdlRTW = mdlRTW_c1_mirageTracking;
  chartInstance->chartInfo.mdlStart = mdlStart_c1_mirageTracking;
  chartInstance->chartInfo.mdlSetWorkWidths = mdlSetWorkWidths_c1_mirageTracking;
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

void c1_mirageTracking_method_dispatcher(SimStruct *S, int_T method, void *data)
{
  switch (method) {
   case SS_CALL_MDL_START:
    mdlStart_c1_mirageTracking(S);
    break;

   case SS_CALL_MDL_SET_WORK_WIDTHS:
    mdlSetWorkWidths_c1_mirageTracking(S);
    break;

   case SS_CALL_MDL_PROCESS_PARAMETERS:
    mdlProcessParameters_c1_mirageTracking(S);
    break;

   default:
    /* Unhandled method */
    sf_mex_error_message("Stateflow Internal Error:\n"
                         "Error calling c1_mirageTracking_method_dispatcher.\n"
                         "Can't handle method %d.\n", method);
    break;
  }
}
