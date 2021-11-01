/* Include files */

#include <stddef.h>
#include "blas.h"
#include "Robot_Control_v01_sfun.h"
#include "c3_Robot_Control_v01.h"
#include "mwmathutil.h"
#define CHARTINSTANCE_CHARTNUMBER      (chartInstance->chartNumber)
#define CHARTINSTANCE_INSTANCENUMBER   (chartInstance->instanceNumber)
#include "Robot_Control_v01_sfun_debug_macros.h"
#define _SF_MEX_LISTEN_FOR_CTRL_C(S)   sf_mex_listen_for_ctrl_c(sfGlobalDebugInstanceStruct,S);

/* Type Definitions */

/* Named Constants */
#define CALL_EVENT                     (-1)

/* Variable Declarations */

/* Variable Definitions */
static real_T _sfTime_;
static const char * c3_debug_family_names[24] = { "p1x", "p1y", "p1z", "P", "f",
  "pp", "nl", "noise", "robotPose", "robotPoseDes", "world2robotDes", "L_p",
  "R_p", "S_PDes", "robot", "param", "sol", "world2robot", "psi", "nargin",
  "nargout", "q", "qr", "qtilde" };

static const char * c3_b_debug_family_names[9] = { "a", "b", "xu", "n", "nargin",
  "nargout", "xini", "xend", "x" };

static const char * c3_c_debug_family_names[9] = { "nargin", "nargout", "tx",
  "ty", "tz", "phi", "theta", "psi", "pose" };

static const char * c3_d_debug_family_names[14] = { "Dx", "Dy", "Dz", "phi",
  "theta", "psi", "Rz", "Ry", "Rx", "Tr", "nargin", "nargout", "Pose", "y_E_x" };

static const char * c3_e_debug_family_names[12] = { "varargin", "pose", "tx",
  "ty", "tz", "phi", "theta", "psi", "DesRobotPose", "nargin", "nargout", "M" };

static const char * c3_f_debug_family_names[10] = { "nargin", "nargout", "pose",
  "fx", "fy", "cx", "cy", "s", "d", "camera" };

static const char * c3_g_debug_family_names[6] = { "nargin", "nargout", "pose",
  "leftCamera", "rightCamera", "robot" };

static const char * c3_h_debug_family_names[4] = { "nargin", "nargout", "cam",
  "K" };

static const char * c3_i_debug_family_names[10] = { "robot2cam", "K", "M", "C_P",
  "p", "nargin", "nargout", "S_P", "camera", "retVal" };

static const char * c3_j_debug_family_names[10] = { "world2robot", "S_P",
  "L_retVal", "R_retVal", "nargin", "nargout", "P", "robot", "L_p", "R_p" };

static const char * c3_k_debug_family_names[16] = { "leftCamPose",
  "rightCamPose", "leftCamera", "rightCamera", "world2robotDes", "nargin",
  "nargout", "pose", "robotPoseDes", "P", "f", "pp", "L_p", "R_p", "S_PDes",
  "robot" };

static const char * c3_l_debug_family_names[14] = { "n", "j", "i", "O", "Vx",
  "Vy", "nargin", "nargout", "Cam_PDes", "S_PDes", "Err", "M", "equations",
  "constants" };

static const char * c3_m_debug_family_names[23] = { "L_retValDes", "R_retValDes",
  "L_pDes", "R_pDes", "L_PDes", "R_PDes", "L_M", "R_M", "Err_Left", "Err_Right",
  "Left_Equations", "Left_Constants", "Right_Equations", "Right_Constants",
  "AllEquations", "AllConstants", "nargin", "nargout", "robot", "S_PDes", "L_p",
  "R_p", "poseError" };

static const char * c3_n_debug_family_names[6] = { "nargin", "nargout", "rotMat",
  "roll", "pitch", "yaw" };

/* Function Declarations */
static void initialize_c3_Robot_Control_v01(SFc3_Robot_Control_v01InstanceStruct
  *chartInstance);
static void initialize_params_c3_Robot_Control_v01
  (SFc3_Robot_Control_v01InstanceStruct *chartInstance);
static void enable_c3_Robot_Control_v01(SFc3_Robot_Control_v01InstanceStruct
  *chartInstance);
static void disable_c3_Robot_Control_v01(SFc3_Robot_Control_v01InstanceStruct
  *chartInstance);
static void c3_update_debugger_state_c3_Robot_Control_v01
  (SFc3_Robot_Control_v01InstanceStruct *chartInstance);
static const mxArray *get_sim_state_c3_Robot_Control_v01
  (SFc3_Robot_Control_v01InstanceStruct *chartInstance);
static void set_sim_state_c3_Robot_Control_v01
  (SFc3_Robot_Control_v01InstanceStruct *chartInstance, const mxArray *c3_st);
static void finalize_c3_Robot_Control_v01(SFc3_Robot_Control_v01InstanceStruct
  *chartInstance);
static void sf_gateway_c3_Robot_Control_v01(SFc3_Robot_Control_v01InstanceStruct
  *chartInstance);
static void mdl_start_c3_Robot_Control_v01(SFc3_Robot_Control_v01InstanceStruct *
  chartInstance);
static void c3_chartstep_c3_Robot_Control_v01
  (SFc3_Robot_Control_v01InstanceStruct *chartInstance);
static void initSimStructsc3_Robot_Control_v01
  (SFc3_Robot_Control_v01InstanceStruct *chartInstance);
static void c3_random(SFc3_Robot_Control_v01InstanceStruct *chartInstance,
                      real_T c3_xini, real_T c3_xend, real_T c3_x[4]);
static c3_sKe46gf3wfNeOrzL8yzt6nF c3_newPose
  (SFc3_Robot_Control_v01InstanceStruct *chartInstance, real_T c3_tx, real_T
   c3_ty, real_T c3_tz, real_T c3_phi, real_T c3_theta, real_T c3_psi);
static void c3_transformationMatrix(SFc3_Robot_Control_v01InstanceStruct
  *chartInstance, c3_sKe46gf3wfNeOrzL8yzt6nF c3_varargin_1, real_T c3_M[16]);
static void c3_EulerTrans(SFc3_Robot_Control_v01InstanceStruct *chartInstance,
  c3_smuwpGFfhzM6MfA0WGyOk6E c3_Pose, real_T c3_y_E_x[16]);
static void c3_newCamera(SFc3_Robot_Control_v01InstanceStruct *chartInstance,
  c3_sKe46gf3wfNeOrzL8yzt6nF c3_pose, real_T c3_fx, real_T c3_fy, real_T c3_cx,
  real_T c3_cy, real_T c3_s, real_T c3_d, c3_s6d9wPYgmS3cwGRsMJd1GAE *c3_camera);
static void c3_transformImageSpace(SFc3_Robot_Control_v01InstanceStruct
  *chartInstance, real_T c3_S_P[16], c3_s6d9wPYgmS3cwGRsMJd1GAE *c3_camera,
  c3_sZiCA83jGK4zdmds7wvlXUF *c3_retVal);
static void c3_mirage(SFc3_Robot_Control_v01InstanceStruct *chartInstance,
                      c3_smiE7r3wwCYeiytRpYXSxxE *c3_robot, real_T c3_S_PDes[16],
                      real_T c3_L_p[8], real_T c3_R_p[8], real_T c3_poseError[12]);
static void c3_getEquations(SFc3_Robot_Control_v01InstanceStruct *chartInstance,
  real_T c3_Cam_PDes[16], real_T c3_S_PDes[16], real_T c3_Err[8], real_T c3_M[16],
  real_T c3_equations[96], real_T c3_constants[8]);
static void init_script_number_translation(uint32_T c3_machineNumber, uint32_T
  c3_chartNumber, uint32_T c3_instanceNumber);
static void c3_emlrt_marshallIn(SFc3_Robot_Control_v01InstanceStruct
  *chartInstance, const mxArray *c3_sprintf, const char_T *c3_identifier, char_T
  c3_y[14]);
static void c3_b_emlrt_marshallIn(SFc3_Robot_Control_v01InstanceStruct
  *chartInstance, const mxArray *c3_u, const emlrtMsgIdentifier *c3_parentId,
  char_T c3_y[14]);
static const mxArray *c3_sf_marshallOut(void *chartInstanceVoid, void *c3_inData);
static void c3_c_emlrt_marshallIn(SFc3_Robot_Control_v01InstanceStruct
  *chartInstance, const mxArray *c3_b_qtilde, const char_T *c3_identifier,
  real_T c3_y[3]);
static void c3_d_emlrt_marshallIn(SFc3_Robot_Control_v01InstanceStruct
  *chartInstance, const mxArray *c3_u, const emlrtMsgIdentifier *c3_parentId,
  real_T c3_y[3]);
static void c3_sf_marshallIn(void *chartInstanceVoid, const mxArray
  *c3_mxArrayInData, const char_T *c3_varName, void *c3_outData);
static const mxArray *c3_b_sf_marshallOut(void *chartInstanceVoid, void
  *c3_inData);
static real_T c3_e_emlrt_marshallIn(SFc3_Robot_Control_v01InstanceStruct
  *chartInstance, const mxArray *c3_u, const emlrtMsgIdentifier *c3_parentId);
static void c3_b_sf_marshallIn(void *chartInstanceVoid, const mxArray
  *c3_mxArrayInData, const char_T *c3_varName, void *c3_outData);
static const mxArray *c3_c_sf_marshallOut(void *chartInstanceVoid, void
  *c3_inData);
static void c3_f_emlrt_marshallIn(SFc3_Robot_Control_v01InstanceStruct
  *chartInstance, const mxArray *c3_u, const emlrtMsgIdentifier *c3_parentId,
  real_T c3_y[16]);
static void c3_c_sf_marshallIn(void *chartInstanceVoid, const mxArray
  *c3_mxArrayInData, const char_T *c3_varName, void *c3_outData);
static const mxArray *c3_d_sf_marshallOut(void *chartInstanceVoid, void
  *c3_inData);
static void c3_g_emlrt_marshallIn(SFc3_Robot_Control_v01InstanceStruct
  *chartInstance, const mxArray *c3_u, const emlrtMsgIdentifier *c3_parentId,
  real_T c3_y[12]);
static void c3_d_sf_marshallIn(void *chartInstanceVoid, const mxArray
  *c3_mxArrayInData, const char_T *c3_varName, void *c3_outData);
static const mxArray *c3_e_sf_marshallOut(void *chartInstanceVoid, void
  *c3_inData);
static void c3_h_emlrt_marshallIn(SFc3_Robot_Control_v01InstanceStruct
  *chartInstance, const mxArray *c3_u, const emlrtMsgIdentifier *c3_parentId,
  c3_smiE7r3wwCYeiytRpYXSxxE *c3_y);
static c3_sKe46gf3wfNeOrzL8yzt6nF c3_i_emlrt_marshallIn
  (SFc3_Robot_Control_v01InstanceStruct *chartInstance, const mxArray *c3_u,
   const emlrtMsgIdentifier *c3_parentId);
static void c3_j_emlrt_marshallIn(SFc3_Robot_Control_v01InstanceStruct
  *chartInstance, const mxArray *c3_u, const emlrtMsgIdentifier *c3_parentId,
  c3_s6d9wPYgmS3cwGRsMJd1GAE *c3_y);
static void c3_e_sf_marshallIn(void *chartInstanceVoid, const mxArray
  *c3_mxArrayInData, const char_T *c3_varName, void *c3_outData);
static const mxArray *c3_f_sf_marshallOut(void *chartInstanceVoid, void
  *c3_inData);
static void c3_k_emlrt_marshallIn(SFc3_Robot_Control_v01InstanceStruct
  *chartInstance, const mxArray *c3_u, const emlrtMsgIdentifier *c3_parentId,
  real_T c3_y[8]);
static void c3_f_sf_marshallIn(void *chartInstanceVoid, const mxArray
  *c3_mxArrayInData, const char_T *c3_varName, void *c3_outData);
static const mxArray *c3_g_sf_marshallOut(void *chartInstanceVoid, void
  *c3_inData);
static void c3_g_sf_marshallIn(void *chartInstanceVoid, const mxArray
  *c3_mxArrayInData, const char_T *c3_varName, void *c3_outData);
static const mxArray *c3_h_sf_marshallOut(void *chartInstanceVoid, void
  *c3_inData);
static void c3_l_emlrt_marshallIn(SFc3_Robot_Control_v01InstanceStruct
  *chartInstance, const mxArray *c3_u, const emlrtMsgIdentifier *c3_parentId,
  real_T c3_y[4]);
static void c3_h_sf_marshallIn(void *chartInstanceVoid, const mxArray
  *c3_mxArrayInData, const char_T *c3_varName, void *c3_outData);
static const mxArray *c3_i_sf_marshallOut(void *chartInstanceVoid, void
  *c3_inData);
static void c3_m_emlrt_marshallIn(SFc3_Robot_Control_v01InstanceStruct
  *chartInstance, const mxArray *c3_u, const emlrtMsgIdentifier *c3_parentId,
  real_T c3_y[2]);
static void c3_i_sf_marshallIn(void *chartInstanceVoid, const mxArray
  *c3_mxArrayInData, const char_T *c3_varName, void *c3_outData);
static const mxArray *c3_j_sf_marshallOut(void *chartInstanceVoid, void
  *c3_inData);
static c3_smuwpGFfhzM6MfA0WGyOk6E c3_n_emlrt_marshallIn
  (SFc3_Robot_Control_v01InstanceStruct *chartInstance, const mxArray *c3_u,
   const emlrtMsgIdentifier *c3_parentId);
static void c3_j_sf_marshallIn(void *chartInstanceVoid, const mxArray
  *c3_mxArrayInData, const char_T *c3_varName, void *c3_outData);
static const mxArray *c3_k_sf_marshallOut(void *chartInstanceVoid, void
  *c3_inData);
static const mxArray *c3_l_sf_marshallOut(void *chartInstanceVoid, void
  *c3_inData);
static void c3_k_sf_marshallIn(void *chartInstanceVoid, const mxArray
  *c3_mxArrayInData, const char_T *c3_varName, void *c3_outData);
static const mxArray *c3_m_sf_marshallOut(void *chartInstanceVoid, void
  *c3_inData);
static void c3_o_emlrt_marshallIn(SFc3_Robot_Control_v01InstanceStruct
  *chartInstance, const mxArray *c3_u, const emlrtMsgIdentifier *c3_parentId,
  c3_sZiCA83jGK4zdmds7wvlXUF *c3_y);
static void c3_l_sf_marshallIn(void *chartInstanceVoid, const mxArray
  *c3_mxArrayInData, const char_T *c3_varName, void *c3_outData);
static const mxArray *c3_n_sf_marshallOut(void *chartInstanceVoid, void
  *c3_inData);
static void c3_p_emlrt_marshallIn(SFc3_Robot_Control_v01InstanceStruct
  *chartInstance, const mxArray *c3_u, const emlrtMsgIdentifier *c3_parentId,
  real_T c3_y[8]);
static void c3_m_sf_marshallIn(void *chartInstanceVoid, const mxArray
  *c3_mxArrayInData, const char_T *c3_varName, void *c3_outData);
static const mxArray *c3_o_sf_marshallOut(void *chartInstanceVoid, void
  *c3_inData);
static void c3_q_emlrt_marshallIn(SFc3_Robot_Control_v01InstanceStruct
  *chartInstance, const mxArray *c3_u, const emlrtMsgIdentifier *c3_parentId,
  real_T c3_y[96]);
static void c3_n_sf_marshallIn(void *chartInstanceVoid, const mxArray
  *c3_mxArrayInData, const char_T *c3_varName, void *c3_outData);
static const mxArray *c3_p_sf_marshallOut(void *chartInstanceVoid, void
  *c3_inData);
static void c3_r_emlrt_marshallIn(SFc3_Robot_Control_v01InstanceStruct
  *chartInstance, const mxArray *c3_u, const emlrtMsgIdentifier *c3_parentId,
  real_T c3_y[3]);
static void c3_o_sf_marshallIn(void *chartInstanceVoid, const mxArray
  *c3_mxArrayInData, const char_T *c3_varName, void *c3_outData);
static const mxArray *c3_q_sf_marshallOut(void *chartInstanceVoid, void
  *c3_inData);
static void c3_s_emlrt_marshallIn(SFc3_Robot_Control_v01InstanceStruct
  *chartInstance, const mxArray *c3_u, const emlrtMsgIdentifier *c3_parentId,
  real_T c3_y[12]);
static void c3_p_sf_marshallIn(void *chartInstanceVoid, const mxArray
  *c3_mxArrayInData, const char_T *c3_varName, void *c3_outData);
static const mxArray *c3_r_sf_marshallOut(void *chartInstanceVoid, void
  *c3_inData);
static void c3_t_emlrt_marshallIn(SFc3_Robot_Control_v01InstanceStruct
  *chartInstance, const mxArray *c3_u, const emlrtMsgIdentifier *c3_parentId,
  real_T c3_y[12]);
static void c3_q_sf_marshallIn(void *chartInstanceVoid, const mxArray
  *c3_mxArrayInData, const char_T *c3_varName, void *c3_outData);
static const mxArray *c3_s_sf_marshallOut(void *chartInstanceVoid, void
  *c3_inData);
static void c3_u_emlrt_marshallIn(SFc3_Robot_Control_v01InstanceStruct
  *chartInstance, const mxArray *c3_u, const emlrtMsgIdentifier *c3_parentId,
  real_T c3_y[16]);
static void c3_r_sf_marshallIn(void *chartInstanceVoid, const mxArray
  *c3_mxArrayInData, const char_T *c3_varName, void *c3_outData);
static const mxArray *c3_t_sf_marshallOut(void *chartInstanceVoid, void
  *c3_inData);
static void c3_v_emlrt_marshallIn(SFc3_Robot_Control_v01InstanceStruct
  *chartInstance, const mxArray *c3_u, const emlrtMsgIdentifier *c3_parentId,
  real_T c3_y[192]);
static void c3_s_sf_marshallIn(void *chartInstanceVoid, const mxArray
  *c3_mxArrayInData, const char_T *c3_varName, void *c3_outData);
static const mxArray *c3_u_sf_marshallOut(void *chartInstanceVoid, void
  *c3_inData);
static void c3_w_emlrt_marshallIn(SFc3_Robot_Control_v01InstanceStruct
  *chartInstance, const mxArray *c3_u, const emlrtMsgIdentifier *c3_parentId,
  real_T c3_y[9]);
static void c3_t_sf_marshallIn(void *chartInstanceVoid, const mxArray
  *c3_mxArrayInData, const char_T *c3_varName, void *c3_outData);
static void c3_info_helper(const mxArray **c3_info);
static const mxArray *c3_emlrt_marshallOut(const char * c3_u);
static const mxArray *c3_b_emlrt_marshallOut(const uint32_T c3_u);
static void c3_b_info_helper(const mxArray **c3_info);
static void c3_c_info_helper(const mxArray **c3_info);
static void c3_d_info_helper(const mxArray **c3_info);
static void c3_e_info_helper(const mxArray **c3_info);
static void c3_f_info_helper(const mxArray **c3_info);
static void c3_rand(SFc3_Robot_Control_v01InstanceStruct *chartInstance, real_T
                    c3_r[4]);
static void c3_eml_rand(SFc3_Robot_Control_v01InstanceStruct *chartInstance,
  real_T c3_r[4]);
static void c3_eml_rand_mt19937ar(SFc3_Robot_Control_v01InstanceStruct
  *chartInstance, uint32_T c3_d_state[625]);
static void c3_twister_state_vector(SFc3_Robot_Control_v01InstanceStruct
  *chartInstance, uint32_T c3_mt[625], real_T c3_seed, uint32_T c3_b_mt[625]);
static void c3_b_eml_rand_mt19937ar(SFc3_Robot_Control_v01InstanceStruct
  *chartInstance, uint32_T c3_d_state[625], uint32_T c3_e_state[625], real_T
  *c3_r);
static void c3_eml_eps(SFc3_Robot_Control_v01InstanceStruct *chartInstance);
static void c3_eml_error(SFc3_Robot_Control_v01InstanceStruct *chartInstance);
static void c3_eml_scalar_eg(SFc3_Robot_Control_v01InstanceStruct *chartInstance);
static void c3_eml_xgemm(SFc3_Robot_Control_v01InstanceStruct *chartInstance,
  real_T c3_A[16], real_T c3_B[16], real_T c3_C[16], real_T c3_b_C[16]);
static void c3_inv(SFc3_Robot_Control_v01InstanceStruct *chartInstance, real_T
                   c3_x[16], real_T c3_y[16]);
static void c3_realmin(SFc3_Robot_Control_v01InstanceStruct *chartInstance);
static void c3_eps(SFc3_Robot_Control_v01InstanceStruct *chartInstance);
static void c3_eml_switch_helper(SFc3_Robot_Control_v01InstanceStruct
  *chartInstance);
static void c3_eml_matlab_zgetrf(SFc3_Robot_Control_v01InstanceStruct
  *chartInstance, real_T c3_A[16], real_T c3_b_A[16], int32_T c3_ipiv[4],
  int32_T *c3_info);
static void c3_threshold(SFc3_Robot_Control_v01InstanceStruct *chartInstance);
static void c3_check_forloop_overflow_error(SFc3_Robot_Control_v01InstanceStruct
  *chartInstance, boolean_T c3_overflow);
static void c3_b_eml_switch_helper(SFc3_Robot_Control_v01InstanceStruct
  *chartInstance);
static void c3_eml_xgeru(SFc3_Robot_Control_v01InstanceStruct *chartInstance,
  int32_T c3_m, int32_T c3_n, real_T c3_alpha1, int32_T c3_ix0, int32_T c3_iy0,
  real_T c3_A[16], int32_T c3_ia0, real_T c3_b_A[16]);
static void c3_b_threshold(SFc3_Robot_Control_v01InstanceStruct *chartInstance);
static void c3_b_eml_scalar_eg(SFc3_Robot_Control_v01InstanceStruct
  *chartInstance);
static void c3_eml_ipiv2perm(SFc3_Robot_Control_v01InstanceStruct *chartInstance,
  int32_T c3_ipiv[4], int32_T c3_perm[4]);
static void c3_c_eml_switch_helper(SFc3_Robot_Control_v01InstanceStruct
  *chartInstance);
static void c3_eml_xtrsm(SFc3_Robot_Control_v01InstanceStruct *chartInstance,
  real_T c3_A[16], real_T c3_B[16], real_T c3_b_B[16]);
static void c3_d_eml_switch_helper(SFc3_Robot_Control_v01InstanceStruct
  *chartInstance);
static real_T c3_rdivide(SFc3_Robot_Control_v01InstanceStruct *chartInstance,
  real_T c3_x, real_T c3_y);
static real_T c3_norm(SFc3_Robot_Control_v01InstanceStruct *chartInstance,
                      real_T c3_x[16]);
static void c3_eml_warning(SFc3_Robot_Control_v01InstanceStruct *chartInstance);
static void c3_b_eml_warning(SFc3_Robot_Control_v01InstanceStruct *chartInstance,
  char_T c3_varargin_2[14]);
static void c3_setupPointsAndPixels(SFc3_Robot_Control_v01InstanceStruct
  *chartInstance, c3_sKe46gf3wfNeOrzL8yzt6nF c3_pose, c3_sKe46gf3wfNeOrzL8yzt6nF
  c3_robotPoseDes, real_T c3_P[16], real_T c3_f, real_T c3_pp[2], real_T c3_L_p
  [8], real_T c3_R_p[8], real_T c3_S_PDes[16], c3_smiE7r3wwCYeiytRpYXSxxE
  *c3_robot);
static void c3_b_rdivide(SFc3_Robot_Control_v01InstanceStruct *chartInstance,
  real_T c3_x[4], real_T c3_y[4], real_T c3_z[4]);
static void c3_mldivide(SFc3_Robot_Control_v01InstanceStruct *chartInstance,
  real_T c3_A[192], real_T c3_B[16], real_T c3_Y[12]);
static void c3_c_eml_scalar_eg(SFc3_Robot_Control_v01InstanceStruct
  *chartInstance);
static real_T c3_sqrt(SFc3_Robot_Control_v01InstanceStruct *chartInstance,
                      real_T c3_x);
static void c3_b_eml_error(SFc3_Robot_Control_v01InstanceStruct *chartInstance);
static real_T c3_eml_xnrm2(SFc3_Robot_Control_v01InstanceStruct *chartInstance,
  real_T c3_x[192], int32_T c3_ix0);
static void c3_c_threshold(SFc3_Robot_Control_v01InstanceStruct *chartInstance);
static int32_T c3_eml_ixamax(SFc3_Robot_Control_v01InstanceStruct *chartInstance,
  int32_T c3_n, real_T c3_x[12], int32_T c3_ix0);
static void c3_eml_xswap(SFc3_Robot_Control_v01InstanceStruct *chartInstance,
  real_T c3_x[192], int32_T c3_ix0, int32_T c3_iy0, real_T c3_b_x[192]);
static void c3_eml_matlab_zlarfg(SFc3_Robot_Control_v01InstanceStruct
  *chartInstance, int32_T c3_n, real_T c3_alpha1, real_T c3_x[192], int32_T
  c3_ix0, real_T *c3_b_alpha1, real_T c3_b_x[192], real_T *c3_tau);
static real_T c3_b_eml_xnrm2(SFc3_Robot_Control_v01InstanceStruct *chartInstance,
  int32_T c3_n, real_T c3_x[192], int32_T c3_ix0);
static void c3_eml_xscal(SFc3_Robot_Control_v01InstanceStruct *chartInstance,
  int32_T c3_n, real_T c3_a, real_T c3_x[192], int32_T c3_ix0, real_T c3_b_x[192]);
static void c3_e_eml_switch_helper(SFc3_Robot_Control_v01InstanceStruct
  *chartInstance);
static void c3_b_eml_matlab_zlarfg(SFc3_Robot_Control_v01InstanceStruct
  *chartInstance, real_T c3_alpha1, real_T c3_x, real_T *c3_b_alpha1, real_T
  *c3_b_x, real_T *c3_tau);
static void c3_c_eml_xnrm2(SFc3_Robot_Control_v01InstanceStruct *chartInstance);
static real_T c3_b_eml_xscal(SFc3_Robot_Control_v01InstanceStruct *chartInstance,
  real_T c3_x);
static void c3_eml_matlab_zlarf(SFc3_Robot_Control_v01InstanceStruct
  *chartInstance, int32_T c3_m, int32_T c3_n, int32_T c3_iv0, real_T c3_tau,
  real_T c3_C[192], int32_T c3_ic0, real_T c3_work[12], real_T c3_b_C[192],
  real_T c3_b_work[12]);
static void c3_eml_xgemv(SFc3_Robot_Control_v01InstanceStruct *chartInstance,
  int32_T c3_m, int32_T c3_n, real_T c3_A[192], int32_T c3_ia0, real_T c3_x[192],
  int32_T c3_ix0, real_T c3_y[12], real_T c3_b_y[12]);
static void c3_below_threshold(SFc3_Robot_Control_v01InstanceStruct
  *chartInstance);
static void c3_eml_xgerc(SFc3_Robot_Control_v01InstanceStruct *chartInstance,
  int32_T c3_m, int32_T c3_n, real_T c3_alpha1, int32_T c3_ix0, real_T c3_y[12],
  real_T c3_A[192], int32_T c3_ia0, real_T c3_b_A[192]);
static void c3_c_eml_warning(SFc3_Robot_Control_v01InstanceStruct *chartInstance,
  real_T c3_varargin_2, char_T c3_varargin_3[14]);
static real_T c3_mpower(SFc3_Robot_Control_v01InstanceStruct *chartInstance,
  real_T c3_a);
static real_T c3_atan2(SFc3_Robot_Control_v01InstanceStruct *chartInstance,
  real_T c3_y, real_T c3_x);
static const mxArray *c3_v_sf_marshallOut(void *chartInstanceVoid, void
  *c3_inData);
static int32_T c3_x_emlrt_marshallIn(SFc3_Robot_Control_v01InstanceStruct
  *chartInstance, const mxArray *c3_u, const emlrtMsgIdentifier *c3_parentId);
static void c3_u_sf_marshallIn(void *chartInstanceVoid, const mxArray
  *c3_mxArrayInData, const char_T *c3_varName, void *c3_outData);
static uint32_T c3_y_emlrt_marshallIn(SFc3_Robot_Control_v01InstanceStruct
  *chartInstance, const mxArray *c3_b_method, const char_T *c3_identifier);
static uint32_T c3_ab_emlrt_marshallIn(SFc3_Robot_Control_v01InstanceStruct
  *chartInstance, const mxArray *c3_u, const emlrtMsgIdentifier *c3_parentId);
static uint32_T c3_bb_emlrt_marshallIn(SFc3_Robot_Control_v01InstanceStruct
  *chartInstance, const mxArray *c3_d_state, const char_T *c3_identifier);
static uint32_T c3_cb_emlrt_marshallIn(SFc3_Robot_Control_v01InstanceStruct
  *chartInstance, const mxArray *c3_u, const emlrtMsgIdentifier *c3_parentId);
static void c3_db_emlrt_marshallIn(SFc3_Robot_Control_v01InstanceStruct
  *chartInstance, const mxArray *c3_d_state, const char_T *c3_identifier,
  uint32_T c3_y[625]);
static void c3_eb_emlrt_marshallIn(SFc3_Robot_Control_v01InstanceStruct
  *chartInstance, const mxArray *c3_u, const emlrtMsgIdentifier *c3_parentId,
  uint32_T c3_y[625]);
static void c3_fb_emlrt_marshallIn(SFc3_Robot_Control_v01InstanceStruct
  *chartInstance, const mxArray *c3_d_state, const char_T *c3_identifier,
  uint32_T c3_y[2]);
static void c3_gb_emlrt_marshallIn(SFc3_Robot_Control_v01InstanceStruct
  *chartInstance, const mxArray *c3_u, const emlrtMsgIdentifier *c3_parentId,
  uint32_T c3_y[2]);
static uint8_T c3_hb_emlrt_marshallIn(SFc3_Robot_Control_v01InstanceStruct
  *chartInstance, const mxArray *c3_b_is_active_c3_Robot_Control_v01, const
  char_T *c3_identifier);
static uint8_T c3_ib_emlrt_marshallIn(SFc3_Robot_Control_v01InstanceStruct
  *chartInstance, const mxArray *c3_u, const emlrtMsgIdentifier *c3_parentId);
static void c3_b_twister_state_vector(SFc3_Robot_Control_v01InstanceStruct
  *chartInstance, uint32_T c3_mt[625], real_T c3_seed);
static real_T c3_c_eml_rand_mt19937ar(SFc3_Robot_Control_v01InstanceStruct
  *chartInstance, uint32_T c3_d_state[625]);
static void c3_b_eml_xgemm(SFc3_Robot_Control_v01InstanceStruct *chartInstance,
  real_T c3_A[16], real_T c3_B[16], real_T c3_C[16]);
static void c3_b_eml_matlab_zgetrf(SFc3_Robot_Control_v01InstanceStruct
  *chartInstance, real_T c3_A[16], int32_T c3_ipiv[4], int32_T *c3_info);
static void c3_b_eml_xgeru(SFc3_Robot_Control_v01InstanceStruct *chartInstance,
  int32_T c3_m, int32_T c3_n, real_T c3_alpha1, int32_T c3_ix0, int32_T c3_iy0,
  real_T c3_A[16], int32_T c3_ia0);
static void c3_b_eml_xtrsm(SFc3_Robot_Control_v01InstanceStruct *chartInstance,
  real_T c3_A[16], real_T c3_B[16]);
static void c3_b_sqrt(SFc3_Robot_Control_v01InstanceStruct *chartInstance,
                      real_T *c3_x);
static void c3_b_eml_xswap(SFc3_Robot_Control_v01InstanceStruct *chartInstance,
  real_T c3_x[192], int32_T c3_ix0, int32_T c3_iy0);
static real_T c3_c_eml_matlab_zlarfg(SFc3_Robot_Control_v01InstanceStruct
  *chartInstance, int32_T c3_n, real_T *c3_alpha1, real_T c3_x[192], int32_T
  c3_ix0);
static void c3_c_eml_xscal(SFc3_Robot_Control_v01InstanceStruct *chartInstance,
  int32_T c3_n, real_T c3_a, real_T c3_x[192], int32_T c3_ix0);
static real_T c3_d_eml_matlab_zlarfg(SFc3_Robot_Control_v01InstanceStruct
  *chartInstance, real_T *c3_alpha1, real_T *c3_x);
static void c3_d_eml_xscal(SFc3_Robot_Control_v01InstanceStruct *chartInstance,
  real_T *c3_x);
static void c3_b_eml_matlab_zlarf(SFc3_Robot_Control_v01InstanceStruct
  *chartInstance, int32_T c3_m, int32_T c3_n, int32_T c3_iv0, real_T c3_tau,
  real_T c3_C[192], int32_T c3_ic0, real_T c3_work[12]);
static void c3_b_eml_xgemv(SFc3_Robot_Control_v01InstanceStruct *chartInstance,
  int32_T c3_m, int32_T c3_n, real_T c3_A[192], int32_T c3_ia0, real_T c3_x[192],
  int32_T c3_ix0, real_T c3_y[12]);
static void c3_b_eml_xgerc(SFc3_Robot_Control_v01InstanceStruct *chartInstance,
  int32_T c3_m, int32_T c3_n, real_T c3_alpha1, int32_T c3_ix0, real_T c3_y[12],
  real_T c3_A[192], int32_T c3_ia0);
static void init_dsm_address_info(SFc3_Robot_Control_v01InstanceStruct
  *chartInstance);
static void init_simulink_io_address(SFc3_Robot_Control_v01InstanceStruct
  *chartInstance);

/* Function Definitions */
static void initialize_c3_Robot_Control_v01(SFc3_Robot_Control_v01InstanceStruct
  *chartInstance)
{
  chartInstance->c3_sfEvent = CALL_EVENT;
  _sfTime_ = sf_get_time(chartInstance->S);
  chartInstance->c3_method_not_empty = false;
  chartInstance->c3_state_not_empty = false;
  chartInstance->c3_b_state_not_empty = false;
  chartInstance->c3_c_state_not_empty = false;
  chartInstance->c3_is_active_c3_Robot_Control_v01 = 0U;
}

static void initialize_params_c3_Robot_Control_v01
  (SFc3_Robot_Control_v01InstanceStruct *chartInstance)
{
  (void)chartInstance;
}

static void enable_c3_Robot_Control_v01(SFc3_Robot_Control_v01InstanceStruct
  *chartInstance)
{
  _sfTime_ = sf_get_time(chartInstance->S);
}

static void disable_c3_Robot_Control_v01(SFc3_Robot_Control_v01InstanceStruct
  *chartInstance)
{
  _sfTime_ = sf_get_time(chartInstance->S);
}

static void c3_update_debugger_state_c3_Robot_Control_v01
  (SFc3_Robot_Control_v01InstanceStruct *chartInstance)
{
  (void)chartInstance;
}

static const mxArray *get_sim_state_c3_Robot_Control_v01
  (SFc3_Robot_Control_v01InstanceStruct *chartInstance)
{
  const mxArray *c3_st;
  const mxArray *c3_y = NULL;
  int32_T c3_i0;
  real_T c3_u[3];
  const mxArray *c3_b_y = NULL;
  uint32_T c3_hoistedGlobal;
  uint32_T c3_b_u;
  const mxArray *c3_c_y = NULL;
  uint32_T c3_b_hoistedGlobal;
  uint32_T c3_c_u;
  const mxArray *c3_d_y = NULL;
  int32_T c3_i1;
  uint32_T c3_d_u[625];
  const mxArray *c3_e_y = NULL;
  int32_T c3_i2;
  uint32_T c3_e_u[2];
  const mxArray *c3_f_y = NULL;
  uint8_T c3_c_hoistedGlobal;
  uint8_T c3_f_u;
  const mxArray *c3_g_y = NULL;
  c3_st = NULL;
  c3_st = NULL;
  c3_y = NULL;
  sf_mex_assign(&c3_y, sf_mex_createcellmatrix(6, 1), false);
  for (c3_i0 = 0; c3_i0 < 3; c3_i0++) {
    c3_u[c3_i0] = (*chartInstance->c3_qtilde)[c3_i0];
  }

  c3_b_y = NULL;
  sf_mex_assign(&c3_b_y, sf_mex_create("y", c3_u, 0, 0U, 1U, 0U, 1, 3), false);
  sf_mex_setcell(c3_y, 0, c3_b_y);
  c3_hoistedGlobal = chartInstance->c3_method;
  c3_b_u = c3_hoistedGlobal;
  c3_c_y = NULL;
  if (!chartInstance->c3_method_not_empty) {
    sf_mex_assign(&c3_c_y, sf_mex_create("y", NULL, 0, 0U, 1U, 0U, 2, 0, 0),
                  false);
  } else {
    sf_mex_assign(&c3_c_y, sf_mex_create("y", &c3_b_u, 7, 0U, 0U, 0U, 0), false);
  }

  sf_mex_setcell(c3_y, 1, c3_c_y);
  c3_b_hoistedGlobal = chartInstance->c3_state;
  c3_c_u = c3_b_hoistedGlobal;
  c3_d_y = NULL;
  if (!chartInstance->c3_state_not_empty) {
    sf_mex_assign(&c3_d_y, sf_mex_create("y", NULL, 0, 0U, 1U, 0U, 2, 0, 0),
                  false);
  } else {
    sf_mex_assign(&c3_d_y, sf_mex_create("y", &c3_c_u, 7, 0U, 0U, 0U, 0), false);
  }

  sf_mex_setcell(c3_y, 2, c3_d_y);
  for (c3_i1 = 0; c3_i1 < 625; c3_i1++) {
    c3_d_u[c3_i1] = chartInstance->c3_c_state[c3_i1];
  }

  c3_e_y = NULL;
  if (!chartInstance->c3_c_state_not_empty) {
    sf_mex_assign(&c3_e_y, sf_mex_create("y", NULL, 0, 0U, 1U, 0U, 2, 0, 0),
                  false);
  } else {
    sf_mex_assign(&c3_e_y, sf_mex_create("y", c3_d_u, 7, 0U, 1U, 0U, 1, 625),
                  false);
  }

  sf_mex_setcell(c3_y, 3, c3_e_y);
  for (c3_i2 = 0; c3_i2 < 2; c3_i2++) {
    c3_e_u[c3_i2] = chartInstance->c3_b_state[c3_i2];
  }

  c3_f_y = NULL;
  if (!chartInstance->c3_b_state_not_empty) {
    sf_mex_assign(&c3_f_y, sf_mex_create("y", NULL, 0, 0U, 1U, 0U, 2, 0, 0),
                  false);
  } else {
    sf_mex_assign(&c3_f_y, sf_mex_create("y", c3_e_u, 7, 0U, 1U, 0U, 1, 2),
                  false);
  }

  sf_mex_setcell(c3_y, 4, c3_f_y);
  c3_c_hoistedGlobal = chartInstance->c3_is_active_c3_Robot_Control_v01;
  c3_f_u = c3_c_hoistedGlobal;
  c3_g_y = NULL;
  sf_mex_assign(&c3_g_y, sf_mex_create("y", &c3_f_u, 3, 0U, 0U, 0U, 0), false);
  sf_mex_setcell(c3_y, 5, c3_g_y);
  sf_mex_assign(&c3_st, c3_y, false);
  return c3_st;
}

static void set_sim_state_c3_Robot_Control_v01
  (SFc3_Robot_Control_v01InstanceStruct *chartInstance, const mxArray *c3_st)
{
  const mxArray *c3_u;
  real_T c3_dv0[3];
  int32_T c3_i3;
  uint32_T c3_uv0[625];
  int32_T c3_i4;
  uint32_T c3_uv1[2];
  int32_T c3_i5;
  chartInstance->c3_doneDoubleBufferReInit = true;
  c3_u = sf_mex_dup(c3_st);
  c3_c_emlrt_marshallIn(chartInstance, sf_mex_dup(sf_mex_getcell(c3_u, 0)),
                        "qtilde", c3_dv0);
  for (c3_i3 = 0; c3_i3 < 3; c3_i3++) {
    (*chartInstance->c3_qtilde)[c3_i3] = c3_dv0[c3_i3];
  }

  chartInstance->c3_method = c3_y_emlrt_marshallIn(chartInstance, sf_mex_dup
    (sf_mex_getcell(c3_u, 1)), "method");
  chartInstance->c3_state = c3_bb_emlrt_marshallIn(chartInstance, sf_mex_dup
    (sf_mex_getcell(c3_u, 2)), "state");
  c3_db_emlrt_marshallIn(chartInstance, sf_mex_dup(sf_mex_getcell(c3_u, 3)),
    "state", c3_uv0);
  for (c3_i4 = 0; c3_i4 < 625; c3_i4++) {
    chartInstance->c3_c_state[c3_i4] = c3_uv0[c3_i4];
  }

  c3_fb_emlrt_marshallIn(chartInstance, sf_mex_dup(sf_mex_getcell(c3_u, 4)),
    "state", c3_uv1);
  for (c3_i5 = 0; c3_i5 < 2; c3_i5++) {
    chartInstance->c3_b_state[c3_i5] = c3_uv1[c3_i5];
  }

  chartInstance->c3_is_active_c3_Robot_Control_v01 = c3_hb_emlrt_marshallIn
    (chartInstance, sf_mex_dup(sf_mex_getcell(c3_u, 5)),
     "is_active_c3_Robot_Control_v01");
  sf_mex_destroy(&c3_u);
  c3_update_debugger_state_c3_Robot_Control_v01(chartInstance);
  sf_mex_destroy(&c3_st);
}

static void finalize_c3_Robot_Control_v01(SFc3_Robot_Control_v01InstanceStruct
  *chartInstance)
{
  (void)chartInstance;
}

static void sf_gateway_c3_Robot_Control_v01(SFc3_Robot_Control_v01InstanceStruct
  *chartInstance)
{
  int32_T c3_i6;
  int32_T c3_i7;
  int32_T c3_i8;
  _SFD_SYMBOL_SCOPE_PUSH(0U, 0U);
  _sfTime_ = sf_get_time(chartInstance->S);
  _SFD_CC_CALL(CHART_ENTER_SFUNCTION_TAG, 2U, chartInstance->c3_sfEvent);
  for (c3_i6 = 0; c3_i6 < 3; c3_i6++) {
    _SFD_DATA_RANGE_CHECK((*chartInstance->c3_q)[c3_i6], 0U);
  }

  chartInstance->c3_sfEvent = CALL_EVENT;
  c3_chartstep_c3_Robot_Control_v01(chartInstance);
  _SFD_SYMBOL_SCOPE_POP();
  _SFD_CHECK_FOR_STATE_INCONSISTENCY(_Robot_Control_v01MachineNumber_,
    chartInstance->chartNumber, chartInstance->instanceNumber);
  for (c3_i7 = 0; c3_i7 < 3; c3_i7++) {
    _SFD_DATA_RANGE_CHECK((*chartInstance->c3_qtilde)[c3_i7], 1U);
  }

  for (c3_i8 = 0; c3_i8 < 3; c3_i8++) {
    _SFD_DATA_RANGE_CHECK((*chartInstance->c3_qr)[c3_i8], 2U);
  }
}

static void mdl_start_c3_Robot_Control_v01(SFc3_Robot_Control_v01InstanceStruct *
  chartInstance)
{
  (void)chartInstance;
}

static void c3_chartstep_c3_Robot_Control_v01
  (SFc3_Robot_Control_v01InstanceStruct *chartInstance)
{
  int32_T c3_i9;
  real_T c3_b_q[3];
  int32_T c3_i10;
  real_T c3_b_qr[3];
  uint32_T c3_debug_family_var_map[24];
  real_T c3_p1x;
  real_T c3_p1y;
  real_T c3_p1z;
  real_T c3_P[16];
  real_T c3_f;
  real_T c3_pp[2];
  real_T c3_nl;
  real_T c3_noise[4];
  c3_sKe46gf3wfNeOrzL8yzt6nF c3_robotPose;
  c3_sKe46gf3wfNeOrzL8yzt6nF c3_robotPoseDes;
  real_T c3_world2robotDes[16];
  real_T c3_L_p[8];
  real_T c3_R_p[8];
  real_T c3_S_PDes[16];
  c3_smiE7r3wwCYeiytRpYXSxxE c3_robot;
  real_T c3_param[12];
  real_T c3_sol[16];
  real_T c3_world2robot[16];
  real_T c3_psi;
  real_T c3_nargin = 2.0;
  real_T c3_nargout = 1.0;
  real_T c3_b_qtilde[3];
  int32_T c3_i11;
  real_T c3_dv1[4];
  int32_T c3_i12;
  int32_T c3_i13;
  int32_T c3_i14;
  int32_T c3_i15;
  int32_T c3_i16;
  int32_T c3_i17;
  int32_T c3_i18;
  int32_T c3_i19;
  int32_T c3_i20;
  real_T c3_dv2[4];
  int32_T c3_i21;
  real_T c3_b_S_PDes[16];
  int32_T c3_i22;
  real_T c3_c_S_PDes[16];
  real_T c3_dv3[16];
  int32_T c3_i23;
  int32_T c3_i24;
  real_T c3_b_P[16];
  int32_T c3_i25;
  real_T c3_dv4[2];
  c3_smiE7r3wwCYeiytRpYXSxxE c3_b_robot;
  real_T c3_b_R_p[8];
  real_T c3_b_L_p[8];
  int32_T c3_i26;
  int32_T c3_i27;
  int32_T c3_i28;
  c3_smiE7r3wwCYeiytRpYXSxxE c3_c_robot;
  int32_T c3_i29;
  real_T c3_d_S_PDes[16];
  int32_T c3_i30;
  real_T c3_c_L_p[8];
  int32_T c3_i31;
  real_T c3_c_R_p[8];
  real_T c3_dv5[12];
  int32_T c3_i32;
  int32_T c3_i33;
  int32_T c3_k;
  int32_T c3_b_k;
  int32_T c3_i34;
  real_T c3_x[12];
  int32_T c3_c_k;
  int32_T c3_d_k;
  real_T c3_y[12];
  int32_T c3_i35;
  int32_T c3_i36;
  int32_T c3_i37;
  int32_T c3_i38;
  int32_T c3_i39;
  real_T c3_a[16];
  int32_T c3_i40;
  real_T c3_b[16];
  int32_T c3_i41;
  int32_T c3_i42;
  int32_T c3_i43;
  real_T c3_b_a[16];
  int32_T c3_i44;
  real_T c3_b_b[16];
  int32_T c3_i45;
  int32_T c3_i46;
  int32_T c3_i47;
  int32_T c3_i48;
  int32_T c3_i49;
  real_T c3_rotMat[9];
  uint32_T c3_b_debug_family_var_map[6];
  real_T c3_b_nargin = 1.0;
  real_T c3_b_nargout = 3.0;
  real_T c3_unusedU0;
  real_T c3_unusedU1;
  real_T c3_b_psi;
  real_T c3_b_x;
  real_T c3_c_x;
  real_T c3_d_x;
  real_T c3_e_x;
  real_T c3_A;
  real_T c3_B;
  real_T c3_b_y;
  real_T c3_f_x;
  real_T c3_g_x;
  real_T c3_b_A;
  real_T c3_b_B;
  real_T c3_c_y;
  real_T c3_h_x;
  real_T c3_i_x;
  real_T c3_c_A;
  real_T c3_c_B;
  real_T c3_d_y;
  real_T c3_j_x;
  real_T c3_k_x;
  real_T c3_d_A;
  real_T c3_d_B;
  real_T c3_e_y;
  int32_T c3_i50;
  int32_T c3_i51;
  _SFD_CC_CALL(CHART_ENTER_DURING_FUNCTION_TAG, 2U, chartInstance->c3_sfEvent);
  for (c3_i9 = 0; c3_i9 < 3; c3_i9++) {
    c3_b_q[c3_i9] = (*chartInstance->c3_q)[c3_i9];
  }

  for (c3_i10 = 0; c3_i10 < 3; c3_i10++) {
    c3_b_qr[c3_i10] = (*chartInstance->c3_qr)[c3_i10];
  }

  _SFD_SYMBOL_SCOPE_PUSH_EML(0U, 24U, 24U, c3_debug_family_names,
    c3_debug_family_var_map);
  _SFD_SYMBOL_SCOPE_ADD_EML_IMPORTABLE(&c3_p1x, 0U, c3_b_sf_marshallOut,
    c3_b_sf_marshallIn);
  _SFD_SYMBOL_SCOPE_ADD_EML_IMPORTABLE(&c3_p1y, 1U, c3_b_sf_marshallOut,
    c3_b_sf_marshallIn);
  _SFD_SYMBOL_SCOPE_ADD_EML_IMPORTABLE(&c3_p1z, 2U, c3_b_sf_marshallOut,
    c3_b_sf_marshallIn);
  _SFD_SYMBOL_SCOPE_ADD_EML_IMPORTABLE(c3_P, 3U, c3_c_sf_marshallOut,
    c3_c_sf_marshallIn);
  _SFD_SYMBOL_SCOPE_ADD_EML_IMPORTABLE(&c3_f, 4U, c3_b_sf_marshallOut,
    c3_b_sf_marshallIn);
  _SFD_SYMBOL_SCOPE_ADD_EML_IMPORTABLE(c3_pp, 5U, c3_i_sf_marshallOut,
    c3_i_sf_marshallIn);
  _SFD_SYMBOL_SCOPE_ADD_EML(&c3_nl, 6U, c3_b_sf_marshallOut);
  _SFD_SYMBOL_SCOPE_ADD_EML_IMPORTABLE(c3_noise, 7U, c3_h_sf_marshallOut,
    c3_h_sf_marshallIn);
  _SFD_SYMBOL_SCOPE_ADD_EML_IMPORTABLE(&c3_robotPose, 8U, c3_g_sf_marshallOut,
    c3_g_sf_marshallIn);
  _SFD_SYMBOL_SCOPE_ADD_EML_IMPORTABLE(&c3_robotPoseDes, 9U, c3_g_sf_marshallOut,
    c3_g_sf_marshallIn);
  _SFD_SYMBOL_SCOPE_ADD_EML_IMPORTABLE(c3_world2robotDes, 10U,
    c3_c_sf_marshallOut, c3_c_sf_marshallIn);
  _SFD_SYMBOL_SCOPE_ADD_EML_IMPORTABLE(c3_L_p, 11U, c3_f_sf_marshallOut,
    c3_f_sf_marshallIn);
  _SFD_SYMBOL_SCOPE_ADD_EML_IMPORTABLE(c3_R_p, 12U, c3_f_sf_marshallOut,
    c3_f_sf_marshallIn);
  _SFD_SYMBOL_SCOPE_ADD_EML_IMPORTABLE(c3_S_PDes, 13U, c3_c_sf_marshallOut,
    c3_c_sf_marshallIn);
  _SFD_SYMBOL_SCOPE_ADD_EML_IMPORTABLE(&c3_robot, 14U, c3_e_sf_marshallOut,
    c3_e_sf_marshallIn);
  _SFD_SYMBOL_SCOPE_ADD_EML_IMPORTABLE(c3_param, 15U, c3_d_sf_marshallOut,
    c3_d_sf_marshallIn);
  _SFD_SYMBOL_SCOPE_ADD_EML_IMPORTABLE(c3_sol, 16U, c3_c_sf_marshallOut,
    c3_c_sf_marshallIn);
  _SFD_SYMBOL_SCOPE_ADD_EML_IMPORTABLE(c3_world2robot, 17U, c3_c_sf_marshallOut,
    c3_c_sf_marshallIn);
  _SFD_SYMBOL_SCOPE_ADD_EML_IMPORTABLE(&c3_psi, 18U, c3_b_sf_marshallOut,
    c3_b_sf_marshallIn);
  _SFD_SYMBOL_SCOPE_ADD_EML_IMPORTABLE(&c3_nargin, 19U, c3_b_sf_marshallOut,
    c3_b_sf_marshallIn);
  _SFD_SYMBOL_SCOPE_ADD_EML_IMPORTABLE(&c3_nargout, 20U, c3_b_sf_marshallOut,
    c3_b_sf_marshallIn);
  _SFD_SYMBOL_SCOPE_ADD_EML(c3_b_q, 21U, c3_sf_marshallOut);
  _SFD_SYMBOL_SCOPE_ADD_EML(c3_b_qr, 22U, c3_sf_marshallOut);
  _SFD_SYMBOL_SCOPE_ADD_EML_IMPORTABLE(c3_b_qtilde, 23U, c3_sf_marshallOut,
    c3_sf_marshallIn);
  CV_EML_FCN(0, 0);
  _SFD_EML_CALL(0U, chartInstance->c3_sfEvent, 9);
  c3_p1x = 0.0;
  _SFD_EML_CALL(0U, chartInstance->c3_sfEvent, 10);
  c3_p1y = 0.0;
  _SFD_EML_CALL(0U, chartInstance->c3_sfEvent, 11);
  c3_p1z = 0.0;
  _SFD_EML_CALL(0U, chartInstance->c3_sfEvent, 18);
  for (c3_i11 = 0; c3_i11 < 16; c3_i11++) {
    c3_P[c3_i11] = 0.0;
  }

  _SFD_EML_CALL(0U, chartInstance->c3_sfEvent, 19);
  c3_random(chartInstance, -5.0, 5.0, c3_dv1);
  c3_i12 = 0;
  for (c3_i13 = 0; c3_i13 < 4; c3_i13++) {
    c3_P[c3_i12] = c3_dv1[c3_i13];
    c3_i12 += 4;
  }

  _SFD_EML_CALL(0U, chartInstance->c3_sfEvent, 20);
  c3_random(chartInstance, -5.0, 5.0, c3_dv1);
  c3_i14 = 0;
  for (c3_i15 = 0; c3_i15 < 4; c3_i15++) {
    c3_P[c3_i14 + 1] = c3_dv1[c3_i15];
    c3_i14 += 4;
  }

  _SFD_EML_CALL(0U, chartInstance->c3_sfEvent, 21);
  c3_random(chartInstance, -5.0, 5.0, c3_dv1);
  c3_i16 = 0;
  for (c3_i17 = 0; c3_i17 < 4; c3_i17++) {
    c3_P[c3_i16 + 2] = c3_dv1[c3_i17];
    c3_i16 += 4;
  }

  _SFD_EML_CALL(0U, chartInstance->c3_sfEvent, 22);
  c3_i18 = 0;
  for (c3_i19 = 0; c3_i19 < 4; c3_i19++) {
    c3_P[c3_i18 + 3] = 1.0;
    c3_i18 += 4;
  }

  _SFD_EML_CALL(0U, chartInstance->c3_sfEvent, 25);
  c3_f = 800.0;
  _SFD_EML_CALL(0U, chartInstance->c3_sfEvent, 26);
  for (c3_i20 = 0; c3_i20 < 2; c3_i20++) {
    c3_pp[c3_i20] = 320.0 + -80.0 * (real_T)c3_i20;
  }

  _SFD_EML_CALL(0U, chartInstance->c3_sfEvent, 28);
  c3_nl = 10.0;
  _SFD_EML_CALL(0U, chartInstance->c3_sfEvent, 29);
  c3_random(chartInstance, -10.0, 10.0, c3_dv2);
  for (c3_i21 = 0; c3_i21 < 4; c3_i21++) {
    c3_noise[c3_i21] = c3_dv2[c3_i21];
  }

  _SFD_EML_CALL(0U, chartInstance->c3_sfEvent, 32);
  c3_robotPose = c3_newPose(chartInstance, c3_b_q[0], c3_b_q[1], 0.0, 0.0, 0.0,
    c3_b_q[2]);
  _SFD_EML_CALL(0U, chartInstance->c3_sfEvent, 33);
  c3_robotPoseDes = c3_newPose(chartInstance, c3_b_qr[0], c3_b_qr[1], 0.0, 0.0,
    0.0, c3_b_qr[2]);
  _SFD_EML_CALL(0U, chartInstance->c3_sfEvent, 35);
  c3_transformationMatrix(chartInstance, c3_robotPoseDes, c3_b_S_PDes);
  for (c3_i22 = 0; c3_i22 < 16; c3_i22++) {
    c3_c_S_PDes[c3_i22] = c3_b_S_PDes[c3_i22];
  }

  c3_inv(chartInstance, c3_c_S_PDes, c3_dv3);
  for (c3_i23 = 0; c3_i23 < 16; c3_i23++) {
    c3_world2robotDes[c3_i23] = c3_dv3[c3_i23];
  }

  _SFD_EML_CALL(0U, chartInstance->c3_sfEvent, 37);
  for (c3_i24 = 0; c3_i24 < 16; c3_i24++) {
    c3_b_P[c3_i24] = c3_P[c3_i24];
  }

  for (c3_i25 = 0; c3_i25 < 2; c3_i25++) {
    c3_dv4[c3_i25] = 320.0 + -80.0 * (real_T)c3_i25;
  }

  c3_setupPointsAndPixels(chartInstance, c3_robotPose, c3_robotPoseDes, c3_b_P,
    800.0, c3_dv4, c3_b_L_p, c3_b_R_p, c3_b_S_PDes, &c3_b_robot);
  for (c3_i26 = 0; c3_i26 < 8; c3_i26++) {
    c3_L_p[c3_i26] = c3_b_L_p[c3_i26];
  }

  for (c3_i27 = 0; c3_i27 < 8; c3_i27++) {
    c3_R_p[c3_i27] = c3_b_R_p[c3_i27];
  }

  for (c3_i28 = 0; c3_i28 < 16; c3_i28++) {
    c3_S_PDes[c3_i28] = c3_b_S_PDes[c3_i28];
  }

  c3_robot = c3_b_robot;
  _SFD_EML_CALL(0U, chartInstance->c3_sfEvent, 45);
  c3_c_robot = c3_robot;
  for (c3_i29 = 0; c3_i29 < 16; c3_i29++) {
    c3_d_S_PDes[c3_i29] = c3_S_PDes[c3_i29];
  }

  for (c3_i30 = 0; c3_i30 < 8; c3_i30++) {
    c3_c_L_p[c3_i30] = c3_L_p[c3_i30];
  }

  for (c3_i31 = 0; c3_i31 < 8; c3_i31++) {
    c3_c_R_p[c3_i31] = c3_R_p[c3_i31];
  }

  c3_mirage(chartInstance, &c3_c_robot, c3_d_S_PDes, c3_c_L_p, c3_c_R_p, c3_dv5);
  for (c3_i32 = 0; c3_i32 < 12; c3_i32++) {
    c3_param[c3_i32] = c3_dv5[c3_i32];
  }

  _SFD_EML_CALL(0U, chartInstance->c3_sfEvent, 47);
  for (c3_i33 = 0; c3_i33 < 16; c3_i33++) {
    c3_sol[c3_i33] = 0.0;
  }

  c3_eml_switch_helper(chartInstance);
  for (c3_k = 1; c3_k < 5; c3_k++) {
    c3_b_k = c3_k;
    c3_sol[(_SFD_EML_ARRAY_BOUNDS_CHECK("", (int32_T)_SFD_INTEGER_CHECK("",
              (real_T)c3_b_k), 1, 4, 1, 0) + ((_SFD_EML_ARRAY_BOUNDS_CHECK("",
               (int32_T)_SFD_INTEGER_CHECK("", (real_T)c3_b_k), 1, 4, 2, 0) - 1)
             << 2)) - 1] = 1.0;
  }

  _SFD_EML_CALL(0U, chartInstance->c3_sfEvent, 47);
  for (c3_i34 = 0; c3_i34 < 12; c3_i34++) {
    c3_x[c3_i34] = c3_param[c3_i34];
  }

  c3_eml_switch_helper(chartInstance);
  for (c3_c_k = 1; c3_c_k < 13; c3_c_k++) {
    c3_d_k = c3_c_k;
    c3_y[_SFD_EML_ARRAY_BOUNDS_CHECK("", (int32_T)_SFD_INTEGER_CHECK("", (real_T)
      c3_d_k), 1, 12, 1, 0) - 1] = c3_x[_SFD_EML_ARRAY_BOUNDS_CHECK("", (int32_T)
      _SFD_INTEGER_CHECK("", (real_T)c3_d_k), 1, 12, 1, 0) - 1];
  }

  c3_i35 = 0;
  c3_i36 = 0;
  for (c3_i37 = 0; c3_i37 < 4; c3_i37++) {
    for (c3_i38 = 0; c3_i38 < 3; c3_i38++) {
      c3_sol[c3_i38 + c3_i35] = c3_y[c3_i38 + c3_i36];
    }

    c3_i35 += 4;
    c3_i36 += 3;
  }

  _SFD_EML_CALL(0U, chartInstance->c3_sfEvent, 49);
  for (c3_i39 = 0; c3_i39 < 16; c3_i39++) {
    c3_a[c3_i39] = c3_sol[c3_i39];
  }

  for (c3_i40 = 0; c3_i40 < 16; c3_i40++) {
    c3_b[c3_i40] = c3_world2robotDes[c3_i40];
  }

  c3_eml_scalar_eg(chartInstance);
  c3_eml_scalar_eg(chartInstance);
  for (c3_i41 = 0; c3_i41 < 16; c3_i41++) {
    c3_world2robot[c3_i41] = 0.0;
  }

  for (c3_i42 = 0; c3_i42 < 16; c3_i42++) {
    c3_b_S_PDes[c3_i42] = 0.0;
  }

  for (c3_i43 = 0; c3_i43 < 16; c3_i43++) {
    c3_b_a[c3_i43] = c3_a[c3_i43];
  }

  for (c3_i44 = 0; c3_i44 < 16; c3_i44++) {
    c3_b_b[c3_i44] = c3_b[c3_i44];
  }

  c3_b_eml_xgemm(chartInstance, c3_b_a, c3_b_b, c3_b_S_PDes);
  for (c3_i45 = 0; c3_i45 < 16; c3_i45++) {
    c3_world2robot[c3_i45] = c3_b_S_PDes[c3_i45];
  }

  _SFD_EML_CALL(0U, chartInstance->c3_sfEvent, 51);
  c3_i46 = 0;
  c3_i47 = 0;
  for (c3_i48 = 0; c3_i48 < 3; c3_i48++) {
    for (c3_i49 = 0; c3_i49 < 3; c3_i49++) {
      c3_rotMat[c3_i49 + c3_i46] = c3_world2robot[c3_i49 + c3_i47];
    }

    c3_i46 += 3;
    c3_i47 += 4;
  }

  _SFD_SYMBOL_SCOPE_PUSH_EML(0U, 6U, 6U, c3_n_debug_family_names,
    c3_b_debug_family_var_map);
  _SFD_SYMBOL_SCOPE_ADD_EML_IMPORTABLE(&c3_b_nargin, 0U, c3_b_sf_marshallOut,
    c3_b_sf_marshallIn);
  _SFD_SYMBOL_SCOPE_ADD_EML_IMPORTABLE(&c3_b_nargout, 1U, c3_b_sf_marshallOut,
    c3_b_sf_marshallIn);
  _SFD_SYMBOL_SCOPE_ADD_EML_IMPORTABLE(c3_rotMat, 2U, c3_u_sf_marshallOut,
    c3_t_sf_marshallIn);
  _SFD_SYMBOL_SCOPE_ADD_EML_IMPORTABLE(&c3_unusedU0, 3U, c3_b_sf_marshallOut,
    c3_b_sf_marshallIn);
  _SFD_SYMBOL_SCOPE_ADD_EML_IMPORTABLE(&c3_unusedU1, 4U, c3_b_sf_marshallOut,
    c3_b_sf_marshallIn);
  _SFD_SYMBOL_SCOPE_ADD_EML_IMPORTABLE(&c3_b_psi, 5U, c3_b_sf_marshallOut,
    c3_b_sf_marshallIn);
  CV_SCRIPT_FCN(11, 0);
  _SFD_SCRIPT_CALL(11U, chartInstance->c3_sfEvent, 11);
  c3_b_x = c3_mpower(chartInstance, c3_rotMat[0]) + c3_mpower(chartInstance,
    c3_rotMat[1]);
  c3_c_x = c3_b_x;
  if (c3_c_x < 0.0) {
    c3_b_eml_error(chartInstance);
  }

  c3_c_x = muDoubleScalarSqrt(c3_c_x);
  c3_unusedU1 = c3_atan2(chartInstance, -c3_rotMat[2], c3_c_x);
  _SFD_SCRIPT_CALL(11U, chartInstance->c3_sfEvent, 12);
  c3_d_x = c3_unusedU1;
  c3_e_x = c3_d_x;
  c3_e_x = muDoubleScalarCos(c3_e_x);
  c3_A = c3_rotMat[5];
  c3_B = c3_e_x;
  c3_b_y = c3_rdivide(chartInstance, c3_A, c3_B);
  c3_f_x = c3_unusedU1;
  c3_g_x = c3_f_x;
  c3_g_x = muDoubleScalarCos(c3_g_x);
  c3_b_A = c3_rotMat[8];
  c3_b_B = c3_g_x;
  c3_c_y = c3_rdivide(chartInstance, c3_b_A, c3_b_B);
  c3_unusedU0 = c3_atan2(chartInstance, c3_b_y, c3_c_y);
  _SFD_SCRIPT_CALL(11U, chartInstance->c3_sfEvent, 13);
  c3_h_x = c3_unusedU1;
  c3_i_x = c3_h_x;
  c3_i_x = muDoubleScalarCos(c3_i_x);
  c3_c_A = c3_rotMat[1];
  c3_c_B = c3_i_x;
  c3_d_y = c3_rdivide(chartInstance, c3_c_A, c3_c_B);
  c3_j_x = c3_unusedU1;
  c3_k_x = c3_j_x;
  c3_k_x = muDoubleScalarCos(c3_k_x);
  c3_d_A = c3_rotMat[0];
  c3_d_B = c3_k_x;
  c3_e_y = c3_rdivide(chartInstance, c3_d_A, c3_d_B);
  c3_b_psi = c3_atan2(chartInstance, c3_d_y, c3_e_y);
  _SFD_SCRIPT_CALL(11U, chartInstance->c3_sfEvent, -13);
  _SFD_SYMBOL_SCOPE_POP();
  c3_psi = c3_b_psi;
  _SFD_EML_CALL(0U, chartInstance->c3_sfEvent, 51);
  _SFD_EML_CALL(0U, chartInstance->c3_sfEvent, 51);
  _SFD_EML_CALL(0U, chartInstance->c3_sfEvent, 53);
  for (c3_i50 = 0; c3_i50 < 3; c3_i50++) {
    c3_b_qtilde[c3_i50] = 0.0;
  }

  _SFD_EML_CALL(0U, chartInstance->c3_sfEvent, 54);
  c3_b_qtilde[0] = -c3_world2robot[12] - c3_b_qr[0];
  _SFD_EML_CALL(0U, chartInstance->c3_sfEvent, 55);
  c3_b_qtilde[1] = -c3_world2robot[13] - c3_b_qr[1];
  _SFD_EML_CALL(0U, chartInstance->c3_sfEvent, 56);
  c3_b_qtilde[2] = -c3_psi - c3_b_qr[2];
  _SFD_EML_CALL(0U, chartInstance->c3_sfEvent, -56);
  _SFD_SYMBOL_SCOPE_POP();
  for (c3_i51 = 0; c3_i51 < 3; c3_i51++) {
    (*chartInstance->c3_qtilde)[c3_i51] = c3_b_qtilde[c3_i51];
  }

  _SFD_CC_CALL(EXIT_OUT_OF_FUNCTION_TAG, 2U, chartInstance->c3_sfEvent);
}

static void initSimStructsc3_Robot_Control_v01
  (SFc3_Robot_Control_v01InstanceStruct *chartInstance)
{
  (void)chartInstance;
}

static void c3_random(SFc3_Robot_Control_v01InstanceStruct *chartInstance,
                      real_T c3_xini, real_T c3_xend, real_T c3_x[4])
{
  uint32_T c3_debug_family_var_map[9];
  real_T c3_a;
  real_T c3_b;
  real_T c3_xu[4];
  real_T c3_n;
  real_T c3_nargin = 3.0;
  real_T c3_nargout = 1.0;
  real_T c3_dv6[4];
  int32_T c3_i52;
  int32_T c3_i53;
  real_T c3_b_a[4];
  real_T c3_b_b;
  int32_T c3_i54;
  int32_T c3_i55;
  _SFD_SYMBOL_SCOPE_PUSH_EML(0U, 9U, 9U, c3_b_debug_family_names,
    c3_debug_family_var_map);
  _SFD_SYMBOL_SCOPE_ADD_EML_IMPORTABLE(&c3_a, 0U, c3_b_sf_marshallOut,
    c3_b_sf_marshallIn);
  _SFD_SYMBOL_SCOPE_ADD_EML_IMPORTABLE(&c3_b, 1U, c3_b_sf_marshallOut,
    c3_b_sf_marshallIn);
  _SFD_SYMBOL_SCOPE_ADD_EML_IMPORTABLE(c3_xu, 2U, c3_h_sf_marshallOut,
    c3_h_sf_marshallIn);
  _SFD_SYMBOL_SCOPE_ADD_EML_IMPORTABLE(&c3_n, 3U, c3_b_sf_marshallOut,
    c3_b_sf_marshallIn);
  _SFD_SYMBOL_SCOPE_ADD_EML_IMPORTABLE(&c3_nargin, 4U, c3_b_sf_marshallOut,
    c3_b_sf_marshallIn);
  _SFD_SYMBOL_SCOPE_ADD_EML_IMPORTABLE(&c3_nargout, 5U, c3_b_sf_marshallOut,
    c3_b_sf_marshallIn);
  _SFD_SYMBOL_SCOPE_ADD_EML_IMPORTABLE(&c3_xini, 6U, c3_b_sf_marshallOut,
    c3_b_sf_marshallIn);
  _SFD_SYMBOL_SCOPE_ADD_EML_IMPORTABLE(&c3_xend, 7U, c3_b_sf_marshallOut,
    c3_b_sf_marshallIn);
  _SFD_SYMBOL_SCOPE_ADD_EML_IMPORTABLE(c3_x, 8U, c3_h_sf_marshallOut,
    c3_h_sf_marshallIn);
  c3_n = 4.0;
  CV_SCRIPT_FCN(0, 0);
  _SFD_SCRIPT_CALL(0U, chartInstance->c3_sfEvent, 20);
  c3_a = c3_xend - c3_xini;
  _SFD_SCRIPT_CALL(0U, chartInstance->c3_sfEvent, 21);
  c3_b = c3_xini;
  _SFD_SCRIPT_CALL(0U, chartInstance->c3_sfEvent, 24);
  c3_rand(chartInstance, c3_dv6);
  for (c3_i52 = 0; c3_i52 < 4; c3_i52++) {
    c3_xu[c3_i52] = c3_dv6[c3_i52];
  }

  _SFD_SCRIPT_CALL(0U, chartInstance->c3_sfEvent, 25);
  for (c3_i53 = 0; c3_i53 < 4; c3_i53++) {
    c3_b_a[c3_i53] = c3_xu[c3_i53];
  }

  c3_b_b = c3_a;
  for (c3_i54 = 0; c3_i54 < 4; c3_i54++) {
    c3_b_a[c3_i54] *= c3_b_b;
  }

  for (c3_i55 = 0; c3_i55 < 4; c3_i55++) {
    c3_x[c3_i55] = c3_b_a[c3_i55] + c3_b;
  }

  _SFD_SCRIPT_CALL(0U, chartInstance->c3_sfEvent, -25);
  _SFD_SYMBOL_SCOPE_POP();
}

static c3_sKe46gf3wfNeOrzL8yzt6nF c3_newPose
  (SFc3_Robot_Control_v01InstanceStruct *chartInstance, real_T c3_tx, real_T
   c3_ty, real_T c3_tz, real_T c3_phi, real_T c3_theta, real_T c3_psi)
{
  c3_sKe46gf3wfNeOrzL8yzt6nF c3_pose;
  uint32_T c3_debug_family_var_map[9];
  real_T c3_nargin = 6.0;
  real_T c3_nargout = 1.0;
  _SFD_SYMBOL_SCOPE_PUSH_EML(0U, 9U, 9U, c3_c_debug_family_names,
    c3_debug_family_var_map);
  _SFD_SYMBOL_SCOPE_ADD_EML_IMPORTABLE(&c3_nargin, 0U, c3_b_sf_marshallOut,
    c3_b_sf_marshallIn);
  _SFD_SYMBOL_SCOPE_ADD_EML_IMPORTABLE(&c3_nargout, 1U, c3_b_sf_marshallOut,
    c3_b_sf_marshallIn);
  _SFD_SYMBOL_SCOPE_ADD_EML_IMPORTABLE(&c3_tx, 2U, c3_b_sf_marshallOut,
    c3_b_sf_marshallIn);
  _SFD_SYMBOL_SCOPE_ADD_EML_IMPORTABLE(&c3_ty, 3U, c3_b_sf_marshallOut,
    c3_b_sf_marshallIn);
  _SFD_SYMBOL_SCOPE_ADD_EML_IMPORTABLE(&c3_tz, 4U, c3_b_sf_marshallOut,
    c3_b_sf_marshallIn);
  _SFD_SYMBOL_SCOPE_ADD_EML_IMPORTABLE(&c3_phi, 5U, c3_b_sf_marshallOut,
    c3_b_sf_marshallIn);
  _SFD_SYMBOL_SCOPE_ADD_EML_IMPORTABLE(&c3_theta, 6U, c3_b_sf_marshallOut,
    c3_b_sf_marshallIn);
  _SFD_SYMBOL_SCOPE_ADD_EML_IMPORTABLE(&c3_psi, 7U, c3_b_sf_marshallOut,
    c3_b_sf_marshallIn);
  _SFD_SYMBOL_SCOPE_ADD_EML_IMPORTABLE(&c3_pose, 8U, c3_g_sf_marshallOut,
    c3_g_sf_marshallIn);
  CV_SCRIPT_FCN(1, 0);
  _SFD_SCRIPT_CALL(1U, chartInstance->c3_sfEvent, 3);
  c3_pose.tx = c3_tx;
  _SFD_SCRIPT_CALL(1U, chartInstance->c3_sfEvent, 4);
  c3_pose.ty = c3_ty;
  _SFD_SCRIPT_CALL(1U, chartInstance->c3_sfEvent, 5);
  c3_pose.tz = c3_tz;
  _SFD_SCRIPT_CALL(1U, chartInstance->c3_sfEvent, 6);
  c3_pose.phi = c3_phi;
  _SFD_SCRIPT_CALL(1U, chartInstance->c3_sfEvent, 7);
  c3_pose.theta = c3_theta;
  _SFD_SCRIPT_CALL(1U, chartInstance->c3_sfEvent, 8);
  c3_pose.psi = c3_psi;
  _SFD_SCRIPT_CALL(1U, chartInstance->c3_sfEvent, -8);
  _SFD_SYMBOL_SCOPE_POP();
  return c3_pose;
}

static void c3_transformationMatrix(SFc3_Robot_Control_v01InstanceStruct
  *chartInstance, c3_sKe46gf3wfNeOrzL8yzt6nF c3_varargin_1, real_T c3_M[16])
{
  uint32_T c3_debug_family_var_map[12];
  void *c3_varargin;
  c3_sKe46gf3wfNeOrzL8yzt6nF c3_pose;
  real_T c3_tx;
  real_T c3_ty;
  real_T c3_tz;
  real_T c3_phi;
  real_T c3_theta;
  real_T c3_psi;
  c3_smuwpGFfhzM6MfA0WGyOk6E c3_DesRobotPose;
  real_T c3_nargin = 1.0;
  real_T c3_nargout = 1.0;
  real_T c3_dv7[16];
  int32_T c3_i56;
  _SFD_SYMBOL_SCOPE_PUSH_EML(0U, 12U, 12U, c3_e_debug_family_names,
    c3_debug_family_var_map);
  _SFD_SYMBOL_SCOPE_ADD_EML(&c3_varargin, 0U, c3_k_sf_marshallOut);
  _SFD_SYMBOL_SCOPE_ADD_EML_IMPORTABLE(&c3_pose, 1U, c3_g_sf_marshallOut,
    c3_g_sf_marshallIn);
  _SFD_SYMBOL_SCOPE_ADD_EML_IMPORTABLE(&c3_tx, 2U, c3_b_sf_marshallOut,
    c3_b_sf_marshallIn);
  _SFD_SYMBOL_SCOPE_ADD_EML_IMPORTABLE(&c3_ty, 3U, c3_b_sf_marshallOut,
    c3_b_sf_marshallIn);
  _SFD_SYMBOL_SCOPE_ADD_EML_IMPORTABLE(&c3_tz, 4U, c3_b_sf_marshallOut,
    c3_b_sf_marshallIn);
  _SFD_SYMBOL_SCOPE_ADD_EML_IMPORTABLE(&c3_phi, 5U, c3_b_sf_marshallOut,
    c3_b_sf_marshallIn);
  _SFD_SYMBOL_SCOPE_ADD_EML_IMPORTABLE(&c3_theta, 6U, c3_b_sf_marshallOut,
    c3_b_sf_marshallIn);
  _SFD_SYMBOL_SCOPE_ADD_EML_IMPORTABLE(&c3_psi, 7U, c3_b_sf_marshallOut,
    c3_b_sf_marshallIn);
  _SFD_SYMBOL_SCOPE_ADD_EML_IMPORTABLE(&c3_DesRobotPose, 8U, c3_j_sf_marshallOut,
    c3_j_sf_marshallIn);
  _SFD_SYMBOL_SCOPE_ADD_EML_IMPORTABLE(&c3_nargin, 9U, c3_b_sf_marshallOut,
    c3_b_sf_marshallIn);
  _SFD_SYMBOL_SCOPE_ADD_EML_IMPORTABLE(&c3_nargout, 10U, c3_b_sf_marshallOut,
    c3_b_sf_marshallIn);
  _SFD_SYMBOL_SCOPE_ADD_EML_IMPORTABLE(c3_M, 11U, c3_c_sf_marshallOut,
    c3_c_sf_marshallIn);
  c3_varargin = (void *)&c3_varargin_1;
  CV_SCRIPT_FCN(2, 0);
  _SFD_SCRIPT_CALL(2U, chartInstance->c3_sfEvent, 3);
  CV_SCRIPT_IF(2, 0, CV_RELATIONAL_EVAL(14U, 2U, 0, 1.0, 1.0, -1, 0U, 1));
  _SFD_SCRIPT_CALL(2U, chartInstance->c3_sfEvent, 4);
  c3_pose = c3_varargin_1;
  _SFD_SCRIPT_CALL(2U, chartInstance->c3_sfEvent, 5);
  c3_tx = c3_pose.tx;
  _SFD_SCRIPT_CALL(2U, chartInstance->c3_sfEvent, 6);
  c3_ty = c3_pose.ty;
  _SFD_SCRIPT_CALL(2U, chartInstance->c3_sfEvent, 7);
  c3_tz = c3_pose.tz;
  _SFD_SCRIPT_CALL(2U, chartInstance->c3_sfEvent, 8);
  c3_phi = c3_pose.phi;
  _SFD_SCRIPT_CALL(2U, chartInstance->c3_sfEvent, 9);
  c3_theta = c3_pose.theta;
  _SFD_SCRIPT_CALL(2U, chartInstance->c3_sfEvent, 10);
  c3_psi = c3_pose.psi;
  _SFD_SCRIPT_CALL(2U, chartInstance->c3_sfEvent, 23);
  c3_DesRobotPose.x = c3_tx;
  _SFD_SCRIPT_CALL(2U, chartInstance->c3_sfEvent, 24);
  c3_DesRobotPose.y = c3_ty;
  _SFD_SCRIPT_CALL(2U, chartInstance->c3_sfEvent, 25);
  c3_DesRobotPose.z = c3_tz;
  _SFD_SCRIPT_CALL(2U, chartInstance->c3_sfEvent, 26);
  c3_DesRobotPose.phi = c3_phi;
  _SFD_SCRIPT_CALL(2U, chartInstance->c3_sfEvent, 27);
  c3_DesRobotPose.theta = c3_theta;
  _SFD_SCRIPT_CALL(2U, chartInstance->c3_sfEvent, 28);
  c3_DesRobotPose.psi = c3_psi;
  _SFD_SCRIPT_CALL(2U, chartInstance->c3_sfEvent, 31);
  c3_EulerTrans(chartInstance, c3_DesRobotPose, c3_dv7);
  for (c3_i56 = 0; c3_i56 < 16; c3_i56++) {
    c3_M[c3_i56] = c3_dv7[c3_i56];
  }

  _SFD_SCRIPT_CALL(2U, chartInstance->c3_sfEvent, -31);
  _SFD_SYMBOL_SCOPE_POP();
}

static void c3_EulerTrans(SFc3_Robot_Control_v01InstanceStruct *chartInstance,
  c3_smuwpGFfhzM6MfA0WGyOk6E c3_Pose, real_T c3_y_E_x[16])
{
  uint32_T c3_debug_family_var_map[14];
  real_T c3_Dx;
  real_T c3_Dy;
  real_T c3_Dz;
  real_T c3_phi;
  real_T c3_theta;
  real_T c3_psi;
  real_T c3_Rz[16];
  real_T c3_Ry[16];
  real_T c3_Rx[16];
  real_T c3_Tr[16];
  real_T c3_nargin = 1.0;
  real_T c3_nargout = 1.0;
  real_T c3_x;
  real_T c3_b_x;
  real_T c3_c_x;
  real_T c3_d_x;
  real_T c3_e_x;
  real_T c3_f_x;
  real_T c3_g_x;
  real_T c3_h_x;
  int32_T c3_i57;
  int32_T c3_i58;
  static real_T c3_dv8[4] = { 0.0, 0.0, 1.0, 0.0 };

  int32_T c3_i59;
  int32_T c3_i60;
  static real_T c3_dv9[4] = { 0.0, 0.0, 0.0, 1.0 };

  real_T c3_i_x;
  real_T c3_j_x;
  real_T c3_k_x;
  real_T c3_l_x;
  real_T c3_m_x;
  real_T c3_n_x;
  real_T c3_o_x;
  real_T c3_p_x;
  int32_T c3_i61;
  int32_T c3_i62;
  static real_T c3_dv10[4] = { 0.0, 1.0, 0.0, 0.0 };

  int32_T c3_i63;
  int32_T c3_i64;
  real_T c3_q_x;
  real_T c3_r_x;
  real_T c3_s_x;
  real_T c3_t_x;
  real_T c3_u_x;
  real_T c3_v_x;
  real_T c3_w_x;
  real_T c3_x_x;
  int32_T c3_i65;
  int32_T c3_i66;
  static real_T c3_dv11[4] = { 1.0, 0.0, 0.0, 0.0 };

  int32_T c3_i67;
  int32_T c3_i68;
  int32_T c3_i69;
  int32_T c3_i70;
  int32_T c3_i71;
  real_T c3_a[16];
  int32_T c3_i72;
  real_T c3_b[16];
  int32_T c3_i73;
  real_T c3_y[16];
  int32_T c3_i74;
  real_T c3_b_a[16];
  int32_T c3_i75;
  real_T c3_b_b[16];
  int32_T c3_i76;
  int32_T c3_i77;
  real_T c3_b_y[16];
  int32_T c3_i78;
  real_T c3_c_y[16];
  int32_T c3_i79;
  real_T c3_c_b[16];
  int32_T c3_i80;
  int32_T c3_i81;
  int32_T c3_i82;
  int32_T c3_i83;
  real_T c3_d_y[16];
  int32_T c3_i84;
  real_T c3_d_b[16];
  int32_T c3_i85;
  _SFD_SYMBOL_SCOPE_PUSH_EML(0U, 14U, 14U, c3_d_debug_family_names,
    c3_debug_family_var_map);
  _SFD_SYMBOL_SCOPE_ADD_EML_IMPORTABLE(&c3_Dx, 0U, c3_b_sf_marshallOut,
    c3_b_sf_marshallIn);
  _SFD_SYMBOL_SCOPE_ADD_EML_IMPORTABLE(&c3_Dy, 1U, c3_b_sf_marshallOut,
    c3_b_sf_marshallIn);
  _SFD_SYMBOL_SCOPE_ADD_EML_IMPORTABLE(&c3_Dz, 2U, c3_b_sf_marshallOut,
    c3_b_sf_marshallIn);
  _SFD_SYMBOL_SCOPE_ADD_EML_IMPORTABLE(&c3_phi, 3U, c3_b_sf_marshallOut,
    c3_b_sf_marshallIn);
  _SFD_SYMBOL_SCOPE_ADD_EML_IMPORTABLE(&c3_theta, 4U, c3_b_sf_marshallOut,
    c3_b_sf_marshallIn);
  _SFD_SYMBOL_SCOPE_ADD_EML_IMPORTABLE(&c3_psi, 5U, c3_b_sf_marshallOut,
    c3_b_sf_marshallIn);
  _SFD_SYMBOL_SCOPE_ADD_EML_IMPORTABLE(c3_Rz, 6U, c3_c_sf_marshallOut,
    c3_c_sf_marshallIn);
  _SFD_SYMBOL_SCOPE_ADD_EML_IMPORTABLE(c3_Ry, 7U, c3_c_sf_marshallOut,
    c3_c_sf_marshallIn);
  _SFD_SYMBOL_SCOPE_ADD_EML_IMPORTABLE(c3_Rx, 8U, c3_c_sf_marshallOut,
    c3_c_sf_marshallIn);
  _SFD_SYMBOL_SCOPE_ADD_EML_IMPORTABLE(c3_Tr, 9U, c3_c_sf_marshallOut,
    c3_c_sf_marshallIn);
  _SFD_SYMBOL_SCOPE_ADD_EML_IMPORTABLE(&c3_nargin, 10U, c3_b_sf_marshallOut,
    c3_b_sf_marshallIn);
  _SFD_SYMBOL_SCOPE_ADD_EML_IMPORTABLE(&c3_nargout, 11U, c3_b_sf_marshallOut,
    c3_b_sf_marshallIn);
  _SFD_SYMBOL_SCOPE_ADD_EML_IMPORTABLE(&c3_Pose, 12U, c3_j_sf_marshallOut,
    c3_j_sf_marshallIn);
  _SFD_SYMBOL_SCOPE_ADD_EML_IMPORTABLE(c3_y_E_x, 13U, c3_c_sf_marshallOut,
    c3_c_sf_marshallIn);
  CV_SCRIPT_FCN(3, 0);
  _SFD_SCRIPT_CALL(3U, chartInstance->c3_sfEvent, 4);
  c3_Dx = c3_Pose.x;
  _SFD_SCRIPT_CALL(3U, chartInstance->c3_sfEvent, 5);
  c3_Dy = c3_Pose.y;
  _SFD_SCRIPT_CALL(3U, chartInstance->c3_sfEvent, 6);
  c3_Dz = c3_Pose.z;
  _SFD_SCRIPT_CALL(3U, chartInstance->c3_sfEvent, 7);
  c3_phi = c3_Pose.phi;
  _SFD_SCRIPT_CALL(3U, chartInstance->c3_sfEvent, 8);
  c3_theta = c3_Pose.theta;
  _SFD_SCRIPT_CALL(3U, chartInstance->c3_sfEvent, 9);
  c3_psi = c3_Pose.psi;
  _SFD_SCRIPT_CALL(3U, chartInstance->c3_sfEvent, 11);
  c3_x = c3_psi;
  c3_b_x = c3_x;
  c3_b_x = muDoubleScalarCos(c3_b_x);
  c3_c_x = c3_psi;
  c3_d_x = c3_c_x;
  c3_d_x = muDoubleScalarSin(c3_d_x);
  c3_e_x = c3_psi;
  c3_f_x = c3_e_x;
  c3_f_x = muDoubleScalarSin(c3_f_x);
  c3_g_x = c3_psi;
  c3_h_x = c3_g_x;
  c3_h_x = muDoubleScalarCos(c3_h_x);
  c3_Rz[0] = c3_b_x;
  c3_Rz[4] = -c3_d_x;
  c3_Rz[8] = 0.0;
  c3_Rz[12] = 0.0;
  c3_Rz[1] = c3_f_x;
  c3_Rz[5] = c3_h_x;
  c3_Rz[9] = 0.0;
  c3_Rz[13] = 0.0;
  c3_i57 = 0;
  for (c3_i58 = 0; c3_i58 < 4; c3_i58++) {
    c3_Rz[c3_i57 + 2] = c3_dv8[c3_i58];
    c3_i57 += 4;
  }

  c3_i59 = 0;
  for (c3_i60 = 0; c3_i60 < 4; c3_i60++) {
    c3_Rz[c3_i59 + 3] = c3_dv9[c3_i60];
    c3_i59 += 4;
  }

  _SFD_SCRIPT_CALL(3U, chartInstance->c3_sfEvent, 12);
  c3_i_x = c3_theta;
  c3_j_x = c3_i_x;
  c3_j_x = muDoubleScalarCos(c3_j_x);
  c3_k_x = c3_theta;
  c3_l_x = c3_k_x;
  c3_l_x = muDoubleScalarSin(c3_l_x);
  c3_m_x = c3_theta;
  c3_n_x = c3_m_x;
  c3_n_x = muDoubleScalarSin(c3_n_x);
  c3_o_x = c3_theta;
  c3_p_x = c3_o_x;
  c3_p_x = muDoubleScalarCos(c3_p_x);
  c3_Ry[0] = c3_j_x;
  c3_Ry[4] = 0.0;
  c3_Ry[8] = c3_l_x;
  c3_Ry[12] = 0.0;
  c3_i61 = 0;
  for (c3_i62 = 0; c3_i62 < 4; c3_i62++) {
    c3_Ry[c3_i61 + 1] = c3_dv10[c3_i62];
    c3_i61 += 4;
  }

  c3_Ry[2] = -c3_n_x;
  c3_Ry[6] = 0.0;
  c3_Ry[10] = c3_p_x;
  c3_Ry[14] = 0.0;
  c3_i63 = 0;
  for (c3_i64 = 0; c3_i64 < 4; c3_i64++) {
    c3_Ry[c3_i63 + 3] = c3_dv9[c3_i64];
    c3_i63 += 4;
  }

  _SFD_SCRIPT_CALL(3U, chartInstance->c3_sfEvent, 13);
  c3_q_x = c3_phi;
  c3_r_x = c3_q_x;
  c3_r_x = muDoubleScalarCos(c3_r_x);
  c3_s_x = c3_phi;
  c3_t_x = c3_s_x;
  c3_t_x = muDoubleScalarSin(c3_t_x);
  c3_u_x = c3_phi;
  c3_v_x = c3_u_x;
  c3_v_x = muDoubleScalarSin(c3_v_x);
  c3_w_x = c3_phi;
  c3_x_x = c3_w_x;
  c3_x_x = muDoubleScalarCos(c3_x_x);
  c3_i65 = 0;
  for (c3_i66 = 0; c3_i66 < 4; c3_i66++) {
    c3_Rx[c3_i65] = c3_dv11[c3_i66];
    c3_i65 += 4;
  }

  c3_Rx[1] = 0.0;
  c3_Rx[5] = c3_r_x;
  c3_Rx[9] = -c3_t_x;
  c3_Rx[13] = 0.0;
  c3_Rx[2] = 0.0;
  c3_Rx[6] = c3_v_x;
  c3_Rx[10] = c3_x_x;
  c3_Rx[14] = 0.0;
  c3_i67 = 0;
  for (c3_i68 = 0; c3_i68 < 4; c3_i68++) {
    c3_Rx[c3_i67 + 3] = c3_dv9[c3_i68];
    c3_i67 += 4;
  }

  _SFD_SCRIPT_CALL(3U, chartInstance->c3_sfEvent, 14);
  c3_Tr[0] = 1.0;
  c3_Tr[4] = 0.0;
  c3_Tr[8] = 0.0;
  c3_Tr[12] = c3_Dx;
  c3_Tr[1] = 0.0;
  c3_Tr[5] = 1.0;
  c3_Tr[9] = 0.0;
  c3_Tr[13] = c3_Dy;
  c3_Tr[2] = 0.0;
  c3_Tr[6] = 0.0;
  c3_Tr[10] = 1.0;
  c3_Tr[14] = c3_Dz;
  c3_i69 = 0;
  for (c3_i70 = 0; c3_i70 < 4; c3_i70++) {
    c3_Tr[c3_i69 + 3] = c3_dv9[c3_i70];
    c3_i69 += 4;
  }

  _SFD_SCRIPT_CALL(3U, chartInstance->c3_sfEvent, 16);
  for (c3_i71 = 0; c3_i71 < 16; c3_i71++) {
    c3_a[c3_i71] = c3_Rx[c3_i71];
  }

  for (c3_i72 = 0; c3_i72 < 16; c3_i72++) {
    c3_b[c3_i72] = c3_Ry[c3_i72];
  }

  c3_eml_scalar_eg(chartInstance);
  c3_eml_scalar_eg(chartInstance);
  for (c3_i73 = 0; c3_i73 < 16; c3_i73++) {
    c3_y[c3_i73] = 0.0;
  }

  for (c3_i74 = 0; c3_i74 < 16; c3_i74++) {
    c3_b_a[c3_i74] = c3_a[c3_i74];
  }

  for (c3_i75 = 0; c3_i75 < 16; c3_i75++) {
    c3_b_b[c3_i75] = c3_b[c3_i75];
  }

  c3_b_eml_xgemm(chartInstance, c3_b_a, c3_b_b, c3_y);
  for (c3_i76 = 0; c3_i76 < 16; c3_i76++) {
    c3_b[c3_i76] = c3_Rz[c3_i76];
  }

  c3_eml_scalar_eg(chartInstance);
  c3_eml_scalar_eg(chartInstance);
  for (c3_i77 = 0; c3_i77 < 16; c3_i77++) {
    c3_b_y[c3_i77] = 0.0;
  }

  for (c3_i78 = 0; c3_i78 < 16; c3_i78++) {
    c3_c_y[c3_i78] = c3_y[c3_i78];
  }

  for (c3_i79 = 0; c3_i79 < 16; c3_i79++) {
    c3_c_b[c3_i79] = c3_b[c3_i79];
  }

  c3_b_eml_xgemm(chartInstance, c3_c_y, c3_c_b, c3_b_y);
  for (c3_i80 = 0; c3_i80 < 16; c3_i80++) {
    c3_b[c3_i80] = c3_Tr[c3_i80];
  }

  c3_eml_scalar_eg(chartInstance);
  c3_eml_scalar_eg(chartInstance);
  for (c3_i81 = 0; c3_i81 < 16; c3_i81++) {
    c3_y_E_x[c3_i81] = 0.0;
  }

  for (c3_i82 = 0; c3_i82 < 16; c3_i82++) {
    c3_a[c3_i82] = 0.0;
  }

  for (c3_i83 = 0; c3_i83 < 16; c3_i83++) {
    c3_d_y[c3_i83] = c3_b_y[c3_i83];
  }

  for (c3_i84 = 0; c3_i84 < 16; c3_i84++) {
    c3_d_b[c3_i84] = c3_b[c3_i84];
  }

  c3_b_eml_xgemm(chartInstance, c3_d_y, c3_d_b, c3_a);
  for (c3_i85 = 0; c3_i85 < 16; c3_i85++) {
    c3_y_E_x[c3_i85] = c3_a[c3_i85];
  }

  _SFD_SCRIPT_CALL(3U, chartInstance->c3_sfEvent, -16);
  _SFD_SYMBOL_SCOPE_POP();
}

static void c3_newCamera(SFc3_Robot_Control_v01InstanceStruct *chartInstance,
  c3_sKe46gf3wfNeOrzL8yzt6nF c3_pose, real_T c3_fx, real_T c3_fy, real_T c3_cx,
  real_T c3_cy, real_T c3_s, real_T c3_d, c3_s6d9wPYgmS3cwGRsMJd1GAE *c3_camera)
{
  uint32_T c3_debug_family_var_map[10];
  real_T c3_nargin = 7.0;
  real_T c3_nargout = 1.0;
  _SFD_SYMBOL_SCOPE_PUSH_EML(0U, 10U, 10U, c3_f_debug_family_names,
    c3_debug_family_var_map);
  _SFD_SYMBOL_SCOPE_ADD_EML_IMPORTABLE(&c3_nargin, 0U, c3_b_sf_marshallOut,
    c3_b_sf_marshallIn);
  _SFD_SYMBOL_SCOPE_ADD_EML_IMPORTABLE(&c3_nargout, 1U, c3_b_sf_marshallOut,
    c3_b_sf_marshallIn);
  _SFD_SYMBOL_SCOPE_ADD_EML_IMPORTABLE(&c3_pose, 2U, c3_g_sf_marshallOut,
    c3_g_sf_marshallIn);
  _SFD_SYMBOL_SCOPE_ADD_EML_IMPORTABLE(&c3_fx, 3U, c3_b_sf_marshallOut,
    c3_b_sf_marshallIn);
  _SFD_SYMBOL_SCOPE_ADD_EML_IMPORTABLE(&c3_fy, 4U, c3_b_sf_marshallOut,
    c3_b_sf_marshallIn);
  _SFD_SYMBOL_SCOPE_ADD_EML_IMPORTABLE(&c3_cx, 5U, c3_b_sf_marshallOut,
    c3_b_sf_marshallIn);
  _SFD_SYMBOL_SCOPE_ADD_EML_IMPORTABLE(&c3_cy, 6U, c3_b_sf_marshallOut,
    c3_b_sf_marshallIn);
  _SFD_SYMBOL_SCOPE_ADD_EML_IMPORTABLE(&c3_s, 7U, c3_b_sf_marshallOut,
    c3_b_sf_marshallIn);
  _SFD_SYMBOL_SCOPE_ADD_EML_IMPORTABLE(&c3_d, 8U, c3_b_sf_marshallOut,
    c3_b_sf_marshallIn);
  _SFD_SYMBOL_SCOPE_ADD_EML_IMPORTABLE(c3_camera, 9U, c3_l_sf_marshallOut,
    c3_k_sf_marshallIn);
  CV_SCRIPT_FCN(4, 0);
  _SFD_SCRIPT_CALL(4U, chartInstance->c3_sfEvent, 4);
  c3_camera->pose = c3_pose;
  _SFD_SCRIPT_CALL(4U, chartInstance->c3_sfEvent, 7);
  c3_camera->fx = c3_fx;
  _SFD_SCRIPT_CALL(4U, chartInstance->c3_sfEvent, 8);
  c3_camera->fy = c3_fy;
  _SFD_SCRIPT_CALL(4U, chartInstance->c3_sfEvent, 11);
  c3_camera->cx = c3_cx;
  _SFD_SCRIPT_CALL(4U, chartInstance->c3_sfEvent, 12);
  c3_camera->cy = c3_cy;
  _SFD_SCRIPT_CALL(4U, chartInstance->c3_sfEvent, 15);
  c3_camera->s = c3_s;
  _SFD_SCRIPT_CALL(4U, chartInstance->c3_sfEvent, 18);
  c3_camera->d = c3_d;
  _SFD_SCRIPT_CALL(4U, chartInstance->c3_sfEvent, -18);
  _SFD_SYMBOL_SCOPE_POP();
}

static void c3_transformImageSpace(SFc3_Robot_Control_v01InstanceStruct
  *chartInstance, real_T c3_S_P[16], c3_s6d9wPYgmS3cwGRsMJd1GAE *c3_camera,
  c3_sZiCA83jGK4zdmds7wvlXUF *c3_retVal)
{
  uint32_T c3_debug_family_var_map[10];
  real_T c3_robot2cam[16];
  real_T c3_K[16];
  real_T c3_M[16];
  real_T c3_C_P[16];
  real_T c3_p[8];
  real_T c3_nargin = 2.0;
  real_T c3_nargout = 1.0;
  real_T c3_a[16];
  int32_T c3_i86;
  real_T c3_b_a[16];
  real_T c3_dv12[16];
  int32_T c3_i87;
  int32_T c3_i88;
  int32_T c3_i89;
  static int32_T c3_iv0[4] = { 1, 2, 0, 3 };

  real_T c3_b_robot2cam[16];
  int32_T c3_i90;
  int32_T c3_i91;
  int32_T c3_i92;
  int32_T c3_i93;
  int32_T c3_i94;
  real_T c3_c_a[4];
  int32_T c3_i95;
  int32_T c3_i96;
  int32_T c3_i97;
  c3_s6d9wPYgmS3cwGRsMJd1GAE c3_cam;
  uint32_T c3_b_debug_family_var_map[4];
  real_T c3_b_nargin = 1.0;
  real_T c3_b_nargout = 1.0;
  int32_T c3_i98;
  int32_T c3_i99;
  static real_T c3_dv13[4] = { 0.0, 0.0, 1.0, 0.0 };

  int32_T c3_i100;
  int32_T c3_i101;
  static real_T c3_dv14[4] = { 0.0, 0.0, 0.0, 1.0 };

  int32_T c3_i102;
  int32_T c3_i103;
  real_T c3_b[16];
  int32_T c3_i104;
  int32_T c3_i105;
  real_T c3_dv15[16];
  int32_T c3_i106;
  real_T c3_d_a[16];
  int32_T c3_i107;
  real_T c3_b_b[16];
  int32_T c3_i108;
  int32_T c3_i109;
  int32_T c3_i110;
  int32_T c3_i111;
  int32_T c3_i112;
  int32_T c3_i113;
  real_T c3_e_a[16];
  int32_T c3_i114;
  real_T c3_c_b[16];
  int32_T c3_i115;
  int32_T c3_i116;
  int32_T c3_i117;
  int32_T c3_i118;
  real_T c3_b_C_P[4];
  int32_T c3_i119;
  int32_T c3_i120;
  real_T c3_c_C_P[4];
  real_T c3_dv16[4];
  int32_T c3_i121;
  int32_T c3_i122;
  int32_T c3_i123;
  int32_T c3_i124;
  real_T c3_d_C_P[4];
  int32_T c3_i125;
  int32_T c3_i126;
  real_T c3_e_C_P[4];
  real_T c3_dv17[4];
  int32_T c3_i127;
  int32_T c3_i128;
  int32_T c3_i129;
  int32_T c3_i130;
  int32_T c3_i131;
  _SFD_SYMBOL_SCOPE_PUSH_EML(0U, 10U, 10U, c3_i_debug_family_names,
    c3_debug_family_var_map);
  _SFD_SYMBOL_SCOPE_ADD_EML_IMPORTABLE(c3_robot2cam, 0U, c3_c_sf_marshallOut,
    c3_c_sf_marshallIn);
  _SFD_SYMBOL_SCOPE_ADD_EML_IMPORTABLE(c3_K, 1U, c3_c_sf_marshallOut,
    c3_c_sf_marshallIn);
  _SFD_SYMBOL_SCOPE_ADD_EML_IMPORTABLE(c3_M, 2U, c3_c_sf_marshallOut,
    c3_c_sf_marshallIn);
  _SFD_SYMBOL_SCOPE_ADD_EML_IMPORTABLE(c3_C_P, 3U, c3_c_sf_marshallOut,
    c3_c_sf_marshallIn);
  _SFD_SYMBOL_SCOPE_ADD_EML_IMPORTABLE(c3_p, 4U, c3_f_sf_marshallOut,
    c3_f_sf_marshallIn);
  _SFD_SYMBOL_SCOPE_ADD_EML_IMPORTABLE(&c3_nargin, 5U, c3_b_sf_marshallOut,
    c3_b_sf_marshallIn);
  _SFD_SYMBOL_SCOPE_ADD_EML_IMPORTABLE(&c3_nargout, 6U, c3_b_sf_marshallOut,
    c3_b_sf_marshallIn);
  _SFD_SYMBOL_SCOPE_ADD_EML_IMPORTABLE(c3_S_P, 7U, c3_c_sf_marshallOut,
    c3_c_sf_marshallIn);
  _SFD_SYMBOL_SCOPE_ADD_EML_IMPORTABLE(c3_camera, 8U, c3_l_sf_marshallOut,
    c3_k_sf_marshallIn);
  _SFD_SYMBOL_SCOPE_ADD_EML_IMPORTABLE(c3_retVal, 9U, c3_m_sf_marshallOut,
    c3_l_sf_marshallIn);
  CV_SCRIPT_FCN(7, 0);
  _SFD_SCRIPT_CALL(7U, chartInstance->c3_sfEvent, 17);
  c3_transformationMatrix(chartInstance, c3_camera->pose, c3_a);
  for (c3_i86 = 0; c3_i86 < 16; c3_i86++) {
    c3_b_a[c3_i86] = c3_a[c3_i86];
  }

  c3_inv(chartInstance, c3_b_a, c3_dv12);
  for (c3_i87 = 0; c3_i87 < 16; c3_i87++) {
    c3_robot2cam[c3_i87] = c3_dv12[c3_i87];
  }

  _SFD_SCRIPT_CALL(7U, chartInstance->c3_sfEvent, 20);
  for (c3_i88 = 0; c3_i88 < 4; c3_i88++) {
    for (c3_i89 = 0; c3_i89 < 4; c3_i89++) {
      c3_b_robot2cam[c3_i89 + (c3_i88 << 2)] = c3_robot2cam[c3_iv0[c3_i89] +
        (c3_i88 << 2)];
    }
  }

  c3_i90 = 0;
  for (c3_i91 = 0; c3_i91 < 4; c3_i91++) {
    for (c3_i92 = 0; c3_i92 < 4; c3_i92++) {
      c3_robot2cam[c3_i92 + c3_i90] = c3_b_robot2cam[c3_i92 + c3_i90];
    }

    c3_i90 += 4;
  }

  _SFD_SCRIPT_CALL(7U, chartInstance->c3_sfEvent, 21);
  c3_i93 = 0;
  for (c3_i94 = 0; c3_i94 < 4; c3_i94++) {
    c3_c_a[c3_i94] = c3_robot2cam[c3_i93];
    c3_i93 += 4;
  }

  for (c3_i95 = 0; c3_i95 < 4; c3_i95++) {
    c3_c_a[c3_i95] = -c3_c_a[c3_i95];
  }

  c3_i96 = 0;
  for (c3_i97 = 0; c3_i97 < 4; c3_i97++) {
    c3_robot2cam[c3_i96] = c3_c_a[c3_i97];
    c3_i96 += 4;
  }

  _SFD_SCRIPT_CALL(7U, chartInstance->c3_sfEvent, 29);
  c3_cam = *c3_camera;
  _SFD_SYMBOL_SCOPE_PUSH_EML(0U, 4U, 4U, c3_h_debug_family_names,
    c3_b_debug_family_var_map);
  _SFD_SYMBOL_SCOPE_ADD_EML_IMPORTABLE(&c3_b_nargin, 0U, c3_b_sf_marshallOut,
    c3_b_sf_marshallIn);
  _SFD_SYMBOL_SCOPE_ADD_EML_IMPORTABLE(&c3_b_nargout, 1U, c3_b_sf_marshallOut,
    c3_b_sf_marshallIn);
  _SFD_SYMBOL_SCOPE_ADD_EML_IMPORTABLE(&c3_cam, 2U, c3_l_sf_marshallOut,
    c3_k_sf_marshallIn);
  _SFD_SYMBOL_SCOPE_ADD_EML_IMPORTABLE(c3_K, 3U, c3_c_sf_marshallOut,
    c3_c_sf_marshallIn);
  CV_SCRIPT_FCN(8, 0);
  _SFD_SCRIPT_CALL(8U, chartInstance->c3_sfEvent, 4);
  c3_K[0] = c3_cam.fx;
  c3_K[4] = 0.0;
  c3_K[8] = c3_cam.cx;
  c3_K[12] = 0.0;
  c3_K[1] = 0.0;
  c3_K[5] = c3_cam.fy;
  c3_K[9] = c3_cam.cy;
  c3_K[13] = 0.0;
  c3_i98 = 0;
  for (c3_i99 = 0; c3_i99 < 4; c3_i99++) {
    c3_K[c3_i98 + 2] = c3_dv13[c3_i99];
    c3_i98 += 4;
  }

  c3_i100 = 0;
  for (c3_i101 = 0; c3_i101 < 4; c3_i101++) {
    c3_K[c3_i100 + 3] = c3_dv14[c3_i101];
    c3_i100 += 4;
  }

  _SFD_SCRIPT_CALL(8U, chartInstance->c3_sfEvent, -4);
  _SFD_SYMBOL_SCOPE_POP();
  _SFD_SCRIPT_CALL(7U, chartInstance->c3_sfEvent, 30);
  for (c3_i102 = 0; c3_i102 < 16; c3_i102++) {
    c3_a[c3_i102] = c3_K[c3_i102];
  }

  for (c3_i103 = 0; c3_i103 < 16; c3_i103++) {
    c3_b[c3_i103] = c3_robot2cam[c3_i103];
  }

  c3_eml_scalar_eg(chartInstance);
  c3_eml_scalar_eg(chartInstance);
  for (c3_i104 = 0; c3_i104 < 16; c3_i104++) {
    c3_M[c3_i104] = 0.0;
  }

  for (c3_i105 = 0; c3_i105 < 16; c3_i105++) {
    c3_dv15[c3_i105] = 0.0;
  }

  for (c3_i106 = 0; c3_i106 < 16; c3_i106++) {
    c3_d_a[c3_i106] = c3_a[c3_i106];
  }

  for (c3_i107 = 0; c3_i107 < 16; c3_i107++) {
    c3_b_b[c3_i107] = c3_b[c3_i107];
  }

  c3_b_eml_xgemm(chartInstance, c3_d_a, c3_b_b, c3_dv15);
  for (c3_i108 = 0; c3_i108 < 16; c3_i108++) {
    c3_M[c3_i108] = c3_dv15[c3_i108];
  }

  _SFD_SCRIPT_CALL(7U, chartInstance->c3_sfEvent, 32);
  for (c3_i109 = 0; c3_i109 < 16; c3_i109++) {
    c3_a[c3_i109] = c3_M[c3_i109];
  }

  for (c3_i110 = 0; c3_i110 < 16; c3_i110++) {
    c3_b[c3_i110] = c3_S_P[c3_i110];
  }

  c3_eml_scalar_eg(chartInstance);
  c3_eml_scalar_eg(chartInstance);
  for (c3_i111 = 0; c3_i111 < 16; c3_i111++) {
    c3_C_P[c3_i111] = 0.0;
  }

  for (c3_i112 = 0; c3_i112 < 16; c3_i112++) {
    c3_dv15[c3_i112] = 0.0;
  }

  for (c3_i113 = 0; c3_i113 < 16; c3_i113++) {
    c3_e_a[c3_i113] = c3_a[c3_i113];
  }

  for (c3_i114 = 0; c3_i114 < 16; c3_i114++) {
    c3_c_b[c3_i114] = c3_b[c3_i114];
  }

  c3_b_eml_xgemm(chartInstance, c3_e_a, c3_c_b, c3_dv15);
  for (c3_i115 = 0; c3_i115 < 16; c3_i115++) {
    c3_C_P[c3_i115] = c3_dv15[c3_i115];
  }

  _SFD_SCRIPT_CALL(7U, chartInstance->c3_sfEvent, 34);
  for (c3_i116 = 0; c3_i116 < 8; c3_i116++) {
    c3_p[c3_i116] = 0.0;
  }

  _SFD_SCRIPT_CALL(7U, chartInstance->c3_sfEvent, 35);
  c3_i117 = 0;
  for (c3_i118 = 0; c3_i118 < 4; c3_i118++) {
    c3_b_C_P[c3_i118] = c3_C_P[c3_i117];
    c3_i117 += 4;
  }

  c3_i119 = 0;
  for (c3_i120 = 0; c3_i120 < 4; c3_i120++) {
    c3_c_C_P[c3_i120] = c3_C_P[c3_i119 + 2];
    c3_i119 += 4;
  }

  c3_b_rdivide(chartInstance, c3_b_C_P, c3_c_C_P, c3_dv16);
  c3_i121 = 0;
  for (c3_i122 = 0; c3_i122 < 4; c3_i122++) {
    c3_p[c3_i121] = c3_dv16[c3_i122];
    c3_i121 += 2;
  }

  _SFD_SCRIPT_CALL(7U, chartInstance->c3_sfEvent, 36);
  c3_i123 = 0;
  for (c3_i124 = 0; c3_i124 < 4; c3_i124++) {
    c3_d_C_P[c3_i124] = c3_C_P[c3_i123 + 1];
    c3_i123 += 4;
  }

  c3_i125 = 0;
  for (c3_i126 = 0; c3_i126 < 4; c3_i126++) {
    c3_e_C_P[c3_i126] = c3_C_P[c3_i125 + 2];
    c3_i125 += 4;
  }

  c3_b_rdivide(chartInstance, c3_d_C_P, c3_e_C_P, c3_dv17);
  c3_i127 = 0;
  for (c3_i128 = 0; c3_i128 < 4; c3_i128++) {
    c3_p[c3_i127 + 1] = c3_dv17[c3_i128];
    c3_i127 += 2;
  }

  _SFD_SCRIPT_CALL(7U, chartInstance->c3_sfEvent, 39);
  for (c3_i129 = 0; c3_i129 < 8; c3_i129++) {
    c3_retVal->p[c3_i129] = c3_p[c3_i129];
  }

  _SFD_SCRIPT_CALL(7U, chartInstance->c3_sfEvent, 40);
  for (c3_i130 = 0; c3_i130 < 16; c3_i130++) {
    c3_retVal->C_P[c3_i130] = c3_C_P[c3_i130];
  }

  _SFD_SCRIPT_CALL(7U, chartInstance->c3_sfEvent, 41);
  for (c3_i131 = 0; c3_i131 < 16; c3_i131++) {
    c3_retVal->M[c3_i131] = c3_M[c3_i131];
  }

  _SFD_SCRIPT_CALL(7U, chartInstance->c3_sfEvent, -41);
  _SFD_SYMBOL_SCOPE_POP();
}

static void c3_mirage(SFc3_Robot_Control_v01InstanceStruct *chartInstance,
                      c3_smiE7r3wwCYeiytRpYXSxxE *c3_robot, real_T c3_S_PDes[16],
                      real_T c3_L_p[8], real_T c3_R_p[8], real_T c3_poseError[12])
{
  uint32_T c3_debug_family_var_map[23];
  c3_sZiCA83jGK4zdmds7wvlXUF c3_L_retValDes;
  c3_sZiCA83jGK4zdmds7wvlXUF c3_R_retValDes;
  real_T c3_L_pDes[8];
  real_T c3_R_pDes[8];
  real_T c3_L_PDes[16];
  real_T c3_R_PDes[16];
  real_T c3_L_M[16];
  real_T c3_R_M[16];
  real_T c3_Err_Left[8];
  real_T c3_Err_Right[8];
  real_T c3_Left_Equations[96];
  real_T c3_Left_Constants[8];
  real_T c3_Right_Equations[96];
  real_T c3_Right_Constants[8];
  real_T c3_AllEquations[192];
  real_T c3_AllConstants[16];
  real_T c3_nargin = 4.0;
  real_T c3_nargout = 1.0;
  int32_T c3_i132;
  real_T c3_b_S_PDes[16];
  c3_s6d9wPYgmS3cwGRsMJd1GAE c3_b_robot;
  c3_sZiCA83jGK4zdmds7wvlXUF c3_r0;
  int32_T c3_i133;
  real_T c3_c_S_PDes[16];
  c3_s6d9wPYgmS3cwGRsMJd1GAE c3_c_robot;
  c3_sZiCA83jGK4zdmds7wvlXUF c3_r1;
  int32_T c3_i134;
  int32_T c3_i135;
  int32_T c3_i136;
  int32_T c3_i137;
  int32_T c3_i138;
  int32_T c3_i139;
  int32_T c3_i140;
  int32_T c3_i141;
  int32_T c3_i142;
  real_T c3_b_L_PDes[16];
  int32_T c3_i143;
  real_T c3_d_S_PDes[16];
  int32_T c3_i144;
  real_T c3_b_Err_Left[8];
  int32_T c3_i145;
  real_T c3_b_L_M[16];
  real_T c3_b_Left_Constants[8];
  real_T c3_b_Left_Equations[96];
  int32_T c3_i146;
  int32_T c3_i147;
  int32_T c3_i148;
  real_T c3_b_R_PDes[16];
  int32_T c3_i149;
  real_T c3_e_S_PDes[16];
  int32_T c3_i150;
  real_T c3_b_Err_Right[8];
  int32_T c3_i151;
  real_T c3_b_R_M[16];
  int32_T c3_i152;
  int32_T c3_i153;
  int32_T c3_i154;
  int32_T c3_i155;
  int32_T c3_i156;
  int32_T c3_i157;
  int32_T c3_i158;
  int32_T c3_i159;
  int32_T c3_i160;
  int32_T c3_i161;
  int32_T c3_i162;
  int32_T c3_i163;
  int32_T c3_i164;
  real_T c3_b_AllEquations[192];
  int32_T c3_i165;
  real_T c3_b_AllConstants[16];
  real_T c3_dv18[12];
  int32_T c3_i166;
  _SFD_SYMBOL_SCOPE_PUSH_EML(0U, 23U, 23U, c3_m_debug_family_names,
    c3_debug_family_var_map);
  _SFD_SYMBOL_SCOPE_ADD_EML_IMPORTABLE(&c3_L_retValDes, 0U, c3_m_sf_marshallOut,
    c3_l_sf_marshallIn);
  _SFD_SYMBOL_SCOPE_ADD_EML_IMPORTABLE(&c3_R_retValDes, 1U, c3_m_sf_marshallOut,
    c3_l_sf_marshallIn);
  _SFD_SYMBOL_SCOPE_ADD_EML_IMPORTABLE(c3_L_pDes, 2U, c3_f_sf_marshallOut,
    c3_f_sf_marshallIn);
  _SFD_SYMBOL_SCOPE_ADD_EML_IMPORTABLE(c3_R_pDes, 3U, c3_f_sf_marshallOut,
    c3_f_sf_marshallIn);
  _SFD_SYMBOL_SCOPE_ADD_EML_IMPORTABLE(c3_L_PDes, 4U, c3_c_sf_marshallOut,
    c3_c_sf_marshallIn);
  _SFD_SYMBOL_SCOPE_ADD_EML_IMPORTABLE(c3_R_PDes, 5U, c3_c_sf_marshallOut,
    c3_c_sf_marshallIn);
  _SFD_SYMBOL_SCOPE_ADD_EML_IMPORTABLE(c3_L_M, 6U, c3_c_sf_marshallOut,
    c3_c_sf_marshallIn);
  _SFD_SYMBOL_SCOPE_ADD_EML_IMPORTABLE(c3_R_M, 7U, c3_c_sf_marshallOut,
    c3_c_sf_marshallIn);
  _SFD_SYMBOL_SCOPE_ADD_EML_IMPORTABLE(c3_Err_Left, 8U, c3_f_sf_marshallOut,
    c3_f_sf_marshallIn);
  _SFD_SYMBOL_SCOPE_ADD_EML_IMPORTABLE(c3_Err_Right, 9U, c3_f_sf_marshallOut,
    c3_f_sf_marshallIn);
  _SFD_SYMBOL_SCOPE_ADD_EML_IMPORTABLE(c3_Left_Equations, 10U,
    c3_o_sf_marshallOut, c3_n_sf_marshallIn);
  _SFD_SYMBOL_SCOPE_ADD_EML_IMPORTABLE(c3_Left_Constants, 11U,
    c3_n_sf_marshallOut, c3_m_sf_marshallIn);
  _SFD_SYMBOL_SCOPE_ADD_EML_IMPORTABLE(c3_Right_Equations, 12U,
    c3_o_sf_marshallOut, c3_n_sf_marshallIn);
  _SFD_SYMBOL_SCOPE_ADD_EML_IMPORTABLE(c3_Right_Constants, 13U,
    c3_n_sf_marshallOut, c3_m_sf_marshallIn);
  _SFD_SYMBOL_SCOPE_ADD_EML_IMPORTABLE(c3_AllEquations, 14U, c3_t_sf_marshallOut,
    c3_s_sf_marshallIn);
  _SFD_SYMBOL_SCOPE_ADD_EML_IMPORTABLE(c3_AllConstants, 15U, c3_s_sf_marshallOut,
    c3_r_sf_marshallIn);
  _SFD_SYMBOL_SCOPE_ADD_EML_IMPORTABLE(&c3_nargin, 16U, c3_b_sf_marshallOut,
    c3_b_sf_marshallIn);
  _SFD_SYMBOL_SCOPE_ADD_EML_IMPORTABLE(&c3_nargout, 17U, c3_b_sf_marshallOut,
    c3_b_sf_marshallIn);
  _SFD_SYMBOL_SCOPE_ADD_EML_IMPORTABLE(c3_robot, 18U, c3_e_sf_marshallOut,
    c3_e_sf_marshallIn);
  _SFD_SYMBOL_SCOPE_ADD_EML_IMPORTABLE(c3_S_PDes, 19U, c3_c_sf_marshallOut,
    c3_c_sf_marshallIn);
  _SFD_SYMBOL_SCOPE_ADD_EML_IMPORTABLE(c3_L_p, 20U, c3_f_sf_marshallOut,
    c3_f_sf_marshallIn);
  _SFD_SYMBOL_SCOPE_ADD_EML_IMPORTABLE(c3_R_p, 21U, c3_f_sf_marshallOut,
    c3_f_sf_marshallIn);
  _SFD_SYMBOL_SCOPE_ADD_EML_IMPORTABLE(c3_poseError, 22U, c3_r_sf_marshallOut,
    c3_q_sf_marshallIn);
  CV_SCRIPT_FCN(9, 0);
  _SFD_SCRIPT_CALL(9U, chartInstance->c3_sfEvent, 9);
  for (c3_i132 = 0; c3_i132 < 16; c3_i132++) {
    c3_b_S_PDes[c3_i132] = c3_S_PDes[c3_i132];
  }

  c3_b_robot = c3_robot->leftCamera;
  c3_transformImageSpace(chartInstance, c3_b_S_PDes, &c3_b_robot, &c3_r0);
  c3_L_retValDes = c3_r0;
  _SFD_SCRIPT_CALL(9U, chartInstance->c3_sfEvent, 10);
  for (c3_i133 = 0; c3_i133 < 16; c3_i133++) {
    c3_c_S_PDes[c3_i133] = c3_S_PDes[c3_i133];
  }

  c3_c_robot = c3_robot->rightCamera;
  c3_transformImageSpace(chartInstance, c3_c_S_PDes, &c3_c_robot, &c3_r1);
  c3_R_retValDes = c3_r1;
  _SFD_SCRIPT_CALL(9U, chartInstance->c3_sfEvent, 13);
  for (c3_i134 = 0; c3_i134 < 8; c3_i134++) {
    c3_L_pDes[c3_i134] = c3_L_retValDes.p[c3_i134];
  }

  _SFD_SCRIPT_CALL(9U, chartInstance->c3_sfEvent, 14);
  for (c3_i135 = 0; c3_i135 < 8; c3_i135++) {
    c3_R_pDes[c3_i135] = c3_R_retValDes.p[c3_i135];
  }

  _SFD_SCRIPT_CALL(9U, chartInstance->c3_sfEvent, 16);
  for (c3_i136 = 0; c3_i136 < 16; c3_i136++) {
    c3_L_PDes[c3_i136] = c3_L_retValDes.C_P[c3_i136];
  }

  _SFD_SCRIPT_CALL(9U, chartInstance->c3_sfEvent, 17);
  for (c3_i137 = 0; c3_i137 < 16; c3_i137++) {
    c3_R_PDes[c3_i137] = c3_R_retValDes.C_P[c3_i137];
  }

  _SFD_SCRIPT_CALL(9U, chartInstance->c3_sfEvent, 19);
  for (c3_i138 = 0; c3_i138 < 16; c3_i138++) {
    c3_L_M[c3_i138] = c3_L_retValDes.M[c3_i138];
  }

  _SFD_SCRIPT_CALL(9U, chartInstance->c3_sfEvent, 20);
  for (c3_i139 = 0; c3_i139 < 16; c3_i139++) {
    c3_R_M[c3_i139] = c3_R_retValDes.M[c3_i139];
  }

  _SFD_SCRIPT_CALL(9U, chartInstance->c3_sfEvent, 23);
  for (c3_i140 = 0; c3_i140 < 8; c3_i140++) {
    c3_Err_Left[c3_i140] = c3_L_p[c3_i140] - c3_L_pDes[c3_i140];
  }

  _SFD_SCRIPT_CALL(9U, chartInstance->c3_sfEvent, 24);
  for (c3_i141 = 0; c3_i141 < 8; c3_i141++) {
    c3_Err_Right[c3_i141] = c3_R_p[c3_i141] - c3_R_pDes[c3_i141];
  }

  _SFD_SCRIPT_CALL(9U, chartInstance->c3_sfEvent, 27);
  for (c3_i142 = 0; c3_i142 < 16; c3_i142++) {
    c3_b_L_PDes[c3_i142] = c3_L_PDes[c3_i142];
  }

  for (c3_i143 = 0; c3_i143 < 16; c3_i143++) {
    c3_d_S_PDes[c3_i143] = c3_S_PDes[c3_i143];
  }

  for (c3_i144 = 0; c3_i144 < 8; c3_i144++) {
    c3_b_Err_Left[c3_i144] = c3_Err_Left[c3_i144];
  }

  for (c3_i145 = 0; c3_i145 < 16; c3_i145++) {
    c3_b_L_M[c3_i145] = c3_L_M[c3_i145];
  }

  c3_getEquations(chartInstance, c3_b_L_PDes, c3_d_S_PDes, c3_b_Err_Left,
                  c3_b_L_M, c3_b_Left_Equations, c3_b_Left_Constants);
  for (c3_i146 = 0; c3_i146 < 96; c3_i146++) {
    c3_Left_Equations[c3_i146] = c3_b_Left_Equations[c3_i146];
  }

  for (c3_i147 = 0; c3_i147 < 8; c3_i147++) {
    c3_Left_Constants[c3_i147] = c3_b_Left_Constants[c3_i147];
  }

  _SFD_SCRIPT_CALL(9U, chartInstance->c3_sfEvent, 28);
  for (c3_i148 = 0; c3_i148 < 16; c3_i148++) {
    c3_b_R_PDes[c3_i148] = c3_R_PDes[c3_i148];
  }

  for (c3_i149 = 0; c3_i149 < 16; c3_i149++) {
    c3_e_S_PDes[c3_i149] = c3_S_PDes[c3_i149];
  }

  for (c3_i150 = 0; c3_i150 < 8; c3_i150++) {
    c3_b_Err_Right[c3_i150] = c3_Err_Right[c3_i150];
  }

  for (c3_i151 = 0; c3_i151 < 16; c3_i151++) {
    c3_b_R_M[c3_i151] = c3_R_M[c3_i151];
  }

  c3_getEquations(chartInstance, c3_b_R_PDes, c3_e_S_PDes, c3_b_Err_Right,
                  c3_b_R_M, c3_b_Left_Equations, c3_b_Left_Constants);
  for (c3_i152 = 0; c3_i152 < 96; c3_i152++) {
    c3_Right_Equations[c3_i152] = c3_b_Left_Equations[c3_i152];
  }

  for (c3_i153 = 0; c3_i153 < 8; c3_i153++) {
    c3_Right_Constants[c3_i153] = c3_b_Left_Constants[c3_i153];
  }

  _SFD_SCRIPT_CALL(9U, chartInstance->c3_sfEvent, 30);
  c3_i154 = 0;
  c3_i155 = 0;
  for (c3_i156 = 0; c3_i156 < 12; c3_i156++) {
    for (c3_i157 = 0; c3_i157 < 8; c3_i157++) {
      c3_AllEquations[c3_i157 + c3_i154] = c3_Left_Equations[c3_i157 + c3_i155];
    }

    c3_i154 += 16;
    c3_i155 += 8;
  }

  c3_i158 = 0;
  c3_i159 = 0;
  for (c3_i160 = 0; c3_i160 < 12; c3_i160++) {
    for (c3_i161 = 0; c3_i161 < 8; c3_i161++) {
      c3_AllEquations[(c3_i161 + c3_i158) + 8] = c3_Right_Equations[c3_i161 +
        c3_i159];
    }

    c3_i158 += 16;
    c3_i159 += 8;
  }

  _SFD_SCRIPT_CALL(9U, chartInstance->c3_sfEvent, 31);
  for (c3_i162 = 0; c3_i162 < 8; c3_i162++) {
    c3_AllConstants[c3_i162] = c3_Left_Constants[c3_i162];
  }

  for (c3_i163 = 0; c3_i163 < 8; c3_i163++) {
    c3_AllConstants[c3_i163 + 8] = c3_Right_Constants[c3_i163];
  }

  _SFD_SCRIPT_CALL(9U, chartInstance->c3_sfEvent, 35);
  for (c3_i164 = 0; c3_i164 < 192; c3_i164++) {
    c3_b_AllEquations[c3_i164] = c3_AllEquations[c3_i164];
  }

  for (c3_i165 = 0; c3_i165 < 16; c3_i165++) {
    c3_b_AllConstants[c3_i165] = c3_AllConstants[c3_i165];
  }

  c3_mldivide(chartInstance, c3_b_AllEquations, c3_b_AllConstants, c3_dv18);
  for (c3_i166 = 0; c3_i166 < 12; c3_i166++) {
    c3_poseError[c3_i166] = c3_dv18[c3_i166];
  }

  _SFD_SCRIPT_CALL(9U, chartInstance->c3_sfEvent, -35);
  _SFD_SYMBOL_SCOPE_POP();
}

static void c3_getEquations(SFc3_Robot_Control_v01InstanceStruct *chartInstance,
  real_T c3_Cam_PDes[16], real_T c3_S_PDes[16], real_T c3_Err[8], real_T c3_M[16],
  real_T c3_equations[96], real_T c3_constants[8])
{
  uint32_T c3_debug_family_var_map[14];
  real_T c3_n[12];
  real_T c3_j;
  real_T c3_i;
  real_T c3_O[12];
  real_T c3_Vx[3];
  real_T c3_Vy[3];
  real_T c3_nargin = 4.0;
  real_T c3_nargout = 2.0;
  int32_T c3_i167;
  int32_T c3_i168;
  int32_T c3_i169;
  int32_T c3_b_i;
  real_T c3_a;
  int32_T c3_i170;
  int32_T c3_i171;
  real_T c3_b[4];
  int32_T c3_i172;
  real_T c3_b_a;
  int32_T c3_i173;
  int32_T c3_i174;
  real_T c3_b_b[4];
  int32_T c3_i175;
  int32_T c3_i176;
  int32_T c3_i177;
  real_T c3_c_a;
  int32_T c3_i178;
  int32_T c3_i179;
  int32_T c3_i180;
  real_T c3_d_a;
  int32_T c3_i181;
  int32_T c3_i182;
  int32_T c3_i183;
  int32_T c3_i184;
  int32_T c3_i185;
  real_T c3_e_a;
  int32_T c3_i186;
  int32_T c3_i187;
  int32_T c3_i188;
  int32_T c3_i189;
  int32_T c3_i190;
  int32_T c3_i191;
  real_T c3_A[12];
  real_T c3_B;
  real_T c3_y;
  real_T c3_b_y;
  real_T c3_c_y;
  int32_T c3_i192;
  real_T c3_f_a;
  int32_T c3_i193;
  int32_T c3_i194;
  real_T c3_c_b[3];
  int32_T c3_i195;
  int32_T c3_i196;
  int32_T c3_i197;
  real_T c3_g_a;
  int32_T c3_i198;
  int32_T c3_i199;
  int32_T c3_i200;
  int32_T c3_i201;
  int32_T c3_i202;
  int32_T c3_b_j;
  real_T c3_b_S_PDes;
  real_T c3_c_S_PDes;
  real_T c3_d_S_PDes;
  int32_T c3_i203;
  int32_T c3_i204;
  int32_T c3_i205;
  int32_T c3_i206;
  int32_T c3_c_j;
  real_T c3_e_S_PDes;
  real_T c3_f_S_PDes;
  real_T c3_g_S_PDes;
  int32_T c3_i207;
  int32_T c3_i208;
  int32_T c3_i209;
  int32_T c3_i210;
  _SFD_SYMBOL_SCOPE_PUSH_EML(0U, 14U, 14U, c3_l_debug_family_names,
    c3_debug_family_var_map);
  _SFD_SYMBOL_SCOPE_ADD_EML_IMPORTABLE(c3_n, 0U, c3_q_sf_marshallOut,
    c3_p_sf_marshallIn);
  _SFD_SYMBOL_SCOPE_ADD_EML_IMPORTABLE(&c3_j, 1U, c3_b_sf_marshallOut,
    c3_b_sf_marshallIn);
  _SFD_SYMBOL_SCOPE_ADD_EML_IMPORTABLE(&c3_i, 2U, c3_b_sf_marshallOut,
    c3_b_sf_marshallIn);
  _SFD_SYMBOL_SCOPE_ADD_EML_IMPORTABLE(c3_O, 3U, c3_q_sf_marshallOut,
    c3_p_sf_marshallIn);
  _SFD_SYMBOL_SCOPE_ADD_EML_IMPORTABLE(c3_Vx, 4U, c3_p_sf_marshallOut,
    c3_o_sf_marshallIn);
  _SFD_SYMBOL_SCOPE_ADD_EML_IMPORTABLE(c3_Vy, 5U, c3_p_sf_marshallOut,
    c3_o_sf_marshallIn);
  _SFD_SYMBOL_SCOPE_ADD_EML_IMPORTABLE(&c3_nargin, 6U, c3_b_sf_marshallOut,
    c3_b_sf_marshallIn);
  _SFD_SYMBOL_SCOPE_ADD_EML_IMPORTABLE(&c3_nargout, 7U, c3_b_sf_marshallOut,
    c3_b_sf_marshallIn);
  _SFD_SYMBOL_SCOPE_ADD_EML_IMPORTABLE(c3_Cam_PDes, 8U, c3_c_sf_marshallOut,
    c3_c_sf_marshallIn);
  _SFD_SYMBOL_SCOPE_ADD_EML_IMPORTABLE(c3_S_PDes, 9U, c3_c_sf_marshallOut,
    c3_c_sf_marshallIn);
  _SFD_SYMBOL_SCOPE_ADD_EML_IMPORTABLE(c3_Err, 10U, c3_f_sf_marshallOut,
    c3_f_sf_marshallIn);
  _SFD_SYMBOL_SCOPE_ADD_EML_IMPORTABLE(c3_M, 11U, c3_c_sf_marshallOut,
    c3_c_sf_marshallIn);
  _SFD_SYMBOL_SCOPE_ADD_EML_IMPORTABLE(c3_equations, 12U, c3_o_sf_marshallOut,
    c3_n_sf_marshallIn);
  _SFD_SYMBOL_SCOPE_ADD_EML_IMPORTABLE(c3_constants, 13U, c3_n_sf_marshallOut,
    c3_m_sf_marshallIn);
  CV_SCRIPT_FCN(10, 0);
  _SFD_SCRIPT_CALL(10U, chartInstance->c3_sfEvent, 14);
  for (c3_i167 = 0; c3_i167 < 96; c3_i167++) {
    c3_equations[c3_i167] = 0.0;
  }

  _SFD_SCRIPT_CALL(10U, chartInstance->c3_sfEvent, 15);
  for (c3_i168 = 0; c3_i168 < 8; c3_i168++) {
    c3_constants[c3_i168] = 0.0;
  }

  _SFD_SCRIPT_CALL(10U, chartInstance->c3_sfEvent, 17);
  for (c3_i169 = 0; c3_i169 < 12; c3_i169++) {
    c3_n[c3_i169] = 0.0;
  }

  _SFD_SCRIPT_CALL(10U, chartInstance->c3_sfEvent, 19);
  c3_j = 1.0;
  _SFD_SCRIPT_CALL(10U, chartInstance->c3_sfEvent, 20);
  c3_i = 1.0;
  c3_b_i = 0;
  while (c3_b_i < 4) {
    c3_i = 1.0 + (real_T)c3_b_i;
    CV_SCRIPT_FOR(10, 0, 1);
    _SFD_SCRIPT_CALL(10U, chartInstance->c3_sfEvent, 22);
    c3_a = c3_Cam_PDes[2 + ((_SFD_EML_ARRAY_BOUNDS_CHECK("Cam_PDes", (int32_T)
      _SFD_INTEGER_CHECK("i", c3_i), 1, 4, 2, 0) - 1) << 2)];
    c3_i170 = 0;
    for (c3_i171 = 0; c3_i171 < 4; c3_i171++) {
      c3_b[c3_i171] = c3_M[c3_i170];
      c3_i170 += 4;
    }

    for (c3_i172 = 0; c3_i172 < 4; c3_i172++) {
      c3_b[c3_i172] *= c3_a;
    }

    c3_b_a = c3_Cam_PDes[(_SFD_EML_ARRAY_BOUNDS_CHECK("Cam_PDes", (int32_T)
      _SFD_INTEGER_CHECK("i", c3_i), 1, 4, 2, 0) - 1) << 2];
    c3_i173 = 0;
    for (c3_i174 = 0; c3_i174 < 4; c3_i174++) {
      c3_b_b[c3_i174] = c3_M[c3_i173 + 2];
      c3_i173 += 4;
    }

    for (c3_i175 = 0; c3_i175 < 4; c3_i175++) {
      c3_b_b[c3_i175] *= c3_b_a;
    }

    c3_i176 = 0;
    for (c3_i177 = 0; c3_i177 < 4; c3_i177++) {
      c3_n[c3_i176] = c3_b[c3_i177] - c3_b_b[c3_i177];
      c3_i176 += 3;
    }

    _SFD_SCRIPT_CALL(10U, chartInstance->c3_sfEvent, 23);
    c3_c_a = c3_Cam_PDes[2 + ((_SFD_EML_ARRAY_BOUNDS_CHECK("Cam_PDes", (int32_T)
      _SFD_INTEGER_CHECK("i", c3_i), 1, 4, 2, 0) - 1) << 2)];
    c3_i178 = 0;
    for (c3_i179 = 0; c3_i179 < 4; c3_i179++) {
      c3_b[c3_i179] = c3_M[c3_i178 + 1];
      c3_i178 += 4;
    }

    for (c3_i180 = 0; c3_i180 < 4; c3_i180++) {
      c3_b[c3_i180] *= c3_c_a;
    }

    c3_d_a = c3_Cam_PDes[1 + ((_SFD_EML_ARRAY_BOUNDS_CHECK("Cam_PDes", (int32_T)
      _SFD_INTEGER_CHECK("i", c3_i), 1, 4, 2, 0) - 1) << 2)];
    c3_i181 = 0;
    for (c3_i182 = 0; c3_i182 < 4; c3_i182++) {
      c3_b_b[c3_i182] = c3_M[c3_i181 + 2];
      c3_i181 += 4;
    }

    for (c3_i183 = 0; c3_i183 < 4; c3_i183++) {
      c3_b_b[c3_i183] *= c3_d_a;
    }

    c3_i184 = 0;
    for (c3_i185 = 0; c3_i185 < 4; c3_i185++) {
      c3_n[c3_i184 + 1] = c3_b[c3_i185] - c3_b_b[c3_i185];
      c3_i184 += 3;
    }

    _SFD_SCRIPT_CALL(10U, chartInstance->c3_sfEvent, 24);
    c3_e_a = c3_Cam_PDes[2 + ((_SFD_EML_ARRAY_BOUNDS_CHECK("Cam_PDes", (int32_T)
      _SFD_INTEGER_CHECK("i", c3_i), 1, 4, 2, 0) - 1) << 2)];
    c3_i186 = 0;
    for (c3_i187 = 0; c3_i187 < 4; c3_i187++) {
      c3_b[c3_i187] = c3_M[c3_i186 + 2];
      c3_i186 += 4;
    }

    for (c3_i188 = 0; c3_i188 < 4; c3_i188++) {
      c3_b[c3_i188] *= c3_e_a;
    }

    c3_i189 = 0;
    for (c3_i190 = 0; c3_i190 < 4; c3_i190++) {
      c3_n[c3_i189 + 2] = c3_b[c3_i190];
      c3_i189 += 3;
    }

    _SFD_SCRIPT_CALL(10U, chartInstance->c3_sfEvent, 26);
    for (c3_i191 = 0; c3_i191 < 12; c3_i191++) {
      c3_A[c3_i191] = c3_n[c3_i191];
    }

    c3_B = c3_n[11];
    c3_y = c3_B;
    c3_b_y = c3_y;
    c3_c_y = c3_b_y;
    for (c3_i192 = 0; c3_i192 < 12; c3_i192++) {
      c3_O[c3_i192] = c3_A[c3_i192] / c3_c_y;
    }

    _SFD_SCRIPT_CALL(10U, chartInstance->c3_sfEvent, 27);
    c3_O[11] = 0.0;
    _SFD_SCRIPT_CALL(10U, chartInstance->c3_sfEvent, 29);
    c3_f_a = c3_Err[(_SFD_EML_ARRAY_BOUNDS_CHECK("Err", (int32_T)
      _SFD_INTEGER_CHECK("i", c3_i), 1, 4, 2, 0) - 1) << 1];
    c3_i193 = 0;
    for (c3_i194 = 0; c3_i194 < 3; c3_i194++) {
      c3_c_b[c3_i194] = c3_O[c3_i193 + 2];
      c3_i193 += 3;
    }

    for (c3_i195 = 0; c3_i195 < 3; c3_i195++) {
      c3_c_b[c3_i195] *= c3_f_a;
    }

    c3_i196 = 0;
    for (c3_i197 = 0; c3_i197 < 3; c3_i197++) {
      c3_Vx[c3_i197] = c3_O[c3_i196] - c3_c_b[c3_i197];
      c3_i196 += 3;
    }

    _SFD_SCRIPT_CALL(10U, chartInstance->c3_sfEvent, 30);
    c3_g_a = c3_Err[1 + ((_SFD_EML_ARRAY_BOUNDS_CHECK("Err", (int32_T)
      _SFD_INTEGER_CHECK("i", c3_i), 1, 4, 2, 0) - 1) << 1)];
    c3_i198 = 0;
    for (c3_i199 = 0; c3_i199 < 3; c3_i199++) {
      c3_c_b[c3_i199] = c3_O[c3_i198 + 2];
      c3_i198 += 3;
    }

    for (c3_i200 = 0; c3_i200 < 3; c3_i200++) {
      c3_c_b[c3_i200] *= c3_g_a;
    }

    c3_i201 = 0;
    for (c3_i202 = 0; c3_i202 < 3; c3_i202++) {
      c3_Vy[c3_i202] = c3_O[c3_i201 + 1] - c3_c_b[c3_i202];
      c3_i201 += 3;
    }

    _SFD_SCRIPT_CALL(10U, chartInstance->c3_sfEvent, 32);
    c3_b_j = _SFD_EML_ARRAY_BOUNDS_CHECK("equations", (int32_T)
      _SFD_INTEGER_CHECK("j", c3_j), 1, 8, 1, 0) - 1;
    c3_b_S_PDes = c3_S_PDes[(_SFD_EML_ARRAY_BOUNDS_CHECK("S_PDes", (int32_T)
      _SFD_INTEGER_CHECK("i", c3_i), 1, 4, 2, 0) - 1) << 2];
    c3_c_S_PDes = c3_S_PDes[1 + ((_SFD_EML_ARRAY_BOUNDS_CHECK("S_PDes", (int32_T)
      _SFD_INTEGER_CHECK("i", c3_i), 1, 4, 2, 0) - 1) << 2)];
    c3_d_S_PDes = c3_S_PDes[2 + ((_SFD_EML_ARRAY_BOUNDS_CHECK("S_PDes", (int32_T)
      _SFD_INTEGER_CHECK("i", c3_i), 1, 4, 2, 0) - 1) << 2)];
    for (c3_i203 = 0; c3_i203 < 3; c3_i203++) {
      c3_equations[c3_b_j + (c3_i203 << 3)] = c3_Vx[c3_i203] * c3_b_S_PDes;
    }

    for (c3_i204 = 0; c3_i204 < 3; c3_i204++) {
      c3_equations[c3_b_j + ((c3_i204 + 3) << 3)] = c3_Vx[c3_i204] * c3_c_S_PDes;
    }

    for (c3_i205 = 0; c3_i205 < 3; c3_i205++) {
      c3_equations[c3_b_j + ((c3_i205 + 6) << 3)] = c3_Vx[c3_i205] * c3_d_S_PDes;
    }

    for (c3_i206 = 0; c3_i206 < 3; c3_i206++) {
      c3_equations[c3_b_j + ((c3_i206 + 9) << 3)] = c3_Vx[c3_i206];
    }

    _SFD_SCRIPT_CALL(10U, chartInstance->c3_sfEvent, 33);
    c3_c_j = _SFD_EML_ARRAY_BOUNDS_CHECK("equations", (int32_T)
      _SFD_INTEGER_CHECK("j+1", c3_j + 1.0), 1, 8, 1, 0) - 1;
    c3_e_S_PDes = c3_S_PDes[(_SFD_EML_ARRAY_BOUNDS_CHECK("S_PDes", (int32_T)
      _SFD_INTEGER_CHECK("i", c3_i), 1, 4, 2, 0) - 1) << 2];
    c3_f_S_PDes = c3_S_PDes[1 + ((_SFD_EML_ARRAY_BOUNDS_CHECK("S_PDes", (int32_T)
      _SFD_INTEGER_CHECK("i", c3_i), 1, 4, 2, 0) - 1) << 2)];
    c3_g_S_PDes = c3_S_PDes[2 + ((_SFD_EML_ARRAY_BOUNDS_CHECK("S_PDes", (int32_T)
      _SFD_INTEGER_CHECK("i", c3_i), 1, 4, 2, 0) - 1) << 2)];
    for (c3_i207 = 0; c3_i207 < 3; c3_i207++) {
      c3_equations[c3_c_j + (c3_i207 << 3)] = c3_Vy[c3_i207] * c3_e_S_PDes;
    }

    for (c3_i208 = 0; c3_i208 < 3; c3_i208++) {
      c3_equations[c3_c_j + ((c3_i208 + 3) << 3)] = c3_Vy[c3_i208] * c3_f_S_PDes;
    }

    for (c3_i209 = 0; c3_i209 < 3; c3_i209++) {
      c3_equations[c3_c_j + ((c3_i209 + 6) << 3)] = c3_Vy[c3_i209] * c3_g_S_PDes;
    }

    for (c3_i210 = 0; c3_i210 < 3; c3_i210++) {
      c3_equations[c3_c_j + ((c3_i210 + 9) << 3)] = c3_Vy[c3_i210];
    }

    _SFD_SCRIPT_CALL(10U, chartInstance->c3_sfEvent, 35);
    c3_constants[_SFD_EML_ARRAY_BOUNDS_CHECK("constants", (int32_T)
      _SFD_INTEGER_CHECK("j", c3_j), 1, 8, 1, 0) - 1] = c3_Err
      [(_SFD_EML_ARRAY_BOUNDS_CHECK("Err", (int32_T)_SFD_INTEGER_CHECK("i", c3_i),
         1, 4, 2, 0) - 1) << 1] - c3_O[9];
    _SFD_SCRIPT_CALL(10U, chartInstance->c3_sfEvent, 36);
    c3_constants[_SFD_EML_ARRAY_BOUNDS_CHECK("constants", (int32_T)
      _SFD_INTEGER_CHECK("j+1", c3_j + 1.0), 1, 8, 1, 0) - 1] = c3_Err[1 +
      ((_SFD_EML_ARRAY_BOUNDS_CHECK("Err", (int32_T)_SFD_INTEGER_CHECK("i", c3_i),
         1, 4, 2, 0) - 1) << 1)] - c3_O[10];
    _SFD_SCRIPT_CALL(10U, chartInstance->c3_sfEvent, 38);
    c3_j += 2.0;
    c3_b_i++;
    _SF_MEX_LISTEN_FOR_CTRL_C(chartInstance->S);
  }

  CV_SCRIPT_FOR(10, 0, 0);
  _SFD_SCRIPT_CALL(10U, chartInstance->c3_sfEvent, -38);
  _SFD_SYMBOL_SCOPE_POP();
}

static void init_script_number_translation(uint32_T c3_machineNumber, uint32_T
  c3_chartNumber, uint32_T c3_instanceNumber)
{
  (void)c3_machineNumber;
  _SFD_SCRIPT_TRANSLATION(c3_chartNumber, c3_instanceNumber, 0U,
    sf_debug_get_script_id(
    "C:\\Users\\sd0016\\Dropbox\\UAH Doktora\\Journal Tracking\\Matlab Code\\3DOF_Experiments\\random.m"));
  _SFD_SCRIPT_TRANSLATION(c3_chartNumber, c3_instanceNumber, 1U,
    sf_debug_get_script_id(
    "C:\\Users\\sd0016\\Dropbox\\UAH Doktora\\Journal Tracking\\Matlab Code\\3DOF_Experiments\\Mirage\\util\\newPose.m"));
  _SFD_SCRIPT_TRANSLATION(c3_chartNumber, c3_instanceNumber, 2U,
    sf_debug_get_script_id(
    "C:\\Users\\sd0016\\Dropbox\\UAH Doktora\\Journal Tracking\\Matlab Code\\3DOF_Experiments\\Mirage\\util\\transformationMatrix.m"));
  _SFD_SCRIPT_TRANSLATION(c3_chartNumber, c3_instanceNumber, 3U,
    sf_debug_get_script_id(
    "C:\\Users\\sd0016\\Dropbox\\UAH Doktora\\Journal Tracking\\Matlab Code\\3DOF_Experiments\\EulerTrans.m"));
  _SFD_SCRIPT_TRANSLATION(c3_chartNumber, c3_instanceNumber, 4U,
    sf_debug_get_script_id(
    "C:\\Users\\sd0016\\Dropbox\\UAH Doktora\\Journal Tracking\\Matlab Code\\3DOF_Experiments\\Mirage\\util\\newCamera.m"));
  _SFD_SCRIPT_TRANSLATION(c3_chartNumber, c3_instanceNumber, 5U,
    sf_debug_get_script_id(
    "C:\\Users\\sd0016\\Dropbox\\UAH Doktora\\Journal Tracking\\Matlab Code\\3DOF_Experiments\\Mirage\\util\\newRobot.m"));
  _SFD_SCRIPT_TRANSLATION(c3_chartNumber, c3_instanceNumber, 6U,
    sf_debug_get_script_id(
    "C:\\Users\\sd0016\\Dropbox\\UAH Doktora\\Journal Tracking\\Matlab Code\\3DOF_Experiments\\Mirage\\util\\simulateRealRobot.m"));
  _SFD_SCRIPT_TRANSLATION(c3_chartNumber, c3_instanceNumber, 7U,
    sf_debug_get_script_id(
    "C:\\Users\\sd0016\\Dropbox\\UAH Doktora\\Journal Tracking\\Matlab Code\\3DOF_Experiments\\Mirage\\util\\transformImageSpace.m"));
  _SFD_SCRIPT_TRANSLATION(c3_chartNumber, c3_instanceNumber, 8U,
    sf_debug_get_script_id(
    "C:\\Users\\sd0016\\Dropbox\\UAH Doktora\\Journal Tracking\\Matlab Code\\3DOF_Experiments\\Mirage\\util\\intrinsicMatrix.m"));
  _SFD_SCRIPT_TRANSLATION(c3_chartNumber, c3_instanceNumber, 9U,
    sf_debug_get_script_id(
    "C:\\Users\\sd0016\\Dropbox\\UAH Doktora\\Journal Tracking\\Matlab Code\\3DOF_Experiments\\Mirage\\mirage.m"));
  _SFD_SCRIPT_TRANSLATION(c3_chartNumber, c3_instanceNumber, 10U,
    sf_debug_get_script_id(
    "C:\\Users\\sd0016\\Dropbox\\UAH Doktora\\Journal Tracking\\Matlab Code\\3DOF_Experiments\\getEquations.m"));
  _SFD_SCRIPT_TRANSLATION(c3_chartNumber, c3_instanceNumber, 11U,
    sf_debug_get_script_id(
    "C:\\Users\\sd0016\\Dropbox\\UAH Doktora\\Journal Tracking\\Matlab Code\\3DOF_Experiments\\get3DRotationAngles.m"));
}

static void c3_emlrt_marshallIn(SFc3_Robot_Control_v01InstanceStruct
  *chartInstance, const mxArray *c3_sprintf, const char_T *c3_identifier, char_T
  c3_y[14])
{
  emlrtMsgIdentifier c3_thisId;
  c3_thisId.fIdentifier = c3_identifier;
  c3_thisId.fParent = NULL;
  c3_b_emlrt_marshallIn(chartInstance, sf_mex_dup(c3_sprintf), &c3_thisId, c3_y);
  sf_mex_destroy(&c3_sprintf);
}

static void c3_b_emlrt_marshallIn(SFc3_Robot_Control_v01InstanceStruct
  *chartInstance, const mxArray *c3_u, const emlrtMsgIdentifier *c3_parentId,
  char_T c3_y[14])
{
  char_T c3_cv0[14];
  int32_T c3_i211;
  (void)chartInstance;
  sf_mex_import(c3_parentId, sf_mex_dup(c3_u), c3_cv0, 1, 10, 0U, 1, 0U, 2, 1,
                14);
  for (c3_i211 = 0; c3_i211 < 14; c3_i211++) {
    c3_y[c3_i211] = c3_cv0[c3_i211];
  }

  sf_mex_destroy(&c3_u);
}

static const mxArray *c3_sf_marshallOut(void *chartInstanceVoid, void *c3_inData)
{
  const mxArray *c3_mxArrayOutData = NULL;
  int32_T c3_i212;
  real_T c3_b_inData[3];
  int32_T c3_i213;
  real_T c3_u[3];
  const mxArray *c3_y = NULL;
  SFc3_Robot_Control_v01InstanceStruct *chartInstance;
  chartInstance = (SFc3_Robot_Control_v01InstanceStruct *)chartInstanceVoid;
  c3_mxArrayOutData = NULL;
  for (c3_i212 = 0; c3_i212 < 3; c3_i212++) {
    c3_b_inData[c3_i212] = (*(real_T (*)[3])c3_inData)[c3_i212];
  }

  for (c3_i213 = 0; c3_i213 < 3; c3_i213++) {
    c3_u[c3_i213] = c3_b_inData[c3_i213];
  }

  c3_y = NULL;
  sf_mex_assign(&c3_y, sf_mex_create("y", c3_u, 0, 0U, 1U, 0U, 1, 3), false);
  sf_mex_assign(&c3_mxArrayOutData, c3_y, false);
  return c3_mxArrayOutData;
}

static void c3_c_emlrt_marshallIn(SFc3_Robot_Control_v01InstanceStruct
  *chartInstance, const mxArray *c3_b_qtilde, const char_T *c3_identifier,
  real_T c3_y[3])
{
  emlrtMsgIdentifier c3_thisId;
  c3_thisId.fIdentifier = c3_identifier;
  c3_thisId.fParent = NULL;
  c3_d_emlrt_marshallIn(chartInstance, sf_mex_dup(c3_b_qtilde), &c3_thisId, c3_y);
  sf_mex_destroy(&c3_b_qtilde);
}

static void c3_d_emlrt_marshallIn(SFc3_Robot_Control_v01InstanceStruct
  *chartInstance, const mxArray *c3_u, const emlrtMsgIdentifier *c3_parentId,
  real_T c3_y[3])
{
  real_T c3_dv19[3];
  int32_T c3_i214;
  (void)chartInstance;
  sf_mex_import(c3_parentId, sf_mex_dup(c3_u), c3_dv19, 1, 0, 0U, 1, 0U, 1, 3);
  for (c3_i214 = 0; c3_i214 < 3; c3_i214++) {
    c3_y[c3_i214] = c3_dv19[c3_i214];
  }

  sf_mex_destroy(&c3_u);
}

static void c3_sf_marshallIn(void *chartInstanceVoid, const mxArray
  *c3_mxArrayInData, const char_T *c3_varName, void *c3_outData)
{
  const mxArray *c3_b_qtilde;
  const char_T *c3_identifier;
  emlrtMsgIdentifier c3_thisId;
  real_T c3_y[3];
  int32_T c3_i215;
  SFc3_Robot_Control_v01InstanceStruct *chartInstance;
  chartInstance = (SFc3_Robot_Control_v01InstanceStruct *)chartInstanceVoid;
  c3_b_qtilde = sf_mex_dup(c3_mxArrayInData);
  c3_identifier = c3_varName;
  c3_thisId.fIdentifier = c3_identifier;
  c3_thisId.fParent = NULL;
  c3_d_emlrt_marshallIn(chartInstance, sf_mex_dup(c3_b_qtilde), &c3_thisId, c3_y);
  sf_mex_destroy(&c3_b_qtilde);
  for (c3_i215 = 0; c3_i215 < 3; c3_i215++) {
    (*(real_T (*)[3])c3_outData)[c3_i215] = c3_y[c3_i215];
  }

  sf_mex_destroy(&c3_mxArrayInData);
}

static const mxArray *c3_b_sf_marshallOut(void *chartInstanceVoid, void
  *c3_inData)
{
  const mxArray *c3_mxArrayOutData = NULL;
  real_T c3_u;
  const mxArray *c3_y = NULL;
  SFc3_Robot_Control_v01InstanceStruct *chartInstance;
  chartInstance = (SFc3_Robot_Control_v01InstanceStruct *)chartInstanceVoid;
  c3_mxArrayOutData = NULL;
  c3_u = *(real_T *)c3_inData;
  c3_y = NULL;
  sf_mex_assign(&c3_y, sf_mex_create("y", &c3_u, 0, 0U, 0U, 0U, 0), false);
  sf_mex_assign(&c3_mxArrayOutData, c3_y, false);
  return c3_mxArrayOutData;
}

static real_T c3_e_emlrt_marshallIn(SFc3_Robot_Control_v01InstanceStruct
  *chartInstance, const mxArray *c3_u, const emlrtMsgIdentifier *c3_parentId)
{
  real_T c3_y;
  real_T c3_d0;
  (void)chartInstance;
  sf_mex_import(c3_parentId, sf_mex_dup(c3_u), &c3_d0, 1, 0, 0U, 0, 0U, 0);
  c3_y = c3_d0;
  sf_mex_destroy(&c3_u);
  return c3_y;
}

static void c3_b_sf_marshallIn(void *chartInstanceVoid, const mxArray
  *c3_mxArrayInData, const char_T *c3_varName, void *c3_outData)
{
  const mxArray *c3_nargout;
  const char_T *c3_identifier;
  emlrtMsgIdentifier c3_thisId;
  real_T c3_y;
  SFc3_Robot_Control_v01InstanceStruct *chartInstance;
  chartInstance = (SFc3_Robot_Control_v01InstanceStruct *)chartInstanceVoid;
  c3_nargout = sf_mex_dup(c3_mxArrayInData);
  c3_identifier = c3_varName;
  c3_thisId.fIdentifier = c3_identifier;
  c3_thisId.fParent = NULL;
  c3_y = c3_e_emlrt_marshallIn(chartInstance, sf_mex_dup(c3_nargout), &c3_thisId);
  sf_mex_destroy(&c3_nargout);
  *(real_T *)c3_outData = c3_y;
  sf_mex_destroy(&c3_mxArrayInData);
}

static const mxArray *c3_c_sf_marshallOut(void *chartInstanceVoid, void
  *c3_inData)
{
  const mxArray *c3_mxArrayOutData = NULL;
  int32_T c3_i216;
  int32_T c3_i217;
  int32_T c3_i218;
  real_T c3_b_inData[16];
  int32_T c3_i219;
  int32_T c3_i220;
  int32_T c3_i221;
  real_T c3_u[16];
  const mxArray *c3_y = NULL;
  SFc3_Robot_Control_v01InstanceStruct *chartInstance;
  chartInstance = (SFc3_Robot_Control_v01InstanceStruct *)chartInstanceVoid;
  c3_mxArrayOutData = NULL;
  c3_i216 = 0;
  for (c3_i217 = 0; c3_i217 < 4; c3_i217++) {
    for (c3_i218 = 0; c3_i218 < 4; c3_i218++) {
      c3_b_inData[c3_i218 + c3_i216] = (*(real_T (*)[16])c3_inData)[c3_i218 +
        c3_i216];
    }

    c3_i216 += 4;
  }

  c3_i219 = 0;
  for (c3_i220 = 0; c3_i220 < 4; c3_i220++) {
    for (c3_i221 = 0; c3_i221 < 4; c3_i221++) {
      c3_u[c3_i221 + c3_i219] = c3_b_inData[c3_i221 + c3_i219];
    }

    c3_i219 += 4;
  }

  c3_y = NULL;
  sf_mex_assign(&c3_y, sf_mex_create("y", c3_u, 0, 0U, 1U, 0U, 2, 4, 4), false);
  sf_mex_assign(&c3_mxArrayOutData, c3_y, false);
  return c3_mxArrayOutData;
}

static void c3_f_emlrt_marshallIn(SFc3_Robot_Control_v01InstanceStruct
  *chartInstance, const mxArray *c3_u, const emlrtMsgIdentifier *c3_parentId,
  real_T c3_y[16])
{
  real_T c3_dv20[16];
  int32_T c3_i222;
  (void)chartInstance;
  sf_mex_import(c3_parentId, sf_mex_dup(c3_u), c3_dv20, 1, 0, 0U, 1, 0U, 2, 4, 4);
  for (c3_i222 = 0; c3_i222 < 16; c3_i222++) {
    c3_y[c3_i222] = c3_dv20[c3_i222];
  }

  sf_mex_destroy(&c3_u);
}

static void c3_c_sf_marshallIn(void *chartInstanceVoid, const mxArray
  *c3_mxArrayInData, const char_T *c3_varName, void *c3_outData)
{
  const mxArray *c3_world2robot;
  const char_T *c3_identifier;
  emlrtMsgIdentifier c3_thisId;
  real_T c3_y[16];
  int32_T c3_i223;
  int32_T c3_i224;
  int32_T c3_i225;
  SFc3_Robot_Control_v01InstanceStruct *chartInstance;
  chartInstance = (SFc3_Robot_Control_v01InstanceStruct *)chartInstanceVoid;
  c3_world2robot = sf_mex_dup(c3_mxArrayInData);
  c3_identifier = c3_varName;
  c3_thisId.fIdentifier = c3_identifier;
  c3_thisId.fParent = NULL;
  c3_f_emlrt_marshallIn(chartInstance, sf_mex_dup(c3_world2robot), &c3_thisId,
                        c3_y);
  sf_mex_destroy(&c3_world2robot);
  c3_i223 = 0;
  for (c3_i224 = 0; c3_i224 < 4; c3_i224++) {
    for (c3_i225 = 0; c3_i225 < 4; c3_i225++) {
      (*(real_T (*)[16])c3_outData)[c3_i225 + c3_i223] = c3_y[c3_i225 + c3_i223];
    }

    c3_i223 += 4;
  }

  sf_mex_destroy(&c3_mxArrayInData);
}

static const mxArray *c3_d_sf_marshallOut(void *chartInstanceVoid, void
  *c3_inData)
{
  const mxArray *c3_mxArrayOutData = NULL;
  int32_T c3_i226;
  real_T c3_b_inData[12];
  int32_T c3_i227;
  real_T c3_u[12];
  const mxArray *c3_y = NULL;
  SFc3_Robot_Control_v01InstanceStruct *chartInstance;
  chartInstance = (SFc3_Robot_Control_v01InstanceStruct *)chartInstanceVoid;
  c3_mxArrayOutData = NULL;
  for (c3_i226 = 0; c3_i226 < 12; c3_i226++) {
    c3_b_inData[c3_i226] = (*(real_T (*)[12])c3_inData)[c3_i226];
  }

  for (c3_i227 = 0; c3_i227 < 12; c3_i227++) {
    c3_u[c3_i227] = c3_b_inData[c3_i227];
  }

  c3_y = NULL;
  sf_mex_assign(&c3_y, sf_mex_create("y", c3_u, 0, 0U, 1U, 0U, 2, 1, 12), false);
  sf_mex_assign(&c3_mxArrayOutData, c3_y, false);
  return c3_mxArrayOutData;
}

static void c3_g_emlrt_marshallIn(SFc3_Robot_Control_v01InstanceStruct
  *chartInstance, const mxArray *c3_u, const emlrtMsgIdentifier *c3_parentId,
  real_T c3_y[12])
{
  real_T c3_dv21[12];
  int32_T c3_i228;
  (void)chartInstance;
  sf_mex_import(c3_parentId, sf_mex_dup(c3_u), c3_dv21, 1, 0, 0U, 1, 0U, 2, 1,
                12);
  for (c3_i228 = 0; c3_i228 < 12; c3_i228++) {
    c3_y[c3_i228] = c3_dv21[c3_i228];
  }

  sf_mex_destroy(&c3_u);
}

static void c3_d_sf_marshallIn(void *chartInstanceVoid, const mxArray
  *c3_mxArrayInData, const char_T *c3_varName, void *c3_outData)
{
  const mxArray *c3_param;
  const char_T *c3_identifier;
  emlrtMsgIdentifier c3_thisId;
  real_T c3_y[12];
  int32_T c3_i229;
  SFc3_Robot_Control_v01InstanceStruct *chartInstance;
  chartInstance = (SFc3_Robot_Control_v01InstanceStruct *)chartInstanceVoid;
  c3_param = sf_mex_dup(c3_mxArrayInData);
  c3_identifier = c3_varName;
  c3_thisId.fIdentifier = c3_identifier;
  c3_thisId.fParent = NULL;
  c3_g_emlrt_marshallIn(chartInstance, sf_mex_dup(c3_param), &c3_thisId, c3_y);
  sf_mex_destroy(&c3_param);
  for (c3_i229 = 0; c3_i229 < 12; c3_i229++) {
    (*(real_T (*)[12])c3_outData)[c3_i229] = c3_y[c3_i229];
  }

  sf_mex_destroy(&c3_mxArrayInData);
}

static const mxArray *c3_e_sf_marshallOut(void *chartInstanceVoid, void
  *c3_inData)
{
  const mxArray *c3_mxArrayOutData;
  c3_smiE7r3wwCYeiytRpYXSxxE c3_u;
  const mxArray *c3_y = NULL;
  c3_sKe46gf3wfNeOrzL8yzt6nF c3_b_u;
  const mxArray *c3_b_y = NULL;
  real_T c3_c_u;
  const mxArray *c3_c_y = NULL;
  real_T c3_d_u;
  const mxArray *c3_d_y = NULL;
  real_T c3_e_u;
  const mxArray *c3_e_y = NULL;
  real_T c3_f_u;
  const mxArray *c3_f_y = NULL;
  real_T c3_g_u;
  const mxArray *c3_g_y = NULL;
  real_T c3_h_u;
  const mxArray *c3_h_y = NULL;
  c3_s6d9wPYgmS3cwGRsMJd1GAE c3_i_u;
  const mxArray *c3_i_y = NULL;
  const mxArray *c3_j_y = NULL;
  real_T c3_j_u;
  const mxArray *c3_k_y = NULL;
  real_T c3_k_u;
  const mxArray *c3_l_y = NULL;
  real_T c3_l_u;
  const mxArray *c3_m_y = NULL;
  real_T c3_m_u;
  const mxArray *c3_n_y = NULL;
  real_T c3_n_u;
  const mxArray *c3_o_y = NULL;
  real_T c3_o_u;
  const mxArray *c3_p_y = NULL;
  real_T c3_p_u;
  const mxArray *c3_q_y = NULL;
  real_T c3_q_u;
  const mxArray *c3_r_y = NULL;
  real_T c3_r_u;
  const mxArray *c3_s_y = NULL;
  real_T c3_s_u;
  const mxArray *c3_t_y = NULL;
  real_T c3_t_u;
  const mxArray *c3_u_y = NULL;
  real_T c3_u_u;
  const mxArray *c3_v_y = NULL;
  const mxArray *c3_w_y = NULL;
  const mxArray *c3_x_y = NULL;
  real_T c3_v_u;
  const mxArray *c3_y_y = NULL;
  real_T c3_w_u;
  const mxArray *c3_ab_y = NULL;
  real_T c3_x_u;
  const mxArray *c3_bb_y = NULL;
  real_T c3_y_u;
  const mxArray *c3_cb_y = NULL;
  real_T c3_ab_u;
  const mxArray *c3_db_y = NULL;
  real_T c3_bb_u;
  const mxArray *c3_eb_y = NULL;
  real_T c3_cb_u;
  const mxArray *c3_fb_y = NULL;
  real_T c3_db_u;
  const mxArray *c3_gb_y = NULL;
  real_T c3_eb_u;
  const mxArray *c3_hb_y = NULL;
  real_T c3_fb_u;
  const mxArray *c3_ib_y = NULL;
  real_T c3_gb_u;
  const mxArray *c3_jb_y = NULL;
  real_T c3_hb_u;
  const mxArray *c3_kb_y = NULL;
  SFc3_Robot_Control_v01InstanceStruct *chartInstance;
  chartInstance = (SFc3_Robot_Control_v01InstanceStruct *)chartInstanceVoid;
  c3_mxArrayOutData = NULL;
  c3_mxArrayOutData = NULL;
  c3_u = *(c3_smiE7r3wwCYeiytRpYXSxxE *)c3_inData;
  c3_y = NULL;
  sf_mex_assign(&c3_y, sf_mex_createstruct("structure", 2, 1, 1), false);
  c3_b_u = c3_u.pose;
  c3_b_y = NULL;
  sf_mex_assign(&c3_b_y, sf_mex_createstruct("structure", 2, 1, 1), false);
  c3_c_u = c3_b_u.tx;
  c3_c_y = NULL;
  sf_mex_assign(&c3_c_y, sf_mex_create("y", &c3_c_u, 0, 0U, 0U, 0U, 0), false);
  sf_mex_addfield(c3_b_y, c3_c_y, "tx", "tx", 0);
  c3_d_u = c3_b_u.ty;
  c3_d_y = NULL;
  sf_mex_assign(&c3_d_y, sf_mex_create("y", &c3_d_u, 0, 0U, 0U, 0U, 0), false);
  sf_mex_addfield(c3_b_y, c3_d_y, "ty", "ty", 0);
  c3_e_u = c3_b_u.tz;
  c3_e_y = NULL;
  sf_mex_assign(&c3_e_y, sf_mex_create("y", &c3_e_u, 0, 0U, 0U, 0U, 0), false);
  sf_mex_addfield(c3_b_y, c3_e_y, "tz", "tz", 0);
  c3_f_u = c3_b_u.phi;
  c3_f_y = NULL;
  sf_mex_assign(&c3_f_y, sf_mex_create("y", &c3_f_u, 0, 0U, 0U, 0U, 0), false);
  sf_mex_addfield(c3_b_y, c3_f_y, "phi", "phi", 0);
  c3_g_u = c3_b_u.theta;
  c3_g_y = NULL;
  sf_mex_assign(&c3_g_y, sf_mex_create("y", &c3_g_u, 0, 0U, 0U, 0U, 0), false);
  sf_mex_addfield(c3_b_y, c3_g_y, "theta", "theta", 0);
  c3_h_u = c3_b_u.psi;
  c3_h_y = NULL;
  sf_mex_assign(&c3_h_y, sf_mex_create("y", &c3_h_u, 0, 0U, 0U, 0U, 0), false);
  sf_mex_addfield(c3_b_y, c3_h_y, "psi", "psi", 0);
  sf_mex_addfield(c3_y, c3_b_y, "pose", "pose", 0);
  c3_i_u = c3_u.leftCamera;
  c3_i_y = NULL;
  sf_mex_assign(&c3_i_y, sf_mex_createstruct("structure", 2, 1, 1), false);
  c3_b_u = c3_i_u.pose;
  c3_j_y = NULL;
  sf_mex_assign(&c3_j_y, sf_mex_createstruct("structure", 2, 1, 1), false);
  c3_j_u = c3_b_u.tx;
  c3_k_y = NULL;
  sf_mex_assign(&c3_k_y, sf_mex_create("y", &c3_j_u, 0, 0U, 0U, 0U, 0), false);
  sf_mex_addfield(c3_j_y, c3_k_y, "tx", "tx", 0);
  c3_k_u = c3_b_u.ty;
  c3_l_y = NULL;
  sf_mex_assign(&c3_l_y, sf_mex_create("y", &c3_k_u, 0, 0U, 0U, 0U, 0), false);
  sf_mex_addfield(c3_j_y, c3_l_y, "ty", "ty", 0);
  c3_l_u = c3_b_u.tz;
  c3_m_y = NULL;
  sf_mex_assign(&c3_m_y, sf_mex_create("y", &c3_l_u, 0, 0U, 0U, 0U, 0), false);
  sf_mex_addfield(c3_j_y, c3_m_y, "tz", "tz", 0);
  c3_m_u = c3_b_u.phi;
  c3_n_y = NULL;
  sf_mex_assign(&c3_n_y, sf_mex_create("y", &c3_m_u, 0, 0U, 0U, 0U, 0), false);
  sf_mex_addfield(c3_j_y, c3_n_y, "phi", "phi", 0);
  c3_n_u = c3_b_u.theta;
  c3_o_y = NULL;
  sf_mex_assign(&c3_o_y, sf_mex_create("y", &c3_n_u, 0, 0U, 0U, 0U, 0), false);
  sf_mex_addfield(c3_j_y, c3_o_y, "theta", "theta", 0);
  c3_o_u = c3_b_u.psi;
  c3_p_y = NULL;
  sf_mex_assign(&c3_p_y, sf_mex_create("y", &c3_o_u, 0, 0U, 0U, 0U, 0), false);
  sf_mex_addfield(c3_j_y, c3_p_y, "psi", "psi", 0);
  sf_mex_addfield(c3_i_y, c3_j_y, "pose", "pose", 0);
  c3_p_u = c3_i_u.fx;
  c3_q_y = NULL;
  sf_mex_assign(&c3_q_y, sf_mex_create("y", &c3_p_u, 0, 0U, 0U, 0U, 0), false);
  sf_mex_addfield(c3_i_y, c3_q_y, "fx", "fx", 0);
  c3_q_u = c3_i_u.fy;
  c3_r_y = NULL;
  sf_mex_assign(&c3_r_y, sf_mex_create("y", &c3_q_u, 0, 0U, 0U, 0U, 0), false);
  sf_mex_addfield(c3_i_y, c3_r_y, "fy", "fy", 0);
  c3_r_u = c3_i_u.cx;
  c3_s_y = NULL;
  sf_mex_assign(&c3_s_y, sf_mex_create("y", &c3_r_u, 0, 0U, 0U, 0U, 0), false);
  sf_mex_addfield(c3_i_y, c3_s_y, "cx", "cx", 0);
  c3_s_u = c3_i_u.cy;
  c3_t_y = NULL;
  sf_mex_assign(&c3_t_y, sf_mex_create("y", &c3_s_u, 0, 0U, 0U, 0U, 0), false);
  sf_mex_addfield(c3_i_y, c3_t_y, "cy", "cy", 0);
  c3_t_u = c3_i_u.s;
  c3_u_y = NULL;
  sf_mex_assign(&c3_u_y, sf_mex_create("y", &c3_t_u, 0, 0U, 0U, 0U, 0), false);
  sf_mex_addfield(c3_i_y, c3_u_y, "s", "s", 0);
  c3_u_u = c3_i_u.d;
  c3_v_y = NULL;
  sf_mex_assign(&c3_v_y, sf_mex_create("y", &c3_u_u, 0, 0U, 0U, 0U, 0), false);
  sf_mex_addfield(c3_i_y, c3_v_y, "d", "d", 0);
  sf_mex_addfield(c3_y, c3_i_y, "leftCamera", "leftCamera", 0);
  c3_i_u = c3_u.rightCamera;
  c3_w_y = NULL;
  sf_mex_assign(&c3_w_y, sf_mex_createstruct("structure", 2, 1, 1), false);
  c3_b_u = c3_i_u.pose;
  c3_x_y = NULL;
  sf_mex_assign(&c3_x_y, sf_mex_createstruct("structure", 2, 1, 1), false);
  c3_v_u = c3_b_u.tx;
  c3_y_y = NULL;
  sf_mex_assign(&c3_y_y, sf_mex_create("y", &c3_v_u, 0, 0U, 0U, 0U, 0), false);
  sf_mex_addfield(c3_x_y, c3_y_y, "tx", "tx", 0);
  c3_w_u = c3_b_u.ty;
  c3_ab_y = NULL;
  sf_mex_assign(&c3_ab_y, sf_mex_create("y", &c3_w_u, 0, 0U, 0U, 0U, 0), false);
  sf_mex_addfield(c3_x_y, c3_ab_y, "ty", "ty", 0);
  c3_x_u = c3_b_u.tz;
  c3_bb_y = NULL;
  sf_mex_assign(&c3_bb_y, sf_mex_create("y", &c3_x_u, 0, 0U, 0U, 0U, 0), false);
  sf_mex_addfield(c3_x_y, c3_bb_y, "tz", "tz", 0);
  c3_y_u = c3_b_u.phi;
  c3_cb_y = NULL;
  sf_mex_assign(&c3_cb_y, sf_mex_create("y", &c3_y_u, 0, 0U, 0U, 0U, 0), false);
  sf_mex_addfield(c3_x_y, c3_cb_y, "phi", "phi", 0);
  c3_ab_u = c3_b_u.theta;
  c3_db_y = NULL;
  sf_mex_assign(&c3_db_y, sf_mex_create("y", &c3_ab_u, 0, 0U, 0U, 0U, 0), false);
  sf_mex_addfield(c3_x_y, c3_db_y, "theta", "theta", 0);
  c3_bb_u = c3_b_u.psi;
  c3_eb_y = NULL;
  sf_mex_assign(&c3_eb_y, sf_mex_create("y", &c3_bb_u, 0, 0U, 0U, 0U, 0), false);
  sf_mex_addfield(c3_x_y, c3_eb_y, "psi", "psi", 0);
  sf_mex_addfield(c3_w_y, c3_x_y, "pose", "pose", 0);
  c3_cb_u = c3_i_u.fx;
  c3_fb_y = NULL;
  sf_mex_assign(&c3_fb_y, sf_mex_create("y", &c3_cb_u, 0, 0U, 0U, 0U, 0), false);
  sf_mex_addfield(c3_w_y, c3_fb_y, "fx", "fx", 0);
  c3_db_u = c3_i_u.fy;
  c3_gb_y = NULL;
  sf_mex_assign(&c3_gb_y, sf_mex_create("y", &c3_db_u, 0, 0U, 0U, 0U, 0), false);
  sf_mex_addfield(c3_w_y, c3_gb_y, "fy", "fy", 0);
  c3_eb_u = c3_i_u.cx;
  c3_hb_y = NULL;
  sf_mex_assign(&c3_hb_y, sf_mex_create("y", &c3_eb_u, 0, 0U, 0U, 0U, 0), false);
  sf_mex_addfield(c3_w_y, c3_hb_y, "cx", "cx", 0);
  c3_fb_u = c3_i_u.cy;
  c3_ib_y = NULL;
  sf_mex_assign(&c3_ib_y, sf_mex_create("y", &c3_fb_u, 0, 0U, 0U, 0U, 0), false);
  sf_mex_addfield(c3_w_y, c3_ib_y, "cy", "cy", 0);
  c3_gb_u = c3_i_u.s;
  c3_jb_y = NULL;
  sf_mex_assign(&c3_jb_y, sf_mex_create("y", &c3_gb_u, 0, 0U, 0U, 0U, 0), false);
  sf_mex_addfield(c3_w_y, c3_jb_y, "s", "s", 0);
  c3_hb_u = c3_i_u.d;
  c3_kb_y = NULL;
  sf_mex_assign(&c3_kb_y, sf_mex_create("y", &c3_hb_u, 0, 0U, 0U, 0U, 0), false);
  sf_mex_addfield(c3_w_y, c3_kb_y, "d", "d", 0);
  sf_mex_addfield(c3_y, c3_w_y, "rightCamera", "rightCamera", 0);
  sf_mex_assign(&c3_mxArrayOutData, c3_y, false);
  return c3_mxArrayOutData;
}

static void c3_h_emlrt_marshallIn(SFc3_Robot_Control_v01InstanceStruct
  *chartInstance, const mxArray *c3_u, const emlrtMsgIdentifier *c3_parentId,
  c3_smiE7r3wwCYeiytRpYXSxxE *c3_y)
{
  emlrtMsgIdentifier c3_thisId;
  static const char * c3_fieldNames[3] = { "pose", "leftCamera", "rightCamera" };

  c3_thisId.fParent = c3_parentId;
  sf_mex_check_struct(c3_parentId, c3_u, 3, c3_fieldNames, 0U, NULL);
  c3_thisId.fIdentifier = "pose";
  c3_y->pose = c3_i_emlrt_marshallIn(chartInstance, sf_mex_dup(sf_mex_getfield
    (c3_u, "pose", "pose", 0)), &c3_thisId);
  c3_thisId.fIdentifier = "leftCamera";
  c3_j_emlrt_marshallIn(chartInstance, sf_mex_dup(sf_mex_getfield(c3_u,
    "leftCamera", "leftCamera", 0)), &c3_thisId, &c3_y->leftCamera);
  c3_thisId.fIdentifier = "rightCamera";
  c3_j_emlrt_marshallIn(chartInstance, sf_mex_dup(sf_mex_getfield(c3_u,
    "rightCamera", "rightCamera", 0)), &c3_thisId, &c3_y->rightCamera);
  sf_mex_destroy(&c3_u);
}

static c3_sKe46gf3wfNeOrzL8yzt6nF c3_i_emlrt_marshallIn
  (SFc3_Robot_Control_v01InstanceStruct *chartInstance, const mxArray *c3_u,
   const emlrtMsgIdentifier *c3_parentId)
{
  c3_sKe46gf3wfNeOrzL8yzt6nF c3_y;
  emlrtMsgIdentifier c3_thisId;
  static const char * c3_fieldNames[6] = { "tx", "ty", "tz", "phi", "theta",
    "psi" };

  c3_thisId.fParent = c3_parentId;
  sf_mex_check_struct(c3_parentId, c3_u, 6, c3_fieldNames, 0U, NULL);
  c3_thisId.fIdentifier = "tx";
  c3_y.tx = c3_e_emlrt_marshallIn(chartInstance, sf_mex_dup(sf_mex_getfield(c3_u,
    "tx", "tx", 0)), &c3_thisId);
  c3_thisId.fIdentifier = "ty";
  c3_y.ty = c3_e_emlrt_marshallIn(chartInstance, sf_mex_dup(sf_mex_getfield(c3_u,
    "ty", "ty", 0)), &c3_thisId);
  c3_thisId.fIdentifier = "tz";
  c3_y.tz = c3_e_emlrt_marshallIn(chartInstance, sf_mex_dup(sf_mex_getfield(c3_u,
    "tz", "tz", 0)), &c3_thisId);
  c3_thisId.fIdentifier = "phi";
  c3_y.phi = c3_e_emlrt_marshallIn(chartInstance, sf_mex_dup(sf_mex_getfield
    (c3_u, "phi", "phi", 0)), &c3_thisId);
  c3_thisId.fIdentifier = "theta";
  c3_y.theta = c3_e_emlrt_marshallIn(chartInstance, sf_mex_dup(sf_mex_getfield
    (c3_u, "theta", "theta", 0)), &c3_thisId);
  c3_thisId.fIdentifier = "psi";
  c3_y.psi = c3_e_emlrt_marshallIn(chartInstance, sf_mex_dup(sf_mex_getfield
    (c3_u, "psi", "psi", 0)), &c3_thisId);
  sf_mex_destroy(&c3_u);
  return c3_y;
}

static void c3_j_emlrt_marshallIn(SFc3_Robot_Control_v01InstanceStruct
  *chartInstance, const mxArray *c3_u, const emlrtMsgIdentifier *c3_parentId,
  c3_s6d9wPYgmS3cwGRsMJd1GAE *c3_y)
{
  emlrtMsgIdentifier c3_thisId;
  static const char * c3_fieldNames[7] = { "pose", "fx", "fy", "cx", "cy", "s",
    "d" };

  c3_thisId.fParent = c3_parentId;
  sf_mex_check_struct(c3_parentId, c3_u, 7, c3_fieldNames, 0U, NULL);
  c3_thisId.fIdentifier = "pose";
  c3_y->pose = c3_i_emlrt_marshallIn(chartInstance, sf_mex_dup(sf_mex_getfield
    (c3_u, "pose", "pose", 0)), &c3_thisId);
  c3_thisId.fIdentifier = "fx";
  c3_y->fx = c3_e_emlrt_marshallIn(chartInstance, sf_mex_dup(sf_mex_getfield
    (c3_u, "fx", "fx", 0)), &c3_thisId);
  c3_thisId.fIdentifier = "fy";
  c3_y->fy = c3_e_emlrt_marshallIn(chartInstance, sf_mex_dup(sf_mex_getfield
    (c3_u, "fy", "fy", 0)), &c3_thisId);
  c3_thisId.fIdentifier = "cx";
  c3_y->cx = c3_e_emlrt_marshallIn(chartInstance, sf_mex_dup(sf_mex_getfield
    (c3_u, "cx", "cx", 0)), &c3_thisId);
  c3_thisId.fIdentifier = "cy";
  c3_y->cy = c3_e_emlrt_marshallIn(chartInstance, sf_mex_dup(sf_mex_getfield
    (c3_u, "cy", "cy", 0)), &c3_thisId);
  c3_thisId.fIdentifier = "s";
  c3_y->s = c3_e_emlrt_marshallIn(chartInstance, sf_mex_dup(sf_mex_getfield(c3_u,
    "s", "s", 0)), &c3_thisId);
  c3_thisId.fIdentifier = "d";
  c3_y->d = c3_e_emlrt_marshallIn(chartInstance, sf_mex_dup(sf_mex_getfield(c3_u,
    "d", "d", 0)), &c3_thisId);
  sf_mex_destroy(&c3_u);
}

static void c3_e_sf_marshallIn(void *chartInstanceVoid, const mxArray
  *c3_mxArrayInData, const char_T *c3_varName, void *c3_outData)
{
  const mxArray *c3_robot;
  const char_T *c3_identifier;
  emlrtMsgIdentifier c3_thisId;
  c3_smiE7r3wwCYeiytRpYXSxxE c3_y;
  SFc3_Robot_Control_v01InstanceStruct *chartInstance;
  chartInstance = (SFc3_Robot_Control_v01InstanceStruct *)chartInstanceVoid;
  c3_robot = sf_mex_dup(c3_mxArrayInData);
  c3_identifier = c3_varName;
  c3_thisId.fIdentifier = c3_identifier;
  c3_thisId.fParent = NULL;
  c3_h_emlrt_marshallIn(chartInstance, sf_mex_dup(c3_robot), &c3_thisId, &c3_y);
  sf_mex_destroy(&c3_robot);
  *(c3_smiE7r3wwCYeiytRpYXSxxE *)c3_outData = c3_y;
  sf_mex_destroy(&c3_mxArrayInData);
}

static const mxArray *c3_f_sf_marshallOut(void *chartInstanceVoid, void
  *c3_inData)
{
  const mxArray *c3_mxArrayOutData = NULL;
  int32_T c3_i230;
  int32_T c3_i231;
  int32_T c3_i232;
  real_T c3_b_inData[8];
  int32_T c3_i233;
  int32_T c3_i234;
  int32_T c3_i235;
  real_T c3_u[8];
  const mxArray *c3_y = NULL;
  SFc3_Robot_Control_v01InstanceStruct *chartInstance;
  chartInstance = (SFc3_Robot_Control_v01InstanceStruct *)chartInstanceVoid;
  c3_mxArrayOutData = NULL;
  c3_i230 = 0;
  for (c3_i231 = 0; c3_i231 < 4; c3_i231++) {
    for (c3_i232 = 0; c3_i232 < 2; c3_i232++) {
      c3_b_inData[c3_i232 + c3_i230] = (*(real_T (*)[8])c3_inData)[c3_i232 +
        c3_i230];
    }

    c3_i230 += 2;
  }

  c3_i233 = 0;
  for (c3_i234 = 0; c3_i234 < 4; c3_i234++) {
    for (c3_i235 = 0; c3_i235 < 2; c3_i235++) {
      c3_u[c3_i235 + c3_i233] = c3_b_inData[c3_i235 + c3_i233];
    }

    c3_i233 += 2;
  }

  c3_y = NULL;
  sf_mex_assign(&c3_y, sf_mex_create("y", c3_u, 0, 0U, 1U, 0U, 2, 2, 4), false);
  sf_mex_assign(&c3_mxArrayOutData, c3_y, false);
  return c3_mxArrayOutData;
}

static void c3_k_emlrt_marshallIn(SFc3_Robot_Control_v01InstanceStruct
  *chartInstance, const mxArray *c3_u, const emlrtMsgIdentifier *c3_parentId,
  real_T c3_y[8])
{
  real_T c3_dv22[8];
  int32_T c3_i236;
  (void)chartInstance;
  sf_mex_import(c3_parentId, sf_mex_dup(c3_u), c3_dv22, 1, 0, 0U, 1, 0U, 2, 2, 4);
  for (c3_i236 = 0; c3_i236 < 8; c3_i236++) {
    c3_y[c3_i236] = c3_dv22[c3_i236];
  }

  sf_mex_destroy(&c3_u);
}

static void c3_f_sf_marshallIn(void *chartInstanceVoid, const mxArray
  *c3_mxArrayInData, const char_T *c3_varName, void *c3_outData)
{
  const mxArray *c3_R_p;
  const char_T *c3_identifier;
  emlrtMsgIdentifier c3_thisId;
  real_T c3_y[8];
  int32_T c3_i237;
  int32_T c3_i238;
  int32_T c3_i239;
  SFc3_Robot_Control_v01InstanceStruct *chartInstance;
  chartInstance = (SFc3_Robot_Control_v01InstanceStruct *)chartInstanceVoid;
  c3_R_p = sf_mex_dup(c3_mxArrayInData);
  c3_identifier = c3_varName;
  c3_thisId.fIdentifier = c3_identifier;
  c3_thisId.fParent = NULL;
  c3_k_emlrt_marshallIn(chartInstance, sf_mex_dup(c3_R_p), &c3_thisId, c3_y);
  sf_mex_destroy(&c3_R_p);
  c3_i237 = 0;
  for (c3_i238 = 0; c3_i238 < 4; c3_i238++) {
    for (c3_i239 = 0; c3_i239 < 2; c3_i239++) {
      (*(real_T (*)[8])c3_outData)[c3_i239 + c3_i237] = c3_y[c3_i239 + c3_i237];
    }

    c3_i237 += 2;
  }

  sf_mex_destroy(&c3_mxArrayInData);
}

static const mxArray *c3_g_sf_marshallOut(void *chartInstanceVoid, void
  *c3_inData)
{
  const mxArray *c3_mxArrayOutData = NULL;
  c3_sKe46gf3wfNeOrzL8yzt6nF c3_u;
  const mxArray *c3_y = NULL;
  real_T c3_b_u;
  const mxArray *c3_b_y = NULL;
  real_T c3_c_u;
  const mxArray *c3_c_y = NULL;
  real_T c3_d_u;
  const mxArray *c3_d_y = NULL;
  real_T c3_e_u;
  const mxArray *c3_e_y = NULL;
  real_T c3_f_u;
  const mxArray *c3_f_y = NULL;
  real_T c3_g_u;
  const mxArray *c3_g_y = NULL;
  SFc3_Robot_Control_v01InstanceStruct *chartInstance;
  chartInstance = (SFc3_Robot_Control_v01InstanceStruct *)chartInstanceVoid;
  c3_mxArrayOutData = NULL;
  c3_u = *(c3_sKe46gf3wfNeOrzL8yzt6nF *)c3_inData;
  c3_y = NULL;
  sf_mex_assign(&c3_y, sf_mex_createstruct("structure", 2, 1, 1), false);
  c3_b_u = c3_u.tx;
  c3_b_y = NULL;
  sf_mex_assign(&c3_b_y, sf_mex_create("y", &c3_b_u, 0, 0U, 0U, 0U, 0), false);
  sf_mex_addfield(c3_y, c3_b_y, "tx", "tx", 0);
  c3_c_u = c3_u.ty;
  c3_c_y = NULL;
  sf_mex_assign(&c3_c_y, sf_mex_create("y", &c3_c_u, 0, 0U, 0U, 0U, 0), false);
  sf_mex_addfield(c3_y, c3_c_y, "ty", "ty", 0);
  c3_d_u = c3_u.tz;
  c3_d_y = NULL;
  sf_mex_assign(&c3_d_y, sf_mex_create("y", &c3_d_u, 0, 0U, 0U, 0U, 0), false);
  sf_mex_addfield(c3_y, c3_d_y, "tz", "tz", 0);
  c3_e_u = c3_u.phi;
  c3_e_y = NULL;
  sf_mex_assign(&c3_e_y, sf_mex_create("y", &c3_e_u, 0, 0U, 0U, 0U, 0), false);
  sf_mex_addfield(c3_y, c3_e_y, "phi", "phi", 0);
  c3_f_u = c3_u.theta;
  c3_f_y = NULL;
  sf_mex_assign(&c3_f_y, sf_mex_create("y", &c3_f_u, 0, 0U, 0U, 0U, 0), false);
  sf_mex_addfield(c3_y, c3_f_y, "theta", "theta", 0);
  c3_g_u = c3_u.psi;
  c3_g_y = NULL;
  sf_mex_assign(&c3_g_y, sf_mex_create("y", &c3_g_u, 0, 0U, 0U, 0U, 0), false);
  sf_mex_addfield(c3_y, c3_g_y, "psi", "psi", 0);
  sf_mex_assign(&c3_mxArrayOutData, c3_y, false);
  return c3_mxArrayOutData;
}

static void c3_g_sf_marshallIn(void *chartInstanceVoid, const mxArray
  *c3_mxArrayInData, const char_T *c3_varName, void *c3_outData)
{
  const mxArray *c3_robotPoseDes;
  const char_T *c3_identifier;
  emlrtMsgIdentifier c3_thisId;
  c3_sKe46gf3wfNeOrzL8yzt6nF c3_y;
  SFc3_Robot_Control_v01InstanceStruct *chartInstance;
  chartInstance = (SFc3_Robot_Control_v01InstanceStruct *)chartInstanceVoid;
  c3_robotPoseDes = sf_mex_dup(c3_mxArrayInData);
  c3_identifier = c3_varName;
  c3_thisId.fIdentifier = c3_identifier;
  c3_thisId.fParent = NULL;
  c3_y = c3_i_emlrt_marshallIn(chartInstance, sf_mex_dup(c3_robotPoseDes),
    &c3_thisId);
  sf_mex_destroy(&c3_robotPoseDes);
  *(c3_sKe46gf3wfNeOrzL8yzt6nF *)c3_outData = c3_y;
  sf_mex_destroy(&c3_mxArrayInData);
}

static const mxArray *c3_h_sf_marshallOut(void *chartInstanceVoid, void
  *c3_inData)
{
  const mxArray *c3_mxArrayOutData = NULL;
  int32_T c3_i240;
  real_T c3_b_inData[4];
  int32_T c3_i241;
  real_T c3_u[4];
  const mxArray *c3_y = NULL;
  SFc3_Robot_Control_v01InstanceStruct *chartInstance;
  chartInstance = (SFc3_Robot_Control_v01InstanceStruct *)chartInstanceVoid;
  c3_mxArrayOutData = NULL;
  for (c3_i240 = 0; c3_i240 < 4; c3_i240++) {
    c3_b_inData[c3_i240] = (*(real_T (*)[4])c3_inData)[c3_i240];
  }

  for (c3_i241 = 0; c3_i241 < 4; c3_i241++) {
    c3_u[c3_i241] = c3_b_inData[c3_i241];
  }

  c3_y = NULL;
  sf_mex_assign(&c3_y, sf_mex_create("y", c3_u, 0, 0U, 1U, 0U, 1, 4), false);
  sf_mex_assign(&c3_mxArrayOutData, c3_y, false);
  return c3_mxArrayOutData;
}

static void c3_l_emlrt_marshallIn(SFc3_Robot_Control_v01InstanceStruct
  *chartInstance, const mxArray *c3_u, const emlrtMsgIdentifier *c3_parentId,
  real_T c3_y[4])
{
  real_T c3_dv23[4];
  int32_T c3_i242;
  (void)chartInstance;
  sf_mex_import(c3_parentId, sf_mex_dup(c3_u), c3_dv23, 1, 0, 0U, 1, 0U, 1, 4);
  for (c3_i242 = 0; c3_i242 < 4; c3_i242++) {
    c3_y[c3_i242] = c3_dv23[c3_i242];
  }

  sf_mex_destroy(&c3_u);
}

static void c3_h_sf_marshallIn(void *chartInstanceVoid, const mxArray
  *c3_mxArrayInData, const char_T *c3_varName, void *c3_outData)
{
  const mxArray *c3_noise;
  const char_T *c3_identifier;
  emlrtMsgIdentifier c3_thisId;
  real_T c3_y[4];
  int32_T c3_i243;
  SFc3_Robot_Control_v01InstanceStruct *chartInstance;
  chartInstance = (SFc3_Robot_Control_v01InstanceStruct *)chartInstanceVoid;
  c3_noise = sf_mex_dup(c3_mxArrayInData);
  c3_identifier = c3_varName;
  c3_thisId.fIdentifier = c3_identifier;
  c3_thisId.fParent = NULL;
  c3_l_emlrt_marshallIn(chartInstance, sf_mex_dup(c3_noise), &c3_thisId, c3_y);
  sf_mex_destroy(&c3_noise);
  for (c3_i243 = 0; c3_i243 < 4; c3_i243++) {
    (*(real_T (*)[4])c3_outData)[c3_i243] = c3_y[c3_i243];
  }

  sf_mex_destroy(&c3_mxArrayInData);
}

static const mxArray *c3_i_sf_marshallOut(void *chartInstanceVoid, void
  *c3_inData)
{
  const mxArray *c3_mxArrayOutData = NULL;
  int32_T c3_i244;
  real_T c3_b_inData[2];
  int32_T c3_i245;
  real_T c3_u[2];
  const mxArray *c3_y = NULL;
  SFc3_Robot_Control_v01InstanceStruct *chartInstance;
  chartInstance = (SFc3_Robot_Control_v01InstanceStruct *)chartInstanceVoid;
  c3_mxArrayOutData = NULL;
  for (c3_i244 = 0; c3_i244 < 2; c3_i244++) {
    c3_b_inData[c3_i244] = (*(real_T (*)[2])c3_inData)[c3_i244];
  }

  for (c3_i245 = 0; c3_i245 < 2; c3_i245++) {
    c3_u[c3_i245] = c3_b_inData[c3_i245];
  }

  c3_y = NULL;
  sf_mex_assign(&c3_y, sf_mex_create("y", c3_u, 0, 0U, 1U, 0U, 2, 1, 2), false);
  sf_mex_assign(&c3_mxArrayOutData, c3_y, false);
  return c3_mxArrayOutData;
}

static void c3_m_emlrt_marshallIn(SFc3_Robot_Control_v01InstanceStruct
  *chartInstance, const mxArray *c3_u, const emlrtMsgIdentifier *c3_parentId,
  real_T c3_y[2])
{
  real_T c3_dv24[2];
  int32_T c3_i246;
  (void)chartInstance;
  sf_mex_import(c3_parentId, sf_mex_dup(c3_u), c3_dv24, 1, 0, 0U, 1, 0U, 2, 1, 2);
  for (c3_i246 = 0; c3_i246 < 2; c3_i246++) {
    c3_y[c3_i246] = c3_dv24[c3_i246];
  }

  sf_mex_destroy(&c3_u);
}

static void c3_i_sf_marshallIn(void *chartInstanceVoid, const mxArray
  *c3_mxArrayInData, const char_T *c3_varName, void *c3_outData)
{
  const mxArray *c3_pp;
  const char_T *c3_identifier;
  emlrtMsgIdentifier c3_thisId;
  real_T c3_y[2];
  int32_T c3_i247;
  SFc3_Robot_Control_v01InstanceStruct *chartInstance;
  chartInstance = (SFc3_Robot_Control_v01InstanceStruct *)chartInstanceVoid;
  c3_pp = sf_mex_dup(c3_mxArrayInData);
  c3_identifier = c3_varName;
  c3_thisId.fIdentifier = c3_identifier;
  c3_thisId.fParent = NULL;
  c3_m_emlrt_marshallIn(chartInstance, sf_mex_dup(c3_pp), &c3_thisId, c3_y);
  sf_mex_destroy(&c3_pp);
  for (c3_i247 = 0; c3_i247 < 2; c3_i247++) {
    (*(real_T (*)[2])c3_outData)[c3_i247] = c3_y[c3_i247];
  }

  sf_mex_destroy(&c3_mxArrayInData);
}

static const mxArray *c3_j_sf_marshallOut(void *chartInstanceVoid, void
  *c3_inData)
{
  const mxArray *c3_mxArrayOutData = NULL;
  c3_smuwpGFfhzM6MfA0WGyOk6E c3_u;
  const mxArray *c3_y = NULL;
  real_T c3_b_u;
  const mxArray *c3_b_y = NULL;
  real_T c3_c_u;
  const mxArray *c3_c_y = NULL;
  real_T c3_d_u;
  const mxArray *c3_d_y = NULL;
  real_T c3_e_u;
  const mxArray *c3_e_y = NULL;
  real_T c3_f_u;
  const mxArray *c3_f_y = NULL;
  real_T c3_g_u;
  const mxArray *c3_g_y = NULL;
  SFc3_Robot_Control_v01InstanceStruct *chartInstance;
  chartInstance = (SFc3_Robot_Control_v01InstanceStruct *)chartInstanceVoid;
  c3_mxArrayOutData = NULL;
  c3_u = *(c3_smuwpGFfhzM6MfA0WGyOk6E *)c3_inData;
  c3_y = NULL;
  sf_mex_assign(&c3_y, sf_mex_createstruct("structure", 2, 1, 1), false);
  c3_b_u = c3_u.x;
  c3_b_y = NULL;
  sf_mex_assign(&c3_b_y, sf_mex_create("y", &c3_b_u, 0, 0U, 0U, 0U, 0), false);
  sf_mex_addfield(c3_y, c3_b_y, "x", "x", 0);
  c3_c_u = c3_u.y;
  c3_c_y = NULL;
  sf_mex_assign(&c3_c_y, sf_mex_create("y", &c3_c_u, 0, 0U, 0U, 0U, 0), false);
  sf_mex_addfield(c3_y, c3_c_y, "y", "y", 0);
  c3_d_u = c3_u.z;
  c3_d_y = NULL;
  sf_mex_assign(&c3_d_y, sf_mex_create("y", &c3_d_u, 0, 0U, 0U, 0U, 0), false);
  sf_mex_addfield(c3_y, c3_d_y, "z", "z", 0);
  c3_e_u = c3_u.phi;
  c3_e_y = NULL;
  sf_mex_assign(&c3_e_y, sf_mex_create("y", &c3_e_u, 0, 0U, 0U, 0U, 0), false);
  sf_mex_addfield(c3_y, c3_e_y, "phi", "phi", 0);
  c3_f_u = c3_u.theta;
  c3_f_y = NULL;
  sf_mex_assign(&c3_f_y, sf_mex_create("y", &c3_f_u, 0, 0U, 0U, 0U, 0), false);
  sf_mex_addfield(c3_y, c3_f_y, "theta", "theta", 0);
  c3_g_u = c3_u.psi;
  c3_g_y = NULL;
  sf_mex_assign(&c3_g_y, sf_mex_create("y", &c3_g_u, 0, 0U, 0U, 0U, 0), false);
  sf_mex_addfield(c3_y, c3_g_y, "psi", "psi", 0);
  sf_mex_assign(&c3_mxArrayOutData, c3_y, false);
  return c3_mxArrayOutData;
}

static c3_smuwpGFfhzM6MfA0WGyOk6E c3_n_emlrt_marshallIn
  (SFc3_Robot_Control_v01InstanceStruct *chartInstance, const mxArray *c3_u,
   const emlrtMsgIdentifier *c3_parentId)
{
  c3_smuwpGFfhzM6MfA0WGyOk6E c3_y;
  emlrtMsgIdentifier c3_thisId;
  static const char * c3_fieldNames[6] = { "x", "y", "z", "phi", "theta", "psi"
  };

  c3_thisId.fParent = c3_parentId;
  sf_mex_check_struct(c3_parentId, c3_u, 6, c3_fieldNames, 0U, NULL);
  c3_thisId.fIdentifier = "x";
  c3_y.x = c3_e_emlrt_marshallIn(chartInstance, sf_mex_dup(sf_mex_getfield(c3_u,
    "x", "x", 0)), &c3_thisId);
  c3_thisId.fIdentifier = "y";
  c3_y.y = c3_e_emlrt_marshallIn(chartInstance, sf_mex_dup(sf_mex_getfield(c3_u,
    "y", "y", 0)), &c3_thisId);
  c3_thisId.fIdentifier = "z";
  c3_y.z = c3_e_emlrt_marshallIn(chartInstance, sf_mex_dup(sf_mex_getfield(c3_u,
    "z", "z", 0)), &c3_thisId);
  c3_thisId.fIdentifier = "phi";
  c3_y.phi = c3_e_emlrt_marshallIn(chartInstance, sf_mex_dup(sf_mex_getfield
    (c3_u, "phi", "phi", 0)), &c3_thisId);
  c3_thisId.fIdentifier = "theta";
  c3_y.theta = c3_e_emlrt_marshallIn(chartInstance, sf_mex_dup(sf_mex_getfield
    (c3_u, "theta", "theta", 0)), &c3_thisId);
  c3_thisId.fIdentifier = "psi";
  c3_y.psi = c3_e_emlrt_marshallIn(chartInstance, sf_mex_dup(sf_mex_getfield
    (c3_u, "psi", "psi", 0)), &c3_thisId);
  sf_mex_destroy(&c3_u);
  return c3_y;
}

static void c3_j_sf_marshallIn(void *chartInstanceVoid, const mxArray
  *c3_mxArrayInData, const char_T *c3_varName, void *c3_outData)
{
  const mxArray *c3_Pose;
  const char_T *c3_identifier;
  emlrtMsgIdentifier c3_thisId;
  c3_smuwpGFfhzM6MfA0WGyOk6E c3_y;
  SFc3_Robot_Control_v01InstanceStruct *chartInstance;
  chartInstance = (SFc3_Robot_Control_v01InstanceStruct *)chartInstanceVoid;
  c3_Pose = sf_mex_dup(c3_mxArrayInData);
  c3_identifier = c3_varName;
  c3_thisId.fIdentifier = c3_identifier;
  c3_thisId.fParent = NULL;
  c3_y = c3_n_emlrt_marshallIn(chartInstance, sf_mex_dup(c3_Pose), &c3_thisId);
  sf_mex_destroy(&c3_Pose);
  *(c3_smuwpGFfhzM6MfA0WGyOk6E *)c3_outData = c3_y;
  sf_mex_destroy(&c3_mxArrayInData);
}

static const mxArray *c3_k_sf_marshallOut(void *chartInstanceVoid, void
  *c3_inData)
{
  const mxArray *c3_mxArrayOutData;
  void *c3_u;
  const mxArray *c3_y = NULL;
  c3_sKe46gf3wfNeOrzL8yzt6nF c3_b_u;
  const mxArray *c3_b_y = NULL;
  real_T c3_c_u;
  const mxArray *c3_c_y = NULL;
  real_T c3_d_u;
  const mxArray *c3_d_y = NULL;
  real_T c3_e_u;
  const mxArray *c3_e_y = NULL;
  real_T c3_f_u;
  const mxArray *c3_f_y = NULL;
  real_T c3_g_u;
  const mxArray *c3_g_y = NULL;
  real_T c3_h_u;
  const mxArray *c3_h_y = NULL;
  SFc3_Robot_Control_v01InstanceStruct *chartInstance;
  chartInstance = (SFc3_Robot_Control_v01InstanceStruct *)chartInstanceVoid;
  c3_mxArrayOutData = NULL;
  c3_mxArrayOutData = NULL;
  c3_u = *(void **)c3_inData;
  c3_y = NULL;
  sf_mex_assign(&c3_y, sf_mex_createcellmatrix(1, 1), false);
  c3_b_u = *(c3_sKe46gf3wfNeOrzL8yzt6nF *)c3_u;
  c3_b_y = NULL;
  sf_mex_assign(&c3_b_y, sf_mex_createstruct("structure", 2, 1, 1), false);
  c3_c_u = c3_b_u.tx;
  c3_c_y = NULL;
  sf_mex_assign(&c3_c_y, sf_mex_create("y", &c3_c_u, 0, 0U, 0U, 0U, 0), false);
  sf_mex_addfield(c3_b_y, c3_c_y, "tx", "tx", 0);
  c3_d_u = c3_b_u.ty;
  c3_d_y = NULL;
  sf_mex_assign(&c3_d_y, sf_mex_create("y", &c3_d_u, 0, 0U, 0U, 0U, 0), false);
  sf_mex_addfield(c3_b_y, c3_d_y, "ty", "ty", 0);
  c3_e_u = c3_b_u.tz;
  c3_e_y = NULL;
  sf_mex_assign(&c3_e_y, sf_mex_create("y", &c3_e_u, 0, 0U, 0U, 0U, 0), false);
  sf_mex_addfield(c3_b_y, c3_e_y, "tz", "tz", 0);
  c3_f_u = c3_b_u.phi;
  c3_f_y = NULL;
  sf_mex_assign(&c3_f_y, sf_mex_create("y", &c3_f_u, 0, 0U, 0U, 0U, 0), false);
  sf_mex_addfield(c3_b_y, c3_f_y, "phi", "phi", 0);
  c3_g_u = c3_b_u.theta;
  c3_g_y = NULL;
  sf_mex_assign(&c3_g_y, sf_mex_create("y", &c3_g_u, 0, 0U, 0U, 0U, 0), false);
  sf_mex_addfield(c3_b_y, c3_g_y, "theta", "theta", 0);
  c3_h_u = c3_b_u.psi;
  c3_h_y = NULL;
  sf_mex_assign(&c3_h_y, sf_mex_create("y", &c3_h_u, 0, 0U, 0U, 0U, 0), false);
  sf_mex_addfield(c3_b_y, c3_h_y, "psi", "psi", 0);
  sf_mex_setcell(c3_y, 0, c3_b_y);
  sf_mex_assign(&c3_mxArrayOutData, c3_y, false);
  return c3_mxArrayOutData;
}

static const mxArray *c3_l_sf_marshallOut(void *chartInstanceVoid, void
  *c3_inData)
{
  const mxArray *c3_mxArrayOutData;
  c3_s6d9wPYgmS3cwGRsMJd1GAE c3_u;
  const mxArray *c3_y = NULL;
  c3_sKe46gf3wfNeOrzL8yzt6nF c3_b_u;
  const mxArray *c3_b_y = NULL;
  real_T c3_c_u;
  const mxArray *c3_c_y = NULL;
  real_T c3_d_u;
  const mxArray *c3_d_y = NULL;
  real_T c3_e_u;
  const mxArray *c3_e_y = NULL;
  real_T c3_f_u;
  const mxArray *c3_f_y = NULL;
  real_T c3_g_u;
  const mxArray *c3_g_y = NULL;
  real_T c3_h_u;
  const mxArray *c3_h_y = NULL;
  real_T c3_i_u;
  const mxArray *c3_i_y = NULL;
  real_T c3_j_u;
  const mxArray *c3_j_y = NULL;
  real_T c3_k_u;
  const mxArray *c3_k_y = NULL;
  real_T c3_l_u;
  const mxArray *c3_l_y = NULL;
  real_T c3_m_u;
  const mxArray *c3_m_y = NULL;
  real_T c3_n_u;
  const mxArray *c3_n_y = NULL;
  SFc3_Robot_Control_v01InstanceStruct *chartInstance;
  chartInstance = (SFc3_Robot_Control_v01InstanceStruct *)chartInstanceVoid;
  c3_mxArrayOutData = NULL;
  c3_mxArrayOutData = NULL;
  c3_u = *(c3_s6d9wPYgmS3cwGRsMJd1GAE *)c3_inData;
  c3_y = NULL;
  sf_mex_assign(&c3_y, sf_mex_createstruct("structure", 2, 1, 1), false);
  c3_b_u = c3_u.pose;
  c3_b_y = NULL;
  sf_mex_assign(&c3_b_y, sf_mex_createstruct("structure", 2, 1, 1), false);
  c3_c_u = c3_b_u.tx;
  c3_c_y = NULL;
  sf_mex_assign(&c3_c_y, sf_mex_create("y", &c3_c_u, 0, 0U, 0U, 0U, 0), false);
  sf_mex_addfield(c3_b_y, c3_c_y, "tx", "tx", 0);
  c3_d_u = c3_b_u.ty;
  c3_d_y = NULL;
  sf_mex_assign(&c3_d_y, sf_mex_create("y", &c3_d_u, 0, 0U, 0U, 0U, 0), false);
  sf_mex_addfield(c3_b_y, c3_d_y, "ty", "ty", 0);
  c3_e_u = c3_b_u.tz;
  c3_e_y = NULL;
  sf_mex_assign(&c3_e_y, sf_mex_create("y", &c3_e_u, 0, 0U, 0U, 0U, 0), false);
  sf_mex_addfield(c3_b_y, c3_e_y, "tz", "tz", 0);
  c3_f_u = c3_b_u.phi;
  c3_f_y = NULL;
  sf_mex_assign(&c3_f_y, sf_mex_create("y", &c3_f_u, 0, 0U, 0U, 0U, 0), false);
  sf_mex_addfield(c3_b_y, c3_f_y, "phi", "phi", 0);
  c3_g_u = c3_b_u.theta;
  c3_g_y = NULL;
  sf_mex_assign(&c3_g_y, sf_mex_create("y", &c3_g_u, 0, 0U, 0U, 0U, 0), false);
  sf_mex_addfield(c3_b_y, c3_g_y, "theta", "theta", 0);
  c3_h_u = c3_b_u.psi;
  c3_h_y = NULL;
  sf_mex_assign(&c3_h_y, sf_mex_create("y", &c3_h_u, 0, 0U, 0U, 0U, 0), false);
  sf_mex_addfield(c3_b_y, c3_h_y, "psi", "psi", 0);
  sf_mex_addfield(c3_y, c3_b_y, "pose", "pose", 0);
  c3_i_u = c3_u.fx;
  c3_i_y = NULL;
  sf_mex_assign(&c3_i_y, sf_mex_create("y", &c3_i_u, 0, 0U, 0U, 0U, 0), false);
  sf_mex_addfield(c3_y, c3_i_y, "fx", "fx", 0);
  c3_j_u = c3_u.fy;
  c3_j_y = NULL;
  sf_mex_assign(&c3_j_y, sf_mex_create("y", &c3_j_u, 0, 0U, 0U, 0U, 0), false);
  sf_mex_addfield(c3_y, c3_j_y, "fy", "fy", 0);
  c3_k_u = c3_u.cx;
  c3_k_y = NULL;
  sf_mex_assign(&c3_k_y, sf_mex_create("y", &c3_k_u, 0, 0U, 0U, 0U, 0), false);
  sf_mex_addfield(c3_y, c3_k_y, "cx", "cx", 0);
  c3_l_u = c3_u.cy;
  c3_l_y = NULL;
  sf_mex_assign(&c3_l_y, sf_mex_create("y", &c3_l_u, 0, 0U, 0U, 0U, 0), false);
  sf_mex_addfield(c3_y, c3_l_y, "cy", "cy", 0);
  c3_m_u = c3_u.s;
  c3_m_y = NULL;
  sf_mex_assign(&c3_m_y, sf_mex_create("y", &c3_m_u, 0, 0U, 0U, 0U, 0), false);
  sf_mex_addfield(c3_y, c3_m_y, "s", "s", 0);
  c3_n_u = c3_u.d;
  c3_n_y = NULL;
  sf_mex_assign(&c3_n_y, sf_mex_create("y", &c3_n_u, 0, 0U, 0U, 0U, 0), false);
  sf_mex_addfield(c3_y, c3_n_y, "d", "d", 0);
  sf_mex_assign(&c3_mxArrayOutData, c3_y, false);
  return c3_mxArrayOutData;
}

static void c3_k_sf_marshallIn(void *chartInstanceVoid, const mxArray
  *c3_mxArrayInData, const char_T *c3_varName, void *c3_outData)
{
  const mxArray *c3_camera;
  const char_T *c3_identifier;
  emlrtMsgIdentifier c3_thisId;
  c3_s6d9wPYgmS3cwGRsMJd1GAE c3_y;
  SFc3_Robot_Control_v01InstanceStruct *chartInstance;
  chartInstance = (SFc3_Robot_Control_v01InstanceStruct *)chartInstanceVoid;
  c3_camera = sf_mex_dup(c3_mxArrayInData);
  c3_identifier = c3_varName;
  c3_thisId.fIdentifier = c3_identifier;
  c3_thisId.fParent = NULL;
  c3_j_emlrt_marshallIn(chartInstance, sf_mex_dup(c3_camera), &c3_thisId, &c3_y);
  sf_mex_destroy(&c3_camera);
  *(c3_s6d9wPYgmS3cwGRsMJd1GAE *)c3_outData = c3_y;
  sf_mex_destroy(&c3_mxArrayInData);
}

static const mxArray *c3_m_sf_marshallOut(void *chartInstanceVoid, void
  *c3_inData)
{
  const mxArray *c3_mxArrayOutData = NULL;
  c3_sZiCA83jGK4zdmds7wvlXUF c3_u;
  const mxArray *c3_y = NULL;
  int32_T c3_i248;
  real_T c3_b_u[8];
  const mxArray *c3_b_y = NULL;
  int32_T c3_i249;
  real_T c3_c_u[16];
  const mxArray *c3_c_y = NULL;
  int32_T c3_i250;
  real_T c3_d_u[16];
  const mxArray *c3_d_y = NULL;
  SFc3_Robot_Control_v01InstanceStruct *chartInstance;
  chartInstance = (SFc3_Robot_Control_v01InstanceStruct *)chartInstanceVoid;
  c3_mxArrayOutData = NULL;
  c3_u = *(c3_sZiCA83jGK4zdmds7wvlXUF *)c3_inData;
  c3_y = NULL;
  sf_mex_assign(&c3_y, sf_mex_createstruct("structure", 2, 1, 1), false);
  for (c3_i248 = 0; c3_i248 < 8; c3_i248++) {
    c3_b_u[c3_i248] = c3_u.p[c3_i248];
  }

  c3_b_y = NULL;
  sf_mex_assign(&c3_b_y, sf_mex_create("y", c3_b_u, 0, 0U, 1U, 0U, 2, 2, 4),
                false);
  sf_mex_addfield(c3_y, c3_b_y, "p", "p", 0);
  for (c3_i249 = 0; c3_i249 < 16; c3_i249++) {
    c3_c_u[c3_i249] = c3_u.C_P[c3_i249];
  }

  c3_c_y = NULL;
  sf_mex_assign(&c3_c_y, sf_mex_create("y", c3_c_u, 0, 0U, 1U, 0U, 2, 4, 4),
                false);
  sf_mex_addfield(c3_y, c3_c_y, "C_P", "C_P", 0);
  for (c3_i250 = 0; c3_i250 < 16; c3_i250++) {
    c3_d_u[c3_i250] = c3_u.M[c3_i250];
  }

  c3_d_y = NULL;
  sf_mex_assign(&c3_d_y, sf_mex_create("y", c3_d_u, 0, 0U, 1U, 0U, 2, 4, 4),
                false);
  sf_mex_addfield(c3_y, c3_d_y, "M", "M", 0);
  sf_mex_assign(&c3_mxArrayOutData, c3_y, false);
  return c3_mxArrayOutData;
}

static void c3_o_emlrt_marshallIn(SFc3_Robot_Control_v01InstanceStruct
  *chartInstance, const mxArray *c3_u, const emlrtMsgIdentifier *c3_parentId,
  c3_sZiCA83jGK4zdmds7wvlXUF *c3_y)
{
  emlrtMsgIdentifier c3_thisId;
  static const char * c3_fieldNames[3] = { "p", "C_P", "M" };

  c3_thisId.fParent = c3_parentId;
  sf_mex_check_struct(c3_parentId, c3_u, 3, c3_fieldNames, 0U, NULL);
  c3_thisId.fIdentifier = "p";
  c3_k_emlrt_marshallIn(chartInstance, sf_mex_dup(sf_mex_getfield(c3_u, "p", "p",
    0)), &c3_thisId, c3_y->p);
  c3_thisId.fIdentifier = "C_P";
  c3_f_emlrt_marshallIn(chartInstance, sf_mex_dup(sf_mex_getfield(c3_u, "C_P",
    "C_P", 0)), &c3_thisId, c3_y->C_P);
  c3_thisId.fIdentifier = "M";
  c3_f_emlrt_marshallIn(chartInstance, sf_mex_dup(sf_mex_getfield(c3_u, "M", "M",
    0)), &c3_thisId, c3_y->M);
  sf_mex_destroy(&c3_u);
}

static void c3_l_sf_marshallIn(void *chartInstanceVoid, const mxArray
  *c3_mxArrayInData, const char_T *c3_varName, void *c3_outData)
{
  const mxArray *c3_retVal;
  const char_T *c3_identifier;
  emlrtMsgIdentifier c3_thisId;
  c3_sZiCA83jGK4zdmds7wvlXUF c3_y;
  SFc3_Robot_Control_v01InstanceStruct *chartInstance;
  chartInstance = (SFc3_Robot_Control_v01InstanceStruct *)chartInstanceVoid;
  c3_retVal = sf_mex_dup(c3_mxArrayInData);
  c3_identifier = c3_varName;
  c3_thisId.fIdentifier = c3_identifier;
  c3_thisId.fParent = NULL;
  c3_o_emlrt_marshallIn(chartInstance, sf_mex_dup(c3_retVal), &c3_thisId, &c3_y);
  sf_mex_destroy(&c3_retVal);
  *(c3_sZiCA83jGK4zdmds7wvlXUF *)c3_outData = c3_y;
  sf_mex_destroy(&c3_mxArrayInData);
}

static const mxArray *c3_n_sf_marshallOut(void *chartInstanceVoid, void
  *c3_inData)
{
  const mxArray *c3_mxArrayOutData = NULL;
  int32_T c3_i251;
  real_T c3_b_inData[8];
  int32_T c3_i252;
  real_T c3_u[8];
  const mxArray *c3_y = NULL;
  SFc3_Robot_Control_v01InstanceStruct *chartInstance;
  chartInstance = (SFc3_Robot_Control_v01InstanceStruct *)chartInstanceVoid;
  c3_mxArrayOutData = NULL;
  for (c3_i251 = 0; c3_i251 < 8; c3_i251++) {
    c3_b_inData[c3_i251] = (*(real_T (*)[8])c3_inData)[c3_i251];
  }

  for (c3_i252 = 0; c3_i252 < 8; c3_i252++) {
    c3_u[c3_i252] = c3_b_inData[c3_i252];
  }

  c3_y = NULL;
  sf_mex_assign(&c3_y, sf_mex_create("y", c3_u, 0, 0U, 1U, 0U, 1, 8), false);
  sf_mex_assign(&c3_mxArrayOutData, c3_y, false);
  return c3_mxArrayOutData;
}

static void c3_p_emlrt_marshallIn(SFc3_Robot_Control_v01InstanceStruct
  *chartInstance, const mxArray *c3_u, const emlrtMsgIdentifier *c3_parentId,
  real_T c3_y[8])
{
  real_T c3_dv25[8];
  int32_T c3_i253;
  (void)chartInstance;
  sf_mex_import(c3_parentId, sf_mex_dup(c3_u), c3_dv25, 1, 0, 0U, 1, 0U, 1, 8);
  for (c3_i253 = 0; c3_i253 < 8; c3_i253++) {
    c3_y[c3_i253] = c3_dv25[c3_i253];
  }

  sf_mex_destroy(&c3_u);
}

static void c3_m_sf_marshallIn(void *chartInstanceVoid, const mxArray
  *c3_mxArrayInData, const char_T *c3_varName, void *c3_outData)
{
  const mxArray *c3_constants;
  const char_T *c3_identifier;
  emlrtMsgIdentifier c3_thisId;
  real_T c3_y[8];
  int32_T c3_i254;
  SFc3_Robot_Control_v01InstanceStruct *chartInstance;
  chartInstance = (SFc3_Robot_Control_v01InstanceStruct *)chartInstanceVoid;
  c3_constants = sf_mex_dup(c3_mxArrayInData);
  c3_identifier = c3_varName;
  c3_thisId.fIdentifier = c3_identifier;
  c3_thisId.fParent = NULL;
  c3_p_emlrt_marshallIn(chartInstance, sf_mex_dup(c3_constants), &c3_thisId,
                        c3_y);
  sf_mex_destroy(&c3_constants);
  for (c3_i254 = 0; c3_i254 < 8; c3_i254++) {
    (*(real_T (*)[8])c3_outData)[c3_i254] = c3_y[c3_i254];
  }

  sf_mex_destroy(&c3_mxArrayInData);
}

static const mxArray *c3_o_sf_marshallOut(void *chartInstanceVoid, void
  *c3_inData)
{
  const mxArray *c3_mxArrayOutData = NULL;
  int32_T c3_i255;
  int32_T c3_i256;
  int32_T c3_i257;
  real_T c3_b_inData[96];
  int32_T c3_i258;
  int32_T c3_i259;
  int32_T c3_i260;
  real_T c3_u[96];
  const mxArray *c3_y = NULL;
  SFc3_Robot_Control_v01InstanceStruct *chartInstance;
  chartInstance = (SFc3_Robot_Control_v01InstanceStruct *)chartInstanceVoid;
  c3_mxArrayOutData = NULL;
  c3_i255 = 0;
  for (c3_i256 = 0; c3_i256 < 12; c3_i256++) {
    for (c3_i257 = 0; c3_i257 < 8; c3_i257++) {
      c3_b_inData[c3_i257 + c3_i255] = (*(real_T (*)[96])c3_inData)[c3_i257 +
        c3_i255];
    }

    c3_i255 += 8;
  }

  c3_i258 = 0;
  for (c3_i259 = 0; c3_i259 < 12; c3_i259++) {
    for (c3_i260 = 0; c3_i260 < 8; c3_i260++) {
      c3_u[c3_i260 + c3_i258] = c3_b_inData[c3_i260 + c3_i258];
    }

    c3_i258 += 8;
  }

  c3_y = NULL;
  sf_mex_assign(&c3_y, sf_mex_create("y", c3_u, 0, 0U, 1U, 0U, 2, 8, 12), false);
  sf_mex_assign(&c3_mxArrayOutData, c3_y, false);
  return c3_mxArrayOutData;
}

static void c3_q_emlrt_marshallIn(SFc3_Robot_Control_v01InstanceStruct
  *chartInstance, const mxArray *c3_u, const emlrtMsgIdentifier *c3_parentId,
  real_T c3_y[96])
{
  real_T c3_dv26[96];
  int32_T c3_i261;
  (void)chartInstance;
  sf_mex_import(c3_parentId, sf_mex_dup(c3_u), c3_dv26, 1, 0, 0U, 1, 0U, 2, 8,
                12);
  for (c3_i261 = 0; c3_i261 < 96; c3_i261++) {
    c3_y[c3_i261] = c3_dv26[c3_i261];
  }

  sf_mex_destroy(&c3_u);
}

static void c3_n_sf_marshallIn(void *chartInstanceVoid, const mxArray
  *c3_mxArrayInData, const char_T *c3_varName, void *c3_outData)
{
  const mxArray *c3_equations;
  const char_T *c3_identifier;
  emlrtMsgIdentifier c3_thisId;
  real_T c3_y[96];
  int32_T c3_i262;
  int32_T c3_i263;
  int32_T c3_i264;
  SFc3_Robot_Control_v01InstanceStruct *chartInstance;
  chartInstance = (SFc3_Robot_Control_v01InstanceStruct *)chartInstanceVoid;
  c3_equations = sf_mex_dup(c3_mxArrayInData);
  c3_identifier = c3_varName;
  c3_thisId.fIdentifier = c3_identifier;
  c3_thisId.fParent = NULL;
  c3_q_emlrt_marshallIn(chartInstance, sf_mex_dup(c3_equations), &c3_thisId,
                        c3_y);
  sf_mex_destroy(&c3_equations);
  c3_i262 = 0;
  for (c3_i263 = 0; c3_i263 < 12; c3_i263++) {
    for (c3_i264 = 0; c3_i264 < 8; c3_i264++) {
      (*(real_T (*)[96])c3_outData)[c3_i264 + c3_i262] = c3_y[c3_i264 + c3_i262];
    }

    c3_i262 += 8;
  }

  sf_mex_destroy(&c3_mxArrayInData);
}

static const mxArray *c3_p_sf_marshallOut(void *chartInstanceVoid, void
  *c3_inData)
{
  const mxArray *c3_mxArrayOutData = NULL;
  int32_T c3_i265;
  real_T c3_b_inData[3];
  int32_T c3_i266;
  real_T c3_u[3];
  const mxArray *c3_y = NULL;
  SFc3_Robot_Control_v01InstanceStruct *chartInstance;
  chartInstance = (SFc3_Robot_Control_v01InstanceStruct *)chartInstanceVoid;
  c3_mxArrayOutData = NULL;
  for (c3_i265 = 0; c3_i265 < 3; c3_i265++) {
    c3_b_inData[c3_i265] = (*(real_T (*)[3])c3_inData)[c3_i265];
  }

  for (c3_i266 = 0; c3_i266 < 3; c3_i266++) {
    c3_u[c3_i266] = c3_b_inData[c3_i266];
  }

  c3_y = NULL;
  sf_mex_assign(&c3_y, sf_mex_create("y", c3_u, 0, 0U, 1U, 0U, 2, 1, 3), false);
  sf_mex_assign(&c3_mxArrayOutData, c3_y, false);
  return c3_mxArrayOutData;
}

static void c3_r_emlrt_marshallIn(SFc3_Robot_Control_v01InstanceStruct
  *chartInstance, const mxArray *c3_u, const emlrtMsgIdentifier *c3_parentId,
  real_T c3_y[3])
{
  real_T c3_dv27[3];
  int32_T c3_i267;
  (void)chartInstance;
  sf_mex_import(c3_parentId, sf_mex_dup(c3_u), c3_dv27, 1, 0, 0U, 1, 0U, 2, 1, 3);
  for (c3_i267 = 0; c3_i267 < 3; c3_i267++) {
    c3_y[c3_i267] = c3_dv27[c3_i267];
  }

  sf_mex_destroy(&c3_u);
}

static void c3_o_sf_marshallIn(void *chartInstanceVoid, const mxArray
  *c3_mxArrayInData, const char_T *c3_varName, void *c3_outData)
{
  const mxArray *c3_Vy;
  const char_T *c3_identifier;
  emlrtMsgIdentifier c3_thisId;
  real_T c3_y[3];
  int32_T c3_i268;
  SFc3_Robot_Control_v01InstanceStruct *chartInstance;
  chartInstance = (SFc3_Robot_Control_v01InstanceStruct *)chartInstanceVoid;
  c3_Vy = sf_mex_dup(c3_mxArrayInData);
  c3_identifier = c3_varName;
  c3_thisId.fIdentifier = c3_identifier;
  c3_thisId.fParent = NULL;
  c3_r_emlrt_marshallIn(chartInstance, sf_mex_dup(c3_Vy), &c3_thisId, c3_y);
  sf_mex_destroy(&c3_Vy);
  for (c3_i268 = 0; c3_i268 < 3; c3_i268++) {
    (*(real_T (*)[3])c3_outData)[c3_i268] = c3_y[c3_i268];
  }

  sf_mex_destroy(&c3_mxArrayInData);
}

static const mxArray *c3_q_sf_marshallOut(void *chartInstanceVoid, void
  *c3_inData)
{
  const mxArray *c3_mxArrayOutData = NULL;
  int32_T c3_i269;
  int32_T c3_i270;
  int32_T c3_i271;
  real_T c3_b_inData[12];
  int32_T c3_i272;
  int32_T c3_i273;
  int32_T c3_i274;
  real_T c3_u[12];
  const mxArray *c3_y = NULL;
  SFc3_Robot_Control_v01InstanceStruct *chartInstance;
  chartInstance = (SFc3_Robot_Control_v01InstanceStruct *)chartInstanceVoid;
  c3_mxArrayOutData = NULL;
  c3_i269 = 0;
  for (c3_i270 = 0; c3_i270 < 4; c3_i270++) {
    for (c3_i271 = 0; c3_i271 < 3; c3_i271++) {
      c3_b_inData[c3_i271 + c3_i269] = (*(real_T (*)[12])c3_inData)[c3_i271 +
        c3_i269];
    }

    c3_i269 += 3;
  }

  c3_i272 = 0;
  for (c3_i273 = 0; c3_i273 < 4; c3_i273++) {
    for (c3_i274 = 0; c3_i274 < 3; c3_i274++) {
      c3_u[c3_i274 + c3_i272] = c3_b_inData[c3_i274 + c3_i272];
    }

    c3_i272 += 3;
  }

  c3_y = NULL;
  sf_mex_assign(&c3_y, sf_mex_create("y", c3_u, 0, 0U, 1U, 0U, 2, 3, 4), false);
  sf_mex_assign(&c3_mxArrayOutData, c3_y, false);
  return c3_mxArrayOutData;
}

static void c3_s_emlrt_marshallIn(SFc3_Robot_Control_v01InstanceStruct
  *chartInstance, const mxArray *c3_u, const emlrtMsgIdentifier *c3_parentId,
  real_T c3_y[12])
{
  real_T c3_dv28[12];
  int32_T c3_i275;
  (void)chartInstance;
  sf_mex_import(c3_parentId, sf_mex_dup(c3_u), c3_dv28, 1, 0, 0U, 1, 0U, 2, 3, 4);
  for (c3_i275 = 0; c3_i275 < 12; c3_i275++) {
    c3_y[c3_i275] = c3_dv28[c3_i275];
  }

  sf_mex_destroy(&c3_u);
}

static void c3_p_sf_marshallIn(void *chartInstanceVoid, const mxArray
  *c3_mxArrayInData, const char_T *c3_varName, void *c3_outData)
{
  const mxArray *c3_O;
  const char_T *c3_identifier;
  emlrtMsgIdentifier c3_thisId;
  real_T c3_y[12];
  int32_T c3_i276;
  int32_T c3_i277;
  int32_T c3_i278;
  SFc3_Robot_Control_v01InstanceStruct *chartInstance;
  chartInstance = (SFc3_Robot_Control_v01InstanceStruct *)chartInstanceVoid;
  c3_O = sf_mex_dup(c3_mxArrayInData);
  c3_identifier = c3_varName;
  c3_thisId.fIdentifier = c3_identifier;
  c3_thisId.fParent = NULL;
  c3_s_emlrt_marshallIn(chartInstance, sf_mex_dup(c3_O), &c3_thisId, c3_y);
  sf_mex_destroy(&c3_O);
  c3_i276 = 0;
  for (c3_i277 = 0; c3_i277 < 4; c3_i277++) {
    for (c3_i278 = 0; c3_i278 < 3; c3_i278++) {
      (*(real_T (*)[12])c3_outData)[c3_i278 + c3_i276] = c3_y[c3_i278 + c3_i276];
    }

    c3_i276 += 3;
  }

  sf_mex_destroy(&c3_mxArrayInData);
}

static const mxArray *c3_r_sf_marshallOut(void *chartInstanceVoid, void
  *c3_inData)
{
  const mxArray *c3_mxArrayOutData = NULL;
  int32_T c3_i279;
  real_T c3_b_inData[12];
  int32_T c3_i280;
  real_T c3_u[12];
  const mxArray *c3_y = NULL;
  SFc3_Robot_Control_v01InstanceStruct *chartInstance;
  chartInstance = (SFc3_Robot_Control_v01InstanceStruct *)chartInstanceVoid;
  c3_mxArrayOutData = NULL;
  for (c3_i279 = 0; c3_i279 < 12; c3_i279++) {
    c3_b_inData[c3_i279] = (*(real_T (*)[12])c3_inData)[c3_i279];
  }

  for (c3_i280 = 0; c3_i280 < 12; c3_i280++) {
    c3_u[c3_i280] = c3_b_inData[c3_i280];
  }

  c3_y = NULL;
  sf_mex_assign(&c3_y, sf_mex_create("y", c3_u, 0, 0U, 1U, 0U, 1, 12), false);
  sf_mex_assign(&c3_mxArrayOutData, c3_y, false);
  return c3_mxArrayOutData;
}

static void c3_t_emlrt_marshallIn(SFc3_Robot_Control_v01InstanceStruct
  *chartInstance, const mxArray *c3_u, const emlrtMsgIdentifier *c3_parentId,
  real_T c3_y[12])
{
  real_T c3_dv29[12];
  int32_T c3_i281;
  (void)chartInstance;
  sf_mex_import(c3_parentId, sf_mex_dup(c3_u), c3_dv29, 1, 0, 0U, 1, 0U, 1, 12);
  for (c3_i281 = 0; c3_i281 < 12; c3_i281++) {
    c3_y[c3_i281] = c3_dv29[c3_i281];
  }

  sf_mex_destroy(&c3_u);
}

static void c3_q_sf_marshallIn(void *chartInstanceVoid, const mxArray
  *c3_mxArrayInData, const char_T *c3_varName, void *c3_outData)
{
  const mxArray *c3_poseError;
  const char_T *c3_identifier;
  emlrtMsgIdentifier c3_thisId;
  real_T c3_y[12];
  int32_T c3_i282;
  SFc3_Robot_Control_v01InstanceStruct *chartInstance;
  chartInstance = (SFc3_Robot_Control_v01InstanceStruct *)chartInstanceVoid;
  c3_poseError = sf_mex_dup(c3_mxArrayInData);
  c3_identifier = c3_varName;
  c3_thisId.fIdentifier = c3_identifier;
  c3_thisId.fParent = NULL;
  c3_t_emlrt_marshallIn(chartInstance, sf_mex_dup(c3_poseError), &c3_thisId,
                        c3_y);
  sf_mex_destroy(&c3_poseError);
  for (c3_i282 = 0; c3_i282 < 12; c3_i282++) {
    (*(real_T (*)[12])c3_outData)[c3_i282] = c3_y[c3_i282];
  }

  sf_mex_destroy(&c3_mxArrayInData);
}

static const mxArray *c3_s_sf_marshallOut(void *chartInstanceVoid, void
  *c3_inData)
{
  const mxArray *c3_mxArrayOutData = NULL;
  int32_T c3_i283;
  real_T c3_b_inData[16];
  int32_T c3_i284;
  real_T c3_u[16];
  const mxArray *c3_y = NULL;
  SFc3_Robot_Control_v01InstanceStruct *chartInstance;
  chartInstance = (SFc3_Robot_Control_v01InstanceStruct *)chartInstanceVoid;
  c3_mxArrayOutData = NULL;
  for (c3_i283 = 0; c3_i283 < 16; c3_i283++) {
    c3_b_inData[c3_i283] = (*(real_T (*)[16])c3_inData)[c3_i283];
  }

  for (c3_i284 = 0; c3_i284 < 16; c3_i284++) {
    c3_u[c3_i284] = c3_b_inData[c3_i284];
  }

  c3_y = NULL;
  sf_mex_assign(&c3_y, sf_mex_create("y", c3_u, 0, 0U, 1U, 0U, 1, 16), false);
  sf_mex_assign(&c3_mxArrayOutData, c3_y, false);
  return c3_mxArrayOutData;
}

static void c3_u_emlrt_marshallIn(SFc3_Robot_Control_v01InstanceStruct
  *chartInstance, const mxArray *c3_u, const emlrtMsgIdentifier *c3_parentId,
  real_T c3_y[16])
{
  real_T c3_dv30[16];
  int32_T c3_i285;
  (void)chartInstance;
  sf_mex_import(c3_parentId, sf_mex_dup(c3_u), c3_dv30, 1, 0, 0U, 1, 0U, 1, 16);
  for (c3_i285 = 0; c3_i285 < 16; c3_i285++) {
    c3_y[c3_i285] = c3_dv30[c3_i285];
  }

  sf_mex_destroy(&c3_u);
}

static void c3_r_sf_marshallIn(void *chartInstanceVoid, const mxArray
  *c3_mxArrayInData, const char_T *c3_varName, void *c3_outData)
{
  const mxArray *c3_AllConstants;
  const char_T *c3_identifier;
  emlrtMsgIdentifier c3_thisId;
  real_T c3_y[16];
  int32_T c3_i286;
  SFc3_Robot_Control_v01InstanceStruct *chartInstance;
  chartInstance = (SFc3_Robot_Control_v01InstanceStruct *)chartInstanceVoid;
  c3_AllConstants = sf_mex_dup(c3_mxArrayInData);
  c3_identifier = c3_varName;
  c3_thisId.fIdentifier = c3_identifier;
  c3_thisId.fParent = NULL;
  c3_u_emlrt_marshallIn(chartInstance, sf_mex_dup(c3_AllConstants), &c3_thisId,
                        c3_y);
  sf_mex_destroy(&c3_AllConstants);
  for (c3_i286 = 0; c3_i286 < 16; c3_i286++) {
    (*(real_T (*)[16])c3_outData)[c3_i286] = c3_y[c3_i286];
  }

  sf_mex_destroy(&c3_mxArrayInData);
}

static const mxArray *c3_t_sf_marshallOut(void *chartInstanceVoid, void
  *c3_inData)
{
  const mxArray *c3_mxArrayOutData = NULL;
  int32_T c3_i287;
  int32_T c3_i288;
  int32_T c3_i289;
  real_T c3_b_inData[192];
  int32_T c3_i290;
  int32_T c3_i291;
  int32_T c3_i292;
  real_T c3_u[192];
  const mxArray *c3_y = NULL;
  SFc3_Robot_Control_v01InstanceStruct *chartInstance;
  chartInstance = (SFc3_Robot_Control_v01InstanceStruct *)chartInstanceVoid;
  c3_mxArrayOutData = NULL;
  c3_i287 = 0;
  for (c3_i288 = 0; c3_i288 < 12; c3_i288++) {
    for (c3_i289 = 0; c3_i289 < 16; c3_i289++) {
      c3_b_inData[c3_i289 + c3_i287] = (*(real_T (*)[192])c3_inData)[c3_i289 +
        c3_i287];
    }

    c3_i287 += 16;
  }

  c3_i290 = 0;
  for (c3_i291 = 0; c3_i291 < 12; c3_i291++) {
    for (c3_i292 = 0; c3_i292 < 16; c3_i292++) {
      c3_u[c3_i292 + c3_i290] = c3_b_inData[c3_i292 + c3_i290];
    }

    c3_i290 += 16;
  }

  c3_y = NULL;
  sf_mex_assign(&c3_y, sf_mex_create("y", c3_u, 0, 0U, 1U, 0U, 2, 16, 12), false);
  sf_mex_assign(&c3_mxArrayOutData, c3_y, false);
  return c3_mxArrayOutData;
}

static void c3_v_emlrt_marshallIn(SFc3_Robot_Control_v01InstanceStruct
  *chartInstance, const mxArray *c3_u, const emlrtMsgIdentifier *c3_parentId,
  real_T c3_y[192])
{
  real_T c3_dv31[192];
  int32_T c3_i293;
  (void)chartInstance;
  sf_mex_import(c3_parentId, sf_mex_dup(c3_u), c3_dv31, 1, 0, 0U, 1, 0U, 2, 16,
                12);
  for (c3_i293 = 0; c3_i293 < 192; c3_i293++) {
    c3_y[c3_i293] = c3_dv31[c3_i293];
  }

  sf_mex_destroy(&c3_u);
}

static void c3_s_sf_marshallIn(void *chartInstanceVoid, const mxArray
  *c3_mxArrayInData, const char_T *c3_varName, void *c3_outData)
{
  const mxArray *c3_AllEquations;
  const char_T *c3_identifier;
  emlrtMsgIdentifier c3_thisId;
  real_T c3_y[192];
  int32_T c3_i294;
  int32_T c3_i295;
  int32_T c3_i296;
  SFc3_Robot_Control_v01InstanceStruct *chartInstance;
  chartInstance = (SFc3_Robot_Control_v01InstanceStruct *)chartInstanceVoid;
  c3_AllEquations = sf_mex_dup(c3_mxArrayInData);
  c3_identifier = c3_varName;
  c3_thisId.fIdentifier = c3_identifier;
  c3_thisId.fParent = NULL;
  c3_v_emlrt_marshallIn(chartInstance, sf_mex_dup(c3_AllEquations), &c3_thisId,
                        c3_y);
  sf_mex_destroy(&c3_AllEquations);
  c3_i294 = 0;
  for (c3_i295 = 0; c3_i295 < 12; c3_i295++) {
    for (c3_i296 = 0; c3_i296 < 16; c3_i296++) {
      (*(real_T (*)[192])c3_outData)[c3_i296 + c3_i294] = c3_y[c3_i296 + c3_i294];
    }

    c3_i294 += 16;
  }

  sf_mex_destroy(&c3_mxArrayInData);
}

static const mxArray *c3_u_sf_marshallOut(void *chartInstanceVoid, void
  *c3_inData)
{
  const mxArray *c3_mxArrayOutData = NULL;
  int32_T c3_i297;
  int32_T c3_i298;
  int32_T c3_i299;
  real_T c3_b_inData[9];
  int32_T c3_i300;
  int32_T c3_i301;
  int32_T c3_i302;
  real_T c3_u[9];
  const mxArray *c3_y = NULL;
  SFc3_Robot_Control_v01InstanceStruct *chartInstance;
  chartInstance = (SFc3_Robot_Control_v01InstanceStruct *)chartInstanceVoid;
  c3_mxArrayOutData = NULL;
  c3_i297 = 0;
  for (c3_i298 = 0; c3_i298 < 3; c3_i298++) {
    for (c3_i299 = 0; c3_i299 < 3; c3_i299++) {
      c3_b_inData[c3_i299 + c3_i297] = (*(real_T (*)[9])c3_inData)[c3_i299 +
        c3_i297];
    }

    c3_i297 += 3;
  }

  c3_i300 = 0;
  for (c3_i301 = 0; c3_i301 < 3; c3_i301++) {
    for (c3_i302 = 0; c3_i302 < 3; c3_i302++) {
      c3_u[c3_i302 + c3_i300] = c3_b_inData[c3_i302 + c3_i300];
    }

    c3_i300 += 3;
  }

  c3_y = NULL;
  sf_mex_assign(&c3_y, sf_mex_create("y", c3_u, 0, 0U, 1U, 0U, 2, 3, 3), false);
  sf_mex_assign(&c3_mxArrayOutData, c3_y, false);
  return c3_mxArrayOutData;
}

static void c3_w_emlrt_marshallIn(SFc3_Robot_Control_v01InstanceStruct
  *chartInstance, const mxArray *c3_u, const emlrtMsgIdentifier *c3_parentId,
  real_T c3_y[9])
{
  real_T c3_dv32[9];
  int32_T c3_i303;
  (void)chartInstance;
  sf_mex_import(c3_parentId, sf_mex_dup(c3_u), c3_dv32, 1, 0, 0U, 1, 0U, 2, 3, 3);
  for (c3_i303 = 0; c3_i303 < 9; c3_i303++) {
    c3_y[c3_i303] = c3_dv32[c3_i303];
  }

  sf_mex_destroy(&c3_u);
}

static void c3_t_sf_marshallIn(void *chartInstanceVoid, const mxArray
  *c3_mxArrayInData, const char_T *c3_varName, void *c3_outData)
{
  const mxArray *c3_rotMat;
  const char_T *c3_identifier;
  emlrtMsgIdentifier c3_thisId;
  real_T c3_y[9];
  int32_T c3_i304;
  int32_T c3_i305;
  int32_T c3_i306;
  SFc3_Robot_Control_v01InstanceStruct *chartInstance;
  chartInstance = (SFc3_Robot_Control_v01InstanceStruct *)chartInstanceVoid;
  c3_rotMat = sf_mex_dup(c3_mxArrayInData);
  c3_identifier = c3_varName;
  c3_thisId.fIdentifier = c3_identifier;
  c3_thisId.fParent = NULL;
  c3_w_emlrt_marshallIn(chartInstance, sf_mex_dup(c3_rotMat), &c3_thisId, c3_y);
  sf_mex_destroy(&c3_rotMat);
  c3_i304 = 0;
  for (c3_i305 = 0; c3_i305 < 3; c3_i305++) {
    for (c3_i306 = 0; c3_i306 < 3; c3_i306++) {
      (*(real_T (*)[9])c3_outData)[c3_i306 + c3_i304] = c3_y[c3_i306 + c3_i304];
    }

    c3_i304 += 3;
  }

  sf_mex_destroy(&c3_mxArrayInData);
}

const mxArray *sf_c3_Robot_Control_v01_get_eml_resolved_functions_info(void)
{
  const mxArray *c3_nameCaptureInfo = NULL;
  c3_nameCaptureInfo = NULL;
  sf_mex_assign(&c3_nameCaptureInfo, sf_mex_createstruct("structure", 2, 356, 1),
                false);
  c3_info_helper(&c3_nameCaptureInfo);
  c3_b_info_helper(&c3_nameCaptureInfo);
  c3_c_info_helper(&c3_nameCaptureInfo);
  c3_d_info_helper(&c3_nameCaptureInfo);
  c3_e_info_helper(&c3_nameCaptureInfo);
  c3_f_info_helper(&c3_nameCaptureInfo);
  sf_mex_emlrtNameCapturePostProcessR2012a(&c3_nameCaptureInfo);
  return c3_nameCaptureInfo;
}

static void c3_info_helper(const mxArray **c3_info)
{
  const mxArray *c3_rhs0 = NULL;
  const mxArray *c3_lhs0 = NULL;
  const mxArray *c3_rhs1 = NULL;
  const mxArray *c3_lhs1 = NULL;
  const mxArray *c3_rhs2 = NULL;
  const mxArray *c3_lhs2 = NULL;
  const mxArray *c3_rhs3 = NULL;
  const mxArray *c3_lhs3 = NULL;
  const mxArray *c3_rhs4 = NULL;
  const mxArray *c3_lhs4 = NULL;
  const mxArray *c3_rhs5 = NULL;
  const mxArray *c3_lhs5 = NULL;
  const mxArray *c3_rhs6 = NULL;
  const mxArray *c3_lhs6 = NULL;
  const mxArray *c3_rhs7 = NULL;
  const mxArray *c3_lhs7 = NULL;
  const mxArray *c3_rhs8 = NULL;
  const mxArray *c3_lhs8 = NULL;
  const mxArray *c3_rhs9 = NULL;
  const mxArray *c3_lhs9 = NULL;
  const mxArray *c3_rhs10 = NULL;
  const mxArray *c3_lhs10 = NULL;
  const mxArray *c3_rhs11 = NULL;
  const mxArray *c3_lhs11 = NULL;
  const mxArray *c3_rhs12 = NULL;
  const mxArray *c3_lhs12 = NULL;
  const mxArray *c3_rhs13 = NULL;
  const mxArray *c3_lhs13 = NULL;
  const mxArray *c3_rhs14 = NULL;
  const mxArray *c3_lhs14 = NULL;
  const mxArray *c3_rhs15 = NULL;
  const mxArray *c3_lhs15 = NULL;
  const mxArray *c3_rhs16 = NULL;
  const mxArray *c3_lhs16 = NULL;
  const mxArray *c3_rhs17 = NULL;
  const mxArray *c3_lhs17 = NULL;
  const mxArray *c3_rhs18 = NULL;
  const mxArray *c3_lhs18 = NULL;
  const mxArray *c3_rhs19 = NULL;
  const mxArray *c3_lhs19 = NULL;
  const mxArray *c3_rhs20 = NULL;
  const mxArray *c3_lhs20 = NULL;
  const mxArray *c3_rhs21 = NULL;
  const mxArray *c3_lhs21 = NULL;
  const mxArray *c3_rhs22 = NULL;
  const mxArray *c3_lhs22 = NULL;
  const mxArray *c3_rhs23 = NULL;
  const mxArray *c3_lhs23 = NULL;
  const mxArray *c3_rhs24 = NULL;
  const mxArray *c3_lhs24 = NULL;
  const mxArray *c3_rhs25 = NULL;
  const mxArray *c3_lhs25 = NULL;
  const mxArray *c3_rhs26 = NULL;
  const mxArray *c3_lhs26 = NULL;
  const mxArray *c3_rhs27 = NULL;
  const mxArray *c3_lhs27 = NULL;
  const mxArray *c3_rhs28 = NULL;
  const mxArray *c3_lhs28 = NULL;
  const mxArray *c3_rhs29 = NULL;
  const mxArray *c3_lhs29 = NULL;
  const mxArray *c3_rhs30 = NULL;
  const mxArray *c3_lhs30 = NULL;
  const mxArray *c3_rhs31 = NULL;
  const mxArray *c3_lhs31 = NULL;
  const mxArray *c3_rhs32 = NULL;
  const mxArray *c3_lhs32 = NULL;
  const mxArray *c3_rhs33 = NULL;
  const mxArray *c3_lhs33 = NULL;
  const mxArray *c3_rhs34 = NULL;
  const mxArray *c3_lhs34 = NULL;
  const mxArray *c3_rhs35 = NULL;
  const mxArray *c3_lhs35 = NULL;
  const mxArray *c3_rhs36 = NULL;
  const mxArray *c3_lhs36 = NULL;
  const mxArray *c3_rhs37 = NULL;
  const mxArray *c3_lhs37 = NULL;
  const mxArray *c3_rhs38 = NULL;
  const mxArray *c3_lhs38 = NULL;
  const mxArray *c3_rhs39 = NULL;
  const mxArray *c3_lhs39 = NULL;
  const mxArray *c3_rhs40 = NULL;
  const mxArray *c3_lhs40 = NULL;
  const mxArray *c3_rhs41 = NULL;
  const mxArray *c3_lhs41 = NULL;
  const mxArray *c3_rhs42 = NULL;
  const mxArray *c3_lhs42 = NULL;
  const mxArray *c3_rhs43 = NULL;
  const mxArray *c3_lhs43 = NULL;
  const mxArray *c3_rhs44 = NULL;
  const mxArray *c3_lhs44 = NULL;
  const mxArray *c3_rhs45 = NULL;
  const mxArray *c3_lhs45 = NULL;
  const mxArray *c3_rhs46 = NULL;
  const mxArray *c3_lhs46 = NULL;
  const mxArray *c3_rhs47 = NULL;
  const mxArray *c3_lhs47 = NULL;
  const mxArray *c3_rhs48 = NULL;
  const mxArray *c3_lhs48 = NULL;
  const mxArray *c3_rhs49 = NULL;
  const mxArray *c3_lhs49 = NULL;
  const mxArray *c3_rhs50 = NULL;
  const mxArray *c3_lhs50 = NULL;
  const mxArray *c3_rhs51 = NULL;
  const mxArray *c3_lhs51 = NULL;
  const mxArray *c3_rhs52 = NULL;
  const mxArray *c3_lhs52 = NULL;
  const mxArray *c3_rhs53 = NULL;
  const mxArray *c3_lhs53 = NULL;
  const mxArray *c3_rhs54 = NULL;
  const mxArray *c3_lhs54 = NULL;
  const mxArray *c3_rhs55 = NULL;
  const mxArray *c3_lhs55 = NULL;
  const mxArray *c3_rhs56 = NULL;
  const mxArray *c3_lhs56 = NULL;
  const mxArray *c3_rhs57 = NULL;
  const mxArray *c3_lhs57 = NULL;
  const mxArray *c3_rhs58 = NULL;
  const mxArray *c3_lhs58 = NULL;
  const mxArray *c3_rhs59 = NULL;
  const mxArray *c3_lhs59 = NULL;
  const mxArray *c3_rhs60 = NULL;
  const mxArray *c3_lhs60 = NULL;
  const mxArray *c3_rhs61 = NULL;
  const mxArray *c3_lhs61 = NULL;
  const mxArray *c3_rhs62 = NULL;
  const mxArray *c3_lhs62 = NULL;
  const mxArray *c3_rhs63 = NULL;
  const mxArray *c3_lhs63 = NULL;
  sf_mex_addfield(*c3_info, c3_emlrt_marshallOut(""), "context", "context", 0);
  sf_mex_addfield(*c3_info, c3_emlrt_marshallOut("random"), "name", "name", 0);
  sf_mex_addfield(*c3_info, c3_emlrt_marshallOut("double"), "dominantType",
                  "dominantType", 0);
  sf_mex_addfield(*c3_info, c3_emlrt_marshallOut(
    "[E]C:/Users/sd0016/Dropbox/UAH Doktora/Journal Tracking/Matlab Code/3DOF_Experiments/random.m"),
                  "resolved", "resolved", 0);
  sf_mex_addfield(*c3_info, c3_b_emlrt_marshallOut(1422130658U), "fileTimeLo",
                  "fileTimeLo", 0);
  sf_mex_addfield(*c3_info, c3_b_emlrt_marshallOut(0U), "fileTimeHi",
                  "fileTimeHi", 0);
  sf_mex_addfield(*c3_info, c3_b_emlrt_marshallOut(0U), "mFileTimeLo",
                  "mFileTimeLo", 0);
  sf_mex_addfield(*c3_info, c3_b_emlrt_marshallOut(0U), "mFileTimeHi",
                  "mFileTimeHi", 0);
  sf_mex_assign(&c3_rhs0, sf_mex_createcellmatrix(0, 1), false);
  sf_mex_assign(&c3_lhs0, sf_mex_createcellmatrix(0, 1), false);
  sf_mex_addfield(*c3_info, sf_mex_duplicatearraysafe(&c3_rhs0), "rhs", "rhs", 0);
  sf_mex_addfield(*c3_info, sf_mex_duplicatearraysafe(&c3_lhs0), "lhs", "lhs", 0);
  sf_mex_addfield(*c3_info, c3_emlrt_marshallOut(
    "[E]C:/Users/sd0016/Dropbox/UAH Doktora/Journal Tracking/Matlab Code/3DOF_Experiments/random.m"),
                  "context", "context", 1);
  sf_mex_addfield(*c3_info, c3_emlrt_marshallOut("rand"), "name", "name", 1);
  sf_mex_addfield(*c3_info, c3_emlrt_marshallOut("double"), "dominantType",
                  "dominantType", 1);
  sf_mex_addfield(*c3_info, c3_emlrt_marshallOut(
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/randfun/rand.m"), "resolved",
                  "resolved", 1);
  sf_mex_addfield(*c3_info, c3_b_emlrt_marshallOut(1383902490U), "fileTimeLo",
                  "fileTimeLo", 1);
  sf_mex_addfield(*c3_info, c3_b_emlrt_marshallOut(0U), "fileTimeHi",
                  "fileTimeHi", 1);
  sf_mex_addfield(*c3_info, c3_b_emlrt_marshallOut(0U), "mFileTimeLo",
                  "mFileTimeLo", 1);
  sf_mex_addfield(*c3_info, c3_b_emlrt_marshallOut(0U), "mFileTimeHi",
                  "mFileTimeHi", 1);
  sf_mex_assign(&c3_rhs1, sf_mex_createcellmatrix(0, 1), false);
  sf_mex_assign(&c3_lhs1, sf_mex_createcellmatrix(0, 1), false);
  sf_mex_addfield(*c3_info, sf_mex_duplicatearraysafe(&c3_rhs1), "rhs", "rhs", 1);
  sf_mex_addfield(*c3_info, sf_mex_duplicatearraysafe(&c3_lhs1), "lhs", "lhs", 1);
  sf_mex_addfield(*c3_info, c3_emlrt_marshallOut(
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/randfun/rand.m"), "context",
                  "context", 2);
  sf_mex_addfield(*c3_info, c3_emlrt_marshallOut("eml_is_rand_extrinsic"),
                  "name", "name", 2);
  sf_mex_addfield(*c3_info, c3_emlrt_marshallOut(""), "dominantType",
                  "dominantType", 2);
  sf_mex_addfield(*c3_info, c3_emlrt_marshallOut(
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/randfun/eml_is_rand_extrinsic.m"),
                  "resolved", "resolved", 2);
  sf_mex_addfield(*c3_info, c3_b_emlrt_marshallOut(1368208232U), "fileTimeLo",
                  "fileTimeLo", 2);
  sf_mex_addfield(*c3_info, c3_b_emlrt_marshallOut(0U), "fileTimeHi",
                  "fileTimeHi", 2);
  sf_mex_addfield(*c3_info, c3_b_emlrt_marshallOut(0U), "mFileTimeLo",
                  "mFileTimeLo", 2);
  sf_mex_addfield(*c3_info, c3_b_emlrt_marshallOut(0U), "mFileTimeHi",
                  "mFileTimeHi", 2);
  sf_mex_assign(&c3_rhs2, sf_mex_createcellmatrix(0, 1), false);
  sf_mex_assign(&c3_lhs2, sf_mex_createcellmatrix(0, 1), false);
  sf_mex_addfield(*c3_info, sf_mex_duplicatearraysafe(&c3_rhs2), "rhs", "rhs", 2);
  sf_mex_addfield(*c3_info, sf_mex_duplicatearraysafe(&c3_lhs2), "lhs", "lhs", 2);
  sf_mex_addfield(*c3_info, c3_emlrt_marshallOut(
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/randfun/rand.m"), "context",
                  "context", 3);
  sf_mex_addfield(*c3_info, c3_emlrt_marshallOut("eml_rand"), "name", "name", 3);
  sf_mex_addfield(*c3_info, c3_emlrt_marshallOut("char"), "dominantType",
                  "dominantType", 3);
  sf_mex_addfield(*c3_info, c3_emlrt_marshallOut(
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/randfun/eml_rand.m"), "resolved",
                  "resolved", 3);
  sf_mex_addfield(*c3_info, c3_b_emlrt_marshallOut(1313373020U), "fileTimeLo",
                  "fileTimeLo", 3);
  sf_mex_addfield(*c3_info, c3_b_emlrt_marshallOut(0U), "fileTimeHi",
                  "fileTimeHi", 3);
  sf_mex_addfield(*c3_info, c3_b_emlrt_marshallOut(0U), "mFileTimeLo",
                  "mFileTimeLo", 3);
  sf_mex_addfield(*c3_info, c3_b_emlrt_marshallOut(0U), "mFileTimeHi",
                  "mFileTimeHi", 3);
  sf_mex_assign(&c3_rhs3, sf_mex_createcellmatrix(0, 1), false);
  sf_mex_assign(&c3_lhs3, sf_mex_createcellmatrix(0, 1), false);
  sf_mex_addfield(*c3_info, sf_mex_duplicatearraysafe(&c3_rhs3), "rhs", "rhs", 3);
  sf_mex_addfield(*c3_info, sf_mex_duplicatearraysafe(&c3_lhs3), "lhs", "lhs", 3);
  sf_mex_addfield(*c3_info, c3_emlrt_marshallOut(
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/randfun/eml_rand.m"), "context",
                  "context", 4);
  sf_mex_addfield(*c3_info, c3_emlrt_marshallOut("eml_rand_str2id"), "name",
                  "name", 4);
  sf_mex_addfield(*c3_info, c3_emlrt_marshallOut("char"), "dominantType",
                  "dominantType", 4);
  sf_mex_addfield(*c3_info, c3_emlrt_marshallOut(
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/randfun/eml_rand_str2id.m"),
                  "resolved", "resolved", 4);
  sf_mex_addfield(*c3_info, c3_b_emlrt_marshallOut(1313373022U), "fileTimeLo",
                  "fileTimeLo", 4);
  sf_mex_addfield(*c3_info, c3_b_emlrt_marshallOut(0U), "fileTimeHi",
                  "fileTimeHi", 4);
  sf_mex_addfield(*c3_info, c3_b_emlrt_marshallOut(0U), "mFileTimeLo",
                  "mFileTimeLo", 4);
  sf_mex_addfield(*c3_info, c3_b_emlrt_marshallOut(0U), "mFileTimeHi",
                  "mFileTimeHi", 4);
  sf_mex_assign(&c3_rhs4, sf_mex_createcellmatrix(0, 1), false);
  sf_mex_assign(&c3_lhs4, sf_mex_createcellmatrix(0, 1), false);
  sf_mex_addfield(*c3_info, sf_mex_duplicatearraysafe(&c3_rhs4), "rhs", "rhs", 4);
  sf_mex_addfield(*c3_info, sf_mex_duplicatearraysafe(&c3_lhs4), "lhs", "lhs", 4);
  sf_mex_addfield(*c3_info, c3_emlrt_marshallOut(
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/randfun/eml_rand_str2id.m"),
                  "context", "context", 5);
  sf_mex_addfield(*c3_info, c3_emlrt_marshallOut("eml_switch_helper"), "name",
                  "name", 5);
  sf_mex_addfield(*c3_info, c3_emlrt_marshallOut(""), "dominantType",
                  "dominantType", 5);
  sf_mex_addfield(*c3_info, c3_emlrt_marshallOut(
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/eml/eml_switch_helper.m"),
                  "resolved", "resolved", 5);
  sf_mex_addfield(*c3_info, c3_b_emlrt_marshallOut(1393356058U), "fileTimeLo",
                  "fileTimeLo", 5);
  sf_mex_addfield(*c3_info, c3_b_emlrt_marshallOut(0U), "fileTimeHi",
                  "fileTimeHi", 5);
  sf_mex_addfield(*c3_info, c3_b_emlrt_marshallOut(0U), "mFileTimeLo",
                  "mFileTimeLo", 5);
  sf_mex_addfield(*c3_info, c3_b_emlrt_marshallOut(0U), "mFileTimeHi",
                  "mFileTimeHi", 5);
  sf_mex_assign(&c3_rhs5, sf_mex_createcellmatrix(0, 1), false);
  sf_mex_assign(&c3_lhs5, sf_mex_createcellmatrix(0, 1), false);
  sf_mex_addfield(*c3_info, sf_mex_duplicatearraysafe(&c3_rhs5), "rhs", "rhs", 5);
  sf_mex_addfield(*c3_info, sf_mex_duplicatearraysafe(&c3_lhs5), "lhs", "lhs", 5);
  sf_mex_addfield(*c3_info, c3_emlrt_marshallOut(
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/randfun/eml_rand.m"), "context",
                  "context", 6);
  sf_mex_addfield(*c3_info, c3_emlrt_marshallOut("eml_rand_mcg16807_stateful"),
                  "name", "name", 6);
  sf_mex_addfield(*c3_info, c3_emlrt_marshallOut("char"), "dominantType",
                  "dominantType", 6);
  sf_mex_addfield(*c3_info, c3_emlrt_marshallOut(
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/randfun/eml_rand_mcg16807_stateful.m"),
                  "resolved", "resolved", 6);
  sf_mex_addfield(*c3_info, c3_b_emlrt_marshallOut(1366187444U), "fileTimeLo",
                  "fileTimeLo", 6);
  sf_mex_addfield(*c3_info, c3_b_emlrt_marshallOut(0U), "fileTimeHi",
                  "fileTimeHi", 6);
  sf_mex_addfield(*c3_info, c3_b_emlrt_marshallOut(0U), "mFileTimeLo",
                  "mFileTimeLo", 6);
  sf_mex_addfield(*c3_info, c3_b_emlrt_marshallOut(0U), "mFileTimeHi",
                  "mFileTimeHi", 6);
  sf_mex_assign(&c3_rhs6, sf_mex_createcellmatrix(0, 1), false);
  sf_mex_assign(&c3_lhs6, sf_mex_createcellmatrix(0, 1), false);
  sf_mex_addfield(*c3_info, sf_mex_duplicatearraysafe(&c3_rhs6), "rhs", "rhs", 6);
  sf_mex_addfield(*c3_info, sf_mex_duplicatearraysafe(&c3_lhs6), "lhs", "lhs", 6);
  sf_mex_addfield(*c3_info, c3_emlrt_marshallOut(
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/randfun/eml_rand_mcg16807_stateful.m"),
                  "context", "context", 7);
  sf_mex_addfield(*c3_info, c3_emlrt_marshallOut("eml_rand_mcg16807"), "name",
                  "name", 7);
  sf_mex_addfield(*c3_info, c3_emlrt_marshallOut("char"), "dominantType",
                  "dominantType", 7);
  sf_mex_addfield(*c3_info, c3_emlrt_marshallOut(
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/randfun/eml_rand_mcg16807.m"),
                  "resolved", "resolved", 7);
  sf_mex_addfield(*c3_info, c3_b_emlrt_marshallOut(1313373020U), "fileTimeLo",
                  "fileTimeLo", 7);
  sf_mex_addfield(*c3_info, c3_b_emlrt_marshallOut(0U), "fileTimeHi",
                  "fileTimeHi", 7);
  sf_mex_addfield(*c3_info, c3_b_emlrt_marshallOut(0U), "mFileTimeLo",
                  "mFileTimeLo", 7);
  sf_mex_addfield(*c3_info, c3_b_emlrt_marshallOut(0U), "mFileTimeHi",
                  "mFileTimeHi", 7);
  sf_mex_assign(&c3_rhs7, sf_mex_createcellmatrix(0, 1), false);
  sf_mex_assign(&c3_lhs7, sf_mex_createcellmatrix(0, 1), false);
  sf_mex_addfield(*c3_info, sf_mex_duplicatearraysafe(&c3_rhs7), "rhs", "rhs", 7);
  sf_mex_addfield(*c3_info, sf_mex_duplicatearraysafe(&c3_lhs7), "lhs", "lhs", 7);
  sf_mex_addfield(*c3_info, c3_emlrt_marshallOut(
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/randfun/eml_rand_mcg16807_stateful.m"),
                  "context", "context", 8);
  sf_mex_addfield(*c3_info, c3_emlrt_marshallOut("eml_rand_mcg16807"), "name",
                  "name", 8);
  sf_mex_addfield(*c3_info, c3_emlrt_marshallOut("uint32"), "dominantType",
                  "dominantType", 8);
  sf_mex_addfield(*c3_info, c3_emlrt_marshallOut(
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/randfun/eml_rand_mcg16807.m"),
                  "resolved", "resolved", 8);
  sf_mex_addfield(*c3_info, c3_b_emlrt_marshallOut(1313373020U), "fileTimeLo",
                  "fileTimeLo", 8);
  sf_mex_addfield(*c3_info, c3_b_emlrt_marshallOut(0U), "fileTimeHi",
                  "fileTimeHi", 8);
  sf_mex_addfield(*c3_info, c3_b_emlrt_marshallOut(0U), "mFileTimeLo",
                  "mFileTimeLo", 8);
  sf_mex_addfield(*c3_info, c3_b_emlrt_marshallOut(0U), "mFileTimeHi",
                  "mFileTimeHi", 8);
  sf_mex_assign(&c3_rhs8, sf_mex_createcellmatrix(0, 1), false);
  sf_mex_assign(&c3_lhs8, sf_mex_createcellmatrix(0, 1), false);
  sf_mex_addfield(*c3_info, sf_mex_duplicatearraysafe(&c3_rhs8), "rhs", "rhs", 8);
  sf_mex_addfield(*c3_info, sf_mex_duplicatearraysafe(&c3_lhs8), "lhs", "lhs", 8);
  sf_mex_addfield(*c3_info, c3_emlrt_marshallOut(
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/randfun/eml_rand.m"), "context",
                  "context", 9);
  sf_mex_addfield(*c3_info, c3_emlrt_marshallOut("eml_rand_shr3cong_stateful"),
                  "name", "name", 9);
  sf_mex_addfield(*c3_info, c3_emlrt_marshallOut("char"), "dominantType",
                  "dominantType", 9);
  sf_mex_addfield(*c3_info, c3_emlrt_marshallOut(
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/randfun/eml_rand_shr3cong_stateful.m"),
                  "resolved", "resolved", 9);
  sf_mex_addfield(*c3_info, c3_b_emlrt_marshallOut(1366187444U), "fileTimeLo",
                  "fileTimeLo", 9);
  sf_mex_addfield(*c3_info, c3_b_emlrt_marshallOut(0U), "fileTimeHi",
                  "fileTimeHi", 9);
  sf_mex_addfield(*c3_info, c3_b_emlrt_marshallOut(0U), "mFileTimeLo",
                  "mFileTimeLo", 9);
  sf_mex_addfield(*c3_info, c3_b_emlrt_marshallOut(0U), "mFileTimeHi",
                  "mFileTimeHi", 9);
  sf_mex_assign(&c3_rhs9, sf_mex_createcellmatrix(0, 1), false);
  sf_mex_assign(&c3_lhs9, sf_mex_createcellmatrix(0, 1), false);
  sf_mex_addfield(*c3_info, sf_mex_duplicatearraysafe(&c3_rhs9), "rhs", "rhs", 9);
  sf_mex_addfield(*c3_info, sf_mex_duplicatearraysafe(&c3_lhs9), "lhs", "lhs", 9);
  sf_mex_addfield(*c3_info, c3_emlrt_marshallOut(
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/randfun/eml_rand_shr3cong_stateful.m"),
                  "context", "context", 10);
  sf_mex_addfield(*c3_info, c3_emlrt_marshallOut("eml_rand_shr3cong"), "name",
                  "name", 10);
  sf_mex_addfield(*c3_info, c3_emlrt_marshallOut("char"), "dominantType",
                  "dominantType", 10);
  sf_mex_addfield(*c3_info, c3_emlrt_marshallOut(
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/randfun/eml_rand_shr3cong.m"),
                  "resolved", "resolved", 10);
  sf_mex_addfield(*c3_info, c3_b_emlrt_marshallOut(1313373020U), "fileTimeLo",
                  "fileTimeLo", 10);
  sf_mex_addfield(*c3_info, c3_b_emlrt_marshallOut(0U), "fileTimeHi",
                  "fileTimeHi", 10);
  sf_mex_addfield(*c3_info, c3_b_emlrt_marshallOut(0U), "mFileTimeLo",
                  "mFileTimeLo", 10);
  sf_mex_addfield(*c3_info, c3_b_emlrt_marshallOut(0U), "mFileTimeHi",
                  "mFileTimeHi", 10);
  sf_mex_assign(&c3_rhs10, sf_mex_createcellmatrix(0, 1), false);
  sf_mex_assign(&c3_lhs10, sf_mex_createcellmatrix(0, 1), false);
  sf_mex_addfield(*c3_info, sf_mex_duplicatearraysafe(&c3_rhs10), "rhs", "rhs",
                  10);
  sf_mex_addfield(*c3_info, sf_mex_duplicatearraysafe(&c3_lhs10), "lhs", "lhs",
                  10);
  sf_mex_addfield(*c3_info, c3_emlrt_marshallOut(
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/randfun/eml_rand_shr3cong_stateful.m"),
                  "context", "context", 11);
  sf_mex_addfield(*c3_info, c3_emlrt_marshallOut("eml_rand_shr3cong"), "name",
                  "name", 11);
  sf_mex_addfield(*c3_info, c3_emlrt_marshallOut("uint32"), "dominantType",
                  "dominantType", 11);
  sf_mex_addfield(*c3_info, c3_emlrt_marshallOut(
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/randfun/eml_rand_shr3cong.m"),
                  "resolved", "resolved", 11);
  sf_mex_addfield(*c3_info, c3_b_emlrt_marshallOut(1313373020U), "fileTimeLo",
                  "fileTimeLo", 11);
  sf_mex_addfield(*c3_info, c3_b_emlrt_marshallOut(0U), "fileTimeHi",
                  "fileTimeHi", 11);
  sf_mex_addfield(*c3_info, c3_b_emlrt_marshallOut(0U), "mFileTimeLo",
                  "mFileTimeLo", 11);
  sf_mex_addfield(*c3_info, c3_b_emlrt_marshallOut(0U), "mFileTimeHi",
                  "mFileTimeHi", 11);
  sf_mex_assign(&c3_rhs11, sf_mex_createcellmatrix(0, 1), false);
  sf_mex_assign(&c3_lhs11, sf_mex_createcellmatrix(0, 1), false);
  sf_mex_addfield(*c3_info, sf_mex_duplicatearraysafe(&c3_rhs11), "rhs", "rhs",
                  11);
  sf_mex_addfield(*c3_info, sf_mex_duplicatearraysafe(&c3_lhs11), "lhs", "lhs",
                  11);
  sf_mex_addfield(*c3_info, c3_emlrt_marshallOut(
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/randfun/eml_rand.m"), "context",
                  "context", 12);
  sf_mex_addfield(*c3_info, c3_emlrt_marshallOut("eml_rand_mt19937ar_stateful"),
                  "name", "name", 12);
  sf_mex_addfield(*c3_info, c3_emlrt_marshallOut("char"), "dominantType",
                  "dominantType", 12);
  sf_mex_addfield(*c3_info, c3_emlrt_marshallOut(
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/randfun/eml_rand_mt19937ar_stateful.m"),
                  "resolved", "resolved", 12);
  sf_mex_addfield(*c3_info, c3_b_emlrt_marshallOut(1366187444U), "fileTimeLo",
                  "fileTimeLo", 12);
  sf_mex_addfield(*c3_info, c3_b_emlrt_marshallOut(0U), "fileTimeHi",
                  "fileTimeHi", 12);
  sf_mex_addfield(*c3_info, c3_b_emlrt_marshallOut(0U), "mFileTimeLo",
                  "mFileTimeLo", 12);
  sf_mex_addfield(*c3_info, c3_b_emlrt_marshallOut(0U), "mFileTimeHi",
                  "mFileTimeHi", 12);
  sf_mex_assign(&c3_rhs12, sf_mex_createcellmatrix(0, 1), false);
  sf_mex_assign(&c3_lhs12, sf_mex_createcellmatrix(0, 1), false);
  sf_mex_addfield(*c3_info, sf_mex_duplicatearraysafe(&c3_rhs12), "rhs", "rhs",
                  12);
  sf_mex_addfield(*c3_info, sf_mex_duplicatearraysafe(&c3_lhs12), "lhs", "lhs",
                  12);
  sf_mex_addfield(*c3_info, c3_emlrt_marshallOut(
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/randfun/eml_rand_mt19937ar_stateful.m"),
                  "context", "context", 13);
  sf_mex_addfield(*c3_info, c3_emlrt_marshallOut("eml_rand_mt19937ar"), "name",
                  "name", 13);
  sf_mex_addfield(*c3_info, c3_emlrt_marshallOut("char"), "dominantType",
                  "dominantType", 13);
  sf_mex_addfield(*c3_info, c3_emlrt_marshallOut(
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/randfun/eml_rand_mt19937ar.m"),
                  "resolved", "resolved", 13);
  sf_mex_addfield(*c3_info, c3_b_emlrt_marshallOut(1406838348U), "fileTimeLo",
                  "fileTimeLo", 13);
  sf_mex_addfield(*c3_info, c3_b_emlrt_marshallOut(0U), "fileTimeHi",
                  "fileTimeHi", 13);
  sf_mex_addfield(*c3_info, c3_b_emlrt_marshallOut(0U), "mFileTimeLo",
                  "mFileTimeLo", 13);
  sf_mex_addfield(*c3_info, c3_b_emlrt_marshallOut(0U), "mFileTimeHi",
                  "mFileTimeHi", 13);
  sf_mex_assign(&c3_rhs13, sf_mex_createcellmatrix(0, 1), false);
  sf_mex_assign(&c3_lhs13, sf_mex_createcellmatrix(0, 1), false);
  sf_mex_addfield(*c3_info, sf_mex_duplicatearraysafe(&c3_rhs13), "rhs", "rhs",
                  13);
  sf_mex_addfield(*c3_info, sf_mex_duplicatearraysafe(&c3_lhs13), "lhs", "lhs",
                  13);
  sf_mex_addfield(*c3_info, c3_emlrt_marshallOut(
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/randfun/eml_rand_mt19937ar_stateful.m"),
                  "context", "context", 14);
  sf_mex_addfield(*c3_info, c3_emlrt_marshallOut("eml_rand_mt19937ar"), "name",
                  "name", 14);
  sf_mex_addfield(*c3_info, c3_emlrt_marshallOut("uint32"), "dominantType",
                  "dominantType", 14);
  sf_mex_addfield(*c3_info, c3_emlrt_marshallOut(
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/randfun/eml_rand_mt19937ar.m"),
                  "resolved", "resolved", 14);
  sf_mex_addfield(*c3_info, c3_b_emlrt_marshallOut(1406838348U), "fileTimeLo",
                  "fileTimeLo", 14);
  sf_mex_addfield(*c3_info, c3_b_emlrt_marshallOut(0U), "fileTimeHi",
                  "fileTimeHi", 14);
  sf_mex_addfield(*c3_info, c3_b_emlrt_marshallOut(0U), "mFileTimeLo",
                  "mFileTimeLo", 14);
  sf_mex_addfield(*c3_info, c3_b_emlrt_marshallOut(0U), "mFileTimeHi",
                  "mFileTimeHi", 14);
  sf_mex_assign(&c3_rhs14, sf_mex_createcellmatrix(0, 1), false);
  sf_mex_assign(&c3_lhs14, sf_mex_createcellmatrix(0, 1), false);
  sf_mex_addfield(*c3_info, sf_mex_duplicatearraysafe(&c3_rhs14), "rhs", "rhs",
                  14);
  sf_mex_addfield(*c3_info, sf_mex_duplicatearraysafe(&c3_lhs14), "lhs", "lhs",
                  14);
  sf_mex_addfield(*c3_info, c3_emlrt_marshallOut(
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/randfun/eml_rand_mt19937ar.m!genrandu"),
                  "context", "context", 15);
  sf_mex_addfield(*c3_info, c3_emlrt_marshallOut("eps"), "name", "name", 15);
  sf_mex_addfield(*c3_info, c3_emlrt_marshallOut(""), "dominantType",
                  "dominantType", 15);
  sf_mex_addfield(*c3_info, c3_emlrt_marshallOut(
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/elmat/eps.m"), "resolved",
                  "resolved", 15);
  sf_mex_addfield(*c3_info, c3_b_emlrt_marshallOut(1326753196U), "fileTimeLo",
                  "fileTimeLo", 15);
  sf_mex_addfield(*c3_info, c3_b_emlrt_marshallOut(0U), "fileTimeHi",
                  "fileTimeHi", 15);
  sf_mex_addfield(*c3_info, c3_b_emlrt_marshallOut(0U), "mFileTimeLo",
                  "mFileTimeLo", 15);
  sf_mex_addfield(*c3_info, c3_b_emlrt_marshallOut(0U), "mFileTimeHi",
                  "mFileTimeHi", 15);
  sf_mex_assign(&c3_rhs15, sf_mex_createcellmatrix(0, 1), false);
  sf_mex_assign(&c3_lhs15, sf_mex_createcellmatrix(0, 1), false);
  sf_mex_addfield(*c3_info, sf_mex_duplicatearraysafe(&c3_rhs15), "rhs", "rhs",
                  15);
  sf_mex_addfield(*c3_info, sf_mex_duplicatearraysafe(&c3_lhs15), "lhs", "lhs",
                  15);
  sf_mex_addfield(*c3_info, c3_emlrt_marshallOut(
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/elmat/eps.m"), "context",
                  "context", 16);
  sf_mex_addfield(*c3_info, c3_emlrt_marshallOut("eml_eps"), "name", "name", 16);
  sf_mex_addfield(*c3_info, c3_emlrt_marshallOut("char"), "dominantType",
                  "dominantType", 16);
  sf_mex_addfield(*c3_info, c3_emlrt_marshallOut(
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/eml/eml_eps.m"), "resolved",
                  "resolved", 16);
  sf_mex_addfield(*c3_info, c3_b_emlrt_marshallOut(1326753196U), "fileTimeLo",
                  "fileTimeLo", 16);
  sf_mex_addfield(*c3_info, c3_b_emlrt_marshallOut(0U), "fileTimeHi",
                  "fileTimeHi", 16);
  sf_mex_addfield(*c3_info, c3_b_emlrt_marshallOut(0U), "mFileTimeLo",
                  "mFileTimeLo", 16);
  sf_mex_addfield(*c3_info, c3_b_emlrt_marshallOut(0U), "mFileTimeHi",
                  "mFileTimeHi", 16);
  sf_mex_assign(&c3_rhs16, sf_mex_createcellmatrix(0, 1), false);
  sf_mex_assign(&c3_lhs16, sf_mex_createcellmatrix(0, 1), false);
  sf_mex_addfield(*c3_info, sf_mex_duplicatearraysafe(&c3_rhs16), "rhs", "rhs",
                  16);
  sf_mex_addfield(*c3_info, sf_mex_duplicatearraysafe(&c3_lhs16), "lhs", "lhs",
                  16);
  sf_mex_addfield(*c3_info, c3_emlrt_marshallOut(
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/eml/eml_eps.m"), "context",
                  "context", 17);
  sf_mex_addfield(*c3_info, c3_emlrt_marshallOut("eml_float_model"), "name",
                  "name", 17);
  sf_mex_addfield(*c3_info, c3_emlrt_marshallOut("char"), "dominantType",
                  "dominantType", 17);
  sf_mex_addfield(*c3_info, c3_emlrt_marshallOut(
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/eml/eml_float_model.m"),
                  "resolved", "resolved", 17);
  sf_mex_addfield(*c3_info, c3_b_emlrt_marshallOut(1326753196U), "fileTimeLo",
                  "fileTimeLo", 17);
  sf_mex_addfield(*c3_info, c3_b_emlrt_marshallOut(0U), "fileTimeHi",
                  "fileTimeHi", 17);
  sf_mex_addfield(*c3_info, c3_b_emlrt_marshallOut(0U), "mFileTimeLo",
                  "mFileTimeLo", 17);
  sf_mex_addfield(*c3_info, c3_b_emlrt_marshallOut(0U), "mFileTimeHi",
                  "mFileTimeHi", 17);
  sf_mex_assign(&c3_rhs17, sf_mex_createcellmatrix(0, 1), false);
  sf_mex_assign(&c3_lhs17, sf_mex_createcellmatrix(0, 1), false);
  sf_mex_addfield(*c3_info, sf_mex_duplicatearraysafe(&c3_rhs17), "rhs", "rhs",
                  17);
  sf_mex_addfield(*c3_info, sf_mex_duplicatearraysafe(&c3_lhs17), "lhs", "lhs",
                  17);
  sf_mex_addfield(*c3_info, c3_emlrt_marshallOut(
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/randfun/eml_rand_mt19937ar.m!is_valid_state"),
                  "context", "context", 18);
  sf_mex_addfield(*c3_info, c3_emlrt_marshallOut("isequal"), "name", "name", 18);
  sf_mex_addfield(*c3_info, c3_emlrt_marshallOut("double"), "dominantType",
                  "dominantType", 18);
  sf_mex_addfield(*c3_info, c3_emlrt_marshallOut(
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/elmat/isequal.m"), "resolved",
                  "resolved", 18);
  sf_mex_addfield(*c3_info, c3_b_emlrt_marshallOut(1286843958U), "fileTimeLo",
                  "fileTimeLo", 18);
  sf_mex_addfield(*c3_info, c3_b_emlrt_marshallOut(0U), "fileTimeHi",
                  "fileTimeHi", 18);
  sf_mex_addfield(*c3_info, c3_b_emlrt_marshallOut(0U), "mFileTimeLo",
                  "mFileTimeLo", 18);
  sf_mex_addfield(*c3_info, c3_b_emlrt_marshallOut(0U), "mFileTimeHi",
                  "mFileTimeHi", 18);
  sf_mex_assign(&c3_rhs18, sf_mex_createcellmatrix(0, 1), false);
  sf_mex_assign(&c3_lhs18, sf_mex_createcellmatrix(0, 1), false);
  sf_mex_addfield(*c3_info, sf_mex_duplicatearraysafe(&c3_rhs18), "rhs", "rhs",
                  18);
  sf_mex_addfield(*c3_info, sf_mex_duplicatearraysafe(&c3_lhs18), "lhs", "lhs",
                  18);
  sf_mex_addfield(*c3_info, c3_emlrt_marshallOut(
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/elmat/isequal.m"), "context",
                  "context", 19);
  sf_mex_addfield(*c3_info, c3_emlrt_marshallOut("eml_isequal_core"), "name",
                  "name", 19);
  sf_mex_addfield(*c3_info, c3_emlrt_marshallOut("double"), "dominantType",
                  "dominantType", 19);
  sf_mex_addfield(*c3_info, c3_emlrt_marshallOut(
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/eml/eml_isequal_core.m"),
                  "resolved", "resolved", 19);
  sf_mex_addfield(*c3_info, c3_b_emlrt_marshallOut(1286843986U), "fileTimeLo",
                  "fileTimeLo", 19);
  sf_mex_addfield(*c3_info, c3_b_emlrt_marshallOut(0U), "fileTimeHi",
                  "fileTimeHi", 19);
  sf_mex_addfield(*c3_info, c3_b_emlrt_marshallOut(0U), "mFileTimeLo",
                  "mFileTimeLo", 19);
  sf_mex_addfield(*c3_info, c3_b_emlrt_marshallOut(0U), "mFileTimeHi",
                  "mFileTimeHi", 19);
  sf_mex_assign(&c3_rhs19, sf_mex_createcellmatrix(0, 1), false);
  sf_mex_assign(&c3_lhs19, sf_mex_createcellmatrix(0, 1), false);
  sf_mex_addfield(*c3_info, sf_mex_duplicatearraysafe(&c3_rhs19), "rhs", "rhs",
                  19);
  sf_mex_addfield(*c3_info, sf_mex_duplicatearraysafe(&c3_lhs19), "lhs", "lhs",
                  19);
  sf_mex_addfield(*c3_info, c3_emlrt_marshallOut(
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/eml/eml_isequal_core.m!isequal_scalar"),
                  "context", "context", 20);
  sf_mex_addfield(*c3_info, c3_emlrt_marshallOut("isnan"), "name", "name", 20);
  sf_mex_addfield(*c3_info, c3_emlrt_marshallOut("double"), "dominantType",
                  "dominantType", 20);
  sf_mex_addfield(*c3_info, c3_emlrt_marshallOut(
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/elmat/isnan.m"), "resolved",
                  "resolved", 20);
  sf_mex_addfield(*c3_info, c3_b_emlrt_marshallOut(1363735458U), "fileTimeLo",
                  "fileTimeLo", 20);
  sf_mex_addfield(*c3_info, c3_b_emlrt_marshallOut(0U), "fileTimeHi",
                  "fileTimeHi", 20);
  sf_mex_addfield(*c3_info, c3_b_emlrt_marshallOut(0U), "mFileTimeLo",
                  "mFileTimeLo", 20);
  sf_mex_addfield(*c3_info, c3_b_emlrt_marshallOut(0U), "mFileTimeHi",
                  "mFileTimeHi", 20);
  sf_mex_assign(&c3_rhs20, sf_mex_createcellmatrix(0, 1), false);
  sf_mex_assign(&c3_lhs20, sf_mex_createcellmatrix(0, 1), false);
  sf_mex_addfield(*c3_info, sf_mex_duplicatearraysafe(&c3_rhs20), "rhs", "rhs",
                  20);
  sf_mex_addfield(*c3_info, sf_mex_duplicatearraysafe(&c3_lhs20), "lhs", "lhs",
                  20);
  sf_mex_addfield(*c3_info, c3_emlrt_marshallOut(
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/elmat/isnan.m"), "context",
                  "context", 21);
  sf_mex_addfield(*c3_info, c3_emlrt_marshallOut(
    "coder.internal.isBuiltInNumeric"), "name", "name", 21);
  sf_mex_addfield(*c3_info, c3_emlrt_marshallOut("double"), "dominantType",
                  "dominantType", 21);
  sf_mex_addfield(*c3_info, c3_emlrt_marshallOut(
    "[ILXE]$matlabroot$/toolbox/shared/coder/coder/+coder/+internal/isBuiltInNumeric.m"),
                  "resolved", "resolved", 21);
  sf_mex_addfield(*c3_info, c3_b_emlrt_marshallOut(1395953456U), "fileTimeLo",
                  "fileTimeLo", 21);
  sf_mex_addfield(*c3_info, c3_b_emlrt_marshallOut(0U), "fileTimeHi",
                  "fileTimeHi", 21);
  sf_mex_addfield(*c3_info, c3_b_emlrt_marshallOut(0U), "mFileTimeLo",
                  "mFileTimeLo", 21);
  sf_mex_addfield(*c3_info, c3_b_emlrt_marshallOut(0U), "mFileTimeHi",
                  "mFileTimeHi", 21);
  sf_mex_assign(&c3_rhs21, sf_mex_createcellmatrix(0, 1), false);
  sf_mex_assign(&c3_lhs21, sf_mex_createcellmatrix(0, 1), false);
  sf_mex_addfield(*c3_info, sf_mex_duplicatearraysafe(&c3_rhs21), "rhs", "rhs",
                  21);
  sf_mex_addfield(*c3_info, sf_mex_duplicatearraysafe(&c3_lhs21), "lhs", "lhs",
                  21);
  sf_mex_addfield(*c3_info, c3_emlrt_marshallOut(
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/randfun/eml_rand_mt19937ar.m!is_valid_state"),
                  "context", "context", 22);
  sf_mex_addfield(*c3_info, c3_emlrt_marshallOut("eml_index_class"), "name",
                  "name", 22);
  sf_mex_addfield(*c3_info, c3_emlrt_marshallOut(""), "dominantType",
                  "dominantType", 22);
  sf_mex_addfield(*c3_info, c3_emlrt_marshallOut(
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/eml/eml_index_class.m"),
                  "resolved", "resolved", 22);
  sf_mex_addfield(*c3_info, c3_b_emlrt_marshallOut(1323195778U), "fileTimeLo",
                  "fileTimeLo", 22);
  sf_mex_addfield(*c3_info, c3_b_emlrt_marshallOut(0U), "fileTimeHi",
                  "fileTimeHi", 22);
  sf_mex_addfield(*c3_info, c3_b_emlrt_marshallOut(0U), "mFileTimeLo",
                  "mFileTimeLo", 22);
  sf_mex_addfield(*c3_info, c3_b_emlrt_marshallOut(0U), "mFileTimeHi",
                  "mFileTimeHi", 22);
  sf_mex_assign(&c3_rhs22, sf_mex_createcellmatrix(0, 1), false);
  sf_mex_assign(&c3_lhs22, sf_mex_createcellmatrix(0, 1), false);
  sf_mex_addfield(*c3_info, sf_mex_duplicatearraysafe(&c3_rhs22), "rhs", "rhs",
                  22);
  sf_mex_addfield(*c3_info, sf_mex_duplicatearraysafe(&c3_lhs22), "lhs", "lhs",
                  22);
  sf_mex_addfield(*c3_info, c3_emlrt_marshallOut(
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/randfun/eml_rand_mt19937ar.m!is_valid_state"),
                  "context", "context", 23);
  sf_mex_addfield(*c3_info, c3_emlrt_marshallOut("eml_index_plus"), "name",
                  "name", 23);
  sf_mex_addfield(*c3_info, c3_emlrt_marshallOut("double"), "dominantType",
                  "dominantType", 23);
  sf_mex_addfield(*c3_info, c3_emlrt_marshallOut(
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/eml/eml_index_plus.m"),
                  "resolved", "resolved", 23);
  sf_mex_addfield(*c3_info, c3_b_emlrt_marshallOut(1372607616U), "fileTimeLo",
                  "fileTimeLo", 23);
  sf_mex_addfield(*c3_info, c3_b_emlrt_marshallOut(0U), "fileTimeHi",
                  "fileTimeHi", 23);
  sf_mex_addfield(*c3_info, c3_b_emlrt_marshallOut(0U), "mFileTimeLo",
                  "mFileTimeLo", 23);
  sf_mex_addfield(*c3_info, c3_b_emlrt_marshallOut(0U), "mFileTimeHi",
                  "mFileTimeHi", 23);
  sf_mex_assign(&c3_rhs23, sf_mex_createcellmatrix(0, 1), false);
  sf_mex_assign(&c3_lhs23, sf_mex_createcellmatrix(0, 1), false);
  sf_mex_addfield(*c3_info, sf_mex_duplicatearraysafe(&c3_rhs23), "rhs", "rhs",
                  23);
  sf_mex_addfield(*c3_info, sf_mex_duplicatearraysafe(&c3_lhs23), "lhs", "lhs",
                  23);
  sf_mex_addfield(*c3_info, c3_emlrt_marshallOut(
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/eml/eml_index_plus.m"), "context",
                  "context", 24);
  sf_mex_addfield(*c3_info, c3_emlrt_marshallOut("coder.internal.indexPlus"),
                  "name", "name", 24);
  sf_mex_addfield(*c3_info, c3_emlrt_marshallOut("double"), "dominantType",
                  "dominantType", 24);
  sf_mex_addfield(*c3_info, c3_emlrt_marshallOut(
    "[ILXE]$matlabroot$/toolbox/shared/coder/coder/+coder/+internal/indexPlus.m"),
                  "resolved", "resolved", 24);
  sf_mex_addfield(*c3_info, c3_b_emlrt_marshallOut(1372608360U), "fileTimeLo",
                  "fileTimeLo", 24);
  sf_mex_addfield(*c3_info, c3_b_emlrt_marshallOut(0U), "fileTimeHi",
                  "fileTimeHi", 24);
  sf_mex_addfield(*c3_info, c3_b_emlrt_marshallOut(0U), "mFileTimeLo",
                  "mFileTimeLo", 24);
  sf_mex_addfield(*c3_info, c3_b_emlrt_marshallOut(0U), "mFileTimeHi",
                  "mFileTimeHi", 24);
  sf_mex_assign(&c3_rhs24, sf_mex_createcellmatrix(0, 1), false);
  sf_mex_assign(&c3_lhs24, sf_mex_createcellmatrix(0, 1), false);
  sf_mex_addfield(*c3_info, sf_mex_duplicatearraysafe(&c3_rhs24), "rhs", "rhs",
                  24);
  sf_mex_addfield(*c3_info, sf_mex_duplicatearraysafe(&c3_lhs24), "lhs", "lhs",
                  24);
  sf_mex_addfield(*c3_info, c3_emlrt_marshallOut(
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/randfun/eml_rand_mt19937ar.m!genrandu"),
                  "context", "context", 25);
  sf_mex_addfield(*c3_info, c3_emlrt_marshallOut("eml_error"), "name", "name",
                  25);
  sf_mex_addfield(*c3_info, c3_emlrt_marshallOut("char"), "dominantType",
                  "dominantType", 25);
  sf_mex_addfield(*c3_info, c3_emlrt_marshallOut(
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/eml/eml_error.m"), "resolved",
                  "resolved", 25);
  sf_mex_addfield(*c3_info, c3_b_emlrt_marshallOut(1343855558U), "fileTimeLo",
                  "fileTimeLo", 25);
  sf_mex_addfield(*c3_info, c3_b_emlrt_marshallOut(0U), "fileTimeHi",
                  "fileTimeHi", 25);
  sf_mex_addfield(*c3_info, c3_b_emlrt_marshallOut(0U), "mFileTimeLo",
                  "mFileTimeLo", 25);
  sf_mex_addfield(*c3_info, c3_b_emlrt_marshallOut(0U), "mFileTimeHi",
                  "mFileTimeHi", 25);
  sf_mex_assign(&c3_rhs25, sf_mex_createcellmatrix(0, 1), false);
  sf_mex_assign(&c3_lhs25, sf_mex_createcellmatrix(0, 1), false);
  sf_mex_addfield(*c3_info, sf_mex_duplicatearraysafe(&c3_rhs25), "rhs", "rhs",
                  25);
  sf_mex_addfield(*c3_info, sf_mex_duplicatearraysafe(&c3_lhs25), "lhs", "lhs",
                  25);
  sf_mex_addfield(*c3_info, c3_emlrt_marshallOut(
    "[E]C:/Users/sd0016/Dropbox/UAH Doktora/Journal Tracking/Matlab Code/3DOF_Experiments/random.m"),
                  "context", "context", 26);
  sf_mex_addfield(*c3_info, c3_emlrt_marshallOut("eml_mtimes_helper"), "name",
                  "name", 26);
  sf_mex_addfield(*c3_info, c3_emlrt_marshallOut(""), "dominantType",
                  "dominantType", 26);
  sf_mex_addfield(*c3_info, c3_emlrt_marshallOut(
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/ops/eml_mtimes_helper.m"),
                  "resolved", "resolved", 26);
  sf_mex_addfield(*c3_info, c3_b_emlrt_marshallOut(1383902494U), "fileTimeLo",
                  "fileTimeLo", 26);
  sf_mex_addfield(*c3_info, c3_b_emlrt_marshallOut(0U), "fileTimeHi",
                  "fileTimeHi", 26);
  sf_mex_addfield(*c3_info, c3_b_emlrt_marshallOut(0U), "mFileTimeLo",
                  "mFileTimeLo", 26);
  sf_mex_addfield(*c3_info, c3_b_emlrt_marshallOut(0U), "mFileTimeHi",
                  "mFileTimeHi", 26);
  sf_mex_assign(&c3_rhs26, sf_mex_createcellmatrix(0, 1), false);
  sf_mex_assign(&c3_lhs26, sf_mex_createcellmatrix(0, 1), false);
  sf_mex_addfield(*c3_info, sf_mex_duplicatearraysafe(&c3_rhs26), "rhs", "rhs",
                  26);
  sf_mex_addfield(*c3_info, sf_mex_duplicatearraysafe(&c3_lhs26), "lhs", "lhs",
                  26);
  sf_mex_addfield(*c3_info, c3_emlrt_marshallOut(
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/ops/eml_mtimes_helper.m!common_checks"),
                  "context", "context", 27);
  sf_mex_addfield(*c3_info, c3_emlrt_marshallOut(
    "coder.internal.isBuiltInNumeric"), "name", "name", 27);
  sf_mex_addfield(*c3_info, c3_emlrt_marshallOut("double"), "dominantType",
                  "dominantType", 27);
  sf_mex_addfield(*c3_info, c3_emlrt_marshallOut(
    "[ILXE]$matlabroot$/toolbox/shared/coder/coder/+coder/+internal/isBuiltInNumeric.m"),
                  "resolved", "resolved", 27);
  sf_mex_addfield(*c3_info, c3_b_emlrt_marshallOut(1395953456U), "fileTimeLo",
                  "fileTimeLo", 27);
  sf_mex_addfield(*c3_info, c3_b_emlrt_marshallOut(0U), "fileTimeHi",
                  "fileTimeHi", 27);
  sf_mex_addfield(*c3_info, c3_b_emlrt_marshallOut(0U), "mFileTimeLo",
                  "mFileTimeLo", 27);
  sf_mex_addfield(*c3_info, c3_b_emlrt_marshallOut(0U), "mFileTimeHi",
                  "mFileTimeHi", 27);
  sf_mex_assign(&c3_rhs27, sf_mex_createcellmatrix(0, 1), false);
  sf_mex_assign(&c3_lhs27, sf_mex_createcellmatrix(0, 1), false);
  sf_mex_addfield(*c3_info, sf_mex_duplicatearraysafe(&c3_rhs27), "rhs", "rhs",
                  27);
  sf_mex_addfield(*c3_info, sf_mex_duplicatearraysafe(&c3_lhs27), "lhs", "lhs",
                  27);
  sf_mex_addfield(*c3_info, c3_emlrt_marshallOut(""), "context", "context", 28);
  sf_mex_addfield(*c3_info, c3_emlrt_marshallOut("newPose"), "name", "name", 28);
  sf_mex_addfield(*c3_info, c3_emlrt_marshallOut("double"), "dominantType",
                  "dominantType", 28);
  sf_mex_addfield(*c3_info, c3_emlrt_marshallOut(
    "[E]C:/Users/sd0016/Dropbox/UAH Doktora/Journal Tracking/Matlab Code/3DOF_Experiments/Mirage/util/newPose.m"),
                  "resolved", "resolved", 28);
  sf_mex_addfield(*c3_info, c3_b_emlrt_marshallOut(1417492848U), "fileTimeLo",
                  "fileTimeLo", 28);
  sf_mex_addfield(*c3_info, c3_b_emlrt_marshallOut(0U), "fileTimeHi",
                  "fileTimeHi", 28);
  sf_mex_addfield(*c3_info, c3_b_emlrt_marshallOut(0U), "mFileTimeLo",
                  "mFileTimeLo", 28);
  sf_mex_addfield(*c3_info, c3_b_emlrt_marshallOut(0U), "mFileTimeHi",
                  "mFileTimeHi", 28);
  sf_mex_assign(&c3_rhs28, sf_mex_createcellmatrix(0, 1), false);
  sf_mex_assign(&c3_lhs28, sf_mex_createcellmatrix(0, 1), false);
  sf_mex_addfield(*c3_info, sf_mex_duplicatearraysafe(&c3_rhs28), "rhs", "rhs",
                  28);
  sf_mex_addfield(*c3_info, sf_mex_duplicatearraysafe(&c3_lhs28), "lhs", "lhs",
                  28);
  sf_mex_addfield(*c3_info, c3_emlrt_marshallOut(""), "context", "context", 29);
  sf_mex_addfield(*c3_info, c3_emlrt_marshallOut("transformationMatrix"), "name",
                  "name", 29);
  sf_mex_addfield(*c3_info, c3_emlrt_marshallOut("struct"), "dominantType",
                  "dominantType", 29);
  sf_mex_addfield(*c3_info, c3_emlrt_marshallOut(
    "[E]C:/Users/sd0016/Dropbox/UAH Doktora/Journal Tracking/Matlab Code/3DOF_Experiments/Mirage/util/transformationMatrix.m"),
                  "resolved", "resolved", 29);
  sf_mex_addfield(*c3_info, c3_b_emlrt_marshallOut(1428445740U), "fileTimeLo",
                  "fileTimeLo", 29);
  sf_mex_addfield(*c3_info, c3_b_emlrt_marshallOut(0U), "fileTimeHi",
                  "fileTimeHi", 29);
  sf_mex_addfield(*c3_info, c3_b_emlrt_marshallOut(0U), "mFileTimeLo",
                  "mFileTimeLo", 29);
  sf_mex_addfield(*c3_info, c3_b_emlrt_marshallOut(0U), "mFileTimeHi",
                  "mFileTimeHi", 29);
  sf_mex_assign(&c3_rhs29, sf_mex_createcellmatrix(0, 1), false);
  sf_mex_assign(&c3_lhs29, sf_mex_createcellmatrix(0, 1), false);
  sf_mex_addfield(*c3_info, sf_mex_duplicatearraysafe(&c3_rhs29), "rhs", "rhs",
                  29);
  sf_mex_addfield(*c3_info, sf_mex_duplicatearraysafe(&c3_lhs29), "lhs", "lhs",
                  29);
  sf_mex_addfield(*c3_info, c3_emlrt_marshallOut(
    "[E]C:/Users/sd0016/Dropbox/UAH Doktora/Journal Tracking/Matlab Code/3DOF_Experiments/Mirage/util/transformationMatrix.m"),
                  "context", "context", 30);
  sf_mex_addfield(*c3_info, c3_emlrt_marshallOut("EulerTrans"), "name", "name",
                  30);
  sf_mex_addfield(*c3_info, c3_emlrt_marshallOut("struct"), "dominantType",
                  "dominantType", 30);
  sf_mex_addfield(*c3_info, c3_emlrt_marshallOut(
    "[E]C:/Users/sd0016/Dropbox/UAH Doktora/Journal Tracking/Matlab Code/3DOF_Experiments/EulerTrans.m"),
                  "resolved", "resolved", 30);
  sf_mex_addfield(*c3_info, c3_b_emlrt_marshallOut(1346342888U), "fileTimeLo",
                  "fileTimeLo", 30);
  sf_mex_addfield(*c3_info, c3_b_emlrt_marshallOut(0U), "fileTimeHi",
                  "fileTimeHi", 30);
  sf_mex_addfield(*c3_info, c3_b_emlrt_marshallOut(0U), "mFileTimeLo",
                  "mFileTimeLo", 30);
  sf_mex_addfield(*c3_info, c3_b_emlrt_marshallOut(0U), "mFileTimeHi",
                  "mFileTimeHi", 30);
  sf_mex_assign(&c3_rhs30, sf_mex_createcellmatrix(0, 1), false);
  sf_mex_assign(&c3_lhs30, sf_mex_createcellmatrix(0, 1), false);
  sf_mex_addfield(*c3_info, sf_mex_duplicatearraysafe(&c3_rhs30), "rhs", "rhs",
                  30);
  sf_mex_addfield(*c3_info, sf_mex_duplicatearraysafe(&c3_lhs30), "lhs", "lhs",
                  30);
  sf_mex_addfield(*c3_info, c3_emlrt_marshallOut(
    "[E]C:/Users/sd0016/Dropbox/UAH Doktora/Journal Tracking/Matlab Code/3DOF_Experiments/EulerTrans.m"),
                  "context", "context", 31);
  sf_mex_addfield(*c3_info, c3_emlrt_marshallOut("cos"), "name", "name", 31);
  sf_mex_addfield(*c3_info, c3_emlrt_marshallOut("double"), "dominantType",
                  "dominantType", 31);
  sf_mex_addfield(*c3_info, c3_emlrt_marshallOut(
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/elfun/cos.m"), "resolved",
                  "resolved", 31);
  sf_mex_addfield(*c3_info, c3_b_emlrt_marshallOut(1395350096U), "fileTimeLo",
                  "fileTimeLo", 31);
  sf_mex_addfield(*c3_info, c3_b_emlrt_marshallOut(0U), "fileTimeHi",
                  "fileTimeHi", 31);
  sf_mex_addfield(*c3_info, c3_b_emlrt_marshallOut(0U), "mFileTimeLo",
                  "mFileTimeLo", 31);
  sf_mex_addfield(*c3_info, c3_b_emlrt_marshallOut(0U), "mFileTimeHi",
                  "mFileTimeHi", 31);
  sf_mex_assign(&c3_rhs31, sf_mex_createcellmatrix(0, 1), false);
  sf_mex_assign(&c3_lhs31, sf_mex_createcellmatrix(0, 1), false);
  sf_mex_addfield(*c3_info, sf_mex_duplicatearraysafe(&c3_rhs31), "rhs", "rhs",
                  31);
  sf_mex_addfield(*c3_info, sf_mex_duplicatearraysafe(&c3_lhs31), "lhs", "lhs",
                  31);
  sf_mex_addfield(*c3_info, c3_emlrt_marshallOut(
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/elfun/cos.m"), "context",
                  "context", 32);
  sf_mex_addfield(*c3_info, c3_emlrt_marshallOut("eml_scalar_cos"), "name",
                  "name", 32);
  sf_mex_addfield(*c3_info, c3_emlrt_marshallOut("double"), "dominantType",
                  "dominantType", 32);
  sf_mex_addfield(*c3_info, c3_emlrt_marshallOut(
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/elfun/eml_scalar_cos.m"),
                  "resolved", "resolved", 32);
  sf_mex_addfield(*c3_info, c3_b_emlrt_marshallOut(1286843922U), "fileTimeLo",
                  "fileTimeLo", 32);
  sf_mex_addfield(*c3_info, c3_b_emlrt_marshallOut(0U), "fileTimeHi",
                  "fileTimeHi", 32);
  sf_mex_addfield(*c3_info, c3_b_emlrt_marshallOut(0U), "mFileTimeLo",
                  "mFileTimeLo", 32);
  sf_mex_addfield(*c3_info, c3_b_emlrt_marshallOut(0U), "mFileTimeHi",
                  "mFileTimeHi", 32);
  sf_mex_assign(&c3_rhs32, sf_mex_createcellmatrix(0, 1), false);
  sf_mex_assign(&c3_lhs32, sf_mex_createcellmatrix(0, 1), false);
  sf_mex_addfield(*c3_info, sf_mex_duplicatearraysafe(&c3_rhs32), "rhs", "rhs",
                  32);
  sf_mex_addfield(*c3_info, sf_mex_duplicatearraysafe(&c3_lhs32), "lhs", "lhs",
                  32);
  sf_mex_addfield(*c3_info, c3_emlrt_marshallOut(
    "[E]C:/Users/sd0016/Dropbox/UAH Doktora/Journal Tracking/Matlab Code/3DOF_Experiments/EulerTrans.m"),
                  "context", "context", 33);
  sf_mex_addfield(*c3_info, c3_emlrt_marshallOut("sin"), "name", "name", 33);
  sf_mex_addfield(*c3_info, c3_emlrt_marshallOut("double"), "dominantType",
                  "dominantType", 33);
  sf_mex_addfield(*c3_info, c3_emlrt_marshallOut(
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/elfun/sin.m"), "resolved",
                  "resolved", 33);
  sf_mex_addfield(*c3_info, c3_b_emlrt_marshallOut(1395350104U), "fileTimeLo",
                  "fileTimeLo", 33);
  sf_mex_addfield(*c3_info, c3_b_emlrt_marshallOut(0U), "fileTimeHi",
                  "fileTimeHi", 33);
  sf_mex_addfield(*c3_info, c3_b_emlrt_marshallOut(0U), "mFileTimeLo",
                  "mFileTimeLo", 33);
  sf_mex_addfield(*c3_info, c3_b_emlrt_marshallOut(0U), "mFileTimeHi",
                  "mFileTimeHi", 33);
  sf_mex_assign(&c3_rhs33, sf_mex_createcellmatrix(0, 1), false);
  sf_mex_assign(&c3_lhs33, sf_mex_createcellmatrix(0, 1), false);
  sf_mex_addfield(*c3_info, sf_mex_duplicatearraysafe(&c3_rhs33), "rhs", "rhs",
                  33);
  sf_mex_addfield(*c3_info, sf_mex_duplicatearraysafe(&c3_lhs33), "lhs", "lhs",
                  33);
  sf_mex_addfield(*c3_info, c3_emlrt_marshallOut(
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/elfun/sin.m"), "context",
                  "context", 34);
  sf_mex_addfield(*c3_info, c3_emlrt_marshallOut("eml_scalar_sin"), "name",
                  "name", 34);
  sf_mex_addfield(*c3_info, c3_emlrt_marshallOut("double"), "dominantType",
                  "dominantType", 34);
  sf_mex_addfield(*c3_info, c3_emlrt_marshallOut(
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/elfun/eml_scalar_sin.m"),
                  "resolved", "resolved", 34);
  sf_mex_addfield(*c3_info, c3_b_emlrt_marshallOut(1286843936U), "fileTimeLo",
                  "fileTimeLo", 34);
  sf_mex_addfield(*c3_info, c3_b_emlrt_marshallOut(0U), "fileTimeHi",
                  "fileTimeHi", 34);
  sf_mex_addfield(*c3_info, c3_b_emlrt_marshallOut(0U), "mFileTimeLo",
                  "mFileTimeLo", 34);
  sf_mex_addfield(*c3_info, c3_b_emlrt_marshallOut(0U), "mFileTimeHi",
                  "mFileTimeHi", 34);
  sf_mex_assign(&c3_rhs34, sf_mex_createcellmatrix(0, 1), false);
  sf_mex_assign(&c3_lhs34, sf_mex_createcellmatrix(0, 1), false);
  sf_mex_addfield(*c3_info, sf_mex_duplicatearraysafe(&c3_rhs34), "rhs", "rhs",
                  34);
  sf_mex_addfield(*c3_info, sf_mex_duplicatearraysafe(&c3_lhs34), "lhs", "lhs",
                  34);
  sf_mex_addfield(*c3_info, c3_emlrt_marshallOut(
    "[E]C:/Users/sd0016/Dropbox/UAH Doktora/Journal Tracking/Matlab Code/3DOF_Experiments/EulerTrans.m"),
                  "context", "context", 35);
  sf_mex_addfield(*c3_info, c3_emlrt_marshallOut("eml_mtimes_helper"), "name",
                  "name", 35);
  sf_mex_addfield(*c3_info, c3_emlrt_marshallOut(""), "dominantType",
                  "dominantType", 35);
  sf_mex_addfield(*c3_info, c3_emlrt_marshallOut(
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/ops/eml_mtimes_helper.m"),
                  "resolved", "resolved", 35);
  sf_mex_addfield(*c3_info, c3_b_emlrt_marshallOut(1383902494U), "fileTimeLo",
                  "fileTimeLo", 35);
  sf_mex_addfield(*c3_info, c3_b_emlrt_marshallOut(0U), "fileTimeHi",
                  "fileTimeHi", 35);
  sf_mex_addfield(*c3_info, c3_b_emlrt_marshallOut(0U), "mFileTimeLo",
                  "mFileTimeLo", 35);
  sf_mex_addfield(*c3_info, c3_b_emlrt_marshallOut(0U), "mFileTimeHi",
                  "mFileTimeHi", 35);
  sf_mex_assign(&c3_rhs35, sf_mex_createcellmatrix(0, 1), false);
  sf_mex_assign(&c3_lhs35, sf_mex_createcellmatrix(0, 1), false);
  sf_mex_addfield(*c3_info, sf_mex_duplicatearraysafe(&c3_rhs35), "rhs", "rhs",
                  35);
  sf_mex_addfield(*c3_info, sf_mex_duplicatearraysafe(&c3_lhs35), "lhs", "lhs",
                  35);
  sf_mex_addfield(*c3_info, c3_emlrt_marshallOut(
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/ops/eml_mtimes_helper.m"),
                  "context", "context", 36);
  sf_mex_addfield(*c3_info, c3_emlrt_marshallOut("eml_index_class"), "name",
                  "name", 36);
  sf_mex_addfield(*c3_info, c3_emlrt_marshallOut(""), "dominantType",
                  "dominantType", 36);
  sf_mex_addfield(*c3_info, c3_emlrt_marshallOut(
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/eml/eml_index_class.m"),
                  "resolved", "resolved", 36);
  sf_mex_addfield(*c3_info, c3_b_emlrt_marshallOut(1323195778U), "fileTimeLo",
                  "fileTimeLo", 36);
  sf_mex_addfield(*c3_info, c3_b_emlrt_marshallOut(0U), "fileTimeHi",
                  "fileTimeHi", 36);
  sf_mex_addfield(*c3_info, c3_b_emlrt_marshallOut(0U), "mFileTimeLo",
                  "mFileTimeLo", 36);
  sf_mex_addfield(*c3_info, c3_b_emlrt_marshallOut(0U), "mFileTimeHi",
                  "mFileTimeHi", 36);
  sf_mex_assign(&c3_rhs36, sf_mex_createcellmatrix(0, 1), false);
  sf_mex_assign(&c3_lhs36, sf_mex_createcellmatrix(0, 1), false);
  sf_mex_addfield(*c3_info, sf_mex_duplicatearraysafe(&c3_rhs36), "rhs", "rhs",
                  36);
  sf_mex_addfield(*c3_info, sf_mex_duplicatearraysafe(&c3_lhs36), "lhs", "lhs",
                  36);
  sf_mex_addfield(*c3_info, c3_emlrt_marshallOut(
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/ops/eml_mtimes_helper.m"),
                  "context", "context", 37);
  sf_mex_addfield(*c3_info, c3_emlrt_marshallOut("eml_scalar_eg"), "name",
                  "name", 37);
  sf_mex_addfield(*c3_info, c3_emlrt_marshallOut("double"), "dominantType",
                  "dominantType", 37);
  sf_mex_addfield(*c3_info, c3_emlrt_marshallOut(
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/eml/eml_scalar_eg.m"), "resolved",
                  "resolved", 37);
  sf_mex_addfield(*c3_info, c3_b_emlrt_marshallOut(1376005888U), "fileTimeLo",
                  "fileTimeLo", 37);
  sf_mex_addfield(*c3_info, c3_b_emlrt_marshallOut(0U), "fileTimeHi",
                  "fileTimeHi", 37);
  sf_mex_addfield(*c3_info, c3_b_emlrt_marshallOut(0U), "mFileTimeLo",
                  "mFileTimeLo", 37);
  sf_mex_addfield(*c3_info, c3_b_emlrt_marshallOut(0U), "mFileTimeHi",
                  "mFileTimeHi", 37);
  sf_mex_assign(&c3_rhs37, sf_mex_createcellmatrix(0, 1), false);
  sf_mex_assign(&c3_lhs37, sf_mex_createcellmatrix(0, 1), false);
  sf_mex_addfield(*c3_info, sf_mex_duplicatearraysafe(&c3_rhs37), "rhs", "rhs",
                  37);
  sf_mex_addfield(*c3_info, sf_mex_duplicatearraysafe(&c3_lhs37), "lhs", "lhs",
                  37);
  sf_mex_addfield(*c3_info, c3_emlrt_marshallOut(
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/eml/eml_scalar_eg.m"), "context",
                  "context", 38);
  sf_mex_addfield(*c3_info, c3_emlrt_marshallOut("coder.internal.scalarEg"),
                  "name", "name", 38);
  sf_mex_addfield(*c3_info, c3_emlrt_marshallOut("double"), "dominantType",
                  "dominantType", 38);
  sf_mex_addfield(*c3_info, c3_emlrt_marshallOut(
    "[ILXE]$matlabroot$/toolbox/coder/coder/+coder/+internal/scalarEg.p"),
                  "resolved", "resolved", 38);
  sf_mex_addfield(*c3_info, c3_b_emlrt_marshallOut(1410832970U), "fileTimeLo",
                  "fileTimeLo", 38);
  sf_mex_addfield(*c3_info, c3_b_emlrt_marshallOut(0U), "fileTimeHi",
                  "fileTimeHi", 38);
  sf_mex_addfield(*c3_info, c3_b_emlrt_marshallOut(0U), "mFileTimeLo",
                  "mFileTimeLo", 38);
  sf_mex_addfield(*c3_info, c3_b_emlrt_marshallOut(0U), "mFileTimeHi",
                  "mFileTimeHi", 38);
  sf_mex_assign(&c3_rhs38, sf_mex_createcellmatrix(0, 1), false);
  sf_mex_assign(&c3_lhs38, sf_mex_createcellmatrix(0, 1), false);
  sf_mex_addfield(*c3_info, sf_mex_duplicatearraysafe(&c3_rhs38), "rhs", "rhs",
                  38);
  sf_mex_addfield(*c3_info, sf_mex_duplicatearraysafe(&c3_lhs38), "lhs", "lhs",
                  38);
  sf_mex_addfield(*c3_info, c3_emlrt_marshallOut(
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/ops/eml_mtimes_helper.m"),
                  "context", "context", 39);
  sf_mex_addfield(*c3_info, c3_emlrt_marshallOut("eml_xgemm"), "name", "name",
                  39);
  sf_mex_addfield(*c3_info, c3_emlrt_marshallOut("char"), "dominantType",
                  "dominantType", 39);
  sf_mex_addfield(*c3_info, c3_emlrt_marshallOut(
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/eml/blas/eml_xgemm.m"),
                  "resolved", "resolved", 39);
  sf_mex_addfield(*c3_info, c3_b_emlrt_marshallOut(1376005890U), "fileTimeLo",
                  "fileTimeLo", 39);
  sf_mex_addfield(*c3_info, c3_b_emlrt_marshallOut(0U), "fileTimeHi",
                  "fileTimeHi", 39);
  sf_mex_addfield(*c3_info, c3_b_emlrt_marshallOut(0U), "mFileTimeLo",
                  "mFileTimeLo", 39);
  sf_mex_addfield(*c3_info, c3_b_emlrt_marshallOut(0U), "mFileTimeHi",
                  "mFileTimeHi", 39);
  sf_mex_assign(&c3_rhs39, sf_mex_createcellmatrix(0, 1), false);
  sf_mex_assign(&c3_lhs39, sf_mex_createcellmatrix(0, 1), false);
  sf_mex_addfield(*c3_info, sf_mex_duplicatearraysafe(&c3_rhs39), "rhs", "rhs",
                  39);
  sf_mex_addfield(*c3_info, sf_mex_duplicatearraysafe(&c3_lhs39), "lhs", "lhs",
                  39);
  sf_mex_addfield(*c3_info, c3_emlrt_marshallOut(
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/eml/blas/eml_xgemm.m"), "context",
                  "context", 40);
  sf_mex_addfield(*c3_info, c3_emlrt_marshallOut("coder.internal.blas.inline"),
                  "name", "name", 40);
  sf_mex_addfield(*c3_info, c3_emlrt_marshallOut(""), "dominantType",
                  "dominantType", 40);
  sf_mex_addfield(*c3_info, c3_emlrt_marshallOut(
    "[ILXE]$matlabroot$/toolbox/coder/coder/+coder/+internal/+blas/inline.p"),
                  "resolved", "resolved", 40);
  sf_mex_addfield(*c3_info, c3_b_emlrt_marshallOut(1410832972U), "fileTimeLo",
                  "fileTimeLo", 40);
  sf_mex_addfield(*c3_info, c3_b_emlrt_marshallOut(0U), "fileTimeHi",
                  "fileTimeHi", 40);
  sf_mex_addfield(*c3_info, c3_b_emlrt_marshallOut(0U), "mFileTimeLo",
                  "mFileTimeLo", 40);
  sf_mex_addfield(*c3_info, c3_b_emlrt_marshallOut(0U), "mFileTimeHi",
                  "mFileTimeHi", 40);
  sf_mex_assign(&c3_rhs40, sf_mex_createcellmatrix(0, 1), false);
  sf_mex_assign(&c3_lhs40, sf_mex_createcellmatrix(0, 1), false);
  sf_mex_addfield(*c3_info, sf_mex_duplicatearraysafe(&c3_rhs40), "rhs", "rhs",
                  40);
  sf_mex_addfield(*c3_info, sf_mex_duplicatearraysafe(&c3_lhs40), "lhs", "lhs",
                  40);
  sf_mex_addfield(*c3_info, c3_emlrt_marshallOut(
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/eml/blas/eml_xgemm.m"), "context",
                  "context", 41);
  sf_mex_addfield(*c3_info, c3_emlrt_marshallOut("coder.internal.blas.xgemm"),
                  "name", "name", 41);
  sf_mex_addfield(*c3_info, c3_emlrt_marshallOut("char"), "dominantType",
                  "dominantType", 41);
  sf_mex_addfield(*c3_info, c3_emlrt_marshallOut(
    "[ILXE]$matlabroot$/toolbox/coder/coder/+coder/+internal/+blas/xgemm.p"),
                  "resolved", "resolved", 41);
  sf_mex_addfield(*c3_info, c3_b_emlrt_marshallOut(1410832970U), "fileTimeLo",
                  "fileTimeLo", 41);
  sf_mex_addfield(*c3_info, c3_b_emlrt_marshallOut(0U), "fileTimeHi",
                  "fileTimeHi", 41);
  sf_mex_addfield(*c3_info, c3_b_emlrt_marshallOut(0U), "mFileTimeLo",
                  "mFileTimeLo", 41);
  sf_mex_addfield(*c3_info, c3_b_emlrt_marshallOut(0U), "mFileTimeHi",
                  "mFileTimeHi", 41);
  sf_mex_assign(&c3_rhs41, sf_mex_createcellmatrix(0, 1), false);
  sf_mex_assign(&c3_lhs41, sf_mex_createcellmatrix(0, 1), false);
  sf_mex_addfield(*c3_info, sf_mex_duplicatearraysafe(&c3_rhs41), "rhs", "rhs",
                  41);
  sf_mex_addfield(*c3_info, sf_mex_duplicatearraysafe(&c3_lhs41), "lhs", "lhs",
                  41);
  sf_mex_addfield(*c3_info, c3_emlrt_marshallOut(
    "[ILXE]$matlabroot$/toolbox/coder/coder/+coder/+internal/+blas/xgemm.p"),
                  "context", "context", 42);
  sf_mex_addfield(*c3_info, c3_emlrt_marshallOut(
    "coder.internal.blas.use_refblas"), "name", "name", 42);
  sf_mex_addfield(*c3_info, c3_emlrt_marshallOut(""), "dominantType",
                  "dominantType", 42);
  sf_mex_addfield(*c3_info, c3_emlrt_marshallOut(
    "[ILXE]$matlabroot$/toolbox/coder/coder/+coder/+internal/+blas/use_refblas.p"),
                  "resolved", "resolved", 42);
  sf_mex_addfield(*c3_info, c3_b_emlrt_marshallOut(1410832970U), "fileTimeLo",
                  "fileTimeLo", 42);
  sf_mex_addfield(*c3_info, c3_b_emlrt_marshallOut(0U), "fileTimeHi",
                  "fileTimeHi", 42);
  sf_mex_addfield(*c3_info, c3_b_emlrt_marshallOut(0U), "mFileTimeLo",
                  "mFileTimeLo", 42);
  sf_mex_addfield(*c3_info, c3_b_emlrt_marshallOut(0U), "mFileTimeHi",
                  "mFileTimeHi", 42);
  sf_mex_assign(&c3_rhs42, sf_mex_createcellmatrix(0, 1), false);
  sf_mex_assign(&c3_lhs42, sf_mex_createcellmatrix(0, 1), false);
  sf_mex_addfield(*c3_info, sf_mex_duplicatearraysafe(&c3_rhs42), "rhs", "rhs",
                  42);
  sf_mex_addfield(*c3_info, sf_mex_duplicatearraysafe(&c3_lhs42), "lhs", "lhs",
                  42);
  sf_mex_addfield(*c3_info, c3_emlrt_marshallOut(
    "[ILXE]$matlabroot$/toolbox/coder/coder/+coder/+internal/+blas/xgemm.p!below_threshold"),
                  "context", "context", 43);
  sf_mex_addfield(*c3_info, c3_emlrt_marshallOut("coder.internal.blas.threshold"),
                  "name", "name", 43);
  sf_mex_addfield(*c3_info, c3_emlrt_marshallOut("char"), "dominantType",
                  "dominantType", 43);
  sf_mex_addfield(*c3_info, c3_emlrt_marshallOut(
    "[ILXE]$matlabroot$/toolbox/coder/coder/+coder/+internal/+blas/threshold.p"),
                  "resolved", "resolved", 43);
  sf_mex_addfield(*c3_info, c3_b_emlrt_marshallOut(1410832972U), "fileTimeLo",
                  "fileTimeLo", 43);
  sf_mex_addfield(*c3_info, c3_b_emlrt_marshallOut(0U), "fileTimeHi",
                  "fileTimeHi", 43);
  sf_mex_addfield(*c3_info, c3_b_emlrt_marshallOut(0U), "mFileTimeLo",
                  "mFileTimeLo", 43);
  sf_mex_addfield(*c3_info, c3_b_emlrt_marshallOut(0U), "mFileTimeHi",
                  "mFileTimeHi", 43);
  sf_mex_assign(&c3_rhs43, sf_mex_createcellmatrix(0, 1), false);
  sf_mex_assign(&c3_lhs43, sf_mex_createcellmatrix(0, 1), false);
  sf_mex_addfield(*c3_info, sf_mex_duplicatearraysafe(&c3_rhs43), "rhs", "rhs",
                  43);
  sf_mex_addfield(*c3_info, sf_mex_duplicatearraysafe(&c3_lhs43), "lhs", "lhs",
                  43);
  sf_mex_addfield(*c3_info, c3_emlrt_marshallOut(
    "[ILXE]$matlabroot$/toolbox/coder/coder/+coder/+internal/+blas/threshold.p"),
                  "context", "context", 44);
  sf_mex_addfield(*c3_info, c3_emlrt_marshallOut("eml_switch_helper"), "name",
                  "name", 44);
  sf_mex_addfield(*c3_info, c3_emlrt_marshallOut(""), "dominantType",
                  "dominantType", 44);
  sf_mex_addfield(*c3_info, c3_emlrt_marshallOut(
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/eml/eml_switch_helper.m"),
                  "resolved", "resolved", 44);
  sf_mex_addfield(*c3_info, c3_b_emlrt_marshallOut(1393356058U), "fileTimeLo",
                  "fileTimeLo", 44);
  sf_mex_addfield(*c3_info, c3_b_emlrt_marshallOut(0U), "fileTimeHi",
                  "fileTimeHi", 44);
  sf_mex_addfield(*c3_info, c3_b_emlrt_marshallOut(0U), "mFileTimeLo",
                  "mFileTimeLo", 44);
  sf_mex_addfield(*c3_info, c3_b_emlrt_marshallOut(0U), "mFileTimeHi",
                  "mFileTimeHi", 44);
  sf_mex_assign(&c3_rhs44, sf_mex_createcellmatrix(0, 1), false);
  sf_mex_assign(&c3_lhs44, sf_mex_createcellmatrix(0, 1), false);
  sf_mex_addfield(*c3_info, sf_mex_duplicatearraysafe(&c3_rhs44), "rhs", "rhs",
                  44);
  sf_mex_addfield(*c3_info, sf_mex_duplicatearraysafe(&c3_lhs44), "lhs", "lhs",
                  44);
  sf_mex_addfield(*c3_info, c3_emlrt_marshallOut(
    "[ILXE]$matlabroot$/toolbox/coder/coder/+coder/+internal/+blas/xgemm.p"),
                  "context", "context", 45);
  sf_mex_addfield(*c3_info, c3_emlrt_marshallOut("coder.internal.scalarEg"),
                  "name", "name", 45);
  sf_mex_addfield(*c3_info, c3_emlrt_marshallOut("double"), "dominantType",
                  "dominantType", 45);
  sf_mex_addfield(*c3_info, c3_emlrt_marshallOut(
    "[ILXE]$matlabroot$/toolbox/coder/coder/+coder/+internal/scalarEg.p"),
                  "resolved", "resolved", 45);
  sf_mex_addfield(*c3_info, c3_b_emlrt_marshallOut(1410832970U), "fileTimeLo",
                  "fileTimeLo", 45);
  sf_mex_addfield(*c3_info, c3_b_emlrt_marshallOut(0U), "fileTimeHi",
                  "fileTimeHi", 45);
  sf_mex_addfield(*c3_info, c3_b_emlrt_marshallOut(0U), "mFileTimeLo",
                  "mFileTimeLo", 45);
  sf_mex_addfield(*c3_info, c3_b_emlrt_marshallOut(0U), "mFileTimeHi",
                  "mFileTimeHi", 45);
  sf_mex_assign(&c3_rhs45, sf_mex_createcellmatrix(0, 1), false);
  sf_mex_assign(&c3_lhs45, sf_mex_createcellmatrix(0, 1), false);
  sf_mex_addfield(*c3_info, sf_mex_duplicatearraysafe(&c3_rhs45), "rhs", "rhs",
                  45);
  sf_mex_addfield(*c3_info, sf_mex_duplicatearraysafe(&c3_lhs45), "lhs", "lhs",
                  45);
  sf_mex_addfield(*c3_info, c3_emlrt_marshallOut(
    "[ILXE]$matlabroot$/toolbox/coder/coder/+coder/+internal/+blas/xgemm.p"),
                  "context", "context", 46);
  sf_mex_addfield(*c3_info, c3_emlrt_marshallOut("coder.internal.refblas.xgemm"),
                  "name", "name", 46);
  sf_mex_addfield(*c3_info, c3_emlrt_marshallOut("char"), "dominantType",
                  "dominantType", 46);
  sf_mex_addfield(*c3_info, c3_emlrt_marshallOut(
    "[ILXE]$matlabroot$/toolbox/coder/coder/+coder/+internal/+refblas/xgemm.p"),
                  "resolved", "resolved", 46);
  sf_mex_addfield(*c3_info, c3_b_emlrt_marshallOut(1410832972U), "fileTimeLo",
                  "fileTimeLo", 46);
  sf_mex_addfield(*c3_info, c3_b_emlrt_marshallOut(0U), "fileTimeHi",
                  "fileTimeHi", 46);
  sf_mex_addfield(*c3_info, c3_b_emlrt_marshallOut(0U), "mFileTimeLo",
                  "mFileTimeLo", 46);
  sf_mex_addfield(*c3_info, c3_b_emlrt_marshallOut(0U), "mFileTimeHi",
                  "mFileTimeHi", 46);
  sf_mex_assign(&c3_rhs46, sf_mex_createcellmatrix(0, 1), false);
  sf_mex_assign(&c3_lhs46, sf_mex_createcellmatrix(0, 1), false);
  sf_mex_addfield(*c3_info, sf_mex_duplicatearraysafe(&c3_rhs46), "rhs", "rhs",
                  46);
  sf_mex_addfield(*c3_info, sf_mex_duplicatearraysafe(&c3_lhs46), "lhs", "lhs",
                  46);
  sf_mex_addfield(*c3_info, c3_emlrt_marshallOut(""), "context", "context", 47);
  sf_mex_addfield(*c3_info, c3_emlrt_marshallOut("inv"), "name", "name", 47);
  sf_mex_addfield(*c3_info, c3_emlrt_marshallOut("double"), "dominantType",
                  "dominantType", 47);
  sf_mex_addfield(*c3_info, c3_emlrt_marshallOut(
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/matfun/inv.m"), "resolved",
                  "resolved", 47);
  sf_mex_addfield(*c3_info, c3_b_emlrt_marshallOut(1305343200U), "fileTimeLo",
                  "fileTimeLo", 47);
  sf_mex_addfield(*c3_info, c3_b_emlrt_marshallOut(0U), "fileTimeHi",
                  "fileTimeHi", 47);
  sf_mex_addfield(*c3_info, c3_b_emlrt_marshallOut(0U), "mFileTimeLo",
                  "mFileTimeLo", 47);
  sf_mex_addfield(*c3_info, c3_b_emlrt_marshallOut(0U), "mFileTimeHi",
                  "mFileTimeHi", 47);
  sf_mex_assign(&c3_rhs47, sf_mex_createcellmatrix(0, 1), false);
  sf_mex_assign(&c3_lhs47, sf_mex_createcellmatrix(0, 1), false);
  sf_mex_addfield(*c3_info, sf_mex_duplicatearraysafe(&c3_rhs47), "rhs", "rhs",
                  47);
  sf_mex_addfield(*c3_info, sf_mex_duplicatearraysafe(&c3_lhs47), "lhs", "lhs",
                  47);
  sf_mex_addfield(*c3_info, c3_emlrt_marshallOut(
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/matfun/inv.m!invNxN"), "context",
                  "context", 48);
  sf_mex_addfield(*c3_info, c3_emlrt_marshallOut("eml_index_class"), "name",
                  "name", 48);
  sf_mex_addfield(*c3_info, c3_emlrt_marshallOut(""), "dominantType",
                  "dominantType", 48);
  sf_mex_addfield(*c3_info, c3_emlrt_marshallOut(
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/eml/eml_index_class.m"),
                  "resolved", "resolved", 48);
  sf_mex_addfield(*c3_info, c3_b_emlrt_marshallOut(1323195778U), "fileTimeLo",
                  "fileTimeLo", 48);
  sf_mex_addfield(*c3_info, c3_b_emlrt_marshallOut(0U), "fileTimeHi",
                  "fileTimeHi", 48);
  sf_mex_addfield(*c3_info, c3_b_emlrt_marshallOut(0U), "mFileTimeLo",
                  "mFileTimeLo", 48);
  sf_mex_addfield(*c3_info, c3_b_emlrt_marshallOut(0U), "mFileTimeHi",
                  "mFileTimeHi", 48);
  sf_mex_assign(&c3_rhs48, sf_mex_createcellmatrix(0, 1), false);
  sf_mex_assign(&c3_lhs48, sf_mex_createcellmatrix(0, 1), false);
  sf_mex_addfield(*c3_info, sf_mex_duplicatearraysafe(&c3_rhs48), "rhs", "rhs",
                  48);
  sf_mex_addfield(*c3_info, sf_mex_duplicatearraysafe(&c3_lhs48), "lhs", "lhs",
                  48);
  sf_mex_addfield(*c3_info, c3_emlrt_marshallOut(
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/matfun/inv.m!invNxN"), "context",
                  "context", 49);
  sf_mex_addfield(*c3_info, c3_emlrt_marshallOut("eml_xgetrf"), "name", "name",
                  49);
  sf_mex_addfield(*c3_info, c3_emlrt_marshallOut("double"), "dominantType",
                  "dominantType", 49);
  sf_mex_addfield(*c3_info, c3_emlrt_marshallOut(
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/eml/lapack/eml_xgetrf.m"),
                  "resolved", "resolved", 49);
  sf_mex_addfield(*c3_info, c3_b_emlrt_marshallOut(1286844006U), "fileTimeLo",
                  "fileTimeLo", 49);
  sf_mex_addfield(*c3_info, c3_b_emlrt_marshallOut(0U), "fileTimeHi",
                  "fileTimeHi", 49);
  sf_mex_addfield(*c3_info, c3_b_emlrt_marshallOut(0U), "mFileTimeLo",
                  "mFileTimeLo", 49);
  sf_mex_addfield(*c3_info, c3_b_emlrt_marshallOut(0U), "mFileTimeHi",
                  "mFileTimeHi", 49);
  sf_mex_assign(&c3_rhs49, sf_mex_createcellmatrix(0, 1), false);
  sf_mex_assign(&c3_lhs49, sf_mex_createcellmatrix(0, 1), false);
  sf_mex_addfield(*c3_info, sf_mex_duplicatearraysafe(&c3_rhs49), "rhs", "rhs",
                  49);
  sf_mex_addfield(*c3_info, sf_mex_duplicatearraysafe(&c3_lhs49), "lhs", "lhs",
                  49);
  sf_mex_addfield(*c3_info, c3_emlrt_marshallOut(
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/eml/lapack/eml_xgetrf.m"),
                  "context", "context", 50);
  sf_mex_addfield(*c3_info, c3_emlrt_marshallOut("eml_lapack_xgetrf"), "name",
                  "name", 50);
  sf_mex_addfield(*c3_info, c3_emlrt_marshallOut("double"), "dominantType",
                  "dominantType", 50);
  sf_mex_addfield(*c3_info, c3_emlrt_marshallOut(
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/eml/lapack/internal/eml_lapack_xgetrf.m"),
                  "resolved", "resolved", 50);
  sf_mex_addfield(*c3_info, c3_b_emlrt_marshallOut(1286844010U), "fileTimeLo",
                  "fileTimeLo", 50);
  sf_mex_addfield(*c3_info, c3_b_emlrt_marshallOut(0U), "fileTimeHi",
                  "fileTimeHi", 50);
  sf_mex_addfield(*c3_info, c3_b_emlrt_marshallOut(0U), "mFileTimeLo",
                  "mFileTimeLo", 50);
  sf_mex_addfield(*c3_info, c3_b_emlrt_marshallOut(0U), "mFileTimeHi",
                  "mFileTimeHi", 50);
  sf_mex_assign(&c3_rhs50, sf_mex_createcellmatrix(0, 1), false);
  sf_mex_assign(&c3_lhs50, sf_mex_createcellmatrix(0, 1), false);
  sf_mex_addfield(*c3_info, sf_mex_duplicatearraysafe(&c3_rhs50), "rhs", "rhs",
                  50);
  sf_mex_addfield(*c3_info, sf_mex_duplicatearraysafe(&c3_lhs50), "lhs", "lhs",
                  50);
  sf_mex_addfield(*c3_info, c3_emlrt_marshallOut(
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/eml/lapack/internal/eml_lapack_xgetrf.m"),
                  "context", "context", 51);
  sf_mex_addfield(*c3_info, c3_emlrt_marshallOut("eml_matlab_zgetrf"), "name",
                  "name", 51);
  sf_mex_addfield(*c3_info, c3_emlrt_marshallOut("double"), "dominantType",
                  "dominantType", 51);
  sf_mex_addfield(*c3_info, c3_emlrt_marshallOut(
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/eml/lapack/matlab/eml_matlab_zgetrf.m"),
                  "resolved", "resolved", 51);
  sf_mex_addfield(*c3_info, c3_b_emlrt_marshallOut(1302714194U), "fileTimeLo",
                  "fileTimeLo", 51);
  sf_mex_addfield(*c3_info, c3_b_emlrt_marshallOut(0U), "fileTimeHi",
                  "fileTimeHi", 51);
  sf_mex_addfield(*c3_info, c3_b_emlrt_marshallOut(0U), "mFileTimeLo",
                  "mFileTimeLo", 51);
  sf_mex_addfield(*c3_info, c3_b_emlrt_marshallOut(0U), "mFileTimeHi",
                  "mFileTimeHi", 51);
  sf_mex_assign(&c3_rhs51, sf_mex_createcellmatrix(0, 1), false);
  sf_mex_assign(&c3_lhs51, sf_mex_createcellmatrix(0, 1), false);
  sf_mex_addfield(*c3_info, sf_mex_duplicatearraysafe(&c3_rhs51), "rhs", "rhs",
                  51);
  sf_mex_addfield(*c3_info, sf_mex_duplicatearraysafe(&c3_lhs51), "lhs", "lhs",
                  51);
  sf_mex_addfield(*c3_info, c3_emlrt_marshallOut(
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/eml/lapack/matlab/eml_matlab_zgetrf.m"),
                  "context", "context", 52);
  sf_mex_addfield(*c3_info, c3_emlrt_marshallOut("realmin"), "name", "name", 52);
  sf_mex_addfield(*c3_info, c3_emlrt_marshallOut("char"), "dominantType",
                  "dominantType", 52);
  sf_mex_addfield(*c3_info, c3_emlrt_marshallOut(
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/elmat/realmin.m"), "resolved",
                  "resolved", 52);
  sf_mex_addfield(*c3_info, c3_b_emlrt_marshallOut(1307676442U), "fileTimeLo",
                  "fileTimeLo", 52);
  sf_mex_addfield(*c3_info, c3_b_emlrt_marshallOut(0U), "fileTimeHi",
                  "fileTimeHi", 52);
  sf_mex_addfield(*c3_info, c3_b_emlrt_marshallOut(0U), "mFileTimeLo",
                  "mFileTimeLo", 52);
  sf_mex_addfield(*c3_info, c3_b_emlrt_marshallOut(0U), "mFileTimeHi",
                  "mFileTimeHi", 52);
  sf_mex_assign(&c3_rhs52, sf_mex_createcellmatrix(0, 1), false);
  sf_mex_assign(&c3_lhs52, sf_mex_createcellmatrix(0, 1), false);
  sf_mex_addfield(*c3_info, sf_mex_duplicatearraysafe(&c3_rhs52), "rhs", "rhs",
                  52);
  sf_mex_addfield(*c3_info, sf_mex_duplicatearraysafe(&c3_lhs52), "lhs", "lhs",
                  52);
  sf_mex_addfield(*c3_info, c3_emlrt_marshallOut(
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/elmat/realmin.m"), "context",
                  "context", 53);
  sf_mex_addfield(*c3_info, c3_emlrt_marshallOut("eml_realmin"), "name", "name",
                  53);
  sf_mex_addfield(*c3_info, c3_emlrt_marshallOut("char"), "dominantType",
                  "dominantType", 53);
  sf_mex_addfield(*c3_info, c3_emlrt_marshallOut(
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/eml/eml_realmin.m"), "resolved",
                  "resolved", 53);
  sf_mex_addfield(*c3_info, c3_b_emlrt_marshallOut(1307676444U), "fileTimeLo",
                  "fileTimeLo", 53);
  sf_mex_addfield(*c3_info, c3_b_emlrt_marshallOut(0U), "fileTimeHi",
                  "fileTimeHi", 53);
  sf_mex_addfield(*c3_info, c3_b_emlrt_marshallOut(0U), "mFileTimeLo",
                  "mFileTimeLo", 53);
  sf_mex_addfield(*c3_info, c3_b_emlrt_marshallOut(0U), "mFileTimeHi",
                  "mFileTimeHi", 53);
  sf_mex_assign(&c3_rhs53, sf_mex_createcellmatrix(0, 1), false);
  sf_mex_assign(&c3_lhs53, sf_mex_createcellmatrix(0, 1), false);
  sf_mex_addfield(*c3_info, sf_mex_duplicatearraysafe(&c3_rhs53), "rhs", "rhs",
                  53);
  sf_mex_addfield(*c3_info, sf_mex_duplicatearraysafe(&c3_lhs53), "lhs", "lhs",
                  53);
  sf_mex_addfield(*c3_info, c3_emlrt_marshallOut(
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/eml/eml_realmin.m"), "context",
                  "context", 54);
  sf_mex_addfield(*c3_info, c3_emlrt_marshallOut("eml_float_model"), "name",
                  "name", 54);
  sf_mex_addfield(*c3_info, c3_emlrt_marshallOut("char"), "dominantType",
                  "dominantType", 54);
  sf_mex_addfield(*c3_info, c3_emlrt_marshallOut(
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/eml/eml_float_model.m"),
                  "resolved", "resolved", 54);
  sf_mex_addfield(*c3_info, c3_b_emlrt_marshallOut(1326753196U), "fileTimeLo",
                  "fileTimeLo", 54);
  sf_mex_addfield(*c3_info, c3_b_emlrt_marshallOut(0U), "fileTimeHi",
                  "fileTimeHi", 54);
  sf_mex_addfield(*c3_info, c3_b_emlrt_marshallOut(0U), "mFileTimeLo",
                  "mFileTimeLo", 54);
  sf_mex_addfield(*c3_info, c3_b_emlrt_marshallOut(0U), "mFileTimeHi",
                  "mFileTimeHi", 54);
  sf_mex_assign(&c3_rhs54, sf_mex_createcellmatrix(0, 1), false);
  sf_mex_assign(&c3_lhs54, sf_mex_createcellmatrix(0, 1), false);
  sf_mex_addfield(*c3_info, sf_mex_duplicatearraysafe(&c3_rhs54), "rhs", "rhs",
                  54);
  sf_mex_addfield(*c3_info, sf_mex_duplicatearraysafe(&c3_lhs54), "lhs", "lhs",
                  54);
  sf_mex_addfield(*c3_info, c3_emlrt_marshallOut(
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/eml/lapack/matlab/eml_matlab_zgetrf.m"),
                  "context", "context", 55);
  sf_mex_addfield(*c3_info, c3_emlrt_marshallOut("eps"), "name", "name", 55);
  sf_mex_addfield(*c3_info, c3_emlrt_marshallOut("char"), "dominantType",
                  "dominantType", 55);
  sf_mex_addfield(*c3_info, c3_emlrt_marshallOut(
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/elmat/eps.m"), "resolved",
                  "resolved", 55);
  sf_mex_addfield(*c3_info, c3_b_emlrt_marshallOut(1326753196U), "fileTimeLo",
                  "fileTimeLo", 55);
  sf_mex_addfield(*c3_info, c3_b_emlrt_marshallOut(0U), "fileTimeHi",
                  "fileTimeHi", 55);
  sf_mex_addfield(*c3_info, c3_b_emlrt_marshallOut(0U), "mFileTimeLo",
                  "mFileTimeLo", 55);
  sf_mex_addfield(*c3_info, c3_b_emlrt_marshallOut(0U), "mFileTimeHi",
                  "mFileTimeHi", 55);
  sf_mex_assign(&c3_rhs55, sf_mex_createcellmatrix(0, 1), false);
  sf_mex_assign(&c3_lhs55, sf_mex_createcellmatrix(0, 1), false);
  sf_mex_addfield(*c3_info, sf_mex_duplicatearraysafe(&c3_rhs55), "rhs", "rhs",
                  55);
  sf_mex_addfield(*c3_info, sf_mex_duplicatearraysafe(&c3_lhs55), "lhs", "lhs",
                  55);
  sf_mex_addfield(*c3_info, c3_emlrt_marshallOut(
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/elmat/eps.m"), "context",
                  "context", 56);
  sf_mex_addfield(*c3_info, c3_emlrt_marshallOut("eml_is_float_class"), "name",
                  "name", 56);
  sf_mex_addfield(*c3_info, c3_emlrt_marshallOut("char"), "dominantType",
                  "dominantType", 56);
  sf_mex_addfield(*c3_info, c3_emlrt_marshallOut(
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/eml/eml_is_float_class.m"),
                  "resolved", "resolved", 56);
  sf_mex_addfield(*c3_info, c3_b_emlrt_marshallOut(1286843982U), "fileTimeLo",
                  "fileTimeLo", 56);
  sf_mex_addfield(*c3_info, c3_b_emlrt_marshallOut(0U), "fileTimeHi",
                  "fileTimeHi", 56);
  sf_mex_addfield(*c3_info, c3_b_emlrt_marshallOut(0U), "mFileTimeLo",
                  "mFileTimeLo", 56);
  sf_mex_addfield(*c3_info, c3_b_emlrt_marshallOut(0U), "mFileTimeHi",
                  "mFileTimeHi", 56);
  sf_mex_assign(&c3_rhs56, sf_mex_createcellmatrix(0, 1), false);
  sf_mex_assign(&c3_lhs56, sf_mex_createcellmatrix(0, 1), false);
  sf_mex_addfield(*c3_info, sf_mex_duplicatearraysafe(&c3_rhs56), "rhs", "rhs",
                  56);
  sf_mex_addfield(*c3_info, sf_mex_duplicatearraysafe(&c3_lhs56), "lhs", "lhs",
                  56);
  sf_mex_addfield(*c3_info, c3_emlrt_marshallOut(
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/eml/lapack/matlab/eml_matlab_zgetrf.m"),
                  "context", "context", 57);
  sf_mex_addfield(*c3_info, c3_emlrt_marshallOut("min"), "name", "name", 57);
  sf_mex_addfield(*c3_info, c3_emlrt_marshallOut("coder.internal.indexInt"),
                  "dominantType", "dominantType", 57);
  sf_mex_addfield(*c3_info, c3_emlrt_marshallOut(
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/datafun/min.m"), "resolved",
                  "resolved", 57);
  sf_mex_addfield(*c3_info, c3_b_emlrt_marshallOut(1311280518U), "fileTimeLo",
                  "fileTimeLo", 57);
  sf_mex_addfield(*c3_info, c3_b_emlrt_marshallOut(0U), "fileTimeHi",
                  "fileTimeHi", 57);
  sf_mex_addfield(*c3_info, c3_b_emlrt_marshallOut(0U), "mFileTimeLo",
                  "mFileTimeLo", 57);
  sf_mex_addfield(*c3_info, c3_b_emlrt_marshallOut(0U), "mFileTimeHi",
                  "mFileTimeHi", 57);
  sf_mex_assign(&c3_rhs57, sf_mex_createcellmatrix(0, 1), false);
  sf_mex_assign(&c3_lhs57, sf_mex_createcellmatrix(0, 1), false);
  sf_mex_addfield(*c3_info, sf_mex_duplicatearraysafe(&c3_rhs57), "rhs", "rhs",
                  57);
  sf_mex_addfield(*c3_info, sf_mex_duplicatearraysafe(&c3_lhs57), "lhs", "lhs",
                  57);
  sf_mex_addfield(*c3_info, c3_emlrt_marshallOut(
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/datafun/min.m"), "context",
                  "context", 58);
  sf_mex_addfield(*c3_info, c3_emlrt_marshallOut("eml_min_or_max"), "name",
                  "name", 58);
  sf_mex_addfield(*c3_info, c3_emlrt_marshallOut("char"), "dominantType",
                  "dominantType", 58);
  sf_mex_addfield(*c3_info, c3_emlrt_marshallOut(
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/eml/eml_min_or_max.m"),
                  "resolved", "resolved", 58);
  sf_mex_addfield(*c3_info, c3_b_emlrt_marshallOut(1378321184U), "fileTimeLo",
                  "fileTimeLo", 58);
  sf_mex_addfield(*c3_info, c3_b_emlrt_marshallOut(0U), "fileTimeHi",
                  "fileTimeHi", 58);
  sf_mex_addfield(*c3_info, c3_b_emlrt_marshallOut(0U), "mFileTimeLo",
                  "mFileTimeLo", 58);
  sf_mex_addfield(*c3_info, c3_b_emlrt_marshallOut(0U), "mFileTimeHi",
                  "mFileTimeHi", 58);
  sf_mex_assign(&c3_rhs58, sf_mex_createcellmatrix(0, 1), false);
  sf_mex_assign(&c3_lhs58, sf_mex_createcellmatrix(0, 1), false);
  sf_mex_addfield(*c3_info, sf_mex_duplicatearraysafe(&c3_rhs58), "rhs", "rhs",
                  58);
  sf_mex_addfield(*c3_info, sf_mex_duplicatearraysafe(&c3_lhs58), "lhs", "lhs",
                  58);
  sf_mex_addfield(*c3_info, c3_emlrt_marshallOut(
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/eml/eml_min_or_max.m!eml_bin_extremum"),
                  "context", "context", 59);
  sf_mex_addfield(*c3_info, c3_emlrt_marshallOut("eml_scalar_eg"), "name",
                  "name", 59);
  sf_mex_addfield(*c3_info, c3_emlrt_marshallOut("coder.internal.indexInt"),
                  "dominantType", "dominantType", 59);
  sf_mex_addfield(*c3_info, c3_emlrt_marshallOut(
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/eml/eml_scalar_eg.m"), "resolved",
                  "resolved", 59);
  sf_mex_addfield(*c3_info, c3_b_emlrt_marshallOut(1376005888U), "fileTimeLo",
                  "fileTimeLo", 59);
  sf_mex_addfield(*c3_info, c3_b_emlrt_marshallOut(0U), "fileTimeHi",
                  "fileTimeHi", 59);
  sf_mex_addfield(*c3_info, c3_b_emlrt_marshallOut(0U), "mFileTimeLo",
                  "mFileTimeLo", 59);
  sf_mex_addfield(*c3_info, c3_b_emlrt_marshallOut(0U), "mFileTimeHi",
                  "mFileTimeHi", 59);
  sf_mex_assign(&c3_rhs59, sf_mex_createcellmatrix(0, 1), false);
  sf_mex_assign(&c3_lhs59, sf_mex_createcellmatrix(0, 1), false);
  sf_mex_addfield(*c3_info, sf_mex_duplicatearraysafe(&c3_rhs59), "rhs", "rhs",
                  59);
  sf_mex_addfield(*c3_info, sf_mex_duplicatearraysafe(&c3_lhs59), "lhs", "lhs",
                  59);
  sf_mex_addfield(*c3_info, c3_emlrt_marshallOut(
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/eml/eml_scalar_eg.m"), "context",
                  "context", 60);
  sf_mex_addfield(*c3_info, c3_emlrt_marshallOut("coder.internal.scalarEg"),
                  "name", "name", 60);
  sf_mex_addfield(*c3_info, c3_emlrt_marshallOut("coder.internal.indexInt"),
                  "dominantType", "dominantType", 60);
  sf_mex_addfield(*c3_info, c3_emlrt_marshallOut(
    "[ILXE]$matlabroot$/toolbox/coder/coder/+coder/+internal/scalarEg.p"),
                  "resolved", "resolved", 60);
  sf_mex_addfield(*c3_info, c3_b_emlrt_marshallOut(1410832970U), "fileTimeLo",
                  "fileTimeLo", 60);
  sf_mex_addfield(*c3_info, c3_b_emlrt_marshallOut(0U), "fileTimeHi",
                  "fileTimeHi", 60);
  sf_mex_addfield(*c3_info, c3_b_emlrt_marshallOut(0U), "mFileTimeLo",
                  "mFileTimeLo", 60);
  sf_mex_addfield(*c3_info, c3_b_emlrt_marshallOut(0U), "mFileTimeHi",
                  "mFileTimeHi", 60);
  sf_mex_assign(&c3_rhs60, sf_mex_createcellmatrix(0, 1), false);
  sf_mex_assign(&c3_lhs60, sf_mex_createcellmatrix(0, 1), false);
  sf_mex_addfield(*c3_info, sf_mex_duplicatearraysafe(&c3_rhs60), "rhs", "rhs",
                  60);
  sf_mex_addfield(*c3_info, sf_mex_duplicatearraysafe(&c3_lhs60), "lhs", "lhs",
                  60);
  sf_mex_addfield(*c3_info, c3_emlrt_marshallOut(
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/eml/eml_min_or_max.m!eml_bin_extremum"),
                  "context", "context", 61);
  sf_mex_addfield(*c3_info, c3_emlrt_marshallOut("eml_scalexp_alloc"), "name",
                  "name", 61);
  sf_mex_addfield(*c3_info, c3_emlrt_marshallOut("coder.internal.indexInt"),
                  "dominantType", "dominantType", 61);
  sf_mex_addfield(*c3_info, c3_emlrt_marshallOut(
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/eml/eml_scalexp_alloc.m"),
                  "resolved", "resolved", 61);
  sf_mex_addfield(*c3_info, c3_b_emlrt_marshallOut(1376005888U), "fileTimeLo",
                  "fileTimeLo", 61);
  sf_mex_addfield(*c3_info, c3_b_emlrt_marshallOut(0U), "fileTimeHi",
                  "fileTimeHi", 61);
  sf_mex_addfield(*c3_info, c3_b_emlrt_marshallOut(0U), "mFileTimeLo",
                  "mFileTimeLo", 61);
  sf_mex_addfield(*c3_info, c3_b_emlrt_marshallOut(0U), "mFileTimeHi",
                  "mFileTimeHi", 61);
  sf_mex_assign(&c3_rhs61, sf_mex_createcellmatrix(0, 1), false);
  sf_mex_assign(&c3_lhs61, sf_mex_createcellmatrix(0, 1), false);
  sf_mex_addfield(*c3_info, sf_mex_duplicatearraysafe(&c3_rhs61), "rhs", "rhs",
                  61);
  sf_mex_addfield(*c3_info, sf_mex_duplicatearraysafe(&c3_lhs61), "lhs", "lhs",
                  61);
  sf_mex_addfield(*c3_info, c3_emlrt_marshallOut(
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/eml/eml_scalexp_alloc.m"),
                  "context", "context", 62);
  sf_mex_addfield(*c3_info, c3_emlrt_marshallOut("coder.internal.scalexpAlloc"),
                  "name", "name", 62);
  sf_mex_addfield(*c3_info, c3_emlrt_marshallOut("coder.internal.indexInt"),
                  "dominantType", "dominantType", 62);
  sf_mex_addfield(*c3_info, c3_emlrt_marshallOut(
    "[ILXE]$matlabroot$/toolbox/coder/coder/+coder/+internal/scalexpAlloc.p"),
                  "resolved", "resolved", 62);
  sf_mex_addfield(*c3_info, c3_b_emlrt_marshallOut(1410832970U), "fileTimeLo",
                  "fileTimeLo", 62);
  sf_mex_addfield(*c3_info, c3_b_emlrt_marshallOut(0U), "fileTimeHi",
                  "fileTimeHi", 62);
  sf_mex_addfield(*c3_info, c3_b_emlrt_marshallOut(0U), "mFileTimeLo",
                  "mFileTimeLo", 62);
  sf_mex_addfield(*c3_info, c3_b_emlrt_marshallOut(0U), "mFileTimeHi",
                  "mFileTimeHi", 62);
  sf_mex_assign(&c3_rhs62, sf_mex_createcellmatrix(0, 1), false);
  sf_mex_assign(&c3_lhs62, sf_mex_createcellmatrix(0, 1), false);
  sf_mex_addfield(*c3_info, sf_mex_duplicatearraysafe(&c3_rhs62), "rhs", "rhs",
                  62);
  sf_mex_addfield(*c3_info, sf_mex_duplicatearraysafe(&c3_lhs62), "lhs", "lhs",
                  62);
  sf_mex_addfield(*c3_info, c3_emlrt_marshallOut(
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/eml/eml_min_or_max.m!eml_bin_extremum"),
                  "context", "context", 63);
  sf_mex_addfield(*c3_info, c3_emlrt_marshallOut("eml_index_class"), "name",
                  "name", 63);
  sf_mex_addfield(*c3_info, c3_emlrt_marshallOut(""), "dominantType",
                  "dominantType", 63);
  sf_mex_addfield(*c3_info, c3_emlrt_marshallOut(
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/eml/eml_index_class.m"),
                  "resolved", "resolved", 63);
  sf_mex_addfield(*c3_info, c3_b_emlrt_marshallOut(1323195778U), "fileTimeLo",
                  "fileTimeLo", 63);
  sf_mex_addfield(*c3_info, c3_b_emlrt_marshallOut(0U), "fileTimeHi",
                  "fileTimeHi", 63);
  sf_mex_addfield(*c3_info, c3_b_emlrt_marshallOut(0U), "mFileTimeLo",
                  "mFileTimeLo", 63);
  sf_mex_addfield(*c3_info, c3_b_emlrt_marshallOut(0U), "mFileTimeHi",
                  "mFileTimeHi", 63);
  sf_mex_assign(&c3_rhs63, sf_mex_createcellmatrix(0, 1), false);
  sf_mex_assign(&c3_lhs63, sf_mex_createcellmatrix(0, 1), false);
  sf_mex_addfield(*c3_info, sf_mex_duplicatearraysafe(&c3_rhs63), "rhs", "rhs",
                  63);
  sf_mex_addfield(*c3_info, sf_mex_duplicatearraysafe(&c3_lhs63), "lhs", "lhs",
                  63);
  sf_mex_destroy(&c3_rhs0);
  sf_mex_destroy(&c3_lhs0);
  sf_mex_destroy(&c3_rhs1);
  sf_mex_destroy(&c3_lhs1);
  sf_mex_destroy(&c3_rhs2);
  sf_mex_destroy(&c3_lhs2);
  sf_mex_destroy(&c3_rhs3);
  sf_mex_destroy(&c3_lhs3);
  sf_mex_destroy(&c3_rhs4);
  sf_mex_destroy(&c3_lhs4);
  sf_mex_destroy(&c3_rhs5);
  sf_mex_destroy(&c3_lhs5);
  sf_mex_destroy(&c3_rhs6);
  sf_mex_destroy(&c3_lhs6);
  sf_mex_destroy(&c3_rhs7);
  sf_mex_destroy(&c3_lhs7);
  sf_mex_destroy(&c3_rhs8);
  sf_mex_destroy(&c3_lhs8);
  sf_mex_destroy(&c3_rhs9);
  sf_mex_destroy(&c3_lhs9);
  sf_mex_destroy(&c3_rhs10);
  sf_mex_destroy(&c3_lhs10);
  sf_mex_destroy(&c3_rhs11);
  sf_mex_destroy(&c3_lhs11);
  sf_mex_destroy(&c3_rhs12);
  sf_mex_destroy(&c3_lhs12);
  sf_mex_destroy(&c3_rhs13);
  sf_mex_destroy(&c3_lhs13);
  sf_mex_destroy(&c3_rhs14);
  sf_mex_destroy(&c3_lhs14);
  sf_mex_destroy(&c3_rhs15);
  sf_mex_destroy(&c3_lhs15);
  sf_mex_destroy(&c3_rhs16);
  sf_mex_destroy(&c3_lhs16);
  sf_mex_destroy(&c3_rhs17);
  sf_mex_destroy(&c3_lhs17);
  sf_mex_destroy(&c3_rhs18);
  sf_mex_destroy(&c3_lhs18);
  sf_mex_destroy(&c3_rhs19);
  sf_mex_destroy(&c3_lhs19);
  sf_mex_destroy(&c3_rhs20);
  sf_mex_destroy(&c3_lhs20);
  sf_mex_destroy(&c3_rhs21);
  sf_mex_destroy(&c3_lhs21);
  sf_mex_destroy(&c3_rhs22);
  sf_mex_destroy(&c3_lhs22);
  sf_mex_destroy(&c3_rhs23);
  sf_mex_destroy(&c3_lhs23);
  sf_mex_destroy(&c3_rhs24);
  sf_mex_destroy(&c3_lhs24);
  sf_mex_destroy(&c3_rhs25);
  sf_mex_destroy(&c3_lhs25);
  sf_mex_destroy(&c3_rhs26);
  sf_mex_destroy(&c3_lhs26);
  sf_mex_destroy(&c3_rhs27);
  sf_mex_destroy(&c3_lhs27);
  sf_mex_destroy(&c3_rhs28);
  sf_mex_destroy(&c3_lhs28);
  sf_mex_destroy(&c3_rhs29);
  sf_mex_destroy(&c3_lhs29);
  sf_mex_destroy(&c3_rhs30);
  sf_mex_destroy(&c3_lhs30);
  sf_mex_destroy(&c3_rhs31);
  sf_mex_destroy(&c3_lhs31);
  sf_mex_destroy(&c3_rhs32);
  sf_mex_destroy(&c3_lhs32);
  sf_mex_destroy(&c3_rhs33);
  sf_mex_destroy(&c3_lhs33);
  sf_mex_destroy(&c3_rhs34);
  sf_mex_destroy(&c3_lhs34);
  sf_mex_destroy(&c3_rhs35);
  sf_mex_destroy(&c3_lhs35);
  sf_mex_destroy(&c3_rhs36);
  sf_mex_destroy(&c3_lhs36);
  sf_mex_destroy(&c3_rhs37);
  sf_mex_destroy(&c3_lhs37);
  sf_mex_destroy(&c3_rhs38);
  sf_mex_destroy(&c3_lhs38);
  sf_mex_destroy(&c3_rhs39);
  sf_mex_destroy(&c3_lhs39);
  sf_mex_destroy(&c3_rhs40);
  sf_mex_destroy(&c3_lhs40);
  sf_mex_destroy(&c3_rhs41);
  sf_mex_destroy(&c3_lhs41);
  sf_mex_destroy(&c3_rhs42);
  sf_mex_destroy(&c3_lhs42);
  sf_mex_destroy(&c3_rhs43);
  sf_mex_destroy(&c3_lhs43);
  sf_mex_destroy(&c3_rhs44);
  sf_mex_destroy(&c3_lhs44);
  sf_mex_destroy(&c3_rhs45);
  sf_mex_destroy(&c3_lhs45);
  sf_mex_destroy(&c3_rhs46);
  sf_mex_destroy(&c3_lhs46);
  sf_mex_destroy(&c3_rhs47);
  sf_mex_destroy(&c3_lhs47);
  sf_mex_destroy(&c3_rhs48);
  sf_mex_destroy(&c3_lhs48);
  sf_mex_destroy(&c3_rhs49);
  sf_mex_destroy(&c3_lhs49);
  sf_mex_destroy(&c3_rhs50);
  sf_mex_destroy(&c3_lhs50);
  sf_mex_destroy(&c3_rhs51);
  sf_mex_destroy(&c3_lhs51);
  sf_mex_destroy(&c3_rhs52);
  sf_mex_destroy(&c3_lhs52);
  sf_mex_destroy(&c3_rhs53);
  sf_mex_destroy(&c3_lhs53);
  sf_mex_destroy(&c3_rhs54);
  sf_mex_destroy(&c3_lhs54);
  sf_mex_destroy(&c3_rhs55);
  sf_mex_destroy(&c3_lhs55);
  sf_mex_destroy(&c3_rhs56);
  sf_mex_destroy(&c3_lhs56);
  sf_mex_destroy(&c3_rhs57);
  sf_mex_destroy(&c3_lhs57);
  sf_mex_destroy(&c3_rhs58);
  sf_mex_destroy(&c3_lhs58);
  sf_mex_destroy(&c3_rhs59);
  sf_mex_destroy(&c3_lhs59);
  sf_mex_destroy(&c3_rhs60);
  sf_mex_destroy(&c3_lhs60);
  sf_mex_destroy(&c3_rhs61);
  sf_mex_destroy(&c3_lhs61);
  sf_mex_destroy(&c3_rhs62);
  sf_mex_destroy(&c3_lhs62);
  sf_mex_destroy(&c3_rhs63);
  sf_mex_destroy(&c3_lhs63);
}

static const mxArray *c3_emlrt_marshallOut(const char * c3_u)
{
  const mxArray *c3_y = NULL;
  c3_y = NULL;
  sf_mex_assign(&c3_y, sf_mex_create("y", c3_u, 15, 0U, 0U, 0U, 2, 1, strlen
    (c3_u)), false);
  return c3_y;
}

static const mxArray *c3_b_emlrt_marshallOut(const uint32_T c3_u)
{
  const mxArray *c3_y = NULL;
  c3_y = NULL;
  sf_mex_assign(&c3_y, sf_mex_create("y", &c3_u, 7, 0U, 0U, 0U, 0), false);
  return c3_y;
}

static void c3_b_info_helper(const mxArray **c3_info)
{
  const mxArray *c3_rhs64 = NULL;
  const mxArray *c3_lhs64 = NULL;
  const mxArray *c3_rhs65 = NULL;
  const mxArray *c3_lhs65 = NULL;
  const mxArray *c3_rhs66 = NULL;
  const mxArray *c3_lhs66 = NULL;
  const mxArray *c3_rhs67 = NULL;
  const mxArray *c3_lhs67 = NULL;
  const mxArray *c3_rhs68 = NULL;
  const mxArray *c3_lhs68 = NULL;
  const mxArray *c3_rhs69 = NULL;
  const mxArray *c3_lhs69 = NULL;
  const mxArray *c3_rhs70 = NULL;
  const mxArray *c3_lhs70 = NULL;
  const mxArray *c3_rhs71 = NULL;
  const mxArray *c3_lhs71 = NULL;
  const mxArray *c3_rhs72 = NULL;
  const mxArray *c3_lhs72 = NULL;
  const mxArray *c3_rhs73 = NULL;
  const mxArray *c3_lhs73 = NULL;
  const mxArray *c3_rhs74 = NULL;
  const mxArray *c3_lhs74 = NULL;
  const mxArray *c3_rhs75 = NULL;
  const mxArray *c3_lhs75 = NULL;
  const mxArray *c3_rhs76 = NULL;
  const mxArray *c3_lhs76 = NULL;
  const mxArray *c3_rhs77 = NULL;
  const mxArray *c3_lhs77 = NULL;
  const mxArray *c3_rhs78 = NULL;
  const mxArray *c3_lhs78 = NULL;
  const mxArray *c3_rhs79 = NULL;
  const mxArray *c3_lhs79 = NULL;
  const mxArray *c3_rhs80 = NULL;
  const mxArray *c3_lhs80 = NULL;
  const mxArray *c3_rhs81 = NULL;
  const mxArray *c3_lhs81 = NULL;
  const mxArray *c3_rhs82 = NULL;
  const mxArray *c3_lhs82 = NULL;
  const mxArray *c3_rhs83 = NULL;
  const mxArray *c3_lhs83 = NULL;
  const mxArray *c3_rhs84 = NULL;
  const mxArray *c3_lhs84 = NULL;
  const mxArray *c3_rhs85 = NULL;
  const mxArray *c3_lhs85 = NULL;
  const mxArray *c3_rhs86 = NULL;
  const mxArray *c3_lhs86 = NULL;
  const mxArray *c3_rhs87 = NULL;
  const mxArray *c3_lhs87 = NULL;
  const mxArray *c3_rhs88 = NULL;
  const mxArray *c3_lhs88 = NULL;
  const mxArray *c3_rhs89 = NULL;
  const mxArray *c3_lhs89 = NULL;
  const mxArray *c3_rhs90 = NULL;
  const mxArray *c3_lhs90 = NULL;
  const mxArray *c3_rhs91 = NULL;
  const mxArray *c3_lhs91 = NULL;
  const mxArray *c3_rhs92 = NULL;
  const mxArray *c3_lhs92 = NULL;
  const mxArray *c3_rhs93 = NULL;
  const mxArray *c3_lhs93 = NULL;
  const mxArray *c3_rhs94 = NULL;
  const mxArray *c3_lhs94 = NULL;
  const mxArray *c3_rhs95 = NULL;
  const mxArray *c3_lhs95 = NULL;
  const mxArray *c3_rhs96 = NULL;
  const mxArray *c3_lhs96 = NULL;
  const mxArray *c3_rhs97 = NULL;
  const mxArray *c3_lhs97 = NULL;
  const mxArray *c3_rhs98 = NULL;
  const mxArray *c3_lhs98 = NULL;
  const mxArray *c3_rhs99 = NULL;
  const mxArray *c3_lhs99 = NULL;
  const mxArray *c3_rhs100 = NULL;
  const mxArray *c3_lhs100 = NULL;
  const mxArray *c3_rhs101 = NULL;
  const mxArray *c3_lhs101 = NULL;
  const mxArray *c3_rhs102 = NULL;
  const mxArray *c3_lhs102 = NULL;
  const mxArray *c3_rhs103 = NULL;
  const mxArray *c3_lhs103 = NULL;
  const mxArray *c3_rhs104 = NULL;
  const mxArray *c3_lhs104 = NULL;
  const mxArray *c3_rhs105 = NULL;
  const mxArray *c3_lhs105 = NULL;
  const mxArray *c3_rhs106 = NULL;
  const mxArray *c3_lhs106 = NULL;
  const mxArray *c3_rhs107 = NULL;
  const mxArray *c3_lhs107 = NULL;
  const mxArray *c3_rhs108 = NULL;
  const mxArray *c3_lhs108 = NULL;
  const mxArray *c3_rhs109 = NULL;
  const mxArray *c3_lhs109 = NULL;
  const mxArray *c3_rhs110 = NULL;
  const mxArray *c3_lhs110 = NULL;
  const mxArray *c3_rhs111 = NULL;
  const mxArray *c3_lhs111 = NULL;
  const mxArray *c3_rhs112 = NULL;
  const mxArray *c3_lhs112 = NULL;
  const mxArray *c3_rhs113 = NULL;
  const mxArray *c3_lhs113 = NULL;
  const mxArray *c3_rhs114 = NULL;
  const mxArray *c3_lhs114 = NULL;
  const mxArray *c3_rhs115 = NULL;
  const mxArray *c3_lhs115 = NULL;
  const mxArray *c3_rhs116 = NULL;
  const mxArray *c3_lhs116 = NULL;
  const mxArray *c3_rhs117 = NULL;
  const mxArray *c3_lhs117 = NULL;
  const mxArray *c3_rhs118 = NULL;
  const mxArray *c3_lhs118 = NULL;
  const mxArray *c3_rhs119 = NULL;
  const mxArray *c3_lhs119 = NULL;
  const mxArray *c3_rhs120 = NULL;
  const mxArray *c3_lhs120 = NULL;
  const mxArray *c3_rhs121 = NULL;
  const mxArray *c3_lhs121 = NULL;
  const mxArray *c3_rhs122 = NULL;
  const mxArray *c3_lhs122 = NULL;
  const mxArray *c3_rhs123 = NULL;
  const mxArray *c3_lhs123 = NULL;
  const mxArray *c3_rhs124 = NULL;
  const mxArray *c3_lhs124 = NULL;
  const mxArray *c3_rhs125 = NULL;
  const mxArray *c3_lhs125 = NULL;
  const mxArray *c3_rhs126 = NULL;
  const mxArray *c3_lhs126 = NULL;
  const mxArray *c3_rhs127 = NULL;
  const mxArray *c3_lhs127 = NULL;
  sf_mex_addfield(*c3_info, c3_emlrt_marshallOut(
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/eml/eml_min_or_max.m!eml_scalar_bin_extremum"),
                  "context", "context", 64);
  sf_mex_addfield(*c3_info, c3_emlrt_marshallOut("eml_scalar_eg"), "name",
                  "name", 64);
  sf_mex_addfield(*c3_info, c3_emlrt_marshallOut("coder.internal.indexInt"),
                  "dominantType", "dominantType", 64);
  sf_mex_addfield(*c3_info, c3_emlrt_marshallOut(
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/eml/eml_scalar_eg.m"), "resolved",
                  "resolved", 64);
  sf_mex_addfield(*c3_info, c3_b_emlrt_marshallOut(1376005888U), "fileTimeLo",
                  "fileTimeLo", 64);
  sf_mex_addfield(*c3_info, c3_b_emlrt_marshallOut(0U), "fileTimeHi",
                  "fileTimeHi", 64);
  sf_mex_addfield(*c3_info, c3_b_emlrt_marshallOut(0U), "mFileTimeLo",
                  "mFileTimeLo", 64);
  sf_mex_addfield(*c3_info, c3_b_emlrt_marshallOut(0U), "mFileTimeHi",
                  "mFileTimeHi", 64);
  sf_mex_assign(&c3_rhs64, sf_mex_createcellmatrix(0, 1), false);
  sf_mex_assign(&c3_lhs64, sf_mex_createcellmatrix(0, 1), false);
  sf_mex_addfield(*c3_info, sf_mex_duplicatearraysafe(&c3_rhs64), "rhs", "rhs",
                  64);
  sf_mex_addfield(*c3_info, sf_mex_duplicatearraysafe(&c3_lhs64), "lhs", "lhs",
                  64);
  sf_mex_addfield(*c3_info, c3_emlrt_marshallOut(
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/eml/eml_min_or_max.m!eml_scalar_bin_extremum"),
                  "context", "context", 65);
  sf_mex_addfield(*c3_info, c3_emlrt_marshallOut(
    "coder.internal.isBuiltInNumeric"), "name", "name", 65);
  sf_mex_addfield(*c3_info, c3_emlrt_marshallOut("coder.internal.indexInt"),
                  "dominantType", "dominantType", 65);
  sf_mex_addfield(*c3_info, c3_emlrt_marshallOut(
    "[ILXE]$matlabroot$/toolbox/shared/coder/coder/+coder/+internal/isBuiltInNumeric.m"),
                  "resolved", "resolved", 65);
  sf_mex_addfield(*c3_info, c3_b_emlrt_marshallOut(1395953456U), "fileTimeLo",
                  "fileTimeLo", 65);
  sf_mex_addfield(*c3_info, c3_b_emlrt_marshallOut(0U), "fileTimeHi",
                  "fileTimeHi", 65);
  sf_mex_addfield(*c3_info, c3_b_emlrt_marshallOut(0U), "mFileTimeLo",
                  "mFileTimeLo", 65);
  sf_mex_addfield(*c3_info, c3_b_emlrt_marshallOut(0U), "mFileTimeHi",
                  "mFileTimeHi", 65);
  sf_mex_assign(&c3_rhs65, sf_mex_createcellmatrix(0, 1), false);
  sf_mex_assign(&c3_lhs65, sf_mex_createcellmatrix(0, 1), false);
  sf_mex_addfield(*c3_info, sf_mex_duplicatearraysafe(&c3_rhs65), "rhs", "rhs",
                  65);
  sf_mex_addfield(*c3_info, sf_mex_duplicatearraysafe(&c3_lhs65), "lhs", "lhs",
                  65);
  sf_mex_addfield(*c3_info, c3_emlrt_marshallOut(
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/eml/lapack/matlab/eml_matlab_zgetrf.m"),
                  "context", "context", 66);
  sf_mex_addfield(*c3_info, c3_emlrt_marshallOut("colon"), "name", "name", 66);
  sf_mex_addfield(*c3_info, c3_emlrt_marshallOut("double"), "dominantType",
                  "dominantType", 66);
  sf_mex_addfield(*c3_info, c3_emlrt_marshallOut(
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/ops/colon.m"), "resolved",
                  "resolved", 66);
  sf_mex_addfield(*c3_info, c3_b_emlrt_marshallOut(1378321188U), "fileTimeLo",
                  "fileTimeLo", 66);
  sf_mex_addfield(*c3_info, c3_b_emlrt_marshallOut(0U), "fileTimeHi",
                  "fileTimeHi", 66);
  sf_mex_addfield(*c3_info, c3_b_emlrt_marshallOut(0U), "mFileTimeLo",
                  "mFileTimeLo", 66);
  sf_mex_addfield(*c3_info, c3_b_emlrt_marshallOut(0U), "mFileTimeHi",
                  "mFileTimeHi", 66);
  sf_mex_assign(&c3_rhs66, sf_mex_createcellmatrix(0, 1), false);
  sf_mex_assign(&c3_lhs66, sf_mex_createcellmatrix(0, 1), false);
  sf_mex_addfield(*c3_info, sf_mex_duplicatearraysafe(&c3_rhs66), "rhs", "rhs",
                  66);
  sf_mex_addfield(*c3_info, sf_mex_duplicatearraysafe(&c3_lhs66), "lhs", "lhs",
                  66);
  sf_mex_addfield(*c3_info, c3_emlrt_marshallOut(
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/ops/colon.m"), "context",
                  "context", 67);
  sf_mex_addfield(*c3_info, c3_emlrt_marshallOut("colon"), "name", "name", 67);
  sf_mex_addfield(*c3_info, c3_emlrt_marshallOut("double"), "dominantType",
                  "dominantType", 67);
  sf_mex_addfield(*c3_info, c3_emlrt_marshallOut(
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/ops/colon.m"), "resolved",
                  "resolved", 67);
  sf_mex_addfield(*c3_info, c3_b_emlrt_marshallOut(1378321188U), "fileTimeLo",
                  "fileTimeLo", 67);
  sf_mex_addfield(*c3_info, c3_b_emlrt_marshallOut(0U), "fileTimeHi",
                  "fileTimeHi", 67);
  sf_mex_addfield(*c3_info, c3_b_emlrt_marshallOut(0U), "mFileTimeLo",
                  "mFileTimeLo", 67);
  sf_mex_addfield(*c3_info, c3_b_emlrt_marshallOut(0U), "mFileTimeHi",
                  "mFileTimeHi", 67);
  sf_mex_assign(&c3_rhs67, sf_mex_createcellmatrix(0, 1), false);
  sf_mex_assign(&c3_lhs67, sf_mex_createcellmatrix(0, 1), false);
  sf_mex_addfield(*c3_info, sf_mex_duplicatearraysafe(&c3_rhs67), "rhs", "rhs",
                  67);
  sf_mex_addfield(*c3_info, sf_mex_duplicatearraysafe(&c3_lhs67), "lhs", "lhs",
                  67);
  sf_mex_addfield(*c3_info, c3_emlrt_marshallOut(
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/ops/colon.m"), "context",
                  "context", 68);
  sf_mex_addfield(*c3_info, c3_emlrt_marshallOut(
    "coder.internal.isBuiltInNumeric"), "name", "name", 68);
  sf_mex_addfield(*c3_info, c3_emlrt_marshallOut("double"), "dominantType",
                  "dominantType", 68);
  sf_mex_addfield(*c3_info, c3_emlrt_marshallOut(
    "[ILXE]$matlabroot$/toolbox/shared/coder/coder/+coder/+internal/isBuiltInNumeric.m"),
                  "resolved", "resolved", 68);
  sf_mex_addfield(*c3_info, c3_b_emlrt_marshallOut(1395953456U), "fileTimeLo",
                  "fileTimeLo", 68);
  sf_mex_addfield(*c3_info, c3_b_emlrt_marshallOut(0U), "fileTimeHi",
                  "fileTimeHi", 68);
  sf_mex_addfield(*c3_info, c3_b_emlrt_marshallOut(0U), "mFileTimeLo",
                  "mFileTimeLo", 68);
  sf_mex_addfield(*c3_info, c3_b_emlrt_marshallOut(0U), "mFileTimeHi",
                  "mFileTimeHi", 68);
  sf_mex_assign(&c3_rhs68, sf_mex_createcellmatrix(0, 1), false);
  sf_mex_assign(&c3_lhs68, sf_mex_createcellmatrix(0, 1), false);
  sf_mex_addfield(*c3_info, sf_mex_duplicatearraysafe(&c3_rhs68), "rhs", "rhs",
                  68);
  sf_mex_addfield(*c3_info, sf_mex_duplicatearraysafe(&c3_lhs68), "lhs", "lhs",
                  68);
  sf_mex_addfield(*c3_info, c3_emlrt_marshallOut(
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/ops/colon.m"), "context",
                  "context", 69);
  sf_mex_addfield(*c3_info, c3_emlrt_marshallOut(
    "coder.internal.isBuiltInNumeric"), "name", "name", 69);
  sf_mex_addfield(*c3_info, c3_emlrt_marshallOut("coder.internal.indexInt"),
                  "dominantType", "dominantType", 69);
  sf_mex_addfield(*c3_info, c3_emlrt_marshallOut(
    "[ILXE]$matlabroot$/toolbox/shared/coder/coder/+coder/+internal/isBuiltInNumeric.m"),
                  "resolved", "resolved", 69);
  sf_mex_addfield(*c3_info, c3_b_emlrt_marshallOut(1395953456U), "fileTimeLo",
                  "fileTimeLo", 69);
  sf_mex_addfield(*c3_info, c3_b_emlrt_marshallOut(0U), "fileTimeHi",
                  "fileTimeHi", 69);
  sf_mex_addfield(*c3_info, c3_b_emlrt_marshallOut(0U), "mFileTimeLo",
                  "mFileTimeLo", 69);
  sf_mex_addfield(*c3_info, c3_b_emlrt_marshallOut(0U), "mFileTimeHi",
                  "mFileTimeHi", 69);
  sf_mex_assign(&c3_rhs69, sf_mex_createcellmatrix(0, 1), false);
  sf_mex_assign(&c3_lhs69, sf_mex_createcellmatrix(0, 1), false);
  sf_mex_addfield(*c3_info, sf_mex_duplicatearraysafe(&c3_rhs69), "rhs", "rhs",
                  69);
  sf_mex_addfield(*c3_info, sf_mex_duplicatearraysafe(&c3_lhs69), "lhs", "lhs",
                  69);
  sf_mex_addfield(*c3_info, c3_emlrt_marshallOut(
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/ops/colon.m"), "context",
                  "context", 70);
  sf_mex_addfield(*c3_info, c3_emlrt_marshallOut("floor"), "name", "name", 70);
  sf_mex_addfield(*c3_info, c3_emlrt_marshallOut("double"), "dominantType",
                  "dominantType", 70);
  sf_mex_addfield(*c3_info, c3_emlrt_marshallOut(
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/elfun/floor.m"), "resolved",
                  "resolved", 70);
  sf_mex_addfield(*c3_info, c3_b_emlrt_marshallOut(1363735454U), "fileTimeLo",
                  "fileTimeLo", 70);
  sf_mex_addfield(*c3_info, c3_b_emlrt_marshallOut(0U), "fileTimeHi",
                  "fileTimeHi", 70);
  sf_mex_addfield(*c3_info, c3_b_emlrt_marshallOut(0U), "mFileTimeLo",
                  "mFileTimeLo", 70);
  sf_mex_addfield(*c3_info, c3_b_emlrt_marshallOut(0U), "mFileTimeHi",
                  "mFileTimeHi", 70);
  sf_mex_assign(&c3_rhs70, sf_mex_createcellmatrix(0, 1), false);
  sf_mex_assign(&c3_lhs70, sf_mex_createcellmatrix(0, 1), false);
  sf_mex_addfield(*c3_info, sf_mex_duplicatearraysafe(&c3_rhs70), "rhs", "rhs",
                  70);
  sf_mex_addfield(*c3_info, sf_mex_duplicatearraysafe(&c3_lhs70), "lhs", "lhs",
                  70);
  sf_mex_addfield(*c3_info, c3_emlrt_marshallOut(
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/elfun/floor.m"), "context",
                  "context", 71);
  sf_mex_addfield(*c3_info, c3_emlrt_marshallOut(
    "coder.internal.isBuiltInNumeric"), "name", "name", 71);
  sf_mex_addfield(*c3_info, c3_emlrt_marshallOut("double"), "dominantType",
                  "dominantType", 71);
  sf_mex_addfield(*c3_info, c3_emlrt_marshallOut(
    "[ILXE]$matlabroot$/toolbox/shared/coder/coder/+coder/+internal/isBuiltInNumeric.m"),
                  "resolved", "resolved", 71);
  sf_mex_addfield(*c3_info, c3_b_emlrt_marshallOut(1395953456U), "fileTimeLo",
                  "fileTimeLo", 71);
  sf_mex_addfield(*c3_info, c3_b_emlrt_marshallOut(0U), "fileTimeHi",
                  "fileTimeHi", 71);
  sf_mex_addfield(*c3_info, c3_b_emlrt_marshallOut(0U), "mFileTimeLo",
                  "mFileTimeLo", 71);
  sf_mex_addfield(*c3_info, c3_b_emlrt_marshallOut(0U), "mFileTimeHi",
                  "mFileTimeHi", 71);
  sf_mex_assign(&c3_rhs71, sf_mex_createcellmatrix(0, 1), false);
  sf_mex_assign(&c3_lhs71, sf_mex_createcellmatrix(0, 1), false);
  sf_mex_addfield(*c3_info, sf_mex_duplicatearraysafe(&c3_rhs71), "rhs", "rhs",
                  71);
  sf_mex_addfield(*c3_info, sf_mex_duplicatearraysafe(&c3_lhs71), "lhs", "lhs",
                  71);
  sf_mex_addfield(*c3_info, c3_emlrt_marshallOut(
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/elfun/floor.m"), "context",
                  "context", 72);
  sf_mex_addfield(*c3_info, c3_emlrt_marshallOut("eml_scalar_floor"), "name",
                  "name", 72);
  sf_mex_addfield(*c3_info, c3_emlrt_marshallOut("double"), "dominantType",
                  "dominantType", 72);
  sf_mex_addfield(*c3_info, c3_emlrt_marshallOut(
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/elfun/eml_scalar_floor.m"),
                  "resolved", "resolved", 72);
  sf_mex_addfield(*c3_info, c3_b_emlrt_marshallOut(1286843926U), "fileTimeLo",
                  "fileTimeLo", 72);
  sf_mex_addfield(*c3_info, c3_b_emlrt_marshallOut(0U), "fileTimeHi",
                  "fileTimeHi", 72);
  sf_mex_addfield(*c3_info, c3_b_emlrt_marshallOut(0U), "mFileTimeLo",
                  "mFileTimeLo", 72);
  sf_mex_addfield(*c3_info, c3_b_emlrt_marshallOut(0U), "mFileTimeHi",
                  "mFileTimeHi", 72);
  sf_mex_assign(&c3_rhs72, sf_mex_createcellmatrix(0, 1), false);
  sf_mex_assign(&c3_lhs72, sf_mex_createcellmatrix(0, 1), false);
  sf_mex_addfield(*c3_info, sf_mex_duplicatearraysafe(&c3_rhs72), "rhs", "rhs",
                  72);
  sf_mex_addfield(*c3_info, sf_mex_duplicatearraysafe(&c3_lhs72), "lhs", "lhs",
                  72);
  sf_mex_addfield(*c3_info, c3_emlrt_marshallOut(
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/ops/colon.m!checkrange"),
                  "context", "context", 73);
  sf_mex_addfield(*c3_info, c3_emlrt_marshallOut("intmin"), "name", "name", 73);
  sf_mex_addfield(*c3_info, c3_emlrt_marshallOut("char"), "dominantType",
                  "dominantType", 73);
  sf_mex_addfield(*c3_info, c3_emlrt_marshallOut(
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/elmat/intmin.m"), "resolved",
                  "resolved", 73);
  sf_mex_addfield(*c3_info, c3_b_emlrt_marshallOut(1362287082U), "fileTimeLo",
                  "fileTimeLo", 73);
  sf_mex_addfield(*c3_info, c3_b_emlrt_marshallOut(0U), "fileTimeHi",
                  "fileTimeHi", 73);
  sf_mex_addfield(*c3_info, c3_b_emlrt_marshallOut(0U), "mFileTimeLo",
                  "mFileTimeLo", 73);
  sf_mex_addfield(*c3_info, c3_b_emlrt_marshallOut(0U), "mFileTimeHi",
                  "mFileTimeHi", 73);
  sf_mex_assign(&c3_rhs73, sf_mex_createcellmatrix(0, 1), false);
  sf_mex_assign(&c3_lhs73, sf_mex_createcellmatrix(0, 1), false);
  sf_mex_addfield(*c3_info, sf_mex_duplicatearraysafe(&c3_rhs73), "rhs", "rhs",
                  73);
  sf_mex_addfield(*c3_info, sf_mex_duplicatearraysafe(&c3_lhs73), "lhs", "lhs",
                  73);
  sf_mex_addfield(*c3_info, c3_emlrt_marshallOut(
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/elmat/intmin.m"), "context",
                  "context", 74);
  sf_mex_addfield(*c3_info, c3_emlrt_marshallOut("eml_switch_helper"), "name",
                  "name", 74);
  sf_mex_addfield(*c3_info, c3_emlrt_marshallOut(""), "dominantType",
                  "dominantType", 74);
  sf_mex_addfield(*c3_info, c3_emlrt_marshallOut(
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/eml/eml_switch_helper.m"),
                  "resolved", "resolved", 74);
  sf_mex_addfield(*c3_info, c3_b_emlrt_marshallOut(1393356058U), "fileTimeLo",
                  "fileTimeLo", 74);
  sf_mex_addfield(*c3_info, c3_b_emlrt_marshallOut(0U), "fileTimeHi",
                  "fileTimeHi", 74);
  sf_mex_addfield(*c3_info, c3_b_emlrt_marshallOut(0U), "mFileTimeLo",
                  "mFileTimeLo", 74);
  sf_mex_addfield(*c3_info, c3_b_emlrt_marshallOut(0U), "mFileTimeHi",
                  "mFileTimeHi", 74);
  sf_mex_assign(&c3_rhs74, sf_mex_createcellmatrix(0, 1), false);
  sf_mex_assign(&c3_lhs74, sf_mex_createcellmatrix(0, 1), false);
  sf_mex_addfield(*c3_info, sf_mex_duplicatearraysafe(&c3_rhs74), "rhs", "rhs",
                  74);
  sf_mex_addfield(*c3_info, sf_mex_duplicatearraysafe(&c3_lhs74), "lhs", "lhs",
                  74);
  sf_mex_addfield(*c3_info, c3_emlrt_marshallOut(
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/ops/colon.m!checkrange"),
                  "context", "context", 75);
  sf_mex_addfield(*c3_info, c3_emlrt_marshallOut("intmax"), "name", "name", 75);
  sf_mex_addfield(*c3_info, c3_emlrt_marshallOut("char"), "dominantType",
                  "dominantType", 75);
  sf_mex_addfield(*c3_info, c3_emlrt_marshallOut(
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/elmat/intmax.m"), "resolved",
                  "resolved", 75);
  sf_mex_addfield(*c3_info, c3_b_emlrt_marshallOut(1362287082U), "fileTimeLo",
                  "fileTimeLo", 75);
  sf_mex_addfield(*c3_info, c3_b_emlrt_marshallOut(0U), "fileTimeHi",
                  "fileTimeHi", 75);
  sf_mex_addfield(*c3_info, c3_b_emlrt_marshallOut(0U), "mFileTimeLo",
                  "mFileTimeLo", 75);
  sf_mex_addfield(*c3_info, c3_b_emlrt_marshallOut(0U), "mFileTimeHi",
                  "mFileTimeHi", 75);
  sf_mex_assign(&c3_rhs75, sf_mex_createcellmatrix(0, 1), false);
  sf_mex_assign(&c3_lhs75, sf_mex_createcellmatrix(0, 1), false);
  sf_mex_addfield(*c3_info, sf_mex_duplicatearraysafe(&c3_rhs75), "rhs", "rhs",
                  75);
  sf_mex_addfield(*c3_info, sf_mex_duplicatearraysafe(&c3_lhs75), "lhs", "lhs",
                  75);
  sf_mex_addfield(*c3_info, c3_emlrt_marshallOut(
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/elmat/intmax.m"), "context",
                  "context", 76);
  sf_mex_addfield(*c3_info, c3_emlrt_marshallOut("eml_switch_helper"), "name",
                  "name", 76);
  sf_mex_addfield(*c3_info, c3_emlrt_marshallOut(""), "dominantType",
                  "dominantType", 76);
  sf_mex_addfield(*c3_info, c3_emlrt_marshallOut(
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/eml/eml_switch_helper.m"),
                  "resolved", "resolved", 76);
  sf_mex_addfield(*c3_info, c3_b_emlrt_marshallOut(1393356058U), "fileTimeLo",
                  "fileTimeLo", 76);
  sf_mex_addfield(*c3_info, c3_b_emlrt_marshallOut(0U), "fileTimeHi",
                  "fileTimeHi", 76);
  sf_mex_addfield(*c3_info, c3_b_emlrt_marshallOut(0U), "mFileTimeLo",
                  "mFileTimeLo", 76);
  sf_mex_addfield(*c3_info, c3_b_emlrt_marshallOut(0U), "mFileTimeHi",
                  "mFileTimeHi", 76);
  sf_mex_assign(&c3_rhs76, sf_mex_createcellmatrix(0, 1), false);
  sf_mex_assign(&c3_lhs76, sf_mex_createcellmatrix(0, 1), false);
  sf_mex_addfield(*c3_info, sf_mex_duplicatearraysafe(&c3_rhs76), "rhs", "rhs",
                  76);
  sf_mex_addfield(*c3_info, sf_mex_duplicatearraysafe(&c3_lhs76), "lhs", "lhs",
                  76);
  sf_mex_addfield(*c3_info, c3_emlrt_marshallOut(
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/ops/colon.m!eml_integer_colon_dispatcher"),
                  "context", "context", 77);
  sf_mex_addfield(*c3_info, c3_emlrt_marshallOut("intmin"), "name", "name", 77);
  sf_mex_addfield(*c3_info, c3_emlrt_marshallOut("char"), "dominantType",
                  "dominantType", 77);
  sf_mex_addfield(*c3_info, c3_emlrt_marshallOut(
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/elmat/intmin.m"), "resolved",
                  "resolved", 77);
  sf_mex_addfield(*c3_info, c3_b_emlrt_marshallOut(1362287082U), "fileTimeLo",
                  "fileTimeLo", 77);
  sf_mex_addfield(*c3_info, c3_b_emlrt_marshallOut(0U), "fileTimeHi",
                  "fileTimeHi", 77);
  sf_mex_addfield(*c3_info, c3_b_emlrt_marshallOut(0U), "mFileTimeLo",
                  "mFileTimeLo", 77);
  sf_mex_addfield(*c3_info, c3_b_emlrt_marshallOut(0U), "mFileTimeHi",
                  "mFileTimeHi", 77);
  sf_mex_assign(&c3_rhs77, sf_mex_createcellmatrix(0, 1), false);
  sf_mex_assign(&c3_lhs77, sf_mex_createcellmatrix(0, 1), false);
  sf_mex_addfield(*c3_info, sf_mex_duplicatearraysafe(&c3_rhs77), "rhs", "rhs",
                  77);
  sf_mex_addfield(*c3_info, sf_mex_duplicatearraysafe(&c3_lhs77), "lhs", "lhs",
                  77);
  sf_mex_addfield(*c3_info, c3_emlrt_marshallOut(
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/ops/colon.m!eml_integer_colon_dispatcher"),
                  "context", "context", 78);
  sf_mex_addfield(*c3_info, c3_emlrt_marshallOut("intmax"), "name", "name", 78);
  sf_mex_addfield(*c3_info, c3_emlrt_marshallOut("char"), "dominantType",
                  "dominantType", 78);
  sf_mex_addfield(*c3_info, c3_emlrt_marshallOut(
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/elmat/intmax.m"), "resolved",
                  "resolved", 78);
  sf_mex_addfield(*c3_info, c3_b_emlrt_marshallOut(1362287082U), "fileTimeLo",
                  "fileTimeLo", 78);
  sf_mex_addfield(*c3_info, c3_b_emlrt_marshallOut(0U), "fileTimeHi",
                  "fileTimeHi", 78);
  sf_mex_addfield(*c3_info, c3_b_emlrt_marshallOut(0U), "mFileTimeLo",
                  "mFileTimeLo", 78);
  sf_mex_addfield(*c3_info, c3_b_emlrt_marshallOut(0U), "mFileTimeHi",
                  "mFileTimeHi", 78);
  sf_mex_assign(&c3_rhs78, sf_mex_createcellmatrix(0, 1), false);
  sf_mex_assign(&c3_lhs78, sf_mex_createcellmatrix(0, 1), false);
  sf_mex_addfield(*c3_info, sf_mex_duplicatearraysafe(&c3_rhs78), "rhs", "rhs",
                  78);
  sf_mex_addfield(*c3_info, sf_mex_duplicatearraysafe(&c3_lhs78), "lhs", "lhs",
                  78);
  sf_mex_addfield(*c3_info, c3_emlrt_marshallOut(
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/ops/colon.m!eml_integer_colon_dispatcher"),
                  "context", "context", 79);
  sf_mex_addfield(*c3_info, c3_emlrt_marshallOut("eml_isa_uint"), "name", "name",
                  79);
  sf_mex_addfield(*c3_info, c3_emlrt_marshallOut("coder.internal.indexInt"),
                  "dominantType", "dominantType", 79);
  sf_mex_addfield(*c3_info, c3_emlrt_marshallOut(
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/eml/eml_isa_uint.m"), "resolved",
                  "resolved", 79);
  sf_mex_addfield(*c3_info, c3_b_emlrt_marshallOut(1376005888U), "fileTimeLo",
                  "fileTimeLo", 79);
  sf_mex_addfield(*c3_info, c3_b_emlrt_marshallOut(0U), "fileTimeHi",
                  "fileTimeHi", 79);
  sf_mex_addfield(*c3_info, c3_b_emlrt_marshallOut(0U), "mFileTimeLo",
                  "mFileTimeLo", 79);
  sf_mex_addfield(*c3_info, c3_b_emlrt_marshallOut(0U), "mFileTimeHi",
                  "mFileTimeHi", 79);
  sf_mex_assign(&c3_rhs79, sf_mex_createcellmatrix(0, 1), false);
  sf_mex_assign(&c3_lhs79, sf_mex_createcellmatrix(0, 1), false);
  sf_mex_addfield(*c3_info, sf_mex_duplicatearraysafe(&c3_rhs79), "rhs", "rhs",
                  79);
  sf_mex_addfield(*c3_info, sf_mex_duplicatearraysafe(&c3_lhs79), "lhs", "lhs",
                  79);
  sf_mex_addfield(*c3_info, c3_emlrt_marshallOut(
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/eml/eml_isa_uint.m"), "context",
                  "context", 80);
  sf_mex_addfield(*c3_info, c3_emlrt_marshallOut("coder.internal.isaUint"),
                  "name", "name", 80);
  sf_mex_addfield(*c3_info, c3_emlrt_marshallOut("coder.internal.indexInt"),
                  "dominantType", "dominantType", 80);
  sf_mex_addfield(*c3_info, c3_emlrt_marshallOut(
    "[ILXE]$matlabroot$/toolbox/coder/coder/+coder/+internal/isaUint.p"),
                  "resolved", "resolved", 80);
  sf_mex_addfield(*c3_info, c3_b_emlrt_marshallOut(1410832970U), "fileTimeLo",
                  "fileTimeLo", 80);
  sf_mex_addfield(*c3_info, c3_b_emlrt_marshallOut(0U), "fileTimeHi",
                  "fileTimeHi", 80);
  sf_mex_addfield(*c3_info, c3_b_emlrt_marshallOut(0U), "mFileTimeLo",
                  "mFileTimeLo", 80);
  sf_mex_addfield(*c3_info, c3_b_emlrt_marshallOut(0U), "mFileTimeHi",
                  "mFileTimeHi", 80);
  sf_mex_assign(&c3_rhs80, sf_mex_createcellmatrix(0, 1), false);
  sf_mex_assign(&c3_lhs80, sf_mex_createcellmatrix(0, 1), false);
  sf_mex_addfield(*c3_info, sf_mex_duplicatearraysafe(&c3_rhs80), "rhs", "rhs",
                  80);
  sf_mex_addfield(*c3_info, sf_mex_duplicatearraysafe(&c3_lhs80), "lhs", "lhs",
                  80);
  sf_mex_addfield(*c3_info, c3_emlrt_marshallOut(
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/ops/colon.m!integer_colon_length_nonnegd"),
                  "context", "context", 81);
  sf_mex_addfield(*c3_info, c3_emlrt_marshallOut("eml_unsigned_class"), "name",
                  "name", 81);
  sf_mex_addfield(*c3_info, c3_emlrt_marshallOut("char"), "dominantType",
                  "dominantType", 81);
  sf_mex_addfield(*c3_info, c3_emlrt_marshallOut(
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/eml/eml_unsigned_class.m"),
                  "resolved", "resolved", 81);
  sf_mex_addfield(*c3_info, c3_b_emlrt_marshallOut(1376005888U), "fileTimeLo",
                  "fileTimeLo", 81);
  sf_mex_addfield(*c3_info, c3_b_emlrt_marshallOut(0U), "fileTimeHi",
                  "fileTimeHi", 81);
  sf_mex_addfield(*c3_info, c3_b_emlrt_marshallOut(0U), "mFileTimeLo",
                  "mFileTimeLo", 81);
  sf_mex_addfield(*c3_info, c3_b_emlrt_marshallOut(0U), "mFileTimeHi",
                  "mFileTimeHi", 81);
  sf_mex_assign(&c3_rhs81, sf_mex_createcellmatrix(0, 1), false);
  sf_mex_assign(&c3_lhs81, sf_mex_createcellmatrix(0, 1), false);
  sf_mex_addfield(*c3_info, sf_mex_duplicatearraysafe(&c3_rhs81), "rhs", "rhs",
                  81);
  sf_mex_addfield(*c3_info, sf_mex_duplicatearraysafe(&c3_lhs81), "lhs", "lhs",
                  81);
  sf_mex_addfield(*c3_info, c3_emlrt_marshallOut(
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/eml/eml_unsigned_class.m"),
                  "context", "context", 82);
  sf_mex_addfield(*c3_info, c3_emlrt_marshallOut("coder.internal.unsignedClass"),
                  "name", "name", 82);
  sf_mex_addfield(*c3_info, c3_emlrt_marshallOut("char"), "dominantType",
                  "dominantType", 82);
  sf_mex_addfield(*c3_info, c3_emlrt_marshallOut(
    "[ILXE]$matlabroot$/toolbox/coder/coder/+coder/+internal/unsignedClass.p"),
                  "resolved", "resolved", 82);
  sf_mex_addfield(*c3_info, c3_b_emlrt_marshallOut(1410832970U), "fileTimeLo",
                  "fileTimeLo", 82);
  sf_mex_addfield(*c3_info, c3_b_emlrt_marshallOut(0U), "fileTimeHi",
                  "fileTimeHi", 82);
  sf_mex_addfield(*c3_info, c3_b_emlrt_marshallOut(0U), "mFileTimeLo",
                  "mFileTimeLo", 82);
  sf_mex_addfield(*c3_info, c3_b_emlrt_marshallOut(0U), "mFileTimeHi",
                  "mFileTimeHi", 82);
  sf_mex_assign(&c3_rhs82, sf_mex_createcellmatrix(0, 1), false);
  sf_mex_assign(&c3_lhs82, sf_mex_createcellmatrix(0, 1), false);
  sf_mex_addfield(*c3_info, sf_mex_duplicatearraysafe(&c3_rhs82), "rhs", "rhs",
                  82);
  sf_mex_addfield(*c3_info, sf_mex_duplicatearraysafe(&c3_lhs82), "lhs", "lhs",
                  82);
  sf_mex_addfield(*c3_info, c3_emlrt_marshallOut(
    "[ILXE]$matlabroot$/toolbox/coder/coder/+coder/+internal/unsignedClass.p"),
                  "context", "context", 83);
  sf_mex_addfield(*c3_info, c3_emlrt_marshallOut("eml_switch_helper"), "name",
                  "name", 83);
  sf_mex_addfield(*c3_info, c3_emlrt_marshallOut(""), "dominantType",
                  "dominantType", 83);
  sf_mex_addfield(*c3_info, c3_emlrt_marshallOut(
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/eml/eml_switch_helper.m"),
                  "resolved", "resolved", 83);
  sf_mex_addfield(*c3_info, c3_b_emlrt_marshallOut(1393356058U), "fileTimeLo",
                  "fileTimeLo", 83);
  sf_mex_addfield(*c3_info, c3_b_emlrt_marshallOut(0U), "fileTimeHi",
                  "fileTimeHi", 83);
  sf_mex_addfield(*c3_info, c3_b_emlrt_marshallOut(0U), "mFileTimeLo",
                  "mFileTimeLo", 83);
  sf_mex_addfield(*c3_info, c3_b_emlrt_marshallOut(0U), "mFileTimeHi",
                  "mFileTimeHi", 83);
  sf_mex_assign(&c3_rhs83, sf_mex_createcellmatrix(0, 1), false);
  sf_mex_assign(&c3_lhs83, sf_mex_createcellmatrix(0, 1), false);
  sf_mex_addfield(*c3_info, sf_mex_duplicatearraysafe(&c3_rhs83), "rhs", "rhs",
                  83);
  sf_mex_addfield(*c3_info, sf_mex_duplicatearraysafe(&c3_lhs83), "lhs", "lhs",
                  83);
  sf_mex_addfield(*c3_info, c3_emlrt_marshallOut(
    "[ILXE]$matlabroot$/toolbox/coder/coder/+coder/+internal/unsignedClass.p"),
                  "context", "context", 84);
  sf_mex_addfield(*c3_info, c3_emlrt_marshallOut("eml_index_class"), "name",
                  "name", 84);
  sf_mex_addfield(*c3_info, c3_emlrt_marshallOut(""), "dominantType",
                  "dominantType", 84);
  sf_mex_addfield(*c3_info, c3_emlrt_marshallOut(
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/eml/eml_index_class.m"),
                  "resolved", "resolved", 84);
  sf_mex_addfield(*c3_info, c3_b_emlrt_marshallOut(1323195778U), "fileTimeLo",
                  "fileTimeLo", 84);
  sf_mex_addfield(*c3_info, c3_b_emlrt_marshallOut(0U), "fileTimeHi",
                  "fileTimeHi", 84);
  sf_mex_addfield(*c3_info, c3_b_emlrt_marshallOut(0U), "mFileTimeLo",
                  "mFileTimeLo", 84);
  sf_mex_addfield(*c3_info, c3_b_emlrt_marshallOut(0U), "mFileTimeHi",
                  "mFileTimeHi", 84);
  sf_mex_assign(&c3_rhs84, sf_mex_createcellmatrix(0, 1), false);
  sf_mex_assign(&c3_lhs84, sf_mex_createcellmatrix(0, 1), false);
  sf_mex_addfield(*c3_info, sf_mex_duplicatearraysafe(&c3_rhs84), "rhs", "rhs",
                  84);
  sf_mex_addfield(*c3_info, sf_mex_duplicatearraysafe(&c3_lhs84), "lhs", "lhs",
                  84);
  sf_mex_addfield(*c3_info, c3_emlrt_marshallOut(
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/ops/colon.m!integer_colon_length_nonnegd"),
                  "context", "context", 85);
  sf_mex_addfield(*c3_info, c3_emlrt_marshallOut("eml_index_class"), "name",
                  "name", 85);
  sf_mex_addfield(*c3_info, c3_emlrt_marshallOut(""), "dominantType",
                  "dominantType", 85);
  sf_mex_addfield(*c3_info, c3_emlrt_marshallOut(
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/eml/eml_index_class.m"),
                  "resolved", "resolved", 85);
  sf_mex_addfield(*c3_info, c3_b_emlrt_marshallOut(1323195778U), "fileTimeLo",
                  "fileTimeLo", 85);
  sf_mex_addfield(*c3_info, c3_b_emlrt_marshallOut(0U), "fileTimeHi",
                  "fileTimeHi", 85);
  sf_mex_addfield(*c3_info, c3_b_emlrt_marshallOut(0U), "mFileTimeLo",
                  "mFileTimeLo", 85);
  sf_mex_addfield(*c3_info, c3_b_emlrt_marshallOut(0U), "mFileTimeHi",
                  "mFileTimeHi", 85);
  sf_mex_assign(&c3_rhs85, sf_mex_createcellmatrix(0, 1), false);
  sf_mex_assign(&c3_lhs85, sf_mex_createcellmatrix(0, 1), false);
  sf_mex_addfield(*c3_info, sf_mex_duplicatearraysafe(&c3_rhs85), "rhs", "rhs",
                  85);
  sf_mex_addfield(*c3_info, sf_mex_duplicatearraysafe(&c3_lhs85), "lhs", "lhs",
                  85);
  sf_mex_addfield(*c3_info, c3_emlrt_marshallOut(
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/ops/colon.m!integer_colon_length_nonnegd"),
                  "context", "context", 86);
  sf_mex_addfield(*c3_info, c3_emlrt_marshallOut("intmax"), "name", "name", 86);
  sf_mex_addfield(*c3_info, c3_emlrt_marshallOut("char"), "dominantType",
                  "dominantType", 86);
  sf_mex_addfield(*c3_info, c3_emlrt_marshallOut(
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/elmat/intmax.m"), "resolved",
                  "resolved", 86);
  sf_mex_addfield(*c3_info, c3_b_emlrt_marshallOut(1362287082U), "fileTimeLo",
                  "fileTimeLo", 86);
  sf_mex_addfield(*c3_info, c3_b_emlrt_marshallOut(0U), "fileTimeHi",
                  "fileTimeHi", 86);
  sf_mex_addfield(*c3_info, c3_b_emlrt_marshallOut(0U), "mFileTimeLo",
                  "mFileTimeLo", 86);
  sf_mex_addfield(*c3_info, c3_b_emlrt_marshallOut(0U), "mFileTimeHi",
                  "mFileTimeHi", 86);
  sf_mex_assign(&c3_rhs86, sf_mex_createcellmatrix(0, 1), false);
  sf_mex_assign(&c3_lhs86, sf_mex_createcellmatrix(0, 1), false);
  sf_mex_addfield(*c3_info, sf_mex_duplicatearraysafe(&c3_rhs86), "rhs", "rhs",
                  86);
  sf_mex_addfield(*c3_info, sf_mex_duplicatearraysafe(&c3_lhs86), "lhs", "lhs",
                  86);
  sf_mex_addfield(*c3_info, c3_emlrt_marshallOut(
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/ops/colon.m!integer_colon_length_nonnegd"),
                  "context", "context", 87);
  sf_mex_addfield(*c3_info, c3_emlrt_marshallOut("eml_isa_uint"), "name", "name",
                  87);
  sf_mex_addfield(*c3_info, c3_emlrt_marshallOut("coder.internal.indexInt"),
                  "dominantType", "dominantType", 87);
  sf_mex_addfield(*c3_info, c3_emlrt_marshallOut(
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/eml/eml_isa_uint.m"), "resolved",
                  "resolved", 87);
  sf_mex_addfield(*c3_info, c3_b_emlrt_marshallOut(1376005888U), "fileTimeLo",
                  "fileTimeLo", 87);
  sf_mex_addfield(*c3_info, c3_b_emlrt_marshallOut(0U), "fileTimeHi",
                  "fileTimeHi", 87);
  sf_mex_addfield(*c3_info, c3_b_emlrt_marshallOut(0U), "mFileTimeLo",
                  "mFileTimeLo", 87);
  sf_mex_addfield(*c3_info, c3_b_emlrt_marshallOut(0U), "mFileTimeHi",
                  "mFileTimeHi", 87);
  sf_mex_assign(&c3_rhs87, sf_mex_createcellmatrix(0, 1), false);
  sf_mex_assign(&c3_lhs87, sf_mex_createcellmatrix(0, 1), false);
  sf_mex_addfield(*c3_info, sf_mex_duplicatearraysafe(&c3_rhs87), "rhs", "rhs",
                  87);
  sf_mex_addfield(*c3_info, sf_mex_duplicatearraysafe(&c3_lhs87), "lhs", "lhs",
                  87);
  sf_mex_addfield(*c3_info, c3_emlrt_marshallOut(
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/ops/colon.m!integer_colon_length_nonnegd"),
                  "context", "context", 88);
  sf_mex_addfield(*c3_info, c3_emlrt_marshallOut("eml_index_plus"), "name",
                  "name", 88);
  sf_mex_addfield(*c3_info, c3_emlrt_marshallOut("double"), "dominantType",
                  "dominantType", 88);
  sf_mex_addfield(*c3_info, c3_emlrt_marshallOut(
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/eml/eml_index_plus.m"),
                  "resolved", "resolved", 88);
  sf_mex_addfield(*c3_info, c3_b_emlrt_marshallOut(1372607616U), "fileTimeLo",
                  "fileTimeLo", 88);
  sf_mex_addfield(*c3_info, c3_b_emlrt_marshallOut(0U), "fileTimeHi",
                  "fileTimeHi", 88);
  sf_mex_addfield(*c3_info, c3_b_emlrt_marshallOut(0U), "mFileTimeLo",
                  "mFileTimeLo", 88);
  sf_mex_addfield(*c3_info, c3_b_emlrt_marshallOut(0U), "mFileTimeHi",
                  "mFileTimeHi", 88);
  sf_mex_assign(&c3_rhs88, sf_mex_createcellmatrix(0, 1), false);
  sf_mex_assign(&c3_lhs88, sf_mex_createcellmatrix(0, 1), false);
  sf_mex_addfield(*c3_info, sf_mex_duplicatearraysafe(&c3_rhs88), "rhs", "rhs",
                  88);
  sf_mex_addfield(*c3_info, sf_mex_duplicatearraysafe(&c3_lhs88), "lhs", "lhs",
                  88);
  sf_mex_addfield(*c3_info, c3_emlrt_marshallOut(
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/ops/colon.m!eml_signed_integer_colon"),
                  "context", "context", 89);
  sf_mex_addfield(*c3_info, c3_emlrt_marshallOut(
    "eml_int_forloop_overflow_check"), "name", "name", 89);
  sf_mex_addfield(*c3_info, c3_emlrt_marshallOut(""), "dominantType",
                  "dominantType", 89);
  sf_mex_addfield(*c3_info, c3_emlrt_marshallOut(
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/eml/eml_int_forloop_overflow_check.m"),
                  "resolved", "resolved", 89);
  sf_mex_addfield(*c3_info, c3_b_emlrt_marshallOut(1397282622U), "fileTimeLo",
                  "fileTimeLo", 89);
  sf_mex_addfield(*c3_info, c3_b_emlrt_marshallOut(0U), "fileTimeHi",
                  "fileTimeHi", 89);
  sf_mex_addfield(*c3_info, c3_b_emlrt_marshallOut(0U), "mFileTimeLo",
                  "mFileTimeLo", 89);
  sf_mex_addfield(*c3_info, c3_b_emlrt_marshallOut(0U), "mFileTimeHi",
                  "mFileTimeHi", 89);
  sf_mex_assign(&c3_rhs89, sf_mex_createcellmatrix(0, 1), false);
  sf_mex_assign(&c3_lhs89, sf_mex_createcellmatrix(0, 1), false);
  sf_mex_addfield(*c3_info, sf_mex_duplicatearraysafe(&c3_rhs89), "rhs", "rhs",
                  89);
  sf_mex_addfield(*c3_info, sf_mex_duplicatearraysafe(&c3_lhs89), "lhs", "lhs",
                  89);
  sf_mex_addfield(*c3_info, c3_emlrt_marshallOut(
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/eml/eml_int_forloop_overflow_check.m!eml_int_forloop_overflow_check_helper"),
                  "context", "context", 90);
  sf_mex_addfield(*c3_info, c3_emlrt_marshallOut("isfi"), "name", "name", 90);
  sf_mex_addfield(*c3_info, c3_emlrt_marshallOut("coder.internal.indexInt"),
                  "dominantType", "dominantType", 90);
  sf_mex_addfield(*c3_info, c3_emlrt_marshallOut(
    "[ILXE]$matlabroot$/toolbox/eml/lib/fixedpoint/isfi.m"), "resolved",
                  "resolved", 90);
  sf_mex_addfield(*c3_info, c3_b_emlrt_marshallOut(1346535558U), "fileTimeLo",
                  "fileTimeLo", 90);
  sf_mex_addfield(*c3_info, c3_b_emlrt_marshallOut(0U), "fileTimeHi",
                  "fileTimeHi", 90);
  sf_mex_addfield(*c3_info, c3_b_emlrt_marshallOut(0U), "mFileTimeLo",
                  "mFileTimeLo", 90);
  sf_mex_addfield(*c3_info, c3_b_emlrt_marshallOut(0U), "mFileTimeHi",
                  "mFileTimeHi", 90);
  sf_mex_assign(&c3_rhs90, sf_mex_createcellmatrix(0, 1), false);
  sf_mex_assign(&c3_lhs90, sf_mex_createcellmatrix(0, 1), false);
  sf_mex_addfield(*c3_info, sf_mex_duplicatearraysafe(&c3_rhs90), "rhs", "rhs",
                  90);
  sf_mex_addfield(*c3_info, sf_mex_duplicatearraysafe(&c3_lhs90), "lhs", "lhs",
                  90);
  sf_mex_addfield(*c3_info, c3_emlrt_marshallOut(
    "[ILXE]$matlabroot$/toolbox/eml/lib/fixedpoint/isfi.m"), "context",
                  "context", 91);
  sf_mex_addfield(*c3_info, c3_emlrt_marshallOut("isnumerictype"), "name",
                  "name", 91);
  sf_mex_addfield(*c3_info, c3_emlrt_marshallOut("char"), "dominantType",
                  "dominantType", 91);
  sf_mex_addfield(*c3_info, c3_emlrt_marshallOut(
    "[ILXE]$matlabroot$/toolbox/eml/lib/fixedpoint/isnumerictype.m"), "resolved",
                  "resolved", 91);
  sf_mex_addfield(*c3_info, c3_b_emlrt_marshallOut(1398900798U), "fileTimeLo",
                  "fileTimeLo", 91);
  sf_mex_addfield(*c3_info, c3_b_emlrt_marshallOut(0U), "fileTimeHi",
                  "fileTimeHi", 91);
  sf_mex_addfield(*c3_info, c3_b_emlrt_marshallOut(0U), "mFileTimeLo",
                  "mFileTimeLo", 91);
  sf_mex_addfield(*c3_info, c3_b_emlrt_marshallOut(0U), "mFileTimeHi",
                  "mFileTimeHi", 91);
  sf_mex_assign(&c3_rhs91, sf_mex_createcellmatrix(0, 1), false);
  sf_mex_assign(&c3_lhs91, sf_mex_createcellmatrix(0, 1), false);
  sf_mex_addfield(*c3_info, sf_mex_duplicatearraysafe(&c3_rhs91), "rhs", "rhs",
                  91);
  sf_mex_addfield(*c3_info, sf_mex_duplicatearraysafe(&c3_lhs91), "lhs", "lhs",
                  91);
  sf_mex_addfield(*c3_info, c3_emlrt_marshallOut(
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/eml/eml_int_forloop_overflow_check.m!eml_int_forloop_overflow_check_helper"),
                  "context", "context", 92);
  sf_mex_addfield(*c3_info, c3_emlrt_marshallOut("intmax"), "name", "name", 92);
  sf_mex_addfield(*c3_info, c3_emlrt_marshallOut("char"), "dominantType",
                  "dominantType", 92);
  sf_mex_addfield(*c3_info, c3_emlrt_marshallOut(
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/elmat/intmax.m"), "resolved",
                  "resolved", 92);
  sf_mex_addfield(*c3_info, c3_b_emlrt_marshallOut(1362287082U), "fileTimeLo",
                  "fileTimeLo", 92);
  sf_mex_addfield(*c3_info, c3_b_emlrt_marshallOut(0U), "fileTimeHi",
                  "fileTimeHi", 92);
  sf_mex_addfield(*c3_info, c3_b_emlrt_marshallOut(0U), "mFileTimeLo",
                  "mFileTimeLo", 92);
  sf_mex_addfield(*c3_info, c3_b_emlrt_marshallOut(0U), "mFileTimeHi",
                  "mFileTimeHi", 92);
  sf_mex_assign(&c3_rhs92, sf_mex_createcellmatrix(0, 1), false);
  sf_mex_assign(&c3_lhs92, sf_mex_createcellmatrix(0, 1), false);
  sf_mex_addfield(*c3_info, sf_mex_duplicatearraysafe(&c3_rhs92), "rhs", "rhs",
                  92);
  sf_mex_addfield(*c3_info, sf_mex_duplicatearraysafe(&c3_lhs92), "lhs", "lhs",
                  92);
  sf_mex_addfield(*c3_info, c3_emlrt_marshallOut(
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/eml/eml_int_forloop_overflow_check.m!eml_int_forloop_overflow_check_helper"),
                  "context", "context", 93);
  sf_mex_addfield(*c3_info, c3_emlrt_marshallOut("intmin"), "name", "name", 93);
  sf_mex_addfield(*c3_info, c3_emlrt_marshallOut("char"), "dominantType",
                  "dominantType", 93);
  sf_mex_addfield(*c3_info, c3_emlrt_marshallOut(
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/elmat/intmin.m"), "resolved",
                  "resolved", 93);
  sf_mex_addfield(*c3_info, c3_b_emlrt_marshallOut(1362287082U), "fileTimeLo",
                  "fileTimeLo", 93);
  sf_mex_addfield(*c3_info, c3_b_emlrt_marshallOut(0U), "fileTimeHi",
                  "fileTimeHi", 93);
  sf_mex_addfield(*c3_info, c3_b_emlrt_marshallOut(0U), "mFileTimeLo",
                  "mFileTimeLo", 93);
  sf_mex_addfield(*c3_info, c3_b_emlrt_marshallOut(0U), "mFileTimeHi",
                  "mFileTimeHi", 93);
  sf_mex_assign(&c3_rhs93, sf_mex_createcellmatrix(0, 1), false);
  sf_mex_assign(&c3_lhs93, sf_mex_createcellmatrix(0, 1), false);
  sf_mex_addfield(*c3_info, sf_mex_duplicatearraysafe(&c3_rhs93), "rhs", "rhs",
                  93);
  sf_mex_addfield(*c3_info, sf_mex_duplicatearraysafe(&c3_lhs93), "lhs", "lhs",
                  93);
  sf_mex_addfield(*c3_info, c3_emlrt_marshallOut(
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/eml/lapack/matlab/eml_matlab_zgetrf.m"),
                  "context", "context", 94);
  sf_mex_addfield(*c3_info, c3_emlrt_marshallOut("eml_index_class"), "name",
                  "name", 94);
  sf_mex_addfield(*c3_info, c3_emlrt_marshallOut(""), "dominantType",
                  "dominantType", 94);
  sf_mex_addfield(*c3_info, c3_emlrt_marshallOut(
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/eml/eml_index_class.m"),
                  "resolved", "resolved", 94);
  sf_mex_addfield(*c3_info, c3_b_emlrt_marshallOut(1323195778U), "fileTimeLo",
                  "fileTimeLo", 94);
  sf_mex_addfield(*c3_info, c3_b_emlrt_marshallOut(0U), "fileTimeHi",
                  "fileTimeHi", 94);
  sf_mex_addfield(*c3_info, c3_b_emlrt_marshallOut(0U), "mFileTimeLo",
                  "mFileTimeLo", 94);
  sf_mex_addfield(*c3_info, c3_b_emlrt_marshallOut(0U), "mFileTimeHi",
                  "mFileTimeHi", 94);
  sf_mex_assign(&c3_rhs94, sf_mex_createcellmatrix(0, 1), false);
  sf_mex_assign(&c3_lhs94, sf_mex_createcellmatrix(0, 1), false);
  sf_mex_addfield(*c3_info, sf_mex_duplicatearraysafe(&c3_rhs94), "rhs", "rhs",
                  94);
  sf_mex_addfield(*c3_info, sf_mex_duplicatearraysafe(&c3_lhs94), "lhs", "lhs",
                  94);
  sf_mex_addfield(*c3_info, c3_emlrt_marshallOut(
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/eml/lapack/matlab/eml_matlab_zgetrf.m"),
                  "context", "context", 95);
  sf_mex_addfield(*c3_info, c3_emlrt_marshallOut("eml_index_plus"), "name",
                  "name", 95);
  sf_mex_addfield(*c3_info, c3_emlrt_marshallOut("double"), "dominantType",
                  "dominantType", 95);
  sf_mex_addfield(*c3_info, c3_emlrt_marshallOut(
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/eml/eml_index_plus.m"),
                  "resolved", "resolved", 95);
  sf_mex_addfield(*c3_info, c3_b_emlrt_marshallOut(1372607616U), "fileTimeLo",
                  "fileTimeLo", 95);
  sf_mex_addfield(*c3_info, c3_b_emlrt_marshallOut(0U), "fileTimeHi",
                  "fileTimeHi", 95);
  sf_mex_addfield(*c3_info, c3_b_emlrt_marshallOut(0U), "mFileTimeLo",
                  "mFileTimeLo", 95);
  sf_mex_addfield(*c3_info, c3_b_emlrt_marshallOut(0U), "mFileTimeHi",
                  "mFileTimeHi", 95);
  sf_mex_assign(&c3_rhs95, sf_mex_createcellmatrix(0, 1), false);
  sf_mex_assign(&c3_lhs95, sf_mex_createcellmatrix(0, 1), false);
  sf_mex_addfield(*c3_info, sf_mex_duplicatearraysafe(&c3_rhs95), "rhs", "rhs",
                  95);
  sf_mex_addfield(*c3_info, sf_mex_duplicatearraysafe(&c3_lhs95), "lhs", "lhs",
                  95);
  sf_mex_addfield(*c3_info, c3_emlrt_marshallOut(
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/eml/lapack/matlab/eml_matlab_zgetrf.m"),
                  "context", "context", 96);
  sf_mex_addfield(*c3_info, c3_emlrt_marshallOut(
    "eml_int_forloop_overflow_check"), "name", "name", 96);
  sf_mex_addfield(*c3_info, c3_emlrt_marshallOut(""), "dominantType",
                  "dominantType", 96);
  sf_mex_addfield(*c3_info, c3_emlrt_marshallOut(
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/eml/eml_int_forloop_overflow_check.m"),
                  "resolved", "resolved", 96);
  sf_mex_addfield(*c3_info, c3_b_emlrt_marshallOut(1397282622U), "fileTimeLo",
                  "fileTimeLo", 96);
  sf_mex_addfield(*c3_info, c3_b_emlrt_marshallOut(0U), "fileTimeHi",
                  "fileTimeHi", 96);
  sf_mex_addfield(*c3_info, c3_b_emlrt_marshallOut(0U), "mFileTimeLo",
                  "mFileTimeLo", 96);
  sf_mex_addfield(*c3_info, c3_b_emlrt_marshallOut(0U), "mFileTimeHi",
                  "mFileTimeHi", 96);
  sf_mex_assign(&c3_rhs96, sf_mex_createcellmatrix(0, 1), false);
  sf_mex_assign(&c3_lhs96, sf_mex_createcellmatrix(0, 1), false);
  sf_mex_addfield(*c3_info, sf_mex_duplicatearraysafe(&c3_rhs96), "rhs", "rhs",
                  96);
  sf_mex_addfield(*c3_info, sf_mex_duplicatearraysafe(&c3_lhs96), "lhs", "lhs",
                  96);
  sf_mex_addfield(*c3_info, c3_emlrt_marshallOut(
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/eml/lapack/matlab/eml_matlab_zgetrf.m"),
                  "context", "context", 97);
  sf_mex_addfield(*c3_info, c3_emlrt_marshallOut("eml_index_minus"), "name",
                  "name", 97);
  sf_mex_addfield(*c3_info, c3_emlrt_marshallOut("double"), "dominantType",
                  "dominantType", 97);
  sf_mex_addfield(*c3_info, c3_emlrt_marshallOut(
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/eml/eml_index_minus.m"),
                  "resolved", "resolved", 97);
  sf_mex_addfield(*c3_info, c3_b_emlrt_marshallOut(1372607616U), "fileTimeLo",
                  "fileTimeLo", 97);
  sf_mex_addfield(*c3_info, c3_b_emlrt_marshallOut(0U), "fileTimeHi",
                  "fileTimeHi", 97);
  sf_mex_addfield(*c3_info, c3_b_emlrt_marshallOut(0U), "mFileTimeLo",
                  "mFileTimeLo", 97);
  sf_mex_addfield(*c3_info, c3_b_emlrt_marshallOut(0U), "mFileTimeHi",
                  "mFileTimeHi", 97);
  sf_mex_assign(&c3_rhs97, sf_mex_createcellmatrix(0, 1), false);
  sf_mex_assign(&c3_lhs97, sf_mex_createcellmatrix(0, 1), false);
  sf_mex_addfield(*c3_info, sf_mex_duplicatearraysafe(&c3_rhs97), "rhs", "rhs",
                  97);
  sf_mex_addfield(*c3_info, sf_mex_duplicatearraysafe(&c3_lhs97), "lhs", "lhs",
                  97);
  sf_mex_addfield(*c3_info, c3_emlrt_marshallOut(
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/eml/eml_index_minus.m"),
                  "context", "context", 98);
  sf_mex_addfield(*c3_info, c3_emlrt_marshallOut("coder.internal.indexMinus"),
                  "name", "name", 98);
  sf_mex_addfield(*c3_info, c3_emlrt_marshallOut("double"), "dominantType",
                  "dominantType", 98);
  sf_mex_addfield(*c3_info, c3_emlrt_marshallOut(
    "[ILXE]$matlabroot$/toolbox/shared/coder/coder/+coder/+internal/indexMinus.m"),
                  "resolved", "resolved", 98);
  sf_mex_addfield(*c3_info, c3_b_emlrt_marshallOut(1372608360U), "fileTimeLo",
                  "fileTimeLo", 98);
  sf_mex_addfield(*c3_info, c3_b_emlrt_marshallOut(0U), "fileTimeHi",
                  "fileTimeHi", 98);
  sf_mex_addfield(*c3_info, c3_b_emlrt_marshallOut(0U), "mFileTimeLo",
                  "mFileTimeLo", 98);
  sf_mex_addfield(*c3_info, c3_b_emlrt_marshallOut(0U), "mFileTimeHi",
                  "mFileTimeHi", 98);
  sf_mex_assign(&c3_rhs98, sf_mex_createcellmatrix(0, 1), false);
  sf_mex_assign(&c3_lhs98, sf_mex_createcellmatrix(0, 1), false);
  sf_mex_addfield(*c3_info, sf_mex_duplicatearraysafe(&c3_rhs98), "rhs", "rhs",
                  98);
  sf_mex_addfield(*c3_info, sf_mex_duplicatearraysafe(&c3_lhs98), "lhs", "lhs",
                  98);
  sf_mex_addfield(*c3_info, c3_emlrt_marshallOut(
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/eml/lapack/matlab/eml_matlab_zgetrf.m"),
                  "context", "context", 99);
  sf_mex_addfield(*c3_info, c3_emlrt_marshallOut("eml_index_minus"), "name",
                  "name", 99);
  sf_mex_addfield(*c3_info, c3_emlrt_marshallOut("coder.internal.indexInt"),
                  "dominantType", "dominantType", 99);
  sf_mex_addfield(*c3_info, c3_emlrt_marshallOut(
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/eml/eml_index_minus.m"),
                  "resolved", "resolved", 99);
  sf_mex_addfield(*c3_info, c3_b_emlrt_marshallOut(1372607616U), "fileTimeLo",
                  "fileTimeLo", 99);
  sf_mex_addfield(*c3_info, c3_b_emlrt_marshallOut(0U), "fileTimeHi",
                  "fileTimeHi", 99);
  sf_mex_addfield(*c3_info, c3_b_emlrt_marshallOut(0U), "mFileTimeLo",
                  "mFileTimeLo", 99);
  sf_mex_addfield(*c3_info, c3_b_emlrt_marshallOut(0U), "mFileTimeHi",
                  "mFileTimeHi", 99);
  sf_mex_assign(&c3_rhs99, sf_mex_createcellmatrix(0, 1), false);
  sf_mex_assign(&c3_lhs99, sf_mex_createcellmatrix(0, 1), false);
  sf_mex_addfield(*c3_info, sf_mex_duplicatearraysafe(&c3_rhs99), "rhs", "rhs",
                  99);
  sf_mex_addfield(*c3_info, sf_mex_duplicatearraysafe(&c3_lhs99), "lhs", "lhs",
                  99);
  sf_mex_addfield(*c3_info, c3_emlrt_marshallOut(
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/eml/eml_index_minus.m"),
                  "context", "context", 100);
  sf_mex_addfield(*c3_info, c3_emlrt_marshallOut("coder.internal.indexMinus"),
                  "name", "name", 100);
  sf_mex_addfield(*c3_info, c3_emlrt_marshallOut("coder.internal.indexInt"),
                  "dominantType", "dominantType", 100);
  sf_mex_addfield(*c3_info, c3_emlrt_marshallOut(
    "[ILXE]$matlabroot$/toolbox/shared/coder/coder/+coder/+internal/indexMinus.m"),
                  "resolved", "resolved", 100);
  sf_mex_addfield(*c3_info, c3_b_emlrt_marshallOut(1372608360U), "fileTimeLo",
                  "fileTimeLo", 100);
  sf_mex_addfield(*c3_info, c3_b_emlrt_marshallOut(0U), "fileTimeHi",
                  "fileTimeHi", 100);
  sf_mex_addfield(*c3_info, c3_b_emlrt_marshallOut(0U), "mFileTimeLo",
                  "mFileTimeLo", 100);
  sf_mex_addfield(*c3_info, c3_b_emlrt_marshallOut(0U), "mFileTimeHi",
                  "mFileTimeHi", 100);
  sf_mex_assign(&c3_rhs100, sf_mex_createcellmatrix(0, 1), false);
  sf_mex_assign(&c3_lhs100, sf_mex_createcellmatrix(0, 1), false);
  sf_mex_addfield(*c3_info, sf_mex_duplicatearraysafe(&c3_rhs100), "rhs", "rhs",
                  100);
  sf_mex_addfield(*c3_info, sf_mex_duplicatearraysafe(&c3_lhs100), "lhs", "lhs",
                  100);
  sf_mex_addfield(*c3_info, c3_emlrt_marshallOut(
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/eml/lapack/matlab/eml_matlab_zgetrf.m"),
                  "context", "context", 101);
  sf_mex_addfield(*c3_info, c3_emlrt_marshallOut("eml_index_times"), "name",
                  "name", 101);
  sf_mex_addfield(*c3_info, c3_emlrt_marshallOut("coder.internal.indexInt"),
                  "dominantType", "dominantType", 101);
  sf_mex_addfield(*c3_info, c3_emlrt_marshallOut(
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/eml/eml_index_times.m"),
                  "resolved", "resolved", 101);
  sf_mex_addfield(*c3_info, c3_b_emlrt_marshallOut(1372607616U), "fileTimeLo",
                  "fileTimeLo", 101);
  sf_mex_addfield(*c3_info, c3_b_emlrt_marshallOut(0U), "fileTimeHi",
                  "fileTimeHi", 101);
  sf_mex_addfield(*c3_info, c3_b_emlrt_marshallOut(0U), "mFileTimeLo",
                  "mFileTimeLo", 101);
  sf_mex_addfield(*c3_info, c3_b_emlrt_marshallOut(0U), "mFileTimeHi",
                  "mFileTimeHi", 101);
  sf_mex_assign(&c3_rhs101, sf_mex_createcellmatrix(0, 1), false);
  sf_mex_assign(&c3_lhs101, sf_mex_createcellmatrix(0, 1), false);
  sf_mex_addfield(*c3_info, sf_mex_duplicatearraysafe(&c3_rhs101), "rhs", "rhs",
                  101);
  sf_mex_addfield(*c3_info, sf_mex_duplicatearraysafe(&c3_lhs101), "lhs", "lhs",
                  101);
  sf_mex_addfield(*c3_info, c3_emlrt_marshallOut(
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/eml/eml_index_times.m"),
                  "context", "context", 102);
  sf_mex_addfield(*c3_info, c3_emlrt_marshallOut("coder.internal.indexTimes"),
                  "name", "name", 102);
  sf_mex_addfield(*c3_info, c3_emlrt_marshallOut("coder.internal.indexInt"),
                  "dominantType", "dominantType", 102);
  sf_mex_addfield(*c3_info, c3_emlrt_marshallOut(
    "[ILXE]$matlabroot$/toolbox/shared/coder/coder/+coder/+internal/indexTimes.m"),
                  "resolved", "resolved", 102);
  sf_mex_addfield(*c3_info, c3_b_emlrt_marshallOut(1372608360U), "fileTimeLo",
                  "fileTimeLo", 102);
  sf_mex_addfield(*c3_info, c3_b_emlrt_marshallOut(0U), "fileTimeHi",
                  "fileTimeHi", 102);
  sf_mex_addfield(*c3_info, c3_b_emlrt_marshallOut(0U), "mFileTimeLo",
                  "mFileTimeLo", 102);
  sf_mex_addfield(*c3_info, c3_b_emlrt_marshallOut(0U), "mFileTimeHi",
                  "mFileTimeHi", 102);
  sf_mex_assign(&c3_rhs102, sf_mex_createcellmatrix(0, 1), false);
  sf_mex_assign(&c3_lhs102, sf_mex_createcellmatrix(0, 1), false);
  sf_mex_addfield(*c3_info, sf_mex_duplicatearraysafe(&c3_rhs102), "rhs", "rhs",
                  102);
  sf_mex_addfield(*c3_info, sf_mex_duplicatearraysafe(&c3_lhs102), "lhs", "lhs",
                  102);
  sf_mex_addfield(*c3_info, c3_emlrt_marshallOut(
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/eml/lapack/matlab/eml_matlab_zgetrf.m"),
                  "context", "context", 103);
  sf_mex_addfield(*c3_info, c3_emlrt_marshallOut("eml_index_plus"), "name",
                  "name", 103);
  sf_mex_addfield(*c3_info, c3_emlrt_marshallOut("coder.internal.indexInt"),
                  "dominantType", "dominantType", 103);
  sf_mex_addfield(*c3_info, c3_emlrt_marshallOut(
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/eml/eml_index_plus.m"),
                  "resolved", "resolved", 103);
  sf_mex_addfield(*c3_info, c3_b_emlrt_marshallOut(1372607616U), "fileTimeLo",
                  "fileTimeLo", 103);
  sf_mex_addfield(*c3_info, c3_b_emlrt_marshallOut(0U), "fileTimeHi",
                  "fileTimeHi", 103);
  sf_mex_addfield(*c3_info, c3_b_emlrt_marshallOut(0U), "mFileTimeLo",
                  "mFileTimeLo", 103);
  sf_mex_addfield(*c3_info, c3_b_emlrt_marshallOut(0U), "mFileTimeHi",
                  "mFileTimeHi", 103);
  sf_mex_assign(&c3_rhs103, sf_mex_createcellmatrix(0, 1), false);
  sf_mex_assign(&c3_lhs103, sf_mex_createcellmatrix(0, 1), false);
  sf_mex_addfield(*c3_info, sf_mex_duplicatearraysafe(&c3_rhs103), "rhs", "rhs",
                  103);
  sf_mex_addfield(*c3_info, sf_mex_duplicatearraysafe(&c3_lhs103), "lhs", "lhs",
                  103);
  sf_mex_addfield(*c3_info, c3_emlrt_marshallOut(
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/eml/eml_index_plus.m"), "context",
                  "context", 104);
  sf_mex_addfield(*c3_info, c3_emlrt_marshallOut("coder.internal.indexPlus"),
                  "name", "name", 104);
  sf_mex_addfield(*c3_info, c3_emlrt_marshallOut("coder.internal.indexInt"),
                  "dominantType", "dominantType", 104);
  sf_mex_addfield(*c3_info, c3_emlrt_marshallOut(
    "[ILXE]$matlabroot$/toolbox/shared/coder/coder/+coder/+internal/indexPlus.m"),
                  "resolved", "resolved", 104);
  sf_mex_addfield(*c3_info, c3_b_emlrt_marshallOut(1372608360U), "fileTimeLo",
                  "fileTimeLo", 104);
  sf_mex_addfield(*c3_info, c3_b_emlrt_marshallOut(0U), "fileTimeHi",
                  "fileTimeHi", 104);
  sf_mex_addfield(*c3_info, c3_b_emlrt_marshallOut(0U), "mFileTimeLo",
                  "mFileTimeLo", 104);
  sf_mex_addfield(*c3_info, c3_b_emlrt_marshallOut(0U), "mFileTimeHi",
                  "mFileTimeHi", 104);
  sf_mex_assign(&c3_rhs104, sf_mex_createcellmatrix(0, 1), false);
  sf_mex_assign(&c3_lhs104, sf_mex_createcellmatrix(0, 1), false);
  sf_mex_addfield(*c3_info, sf_mex_duplicatearraysafe(&c3_rhs104), "rhs", "rhs",
                  104);
  sf_mex_addfield(*c3_info, sf_mex_duplicatearraysafe(&c3_lhs104), "lhs", "lhs",
                  104);
  sf_mex_addfield(*c3_info, c3_emlrt_marshallOut(
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/eml/lapack/matlab/eml_matlab_zgetrf.m"),
                  "context", "context", 105);
  sf_mex_addfield(*c3_info, c3_emlrt_marshallOut("eml_ixamax"), "name", "name",
                  105);
  sf_mex_addfield(*c3_info, c3_emlrt_marshallOut("double"), "dominantType",
                  "dominantType", 105);
  sf_mex_addfield(*c3_info, c3_emlrt_marshallOut(
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/eml/blas/eml_ixamax.m"),
                  "resolved", "resolved", 105);
  sf_mex_addfield(*c3_info, c3_b_emlrt_marshallOut(1376005888U), "fileTimeLo",
                  "fileTimeLo", 105);
  sf_mex_addfield(*c3_info, c3_b_emlrt_marshallOut(0U), "fileTimeHi",
                  "fileTimeHi", 105);
  sf_mex_addfield(*c3_info, c3_b_emlrt_marshallOut(0U), "mFileTimeLo",
                  "mFileTimeLo", 105);
  sf_mex_addfield(*c3_info, c3_b_emlrt_marshallOut(0U), "mFileTimeHi",
                  "mFileTimeHi", 105);
  sf_mex_assign(&c3_rhs105, sf_mex_createcellmatrix(0, 1), false);
  sf_mex_assign(&c3_lhs105, sf_mex_createcellmatrix(0, 1), false);
  sf_mex_addfield(*c3_info, sf_mex_duplicatearraysafe(&c3_rhs105), "rhs", "rhs",
                  105);
  sf_mex_addfield(*c3_info, sf_mex_duplicatearraysafe(&c3_lhs105), "lhs", "lhs",
                  105);
  sf_mex_addfield(*c3_info, c3_emlrt_marshallOut(
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/eml/blas/eml_ixamax.m"),
                  "context", "context", 106);
  sf_mex_addfield(*c3_info, c3_emlrt_marshallOut("coder.internal.blas.inline"),
                  "name", "name", 106);
  sf_mex_addfield(*c3_info, c3_emlrt_marshallOut(""), "dominantType",
                  "dominantType", 106);
  sf_mex_addfield(*c3_info, c3_emlrt_marshallOut(
    "[ILXE]$matlabroot$/toolbox/coder/coder/+coder/+internal/+blas/inline.p"),
                  "resolved", "resolved", 106);
  sf_mex_addfield(*c3_info, c3_b_emlrt_marshallOut(1410832972U), "fileTimeLo",
                  "fileTimeLo", 106);
  sf_mex_addfield(*c3_info, c3_b_emlrt_marshallOut(0U), "fileTimeHi",
                  "fileTimeHi", 106);
  sf_mex_addfield(*c3_info, c3_b_emlrt_marshallOut(0U), "mFileTimeLo",
                  "mFileTimeLo", 106);
  sf_mex_addfield(*c3_info, c3_b_emlrt_marshallOut(0U), "mFileTimeHi",
                  "mFileTimeHi", 106);
  sf_mex_assign(&c3_rhs106, sf_mex_createcellmatrix(0, 1), false);
  sf_mex_assign(&c3_lhs106, sf_mex_createcellmatrix(0, 1), false);
  sf_mex_addfield(*c3_info, sf_mex_duplicatearraysafe(&c3_rhs106), "rhs", "rhs",
                  106);
  sf_mex_addfield(*c3_info, sf_mex_duplicatearraysafe(&c3_lhs106), "lhs", "lhs",
                  106);
  sf_mex_addfield(*c3_info, c3_emlrt_marshallOut(
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/eml/blas/eml_ixamax.m"),
                  "context", "context", 107);
  sf_mex_addfield(*c3_info, c3_emlrt_marshallOut("coder.internal.blas.ixamax"),
                  "name", "name", 107);
  sf_mex_addfield(*c3_info, c3_emlrt_marshallOut("double"), "dominantType",
                  "dominantType", 107);
  sf_mex_addfield(*c3_info, c3_emlrt_marshallOut(
    "[ILXE]$matlabroot$/toolbox/coder/coder/+coder/+internal/+blas/ixamax.p"),
                  "resolved", "resolved", 107);
  sf_mex_addfield(*c3_info, c3_b_emlrt_marshallOut(1410832972U), "fileTimeLo",
                  "fileTimeLo", 107);
  sf_mex_addfield(*c3_info, c3_b_emlrt_marshallOut(0U), "fileTimeHi",
                  "fileTimeHi", 107);
  sf_mex_addfield(*c3_info, c3_b_emlrt_marshallOut(0U), "mFileTimeLo",
                  "mFileTimeLo", 107);
  sf_mex_addfield(*c3_info, c3_b_emlrt_marshallOut(0U), "mFileTimeHi",
                  "mFileTimeHi", 107);
  sf_mex_assign(&c3_rhs107, sf_mex_createcellmatrix(0, 1), false);
  sf_mex_assign(&c3_lhs107, sf_mex_createcellmatrix(0, 1), false);
  sf_mex_addfield(*c3_info, sf_mex_duplicatearraysafe(&c3_rhs107), "rhs", "rhs",
                  107);
  sf_mex_addfield(*c3_info, sf_mex_duplicatearraysafe(&c3_lhs107), "lhs", "lhs",
                  107);
  sf_mex_addfield(*c3_info, c3_emlrt_marshallOut(
    "[ILXE]$matlabroot$/toolbox/coder/coder/+coder/+internal/+blas/ixamax.p"),
                  "context", "context", 108);
  sf_mex_addfield(*c3_info, c3_emlrt_marshallOut(
    "coder.internal.blas.use_refblas"), "name", "name", 108);
  sf_mex_addfield(*c3_info, c3_emlrt_marshallOut(""), "dominantType",
                  "dominantType", 108);
  sf_mex_addfield(*c3_info, c3_emlrt_marshallOut(
    "[ILXE]$matlabroot$/toolbox/coder/coder/+coder/+internal/+blas/use_refblas.p"),
                  "resolved", "resolved", 108);
  sf_mex_addfield(*c3_info, c3_b_emlrt_marshallOut(1410832970U), "fileTimeLo",
                  "fileTimeLo", 108);
  sf_mex_addfield(*c3_info, c3_b_emlrt_marshallOut(0U), "fileTimeHi",
                  "fileTimeHi", 108);
  sf_mex_addfield(*c3_info, c3_b_emlrt_marshallOut(0U), "mFileTimeLo",
                  "mFileTimeLo", 108);
  sf_mex_addfield(*c3_info, c3_b_emlrt_marshallOut(0U), "mFileTimeHi",
                  "mFileTimeHi", 108);
  sf_mex_assign(&c3_rhs108, sf_mex_createcellmatrix(0, 1), false);
  sf_mex_assign(&c3_lhs108, sf_mex_createcellmatrix(0, 1), false);
  sf_mex_addfield(*c3_info, sf_mex_duplicatearraysafe(&c3_rhs108), "rhs", "rhs",
                  108);
  sf_mex_addfield(*c3_info, sf_mex_duplicatearraysafe(&c3_lhs108), "lhs", "lhs",
                  108);
  sf_mex_addfield(*c3_info, c3_emlrt_marshallOut(
    "[ILXE]$matlabroot$/toolbox/coder/coder/+coder/+internal/+blas/ixamax.p!below_threshold"),
                  "context", "context", 109);
  sf_mex_addfield(*c3_info, c3_emlrt_marshallOut("coder.internal.blas.threshold"),
                  "name", "name", 109);
  sf_mex_addfield(*c3_info, c3_emlrt_marshallOut("char"), "dominantType",
                  "dominantType", 109);
  sf_mex_addfield(*c3_info, c3_emlrt_marshallOut(
    "[ILXE]$matlabroot$/toolbox/coder/coder/+coder/+internal/+blas/threshold.p"),
                  "resolved", "resolved", 109);
  sf_mex_addfield(*c3_info, c3_b_emlrt_marshallOut(1410832972U), "fileTimeLo",
                  "fileTimeLo", 109);
  sf_mex_addfield(*c3_info, c3_b_emlrt_marshallOut(0U), "fileTimeHi",
                  "fileTimeHi", 109);
  sf_mex_addfield(*c3_info, c3_b_emlrt_marshallOut(0U), "mFileTimeLo",
                  "mFileTimeLo", 109);
  sf_mex_addfield(*c3_info, c3_b_emlrt_marshallOut(0U), "mFileTimeHi",
                  "mFileTimeHi", 109);
  sf_mex_assign(&c3_rhs109, sf_mex_createcellmatrix(0, 1), false);
  sf_mex_assign(&c3_lhs109, sf_mex_createcellmatrix(0, 1), false);
  sf_mex_addfield(*c3_info, sf_mex_duplicatearraysafe(&c3_rhs109), "rhs", "rhs",
                  109);
  sf_mex_addfield(*c3_info, sf_mex_duplicatearraysafe(&c3_lhs109), "lhs", "lhs",
                  109);
  sf_mex_addfield(*c3_info, c3_emlrt_marshallOut(
    "[ILXE]$matlabroot$/toolbox/coder/coder/+coder/+internal/+blas/ixamax.p!below_threshold"),
                  "context", "context", 110);
  sf_mex_addfield(*c3_info, c3_emlrt_marshallOut("length"), "name", "name", 110);
  sf_mex_addfield(*c3_info, c3_emlrt_marshallOut("double"), "dominantType",
                  "dominantType", 110);
  sf_mex_addfield(*c3_info, c3_emlrt_marshallOut(
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/elmat/length.m"), "resolved",
                  "resolved", 110);
  sf_mex_addfield(*c3_info, c3_b_emlrt_marshallOut(1303171406U), "fileTimeLo",
                  "fileTimeLo", 110);
  sf_mex_addfield(*c3_info, c3_b_emlrt_marshallOut(0U), "fileTimeHi",
                  "fileTimeHi", 110);
  sf_mex_addfield(*c3_info, c3_b_emlrt_marshallOut(0U), "mFileTimeLo",
                  "mFileTimeLo", 110);
  sf_mex_addfield(*c3_info, c3_b_emlrt_marshallOut(0U), "mFileTimeHi",
                  "mFileTimeHi", 110);
  sf_mex_assign(&c3_rhs110, sf_mex_createcellmatrix(0, 1), false);
  sf_mex_assign(&c3_lhs110, sf_mex_createcellmatrix(0, 1), false);
  sf_mex_addfield(*c3_info, sf_mex_duplicatearraysafe(&c3_rhs110), "rhs", "rhs",
                  110);
  sf_mex_addfield(*c3_info, sf_mex_duplicatearraysafe(&c3_lhs110), "lhs", "lhs",
                  110);
  sf_mex_addfield(*c3_info, c3_emlrt_marshallOut(
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/elmat/length.m!intlength"),
                  "context", "context", 111);
  sf_mex_addfield(*c3_info, c3_emlrt_marshallOut("eml_index_class"), "name",
                  "name", 111);
  sf_mex_addfield(*c3_info, c3_emlrt_marshallOut(""), "dominantType",
                  "dominantType", 111);
  sf_mex_addfield(*c3_info, c3_emlrt_marshallOut(
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/eml/eml_index_class.m"),
                  "resolved", "resolved", 111);
  sf_mex_addfield(*c3_info, c3_b_emlrt_marshallOut(1323195778U), "fileTimeLo",
                  "fileTimeLo", 111);
  sf_mex_addfield(*c3_info, c3_b_emlrt_marshallOut(0U), "fileTimeHi",
                  "fileTimeHi", 111);
  sf_mex_addfield(*c3_info, c3_b_emlrt_marshallOut(0U), "mFileTimeLo",
                  "mFileTimeLo", 111);
  sf_mex_addfield(*c3_info, c3_b_emlrt_marshallOut(0U), "mFileTimeHi",
                  "mFileTimeHi", 111);
  sf_mex_assign(&c3_rhs111, sf_mex_createcellmatrix(0, 1), false);
  sf_mex_assign(&c3_lhs111, sf_mex_createcellmatrix(0, 1), false);
  sf_mex_addfield(*c3_info, sf_mex_duplicatearraysafe(&c3_rhs111), "rhs", "rhs",
                  111);
  sf_mex_addfield(*c3_info, sf_mex_duplicatearraysafe(&c3_lhs111), "lhs", "lhs",
                  111);
  sf_mex_addfield(*c3_info, c3_emlrt_marshallOut(
    "[ILXE]$matlabroot$/toolbox/coder/coder/+coder/+internal/+blas/ixamax.p"),
                  "context", "context", 112);
  sf_mex_addfield(*c3_info, c3_emlrt_marshallOut("coder.internal.refblas.ixamax"),
                  "name", "name", 112);
  sf_mex_addfield(*c3_info, c3_emlrt_marshallOut("double"), "dominantType",
                  "dominantType", 112);
  sf_mex_addfield(*c3_info, c3_emlrt_marshallOut(
    "[ILXE]$matlabroot$/toolbox/coder/coder/+coder/+internal/+refblas/ixamax.p"),
                  "resolved", "resolved", 112);
  sf_mex_addfield(*c3_info, c3_b_emlrt_marshallOut(1410832972U), "fileTimeLo",
                  "fileTimeLo", 112);
  sf_mex_addfield(*c3_info, c3_b_emlrt_marshallOut(0U), "fileTimeHi",
                  "fileTimeHi", 112);
  sf_mex_addfield(*c3_info, c3_b_emlrt_marshallOut(0U), "mFileTimeLo",
                  "mFileTimeLo", 112);
  sf_mex_addfield(*c3_info, c3_b_emlrt_marshallOut(0U), "mFileTimeHi",
                  "mFileTimeHi", 112);
  sf_mex_assign(&c3_rhs112, sf_mex_createcellmatrix(0, 1), false);
  sf_mex_assign(&c3_lhs112, sf_mex_createcellmatrix(0, 1), false);
  sf_mex_addfield(*c3_info, sf_mex_duplicatearraysafe(&c3_rhs112), "rhs", "rhs",
                  112);
  sf_mex_addfield(*c3_info, sf_mex_duplicatearraysafe(&c3_lhs112), "lhs", "lhs",
                  112);
  sf_mex_addfield(*c3_info, c3_emlrt_marshallOut(
    "[ILXE]$matlabroot$/toolbox/coder/coder/+coder/+internal/+refblas/ixamax.p"),
                  "context", "context", 113);
  sf_mex_addfield(*c3_info, c3_emlrt_marshallOut("coder.internal.refblas.xcabs1"),
                  "name", "name", 113);
  sf_mex_addfield(*c3_info, c3_emlrt_marshallOut("double"), "dominantType",
                  "dominantType", 113);
  sf_mex_addfield(*c3_info, c3_emlrt_marshallOut(
    "[ILXE]$matlabroot$/toolbox/coder/coder/+coder/+internal/+refblas/xcabs1.p"),
                  "resolved", "resolved", 113);
  sf_mex_addfield(*c3_info, c3_b_emlrt_marshallOut(1410832972U), "fileTimeLo",
                  "fileTimeLo", 113);
  sf_mex_addfield(*c3_info, c3_b_emlrt_marshallOut(0U), "fileTimeHi",
                  "fileTimeHi", 113);
  sf_mex_addfield(*c3_info, c3_b_emlrt_marshallOut(0U), "mFileTimeLo",
                  "mFileTimeLo", 113);
  sf_mex_addfield(*c3_info, c3_b_emlrt_marshallOut(0U), "mFileTimeHi",
                  "mFileTimeHi", 113);
  sf_mex_assign(&c3_rhs113, sf_mex_createcellmatrix(0, 1), false);
  sf_mex_assign(&c3_lhs113, sf_mex_createcellmatrix(0, 1), false);
  sf_mex_addfield(*c3_info, sf_mex_duplicatearraysafe(&c3_rhs113), "rhs", "rhs",
                  113);
  sf_mex_addfield(*c3_info, sf_mex_duplicatearraysafe(&c3_lhs113), "lhs", "lhs",
                  113);
  sf_mex_addfield(*c3_info, c3_emlrt_marshallOut(
    "[ILXE]$matlabroot$/toolbox/coder/coder/+coder/+internal/+refblas/xcabs1.p"),
                  "context", "context", 114);
  sf_mex_addfield(*c3_info, c3_emlrt_marshallOut("abs"), "name", "name", 114);
  sf_mex_addfield(*c3_info, c3_emlrt_marshallOut("double"), "dominantType",
                  "dominantType", 114);
  sf_mex_addfield(*c3_info, c3_emlrt_marshallOut(
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/elfun/abs.m"), "resolved",
                  "resolved", 114);
  sf_mex_addfield(*c3_info, c3_b_emlrt_marshallOut(1363735452U), "fileTimeLo",
                  "fileTimeLo", 114);
  sf_mex_addfield(*c3_info, c3_b_emlrt_marshallOut(0U), "fileTimeHi",
                  "fileTimeHi", 114);
  sf_mex_addfield(*c3_info, c3_b_emlrt_marshallOut(0U), "mFileTimeLo",
                  "mFileTimeLo", 114);
  sf_mex_addfield(*c3_info, c3_b_emlrt_marshallOut(0U), "mFileTimeHi",
                  "mFileTimeHi", 114);
  sf_mex_assign(&c3_rhs114, sf_mex_createcellmatrix(0, 1), false);
  sf_mex_assign(&c3_lhs114, sf_mex_createcellmatrix(0, 1), false);
  sf_mex_addfield(*c3_info, sf_mex_duplicatearraysafe(&c3_rhs114), "rhs", "rhs",
                  114);
  sf_mex_addfield(*c3_info, sf_mex_duplicatearraysafe(&c3_lhs114), "lhs", "lhs",
                  114);
  sf_mex_addfield(*c3_info, c3_emlrt_marshallOut(
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/elfun/abs.m"), "context",
                  "context", 115);
  sf_mex_addfield(*c3_info, c3_emlrt_marshallOut(
    "coder.internal.isBuiltInNumeric"), "name", "name", 115);
  sf_mex_addfield(*c3_info, c3_emlrt_marshallOut("double"), "dominantType",
                  "dominantType", 115);
  sf_mex_addfield(*c3_info, c3_emlrt_marshallOut(
    "[ILXE]$matlabroot$/toolbox/shared/coder/coder/+coder/+internal/isBuiltInNumeric.m"),
                  "resolved", "resolved", 115);
  sf_mex_addfield(*c3_info, c3_b_emlrt_marshallOut(1395953456U), "fileTimeLo",
                  "fileTimeLo", 115);
  sf_mex_addfield(*c3_info, c3_b_emlrt_marshallOut(0U), "fileTimeHi",
                  "fileTimeHi", 115);
  sf_mex_addfield(*c3_info, c3_b_emlrt_marshallOut(0U), "mFileTimeLo",
                  "mFileTimeLo", 115);
  sf_mex_addfield(*c3_info, c3_b_emlrt_marshallOut(0U), "mFileTimeHi",
                  "mFileTimeHi", 115);
  sf_mex_assign(&c3_rhs115, sf_mex_createcellmatrix(0, 1), false);
  sf_mex_assign(&c3_lhs115, sf_mex_createcellmatrix(0, 1), false);
  sf_mex_addfield(*c3_info, sf_mex_duplicatearraysafe(&c3_rhs115), "rhs", "rhs",
                  115);
  sf_mex_addfield(*c3_info, sf_mex_duplicatearraysafe(&c3_lhs115), "lhs", "lhs",
                  115);
  sf_mex_addfield(*c3_info, c3_emlrt_marshallOut(
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/elfun/abs.m"), "context",
                  "context", 116);
  sf_mex_addfield(*c3_info, c3_emlrt_marshallOut("eml_scalar_abs"), "name",
                  "name", 116);
  sf_mex_addfield(*c3_info, c3_emlrt_marshallOut("double"), "dominantType",
                  "dominantType", 116);
  sf_mex_addfield(*c3_info, c3_emlrt_marshallOut(
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/elfun/eml_scalar_abs.m"),
                  "resolved", "resolved", 116);
  sf_mex_addfield(*c3_info, c3_b_emlrt_marshallOut(1286843912U), "fileTimeLo",
                  "fileTimeLo", 116);
  sf_mex_addfield(*c3_info, c3_b_emlrt_marshallOut(0U), "fileTimeHi",
                  "fileTimeHi", 116);
  sf_mex_addfield(*c3_info, c3_b_emlrt_marshallOut(0U), "mFileTimeLo",
                  "mFileTimeLo", 116);
  sf_mex_addfield(*c3_info, c3_b_emlrt_marshallOut(0U), "mFileTimeHi",
                  "mFileTimeHi", 116);
  sf_mex_assign(&c3_rhs116, sf_mex_createcellmatrix(0, 1), false);
  sf_mex_assign(&c3_lhs116, sf_mex_createcellmatrix(0, 1), false);
  sf_mex_addfield(*c3_info, sf_mex_duplicatearraysafe(&c3_rhs116), "rhs", "rhs",
                  116);
  sf_mex_addfield(*c3_info, sf_mex_duplicatearraysafe(&c3_lhs116), "lhs", "lhs",
                  116);
  sf_mex_addfield(*c3_info, c3_emlrt_marshallOut(
    "[ILXE]$matlabroot$/toolbox/coder/coder/+coder/+internal/+refblas/ixamax.p"),
                  "context", "context", 117);
  sf_mex_addfield(*c3_info, c3_emlrt_marshallOut(
    "eml_int_forloop_overflow_check"), "name", "name", 117);
  sf_mex_addfield(*c3_info, c3_emlrt_marshallOut(""), "dominantType",
                  "dominantType", 117);
  sf_mex_addfield(*c3_info, c3_emlrt_marshallOut(
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/eml/eml_int_forloop_overflow_check.m"),
                  "resolved", "resolved", 117);
  sf_mex_addfield(*c3_info, c3_b_emlrt_marshallOut(1397282622U), "fileTimeLo",
                  "fileTimeLo", 117);
  sf_mex_addfield(*c3_info, c3_b_emlrt_marshallOut(0U), "fileTimeHi",
                  "fileTimeHi", 117);
  sf_mex_addfield(*c3_info, c3_b_emlrt_marshallOut(0U), "mFileTimeLo",
                  "mFileTimeLo", 117);
  sf_mex_addfield(*c3_info, c3_b_emlrt_marshallOut(0U), "mFileTimeHi",
                  "mFileTimeHi", 117);
  sf_mex_assign(&c3_rhs117, sf_mex_createcellmatrix(0, 1), false);
  sf_mex_assign(&c3_lhs117, sf_mex_createcellmatrix(0, 1), false);
  sf_mex_addfield(*c3_info, sf_mex_duplicatearraysafe(&c3_rhs117), "rhs", "rhs",
                  117);
  sf_mex_addfield(*c3_info, sf_mex_duplicatearraysafe(&c3_lhs117), "lhs", "lhs",
                  117);
  sf_mex_addfield(*c3_info, c3_emlrt_marshallOut(
    "[ILXE]$matlabroot$/toolbox/coder/coder/+coder/+internal/+refblas/ixamax.p"),
                  "context", "context", 118);
  sf_mex_addfield(*c3_info, c3_emlrt_marshallOut("coder.internal.indexPlus"),
                  "name", "name", 118);
  sf_mex_addfield(*c3_info, c3_emlrt_marshallOut("coder.internal.indexInt"),
                  "dominantType", "dominantType", 118);
  sf_mex_addfield(*c3_info, c3_emlrt_marshallOut(
    "[ILXE]$matlabroot$/toolbox/shared/coder/coder/+coder/+internal/indexPlus.m"),
                  "resolved", "resolved", 118);
  sf_mex_addfield(*c3_info, c3_b_emlrt_marshallOut(1372608360U), "fileTimeLo",
                  "fileTimeLo", 118);
  sf_mex_addfield(*c3_info, c3_b_emlrt_marshallOut(0U), "fileTimeHi",
                  "fileTimeHi", 118);
  sf_mex_addfield(*c3_info, c3_b_emlrt_marshallOut(0U), "mFileTimeLo",
                  "mFileTimeLo", 118);
  sf_mex_addfield(*c3_info, c3_b_emlrt_marshallOut(0U), "mFileTimeHi",
                  "mFileTimeHi", 118);
  sf_mex_assign(&c3_rhs118, sf_mex_createcellmatrix(0, 1), false);
  sf_mex_assign(&c3_lhs118, sf_mex_createcellmatrix(0, 1), false);
  sf_mex_addfield(*c3_info, sf_mex_duplicatearraysafe(&c3_rhs118), "rhs", "rhs",
                  118);
  sf_mex_addfield(*c3_info, sf_mex_duplicatearraysafe(&c3_lhs118), "lhs", "lhs",
                  118);
  sf_mex_addfield(*c3_info, c3_emlrt_marshallOut(
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/eml/lapack/matlab/eml_matlab_zgetrf.m"),
                  "context", "context", 119);
  sf_mex_addfield(*c3_info, c3_emlrt_marshallOut("eml_xswap"), "name", "name",
                  119);
  sf_mex_addfield(*c3_info, c3_emlrt_marshallOut("double"), "dominantType",
                  "dominantType", 119);
  sf_mex_addfield(*c3_info, c3_emlrt_marshallOut(
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/eml/blas/eml_xswap.m"),
                  "resolved", "resolved", 119);
  sf_mex_addfield(*c3_info, c3_b_emlrt_marshallOut(1376005892U), "fileTimeLo",
                  "fileTimeLo", 119);
  sf_mex_addfield(*c3_info, c3_b_emlrt_marshallOut(0U), "fileTimeHi",
                  "fileTimeHi", 119);
  sf_mex_addfield(*c3_info, c3_b_emlrt_marshallOut(0U), "mFileTimeLo",
                  "mFileTimeLo", 119);
  sf_mex_addfield(*c3_info, c3_b_emlrt_marshallOut(0U), "mFileTimeHi",
                  "mFileTimeHi", 119);
  sf_mex_assign(&c3_rhs119, sf_mex_createcellmatrix(0, 1), false);
  sf_mex_assign(&c3_lhs119, sf_mex_createcellmatrix(0, 1), false);
  sf_mex_addfield(*c3_info, sf_mex_duplicatearraysafe(&c3_rhs119), "rhs", "rhs",
                  119);
  sf_mex_addfield(*c3_info, sf_mex_duplicatearraysafe(&c3_lhs119), "lhs", "lhs",
                  119);
  sf_mex_addfield(*c3_info, c3_emlrt_marshallOut(
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/eml/blas/eml_xswap.m"), "context",
                  "context", 120);
  sf_mex_addfield(*c3_info, c3_emlrt_marshallOut("coder.internal.blas.inline"),
                  "name", "name", 120);
  sf_mex_addfield(*c3_info, c3_emlrt_marshallOut(""), "dominantType",
                  "dominantType", 120);
  sf_mex_addfield(*c3_info, c3_emlrt_marshallOut(
    "[ILXE]$matlabroot$/toolbox/coder/coder/+coder/+internal/+blas/inline.p"),
                  "resolved", "resolved", 120);
  sf_mex_addfield(*c3_info, c3_b_emlrt_marshallOut(1410832972U), "fileTimeLo",
                  "fileTimeLo", 120);
  sf_mex_addfield(*c3_info, c3_b_emlrt_marshallOut(0U), "fileTimeHi",
                  "fileTimeHi", 120);
  sf_mex_addfield(*c3_info, c3_b_emlrt_marshallOut(0U), "mFileTimeLo",
                  "mFileTimeLo", 120);
  sf_mex_addfield(*c3_info, c3_b_emlrt_marshallOut(0U), "mFileTimeHi",
                  "mFileTimeHi", 120);
  sf_mex_assign(&c3_rhs120, sf_mex_createcellmatrix(0, 1), false);
  sf_mex_assign(&c3_lhs120, sf_mex_createcellmatrix(0, 1), false);
  sf_mex_addfield(*c3_info, sf_mex_duplicatearraysafe(&c3_rhs120), "rhs", "rhs",
                  120);
  sf_mex_addfield(*c3_info, sf_mex_duplicatearraysafe(&c3_lhs120), "lhs", "lhs",
                  120);
  sf_mex_addfield(*c3_info, c3_emlrt_marshallOut(
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/eml/blas/eml_xswap.m"), "context",
                  "context", 121);
  sf_mex_addfield(*c3_info, c3_emlrt_marshallOut("coder.internal.blas.xswap"),
                  "name", "name", 121);
  sf_mex_addfield(*c3_info, c3_emlrt_marshallOut("double"), "dominantType",
                  "dominantType", 121);
  sf_mex_addfield(*c3_info, c3_emlrt_marshallOut(
    "[ILXE]$matlabroot$/toolbox/coder/coder/+coder/+internal/+blas/xswap.p"),
                  "resolved", "resolved", 121);
  sf_mex_addfield(*c3_info, c3_b_emlrt_marshallOut(1410832970U), "fileTimeLo",
                  "fileTimeLo", 121);
  sf_mex_addfield(*c3_info, c3_b_emlrt_marshallOut(0U), "fileTimeHi",
                  "fileTimeHi", 121);
  sf_mex_addfield(*c3_info, c3_b_emlrt_marshallOut(0U), "mFileTimeLo",
                  "mFileTimeLo", 121);
  sf_mex_addfield(*c3_info, c3_b_emlrt_marshallOut(0U), "mFileTimeHi",
                  "mFileTimeHi", 121);
  sf_mex_assign(&c3_rhs121, sf_mex_createcellmatrix(0, 1), false);
  sf_mex_assign(&c3_lhs121, sf_mex_createcellmatrix(0, 1), false);
  sf_mex_addfield(*c3_info, sf_mex_duplicatearraysafe(&c3_rhs121), "rhs", "rhs",
                  121);
  sf_mex_addfield(*c3_info, sf_mex_duplicatearraysafe(&c3_lhs121), "lhs", "lhs",
                  121);
  sf_mex_addfield(*c3_info, c3_emlrt_marshallOut(
    "[ILXE]$matlabroot$/toolbox/coder/coder/+coder/+internal/+blas/xswap.p"),
                  "context", "context", 122);
  sf_mex_addfield(*c3_info, c3_emlrt_marshallOut(
    "coder.internal.blas.use_refblas"), "name", "name", 122);
  sf_mex_addfield(*c3_info, c3_emlrt_marshallOut(""), "dominantType",
                  "dominantType", 122);
  sf_mex_addfield(*c3_info, c3_emlrt_marshallOut(
    "[ILXE]$matlabroot$/toolbox/coder/coder/+coder/+internal/+blas/use_refblas.p"),
                  "resolved", "resolved", 122);
  sf_mex_addfield(*c3_info, c3_b_emlrt_marshallOut(1410832970U), "fileTimeLo",
                  "fileTimeLo", 122);
  sf_mex_addfield(*c3_info, c3_b_emlrt_marshallOut(0U), "fileTimeHi",
                  "fileTimeHi", 122);
  sf_mex_addfield(*c3_info, c3_b_emlrt_marshallOut(0U), "mFileTimeLo",
                  "mFileTimeLo", 122);
  sf_mex_addfield(*c3_info, c3_b_emlrt_marshallOut(0U), "mFileTimeHi",
                  "mFileTimeHi", 122);
  sf_mex_assign(&c3_rhs122, sf_mex_createcellmatrix(0, 1), false);
  sf_mex_assign(&c3_lhs122, sf_mex_createcellmatrix(0, 1), false);
  sf_mex_addfield(*c3_info, sf_mex_duplicatearraysafe(&c3_rhs122), "rhs", "rhs",
                  122);
  sf_mex_addfield(*c3_info, sf_mex_duplicatearraysafe(&c3_lhs122), "lhs", "lhs",
                  122);
  sf_mex_addfield(*c3_info, c3_emlrt_marshallOut(
    "[ILXE]$matlabroot$/toolbox/coder/coder/+coder/+internal/+blas/xswap.p!below_threshold"),
                  "context", "context", 123);
  sf_mex_addfield(*c3_info, c3_emlrt_marshallOut("coder.internal.blas.threshold"),
                  "name", "name", 123);
  sf_mex_addfield(*c3_info, c3_emlrt_marshallOut("char"), "dominantType",
                  "dominantType", 123);
  sf_mex_addfield(*c3_info, c3_emlrt_marshallOut(
    "[ILXE]$matlabroot$/toolbox/coder/coder/+coder/+internal/+blas/threshold.p"),
                  "resolved", "resolved", 123);
  sf_mex_addfield(*c3_info, c3_b_emlrt_marshallOut(1410832972U), "fileTimeLo",
                  "fileTimeLo", 123);
  sf_mex_addfield(*c3_info, c3_b_emlrt_marshallOut(0U), "fileTimeHi",
                  "fileTimeHi", 123);
  sf_mex_addfield(*c3_info, c3_b_emlrt_marshallOut(0U), "mFileTimeLo",
                  "mFileTimeLo", 123);
  sf_mex_addfield(*c3_info, c3_b_emlrt_marshallOut(0U), "mFileTimeHi",
                  "mFileTimeHi", 123);
  sf_mex_assign(&c3_rhs123, sf_mex_createcellmatrix(0, 1), false);
  sf_mex_assign(&c3_lhs123, sf_mex_createcellmatrix(0, 1), false);
  sf_mex_addfield(*c3_info, sf_mex_duplicatearraysafe(&c3_rhs123), "rhs", "rhs",
                  123);
  sf_mex_addfield(*c3_info, sf_mex_duplicatearraysafe(&c3_lhs123), "lhs", "lhs",
                  123);
  sf_mex_addfield(*c3_info, c3_emlrt_marshallOut(
    "[ILXE]$matlabroot$/toolbox/coder/coder/+coder/+internal/+blas/xswap.p"),
                  "context", "context", 124);
  sf_mex_addfield(*c3_info, c3_emlrt_marshallOut("coder.internal.refblas.xswap"),
                  "name", "name", 124);
  sf_mex_addfield(*c3_info, c3_emlrt_marshallOut("double"), "dominantType",
                  "dominantType", 124);
  sf_mex_addfield(*c3_info, c3_emlrt_marshallOut(
    "[ILXE]$matlabroot$/toolbox/coder/coder/+coder/+internal/+refblas/xswap.p"),
                  "resolved", "resolved", 124);
  sf_mex_addfield(*c3_info, c3_b_emlrt_marshallOut(1410832972U), "fileTimeLo",
                  "fileTimeLo", 124);
  sf_mex_addfield(*c3_info, c3_b_emlrt_marshallOut(0U), "fileTimeHi",
                  "fileTimeHi", 124);
  sf_mex_addfield(*c3_info, c3_b_emlrt_marshallOut(0U), "mFileTimeLo",
                  "mFileTimeLo", 124);
  sf_mex_addfield(*c3_info, c3_b_emlrt_marshallOut(0U), "mFileTimeHi",
                  "mFileTimeHi", 124);
  sf_mex_assign(&c3_rhs124, sf_mex_createcellmatrix(0, 1), false);
  sf_mex_assign(&c3_lhs124, sf_mex_createcellmatrix(0, 1), false);
  sf_mex_addfield(*c3_info, sf_mex_duplicatearraysafe(&c3_rhs124), "rhs", "rhs",
                  124);
  sf_mex_addfield(*c3_info, sf_mex_duplicatearraysafe(&c3_lhs124), "lhs", "lhs",
                  124);
  sf_mex_addfield(*c3_info, c3_emlrt_marshallOut(
    "[ILXE]$matlabroot$/toolbox/coder/coder/+coder/+internal/+refblas/xswap.p"),
                  "context", "context", 125);
  sf_mex_addfield(*c3_info, c3_emlrt_marshallOut("abs"), "name", "name", 125);
  sf_mex_addfield(*c3_info, c3_emlrt_marshallOut("coder.internal.indexInt"),
                  "dominantType", "dominantType", 125);
  sf_mex_addfield(*c3_info, c3_emlrt_marshallOut(
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/elfun/abs.m"), "resolved",
                  "resolved", 125);
  sf_mex_addfield(*c3_info, c3_b_emlrt_marshallOut(1363735452U), "fileTimeLo",
                  "fileTimeLo", 125);
  sf_mex_addfield(*c3_info, c3_b_emlrt_marshallOut(0U), "fileTimeHi",
                  "fileTimeHi", 125);
  sf_mex_addfield(*c3_info, c3_b_emlrt_marshallOut(0U), "mFileTimeLo",
                  "mFileTimeLo", 125);
  sf_mex_addfield(*c3_info, c3_b_emlrt_marshallOut(0U), "mFileTimeHi",
                  "mFileTimeHi", 125);
  sf_mex_assign(&c3_rhs125, sf_mex_createcellmatrix(0, 1), false);
  sf_mex_assign(&c3_lhs125, sf_mex_createcellmatrix(0, 1), false);
  sf_mex_addfield(*c3_info, sf_mex_duplicatearraysafe(&c3_rhs125), "rhs", "rhs",
                  125);
  sf_mex_addfield(*c3_info, sf_mex_duplicatearraysafe(&c3_lhs125), "lhs", "lhs",
                  125);
  sf_mex_addfield(*c3_info, c3_emlrt_marshallOut(
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/elfun/abs.m"), "context",
                  "context", 126);
  sf_mex_addfield(*c3_info, c3_emlrt_marshallOut(
    "coder.internal.isBuiltInNumeric"), "name", "name", 126);
  sf_mex_addfield(*c3_info, c3_emlrt_marshallOut("coder.internal.indexInt"),
                  "dominantType", "dominantType", 126);
  sf_mex_addfield(*c3_info, c3_emlrt_marshallOut(
    "[ILXE]$matlabroot$/toolbox/shared/coder/coder/+coder/+internal/isBuiltInNumeric.m"),
                  "resolved", "resolved", 126);
  sf_mex_addfield(*c3_info, c3_b_emlrt_marshallOut(1395953456U), "fileTimeLo",
                  "fileTimeLo", 126);
  sf_mex_addfield(*c3_info, c3_b_emlrt_marshallOut(0U), "fileTimeHi",
                  "fileTimeHi", 126);
  sf_mex_addfield(*c3_info, c3_b_emlrt_marshallOut(0U), "mFileTimeLo",
                  "mFileTimeLo", 126);
  sf_mex_addfield(*c3_info, c3_b_emlrt_marshallOut(0U), "mFileTimeHi",
                  "mFileTimeHi", 126);
  sf_mex_assign(&c3_rhs126, sf_mex_createcellmatrix(0, 1), false);
  sf_mex_assign(&c3_lhs126, sf_mex_createcellmatrix(0, 1), false);
  sf_mex_addfield(*c3_info, sf_mex_duplicatearraysafe(&c3_rhs126), "rhs", "rhs",
                  126);
  sf_mex_addfield(*c3_info, sf_mex_duplicatearraysafe(&c3_lhs126), "lhs", "lhs",
                  126);
  sf_mex_addfield(*c3_info, c3_emlrt_marshallOut(
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/elfun/abs.m"), "context",
                  "context", 127);
  sf_mex_addfield(*c3_info, c3_emlrt_marshallOut("eml_scalar_abs"), "name",
                  "name", 127);
  sf_mex_addfield(*c3_info, c3_emlrt_marshallOut("coder.internal.indexInt"),
                  "dominantType", "dominantType", 127);
  sf_mex_addfield(*c3_info, c3_emlrt_marshallOut(
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/elfun/eml_scalar_abs.m"),
                  "resolved", "resolved", 127);
  sf_mex_addfield(*c3_info, c3_b_emlrt_marshallOut(1286843912U), "fileTimeLo",
                  "fileTimeLo", 127);
  sf_mex_addfield(*c3_info, c3_b_emlrt_marshallOut(0U), "fileTimeHi",
                  "fileTimeHi", 127);
  sf_mex_addfield(*c3_info, c3_b_emlrt_marshallOut(0U), "mFileTimeLo",
                  "mFileTimeLo", 127);
  sf_mex_addfield(*c3_info, c3_b_emlrt_marshallOut(0U), "mFileTimeHi",
                  "mFileTimeHi", 127);
  sf_mex_assign(&c3_rhs127, sf_mex_createcellmatrix(0, 1), false);
  sf_mex_assign(&c3_lhs127, sf_mex_createcellmatrix(0, 1), false);
  sf_mex_addfield(*c3_info, sf_mex_duplicatearraysafe(&c3_rhs127), "rhs", "rhs",
                  127);
  sf_mex_addfield(*c3_info, sf_mex_duplicatearraysafe(&c3_lhs127), "lhs", "lhs",
                  127);
  sf_mex_destroy(&c3_rhs64);
  sf_mex_destroy(&c3_lhs64);
  sf_mex_destroy(&c3_rhs65);
  sf_mex_destroy(&c3_lhs65);
  sf_mex_destroy(&c3_rhs66);
  sf_mex_destroy(&c3_lhs66);
  sf_mex_destroy(&c3_rhs67);
  sf_mex_destroy(&c3_lhs67);
  sf_mex_destroy(&c3_rhs68);
  sf_mex_destroy(&c3_lhs68);
  sf_mex_destroy(&c3_rhs69);
  sf_mex_destroy(&c3_lhs69);
  sf_mex_destroy(&c3_rhs70);
  sf_mex_destroy(&c3_lhs70);
  sf_mex_destroy(&c3_rhs71);
  sf_mex_destroy(&c3_lhs71);
  sf_mex_destroy(&c3_rhs72);
  sf_mex_destroy(&c3_lhs72);
  sf_mex_destroy(&c3_rhs73);
  sf_mex_destroy(&c3_lhs73);
  sf_mex_destroy(&c3_rhs74);
  sf_mex_destroy(&c3_lhs74);
  sf_mex_destroy(&c3_rhs75);
  sf_mex_destroy(&c3_lhs75);
  sf_mex_destroy(&c3_rhs76);
  sf_mex_destroy(&c3_lhs76);
  sf_mex_destroy(&c3_rhs77);
  sf_mex_destroy(&c3_lhs77);
  sf_mex_destroy(&c3_rhs78);
  sf_mex_destroy(&c3_lhs78);
  sf_mex_destroy(&c3_rhs79);
  sf_mex_destroy(&c3_lhs79);
  sf_mex_destroy(&c3_rhs80);
  sf_mex_destroy(&c3_lhs80);
  sf_mex_destroy(&c3_rhs81);
  sf_mex_destroy(&c3_lhs81);
  sf_mex_destroy(&c3_rhs82);
  sf_mex_destroy(&c3_lhs82);
  sf_mex_destroy(&c3_rhs83);
  sf_mex_destroy(&c3_lhs83);
  sf_mex_destroy(&c3_rhs84);
  sf_mex_destroy(&c3_lhs84);
  sf_mex_destroy(&c3_rhs85);
  sf_mex_destroy(&c3_lhs85);
  sf_mex_destroy(&c3_rhs86);
  sf_mex_destroy(&c3_lhs86);
  sf_mex_destroy(&c3_rhs87);
  sf_mex_destroy(&c3_lhs87);
  sf_mex_destroy(&c3_rhs88);
  sf_mex_destroy(&c3_lhs88);
  sf_mex_destroy(&c3_rhs89);
  sf_mex_destroy(&c3_lhs89);
  sf_mex_destroy(&c3_rhs90);
  sf_mex_destroy(&c3_lhs90);
  sf_mex_destroy(&c3_rhs91);
  sf_mex_destroy(&c3_lhs91);
  sf_mex_destroy(&c3_rhs92);
  sf_mex_destroy(&c3_lhs92);
  sf_mex_destroy(&c3_rhs93);
  sf_mex_destroy(&c3_lhs93);
  sf_mex_destroy(&c3_rhs94);
  sf_mex_destroy(&c3_lhs94);
  sf_mex_destroy(&c3_rhs95);
  sf_mex_destroy(&c3_lhs95);
  sf_mex_destroy(&c3_rhs96);
  sf_mex_destroy(&c3_lhs96);
  sf_mex_destroy(&c3_rhs97);
  sf_mex_destroy(&c3_lhs97);
  sf_mex_destroy(&c3_rhs98);
  sf_mex_destroy(&c3_lhs98);
  sf_mex_destroy(&c3_rhs99);
  sf_mex_destroy(&c3_lhs99);
  sf_mex_destroy(&c3_rhs100);
  sf_mex_destroy(&c3_lhs100);
  sf_mex_destroy(&c3_rhs101);
  sf_mex_destroy(&c3_lhs101);
  sf_mex_destroy(&c3_rhs102);
  sf_mex_destroy(&c3_lhs102);
  sf_mex_destroy(&c3_rhs103);
  sf_mex_destroy(&c3_lhs103);
  sf_mex_destroy(&c3_rhs104);
  sf_mex_destroy(&c3_lhs104);
  sf_mex_destroy(&c3_rhs105);
  sf_mex_destroy(&c3_lhs105);
  sf_mex_destroy(&c3_rhs106);
  sf_mex_destroy(&c3_lhs106);
  sf_mex_destroy(&c3_rhs107);
  sf_mex_destroy(&c3_lhs107);
  sf_mex_destroy(&c3_rhs108);
  sf_mex_destroy(&c3_lhs108);
  sf_mex_destroy(&c3_rhs109);
  sf_mex_destroy(&c3_lhs109);
  sf_mex_destroy(&c3_rhs110);
  sf_mex_destroy(&c3_lhs110);
  sf_mex_destroy(&c3_rhs111);
  sf_mex_destroy(&c3_lhs111);
  sf_mex_destroy(&c3_rhs112);
  sf_mex_destroy(&c3_lhs112);
  sf_mex_destroy(&c3_rhs113);
  sf_mex_destroy(&c3_lhs113);
  sf_mex_destroy(&c3_rhs114);
  sf_mex_destroy(&c3_lhs114);
  sf_mex_destroy(&c3_rhs115);
  sf_mex_destroy(&c3_lhs115);
  sf_mex_destroy(&c3_rhs116);
  sf_mex_destroy(&c3_lhs116);
  sf_mex_destroy(&c3_rhs117);
  sf_mex_destroy(&c3_lhs117);
  sf_mex_destroy(&c3_rhs118);
  sf_mex_destroy(&c3_lhs118);
  sf_mex_destroy(&c3_rhs119);
  sf_mex_destroy(&c3_lhs119);
  sf_mex_destroy(&c3_rhs120);
  sf_mex_destroy(&c3_lhs120);
  sf_mex_destroy(&c3_rhs121);
  sf_mex_destroy(&c3_lhs121);
  sf_mex_destroy(&c3_rhs122);
  sf_mex_destroy(&c3_lhs122);
  sf_mex_destroy(&c3_rhs123);
  sf_mex_destroy(&c3_lhs123);
  sf_mex_destroy(&c3_rhs124);
  sf_mex_destroy(&c3_lhs124);
  sf_mex_destroy(&c3_rhs125);
  sf_mex_destroy(&c3_lhs125);
  sf_mex_destroy(&c3_rhs126);
  sf_mex_destroy(&c3_lhs126);
  sf_mex_destroy(&c3_rhs127);
  sf_mex_destroy(&c3_lhs127);
}

static void c3_c_info_helper(const mxArray **c3_info)
{
  const mxArray *c3_rhs128 = NULL;
  const mxArray *c3_lhs128 = NULL;
  const mxArray *c3_rhs129 = NULL;
  const mxArray *c3_lhs129 = NULL;
  const mxArray *c3_rhs130 = NULL;
  const mxArray *c3_lhs130 = NULL;
  const mxArray *c3_rhs131 = NULL;
  const mxArray *c3_lhs131 = NULL;
  const mxArray *c3_rhs132 = NULL;
  const mxArray *c3_lhs132 = NULL;
  const mxArray *c3_rhs133 = NULL;
  const mxArray *c3_lhs133 = NULL;
  const mxArray *c3_rhs134 = NULL;
  const mxArray *c3_lhs134 = NULL;
  const mxArray *c3_rhs135 = NULL;
  const mxArray *c3_lhs135 = NULL;
  const mxArray *c3_rhs136 = NULL;
  const mxArray *c3_lhs136 = NULL;
  const mxArray *c3_rhs137 = NULL;
  const mxArray *c3_lhs137 = NULL;
  const mxArray *c3_rhs138 = NULL;
  const mxArray *c3_lhs138 = NULL;
  const mxArray *c3_rhs139 = NULL;
  const mxArray *c3_lhs139 = NULL;
  const mxArray *c3_rhs140 = NULL;
  const mxArray *c3_lhs140 = NULL;
  const mxArray *c3_rhs141 = NULL;
  const mxArray *c3_lhs141 = NULL;
  const mxArray *c3_rhs142 = NULL;
  const mxArray *c3_lhs142 = NULL;
  const mxArray *c3_rhs143 = NULL;
  const mxArray *c3_lhs143 = NULL;
  const mxArray *c3_rhs144 = NULL;
  const mxArray *c3_lhs144 = NULL;
  const mxArray *c3_rhs145 = NULL;
  const mxArray *c3_lhs145 = NULL;
  const mxArray *c3_rhs146 = NULL;
  const mxArray *c3_lhs146 = NULL;
  const mxArray *c3_rhs147 = NULL;
  const mxArray *c3_lhs147 = NULL;
  const mxArray *c3_rhs148 = NULL;
  const mxArray *c3_lhs148 = NULL;
  const mxArray *c3_rhs149 = NULL;
  const mxArray *c3_lhs149 = NULL;
  const mxArray *c3_rhs150 = NULL;
  const mxArray *c3_lhs150 = NULL;
  const mxArray *c3_rhs151 = NULL;
  const mxArray *c3_lhs151 = NULL;
  const mxArray *c3_rhs152 = NULL;
  const mxArray *c3_lhs152 = NULL;
  const mxArray *c3_rhs153 = NULL;
  const mxArray *c3_lhs153 = NULL;
  const mxArray *c3_rhs154 = NULL;
  const mxArray *c3_lhs154 = NULL;
  const mxArray *c3_rhs155 = NULL;
  const mxArray *c3_lhs155 = NULL;
  const mxArray *c3_rhs156 = NULL;
  const mxArray *c3_lhs156 = NULL;
  const mxArray *c3_rhs157 = NULL;
  const mxArray *c3_lhs157 = NULL;
  const mxArray *c3_rhs158 = NULL;
  const mxArray *c3_lhs158 = NULL;
  const mxArray *c3_rhs159 = NULL;
  const mxArray *c3_lhs159 = NULL;
  const mxArray *c3_rhs160 = NULL;
  const mxArray *c3_lhs160 = NULL;
  const mxArray *c3_rhs161 = NULL;
  const mxArray *c3_lhs161 = NULL;
  const mxArray *c3_rhs162 = NULL;
  const mxArray *c3_lhs162 = NULL;
  const mxArray *c3_rhs163 = NULL;
  const mxArray *c3_lhs163 = NULL;
  const mxArray *c3_rhs164 = NULL;
  const mxArray *c3_lhs164 = NULL;
  const mxArray *c3_rhs165 = NULL;
  const mxArray *c3_lhs165 = NULL;
  const mxArray *c3_rhs166 = NULL;
  const mxArray *c3_lhs166 = NULL;
  const mxArray *c3_rhs167 = NULL;
  const mxArray *c3_lhs167 = NULL;
  const mxArray *c3_rhs168 = NULL;
  const mxArray *c3_lhs168 = NULL;
  const mxArray *c3_rhs169 = NULL;
  const mxArray *c3_lhs169 = NULL;
  const mxArray *c3_rhs170 = NULL;
  const mxArray *c3_lhs170 = NULL;
  const mxArray *c3_rhs171 = NULL;
  const mxArray *c3_lhs171 = NULL;
  const mxArray *c3_rhs172 = NULL;
  const mxArray *c3_lhs172 = NULL;
  const mxArray *c3_rhs173 = NULL;
  const mxArray *c3_lhs173 = NULL;
  const mxArray *c3_rhs174 = NULL;
  const mxArray *c3_lhs174 = NULL;
  const mxArray *c3_rhs175 = NULL;
  const mxArray *c3_lhs175 = NULL;
  const mxArray *c3_rhs176 = NULL;
  const mxArray *c3_lhs176 = NULL;
  const mxArray *c3_rhs177 = NULL;
  const mxArray *c3_lhs177 = NULL;
  const mxArray *c3_rhs178 = NULL;
  const mxArray *c3_lhs178 = NULL;
  const mxArray *c3_rhs179 = NULL;
  const mxArray *c3_lhs179 = NULL;
  const mxArray *c3_rhs180 = NULL;
  const mxArray *c3_lhs180 = NULL;
  const mxArray *c3_rhs181 = NULL;
  const mxArray *c3_lhs181 = NULL;
  const mxArray *c3_rhs182 = NULL;
  const mxArray *c3_lhs182 = NULL;
  const mxArray *c3_rhs183 = NULL;
  const mxArray *c3_lhs183 = NULL;
  const mxArray *c3_rhs184 = NULL;
  const mxArray *c3_lhs184 = NULL;
  const mxArray *c3_rhs185 = NULL;
  const mxArray *c3_lhs185 = NULL;
  const mxArray *c3_rhs186 = NULL;
  const mxArray *c3_lhs186 = NULL;
  const mxArray *c3_rhs187 = NULL;
  const mxArray *c3_lhs187 = NULL;
  const mxArray *c3_rhs188 = NULL;
  const mxArray *c3_lhs188 = NULL;
  const mxArray *c3_rhs189 = NULL;
  const mxArray *c3_lhs189 = NULL;
  const mxArray *c3_rhs190 = NULL;
  const mxArray *c3_lhs190 = NULL;
  const mxArray *c3_rhs191 = NULL;
  const mxArray *c3_lhs191 = NULL;
  sf_mex_addfield(*c3_info, c3_emlrt_marshallOut(
    "[ILXE]$matlabroot$/toolbox/coder/coder/+coder/+internal/+refblas/xswap.p"),
                  "context", "context", 128);
  sf_mex_addfield(*c3_info, c3_emlrt_marshallOut(
    "eml_int_forloop_overflow_check"), "name", "name", 128);
  sf_mex_addfield(*c3_info, c3_emlrt_marshallOut(""), "dominantType",
                  "dominantType", 128);
  sf_mex_addfield(*c3_info, c3_emlrt_marshallOut(
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/eml/eml_int_forloop_overflow_check.m"),
                  "resolved", "resolved", 128);
  sf_mex_addfield(*c3_info, c3_b_emlrt_marshallOut(1397282622U), "fileTimeLo",
                  "fileTimeLo", 128);
  sf_mex_addfield(*c3_info, c3_b_emlrt_marshallOut(0U), "fileTimeHi",
                  "fileTimeHi", 128);
  sf_mex_addfield(*c3_info, c3_b_emlrt_marshallOut(0U), "mFileTimeLo",
                  "mFileTimeLo", 128);
  sf_mex_addfield(*c3_info, c3_b_emlrt_marshallOut(0U), "mFileTimeHi",
                  "mFileTimeHi", 128);
  sf_mex_assign(&c3_rhs128, sf_mex_createcellmatrix(0, 1), false);
  sf_mex_assign(&c3_lhs128, sf_mex_createcellmatrix(0, 1), false);
  sf_mex_addfield(*c3_info, sf_mex_duplicatearraysafe(&c3_rhs128), "rhs", "rhs",
                  128);
  sf_mex_addfield(*c3_info, sf_mex_duplicatearraysafe(&c3_lhs128), "lhs", "lhs",
                  128);
  sf_mex_addfield(*c3_info, c3_emlrt_marshallOut(
    "[ILXE]$matlabroot$/toolbox/coder/coder/+coder/+internal/+refblas/xswap.p"),
                  "context", "context", 129);
  sf_mex_addfield(*c3_info, c3_emlrt_marshallOut("coder.internal.indexPlus"),
                  "name", "name", 129);
  sf_mex_addfield(*c3_info, c3_emlrt_marshallOut("coder.internal.indexInt"),
                  "dominantType", "dominantType", 129);
  sf_mex_addfield(*c3_info, c3_emlrt_marshallOut(
    "[ILXE]$matlabroot$/toolbox/shared/coder/coder/+coder/+internal/indexPlus.m"),
                  "resolved", "resolved", 129);
  sf_mex_addfield(*c3_info, c3_b_emlrt_marshallOut(1372608360U), "fileTimeLo",
                  "fileTimeLo", 129);
  sf_mex_addfield(*c3_info, c3_b_emlrt_marshallOut(0U), "fileTimeHi",
                  "fileTimeHi", 129);
  sf_mex_addfield(*c3_info, c3_b_emlrt_marshallOut(0U), "mFileTimeLo",
                  "mFileTimeLo", 129);
  sf_mex_addfield(*c3_info, c3_b_emlrt_marshallOut(0U), "mFileTimeHi",
                  "mFileTimeHi", 129);
  sf_mex_assign(&c3_rhs129, sf_mex_createcellmatrix(0, 1), false);
  sf_mex_assign(&c3_lhs129, sf_mex_createcellmatrix(0, 1), false);
  sf_mex_addfield(*c3_info, sf_mex_duplicatearraysafe(&c3_rhs129), "rhs", "rhs",
                  129);
  sf_mex_addfield(*c3_info, sf_mex_duplicatearraysafe(&c3_lhs129), "lhs", "lhs",
                  129);
  sf_mex_addfield(*c3_info, c3_emlrt_marshallOut(
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/eml/lapack/matlab/eml_matlab_zgetrf.m"),
                  "context", "context", 130);
  sf_mex_addfield(*c3_info, c3_emlrt_marshallOut("eml_div"), "name", "name", 130);
  sf_mex_addfield(*c3_info, c3_emlrt_marshallOut("double"), "dominantType",
                  "dominantType", 130);
  sf_mex_addfield(*c3_info, c3_emlrt_marshallOut(
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/eml/eml_div.m"), "resolved",
                  "resolved", 130);
  sf_mex_addfield(*c3_info, c3_b_emlrt_marshallOut(1386449152U), "fileTimeLo",
                  "fileTimeLo", 130);
  sf_mex_addfield(*c3_info, c3_b_emlrt_marshallOut(0U), "fileTimeHi",
                  "fileTimeHi", 130);
  sf_mex_addfield(*c3_info, c3_b_emlrt_marshallOut(0U), "mFileTimeLo",
                  "mFileTimeLo", 130);
  sf_mex_addfield(*c3_info, c3_b_emlrt_marshallOut(0U), "mFileTimeHi",
                  "mFileTimeHi", 130);
  sf_mex_assign(&c3_rhs130, sf_mex_createcellmatrix(0, 1), false);
  sf_mex_assign(&c3_lhs130, sf_mex_createcellmatrix(0, 1), false);
  sf_mex_addfield(*c3_info, sf_mex_duplicatearraysafe(&c3_rhs130), "rhs", "rhs",
                  130);
  sf_mex_addfield(*c3_info, sf_mex_duplicatearraysafe(&c3_lhs130), "lhs", "lhs",
                  130);
  sf_mex_addfield(*c3_info, c3_emlrt_marshallOut(
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/eml/eml_div.m"), "context",
                  "context", 131);
  sf_mex_addfield(*c3_info, c3_emlrt_marshallOut("coder.internal.div"), "name",
                  "name", 131);
  sf_mex_addfield(*c3_info, c3_emlrt_marshallOut("double"), "dominantType",
                  "dominantType", 131);
  sf_mex_addfield(*c3_info, c3_emlrt_marshallOut(
    "[ILXE]$matlabroot$/toolbox/coder/coder/+coder/+internal/div.p"), "resolved",
                  "resolved", 131);
  sf_mex_addfield(*c3_info, c3_b_emlrt_marshallOut(1410832970U), "fileTimeLo",
                  "fileTimeLo", 131);
  sf_mex_addfield(*c3_info, c3_b_emlrt_marshallOut(0U), "fileTimeHi",
                  "fileTimeHi", 131);
  sf_mex_addfield(*c3_info, c3_b_emlrt_marshallOut(0U), "mFileTimeLo",
                  "mFileTimeLo", 131);
  sf_mex_addfield(*c3_info, c3_b_emlrt_marshallOut(0U), "mFileTimeHi",
                  "mFileTimeHi", 131);
  sf_mex_assign(&c3_rhs131, sf_mex_createcellmatrix(0, 1), false);
  sf_mex_assign(&c3_lhs131, sf_mex_createcellmatrix(0, 1), false);
  sf_mex_addfield(*c3_info, sf_mex_duplicatearraysafe(&c3_rhs131), "rhs", "rhs",
                  131);
  sf_mex_addfield(*c3_info, sf_mex_duplicatearraysafe(&c3_lhs131), "lhs", "lhs",
                  131);
  sf_mex_addfield(*c3_info, c3_emlrt_marshallOut(
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/eml/lapack/matlab/eml_matlab_zgetrf.m"),
                  "context", "context", 132);
  sf_mex_addfield(*c3_info, c3_emlrt_marshallOut("eml_xgeru"), "name", "name",
                  132);
  sf_mex_addfield(*c3_info, c3_emlrt_marshallOut("double"), "dominantType",
                  "dominantType", 132);
  sf_mex_addfield(*c3_info, c3_emlrt_marshallOut(
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/eml/blas/eml_xgeru.m"),
                  "resolved", "resolved", 132);
  sf_mex_addfield(*c3_info, c3_b_emlrt_marshallOut(1376005890U), "fileTimeLo",
                  "fileTimeLo", 132);
  sf_mex_addfield(*c3_info, c3_b_emlrt_marshallOut(0U), "fileTimeHi",
                  "fileTimeHi", 132);
  sf_mex_addfield(*c3_info, c3_b_emlrt_marshallOut(0U), "mFileTimeLo",
                  "mFileTimeLo", 132);
  sf_mex_addfield(*c3_info, c3_b_emlrt_marshallOut(0U), "mFileTimeHi",
                  "mFileTimeHi", 132);
  sf_mex_assign(&c3_rhs132, sf_mex_createcellmatrix(0, 1), false);
  sf_mex_assign(&c3_lhs132, sf_mex_createcellmatrix(0, 1), false);
  sf_mex_addfield(*c3_info, sf_mex_duplicatearraysafe(&c3_rhs132), "rhs", "rhs",
                  132);
  sf_mex_addfield(*c3_info, sf_mex_duplicatearraysafe(&c3_lhs132), "lhs", "lhs",
                  132);
  sf_mex_addfield(*c3_info, c3_emlrt_marshallOut(
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/eml/blas/eml_xgeru.m"), "context",
                  "context", 133);
  sf_mex_addfield(*c3_info, c3_emlrt_marshallOut("coder.internal.blas.inline"),
                  "name", "name", 133);
  sf_mex_addfield(*c3_info, c3_emlrt_marshallOut(""), "dominantType",
                  "dominantType", 133);
  sf_mex_addfield(*c3_info, c3_emlrt_marshallOut(
    "[ILXE]$matlabroot$/toolbox/coder/coder/+coder/+internal/+blas/inline.p"),
                  "resolved", "resolved", 133);
  sf_mex_addfield(*c3_info, c3_b_emlrt_marshallOut(1410832972U), "fileTimeLo",
                  "fileTimeLo", 133);
  sf_mex_addfield(*c3_info, c3_b_emlrt_marshallOut(0U), "fileTimeHi",
                  "fileTimeHi", 133);
  sf_mex_addfield(*c3_info, c3_b_emlrt_marshallOut(0U), "mFileTimeLo",
                  "mFileTimeLo", 133);
  sf_mex_addfield(*c3_info, c3_b_emlrt_marshallOut(0U), "mFileTimeHi",
                  "mFileTimeHi", 133);
  sf_mex_assign(&c3_rhs133, sf_mex_createcellmatrix(0, 1), false);
  sf_mex_assign(&c3_lhs133, sf_mex_createcellmatrix(0, 1), false);
  sf_mex_addfield(*c3_info, sf_mex_duplicatearraysafe(&c3_rhs133), "rhs", "rhs",
                  133);
  sf_mex_addfield(*c3_info, sf_mex_duplicatearraysafe(&c3_lhs133), "lhs", "lhs",
                  133);
  sf_mex_addfield(*c3_info, c3_emlrt_marshallOut(
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/eml/blas/eml_xgeru.m"), "context",
                  "context", 134);
  sf_mex_addfield(*c3_info, c3_emlrt_marshallOut("coder.internal.blas.xgeru"),
                  "name", "name", 134);
  sf_mex_addfield(*c3_info, c3_emlrt_marshallOut("double"), "dominantType",
                  "dominantType", 134);
  sf_mex_addfield(*c3_info, c3_emlrt_marshallOut(
    "[ILXE]$matlabroot$/toolbox/coder/coder/+coder/+internal/+blas/xgeru.p"),
                  "resolved", "resolved", 134);
  sf_mex_addfield(*c3_info, c3_b_emlrt_marshallOut(1410832972U), "fileTimeLo",
                  "fileTimeLo", 134);
  sf_mex_addfield(*c3_info, c3_b_emlrt_marshallOut(0U), "fileTimeHi",
                  "fileTimeHi", 134);
  sf_mex_addfield(*c3_info, c3_b_emlrt_marshallOut(0U), "mFileTimeLo",
                  "mFileTimeLo", 134);
  sf_mex_addfield(*c3_info, c3_b_emlrt_marshallOut(0U), "mFileTimeHi",
                  "mFileTimeHi", 134);
  sf_mex_assign(&c3_rhs134, sf_mex_createcellmatrix(0, 1), false);
  sf_mex_assign(&c3_lhs134, sf_mex_createcellmatrix(0, 1), false);
  sf_mex_addfield(*c3_info, sf_mex_duplicatearraysafe(&c3_rhs134), "rhs", "rhs",
                  134);
  sf_mex_addfield(*c3_info, sf_mex_duplicatearraysafe(&c3_lhs134), "lhs", "lhs",
                  134);
  sf_mex_addfield(*c3_info, c3_emlrt_marshallOut(
    "[ILXE]$matlabroot$/toolbox/coder/coder/+coder/+internal/+blas/xgeru.p"),
                  "context", "context", 135);
  sf_mex_addfield(*c3_info, c3_emlrt_marshallOut("coder.internal.blas.xger"),
                  "name", "name", 135);
  sf_mex_addfield(*c3_info, c3_emlrt_marshallOut("double"), "dominantType",
                  "dominantType", 135);
  sf_mex_addfield(*c3_info, c3_emlrt_marshallOut(
    "[ILXE]$matlabroot$/toolbox/coder/coder/+coder/+internal/+blas/xger.p"),
                  "resolved", "resolved", 135);
  sf_mex_addfield(*c3_info, c3_b_emlrt_marshallOut(1410832970U), "fileTimeLo",
                  "fileTimeLo", 135);
  sf_mex_addfield(*c3_info, c3_b_emlrt_marshallOut(0U), "fileTimeHi",
                  "fileTimeHi", 135);
  sf_mex_addfield(*c3_info, c3_b_emlrt_marshallOut(0U), "mFileTimeLo",
                  "mFileTimeLo", 135);
  sf_mex_addfield(*c3_info, c3_b_emlrt_marshallOut(0U), "mFileTimeHi",
                  "mFileTimeHi", 135);
  sf_mex_assign(&c3_rhs135, sf_mex_createcellmatrix(0, 1), false);
  sf_mex_assign(&c3_lhs135, sf_mex_createcellmatrix(0, 1), false);
  sf_mex_addfield(*c3_info, sf_mex_duplicatearraysafe(&c3_rhs135), "rhs", "rhs",
                  135);
  sf_mex_addfield(*c3_info, sf_mex_duplicatearraysafe(&c3_lhs135), "lhs", "lhs",
                  135);
  sf_mex_addfield(*c3_info, c3_emlrt_marshallOut(
    "[ILXE]$matlabroot$/toolbox/coder/coder/+coder/+internal/+blas/xger.p"),
                  "context", "context", 136);
  sf_mex_addfield(*c3_info, c3_emlrt_marshallOut(
    "coder.internal.blas.use_refblas"), "name", "name", 136);
  sf_mex_addfield(*c3_info, c3_emlrt_marshallOut(""), "dominantType",
                  "dominantType", 136);
  sf_mex_addfield(*c3_info, c3_emlrt_marshallOut(
    "[ILXE]$matlabroot$/toolbox/coder/coder/+coder/+internal/+blas/use_refblas.p"),
                  "resolved", "resolved", 136);
  sf_mex_addfield(*c3_info, c3_b_emlrt_marshallOut(1410832970U), "fileTimeLo",
                  "fileTimeLo", 136);
  sf_mex_addfield(*c3_info, c3_b_emlrt_marshallOut(0U), "fileTimeHi",
                  "fileTimeHi", 136);
  sf_mex_addfield(*c3_info, c3_b_emlrt_marshallOut(0U), "mFileTimeLo",
                  "mFileTimeLo", 136);
  sf_mex_addfield(*c3_info, c3_b_emlrt_marshallOut(0U), "mFileTimeHi",
                  "mFileTimeHi", 136);
  sf_mex_assign(&c3_rhs136, sf_mex_createcellmatrix(0, 1), false);
  sf_mex_assign(&c3_lhs136, sf_mex_createcellmatrix(0, 1), false);
  sf_mex_addfield(*c3_info, sf_mex_duplicatearraysafe(&c3_rhs136), "rhs", "rhs",
                  136);
  sf_mex_addfield(*c3_info, sf_mex_duplicatearraysafe(&c3_lhs136), "lhs", "lhs",
                  136);
  sf_mex_addfield(*c3_info, c3_emlrt_marshallOut(
    "[ILXE]$matlabroot$/toolbox/coder/coder/+coder/+internal/+blas/xger.p!below_threshold"),
                  "context", "context", 137);
  sf_mex_addfield(*c3_info, c3_emlrt_marshallOut("coder.internal.blas.threshold"),
                  "name", "name", 137);
  sf_mex_addfield(*c3_info, c3_emlrt_marshallOut("char"), "dominantType",
                  "dominantType", 137);
  sf_mex_addfield(*c3_info, c3_emlrt_marshallOut(
    "[ILXE]$matlabroot$/toolbox/coder/coder/+coder/+internal/+blas/threshold.p"),
                  "resolved", "resolved", 137);
  sf_mex_addfield(*c3_info, c3_b_emlrt_marshallOut(1410832972U), "fileTimeLo",
                  "fileTimeLo", 137);
  sf_mex_addfield(*c3_info, c3_b_emlrt_marshallOut(0U), "fileTimeHi",
                  "fileTimeHi", 137);
  sf_mex_addfield(*c3_info, c3_b_emlrt_marshallOut(0U), "mFileTimeLo",
                  "mFileTimeLo", 137);
  sf_mex_addfield(*c3_info, c3_b_emlrt_marshallOut(0U), "mFileTimeHi",
                  "mFileTimeHi", 137);
  sf_mex_assign(&c3_rhs137, sf_mex_createcellmatrix(0, 1), false);
  sf_mex_assign(&c3_lhs137, sf_mex_createcellmatrix(0, 1), false);
  sf_mex_addfield(*c3_info, sf_mex_duplicatearraysafe(&c3_rhs137), "rhs", "rhs",
                  137);
  sf_mex_addfield(*c3_info, sf_mex_duplicatearraysafe(&c3_lhs137), "lhs", "lhs",
                  137);
  sf_mex_addfield(*c3_info, c3_emlrt_marshallOut(
    "[ILXE]$matlabroot$/toolbox/coder/coder/+coder/+internal/+blas/xger.p!below_threshold"),
                  "context", "context", 138);
  sf_mex_addfield(*c3_info, c3_emlrt_marshallOut("coder.internal.blas.int"),
                  "name", "name", 138);
  sf_mex_addfield(*c3_info, c3_emlrt_marshallOut(""), "dominantType",
                  "dominantType", 138);
  sf_mex_addfield(*c3_info, c3_emlrt_marshallOut(
    "[ILXE]$matlabroot$/toolbox/coder/coder/+coder/+internal/+blas/int.p"),
                  "resolved", "resolved", 138);
  sf_mex_addfield(*c3_info, c3_b_emlrt_marshallOut(1410832972U), "fileTimeLo",
                  "fileTimeLo", 138);
  sf_mex_addfield(*c3_info, c3_b_emlrt_marshallOut(0U), "fileTimeHi",
                  "fileTimeHi", 138);
  sf_mex_addfield(*c3_info, c3_b_emlrt_marshallOut(0U), "mFileTimeLo",
                  "mFileTimeLo", 138);
  sf_mex_addfield(*c3_info, c3_b_emlrt_marshallOut(0U), "mFileTimeHi",
                  "mFileTimeHi", 138);
  sf_mex_assign(&c3_rhs138, sf_mex_createcellmatrix(0, 1), false);
  sf_mex_assign(&c3_lhs138, sf_mex_createcellmatrix(0, 1), false);
  sf_mex_addfield(*c3_info, sf_mex_duplicatearraysafe(&c3_rhs138), "rhs", "rhs",
                  138);
  sf_mex_addfield(*c3_info, sf_mex_duplicatearraysafe(&c3_lhs138), "lhs", "lhs",
                  138);
  sf_mex_addfield(*c3_info, c3_emlrt_marshallOut(
    "[ILXE]$matlabroot$/toolbox/coder/coder/+coder/+internal/+blas/xger.p!below_threshold"),
                  "context", "context", 139);
  sf_mex_addfield(*c3_info, c3_emlrt_marshallOut("intmax"), "name", "name", 139);
  sf_mex_addfield(*c3_info, c3_emlrt_marshallOut("char"), "dominantType",
                  "dominantType", 139);
  sf_mex_addfield(*c3_info, c3_emlrt_marshallOut(
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/elmat/intmax.m"), "resolved",
                  "resolved", 139);
  sf_mex_addfield(*c3_info, c3_b_emlrt_marshallOut(1362287082U), "fileTimeLo",
                  "fileTimeLo", 139);
  sf_mex_addfield(*c3_info, c3_b_emlrt_marshallOut(0U), "fileTimeHi",
                  "fileTimeHi", 139);
  sf_mex_addfield(*c3_info, c3_b_emlrt_marshallOut(0U), "mFileTimeLo",
                  "mFileTimeLo", 139);
  sf_mex_addfield(*c3_info, c3_b_emlrt_marshallOut(0U), "mFileTimeHi",
                  "mFileTimeHi", 139);
  sf_mex_assign(&c3_rhs139, sf_mex_createcellmatrix(0, 1), false);
  sf_mex_assign(&c3_lhs139, sf_mex_createcellmatrix(0, 1), false);
  sf_mex_addfield(*c3_info, sf_mex_duplicatearraysafe(&c3_rhs139), "rhs", "rhs",
                  139);
  sf_mex_addfield(*c3_info, sf_mex_duplicatearraysafe(&c3_lhs139), "lhs", "lhs",
                  139);
  sf_mex_addfield(*c3_info, c3_emlrt_marshallOut(
    "[ILXE]$matlabroot$/toolbox/coder/coder/+coder/+internal/+blas/xger.p!below_threshold"),
                  "context", "context", 140);
  sf_mex_addfield(*c3_info, c3_emlrt_marshallOut("min"), "name", "name", 140);
  sf_mex_addfield(*c3_info, c3_emlrt_marshallOut("double"), "dominantType",
                  "dominantType", 140);
  sf_mex_addfield(*c3_info, c3_emlrt_marshallOut(
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/datafun/min.m"), "resolved",
                  "resolved", 140);
  sf_mex_addfield(*c3_info, c3_b_emlrt_marshallOut(1311280518U), "fileTimeLo",
                  "fileTimeLo", 140);
  sf_mex_addfield(*c3_info, c3_b_emlrt_marshallOut(0U), "fileTimeHi",
                  "fileTimeHi", 140);
  sf_mex_addfield(*c3_info, c3_b_emlrt_marshallOut(0U), "mFileTimeLo",
                  "mFileTimeLo", 140);
  sf_mex_addfield(*c3_info, c3_b_emlrt_marshallOut(0U), "mFileTimeHi",
                  "mFileTimeHi", 140);
  sf_mex_assign(&c3_rhs140, sf_mex_createcellmatrix(0, 1), false);
  sf_mex_assign(&c3_lhs140, sf_mex_createcellmatrix(0, 1), false);
  sf_mex_addfield(*c3_info, sf_mex_duplicatearraysafe(&c3_rhs140), "rhs", "rhs",
                  140);
  sf_mex_addfield(*c3_info, sf_mex_duplicatearraysafe(&c3_lhs140), "lhs", "lhs",
                  140);
  sf_mex_addfield(*c3_info, c3_emlrt_marshallOut(
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/eml/eml_min_or_max.m!eml_bin_extremum"),
                  "context", "context", 141);
  sf_mex_addfield(*c3_info, c3_emlrt_marshallOut("eml_scalar_eg"), "name",
                  "name", 141);
  sf_mex_addfield(*c3_info, c3_emlrt_marshallOut("double"), "dominantType",
                  "dominantType", 141);
  sf_mex_addfield(*c3_info, c3_emlrt_marshallOut(
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/eml/eml_scalar_eg.m"), "resolved",
                  "resolved", 141);
  sf_mex_addfield(*c3_info, c3_b_emlrt_marshallOut(1376005888U), "fileTimeLo",
                  "fileTimeLo", 141);
  sf_mex_addfield(*c3_info, c3_b_emlrt_marshallOut(0U), "fileTimeHi",
                  "fileTimeHi", 141);
  sf_mex_addfield(*c3_info, c3_b_emlrt_marshallOut(0U), "mFileTimeLo",
                  "mFileTimeLo", 141);
  sf_mex_addfield(*c3_info, c3_b_emlrt_marshallOut(0U), "mFileTimeHi",
                  "mFileTimeHi", 141);
  sf_mex_assign(&c3_rhs141, sf_mex_createcellmatrix(0, 1), false);
  sf_mex_assign(&c3_lhs141, sf_mex_createcellmatrix(0, 1), false);
  sf_mex_addfield(*c3_info, sf_mex_duplicatearraysafe(&c3_rhs141), "rhs", "rhs",
                  141);
  sf_mex_addfield(*c3_info, sf_mex_duplicatearraysafe(&c3_lhs141), "lhs", "lhs",
                  141);
  sf_mex_addfield(*c3_info, c3_emlrt_marshallOut(
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/eml/eml_min_or_max.m!eml_bin_extremum"),
                  "context", "context", 142);
  sf_mex_addfield(*c3_info, c3_emlrt_marshallOut("eml_scalexp_alloc"), "name",
                  "name", 142);
  sf_mex_addfield(*c3_info, c3_emlrt_marshallOut("double"), "dominantType",
                  "dominantType", 142);
  sf_mex_addfield(*c3_info, c3_emlrt_marshallOut(
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/eml/eml_scalexp_alloc.m"),
                  "resolved", "resolved", 142);
  sf_mex_addfield(*c3_info, c3_b_emlrt_marshallOut(1376005888U), "fileTimeLo",
                  "fileTimeLo", 142);
  sf_mex_addfield(*c3_info, c3_b_emlrt_marshallOut(0U), "fileTimeHi",
                  "fileTimeHi", 142);
  sf_mex_addfield(*c3_info, c3_b_emlrt_marshallOut(0U), "mFileTimeLo",
                  "mFileTimeLo", 142);
  sf_mex_addfield(*c3_info, c3_b_emlrt_marshallOut(0U), "mFileTimeHi",
                  "mFileTimeHi", 142);
  sf_mex_assign(&c3_rhs142, sf_mex_createcellmatrix(0, 1), false);
  sf_mex_assign(&c3_lhs142, sf_mex_createcellmatrix(0, 1), false);
  sf_mex_addfield(*c3_info, sf_mex_duplicatearraysafe(&c3_rhs142), "rhs", "rhs",
                  142);
  sf_mex_addfield(*c3_info, sf_mex_duplicatearraysafe(&c3_lhs142), "lhs", "lhs",
                  142);
  sf_mex_addfield(*c3_info, c3_emlrt_marshallOut(
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/eml/eml_scalexp_alloc.m"),
                  "context", "context", 143);
  sf_mex_addfield(*c3_info, c3_emlrt_marshallOut("coder.internal.scalexpAlloc"),
                  "name", "name", 143);
  sf_mex_addfield(*c3_info, c3_emlrt_marshallOut("double"), "dominantType",
                  "dominantType", 143);
  sf_mex_addfield(*c3_info, c3_emlrt_marshallOut(
    "[ILXE]$matlabroot$/toolbox/coder/coder/+coder/+internal/scalexpAlloc.p"),
                  "resolved", "resolved", 143);
  sf_mex_addfield(*c3_info, c3_b_emlrt_marshallOut(1410832970U), "fileTimeLo",
                  "fileTimeLo", 143);
  sf_mex_addfield(*c3_info, c3_b_emlrt_marshallOut(0U), "fileTimeHi",
                  "fileTimeHi", 143);
  sf_mex_addfield(*c3_info, c3_b_emlrt_marshallOut(0U), "mFileTimeLo",
                  "mFileTimeLo", 143);
  sf_mex_addfield(*c3_info, c3_b_emlrt_marshallOut(0U), "mFileTimeHi",
                  "mFileTimeHi", 143);
  sf_mex_assign(&c3_rhs143, sf_mex_createcellmatrix(0, 1), false);
  sf_mex_assign(&c3_lhs143, sf_mex_createcellmatrix(0, 1), false);
  sf_mex_addfield(*c3_info, sf_mex_duplicatearraysafe(&c3_rhs143), "rhs", "rhs",
                  143);
  sf_mex_addfield(*c3_info, sf_mex_duplicatearraysafe(&c3_lhs143), "lhs", "lhs",
                  143);
  sf_mex_addfield(*c3_info, c3_emlrt_marshallOut(
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/eml/eml_min_or_max.m!eml_scalar_bin_extremum"),
                  "context", "context", 144);
  sf_mex_addfield(*c3_info, c3_emlrt_marshallOut("eml_scalar_eg"), "name",
                  "name", 144);
  sf_mex_addfield(*c3_info, c3_emlrt_marshallOut("double"), "dominantType",
                  "dominantType", 144);
  sf_mex_addfield(*c3_info, c3_emlrt_marshallOut(
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/eml/eml_scalar_eg.m"), "resolved",
                  "resolved", 144);
  sf_mex_addfield(*c3_info, c3_b_emlrt_marshallOut(1376005888U), "fileTimeLo",
                  "fileTimeLo", 144);
  sf_mex_addfield(*c3_info, c3_b_emlrt_marshallOut(0U), "fileTimeHi",
                  "fileTimeHi", 144);
  sf_mex_addfield(*c3_info, c3_b_emlrt_marshallOut(0U), "mFileTimeLo",
                  "mFileTimeLo", 144);
  sf_mex_addfield(*c3_info, c3_b_emlrt_marshallOut(0U), "mFileTimeHi",
                  "mFileTimeHi", 144);
  sf_mex_assign(&c3_rhs144, sf_mex_createcellmatrix(0, 1), false);
  sf_mex_assign(&c3_lhs144, sf_mex_createcellmatrix(0, 1), false);
  sf_mex_addfield(*c3_info, sf_mex_duplicatearraysafe(&c3_rhs144), "rhs", "rhs",
                  144);
  sf_mex_addfield(*c3_info, sf_mex_duplicatearraysafe(&c3_lhs144), "lhs", "lhs",
                  144);
  sf_mex_addfield(*c3_info, c3_emlrt_marshallOut(
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/eml/eml_min_or_max.m!eml_scalar_bin_extremum"),
                  "context", "context", 145);
  sf_mex_addfield(*c3_info, c3_emlrt_marshallOut(
    "coder.internal.isBuiltInNumeric"), "name", "name", 145);
  sf_mex_addfield(*c3_info, c3_emlrt_marshallOut("double"), "dominantType",
                  "dominantType", 145);
  sf_mex_addfield(*c3_info, c3_emlrt_marshallOut(
    "[ILXE]$matlabroot$/toolbox/shared/coder/coder/+coder/+internal/isBuiltInNumeric.m"),
                  "resolved", "resolved", 145);
  sf_mex_addfield(*c3_info, c3_b_emlrt_marshallOut(1395953456U), "fileTimeLo",
                  "fileTimeLo", 145);
  sf_mex_addfield(*c3_info, c3_b_emlrt_marshallOut(0U), "fileTimeHi",
                  "fileTimeHi", 145);
  sf_mex_addfield(*c3_info, c3_b_emlrt_marshallOut(0U), "mFileTimeLo",
                  "mFileTimeLo", 145);
  sf_mex_addfield(*c3_info, c3_b_emlrt_marshallOut(0U), "mFileTimeHi",
                  "mFileTimeHi", 145);
  sf_mex_assign(&c3_rhs145, sf_mex_createcellmatrix(0, 1), false);
  sf_mex_assign(&c3_lhs145, sf_mex_createcellmatrix(0, 1), false);
  sf_mex_addfield(*c3_info, sf_mex_duplicatearraysafe(&c3_rhs145), "rhs", "rhs",
                  145);
  sf_mex_addfield(*c3_info, sf_mex_duplicatearraysafe(&c3_lhs145), "lhs", "lhs",
                  145);
  sf_mex_addfield(*c3_info, c3_emlrt_marshallOut(
    "[ILXE]$matlabroot$/toolbox/coder/coder/+coder/+internal/+blas/xger.p"),
                  "context", "context", 146);
  sf_mex_addfield(*c3_info, c3_emlrt_marshallOut("coder.internal.refblas.xger"),
                  "name", "name", 146);
  sf_mex_addfield(*c3_info, c3_emlrt_marshallOut("double"), "dominantType",
                  "dominantType", 146);
  sf_mex_addfield(*c3_info, c3_emlrt_marshallOut(
    "[ILXE]$matlabroot$/toolbox/coder/coder/+coder/+internal/+refblas/xger.p"),
                  "resolved", "resolved", 146);
  sf_mex_addfield(*c3_info, c3_b_emlrt_marshallOut(1410832972U), "fileTimeLo",
                  "fileTimeLo", 146);
  sf_mex_addfield(*c3_info, c3_b_emlrt_marshallOut(0U), "fileTimeHi",
                  "fileTimeHi", 146);
  sf_mex_addfield(*c3_info, c3_b_emlrt_marshallOut(0U), "mFileTimeLo",
                  "mFileTimeLo", 146);
  sf_mex_addfield(*c3_info, c3_b_emlrt_marshallOut(0U), "mFileTimeHi",
                  "mFileTimeHi", 146);
  sf_mex_assign(&c3_rhs146, sf_mex_createcellmatrix(0, 1), false);
  sf_mex_assign(&c3_lhs146, sf_mex_createcellmatrix(0, 1), false);
  sf_mex_addfield(*c3_info, sf_mex_duplicatearraysafe(&c3_rhs146), "rhs", "rhs",
                  146);
  sf_mex_addfield(*c3_info, sf_mex_duplicatearraysafe(&c3_lhs146), "lhs", "lhs",
                  146);
  sf_mex_addfield(*c3_info, c3_emlrt_marshallOut(
    "[ILXE]$matlabroot$/toolbox/coder/coder/+coder/+internal/+refblas/xger.p"),
                  "context", "context", 147);
  sf_mex_addfield(*c3_info, c3_emlrt_marshallOut("coder.internal.refblas.xgerx"),
                  "name", "name", 147);
  sf_mex_addfield(*c3_info, c3_emlrt_marshallOut("char"), "dominantType",
                  "dominantType", 147);
  sf_mex_addfield(*c3_info, c3_emlrt_marshallOut(
    "[ILXE]$matlabroot$/toolbox/coder/coder/+coder/+internal/+refblas/xgerx.p"),
                  "resolved", "resolved", 147);
  sf_mex_addfield(*c3_info, c3_b_emlrt_marshallOut(1410832972U), "fileTimeLo",
                  "fileTimeLo", 147);
  sf_mex_addfield(*c3_info, c3_b_emlrt_marshallOut(0U), "fileTimeHi",
                  "fileTimeHi", 147);
  sf_mex_addfield(*c3_info, c3_b_emlrt_marshallOut(0U), "mFileTimeLo",
                  "mFileTimeLo", 147);
  sf_mex_addfield(*c3_info, c3_b_emlrt_marshallOut(0U), "mFileTimeHi",
                  "mFileTimeHi", 147);
  sf_mex_assign(&c3_rhs147, sf_mex_createcellmatrix(0, 1), false);
  sf_mex_assign(&c3_lhs147, sf_mex_createcellmatrix(0, 1), false);
  sf_mex_addfield(*c3_info, sf_mex_duplicatearraysafe(&c3_rhs147), "rhs", "rhs",
                  147);
  sf_mex_addfield(*c3_info, sf_mex_duplicatearraysafe(&c3_lhs147), "lhs", "lhs",
                  147);
  sf_mex_addfield(*c3_info, c3_emlrt_marshallOut(
    "[ILXE]$matlabroot$/toolbox/coder/coder/+coder/+internal/+refblas/xgerx.p"),
                  "context", "context", 148);
  sf_mex_addfield(*c3_info, c3_emlrt_marshallOut("abs"), "name", "name", 148);
  sf_mex_addfield(*c3_info, c3_emlrt_marshallOut("coder.internal.indexInt"),
                  "dominantType", "dominantType", 148);
  sf_mex_addfield(*c3_info, c3_emlrt_marshallOut(
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/elfun/abs.m"), "resolved",
                  "resolved", 148);
  sf_mex_addfield(*c3_info, c3_b_emlrt_marshallOut(1363735452U), "fileTimeLo",
                  "fileTimeLo", 148);
  sf_mex_addfield(*c3_info, c3_b_emlrt_marshallOut(0U), "fileTimeHi",
                  "fileTimeHi", 148);
  sf_mex_addfield(*c3_info, c3_b_emlrt_marshallOut(0U), "mFileTimeLo",
                  "mFileTimeLo", 148);
  sf_mex_addfield(*c3_info, c3_b_emlrt_marshallOut(0U), "mFileTimeHi",
                  "mFileTimeHi", 148);
  sf_mex_assign(&c3_rhs148, sf_mex_createcellmatrix(0, 1), false);
  sf_mex_assign(&c3_lhs148, sf_mex_createcellmatrix(0, 1), false);
  sf_mex_addfield(*c3_info, sf_mex_duplicatearraysafe(&c3_rhs148), "rhs", "rhs",
                  148);
  sf_mex_addfield(*c3_info, sf_mex_duplicatearraysafe(&c3_lhs148), "lhs", "lhs",
                  148);
  sf_mex_addfield(*c3_info, c3_emlrt_marshallOut(
    "[ILXE]$matlabroot$/toolbox/coder/coder/+coder/+internal/+refblas/xgerx.p"),
                  "context", "context", 149);
  sf_mex_addfield(*c3_info, c3_emlrt_marshallOut("coder.internal.indexMinus"),
                  "name", "name", 149);
  sf_mex_addfield(*c3_info, c3_emlrt_marshallOut("double"), "dominantType",
                  "dominantType", 149);
  sf_mex_addfield(*c3_info, c3_emlrt_marshallOut(
    "[ILXE]$matlabroot$/toolbox/shared/coder/coder/+coder/+internal/indexMinus.m"),
                  "resolved", "resolved", 149);
  sf_mex_addfield(*c3_info, c3_b_emlrt_marshallOut(1372608360U), "fileTimeLo",
                  "fileTimeLo", 149);
  sf_mex_addfield(*c3_info, c3_b_emlrt_marshallOut(0U), "fileTimeHi",
                  "fileTimeHi", 149);
  sf_mex_addfield(*c3_info, c3_b_emlrt_marshallOut(0U), "mFileTimeLo",
                  "mFileTimeLo", 149);
  sf_mex_addfield(*c3_info, c3_b_emlrt_marshallOut(0U), "mFileTimeHi",
                  "mFileTimeHi", 149);
  sf_mex_assign(&c3_rhs149, sf_mex_createcellmatrix(0, 1), false);
  sf_mex_assign(&c3_lhs149, sf_mex_createcellmatrix(0, 1), false);
  sf_mex_addfield(*c3_info, sf_mex_duplicatearraysafe(&c3_rhs149), "rhs", "rhs",
                  149);
  sf_mex_addfield(*c3_info, sf_mex_duplicatearraysafe(&c3_lhs149), "lhs", "lhs",
                  149);
  sf_mex_addfield(*c3_info, c3_emlrt_marshallOut(
    "[ILXE]$matlabroot$/toolbox/coder/coder/+coder/+internal/+refblas/xgerx.p"),
                  "context", "context", 150);
  sf_mex_addfield(*c3_info, c3_emlrt_marshallOut(
    "eml_int_forloop_overflow_check"), "name", "name", 150);
  sf_mex_addfield(*c3_info, c3_emlrt_marshallOut(""), "dominantType",
                  "dominantType", 150);
  sf_mex_addfield(*c3_info, c3_emlrt_marshallOut(
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/eml/eml_int_forloop_overflow_check.m"),
                  "resolved", "resolved", 150);
  sf_mex_addfield(*c3_info, c3_b_emlrt_marshallOut(1397282622U), "fileTimeLo",
                  "fileTimeLo", 150);
  sf_mex_addfield(*c3_info, c3_b_emlrt_marshallOut(0U), "fileTimeHi",
                  "fileTimeHi", 150);
  sf_mex_addfield(*c3_info, c3_b_emlrt_marshallOut(0U), "mFileTimeLo",
                  "mFileTimeLo", 150);
  sf_mex_addfield(*c3_info, c3_b_emlrt_marshallOut(0U), "mFileTimeHi",
                  "mFileTimeHi", 150);
  sf_mex_assign(&c3_rhs150, sf_mex_createcellmatrix(0, 1), false);
  sf_mex_assign(&c3_lhs150, sf_mex_createcellmatrix(0, 1), false);
  sf_mex_addfield(*c3_info, sf_mex_duplicatearraysafe(&c3_rhs150), "rhs", "rhs",
                  150);
  sf_mex_addfield(*c3_info, sf_mex_duplicatearraysafe(&c3_lhs150), "lhs", "lhs",
                  150);
  sf_mex_addfield(*c3_info, c3_emlrt_marshallOut(
    "[ILXE]$matlabroot$/toolbox/coder/coder/+coder/+internal/+refblas/xgerx.p"),
                  "context", "context", 151);
  sf_mex_addfield(*c3_info, c3_emlrt_marshallOut("coder.internal.indexPlus"),
                  "name", "name", 151);
  sf_mex_addfield(*c3_info, c3_emlrt_marshallOut("double"), "dominantType",
                  "dominantType", 151);
  sf_mex_addfield(*c3_info, c3_emlrt_marshallOut(
    "[ILXE]$matlabroot$/toolbox/shared/coder/coder/+coder/+internal/indexPlus.m"),
                  "resolved", "resolved", 151);
  sf_mex_addfield(*c3_info, c3_b_emlrt_marshallOut(1372608360U), "fileTimeLo",
                  "fileTimeLo", 151);
  sf_mex_addfield(*c3_info, c3_b_emlrt_marshallOut(0U), "fileTimeHi",
                  "fileTimeHi", 151);
  sf_mex_addfield(*c3_info, c3_b_emlrt_marshallOut(0U), "mFileTimeLo",
                  "mFileTimeLo", 151);
  sf_mex_addfield(*c3_info, c3_b_emlrt_marshallOut(0U), "mFileTimeHi",
                  "mFileTimeHi", 151);
  sf_mex_assign(&c3_rhs151, sf_mex_createcellmatrix(0, 1), false);
  sf_mex_assign(&c3_lhs151, sf_mex_createcellmatrix(0, 1), false);
  sf_mex_addfield(*c3_info, sf_mex_duplicatearraysafe(&c3_rhs151), "rhs", "rhs",
                  151);
  sf_mex_addfield(*c3_info, sf_mex_duplicatearraysafe(&c3_lhs151), "lhs", "lhs",
                  151);
  sf_mex_addfield(*c3_info, c3_emlrt_marshallOut(
    "[ILXE]$matlabroot$/toolbox/coder/coder/+coder/+internal/+refblas/xgerx.p"),
                  "context", "context", 152);
  sf_mex_addfield(*c3_info, c3_emlrt_marshallOut("coder.internal.indexPlus"),
                  "name", "name", 152);
  sf_mex_addfield(*c3_info, c3_emlrt_marshallOut("coder.internal.indexInt"),
                  "dominantType", "dominantType", 152);
  sf_mex_addfield(*c3_info, c3_emlrt_marshallOut(
    "[ILXE]$matlabroot$/toolbox/shared/coder/coder/+coder/+internal/indexPlus.m"),
                  "resolved", "resolved", 152);
  sf_mex_addfield(*c3_info, c3_b_emlrt_marshallOut(1372608360U), "fileTimeLo",
                  "fileTimeLo", 152);
  sf_mex_addfield(*c3_info, c3_b_emlrt_marshallOut(0U), "fileTimeHi",
                  "fileTimeHi", 152);
  sf_mex_addfield(*c3_info, c3_b_emlrt_marshallOut(0U), "mFileTimeLo",
                  "mFileTimeLo", 152);
  sf_mex_addfield(*c3_info, c3_b_emlrt_marshallOut(0U), "mFileTimeHi",
                  "mFileTimeHi", 152);
  sf_mex_assign(&c3_rhs152, sf_mex_createcellmatrix(0, 1), false);
  sf_mex_assign(&c3_lhs152, sf_mex_createcellmatrix(0, 1), false);
  sf_mex_addfield(*c3_info, sf_mex_duplicatearraysafe(&c3_rhs152), "rhs", "rhs",
                  152);
  sf_mex_addfield(*c3_info, sf_mex_duplicatearraysafe(&c3_lhs152), "lhs", "lhs",
                  152);
  sf_mex_addfield(*c3_info, c3_emlrt_marshallOut(
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/matfun/inv.m!invNxN"), "context",
                  "context", 153);
  sf_mex_addfield(*c3_info, c3_emlrt_marshallOut("eml_ipiv2perm"), "name",
                  "name", 153);
  sf_mex_addfield(*c3_info, c3_emlrt_marshallOut("coder.internal.indexInt"),
                  "dominantType", "dominantType", 153);
  sf_mex_addfield(*c3_info, c3_emlrt_marshallOut(
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/eml/eml_ipiv2perm.m"), "resolved",
                  "resolved", 153);
  sf_mex_addfield(*c3_info, c3_b_emlrt_marshallOut(1286843982U), "fileTimeLo",
                  "fileTimeLo", 153);
  sf_mex_addfield(*c3_info, c3_b_emlrt_marshallOut(0U), "fileTimeHi",
                  "fileTimeHi", 153);
  sf_mex_addfield(*c3_info, c3_b_emlrt_marshallOut(0U), "mFileTimeLo",
                  "mFileTimeLo", 153);
  sf_mex_addfield(*c3_info, c3_b_emlrt_marshallOut(0U), "mFileTimeHi",
                  "mFileTimeHi", 153);
  sf_mex_assign(&c3_rhs153, sf_mex_createcellmatrix(0, 1), false);
  sf_mex_assign(&c3_lhs153, sf_mex_createcellmatrix(0, 1), false);
  sf_mex_addfield(*c3_info, sf_mex_duplicatearraysafe(&c3_rhs153), "rhs", "rhs",
                  153);
  sf_mex_addfield(*c3_info, sf_mex_duplicatearraysafe(&c3_lhs153), "lhs", "lhs",
                  153);
  sf_mex_addfield(*c3_info, c3_emlrt_marshallOut(
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/eml/eml_ipiv2perm.m"), "context",
                  "context", 154);
  sf_mex_addfield(*c3_info, c3_emlrt_marshallOut("colon"), "name", "name", 154);
  sf_mex_addfield(*c3_info, c3_emlrt_marshallOut("double"), "dominantType",
                  "dominantType", 154);
  sf_mex_addfield(*c3_info, c3_emlrt_marshallOut(
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/ops/colon.m"), "resolved",
                  "resolved", 154);
  sf_mex_addfield(*c3_info, c3_b_emlrt_marshallOut(1378321188U), "fileTimeLo",
                  "fileTimeLo", 154);
  sf_mex_addfield(*c3_info, c3_b_emlrt_marshallOut(0U), "fileTimeHi",
                  "fileTimeHi", 154);
  sf_mex_addfield(*c3_info, c3_b_emlrt_marshallOut(0U), "mFileTimeLo",
                  "mFileTimeLo", 154);
  sf_mex_addfield(*c3_info, c3_b_emlrt_marshallOut(0U), "mFileTimeHi",
                  "mFileTimeHi", 154);
  sf_mex_assign(&c3_rhs154, sf_mex_createcellmatrix(0, 1), false);
  sf_mex_assign(&c3_lhs154, sf_mex_createcellmatrix(0, 1), false);
  sf_mex_addfield(*c3_info, sf_mex_duplicatearraysafe(&c3_rhs154), "rhs", "rhs",
                  154);
  sf_mex_addfield(*c3_info, sf_mex_duplicatearraysafe(&c3_lhs154), "lhs", "lhs",
                  154);
  sf_mex_addfield(*c3_info, c3_emlrt_marshallOut(
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/eml/eml_ipiv2perm.m"), "context",
                  "context", 155);
  sf_mex_addfield(*c3_info, c3_emlrt_marshallOut("eml_index_class"), "name",
                  "name", 155);
  sf_mex_addfield(*c3_info, c3_emlrt_marshallOut(""), "dominantType",
                  "dominantType", 155);
  sf_mex_addfield(*c3_info, c3_emlrt_marshallOut(
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/eml/eml_index_class.m"),
                  "resolved", "resolved", 155);
  sf_mex_addfield(*c3_info, c3_b_emlrt_marshallOut(1323195778U), "fileTimeLo",
                  "fileTimeLo", 155);
  sf_mex_addfield(*c3_info, c3_b_emlrt_marshallOut(0U), "fileTimeHi",
                  "fileTimeHi", 155);
  sf_mex_addfield(*c3_info, c3_b_emlrt_marshallOut(0U), "mFileTimeLo",
                  "mFileTimeLo", 155);
  sf_mex_addfield(*c3_info, c3_b_emlrt_marshallOut(0U), "mFileTimeHi",
                  "mFileTimeHi", 155);
  sf_mex_assign(&c3_rhs155, sf_mex_createcellmatrix(0, 1), false);
  sf_mex_assign(&c3_lhs155, sf_mex_createcellmatrix(0, 1), false);
  sf_mex_addfield(*c3_info, sf_mex_duplicatearraysafe(&c3_rhs155), "rhs", "rhs",
                  155);
  sf_mex_addfield(*c3_info, sf_mex_duplicatearraysafe(&c3_lhs155), "lhs", "lhs",
                  155);
  sf_mex_addfield(*c3_info, c3_emlrt_marshallOut(
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/eml/eml_ipiv2perm.m"), "context",
                  "context", 156);
  sf_mex_addfield(*c3_info, c3_emlrt_marshallOut("coder.internal.indexIntRelop"),
                  "name", "name", 156);
  sf_mex_addfield(*c3_info, c3_emlrt_marshallOut(""), "dominantType",
                  "dominantType", 156);
  sf_mex_addfield(*c3_info, c3_emlrt_marshallOut(
    "[ILXE]$matlabroot$/toolbox/shared/coder/coder/+coder/+internal/indexIntRelop.m"),
                  "resolved", "resolved", 156);
  sf_mex_addfield(*c3_info, c3_b_emlrt_marshallOut(1326753522U), "fileTimeLo",
                  "fileTimeLo", 156);
  sf_mex_addfield(*c3_info, c3_b_emlrt_marshallOut(0U), "fileTimeHi",
                  "fileTimeHi", 156);
  sf_mex_addfield(*c3_info, c3_b_emlrt_marshallOut(0U), "mFileTimeLo",
                  "mFileTimeLo", 156);
  sf_mex_addfield(*c3_info, c3_b_emlrt_marshallOut(0U), "mFileTimeHi",
                  "mFileTimeHi", 156);
  sf_mex_assign(&c3_rhs156, sf_mex_createcellmatrix(0, 1), false);
  sf_mex_assign(&c3_lhs156, sf_mex_createcellmatrix(0, 1), false);
  sf_mex_addfield(*c3_info, sf_mex_duplicatearraysafe(&c3_rhs156), "rhs", "rhs",
                  156);
  sf_mex_addfield(*c3_info, sf_mex_duplicatearraysafe(&c3_lhs156), "lhs", "lhs",
                  156);
  sf_mex_addfield(*c3_info, c3_emlrt_marshallOut(
    "[ILXE]$matlabroot$/toolbox/shared/coder/coder/+coder/+internal/indexIntRelop.m!apply_float_relop"),
                  "context", "context", 157);
  sf_mex_addfield(*c3_info, c3_emlrt_marshallOut("eml_switch_helper"), "name",
                  "name", 157);
  sf_mex_addfield(*c3_info, c3_emlrt_marshallOut(""), "dominantType",
                  "dominantType", 157);
  sf_mex_addfield(*c3_info, c3_emlrt_marshallOut(
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/eml/eml_switch_helper.m"),
                  "resolved", "resolved", 157);
  sf_mex_addfield(*c3_info, c3_b_emlrt_marshallOut(1393356058U), "fileTimeLo",
                  "fileTimeLo", 157);
  sf_mex_addfield(*c3_info, c3_b_emlrt_marshallOut(0U), "fileTimeHi",
                  "fileTimeHi", 157);
  sf_mex_addfield(*c3_info, c3_b_emlrt_marshallOut(0U), "mFileTimeLo",
                  "mFileTimeLo", 157);
  sf_mex_addfield(*c3_info, c3_b_emlrt_marshallOut(0U), "mFileTimeHi",
                  "mFileTimeHi", 157);
  sf_mex_assign(&c3_rhs157, sf_mex_createcellmatrix(0, 1), false);
  sf_mex_assign(&c3_lhs157, sf_mex_createcellmatrix(0, 1), false);
  sf_mex_addfield(*c3_info, sf_mex_duplicatearraysafe(&c3_rhs157), "rhs", "rhs",
                  157);
  sf_mex_addfield(*c3_info, sf_mex_duplicatearraysafe(&c3_lhs157), "lhs", "lhs",
                  157);
  sf_mex_addfield(*c3_info, c3_emlrt_marshallOut(
    "[ILXE]$matlabroot$/toolbox/shared/coder/coder/+coder/+internal/indexIntRelop.m!float_class_contains_indexIntClass"),
                  "context", "context", 158);
  sf_mex_addfield(*c3_info, c3_emlrt_marshallOut("eml_float_model"), "name",
                  "name", 158);
  sf_mex_addfield(*c3_info, c3_emlrt_marshallOut("char"), "dominantType",
                  "dominantType", 158);
  sf_mex_addfield(*c3_info, c3_emlrt_marshallOut(
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/eml/eml_float_model.m"),
                  "resolved", "resolved", 158);
  sf_mex_addfield(*c3_info, c3_b_emlrt_marshallOut(1326753196U), "fileTimeLo",
                  "fileTimeLo", 158);
  sf_mex_addfield(*c3_info, c3_b_emlrt_marshallOut(0U), "fileTimeHi",
                  "fileTimeHi", 158);
  sf_mex_addfield(*c3_info, c3_b_emlrt_marshallOut(0U), "mFileTimeLo",
                  "mFileTimeLo", 158);
  sf_mex_addfield(*c3_info, c3_b_emlrt_marshallOut(0U), "mFileTimeHi",
                  "mFileTimeHi", 158);
  sf_mex_assign(&c3_rhs158, sf_mex_createcellmatrix(0, 1), false);
  sf_mex_assign(&c3_lhs158, sf_mex_createcellmatrix(0, 1), false);
  sf_mex_addfield(*c3_info, sf_mex_duplicatearraysafe(&c3_rhs158), "rhs", "rhs",
                  158);
  sf_mex_addfield(*c3_info, sf_mex_duplicatearraysafe(&c3_lhs158), "lhs", "lhs",
                  158);
  sf_mex_addfield(*c3_info, c3_emlrt_marshallOut(
    "[ILXE]$matlabroot$/toolbox/shared/coder/coder/+coder/+internal/indexIntRelop.m!is_signed_indexIntClass"),
                  "context", "context", 159);
  sf_mex_addfield(*c3_info, c3_emlrt_marshallOut("intmin"), "name", "name", 159);
  sf_mex_addfield(*c3_info, c3_emlrt_marshallOut("char"), "dominantType",
                  "dominantType", 159);
  sf_mex_addfield(*c3_info, c3_emlrt_marshallOut(
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/elmat/intmin.m"), "resolved",
                  "resolved", 159);
  sf_mex_addfield(*c3_info, c3_b_emlrt_marshallOut(1362287082U), "fileTimeLo",
                  "fileTimeLo", 159);
  sf_mex_addfield(*c3_info, c3_b_emlrt_marshallOut(0U), "fileTimeHi",
                  "fileTimeHi", 159);
  sf_mex_addfield(*c3_info, c3_b_emlrt_marshallOut(0U), "mFileTimeLo",
                  "mFileTimeLo", 159);
  sf_mex_addfield(*c3_info, c3_b_emlrt_marshallOut(0U), "mFileTimeHi",
                  "mFileTimeHi", 159);
  sf_mex_assign(&c3_rhs159, sf_mex_createcellmatrix(0, 1), false);
  sf_mex_assign(&c3_lhs159, sf_mex_createcellmatrix(0, 1), false);
  sf_mex_addfield(*c3_info, sf_mex_duplicatearraysafe(&c3_rhs159), "rhs", "rhs",
                  159);
  sf_mex_addfield(*c3_info, sf_mex_duplicatearraysafe(&c3_lhs159), "lhs", "lhs",
                  159);
  sf_mex_addfield(*c3_info, c3_emlrt_marshallOut(
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/matfun/inv.m!invNxN"), "context",
                  "context", 160);
  sf_mex_addfield(*c3_info, c3_emlrt_marshallOut(
    "eml_int_forloop_overflow_check"), "name", "name", 160);
  sf_mex_addfield(*c3_info, c3_emlrt_marshallOut(""), "dominantType",
                  "dominantType", 160);
  sf_mex_addfield(*c3_info, c3_emlrt_marshallOut(
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/eml/eml_int_forloop_overflow_check.m"),
                  "resolved", "resolved", 160);
  sf_mex_addfield(*c3_info, c3_b_emlrt_marshallOut(1397282622U), "fileTimeLo",
                  "fileTimeLo", 160);
  sf_mex_addfield(*c3_info, c3_b_emlrt_marshallOut(0U), "fileTimeHi",
                  "fileTimeHi", 160);
  sf_mex_addfield(*c3_info, c3_b_emlrt_marshallOut(0U), "mFileTimeLo",
                  "mFileTimeLo", 160);
  sf_mex_addfield(*c3_info, c3_b_emlrt_marshallOut(0U), "mFileTimeHi",
                  "mFileTimeHi", 160);
  sf_mex_assign(&c3_rhs160, sf_mex_createcellmatrix(0, 1), false);
  sf_mex_assign(&c3_lhs160, sf_mex_createcellmatrix(0, 1), false);
  sf_mex_addfield(*c3_info, sf_mex_duplicatearraysafe(&c3_rhs160), "rhs", "rhs",
                  160);
  sf_mex_addfield(*c3_info, sf_mex_duplicatearraysafe(&c3_lhs160), "lhs", "lhs",
                  160);
  sf_mex_addfield(*c3_info, c3_emlrt_marshallOut(
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/matfun/inv.m!invNxN"), "context",
                  "context", 161);
  sf_mex_addfield(*c3_info, c3_emlrt_marshallOut("eml_index_plus"), "name",
                  "name", 161);
  sf_mex_addfield(*c3_info, c3_emlrt_marshallOut("double"), "dominantType",
                  "dominantType", 161);
  sf_mex_addfield(*c3_info, c3_emlrt_marshallOut(
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/eml/eml_index_plus.m"),
                  "resolved", "resolved", 161);
  sf_mex_addfield(*c3_info, c3_b_emlrt_marshallOut(1372607616U), "fileTimeLo",
                  "fileTimeLo", 161);
  sf_mex_addfield(*c3_info, c3_b_emlrt_marshallOut(0U), "fileTimeHi",
                  "fileTimeHi", 161);
  sf_mex_addfield(*c3_info, c3_b_emlrt_marshallOut(0U), "mFileTimeLo",
                  "mFileTimeLo", 161);
  sf_mex_addfield(*c3_info, c3_b_emlrt_marshallOut(0U), "mFileTimeHi",
                  "mFileTimeHi", 161);
  sf_mex_assign(&c3_rhs161, sf_mex_createcellmatrix(0, 1), false);
  sf_mex_assign(&c3_lhs161, sf_mex_createcellmatrix(0, 1), false);
  sf_mex_addfield(*c3_info, sf_mex_duplicatearraysafe(&c3_rhs161), "rhs", "rhs",
                  161);
  sf_mex_addfield(*c3_info, sf_mex_duplicatearraysafe(&c3_lhs161), "lhs", "lhs",
                  161);
  sf_mex_addfield(*c3_info, c3_emlrt_marshallOut(
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/matfun/inv.m!invNxN"), "context",
                  "context", 162);
  sf_mex_addfield(*c3_info, c3_emlrt_marshallOut("eml_scalar_eg"), "name",
                  "name", 162);
  sf_mex_addfield(*c3_info, c3_emlrt_marshallOut("double"), "dominantType",
                  "dominantType", 162);
  sf_mex_addfield(*c3_info, c3_emlrt_marshallOut(
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/eml/eml_scalar_eg.m"), "resolved",
                  "resolved", 162);
  sf_mex_addfield(*c3_info, c3_b_emlrt_marshallOut(1376005888U), "fileTimeLo",
                  "fileTimeLo", 162);
  sf_mex_addfield(*c3_info, c3_b_emlrt_marshallOut(0U), "fileTimeHi",
                  "fileTimeHi", 162);
  sf_mex_addfield(*c3_info, c3_b_emlrt_marshallOut(0U), "mFileTimeLo",
                  "mFileTimeLo", 162);
  sf_mex_addfield(*c3_info, c3_b_emlrt_marshallOut(0U), "mFileTimeHi",
                  "mFileTimeHi", 162);
  sf_mex_assign(&c3_rhs162, sf_mex_createcellmatrix(0, 1), false);
  sf_mex_assign(&c3_lhs162, sf_mex_createcellmatrix(0, 1), false);
  sf_mex_addfield(*c3_info, sf_mex_duplicatearraysafe(&c3_rhs162), "rhs", "rhs",
                  162);
  sf_mex_addfield(*c3_info, sf_mex_duplicatearraysafe(&c3_lhs162), "lhs", "lhs",
                  162);
  sf_mex_addfield(*c3_info, c3_emlrt_marshallOut(
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/matfun/inv.m!invNxN"), "context",
                  "context", 163);
  sf_mex_addfield(*c3_info, c3_emlrt_marshallOut("eml_xtrsm"), "name", "name",
                  163);
  sf_mex_addfield(*c3_info, c3_emlrt_marshallOut("char"), "dominantType",
                  "dominantType", 163);
  sf_mex_addfield(*c3_info, c3_emlrt_marshallOut(
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/eml/blas/eml_xtrsm.m"),
                  "resolved", "resolved", 163);
  sf_mex_addfield(*c3_info, c3_b_emlrt_marshallOut(1376005892U), "fileTimeLo",
                  "fileTimeLo", 163);
  sf_mex_addfield(*c3_info, c3_b_emlrt_marshallOut(0U), "fileTimeHi",
                  "fileTimeHi", 163);
  sf_mex_addfield(*c3_info, c3_b_emlrt_marshallOut(0U), "mFileTimeLo",
                  "mFileTimeLo", 163);
  sf_mex_addfield(*c3_info, c3_b_emlrt_marshallOut(0U), "mFileTimeHi",
                  "mFileTimeHi", 163);
  sf_mex_assign(&c3_rhs163, sf_mex_createcellmatrix(0, 1), false);
  sf_mex_assign(&c3_lhs163, sf_mex_createcellmatrix(0, 1), false);
  sf_mex_addfield(*c3_info, sf_mex_duplicatearraysafe(&c3_rhs163), "rhs", "rhs",
                  163);
  sf_mex_addfield(*c3_info, sf_mex_duplicatearraysafe(&c3_lhs163), "lhs", "lhs",
                  163);
  sf_mex_addfield(*c3_info, c3_emlrt_marshallOut(
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/eml/blas/eml_xtrsm.m"), "context",
                  "context", 164);
  sf_mex_addfield(*c3_info, c3_emlrt_marshallOut("coder.internal.blas.inline"),
                  "name", "name", 164);
  sf_mex_addfield(*c3_info, c3_emlrt_marshallOut(""), "dominantType",
                  "dominantType", 164);
  sf_mex_addfield(*c3_info, c3_emlrt_marshallOut(
    "[ILXE]$matlabroot$/toolbox/coder/coder/+coder/+internal/+blas/inline.p"),
                  "resolved", "resolved", 164);
  sf_mex_addfield(*c3_info, c3_b_emlrt_marshallOut(1410832972U), "fileTimeLo",
                  "fileTimeLo", 164);
  sf_mex_addfield(*c3_info, c3_b_emlrt_marshallOut(0U), "fileTimeHi",
                  "fileTimeHi", 164);
  sf_mex_addfield(*c3_info, c3_b_emlrt_marshallOut(0U), "mFileTimeLo",
                  "mFileTimeLo", 164);
  sf_mex_addfield(*c3_info, c3_b_emlrt_marshallOut(0U), "mFileTimeHi",
                  "mFileTimeHi", 164);
  sf_mex_assign(&c3_rhs164, sf_mex_createcellmatrix(0, 1), false);
  sf_mex_assign(&c3_lhs164, sf_mex_createcellmatrix(0, 1), false);
  sf_mex_addfield(*c3_info, sf_mex_duplicatearraysafe(&c3_rhs164), "rhs", "rhs",
                  164);
  sf_mex_addfield(*c3_info, sf_mex_duplicatearraysafe(&c3_lhs164), "lhs", "lhs",
                  164);
  sf_mex_addfield(*c3_info, c3_emlrt_marshallOut(
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/eml/blas/eml_xtrsm.m"), "context",
                  "context", 165);
  sf_mex_addfield(*c3_info, c3_emlrt_marshallOut("coder.internal.blas.xtrsm"),
                  "name", "name", 165);
  sf_mex_addfield(*c3_info, c3_emlrt_marshallOut("char"), "dominantType",
                  "dominantType", 165);
  sf_mex_addfield(*c3_info, c3_emlrt_marshallOut(
    "[ILXE]$matlabroot$/toolbox/coder/coder/+coder/+internal/+blas/xtrsm.p"),
                  "resolved", "resolved", 165);
  sf_mex_addfield(*c3_info, c3_b_emlrt_marshallOut(1410832970U), "fileTimeLo",
                  "fileTimeLo", 165);
  sf_mex_addfield(*c3_info, c3_b_emlrt_marshallOut(0U), "fileTimeHi",
                  "fileTimeHi", 165);
  sf_mex_addfield(*c3_info, c3_b_emlrt_marshallOut(0U), "mFileTimeLo",
                  "mFileTimeLo", 165);
  sf_mex_addfield(*c3_info, c3_b_emlrt_marshallOut(0U), "mFileTimeHi",
                  "mFileTimeHi", 165);
  sf_mex_assign(&c3_rhs165, sf_mex_createcellmatrix(0, 1), false);
  sf_mex_assign(&c3_lhs165, sf_mex_createcellmatrix(0, 1), false);
  sf_mex_addfield(*c3_info, sf_mex_duplicatearraysafe(&c3_rhs165), "rhs", "rhs",
                  165);
  sf_mex_addfield(*c3_info, sf_mex_duplicatearraysafe(&c3_lhs165), "lhs", "lhs",
                  165);
  sf_mex_addfield(*c3_info, c3_emlrt_marshallOut(
    "[ILXE]$matlabroot$/toolbox/coder/coder/+coder/+internal/+blas/xtrsm.p"),
                  "context", "context", 166);
  sf_mex_addfield(*c3_info, c3_emlrt_marshallOut(
    "coder.internal.blas.use_refblas"), "name", "name", 166);
  sf_mex_addfield(*c3_info, c3_emlrt_marshallOut(""), "dominantType",
                  "dominantType", 166);
  sf_mex_addfield(*c3_info, c3_emlrt_marshallOut(
    "[ILXE]$matlabroot$/toolbox/coder/coder/+coder/+internal/+blas/use_refblas.p"),
                  "resolved", "resolved", 166);
  sf_mex_addfield(*c3_info, c3_b_emlrt_marshallOut(1410832970U), "fileTimeLo",
                  "fileTimeLo", 166);
  sf_mex_addfield(*c3_info, c3_b_emlrt_marshallOut(0U), "fileTimeHi",
                  "fileTimeHi", 166);
  sf_mex_addfield(*c3_info, c3_b_emlrt_marshallOut(0U), "mFileTimeLo",
                  "mFileTimeLo", 166);
  sf_mex_addfield(*c3_info, c3_b_emlrt_marshallOut(0U), "mFileTimeHi",
                  "mFileTimeHi", 166);
  sf_mex_assign(&c3_rhs166, sf_mex_createcellmatrix(0, 1), false);
  sf_mex_assign(&c3_lhs166, sf_mex_createcellmatrix(0, 1), false);
  sf_mex_addfield(*c3_info, sf_mex_duplicatearraysafe(&c3_rhs166), "rhs", "rhs",
                  166);
  sf_mex_addfield(*c3_info, sf_mex_duplicatearraysafe(&c3_lhs166), "lhs", "lhs",
                  166);
  sf_mex_addfield(*c3_info, c3_emlrt_marshallOut(
    "[ILXE]$matlabroot$/toolbox/coder/coder/+coder/+internal/+blas/xtrsm.p!below_threshold"),
                  "context", "context", 167);
  sf_mex_addfield(*c3_info, c3_emlrt_marshallOut("coder.internal.blas.threshold"),
                  "name", "name", 167);
  sf_mex_addfield(*c3_info, c3_emlrt_marshallOut("char"), "dominantType",
                  "dominantType", 167);
  sf_mex_addfield(*c3_info, c3_emlrt_marshallOut(
    "[ILXE]$matlabroot$/toolbox/coder/coder/+coder/+internal/+blas/threshold.p"),
                  "resolved", "resolved", 167);
  sf_mex_addfield(*c3_info, c3_b_emlrt_marshallOut(1410832972U), "fileTimeLo",
                  "fileTimeLo", 167);
  sf_mex_addfield(*c3_info, c3_b_emlrt_marshallOut(0U), "fileTimeHi",
                  "fileTimeHi", 167);
  sf_mex_addfield(*c3_info, c3_b_emlrt_marshallOut(0U), "mFileTimeLo",
                  "mFileTimeLo", 167);
  sf_mex_addfield(*c3_info, c3_b_emlrt_marshallOut(0U), "mFileTimeHi",
                  "mFileTimeHi", 167);
  sf_mex_assign(&c3_rhs167, sf_mex_createcellmatrix(0, 1), false);
  sf_mex_assign(&c3_lhs167, sf_mex_createcellmatrix(0, 1), false);
  sf_mex_addfield(*c3_info, sf_mex_duplicatearraysafe(&c3_rhs167), "rhs", "rhs",
                  167);
  sf_mex_addfield(*c3_info, sf_mex_duplicatearraysafe(&c3_lhs167), "lhs", "lhs",
                  167);
  sf_mex_addfield(*c3_info, c3_emlrt_marshallOut(
    "[ILXE]$matlabroot$/toolbox/coder/coder/+coder/+internal/+blas/xtrsm.p"),
                  "context", "context", 168);
  sf_mex_addfield(*c3_info, c3_emlrt_marshallOut("coder.internal.scalarEg"),
                  "name", "name", 168);
  sf_mex_addfield(*c3_info, c3_emlrt_marshallOut("double"), "dominantType",
                  "dominantType", 168);
  sf_mex_addfield(*c3_info, c3_emlrt_marshallOut(
    "[ILXE]$matlabroot$/toolbox/coder/coder/+coder/+internal/scalarEg.p"),
                  "resolved", "resolved", 168);
  sf_mex_addfield(*c3_info, c3_b_emlrt_marshallOut(1410832970U), "fileTimeLo",
                  "fileTimeLo", 168);
  sf_mex_addfield(*c3_info, c3_b_emlrt_marshallOut(0U), "fileTimeHi",
                  "fileTimeHi", 168);
  sf_mex_addfield(*c3_info, c3_b_emlrt_marshallOut(0U), "mFileTimeLo",
                  "mFileTimeLo", 168);
  sf_mex_addfield(*c3_info, c3_b_emlrt_marshallOut(0U), "mFileTimeHi",
                  "mFileTimeHi", 168);
  sf_mex_assign(&c3_rhs168, sf_mex_createcellmatrix(0, 1), false);
  sf_mex_assign(&c3_lhs168, sf_mex_createcellmatrix(0, 1), false);
  sf_mex_addfield(*c3_info, sf_mex_duplicatearraysafe(&c3_rhs168), "rhs", "rhs",
                  168);
  sf_mex_addfield(*c3_info, sf_mex_duplicatearraysafe(&c3_lhs168), "lhs", "lhs",
                  168);
  sf_mex_addfield(*c3_info, c3_emlrt_marshallOut(
    "[ILXE]$matlabroot$/toolbox/coder/coder/+coder/+internal/+blas/xtrsm.p"),
                  "context", "context", 169);
  sf_mex_addfield(*c3_info, c3_emlrt_marshallOut("coder.internal.refblas.xtrsm"),
                  "name", "name", 169);
  sf_mex_addfield(*c3_info, c3_emlrt_marshallOut("char"), "dominantType",
                  "dominantType", 169);
  sf_mex_addfield(*c3_info, c3_emlrt_marshallOut(
    "[ILXE]$matlabroot$/toolbox/coder/coder/+coder/+internal/+refblas/xtrsm.p"),
                  "resolved", "resolved", 169);
  sf_mex_addfield(*c3_info, c3_b_emlrt_marshallOut(1410832972U), "fileTimeLo",
                  "fileTimeLo", 169);
  sf_mex_addfield(*c3_info, c3_b_emlrt_marshallOut(0U), "fileTimeHi",
                  "fileTimeHi", 169);
  sf_mex_addfield(*c3_info, c3_b_emlrt_marshallOut(0U), "mFileTimeLo",
                  "mFileTimeLo", 169);
  sf_mex_addfield(*c3_info, c3_b_emlrt_marshallOut(0U), "mFileTimeHi",
                  "mFileTimeHi", 169);
  sf_mex_assign(&c3_rhs169, sf_mex_createcellmatrix(0, 1), false);
  sf_mex_assign(&c3_lhs169, sf_mex_createcellmatrix(0, 1), false);
  sf_mex_addfield(*c3_info, sf_mex_duplicatearraysafe(&c3_rhs169), "rhs", "rhs",
                  169);
  sf_mex_addfield(*c3_info, sf_mex_duplicatearraysafe(&c3_lhs169), "lhs", "lhs",
                  169);
  sf_mex_addfield(*c3_info, c3_emlrt_marshallOut(
    "[ILXE]$matlabroot$/toolbox/coder/coder/+coder/+internal/+refblas/xtrsm.p"),
                  "context", "context", 170);
  sf_mex_addfield(*c3_info, c3_emlrt_marshallOut("coder.internal.assert"),
                  "name", "name", 170);
  sf_mex_addfield(*c3_info, c3_emlrt_marshallOut("char"), "dominantType",
                  "dominantType", 170);
  sf_mex_addfield(*c3_info, c3_emlrt_marshallOut(
    "[ILXE]$matlabroot$/toolbox/shared/coder/coder/+coder/+internal/assert.m"),
                  "resolved", "resolved", 170);
  sf_mex_addfield(*c3_info, c3_b_emlrt_marshallOut(1389742974U), "fileTimeLo",
                  "fileTimeLo", 170);
  sf_mex_addfield(*c3_info, c3_b_emlrt_marshallOut(0U), "fileTimeHi",
                  "fileTimeHi", 170);
  sf_mex_addfield(*c3_info, c3_b_emlrt_marshallOut(0U), "mFileTimeLo",
                  "mFileTimeLo", 170);
  sf_mex_addfield(*c3_info, c3_b_emlrt_marshallOut(0U), "mFileTimeHi",
                  "mFileTimeHi", 170);
  sf_mex_assign(&c3_rhs170, sf_mex_createcellmatrix(0, 1), false);
  sf_mex_assign(&c3_lhs170, sf_mex_createcellmatrix(0, 1), false);
  sf_mex_addfield(*c3_info, sf_mex_duplicatearraysafe(&c3_rhs170), "rhs", "rhs",
                  170);
  sf_mex_addfield(*c3_info, sf_mex_duplicatearraysafe(&c3_lhs170), "lhs", "lhs",
                  170);
  sf_mex_addfield(*c3_info, c3_emlrt_marshallOut(
    "[ILXE]$matlabroot$/toolbox/coder/coder/+coder/+internal/+refblas/xtrsm.p"),
                  "context", "context", 171);
  sf_mex_addfield(*c3_info, c3_emlrt_marshallOut("coder.internal.scalarEg"),
                  "name", "name", 171);
  sf_mex_addfield(*c3_info, c3_emlrt_marshallOut("double"), "dominantType",
                  "dominantType", 171);
  sf_mex_addfield(*c3_info, c3_emlrt_marshallOut(
    "[ILXE]$matlabroot$/toolbox/coder/coder/+coder/+internal/scalarEg.p"),
                  "resolved", "resolved", 171);
  sf_mex_addfield(*c3_info, c3_b_emlrt_marshallOut(1410832970U), "fileTimeLo",
                  "fileTimeLo", 171);
  sf_mex_addfield(*c3_info, c3_b_emlrt_marshallOut(0U), "fileTimeHi",
                  "fileTimeHi", 171);
  sf_mex_addfield(*c3_info, c3_b_emlrt_marshallOut(0U), "mFileTimeLo",
                  "mFileTimeLo", 171);
  sf_mex_addfield(*c3_info, c3_b_emlrt_marshallOut(0U), "mFileTimeHi",
                  "mFileTimeHi", 171);
  sf_mex_assign(&c3_rhs171, sf_mex_createcellmatrix(0, 1), false);
  sf_mex_assign(&c3_lhs171, sf_mex_createcellmatrix(0, 1), false);
  sf_mex_addfield(*c3_info, sf_mex_duplicatearraysafe(&c3_rhs171), "rhs", "rhs",
                  171);
  sf_mex_addfield(*c3_info, sf_mex_duplicatearraysafe(&c3_lhs171), "lhs", "lhs",
                  171);
  sf_mex_addfield(*c3_info, c3_emlrt_marshallOut(
    "[ILXE]$matlabroot$/toolbox/coder/coder/+coder/+internal/+refblas/xtrsm.p"),
                  "context", "context", 172);
  sf_mex_addfield(*c3_info, c3_emlrt_marshallOut(
    "eml_int_forloop_overflow_check"), "name", "name", 172);
  sf_mex_addfield(*c3_info, c3_emlrt_marshallOut(""), "dominantType",
                  "dominantType", 172);
  sf_mex_addfield(*c3_info, c3_emlrt_marshallOut(
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/eml/eml_int_forloop_overflow_check.m"),
                  "resolved", "resolved", 172);
  sf_mex_addfield(*c3_info, c3_b_emlrt_marshallOut(1397282622U), "fileTimeLo",
                  "fileTimeLo", 172);
  sf_mex_addfield(*c3_info, c3_b_emlrt_marshallOut(0U), "fileTimeHi",
                  "fileTimeHi", 172);
  sf_mex_addfield(*c3_info, c3_b_emlrt_marshallOut(0U), "mFileTimeLo",
                  "mFileTimeLo", 172);
  sf_mex_addfield(*c3_info, c3_b_emlrt_marshallOut(0U), "mFileTimeHi",
                  "mFileTimeHi", 172);
  sf_mex_assign(&c3_rhs172, sf_mex_createcellmatrix(0, 1), false);
  sf_mex_assign(&c3_lhs172, sf_mex_createcellmatrix(0, 1), false);
  sf_mex_addfield(*c3_info, sf_mex_duplicatearraysafe(&c3_rhs172), "rhs", "rhs",
                  172);
  sf_mex_addfield(*c3_info, sf_mex_duplicatearraysafe(&c3_lhs172), "lhs", "lhs",
                  172);
  sf_mex_addfield(*c3_info, c3_emlrt_marshallOut(
    "[ILXE]$matlabroot$/toolbox/coder/coder/+coder/+internal/+refblas/xtrsm.p"),
                  "context", "context", 173);
  sf_mex_addfield(*c3_info, c3_emlrt_marshallOut("rdivide"), "name", "name", 173);
  sf_mex_addfield(*c3_info, c3_emlrt_marshallOut("double"), "dominantType",
                  "dominantType", 173);
  sf_mex_addfield(*c3_info, c3_emlrt_marshallOut(
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/ops/rdivide.m"), "resolved",
                  "resolved", 173);
  sf_mex_addfield(*c3_info, c3_b_emlrt_marshallOut(1363735480U), "fileTimeLo",
                  "fileTimeLo", 173);
  sf_mex_addfield(*c3_info, c3_b_emlrt_marshallOut(0U), "fileTimeHi",
                  "fileTimeHi", 173);
  sf_mex_addfield(*c3_info, c3_b_emlrt_marshallOut(0U), "mFileTimeLo",
                  "mFileTimeLo", 173);
  sf_mex_addfield(*c3_info, c3_b_emlrt_marshallOut(0U), "mFileTimeHi",
                  "mFileTimeHi", 173);
  sf_mex_assign(&c3_rhs173, sf_mex_createcellmatrix(0, 1), false);
  sf_mex_assign(&c3_lhs173, sf_mex_createcellmatrix(0, 1), false);
  sf_mex_addfield(*c3_info, sf_mex_duplicatearraysafe(&c3_rhs173), "rhs", "rhs",
                  173);
  sf_mex_addfield(*c3_info, sf_mex_duplicatearraysafe(&c3_lhs173), "lhs", "lhs",
                  173);
  sf_mex_addfield(*c3_info, c3_emlrt_marshallOut(
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/ops/rdivide.m"), "context",
                  "context", 174);
  sf_mex_addfield(*c3_info, c3_emlrt_marshallOut(
    "coder.internal.isBuiltInNumeric"), "name", "name", 174);
  sf_mex_addfield(*c3_info, c3_emlrt_marshallOut("double"), "dominantType",
                  "dominantType", 174);
  sf_mex_addfield(*c3_info, c3_emlrt_marshallOut(
    "[ILXE]$matlabroot$/toolbox/shared/coder/coder/+coder/+internal/isBuiltInNumeric.m"),
                  "resolved", "resolved", 174);
  sf_mex_addfield(*c3_info, c3_b_emlrt_marshallOut(1395953456U), "fileTimeLo",
                  "fileTimeLo", 174);
  sf_mex_addfield(*c3_info, c3_b_emlrt_marshallOut(0U), "fileTimeHi",
                  "fileTimeHi", 174);
  sf_mex_addfield(*c3_info, c3_b_emlrt_marshallOut(0U), "mFileTimeLo",
                  "mFileTimeLo", 174);
  sf_mex_addfield(*c3_info, c3_b_emlrt_marshallOut(0U), "mFileTimeHi",
                  "mFileTimeHi", 174);
  sf_mex_assign(&c3_rhs174, sf_mex_createcellmatrix(0, 1), false);
  sf_mex_assign(&c3_lhs174, sf_mex_createcellmatrix(0, 1), false);
  sf_mex_addfield(*c3_info, sf_mex_duplicatearraysafe(&c3_rhs174), "rhs", "rhs",
                  174);
  sf_mex_addfield(*c3_info, sf_mex_duplicatearraysafe(&c3_lhs174), "lhs", "lhs",
                  174);
  sf_mex_addfield(*c3_info, c3_emlrt_marshallOut(
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/ops/rdivide.m"), "context",
                  "context", 175);
  sf_mex_addfield(*c3_info, c3_emlrt_marshallOut("eml_scalexp_compatible"),
                  "name", "name", 175);
  sf_mex_addfield(*c3_info, c3_emlrt_marshallOut("double"), "dominantType",
                  "dominantType", 175);
  sf_mex_addfield(*c3_info, c3_emlrt_marshallOut(
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/eml/eml_scalexp_compatible.m"),
                  "resolved", "resolved", 175);
  sf_mex_addfield(*c3_info, c3_b_emlrt_marshallOut(1286843996U), "fileTimeLo",
                  "fileTimeLo", 175);
  sf_mex_addfield(*c3_info, c3_b_emlrt_marshallOut(0U), "fileTimeHi",
                  "fileTimeHi", 175);
  sf_mex_addfield(*c3_info, c3_b_emlrt_marshallOut(0U), "mFileTimeLo",
                  "mFileTimeLo", 175);
  sf_mex_addfield(*c3_info, c3_b_emlrt_marshallOut(0U), "mFileTimeHi",
                  "mFileTimeHi", 175);
  sf_mex_assign(&c3_rhs175, sf_mex_createcellmatrix(0, 1), false);
  sf_mex_assign(&c3_lhs175, sf_mex_createcellmatrix(0, 1), false);
  sf_mex_addfield(*c3_info, sf_mex_duplicatearraysafe(&c3_rhs175), "rhs", "rhs",
                  175);
  sf_mex_addfield(*c3_info, sf_mex_duplicatearraysafe(&c3_lhs175), "lhs", "lhs",
                  175);
  sf_mex_addfield(*c3_info, c3_emlrt_marshallOut(
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/ops/rdivide.m"), "context",
                  "context", 176);
  sf_mex_addfield(*c3_info, c3_emlrt_marshallOut("eml_div"), "name", "name", 176);
  sf_mex_addfield(*c3_info, c3_emlrt_marshallOut("double"), "dominantType",
                  "dominantType", 176);
  sf_mex_addfield(*c3_info, c3_emlrt_marshallOut(
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/eml/eml_div.m"), "resolved",
                  "resolved", 176);
  sf_mex_addfield(*c3_info, c3_b_emlrt_marshallOut(1386449152U), "fileTimeLo",
                  "fileTimeLo", 176);
  sf_mex_addfield(*c3_info, c3_b_emlrt_marshallOut(0U), "fileTimeHi",
                  "fileTimeHi", 176);
  sf_mex_addfield(*c3_info, c3_b_emlrt_marshallOut(0U), "mFileTimeLo",
                  "mFileTimeLo", 176);
  sf_mex_addfield(*c3_info, c3_b_emlrt_marshallOut(0U), "mFileTimeHi",
                  "mFileTimeHi", 176);
  sf_mex_assign(&c3_rhs176, sf_mex_createcellmatrix(0, 1), false);
  sf_mex_assign(&c3_lhs176, sf_mex_createcellmatrix(0, 1), false);
  sf_mex_addfield(*c3_info, sf_mex_duplicatearraysafe(&c3_rhs176), "rhs", "rhs",
                  176);
  sf_mex_addfield(*c3_info, sf_mex_duplicatearraysafe(&c3_lhs176), "lhs", "lhs",
                  176);
  sf_mex_addfield(*c3_info, c3_emlrt_marshallOut(
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/matfun/inv.m!checkcond"),
                  "context", "context", 177);
  sf_mex_addfield(*c3_info, c3_emlrt_marshallOut("norm"), "name", "name", 177);
  sf_mex_addfield(*c3_info, c3_emlrt_marshallOut("double"), "dominantType",
                  "dominantType", 177);
  sf_mex_addfield(*c3_info, c3_emlrt_marshallOut(
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/matfun/norm.m"), "resolved",
                  "resolved", 177);
  sf_mex_addfield(*c3_info, c3_b_emlrt_marshallOut(1363735468U), "fileTimeLo",
                  "fileTimeLo", 177);
  sf_mex_addfield(*c3_info, c3_b_emlrt_marshallOut(0U), "fileTimeHi",
                  "fileTimeHi", 177);
  sf_mex_addfield(*c3_info, c3_b_emlrt_marshallOut(0U), "mFileTimeLo",
                  "mFileTimeLo", 177);
  sf_mex_addfield(*c3_info, c3_b_emlrt_marshallOut(0U), "mFileTimeHi",
                  "mFileTimeHi", 177);
  sf_mex_assign(&c3_rhs177, sf_mex_createcellmatrix(0, 1), false);
  sf_mex_assign(&c3_lhs177, sf_mex_createcellmatrix(0, 1), false);
  sf_mex_addfield(*c3_info, sf_mex_duplicatearraysafe(&c3_rhs177), "rhs", "rhs",
                  177);
  sf_mex_addfield(*c3_info, sf_mex_duplicatearraysafe(&c3_lhs177), "lhs", "lhs",
                  177);
  sf_mex_addfield(*c3_info, c3_emlrt_marshallOut(
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/matfun/norm.m"), "context",
                  "context", 178);
  sf_mex_addfield(*c3_info, c3_emlrt_marshallOut(
    "coder.internal.isBuiltInNumeric"), "name", "name", 178);
  sf_mex_addfield(*c3_info, c3_emlrt_marshallOut("double"), "dominantType",
                  "dominantType", 178);
  sf_mex_addfield(*c3_info, c3_emlrt_marshallOut(
    "[ILXE]$matlabroot$/toolbox/shared/coder/coder/+coder/+internal/isBuiltInNumeric.m"),
                  "resolved", "resolved", 178);
  sf_mex_addfield(*c3_info, c3_b_emlrt_marshallOut(1395953456U), "fileTimeLo",
                  "fileTimeLo", 178);
  sf_mex_addfield(*c3_info, c3_b_emlrt_marshallOut(0U), "fileTimeHi",
                  "fileTimeHi", 178);
  sf_mex_addfield(*c3_info, c3_b_emlrt_marshallOut(0U), "mFileTimeLo",
                  "mFileTimeLo", 178);
  sf_mex_addfield(*c3_info, c3_b_emlrt_marshallOut(0U), "mFileTimeHi",
                  "mFileTimeHi", 178);
  sf_mex_assign(&c3_rhs178, sf_mex_createcellmatrix(0, 1), false);
  sf_mex_assign(&c3_lhs178, sf_mex_createcellmatrix(0, 1), false);
  sf_mex_addfield(*c3_info, sf_mex_duplicatearraysafe(&c3_rhs178), "rhs", "rhs",
                  178);
  sf_mex_addfield(*c3_info, sf_mex_duplicatearraysafe(&c3_lhs178), "lhs", "lhs",
                  178);
  sf_mex_addfield(*c3_info, c3_emlrt_marshallOut(
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/matfun/norm.m!mat1norm"),
                  "context", "context", 179);
  sf_mex_addfield(*c3_info, c3_emlrt_marshallOut("abs"), "name", "name", 179);
  sf_mex_addfield(*c3_info, c3_emlrt_marshallOut("double"), "dominantType",
                  "dominantType", 179);
  sf_mex_addfield(*c3_info, c3_emlrt_marshallOut(
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/elfun/abs.m"), "resolved",
                  "resolved", 179);
  sf_mex_addfield(*c3_info, c3_b_emlrt_marshallOut(1363735452U), "fileTimeLo",
                  "fileTimeLo", 179);
  sf_mex_addfield(*c3_info, c3_b_emlrt_marshallOut(0U), "fileTimeHi",
                  "fileTimeHi", 179);
  sf_mex_addfield(*c3_info, c3_b_emlrt_marshallOut(0U), "mFileTimeLo",
                  "mFileTimeLo", 179);
  sf_mex_addfield(*c3_info, c3_b_emlrt_marshallOut(0U), "mFileTimeHi",
                  "mFileTimeHi", 179);
  sf_mex_assign(&c3_rhs179, sf_mex_createcellmatrix(0, 1), false);
  sf_mex_assign(&c3_lhs179, sf_mex_createcellmatrix(0, 1), false);
  sf_mex_addfield(*c3_info, sf_mex_duplicatearraysafe(&c3_rhs179), "rhs", "rhs",
                  179);
  sf_mex_addfield(*c3_info, sf_mex_duplicatearraysafe(&c3_lhs179), "lhs", "lhs",
                  179);
  sf_mex_addfield(*c3_info, c3_emlrt_marshallOut(
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/matfun/norm.m!mat1norm"),
                  "context", "context", 180);
  sf_mex_addfield(*c3_info, c3_emlrt_marshallOut("isnan"), "name", "name", 180);
  sf_mex_addfield(*c3_info, c3_emlrt_marshallOut("double"), "dominantType",
                  "dominantType", 180);
  sf_mex_addfield(*c3_info, c3_emlrt_marshallOut(
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/elmat/isnan.m"), "resolved",
                  "resolved", 180);
  sf_mex_addfield(*c3_info, c3_b_emlrt_marshallOut(1363735458U), "fileTimeLo",
                  "fileTimeLo", 180);
  sf_mex_addfield(*c3_info, c3_b_emlrt_marshallOut(0U), "fileTimeHi",
                  "fileTimeHi", 180);
  sf_mex_addfield(*c3_info, c3_b_emlrt_marshallOut(0U), "mFileTimeLo",
                  "mFileTimeLo", 180);
  sf_mex_addfield(*c3_info, c3_b_emlrt_marshallOut(0U), "mFileTimeHi",
                  "mFileTimeHi", 180);
  sf_mex_assign(&c3_rhs180, sf_mex_createcellmatrix(0, 1), false);
  sf_mex_assign(&c3_lhs180, sf_mex_createcellmatrix(0, 1), false);
  sf_mex_addfield(*c3_info, sf_mex_duplicatearraysafe(&c3_rhs180), "rhs", "rhs",
                  180);
  sf_mex_addfield(*c3_info, sf_mex_duplicatearraysafe(&c3_lhs180), "lhs", "lhs",
                  180);
  sf_mex_addfield(*c3_info, c3_emlrt_marshallOut(
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/matfun/norm.m!mat1norm"),
                  "context", "context", 181);
  sf_mex_addfield(*c3_info, c3_emlrt_marshallOut("eml_guarded_nan"), "name",
                  "name", 181);
  sf_mex_addfield(*c3_info, c3_emlrt_marshallOut("char"), "dominantType",
                  "dominantType", 181);
  sf_mex_addfield(*c3_info, c3_emlrt_marshallOut(
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/eml/eml_guarded_nan.m"),
                  "resolved", "resolved", 181);
  sf_mex_addfield(*c3_info, c3_b_emlrt_marshallOut(1286843976U), "fileTimeLo",
                  "fileTimeLo", 181);
  sf_mex_addfield(*c3_info, c3_b_emlrt_marshallOut(0U), "fileTimeHi",
                  "fileTimeHi", 181);
  sf_mex_addfield(*c3_info, c3_b_emlrt_marshallOut(0U), "mFileTimeLo",
                  "mFileTimeLo", 181);
  sf_mex_addfield(*c3_info, c3_b_emlrt_marshallOut(0U), "mFileTimeHi",
                  "mFileTimeHi", 181);
  sf_mex_assign(&c3_rhs181, sf_mex_createcellmatrix(0, 1), false);
  sf_mex_assign(&c3_lhs181, sf_mex_createcellmatrix(0, 1), false);
  sf_mex_addfield(*c3_info, sf_mex_duplicatearraysafe(&c3_rhs181), "rhs", "rhs",
                  181);
  sf_mex_addfield(*c3_info, sf_mex_duplicatearraysafe(&c3_lhs181), "lhs", "lhs",
                  181);
  sf_mex_addfield(*c3_info, c3_emlrt_marshallOut(
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/eml/eml_guarded_nan.m"),
                  "context", "context", 182);
  sf_mex_addfield(*c3_info, c3_emlrt_marshallOut("eml_is_float_class"), "name",
                  "name", 182);
  sf_mex_addfield(*c3_info, c3_emlrt_marshallOut("char"), "dominantType",
                  "dominantType", 182);
  sf_mex_addfield(*c3_info, c3_emlrt_marshallOut(
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/eml/eml_is_float_class.m"),
                  "resolved", "resolved", 182);
  sf_mex_addfield(*c3_info, c3_b_emlrt_marshallOut(1286843982U), "fileTimeLo",
                  "fileTimeLo", 182);
  sf_mex_addfield(*c3_info, c3_b_emlrt_marshallOut(0U), "fileTimeHi",
                  "fileTimeHi", 182);
  sf_mex_addfield(*c3_info, c3_b_emlrt_marshallOut(0U), "mFileTimeLo",
                  "mFileTimeLo", 182);
  sf_mex_addfield(*c3_info, c3_b_emlrt_marshallOut(0U), "mFileTimeHi",
                  "mFileTimeHi", 182);
  sf_mex_assign(&c3_rhs182, sf_mex_createcellmatrix(0, 1), false);
  sf_mex_assign(&c3_lhs182, sf_mex_createcellmatrix(0, 1), false);
  sf_mex_addfield(*c3_info, sf_mex_duplicatearraysafe(&c3_rhs182), "rhs", "rhs",
                  182);
  sf_mex_addfield(*c3_info, sf_mex_duplicatearraysafe(&c3_lhs182), "lhs", "lhs",
                  182);
  sf_mex_addfield(*c3_info, c3_emlrt_marshallOut(
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/matfun/inv.m!checkcond"),
                  "context", "context", 183);
  sf_mex_addfield(*c3_info, c3_emlrt_marshallOut("eml_warning"), "name", "name",
                  183);
  sf_mex_addfield(*c3_info, c3_emlrt_marshallOut("char"), "dominantType",
                  "dominantType", 183);
  sf_mex_addfield(*c3_info, c3_emlrt_marshallOut(
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/eml/eml_warning.m"), "resolved",
                  "resolved", 183);
  sf_mex_addfield(*c3_info, c3_b_emlrt_marshallOut(1286844002U), "fileTimeLo",
                  "fileTimeLo", 183);
  sf_mex_addfield(*c3_info, c3_b_emlrt_marshallOut(0U), "fileTimeHi",
                  "fileTimeHi", 183);
  sf_mex_addfield(*c3_info, c3_b_emlrt_marshallOut(0U), "mFileTimeLo",
                  "mFileTimeLo", 183);
  sf_mex_addfield(*c3_info, c3_b_emlrt_marshallOut(0U), "mFileTimeHi",
                  "mFileTimeHi", 183);
  sf_mex_assign(&c3_rhs183, sf_mex_createcellmatrix(0, 1), false);
  sf_mex_assign(&c3_lhs183, sf_mex_createcellmatrix(0, 1), false);
  sf_mex_addfield(*c3_info, sf_mex_duplicatearraysafe(&c3_rhs183), "rhs", "rhs",
                  183);
  sf_mex_addfield(*c3_info, sf_mex_duplicatearraysafe(&c3_lhs183), "lhs", "lhs",
                  183);
  sf_mex_addfield(*c3_info, c3_emlrt_marshallOut(
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/matfun/inv.m!checkcond"),
                  "context", "context", 184);
  sf_mex_addfield(*c3_info, c3_emlrt_marshallOut("isnan"), "name", "name", 184);
  sf_mex_addfield(*c3_info, c3_emlrt_marshallOut("double"), "dominantType",
                  "dominantType", 184);
  sf_mex_addfield(*c3_info, c3_emlrt_marshallOut(
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/elmat/isnan.m"), "resolved",
                  "resolved", 184);
  sf_mex_addfield(*c3_info, c3_b_emlrt_marshallOut(1363735458U), "fileTimeLo",
                  "fileTimeLo", 184);
  sf_mex_addfield(*c3_info, c3_b_emlrt_marshallOut(0U), "fileTimeHi",
                  "fileTimeHi", 184);
  sf_mex_addfield(*c3_info, c3_b_emlrt_marshallOut(0U), "mFileTimeLo",
                  "mFileTimeLo", 184);
  sf_mex_addfield(*c3_info, c3_b_emlrt_marshallOut(0U), "mFileTimeHi",
                  "mFileTimeHi", 184);
  sf_mex_assign(&c3_rhs184, sf_mex_createcellmatrix(0, 1), false);
  sf_mex_assign(&c3_lhs184, sf_mex_createcellmatrix(0, 1), false);
  sf_mex_addfield(*c3_info, sf_mex_duplicatearraysafe(&c3_rhs184), "rhs", "rhs",
                  184);
  sf_mex_addfield(*c3_info, sf_mex_duplicatearraysafe(&c3_lhs184), "lhs", "lhs",
                  184);
  sf_mex_addfield(*c3_info, c3_emlrt_marshallOut(
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/matfun/inv.m!checkcond"),
                  "context", "context", 185);
  sf_mex_addfield(*c3_info, c3_emlrt_marshallOut("eps"), "name", "name", 185);
  sf_mex_addfield(*c3_info, c3_emlrt_marshallOut("char"), "dominantType",
                  "dominantType", 185);
  sf_mex_addfield(*c3_info, c3_emlrt_marshallOut(
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/elmat/eps.m"), "resolved",
                  "resolved", 185);
  sf_mex_addfield(*c3_info, c3_b_emlrt_marshallOut(1326753196U), "fileTimeLo",
                  "fileTimeLo", 185);
  sf_mex_addfield(*c3_info, c3_b_emlrt_marshallOut(0U), "fileTimeHi",
                  "fileTimeHi", 185);
  sf_mex_addfield(*c3_info, c3_b_emlrt_marshallOut(0U), "mFileTimeLo",
                  "mFileTimeLo", 185);
  sf_mex_addfield(*c3_info, c3_b_emlrt_marshallOut(0U), "mFileTimeHi",
                  "mFileTimeHi", 185);
  sf_mex_assign(&c3_rhs185, sf_mex_createcellmatrix(0, 1), false);
  sf_mex_assign(&c3_lhs185, sf_mex_createcellmatrix(0, 1), false);
  sf_mex_addfield(*c3_info, sf_mex_duplicatearraysafe(&c3_rhs185), "rhs", "rhs",
                  185);
  sf_mex_addfield(*c3_info, sf_mex_duplicatearraysafe(&c3_lhs185), "lhs", "lhs",
                  185);
  sf_mex_addfield(*c3_info, c3_emlrt_marshallOut(
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/matfun/inv.m!checkcond"),
                  "context", "context", 186);
  sf_mex_addfield(*c3_info, c3_emlrt_marshallOut("eml_flt2str"), "name", "name",
                  186);
  sf_mex_addfield(*c3_info, c3_emlrt_marshallOut("double"), "dominantType",
                  "dominantType", 186);
  sf_mex_addfield(*c3_info, c3_emlrt_marshallOut(
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/eml/eml_flt2str.m"), "resolved",
                  "resolved", 186);
  sf_mex_addfield(*c3_info, c3_b_emlrt_marshallOut(1360307550U), "fileTimeLo",
                  "fileTimeLo", 186);
  sf_mex_addfield(*c3_info, c3_b_emlrt_marshallOut(0U), "fileTimeHi",
                  "fileTimeHi", 186);
  sf_mex_addfield(*c3_info, c3_b_emlrt_marshallOut(0U), "mFileTimeLo",
                  "mFileTimeLo", 186);
  sf_mex_addfield(*c3_info, c3_b_emlrt_marshallOut(0U), "mFileTimeHi",
                  "mFileTimeHi", 186);
  sf_mex_assign(&c3_rhs186, sf_mex_createcellmatrix(0, 1), false);
  sf_mex_assign(&c3_lhs186, sf_mex_createcellmatrix(0, 1), false);
  sf_mex_addfield(*c3_info, sf_mex_duplicatearraysafe(&c3_rhs186), "rhs", "rhs",
                  186);
  sf_mex_addfield(*c3_info, sf_mex_duplicatearraysafe(&c3_lhs186), "lhs", "lhs",
                  186);
  sf_mex_addfield(*c3_info, c3_emlrt_marshallOut(
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/eml/eml_flt2str.m"), "context",
                  "context", 187);
  sf_mex_addfield(*c3_info, c3_emlrt_marshallOut("char"), "name", "name", 187);
  sf_mex_addfield(*c3_info, c3_emlrt_marshallOut("double"), "dominantType",
                  "dominantType", 187);
  sf_mex_addfield(*c3_info, c3_emlrt_marshallOut(
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/strfun/char.m"), "resolved",
                  "resolved", 187);
  sf_mex_addfield(*c3_info, c3_b_emlrt_marshallOut(1319755168U), "fileTimeLo",
                  "fileTimeLo", 187);
  sf_mex_addfield(*c3_info, c3_b_emlrt_marshallOut(0U), "fileTimeHi",
                  "fileTimeHi", 187);
  sf_mex_addfield(*c3_info, c3_b_emlrt_marshallOut(0U), "mFileTimeLo",
                  "mFileTimeLo", 187);
  sf_mex_addfield(*c3_info, c3_b_emlrt_marshallOut(0U), "mFileTimeHi",
                  "mFileTimeHi", 187);
  sf_mex_assign(&c3_rhs187, sf_mex_createcellmatrix(0, 1), false);
  sf_mex_assign(&c3_lhs187, sf_mex_createcellmatrix(0, 1), false);
  sf_mex_addfield(*c3_info, sf_mex_duplicatearraysafe(&c3_rhs187), "rhs", "rhs",
                  187);
  sf_mex_addfield(*c3_info, sf_mex_duplicatearraysafe(&c3_lhs187), "lhs", "lhs",
                  187);
  sf_mex_addfield(*c3_info, c3_emlrt_marshallOut(""), "context", "context", 188);
  sf_mex_addfield(*c3_info, c3_emlrt_marshallOut("newCamera"), "name", "name",
                  188);
  sf_mex_addfield(*c3_info, c3_emlrt_marshallOut("struct"), "dominantType",
                  "dominantType", 188);
  sf_mex_addfield(*c3_info, c3_emlrt_marshallOut(
    "[E]C:/Users/sd0016/Dropbox/UAH Doktora/Journal Tracking/Matlab Code/3DOF_Experiments/Mirage/util/newCamera.m"),
                  "resolved", "resolved", 188);
  sf_mex_addfield(*c3_info, c3_b_emlrt_marshallOut(1417540283U), "fileTimeLo",
                  "fileTimeLo", 188);
  sf_mex_addfield(*c3_info, c3_b_emlrt_marshallOut(0U), "fileTimeHi",
                  "fileTimeHi", 188);
  sf_mex_addfield(*c3_info, c3_b_emlrt_marshallOut(0U), "mFileTimeLo",
                  "mFileTimeLo", 188);
  sf_mex_addfield(*c3_info, c3_b_emlrt_marshallOut(0U), "mFileTimeHi",
                  "mFileTimeHi", 188);
  sf_mex_assign(&c3_rhs188, sf_mex_createcellmatrix(0, 1), false);
  sf_mex_assign(&c3_lhs188, sf_mex_createcellmatrix(0, 1), false);
  sf_mex_addfield(*c3_info, sf_mex_duplicatearraysafe(&c3_rhs188), "rhs", "rhs",
                  188);
  sf_mex_addfield(*c3_info, sf_mex_duplicatearraysafe(&c3_lhs188), "lhs", "lhs",
                  188);
  sf_mex_addfield(*c3_info, c3_emlrt_marshallOut(""), "context", "context", 189);
  sf_mex_addfield(*c3_info, c3_emlrt_marshallOut("newRobot"), "name", "name",
                  189);
  sf_mex_addfield(*c3_info, c3_emlrt_marshallOut("struct"), "dominantType",
                  "dominantType", 189);
  sf_mex_addfield(*c3_info, c3_emlrt_marshallOut(
    "[E]C:/Users/sd0016/Dropbox/UAH Doktora/Journal Tracking/Matlab Code/3DOF_Experiments/Mirage/util/newRobot.m"),
                  "resolved", "resolved", 189);
  sf_mex_addfield(*c3_info, c3_b_emlrt_marshallOut(1417540539U), "fileTimeLo",
                  "fileTimeLo", 189);
  sf_mex_addfield(*c3_info, c3_b_emlrt_marshallOut(0U), "fileTimeHi",
                  "fileTimeHi", 189);
  sf_mex_addfield(*c3_info, c3_b_emlrt_marshallOut(0U), "mFileTimeLo",
                  "mFileTimeLo", 189);
  sf_mex_addfield(*c3_info, c3_b_emlrt_marshallOut(0U), "mFileTimeHi",
                  "mFileTimeHi", 189);
  sf_mex_assign(&c3_rhs189, sf_mex_createcellmatrix(0, 1), false);
  sf_mex_assign(&c3_lhs189, sf_mex_createcellmatrix(0, 1), false);
  sf_mex_addfield(*c3_info, sf_mex_duplicatearraysafe(&c3_rhs189), "rhs", "rhs",
                  189);
  sf_mex_addfield(*c3_info, sf_mex_duplicatearraysafe(&c3_lhs189), "lhs", "lhs",
                  189);
  sf_mex_addfield(*c3_info, c3_emlrt_marshallOut(""), "context", "context", 190);
  sf_mex_addfield(*c3_info, c3_emlrt_marshallOut("eml_mtimes_helper"), "name",
                  "name", 190);
  sf_mex_addfield(*c3_info, c3_emlrt_marshallOut(""), "dominantType",
                  "dominantType", 190);
  sf_mex_addfield(*c3_info, c3_emlrt_marshallOut(
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/ops/eml_mtimes_helper.m"),
                  "resolved", "resolved", 190);
  sf_mex_addfield(*c3_info, c3_b_emlrt_marshallOut(1383902494U), "fileTimeLo",
                  "fileTimeLo", 190);
  sf_mex_addfield(*c3_info, c3_b_emlrt_marshallOut(0U), "fileTimeHi",
                  "fileTimeHi", 190);
  sf_mex_addfield(*c3_info, c3_b_emlrt_marshallOut(0U), "mFileTimeLo",
                  "mFileTimeLo", 190);
  sf_mex_addfield(*c3_info, c3_b_emlrt_marshallOut(0U), "mFileTimeHi",
                  "mFileTimeHi", 190);
  sf_mex_assign(&c3_rhs190, sf_mex_createcellmatrix(0, 1), false);
  sf_mex_assign(&c3_lhs190, sf_mex_createcellmatrix(0, 1), false);
  sf_mex_addfield(*c3_info, sf_mex_duplicatearraysafe(&c3_rhs190), "rhs", "rhs",
                  190);
  sf_mex_addfield(*c3_info, sf_mex_duplicatearraysafe(&c3_lhs190), "lhs", "lhs",
                  190);
  sf_mex_addfield(*c3_info, c3_emlrt_marshallOut(""), "context", "context", 191);
  sf_mex_addfield(*c3_info, c3_emlrt_marshallOut("simulateRealRobot"), "name",
                  "name", 191);
  sf_mex_addfield(*c3_info, c3_emlrt_marshallOut("struct"), "dominantType",
                  "dominantType", 191);
  sf_mex_addfield(*c3_info, c3_emlrt_marshallOut(
    "[E]C:/Users/sd0016/Dropbox/UAH Doktora/Journal Tracking/Matlab Code/3DOF_Experiments/Mirage/util/simulateRealRobot.m"),
                  "resolved", "resolved", 191);
  sf_mex_addfield(*c3_info, c3_b_emlrt_marshallOut(1417542053U), "fileTimeLo",
                  "fileTimeLo", 191);
  sf_mex_addfield(*c3_info, c3_b_emlrt_marshallOut(0U), "fileTimeHi",
                  "fileTimeHi", 191);
  sf_mex_addfield(*c3_info, c3_b_emlrt_marshallOut(0U), "mFileTimeLo",
                  "mFileTimeLo", 191);
  sf_mex_addfield(*c3_info, c3_b_emlrt_marshallOut(0U), "mFileTimeHi",
                  "mFileTimeHi", 191);
  sf_mex_assign(&c3_rhs191, sf_mex_createcellmatrix(0, 1), false);
  sf_mex_assign(&c3_lhs191, sf_mex_createcellmatrix(0, 1), false);
  sf_mex_addfield(*c3_info, sf_mex_duplicatearraysafe(&c3_rhs191), "rhs", "rhs",
                  191);
  sf_mex_addfield(*c3_info, sf_mex_duplicatearraysafe(&c3_lhs191), "lhs", "lhs",
                  191);
  sf_mex_destroy(&c3_rhs128);
  sf_mex_destroy(&c3_lhs128);
  sf_mex_destroy(&c3_rhs129);
  sf_mex_destroy(&c3_lhs129);
  sf_mex_destroy(&c3_rhs130);
  sf_mex_destroy(&c3_lhs130);
  sf_mex_destroy(&c3_rhs131);
  sf_mex_destroy(&c3_lhs131);
  sf_mex_destroy(&c3_rhs132);
  sf_mex_destroy(&c3_lhs132);
  sf_mex_destroy(&c3_rhs133);
  sf_mex_destroy(&c3_lhs133);
  sf_mex_destroy(&c3_rhs134);
  sf_mex_destroy(&c3_lhs134);
  sf_mex_destroy(&c3_rhs135);
  sf_mex_destroy(&c3_lhs135);
  sf_mex_destroy(&c3_rhs136);
  sf_mex_destroy(&c3_lhs136);
  sf_mex_destroy(&c3_rhs137);
  sf_mex_destroy(&c3_lhs137);
  sf_mex_destroy(&c3_rhs138);
  sf_mex_destroy(&c3_lhs138);
  sf_mex_destroy(&c3_rhs139);
  sf_mex_destroy(&c3_lhs139);
  sf_mex_destroy(&c3_rhs140);
  sf_mex_destroy(&c3_lhs140);
  sf_mex_destroy(&c3_rhs141);
  sf_mex_destroy(&c3_lhs141);
  sf_mex_destroy(&c3_rhs142);
  sf_mex_destroy(&c3_lhs142);
  sf_mex_destroy(&c3_rhs143);
  sf_mex_destroy(&c3_lhs143);
  sf_mex_destroy(&c3_rhs144);
  sf_mex_destroy(&c3_lhs144);
  sf_mex_destroy(&c3_rhs145);
  sf_mex_destroy(&c3_lhs145);
  sf_mex_destroy(&c3_rhs146);
  sf_mex_destroy(&c3_lhs146);
  sf_mex_destroy(&c3_rhs147);
  sf_mex_destroy(&c3_lhs147);
  sf_mex_destroy(&c3_rhs148);
  sf_mex_destroy(&c3_lhs148);
  sf_mex_destroy(&c3_rhs149);
  sf_mex_destroy(&c3_lhs149);
  sf_mex_destroy(&c3_rhs150);
  sf_mex_destroy(&c3_lhs150);
  sf_mex_destroy(&c3_rhs151);
  sf_mex_destroy(&c3_lhs151);
  sf_mex_destroy(&c3_rhs152);
  sf_mex_destroy(&c3_lhs152);
  sf_mex_destroy(&c3_rhs153);
  sf_mex_destroy(&c3_lhs153);
  sf_mex_destroy(&c3_rhs154);
  sf_mex_destroy(&c3_lhs154);
  sf_mex_destroy(&c3_rhs155);
  sf_mex_destroy(&c3_lhs155);
  sf_mex_destroy(&c3_rhs156);
  sf_mex_destroy(&c3_lhs156);
  sf_mex_destroy(&c3_rhs157);
  sf_mex_destroy(&c3_lhs157);
  sf_mex_destroy(&c3_rhs158);
  sf_mex_destroy(&c3_lhs158);
  sf_mex_destroy(&c3_rhs159);
  sf_mex_destroy(&c3_lhs159);
  sf_mex_destroy(&c3_rhs160);
  sf_mex_destroy(&c3_lhs160);
  sf_mex_destroy(&c3_rhs161);
  sf_mex_destroy(&c3_lhs161);
  sf_mex_destroy(&c3_rhs162);
  sf_mex_destroy(&c3_lhs162);
  sf_mex_destroy(&c3_rhs163);
  sf_mex_destroy(&c3_lhs163);
  sf_mex_destroy(&c3_rhs164);
  sf_mex_destroy(&c3_lhs164);
  sf_mex_destroy(&c3_rhs165);
  sf_mex_destroy(&c3_lhs165);
  sf_mex_destroy(&c3_rhs166);
  sf_mex_destroy(&c3_lhs166);
  sf_mex_destroy(&c3_rhs167);
  sf_mex_destroy(&c3_lhs167);
  sf_mex_destroy(&c3_rhs168);
  sf_mex_destroy(&c3_lhs168);
  sf_mex_destroy(&c3_rhs169);
  sf_mex_destroy(&c3_lhs169);
  sf_mex_destroy(&c3_rhs170);
  sf_mex_destroy(&c3_lhs170);
  sf_mex_destroy(&c3_rhs171);
  sf_mex_destroy(&c3_lhs171);
  sf_mex_destroy(&c3_rhs172);
  sf_mex_destroy(&c3_lhs172);
  sf_mex_destroy(&c3_rhs173);
  sf_mex_destroy(&c3_lhs173);
  sf_mex_destroy(&c3_rhs174);
  sf_mex_destroy(&c3_lhs174);
  sf_mex_destroy(&c3_rhs175);
  sf_mex_destroy(&c3_lhs175);
  sf_mex_destroy(&c3_rhs176);
  sf_mex_destroy(&c3_lhs176);
  sf_mex_destroy(&c3_rhs177);
  sf_mex_destroy(&c3_lhs177);
  sf_mex_destroy(&c3_rhs178);
  sf_mex_destroy(&c3_lhs178);
  sf_mex_destroy(&c3_rhs179);
  sf_mex_destroy(&c3_lhs179);
  sf_mex_destroy(&c3_rhs180);
  sf_mex_destroy(&c3_lhs180);
  sf_mex_destroy(&c3_rhs181);
  sf_mex_destroy(&c3_lhs181);
  sf_mex_destroy(&c3_rhs182);
  sf_mex_destroy(&c3_lhs182);
  sf_mex_destroy(&c3_rhs183);
  sf_mex_destroy(&c3_lhs183);
  sf_mex_destroy(&c3_rhs184);
  sf_mex_destroy(&c3_lhs184);
  sf_mex_destroy(&c3_rhs185);
  sf_mex_destroy(&c3_lhs185);
  sf_mex_destroy(&c3_rhs186);
  sf_mex_destroy(&c3_lhs186);
  sf_mex_destroy(&c3_rhs187);
  sf_mex_destroy(&c3_lhs187);
  sf_mex_destroy(&c3_rhs188);
  sf_mex_destroy(&c3_lhs188);
  sf_mex_destroy(&c3_rhs189);
  sf_mex_destroy(&c3_lhs189);
  sf_mex_destroy(&c3_rhs190);
  sf_mex_destroy(&c3_lhs190);
  sf_mex_destroy(&c3_rhs191);
  sf_mex_destroy(&c3_lhs191);
}

static void c3_d_info_helper(const mxArray **c3_info)
{
  const mxArray *c3_rhs192 = NULL;
  const mxArray *c3_lhs192 = NULL;
  const mxArray *c3_rhs193 = NULL;
  const mxArray *c3_lhs193 = NULL;
  const mxArray *c3_rhs194 = NULL;
  const mxArray *c3_lhs194 = NULL;
  const mxArray *c3_rhs195 = NULL;
  const mxArray *c3_lhs195 = NULL;
  const mxArray *c3_rhs196 = NULL;
  const mxArray *c3_lhs196 = NULL;
  const mxArray *c3_rhs197 = NULL;
  const mxArray *c3_lhs197 = NULL;
  const mxArray *c3_rhs198 = NULL;
  const mxArray *c3_lhs198 = NULL;
  const mxArray *c3_rhs199 = NULL;
  const mxArray *c3_lhs199 = NULL;
  const mxArray *c3_rhs200 = NULL;
  const mxArray *c3_lhs200 = NULL;
  const mxArray *c3_rhs201 = NULL;
  const mxArray *c3_lhs201 = NULL;
  const mxArray *c3_rhs202 = NULL;
  const mxArray *c3_lhs202 = NULL;
  const mxArray *c3_rhs203 = NULL;
  const mxArray *c3_lhs203 = NULL;
  const mxArray *c3_rhs204 = NULL;
  const mxArray *c3_lhs204 = NULL;
  const mxArray *c3_rhs205 = NULL;
  const mxArray *c3_lhs205 = NULL;
  const mxArray *c3_rhs206 = NULL;
  const mxArray *c3_lhs206 = NULL;
  const mxArray *c3_rhs207 = NULL;
  const mxArray *c3_lhs207 = NULL;
  const mxArray *c3_rhs208 = NULL;
  const mxArray *c3_lhs208 = NULL;
  const mxArray *c3_rhs209 = NULL;
  const mxArray *c3_lhs209 = NULL;
  const mxArray *c3_rhs210 = NULL;
  const mxArray *c3_lhs210 = NULL;
  const mxArray *c3_rhs211 = NULL;
  const mxArray *c3_lhs211 = NULL;
  const mxArray *c3_rhs212 = NULL;
  const mxArray *c3_lhs212 = NULL;
  const mxArray *c3_rhs213 = NULL;
  const mxArray *c3_lhs213 = NULL;
  const mxArray *c3_rhs214 = NULL;
  const mxArray *c3_lhs214 = NULL;
  const mxArray *c3_rhs215 = NULL;
  const mxArray *c3_lhs215 = NULL;
  const mxArray *c3_rhs216 = NULL;
  const mxArray *c3_lhs216 = NULL;
  const mxArray *c3_rhs217 = NULL;
  const mxArray *c3_lhs217 = NULL;
  const mxArray *c3_rhs218 = NULL;
  const mxArray *c3_lhs218 = NULL;
  const mxArray *c3_rhs219 = NULL;
  const mxArray *c3_lhs219 = NULL;
  const mxArray *c3_rhs220 = NULL;
  const mxArray *c3_lhs220 = NULL;
  const mxArray *c3_rhs221 = NULL;
  const mxArray *c3_lhs221 = NULL;
  const mxArray *c3_rhs222 = NULL;
  const mxArray *c3_lhs222 = NULL;
  const mxArray *c3_rhs223 = NULL;
  const mxArray *c3_lhs223 = NULL;
  const mxArray *c3_rhs224 = NULL;
  const mxArray *c3_lhs224 = NULL;
  const mxArray *c3_rhs225 = NULL;
  const mxArray *c3_lhs225 = NULL;
  const mxArray *c3_rhs226 = NULL;
  const mxArray *c3_lhs226 = NULL;
  const mxArray *c3_rhs227 = NULL;
  const mxArray *c3_lhs227 = NULL;
  const mxArray *c3_rhs228 = NULL;
  const mxArray *c3_lhs228 = NULL;
  const mxArray *c3_rhs229 = NULL;
  const mxArray *c3_lhs229 = NULL;
  const mxArray *c3_rhs230 = NULL;
  const mxArray *c3_lhs230 = NULL;
  const mxArray *c3_rhs231 = NULL;
  const mxArray *c3_lhs231 = NULL;
  const mxArray *c3_rhs232 = NULL;
  const mxArray *c3_lhs232 = NULL;
  const mxArray *c3_rhs233 = NULL;
  const mxArray *c3_lhs233 = NULL;
  const mxArray *c3_rhs234 = NULL;
  const mxArray *c3_lhs234 = NULL;
  const mxArray *c3_rhs235 = NULL;
  const mxArray *c3_lhs235 = NULL;
  const mxArray *c3_rhs236 = NULL;
  const mxArray *c3_lhs236 = NULL;
  const mxArray *c3_rhs237 = NULL;
  const mxArray *c3_lhs237 = NULL;
  const mxArray *c3_rhs238 = NULL;
  const mxArray *c3_lhs238 = NULL;
  const mxArray *c3_rhs239 = NULL;
  const mxArray *c3_lhs239 = NULL;
  const mxArray *c3_rhs240 = NULL;
  const mxArray *c3_lhs240 = NULL;
  const mxArray *c3_rhs241 = NULL;
  const mxArray *c3_lhs241 = NULL;
  const mxArray *c3_rhs242 = NULL;
  const mxArray *c3_lhs242 = NULL;
  const mxArray *c3_rhs243 = NULL;
  const mxArray *c3_lhs243 = NULL;
  const mxArray *c3_rhs244 = NULL;
  const mxArray *c3_lhs244 = NULL;
  const mxArray *c3_rhs245 = NULL;
  const mxArray *c3_lhs245 = NULL;
  const mxArray *c3_rhs246 = NULL;
  const mxArray *c3_lhs246 = NULL;
  const mxArray *c3_rhs247 = NULL;
  const mxArray *c3_lhs247 = NULL;
  const mxArray *c3_rhs248 = NULL;
  const mxArray *c3_lhs248 = NULL;
  const mxArray *c3_rhs249 = NULL;
  const mxArray *c3_lhs249 = NULL;
  const mxArray *c3_rhs250 = NULL;
  const mxArray *c3_lhs250 = NULL;
  const mxArray *c3_rhs251 = NULL;
  const mxArray *c3_lhs251 = NULL;
  const mxArray *c3_rhs252 = NULL;
  const mxArray *c3_lhs252 = NULL;
  const mxArray *c3_rhs253 = NULL;
  const mxArray *c3_lhs253 = NULL;
  const mxArray *c3_rhs254 = NULL;
  const mxArray *c3_lhs254 = NULL;
  const mxArray *c3_rhs255 = NULL;
  const mxArray *c3_lhs255 = NULL;
  sf_mex_addfield(*c3_info, c3_emlrt_marshallOut(
    "[E]C:/Users/sd0016/Dropbox/UAH Doktora/Journal Tracking/Matlab Code/3DOF_Experiments/Mirage/util/simulateRealRobot.m"),
                  "context", "context", 192);
  sf_mex_addfield(*c3_info, c3_emlrt_marshallOut("transformationMatrix"), "name",
                  "name", 192);
  sf_mex_addfield(*c3_info, c3_emlrt_marshallOut("struct"), "dominantType",
                  "dominantType", 192);
  sf_mex_addfield(*c3_info, c3_emlrt_marshallOut(
    "[E]C:/Users/sd0016/Dropbox/UAH Doktora/Journal Tracking/Matlab Code/3DOF_Experiments/Mirage/util/transformationMatrix.m"),
                  "resolved", "resolved", 192);
  sf_mex_addfield(*c3_info, c3_b_emlrt_marshallOut(1428445740U), "fileTimeLo",
                  "fileTimeLo", 192);
  sf_mex_addfield(*c3_info, c3_b_emlrt_marshallOut(0U), "fileTimeHi",
                  "fileTimeHi", 192);
  sf_mex_addfield(*c3_info, c3_b_emlrt_marshallOut(0U), "mFileTimeLo",
                  "mFileTimeLo", 192);
  sf_mex_addfield(*c3_info, c3_b_emlrt_marshallOut(0U), "mFileTimeHi",
                  "mFileTimeHi", 192);
  sf_mex_assign(&c3_rhs192, sf_mex_createcellmatrix(0, 1), false);
  sf_mex_assign(&c3_lhs192, sf_mex_createcellmatrix(0, 1), false);
  sf_mex_addfield(*c3_info, sf_mex_duplicatearraysafe(&c3_rhs192), "rhs", "rhs",
                  192);
  sf_mex_addfield(*c3_info, sf_mex_duplicatearraysafe(&c3_lhs192), "lhs", "lhs",
                  192);
  sf_mex_addfield(*c3_info, c3_emlrt_marshallOut(
    "[E]C:/Users/sd0016/Dropbox/UAH Doktora/Journal Tracking/Matlab Code/3DOF_Experiments/Mirage/util/simulateRealRobot.m"),
                  "context", "context", 193);
  sf_mex_addfield(*c3_info, c3_emlrt_marshallOut("inv"), "name", "name", 193);
  sf_mex_addfield(*c3_info, c3_emlrt_marshallOut("double"), "dominantType",
                  "dominantType", 193);
  sf_mex_addfield(*c3_info, c3_emlrt_marshallOut(
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/matfun/inv.m"), "resolved",
                  "resolved", 193);
  sf_mex_addfield(*c3_info, c3_b_emlrt_marshallOut(1305343200U), "fileTimeLo",
                  "fileTimeLo", 193);
  sf_mex_addfield(*c3_info, c3_b_emlrt_marshallOut(0U), "fileTimeHi",
                  "fileTimeHi", 193);
  sf_mex_addfield(*c3_info, c3_b_emlrt_marshallOut(0U), "mFileTimeLo",
                  "mFileTimeLo", 193);
  sf_mex_addfield(*c3_info, c3_b_emlrt_marshallOut(0U), "mFileTimeHi",
                  "mFileTimeHi", 193);
  sf_mex_assign(&c3_rhs193, sf_mex_createcellmatrix(0, 1), false);
  sf_mex_assign(&c3_lhs193, sf_mex_createcellmatrix(0, 1), false);
  sf_mex_addfield(*c3_info, sf_mex_duplicatearraysafe(&c3_rhs193), "rhs", "rhs",
                  193);
  sf_mex_addfield(*c3_info, sf_mex_duplicatearraysafe(&c3_lhs193), "lhs", "lhs",
                  193);
  sf_mex_addfield(*c3_info, c3_emlrt_marshallOut(
    "[E]C:/Users/sd0016/Dropbox/UAH Doktora/Journal Tracking/Matlab Code/3DOF_Experiments/Mirage/util/simulateRealRobot.m"),
                  "context", "context", 194);
  sf_mex_addfield(*c3_info, c3_emlrt_marshallOut("eml_mtimes_helper"), "name",
                  "name", 194);
  sf_mex_addfield(*c3_info, c3_emlrt_marshallOut(""), "dominantType",
                  "dominantType", 194);
  sf_mex_addfield(*c3_info, c3_emlrt_marshallOut(
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/ops/eml_mtimes_helper.m"),
                  "resolved", "resolved", 194);
  sf_mex_addfield(*c3_info, c3_b_emlrt_marshallOut(1383902494U), "fileTimeLo",
                  "fileTimeLo", 194);
  sf_mex_addfield(*c3_info, c3_b_emlrt_marshallOut(0U), "fileTimeHi",
                  "fileTimeHi", 194);
  sf_mex_addfield(*c3_info, c3_b_emlrt_marshallOut(0U), "mFileTimeLo",
                  "mFileTimeLo", 194);
  sf_mex_addfield(*c3_info, c3_b_emlrt_marshallOut(0U), "mFileTimeHi",
                  "mFileTimeHi", 194);
  sf_mex_assign(&c3_rhs194, sf_mex_createcellmatrix(0, 1), false);
  sf_mex_assign(&c3_lhs194, sf_mex_createcellmatrix(0, 1), false);
  sf_mex_addfield(*c3_info, sf_mex_duplicatearraysafe(&c3_rhs194), "rhs", "rhs",
                  194);
  sf_mex_addfield(*c3_info, sf_mex_duplicatearraysafe(&c3_lhs194), "lhs", "lhs",
                  194);
  sf_mex_addfield(*c3_info, c3_emlrt_marshallOut(
    "[E]C:/Users/sd0016/Dropbox/UAH Doktora/Journal Tracking/Matlab Code/3DOF_Experiments/Mirage/util/simulateRealRobot.m"),
                  "context", "context", 195);
  sf_mex_addfield(*c3_info, c3_emlrt_marshallOut("transformImageSpace"), "name",
                  "name", 195);
  sf_mex_addfield(*c3_info, c3_emlrt_marshallOut("struct"), "dominantType",
                  "dominantType", 195);
  sf_mex_addfield(*c3_info, c3_emlrt_marshallOut(
    "[E]C:/Users/sd0016/Dropbox/UAH Doktora/Journal Tracking/Matlab Code/3DOF_Experiments/Mirage/util/transformImageSpace.m"),
                  "resolved", "resolved", 195);
  sf_mex_addfield(*c3_info, c3_b_emlrt_marshallOut(1428445839U), "fileTimeLo",
                  "fileTimeLo", 195);
  sf_mex_addfield(*c3_info, c3_b_emlrt_marshallOut(0U), "fileTimeHi",
                  "fileTimeHi", 195);
  sf_mex_addfield(*c3_info, c3_b_emlrt_marshallOut(0U), "mFileTimeLo",
                  "mFileTimeLo", 195);
  sf_mex_addfield(*c3_info, c3_b_emlrt_marshallOut(0U), "mFileTimeHi",
                  "mFileTimeHi", 195);
  sf_mex_assign(&c3_rhs195, sf_mex_createcellmatrix(0, 1), false);
  sf_mex_assign(&c3_lhs195, sf_mex_createcellmatrix(0, 1), false);
  sf_mex_addfield(*c3_info, sf_mex_duplicatearraysafe(&c3_rhs195), "rhs", "rhs",
                  195);
  sf_mex_addfield(*c3_info, sf_mex_duplicatearraysafe(&c3_lhs195), "lhs", "lhs",
                  195);
  sf_mex_addfield(*c3_info, c3_emlrt_marshallOut(
    "[E]C:/Users/sd0016/Dropbox/UAH Doktora/Journal Tracking/Matlab Code/3DOF_Experiments/Mirage/util/transformImageSpace.m"),
                  "context", "context", 196);
  sf_mex_addfield(*c3_info, c3_emlrt_marshallOut("transformationMatrix"), "name",
                  "name", 196);
  sf_mex_addfield(*c3_info, c3_emlrt_marshallOut("struct"), "dominantType",
                  "dominantType", 196);
  sf_mex_addfield(*c3_info, c3_emlrt_marshallOut(
    "[E]C:/Users/sd0016/Dropbox/UAH Doktora/Journal Tracking/Matlab Code/3DOF_Experiments/Mirage/util/transformationMatrix.m"),
                  "resolved", "resolved", 196);
  sf_mex_addfield(*c3_info, c3_b_emlrt_marshallOut(1428445740U), "fileTimeLo",
                  "fileTimeLo", 196);
  sf_mex_addfield(*c3_info, c3_b_emlrt_marshallOut(0U), "fileTimeHi",
                  "fileTimeHi", 196);
  sf_mex_addfield(*c3_info, c3_b_emlrt_marshallOut(0U), "mFileTimeLo",
                  "mFileTimeLo", 196);
  sf_mex_addfield(*c3_info, c3_b_emlrt_marshallOut(0U), "mFileTimeHi",
                  "mFileTimeHi", 196);
  sf_mex_assign(&c3_rhs196, sf_mex_createcellmatrix(0, 1), false);
  sf_mex_assign(&c3_lhs196, sf_mex_createcellmatrix(0, 1), false);
  sf_mex_addfield(*c3_info, sf_mex_duplicatearraysafe(&c3_rhs196), "rhs", "rhs",
                  196);
  sf_mex_addfield(*c3_info, sf_mex_duplicatearraysafe(&c3_lhs196), "lhs", "lhs",
                  196);
  sf_mex_addfield(*c3_info, c3_emlrt_marshallOut(
    "[E]C:/Users/sd0016/Dropbox/UAH Doktora/Journal Tracking/Matlab Code/3DOF_Experiments/Mirage/util/transformImageSpace.m"),
                  "context", "context", 197);
  sf_mex_addfield(*c3_info, c3_emlrt_marshallOut("mpower"), "name", "name", 197);
  sf_mex_addfield(*c3_info, c3_emlrt_marshallOut("double"), "dominantType",
                  "dominantType", 197);
  sf_mex_addfield(*c3_info, c3_emlrt_marshallOut(
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/ops/mpower.m"), "resolved",
                  "resolved", 197);
  sf_mex_addfield(*c3_info, c3_b_emlrt_marshallOut(1363735478U), "fileTimeLo",
                  "fileTimeLo", 197);
  sf_mex_addfield(*c3_info, c3_b_emlrt_marshallOut(0U), "fileTimeHi",
                  "fileTimeHi", 197);
  sf_mex_addfield(*c3_info, c3_b_emlrt_marshallOut(0U), "mFileTimeLo",
                  "mFileTimeLo", 197);
  sf_mex_addfield(*c3_info, c3_b_emlrt_marshallOut(0U), "mFileTimeHi",
                  "mFileTimeHi", 197);
  sf_mex_assign(&c3_rhs197, sf_mex_createcellmatrix(0, 1), false);
  sf_mex_assign(&c3_lhs197, sf_mex_createcellmatrix(0, 1), false);
  sf_mex_addfield(*c3_info, sf_mex_duplicatearraysafe(&c3_rhs197), "rhs", "rhs",
                  197);
  sf_mex_addfield(*c3_info, sf_mex_duplicatearraysafe(&c3_lhs197), "lhs", "lhs",
                  197);
  sf_mex_addfield(*c3_info, c3_emlrt_marshallOut(
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/ops/mpower.m"), "context",
                  "context", 198);
  sf_mex_addfield(*c3_info, c3_emlrt_marshallOut(
    "coder.internal.isBuiltInNumeric"), "name", "name", 198);
  sf_mex_addfield(*c3_info, c3_emlrt_marshallOut("double"), "dominantType",
                  "dominantType", 198);
  sf_mex_addfield(*c3_info, c3_emlrt_marshallOut(
    "[ILXE]$matlabroot$/toolbox/shared/coder/coder/+coder/+internal/isBuiltInNumeric.m"),
                  "resolved", "resolved", 198);
  sf_mex_addfield(*c3_info, c3_b_emlrt_marshallOut(1395953456U), "fileTimeLo",
                  "fileTimeLo", 198);
  sf_mex_addfield(*c3_info, c3_b_emlrt_marshallOut(0U), "fileTimeHi",
                  "fileTimeHi", 198);
  sf_mex_addfield(*c3_info, c3_b_emlrt_marshallOut(0U), "mFileTimeLo",
                  "mFileTimeLo", 198);
  sf_mex_addfield(*c3_info, c3_b_emlrt_marshallOut(0U), "mFileTimeHi",
                  "mFileTimeHi", 198);
  sf_mex_assign(&c3_rhs198, sf_mex_createcellmatrix(0, 1), false);
  sf_mex_assign(&c3_lhs198, sf_mex_createcellmatrix(0, 1), false);
  sf_mex_addfield(*c3_info, sf_mex_duplicatearraysafe(&c3_rhs198), "rhs", "rhs",
                  198);
  sf_mex_addfield(*c3_info, sf_mex_duplicatearraysafe(&c3_lhs198), "lhs", "lhs",
                  198);
  sf_mex_addfield(*c3_info, c3_emlrt_marshallOut(
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/ops/mpower.m"), "context",
                  "context", 199);
  sf_mex_addfield(*c3_info, c3_emlrt_marshallOut("ismatrix"), "name", "name",
                  199);
  sf_mex_addfield(*c3_info, c3_emlrt_marshallOut("double"), "dominantType",
                  "dominantType", 199);
  sf_mex_addfield(*c3_info, c3_emlrt_marshallOut(
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/elmat/ismatrix.m"), "resolved",
                  "resolved", 199);
  sf_mex_addfield(*c3_info, c3_b_emlrt_marshallOut(1331330058U), "fileTimeLo",
                  "fileTimeLo", 199);
  sf_mex_addfield(*c3_info, c3_b_emlrt_marshallOut(0U), "fileTimeHi",
                  "fileTimeHi", 199);
  sf_mex_addfield(*c3_info, c3_b_emlrt_marshallOut(0U), "mFileTimeLo",
                  "mFileTimeLo", 199);
  sf_mex_addfield(*c3_info, c3_b_emlrt_marshallOut(0U), "mFileTimeHi",
                  "mFileTimeHi", 199);
  sf_mex_assign(&c3_rhs199, sf_mex_createcellmatrix(0, 1), false);
  sf_mex_assign(&c3_lhs199, sf_mex_createcellmatrix(0, 1), false);
  sf_mex_addfield(*c3_info, sf_mex_duplicatearraysafe(&c3_rhs199), "rhs", "rhs",
                  199);
  sf_mex_addfield(*c3_info, sf_mex_duplicatearraysafe(&c3_lhs199), "lhs", "lhs",
                  199);
  sf_mex_addfield(*c3_info, c3_emlrt_marshallOut(
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/ops/mpower.m"), "context",
                  "context", 200);
  sf_mex_addfield(*c3_info, c3_emlrt_marshallOut("eml_scalar_floor"), "name",
                  "name", 200);
  sf_mex_addfield(*c3_info, c3_emlrt_marshallOut("double"), "dominantType",
                  "dominantType", 200);
  sf_mex_addfield(*c3_info, c3_emlrt_marshallOut(
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/elfun/eml_scalar_floor.m"),
                  "resolved", "resolved", 200);
  sf_mex_addfield(*c3_info, c3_b_emlrt_marshallOut(1286843926U), "fileTimeLo",
                  "fileTimeLo", 200);
  sf_mex_addfield(*c3_info, c3_b_emlrt_marshallOut(0U), "fileTimeHi",
                  "fileTimeHi", 200);
  sf_mex_addfield(*c3_info, c3_b_emlrt_marshallOut(0U), "mFileTimeLo",
                  "mFileTimeLo", 200);
  sf_mex_addfield(*c3_info, c3_b_emlrt_marshallOut(0U), "mFileTimeHi",
                  "mFileTimeHi", 200);
  sf_mex_assign(&c3_rhs200, sf_mex_createcellmatrix(0, 1), false);
  sf_mex_assign(&c3_lhs200, sf_mex_createcellmatrix(0, 1), false);
  sf_mex_addfield(*c3_info, sf_mex_duplicatearraysafe(&c3_rhs200), "rhs", "rhs",
                  200);
  sf_mex_addfield(*c3_info, sf_mex_duplicatearraysafe(&c3_lhs200), "lhs", "lhs",
                  200);
  sf_mex_addfield(*c3_info, c3_emlrt_marshallOut(
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/ops/mpower.m!matrix_to_integer_power"),
                  "context", "context", 201);
  sf_mex_addfield(*c3_info, c3_emlrt_marshallOut("eml_scalar_eg"), "name",
                  "name", 201);
  sf_mex_addfield(*c3_info, c3_emlrt_marshallOut("double"), "dominantType",
                  "dominantType", 201);
  sf_mex_addfield(*c3_info, c3_emlrt_marshallOut(
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/eml/eml_scalar_eg.m"), "resolved",
                  "resolved", 201);
  sf_mex_addfield(*c3_info, c3_b_emlrt_marshallOut(1376005888U), "fileTimeLo",
                  "fileTimeLo", 201);
  sf_mex_addfield(*c3_info, c3_b_emlrt_marshallOut(0U), "fileTimeHi",
                  "fileTimeHi", 201);
  sf_mex_addfield(*c3_info, c3_b_emlrt_marshallOut(0U), "mFileTimeLo",
                  "mFileTimeLo", 201);
  sf_mex_addfield(*c3_info, c3_b_emlrt_marshallOut(0U), "mFileTimeHi",
                  "mFileTimeHi", 201);
  sf_mex_assign(&c3_rhs201, sf_mex_createcellmatrix(0, 1), false);
  sf_mex_assign(&c3_lhs201, sf_mex_createcellmatrix(0, 1), false);
  sf_mex_addfield(*c3_info, sf_mex_duplicatearraysafe(&c3_rhs201), "rhs", "rhs",
                  201);
  sf_mex_addfield(*c3_info, sf_mex_duplicatearraysafe(&c3_lhs201), "lhs", "lhs",
                  201);
  sf_mex_addfield(*c3_info, c3_emlrt_marshallOut(
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/ops/mpower.m!matrix_to_integer_power"),
                  "context", "context", 202);
  sf_mex_addfield(*c3_info, c3_emlrt_marshallOut("inv"), "name", "name", 202);
  sf_mex_addfield(*c3_info, c3_emlrt_marshallOut("double"), "dominantType",
                  "dominantType", 202);
  sf_mex_addfield(*c3_info, c3_emlrt_marshallOut(
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/matfun/inv.m"), "resolved",
                  "resolved", 202);
  sf_mex_addfield(*c3_info, c3_b_emlrt_marshallOut(1305343200U), "fileTimeLo",
                  "fileTimeLo", 202);
  sf_mex_addfield(*c3_info, c3_b_emlrt_marshallOut(0U), "fileTimeHi",
                  "fileTimeHi", 202);
  sf_mex_addfield(*c3_info, c3_b_emlrt_marshallOut(0U), "mFileTimeLo",
                  "mFileTimeLo", 202);
  sf_mex_addfield(*c3_info, c3_b_emlrt_marshallOut(0U), "mFileTimeHi",
                  "mFileTimeHi", 202);
  sf_mex_assign(&c3_rhs202, sf_mex_createcellmatrix(0, 1), false);
  sf_mex_assign(&c3_lhs202, sf_mex_createcellmatrix(0, 1), false);
  sf_mex_addfield(*c3_info, sf_mex_duplicatearraysafe(&c3_rhs202), "rhs", "rhs",
                  202);
  sf_mex_addfield(*c3_info, sf_mex_duplicatearraysafe(&c3_lhs202), "lhs", "lhs",
                  202);
  sf_mex_addfield(*c3_info, c3_emlrt_marshallOut(
    "[E]C:/Users/sd0016/Dropbox/UAH Doktora/Journal Tracking/Matlab Code/3DOF_Experiments/Mirage/util/transformImageSpace.m"),
                  "context", "context", 203);
  sf_mex_addfield(*c3_info, c3_emlrt_marshallOut("eml_mtimes_helper"), "name",
                  "name", 203);
  sf_mex_addfield(*c3_info, c3_emlrt_marshallOut(""), "dominantType",
                  "dominantType", 203);
  sf_mex_addfield(*c3_info, c3_emlrt_marshallOut(
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/ops/eml_mtimes_helper.m"),
                  "resolved", "resolved", 203);
  sf_mex_addfield(*c3_info, c3_b_emlrt_marshallOut(1383902494U), "fileTimeLo",
                  "fileTimeLo", 203);
  sf_mex_addfield(*c3_info, c3_b_emlrt_marshallOut(0U), "fileTimeHi",
                  "fileTimeHi", 203);
  sf_mex_addfield(*c3_info, c3_b_emlrt_marshallOut(0U), "mFileTimeLo",
                  "mFileTimeLo", 203);
  sf_mex_addfield(*c3_info, c3_b_emlrt_marshallOut(0U), "mFileTimeHi",
                  "mFileTimeHi", 203);
  sf_mex_assign(&c3_rhs203, sf_mex_createcellmatrix(0, 1), false);
  sf_mex_assign(&c3_lhs203, sf_mex_createcellmatrix(0, 1), false);
  sf_mex_addfield(*c3_info, sf_mex_duplicatearraysafe(&c3_rhs203), "rhs", "rhs",
                  203);
  sf_mex_addfield(*c3_info, sf_mex_duplicatearraysafe(&c3_lhs203), "lhs", "lhs",
                  203);
  sf_mex_addfield(*c3_info, c3_emlrt_marshallOut(
    "[E]C:/Users/sd0016/Dropbox/UAH Doktora/Journal Tracking/Matlab Code/3DOF_Experiments/Mirage/util/transformImageSpace.m"),
                  "context", "context", 204);
  sf_mex_addfield(*c3_info, c3_emlrt_marshallOut("intrinsicMatrix"), "name",
                  "name", 204);
  sf_mex_addfield(*c3_info, c3_emlrt_marshallOut("struct"), "dominantType",
                  "dominantType", 204);
  sf_mex_addfield(*c3_info, c3_emlrt_marshallOut(
    "[E]C:/Users/sd0016/Dropbox/UAH Doktora/Journal Tracking/Matlab Code/3DOF_Experiments/Mirage/util/intrinsicMatrix.m"),
                  "resolved", "resolved", 204);
  sf_mex_addfield(*c3_info, c3_b_emlrt_marshallOut(1417491920U), "fileTimeLo",
                  "fileTimeLo", 204);
  sf_mex_addfield(*c3_info, c3_b_emlrt_marshallOut(0U), "fileTimeHi",
                  "fileTimeHi", 204);
  sf_mex_addfield(*c3_info, c3_b_emlrt_marshallOut(0U), "mFileTimeLo",
                  "mFileTimeLo", 204);
  sf_mex_addfield(*c3_info, c3_b_emlrt_marshallOut(0U), "mFileTimeHi",
                  "mFileTimeHi", 204);
  sf_mex_assign(&c3_rhs204, sf_mex_createcellmatrix(0, 1), false);
  sf_mex_assign(&c3_lhs204, sf_mex_createcellmatrix(0, 1), false);
  sf_mex_addfield(*c3_info, sf_mex_duplicatearraysafe(&c3_rhs204), "rhs", "rhs",
                  204);
  sf_mex_addfield(*c3_info, sf_mex_duplicatearraysafe(&c3_lhs204), "lhs", "lhs",
                  204);
  sf_mex_addfield(*c3_info, c3_emlrt_marshallOut(
    "[E]C:/Users/sd0016/Dropbox/UAH Doktora/Journal Tracking/Matlab Code/3DOF_Experiments/Mirage/util/transformImageSpace.m"),
                  "context", "context", 205);
  sf_mex_addfield(*c3_info, c3_emlrt_marshallOut("rdivide"), "name", "name", 205);
  sf_mex_addfield(*c3_info, c3_emlrt_marshallOut("double"), "dominantType",
                  "dominantType", 205);
  sf_mex_addfield(*c3_info, c3_emlrt_marshallOut(
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/ops/rdivide.m"), "resolved",
                  "resolved", 205);
  sf_mex_addfield(*c3_info, c3_b_emlrt_marshallOut(1363735480U), "fileTimeLo",
                  "fileTimeLo", 205);
  sf_mex_addfield(*c3_info, c3_b_emlrt_marshallOut(0U), "fileTimeHi",
                  "fileTimeHi", 205);
  sf_mex_addfield(*c3_info, c3_b_emlrt_marshallOut(0U), "mFileTimeLo",
                  "mFileTimeLo", 205);
  sf_mex_addfield(*c3_info, c3_b_emlrt_marshallOut(0U), "mFileTimeHi",
                  "mFileTimeHi", 205);
  sf_mex_assign(&c3_rhs205, sf_mex_createcellmatrix(0, 1), false);
  sf_mex_assign(&c3_lhs205, sf_mex_createcellmatrix(0, 1), false);
  sf_mex_addfield(*c3_info, sf_mex_duplicatearraysafe(&c3_rhs205), "rhs", "rhs",
                  205);
  sf_mex_addfield(*c3_info, sf_mex_duplicatearraysafe(&c3_lhs205), "lhs", "lhs",
                  205);
  sf_mex_addfield(*c3_info, c3_emlrt_marshallOut(
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/eml/eml_scalexp_compatible.m"),
                  "context", "context", 206);
  sf_mex_addfield(*c3_info, c3_emlrt_marshallOut("isequal"), "name", "name", 206);
  sf_mex_addfield(*c3_info, c3_emlrt_marshallOut("double"), "dominantType",
                  "dominantType", 206);
  sf_mex_addfield(*c3_info, c3_emlrt_marshallOut(
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/elmat/isequal.m"), "resolved",
                  "resolved", 206);
  sf_mex_addfield(*c3_info, c3_b_emlrt_marshallOut(1286843958U), "fileTimeLo",
                  "fileTimeLo", 206);
  sf_mex_addfield(*c3_info, c3_b_emlrt_marshallOut(0U), "fileTimeHi",
                  "fileTimeHi", 206);
  sf_mex_addfield(*c3_info, c3_b_emlrt_marshallOut(0U), "mFileTimeLo",
                  "mFileTimeLo", 206);
  sf_mex_addfield(*c3_info, c3_b_emlrt_marshallOut(0U), "mFileTimeHi",
                  "mFileTimeHi", 206);
  sf_mex_assign(&c3_rhs206, sf_mex_createcellmatrix(0, 1), false);
  sf_mex_assign(&c3_lhs206, sf_mex_createcellmatrix(0, 1), false);
  sf_mex_addfield(*c3_info, sf_mex_duplicatearraysafe(&c3_rhs206), "rhs", "rhs",
                  206);
  sf_mex_addfield(*c3_info, sf_mex_duplicatearraysafe(&c3_lhs206), "lhs", "lhs",
                  206);
  sf_mex_addfield(*c3_info, c3_emlrt_marshallOut(""), "context", "context", 207);
  sf_mex_addfield(*c3_info, c3_emlrt_marshallOut("mirage"), "name", "name", 207);
  sf_mex_addfield(*c3_info, c3_emlrt_marshallOut("struct"), "dominantType",
                  "dominantType", 207);
  sf_mex_addfield(*c3_info, c3_emlrt_marshallOut(
    "[E]C:/Users/sd0016/Dropbox/UAH Doktora/Journal Tracking/Matlab Code/3DOF_Experiments/Mirage/mirage.m"),
                  "resolved", "resolved", 207);
  sf_mex_addfield(*c3_info, c3_b_emlrt_marshallOut(1428437021U), "fileTimeLo",
                  "fileTimeLo", 207);
  sf_mex_addfield(*c3_info, c3_b_emlrt_marshallOut(0U), "fileTimeHi",
                  "fileTimeHi", 207);
  sf_mex_addfield(*c3_info, c3_b_emlrt_marshallOut(0U), "mFileTimeLo",
                  "mFileTimeLo", 207);
  sf_mex_addfield(*c3_info, c3_b_emlrt_marshallOut(0U), "mFileTimeHi",
                  "mFileTimeHi", 207);
  sf_mex_assign(&c3_rhs207, sf_mex_createcellmatrix(0, 1), false);
  sf_mex_assign(&c3_lhs207, sf_mex_createcellmatrix(0, 1), false);
  sf_mex_addfield(*c3_info, sf_mex_duplicatearraysafe(&c3_rhs207), "rhs", "rhs",
                  207);
  sf_mex_addfield(*c3_info, sf_mex_duplicatearraysafe(&c3_lhs207), "lhs", "lhs",
                  207);
  sf_mex_addfield(*c3_info, c3_emlrt_marshallOut(
    "[E]C:/Users/sd0016/Dropbox/UAH Doktora/Journal Tracking/Matlab Code/3DOF_Experiments/Mirage/mirage.m"),
                  "context", "context", 208);
  sf_mex_addfield(*c3_info, c3_emlrt_marshallOut("transformImageSpace"), "name",
                  "name", 208);
  sf_mex_addfield(*c3_info, c3_emlrt_marshallOut("struct"), "dominantType",
                  "dominantType", 208);
  sf_mex_addfield(*c3_info, c3_emlrt_marshallOut(
    "[E]C:/Users/sd0016/Dropbox/UAH Doktora/Journal Tracking/Matlab Code/3DOF_Experiments/Mirage/util/transformImageSpace.m"),
                  "resolved", "resolved", 208);
  sf_mex_addfield(*c3_info, c3_b_emlrt_marshallOut(1428445839U), "fileTimeLo",
                  "fileTimeLo", 208);
  sf_mex_addfield(*c3_info, c3_b_emlrt_marshallOut(0U), "fileTimeHi",
                  "fileTimeHi", 208);
  sf_mex_addfield(*c3_info, c3_b_emlrt_marshallOut(0U), "mFileTimeLo",
                  "mFileTimeLo", 208);
  sf_mex_addfield(*c3_info, c3_b_emlrt_marshallOut(0U), "mFileTimeHi",
                  "mFileTimeHi", 208);
  sf_mex_assign(&c3_rhs208, sf_mex_createcellmatrix(0, 1), false);
  sf_mex_assign(&c3_lhs208, sf_mex_createcellmatrix(0, 1), false);
  sf_mex_addfield(*c3_info, sf_mex_duplicatearraysafe(&c3_rhs208), "rhs", "rhs",
                  208);
  sf_mex_addfield(*c3_info, sf_mex_duplicatearraysafe(&c3_lhs208), "lhs", "lhs",
                  208);
  sf_mex_addfield(*c3_info, c3_emlrt_marshallOut(
    "[E]C:/Users/sd0016/Dropbox/UAH Doktora/Journal Tracking/Matlab Code/3DOF_Experiments/Mirage/mirage.m"),
                  "context", "context", 209);
  sf_mex_addfield(*c3_info, c3_emlrt_marshallOut("getEquations"), "name", "name",
                  209);
  sf_mex_addfield(*c3_info, c3_emlrt_marshallOut("double"), "dominantType",
                  "dominantType", 209);
  sf_mex_addfield(*c3_info, c3_emlrt_marshallOut(
    "[E]C:/Users/sd0016/Dropbox/UAH Doktora/Journal Tracking/Matlab Code/3DOF_Experiments/getEquations.m"),
                  "resolved", "resolved", 209);
  sf_mex_addfield(*c3_info, c3_b_emlrt_marshallOut(1333640326U), "fileTimeLo",
                  "fileTimeLo", 209);
  sf_mex_addfield(*c3_info, c3_b_emlrt_marshallOut(0U), "fileTimeHi",
                  "fileTimeHi", 209);
  sf_mex_addfield(*c3_info, c3_b_emlrt_marshallOut(0U), "mFileTimeLo",
                  "mFileTimeLo", 209);
  sf_mex_addfield(*c3_info, c3_b_emlrt_marshallOut(0U), "mFileTimeHi",
                  "mFileTimeHi", 209);
  sf_mex_assign(&c3_rhs209, sf_mex_createcellmatrix(0, 1), false);
  sf_mex_assign(&c3_lhs209, sf_mex_createcellmatrix(0, 1), false);
  sf_mex_addfield(*c3_info, sf_mex_duplicatearraysafe(&c3_rhs209), "rhs", "rhs",
                  209);
  sf_mex_addfield(*c3_info, sf_mex_duplicatearraysafe(&c3_lhs209), "lhs", "lhs",
                  209);
  sf_mex_addfield(*c3_info, c3_emlrt_marshallOut(
    "[E]C:/Users/sd0016/Dropbox/UAH Doktora/Journal Tracking/Matlab Code/3DOF_Experiments/getEquations.m"),
                  "context", "context", 210);
  sf_mex_addfield(*c3_info, c3_emlrt_marshallOut("eml_mtimes_helper"), "name",
                  "name", 210);
  sf_mex_addfield(*c3_info, c3_emlrt_marshallOut(""), "dominantType",
                  "dominantType", 210);
  sf_mex_addfield(*c3_info, c3_emlrt_marshallOut(
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/ops/eml_mtimes_helper.m"),
                  "resolved", "resolved", 210);
  sf_mex_addfield(*c3_info, c3_b_emlrt_marshallOut(1383902494U), "fileTimeLo",
                  "fileTimeLo", 210);
  sf_mex_addfield(*c3_info, c3_b_emlrt_marshallOut(0U), "fileTimeHi",
                  "fileTimeHi", 210);
  sf_mex_addfield(*c3_info, c3_b_emlrt_marshallOut(0U), "mFileTimeLo",
                  "mFileTimeLo", 210);
  sf_mex_addfield(*c3_info, c3_b_emlrt_marshallOut(0U), "mFileTimeHi",
                  "mFileTimeHi", 210);
  sf_mex_assign(&c3_rhs210, sf_mex_createcellmatrix(0, 1), false);
  sf_mex_assign(&c3_lhs210, sf_mex_createcellmatrix(0, 1), false);
  sf_mex_addfield(*c3_info, sf_mex_duplicatearraysafe(&c3_rhs210), "rhs", "rhs",
                  210);
  sf_mex_addfield(*c3_info, sf_mex_duplicatearraysafe(&c3_lhs210), "lhs", "lhs",
                  210);
  sf_mex_addfield(*c3_info, c3_emlrt_marshallOut(
    "[E]C:/Users/sd0016/Dropbox/UAH Doktora/Journal Tracking/Matlab Code/3DOF_Experiments/getEquations.m"),
                  "context", "context", 211);
  sf_mex_addfield(*c3_info, c3_emlrt_marshallOut("mrdivide"), "name", "name",
                  211);
  sf_mex_addfield(*c3_info, c3_emlrt_marshallOut("double"), "dominantType",
                  "dominantType", 211);
  sf_mex_addfield(*c3_info, c3_emlrt_marshallOut(
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/ops/mrdivide.p"), "resolved",
                  "resolved", 211);
  sf_mex_addfield(*c3_info, c3_b_emlrt_marshallOut(1410832848U), "fileTimeLo",
                  "fileTimeLo", 211);
  sf_mex_addfield(*c3_info, c3_b_emlrt_marshallOut(0U), "fileTimeHi",
                  "fileTimeHi", 211);
  sf_mex_addfield(*c3_info, c3_b_emlrt_marshallOut(1370035086U), "mFileTimeLo",
                  "mFileTimeLo", 211);
  sf_mex_addfield(*c3_info, c3_b_emlrt_marshallOut(0U), "mFileTimeHi",
                  "mFileTimeHi", 211);
  sf_mex_assign(&c3_rhs211, sf_mex_createcellmatrix(0, 1), false);
  sf_mex_assign(&c3_lhs211, sf_mex_createcellmatrix(0, 1), false);
  sf_mex_addfield(*c3_info, sf_mex_duplicatearraysafe(&c3_rhs211), "rhs", "rhs",
                  211);
  sf_mex_addfield(*c3_info, sf_mex_duplicatearraysafe(&c3_lhs211), "lhs", "lhs",
                  211);
  sf_mex_addfield(*c3_info, c3_emlrt_marshallOut(
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/ops/mrdivide.p"), "context",
                  "context", 212);
  sf_mex_addfield(*c3_info, c3_emlrt_marshallOut("coder.internal.assert"),
                  "name", "name", 212);
  sf_mex_addfield(*c3_info, c3_emlrt_marshallOut("char"), "dominantType",
                  "dominantType", 212);
  sf_mex_addfield(*c3_info, c3_emlrt_marshallOut(
    "[ILXE]$matlabroot$/toolbox/shared/coder/coder/+coder/+internal/assert.m"),
                  "resolved", "resolved", 212);
  sf_mex_addfield(*c3_info, c3_b_emlrt_marshallOut(1389742974U), "fileTimeLo",
                  "fileTimeLo", 212);
  sf_mex_addfield(*c3_info, c3_b_emlrt_marshallOut(0U), "fileTimeHi",
                  "fileTimeHi", 212);
  sf_mex_addfield(*c3_info, c3_b_emlrt_marshallOut(0U), "mFileTimeLo",
                  "mFileTimeLo", 212);
  sf_mex_addfield(*c3_info, c3_b_emlrt_marshallOut(0U), "mFileTimeHi",
                  "mFileTimeHi", 212);
  sf_mex_assign(&c3_rhs212, sf_mex_createcellmatrix(0, 1), false);
  sf_mex_assign(&c3_lhs212, sf_mex_createcellmatrix(0, 1), false);
  sf_mex_addfield(*c3_info, sf_mex_duplicatearraysafe(&c3_rhs212), "rhs", "rhs",
                  212);
  sf_mex_addfield(*c3_info, sf_mex_duplicatearraysafe(&c3_lhs212), "lhs", "lhs",
                  212);
  sf_mex_addfield(*c3_info, c3_emlrt_marshallOut(
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/ops/mrdivide.p"), "context",
                  "context", 213);
  sf_mex_addfield(*c3_info, c3_emlrt_marshallOut("rdivide"), "name", "name", 213);
  sf_mex_addfield(*c3_info, c3_emlrt_marshallOut("double"), "dominantType",
                  "dominantType", 213);
  sf_mex_addfield(*c3_info, c3_emlrt_marshallOut(
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/ops/rdivide.m"), "resolved",
                  "resolved", 213);
  sf_mex_addfield(*c3_info, c3_b_emlrt_marshallOut(1363735480U), "fileTimeLo",
                  "fileTimeLo", 213);
  sf_mex_addfield(*c3_info, c3_b_emlrt_marshallOut(0U), "fileTimeHi",
                  "fileTimeHi", 213);
  sf_mex_addfield(*c3_info, c3_b_emlrt_marshallOut(0U), "mFileTimeLo",
                  "mFileTimeLo", 213);
  sf_mex_addfield(*c3_info, c3_b_emlrt_marshallOut(0U), "mFileTimeHi",
                  "mFileTimeHi", 213);
  sf_mex_assign(&c3_rhs213, sf_mex_createcellmatrix(0, 1), false);
  sf_mex_assign(&c3_lhs213, sf_mex_createcellmatrix(0, 1), false);
  sf_mex_addfield(*c3_info, sf_mex_duplicatearraysafe(&c3_rhs213), "rhs", "rhs",
                  213);
  sf_mex_addfield(*c3_info, sf_mex_duplicatearraysafe(&c3_lhs213), "lhs", "lhs",
                  213);
  sf_mex_addfield(*c3_info, c3_emlrt_marshallOut(
    "[E]C:/Users/sd0016/Dropbox/UAH Doktora/Journal Tracking/Matlab Code/3DOF_Experiments/Mirage/mirage.m"),
                  "context", "context", 214);
  sf_mex_addfield(*c3_info, c3_emlrt_marshallOut("mldivide"), "name", "name",
                  214);
  sf_mex_addfield(*c3_info, c3_emlrt_marshallOut("double"), "dominantType",
                  "dominantType", 214);
  sf_mex_addfield(*c3_info, c3_emlrt_marshallOut(
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/ops/mldivide.p"), "resolved",
                  "resolved", 214);
  sf_mex_addfield(*c3_info, c3_b_emlrt_marshallOut(1410832848U), "fileTimeLo",
                  "fileTimeLo", 214);
  sf_mex_addfield(*c3_info, c3_b_emlrt_marshallOut(0U), "fileTimeHi",
                  "fileTimeHi", 214);
  sf_mex_addfield(*c3_info, c3_b_emlrt_marshallOut(1319755166U), "mFileTimeLo",
                  "mFileTimeLo", 214);
  sf_mex_addfield(*c3_info, c3_b_emlrt_marshallOut(0U), "mFileTimeHi",
                  "mFileTimeHi", 214);
  sf_mex_assign(&c3_rhs214, sf_mex_createcellmatrix(0, 1), false);
  sf_mex_assign(&c3_lhs214, sf_mex_createcellmatrix(0, 1), false);
  sf_mex_addfield(*c3_info, sf_mex_duplicatearraysafe(&c3_rhs214), "rhs", "rhs",
                  214);
  sf_mex_addfield(*c3_info, sf_mex_duplicatearraysafe(&c3_lhs214), "lhs", "lhs",
                  214);
  sf_mex_addfield(*c3_info, c3_emlrt_marshallOut(
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/ops/mldivide.p"), "context",
                  "context", 215);
  sf_mex_addfield(*c3_info, c3_emlrt_marshallOut("eml_qrsolve"), "name", "name",
                  215);
  sf_mex_addfield(*c3_info, c3_emlrt_marshallOut("double"), "dominantType",
                  "dominantType", 215);
  sf_mex_addfield(*c3_info, c3_emlrt_marshallOut(
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/eml/eml_qrsolve.m"), "resolved",
                  "resolved", 215);
  sf_mex_addfield(*c3_info, c3_b_emlrt_marshallOut(1360307550U), "fileTimeLo",
                  "fileTimeLo", 215);
  sf_mex_addfield(*c3_info, c3_b_emlrt_marshallOut(0U), "fileTimeHi",
                  "fileTimeHi", 215);
  sf_mex_addfield(*c3_info, c3_b_emlrt_marshallOut(0U), "mFileTimeLo",
                  "mFileTimeLo", 215);
  sf_mex_addfield(*c3_info, c3_b_emlrt_marshallOut(0U), "mFileTimeHi",
                  "mFileTimeHi", 215);
  sf_mex_assign(&c3_rhs215, sf_mex_createcellmatrix(0, 1), false);
  sf_mex_assign(&c3_lhs215, sf_mex_createcellmatrix(0, 1), false);
  sf_mex_addfield(*c3_info, sf_mex_duplicatearraysafe(&c3_rhs215), "rhs", "rhs",
                  215);
  sf_mex_addfield(*c3_info, sf_mex_duplicatearraysafe(&c3_lhs215), "lhs", "lhs",
                  215);
  sf_mex_addfield(*c3_info, c3_emlrt_marshallOut(
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/eml/eml_qrsolve.m"), "context",
                  "context", 216);
  sf_mex_addfield(*c3_info, c3_emlrt_marshallOut("min"), "name", "name", 216);
  sf_mex_addfield(*c3_info, c3_emlrt_marshallOut("double"), "dominantType",
                  "dominantType", 216);
  sf_mex_addfield(*c3_info, c3_emlrt_marshallOut(
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/datafun/min.m"), "resolved",
                  "resolved", 216);
  sf_mex_addfield(*c3_info, c3_b_emlrt_marshallOut(1311280518U), "fileTimeLo",
                  "fileTimeLo", 216);
  sf_mex_addfield(*c3_info, c3_b_emlrt_marshallOut(0U), "fileTimeHi",
                  "fileTimeHi", 216);
  sf_mex_addfield(*c3_info, c3_b_emlrt_marshallOut(0U), "mFileTimeLo",
                  "mFileTimeLo", 216);
  sf_mex_addfield(*c3_info, c3_b_emlrt_marshallOut(0U), "mFileTimeHi",
                  "mFileTimeHi", 216);
  sf_mex_assign(&c3_rhs216, sf_mex_createcellmatrix(0, 1), false);
  sf_mex_assign(&c3_lhs216, sf_mex_createcellmatrix(0, 1), false);
  sf_mex_addfield(*c3_info, sf_mex_duplicatearraysafe(&c3_rhs216), "rhs", "rhs",
                  216);
  sf_mex_addfield(*c3_info, sf_mex_duplicatearraysafe(&c3_lhs216), "lhs", "lhs",
                  216);
  sf_mex_addfield(*c3_info, c3_emlrt_marshallOut(
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/eml/eml_qrsolve.m"), "context",
                  "context", 217);
  sf_mex_addfield(*c3_info, c3_emlrt_marshallOut("eml_xgeqp3"), "name", "name",
                  217);
  sf_mex_addfield(*c3_info, c3_emlrt_marshallOut("double"), "dominantType",
                  "dominantType", 217);
  sf_mex_addfield(*c3_info, c3_emlrt_marshallOut(
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/eml/lapack/eml_xgeqp3.m"),
                  "resolved", "resolved", 217);
  sf_mex_addfield(*c3_info, c3_b_emlrt_marshallOut(1286844004U), "fileTimeLo",
                  "fileTimeLo", 217);
  sf_mex_addfield(*c3_info, c3_b_emlrt_marshallOut(0U), "fileTimeHi",
                  "fileTimeHi", 217);
  sf_mex_addfield(*c3_info, c3_b_emlrt_marshallOut(0U), "mFileTimeLo",
                  "mFileTimeLo", 217);
  sf_mex_addfield(*c3_info, c3_b_emlrt_marshallOut(0U), "mFileTimeHi",
                  "mFileTimeHi", 217);
  sf_mex_assign(&c3_rhs217, sf_mex_createcellmatrix(0, 1), false);
  sf_mex_assign(&c3_lhs217, sf_mex_createcellmatrix(0, 1), false);
  sf_mex_addfield(*c3_info, sf_mex_duplicatearraysafe(&c3_rhs217), "rhs", "rhs",
                  217);
  sf_mex_addfield(*c3_info, sf_mex_duplicatearraysafe(&c3_lhs217), "lhs", "lhs",
                  217);
  sf_mex_addfield(*c3_info, c3_emlrt_marshallOut(
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/eml/lapack/eml_xgeqp3.m"),
                  "context", "context", 218);
  sf_mex_addfield(*c3_info, c3_emlrt_marshallOut("eml_lapack_xgeqp3"), "name",
                  "name", 218);
  sf_mex_addfield(*c3_info, c3_emlrt_marshallOut("double"), "dominantType",
                  "dominantType", 218);
  sf_mex_addfield(*c3_info, c3_emlrt_marshallOut(
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/eml/lapack/internal/eml_lapack_xgeqp3.m"),
                  "resolved", "resolved", 218);
  sf_mex_addfield(*c3_info, c3_b_emlrt_marshallOut(1286844008U), "fileTimeLo",
                  "fileTimeLo", 218);
  sf_mex_addfield(*c3_info, c3_b_emlrt_marshallOut(0U), "fileTimeHi",
                  "fileTimeHi", 218);
  sf_mex_addfield(*c3_info, c3_b_emlrt_marshallOut(0U), "mFileTimeLo",
                  "mFileTimeLo", 218);
  sf_mex_addfield(*c3_info, c3_b_emlrt_marshallOut(0U), "mFileTimeHi",
                  "mFileTimeHi", 218);
  sf_mex_assign(&c3_rhs218, sf_mex_createcellmatrix(0, 1), false);
  sf_mex_assign(&c3_lhs218, sf_mex_createcellmatrix(0, 1), false);
  sf_mex_addfield(*c3_info, sf_mex_duplicatearraysafe(&c3_rhs218), "rhs", "rhs",
                  218);
  sf_mex_addfield(*c3_info, sf_mex_duplicatearraysafe(&c3_lhs218), "lhs", "lhs",
                  218);
  sf_mex_addfield(*c3_info, c3_emlrt_marshallOut(
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/eml/lapack/internal/eml_lapack_xgeqp3.m"),
                  "context", "context", 219);
  sf_mex_addfield(*c3_info, c3_emlrt_marshallOut("eml_matlab_zgeqp3"), "name",
                  "name", 219);
  sf_mex_addfield(*c3_info, c3_emlrt_marshallOut("double"), "dominantType",
                  "dominantType", 219);
  sf_mex_addfield(*c3_info, c3_emlrt_marshallOut(
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/eml/lapack/matlab/eml_matlab_zgeqp3.m"),
                  "resolved", "resolved", 219);
  sf_mex_addfield(*c3_info, c3_b_emlrt_marshallOut(1290024566U), "fileTimeLo",
                  "fileTimeLo", 219);
  sf_mex_addfield(*c3_info, c3_b_emlrt_marshallOut(0U), "fileTimeHi",
                  "fileTimeHi", 219);
  sf_mex_addfield(*c3_info, c3_b_emlrt_marshallOut(0U), "mFileTimeLo",
                  "mFileTimeLo", 219);
  sf_mex_addfield(*c3_info, c3_b_emlrt_marshallOut(0U), "mFileTimeHi",
                  "mFileTimeHi", 219);
  sf_mex_assign(&c3_rhs219, sf_mex_createcellmatrix(0, 1), false);
  sf_mex_assign(&c3_lhs219, sf_mex_createcellmatrix(0, 1), false);
  sf_mex_addfield(*c3_info, sf_mex_duplicatearraysafe(&c3_rhs219), "rhs", "rhs",
                  219);
  sf_mex_addfield(*c3_info, sf_mex_duplicatearraysafe(&c3_lhs219), "lhs", "lhs",
                  219);
  sf_mex_addfield(*c3_info, c3_emlrt_marshallOut(
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/eml/lapack/matlab/eml_matlab_zgeqp3.m"),
                  "context", "context", 220);
  sf_mex_addfield(*c3_info, c3_emlrt_marshallOut("eml_index_class"), "name",
                  "name", 220);
  sf_mex_addfield(*c3_info, c3_emlrt_marshallOut(""), "dominantType",
                  "dominantType", 220);
  sf_mex_addfield(*c3_info, c3_emlrt_marshallOut(
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/eml/eml_index_class.m"),
                  "resolved", "resolved", 220);
  sf_mex_addfield(*c3_info, c3_b_emlrt_marshallOut(1323195778U), "fileTimeLo",
                  "fileTimeLo", 220);
  sf_mex_addfield(*c3_info, c3_b_emlrt_marshallOut(0U), "fileTimeHi",
                  "fileTimeHi", 220);
  sf_mex_addfield(*c3_info, c3_b_emlrt_marshallOut(0U), "mFileTimeLo",
                  "mFileTimeLo", 220);
  sf_mex_addfield(*c3_info, c3_b_emlrt_marshallOut(0U), "mFileTimeHi",
                  "mFileTimeHi", 220);
  sf_mex_assign(&c3_rhs220, sf_mex_createcellmatrix(0, 1), false);
  sf_mex_assign(&c3_lhs220, sf_mex_createcellmatrix(0, 1), false);
  sf_mex_addfield(*c3_info, sf_mex_duplicatearraysafe(&c3_rhs220), "rhs", "rhs",
                  220);
  sf_mex_addfield(*c3_info, sf_mex_duplicatearraysafe(&c3_lhs220), "lhs", "lhs",
                  220);
  sf_mex_addfield(*c3_info, c3_emlrt_marshallOut(
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/eml/lapack/matlab/eml_matlab_zgeqp3.m"),
                  "context", "context", 221);
  sf_mex_addfield(*c3_info, c3_emlrt_marshallOut("min"), "name", "name", 221);
  sf_mex_addfield(*c3_info, c3_emlrt_marshallOut("coder.internal.indexInt"),
                  "dominantType", "dominantType", 221);
  sf_mex_addfield(*c3_info, c3_emlrt_marshallOut(
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/datafun/min.m"), "resolved",
                  "resolved", 221);
  sf_mex_addfield(*c3_info, c3_b_emlrt_marshallOut(1311280518U), "fileTimeLo",
                  "fileTimeLo", 221);
  sf_mex_addfield(*c3_info, c3_b_emlrt_marshallOut(0U), "fileTimeHi",
                  "fileTimeHi", 221);
  sf_mex_addfield(*c3_info, c3_b_emlrt_marshallOut(0U), "mFileTimeLo",
                  "mFileTimeLo", 221);
  sf_mex_addfield(*c3_info, c3_b_emlrt_marshallOut(0U), "mFileTimeHi",
                  "mFileTimeHi", 221);
  sf_mex_assign(&c3_rhs221, sf_mex_createcellmatrix(0, 1), false);
  sf_mex_assign(&c3_lhs221, sf_mex_createcellmatrix(0, 1), false);
  sf_mex_addfield(*c3_info, sf_mex_duplicatearraysafe(&c3_rhs221), "rhs", "rhs",
                  221);
  sf_mex_addfield(*c3_info, sf_mex_duplicatearraysafe(&c3_lhs221), "lhs", "lhs",
                  221);
  sf_mex_addfield(*c3_info, c3_emlrt_marshallOut(
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/eml/lapack/matlab/eml_matlab_zgeqp3.m"),
                  "context", "context", 222);
  sf_mex_addfield(*c3_info, c3_emlrt_marshallOut("eml_scalar_eg"), "name",
                  "name", 222);
  sf_mex_addfield(*c3_info, c3_emlrt_marshallOut("double"), "dominantType",
                  "dominantType", 222);
  sf_mex_addfield(*c3_info, c3_emlrt_marshallOut(
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/eml/eml_scalar_eg.m"), "resolved",
                  "resolved", 222);
  sf_mex_addfield(*c3_info, c3_b_emlrt_marshallOut(1376005888U), "fileTimeLo",
                  "fileTimeLo", 222);
  sf_mex_addfield(*c3_info, c3_b_emlrt_marshallOut(0U), "fileTimeHi",
                  "fileTimeHi", 222);
  sf_mex_addfield(*c3_info, c3_b_emlrt_marshallOut(0U), "mFileTimeLo",
                  "mFileTimeLo", 222);
  sf_mex_addfield(*c3_info, c3_b_emlrt_marshallOut(0U), "mFileTimeHi",
                  "mFileTimeHi", 222);
  sf_mex_assign(&c3_rhs222, sf_mex_createcellmatrix(0, 1), false);
  sf_mex_assign(&c3_lhs222, sf_mex_createcellmatrix(0, 1), false);
  sf_mex_addfield(*c3_info, sf_mex_duplicatearraysafe(&c3_rhs222), "rhs", "rhs",
                  222);
  sf_mex_addfield(*c3_info, sf_mex_duplicatearraysafe(&c3_lhs222), "lhs", "lhs",
                  222);
  sf_mex_addfield(*c3_info, c3_emlrt_marshallOut(
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/eml/lapack/matlab/eml_matlab_zgeqp3.m"),
                  "context", "context", 223);
  sf_mex_addfield(*c3_info, c3_emlrt_marshallOut("colon"), "name", "name", 223);
  sf_mex_addfield(*c3_info, c3_emlrt_marshallOut("coder.internal.indexInt"),
                  "dominantType", "dominantType", 223);
  sf_mex_addfield(*c3_info, c3_emlrt_marshallOut(
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/ops/colon.m"), "resolved",
                  "resolved", 223);
  sf_mex_addfield(*c3_info, c3_b_emlrt_marshallOut(1378321188U), "fileTimeLo",
                  "fileTimeLo", 223);
  sf_mex_addfield(*c3_info, c3_b_emlrt_marshallOut(0U), "fileTimeHi",
                  "fileTimeHi", 223);
  sf_mex_addfield(*c3_info, c3_b_emlrt_marshallOut(0U), "mFileTimeLo",
                  "mFileTimeLo", 223);
  sf_mex_addfield(*c3_info, c3_b_emlrt_marshallOut(0U), "mFileTimeHi",
                  "mFileTimeHi", 223);
  sf_mex_assign(&c3_rhs223, sf_mex_createcellmatrix(0, 1), false);
  sf_mex_assign(&c3_lhs223, sf_mex_createcellmatrix(0, 1), false);
  sf_mex_addfield(*c3_info, sf_mex_duplicatearraysafe(&c3_rhs223), "rhs", "rhs",
                  223);
  sf_mex_addfield(*c3_info, sf_mex_duplicatearraysafe(&c3_lhs223), "lhs", "lhs",
                  223);
  sf_mex_addfield(*c3_info, c3_emlrt_marshallOut(
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/eml/lapack/matlab/eml_matlab_zgeqp3.m"),
                  "context", "context", 224);
  sf_mex_addfield(*c3_info, c3_emlrt_marshallOut("eps"), "name", "name", 224);
  sf_mex_addfield(*c3_info, c3_emlrt_marshallOut("char"), "dominantType",
                  "dominantType", 224);
  sf_mex_addfield(*c3_info, c3_emlrt_marshallOut(
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/elmat/eps.m"), "resolved",
                  "resolved", 224);
  sf_mex_addfield(*c3_info, c3_b_emlrt_marshallOut(1326753196U), "fileTimeLo",
                  "fileTimeLo", 224);
  sf_mex_addfield(*c3_info, c3_b_emlrt_marshallOut(0U), "fileTimeHi",
                  "fileTimeHi", 224);
  sf_mex_addfield(*c3_info, c3_b_emlrt_marshallOut(0U), "mFileTimeLo",
                  "mFileTimeLo", 224);
  sf_mex_addfield(*c3_info, c3_b_emlrt_marshallOut(0U), "mFileTimeHi",
                  "mFileTimeHi", 224);
  sf_mex_assign(&c3_rhs224, sf_mex_createcellmatrix(0, 1), false);
  sf_mex_assign(&c3_lhs224, sf_mex_createcellmatrix(0, 1), false);
  sf_mex_addfield(*c3_info, sf_mex_duplicatearraysafe(&c3_rhs224), "rhs", "rhs",
                  224);
  sf_mex_addfield(*c3_info, sf_mex_duplicatearraysafe(&c3_lhs224), "lhs", "lhs",
                  224);
  sf_mex_addfield(*c3_info, c3_emlrt_marshallOut(
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/eml/lapack/matlab/eml_matlab_zgeqp3.m"),
                  "context", "context", 225);
  sf_mex_addfield(*c3_info, c3_emlrt_marshallOut("sqrt"), "name", "name", 225);
  sf_mex_addfield(*c3_info, c3_emlrt_marshallOut("double"), "dominantType",
                  "dominantType", 225);
  sf_mex_addfield(*c3_info, c3_emlrt_marshallOut(
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/elfun/sqrt.m"), "resolved",
                  "resolved", 225);
  sf_mex_addfield(*c3_info, c3_b_emlrt_marshallOut(1343855586U), "fileTimeLo",
                  "fileTimeLo", 225);
  sf_mex_addfield(*c3_info, c3_b_emlrt_marshallOut(0U), "fileTimeHi",
                  "fileTimeHi", 225);
  sf_mex_addfield(*c3_info, c3_b_emlrt_marshallOut(0U), "mFileTimeLo",
                  "mFileTimeLo", 225);
  sf_mex_addfield(*c3_info, c3_b_emlrt_marshallOut(0U), "mFileTimeHi",
                  "mFileTimeHi", 225);
  sf_mex_assign(&c3_rhs225, sf_mex_createcellmatrix(0, 1), false);
  sf_mex_assign(&c3_lhs225, sf_mex_createcellmatrix(0, 1), false);
  sf_mex_addfield(*c3_info, sf_mex_duplicatearraysafe(&c3_rhs225), "rhs", "rhs",
                  225);
  sf_mex_addfield(*c3_info, sf_mex_duplicatearraysafe(&c3_lhs225), "lhs", "lhs",
                  225);
  sf_mex_addfield(*c3_info, c3_emlrt_marshallOut(
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/elfun/sqrt.m"), "context",
                  "context", 226);
  sf_mex_addfield(*c3_info, c3_emlrt_marshallOut("eml_error"), "name", "name",
                  226);
  sf_mex_addfield(*c3_info, c3_emlrt_marshallOut("char"), "dominantType",
                  "dominantType", 226);
  sf_mex_addfield(*c3_info, c3_emlrt_marshallOut(
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/eml/eml_error.m"), "resolved",
                  "resolved", 226);
  sf_mex_addfield(*c3_info, c3_b_emlrt_marshallOut(1343855558U), "fileTimeLo",
                  "fileTimeLo", 226);
  sf_mex_addfield(*c3_info, c3_b_emlrt_marshallOut(0U), "fileTimeHi",
                  "fileTimeHi", 226);
  sf_mex_addfield(*c3_info, c3_b_emlrt_marshallOut(0U), "mFileTimeLo",
                  "mFileTimeLo", 226);
  sf_mex_addfield(*c3_info, c3_b_emlrt_marshallOut(0U), "mFileTimeHi",
                  "mFileTimeHi", 226);
  sf_mex_assign(&c3_rhs226, sf_mex_createcellmatrix(0, 1), false);
  sf_mex_assign(&c3_lhs226, sf_mex_createcellmatrix(0, 1), false);
  sf_mex_addfield(*c3_info, sf_mex_duplicatearraysafe(&c3_rhs226), "rhs", "rhs",
                  226);
  sf_mex_addfield(*c3_info, sf_mex_duplicatearraysafe(&c3_lhs226), "lhs", "lhs",
                  226);
  sf_mex_addfield(*c3_info, c3_emlrt_marshallOut(
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/elfun/sqrt.m"), "context",
                  "context", 227);
  sf_mex_addfield(*c3_info, c3_emlrt_marshallOut("eml_scalar_sqrt"), "name",
                  "name", 227);
  sf_mex_addfield(*c3_info, c3_emlrt_marshallOut("double"), "dominantType",
                  "dominantType", 227);
  sf_mex_addfield(*c3_info, c3_emlrt_marshallOut(
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/elfun/eml_scalar_sqrt.m"),
                  "resolved", "resolved", 227);
  sf_mex_addfield(*c3_info, c3_b_emlrt_marshallOut(1286843938U), "fileTimeLo",
                  "fileTimeLo", 227);
  sf_mex_addfield(*c3_info, c3_b_emlrt_marshallOut(0U), "fileTimeHi",
                  "fileTimeHi", 227);
  sf_mex_addfield(*c3_info, c3_b_emlrt_marshallOut(0U), "mFileTimeLo",
                  "mFileTimeLo", 227);
  sf_mex_addfield(*c3_info, c3_b_emlrt_marshallOut(0U), "mFileTimeHi",
                  "mFileTimeHi", 227);
  sf_mex_assign(&c3_rhs227, sf_mex_createcellmatrix(0, 1), false);
  sf_mex_assign(&c3_lhs227, sf_mex_createcellmatrix(0, 1), false);
  sf_mex_addfield(*c3_info, sf_mex_duplicatearraysafe(&c3_rhs227), "rhs", "rhs",
                  227);
  sf_mex_addfield(*c3_info, sf_mex_duplicatearraysafe(&c3_lhs227), "lhs", "lhs",
                  227);
  sf_mex_addfield(*c3_info, c3_emlrt_marshallOut(
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/eml/lapack/matlab/eml_matlab_zgeqp3.m"),
                  "context", "context", 228);
  sf_mex_addfield(*c3_info, c3_emlrt_marshallOut(
    "eml_int_forloop_overflow_check"), "name", "name", 228);
  sf_mex_addfield(*c3_info, c3_emlrt_marshallOut(""), "dominantType",
                  "dominantType", 228);
  sf_mex_addfield(*c3_info, c3_emlrt_marshallOut(
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/eml/eml_int_forloop_overflow_check.m"),
                  "resolved", "resolved", 228);
  sf_mex_addfield(*c3_info, c3_b_emlrt_marshallOut(1397282622U), "fileTimeLo",
                  "fileTimeLo", 228);
  sf_mex_addfield(*c3_info, c3_b_emlrt_marshallOut(0U), "fileTimeHi",
                  "fileTimeHi", 228);
  sf_mex_addfield(*c3_info, c3_b_emlrt_marshallOut(0U), "mFileTimeLo",
                  "mFileTimeLo", 228);
  sf_mex_addfield(*c3_info, c3_b_emlrt_marshallOut(0U), "mFileTimeHi",
                  "mFileTimeHi", 228);
  sf_mex_assign(&c3_rhs228, sf_mex_createcellmatrix(0, 1), false);
  sf_mex_assign(&c3_lhs228, sf_mex_createcellmatrix(0, 1), false);
  sf_mex_addfield(*c3_info, sf_mex_duplicatearraysafe(&c3_rhs228), "rhs", "rhs",
                  228);
  sf_mex_addfield(*c3_info, sf_mex_duplicatearraysafe(&c3_lhs228), "lhs", "lhs",
                  228);
  sf_mex_addfield(*c3_info, c3_emlrt_marshallOut(
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/eml/lapack/matlab/eml_matlab_zgeqp3.m"),
                  "context", "context", 229);
  sf_mex_addfield(*c3_info, c3_emlrt_marshallOut("eml_xnrm2"), "name", "name",
                  229);
  sf_mex_addfield(*c3_info, c3_emlrt_marshallOut("double"), "dominantType",
                  "dominantType", 229);
  sf_mex_addfield(*c3_info, c3_emlrt_marshallOut(
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/eml/blas/eml_xnrm2.m"),
                  "resolved", "resolved", 229);
  sf_mex_addfield(*c3_info, c3_b_emlrt_marshallOut(1376005892U), "fileTimeLo",
                  "fileTimeLo", 229);
  sf_mex_addfield(*c3_info, c3_b_emlrt_marshallOut(0U), "fileTimeHi",
                  "fileTimeHi", 229);
  sf_mex_addfield(*c3_info, c3_b_emlrt_marshallOut(0U), "mFileTimeLo",
                  "mFileTimeLo", 229);
  sf_mex_addfield(*c3_info, c3_b_emlrt_marshallOut(0U), "mFileTimeHi",
                  "mFileTimeHi", 229);
  sf_mex_assign(&c3_rhs229, sf_mex_createcellmatrix(0, 1), false);
  sf_mex_assign(&c3_lhs229, sf_mex_createcellmatrix(0, 1), false);
  sf_mex_addfield(*c3_info, sf_mex_duplicatearraysafe(&c3_rhs229), "rhs", "rhs",
                  229);
  sf_mex_addfield(*c3_info, sf_mex_duplicatearraysafe(&c3_lhs229), "lhs", "lhs",
                  229);
  sf_mex_addfield(*c3_info, c3_emlrt_marshallOut(
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/eml/blas/eml_xnrm2.m"), "context",
                  "context", 230);
  sf_mex_addfield(*c3_info, c3_emlrt_marshallOut("coder.internal.blas.inline"),
                  "name", "name", 230);
  sf_mex_addfield(*c3_info, c3_emlrt_marshallOut(""), "dominantType",
                  "dominantType", 230);
  sf_mex_addfield(*c3_info, c3_emlrt_marshallOut(
    "[ILXE]$matlabroot$/toolbox/coder/coder/+coder/+internal/+blas/inline.p"),
                  "resolved", "resolved", 230);
  sf_mex_addfield(*c3_info, c3_b_emlrt_marshallOut(1410832972U), "fileTimeLo",
                  "fileTimeLo", 230);
  sf_mex_addfield(*c3_info, c3_b_emlrt_marshallOut(0U), "fileTimeHi",
                  "fileTimeHi", 230);
  sf_mex_addfield(*c3_info, c3_b_emlrt_marshallOut(0U), "mFileTimeLo",
                  "mFileTimeLo", 230);
  sf_mex_addfield(*c3_info, c3_b_emlrt_marshallOut(0U), "mFileTimeHi",
                  "mFileTimeHi", 230);
  sf_mex_assign(&c3_rhs230, sf_mex_createcellmatrix(0, 1), false);
  sf_mex_assign(&c3_lhs230, sf_mex_createcellmatrix(0, 1), false);
  sf_mex_addfield(*c3_info, sf_mex_duplicatearraysafe(&c3_rhs230), "rhs", "rhs",
                  230);
  sf_mex_addfield(*c3_info, sf_mex_duplicatearraysafe(&c3_lhs230), "lhs", "lhs",
                  230);
  sf_mex_addfield(*c3_info, c3_emlrt_marshallOut(
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/eml/blas/eml_xnrm2.m"), "context",
                  "context", 231);
  sf_mex_addfield(*c3_info, c3_emlrt_marshallOut("coder.internal.blas.xnrm2"),
                  "name", "name", 231);
  sf_mex_addfield(*c3_info, c3_emlrt_marshallOut("double"), "dominantType",
                  "dominantType", 231);
  sf_mex_addfield(*c3_info, c3_emlrt_marshallOut(
    "[ILXE]$matlabroot$/toolbox/coder/coder/+coder/+internal/+blas/xnrm2.p"),
                  "resolved", "resolved", 231);
  sf_mex_addfield(*c3_info, c3_b_emlrt_marshallOut(1410832970U), "fileTimeLo",
                  "fileTimeLo", 231);
  sf_mex_addfield(*c3_info, c3_b_emlrt_marshallOut(0U), "fileTimeHi",
                  "fileTimeHi", 231);
  sf_mex_addfield(*c3_info, c3_b_emlrt_marshallOut(0U), "mFileTimeLo",
                  "mFileTimeLo", 231);
  sf_mex_addfield(*c3_info, c3_b_emlrt_marshallOut(0U), "mFileTimeHi",
                  "mFileTimeHi", 231);
  sf_mex_assign(&c3_rhs231, sf_mex_createcellmatrix(0, 1), false);
  sf_mex_assign(&c3_lhs231, sf_mex_createcellmatrix(0, 1), false);
  sf_mex_addfield(*c3_info, sf_mex_duplicatearraysafe(&c3_rhs231), "rhs", "rhs",
                  231);
  sf_mex_addfield(*c3_info, sf_mex_duplicatearraysafe(&c3_lhs231), "lhs", "lhs",
                  231);
  sf_mex_addfield(*c3_info, c3_emlrt_marshallOut(
    "[ILXE]$matlabroot$/toolbox/coder/coder/+coder/+internal/+blas/xnrm2.p"),
                  "context", "context", 232);
  sf_mex_addfield(*c3_info, c3_emlrt_marshallOut(
    "coder.internal.blas.use_refblas"), "name", "name", 232);
  sf_mex_addfield(*c3_info, c3_emlrt_marshallOut(""), "dominantType",
                  "dominantType", 232);
  sf_mex_addfield(*c3_info, c3_emlrt_marshallOut(
    "[ILXE]$matlabroot$/toolbox/coder/coder/+coder/+internal/+blas/use_refblas.p"),
                  "resolved", "resolved", 232);
  sf_mex_addfield(*c3_info, c3_b_emlrt_marshallOut(1410832970U), "fileTimeLo",
                  "fileTimeLo", 232);
  sf_mex_addfield(*c3_info, c3_b_emlrt_marshallOut(0U), "fileTimeHi",
                  "fileTimeHi", 232);
  sf_mex_addfield(*c3_info, c3_b_emlrt_marshallOut(0U), "mFileTimeLo",
                  "mFileTimeLo", 232);
  sf_mex_addfield(*c3_info, c3_b_emlrt_marshallOut(0U), "mFileTimeHi",
                  "mFileTimeHi", 232);
  sf_mex_assign(&c3_rhs232, sf_mex_createcellmatrix(0, 1), false);
  sf_mex_assign(&c3_lhs232, sf_mex_createcellmatrix(0, 1), false);
  sf_mex_addfield(*c3_info, sf_mex_duplicatearraysafe(&c3_rhs232), "rhs", "rhs",
                  232);
  sf_mex_addfield(*c3_info, sf_mex_duplicatearraysafe(&c3_lhs232), "lhs", "lhs",
                  232);
  sf_mex_addfield(*c3_info, c3_emlrt_marshallOut(
    "[ILXE]$matlabroot$/toolbox/coder/coder/+coder/+internal/+blas/xnrm2.p!below_threshold"),
                  "context", "context", 233);
  sf_mex_addfield(*c3_info, c3_emlrt_marshallOut("coder.internal.blas.threshold"),
                  "name", "name", 233);
  sf_mex_addfield(*c3_info, c3_emlrt_marshallOut("char"), "dominantType",
                  "dominantType", 233);
  sf_mex_addfield(*c3_info, c3_emlrt_marshallOut(
    "[ILXE]$matlabroot$/toolbox/coder/coder/+coder/+internal/+blas/threshold.p"),
                  "resolved", "resolved", 233);
  sf_mex_addfield(*c3_info, c3_b_emlrt_marshallOut(1410832972U), "fileTimeLo",
                  "fileTimeLo", 233);
  sf_mex_addfield(*c3_info, c3_b_emlrt_marshallOut(0U), "fileTimeHi",
                  "fileTimeHi", 233);
  sf_mex_addfield(*c3_info, c3_b_emlrt_marshallOut(0U), "mFileTimeLo",
                  "mFileTimeLo", 233);
  sf_mex_addfield(*c3_info, c3_b_emlrt_marshallOut(0U), "mFileTimeHi",
                  "mFileTimeHi", 233);
  sf_mex_assign(&c3_rhs233, sf_mex_createcellmatrix(0, 1), false);
  sf_mex_assign(&c3_lhs233, sf_mex_createcellmatrix(0, 1), false);
  sf_mex_addfield(*c3_info, sf_mex_duplicatearraysafe(&c3_rhs233), "rhs", "rhs",
                  233);
  sf_mex_addfield(*c3_info, sf_mex_duplicatearraysafe(&c3_lhs233), "lhs", "lhs",
                  233);
  sf_mex_addfield(*c3_info, c3_emlrt_marshallOut(
    "[ILXE]$matlabroot$/toolbox/coder/coder/+coder/+internal/+blas/xnrm2.p"),
                  "context", "context", 234);
  sf_mex_addfield(*c3_info, c3_emlrt_marshallOut("coder.internal.refblas.xnrm2"),
                  "name", "name", 234);
  sf_mex_addfield(*c3_info, c3_emlrt_marshallOut("double"), "dominantType",
                  "dominantType", 234);
  sf_mex_addfield(*c3_info, c3_emlrt_marshallOut(
    "[ILXE]$matlabroot$/toolbox/coder/coder/+coder/+internal/+refblas/xnrm2.p"),
                  "resolved", "resolved", 234);
  sf_mex_addfield(*c3_info, c3_b_emlrt_marshallOut(1410832972U), "fileTimeLo",
                  "fileTimeLo", 234);
  sf_mex_addfield(*c3_info, c3_b_emlrt_marshallOut(0U), "fileTimeHi",
                  "fileTimeHi", 234);
  sf_mex_addfield(*c3_info, c3_b_emlrt_marshallOut(0U), "mFileTimeLo",
                  "mFileTimeLo", 234);
  sf_mex_addfield(*c3_info, c3_b_emlrt_marshallOut(0U), "mFileTimeHi",
                  "mFileTimeHi", 234);
  sf_mex_assign(&c3_rhs234, sf_mex_createcellmatrix(0, 1), false);
  sf_mex_assign(&c3_lhs234, sf_mex_createcellmatrix(0, 1), false);
  sf_mex_addfield(*c3_info, sf_mex_duplicatearraysafe(&c3_rhs234), "rhs", "rhs",
                  234);
  sf_mex_addfield(*c3_info, sf_mex_duplicatearraysafe(&c3_lhs234), "lhs", "lhs",
                  234);
  sf_mex_addfield(*c3_info, c3_emlrt_marshallOut(
    "[ILXE]$matlabroot$/toolbox/coder/coder/+coder/+internal/+refblas/xnrm2.p"),
                  "context", "context", 235);
  sf_mex_addfield(*c3_info, c3_emlrt_marshallOut("realmin"), "name", "name", 235);
  sf_mex_addfield(*c3_info, c3_emlrt_marshallOut("char"), "dominantType",
                  "dominantType", 235);
  sf_mex_addfield(*c3_info, c3_emlrt_marshallOut(
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/elmat/realmin.m"), "resolved",
                  "resolved", 235);
  sf_mex_addfield(*c3_info, c3_b_emlrt_marshallOut(1307676442U), "fileTimeLo",
                  "fileTimeLo", 235);
  sf_mex_addfield(*c3_info, c3_b_emlrt_marshallOut(0U), "fileTimeHi",
                  "fileTimeHi", 235);
  sf_mex_addfield(*c3_info, c3_b_emlrt_marshallOut(0U), "mFileTimeLo",
                  "mFileTimeLo", 235);
  sf_mex_addfield(*c3_info, c3_b_emlrt_marshallOut(0U), "mFileTimeHi",
                  "mFileTimeHi", 235);
  sf_mex_assign(&c3_rhs235, sf_mex_createcellmatrix(0, 1), false);
  sf_mex_assign(&c3_lhs235, sf_mex_createcellmatrix(0, 1), false);
  sf_mex_addfield(*c3_info, sf_mex_duplicatearraysafe(&c3_rhs235), "rhs", "rhs",
                  235);
  sf_mex_addfield(*c3_info, sf_mex_duplicatearraysafe(&c3_lhs235), "lhs", "lhs",
                  235);
  sf_mex_addfield(*c3_info, c3_emlrt_marshallOut(
    "[ILXE]$matlabroot$/toolbox/coder/coder/+coder/+internal/+refblas/xnrm2.p"),
                  "context", "context", 236);
  sf_mex_addfield(*c3_info, c3_emlrt_marshallOut("coder.internal.indexMinus"),
                  "name", "name", 236);
  sf_mex_addfield(*c3_info, c3_emlrt_marshallOut("double"), "dominantType",
                  "dominantType", 236);
  sf_mex_addfield(*c3_info, c3_emlrt_marshallOut(
    "[ILXE]$matlabroot$/toolbox/shared/coder/coder/+coder/+internal/indexMinus.m"),
                  "resolved", "resolved", 236);
  sf_mex_addfield(*c3_info, c3_b_emlrt_marshallOut(1372608360U), "fileTimeLo",
                  "fileTimeLo", 236);
  sf_mex_addfield(*c3_info, c3_b_emlrt_marshallOut(0U), "fileTimeHi",
                  "fileTimeHi", 236);
  sf_mex_addfield(*c3_info, c3_b_emlrt_marshallOut(0U), "mFileTimeLo",
                  "mFileTimeLo", 236);
  sf_mex_addfield(*c3_info, c3_b_emlrt_marshallOut(0U), "mFileTimeHi",
                  "mFileTimeHi", 236);
  sf_mex_assign(&c3_rhs236, sf_mex_createcellmatrix(0, 1), false);
  sf_mex_assign(&c3_lhs236, sf_mex_createcellmatrix(0, 1), false);
  sf_mex_addfield(*c3_info, sf_mex_duplicatearraysafe(&c3_rhs236), "rhs", "rhs",
                  236);
  sf_mex_addfield(*c3_info, sf_mex_duplicatearraysafe(&c3_lhs236), "lhs", "lhs",
                  236);
  sf_mex_addfield(*c3_info, c3_emlrt_marshallOut(
    "[ILXE]$matlabroot$/toolbox/coder/coder/+coder/+internal/+refblas/xnrm2.p"),
                  "context", "context", 237);
  sf_mex_addfield(*c3_info, c3_emlrt_marshallOut("coder.internal.indexTimes"),
                  "name", "name", 237);
  sf_mex_addfield(*c3_info, c3_emlrt_marshallOut("coder.internal.indexInt"),
                  "dominantType", "dominantType", 237);
  sf_mex_addfield(*c3_info, c3_emlrt_marshallOut(
    "[ILXE]$matlabroot$/toolbox/shared/coder/coder/+coder/+internal/indexTimes.m"),
                  "resolved", "resolved", 237);
  sf_mex_addfield(*c3_info, c3_b_emlrt_marshallOut(1372608360U), "fileTimeLo",
                  "fileTimeLo", 237);
  sf_mex_addfield(*c3_info, c3_b_emlrt_marshallOut(0U), "fileTimeHi",
                  "fileTimeHi", 237);
  sf_mex_addfield(*c3_info, c3_b_emlrt_marshallOut(0U), "mFileTimeLo",
                  "mFileTimeLo", 237);
  sf_mex_addfield(*c3_info, c3_b_emlrt_marshallOut(0U), "mFileTimeHi",
                  "mFileTimeHi", 237);
  sf_mex_assign(&c3_rhs237, sf_mex_createcellmatrix(0, 1), false);
  sf_mex_assign(&c3_lhs237, sf_mex_createcellmatrix(0, 1), false);
  sf_mex_addfield(*c3_info, sf_mex_duplicatearraysafe(&c3_rhs237), "rhs", "rhs",
                  237);
  sf_mex_addfield(*c3_info, sf_mex_duplicatearraysafe(&c3_lhs237), "lhs", "lhs",
                  237);
  sf_mex_addfield(*c3_info, c3_emlrt_marshallOut(
    "[ILXE]$matlabroot$/toolbox/coder/coder/+coder/+internal/+refblas/xnrm2.p"),
                  "context", "context", 238);
  sf_mex_addfield(*c3_info, c3_emlrt_marshallOut("coder.internal.indexPlus"),
                  "name", "name", 238);
  sf_mex_addfield(*c3_info, c3_emlrt_marshallOut("coder.internal.indexInt"),
                  "dominantType", "dominantType", 238);
  sf_mex_addfield(*c3_info, c3_emlrt_marshallOut(
    "[ILXE]$matlabroot$/toolbox/shared/coder/coder/+coder/+internal/indexPlus.m"),
                  "resolved", "resolved", 238);
  sf_mex_addfield(*c3_info, c3_b_emlrt_marshallOut(1372608360U), "fileTimeLo",
                  "fileTimeLo", 238);
  sf_mex_addfield(*c3_info, c3_b_emlrt_marshallOut(0U), "fileTimeHi",
                  "fileTimeHi", 238);
  sf_mex_addfield(*c3_info, c3_b_emlrt_marshallOut(0U), "mFileTimeLo",
                  "mFileTimeLo", 238);
  sf_mex_addfield(*c3_info, c3_b_emlrt_marshallOut(0U), "mFileTimeHi",
                  "mFileTimeHi", 238);
  sf_mex_assign(&c3_rhs238, sf_mex_createcellmatrix(0, 1), false);
  sf_mex_assign(&c3_lhs238, sf_mex_createcellmatrix(0, 1), false);
  sf_mex_addfield(*c3_info, sf_mex_duplicatearraysafe(&c3_rhs238), "rhs", "rhs",
                  238);
  sf_mex_addfield(*c3_info, sf_mex_duplicatearraysafe(&c3_lhs238), "lhs", "lhs",
                  238);
  sf_mex_addfield(*c3_info, c3_emlrt_marshallOut(
    "[ILXE]$matlabroot$/toolbox/coder/coder/+coder/+internal/+refblas/xnrm2.p"),
                  "context", "context", 239);
  sf_mex_addfield(*c3_info, c3_emlrt_marshallOut(
    "eml_int_forloop_overflow_check"), "name", "name", 239);
  sf_mex_addfield(*c3_info, c3_emlrt_marshallOut(""), "dominantType",
                  "dominantType", 239);
  sf_mex_addfield(*c3_info, c3_emlrt_marshallOut(
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/eml/eml_int_forloop_overflow_check.m"),
                  "resolved", "resolved", 239);
  sf_mex_addfield(*c3_info, c3_b_emlrt_marshallOut(1397282622U), "fileTimeLo",
                  "fileTimeLo", 239);
  sf_mex_addfield(*c3_info, c3_b_emlrt_marshallOut(0U), "fileTimeHi",
                  "fileTimeHi", 239);
  sf_mex_addfield(*c3_info, c3_b_emlrt_marshallOut(0U), "mFileTimeLo",
                  "mFileTimeLo", 239);
  sf_mex_addfield(*c3_info, c3_b_emlrt_marshallOut(0U), "mFileTimeHi",
                  "mFileTimeHi", 239);
  sf_mex_assign(&c3_rhs239, sf_mex_createcellmatrix(0, 1), false);
  sf_mex_assign(&c3_lhs239, sf_mex_createcellmatrix(0, 1), false);
  sf_mex_addfield(*c3_info, sf_mex_duplicatearraysafe(&c3_rhs239), "rhs", "rhs",
                  239);
  sf_mex_addfield(*c3_info, sf_mex_duplicatearraysafe(&c3_lhs239), "lhs", "lhs",
                  239);
  sf_mex_addfield(*c3_info, c3_emlrt_marshallOut(
    "[ILXE]$matlabroot$/toolbox/coder/coder/+coder/+internal/+refblas/xnrm2.p"),
                  "context", "context", 240);
  sf_mex_addfield(*c3_info, c3_emlrt_marshallOut("abs"), "name", "name", 240);
  sf_mex_addfield(*c3_info, c3_emlrt_marshallOut("double"), "dominantType",
                  "dominantType", 240);
  sf_mex_addfield(*c3_info, c3_emlrt_marshallOut(
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/elfun/abs.m"), "resolved",
                  "resolved", 240);
  sf_mex_addfield(*c3_info, c3_b_emlrt_marshallOut(1363735452U), "fileTimeLo",
                  "fileTimeLo", 240);
  sf_mex_addfield(*c3_info, c3_b_emlrt_marshallOut(0U), "fileTimeHi",
                  "fileTimeHi", 240);
  sf_mex_addfield(*c3_info, c3_b_emlrt_marshallOut(0U), "mFileTimeLo",
                  "mFileTimeLo", 240);
  sf_mex_addfield(*c3_info, c3_b_emlrt_marshallOut(0U), "mFileTimeHi",
                  "mFileTimeHi", 240);
  sf_mex_assign(&c3_rhs240, sf_mex_createcellmatrix(0, 1), false);
  sf_mex_assign(&c3_lhs240, sf_mex_createcellmatrix(0, 1), false);
  sf_mex_addfield(*c3_info, sf_mex_duplicatearraysafe(&c3_rhs240), "rhs", "rhs",
                  240);
  sf_mex_addfield(*c3_info, sf_mex_duplicatearraysafe(&c3_lhs240), "lhs", "lhs",
                  240);
  sf_mex_addfield(*c3_info, c3_emlrt_marshallOut(
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/eml/lapack/matlab/eml_matlab_zgeqp3.m"),
                  "context", "context", 241);
  sf_mex_addfield(*c3_info, c3_emlrt_marshallOut("eml_index_plus"), "name",
                  "name", 241);
  sf_mex_addfield(*c3_info, c3_emlrt_marshallOut("coder.internal.indexInt"),
                  "dominantType", "dominantType", 241);
  sf_mex_addfield(*c3_info, c3_emlrt_marshallOut(
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/eml/eml_index_plus.m"),
                  "resolved", "resolved", 241);
  sf_mex_addfield(*c3_info, c3_b_emlrt_marshallOut(1372607616U), "fileTimeLo",
                  "fileTimeLo", 241);
  sf_mex_addfield(*c3_info, c3_b_emlrt_marshallOut(0U), "fileTimeHi",
                  "fileTimeHi", 241);
  sf_mex_addfield(*c3_info, c3_b_emlrt_marshallOut(0U), "mFileTimeLo",
                  "mFileTimeLo", 241);
  sf_mex_addfield(*c3_info, c3_b_emlrt_marshallOut(0U), "mFileTimeHi",
                  "mFileTimeHi", 241);
  sf_mex_assign(&c3_rhs241, sf_mex_createcellmatrix(0, 1), false);
  sf_mex_assign(&c3_lhs241, sf_mex_createcellmatrix(0, 1), false);
  sf_mex_addfield(*c3_info, sf_mex_duplicatearraysafe(&c3_rhs241), "rhs", "rhs",
                  241);
  sf_mex_addfield(*c3_info, sf_mex_duplicatearraysafe(&c3_lhs241), "lhs", "lhs",
                  241);
  sf_mex_addfield(*c3_info, c3_emlrt_marshallOut(
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/eml/lapack/matlab/eml_matlab_zgeqp3.m"),
                  "context", "context", 242);
  sf_mex_addfield(*c3_info, c3_emlrt_marshallOut("eml_index_minus"), "name",
                  "name", 242);
  sf_mex_addfield(*c3_info, c3_emlrt_marshallOut("double"), "dominantType",
                  "dominantType", 242);
  sf_mex_addfield(*c3_info, c3_emlrt_marshallOut(
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/eml/eml_index_minus.m"),
                  "resolved", "resolved", 242);
  sf_mex_addfield(*c3_info, c3_b_emlrt_marshallOut(1372607616U), "fileTimeLo",
                  "fileTimeLo", 242);
  sf_mex_addfield(*c3_info, c3_b_emlrt_marshallOut(0U), "fileTimeHi",
                  "fileTimeHi", 242);
  sf_mex_addfield(*c3_info, c3_b_emlrt_marshallOut(0U), "mFileTimeLo",
                  "mFileTimeLo", 242);
  sf_mex_addfield(*c3_info, c3_b_emlrt_marshallOut(0U), "mFileTimeHi",
                  "mFileTimeHi", 242);
  sf_mex_assign(&c3_rhs242, sf_mex_createcellmatrix(0, 1), false);
  sf_mex_assign(&c3_lhs242, sf_mex_createcellmatrix(0, 1), false);
  sf_mex_addfield(*c3_info, sf_mex_duplicatearraysafe(&c3_rhs242), "rhs", "rhs",
                  242);
  sf_mex_addfield(*c3_info, sf_mex_duplicatearraysafe(&c3_lhs242), "lhs", "lhs",
                  242);
  sf_mex_addfield(*c3_info, c3_emlrt_marshallOut(
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/eml/lapack/matlab/eml_matlab_zgeqp3.m"),
                  "context", "context", 243);
  sf_mex_addfield(*c3_info, c3_emlrt_marshallOut("eml_index_plus"), "name",
                  "name", 243);
  sf_mex_addfield(*c3_info, c3_emlrt_marshallOut("double"), "dominantType",
                  "dominantType", 243);
  sf_mex_addfield(*c3_info, c3_emlrt_marshallOut(
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/eml/eml_index_plus.m"),
                  "resolved", "resolved", 243);
  sf_mex_addfield(*c3_info, c3_b_emlrt_marshallOut(1372607616U), "fileTimeLo",
                  "fileTimeLo", 243);
  sf_mex_addfield(*c3_info, c3_b_emlrt_marshallOut(0U), "fileTimeHi",
                  "fileTimeHi", 243);
  sf_mex_addfield(*c3_info, c3_b_emlrt_marshallOut(0U), "mFileTimeLo",
                  "mFileTimeLo", 243);
  sf_mex_addfield(*c3_info, c3_b_emlrt_marshallOut(0U), "mFileTimeHi",
                  "mFileTimeHi", 243);
  sf_mex_assign(&c3_rhs243, sf_mex_createcellmatrix(0, 1), false);
  sf_mex_assign(&c3_lhs243, sf_mex_createcellmatrix(0, 1), false);
  sf_mex_addfield(*c3_info, sf_mex_duplicatearraysafe(&c3_rhs243), "rhs", "rhs",
                  243);
  sf_mex_addfield(*c3_info, sf_mex_duplicatearraysafe(&c3_lhs243), "lhs", "lhs",
                  243);
  sf_mex_addfield(*c3_info, c3_emlrt_marshallOut(
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/eml/lapack/matlab/eml_matlab_zgeqp3.m"),
                  "context", "context", 244);
  sf_mex_addfield(*c3_info, c3_emlrt_marshallOut("eml_index_times"), "name",
                  "name", 244);
  sf_mex_addfield(*c3_info, c3_emlrt_marshallOut("coder.internal.indexInt"),
                  "dominantType", "dominantType", 244);
  sf_mex_addfield(*c3_info, c3_emlrt_marshallOut(
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/eml/eml_index_times.m"),
                  "resolved", "resolved", 244);
  sf_mex_addfield(*c3_info, c3_b_emlrt_marshallOut(1372607616U), "fileTimeLo",
                  "fileTimeLo", 244);
  sf_mex_addfield(*c3_info, c3_b_emlrt_marshallOut(0U), "fileTimeHi",
                  "fileTimeHi", 244);
  sf_mex_addfield(*c3_info, c3_b_emlrt_marshallOut(0U), "mFileTimeLo",
                  "mFileTimeLo", 244);
  sf_mex_addfield(*c3_info, c3_b_emlrt_marshallOut(0U), "mFileTimeHi",
                  "mFileTimeHi", 244);
  sf_mex_assign(&c3_rhs244, sf_mex_createcellmatrix(0, 1), false);
  sf_mex_assign(&c3_lhs244, sf_mex_createcellmatrix(0, 1), false);
  sf_mex_addfield(*c3_info, sf_mex_duplicatearraysafe(&c3_rhs244), "rhs", "rhs",
                  244);
  sf_mex_addfield(*c3_info, sf_mex_duplicatearraysafe(&c3_lhs244), "lhs", "lhs",
                  244);
  sf_mex_addfield(*c3_info, c3_emlrt_marshallOut(
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/eml/lapack/matlab/eml_matlab_zgeqp3.m"),
                  "context", "context", 245);
  sf_mex_addfield(*c3_info, c3_emlrt_marshallOut("eml_index_minus"), "name",
                  "name", 245);
  sf_mex_addfield(*c3_info, c3_emlrt_marshallOut("coder.internal.indexInt"),
                  "dominantType", "dominantType", 245);
  sf_mex_addfield(*c3_info, c3_emlrt_marshallOut(
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/eml/eml_index_minus.m"),
                  "resolved", "resolved", 245);
  sf_mex_addfield(*c3_info, c3_b_emlrt_marshallOut(1372607616U), "fileTimeLo",
                  "fileTimeLo", 245);
  sf_mex_addfield(*c3_info, c3_b_emlrt_marshallOut(0U), "fileTimeHi",
                  "fileTimeHi", 245);
  sf_mex_addfield(*c3_info, c3_b_emlrt_marshallOut(0U), "mFileTimeLo",
                  "mFileTimeLo", 245);
  sf_mex_addfield(*c3_info, c3_b_emlrt_marshallOut(0U), "mFileTimeHi",
                  "mFileTimeHi", 245);
  sf_mex_assign(&c3_rhs245, sf_mex_createcellmatrix(0, 1), false);
  sf_mex_assign(&c3_lhs245, sf_mex_createcellmatrix(0, 1), false);
  sf_mex_addfield(*c3_info, sf_mex_duplicatearraysafe(&c3_rhs245), "rhs", "rhs",
                  245);
  sf_mex_addfield(*c3_info, sf_mex_duplicatearraysafe(&c3_lhs245), "lhs", "lhs",
                  245);
  sf_mex_addfield(*c3_info, c3_emlrt_marshallOut(
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/eml/lapack/matlab/eml_matlab_zgeqp3.m"),
                  "context", "context", 246);
  sf_mex_addfield(*c3_info, c3_emlrt_marshallOut("eml_ixamax"), "name", "name",
                  246);
  sf_mex_addfield(*c3_info, c3_emlrt_marshallOut("double"), "dominantType",
                  "dominantType", 246);
  sf_mex_addfield(*c3_info, c3_emlrt_marshallOut(
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/eml/blas/eml_ixamax.m"),
                  "resolved", "resolved", 246);
  sf_mex_addfield(*c3_info, c3_b_emlrt_marshallOut(1376005888U), "fileTimeLo",
                  "fileTimeLo", 246);
  sf_mex_addfield(*c3_info, c3_b_emlrt_marshallOut(0U), "fileTimeHi",
                  "fileTimeHi", 246);
  sf_mex_addfield(*c3_info, c3_b_emlrt_marshallOut(0U), "mFileTimeLo",
                  "mFileTimeLo", 246);
  sf_mex_addfield(*c3_info, c3_b_emlrt_marshallOut(0U), "mFileTimeHi",
                  "mFileTimeHi", 246);
  sf_mex_assign(&c3_rhs246, sf_mex_createcellmatrix(0, 1), false);
  sf_mex_assign(&c3_lhs246, sf_mex_createcellmatrix(0, 1), false);
  sf_mex_addfield(*c3_info, sf_mex_duplicatearraysafe(&c3_rhs246), "rhs", "rhs",
                  246);
  sf_mex_addfield(*c3_info, sf_mex_duplicatearraysafe(&c3_lhs246), "lhs", "lhs",
                  246);
  sf_mex_addfield(*c3_info, c3_emlrt_marshallOut(
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/eml/lapack/matlab/eml_matlab_zgeqp3.m"),
                  "context", "context", 247);
  sf_mex_addfield(*c3_info, c3_emlrt_marshallOut("eml_xswap"), "name", "name",
                  247);
  sf_mex_addfield(*c3_info, c3_emlrt_marshallOut("double"), "dominantType",
                  "dominantType", 247);
  sf_mex_addfield(*c3_info, c3_emlrt_marshallOut(
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/eml/blas/eml_xswap.m"),
                  "resolved", "resolved", 247);
  sf_mex_addfield(*c3_info, c3_b_emlrt_marshallOut(1376005892U), "fileTimeLo",
                  "fileTimeLo", 247);
  sf_mex_addfield(*c3_info, c3_b_emlrt_marshallOut(0U), "fileTimeHi",
                  "fileTimeHi", 247);
  sf_mex_addfield(*c3_info, c3_b_emlrt_marshallOut(0U), "mFileTimeLo",
                  "mFileTimeLo", 247);
  sf_mex_addfield(*c3_info, c3_b_emlrt_marshallOut(0U), "mFileTimeHi",
                  "mFileTimeHi", 247);
  sf_mex_assign(&c3_rhs247, sf_mex_createcellmatrix(0, 1), false);
  sf_mex_assign(&c3_lhs247, sf_mex_createcellmatrix(0, 1), false);
  sf_mex_addfield(*c3_info, sf_mex_duplicatearraysafe(&c3_rhs247), "rhs", "rhs",
                  247);
  sf_mex_addfield(*c3_info, sf_mex_duplicatearraysafe(&c3_lhs247), "lhs", "lhs",
                  247);
  sf_mex_addfield(*c3_info, c3_emlrt_marshallOut(
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/eml/lapack/matlab/eml_matlab_zgeqp3.m"),
                  "context", "context", 248);
  sf_mex_addfield(*c3_info, c3_emlrt_marshallOut("eml_matlab_zlarfg"), "name",
                  "name", 248);
  sf_mex_addfield(*c3_info, c3_emlrt_marshallOut("double"), "dominantType",
                  "dominantType", 248);
  sf_mex_addfield(*c3_info, c3_emlrt_marshallOut(
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/eml/lapack/matlab/eml_matlab_zlarfg.m"),
                  "resolved", "resolved", 248);
  sf_mex_addfield(*c3_info, c3_b_emlrt_marshallOut(1389742904U), "fileTimeLo",
                  "fileTimeLo", 248);
  sf_mex_addfield(*c3_info, c3_b_emlrt_marshallOut(0U), "fileTimeHi",
                  "fileTimeHi", 248);
  sf_mex_addfield(*c3_info, c3_b_emlrt_marshallOut(0U), "mFileTimeLo",
                  "mFileTimeLo", 248);
  sf_mex_addfield(*c3_info, c3_b_emlrt_marshallOut(0U), "mFileTimeHi",
                  "mFileTimeHi", 248);
  sf_mex_assign(&c3_rhs248, sf_mex_createcellmatrix(0, 1), false);
  sf_mex_assign(&c3_lhs248, sf_mex_createcellmatrix(0, 1), false);
  sf_mex_addfield(*c3_info, sf_mex_duplicatearraysafe(&c3_rhs248), "rhs", "rhs",
                  248);
  sf_mex_addfield(*c3_info, sf_mex_duplicatearraysafe(&c3_lhs248), "lhs", "lhs",
                  248);
  sf_mex_addfield(*c3_info, c3_emlrt_marshallOut(
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/eml/lapack/matlab/eml_matlab_zlarfg.m"),
                  "context", "context", 249);
  sf_mex_addfield(*c3_info, c3_emlrt_marshallOut("eml_scalar_eg"), "name",
                  "name", 249);
  sf_mex_addfield(*c3_info, c3_emlrt_marshallOut("double"), "dominantType",
                  "dominantType", 249);
  sf_mex_addfield(*c3_info, c3_emlrt_marshallOut(
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/eml/eml_scalar_eg.m"), "resolved",
                  "resolved", 249);
  sf_mex_addfield(*c3_info, c3_b_emlrt_marshallOut(1376005888U), "fileTimeLo",
                  "fileTimeLo", 249);
  sf_mex_addfield(*c3_info, c3_b_emlrt_marshallOut(0U), "fileTimeHi",
                  "fileTimeHi", 249);
  sf_mex_addfield(*c3_info, c3_b_emlrt_marshallOut(0U), "mFileTimeLo",
                  "mFileTimeLo", 249);
  sf_mex_addfield(*c3_info, c3_b_emlrt_marshallOut(0U), "mFileTimeHi",
                  "mFileTimeHi", 249);
  sf_mex_assign(&c3_rhs249, sf_mex_createcellmatrix(0, 1), false);
  sf_mex_assign(&c3_lhs249, sf_mex_createcellmatrix(0, 1), false);
  sf_mex_addfield(*c3_info, sf_mex_duplicatearraysafe(&c3_rhs249), "rhs", "rhs",
                  249);
  sf_mex_addfield(*c3_info, sf_mex_duplicatearraysafe(&c3_lhs249), "lhs", "lhs",
                  249);
  sf_mex_addfield(*c3_info, c3_emlrt_marshallOut(
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/eml/lapack/matlab/eml_matlab_zlarfg.m"),
                  "context", "context", 250);
  sf_mex_addfield(*c3_info, c3_emlrt_marshallOut("eml_xnrm2"), "name", "name",
                  250);
  sf_mex_addfield(*c3_info, c3_emlrt_marshallOut("double"), "dominantType",
                  "dominantType", 250);
  sf_mex_addfield(*c3_info, c3_emlrt_marshallOut(
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/eml/blas/eml_xnrm2.m"),
                  "resolved", "resolved", 250);
  sf_mex_addfield(*c3_info, c3_b_emlrt_marshallOut(1376005892U), "fileTimeLo",
                  "fileTimeLo", 250);
  sf_mex_addfield(*c3_info, c3_b_emlrt_marshallOut(0U), "fileTimeHi",
                  "fileTimeHi", 250);
  sf_mex_addfield(*c3_info, c3_b_emlrt_marshallOut(0U), "mFileTimeLo",
                  "mFileTimeLo", 250);
  sf_mex_addfield(*c3_info, c3_b_emlrt_marshallOut(0U), "mFileTimeHi",
                  "mFileTimeHi", 250);
  sf_mex_assign(&c3_rhs250, sf_mex_createcellmatrix(0, 1), false);
  sf_mex_assign(&c3_lhs250, sf_mex_createcellmatrix(0, 1), false);
  sf_mex_addfield(*c3_info, sf_mex_duplicatearraysafe(&c3_rhs250), "rhs", "rhs",
                  250);
  sf_mex_addfield(*c3_info, sf_mex_duplicatearraysafe(&c3_lhs250), "lhs", "lhs",
                  250);
  sf_mex_addfield(*c3_info, c3_emlrt_marshallOut(
    "[ILXE]$matlabroot$/toolbox/coder/coder/+coder/+internal/+blas/xnrm2.p!below_threshold"),
                  "context", "context", 251);
  sf_mex_addfield(*c3_info, c3_emlrt_marshallOut("length"), "name", "name", 251);
  sf_mex_addfield(*c3_info, c3_emlrt_marshallOut("double"), "dominantType",
                  "dominantType", 251);
  sf_mex_addfield(*c3_info, c3_emlrt_marshallOut(
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/elmat/length.m"), "resolved",
                  "resolved", 251);
  sf_mex_addfield(*c3_info, c3_b_emlrt_marshallOut(1303171406U), "fileTimeLo",
                  "fileTimeLo", 251);
  sf_mex_addfield(*c3_info, c3_b_emlrt_marshallOut(0U), "fileTimeHi",
                  "fileTimeHi", 251);
  sf_mex_addfield(*c3_info, c3_b_emlrt_marshallOut(0U), "mFileTimeLo",
                  "mFileTimeLo", 251);
  sf_mex_addfield(*c3_info, c3_b_emlrt_marshallOut(0U), "mFileTimeHi",
                  "mFileTimeHi", 251);
  sf_mex_assign(&c3_rhs251, sf_mex_createcellmatrix(0, 1), false);
  sf_mex_assign(&c3_lhs251, sf_mex_createcellmatrix(0, 1), false);
  sf_mex_addfield(*c3_info, sf_mex_duplicatearraysafe(&c3_rhs251), "rhs", "rhs",
                  251);
  sf_mex_addfield(*c3_info, sf_mex_duplicatearraysafe(&c3_lhs251), "lhs", "lhs",
                  251);
  sf_mex_addfield(*c3_info, c3_emlrt_marshallOut(
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/eml/lapack/matlab/eml_matlab_zlarfg.m"),
                  "context", "context", 252);
  sf_mex_addfield(*c3_info, c3_emlrt_marshallOut("eml_dlapy2"), "name", "name",
                  252);
  sf_mex_addfield(*c3_info, c3_emlrt_marshallOut("double"), "dominantType",
                  "dominantType", 252);
  sf_mex_addfield(*c3_info, c3_emlrt_marshallOut(
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/eml/eml_dlapy2.m"), "resolved",
                  "resolved", 252);
  sf_mex_addfield(*c3_info, c3_b_emlrt_marshallOut(1350435854U), "fileTimeLo",
                  "fileTimeLo", 252);
  sf_mex_addfield(*c3_info, c3_b_emlrt_marshallOut(0U), "fileTimeHi",
                  "fileTimeHi", 252);
  sf_mex_addfield(*c3_info, c3_b_emlrt_marshallOut(0U), "mFileTimeLo",
                  "mFileTimeLo", 252);
  sf_mex_addfield(*c3_info, c3_b_emlrt_marshallOut(0U), "mFileTimeHi",
                  "mFileTimeHi", 252);
  sf_mex_assign(&c3_rhs252, sf_mex_createcellmatrix(0, 1), false);
  sf_mex_assign(&c3_lhs252, sf_mex_createcellmatrix(0, 1), false);
  sf_mex_addfield(*c3_info, sf_mex_duplicatearraysafe(&c3_rhs252), "rhs", "rhs",
                  252);
  sf_mex_addfield(*c3_info, sf_mex_duplicatearraysafe(&c3_lhs252), "lhs", "lhs",
                  252);
  sf_mex_addfield(*c3_info, c3_emlrt_marshallOut(
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/eml/lapack/matlab/eml_matlab_zlarfg.m"),
                  "context", "context", 253);
  sf_mex_addfield(*c3_info, c3_emlrt_marshallOut("realmin"), "name", "name", 253);
  sf_mex_addfield(*c3_info, c3_emlrt_marshallOut("char"), "dominantType",
                  "dominantType", 253);
  sf_mex_addfield(*c3_info, c3_emlrt_marshallOut(
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/elmat/realmin.m"), "resolved",
                  "resolved", 253);
  sf_mex_addfield(*c3_info, c3_b_emlrt_marshallOut(1307676442U), "fileTimeLo",
                  "fileTimeLo", 253);
  sf_mex_addfield(*c3_info, c3_b_emlrt_marshallOut(0U), "fileTimeHi",
                  "fileTimeHi", 253);
  sf_mex_addfield(*c3_info, c3_b_emlrt_marshallOut(0U), "mFileTimeLo",
                  "mFileTimeLo", 253);
  sf_mex_addfield(*c3_info, c3_b_emlrt_marshallOut(0U), "mFileTimeHi",
                  "mFileTimeHi", 253);
  sf_mex_assign(&c3_rhs253, sf_mex_createcellmatrix(0, 1), false);
  sf_mex_assign(&c3_lhs253, sf_mex_createcellmatrix(0, 1), false);
  sf_mex_addfield(*c3_info, sf_mex_duplicatearraysafe(&c3_rhs253), "rhs", "rhs",
                  253);
  sf_mex_addfield(*c3_info, sf_mex_duplicatearraysafe(&c3_lhs253), "lhs", "lhs",
                  253);
  sf_mex_addfield(*c3_info, c3_emlrt_marshallOut(
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/eml/lapack/matlab/eml_matlab_zlarfg.m"),
                  "context", "context", 254);
  sf_mex_addfield(*c3_info, c3_emlrt_marshallOut("eps"), "name", "name", 254);
  sf_mex_addfield(*c3_info, c3_emlrt_marshallOut("char"), "dominantType",
                  "dominantType", 254);
  sf_mex_addfield(*c3_info, c3_emlrt_marshallOut(
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/elmat/eps.m"), "resolved",
                  "resolved", 254);
  sf_mex_addfield(*c3_info, c3_b_emlrt_marshallOut(1326753196U), "fileTimeLo",
                  "fileTimeLo", 254);
  sf_mex_addfield(*c3_info, c3_b_emlrt_marshallOut(0U), "fileTimeHi",
                  "fileTimeHi", 254);
  sf_mex_addfield(*c3_info, c3_b_emlrt_marshallOut(0U), "mFileTimeLo",
                  "mFileTimeLo", 254);
  sf_mex_addfield(*c3_info, c3_b_emlrt_marshallOut(0U), "mFileTimeHi",
                  "mFileTimeHi", 254);
  sf_mex_assign(&c3_rhs254, sf_mex_createcellmatrix(0, 1), false);
  sf_mex_assign(&c3_lhs254, sf_mex_createcellmatrix(0, 1), false);
  sf_mex_addfield(*c3_info, sf_mex_duplicatearraysafe(&c3_rhs254), "rhs", "rhs",
                  254);
  sf_mex_addfield(*c3_info, sf_mex_duplicatearraysafe(&c3_lhs254), "lhs", "lhs",
                  254);
  sf_mex_addfield(*c3_info, c3_emlrt_marshallOut(
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/eml/lapack/matlab/eml_matlab_zlarfg.m"),
                  "context", "context", 255);
  sf_mex_addfield(*c3_info, c3_emlrt_marshallOut("abs"), "name", "name", 255);
  sf_mex_addfield(*c3_info, c3_emlrt_marshallOut("double"), "dominantType",
                  "dominantType", 255);
  sf_mex_addfield(*c3_info, c3_emlrt_marshallOut(
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/elfun/abs.m"), "resolved",
                  "resolved", 255);
  sf_mex_addfield(*c3_info, c3_b_emlrt_marshallOut(1363735452U), "fileTimeLo",
                  "fileTimeLo", 255);
  sf_mex_addfield(*c3_info, c3_b_emlrt_marshallOut(0U), "fileTimeHi",
                  "fileTimeHi", 255);
  sf_mex_addfield(*c3_info, c3_b_emlrt_marshallOut(0U), "mFileTimeLo",
                  "mFileTimeLo", 255);
  sf_mex_addfield(*c3_info, c3_b_emlrt_marshallOut(0U), "mFileTimeHi",
                  "mFileTimeHi", 255);
  sf_mex_assign(&c3_rhs255, sf_mex_createcellmatrix(0, 1), false);
  sf_mex_assign(&c3_lhs255, sf_mex_createcellmatrix(0, 1), false);
  sf_mex_addfield(*c3_info, sf_mex_duplicatearraysafe(&c3_rhs255), "rhs", "rhs",
                  255);
  sf_mex_addfield(*c3_info, sf_mex_duplicatearraysafe(&c3_lhs255), "lhs", "lhs",
                  255);
  sf_mex_destroy(&c3_rhs192);
  sf_mex_destroy(&c3_lhs192);
  sf_mex_destroy(&c3_rhs193);
  sf_mex_destroy(&c3_lhs193);
  sf_mex_destroy(&c3_rhs194);
  sf_mex_destroy(&c3_lhs194);
  sf_mex_destroy(&c3_rhs195);
  sf_mex_destroy(&c3_lhs195);
  sf_mex_destroy(&c3_rhs196);
  sf_mex_destroy(&c3_lhs196);
  sf_mex_destroy(&c3_rhs197);
  sf_mex_destroy(&c3_lhs197);
  sf_mex_destroy(&c3_rhs198);
  sf_mex_destroy(&c3_lhs198);
  sf_mex_destroy(&c3_rhs199);
  sf_mex_destroy(&c3_lhs199);
  sf_mex_destroy(&c3_rhs200);
  sf_mex_destroy(&c3_lhs200);
  sf_mex_destroy(&c3_rhs201);
  sf_mex_destroy(&c3_lhs201);
  sf_mex_destroy(&c3_rhs202);
  sf_mex_destroy(&c3_lhs202);
  sf_mex_destroy(&c3_rhs203);
  sf_mex_destroy(&c3_lhs203);
  sf_mex_destroy(&c3_rhs204);
  sf_mex_destroy(&c3_lhs204);
  sf_mex_destroy(&c3_rhs205);
  sf_mex_destroy(&c3_lhs205);
  sf_mex_destroy(&c3_rhs206);
  sf_mex_destroy(&c3_lhs206);
  sf_mex_destroy(&c3_rhs207);
  sf_mex_destroy(&c3_lhs207);
  sf_mex_destroy(&c3_rhs208);
  sf_mex_destroy(&c3_lhs208);
  sf_mex_destroy(&c3_rhs209);
  sf_mex_destroy(&c3_lhs209);
  sf_mex_destroy(&c3_rhs210);
  sf_mex_destroy(&c3_lhs210);
  sf_mex_destroy(&c3_rhs211);
  sf_mex_destroy(&c3_lhs211);
  sf_mex_destroy(&c3_rhs212);
  sf_mex_destroy(&c3_lhs212);
  sf_mex_destroy(&c3_rhs213);
  sf_mex_destroy(&c3_lhs213);
  sf_mex_destroy(&c3_rhs214);
  sf_mex_destroy(&c3_lhs214);
  sf_mex_destroy(&c3_rhs215);
  sf_mex_destroy(&c3_lhs215);
  sf_mex_destroy(&c3_rhs216);
  sf_mex_destroy(&c3_lhs216);
  sf_mex_destroy(&c3_rhs217);
  sf_mex_destroy(&c3_lhs217);
  sf_mex_destroy(&c3_rhs218);
  sf_mex_destroy(&c3_lhs218);
  sf_mex_destroy(&c3_rhs219);
  sf_mex_destroy(&c3_lhs219);
  sf_mex_destroy(&c3_rhs220);
  sf_mex_destroy(&c3_lhs220);
  sf_mex_destroy(&c3_rhs221);
  sf_mex_destroy(&c3_lhs221);
  sf_mex_destroy(&c3_rhs222);
  sf_mex_destroy(&c3_lhs222);
  sf_mex_destroy(&c3_rhs223);
  sf_mex_destroy(&c3_lhs223);
  sf_mex_destroy(&c3_rhs224);
  sf_mex_destroy(&c3_lhs224);
  sf_mex_destroy(&c3_rhs225);
  sf_mex_destroy(&c3_lhs225);
  sf_mex_destroy(&c3_rhs226);
  sf_mex_destroy(&c3_lhs226);
  sf_mex_destroy(&c3_rhs227);
  sf_mex_destroy(&c3_lhs227);
  sf_mex_destroy(&c3_rhs228);
  sf_mex_destroy(&c3_lhs228);
  sf_mex_destroy(&c3_rhs229);
  sf_mex_destroy(&c3_lhs229);
  sf_mex_destroy(&c3_rhs230);
  sf_mex_destroy(&c3_lhs230);
  sf_mex_destroy(&c3_rhs231);
  sf_mex_destroy(&c3_lhs231);
  sf_mex_destroy(&c3_rhs232);
  sf_mex_destroy(&c3_lhs232);
  sf_mex_destroy(&c3_rhs233);
  sf_mex_destroy(&c3_lhs233);
  sf_mex_destroy(&c3_rhs234);
  sf_mex_destroy(&c3_lhs234);
  sf_mex_destroy(&c3_rhs235);
  sf_mex_destroy(&c3_lhs235);
  sf_mex_destroy(&c3_rhs236);
  sf_mex_destroy(&c3_lhs236);
  sf_mex_destroy(&c3_rhs237);
  sf_mex_destroy(&c3_lhs237);
  sf_mex_destroy(&c3_rhs238);
  sf_mex_destroy(&c3_lhs238);
  sf_mex_destroy(&c3_rhs239);
  sf_mex_destroy(&c3_lhs239);
  sf_mex_destroy(&c3_rhs240);
  sf_mex_destroy(&c3_lhs240);
  sf_mex_destroy(&c3_rhs241);
  sf_mex_destroy(&c3_lhs241);
  sf_mex_destroy(&c3_rhs242);
  sf_mex_destroy(&c3_lhs242);
  sf_mex_destroy(&c3_rhs243);
  sf_mex_destroy(&c3_lhs243);
  sf_mex_destroy(&c3_rhs244);
  sf_mex_destroy(&c3_lhs244);
  sf_mex_destroy(&c3_rhs245);
  sf_mex_destroy(&c3_lhs245);
  sf_mex_destroy(&c3_rhs246);
  sf_mex_destroy(&c3_lhs246);
  sf_mex_destroy(&c3_rhs247);
  sf_mex_destroy(&c3_lhs247);
  sf_mex_destroy(&c3_rhs248);
  sf_mex_destroy(&c3_lhs248);
  sf_mex_destroy(&c3_rhs249);
  sf_mex_destroy(&c3_lhs249);
  sf_mex_destroy(&c3_rhs250);
  sf_mex_destroy(&c3_lhs250);
  sf_mex_destroy(&c3_rhs251);
  sf_mex_destroy(&c3_lhs251);
  sf_mex_destroy(&c3_rhs252);
  sf_mex_destroy(&c3_lhs252);
  sf_mex_destroy(&c3_rhs253);
  sf_mex_destroy(&c3_lhs253);
  sf_mex_destroy(&c3_rhs254);
  sf_mex_destroy(&c3_lhs254);
  sf_mex_destroy(&c3_rhs255);
  sf_mex_destroy(&c3_lhs255);
}

static void c3_e_info_helper(const mxArray **c3_info)
{
  const mxArray *c3_rhs256 = NULL;
  const mxArray *c3_lhs256 = NULL;
  const mxArray *c3_rhs257 = NULL;
  const mxArray *c3_lhs257 = NULL;
  const mxArray *c3_rhs258 = NULL;
  const mxArray *c3_lhs258 = NULL;
  const mxArray *c3_rhs259 = NULL;
  const mxArray *c3_lhs259 = NULL;
  const mxArray *c3_rhs260 = NULL;
  const mxArray *c3_lhs260 = NULL;
  const mxArray *c3_rhs261 = NULL;
  const mxArray *c3_lhs261 = NULL;
  const mxArray *c3_rhs262 = NULL;
  const mxArray *c3_lhs262 = NULL;
  const mxArray *c3_rhs263 = NULL;
  const mxArray *c3_lhs263 = NULL;
  const mxArray *c3_rhs264 = NULL;
  const mxArray *c3_lhs264 = NULL;
  const mxArray *c3_rhs265 = NULL;
  const mxArray *c3_lhs265 = NULL;
  const mxArray *c3_rhs266 = NULL;
  const mxArray *c3_lhs266 = NULL;
  const mxArray *c3_rhs267 = NULL;
  const mxArray *c3_lhs267 = NULL;
  const mxArray *c3_rhs268 = NULL;
  const mxArray *c3_lhs268 = NULL;
  const mxArray *c3_rhs269 = NULL;
  const mxArray *c3_lhs269 = NULL;
  const mxArray *c3_rhs270 = NULL;
  const mxArray *c3_lhs270 = NULL;
  const mxArray *c3_rhs271 = NULL;
  const mxArray *c3_lhs271 = NULL;
  const mxArray *c3_rhs272 = NULL;
  const mxArray *c3_lhs272 = NULL;
  const mxArray *c3_rhs273 = NULL;
  const mxArray *c3_lhs273 = NULL;
  const mxArray *c3_rhs274 = NULL;
  const mxArray *c3_lhs274 = NULL;
  const mxArray *c3_rhs275 = NULL;
  const mxArray *c3_lhs275 = NULL;
  const mxArray *c3_rhs276 = NULL;
  const mxArray *c3_lhs276 = NULL;
  const mxArray *c3_rhs277 = NULL;
  const mxArray *c3_lhs277 = NULL;
  const mxArray *c3_rhs278 = NULL;
  const mxArray *c3_lhs278 = NULL;
  const mxArray *c3_rhs279 = NULL;
  const mxArray *c3_lhs279 = NULL;
  const mxArray *c3_rhs280 = NULL;
  const mxArray *c3_lhs280 = NULL;
  const mxArray *c3_rhs281 = NULL;
  const mxArray *c3_lhs281 = NULL;
  const mxArray *c3_rhs282 = NULL;
  const mxArray *c3_lhs282 = NULL;
  const mxArray *c3_rhs283 = NULL;
  const mxArray *c3_lhs283 = NULL;
  const mxArray *c3_rhs284 = NULL;
  const mxArray *c3_lhs284 = NULL;
  const mxArray *c3_rhs285 = NULL;
  const mxArray *c3_lhs285 = NULL;
  const mxArray *c3_rhs286 = NULL;
  const mxArray *c3_lhs286 = NULL;
  const mxArray *c3_rhs287 = NULL;
  const mxArray *c3_lhs287 = NULL;
  const mxArray *c3_rhs288 = NULL;
  const mxArray *c3_lhs288 = NULL;
  const mxArray *c3_rhs289 = NULL;
  const mxArray *c3_lhs289 = NULL;
  const mxArray *c3_rhs290 = NULL;
  const mxArray *c3_lhs290 = NULL;
  const mxArray *c3_rhs291 = NULL;
  const mxArray *c3_lhs291 = NULL;
  const mxArray *c3_rhs292 = NULL;
  const mxArray *c3_lhs292 = NULL;
  const mxArray *c3_rhs293 = NULL;
  const mxArray *c3_lhs293 = NULL;
  const mxArray *c3_rhs294 = NULL;
  const mxArray *c3_lhs294 = NULL;
  const mxArray *c3_rhs295 = NULL;
  const mxArray *c3_lhs295 = NULL;
  const mxArray *c3_rhs296 = NULL;
  const mxArray *c3_lhs296 = NULL;
  const mxArray *c3_rhs297 = NULL;
  const mxArray *c3_lhs297 = NULL;
  const mxArray *c3_rhs298 = NULL;
  const mxArray *c3_lhs298 = NULL;
  const mxArray *c3_rhs299 = NULL;
  const mxArray *c3_lhs299 = NULL;
  const mxArray *c3_rhs300 = NULL;
  const mxArray *c3_lhs300 = NULL;
  const mxArray *c3_rhs301 = NULL;
  const mxArray *c3_lhs301 = NULL;
  const mxArray *c3_rhs302 = NULL;
  const mxArray *c3_lhs302 = NULL;
  const mxArray *c3_rhs303 = NULL;
  const mxArray *c3_lhs303 = NULL;
  const mxArray *c3_rhs304 = NULL;
  const mxArray *c3_lhs304 = NULL;
  const mxArray *c3_rhs305 = NULL;
  const mxArray *c3_lhs305 = NULL;
  const mxArray *c3_rhs306 = NULL;
  const mxArray *c3_lhs306 = NULL;
  const mxArray *c3_rhs307 = NULL;
  const mxArray *c3_lhs307 = NULL;
  const mxArray *c3_rhs308 = NULL;
  const mxArray *c3_lhs308 = NULL;
  const mxArray *c3_rhs309 = NULL;
  const mxArray *c3_lhs309 = NULL;
  const mxArray *c3_rhs310 = NULL;
  const mxArray *c3_lhs310 = NULL;
  const mxArray *c3_rhs311 = NULL;
  const mxArray *c3_lhs311 = NULL;
  const mxArray *c3_rhs312 = NULL;
  const mxArray *c3_lhs312 = NULL;
  const mxArray *c3_rhs313 = NULL;
  const mxArray *c3_lhs313 = NULL;
  const mxArray *c3_rhs314 = NULL;
  const mxArray *c3_lhs314 = NULL;
  const mxArray *c3_rhs315 = NULL;
  const mxArray *c3_lhs315 = NULL;
  const mxArray *c3_rhs316 = NULL;
  const mxArray *c3_lhs316 = NULL;
  const mxArray *c3_rhs317 = NULL;
  const mxArray *c3_lhs317 = NULL;
  const mxArray *c3_rhs318 = NULL;
  const mxArray *c3_lhs318 = NULL;
  const mxArray *c3_rhs319 = NULL;
  const mxArray *c3_lhs319 = NULL;
  sf_mex_addfield(*c3_info, c3_emlrt_marshallOut(
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/eml/lapack/matlab/eml_matlab_zlarfg.m"),
                  "context", "context", 256);
  sf_mex_addfield(*c3_info, c3_emlrt_marshallOut("eml_index_class"), "name",
                  "name", 256);
  sf_mex_addfield(*c3_info, c3_emlrt_marshallOut(""), "dominantType",
                  "dominantType", 256);
  sf_mex_addfield(*c3_info, c3_emlrt_marshallOut(
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/eml/eml_index_class.m"),
                  "resolved", "resolved", 256);
  sf_mex_addfield(*c3_info, c3_b_emlrt_marshallOut(1323195778U), "fileTimeLo",
                  "fileTimeLo", 256);
  sf_mex_addfield(*c3_info, c3_b_emlrt_marshallOut(0U), "fileTimeHi",
                  "fileTimeHi", 256);
  sf_mex_addfield(*c3_info, c3_b_emlrt_marshallOut(0U), "mFileTimeLo",
                  "mFileTimeLo", 256);
  sf_mex_addfield(*c3_info, c3_b_emlrt_marshallOut(0U), "mFileTimeHi",
                  "mFileTimeHi", 256);
  sf_mex_assign(&c3_rhs256, sf_mex_createcellmatrix(0, 1), false);
  sf_mex_assign(&c3_lhs256, sf_mex_createcellmatrix(0, 1), false);
  sf_mex_addfield(*c3_info, sf_mex_duplicatearraysafe(&c3_rhs256), "rhs", "rhs",
                  256);
  sf_mex_addfield(*c3_info, sf_mex_duplicatearraysafe(&c3_lhs256), "lhs", "lhs",
                  256);
  sf_mex_addfield(*c3_info, c3_emlrt_marshallOut(
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/eml/lapack/matlab/eml_matlab_zlarfg.m"),
                  "context", "context", 257);
  sf_mex_addfield(*c3_info, c3_emlrt_marshallOut("eml_index_plus"), "name",
                  "name", 257);
  sf_mex_addfield(*c3_info, c3_emlrt_marshallOut("double"), "dominantType",
                  "dominantType", 257);
  sf_mex_addfield(*c3_info, c3_emlrt_marshallOut(
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/eml/eml_index_plus.m"),
                  "resolved", "resolved", 257);
  sf_mex_addfield(*c3_info, c3_b_emlrt_marshallOut(1372607616U), "fileTimeLo",
                  "fileTimeLo", 257);
  sf_mex_addfield(*c3_info, c3_b_emlrt_marshallOut(0U), "fileTimeHi",
                  "fileTimeHi", 257);
  sf_mex_addfield(*c3_info, c3_b_emlrt_marshallOut(0U), "mFileTimeLo",
                  "mFileTimeLo", 257);
  sf_mex_addfield(*c3_info, c3_b_emlrt_marshallOut(0U), "mFileTimeHi",
                  "mFileTimeHi", 257);
  sf_mex_assign(&c3_rhs257, sf_mex_createcellmatrix(0, 1), false);
  sf_mex_assign(&c3_lhs257, sf_mex_createcellmatrix(0, 1), false);
  sf_mex_addfield(*c3_info, sf_mex_duplicatearraysafe(&c3_rhs257), "rhs", "rhs",
                  257);
  sf_mex_addfield(*c3_info, sf_mex_duplicatearraysafe(&c3_lhs257), "lhs", "lhs",
                  257);
  sf_mex_addfield(*c3_info, c3_emlrt_marshallOut(
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/eml/lapack/matlab/eml_matlab_zlarfg.m"),
                  "context", "context", 258);
  sf_mex_addfield(*c3_info, c3_emlrt_marshallOut("eml_xscal"), "name", "name",
                  258);
  sf_mex_addfield(*c3_info, c3_emlrt_marshallOut("double"), "dominantType",
                  "dominantType", 258);
  sf_mex_addfield(*c3_info, c3_emlrt_marshallOut(
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/eml/blas/eml_xscal.m"),
                  "resolved", "resolved", 258);
  sf_mex_addfield(*c3_info, c3_b_emlrt_marshallOut(1376005892U), "fileTimeLo",
                  "fileTimeLo", 258);
  sf_mex_addfield(*c3_info, c3_b_emlrt_marshallOut(0U), "fileTimeHi",
                  "fileTimeHi", 258);
  sf_mex_addfield(*c3_info, c3_b_emlrt_marshallOut(0U), "mFileTimeLo",
                  "mFileTimeLo", 258);
  sf_mex_addfield(*c3_info, c3_b_emlrt_marshallOut(0U), "mFileTimeHi",
                  "mFileTimeHi", 258);
  sf_mex_assign(&c3_rhs258, sf_mex_createcellmatrix(0, 1), false);
  sf_mex_assign(&c3_lhs258, sf_mex_createcellmatrix(0, 1), false);
  sf_mex_addfield(*c3_info, sf_mex_duplicatearraysafe(&c3_rhs258), "rhs", "rhs",
                  258);
  sf_mex_addfield(*c3_info, sf_mex_duplicatearraysafe(&c3_lhs258), "lhs", "lhs",
                  258);
  sf_mex_addfield(*c3_info, c3_emlrt_marshallOut(
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/eml/blas/eml_xscal.m"), "context",
                  "context", 259);
  sf_mex_addfield(*c3_info, c3_emlrt_marshallOut("coder.internal.blas.inline"),
                  "name", "name", 259);
  sf_mex_addfield(*c3_info, c3_emlrt_marshallOut(""), "dominantType",
                  "dominantType", 259);
  sf_mex_addfield(*c3_info, c3_emlrt_marshallOut(
    "[ILXE]$matlabroot$/toolbox/coder/coder/+coder/+internal/+blas/inline.p"),
                  "resolved", "resolved", 259);
  sf_mex_addfield(*c3_info, c3_b_emlrt_marshallOut(1410832972U), "fileTimeLo",
                  "fileTimeLo", 259);
  sf_mex_addfield(*c3_info, c3_b_emlrt_marshallOut(0U), "fileTimeHi",
                  "fileTimeHi", 259);
  sf_mex_addfield(*c3_info, c3_b_emlrt_marshallOut(0U), "mFileTimeLo",
                  "mFileTimeLo", 259);
  sf_mex_addfield(*c3_info, c3_b_emlrt_marshallOut(0U), "mFileTimeHi",
                  "mFileTimeHi", 259);
  sf_mex_assign(&c3_rhs259, sf_mex_createcellmatrix(0, 1), false);
  sf_mex_assign(&c3_lhs259, sf_mex_createcellmatrix(0, 1), false);
  sf_mex_addfield(*c3_info, sf_mex_duplicatearraysafe(&c3_rhs259), "rhs", "rhs",
                  259);
  sf_mex_addfield(*c3_info, sf_mex_duplicatearraysafe(&c3_lhs259), "lhs", "lhs",
                  259);
  sf_mex_addfield(*c3_info, c3_emlrt_marshallOut(
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/eml/blas/eml_xscal.m"), "context",
                  "context", 260);
  sf_mex_addfield(*c3_info, c3_emlrt_marshallOut("coder.internal.blas.xscal"),
                  "name", "name", 260);
  sf_mex_addfield(*c3_info, c3_emlrt_marshallOut("double"), "dominantType",
                  "dominantType", 260);
  sf_mex_addfield(*c3_info, c3_emlrt_marshallOut(
    "[ILXE]$matlabroot$/toolbox/coder/coder/+coder/+internal/+blas/xscal.p"),
                  "resolved", "resolved", 260);
  sf_mex_addfield(*c3_info, c3_b_emlrt_marshallOut(1410832972U), "fileTimeLo",
                  "fileTimeLo", 260);
  sf_mex_addfield(*c3_info, c3_b_emlrt_marshallOut(0U), "fileTimeHi",
                  "fileTimeHi", 260);
  sf_mex_addfield(*c3_info, c3_b_emlrt_marshallOut(0U), "mFileTimeLo",
                  "mFileTimeLo", 260);
  sf_mex_addfield(*c3_info, c3_b_emlrt_marshallOut(0U), "mFileTimeHi",
                  "mFileTimeHi", 260);
  sf_mex_assign(&c3_rhs260, sf_mex_createcellmatrix(0, 1), false);
  sf_mex_assign(&c3_lhs260, sf_mex_createcellmatrix(0, 1), false);
  sf_mex_addfield(*c3_info, sf_mex_duplicatearraysafe(&c3_rhs260), "rhs", "rhs",
                  260);
  sf_mex_addfield(*c3_info, sf_mex_duplicatearraysafe(&c3_lhs260), "lhs", "lhs",
                  260);
  sf_mex_addfield(*c3_info, c3_emlrt_marshallOut(
    "[ILXE]$matlabroot$/toolbox/coder/coder/+coder/+internal/+blas/xscal.p"),
                  "context", "context", 261);
  sf_mex_addfield(*c3_info, c3_emlrt_marshallOut(
    "coder.internal.blas.use_refblas"), "name", "name", 261);
  sf_mex_addfield(*c3_info, c3_emlrt_marshallOut(""), "dominantType",
                  "dominantType", 261);
  sf_mex_addfield(*c3_info, c3_emlrt_marshallOut(
    "[ILXE]$matlabroot$/toolbox/coder/coder/+coder/+internal/+blas/use_refblas.p"),
                  "resolved", "resolved", 261);
  sf_mex_addfield(*c3_info, c3_b_emlrt_marshallOut(1410832970U), "fileTimeLo",
                  "fileTimeLo", 261);
  sf_mex_addfield(*c3_info, c3_b_emlrt_marshallOut(0U), "fileTimeHi",
                  "fileTimeHi", 261);
  sf_mex_addfield(*c3_info, c3_b_emlrt_marshallOut(0U), "mFileTimeLo",
                  "mFileTimeLo", 261);
  sf_mex_addfield(*c3_info, c3_b_emlrt_marshallOut(0U), "mFileTimeHi",
                  "mFileTimeHi", 261);
  sf_mex_assign(&c3_rhs261, sf_mex_createcellmatrix(0, 1), false);
  sf_mex_assign(&c3_lhs261, sf_mex_createcellmatrix(0, 1), false);
  sf_mex_addfield(*c3_info, sf_mex_duplicatearraysafe(&c3_rhs261), "rhs", "rhs",
                  261);
  sf_mex_addfield(*c3_info, sf_mex_duplicatearraysafe(&c3_lhs261), "lhs", "lhs",
                  261);
  sf_mex_addfield(*c3_info, c3_emlrt_marshallOut(
    "[ILXE]$matlabroot$/toolbox/coder/coder/+coder/+internal/+blas/xscal.p!below_threshold"),
                  "context", "context", 262);
  sf_mex_addfield(*c3_info, c3_emlrt_marshallOut("coder.internal.blas.threshold"),
                  "name", "name", 262);
  sf_mex_addfield(*c3_info, c3_emlrt_marshallOut("char"), "dominantType",
                  "dominantType", 262);
  sf_mex_addfield(*c3_info, c3_emlrt_marshallOut(
    "[ILXE]$matlabroot$/toolbox/coder/coder/+coder/+internal/+blas/threshold.p"),
                  "resolved", "resolved", 262);
  sf_mex_addfield(*c3_info, c3_b_emlrt_marshallOut(1410832972U), "fileTimeLo",
                  "fileTimeLo", 262);
  sf_mex_addfield(*c3_info, c3_b_emlrt_marshallOut(0U), "fileTimeHi",
                  "fileTimeHi", 262);
  sf_mex_addfield(*c3_info, c3_b_emlrt_marshallOut(0U), "mFileTimeLo",
                  "mFileTimeLo", 262);
  sf_mex_addfield(*c3_info, c3_b_emlrt_marshallOut(0U), "mFileTimeHi",
                  "mFileTimeHi", 262);
  sf_mex_assign(&c3_rhs262, sf_mex_createcellmatrix(0, 1), false);
  sf_mex_assign(&c3_lhs262, sf_mex_createcellmatrix(0, 1), false);
  sf_mex_addfield(*c3_info, sf_mex_duplicatearraysafe(&c3_rhs262), "rhs", "rhs",
                  262);
  sf_mex_addfield(*c3_info, sf_mex_duplicatearraysafe(&c3_lhs262), "lhs", "lhs",
                  262);
  sf_mex_addfield(*c3_info, c3_emlrt_marshallOut(
    "[ILXE]$matlabroot$/toolbox/coder/coder/+coder/+internal/+blas/xscal.p!below_threshold"),
                  "context", "context", 263);
  sf_mex_addfield(*c3_info, c3_emlrt_marshallOut("length"), "name", "name", 263);
  sf_mex_addfield(*c3_info, c3_emlrt_marshallOut("double"), "dominantType",
                  "dominantType", 263);
  sf_mex_addfield(*c3_info, c3_emlrt_marshallOut(
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/elmat/length.m"), "resolved",
                  "resolved", 263);
  sf_mex_addfield(*c3_info, c3_b_emlrt_marshallOut(1303171406U), "fileTimeLo",
                  "fileTimeLo", 263);
  sf_mex_addfield(*c3_info, c3_b_emlrt_marshallOut(0U), "fileTimeHi",
                  "fileTimeHi", 263);
  sf_mex_addfield(*c3_info, c3_b_emlrt_marshallOut(0U), "mFileTimeLo",
                  "mFileTimeLo", 263);
  sf_mex_addfield(*c3_info, c3_b_emlrt_marshallOut(0U), "mFileTimeHi",
                  "mFileTimeHi", 263);
  sf_mex_assign(&c3_rhs263, sf_mex_createcellmatrix(0, 1), false);
  sf_mex_assign(&c3_lhs263, sf_mex_createcellmatrix(0, 1), false);
  sf_mex_addfield(*c3_info, sf_mex_duplicatearraysafe(&c3_rhs263), "rhs", "rhs",
                  263);
  sf_mex_addfield(*c3_info, sf_mex_duplicatearraysafe(&c3_lhs263), "lhs", "lhs",
                  263);
  sf_mex_addfield(*c3_info, c3_emlrt_marshallOut(
    "[ILXE]$matlabroot$/toolbox/coder/coder/+coder/+internal/+blas/xscal.p"),
                  "context", "context", 264);
  sf_mex_addfield(*c3_info, c3_emlrt_marshallOut("coder.internal.scalarEg"),
                  "name", "name", 264);
  sf_mex_addfield(*c3_info, c3_emlrt_marshallOut("double"), "dominantType",
                  "dominantType", 264);
  sf_mex_addfield(*c3_info, c3_emlrt_marshallOut(
    "[ILXE]$matlabroot$/toolbox/coder/coder/+coder/+internal/scalarEg.p"),
                  "resolved", "resolved", 264);
  sf_mex_addfield(*c3_info, c3_b_emlrt_marshallOut(1410832970U), "fileTimeLo",
                  "fileTimeLo", 264);
  sf_mex_addfield(*c3_info, c3_b_emlrt_marshallOut(0U), "fileTimeHi",
                  "fileTimeHi", 264);
  sf_mex_addfield(*c3_info, c3_b_emlrt_marshallOut(0U), "mFileTimeLo",
                  "mFileTimeLo", 264);
  sf_mex_addfield(*c3_info, c3_b_emlrt_marshallOut(0U), "mFileTimeHi",
                  "mFileTimeHi", 264);
  sf_mex_assign(&c3_rhs264, sf_mex_createcellmatrix(0, 1), false);
  sf_mex_assign(&c3_lhs264, sf_mex_createcellmatrix(0, 1), false);
  sf_mex_addfield(*c3_info, sf_mex_duplicatearraysafe(&c3_rhs264), "rhs", "rhs",
                  264);
  sf_mex_addfield(*c3_info, sf_mex_duplicatearraysafe(&c3_lhs264), "lhs", "lhs",
                  264);
  sf_mex_addfield(*c3_info, c3_emlrt_marshallOut(
    "[ILXE]$matlabroot$/toolbox/coder/coder/+coder/+internal/+blas/xscal.p"),
                  "context", "context", 265);
  sf_mex_addfield(*c3_info, c3_emlrt_marshallOut("coder.internal.refblas.xscal"),
                  "name", "name", 265);
  sf_mex_addfield(*c3_info, c3_emlrt_marshallOut("double"), "dominantType",
                  "dominantType", 265);
  sf_mex_addfield(*c3_info, c3_emlrt_marshallOut(
    "[ILXE]$matlabroot$/toolbox/coder/coder/+coder/+internal/+refblas/xscal.p"),
                  "resolved", "resolved", 265);
  sf_mex_addfield(*c3_info, c3_b_emlrt_marshallOut(1410832972U), "fileTimeLo",
                  "fileTimeLo", 265);
  sf_mex_addfield(*c3_info, c3_b_emlrt_marshallOut(0U), "fileTimeHi",
                  "fileTimeHi", 265);
  sf_mex_addfield(*c3_info, c3_b_emlrt_marshallOut(0U), "mFileTimeLo",
                  "mFileTimeLo", 265);
  sf_mex_addfield(*c3_info, c3_b_emlrt_marshallOut(0U), "mFileTimeHi",
                  "mFileTimeHi", 265);
  sf_mex_assign(&c3_rhs265, sf_mex_createcellmatrix(0, 1), false);
  sf_mex_assign(&c3_lhs265, sf_mex_createcellmatrix(0, 1), false);
  sf_mex_addfield(*c3_info, sf_mex_duplicatearraysafe(&c3_rhs265), "rhs", "rhs",
                  265);
  sf_mex_addfield(*c3_info, sf_mex_duplicatearraysafe(&c3_lhs265), "lhs", "lhs",
                  265);
  sf_mex_addfield(*c3_info, c3_emlrt_marshallOut(
    "[ILXE]$matlabroot$/toolbox/coder/coder/+coder/+internal/+refblas/xscal.p"),
                  "context", "context", 266);
  sf_mex_addfield(*c3_info, c3_emlrt_marshallOut("coder.internal.indexMinus"),
                  "name", "name", 266);
  sf_mex_addfield(*c3_info, c3_emlrt_marshallOut("double"), "dominantType",
                  "dominantType", 266);
  sf_mex_addfield(*c3_info, c3_emlrt_marshallOut(
    "[ILXE]$matlabroot$/toolbox/shared/coder/coder/+coder/+internal/indexMinus.m"),
                  "resolved", "resolved", 266);
  sf_mex_addfield(*c3_info, c3_b_emlrt_marshallOut(1372608360U), "fileTimeLo",
                  "fileTimeLo", 266);
  sf_mex_addfield(*c3_info, c3_b_emlrt_marshallOut(0U), "fileTimeHi",
                  "fileTimeHi", 266);
  sf_mex_addfield(*c3_info, c3_b_emlrt_marshallOut(0U), "mFileTimeLo",
                  "mFileTimeLo", 266);
  sf_mex_addfield(*c3_info, c3_b_emlrt_marshallOut(0U), "mFileTimeHi",
                  "mFileTimeHi", 266);
  sf_mex_assign(&c3_rhs266, sf_mex_createcellmatrix(0, 1), false);
  sf_mex_assign(&c3_lhs266, sf_mex_createcellmatrix(0, 1), false);
  sf_mex_addfield(*c3_info, sf_mex_duplicatearraysafe(&c3_rhs266), "rhs", "rhs",
                  266);
  sf_mex_addfield(*c3_info, sf_mex_duplicatearraysafe(&c3_lhs266), "lhs", "lhs",
                  266);
  sf_mex_addfield(*c3_info, c3_emlrt_marshallOut(
    "[ILXE]$matlabroot$/toolbox/coder/coder/+coder/+internal/+refblas/xscal.p"),
                  "context", "context", 267);
  sf_mex_addfield(*c3_info, c3_emlrt_marshallOut("coder.internal.indexTimes"),
                  "name", "name", 267);
  sf_mex_addfield(*c3_info, c3_emlrt_marshallOut("coder.internal.indexInt"),
                  "dominantType", "dominantType", 267);
  sf_mex_addfield(*c3_info, c3_emlrt_marshallOut(
    "[ILXE]$matlabroot$/toolbox/shared/coder/coder/+coder/+internal/indexTimes.m"),
                  "resolved", "resolved", 267);
  sf_mex_addfield(*c3_info, c3_b_emlrt_marshallOut(1372608360U), "fileTimeLo",
                  "fileTimeLo", 267);
  sf_mex_addfield(*c3_info, c3_b_emlrt_marshallOut(0U), "fileTimeHi",
                  "fileTimeHi", 267);
  sf_mex_addfield(*c3_info, c3_b_emlrt_marshallOut(0U), "mFileTimeLo",
                  "mFileTimeLo", 267);
  sf_mex_addfield(*c3_info, c3_b_emlrt_marshallOut(0U), "mFileTimeHi",
                  "mFileTimeHi", 267);
  sf_mex_assign(&c3_rhs267, sf_mex_createcellmatrix(0, 1), false);
  sf_mex_assign(&c3_lhs267, sf_mex_createcellmatrix(0, 1), false);
  sf_mex_addfield(*c3_info, sf_mex_duplicatearraysafe(&c3_rhs267), "rhs", "rhs",
                  267);
  sf_mex_addfield(*c3_info, sf_mex_duplicatearraysafe(&c3_lhs267), "lhs", "lhs",
                  267);
  sf_mex_addfield(*c3_info, c3_emlrt_marshallOut(
    "[ILXE]$matlabroot$/toolbox/coder/coder/+coder/+internal/+refblas/xscal.p"),
                  "context", "context", 268);
  sf_mex_addfield(*c3_info, c3_emlrt_marshallOut("coder.internal.indexPlus"),
                  "name", "name", 268);
  sf_mex_addfield(*c3_info, c3_emlrt_marshallOut("coder.internal.indexInt"),
                  "dominantType", "dominantType", 268);
  sf_mex_addfield(*c3_info, c3_emlrt_marshallOut(
    "[ILXE]$matlabroot$/toolbox/shared/coder/coder/+coder/+internal/indexPlus.m"),
                  "resolved", "resolved", 268);
  sf_mex_addfield(*c3_info, c3_b_emlrt_marshallOut(1372608360U), "fileTimeLo",
                  "fileTimeLo", 268);
  sf_mex_addfield(*c3_info, c3_b_emlrt_marshallOut(0U), "fileTimeHi",
                  "fileTimeHi", 268);
  sf_mex_addfield(*c3_info, c3_b_emlrt_marshallOut(0U), "mFileTimeLo",
                  "mFileTimeLo", 268);
  sf_mex_addfield(*c3_info, c3_b_emlrt_marshallOut(0U), "mFileTimeHi",
                  "mFileTimeHi", 268);
  sf_mex_assign(&c3_rhs268, sf_mex_createcellmatrix(0, 1), false);
  sf_mex_assign(&c3_lhs268, sf_mex_createcellmatrix(0, 1), false);
  sf_mex_addfield(*c3_info, sf_mex_duplicatearraysafe(&c3_rhs268), "rhs", "rhs",
                  268);
  sf_mex_addfield(*c3_info, sf_mex_duplicatearraysafe(&c3_lhs268), "lhs", "lhs",
                  268);
  sf_mex_addfield(*c3_info, c3_emlrt_marshallOut(
    "[ILXE]$matlabroot$/toolbox/coder/coder/+coder/+internal/+refblas/xscal.p"),
                  "context", "context", 269);
  sf_mex_addfield(*c3_info, c3_emlrt_marshallOut(
    "eml_int_forloop_overflow_check"), "name", "name", 269);
  sf_mex_addfield(*c3_info, c3_emlrt_marshallOut(""), "dominantType",
                  "dominantType", 269);
  sf_mex_addfield(*c3_info, c3_emlrt_marshallOut(
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/eml/eml_int_forloop_overflow_check.m"),
                  "resolved", "resolved", 269);
  sf_mex_addfield(*c3_info, c3_b_emlrt_marshallOut(1397282622U), "fileTimeLo",
                  "fileTimeLo", 269);
  sf_mex_addfield(*c3_info, c3_b_emlrt_marshallOut(0U), "fileTimeHi",
                  "fileTimeHi", 269);
  sf_mex_addfield(*c3_info, c3_b_emlrt_marshallOut(0U), "mFileTimeLo",
                  "mFileTimeLo", 269);
  sf_mex_addfield(*c3_info, c3_b_emlrt_marshallOut(0U), "mFileTimeHi",
                  "mFileTimeHi", 269);
  sf_mex_assign(&c3_rhs269, sf_mex_createcellmatrix(0, 1), false);
  sf_mex_assign(&c3_lhs269, sf_mex_createcellmatrix(0, 1), false);
  sf_mex_addfield(*c3_info, sf_mex_duplicatearraysafe(&c3_rhs269), "rhs", "rhs",
                  269);
  sf_mex_addfield(*c3_info, sf_mex_duplicatearraysafe(&c3_lhs269), "lhs", "lhs",
                  269);
  sf_mex_addfield(*c3_info, c3_emlrt_marshallOut(
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/eml/lapack/matlab/eml_matlab_zlarfg.m"),
                  "context", "context", 270);
  sf_mex_addfield(*c3_info, c3_emlrt_marshallOut("eml_div"), "name", "name", 270);
  sf_mex_addfield(*c3_info, c3_emlrt_marshallOut("double"), "dominantType",
                  "dominantType", 270);
  sf_mex_addfield(*c3_info, c3_emlrt_marshallOut(
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/eml/eml_div.m"), "resolved",
                  "resolved", 270);
  sf_mex_addfield(*c3_info, c3_b_emlrt_marshallOut(1386449152U), "fileTimeLo",
                  "fileTimeLo", 270);
  sf_mex_addfield(*c3_info, c3_b_emlrt_marshallOut(0U), "fileTimeHi",
                  "fileTimeHi", 270);
  sf_mex_addfield(*c3_info, c3_b_emlrt_marshallOut(0U), "mFileTimeLo",
                  "mFileTimeLo", 270);
  sf_mex_addfield(*c3_info, c3_b_emlrt_marshallOut(0U), "mFileTimeHi",
                  "mFileTimeHi", 270);
  sf_mex_assign(&c3_rhs270, sf_mex_createcellmatrix(0, 1), false);
  sf_mex_assign(&c3_lhs270, sf_mex_createcellmatrix(0, 1), false);
  sf_mex_addfield(*c3_info, sf_mex_duplicatearraysafe(&c3_rhs270), "rhs", "rhs",
                  270);
  sf_mex_addfield(*c3_info, sf_mex_duplicatearraysafe(&c3_lhs270), "lhs", "lhs",
                  270);
  sf_mex_addfield(*c3_info, c3_emlrt_marshallOut(
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/eml/lapack/matlab/eml_matlab_zlarfg.m"),
                  "context", "context", 271);
  sf_mex_addfield(*c3_info, c3_emlrt_marshallOut(
    "eml_int_forloop_overflow_check"), "name", "name", 271);
  sf_mex_addfield(*c3_info, c3_emlrt_marshallOut(""), "dominantType",
                  "dominantType", 271);
  sf_mex_addfield(*c3_info, c3_emlrt_marshallOut(
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/eml/eml_int_forloop_overflow_check.m"),
                  "resolved", "resolved", 271);
  sf_mex_addfield(*c3_info, c3_b_emlrt_marshallOut(1397282622U), "fileTimeLo",
                  "fileTimeLo", 271);
  sf_mex_addfield(*c3_info, c3_b_emlrt_marshallOut(0U), "fileTimeHi",
                  "fileTimeHi", 271);
  sf_mex_addfield(*c3_info, c3_b_emlrt_marshallOut(0U), "mFileTimeLo",
                  "mFileTimeLo", 271);
  sf_mex_addfield(*c3_info, c3_b_emlrt_marshallOut(0U), "mFileTimeHi",
                  "mFileTimeHi", 271);
  sf_mex_assign(&c3_rhs271, sf_mex_createcellmatrix(0, 1), false);
  sf_mex_assign(&c3_lhs271, sf_mex_createcellmatrix(0, 1), false);
  sf_mex_addfield(*c3_info, sf_mex_duplicatearraysafe(&c3_rhs271), "rhs", "rhs",
                  271);
  sf_mex_addfield(*c3_info, sf_mex_duplicatearraysafe(&c3_lhs271), "lhs", "lhs",
                  271);
  sf_mex_addfield(*c3_info, c3_emlrt_marshallOut(
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/eml/lapack/matlab/eml_matlab_zgeqp3.m"),
                  "context", "context", 272);
  sf_mex_addfield(*c3_info, c3_emlrt_marshallOut("eml_matlab_zlarf"), "name",
                  "name", 272);
  sf_mex_addfield(*c3_info, c3_emlrt_marshallOut("char"), "dominantType",
                  "dominantType", 272);
  sf_mex_addfield(*c3_info, c3_emlrt_marshallOut(
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/eml/lapack/matlab/eml_matlab_zlarf.m"),
                  "resolved", "resolved", 272);
  sf_mex_addfield(*c3_info, c3_b_emlrt_marshallOut(1286844022U), "fileTimeLo",
                  "fileTimeLo", 272);
  sf_mex_addfield(*c3_info, c3_b_emlrt_marshallOut(0U), "fileTimeHi",
                  "fileTimeHi", 272);
  sf_mex_addfield(*c3_info, c3_b_emlrt_marshallOut(0U), "mFileTimeLo",
                  "mFileTimeLo", 272);
  sf_mex_addfield(*c3_info, c3_b_emlrt_marshallOut(0U), "mFileTimeHi",
                  "mFileTimeHi", 272);
  sf_mex_assign(&c3_rhs272, sf_mex_createcellmatrix(0, 1), false);
  sf_mex_assign(&c3_lhs272, sf_mex_createcellmatrix(0, 1), false);
  sf_mex_addfield(*c3_info, sf_mex_duplicatearraysafe(&c3_rhs272), "rhs", "rhs",
                  272);
  sf_mex_addfield(*c3_info, sf_mex_duplicatearraysafe(&c3_lhs272), "lhs", "lhs",
                  272);
  sf_mex_addfield(*c3_info, c3_emlrt_marshallOut(
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/eml/lapack/matlab/eml_matlab_zlarf.m"),
                  "context", "context", 273);
  sf_mex_addfield(*c3_info, c3_emlrt_marshallOut("eml_scalar_eg"), "name",
                  "name", 273);
  sf_mex_addfield(*c3_info, c3_emlrt_marshallOut("double"), "dominantType",
                  "dominantType", 273);
  sf_mex_addfield(*c3_info, c3_emlrt_marshallOut(
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/eml/eml_scalar_eg.m"), "resolved",
                  "resolved", 273);
  sf_mex_addfield(*c3_info, c3_b_emlrt_marshallOut(1376005888U), "fileTimeLo",
                  "fileTimeLo", 273);
  sf_mex_addfield(*c3_info, c3_b_emlrt_marshallOut(0U), "fileTimeHi",
                  "fileTimeHi", 273);
  sf_mex_addfield(*c3_info, c3_b_emlrt_marshallOut(0U), "mFileTimeLo",
                  "mFileTimeLo", 273);
  sf_mex_addfield(*c3_info, c3_b_emlrt_marshallOut(0U), "mFileTimeHi",
                  "mFileTimeHi", 273);
  sf_mex_assign(&c3_rhs273, sf_mex_createcellmatrix(0, 1), false);
  sf_mex_assign(&c3_lhs273, sf_mex_createcellmatrix(0, 1), false);
  sf_mex_addfield(*c3_info, sf_mex_duplicatearraysafe(&c3_rhs273), "rhs", "rhs",
                  273);
  sf_mex_addfield(*c3_info, sf_mex_duplicatearraysafe(&c3_lhs273), "lhs", "lhs",
                  273);
  sf_mex_addfield(*c3_info, c3_emlrt_marshallOut(
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/eml/lapack/matlab/eml_matlab_zlarf.m"),
                  "context", "context", 274);
  sf_mex_addfield(*c3_info, c3_emlrt_marshallOut("eml_index_class"), "name",
                  "name", 274);
  sf_mex_addfield(*c3_info, c3_emlrt_marshallOut(""), "dominantType",
                  "dominantType", 274);
  sf_mex_addfield(*c3_info, c3_emlrt_marshallOut(
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/eml/eml_index_class.m"),
                  "resolved", "resolved", 274);
  sf_mex_addfield(*c3_info, c3_b_emlrt_marshallOut(1323195778U), "fileTimeLo",
                  "fileTimeLo", 274);
  sf_mex_addfield(*c3_info, c3_b_emlrt_marshallOut(0U), "fileTimeHi",
                  "fileTimeHi", 274);
  sf_mex_addfield(*c3_info, c3_b_emlrt_marshallOut(0U), "mFileTimeLo",
                  "mFileTimeLo", 274);
  sf_mex_addfield(*c3_info, c3_b_emlrt_marshallOut(0U), "mFileTimeHi",
                  "mFileTimeHi", 274);
  sf_mex_assign(&c3_rhs274, sf_mex_createcellmatrix(0, 1), false);
  sf_mex_assign(&c3_lhs274, sf_mex_createcellmatrix(0, 1), false);
  sf_mex_addfield(*c3_info, sf_mex_duplicatearraysafe(&c3_rhs274), "rhs", "rhs",
                  274);
  sf_mex_addfield(*c3_info, sf_mex_duplicatearraysafe(&c3_lhs274), "lhs", "lhs",
                  274);
  sf_mex_addfield(*c3_info, c3_emlrt_marshallOut(
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/eml/lapack/matlab/eml_matlab_zlarf.m"),
                  "context", "context", 275);
  sf_mex_addfield(*c3_info, c3_emlrt_marshallOut("isequal"), "name", "name", 275);
  sf_mex_addfield(*c3_info, c3_emlrt_marshallOut("double"), "dominantType",
                  "dominantType", 275);
  sf_mex_addfield(*c3_info, c3_emlrt_marshallOut(
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/elmat/isequal.m"), "resolved",
                  "resolved", 275);
  sf_mex_addfield(*c3_info, c3_b_emlrt_marshallOut(1286843958U), "fileTimeLo",
                  "fileTimeLo", 275);
  sf_mex_addfield(*c3_info, c3_b_emlrt_marshallOut(0U), "fileTimeHi",
                  "fileTimeHi", 275);
  sf_mex_addfield(*c3_info, c3_b_emlrt_marshallOut(0U), "mFileTimeLo",
                  "mFileTimeLo", 275);
  sf_mex_addfield(*c3_info, c3_b_emlrt_marshallOut(0U), "mFileTimeHi",
                  "mFileTimeHi", 275);
  sf_mex_assign(&c3_rhs275, sf_mex_createcellmatrix(0, 1), false);
  sf_mex_assign(&c3_lhs275, sf_mex_createcellmatrix(0, 1), false);
  sf_mex_addfield(*c3_info, sf_mex_duplicatearraysafe(&c3_rhs275), "rhs", "rhs",
                  275);
  sf_mex_addfield(*c3_info, sf_mex_duplicatearraysafe(&c3_lhs275), "lhs", "lhs",
                  275);
  sf_mex_addfield(*c3_info, c3_emlrt_marshallOut(
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/eml/lapack/matlab/eml_matlab_zlarf.m"),
                  "context", "context", 276);
  sf_mex_addfield(*c3_info, c3_emlrt_marshallOut("coder.internal.indexIntRelop"),
                  "name", "name", 276);
  sf_mex_addfield(*c3_info, c3_emlrt_marshallOut(""), "dominantType",
                  "dominantType", 276);
  sf_mex_addfield(*c3_info, c3_emlrt_marshallOut(
    "[ILXE]$matlabroot$/toolbox/shared/coder/coder/+coder/+internal/indexIntRelop.m"),
                  "resolved", "resolved", 276);
  sf_mex_addfield(*c3_info, c3_b_emlrt_marshallOut(1326753522U), "fileTimeLo",
                  "fileTimeLo", 276);
  sf_mex_addfield(*c3_info, c3_b_emlrt_marshallOut(0U), "fileTimeHi",
                  "fileTimeHi", 276);
  sf_mex_addfield(*c3_info, c3_b_emlrt_marshallOut(0U), "mFileTimeLo",
                  "mFileTimeLo", 276);
  sf_mex_addfield(*c3_info, c3_b_emlrt_marshallOut(0U), "mFileTimeHi",
                  "mFileTimeHi", 276);
  sf_mex_assign(&c3_rhs276, sf_mex_createcellmatrix(0, 1), false);
  sf_mex_assign(&c3_lhs276, sf_mex_createcellmatrix(0, 1), false);
  sf_mex_addfield(*c3_info, sf_mex_duplicatearraysafe(&c3_rhs276), "rhs", "rhs",
                  276);
  sf_mex_addfield(*c3_info, sf_mex_duplicatearraysafe(&c3_lhs276), "lhs", "lhs",
                  276);
  sf_mex_addfield(*c3_info, c3_emlrt_marshallOut(
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/eml/lapack/matlab/eml_matlab_zlarf.m"),
                  "context", "context", 277);
  sf_mex_addfield(*c3_info, c3_emlrt_marshallOut("eml_index_minus"), "name",
                  "name", 277);
  sf_mex_addfield(*c3_info, c3_emlrt_marshallOut("double"), "dominantType",
                  "dominantType", 277);
  sf_mex_addfield(*c3_info, c3_emlrt_marshallOut(
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/eml/eml_index_minus.m"),
                  "resolved", "resolved", 277);
  sf_mex_addfield(*c3_info, c3_b_emlrt_marshallOut(1372607616U), "fileTimeLo",
                  "fileTimeLo", 277);
  sf_mex_addfield(*c3_info, c3_b_emlrt_marshallOut(0U), "fileTimeHi",
                  "fileTimeHi", 277);
  sf_mex_addfield(*c3_info, c3_b_emlrt_marshallOut(0U), "mFileTimeLo",
                  "mFileTimeLo", 277);
  sf_mex_addfield(*c3_info, c3_b_emlrt_marshallOut(0U), "mFileTimeHi",
                  "mFileTimeHi", 277);
  sf_mex_assign(&c3_rhs277, sf_mex_createcellmatrix(0, 1), false);
  sf_mex_assign(&c3_lhs277, sf_mex_createcellmatrix(0, 1), false);
  sf_mex_addfield(*c3_info, sf_mex_duplicatearraysafe(&c3_rhs277), "rhs", "rhs",
                  277);
  sf_mex_addfield(*c3_info, sf_mex_duplicatearraysafe(&c3_lhs277), "lhs", "lhs",
                  277);
  sf_mex_addfield(*c3_info, c3_emlrt_marshallOut(
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/eml/lapack/matlab/eml_matlab_zlarf.m"),
                  "context", "context", 278);
  sf_mex_addfield(*c3_info, c3_emlrt_marshallOut("eml_index_times"), "name",
                  "name", 278);
  sf_mex_addfield(*c3_info, c3_emlrt_marshallOut("double"), "dominantType",
                  "dominantType", 278);
  sf_mex_addfield(*c3_info, c3_emlrt_marshallOut(
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/eml/eml_index_times.m"),
                  "resolved", "resolved", 278);
  sf_mex_addfield(*c3_info, c3_b_emlrt_marshallOut(1372607616U), "fileTimeLo",
                  "fileTimeLo", 278);
  sf_mex_addfield(*c3_info, c3_b_emlrt_marshallOut(0U), "fileTimeHi",
                  "fileTimeHi", 278);
  sf_mex_addfield(*c3_info, c3_b_emlrt_marshallOut(0U), "mFileTimeLo",
                  "mFileTimeLo", 278);
  sf_mex_addfield(*c3_info, c3_b_emlrt_marshallOut(0U), "mFileTimeHi",
                  "mFileTimeHi", 278);
  sf_mex_assign(&c3_rhs278, sf_mex_createcellmatrix(0, 1), false);
  sf_mex_assign(&c3_lhs278, sf_mex_createcellmatrix(0, 1), false);
  sf_mex_addfield(*c3_info, sf_mex_duplicatearraysafe(&c3_rhs278), "rhs", "rhs",
                  278);
  sf_mex_addfield(*c3_info, sf_mex_duplicatearraysafe(&c3_lhs278), "lhs", "lhs",
                  278);
  sf_mex_addfield(*c3_info, c3_emlrt_marshallOut(
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/eml/eml_index_times.m"),
                  "context", "context", 279);
  sf_mex_addfield(*c3_info, c3_emlrt_marshallOut("coder.internal.indexTimes"),
                  "name", "name", 279);
  sf_mex_addfield(*c3_info, c3_emlrt_marshallOut("double"), "dominantType",
                  "dominantType", 279);
  sf_mex_addfield(*c3_info, c3_emlrt_marshallOut(
    "[ILXE]$matlabroot$/toolbox/shared/coder/coder/+coder/+internal/indexTimes.m"),
                  "resolved", "resolved", 279);
  sf_mex_addfield(*c3_info, c3_b_emlrt_marshallOut(1372608360U), "fileTimeLo",
                  "fileTimeLo", 279);
  sf_mex_addfield(*c3_info, c3_b_emlrt_marshallOut(0U), "fileTimeHi",
                  "fileTimeHi", 279);
  sf_mex_addfield(*c3_info, c3_b_emlrt_marshallOut(0U), "mFileTimeLo",
                  "mFileTimeLo", 279);
  sf_mex_addfield(*c3_info, c3_b_emlrt_marshallOut(0U), "mFileTimeHi",
                  "mFileTimeHi", 279);
  sf_mex_assign(&c3_rhs279, sf_mex_createcellmatrix(0, 1), false);
  sf_mex_assign(&c3_lhs279, sf_mex_createcellmatrix(0, 1), false);
  sf_mex_addfield(*c3_info, sf_mex_duplicatearraysafe(&c3_rhs279), "rhs", "rhs",
                  279);
  sf_mex_addfield(*c3_info, sf_mex_duplicatearraysafe(&c3_lhs279), "lhs", "lhs",
                  279);
  sf_mex_addfield(*c3_info, c3_emlrt_marshallOut(
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/eml/lapack/matlab/eml_matlab_zlarf.m"),
                  "context", "context", 280);
  sf_mex_addfield(*c3_info, c3_emlrt_marshallOut("eml_index_plus"), "name",
                  "name", 280);
  sf_mex_addfield(*c3_info, c3_emlrt_marshallOut("coder.internal.indexInt"),
                  "dominantType", "dominantType", 280);
  sf_mex_addfield(*c3_info, c3_emlrt_marshallOut(
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/eml/eml_index_plus.m"),
                  "resolved", "resolved", 280);
  sf_mex_addfield(*c3_info, c3_b_emlrt_marshallOut(1372607616U), "fileTimeLo",
                  "fileTimeLo", 280);
  sf_mex_addfield(*c3_info, c3_b_emlrt_marshallOut(0U), "fileTimeHi",
                  "fileTimeHi", 280);
  sf_mex_addfield(*c3_info, c3_b_emlrt_marshallOut(0U), "mFileTimeLo",
                  "mFileTimeLo", 280);
  sf_mex_addfield(*c3_info, c3_b_emlrt_marshallOut(0U), "mFileTimeHi",
                  "mFileTimeHi", 280);
  sf_mex_assign(&c3_rhs280, sf_mex_createcellmatrix(0, 1), false);
  sf_mex_assign(&c3_lhs280, sf_mex_createcellmatrix(0, 1), false);
  sf_mex_addfield(*c3_info, sf_mex_duplicatearraysafe(&c3_rhs280), "rhs", "rhs",
                  280);
  sf_mex_addfield(*c3_info, sf_mex_duplicatearraysafe(&c3_lhs280), "lhs", "lhs",
                  280);
  sf_mex_addfield(*c3_info, c3_emlrt_marshallOut(
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/eml/lapack/matlab/eml_matlab_zlarf.m!ilazlc"),
                  "context", "context", 281);
  sf_mex_addfield(*c3_info, c3_emlrt_marshallOut("eml_index_class"), "name",
                  "name", 281);
  sf_mex_addfield(*c3_info, c3_emlrt_marshallOut(""), "dominantType",
                  "dominantType", 281);
  sf_mex_addfield(*c3_info, c3_emlrt_marshallOut(
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/eml/eml_index_class.m"),
                  "resolved", "resolved", 281);
  sf_mex_addfield(*c3_info, c3_b_emlrt_marshallOut(1323195778U), "fileTimeLo",
                  "fileTimeLo", 281);
  sf_mex_addfield(*c3_info, c3_b_emlrt_marshallOut(0U), "fileTimeHi",
                  "fileTimeHi", 281);
  sf_mex_addfield(*c3_info, c3_b_emlrt_marshallOut(0U), "mFileTimeLo",
                  "mFileTimeLo", 281);
  sf_mex_addfield(*c3_info, c3_b_emlrt_marshallOut(0U), "mFileTimeHi",
                  "mFileTimeHi", 281);
  sf_mex_assign(&c3_rhs281, sf_mex_createcellmatrix(0, 1), false);
  sf_mex_assign(&c3_lhs281, sf_mex_createcellmatrix(0, 1), false);
  sf_mex_addfield(*c3_info, sf_mex_duplicatearraysafe(&c3_rhs281), "rhs", "rhs",
                  281);
  sf_mex_addfield(*c3_info, sf_mex_duplicatearraysafe(&c3_lhs281), "lhs", "lhs",
                  281);
  sf_mex_addfield(*c3_info, c3_emlrt_marshallOut(
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/eml/lapack/matlab/eml_matlab_zlarf.m!ilazlc"),
                  "context", "context", 282);
  sf_mex_addfield(*c3_info, c3_emlrt_marshallOut("eml_index_minus"), "name",
                  "name", 282);
  sf_mex_addfield(*c3_info, c3_emlrt_marshallOut("double"), "dominantType",
                  "dominantType", 282);
  sf_mex_addfield(*c3_info, c3_emlrt_marshallOut(
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/eml/eml_index_minus.m"),
                  "resolved", "resolved", 282);
  sf_mex_addfield(*c3_info, c3_b_emlrt_marshallOut(1372607616U), "fileTimeLo",
                  "fileTimeLo", 282);
  sf_mex_addfield(*c3_info, c3_b_emlrt_marshallOut(0U), "fileTimeHi",
                  "fileTimeHi", 282);
  sf_mex_addfield(*c3_info, c3_b_emlrt_marshallOut(0U), "mFileTimeLo",
                  "mFileTimeLo", 282);
  sf_mex_addfield(*c3_info, c3_b_emlrt_marshallOut(0U), "mFileTimeHi",
                  "mFileTimeHi", 282);
  sf_mex_assign(&c3_rhs282, sf_mex_createcellmatrix(0, 1), false);
  sf_mex_assign(&c3_lhs282, sf_mex_createcellmatrix(0, 1), false);
  sf_mex_addfield(*c3_info, sf_mex_duplicatearraysafe(&c3_rhs282), "rhs", "rhs",
                  282);
  sf_mex_addfield(*c3_info, sf_mex_duplicatearraysafe(&c3_lhs282), "lhs", "lhs",
                  282);
  sf_mex_addfield(*c3_info, c3_emlrt_marshallOut(
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/eml/lapack/matlab/eml_matlab_zlarf.m!ilazlc"),
                  "context", "context", 283);
  sf_mex_addfield(*c3_info, c3_emlrt_marshallOut("eml_index_times"), "name",
                  "name", 283);
  sf_mex_addfield(*c3_info, c3_emlrt_marshallOut("coder.internal.indexInt"),
                  "dominantType", "dominantType", 283);
  sf_mex_addfield(*c3_info, c3_emlrt_marshallOut(
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/eml/eml_index_times.m"),
                  "resolved", "resolved", 283);
  sf_mex_addfield(*c3_info, c3_b_emlrt_marshallOut(1372607616U), "fileTimeLo",
                  "fileTimeLo", 283);
  sf_mex_addfield(*c3_info, c3_b_emlrt_marshallOut(0U), "fileTimeHi",
                  "fileTimeHi", 283);
  sf_mex_addfield(*c3_info, c3_b_emlrt_marshallOut(0U), "mFileTimeLo",
                  "mFileTimeLo", 283);
  sf_mex_addfield(*c3_info, c3_b_emlrt_marshallOut(0U), "mFileTimeHi",
                  "mFileTimeHi", 283);
  sf_mex_assign(&c3_rhs283, sf_mex_createcellmatrix(0, 1), false);
  sf_mex_assign(&c3_lhs283, sf_mex_createcellmatrix(0, 1), false);
  sf_mex_addfield(*c3_info, sf_mex_duplicatearraysafe(&c3_rhs283), "rhs", "rhs",
                  283);
  sf_mex_addfield(*c3_info, sf_mex_duplicatearraysafe(&c3_lhs283), "lhs", "lhs",
                  283);
  sf_mex_addfield(*c3_info, c3_emlrt_marshallOut(
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/eml/lapack/matlab/eml_matlab_zlarf.m!ilazlc"),
                  "context", "context", 284);
  sf_mex_addfield(*c3_info, c3_emlrt_marshallOut("eml_index_plus"), "name",
                  "name", 284);
  sf_mex_addfield(*c3_info, c3_emlrt_marshallOut("coder.internal.indexInt"),
                  "dominantType", "dominantType", 284);
  sf_mex_addfield(*c3_info, c3_emlrt_marshallOut(
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/eml/eml_index_plus.m"),
                  "resolved", "resolved", 284);
  sf_mex_addfield(*c3_info, c3_b_emlrt_marshallOut(1372607616U), "fileTimeLo",
                  "fileTimeLo", 284);
  sf_mex_addfield(*c3_info, c3_b_emlrt_marshallOut(0U), "fileTimeHi",
                  "fileTimeHi", 284);
  sf_mex_addfield(*c3_info, c3_b_emlrt_marshallOut(0U), "mFileTimeLo",
                  "mFileTimeLo", 284);
  sf_mex_addfield(*c3_info, c3_b_emlrt_marshallOut(0U), "mFileTimeHi",
                  "mFileTimeHi", 284);
  sf_mex_assign(&c3_rhs284, sf_mex_createcellmatrix(0, 1), false);
  sf_mex_assign(&c3_lhs284, sf_mex_createcellmatrix(0, 1), false);
  sf_mex_addfield(*c3_info, sf_mex_duplicatearraysafe(&c3_rhs284), "rhs", "rhs",
                  284);
  sf_mex_addfield(*c3_info, sf_mex_duplicatearraysafe(&c3_lhs284), "lhs", "lhs",
                  284);
  sf_mex_addfield(*c3_info, c3_emlrt_marshallOut(
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/eml/lapack/matlab/eml_matlab_zlarf.m!ilazlc"),
                  "context", "context", 285);
  sf_mex_addfield(*c3_info, c3_emlrt_marshallOut(
    "eml_int_forloop_overflow_check"), "name", "name", 285);
  sf_mex_addfield(*c3_info, c3_emlrt_marshallOut(""), "dominantType",
                  "dominantType", 285);
  sf_mex_addfield(*c3_info, c3_emlrt_marshallOut(
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/eml/eml_int_forloop_overflow_check.m"),
                  "resolved", "resolved", 285);
  sf_mex_addfield(*c3_info, c3_b_emlrt_marshallOut(1397282622U), "fileTimeLo",
                  "fileTimeLo", 285);
  sf_mex_addfield(*c3_info, c3_b_emlrt_marshallOut(0U), "fileTimeHi",
                  "fileTimeHi", 285);
  sf_mex_addfield(*c3_info, c3_b_emlrt_marshallOut(0U), "mFileTimeLo",
                  "mFileTimeLo", 285);
  sf_mex_addfield(*c3_info, c3_b_emlrt_marshallOut(0U), "mFileTimeHi",
                  "mFileTimeHi", 285);
  sf_mex_assign(&c3_rhs285, sf_mex_createcellmatrix(0, 1), false);
  sf_mex_assign(&c3_lhs285, sf_mex_createcellmatrix(0, 1), false);
  sf_mex_addfield(*c3_info, sf_mex_duplicatearraysafe(&c3_rhs285), "rhs", "rhs",
                  285);
  sf_mex_addfield(*c3_info, sf_mex_duplicatearraysafe(&c3_lhs285), "lhs", "lhs",
                  285);
  sf_mex_addfield(*c3_info, c3_emlrt_marshallOut(
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/eml/lapack/matlab/eml_matlab_zlarf.m"),
                  "context", "context", 286);
  sf_mex_addfield(*c3_info, c3_emlrt_marshallOut("eml_xgemv"), "name", "name",
                  286);
  sf_mex_addfield(*c3_info, c3_emlrt_marshallOut("char"), "dominantType",
                  "dominantType", 286);
  sf_mex_addfield(*c3_info, c3_emlrt_marshallOut(
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/eml/blas/eml_xgemv.m"),
                  "resolved", "resolved", 286);
  sf_mex_addfield(*c3_info, c3_b_emlrt_marshallOut(1376005890U), "fileTimeLo",
                  "fileTimeLo", 286);
  sf_mex_addfield(*c3_info, c3_b_emlrt_marshallOut(0U), "fileTimeHi",
                  "fileTimeHi", 286);
  sf_mex_addfield(*c3_info, c3_b_emlrt_marshallOut(0U), "mFileTimeLo",
                  "mFileTimeLo", 286);
  sf_mex_addfield(*c3_info, c3_b_emlrt_marshallOut(0U), "mFileTimeHi",
                  "mFileTimeHi", 286);
  sf_mex_assign(&c3_rhs286, sf_mex_createcellmatrix(0, 1), false);
  sf_mex_assign(&c3_lhs286, sf_mex_createcellmatrix(0, 1), false);
  sf_mex_addfield(*c3_info, sf_mex_duplicatearraysafe(&c3_rhs286), "rhs", "rhs",
                  286);
  sf_mex_addfield(*c3_info, sf_mex_duplicatearraysafe(&c3_lhs286), "lhs", "lhs",
                  286);
  sf_mex_addfield(*c3_info, c3_emlrt_marshallOut(
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/eml/blas/eml_xgemv.m"), "context",
                  "context", 287);
  sf_mex_addfield(*c3_info, c3_emlrt_marshallOut("coder.internal.blas.inline"),
                  "name", "name", 287);
  sf_mex_addfield(*c3_info, c3_emlrt_marshallOut(""), "dominantType",
                  "dominantType", 287);
  sf_mex_addfield(*c3_info, c3_emlrt_marshallOut(
    "[ILXE]$matlabroot$/toolbox/coder/coder/+coder/+internal/+blas/inline.p"),
                  "resolved", "resolved", 287);
  sf_mex_addfield(*c3_info, c3_b_emlrt_marshallOut(1410832972U), "fileTimeLo",
                  "fileTimeLo", 287);
  sf_mex_addfield(*c3_info, c3_b_emlrt_marshallOut(0U), "fileTimeHi",
                  "fileTimeHi", 287);
  sf_mex_addfield(*c3_info, c3_b_emlrt_marshallOut(0U), "mFileTimeLo",
                  "mFileTimeLo", 287);
  sf_mex_addfield(*c3_info, c3_b_emlrt_marshallOut(0U), "mFileTimeHi",
                  "mFileTimeHi", 287);
  sf_mex_assign(&c3_rhs287, sf_mex_createcellmatrix(0, 1), false);
  sf_mex_assign(&c3_lhs287, sf_mex_createcellmatrix(0, 1), false);
  sf_mex_addfield(*c3_info, sf_mex_duplicatearraysafe(&c3_rhs287), "rhs", "rhs",
                  287);
  sf_mex_addfield(*c3_info, sf_mex_duplicatearraysafe(&c3_lhs287), "lhs", "lhs",
                  287);
  sf_mex_addfield(*c3_info, c3_emlrt_marshallOut(
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/eml/blas/eml_xgemv.m"), "context",
                  "context", 288);
  sf_mex_addfield(*c3_info, c3_emlrt_marshallOut("coder.internal.blas.xgemv"),
                  "name", "name", 288);
  sf_mex_addfield(*c3_info, c3_emlrt_marshallOut("char"), "dominantType",
                  "dominantType", 288);
  sf_mex_addfield(*c3_info, c3_emlrt_marshallOut(
    "[ILXE]$matlabroot$/toolbox/coder/coder/+coder/+internal/+blas/xgemv.p"),
                  "resolved", "resolved", 288);
  sf_mex_addfield(*c3_info, c3_b_emlrt_marshallOut(1410832970U), "fileTimeLo",
                  "fileTimeLo", 288);
  sf_mex_addfield(*c3_info, c3_b_emlrt_marshallOut(0U), "fileTimeHi",
                  "fileTimeHi", 288);
  sf_mex_addfield(*c3_info, c3_b_emlrt_marshallOut(0U), "mFileTimeLo",
                  "mFileTimeLo", 288);
  sf_mex_addfield(*c3_info, c3_b_emlrt_marshallOut(0U), "mFileTimeHi",
                  "mFileTimeHi", 288);
  sf_mex_assign(&c3_rhs288, sf_mex_createcellmatrix(0, 1), false);
  sf_mex_assign(&c3_lhs288, sf_mex_createcellmatrix(0, 1), false);
  sf_mex_addfield(*c3_info, sf_mex_duplicatearraysafe(&c3_rhs288), "rhs", "rhs",
                  288);
  sf_mex_addfield(*c3_info, sf_mex_duplicatearraysafe(&c3_lhs288), "lhs", "lhs",
                  288);
  sf_mex_addfield(*c3_info, c3_emlrt_marshallOut(
    "[ILXE]$matlabroot$/toolbox/coder/coder/+coder/+internal/+blas/xgemv.p"),
                  "context", "context", 289);
  sf_mex_addfield(*c3_info, c3_emlrt_marshallOut(
    "coder.internal.blas.use_refblas"), "name", "name", 289);
  sf_mex_addfield(*c3_info, c3_emlrt_marshallOut(""), "dominantType",
                  "dominantType", 289);
  sf_mex_addfield(*c3_info, c3_emlrt_marshallOut(
    "[ILXE]$matlabroot$/toolbox/coder/coder/+coder/+internal/+blas/use_refblas.p"),
                  "resolved", "resolved", 289);
  sf_mex_addfield(*c3_info, c3_b_emlrt_marshallOut(1410832970U), "fileTimeLo",
                  "fileTimeLo", 289);
  sf_mex_addfield(*c3_info, c3_b_emlrt_marshallOut(0U), "fileTimeHi",
                  "fileTimeHi", 289);
  sf_mex_addfield(*c3_info, c3_b_emlrt_marshallOut(0U), "mFileTimeLo",
                  "mFileTimeLo", 289);
  sf_mex_addfield(*c3_info, c3_b_emlrt_marshallOut(0U), "mFileTimeHi",
                  "mFileTimeHi", 289);
  sf_mex_assign(&c3_rhs289, sf_mex_createcellmatrix(0, 1), false);
  sf_mex_assign(&c3_lhs289, sf_mex_createcellmatrix(0, 1), false);
  sf_mex_addfield(*c3_info, sf_mex_duplicatearraysafe(&c3_rhs289), "rhs", "rhs",
                  289);
  sf_mex_addfield(*c3_info, sf_mex_duplicatearraysafe(&c3_lhs289), "lhs", "lhs",
                  289);
  sf_mex_addfield(*c3_info, c3_emlrt_marshallOut(
    "[ILXE]$matlabroot$/toolbox/coder/coder/+coder/+internal/+blas/xgemv.p!below_threshold"),
                  "context", "context", 290);
  sf_mex_addfield(*c3_info, c3_emlrt_marshallOut("coder.internal.blas.threshold"),
                  "name", "name", 290);
  sf_mex_addfield(*c3_info, c3_emlrt_marshallOut("char"), "dominantType",
                  "dominantType", 290);
  sf_mex_addfield(*c3_info, c3_emlrt_marshallOut(
    "[ILXE]$matlabroot$/toolbox/coder/coder/+coder/+internal/+blas/threshold.p"),
                  "resolved", "resolved", 290);
  sf_mex_addfield(*c3_info, c3_b_emlrt_marshallOut(1410832972U), "fileTimeLo",
                  "fileTimeLo", 290);
  sf_mex_addfield(*c3_info, c3_b_emlrt_marshallOut(0U), "fileTimeHi",
                  "fileTimeHi", 290);
  sf_mex_addfield(*c3_info, c3_b_emlrt_marshallOut(0U), "mFileTimeLo",
                  "mFileTimeLo", 290);
  sf_mex_addfield(*c3_info, c3_b_emlrt_marshallOut(0U), "mFileTimeHi",
                  "mFileTimeHi", 290);
  sf_mex_assign(&c3_rhs290, sf_mex_createcellmatrix(0, 1), false);
  sf_mex_assign(&c3_lhs290, sf_mex_createcellmatrix(0, 1), false);
  sf_mex_addfield(*c3_info, sf_mex_duplicatearraysafe(&c3_rhs290), "rhs", "rhs",
                  290);
  sf_mex_addfield(*c3_info, sf_mex_duplicatearraysafe(&c3_lhs290), "lhs", "lhs",
                  290);
  sf_mex_addfield(*c3_info, c3_emlrt_marshallOut(
    "[ILXE]$matlabroot$/toolbox/coder/coder/+coder/+internal/+blas/xgemv.p!below_threshold"),
                  "context", "context", 291);
  sf_mex_addfield(*c3_info, c3_emlrt_marshallOut("length"), "name", "name", 291);
  sf_mex_addfield(*c3_info, c3_emlrt_marshallOut("double"), "dominantType",
                  "dominantType", 291);
  sf_mex_addfield(*c3_info, c3_emlrt_marshallOut(
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/elmat/length.m"), "resolved",
                  "resolved", 291);
  sf_mex_addfield(*c3_info, c3_b_emlrt_marshallOut(1303171406U), "fileTimeLo",
                  "fileTimeLo", 291);
  sf_mex_addfield(*c3_info, c3_b_emlrt_marshallOut(0U), "fileTimeHi",
                  "fileTimeHi", 291);
  sf_mex_addfield(*c3_info, c3_b_emlrt_marshallOut(0U), "mFileTimeLo",
                  "mFileTimeLo", 291);
  sf_mex_addfield(*c3_info, c3_b_emlrt_marshallOut(0U), "mFileTimeHi",
                  "mFileTimeHi", 291);
  sf_mex_assign(&c3_rhs291, sf_mex_createcellmatrix(0, 1), false);
  sf_mex_assign(&c3_lhs291, sf_mex_createcellmatrix(0, 1), false);
  sf_mex_addfield(*c3_info, sf_mex_duplicatearraysafe(&c3_rhs291), "rhs", "rhs",
                  291);
  sf_mex_addfield(*c3_info, sf_mex_duplicatearraysafe(&c3_lhs291), "lhs", "lhs",
                  291);
  sf_mex_addfield(*c3_info, c3_emlrt_marshallOut(
    "[ILXE]$matlabroot$/toolbox/coder/coder/+coder/+internal/+blas/xgemv.p!below_threshold"),
                  "context", "context", 292);
  sf_mex_addfield(*c3_info, c3_emlrt_marshallOut("min"), "name", "name", 292);
  sf_mex_addfield(*c3_info, c3_emlrt_marshallOut("double"), "dominantType",
                  "dominantType", 292);
  sf_mex_addfield(*c3_info, c3_emlrt_marshallOut(
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/datafun/min.m"), "resolved",
                  "resolved", 292);
  sf_mex_addfield(*c3_info, c3_b_emlrt_marshallOut(1311280518U), "fileTimeLo",
                  "fileTimeLo", 292);
  sf_mex_addfield(*c3_info, c3_b_emlrt_marshallOut(0U), "fileTimeHi",
                  "fileTimeHi", 292);
  sf_mex_addfield(*c3_info, c3_b_emlrt_marshallOut(0U), "mFileTimeLo",
                  "mFileTimeLo", 292);
  sf_mex_addfield(*c3_info, c3_b_emlrt_marshallOut(0U), "mFileTimeHi",
                  "mFileTimeHi", 292);
  sf_mex_assign(&c3_rhs292, sf_mex_createcellmatrix(0, 1), false);
  sf_mex_assign(&c3_lhs292, sf_mex_createcellmatrix(0, 1), false);
  sf_mex_addfield(*c3_info, sf_mex_duplicatearraysafe(&c3_rhs292), "rhs", "rhs",
                  292);
  sf_mex_addfield(*c3_info, sf_mex_duplicatearraysafe(&c3_lhs292), "lhs", "lhs",
                  292);
  sf_mex_addfield(*c3_info, c3_emlrt_marshallOut(
    "[ILXE]$matlabroot$/toolbox/coder/coder/+coder/+internal/+blas/xgemv.p"),
                  "context", "context", 293);
  sf_mex_addfield(*c3_info, c3_emlrt_marshallOut("coder.internal.scalarEg"),
                  "name", "name", 293);
  sf_mex_addfield(*c3_info, c3_emlrt_marshallOut("double"), "dominantType",
                  "dominantType", 293);
  sf_mex_addfield(*c3_info, c3_emlrt_marshallOut(
    "[ILXE]$matlabroot$/toolbox/coder/coder/+coder/+internal/scalarEg.p"),
                  "resolved", "resolved", 293);
  sf_mex_addfield(*c3_info, c3_b_emlrt_marshallOut(1410832970U), "fileTimeLo",
                  "fileTimeLo", 293);
  sf_mex_addfield(*c3_info, c3_b_emlrt_marshallOut(0U), "fileTimeHi",
                  "fileTimeHi", 293);
  sf_mex_addfield(*c3_info, c3_b_emlrt_marshallOut(0U), "mFileTimeLo",
                  "mFileTimeLo", 293);
  sf_mex_addfield(*c3_info, c3_b_emlrt_marshallOut(0U), "mFileTimeHi",
                  "mFileTimeHi", 293);
  sf_mex_assign(&c3_rhs293, sf_mex_createcellmatrix(0, 1), false);
  sf_mex_assign(&c3_lhs293, sf_mex_createcellmatrix(0, 1), false);
  sf_mex_addfield(*c3_info, sf_mex_duplicatearraysafe(&c3_rhs293), "rhs", "rhs",
                  293);
  sf_mex_addfield(*c3_info, sf_mex_duplicatearraysafe(&c3_lhs293), "lhs", "lhs",
                  293);
  sf_mex_addfield(*c3_info, c3_emlrt_marshallOut(
    "[ILXE]$matlabroot$/toolbox/coder/coder/+coder/+internal/+blas/xgemv.p"),
                  "context", "context", 294);
  sf_mex_addfield(*c3_info, c3_emlrt_marshallOut("coder.internal.refblas.xgemv"),
                  "name", "name", 294);
  sf_mex_addfield(*c3_info, c3_emlrt_marshallOut("char"), "dominantType",
                  "dominantType", 294);
  sf_mex_addfield(*c3_info, c3_emlrt_marshallOut(
    "[ILXE]$matlabroot$/toolbox/coder/coder/+coder/+internal/+refblas/xgemv.p"),
                  "resolved", "resolved", 294);
  sf_mex_addfield(*c3_info, c3_b_emlrt_marshallOut(1410832972U), "fileTimeLo",
                  "fileTimeLo", 294);
  sf_mex_addfield(*c3_info, c3_b_emlrt_marshallOut(0U), "fileTimeHi",
                  "fileTimeHi", 294);
  sf_mex_addfield(*c3_info, c3_b_emlrt_marshallOut(0U), "mFileTimeLo",
                  "mFileTimeLo", 294);
  sf_mex_addfield(*c3_info, c3_b_emlrt_marshallOut(0U), "mFileTimeHi",
                  "mFileTimeHi", 294);
  sf_mex_assign(&c3_rhs294, sf_mex_createcellmatrix(0, 1), false);
  sf_mex_assign(&c3_lhs294, sf_mex_createcellmatrix(0, 1), false);
  sf_mex_addfield(*c3_info, sf_mex_duplicatearraysafe(&c3_rhs294), "rhs", "rhs",
                  294);
  sf_mex_addfield(*c3_info, sf_mex_duplicatearraysafe(&c3_lhs294), "lhs", "lhs",
                  294);
  sf_mex_addfield(*c3_info, c3_emlrt_marshallOut(
    "[ILXE]$matlabroot$/toolbox/coder/coder/+coder/+internal/+refblas/xgemv.p"),
                  "context", "context", 295);
  sf_mex_addfield(*c3_info, c3_emlrt_marshallOut("coder.internal.indexMinus"),
                  "name", "name", 295);
  sf_mex_addfield(*c3_info, c3_emlrt_marshallOut("double"), "dominantType",
                  "dominantType", 295);
  sf_mex_addfield(*c3_info, c3_emlrt_marshallOut(
    "[ILXE]$matlabroot$/toolbox/shared/coder/coder/+coder/+internal/indexMinus.m"),
                  "resolved", "resolved", 295);
  sf_mex_addfield(*c3_info, c3_b_emlrt_marshallOut(1372608360U), "fileTimeLo",
                  "fileTimeLo", 295);
  sf_mex_addfield(*c3_info, c3_b_emlrt_marshallOut(0U), "fileTimeHi",
                  "fileTimeHi", 295);
  sf_mex_addfield(*c3_info, c3_b_emlrt_marshallOut(0U), "mFileTimeLo",
                  "mFileTimeLo", 295);
  sf_mex_addfield(*c3_info, c3_b_emlrt_marshallOut(0U), "mFileTimeHi",
                  "mFileTimeHi", 295);
  sf_mex_assign(&c3_rhs295, sf_mex_createcellmatrix(0, 1), false);
  sf_mex_assign(&c3_lhs295, sf_mex_createcellmatrix(0, 1), false);
  sf_mex_addfield(*c3_info, sf_mex_duplicatearraysafe(&c3_rhs295), "rhs", "rhs",
                  295);
  sf_mex_addfield(*c3_info, sf_mex_duplicatearraysafe(&c3_lhs295), "lhs", "lhs",
                  295);
  sf_mex_addfield(*c3_info, c3_emlrt_marshallOut(
    "[ILXE]$matlabroot$/toolbox/coder/coder/+coder/+internal/+refblas/xgemv.p"),
                  "context", "context", 296);
  sf_mex_addfield(*c3_info, c3_emlrt_marshallOut("coder.internal.indexTimes"),
                  "name", "name", 296);
  sf_mex_addfield(*c3_info, c3_emlrt_marshallOut("coder.internal.indexInt"),
                  "dominantType", "dominantType", 296);
  sf_mex_addfield(*c3_info, c3_emlrt_marshallOut(
    "[ILXE]$matlabroot$/toolbox/shared/coder/coder/+coder/+internal/indexTimes.m"),
                  "resolved", "resolved", 296);
  sf_mex_addfield(*c3_info, c3_b_emlrt_marshallOut(1372608360U), "fileTimeLo",
                  "fileTimeLo", 296);
  sf_mex_addfield(*c3_info, c3_b_emlrt_marshallOut(0U), "fileTimeHi",
                  "fileTimeHi", 296);
  sf_mex_addfield(*c3_info, c3_b_emlrt_marshallOut(0U), "mFileTimeLo",
                  "mFileTimeLo", 296);
  sf_mex_addfield(*c3_info, c3_b_emlrt_marshallOut(0U), "mFileTimeHi",
                  "mFileTimeHi", 296);
  sf_mex_assign(&c3_rhs296, sf_mex_createcellmatrix(0, 1), false);
  sf_mex_assign(&c3_lhs296, sf_mex_createcellmatrix(0, 1), false);
  sf_mex_addfield(*c3_info, sf_mex_duplicatearraysafe(&c3_rhs296), "rhs", "rhs",
                  296);
  sf_mex_addfield(*c3_info, sf_mex_duplicatearraysafe(&c3_lhs296), "lhs", "lhs",
                  296);
  sf_mex_addfield(*c3_info, c3_emlrt_marshallOut(
    "[ILXE]$matlabroot$/toolbox/coder/coder/+coder/+internal/+refblas/xgemv.p"),
                  "context", "context", 297);
  sf_mex_addfield(*c3_info, c3_emlrt_marshallOut("coder.internal.indexPlus"),
                  "name", "name", 297);
  sf_mex_addfield(*c3_info, c3_emlrt_marshallOut("coder.internal.indexInt"),
                  "dominantType", "dominantType", 297);
  sf_mex_addfield(*c3_info, c3_emlrt_marshallOut(
    "[ILXE]$matlabroot$/toolbox/shared/coder/coder/+coder/+internal/indexPlus.m"),
                  "resolved", "resolved", 297);
  sf_mex_addfield(*c3_info, c3_b_emlrt_marshallOut(1372608360U), "fileTimeLo",
                  "fileTimeLo", 297);
  sf_mex_addfield(*c3_info, c3_b_emlrt_marshallOut(0U), "fileTimeHi",
                  "fileTimeHi", 297);
  sf_mex_addfield(*c3_info, c3_b_emlrt_marshallOut(0U), "mFileTimeLo",
                  "mFileTimeLo", 297);
  sf_mex_addfield(*c3_info, c3_b_emlrt_marshallOut(0U), "mFileTimeHi",
                  "mFileTimeHi", 297);
  sf_mex_assign(&c3_rhs297, sf_mex_createcellmatrix(0, 1), false);
  sf_mex_assign(&c3_lhs297, sf_mex_createcellmatrix(0, 1), false);
  sf_mex_addfield(*c3_info, sf_mex_duplicatearraysafe(&c3_rhs297), "rhs", "rhs",
                  297);
  sf_mex_addfield(*c3_info, sf_mex_duplicatearraysafe(&c3_lhs297), "lhs", "lhs",
                  297);
  sf_mex_addfield(*c3_info, c3_emlrt_marshallOut(
    "[ILXE]$matlabroot$/toolbox/coder/coder/+coder/+internal/+refblas/xgemv.p"),
                  "context", "context", 298);
  sf_mex_addfield(*c3_info, c3_emlrt_marshallOut(
    "eml_int_forloop_overflow_check"), "name", "name", 298);
  sf_mex_addfield(*c3_info, c3_emlrt_marshallOut(""), "dominantType",
                  "dominantType", 298);
  sf_mex_addfield(*c3_info, c3_emlrt_marshallOut(
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/eml/eml_int_forloop_overflow_check.m"),
                  "resolved", "resolved", 298);
  sf_mex_addfield(*c3_info, c3_b_emlrt_marshallOut(1397282622U), "fileTimeLo",
                  "fileTimeLo", 298);
  sf_mex_addfield(*c3_info, c3_b_emlrt_marshallOut(0U), "fileTimeHi",
                  "fileTimeHi", 298);
  sf_mex_addfield(*c3_info, c3_b_emlrt_marshallOut(0U), "mFileTimeLo",
                  "mFileTimeLo", 298);
  sf_mex_addfield(*c3_info, c3_b_emlrt_marshallOut(0U), "mFileTimeHi",
                  "mFileTimeHi", 298);
  sf_mex_assign(&c3_rhs298, sf_mex_createcellmatrix(0, 1), false);
  sf_mex_assign(&c3_lhs298, sf_mex_createcellmatrix(0, 1), false);
  sf_mex_addfield(*c3_info, sf_mex_duplicatearraysafe(&c3_rhs298), "rhs", "rhs",
                  298);
  sf_mex_addfield(*c3_info, sf_mex_duplicatearraysafe(&c3_lhs298), "lhs", "lhs",
                  298);
  sf_mex_addfield(*c3_info, c3_emlrt_marshallOut(
    "[ILXE]$matlabroot$/toolbox/coder/coder/+coder/+internal/+refblas/xgemv.p"),
                  "context", "context", 299);
  sf_mex_addfield(*c3_info, c3_emlrt_marshallOut("coder.internal.scalarEg"),
                  "name", "name", 299);
  sf_mex_addfield(*c3_info, c3_emlrt_marshallOut("double"), "dominantType",
                  "dominantType", 299);
  sf_mex_addfield(*c3_info, c3_emlrt_marshallOut(
    "[ILXE]$matlabroot$/toolbox/coder/coder/+coder/+internal/scalarEg.p"),
                  "resolved", "resolved", 299);
  sf_mex_addfield(*c3_info, c3_b_emlrt_marshallOut(1410832970U), "fileTimeLo",
                  "fileTimeLo", 299);
  sf_mex_addfield(*c3_info, c3_b_emlrt_marshallOut(0U), "fileTimeHi",
                  "fileTimeHi", 299);
  sf_mex_addfield(*c3_info, c3_b_emlrt_marshallOut(0U), "mFileTimeLo",
                  "mFileTimeLo", 299);
  sf_mex_addfield(*c3_info, c3_b_emlrt_marshallOut(0U), "mFileTimeHi",
                  "mFileTimeHi", 299);
  sf_mex_assign(&c3_rhs299, sf_mex_createcellmatrix(0, 1), false);
  sf_mex_assign(&c3_lhs299, sf_mex_createcellmatrix(0, 1), false);
  sf_mex_addfield(*c3_info, sf_mex_duplicatearraysafe(&c3_rhs299), "rhs", "rhs",
                  299);
  sf_mex_addfield(*c3_info, sf_mex_duplicatearraysafe(&c3_lhs299), "lhs", "lhs",
                  299);
  sf_mex_addfield(*c3_info, c3_emlrt_marshallOut(
    "[ILXE]$matlabroot$/toolbox/coder/coder/+coder/+internal/+refblas/xgemv.p"),
                  "context", "context", 300);
  sf_mex_addfield(*c3_info, c3_emlrt_marshallOut("coder.internal.conjtimes"),
                  "name", "name", 300);
  sf_mex_addfield(*c3_info, c3_emlrt_marshallOut("double"), "dominantType",
                  "dominantType", 300);
  sf_mex_addfield(*c3_info, c3_emlrt_marshallOut(
    "[ILXE]$matlabroot$/toolbox/shared/coder/coder/+coder/+internal/conjtimes.m"),
                  "resolved", "resolved", 300);
  sf_mex_addfield(*c3_info, c3_b_emlrt_marshallOut(1360307786U), "fileTimeLo",
                  "fileTimeLo", 300);
  sf_mex_addfield(*c3_info, c3_b_emlrt_marshallOut(0U), "fileTimeHi",
                  "fileTimeHi", 300);
  sf_mex_addfield(*c3_info, c3_b_emlrt_marshallOut(0U), "mFileTimeLo",
                  "mFileTimeLo", 300);
  sf_mex_addfield(*c3_info, c3_b_emlrt_marshallOut(0U), "mFileTimeHi",
                  "mFileTimeHi", 300);
  sf_mex_assign(&c3_rhs300, sf_mex_createcellmatrix(0, 1), false);
  sf_mex_assign(&c3_lhs300, sf_mex_createcellmatrix(0, 1), false);
  sf_mex_addfield(*c3_info, sf_mex_duplicatearraysafe(&c3_rhs300), "rhs", "rhs",
                  300);
  sf_mex_addfield(*c3_info, sf_mex_duplicatearraysafe(&c3_lhs300), "lhs", "lhs",
                  300);
  sf_mex_addfield(*c3_info, c3_emlrt_marshallOut(
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/eml/lapack/matlab/eml_matlab_zlarf.m"),
                  "context", "context", 301);
  sf_mex_addfield(*c3_info, c3_emlrt_marshallOut("eml_xgerc"), "name", "name",
                  301);
  sf_mex_addfield(*c3_info, c3_emlrt_marshallOut("double"), "dominantType",
                  "dominantType", 301);
  sf_mex_addfield(*c3_info, c3_emlrt_marshallOut(
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/eml/blas/eml_xgerc.m"),
                  "resolved", "resolved", 301);
  sf_mex_addfield(*c3_info, c3_b_emlrt_marshallOut(1376005890U), "fileTimeLo",
                  "fileTimeLo", 301);
  sf_mex_addfield(*c3_info, c3_b_emlrt_marshallOut(0U), "fileTimeHi",
                  "fileTimeHi", 301);
  sf_mex_addfield(*c3_info, c3_b_emlrt_marshallOut(0U), "mFileTimeLo",
                  "mFileTimeLo", 301);
  sf_mex_addfield(*c3_info, c3_b_emlrt_marshallOut(0U), "mFileTimeHi",
                  "mFileTimeHi", 301);
  sf_mex_assign(&c3_rhs301, sf_mex_createcellmatrix(0, 1), false);
  sf_mex_assign(&c3_lhs301, sf_mex_createcellmatrix(0, 1), false);
  sf_mex_addfield(*c3_info, sf_mex_duplicatearraysafe(&c3_rhs301), "rhs", "rhs",
                  301);
  sf_mex_addfield(*c3_info, sf_mex_duplicatearraysafe(&c3_lhs301), "lhs", "lhs",
                  301);
  sf_mex_addfield(*c3_info, c3_emlrt_marshallOut(
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/eml/blas/eml_xgerc.m"), "context",
                  "context", 302);
  sf_mex_addfield(*c3_info, c3_emlrt_marshallOut("coder.internal.blas.inline"),
                  "name", "name", 302);
  sf_mex_addfield(*c3_info, c3_emlrt_marshallOut(""), "dominantType",
                  "dominantType", 302);
  sf_mex_addfield(*c3_info, c3_emlrt_marshallOut(
    "[ILXE]$matlabroot$/toolbox/coder/coder/+coder/+internal/+blas/inline.p"),
                  "resolved", "resolved", 302);
  sf_mex_addfield(*c3_info, c3_b_emlrt_marshallOut(1410832972U), "fileTimeLo",
                  "fileTimeLo", 302);
  sf_mex_addfield(*c3_info, c3_b_emlrt_marshallOut(0U), "fileTimeHi",
                  "fileTimeHi", 302);
  sf_mex_addfield(*c3_info, c3_b_emlrt_marshallOut(0U), "mFileTimeLo",
                  "mFileTimeLo", 302);
  sf_mex_addfield(*c3_info, c3_b_emlrt_marshallOut(0U), "mFileTimeHi",
                  "mFileTimeHi", 302);
  sf_mex_assign(&c3_rhs302, sf_mex_createcellmatrix(0, 1), false);
  sf_mex_assign(&c3_lhs302, sf_mex_createcellmatrix(0, 1), false);
  sf_mex_addfield(*c3_info, sf_mex_duplicatearraysafe(&c3_rhs302), "rhs", "rhs",
                  302);
  sf_mex_addfield(*c3_info, sf_mex_duplicatearraysafe(&c3_lhs302), "lhs", "lhs",
                  302);
  sf_mex_addfield(*c3_info, c3_emlrt_marshallOut(
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/eml/blas/eml_xgerc.m"), "context",
                  "context", 303);
  sf_mex_addfield(*c3_info, c3_emlrt_marshallOut("coder.internal.blas.xgerc"),
                  "name", "name", 303);
  sf_mex_addfield(*c3_info, c3_emlrt_marshallOut("double"), "dominantType",
                  "dominantType", 303);
  sf_mex_addfield(*c3_info, c3_emlrt_marshallOut(
    "[ILXE]$matlabroot$/toolbox/coder/coder/+coder/+internal/+blas/xgerc.p"),
                  "resolved", "resolved", 303);
  sf_mex_addfield(*c3_info, c3_b_emlrt_marshallOut(1410832970U), "fileTimeLo",
                  "fileTimeLo", 303);
  sf_mex_addfield(*c3_info, c3_b_emlrt_marshallOut(0U), "fileTimeHi",
                  "fileTimeHi", 303);
  sf_mex_addfield(*c3_info, c3_b_emlrt_marshallOut(0U), "mFileTimeLo",
                  "mFileTimeLo", 303);
  sf_mex_addfield(*c3_info, c3_b_emlrt_marshallOut(0U), "mFileTimeHi",
                  "mFileTimeHi", 303);
  sf_mex_assign(&c3_rhs303, sf_mex_createcellmatrix(0, 1), false);
  sf_mex_assign(&c3_lhs303, sf_mex_createcellmatrix(0, 1), false);
  sf_mex_addfield(*c3_info, sf_mex_duplicatearraysafe(&c3_rhs303), "rhs", "rhs",
                  303);
  sf_mex_addfield(*c3_info, sf_mex_duplicatearraysafe(&c3_lhs303), "lhs", "lhs",
                  303);
  sf_mex_addfield(*c3_info, c3_emlrt_marshallOut(
    "[ILXE]$matlabroot$/toolbox/coder/coder/+coder/+internal/+blas/xgerc.p"),
                  "context", "context", 304);
  sf_mex_addfield(*c3_info, c3_emlrt_marshallOut("coder.internal.blas.xger"),
                  "name", "name", 304);
  sf_mex_addfield(*c3_info, c3_emlrt_marshallOut("double"), "dominantType",
                  "dominantType", 304);
  sf_mex_addfield(*c3_info, c3_emlrt_marshallOut(
    "[ILXE]$matlabroot$/toolbox/coder/coder/+coder/+internal/+blas/xger.p"),
                  "resolved", "resolved", 304);
  sf_mex_addfield(*c3_info, c3_b_emlrt_marshallOut(1410832970U), "fileTimeLo",
                  "fileTimeLo", 304);
  sf_mex_addfield(*c3_info, c3_b_emlrt_marshallOut(0U), "fileTimeHi",
                  "fileTimeHi", 304);
  sf_mex_addfield(*c3_info, c3_b_emlrt_marshallOut(0U), "mFileTimeLo",
                  "mFileTimeLo", 304);
  sf_mex_addfield(*c3_info, c3_b_emlrt_marshallOut(0U), "mFileTimeHi",
                  "mFileTimeHi", 304);
  sf_mex_assign(&c3_rhs304, sf_mex_createcellmatrix(0, 1), false);
  sf_mex_assign(&c3_lhs304, sf_mex_createcellmatrix(0, 1), false);
  sf_mex_addfield(*c3_info, sf_mex_duplicatearraysafe(&c3_rhs304), "rhs", "rhs",
                  304);
  sf_mex_addfield(*c3_info, sf_mex_duplicatearraysafe(&c3_lhs304), "lhs", "lhs",
                  304);
  sf_mex_addfield(*c3_info, c3_emlrt_marshallOut(
    "[ILXE]$matlabroot$/toolbox/coder/coder/+coder/+internal/+blas/xger.p!below_threshold"),
                  "context", "context", 305);
  sf_mex_addfield(*c3_info, c3_emlrt_marshallOut("length"), "name", "name", 305);
  sf_mex_addfield(*c3_info, c3_emlrt_marshallOut("double"), "dominantType",
                  "dominantType", 305);
  sf_mex_addfield(*c3_info, c3_emlrt_marshallOut(
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/elmat/length.m"), "resolved",
                  "resolved", 305);
  sf_mex_addfield(*c3_info, c3_b_emlrt_marshallOut(1303171406U), "fileTimeLo",
                  "fileTimeLo", 305);
  sf_mex_addfield(*c3_info, c3_b_emlrt_marshallOut(0U), "fileTimeHi",
                  "fileTimeHi", 305);
  sf_mex_addfield(*c3_info, c3_b_emlrt_marshallOut(0U), "mFileTimeLo",
                  "mFileTimeLo", 305);
  sf_mex_addfield(*c3_info, c3_b_emlrt_marshallOut(0U), "mFileTimeHi",
                  "mFileTimeHi", 305);
  sf_mex_assign(&c3_rhs305, sf_mex_createcellmatrix(0, 1), false);
  sf_mex_assign(&c3_lhs305, sf_mex_createcellmatrix(0, 1), false);
  sf_mex_addfield(*c3_info, sf_mex_duplicatearraysafe(&c3_rhs305), "rhs", "rhs",
                  305);
  sf_mex_addfield(*c3_info, sf_mex_duplicatearraysafe(&c3_lhs305), "lhs", "lhs",
                  305);
  sf_mex_addfield(*c3_info, c3_emlrt_marshallOut(
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/eml/lapack/matlab/eml_matlab_zgeqp3.m"),
                  "context", "context", 306);
  sf_mex_addfield(*c3_info, c3_emlrt_marshallOut("abs"), "name", "name", 306);
  sf_mex_addfield(*c3_info, c3_emlrt_marshallOut("double"), "dominantType",
                  "dominantType", 306);
  sf_mex_addfield(*c3_info, c3_emlrt_marshallOut(
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/elfun/abs.m"), "resolved",
                  "resolved", 306);
  sf_mex_addfield(*c3_info, c3_b_emlrt_marshallOut(1363735452U), "fileTimeLo",
                  "fileTimeLo", 306);
  sf_mex_addfield(*c3_info, c3_b_emlrt_marshallOut(0U), "fileTimeHi",
                  "fileTimeHi", 306);
  sf_mex_addfield(*c3_info, c3_b_emlrt_marshallOut(0U), "mFileTimeLo",
                  "mFileTimeLo", 306);
  sf_mex_addfield(*c3_info, c3_b_emlrt_marshallOut(0U), "mFileTimeHi",
                  "mFileTimeHi", 306);
  sf_mex_assign(&c3_rhs306, sf_mex_createcellmatrix(0, 1), false);
  sf_mex_assign(&c3_lhs306, sf_mex_createcellmatrix(0, 1), false);
  sf_mex_addfield(*c3_info, sf_mex_duplicatearraysafe(&c3_rhs306), "rhs", "rhs",
                  306);
  sf_mex_addfield(*c3_info, sf_mex_duplicatearraysafe(&c3_lhs306), "lhs", "lhs",
                  306);
  sf_mex_addfield(*c3_info, c3_emlrt_marshallOut(
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/eml/eml_qrsolve.m"), "context",
                  "context", 307);
  sf_mex_addfield(*c3_info, c3_emlrt_marshallOut("max"), "name", "name", 307);
  sf_mex_addfield(*c3_info, c3_emlrt_marshallOut("double"), "dominantType",
                  "dominantType", 307);
  sf_mex_addfield(*c3_info, c3_emlrt_marshallOut(
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/datafun/max.m"), "resolved",
                  "resolved", 307);
  sf_mex_addfield(*c3_info, c3_b_emlrt_marshallOut(1311280516U), "fileTimeLo",
                  "fileTimeLo", 307);
  sf_mex_addfield(*c3_info, c3_b_emlrt_marshallOut(0U), "fileTimeHi",
                  "fileTimeHi", 307);
  sf_mex_addfield(*c3_info, c3_b_emlrt_marshallOut(0U), "mFileTimeLo",
                  "mFileTimeLo", 307);
  sf_mex_addfield(*c3_info, c3_b_emlrt_marshallOut(0U), "mFileTimeHi",
                  "mFileTimeHi", 307);
  sf_mex_assign(&c3_rhs307, sf_mex_createcellmatrix(0, 1), false);
  sf_mex_assign(&c3_lhs307, sf_mex_createcellmatrix(0, 1), false);
  sf_mex_addfield(*c3_info, sf_mex_duplicatearraysafe(&c3_rhs307), "rhs", "rhs",
                  307);
  sf_mex_addfield(*c3_info, sf_mex_duplicatearraysafe(&c3_lhs307), "lhs", "lhs",
                  307);
  sf_mex_addfield(*c3_info, c3_emlrt_marshallOut(
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/datafun/max.m"), "context",
                  "context", 308);
  sf_mex_addfield(*c3_info, c3_emlrt_marshallOut("eml_min_or_max"), "name",
                  "name", 308);
  sf_mex_addfield(*c3_info, c3_emlrt_marshallOut("char"), "dominantType",
                  "dominantType", 308);
  sf_mex_addfield(*c3_info, c3_emlrt_marshallOut(
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/eml/eml_min_or_max.m"),
                  "resolved", "resolved", 308);
  sf_mex_addfield(*c3_info, c3_b_emlrt_marshallOut(1378321184U), "fileTimeLo",
                  "fileTimeLo", 308);
  sf_mex_addfield(*c3_info, c3_b_emlrt_marshallOut(0U), "fileTimeHi",
                  "fileTimeHi", 308);
  sf_mex_addfield(*c3_info, c3_b_emlrt_marshallOut(0U), "mFileTimeLo",
                  "mFileTimeLo", 308);
  sf_mex_addfield(*c3_info, c3_b_emlrt_marshallOut(0U), "mFileTimeHi",
                  "mFileTimeHi", 308);
  sf_mex_assign(&c3_rhs308, sf_mex_createcellmatrix(0, 1), false);
  sf_mex_assign(&c3_lhs308, sf_mex_createcellmatrix(0, 1), false);
  sf_mex_addfield(*c3_info, sf_mex_duplicatearraysafe(&c3_rhs308), "rhs", "rhs",
                  308);
  sf_mex_addfield(*c3_info, sf_mex_duplicatearraysafe(&c3_lhs308), "lhs", "lhs",
                  308);
  sf_mex_addfield(*c3_info, c3_emlrt_marshallOut(
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/eml/eml_qrsolve.m"), "context",
                  "context", 309);
  sf_mex_addfield(*c3_info, c3_emlrt_marshallOut("eml_xcabs1"), "name", "name",
                  309);
  sf_mex_addfield(*c3_info, c3_emlrt_marshallOut("double"), "dominantType",
                  "dominantType", 309);
  sf_mex_addfield(*c3_info, c3_emlrt_marshallOut(
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/eml/blas/eml_xcabs1.m"),
                  "resolved", "resolved", 309);
  sf_mex_addfield(*c3_info, c3_b_emlrt_marshallOut(1376005888U), "fileTimeLo",
                  "fileTimeLo", 309);
  sf_mex_addfield(*c3_info, c3_b_emlrt_marshallOut(0U), "fileTimeHi",
                  "fileTimeHi", 309);
  sf_mex_addfield(*c3_info, c3_b_emlrt_marshallOut(0U), "mFileTimeLo",
                  "mFileTimeLo", 309);
  sf_mex_addfield(*c3_info, c3_b_emlrt_marshallOut(0U), "mFileTimeHi",
                  "mFileTimeHi", 309);
  sf_mex_assign(&c3_rhs309, sf_mex_createcellmatrix(0, 1), false);
  sf_mex_assign(&c3_lhs309, sf_mex_createcellmatrix(0, 1), false);
  sf_mex_addfield(*c3_info, sf_mex_duplicatearraysafe(&c3_rhs309), "rhs", "rhs",
                  309);
  sf_mex_addfield(*c3_info, sf_mex_duplicatearraysafe(&c3_lhs309), "lhs", "lhs",
                  309);
  sf_mex_addfield(*c3_info, c3_emlrt_marshallOut(
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/eml/blas/eml_xcabs1.m"),
                  "context", "context", 310);
  sf_mex_addfield(*c3_info, c3_emlrt_marshallOut("coder.internal.refblas.xcabs1"),
                  "name", "name", 310);
  sf_mex_addfield(*c3_info, c3_emlrt_marshallOut("double"), "dominantType",
                  "dominantType", 310);
  sf_mex_addfield(*c3_info, c3_emlrt_marshallOut(
    "[ILXE]$matlabroot$/toolbox/coder/coder/+coder/+internal/+refblas/xcabs1.p"),
                  "resolved", "resolved", 310);
  sf_mex_addfield(*c3_info, c3_b_emlrt_marshallOut(1410832972U), "fileTimeLo",
                  "fileTimeLo", 310);
  sf_mex_addfield(*c3_info, c3_b_emlrt_marshallOut(0U), "fileTimeHi",
                  "fileTimeHi", 310);
  sf_mex_addfield(*c3_info, c3_b_emlrt_marshallOut(0U), "mFileTimeLo",
                  "mFileTimeLo", 310);
  sf_mex_addfield(*c3_info, c3_b_emlrt_marshallOut(0U), "mFileTimeHi",
                  "mFileTimeHi", 310);
  sf_mex_assign(&c3_rhs310, sf_mex_createcellmatrix(0, 1), false);
  sf_mex_assign(&c3_lhs310, sf_mex_createcellmatrix(0, 1), false);
  sf_mex_addfield(*c3_info, sf_mex_duplicatearraysafe(&c3_rhs310), "rhs", "rhs",
                  310);
  sf_mex_addfield(*c3_info, sf_mex_duplicatearraysafe(&c3_lhs310), "lhs", "lhs",
                  310);
  sf_mex_addfield(*c3_info, c3_emlrt_marshallOut(
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/eml/eml_qrsolve.m"), "context",
                  "context", 311);
  sf_mex_addfield(*c3_info, c3_emlrt_marshallOut("eps"), "name", "name", 311);
  sf_mex_addfield(*c3_info, c3_emlrt_marshallOut("char"), "dominantType",
                  "dominantType", 311);
  sf_mex_addfield(*c3_info, c3_emlrt_marshallOut(
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/elmat/eps.m"), "resolved",
                  "resolved", 311);
  sf_mex_addfield(*c3_info, c3_b_emlrt_marshallOut(1326753196U), "fileTimeLo",
                  "fileTimeLo", 311);
  sf_mex_addfield(*c3_info, c3_b_emlrt_marshallOut(0U), "fileTimeHi",
                  "fileTimeHi", 311);
  sf_mex_addfield(*c3_info, c3_b_emlrt_marshallOut(0U), "mFileTimeLo",
                  "mFileTimeLo", 311);
  sf_mex_addfield(*c3_info, c3_b_emlrt_marshallOut(0U), "mFileTimeHi",
                  "mFileTimeHi", 311);
  sf_mex_assign(&c3_rhs311, sf_mex_createcellmatrix(0, 1), false);
  sf_mex_assign(&c3_lhs311, sf_mex_createcellmatrix(0, 1), false);
  sf_mex_addfield(*c3_info, sf_mex_duplicatearraysafe(&c3_rhs311), "rhs", "rhs",
                  311);
  sf_mex_addfield(*c3_info, sf_mex_duplicatearraysafe(&c3_lhs311), "lhs", "lhs",
                  311);
  sf_mex_addfield(*c3_info, c3_emlrt_marshallOut(
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/eml/eml_qrsolve.m"), "context",
                  "context", 312);
  sf_mex_addfield(*c3_info, c3_emlrt_marshallOut("eml_flt2str"), "name", "name",
                  312);
  sf_mex_addfield(*c3_info, c3_emlrt_marshallOut("double"), "dominantType",
                  "dominantType", 312);
  sf_mex_addfield(*c3_info, c3_emlrt_marshallOut(
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/eml/eml_flt2str.m"), "resolved",
                  "resolved", 312);
  sf_mex_addfield(*c3_info, c3_b_emlrt_marshallOut(1360307550U), "fileTimeLo",
                  "fileTimeLo", 312);
  sf_mex_addfield(*c3_info, c3_b_emlrt_marshallOut(0U), "fileTimeHi",
                  "fileTimeHi", 312);
  sf_mex_addfield(*c3_info, c3_b_emlrt_marshallOut(0U), "mFileTimeLo",
                  "mFileTimeLo", 312);
  sf_mex_addfield(*c3_info, c3_b_emlrt_marshallOut(0U), "mFileTimeHi",
                  "mFileTimeHi", 312);
  sf_mex_assign(&c3_rhs312, sf_mex_createcellmatrix(0, 1), false);
  sf_mex_assign(&c3_lhs312, sf_mex_createcellmatrix(0, 1), false);
  sf_mex_addfield(*c3_info, sf_mex_duplicatearraysafe(&c3_rhs312), "rhs", "rhs",
                  312);
  sf_mex_addfield(*c3_info, sf_mex_duplicatearraysafe(&c3_lhs312), "lhs", "lhs",
                  312);
  sf_mex_addfield(*c3_info, c3_emlrt_marshallOut(
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/eml/eml_qrsolve.m"), "context",
                  "context", 313);
  sf_mex_addfield(*c3_info, c3_emlrt_marshallOut("eml_warning"), "name", "name",
                  313);
  sf_mex_addfield(*c3_info, c3_emlrt_marshallOut("char"), "dominantType",
                  "dominantType", 313);
  sf_mex_addfield(*c3_info, c3_emlrt_marshallOut(
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/eml/eml_warning.m"), "resolved",
                  "resolved", 313);
  sf_mex_addfield(*c3_info, c3_b_emlrt_marshallOut(1286844002U), "fileTimeLo",
                  "fileTimeLo", 313);
  sf_mex_addfield(*c3_info, c3_b_emlrt_marshallOut(0U), "fileTimeHi",
                  "fileTimeHi", 313);
  sf_mex_addfield(*c3_info, c3_b_emlrt_marshallOut(0U), "mFileTimeLo",
                  "mFileTimeLo", 313);
  sf_mex_addfield(*c3_info, c3_b_emlrt_marshallOut(0U), "mFileTimeHi",
                  "mFileTimeHi", 313);
  sf_mex_assign(&c3_rhs313, sf_mex_createcellmatrix(0, 1), false);
  sf_mex_assign(&c3_lhs313, sf_mex_createcellmatrix(0, 1), false);
  sf_mex_addfield(*c3_info, sf_mex_duplicatearraysafe(&c3_rhs313), "rhs", "rhs",
                  313);
  sf_mex_addfield(*c3_info, sf_mex_duplicatearraysafe(&c3_lhs313), "lhs", "lhs",
                  313);
  sf_mex_addfield(*c3_info, c3_emlrt_marshallOut(
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/eml/eml_qrsolve.m"), "context",
                  "context", 314);
  sf_mex_addfield(*c3_info, c3_emlrt_marshallOut("eml_scalar_eg"), "name",
                  "name", 314);
  sf_mex_addfield(*c3_info, c3_emlrt_marshallOut("double"), "dominantType",
                  "dominantType", 314);
  sf_mex_addfield(*c3_info, c3_emlrt_marshallOut(
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/eml/eml_scalar_eg.m"), "resolved",
                  "resolved", 314);
  sf_mex_addfield(*c3_info, c3_b_emlrt_marshallOut(1376005888U), "fileTimeLo",
                  "fileTimeLo", 314);
  sf_mex_addfield(*c3_info, c3_b_emlrt_marshallOut(0U), "fileTimeHi",
                  "fileTimeHi", 314);
  sf_mex_addfield(*c3_info, c3_b_emlrt_marshallOut(0U), "mFileTimeLo",
                  "mFileTimeLo", 314);
  sf_mex_addfield(*c3_info, c3_b_emlrt_marshallOut(0U), "mFileTimeHi",
                  "mFileTimeHi", 314);
  sf_mex_assign(&c3_rhs314, sf_mex_createcellmatrix(0, 1), false);
  sf_mex_assign(&c3_lhs314, sf_mex_createcellmatrix(0, 1), false);
  sf_mex_addfield(*c3_info, sf_mex_duplicatearraysafe(&c3_rhs314), "rhs", "rhs",
                  314);
  sf_mex_addfield(*c3_info, sf_mex_duplicatearraysafe(&c3_lhs314), "lhs", "lhs",
                  314);
  sf_mex_addfield(*c3_info, c3_emlrt_marshallOut(
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/eml/eml_qrsolve.m"), "context",
                  "context", 315);
  sf_mex_addfield(*c3_info, c3_emlrt_marshallOut("coder.internal.conjtimes"),
                  "name", "name", 315);
  sf_mex_addfield(*c3_info, c3_emlrt_marshallOut("double"), "dominantType",
                  "dominantType", 315);
  sf_mex_addfield(*c3_info, c3_emlrt_marshallOut(
    "[ILXE]$matlabroot$/toolbox/shared/coder/coder/+coder/+internal/conjtimes.m"),
                  "resolved", "resolved", 315);
  sf_mex_addfield(*c3_info, c3_b_emlrt_marshallOut(1360307786U), "fileTimeLo",
                  "fileTimeLo", 315);
  sf_mex_addfield(*c3_info, c3_b_emlrt_marshallOut(0U), "fileTimeHi",
                  "fileTimeHi", 315);
  sf_mex_addfield(*c3_info, c3_b_emlrt_marshallOut(0U), "mFileTimeLo",
                  "mFileTimeLo", 315);
  sf_mex_addfield(*c3_info, c3_b_emlrt_marshallOut(0U), "mFileTimeHi",
                  "mFileTimeHi", 315);
  sf_mex_assign(&c3_rhs315, sf_mex_createcellmatrix(0, 1), false);
  sf_mex_assign(&c3_lhs315, sf_mex_createcellmatrix(0, 1), false);
  sf_mex_addfield(*c3_info, sf_mex_duplicatearraysafe(&c3_rhs315), "rhs", "rhs",
                  315);
  sf_mex_addfield(*c3_info, sf_mex_duplicatearraysafe(&c3_lhs315), "lhs", "lhs",
                  315);
  sf_mex_addfield(*c3_info, c3_emlrt_marshallOut(
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/eml/eml_qrsolve.m"), "context",
                  "context", 316);
  sf_mex_addfield(*c3_info, c3_emlrt_marshallOut("eml_div"), "name", "name", 316);
  sf_mex_addfield(*c3_info, c3_emlrt_marshallOut("double"), "dominantType",
                  "dominantType", 316);
  sf_mex_addfield(*c3_info, c3_emlrt_marshallOut(
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/eml/eml_div.m"), "resolved",
                  "resolved", 316);
  sf_mex_addfield(*c3_info, c3_b_emlrt_marshallOut(1386449152U), "fileTimeLo",
                  "fileTimeLo", 316);
  sf_mex_addfield(*c3_info, c3_b_emlrt_marshallOut(0U), "fileTimeHi",
                  "fileTimeHi", 316);
  sf_mex_addfield(*c3_info, c3_b_emlrt_marshallOut(0U), "mFileTimeLo",
                  "mFileTimeLo", 316);
  sf_mex_addfield(*c3_info, c3_b_emlrt_marshallOut(0U), "mFileTimeHi",
                  "mFileTimeHi", 316);
  sf_mex_assign(&c3_rhs316, sf_mex_createcellmatrix(0, 1), false);
  sf_mex_assign(&c3_lhs316, sf_mex_createcellmatrix(0, 1), false);
  sf_mex_addfield(*c3_info, sf_mex_duplicatearraysafe(&c3_rhs316), "rhs", "rhs",
                  316);
  sf_mex_addfield(*c3_info, sf_mex_duplicatearraysafe(&c3_lhs316), "lhs", "lhs",
                  316);
  sf_mex_addfield(*c3_info, c3_emlrt_marshallOut(""), "context", "context", 317);
  sf_mex_addfield(*c3_info, c3_emlrt_marshallOut("eye"), "name", "name", 317);
  sf_mex_addfield(*c3_info, c3_emlrt_marshallOut("double"), "dominantType",
                  "dominantType", 317);
  sf_mex_addfield(*c3_info, c3_emlrt_marshallOut(
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/elmat/eye.m"), "resolved",
                  "resolved", 317);
  sf_mex_addfield(*c3_info, c3_b_emlrt_marshallOut(1406838348U), "fileTimeLo",
                  "fileTimeLo", 317);
  sf_mex_addfield(*c3_info, c3_b_emlrt_marshallOut(0U), "fileTimeHi",
                  "fileTimeHi", 317);
  sf_mex_addfield(*c3_info, c3_b_emlrt_marshallOut(0U), "mFileTimeLo",
                  "mFileTimeLo", 317);
  sf_mex_addfield(*c3_info, c3_b_emlrt_marshallOut(0U), "mFileTimeHi",
                  "mFileTimeHi", 317);
  sf_mex_assign(&c3_rhs317, sf_mex_createcellmatrix(0, 1), false);
  sf_mex_assign(&c3_lhs317, sf_mex_createcellmatrix(0, 1), false);
  sf_mex_addfield(*c3_info, sf_mex_duplicatearraysafe(&c3_rhs317), "rhs", "rhs",
                  317);
  sf_mex_addfield(*c3_info, sf_mex_duplicatearraysafe(&c3_lhs317), "lhs", "lhs",
                  317);
  sf_mex_addfield(*c3_info, c3_emlrt_marshallOut(
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/elmat/eye.m"), "context",
                  "context", 318);
  sf_mex_addfield(*c3_info, c3_emlrt_marshallOut("eml_assert_valid_size_arg"),
                  "name", "name", 318);
  sf_mex_addfield(*c3_info, c3_emlrt_marshallOut("double"), "dominantType",
                  "dominantType", 318);
  sf_mex_addfield(*c3_info, c3_emlrt_marshallOut(
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/eml/eml_assert_valid_size_arg.m"),
                  "resolved", "resolved", 318);
  sf_mex_addfield(*c3_info, c3_b_emlrt_marshallOut(1368208230U), "fileTimeLo",
                  "fileTimeLo", 318);
  sf_mex_addfield(*c3_info, c3_b_emlrt_marshallOut(0U), "fileTimeHi",
                  "fileTimeHi", 318);
  sf_mex_addfield(*c3_info, c3_b_emlrt_marshallOut(0U), "mFileTimeLo",
                  "mFileTimeLo", 318);
  sf_mex_addfield(*c3_info, c3_b_emlrt_marshallOut(0U), "mFileTimeHi",
                  "mFileTimeHi", 318);
  sf_mex_assign(&c3_rhs318, sf_mex_createcellmatrix(0, 1), false);
  sf_mex_assign(&c3_lhs318, sf_mex_createcellmatrix(0, 1), false);
  sf_mex_addfield(*c3_info, sf_mex_duplicatearraysafe(&c3_rhs318), "rhs", "rhs",
                  318);
  sf_mex_addfield(*c3_info, sf_mex_duplicatearraysafe(&c3_lhs318), "lhs", "lhs",
                  318);
  sf_mex_addfield(*c3_info, c3_emlrt_marshallOut(
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/eml/eml_assert_valid_size_arg.m"),
                  "context", "context", 319);
  sf_mex_addfield(*c3_info, c3_emlrt_marshallOut(
    "coder.internal.isBuiltInNumeric"), "name", "name", 319);
  sf_mex_addfield(*c3_info, c3_emlrt_marshallOut("double"), "dominantType",
                  "dominantType", 319);
  sf_mex_addfield(*c3_info, c3_emlrt_marshallOut(
    "[ILXE]$matlabroot$/toolbox/shared/coder/coder/+coder/+internal/isBuiltInNumeric.m"),
                  "resolved", "resolved", 319);
  sf_mex_addfield(*c3_info, c3_b_emlrt_marshallOut(1395953456U), "fileTimeLo",
                  "fileTimeLo", 319);
  sf_mex_addfield(*c3_info, c3_b_emlrt_marshallOut(0U), "fileTimeHi",
                  "fileTimeHi", 319);
  sf_mex_addfield(*c3_info, c3_b_emlrt_marshallOut(0U), "mFileTimeLo",
                  "mFileTimeLo", 319);
  sf_mex_addfield(*c3_info, c3_b_emlrt_marshallOut(0U), "mFileTimeHi",
                  "mFileTimeHi", 319);
  sf_mex_assign(&c3_rhs319, sf_mex_createcellmatrix(0, 1), false);
  sf_mex_assign(&c3_lhs319, sf_mex_createcellmatrix(0, 1), false);
  sf_mex_addfield(*c3_info, sf_mex_duplicatearraysafe(&c3_rhs319), "rhs", "rhs",
                  319);
  sf_mex_addfield(*c3_info, sf_mex_duplicatearraysafe(&c3_lhs319), "lhs", "lhs",
                  319);
  sf_mex_destroy(&c3_rhs256);
  sf_mex_destroy(&c3_lhs256);
  sf_mex_destroy(&c3_rhs257);
  sf_mex_destroy(&c3_lhs257);
  sf_mex_destroy(&c3_rhs258);
  sf_mex_destroy(&c3_lhs258);
  sf_mex_destroy(&c3_rhs259);
  sf_mex_destroy(&c3_lhs259);
  sf_mex_destroy(&c3_rhs260);
  sf_mex_destroy(&c3_lhs260);
  sf_mex_destroy(&c3_rhs261);
  sf_mex_destroy(&c3_lhs261);
  sf_mex_destroy(&c3_rhs262);
  sf_mex_destroy(&c3_lhs262);
  sf_mex_destroy(&c3_rhs263);
  sf_mex_destroy(&c3_lhs263);
  sf_mex_destroy(&c3_rhs264);
  sf_mex_destroy(&c3_lhs264);
  sf_mex_destroy(&c3_rhs265);
  sf_mex_destroy(&c3_lhs265);
  sf_mex_destroy(&c3_rhs266);
  sf_mex_destroy(&c3_lhs266);
  sf_mex_destroy(&c3_rhs267);
  sf_mex_destroy(&c3_lhs267);
  sf_mex_destroy(&c3_rhs268);
  sf_mex_destroy(&c3_lhs268);
  sf_mex_destroy(&c3_rhs269);
  sf_mex_destroy(&c3_lhs269);
  sf_mex_destroy(&c3_rhs270);
  sf_mex_destroy(&c3_lhs270);
  sf_mex_destroy(&c3_rhs271);
  sf_mex_destroy(&c3_lhs271);
  sf_mex_destroy(&c3_rhs272);
  sf_mex_destroy(&c3_lhs272);
  sf_mex_destroy(&c3_rhs273);
  sf_mex_destroy(&c3_lhs273);
  sf_mex_destroy(&c3_rhs274);
  sf_mex_destroy(&c3_lhs274);
  sf_mex_destroy(&c3_rhs275);
  sf_mex_destroy(&c3_lhs275);
  sf_mex_destroy(&c3_rhs276);
  sf_mex_destroy(&c3_lhs276);
  sf_mex_destroy(&c3_rhs277);
  sf_mex_destroy(&c3_lhs277);
  sf_mex_destroy(&c3_rhs278);
  sf_mex_destroy(&c3_lhs278);
  sf_mex_destroy(&c3_rhs279);
  sf_mex_destroy(&c3_lhs279);
  sf_mex_destroy(&c3_rhs280);
  sf_mex_destroy(&c3_lhs280);
  sf_mex_destroy(&c3_rhs281);
  sf_mex_destroy(&c3_lhs281);
  sf_mex_destroy(&c3_rhs282);
  sf_mex_destroy(&c3_lhs282);
  sf_mex_destroy(&c3_rhs283);
  sf_mex_destroy(&c3_lhs283);
  sf_mex_destroy(&c3_rhs284);
  sf_mex_destroy(&c3_lhs284);
  sf_mex_destroy(&c3_rhs285);
  sf_mex_destroy(&c3_lhs285);
  sf_mex_destroy(&c3_rhs286);
  sf_mex_destroy(&c3_lhs286);
  sf_mex_destroy(&c3_rhs287);
  sf_mex_destroy(&c3_lhs287);
  sf_mex_destroy(&c3_rhs288);
  sf_mex_destroy(&c3_lhs288);
  sf_mex_destroy(&c3_rhs289);
  sf_mex_destroy(&c3_lhs289);
  sf_mex_destroy(&c3_rhs290);
  sf_mex_destroy(&c3_lhs290);
  sf_mex_destroy(&c3_rhs291);
  sf_mex_destroy(&c3_lhs291);
  sf_mex_destroy(&c3_rhs292);
  sf_mex_destroy(&c3_lhs292);
  sf_mex_destroy(&c3_rhs293);
  sf_mex_destroy(&c3_lhs293);
  sf_mex_destroy(&c3_rhs294);
  sf_mex_destroy(&c3_lhs294);
  sf_mex_destroy(&c3_rhs295);
  sf_mex_destroy(&c3_lhs295);
  sf_mex_destroy(&c3_rhs296);
  sf_mex_destroy(&c3_lhs296);
  sf_mex_destroy(&c3_rhs297);
  sf_mex_destroy(&c3_lhs297);
  sf_mex_destroy(&c3_rhs298);
  sf_mex_destroy(&c3_lhs298);
  sf_mex_destroy(&c3_rhs299);
  sf_mex_destroy(&c3_lhs299);
  sf_mex_destroy(&c3_rhs300);
  sf_mex_destroy(&c3_lhs300);
  sf_mex_destroy(&c3_rhs301);
  sf_mex_destroy(&c3_lhs301);
  sf_mex_destroy(&c3_rhs302);
  sf_mex_destroy(&c3_lhs302);
  sf_mex_destroy(&c3_rhs303);
  sf_mex_destroy(&c3_lhs303);
  sf_mex_destroy(&c3_rhs304);
  sf_mex_destroy(&c3_lhs304);
  sf_mex_destroy(&c3_rhs305);
  sf_mex_destroy(&c3_lhs305);
  sf_mex_destroy(&c3_rhs306);
  sf_mex_destroy(&c3_lhs306);
  sf_mex_destroy(&c3_rhs307);
  sf_mex_destroy(&c3_lhs307);
  sf_mex_destroy(&c3_rhs308);
  sf_mex_destroy(&c3_lhs308);
  sf_mex_destroy(&c3_rhs309);
  sf_mex_destroy(&c3_lhs309);
  sf_mex_destroy(&c3_rhs310);
  sf_mex_destroy(&c3_lhs310);
  sf_mex_destroy(&c3_rhs311);
  sf_mex_destroy(&c3_lhs311);
  sf_mex_destroy(&c3_rhs312);
  sf_mex_destroy(&c3_lhs312);
  sf_mex_destroy(&c3_rhs313);
  sf_mex_destroy(&c3_lhs313);
  sf_mex_destroy(&c3_rhs314);
  sf_mex_destroy(&c3_lhs314);
  sf_mex_destroy(&c3_rhs315);
  sf_mex_destroy(&c3_lhs315);
  sf_mex_destroy(&c3_rhs316);
  sf_mex_destroy(&c3_lhs316);
  sf_mex_destroy(&c3_rhs317);
  sf_mex_destroy(&c3_lhs317);
  sf_mex_destroy(&c3_rhs318);
  sf_mex_destroy(&c3_lhs318);
  sf_mex_destroy(&c3_rhs319);
  sf_mex_destroy(&c3_lhs319);
}

static void c3_f_info_helper(const mxArray **c3_info)
{
  const mxArray *c3_rhs320 = NULL;
  const mxArray *c3_lhs320 = NULL;
  const mxArray *c3_rhs321 = NULL;
  const mxArray *c3_lhs321 = NULL;
  const mxArray *c3_rhs322 = NULL;
  const mxArray *c3_lhs322 = NULL;
  const mxArray *c3_rhs323 = NULL;
  const mxArray *c3_lhs323 = NULL;
  const mxArray *c3_rhs324 = NULL;
  const mxArray *c3_lhs324 = NULL;
  const mxArray *c3_rhs325 = NULL;
  const mxArray *c3_lhs325 = NULL;
  const mxArray *c3_rhs326 = NULL;
  const mxArray *c3_lhs326 = NULL;
  const mxArray *c3_rhs327 = NULL;
  const mxArray *c3_lhs327 = NULL;
  const mxArray *c3_rhs328 = NULL;
  const mxArray *c3_lhs328 = NULL;
  const mxArray *c3_rhs329 = NULL;
  const mxArray *c3_lhs329 = NULL;
  const mxArray *c3_rhs330 = NULL;
  const mxArray *c3_lhs330 = NULL;
  const mxArray *c3_rhs331 = NULL;
  const mxArray *c3_lhs331 = NULL;
  const mxArray *c3_rhs332 = NULL;
  const mxArray *c3_lhs332 = NULL;
  const mxArray *c3_rhs333 = NULL;
  const mxArray *c3_lhs333 = NULL;
  const mxArray *c3_rhs334 = NULL;
  const mxArray *c3_lhs334 = NULL;
  const mxArray *c3_rhs335 = NULL;
  const mxArray *c3_lhs335 = NULL;
  const mxArray *c3_rhs336 = NULL;
  const mxArray *c3_lhs336 = NULL;
  const mxArray *c3_rhs337 = NULL;
  const mxArray *c3_lhs337 = NULL;
  const mxArray *c3_rhs338 = NULL;
  const mxArray *c3_lhs338 = NULL;
  const mxArray *c3_rhs339 = NULL;
  const mxArray *c3_lhs339 = NULL;
  const mxArray *c3_rhs340 = NULL;
  const mxArray *c3_lhs340 = NULL;
  const mxArray *c3_rhs341 = NULL;
  const mxArray *c3_lhs341 = NULL;
  const mxArray *c3_rhs342 = NULL;
  const mxArray *c3_lhs342 = NULL;
  const mxArray *c3_rhs343 = NULL;
  const mxArray *c3_lhs343 = NULL;
  const mxArray *c3_rhs344 = NULL;
  const mxArray *c3_lhs344 = NULL;
  const mxArray *c3_rhs345 = NULL;
  const mxArray *c3_lhs345 = NULL;
  const mxArray *c3_rhs346 = NULL;
  const mxArray *c3_lhs346 = NULL;
  const mxArray *c3_rhs347 = NULL;
  const mxArray *c3_lhs347 = NULL;
  const mxArray *c3_rhs348 = NULL;
  const mxArray *c3_lhs348 = NULL;
  const mxArray *c3_rhs349 = NULL;
  const mxArray *c3_lhs349 = NULL;
  const mxArray *c3_rhs350 = NULL;
  const mxArray *c3_lhs350 = NULL;
  const mxArray *c3_rhs351 = NULL;
  const mxArray *c3_lhs351 = NULL;
  const mxArray *c3_rhs352 = NULL;
  const mxArray *c3_lhs352 = NULL;
  const mxArray *c3_rhs353 = NULL;
  const mxArray *c3_lhs353 = NULL;
  const mxArray *c3_rhs354 = NULL;
  const mxArray *c3_lhs354 = NULL;
  const mxArray *c3_rhs355 = NULL;
  const mxArray *c3_lhs355 = NULL;
  sf_mex_addfield(*c3_info, c3_emlrt_marshallOut(
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/eml/eml_assert_valid_size_arg.m!isintegral"),
                  "context", "context", 320);
  sf_mex_addfield(*c3_info, c3_emlrt_marshallOut("isinf"), "name", "name", 320);
  sf_mex_addfield(*c3_info, c3_emlrt_marshallOut("double"), "dominantType",
                  "dominantType", 320);
  sf_mex_addfield(*c3_info, c3_emlrt_marshallOut(
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/elmat/isinf.m"), "resolved",
                  "resolved", 320);
  sf_mex_addfield(*c3_info, c3_b_emlrt_marshallOut(1363735456U), "fileTimeLo",
                  "fileTimeLo", 320);
  sf_mex_addfield(*c3_info, c3_b_emlrt_marshallOut(0U), "fileTimeHi",
                  "fileTimeHi", 320);
  sf_mex_addfield(*c3_info, c3_b_emlrt_marshallOut(0U), "mFileTimeLo",
                  "mFileTimeLo", 320);
  sf_mex_addfield(*c3_info, c3_b_emlrt_marshallOut(0U), "mFileTimeHi",
                  "mFileTimeHi", 320);
  sf_mex_assign(&c3_rhs320, sf_mex_createcellmatrix(0, 1), false);
  sf_mex_assign(&c3_lhs320, sf_mex_createcellmatrix(0, 1), false);
  sf_mex_addfield(*c3_info, sf_mex_duplicatearraysafe(&c3_rhs320), "rhs", "rhs",
                  320);
  sf_mex_addfield(*c3_info, sf_mex_duplicatearraysafe(&c3_lhs320), "lhs", "lhs",
                  320);
  sf_mex_addfield(*c3_info, c3_emlrt_marshallOut(
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/elmat/isinf.m"), "context",
                  "context", 321);
  sf_mex_addfield(*c3_info, c3_emlrt_marshallOut(
    "coder.internal.isBuiltInNumeric"), "name", "name", 321);
  sf_mex_addfield(*c3_info, c3_emlrt_marshallOut("double"), "dominantType",
                  "dominantType", 321);
  sf_mex_addfield(*c3_info, c3_emlrt_marshallOut(
    "[ILXE]$matlabroot$/toolbox/shared/coder/coder/+coder/+internal/isBuiltInNumeric.m"),
                  "resolved", "resolved", 321);
  sf_mex_addfield(*c3_info, c3_b_emlrt_marshallOut(1395953456U), "fileTimeLo",
                  "fileTimeLo", 321);
  sf_mex_addfield(*c3_info, c3_b_emlrt_marshallOut(0U), "fileTimeHi",
                  "fileTimeHi", 321);
  sf_mex_addfield(*c3_info, c3_b_emlrt_marshallOut(0U), "mFileTimeLo",
                  "mFileTimeLo", 321);
  sf_mex_addfield(*c3_info, c3_b_emlrt_marshallOut(0U), "mFileTimeHi",
                  "mFileTimeHi", 321);
  sf_mex_assign(&c3_rhs321, sf_mex_createcellmatrix(0, 1), false);
  sf_mex_assign(&c3_lhs321, sf_mex_createcellmatrix(0, 1), false);
  sf_mex_addfield(*c3_info, sf_mex_duplicatearraysafe(&c3_rhs321), "rhs", "rhs",
                  321);
  sf_mex_addfield(*c3_info, sf_mex_duplicatearraysafe(&c3_lhs321), "lhs", "lhs",
                  321);
  sf_mex_addfield(*c3_info, c3_emlrt_marshallOut(
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/eml/eml_assert_valid_size_arg.m!isinbounds"),
                  "context", "context", 322);
  sf_mex_addfield(*c3_info, c3_emlrt_marshallOut("eml_is_integer_class"), "name",
                  "name", 322);
  sf_mex_addfield(*c3_info, c3_emlrt_marshallOut("char"), "dominantType",
                  "dominantType", 322);
  sf_mex_addfield(*c3_info, c3_emlrt_marshallOut(
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/eml/eml_is_integer_class.m"),
                  "resolved", "resolved", 322);
  sf_mex_addfield(*c3_info, c3_b_emlrt_marshallOut(1286843982U), "fileTimeLo",
                  "fileTimeLo", 322);
  sf_mex_addfield(*c3_info, c3_b_emlrt_marshallOut(0U), "fileTimeHi",
                  "fileTimeHi", 322);
  sf_mex_addfield(*c3_info, c3_b_emlrt_marshallOut(0U), "mFileTimeLo",
                  "mFileTimeLo", 322);
  sf_mex_addfield(*c3_info, c3_b_emlrt_marshallOut(0U), "mFileTimeHi",
                  "mFileTimeHi", 322);
  sf_mex_assign(&c3_rhs322, sf_mex_createcellmatrix(0, 1), false);
  sf_mex_assign(&c3_lhs322, sf_mex_createcellmatrix(0, 1), false);
  sf_mex_addfield(*c3_info, sf_mex_duplicatearraysafe(&c3_rhs322), "rhs", "rhs",
                  322);
  sf_mex_addfield(*c3_info, sf_mex_duplicatearraysafe(&c3_lhs322), "lhs", "lhs",
                  322);
  sf_mex_addfield(*c3_info, c3_emlrt_marshallOut(
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/eml/eml_assert_valid_size_arg.m!isinbounds"),
                  "context", "context", 323);
  sf_mex_addfield(*c3_info, c3_emlrt_marshallOut("intmax"), "name", "name", 323);
  sf_mex_addfield(*c3_info, c3_emlrt_marshallOut("char"), "dominantType",
                  "dominantType", 323);
  sf_mex_addfield(*c3_info, c3_emlrt_marshallOut(
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/elmat/intmax.m"), "resolved",
                  "resolved", 323);
  sf_mex_addfield(*c3_info, c3_b_emlrt_marshallOut(1362287082U), "fileTimeLo",
                  "fileTimeLo", 323);
  sf_mex_addfield(*c3_info, c3_b_emlrt_marshallOut(0U), "fileTimeHi",
                  "fileTimeHi", 323);
  sf_mex_addfield(*c3_info, c3_b_emlrt_marshallOut(0U), "mFileTimeLo",
                  "mFileTimeLo", 323);
  sf_mex_addfield(*c3_info, c3_b_emlrt_marshallOut(0U), "mFileTimeHi",
                  "mFileTimeHi", 323);
  sf_mex_assign(&c3_rhs323, sf_mex_createcellmatrix(0, 1), false);
  sf_mex_assign(&c3_lhs323, sf_mex_createcellmatrix(0, 1), false);
  sf_mex_addfield(*c3_info, sf_mex_duplicatearraysafe(&c3_rhs323), "rhs", "rhs",
                  323);
  sf_mex_addfield(*c3_info, sf_mex_duplicatearraysafe(&c3_lhs323), "lhs", "lhs",
                  323);
  sf_mex_addfield(*c3_info, c3_emlrt_marshallOut(
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/eml/eml_assert_valid_size_arg.m!isinbounds"),
                  "context", "context", 324);
  sf_mex_addfield(*c3_info, c3_emlrt_marshallOut("intmin"), "name", "name", 324);
  sf_mex_addfield(*c3_info, c3_emlrt_marshallOut("char"), "dominantType",
                  "dominantType", 324);
  sf_mex_addfield(*c3_info, c3_emlrt_marshallOut(
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/elmat/intmin.m"), "resolved",
                  "resolved", 324);
  sf_mex_addfield(*c3_info, c3_b_emlrt_marshallOut(1362287082U), "fileTimeLo",
                  "fileTimeLo", 324);
  sf_mex_addfield(*c3_info, c3_b_emlrt_marshallOut(0U), "fileTimeHi",
                  "fileTimeHi", 324);
  sf_mex_addfield(*c3_info, c3_b_emlrt_marshallOut(0U), "mFileTimeLo",
                  "mFileTimeLo", 324);
  sf_mex_addfield(*c3_info, c3_b_emlrt_marshallOut(0U), "mFileTimeHi",
                  "mFileTimeHi", 324);
  sf_mex_assign(&c3_rhs324, sf_mex_createcellmatrix(0, 1), false);
  sf_mex_assign(&c3_lhs324, sf_mex_createcellmatrix(0, 1), false);
  sf_mex_addfield(*c3_info, sf_mex_duplicatearraysafe(&c3_rhs324), "rhs", "rhs",
                  324);
  sf_mex_addfield(*c3_info, sf_mex_duplicatearraysafe(&c3_lhs324), "lhs", "lhs",
                  324);
  sf_mex_addfield(*c3_info, c3_emlrt_marshallOut(
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/eml/eml_assert_valid_size_arg.m!isinbounds"),
                  "context", "context", 325);
  sf_mex_addfield(*c3_info, c3_emlrt_marshallOut("coder.internal.indexIntRelop"),
                  "name", "name", 325);
  sf_mex_addfield(*c3_info, c3_emlrt_marshallOut(""), "dominantType",
                  "dominantType", 325);
  sf_mex_addfield(*c3_info, c3_emlrt_marshallOut(
    "[ILXE]$matlabroot$/toolbox/shared/coder/coder/+coder/+internal/indexIntRelop.m"),
                  "resolved", "resolved", 325);
  sf_mex_addfield(*c3_info, c3_b_emlrt_marshallOut(1326753522U), "fileTimeLo",
                  "fileTimeLo", 325);
  sf_mex_addfield(*c3_info, c3_b_emlrt_marshallOut(0U), "fileTimeHi",
                  "fileTimeHi", 325);
  sf_mex_addfield(*c3_info, c3_b_emlrt_marshallOut(0U), "mFileTimeLo",
                  "mFileTimeLo", 325);
  sf_mex_addfield(*c3_info, c3_b_emlrt_marshallOut(0U), "mFileTimeHi",
                  "mFileTimeHi", 325);
  sf_mex_assign(&c3_rhs325, sf_mex_createcellmatrix(0, 1), false);
  sf_mex_assign(&c3_lhs325, sf_mex_createcellmatrix(0, 1), false);
  sf_mex_addfield(*c3_info, sf_mex_duplicatearraysafe(&c3_rhs325), "rhs", "rhs",
                  325);
  sf_mex_addfield(*c3_info, sf_mex_duplicatearraysafe(&c3_lhs325), "lhs", "lhs",
                  325);
  sf_mex_addfield(*c3_info, c3_emlrt_marshallOut(
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/eml/eml_assert_valid_size_arg.m"),
                  "context", "context", 326);
  sf_mex_addfield(*c3_info, c3_emlrt_marshallOut("eml_index_class"), "name",
                  "name", 326);
  sf_mex_addfield(*c3_info, c3_emlrt_marshallOut(""), "dominantType",
                  "dominantType", 326);
  sf_mex_addfield(*c3_info, c3_emlrt_marshallOut(
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/eml/eml_index_class.m"),
                  "resolved", "resolved", 326);
  sf_mex_addfield(*c3_info, c3_b_emlrt_marshallOut(1323195778U), "fileTimeLo",
                  "fileTimeLo", 326);
  sf_mex_addfield(*c3_info, c3_b_emlrt_marshallOut(0U), "fileTimeHi",
                  "fileTimeHi", 326);
  sf_mex_addfield(*c3_info, c3_b_emlrt_marshallOut(0U), "mFileTimeLo",
                  "mFileTimeLo", 326);
  sf_mex_addfield(*c3_info, c3_b_emlrt_marshallOut(0U), "mFileTimeHi",
                  "mFileTimeHi", 326);
  sf_mex_assign(&c3_rhs326, sf_mex_createcellmatrix(0, 1), false);
  sf_mex_assign(&c3_lhs326, sf_mex_createcellmatrix(0, 1), false);
  sf_mex_addfield(*c3_info, sf_mex_duplicatearraysafe(&c3_rhs326), "rhs", "rhs",
                  326);
  sf_mex_addfield(*c3_info, sf_mex_duplicatearraysafe(&c3_lhs326), "lhs", "lhs",
                  326);
  sf_mex_addfield(*c3_info, c3_emlrt_marshallOut(
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/eml/eml_assert_valid_size_arg.m"),
                  "context", "context", 327);
  sf_mex_addfield(*c3_info, c3_emlrt_marshallOut("intmax"), "name", "name", 327);
  sf_mex_addfield(*c3_info, c3_emlrt_marshallOut("char"), "dominantType",
                  "dominantType", 327);
  sf_mex_addfield(*c3_info, c3_emlrt_marshallOut(
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/elmat/intmax.m"), "resolved",
                  "resolved", 327);
  sf_mex_addfield(*c3_info, c3_b_emlrt_marshallOut(1362287082U), "fileTimeLo",
                  "fileTimeLo", 327);
  sf_mex_addfield(*c3_info, c3_b_emlrt_marshallOut(0U), "fileTimeHi",
                  "fileTimeHi", 327);
  sf_mex_addfield(*c3_info, c3_b_emlrt_marshallOut(0U), "mFileTimeLo",
                  "mFileTimeLo", 327);
  sf_mex_addfield(*c3_info, c3_b_emlrt_marshallOut(0U), "mFileTimeHi",
                  "mFileTimeHi", 327);
  sf_mex_assign(&c3_rhs327, sf_mex_createcellmatrix(0, 1), false);
  sf_mex_assign(&c3_lhs327, sf_mex_createcellmatrix(0, 1), false);
  sf_mex_addfield(*c3_info, sf_mex_duplicatearraysafe(&c3_rhs327), "rhs", "rhs",
                  327);
  sf_mex_addfield(*c3_info, sf_mex_duplicatearraysafe(&c3_lhs327), "lhs", "lhs",
                  327);
  sf_mex_addfield(*c3_info, c3_emlrt_marshallOut(
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/elmat/eye.m"), "context",
                  "context", 328);
  sf_mex_addfield(*c3_info, c3_emlrt_marshallOut(
    "eml_int_forloop_overflow_check"), "name", "name", 328);
  sf_mex_addfield(*c3_info, c3_emlrt_marshallOut(""), "dominantType",
                  "dominantType", 328);
  sf_mex_addfield(*c3_info, c3_emlrt_marshallOut(
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/eml/eml_int_forloop_overflow_check.m"),
                  "resolved", "resolved", 328);
  sf_mex_addfield(*c3_info, c3_b_emlrt_marshallOut(1397282622U), "fileTimeLo",
                  "fileTimeLo", 328);
  sf_mex_addfield(*c3_info, c3_b_emlrt_marshallOut(0U), "fileTimeHi",
                  "fileTimeHi", 328);
  sf_mex_addfield(*c3_info, c3_b_emlrt_marshallOut(0U), "mFileTimeLo",
                  "mFileTimeLo", 328);
  sf_mex_addfield(*c3_info, c3_b_emlrt_marshallOut(0U), "mFileTimeHi",
                  "mFileTimeHi", 328);
  sf_mex_assign(&c3_rhs328, sf_mex_createcellmatrix(0, 1), false);
  sf_mex_assign(&c3_lhs328, sf_mex_createcellmatrix(0, 1), false);
  sf_mex_addfield(*c3_info, sf_mex_duplicatearraysafe(&c3_rhs328), "rhs", "rhs",
                  328);
  sf_mex_addfield(*c3_info, sf_mex_duplicatearraysafe(&c3_lhs328), "lhs", "lhs",
                  328);
  sf_mex_addfield(*c3_info, c3_emlrt_marshallOut(""), "context", "context", 329);
  sf_mex_addfield(*c3_info, c3_emlrt_marshallOut("reshape"), "name", "name", 329);
  sf_mex_addfield(*c3_info, c3_emlrt_marshallOut("double"), "dominantType",
                  "dominantType", 329);
  sf_mex_addfield(*c3_info, c3_emlrt_marshallOut(
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/elmat/reshape.m"), "resolved",
                  "resolved", 329);
  sf_mex_addfield(*c3_info, c3_b_emlrt_marshallOut(1378321182U), "fileTimeLo",
                  "fileTimeLo", 329);
  sf_mex_addfield(*c3_info, c3_b_emlrt_marshallOut(0U), "fileTimeHi",
                  "fileTimeHi", 329);
  sf_mex_addfield(*c3_info, c3_b_emlrt_marshallOut(0U), "mFileTimeLo",
                  "mFileTimeLo", 329);
  sf_mex_addfield(*c3_info, c3_b_emlrt_marshallOut(0U), "mFileTimeHi",
                  "mFileTimeHi", 329);
  sf_mex_assign(&c3_rhs329, sf_mex_createcellmatrix(0, 1), false);
  sf_mex_assign(&c3_lhs329, sf_mex_createcellmatrix(0, 1), false);
  sf_mex_addfield(*c3_info, sf_mex_duplicatearraysafe(&c3_rhs329), "rhs", "rhs",
                  329);
  sf_mex_addfield(*c3_info, sf_mex_duplicatearraysafe(&c3_lhs329), "lhs", "lhs",
                  329);
  sf_mex_addfield(*c3_info, c3_emlrt_marshallOut(
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/elmat/reshape.m"), "context",
                  "context", 330);
  sf_mex_addfield(*c3_info, c3_emlrt_marshallOut("eml_index_class"), "name",
                  "name", 330);
  sf_mex_addfield(*c3_info, c3_emlrt_marshallOut(""), "dominantType",
                  "dominantType", 330);
  sf_mex_addfield(*c3_info, c3_emlrt_marshallOut(
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/eml/eml_index_class.m"),
                  "resolved", "resolved", 330);
  sf_mex_addfield(*c3_info, c3_b_emlrt_marshallOut(1323195778U), "fileTimeLo",
                  "fileTimeLo", 330);
  sf_mex_addfield(*c3_info, c3_b_emlrt_marshallOut(0U), "fileTimeHi",
                  "fileTimeHi", 330);
  sf_mex_addfield(*c3_info, c3_b_emlrt_marshallOut(0U), "mFileTimeLo",
                  "mFileTimeLo", 330);
  sf_mex_addfield(*c3_info, c3_b_emlrt_marshallOut(0U), "mFileTimeHi",
                  "mFileTimeHi", 330);
  sf_mex_assign(&c3_rhs330, sf_mex_createcellmatrix(0, 1), false);
  sf_mex_assign(&c3_lhs330, sf_mex_createcellmatrix(0, 1), false);
  sf_mex_addfield(*c3_info, sf_mex_duplicatearraysafe(&c3_rhs330), "rhs", "rhs",
                  330);
  sf_mex_addfield(*c3_info, sf_mex_duplicatearraysafe(&c3_lhs330), "lhs", "lhs",
                  330);
  sf_mex_addfield(*c3_info, c3_emlrt_marshallOut(
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/elmat/reshape.m!reshape_varargin_to_size"),
                  "context", "context", 331);
  sf_mex_addfield(*c3_info, c3_emlrt_marshallOut("eml_index_class"), "name",
                  "name", 331);
  sf_mex_addfield(*c3_info, c3_emlrt_marshallOut(""), "dominantType",
                  "dominantType", 331);
  sf_mex_addfield(*c3_info, c3_emlrt_marshallOut(
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/eml/eml_index_class.m"),
                  "resolved", "resolved", 331);
  sf_mex_addfield(*c3_info, c3_b_emlrt_marshallOut(1323195778U), "fileTimeLo",
                  "fileTimeLo", 331);
  sf_mex_addfield(*c3_info, c3_b_emlrt_marshallOut(0U), "fileTimeHi",
                  "fileTimeHi", 331);
  sf_mex_addfield(*c3_info, c3_b_emlrt_marshallOut(0U), "mFileTimeLo",
                  "mFileTimeLo", 331);
  sf_mex_addfield(*c3_info, c3_b_emlrt_marshallOut(0U), "mFileTimeHi",
                  "mFileTimeHi", 331);
  sf_mex_assign(&c3_rhs331, sf_mex_createcellmatrix(0, 1), false);
  sf_mex_assign(&c3_lhs331, sf_mex_createcellmatrix(0, 1), false);
  sf_mex_addfield(*c3_info, sf_mex_duplicatearraysafe(&c3_rhs331), "rhs", "rhs",
                  331);
  sf_mex_addfield(*c3_info, sf_mex_duplicatearraysafe(&c3_lhs331), "lhs", "lhs",
                  331);
  sf_mex_addfield(*c3_info, c3_emlrt_marshallOut(
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/elmat/reshape.m!varargin_nempty"),
                  "context", "context", 332);
  sf_mex_addfield(*c3_info, c3_emlrt_marshallOut("eml_index_class"), "name",
                  "name", 332);
  sf_mex_addfield(*c3_info, c3_emlrt_marshallOut(""), "dominantType",
                  "dominantType", 332);
  sf_mex_addfield(*c3_info, c3_emlrt_marshallOut(
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/eml/eml_index_class.m"),
                  "resolved", "resolved", 332);
  sf_mex_addfield(*c3_info, c3_b_emlrt_marshallOut(1323195778U), "fileTimeLo",
                  "fileTimeLo", 332);
  sf_mex_addfield(*c3_info, c3_b_emlrt_marshallOut(0U), "fileTimeHi",
                  "fileTimeHi", 332);
  sf_mex_addfield(*c3_info, c3_b_emlrt_marshallOut(0U), "mFileTimeLo",
                  "mFileTimeLo", 332);
  sf_mex_addfield(*c3_info, c3_b_emlrt_marshallOut(0U), "mFileTimeHi",
                  "mFileTimeHi", 332);
  sf_mex_assign(&c3_rhs332, sf_mex_createcellmatrix(0, 1), false);
  sf_mex_assign(&c3_lhs332, sf_mex_createcellmatrix(0, 1), false);
  sf_mex_addfield(*c3_info, sf_mex_duplicatearraysafe(&c3_rhs332), "rhs", "rhs",
                  332);
  sf_mex_addfield(*c3_info, sf_mex_duplicatearraysafe(&c3_lhs332), "lhs", "lhs",
                  332);
  sf_mex_addfield(*c3_info, c3_emlrt_marshallOut(
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/elmat/reshape.m!varargin_nempty"),
                  "context", "context", 333);
  sf_mex_addfield(*c3_info, c3_emlrt_marshallOut(
    "eml_int_forloop_overflow_check"), "name", "name", 333);
  sf_mex_addfield(*c3_info, c3_emlrt_marshallOut(""), "dominantType",
                  "dominantType", 333);
  sf_mex_addfield(*c3_info, c3_emlrt_marshallOut(
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/eml/eml_int_forloop_overflow_check.m"),
                  "resolved", "resolved", 333);
  sf_mex_addfield(*c3_info, c3_b_emlrt_marshallOut(1397282622U), "fileTimeLo",
                  "fileTimeLo", 333);
  sf_mex_addfield(*c3_info, c3_b_emlrt_marshallOut(0U), "fileTimeHi",
                  "fileTimeHi", 333);
  sf_mex_addfield(*c3_info, c3_b_emlrt_marshallOut(0U), "mFileTimeLo",
                  "mFileTimeLo", 333);
  sf_mex_addfield(*c3_info, c3_b_emlrt_marshallOut(0U), "mFileTimeHi",
                  "mFileTimeHi", 333);
  sf_mex_assign(&c3_rhs333, sf_mex_createcellmatrix(0, 1), false);
  sf_mex_assign(&c3_lhs333, sf_mex_createcellmatrix(0, 1), false);
  sf_mex_addfield(*c3_info, sf_mex_duplicatearraysafe(&c3_rhs333), "rhs", "rhs",
                  333);
  sf_mex_addfield(*c3_info, sf_mex_duplicatearraysafe(&c3_lhs333), "lhs", "lhs",
                  333);
  sf_mex_addfield(*c3_info, c3_emlrt_marshallOut(
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/elmat/reshape.m!computeDimsData"),
                  "context", "context", 334);
  sf_mex_addfield(*c3_info, c3_emlrt_marshallOut("eml_index_class"), "name",
                  "name", 334);
  sf_mex_addfield(*c3_info, c3_emlrt_marshallOut(""), "dominantType",
                  "dominantType", 334);
  sf_mex_addfield(*c3_info, c3_emlrt_marshallOut(
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/eml/eml_index_class.m"),
                  "resolved", "resolved", 334);
  sf_mex_addfield(*c3_info, c3_b_emlrt_marshallOut(1323195778U), "fileTimeLo",
                  "fileTimeLo", 334);
  sf_mex_addfield(*c3_info, c3_b_emlrt_marshallOut(0U), "fileTimeHi",
                  "fileTimeHi", 334);
  sf_mex_addfield(*c3_info, c3_b_emlrt_marshallOut(0U), "mFileTimeLo",
                  "mFileTimeLo", 334);
  sf_mex_addfield(*c3_info, c3_b_emlrt_marshallOut(0U), "mFileTimeHi",
                  "mFileTimeHi", 334);
  sf_mex_assign(&c3_rhs334, sf_mex_createcellmatrix(0, 1), false);
  sf_mex_assign(&c3_lhs334, sf_mex_createcellmatrix(0, 1), false);
  sf_mex_addfield(*c3_info, sf_mex_duplicatearraysafe(&c3_rhs334), "rhs", "rhs",
                  334);
  sf_mex_addfield(*c3_info, sf_mex_duplicatearraysafe(&c3_lhs334), "lhs", "lhs",
                  334);
  sf_mex_addfield(*c3_info, c3_emlrt_marshallOut(
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/elmat/reshape.m!computeDimsData"),
                  "context", "context", 335);
  sf_mex_addfield(*c3_info, c3_emlrt_marshallOut(
    "eml_int_forloop_overflow_check"), "name", "name", 335);
  sf_mex_addfield(*c3_info, c3_emlrt_marshallOut(""), "dominantType",
                  "dominantType", 335);
  sf_mex_addfield(*c3_info, c3_emlrt_marshallOut(
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/eml/eml_int_forloop_overflow_check.m"),
                  "resolved", "resolved", 335);
  sf_mex_addfield(*c3_info, c3_b_emlrt_marshallOut(1397282622U), "fileTimeLo",
                  "fileTimeLo", 335);
  sf_mex_addfield(*c3_info, c3_b_emlrt_marshallOut(0U), "fileTimeHi",
                  "fileTimeHi", 335);
  sf_mex_addfield(*c3_info, c3_b_emlrt_marshallOut(0U), "mFileTimeLo",
                  "mFileTimeLo", 335);
  sf_mex_addfield(*c3_info, c3_b_emlrt_marshallOut(0U), "mFileTimeHi",
                  "mFileTimeHi", 335);
  sf_mex_assign(&c3_rhs335, sf_mex_createcellmatrix(0, 1), false);
  sf_mex_assign(&c3_lhs335, sf_mex_createcellmatrix(0, 1), false);
  sf_mex_addfield(*c3_info, sf_mex_duplicatearraysafe(&c3_rhs335), "rhs", "rhs",
                  335);
  sf_mex_addfield(*c3_info, sf_mex_duplicatearraysafe(&c3_lhs335), "lhs", "lhs",
                  335);
  sf_mex_addfield(*c3_info, c3_emlrt_marshallOut(
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/elmat/reshape.m!computeDimsData"),
                  "context", "context", 336);
  sf_mex_addfield(*c3_info, c3_emlrt_marshallOut("eml_assert_valid_size_arg"),
                  "name", "name", 336);
  sf_mex_addfield(*c3_info, c3_emlrt_marshallOut("double"), "dominantType",
                  "dominantType", 336);
  sf_mex_addfield(*c3_info, c3_emlrt_marshallOut(
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/eml/eml_assert_valid_size_arg.m"),
                  "resolved", "resolved", 336);
  sf_mex_addfield(*c3_info, c3_b_emlrt_marshallOut(1368208230U), "fileTimeLo",
                  "fileTimeLo", 336);
  sf_mex_addfield(*c3_info, c3_b_emlrt_marshallOut(0U), "fileTimeHi",
                  "fileTimeHi", 336);
  sf_mex_addfield(*c3_info, c3_b_emlrt_marshallOut(0U), "mFileTimeLo",
                  "mFileTimeLo", 336);
  sf_mex_addfield(*c3_info, c3_b_emlrt_marshallOut(0U), "mFileTimeHi",
                  "mFileTimeHi", 336);
  sf_mex_assign(&c3_rhs336, sf_mex_createcellmatrix(0, 1), false);
  sf_mex_assign(&c3_lhs336, sf_mex_createcellmatrix(0, 1), false);
  sf_mex_addfield(*c3_info, sf_mex_duplicatearraysafe(&c3_rhs336), "rhs", "rhs",
                  336);
  sf_mex_addfield(*c3_info, sf_mex_duplicatearraysafe(&c3_lhs336), "lhs", "lhs",
                  336);
  sf_mex_addfield(*c3_info, c3_emlrt_marshallOut(
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/elmat/reshape.m!computeDimsData"),
                  "context", "context", 337);
  sf_mex_addfield(*c3_info, c3_emlrt_marshallOut("coder.internal.indexTimes"),
                  "name", "name", 337);
  sf_mex_addfield(*c3_info, c3_emlrt_marshallOut("double"), "dominantType",
                  "dominantType", 337);
  sf_mex_addfield(*c3_info, c3_emlrt_marshallOut(
    "[ILXE]$matlabroot$/toolbox/shared/coder/coder/+coder/+internal/indexTimes.m"),
                  "resolved", "resolved", 337);
  sf_mex_addfield(*c3_info, c3_b_emlrt_marshallOut(1372608360U), "fileTimeLo",
                  "fileTimeLo", 337);
  sf_mex_addfield(*c3_info, c3_b_emlrt_marshallOut(0U), "fileTimeHi",
                  "fileTimeHi", 337);
  sf_mex_addfield(*c3_info, c3_b_emlrt_marshallOut(0U), "mFileTimeLo",
                  "mFileTimeLo", 337);
  sf_mex_addfield(*c3_info, c3_b_emlrt_marshallOut(0U), "mFileTimeHi",
                  "mFileTimeHi", 337);
  sf_mex_assign(&c3_rhs337, sf_mex_createcellmatrix(0, 1), false);
  sf_mex_assign(&c3_lhs337, sf_mex_createcellmatrix(0, 1), false);
  sf_mex_addfield(*c3_info, sf_mex_duplicatearraysafe(&c3_rhs337), "rhs", "rhs",
                  337);
  sf_mex_addfield(*c3_info, sf_mex_duplicatearraysafe(&c3_lhs337), "lhs", "lhs",
                  337);
  sf_mex_addfield(*c3_info, c3_emlrt_marshallOut(
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/elmat/reshape.m!reshape_varargin_to_size"),
                  "context", "context", 338);
  sf_mex_addfield(*c3_info, c3_emlrt_marshallOut(
    "eml_int_forloop_overflow_check"), "name", "name", 338);
  sf_mex_addfield(*c3_info, c3_emlrt_marshallOut(""), "dominantType",
                  "dominantType", 338);
  sf_mex_addfield(*c3_info, c3_emlrt_marshallOut(
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/eml/eml_int_forloop_overflow_check.m"),
                  "resolved", "resolved", 338);
  sf_mex_addfield(*c3_info, c3_b_emlrt_marshallOut(1397282622U), "fileTimeLo",
                  "fileTimeLo", 338);
  sf_mex_addfield(*c3_info, c3_b_emlrt_marshallOut(0U), "fileTimeHi",
                  "fileTimeHi", 338);
  sf_mex_addfield(*c3_info, c3_b_emlrt_marshallOut(0U), "mFileTimeLo",
                  "mFileTimeLo", 338);
  sf_mex_addfield(*c3_info, c3_b_emlrt_marshallOut(0U), "mFileTimeHi",
                  "mFileTimeHi", 338);
  sf_mex_assign(&c3_rhs338, sf_mex_createcellmatrix(0, 1), false);
  sf_mex_assign(&c3_lhs338, sf_mex_createcellmatrix(0, 1), false);
  sf_mex_addfield(*c3_info, sf_mex_duplicatearraysafe(&c3_rhs338), "rhs", "rhs",
                  338);
  sf_mex_addfield(*c3_info, sf_mex_duplicatearraysafe(&c3_lhs338), "lhs", "lhs",
                  338);
  sf_mex_addfield(*c3_info, c3_emlrt_marshallOut(
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/elmat/reshape.m"), "context",
                  "context", 339);
  sf_mex_addfield(*c3_info, c3_emlrt_marshallOut("eml_scalar_eg"), "name",
                  "name", 339);
  sf_mex_addfield(*c3_info, c3_emlrt_marshallOut("double"), "dominantType",
                  "dominantType", 339);
  sf_mex_addfield(*c3_info, c3_emlrt_marshallOut(
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/eml/eml_scalar_eg.m"), "resolved",
                  "resolved", 339);
  sf_mex_addfield(*c3_info, c3_b_emlrt_marshallOut(1376005888U), "fileTimeLo",
                  "fileTimeLo", 339);
  sf_mex_addfield(*c3_info, c3_b_emlrt_marshallOut(0U), "fileTimeHi",
                  "fileTimeHi", 339);
  sf_mex_addfield(*c3_info, c3_b_emlrt_marshallOut(0U), "mFileTimeLo",
                  "mFileTimeLo", 339);
  sf_mex_addfield(*c3_info, c3_b_emlrt_marshallOut(0U), "mFileTimeHi",
                  "mFileTimeHi", 339);
  sf_mex_assign(&c3_rhs339, sf_mex_createcellmatrix(0, 1), false);
  sf_mex_assign(&c3_lhs339, sf_mex_createcellmatrix(0, 1), false);
  sf_mex_addfield(*c3_info, sf_mex_duplicatearraysafe(&c3_rhs339), "rhs", "rhs",
                  339);
  sf_mex_addfield(*c3_info, sf_mex_duplicatearraysafe(&c3_lhs339), "lhs", "lhs",
                  339);
  sf_mex_addfield(*c3_info, c3_emlrt_marshallOut(
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/elmat/reshape.m"), "context",
                  "context", 340);
  sf_mex_addfield(*c3_info, c3_emlrt_marshallOut(
    "eml_int_forloop_overflow_check"), "name", "name", 340);
  sf_mex_addfield(*c3_info, c3_emlrt_marshallOut(""), "dominantType",
                  "dominantType", 340);
  sf_mex_addfield(*c3_info, c3_emlrt_marshallOut(
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/eml/eml_int_forloop_overflow_check.m"),
                  "resolved", "resolved", 340);
  sf_mex_addfield(*c3_info, c3_b_emlrt_marshallOut(1397282622U), "fileTimeLo",
                  "fileTimeLo", 340);
  sf_mex_addfield(*c3_info, c3_b_emlrt_marshallOut(0U), "fileTimeHi",
                  "fileTimeHi", 340);
  sf_mex_addfield(*c3_info, c3_b_emlrt_marshallOut(0U), "mFileTimeLo",
                  "mFileTimeLo", 340);
  sf_mex_addfield(*c3_info, c3_b_emlrt_marshallOut(0U), "mFileTimeHi",
                  "mFileTimeHi", 340);
  sf_mex_assign(&c3_rhs340, sf_mex_createcellmatrix(0, 1), false);
  sf_mex_assign(&c3_lhs340, sf_mex_createcellmatrix(0, 1), false);
  sf_mex_addfield(*c3_info, sf_mex_duplicatearraysafe(&c3_rhs340), "rhs", "rhs",
                  340);
  sf_mex_addfield(*c3_info, sf_mex_duplicatearraysafe(&c3_lhs340), "lhs", "lhs",
                  340);
  sf_mex_addfield(*c3_info, c3_emlrt_marshallOut(""), "context", "context", 341);
  sf_mex_addfield(*c3_info, c3_emlrt_marshallOut("get3DRotationAngles"), "name",
                  "name", 341);
  sf_mex_addfield(*c3_info, c3_emlrt_marshallOut("double"), "dominantType",
                  "dominantType", 341);
  sf_mex_addfield(*c3_info, c3_emlrt_marshallOut(
    "[E]C:/Users/sd0016/Dropbox/UAH Doktora/Journal Tracking/Matlab Code/3DOF_Experiments/get3DRotationAngles.m"),
                  "resolved", "resolved", 341);
  sf_mex_addfield(*c3_info, c3_b_emlrt_marshallOut(1428460798U), "fileTimeLo",
                  "fileTimeLo", 341);
  sf_mex_addfield(*c3_info, c3_b_emlrt_marshallOut(0U), "fileTimeHi",
                  "fileTimeHi", 341);
  sf_mex_addfield(*c3_info, c3_b_emlrt_marshallOut(0U), "mFileTimeLo",
                  "mFileTimeLo", 341);
  sf_mex_addfield(*c3_info, c3_b_emlrt_marshallOut(0U), "mFileTimeHi",
                  "mFileTimeHi", 341);
  sf_mex_assign(&c3_rhs341, sf_mex_createcellmatrix(0, 1), false);
  sf_mex_assign(&c3_lhs341, sf_mex_createcellmatrix(0, 1), false);
  sf_mex_addfield(*c3_info, sf_mex_duplicatearraysafe(&c3_rhs341), "rhs", "rhs",
                  341);
  sf_mex_addfield(*c3_info, sf_mex_duplicatearraysafe(&c3_lhs341), "lhs", "lhs",
                  341);
  sf_mex_addfield(*c3_info, c3_emlrt_marshallOut(
    "[E]C:/Users/sd0016/Dropbox/UAH Doktora/Journal Tracking/Matlab Code/3DOF_Experiments/get3DRotationAngles.m"),
                  "context", "context", 342);
  sf_mex_addfield(*c3_info, c3_emlrt_marshallOut("mpower"), "name", "name", 342);
  sf_mex_addfield(*c3_info, c3_emlrt_marshallOut("double"), "dominantType",
                  "dominantType", 342);
  sf_mex_addfield(*c3_info, c3_emlrt_marshallOut(
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/ops/mpower.m"), "resolved",
                  "resolved", 342);
  sf_mex_addfield(*c3_info, c3_b_emlrt_marshallOut(1363735478U), "fileTimeLo",
                  "fileTimeLo", 342);
  sf_mex_addfield(*c3_info, c3_b_emlrt_marshallOut(0U), "fileTimeHi",
                  "fileTimeHi", 342);
  sf_mex_addfield(*c3_info, c3_b_emlrt_marshallOut(0U), "mFileTimeLo",
                  "mFileTimeLo", 342);
  sf_mex_addfield(*c3_info, c3_b_emlrt_marshallOut(0U), "mFileTimeHi",
                  "mFileTimeHi", 342);
  sf_mex_assign(&c3_rhs342, sf_mex_createcellmatrix(0, 1), false);
  sf_mex_assign(&c3_lhs342, sf_mex_createcellmatrix(0, 1), false);
  sf_mex_addfield(*c3_info, sf_mex_duplicatearraysafe(&c3_rhs342), "rhs", "rhs",
                  342);
  sf_mex_addfield(*c3_info, sf_mex_duplicatearraysafe(&c3_lhs342), "lhs", "lhs",
                  342);
  sf_mex_addfield(*c3_info, c3_emlrt_marshallOut(
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/ops/mpower.m"), "context",
                  "context", 343);
  sf_mex_addfield(*c3_info, c3_emlrt_marshallOut("power"), "name", "name", 343);
  sf_mex_addfield(*c3_info, c3_emlrt_marshallOut("double"), "dominantType",
                  "dominantType", 343);
  sf_mex_addfield(*c3_info, c3_emlrt_marshallOut(
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/ops/power.m"), "resolved",
                  "resolved", 343);
  sf_mex_addfield(*c3_info, c3_b_emlrt_marshallOut(1395350106U), "fileTimeLo",
                  "fileTimeLo", 343);
  sf_mex_addfield(*c3_info, c3_b_emlrt_marshallOut(0U), "fileTimeHi",
                  "fileTimeHi", 343);
  sf_mex_addfield(*c3_info, c3_b_emlrt_marshallOut(0U), "mFileTimeLo",
                  "mFileTimeLo", 343);
  sf_mex_addfield(*c3_info, c3_b_emlrt_marshallOut(0U), "mFileTimeHi",
                  "mFileTimeHi", 343);
  sf_mex_assign(&c3_rhs343, sf_mex_createcellmatrix(0, 1), false);
  sf_mex_assign(&c3_lhs343, sf_mex_createcellmatrix(0, 1), false);
  sf_mex_addfield(*c3_info, sf_mex_duplicatearraysafe(&c3_rhs343), "rhs", "rhs",
                  343);
  sf_mex_addfield(*c3_info, sf_mex_duplicatearraysafe(&c3_lhs343), "lhs", "lhs",
                  343);
  sf_mex_addfield(*c3_info, c3_emlrt_marshallOut(
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/ops/power.m"), "context",
                  "context", 344);
  sf_mex_addfield(*c3_info, c3_emlrt_marshallOut(
    "coder.internal.isBuiltInNumeric"), "name", "name", 344);
  sf_mex_addfield(*c3_info, c3_emlrt_marshallOut("double"), "dominantType",
                  "dominantType", 344);
  sf_mex_addfield(*c3_info, c3_emlrt_marshallOut(
    "[ILXE]$matlabroot$/toolbox/shared/coder/coder/+coder/+internal/isBuiltInNumeric.m"),
                  "resolved", "resolved", 344);
  sf_mex_addfield(*c3_info, c3_b_emlrt_marshallOut(1395953456U), "fileTimeLo",
                  "fileTimeLo", 344);
  sf_mex_addfield(*c3_info, c3_b_emlrt_marshallOut(0U), "fileTimeHi",
                  "fileTimeHi", 344);
  sf_mex_addfield(*c3_info, c3_b_emlrt_marshallOut(0U), "mFileTimeLo",
                  "mFileTimeLo", 344);
  sf_mex_addfield(*c3_info, c3_b_emlrt_marshallOut(0U), "mFileTimeHi",
                  "mFileTimeHi", 344);
  sf_mex_assign(&c3_rhs344, sf_mex_createcellmatrix(0, 1), false);
  sf_mex_assign(&c3_lhs344, sf_mex_createcellmatrix(0, 1), false);
  sf_mex_addfield(*c3_info, sf_mex_duplicatearraysafe(&c3_rhs344), "rhs", "rhs",
                  344);
  sf_mex_addfield(*c3_info, sf_mex_duplicatearraysafe(&c3_lhs344), "lhs", "lhs",
                  344);
  sf_mex_addfield(*c3_info, c3_emlrt_marshallOut(
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/ops/power.m!fltpower"), "context",
                  "context", 345);
  sf_mex_addfield(*c3_info, c3_emlrt_marshallOut("eml_scalar_eg"), "name",
                  "name", 345);
  sf_mex_addfield(*c3_info, c3_emlrt_marshallOut("double"), "dominantType",
                  "dominantType", 345);
  sf_mex_addfield(*c3_info, c3_emlrt_marshallOut(
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/eml/eml_scalar_eg.m"), "resolved",
                  "resolved", 345);
  sf_mex_addfield(*c3_info, c3_b_emlrt_marshallOut(1376005888U), "fileTimeLo",
                  "fileTimeLo", 345);
  sf_mex_addfield(*c3_info, c3_b_emlrt_marshallOut(0U), "fileTimeHi",
                  "fileTimeHi", 345);
  sf_mex_addfield(*c3_info, c3_b_emlrt_marshallOut(0U), "mFileTimeLo",
                  "mFileTimeLo", 345);
  sf_mex_addfield(*c3_info, c3_b_emlrt_marshallOut(0U), "mFileTimeHi",
                  "mFileTimeHi", 345);
  sf_mex_assign(&c3_rhs345, sf_mex_createcellmatrix(0, 1), false);
  sf_mex_assign(&c3_lhs345, sf_mex_createcellmatrix(0, 1), false);
  sf_mex_addfield(*c3_info, sf_mex_duplicatearraysafe(&c3_rhs345), "rhs", "rhs",
                  345);
  sf_mex_addfield(*c3_info, sf_mex_duplicatearraysafe(&c3_lhs345), "lhs", "lhs",
                  345);
  sf_mex_addfield(*c3_info, c3_emlrt_marshallOut(
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/ops/power.m!fltpower"), "context",
                  "context", 346);
  sf_mex_addfield(*c3_info, c3_emlrt_marshallOut("eml_scalexp_alloc"), "name",
                  "name", 346);
  sf_mex_addfield(*c3_info, c3_emlrt_marshallOut("double"), "dominantType",
                  "dominantType", 346);
  sf_mex_addfield(*c3_info, c3_emlrt_marshallOut(
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/eml/eml_scalexp_alloc.m"),
                  "resolved", "resolved", 346);
  sf_mex_addfield(*c3_info, c3_b_emlrt_marshallOut(1376005888U), "fileTimeLo",
                  "fileTimeLo", 346);
  sf_mex_addfield(*c3_info, c3_b_emlrt_marshallOut(0U), "fileTimeHi",
                  "fileTimeHi", 346);
  sf_mex_addfield(*c3_info, c3_b_emlrt_marshallOut(0U), "mFileTimeLo",
                  "mFileTimeLo", 346);
  sf_mex_addfield(*c3_info, c3_b_emlrt_marshallOut(0U), "mFileTimeHi",
                  "mFileTimeHi", 346);
  sf_mex_assign(&c3_rhs346, sf_mex_createcellmatrix(0, 1), false);
  sf_mex_assign(&c3_lhs346, sf_mex_createcellmatrix(0, 1), false);
  sf_mex_addfield(*c3_info, sf_mex_duplicatearraysafe(&c3_rhs346), "rhs", "rhs",
                  346);
  sf_mex_addfield(*c3_info, sf_mex_duplicatearraysafe(&c3_lhs346), "lhs", "lhs",
                  346);
  sf_mex_addfield(*c3_info, c3_emlrt_marshallOut(
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/ops/power.m!fltpower"), "context",
                  "context", 347);
  sf_mex_addfield(*c3_info, c3_emlrt_marshallOut("floor"), "name", "name", 347);
  sf_mex_addfield(*c3_info, c3_emlrt_marshallOut("double"), "dominantType",
                  "dominantType", 347);
  sf_mex_addfield(*c3_info, c3_emlrt_marshallOut(
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/elfun/floor.m"), "resolved",
                  "resolved", 347);
  sf_mex_addfield(*c3_info, c3_b_emlrt_marshallOut(1363735454U), "fileTimeLo",
                  "fileTimeLo", 347);
  sf_mex_addfield(*c3_info, c3_b_emlrt_marshallOut(0U), "fileTimeHi",
                  "fileTimeHi", 347);
  sf_mex_addfield(*c3_info, c3_b_emlrt_marshallOut(0U), "mFileTimeLo",
                  "mFileTimeLo", 347);
  sf_mex_addfield(*c3_info, c3_b_emlrt_marshallOut(0U), "mFileTimeHi",
                  "mFileTimeHi", 347);
  sf_mex_assign(&c3_rhs347, sf_mex_createcellmatrix(0, 1), false);
  sf_mex_assign(&c3_lhs347, sf_mex_createcellmatrix(0, 1), false);
  sf_mex_addfield(*c3_info, sf_mex_duplicatearraysafe(&c3_rhs347), "rhs", "rhs",
                  347);
  sf_mex_addfield(*c3_info, sf_mex_duplicatearraysafe(&c3_lhs347), "lhs", "lhs",
                  347);
  sf_mex_addfield(*c3_info, c3_emlrt_marshallOut(
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/ops/power.m!scalar_float_power"),
                  "context", "context", 348);
  sf_mex_addfield(*c3_info, c3_emlrt_marshallOut("eml_scalar_eg"), "name",
                  "name", 348);
  sf_mex_addfield(*c3_info, c3_emlrt_marshallOut("double"), "dominantType",
                  "dominantType", 348);
  sf_mex_addfield(*c3_info, c3_emlrt_marshallOut(
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/eml/eml_scalar_eg.m"), "resolved",
                  "resolved", 348);
  sf_mex_addfield(*c3_info, c3_b_emlrt_marshallOut(1376005888U), "fileTimeLo",
                  "fileTimeLo", 348);
  sf_mex_addfield(*c3_info, c3_b_emlrt_marshallOut(0U), "fileTimeHi",
                  "fileTimeHi", 348);
  sf_mex_addfield(*c3_info, c3_b_emlrt_marshallOut(0U), "mFileTimeLo",
                  "mFileTimeLo", 348);
  sf_mex_addfield(*c3_info, c3_b_emlrt_marshallOut(0U), "mFileTimeHi",
                  "mFileTimeHi", 348);
  sf_mex_assign(&c3_rhs348, sf_mex_createcellmatrix(0, 1), false);
  sf_mex_assign(&c3_lhs348, sf_mex_createcellmatrix(0, 1), false);
  sf_mex_addfield(*c3_info, sf_mex_duplicatearraysafe(&c3_rhs348), "rhs", "rhs",
                  348);
  sf_mex_addfield(*c3_info, sf_mex_duplicatearraysafe(&c3_lhs348), "lhs", "lhs",
                  348);
  sf_mex_addfield(*c3_info, c3_emlrt_marshallOut(
    "[E]C:/Users/sd0016/Dropbox/UAH Doktora/Journal Tracking/Matlab Code/3DOF_Experiments/get3DRotationAngles.m"),
                  "context", "context", 349);
  sf_mex_addfield(*c3_info, c3_emlrt_marshallOut("sqrt"), "name", "name", 349);
  sf_mex_addfield(*c3_info, c3_emlrt_marshallOut("double"), "dominantType",
                  "dominantType", 349);
  sf_mex_addfield(*c3_info, c3_emlrt_marshallOut(
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/elfun/sqrt.m"), "resolved",
                  "resolved", 349);
  sf_mex_addfield(*c3_info, c3_b_emlrt_marshallOut(1343855586U), "fileTimeLo",
                  "fileTimeLo", 349);
  sf_mex_addfield(*c3_info, c3_b_emlrt_marshallOut(0U), "fileTimeHi",
                  "fileTimeHi", 349);
  sf_mex_addfield(*c3_info, c3_b_emlrt_marshallOut(0U), "mFileTimeLo",
                  "mFileTimeLo", 349);
  sf_mex_addfield(*c3_info, c3_b_emlrt_marshallOut(0U), "mFileTimeHi",
                  "mFileTimeHi", 349);
  sf_mex_assign(&c3_rhs349, sf_mex_createcellmatrix(0, 1), false);
  sf_mex_assign(&c3_lhs349, sf_mex_createcellmatrix(0, 1), false);
  sf_mex_addfield(*c3_info, sf_mex_duplicatearraysafe(&c3_rhs349), "rhs", "rhs",
                  349);
  sf_mex_addfield(*c3_info, sf_mex_duplicatearraysafe(&c3_lhs349), "lhs", "lhs",
                  349);
  sf_mex_addfield(*c3_info, c3_emlrt_marshallOut(
    "[E]C:/Users/sd0016/Dropbox/UAH Doktora/Journal Tracking/Matlab Code/3DOF_Experiments/get3DRotationAngles.m"),
                  "context", "context", 350);
  sf_mex_addfield(*c3_info, c3_emlrt_marshallOut("atan2"), "name", "name", 350);
  sf_mex_addfield(*c3_info, c3_emlrt_marshallOut("double"), "dominantType",
                  "dominantType", 350);
  sf_mex_addfield(*c3_info, c3_emlrt_marshallOut(
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/elfun/atan2.m"), "resolved",
                  "resolved", 350);
  sf_mex_addfield(*c3_info, c3_b_emlrt_marshallOut(1395350096U), "fileTimeLo",
                  "fileTimeLo", 350);
  sf_mex_addfield(*c3_info, c3_b_emlrt_marshallOut(0U), "fileTimeHi",
                  "fileTimeHi", 350);
  sf_mex_addfield(*c3_info, c3_b_emlrt_marshallOut(0U), "mFileTimeLo",
                  "mFileTimeLo", 350);
  sf_mex_addfield(*c3_info, c3_b_emlrt_marshallOut(0U), "mFileTimeHi",
                  "mFileTimeHi", 350);
  sf_mex_assign(&c3_rhs350, sf_mex_createcellmatrix(0, 1), false);
  sf_mex_assign(&c3_lhs350, sf_mex_createcellmatrix(0, 1), false);
  sf_mex_addfield(*c3_info, sf_mex_duplicatearraysafe(&c3_rhs350), "rhs", "rhs",
                  350);
  sf_mex_addfield(*c3_info, sf_mex_duplicatearraysafe(&c3_lhs350), "lhs", "lhs",
                  350);
  sf_mex_addfield(*c3_info, c3_emlrt_marshallOut(
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/elfun/atan2.m"), "context",
                  "context", 351);
  sf_mex_addfield(*c3_info, c3_emlrt_marshallOut("eml_scalar_eg"), "name",
                  "name", 351);
  sf_mex_addfield(*c3_info, c3_emlrt_marshallOut("double"), "dominantType",
                  "dominantType", 351);
  sf_mex_addfield(*c3_info, c3_emlrt_marshallOut(
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/eml/eml_scalar_eg.m"), "resolved",
                  "resolved", 351);
  sf_mex_addfield(*c3_info, c3_b_emlrt_marshallOut(1376005888U), "fileTimeLo",
                  "fileTimeLo", 351);
  sf_mex_addfield(*c3_info, c3_b_emlrt_marshallOut(0U), "fileTimeHi",
                  "fileTimeHi", 351);
  sf_mex_addfield(*c3_info, c3_b_emlrt_marshallOut(0U), "mFileTimeLo",
                  "mFileTimeLo", 351);
  sf_mex_addfield(*c3_info, c3_b_emlrt_marshallOut(0U), "mFileTimeHi",
                  "mFileTimeHi", 351);
  sf_mex_assign(&c3_rhs351, sf_mex_createcellmatrix(0, 1), false);
  sf_mex_assign(&c3_lhs351, sf_mex_createcellmatrix(0, 1), false);
  sf_mex_addfield(*c3_info, sf_mex_duplicatearraysafe(&c3_rhs351), "rhs", "rhs",
                  351);
  sf_mex_addfield(*c3_info, sf_mex_duplicatearraysafe(&c3_lhs351), "lhs", "lhs",
                  351);
  sf_mex_addfield(*c3_info, c3_emlrt_marshallOut(
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/elfun/atan2.m"), "context",
                  "context", 352);
  sf_mex_addfield(*c3_info, c3_emlrt_marshallOut("eml_scalexp_alloc"), "name",
                  "name", 352);
  sf_mex_addfield(*c3_info, c3_emlrt_marshallOut("double"), "dominantType",
                  "dominantType", 352);
  sf_mex_addfield(*c3_info, c3_emlrt_marshallOut(
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/eml/eml_scalexp_alloc.m"),
                  "resolved", "resolved", 352);
  sf_mex_addfield(*c3_info, c3_b_emlrt_marshallOut(1376005888U), "fileTimeLo",
                  "fileTimeLo", 352);
  sf_mex_addfield(*c3_info, c3_b_emlrt_marshallOut(0U), "fileTimeHi",
                  "fileTimeHi", 352);
  sf_mex_addfield(*c3_info, c3_b_emlrt_marshallOut(0U), "mFileTimeLo",
                  "mFileTimeLo", 352);
  sf_mex_addfield(*c3_info, c3_b_emlrt_marshallOut(0U), "mFileTimeHi",
                  "mFileTimeHi", 352);
  sf_mex_assign(&c3_rhs352, sf_mex_createcellmatrix(0, 1), false);
  sf_mex_assign(&c3_lhs352, sf_mex_createcellmatrix(0, 1), false);
  sf_mex_addfield(*c3_info, sf_mex_duplicatearraysafe(&c3_rhs352), "rhs", "rhs",
                  352);
  sf_mex_addfield(*c3_info, sf_mex_duplicatearraysafe(&c3_lhs352), "lhs", "lhs",
                  352);
  sf_mex_addfield(*c3_info, c3_emlrt_marshallOut(
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/elfun/atan2.m"), "context",
                  "context", 353);
  sf_mex_addfield(*c3_info, c3_emlrt_marshallOut("eml_scalar_atan2"), "name",
                  "name", 353);
  sf_mex_addfield(*c3_info, c3_emlrt_marshallOut("double"), "dominantType",
                  "dominantType", 353);
  sf_mex_addfield(*c3_info, c3_emlrt_marshallOut(
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/elfun/eml_scalar_atan2.m"),
                  "resolved", "resolved", 353);
  sf_mex_addfield(*c3_info, c3_b_emlrt_marshallOut(1286843920U), "fileTimeLo",
                  "fileTimeLo", 353);
  sf_mex_addfield(*c3_info, c3_b_emlrt_marshallOut(0U), "fileTimeHi",
                  "fileTimeHi", 353);
  sf_mex_addfield(*c3_info, c3_b_emlrt_marshallOut(0U), "mFileTimeLo",
                  "mFileTimeLo", 353);
  sf_mex_addfield(*c3_info, c3_b_emlrt_marshallOut(0U), "mFileTimeHi",
                  "mFileTimeHi", 353);
  sf_mex_assign(&c3_rhs353, sf_mex_createcellmatrix(0, 1), false);
  sf_mex_assign(&c3_lhs353, sf_mex_createcellmatrix(0, 1), false);
  sf_mex_addfield(*c3_info, sf_mex_duplicatearraysafe(&c3_rhs353), "rhs", "rhs",
                  353);
  sf_mex_addfield(*c3_info, sf_mex_duplicatearraysafe(&c3_lhs353), "lhs", "lhs",
                  353);
  sf_mex_addfield(*c3_info, c3_emlrt_marshallOut(
    "[E]C:/Users/sd0016/Dropbox/UAH Doktora/Journal Tracking/Matlab Code/3DOF_Experiments/get3DRotationAngles.m"),
                  "context", "context", 354);
  sf_mex_addfield(*c3_info, c3_emlrt_marshallOut("cos"), "name", "name", 354);
  sf_mex_addfield(*c3_info, c3_emlrt_marshallOut("double"), "dominantType",
                  "dominantType", 354);
  sf_mex_addfield(*c3_info, c3_emlrt_marshallOut(
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/elfun/cos.m"), "resolved",
                  "resolved", 354);
  sf_mex_addfield(*c3_info, c3_b_emlrt_marshallOut(1395350096U), "fileTimeLo",
                  "fileTimeLo", 354);
  sf_mex_addfield(*c3_info, c3_b_emlrt_marshallOut(0U), "fileTimeHi",
                  "fileTimeHi", 354);
  sf_mex_addfield(*c3_info, c3_b_emlrt_marshallOut(0U), "mFileTimeLo",
                  "mFileTimeLo", 354);
  sf_mex_addfield(*c3_info, c3_b_emlrt_marshallOut(0U), "mFileTimeHi",
                  "mFileTimeHi", 354);
  sf_mex_assign(&c3_rhs354, sf_mex_createcellmatrix(0, 1), false);
  sf_mex_assign(&c3_lhs354, sf_mex_createcellmatrix(0, 1), false);
  sf_mex_addfield(*c3_info, sf_mex_duplicatearraysafe(&c3_rhs354), "rhs", "rhs",
                  354);
  sf_mex_addfield(*c3_info, sf_mex_duplicatearraysafe(&c3_lhs354), "lhs", "lhs",
                  354);
  sf_mex_addfield(*c3_info, c3_emlrt_marshallOut(
    "[E]C:/Users/sd0016/Dropbox/UAH Doktora/Journal Tracking/Matlab Code/3DOF_Experiments/get3DRotationAngles.m"),
                  "context", "context", 355);
  sf_mex_addfield(*c3_info, c3_emlrt_marshallOut("mrdivide"), "name", "name",
                  355);
  sf_mex_addfield(*c3_info, c3_emlrt_marshallOut("double"), "dominantType",
                  "dominantType", 355);
  sf_mex_addfield(*c3_info, c3_emlrt_marshallOut(
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/ops/mrdivide.p"), "resolved",
                  "resolved", 355);
  sf_mex_addfield(*c3_info, c3_b_emlrt_marshallOut(1410832848U), "fileTimeLo",
                  "fileTimeLo", 355);
  sf_mex_addfield(*c3_info, c3_b_emlrt_marshallOut(0U), "fileTimeHi",
                  "fileTimeHi", 355);
  sf_mex_addfield(*c3_info, c3_b_emlrt_marshallOut(1370035086U), "mFileTimeLo",
                  "mFileTimeLo", 355);
  sf_mex_addfield(*c3_info, c3_b_emlrt_marshallOut(0U), "mFileTimeHi",
                  "mFileTimeHi", 355);
  sf_mex_assign(&c3_rhs355, sf_mex_createcellmatrix(0, 1), false);
  sf_mex_assign(&c3_lhs355, sf_mex_createcellmatrix(0, 1), false);
  sf_mex_addfield(*c3_info, sf_mex_duplicatearraysafe(&c3_rhs355), "rhs", "rhs",
                  355);
  sf_mex_addfield(*c3_info, sf_mex_duplicatearraysafe(&c3_lhs355), "lhs", "lhs",
                  355);
  sf_mex_destroy(&c3_rhs320);
  sf_mex_destroy(&c3_lhs320);
  sf_mex_destroy(&c3_rhs321);
  sf_mex_destroy(&c3_lhs321);
  sf_mex_destroy(&c3_rhs322);
  sf_mex_destroy(&c3_lhs322);
  sf_mex_destroy(&c3_rhs323);
  sf_mex_destroy(&c3_lhs323);
  sf_mex_destroy(&c3_rhs324);
  sf_mex_destroy(&c3_lhs324);
  sf_mex_destroy(&c3_rhs325);
  sf_mex_destroy(&c3_lhs325);
  sf_mex_destroy(&c3_rhs326);
  sf_mex_destroy(&c3_lhs326);
  sf_mex_destroy(&c3_rhs327);
  sf_mex_destroy(&c3_lhs327);
  sf_mex_destroy(&c3_rhs328);
  sf_mex_destroy(&c3_lhs328);
  sf_mex_destroy(&c3_rhs329);
  sf_mex_destroy(&c3_lhs329);
  sf_mex_destroy(&c3_rhs330);
  sf_mex_destroy(&c3_lhs330);
  sf_mex_destroy(&c3_rhs331);
  sf_mex_destroy(&c3_lhs331);
  sf_mex_destroy(&c3_rhs332);
  sf_mex_destroy(&c3_lhs332);
  sf_mex_destroy(&c3_rhs333);
  sf_mex_destroy(&c3_lhs333);
  sf_mex_destroy(&c3_rhs334);
  sf_mex_destroy(&c3_lhs334);
  sf_mex_destroy(&c3_rhs335);
  sf_mex_destroy(&c3_lhs335);
  sf_mex_destroy(&c3_rhs336);
  sf_mex_destroy(&c3_lhs336);
  sf_mex_destroy(&c3_rhs337);
  sf_mex_destroy(&c3_lhs337);
  sf_mex_destroy(&c3_rhs338);
  sf_mex_destroy(&c3_lhs338);
  sf_mex_destroy(&c3_rhs339);
  sf_mex_destroy(&c3_lhs339);
  sf_mex_destroy(&c3_rhs340);
  sf_mex_destroy(&c3_lhs340);
  sf_mex_destroy(&c3_rhs341);
  sf_mex_destroy(&c3_lhs341);
  sf_mex_destroy(&c3_rhs342);
  sf_mex_destroy(&c3_lhs342);
  sf_mex_destroy(&c3_rhs343);
  sf_mex_destroy(&c3_lhs343);
  sf_mex_destroy(&c3_rhs344);
  sf_mex_destroy(&c3_lhs344);
  sf_mex_destroy(&c3_rhs345);
  sf_mex_destroy(&c3_lhs345);
  sf_mex_destroy(&c3_rhs346);
  sf_mex_destroy(&c3_lhs346);
  sf_mex_destroy(&c3_rhs347);
  sf_mex_destroy(&c3_lhs347);
  sf_mex_destroy(&c3_rhs348);
  sf_mex_destroy(&c3_lhs348);
  sf_mex_destroy(&c3_rhs349);
  sf_mex_destroy(&c3_lhs349);
  sf_mex_destroy(&c3_rhs350);
  sf_mex_destroy(&c3_lhs350);
  sf_mex_destroy(&c3_rhs351);
  sf_mex_destroy(&c3_lhs351);
  sf_mex_destroy(&c3_rhs352);
  sf_mex_destroy(&c3_lhs352);
  sf_mex_destroy(&c3_rhs353);
  sf_mex_destroy(&c3_lhs353);
  sf_mex_destroy(&c3_rhs354);
  sf_mex_destroy(&c3_lhs354);
  sf_mex_destroy(&c3_rhs355);
  sf_mex_destroy(&c3_lhs355);
}

static void c3_rand(SFc3_Robot_Control_v01InstanceStruct *chartInstance, real_T
                    c3_r[4])
{
  c3_eml_rand(chartInstance, c3_r);
}

static void c3_eml_rand(SFc3_Robot_Control_v01InstanceStruct *chartInstance,
  real_T c3_r[4])
{
  int32_T c3_k;
  real_T c3_b_k;
  uint32_T c3_hoistedGlobal;
  uint32_T c3_d_state;
  uint32_T c3_e_state;
  uint32_T c3_s;
  uint32_T c3_u0;
  uint32_T c3_hi;
  uint32_T c3_lo;
  uint32_T c3_test1;
  uint32_T c3_test2;
  uint32_T c3_f_state;
  real_T c3_b_r;
  real_T c3_d1;
  int32_T c3_i307;
  int32_T c3_c_k;
  real_T c3_d_k;
  uint32_T c3_icng;
  uint32_T c3_jsr;
  uint32_T c3_u1;
  uint32_T c3_u2;
  uint32_T c3_ui;
  uint32_T c3_b_ui;
  real_T c3_c_r;
  real_T c3_d2;
  uint32_T c3_uv2[625];
  int32_T c3_i308;
  int32_T c3_e_k;
  real_T c3_f_k;
  real_T c3_d3;
  if (!chartInstance->c3_method_not_empty) {
    chartInstance->c3_method = 7U;
    chartInstance->c3_method_not_empty = true;
  }

  if (chartInstance->c3_method == 4U) {
    if (!chartInstance->c3_state_not_empty) {
      chartInstance->c3_state = 1144108930U;
      chartInstance->c3_state_not_empty = true;
    }

    for (c3_k = 0; c3_k < 4; c3_k++) {
      c3_b_k = 1.0 + (real_T)c3_k;
      c3_hoistedGlobal = chartInstance->c3_state;
      c3_d_state = c3_hoistedGlobal;
      c3_e_state = c3_d_state;
      c3_s = c3_e_state;
      c3_u0 = 127773U;
      if (c3_u0 == 0U) {
        c3_hi = MAX_uint32_T;
      } else {
        c3_hi = c3_s / c3_u0;
      }

      c3_lo = c3_s - c3_hi * 127773U;
      c3_test1 = 16807U * c3_lo;
      c3_test2 = 2836U * c3_hi;
      if (c3_test1 < c3_test2) {
        c3_f_state = (c3_test1 - c3_test2) + 2147483647U;
      } else {
        c3_f_state = c3_test1 - c3_test2;
      }

      c3_b_r = (real_T)c3_f_state * 4.6566128752457969E-10;
      c3_e_state = c3_f_state;
      c3_d1 = c3_b_r;
      chartInstance->c3_state = c3_e_state;
      c3_r[_SFD_EML_ARRAY_BOUNDS_CHECK("", (int32_T)_SFD_INTEGER_CHECK("",
        c3_b_k), 1, 4, 1, 0) - 1] = c3_d1;
    }
  } else if (chartInstance->c3_method == 5U) {
    if (!chartInstance->c3_b_state_not_empty) {
      for (c3_i307 = 0; c3_i307 < 2; c3_i307++) {
        chartInstance->c3_b_state[c3_i307] = 362436069U + 158852560U * (uint32_T)
          c3_i307;
      }

      chartInstance->c3_b_state_not_empty = true;
    }

    for (c3_c_k = 0; c3_c_k < 4; c3_c_k++) {
      c3_d_k = 1.0 + (real_T)c3_c_k;
      c3_icng = chartInstance->c3_b_state[0];
      c3_jsr = chartInstance->c3_b_state[1];
      c3_u1 = c3_jsr;
      c3_u2 = c3_icng;
      c3_u2 = 69069U * c3_u2 + 1234567U;
      c3_u1 ^= c3_u1 << 13;
      c3_u1 ^= c3_u1 >> 17;
      c3_u1 ^= c3_u1 << 5;
      c3_ui = c3_u2 + c3_u1;
      chartInstance->c3_b_state[0] = c3_u2;
      chartInstance->c3_b_state[1] = c3_u1;
      c3_b_ui = c3_ui;
      c3_c_r = (real_T)c3_b_ui * 2.328306436538696E-10;
      c3_d2 = c3_c_r;
      c3_r[_SFD_EML_ARRAY_BOUNDS_CHECK("", (int32_T)_SFD_INTEGER_CHECK("",
        c3_d_k), 1, 4, 1, 0) - 1] = c3_d2;
    }
  } else {
    if (!chartInstance->c3_c_state_not_empty) {
      c3_eml_rand_mt19937ar(chartInstance, c3_uv2);
      for (c3_i308 = 0; c3_i308 < 625; c3_i308++) {
        chartInstance->c3_c_state[c3_i308] = c3_uv2[c3_i308];
      }

      chartInstance->c3_c_state_not_empty = true;
    }

    for (c3_e_k = 0; c3_e_k < 4; c3_e_k++) {
      c3_f_k = 1.0 + (real_T)c3_e_k;
      c3_d3 = c3_c_eml_rand_mt19937ar(chartInstance, chartInstance->c3_c_state);
      c3_r[_SFD_EML_ARRAY_BOUNDS_CHECK("", (int32_T)_SFD_INTEGER_CHECK("",
        c3_f_k), 1, 4, 1, 0) - 1] = c3_d3;
    }
  }
}

static void c3_eml_rand_mt19937ar(SFc3_Robot_Control_v01InstanceStruct
  *chartInstance, uint32_T c3_d_state[625])
{
  int32_T c3_i309;
  for (c3_i309 = 0; c3_i309 < 625; c3_i309++) {
    c3_d_state[c3_i309] = 0U;
  }

  c3_b_twister_state_vector(chartInstance, c3_d_state, 5489.0);
}

static void c3_twister_state_vector(SFc3_Robot_Control_v01InstanceStruct
  *chartInstance, uint32_T c3_mt[625], real_T c3_seed, uint32_T c3_b_mt[625])
{
  int32_T c3_i310;
  for (c3_i310 = 0; c3_i310 < 625; c3_i310++) {
    c3_b_mt[c3_i310] = c3_mt[c3_i310];
  }

  c3_b_twister_state_vector(chartInstance, c3_b_mt, c3_seed);
}

static void c3_b_eml_rand_mt19937ar(SFc3_Robot_Control_v01InstanceStruct
  *chartInstance, uint32_T c3_d_state[625], uint32_T c3_e_state[625], real_T
  *c3_r)
{
  int32_T c3_i311;
  for (c3_i311 = 0; c3_i311 < 625; c3_i311++) {
    c3_e_state[c3_i311] = c3_d_state[c3_i311];
  }

  *c3_r = c3_c_eml_rand_mt19937ar(chartInstance, c3_e_state);
}

static void c3_eml_eps(SFc3_Robot_Control_v01InstanceStruct *chartInstance)
{
  (void)chartInstance;
}

static void c3_eml_error(SFc3_Robot_Control_v01InstanceStruct *chartInstance)
{
  int32_T c3_i312;
  static char_T c3_cv1[37] = { 'C', 'o', 'd', 'e', 'r', ':', 'M', 'A', 'T', 'L',
    'A', 'B', ':', 'r', 'a', 'n', 'd', '_', 'i', 'n', 'v', 'a', 'l', 'i', 'd',
    'T', 'w', 'i', 's', 't', 'e', 'r', 'S', 't', 'a', 't', 'e' };

  char_T c3_u[37];
  const mxArray *c3_y = NULL;
  (void)chartInstance;
  for (c3_i312 = 0; c3_i312 < 37; c3_i312++) {
    c3_u[c3_i312] = c3_cv1[c3_i312];
  }

  c3_y = NULL;
  sf_mex_assign(&c3_y, sf_mex_create("y", c3_u, 10, 0U, 1U, 0U, 2, 1, 37), false);
  sf_mex_call_debug(sfGlobalDebugInstanceStruct, "error", 0U, 1U, 14,
                    sf_mex_call_debug(sfGlobalDebugInstanceStruct, "message", 1U,
    1U, 14, c3_y));
}

static void c3_eml_scalar_eg(SFc3_Robot_Control_v01InstanceStruct *chartInstance)
{
  (void)chartInstance;
}

static void c3_eml_xgemm(SFc3_Robot_Control_v01InstanceStruct *chartInstance,
  real_T c3_A[16], real_T c3_B[16], real_T c3_C[16], real_T c3_b_C[16])
{
  int32_T c3_i313;
  int32_T c3_i314;
  real_T c3_b_A[16];
  int32_T c3_i315;
  real_T c3_b_B[16];
  for (c3_i313 = 0; c3_i313 < 16; c3_i313++) {
    c3_b_C[c3_i313] = c3_C[c3_i313];
  }

  for (c3_i314 = 0; c3_i314 < 16; c3_i314++) {
    c3_b_A[c3_i314] = c3_A[c3_i314];
  }

  for (c3_i315 = 0; c3_i315 < 16; c3_i315++) {
    c3_b_B[c3_i315] = c3_B[c3_i315];
  }

  c3_b_eml_xgemm(chartInstance, c3_b_A, c3_b_B, c3_b_C);
}

static void c3_inv(SFc3_Robot_Control_v01InstanceStruct *chartInstance, real_T
                   c3_x[16], real_T c3_y[16])
{
  int32_T c3_i316;
  real_T c3_b_x[16];
  int32_T c3_i317;
  int32_T c3_info;
  int32_T c3_ipiv[4];
  int32_T c3_i318;
  int32_T c3_b_ipiv[4];
  int32_T c3_p[4];
  int32_T c3_k;
  int32_T c3_b_k;
  int32_T c3_c;
  int32_T c3_c_k;
  int32_T c3_a;
  int32_T c3_b_a;
  boolean_T c3_overflow;
  int32_T c3_j;
  int32_T c3_b_j;
  int32_T c3_c_a;
  int32_T c3_d_a;
  int32_T c3_i319;
  int32_T c3_e_a;
  int32_T c3_f_a;
  boolean_T c3_b_overflow;
  int32_T c3_i;
  int32_T c3_b_i;
  int32_T c3_i320;
  real_T c3_c_x[16];
  int32_T c3_i321;
  real_T c3_d_x[16];
  real_T c3_n1x;
  int32_T c3_i322;
  real_T c3_b_y[16];
  real_T c3_n1xinv;
  real_T c3_rc;
  real_T c3_e_x;
  boolean_T c3_b;
  real_T c3_f_x;
  int32_T c3_i323;
  static char_T c3_cv2[8] = { '%', '%', '%', 'd', '.', '%', 'd', 'e' };

  char_T c3_u[8];
  const mxArray *c3_c_y = NULL;
  real_T c3_b_u;
  const mxArray *c3_d_y = NULL;
  real_T c3_c_u;
  const mxArray *c3_e_y = NULL;
  real_T c3_d_u;
  const mxArray *c3_f_y = NULL;
  char_T c3_str[14];
  int32_T c3_i324;
  char_T c3_b_str[14];
  boolean_T guard1 = false;
  boolean_T guard2 = false;
  boolean_T guard3 = false;
  for (c3_i316 = 0; c3_i316 < 16; c3_i316++) {
    c3_b_x[c3_i316] = c3_x[c3_i316];
  }

  for (c3_i317 = 0; c3_i317 < 16; c3_i317++) {
    c3_y[c3_i317] = 0.0;
  }

  c3_b_eml_matlab_zgetrf(chartInstance, c3_b_x, c3_ipiv, &c3_info);
  for (c3_i318 = 0; c3_i318 < 4; c3_i318++) {
    c3_b_ipiv[c3_i318] = c3_ipiv[c3_i318];
  }

  c3_eml_ipiv2perm(chartInstance, c3_b_ipiv, c3_p);
  c3_eml_switch_helper(chartInstance);
  for (c3_k = 1; c3_k < 5; c3_k++) {
    c3_b_k = c3_k;
    c3_c = c3_p[_SFD_EML_ARRAY_BOUNDS_CHECK("", (int32_T)_SFD_INTEGER_CHECK("",
      (real_T)c3_b_k), 1, 4, 1, 0) - 1];
    c3_y[(_SFD_EML_ARRAY_BOUNDS_CHECK("", (int32_T)_SFD_INTEGER_CHECK("",
            (real_T)c3_b_k), 1, 4, 1, 0) + ((_SFD_EML_ARRAY_BOUNDS_CHECK("",
             (int32_T)_SFD_INTEGER_CHECK("", (real_T)c3_c), 1, 4, 2, 0) - 1) <<
           2)) - 1] = 1.0;
    c3_c_k = c3_b_k;
    c3_a = c3_c_k;
    c3_b_a = c3_a;
    if (c3_b_a > 4) {
      c3_overflow = false;
    } else {
      c3_eml_switch_helper(chartInstance);
      c3_eml_switch_helper(chartInstance);
      c3_overflow = false;
    }

    if (c3_overflow) {
      c3_check_forloop_overflow_error(chartInstance, c3_overflow);
    }

    for (c3_j = c3_c_k; c3_j < 5; c3_j++) {
      c3_b_j = c3_j;
      if (c3_y[(_SFD_EML_ARRAY_BOUNDS_CHECK("", (int32_T)_SFD_INTEGER_CHECK("",
             (real_T)c3_b_j), 1, 4, 1, 0) + ((_SFD_EML_ARRAY_BOUNDS_CHECK("",
              (int32_T)_SFD_INTEGER_CHECK("", (real_T)c3_c), 1, 4, 2, 0) - 1) <<
            2)) - 1] != 0.0) {
        c3_c_a = c3_b_j;
        c3_d_a = c3_c_a + 1;
        c3_i319 = c3_d_a;
        c3_e_a = c3_i319;
        c3_f_a = c3_e_a;
        if (c3_f_a > 4) {
          c3_b_overflow = false;
        } else {
          c3_eml_switch_helper(chartInstance);
          c3_eml_switch_helper(chartInstance);
          c3_b_overflow = false;
        }

        if (c3_b_overflow) {
          c3_check_forloop_overflow_error(chartInstance, c3_b_overflow);
        }

        for (c3_i = c3_i319; c3_i < 5; c3_i++) {
          c3_b_i = c3_i;
          c3_y[(_SFD_EML_ARRAY_BOUNDS_CHECK("", (int32_T)_SFD_INTEGER_CHECK("",
                  (real_T)c3_b_i), 1, 4, 1, 0) + ((_SFD_EML_ARRAY_BOUNDS_CHECK(
                   "", (int32_T)_SFD_INTEGER_CHECK("", (real_T)c3_c), 1, 4, 2, 0)
                  - 1) << 2)) - 1] = c3_y[(_SFD_EML_ARRAY_BOUNDS_CHECK("",
            (int32_T)_SFD_INTEGER_CHECK("", (real_T)c3_b_i), 1, 4, 1, 0) +
            ((_SFD_EML_ARRAY_BOUNDS_CHECK("", (int32_T)_SFD_INTEGER_CHECK("",
            (real_T)c3_c), 1, 4, 2, 0) - 1) << 2)) - 1] - c3_y
            [(_SFD_EML_ARRAY_BOUNDS_CHECK("", (int32_T)_SFD_INTEGER_CHECK("",
                (real_T)c3_b_j), 1, 4, 1, 0) + ((_SFD_EML_ARRAY_BOUNDS_CHECK("",
                 (int32_T)_SFD_INTEGER_CHECK("", (real_T)c3_c), 1, 4, 2, 0) - 1)
               << 2)) - 1] * c3_b_x[(_SFD_EML_ARRAY_BOUNDS_CHECK("", (int32_T)
            _SFD_INTEGER_CHECK("", (real_T)c3_b_i), 1, 4, 1, 0) +
            ((_SFD_EML_ARRAY_BOUNDS_CHECK("", (int32_T)_SFD_INTEGER_CHECK("",
            (real_T)c3_b_j), 1, 4, 2, 0) - 1) << 2)) - 1];
        }
      }
    }
  }

  for (c3_i320 = 0; c3_i320 < 16; c3_i320++) {
    c3_c_x[c3_i320] = c3_b_x[c3_i320];
  }

  c3_b_eml_xtrsm(chartInstance, c3_c_x, c3_y);
  for (c3_i321 = 0; c3_i321 < 16; c3_i321++) {
    c3_d_x[c3_i321] = c3_x[c3_i321];
  }

  c3_n1x = c3_norm(chartInstance, c3_d_x);
  for (c3_i322 = 0; c3_i322 < 16; c3_i322++) {
    c3_b_y[c3_i322] = c3_y[c3_i322];
  }

  c3_n1xinv = c3_norm(chartInstance, c3_b_y);
  c3_rc = 1.0 / (c3_n1x * c3_n1xinv);
  guard1 = false;
  guard2 = false;
  if (c3_n1x == 0.0) {
    guard2 = true;
  } else if (c3_n1xinv == 0.0) {
    guard2 = true;
  } else if (c3_rc == 0.0) {
    guard1 = true;
  } else {
    c3_e_x = c3_rc;
    c3_b = muDoubleScalarIsNaN(c3_e_x);
    guard3 = false;
    if (c3_b) {
      guard3 = true;
    } else {
      c3_eml_eps(chartInstance);
      if (c3_rc < 2.2204460492503131E-16) {
        guard3 = true;
      }
    }

    if (guard3 == true) {
      c3_f_x = c3_rc;
      for (c3_i323 = 0; c3_i323 < 8; c3_i323++) {
        c3_u[c3_i323] = c3_cv2[c3_i323];
      }

      c3_c_y = NULL;
      sf_mex_assign(&c3_c_y, sf_mex_create("y", c3_u, 10, 0U, 1U, 0U, 2, 1, 8),
                    false);
      c3_b_u = 14.0;
      c3_d_y = NULL;
      sf_mex_assign(&c3_d_y, sf_mex_create("y", &c3_b_u, 0, 0U, 0U, 0U, 0),
                    false);
      c3_c_u = 6.0;
      c3_e_y = NULL;
      sf_mex_assign(&c3_e_y, sf_mex_create("y", &c3_c_u, 0, 0U, 0U, 0U, 0),
                    false);
      c3_d_u = c3_f_x;
      c3_f_y = NULL;
      sf_mex_assign(&c3_f_y, sf_mex_create("y", &c3_d_u, 0, 0U, 0U, 0U, 0),
                    false);
      c3_emlrt_marshallIn(chartInstance, sf_mex_call_debug
                          (sfGlobalDebugInstanceStruct, "sprintf", 1U, 2U, 14,
                           sf_mex_call_debug(sfGlobalDebugInstanceStruct,
        "sprintf", 1U, 3U, 14, c3_c_y, 14, c3_d_y, 14, c3_e_y), 14, c3_f_y),
                          "sprintf", c3_str);
      for (c3_i324 = 0; c3_i324 < 14; c3_i324++) {
        c3_b_str[c3_i324] = c3_str[c3_i324];
      }

      c3_b_eml_warning(chartInstance, c3_b_str);
    }
  }

  if (guard2 == true) {
    guard1 = true;
  }

  if (guard1 == true) {
    c3_eml_warning(chartInstance);
  }
}

static void c3_realmin(SFc3_Robot_Control_v01InstanceStruct *chartInstance)
{
  (void)chartInstance;
}

static void c3_eps(SFc3_Robot_Control_v01InstanceStruct *chartInstance)
{
  c3_eml_eps(chartInstance);
}

static void c3_eml_switch_helper(SFc3_Robot_Control_v01InstanceStruct
  *chartInstance)
{
  (void)chartInstance;
}

static void c3_eml_matlab_zgetrf(SFc3_Robot_Control_v01InstanceStruct
  *chartInstance, real_T c3_A[16], real_T c3_b_A[16], int32_T c3_ipiv[4],
  int32_T *c3_info)
{
  int32_T c3_i325;
  for (c3_i325 = 0; c3_i325 < 16; c3_i325++) {
    c3_b_A[c3_i325] = c3_A[c3_i325];
  }

  c3_b_eml_matlab_zgetrf(chartInstance, c3_b_A, c3_ipiv, c3_info);
}

static void c3_threshold(SFc3_Robot_Control_v01InstanceStruct *chartInstance)
{
  (void)chartInstance;
}

static void c3_check_forloop_overflow_error(SFc3_Robot_Control_v01InstanceStruct
  *chartInstance, boolean_T c3_overflow)
{
  int32_T c3_i326;
  static char_T c3_cv3[34] = { 'C', 'o', 'd', 'e', 'r', ':', 't', 'o', 'o', 'l',
    'b', 'o', 'x', ':', 'i', 'n', 't', '_', 'f', 'o', 'r', 'l', 'o', 'o', 'p',
    '_', 'o', 'v', 'e', 'r', 'f', 'l', 'o', 'w' };

  char_T c3_u[34];
  const mxArray *c3_y = NULL;
  int32_T c3_i327;
  static char_T c3_cv4[5] = { 'i', 'n', 't', '3', '2' };

  char_T c3_b_u[5];
  const mxArray *c3_b_y = NULL;
  (void)chartInstance;
  if (!c3_overflow) {
  } else {
    for (c3_i326 = 0; c3_i326 < 34; c3_i326++) {
      c3_u[c3_i326] = c3_cv3[c3_i326];
    }

    c3_y = NULL;
    sf_mex_assign(&c3_y, sf_mex_create("y", c3_u, 10, 0U, 1U, 0U, 2, 1, 34),
                  false);
    for (c3_i327 = 0; c3_i327 < 5; c3_i327++) {
      c3_b_u[c3_i327] = c3_cv4[c3_i327];
    }

    c3_b_y = NULL;
    sf_mex_assign(&c3_b_y, sf_mex_create("y", c3_b_u, 10, 0U, 1U, 0U, 2, 1, 5),
                  false);
    sf_mex_call_debug(sfGlobalDebugInstanceStruct, "error", 0U, 1U, 14,
                      sf_mex_call_debug(sfGlobalDebugInstanceStruct, "message",
      1U, 2U, 14, c3_y, 14, c3_b_y));
  }
}

static void c3_b_eml_switch_helper(SFc3_Robot_Control_v01InstanceStruct
  *chartInstance)
{
  (void)chartInstance;
}

static void c3_eml_xgeru(SFc3_Robot_Control_v01InstanceStruct *chartInstance,
  int32_T c3_m, int32_T c3_n, real_T c3_alpha1, int32_T c3_ix0, int32_T c3_iy0,
  real_T c3_A[16], int32_T c3_ia0, real_T c3_b_A[16])
{
  int32_T c3_i328;
  for (c3_i328 = 0; c3_i328 < 16; c3_i328++) {
    c3_b_A[c3_i328] = c3_A[c3_i328];
  }

  c3_b_eml_xgeru(chartInstance, c3_m, c3_n, c3_alpha1, c3_ix0, c3_iy0, c3_b_A,
                 c3_ia0);
}

static void c3_b_threshold(SFc3_Robot_Control_v01InstanceStruct *chartInstance)
{
  (void)chartInstance;
}

static void c3_b_eml_scalar_eg(SFc3_Robot_Control_v01InstanceStruct
  *chartInstance)
{
  (void)chartInstance;
}

static void c3_eml_ipiv2perm(SFc3_Robot_Control_v01InstanceStruct *chartInstance,
  int32_T c3_ipiv[4], int32_T c3_perm[4])
{
  int32_T c3_i329;
  int32_T c3_k;
  real_T c3_b_k;
  int32_T c3_ipk;
  int32_T c3_a;
  real_T c3_b;
  int32_T c3_b_a;
  real_T c3_b_b;
  int32_T c3_idx;
  real_T c3_flt;
  boolean_T c3_p;
  int32_T c3_pipk;
  for (c3_i329 = 0; c3_i329 < 4; c3_i329++) {
    c3_perm[c3_i329] = 1 + c3_i329;
  }

  for (c3_k = 0; c3_k < 3; c3_k++) {
    c3_b_k = 1.0 + (real_T)c3_k;
    c3_ipk = c3_ipiv[_SFD_EML_ARRAY_BOUNDS_CHECK("", (int32_T)_SFD_INTEGER_CHECK
      ("", c3_b_k), 1, 4, 1, 0) - 1];
    c3_a = c3_ipk;
    c3_b = c3_b_k;
    c3_b_a = c3_a;
    c3_b_b = c3_b;
    c3_c_eml_switch_helper(chartInstance);
    c3_idx = c3_b_a;
    c3_flt = c3_b_b;
    c3_p = ((real_T)c3_idx > c3_flt);
    if (c3_p) {
      c3_pipk = c3_perm[_SFD_EML_ARRAY_BOUNDS_CHECK("", (int32_T)
        _SFD_INTEGER_CHECK("", (real_T)c3_ipk), 1, 4, 1, 0) - 1];
      c3_perm[_SFD_EML_ARRAY_BOUNDS_CHECK("", (int32_T)_SFD_INTEGER_CHECK("",
        (real_T)c3_ipk), 1, 4, 1, 0) - 1] = c3_perm[_SFD_EML_ARRAY_BOUNDS_CHECK(
        "", (int32_T)_SFD_INTEGER_CHECK("", c3_b_k), 1, 4, 1, 0) - 1];
      c3_perm[_SFD_EML_ARRAY_BOUNDS_CHECK("", (int32_T)_SFD_INTEGER_CHECK("",
        c3_b_k), 1, 4, 1, 0) - 1] = c3_pipk;
    }
  }
}

static void c3_c_eml_switch_helper(SFc3_Robot_Control_v01InstanceStruct
  *chartInstance)
{
  (void)chartInstance;
}

static void c3_eml_xtrsm(SFc3_Robot_Control_v01InstanceStruct *chartInstance,
  real_T c3_A[16], real_T c3_B[16], real_T c3_b_B[16])
{
  int32_T c3_i330;
  int32_T c3_i331;
  real_T c3_b_A[16];
  for (c3_i330 = 0; c3_i330 < 16; c3_i330++) {
    c3_b_B[c3_i330] = c3_B[c3_i330];
  }

  for (c3_i331 = 0; c3_i331 < 16; c3_i331++) {
    c3_b_A[c3_i331] = c3_A[c3_i331];
  }

  c3_b_eml_xtrsm(chartInstance, c3_b_A, c3_b_B);
}

static void c3_d_eml_switch_helper(SFc3_Robot_Control_v01InstanceStruct
  *chartInstance)
{
  (void)chartInstance;
}

static real_T c3_rdivide(SFc3_Robot_Control_v01InstanceStruct *chartInstance,
  real_T c3_x, real_T c3_y)
{
  real_T c3_b_x;
  real_T c3_b_y;
  real_T c3_c_x;
  real_T c3_c_y;
  (void)chartInstance;
  c3_b_x = c3_x;
  c3_b_y = c3_y;
  c3_c_x = c3_b_x;
  c3_c_y = c3_b_y;
  return c3_c_x / c3_c_y;
}

static real_T c3_norm(SFc3_Robot_Control_v01InstanceStruct *chartInstance,
                      real_T c3_x[16])
{
  real_T c3_y;
  int32_T c3_j;
  real_T c3_b_j;
  real_T c3_s;
  int32_T c3_i;
  real_T c3_b_i;
  real_T c3_b_x;
  real_T c3_c_x;
  real_T c3_b_y;
  real_T c3_d_x;
  boolean_T c3_b;
  boolean_T exitg1;
  (void)chartInstance;
  c3_y = 0.0;
  c3_j = 0;
  exitg1 = false;
  while ((exitg1 == false) && (c3_j < 4)) {
    c3_b_j = 1.0 + (real_T)c3_j;
    c3_s = 0.0;
    for (c3_i = 0; c3_i < 4; c3_i++) {
      c3_b_i = 1.0 + (real_T)c3_i;
      c3_b_x = c3_x[(_SFD_EML_ARRAY_BOUNDS_CHECK("", (int32_T)_SFD_INTEGER_CHECK
                      ("", c3_b_i), 1, 4, 1, 0) + ((_SFD_EML_ARRAY_BOUNDS_CHECK(
        "", (int32_T)_SFD_INTEGER_CHECK("", c3_b_j), 1, 4, 2, 0) - 1) << 2)) - 1];
      c3_c_x = c3_b_x;
      c3_b_y = muDoubleScalarAbs(c3_c_x);
      c3_s += c3_b_y;
    }

    c3_d_x = c3_s;
    c3_b = muDoubleScalarIsNaN(c3_d_x);
    if (c3_b) {
      c3_y = rtNaN;
      exitg1 = true;
    } else {
      if (c3_s > c3_y) {
        c3_y = c3_s;
      }

      c3_j++;
    }
  }

  return c3_y;
}

static void c3_eml_warning(SFc3_Robot_Control_v01InstanceStruct *chartInstance)
{
  int32_T c3_i332;
  static char_T c3_varargin_1[27] = { 'C', 'o', 'd', 'e', 'r', ':', 'M', 'A',
    'T', 'L', 'A', 'B', ':', 's', 'i', 'n', 'g', 'u', 'l', 'a', 'r', 'M', 'a',
    't', 'r', 'i', 'x' };

  char_T c3_u[27];
  const mxArray *c3_y = NULL;
  (void)chartInstance;
  for (c3_i332 = 0; c3_i332 < 27; c3_i332++) {
    c3_u[c3_i332] = c3_varargin_1[c3_i332];
  }

  c3_y = NULL;
  sf_mex_assign(&c3_y, sf_mex_create("y", c3_u, 10, 0U, 1U, 0U, 2, 1, 27), false);
  sf_mex_call_debug(sfGlobalDebugInstanceStruct, "warning", 0U, 1U, 14,
                    sf_mex_call_debug(sfGlobalDebugInstanceStruct, "message", 1U,
    1U, 14, c3_y));
}

static void c3_b_eml_warning(SFc3_Robot_Control_v01InstanceStruct *chartInstance,
  char_T c3_varargin_2[14])
{
  int32_T c3_i333;
  static char_T c3_varargin_1[33] = { 'C', 'o', 'd', 'e', 'r', ':', 'M', 'A',
    'T', 'L', 'A', 'B', ':', 'i', 'l', 'l', 'C', 'o', 'n', 'd', 'i', 't', 'i',
    'o', 'n', 'e', 'd', 'M', 'a', 't', 'r', 'i', 'x' };

  char_T c3_u[33];
  const mxArray *c3_y = NULL;
  int32_T c3_i334;
  char_T c3_b_u[14];
  const mxArray *c3_b_y = NULL;
  (void)chartInstance;
  for (c3_i333 = 0; c3_i333 < 33; c3_i333++) {
    c3_u[c3_i333] = c3_varargin_1[c3_i333];
  }

  c3_y = NULL;
  sf_mex_assign(&c3_y, sf_mex_create("y", c3_u, 10, 0U, 1U, 0U, 2, 1, 33), false);
  for (c3_i334 = 0; c3_i334 < 14; c3_i334++) {
    c3_b_u[c3_i334] = c3_varargin_2[c3_i334];
  }

  c3_b_y = NULL;
  sf_mex_assign(&c3_b_y, sf_mex_create("y", c3_b_u, 10, 0U, 1U, 0U, 2, 1, 14),
                false);
  sf_mex_call_debug(sfGlobalDebugInstanceStruct, "warning", 0U, 1U, 14,
                    sf_mex_call_debug(sfGlobalDebugInstanceStruct, "message", 1U,
    2U, 14, c3_y, 14, c3_b_y));
}

static void c3_setupPointsAndPixels(SFc3_Robot_Control_v01InstanceStruct
  *chartInstance, c3_sKe46gf3wfNeOrzL8yzt6nF c3_pose, c3_sKe46gf3wfNeOrzL8yzt6nF
  c3_robotPoseDes, real_T c3_P[16], real_T c3_f, real_T c3_pp[2], real_T c3_L_p
  [8], real_T c3_R_p[8], real_T c3_S_PDes[16], c3_smiE7r3wwCYeiytRpYXSxxE
  *c3_robot)
{
  uint32_T c3_debug_family_var_map[16];
  c3_sKe46gf3wfNeOrzL8yzt6nF c3_leftCamPose;
  c3_sKe46gf3wfNeOrzL8yzt6nF c3_rightCamPose;
  c3_s6d9wPYgmS3cwGRsMJd1GAE c3_leftCamera;
  c3_s6d9wPYgmS3cwGRsMJd1GAE c3_rightCamera;
  real_T c3_world2robotDes[16];
  real_T c3_nargin = 5.0;
  real_T c3_nargout = 4.0;
  static c3_sKe46gf3wfNeOrzL8yzt6nF c3_r2 = { 2.0, 1.0, 1.0, 0.0, 0.0, 0.0 };

  static c3_sKe46gf3wfNeOrzL8yzt6nF c3_r3 = { 2.0, -1.0, 1.0, 0.0, 0.0, 0.0 };

  c3_s6d9wPYgmS3cwGRsMJd1GAE c3_r4;
  c3_s6d9wPYgmS3cwGRsMJd1GAE c3_r5;
  c3_sKe46gf3wfNeOrzL8yzt6nF c3_b_pose;
  c3_s6d9wPYgmS3cwGRsMJd1GAE c3_b_leftCamera;
  c3_s6d9wPYgmS3cwGRsMJd1GAE c3_b_rightCamera;
  uint32_T c3_b_debug_family_var_map[6];
  real_T c3_b_nargin = 3.0;
  real_T c3_b_nargout = 1.0;
  real_T c3_dv33[16];
  int32_T c3_i335;
  real_T c3_dv34[16];
  real_T c3_dv35[16];
  int32_T c3_i336;
  int32_T c3_i337;
  real_T c3_a[16];
  int32_T c3_i338;
  real_T c3_b[16];
  int32_T c3_i339;
  int32_T c3_i340;
  int32_T c3_i341;
  real_T c3_b_a[16];
  int32_T c3_i342;
  real_T c3_b_b[16];
  int32_T c3_i343;
  int32_T c3_i344;
  real_T c3_b_P[16];
  c3_smiE7r3wwCYeiytRpYXSxxE c3_b_robot;
  uint32_T c3_c_debug_family_var_map[10];
  real_T c3_world2robot[16];
  real_T c3_S_P[16];
  c3_sZiCA83jGK4zdmds7wvlXUF c3_L_retVal;
  c3_sZiCA83jGK4zdmds7wvlXUF c3_R_retVal;
  real_T c3_c_nargin = 2.0;
  real_T c3_c_nargout = 2.0;
  real_T c3_b_L_p[8];
  real_T c3_b_R_p[8];
  int32_T c3_i345;
  real_T c3_dv36[16];
  real_T c3_dv37[16];
  int32_T c3_i346;
  int32_T c3_i347;
  int32_T c3_i348;
  int32_T c3_i349;
  int32_T c3_i350;
  int32_T c3_i351;
  real_T c3_c_a[16];
  int32_T c3_i352;
  real_T c3_c_b[16];
  int32_T c3_i353;
  int32_T c3_i354;
  real_T c3_b_S_P[16];
  c3_s6d9wPYgmS3cwGRsMJd1GAE c3_c_robot;
  c3_sZiCA83jGK4zdmds7wvlXUF c3_r6;
  int32_T c3_i355;
  real_T c3_c_S_P[16];
  c3_s6d9wPYgmS3cwGRsMJd1GAE c3_d_robot;
  c3_sZiCA83jGK4zdmds7wvlXUF c3_r7;
  int32_T c3_i356;
  int32_T c3_i357;
  int32_T c3_i358;
  int32_T c3_i359;
  _SFD_SYMBOL_SCOPE_PUSH_EML(0U, 16U, 16U, c3_k_debug_family_names,
    c3_debug_family_var_map);
  _SFD_SYMBOL_SCOPE_ADD_EML(&c3_leftCamPose, 0U, c3_g_sf_marshallOut);
  _SFD_SYMBOL_SCOPE_ADD_EML(&c3_rightCamPose, 1U, c3_g_sf_marshallOut);
  _SFD_SYMBOL_SCOPE_ADD_EML_IMPORTABLE(&c3_leftCamera, 2U, c3_l_sf_marshallOut,
    c3_k_sf_marshallIn);
  _SFD_SYMBOL_SCOPE_ADD_EML_IMPORTABLE(&c3_rightCamera, 3U, c3_l_sf_marshallOut,
    c3_k_sf_marshallIn);
  _SFD_SYMBOL_SCOPE_ADD_EML_IMPORTABLE(c3_world2robotDes, 4U,
    c3_c_sf_marshallOut, c3_c_sf_marshallIn);
  _SFD_SYMBOL_SCOPE_ADD_EML_IMPORTABLE(&c3_nargin, 5U, c3_b_sf_marshallOut,
    c3_b_sf_marshallIn);
  _SFD_SYMBOL_SCOPE_ADD_EML_IMPORTABLE(&c3_nargout, 6U, c3_b_sf_marshallOut,
    c3_b_sf_marshallIn);
  _SFD_SYMBOL_SCOPE_ADD_EML_IMPORTABLE(&c3_pose, 7U, c3_g_sf_marshallOut,
    c3_g_sf_marshallIn);
  _SFD_SYMBOL_SCOPE_ADD_EML_IMPORTABLE(&c3_robotPoseDes, 8U, c3_g_sf_marshallOut,
    c3_g_sf_marshallIn);
  _SFD_SYMBOL_SCOPE_ADD_EML_IMPORTABLE(c3_P, 9U, c3_c_sf_marshallOut,
    c3_c_sf_marshallIn);
  _SFD_SYMBOL_SCOPE_ADD_EML_IMPORTABLE(&c3_f, 10U, c3_b_sf_marshallOut,
    c3_b_sf_marshallIn);
  _SFD_SYMBOL_SCOPE_ADD_EML_IMPORTABLE(c3_pp, 11U, c3_i_sf_marshallOut,
    c3_i_sf_marshallIn);
  _SFD_SYMBOL_SCOPE_ADD_EML_IMPORTABLE(c3_L_p, 12U, c3_f_sf_marshallOut,
    c3_f_sf_marshallIn);
  _SFD_SYMBOL_SCOPE_ADD_EML_IMPORTABLE(c3_R_p, 13U, c3_f_sf_marshallOut,
    c3_f_sf_marshallIn);
  _SFD_SYMBOL_SCOPE_ADD_EML_IMPORTABLE(c3_S_PDes, 14U, c3_c_sf_marshallOut,
    c3_c_sf_marshallIn);
  _SFD_SYMBOL_SCOPE_ADD_EML_IMPORTABLE(c3_robot, 15U, c3_e_sf_marshallOut,
    c3_e_sf_marshallIn);
  CV_EML_FCN(0, 1);
  _SFD_EML_CALL(0U, chartInstance->c3_sfEvent, 63);
  c3_leftCamPose = c3_r2;
  _SFD_EML_CALL(0U, chartInstance->c3_sfEvent, 64);
  c3_rightCamPose = c3_r3;
  _SFD_EML_CALL(0U, chartInstance->c3_sfEvent, 67);
  c3_newCamera(chartInstance, c3_r2, c3_f, c3_f, c3_pp[1], c3_pp[0], 0.0, 0.0,
               &c3_r4);
  c3_leftCamera = c3_r4;
  _SFD_EML_CALL(0U, chartInstance->c3_sfEvent, 68);
  c3_newCamera(chartInstance, c3_r3, c3_f, c3_f, c3_pp[1], c3_pp[0], 0.0, 0.0,
               &c3_r5);
  c3_rightCamera = c3_r5;
  _SFD_EML_CALL(0U, chartInstance->c3_sfEvent, 71);
  c3_b_pose = c3_pose;
  c3_b_leftCamera = c3_leftCamera;
  c3_b_rightCamera = c3_rightCamera;
  _SFD_SYMBOL_SCOPE_PUSH_EML(0U, 6U, 6U, c3_g_debug_family_names,
    c3_b_debug_family_var_map);
  _SFD_SYMBOL_SCOPE_ADD_EML_IMPORTABLE(&c3_b_nargin, 0U, c3_b_sf_marshallOut,
    c3_b_sf_marshallIn);
  _SFD_SYMBOL_SCOPE_ADD_EML_IMPORTABLE(&c3_b_nargout, 1U, c3_b_sf_marshallOut,
    c3_b_sf_marshallIn);
  _SFD_SYMBOL_SCOPE_ADD_EML_IMPORTABLE(&c3_b_pose, 2U, c3_g_sf_marshallOut,
    c3_g_sf_marshallIn);
  _SFD_SYMBOL_SCOPE_ADD_EML_IMPORTABLE(&c3_b_leftCamera, 3U, c3_l_sf_marshallOut,
    c3_k_sf_marshallIn);
  _SFD_SYMBOL_SCOPE_ADD_EML_IMPORTABLE(&c3_b_rightCamera, 4U,
    c3_l_sf_marshallOut, c3_k_sf_marshallIn);
  _SFD_SYMBOL_SCOPE_ADD_EML_IMPORTABLE(c3_robot, 5U, c3_e_sf_marshallOut,
    c3_e_sf_marshallIn);
  CV_SCRIPT_FCN(5, 0);
  _SFD_SCRIPT_CALL(5U, chartInstance->c3_sfEvent, 7);
  c3_robot->pose = c3_b_pose;
  _SFD_SCRIPT_CALL(5U, chartInstance->c3_sfEvent, 8);
  c3_robot->leftCamera = c3_b_leftCamera;
  _SFD_SCRIPT_CALL(5U, chartInstance->c3_sfEvent, 9);
  c3_robot->rightCamera = c3_b_rightCamera;
  _SFD_SCRIPT_CALL(5U, chartInstance->c3_sfEvent, -9);
  _SFD_SYMBOL_SCOPE_POP();
  _SFD_EML_CALL(0U, chartInstance->c3_sfEvent, 73);
  c3_transformationMatrix(chartInstance, c3_robotPoseDes, c3_dv33);
  for (c3_i335 = 0; c3_i335 < 16; c3_i335++) {
    c3_dv34[c3_i335] = c3_dv33[c3_i335];
  }

  c3_inv(chartInstance, c3_dv34, c3_dv35);
  for (c3_i336 = 0; c3_i336 < 16; c3_i336++) {
    c3_world2robotDes[c3_i336] = c3_dv35[c3_i336];
  }

  _SFD_EML_CALL(0U, chartInstance->c3_sfEvent, 74);
  for (c3_i337 = 0; c3_i337 < 16; c3_i337++) {
    c3_a[c3_i337] = c3_world2robotDes[c3_i337];
  }

  for (c3_i338 = 0; c3_i338 < 16; c3_i338++) {
    c3_b[c3_i338] = c3_P[c3_i338];
  }

  c3_eml_scalar_eg(chartInstance);
  c3_eml_scalar_eg(chartInstance);
  for (c3_i339 = 0; c3_i339 < 16; c3_i339++) {
    c3_S_PDes[c3_i339] = 0.0;
  }

  for (c3_i340 = 0; c3_i340 < 16; c3_i340++) {
    c3_dv33[c3_i340] = 0.0;
  }

  for (c3_i341 = 0; c3_i341 < 16; c3_i341++) {
    c3_b_a[c3_i341] = c3_a[c3_i341];
  }

  for (c3_i342 = 0; c3_i342 < 16; c3_i342++) {
    c3_b_b[c3_i342] = c3_b[c3_i342];
  }

  c3_b_eml_xgemm(chartInstance, c3_b_a, c3_b_b, c3_dv33);
  for (c3_i343 = 0; c3_i343 < 16; c3_i343++) {
    c3_S_PDes[c3_i343] = c3_dv33[c3_i343];
  }

  _SFD_EML_CALL(0U, chartInstance->c3_sfEvent, 76);
  for (c3_i344 = 0; c3_i344 < 16; c3_i344++) {
    c3_b_P[c3_i344] = c3_P[c3_i344];
  }

  c3_b_robot = *c3_robot;
  _SFD_SYMBOL_SCOPE_PUSH_EML(0U, 10U, 10U, c3_j_debug_family_names,
    c3_c_debug_family_var_map);
  _SFD_SYMBOL_SCOPE_ADD_EML_IMPORTABLE(c3_world2robot, 0U, c3_c_sf_marshallOut,
    c3_c_sf_marshallIn);
  _SFD_SYMBOL_SCOPE_ADD_EML_IMPORTABLE(c3_S_P, 1U, c3_c_sf_marshallOut,
    c3_c_sf_marshallIn);
  _SFD_SYMBOL_SCOPE_ADD_EML_IMPORTABLE(&c3_L_retVal, 2U, c3_m_sf_marshallOut,
    c3_l_sf_marshallIn);
  _SFD_SYMBOL_SCOPE_ADD_EML_IMPORTABLE(&c3_R_retVal, 3U, c3_m_sf_marshallOut,
    c3_l_sf_marshallIn);
  _SFD_SYMBOL_SCOPE_ADD_EML_IMPORTABLE(&c3_c_nargin, 4U, c3_b_sf_marshallOut,
    c3_b_sf_marshallIn);
  _SFD_SYMBOL_SCOPE_ADD_EML_IMPORTABLE(&c3_c_nargout, 5U, c3_b_sf_marshallOut,
    c3_b_sf_marshallIn);
  _SFD_SYMBOL_SCOPE_ADD_EML_IMPORTABLE(c3_b_P, 6U, c3_c_sf_marshallOut,
    c3_c_sf_marshallIn);
  _SFD_SYMBOL_SCOPE_ADD_EML_IMPORTABLE(&c3_b_robot, 7U, c3_e_sf_marshallOut,
    c3_e_sf_marshallIn);
  _SFD_SYMBOL_SCOPE_ADD_EML_IMPORTABLE(c3_b_L_p, 8U, c3_f_sf_marshallOut,
    c3_f_sf_marshallIn);
  _SFD_SYMBOL_SCOPE_ADD_EML_IMPORTABLE(c3_b_R_p, 9U, c3_f_sf_marshallOut,
    c3_f_sf_marshallIn);
  CV_SCRIPT_FCN(6, 0);
  _SFD_SCRIPT_CALL(6U, chartInstance->c3_sfEvent, 8);
  c3_transformationMatrix(chartInstance, c3_b_robot.pose, c3_dv33);
  for (c3_i345 = 0; c3_i345 < 16; c3_i345++) {
    c3_dv36[c3_i345] = c3_dv33[c3_i345];
  }

  c3_inv(chartInstance, c3_dv36, c3_dv37);
  for (c3_i346 = 0; c3_i346 < 16; c3_i346++) {
    c3_world2robot[c3_i346] = c3_dv37[c3_i346];
  }

  _SFD_SCRIPT_CALL(6U, chartInstance->c3_sfEvent, 9);
  for (c3_i347 = 0; c3_i347 < 16; c3_i347++) {
    c3_a[c3_i347] = c3_world2robot[c3_i347];
  }

  for (c3_i348 = 0; c3_i348 < 16; c3_i348++) {
    c3_b[c3_i348] = c3_b_P[c3_i348];
  }

  c3_eml_scalar_eg(chartInstance);
  c3_eml_scalar_eg(chartInstance);
  for (c3_i349 = 0; c3_i349 < 16; c3_i349++) {
    c3_S_P[c3_i349] = 0.0;
  }

  for (c3_i350 = 0; c3_i350 < 16; c3_i350++) {
    c3_dv33[c3_i350] = 0.0;
  }

  for (c3_i351 = 0; c3_i351 < 16; c3_i351++) {
    c3_c_a[c3_i351] = c3_a[c3_i351];
  }

  for (c3_i352 = 0; c3_i352 < 16; c3_i352++) {
    c3_c_b[c3_i352] = c3_b[c3_i352];
  }

  c3_b_eml_xgemm(chartInstance, c3_c_a, c3_c_b, c3_dv33);
  for (c3_i353 = 0; c3_i353 < 16; c3_i353++) {
    c3_S_P[c3_i353] = c3_dv33[c3_i353];
  }

  _SFD_SCRIPT_CALL(6U, chartInstance->c3_sfEvent, 12);
  for (c3_i354 = 0; c3_i354 < 16; c3_i354++) {
    c3_b_S_P[c3_i354] = c3_S_P[c3_i354];
  }

  c3_c_robot = c3_b_robot.leftCamera;
  c3_transformImageSpace(chartInstance, c3_b_S_P, &c3_c_robot, &c3_r6);
  c3_L_retVal = c3_r6;
  _SFD_SCRIPT_CALL(6U, chartInstance->c3_sfEvent, 13);
  for (c3_i355 = 0; c3_i355 < 16; c3_i355++) {
    c3_c_S_P[c3_i355] = c3_S_P[c3_i355];
  }

  c3_d_robot = c3_b_robot.rightCamera;
  c3_transformImageSpace(chartInstance, c3_c_S_P, &c3_d_robot, &c3_r7);
  c3_R_retVal = c3_r7;
  _SFD_SCRIPT_CALL(6U, chartInstance->c3_sfEvent, 15);
  for (c3_i356 = 0; c3_i356 < 8; c3_i356++) {
    c3_b_L_p[c3_i356] = c3_L_retVal.p[c3_i356];
  }

  _SFD_SCRIPT_CALL(6U, chartInstance->c3_sfEvent, 16);
  for (c3_i357 = 0; c3_i357 < 8; c3_i357++) {
    c3_b_R_p[c3_i357] = c3_R_retVal.p[c3_i357];
  }

  _SFD_SCRIPT_CALL(6U, chartInstance->c3_sfEvent, -16);
  _SFD_SYMBOL_SCOPE_POP();
  for (c3_i358 = 0; c3_i358 < 8; c3_i358++) {
    c3_L_p[c3_i358] = c3_b_L_p[c3_i358];
  }

  for (c3_i359 = 0; c3_i359 < 8; c3_i359++) {
    c3_R_p[c3_i359] = c3_b_R_p[c3_i359];
  }

  _SFD_EML_CALL(0U, chartInstance->c3_sfEvent, -76);
  _SFD_SYMBOL_SCOPE_POP();
}

static void c3_b_rdivide(SFc3_Robot_Control_v01InstanceStruct *chartInstance,
  real_T c3_x[4], real_T c3_y[4], real_T c3_z[4])
{
  int32_T c3_i360;
  (void)chartInstance;
  for (c3_i360 = 0; c3_i360 < 4; c3_i360++) {
    c3_z[c3_i360] = c3_x[c3_i360] / c3_y[c3_i360];
  }
}

static void c3_mldivide(SFc3_Robot_Control_v01InstanceStruct *chartInstance,
  real_T c3_A[192], real_T c3_B[16], real_T c3_Y[12])
{
  int32_T c3_i361;
  real_T c3_b_A[192];
  int32_T c3_i362;
  real_T c3_b_B[16];
  int32_T c3_i363;
  int32_T c3_jpvt[12];
  int32_T c3_i364;
  real_T c3_work[12];
  real_T c3_TOL3Z;
  int32_T c3_k;
  int32_T c3_j;
  int32_T c3_b_j;
  int32_T c3_i365;
  real_T c3_c_A[192];
  real_T c3_vn1[12];
  real_T c3_vn2[12];
  int32_T c3_a;
  int32_T c3_b_a;
  int32_T c3_i;
  int32_T c3_b_i;
  int32_T c3_c_a;
  int32_T c3_d_a;
  int32_T c3_im1;
  int32_T c3_e_a;
  int32_T c3_f_a;
  int32_T c3_ip1;
  int32_T c3_g_a;
  int32_T c3_h_a;
  int32_T c3_c;
  int32_T c3_i_a;
  int32_T c3_b;
  int32_T c3_j_a;
  int32_T c3_b_b;
  int32_T c3_i_i;
  int32_T c3_c_b;
  int32_T c3_d_b;
  int32_T c3_nmi;
  int32_T c3_e_b;
  int32_T c3_f_b;
  int32_T c3_mmi;
  int32_T c3_g_b;
  int32_T c3_h_b;
  int32_T c3_mmip1;
  int32_T c3_i_b;
  int32_T c3_j_b;
  int32_T c3_nmip1;
  int32_T c3_k_a;
  int32_T c3_i366;
  real_T c3_b_vn1[12];
  int32_T c3_k_b;
  int32_T c3_l_a;
  int32_T c3_l_b;
  int32_T c3_pvt;
  int32_T c3_m_a;
  int32_T c3_n_a;
  int32_T c3_b_c;
  int32_T c3_m_b;
  int32_T c3_n_b;
  int32_T c3_c_c;
  int32_T c3_o_b;
  int32_T c3_p_b;
  int32_T c3_pvtcol;
  int32_T c3_q_b;
  int32_T c3_r_b;
  int32_T c3_d_c;
  int32_T c3_s_b;
  int32_T c3_t_b;
  int32_T c3_mcol;
  int32_T c3_itemp;
  real_T c3_atmp;
  int32_T c3_o_a;
  int32_T c3_p_a;
  int32_T c3_e_c;
  real_T c3_b_atmp;
  real_T c3_d4;
  real_T c3_tau[12];
  real_T c3_c_atmp;
  real_T c3_d5;
  real_T c3_d6;
  int32_T c3_q_a;
  int32_T c3_r_a;
  int32_T c3_f_c;
  int32_T c3_s_a;
  int32_T c3_u_b;
  int32_T c3_t_a;
  int32_T c3_v_b;
  int32_T c3_i_ip1;
  int32_T c3_b_ip1;
  int32_T c3_u_a;
  int32_T c3_v_a;
  boolean_T c3_overflow;
  int32_T c3_c_j;
  int32_T c3_w_a;
  int32_T c3_x_a;
  int32_T c3_g_c;
  int32_T c3_w_b;
  int32_T c3_x_b;
  int32_T c3_h_c;
  int32_T c3_y_a;
  int32_T c3_y_b;
  int32_T c3_ab_a;
  int32_T c3_ab_b;
  int32_T c3_i_j;
  real_T c3_x;
  real_T c3_b_x;
  real_T c3_y;
  real_T c3_temp1;
  real_T c3_temp2;
  int32_T c3_bb_a;
  int32_T c3_cb_a;
  int32_T c3_i_c;
  int32_T c3_n;
  int32_T c3_ix0;
  int32_T c3_b_n;
  int32_T c3_b_ix0;
  int32_T c3_c_n;
  int32_T c3_c_ix0;
  real_T c3_b_y;
  real_T c3_c_x;
  real_T c3_d_x;
  real_T c3_scale;
  int32_T c3_kstart;
  int32_T c3_db_a;
  int32_T c3_j_c;
  int32_T c3_eb_a;
  int32_T c3_k_c;
  int32_T c3_fb_a;
  int32_T c3_bb_b;
  int32_T c3_kend;
  int32_T c3_b_kstart;
  int32_T c3_b_kend;
  int32_T c3_gb_a;
  int32_T c3_cb_b;
  int32_T c3_hb_a;
  int32_T c3_db_b;
  boolean_T c3_b_overflow;
  int32_T c3_b_k;
  int32_T c3_c_k;
  real_T c3_e_x;
  real_T c3_f_x;
  real_T c3_absxk;
  real_T c3_t;
  real_T c3_g_x;
  real_T c3_h_x;
  real_T c3_rankR;
  real_T c3_i_x;
  real_T c3_j_x;
  real_T c3_k_x;
  real_T c3_l_x;
  real_T c3_c_y;
  real_T c3_m_x;
  real_T c3_n_x;
  real_T c3_d_y;
  real_T c3_d;
  real_T c3_tol;
  int32_T c3_d_k;
  real_T c3_e_k;
  real_T c3_o_x;
  real_T c3_p_x;
  real_T c3_q_x;
  real_T c3_r_x;
  real_T c3_e_y;
  real_T c3_s_x;
  real_T c3_t_x;
  real_T c3_f_y;
  real_T c3_b_d;
  real_T c3_u_x;
  int32_T c3_i367;
  static char_T c3_cv5[8] = { '%', '%', '%', 'd', '.', '%', 'd', 'e' };

  char_T c3_u[8];
  const mxArray *c3_g_y = NULL;
  real_T c3_b_u;
  const mxArray *c3_h_y = NULL;
  real_T c3_c_u;
  const mxArray *c3_i_y = NULL;
  real_T c3_d_u;
  const mxArray *c3_j_y = NULL;
  char_T c3_str[14];
  int32_T c3_i368;
  char_T c3_b_str[14];
  int32_T c3_i369;
  int32_T c3_d_j;
  real_T c3_e_j;
  real_T c3_tauj;
  real_T c3_wj;
  real_T c3_d7;
  int32_T c3_i370;
  int32_T c3_c_i;
  real_T c3_d_i;
  real_T c3_ib_a;
  real_T c3_eb_b;
  real_T c3_z;
  real_T c3_d8;
  int32_T c3_i371;
  int32_T c3_e_i;
  real_T c3_rr;
  real_T c3_b_rr;
  int32_T c3_i372;
  int32_T c3_f_i;
  real_T c3_c_rr;
  int32_T c3_i373;
  int32_T c3_f_j;
  int32_T c3_pj;
  real_T c3_v_x;
  real_T c3_k_y;
  real_T c3_w_x;
  real_T c3_l_y;
  real_T c3_b_z;
  real_T c3_d9;
  int32_T c3_i374;
  int32_T c3_g_i;
  boolean_T exitg1;
  for (c3_i361 = 0; c3_i361 < 192; c3_i361++) {
    c3_b_A[c3_i361] = c3_A[c3_i361];
  }

  for (c3_i362 = 0; c3_i362 < 16; c3_i362++) {
    c3_b_B[c3_i362] = c3_B[c3_i362];
  }

  c3_c_eml_scalar_eg(chartInstance);
  c3_eml_switch_helper(chartInstance);
  c3_eml_switch_helper(chartInstance);
  for (c3_i363 = 0; c3_i363 < 12; c3_i363++) {
    c3_jpvt[c3_i363] = 1 + c3_i363;
  }

  c3_c_eml_scalar_eg(chartInstance);
  for (c3_i364 = 0; c3_i364 < 12; c3_i364++) {
    c3_work[c3_i364] = 0.0;
  }

  c3_eps(chartInstance);
  c3_TOL3Z = 2.2204460492503131E-16;
  c3_b_sqrt(chartInstance, &c3_TOL3Z);
  c3_c_eml_scalar_eg(chartInstance);
  c3_k = 1;
  c3_eml_switch_helper(chartInstance);
  for (c3_j = 1; c3_j < 13; c3_j++) {
    c3_b_j = c3_j;
    for (c3_i365 = 0; c3_i365 < 192; c3_i365++) {
      c3_c_A[c3_i365] = c3_b_A[c3_i365];
    }

    c3_vn1[_SFD_EML_ARRAY_BOUNDS_CHECK("", (int32_T)_SFD_INTEGER_CHECK("",
      (real_T)c3_b_j), 1, 12, 1, 0) - 1] = c3_eml_xnrm2(chartInstance, c3_c_A,
      c3_k);
    c3_vn2[_SFD_EML_ARRAY_BOUNDS_CHECK("", (int32_T)_SFD_INTEGER_CHECK("",
      (real_T)c3_b_j), 1, 12, 1, 0) - 1] = c3_vn1[_SFD_EML_ARRAY_BOUNDS_CHECK("",
      (int32_T)_SFD_INTEGER_CHECK("", (real_T)c3_b_j), 1, 12, 1, 0) - 1];
    c3_a = c3_k;
    c3_b_a = c3_a + 16;
    c3_k = c3_b_a;
  }

  c3_eml_switch_helper(chartInstance);
  for (c3_i = 1; c3_i < 13; c3_i++) {
    c3_b_i = c3_i;
    c3_c_a = c3_b_i;
    c3_d_a = c3_c_a - 1;
    c3_im1 = c3_d_a;
    c3_e_a = c3_b_i;
    c3_f_a = c3_e_a;
    c3_ip1 = c3_f_a;
    c3_g_a = c3_im1;
    c3_h_a = c3_g_a;
    c3_c = c3_h_a << 4;
    c3_i_a = c3_b_i;
    c3_b = c3_c;
    c3_j_a = c3_i_a;
    c3_b_b = c3_b;
    c3_i_i = c3_j_a + c3_b_b;
    c3_c_b = c3_b_i;
    c3_d_b = c3_c_b;
    c3_nmi = 12 - c3_d_b;
    c3_e_b = c3_b_i;
    c3_f_b = c3_e_b;
    c3_mmi = 16 - c3_f_b;
    c3_g_b = c3_mmi;
    c3_h_b = c3_g_b + 1;
    c3_mmip1 = c3_h_b;
    c3_i_b = c3_nmi;
    c3_j_b = c3_i_b;
    c3_nmip1 = c3_j_b;
    c3_k_a = c3_im1;
    for (c3_i366 = 0; c3_i366 < 12; c3_i366++) {
      c3_b_vn1[c3_i366] = c3_vn1[c3_i366];
    }

    c3_k_b = c3_eml_ixamax(chartInstance, c3_nmip1 + 1, c3_b_vn1, c3_b_i);
    c3_l_a = c3_k_a;
    c3_l_b = c3_k_b;
    c3_pvt = c3_l_a + c3_l_b;
    if (c3_pvt != c3_b_i) {
      c3_m_a = c3_pvt;
      c3_n_a = c3_m_a;
      c3_b_c = c3_n_a;
      c3_m_b = c3_b_c - 1;
      c3_n_b = c3_m_b;
      c3_c_c = c3_n_b << 4;
      c3_o_b = c3_c_c;
      c3_p_b = c3_o_b;
      c3_pvtcol = c3_p_b;
      c3_q_b = c3_im1;
      c3_r_b = c3_q_b;
      c3_d_c = c3_r_b << 4;
      c3_s_b = c3_d_c;
      c3_t_b = c3_s_b;
      c3_mcol = c3_t_b;
      c3_b_eml_xswap(chartInstance, c3_b_A, c3_pvtcol + 1, c3_mcol + 1);
      c3_itemp = c3_jpvt[_SFD_EML_ARRAY_BOUNDS_CHECK("", (int32_T)
        _SFD_INTEGER_CHECK("", (real_T)c3_pvt), 1, 12, 1, 0) - 1];
      c3_jpvt[_SFD_EML_ARRAY_BOUNDS_CHECK("", (int32_T)_SFD_INTEGER_CHECK("",
        (real_T)c3_pvt), 1, 12, 1, 0) - 1] = c3_jpvt[_SFD_EML_ARRAY_BOUNDS_CHECK
        ("", (int32_T)_SFD_INTEGER_CHECK("", (real_T)c3_b_i), 1, 12, 1, 0) - 1];
      c3_jpvt[_SFD_EML_ARRAY_BOUNDS_CHECK("", (int32_T)_SFD_INTEGER_CHECK("",
        (real_T)c3_b_i), 1, 12, 1, 0) - 1] = c3_itemp;
      c3_vn1[_SFD_EML_ARRAY_BOUNDS_CHECK("", (int32_T)_SFD_INTEGER_CHECK("",
        (real_T)c3_pvt), 1, 12, 1, 0) - 1] = c3_vn1[_SFD_EML_ARRAY_BOUNDS_CHECK(
        "", (int32_T)_SFD_INTEGER_CHECK("", (real_T)c3_b_i), 1, 12, 1, 0) - 1];
      c3_vn2[_SFD_EML_ARRAY_BOUNDS_CHECK("", (int32_T)_SFD_INTEGER_CHECK("",
        (real_T)c3_pvt), 1, 12, 1, 0) - 1] = c3_vn2[_SFD_EML_ARRAY_BOUNDS_CHECK(
        "", (int32_T)_SFD_INTEGER_CHECK("", (real_T)c3_b_i), 1, 12, 1, 0) - 1];
    }

    c3_atmp = c3_b_A[_SFD_EML_ARRAY_BOUNDS_CHECK("", (int32_T)_SFD_INTEGER_CHECK
      ("", (real_T)c3_i_i), 1, 192, 1, 0) - 1];
    if (c3_b_i < 16) {
      c3_o_a = c3_i_i;
      c3_p_a = c3_o_a;
      c3_e_c = c3_p_a;
      c3_b_atmp = c3_atmp;
      c3_d4 = c3_c_eml_matlab_zlarfg(chartInstance, c3_mmip1, &c3_b_atmp, c3_b_A,
        c3_e_c + 1);
      c3_atmp = c3_b_atmp;
      c3_tau[_SFD_EML_ARRAY_BOUNDS_CHECK("", (int32_T)_SFD_INTEGER_CHECK("",
        (real_T)c3_b_i), 1, 12, 1, 0) - 1] = c3_d4;
    } else {
      c3_c_atmp = c3_atmp;
      c3_d5 = c3_b_A[_SFD_EML_ARRAY_BOUNDS_CHECK("", (int32_T)_SFD_INTEGER_CHECK
        ("", (real_T)c3_i_i), 1, 192, 1, 0) - 1];
      c3_d6 = c3_d_eml_matlab_zlarfg(chartInstance, &c3_c_atmp, &c3_d5);
      c3_atmp = c3_c_atmp;
      c3_b_A[_SFD_EML_ARRAY_BOUNDS_CHECK("", (int32_T)_SFD_INTEGER_CHECK("",
        (real_T)c3_i_i), 1, 192, 1, 0) - 1] = c3_d5;
      c3_tau[_SFD_EML_ARRAY_BOUNDS_CHECK("", (int32_T)_SFD_INTEGER_CHECK("",
        (real_T)c3_b_i), 1, 12, 1, 0) - 1] = c3_d6;
    }

    c3_b_A[_SFD_EML_ARRAY_BOUNDS_CHECK("", (int32_T)_SFD_INTEGER_CHECK("",
      (real_T)c3_i_i), 1, 192, 1, 0) - 1] = c3_atmp;
    if (c3_b_i < 12) {
      c3_atmp = c3_b_A[_SFD_EML_ARRAY_BOUNDS_CHECK("", (int32_T)
        _SFD_INTEGER_CHECK("", (real_T)c3_i_i), 1, 192, 1, 0) - 1];
      c3_b_A[_SFD_EML_ARRAY_BOUNDS_CHECK("", (int32_T)_SFD_INTEGER_CHECK("",
        (real_T)c3_i_i), 1, 192, 1, 0) - 1] = 1.0;
      c3_q_a = c3_b_i;
      c3_r_a = c3_q_a;
      c3_f_c = c3_r_a << 4;
      c3_s_a = c3_b_i;
      c3_u_b = c3_f_c;
      c3_t_a = c3_s_a;
      c3_v_b = c3_u_b;
      c3_i_ip1 = c3_t_a + c3_v_b;
      c3_b_eml_matlab_zlarf(chartInstance, c3_mmip1, c3_nmi, c3_i_i,
                            c3_tau[_SFD_EML_ARRAY_BOUNDS_CHECK("", (int32_T)
        _SFD_INTEGER_CHECK("", (real_T)c3_b_i), 1, 12, 1, 0) - 1], c3_b_A,
                            c3_i_ip1, c3_work);
      c3_b_A[_SFD_EML_ARRAY_BOUNDS_CHECK("", (int32_T)_SFD_INTEGER_CHECK("",
        (real_T)c3_i_i), 1, 192, 1, 0) - 1] = c3_atmp;
    }

    c3_b_ip1 = c3_ip1 + 1;
    c3_u_a = c3_b_ip1;
    c3_v_a = c3_u_a;
    if (c3_v_a > 12) {
      c3_overflow = false;
    } else {
      c3_eml_switch_helper(chartInstance);
      c3_eml_switch_helper(chartInstance);
      c3_overflow = false;
    }

    if (c3_overflow) {
      c3_check_forloop_overflow_error(chartInstance, c3_overflow);
    }

    for (c3_c_j = c3_b_ip1; c3_c_j < 13; c3_c_j++) {
      c3_b_j = c3_c_j;
      c3_w_a = c3_b_j;
      c3_x_a = c3_w_a;
      c3_g_c = c3_x_a;
      c3_w_b = c3_g_c - 1;
      c3_x_b = c3_w_b;
      c3_h_c = c3_x_b << 4;
      c3_y_a = c3_b_i;
      c3_y_b = c3_h_c;
      c3_ab_a = c3_y_a;
      c3_ab_b = c3_y_b;
      c3_i_j = c3_ab_a + c3_ab_b;
      if (c3_vn1[_SFD_EML_ARRAY_BOUNDS_CHECK("", (int32_T)_SFD_INTEGER_CHECK("",
            (real_T)c3_b_j), 1, 12, 1, 0) - 1] != 0.0) {
        c3_x = c3_b_A[(_SFD_EML_ARRAY_BOUNDS_CHECK("", (int32_T)
          _SFD_INTEGER_CHECK("", (real_T)c3_b_i), 1, 16, 1, 0) +
                       ((_SFD_EML_ARRAY_BOUNDS_CHECK("", (int32_T)
          _SFD_INTEGER_CHECK("", (real_T)c3_b_j), 1, 12, 2, 0) - 1) << 4)) - 1];
        c3_b_x = c3_x;
        c3_y = muDoubleScalarAbs(c3_b_x);
        c3_temp1 = c3_y / c3_vn1[_SFD_EML_ARRAY_BOUNDS_CHECK("", (int32_T)
          _SFD_INTEGER_CHECK("", (real_T)c3_b_j), 1, 12, 1, 0) - 1];
        c3_temp1 = 1.0 - c3_temp1 * c3_temp1;
        if (c3_temp1 < 0.0) {
          c3_temp1 = 0.0;
        }

        c3_temp2 = c3_vn1[_SFD_EML_ARRAY_BOUNDS_CHECK("", (int32_T)
          _SFD_INTEGER_CHECK("", (real_T)c3_b_j), 1, 12, 1, 0) - 1] /
          c3_vn2[_SFD_EML_ARRAY_BOUNDS_CHECK("", (int32_T)_SFD_INTEGER_CHECK("",
          (real_T)c3_b_j), 1, 12, 1, 0) - 1];
        c3_temp2 = c3_temp1 * (c3_temp2 * c3_temp2);
        if (c3_temp2 <= c3_TOL3Z) {
          if (c3_b_i < 16) {
            c3_bb_a = c3_i_j;
            c3_cb_a = c3_bb_a;
            c3_i_c = c3_cb_a;
            c3_n = c3_mmi;
            c3_ix0 = c3_i_c + 1;
            c3_b_n = c3_n;
            c3_b_ix0 = c3_ix0;
            c3_c_threshold(chartInstance);
            c3_c_n = c3_b_n;
            c3_c_ix0 = c3_b_ix0;
            c3_b_y = 0.0;
            if (c3_c_n < 1) {
            } else if (c3_c_n == 1) {
              c3_c_x = c3_b_A[_SFD_EML_ARRAY_BOUNDS_CHECK("", (int32_T)
                _SFD_INTEGER_CHECK("", (real_T)c3_c_ix0), 1, 192, 1, 0) - 1];
              c3_d_x = c3_c_x;
              c3_b_y = muDoubleScalarAbs(c3_d_x);
            } else {
              c3_realmin(chartInstance);
              c3_scale = 2.2250738585072014E-308;
              c3_kstart = c3_c_ix0;
              c3_db_a = c3_c_n;
              c3_j_c = c3_db_a;
              c3_eb_a = c3_j_c - 1;
              c3_k_c = c3_eb_a;
              c3_fb_a = c3_kstart;
              c3_bb_b = c3_k_c;
              c3_kend = c3_fb_a + c3_bb_b;
              c3_b_kstart = c3_kstart;
              c3_b_kend = c3_kend;
              c3_gb_a = c3_b_kstart;
              c3_cb_b = c3_b_kend;
              c3_hb_a = c3_gb_a;
              c3_db_b = c3_cb_b;
              if (c3_hb_a > c3_db_b) {
                c3_b_overflow = false;
              } else {
                c3_eml_switch_helper(chartInstance);
                c3_eml_switch_helper(chartInstance);
                c3_b_overflow = (c3_db_b > 2147483646);
              }

              if (c3_b_overflow) {
                c3_check_forloop_overflow_error(chartInstance, c3_b_overflow);
              }

              for (c3_b_k = c3_b_kstart; c3_b_k <= c3_b_kend; c3_b_k++) {
                c3_c_k = c3_b_k;
                c3_e_x = c3_b_A[_SFD_EML_ARRAY_BOUNDS_CHECK("", (int32_T)
                  _SFD_INTEGER_CHECK("", (real_T)c3_c_k), 1, 192, 1, 0) - 1];
                c3_f_x = c3_e_x;
                c3_absxk = muDoubleScalarAbs(c3_f_x);
                if (c3_absxk > c3_scale) {
                  c3_t = c3_scale / c3_absxk;
                  c3_b_y = 1.0 + c3_b_y * c3_t * c3_t;
                  c3_scale = c3_absxk;
                } else {
                  c3_t = c3_absxk / c3_scale;
                  c3_b_y += c3_t * c3_t;
                }
              }

              c3_b_y = c3_scale * muDoubleScalarSqrt(c3_b_y);
            }

            c3_vn1[_SFD_EML_ARRAY_BOUNDS_CHECK("", (int32_T)_SFD_INTEGER_CHECK(
              "", (real_T)c3_b_j), 1, 12, 1, 0) - 1] = c3_b_y;
            c3_vn2[_SFD_EML_ARRAY_BOUNDS_CHECK("", (int32_T)_SFD_INTEGER_CHECK(
              "", (real_T)c3_b_j), 1, 12, 1, 0) - 1] =
              c3_vn1[_SFD_EML_ARRAY_BOUNDS_CHECK("", (int32_T)_SFD_INTEGER_CHECK
              ("", (real_T)c3_b_j), 1, 12, 1, 0) - 1];
          } else {
            c3_vn1[_SFD_EML_ARRAY_BOUNDS_CHECK("", (int32_T)_SFD_INTEGER_CHECK(
              "", (real_T)c3_b_j), 1, 12, 1, 0) - 1] = 0.0;
            c3_vn2[_SFD_EML_ARRAY_BOUNDS_CHECK("", (int32_T)_SFD_INTEGER_CHECK(
              "", (real_T)c3_b_j), 1, 12, 1, 0) - 1] = 0.0;
          }
        } else {
          c3_g_x = c3_temp1;
          c3_h_x = c3_g_x;
          if (c3_h_x < 0.0) {
            c3_b_eml_error(chartInstance);
          }

          c3_h_x = muDoubleScalarSqrt(c3_h_x);
          c3_vn1[_SFD_EML_ARRAY_BOUNDS_CHECK("", (int32_T)_SFD_INTEGER_CHECK("",
            (real_T)c3_b_j), 1, 12, 1, 0) - 1] =
            c3_vn1[_SFD_EML_ARRAY_BOUNDS_CHECK("", (int32_T)_SFD_INTEGER_CHECK(
            "", (real_T)c3_b_j), 1, 12, 1, 0) - 1] * c3_h_x;
        }
      }
    }
  }

  c3_rankR = 0.0;
  c3_eml_eps(chartInstance);
  c3_i_x = c3_b_A[0];
  c3_j_x = c3_i_x;
  c3_k_x = c3_j_x;
  c3_l_x = c3_k_x;
  c3_c_y = muDoubleScalarAbs(c3_l_x);
  c3_m_x = 0.0;
  c3_n_x = c3_m_x;
  c3_d_y = muDoubleScalarAbs(c3_n_x);
  c3_d = c3_c_y + c3_d_y;
  c3_tol = 16.0 * c3_d * 2.2204460492503131E-16;
  c3_d_k = 0;
  exitg1 = false;
  while ((exitg1 == false) && (c3_d_k < 12)) {
    c3_e_k = 1.0 + (real_T)c3_d_k;
    c3_o_x = c3_b_A[(_SFD_EML_ARRAY_BOUNDS_CHECK("", (int32_T)_SFD_INTEGER_CHECK
                      ("", c3_e_k), 1, 16, 1, 0) + ((_SFD_EML_ARRAY_BOUNDS_CHECK
                       ("", (int32_T)_SFD_INTEGER_CHECK("", c3_e_k), 1, 12, 2, 0)
      - 1) << 4)) - 1];
    c3_p_x = c3_o_x;
    c3_q_x = c3_p_x;
    c3_r_x = c3_q_x;
    c3_e_y = muDoubleScalarAbs(c3_r_x);
    c3_s_x = 0.0;
    c3_t_x = c3_s_x;
    c3_f_y = muDoubleScalarAbs(c3_t_x);
    c3_b_d = c3_e_y + c3_f_y;
    if (c3_b_d <= c3_tol) {
      c3_u_x = c3_tol;
      for (c3_i367 = 0; c3_i367 < 8; c3_i367++) {
        c3_u[c3_i367] = c3_cv5[c3_i367];
      }

      c3_g_y = NULL;
      sf_mex_assign(&c3_g_y, sf_mex_create("y", c3_u, 10, 0U, 1U, 0U, 2, 1, 8),
                    false);
      c3_b_u = 14.0;
      c3_h_y = NULL;
      sf_mex_assign(&c3_h_y, sf_mex_create("y", &c3_b_u, 0, 0U, 0U, 0U, 0),
                    false);
      c3_c_u = 6.0;
      c3_i_y = NULL;
      sf_mex_assign(&c3_i_y, sf_mex_create("y", &c3_c_u, 0, 0U, 0U, 0U, 0),
                    false);
      c3_d_u = c3_u_x;
      c3_j_y = NULL;
      sf_mex_assign(&c3_j_y, sf_mex_create("y", &c3_d_u, 0, 0U, 0U, 0U, 0),
                    false);
      c3_emlrt_marshallIn(chartInstance, sf_mex_call_debug
                          (sfGlobalDebugInstanceStruct, "sprintf", 1U, 2U, 14,
                           sf_mex_call_debug(sfGlobalDebugInstanceStruct,
        "sprintf", 1U, 3U, 14, c3_g_y, 14, c3_h_y, 14, c3_i_y), 14, c3_j_y),
                          "sprintf", c3_str);
      for (c3_i368 = 0; c3_i368 < 14; c3_i368++) {
        c3_b_str[c3_i368] = c3_str[c3_i368];
      }

      c3_c_eml_warning(chartInstance, c3_rankR, c3_b_str);
      exitg1 = true;
    } else {
      c3_rankR++;
      c3_d_k++;
    }
  }

  for (c3_i369 = 0; c3_i369 < 12; c3_i369++) {
    c3_Y[c3_i369] = 0.0;
  }

  for (c3_d_j = 0; c3_d_j < 12; c3_d_j++) {
    c3_e_j = 1.0 + (real_T)c3_d_j;
    c3_tauj = c3_tau[_SFD_EML_ARRAY_BOUNDS_CHECK("", (int32_T)_SFD_INTEGER_CHECK
      ("", c3_e_j), 1, 12, 1, 0) - 1];
    if (c3_tauj != 0.0) {
      c3_wj = c3_b_B[_SFD_EML_ARRAY_BOUNDS_CHECK("", (int32_T)_SFD_INTEGER_CHECK
        ("", c3_e_j), 1, 16, 1, 0) - 1];
      c3_d7 = c3_e_j + 1.0;
      c3_i370 = (int32_T)(16.0 + (1.0 - c3_d7));
      _SFD_FOR_LOOP_VECTOR_CHECK(c3_d7, 1.0, 16.0, mxDOUBLE_CLASS, c3_i370);
      for (c3_c_i = 0; c3_c_i < c3_i370; c3_c_i++) {
        c3_d_i = c3_d7 + (real_T)c3_c_i;
        c3_ib_a = c3_b_A[(_SFD_EML_ARRAY_BOUNDS_CHECK("", (int32_T)
          _SFD_INTEGER_CHECK("", c3_d_i), 1, 16, 1, 0) +
                          ((_SFD_EML_ARRAY_BOUNDS_CHECK("", (int32_T)
          _SFD_INTEGER_CHECK("", c3_e_j), 1, 12, 2, 0) - 1) << 4)) - 1];
        c3_eb_b = c3_b_B[_SFD_EML_ARRAY_BOUNDS_CHECK("", (int32_T)
          _SFD_INTEGER_CHECK("", c3_d_i), 1, 16, 1, 0) - 1];
        c3_z = c3_ib_a * c3_eb_b;
        c3_wj += c3_z;
      }

      c3_wj *= c3_tauj;
      if (c3_wj != 0.0) {
        c3_b_B[_SFD_EML_ARRAY_BOUNDS_CHECK("", (int32_T)_SFD_INTEGER_CHECK("",
          c3_e_j), 1, 16, 1, 0) - 1] = c3_b_B[_SFD_EML_ARRAY_BOUNDS_CHECK("",
          (int32_T)_SFD_INTEGER_CHECK("", c3_e_j), 1, 16, 1, 0) - 1] - c3_wj;
        c3_d8 = c3_e_j + 1.0;
        c3_i371 = (int32_T)(16.0 + (1.0 - c3_d8));
        _SFD_FOR_LOOP_VECTOR_CHECK(c3_d8, 1.0, 16.0, mxDOUBLE_CLASS, c3_i371);
        for (c3_e_i = 0; c3_e_i < c3_i371; c3_e_i++) {
          c3_d_i = c3_d8 + (real_T)c3_e_i;
          c3_b_B[_SFD_EML_ARRAY_BOUNDS_CHECK("", (int32_T)_SFD_INTEGER_CHECK("",
            c3_d_i), 1, 16, 1, 0) - 1] = c3_b_B[_SFD_EML_ARRAY_BOUNDS_CHECK("",
            (int32_T)_SFD_INTEGER_CHECK("", c3_d_i), 1, 16, 1, 0) - 1] - c3_b_A
            [(_SFD_EML_ARRAY_BOUNDS_CHECK("", (int32_T)_SFD_INTEGER_CHECK("",
                c3_d_i), 1, 16, 1, 0) + ((_SFD_EML_ARRAY_BOUNDS_CHECK("",
                 (int32_T)_SFD_INTEGER_CHECK("", c3_e_j), 1, 12, 2, 0) - 1) << 4))
            - 1] * c3_wj;
        }
      }
    }
  }

  c3_rr = c3_rankR;
  c3_b_rr = c3_rr;
  c3_i372 = (int32_T)c3_b_rr;
  _SFD_FOR_LOOP_VECTOR_CHECK(1.0, 1.0, c3_b_rr, mxDOUBLE_CLASS, c3_i372);
  for (c3_f_i = 0; c3_f_i < c3_i372; c3_f_i++) {
    c3_d_i = 1.0 + (real_T)c3_f_i;
    c3_Y[_SFD_EML_ARRAY_BOUNDS_CHECK("", (int32_T)_SFD_INTEGER_CHECK("", (real_T)
      c3_jpvt[_SFD_EML_ARRAY_BOUNDS_CHECK("", (int32_T)_SFD_INTEGER_CHECK("",
      c3_d_i), 1, 12, 1, 0) - 1]), 1, 12, 1, 0) - 1] =
      c3_b_B[_SFD_EML_ARRAY_BOUNDS_CHECK("", (int32_T)_SFD_INTEGER_CHECK("",
      c3_d_i), 1, 16, 1, 0) - 1];
  }

  c3_c_rr = c3_rr;
  c3_i373 = (int32_T)-(1.0 + (-1.0 - c3_c_rr));
  _SFD_FOR_LOOP_VECTOR_CHECK(c3_c_rr, -1.0, 1.0, mxDOUBLE_CLASS, c3_i373);
  for (c3_f_j = 0; c3_f_j < c3_i373; c3_f_j++) {
    c3_e_j = c3_c_rr + -(real_T)c3_f_j;
    c3_pj = c3_jpvt[_SFD_EML_ARRAY_BOUNDS_CHECK("", (int32_T)_SFD_INTEGER_CHECK(
      "", c3_e_j), 1, 12, 1, 0) - 1];
    c3_v_x = c3_Y[_SFD_EML_ARRAY_BOUNDS_CHECK("", (int32_T)_SFD_INTEGER_CHECK("",
      (real_T)c3_pj), 1, 12, 1, 0) - 1];
    c3_k_y = c3_b_A[(_SFD_EML_ARRAY_BOUNDS_CHECK("", (int32_T)_SFD_INTEGER_CHECK
                      ("", c3_e_j), 1, 16, 1, 0) + ((_SFD_EML_ARRAY_BOUNDS_CHECK
                       ("", (int32_T)_SFD_INTEGER_CHECK("", c3_e_j), 1, 12, 2, 0)
      - 1) << 4)) - 1];
    c3_w_x = c3_v_x;
    c3_l_y = c3_k_y;
    c3_b_z = c3_w_x / c3_l_y;
    c3_Y[_SFD_EML_ARRAY_BOUNDS_CHECK("", (int32_T)_SFD_INTEGER_CHECK("", (real_T)
      c3_pj), 1, 12, 1, 0) - 1] = c3_b_z;
    c3_d9 = c3_e_j - 1.0;
    c3_i374 = (int32_T)c3_d9;
    _SFD_FOR_LOOP_VECTOR_CHECK(1.0, 1.0, c3_d9, mxDOUBLE_CLASS, c3_i374);
    for (c3_g_i = 0; c3_g_i < c3_i374; c3_g_i++) {
      c3_d_i = 1.0 + (real_T)c3_g_i;
      c3_Y[_SFD_EML_ARRAY_BOUNDS_CHECK("", (int32_T)_SFD_INTEGER_CHECK("",
        (real_T)c3_jpvt[_SFD_EML_ARRAY_BOUNDS_CHECK("", (int32_T)
        _SFD_INTEGER_CHECK("", c3_d_i), 1, 12, 1, 0) - 1]), 1, 12, 1, 0) - 1] =
        c3_Y[_SFD_EML_ARRAY_BOUNDS_CHECK("", (int32_T)_SFD_INTEGER_CHECK("",
        (real_T)c3_jpvt[_SFD_EML_ARRAY_BOUNDS_CHECK("", (int32_T)
        _SFD_INTEGER_CHECK("", c3_d_i), 1, 12, 1, 0) - 1]), 1, 12, 1, 0) - 1] -
        c3_Y[_SFD_EML_ARRAY_BOUNDS_CHECK("", (int32_T)_SFD_INTEGER_CHECK("",
        (real_T)c3_pj), 1, 12, 1, 0) - 1] * c3_b_A[(_SFD_EML_ARRAY_BOUNDS_CHECK(
        "", (int32_T)_SFD_INTEGER_CHECK("", c3_d_i), 1, 16, 1, 0) +
        ((_SFD_EML_ARRAY_BOUNDS_CHECK("", (int32_T)_SFD_INTEGER_CHECK("", c3_e_j),
        1, 12, 2, 0) - 1) << 4)) - 1];
    }
  }
}

static void c3_c_eml_scalar_eg(SFc3_Robot_Control_v01InstanceStruct
  *chartInstance)
{
  (void)chartInstance;
}

static real_T c3_sqrt(SFc3_Robot_Control_v01InstanceStruct *chartInstance,
                      real_T c3_x)
{
  real_T c3_b_x;
  c3_b_x = c3_x;
  c3_b_sqrt(chartInstance, &c3_b_x);
  return c3_b_x;
}

static void c3_b_eml_error(SFc3_Robot_Control_v01InstanceStruct *chartInstance)
{
  int32_T c3_i375;
  static char_T c3_cv6[30] = { 'C', 'o', 'd', 'e', 'r', ':', 't', 'o', 'o', 'l',
    'b', 'o', 'x', ':', 'E', 'l', 'F', 'u', 'n', 'D', 'o', 'm', 'a', 'i', 'n',
    'E', 'r', 'r', 'o', 'r' };

  char_T c3_u[30];
  const mxArray *c3_y = NULL;
  int32_T c3_i376;
  static char_T c3_cv7[4] = { 's', 'q', 'r', 't' };

  char_T c3_b_u[4];
  const mxArray *c3_b_y = NULL;
  (void)chartInstance;
  for (c3_i375 = 0; c3_i375 < 30; c3_i375++) {
    c3_u[c3_i375] = c3_cv6[c3_i375];
  }

  c3_y = NULL;
  sf_mex_assign(&c3_y, sf_mex_create("y", c3_u, 10, 0U, 1U, 0U, 2, 1, 30), false);
  for (c3_i376 = 0; c3_i376 < 4; c3_i376++) {
    c3_b_u[c3_i376] = c3_cv7[c3_i376];
  }

  c3_b_y = NULL;
  sf_mex_assign(&c3_b_y, sf_mex_create("y", c3_b_u, 10, 0U, 1U, 0U, 2, 1, 4),
                false);
  sf_mex_call_debug(sfGlobalDebugInstanceStruct, "error", 0U, 1U, 14,
                    sf_mex_call_debug(sfGlobalDebugInstanceStruct, "message", 1U,
    2U, 14, c3_y, 14, c3_b_y));
}

static real_T c3_eml_xnrm2(SFc3_Robot_Control_v01InstanceStruct *chartInstance,
  real_T c3_x[192], int32_T c3_ix0)
{
  real_T c3_y;
  int32_T c3_b_ix0;
  int32_T c3_c_ix0;
  real_T c3_scale;
  int32_T c3_kstart;
  int32_T c3_a;
  int32_T c3_kend;
  int32_T c3_b_kstart;
  int32_T c3_b_kend;
  int32_T c3_b_a;
  int32_T c3_b;
  int32_T c3_c_a;
  int32_T c3_b_b;
  boolean_T c3_overflow;
  int32_T c3_k;
  int32_T c3_b_k;
  real_T c3_b_x;
  real_T c3_c_x;
  real_T c3_absxk;
  real_T c3_t;
  c3_b_ix0 = c3_ix0;
  c3_c_threshold(chartInstance);
  c3_c_ix0 = c3_b_ix0;
  c3_y = 0.0;
  c3_realmin(chartInstance);
  c3_scale = 2.2250738585072014E-308;
  c3_kstart = c3_c_ix0;
  c3_a = c3_kstart;
  c3_kend = c3_a;
  c3_b_kstart = c3_kstart;
  c3_b_kend = c3_kend + 15;
  c3_b_a = c3_b_kstart;
  c3_b = c3_b_kend;
  c3_c_a = c3_b_a;
  c3_b_b = c3_b;
  if (c3_c_a > c3_b_b) {
    c3_overflow = false;
  } else {
    c3_eml_switch_helper(chartInstance);
    c3_eml_switch_helper(chartInstance);
    c3_overflow = (c3_b_b > 2147483646);
  }

  if (c3_overflow) {
    c3_check_forloop_overflow_error(chartInstance, c3_overflow);
  }

  for (c3_k = c3_b_kstart; c3_k <= c3_b_kend; c3_k++) {
    c3_b_k = c3_k;
    c3_b_x = c3_x[_SFD_EML_ARRAY_BOUNDS_CHECK("", (int32_T)_SFD_INTEGER_CHECK("",
      (real_T)c3_b_k), 1, 192, 1, 0) - 1];
    c3_c_x = c3_b_x;
    c3_absxk = muDoubleScalarAbs(c3_c_x);
    if (c3_absxk > c3_scale) {
      c3_t = c3_scale / c3_absxk;
      c3_y = 1.0 + c3_y * c3_t * c3_t;
      c3_scale = c3_absxk;
    } else {
      c3_t = c3_absxk / c3_scale;
      c3_y += c3_t * c3_t;
    }
  }

  return c3_scale * muDoubleScalarSqrt(c3_y);
}

static void c3_c_threshold(SFc3_Robot_Control_v01InstanceStruct *chartInstance)
{
  (void)chartInstance;
}

static int32_T c3_eml_ixamax(SFc3_Robot_Control_v01InstanceStruct *chartInstance,
  int32_T c3_n, real_T c3_x[12], int32_T c3_ix0)
{
  int32_T c3_idxmax;
  int32_T c3_b_n;
  int32_T c3_b_ix0;
  int32_T c3_c_n;
  int32_T c3_c_ix0;
  int32_T c3_ix;
  real_T c3_b_x;
  real_T c3_c_x;
  real_T c3_d_x;
  real_T c3_y;
  real_T c3_e_x;
  real_T c3_f_x;
  real_T c3_b_y;
  real_T c3_smax;
  int32_T c3_d_n;
  int32_T c3_b;
  int32_T c3_b_b;
  boolean_T c3_overflow;
  int32_T c3_k;
  int32_T c3_b_k;
  int32_T c3_a;
  real_T c3_g_x;
  real_T c3_h_x;
  real_T c3_i_x;
  real_T c3_c_y;
  real_T c3_j_x;
  real_T c3_k_x;
  real_T c3_d_y;
  real_T c3_s;
  c3_b_n = c3_n;
  c3_b_ix0 = c3_ix0;
  c3_threshold(chartInstance);
  c3_c_n = c3_b_n;
  c3_c_ix0 = c3_b_ix0;
  if (c3_c_n < 1) {
    c3_idxmax = 0;
  } else {
    c3_idxmax = 1;
    if (c3_c_n > 1) {
      c3_ix = c3_c_ix0;
      c3_b_x = c3_x[_SFD_EML_ARRAY_BOUNDS_CHECK("", (int32_T)_SFD_INTEGER_CHECK(
        "", (real_T)c3_ix), 1, 12, 1, 0) - 1];
      c3_c_x = c3_b_x;
      c3_d_x = c3_c_x;
      c3_y = muDoubleScalarAbs(c3_d_x);
      c3_e_x = 0.0;
      c3_f_x = c3_e_x;
      c3_b_y = muDoubleScalarAbs(c3_f_x);
      c3_smax = c3_y + c3_b_y;
      c3_d_n = c3_c_n;
      c3_b = c3_d_n;
      c3_b_b = c3_b;
      if (2 > c3_b_b) {
        c3_overflow = false;
      } else {
        c3_eml_switch_helper(chartInstance);
        c3_eml_switch_helper(chartInstance);
        c3_overflow = (c3_b_b > 2147483646);
      }

      if (c3_overflow) {
        c3_check_forloop_overflow_error(chartInstance, c3_overflow);
      }

      for (c3_k = 2; c3_k <= c3_d_n; c3_k++) {
        c3_b_k = c3_k;
        c3_a = c3_ix + 1;
        c3_ix = c3_a;
        c3_g_x = c3_x[_SFD_EML_ARRAY_BOUNDS_CHECK("", (int32_T)
          _SFD_INTEGER_CHECK("", (real_T)c3_ix), 1, 12, 1, 0) - 1];
        c3_h_x = c3_g_x;
        c3_i_x = c3_h_x;
        c3_c_y = muDoubleScalarAbs(c3_i_x);
        c3_j_x = 0.0;
        c3_k_x = c3_j_x;
        c3_d_y = muDoubleScalarAbs(c3_k_x);
        c3_s = c3_c_y + c3_d_y;
        if (c3_s > c3_smax) {
          c3_idxmax = c3_b_k;
          c3_smax = c3_s;
        }
      }
    }
  }

  return c3_idxmax;
}

static void c3_eml_xswap(SFc3_Robot_Control_v01InstanceStruct *chartInstance,
  real_T c3_x[192], int32_T c3_ix0, int32_T c3_iy0, real_T c3_b_x[192])
{
  int32_T c3_i377;
  for (c3_i377 = 0; c3_i377 < 192; c3_i377++) {
    c3_b_x[c3_i377] = c3_x[c3_i377];
  }

  c3_b_eml_xswap(chartInstance, c3_b_x, c3_ix0, c3_iy0);
}

static void c3_eml_matlab_zlarfg(SFc3_Robot_Control_v01InstanceStruct
  *chartInstance, int32_T c3_n, real_T c3_alpha1, real_T c3_x[192], int32_T
  c3_ix0, real_T *c3_b_alpha1, real_T c3_b_x[192], real_T *c3_tau)
{
  int32_T c3_i378;
  *c3_b_alpha1 = c3_alpha1;
  for (c3_i378 = 0; c3_i378 < 192; c3_i378++) {
    c3_b_x[c3_i378] = c3_x[c3_i378];
  }

  *c3_tau = c3_c_eml_matlab_zlarfg(chartInstance, c3_n, c3_b_alpha1, c3_b_x,
    c3_ix0);
}

static real_T c3_b_eml_xnrm2(SFc3_Robot_Control_v01InstanceStruct *chartInstance,
  int32_T c3_n, real_T c3_x[192], int32_T c3_ix0)
{
  real_T c3_y;
  int32_T c3_b_n;
  int32_T c3_b_ix0;
  int32_T c3_c_n;
  int32_T c3_c_ix0;
  real_T c3_b_x;
  real_T c3_c_x;
  real_T c3_scale;
  int32_T c3_kstart;
  int32_T c3_a;
  int32_T c3_c;
  int32_T c3_b_a;
  int32_T c3_b_c;
  int32_T c3_c_a;
  int32_T c3_b;
  int32_T c3_kend;
  int32_T c3_b_kstart;
  int32_T c3_b_kend;
  int32_T c3_d_a;
  int32_T c3_b_b;
  int32_T c3_e_a;
  int32_T c3_c_b;
  boolean_T c3_overflow;
  int32_T c3_k;
  int32_T c3_b_k;
  real_T c3_d_x;
  real_T c3_e_x;
  real_T c3_absxk;
  real_T c3_t;
  c3_b_n = c3_n;
  c3_b_ix0 = c3_ix0;
  c3_c_threshold(chartInstance);
  c3_c_n = c3_b_n;
  c3_c_ix0 = c3_b_ix0;
  c3_y = 0.0;
  if (c3_c_n < 1) {
  } else if (c3_c_n == 1) {
    c3_b_x = c3_x[_SFD_EML_ARRAY_BOUNDS_CHECK("", (int32_T)_SFD_INTEGER_CHECK("",
      (real_T)c3_c_ix0), 1, 192, 1, 0) - 1];
    c3_c_x = c3_b_x;
    c3_y = muDoubleScalarAbs(c3_c_x);
  } else {
    c3_realmin(chartInstance);
    c3_scale = 2.2250738585072014E-308;
    c3_kstart = c3_c_ix0;
    c3_a = c3_c_n;
    c3_c = c3_a;
    c3_b_a = c3_c - 1;
    c3_b_c = c3_b_a;
    c3_c_a = c3_kstart;
    c3_b = c3_b_c;
    c3_kend = c3_c_a + c3_b;
    c3_b_kstart = c3_kstart;
    c3_b_kend = c3_kend;
    c3_d_a = c3_b_kstart;
    c3_b_b = c3_b_kend;
    c3_e_a = c3_d_a;
    c3_c_b = c3_b_b;
    if (c3_e_a > c3_c_b) {
      c3_overflow = false;
    } else {
      c3_eml_switch_helper(chartInstance);
      c3_eml_switch_helper(chartInstance);
      c3_overflow = (c3_c_b > 2147483646);
    }

    if (c3_overflow) {
      c3_check_forloop_overflow_error(chartInstance, c3_overflow);
    }

    for (c3_k = c3_b_kstart; c3_k <= c3_b_kend; c3_k++) {
      c3_b_k = c3_k;
      c3_d_x = c3_x[_SFD_EML_ARRAY_BOUNDS_CHECK("", (int32_T)_SFD_INTEGER_CHECK(
        "", (real_T)c3_b_k), 1, 192, 1, 0) - 1];
      c3_e_x = c3_d_x;
      c3_absxk = muDoubleScalarAbs(c3_e_x);
      if (c3_absxk > c3_scale) {
        c3_t = c3_scale / c3_absxk;
        c3_y = 1.0 + c3_y * c3_t * c3_t;
        c3_scale = c3_absxk;
      } else {
        c3_t = c3_absxk / c3_scale;
        c3_y += c3_t * c3_t;
      }
    }

    c3_y = c3_scale * muDoubleScalarSqrt(c3_y);
  }

  return c3_y;
}

static void c3_eml_xscal(SFc3_Robot_Control_v01InstanceStruct *chartInstance,
  int32_T c3_n, real_T c3_a, real_T c3_x[192], int32_T c3_ix0, real_T c3_b_x[192])
{
  int32_T c3_i379;
  for (c3_i379 = 0; c3_i379 < 192; c3_i379++) {
    c3_b_x[c3_i379] = c3_x[c3_i379];
  }

  c3_c_eml_xscal(chartInstance, c3_n, c3_a, c3_b_x, c3_ix0);
}

static void c3_e_eml_switch_helper(SFc3_Robot_Control_v01InstanceStruct
  *chartInstance)
{
  (void)chartInstance;
}

static void c3_b_eml_matlab_zlarfg(SFc3_Robot_Control_v01InstanceStruct
  *chartInstance, real_T c3_alpha1, real_T c3_x, real_T *c3_b_alpha1, real_T
  *c3_b_x, real_T *c3_tau)
{
  *c3_b_alpha1 = c3_alpha1;
  *c3_b_x = c3_x;
  *c3_tau = c3_d_eml_matlab_zlarfg(chartInstance, c3_b_alpha1, c3_b_x);
}

static void c3_c_eml_xnrm2(SFc3_Robot_Control_v01InstanceStruct *chartInstance)
{
  c3_c_threshold(chartInstance);
}

static real_T c3_b_eml_xscal(SFc3_Robot_Control_v01InstanceStruct *chartInstance,
  real_T c3_x)
{
  real_T c3_b_x;
  c3_b_x = c3_x;
  c3_d_eml_xscal(chartInstance, &c3_b_x);
  return c3_b_x;
}

static void c3_eml_matlab_zlarf(SFc3_Robot_Control_v01InstanceStruct
  *chartInstance, int32_T c3_m, int32_T c3_n, int32_T c3_iv0, real_T c3_tau,
  real_T c3_C[192], int32_T c3_ic0, real_T c3_work[12], real_T c3_b_C[192],
  real_T c3_b_work[12])
{
  int32_T c3_i380;
  int32_T c3_i381;
  for (c3_i380 = 0; c3_i380 < 192; c3_i380++) {
    c3_b_C[c3_i380] = c3_C[c3_i380];
  }

  for (c3_i381 = 0; c3_i381 < 12; c3_i381++) {
    c3_b_work[c3_i381] = c3_work[c3_i381];
  }

  c3_b_eml_matlab_zlarf(chartInstance, c3_m, c3_n, c3_iv0, c3_tau, c3_b_C,
                        c3_ic0, c3_b_work);
}

static void c3_eml_xgemv(SFc3_Robot_Control_v01InstanceStruct *chartInstance,
  int32_T c3_m, int32_T c3_n, real_T c3_A[192], int32_T c3_ia0, real_T c3_x[192],
  int32_T c3_ix0, real_T c3_y[12], real_T c3_b_y[12])
{
  int32_T c3_i382;
  int32_T c3_i383;
  real_T c3_b_A[192];
  int32_T c3_i384;
  real_T c3_b_x[192];
  for (c3_i382 = 0; c3_i382 < 12; c3_i382++) {
    c3_b_y[c3_i382] = c3_y[c3_i382];
  }

  for (c3_i383 = 0; c3_i383 < 192; c3_i383++) {
    c3_b_A[c3_i383] = c3_A[c3_i383];
  }

  for (c3_i384 = 0; c3_i384 < 192; c3_i384++) {
    c3_b_x[c3_i384] = c3_x[c3_i384];
  }

  c3_b_eml_xgemv(chartInstance, c3_m, c3_n, c3_b_A, c3_ia0, c3_b_x, c3_ix0,
                 c3_b_y);
}

static void c3_below_threshold(SFc3_Robot_Control_v01InstanceStruct
  *chartInstance)
{
  (void)chartInstance;
}

static void c3_eml_xgerc(SFc3_Robot_Control_v01InstanceStruct *chartInstance,
  int32_T c3_m, int32_T c3_n, real_T c3_alpha1, int32_T c3_ix0, real_T c3_y[12],
  real_T c3_A[192], int32_T c3_ia0, real_T c3_b_A[192])
{
  int32_T c3_i385;
  int32_T c3_i386;
  real_T c3_b_y[12];
  for (c3_i385 = 0; c3_i385 < 192; c3_i385++) {
    c3_b_A[c3_i385] = c3_A[c3_i385];
  }

  for (c3_i386 = 0; c3_i386 < 12; c3_i386++) {
    c3_b_y[c3_i386] = c3_y[c3_i386];
  }

  c3_b_eml_xgerc(chartInstance, c3_m, c3_n, c3_alpha1, c3_ix0, c3_b_y, c3_b_A,
                 c3_ia0);
}

static void c3_c_eml_warning(SFc3_Robot_Control_v01InstanceStruct *chartInstance,
  real_T c3_varargin_2, char_T c3_varargin_3[14])
{
  int32_T c3_i387;
  static char_T c3_varargin_1[32] = { 'C', 'o', 'd', 'e', 'r', ':', 'M', 'A',
    'T', 'L', 'A', 'B', ':', 'r', 'a', 'n', 'k', 'D', 'e', 'f', 'i', 'c', 'i',
    'e', 'n', 't', 'M', 'a', 't', 'r', 'i', 'x' };

  char_T c3_u[32];
  const mxArray *c3_y = NULL;
  real_T c3_b_u;
  const mxArray *c3_b_y = NULL;
  int32_T c3_i388;
  char_T c3_c_u[14];
  const mxArray *c3_c_y = NULL;
  (void)chartInstance;
  for (c3_i387 = 0; c3_i387 < 32; c3_i387++) {
    c3_u[c3_i387] = c3_varargin_1[c3_i387];
  }

  c3_y = NULL;
  sf_mex_assign(&c3_y, sf_mex_create("y", c3_u, 10, 0U, 1U, 0U, 2, 1, 32), false);
  c3_b_u = c3_varargin_2;
  c3_b_y = NULL;
  sf_mex_assign(&c3_b_y, sf_mex_create("y", &c3_b_u, 0, 0U, 0U, 0U, 0), false);
  for (c3_i388 = 0; c3_i388 < 14; c3_i388++) {
    c3_c_u[c3_i388] = c3_varargin_3[c3_i388];
  }

  c3_c_y = NULL;
  sf_mex_assign(&c3_c_y, sf_mex_create("y", c3_c_u, 10, 0U, 1U, 0U, 2, 1, 14),
                false);
  sf_mex_call_debug(sfGlobalDebugInstanceStruct, "warning", 0U, 1U, 14,
                    sf_mex_call_debug(sfGlobalDebugInstanceStruct, "message", 1U,
    3U, 14, c3_y, 14, c3_b_y, 14, c3_c_y));
}

static real_T c3_mpower(SFc3_Robot_Control_v01InstanceStruct *chartInstance,
  real_T c3_a)
{
  real_T c3_b_a;
  real_T c3_c_a;
  real_T c3_ak;
  real_T c3_d_a;
  c3_b_a = c3_a;
  c3_c_a = c3_b_a;
  c3_b_eml_scalar_eg(chartInstance);
  c3_ak = c3_c_a;
  c3_d_a = c3_ak;
  c3_b_eml_scalar_eg(chartInstance);
  return c3_d_a * c3_d_a;
}

static real_T c3_atan2(SFc3_Robot_Control_v01InstanceStruct *chartInstance,
  real_T c3_y, real_T c3_x)
{
  real_T c3_b_y;
  real_T c3_b_x;
  c3_b_eml_scalar_eg(chartInstance);
  c3_b_y = c3_y;
  c3_b_x = c3_x;
  return muDoubleScalarAtan2(c3_b_y, c3_b_x);
}

static const mxArray *c3_v_sf_marshallOut(void *chartInstanceVoid, void
  *c3_inData)
{
  const mxArray *c3_mxArrayOutData = NULL;
  int32_T c3_u;
  const mxArray *c3_y = NULL;
  SFc3_Robot_Control_v01InstanceStruct *chartInstance;
  chartInstance = (SFc3_Robot_Control_v01InstanceStruct *)chartInstanceVoid;
  c3_mxArrayOutData = NULL;
  c3_u = *(int32_T *)c3_inData;
  c3_y = NULL;
  sf_mex_assign(&c3_y, sf_mex_create("y", &c3_u, 6, 0U, 0U, 0U, 0), false);
  sf_mex_assign(&c3_mxArrayOutData, c3_y, false);
  return c3_mxArrayOutData;
}

static int32_T c3_x_emlrt_marshallIn(SFc3_Robot_Control_v01InstanceStruct
  *chartInstance, const mxArray *c3_u, const emlrtMsgIdentifier *c3_parentId)
{
  int32_T c3_y;
  int32_T c3_i389;
  (void)chartInstance;
  sf_mex_import(c3_parentId, sf_mex_dup(c3_u), &c3_i389, 1, 6, 0U, 0, 0U, 0);
  c3_y = c3_i389;
  sf_mex_destroy(&c3_u);
  return c3_y;
}

static void c3_u_sf_marshallIn(void *chartInstanceVoid, const mxArray
  *c3_mxArrayInData, const char_T *c3_varName, void *c3_outData)
{
  const mxArray *c3_b_sfEvent;
  const char_T *c3_identifier;
  emlrtMsgIdentifier c3_thisId;
  int32_T c3_y;
  SFc3_Robot_Control_v01InstanceStruct *chartInstance;
  chartInstance = (SFc3_Robot_Control_v01InstanceStruct *)chartInstanceVoid;
  c3_b_sfEvent = sf_mex_dup(c3_mxArrayInData);
  c3_identifier = c3_varName;
  c3_thisId.fIdentifier = c3_identifier;
  c3_thisId.fParent = NULL;
  c3_y = c3_x_emlrt_marshallIn(chartInstance, sf_mex_dup(c3_b_sfEvent),
    &c3_thisId);
  sf_mex_destroy(&c3_b_sfEvent);
  *(int32_T *)c3_outData = c3_y;
  sf_mex_destroy(&c3_mxArrayInData);
}

static uint32_T c3_y_emlrt_marshallIn(SFc3_Robot_Control_v01InstanceStruct
  *chartInstance, const mxArray *c3_b_method, const char_T *c3_identifier)
{
  uint32_T c3_y;
  emlrtMsgIdentifier c3_thisId;
  c3_thisId.fIdentifier = c3_identifier;
  c3_thisId.fParent = NULL;
  c3_y = c3_ab_emlrt_marshallIn(chartInstance, sf_mex_dup(c3_b_method),
    &c3_thisId);
  sf_mex_destroy(&c3_b_method);
  return c3_y;
}

static uint32_T c3_ab_emlrt_marshallIn(SFc3_Robot_Control_v01InstanceStruct
  *chartInstance, const mxArray *c3_u, const emlrtMsgIdentifier *c3_parentId)
{
  uint32_T c3_y;
  uint32_T c3_u3;
  if (mxIsEmpty(c3_u)) {
    chartInstance->c3_method_not_empty = false;
  } else {
    chartInstance->c3_method_not_empty = true;
    sf_mex_import(c3_parentId, sf_mex_dup(c3_u), &c3_u3, 1, 7, 0U, 0, 0U, 0);
    c3_y = c3_u3;
  }

  sf_mex_destroy(&c3_u);
  return c3_y;
}

static uint32_T c3_bb_emlrt_marshallIn(SFc3_Robot_Control_v01InstanceStruct
  *chartInstance, const mxArray *c3_d_state, const char_T *c3_identifier)
{
  uint32_T c3_y;
  emlrtMsgIdentifier c3_thisId;
  c3_thisId.fIdentifier = c3_identifier;
  c3_thisId.fParent = NULL;
  c3_y = c3_cb_emlrt_marshallIn(chartInstance, sf_mex_dup(c3_d_state),
    &c3_thisId);
  sf_mex_destroy(&c3_d_state);
  return c3_y;
}

static uint32_T c3_cb_emlrt_marshallIn(SFc3_Robot_Control_v01InstanceStruct
  *chartInstance, const mxArray *c3_u, const emlrtMsgIdentifier *c3_parentId)
{
  uint32_T c3_y;
  uint32_T c3_u4;
  if (mxIsEmpty(c3_u)) {
    chartInstance->c3_state_not_empty = false;
  } else {
    chartInstance->c3_state_not_empty = true;
    sf_mex_import(c3_parentId, sf_mex_dup(c3_u), &c3_u4, 1, 7, 0U, 0, 0U, 0);
    c3_y = c3_u4;
  }

  sf_mex_destroy(&c3_u);
  return c3_y;
}

static void c3_db_emlrt_marshallIn(SFc3_Robot_Control_v01InstanceStruct
  *chartInstance, const mxArray *c3_d_state, const char_T *c3_identifier,
  uint32_T c3_y[625])
{
  emlrtMsgIdentifier c3_thisId;
  c3_thisId.fIdentifier = c3_identifier;
  c3_thisId.fParent = NULL;
  c3_eb_emlrt_marshallIn(chartInstance, sf_mex_dup(c3_d_state), &c3_thisId, c3_y);
  sf_mex_destroy(&c3_d_state);
}

static void c3_eb_emlrt_marshallIn(SFc3_Robot_Control_v01InstanceStruct
  *chartInstance, const mxArray *c3_u, const emlrtMsgIdentifier *c3_parentId,
  uint32_T c3_y[625])
{
  uint32_T c3_uv3[625];
  int32_T c3_i390;
  if (mxIsEmpty(c3_u)) {
    chartInstance->c3_c_state_not_empty = false;
  } else {
    chartInstance->c3_c_state_not_empty = true;
    sf_mex_import(c3_parentId, sf_mex_dup(c3_u), c3_uv3, 1, 7, 0U, 1, 0U, 1, 625);
    for (c3_i390 = 0; c3_i390 < 625; c3_i390++) {
      c3_y[c3_i390] = c3_uv3[c3_i390];
    }
  }

  sf_mex_destroy(&c3_u);
}

static void c3_fb_emlrt_marshallIn(SFc3_Robot_Control_v01InstanceStruct
  *chartInstance, const mxArray *c3_d_state, const char_T *c3_identifier,
  uint32_T c3_y[2])
{
  emlrtMsgIdentifier c3_thisId;
  c3_thisId.fIdentifier = c3_identifier;
  c3_thisId.fParent = NULL;
  c3_gb_emlrt_marshallIn(chartInstance, sf_mex_dup(c3_d_state), &c3_thisId, c3_y);
  sf_mex_destroy(&c3_d_state);
}

static void c3_gb_emlrt_marshallIn(SFc3_Robot_Control_v01InstanceStruct
  *chartInstance, const mxArray *c3_u, const emlrtMsgIdentifier *c3_parentId,
  uint32_T c3_y[2])
{
  uint32_T c3_uv4[2];
  int32_T c3_i391;
  if (mxIsEmpty(c3_u)) {
    chartInstance->c3_b_state_not_empty = false;
  } else {
    chartInstance->c3_b_state_not_empty = true;
    sf_mex_import(c3_parentId, sf_mex_dup(c3_u), c3_uv4, 1, 7, 0U, 1, 0U, 1, 2);
    for (c3_i391 = 0; c3_i391 < 2; c3_i391++) {
      c3_y[c3_i391] = c3_uv4[c3_i391];
    }
  }

  sf_mex_destroy(&c3_u);
}

static uint8_T c3_hb_emlrt_marshallIn(SFc3_Robot_Control_v01InstanceStruct
  *chartInstance, const mxArray *c3_b_is_active_c3_Robot_Control_v01, const
  char_T *c3_identifier)
{
  uint8_T c3_y;
  emlrtMsgIdentifier c3_thisId;
  c3_thisId.fIdentifier = c3_identifier;
  c3_thisId.fParent = NULL;
  c3_y = c3_ib_emlrt_marshallIn(chartInstance, sf_mex_dup
    (c3_b_is_active_c3_Robot_Control_v01), &c3_thisId);
  sf_mex_destroy(&c3_b_is_active_c3_Robot_Control_v01);
  return c3_y;
}

static uint8_T c3_ib_emlrt_marshallIn(SFc3_Robot_Control_v01InstanceStruct
  *chartInstance, const mxArray *c3_u, const emlrtMsgIdentifier *c3_parentId)
{
  uint8_T c3_y;
  uint8_T c3_u5;
  (void)chartInstance;
  sf_mex_import(c3_parentId, sf_mex_dup(c3_u), &c3_u5, 1, 3, 0U, 0, 0U, 0);
  c3_y = c3_u5;
  sf_mex_destroy(&c3_u);
  return c3_y;
}

static void c3_b_twister_state_vector(SFc3_Robot_Control_v01InstanceStruct
  *chartInstance, uint32_T c3_mt[625], real_T c3_seed)
{
  real_T c3_d10;
  uint32_T c3_u6;
  uint32_T c3_r;
  int32_T c3_mti;
  real_T c3_b_mti;
  real_T c3_d11;
  uint32_T c3_u7;
  (void)chartInstance;
  c3_d10 = c3_seed;
  if (c3_d10 < 4.294967296E+9) {
    if (c3_d10 >= 0.0) {
      c3_u6 = (uint32_T)c3_d10;
    } else {
      c3_u6 = 0U;
    }
  } else if (c3_d10 >= 4.294967296E+9) {
    c3_u6 = MAX_uint32_T;
  } else {
    c3_u6 = 0U;
  }

  c3_r = c3_u6;
  c3_mt[0] = c3_r;
  for (c3_mti = 0; c3_mti < 623; c3_mti++) {
    c3_b_mti = 2.0 + (real_T)c3_mti;
    c3_d11 = muDoubleScalarRound(c3_b_mti - 1.0);
    if (c3_d11 < 4.294967296E+9) {
      if (c3_d11 >= 0.0) {
        c3_u7 = (uint32_T)c3_d11;
      } else {
        c3_u7 = 0U;
      }
    } else if (c3_d11 >= 4.294967296E+9) {
      c3_u7 = MAX_uint32_T;
    } else {
      c3_u7 = 0U;
    }

    c3_r = (c3_r ^ c3_r >> 30U) * 1812433253U + c3_u7;
    c3_mt[_SFD_EML_ARRAY_BOUNDS_CHECK("", (int32_T)_SFD_INTEGER_CHECK("",
      c3_b_mti), 1, 625, 1, 0) - 1] = c3_r;
  }

  c3_mt[624] = 624U;
}

static real_T c3_c_eml_rand_mt19937ar(SFc3_Robot_Control_v01InstanceStruct
  *chartInstance, uint32_T c3_d_state[625])
{
  int32_T c3_i392;
  uint32_T c3_u[2];
  int32_T c3_j;
  real_T c3_b_j;
  uint32_T c3_mti;
  int32_T c3_kk;
  real_T c3_b_kk;
  uint32_T c3_y;
  uint32_T c3_b_y;
  uint32_T c3_c_y;
  int32_T c3_c_kk;
  uint32_T c3_d_y;
  uint32_T c3_e_y;
  uint32_T c3_f_y;
  uint32_T c3_g_y;
  real_T c3_b_r;
  boolean_T c3_b0;
  boolean_T c3_isvalid;
  int32_T c3_k;
  int32_T c3_a;
  int32_T c3_b_a;
  boolean_T guard1 = false;
  int32_T exitg1;
  boolean_T exitg2;

  /* ========================= COPYRIGHT NOTICE ============================ */
  /*  This is a uniform (0,1) pseudorandom number generator based on:        */
  /*                                                                         */
  /*  A C-program for MT19937, with initialization improved 2002/1/26.       */
  /*  Coded by Takuji Nishimura and Makoto Matsumoto.                        */
  /*                                                                         */
  /*  Copyright (C) 1997 - 2002, Makoto Matsumoto and Takuji Nishimura,      */
  /*  All rights reserved.                                                   */
  /*                                                                         */
  /*  Redistribution and use in source and binary forms, with or without     */
  /*  modification, are permitted provided that the following conditions     */
  /*  are met:                                                               */
  /*                                                                         */
  /*    1. Redistributions of source code must retain the above copyright    */
  /*       notice, this list of conditions and the following disclaimer.     */
  /*                                                                         */
  /*    2. Redistributions in binary form must reproduce the above copyright */
  /*       notice, this list of conditions and the following disclaimer      */
  /*       in the documentation and/or other materials provided with the     */
  /*       distribution.                                                     */
  /*                                                                         */
  /*    3. The names of its contributors may not be used to endorse or       */
  /*       promote products derived from this software without specific      */
  /*       prior written permission.                                         */
  /*                                                                         */
  /*  THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS    */
  /*  "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT      */
  /*  LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR  */
  /*  A PARTICULAR PURPOSE ARE DISCLAIMED.  IN NO EVENT SHALL THE COPYRIGHT  */
  /*  OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL,  */
  /*  SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT       */
  /*  LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE,  */
  /*  DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY  */
  /*  THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT    */
  /*  (INCLUDING  NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE */
  /*  OF THIS  SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.  */
  /*                                                                         */
  /* =============================   END   ================================= */
  c3_eml_eps(chartInstance);
  do {
    exitg1 = 0;
    for (c3_i392 = 0; c3_i392 < 2; c3_i392++) {
      c3_u[c3_i392] = 0U;
    }

    for (c3_j = 0; c3_j < 2; c3_j++) {
      c3_b_j = 1.0 + (real_T)c3_j;
      c3_mti = c3_d_state[624] + 1U;
      if ((real_T)c3_mti >= 625.0) {
        for (c3_kk = 0; c3_kk < 227; c3_kk++) {
          c3_b_kk = 1.0 + (real_T)c3_kk;
          c3_y = (c3_d_state[_SFD_EML_ARRAY_BOUNDS_CHECK("", (int32_T)
                   _SFD_INTEGER_CHECK("", c3_b_kk), 1, 625, 1, 0) - 1] &
                  2147483648U) | (c3_d_state[_SFD_EML_ARRAY_BOUNDS_CHECK("",
            (int32_T)_SFD_INTEGER_CHECK("", c3_b_kk + 1.0), 1, 625, 1, 0) - 1] &
            2147483647U);
          c3_b_y = c3_y;
          c3_c_y = c3_b_y;
          if ((real_T)(c3_c_y & 1U) == 0.0) {
            c3_c_y >>= 1U;
          } else {
            c3_c_y = c3_c_y >> 1U ^ 2567483615U;
          }

          c3_d_state[_SFD_EML_ARRAY_BOUNDS_CHECK("", (int32_T)_SFD_INTEGER_CHECK
            ("", c3_b_kk), 1, 625, 1, 0) - 1] =
            c3_d_state[_SFD_EML_ARRAY_BOUNDS_CHECK("", (int32_T)
            _SFD_INTEGER_CHECK("", c3_b_kk + 397.0), 1, 625, 1, 0) - 1] ^ c3_c_y;
        }

        for (c3_c_kk = 0; c3_c_kk < 396; c3_c_kk++) {
          c3_b_kk = 228.0 + (real_T)c3_c_kk;
          c3_y = (c3_d_state[_SFD_EML_ARRAY_BOUNDS_CHECK("", (int32_T)
                   _SFD_INTEGER_CHECK("", c3_b_kk), 1, 625, 1, 0) - 1] &
                  2147483648U) | (c3_d_state[_SFD_EML_ARRAY_BOUNDS_CHECK("",
            (int32_T)_SFD_INTEGER_CHECK("", c3_b_kk + 1.0), 1, 625, 1, 0) - 1] &
            2147483647U);
          c3_d_y = c3_y;
          c3_e_y = c3_d_y;
          if ((real_T)(c3_e_y & 1U) == 0.0) {
            c3_e_y >>= 1U;
          } else {
            c3_e_y = c3_e_y >> 1U ^ 2567483615U;
          }

          c3_d_state[_SFD_EML_ARRAY_BOUNDS_CHECK("", (int32_T)_SFD_INTEGER_CHECK
            ("", c3_b_kk), 1, 625, 1, 0) - 1] =
            c3_d_state[_SFD_EML_ARRAY_BOUNDS_CHECK("", (int32_T)
            _SFD_INTEGER_CHECK("", (c3_b_kk + 1.0) - 228.0), 1, 625, 1, 0) - 1] ^
            c3_e_y;
        }

        c3_y = (c3_d_state[623] & 2147483648U) | (c3_d_state[0] & 2147483647U);
        c3_f_y = c3_y;
        c3_g_y = c3_f_y;
        if ((real_T)(c3_g_y & 1U) == 0.0) {
          c3_g_y >>= 1U;
        } else {
          c3_g_y = c3_g_y >> 1U ^ 2567483615U;
        }

        c3_d_state[623] = c3_d_state[396] ^ c3_g_y;
        c3_mti = 1U;
      }

      c3_y = c3_d_state[_SFD_EML_ARRAY_BOUNDS_CHECK("", (int32_T)(uint32_T)
        _SFD_INTEGER_CHECK("", (real_T)c3_mti), 1, 625, 1, 0) - 1];
      c3_d_state[624] = c3_mti;
      c3_y ^= c3_y >> 11U;
      c3_y ^= c3_y << 7U & 2636928640U;
      c3_y ^= c3_y << 15U & 4022730752U;
      c3_y ^= c3_y >> 18U;
      c3_u[_SFD_EML_ARRAY_BOUNDS_CHECK("", (int32_T)_SFD_INTEGER_CHECK("",
        c3_b_j), 1, 2, 1, 0) - 1] = c3_y;
    }

    c3_u[0] >>= 5U;
    c3_u[1] >>= 6U;
    c3_b_r = 1.1102230246251565E-16 * ((real_T)c3_u[0] * 6.7108864E+7 + (real_T)
      c3_u[1]);
    if (c3_b_r == 0.0) {
      guard1 = false;
      if ((real_T)c3_d_state[624] >= 1.0) {
        if ((real_T)c3_d_state[624] < 625.0) {
          c3_b0 = true;
        } else {
          guard1 = true;
        }
      } else {
        guard1 = true;
      }

      if (guard1 == true) {
        c3_b0 = false;
      }

      c3_isvalid = c3_b0;
      if (c3_isvalid) {
        c3_isvalid = false;
        c3_k = 1;
        exitg2 = false;
        while ((exitg2 == false) && (c3_k < 625)) {
          if ((real_T)c3_d_state[_SFD_EML_ARRAY_BOUNDS_CHECK("", (int32_T)
               _SFD_INTEGER_CHECK("", (real_T)c3_k), 1, 625, 1, 0) - 1] == 0.0)
          {
            c3_a = c3_k;
            c3_b_a = c3_a + 1;
            c3_k = c3_b_a;
          } else {
            c3_isvalid = true;
            exitg2 = true;
          }
        }
      }

      if (!c3_isvalid) {
        c3_eml_error(chartInstance);
      }
    } else {
      exitg1 = 1;
    }
  } while (exitg1 == 0);

  return c3_b_r;
}

static void c3_b_eml_xgemm(SFc3_Robot_Control_v01InstanceStruct *chartInstance,
  real_T c3_A[16], real_T c3_B[16], real_T c3_C[16])
{
  int32_T c3_i393;
  int32_T c3_i394;
  int32_T c3_i395;
  int32_T c3_i396;
  int32_T c3_i397;
  (void)chartInstance;
  for (c3_i393 = 0; c3_i393 < 4; c3_i393++) {
    c3_i394 = 0;
    for (c3_i395 = 0; c3_i395 < 4; c3_i395++) {
      c3_C[c3_i394 + c3_i393] = 0.0;
      c3_i396 = 0;
      for (c3_i397 = 0; c3_i397 < 4; c3_i397++) {
        c3_C[c3_i394 + c3_i393] += c3_A[c3_i396 + c3_i393] * c3_B[c3_i397 +
          c3_i394];
        c3_i396 += 4;
      }

      c3_i394 += 4;
    }
  }
}

static void c3_b_eml_matlab_zgetrf(SFc3_Robot_Control_v01InstanceStruct
  *chartInstance, real_T c3_A[16], int32_T c3_ipiv[4], int32_T *c3_info)
{
  int32_T c3_i398;
  int32_T c3_j;
  int32_T c3_b_j;
  int32_T c3_a;
  int32_T c3_b_a;
  int32_T c3_jm1;
  int32_T c3_b;
  int32_T c3_b_b;
  int32_T c3_mmj;
  int32_T c3_c_a;
  int32_T c3_d_a;
  int32_T c3_c;
  int32_T c3_c_b;
  int32_T c3_d_b;
  int32_T c3_jj;
  int32_T c3_e_a;
  int32_T c3_f_a;
  int32_T c3_jp1j;
  int32_T c3_g_a;
  int32_T c3_h_a;
  int32_T c3_b_c;
  int32_T c3_n;
  int32_T c3_ix0;
  int32_T c3_b_n;
  int32_T c3_b_ix0;
  int32_T c3_c_n;
  int32_T c3_c_ix0;
  int32_T c3_idxmax;
  int32_T c3_ix;
  real_T c3_x;
  real_T c3_b_x;
  real_T c3_c_x;
  real_T c3_y;
  real_T c3_d_x;
  real_T c3_e_x;
  real_T c3_b_y;
  real_T c3_smax;
  int32_T c3_d_n;
  int32_T c3_e_b;
  int32_T c3_f_b;
  boolean_T c3_overflow;
  int32_T c3_k;
  int32_T c3_b_k;
  int32_T c3_i_a;
  real_T c3_f_x;
  real_T c3_g_x;
  real_T c3_h_x;
  real_T c3_c_y;
  real_T c3_i_x;
  real_T c3_j_x;
  real_T c3_d_y;
  real_T c3_s;
  int32_T c3_j_a;
  int32_T c3_k_a;
  int32_T c3_jpiv_offset;
  int32_T c3_l_a;
  int32_T c3_g_b;
  int32_T c3_m_a;
  int32_T c3_h_b;
  int32_T c3_jpiv;
  int32_T c3_n_a;
  int32_T c3_i_b;
  int32_T c3_o_a;
  int32_T c3_j_b;
  int32_T c3_c_c;
  int32_T c3_k_b;
  int32_T c3_l_b;
  int32_T c3_jrow;
  int32_T c3_p_a;
  int32_T c3_m_b;
  int32_T c3_q_a;
  int32_T c3_n_b;
  int32_T c3_jprow;
  int32_T c3_d_ix0;
  int32_T c3_iy0;
  int32_T c3_e_ix0;
  int32_T c3_b_iy0;
  int32_T c3_f_ix0;
  int32_T c3_c_iy0;
  int32_T c3_b_ix;
  int32_T c3_iy;
  int32_T c3_c_k;
  real_T c3_temp;
  int32_T c3_r_a;
  int32_T c3_s_a;
  int32_T c3_b_jp1j;
  int32_T c3_t_a;
  int32_T c3_u_a;
  int32_T c3_d_c;
  int32_T c3_v_a;
  int32_T c3_o_b;
  int32_T c3_w_a;
  int32_T c3_p_b;
  int32_T c3_i399;
  int32_T c3_x_a;
  int32_T c3_q_b;
  int32_T c3_y_a;
  int32_T c3_r_b;
  boolean_T c3_b_overflow;
  int32_T c3_i;
  int32_T c3_b_i;
  real_T c3_k_x;
  real_T c3_e_y;
  real_T c3_l_x;
  real_T c3_f_y;
  real_T c3_z;
  int32_T c3_s_b;
  int32_T c3_t_b;
  int32_T c3_e_c;
  int32_T c3_ab_a;
  int32_T c3_bb_a;
  int32_T c3_f_c;
  int32_T c3_cb_a;
  int32_T c3_db_a;
  int32_T c3_g_c;
  c3_realmin(chartInstance);
  c3_eps(chartInstance);
  for (c3_i398 = 0; c3_i398 < 4; c3_i398++) {
    c3_ipiv[c3_i398] = 1 + c3_i398;
  }

  *c3_info = 0;
  c3_eml_switch_helper(chartInstance);
  for (c3_j = 1; c3_j < 4; c3_j++) {
    c3_b_j = c3_j;
    c3_a = c3_b_j;
    c3_b_a = c3_a - 1;
    c3_jm1 = c3_b_a;
    c3_b = c3_b_j;
    c3_b_b = c3_b;
    c3_mmj = 4 - c3_b_b;
    c3_c_a = c3_jm1;
    c3_d_a = c3_c_a;
    c3_c = c3_d_a * 5;
    c3_c_b = c3_c;
    c3_d_b = c3_c_b + 1;
    c3_jj = c3_d_b;
    c3_e_a = c3_jj;
    c3_f_a = c3_e_a + 1;
    c3_jp1j = c3_f_a;
    c3_g_a = c3_mmj;
    c3_h_a = c3_g_a;
    c3_b_c = c3_h_a;
    c3_n = c3_b_c + 1;
    c3_ix0 = c3_jj;
    c3_b_n = c3_n;
    c3_b_ix0 = c3_ix0;
    c3_threshold(chartInstance);
    c3_c_n = c3_b_n;
    c3_c_ix0 = c3_b_ix0;
    if (c3_c_n < 1) {
      c3_idxmax = 0;
    } else {
      c3_idxmax = 1;
      if (c3_c_n > 1) {
        c3_ix = c3_c_ix0;
        c3_x = c3_A[_SFD_EML_ARRAY_BOUNDS_CHECK("", (int32_T)_SFD_INTEGER_CHECK(
          "", (real_T)c3_ix), 1, 16, 1, 0) - 1];
        c3_b_x = c3_x;
        c3_c_x = c3_b_x;
        c3_y = muDoubleScalarAbs(c3_c_x);
        c3_d_x = 0.0;
        c3_e_x = c3_d_x;
        c3_b_y = muDoubleScalarAbs(c3_e_x);
        c3_smax = c3_y + c3_b_y;
        c3_d_n = c3_c_n;
        c3_e_b = c3_d_n;
        c3_f_b = c3_e_b;
        if (2 > c3_f_b) {
          c3_overflow = false;
        } else {
          c3_eml_switch_helper(chartInstance);
          c3_eml_switch_helper(chartInstance);
          c3_overflow = (c3_f_b > 2147483646);
        }

        if (c3_overflow) {
          c3_check_forloop_overflow_error(chartInstance, c3_overflow);
        }

        for (c3_k = 2; c3_k <= c3_d_n; c3_k++) {
          c3_b_k = c3_k;
          c3_i_a = c3_ix + 1;
          c3_ix = c3_i_a;
          c3_f_x = c3_A[_SFD_EML_ARRAY_BOUNDS_CHECK("", (int32_T)
            _SFD_INTEGER_CHECK("", (real_T)c3_ix), 1, 16, 1, 0) - 1];
          c3_g_x = c3_f_x;
          c3_h_x = c3_g_x;
          c3_c_y = muDoubleScalarAbs(c3_h_x);
          c3_i_x = 0.0;
          c3_j_x = c3_i_x;
          c3_d_y = muDoubleScalarAbs(c3_j_x);
          c3_s = c3_c_y + c3_d_y;
          if (c3_s > c3_smax) {
            c3_idxmax = c3_b_k;
            c3_smax = c3_s;
          }
        }
      }
    }

    c3_j_a = c3_idxmax;
    c3_k_a = c3_j_a - 1;
    c3_jpiv_offset = c3_k_a;
    c3_l_a = c3_jj;
    c3_g_b = c3_jpiv_offset;
    c3_m_a = c3_l_a;
    c3_h_b = c3_g_b;
    c3_jpiv = c3_m_a + c3_h_b;
    if (c3_A[_SFD_EML_ARRAY_BOUNDS_CHECK("", (int32_T)_SFD_INTEGER_CHECK("",
          (real_T)c3_jpiv), 1, 16, 1, 0) - 1] != 0.0) {
      if (c3_jpiv_offset != 0) {
        c3_n_a = c3_b_j;
        c3_i_b = c3_jpiv_offset;
        c3_o_a = c3_n_a;
        c3_j_b = c3_i_b;
        c3_c_c = c3_o_a + c3_j_b;
        c3_ipiv[_SFD_EML_ARRAY_BOUNDS_CHECK("", (int32_T)_SFD_INTEGER_CHECK("",
          (real_T)c3_b_j), 1, 4, 1, 0) - 1] = c3_c_c;
        c3_k_b = c3_jm1;
        c3_l_b = c3_k_b + 1;
        c3_jrow = c3_l_b;
        c3_p_a = c3_jrow;
        c3_m_b = c3_jpiv_offset;
        c3_q_a = c3_p_a;
        c3_n_b = c3_m_b;
        c3_jprow = c3_q_a + c3_n_b;
        c3_d_ix0 = c3_jrow;
        c3_iy0 = c3_jprow;
        c3_e_ix0 = c3_d_ix0;
        c3_b_iy0 = c3_iy0;
        c3_b_eml_switch_helper(chartInstance);
        c3_f_ix0 = c3_e_ix0;
        c3_c_iy0 = c3_b_iy0;
        c3_b_ix = c3_f_ix0;
        c3_iy = c3_c_iy0;
        c3_eml_switch_helper(chartInstance);
        for (c3_c_k = 1; c3_c_k < 5; c3_c_k++) {
          c3_temp = c3_A[_SFD_EML_ARRAY_BOUNDS_CHECK("", (int32_T)
            _SFD_INTEGER_CHECK("", (real_T)c3_b_ix), 1, 16, 1, 0) - 1];
          c3_A[_SFD_EML_ARRAY_BOUNDS_CHECK("", (int32_T)_SFD_INTEGER_CHECK("",
            (real_T)c3_b_ix), 1, 16, 1, 0) - 1] =
            c3_A[_SFD_EML_ARRAY_BOUNDS_CHECK("", (int32_T)_SFD_INTEGER_CHECK("",
            (real_T)c3_iy), 1, 16, 1, 0) - 1];
          c3_A[_SFD_EML_ARRAY_BOUNDS_CHECK("", (int32_T)_SFD_INTEGER_CHECK("",
            (real_T)c3_iy), 1, 16, 1, 0) - 1] = c3_temp;
          c3_r_a = c3_b_ix + 4;
          c3_b_ix = c3_r_a;
          c3_s_a = c3_iy + 4;
          c3_iy = c3_s_a;
        }
      }

      c3_b_jp1j = c3_jp1j;
      c3_t_a = c3_mmj;
      c3_u_a = c3_t_a;
      c3_d_c = c3_u_a;
      c3_v_a = c3_jp1j;
      c3_o_b = c3_d_c - 1;
      c3_w_a = c3_v_a;
      c3_p_b = c3_o_b;
      c3_i399 = c3_w_a + c3_p_b;
      c3_x_a = c3_b_jp1j;
      c3_q_b = c3_i399;
      c3_y_a = c3_x_a;
      c3_r_b = c3_q_b;
      if (c3_y_a > c3_r_b) {
        c3_b_overflow = false;
      } else {
        c3_eml_switch_helper(chartInstance);
        c3_eml_switch_helper(chartInstance);
        c3_b_overflow = (c3_r_b > 2147483646);
      }

      if (c3_b_overflow) {
        c3_check_forloop_overflow_error(chartInstance, c3_b_overflow);
      }

      for (c3_i = c3_b_jp1j; c3_i <= c3_i399; c3_i++) {
        c3_b_i = c3_i;
        c3_k_x = c3_A[_SFD_EML_ARRAY_BOUNDS_CHECK("", (int32_T)
          _SFD_INTEGER_CHECK("", (real_T)c3_b_i), 1, 16, 1, 0) - 1];
        c3_e_y = c3_A[_SFD_EML_ARRAY_BOUNDS_CHECK("", (int32_T)
          _SFD_INTEGER_CHECK("", (real_T)c3_jj), 1, 16, 1, 0) - 1];
        c3_l_x = c3_k_x;
        c3_f_y = c3_e_y;
        c3_z = c3_l_x / c3_f_y;
        c3_A[_SFD_EML_ARRAY_BOUNDS_CHECK("", (int32_T)_SFD_INTEGER_CHECK("",
          (real_T)c3_b_i), 1, 16, 1, 0) - 1] = c3_z;
      }
    } else {
      *c3_info = c3_b_j;
    }

    c3_s_b = c3_b_j;
    c3_t_b = c3_s_b;
    c3_e_c = 4 - c3_t_b;
    c3_ab_a = c3_jj;
    c3_bb_a = c3_ab_a;
    c3_f_c = c3_bb_a;
    c3_cb_a = c3_jj;
    c3_db_a = c3_cb_a;
    c3_g_c = c3_db_a;
    c3_b_eml_xgeru(chartInstance, c3_mmj, c3_e_c, -1.0, c3_jp1j, c3_f_c + 4,
                   c3_A, c3_g_c + 5);
  }

  if (*c3_info == 0) {
    if (!(c3_A[15] != 0.0)) {
      *c3_info = 4;
    }
  }
}

static void c3_b_eml_xgeru(SFc3_Robot_Control_v01InstanceStruct *chartInstance,
  int32_T c3_m, int32_T c3_n, real_T c3_alpha1, int32_T c3_ix0, int32_T c3_iy0,
  real_T c3_A[16], int32_T c3_ia0)
{
  int32_T c3_b_m;
  int32_T c3_b_n;
  real_T c3_b_alpha1;
  int32_T c3_b_ix0;
  int32_T c3_b_iy0;
  int32_T c3_b_ia0;
  int32_T c3_c_m;
  int32_T c3_c_n;
  real_T c3_c_alpha1;
  int32_T c3_c_ix0;
  int32_T c3_c_iy0;
  int32_T c3_c_ia0;
  int32_T c3_d_m;
  int32_T c3_d_n;
  real_T c3_d_alpha1;
  int32_T c3_d_ix0;
  int32_T c3_d_iy0;
  int32_T c3_d_ia0;
  int32_T c3_e_m;
  int32_T c3_e_n;
  real_T c3_e_alpha1;
  int32_T c3_e_ix0;
  int32_T c3_e_iy0;
  int32_T c3_e_ia0;
  int32_T c3_ixstart;
  int32_T c3_a;
  int32_T c3_jA;
  int32_T c3_jy;
  int32_T c3_f_n;
  int32_T c3_b;
  int32_T c3_b_b;
  boolean_T c3_overflow;
  int32_T c3_j;
  real_T c3_yjy;
  real_T c3_temp;
  int32_T c3_ix;
  int32_T c3_c_b;
  int32_T c3_i400;
  int32_T c3_b_a;
  int32_T c3_d_b;
  int32_T c3_i401;
  int32_T c3_c_a;
  int32_T c3_e_b;
  int32_T c3_d_a;
  int32_T c3_f_b;
  boolean_T c3_b_overflow;
  int32_T c3_ijA;
  int32_T c3_b_ijA;
  int32_T c3_e_a;
  int32_T c3_f_a;
  int32_T c3_g_a;
  c3_b_m = c3_m;
  c3_b_n = c3_n;
  c3_b_alpha1 = c3_alpha1;
  c3_b_ix0 = c3_ix0;
  c3_b_iy0 = c3_iy0;
  c3_b_ia0 = c3_ia0;
  c3_c_m = c3_b_m;
  c3_c_n = c3_b_n;
  c3_c_alpha1 = c3_b_alpha1;
  c3_c_ix0 = c3_b_ix0;
  c3_c_iy0 = c3_b_iy0;
  c3_c_ia0 = c3_b_ia0;
  c3_b_threshold(chartInstance);
  c3_d_m = c3_c_m;
  c3_d_n = c3_c_n;
  c3_d_alpha1 = c3_c_alpha1;
  c3_d_ix0 = c3_c_ix0;
  c3_d_iy0 = c3_c_iy0;
  c3_d_ia0 = c3_c_ia0;
  c3_e_m = c3_d_m;
  c3_e_n = c3_d_n;
  c3_e_alpha1 = c3_d_alpha1;
  c3_e_ix0 = c3_d_ix0;
  c3_e_iy0 = c3_d_iy0;
  c3_e_ia0 = c3_d_ia0;
  if (c3_e_alpha1 == 0.0) {
  } else {
    c3_ixstart = c3_e_ix0;
    c3_a = c3_e_ia0 - 1;
    c3_jA = c3_a;
    c3_jy = c3_e_iy0;
    c3_f_n = c3_e_n;
    c3_b = c3_f_n;
    c3_b_b = c3_b;
    if (1 > c3_b_b) {
      c3_overflow = false;
    } else {
      c3_eml_switch_helper(chartInstance);
      c3_eml_switch_helper(chartInstance);
      c3_overflow = (c3_b_b > 2147483646);
    }

    if (c3_overflow) {
      c3_check_forloop_overflow_error(chartInstance, c3_overflow);
    }

    for (c3_j = 1; c3_j <= c3_f_n; c3_j++) {
      c3_yjy = c3_A[_SFD_EML_ARRAY_BOUNDS_CHECK("", (int32_T)_SFD_INTEGER_CHECK(
        "", (real_T)c3_jy), 1, 16, 1, 0) - 1];
      if (c3_yjy != 0.0) {
        c3_temp = c3_yjy * c3_e_alpha1;
        c3_ix = c3_ixstart;
        c3_c_b = c3_jA + 1;
        c3_i400 = c3_c_b;
        c3_b_a = c3_e_m;
        c3_d_b = c3_jA;
        c3_i401 = c3_b_a + c3_d_b;
        c3_c_a = c3_i400;
        c3_e_b = c3_i401;
        c3_d_a = c3_c_a;
        c3_f_b = c3_e_b;
        if (c3_d_a > c3_f_b) {
          c3_b_overflow = false;
        } else {
          c3_eml_switch_helper(chartInstance);
          c3_eml_switch_helper(chartInstance);
          c3_b_overflow = (c3_f_b > 2147483646);
        }

        if (c3_b_overflow) {
          c3_check_forloop_overflow_error(chartInstance, c3_b_overflow);
        }

        for (c3_ijA = c3_i400; c3_ijA <= c3_i401; c3_ijA++) {
          c3_b_ijA = c3_ijA;
          c3_A[_SFD_EML_ARRAY_BOUNDS_CHECK("", (int32_T)_SFD_INTEGER_CHECK("",
            (real_T)c3_b_ijA), 1, 16, 1, 0) - 1] =
            c3_A[_SFD_EML_ARRAY_BOUNDS_CHECK("", (int32_T)_SFD_INTEGER_CHECK("",
            (real_T)c3_b_ijA), 1, 16, 1, 0) - 1] +
            c3_A[_SFD_EML_ARRAY_BOUNDS_CHECK("", (int32_T)_SFD_INTEGER_CHECK("",
            (real_T)c3_ix), 1, 16, 1, 0) - 1] * c3_temp;
          c3_e_a = c3_ix + 1;
          c3_ix = c3_e_a;
        }
      }

      c3_f_a = c3_jy + 4;
      c3_jy = c3_f_a;
      c3_g_a = c3_jA + 4;
      c3_jA = c3_g_a;
    }
  }
}

static void c3_b_eml_xtrsm(SFc3_Robot_Control_v01InstanceStruct *chartInstance,
  real_T c3_A[16], real_T c3_B[16])
{
  int32_T c3_j;
  int32_T c3_b_j;
  int32_T c3_jBcol;
  int32_T c3_k;
  int32_T c3_b_k;
  int32_T c3_kAcol;
  int32_T c3_i402;
  int32_T c3_b;
  int32_T c3_b_b;
  boolean_T c3_overflow;
  int32_T c3_i;
  int32_T c3_b_i;
  c3_d_eml_switch_helper(chartInstance);
  c3_eml_switch_helper(chartInstance);
  for (c3_j = 1; c3_j < 5; c3_j++) {
    c3_b_j = c3_j - 1;
    c3_jBcol = c3_b_j << 2;
    c3_eml_switch_helper(chartInstance);
    for (c3_k = 4; c3_k > 0; c3_k--) {
      c3_b_k = c3_k;
      c3_kAcol = (c3_b_k - 1) << 2;
      if (c3_B[_SFD_EML_ARRAY_BOUNDS_CHECK("", (int32_T)_SFD_INTEGER_CHECK("",
            (real_T)(c3_b_k + c3_jBcol)), 1, 16, 1, 0) - 1] != 0.0) {
        c3_B[_SFD_EML_ARRAY_BOUNDS_CHECK("", (int32_T)_SFD_INTEGER_CHECK("",
          (real_T)(c3_b_k + c3_jBcol)), 1, 16, 1, 0) - 1] = c3_rdivide
          (chartInstance, c3_B[_SFD_EML_ARRAY_BOUNDS_CHECK("", (int32_T)
            _SFD_INTEGER_CHECK("", (real_T)(c3_b_k + c3_jBcol)), 1, 16, 1, 0) -
           1], c3_A[_SFD_EML_ARRAY_BOUNDS_CHECK("", (int32_T)_SFD_INTEGER_CHECK(
             "", (real_T)(c3_b_k + c3_kAcol)), 1, 16, 1, 0) - 1]);
        c3_i402 = c3_b_k - 1;
        c3_b = c3_i402;
        c3_b_b = c3_b;
        if (1 > c3_b_b) {
          c3_overflow = false;
        } else {
          c3_eml_switch_helper(chartInstance);
          c3_eml_switch_helper(chartInstance);
          c3_overflow = (c3_b_b > 2147483646);
        }

        if (c3_overflow) {
          c3_check_forloop_overflow_error(chartInstance, c3_overflow);
        }

        for (c3_i = 1; c3_i <= c3_i402; c3_i++) {
          c3_b_i = c3_i;
          c3_B[_SFD_EML_ARRAY_BOUNDS_CHECK("", (int32_T)_SFD_INTEGER_CHECK("",
            (real_T)(c3_b_i + c3_jBcol)), 1, 16, 1, 0) - 1] =
            c3_B[_SFD_EML_ARRAY_BOUNDS_CHECK("", (int32_T)_SFD_INTEGER_CHECK("",
            (real_T)(c3_b_i + c3_jBcol)), 1, 16, 1, 0) - 1] -
            c3_B[_SFD_EML_ARRAY_BOUNDS_CHECK("", (int32_T)_SFD_INTEGER_CHECK("",
            (real_T)(c3_b_k + c3_jBcol)), 1, 16, 1, 0) - 1] *
            c3_A[_SFD_EML_ARRAY_BOUNDS_CHECK("", (int32_T)_SFD_INTEGER_CHECK("",
            (real_T)(c3_b_i + c3_kAcol)), 1, 16, 1, 0) - 1];
        }
      }
    }
  }
}

static void c3_b_sqrt(SFc3_Robot_Control_v01InstanceStruct *chartInstance,
                      real_T *c3_x)
{
  if (*c3_x < 0.0) {
    c3_b_eml_error(chartInstance);
  }

  *c3_x = muDoubleScalarSqrt(*c3_x);
}

static void c3_b_eml_xswap(SFc3_Robot_Control_v01InstanceStruct *chartInstance,
  real_T c3_x[192], int32_T c3_ix0, int32_T c3_iy0)
{
  int32_T c3_b_ix0;
  int32_T c3_b_iy0;
  int32_T c3_c_ix0;
  int32_T c3_c_iy0;
  int32_T c3_ix;
  int32_T c3_iy;
  int32_T c3_k;
  real_T c3_temp;
  int32_T c3_a;
  int32_T c3_b_a;
  c3_b_ix0 = c3_ix0;
  c3_b_iy0 = c3_iy0;
  c3_b_eml_switch_helper(chartInstance);
  c3_c_ix0 = c3_b_ix0;
  c3_c_iy0 = c3_b_iy0;
  c3_ix = c3_c_ix0;
  c3_iy = c3_c_iy0;
  c3_eml_switch_helper(chartInstance);
  for (c3_k = 1; c3_k < 17; c3_k++) {
    c3_temp = c3_x[_SFD_EML_ARRAY_BOUNDS_CHECK("", (int32_T)_SFD_INTEGER_CHECK(
      "", (real_T)c3_ix), 1, 192, 1, 0) - 1];
    c3_x[_SFD_EML_ARRAY_BOUNDS_CHECK("", (int32_T)_SFD_INTEGER_CHECK("", (real_T)
      c3_ix), 1, 192, 1, 0) - 1] = c3_x[_SFD_EML_ARRAY_BOUNDS_CHECK("", (int32_T)
      _SFD_INTEGER_CHECK("", (real_T)c3_iy), 1, 192, 1, 0) - 1];
    c3_x[_SFD_EML_ARRAY_BOUNDS_CHECK("", (int32_T)_SFD_INTEGER_CHECK("", (real_T)
      c3_iy), 1, 192, 1, 0) - 1] = c3_temp;
    c3_a = c3_ix + 1;
    c3_ix = c3_a;
    c3_b_a = c3_iy + 1;
    c3_iy = c3_b_a;
  }
}

static real_T c3_c_eml_matlab_zlarfg(SFc3_Robot_Control_v01InstanceStruct
  *chartInstance, int32_T c3_n, real_T *c3_alpha1, real_T c3_x[192], int32_T
  c3_ix0)
{
  real_T c3_tau;
  int32_T c3_nm1;
  int32_T c3_i403;
  real_T c3_b_x[192];
  real_T c3_xnorm;
  real_T c3_x1;
  real_T c3_x2;
  real_T c3_a;
  real_T c3_b;
  real_T c3_beta1;
  real_T c3_c_x;
  real_T c3_d_x;
  real_T c3_y;
  int32_T c3_knt;
  int32_T c3_b_a;
  int32_T c3_c_a;
  real_T c3_e_x;
  real_T c3_f_x;
  real_T c3_b_y;
  int32_T c3_i404;
  real_T c3_g_x[192];
  real_T c3_b_x1;
  real_T c3_b_x2;
  real_T c3_d_a;
  real_T c3_b_b;
  real_T c3_h_x;
  real_T c3_c_y;
  real_T c3_i_x;
  real_T c3_d_y;
  real_T c3_e_y;
  real_T c3_f_y;
  int32_T c3_b_knt;
  int32_T c3_c_b;
  int32_T c3_d_b;
  boolean_T c3_overflow;
  int32_T c3_k;
  real_T c3_j_x;
  real_T c3_g_y;
  real_T c3_k_x;
  real_T c3_h_y;
  real_T c3_i_y;
  real_T c3_j_y;
  c3_tau = 0.0;
  if (c3_n <= 0) {
  } else {
    c3_nm1 = c3_n - 1;
    for (c3_i403 = 0; c3_i403 < 192; c3_i403++) {
      c3_b_x[c3_i403] = c3_x[c3_i403];
    }

    c3_xnorm = c3_b_eml_xnrm2(chartInstance, c3_nm1, c3_b_x, c3_ix0);
    if (c3_xnorm != 0.0) {
      c3_x1 = *c3_alpha1;
      c3_x2 = c3_xnorm;
      c3_a = c3_x1;
      c3_b = c3_x2;
      c3_beta1 = muDoubleScalarHypot(c3_a, c3_b);
      if (*c3_alpha1 >= 0.0) {
        c3_beta1 = -c3_beta1;
      }

      c3_realmin(chartInstance);
      c3_eml_eps(chartInstance);
      c3_c_x = c3_beta1;
      c3_d_x = c3_c_x;
      c3_y = muDoubleScalarAbs(c3_d_x);
      if (c3_y < 1.0020841800044864E-292) {
        c3_knt = 0;
        do {
          c3_b_a = c3_knt;
          c3_c_a = c3_b_a + 1;
          c3_knt = c3_c_a;
          c3_c_eml_xscal(chartInstance, c3_nm1, 9.9792015476736E+291, c3_x,
                         c3_ix0);
          c3_beta1 *= 9.9792015476736E+291;
          *c3_alpha1 *= 9.9792015476736E+291;
          c3_e_x = c3_beta1;
          c3_f_x = c3_e_x;
          c3_b_y = muDoubleScalarAbs(c3_f_x);
        } while (!(c3_b_y >= 1.0020841800044864E-292));

        for (c3_i404 = 0; c3_i404 < 192; c3_i404++) {
          c3_g_x[c3_i404] = c3_x[c3_i404];
        }

        c3_xnorm = c3_b_eml_xnrm2(chartInstance, c3_nm1, c3_g_x, c3_ix0);
        c3_b_x1 = *c3_alpha1;
        c3_b_x2 = c3_xnorm;
        c3_d_a = c3_b_x1;
        c3_b_b = c3_b_x2;
        c3_beta1 = muDoubleScalarHypot(c3_d_a, c3_b_b);
        if (*c3_alpha1 >= 0.0) {
          c3_beta1 = -c3_beta1;
        }

        c3_h_x = c3_beta1 - *c3_alpha1;
        c3_c_y = c3_beta1;
        c3_i_x = c3_h_x;
        c3_d_y = c3_c_y;
        c3_tau = c3_i_x / c3_d_y;
        c3_e_y = *c3_alpha1 - c3_beta1;
        c3_f_y = c3_e_y;
        *c3_alpha1 = 1.0 / c3_f_y;
        c3_c_eml_xscal(chartInstance, c3_nm1, *c3_alpha1, c3_x, c3_ix0);
        c3_b_knt = c3_knt;
        c3_c_b = c3_b_knt;
        c3_d_b = c3_c_b;
        if (1 > c3_d_b) {
          c3_overflow = false;
        } else {
          c3_eml_switch_helper(chartInstance);
          c3_eml_switch_helper(chartInstance);
          c3_overflow = (c3_d_b > 2147483646);
        }

        if (c3_overflow) {
          c3_check_forloop_overflow_error(chartInstance, c3_overflow);
        }

        for (c3_k = 1; c3_k <= c3_b_knt; c3_k++) {
          c3_beta1 *= 1.0020841800044864E-292;
        }

        *c3_alpha1 = c3_beta1;
      } else {
        c3_j_x = c3_beta1 - *c3_alpha1;
        c3_g_y = c3_beta1;
        c3_k_x = c3_j_x;
        c3_h_y = c3_g_y;
        c3_tau = c3_k_x / c3_h_y;
        c3_i_y = *c3_alpha1 - c3_beta1;
        c3_j_y = c3_i_y;
        *c3_alpha1 = 1.0 / c3_j_y;
        c3_c_eml_xscal(chartInstance, c3_nm1, *c3_alpha1, c3_x, c3_ix0);
        *c3_alpha1 = c3_beta1;
      }
    }
  }

  return c3_tau;
}

static void c3_c_eml_xscal(SFc3_Robot_Control_v01InstanceStruct *chartInstance,
  int32_T c3_n, real_T c3_a, real_T c3_x[192], int32_T c3_ix0)
{
  int32_T c3_b_n;
  real_T c3_b_a;
  int32_T c3_b_ix0;
  int32_T c3_c_n;
  real_T c3_c_a;
  int32_T c3_c_ix0;
  int32_T c3_d_ix0;
  int32_T c3_d_a;
  int32_T c3_c;
  int32_T c3_b;
  int32_T c3_b_c;
  int32_T c3_e_a;
  int32_T c3_b_b;
  int32_T c3_i405;
  int32_T c3_f_a;
  int32_T c3_c_b;
  int32_T c3_g_a;
  int32_T c3_d_b;
  boolean_T c3_overflow;
  int32_T c3_k;
  int32_T c3_b_k;
  c3_b_n = c3_n;
  c3_b_a = c3_a;
  c3_b_ix0 = c3_ix0;
  c3_e_eml_switch_helper(chartInstance);
  c3_c_n = c3_b_n;
  c3_c_a = c3_b_a;
  c3_c_ix0 = c3_b_ix0;
  c3_d_ix0 = c3_c_ix0;
  c3_d_a = c3_c_n;
  c3_c = c3_d_a;
  c3_b = c3_c - 1;
  c3_b_c = c3_b;
  c3_e_a = c3_c_ix0;
  c3_b_b = c3_b_c;
  c3_i405 = c3_e_a + c3_b_b;
  c3_f_a = c3_d_ix0;
  c3_c_b = c3_i405;
  c3_g_a = c3_f_a;
  c3_d_b = c3_c_b;
  if (c3_g_a > c3_d_b) {
    c3_overflow = false;
  } else {
    c3_eml_switch_helper(chartInstance);
    c3_eml_switch_helper(chartInstance);
    c3_overflow = (c3_d_b > 2147483646);
  }

  if (c3_overflow) {
    c3_check_forloop_overflow_error(chartInstance, c3_overflow);
  }

  for (c3_k = c3_d_ix0; c3_k <= c3_i405; c3_k++) {
    c3_b_k = c3_k;
    c3_x[_SFD_EML_ARRAY_BOUNDS_CHECK("", (int32_T)_SFD_INTEGER_CHECK("", (real_T)
      c3_b_k), 1, 192, 1, 0) - 1] = c3_c_a * c3_x[_SFD_EML_ARRAY_BOUNDS_CHECK("",
      (int32_T)_SFD_INTEGER_CHECK("", (real_T)c3_b_k), 1, 192, 1, 0) - 1];
  }
}

static real_T c3_d_eml_matlab_zlarfg(SFc3_Robot_Control_v01InstanceStruct
  *chartInstance, real_T *c3_alpha1, real_T *c3_x)
{
  real_T c3_tau;
  (void)c3_alpha1;
  (void)c3_x;
  c3_tau = 0.0;
  c3_c_eml_xnrm2(chartInstance);
  return c3_tau;
}

static void c3_d_eml_xscal(SFc3_Robot_Control_v01InstanceStruct *chartInstance,
  real_T *c3_x)
{
  (void)c3_x;
  c3_e_eml_switch_helper(chartInstance);
}

static void c3_b_eml_matlab_zlarf(SFc3_Robot_Control_v01InstanceStruct
  *chartInstance, int32_T c3_m, int32_T c3_n, int32_T c3_iv0, real_T c3_tau,
  real_T c3_C[192], int32_T c3_ic0, real_T c3_work[12])
{
  int32_T c3_lastv;
  int32_T c3_a;
  int32_T c3_b_a;
  int32_T c3_c;
  int32_T c3_b;
  int32_T c3_b_b;
  int32_T c3_b_c;
  int32_T c3_c_a;
  int32_T c3_c_b;
  int32_T c3_d_a;
  int32_T c3_d_b;
  int32_T c3_i;
  int32_T c3_e_a;
  int32_T c3_f_a;
  int32_T c3_g_a;
  int32_T c3_h_a;
  int32_T c3_b_m;
  int32_T c3_b_n;
  int32_T c3_ia0;
  int32_T c3_lastc;
  int32_T c3_i_a;
  int32_T c3_j_a;
  int32_T c3_c_c;
  int32_T c3_k_a;
  int32_T c3_l_a;
  int32_T c3_d_c;
  int32_T c3_m_a;
  int32_T c3_e_b;
  int32_T c3_n_a;
  int32_T c3_f_b;
  int32_T c3_coltop;
  int32_T c3_o_a;
  int32_T c3_p_a;
  int32_T c3_e_c;
  int32_T c3_q_a;
  int32_T c3_g_b;
  int32_T c3_r_a;
  int32_T c3_h_b;
  int32_T c3_colbottom;
  int32_T c3_b_coltop;
  int32_T c3_b_colbottom;
  int32_T c3_s_a;
  int32_T c3_i_b;
  int32_T c3_t_a;
  int32_T c3_j_b;
  boolean_T c3_overflow;
  int32_T c3_ia;
  int32_T c3_b_ia;
  int32_T c3_u_a;
  int32_T c3_v_a;
  int32_T c3_i406;
  real_T c3_b_C[192];
  int32_T c3_i407;
  real_T c3_c_C[192];
  int32_T c3_i408;
  real_T c3_b_work[12];
  int32_T exitg1;
  boolean_T exitg2;
  boolean_T exitg3;
  if (c3_tau != 0.0) {
    c3_lastv = c3_m;
    c3_c_eml_switch_helper(chartInstance);
    c3_a = c3_lastv;
    c3_b_a = c3_a - 1;
    c3_c = c3_b_a;
    c3_b = c3_c;
    c3_b_b = c3_b;
    c3_b_c = c3_b_b;
    c3_c_a = c3_iv0;
    c3_c_b = c3_b_c;
    c3_d_a = c3_c_a;
    c3_d_b = c3_c_b;
    c3_i = c3_d_a + c3_d_b;
    exitg3 = false;
    while ((exitg3 == false) && (c3_lastv > 0)) {
      if (c3_C[_SFD_EML_ARRAY_BOUNDS_CHECK("", (int32_T)_SFD_INTEGER_CHECK("",
            (real_T)c3_i), 1, 192, 1, 0) - 1] == 0.0) {
        c3_e_a = c3_lastv;
        c3_f_a = c3_e_a - 1;
        c3_lastv = c3_f_a;
        c3_g_a = c3_i;
        c3_h_a = c3_g_a - 1;
        c3_i = c3_h_a;
      } else {
        exitg3 = true;
      }
    }

    c3_b_m = c3_lastv;
    c3_b_n = c3_n;
    c3_ia0 = c3_ic0;
    c3_lastc = c3_b_n;
    exitg2 = false;
    while ((exitg2 == false) && (c3_lastc > 0)) {
      c3_i_a = c3_lastc;
      c3_j_a = c3_i_a - 1;
      c3_c_c = c3_j_a;
      c3_k_a = c3_c_c;
      c3_l_a = c3_k_a;
      c3_d_c = c3_l_a << 4;
      c3_m_a = c3_ia0;
      c3_e_b = c3_d_c;
      c3_n_a = c3_m_a;
      c3_f_b = c3_e_b;
      c3_coltop = c3_n_a + c3_f_b;
      c3_o_a = c3_b_m;
      c3_p_a = c3_o_a - 1;
      c3_e_c = c3_p_a;
      c3_q_a = c3_coltop;
      c3_g_b = c3_e_c;
      c3_r_a = c3_q_a;
      c3_h_b = c3_g_b;
      c3_colbottom = c3_r_a + c3_h_b;
      c3_b_coltop = c3_coltop;
      c3_b_colbottom = c3_colbottom;
      c3_s_a = c3_b_coltop;
      c3_i_b = c3_b_colbottom;
      c3_t_a = c3_s_a;
      c3_j_b = c3_i_b;
      if (c3_t_a > c3_j_b) {
        c3_overflow = false;
      } else {
        c3_eml_switch_helper(chartInstance);
        c3_eml_switch_helper(chartInstance);
        c3_overflow = (c3_j_b > 2147483646);
      }

      if (c3_overflow) {
        c3_check_forloop_overflow_error(chartInstance, c3_overflow);
      }

      c3_ia = c3_b_coltop;
      do {
        exitg1 = 0;
        if (c3_ia <= c3_b_colbottom) {
          c3_b_ia = c3_ia;
          if (c3_C[_SFD_EML_ARRAY_BOUNDS_CHECK("", (int32_T)_SFD_INTEGER_CHECK(
                "", (real_T)c3_b_ia), 1, 192, 1, 0) - 1] != 0.0) {
            exitg1 = 1;
          } else {
            c3_ia++;
          }
        } else {
          c3_u_a = c3_lastc;
          c3_v_a = c3_u_a - 1;
          c3_lastc = c3_v_a;
          exitg1 = 2;
        }
      } while (exitg1 == 0);

      if (exitg1 == 1) {
        exitg2 = true;
      }
    }
  } else {
    c3_lastv = 0;
    c3_lastc = 0;
  }

  if (c3_lastv > 0) {
    for (c3_i406 = 0; c3_i406 < 192; c3_i406++) {
      c3_b_C[c3_i406] = c3_C[c3_i406];
    }

    for (c3_i407 = 0; c3_i407 < 192; c3_i407++) {
      c3_c_C[c3_i407] = c3_C[c3_i407];
    }

    c3_b_eml_xgemv(chartInstance, c3_lastv, c3_lastc, c3_b_C, c3_ic0, c3_c_C,
                   c3_iv0, c3_work);
    for (c3_i408 = 0; c3_i408 < 12; c3_i408++) {
      c3_b_work[c3_i408] = c3_work[c3_i408];
    }

    c3_b_eml_xgerc(chartInstance, c3_lastv, c3_lastc, -c3_tau, c3_iv0, c3_b_work,
                   c3_C, c3_ic0);
  }
}

static void c3_b_eml_xgemv(SFc3_Robot_Control_v01InstanceStruct *chartInstance,
  int32_T c3_m, int32_T c3_n, real_T c3_A[192], int32_T c3_ia0, real_T c3_x[192],
  int32_T c3_ix0, real_T c3_y[12])
{
  int32_T c3_b_m;
  int32_T c3_b_n;
  int32_T c3_b_ia0;
  int32_T c3_b_ix0;
  int32_T c3_c_m;
  int32_T c3_c_n;
  int32_T c3_c_ia0;
  int32_T c3_c_ix0;
  int32_T c3_a;
  int32_T c3_mm1;
  int32_T c3_b_a;
  int32_T c3_nm1;
  int32_T c3_b;
  int32_T c3_c;
  int32_T c3_b_b;
  int32_T c3_iyend;
  int32_T c3_b_iyend;
  int32_T c3_c_b;
  int32_T c3_d_b;
  boolean_T c3_overflow;
  int32_T c3_iy;
  int32_T c3_b_iy;
  int32_T c3_d_ia0;
  int32_T c3_e_b;
  int32_T c3_b_c;
  int32_T c3_c_a;
  int32_T c3_f_b;
  int32_T c3_i409;
  int32_T c3_d_a;
  int32_T c3_g_b;
  int32_T c3_e_a;
  int32_T c3_h_b;
  boolean_T c3_b_overflow;
  int32_T c3_iac;
  int32_T c3_b_iac;
  int32_T c3_ix;
  real_T c3_c_c;
  int32_T c3_c_iac;
  int32_T c3_f_a;
  int32_T c3_i_b;
  int32_T c3_i410;
  int32_T c3_g_a;
  int32_T c3_j_b;
  int32_T c3_h_a;
  int32_T c3_k_b;
  boolean_T c3_c_overflow;
  int32_T c3_ia;
  int32_T c3_b_ia;
  real_T c3_i_a;
  real_T c3_l_b;
  real_T c3_z;
  int32_T c3_j_a;
  int32_T c3_k_a;
  c3_b_m = c3_m;
  c3_b_n = c3_n;
  c3_b_ia0 = c3_ia0;
  c3_b_ix0 = c3_ix0;
  c3_below_threshold(chartInstance);
  c3_c_m = c3_b_m;
  c3_c_n = c3_b_n;
  c3_c_ia0 = c3_b_ia0;
  c3_c_ix0 = c3_b_ix0;
  if (c3_c_m == 0) {
  } else if (c3_c_n == 0) {
  } else {
    c3_a = c3_c_m;
    c3_mm1 = c3_a;
    c3_b_a = c3_c_n - 1;
    c3_nm1 = c3_b_a;
    c3_b = c3_nm1;
    c3_c = c3_b;
    c3_b_b = c3_c;
    c3_iyend = c3_b_b;
    c3_b_iyend = c3_iyend + 1;
    c3_c_b = c3_b_iyend;
    c3_d_b = c3_c_b;
    if (1 > c3_d_b) {
      c3_overflow = false;
    } else {
      c3_eml_switch_helper(chartInstance);
      c3_eml_switch_helper(chartInstance);
      c3_overflow = (c3_d_b > 2147483646);
    }

    if (c3_overflow) {
      c3_check_forloop_overflow_error(chartInstance, c3_overflow);
    }

    for (c3_iy = 1; c3_iy <= c3_b_iyend; c3_iy++) {
      c3_b_iy = c3_iy;
      c3_y[_SFD_EML_ARRAY_BOUNDS_CHECK("", (int32_T)_SFD_INTEGER_CHECK("",
        (real_T)c3_b_iy), 1, 12, 1, 0) - 1] = 0.0;
    }

    c3_b_iy = 1;
    c3_d_ia0 = c3_c_ia0;
    c3_e_b = c3_nm1;
    c3_b_c = c3_e_b << 4;
    c3_c_a = c3_c_ia0;
    c3_f_b = c3_b_c;
    c3_i409 = c3_c_a + c3_f_b;
    c3_d_a = c3_d_ia0;
    c3_g_b = c3_i409;
    c3_e_a = c3_d_a;
    c3_h_b = c3_g_b;
    if (c3_e_a > c3_h_b) {
      c3_b_overflow = false;
    } else {
      c3_eml_switch_helper(chartInstance);
      c3_eml_switch_helper(chartInstance);
      c3_b_overflow = (c3_h_b > 2147483631);
    }

    if (c3_b_overflow) {
      c3_check_forloop_overflow_error(chartInstance, c3_b_overflow);
    }

    for (c3_iac = c3_d_ia0; c3_iac <= c3_i409; c3_iac += 16) {
      c3_b_iac = c3_iac;
      c3_ix = c3_c_ix0;
      c3_c_c = 0.0;
      c3_c_iac = c3_b_iac;
      c3_f_a = c3_b_iac;
      c3_i_b = c3_mm1 - 1;
      c3_i410 = c3_f_a + c3_i_b;
      c3_g_a = c3_c_iac;
      c3_j_b = c3_i410;
      c3_h_a = c3_g_a;
      c3_k_b = c3_j_b;
      if (c3_h_a > c3_k_b) {
        c3_c_overflow = false;
      } else {
        c3_eml_switch_helper(chartInstance);
        c3_eml_switch_helper(chartInstance);
        c3_c_overflow = (c3_k_b > 2147483646);
      }

      if (c3_c_overflow) {
        c3_check_forloop_overflow_error(chartInstance, c3_c_overflow);
      }

      for (c3_ia = c3_c_iac; c3_ia <= c3_i410; c3_ia++) {
        c3_b_ia = c3_ia;
        c3_i_a = c3_A[_SFD_EML_ARRAY_BOUNDS_CHECK("", (int32_T)
          _SFD_INTEGER_CHECK("", (real_T)c3_b_ia), 1, 192, 1, 0) - 1];
        c3_l_b = c3_x[_SFD_EML_ARRAY_BOUNDS_CHECK("", (int32_T)
          _SFD_INTEGER_CHECK("", (real_T)c3_ix), 1, 192, 1, 0) - 1];
        c3_z = c3_i_a * c3_l_b;
        c3_c_c += c3_z;
        c3_j_a = c3_ix + 1;
        c3_ix = c3_j_a;
      }

      c3_y[_SFD_EML_ARRAY_BOUNDS_CHECK("", (int32_T)_SFD_INTEGER_CHECK("",
        (real_T)c3_b_iy), 1, 12, 1, 0) - 1] = c3_y[_SFD_EML_ARRAY_BOUNDS_CHECK(
        "", (int32_T)_SFD_INTEGER_CHECK("", (real_T)c3_b_iy), 1, 12, 1, 0) - 1]
        + c3_c_c;
      c3_k_a = c3_b_iy + 1;
      c3_b_iy = c3_k_a;
    }
  }
}

static void c3_b_eml_xgerc(SFc3_Robot_Control_v01InstanceStruct *chartInstance,
  int32_T c3_m, int32_T c3_n, real_T c3_alpha1, int32_T c3_ix0, real_T c3_y[12],
  real_T c3_A[192], int32_T c3_ia0)
{
  int32_T c3_b_m;
  int32_T c3_b_n;
  real_T c3_b_alpha1;
  int32_T c3_b_ix0;
  int32_T c3_b_ia0;
  int32_T c3_c_m;
  int32_T c3_c_n;
  real_T c3_c_alpha1;
  int32_T c3_c_ix0;
  int32_T c3_c_ia0;
  int32_T c3_d_m;
  int32_T c3_d_n;
  real_T c3_d_alpha1;
  int32_T c3_d_ix0;
  int32_T c3_d_ia0;
  int32_T c3_e_m;
  int32_T c3_e_n;
  real_T c3_e_alpha1;
  int32_T c3_e_ix0;
  int32_T c3_e_ia0;
  int32_T c3_ixstart;
  int32_T c3_a;
  int32_T c3_jA;
  int32_T c3_jy;
  int32_T c3_f_n;
  int32_T c3_b;
  int32_T c3_b_b;
  boolean_T c3_overflow;
  int32_T c3_j;
  real_T c3_yjy;
  real_T c3_temp;
  int32_T c3_ix;
  int32_T c3_c_b;
  int32_T c3_i411;
  int32_T c3_b_a;
  int32_T c3_d_b;
  int32_T c3_i412;
  int32_T c3_c_a;
  int32_T c3_e_b;
  int32_T c3_d_a;
  int32_T c3_f_b;
  boolean_T c3_b_overflow;
  int32_T c3_ijA;
  int32_T c3_b_ijA;
  int32_T c3_e_a;
  int32_T c3_f_a;
  int32_T c3_g_a;
  c3_b_m = c3_m;
  c3_b_n = c3_n;
  c3_b_alpha1 = c3_alpha1;
  c3_b_ix0 = c3_ix0;
  c3_b_ia0 = c3_ia0;
  c3_c_m = c3_b_m;
  c3_c_n = c3_b_n;
  c3_c_alpha1 = c3_b_alpha1;
  c3_c_ix0 = c3_b_ix0;
  c3_c_ia0 = c3_b_ia0;
  c3_b_threshold(chartInstance);
  c3_d_m = c3_c_m;
  c3_d_n = c3_c_n;
  c3_d_alpha1 = c3_c_alpha1;
  c3_d_ix0 = c3_c_ix0;
  c3_d_ia0 = c3_c_ia0;
  c3_e_m = c3_d_m;
  c3_e_n = c3_d_n;
  c3_e_alpha1 = c3_d_alpha1;
  c3_e_ix0 = c3_d_ix0;
  c3_e_ia0 = c3_d_ia0;
  if (c3_e_alpha1 == 0.0) {
  } else {
    c3_ixstart = c3_e_ix0;
    c3_a = c3_e_ia0 - 1;
    c3_jA = c3_a;
    c3_jy = 1;
    c3_f_n = c3_e_n;
    c3_b = c3_f_n;
    c3_b_b = c3_b;
    if (1 > c3_b_b) {
      c3_overflow = false;
    } else {
      c3_eml_switch_helper(chartInstance);
      c3_eml_switch_helper(chartInstance);
      c3_overflow = (c3_b_b > 2147483646);
    }

    if (c3_overflow) {
      c3_check_forloop_overflow_error(chartInstance, c3_overflow);
    }

    for (c3_j = 1; c3_j <= c3_f_n; c3_j++) {
      c3_yjy = c3_y[_SFD_EML_ARRAY_BOUNDS_CHECK("", (int32_T)_SFD_INTEGER_CHECK(
        "", (real_T)c3_jy), 1, 12, 1, 0) - 1];
      if (c3_yjy != 0.0) {
        c3_temp = c3_yjy * c3_e_alpha1;
        c3_ix = c3_ixstart;
        c3_c_b = c3_jA + 1;
        c3_i411 = c3_c_b;
        c3_b_a = c3_e_m;
        c3_d_b = c3_jA;
        c3_i412 = c3_b_a + c3_d_b;
        c3_c_a = c3_i411;
        c3_e_b = c3_i412;
        c3_d_a = c3_c_a;
        c3_f_b = c3_e_b;
        if (c3_d_a > c3_f_b) {
          c3_b_overflow = false;
        } else {
          c3_eml_switch_helper(chartInstance);
          c3_eml_switch_helper(chartInstance);
          c3_b_overflow = (c3_f_b > 2147483646);
        }

        if (c3_b_overflow) {
          c3_check_forloop_overflow_error(chartInstance, c3_b_overflow);
        }

        for (c3_ijA = c3_i411; c3_ijA <= c3_i412; c3_ijA++) {
          c3_b_ijA = c3_ijA;
          c3_A[_SFD_EML_ARRAY_BOUNDS_CHECK("", (int32_T)_SFD_INTEGER_CHECK("",
            (real_T)c3_b_ijA), 1, 192, 1, 0) - 1] =
            c3_A[_SFD_EML_ARRAY_BOUNDS_CHECK("", (int32_T)_SFD_INTEGER_CHECK("",
            (real_T)c3_b_ijA), 1, 192, 1, 0) - 1] +
            c3_A[_SFD_EML_ARRAY_BOUNDS_CHECK("", (int32_T)_SFD_INTEGER_CHECK("",
            (real_T)c3_ix), 1, 192, 1, 0) - 1] * c3_temp;
          c3_e_a = c3_ix + 1;
          c3_ix = c3_e_a;
        }
      }

      c3_f_a = c3_jy + 1;
      c3_jy = c3_f_a;
      c3_g_a = c3_jA + 16;
      c3_jA = c3_g_a;
    }
  }
}

static void init_dsm_address_info(SFc3_Robot_Control_v01InstanceStruct
  *chartInstance)
{
  (void)chartInstance;
}

static void init_simulink_io_address(SFc3_Robot_Control_v01InstanceStruct
  *chartInstance)
{
  chartInstance->c3_q = (real_T (*)[3])ssGetInputPortSignal_wrapper
    (chartInstance->S, 0);
  chartInstance->c3_qtilde = (real_T (*)[3])ssGetOutputPortSignal_wrapper
    (chartInstance->S, 1);
  chartInstance->c3_qr = (real_T (*)[3])ssGetInputPortSignal_wrapper
    (chartInstance->S, 1);
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

void sf_c3_Robot_Control_v01_get_check_sum(mxArray *plhs[])
{
  ((real_T *)mxGetPr((plhs[0])))[0] = (real_T)(425795686U);
  ((real_T *)mxGetPr((plhs[0])))[1] = (real_T)(3500244279U);
  ((real_T *)mxGetPr((plhs[0])))[2] = (real_T)(1033611581U);
  ((real_T *)mxGetPr((plhs[0])))[3] = (real_T)(1148286449U);
}

mxArray* sf_c3_Robot_Control_v01_get_post_codegen_info(void);
mxArray *sf_c3_Robot_Control_v01_get_autoinheritance_info(void)
{
  const char *autoinheritanceFields[] = { "checksum", "inputs", "parameters",
    "outputs", "locals", "postCodegenInfo" };

  mxArray *mxAutoinheritanceInfo = mxCreateStructMatrix(1, 1, sizeof
    (autoinheritanceFields)/sizeof(autoinheritanceFields[0]),
    autoinheritanceFields);

  {
    mxArray *mxChecksum = mxCreateString("lPuRJFFHqDAKN2KxUCL7hB");
    mxSetField(mxAutoinheritanceInfo,0,"checksum",mxChecksum);
  }

  {
    const char *dataFields[] = { "size", "type", "complexity" };

    mxArray *mxData = mxCreateStructMatrix(1,2,3,dataFields);

    {
      mxArray *mxSize = mxCreateDoubleMatrix(1,2,mxREAL);
      double *pr = mxGetPr(mxSize);
      pr[0] = (double)(3);
      pr[1] = (double)(1);
      mxSetField(mxData,0,"size",mxSize);
    }

    {
      const char *typeFields[] = { "base", "fixpt" };

      mxArray *mxType = mxCreateStructMatrix(1,1,2,typeFields);
      mxSetField(mxType,0,"base",mxCreateDoubleScalar(10));
      mxSetField(mxType,0,"fixpt",mxCreateDoubleMatrix(0,0,mxREAL));
      mxSetField(mxData,0,"type",mxType);
    }

    mxSetField(mxData,0,"complexity",mxCreateDoubleScalar(0));

    {
      mxArray *mxSize = mxCreateDoubleMatrix(1,2,mxREAL);
      double *pr = mxGetPr(mxSize);
      pr[0] = (double)(3);
      pr[1] = (double)(1);
      mxSetField(mxData,1,"size",mxSize);
    }

    {
      const char *typeFields[] = { "base", "fixpt" };

      mxArray *mxType = mxCreateStructMatrix(1,1,2,typeFields);
      mxSetField(mxType,0,"base",mxCreateDoubleScalar(10));
      mxSetField(mxType,0,"fixpt",mxCreateDoubleMatrix(0,0,mxREAL));
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

    mxArray *mxData = mxCreateStructMatrix(1,1,3,dataFields);

    {
      mxArray *mxSize = mxCreateDoubleMatrix(1,2,mxREAL);
      double *pr = mxGetPr(mxSize);
      pr[0] = (double)(3);
      pr[1] = (double)(1);
      mxSetField(mxData,0,"size",mxSize);
    }

    {
      const char *typeFields[] = { "base", "fixpt" };

      mxArray *mxType = mxCreateStructMatrix(1,1,2,typeFields);
      mxSetField(mxType,0,"base",mxCreateDoubleScalar(10));
      mxSetField(mxType,0,"fixpt",mxCreateDoubleMatrix(0,0,mxREAL));
      mxSetField(mxData,0,"type",mxType);
    }

    mxSetField(mxData,0,"complexity",mxCreateDoubleScalar(0));
    mxSetField(mxAutoinheritanceInfo,0,"outputs",mxData);
  }

  {
    mxSetField(mxAutoinheritanceInfo,0,"locals",mxCreateDoubleMatrix(0,0,mxREAL));
  }

  {
    mxArray* mxPostCodegenInfo = sf_c3_Robot_Control_v01_get_post_codegen_info();
    mxSetField(mxAutoinheritanceInfo,0,"postCodegenInfo",mxPostCodegenInfo);
  }

  return(mxAutoinheritanceInfo);
}

mxArray *sf_c3_Robot_Control_v01_third_party_uses_info(void)
{
  mxArray * mxcell3p = mxCreateCellMatrix(1,0);
  return(mxcell3p);
}

mxArray *sf_c3_Robot_Control_v01_jit_fallback_info(void)
{
  const char *infoFields[] = { "fallbackType", "fallbackReason",
    "incompatibleSymbol", };

  mxArray *mxInfo = mxCreateStructMatrix(1, 1, 3, infoFields);
  mxArray *fallbackReason = mxCreateString("feature_off");
  mxArray *incompatibleSymbol = mxCreateString("");
  mxArray *fallbackType = mxCreateString("early");
  mxSetField(mxInfo, 0, infoFields[0], fallbackType);
  mxSetField(mxInfo, 0, infoFields[1], fallbackReason);
  mxSetField(mxInfo, 0, infoFields[2], incompatibleSymbol);
  return mxInfo;
}

mxArray *sf_c3_Robot_Control_v01_updateBuildInfo_args_info(void)
{
  mxArray *mxBIArgs = mxCreateCellMatrix(1,0);
  return mxBIArgs;
}

mxArray* sf_c3_Robot_Control_v01_get_post_codegen_info(void)
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

static const mxArray *sf_get_sim_state_info_c3_Robot_Control_v01(void)
{
  const char *infoFields[] = { "chartChecksum", "varInfo" };

  mxArray *mxInfo = mxCreateStructMatrix(1, 1, 2, infoFields);
  const char *infoEncStr[] = {
    "100 S1x6'type','srcId','name','auxInfo'{{M[1],M[5],T\"qtilde\",},{M[4],M[0],T\"method\",S'l','i','p'{{M1x2[512 518],M[1],T\"C:\\Program Files\\MATLAB\\R2014b\\toolbox\\eml\\lib\\matlab\\randfun\\eml_rand.m\"}}},{M[4],M[0],T\"state\",S'l','i','p'{{M1x2[165 170],M[1],T\"C:\\Program Files\\MATLAB\\R2014b\\toolbox\\eml\\lib\\matlab\\randfun\\eml_rand_mcg16807_stateful.m\"}}},{M[4],M[0],T\"state\",S'l','i','p'{{M1x2[166 171],M[1],T\"C:\\Program Files\\MATLAB\\R2014b\\toolbox\\eml\\lib\\matlab\\randfun\\eml_rand_mt19937ar_stateful.m\"}}},{M[4],M[0],T\"state\",S'l','i','p'{{M1x2[165 170],M[1],T\"C:\\Program Files\\MATLAB\\R2014b\\toolbox\\eml\\lib\\matlab\\randfun\\eml_rand_shr3cong_stateful.m\"}}},{M[8],M[0],T\"is_active_c3_Robot_Control_v01\",}}"
  };

  mxArray *mxVarInfo = sf_mex_decode_encoded_mx_struct_array(infoEncStr, 6, 10);
  mxArray *mxChecksum = mxCreateDoubleMatrix(1, 4, mxREAL);
  sf_c3_Robot_Control_v01_get_check_sum(&mxChecksum);
  mxSetField(mxInfo, 0, infoFields[0], mxChecksum);
  mxSetField(mxInfo, 0, infoFields[1], mxVarInfo);
  return mxInfo;
}

static void chart_debug_initialization(SimStruct *S, unsigned int
  fullDebuggerInitialization)
{
  if (!sim_mode_is_rtw_gen(S)) {
    SFc3_Robot_Control_v01InstanceStruct *chartInstance;
    ChartRunTimeInfo * crtInfo = (ChartRunTimeInfo *)(ssGetUserData(S));
    ChartInfoStruct * chartInfo = (ChartInfoStruct *)(crtInfo->instanceInfo);
    chartInstance = (SFc3_Robot_Control_v01InstanceStruct *)
      chartInfo->chartInstance;
    if (ssIsFirstInitCond(S) && fullDebuggerInitialization==1) {
      /* do this only if simulation is starting */
      {
        unsigned int chartAlreadyPresent;
        chartAlreadyPresent = sf_debug_initialize_chart
          (sfGlobalDebugInstanceStruct,
           _Robot_Control_v01MachineNumber_,
           3,
           1,
           1,
           0,
           3,
           0,
           0,
           0,
           0,
           12,
           &(chartInstance->chartNumber),
           &(chartInstance->instanceNumber),
           (void *)S);

        /* Each instance must initialize its own list of scripts */
        init_script_number_translation(_Robot_Control_v01MachineNumber_,
          chartInstance->chartNumber,chartInstance->instanceNumber);
        if (chartAlreadyPresent==0) {
          /* this is the first instance */
          sf_debug_set_chart_disable_implicit_casting
            (sfGlobalDebugInstanceStruct,_Robot_Control_v01MachineNumber_,
             chartInstance->chartNumber,1);
          sf_debug_set_chart_event_thresholds(sfGlobalDebugInstanceStruct,
            _Robot_Control_v01MachineNumber_,
            chartInstance->chartNumber,
            0,
            0,
            0);
          _SFD_SET_DATA_PROPS(0,1,1,0,"q");
          _SFD_SET_DATA_PROPS(1,2,0,1,"qtilde");
          _SFD_SET_DATA_PROPS(2,1,1,0,"qr");
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
        _SFD_CV_INIT_EML(0,1,2,0,0,0,0,0,0,0,0);
        _SFD_CV_INIT_EML_FCN(0,0,"eML_blk_kernel",0,-1,1516);
        _SFD_CV_INIT_EML_FCN(0,1,"setupPointsAndPixels",1518,-1,2131);
        _SFD_CV_INIT_SCRIPT(0,1,0,0,0,0,0,0,0,0);
        _SFD_CV_INIT_SCRIPT_FCN(0,0,"random",0,-1,857);
        _SFD_CV_INIT_SCRIPT(1,1,0,0,0,0,0,0,0,0);
        _SFD_CV_INIT_SCRIPT_FCN(1,0,"newPose",0,-1,171);
        _SFD_CV_INIT_SCRIPT(2,1,2,0,0,0,0,0,0,0);
        _SFD_CV_INIT_SCRIPT_FCN(2,0,"transformationMatrix",0,-1,789);
        _SFD_CV_INIT_SCRIPT_IF(2,0,45,60,208,227);
        _SFD_CV_INIT_SCRIPT_IF(2,1,208,227,-1,227);
        _SFD_CV_INIT_SCRIPT_RELATIONAL(2,0,48,59,-1,0);
        _SFD_CV_INIT_SCRIPT(3,1,0,0,0,0,0,0,0,0);
        _SFD_CV_INIT_SCRIPT_FCN(3,0,"EulerTrans",0,-1,450);
        _SFD_CV_INIT_SCRIPT(4,1,0,0,0,0,0,0,0,0);
        _SFD_CV_INIT_SCRIPT_FCN(4,0,"newCamera",0,-1,282);
        _SFD_CV_INIT_SCRIPT(5,1,0,0,0,0,0,0,0,0);
        _SFD_CV_INIT_SCRIPT_FCN(5,0,"newRobot",0,-1,273);
        _SFD_CV_INIT_SCRIPT(6,1,0,0,0,0,0,0,0,0);
        _SFD_CV_INIT_SCRIPT_FCN(6,0,"simulateRealRobot",0,-1,581);
        _SFD_CV_INIT_SCRIPT(7,1,0,0,0,0,0,0,0,0);
        _SFD_CV_INIT_SCRIPT_FCN(7,0,"transformImageSpace",0,-1,1350);
        _SFD_CV_INIT_SCRIPT(8,1,0,0,0,0,0,0,0,0);
        _SFD_CV_INIT_SCRIPT_FCN(8,0,"intrinsicMatrix",0,-1,272);
        _SFD_CV_INIT_SCRIPT(9,1,0,0,0,0,0,0,0,0);
        _SFD_CV_INIT_SCRIPT_FCN(9,0,"mirage",48,-1,1272);
        _SFD_CV_INIT_SCRIPT(10,1,0,0,0,0,1,0,0,0);
        _SFD_CV_INIT_SCRIPT_FCN(10,0,"getEquations",0,-1,1254);
        _SFD_CV_INIT_SCRIPT_FOR(10,0,711,721,1254);
        _SFD_CV_INIT_SCRIPT(11,1,0,0,0,0,0,0,0,0);
        _SFD_CV_INIT_SCRIPT_FCN(11,0,"get3DRotationAngles",0,-1,424);

        {
          unsigned int dimVector[1];
          dimVector[0]= 3;
          _SFD_SET_DATA_COMPILED_PROPS(0,SF_DOUBLE,1,&(dimVector[0]),0,0,0,0.0,
            1.0,0,0,(MexFcnForType)c3_sf_marshallOut,(MexInFcnForType)NULL);
        }

        {
          unsigned int dimVector[1];
          dimVector[0]= 3;
          _SFD_SET_DATA_COMPILED_PROPS(1,SF_DOUBLE,1,&(dimVector[0]),0,0,0,0.0,
            1.0,0,0,(MexFcnForType)c3_sf_marshallOut,(MexInFcnForType)
            c3_sf_marshallIn);
        }

        {
          unsigned int dimVector[1];
          dimVector[0]= 3;
          _SFD_SET_DATA_COMPILED_PROPS(2,SF_DOUBLE,1,&(dimVector[0]),0,0,0,0.0,
            1.0,0,0,(MexFcnForType)c3_sf_marshallOut,(MexInFcnForType)NULL);
        }

        _SFD_SET_DATA_VALUE_PTR(0U, *chartInstance->c3_q);
        _SFD_SET_DATA_VALUE_PTR(1U, *chartInstance->c3_qtilde);
        _SFD_SET_DATA_VALUE_PTR(2U, *chartInstance->c3_qr);
      }
    } else {
      sf_debug_reset_current_state_configuration(sfGlobalDebugInstanceStruct,
        _Robot_Control_v01MachineNumber_,chartInstance->chartNumber,
        chartInstance->instanceNumber);
    }
  }
}

static const char* sf_get_instance_specialization(void)
{
  return "dHXSXKPVPazPj3VhTZAqlH";
}

static void sf_opaque_initialize_c3_Robot_Control_v01(void *chartInstanceVar)
{
  chart_debug_initialization(((SFc3_Robot_Control_v01InstanceStruct*)
    chartInstanceVar)->S,0);
  initialize_params_c3_Robot_Control_v01((SFc3_Robot_Control_v01InstanceStruct*)
    chartInstanceVar);
  initialize_c3_Robot_Control_v01((SFc3_Robot_Control_v01InstanceStruct*)
    chartInstanceVar);
}

static void sf_opaque_enable_c3_Robot_Control_v01(void *chartInstanceVar)
{
  enable_c3_Robot_Control_v01((SFc3_Robot_Control_v01InstanceStruct*)
    chartInstanceVar);
}

static void sf_opaque_disable_c3_Robot_Control_v01(void *chartInstanceVar)
{
  disable_c3_Robot_Control_v01((SFc3_Robot_Control_v01InstanceStruct*)
    chartInstanceVar);
}

static void sf_opaque_gateway_c3_Robot_Control_v01(void *chartInstanceVar)
{
  sf_gateway_c3_Robot_Control_v01((SFc3_Robot_Control_v01InstanceStruct*)
    chartInstanceVar);
}

static const mxArray* sf_opaque_get_sim_state_c3_Robot_Control_v01(SimStruct* S)
{
  ChartRunTimeInfo * crtInfo = (ChartRunTimeInfo *)(ssGetUserData(S));
  ChartInfoStruct * chartInfo = (ChartInfoStruct *)(crtInfo->instanceInfo);
  return get_sim_state_c3_Robot_Control_v01
    ((SFc3_Robot_Control_v01InstanceStruct*)chartInfo->chartInstance);/* raw sim ctx */
}

static void sf_opaque_set_sim_state_c3_Robot_Control_v01(SimStruct* S, const
  mxArray *st)
{
  ChartRunTimeInfo * crtInfo = (ChartRunTimeInfo *)(ssGetUserData(S));
  ChartInfoStruct * chartInfo = (ChartInfoStruct *)(crtInfo->instanceInfo);
  set_sim_state_c3_Robot_Control_v01((SFc3_Robot_Control_v01InstanceStruct*)
    chartInfo->chartInstance, st);
}

static void sf_opaque_terminate_c3_Robot_Control_v01(void *chartInstanceVar)
{
  if (chartInstanceVar!=NULL) {
    SimStruct *S = ((SFc3_Robot_Control_v01InstanceStruct*) chartInstanceVar)->S;
    ChartRunTimeInfo * crtInfo = (ChartRunTimeInfo *)(ssGetUserData(S));
    if (sim_mode_is_rtw_gen(S) || sim_mode_is_external(S)) {
      sf_clear_rtw_identifier(S);
      unload_Robot_Control_v01_optimization_info();
    }

    finalize_c3_Robot_Control_v01((SFc3_Robot_Control_v01InstanceStruct*)
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
  initSimStructsc3_Robot_Control_v01((SFc3_Robot_Control_v01InstanceStruct*)
    chartInstanceVar);
}

extern unsigned int sf_machine_global_initializer_called(void);
static void mdlProcessParameters_c3_Robot_Control_v01(SimStruct *S)
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
    initialize_params_c3_Robot_Control_v01((SFc3_Robot_Control_v01InstanceStruct*)
      (chartInfo->chartInstance));
  }
}

static void mdlSetWorkWidths_c3_Robot_Control_v01(SimStruct *S)
{
  if (sim_mode_is_rtw_gen(S) || sim_mode_is_external(S)) {
    mxArray *infoStruct = load_Robot_Control_v01_optimization_info();
    int_T chartIsInlinable =
      (int_T)sf_is_chart_inlinable(sf_get_instance_specialization(),infoStruct,3);
    ssSetStateflowIsInlinable(S,chartIsInlinable);
    ssSetRTWCG(S,sf_rtw_info_uint_prop(sf_get_instance_specialization(),
                infoStruct,3,"RTWCG"));
    ssSetEnableFcnIsTrivial(S,1);
    ssSetDisableFcnIsTrivial(S,1);
    ssSetNotMultipleInlinable(S,sf_rtw_info_uint_prop
      (sf_get_instance_specialization(),infoStruct,3,
       "gatewayCannotBeInlinedMultipleTimes"));
    sf_update_buildInfo(sf_get_instance_specialization(),infoStruct,3);
    if (chartIsInlinable) {
      ssSetInputPortOptimOpts(S, 0, SS_REUSABLE_AND_LOCAL);
      ssSetInputPortOptimOpts(S, 1, SS_REUSABLE_AND_LOCAL);
      sf_mark_chart_expressionable_inputs(S,sf_get_instance_specialization(),
        infoStruct,3,2);
      sf_mark_chart_reusable_outputs(S,sf_get_instance_specialization(),
        infoStruct,3,1);
    }

    {
      unsigned int outPortIdx;
      for (outPortIdx=1; outPortIdx<=1; ++outPortIdx) {
        ssSetOutputPortOptimizeInIR(S, outPortIdx, 1U);
      }
    }

    {
      unsigned int inPortIdx;
      for (inPortIdx=0; inPortIdx < 2; ++inPortIdx) {
        ssSetInputPortOptimizeInIR(S, inPortIdx, 1U);
      }
    }

    sf_set_rtw_dwork_info(S,sf_get_instance_specialization(),infoStruct,3);
    ssSetHasSubFunctions(S,!(chartIsInlinable));
  } else {
  }

  ssSetOptions(S,ssGetOptions(S)|SS_OPTION_WORKS_WITH_CODE_REUSE);
  ssSetChecksum0(S,(3629204890U));
  ssSetChecksum1(S,(2522167838U));
  ssSetChecksum2(S,(2843102012U));
  ssSetChecksum3(S,(3192422484U));
  ssSetmdlDerivatives(S, NULL);
  ssSetExplicitFCSSCtrl(S,1);
  ssSupportsMultipleExecInstances(S,1);
}

static void mdlRTW_c3_Robot_Control_v01(SimStruct *S)
{
  if (sim_mode_is_rtw_gen(S)) {
    ssWriteRTWStrParam(S, "StateflowChartType", "Embedded MATLAB");
  }
}

static void mdlStart_c3_Robot_Control_v01(SimStruct *S)
{
  SFc3_Robot_Control_v01InstanceStruct *chartInstance;
  ChartRunTimeInfo * crtInfo = (ChartRunTimeInfo *)utMalloc(sizeof
    (ChartRunTimeInfo));
  chartInstance = (SFc3_Robot_Control_v01InstanceStruct *)utMalloc(sizeof
    (SFc3_Robot_Control_v01InstanceStruct));
  memset(chartInstance, 0, sizeof(SFc3_Robot_Control_v01InstanceStruct));
  if (chartInstance==NULL) {
    sf_mex_error_message("Could not allocate memory for chart instance.");
  }

  chartInstance->chartInfo.chartInstance = chartInstance;
  chartInstance->chartInfo.isEMLChart = 1;
  chartInstance->chartInfo.chartInitialized = 0;
  chartInstance->chartInfo.sFunctionGateway =
    sf_opaque_gateway_c3_Robot_Control_v01;
  chartInstance->chartInfo.initializeChart =
    sf_opaque_initialize_c3_Robot_Control_v01;
  chartInstance->chartInfo.terminateChart =
    sf_opaque_terminate_c3_Robot_Control_v01;
  chartInstance->chartInfo.enableChart = sf_opaque_enable_c3_Robot_Control_v01;
  chartInstance->chartInfo.disableChart = sf_opaque_disable_c3_Robot_Control_v01;
  chartInstance->chartInfo.getSimState =
    sf_opaque_get_sim_state_c3_Robot_Control_v01;
  chartInstance->chartInfo.setSimState =
    sf_opaque_set_sim_state_c3_Robot_Control_v01;
  chartInstance->chartInfo.getSimStateInfo =
    sf_get_sim_state_info_c3_Robot_Control_v01;
  chartInstance->chartInfo.zeroCrossings = NULL;
  chartInstance->chartInfo.outputs = NULL;
  chartInstance->chartInfo.derivatives = NULL;
  chartInstance->chartInfo.mdlRTW = mdlRTW_c3_Robot_Control_v01;
  chartInstance->chartInfo.mdlStart = mdlStart_c3_Robot_Control_v01;
  chartInstance->chartInfo.mdlSetWorkWidths =
    mdlSetWorkWidths_c3_Robot_Control_v01;
  chartInstance->chartInfo.extModeExec = NULL;
  chartInstance->chartInfo.restoreLastMajorStepConfiguration = NULL;
  chartInstance->chartInfo.restoreBeforeLastMajorStepConfiguration = NULL;
  chartInstance->chartInfo.storeCurrentConfiguration = NULL;
  chartInstance->chartInfo.callAtomicSubchartUserFcn = NULL;
  chartInstance->chartInfo.callAtomicSubchartAutoFcn = NULL;
  chartInstance->chartInfo.debugInstance = sfGlobalDebugInstanceStruct;
  chartInstance->S = S;
  crtInfo->checksum = SF_RUNTIME_INFO_CHECKSUM;
  crtInfo->instanceInfo = (&(chartInstance->chartInfo));
  crtInfo->isJITEnabled = false;
  crtInfo->compiledInfo = NULL;
  ssSetUserData(S,(void *)(crtInfo));  /* register the chart instance with simstruct */
  init_dsm_address_info(chartInstance);
  init_simulink_io_address(chartInstance);
  if (!sim_mode_is_rtw_gen(S)) {
  }

  sf_opaque_init_subchart_simstructs(chartInstance->chartInfo.chartInstance);
  chart_debug_initialization(S,1);
}

void c3_Robot_Control_v01_method_dispatcher(SimStruct *S, int_T method, void
  *data)
{
  switch (method) {
   case SS_CALL_MDL_START:
    mdlStart_c3_Robot_Control_v01(S);
    break;

   case SS_CALL_MDL_SET_WORK_WIDTHS:
    mdlSetWorkWidths_c3_Robot_Control_v01(S);
    break;

   case SS_CALL_MDL_PROCESS_PARAMETERS:
    mdlProcessParameters_c3_Robot_Control_v01(S);
    break;

   default:
    /* Unhandled method */
    sf_mex_error_message("Stateflow Internal Error:\n"
                         "Error calling c3_Robot_Control_v01_method_dispatcher.\n"
                         "Can't handle method %d.\n", method);
    break;
  }
}
