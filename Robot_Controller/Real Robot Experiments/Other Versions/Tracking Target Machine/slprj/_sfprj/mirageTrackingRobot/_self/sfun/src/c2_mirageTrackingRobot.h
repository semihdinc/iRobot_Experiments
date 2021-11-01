#ifndef __c2_mirageTrackingRobot_h__
#define __c2_mirageTrackingRobot_h__

/* Include files */
#include "sf_runtime/sfc_sf.h"
#include "sf_runtime/sfc_mex.h"
#include "rtwtypes.h"
#include "multiword_types.h"

/* Type Definitions */
#ifndef struct_sMB5GaXc40ozYzvPFeCX3Q
#define struct_sMB5GaXc40ozYzvPFeCX3Q

struct sMB5GaXc40ozYzvPFeCX3Q
{
  int32_T dummy;
};

#endif                                 /*struct_sMB5GaXc40ozYzvPFeCX3Q*/

#ifndef typedef_c2_coder_internal_cell_4
#define typedef_c2_coder_internal_cell_4

typedef struct sMB5GaXc40ozYzvPFeCX3Q c2_coder_internal_cell_4;

#endif                                 /*typedef_c2_coder_internal_cell_4*/

#ifndef struct_sMB5GaXc40ozYzvPFeCX3Q
#define struct_sMB5GaXc40ozYzvPFeCX3Q

struct sMB5GaXc40ozYzvPFeCX3Q
{
  int32_T dummy;
};

#endif                                 /*struct_sMB5GaXc40ozYzvPFeCX3Q*/

#ifndef typedef_c2_coder_internal_cell_5
#define typedef_c2_coder_internal_cell_5

typedef struct sMB5GaXc40ozYzvPFeCX3Q c2_coder_internal_cell_5;

#endif                                 /*typedef_c2_coder_internal_cell_5*/

#ifndef struct_sMB5GaXc40ozYzvPFeCX3Q
#define struct_sMB5GaXc40ozYzvPFeCX3Q

struct sMB5GaXc40ozYzvPFeCX3Q
{
  int32_T dummy;
};

#endif                                 /*struct_sMB5GaXc40ozYzvPFeCX3Q*/

#ifndef typedef_c2_coder_internal_cell_6
#define typedef_c2_coder_internal_cell_6

typedef struct sMB5GaXc40ozYzvPFeCX3Q c2_coder_internal_cell_6;

#endif                                 /*typedef_c2_coder_internal_cell_6*/

#ifndef struct_sMB5GaXc40ozYzvPFeCX3Q
#define struct_sMB5GaXc40ozYzvPFeCX3Q

struct sMB5GaXc40ozYzvPFeCX3Q
{
  int32_T dummy;
};

#endif                                 /*struct_sMB5GaXc40ozYzvPFeCX3Q*/

#ifndef typedef_c2_coder_internal_cell_7
#define typedef_c2_coder_internal_cell_7

typedef struct sMB5GaXc40ozYzvPFeCX3Q c2_coder_internal_cell_7;

#endif                                 /*typedef_c2_coder_internal_cell_7*/

#ifndef struct_md5507246048cd485ce2558b2939c43a90
#define struct_md5507246048cd485ce2558b2939c43a90

struct md5507246048cd485ce2558b2939c43a90
{
  int32_T S0_isInitialized;
  int16_T W0_N_PIXLIST_DW[1228800];
  int16_T W1_M_PIXLIST_DW[1228800];
  uint32_T W2_NUM_PIX_DW[50];
  uint8_T W3_PAD_DW[1233284];
  uint32_T W4_STACK_DW[1228800];
  int32_T P0_WALKER_RTP[8];
  uint32_T P1_MINAREA_RTP;
  uint32_T P2_MAXAREA_RTP;
};

#endif                                 /*struct_md5507246048cd485ce2558b2939c43a90*/

#ifndef typedef_c2_vision_BlobAnalysis_1
#define typedef_c2_vision_BlobAnalysis_1

typedef struct md5507246048cd485ce2558b2939c43a90 c2_vision_BlobAnalysis_1;

#endif                                 /*typedef_c2_vision_BlobAnalysis_1*/

#ifndef struct_mdExf0M9yRPLoMMSfshimjVE
#define struct_mdExf0M9yRPLoMMSfshimjVE

struct mdExf0M9yRPLoMMSfshimjVE
{
  int32_T isInitialized;
  c2_vision_BlobAnalysis_1 cSFunObject;
  boolean_T NoTuningBeforeLockingCodeGenError;
};

#endif                                 /*struct_mdExf0M9yRPLoMMSfshimjVE*/

#ifndef typedef_c2_visioncodegen_BlobAnalysis
#define typedef_c2_visioncodegen_BlobAnalysis

typedef struct mdExf0M9yRPLoMMSfshimjVE c2_visioncodegen_BlobAnalysis;

#endif                                 /*typedef_c2_visioncodegen_BlobAnalysis*/

#ifndef typedef_SFc2_mirageTrackingRobotInstanceStruct
#define typedef_SFc2_mirageTrackingRobotInstanceStruct

typedef struct {
  SimStruct *S;
  ChartInfoStruct chartInfo;
  uint32_T chartNumber;
  uint32_T instanceNumber;
  int32_T c2_sfEvent;
  boolean_T c2_isStable;
  boolean_T c2_doneDoubleBufferReInit;
  uint8_T c2_is_active_c2_mirageTrackingRobot;
  boolean_T c2_isInitialized;
  real_T c2_l_rgb1[3686400];
  real_T c2_r_rgb1[3686400];
  c2_visioncodegen_BlobAnalysis c2_hblob;
  real_T c2_l_gray[1228800];
  real_T c2_r_gray[1228800];
  real_T c2_l_rgb2[1228800];
  real_T c2_r_rgb2[1228800];
  real_T c2_dv2[1228800];
  real_T c2_dv3[1228800];
  real_T c2_imgTmp[1228800];
  real_T c2_imgIn[1228800];
  real_T c2_varargin_1[1228800];
  real_T c2_A[1228800];
  uint8_T c2_l_rgb[3686400];
  uint8_T c2_r_rgb[3686400];
  uint8_T c2_b_l_rgb1[3686400];
  uint8_T c2_b_r_rgb1[3686400];
  uint8_T c2_uv0[3686400];
  boolean_T c2_l_bw[1228800];
  boolean_T c2_r_bw[1228800];
  uint8_T c2_b_l_gray[1228800];
  uint8_T c2_b_r_gray[1228800];
  uint8_T c2_b_l_rgb2[1228800];
  uint8_T c2_b_r_rgb2[1228800];
  uint8_T c2_b[1228800];
  uint8_T c2_b_b[1228800];
  uint8_T c2_c_b[1228800];
  uint8_T c2_c_l_rgb2[1228800];
  uint8_T c2_uv1[1228800];
  uint8_T c2_c_r_rgb2[1228800];
  uint8_T c2_uv2[1228800];
  uint8_T c2_d_l_rgb2[1228800];
  uint8_T c2_d_r_rgb2[1228800];
  boolean_T c2_b_l_bw[1228800];
  boolean_T c2_b_r_bw[1228800];
  real_T c2_y[3686400];
  real_T c2_dv8[3686400];
  real_T c2_u[3686400];
  real_T c2_b_y[1228800];
  real_T c2_dv7[1228800];
  real_T c2_b_u[1228800];
  uint8_T c2_c_y[3686400];
  uint8_T c2_uv4[3686400];
  uint8_T c2_c_u[3686400];
  uint8_T c2_a[1233284];
  uint8_T c2_b_varargin_1[1228800];
  boolean_T c2_c_varargin_1[1228800];
  boolean_T c2_d_varargin_1[1228800];
  boolean_T c2_e_varargin_1[1228800];
  boolean_T c2_f_varargin_1[1228800];
  boolean_T c2_d_y[1228800];
  boolean_T c2_bv1[1228800];
  uint8_T c2_e_y[1228800];
  uint8_T c2_uv3[1228800];
  boolean_T c2_d_u[1228800];
  uint8_T c2_e_u[1228800];
  uint8_T (*c2_b_l_rgb)[3686400];
  real_T (*c2_L_p)[8];
  uint8_T (*c2_b_r_rgb)[3686400];
  real_T (*c2_R_p)[8];
} SFc2_mirageTrackingRobotInstanceStruct;

#endif                                 /*typedef_SFc2_mirageTrackingRobotInstanceStruct*/

/* Named Constants */

/* Variable Declarations */
extern struct SfDebugInstanceStruct *sfGlobalDebugInstanceStruct;

/* Variable Definitions */

/* Function Declarations */
extern const mxArray *sf_c2_mirageTrackingRobot_get_eml_resolved_functions_info
  (void);

/* Function Definitions */
extern void sf_c2_mirageTrackingRobot_get_check_sum(mxArray *plhs[]);
extern void c2_mirageTrackingRobot_method_dispatcher(SimStruct *S, int_T method,
  void *data);

#endif
