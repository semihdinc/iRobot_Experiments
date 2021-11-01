#ifndef __c1_mirageTracking_h__
#define __c1_mirageTracking_h__

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

#ifndef typedef_c1_coder_internal_cell_4
#define typedef_c1_coder_internal_cell_4

typedef struct sMB5GaXc40ozYzvPFeCX3Q c1_coder_internal_cell_4;

#endif                                 /*typedef_c1_coder_internal_cell_4*/

#ifndef struct_sMB5GaXc40ozYzvPFeCX3Q
#define struct_sMB5GaXc40ozYzvPFeCX3Q

struct sMB5GaXc40ozYzvPFeCX3Q
{
  int32_T dummy;
};

#endif                                 /*struct_sMB5GaXc40ozYzvPFeCX3Q*/

#ifndef typedef_c1_coder_internal_cell_5
#define typedef_c1_coder_internal_cell_5

typedef struct sMB5GaXc40ozYzvPFeCX3Q c1_coder_internal_cell_5;

#endif                                 /*typedef_c1_coder_internal_cell_5*/

#ifndef struct_sMB5GaXc40ozYzvPFeCX3Q
#define struct_sMB5GaXc40ozYzvPFeCX3Q

struct sMB5GaXc40ozYzvPFeCX3Q
{
  int32_T dummy;
};

#endif                                 /*struct_sMB5GaXc40ozYzvPFeCX3Q*/

#ifndef typedef_c1_coder_internal_cell_6
#define typedef_c1_coder_internal_cell_6

typedef struct sMB5GaXc40ozYzvPFeCX3Q c1_coder_internal_cell_6;

#endif                                 /*typedef_c1_coder_internal_cell_6*/

#ifndef struct_sMB5GaXc40ozYzvPFeCX3Q
#define struct_sMB5GaXc40ozYzvPFeCX3Q

struct sMB5GaXc40ozYzvPFeCX3Q
{
  int32_T dummy;
};

#endif                                 /*struct_sMB5GaXc40ozYzvPFeCX3Q*/

#ifndef typedef_c1_coder_internal_cell_7
#define typedef_c1_coder_internal_cell_7

typedef struct sMB5GaXc40ozYzvPFeCX3Q c1_coder_internal_cell_7;

#endif                                 /*typedef_c1_coder_internal_cell_7*/

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

#ifndef typedef_c1_vision_BlobAnalysis_1
#define typedef_c1_vision_BlobAnalysis_1

typedef struct md5507246048cd485ce2558b2939c43a90 c1_vision_BlobAnalysis_1;

#endif                                 /*typedef_c1_vision_BlobAnalysis_1*/

#ifndef struct_mdExf0M9yRPLoMMSfshimjVE
#define struct_mdExf0M9yRPLoMMSfshimjVE

struct mdExf0M9yRPLoMMSfshimjVE
{
  int32_T isInitialized;
  c1_vision_BlobAnalysis_1 cSFunObject;
  boolean_T NoTuningBeforeLockingCodeGenError;
};

#endif                                 /*struct_mdExf0M9yRPLoMMSfshimjVE*/

#ifndef typedef_c1_visioncodegen_BlobAnalysis
#define typedef_c1_visioncodegen_BlobAnalysis

typedef struct mdExf0M9yRPLoMMSfshimjVE c1_visioncodegen_BlobAnalysis;

#endif                                 /*typedef_c1_visioncodegen_BlobAnalysis*/

#ifndef typedef_SFc1_mirageTrackingInstanceStruct
#define typedef_SFc1_mirageTrackingInstanceStruct

typedef struct {
  SimStruct *S;
  ChartInfoStruct chartInfo;
  uint32_T chartNumber;
  uint32_T instanceNumber;
  int32_T c1_sfEvent;
  boolean_T c1_isStable;
  boolean_T c1_doneDoubleBufferReInit;
  uint8_T c1_is_active_c1_mirageTracking;
  boolean_T c1_isInitialized;
  real_T c1_l_rgb1[3686400];
  real_T c1_r_rgb1[3686400];
  real32_T c1_l_rgb[3686400];
  real32_T c1_r_rgb[3686400];
  real32_T c1_b_l_rgb1[3686400];
  real32_T c1_b_r_rgb1[3686400];
  real32_T c1_fv0[3686400];
  c1_visioncodegen_BlobAnalysis c1_hblob;
  real_T c1_l_gray[1228800];
  real_T c1_r_gray[1228800];
  real_T c1_l_rgb2[1228800];
  real_T c1_r_rgb2[1228800];
  real_T c1_dv2[1228800];
  real_T c1_dv3[1228800];
  real_T c1_imgTmp[1228800];
  real_T c1_imgIn[1228800];
  real_T c1_varargin_1[1228800];
  real_T c1_A[1228800];
  real32_T c1_b_l_gray[1228800];
  real32_T c1_b_r_gray[1228800];
  real32_T c1_b_l_rgb2[1228800];
  real32_T c1_b_r_rgb2[1228800];
  real32_T c1_b[1228800];
  real32_T c1_b_b[1228800];
  real32_T c1_c_b[1228800];
  real32_T c1_c_l_rgb2[1228800];
  real32_T c1_fv1[1228800];
  real32_T c1_c_r_rgb2[1228800];
  real32_T c1_fv2[1228800];
  real32_T c1_d_l_rgb2[1228800];
  real32_T c1_d_r_rgb2[1228800];
  boolean_T c1_l_bw[1228800];
  boolean_T c1_r_bw[1228800];
  boolean_T c1_b_l_bw[1228800];
  boolean_T c1_b_r_bw[1228800];
  real_T c1_y[3686400];
  real_T c1_dv8[3686400];
  real32_T c1_b_y[3686400];
  real32_T c1_fv4[3686400];
  real_T c1_u[3686400];
  real_T c1_c_y[1228800];
  real_T c1_dv7[1228800];
  real32_T c1_b_u[3686400];
  real32_T c1_a[1233284];
  real32_T c1_b_varargin_1[1228800];
  real32_T c1_d_y[1228800];
  real32_T c1_fv3[1228800];
  real_T c1_c_u[1228800];
  real32_T c1_d_u[1228800];
  boolean_T c1_c_varargin_1[1228800];
  boolean_T c1_d_varargin_1[1228800];
  boolean_T c1_e_varargin_1[1228800];
  boolean_T c1_f_varargin_1[1228800];
  boolean_T c1_e_y[1228800];
  boolean_T c1_bv1[1228800];
  boolean_T c1_e_u[1228800];
  real32_T (*c1_b_l_rgb)[3686400];
  real_T (*c1_L_p)[8];
  real32_T (*c1_b_r_rgb)[3686400];
  real_T (*c1_R_p)[8];
} SFc1_mirageTrackingInstanceStruct;

#endif                                 /*typedef_SFc1_mirageTrackingInstanceStruct*/

/* Named Constants */

/* Variable Declarations */
extern struct SfDebugInstanceStruct *sfGlobalDebugInstanceStruct;

/* Variable Definitions */

/* Function Declarations */
extern const mxArray *sf_c1_mirageTracking_get_eml_resolved_functions_info(void);

/* Function Definitions */
extern void sf_c1_mirageTracking_get_check_sum(mxArray *plhs[]);
extern void c1_mirageTracking_method_dispatcher(SimStruct *S, int_T method, void
  *data);

#endif
