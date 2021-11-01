/*
 * mirageTracking_types.h
 *
 * Academic License - for use in teaching, academic research, and meeting
 * course requirements at degree granting institutions only.  Not for
 * government, commercial, or other organizational use.
 *
 * Code generation for model "mirageTracking".
 *
 * Model version              : 1.38
 * Simulink Coder version : 8.8 (R2015a) 09-Feb-2015
 * C source code generated on : Thu Aug 20 09:32:58 2015
 *
 * Target selection: grt.tlc
 * Note: GRT includes extra infrastructure and instrumentation for prototyping
 * Embedded hardware selection: 32-bit Generic
 * Code generation objectives: Unspecified
 * Validation result: Not run
 */

#ifndef RTW_HEADER_mirageTracking_types_h_
#define RTW_HEADER_mirageTracking_types_h_
#include "rtwtypes.h"
#include "builtin_typeid_types.h"
#include "multiword_types.h"

/* Custom Type definition for MATLAB Function: '<Root>/Calculate Target Points' */
#ifndef struct_sPmiUZY2qQ0dLsc3OKaugUE
#define struct_sPmiUZY2qQ0dLsc3OKaugUE

struct sPmiUZY2qQ0dLsc3OKaugUE
{
  real_T DEFAULT;
  real_T TOPLEFT;
  real_T BOTTOMRIGHT;
};

#endif                                 /*struct_sPmiUZY2qQ0dLsc3OKaugUE*/

#ifndef typedef_sPmiUZY2qQ0dLsc3OKaugUE_mirag_T
#define typedef_sPmiUZY2qQ0dLsc3OKaugUE_mirag_T

typedef struct sPmiUZY2qQ0dLsc3OKaugUE sPmiUZY2qQ0dLsc3OKaugUE_mirag_T;

#endif                                 /*typedef_sPmiUZY2qQ0dLsc3OKaugUE_mirag_T*/

#ifndef struct_sho5krkycRf4zNoOFwnsnyC
#define struct_sho5krkycRf4zNoOFwnsnyC

struct sho5krkycRf4zNoOFwnsnyC
{
  boolean_T YES;
  boolean_T NO;
};

#endif                                 /*struct_sho5krkycRf4zNoOFwnsnyC*/

#ifndef typedef_sho5krkycRf4zNoOFwnsnyC_mirag_T
#define typedef_sho5krkycRf4zNoOFwnsnyC_mirag_T

typedef struct sho5krkycRf4zNoOFwnsnyC sho5krkycRf4zNoOFwnsnyC_mirag_T;

#endif                                 /*typedef_sho5krkycRf4zNoOFwnsnyC_mirag_T*/

#ifndef struct_s0EJTkZ9FbbyaWo4ybLegeE
#define struct_s0EJTkZ9FbbyaWo4ybLegeE

struct s0EJTkZ9FbbyaWo4ybLegeE
{
  real_T NONE;
};

#endif                                 /*struct_s0EJTkZ9FbbyaWo4ybLegeE*/

#ifndef typedef_s0EJTkZ9FbbyaWo4ybLegeE_mirag_T
#define typedef_s0EJTkZ9FbbyaWo4ybLegeE_mirag_T

typedef struct s0EJTkZ9FbbyaWo4ybLegeE s0EJTkZ9FbbyaWo4ybLegeE_mirag_T;

#endif                                 /*typedef_s0EJTkZ9FbbyaWo4ybLegeE_mirag_T*/

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

#ifndef typedef_vision_BlobAnalysis_1_mirageT_T
#define typedef_vision_BlobAnalysis_1_mirageT_T

typedef struct md5507246048cd485ce2558b2939c43a90
  vision_BlobAnalysis_1_mirageT_T;

#endif                                 /*typedef_vision_BlobAnalysis_1_mirageT_T*/

#ifndef struct_mdExf0M9yRPLoMMSfshimjVE
#define struct_mdExf0M9yRPLoMMSfshimjVE

struct mdExf0M9yRPLoMMSfshimjVE
{
  int32_T isInitialized;
  vision_BlobAnalysis_1_mirageT_T cSFunObject;
  boolean_T NoTuningBeforeLockingCodeGenError;
};

#endif                                 /*struct_mdExf0M9yRPLoMMSfshimjVE*/

#ifndef typedef_visioncodegen_BlobAnalysis_mi_T
#define typedef_visioncodegen_BlobAnalysis_mi_T

typedef struct mdExf0M9yRPLoMMSfshimjVE visioncodegen_BlobAnalysis_mi_T;

#endif                                 /*typedef_visioncodegen_BlobAnalysis_mi_T*/

/* Custom Type definition for MATLAB Function: '<Root>/Estimate Pose Error' */
#ifndef struct_smuwpGFfhzM6MfA0WGyOk6E
#define struct_smuwpGFfhzM6MfA0WGyOk6E

struct smuwpGFfhzM6MfA0WGyOk6E
{
  real_T x;
  real_T y;
  real_T z;
  real_T phi;
  real_T theta;
  real_T psi;
};

#endif                                 /*struct_smuwpGFfhzM6MfA0WGyOk6E*/

#ifndef typedef_smuwpGFfhzM6MfA0WGyOk6E_mirag_T
#define typedef_smuwpGFfhzM6MfA0WGyOk6E_mirag_T

typedef struct smuwpGFfhzM6MfA0WGyOk6E smuwpGFfhzM6MfA0WGyOk6E_mirag_T;

#endif                                 /*typedef_smuwpGFfhzM6MfA0WGyOk6E_mirag_T*/

/* Parameters (auto storage) */
typedef struct P_mirageTracking_T_ P_mirageTracking_T;

/* Forward declaration for rtModel */
typedef struct tag_RTM_mirageTracking_T RT_MODEL_mirageTracking_T;

#endif                                 /* RTW_HEADER_mirageTracking_types_h_ */
