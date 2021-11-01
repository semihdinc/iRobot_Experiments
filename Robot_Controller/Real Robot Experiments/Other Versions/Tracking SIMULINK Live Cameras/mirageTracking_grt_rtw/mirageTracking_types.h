/*
 * mirageTracking_types.h
 *
 * Academic License - for use in teaching, academic research, and meeting
 * course requirements at degree granting institutions only.  Not for
 * government, commercial, or other organizational use.
 *
 * Code generation for model "mirageTracking".
 *
 * Model version              : 1.198
 * Simulink Coder version : 8.8 (R2015a) 09-Feb-2015
 * C source code generated on : Mon Aug 31 14:36:40 2015
 *
 * Target selection: grt.tlc
 * Note: GRT includes extra infrastructure and instrumentation for prototyping
 * Embedded hardware selection: 32-bit Generic
 * Code generation objective: Execution efficiency
 * Validation result: Not run
 */

#ifndef RTW_HEADER_mirageTracking_types_h_
#define RTW_HEADER_mirageTracking_types_h_
#include "rtwtypes.h"
#include "builtin_typeid_types.h"
#include "multiword_types.h"

/* Custom Type definition for MATLAB Function: '<Root>/Calculate Target Points Version 3' */
#ifndef struct_md4a3bc705e9a5c5f2f5544ca0b29a96b8
#define struct_md4a3bc705e9a5c5f2f5544ca0b29a96b8

struct md4a3bc705e9a5c5f2f5544ca0b29a96b8
{
  int32_T S0_isInitialized;
  real32_T W0_P_DW[256];
  real32_T W1_MU_DW[256];
  real32_T P0_UMIN_RTP;
  real32_T P1_UMAX_RTP;
};

#endif                                 /*struct_md4a3bc705e9a5c5f2f5544ca0b29a96b8*/

#ifndef typedef_vision_Autothresholder_1_mira_T
#define typedef_vision_Autothresholder_1_mira_T

typedef struct md4a3bc705e9a5c5f2f5544ca0b29a96b8
  vision_Autothresholder_1_mira_T;

#endif                                 /*typedef_vision_Autothresholder_1_mira_T*/

#ifndef struct_mdcf6b0f7b2a365b831e863b0db8a2ad4
#define struct_mdcf6b0f7b2a365b831e863b0db8a2ad4

struct mdcf6b0f7b2a365b831e863b0db8a2ad4
{
  int32_T S0_isInitialized;
  int16_T W0_N_PIXLIST_DW[6561];
  int16_T W1_M_PIXLIST_DW[6561];
  uint32_T W2_NUM_PIX_DW[50];
  uint8_T W3_PAD_DW[6889];
  uint32_T W4_STACK_DW[6561];
  int32_T P0_WALKER_RTP[8];
  uint32_T P1_MINAREA_RTP;
  uint32_T P2_MAXAREA_RTP;
};

#endif                                 /*struct_mdcf6b0f7b2a365b831e863b0db8a2ad4*/

#ifndef typedef_vision_BlobAnalysis_3_mirageT_T
#define typedef_vision_BlobAnalysis_3_mirageT_T

typedef struct mdcf6b0f7b2a365b831e863b0db8a2ad4 vision_BlobAnalysis_3_mirageT_T;

#endif                                 /*typedef_vision_BlobAnalysis_3_mirageT_T*/

#ifndef struct_mdsSajGWyrgG4QH2ZfbV1SFE
#define struct_mdsSajGWyrgG4QH2ZfbV1SFE

struct mdsSajGWyrgG4QH2ZfbV1SFE
{
  int32_T isInitialized;
  vision_BlobAnalysis_3_mirageT_T cSFunObject;
  boolean_T NoTuningBeforeLockingCodeGenError;
};

#endif                                 /*struct_mdsSajGWyrgG4QH2ZfbV1SFE*/

#ifndef typedef_visioncodegen_BlobAnalysis_mi_T
#define typedef_visioncodegen_BlobAnalysis_mi_T

typedef struct mdsSajGWyrgG4QH2ZfbV1SFE visioncodegen_BlobAnalysis_mi_T;

#endif                                 /*typedef_visioncodegen_BlobAnalysis_mi_T*/

#ifndef struct_mdz5zfzZGesKqn61NdA4eqcC
#define struct_mdz5zfzZGesKqn61NdA4eqcC

struct mdz5zfzZGesKqn61NdA4eqcC
{
  int32_T isInitialized;
  vision_Autothresholder_1_mira_T cSFunObject;
  boolean_T NoTuningBeforeLockingCodeGenError;
};

#endif                                 /*struct_mdz5zfzZGesKqn61NdA4eqcC*/

#ifndef typedef_visioncodegen_Autothresholder_T
#define typedef_visioncodegen_Autothresholder_T

typedef struct mdz5zfzZGesKqn61NdA4eqcC visioncodegen_Autothresholder_T;

#endif                                 /*typedef_visioncodegen_Autothresholder_T*/

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
