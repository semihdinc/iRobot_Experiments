/*
 * RobotControl1_640_480_types.h
 *
 * Academic License - for use in teaching, academic research, and meeting
 * course requirements at degree granting institutions only.  Not for
 * government, commercial, or other organizational use.
 *
 * Code generation for model "RobotControl1_640_480".
 *
 * Model version              : 1.1352
 * Simulink Coder version : 8.8 (R2015a) 09-Feb-2015
 * C source code generated on : Tue Oct 06 10:25:36 2015
 *
 * Target selection: slrt.tlc
 * Note: GRT includes extra infrastructure and instrumentation for prototyping
 * Embedded hardware selection: Generic->32-bit x86 compatible
 * Code generation objective: Execution efficiency
 * Validation result: Not run
 */

#ifndef RTW_HEADER_RobotControl1_640_480_types_h_
#define RTW_HEADER_RobotControl1_640_480_types_h_
#include "rtwtypes.h"
#include "builtin_typeid_types.h"
#include "multiword_types.h"
#include "zero_crossing_types.h"

/* Custom Type definition for MATLAB Function: '<S10>/Calculate Target Points' */
#ifndef struct_mdc72aa3d8fa348809c647c0e2d37c158
#define struct_mdc72aa3d8fa348809c647c0e2d37c158

struct mdc72aa3d8fa348809c647c0e2d37c158
{
  int32_T S0_isInitialized;
  int16_T W0_N_PIXLIST_DW[307200];
  int16_T W1_M_PIXLIST_DW[307200];
  uint32_T W2_NUM_PIX_DW[50];
  uint8_T W3_PAD_DW[309444];
  uint32_T W4_STACK_DW[307200];
  int32_T P0_WALKER_RTP[8];
  uint32_T P1_MINAREA_RTP;
  uint32_T P2_MAXAREA_RTP;
};

#endif                                 /*struct_mdc72aa3d8fa348809c647c0e2d37c158*/

#ifndef typedef_vision_BlobAnalysis_1_RobotControl1_640_480_T
#define typedef_vision_BlobAnalysis_1_RobotControl1_640_480_T

typedef struct mdc72aa3d8fa348809c647c0e2d37c158
  vision_BlobAnalysis_1_RobotControl1_640_480_T;

#endif                                 /*typedef_vision_BlobAnalysis_1_RobotControl1_640_480_T*/

#ifndef struct_mdIMzzzmZ0oFJkrlFW42vcAC
#define struct_mdIMzzzmZ0oFJkrlFW42vcAC

struct mdIMzzzmZ0oFJkrlFW42vcAC
{
  int32_T isInitialized;
  vision_BlobAnalysis_1_RobotControl1_640_480_T cSFunObject;
  boolean_T NoTuningBeforeLockingCodeGenError;
};

#endif                                 /*struct_mdIMzzzmZ0oFJkrlFW42vcAC*/

#ifndef typedef_visioncodegen_BlobAnalysis_RobotControl1_640_480_T
#define typedef_visioncodegen_BlobAnalysis_RobotControl1_640_480_T

typedef struct mdIMzzzmZ0oFJkrlFW42vcAC
  visioncodegen_BlobAnalysis_RobotControl1_640_480_T;

#endif                                 /*typedef_visioncodegen_BlobAnalysis_RobotControl1_640_480_T*/

/* Custom Type definition for MATLAB Function: '<S10>/Pose Estimation' */
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

#ifndef typedef_smuwpGFfhzM6MfA0WGyOk6E_RobotControl1_640_480_T
#define typedef_smuwpGFfhzM6MfA0WGyOk6E_RobotControl1_640_480_T

typedef struct smuwpGFfhzM6MfA0WGyOk6E
  smuwpGFfhzM6MfA0WGyOk6E_RobotControl1_640_480_T;

#endif                                 /*typedef_smuwpGFfhzM6MfA0WGyOk6E_RobotControl1_640_480_T*/

/* Parameters (auto storage) */
typedef struct P_RobotControl1_640_480_T_ P_RobotControl1_640_480_T;

/* Forward declaration for rtModel */
typedef struct tag_RTM_RobotControl1_640_480_T RT_MODEL_RobotControl1_640_480_T;

#endif                                 /* RTW_HEADER_RobotControl1_640_480_types_h_ */
