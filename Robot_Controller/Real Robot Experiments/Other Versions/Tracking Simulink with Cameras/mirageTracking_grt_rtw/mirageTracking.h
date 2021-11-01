/*
 * mirageTracking.h
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

#ifndef RTW_HEADER_mirageTracking_h_
#define RTW_HEADER_mirageTracking_h_
#include <stddef.h>
#include <float.h>
#include <math.h>
#include <string.h>
#ifndef mirageTracking_COMMON_INCLUDES_
# define mirageTracking_COMMON_INCLUDES_
#include "rtwtypes.h"
#include "rtw_continuous.h"
#include "rtw_solver.h"
#include "rt_logging.h"
#include "simaqcodegen.h"
#endif                                 /* mirageTracking_COMMON_INCLUDES_ */

#include "mirageTracking_types.h"

/* Shared type includes */
#include "multiword_types.h"
#include "rtGetInf.h"
#include "rt_nonfinite.h"
#include "rt_defines.h"

/* Macros for accessing real-time model data structure */
#ifndef rtmGetFinalTime
# define rtmGetFinalTime(rtm)          ((rtm)->Timing.tFinal)
#endif

#ifndef rtmGetRTWLogInfo
# define rtmGetRTWLogInfo(rtm)         ((rtm)->rtwLogInfo)
#endif

#ifndef rtmGetErrorStatus
# define rtmGetErrorStatus(rtm)        ((rtm)->errorStatus)
#endif

#ifndef rtmSetErrorStatus
# define rtmSetErrorStatus(rtm, val)   ((rtm)->errorStatus = (val))
#endif

#ifndef rtmGetStopRequested
# define rtmGetStopRequested(rtm)      ((rtm)->Timing.stopRequestedFlag)
#endif

#ifndef rtmSetStopRequested
# define rtmSetStopRequested(rtm, val) ((rtm)->Timing.stopRequestedFlag = (val))
#endif

#ifndef rtmGetStopRequestedPtr
# define rtmGetStopRequestedPtr(rtm)   (&((rtm)->Timing.stopRequestedFlag))
#endif

#ifndef rtmGetT
# define rtmGetT(rtm)                  (rtmGetTPtr((rtm))[0])
#endif

#ifndef rtmGetTFinal
# define rtmGetTFinal(rtm)             ((rtm)->Timing.tFinal)
#endif

/* Block signals (auto storage) */
typedef struct {
  real32_T r_rgb1[3686400];
  visioncodegen_BlobAnalysis_mi_T hblob;
  real_T dv0[1228800];
  real32_T a[1233284];
  real32_T b_pad[1233284];
  real32_T l_rgb1[1228800];
  real32_T fv0[1228800];
  boolean_T bv0[1228800];
  real_T qtilde[6];                    /* '<Root>/Estimate Pose Error' */
  real_T L_p[8];                       /* '<Root>/Calculate Target Points' */
  real_T R_p[8];                       /* '<Root>/Calculate Target Points' */
  real32_T LeftCamera[3686400];        /* '<Root>/Left Camera' */
  real32_T RightCamera[3686400];       /* '<Root>/Right Camera' */
  real32_T l_rgb1_m[3686400];
} B_mirageTracking_T;

/* Block states (auto storage) for system '<Root>' */
typedef struct {
  real_T LeftCamera_FromVideoDevice[137];/* '<Root>/Left Camera' */
  real_T RightCamera_FromVideoDevice[137];/* '<Root>/Right Camera' */
  void* LeftCamera_IMAQObject;         /* '<Root>/Left Camera' */
  void* RightCamera_IMAQObject;        /* '<Root>/Right Camera' */
  struct {
    void *LoggedData;
  } PoseError_PWORK;                   /* '<Root>/Pose Error' */
} DW_mirageTracking_T;

/* Parameters (auto storage) */
struct P_mirageTracking_T_ {
  real_T ObjectModel_Value[16];        /* Expression: [-9 -9 13.5 0; 12.75 -12.75 0 0; 2 1.75 2 22.5; 1 1 1 1];
                                        * Referenced by: '<Root>/Object Model'
                                        */
};

/* Real-time Model Data Structure */
struct tag_RTM_mirageTracking_T {
  const char_T *errorStatus;
  RTWLogInfo *rtwLogInfo;
  RTWSolverInfo solverInfo;

  /*
   * Timing:
   * The following substructure contains information regarding
   * the timing information for the model.
   */
  struct {
    uint32_T clockTick0;
    uint32_T clockTickH0;
    time_T stepSize0;
    uint32_T clockTick1;
    uint32_T clockTickH1;
    time_T tFinal;
    SimTimeStep simTimeStep;
    boolean_T stopRequestedFlag;
    time_T *t;
    time_T tArray[2];
  } Timing;
};

/* Block parameters (auto storage) */
extern P_mirageTracking_T mirageTracking_P;

/* Block signals (auto storage) */
extern B_mirageTracking_T mirageTracking_B;

/* Block states (auto storage) */
extern DW_mirageTracking_T mirageTracking_DW;

/* Model entry point functions */
extern void mirageTracking_initialize(void);
extern void mirageTracking_step(void);
extern void mirageTracking_terminate(void);

/* Real-time Model object */
extern RT_MODEL_mirageTracking_T *const mirageTracking_M;

/*-
 * The generated code includes comments that allow you to trace directly
 * back to the appropriate location in the model.  The basic format
 * is <system>/block_name, where system is the system number (uniquely
 * assigned by Simulink) and block_name is the name of the block.
 *
 * Use the MATLAB hilite_system command to trace the generated code back
 * to the model.  For example,
 *
 * hilite_system('<S3>')    - opens system 3
 * hilite_system('<S3>/Kp') - opens and selects block Kp which resides in S3
 *
 * Here is the system hierarchy for this model
 *
 * '<Root>' : 'mirageTracking'
 * '<S1>'   : 'mirageTracking/Calculate Target Points'
 * '<S2>'   : 'mirageTracking/Estimate Pose Error'
 * '<S3>'   : 'mirageTracking/MATLAB Function'
 */
#endif                                 /* RTW_HEADER_mirageTracking_h_ */
