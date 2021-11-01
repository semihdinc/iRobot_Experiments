/*
 * mirageTracking.h
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

#ifndef RTW_HEADER_mirageTracking_h_
#define RTW_HEADER_mirageTracking_h_
#include <stddef.h>
#include <math.h>
#include <string.h>
#include <float.h>
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
  real32_T leftImage[4243200];
  real32_T rightImage[4243200];
  real32_T fv0[3686400];
  real32_T pWinR[19683];
  visioncodegen_BlobAnalysis_mi_T hblob;
  real32_T pWinL[6561];
  boolean_T pBWL[6561];
  boolean_T pBWR[6561];
  real_T qtilde[6];                    /* '<Root>/Estimate Pose Error' */
  real_T L_p[8];                       /* '<Root>/Calculate Target Points Version 3' */
  real_T R_p[8];                       /* '<Root>/Calculate Target Points Version 3' */
  real32_T LeftCamera[3686400];        /* '<Root>/Left Camera' */
  real32_T RightCamera[3686400];       /* '<Root>/Right Camera' */
  real32_T pWinL_m[19683];
} B_mirageTracking_T;

/* Block states (auto storage) for system '<Root>' */
typedef struct {
  real_T LeftCamera_FromVideoDevice[137];/* '<Root>/Left Camera' */
  real_T RightCamera_FromVideoDevice[137];/* '<Root>/Right Camera' */
  real_T LeftCamCenters_PreviousInput[8];/* '<Root>/Left Cam Centers' */
  real_T RightCamCenters_PreviousInput[8];/* '<Root>/Right Cam Centers' */
  void* LeftCamera_IMAQObject;         /* '<Root>/Left Camera' */
  void* RightCamera_IMAQObject;        /* '<Root>/Right Camera' */
  struct {
    void *LoggedData;
  } PoseError_PWORK;                   /* '<Root>/Pose Error' */
} DW_mirageTracking_T;

/* Parameters (auto storage) */
struct P_mirageTracking_T_ {
  real_T LeftCamCenters_X0[8];         /* Expression: [502.685344827586,883.099362579678,677.620915032680,683.780520052887;809.187140804598,812.931758530184,756.506535947713,501.432789775231]
                                        * Referenced by: '<Root>/Left Cam Centers'
                                        */
  real_T RightCamCenters_X0[8];        /* Expression: [345.585050646175,726.556903317874,552.454316320101,537.066725197542;885.680405169403,883.749197288619,830.574669187146,575.655399473222]
                                        * Referenced by: '<Root>/Right Cam Centers'
                                        */
  real_T ObjectModel_Value[16];        /* Expression: [-9 -9 13.5 0; 12.75 -12.75 0 0; 2 1.75 2 22.5; 1 1 1 1]
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
 * '<S2>'   : 'mirageTracking/Calculate Target Points Version 2'
 * '<S3>'   : 'mirageTracking/Calculate Target Points Version 3'
 * '<S4>'   : 'mirageTracking/Estimate Pose Error'
 * '<S5>'   : 'mirageTracking/MATLAB Function'
 */
#endif                                 /* RTW_HEADER_mirageTracking_h_ */
