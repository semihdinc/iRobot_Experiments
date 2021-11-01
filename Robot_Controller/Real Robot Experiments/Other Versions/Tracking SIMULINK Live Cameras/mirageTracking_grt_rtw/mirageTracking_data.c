/*
 * mirageTracking_data.c
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

#include "mirageTracking.h"
#include "mirageTracking_private.h"

/* Block parameters (auto storage) */
P_mirageTracking_T mirageTracking_P = {
  /*  Expression: [502.685344827586,883.099362579678,677.620915032680,683.780520052887;809.187140804598,812.931758530184,756.506535947713,501.432789775231]
   * Referenced by: '<Root>/Left Cam Centers'
   */
  { 502.685344827586, 809.187140804598, 883.099362579678, 812.931758530184,
    677.62091503268, 756.506535947713, 683.780520052887, 501.432789775231 },

  /*  Expression: [345.585050646175,726.556903317874,552.454316320101,537.066725197542;885.680405169403,883.749197288619,830.574669187146,575.655399473222]
   * Referenced by: '<Root>/Right Cam Centers'
   */
  { 345.585050646175, 885.680405169403, 726.556903317874, 883.749197288619,
    552.454316320101, 830.574669187146, 537.066725197542, 575.655399473222 },

  /*  Expression: [-9 -9 13.5 0; 12.75 -12.75 0 0; 2 1.75 2 22.5; 1 1 1 1]
   * Referenced by: '<Root>/Object Model'
   */
  { -9.0, 12.75, 2.0, 1.0, -9.0, -12.75, 1.75, 1.0, 13.5, 0.0, 2.0, 1.0, 0.0,
    0.0, 22.5, 1.0 }
};
