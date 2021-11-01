/*
 * mirageTrackingRobot_data.c
 *
 * Academic License - for use in teaching, academic research, and meeting
 * course requirements at degree granting institutions only.  Not for
 * government, commercial, or other organizational use.
 *
 * Code generation for model "mirageTrackingRobot".
 *
 * Model version              : 1.14
 * Simulink Coder version : 8.8 (R2015a) 09-Feb-2015
 * C source code generated on : Thu Aug 20 12:41:56 2015
 *
 * Target selection: slrt.tlc
 * Note: GRT includes extra infrastructure and instrumentation for prototyping
 * Embedded hardware selection: Generic->32-bit x86 compatible
 * Code generation objectives: Unspecified
 * Validation result: Not run
 */

#include "mirageTrackingRobot.h"
#include "mirageTrackingRobot_private.h"

/* Block parameters (auto storage) */
P_mirageTrackingRobot_T mirageTrackingRobot_P = {
  /*  Computed Parameter: FromLeftCamera_P1_Size
   * Referenced by: '<Root>/From Left Camera'
   */
  { 1.0, 1.0 },
  1.0,                                 /* Expression: isig
                                        * Referenced by: '<Root>/From Left Camera'
                                        */

  /*  Computed Parameter: FromLeftCamera_P2_Size
   * Referenced by: '<Root>/From Left Camera'
   */
  { 1.0, 1.0 },
  1280.0,                              /* Expression: iwidth
                                        * Referenced by: '<Root>/From Left Camera'
                                        */

  /*  Computed Parameter: FromLeftCamera_P3_Size
   * Referenced by: '<Root>/From Left Camera'
   */
  { 1.0, 1.0 },
  960.0,                               /* Expression: iheight
                                        * Referenced by: '<Root>/From Left Camera'
                                        */

  /*  Computed Parameter: FromLeftCamera_P4_Size
   * Referenced by: '<Root>/From Left Camera'
   */
  { 1.0, 1.0 },
  8.0,                                 /* Expression: fint
                                        * Referenced by: '<Root>/From Left Camera'
                                        */

  /*  Computed Parameter: FromLeftCamera_P5_Size
   * Referenced by: '<Root>/From Left Camera'
   */
  { 1.0, 1.0 },
  1.0,                                 /* Expression: ffmt
                                        * Referenced by: '<Root>/From Left Camera'
                                        */

  /*  Computed Parameter: FromLeftCamera_P6_Size
   * Referenced by: '<Root>/From Left Camera'
   */
  { 1.0, 1.0 },
  1.0,                                 /* Expression: cfmt
                                        * Referenced by: '<Root>/From Left Camera'
                                        */

  /*  Computed Parameter: FromLeftCamera_P7_Size
   * Referenced by: '<Root>/From Left Camera'
   */
  { 1.0, 1.0 },
  0.0,                                 /* Expression: trigger
                                        * Referenced by: '<Root>/From Left Camera'
                                        */

  /*  Computed Parameter: FromLeftCamera_P8_Size
   * Referenced by: '<Root>/From Left Camera'
   */
  { 1.0, 1.0 },
  0.0,                                 /* Expression: status
                                        * Referenced by: '<Root>/From Left Camera'
                                        */

  /*  Computed Parameter: FromLeftCamera_P9_Size
   * Referenced by: '<Root>/From Left Camera'
   */
  { 1.0, 2.0 },

  /*  Computed Parameter: FromLeftCamera_P9
   * Referenced by: '<Root>/From Left Camera'
   */
  { 45.0, 49.0 },

  /*  Computed Parameter: FromRightCamera_P1_Size
   * Referenced by: '<Root>/From Right Camera'
   */
  { 1.0, 1.0 },
  1.0,                                 /* Expression: isig
                                        * Referenced by: '<Root>/From Right Camera'
                                        */

  /*  Computed Parameter: FromRightCamera_P2_Size
   * Referenced by: '<Root>/From Right Camera'
   */
  { 1.0, 1.0 },
  1280.0,                              /* Expression: iwidth
                                        * Referenced by: '<Root>/From Right Camera'
                                        */

  /*  Computed Parameter: FromRightCamera_P3_Size
   * Referenced by: '<Root>/From Right Camera'
   */
  { 1.0, 1.0 },
  960.0,                               /* Expression: iheight
                                        * Referenced by: '<Root>/From Right Camera'
                                        */

  /*  Computed Parameter: FromRightCamera_P4_Size
   * Referenced by: '<Root>/From Right Camera'
   */
  { 1.0, 1.0 },
  8.0,                                 /* Expression: fint
                                        * Referenced by: '<Root>/From Right Camera'
                                        */

  /*  Computed Parameter: FromRightCamera_P5_Size
   * Referenced by: '<Root>/From Right Camera'
   */
  { 1.0, 1.0 },
  1.0,                                 /* Expression: ffmt
                                        * Referenced by: '<Root>/From Right Camera'
                                        */

  /*  Computed Parameter: FromRightCamera_P6_Size
   * Referenced by: '<Root>/From Right Camera'
   */
  { 1.0, 1.0 },
  1.0,                                 /* Expression: cfmt
                                        * Referenced by: '<Root>/From Right Camera'
                                        */

  /*  Computed Parameter: FromRightCamera_P7_Size
   * Referenced by: '<Root>/From Right Camera'
   */
  { 1.0, 1.0 },
  0.0,                                 /* Expression: trigger
                                        * Referenced by: '<Root>/From Right Camera'
                                        */

  /*  Computed Parameter: FromRightCamera_P8_Size
   * Referenced by: '<Root>/From Right Camera'
   */
  { 1.0, 1.0 },
  0.0,                                 /* Expression: status
                                        * Referenced by: '<Root>/From Right Camera'
                                        */

  /*  Computed Parameter: FromRightCamera_P9_Size
   * Referenced by: '<Root>/From Right Camera'
   */
  { 1.0, 2.0 },

  /*  Computed Parameter: FromRightCamera_P9
   * Referenced by: '<Root>/From Right Camera'
   */
  { 45.0, 49.0 },

  /*  Expression: [-395 2 45 0 0 0]
   * Referenced by: '<Root>/Desired Robot Pose'
   */
  { -395.0, 2.0, 45.0, 0.0, 0.0, 0.0 },

  /*  Expression: [-9 -9 13.5 0; 12.75 -12.75 0 0; 2 1.75 2 22.5; 1 1 1 1]
   * Referenced by: '<Root>/Object Model'
   */
  { -9.0, 12.75, 2.0, 1.0, -9.0, -12.75, 1.75, 1.0, 13.5, 0.0, 2.0, 1.0, 0.0,
    0.0, 22.5, 1.0 }
};
