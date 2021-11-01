/*
 * RobotControl1_640_480_data.c
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

#include "RobotControl1_640_480.h"
#include "RobotControl1_640_480_private.h"

/* Block parameters (auto storage) */
P_RobotControl1_640_480_T RobotControl1_640_480_P = {
  /*  Mask Parameter: TappedDelay_vinit
   * Referenced by: '<S12>/Tapped Delay'
   */
  { 0.0, 0.0 },

  /*  Computed Parameter: RS232BinarySend1_P1_Size
   * Referenced by: '<S35>/RS232 Binary Send1'
   */
  { 1.0, 1.0 },
  1.0,                                 /* Expression: port
                                        * Referenced by: '<S35>/RS232 Binary Send1'
                                        */

  /*  Computed Parameter: RS232BinarySend1_P2_Size
   * Referenced by: '<S35>/RS232 Binary Send1'
   */
  { 1.0, 1.0 },
  22.0,                                /* Expression: width
                                        * Referenced by: '<S35>/RS232 Binary Send1'
                                        */

  /*  Computed Parameter: RS232BinarySend1_P3_Size
   * Referenced by: '<S35>/RS232 Binary Send1'
   */
  { 1.0, 1.0 },
  -1.0,                                /* Expression: samptime
                                        * Referenced by: '<S35>/RS232 Binary Send1'
                                        */
  8.0,                                 /* Expression: 8
                                        * Referenced by: '<Root>/packet size'
                                        */
  1.0,                                 /* Expression: 1
                                        * Referenced by: '<Root>/packet size4'
                                        */

  /*  Computed Parameter: RS232BinaryReceive1_P1_Size
   * Referenced by: '<Root>/RS232 Binary Receive1'
   */
  { 1.0, 1.0 },
  1.0,                                 /* Expression: port
                                        * Referenced by: '<Root>/RS232 Binary Receive1'
                                        */

  /*  Computed Parameter: RS232BinaryReceive1_P2_Size
   * Referenced by: '<Root>/RS232 Binary Receive1'
   */
  { 1.0, 1.0 },
  8.0,                                 /* Expression: width
                                        * Referenced by: '<Root>/RS232 Binary Receive1'
                                        */

  /*  Computed Parameter: RS232BinaryReceive1_P3_Size
   * Referenced by: '<Root>/RS232 Binary Receive1'
   */
  { 1.0, 1.0 },
  -1.0,                                /* Expression: samptime
                                        * Referenced by: '<Root>/RS232 Binary Receive1'
                                        */

  /*  Computed Parameter: LeftCamera_P1_Size
   * Referenced by: '<S10>/Left Camera'
   */
  { 1.0, 1.0 },
  1.0,                                 /* Expression: isig
                                        * Referenced by: '<S10>/Left Camera'
                                        */

  /*  Computed Parameter: LeftCamera_P2_Size
   * Referenced by: '<S10>/Left Camera'
   */
  { 1.0, 1.0 },
  640.0,                               /* Expression: iwidth
                                        * Referenced by: '<S10>/Left Camera'
                                        */

  /*  Computed Parameter: LeftCamera_P3_Size
   * Referenced by: '<S10>/Left Camera'
   */
  { 1.0, 1.0 },
  480.0,                               /* Expression: iheight
                                        * Referenced by: '<S10>/Left Camera'
                                        */

  /*  Computed Parameter: LeftCamera_P4_Size
   * Referenced by: '<S10>/Left Camera'
   */
  { 1.0, 1.0 },
  2.0,                                 /* Expression: fint
                                        * Referenced by: '<S10>/Left Camera'
                                        */

  /*  Computed Parameter: LeftCamera_P5_Size
   * Referenced by: '<S10>/Left Camera'
   */
  { 1.0, 1.0 },
  1.0,                                 /* Expression: ffmt
                                        * Referenced by: '<S10>/Left Camera'
                                        */

  /*  Computed Parameter: LeftCamera_P6_Size
   * Referenced by: '<S10>/Left Camera'
   */
  { 1.0, 1.0 },
  1.0,                                 /* Expression: cfmt
                                        * Referenced by: '<S10>/Left Camera'
                                        */

  /*  Computed Parameter: LeftCamera_P7_Size
   * Referenced by: '<S10>/Left Camera'
   */
  { 1.0, 1.0 },
  0.0,                                 /* Expression: trigger
                                        * Referenced by: '<S10>/Left Camera'
                                        */

  /*  Computed Parameter: LeftCamera_P8_Size
   * Referenced by: '<S10>/Left Camera'
   */
  { 1.0, 1.0 },
  0.0,                                 /* Expression: status
                                        * Referenced by: '<S10>/Left Camera'
                                        */

  /*  Computed Parameter: LeftCamera_P9_Size
   * Referenced by: '<S10>/Left Camera'
   */
  { 1.0, 3.0 },

  /*  Computed Parameter: LeftCamera_P9
   * Referenced by: '<S10>/Left Camera'
   */
  { 49.0, 46.0, 48.0 },

  /*  Computed Parameter: RightCamera_P1_Size
   * Referenced by: '<S10>/Right Camera'
   */
  { 1.0, 1.0 },
  1.0,                                 /* Expression: isig
                                        * Referenced by: '<S10>/Right Camera'
                                        */

  /*  Computed Parameter: RightCamera_P2_Size
   * Referenced by: '<S10>/Right Camera'
   */
  { 1.0, 1.0 },
  640.0,                               /* Expression: iwidth
                                        * Referenced by: '<S10>/Right Camera'
                                        */

  /*  Computed Parameter: RightCamera_P3_Size
   * Referenced by: '<S10>/Right Camera'
   */
  { 1.0, 1.0 },
  480.0,                               /* Expression: iheight
                                        * Referenced by: '<S10>/Right Camera'
                                        */

  /*  Computed Parameter: RightCamera_P4_Size
   * Referenced by: '<S10>/Right Camera'
   */
  { 1.0, 1.0 },
  2.0,                                 /* Expression: fint
                                        * Referenced by: '<S10>/Right Camera'
                                        */

  /*  Computed Parameter: RightCamera_P5_Size
   * Referenced by: '<S10>/Right Camera'
   */
  { 1.0, 1.0 },
  1.0,                                 /* Expression: ffmt
                                        * Referenced by: '<S10>/Right Camera'
                                        */

  /*  Computed Parameter: RightCamera_P6_Size
   * Referenced by: '<S10>/Right Camera'
   */
  { 1.0, 1.0 },
  1.0,                                 /* Expression: cfmt
                                        * Referenced by: '<S10>/Right Camera'
                                        */

  /*  Computed Parameter: RightCamera_P7_Size
   * Referenced by: '<S10>/Right Camera'
   */
  { 1.0, 1.0 },
  0.0,                                 /* Expression: trigger
                                        * Referenced by: '<S10>/Right Camera'
                                        */

  /*  Computed Parameter: RightCamera_P8_Size
   * Referenced by: '<S10>/Right Camera'
   */
  { 1.0, 1.0 },
  0.0,                                 /* Expression: status
                                        * Referenced by: '<S10>/Right Camera'
                                        */

  /*  Computed Parameter: RightCamera_P9_Size
   * Referenced by: '<S10>/Right Camera'
   */
  { 1.0, 3.0 },

  /*  Computed Parameter: RightCamera_P9
   * Referenced by: '<S10>/Right Camera'
   */
  { 49.0, 46.0, 51.0 },
  0.0,                                 /* Expression: 0
                                        * Referenced by: '<S3>/Switch'
                                        */
  36.0,                                /* Expression: 36
                                        * Referenced by: '<S10>/T'
                                        */

  /*  Expression: [0 0 22.5 9; 12.75 -12.75 0 0; 2 2 2 22.5; 1 1 1 1]
   * Referenced by: '<S10>/Object Model'
   */
  { 0.0, 12.75, 2.0, 1.0, 0.0, -12.75, 2.0, 1.0, 22.5, 0.0, 2.0, 1.0, 9.0, 0.0,
    22.5, 1.0 },
  0.25,                                /* Expression: 0.25
                                        * Referenced by: '<S10>/kx'
                                        */
  1.5,                                 /* Expression: 1.5
                                        * Referenced by: '<S10>/k'
                                        */
  1.5,                                 /* Expression: 1.5
                                        * Referenced by: '<S10>/ks'
                                        */
  20.0,                                /* Expression: 20
                                        * Referenced by: '<S1>/PWM_L'
                                        */
  15.0,                                /* Expression: 15
                                        * Referenced by: '<S1>/PWM_R'
                                        */
  0.0,                                 /* Expression: 0
                                        * Referenced by: '<S3>/Data Store Memory'
                                        */

  /*  Computed Parameter: RS232Setup_P1_Size
   * Referenced by: '<Root>/RS232 Setup '
   */
  { 1.0, 1.0 },
  1.0,                                 /* Expression: port
                                        * Referenced by: '<Root>/RS232 Setup '
                                        */

  /*  Computed Parameter: RS232Setup_P2_Size
   * Referenced by: '<Root>/RS232 Setup '
   */
  { 1.0, 1.0 },
  1.0,                                 /* Expression: baud
                                        * Referenced by: '<Root>/RS232 Setup '
                                        */

  /*  Computed Parameter: RS232Setup_P3_Size
   * Referenced by: '<Root>/RS232 Setup '
   */
  { 1.0, 1.0 },
  1.0,                                 /* Expression: dbits
                                        * Referenced by: '<Root>/RS232 Setup '
                                        */

  /*  Computed Parameter: RS232Setup_P4_Size
   * Referenced by: '<Root>/RS232 Setup '
   */
  { 1.0, 1.0 },
  1.0,                                 /* Expression: sbits
                                        * Referenced by: '<Root>/RS232 Setup '
                                        */

  /*  Computed Parameter: RS232Setup_P5_Size
   * Referenced by: '<Root>/RS232 Setup '
   */
  { 1.0, 1.0 },
  1.0,                                 /* Expression: parity
                                        * Referenced by: '<Root>/RS232 Setup '
                                        */

  /*  Computed Parameter: RS232Setup_P6_Size
   * Referenced by: '<Root>/RS232 Setup '
   */
  { 1.0, 1.0 },
  1.0,                                 /* Expression: prot
                                        * Referenced by: '<Root>/RS232 Setup '
                                        */

  /*  Computed Parameter: RS232Setup_P7_Size
   * Referenced by: '<Root>/RS232 Setup '
   */
  { 1.0, 1.0 },
  2048.0,                              /* Expression: sbuf
                                        * Referenced by: '<Root>/RS232 Setup '
                                        */

  /*  Computed Parameter: RS232Setup_P8_Size
   * Referenced by: '<Root>/RS232 Setup '
   */
  { 1.0, 1.0 },
  2048.0,                              /* Expression: rbuf
                                        * Referenced by: '<Root>/RS232 Setup '
                                        */

  /*  Computed Parameter: RS232Setup_P9_Size
   * Referenced by: '<Root>/RS232 Setup '
   */
  { 1.0, 1.0 },
  0.0,                                 /* Expression: 0
                                        * Referenced by: '<Root>/RS232 Setup '
                                        */

  /*  Computed Parameter: RS232Setup_P10_Size
   * Referenced by: '<Root>/RS232 Setup '
   */
  { 0.0, 0.0 },

  /*  Computed Parameter: RS232Setup_P11_Size
   * Referenced by: '<Root>/RS232 Setup '
   */
  { 0.0, 0.0 },

  /*  Computed Parameter: RS232Setup_P12_Size
   * Referenced by: '<Root>/RS232 Setup '
   */
  { 1.0, 1.0 },
  0.0,                                 /* Expression: iackto
                                        * Referenced by: '<Root>/RS232 Setup '
                                        */

  /*  Computed Parameter: RS232Setup_P13_Size
   * Referenced by: '<Root>/RS232 Setup '
   */
  { 0.0, 0.0 },

  /*  Computed Parameter: RS232Setup_P14_Size
   * Referenced by: '<Root>/RS232 Setup '
   */
  { 0.0, 0.0 },

  /*  Computed Parameter: RS232Setup_P15_Size
   * Referenced by: '<Root>/RS232 Setup '
   */
  { 1.0, 1.0 },
  0.0,                                 /* Expression: tackto
                                        * Referenced by: '<Root>/RS232 Setup '
                                        */

  /*  Computed Parameter: RS232Setup_P16_Size
   * Referenced by: '<Root>/RS232 Setup '
   */
  { 1.0, 1.0 },
  1.0,                                 /* Expression: initinfo
                                        * Referenced by: '<Root>/RS232 Setup '
                                        */

  /*  Computed Parameter: RS232Setup_P17_Size
   * Referenced by: '<Root>/RS232 Setup '
   */
  { 1.0, 1.0 },
  1.0,                                 /* Expression: initackinfo
                                        * Referenced by: '<Root>/RS232 Setup '
                                        */

  /*  Computed Parameter: RS232Setup_P18_Size
   * Referenced by: '<Root>/RS232 Setup '
   */
  { 1.0, 1.0 },
  1.0,                                 /* Expression: terminfo
                                        * Referenced by: '<Root>/RS232 Setup '
                                        */

  /*  Computed Parameter: RS232Setup_P19_Size
   * Referenced by: '<Root>/RS232 Setup '
   */
  { 1.0, 1.0 },
  1.0,                                 /* Expression: termackinfo
                                        * Referenced by: '<Root>/RS232 Setup '
                                        */
  1500U,                               /* Computed Parameter: Servo1_Value
                                        * Referenced by: '<S11>/Servo1'
                                        */
  129U,                                /* Computed Parameter: SYNC0_Value
                                        * Referenced by: '<S35>/SYNC 0'
                                        */
  161U,                                /* Computed Parameter: SYNC1_Value
                                        * Referenced by: '<S35>/SYNC 1'
                                        */
  20U,                                 /* Computed Parameter: ID_Value
                                        * Referenced by: '<S35>/ID'
                                        */
  16U                                  /* Computed Parameter: Count_Value
                                        * Referenced by: '<S35>/Count'
                                        */
};
