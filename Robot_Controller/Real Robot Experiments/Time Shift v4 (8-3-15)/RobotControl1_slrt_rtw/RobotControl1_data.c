/*
 * RobotControl1_data.c
 *
 * Academic License - for use in teaching, academic research, and meeting
 * course requirements at degree granting institutions only.  Not for
 * government, commercial, or other organizational use.
 *
 * Code generation for model "RobotControl1".
 *
 * Model version              : 1.848
 * Simulink Coder version : 8.8 (R2015a) 09-Feb-2015
 * C source code generated on : Wed Sep 23 18:41:12 2015
 *
 * Target selection: slrt.tlc
 * Note: GRT includes extra infrastructure and instrumentation for prototyping
 * Embedded hardware selection: Generic->32-bit x86 compatible
 * Code generation objective: Execution efficiency
 * Validation result: Not run
 */

#include "RobotControl1.h"
#include "RobotControl1_private.h"

/* Block parameters (auto storage) */
P_RobotControl1_T RobotControl1_P = {
  /*  Computed Parameter: RS232BinarySend1_P1_Size
   * Referenced by: '<S30>/RS232 Binary Send1'
   */
  { 1.0, 1.0 },
  1.0,                                 /* Expression: port
                                        * Referenced by: '<S30>/RS232 Binary Send1'
                                        */

  /*  Computed Parameter: RS232BinarySend1_P2_Size
   * Referenced by: '<S30>/RS232 Binary Send1'
   */
  { 1.0, 1.0 },
  22.0,                                /* Expression: width
                                        * Referenced by: '<S30>/RS232 Binary Send1'
                                        */

  /*  Computed Parameter: RS232BinarySend1_P3_Size
   * Referenced by: '<S30>/RS232 Binary Send1'
   */
  { 1.0, 1.0 },
  -1.0,                                /* Expression: samptime
                                        * Referenced by: '<S30>/RS232 Binary Send1'
                                        */
  0.0,                                 /* Expression: 0
                                        * Referenced by: '<Root>/Unit Delay'
                                        */
  0.0,                                 /* Expression: 0
                                        * Referenced by: '<Root>/Unit Delay1'
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
  0.0,                                 /* Expression: 0
                                        * Referenced by: '<Root>/Data Store Memory'
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
                                        * Referenced by: '<S17>/Servo1'
                                        */
  129U,                                /* Computed Parameter: SYNC0_Value
                                        * Referenced by: '<S30>/SYNC 0'
                                        */
  161U,                                /* Computed Parameter: SYNC1_Value
                                        * Referenced by: '<S30>/SYNC 1'
                                        */
  20U,                                 /* Computed Parameter: ID_Value
                                        * Referenced by: '<S30>/ID'
                                        */
  16U                                  /* Computed Parameter: Count_Value
                                        * Referenced by: '<S30>/Count'
                                        */
};
