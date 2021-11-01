/*
 * RobotControl1_private.h
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

#ifndef RTW_HEADER_RobotControl1_private_h_
#define RTW_HEADER_RobotControl1_private_h_
#include "rtwtypes.h"
#include "builtin_typeid_types.h"
#include "multiword_types.h"
#include "zero_crossing_types.h"

/* Used to reverse endianness */
#define SWAP16(x)                      (((x) >> 8) | (((x) & 0xff) << 8))
#define SWAP32(x)                      (SWAP16((x) >> 16) | (SWAP16((x) & 0xffff) << 16))

extern const serialfifoptr serialfifoground;
extern const bcmsglist1553 bcmsg1553ground;
extern const bcstatus1553 bcstatground;
extern const bmmsglist1553 bmmsg1553ground;
extern real_T rt_roundd_snf(real_T u);
extern const char *getRefMdlPath(const char *refMdl);
extern int getRefMdlSignalNumber(const char *mdlBlock, const char *signalName);
extern void rs232bsend(SimStruct *rts);
extern void rs232brec(SimStruct *rts);
extern void rs232setup(SimStruct *rts);
extern void RobotControl1_StatusExtraction(void);
extern boolean_T RobotControl1_StatusExtractionFNI(RT_MODEL_RobotControl1_T *
  const RobotControl1_M, int_T controlPortIdx, int_T tid);
extern boolean_T RobotControl1_StatusExtraction_InitFNI(RT_MODEL_RobotControl1_T
  *const RobotControl1_M, int_T controlPortIdx, int_T tid);

#endif                                 /* RTW_HEADER_RobotControl1_private_h_ */
