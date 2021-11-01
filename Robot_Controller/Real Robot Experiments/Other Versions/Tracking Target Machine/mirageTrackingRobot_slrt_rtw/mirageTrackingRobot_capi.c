/*
 * mirageTrackingRobot_capi.c
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
#include "rtw_capi.h"
#include "mirageTrackingRobot_private.h"

/* Block output signal information */
static const rtwCAPI_Signals rtBlockSignals[] = {
  /* addrMapIndex, sysNum, blockPath,
   * signalName, portNumber, dataTypeIndex, dimIndex, fxpIndex, sTimeIndex
   */
  { 0, 1, "Calculate Target Points/p1",
    "L_p", 0, 0, 0, 0, 0 },

  { 1, 1, "Calculate Target Points/p2",
    "R_p", 1, 0, 0, 0, 0 },

  { 2, 2, "Estimate Pose Error",
    "qtilde", 0, 0, 1, 0, 0 },

  { 3, 0, "From Left Camera",
    "", 0, 1, 2, 0, 0 },

  { 4, 0, "From Right Camera",
    "", 0, 1, 2, 0, 0 },

  {
    0, 0, NULL, NULL, 0, 0, 0, 0, 0
  }
};

static const rtwCAPI_BlockParameters rtBlockParameters[] = {
  /* addrMapIndex, blockPath,
   * paramName, dataTypeIndex, dimIndex, fixPtIdx
   */
  { 5, "Desired Robot Pose",
    "Value", 0, 3, 0 },

  { 6, "Object Model",
    "Value", 0, 4, 0 },

  { 7, "From Left Camera",
    "P1", 0, 5, 0 },

  { 8, "From Left Camera",
    "P2", 0, 5, 0 },

  { 9, "From Left Camera",
    "P3", 0, 5, 0 },

  { 10, "From Left Camera",
    "P4", 0, 5, 0 },

  { 11, "From Left Camera",
    "P5", 0, 5, 0 },

  { 12, "From Left Camera",
    "P6", 0, 5, 0 },

  { 13, "From Left Camera",
    "P7", 0, 5, 0 },

  { 14, "From Left Camera",
    "P8", 0, 5, 0 },

  { 15, "From Left Camera",
    "P9", 0, 6, 0 },

  { 16, "From Right Camera",
    "P1", 0, 5, 0 },

  { 17, "From Right Camera",
    "P2", 0, 5, 0 },

  { 18, "From Right Camera",
    "P3", 0, 5, 0 },

  { 19, "From Right Camera",
    "P4", 0, 5, 0 },

  { 20, "From Right Camera",
    "P5", 0, 5, 0 },

  { 21, "From Right Camera",
    "P6", 0, 5, 0 },

  { 22, "From Right Camera",
    "P7", 0, 5, 0 },

  { 23, "From Right Camera",
    "P8", 0, 5, 0 },

  { 24, "From Right Camera",
    "P9", 0, 6, 0 },

  {
    0, NULL, NULL, 0, 0, 0
  }
};

/* Tunable variable parameters */
static const rtwCAPI_ModelParameters rtModelParameters[] = {
  /* addrMapIndex, varName, dataTypeIndex, dimIndex, fixPtIndex */
  { 0, NULL, 0, 0, 0 }
};

/* Declare Data Addresses statically */
static void* rtDataAddrMap[] = {
  &mirageTrackingRobot_B.L_p[0],       /* 0: Signal */
  &mirageTrackingRobot_B.R_p[0],       /* 1: Signal */
  &mirageTrackingRobot_B.qtilde[0],    /* 2: Signal */
  &mirageTrackingRobot_B.FromLeftCamera[0],/* 3: Signal */
  &mirageTrackingRobot_B.FromRightCamera[0],/* 4: Signal */
  &mirageTrackingRobot_P.DesiredRobotPose_Value[0],/* 5: Block Parameter */
  &mirageTrackingRobot_P.ObjectModel_Value[0],/* 6: Block Parameter */
  &mirageTrackingRobot_P.FromLeftCamera_P1,/* 7: Block Parameter */
  &mirageTrackingRobot_P.FromLeftCamera_P2,/* 8: Block Parameter */
  &mirageTrackingRobot_P.FromLeftCamera_P3,/* 9: Block Parameter */
  &mirageTrackingRobot_P.FromLeftCamera_P4,/* 10: Block Parameter */
  &mirageTrackingRobot_P.FromLeftCamera_P5,/* 11: Block Parameter */
  &mirageTrackingRobot_P.FromLeftCamera_P6,/* 12: Block Parameter */
  &mirageTrackingRobot_P.FromLeftCamera_P7,/* 13: Block Parameter */
  &mirageTrackingRobot_P.FromLeftCamera_P8,/* 14: Block Parameter */
  &mirageTrackingRobot_P.FromLeftCamera_P9[0],/* 15: Block Parameter */
  &mirageTrackingRobot_P.FromRightCamera_P1,/* 16: Block Parameter */
  &mirageTrackingRobot_P.FromRightCamera_P2,/* 17: Block Parameter */
  &mirageTrackingRobot_P.FromRightCamera_P3,/* 18: Block Parameter */
  &mirageTrackingRobot_P.FromRightCamera_P4,/* 19: Block Parameter */
  &mirageTrackingRobot_P.FromRightCamera_P5,/* 20: Block Parameter */
  &mirageTrackingRobot_P.FromRightCamera_P6,/* 21: Block Parameter */
  &mirageTrackingRobot_P.FromRightCamera_P7,/* 22: Block Parameter */
  &mirageTrackingRobot_P.FromRightCamera_P8,/* 23: Block Parameter */
  &mirageTrackingRobot_P.FromRightCamera_P9[0]/* 24: Block Parameter */
};

/* Declare Data Run-Time Dimension Buffer Addresses statically */
static int32_T* rtVarDimsAddrMap[] = {
  (NULL)
};

/* Data Type Map - use dataTypeMapIndex to access this structure */
static const rtwCAPI_DataTypeMap rtDataTypeMap[] = {
  /* cName, mwName, numElements, elemMapIndex, dataSize, slDataId, *
   * isComplex, isPointer */
  { "double", "real_T", 0, 0, sizeof(real_T), SS_DOUBLE, 0, 0 },

  { "unsigned char", "uint8_T", 0, 0, sizeof(uint8_T), SS_UINT8, 0, 0 }
};

/* Structure Element Map - use elemMapIndex to access this structure */
static const rtwCAPI_ElementMap rtElementMap[] = {
  /* elementName, elementOffset, dataTypeIndex, dimIndex, fxpIndex */
  { NULL, 0, 0, 0, 0 },
};

/* Dimension Map - use dimensionMapIndex to access elements of ths structure*/
static const rtwCAPI_DimensionMap rtDimensionMap[] = {
  /* dataOrientation, dimArrayIndex, numDims, vardimsIndex */
  { rtwCAPI_MATRIX_COL_MAJOR, 0, 2, 0 },

  { rtwCAPI_VECTOR, 2, 2, 0 },

  { rtwCAPI_MATRIX_COL_MAJOR_ND, 4, 3, 0 },

  { rtwCAPI_VECTOR, 7, 2, 0 },

  { rtwCAPI_MATRIX_COL_MAJOR, 9, 2, 0 },

  { rtwCAPI_SCALAR, 11, 2, 0 },

  { rtwCAPI_VECTOR, 13, 2, 0 }
};

/* Dimension Array- use dimArrayIndex to access elements of this array */
static const uint_T rtDimensionArray[] = {
  2,                                   /* 0 */
  4,                                   /* 1 */
  6,                                   /* 2 */
  1,                                   /* 3 */
  960,                                 /* 4 */
  1280,                                /* 5 */
  3,                                   /* 6 */
  1,                                   /* 7 */
  6,                                   /* 8 */
  4,                                   /* 9 */
  4,                                   /* 10 */
  1,                                   /* 11 */
  1,                                   /* 12 */
  1,                                   /* 13 */
  2                                    /* 14 */
};

/* C-API stores floating point values in an array. The elements of this  *
 * are unique. This ensures that values which are shared across the model*
 * are stored in the most efficient way. These values are referenced by  *
 *           - rtwCAPI_FixPtMap.fracSlopePtr,                            *
 *           - rtwCAPI_FixPtMap.biasPtr,                                 *
 *           - rtwCAPI_SampleTimeMap.samplePeriodPtr,                    *
 *           - rtwCAPI_SampleTimeMap.sampleOffsetPtr                     */
static const real_T rtcapiStoredFloats[] = {
  0.2, 0.0
};

/* Fixed Point Map */
static const rtwCAPI_FixPtMap rtFixPtMap[] = {
  /* fracSlopePtr, biasPtr, scaleType, wordLength, exponent, isSigned */
  { NULL, NULL, rtwCAPI_FIX_RESERVED, 0, 0, 0 },
};

/* Sample Time Map - use sTimeIndex to access elements of ths structure */
static const rtwCAPI_SampleTimeMap rtSampleTimeMap[] = {
  /* samplePeriodPtr, sampleOffsetPtr, tid, samplingMode */
  { (const void *) &rtcapiStoredFloats[0], (const void *) &rtcapiStoredFloats[1],
    0, 0 }
};

static rtwCAPI_ModelMappingStaticInfo mmiStatic = {
  /* Signals:{signals, numSignals},
   * Params: {blockParameters, numBlockParameters,
   *          modelParameters, numModelParameters},
   * States: {states, numStates},
   * Maps:   {dataTypeMap, dimensionMap, fixPtMap,
   *          elementMap, sampleTimeMap, dimensionArray},
   * TargetType: targetType
   */
  { rtBlockSignals, 5 },

  { rtBlockParameters, 20,
    rtModelParameters, 0 },

  { NULL, 0 },

  { rtDataTypeMap, rtDimensionMap, rtFixPtMap,
    rtElementMap, rtSampleTimeMap, rtDimensionArray },
  "float",

  { 3854121767U,
    1068945715U,
    712960121U,
    2985521636U },
  NULL
};

/* Cache pointers into DataMapInfo substructure of RTModel */
void mirageTrackingRobot_InitializeDataMapInfo(RT_MODEL_mirageTrackingRobot_T
  *mirageTrackingRobot_M
  )
{
  /* Set C-API version */
  rtwCAPI_SetVersion(mirageTrackingRobot_M->DataMapInfo.mmi, 1);

  /* Cache static C-API data into the Real-time Model Data structure */
  rtwCAPI_SetStaticMap(mirageTrackingRobot_M->DataMapInfo.mmi, &mmiStatic);

  /* Cache static C-API logging data into the Real-time Model Data structure */
  rtwCAPI_SetLoggingStaticMap(mirageTrackingRobot_M->DataMapInfo.mmi, NULL);

  /* Cache C-API Data Addresses into the Real-Time Model Data structure */
  rtwCAPI_SetDataAddressMap(mirageTrackingRobot_M->DataMapInfo.mmi,
    rtDataAddrMap);

  /* Cache C-API Data Run-Time Dimension Buffer Addresses into the Real-Time Model Data structure */
  rtwCAPI_SetVarDimsAddressMap(mirageTrackingRobot_M->DataMapInfo.mmi,
    rtVarDimsAddrMap);

  /* Cache C-API rtp Address and size  into the Real-Time Model Data structure */
  mirageTrackingRobot_M->DataMapInfo.mmi.InstanceMap.rtpAddress =
    rtmGetDefaultParam(mirageTrackingRobot_M);
  mirageTrackingRobot_M->DataMapInfo.mmi.staticMap->rtpSize = sizeof
    (P_mirageTrackingRobot_T);

  /* Cache the instance C-API logging pointer */
  rtwCAPI_SetInstanceLoggingInfo(mirageTrackingRobot_M->DataMapInfo.mmi, NULL);

  /* Set Reference to submodels */
  rtwCAPI_SetChildMMIArray(mirageTrackingRobot_M->DataMapInfo.mmi, NULL);
  rtwCAPI_SetChildMMIArrayLen(mirageTrackingRobot_M->DataMapInfo.mmi, 0);
}

/* EOF: mirageTrackingRobot_capi.c */
