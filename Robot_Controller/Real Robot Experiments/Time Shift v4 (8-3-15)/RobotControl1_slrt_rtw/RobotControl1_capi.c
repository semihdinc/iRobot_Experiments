/*
 * RobotControl1_capi.c
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
#include "rtw_capi.h"
#include "RobotControl1_private.h"

/* Block output signal information */
static const rtwCAPI_Signals rtBlockSignals[] = {
  /* addrMapIndex, sysNum, blockPath,
   * signalName, portNumber, dataTypeIndex, dimIndex, fxpIndex, sTimeIndex
   */
  { 0, 14, "status checker",
    "enable", 0, 0, 0, 0, 0 },

  { 1, 0, "Clock",
    "", 0, 0, 0, 0, 1 },

  { 2, 0, "packet size",
    "", 0, 0, 0, 0, 0 },

  { 3, 0, "packet size4",
    "", 0, 0, 0, 0, 0 },

  { 4, 0, "RS232 Binary Receive1/p1",
    "", 0, 1, 1, 0, 0 },

  { 5, 0, "Add",
    "", 0, 0, 0, 0, 1 },

  { 6, 0, "Unit Delay",
    "", 0, 0, 0, 0, 0 },

  { 7, 0, "Unit Delay1",
    "", 0, 0, 0, 0, 0 },

  { 8, 1, "Open-loop control/ MATLAB Function1/p1",
    "PWM_L", 0, 0, 0, 0, 1 },

  { 9, 1, "Open-loop control/ MATLAB Function1/p2",
    "PWM_R", 1, 0, 0, 0, 1 },

  { 10, 2, "Open-loop control/Camera Servos Function/p1",
    "PAN", 0, 0, 0, 0, 1 },

  { 11, 2, "Open-loop control/Camera Servos Function/p2",
    "TILT", 1, 0, 0, 0, 1 },

  { 12, 3, "Open-loop control/MATLAB Function 2/p1",
    "v", 0, 0, 0, 0, 1 },

  { 13, 3, "Open-loop control/MATLAB Function 2/p2",
    "omega", 1, 0, 0, 0, 1 },

  { 14, 10, "Read/Data Store Read",
    "", 0, 0, 0, 0, 2 },

  { 15, 11, "Status Extraction/Command State",
    "CCS", 0, 0, 0, 0, 2 },

  { 16, 12, "Status Extraction/Unpack/p1",
    "", 0, 1, 0, 0, 2 },

  { 17, 12, "Status Extraction/Unpack/p2",
    "", 1, 1, 0, 0, 2 },

  { 18, 12, "Status Extraction/Unpack/p3",
    "", 2, 1, 0, 0, 2 },

  { 19, 12, "Status Extraction/Unpack/p4",
    "", 3, 1, 0, 0, 2 },

  { 20, 12, "Status Extraction/Unpack/p5",
    "", 4, 1, 0, 0, 2 },

  { 21, 12, "Status Extraction/Unpack/p6",
    "", 5, 1, 0, 0, 2 },

  { 22, 12, "Status Extraction/Unpack/p7",
    "", 6, 1, 0, 0, 2 },

  { 23, 12, "Status Extraction/Unpack/p8",
    "", 7, 1, 0, 0, 2 },

  { 24, 7, "Open-loop control/plant/pulse transformer camera/p1",
    "PAN_O", 0, 2, 0, 0, 1 },

  { 25, 7, "Open-loop control/plant/pulse transformer camera/p2",
    "TILT_O", 1, 2, 0, 0, 1 },

  { 26, 8, "Open-loop control/plant/pulse transformer1/p1",
    "Left_PWM", 0, 2, 0, 0, 1 },

  { 27, 8, "Open-loop control/plant/pulse transformer1/p2",
    "Right_PWM", 1, 2, 0, 0, 1 },

  { 28, 4,
    "Open-loop control/plant/Send Serial Packet to Servo Controller And recieve status message from SSC/CheckSum/p1",
    "csum0", 0, 1, 0, 0, 1 },

  { 29, 4,
    "Open-loop control/plant/Send Serial Packet to Servo Controller And recieve status message from SSC/CheckSum/p2",
    "csum1", 1, 1, 0, 0, 1 },

  { 30, 5,
    "Open-loop control/plant/Send Serial Packet to Servo Controller And recieve status message from SSC/Embedded MATLAB Function",
    "CharArray", 0, 1, 2, 0, 0 },

  { 31, 6,
    "Open-loop control/plant/Send Serial Packet to Servo Controller And recieve status message from SSC/Byte Reversal/p1",
    "", 0, 2, 0, 0, 0 },

  { 32, 6,
    "Open-loop control/plant/Send Serial Packet to Servo Controller And recieve status message from SSC/Byte Reversal/p2",
    "", 1, 2, 0, 0, 0 },

  { 33, 6,
    "Open-loop control/plant/Send Serial Packet to Servo Controller And recieve status message from SSC/Byte Reversal/p3",
    "", 2, 2, 0, 0, 0 },

  { 34, 6,
    "Open-loop control/plant/Send Serial Packet to Servo Controller And recieve status message from SSC/Byte Reversal/p4",
    "", 3, 2, 0, 0, 0 },

  { 35, 6,
    "Open-loop control/plant/Send Serial Packet to Servo Controller And recieve status message from SSC/Byte Reversal/p5",
    "", 4, 2, 0, 0, 0 },

  { 36, 6,
    "Open-loop control/plant/Send Serial Packet to Servo Controller And recieve status message from SSC/Byte Reversal/p6",
    "", 5, 2, 0, 0, 0 },

  { 37, 6,
    "Open-loop control/plant/Send Serial Packet to Servo Controller And recieve status message from SSC/Byte Reversal/p7",
    "", 6, 2, 0, 0, 0 },

  { 38, 6,
    "Open-loop control/plant/Send Serial Packet to Servo Controller And recieve status message from SSC/Byte Reversal/p8",
    "", 7, 2, 0, 0, 0 },

  { 39, 6,
    "Open-loop control/plant/Send Serial Packet to Servo Controller And recieve status message from SSC/Pack",
    "", 0, 1, 2, 0, 0 },

  {
    0, 0, NULL, NULL, 0, 0, 0, 0, 0
  }
};

static const rtwCAPI_BlockParameters rtBlockParameters[] = {
  /* addrMapIndex, blockPath,
   * paramName, dataTypeIndex, dimIndex, fixPtIdx
   */
  { 40, "Data Store Memory",
    "InitialValue", 0, 0, 0 },

  { 41, "packet size",
    "Value", 0, 0, 0 },

  { 42, "packet size4",
    "Value", 0, 0, 0 },

  { 43, "RS232 Binary Receive1",
    "P1", 0, 0, 0 },

  { 44, "RS232 Binary Receive1",
    "P2", 0, 0, 0 },

  { 45, "RS232 Binary Receive1",
    "P3", 0, 0, 0 },

  { 46, "RS232 Setup ",
    "P1", 0, 0, 0 },

  { 47, "RS232 Setup ",
    "P2", 0, 0, 0 },

  { 48, "RS232 Setup ",
    "P3", 0, 0, 0 },

  { 49, "RS232 Setup ",
    "P4", 0, 0, 0 },

  { 50, "RS232 Setup ",
    "P5", 0, 0, 0 },

  { 51, "RS232 Setup ",
    "P6", 0, 0, 0 },

  { 52, "RS232 Setup ",
    "P7", 0, 0, 0 },

  { 53, "RS232 Setup ",
    "P8", 0, 0, 0 },

  { 54, "RS232 Setup ",
    "P9", 0, 0, 0 },

  { 55, "RS232 Setup ",
    "P12", 0, 0, 0 },

  { 56, "RS232 Setup ",
    "P15", 0, 0, 0 },

  { 57, "RS232 Setup ",
    "P16", 0, 0, 0 },

  { 58, "RS232 Setup ",
    "P17", 0, 0, 0 },

  { 59, "RS232 Setup ",
    "P18", 0, 0, 0 },

  { 60, "RS232 Setup ",
    "P19", 0, 0, 0 },

  { 61, "Unit Delay",
    "InitialCondition", 0, 0, 0 },

  { 62, "Unit Delay1",
    "InitialCondition", 0, 0, 0 },

  { 63, "Open-loop control/plant/Servo1",
    "Value", 2, 0, 0 },

  { 64,
    "Open-loop control/plant/Send Serial Packet to Servo Controller And recieve status message from SSC/Count",
    "Value", 1, 0, 0 },

  { 65,
    "Open-loop control/plant/Send Serial Packet to Servo Controller And recieve status message from SSC/ID",
    "Value", 1, 0, 0 },

  { 66,
    "Open-loop control/plant/Send Serial Packet to Servo Controller And recieve status message from SSC/SYNC 0",
    "Value", 1, 0, 0 },

  { 67,
    "Open-loop control/plant/Send Serial Packet to Servo Controller And recieve status message from SSC/SYNC 1",
    "Value", 1, 0, 0 },

  { 68,
    "Open-loop control/plant/Send Serial Packet to Servo Controller And recieve status message from SSC/RS232 Binary Send1",
    "P1", 0, 0, 0 },

  { 69,
    "Open-loop control/plant/Send Serial Packet to Servo Controller And recieve status message from SSC/RS232 Binary Send1",
    "P2", 0, 0, 0 },

  { 70,
    "Open-loop control/plant/Send Serial Packet to Servo Controller And recieve status message from SSC/RS232 Binary Send1",
    "P3", 0, 0, 0 },

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
  &RobotControl1_B.enable,             /* 0: Signal */
  &RobotControl1_B.Clock,              /* 1: Signal */
  &RobotControl1_B.packetsize,         /* 2: Signal */
  &RobotControl1_B.packetsize4,        /* 3: Signal */
  &RobotControl1_B.RS232BinaryReceive1_o2[0],/* 4: Signal */
  &RobotControl1_B.Add,                /* 5: Signal */
  &RobotControl1_B.UnitDelay,          /* 6: Signal */
  &RobotControl1_B.UnitDelay1,         /* 7: Signal */
  &RobotControl1_B.PWM_L,              /* 8: Signal */
  &RobotControl1_B.PWM_R,              /* 9: Signal */
  &RobotControl1_B.PAN,                /* 10: Signal */
  &RobotControl1_B.TILT,               /* 11: Signal */
  &RobotControl1_B.v,                  /* 12: Signal */
  &RobotControl1_B.omega,              /* 13: Signal */
  &RobotControl1_B.DataStoreRead,      /* 14: Signal */
  &RobotControl1_B.CCS,                /* 15: Signal */
  &RobotControl1_B.Unpack_o1,          /* 16: Signal */
  &RobotControl1_B.Unpack_o2,          /* 17: Signal */
  &RobotControl1_B.Unpack_o3,          /* 18: Signal */
  &RobotControl1_B.Unpack_o4,          /* 19: Signal */
  &RobotControl1_B.Unpack_o5,          /* 20: Signal */
  &RobotControl1_B.Unpack_o6,          /* 21: Signal */
  &RobotControl1_B.Unpack_o7,          /* 22: Signal */
  &RobotControl1_B.Unpack_o8,          /* 23: Signal */
  &RobotControl1_B.PAN_O,              /* 24: Signal */
  &RobotControl1_B.TILT_O,             /* 25: Signal */
  &RobotControl1_B.Left_PWM,           /* 26: Signal */
  &RobotControl1_B.Right_PWM,          /* 27: Signal */
  &RobotControl1_B.csum0,              /* 28: Signal */
  &RobotControl1_B.csum1,              /* 29: Signal */
  &RobotControl1_B.CharArray[0],       /* 30: Signal */
  &RobotControl1_B.ByteReversal_o1,    /* 31: Signal */
  &RobotControl1_B.ByteReversal_o2,    /* 32: Signal */
  &RobotControl1_B.ByteReversal_o3,    /* 33: Signal */
  &RobotControl1_B.ByteReversal_o4,    /* 34: Signal */
  &RobotControl1_B.ByteReversal_o5,    /* 35: Signal */
  &RobotControl1_B.ByteReversal_o6,    /* 36: Signal */
  &RobotControl1_B.ByteReversal_o7,    /* 37: Signal */
  &RobotControl1_B.ByteReversal_o8,    /* 38: Signal */
  &RobotControl1_B.Pack[0],            /* 39: Signal */
  &RobotControl1_P.DataStoreMemory_InitialValue,/* 40: Block Parameter */
  &RobotControl1_P.packetsize_Value,   /* 41: Block Parameter */
  &RobotControl1_P.packetsize4_Value,  /* 42: Block Parameter */
  &RobotControl1_P.RS232BinaryReceive1_P1,/* 43: Block Parameter */
  &RobotControl1_P.RS232BinaryReceive1_P2,/* 44: Block Parameter */
  &RobotControl1_P.RS232BinaryReceive1_P3,/* 45: Block Parameter */
  &RobotControl1_P.RS232Setup_P1,      /* 46: Block Parameter */
  &RobotControl1_P.RS232Setup_P2,      /* 47: Block Parameter */
  &RobotControl1_P.RS232Setup_P3,      /* 48: Block Parameter */
  &RobotControl1_P.RS232Setup_P4,      /* 49: Block Parameter */
  &RobotControl1_P.RS232Setup_P5,      /* 50: Block Parameter */
  &RobotControl1_P.RS232Setup_P6,      /* 51: Block Parameter */
  &RobotControl1_P.RS232Setup_P7,      /* 52: Block Parameter */
  &RobotControl1_P.RS232Setup_P8,      /* 53: Block Parameter */
  &RobotControl1_P.RS232Setup_P9,      /* 54: Block Parameter */
  &RobotControl1_P.RS232Setup_P12,     /* 55: Block Parameter */
  &RobotControl1_P.RS232Setup_P15,     /* 56: Block Parameter */
  &RobotControl1_P.RS232Setup_P16,     /* 57: Block Parameter */
  &RobotControl1_P.RS232Setup_P17,     /* 58: Block Parameter */
  &RobotControl1_P.RS232Setup_P18,     /* 59: Block Parameter */
  &RobotControl1_P.RS232Setup_P19,     /* 60: Block Parameter */
  &RobotControl1_P.UnitDelay_InitialCondition,/* 61: Block Parameter */
  &RobotControl1_P.UnitDelay1_InitialCondition,/* 62: Block Parameter */
  &RobotControl1_P.Servo1_Value,       /* 63: Block Parameter */
  &RobotControl1_P.Count_Value,        /* 64: Block Parameter */
  &RobotControl1_P.ID_Value,           /* 65: Block Parameter */
  &RobotControl1_P.SYNC0_Value,        /* 66: Block Parameter */
  &RobotControl1_P.SYNC1_Value,        /* 67: Block Parameter */
  &RobotControl1_P.RS232BinarySend1_P1,/* 68: Block Parameter */
  &RobotControl1_P.RS232BinarySend1_P2,/* 69: Block Parameter */
  &RobotControl1_P.RS232BinarySend1_P3 /* 70: Block Parameter */
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

  { "unsigned char", "uint8_T", 0, 0, sizeof(uint8_T), SS_UINT8, 0, 0 },

  { "unsigned short", "uint16_T", 0, 0, sizeof(uint16_T), SS_UINT16, 0, 0 }
};

/* Structure Element Map - use elemMapIndex to access this structure */
static const rtwCAPI_ElementMap rtElementMap[] = {
  /* elementName, elementOffset, dataTypeIndex, dimIndex, fxpIndex */
  { NULL, 0, 0, 0, 0 },
};

/* Dimension Map - use dimensionMapIndex to access elements of ths structure*/
static const rtwCAPI_DimensionMap rtDimensionMap[] = {
  /* dataOrientation, dimArrayIndex, numDims, vardimsIndex */
  { rtwCAPI_SCALAR, 0, 2, 0 },

  { rtwCAPI_VECTOR, 2, 2, 0 },

  { rtwCAPI_VECTOR, 4, 2, 0 }
};

/* Dimension Array- use dimArrayIndex to access elements of this array */
static const uint_T rtDimensionArray[] = {
  1,                                   /* 0 */
  1,                                   /* 1 */
  8,                                   /* 2 */
  1,                                   /* 3 */
  22,                                  /* 4 */
  1                                    /* 5 */
};

/* C-API stores floating point values in an array. The elements of this  *
 * are unique. This ensures that values which are shared across the model*
 * are stored in the most efficient way. These values are referenced by  *
 *           - rtwCAPI_FixPtMap.fracSlopePtr,                            *
 *           - rtwCAPI_FixPtMap.biasPtr,                                 *
 *           - rtwCAPI_SampleTimeMap.samplePeriodPtr,                    *
 *           - rtwCAPI_SampleTimeMap.sampleOffsetPtr                     */
static const real_T rtcapiStoredFloats[] = {
  0.13333333333333333, 0.0
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
    1, 0 },

  { (const void *) &rtcapiStoredFloats[1], (const void *) &rtcapiStoredFloats[1],
    0, 0 },

  { (NULL), (NULL), -1, 0 }
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
  { rtBlockSignals, 40 },

  { rtBlockParameters, 31,
    rtModelParameters, 0 },

  { NULL, 0 },

  { rtDataTypeMap, rtDimensionMap, rtFixPtMap,
    rtElementMap, rtSampleTimeMap, rtDimensionArray },
  "float",

  { 1190974694U,
    401083059U,
    2116853257U,
    2889573313U },
  NULL
};

/* Cache pointers into DataMapInfo substructure of RTModel */
void RobotControl1_InitializeDataMapInfo(RT_MODEL_RobotControl1_T
  *RobotControl1_M
  )
{
  /* Set C-API version */
  rtwCAPI_SetVersion(RobotControl1_M->DataMapInfo.mmi, 1);

  /* Cache static C-API data into the Real-time Model Data structure */
  rtwCAPI_SetStaticMap(RobotControl1_M->DataMapInfo.mmi, &mmiStatic);

  /* Cache static C-API logging data into the Real-time Model Data structure */
  rtwCAPI_SetLoggingStaticMap(RobotControl1_M->DataMapInfo.mmi, NULL);

  /* Cache C-API Data Addresses into the Real-Time Model Data structure */
  rtwCAPI_SetDataAddressMap(RobotControl1_M->DataMapInfo.mmi, rtDataAddrMap);

  /* Cache C-API Data Run-Time Dimension Buffer Addresses into the Real-Time Model Data structure */
  rtwCAPI_SetVarDimsAddressMap(RobotControl1_M->DataMapInfo.mmi,
    rtVarDimsAddrMap);

  /* Cache C-API rtp Address and size  into the Real-Time Model Data structure */
  RobotControl1_M->DataMapInfo.mmi.InstanceMap.rtpAddress = rtmGetDefaultParam
    (RobotControl1_M);
  RobotControl1_M->DataMapInfo.mmi.staticMap->rtpSize = sizeof(P_RobotControl1_T);

  /* Cache the instance C-API logging pointer */
  rtwCAPI_SetInstanceLoggingInfo(RobotControl1_M->DataMapInfo.mmi, NULL);

  /* Set Reference to submodels */
  rtwCAPI_SetChildMMIArray(RobotControl1_M->DataMapInfo.mmi, NULL);
  rtwCAPI_SetChildMMIArrayLen(RobotControl1_M->DataMapInfo.mmi, 0);
}

/* EOF: RobotControl1_capi.c */
