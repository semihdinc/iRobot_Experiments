/*
 * RobotControl1_640_480_capi.c
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
#include "rtw_capi.h"
#include "RobotControl1_640_480_private.h"

/* Block output signal information */
static const rtwCAPI_Signals rtBlockSignals[] = {
  /* addrMapIndex, sysNum, blockPath,
   * signalName, portNumber, dataTypeIndex, dimIndex, fxpIndex, sTimeIndex
   */
  { 0, 18, "status checker",
    "enable", 0, 0, 0, 0, 0 },

  { 1, 0, "Clock",
    "", 0, 0, 0, 0, 1 },

  { 2, 0, "packet size",
    "", 0, 0, 0, 0, 0 },

  { 3, 0, "packet size4",
    "", 0, 0, 0, 0, 0 },

  { 4, 0, "RS232 Binary Receive1/p1",
    "", 0, 1, 1, 0, 0 },

  { 5, 1, "Open-loop control/Camera Servos Function/p1",
    "PAN", 0, 0, 0, 0, 1 },

  { 6, 1, "Open-loop control/Camera Servos Function/p2",
    "TILT", 1, 0, 0, 0, 1 },

  { 7, 2, "Open-loop control/PWM Calculation/p1",
    "PWM_L", 0, 0, 0, 0, 1 },

  { 8, 2, "Open-loop control/PWM Calculation/p2",
    "PWM_R", 1, 0, 0, 0, 1 },

  { 9, 12, "Status Extraction/Command State",
    "CCS", 0, 0, 0, 0, 2 },

  { 10, 13, "Status Extraction/Unpack/p1",
    "", 0, 1, 0, 0, 2 },

  { 11, 13, "Status Extraction/Unpack/p2",
    "", 1, 1, 0, 0, 2 },

  { 12, 13, "Status Extraction/Unpack/p3",
    "", 2, 1, 0, 0, 2 },

  { 13, 13, "Status Extraction/Unpack/p4",
    "", 3, 1, 0, 0, 2 },

  { 14, 13, "Status Extraction/Unpack/p5",
    "", 4, 1, 0, 0, 2 },

  { 15, 13, "Status Extraction/Unpack/p6",
    "", 5, 1, 0, 0, 2 },

  { 16, 13, "Status Extraction/Unpack/p7",
    "", 6, 1, 0, 0, 2 },

  { 17, 13, "Status Extraction/Unpack/p8",
    "", 7, 1, 0, 0, 2 },

  { 18, 15, "Timer control/Subtract",
    "", 0, 0, 0, 0, 1 },

  { 19, 14, "Timer control/Subtract1",
    "", 0, 0, 0, 0, 1 },

  { 20, 0, "Timer control/Switch",
    "", 0, 0, 0, 0, 1 },

  { 21, 0, "Timer control/Unary Minus",
    "", 0, 0, 0, 0, 0 },

  { 22, 3, "Open-loop control/Vision System/Calculate Target Points/p1",
    "L_p", 0, 0, 2, 0, 0 },

  { 23, 3, "Open-loop control/Vision System/Calculate Target Points/p2",
    "R_p", 1, 0, 2, 0, 0 },

  { 24, 4, "Open-loop control/Vision System/Desired Pose V2/p1",
    "qr", 0, 0, 3, 0, 1 },

  { 25, 4, "Open-loop control/Vision System/Desired Pose V2/p2",
    "ur", 1, 0, 4, 0, 1 },

  { 26, 5, "Open-loop control/Vision System/Non-Holonomic Robot Controller/p1",
    "v", 0, 0, 0, 0, 1 },

  { 27, 5, "Open-loop control/Vision System/Non-Holonomic Robot Controller/p2",
    "omega", 1, 0, 0, 0, 1 },

  { 28, 6, "Open-loop control/Vision System/Pose Estimation",
    "q", 0, 0, 3, 0, 1 },

  { 29, 0, "Open-loop control/Vision System/Left Camera",
    "", 0, 1, 5, 0, 0 },

  { 30, 0, "Open-loop control/Vision System/Right Camera",
    "", 0, 1, 5, 0, 0 },

  { 31, 10, "Open-loop control/plant/pulse transformer camera/p1",
    "PAN_O", 0, 2, 0, 0, 1 },

  { 32, 10, "Open-loop control/plant/pulse transformer camera/p2",
    "TILT_O", 1, 2, 0, 0, 1 },

  { 33, 11, "Open-loop control/plant/pulse transformer1/p1",
    "Left_PWM", 0, 2, 0, 0, 1 },

  { 34, 11, "Open-loop control/plant/pulse transformer1/p2",
    "Right_PWM", 1, 2, 0, 0, 1 },

  { 35, 16, "Timer control/Read/Data Store Read",
    "", 0, 0, 0, 0, 1 },

  { 36, 0, "Open-loop control/Vision System/All Scopes/Mean",
    "", 0, 0, 0, 0, 1 },

  { 37, 0, "Open-loop control/Vision System/All Scopes/Tapped Delay",
    "", 0, 0, 4, 0, 0 },

  { 38, 0, "Open-loop control/Vision System/All Scopes/theta_tilde",
    "etheta", 0, 0, 0, 0, 1 },

  { 39, 0, "Open-loop control/Vision System/All Scopes/xtilde",
    "ex", 0, 0, 0, 0, 1 },

  { 40, 0, "Open-loop control/Vision System/All Scopes/ytilde",
    "ey", 0, 0, 0, 0, 1 },

  { 41, 7,
    "Open-loop control/plant/Send Serial Packet to Servo Controller And recieve status message from SSC/CheckSum/p1",
    "csum0", 0, 1, 0, 0, 1 },

  { 42, 7,
    "Open-loop control/plant/Send Serial Packet to Servo Controller And recieve status message from SSC/CheckSum/p2",
    "csum1", 1, 1, 0, 0, 1 },

  { 43, 8,
    "Open-loop control/plant/Send Serial Packet to Servo Controller And recieve status message from SSC/Embedded MATLAB Function",
    "CharArray", 0, 1, 6, 0, 0 },

  { 44, 9,
    "Open-loop control/plant/Send Serial Packet to Servo Controller And recieve status message from SSC/Byte Reversal/p1",
    "", 0, 2, 0, 0, 0 },

  { 45, 9,
    "Open-loop control/plant/Send Serial Packet to Servo Controller And recieve status message from SSC/Byte Reversal/p2",
    "", 1, 2, 0, 0, 0 },

  { 46, 9,
    "Open-loop control/plant/Send Serial Packet to Servo Controller And recieve status message from SSC/Byte Reversal/p3",
    "", 2, 2, 0, 0, 0 },

  { 47, 9,
    "Open-loop control/plant/Send Serial Packet to Servo Controller And recieve status message from SSC/Byte Reversal/p4",
    "", 3, 2, 0, 0, 0 },

  { 48, 9,
    "Open-loop control/plant/Send Serial Packet to Servo Controller And recieve status message from SSC/Byte Reversal/p5",
    "", 4, 2, 0, 0, 0 },

  { 49, 9,
    "Open-loop control/plant/Send Serial Packet to Servo Controller And recieve status message from SSC/Byte Reversal/p6",
    "", 5, 2, 0, 0, 0 },

  { 50, 9,
    "Open-loop control/plant/Send Serial Packet to Servo Controller And recieve status message from SSC/Byte Reversal/p7",
    "", 6, 2, 0, 0, 0 },

  { 51, 9,
    "Open-loop control/plant/Send Serial Packet to Servo Controller And recieve status message from SSC/Byte Reversal/p8",
    "", 7, 2, 0, 0, 0 },

  { 52, 9,
    "Open-loop control/plant/Send Serial Packet to Servo Controller And recieve status message from SSC/Pack",
    "", 0, 1, 6, 0, 0 },

  {
    0, 0, NULL, NULL, 0, 0, 0, 0, 0
  }
};

static const rtwCAPI_BlockParameters rtBlockParameters[] = {
  /* addrMapIndex, blockPath,
   * paramName, dataTypeIndex, dimIndex, fixPtIdx
   */
  { 53, "packet size",
    "Value", 0, 0, 0 },

  { 54, "packet size4",
    "Value", 0, 0, 0 },

  { 55, "RS232 Binary Receive1",
    "P1", 0, 0, 0 },

  { 56, "RS232 Binary Receive1",
    "P2", 0, 0, 0 },

  { 57, "RS232 Binary Receive1",
    "P3", 0, 0, 0 },

  { 58, "RS232 Setup ",
    "P1", 0, 0, 0 },

  { 59, "RS232 Setup ",
    "P2", 0, 0, 0 },

  { 60, "RS232 Setup ",
    "P3", 0, 0, 0 },

  { 61, "RS232 Setup ",
    "P4", 0, 0, 0 },

  { 62, "RS232 Setup ",
    "P5", 0, 0, 0 },

  { 63, "RS232 Setup ",
    "P6", 0, 0, 0 },

  { 64, "RS232 Setup ",
    "P7", 0, 0, 0 },

  { 65, "RS232 Setup ",
    "P8", 0, 0, 0 },

  { 66, "RS232 Setup ",
    "P9", 0, 0, 0 },

  { 67, "RS232 Setup ",
    "P12", 0, 0, 0 },

  { 68, "RS232 Setup ",
    "P15", 0, 0, 0 },

  { 69, "RS232 Setup ",
    "P16", 0, 0, 0 },

  { 70, "RS232 Setup ",
    "P17", 0, 0, 0 },

  { 71, "RS232 Setup ",
    "P18", 0, 0, 0 },

  { 72, "RS232 Setup ",
    "P19", 0, 0, 0 },

  { 73, "Open-loop control/PWM_L",
    "Value", 0, 0, 0 },

  { 74, "Open-loop control/PWM_R",
    "Value", 0, 0, 0 },

  { 75, "Timer control/Data Store Memory",
    "InitialValue", 0, 0, 0 },

  { 76, "Timer control/Switch",
    "Threshold", 0, 0, 0 },

  { 77, "Open-loop control/Vision System/Object Model",
    "Value", 0, 7, 0 },

  { 78, "Open-loop control/Vision System/T",
    "Value", 0, 0, 0 },

  { 79, "Open-loop control/Vision System/k",
    "Value", 0, 0, 0 },

  { 80, "Open-loop control/Vision System/ks",
    "Value", 0, 0, 0 },

  { 81, "Open-loop control/Vision System/kx",
    "Value", 0, 0, 0 },

  { 82, "Open-loop control/Vision System/Left Camera",
    "P1", 0, 0, 0 },

  { 83, "Open-loop control/Vision System/Left Camera",
    "P2", 0, 0, 0 },

  { 84, "Open-loop control/Vision System/Left Camera",
    "P3", 0, 0, 0 },

  { 85, "Open-loop control/Vision System/Left Camera",
    "P4", 0, 0, 0 },

  { 86, "Open-loop control/Vision System/Left Camera",
    "P5", 0, 0, 0 },

  { 87, "Open-loop control/Vision System/Left Camera",
    "P6", 0, 0, 0 },

  { 88, "Open-loop control/Vision System/Left Camera",
    "P7", 0, 0, 0 },

  { 89, "Open-loop control/Vision System/Left Camera",
    "P8", 0, 0, 0 },

  { 90, "Open-loop control/Vision System/Left Camera",
    "P9", 0, 8, 0 },

  { 91, "Open-loop control/Vision System/Right Camera",
    "P1", 0, 0, 0 },

  { 92, "Open-loop control/Vision System/Right Camera",
    "P2", 0, 0, 0 },

  { 93, "Open-loop control/Vision System/Right Camera",
    "P3", 0, 0, 0 },

  { 94, "Open-loop control/Vision System/Right Camera",
    "P4", 0, 0, 0 },

  { 95, "Open-loop control/Vision System/Right Camera",
    "P5", 0, 0, 0 },

  { 96, "Open-loop control/Vision System/Right Camera",
    "P6", 0, 0, 0 },

  { 97, "Open-loop control/Vision System/Right Camera",
    "P7", 0, 0, 0 },

  { 98, "Open-loop control/Vision System/Right Camera",
    "P8", 0, 0, 0 },

  { 99, "Open-loop control/Vision System/Right Camera",
    "P9", 0, 8, 0 },

  { 100, "Open-loop control/plant/Servo1",
    "Value", 2, 0, 0 },

  { 101, "Open-loop control/Vision System/All Scopes/Tapped Delay",
    "vinit", 0, 4, 0 },

  { 102,
    "Open-loop control/plant/Send Serial Packet to Servo Controller And recieve status message from SSC/Count",
    "Value", 1, 0, 0 },

  { 103,
    "Open-loop control/plant/Send Serial Packet to Servo Controller And recieve status message from SSC/ID",
    "Value", 1, 0, 0 },

  { 104,
    "Open-loop control/plant/Send Serial Packet to Servo Controller And recieve status message from SSC/SYNC 0",
    "Value", 1, 0, 0 },

  { 105,
    "Open-loop control/plant/Send Serial Packet to Servo Controller And recieve status message from SSC/SYNC 1",
    "Value", 1, 0, 0 },

  { 106,
    "Open-loop control/plant/Send Serial Packet to Servo Controller And recieve status message from SSC/RS232 Binary Send1",
    "P1", 0, 0, 0 },

  { 107,
    "Open-loop control/plant/Send Serial Packet to Servo Controller And recieve status message from SSC/RS232 Binary Send1",
    "P2", 0, 0, 0 },

  { 108,
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
  &RobotControl1_640_480_B.enable,     /* 0: Signal */
  &RobotControl1_640_480_B.Clock,      /* 1: Signal */
  &RobotControl1_640_480_B.packetsize, /* 2: Signal */
  &RobotControl1_640_480_B.packetsize4,/* 3: Signal */
  &RobotControl1_640_480_B.RS232BinaryReceive1_o2[0],/* 4: Signal */
  &RobotControl1_640_480_B.PAN,        /* 5: Signal */
  &RobotControl1_640_480_B.TILT,       /* 6: Signal */
  &RobotControl1_640_480_B.PWM_L,      /* 7: Signal */
  &RobotControl1_640_480_B.PWM_R,      /* 8: Signal */
  &RobotControl1_640_480_B.CCS,        /* 9: Signal */
  &RobotControl1_640_480_B.Unpack_o1,  /* 10: Signal */
  &RobotControl1_640_480_B.Unpack_o2,  /* 11: Signal */
  &RobotControl1_640_480_B.Unpack_o3,  /* 12: Signal */
  &RobotControl1_640_480_B.Unpack_o4,  /* 13: Signal */
  &RobotControl1_640_480_B.Unpack_o5,  /* 14: Signal */
  &RobotControl1_640_480_B.Unpack_o6,  /* 15: Signal */
  &RobotControl1_640_480_B.Unpack_o7,  /* 16: Signal */
  &RobotControl1_640_480_B.Unpack_o8,  /* 17: Signal */
  &RobotControl1_640_480_B.Subtract,   /* 18: Signal */
  &RobotControl1_640_480_B.Subtract1,  /* 19: Signal */
  &RobotControl1_640_480_B.Switch,     /* 20: Signal */
  &RobotControl1_640_480_B.UnaryMinus, /* 21: Signal */
  &RobotControl1_640_480_B.L_p[0],     /* 22: Signal */
  &RobotControl1_640_480_B.R_p[0],     /* 23: Signal */
  &RobotControl1_640_480_B.qr[0],      /* 24: Signal */
  &RobotControl1_640_480_B.ur[0],      /* 25: Signal */
  &RobotControl1_640_480_B.v,          /* 26: Signal */
  &RobotControl1_640_480_B.omega,      /* 27: Signal */
  &RobotControl1_640_480_B.q[0],       /* 28: Signal */
  &RobotControl1_640_480_B.LeftCamera[0],/* 29: Signal */
  &RobotControl1_640_480_B.RightCamera[0],/* 30: Signal */
  &RobotControl1_640_480_B.PAN_O,      /* 31: Signal */
  &RobotControl1_640_480_B.TILT_O,     /* 32: Signal */
  &RobotControl1_640_480_B.Left_PWM,   /* 33: Signal */
  &RobotControl1_640_480_B.Right_PWM,  /* 34: Signal */
  &RobotControl1_640_480_B.DataStoreRead,/* 35: Signal */
  &RobotControl1_640_480_B.Mean,       /* 36: Signal */
  &RobotControl1_640_480_B.TappedDelay[0],/* 37: Signal */
  &RobotControl1_640_480_B.etheta,     /* 38: Signal */
  &RobotControl1_640_480_B.ex,         /* 39: Signal */
  &RobotControl1_640_480_B.ey,         /* 40: Signal */
  &RobotControl1_640_480_B.csum0,      /* 41: Signal */
  &RobotControl1_640_480_B.csum1,      /* 42: Signal */
  &RobotControl1_640_480_B.CharArray[0],/* 43: Signal */
  &RobotControl1_640_480_B.ByteReversal_o1,/* 44: Signal */
  &RobotControl1_640_480_B.ByteReversal_o2,/* 45: Signal */
  &RobotControl1_640_480_B.ByteReversal_o3,/* 46: Signal */
  &RobotControl1_640_480_B.ByteReversal_o4,/* 47: Signal */
  &RobotControl1_640_480_B.ByteReversal_o5,/* 48: Signal */
  &RobotControl1_640_480_B.ByteReversal_o6,/* 49: Signal */
  &RobotControl1_640_480_B.ByteReversal_o7,/* 50: Signal */
  &RobotControl1_640_480_B.ByteReversal_o8,/* 51: Signal */
  &RobotControl1_640_480_B.Pack[0],    /* 52: Signal */
  &RobotControl1_640_480_P.packetsize_Value,/* 53: Block Parameter */
  &RobotControl1_640_480_P.packetsize4_Value,/* 54: Block Parameter */
  &RobotControl1_640_480_P.RS232BinaryReceive1_P1,/* 55: Block Parameter */
  &RobotControl1_640_480_P.RS232BinaryReceive1_P2,/* 56: Block Parameter */
  &RobotControl1_640_480_P.RS232BinaryReceive1_P3,/* 57: Block Parameter */
  &RobotControl1_640_480_P.RS232Setup_P1,/* 58: Block Parameter */
  &RobotControl1_640_480_P.RS232Setup_P2,/* 59: Block Parameter */
  &RobotControl1_640_480_P.RS232Setup_P3,/* 60: Block Parameter */
  &RobotControl1_640_480_P.RS232Setup_P4,/* 61: Block Parameter */
  &RobotControl1_640_480_P.RS232Setup_P5,/* 62: Block Parameter */
  &RobotControl1_640_480_P.RS232Setup_P6,/* 63: Block Parameter */
  &RobotControl1_640_480_P.RS232Setup_P7,/* 64: Block Parameter */
  &RobotControl1_640_480_P.RS232Setup_P8,/* 65: Block Parameter */
  &RobotControl1_640_480_P.RS232Setup_P9,/* 66: Block Parameter */
  &RobotControl1_640_480_P.RS232Setup_P12,/* 67: Block Parameter */
  &RobotControl1_640_480_P.RS232Setup_P15,/* 68: Block Parameter */
  &RobotControl1_640_480_P.RS232Setup_P16,/* 69: Block Parameter */
  &RobotControl1_640_480_P.RS232Setup_P17,/* 70: Block Parameter */
  &RobotControl1_640_480_P.RS232Setup_P18,/* 71: Block Parameter */
  &RobotControl1_640_480_P.RS232Setup_P19,/* 72: Block Parameter */
  &RobotControl1_640_480_P.PWM_L_Value,/* 73: Block Parameter */
  &RobotControl1_640_480_P.PWM_R_Value,/* 74: Block Parameter */
  &RobotControl1_640_480_P.DataStoreMemory_InitialValue,/* 75: Block Parameter */
  &RobotControl1_640_480_P.Switch_Threshold,/* 76: Block Parameter */
  &RobotControl1_640_480_P.ObjectModel_Value[0],/* 77: Block Parameter */
  &RobotControl1_640_480_P.T_Value,    /* 78: Block Parameter */
  &RobotControl1_640_480_P.k_Value,    /* 79: Block Parameter */
  &RobotControl1_640_480_P.ks_Value,   /* 80: Block Parameter */
  &RobotControl1_640_480_P.kx_Value,   /* 81: Block Parameter */
  &RobotControl1_640_480_P.LeftCamera_P1,/* 82: Block Parameter */
  &RobotControl1_640_480_P.LeftCamera_P2,/* 83: Block Parameter */
  &RobotControl1_640_480_P.LeftCamera_P3,/* 84: Block Parameter */
  &RobotControl1_640_480_P.LeftCamera_P4,/* 85: Block Parameter */
  &RobotControl1_640_480_P.LeftCamera_P5,/* 86: Block Parameter */
  &RobotControl1_640_480_P.LeftCamera_P6,/* 87: Block Parameter */
  &RobotControl1_640_480_P.LeftCamera_P7,/* 88: Block Parameter */
  &RobotControl1_640_480_P.LeftCamera_P8,/* 89: Block Parameter */
  &RobotControl1_640_480_P.LeftCamera_P9[0],/* 90: Block Parameter */
  &RobotControl1_640_480_P.RightCamera_P1,/* 91: Block Parameter */
  &RobotControl1_640_480_P.RightCamera_P2,/* 92: Block Parameter */
  &RobotControl1_640_480_P.RightCamera_P3,/* 93: Block Parameter */
  &RobotControl1_640_480_P.RightCamera_P4,/* 94: Block Parameter */
  &RobotControl1_640_480_P.RightCamera_P5,/* 95: Block Parameter */
  &RobotControl1_640_480_P.RightCamera_P6,/* 96: Block Parameter */
  &RobotControl1_640_480_P.RightCamera_P7,/* 97: Block Parameter */
  &RobotControl1_640_480_P.RightCamera_P8,/* 98: Block Parameter */
  &RobotControl1_640_480_P.RightCamera_P9[0],/* 99: Block Parameter */
  &RobotControl1_640_480_P.Servo1_Value,/* 100: Block Parameter */
  &RobotControl1_640_480_P.TappedDelay_vinit[0],/* 101: Block Parameter */
  &RobotControl1_640_480_P.Count_Value,/* 102: Block Parameter */
  &RobotControl1_640_480_P.ID_Value,   /* 103: Block Parameter */
  &RobotControl1_640_480_P.SYNC0_Value,/* 104: Block Parameter */
  &RobotControl1_640_480_P.SYNC1_Value,/* 105: Block Parameter */
  &RobotControl1_640_480_P.RS232BinarySend1_P1,/* 106: Block Parameter */
  &RobotControl1_640_480_P.RS232BinarySend1_P2,/* 107: Block Parameter */
  &RobotControl1_640_480_P.RS232BinarySend1_P3/* 108: Block Parameter */
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

  { rtwCAPI_MATRIX_COL_MAJOR, 4, 2, 0 },

  { rtwCAPI_VECTOR, 6, 2, 0 },

  { rtwCAPI_VECTOR, 8, 2, 0 },

  { rtwCAPI_MATRIX_COL_MAJOR_ND, 10, 3, 0 },

  { rtwCAPI_VECTOR, 13, 2, 0 },

  { rtwCAPI_MATRIX_COL_MAJOR, 15, 2, 0 },

  { rtwCAPI_VECTOR, 17, 2, 0 }
};

/* Dimension Array- use dimArrayIndex to access elements of this array */
static const uint_T rtDimensionArray[] = {
  1,                                   /* 0 */
  1,                                   /* 1 */
  8,                                   /* 2 */
  1,                                   /* 3 */
  2,                                   /* 4 */
  4,                                   /* 5 */
  6,                                   /* 6 */
  1,                                   /* 7 */
  2,                                   /* 8 */
  1,                                   /* 9 */
  480,                                 /* 10 */
  640,                                 /* 11 */
  3,                                   /* 12 */
  22,                                  /* 13 */
  1,                                   /* 14 */
  4,                                   /* 15 */
  4,                                   /* 16 */
  1,                                   /* 17 */
  3                                    /* 18 */
};

/* C-API stores floating point values in an array. The elements of this  *
 * are unique. This ensures that values which are shared across the model*
 * are stored in the most efficient way. These values are referenced by  *
 *           - rtwCAPI_FixPtMap.fracSlopePtr,                            *
 *           - rtwCAPI_FixPtMap.biasPtr,                                 *
 *           - rtwCAPI_SampleTimeMap.samplePeriodPtr,                    *
 *           - rtwCAPI_SampleTimeMap.sampleOffsetPtr                     */
static const real_T rtcapiStoredFloats[] = {
  0.033333333333333333, 0.0
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
  { rtBlockSignals, 53 },

  { rtBlockParameters, 56,
    rtModelParameters, 0 },

  { NULL, 0 },

  { rtDataTypeMap, rtDimensionMap, rtFixPtMap,
    rtElementMap, rtSampleTimeMap, rtDimensionArray },
  "float",

  { 2758163337U,
    2301892844U,
    2012447351U,
    1341648013U },
  NULL
};

/* Cache pointers into DataMapInfo substructure of RTModel */
void RobotControl1_640_480_InitializeDataMapInfo
  (RT_MODEL_RobotControl1_640_480_T *RobotControl1_640_480_M
   )
{
  /* Set C-API version */
  rtwCAPI_SetVersion(RobotControl1_640_480_M->DataMapInfo.mmi, 1);

  /* Cache static C-API data into the Real-time Model Data structure */
  rtwCAPI_SetStaticMap(RobotControl1_640_480_M->DataMapInfo.mmi, &mmiStatic);

  /* Cache static C-API logging data into the Real-time Model Data structure */
  rtwCAPI_SetLoggingStaticMap(RobotControl1_640_480_M->DataMapInfo.mmi, NULL);

  /* Cache C-API Data Addresses into the Real-Time Model Data structure */
  rtwCAPI_SetDataAddressMap(RobotControl1_640_480_M->DataMapInfo.mmi,
    rtDataAddrMap);

  /* Cache C-API Data Run-Time Dimension Buffer Addresses into the Real-Time Model Data structure */
  rtwCAPI_SetVarDimsAddressMap(RobotControl1_640_480_M->DataMapInfo.mmi,
    rtVarDimsAddrMap);

  /* Cache C-API rtp Address and size  into the Real-Time Model Data structure */
  RobotControl1_640_480_M->DataMapInfo.mmi.InstanceMap.rtpAddress =
    rtmGetDefaultParam(RobotControl1_640_480_M);
  RobotControl1_640_480_M->DataMapInfo.mmi.staticMap->rtpSize = sizeof
    (P_RobotControl1_640_480_T);

  /* Cache the instance C-API logging pointer */
  rtwCAPI_SetInstanceLoggingInfo(RobotControl1_640_480_M->DataMapInfo.mmi, NULL);

  /* Set Reference to submodels */
  rtwCAPI_SetChildMMIArray(RobotControl1_640_480_M->DataMapInfo.mmi, NULL);
  rtwCAPI_SetChildMMIArrayLen(RobotControl1_640_480_M->DataMapInfo.mmi, 0);
}

/* EOF: RobotControl1_640_480_capi.c */
