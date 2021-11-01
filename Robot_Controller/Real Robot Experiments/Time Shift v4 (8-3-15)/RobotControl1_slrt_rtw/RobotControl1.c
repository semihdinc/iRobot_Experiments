/*
 * RobotControl1.c
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

#include "rt_logging_mmi.h"
#include "RobotControl1_capi.h"
#include "RobotControl1.h"
#include "RobotControl1_private.h"

/* Block signals (auto storage) */
B_RobotControl1_T RobotControl1_B;

/* Block states (auto storage) */
DW_RobotControl1_T RobotControl1_DW;

/* Previous zero-crossings (trigger) states */
PrevZCX_RobotControl1_T RobotControl1_PrevZCX;

/* Real-time model */
RT_MODEL_RobotControl1_T RobotControl1_M_;
RT_MODEL_RobotControl1_T *const RobotControl1_M = &RobotControl1_M_;

/* Output and update for function-call system: '<Root>/Status Extraction' */
void RobotControl1_StatusExtraction(void)
{
  int32_T out;

  /* Unpack: <S4>/Unpack */
  (void) memcpy(&RobotControl1_B.Unpack_o1,
                &RobotControl1_B.RS232BinaryReceive1_o2[0],
                1);
  (void) memcpy(&RobotControl1_B.Unpack_o2,
                &RobotControl1_B.RS232BinaryReceive1_o2[1],
                1);
  (void) memcpy(&RobotControl1_B.Unpack_o3,
                &RobotControl1_B.RS232BinaryReceive1_o2[2],
                1);
  (void) memcpy(&RobotControl1_B.Unpack_o4,
                &RobotControl1_B.RS232BinaryReceive1_o2[3],
                1);
  (void) memcpy(&RobotControl1_B.Unpack_o5,
                &RobotControl1_B.RS232BinaryReceive1_o2[4],
                1);
  (void) memcpy(&RobotControl1_B.Unpack_o6,
                &RobotControl1_B.RS232BinaryReceive1_o2[5],
                1);
  (void) memcpy(&RobotControl1_B.Unpack_o7,
                &RobotControl1_B.RS232BinaryReceive1_o2[6],
                1);
  (void) memcpy(&RobotControl1_B.Unpack_o8,
                &RobotControl1_B.RS232BinaryReceive1_o2[7],
                1);

  /* MATLAB Function: '<S4>/Command State' */
  /* MATLAB Function 'Status Extraction/Command State': '<S35>:1' */
  /*  This block supports an embeddable subset of the MATLAB language. */
  /*  See the help menu for details.  */
  /*  outputs the command channel state (CCS) given status output */
  /*  first check that have proper status message */
  /*  ID = 10 for SSC status info */
  if (RobotControl1_B.Unpack_o2 == 10) {
    /* '<S35>:1:8' */
    if (RobotControl1_B.Unpack_o5 == 2) {
      /* '<S35>:1:9' */
      /* '<S35>:1:10' */
      out = 1;
    } else if (RobotControl1_B.Unpack_o5 == 4) {
      /* '<S35>:1:11' */
      /* '<S35>:1:12' */
      out = 2;
    } else if (RobotControl1_B.Unpack_o5 == 6) {
      /* '<S35>:1:13' */
      /* '<S35>:1:14' */
      out = 3;
    } else {
      /* '<S35>:1:16' */
      out = 0;
    }
  } else {
    /* '<S35>:1:19' */
    out = 4;
  }

  /* '<S35>:1:21' */
  RobotControl1_B.CCS = out;

  /* End of MATLAB Function: '<S4>/Command State' */
  RobotControl1_DW.StatusExtraction_SubsysRanBC = 4;
}

/*
 * Forced non-inlined (FNI) function call stub
 * for '<Root>/Status Extraction'
 */
boolean_T RobotControl1_StatusExtractionFNI(RT_MODEL_RobotControl1_T *const
  RobotControl1_M, int_T controlPortIdx, int_T tid)
{
  RobotControl1_StatusExtraction();
  UNUSED_PARAMETER(RobotControl1_M);
  UNUSED_PARAMETER(controlPortIdx);
  UNUSED_PARAMETER(tid);
  return (1);
}

/*
 * Forced non-inlined (FNI) function call stub
 * for '<Root>/Status Extraction'
 */
boolean_T RobotControl1_StatusExtraction_InitFNI(RT_MODEL_RobotControl1_T *const
  RobotControl1_M, int_T controlPortIdx, int_T tid)
{
  UNUSED_PARAMETER(RobotControl1_M);
  UNUSED_PARAMETER(controlPortIdx);
  UNUSED_PARAMETER(tid);
  return (1);
}

real_T rt_roundd_snf(real_T u)
{
  real_T y;
  if (fabs(u) < 4.503599627370496E+15) {
    if (u >= 0.5) {
      y = floor(u + 0.5);
    } else if (u > -0.5) {
      y = u * 0.0;
    } else {
      y = ceil(u - 0.5);
    }
  } else {
    y = u;
  }

  return y;
}

/* Model output function */
static void RobotControl1_output(void)
{
  uint8_T servo1a;
  uint8_T servo2a;
  uint8_T servo3a;
  uint8_T servo4a;
  uint8_T servo5a;
  uint8_T servo6a;
  uint8_T servo7a;
  uint8_T servo8a;
  uint8_T byte[18];
  uint16_T cksum0;
  uint16_T cksum1;
  uint16_T x;
  ZCEventType zcEvent;
  int32_T i;
  uint32_T qY;
  real_T tmp;

  /* Reset subsysRan breadcrumbs */
  srClearBC(RobotControl1_DW.Openloopcontrol_SubsysRanBC);

  /* Reset subsysRan breadcrumbs */
  srClearBC(RobotControl1_DW.StatusExtraction_SubsysRanBC);

  /* Reset subsysRan breadcrumbs */
  srClearBC(RobotControl1_DW.Read_SubsysRanBC);

  /* Reset subsysRan breadcrumbs */
  srClearBC(RobotControl1_DW.Write_SubsysRanBC);

  /* Clock: '<Root>/Clock' */
  RobotControl1_B.Clock = RobotControl1_M->Timing.t[0];

  /* UnitDelay: '<Root>/Unit Delay' */
  RobotControl1_B.UnitDelay = RobotControl1_DW.UnitDelay_DSTATE;

  /* Outputs for Triggered SubSystem: '<Root>/Read' incorporates:
   *  TriggerPort: '<S2>/Trigger'
   */
  zcEvent = rt_ZCFcn(RISING_ZERO_CROSSING,&RobotControl1_PrevZCX.Read_Trig_ZCE,
                     (RobotControl1_B.UnitDelay));
  if (zcEvent != NO_ZCEVENT) {
    /* DataStoreRead: '<S2>/Data Store Read' */
    RobotControl1_B.DataStoreRead = RobotControl1_DW.t_start;
    RobotControl1_DW.Read_SubsysRanBC = 4;
  }

  /* End of Outputs for SubSystem: '<Root>/Read' */

  /* Sum: '<Root>/Add' */
  RobotControl1_B.Add = RobotControl1_B.Clock - RobotControl1_B.DataStoreRead;

  /* UnitDelay: '<Root>/Unit Delay1' */
  RobotControl1_B.UnitDelay1 = RobotControl1_DW.UnitDelay1_DSTATE;

  /* Outputs for Enabled SubSystem: '<Root>/Open-loop control' incorporates:
   *  EnablePort: '<S1>/Enable'
   */
  if (RobotControl1_B.UnitDelay1 > 0.0) {
    if (!RobotControl1_DW.Openloopcontrol_MODE) {
      RobotControl1_DW.Openloopcontrol_MODE = true;
    }
  } else {
    if (RobotControl1_DW.Openloopcontrol_MODE) {
      RobotControl1_DW.Openloopcontrol_MODE = false;
    }
  }

  if (RobotControl1_DW.Openloopcontrol_MODE) {
    /* ok to acquire for <S14>/S-Function */
    RobotControl1_DW.SFunction_IWORK_o.AcquireOK = 1;

    /* ok to acquire for <S15>/S-Function */
    RobotControl1_DW.SFunction_IWORK_h.AcquireOK = 1;

    /* ok to acquire for <S16>/S-Function */
    RobotControl1_DW.SFunction_IWORK_l.AcquireOK = 1;

    /* MATLAB Function: '<S1>/MATLAB Function 2' */
    /* MATLAB Function 'Open-loop control/MATLAB Function 2': '<S10>:1' */
    /* '<S10>:1:9' */
    /* '<S10>:1:11' */
    RobotControl1_B.v = sin(6.2831853071795862 * RobotControl1_B.Add / 10.0) *
      0.9;

    /* '<S10>:1:12' */
    RobotControl1_B.omega = sin((RobotControl1_B.Add - 1.0) * 6.2831853071795862
      / 10.0) * 0.5;

    /* MATLAB Function: '<S1>/ MATLAB Function1' */
    /* MATLAB Function 'Open-loop control/ MATLAB Function1': '<S8>:1' */
    /* '<S8>:1:3' */
    /* '<S8>:1:4' */
    /* '<S8>:1:8' */
    /* '<S8>:1:9' */
    /* '<S8>:1:11' */
    /* '<S8>:1:13' */
    /* '<S8>:1:16' */
    /* '<S8>:1:19' */
    /* '<S8>:1:24' */
    /* PWM(1); */
    /* '<S8>:1:25' */
    /* PWM(2); */
    RobotControl1_B.PWM_L = 1350.0;
    RobotControl1_B.PWM_R = 1350.0;

    /* MATLAB Function: '<S17>/pulse transformer1' */
    /* MATLAB Function 'Open-loop control/plant/pulse transformer1': '<S32>:1' */
    /*  This block supports an embeddable subset of the MATLAB language. */
    /*  Converts output angle and speed value into a pulse width */
    /*  The input angle is between +pi/4 and -pi/4  (from limits) */
    /*  The speed is between 140 and 0 rps(Hz) */
    /*  angle=uint16(angI*16.25*180/pi + 1505.5); */
    /*  pw=-0.0002*speedI^3 +0.03*speedI^2 -2.66*speedI +1528; */
    /*  throtle = uint16(pw); */
    /*   */
    /* '<S32>:1:11' */
    tmp = rt_roundd_snf(RobotControl1_B.PWM_L);
    if (tmp < 65536.0) {
      if (tmp >= 0.0) {
        cksum0 = (uint16_T)tmp;
      } else {
        cksum0 = 0U;
      }
    } else {
      cksum0 = MAX_uint16_T;
    }

    RobotControl1_B.Left_PWM = cksum0;

    /* '<S32>:1:12' */
    tmp = rt_roundd_snf(RobotControl1_B.PWM_R);
    if (tmp < 65536.0) {
      if (tmp >= 0.0) {
        cksum0 = (uint16_T)tmp;
      } else {
        cksum0 = 0U;
      }
    } else {
      cksum0 = MAX_uint16_T;
    }

    RobotControl1_B.Right_PWM = cksum0;

    /* End of MATLAB Function: '<S17>/pulse transformer1' */

    /* MATLAB Function: '<S1>/Camera Servos Function' */
    /* MATLAB Function 'Open-loop control/Camera Servos Function': '<S9>:1' */
    /* Insert Function for camera servo manipulation here */
    /* By default, this function holds camera servos at their center position (1500 ms) */
    /* '<S9>:1:4' */
    RobotControl1_B.PAN = 1525.0;

    /* '<S9>:1:5' */
    RobotControl1_B.TILT = 1400.0;

    /* MATLAB Function: '<S17>/pulse transformer camera' */
    /* MATLAB Function 'Open-loop control/plant/pulse transformer camera': '<S31>:1' */
    /* '<S31>:1:3' */
    tmp = rt_roundd_snf(RobotControl1_B.PAN);
    if (tmp < 65536.0) {
      if (tmp >= 0.0) {
        cksum0 = (uint16_T)tmp;
      } else {
        cksum0 = 0U;
      }
    } else {
      cksum0 = MAX_uint16_T;
    }

    RobotControl1_B.PAN_O = cksum0;

    /* '<S31>:1:4' */
    tmp = rt_roundd_snf(RobotControl1_B.TILT);
    if (tmp < 65536.0) {
      if (tmp >= 0.0) {
        cksum0 = (uint16_T)tmp;
      } else {
        cksum0 = 0U;
      }
    } else {
      cksum0 = MAX_uint16_T;
    }

    RobotControl1_B.TILT_O = cksum0;

    /* End of MATLAB Function: '<S17>/pulse transformer camera' */

    /* Outputs for Atomic SubSystem: '<S17>/Send Serial Packet to Servo Controller And recieve status message from SSC' */

    /* ReverseEndian: <S30>/Byte Reversal */

    /* 2 byte-wide input datatypes */
    ((uint16_T *)&RobotControl1_B.ByteReversal_o1)[0] =
      SWAP16(((uint16_T *)&RobotControl1_P.Servo1_Value)[0]);
    ((uint16_T *)&RobotControl1_B.ByteReversal_o2)[0] =
      SWAP16(((uint16_T *)&RobotControl1_B.Left_PWM)[0]);
    ((uint16_T *)&RobotControl1_B.ByteReversal_o3)[0] =
      SWAP16(((uint16_T *)&RobotControl1_B.Right_PWM)[0]);
    ((uint16_T *)&RobotControl1_B.ByteReversal_o4)[0] =
      SWAP16(((uint16_T *)&RobotControl1_P.Servo1_Value)[0]);
    ((uint16_T *)&RobotControl1_B.ByteReversal_o5)[0] =
      SWAP16(((uint16_T *)&RobotControl1_B.PAN_O)[0]);
    ((uint16_T *)&RobotControl1_B.ByteReversal_o6)[0] =
      SWAP16(((uint16_T *)&RobotControl1_P.Servo1_Value)[0]);
    ((uint16_T *)&RobotControl1_B.ByteReversal_o7)[0] =
      SWAP16(((uint16_T *)&RobotControl1_B.TILT_O)[0]);
    ((uint16_T *)&RobotControl1_B.ByteReversal_o8)[0] =
      SWAP16(((uint16_T *)&RobotControl1_P.Servo1_Value)[0]);

    /* MATLAB Function: '<S30>/CheckSum' incorporates:
     *  Constant: '<S17>/Servo1'
     *  Constant: '<S30>/Count'
     *  Constant: '<S30>/ID'
     */
    /* MATLAB Function 'Open-loop control/plant/Send Serial Packet to Servo Controller And recieve status message from SSC/CheckSum': '<S33>:1' */
    /*  The purpose of this function is to calculate the fletcher checksum values */
    /*  as defined in internet RFC 1145 */
    /*  See page 11 of the Microbotics Servo Switch/Controller Users Manual for  */
    /*  more information */
    /*  break into bytes */
    /* '<S33>:1:8' */
    servo1a = (uint8_T)((uint32_T)RobotControl1_P.Servo1_Value >> 8);

    /* '<S33>:1:8' */
    /* '<S33>:1:9' */
    servo2a = (uint8_T)((uint32_T)RobotControl1_B.Left_PWM >> 8);

    /* '<S33>:1:9' */
    /* '<S33>:1:10' */
    servo3a = (uint8_T)((uint32_T)RobotControl1_B.Right_PWM >> 8);

    /* '<S33>:1:10' */
    /* '<S33>:1:11' */
    servo4a = (uint8_T)((uint32_T)RobotControl1_P.Servo1_Value >> 8);

    /* '<S33>:1:11' */
    /* '<S33>:1:12' */
    servo5a = (uint8_T)((uint32_T)RobotControl1_B.PAN_O >> 8);

    /* '<S33>:1:12' */
    /* '<S33>:1:13' */
    servo6a = (uint8_T)((uint32_T)RobotControl1_P.Servo1_Value >> 8);

    /* '<S33>:1:13' */
    /* '<S33>:1:14' */
    servo7a = (uint8_T)((uint32_T)RobotControl1_B.TILT_O >> 8);

    /* '<S33>:1:14' */
    /* '<S33>:1:15' */
    servo8a = (uint8_T)((uint32_T)RobotControl1_P.Servo1_Value >> 8);

    /* '<S33>:1:15' */
    /* '<S33>:1:16' */
    byte[0] = RobotControl1_P.ID_Value;
    byte[1] = RobotControl1_P.Count_Value;
    byte[2] = servo1a;
    i = RobotControl1_P.Servo1_Value;
    qY = (uint32_T)i - (servo1a << 8);
    if (qY > (uint32_T)i) {
      qY = 0U;
    }

    i = (int32_T)qY;
    cksum0 = (uint16_T)i;
    if (cksum0 > 255) {
      cksum0 = 255U;
    }

    byte[3] = (uint8_T)cksum0;
    byte[4] = servo2a;
    i = RobotControl1_B.Left_PWM;
    qY = (uint32_T)i - (servo2a << 8);
    if (qY > (uint32_T)i) {
      qY = 0U;
    }

    i = (int32_T)qY;
    cksum0 = (uint16_T)i;
    if (cksum0 > 255) {
      cksum0 = 255U;
    }

    byte[5] = (uint8_T)cksum0;
    byte[6] = servo3a;
    i = RobotControl1_B.Right_PWM;
    qY = (uint32_T)i - (servo3a << 8);
    if (qY > (uint32_T)i) {
      qY = 0U;
    }

    i = (int32_T)qY;
    cksum0 = (uint16_T)i;
    if (cksum0 > 255) {
      cksum0 = 255U;
    }

    byte[7] = (uint8_T)cksum0;
    byte[8] = servo4a;
    i = RobotControl1_P.Servo1_Value;
    qY = (uint32_T)i - (servo4a << 8);
    if (qY > (uint32_T)i) {
      qY = 0U;
    }

    i = (int32_T)qY;
    cksum0 = (uint16_T)i;
    if (cksum0 > 255) {
      cksum0 = 255U;
    }

    byte[9] = (uint8_T)cksum0;
    byte[10] = servo5a;
    i = RobotControl1_B.PAN_O;
    qY = (uint32_T)i - (servo5a << 8);
    if (qY > (uint32_T)i) {
      qY = 0U;
    }

    i = (int32_T)qY;
    cksum0 = (uint16_T)i;
    if (cksum0 > 255) {
      cksum0 = 255U;
    }

    byte[11] = (uint8_T)cksum0;
    byte[12] = servo6a;
    i = RobotControl1_P.Servo1_Value;
    qY = (uint32_T)i - (servo6a << 8);
    if (qY > (uint32_T)i) {
      qY = 0U;
    }

    i = (int32_T)qY;
    cksum0 = (uint16_T)i;
    if (cksum0 > 255) {
      cksum0 = 255U;
    }

    byte[13] = (uint8_T)cksum0;
    byte[14] = servo7a;
    i = RobotControl1_B.TILT_O;
    qY = (uint32_T)i - (servo7a << 8);
    if (qY > (uint32_T)i) {
      qY = 0U;
    }

    i = (int32_T)qY;
    cksum0 = (uint16_T)i;
    if (cksum0 > 255) {
      cksum0 = 255U;
    }

    byte[15] = (uint8_T)cksum0;
    byte[16] = servo8a;
    i = RobotControl1_P.Servo1_Value;
    qY = (uint32_T)i - (servo8a << 8);
    if (qY > (uint32_T)i) {
      qY = 0U;
    }

    i = (int32_T)qY;
    cksum0 = (uint16_T)i;
    if (cksum0 > 255) {
      cksum0 = 255U;
    }

    byte[17] = (uint8_T)cksum0;

    /* '<S33>:1:18' */
    cksum0 = 0U;

    /* '<S33>:1:18' */
    cksum1 = 0U;

    /*  initialize to zero */
    /* '<S33>:1:19' */
    i = (int32_T)(RobotControl1_P.Count_Value + 2U);
    if ((uint32_T)i > 255U) {
      i = 255;
    }

    servo1a = (uint8_T)i;

    /* '<S33>:1:19' */
    for (servo2a = 1U; servo2a <= servo1a; servo2a++) {
      /* '<S33>:1:19' */
      /* '<S33>:1:20' */
      qY = (uint32_T)byte[servo2a - 1] + cksum0;
      if (qY > 65535U) {
        qY = 65535U;
      }

      x = (uint16_T)qY;
      cksum0 = (uint16_T)((uint32_T)x - ((int32_T)((uint32_T)x >> 8) << 8));

      /* '<S33>:1:21' */
      qY = (uint32_T)cksum1 + cksum0;
      if (qY > 65535U) {
        qY = 65535U;
      }

      x = (uint16_T)qY;
      cksum1 = (uint16_T)((uint32_T)x - ((int32_T)((uint32_T)x >> 8) << 8));
    }

    /* '<S33>:1:24' */
    if (cksum0 > 255) {
      cksum0 = 255U;
    }

    RobotControl1_B.csum0 = (uint8_T)cksum0;

    /* '<S33>:1:24' */
    if (cksum1 > 255) {
      cksum1 = 255U;
    }

    RobotControl1_B.csum1 = (uint8_T)cksum1;

    /* End of MATLAB Function: '<S30>/CheckSum' */

    /* Pack: <S30>/Pack */
    (void) memcpy(&RobotControl1_B.Pack[0], &RobotControl1_P.SYNC0_Value,
                  1);
    (void) memcpy(&RobotControl1_B.Pack[1], &RobotControl1_P.SYNC1_Value,
                  1);
    (void) memcpy(&RobotControl1_B.Pack[2], &RobotControl1_P.ID_Value,
                  1);
    (void) memcpy(&RobotControl1_B.Pack[3], &RobotControl1_P.Count_Value,
                  1);
    (void) memcpy(&RobotControl1_B.Pack[4], &RobotControl1_B.ByteReversal_o1,
                  2);
    (void) memcpy(&RobotControl1_B.Pack[6], &RobotControl1_B.ByteReversal_o2,
                  2);
    (void) memcpy(&RobotControl1_B.Pack[8], &RobotControl1_B.ByteReversal_o3,
                  2);
    (void) memcpy(&RobotControl1_B.Pack[10], &RobotControl1_B.ByteReversal_o4,
                  2);
    (void) memcpy(&RobotControl1_B.Pack[12], &RobotControl1_B.ByteReversal_o5,
                  2);
    (void) memcpy(&RobotControl1_B.Pack[14], &RobotControl1_B.ByteReversal_o6,
                  2);
    (void) memcpy(&RobotControl1_B.Pack[16], &RobotControl1_B.ByteReversal_o7,
                  2);
    (void) memcpy(&RobotControl1_B.Pack[18], &RobotControl1_B.ByteReversal_o8,
                  2);
    (void) memcpy(&RobotControl1_B.Pack[20], &RobotControl1_B.csum0,
                  1);
    (void) memcpy(&RobotControl1_B.Pack[21], &RobotControl1_B.csum1,
                  1);

    /* MATLAB Function: '<S30>/Embedded MATLAB Function' */
    /* MATLAB Function 'Open-loop control/plant/Send Serial Packet to Servo Controller And recieve status message from SSC/Embedded MATLAB Function': '<S34>:1' */
    /*  */
    /* '<S34>:1:4' */
    for (i = 0; i < 22; i++) {
      RobotControl1_B.CharArray[i] = RobotControl1_B.Pack[i];
    }

    /* End of MATLAB Function: '<S30>/Embedded MATLAB Function' */

    /* Level2 S-Function Block: '<S30>/RS232 Binary Send1' (rs232bsend) */
    {
      SimStruct *rts = RobotControl1_M->childSfunctions[0];
      sfcnOutputs(rts, 1);
    }

    /* End of Outputs for SubSystem: '<S17>/Send Serial Packet to Servo Controller And recieve status message from SSC' */
    srUpdateBC(RobotControl1_DW.Openloopcontrol_SubsysRanBC);
  }

  /* End of Outputs for SubSystem: '<Root>/Open-loop control' */
  /* ok to acquire for <S3>/S-Function */
  RobotControl1_DW.SFunction_IWORK.AcquireOK = 1;

  /* Constant: '<Root>/packet size' */
  RobotControl1_B.packetsize = RobotControl1_P.packetsize_Value;

  /* Constant: '<Root>/packet size4' */
  RobotControl1_B.packetsize4 = RobotControl1_P.packetsize4_Value;

  /* S-Function (rs232brec): '<Root>/RS232 Binary Receive1' */

  /* Level2 S-Function Block: '<Root>/RS232 Binary Receive1' (rs232brec) */
  {
    SimStruct *rts = RobotControl1_M->childSfunctions[1];
    sfcnOutputs(rts, 1);
  }

  /* End of Outputs for S-Function (rs232brec): '<Root>/RS232 Binary Receive1' */

  /* MATLAB Function: '<Root>/status checker' */
  /* MATLAB Function 'status checker': '<S7>:1' */
  /*  This block supports an embeddable subset of the MATLAB language. */
  /*  See the help menu for details.  */
  if (RobotControl1_B.CCS == 1.0) {
    /* '<S7>:1:4' */
    /* '<S7>:1:5' */
    i = 1;
  } else {
    /* '<S7>:1:7' */
    i = -1;
  }

  /* '<S7>:1:9' */
  RobotControl1_B.enable = i;

  /* End of MATLAB Function: '<Root>/status checker' */

  /* Outputs for Triggered SubSystem: '<Root>/Write' incorporates:
   *  TriggerPort: '<S5>/Trigger'
   */
  zcEvent = rt_ZCFcn(RISING_ZERO_CROSSING,&RobotControl1_PrevZCX.Write_Trig_ZCE,
                     (RobotControl1_B.enable));
  if (zcEvent != NO_ZCEVENT) {
    /* DataStoreWrite: '<S5>/Data Store Write' */
    RobotControl1_DW.t_start = RobotControl1_B.Clock;
    RobotControl1_DW.Write_SubsysRanBC = 4;
  }

  /* End of Outputs for SubSystem: '<Root>/Write' */
  /* ok to acquire for <S6>/S-Function */
  RobotControl1_DW.SFunction_IWORK_j.AcquireOK = 1;

  /* Level2 S-Function Block: '<Root>/RS232 Setup ' (rs232setup) */
  {
    SimStruct *rts = RobotControl1_M->childSfunctions[2];
    sfcnOutputs(rts, 1);
  }
}

/* Model update function */
static void RobotControl1_update(void)
{
  /* Update for UnitDelay: '<Root>/Unit Delay' */
  RobotControl1_DW.UnitDelay_DSTATE = RobotControl1_B.enable;

  /* Update for UnitDelay: '<Root>/Unit Delay1' */
  RobotControl1_DW.UnitDelay1_DSTATE = RobotControl1_B.enable;

  /* Update absolute time for base rate */
  /* The "clockTick0" counts the number of times the code of this task has
   * been executed. The absolute time is the multiplication of "clockTick0"
   * and "Timing.stepSize0". Size of "clockTick0" ensures timer will not
   * overflow during the application lifespan selected.
   * Timer of this task consists of two 32 bit unsigned integers.
   * The two integers represent the low bits Timing.clockTick0 and the high bits
   * Timing.clockTickH0. When the low bit overflows to 0, the high bits increment.
   */
  if (!(++RobotControl1_M->Timing.clockTick0)) {
    ++RobotControl1_M->Timing.clockTickH0;
  }

  RobotControl1_M->Timing.t[0] = RobotControl1_M->Timing.clockTick0 *
    RobotControl1_M->Timing.stepSize0 + RobotControl1_M->Timing.clockTickH0 *
    RobotControl1_M->Timing.stepSize0 * 4294967296.0;

  {
    /* Update absolute timer for sample time: [0.13333333333333333s, 0.0s] */
    /* The "clockTick1" counts the number of times the code of this task has
     * been executed. The absolute time is the multiplication of "clockTick1"
     * and "Timing.stepSize1". Size of "clockTick1" ensures timer will not
     * overflow during the application lifespan selected.
     * Timer of this task consists of two 32 bit unsigned integers.
     * The two integers represent the low bits Timing.clockTick1 and the high bits
     * Timing.clockTickH1. When the low bit overflows to 0, the high bits increment.
     */
    if (!(++RobotControl1_M->Timing.clockTick1)) {
      ++RobotControl1_M->Timing.clockTickH1;
    }

    RobotControl1_M->Timing.t[1] = RobotControl1_M->Timing.clockTick1 *
      RobotControl1_M->Timing.stepSize1 + RobotControl1_M->Timing.clockTickH1 *
      RobotControl1_M->Timing.stepSize1 * 4294967296.0;
  }
}

/* Model initialize function */
static void RobotControl1_initialize(void)
{
  /* Start for Enabled SubSystem: '<Root>/Open-loop control' */
  RobotControl1_DW.Openloopcontrol_MODE = false;

  /* S-Function Block: <S14>/S-Function (scblock) */
  {
    int i;
    if ((i = rl32eScopeExists(15)) == 0) {
      if ((i = rl32eDefScope(15,2)) != 0) {
        printf("Error creating scope 15\n");
      } else {
        rl32eAddSignal(15, rl32eGetSignalNo("Add"));
        rl32eSetTargetScopeSigFt(15,rl32eGetSignalNo("Add"),"t=%3.3f");
        rl32eSetScope(15, 4, 3);
        rl32eSetScope(15, 5, 0);
        rl32eSetScope(15, 6, 1);
        rl32eSetScope(15, 0, 0);
        rl32eSetScope(15, 3, rl32eGetSignalNo("Add"));
        rl32eSetScope(15, 1, 0.0);
        rl32eSetScope(15, 2, 0);
        rl32eSetScope(15, 9, 0);
        rl32eSetTargetScope(15, 1, 0.0);
        rl32eSetTargetScope(15, 11, 0.0);
        rl32eSetTargetScope(15, 10, 0.0);
        xpceScopeAcqOK(15, &RobotControl1_DW.SFunction_IWORK_o.AcquireOK);
      }
    }

    if (i) {
      rl32eRestartAcquisition(15);
    }
  }

  /* S-Function Block: <S15>/S-Function (scblock) */
  {
    int i;
    if ((i = rl32eScopeExists(11)) == 0) {
      if ((i = rl32eDefScope(11,2)) != 0) {
        printf("Error creating scope 11\n");
      } else {
        rl32eAddSignal(11, rl32eGetSignalNo(
          "Open-loop control/ MATLAB Function1/p1"));
        rl32eAddSignal(11, rl32eGetSignalNo(
          "Open-loop control/ MATLAB Function1/p2"));
        rl32eSetTargetScopeSigFt(11,rl32eGetSignalNo(
          "Open-loop control/ MATLAB Function1/p1"),"%15.6f");
        rl32eSetTargetScopeSigFt(11,rl32eGetSignalNo(
          "Open-loop control/ MATLAB Function1/p2"),"%15.6f");
        rl32eSetScope(11, 4, 3);
        rl32eSetScope(11, 5, 0);
        rl32eSetScope(11, 6, 1);
        rl32eSetScope(11, 0, 0);
        rl32eSetScope(11, 3, rl32eGetSignalNo(
          "Open-loop control/ MATLAB Function1/p1"));
        rl32eSetScope(11, 1, 0.0);
        rl32eSetScope(11, 2, 0);
        rl32eSetScope(11, 9, 0);
        rl32eSetTargetScope(11, 1, 0.0);
        rl32eSetTargetScope(11, 11, 0.0);
        rl32eSetTargetScope(11, 10, 0.0);
        xpceScopeAcqOK(11, &RobotControl1_DW.SFunction_IWORK_h.AcquireOK);
      }
    }

    if (i) {
      rl32eRestartAcquisition(11);
    }
  }

  /* S-Function Block: <S16>/S-Function (scblock) */
  {
    int i;
    if ((i = rl32eScopeExists(10)) == 0) {
      if ((i = rl32eDefScope(10,2)) != 0) {
        printf("Error creating scope 10\n");
      } else {
        rl32eAddSignal(10, rl32eGetSignalNo(
          "Open-loop control/MATLAB Function 2/p1"));
        rl32eAddSignal(10, rl32eGetSignalNo(
          "Open-loop control/MATLAB Function 2/p2"));
        rl32eSetTargetScopeSigFt(10,rl32eGetSignalNo(
          "Open-loop control/MATLAB Function 2/p1"),"%15.6f");
        rl32eSetTargetScopeSigFt(10,rl32eGetSignalNo(
          "Open-loop control/MATLAB Function 2/p2"),"%15.6f");
        rl32eSetScope(10, 4, 3);
        rl32eSetScope(10, 5, 0);
        rl32eSetScope(10, 6, 1);
        rl32eSetScope(10, 0, 0);
        rl32eSetScope(10, 3, rl32eGetSignalNo(
          "Open-loop control/MATLAB Function 2/p1"));
        rl32eSetScope(10, 1, 0.0);
        rl32eSetScope(10, 2, 0);
        rl32eSetScope(10, 9, 0);
        rl32eSetTargetScope(10, 1, 0.0);
        rl32eSetTargetScope(10, 11, 0.0);
        rl32eSetTargetScope(10, 10, 0.0);
        xpceScopeAcqOK(10, &RobotControl1_DW.SFunction_IWORK_l.AcquireOK);
      }
    }

    if (i) {
      rl32eRestartAcquisition(10);
    }
  }

  /* Start for Atomic SubSystem: '<S17>/Send Serial Packet to Servo Controller And recieve status message from SSC' */

  /* Level2 S-Function Block: '<S30>/RS232 Binary Send1' (rs232bsend) */
  {
    SimStruct *rts = RobotControl1_M->childSfunctions[0];
    sfcnStart(rts);
    if (ssGetErrorStatus(rts) != (NULL))
      return;
  }

  /* End of Start for SubSystem: '<S17>/Send Serial Packet to Servo Controller And recieve status message from SSC' */

  /* End of Start for SubSystem: '<Root>/Open-loop control' */

  /* S-Function Block: <S3>/S-Function (scblock) */
  {
    int i;
    if ((i = rl32eScopeExists(20)) == 0) {
      if ((i = rl32eDefScope(20,2)) != 0) {
        printf("Error creating scope 20\n");
      } else {
        rl32eAddSignal(20, rl32eGetSignalNo("Status Extraction/Command State"));
        rl32eSetScope(20, 4, 2000);
        rl32eSetScope(20, 5, 0);
        rl32eSetScope(20, 6, 1);
        rl32eSetScope(20, 0, 0);
        rl32eSetScope(20, 3, rl32eGetSignalNo("Status Extraction/Command State"));
        rl32eSetScope(20, 1, 0.0);
        rl32eSetScope(20, 2, 0);
        rl32eSetScope(20, 9, 0);
        rl32eSetTargetScope(20, 1, 3.0);
        rl32eSetTargetScope(20, 11, 0.0);
        rl32eSetTargetScope(20, 10, 0.0);
        xpceScopeAcqOK(20, &RobotControl1_DW.SFunction_IWORK.AcquireOK);
      }
    }

    if (i) {
      rl32eRestartAcquisition(20);
    }
  }

  /* Start for Constant: '<Root>/packet size' */
  RobotControl1_B.packetsize = RobotControl1_P.packetsize_Value;

  /* Start for Constant: '<Root>/packet size4' */
  RobotControl1_B.packetsize4 = RobotControl1_P.packetsize4_Value;

  /* S-Function (rs232brec): '<Root>/RS232 Binary Receive1' */

  /* Level2 S-Function Block: '<Root>/RS232 Binary Receive1' (rs232brec) */
  {
    SimStruct *rts = RobotControl1_M->childSfunctions[1];
    sfcnStart(rts);
    if (ssGetErrorStatus(rts) != (NULL))
      return;
  }

  /* End of Outputs for S-Function (rs232brec): '<Root>/RS232 Binary Receive1' */

  /* S-Function Block: <S6>/S-Function (scblock) */
  {
    int i;
    if ((i = rl32eScopeExists(6)) == 0) {
      if ((i = rl32eDefScope(6,3)) != 0) {
        printf("Error creating scope 6\n");
      } else {
        rl32eAddSignal(6, rl32eGetSignalNo("Status Extraction/Command State"));
        rl32eSetScope(6, 4, 3000);
        rl32eSetScope(6, 5, 0);
        rl32eSetScope(6, 6, 1);
        rl32eSetScope(6, 0, 0);
        xpceFSScopeSet(6, "1sttus01.dat", 1, 512, 0, 536870912);
        rl32eSetScope (6, 10, 1);
        rl32eSetScope(6, 3, rl32eGetSignalNo("Status Extraction/Command State"));
        rl32eSetScope(6, 1, 0.0);
        rl32eSetScope(6, 2, 0);
        rl32eSetScope(6, 9, 0);
        xpceScopeAcqOK(6, &RobotControl1_DW.SFunction_IWORK_j.AcquireOK);
      }
    }

    if (i) {
      rl32eRestartAcquisition(6);
    }
  }

  /* Start for DataStoreMemory: '<Root>/Data Store Memory' */
  RobotControl1_DW.t_start = RobotControl1_P.DataStoreMemory_InitialValue;

  /* Level2 S-Function Block: '<Root>/RS232 Setup ' (rs232setup) */
  {
    SimStruct *rts = RobotControl1_M->childSfunctions[2];
    sfcnStart(rts);
    if (ssGetErrorStatus(rts) != (NULL))
      return;
  }

  RobotControl1_PrevZCX.Read_Trig_ZCE = UNINITIALIZED_ZCSIG;
  RobotControl1_PrevZCX.Write_Trig_ZCE = UNINITIALIZED_ZCSIG;

  /* InitializeConditions for UnitDelay: '<Root>/Unit Delay' */
  RobotControl1_DW.UnitDelay_DSTATE = RobotControl1_P.UnitDelay_InitialCondition;

  /* InitializeConditions for UnitDelay: '<Root>/Unit Delay1' */
  RobotControl1_DW.UnitDelay1_DSTATE =
    RobotControl1_P.UnitDelay1_InitialCondition;
}

/* Model terminate function */
static void RobotControl1_terminate(void)
{
  /* Terminate for Enabled SubSystem: '<Root>/Open-loop control' */

  /* Terminate for Atomic SubSystem: '<S17>/Send Serial Packet to Servo Controller And recieve status message from SSC' */

  /* Level2 S-Function Block: '<S30>/RS232 Binary Send1' (rs232bsend) */
  {
    SimStruct *rts = RobotControl1_M->childSfunctions[0];
    sfcnTerminate(rts);
  }

  /* End of Terminate for SubSystem: '<S17>/Send Serial Packet to Servo Controller And recieve status message from SSC' */

  /* End of Terminate for SubSystem: '<Root>/Open-loop control' */

  /* S-Function (rs232brec): '<Root>/RS232 Binary Receive1' */

  /* Level2 S-Function Block: '<Root>/RS232 Binary Receive1' (rs232brec) */
  {
    SimStruct *rts = RobotControl1_M->childSfunctions[1];
    sfcnTerminate(rts);
  }

  /* End of Outputs for S-Function (rs232brec): '<Root>/RS232 Binary Receive1' */

  /* Level2 S-Function Block: '<Root>/RS232 Setup ' (rs232setup) */
  {
    SimStruct *rts = RobotControl1_M->childSfunctions[2];
    sfcnTerminate(rts);
  }
}

/*========================================================================*
 * Start of Classic call interface                                        *
 *========================================================================*/
void MdlOutputs(int_T tid)
{
  RobotControl1_output();
  UNUSED_PARAMETER(tid);
}

void MdlUpdate(int_T tid)
{
  RobotControl1_update();
  UNUSED_PARAMETER(tid);
}

void MdlInitializeSizes(void)
{
}

void MdlInitializeSampleTimes(void)
{
}

void MdlInitialize(void)
{
}

void MdlStart(void)
{
  RobotControl1_initialize();
}

void MdlTerminate(void)
{
  RobotControl1_terminate();
}

/* Registration function */
RT_MODEL_RobotControl1_T *RobotControl1(void)
{
  /* Registration code */

  /* initialize non-finites */
  rt_InitInfAndNaN(sizeof(real_T));

  /* initialize real-time model */
  (void) memset((void *)RobotControl1_M, 0,
                sizeof(RT_MODEL_RobotControl1_T));

  {
    /* Setup solver object */
    rtsiSetSimTimeStepPtr(&RobotControl1_M->solverInfo,
                          &RobotControl1_M->Timing.simTimeStep);
    rtsiSetTPtr(&RobotControl1_M->solverInfo, &rtmGetTPtr(RobotControl1_M));
    rtsiSetStepSizePtr(&RobotControl1_M->solverInfo,
                       &RobotControl1_M->Timing.stepSize0);
    rtsiSetErrorStatusPtr(&RobotControl1_M->solverInfo, (&rtmGetErrorStatus
      (RobotControl1_M)));
    rtsiSetRTModelPtr(&RobotControl1_M->solverInfo, RobotControl1_M);
  }

  rtsiSetSimTimeStep(&RobotControl1_M->solverInfo, MAJOR_TIME_STEP);
  rtsiSetSolverName(&RobotControl1_M->solverInfo,"FixedStepDiscrete");
  RobotControl1_M->solverInfoPtr = (&RobotControl1_M->solverInfo);

  /* Initialize timing info */
  {
    int_T *mdlTsMap = RobotControl1_M->Timing.sampleTimeTaskIDArray;
    mdlTsMap[0] = 0;
    mdlTsMap[1] = 1;
    RobotControl1_M->Timing.sampleTimeTaskIDPtr = (&mdlTsMap[0]);
    RobotControl1_M->Timing.sampleTimes =
      (&RobotControl1_M->Timing.sampleTimesArray[0]);
    RobotControl1_M->Timing.offsetTimes =
      (&RobotControl1_M->Timing.offsetTimesArray[0]);

    /* task periods */
    RobotControl1_M->Timing.sampleTimes[0] = (0.0);
    RobotControl1_M->Timing.sampleTimes[1] = (0.13333333333333333);

    /* task offsets */
    RobotControl1_M->Timing.offsetTimes[0] = (0.0);
    RobotControl1_M->Timing.offsetTimes[1] = (0.0);
  }

  rtmSetTPtr(RobotControl1_M, &RobotControl1_M->Timing.tArray[0]);

  {
    int_T *mdlSampleHits = RobotControl1_M->Timing.sampleHitArray;
    mdlSampleHits[0] = 1;
    mdlSampleHits[1] = 1;
    RobotControl1_M->Timing.sampleHits = (&mdlSampleHits[0]);
  }

  rtmSetTFinal(RobotControl1_M, -1);
  RobotControl1_M->Timing.stepSize0 = 0.13333333333333333;
  RobotControl1_M->Timing.stepSize1 = 0.13333333333333333;

  /* Setup for data logging */
  {
    static RTWLogInfo rt_DataLoggingInfo;
    RobotControl1_M->rtwLogInfo = &rt_DataLoggingInfo;
  }

  /* Setup for data logging */
  {
    rtliSetLogXSignalInfo(RobotControl1_M->rtwLogInfo, (NULL));
    rtliSetLogXSignalPtrs(RobotControl1_M->rtwLogInfo, (NULL));
    rtliSetLogT(RobotControl1_M->rtwLogInfo, "tout");
    rtliSetLogX(RobotControl1_M->rtwLogInfo, "");
    rtliSetLogXFinal(RobotControl1_M->rtwLogInfo, "");
    rtliSetLogVarNameModifier(RobotControl1_M->rtwLogInfo, "rt_");
    rtliSetLogFormat(RobotControl1_M->rtwLogInfo, 0);
    rtliSetLogMaxRows(RobotControl1_M->rtwLogInfo, 1000);
    rtliSetLogDecimation(RobotControl1_M->rtwLogInfo, 1);
    rtliSetLogY(RobotControl1_M->rtwLogInfo, "");
    rtliSetLogYSignalInfo(RobotControl1_M->rtwLogInfo, (NULL));
    rtliSetLogYSignalPtrs(RobotControl1_M->rtwLogInfo, (NULL));
  }

  RobotControl1_M->solverInfoPtr = (&RobotControl1_M->solverInfo);
  RobotControl1_M->Timing.stepSize = (0.13333333333333333);
  rtsiSetFixedStepSize(&RobotControl1_M->solverInfo, 0.13333333333333333);
  rtsiSetSolverMode(&RobotControl1_M->solverInfo, SOLVER_MODE_SINGLETASKING);

  /* block I/O */
  RobotControl1_M->ModelData.blockIO = ((void *) &RobotControl1_B);
  (void) memset(((void *) &RobotControl1_B), 0,
                sizeof(B_RobotControl1_T));

  /* parameters */
  RobotControl1_M->ModelData.defaultParam = ((real_T *)&RobotControl1_P);

  /* states (dwork) */
  RobotControl1_M->ModelData.dwork = ((void *) &RobotControl1_DW);
  (void) memset((void *)&RobotControl1_DW, 0,
                sizeof(DW_RobotControl1_T));

  /* Initialize DataMapInfo substructure containing ModelMap for C API */
  RobotControl1_InitializeDataMapInfo(RobotControl1_M);

  /* child S-Function registration */
  {
    RTWSfcnInfo *sfcnInfo = &RobotControl1_M->NonInlinedSFcns.sfcnInfo;
    RobotControl1_M->sfcnInfo = (sfcnInfo);
    rtssSetErrorStatusPtr(sfcnInfo, (&rtmGetErrorStatus(RobotControl1_M)));
    rtssSetNumRootSampTimesPtr(sfcnInfo, &RobotControl1_M->Sizes.numSampTimes);
    RobotControl1_M->NonInlinedSFcns.taskTimePtrs[0] = &(rtmGetTPtr
      (RobotControl1_M)[0]);
    RobotControl1_M->NonInlinedSFcns.taskTimePtrs[1] = &(rtmGetTPtr
      (RobotControl1_M)[1]);
    rtssSetTPtrPtr(sfcnInfo,RobotControl1_M->NonInlinedSFcns.taskTimePtrs);
    rtssSetTStartPtr(sfcnInfo, &rtmGetTStart(RobotControl1_M));
    rtssSetTFinalPtr(sfcnInfo, &rtmGetTFinal(RobotControl1_M));
    rtssSetTimeOfLastOutputPtr(sfcnInfo, &rtmGetTimeOfLastOutput(RobotControl1_M));
    rtssSetStepSizePtr(sfcnInfo, &RobotControl1_M->Timing.stepSize);
    rtssSetStopRequestedPtr(sfcnInfo, &rtmGetStopRequested(RobotControl1_M));
    rtssSetDerivCacheNeedsResetPtr(sfcnInfo,
      &RobotControl1_M->ModelData.derivCacheNeedsReset);
    rtssSetZCCacheNeedsResetPtr(sfcnInfo,
      &RobotControl1_M->ModelData.zCCacheNeedsReset);
    rtssSetBlkStateChangePtr(sfcnInfo,
      &RobotControl1_M->ModelData.blkStateChange);
    rtssSetSampleHitsPtr(sfcnInfo, &RobotControl1_M->Timing.sampleHits);
    rtssSetPerTaskSampleHitsPtr(sfcnInfo,
      &RobotControl1_M->Timing.perTaskSampleHits);
    rtssSetSimModePtr(sfcnInfo, &RobotControl1_M->simMode);
    rtssSetSolverInfoPtr(sfcnInfo, &RobotControl1_M->solverInfoPtr);
  }

  RobotControl1_M->Sizes.numSFcns = (3);

  /* register each child */
  {
    (void) memset((void *)&RobotControl1_M->NonInlinedSFcns.childSFunctions[0],
                  0,
                  3*sizeof(SimStruct));
    RobotControl1_M->childSfunctions =
      (&RobotControl1_M->NonInlinedSFcns.childSFunctionPtrs[0]);
    RobotControl1_M->childSfunctions[0] =
      (&RobotControl1_M->NonInlinedSFcns.childSFunctions[0]);
    RobotControl1_M->childSfunctions[1] =
      (&RobotControl1_M->NonInlinedSFcns.childSFunctions[1]);
    RobotControl1_M->childSfunctions[2] =
      (&RobotControl1_M->NonInlinedSFcns.childSFunctions[2]);

    /* Level2 S-Function Block: RobotControl1/<S30>/RS232 Binary Send1 (rs232bsend) */
    {
      SimStruct *rts = RobotControl1_M->childSfunctions[0];

      /* timing info */
      time_T *sfcnPeriod = RobotControl1_M->NonInlinedSFcns.Sfcn0.sfcnPeriod;
      time_T *sfcnOffset = RobotControl1_M->NonInlinedSFcns.Sfcn0.sfcnOffset;
      int_T *sfcnTsMap = RobotControl1_M->NonInlinedSFcns.Sfcn0.sfcnTsMap;
      (void) memset((void*)sfcnPeriod, 0,
                    sizeof(time_T)*1);
      (void) memset((void*)sfcnOffset, 0,
                    sizeof(time_T)*1);
      ssSetSampleTimePtr(rts, &sfcnPeriod[0]);
      ssSetOffsetTimePtr(rts, &sfcnOffset[0]);
      ssSetSampleTimeTaskIDPtr(rts, sfcnTsMap);

      /* Set up the mdlInfo pointer */
      {
        ssSetBlkInfo2Ptr(rts, &RobotControl1_M->NonInlinedSFcns.blkInfo2[0]);
      }

      ssSetRTWSfcnInfo(rts, RobotControl1_M->sfcnInfo);

      /* Allocate memory of model methods 2 */
      {
        ssSetModelMethods2(rts, &RobotControl1_M->NonInlinedSFcns.methods2[0]);
      }

      /* Allocate memory of model methods 3 */
      {
        ssSetModelMethods3(rts, &RobotControl1_M->NonInlinedSFcns.methods3[0]);
      }

      /* Allocate memory for states auxilliary information */
      {
        ssSetStatesInfo2(rts, &RobotControl1_M->NonInlinedSFcns.statesInfo2[0]);
      }

      /* inputs */
      {
        _ssSetNumInputPorts(rts, 1);
        ssSetPortInfoForInputs(rts,
          &RobotControl1_M->NonInlinedSFcns.Sfcn0.inputPortInfo[0]);

        /* port 0 */
        {
          ssSetInputPortRequiredContiguous(rts, 0, 1);
          ssSetInputPortSignal(rts, 0, RobotControl1_B.CharArray);
          _ssSetInputPortNumDimensions(rts, 0, 1);
          ssSetInputPortWidth(rts, 0, 22);
        }
      }

      /* path info */
      ssSetModelName(rts, "RS232\nBinary Send1");
      ssSetPath(rts,
                "RobotControl1/Open-loop control/plant/Send Serial Packet to Servo Controller And recieve status message from SSC/RS232 Binary Send1");
      ssSetRTModel(rts,RobotControl1_M);
      ssSetParentSS(rts, (NULL));
      ssSetRootSS(rts, rts);
      ssSetVersion(rts, SIMSTRUCT_VERSION_LEVEL2);

      /* parameters */
      {
        mxArray **sfcnParams = (mxArray **)
          &RobotControl1_M->NonInlinedSFcns.Sfcn0.params;
        ssSetSFcnParamsCount(rts, 3);
        ssSetSFcnParamsPtr(rts, &sfcnParams[0]);
        ssSetSFcnParam(rts, 0, (mxArray*)
                       RobotControl1_P.RS232BinarySend1_P1_Size);
        ssSetSFcnParam(rts, 1, (mxArray*)
                       RobotControl1_P.RS232BinarySend1_P2_Size);
        ssSetSFcnParam(rts, 2, (mxArray*)
                       RobotControl1_P.RS232BinarySend1_P3_Size);
      }

      /* registration */
      rs232bsend(rts);
      sfcnInitializeSizes(rts);
      sfcnInitializeSampleTimes(rts);

      /* adjust sample time */
      ssSetSampleTime(rts, 0, 0.13333333333333333);
      ssSetOffsetTime(rts, 0, 0.0);
      sfcnTsMap[0] = 1;

      /* set compiled values of dynamic vector attributes */
      ssSetNumNonsampledZCs(rts, 0);

      /* Update connectivity flags for each port */
      _ssSetInputPortConnected(rts, 0, 1);

      /* Update the BufferDstPort flags for each input port */
      ssSetInputPortBufferDstPort(rts, 0, -1);
    }

    /* Level2 S-Function Block: RobotControl1/<Root>/RS232 Binary Receive1 (rs232brec) */
    {
      SimStruct *rts = RobotControl1_M->childSfunctions[1];

      /* timing info */
      time_T *sfcnPeriod = RobotControl1_M->NonInlinedSFcns.Sfcn1.sfcnPeriod;
      time_T *sfcnOffset = RobotControl1_M->NonInlinedSFcns.Sfcn1.sfcnOffset;
      int_T *sfcnTsMap = RobotControl1_M->NonInlinedSFcns.Sfcn1.sfcnTsMap;
      (void) memset((void*)sfcnPeriod, 0,
                    sizeof(time_T)*1);
      (void) memset((void*)sfcnOffset, 0,
                    sizeof(time_T)*1);
      ssSetSampleTimePtr(rts, &sfcnPeriod[0]);
      ssSetOffsetTimePtr(rts, &sfcnOffset[0]);
      ssSetSampleTimeTaskIDPtr(rts, sfcnTsMap);

      /* Set up the mdlInfo pointer */
      {
        ssSetBlkInfo2Ptr(rts, &RobotControl1_M->NonInlinedSFcns.blkInfo2[1]);
      }

      ssSetRTWSfcnInfo(rts, RobotControl1_M->sfcnInfo);

      /* Allocate memory of model methods 2 */
      {
        ssSetModelMethods2(rts, &RobotControl1_M->NonInlinedSFcns.methods2[1]);
      }

      /* Allocate memory of model methods 3 */
      {
        ssSetModelMethods3(rts, &RobotControl1_M->NonInlinedSFcns.methods3[1]);
      }

      /* Allocate memory for states auxilliary information */
      {
        ssSetStatesInfo2(rts, &RobotControl1_M->NonInlinedSFcns.statesInfo2[1]);
      }

      /* inputs */
      {
        _ssSetNumInputPorts(rts, 2);
        ssSetPortInfoForInputs(rts,
          &RobotControl1_M->NonInlinedSFcns.Sfcn1.inputPortInfo[0]);

        /* port 0 */
        {
          ssSetInputPortRequiredContiguous(rts, 0, 1);
          ssSetInputPortSignal(rts, 0, &RobotControl1_B.packetsize);
          _ssSetInputPortNumDimensions(rts, 0, 1);
          ssSetInputPortWidth(rts, 0, 1);
        }

        /* port 1 */
        {
          ssSetInputPortRequiredContiguous(rts, 1, 1);
          ssSetInputPortSignal(rts, 1, &RobotControl1_B.packetsize4);
          _ssSetInputPortNumDimensions(rts, 1, 1);
          ssSetInputPortWidth(rts, 1, 1);
        }
      }

      /* outputs */
      {
        ssSetPortInfoForOutputs(rts,
          &RobotControl1_M->NonInlinedSFcns.Sfcn1.outputPortInfo[0]);
        _ssSetNumOutputPorts(rts, 2);

        /* port 0 */
        {
          _ssSetOutputPortNumDimensions(rts, 0, 1);
          ssSetOutputPortWidth(rts, 0, 1);
          ssSetOutputPortSignal(rts, 0, (NULL));
        }

        /* port 1 */
        {
          _ssSetOutputPortNumDimensions(rts, 1, 1);
          ssSetOutputPortWidth(rts, 1, 8);
          ssSetOutputPortSignal(rts, 1, ((uint8_T *)
            RobotControl1_B.RS232BinaryReceive1_o2));
        }
      }

      /* path info */
      ssSetModelName(rts, "RS232\nBinary Receive1");
      ssSetPath(rts, "RobotControl1/RS232 Binary Receive1");
      ssSetRTModel(rts,RobotControl1_M);
      ssSetParentSS(rts, (NULL));
      ssSetRootSS(rts, rts);
      ssSetVersion(rts, SIMSTRUCT_VERSION_LEVEL2);

      /* parameters */
      {
        mxArray **sfcnParams = (mxArray **)
          &RobotControl1_M->NonInlinedSFcns.Sfcn1.params;
        ssSetSFcnParamsCount(rts, 3);
        ssSetSFcnParamsPtr(rts, &sfcnParams[0]);
        ssSetSFcnParam(rts, 0, (mxArray*)
                       RobotControl1_P.RS232BinaryReceive1_P1_Size);
        ssSetSFcnParam(rts, 1, (mxArray*)
                       RobotControl1_P.RS232BinaryReceive1_P2_Size);
        ssSetSFcnParam(rts, 2, (mxArray*)
                       RobotControl1_P.RS232BinaryReceive1_P3_Size);
      }

      /* work vectors */
      ssSetIWork(rts, (int_T *) &RobotControl1_DW.RS232BinaryReceive1_IWORK[0]);

      {
        struct _ssDWorkRecord *dWorkRecord = (struct _ssDWorkRecord *)
          &RobotControl1_M->NonInlinedSFcns.Sfcn1.dWork;
        struct _ssDWorkAuxRecord *dWorkAuxRecord = (struct _ssDWorkAuxRecord *)
          &RobotControl1_M->NonInlinedSFcns.Sfcn1.dWorkAux;
        ssSetSFcnDWork(rts, dWorkRecord);
        ssSetSFcnDWorkAux(rts, dWorkAuxRecord);
        _ssSetNumDWork(rts, 2);

        /* DWORK1 */
        ssSetDWorkWidth(rts, 0, 8);
        ssSetDWorkDataType(rts, 0,SS_UINT8);
        ssSetDWorkComplexSignal(rts, 0, 0);
        ssSetDWork(rts, 0, &RobotControl1_DW.RS232BinaryReceive1_DWORK1[0]);

        /* IWORK */
        ssSetDWorkWidth(rts, 1, 2);
        ssSetDWorkDataType(rts, 1,SS_INTEGER);
        ssSetDWorkComplexSignal(rts, 1, 0);
        ssSetDWork(rts, 1, &RobotControl1_DW.RS232BinaryReceive1_IWORK[0]);
      }

      /* register function-calls */
      {
        int_T *callSysOutputs = (int_T *)
          &RobotControl1_M->NonInlinedSFcns.Sfcn1.callSysOutputs;
        void **callSysArgs1 = (void **)
          &RobotControl1_M->NonInlinedSFcns.Sfcn1.callSysArgs1;
        int_T *callSysArgs2 = (int_T *)
          &RobotControl1_M->NonInlinedSFcns.Sfcn1.callSysArgs2;
        SysOutputFcn *callSysFcns = (SysOutputFcn *)
          &RobotControl1_M->NonInlinedSFcns.Sfcn1.callSysFcns;

        {
          int32_T i;
          for (i = 0; i < 1; i++) {
            callSysOutputs[i] = 0;
            callSysFcns[i] = (SysOutputFcn) (NULL);
            callSysFcns[1+i] = (SysOutputFcn) (NULL);
            callSysFcns[2*1+i] = (SysOutputFcn) (NULL);
            callSysFcns[3*1+i] = (SysOutputFcn) (NULL);
          }
        }

        ssSetCallSystemOutputPtr(rts, &callSysOutputs[0]);
        ssSetCallSystemOutputArg1List(rts, &callSysArgs1[0]);
        ssSetCallSystemOutputArg2List(rts, &callSysArgs2[0]);
        ssSetCallSystemOutputFcnList(rts, &callSysFcns[0]);
        callSysArgs1[0] = (void *)RobotControl1_M;
        callSysArgs2[0] = 0;
        callSysFcns[0] = (SysOutputFcn) RobotControl1_StatusExtractionFNI;
        callSysFcns[1+0] = (SysOutputFcn) RobotControl1_StatusExtraction_InitFNI;
        callSysFcns[2+0] = (SysOutputFcn) (NULL);
        callSysFcns[3+0] = (SysOutputFcn) (NULL);
        callSysOutputs[0] = 1;
      }

      /* registration */
      rs232brec(rts);
      sfcnInitializeSizes(rts);
      sfcnInitializeSampleTimes(rts);

      /* adjust sample time */
      ssSetSampleTime(rts, 0, 0.13333333333333333);
      ssSetOffsetTime(rts, 0, 0.0);
      sfcnTsMap[0] = 1;

      /* set compiled values of dynamic vector attributes */
      ssSetOutputPortWidth(rts, 0, 1);
      ssSetNumNonsampledZCs(rts, 0);

      /* Update connectivity flags for each port */
      _ssSetInputPortConnected(rts, 0, 1);
      _ssSetInputPortConnected(rts, 1, 1);
      _ssSetOutputPortConnected(rts, 0, 1);
      _ssSetOutputPortConnected(rts, 1, 1);
      _ssSetOutputPortBeingMerged(rts, 0, 0);
      _ssSetOutputPortBeingMerged(rts, 1, 0);

      /* Update the BufferDstPort flags for each input port */
      ssSetInputPortBufferDstPort(rts, 0, -1);
      ssSetInputPortBufferDstPort(rts, 1, -1);
    }

    /* Level2 S-Function Block: RobotControl1/<Root>/RS232 Setup  (rs232setup) */
    {
      SimStruct *rts = RobotControl1_M->childSfunctions[2];

      /* timing info */
      time_T *sfcnPeriod = RobotControl1_M->NonInlinedSFcns.Sfcn2.sfcnPeriod;
      time_T *sfcnOffset = RobotControl1_M->NonInlinedSFcns.Sfcn2.sfcnOffset;
      int_T *sfcnTsMap = RobotControl1_M->NonInlinedSFcns.Sfcn2.sfcnTsMap;
      (void) memset((void*)sfcnPeriod, 0,
                    sizeof(time_T)*1);
      (void) memset((void*)sfcnOffset, 0,
                    sizeof(time_T)*1);
      ssSetSampleTimePtr(rts, &sfcnPeriod[0]);
      ssSetOffsetTimePtr(rts, &sfcnOffset[0]);
      ssSetSampleTimeTaskIDPtr(rts, sfcnTsMap);

      /* Set up the mdlInfo pointer */
      {
        ssSetBlkInfo2Ptr(rts, &RobotControl1_M->NonInlinedSFcns.blkInfo2[2]);
      }

      ssSetRTWSfcnInfo(rts, RobotControl1_M->sfcnInfo);

      /* Allocate memory of model methods 2 */
      {
        ssSetModelMethods2(rts, &RobotControl1_M->NonInlinedSFcns.methods2[2]);
      }

      /* Allocate memory of model methods 3 */
      {
        ssSetModelMethods3(rts, &RobotControl1_M->NonInlinedSFcns.methods3[2]);
      }

      /* Allocate memory for states auxilliary information */
      {
        ssSetStatesInfo2(rts, &RobotControl1_M->NonInlinedSFcns.statesInfo2[2]);
      }

      /* path info */
      ssSetModelName(rts, "RS232 Setup ");
      ssSetPath(rts, "RobotControl1/RS232 Setup ");
      ssSetRTModel(rts,RobotControl1_M);
      ssSetParentSS(rts, (NULL));
      ssSetRootSS(rts, rts);
      ssSetVersion(rts, SIMSTRUCT_VERSION_LEVEL2);

      /* parameters */
      {
        mxArray **sfcnParams = (mxArray **)
          &RobotControl1_M->NonInlinedSFcns.Sfcn2.params;
        ssSetSFcnParamsCount(rts, 19);
        ssSetSFcnParamsPtr(rts, &sfcnParams[0]);
        ssSetSFcnParam(rts, 0, (mxArray*)RobotControl1_P.RS232Setup_P1_Size);
        ssSetSFcnParam(rts, 1, (mxArray*)RobotControl1_P.RS232Setup_P2_Size);
        ssSetSFcnParam(rts, 2, (mxArray*)RobotControl1_P.RS232Setup_P3_Size);
        ssSetSFcnParam(rts, 3, (mxArray*)RobotControl1_P.RS232Setup_P4_Size);
        ssSetSFcnParam(rts, 4, (mxArray*)RobotControl1_P.RS232Setup_P5_Size);
        ssSetSFcnParam(rts, 5, (mxArray*)RobotControl1_P.RS232Setup_P6_Size);
        ssSetSFcnParam(rts, 6, (mxArray*)RobotControl1_P.RS232Setup_P7_Size);
        ssSetSFcnParam(rts, 7, (mxArray*)RobotControl1_P.RS232Setup_P8_Size);
        ssSetSFcnParam(rts, 8, (mxArray*)RobotControl1_P.RS232Setup_P9_Size);
        ssSetSFcnParam(rts, 9, (mxArray*)RobotControl1_P.RS232Setup_P10_Size);
        ssSetSFcnParam(rts, 10, (mxArray*)RobotControl1_P.RS232Setup_P11_Size);
        ssSetSFcnParam(rts, 11, (mxArray*)RobotControl1_P.RS232Setup_P12_Size);
        ssSetSFcnParam(rts, 12, (mxArray*)RobotControl1_P.RS232Setup_P13_Size);
        ssSetSFcnParam(rts, 13, (mxArray*)RobotControl1_P.RS232Setup_P14_Size);
        ssSetSFcnParam(rts, 14, (mxArray*)RobotControl1_P.RS232Setup_P15_Size);
        ssSetSFcnParam(rts, 15, (mxArray*)RobotControl1_P.RS232Setup_P16_Size);
        ssSetSFcnParam(rts, 16, (mxArray*)RobotControl1_P.RS232Setup_P17_Size);
        ssSetSFcnParam(rts, 17, (mxArray*)RobotControl1_P.RS232Setup_P18_Size);
        ssSetSFcnParam(rts, 18, (mxArray*)RobotControl1_P.RS232Setup_P19_Size);
      }

      /* registration */
      rs232setup(rts);
      sfcnInitializeSizes(rts);
      sfcnInitializeSampleTimes(rts);

      /* adjust sample time */
      ssSetSampleTime(rts, 0, 0.13333333333333333);
      ssSetOffsetTime(rts, 0, 0.0);
      sfcnTsMap[0] = 1;

      /* set compiled values of dynamic vector attributes */
      ssSetNumNonsampledZCs(rts, 0);

      /* Update connectivity flags for each port */
      /* Update the BufferDstPort flags for each input port */
    }
  }

  /* Initialize Sizes */
  RobotControl1_M->Sizes.numContStates = (0);/* Number of continuous states */
  RobotControl1_M->Sizes.numY = (0);   /* Number of model outputs */
  RobotControl1_M->Sizes.numU = (0);   /* Number of model inputs */
  RobotControl1_M->Sizes.sysDirFeedThru = (0);/* The model is not direct feedthrough */
  RobotControl1_M->Sizes.numSampTimes = (2);/* Number of sample times */
  RobotControl1_M->Sizes.numBlocks = (48);/* Number of blocks */
  RobotControl1_M->Sizes.numBlockIO = (40);/* Number of block outputs */
  RobotControl1_M->Sizes.numBlockPrms = (81);/* Sum of parameter "widths" */
  return RobotControl1_M;
}

/*========================================================================*
 * End of Classic call interface                                          *
 *========================================================================*/
