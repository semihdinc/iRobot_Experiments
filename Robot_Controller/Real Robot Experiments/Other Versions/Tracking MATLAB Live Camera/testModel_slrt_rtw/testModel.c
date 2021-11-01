/*
 * testModel.c
 *
 * Academic License - for use in teaching, academic research, and meeting
 * course requirements at degree granting institutions only.  Not for
 * government, commercial, or other organizational use.
 *
 * Code generation for model "testModel".
 *
 * Model version              : 1.36
 * Simulink Coder version : 8.8 (R2015a) 09-Feb-2015
 * C source code generated on : Thu Sep 17 15:16:28 2015
 *
 * Target selection: slrt.tlc
 * Note: GRT includes extra infrastructure and instrumentation for prototyping
 * Embedded hardware selection: Generic->32-bit x86 compatible
 * Code generation objective: Execution efficiency
 * Validation result: Not run
 */

#include "rt_logging_mmi.h"
#include "testModel_capi.h"
#include "testModel.h"
#include "testModel_private.h"

/* Block signals (auto storage) */
B_testModel_T testModel_B;

/* Block states (auto storage) */
DW_testModel_T testModel_DW;

/* Real-time model */
RT_MODEL_testModel_T testModel_M_;
RT_MODEL_testModel_T *const testModel_M = &testModel_M_;

/* Model output function */
static void testModel_output(void)
{
  /* ok to acquire for <S2>/S-Function */
  testModel_DW.SFunction_IWORK.AcquireOK = 1;

  /* FromFile: '<Root>/From File' */
  {
    const real_T *pT = (const real_T *) testModel_DW.FromFile_PWORK.PrevTimePtr;

    {
      int_T i1;
      real_T *y0 = &testModel_B.FromFile[0];
      for (i1=0; i1 < 7; i1++) {
        y0[i1] = pT[i1 + 1];
      }
    }
  }
}

/* Model update function */
static void testModel_update(void)
{
  /* Update absolute time for base rate */
  /* The "clockTick0" counts the number of times the code of this task has
   * been executed. The absolute time is the multiplication of "clockTick0"
   * and "Timing.stepSize0". Size of "clockTick0" ensures timer will not
   * overflow during the application lifespan selected.
   * Timer of this task consists of two 32 bit unsigned integers.
   * The two integers represent the low bits Timing.clockTick0 and the high bits
   * Timing.clockTickH0. When the low bit overflows to 0, the high bits increment.
   */
  if (!(++testModel_M->Timing.clockTick0)) {
    ++testModel_M->Timing.clockTickH0;
  }

  testModel_M->Timing.t[0] = testModel_M->Timing.clockTick0 *
    testModel_M->Timing.stepSize0 + testModel_M->Timing.clockTickH0 *
    testModel_M->Timing.stepSize0 * 4294967296.0;
}

/* Model initialize function */
static void testModel_initialize(void)
{
  /* S-Function Block: <S2>/S-Function (scblock) */
  {
    int i;
    if ((i = rl32eScopeExists(2)) == 0) {
      if ((i = rl32eDefScope(2,2)) != 0) {
        printf("Error creating scope 2\n");
      } else {
        rl32eAddSignal(2, rl32eGetSignalNo("From File/s1"));
        rl32eAddSignal(2, rl32eGetSignalNo("From File/s2"));
        rl32eAddSignal(2, rl32eGetSignalNo("From File/s3"));
        rl32eAddSignal(2, rl32eGetSignalNo("From File/s4"));
        rl32eAddSignal(2, rl32eGetSignalNo("From File/s5"));
        rl32eAddSignal(2, rl32eGetSignalNo("From File/s6"));
        rl32eAddSignal(2, rl32eGetSignalNo("From File/s7"));
        rl32eSetScope(2, 4, 20);
        rl32eSetScope(2, 5, 0);
        rl32eSetScope(2, 6, 1);
        rl32eSetScope(2, 0, 0);
        rl32eSetScope(2, 3, rl32eGetSignalNo("From File/s1"));
        rl32eSetScope(2, 1, 0.0);
        rl32eSetScope(2, 2, 0);
        rl32eSetScope(2, 9, 0);
        rl32eSetTargetScope(2, 1, 3.0);
        rl32eSetTargetScope(2, 11, 0.0);
        rl32eSetTargetScope(2, 10, 0.0);
        xpceScopeAcqOK(2, &testModel_DW.SFunction_IWORK.AcquireOK);
      }
    }

    if (i) {
      rl32eRestartAcquisition(2);
    }
  }

  /* Start for FromFile: '<Root>/From File' */
  {
    static const real_T tuData[8] = { 660.43986820428336, 857.6,
      752.12200435729847, 746.01579778830967, 434.840197693575, 425.746456692913,
      412.409586056645, 268.717219589258 } ;

    testModel_DW.FromFile_PWORK.PrevTimePtr = (void *) &tuData[0];
  }
}

/* Model terminate function */
static void testModel_terminate(void)
{
  /* (no terminate code required) */
}

/*========================================================================*
 * Start of Classic call interface                                        *
 *========================================================================*/
void MdlOutputs(int_T tid)
{
  testModel_output();
  UNUSED_PARAMETER(tid);
}

void MdlUpdate(int_T tid)
{
  testModel_update();
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
  testModel_initialize();
}

void MdlTerminate(void)
{
  testModel_terminate();
}

/* Registration function */
RT_MODEL_testModel_T *testModel(void)
{
  /* Registration code */

  /* initialize non-finites */
  rt_InitInfAndNaN(sizeof(real_T));

  /* initialize real-time model */
  (void) memset((void *)testModel_M, 0,
                sizeof(RT_MODEL_testModel_T));

  /* Initialize timing info */
  {
    int_T *mdlTsMap = testModel_M->Timing.sampleTimeTaskIDArray;
    mdlTsMap[0] = 0;
    testModel_M->Timing.sampleTimeTaskIDPtr = (&mdlTsMap[0]);
    testModel_M->Timing.sampleTimes = (&testModel_M->Timing.sampleTimesArray[0]);
    testModel_M->Timing.offsetTimes = (&testModel_M->Timing.offsetTimesArray[0]);

    /* task periods */
    testModel_M->Timing.sampleTimes[0] = (1.0);

    /* task offsets */
    testModel_M->Timing.offsetTimes[0] = (0.0);
  }

  rtmSetTPtr(testModel_M, &testModel_M->Timing.tArray[0]);

  {
    int_T *mdlSampleHits = testModel_M->Timing.sampleHitArray;
    mdlSampleHits[0] = 1;
    testModel_M->Timing.sampleHits = (&mdlSampleHits[0]);
  }

  rtmSetTFinal(testModel_M, 10.0);
  testModel_M->Timing.stepSize0 = 1.0;

  /* Setup for data logging */
  {
    static RTWLogInfo rt_DataLoggingInfo;
    testModel_M->rtwLogInfo = &rt_DataLoggingInfo;
  }

  /* Setup for data logging */
  {
    rtliSetLogXSignalInfo(testModel_M->rtwLogInfo, (NULL));
    rtliSetLogXSignalPtrs(testModel_M->rtwLogInfo, (NULL));
    rtliSetLogT(testModel_M->rtwLogInfo, "tout");
    rtliSetLogX(testModel_M->rtwLogInfo, "");
    rtliSetLogXFinal(testModel_M->rtwLogInfo, "");
    rtliSetLogVarNameModifier(testModel_M->rtwLogInfo, "rt_");
    rtliSetLogFormat(testModel_M->rtwLogInfo, 0);
    rtliSetLogMaxRows(testModel_M->rtwLogInfo, 1000);
    rtliSetLogDecimation(testModel_M->rtwLogInfo, 1);
    rtliSetLogY(testModel_M->rtwLogInfo, "");
    rtliSetLogYSignalInfo(testModel_M->rtwLogInfo, (NULL));
    rtliSetLogYSignalPtrs(testModel_M->rtwLogInfo, (NULL));
  }

  testModel_M->solverInfoPtr = (&testModel_M->solverInfo);
  testModel_M->Timing.stepSize = (1.0);
  rtsiSetFixedStepSize(&testModel_M->solverInfo, 1.0);
  rtsiSetSolverMode(&testModel_M->solverInfo, SOLVER_MODE_SINGLETASKING);

  /* block I/O */
  testModel_M->ModelData.blockIO = ((void *) &testModel_B);
  (void) memset(((void *) &testModel_B), 0,
                sizeof(B_testModel_T));

  /* states (dwork) */
  testModel_M->ModelData.dwork = ((void *) &testModel_DW);
  (void) memset((void *)&testModel_DW, 0,
                sizeof(DW_testModel_T));

  /* Initialize DataMapInfo substructure containing ModelMap for C API */
  testModel_InitializeDataMapInfo(testModel_M);

  /* Initialize Sizes */
  testModel_M->Sizes.numContStates = (0);/* Number of continuous states */
  testModel_M->Sizes.numY = (0);       /* Number of model outputs */
  testModel_M->Sizes.numU = (0);       /* Number of model inputs */
  testModel_M->Sizes.sysDirFeedThru = (0);/* The model is not direct feedthrough */
  testModel_M->Sizes.numSampTimes = (1);/* Number of sample times */
  testModel_M->Sizes.numBlocks = (2);  /* Number of blocks */
  testModel_M->Sizes.numBlockIO = (1); /* Number of block outputs */
  return testModel_M;
}

/*========================================================================*
 * End of Classic call interface                                          *
 *========================================================================*/
