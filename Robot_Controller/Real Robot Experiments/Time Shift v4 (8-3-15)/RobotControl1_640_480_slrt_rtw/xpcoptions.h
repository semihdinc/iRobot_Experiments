#ifndef __RobotControl1_640_480_XPCOPTIONS_H___
#define __RobotControl1_640_480_XPCOPTIONS_H___
#include "simstruc_types.h"
#ifndef MT
#define MT                             0                         /* MT may be undefined by simstruc_types.h */
#endif

#include "RobotControl1_640_480.h"
#define XPCMDSSUPPORT                  0
#define MDSTASKSNUM                    0
#define FULLMULTITHREAD                0
#define SIZEOF_PARAMS                  (-1 * (int)sizeof(P_RobotControl1_640_480_T))
#define SIMMODE                        0
#define LOGTET                         1
#define LOGBUFSIZE                     100000
#define PROFILINGFLAG                  0
#define EVENTNUMBER                    5000
#define IRQ_NO                         0
#define IO_IRQ                         0
#define WWW_ACCESS_LEVEL               0
#define CPUCLOCK                       0
#define MAXOVERLOAD                    0
#define MAXOVERLOADLEN                 0
#define XPCMODELSTACKSIZEKB            512
#define XPCSTARTUPFLAG                 1
#define PTLOADPARAMFLAG                0
#define DOUBLEBUFFERING                0

/* Change all stepsize using the newBaseRateStepSize */
void RobotControl1_640_480_ChangeStepSize(real_T newBaseRateStepSize,
  RT_MODEL_RobotControl1_640_480_T *const RobotControl1_640_480_M)
{
  real_T ratio = newBaseRateStepSize / 0.033333333333333333;

  /* update non-zore stepsize of periodic
   * sample time. Stepsize of asynchronous
   * sample time is not changed in this function */
  RobotControl1_640_480_M->Timing.stepSize0 =
    RobotControl1_640_480_M->Timing.stepSize0 * ratio;
  RobotControl1_640_480_M->Timing.stepSize1 =
    RobotControl1_640_480_M->Timing.stepSize1 * ratio;
  RobotControl1_640_480_M->Timing.stepSize =
    RobotControl1_640_480_M->Timing.stepSize * ratio;
}

void XPCCALLCONV changeStepSize(real_T stepSize)
{
  /* Change all stepsize using the newBaseRateStepSize */
  RobotControl1_640_480_ChangeStepSize(stepSize, RobotControl1_640_480_M);
}

#endif                                 /* __RobotControl1_640_480_XPCOPTIONS_H___ */
