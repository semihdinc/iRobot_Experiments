/*
 * RobotControl1.h
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

#ifndef RTW_HEADER_RobotControl1_h_
#define RTW_HEADER_RobotControl1_h_
#include <string.h>
#include <stddef.h>
#include <math.h>
#include "rtw_modelmap.h"
#ifndef RobotControl1_COMMON_INCLUDES_
# define RobotControl1_COMMON_INCLUDES_
#include <xpcimports.h>
#include <xpcdatatypes.h>
#include "rtwtypes.h"
#include "zero_crossing_types.h"
#include "simstruc.h"
#include "fixedpoint.h"
#include "rt_logging.h"
#endif                                 /* RobotControl1_COMMON_INCLUDES_ */

#include "RobotControl1_types.h"

/* Shared type includes */
#include "multiword_types.h"
#include "rt_zcfcn.h"
#include "rt_defines.h"
#include "rt_sfcn_helper.h"
#include "rt_nonfinite.h"

/* Macros for accessing real-time model data structure */
#ifndef rtmGetBlkStateChangeFlag
# define rtmGetBlkStateChangeFlag(rtm) ((rtm)->ModelData.blkStateChange)
#endif

#ifndef rtmSetBlkStateChangeFlag
# define rtmSetBlkStateChangeFlag(rtm, val) ((rtm)->ModelData.blkStateChange = (val))
#endif

#ifndef rtmGetBlockIO
# define rtmGetBlockIO(rtm)            ((rtm)->ModelData.blockIO)
#endif

#ifndef rtmSetBlockIO
# define rtmSetBlockIO(rtm, val)       ((rtm)->ModelData.blockIO = (val))
#endif

#ifndef rtmGetChecksums
# define rtmGetChecksums(rtm)          ((rtm)->Sizes.checksums)
#endif

#ifndef rtmSetChecksums
# define rtmSetChecksums(rtm, val)     ((rtm)->Sizes.checksums = (val))
#endif

#ifndef rtmGetConstBlockIO
# define rtmGetConstBlockIO(rtm)       ((rtm)->ModelData.constBlockIO)
#endif

#ifndef rtmSetConstBlockIO
# define rtmSetConstBlockIO(rtm, val)  ((rtm)->ModelData.constBlockIO = (val))
#endif

#ifndef rtmGetContStateDisabled
# define rtmGetContStateDisabled(rtm)  ((rtm)->ModelData.contStateDisabled)
#endif

#ifndef rtmSetContStateDisabled
# define rtmSetContStateDisabled(rtm, val) ((rtm)->ModelData.contStateDisabled = (val))
#endif

#ifndef rtmGetContStates
# define rtmGetContStates(rtm)         ((rtm)->ModelData.contStates)
#endif

#ifndef rtmSetContStates
# define rtmSetContStates(rtm, val)    ((rtm)->ModelData.contStates = (val))
#endif

#ifndef rtmGetDataMapInfo
# define rtmGetDataMapInfo(rtm)        ((rtm)->DataMapInfo)
#endif

#ifndef rtmSetDataMapInfo
# define rtmSetDataMapInfo(rtm, val)   ((rtm)->DataMapInfo = (val))
#endif

#ifndef rtmGetDefaultParam
# define rtmGetDefaultParam(rtm)       ((rtm)->ModelData.defaultParam)
#endif

#ifndef rtmSetDefaultParam
# define rtmSetDefaultParam(rtm, val)  ((rtm)->ModelData.defaultParam = (val))
#endif

#ifndef rtmGetDerivCacheNeedsReset
# define rtmGetDerivCacheNeedsReset(rtm) ((rtm)->ModelData.derivCacheNeedsReset)
#endif

#ifndef rtmSetDerivCacheNeedsReset
# define rtmSetDerivCacheNeedsReset(rtm, val) ((rtm)->ModelData.derivCacheNeedsReset = (val))
#endif

#ifndef rtmGetDirectFeedThrough
# define rtmGetDirectFeedThrough(rtm)  ((rtm)->Sizes.sysDirFeedThru)
#endif

#ifndef rtmSetDirectFeedThrough
# define rtmSetDirectFeedThrough(rtm, val) ((rtm)->Sizes.sysDirFeedThru = (val))
#endif

#ifndef rtmGetErrorStatusFlag
# define rtmGetErrorStatusFlag(rtm)    ((rtm)->errorStatus)
#endif

#ifndef rtmSetErrorStatusFlag
# define rtmSetErrorStatusFlag(rtm, val) ((rtm)->errorStatus = (val))
#endif

#ifndef rtmGetFinalTime
# define rtmGetFinalTime(rtm)          ((rtm)->Timing.tFinal)
#endif

#ifndef rtmSetFinalTime
# define rtmSetFinalTime(rtm, val)     ((rtm)->Timing.tFinal = (val))
#endif

#ifndef rtmGetFirstInitCondFlag
# define rtmGetFirstInitCondFlag(rtm)  ()
#endif

#ifndef rtmSetFirstInitCondFlag
# define rtmSetFirstInitCondFlag(rtm, val) ()
#endif

#ifndef rtmGetIntgData
# define rtmGetIntgData(rtm)           ()
#endif

#ifndef rtmSetIntgData
# define rtmSetIntgData(rtm, val)      ()
#endif

#ifndef rtmGetMdlRefGlobalTID
# define rtmGetMdlRefGlobalTID(rtm)    ()
#endif

#ifndef rtmSetMdlRefGlobalTID
# define rtmSetMdlRefGlobalTID(rtm, val) ()
#endif

#ifndef rtmGetMdlRefTriggerTID
# define rtmGetMdlRefTriggerTID(rtm)   ()
#endif

#ifndef rtmSetMdlRefTriggerTID
# define rtmSetMdlRefTriggerTID(rtm, val) ()
#endif

#ifndef rtmGetModelMappingInfo
# define rtmGetModelMappingInfo(rtm)   ((rtm)->SpecialInfo.mappingInfo)
#endif

#ifndef rtmSetModelMappingInfo
# define rtmSetModelMappingInfo(rtm, val) ((rtm)->SpecialInfo.mappingInfo = (val))
#endif

#ifndef rtmGetModelName
# define rtmGetModelName(rtm)          ((rtm)->modelName)
#endif

#ifndef rtmSetModelName
# define rtmSetModelName(rtm, val)     ((rtm)->modelName = (val))
#endif

#ifndef rtmGetNonInlinedSFcns
# define rtmGetNonInlinedSFcns(rtm)    ((rtm)->NonInlinedSFcns)
#endif

#ifndef rtmSetNonInlinedSFcns
# define rtmSetNonInlinedSFcns(rtm, val) ((rtm)->NonInlinedSFcns = (val))
#endif

#ifndef rtmGetNumBlockIO
# define rtmGetNumBlockIO(rtm)         ((rtm)->Sizes.numBlockIO)
#endif

#ifndef rtmSetNumBlockIO
# define rtmSetNumBlockIO(rtm, val)    ((rtm)->Sizes.numBlockIO = (val))
#endif

#ifndef rtmGetNumBlockParams
# define rtmGetNumBlockParams(rtm)     ((rtm)->Sizes.numBlockPrms)
#endif

#ifndef rtmSetNumBlockParams
# define rtmSetNumBlockParams(rtm, val) ((rtm)->Sizes.numBlockPrms = (val))
#endif

#ifndef rtmGetNumBlocks
# define rtmGetNumBlocks(rtm)          ((rtm)->Sizes.numBlocks)
#endif

#ifndef rtmSetNumBlocks
# define rtmSetNumBlocks(rtm, val)     ((rtm)->Sizes.numBlocks = (val))
#endif

#ifndef rtmGetNumContStates
# define rtmGetNumContStates(rtm)      ((rtm)->Sizes.numContStates)
#endif

#ifndef rtmSetNumContStates
# define rtmSetNumContStates(rtm, val) ((rtm)->Sizes.numContStates = (val))
#endif

#ifndef rtmGetNumDWork
# define rtmGetNumDWork(rtm)           ((rtm)->Sizes.numDwork)
#endif

#ifndef rtmSetNumDWork
# define rtmSetNumDWork(rtm, val)      ((rtm)->Sizes.numDwork = (val))
#endif

#ifndef rtmGetNumInputPorts
# define rtmGetNumInputPorts(rtm)      ((rtm)->Sizes.numIports)
#endif

#ifndef rtmSetNumInputPorts
# define rtmSetNumInputPorts(rtm, val) ((rtm)->Sizes.numIports = (val))
#endif

#ifndef rtmGetNumNonSampledZCs
# define rtmGetNumNonSampledZCs(rtm)   ((rtm)->Sizes.numNonSampZCs)
#endif

#ifndef rtmSetNumNonSampledZCs
# define rtmSetNumNonSampledZCs(rtm, val) ((rtm)->Sizes.numNonSampZCs = (val))
#endif

#ifndef rtmGetNumOutputPorts
# define rtmGetNumOutputPorts(rtm)     ((rtm)->Sizes.numOports)
#endif

#ifndef rtmSetNumOutputPorts
# define rtmSetNumOutputPorts(rtm, val) ((rtm)->Sizes.numOports = (val))
#endif

#ifndef rtmGetNumPeriodicContStates
# define rtmGetNumPeriodicContStates(rtm) ((rtm)->Sizes.numPeriodicContStates)
#endif

#ifndef rtmSetNumPeriodicContStates
# define rtmSetNumPeriodicContStates(rtm, val) ((rtm)->Sizes.numPeriodicContStates = (val))
#endif

#ifndef rtmGetNumSFcnParams
# define rtmGetNumSFcnParams(rtm)      ((rtm)->Sizes.numSFcnPrms)
#endif

#ifndef rtmSetNumSFcnParams
# define rtmSetNumSFcnParams(rtm, val) ((rtm)->Sizes.numSFcnPrms = (val))
#endif

#ifndef rtmGetNumSFunctions
# define rtmGetNumSFunctions(rtm)      ((rtm)->Sizes.numSFcns)
#endif

#ifndef rtmSetNumSFunctions
# define rtmSetNumSFunctions(rtm, val) ((rtm)->Sizes.numSFcns = (val))
#endif

#ifndef rtmGetNumSampleTimes
# define rtmGetNumSampleTimes(rtm)     ((rtm)->Sizes.numSampTimes)
#endif

#ifndef rtmSetNumSampleTimes
# define rtmSetNumSampleTimes(rtm, val) ((rtm)->Sizes.numSampTimes = (val))
#endif

#ifndef rtmGetNumU
# define rtmGetNumU(rtm)               ((rtm)->Sizes.numU)
#endif

#ifndef rtmSetNumU
# define rtmSetNumU(rtm, val)          ((rtm)->Sizes.numU = (val))
#endif

#ifndef rtmGetNumY
# define rtmGetNumY(rtm)               ((rtm)->Sizes.numY)
#endif

#ifndef rtmSetNumY
# define rtmSetNumY(rtm, val)          ((rtm)->Sizes.numY = (val))
#endif

#ifndef rtmGetOdeF
# define rtmGetOdeF(rtm)               ()
#endif

#ifndef rtmSetOdeF
# define rtmSetOdeF(rtm, val)          ()
#endif

#ifndef rtmGetOdeY
# define rtmGetOdeY(rtm)               ()
#endif

#ifndef rtmSetOdeY
# define rtmSetOdeY(rtm, val)          ()
#endif

#ifndef rtmGetOffsetTimeArray
# define rtmGetOffsetTimeArray(rtm)    ((rtm)->Timing.offsetTimesArray)
#endif

#ifndef rtmSetOffsetTimeArray
# define rtmSetOffsetTimeArray(rtm, val) ((rtm)->Timing.offsetTimesArray = (val))
#endif

#ifndef rtmGetOffsetTimePtr
# define rtmGetOffsetTimePtr(rtm)      ((rtm)->Timing.offsetTimes)
#endif

#ifndef rtmSetOffsetTimePtr
# define rtmSetOffsetTimePtr(rtm, val) ((rtm)->Timing.offsetTimes = (val))
#endif

#ifndef rtmGetOptions
# define rtmGetOptions(rtm)            ((rtm)->Sizes.options)
#endif

#ifndef rtmSetOptions
# define rtmSetOptions(rtm, val)       ((rtm)->Sizes.options = (val))
#endif

#ifndef rtmGetParamIsMalloced
# define rtmGetParamIsMalloced(rtm)    ()
#endif

#ifndef rtmSetParamIsMalloced
# define rtmSetParamIsMalloced(rtm, val) ()
#endif

#ifndef rtmGetPath
# define rtmGetPath(rtm)               ((rtm)->path)
#endif

#ifndef rtmSetPath
# define rtmSetPath(rtm, val)          ((rtm)->path = (val))
#endif

#ifndef rtmGetPerTaskSampleHits
# define rtmGetPerTaskSampleHits(rtm)  ()
#endif

#ifndef rtmSetPerTaskSampleHits
# define rtmSetPerTaskSampleHits(rtm, val) ()
#endif

#ifndef rtmGetPerTaskSampleHitsArray
# define rtmGetPerTaskSampleHitsArray(rtm) ((rtm)->Timing.perTaskSampleHitsArray)
#endif

#ifndef rtmSetPerTaskSampleHitsArray
# define rtmSetPerTaskSampleHitsArray(rtm, val) ((rtm)->Timing.perTaskSampleHitsArray = (val))
#endif

#ifndef rtmGetPerTaskSampleHitsPtr
# define rtmGetPerTaskSampleHitsPtr(rtm) ((rtm)->Timing.perTaskSampleHits)
#endif

#ifndef rtmSetPerTaskSampleHitsPtr
# define rtmSetPerTaskSampleHitsPtr(rtm, val) ((rtm)->Timing.perTaskSampleHits = (val))
#endif

#ifndef rtmGetPeriodicContStateIndices
# define rtmGetPeriodicContStateIndices(rtm) ((rtm)->ModelData.periodicContStateIndices)
#endif

#ifndef rtmSetPeriodicContStateIndices
# define rtmSetPeriodicContStateIndices(rtm, val) ((rtm)->ModelData.periodicContStateIndices = (val))
#endif

#ifndef rtmGetPeriodicContStateRanges
# define rtmGetPeriodicContStateRanges(rtm) ((rtm)->ModelData.periodicContStateRanges)
#endif

#ifndef rtmSetPeriodicContStateRanges
# define rtmSetPeriodicContStateRanges(rtm, val) ((rtm)->ModelData.periodicContStateRanges = (val))
#endif

#ifndef rtmGetPrevZCSigState
# define rtmGetPrevZCSigState(rtm)     ((rtm)->ModelData.prevZCSigState)
#endif

#ifndef rtmSetPrevZCSigState
# define rtmSetPrevZCSigState(rtm, val) ((rtm)->ModelData.prevZCSigState = (val))
#endif

#ifndef rtmGetRTWExtModeInfo
# define rtmGetRTWExtModeInfo(rtm)     ((rtm)->extModeInfo)
#endif

#ifndef rtmSetRTWExtModeInfo
# define rtmSetRTWExtModeInfo(rtm, val) ((rtm)->extModeInfo = (val))
#endif

#ifndef rtmGetRTWGeneratedSFcn
# define rtmGetRTWGeneratedSFcn(rtm)   ((rtm)->Sizes.rtwGenSfcn)
#endif

#ifndef rtmSetRTWGeneratedSFcn
# define rtmSetRTWGeneratedSFcn(rtm, val) ((rtm)->Sizes.rtwGenSfcn = (val))
#endif

#ifndef rtmGetRTWLogInfo
# define rtmGetRTWLogInfo(rtm)         ((rtm)->rtwLogInfo)
#endif

#ifndef rtmSetRTWLogInfo
# define rtmSetRTWLogInfo(rtm, val)    ((rtm)->rtwLogInfo = (val))
#endif

#ifndef rtmGetRTWRTModelMethodsInfo
# define rtmGetRTWRTModelMethodsInfo(rtm) ()
#endif

#ifndef rtmSetRTWRTModelMethodsInfo
# define rtmSetRTWRTModelMethodsInfo(rtm, val) ()
#endif

#ifndef rtmGetRTWSfcnInfo
# define rtmGetRTWSfcnInfo(rtm)        ((rtm)->sfcnInfo)
#endif

#ifndef rtmSetRTWSfcnInfo
# define rtmSetRTWSfcnInfo(rtm, val)   ((rtm)->sfcnInfo = (val))
#endif

#ifndef rtmGetRTWSolverInfo
# define rtmGetRTWSolverInfo(rtm)      ((rtm)->solverInfo)
#endif

#ifndef rtmSetRTWSolverInfo
# define rtmSetRTWSolverInfo(rtm, val) ((rtm)->solverInfo = (val))
#endif

#ifndef rtmGetRTWSolverInfoPtr
# define rtmGetRTWSolverInfoPtr(rtm)   ((rtm)->solverInfoPtr)
#endif

#ifndef rtmSetRTWSolverInfoPtr
# define rtmSetRTWSolverInfoPtr(rtm, val) ((rtm)->solverInfoPtr = (val))
#endif

#ifndef rtmGetReservedForXPC
# define rtmGetReservedForXPC(rtm)     ((rtm)->SpecialInfo.xpcData)
#endif

#ifndef rtmSetReservedForXPC
# define rtmSetReservedForXPC(rtm, val) ((rtm)->SpecialInfo.xpcData = (val))
#endif

#ifndef rtmGetRootDWork
# define rtmGetRootDWork(rtm)          ((rtm)->ModelData.dwork)
#endif

#ifndef rtmSetRootDWork
# define rtmSetRootDWork(rtm, val)     ((rtm)->ModelData.dwork = (val))
#endif

#ifndef rtmGetSFunctions
# define rtmGetSFunctions(rtm)         ((rtm)->childSfunctions)
#endif

#ifndef rtmSetSFunctions
# define rtmSetSFunctions(rtm, val)    ((rtm)->childSfunctions = (val))
#endif

#ifndef rtmGetSampleHitArray
# define rtmGetSampleHitArray(rtm)     ((rtm)->Timing.sampleHitArray)
#endif

#ifndef rtmSetSampleHitArray
# define rtmSetSampleHitArray(rtm, val) ((rtm)->Timing.sampleHitArray = (val))
#endif

#ifndef rtmGetSampleHitPtr
# define rtmGetSampleHitPtr(rtm)       ((rtm)->Timing.sampleHits)
#endif

#ifndef rtmSetSampleHitPtr
# define rtmSetSampleHitPtr(rtm, val)  ((rtm)->Timing.sampleHits = (val))
#endif

#ifndef rtmGetSampleTimeArray
# define rtmGetSampleTimeArray(rtm)    ((rtm)->Timing.sampleTimesArray)
#endif

#ifndef rtmSetSampleTimeArray
# define rtmSetSampleTimeArray(rtm, val) ((rtm)->Timing.sampleTimesArray = (val))
#endif

#ifndef rtmGetSampleTimePtr
# define rtmGetSampleTimePtr(rtm)      ((rtm)->Timing.sampleTimes)
#endif

#ifndef rtmSetSampleTimePtr
# define rtmSetSampleTimePtr(rtm, val) ((rtm)->Timing.sampleTimes = (val))
#endif

#ifndef rtmGetSampleTimeTaskIDArray
# define rtmGetSampleTimeTaskIDArray(rtm) ((rtm)->Timing.sampleTimeTaskIDArray)
#endif

#ifndef rtmSetSampleTimeTaskIDArray
# define rtmSetSampleTimeTaskIDArray(rtm, val) ((rtm)->Timing.sampleTimeTaskIDArray = (val))
#endif

#ifndef rtmGetSampleTimeTaskIDPtr
# define rtmGetSampleTimeTaskIDPtr(rtm) ((rtm)->Timing.sampleTimeTaskIDPtr)
#endif

#ifndef rtmSetSampleTimeTaskIDPtr
# define rtmSetSampleTimeTaskIDPtr(rtm, val) ((rtm)->Timing.sampleTimeTaskIDPtr = (val))
#endif

#ifndef rtmGetSimMode
# define rtmGetSimMode(rtm)            ((rtm)->simMode)
#endif

#ifndef rtmSetSimMode
# define rtmSetSimMode(rtm, val)       ((rtm)->simMode = (val))
#endif

#ifndef rtmGetSimTimeStep
# define rtmGetSimTimeStep(rtm)        ((rtm)->Timing.simTimeStep)
#endif

#ifndef rtmSetSimTimeStep
# define rtmSetSimTimeStep(rtm, val)   ((rtm)->Timing.simTimeStep = (val))
#endif

#ifndef rtmGetStartTime
# define rtmGetStartTime(rtm)          ((rtm)->Timing.tStart)
#endif

#ifndef rtmSetStartTime
# define rtmSetStartTime(rtm, val)     ((rtm)->Timing.tStart = (val))
#endif

#ifndef rtmGetStepSize
# define rtmGetStepSize(rtm)           ((rtm)->Timing.stepSize)
#endif

#ifndef rtmSetStepSize
# define rtmSetStepSize(rtm, val)      ((rtm)->Timing.stepSize = (val))
#endif

#ifndef rtmGetStopRequestedFlag
# define rtmGetStopRequestedFlag(rtm)  ((rtm)->Timing.stopRequestedFlag)
#endif

#ifndef rtmSetStopRequestedFlag
# define rtmSetStopRequestedFlag(rtm, val) ((rtm)->Timing.stopRequestedFlag = (val))
#endif

#ifndef rtmGetTaskCounters
# define rtmGetTaskCounters(rtm)       ()
#endif

#ifndef rtmSetTaskCounters
# define rtmSetTaskCounters(rtm, val)  ()
#endif

#ifndef rtmGetTaskTimeArray
# define rtmGetTaskTimeArray(rtm)      ((rtm)->Timing.tArray)
#endif

#ifndef rtmSetTaskTimeArray
# define rtmSetTaskTimeArray(rtm, val) ((rtm)->Timing.tArray = (val))
#endif

#ifndef rtmGetTimePtr
# define rtmGetTimePtr(rtm)            ((rtm)->Timing.t)
#endif

#ifndef rtmSetTimePtr
# define rtmSetTimePtr(rtm, val)       ((rtm)->Timing.t = (val))
#endif

#ifndef rtmGetTimingData
# define rtmGetTimingData(rtm)         ((rtm)->Timing.timingData)
#endif

#ifndef rtmSetTimingData
# define rtmSetTimingData(rtm, val)    ((rtm)->Timing.timingData = (val))
#endif

#ifndef rtmGetU
# define rtmGetU(rtm)                  ((rtm)->ModelData.inputs)
#endif

#ifndef rtmSetU
# define rtmSetU(rtm, val)             ((rtm)->ModelData.inputs = (val))
#endif

#ifndef rtmGetVarNextHitTimesListPtr
# define rtmGetVarNextHitTimesListPtr(rtm) ((rtm)->Timing.varNextHitTimesList)
#endif

#ifndef rtmSetVarNextHitTimesListPtr
# define rtmSetVarNextHitTimesListPtr(rtm, val) ((rtm)->Timing.varNextHitTimesList = (val))
#endif

#ifndef rtmGetY
# define rtmGetY(rtm)                  ((rtm)->ModelData.outputs)
#endif

#ifndef rtmSetY
# define rtmSetY(rtm, val)             ((rtm)->ModelData.outputs = (val))
#endif

#ifndef rtmGetZCCacheNeedsReset
# define rtmGetZCCacheNeedsReset(rtm)  ((rtm)->ModelData.zCCacheNeedsReset)
#endif

#ifndef rtmSetZCCacheNeedsReset
# define rtmSetZCCacheNeedsReset(rtm, val) ((rtm)->ModelData.zCCacheNeedsReset = (val))
#endif

#ifndef rtmGetZCSignalValues
# define rtmGetZCSignalValues(rtm)     ((rtm)->ModelData.zcSignalValues)
#endif

#ifndef rtmSetZCSignalValues
# define rtmSetZCSignalValues(rtm, val) ((rtm)->ModelData.zcSignalValues = (val))
#endif

#ifndef rtmGet_TimeOfLastOutput
# define rtmGet_TimeOfLastOutput(rtm)  ((rtm)->Timing.timeOfLastOutput)
#endif

#ifndef rtmSet_TimeOfLastOutput
# define rtmSet_TimeOfLastOutput(rtm, val) ((rtm)->Timing.timeOfLastOutput = (val))
#endif

#ifndef rtmGetdX
# define rtmGetdX(rtm)                 ((rtm)->ModelData.derivs)
#endif

#ifndef rtmSetdX
# define rtmSetdX(rtm, val)            ((rtm)->ModelData.derivs = (val))
#endif

#ifndef rtmGetChecksumVal
# define rtmGetChecksumVal(rtm, idx)   ((rtm)->Sizes.checksums[idx])
#endif

#ifndef rtmSetChecksumVal
# define rtmSetChecksumVal(rtm, idx, val) ((rtm)->Sizes.checksums[idx] = (val))
#endif

#ifndef rtmGetDWork
# define rtmGetDWork(rtm, idx)         ((rtm)->ModelData.dwork[idx])
#endif

#ifndef rtmSetDWork
# define rtmSetDWork(rtm, idx, val)    ((rtm)->ModelData.dwork[idx] = (val))
#endif

#ifndef rtmGetOffsetTime
# define rtmGetOffsetTime(rtm, idx)    ((rtm)->Timing.offsetTimes[idx])
#endif

#ifndef rtmSetOffsetTime
# define rtmSetOffsetTime(rtm, idx, val) ((rtm)->Timing.offsetTimes[idx] = (val))
#endif

#ifndef rtmGetSFunction
# define rtmGetSFunction(rtm, idx)     ((rtm)->childSfunctions[idx])
#endif

#ifndef rtmSetSFunction
# define rtmSetSFunction(rtm, idx, val) ((rtm)->childSfunctions[idx] = (val))
#endif

#ifndef rtmGetSampleTime
# define rtmGetSampleTime(rtm, idx)    ((rtm)->Timing.sampleTimes[idx])
#endif

#ifndef rtmSetSampleTime
# define rtmSetSampleTime(rtm, idx, val) ((rtm)->Timing.sampleTimes[idx] = (val))
#endif

#ifndef rtmGetSampleTimeTaskID
# define rtmGetSampleTimeTaskID(rtm, idx) ((rtm)->Timing.sampleTimeTaskIDPtr[idx])
#endif

#ifndef rtmSetSampleTimeTaskID
# define rtmSetSampleTimeTaskID(rtm, idx, val) ((rtm)->Timing.sampleTimeTaskIDPtr[idx] = (val))
#endif

#ifndef rtmGetVarNextHitTimeList
# define rtmGetVarNextHitTimeList(rtm, idx) ((rtm)->Timing.varNextHitTimesList[idx])
#endif

#ifndef rtmSetVarNextHitTimeList
# define rtmSetVarNextHitTimeList(rtm, idx, val) ((rtm)->Timing.varNextHitTimesList[idx] = (val))
#endif

#ifndef rtmIsContinuousTask
# define rtmIsContinuousTask(rtm, tid) ((tid) == 0)
#endif

#ifndef rtmGetErrorStatus
# define rtmGetErrorStatus(rtm)        ((rtm)->errorStatus)
#endif

#ifndef rtmSetErrorStatus
# define rtmSetErrorStatus(rtm, val)   ((rtm)->errorStatus = (val))
#endif

#ifndef rtmIsMajorTimeStep
# define rtmIsMajorTimeStep(rtm)       (((rtm)->Timing.simTimeStep) == MAJOR_TIME_STEP)
#endif

#ifndef rtmIsMinorTimeStep
# define rtmIsMinorTimeStep(rtm)       (((rtm)->Timing.simTimeStep) == MINOR_TIME_STEP)
#endif

#ifndef rtmIsSampleHit
# define rtmIsSampleHit(rtm, sti, tid) ((rtmIsMajorTimeStep((rtm)) && (rtm)->Timing.sampleHits[(rtm)->Timing.sampleTimeTaskIDPtr[sti]]))
#endif

#ifndef rtmGetStopRequested
# define rtmGetStopRequested(rtm)      ((rtm)->Timing.stopRequestedFlag)
#endif

#ifndef rtmSetStopRequested
# define rtmSetStopRequested(rtm, val) ((rtm)->Timing.stopRequestedFlag = (val))
#endif

#ifndef rtmGetStopRequestedPtr
# define rtmGetStopRequestedPtr(rtm)   (&((rtm)->Timing.stopRequestedFlag))
#endif

#ifndef rtmGetT
# define rtmGetT(rtm)                  (rtmGetTPtr((rtm))[0])
#endif

#ifndef rtmSetT
# define rtmSetT(rtm, val)                                       /* Do Nothing */
#endif

#ifndef rtmGetTFinal
# define rtmGetTFinal(rtm)             ((rtm)->Timing.tFinal)
#endif

#ifndef rtmSetTFinal
# define rtmSetTFinal(rtm, val)        ((rtm)->Timing.tFinal = (val))
#endif

#ifndef rtmGetTPtr
# define rtmGetTPtr(rtm)               ((rtm)->Timing.t)
#endif

#ifndef rtmSetTPtr
# define rtmSetTPtr(rtm, val)          ((rtm)->Timing.t = (val))
#endif

#ifndef rtmGetTStart
# define rtmGetTStart(rtm)             ((rtm)->Timing.tStart)
#endif

#ifndef rtmSetTStart
# define rtmSetTStart(rtm, val)        ((rtm)->Timing.tStart = (val))
#endif

#ifndef rtmGetTaskTime
# define rtmGetTaskTime(rtm, sti)      (rtmGetTPtr((rtm))[(rtm)->Timing.sampleTimeTaskIDPtr[sti]])
#endif

#ifndef rtmSetTaskTime
# define rtmSetTaskTime(rtm, sti, val) (rtmGetTPtr((rtm))[sti] = (val))
#endif

#ifndef rtmGetTimeOfLastOutput
# define rtmGetTimeOfLastOutput(rtm)   ((rtm)->Timing.timeOfLastOutput)
#endif

#ifdef rtmGetRTWSolverInfo
#undef rtmGetRTWSolverInfo
#endif

#define rtmGetRTWSolverInfo(rtm)       &((rtm)->solverInfo)
#define rtModel_RobotControl1          RT_MODEL_RobotControl1_T

/* Definition for use in the target main file */
#define RobotControl1_rtModel          RT_MODEL_RobotControl1_T

/* user code (top of export header file) */
#include "xpcdatatypes.h"

/* Block signals (auto storage) */
typedef struct {
  real_T Clock;                        /* '<Root>/Clock' */
  real_T UnitDelay;                    /* '<Root>/Unit Delay' */
  real_T Add;                          /* '<Root>/Add' */
  real_T UnitDelay1;                   /* '<Root>/Unit Delay1' */
  real_T packetsize;                   /* '<Root>/packet size' */
  real_T packetsize4;                  /* '<Root>/packet size4' */
  real_T enable;                       /* '<Root>/status checker' */
  real_T DataStoreRead;                /* '<S2>/Data Store Read' */
  real_T CCS;                          /* '<S4>/Command State' */
  real_T v;                            /* '<S1>/MATLAB Function 2' */
  real_T omega;                        /* '<S1>/MATLAB Function 2' */
  real_T PAN;                          /* '<S1>/Camera Servos Function' */
  real_T TILT;                         /* '<S1>/Camera Servos Function' */
  real_T PWM_L;                        /* '<S1>/ MATLAB Function1' */
  real_T PWM_R;                        /* '<S1>/ MATLAB Function1' */
  uint16_T Left_PWM;                   /* '<S17>/pulse transformer1' */
  uint16_T Right_PWM;                  /* '<S17>/pulse transformer1' */
  uint16_T PAN_O;                      /* '<S17>/pulse transformer camera' */
  uint16_T TILT_O;                     /* '<S17>/pulse transformer camera' */
  uint16_T ByteReversal_o1;            /* '<S30>/Byte Reversal' */
  uint16_T ByteReversal_o2;            /* '<S30>/Byte Reversal' */
  uint16_T ByteReversal_o3;            /* '<S30>/Byte Reversal' */
  uint16_T ByteReversal_o4;            /* '<S30>/Byte Reversal' */
  uint16_T ByteReversal_o5;            /* '<S30>/Byte Reversal' */
  uint16_T ByteReversal_o6;            /* '<S30>/Byte Reversal' */
  uint16_T ByteReversal_o7;            /* '<S30>/Byte Reversal' */
  uint16_T ByteReversal_o8;            /* '<S30>/Byte Reversal' */
  uint8_T RS232BinaryReceive1_o2[8];   /* '<Root>/RS232 Binary Receive1' */
  uint8_T Unpack_o1;                   /* '<S4>/Unpack' */
  uint8_T Unpack_o2;                   /* '<S4>/Unpack' */
  uint8_T Unpack_o3;                   /* '<S4>/Unpack' */
  uint8_T Unpack_o4;                   /* '<S4>/Unpack' */
  uint8_T Unpack_o5;                   /* '<S4>/Unpack' */
  uint8_T Unpack_o6;                   /* '<S4>/Unpack' */
  uint8_T Unpack_o7;                   /* '<S4>/Unpack' */
  uint8_T Unpack_o8;                   /* '<S4>/Unpack' */
  uint8_T Pack[22];                    /* '<S30>/Pack' */
  uint8_T CharArray[22];               /* '<S30>/Embedded MATLAB Function' */
  uint8_T csum0;                       /* '<S30>/CheckSum' */
  uint8_T csum1;                       /* '<S30>/CheckSum' */
} B_RobotControl1_T;

/* Block states (auto storage) for system '<Root>' */
typedef struct {
  real_T UnitDelay_DSTATE;             /* '<Root>/Unit Delay' */
  real_T UnitDelay1_DSTATE;            /* '<Root>/Unit Delay1' */
  real_T t_start;                      /* '<Root>/Data Store Memory' */
  struct {
    int_T AcquireOK;
  } SFunction_IWORK;                   /* '<S3>/S-Function' */

  int_T RS232BinaryReceive1_IWORK[2];  /* '<Root>/RS232 Binary Receive1' */
  struct {
    int_T AcquireOK;
  } SFunction_IWORK_j;                 /* '<S6>/S-Function' */

  struct {
    int_T AcquireOK;
  } SFunction_IWORK_o;                 /* '<S14>/S-Function' */

  struct {
    int_T AcquireOK;
  } SFunction_IWORK_h;                 /* '<S15>/S-Function' */

  struct {
    int_T AcquireOK;
  } SFunction_IWORK_l;                 /* '<S16>/S-Function' */

  int8_T Read_SubsysRanBC;             /* '<Root>/Read' */
  int8_T Openloopcontrol_SubsysRanBC;  /* '<Root>/Open-loop control' */
  int8_T StatusExtraction_SubsysRanBC; /* '<Root>/Status Extraction' */
  int8_T Write_SubsysRanBC;            /* '<Root>/Write' */
  uint8_T RS232BinaryReceive1_DWORK1[8];/* '<Root>/RS232 Binary Receive1' */
  boolean_T Openloopcontrol_MODE;      /* '<Root>/Open-loop control' */
} DW_RobotControl1_T;

/* Zero-crossing (trigger) state */
typedef struct {
  ZCSigState Write_Trig_ZCE;           /* '<Root>/Write' */
  ZCSigState Read_Trig_ZCE;            /* '<Root>/Read' */
} PrevZCX_RobotControl1_T;

/* Backward compatible GRT Identifiers */
#define rtB                            RobotControl1_B
#define BlockIO                        B_RobotControl1_T
#define rtP                            RobotControl1_P
#define Parameters                     P_RobotControl1_T
#define rtDWork                        RobotControl1_DW
#define D_Work                         DW_RobotControl1_T
#define rtPrevZCSigState               RobotControl1_PrevZCX
#define PrevZCSigStates                PrevZCX_RobotControl1_T

/* Parameters (auto storage) */
struct P_RobotControl1_T_ {
  real_T RS232BinarySend1_P1_Size[2];  /* Computed Parameter: RS232BinarySend1_P1_Size
                                        * Referenced by: '<S30>/RS232 Binary Send1'
                                        */
  real_T RS232BinarySend1_P1;          /* Expression: port
                                        * Referenced by: '<S30>/RS232 Binary Send1'
                                        */
  real_T RS232BinarySend1_P2_Size[2];  /* Computed Parameter: RS232BinarySend1_P2_Size
                                        * Referenced by: '<S30>/RS232 Binary Send1'
                                        */
  real_T RS232BinarySend1_P2;          /* Expression: width
                                        * Referenced by: '<S30>/RS232 Binary Send1'
                                        */
  real_T RS232BinarySend1_P3_Size[2];  /* Computed Parameter: RS232BinarySend1_P3_Size
                                        * Referenced by: '<S30>/RS232 Binary Send1'
                                        */
  real_T RS232BinarySend1_P3;          /* Expression: samptime
                                        * Referenced by: '<S30>/RS232 Binary Send1'
                                        */
  real_T UnitDelay_InitialCondition;   /* Expression: 0
                                        * Referenced by: '<Root>/Unit Delay'
                                        */
  real_T UnitDelay1_InitialCondition;  /* Expression: 0
                                        * Referenced by: '<Root>/Unit Delay1'
                                        */
  real_T packetsize_Value;             /* Expression: 8
                                        * Referenced by: '<Root>/packet size'
                                        */
  real_T packetsize4_Value;            /* Expression: 1
                                        * Referenced by: '<Root>/packet size4'
                                        */
  real_T RS232BinaryReceive1_P1_Size[2];/* Computed Parameter: RS232BinaryReceive1_P1_Size
                                         * Referenced by: '<Root>/RS232 Binary Receive1'
                                         */
  real_T RS232BinaryReceive1_P1;       /* Expression: port
                                        * Referenced by: '<Root>/RS232 Binary Receive1'
                                        */
  real_T RS232BinaryReceive1_P2_Size[2];/* Computed Parameter: RS232BinaryReceive1_P2_Size
                                         * Referenced by: '<Root>/RS232 Binary Receive1'
                                         */
  real_T RS232BinaryReceive1_P2;       /* Expression: width
                                        * Referenced by: '<Root>/RS232 Binary Receive1'
                                        */
  real_T RS232BinaryReceive1_P3_Size[2];/* Computed Parameter: RS232BinaryReceive1_P3_Size
                                         * Referenced by: '<Root>/RS232 Binary Receive1'
                                         */
  real_T RS232BinaryReceive1_P3;       /* Expression: samptime
                                        * Referenced by: '<Root>/RS232 Binary Receive1'
                                        */
  real_T DataStoreMemory_InitialValue; /* Expression: 0
                                        * Referenced by: '<Root>/Data Store Memory'
                                        */
  real_T RS232Setup_P1_Size[2];        /* Computed Parameter: RS232Setup_P1_Size
                                        * Referenced by: '<Root>/RS232 Setup '
                                        */
  real_T RS232Setup_P1;                /* Expression: port
                                        * Referenced by: '<Root>/RS232 Setup '
                                        */
  real_T RS232Setup_P2_Size[2];        /* Computed Parameter: RS232Setup_P2_Size
                                        * Referenced by: '<Root>/RS232 Setup '
                                        */
  real_T RS232Setup_P2;                /* Expression: baud
                                        * Referenced by: '<Root>/RS232 Setup '
                                        */
  real_T RS232Setup_P3_Size[2];        /* Computed Parameter: RS232Setup_P3_Size
                                        * Referenced by: '<Root>/RS232 Setup '
                                        */
  real_T RS232Setup_P3;                /* Expression: dbits
                                        * Referenced by: '<Root>/RS232 Setup '
                                        */
  real_T RS232Setup_P4_Size[2];        /* Computed Parameter: RS232Setup_P4_Size
                                        * Referenced by: '<Root>/RS232 Setup '
                                        */
  real_T RS232Setup_P4;                /* Expression: sbits
                                        * Referenced by: '<Root>/RS232 Setup '
                                        */
  real_T RS232Setup_P5_Size[2];        /* Computed Parameter: RS232Setup_P5_Size
                                        * Referenced by: '<Root>/RS232 Setup '
                                        */
  real_T RS232Setup_P5;                /* Expression: parity
                                        * Referenced by: '<Root>/RS232 Setup '
                                        */
  real_T RS232Setup_P6_Size[2];        /* Computed Parameter: RS232Setup_P6_Size
                                        * Referenced by: '<Root>/RS232 Setup '
                                        */
  real_T RS232Setup_P6;                /* Expression: prot
                                        * Referenced by: '<Root>/RS232 Setup '
                                        */
  real_T RS232Setup_P7_Size[2];        /* Computed Parameter: RS232Setup_P7_Size
                                        * Referenced by: '<Root>/RS232 Setup '
                                        */
  real_T RS232Setup_P7;                /* Expression: sbuf
                                        * Referenced by: '<Root>/RS232 Setup '
                                        */
  real_T RS232Setup_P8_Size[2];        /* Computed Parameter: RS232Setup_P8_Size
                                        * Referenced by: '<Root>/RS232 Setup '
                                        */
  real_T RS232Setup_P8;                /* Expression: rbuf
                                        * Referenced by: '<Root>/RS232 Setup '
                                        */
  real_T RS232Setup_P9_Size[2];        /* Computed Parameter: RS232Setup_P9_Size
                                        * Referenced by: '<Root>/RS232 Setup '
                                        */
  real_T RS232Setup_P9;                /* Expression: 0
                                        * Referenced by: '<Root>/RS232 Setup '
                                        */
  real_T RS232Setup_P10_Size[2];       /* Computed Parameter: RS232Setup_P10_Size
                                        * Referenced by: '<Root>/RS232 Setup '
                                        */
  real_T RS232Setup_P11_Size[2];       /* Computed Parameter: RS232Setup_P11_Size
                                        * Referenced by: '<Root>/RS232 Setup '
                                        */
  real_T RS232Setup_P12_Size[2];       /* Computed Parameter: RS232Setup_P12_Size
                                        * Referenced by: '<Root>/RS232 Setup '
                                        */
  real_T RS232Setup_P12;               /* Expression: iackto
                                        * Referenced by: '<Root>/RS232 Setup '
                                        */
  real_T RS232Setup_P13_Size[2];       /* Computed Parameter: RS232Setup_P13_Size
                                        * Referenced by: '<Root>/RS232 Setup '
                                        */
  real_T RS232Setup_P14_Size[2];       /* Computed Parameter: RS232Setup_P14_Size
                                        * Referenced by: '<Root>/RS232 Setup '
                                        */
  real_T RS232Setup_P15_Size[2];       /* Computed Parameter: RS232Setup_P15_Size
                                        * Referenced by: '<Root>/RS232 Setup '
                                        */
  real_T RS232Setup_P15;               /* Expression: tackto
                                        * Referenced by: '<Root>/RS232 Setup '
                                        */
  real_T RS232Setup_P16_Size[2];       /* Computed Parameter: RS232Setup_P16_Size
                                        * Referenced by: '<Root>/RS232 Setup '
                                        */
  real_T RS232Setup_P16;               /* Expression: initinfo
                                        * Referenced by: '<Root>/RS232 Setup '
                                        */
  real_T RS232Setup_P17_Size[2];       /* Computed Parameter: RS232Setup_P17_Size
                                        * Referenced by: '<Root>/RS232 Setup '
                                        */
  real_T RS232Setup_P17;               /* Expression: initackinfo
                                        * Referenced by: '<Root>/RS232 Setup '
                                        */
  real_T RS232Setup_P18_Size[2];       /* Computed Parameter: RS232Setup_P18_Size
                                        * Referenced by: '<Root>/RS232 Setup '
                                        */
  real_T RS232Setup_P18;               /* Expression: terminfo
                                        * Referenced by: '<Root>/RS232 Setup '
                                        */
  real_T RS232Setup_P19_Size[2];       /* Computed Parameter: RS232Setup_P19_Size
                                        * Referenced by: '<Root>/RS232 Setup '
                                        */
  real_T RS232Setup_P19;               /* Expression: termackinfo
                                        * Referenced by: '<Root>/RS232 Setup '
                                        */
  uint16_T Servo1_Value;               /* Computed Parameter: Servo1_Value
                                        * Referenced by: '<S17>/Servo1'
                                        */
  uint8_T SYNC0_Value;                 /* Computed Parameter: SYNC0_Value
                                        * Referenced by: '<S30>/SYNC 0'
                                        */
  uint8_T SYNC1_Value;                 /* Computed Parameter: SYNC1_Value
                                        * Referenced by: '<S30>/SYNC 1'
                                        */
  uint8_T ID_Value;                    /* Computed Parameter: ID_Value
                                        * Referenced by: '<S30>/ID'
                                        */
  uint8_T Count_Value;                 /* Computed Parameter: Count_Value
                                        * Referenced by: '<S30>/Count'
                                        */
};

/* Real-time Model Data Structure */
struct tag_RTM_RobotControl1_T {
  const char_T *path;
  const char_T *modelName;
  struct SimStruct_tag * *childSfunctions;
  const char_T *errorStatus;
  SS_SimMode simMode;
  RTWLogInfo *rtwLogInfo;
  RTWExtModeInfo *extModeInfo;
  RTWSolverInfo solverInfo;
  RTWSolverInfo *solverInfoPtr;
  void *sfcnInfo;

  /*
   * NonInlinedSFcns:
   * The following substructure contains information regarding
   * non-inlined s-functions used in the model.
   */
  struct {
    RTWSfcnInfo sfcnInfo;
    time_T *taskTimePtrs[2];
    SimStruct childSFunctions[3];
    SimStruct *childSFunctionPtrs[3];
    struct _ssBlkInfo2 blkInfo2[3];
    struct _ssSFcnModelMethods2 methods2[3];
    struct _ssSFcnModelMethods3 methods3[3];
    struct _ssStatesInfo2 statesInfo2[3];
    struct {
      time_T sfcnPeriod[1];
      time_T sfcnOffset[1];
      int_T sfcnTsMap[1];
      struct _ssPortInputs inputPortInfo[1];
      uint_T attribs[3];
      mxArray *params[3];
    } Sfcn0;

    struct {
      time_T sfcnPeriod[1];
      time_T sfcnOffset[1];
      int_T sfcnTsMap[1];
      struct _ssPortInputs inputPortInfo[2];
      struct _ssPortOutputs outputPortInfo[2];
      uint_T attribs[3];
      mxArray *params[3];
      struct _ssDWorkRecord dWork[2];
      struct _ssDWorkAuxRecord dWorkAux[2];
      int_T callSysOutputs[1];
      void *callSysArgs1[1];
      int_T callSysArgs2[1];
      SysOutputFcn callSysFcns[4];
    } Sfcn1;

    struct {
      time_T sfcnPeriod[1];
      time_T sfcnOffset[1];
      int_T sfcnTsMap[1];
      uint_T attribs[19];
      mxArray *params[19];
    } Sfcn2;
  } NonInlinedSFcns;

  /*
   * DataMapInfo:
   * The following substructure contains information regarding
   * structures generated in the model's C API.
   */
  struct {
    rtwCAPI_ModelMappingInfo mmi;
  } DataMapInfo;

  /*
   * ModelData:
   * The following substructure contains information regarding
   * the data used in the model.
   */
  struct {
    void *blockIO;
    const void *constBlockIO;
    void *defaultParam;
    ZCSigState *prevZCSigState;
    real_T *contStates;
    int_T *periodicContStateIndices;
    real_T *periodicContStateRanges;
    real_T *derivs;
    void *zcSignalValues;
    void *inputs;
    void *outputs;
    boolean_T *contStateDisabled;
    boolean_T zCCacheNeedsReset;
    boolean_T derivCacheNeedsReset;
    boolean_T blkStateChange;
    void *dwork;
  } ModelData;

  /*
   * Sizes:
   * The following substructure contains sizes information
   * for many of the model attributes such as inputs, outputs,
   * dwork, sample times, etc.
   */
  struct {
    uint32_T checksums[4];
    uint32_T options;
    int_T numContStates;
    int_T numPeriodicContStates;
    int_T numU;
    int_T numY;
    int_T numSampTimes;
    int_T numBlocks;
    int_T numBlockIO;
    int_T numBlockPrms;
    int_T numDwork;
    int_T numSFcnPrms;
    int_T numSFcns;
    int_T numIports;
    int_T numOports;
    int_T numNonSampZCs;
    int_T sysDirFeedThru;
    int_T rtwGenSfcn;
  } Sizes;

  /*
   * SpecialInfo:
   * The following substructure contains special information
   * related to other components that are dependent on RTW.
   */
  struct {
    const void *mappingInfo;
    void *xpcData;
  } SpecialInfo;

  /*
   * Timing:
   * The following substructure contains information regarding
   * the timing information for the model.
   */
  struct {
    time_T stepSize;
    uint32_T clockTick0;
    uint32_T clockTickH0;
    time_T stepSize0;
    uint32_T clockTick1;
    uint32_T clockTickH1;
    time_T stepSize1;
    time_T tStart;
    time_T tFinal;
    time_T timeOfLastOutput;
    void *timingData;
    real_T *varNextHitTimesList;
    SimTimeStep simTimeStep;
    boolean_T stopRequestedFlag;
    time_T *sampleTimes;
    time_T *offsetTimes;
    int_T *sampleTimeTaskIDPtr;
    int_T *sampleHits;
    int_T *perTaskSampleHits;
    time_T *t;
    time_T sampleTimesArray[2];
    time_T offsetTimesArray[2];
    int_T sampleTimeTaskIDArray[2];
    int_T sampleHitArray[2];
    int_T perTaskSampleHitsArray[4];
    time_T tArray[2];
  } Timing;
};

/* Block parameters (auto storage) */
extern P_RobotControl1_T RobotControl1_P;

/* Block signals (auto storage) */
extern B_RobotControl1_T RobotControl1_B;

/* Block states (auto storage) */
extern DW_RobotControl1_T RobotControl1_DW;

/* External data declarations for dependent source files */

/* Zero-crossing (trigger) state */
extern PrevZCX_RobotControl1_T RobotControl1_PrevZCX;

/*====================*
 * External functions *
 *====================*/
extern RobotControl1_rtModel *RobotControl1(void);
extern void MdlInitializeSizes(void);
extern void MdlInitializeSampleTimes(void);
extern void MdlInitialize(void);
extern void MdlStart(void);
extern void MdlOutputs(int_T tid);
extern void MdlUpdate(int_T tid);
extern void MdlTerminate(void);

/* Real-time Model object */
extern RT_MODEL_RobotControl1_T *const RobotControl1_M;

/*-
 * The generated code includes comments that allow you to trace directly
 * back to the appropriate location in the model.  The basic format
 * is <system>/block_name, where system is the system number (uniquely
 * assigned by Simulink) and block_name is the name of the block.
 *
 * Use the MATLAB hilite_system command to trace the generated code back
 * to the model.  For example,
 *
 * hilite_system('<S3>')    - opens system 3
 * hilite_system('<S3>/Kp') - opens and selects block Kp which resides in S3
 *
 * Here is the system hierarchy for this model
 *
 * '<Root>' : 'RobotControl1'
 * '<S1>'   : 'RobotControl1/Open-loop control'
 * '<S2>'   : 'RobotControl1/Read'
 * '<S3>'   : 'RobotControl1/Scope (xPC) '
 * '<S4>'   : 'RobotControl1/Status Extraction'
 * '<S5>'   : 'RobotControl1/Write'
 * '<S6>'   : 'RobotControl1/status'
 * '<S7>'   : 'RobotControl1/status checker'
 * '<S8>'   : 'RobotControl1/Open-loop control/ MATLAB Function1'
 * '<S9>'   : 'RobotControl1/Open-loop control/Camera Servos Function'
 * '<S10>'  : 'RobotControl1/Open-loop control/MATLAB Function 2'
 * '<S11>'  : 'RobotControl1/Open-loop control/Scope (xPC) 2'
 * '<S12>'  : 'RobotControl1/Open-loop control/Vision System'
 * '<S13>'  : 'RobotControl1/Open-loop control/actual out'
 * '<S14>'  : 'RobotControl1/Open-loop control/control time'
 * '<S15>'  : 'RobotControl1/Open-loop control/output 2 - L'
 * '<S16>'  : 'RobotControl1/Open-loop control/output 2 - L1'
 * '<S17>'  : 'RobotControl1/Open-loop control/plant'
 * '<S18>'  : 'RobotControl1/Open-loop control/Vision System/Calculate Target Points Version 3'
 * '<S19>'  : 'RobotControl1/Open-loop control/Vision System/Desired Pose'
 * '<S20>'  : 'RobotControl1/Open-loop control/Vision System/Desired Pose V2'
 * '<S21>'  : 'RobotControl1/Open-loop control/Vision System/Estimate Pose Error'
 * '<S22>'  : 'RobotControl1/Open-loop control/Vision System/Motion Simulation'
 * '<S23>'  : 'RobotControl1/Open-loop control/Vision System/Non-Holonomic Robot Controller'
 * '<S24>'  : 'RobotControl1/Open-loop control/Vision System/Scope '
 * '<S25>'  : 'RobotControl1/Open-loop control/Vision System/Scope 1'
 * '<S26>'  : 'RobotControl1/Open-loop control/Vision System/Scope 2'
 * '<S27>'  : 'RobotControl1/Open-loop control/Vision System/Scope 3'
 * '<S28>'  : 'RobotControl1/Open-loop control/Vision System/Motion Simulation/Scope 4'
 * '<S29>'  : 'RobotControl1/Open-loop control/Vision System/Motion Simulation/kinematic_model'
 * '<S30>'  : 'RobotControl1/Open-loop control/plant/Send Serial Packet to Servo Controller And recieve status message from SSC'
 * '<S31>'  : 'RobotControl1/Open-loop control/plant/pulse transformer camera'
 * '<S32>'  : 'RobotControl1/Open-loop control/plant/pulse transformer1'
 * '<S33>'  : 'RobotControl1/Open-loop control/plant/Send Serial Packet to Servo Controller And recieve status message from SSC/CheckSum'
 * '<S34>'  : 'RobotControl1/Open-loop control/plant/Send Serial Packet to Servo Controller And recieve status message from SSC/Embedded MATLAB Function'
 * '<S35>'  : 'RobotControl1/Status Extraction/Command State'
 */
#endif                                 /* RTW_HEADER_RobotControl1_h_ */
