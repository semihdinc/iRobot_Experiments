/*
 * mirageTrackingRobot.c
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

#include "rt_logging_mmi.h"
#include "mirageTrackingRobot_capi.h"
#include "mirageTrackingRobot.h"
#include "mirageTrackingRobot_private.h"

/* Block signals (auto storage) */
B_mirageTrackingRobot_T mirageTrackingRobot_B;

/* Block states (auto storage) */
DW_mirageTrackingRobot_T mirageTrackingRobot_DW;

/* Real-time model */
RT_MODEL_mirageTrackingRobot_T mirageTrackingRobot_M_;
RT_MODEL_mirageTrackingRobot_T *const mirageTrackingRobot_M =
  &mirageTrackingRobot_M_;

/* Forward declaration for local functions */
static void mirageTrackingRobot_medfilt2(const uint8_T varargin_1[1228800],
  uint8_T b[1228800]);
static void mirageTrackingRobot_myMat2gray(const uint8_T imgIn[1228800], real_T
  imgOut[1228800]);
static void mirageTrackingR_SystemCore_step(visioncodegen_BlobAnalysis_mi_T *obj,
  const boolean_T varargin_1[1228800], int32_T varargout_1_data[], int32_T
  varargout_1_sizes[2], real_T varargout_2_data[], int32_T varargout_2_sizes[2],
  int32_T varargout_3_data[], int32_T varargout_3_sizes[2]);
static void mirageTrackin_SystemCore_step_i(visioncodegen_BlobAnalysis_mi_T *obj,
  const boolean_T varargin_1[1228800], int32_T varargout_1_data[], int32_T
  varargout_1_sizes[2], real_T varargout_2_data[], int32_T varargout_2_sizes[2],
  int32_T varargout_3_data[], int32_T varargout_3_sizes[2]);
static void mirageTrackingRobot_eml_sort(real_T x[4], int32_T idx[4]);
static void mirageTrackingRobot_orderPoints(const real_T p[8], real_T new_p[8]);
static void mirageTrackingRobot_invNxN(const real_T x[16], real_T y[16]);
static void mirageTra_transformToImageSpace(const real_T S_P[16], real_T
  Cam_E_S[16], const real_T K[16], real_T p[8], real_T P[16], real_T M[16]);
static void mirageTrackingRobo_getEquations(const real_T Cam_PDes[16], const
  real_T S_PDes[16], const real_T Err[8], const real_T M[16], real_T equations
  [96], real_T constants[8]);
static void mirageTrackingRobot_eml_xgetrf(real_T A[144], int32_T ipiv[12]);
static void mirageTrackingRo_solveEquations(const real_T A[192], const real_T B
  [16], real_T params[12]);
static void mirageTrackingRobot_visioMotor(const real_T L_p[8], const real_T
  R_p[8], const real_T S_PDes[16], real_T errParams[12]);
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

real_T rt_remd_snf(real_T u0, real_T u1)
{
  real_T y;
  real_T tr;
  boolean_T y_0;
  boolean_T y_1;
  y_0 = ((!rtIsNaN(u0)) && (!rtIsInf(u0)));
  y_1 = ((!rtIsNaN(u1)) && (!rtIsInf(u1)));
  if (!(y_0 && y_1)) {
    y = (rtNaN);
  } else {
    if (u1 < 0.0) {
      y = ceil(u1);
    } else {
      y = floor(u1);
    }

    if ((u1 != 0.0) && (u1 != y)) {
      tr = u0 / u1;
      if (fabs(tr - rt_roundd_snf(tr)) <= DBL_EPSILON * fabs(tr)) {
        y = 0.0;
      } else {
        y = fmod(u0, u1);
      }
    } else {
      y = fmod(u0, u1);
    }
  }

  return y;
}

/* Function for MATLAB Function: '<Root>/Calculate Target Points' */
static void mirageTrackingRobot_medfilt2(const uint8_T varargin_1[1228800],
  uint8_T b[1228800])
{
  int32_T np_ImageNeighborLinearOffsets[9];
  real_T indx;
  int32_T centerLocationInImage;
  int32_T b_pind;
  int32_T secondInd;
  int32_T firstInd;
  int32_T idx[9];
  int32_T iwork[9];
  int32_T c_k;
  int32_T e_i;
  int32_T i2;
  int32_T e_j;
  int32_T p;
  int32_T q;
  int32_T qEnd;
  int32_T kEnd;
  boolean_T b_p;
  int16_T subs_idx_0;
  int16_T subs_idx_1;
  int8_T subs_idx_1_0;
  int8_T subs_idx_0_0;
  memset(&mirageTrackingRobot_B.a[0], 0, 962U * sizeof(uint8_T));
  memset(&mirageTrackingRobot_B.a[1232322], 0, 962U * sizeof(uint8_T));
  for (centerLocationInImage = 0; centerLocationInImage < 1280;
       centerLocationInImage++) {
    mirageTrackingRobot_B.a[962 * (centerLocationInImage + 1)] = 0U;
  }

  for (centerLocationInImage = 0; centerLocationInImage < 1280;
       centerLocationInImage++) {
    mirageTrackingRobot_B.a[961 + 962 * (centerLocationInImage + 1)] = 0U;
  }

  for (centerLocationInImage = 0; centerLocationInImage < 1280;
       centerLocationInImage++) {
    memcpy(&mirageTrackingRobot_B.a[962 * (centerLocationInImage + 1) + 1],
           &varargin_1[960 * centerLocationInImage], 960U * sizeof(uint8_T));
  }

  indx = 1.0;
  for (centerLocationInImage = 0; centerLocationInImage < 9;
       centerLocationInImage++) {
    p = (int32_T)rt_remd_snf((1.0 + (real_T)centerLocationInImage) - 1.0, 3.0) +
      1;
    subs_idx_1_0 = (int8_T)((int32_T)(((real_T)(centerLocationInImage - p) + 1.0)
      / 3.0) + 1);
    subs_idx_0_0 = (int8_T)p;
    subs_idx_0 = subs_idx_0_0;
    subs_idx_1 = (int16_T)(subs_idx_1_0 - 1);
    subs_idx_1 = (int16_T)(subs_idx_1 * 962);
    np_ImageNeighborLinearOffsets[(int32_T)indx - 1] = subs_idx_0 + subs_idx_1;
    indx++;
  }

  for (centerLocationInImage = 0; centerLocationInImage < 9;
       centerLocationInImage++) {
    secondInd = np_ImageNeighborLinearOffsets[centerLocationInImage];
    secondInd -= 964;
    np_ImageNeighborLinearOffsets[centerLocationInImage] = secondInd;
  }

  for (secondInd = 1; secondInd + 1 < 1282; secondInd++) {
    for (firstInd = 1; firstInd + 1 < 962; firstInd++) {
      b_pind = secondInd * 962 + firstInd;
      b_p = (mirageTrackingRobot_B.a[np_ImageNeighborLinearOffsets[0] + b_pind] <=
             mirageTrackingRobot_B.a[np_ImageNeighborLinearOffsets[1] + b_pind]);
      if (b_p) {
        idx[0] = 1;
        idx[1] = 2;
      } else {
        idx[0] = 2;
        idx[1] = 1;
      }

      b_p = (mirageTrackingRobot_B.a[np_ImageNeighborLinearOffsets[2] + b_pind] <=
             mirageTrackingRobot_B.a[np_ImageNeighborLinearOffsets[3] + b_pind]);
      if (b_p) {
        idx[2] = 3;
        idx[3] = 4;
      } else {
        idx[2] = 4;
        idx[3] = 3;
      }

      b_p = (mirageTrackingRobot_B.a[np_ImageNeighborLinearOffsets[4] + b_pind] <=
             mirageTrackingRobot_B.a[np_ImageNeighborLinearOffsets[5] + b_pind]);
      if (b_p) {
        idx[4] = 5;
        idx[5] = 6;
      } else {
        idx[4] = 6;
        idx[5] = 5;
      }

      b_p = (mirageTrackingRobot_B.a[np_ImageNeighborLinearOffsets[6] + b_pind] <=
             mirageTrackingRobot_B.a[np_ImageNeighborLinearOffsets[7] + b_pind]);
      if (b_p) {
        idx[6] = 7;
        idx[7] = 8;
      } else {
        idx[6] = 8;
        idx[7] = 7;
      }

      idx[8] = 9;
      e_i = 2;
      while (e_i < 9) {
        i2 = e_i << 1;
        e_j = 1;
        centerLocationInImage = 1 + e_i;
        while (centerLocationInImage < 10) {
          p = e_j;
          q = centerLocationInImage;
          qEnd = e_j + i2;
          if (qEnd > 10) {
            qEnd = 10;
          }

          c_k = 0;
          kEnd = qEnd - e_j;
          while (c_k + 1 <= kEnd) {
            b_p = (mirageTrackingRobot_B.a[np_ImageNeighborLinearOffsets[idx[p -
                   1] - 1] + b_pind] <=
                   mirageTrackingRobot_B.a[np_ImageNeighborLinearOffsets[idx[q -
                   1] - 1] + b_pind]);
            if (b_p) {
              iwork[c_k] = idx[p - 1];
              p++;
              if (p == centerLocationInImage) {
                while (q < qEnd) {
                  c_k++;
                  iwork[c_k] = idx[q - 1];
                  q++;
                }
              }
            } else {
              iwork[c_k] = idx[q - 1];
              q++;
              if (q == qEnd) {
                while (p < centerLocationInImage) {
                  c_k++;
                  iwork[c_k] = idx[p - 1];
                  p++;
                }
              }
            }

            c_k++;
          }

          for (centerLocationInImage = 0; centerLocationInImage + 1 <= kEnd;
               centerLocationInImage++) {
            idx[(e_j + centerLocationInImage) - 1] = iwork[centerLocationInImage];
          }

          e_j = qEnd;
          centerLocationInImage = qEnd + e_i;
        }

        e_i = i2;
      }

      mirageTrackingRobot_B.b_pad[firstInd + 962 * secondInd] =
        mirageTrackingRobot_B.a[np_ImageNeighborLinearOffsets[idx[4] - 1] +
        b_pind];
    }
  }

  for (centerLocationInImage = 0; centerLocationInImage < 1280;
       centerLocationInImage++) {
    memcpy(&b[960 * centerLocationInImage], &mirageTrackingRobot_B.b_pad
           [(centerLocationInImage + 1) * 962 + 1], 960U * sizeof(uint8_T));
  }
}

/* Function for MATLAB Function: '<Root>/Calculate Target Points' */
static void mirageTrackingRobot_myMat2gray(const uint8_T imgIn[1228800], real_T
  imgOut[1228800])
{
  int32_T mtmp;
  int32_T ix;
  real_T imgOut_0;

  /* '<S1>:1:72' */
  for (ix = 0; ix < 1228800; ix++) {
    imgOut[ix] = imgIn[ix];
  }

  /* '<S1>:1:74' */
  mtmp = (int32_T)imgOut[0];
  for (ix = 1; ix + 1 < 1228801; ix++) {
    if ((int32_T)imgOut[ix] < mtmp) {
      mtmp = (int32_T)imgOut[ix];
    }
  }

  /* '<S1>:1:76' */
  for (ix = 0; ix < 1228800; ix++) {
    imgOut_0 = imgOut[ix];
    imgOut_0 -= (real_T)mtmp;
    imgOut[ix] = imgOut_0;
  }

  /* '<S1>:1:78' */
  mtmp = (int16_T)imgOut[0];
  for (ix = 1; ix + 1 < 1228801; ix++) {
    if ((int16_T)imgOut[ix] > mtmp) {
      mtmp = (int16_T)imgOut[ix];
    }
  }

  /* '<S1>:1:80' */
  for (ix = 0; ix < 1228800; ix++) {
    imgOut_0 = imgOut[ix];
    imgOut_0 /= (real_T)mtmp;
    imgOut[ix] = imgOut_0;
  }
}

/* Function for MATLAB Function: '<Root>/Calculate Target Points' */
static void mirageTrackingR_SystemCore_step(visioncodegen_BlobAnalysis_mi_T *obj,
  const boolean_T varargin_1[1228800], int32_T varargout_1_data[], int32_T
  varargout_1_sizes[2], real_T varargout_2_data[], int32_T varargout_2_sizes[2],
  int32_T varargout_3_data[], int32_T varargout_3_sizes[2])
{
  visioncodegen_BlobAnalysis_mi_T *b_obj;
  vision_BlobAnalysis_1_mirageT_T *d_obj;
  boolean_T maxNumBlobsReached;
  int32_T loop;
  uint8_T currentLabel;
  int32_T idx;
  int32_T n;
  uint32_T stackIdx;
  uint32_T pixIdx;
  uint32_T start_pixIdx;
  uint32_T padIdx;
  uint32_T walkerIdx;
  int32_T numBlobs;
  int32_T pixListMinc;
  int32_T pixListNinc;
  int32_T c_i;
  int32_T j;
  int32_T maxc;
  int32_T maxr;
  real_T centroid_idx_0;
  real_T centroid_idx_1;
  if (obj->isInitialized != 1) {
    b_obj = obj;
    b_obj->isInitialized = 1;
    b_obj->NoTuningBeforeLockingCodeGenError = true;
  }

  b_obj = obj;
  d_obj = &b_obj->cSFunObject;

  /* System object Outputs function: vision.BlobAnalysis */
  maxNumBlobsReached = false;
  for (loop = 0; loop < 963; loop++) {
    d_obj->W3_PAD_DW[loop] = 0U;
  }

  currentLabel = 1U;
  loop = 0;
  idx = 963;
  for (n = 0; n < 1280; n++) {
    for (maxc = 0; maxc < 960; maxc++) {
      d_obj->W3_PAD_DW[idx] = (uint8_T)(varargin_1[loop] ? 255 : 0);
      loop++;
      idx++;
    }

    d_obj->W3_PAD_DW[idx] = 0U;
    d_obj->W3_PAD_DW[idx + 1] = 0U;
    idx += 2;
  }

  for (loop = 0; loop < 961; loop++) {
    d_obj->W3_PAD_DW[loop + idx] = 0U;
  }

  loop = 1;
  pixIdx = 0U;
  n = 0;
  while (n < 1280) {
    maxc = 1;
    idx = loop * 962;
    maxr = 0;
    while (maxr < 960) {
      padIdx = (uint32_T)(idx + maxc);
      start_pixIdx = pixIdx;
      if (d_obj->W3_PAD_DW[padIdx] == 255) {
        d_obj->W3_PAD_DW[padIdx] = currentLabel;
        d_obj->W0_N_PIXLIST_DW[pixIdx] = (int16_T)(loop - 1);
        d_obj->W1_M_PIXLIST_DW[pixIdx] = (int16_T)(maxc - 1);
        pixIdx++;
        d_obj->W2_NUM_PIX_DW[currentLabel - 1] = 1U;
        d_obj->W4_STACK_DW[0U] = padIdx;
        stackIdx = 1U;
        while (stackIdx != 0U) {
          stackIdx--;
          padIdx = d_obj->W4_STACK_DW[stackIdx];
          for (numBlobs = 0; numBlobs < 8; numBlobs++) {
            walkerIdx = padIdx + d_obj->P0_WALKER_RTP[numBlobs];
            if (d_obj->W3_PAD_DW[walkerIdx] == 255) {
              d_obj->W3_PAD_DW[walkerIdx] = currentLabel;
              d_obj->W0_N_PIXLIST_DW[pixIdx] = (int16_T)((int16_T)(walkerIdx /
                962U) - 1);
              d_obj->W1_M_PIXLIST_DW[pixIdx] = (int16_T)(walkerIdx % 962U - 1U);
              pixIdx++;
              d_obj->W2_NUM_PIX_DW[currentLabel - 1]++;
              d_obj->W4_STACK_DW[stackIdx] = walkerIdx;
              stackIdx++;
            }
          }
        }

        if ((d_obj->W2_NUM_PIX_DW[currentLabel - 1] < d_obj->P1_MINAREA_RTP) ||
            (d_obj->W2_NUM_PIX_DW[currentLabel - 1] > d_obj->P2_MAXAREA_RTP)) {
          currentLabel--;
          pixIdx = start_pixIdx;
        }

        if (currentLabel == 50) {
          maxNumBlobsReached = true;
          n = 1280;
          maxr = 960;
        }

        if (maxr < 960) {
          currentLabel++;
        }
      }

      maxc++;
      maxr++;
    }

    loop++;
    n++;
  }

  numBlobs = maxNumBlobsReached ? (int32_T)currentLabel : (int32_T)(uint8_T)
    (currentLabel - 1U);
  pixListMinc = 0;
  pixListNinc = 0;
  for (c_i = 0; c_i < numBlobs; c_i++) {
    varargout_1_data[c_i] = (int32_T)d_obj->W2_NUM_PIX_DW[c_i];
    loop = 0;
    n = 0;
    for (j = 0; j < (int32_T)d_obj->W2_NUM_PIX_DW[c_i]; j++) {
      loop += d_obj->W0_N_PIXLIST_DW[j + pixListNinc];
      n += d_obj->W1_M_PIXLIST_DW[j + pixListMinc];
    }

    centroid_idx_0 = (real_T)n / (real_T)d_obj->W2_NUM_PIX_DW[c_i];
    centroid_idx_1 = (real_T)loop / (real_T)d_obj->W2_NUM_PIX_DW[c_i];
    varargout_2_data[c_i] = centroid_idx_1 + 1.0;
    varargout_2_data[(uint32_T)numBlobs + c_i] = centroid_idx_0 + 1.0;
    n = 1280;
    idx = 960;
    maxc = 0;
    maxr = 0;
    for (j = 0; j < (int32_T)d_obj->W2_NUM_PIX_DW[c_i]; j++) {
      loop = j + pixListNinc;
      if (d_obj->W0_N_PIXLIST_DW[loop] < n) {
        n = d_obj->W0_N_PIXLIST_DW[loop];
      }

      if (d_obj->W0_N_PIXLIST_DW[loop] > maxc) {
        maxc = d_obj->W0_N_PIXLIST_DW[loop];
      }

      loop = j + pixListMinc;
      if (d_obj->W1_M_PIXLIST_DW[loop] < idx) {
        idx = d_obj->W1_M_PIXLIST_DW[loop];
      }

      if (d_obj->W1_M_PIXLIST_DW[loop] > maxr) {
        maxr = d_obj->W1_M_PIXLIST_DW[loop];
      }
    }

    varargout_3_data[c_i] = n + 1;
    varargout_3_data[(uint32_T)numBlobs + c_i] = idx + 1;
    varargout_3_data[(numBlobs << 1) + c_i] = (maxc - n) + 1;
    varargout_3_data[3 * numBlobs + c_i] = (maxr - idx) + 1;
    pixListMinc += (int32_T)d_obj->W2_NUM_PIX_DW[c_i];
    pixListNinc += (int32_T)d_obj->W2_NUM_PIX_DW[c_i];
  }

  varargout_1_sizes[0] = numBlobs;
  varargout_1_sizes[1] = 1;
  varargout_2_sizes[0] = numBlobs;
  varargout_2_sizes[1] = 2;
  varargout_3_sizes[0] = numBlobs;
  varargout_3_sizes[1] = 4;
}

/* Function for MATLAB Function: '<Root>/Calculate Target Points' */
static void mirageTrackin_SystemCore_step_i(visioncodegen_BlobAnalysis_mi_T *obj,
  const boolean_T varargin_1[1228800], int32_T varargout_1_data[], int32_T
  varargout_1_sizes[2], real_T varargout_2_data[], int32_T varargout_2_sizes[2],
  int32_T varargout_3_data[], int32_T varargout_3_sizes[2])
{
  visioncodegen_BlobAnalysis_mi_T *b_obj;
  vision_BlobAnalysis_1_mirageT_T *d_obj;
  boolean_T maxNumBlobsReached;
  int32_T loop;
  uint8_T currentLabel;
  int32_T idx;
  int32_T n;
  uint32_T stackIdx;
  uint32_T pixIdx;
  uint32_T start_pixIdx;
  uint32_T padIdx;
  uint32_T walkerIdx;
  int32_T numBlobs;
  int32_T pixListMinc;
  int32_T pixListNinc;
  int32_T c_i;
  int32_T j;
  int32_T maxc;
  int32_T maxr;
  real_T centroid_idx_0;
  real_T centroid_idx_1;
  if (obj->isInitialized != 1) {
    b_obj = obj;
    b_obj->isInitialized = 1;
    b_obj->NoTuningBeforeLockingCodeGenError = true;
  }

  b_obj = obj;
  d_obj = &b_obj->cSFunObject;

  /* System object Outputs function: vision.BlobAnalysis */
  maxNumBlobsReached = false;
  for (loop = 0; loop < 963; loop++) {
    d_obj->W3_PAD_DW[loop] = 0U;
  }

  currentLabel = 1U;
  loop = 0;
  idx = 963;
  for (n = 0; n < 1280; n++) {
    for (maxc = 0; maxc < 960; maxc++) {
      d_obj->W3_PAD_DW[idx] = (uint8_T)(varargin_1[loop] ? 255 : 0);
      loop++;
      idx++;
    }

    d_obj->W3_PAD_DW[idx] = 0U;
    d_obj->W3_PAD_DW[idx + 1] = 0U;
    idx += 2;
  }

  for (loop = 0; loop < 961; loop++) {
    d_obj->W3_PAD_DW[loop + idx] = 0U;
  }

  loop = 1;
  pixIdx = 0U;
  n = 0;
  while (n < 1280) {
    maxc = 1;
    idx = loop * 962;
    maxr = 0;
    while (maxr < 960) {
      padIdx = (uint32_T)(idx + maxc);
      start_pixIdx = pixIdx;
      if (d_obj->W3_PAD_DW[padIdx] == 255) {
        d_obj->W3_PAD_DW[padIdx] = currentLabel;
        d_obj->W0_N_PIXLIST_DW[pixIdx] = (int16_T)(loop - 1);
        d_obj->W1_M_PIXLIST_DW[pixIdx] = (int16_T)(maxc - 1);
        pixIdx++;
        d_obj->W2_NUM_PIX_DW[currentLabel - 1] = 1U;
        d_obj->W4_STACK_DW[0U] = padIdx;
        stackIdx = 1U;
        while (stackIdx != 0U) {
          stackIdx--;
          padIdx = d_obj->W4_STACK_DW[stackIdx];
          for (numBlobs = 0; numBlobs < 8; numBlobs++) {
            walkerIdx = padIdx + d_obj->P0_WALKER_RTP[numBlobs];
            if (d_obj->W3_PAD_DW[walkerIdx] == 255) {
              d_obj->W3_PAD_DW[walkerIdx] = currentLabel;
              d_obj->W0_N_PIXLIST_DW[pixIdx] = (int16_T)((int16_T)(walkerIdx /
                962U) - 1);
              d_obj->W1_M_PIXLIST_DW[pixIdx] = (int16_T)(walkerIdx % 962U - 1U);
              pixIdx++;
              d_obj->W2_NUM_PIX_DW[currentLabel - 1]++;
              d_obj->W4_STACK_DW[stackIdx] = walkerIdx;
              stackIdx++;
            }
          }
        }

        if ((d_obj->W2_NUM_PIX_DW[currentLabel - 1] < d_obj->P1_MINAREA_RTP) ||
            (d_obj->W2_NUM_PIX_DW[currentLabel - 1] > d_obj->P2_MAXAREA_RTP)) {
          currentLabel--;
          pixIdx = start_pixIdx;
        }

        if (currentLabel == 50) {
          maxNumBlobsReached = true;
          n = 1280;
          maxr = 960;
        }

        if (maxr < 960) {
          currentLabel++;
        }
      }

      maxc++;
      maxr++;
    }

    loop++;
    n++;
  }

  numBlobs = maxNumBlobsReached ? (int32_T)currentLabel : (int32_T)(uint8_T)
    (currentLabel - 1U);
  pixListMinc = 0;
  pixListNinc = 0;
  for (c_i = 0; c_i < numBlobs; c_i++) {
    varargout_1_data[c_i] = (int32_T)d_obj->W2_NUM_PIX_DW[c_i];
    loop = 0;
    n = 0;
    for (j = 0; j < (int32_T)d_obj->W2_NUM_PIX_DW[c_i]; j++) {
      loop += d_obj->W0_N_PIXLIST_DW[j + pixListNinc];
      n += d_obj->W1_M_PIXLIST_DW[j + pixListMinc];
    }

    centroid_idx_0 = (real_T)n / (real_T)d_obj->W2_NUM_PIX_DW[c_i];
    centroid_idx_1 = (real_T)loop / (real_T)d_obj->W2_NUM_PIX_DW[c_i];
    varargout_2_data[c_i] = centroid_idx_1 + 1.0;
    varargout_2_data[(uint32_T)numBlobs + c_i] = centroid_idx_0 + 1.0;
    n = 1280;
    idx = 960;
    maxc = 0;
    maxr = 0;
    for (j = 0; j < (int32_T)d_obj->W2_NUM_PIX_DW[c_i]; j++) {
      loop = j + pixListNinc;
      if (d_obj->W0_N_PIXLIST_DW[loop] < n) {
        n = d_obj->W0_N_PIXLIST_DW[loop];
      }

      if (d_obj->W0_N_PIXLIST_DW[loop] > maxc) {
        maxc = d_obj->W0_N_PIXLIST_DW[loop];
      }

      loop = j + pixListMinc;
      if (d_obj->W1_M_PIXLIST_DW[loop] < idx) {
        idx = d_obj->W1_M_PIXLIST_DW[loop];
      }

      if (d_obj->W1_M_PIXLIST_DW[loop] > maxr) {
        maxr = d_obj->W1_M_PIXLIST_DW[loop];
      }
    }

    varargout_3_data[c_i] = n + 1;
    varargout_3_data[(uint32_T)numBlobs + c_i] = idx + 1;
    varargout_3_data[(numBlobs << 1) + c_i] = (maxc - n) + 1;
    varargout_3_data[3 * numBlobs + c_i] = (maxr - idx) + 1;
    pixListMinc += (int32_T)d_obj->W2_NUM_PIX_DW[c_i];
    pixListNinc += (int32_T)d_obj->W2_NUM_PIX_DW[c_i];
  }

  varargout_1_sizes[0] = numBlobs;
  varargout_1_sizes[1] = 1;
  varargout_2_sizes[0] = numBlobs;
  varargout_2_sizes[1] = 2;
  varargout_3_sizes[0] = numBlobs;
  varargout_3_sizes[1] = 4;
}

/* Function for MATLAB Function: '<Root>/Calculate Target Points' */
static void mirageTrackingRobot_eml_sort(real_T x[4], int32_T idx[4])
{
  int32_T nNaNs;
  real_T x4[4];
  int8_T idx4[4];
  real_T xwork[4];
  int32_T ib;
  int32_T m;
  int8_T perm[4];
  int32_T i4;
  int32_T bLen;
  int32_T nPairs;
  idx[0] = 0;
  idx[1] = 0;
  idx[2] = 0;
  idx[3] = 0;
  x4[0] = 0.0;
  x4[1] = 0.0;
  x4[2] = 0.0;
  x4[3] = 0.0;
  idx4[0] = 0;
  idx4[1] = 0;
  idx4[2] = 0;
  idx4[3] = 0;
  nNaNs = 0;
  ib = 0;
  if (rtIsNaN(x[0])) {
    idx[3] = 1;
    xwork[3] = x[0];
    nNaNs = 1;
  } else {
    ib = 1;
    idx4[0] = 1;
    x4[0] = x[0];
  }

  if (rtIsNaN(x[1])) {
    idx[3 - nNaNs] = 2;
    xwork[3 - nNaNs] = x[1];
    nNaNs++;
  } else {
    ib++;
    idx4[ib - 1] = 2;
    x4[ib - 1] = x[1];
  }

  if (rtIsNaN(x[2])) {
    idx[3 - nNaNs] = 3;
    xwork[3 - nNaNs] = x[2];
    nNaNs++;
  } else {
    ib++;
    idx4[ib - 1] = 3;
    x4[ib - 1] = x[2];
  }

  if (rtIsNaN(x[3])) {
    idx[3 - nNaNs] = 4;
    xwork[3 - nNaNs] = x[3];
    nNaNs++;
  } else {
    ib++;
    idx4[ib - 1] = 4;
    x4[ib - 1] = x[3];
    if (ib == 4) {
      ib = 3 - nNaNs;
      if (x4[0] <= x4[1]) {
        m = 1;
        bLen = 2;
      } else {
        m = 2;
        bLen = 1;
      }

      if (x4[2] <= x4[3]) {
        nPairs = 3;
        i4 = 4;
      } else {
        nPairs = 4;
        i4 = 3;
      }

      if (x4[m - 1] <= x4[nPairs - 1]) {
        if (x4[bLen - 1] <= x4[nPairs - 1]) {
          perm[0] = (int8_T)m;
          perm[1] = (int8_T)bLen;
          perm[2] = (int8_T)nPairs;
          perm[3] = (int8_T)i4;
        } else if (x4[bLen - 1] <= x4[i4 - 1]) {
          perm[0] = (int8_T)m;
          perm[1] = (int8_T)nPairs;
          perm[2] = (int8_T)bLen;
          perm[3] = (int8_T)i4;
        } else {
          perm[0] = (int8_T)m;
          perm[1] = (int8_T)nPairs;
          perm[2] = (int8_T)i4;
          perm[3] = (int8_T)bLen;
        }
      } else if (x4[m - 1] <= x4[i4 - 1]) {
        if (x4[bLen - 1] <= x4[i4 - 1]) {
          perm[0] = (int8_T)nPairs;
          perm[1] = (int8_T)m;
          perm[2] = (int8_T)bLen;
          perm[3] = (int8_T)i4;
        } else {
          perm[0] = (int8_T)nPairs;
          perm[1] = (int8_T)m;
          perm[2] = (int8_T)i4;
          perm[3] = (int8_T)bLen;
        }
      } else {
        perm[0] = (int8_T)nPairs;
        perm[1] = (int8_T)i4;
        perm[2] = (int8_T)m;
        perm[3] = (int8_T)bLen;
      }

      idx[ib - 3] = idx4[perm[0] - 1];
      idx[ib - 2] = idx4[perm[1] - 1];
      idx[ib - 1] = idx4[perm[2] - 1];
      idx[ib] = idx4[perm[3] - 1];
      x[ib - 3] = x4[perm[0] - 1];
      x[ib - 2] = x4[perm[1] - 1];
      x[ib - 1] = x4[perm[2] - 1];
      x[ib] = x4[perm[3] - 1];
      ib = 0;
    }
  }

  if (ib > 0) {
    perm[1] = 0;
    perm[2] = 0;
    perm[3] = 0;
    if (ib == 1) {
      perm[0] = 1;
    } else if (ib == 2) {
      if (x4[0] <= x4[1]) {
        perm[0] = 1;
        perm[1] = 2;
      } else {
        perm[0] = 2;
        perm[1] = 1;
      }
    } else if (x4[0] <= x4[1]) {
      if (x4[1] <= x4[2]) {
        perm[0] = 1;
        perm[1] = 2;
        perm[2] = 3;
      } else if (x4[0] <= x4[2]) {
        perm[0] = 1;
        perm[1] = 3;
        perm[2] = 2;
      } else {
        perm[0] = 3;
        perm[1] = 1;
        perm[2] = 2;
      }
    } else if (x4[0] <= x4[2]) {
      perm[0] = 2;
      perm[1] = 1;
      perm[2] = 3;
    } else if (x4[1] <= x4[2]) {
      perm[0] = 2;
      perm[1] = 3;
      perm[2] = 1;
    } else {
      perm[0] = 3;
      perm[1] = 2;
      perm[2] = 1;
    }

    for (m = 4; m - 3 <= ib; m++) {
      idx[(m - nNaNs) - ib] = idx4[perm[m - 4] - 1];
      x[(m - nNaNs) - ib] = x4[perm[m - 4] - 1];
    }
  }

  m = nNaNs >> 1;
  for (ib = 1; ib <= m; ib++) {
    bLen = idx[(ib - nNaNs) + 3];
    idx[(ib - nNaNs) + 3] = idx[4 - ib];
    idx[4 - ib] = bLen;
    x[(ib - nNaNs) + 3] = xwork[4 - ib];
    x[4 - ib] = xwork[(ib - nNaNs) + 3];
  }

  if ((nNaNs & 1) != 0) {
    x[(m - nNaNs) + 4] = xwork[(m - nNaNs) + 4];
  }
}

/* Function for MATLAB Function: '<Root>/Calculate Target Points' */
static void mirageTrackingRobot_orderPoints(const real_T p[8], real_T new_p[8])
{
  real_T x[4];
  int32_T iidx[4];
  int32_T indx[4];
  int32_T indx_idx_0;
  int32_T indx_idx_3;
  x[0] = p[0];
  x[1] = p[2];
  x[2] = p[4];
  x[3] = p[6];
  mirageTrackingRobot_eml_sort(x, iidx);
  indx_idx_0 = iidx[0];
  indx_idx_3 = iidx[3];
  x[0] = 480.0 - p[1];
  x[1] = 480.0 - p[3];
  x[2] = 480.0 - p[5];
  x[3] = 480.0 - p[7];
  mirageTrackingRobot_eml_sort(x, iidx);
  x[2] = iidx[2];
  x[3] = iidx[3];
  indx[0] = indx_idx_0;
  indx[1] = indx_idx_3;
  indx[2] = (int32_T)x[2];
  indx[3] = (int32_T)x[3];
  for (indx_idx_0 = 0; indx_idx_0 < 4; indx_idx_0++) {
    new_p[indx_idx_0 << 1] = p[(indx[indx_idx_0] - 1) << 1];
    new_p[1 + (indx_idx_0 << 1)] = p[((indx[indx_idx_0] - 1) << 1) + 1];
  }
}

/* Function for MATLAB Function: '<Root>/Estimate Pose Error' */
static void mirageTrackingRobot_invNxN(const real_T x[16], real_T y[16])
{
  int8_T p[4];
  real_T A[16];
  int8_T ipiv[4];
  int32_T b_j;
  int32_T b_c;
  int32_T ix;
  real_T smax;
  real_T s;
  int32_T jy;
  int32_T c_ix;
  int32_T ijA;
  int32_T jBcol;
  int32_T kAcol;
  for (b_j = 0; b_j < 16; b_j++) {
    y[b_j] = 0.0;
    A[b_j] = x[b_j];
  }

  ipiv[0] = 1;
  ipiv[1] = 2;
  ipiv[2] = 3;
  for (b_j = 0; b_j < 3; b_j++) {
    b_c = b_j * 5;
    jy = 0;
    ix = b_c;
    smax = fabs(A[b_c]);
    for (jBcol = 2; jBcol <= 4 - b_j; jBcol++) {
      ix++;
      s = fabs(A[ix]);
      if (s > smax) {
        jy = jBcol - 1;
        smax = s;
      }
    }

    if (A[b_c + jy] != 0.0) {
      if (jy != 0) {
        ipiv[b_j] = (int8_T)((b_j + jy) + 1);
        kAcol = b_j;
        jBcol = b_j + jy;
        smax = A[kAcol];
        A[kAcol] = A[jBcol];
        A[jBcol] = smax;
        kAcol += 4;
        jBcol += 4;
        smax = A[kAcol];
        A[kAcol] = A[jBcol];
        A[jBcol] = smax;
        kAcol += 4;
        jBcol += 4;
        smax = A[kAcol];
        A[kAcol] = A[jBcol];
        A[jBcol] = smax;
        kAcol += 4;
        jBcol += 4;
        smax = A[kAcol];
        A[kAcol] = A[jBcol];
        A[jBcol] = smax;
      }

      ix = (b_c - b_j) + 4;
      for (jBcol = b_c + 1; jBcol + 1 <= ix; jBcol++) {
        A[jBcol] /= A[b_c];
      }
    }

    jBcol = b_c;
    jy = b_c + 4;
    for (kAcol = 1; kAcol <= 3 - b_j; kAcol++) {
      smax = A[jy];
      if (A[jy] != 0.0) {
        c_ix = b_c + 1;
        ix = (jBcol - b_j) + 8;
        for (ijA = 5 + jBcol; ijA + 1 <= ix; ijA++) {
          A[ijA] += A[c_ix] * -smax;
          c_ix++;
        }
      }

      jy += 4;
      jBcol += 4;
    }
  }

  p[0] = 1;
  p[1] = 2;
  p[2] = 3;
  p[3] = 4;
  if (ipiv[0] > 1) {
    jBcol = p[ipiv[0] - 1];
    p[ipiv[0] - 1] = 1;
    p[0] = (int8_T)jBcol;
  }

  if (ipiv[1] > 2) {
    jBcol = p[ipiv[1] - 1];
    p[ipiv[1] - 1] = p[1];
    p[1] = (int8_T)jBcol;
  }

  if (ipiv[2] > 3) {
    jBcol = p[ipiv[2] - 1];
    p[ipiv[2] - 1] = p[2];
    p[2] = (int8_T)jBcol;
  }

  jBcol = p[0] - 1;
  y[(p[0] - 1) << 2] = 1.0;
  for (jy = 0; jy + 1 < 5; jy++) {
    if (y[(jBcol << 2) + jy] != 0.0) {
      for (kAcol = jy + 1; kAcol + 1 < 5; kAcol++) {
        y[kAcol + (jBcol << 2)] -= y[(jBcol << 2) + jy] * A[(jy << 2) + kAcol];
      }
    }
  }

  jBcol = p[1] - 1;
  y[1 + ((p[1] - 1) << 2)] = 1.0;
  for (jy = 1; jy + 1 < 5; jy++) {
    if (y[(jBcol << 2) + jy] != 0.0) {
      for (kAcol = jy + 1; kAcol + 1 < 5; kAcol++) {
        y[kAcol + (jBcol << 2)] -= y[(jBcol << 2) + jy] * A[(jy << 2) + kAcol];
      }
    }
  }

  jBcol = p[2] - 1;
  y[2 + ((p[2] - 1) << 2)] = 1.0;
  for (jy = 2; jy + 1 < 5; jy++) {
    if (y[(jBcol << 2) + jy] != 0.0) {
      for (kAcol = jy + 1; kAcol + 1 < 5; kAcol++) {
        y[kAcol + (jBcol << 2)] -= y[(jBcol << 2) + jy] * A[(jy << 2) + kAcol];
      }
    }
  }

  jBcol = p[3] - 1;
  y[3 + ((p[3] - 1) << 2)] = 1.0;
  for (jy = 3; jy + 1 < 5; jy++) {
    if (y[(jBcol << 2) + jy] != 0.0) {
      for (kAcol = jy + 1; kAcol + 1 < 5; kAcol++) {
        y[kAcol + (jBcol << 2)] -= y[(jBcol << 2) + jy] * A[(jy << 2) + kAcol];
      }
    }
  }

  for (ix = 0; ix < 4; ix++) {
    jBcol = ix << 2;
    if (y[3 + jBcol] != 0.0) {
      y[3 + jBcol] /= A[15];
      for (c_ix = 0; c_ix + 1 < 4; c_ix++) {
        y[c_ix + jBcol] -= y[3 + jBcol] * A[c_ix + 12];
      }
    }

    if (y[2 + jBcol] != 0.0) {
      y[2 + jBcol] /= A[10];
      for (c_ix = 0; c_ix + 1 < 3; c_ix++) {
        y[c_ix + jBcol] -= y[2 + jBcol] * A[c_ix + 8];
      }
    }

    if (y[1 + jBcol] != 0.0) {
      y[1 + jBcol] /= A[5];
      for (c_ix = 0; c_ix + 1 < 2; c_ix++) {
        y[c_ix + jBcol] -= y[1 + jBcol] * A[c_ix + 4];
      }
    }

    if (y[jBcol] != 0.0) {
      y[jBcol] /= A[0];
    }
  }
}

/* Function for MATLAB Function: '<Root>/Estimate Pose Error' */
static void mirageTra_transformToImageSpace(const real_T S_P[16], real_T
  Cam_E_S[16], const real_T K[16], real_T p[8], real_T P[16], real_T M[16])
{
  real_T Cam_E_S_0[16];
  int32_T i;
  int32_T i_0;

  /*  This function takes the 3D target points in robot space and transforms  */
  /*  them into 2D image space.  */
  /*  */
  /*  S_P     :(4x4) Desired position of the 3 target points in Robot Space  */
  /*  Cam_E_S :(4x4) Transformation Matrix (from robot space to camera space) */
  /*  K       :(4x4) Intrinsic Parameters Matrix for specified camera */
  /*  */
  /*  P     :(4x4) 3 Target points in 3D camera space */
  /*  p     :(2x4) 3 Target points in 2D image space */
  for (i = 0; i < 4; i++) {
    Cam_E_S_0[i << 2] = Cam_E_S[(i << 2) + 1];
    Cam_E_S_0[1 + (i << 2)] = Cam_E_S[(i << 2) + 2];
    Cam_E_S_0[2 + (i << 2)] = Cam_E_S[i << 2];
    Cam_E_S_0[3 + (i << 2)] = Cam_E_S[(i << 2) + 3];
  }

  for (i = 0; i < 4; i++) {
    Cam_E_S[i << 2] = Cam_E_S_0[i << 2];
    Cam_E_S[1 + (i << 2)] = Cam_E_S_0[(i << 2) + 1];
    Cam_E_S[2 + (i << 2)] = Cam_E_S_0[(i << 2) + 2];
    Cam_E_S[3 + (i << 2)] = Cam_E_S_0[(i << 2) + 3];
  }

  Cam_E_S[0] = -Cam_E_S[0];
  Cam_E_S[4] = -Cam_E_S[4];
  Cam_E_S[8] = -Cam_E_S[8];
  Cam_E_S[12] = -Cam_E_S[12];

  /* transformation matrix */
  for (i = 0; i < 4; i++) {
    for (i_0 = 0; i_0 < 4; i_0++) {
      M[i + (i_0 << 2)] = 0.0;
      M[i + (i_0 << 2)] += Cam_E_S[i_0 << 2] * K[i];
      M[i + (i_0 << 2)] += Cam_E_S[(i_0 << 2) + 1] * K[i + 4];
      M[i + (i_0 << 2)] += Cam_E_S[(i_0 << 2) + 2] * K[i + 8];
      M[i + (i_0 << 2)] += Cam_E_S[(i_0 << 2) + 3] * K[i + 12];
    }
  }

  /* transform into camera space (3D) */
  for (i = 0; i < 4; i++) {
    for (i_0 = 0; i_0 < 4; i_0++) {
      P[i + (i_0 << 2)] = 0.0;
      P[i + (i_0 << 2)] += S_P[i_0 << 2] * M[i];
      P[i + (i_0 << 2)] += S_P[(i_0 << 2) + 1] * M[i + 4];
      P[i + (i_0 << 2)] += S_P[(i_0 << 2) + 2] * M[i + 8];
      P[i + (i_0 << 2)] += S_P[(i_0 << 2) + 3] * M[i + 12];
    }
  }

  /*  Project 3D point into 2D image space */
  p[0] = P[0] / P[2];
  p[2] = P[4] / P[6];
  p[4] = P[8] / P[10];
  p[6] = P[12] / P[14];
  p[1] = P[1] / P[2];
  p[3] = P[5] / P[6];
  p[5] = P[9] / P[10];
  p[7] = P[13] / P[14];
}

/* Function for MATLAB Function: '<Root>/Estimate Pose Error' */
static void mirageTrackingRobo_getEquations(const real_T Cam_PDes[16], const
  real_T S_PDes[16], const real_T Err[8], const real_T M[16], real_T equations
  [96], real_T constants[8])
{
  real_T n[12];
  real_T j;
  real_T O[12];
  int32_T i;
  real_T Cam_PDes_0;
  int32_T j_0;
  real_T S_PDes_0;
  real_T S_PDes_1;
  real_T Vx_idx_0;
  real_T Vx_idx_1;
  real_T Vx_idx_2;
  real_T Vy_idx_0;
  real_T Vy_idx_1;
  real_T Vy_idx_2;

  /*  This function takes the 3 desired target points in selected camera  */
  /*  and robot space, image errors for 3 points and M transformation matrix. */
  /*  Then computes the equation (6x12) for selected camera. */
  /*  */
  /*  Cam_PDes  :(4x4) Desired 3 target points in Selected Camera Space  */
  /*  S_PDes    :(4x4) Desired 3 target points in robot space */
  /*  Err       :(2x4) Image Errors of 3 target points of camera  */
  /*  M         :(4x4) Transformation matrix (Intrinsic & Extrinsic Parameters) */
  /*  */
  /*  equations :(6x12) 6 equations (3 Points x 2 errors) */
  /*  constants :(6x1) 6 constants for equations */
  memset(&equations[0], 0, 96U * sizeof(real_T));
  memset(&constants[0], 0, sizeof(real_T) << 3U);
  j = 1.0;
  for (i = 0; i < 4; i++) {
    /* number of points */
    Vy_idx_2 = Cam_PDes[(i << 2) + 2];
    Cam_PDes_0 = Cam_PDes[i << 2];
    n[0] = Vy_idx_2 * M[0] - Cam_PDes_0 * M[2];
    n[3] = Vy_idx_2 * M[4] - Cam_PDes_0 * M[6];
    n[6] = Vy_idx_2 * M[8] - Cam_PDes_0 * M[10];
    n[9] = Vy_idx_2 * M[12] - Cam_PDes_0 * M[14];
    Vy_idx_2 = Cam_PDes[(i << 2) + 2];
    Cam_PDes_0 = Cam_PDes[(i << 2) + 1];
    n[1] = Vy_idx_2 * M[1] - Cam_PDes_0 * M[2];
    n[4] = Vy_idx_2 * M[5] - Cam_PDes_0 * M[6];
    n[7] = Vy_idx_2 * M[9] - Cam_PDes_0 * M[10];
    n[10] = Vy_idx_2 * M[13] - Cam_PDes_0 * M[14];
    Vy_idx_2 = Cam_PDes[(i << 2) + 2];
    n[2] = Vy_idx_2 * M[2];
    n[5] = Vy_idx_2 * M[6];
    n[8] = Vy_idx_2 * M[10];
    n[11] = Vy_idx_2 * M[14];
    Vy_idx_2 = n[11];
    for (j_0 = 0; j_0 < 12; j_0++) {
      O[j_0] = n[j_0] / Vy_idx_2;
    }

    Vy_idx_2 = Err[i << 1];
    Vx_idx_0 = O[0] - Vy_idx_2 * O[2];
    Vx_idx_1 = O[3] - Vy_idx_2 * O[5];
    Vx_idx_2 = O[6] - Vy_idx_2 * O[8];
    Vy_idx_2 = Err[(i << 1) + 1];
    Vy_idx_0 = O[1] - Vy_idx_2 * O[2];
    Vy_idx_1 = O[4] - Vy_idx_2 * O[5];
    Vy_idx_2 = O[7] - Vy_idx_2 * O[8];
    j_0 = (int32_T)j - 1;
    Cam_PDes_0 = S_PDes[i << 2];
    S_PDes_0 = S_PDes[(i << 2) + 1];
    S_PDes_1 = S_PDes[(i << 2) + 2];
    equations[j_0] = Vx_idx_0 * Cam_PDes_0;
    equations[j_0 + 8] = Vx_idx_1 * Cam_PDes_0;
    equations[j_0 + 16] = Vx_idx_2 * Cam_PDes_0;
    equations[j_0 + 24] = Vx_idx_0 * S_PDes_0;
    equations[j_0 + 32] = Vx_idx_1 * S_PDes_0;
    equations[j_0 + 40] = Vx_idx_2 * S_PDes_0;
    equations[j_0 + 48] = Vx_idx_0 * S_PDes_1;
    equations[j_0 + 56] = Vx_idx_1 * S_PDes_1;
    equations[j_0 + 64] = Vx_idx_2 * S_PDes_1;
    equations[j_0 + 72] = Vx_idx_0;
    equations[j_0 + 80] = Vx_idx_1;
    equations[j_0 + 88] = Vx_idx_2;
    j_0 = (int32_T)(j + 1.0) - 1;
    Cam_PDes_0 = S_PDes[i << 2];
    S_PDes_0 = S_PDes[(i << 2) + 1];
    S_PDes_1 = S_PDes[(i << 2) + 2];
    equations[j_0] = Vy_idx_0 * Cam_PDes_0;
    equations[j_0 + 8] = Vy_idx_1 * Cam_PDes_0;
    equations[j_0 + 16] = Vy_idx_2 * Cam_PDes_0;
    equations[j_0 + 24] = Vy_idx_0 * S_PDes_0;
    equations[j_0 + 32] = Vy_idx_1 * S_PDes_0;
    equations[j_0 + 40] = Vy_idx_2 * S_PDes_0;
    equations[j_0 + 48] = Vy_idx_0 * S_PDes_1;
    equations[j_0 + 56] = Vy_idx_1 * S_PDes_1;
    equations[j_0 + 64] = Vy_idx_2 * S_PDes_1;
    equations[j_0 + 72] = Vy_idx_0;
    equations[j_0 + 80] = Vy_idx_1;
    equations[j_0 + 88] = Vy_idx_2;
    constants[(int32_T)j - 1] = Err[i << 1] - O[9];
    constants[(int32_T)(j + 1.0) - 1] = Err[(i << 1) + 1] - O[10];
    j += 2.0;
  }
}

/* Function for MATLAB Function: '<Root>/Estimate Pose Error' */
static void mirageTrackingRobot_eml_xgetrf(real_T A[144], int32_T ipiv[12])
{
  int32_T j;
  int32_T c;
  int32_T idxmax;
  int32_T ix;
  real_T smax;
  real_T s;
  int32_T b_ix;
  int32_T jy;
  int32_T d;
  int32_T ijA;
  for (j = 0; j < 12; j++) {
    ipiv[j] = 1 + j;
  }

  for (j = 0; j < 11; j++) {
    c = j * 13;
    idxmax = 0;
    ix = c;
    smax = fabs(A[c]);
    for (jy = 2; jy <= 12 - j; jy++) {
      ix++;
      s = fabs(A[ix]);
      if (s > smax) {
        idxmax = jy - 1;
        smax = s;
      }
    }

    if (A[c + idxmax] != 0.0) {
      if (idxmax != 0) {
        ipiv[j] = (j + idxmax) + 1;
        b_ix = j;
        ix = j + idxmax;
        for (jy = 0; jy < 12; jy++) {
          smax = A[b_ix];
          A[b_ix] = A[ix];
          A[ix] = smax;
          b_ix += 12;
          ix += 12;
        }
      }

      ix = (c - j) + 12;
      for (jy = c + 1; jy + 1 <= ix; jy++) {
        A[jy] /= A[c];
      }
    }

    ix = c;
    jy = c + 12;
    for (idxmax = 1; idxmax <= 11 - j; idxmax++) {
      smax = A[jy];
      if (A[jy] != 0.0) {
        b_ix = c + 1;
        d = (ix - j) + 24;
        for (ijA = 13 + ix; ijA + 1 <= d; ijA++) {
          A[ijA] += A[b_ix] * -smax;
          b_ix++;
        }
      }

      jy += 12;
      ix += 12;
    }
  }
}

/* Function for MATLAB Function: '<Root>/Estimate Pose Error' */
static void mirageTrackingRo_solveEquations(const real_T A[192], const real_T B
  [16], real_T params[12])
{
  real_T y[144];
  real_T b_y[144];
  int8_T p[12];
  int32_T ipiv[12];
  int32_T b_k;
  int32_T jBcol;
  int32_T kAcol;
  int32_T c_k;
  int32_T b_i;
  real_T A_0[12];
  real_T params_0;

  /*  This function solves the equations as Ax-B=0. Gets A and B, computes x */
  /*  parameters */
  /*  A :(mxn) Equation Matrix  */
  /*  B :(mx1) Constants Vector  */
  /*  */
  /*  params :(nx1) Parameters Vector */
  for (b_k = 0; b_k < 12; b_k++) {
    for (jBcol = 0; jBcol < 12; jBcol++) {
      y[b_k + 12 * jBcol] = 0.0;
      for (c_k = 0; c_k < 16; c_k++) {
        y[b_k + 12 * jBcol] += A[(b_k << 4) + c_k] * A[(jBcol << 4) + c_k];
      }
    }
  }

  memset(&b_y[0], 0, 144U * sizeof(real_T));
  mirageTrackingRobot_eml_xgetrf(y, ipiv);
  for (b_k = 0; b_k < 12; b_k++) {
    p[b_k] = (int8_T)(1 + b_k);
  }

  for (b_k = 0; b_k < 11; b_k++) {
    if (ipiv[b_k] > 1 + b_k) {
      jBcol = p[ipiv[b_k] - 1];
      p[ipiv[b_k] - 1] = p[b_k];
      p[b_k] = (int8_T)jBcol;
    }
  }

  for (b_k = 0; b_k < 12; b_k++) {
    jBcol = p[b_k] - 1;
    b_y[b_k + 12 * (p[b_k] - 1)] = 1.0;
    for (c_k = b_k; c_k + 1 < 13; c_k++) {
      if (b_y[12 * jBcol + c_k] != 0.0) {
        for (kAcol = c_k + 1; kAcol + 1 < 13; kAcol++) {
          b_y[kAcol + 12 * jBcol] -= b_y[12 * jBcol + c_k] * y[12 * c_k + kAcol];
        }
      }
    }
  }

  for (b_k = 0; b_k < 12; b_k++) {
    jBcol = 12 * b_k;
    for (c_k = 11; c_k >= 0; c_k += -1) {
      kAcol = 12 * c_k;
      if (b_y[c_k + jBcol] != 0.0) {
        b_y[c_k + jBcol] /= y[c_k + kAcol];
        for (b_i = 0; b_i + 1 <= c_k; b_i++) {
          b_y[b_i + jBcol] -= b_y[c_k + jBcol] * y[b_i + kAcol];
        }
      }
    }
  }

  for (b_k = 0; b_k < 12; b_k++) {
    A_0[b_k] = 0.0;
    for (jBcol = 0; jBcol < 16; jBcol++) {
      params_0 = A_0[b_k];
      params_0 += A[(b_k << 4) + jBcol] * B[jBcol];
      A_0[b_k] = params_0;
    }
  }

  for (b_k = 0; b_k < 12; b_k++) {
    params[b_k] = 0.0;
    for (jBcol = 0; jBcol < 12; jBcol++) {
      params_0 = params[b_k];
      params_0 += b_y[12 * jBcol + b_k] * A_0[jBcol];
      params[b_k] = params_0;
    }
  }
}

/* Function for MATLAB Function: '<Root>/Estimate Pose Error' */
static void mirageTrackingRobot_visioMotor(const real_T L_p[8], const real_T
  R_p[8], const real_T S_PDes[16], real_T errParams[12])
{
  real_T L_pDes1[8];
  real_T L_PDes[16];
  real_T L_M[16];
  real_T R_pDes1[8];
  real_T R_M[16];
  real_T Left_Equations[96];
  real_T Right_Equations[96];
  static const real_T b[16] = { 0.99991934176358022, -0.011399574329943383,
    -0.0055999707307125618, 0.0, 0.01144253168026184, 0.99990488622657669,
    0.0076998031769008228, 0.0, 0.0055116636177242892, -0.0077632599668505118,
    0.999954675652278, 0.0, -1.1447190415288215, -6.6644855915866081,
    -0.045430887024426554, 1.0 };

  static const real_T c[16] = { 1412.45843318963, 0.0, 0.0, 0.0, 0.0,
    1444.55255979655, 0.0, 0.0, 606.056420829776, 448.719680077129, 1.0, 0.0,
    0.0, 0.0, 0.0, 1.0 };

  static const real_T d[16] = { 0.9999102213434, 2.710675443000345E-20,
    -0.013399598986266985, 0.0, -0.00015007237108723492, 0.99993728065563026,
    -0.011198760346869984, 0.0, 0.013398758572203748, 0.011199765846801945,
    0.99984750762988872, 0.0, -0.89079778302692991, 4.9496604198541689,
    -0.0461064841791679, 1.0 };

  static const real_T e[16] = { 1418.69905797363, 0.0, 0.0, 0.0, 0.0,
    1444.76712590545, 0.0, 0.0, 631.297389348331, 386.501819479235, 1.0, 0.0,
    0.0, 0.0, 0.0, 1.0 };

  real_T Left_Equations_0[192];
  real_T R_p_0[8];
  real_T L_p_0[8];
  real_T d_0[16];
  real_T b_0[16];
  int32_T i;

  /*  This function gets the desired positions of 4 3D target points in  */
  /*  robot space and 8 2D target points on the image for left and right  */
  /*  cameras. Then it computes the t parameters that contains direction  */
  /*  information to close desired position */
  /*  */
  /*  S_PDes :(3x3) Desired position of the 3 target points in Robot Space  */
  /*  L_p    :(2x3) Pixel values of the 3 target points on the left image */
  /*  R_p    :(2x3) Pixel values of the 3 target points on the right image */
  /*  */
  /*  errParams :(12x1) 9 Rotation and 3 Translation Parameters. Robot will use */
  /*  these t parameters and change its position into desired position */
  /*  Transformation Matrices (Extrinsic Parameters) */
  /* Transform from robot space to camera space */
  /*  Intrinsic Camera Parameters */
  /*  focal length of the lens (pixels) */
  /*  coordinates of the optical axis (pixels) */
  /*  focal length of the lens (pixels) */
  /*  coordinates of the optical axis (pixels) */
  /*  Project Points into the camera and image space */
  for (i = 0; i < 16; i++) {
    b_0[i] = b[i];
    d_0[i] = d[i];
  }

  mirageTra_transformToImageSpace(S_PDes, b_0, c, L_pDes1, L_PDes, L_M);
  mirageTra_transformToImageSpace(S_PDes, d_0, e, R_pDes1, b_0, R_M);

  /*  Pixel Error */
  /*  Left and Right Camera Equations (2 equations for 1 point) */
  for (i = 0; i < 8; i++) {
    L_p_0[i] = L_p[i] - L_pDes1[i];
    R_p_0[i] = R_p[i] - R_pDes1[i];
  }

  mirageTrackingRobo_getEquations(L_PDes, S_PDes, L_p_0, L_M, Left_Equations,
    R_pDes1);
  mirageTrackingRobo_getEquations(b_0, S_PDes, R_p_0, R_M, Right_Equations,
    L_pDes1);
  for (i = 0; i < 12; i++) {
    memcpy(&Left_Equations_0[i << 4], &Left_Equations[i << 3], sizeof(real_T) <<
           3U);
  }

  for (i = 0; i < 12; i++) {
    memcpy(&Left_Equations_0[(i << 4) + 8], &Right_Equations[i << 3], sizeof
           (real_T) << 3U);
  }

  memcpy(&b_0[0], &R_pDes1[0], sizeof(real_T) << 3U);
  memcpy(&b_0[8], &L_pDes1[0], sizeof(real_T) << 3U);
  mirageTrackingRo_solveEquations(Left_Equations_0, b_0, errParams);
}

real_T rt_atan2d_snf(real_T u0, real_T u1)
{
  real_T y;
  int32_T tmp;
  int32_T tmp_0;
  if (rtIsNaN(u0) || rtIsNaN(u1)) {
    y = (rtNaN);
  } else if (rtIsInf(u0) && rtIsInf(u1)) {
    if (u1 > 0.0) {
      tmp = 1;
    } else {
      tmp = -1;
    }

    if (u0 > 0.0) {
      tmp_0 = 1;
    } else {
      tmp_0 = -1;
    }

    y = atan2(tmp_0, tmp);
  } else if (u1 == 0.0) {
    if (u0 > 0.0) {
      y = RT_PI / 2.0;
    } else if (u0 < 0.0) {
      y = -(RT_PI / 2.0);
    } else {
      y = 0.0;
    }
  } else {
    y = atan2(u0, u1);
  }

  return y;
}

/* Model output function */
static void mirageTrackingRobot_output(void)
{
  int32_T ml;
  visioncodegen_BlobAnalysis_mi_T *obj;
  static const int16_T b[8] = { -1, 961, 962, 963, 1, -961, -962, -963 };

  real_T robotPoseDes_x;
  real_T robotPoseDes_y;
  real_T robotPoseDes_z;
  real_T robotPoseDes_phi;
  real_T robotPoseDes_theta;
  real_T robotPoseDes_psi;
  real_T world2robotDes[16];
  real_T errParams[12];
  real_T poseErrorMat[16];
  real_T world2robot[16];
  real_T y[12];
  real_T theta;
  static const int8_T b_0[16] = { 1, 0, 0, 0, 0, 1, 0, 0, 0, 0, 1, 0, 0, 0, 0, 1
  };

  int32_T i;
  real_T world2robotDes_0[16];
  real_T tmp[8];
  int32_T i_0;
  real_T tmp_0[16];
  real_T tmp_1[16];
  real_T tmp_2[16];
  int32_T loop_ub;
  int32_T unusedU0_data[50];
  int32_T unusedU0_sizes[2];
  real_T l_center_data[100];
  int32_T l_center_sizes[2];
  int32_T unusedU1_data[200];
  int32_T unusedU1_sizes[2];
  real_T r_center_data[100];
  int32_T r_center_sizes[2];
  real_T r_center_data_0[2];
  uint32_T qY;
  int32_T tmp_3;
  uint8_T tmp_4;
  uint8_T tmp_5;

  /* ok to acquire for <S3>/S-Function */
  mirageTrackingRobot_DW.SFunction_IWORK.AcquireOK = 1;

  /* Level2 S-Function Block: '<Root>/From Left Camera' (xpcusbvideoin) */
  {
    SimStruct *rts = mirageTrackingRobot_M->childSfunctions[0];
    sfcnOutputs(rts, 0);
  }

  /* Level2 S-Function Block: '<Root>/From Right Camera' (xpcusbvideoin) */
  {
    SimStruct *rts = mirageTrackingRobot_M->childSfunctions[1];
    sfcnOutputs(rts, 0);
  }

  /* MATLAB Function: '<Root>/Calculate Target Points' */
  /* MATLAB Function 'Calculate Target Points': '<S1>:1' */
  /* '<S1>:1:4' */
  /* '<S1>:1:5' */
  for (i = 0; i < 8; i++) {
    mirageTrackingRobot_B.L_p[i] = 0.0;
    mirageTrackingRobot_B.R_p[i] = 0.0;
  }

  /* input left image(l_rgb), right image(r_rgb), Pose target position  */
  /* define image variables for processing */
  /* '<S1>:1:17' */
  /*  converts RGB to CMY color space  */
  /* '<S1>:1:18' */
  for (i_0 = 0; i_0 < 3686400; i_0++) {
    mirageTrackingRobot_B.l_rgb1_m[i_0] = (uint8_T)(255U -
      mirageTrackingRobot_B.FromLeftCamera[i_0]);
    mirageTrackingRobot_B.r_rgb1[i_0] = (uint8_T)(255U -
      mirageTrackingRobot_B.FromRightCamera[i_0]);
  }

  /*  converts RGB to CMY color space  */
  /* '<S1>:1:20' */
  /*  rgb2gray(rgb) ;convert rgb to grayscale  */
  /* '<S1>:1:21' */
  /*  rgb2gray(rgb) ;convert rgb to grayscale */
  /* '<S1>:1:23' */
  /*  imsubtract(rgb1(:,:,3),gray) */
  /* '<S1>:1:24' */
  /*  imsubtract(rgb1(:,:,3),gray)  */
  /* '<S1>:1:26' */
  for (i_0 = 0; i_0 < 1280; i_0++) {
    for (i = 0; i < 960; i++) {
      ml = mirageTrackingRobot_B.l_rgb1_m[(960 * i_0 + i) + 2457600];
      loop_ub = (int32_T)rt_roundd_snf((real_T)mirageTrackingRobot_B.l_rgb1_m
        [960 * i_0 + i] * 0.2989);
      tmp_4 = (uint8_T)loop_ub;
      loop_ub = (int32_T)rt_roundd_snf((real_T)mirageTrackingRobot_B.l_rgb1_m
        [(960 * i_0 + i) + 1228800] * 0.587);
      tmp_5 = (uint8_T)loop_ub;
      loop_ub = (int32_T)((uint32_T)tmp_4 + tmp_5);
      tmp_3 = (int32_T)rt_roundd_snf((real_T)mirageTrackingRobot_B.l_rgb1_m[(960
        * i_0 + i) + 2457600] * 0.114);
      tmp_4 = (uint8_T)tmp_3;
      loop_ub = (int32_T)((uint32_T)loop_ub + tmp_4);
      qY = (uint32_T)ml - loop_ub;
      if (qY > (uint32_T)ml) {
        qY = 0U;
      }

      loop_ub = (int32_T)qY;
      mirageTrackingRobot_B.l_rgb1[i + 960 * i_0] = (uint8_T)loop_ub;
    }
  }

  mirageTrackingRobot_medfilt2(mirageTrackingRobot_B.l_rgb1,
    mirageTrackingRobot_B.l_rgb2);

  /*  medfilt2(rgb2,[3 3]); */
  /* '<S1>:1:27' */
  for (i_0 = 0; i_0 < 1280; i_0++) {
    for (i = 0; i < 960; i++) {
      ml = mirageTrackingRobot_B.r_rgb1[(960 * i_0 + i) + 2457600];
      loop_ub = (int32_T)rt_roundd_snf((real_T)mirageTrackingRobot_B.r_rgb1[960 *
        i_0 + i] * 0.2989);
      tmp_4 = (uint8_T)loop_ub;
      loop_ub = (int32_T)rt_roundd_snf((real_T)mirageTrackingRobot_B.r_rgb1[(960
        * i_0 + i) + 1228800] * 0.587);
      tmp_5 = (uint8_T)loop_ub;
      loop_ub = (int32_T)((uint32_T)tmp_4 + tmp_5);
      tmp_3 = (int32_T)rt_roundd_snf((real_T)mirageTrackingRobot_B.r_rgb1[(960 *
        i_0 + i) + 2457600] * 0.114);
      tmp_4 = (uint8_T)tmp_3;
      loop_ub = (int32_T)((uint32_T)loop_ub + tmp_4);
      qY = (uint32_T)ml - loop_ub;
      if (qY > (uint32_T)ml) {
        qY = 0U;
      }

      loop_ub = (int32_T)qY;
      mirageTrackingRobot_B.l_rgb1[i + 960 * i_0] = (uint8_T)loop_ub;
    }
  }

  mirageTrackingRobot_medfilt2(mirageTrackingRobot_B.l_rgb1,
    mirageTrackingRobot_B.r_rgb2);

  /*  medfilt2(rgb2,[3 3]); */
  /* '<S1>:1:29' */
  /* '<S1>:1:30' */
  /* '<S1>:1:32' */
  /* '<S1>:1:33' */
  /* finds yellow regions in image using BlobAnalysis. */
  obj = &mirageTrackingRobot_B.hblob;
  mirageTrackingRobot_B.hblob.isInitialized = 0;
  mirageTrackingRobot_B.hblob.NoTuningBeforeLockingCodeGenError = true;

  /* System object Constructor function: vision.BlobAnalysis */
  for (i = 0; i < 8; i++) {
    obj->cSFunObject.P0_WALKER_RTP[i] = b[i];
  }

  obj->cSFunObject.P1_MINAREA_RTP = 0U;
  obj->cSFunObject.P2_MAXAREA_RTP = MAX_uint32_T;
  obj->cSFunObject.P1_MINAREA_RTP = 0U;
  obj->cSFunObject.P2_MAXAREA_RTP = MAX_uint32_T;
  mirageTrackingRobot_B.hblob.NoTuningBeforeLockingCodeGenError = false;
  mirageTrackingRobot_myMat2gray(mirageTrackingRobot_B.l_rgb2,
    mirageTrackingRobot_B.dv0);
  for (i_0 = 0; i_0 < 1228800; i_0++) {
    mirageTrackingRobot_B.bv0[i_0] = (mirageTrackingRobot_B.dv0[i_0] > 0.6);
  }

  mirageTrackingR_SystemCore_step(&mirageTrackingRobot_B.hblob,
    mirageTrackingRobot_B.bv0, unusedU0_data, unusedU0_sizes, l_center_data,
    l_center_sizes, unusedU1_data, unusedU1_sizes);

  /*  get [x y] is a centroid of region */
  mirageTrackingRobot_myMat2gray(mirageTrackingRobot_B.r_rgb2,
    mirageTrackingRobot_B.dv0);
  for (i_0 = 0; i_0 < 1228800; i_0++) {
    mirageTrackingRobot_B.bv0[i_0] = (mirageTrackingRobot_B.dv0[i_0] > 0.6);
  }

  mirageTrackin_SystemCore_step_i(&mirageTrackingRobot_B.hblob,
    mirageTrackingRobot_B.bv0, unusedU0_data, unusedU0_sizes, r_center_data,
    r_center_sizes, unusedU1_data, unusedU1_sizes);

  /*  get [x y] is a centroid of region */
  /* Send Left Camera Points */
  /* '<S1>:1:42' */
  ml = l_center_sizes[0];
  if (l_center_sizes[0] > 4) {
    /* '<S1>:1:43' */
    /* '<S1>:1:44' */
    ml = 4;
  }

  /* '<S1>:1:46' */
  for (i = 0; i < ml; i++) {
    /* '<S1>:1:46' */
    /* '<S1>:1:47' */
    loop_ub = l_center_sizes[1];
    for (i_0 = 0; i_0 < loop_ub; i_0++) {
      r_center_data_0[i_0] = l_center_data[l_center_sizes[0] * i_0 + i];
    }

    mirageTrackingRobot_B.L_p[i << 1] = r_center_data_0[0];
    mirageTrackingRobot_B.L_p[1 + (i << 1)] = r_center_data_0[1];

    /* '<S1>:1:46' */
  }

  /* Send Right Camera Points */
  /* '<S1>:1:51' */
  ml = r_center_sizes[0];
  if (r_center_sizes[0] > 4) {
    /* '<S1>:1:52' */
    /* '<S1>:1:53' */
    ml = 4;
  }

  /* '<S1>:1:55' */
  for (i = 0; i < ml; i++) {
    /* '<S1>:1:55' */
    /* '<S1>:1:56' */
    loop_ub = r_center_sizes[1];
    for (i_0 = 0; i_0 < loop_ub; i_0++) {
      r_center_data_0[i_0] = r_center_data[r_center_sizes[0] * i_0 + i];
    }

    mirageTrackingRobot_B.R_p[i << 1] = r_center_data_0[0];
    mirageTrackingRobot_B.R_p[1 + (i << 1)] = r_center_data_0[1];

    /* '<S1>:1:55' */
  }

  /* '<S1>:1:59' */
  memcpy(&tmp[0], &mirageTrackingRobot_B.L_p[0], sizeof(real_T) << 3U);
  mirageTrackingRobot_orderPoints(tmp, mirageTrackingRobot_B.L_p);

  /* '<S1>:1:60' */
  memcpy(&tmp[0], &mirageTrackingRobot_B.R_p[0], sizeof(real_T) << 3U);
  mirageTrackingRobot_orderPoints(tmp, mirageTrackingRobot_B.R_p);

  /*      coder.extrinsic('imshow','plot'); */
  /*      imshow(l_rgb2);hold on; */
  /*      plot(L_p(1,:),L_p(2,:),'ro'); */
  /* '<S1>:1:66' */
  mirageTrackingRobot_B.L_p[1] = 960.0 - mirageTrackingRobot_B.L_p[1];
  mirageTrackingRobot_B.L_p[3] = 960.0 - mirageTrackingRobot_B.L_p[3];
  mirageTrackingRobot_B.L_p[5] = 960.0 - mirageTrackingRobot_B.L_p[5];
  mirageTrackingRobot_B.L_p[7] = 960.0 - mirageTrackingRobot_B.L_p[7];

  /* '<S1>:1:67' */
  mirageTrackingRobot_B.R_p[1] = 960.0 - mirageTrackingRobot_B.R_p[1];
  mirageTrackingRobot_B.R_p[3] = 960.0 - mirageTrackingRobot_B.R_p[3];
  mirageTrackingRobot_B.R_p[5] = 960.0 - mirageTrackingRobot_B.R_p[5];
  mirageTrackingRobot_B.R_p[7] = 960.0 - mirageTrackingRobot_B.R_p[7];

  /* End of MATLAB Function: '<Root>/Calculate Target Points' */

  /* MATLAB Function: '<Root>/Estimate Pose Error' incorporates:
   *  Constant: '<Root>/Desired Robot Pose'
   *  Constant: '<Root>/Object Model'
   */
  /* MATLAB Function 'Estimate Pose Error': '<S2>:1' */
  /* Desired Robot Pose for current step qr() */
  /* '<S2>:1:5' */
  robotPoseDes_x = mirageTrackingRobot_P.DesiredRobotPose_Value[0];

  /* '<S2>:1:6' */
  robotPoseDes_y = mirageTrackingRobot_P.DesiredRobotPose_Value[1];

  /* '<S2>:1:7' */
  robotPoseDes_z = mirageTrackingRobot_P.DesiredRobotPose_Value[2];

  /* '<S2>:1:8' */
  robotPoseDes_phi = mirageTrackingRobot_P.DesiredRobotPose_Value[3];

  /* '<S2>:1:9' */
  robotPoseDes_theta = mirageTrackingRobot_P.DesiredRobotPose_Value[4];

  /* '<S2>:1:10' */
  robotPoseDes_psi = mirageTrackingRobot_P.DesiredRobotPose_Value[5];

  /* Transform target points from I space into Desired Robot Space */
  /* '<S2>:1:13' */
  world2robotDes_0[0] = 1.0;
  world2robotDes_0[4] = 0.0;
  world2robotDes_0[8] = 0.0;
  world2robotDes_0[12] = mirageTrackingRobot_P.DesiredRobotPose_Value[0];
  world2robotDes_0[1] = 0.0;
  world2robotDes_0[5] = 1.0;
  world2robotDes_0[9] = 0.0;
  world2robotDes_0[13] = mirageTrackingRobot_P.DesiredRobotPose_Value[1];
  world2robotDes_0[2] = 0.0;
  world2robotDes_0[6] = 0.0;
  world2robotDes_0[10] = 1.0;
  world2robotDes_0[14] = mirageTrackingRobot_P.DesiredRobotPose_Value[2];
  world2robotDes_0[3] = 0.0;
  world2robotDes_0[7] = 0.0;
  world2robotDes_0[11] = 0.0;
  world2robotDes_0[15] = 1.0;
  world2robotDes[0] = 1.0;
  world2robotDes[4] = 0.0;
  world2robotDes[8] = 0.0;
  world2robotDes[12] = 0.0;
  world2robotDes[1] = 0.0;
  world2robotDes[5] = cos(mirageTrackingRobot_P.DesiredRobotPose_Value[3]);
  world2robotDes[9] = -sin(mirageTrackingRobot_P.DesiredRobotPose_Value[3]);
  world2robotDes[13] = 0.0;
  world2robotDes[2] = 0.0;
  world2robotDes[6] = sin(mirageTrackingRobot_P.DesiredRobotPose_Value[3]);
  world2robotDes[10] = cos(mirageTrackingRobot_P.DesiredRobotPose_Value[3]);
  world2robotDes[14] = 0.0;
  world2robotDes[3] = 0.0;
  world2robotDes[7] = 0.0;
  world2robotDes[11] = 0.0;
  world2robotDes[15] = 1.0;
  for (i_0 = 0; i_0 < 4; i_0++) {
    for (i = 0; i < 4; i++) {
      tmp_0[i_0 + (i << 2)] = 0.0;
      tmp_0[i_0 + (i << 2)] += world2robotDes[i << 2] * world2robotDes_0[i_0];
      tmp_0[i_0 + (i << 2)] += world2robotDes[(i << 2) + 1] *
        world2robotDes_0[i_0 + 4];
      tmp_0[i_0 + (i << 2)] += world2robotDes[(i << 2) + 2] *
        world2robotDes_0[i_0 + 8];
      tmp_0[i_0 + (i << 2)] += world2robotDes[(i << 2) + 3] *
        world2robotDes_0[i_0 + 12];
    }
  }

  tmp_1[0] = cos(mirageTrackingRobot_P.DesiredRobotPose_Value[4]);
  tmp_1[4] = 0.0;
  tmp_1[8] = sin(mirageTrackingRobot_P.DesiredRobotPose_Value[4]);
  tmp_1[12] = 0.0;
  tmp_1[1] = 0.0;
  tmp_1[5] = 1.0;
  tmp_1[9] = 0.0;
  tmp_1[13] = 0.0;
  tmp_1[2] = -sin(mirageTrackingRobot_P.DesiredRobotPose_Value[4]);
  tmp_1[6] = 0.0;
  tmp_1[10] = cos(mirageTrackingRobot_P.DesiredRobotPose_Value[4]);
  tmp_1[14] = 0.0;
  tmp_1[3] = 0.0;
  tmp_1[7] = 0.0;
  tmp_1[11] = 0.0;
  tmp_1[15] = 1.0;
  for (i_0 = 0; i_0 < 4; i_0++) {
    for (i = 0; i < 4; i++) {
      world2robotDes_0[i_0 + (i << 2)] = 0.0;
      world2robotDes_0[i_0 + (i << 2)] += tmp_1[i << 2] * tmp_0[i_0];
      world2robotDes_0[i_0 + (i << 2)] += tmp_1[(i << 2) + 1] * tmp_0[i_0 + 4];
      world2robotDes_0[i_0 + (i << 2)] += tmp_1[(i << 2) + 2] * tmp_0[i_0 + 8];
      world2robotDes_0[i_0 + (i << 2)] += tmp_1[(i << 2) + 3] * tmp_0[i_0 + 12];
    }
  }

  tmp_2[0] = cos(mirageTrackingRobot_P.DesiredRobotPose_Value[5]);
  tmp_2[4] = -sin(mirageTrackingRobot_P.DesiredRobotPose_Value[5]);
  tmp_2[8] = 0.0;
  tmp_2[12] = 0.0;
  tmp_2[1] = sin(mirageTrackingRobot_P.DesiredRobotPose_Value[5]);
  tmp_2[5] = cos(mirageTrackingRobot_P.DesiredRobotPose_Value[5]);
  tmp_2[9] = 0.0;
  tmp_2[13] = 0.0;
  tmp_2[2] = 0.0;
  tmp_2[6] = 0.0;
  tmp_2[10] = 1.0;
  tmp_2[14] = 0.0;
  tmp_2[3] = 0.0;
  tmp_2[7] = 0.0;
  tmp_2[11] = 0.0;
  tmp_2[15] = 1.0;
  for (i_0 = 0; i_0 < 4; i_0++) {
    for (i = 0; i < 4; i++) {
      poseErrorMat[i_0 + (i << 2)] = 0.0;
      poseErrorMat[i_0 + (i << 2)] += tmp_2[i << 2] * world2robotDes_0[i_0];
      poseErrorMat[i_0 + (i << 2)] += tmp_2[(i << 2) + 1] * world2robotDes_0[i_0
        + 4];
      poseErrorMat[i_0 + (i << 2)] += tmp_2[(i << 2) + 2] * world2robotDes_0[i_0
        + 8];
      poseErrorMat[i_0 + (i << 2)] += tmp_2[(i << 2) + 3] * world2robotDes_0[i_0
        + 12];
    }
  }

  mirageTrackingRobot_invNxN(poseErrorMat, world2robotDes);

  /* '<S2>:1:14' */
  /* Calculate the error parameters for current step */
  /* '<S2>:1:17' */
  for (i_0 = 0; i_0 < 4; i_0++) {
    for (i = 0; i < 4; i++) {
      world2robotDes_0[i_0 + (i << 2)] = 0.0;
      world2robotDes_0[i_0 + (i << 2)] +=
        mirageTrackingRobot_P.ObjectModel_Value[i << 2] * world2robotDes[i_0];
      world2robotDes_0[i_0 + (i << 2)] +=
        mirageTrackingRobot_P.ObjectModel_Value[(i << 2) + 1] *
        world2robotDes[i_0 + 4];
      world2robotDes_0[i_0 + (i << 2)] +=
        mirageTrackingRobot_P.ObjectModel_Value[(i << 2) + 2] *
        world2robotDes[i_0 + 8];
      world2robotDes_0[i_0 + (i << 2)] +=
        mirageTrackingRobot_P.ObjectModel_Value[(i << 2) + 3] *
        world2robotDes[i_0 + 12];
    }
  }

  mirageTrackingRobot_visioMotor(mirageTrackingRobot_B.L_p,
    mirageTrackingRobot_B.R_p, world2robotDes_0, errParams);

  /* '<S2>:1:19' */
  for (i_0 = 0; i_0 < 16; i_0++) {
    poseErrorMat[i_0] = b_0[i_0];
  }

  /* '<S2>:1:20' */
  memcpy(&y[0], &errParams[0], 12U * sizeof(real_T));
  for (i_0 = 0; i_0 < 4; i_0++) {
    poseErrorMat[i_0 << 2] = y[3 * i_0];
    poseErrorMat[1 + (i_0 << 2)] = y[3 * i_0 + 1];
    poseErrorMat[2 + (i_0 << 2)] = y[3 * i_0 + 2];
  }

  /* '<S2>:1:22' */
  for (i_0 = 0; i_0 < 4; i_0++) {
    for (i = 0; i < 4; i++) {
      world2robot[i_0 + (i << 2)] = 0.0;
      world2robot[i_0 + (i << 2)] += world2robotDes[i << 2] * poseErrorMat[i_0];
      world2robot[i_0 + (i << 2)] += world2robotDes[(i << 2) + 1] *
        poseErrorMat[i_0 + 4];
      world2robot[i_0 + (i << 2)] += world2robotDes[(i << 2) + 2] *
        poseErrorMat[i_0 + 8];
      world2robot[i_0 + (i << 2)] += world2robotDes[(i << 2) + 3] *
        poseErrorMat[i_0 + 12];
    }
  }

  /* '<S2>:1:24' */
  /*  This function get the rotation matrix and computes the euler angles. */
  /*  rotMat :(3x3) rotation matrix  */
  /*  */
  /*  roll  : angle phi */
  /*  pitch : angle theta */
  /*  yaw   : angle psi */
  theta = rt_atan2d_snf(-world2robot[2], sqrt(world2robot[0] * world2robot[0] +
    world2robot[1] * world2robot[1]));

  /* '<S2>:1:26' */
  mirageTrackingRobot_B.qtilde[0] = -world2robot[12] - robotPoseDes_x;
  mirageTrackingRobot_B.qtilde[1] = -world2robot[13] - robotPoseDes_y;
  mirageTrackingRobot_B.qtilde[2] = -world2robot[14] - robotPoseDes_z;
  mirageTrackingRobot_B.qtilde[3] = -rt_atan2d_snf(world2robot[6] / cos(theta),
    world2robot[10] / cos(theta)) - robotPoseDes_phi;
  mirageTrackingRobot_B.qtilde[4] = -theta - robotPoseDes_theta;
  mirageTrackingRobot_B.qtilde[5] = -rt_atan2d_snf(world2robot[1] / cos(theta),
    world2robot[0] / cos(theta)) - robotPoseDes_psi;

  /* End of MATLAB Function: '<Root>/Estimate Pose Error' */
}

/* Model update function */
static void mirageTrackingRobot_update(void)
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
  if (!(++mirageTrackingRobot_M->Timing.clockTick0)) {
    ++mirageTrackingRobot_M->Timing.clockTickH0;
  }

  mirageTrackingRobot_M->Timing.t[0] = mirageTrackingRobot_M->Timing.clockTick0 *
    mirageTrackingRobot_M->Timing.stepSize0 +
    mirageTrackingRobot_M->Timing.clockTickH0 *
    mirageTrackingRobot_M->Timing.stepSize0 * 4294967296.0;
}

/* Model initialize function */
static void mirageTrackingRobot_initialize(void)
{
  /* S-Function Block: <S3>/S-Function (scblock) */
  {
    int i;
    if ((i = rl32eScopeExists(1)) == 0) {
      if ((i = rl32eDefScope(1,2)) != 0) {
        printf("Error creating scope 1\n");
      } else {
        rl32eAddSignal(1, rl32eGetSignalNo("Estimate Pose Error/s1"));
        rl32eAddSignal(1, rl32eGetSignalNo("Estimate Pose Error/s2"));
        rl32eAddSignal(1, rl32eGetSignalNo("Estimate Pose Error/s3"));
        rl32eAddSignal(1, rl32eGetSignalNo("Estimate Pose Error/s4"));
        rl32eAddSignal(1, rl32eGetSignalNo("Estimate Pose Error/s5"));
        rl32eAddSignal(1, rl32eGetSignalNo("Estimate Pose Error/s6"));
        rl32eSetScope(1, 4, 3);
        rl32eSetScope(1, 5, 0);
        rl32eSetScope(1, 6, 1);
        rl32eSetScope(1, 0, 0);
        rl32eSetScope(1, 3, rl32eGetSignalNo("Estimate Pose Error/s1"));
        rl32eSetScope(1, 1, 0.0);
        rl32eSetScope(1, 2, 0);
        rl32eSetScope(1, 9, 0);
        rl32eSetTargetScope(1, 11, 0.0);
        rl32eSetTargetScope(1, 10, 0.0);
        xpceScopeAcqOK(1, &mirageTrackingRobot_DW.SFunction_IWORK.AcquireOK);
      }
    }

    if (i) {
      rl32eRestartAcquisition(1);
    }
  }

  /* Level2 S-Function Block: '<Root>/From Left Camera' (xpcusbvideoin) */
  {
    SimStruct *rts = mirageTrackingRobot_M->childSfunctions[0];
    sfcnStart(rts);
    if (ssGetErrorStatus(rts) != (NULL))
      return;
  }

  /* Level2 S-Function Block: '<Root>/From Right Camera' (xpcusbvideoin) */
  {
    SimStruct *rts = mirageTrackingRobot_M->childSfunctions[1];
    sfcnStart(rts);
    if (ssGetErrorStatus(rts) != (NULL))
      return;
  }
}

/* Model terminate function */
static void mirageTrackingRobot_terminate(void)
{
  /* Level2 S-Function Block: '<Root>/From Left Camera' (xpcusbvideoin) */
  {
    SimStruct *rts = mirageTrackingRobot_M->childSfunctions[0];
    sfcnTerminate(rts);
  }

  /* Level2 S-Function Block: '<Root>/From Right Camera' (xpcusbvideoin) */
  {
    SimStruct *rts = mirageTrackingRobot_M->childSfunctions[1];
    sfcnTerminate(rts);
  }
}

/*========================================================================*
 * Start of Classic call interface                                        *
 *========================================================================*/
void MdlOutputs(int_T tid)
{
  mirageTrackingRobot_output();
  UNUSED_PARAMETER(tid);
}

void MdlUpdate(int_T tid)
{
  mirageTrackingRobot_update();
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
  mirageTrackingRobot_initialize();
}

void MdlTerminate(void)
{
  mirageTrackingRobot_terminate();
}

/* Registration function */
RT_MODEL_mirageTrackingRobot_T *mirageTrackingRobot(void)
{
  /* Registration code */

  /* initialize non-finites */
  rt_InitInfAndNaN(sizeof(real_T));

  /* initialize real-time model */
  (void) memset((void *)mirageTrackingRobot_M, 0,
                sizeof(RT_MODEL_mirageTrackingRobot_T));
  rtsiSetSolverName(&mirageTrackingRobot_M->solverInfo,"FixedStepDiscrete");
  mirageTrackingRobot_M->solverInfoPtr = (&mirageTrackingRobot_M->solverInfo);

  /* Initialize timing info */
  {
    int_T *mdlTsMap = mirageTrackingRobot_M->Timing.sampleTimeTaskIDArray;
    mdlTsMap[0] = 0;
    mirageTrackingRobot_M->Timing.sampleTimeTaskIDPtr = (&mdlTsMap[0]);
    mirageTrackingRobot_M->Timing.sampleTimes =
      (&mirageTrackingRobot_M->Timing.sampleTimesArray[0]);
    mirageTrackingRobot_M->Timing.offsetTimes =
      (&mirageTrackingRobot_M->Timing.offsetTimesArray[0]);

    /* task periods */
    mirageTrackingRobot_M->Timing.sampleTimes[0] = (0.2);

    /* task offsets */
    mirageTrackingRobot_M->Timing.offsetTimes[0] = (0.0);
  }

  rtmSetTPtr(mirageTrackingRobot_M, &mirageTrackingRobot_M->Timing.tArray[0]);

  {
    int_T *mdlSampleHits = mirageTrackingRobot_M->Timing.sampleHitArray;
    mdlSampleHits[0] = 1;
    mirageTrackingRobot_M->Timing.sampleHits = (&mdlSampleHits[0]);
  }

  rtmSetTFinal(mirageTrackingRobot_M, 10.0);
  mirageTrackingRobot_M->Timing.stepSize0 = 0.2;

  /* Setup for data logging */
  {
    static RTWLogInfo rt_DataLoggingInfo;
    mirageTrackingRobot_M->rtwLogInfo = &rt_DataLoggingInfo;
  }

  /* Setup for data logging */
  {
    rtliSetLogXSignalInfo(mirageTrackingRobot_M->rtwLogInfo, (NULL));
    rtliSetLogXSignalPtrs(mirageTrackingRobot_M->rtwLogInfo, (NULL));
    rtliSetLogT(mirageTrackingRobot_M->rtwLogInfo, "tout");
    rtliSetLogX(mirageTrackingRobot_M->rtwLogInfo, "");
    rtliSetLogXFinal(mirageTrackingRobot_M->rtwLogInfo, "");
    rtliSetLogVarNameModifier(mirageTrackingRobot_M->rtwLogInfo, "rt_");
    rtliSetLogFormat(mirageTrackingRobot_M->rtwLogInfo, 0);
    rtliSetLogMaxRows(mirageTrackingRobot_M->rtwLogInfo, 1000);
    rtliSetLogDecimation(mirageTrackingRobot_M->rtwLogInfo, 1);
    rtliSetLogY(mirageTrackingRobot_M->rtwLogInfo, "");
    rtliSetLogYSignalInfo(mirageTrackingRobot_M->rtwLogInfo, (NULL));
    rtliSetLogYSignalPtrs(mirageTrackingRobot_M->rtwLogInfo, (NULL));
  }

  mirageTrackingRobot_M->solverInfoPtr = (&mirageTrackingRobot_M->solverInfo);
  mirageTrackingRobot_M->Timing.stepSize = (0.2);
  rtsiSetFixedStepSize(&mirageTrackingRobot_M->solverInfo, 0.2);
  rtsiSetSolverMode(&mirageTrackingRobot_M->solverInfo,
                    SOLVER_MODE_SINGLETASKING);

  /* block I/O */
  mirageTrackingRobot_M->ModelData.blockIO = ((void *) &mirageTrackingRobot_B);
  (void) memset(((void *) &mirageTrackingRobot_B), 0,
                sizeof(B_mirageTrackingRobot_T));

  /* parameters */
  mirageTrackingRobot_M->ModelData.defaultParam = ((real_T *)
    &mirageTrackingRobot_P);

  /* states (dwork) */
  mirageTrackingRobot_M->ModelData.dwork = ((void *) &mirageTrackingRobot_DW);
  (void) memset((void *)&mirageTrackingRobot_DW, 0,
                sizeof(DW_mirageTrackingRobot_T));

  /* Initialize DataMapInfo substructure containing ModelMap for C API */
  mirageTrackingRobot_InitializeDataMapInfo(mirageTrackingRobot_M);

  /* child S-Function registration */
  {
    RTWSfcnInfo *sfcnInfo = &mirageTrackingRobot_M->NonInlinedSFcns.sfcnInfo;
    mirageTrackingRobot_M->sfcnInfo = (sfcnInfo);
    rtssSetErrorStatusPtr(sfcnInfo, (&rtmGetErrorStatus(mirageTrackingRobot_M)));
    rtssSetNumRootSampTimesPtr(sfcnInfo,
      &mirageTrackingRobot_M->Sizes.numSampTimes);
    mirageTrackingRobot_M->NonInlinedSFcns.taskTimePtrs[0] = &(rtmGetTPtr
      (mirageTrackingRobot_M)[0]);
    rtssSetTPtrPtr(sfcnInfo,mirageTrackingRobot_M->NonInlinedSFcns.taskTimePtrs);
    rtssSetTStartPtr(sfcnInfo, &rtmGetTStart(mirageTrackingRobot_M));
    rtssSetTFinalPtr(sfcnInfo, &rtmGetTFinal(mirageTrackingRobot_M));
    rtssSetTimeOfLastOutputPtr(sfcnInfo, &rtmGetTimeOfLastOutput
      (mirageTrackingRobot_M));
    rtssSetStepSizePtr(sfcnInfo, &mirageTrackingRobot_M->Timing.stepSize);
    rtssSetStopRequestedPtr(sfcnInfo, &rtmGetStopRequested(mirageTrackingRobot_M));
    rtssSetDerivCacheNeedsResetPtr(sfcnInfo,
      &mirageTrackingRobot_M->ModelData.derivCacheNeedsReset);
    rtssSetZCCacheNeedsResetPtr(sfcnInfo,
      &mirageTrackingRobot_M->ModelData.zCCacheNeedsReset);
    rtssSetBlkStateChangePtr(sfcnInfo,
      &mirageTrackingRobot_M->ModelData.blkStateChange);
    rtssSetSampleHitsPtr(sfcnInfo, &mirageTrackingRobot_M->Timing.sampleHits);
    rtssSetPerTaskSampleHitsPtr(sfcnInfo,
      &mirageTrackingRobot_M->Timing.perTaskSampleHits);
    rtssSetSimModePtr(sfcnInfo, &mirageTrackingRobot_M->simMode);
    rtssSetSolverInfoPtr(sfcnInfo, &mirageTrackingRobot_M->solverInfoPtr);
  }

  mirageTrackingRobot_M->Sizes.numSFcns = (2);

  /* register each child */
  {
    (void) memset((void *)
                  &mirageTrackingRobot_M->NonInlinedSFcns.childSFunctions[0], 0,
                  2*sizeof(SimStruct));
    mirageTrackingRobot_M->childSfunctions =
      (&mirageTrackingRobot_M->NonInlinedSFcns.childSFunctionPtrs[0]);
    mirageTrackingRobot_M->childSfunctions[0] =
      (&mirageTrackingRobot_M->NonInlinedSFcns.childSFunctions[0]);
    mirageTrackingRobot_M->childSfunctions[1] =
      (&mirageTrackingRobot_M->NonInlinedSFcns.childSFunctions[1]);

    /* Level2 S-Function Block: mirageTrackingRobot/<Root>/From Left Camera (xpcusbvideoin) */
    {
      SimStruct *rts = mirageTrackingRobot_M->childSfunctions[0];

      /* timing info */
      time_T *sfcnPeriod =
        mirageTrackingRobot_M->NonInlinedSFcns.Sfcn0.sfcnPeriod;
      time_T *sfcnOffset =
        mirageTrackingRobot_M->NonInlinedSFcns.Sfcn0.sfcnOffset;
      int_T *sfcnTsMap = mirageTrackingRobot_M->NonInlinedSFcns.Sfcn0.sfcnTsMap;
      (void) memset((void*)sfcnPeriod, 0,
                    sizeof(time_T)*1);
      (void) memset((void*)sfcnOffset, 0,
                    sizeof(time_T)*1);
      ssSetSampleTimePtr(rts, &sfcnPeriod[0]);
      ssSetOffsetTimePtr(rts, &sfcnOffset[0]);
      ssSetSampleTimeTaskIDPtr(rts, sfcnTsMap);

      /* Set up the mdlInfo pointer */
      {
        ssSetBlkInfo2Ptr(rts, &mirageTrackingRobot_M->NonInlinedSFcns.blkInfo2[0]);
      }

      ssSetRTWSfcnInfo(rts, mirageTrackingRobot_M->sfcnInfo);

      /* Allocate memory of model methods 2 */
      {
        ssSetModelMethods2(rts, &mirageTrackingRobot_M->
                           NonInlinedSFcns.methods2[0]);
      }

      /* Allocate memory of model methods 3 */
      {
        ssSetModelMethods3(rts, &mirageTrackingRobot_M->
                           NonInlinedSFcns.methods3[0]);
      }

      /* Allocate memory for states auxilliary information */
      {
        ssSetStatesInfo2(rts,
                         &mirageTrackingRobot_M->NonInlinedSFcns.statesInfo2[0]);
      }

      /* outputs */
      {
        ssSetPortInfoForOutputs(rts,
          &mirageTrackingRobot_M->NonInlinedSFcns.Sfcn0.outputPortInfo[0]);
        _ssSetNumOutputPorts(rts, 1);

        /* port 0 */
        {
          int_T *dimensions = (int_T *)
            &mirageTrackingRobot_M->NonInlinedSFcns.Sfcn0.oDims0;
          dimensions[0] = 960;
          dimensions[1] = 1280;
          dimensions[2] = 3;
          _ssSetOutputPortDimensionsPtr(rts, 0, dimensions);
          _ssSetOutputPortNumDimensions(rts, 0, 3);
          ssSetOutputPortWidth(rts, 0, 3686400);
          ssSetOutputPortSignal(rts, 0, ((uint8_T *)
            mirageTrackingRobot_B.FromLeftCamera));
        }
      }

      /* path info */
      ssSetModelName(rts, "From Left Camera");
      ssSetPath(rts, "mirageTrackingRobot/From Left Camera");
      ssSetRTModel(rts,mirageTrackingRobot_M);
      ssSetParentSS(rts, (NULL));
      ssSetRootSS(rts, rts);
      ssSetVersion(rts, SIMSTRUCT_VERSION_LEVEL2);

      /* parameters */
      {
        mxArray **sfcnParams = (mxArray **)
          &mirageTrackingRobot_M->NonInlinedSFcns.Sfcn0.params;
        ssSetSFcnParamsCount(rts, 9);
        ssSetSFcnParamsPtr(rts, &sfcnParams[0]);
        ssSetSFcnParam(rts, 0, (mxArray*)
                       mirageTrackingRobot_P.FromLeftCamera_P1_Size);
        ssSetSFcnParam(rts, 1, (mxArray*)
                       mirageTrackingRobot_P.FromLeftCamera_P2_Size);
        ssSetSFcnParam(rts, 2, (mxArray*)
                       mirageTrackingRobot_P.FromLeftCamera_P3_Size);
        ssSetSFcnParam(rts, 3, (mxArray*)
                       mirageTrackingRobot_P.FromLeftCamera_P4_Size);
        ssSetSFcnParam(rts, 4, (mxArray*)
                       mirageTrackingRobot_P.FromLeftCamera_P5_Size);
        ssSetSFcnParam(rts, 5, (mxArray*)
                       mirageTrackingRobot_P.FromLeftCamera_P6_Size);
        ssSetSFcnParam(rts, 6, (mxArray*)
                       mirageTrackingRobot_P.FromLeftCamera_P7_Size);
        ssSetSFcnParam(rts, 7, (mxArray*)
                       mirageTrackingRobot_P.FromLeftCamera_P8_Size);
        ssSetSFcnParam(rts, 8, (mxArray*)
                       mirageTrackingRobot_P.FromLeftCamera_P9_Size);
      }

      /* work vectors */
      ssSetIWork(rts, (int_T *) &mirageTrackingRobot_DW.FromLeftCamera_IWORK);
      ssSetPWork(rts, (void **) &mirageTrackingRobot_DW.FromLeftCamera_PWORK[0]);

      {
        struct _ssDWorkRecord *dWorkRecord = (struct _ssDWorkRecord *)
          &mirageTrackingRobot_M->NonInlinedSFcns.Sfcn0.dWork;
        struct _ssDWorkAuxRecord *dWorkAuxRecord = (struct _ssDWorkAuxRecord *)
          &mirageTrackingRobot_M->NonInlinedSFcns.Sfcn0.dWorkAux;
        ssSetSFcnDWork(rts, dWorkRecord);
        ssSetSFcnDWorkAux(rts, dWorkAuxRecord);
        _ssSetNumDWork(rts, 2);

        /* IWORK */
        ssSetDWorkWidth(rts, 0, 1);
        ssSetDWorkDataType(rts, 0,SS_INTEGER);
        ssSetDWorkComplexSignal(rts, 0, 0);
        ssSetDWork(rts, 0, &mirageTrackingRobot_DW.FromLeftCamera_IWORK);

        /* PWORK */
        ssSetDWorkWidth(rts, 1, 3);
        ssSetDWorkDataType(rts, 1,SS_POINTER);
        ssSetDWorkComplexSignal(rts, 1, 0);
        ssSetDWork(rts, 1, &mirageTrackingRobot_DW.FromLeftCamera_PWORK[0]);
      }

      /* registration */
      xpcusbvideoin(rts);
      sfcnInitializeSizes(rts);
      sfcnInitializeSampleTimes(rts);

      /* adjust sample time */
      ssSetSampleTime(rts, 0, 0.2);
      ssSetOffsetTime(rts, 0, 0.0);
      sfcnTsMap[0] = 0;

      /* set compiled values of dynamic vector attributes */
      ssSetNumNonsampledZCs(rts, 0);

      /* Update connectivity flags for each port */
      _ssSetOutputPortConnected(rts, 0, 1);
      _ssSetOutputPortBeingMerged(rts, 0, 0);

      /* Update the BufferDstPort flags for each input port */
    }

    /* Level2 S-Function Block: mirageTrackingRobot/<Root>/From Right Camera (xpcusbvideoin) */
    {
      SimStruct *rts = mirageTrackingRobot_M->childSfunctions[1];

      /* timing info */
      time_T *sfcnPeriod =
        mirageTrackingRobot_M->NonInlinedSFcns.Sfcn1.sfcnPeriod;
      time_T *sfcnOffset =
        mirageTrackingRobot_M->NonInlinedSFcns.Sfcn1.sfcnOffset;
      int_T *sfcnTsMap = mirageTrackingRobot_M->NonInlinedSFcns.Sfcn1.sfcnTsMap;
      (void) memset((void*)sfcnPeriod, 0,
                    sizeof(time_T)*1);
      (void) memset((void*)sfcnOffset, 0,
                    sizeof(time_T)*1);
      ssSetSampleTimePtr(rts, &sfcnPeriod[0]);
      ssSetOffsetTimePtr(rts, &sfcnOffset[0]);
      ssSetSampleTimeTaskIDPtr(rts, sfcnTsMap);

      /* Set up the mdlInfo pointer */
      {
        ssSetBlkInfo2Ptr(rts, &mirageTrackingRobot_M->NonInlinedSFcns.blkInfo2[1]);
      }

      ssSetRTWSfcnInfo(rts, mirageTrackingRobot_M->sfcnInfo);

      /* Allocate memory of model methods 2 */
      {
        ssSetModelMethods2(rts, &mirageTrackingRobot_M->
                           NonInlinedSFcns.methods2[1]);
      }

      /* Allocate memory of model methods 3 */
      {
        ssSetModelMethods3(rts, &mirageTrackingRobot_M->
                           NonInlinedSFcns.methods3[1]);
      }

      /* Allocate memory for states auxilliary information */
      {
        ssSetStatesInfo2(rts,
                         &mirageTrackingRobot_M->NonInlinedSFcns.statesInfo2[1]);
      }

      /* outputs */
      {
        ssSetPortInfoForOutputs(rts,
          &mirageTrackingRobot_M->NonInlinedSFcns.Sfcn1.outputPortInfo[0]);
        _ssSetNumOutputPorts(rts, 1);

        /* port 0 */
        {
          int_T *dimensions = (int_T *)
            &mirageTrackingRobot_M->NonInlinedSFcns.Sfcn1.oDims0;
          dimensions[0] = 960;
          dimensions[1] = 1280;
          dimensions[2] = 3;
          _ssSetOutputPortDimensionsPtr(rts, 0, dimensions);
          _ssSetOutputPortNumDimensions(rts, 0, 3);
          ssSetOutputPortWidth(rts, 0, 3686400);
          ssSetOutputPortSignal(rts, 0, ((uint8_T *)
            mirageTrackingRobot_B.FromRightCamera));
        }
      }

      /* path info */
      ssSetModelName(rts, "From Right Camera");
      ssSetPath(rts, "mirageTrackingRobot/From Right Camera");
      ssSetRTModel(rts,mirageTrackingRobot_M);
      ssSetParentSS(rts, (NULL));
      ssSetRootSS(rts, rts);
      ssSetVersion(rts, SIMSTRUCT_VERSION_LEVEL2);

      /* parameters */
      {
        mxArray **sfcnParams = (mxArray **)
          &mirageTrackingRobot_M->NonInlinedSFcns.Sfcn1.params;
        ssSetSFcnParamsCount(rts, 9);
        ssSetSFcnParamsPtr(rts, &sfcnParams[0]);
        ssSetSFcnParam(rts, 0, (mxArray*)
                       mirageTrackingRobot_P.FromRightCamera_P1_Size);
        ssSetSFcnParam(rts, 1, (mxArray*)
                       mirageTrackingRobot_P.FromRightCamera_P2_Size);
        ssSetSFcnParam(rts, 2, (mxArray*)
                       mirageTrackingRobot_P.FromRightCamera_P3_Size);
        ssSetSFcnParam(rts, 3, (mxArray*)
                       mirageTrackingRobot_P.FromRightCamera_P4_Size);
        ssSetSFcnParam(rts, 4, (mxArray*)
                       mirageTrackingRobot_P.FromRightCamera_P5_Size);
        ssSetSFcnParam(rts, 5, (mxArray*)
                       mirageTrackingRobot_P.FromRightCamera_P6_Size);
        ssSetSFcnParam(rts, 6, (mxArray*)
                       mirageTrackingRobot_P.FromRightCamera_P7_Size);
        ssSetSFcnParam(rts, 7, (mxArray*)
                       mirageTrackingRobot_P.FromRightCamera_P8_Size);
        ssSetSFcnParam(rts, 8, (mxArray*)
                       mirageTrackingRobot_P.FromRightCamera_P9_Size);
      }

      /* work vectors */
      ssSetIWork(rts, (int_T *) &mirageTrackingRobot_DW.FromRightCamera_IWORK);
      ssSetPWork(rts, (void **) &mirageTrackingRobot_DW.FromRightCamera_PWORK[0]);

      {
        struct _ssDWorkRecord *dWorkRecord = (struct _ssDWorkRecord *)
          &mirageTrackingRobot_M->NonInlinedSFcns.Sfcn1.dWork;
        struct _ssDWorkAuxRecord *dWorkAuxRecord = (struct _ssDWorkAuxRecord *)
          &mirageTrackingRobot_M->NonInlinedSFcns.Sfcn1.dWorkAux;
        ssSetSFcnDWork(rts, dWorkRecord);
        ssSetSFcnDWorkAux(rts, dWorkAuxRecord);
        _ssSetNumDWork(rts, 2);

        /* IWORK */
        ssSetDWorkWidth(rts, 0, 1);
        ssSetDWorkDataType(rts, 0,SS_INTEGER);
        ssSetDWorkComplexSignal(rts, 0, 0);
        ssSetDWork(rts, 0, &mirageTrackingRobot_DW.FromRightCamera_IWORK);

        /* PWORK */
        ssSetDWorkWidth(rts, 1, 3);
        ssSetDWorkDataType(rts, 1,SS_POINTER);
        ssSetDWorkComplexSignal(rts, 1, 0);
        ssSetDWork(rts, 1, &mirageTrackingRobot_DW.FromRightCamera_PWORK[0]);
      }

      /* registration */
      xpcusbvideoin(rts);
      sfcnInitializeSizes(rts);
      sfcnInitializeSampleTimes(rts);

      /* adjust sample time */
      ssSetSampleTime(rts, 0, 0.2);
      ssSetOffsetTime(rts, 0, 0.0);
      sfcnTsMap[0] = 0;

      /* set compiled values of dynamic vector attributes */
      ssSetNumNonsampledZCs(rts, 0);

      /* Update connectivity flags for each port */
      _ssSetOutputPortConnected(rts, 0, 1);
      _ssSetOutputPortBeingMerged(rts, 0, 0);

      /* Update the BufferDstPort flags for each input port */
    }
  }

  /* Initialize Sizes */
  mirageTrackingRobot_M->Sizes.numContStates = (0);/* Number of continuous states */
  mirageTrackingRobot_M->Sizes.numY = (0);/* Number of model outputs */
  mirageTrackingRobot_M->Sizes.numU = (0);/* Number of model inputs */
  mirageTrackingRobot_M->Sizes.sysDirFeedThru = (0);/* The model is not direct feedthrough */
  mirageTrackingRobot_M->Sizes.numSampTimes = (1);/* Number of sample times */
  mirageTrackingRobot_M->Sizes.numBlocks = (9);/* Number of blocks */
  mirageTrackingRobot_M->Sizes.numBlockIO = (5);/* Number of block outputs */
  mirageTrackingRobot_M->Sizes.numBlockPrms = (78);/* Sum of parameter "widths" */
  return mirageTrackingRobot_M;
}

/*========================================================================*
 * End of Classic call interface                                          *
 *========================================================================*/
