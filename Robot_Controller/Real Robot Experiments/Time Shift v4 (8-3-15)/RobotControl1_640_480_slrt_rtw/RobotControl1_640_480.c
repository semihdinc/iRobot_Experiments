/*
 * RobotControl1_640_480.c
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

#include "rt_logging_mmi.h"
#include "RobotControl1_640_480_capi.h"
#include "RobotControl1_640_480.h"
#include "RobotControl1_640_480_private.h"

/* Block signals (auto storage) */
B_RobotControl1_640_480_T RobotControl1_640_480_B;

/* Block states (auto storage) */
DW_RobotControl1_640_480_T RobotControl1_640_480_DW;

/* Real-time model */
RT_MODEL_RobotControl1_640_480_T RobotControl1_640_480_M_;
RT_MODEL_RobotControl1_640_480_T *const RobotControl1_640_480_M =
  &RobotControl1_640_480_M_;

/* Forward declaration for local functions */
static void RobotControl1_640_480_SystemCore_step
  (visioncodegen_BlobAnalysis_RobotControl1_640_480_T *obj, const boolean_T
   varargin_1[307200], int32_T varargout_1_data[], int32_T varargout_1_sizes[2],
   real_T varargout_2_data[], int32_T varargout_2_sizes[2], int32_T
   varargout_3_data[], int32_T varargout_3_sizes[2]);
static void RobotControl1_640_480_SystemCore_step_h
  (visioncodegen_BlobAnalysis_RobotControl1_640_480_T *obj, const boolean_T
   varargin_1[307200], int32_T varargout_1_data[], int32_T varargout_1_sizes[2],
   real_T varargout_2_data[], int32_T varargout_2_sizes[2], int32_T
   varargout_3_data[], int32_T varargout_3_sizes[2]);
static void RobotControl1_640_480_merge(int32_T idx_data[], int32_T x_data[],
  int32_T offset, int32_T np, int32_T nq);
static void RobotControl1_640_480_eml_sort_idx(int32_T x_data[], int32_T
  *x_sizes, int32_T idx_data[], int32_T *idx_sizes);
static void RobotControl1_640_480_eml_sort_f(int32_T x_data[], int32_T x_sizes[2],
  int32_T dim, int32_T idx_data[], int32_T idx_sizes[2]);
static void RobotControl1_640_480_eml_sort(int32_T x_data[], int32_T x_sizes[2],
  int32_T idx_data[], int32_T idx_sizes[2]);
static void RobotControl1_640_480_eml_sort_fr(real_T x[4], int32_T idx[4]);
static void RobotControl1_640_480_orderPoints(const real_T p[8], real_T new_p[8]);
static void RobotControl1_640_480_invNxN(const real_T x[16], real_T y[16]);
static void RobotControl1_640_480_transformToImageSpace(const real_T S_P[16],
  real_T Cam_E_S[16], const real_T K[16], real_T p[8], real_T P[16], real_T M[16]);
static void RobotControl1_640_480_getEquations(const real_T Cam_PDes[16], const
  real_T S_PDes[16], const real_T Err[8], const real_T M[16], real_T equations
  [96], real_T constants[8]);
static void RobotControl1_640_480_eml_xgetrf(real_T A[144], int32_T ipiv[12]);
static void RobotControl1_640_480_solveEquations(const real_T A[192], const
  real_T B[16], real_T params[12]);
static void RobotControl1_640_480_mirage(real_T L_p[8], real_T R_p[8], const
  real_T S_PDes[16], real_T errParams[12]);

/* Output and update for function-call system: '<Root>/Status Extraction' */
void RobotControl1_640_480_StatusExtraction(void)
{
  int32_T out;

  /* Unpack: <S2>/Unpack */
  (void) memcpy(&RobotControl1_640_480_B.Unpack_o1,
                &RobotControl1_640_480_B.RS232BinaryReceive1_o2[0],
                1);
  (void) memcpy(&RobotControl1_640_480_B.Unpack_o2,
                &RobotControl1_640_480_B.RS232BinaryReceive1_o2[1],
                1);
  (void) memcpy(&RobotControl1_640_480_B.Unpack_o3,
                &RobotControl1_640_480_B.RS232BinaryReceive1_o2[2],
                1);
  (void) memcpy(&RobotControl1_640_480_B.Unpack_o4,
                &RobotControl1_640_480_B.RS232BinaryReceive1_o2[3],
                1);
  (void) memcpy(&RobotControl1_640_480_B.Unpack_o5,
                &RobotControl1_640_480_B.RS232BinaryReceive1_o2[4],
                1);
  (void) memcpy(&RobotControl1_640_480_B.Unpack_o6,
                &RobotControl1_640_480_B.RS232BinaryReceive1_o2[5],
                1);
  (void) memcpy(&RobotControl1_640_480_B.Unpack_o7,
                &RobotControl1_640_480_B.RS232BinaryReceive1_o2[6],
                1);
  (void) memcpy(&RobotControl1_640_480_B.Unpack_o8,
                &RobotControl1_640_480_B.RS232BinaryReceive1_o2[7],
                1);

  /* MATLAB Function: '<S2>/Command State' */
  /* MATLAB Function 'Status Extraction/Command State': '<S40>:1' */
  /*  This block supports an embeddable subset of the MATLAB language. */
  /*  See the help menu for details.  */
  /*  outputs the command channel state (CCS) given status output */
  /*  first check that have proper status message */
  /*  ID = 10 for SSC status info */
  if (RobotControl1_640_480_B.Unpack_o2 == 10) {
    /* '<S40>:1:8' */
    if (RobotControl1_640_480_B.Unpack_o5 == 2) {
      /* '<S40>:1:9' */
      /* '<S40>:1:10' */
      out = 1;
    } else if (RobotControl1_640_480_B.Unpack_o5 == 4) {
      /* '<S40>:1:11' */
      /* '<S40>:1:12' */
      out = 2;
    } else if (RobotControl1_640_480_B.Unpack_o5 == 6) {
      /* '<S40>:1:13' */
      /* '<S40>:1:14' */
      out = 3;
    } else {
      /* '<S40>:1:16' */
      out = 0;
    }
  } else {
    /* '<S40>:1:19' */
    out = 4;
  }

  /* '<S40>:1:21' */
  RobotControl1_640_480_B.CCS = out;

  /* End of MATLAB Function: '<S2>/Command State' */
  RobotControl1_640_480_DW.StatusExtraction_SubsysRanBC = 4;
}

/*
 * Forced non-inlined (FNI) function call stub
 * for '<Root>/Status Extraction'
 */
boolean_T RobotControl1_640_480_StatusExtractionFNI
  (RT_MODEL_RobotControl1_640_480_T *const RobotControl1_640_480_M, int_T
   controlPortIdx, int_T tid)
{
  RobotControl1_640_480_StatusExtraction();
  UNUSED_PARAMETER(RobotControl1_640_480_M);
  UNUSED_PARAMETER(controlPortIdx);
  UNUSED_PARAMETER(tid);
  return (1);
}

/*
 * Forced non-inlined (FNI) function call stub
 * for '<Root>/Status Extraction'
 */
boolean_T RobotControl1_640_480_StatusExtraction_InitFNI
  (RT_MODEL_RobotControl1_640_480_T *const RobotControl1_640_480_M, int_T
   controlPortIdx, int_T tid)
{
  UNUSED_PARAMETER(RobotControl1_640_480_M);
  UNUSED_PARAMETER(controlPortIdx);
  UNUSED_PARAMETER(tid);
  return (1);
}

/* Function for MATLAB Function: '<S10>/Calculate Target Points' */
static void RobotControl1_640_480_SystemCore_step
  (visioncodegen_BlobAnalysis_RobotControl1_640_480_T *obj, const boolean_T
   varargin_1[307200], int32_T varargout_1_data[], int32_T varargout_1_sizes[2],
   real_T varargout_2_data[], int32_T varargout_2_sizes[2], int32_T
   varargout_3_data[], int32_T varargout_3_sizes[2])
{
  visioncodegen_BlobAnalysis_RobotControl1_640_480_T *b_obj;
  vision_BlobAnalysis_1_RobotControl1_640_480_T *d_obj;
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
  for (loop = 0; loop < 483; loop++) {
    d_obj->W3_PAD_DW[loop] = 0U;
  }

  currentLabel = 1U;
  loop = 0;
  idx = 483;
  for (n = 0; n < 640; n++) {
    for (maxc = 0; maxc < 480; maxc++) {
      d_obj->W3_PAD_DW[idx] = (uint8_T)(varargin_1[loop] ? 255 : 0);
      loop++;
      idx++;
    }

    d_obj->W3_PAD_DW[idx] = 0U;
    d_obj->W3_PAD_DW[idx + 1] = 0U;
    idx += 2;
  }

  for (loop = 0; loop < 481; loop++) {
    d_obj->W3_PAD_DW[loop + idx] = 0U;
  }

  loop = 1;
  pixIdx = 0U;
  n = 0;
  while (n < 640) {
    maxc = 1;
    idx = loop * 482;
    maxr = 0;
    while (maxr < 480) {
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
                482U) - 1);
              d_obj->W1_M_PIXLIST_DW[pixIdx] = (int16_T)(walkerIdx % 482U - 1U);
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
          n = 640;
          maxr = 480;
        }

        if (maxr < 480) {
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
    n = 640;
    idx = 480;
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

/* Function for MATLAB Function: '<S10>/Calculate Target Points' */
static void RobotControl1_640_480_SystemCore_step_h
  (visioncodegen_BlobAnalysis_RobotControl1_640_480_T *obj, const boolean_T
   varargin_1[307200], int32_T varargout_1_data[], int32_T varargout_1_sizes[2],
   real_T varargout_2_data[], int32_T varargout_2_sizes[2], int32_T
   varargout_3_data[], int32_T varargout_3_sizes[2])
{
  visioncodegen_BlobAnalysis_RobotControl1_640_480_T *b_obj;
  vision_BlobAnalysis_1_RobotControl1_640_480_T *d_obj;
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
  for (loop = 0; loop < 483; loop++) {
    d_obj->W3_PAD_DW[loop] = 0U;
  }

  currentLabel = 1U;
  loop = 0;
  idx = 483;
  for (n = 0; n < 640; n++) {
    for (maxc = 0; maxc < 480; maxc++) {
      d_obj->W3_PAD_DW[idx] = (uint8_T)(varargin_1[loop] ? 255 : 0);
      loop++;
      idx++;
    }

    d_obj->W3_PAD_DW[idx] = 0U;
    d_obj->W3_PAD_DW[idx + 1] = 0U;
    idx += 2;
  }

  for (loop = 0; loop < 481; loop++) {
    d_obj->W3_PAD_DW[loop + idx] = 0U;
  }

  loop = 1;
  pixIdx = 0U;
  n = 0;
  while (n < 640) {
    maxc = 1;
    idx = loop * 482;
    maxr = 0;
    while (maxr < 480) {
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
                482U) - 1);
              d_obj->W1_M_PIXLIST_DW[pixIdx] = (int16_T)(walkerIdx % 482U - 1U);
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
          n = 640;
          maxr = 480;
        }

        if (maxr < 480) {
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
    n = 640;
    idx = 480;
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

/* Function for MATLAB Function: '<S10>/Calculate Target Points' */
static void RobotControl1_640_480_merge(int32_T idx_data[], int32_T x_data[],
  int32_T offset, int32_T np, int32_T nq)
{
  int32_T n;
  int32_T p;
  int32_T q;
  int32_T qend;
  int32_T iout;
  int32_T j;
  int32_T iwork_data[50];
  int32_T xwork_data[50];
  if (nq != 0) {
    n = np + nq;
    for (j = 0; j + 1 <= n; j++) {
      iwork_data[j] = idx_data[offset + j];
      xwork_data[j] = x_data[offset + j];
    }

    p = 0;
    q = np;
    qend = np + nq;
    iout = offset - 1;
    do {
      n = 0;
      iout++;
      if (xwork_data[p] >= xwork_data[q]) {
        idx_data[iout] = iwork_data[p];
        x_data[iout] = xwork_data[p];
        if (p + 1 < np) {
          p++;
        } else {
          n = 1;
        }
      } else {
        idx_data[iout] = iwork_data[q];
        x_data[iout] = xwork_data[q];
        if (q + 1 < qend) {
          q++;
        } else {
          n = (iout - p) + 1;
          for (j = p; j + 1 <= np; j++) {
            idx_data[n + j] = iwork_data[j];
            x_data[n + j] = xwork_data[j];
          }

          n = 1;
        }
      }
    } while (n == 0);
  }
}

/* Function for MATLAB Function: '<S10>/Calculate Target Points' */
static void RobotControl1_640_480_eml_sort_idx(int32_T x_data[], int32_T
  *x_sizes, int32_T idx_data[], int32_T *idx_sizes)
{
  int32_T x4[4];
  int8_T idx4[4];
  int8_T perm[4];
  int32_T nQuartets;
  int32_T nLeft;
  int32_T i3;
  int32_T i4;
  int32_T bLen;
  int32_T tailOffset;
  int32_T nPairs;
  int32_T b_x_data[50];
  int32_T b_x_sizes;
  int8_T b_idx_0;
  b_idx_0 = (int8_T)*x_sizes;
  b_x_sizes = *x_sizes;
  nQuartets = *x_sizes;
  for (nLeft = 0; nLeft < nQuartets; nLeft++) {
    b_x_data[nLeft] = x_data[nLeft];
  }

  *idx_sizes = b_idx_0;
  nQuartets = b_idx_0;
  for (nLeft = 0; nLeft < nQuartets; nLeft++) {
    idx_data[nLeft] = 0;
  }

  x4[0] = 0;
  x4[1] = 0;
  x4[2] = 0;
  x4[3] = 0;
  idx4[0] = 0;
  idx4[1] = 0;
  idx4[2] = 0;
  idx4[3] = 0;
  nQuartets = *x_sizes >> 2;
  for (nLeft = 1; nLeft <= nQuartets; nLeft++) {
    tailOffset = (nLeft - 1) << 2;
    idx4[0] = (int8_T)(tailOffset + 1);
    idx4[1] = (int8_T)(tailOffset + 2);
    idx4[2] = (int8_T)(tailOffset + 3);
    idx4[3] = (int8_T)(tailOffset + 4);
    x4[0] = b_x_data[tailOffset];
    x4[1] = b_x_data[tailOffset + 1];
    x4[2] = b_x_data[tailOffset + 2];
    x4[3] = b_x_data[tailOffset + 3];
    if (b_x_data[tailOffset] >= b_x_data[tailOffset + 1]) {
      bLen = 1;
      nPairs = 2;
    } else {
      bLen = 2;
      nPairs = 1;
    }

    if (b_x_data[tailOffset + 2] >= b_x_data[tailOffset + 3]) {
      i3 = 3;
      i4 = 4;
    } else {
      i3 = 4;
      i4 = 3;
    }

    if (x4[bLen - 1] >= x4[i3 - 1]) {
      if (x4[nPairs - 1] >= x4[i3 - 1]) {
        perm[0] = (int8_T)bLen;
        perm[1] = (int8_T)nPairs;
        perm[2] = (int8_T)i3;
        perm[3] = (int8_T)i4;
      } else if (x4[nPairs - 1] >= x4[i4 - 1]) {
        perm[0] = (int8_T)bLen;
        perm[1] = (int8_T)i3;
        perm[2] = (int8_T)nPairs;
        perm[3] = (int8_T)i4;
      } else {
        perm[0] = (int8_T)bLen;
        perm[1] = (int8_T)i3;
        perm[2] = (int8_T)i4;
        perm[3] = (int8_T)nPairs;
      }
    } else if (x4[bLen - 1] >= x4[i4 - 1]) {
      if (x4[nPairs - 1] >= x4[i4 - 1]) {
        perm[0] = (int8_T)i3;
        perm[1] = (int8_T)bLen;
        perm[2] = (int8_T)nPairs;
        perm[3] = (int8_T)i4;
      } else {
        perm[0] = (int8_T)i3;
        perm[1] = (int8_T)bLen;
        perm[2] = (int8_T)i4;
        perm[3] = (int8_T)nPairs;
      }
    } else {
      perm[0] = (int8_T)i3;
      perm[1] = (int8_T)i4;
      perm[2] = (int8_T)bLen;
      perm[3] = (int8_T)nPairs;
    }

    idx_data[tailOffset] = idx4[perm[0] - 1];
    idx_data[tailOffset + 1] = idx4[perm[1] - 1];
    idx_data[tailOffset + 2] = idx4[perm[2] - 1];
    idx_data[tailOffset + 3] = idx4[perm[3] - 1];
    b_x_data[tailOffset] = x4[perm[0] - 1];
    b_x_data[tailOffset + 1] = x4[perm[1] - 1];
    b_x_data[tailOffset + 2] = x4[perm[2] - 1];
    b_x_data[tailOffset + 3] = x4[perm[3] - 1];
  }

  nQuartets <<= 2;
  nLeft = *x_sizes - nQuartets;
  if (nLeft > 0) {
    for (tailOffset = 1; tailOffset <= nLeft; tailOffset++) {
      idx4[tailOffset - 1] = (int8_T)(nQuartets + tailOffset);
      x4[tailOffset - 1] = b_x_data[(nQuartets + tailOffset) - 1];
    }

    perm[1] = 0;
    perm[2] = 0;
    perm[3] = 0;
    if (nLeft == 1) {
      perm[0] = 1;
    } else if (nLeft == 2) {
      if (x4[0] >= x4[1]) {
        perm[0] = 1;
        perm[1] = 2;
      } else {
        perm[0] = 2;
        perm[1] = 1;
      }
    } else if (x4[0] >= x4[1]) {
      if (x4[1] >= x4[2]) {
        perm[0] = 1;
        perm[1] = 2;
        perm[2] = 3;
      } else if (x4[0] >= x4[2]) {
        perm[0] = 1;
        perm[1] = 3;
        perm[2] = 2;
      } else {
        perm[0] = 3;
        perm[1] = 1;
        perm[2] = 2;
      }
    } else if (x4[0] >= x4[2]) {
      perm[0] = 2;
      perm[1] = 1;
      perm[2] = 3;
    } else if (x4[1] >= x4[2]) {
      perm[0] = 2;
      perm[1] = 3;
      perm[2] = 1;
    } else {
      perm[0] = 3;
      perm[1] = 2;
      perm[2] = 1;
    }

    for (tailOffset = 1; tailOffset <= nLeft; tailOffset++) {
      idx_data[(nQuartets + tailOffset) - 1] = idx4[perm[tailOffset - 1] - 1];
      b_x_data[(nQuartets + tailOffset) - 1] = x4[perm[tailOffset - 1] - 1];
    }
  }

  if (*x_sizes > 1) {
    nPairs = *x_sizes >> 2;
    bLen = 4;
    while (nPairs > 1) {
      if ((nPairs & 1) != 0) {
        nPairs--;
        tailOffset = bLen * nPairs;
        nQuartets = *x_sizes - tailOffset;
        if (nQuartets > bLen) {
          RobotControl1_640_480_merge(idx_data, b_x_data, tailOffset, bLen,
            nQuartets - bLen);
        }
      }

      nQuartets = bLen << 1;
      nPairs >>= 1;
      for (nLeft = 1; nLeft <= nPairs; nLeft++) {
        RobotControl1_640_480_merge(idx_data, b_x_data, (nLeft - 1) * nQuartets,
          bLen, bLen);
      }

      bLen = nQuartets;
    }

    if (*x_sizes > bLen) {
      RobotControl1_640_480_merge(idx_data, b_x_data, 0, bLen, *x_sizes - bLen);
    }
  }

  *x_sizes = b_x_sizes;
  for (nLeft = 0; nLeft < b_x_sizes; nLeft++) {
    x_data[nLeft] = b_x_data[nLeft];
  }
}

/* Function for MATLAB Function: '<S10>/Calculate Target Points' */
static void RobotControl1_640_480_eml_sort_f(int32_T x_data[], int32_T x_sizes[2],
  int32_T dim, int32_T idx_data[], int32_T idx_sizes[2])
{
  int32_T vstride;
  int32_T npages;
  int32_T b;
  int32_T j;
  int32_T c_k;
  int32_T vwork_data[50];
  int32_T vwork_sizes;
  int32_T iidx_data[50];
  int8_T c_idx_0;
  int8_T c_idx_1;
  b = x_sizes[dim - 1];
  c_idx_0 = (int8_T)x_sizes[dim - 1];
  vwork_sizes = c_idx_0;
  c_idx_0 = (int8_T)x_sizes[0];
  c_idx_1 = (int8_T)x_sizes[1];
  idx_sizes[0] = c_idx_0;
  idx_sizes[1] = c_idx_1;
  vstride = 1;
  c_k = 1;
  while (c_k <= dim - 1) {
    vstride *= x_sizes[0];
    c_k = 2;
  }

  npages = 1;
  c_k = dim + 1;
  while (c_k < 3) {
    npages *= x_sizes[1];
    c_k = 3;
  }

  c_k = 1;
  while (c_k <= npages) {
    for (j = 0; j + 1 <= vstride; j++) {
      for (c_k = 0; c_k + 1 <= b; c_k++) {
        vwork_data[c_k] = x_data[c_k * vstride + j];
      }

      RobotControl1_640_480_eml_sort_idx(vwork_data, &vwork_sizes, iidx_data,
        &c_k);
      for (c_k = 0; c_k + 1 <= b; c_k++) {
        x_data[j + c_k * vstride] = vwork_data[c_k];
        idx_data[j + c_k * vstride] = iidx_data[c_k];
      }
    }

    c_k = 2;
  }
}

/* Function for MATLAB Function: '<S10>/Calculate Target Points' */
static void RobotControl1_640_480_eml_sort(int32_T x_data[], int32_T x_sizes[2],
  int32_T idx_data[], int32_T idx_sizes[2])
{
  int32_T dim;
  dim = 2;
  if (x_sizes[0] != 1) {
    dim = 1;
  }

  RobotControl1_640_480_eml_sort_f(x_data, x_sizes, dim, idx_data, idx_sizes);
}

/* Function for MATLAB Function: '<S10>/Calculate Target Points' */
static void RobotControl1_640_480_eml_sort_fr(real_T x[4], int32_T idx[4])
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

/* Function for MATLAB Function: '<S10>/Calculate Target Points' */
static void RobotControl1_640_480_orderPoints(const real_T p[8], real_T new_p[8])
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
  RobotControl1_640_480_eml_sort_fr(x, iidx);
  indx_idx_0 = iidx[0];
  indx_idx_3 = iidx[3];
  x[0] = 480.0 - p[1];
  x[1] = 480.0 - p[3];
  x[2] = 480.0 - p[5];
  x[3] = 480.0 - p[7];
  RobotControl1_640_480_eml_sort_fr(x, iidx);
  x[2] = iidx[2];
  x[3] = iidx[3];

  /* sort(480-p(2,:)); */
  indx[0] = indx_idx_0;
  indx[1] = indx_idx_3;
  indx[2] = (int32_T)x[2];
  indx[3] = (int32_T)x[3];
  for (indx_idx_0 = 0; indx_idx_0 < 4; indx_idx_0++) {
    new_p[indx_idx_0 << 1] = p[(indx[indx_idx_0] - 1) << 1];
    new_p[1 + (indx_idx_0 << 1)] = p[((indx[indx_idx_0] - 1) << 1) + 1];
  }
}

/* Function for MATLAB Function: '<S10>/Pose Estimation' */
static void RobotControl1_640_480_invNxN(const real_T x[16], real_T y[16])
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

/* Function for MATLAB Function: '<S10>/Pose Estimation' */
static void RobotControl1_640_480_transformToImageSpace(const real_T S_P[16],
  real_T Cam_E_S[16], const real_T K[16], real_T p[8], real_T P[16], real_T M[16])
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

/* Function for MATLAB Function: '<S10>/Pose Estimation' */
static void RobotControl1_640_480_getEquations(const real_T Cam_PDes[16], const
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

/* Function for MATLAB Function: '<S10>/Pose Estimation' */
static void RobotControl1_640_480_eml_xgetrf(real_T A[144], int32_T ipiv[12])
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

/* Function for MATLAB Function: '<S10>/Pose Estimation' */
static void RobotControl1_640_480_solveEquations(const real_T A[192], const
  real_T B[16], real_T params[12])
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
  RobotControl1_640_480_eml_xgetrf(y, ipiv);
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

/* Function for MATLAB Function: '<S10>/Pose Estimation' */
static void RobotControl1_640_480_mirage(real_T L_p[8], real_T R_p[8], const
  real_T S_PDes[16], real_T errParams[12])
{
  real_T L_pDes1[8];
  real_T L_PDes[16];
  real_T L_M[16];
  real_T R_pDes1[8];
  real_T R_M[16];
  real_T Left_Equations[96];
  real_T Right_Equations[96];
  static const real_T b[16] = { 0.99408939292827669, -0.0303012712375131,
    0.10425023658908782, 0.0, 0.03559015855839049, 0.99815208822027335,
    -0.049251897377625246, 0.0, -0.102565196247454, 0.052671071214644635,
    0.99333083047684723, 0.0, -3.3855740411858486, -5.3701232919726891,
    -0.064827854150750527, 1.0 };

  static const real_T c[16] = { 714.6172, 0.0, 0.0, 0.0, 0.0, 725.6549, 0.0, 0.0,
    322.847, 263.4883, 1.0, 0.0, 0.0, 0.0, 0.0, 1.0 };

  static const real_T d[16] = { 0.99465869522403538, -0.029944289509316737,
    0.098779651452116507, 0.0, 0.035178706104138979, 0.99804382087417587,
    -0.051681623926830796, 0.0, -0.097038851250713043, 0.0548805169496241,
    0.99376636600726453, 0.0, -3.4263357315865255, 6.2161780819644559,
    -0.677531604019529, 1.0 };

  static const real_T e[16] = { 709.8338, 0.0, 0.0, 0.0, 0.0, 719.9022, 0.0, 0.0,
    322.7411, 231.1758, 1.0, 0.0, 0.0, 0.0, 0.0, 1.0 };

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
  /*  coordinates of the optical axis (pixels)  */
  /*  focal length of the lens (pixels) */
  /*  coordinates of the optical axis (pixels) */
  /*  Project Points into the camera and image space */
  for (i = 0; i < 16; i++) {
    b_0[i] = b[i];
    d_0[i] = d[i];
  }

  RobotControl1_640_480_transformToImageSpace(S_PDes, b_0, c, L_pDes1, L_PDes,
    L_M);
  RobotControl1_640_480_transformToImageSpace(S_PDes, d_0, e, R_pDes1, b_0, R_M);
  L_p[1] = 480.0 - L_p[1];
  L_p[3] = 480.0 - L_p[3];
  L_p[5] = 480.0 - L_p[5];
  L_p[7] = 480.0 - L_p[7];
  R_p[1] = 480.0 - R_p[1];
  R_p[3] = 480.0 - R_p[3];
  R_p[5] = 480.0 - R_p[5];
  R_p[7] = 480.0 - R_p[7];

  /*  Pixel Error */
  /*  Left and Right Camera Equations (2 equations for 1 point) */
  for (i = 0; i < 8; i++) {
    L_p_0[i] = L_p[i] - L_pDes1[i];
    R_p_0[i] = R_p[i] - R_pDes1[i];
  }

  RobotControl1_640_480_getEquations(L_PDes, S_PDes, L_p_0, L_M, Left_Equations,
    R_pDes1);
  RobotControl1_640_480_getEquations(b_0, S_PDes, R_p_0, R_M, Right_Equations,
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
  RobotControl1_640_480_solveEquations(Left_Equations_0, b_0, errParams);
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

real_T rt_powd_snf(real_T u0, real_T u1)
{
  real_T y;
  real_T tmp;
  real_T tmp_0;
  if (rtIsNaN(u0) || rtIsNaN(u1)) {
    y = (rtNaN);
  } else {
    tmp = fabs(u0);
    tmp_0 = fabs(u1);
    if (rtIsInf(u1)) {
      if (tmp == 1.0) {
        y = (rtNaN);
      } else if (tmp > 1.0) {
        if (u1 > 0.0) {
          y = (rtInf);
        } else {
          y = 0.0;
        }
      } else if (u1 > 0.0) {
        y = 0.0;
      } else {
        y = (rtInf);
      }
    } else if (tmp_0 == 0.0) {
      y = 1.0;
    } else if (tmp_0 == 1.0) {
      if (u1 > 0.0) {
        y = u0;
      } else {
        y = 1.0 / u0;
      }
    } else if (u1 == 2.0) {
      y = u0 * u0;
    } else if ((u1 == 0.5) && (u0 >= 0.0)) {
      y = sqrt(u0);
    } else if ((u0 < 0.0) && (u1 > floor(u1))) {
      y = (rtNaN);
    } else {
      y = pow(u0, u1);
    }
  }

  return y;
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
static void RobotControl1_640_480_output(void)
{
  int32_T en;
  visioncodegen_BlobAnalysis_RobotControl1_640_480_T *obj;
  static const int16_T b[8] = { -1, 481, 482, 483, 1, -481, -482, -483 };

  real_T xrdot;
  real_T world2robotDes[16];
  real_T errParams[12];
  real_T poseErrorMat[16];
  real_T world2robot[16];
  real_T y[12];
  static const int8_T b_0[16] = { 1, 0, 0, 0, 0, 1, 0, 0, 0, 0, 1, 0, 0, 0, 0, 1
  };

  real_T s;
  real_T c;
  real_T y_0;
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
  int32_T i;
  real_T tmp[8];
  real_T world2robotDes_0[16];
  real_T l_center[8];
  int32_T i_0;
  int32_T loop_ub;
  real_T tmp_0[16];
  real_T tmp_1[16];
  real_T tmp_2[16];
  int32_T l_area_data[50];
  int32_T l_area_sizes[2];
  real_T l_center_data[100];
  int32_T l_center_sizes[2];
  int32_T unusedU2_data[200];
  int32_T unusedU2_sizes[2];
  int32_T r_area_data[50];
  int32_T r_area_sizes[2];
  real_T r_center_data[100];
  int32_T r_center_sizes[2];
  int32_T indL_data[50];
  int32_T indR_data[50];
  int32_T iidx_data[50];
  int32_T indR_sizes_idx_0;
  int32_T indR_sizes_idx_1;
  uint32_T qY;

  /* Reset subsysRan breadcrumbs */
  srClearBC(RobotControl1_640_480_DW.StatusExtraction_SubsysRanBC);

  /* Reset subsysRan breadcrumbs */
  srClearBC(RobotControl1_640_480_DW.Read_SubsysRanBC);

  /* Reset subsysRan breadcrumbs */
  srClearBC(RobotControl1_640_480_DW.Write_SubsysRanBC);

  /* Constant: '<Root>/packet size' */
  RobotControl1_640_480_B.packetsize = RobotControl1_640_480_P.packetsize_Value;

  /* Constant: '<Root>/packet size4' */
  RobotControl1_640_480_B.packetsize4 =
    RobotControl1_640_480_P.packetsize4_Value;

  /* S-Function (rs232brec): '<Root>/RS232 Binary Receive1' */

  /* Level2 S-Function Block: '<Root>/RS232 Binary Receive1' (rs232brec) */
  {
    SimStruct *rts = RobotControl1_640_480_M->childSfunctions[1];
    sfcnOutputs(rts, 1);
  }

  /* End of Outputs for S-Function (rs232brec): '<Root>/RS232 Binary Receive1' */

  /* MATLAB Function: '<Root>/status checker' */
  /* MATLAB Function 'status checker': '<S4>:1' */
  /*  This block supports an embeddable subset of the MATLAB language. */
  /*  See the help menu for details.  */
  if (RobotControl1_640_480_B.CCS == 1.0) {
    /* '<S4>:1:4' */
    /* '<S4>:1:5' */
    en = 1;
  } else {
    /* '<S4>:1:7' */
    en = -1;
  }

  /* '<S4>:1:10' */
  RobotControl1_640_480_B.enable = en;

  /* End of MATLAB Function: '<Root>/status checker' */

  /* Level2 S-Function Block: '<S10>/Left Camera' (xpcusbvideoin) */
  {
    SimStruct *rts = RobotControl1_640_480_M->childSfunctions[2];
    sfcnOutputs(rts, 1);
  }

  /* Level2 S-Function Block: '<S10>/Right Camera' (xpcusbvideoin) */
  {
    SimStruct *rts = RobotControl1_640_480_M->childSfunctions[3];
    sfcnOutputs(rts, 1);
  }

  /* MATLAB Function: '<S10>/Calculate Target Points' */
  /* MATLAB Function 'Open-loop control/Vision System/Calculate Target Points': '<S13>:1' */
  /* '<S13>:1:4' */
  /* '<S13>:1:5' */
  for (i = 0; i < 8; i++) {
    RobotControl1_640_480_B.L_p[i] = 0.0;
    RobotControl1_640_480_B.R_p[i] = 0.0;
  }

  /* input left image(l_rgb), right image(r_rgb), Pose target position  */
  /* '<S13>:1:10' */
  /*  converts RGB to CMY color space  */
  /* '<S13>:1:11' */
  for (i_0 = 0; i_0 < 921600; i_0++) {
    RobotControl1_640_480_B.l_rgb1[i_0] = (uint8_T)(255U -
      RobotControl1_640_480_B.LeftCamera[i_0]);
    RobotControl1_640_480_B.r_rgb1[i_0] = (uint8_T)(255U -
      RobotControl1_640_480_B.RightCamera[i_0]);
  }

  /*  converts RGB to CMY color space  */
  /* '<S13>:1:13' */
  /*  rgb2gray(rgb) ;convert rgb to grayscale  */
  /* '<S13>:1:14' */
  /*  rgb2gray(rgb) ;convert rgb to grayscale */
  /* '<S13>:1:16' */
  /*  imsubtract(rgb1(:,:,3),gray) */
  /* '<S13>:1:17' */
  /*  imsubtract(rgb1(:,:,3),gray)  */
  /* '<S13>:1:19' */
  /* '<S13>:1:20' */
  /* finds yellow regions in image using BlobAnalysis. */
  obj = &RobotControl1_640_480_B.hblob;
  RobotControl1_640_480_B.hblob.isInitialized = 0;
  RobotControl1_640_480_B.hblob.NoTuningBeforeLockingCodeGenError = true;

  /* System object Constructor function: vision.BlobAnalysis */
  for (i = 0; i < 8; i++) {
    obj->cSFunObject.P0_WALKER_RTP[i] = b[i];
  }

  obj->cSFunObject.P1_MINAREA_RTP = 0U;
  obj->cSFunObject.P2_MAXAREA_RTP = MAX_uint32_T;
  obj->cSFunObject.P1_MINAREA_RTP = 0U;
  obj->cSFunObject.P2_MAXAREA_RTP = MAX_uint32_T;
  RobotControl1_640_480_B.hblob.NoTuningBeforeLockingCodeGenError = false;
  for (i_0 = 0; i_0 < 640; i_0++) {
    for (loop_ub = 0; loop_ub < 480; loop_ub++) {
      i = RobotControl1_640_480_B.l_rgb1[(480 * i_0 + loop_ub) + 614400];
      indR_sizes_idx_0 = (int32_T)rt_roundd_snf((real_T)
        RobotControl1_640_480_B.l_rgb1[480 * i_0 + loop_ub] * 0.2989);
      servo1a = (uint8_T)indR_sizes_idx_0;
      indR_sizes_idx_0 = (int32_T)rt_roundd_snf((real_T)
        RobotControl1_640_480_B.l_rgb1[(480 * i_0 + loop_ub) + 307200] * 0.587);
      servo2a = (uint8_T)indR_sizes_idx_0;
      indR_sizes_idx_0 = (int32_T)((uint32_T)servo1a + servo2a);
      indR_sizes_idx_1 = (int32_T)rt_roundd_snf((real_T)
        RobotControl1_640_480_B.l_rgb1[(480 * i_0 + loop_ub) + 614400] * 0.114);
      servo1a = (uint8_T)indR_sizes_idx_1;
      indR_sizes_idx_0 = (int32_T)((uint32_T)indR_sizes_idx_0 + servo1a);
      qY = (uint32_T)i - indR_sizes_idx_0;
      if (qY > (uint32_T)i) {
        qY = 0U;
      }

      indR_sizes_idx_0 = (int32_T)qY;
      RobotControl1_640_480_B.l_rgb1_m[loop_ub + 480 * i_0] = (indR_sizes_idx_0 >
        50);
    }
  }

  RobotControl1_640_480_SystemCore_step(&RobotControl1_640_480_B.hblob,
    RobotControl1_640_480_B.l_rgb1_m, l_area_data, l_area_sizes, l_center_data,
    l_center_sizes, unusedU2_data, unusedU2_sizes);

  /*  get [x y] is a centroid of region */
  for (i_0 = 0; i_0 < 640; i_0++) {
    for (loop_ub = 0; loop_ub < 480; loop_ub++) {
      i = RobotControl1_640_480_B.r_rgb1[(480 * i_0 + loop_ub) + 614400];
      indR_sizes_idx_0 = (int32_T)rt_roundd_snf((real_T)
        RobotControl1_640_480_B.r_rgb1[480 * i_0 + loop_ub] * 0.2989);
      servo1a = (uint8_T)indR_sizes_idx_0;
      indR_sizes_idx_0 = (int32_T)rt_roundd_snf((real_T)
        RobotControl1_640_480_B.r_rgb1[(480 * i_0 + loop_ub) + 307200] * 0.587);
      servo2a = (uint8_T)indR_sizes_idx_0;
      indR_sizes_idx_0 = (int32_T)((uint32_T)servo1a + servo2a);
      indR_sizes_idx_1 = (int32_T)rt_roundd_snf((real_T)
        RobotControl1_640_480_B.r_rgb1[(480 * i_0 + loop_ub) + 614400] * 0.114);
      servo1a = (uint8_T)indR_sizes_idx_1;
      indR_sizes_idx_0 = (int32_T)((uint32_T)indR_sizes_idx_0 + servo1a);
      qY = (uint32_T)i - indR_sizes_idx_0;
      if (qY > (uint32_T)i) {
        qY = 0U;
      }

      indR_sizes_idx_0 = (int32_T)qY;
      RobotControl1_640_480_B.l_rgb1_m[loop_ub + 480 * i_0] = (indR_sizes_idx_0 >
        50);
    }
  }

  RobotControl1_640_480_SystemCore_step_h(&RobotControl1_640_480_B.hblob,
    RobotControl1_640_480_B.l_rgb1_m, r_area_data, r_area_sizes, r_center_data,
    r_center_sizes, unusedU2_data, unusedU2_sizes);

  /*  get [x y] is a centroid of region */
  RobotControl1_640_480_eml_sort(l_area_data, l_area_sizes, iidx_data,
    unusedU2_sizes);
  en = unusedU2_sizes[0];
  i = unusedU2_sizes[1];
  loop_ub = unusedU2_sizes[0] * unusedU2_sizes[1];
  for (i_0 = 0; i_0 < loop_ub; i_0++) {
    indL_data[i_0] = iidx_data[i_0];
  }

  RobotControl1_640_480_eml_sort(r_area_data, r_area_sizes, iidx_data,
    unusedU2_sizes);
  indR_sizes_idx_0 = unusedU2_sizes[0];
  indR_sizes_idx_1 = unusedU2_sizes[1];
  loop_ub = unusedU2_sizes[0] * unusedU2_sizes[1];
  for (i_0 = 0; i_0 < loop_ub; i_0++) {
    indR_data[i_0] = iidx_data[i_0];
  }

  if ((0 == en) || (0 == i)) {
    en = 0;
  } else {
    if (!(en >= 1)) {
      en = 1;
    }
  }

  if (en >= 4) {
    /* '<S13>:1:31' */
    if ((0 == indR_sizes_idx_0) || (0 == indR_sizes_idx_1)) {
      en = 0;
    } else if (indR_sizes_idx_0 >= 1) {
      en = indR_sizes_idx_0;
    } else {
      en = 1;
    }

    if (en >= 4) {
      /* '<S13>:1:31' */
      /* '<S13>:1:32' */
      /* '<S13>:1:33' */
      /* '<S13>:1:35' */
      for (i_0 = 0; i_0 < 4; i_0++) {
        l_center[i_0 << 1] = l_center_data[indL_data[i_0] - 1];
        l_center[1 + (i_0 << 1)] = l_center_data[(indL_data[i_0] +
          l_center_sizes[0]) - 1];
      }

      RobotControl1_640_480_orderPoints(l_center, RobotControl1_640_480_B.L_p);

      /* '<S13>:1:36' */
      for (i_0 = 0; i_0 < 4; i_0++) {
        l_center[i_0 << 1] = r_center_data[indR_data[i_0] - 1];
        l_center[1 + (i_0 << 1)] = r_center_data[(indR_data[i_0] +
          r_center_sizes[0]) - 1];
      }

      RobotControl1_640_480_orderPoints(l_center, RobotControl1_640_480_B.R_p);
    }
  }

  /* End of MATLAB Function: '<S10>/Calculate Target Points' */

  /* Clock: '<Root>/Clock' */
  RobotControl1_640_480_B.Clock = RobotControl1_640_480_M->Timing.t[0];

  /* Outputs for Enabled SubSystem: '<S3>/Read' incorporates:
   *  EnablePort: '<S41>/Enable'
   */
  if (RobotControl1_640_480_B.enable > 0.0) {
    if (!RobotControl1_640_480_DW.Read_MODE) {
      RobotControl1_640_480_DW.Read_MODE = true;
    }
  } else {
    if (RobotControl1_640_480_DW.Read_MODE) {
      RobotControl1_640_480_DW.Read_MODE = false;
    }
  }

  if (RobotControl1_640_480_DW.Read_MODE) {
    /* DataStoreRead: '<S41>/Data Store Read' */
    RobotControl1_640_480_B.DataStoreRead = RobotControl1_640_480_DW.t_start;
    srUpdateBC(RobotControl1_640_480_DW.Read_SubsysRanBC);
  }

  /* End of Outputs for SubSystem: '<S3>/Read' */

  /* Switch: '<S3>/Switch' */
  if (RobotControl1_640_480_B.enable > RobotControl1_640_480_P.Switch_Threshold)
  {
    /* Sum: '<S3>/Subtract' */
    RobotControl1_640_480_B.Subtract = RobotControl1_640_480_B.Clock -
      RobotControl1_640_480_B.DataStoreRead;
    RobotControl1_640_480_B.Switch = RobotControl1_640_480_B.Subtract;
  } else {
    /* Sum: '<S3>/Subtract1' */
    RobotControl1_640_480_B.Subtract1 = RobotControl1_640_480_B.Clock -
      RobotControl1_640_480_B.Clock;
    RobotControl1_640_480_B.Switch = RobotControl1_640_480_B.Subtract1;
  }

  /* End of Switch: '<S3>/Switch' */

  /* MATLAB Function: '<S10>/Desired Pose V2' incorporates:
   *  Constant: '<S10>/T'
   */
  s = RobotControl1_640_480_B.Switch;

  /* MATLAB Function 'Open-loop control/Vision System/Desired Pose V2': '<S14>:1' */
  /*      T = 36; */
  if (RobotControl1_640_480_B.Switch > RobotControl1_640_480_P.T_Value) {
    /* '<S14>:1:8' */
    /* '<S14>:1:9' */
    s = RobotControl1_640_480_P.T_Value;
  }

  /* Position/Time */
  /* '<S14>:1:13' */
  /* '<S14>:1:14' */
  /* '<S14>:1:15' */
  /* '<S14>:1:16' */
  /* '<S14>:1:17' */
  /* '<S14>:1:19' */
  xrdot = sin(6.2831853071795862 * s / RobotControl1_640_480_P.T_Value) *
    191.63715186897738 / (20.0 * RobotControl1_640_480_P.T_Value);

  /* '<S14>:1:23' */
  /*  desired second derivatives */
  /* '<S14>:1:26' */
  /* '<S14>:1:27' */
  if (xrdot == 0.0) {
    /* '<S14>:1:29' */
    /* '<S14>:1:30' */
    xrdot = 0.01;
  }

  /*  desired theta derivative */
  /* '<S14>:1:34' */
  /* sqrt(xrdot^2+yrdot^2); */
  /* '<S14>:1:39' */
  RobotControl1_640_480_B.qr[0] = -2.45 - cos(6.2831853071795862 * s /
    RobotControl1_640_480_P.T_Value) * 1.55;
  RobotControl1_640_480_B.qr[1] = 0.0;
  RobotControl1_640_480_B.qr[2] = 0.25;
  RobotControl1_640_480_B.qr[3] = 0.0;
  RobotControl1_640_480_B.qr[4] = 0.0;
  RobotControl1_640_480_B.qr[5] = 0.0;

  /* '<S14>:1:40' */
  RobotControl1_640_480_B.ur[0] = xrdot;
  RobotControl1_640_480_B.ur[1] = (0.0 - cos(6.2831853071795862 * s /
    RobotControl1_640_480_P.T_Value) * 602.04586846645088 /
    (RobotControl1_640_480_P.T_Value * RobotControl1_640_480_P.T_Value * 10.0) *
    0.0) / xrdot;

  /* End of MATLAB Function: '<S10>/Desired Pose V2' */

  /* MATLAB Function: '<S10>/Pose Estimation' incorporates:
   *  Constant: '<S10>/Object Model'
   */
  /* MATLAB Function 'Open-loop control/Vision System/Pose Estimation': '<S17>:1' */
  /* Desired Robot Pose for current step qr() */
  /* '<S17>:1:5' */
  /* '<S17>:1:6' */
  /* '<S17>:1:7' */
  /* '<S17>:1:8' */
  /* '<S17>:1:9' */
  /* '<S17>:1:10' */
  /* Transform target points from I space into Desired Robot Space */
  /* '<S17>:1:13' */
  world2robotDes_0[0] = 1.0;
  world2robotDes_0[4] = 0.0;
  world2robotDes_0[8] = 0.0;
  world2robotDes_0[12] = RobotControl1_640_480_B.qr[0];
  world2robotDes_0[1] = 0.0;
  world2robotDes_0[5] = 1.0;
  world2robotDes_0[9] = 0.0;
  world2robotDes_0[13] = RobotControl1_640_480_B.qr[1];
  world2robotDes_0[2] = 0.0;
  world2robotDes_0[6] = 0.0;
  world2robotDes_0[10] = 1.0;
  world2robotDes_0[14] = RobotControl1_640_480_B.qr[2];
  world2robotDes_0[3] = 0.0;
  world2robotDes_0[7] = 0.0;
  world2robotDes_0[11] = 0.0;
  world2robotDes_0[15] = 1.0;
  world2robotDes[0] = 1.0;
  world2robotDes[4] = 0.0;
  world2robotDes[8] = 0.0;
  world2robotDes[12] = 0.0;
  world2robotDes[1] = 0.0;
  world2robotDes[5] = cos(RobotControl1_640_480_B.qr[3]);
  world2robotDes[9] = -sin(RobotControl1_640_480_B.qr[3]);
  world2robotDes[13] = 0.0;
  world2robotDes[2] = 0.0;
  world2robotDes[6] = sin(RobotControl1_640_480_B.qr[3]);
  world2robotDes[10] = cos(RobotControl1_640_480_B.qr[3]);
  world2robotDes[14] = 0.0;
  world2robotDes[3] = 0.0;
  world2robotDes[7] = 0.0;
  world2robotDes[11] = 0.0;
  world2robotDes[15] = 1.0;
  for (i_0 = 0; i_0 < 4; i_0++) {
    for (loop_ub = 0; loop_ub < 4; loop_ub++) {
      tmp_0[i_0 + (loop_ub << 2)] = 0.0;
      tmp_0[i_0 + (loop_ub << 2)] += world2robotDes[loop_ub << 2] *
        world2robotDes_0[i_0];
      tmp_0[i_0 + (loop_ub << 2)] += world2robotDes[(loop_ub << 2) + 1] *
        world2robotDes_0[i_0 + 4];
      tmp_0[i_0 + (loop_ub << 2)] += world2robotDes[(loop_ub << 2) + 2] *
        world2robotDes_0[i_0 + 8];
      tmp_0[i_0 + (loop_ub << 2)] += world2robotDes[(loop_ub << 2) + 3] *
        world2robotDes_0[i_0 + 12];
    }
  }

  tmp_1[0] = cos(RobotControl1_640_480_B.qr[4]);
  tmp_1[4] = 0.0;
  tmp_1[8] = sin(RobotControl1_640_480_B.qr[4]);
  tmp_1[12] = 0.0;
  tmp_1[1] = 0.0;
  tmp_1[5] = 1.0;
  tmp_1[9] = 0.0;
  tmp_1[13] = 0.0;
  tmp_1[2] = -sin(RobotControl1_640_480_B.qr[4]);
  tmp_1[6] = 0.0;
  tmp_1[10] = cos(RobotControl1_640_480_B.qr[4]);
  tmp_1[14] = 0.0;
  tmp_1[3] = 0.0;
  tmp_1[7] = 0.0;
  tmp_1[11] = 0.0;
  tmp_1[15] = 1.0;
  for (i_0 = 0; i_0 < 4; i_0++) {
    for (loop_ub = 0; loop_ub < 4; loop_ub++) {
      world2robotDes_0[i_0 + (loop_ub << 2)] = 0.0;
      world2robotDes_0[i_0 + (loop_ub << 2)] += tmp_1[loop_ub << 2] * tmp_0[i_0];
      world2robotDes_0[i_0 + (loop_ub << 2)] += tmp_1[(loop_ub << 2) + 1] *
        tmp_0[i_0 + 4];
      world2robotDes_0[i_0 + (loop_ub << 2)] += tmp_1[(loop_ub << 2) + 2] *
        tmp_0[i_0 + 8];
      world2robotDes_0[i_0 + (loop_ub << 2)] += tmp_1[(loop_ub << 2) + 3] *
        tmp_0[i_0 + 12];
    }
  }

  tmp_2[0] = cos(RobotControl1_640_480_B.qr[5]);
  tmp_2[4] = -sin(RobotControl1_640_480_B.qr[5]);
  tmp_2[8] = 0.0;
  tmp_2[12] = 0.0;
  tmp_2[1] = sin(RobotControl1_640_480_B.qr[5]);
  tmp_2[5] = cos(RobotControl1_640_480_B.qr[5]);
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
    for (loop_ub = 0; loop_ub < 4; loop_ub++) {
      poseErrorMat[i_0 + (loop_ub << 2)] = 0.0;
      poseErrorMat[i_0 + (loop_ub << 2)] += tmp_2[loop_ub << 2] *
        world2robotDes_0[i_0];
      poseErrorMat[i_0 + (loop_ub << 2)] += tmp_2[(loop_ub << 2) + 1] *
        world2robotDes_0[i_0 + 4];
      poseErrorMat[i_0 + (loop_ub << 2)] += tmp_2[(loop_ub << 2) + 2] *
        world2robotDes_0[i_0 + 8];
      poseErrorMat[i_0 + (loop_ub << 2)] += tmp_2[(loop_ub << 2) + 3] *
        world2robotDes_0[i_0 + 12];
    }
  }

  RobotControl1_640_480_invNxN(poseErrorMat, world2robotDes);

  /* '<S17>:1:14' */
  /* Calculate the error parameters for current step */
  /* '<S17>:1:17' */
  for (i_0 = 0; i_0 < 8; i_0++) {
    l_center[i_0] = RobotControl1_640_480_B.L_p[i_0];
    tmp[i_0] = RobotControl1_640_480_B.R_p[i_0];
  }

  for (i_0 = 0; i_0 < 4; i_0++) {
    for (loop_ub = 0; loop_ub < 4; loop_ub++) {
      world2robotDes_0[i_0 + (loop_ub << 2)] = 0.0;
      world2robotDes_0[i_0 + (loop_ub << 2)] +=
        RobotControl1_640_480_P.ObjectModel_Value[loop_ub << 2] *
        world2robotDes[i_0];
      world2robotDes_0[i_0 + (loop_ub << 2)] +=
        RobotControl1_640_480_P.ObjectModel_Value[(loop_ub << 2) + 1] *
        world2robotDes[i_0 + 4];
      world2robotDes_0[i_0 + (loop_ub << 2)] +=
        RobotControl1_640_480_P.ObjectModel_Value[(loop_ub << 2) + 2] *
        world2robotDes[i_0 + 8];
      world2robotDes_0[i_0 + (loop_ub << 2)] +=
        RobotControl1_640_480_P.ObjectModel_Value[(loop_ub << 2) + 3] *
        world2robotDes[i_0 + 12];
    }
  }

  RobotControl1_640_480_mirage(l_center, tmp, world2robotDes_0, errParams);

  /* '<S17>:1:19' */
  for (i_0 = 0; i_0 < 16; i_0++) {
    poseErrorMat[i_0] = b_0[i_0];
  }

  /* '<S17>:1:20' */
  memcpy(&y[0], &errParams[0], 12U * sizeof(real_T));
  for (i_0 = 0; i_0 < 4; i_0++) {
    poseErrorMat[i_0 << 2] = y[3 * i_0];
    poseErrorMat[1 + (i_0 << 2)] = y[3 * i_0 + 1];
    poseErrorMat[2 + (i_0 << 2)] = y[3 * i_0 + 2];
  }

  /* '<S17>:1:22' */
  for (i_0 = 0; i_0 < 4; i_0++) {
    for (loop_ub = 0; loop_ub < 4; loop_ub++) {
      world2robot[i_0 + (loop_ub << 2)] = 0.0;
      world2robot[i_0 + (loop_ub << 2)] += world2robotDes[loop_ub << 2] *
        poseErrorMat[i_0];
      world2robot[i_0 + (loop_ub << 2)] += world2robotDes[(loop_ub << 2) + 1] *
        poseErrorMat[i_0 + 4];
      world2robot[i_0 + (loop_ub << 2)] += world2robotDes[(loop_ub << 2) + 2] *
        poseErrorMat[i_0 + 8];
      world2robot[i_0 + (loop_ub << 2)] += world2robotDes[(loop_ub << 2) + 3] *
        poseErrorMat[i_0 + 12];
    }
  }

  /* '<S17>:1:24' */
  /*  This function get the rotation matrix and computes the euler angles. */
  /*  rotMat :(3x3) rotation matrix  */
  /*  */
  /*  roll  : angle phi */
  /*  pitch : angle theta */
  /*  yaw   : angle psi */
  s = rt_atan2d_snf(-world2robot[2], sqrt(world2robot[0] * world2robot[0] +
    world2robot[1] * world2robot[1]));

  /* '<S17>:1:26' */
  RobotControl1_640_480_B.q[0] = -world2robot[12] / 100.0;
  RobotControl1_640_480_B.q[1] = -world2robot[13] / 100.0;
  RobotControl1_640_480_B.q[2] = -world2robot[14] / 100.0;
  RobotControl1_640_480_B.q[3] = -rt_atan2d_snf(world2robot[6] / cos(s),
    world2robot[10] / cos(s));
  RobotControl1_640_480_B.q[4] = -s;
  RobotControl1_640_480_B.q[5] = -rt_atan2d_snf(world2robot[1] / cos(s),
    world2robot[0] / cos(s));

  /* Temporary Code */
  if (fabs(RobotControl1_640_480_B.q[0] - RobotControl1_640_480_B.qr[0]) > 0.8)
  {
    /* '<S17>:1:29' */
    /* '<S17>:1:30' */
    RobotControl1_640_480_B.q[0] = RobotControl1_640_480_B.qr[0];

    /* '<S17>:1:31' */
    RobotControl1_640_480_B.q[1] = RobotControl1_640_480_B.qr[1];
  }

  if (fabs(RobotControl1_640_480_B.q[5] - RobotControl1_640_480_B.qr[5]) > 0.8)
  {
    /* '<S17>:1:34' */
    /* '<S17>:1:35' */
    RobotControl1_640_480_B.q[5] = RobotControl1_640_480_B.qr[5];
  }

  /* End of MATLAB Function: '<S10>/Pose Estimation' */

  /* MATLAB Function: '<S10>/Non-Holonomic Robot Controller' incorporates:
   *  Constant: '<S10>/k'
   *  Constant: '<S10>/ks'
   *  Constant: '<S10>/kx'
   */
  /*  qtilde = [ -robotPoseEst.x/100  - robotPoseDes.x; */
  /*             -robotPoseEst.y/100  - robotPoseDes.y; */
  /*             -robotPoseEst.z/100  - robotPoseDes.z; */
  /*             -robotPoseEst.phi  - robotPoseDes.phi; */
  /*             -robotPoseEst.theta  - robotPoseDes.theta; */
  /*             -robotPoseEst.psi  - robotPoseDes.psi;]; */
  /* MATLAB Function 'Open-loop control/Vision System/Non-Holonomic Robot Controller': '<S16>:1' */
  /*  the controller */
  /*  u = [v omega]^T */
  /*  qtilde: posture error */
  /*  qr    : reference (desired) posture */
  /* %----------Input Parametleri Duzenleniyor--------------------------------- */
  /*  for now */
  /* '<S16>:1:12' */
  /* '<S16>:1:13' */
  /*  According to the vision system: qtilde = q - qr, so */
  /*  q = qtilde + qr; */
  /* '<S16>:1:18' */
  /* '<S16>:1:19' */
  /* '<S16>:1:20' */
  /* '<S16>:1:22' */
  /* '<S16>:1:23' */
  /* '<S16>:1:24' */
  /* '<S16>:1:26' */
  s = sin(RobotControl1_640_480_B.q[5]);

  /* '<S16>:1:27' */
  c = cos(RobotControl1_640_480_B.q[5]);

  /* % -------------------------Error Hesaplaniyor---------------------------- */
  /*  Burada temelde 3 tane olan hata parametreleri 4 e cikariliyor.  */
  /*  theta parametresi es ve ec olarak iki ayri hata parametresine cevriliyor */
  /*  Eq.(4), third element! */
  /* '<S16>:1:34' */
  /*  Local errors from Eq.(10) */
  /* '<S16>:1:37' */
  /* '<S16>:1:38' */
  /* '<S16>:1:40' */
  /* '<S16>:1:41' */
  xrdot = cos(RobotControl1_640_480_B.qr[5] - RobotControl1_640_480_B.q[5]);

  /* '<S16>:1:43' */
  /* -------------------------------------------------------------------------- */
  /* % - u ya ulasabilmek icin ihtiyacimiz olanlar */
  /*  vb,omegab,vr,omegar: ilk ikisi hesaplaniyor son ikisi ur'den geliyor. */
  /*  k,kx,ks,a,n paper temel alinarak secilmis secilmis */
  /*  according to the paper */
  /*  k = 0.8; % k > 0 */
  /*  kx = 0.2; % kx(t) > 0 */
  /*  ks = 0.8; % ks(t) > 0 */
  /*  a > 2 */
  /*  n = -2, -1, 0, 1 or 2 */
  /*  Eq.(18) */
  /* '<S16>:1:58' */
  /* '<S16>:1:59' */
  y_0 = (xrdot - 1.0) / 3.0;
  xrdot = (xrdot - 1.0) / 3.0;

  /* '<S16>:1:61' */
  RobotControl1_640_480_B.v = ((RobotControl1_640_480_B.qr[0] -
    RobotControl1_640_480_B.q[0]) * c + (RobotControl1_640_480_B.qr[1] -
    RobotControl1_640_480_B.q[1]) * s) * RobotControl1_640_480_P.kx_Value + cos
    (RobotControl1_640_480_B.qr[5] - RobotControl1_640_480_B.q[5]) *
    RobotControl1_640_480_B.ur[0];

  /* '<S16>:1:62' */
  RobotControl1_640_480_B.omega = (((RobotControl1_640_480_B.qr[0] -
    RobotControl1_640_480_B.q[0]) * -s + (RobotControl1_640_480_B.qr[1] -
    RobotControl1_640_480_B.q[1]) * c) * (RobotControl1_640_480_P.k_Value *
    RobotControl1_640_480_B.ur[0]) * ((1.0 + y_0) * (1.0 + y_0)) + sin
    (RobotControl1_640_480_B.qr[5] - RobotControl1_640_480_B.q[5]) *
    RobotControl1_640_480_P.ks_Value * rt_powd_snf((1.0 + xrdot) * (1.0 + xrdot),
    0.0)) + RobotControl1_640_480_B.ur[1];

  /* MATLAB Function: '<S1>/PWM Calculation' incorporates:
   *  Constant: '<S1>/PWM_L'
   *  Constant: '<S1>/PWM_R'
   */
  /*  Eq.(6) */
  /* buradaki u degeri controllerdan cikan ve sisteme giris olarak verilen u */
  /*  out = [xr x yr y ex vr omegab omegar]; */
  xrdot = RobotControl1_640_480_B.v;
  s = RobotControl1_640_480_B.omega;

  /* MATLAB Function 'Open-loop control/PWM Calculation': '<S9>:1' */
  if (RobotControl1_640_480_B.enable == -1.0) {
    /* '<S9>:1:4' */
    /* '<S9>:1:5' */
    xrdot = 0.0;

    /* '<S9>:1:6' */
    s = 0.0;
  }

  /* '<S9>:1:10' */
  c = xrdot - 0.211 * s;

  /* '<S9>:1:11' */
  s = 0.211 * s + xrdot;

  /* ---------------------------------- */
  /* Calculations for left wheel motor */
  if (c < -1.7) {
    /* '<S9>:1:15' */
    /* '<S9>:1:16' */
    RobotControl1_640_480_B.PWM_L = 1000.0;
  } else if (c < 0.0) {
    /* '<S9>:1:18' */
    /* function for the negative area */
    /* '<S9>:1:20' */
    RobotControl1_640_480_B.PWM_L = ((c * c * 264.6006 + 70.971 * rt_powd_snf(c,
      3.0)) + 479.6339 * c) + 1395.7533;

    /*  elseif(vL < 0.04) */
    /*      PWM_L = 1500; */
  } else if (c < 1.7) {
    /* '<S9>:1:25' */
    /* function for the positive area */
    /* '<S9>:1:27' */
    RobotControl1_640_480_B.PWM_L = (((c * c * -181.5969 + 45.3547 * rt_powd_snf
      (c, 3.0)) + 411.962 * c) + 1590.6833) +
      RobotControl1_640_480_P.PWM_L_Value;
  } else {
    /* '<S9>:1:30' */
    RobotControl1_640_480_B.PWM_L = 2000.0;
  }

  /* ---------------------------------- */
  /* Calculations for right wheel motor */
  if (s < -1.7) {
    /* '<S9>:1:36' */
    /* '<S9>:1:37' */
    RobotControl1_640_480_B.PWM_R = 1000.0;
  } else if (s < 0.0) {
    /* '<S9>:1:39' */
    /* function for negative area */
    /* '<S9>:1:41' */
    RobotControl1_640_480_B.PWM_R = (((s * s * 266.55 + 73.2531 * rt_powd_snf(s,
      3.0)) + 478.8239 * s) + 1400.4623) - RobotControl1_640_480_P.PWM_R_Value;

    /*  elseif(vR < 0.04) */
    /*      PWM_R = 1500; */
  } else if (s < 1.7) {
    /* '<S9>:1:46' */
    /* function for positve area */
    /* '<S9>:1:48' */
    RobotControl1_640_480_B.PWM_R = ((s * s * -193.9456 + 47.0508 * rt_powd_snf
      (s, 3.0)) + 429.0898 * s) + 1593.7706;
  } else {
    /* '<S9>:1:51' */
    RobotControl1_640_480_B.PWM_R = 2000.0;
  }

  /* End of MATLAB Function: '<S1>/PWM Calculation' */

  /* MATLAB Function: '<S11>/pulse transformer1' */
  /*  v = 1950; */
  /*  PWM_L = v; */
  /*  PWM_R = v; */
  /* MATLAB Function 'Open-loop control/plant/pulse transformer1': '<S37>:1' */
  /*  This block supports an embeddable subset of the MATLAB language. */
  /*  Converts output angle and speed value into a pulse width */
  /*  The input angle is between +pi/4 and -pi/4  (from limits) */
  /*  The speed is between 140 and 0 rps(Hz) */
  /*  angle=uint16(angI*16.25*180/pi + 1505.5); */
  /*  pw=-0.0002*speedI^3 +0.03*speedI^2 -2.66*speedI +1528; */
  /*  throtle = uint16(pw); */
  /*   */
  /* '<S37>:1:11' */
  s = rt_roundd_snf(RobotControl1_640_480_B.PWM_L);
  if (s < 65536.0) {
    if (s >= 0.0) {
      cksum0 = (uint16_T)s;
    } else {
      cksum0 = 0U;
    }
  } else {
    cksum0 = MAX_uint16_T;
  }

  RobotControl1_640_480_B.Left_PWM = cksum0;

  /* '<S37>:1:12' */
  s = rt_roundd_snf(RobotControl1_640_480_B.PWM_R);
  if (s < 65536.0) {
    if (s >= 0.0) {
      cksum0 = (uint16_T)s;
    } else {
      cksum0 = 0U;
    }
  } else {
    cksum0 = MAX_uint16_T;
  }

  RobotControl1_640_480_B.Right_PWM = cksum0;

  /* End of MATLAB Function: '<S11>/pulse transformer1' */

  /* MATLAB Function: '<S1>/Camera Servos Function' */
  /* MATLAB Function 'Open-loop control/Camera Servos Function': '<S6>:1' */
  /* Insert Function for camera servo manipulation here */
  /* By default, this function holds camera servos at their center position (1500 ms) */
  /* '<S6>:1:4' */
  RobotControl1_640_480_B.PAN = 1525.0;

  /* '<S6>:1:5' */
  RobotControl1_640_480_B.TILT = 1400.0;

  /* MATLAB Function: '<S11>/pulse transformer camera' */
  /* MATLAB Function 'Open-loop control/plant/pulse transformer camera': '<S36>:1' */
  /* '<S36>:1:3' */
  s = rt_roundd_snf(RobotControl1_640_480_B.PAN);
  if (s < 65536.0) {
    if (s >= 0.0) {
      cksum0 = (uint16_T)s;
    } else {
      cksum0 = 0U;
    }
  } else {
    cksum0 = MAX_uint16_T;
  }

  RobotControl1_640_480_B.PAN_O = cksum0;

  /* '<S36>:1:4' */
  s = rt_roundd_snf(RobotControl1_640_480_B.TILT);
  if (s < 65536.0) {
    if (s >= 0.0) {
      cksum0 = (uint16_T)s;
    } else {
      cksum0 = 0U;
    }
  } else {
    cksum0 = MAX_uint16_T;
  }

  RobotControl1_640_480_B.TILT_O = cksum0;

  /* End of MATLAB Function: '<S11>/pulse transformer camera' */

  /* Outputs for Atomic SubSystem: '<S11>/Send Serial Packet to Servo Controller And recieve status message from SSC' */

  /* ReverseEndian: <S35>/Byte Reversal */

  /* 2 byte-wide input datatypes */
  ((uint16_T *)&RobotControl1_640_480_B.ByteReversal_o1)[0] =
    SWAP16(((uint16_T *)&RobotControl1_640_480_P.Servo1_Value)[0]);
  ((uint16_T *)&RobotControl1_640_480_B.ByteReversal_o2)[0] =
    SWAP16(((uint16_T *)&RobotControl1_640_480_B.Left_PWM)[0]);
  ((uint16_T *)&RobotControl1_640_480_B.ByteReversal_o3)[0] =
    SWAP16(((uint16_T *)&RobotControl1_640_480_B.Right_PWM)[0]);
  ((uint16_T *)&RobotControl1_640_480_B.ByteReversal_o4)[0] =
    SWAP16(((uint16_T *)&RobotControl1_640_480_P.Servo1_Value)[0]);
  ((uint16_T *)&RobotControl1_640_480_B.ByteReversal_o5)[0] =
    SWAP16(((uint16_T *)&RobotControl1_640_480_B.PAN_O)[0]);
  ((uint16_T *)&RobotControl1_640_480_B.ByteReversal_o6)[0] =
    SWAP16(((uint16_T *)&RobotControl1_640_480_P.Servo1_Value)[0]);
  ((uint16_T *)&RobotControl1_640_480_B.ByteReversal_o7)[0] =
    SWAP16(((uint16_T *)&RobotControl1_640_480_B.TILT_O)[0]);
  ((uint16_T *)&RobotControl1_640_480_B.ByteReversal_o8)[0] =
    SWAP16(((uint16_T *)&RobotControl1_640_480_P.Servo1_Value)[0]);

  /* MATLAB Function: '<S35>/CheckSum' incorporates:
   *  Constant: '<S11>/Servo1'
   *  Constant: '<S35>/Count'
   *  Constant: '<S35>/ID'
   */
  /* MATLAB Function 'Open-loop control/plant/Send Serial Packet to Servo Controller And recieve status message from SSC/CheckSum': '<S38>:1' */
  /*  The purpose of this function is to calculate the fletcher checksum values */
  /*  as defined in internet RFC 1145 */
  /*  See page 11 of the Microbotics Servo Switch/Controller Users Manual for  */
  /*  more information */
  /*  break into bytes */
  /* '<S38>:1:8' */
  servo1a = (uint8_T)((uint32_T)RobotControl1_640_480_P.Servo1_Value >> 8);

  /* '<S38>:1:8' */
  /* '<S38>:1:9' */
  servo2a = (uint8_T)((uint32_T)RobotControl1_640_480_B.Left_PWM >> 8);

  /* '<S38>:1:9' */
  /* '<S38>:1:10' */
  servo3a = (uint8_T)((uint32_T)RobotControl1_640_480_B.Right_PWM >> 8);

  /* '<S38>:1:10' */
  /* '<S38>:1:11' */
  servo4a = (uint8_T)((uint32_T)RobotControl1_640_480_P.Servo1_Value >> 8);

  /* '<S38>:1:11' */
  /* '<S38>:1:12' */
  servo5a = (uint8_T)((uint32_T)RobotControl1_640_480_B.PAN_O >> 8);

  /* '<S38>:1:12' */
  /* '<S38>:1:13' */
  servo6a = (uint8_T)((uint32_T)RobotControl1_640_480_P.Servo1_Value >> 8);

  /* '<S38>:1:13' */
  /* '<S38>:1:14' */
  servo7a = (uint8_T)((uint32_T)RobotControl1_640_480_B.TILT_O >> 8);

  /* '<S38>:1:14' */
  /* '<S38>:1:15' */
  servo8a = (uint8_T)((uint32_T)RobotControl1_640_480_P.Servo1_Value >> 8);

  /* '<S38>:1:15' */
  /* '<S38>:1:16' */
  byte[0] = RobotControl1_640_480_P.ID_Value;
  byte[1] = RobotControl1_640_480_P.Count_Value;
  byte[2] = servo1a;
  i = RobotControl1_640_480_P.Servo1_Value;
  qY = (uint32_T)i - (servo1a << 8);
  if (qY > (uint32_T)i) {
    qY = 0U;
  }

  i_0 = (int32_T)qY;
  cksum0 = (uint16_T)i_0;
  if (cksum0 > 255) {
    cksum0 = 255U;
  }

  byte[3] = (uint8_T)cksum0;
  byte[4] = servo2a;
  i = RobotControl1_640_480_B.Left_PWM;
  qY = (uint32_T)i - (servo2a << 8);
  if (qY > (uint32_T)i) {
    qY = 0U;
  }

  i_0 = (int32_T)qY;
  cksum0 = (uint16_T)i_0;
  if (cksum0 > 255) {
    cksum0 = 255U;
  }

  byte[5] = (uint8_T)cksum0;
  byte[6] = servo3a;
  i = RobotControl1_640_480_B.Right_PWM;
  qY = (uint32_T)i - (servo3a << 8);
  if (qY > (uint32_T)i) {
    qY = 0U;
  }

  i_0 = (int32_T)qY;
  cksum0 = (uint16_T)i_0;
  if (cksum0 > 255) {
    cksum0 = 255U;
  }

  byte[7] = (uint8_T)cksum0;
  byte[8] = servo4a;
  i = RobotControl1_640_480_P.Servo1_Value;
  qY = (uint32_T)i - (servo4a << 8);
  if (qY > (uint32_T)i) {
    qY = 0U;
  }

  i_0 = (int32_T)qY;
  cksum0 = (uint16_T)i_0;
  if (cksum0 > 255) {
    cksum0 = 255U;
  }

  byte[9] = (uint8_T)cksum0;
  byte[10] = servo5a;
  i = RobotControl1_640_480_B.PAN_O;
  qY = (uint32_T)i - (servo5a << 8);
  if (qY > (uint32_T)i) {
    qY = 0U;
  }

  i_0 = (int32_T)qY;
  cksum0 = (uint16_T)i_0;
  if (cksum0 > 255) {
    cksum0 = 255U;
  }

  byte[11] = (uint8_T)cksum0;
  byte[12] = servo6a;
  i = RobotControl1_640_480_P.Servo1_Value;
  qY = (uint32_T)i - (servo6a << 8);
  if (qY > (uint32_T)i) {
    qY = 0U;
  }

  i_0 = (int32_T)qY;
  cksum0 = (uint16_T)i_0;
  if (cksum0 > 255) {
    cksum0 = 255U;
  }

  byte[13] = (uint8_T)cksum0;
  byte[14] = servo7a;
  i = RobotControl1_640_480_B.TILT_O;
  qY = (uint32_T)i - (servo7a << 8);
  if (qY > (uint32_T)i) {
    qY = 0U;
  }

  i_0 = (int32_T)qY;
  cksum0 = (uint16_T)i_0;
  if (cksum0 > 255) {
    cksum0 = 255U;
  }

  byte[15] = (uint8_T)cksum0;
  byte[16] = servo8a;
  i = RobotControl1_640_480_P.Servo1_Value;
  qY = (uint32_T)i - (servo8a << 8);
  if (qY > (uint32_T)i) {
    qY = 0U;
  }

  i_0 = (int32_T)qY;
  cksum0 = (uint16_T)i_0;
  if (cksum0 > 255) {
    cksum0 = 255U;
  }

  byte[17] = (uint8_T)cksum0;

  /* '<S38>:1:18' */
  cksum0 = 0U;

  /* '<S38>:1:18' */
  cksum1 = 0U;

  /*  initialize to zero */
  /* '<S38>:1:19' */
  i_0 = (int32_T)(RobotControl1_640_480_P.Count_Value + 2U);
  if ((uint32_T)i_0 > 255U) {
    i_0 = 255;
  }

  servo1a = (uint8_T)i_0;

  /* '<S38>:1:19' */
  for (servo2a = 1U; servo2a <= servo1a; servo2a++) {
    /* '<S38>:1:19' */
    /* '<S38>:1:20' */
    qY = (uint32_T)byte[servo2a - 1] + cksum0;
    if (qY > 65535U) {
      qY = 65535U;
    }

    x = (uint16_T)qY;
    cksum0 = (uint16_T)((uint32_T)x - ((int32_T)((uint32_T)x >> 8) << 8));

    /* '<S38>:1:21' */
    qY = (uint32_T)cksum1 + cksum0;
    if (qY > 65535U) {
      qY = 65535U;
    }

    x = (uint16_T)qY;
    cksum1 = (uint16_T)((uint32_T)x - ((int32_T)((uint32_T)x >> 8) << 8));
  }

  /* '<S38>:1:24' */
  if (cksum0 > 255) {
    cksum0 = 255U;
  }

  RobotControl1_640_480_B.csum0 = (uint8_T)cksum0;

  /* '<S38>:1:24' */
  if (cksum1 > 255) {
    cksum1 = 255U;
  }

  RobotControl1_640_480_B.csum1 = (uint8_T)cksum1;

  /* End of MATLAB Function: '<S35>/CheckSum' */

  /* Pack: <S35>/Pack */
  (void) memcpy(&RobotControl1_640_480_B.Pack[0],
                &RobotControl1_640_480_P.SYNC0_Value,
                1);
  (void) memcpy(&RobotControl1_640_480_B.Pack[1],
                &RobotControl1_640_480_P.SYNC1_Value,
                1);
  (void) memcpy(&RobotControl1_640_480_B.Pack[2],
                &RobotControl1_640_480_P.ID_Value,
                1);
  (void) memcpy(&RobotControl1_640_480_B.Pack[3],
                &RobotControl1_640_480_P.Count_Value,
                1);
  (void) memcpy(&RobotControl1_640_480_B.Pack[4],
                &RobotControl1_640_480_B.ByteReversal_o1,
                2);
  (void) memcpy(&RobotControl1_640_480_B.Pack[6],
                &RobotControl1_640_480_B.ByteReversal_o2,
                2);
  (void) memcpy(&RobotControl1_640_480_B.Pack[8],
                &RobotControl1_640_480_B.ByteReversal_o3,
                2);
  (void) memcpy(&RobotControl1_640_480_B.Pack[10],
                &RobotControl1_640_480_B.ByteReversal_o4,
                2);
  (void) memcpy(&RobotControl1_640_480_B.Pack[12],
                &RobotControl1_640_480_B.ByteReversal_o5,
                2);
  (void) memcpy(&RobotControl1_640_480_B.Pack[14],
                &RobotControl1_640_480_B.ByteReversal_o6,
                2);
  (void) memcpy(&RobotControl1_640_480_B.Pack[16],
                &RobotControl1_640_480_B.ByteReversal_o7,
                2);
  (void) memcpy(&RobotControl1_640_480_B.Pack[18],
                &RobotControl1_640_480_B.ByteReversal_o8,
                2);
  (void) memcpy(&RobotControl1_640_480_B.Pack[20],
                &RobotControl1_640_480_B.csum0,
                1);
  (void) memcpy(&RobotControl1_640_480_B.Pack[21],
                &RobotControl1_640_480_B.csum1,
                1);

  /* MATLAB Function: '<S35>/Embedded MATLAB Function' */
  /* MATLAB Function 'Open-loop control/plant/Send Serial Packet to Servo Controller And recieve status message from SSC/Embedded MATLAB Function': '<S39>:1' */
  /*  */
  /* '<S39>:1:4' */
  for (i = 0; i < 22; i++) {
    RobotControl1_640_480_B.CharArray[i] = RobotControl1_640_480_B.Pack[i];
  }

  /* End of MATLAB Function: '<S35>/Embedded MATLAB Function' */

  /* Level2 S-Function Block: '<S35>/RS232 Binary Send1' (rs232bsend) */
  {
    SimStruct *rts = RobotControl1_640_480_M->childSfunctions[0];
    sfcnOutputs(rts, 1);
  }

  /* End of Outputs for SubSystem: '<S11>/Send Serial Packet to Servo Controller And recieve status message from SSC' */
  /* ok to acquire for <S19>/S-Function */
  RobotControl1_640_480_DW.SFunction_IWORK.AcquireOK = 1;

  /* ok to acquire for <S20>/S-Function */
  RobotControl1_640_480_DW.SFunction_IWORK_m.AcquireOK = 1;

  /* ok to acquire for <S21>/S-Function */
  RobotControl1_640_480_DW.SFunction_IWORK_p.AcquireOK = 1;

  /* ok to acquire for <S22>/S-Function */
  RobotControl1_640_480_DW.SFunction_IWORK_m4.AcquireOK = 1;

  /* ok to acquire for <S23>/S-Function */
  RobotControl1_640_480_DW.SFunction_IWORK_o.AcquireOK = 1;

  /* S-Function (sfix_udelay): '<S12>/Tapped Delay' */
  RobotControl1_640_480_B.TappedDelay[0] =
    RobotControl1_640_480_DW.TappedDelay_X[0];
  RobotControl1_640_480_B.TappedDelay[1] =
    RobotControl1_640_480_DW.TappedDelay_X[1];

  /* SignalConversion: '<S12>/TmpSignal ConversionAtMeanInport1' */
  RobotControl1_640_480_B.TmpSignalConversionAtMeanInport1[0] =
    RobotControl1_640_480_B.q[0];
  RobotControl1_640_480_B.TmpSignalConversionAtMeanInport1[1] =
    RobotControl1_640_480_B.TappedDelay[0];
  RobotControl1_640_480_B.TmpSignalConversionAtMeanInport1[2] =
    RobotControl1_640_480_B.TappedDelay[1];

  /* S-Function (sdspstatfcns): '<S12>/Mean' */
  for (i = 0; i < 3; i += 3) {
    for (loop_ub = i; loop_ub < i + 1; loop_ub++) {
      RobotControl1_640_480_DW.Mean_AccVal =
        RobotControl1_640_480_B.TmpSignalConversionAtMeanInport1[loop_ub];
      indR_sizes_idx_0 = 2;
      indR_sizes_idx_1 = 1;
      while (indR_sizes_idx_0 > 0) {
        en = loop_ub + indR_sizes_idx_1;
        RobotControl1_640_480_DW.Mean_AccVal +=
          RobotControl1_640_480_B.TmpSignalConversionAtMeanInport1[en];
        indR_sizes_idx_1++;
        indR_sizes_idx_0--;
      }

      RobotControl1_640_480_B.Mean = RobotControl1_640_480_DW.Mean_AccVal / 3.0;
    }
  }

  /* End of S-Function (sdspstatfcns): '<S12>/Mean' */

  /* Sum: '<S12>/theta_tilde' */
  RobotControl1_640_480_B.etheta = RobotControl1_640_480_B.qr[5] -
    RobotControl1_640_480_B.q[5];

  /* Sum: '<S12>/xtilde' */
  RobotControl1_640_480_B.ex = RobotControl1_640_480_B.qr[0] -
    RobotControl1_640_480_B.q[0];

  /* Sum: '<S12>/ytilde' */
  RobotControl1_640_480_B.ey = RobotControl1_640_480_B.qr[1] -
    RobotControl1_640_480_B.q[1];

  /* UnaryMinus: '<S3>/Unary Minus' */
  RobotControl1_640_480_B.UnaryMinus = -RobotControl1_640_480_B.enable;

  /* Outputs for Enabled SubSystem: '<S3>/Write' incorporates:
   *  EnablePort: '<S42>/Enable'
   */
  if (RobotControl1_640_480_B.UnaryMinus > 0.0) {
    if (!RobotControl1_640_480_DW.Write_MODE) {
      RobotControl1_640_480_DW.Write_MODE = true;
    }
  } else {
    if (RobotControl1_640_480_DW.Write_MODE) {
      RobotControl1_640_480_DW.Write_MODE = false;
    }
  }

  if (RobotControl1_640_480_DW.Write_MODE) {
    /* DataStoreWrite: '<S42>/Data Store Write' */
    RobotControl1_640_480_DW.t_start = RobotControl1_640_480_B.Clock;
    srUpdateBC(RobotControl1_640_480_DW.Write_SubsysRanBC);
  }

  /* End of Outputs for SubSystem: '<S3>/Write' */

  /* Level2 S-Function Block: '<Root>/RS232 Setup ' (rs232setup) */
  {
    SimStruct *rts = RobotControl1_640_480_M->childSfunctions[4];
    sfcnOutputs(rts, 1);
  }
}

/* Model update function */
static void RobotControl1_640_480_update(void)
{
  /* Update for S-Function (sfix_udelay): '<S12>/Tapped Delay' */
  RobotControl1_640_480_DW.TappedDelay_X[0] =
    RobotControl1_640_480_DW.TappedDelay_X[1];
  RobotControl1_640_480_DW.TappedDelay_X[1] = RobotControl1_640_480_B.q[0];

  /* Update absolute time for base rate */
  /* The "clockTick0" counts the number of times the code of this task has
   * been executed. The absolute time is the multiplication of "clockTick0"
   * and "Timing.stepSize0". Size of "clockTick0" ensures timer will not
   * overflow during the application lifespan selected.
   * Timer of this task consists of two 32 bit unsigned integers.
   * The two integers represent the low bits Timing.clockTick0 and the high bits
   * Timing.clockTickH0. When the low bit overflows to 0, the high bits increment.
   */
  if (!(++RobotControl1_640_480_M->Timing.clockTick0)) {
    ++RobotControl1_640_480_M->Timing.clockTickH0;
  }

  RobotControl1_640_480_M->Timing.t[0] =
    RobotControl1_640_480_M->Timing.clockTick0 *
    RobotControl1_640_480_M->Timing.stepSize0 +
    RobotControl1_640_480_M->Timing.clockTickH0 *
    RobotControl1_640_480_M->Timing.stepSize0 * 4294967296.0;

  {
    /* Update absolute timer for sample time: [0.033333333333333333s, 0.0s] */
    /* The "clockTick1" counts the number of times the code of this task has
     * been executed. The absolute time is the multiplication of "clockTick1"
     * and "Timing.stepSize1". Size of "clockTick1" ensures timer will not
     * overflow during the application lifespan selected.
     * Timer of this task consists of two 32 bit unsigned integers.
     * The two integers represent the low bits Timing.clockTick1 and the high bits
     * Timing.clockTickH1. When the low bit overflows to 0, the high bits increment.
     */
    if (!(++RobotControl1_640_480_M->Timing.clockTick1)) {
      ++RobotControl1_640_480_M->Timing.clockTickH1;
    }

    RobotControl1_640_480_M->Timing.t[1] =
      RobotControl1_640_480_M->Timing.clockTick1 *
      RobotControl1_640_480_M->Timing.stepSize1 +
      RobotControl1_640_480_M->Timing.clockTickH1 *
      RobotControl1_640_480_M->Timing.stepSize1 * 4294967296.0;
  }
}

/* Model initialize function */
static void RobotControl1_640_480_initialize(void)
{
  /* Start for Constant: '<Root>/packet size' */
  RobotControl1_640_480_B.packetsize = RobotControl1_640_480_P.packetsize_Value;

  /* Start for Constant: '<Root>/packet size4' */
  RobotControl1_640_480_B.packetsize4 =
    RobotControl1_640_480_P.packetsize4_Value;

  /* S-Function (rs232brec): '<Root>/RS232 Binary Receive1' */

  /* Level2 S-Function Block: '<Root>/RS232 Binary Receive1' (rs232brec) */
  {
    SimStruct *rts = RobotControl1_640_480_M->childSfunctions[1];
    sfcnStart(rts);
    if (ssGetErrorStatus(rts) != (NULL))
      return;
  }

  /* End of Outputs for S-Function (rs232brec): '<Root>/RS232 Binary Receive1' */

  /* Level2 S-Function Block: '<S10>/Left Camera' (xpcusbvideoin) */
  {
    SimStruct *rts = RobotControl1_640_480_M->childSfunctions[2];
    sfcnStart(rts);
    if (ssGetErrorStatus(rts) != (NULL))
      return;
  }

  /* Level2 S-Function Block: '<S10>/Right Camera' (xpcusbvideoin) */
  {
    SimStruct *rts = RobotControl1_640_480_M->childSfunctions[3];
    sfcnStart(rts);
    if (ssGetErrorStatus(rts) != (NULL))
      return;
  }

  /* Start for Enabled SubSystem: '<S3>/Read' */
  RobotControl1_640_480_DW.Read_MODE = false;

  /* End of Start for SubSystem: '<S3>/Read' */

  /* Start for Atomic SubSystem: '<S11>/Send Serial Packet to Servo Controller And recieve status message from SSC' */

  /* Level2 S-Function Block: '<S35>/RS232 Binary Send1' (rs232bsend) */
  {
    SimStruct *rts = RobotControl1_640_480_M->childSfunctions[0];
    sfcnStart(rts);
    if (ssGetErrorStatus(rts) != (NULL))
      return;
  }

  /* End of Start for SubSystem: '<S11>/Send Serial Packet to Servo Controller And recieve status message from SSC' */

  /* S-Function Block: <S19>/S-Function (scblock) */
  {
    int i;
    if ((i = rl32eScopeExists(6)) == 0) {
      if ((i = rl32eDefScope(6,2)) != 0) {
        printf("Error creating scope 6\n");
      } else {
        rl32eAddSignal(6, rl32eGetSignalNo(
          "Open-loop control/Vision System/Desired Pose V2/p1/s6"));
        rl32eAddSignal(6, rl32eGetSignalNo(
          "Open-loop control/Vision System/Pose Estimation/s6"));
        rl32eSetScope(6, 4, 1000);
        rl32eSetScope(6, 5, 0);
        rl32eSetScope(6, 6, 1);
        rl32eSetScope(6, 0, 0);
        rl32eSetScope(6, 3, rl32eGetSignalNo(
          "Open-loop control/Vision System/Desired Pose V2/p1/s6"));
        rl32eSetScope(6, 1, 0.0);
        rl32eSetScope(6, 2, 0);
        rl32eSetScope(6, 9, 0);
        rl32eSetTargetScope(6, 1, 3.0);
        rl32eSetTargetScope(6, 11, -3.0);
        rl32eSetTargetScope(6, 10, 3.0);
        xpceScopeAcqOK(6, &RobotControl1_640_480_DW.SFunction_IWORK.AcquireOK);
      }
    }

    if (i) {
      rl32eRestartAcquisition(6);
    }
  }

  /* S-Function Block: <S20>/S-Function (scblock) */
  {
    int i;
    if ((i = rl32eScopeExists(3)) == 0) {
      if ((i = rl32eDefScope(3,2)) != 0) {
        printf("Error creating scope 3\n");
      } else {
        rl32eAddSignal(3, rl32eGetSignalNo(
          "Open-loop control/Vision System/All Scopes/xtilde"));
        rl32eAddSignal(3, rl32eGetSignalNo(
          "Open-loop control/Vision System/All Scopes/ytilde"));
        rl32eAddSignal(3, rl32eGetSignalNo(
          "Open-loop control/Vision System/All Scopes/theta_tilde"));
        rl32eSetTargetScopeSigFt(3,rl32eGetSignalNo(
          "Open-loop control/Vision System/All Scopes/xtilde"),"%15.6f");
        rl32eSetTargetScopeSigFt(3,rl32eGetSignalNo(
          "Open-loop control/Vision System/All Scopes/ytilde"),"%15.6f");
        rl32eSetTargetScopeSigFt(3,rl32eGetSignalNo(
          "Open-loop control/Vision System/All Scopes/theta_tilde"),"%15.6f");
        rl32eSetScope(3, 4, 3);
        rl32eSetScope(3, 5, 0);
        rl32eSetScope(3, 6, 1);
        rl32eSetScope(3, 0, 0);
        rl32eSetScope(3, 3, rl32eGetSignalNo(
          "Open-loop control/Vision System/All Scopes/xtilde"));
        rl32eSetScope(3, 1, 0.0);
        rl32eSetScope(3, 2, 0);
        rl32eSetScope(3, 9, 0);
        rl32eSetTargetScope(3, 1, 0.0);
        rl32eSetTargetScope(3, 11, 0.0);
        rl32eSetTargetScope(3, 10, 0.0);
        xpceScopeAcqOK(3, &RobotControl1_640_480_DW.SFunction_IWORK_m.AcquireOK);
      }
    }

    if (i) {
      rl32eRestartAcquisition(3);
    }
  }

  /* S-Function Block: <S21>/S-Function (scblock) */
  {
    int i;
    if ((i = rl32eScopeExists(2)) == 0) {
      if ((i = rl32eDefScope(2,3)) != 0) {
        printf("Error creating scope 2\n");
      } else {
        rl32eAddSignal(2, rl32eGetSignalNo(
          "Open-loop control/Vision System/Desired Pose V2/p1/s1"));
        rl32eAddSignal(2, rl32eGetSignalNo(
          "Open-loop control/Vision System/Pose Estimation/s1"));
        rl32eAddSignal(2, rl32eGetSignalNo(
          "Open-loop control/Vision System/Desired Pose V2/p1/s2"));
        rl32eAddSignal(2, rl32eGetSignalNo(
          "Open-loop control/Vision System/Pose Estimation/s2"));
        rl32eAddSignal(2, rl32eGetSignalNo(
          "Open-loop control/Vision System/Desired Pose V2/p1/s6"));
        rl32eAddSignal(2, rl32eGetSignalNo(
          "Open-loop control/Vision System/Pose Estimation/s6"));
        rl32eAddSignal(2, rl32eGetSignalNo(
          "Open-loop control/Vision System/All Scopes/Mean"));
        rl32eSetScope(2, 4, 6000);
        rl32eSetScope(2, 5, 0);
        rl32eSetScope(2, 6, 1);
        rl32eSetScope(2, 0, 0);
        xpceFSScopeSet(2, "q.dat", 0, 512, 0, 536870912);
        rl32eSetScope (2, 10, 0);
        rl32eSetScope(2, 3, rl32eGetSignalNo(
          "Open-loop control/Vision System/Desired Pose V2/p1/s1"));
        rl32eSetScope(2, 1, 0.0);
        rl32eSetScope(2, 2, 0);
        rl32eSetScope(2, 9, 0);
        xpceScopeAcqOK(2, &RobotControl1_640_480_DW.SFunction_IWORK_p.AcquireOK);
      }
    }

    if (i) {
      rl32eRestartAcquisition(2);
    }
  }

  /* S-Function Block: <S22>/S-Function (scblock) */
  {
    int i;
    if ((i = rl32eScopeExists(1)) == 0) {
      if ((i = rl32eDefScope(1,2)) != 0) {
        printf("Error creating scope 1\n");
      } else {
        rl32eAddSignal(1, rl32eGetSignalNo(
          "Open-loop control/Vision System/Desired Pose V2/p1/s1"));
        rl32eAddSignal(1, rl32eGetSignalNo(
          "Open-loop control/Vision System/Pose Estimation/s1"));
        rl32eAddSignal(1, rl32eGetSignalNo(
          "Open-loop control/Vision System/All Scopes/Mean"));
        rl32eSetScope(1, 4, 1000);
        rl32eSetScope(1, 5, 0);
        rl32eSetScope(1, 6, 1);
        rl32eSetScope(1, 0, 0);
        rl32eSetScope(1, 3, rl32eGetSignalNo(
          "Open-loop control/Vision System/Desired Pose V2/p1/s1"));
        rl32eSetScope(1, 1, 0.0);
        rl32eSetScope(1, 2, 0);
        rl32eSetScope(1, 9, 0);
        rl32eSetTargetScope(1, 1, 3.0);
        rl32eSetTargetScope(1, 11, -4.5);
        rl32eSetTargetScope(1, 10, 0.5);
        xpceScopeAcqOK(1, &RobotControl1_640_480_DW.SFunction_IWORK_m4.AcquireOK);
      }
    }

    if (i) {
      rl32eRestartAcquisition(1);
    }
  }

  /* S-Function Block: <S23>/S-Function (scblock) */
  {
    int i;
    if ((i = rl32eScopeExists(4)) == 0) {
      if ((i = rl32eDefScope(4,2)) != 0) {
        printf("Error creating scope 4\n");
      } else {
        rl32eAddSignal(4, rl32eGetSignalNo(
          "Open-loop control/Vision System/Desired Pose V2/p1/s2"));
        rl32eAddSignal(4, rl32eGetSignalNo(
          "Open-loop control/Vision System/Pose Estimation/s2"));
        rl32eSetScope(4, 4, 1000);
        rl32eSetScope(4, 5, 0);
        rl32eSetScope(4, 6, 1);
        rl32eSetScope(4, 0, 0);
        rl32eSetScope(4, 3, rl32eGetSignalNo(
          "Open-loop control/Vision System/Desired Pose V2/p1/s2"));
        rl32eSetScope(4, 1, 0.0);
        rl32eSetScope(4, 2, 0);
        rl32eSetScope(4, 9, 0);
        rl32eSetTargetScope(4, 1, 3.0);
        rl32eSetTargetScope(4, 11, -2.0);
        rl32eSetTargetScope(4, 10, 2.0);
        xpceScopeAcqOK(4, &RobotControl1_640_480_DW.SFunction_IWORK_o.AcquireOK);
      }
    }

    if (i) {
      rl32eRestartAcquisition(4);
    }
  }

  /* Start for Enabled SubSystem: '<S3>/Write' */
  RobotControl1_640_480_DW.Write_MODE = false;

  /* End of Start for SubSystem: '<S3>/Write' */

  /* Start for DataStoreMemory: '<S3>/Data Store Memory' */
  RobotControl1_640_480_DW.t_start =
    RobotControl1_640_480_P.DataStoreMemory_InitialValue;

  /* Level2 S-Function Block: '<Root>/RS232 Setup ' (rs232setup) */
  {
    SimStruct *rts = RobotControl1_640_480_M->childSfunctions[4];
    sfcnStart(rts);
    if (ssGetErrorStatus(rts) != (NULL))
      return;
  }

  /* InitializeConditions for S-Function (sfix_udelay): '<S12>/Tapped Delay' */
  RobotControl1_640_480_DW.TappedDelay_X[0] =
    RobotControl1_640_480_P.TappedDelay_vinit[0];
  RobotControl1_640_480_DW.TappedDelay_X[1] =
    RobotControl1_640_480_P.TappedDelay_vinit[1];
}

/* Model terminate function */
static void RobotControl1_640_480_terminate(void)
{
  /* S-Function (rs232brec): '<Root>/RS232 Binary Receive1' */

  /* Level2 S-Function Block: '<Root>/RS232 Binary Receive1' (rs232brec) */
  {
    SimStruct *rts = RobotControl1_640_480_M->childSfunctions[1];
    sfcnTerminate(rts);
  }

  /* End of Outputs for S-Function (rs232brec): '<Root>/RS232 Binary Receive1' */

  /* Level2 S-Function Block: '<S10>/Left Camera' (xpcusbvideoin) */
  {
    SimStruct *rts = RobotControl1_640_480_M->childSfunctions[2];
    sfcnTerminate(rts);
  }

  /* Level2 S-Function Block: '<S10>/Right Camera' (xpcusbvideoin) */
  {
    SimStruct *rts = RobotControl1_640_480_M->childSfunctions[3];
    sfcnTerminate(rts);
  }

  /* Terminate for Atomic SubSystem: '<S11>/Send Serial Packet to Servo Controller And recieve status message from SSC' */

  /* Level2 S-Function Block: '<S35>/RS232 Binary Send1' (rs232bsend) */
  {
    SimStruct *rts = RobotControl1_640_480_M->childSfunctions[0];
    sfcnTerminate(rts);
  }

  /* End of Terminate for SubSystem: '<S11>/Send Serial Packet to Servo Controller And recieve status message from SSC' */

  /* Level2 S-Function Block: '<Root>/RS232 Setup ' (rs232setup) */
  {
    SimStruct *rts = RobotControl1_640_480_M->childSfunctions[4];
    sfcnTerminate(rts);
  }
}

/*========================================================================*
 * Start of Classic call interface                                        *
 *========================================================================*/
void MdlOutputs(int_T tid)
{
  RobotControl1_640_480_output();
  UNUSED_PARAMETER(tid);
}

void MdlUpdate(int_T tid)
{
  RobotControl1_640_480_update();
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
  RobotControl1_640_480_initialize();
}

void MdlTerminate(void)
{
  RobotControl1_640_480_terminate();
}

/* Registration function */
RT_MODEL_RobotControl1_640_480_T *RobotControl1_640_480(void)
{
  /* Registration code */

  /* initialize non-finites */
  rt_InitInfAndNaN(sizeof(real_T));

  /* initialize real-time model */
  (void) memset((void *)RobotControl1_640_480_M, 0,
                sizeof(RT_MODEL_RobotControl1_640_480_T));

  {
    /* Setup solver object */
    rtsiSetSimTimeStepPtr(&RobotControl1_640_480_M->solverInfo,
                          &RobotControl1_640_480_M->Timing.simTimeStep);
    rtsiSetTPtr(&RobotControl1_640_480_M->solverInfo, &rtmGetTPtr
                (RobotControl1_640_480_M));
    rtsiSetStepSizePtr(&RobotControl1_640_480_M->solverInfo,
                       &RobotControl1_640_480_M->Timing.stepSize0);
    rtsiSetErrorStatusPtr(&RobotControl1_640_480_M->solverInfo,
                          (&rtmGetErrorStatus(RobotControl1_640_480_M)));
    rtsiSetRTModelPtr(&RobotControl1_640_480_M->solverInfo,
                      RobotControl1_640_480_M);
  }

  rtsiSetSimTimeStep(&RobotControl1_640_480_M->solverInfo, MAJOR_TIME_STEP);
  rtsiSetSolverName(&RobotControl1_640_480_M->solverInfo,"FixedStepDiscrete");
  RobotControl1_640_480_M->solverInfoPtr = (&RobotControl1_640_480_M->solverInfo);

  /* Initialize timing info */
  {
    int_T *mdlTsMap = RobotControl1_640_480_M->Timing.sampleTimeTaskIDArray;
    mdlTsMap[0] = 0;
    mdlTsMap[1] = 1;
    RobotControl1_640_480_M->Timing.sampleTimeTaskIDPtr = (&mdlTsMap[0]);
    RobotControl1_640_480_M->Timing.sampleTimes =
      (&RobotControl1_640_480_M->Timing.sampleTimesArray[0]);
    RobotControl1_640_480_M->Timing.offsetTimes =
      (&RobotControl1_640_480_M->Timing.offsetTimesArray[0]);

    /* task periods */
    RobotControl1_640_480_M->Timing.sampleTimes[0] = (0.0);
    RobotControl1_640_480_M->Timing.sampleTimes[1] = (0.033333333333333333);

    /* task offsets */
    RobotControl1_640_480_M->Timing.offsetTimes[0] = (0.0);
    RobotControl1_640_480_M->Timing.offsetTimes[1] = (0.0);
  }

  rtmSetTPtr(RobotControl1_640_480_M, &RobotControl1_640_480_M->Timing.tArray[0]);

  {
    int_T *mdlSampleHits = RobotControl1_640_480_M->Timing.sampleHitArray;
    mdlSampleHits[0] = 1;
    mdlSampleHits[1] = 1;
    RobotControl1_640_480_M->Timing.sampleHits = (&mdlSampleHits[0]);
  }

  rtmSetTFinal(RobotControl1_640_480_M, -1);
  RobotControl1_640_480_M->Timing.stepSize0 = 0.033333333333333333;
  RobotControl1_640_480_M->Timing.stepSize1 = 0.033333333333333333;

  /* Setup for data logging */
  {
    static RTWLogInfo rt_DataLoggingInfo;
    RobotControl1_640_480_M->rtwLogInfo = &rt_DataLoggingInfo;
  }

  /* Setup for data logging */
  {
    rtliSetLogXSignalInfo(RobotControl1_640_480_M->rtwLogInfo, (NULL));
    rtliSetLogXSignalPtrs(RobotControl1_640_480_M->rtwLogInfo, (NULL));
    rtliSetLogT(RobotControl1_640_480_M->rtwLogInfo, "tout");
    rtliSetLogX(RobotControl1_640_480_M->rtwLogInfo, "");
    rtliSetLogXFinal(RobotControl1_640_480_M->rtwLogInfo, "");
    rtliSetLogVarNameModifier(RobotControl1_640_480_M->rtwLogInfo, "rt_");
    rtliSetLogFormat(RobotControl1_640_480_M->rtwLogInfo, 0);
    rtliSetLogMaxRows(RobotControl1_640_480_M->rtwLogInfo, 1000);
    rtliSetLogDecimation(RobotControl1_640_480_M->rtwLogInfo, 1);
    rtliSetLogY(RobotControl1_640_480_M->rtwLogInfo, "");
    rtliSetLogYSignalInfo(RobotControl1_640_480_M->rtwLogInfo, (NULL));
    rtliSetLogYSignalPtrs(RobotControl1_640_480_M->rtwLogInfo, (NULL));
  }

  RobotControl1_640_480_M->solverInfoPtr = (&RobotControl1_640_480_M->solverInfo);
  RobotControl1_640_480_M->Timing.stepSize = (0.033333333333333333);
  rtsiSetFixedStepSize(&RobotControl1_640_480_M->solverInfo,
                       0.033333333333333333);
  rtsiSetSolverMode(&RobotControl1_640_480_M->solverInfo,
                    SOLVER_MODE_SINGLETASKING);

  /* block I/O */
  RobotControl1_640_480_M->ModelData.blockIO = ((void *)
    &RobotControl1_640_480_B);
  (void) memset(((void *) &RobotControl1_640_480_B), 0,
                sizeof(B_RobotControl1_640_480_T));

  /* parameters */
  RobotControl1_640_480_M->ModelData.defaultParam = ((real_T *)
    &RobotControl1_640_480_P);

  /* states (dwork) */
  RobotControl1_640_480_M->ModelData.dwork = ((void *) &RobotControl1_640_480_DW);
  (void) memset((void *)&RobotControl1_640_480_DW, 0,
                sizeof(DW_RobotControl1_640_480_T));

  /* Initialize DataMapInfo substructure containing ModelMap for C API */
  RobotControl1_640_480_InitializeDataMapInfo(RobotControl1_640_480_M);

  /* child S-Function registration */
  {
    RTWSfcnInfo *sfcnInfo = &RobotControl1_640_480_M->NonInlinedSFcns.sfcnInfo;
    RobotControl1_640_480_M->sfcnInfo = (sfcnInfo);
    rtssSetErrorStatusPtr(sfcnInfo, (&rtmGetErrorStatus(RobotControl1_640_480_M)));
    rtssSetNumRootSampTimesPtr(sfcnInfo,
      &RobotControl1_640_480_M->Sizes.numSampTimes);
    RobotControl1_640_480_M->NonInlinedSFcns.taskTimePtrs[0] = &(rtmGetTPtr
      (RobotControl1_640_480_M)[0]);
    RobotControl1_640_480_M->NonInlinedSFcns.taskTimePtrs[1] = &(rtmGetTPtr
      (RobotControl1_640_480_M)[1]);
    rtssSetTPtrPtr(sfcnInfo,
                   RobotControl1_640_480_M->NonInlinedSFcns.taskTimePtrs);
    rtssSetTStartPtr(sfcnInfo, &rtmGetTStart(RobotControl1_640_480_M));
    rtssSetTFinalPtr(sfcnInfo, &rtmGetTFinal(RobotControl1_640_480_M));
    rtssSetTimeOfLastOutputPtr(sfcnInfo, &rtmGetTimeOfLastOutput
      (RobotControl1_640_480_M));
    rtssSetStepSizePtr(sfcnInfo, &RobotControl1_640_480_M->Timing.stepSize);
    rtssSetStopRequestedPtr(sfcnInfo, &rtmGetStopRequested
      (RobotControl1_640_480_M));
    rtssSetDerivCacheNeedsResetPtr(sfcnInfo,
      &RobotControl1_640_480_M->ModelData.derivCacheNeedsReset);
    rtssSetZCCacheNeedsResetPtr(sfcnInfo,
      &RobotControl1_640_480_M->ModelData.zCCacheNeedsReset);
    rtssSetBlkStateChangePtr(sfcnInfo,
      &RobotControl1_640_480_M->ModelData.blkStateChange);
    rtssSetSampleHitsPtr(sfcnInfo, &RobotControl1_640_480_M->Timing.sampleHits);
    rtssSetPerTaskSampleHitsPtr(sfcnInfo,
      &RobotControl1_640_480_M->Timing.perTaskSampleHits);
    rtssSetSimModePtr(sfcnInfo, &RobotControl1_640_480_M->simMode);
    rtssSetSolverInfoPtr(sfcnInfo, &RobotControl1_640_480_M->solverInfoPtr);
  }

  RobotControl1_640_480_M->Sizes.numSFcns = (5);

  /* register each child */
  {
    (void) memset((void *)
                  &RobotControl1_640_480_M->NonInlinedSFcns.childSFunctions[0],
                  0,
                  5*sizeof(SimStruct));
    RobotControl1_640_480_M->childSfunctions =
      (&RobotControl1_640_480_M->NonInlinedSFcns.childSFunctionPtrs[0]);

    {
      int_T i;
      for (i = 0; i < 5; i++) {
        RobotControl1_640_480_M->childSfunctions[i] =
          (&RobotControl1_640_480_M->NonInlinedSFcns.childSFunctions[i]);
      }
    }

    /* Level2 S-Function Block: RobotControl1_640_480/<S35>/RS232 Binary Send1 (rs232bsend) */
    {
      SimStruct *rts = RobotControl1_640_480_M->childSfunctions[0];

      /* timing info */
      time_T *sfcnPeriod =
        RobotControl1_640_480_M->NonInlinedSFcns.Sfcn0.sfcnPeriod;
      time_T *sfcnOffset =
        RobotControl1_640_480_M->NonInlinedSFcns.Sfcn0.sfcnOffset;
      int_T *sfcnTsMap =
        RobotControl1_640_480_M->NonInlinedSFcns.Sfcn0.sfcnTsMap;
      (void) memset((void*)sfcnPeriod, 0,
                    sizeof(time_T)*1);
      (void) memset((void*)sfcnOffset, 0,
                    sizeof(time_T)*1);
      ssSetSampleTimePtr(rts, &sfcnPeriod[0]);
      ssSetOffsetTimePtr(rts, &sfcnOffset[0]);
      ssSetSampleTimeTaskIDPtr(rts, sfcnTsMap);

      /* Set up the mdlInfo pointer */
      {
        ssSetBlkInfo2Ptr(rts, &RobotControl1_640_480_M->
                         NonInlinedSFcns.blkInfo2[0]);
      }

      ssSetRTWSfcnInfo(rts, RobotControl1_640_480_M->sfcnInfo);

      /* Allocate memory of model methods 2 */
      {
        ssSetModelMethods2(rts,
                           &RobotControl1_640_480_M->NonInlinedSFcns.methods2[0]);
      }

      /* Allocate memory of model methods 3 */
      {
        ssSetModelMethods3(rts,
                           &RobotControl1_640_480_M->NonInlinedSFcns.methods3[0]);
      }

      /* Allocate memory for states auxilliary information */
      {
        ssSetStatesInfo2(rts,
                         &RobotControl1_640_480_M->NonInlinedSFcns.statesInfo2[0]);
      }

      /* inputs */
      {
        _ssSetNumInputPorts(rts, 1);
        ssSetPortInfoForInputs(rts,
          &RobotControl1_640_480_M->NonInlinedSFcns.Sfcn0.inputPortInfo[0]);

        /* port 0 */
        {
          ssSetInputPortRequiredContiguous(rts, 0, 1);
          ssSetInputPortSignal(rts, 0, RobotControl1_640_480_B.CharArray);
          _ssSetInputPortNumDimensions(rts, 0, 1);
          ssSetInputPortWidth(rts, 0, 22);
        }
      }

      /* path info */
      ssSetModelName(rts, "RS232\nBinary Send1");
      ssSetPath(rts,
                "RobotControl1_640_480/Open-loop control/plant/Send Serial Packet to Servo Controller And recieve status message from SSC/RS232 Binary Send1");
      ssSetRTModel(rts,RobotControl1_640_480_M);
      ssSetParentSS(rts, (NULL));
      ssSetRootSS(rts, rts);
      ssSetVersion(rts, SIMSTRUCT_VERSION_LEVEL2);

      /* parameters */
      {
        mxArray **sfcnParams = (mxArray **)
          &RobotControl1_640_480_M->NonInlinedSFcns.Sfcn0.params;
        ssSetSFcnParamsCount(rts, 3);
        ssSetSFcnParamsPtr(rts, &sfcnParams[0]);
        ssSetSFcnParam(rts, 0, (mxArray*)
                       RobotControl1_640_480_P.RS232BinarySend1_P1_Size);
        ssSetSFcnParam(rts, 1, (mxArray*)
                       RobotControl1_640_480_P.RS232BinarySend1_P2_Size);
        ssSetSFcnParam(rts, 2, (mxArray*)
                       RobotControl1_640_480_P.RS232BinarySend1_P3_Size);
      }

      /* registration */
      rs232bsend(rts);
      sfcnInitializeSizes(rts);
      sfcnInitializeSampleTimes(rts);

      /* adjust sample time */
      ssSetSampleTime(rts, 0, 0.033333333333333333);
      ssSetOffsetTime(rts, 0, 0.0);
      sfcnTsMap[0] = 1;

      /* set compiled values of dynamic vector attributes */
      ssSetNumNonsampledZCs(rts, 0);

      /* Update connectivity flags for each port */
      _ssSetInputPortConnected(rts, 0, 1);

      /* Update the BufferDstPort flags for each input port */
      ssSetInputPortBufferDstPort(rts, 0, -1);
    }

    /* Level2 S-Function Block: RobotControl1_640_480/<Root>/RS232 Binary Receive1 (rs232brec) */
    {
      SimStruct *rts = RobotControl1_640_480_M->childSfunctions[1];

      /* timing info */
      time_T *sfcnPeriod =
        RobotControl1_640_480_M->NonInlinedSFcns.Sfcn1.sfcnPeriod;
      time_T *sfcnOffset =
        RobotControl1_640_480_M->NonInlinedSFcns.Sfcn1.sfcnOffset;
      int_T *sfcnTsMap =
        RobotControl1_640_480_M->NonInlinedSFcns.Sfcn1.sfcnTsMap;
      (void) memset((void*)sfcnPeriod, 0,
                    sizeof(time_T)*1);
      (void) memset((void*)sfcnOffset, 0,
                    sizeof(time_T)*1);
      ssSetSampleTimePtr(rts, &sfcnPeriod[0]);
      ssSetOffsetTimePtr(rts, &sfcnOffset[0]);
      ssSetSampleTimeTaskIDPtr(rts, sfcnTsMap);

      /* Set up the mdlInfo pointer */
      {
        ssSetBlkInfo2Ptr(rts, &RobotControl1_640_480_M->
                         NonInlinedSFcns.blkInfo2[1]);
      }

      ssSetRTWSfcnInfo(rts, RobotControl1_640_480_M->sfcnInfo);

      /* Allocate memory of model methods 2 */
      {
        ssSetModelMethods2(rts,
                           &RobotControl1_640_480_M->NonInlinedSFcns.methods2[1]);
      }

      /* Allocate memory of model methods 3 */
      {
        ssSetModelMethods3(rts,
                           &RobotControl1_640_480_M->NonInlinedSFcns.methods3[1]);
      }

      /* Allocate memory for states auxilliary information */
      {
        ssSetStatesInfo2(rts,
                         &RobotControl1_640_480_M->NonInlinedSFcns.statesInfo2[1]);
      }

      /* inputs */
      {
        _ssSetNumInputPorts(rts, 2);
        ssSetPortInfoForInputs(rts,
          &RobotControl1_640_480_M->NonInlinedSFcns.Sfcn1.inputPortInfo[0]);

        /* port 0 */
        {
          ssSetInputPortRequiredContiguous(rts, 0, 1);
          ssSetInputPortSignal(rts, 0, &RobotControl1_640_480_B.packetsize);
          _ssSetInputPortNumDimensions(rts, 0, 1);
          ssSetInputPortWidth(rts, 0, 1);
        }

        /* port 1 */
        {
          ssSetInputPortRequiredContiguous(rts, 1, 1);
          ssSetInputPortSignal(rts, 1, &RobotControl1_640_480_B.packetsize4);
          _ssSetInputPortNumDimensions(rts, 1, 1);
          ssSetInputPortWidth(rts, 1, 1);
        }
      }

      /* outputs */
      {
        ssSetPortInfoForOutputs(rts,
          &RobotControl1_640_480_M->NonInlinedSFcns.Sfcn1.outputPortInfo[0]);
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
            RobotControl1_640_480_B.RS232BinaryReceive1_o2));
        }
      }

      /* path info */
      ssSetModelName(rts, "RS232\nBinary Receive1");
      ssSetPath(rts, "RobotControl1_640_480/RS232 Binary Receive1");
      ssSetRTModel(rts,RobotControl1_640_480_M);
      ssSetParentSS(rts, (NULL));
      ssSetRootSS(rts, rts);
      ssSetVersion(rts, SIMSTRUCT_VERSION_LEVEL2);

      /* parameters */
      {
        mxArray **sfcnParams = (mxArray **)
          &RobotControl1_640_480_M->NonInlinedSFcns.Sfcn1.params;
        ssSetSFcnParamsCount(rts, 3);
        ssSetSFcnParamsPtr(rts, &sfcnParams[0]);
        ssSetSFcnParam(rts, 0, (mxArray*)
                       RobotControl1_640_480_P.RS232BinaryReceive1_P1_Size);
        ssSetSFcnParam(rts, 1, (mxArray*)
                       RobotControl1_640_480_P.RS232BinaryReceive1_P2_Size);
        ssSetSFcnParam(rts, 2, (mxArray*)
                       RobotControl1_640_480_P.RS232BinaryReceive1_P3_Size);
      }

      /* work vectors */
      ssSetIWork(rts, (int_T *)
                 &RobotControl1_640_480_DW.RS232BinaryReceive1_IWORK[0]);

      {
        struct _ssDWorkRecord *dWorkRecord = (struct _ssDWorkRecord *)
          &RobotControl1_640_480_M->NonInlinedSFcns.Sfcn1.dWork;
        struct _ssDWorkAuxRecord *dWorkAuxRecord = (struct _ssDWorkAuxRecord *)
          &RobotControl1_640_480_M->NonInlinedSFcns.Sfcn1.dWorkAux;
        ssSetSFcnDWork(rts, dWorkRecord);
        ssSetSFcnDWorkAux(rts, dWorkAuxRecord);
        _ssSetNumDWork(rts, 2);

        /* DWORK1 */
        ssSetDWorkWidth(rts, 0, 8);
        ssSetDWorkDataType(rts, 0,SS_UINT8);
        ssSetDWorkComplexSignal(rts, 0, 0);
        ssSetDWork(rts, 0, &RobotControl1_640_480_DW.RS232BinaryReceive1_DWORK1
                   [0]);

        /* IWORK */
        ssSetDWorkWidth(rts, 1, 2);
        ssSetDWorkDataType(rts, 1,SS_INTEGER);
        ssSetDWorkComplexSignal(rts, 1, 0);
        ssSetDWork(rts, 1, &RobotControl1_640_480_DW.RS232BinaryReceive1_IWORK[0]);
      }

      /* register function-calls */
      {
        int_T *callSysOutputs = (int_T *)
          &RobotControl1_640_480_M->NonInlinedSFcns.Sfcn1.callSysOutputs;
        void **callSysArgs1 = (void **)
          &RobotControl1_640_480_M->NonInlinedSFcns.Sfcn1.callSysArgs1;
        int_T *callSysArgs2 = (int_T *)
          &RobotControl1_640_480_M->NonInlinedSFcns.Sfcn1.callSysArgs2;
        SysOutputFcn *callSysFcns = (SysOutputFcn *)
          &RobotControl1_640_480_M->NonInlinedSFcns.Sfcn1.callSysFcns;

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
        callSysArgs1[0] = (void *)RobotControl1_640_480_M;
        callSysArgs2[0] = 0;
        callSysFcns[0] = (SysOutputFcn)
          RobotControl1_640_480_StatusExtractionFNI;
        callSysFcns[1+0] = (SysOutputFcn)
          RobotControl1_640_480_StatusExtraction_InitFNI;
        callSysFcns[2+0] = (SysOutputFcn) (NULL);
        callSysFcns[3+0] = (SysOutputFcn) (NULL);
        callSysOutputs[0] = 1;
      }

      /* registration */
      rs232brec(rts);
      sfcnInitializeSizes(rts);
      sfcnInitializeSampleTimes(rts);

      /* adjust sample time */
      ssSetSampleTime(rts, 0, 0.033333333333333333);
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

    /* Level2 S-Function Block: RobotControl1_640_480/<S10>/Left Camera (xpcusbvideoin) */
    {
      SimStruct *rts = RobotControl1_640_480_M->childSfunctions[2];

      /* timing info */
      time_T *sfcnPeriod =
        RobotControl1_640_480_M->NonInlinedSFcns.Sfcn2.sfcnPeriod;
      time_T *sfcnOffset =
        RobotControl1_640_480_M->NonInlinedSFcns.Sfcn2.sfcnOffset;
      int_T *sfcnTsMap =
        RobotControl1_640_480_M->NonInlinedSFcns.Sfcn2.sfcnTsMap;
      (void) memset((void*)sfcnPeriod, 0,
                    sizeof(time_T)*1);
      (void) memset((void*)sfcnOffset, 0,
                    sizeof(time_T)*1);
      ssSetSampleTimePtr(rts, &sfcnPeriod[0]);
      ssSetOffsetTimePtr(rts, &sfcnOffset[0]);
      ssSetSampleTimeTaskIDPtr(rts, sfcnTsMap);

      /* Set up the mdlInfo pointer */
      {
        ssSetBlkInfo2Ptr(rts, &RobotControl1_640_480_M->
                         NonInlinedSFcns.blkInfo2[2]);
      }

      ssSetRTWSfcnInfo(rts, RobotControl1_640_480_M->sfcnInfo);

      /* Allocate memory of model methods 2 */
      {
        ssSetModelMethods2(rts,
                           &RobotControl1_640_480_M->NonInlinedSFcns.methods2[2]);
      }

      /* Allocate memory of model methods 3 */
      {
        ssSetModelMethods3(rts,
                           &RobotControl1_640_480_M->NonInlinedSFcns.methods3[2]);
      }

      /* Allocate memory for states auxilliary information */
      {
        ssSetStatesInfo2(rts,
                         &RobotControl1_640_480_M->NonInlinedSFcns.statesInfo2[2]);
      }

      /* outputs */
      {
        ssSetPortInfoForOutputs(rts,
          &RobotControl1_640_480_M->NonInlinedSFcns.Sfcn2.outputPortInfo[0]);
        _ssSetNumOutputPorts(rts, 1);

        /* port 0 */
        {
          int_T *dimensions = (int_T *)
            &RobotControl1_640_480_M->NonInlinedSFcns.Sfcn2.oDims0;
          dimensions[0] = 480;
          dimensions[1] = 640;
          dimensions[2] = 3;
          _ssSetOutputPortDimensionsPtr(rts, 0, dimensions);
          _ssSetOutputPortNumDimensions(rts, 0, 3);
          ssSetOutputPortWidth(rts, 0, 921600);
          ssSetOutputPortSignal(rts, 0, ((uint8_T *)
            RobotControl1_640_480_B.LeftCamera));
        }
      }

      /* path info */
      ssSetModelName(rts, "Left Camera");
      ssSetPath(rts,
                "RobotControl1_640_480/Open-loop control/Vision System/Left Camera");
      ssSetRTModel(rts,RobotControl1_640_480_M);
      ssSetParentSS(rts, (NULL));
      ssSetRootSS(rts, rts);
      ssSetVersion(rts, SIMSTRUCT_VERSION_LEVEL2);

      /* parameters */
      {
        mxArray **sfcnParams = (mxArray **)
          &RobotControl1_640_480_M->NonInlinedSFcns.Sfcn2.params;
        ssSetSFcnParamsCount(rts, 9);
        ssSetSFcnParamsPtr(rts, &sfcnParams[0]);
        ssSetSFcnParam(rts, 0, (mxArray*)
                       RobotControl1_640_480_P.LeftCamera_P1_Size);
        ssSetSFcnParam(rts, 1, (mxArray*)
                       RobotControl1_640_480_P.LeftCamera_P2_Size);
        ssSetSFcnParam(rts, 2, (mxArray*)
                       RobotControl1_640_480_P.LeftCamera_P3_Size);
        ssSetSFcnParam(rts, 3, (mxArray*)
                       RobotControl1_640_480_P.LeftCamera_P4_Size);
        ssSetSFcnParam(rts, 4, (mxArray*)
                       RobotControl1_640_480_P.LeftCamera_P5_Size);
        ssSetSFcnParam(rts, 5, (mxArray*)
                       RobotControl1_640_480_P.LeftCamera_P6_Size);
        ssSetSFcnParam(rts, 6, (mxArray*)
                       RobotControl1_640_480_P.LeftCamera_P7_Size);
        ssSetSFcnParam(rts, 7, (mxArray*)
                       RobotControl1_640_480_P.LeftCamera_P8_Size);
        ssSetSFcnParam(rts, 8, (mxArray*)
                       RobotControl1_640_480_P.LeftCamera_P9_Size);
      }

      /* work vectors */
      ssSetIWork(rts, (int_T *) &RobotControl1_640_480_DW.LeftCamera_IWORK);
      ssSetPWork(rts, (void **) &RobotControl1_640_480_DW.LeftCamera_PWORK[0]);

      {
        struct _ssDWorkRecord *dWorkRecord = (struct _ssDWorkRecord *)
          &RobotControl1_640_480_M->NonInlinedSFcns.Sfcn2.dWork;
        struct _ssDWorkAuxRecord *dWorkAuxRecord = (struct _ssDWorkAuxRecord *)
          &RobotControl1_640_480_M->NonInlinedSFcns.Sfcn2.dWorkAux;
        ssSetSFcnDWork(rts, dWorkRecord);
        ssSetSFcnDWorkAux(rts, dWorkAuxRecord);
        _ssSetNumDWork(rts, 2);

        /* IWORK */
        ssSetDWorkWidth(rts, 0, 1);
        ssSetDWorkDataType(rts, 0,SS_INTEGER);
        ssSetDWorkComplexSignal(rts, 0, 0);
        ssSetDWork(rts, 0, &RobotControl1_640_480_DW.LeftCamera_IWORK);

        /* PWORK */
        ssSetDWorkWidth(rts, 1, 3);
        ssSetDWorkDataType(rts, 1,SS_POINTER);
        ssSetDWorkComplexSignal(rts, 1, 0);
        ssSetDWork(rts, 1, &RobotControl1_640_480_DW.LeftCamera_PWORK[0]);
      }

      /* registration */
      xpcusbvideoin(rts);
      sfcnInitializeSizes(rts);
      sfcnInitializeSampleTimes(rts);

      /* adjust sample time */
      ssSetSampleTime(rts, 0, 0.033333333333333333);
      ssSetOffsetTime(rts, 0, 0.0);
      sfcnTsMap[0] = 1;

      /* set compiled values of dynamic vector attributes */
      ssSetNumNonsampledZCs(rts, 0);

      /* Update connectivity flags for each port */
      _ssSetOutputPortConnected(rts, 0, 1);
      _ssSetOutputPortBeingMerged(rts, 0, 0);

      /* Update the BufferDstPort flags for each input port */
    }

    /* Level2 S-Function Block: RobotControl1_640_480/<S10>/Right Camera (xpcusbvideoin) */
    {
      SimStruct *rts = RobotControl1_640_480_M->childSfunctions[3];

      /* timing info */
      time_T *sfcnPeriod =
        RobotControl1_640_480_M->NonInlinedSFcns.Sfcn3.sfcnPeriod;
      time_T *sfcnOffset =
        RobotControl1_640_480_M->NonInlinedSFcns.Sfcn3.sfcnOffset;
      int_T *sfcnTsMap =
        RobotControl1_640_480_M->NonInlinedSFcns.Sfcn3.sfcnTsMap;
      (void) memset((void*)sfcnPeriod, 0,
                    sizeof(time_T)*1);
      (void) memset((void*)sfcnOffset, 0,
                    sizeof(time_T)*1);
      ssSetSampleTimePtr(rts, &sfcnPeriod[0]);
      ssSetOffsetTimePtr(rts, &sfcnOffset[0]);
      ssSetSampleTimeTaskIDPtr(rts, sfcnTsMap);

      /* Set up the mdlInfo pointer */
      {
        ssSetBlkInfo2Ptr(rts, &RobotControl1_640_480_M->
                         NonInlinedSFcns.blkInfo2[3]);
      }

      ssSetRTWSfcnInfo(rts, RobotControl1_640_480_M->sfcnInfo);

      /* Allocate memory of model methods 2 */
      {
        ssSetModelMethods2(rts,
                           &RobotControl1_640_480_M->NonInlinedSFcns.methods2[3]);
      }

      /* Allocate memory of model methods 3 */
      {
        ssSetModelMethods3(rts,
                           &RobotControl1_640_480_M->NonInlinedSFcns.methods3[3]);
      }

      /* Allocate memory for states auxilliary information */
      {
        ssSetStatesInfo2(rts,
                         &RobotControl1_640_480_M->NonInlinedSFcns.statesInfo2[3]);
      }

      /* outputs */
      {
        ssSetPortInfoForOutputs(rts,
          &RobotControl1_640_480_M->NonInlinedSFcns.Sfcn3.outputPortInfo[0]);
        _ssSetNumOutputPorts(rts, 1);

        /* port 0 */
        {
          int_T *dimensions = (int_T *)
            &RobotControl1_640_480_M->NonInlinedSFcns.Sfcn3.oDims0;
          dimensions[0] = 480;
          dimensions[1] = 640;
          dimensions[2] = 3;
          _ssSetOutputPortDimensionsPtr(rts, 0, dimensions);
          _ssSetOutputPortNumDimensions(rts, 0, 3);
          ssSetOutputPortWidth(rts, 0, 921600);
          ssSetOutputPortSignal(rts, 0, ((uint8_T *)
            RobotControl1_640_480_B.RightCamera));
        }
      }

      /* path info */
      ssSetModelName(rts, "Right Camera");
      ssSetPath(rts,
                "RobotControl1_640_480/Open-loop control/Vision System/Right Camera");
      ssSetRTModel(rts,RobotControl1_640_480_M);
      ssSetParentSS(rts, (NULL));
      ssSetRootSS(rts, rts);
      ssSetVersion(rts, SIMSTRUCT_VERSION_LEVEL2);

      /* parameters */
      {
        mxArray **sfcnParams = (mxArray **)
          &RobotControl1_640_480_M->NonInlinedSFcns.Sfcn3.params;
        ssSetSFcnParamsCount(rts, 9);
        ssSetSFcnParamsPtr(rts, &sfcnParams[0]);
        ssSetSFcnParam(rts, 0, (mxArray*)
                       RobotControl1_640_480_P.RightCamera_P1_Size);
        ssSetSFcnParam(rts, 1, (mxArray*)
                       RobotControl1_640_480_P.RightCamera_P2_Size);
        ssSetSFcnParam(rts, 2, (mxArray*)
                       RobotControl1_640_480_P.RightCamera_P3_Size);
        ssSetSFcnParam(rts, 3, (mxArray*)
                       RobotControl1_640_480_P.RightCamera_P4_Size);
        ssSetSFcnParam(rts, 4, (mxArray*)
                       RobotControl1_640_480_P.RightCamera_P5_Size);
        ssSetSFcnParam(rts, 5, (mxArray*)
                       RobotControl1_640_480_P.RightCamera_P6_Size);
        ssSetSFcnParam(rts, 6, (mxArray*)
                       RobotControl1_640_480_P.RightCamera_P7_Size);
        ssSetSFcnParam(rts, 7, (mxArray*)
                       RobotControl1_640_480_P.RightCamera_P8_Size);
        ssSetSFcnParam(rts, 8, (mxArray*)
                       RobotControl1_640_480_P.RightCamera_P9_Size);
      }

      /* work vectors */
      ssSetIWork(rts, (int_T *) &RobotControl1_640_480_DW.RightCamera_IWORK);
      ssSetPWork(rts, (void **) &RobotControl1_640_480_DW.RightCamera_PWORK[0]);

      {
        struct _ssDWorkRecord *dWorkRecord = (struct _ssDWorkRecord *)
          &RobotControl1_640_480_M->NonInlinedSFcns.Sfcn3.dWork;
        struct _ssDWorkAuxRecord *dWorkAuxRecord = (struct _ssDWorkAuxRecord *)
          &RobotControl1_640_480_M->NonInlinedSFcns.Sfcn3.dWorkAux;
        ssSetSFcnDWork(rts, dWorkRecord);
        ssSetSFcnDWorkAux(rts, dWorkAuxRecord);
        _ssSetNumDWork(rts, 2);

        /* IWORK */
        ssSetDWorkWidth(rts, 0, 1);
        ssSetDWorkDataType(rts, 0,SS_INTEGER);
        ssSetDWorkComplexSignal(rts, 0, 0);
        ssSetDWork(rts, 0, &RobotControl1_640_480_DW.RightCamera_IWORK);

        /* PWORK */
        ssSetDWorkWidth(rts, 1, 3);
        ssSetDWorkDataType(rts, 1,SS_POINTER);
        ssSetDWorkComplexSignal(rts, 1, 0);
        ssSetDWork(rts, 1, &RobotControl1_640_480_DW.RightCamera_PWORK[0]);
      }

      /* registration */
      xpcusbvideoin(rts);
      sfcnInitializeSizes(rts);
      sfcnInitializeSampleTimes(rts);

      /* adjust sample time */
      ssSetSampleTime(rts, 0, 0.033333333333333333);
      ssSetOffsetTime(rts, 0, 0.0);
      sfcnTsMap[0] = 1;

      /* set compiled values of dynamic vector attributes */
      ssSetNumNonsampledZCs(rts, 0);

      /* Update connectivity flags for each port */
      _ssSetOutputPortConnected(rts, 0, 1);
      _ssSetOutputPortBeingMerged(rts, 0, 0);

      /* Update the BufferDstPort flags for each input port */
    }

    /* Level2 S-Function Block: RobotControl1_640_480/<Root>/RS232 Setup  (rs232setup) */
    {
      SimStruct *rts = RobotControl1_640_480_M->childSfunctions[4];

      /* timing info */
      time_T *sfcnPeriod =
        RobotControl1_640_480_M->NonInlinedSFcns.Sfcn4.sfcnPeriod;
      time_T *sfcnOffset =
        RobotControl1_640_480_M->NonInlinedSFcns.Sfcn4.sfcnOffset;
      int_T *sfcnTsMap =
        RobotControl1_640_480_M->NonInlinedSFcns.Sfcn4.sfcnTsMap;
      (void) memset((void*)sfcnPeriod, 0,
                    sizeof(time_T)*1);
      (void) memset((void*)sfcnOffset, 0,
                    sizeof(time_T)*1);
      ssSetSampleTimePtr(rts, &sfcnPeriod[0]);
      ssSetOffsetTimePtr(rts, &sfcnOffset[0]);
      ssSetSampleTimeTaskIDPtr(rts, sfcnTsMap);

      /* Set up the mdlInfo pointer */
      {
        ssSetBlkInfo2Ptr(rts, &RobotControl1_640_480_M->
                         NonInlinedSFcns.blkInfo2[4]);
      }

      ssSetRTWSfcnInfo(rts, RobotControl1_640_480_M->sfcnInfo);

      /* Allocate memory of model methods 2 */
      {
        ssSetModelMethods2(rts,
                           &RobotControl1_640_480_M->NonInlinedSFcns.methods2[4]);
      }

      /* Allocate memory of model methods 3 */
      {
        ssSetModelMethods3(rts,
                           &RobotControl1_640_480_M->NonInlinedSFcns.methods3[4]);
      }

      /* Allocate memory for states auxilliary information */
      {
        ssSetStatesInfo2(rts,
                         &RobotControl1_640_480_M->NonInlinedSFcns.statesInfo2[4]);
      }

      /* path info */
      ssSetModelName(rts, "RS232 Setup ");
      ssSetPath(rts, "RobotControl1_640_480/RS232 Setup ");
      ssSetRTModel(rts,RobotControl1_640_480_M);
      ssSetParentSS(rts, (NULL));
      ssSetRootSS(rts, rts);
      ssSetVersion(rts, SIMSTRUCT_VERSION_LEVEL2);

      /* parameters */
      {
        mxArray **sfcnParams = (mxArray **)
          &RobotControl1_640_480_M->NonInlinedSFcns.Sfcn4.params;
        ssSetSFcnParamsCount(rts, 19);
        ssSetSFcnParamsPtr(rts, &sfcnParams[0]);
        ssSetSFcnParam(rts, 0, (mxArray*)
                       RobotControl1_640_480_P.RS232Setup_P1_Size);
        ssSetSFcnParam(rts, 1, (mxArray*)
                       RobotControl1_640_480_P.RS232Setup_P2_Size);
        ssSetSFcnParam(rts, 2, (mxArray*)
                       RobotControl1_640_480_P.RS232Setup_P3_Size);
        ssSetSFcnParam(rts, 3, (mxArray*)
                       RobotControl1_640_480_P.RS232Setup_P4_Size);
        ssSetSFcnParam(rts, 4, (mxArray*)
                       RobotControl1_640_480_P.RS232Setup_P5_Size);
        ssSetSFcnParam(rts, 5, (mxArray*)
                       RobotControl1_640_480_P.RS232Setup_P6_Size);
        ssSetSFcnParam(rts, 6, (mxArray*)
                       RobotControl1_640_480_P.RS232Setup_P7_Size);
        ssSetSFcnParam(rts, 7, (mxArray*)
                       RobotControl1_640_480_P.RS232Setup_P8_Size);
        ssSetSFcnParam(rts, 8, (mxArray*)
                       RobotControl1_640_480_P.RS232Setup_P9_Size);
        ssSetSFcnParam(rts, 9, (mxArray*)
                       RobotControl1_640_480_P.RS232Setup_P10_Size);
        ssSetSFcnParam(rts, 10, (mxArray*)
                       RobotControl1_640_480_P.RS232Setup_P11_Size);
        ssSetSFcnParam(rts, 11, (mxArray*)
                       RobotControl1_640_480_P.RS232Setup_P12_Size);
        ssSetSFcnParam(rts, 12, (mxArray*)
                       RobotControl1_640_480_P.RS232Setup_P13_Size);
        ssSetSFcnParam(rts, 13, (mxArray*)
                       RobotControl1_640_480_P.RS232Setup_P14_Size);
        ssSetSFcnParam(rts, 14, (mxArray*)
                       RobotControl1_640_480_P.RS232Setup_P15_Size);
        ssSetSFcnParam(rts, 15, (mxArray*)
                       RobotControl1_640_480_P.RS232Setup_P16_Size);
        ssSetSFcnParam(rts, 16, (mxArray*)
                       RobotControl1_640_480_P.RS232Setup_P17_Size);
        ssSetSFcnParam(rts, 17, (mxArray*)
                       RobotControl1_640_480_P.RS232Setup_P18_Size);
        ssSetSFcnParam(rts, 18, (mxArray*)
                       RobotControl1_640_480_P.RS232Setup_P19_Size);
      }

      /* registration */
      rs232setup(rts);
      sfcnInitializeSizes(rts);
      sfcnInitializeSampleTimes(rts);

      /* adjust sample time */
      ssSetSampleTime(rts, 0, 0.033333333333333333);
      ssSetOffsetTime(rts, 0, 0.0);
      sfcnTsMap[0] = 1;

      /* set compiled values of dynamic vector attributes */
      ssSetNumNonsampledZCs(rts, 0);

      /* Update connectivity flags for each port */
      /* Update the BufferDstPort flags for each input port */
    }
  }

  /* Initialize Sizes */
  RobotControl1_640_480_M->Sizes.numContStates = (0);/* Number of continuous states */
  RobotControl1_640_480_M->Sizes.numY = (0);/* Number of model outputs */
  RobotControl1_640_480_M->Sizes.numU = (0);/* Number of model inputs */
  RobotControl1_640_480_M->Sizes.sysDirFeedThru = (0);/* The model is not direct feedthrough */
  RobotControl1_640_480_M->Sizes.numSampTimes = (2);/* Number of sample times */
  RobotControl1_640_480_M->Sizes.numBlocks = (71);/* Number of blocks */
  RobotControl1_640_480_M->Sizes.numBlockIO = (54);/* Number of block outputs */
  RobotControl1_640_480_M->Sizes.numBlockPrms = (162);/* Sum of parameter "widths" */
  return RobotControl1_640_480_M;
}

/*========================================================================*
 * End of Classic call interface                                          *
 *========================================================================*/
