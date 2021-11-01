/*
 * mirageTracking.c
 *
 * Academic License - for use in teaching, academic research, and meeting
 * course requirements at degree granting institutions only.  Not for
 * government, commercial, or other organizational use.
 *
 * Code generation for model "mirageTracking".
 *
 * Model version              : 1.198
 * Simulink Coder version : 8.8 (R2015a) 09-Feb-2015
 * C source code generated on : Mon Aug 31 14:36:40 2015
 *
 * Target selection: grt.tlc
 * Note: GRT includes extra infrastructure and instrumentation for prototyping
 * Embedded hardware selection: 32-bit Generic
 * Code generation objective: Execution efficiency
 * Validation result: Not run
 */

#include "mirageTracking.h"
#include "mirageTracking_private.h"

/* Block signals (auto storage) */
B_mirageTracking_T mirageTracking_B;

/* Block states (auto storage) */
DW_mirageTracking_T mirageTracking_DW;

/* Real-time model */
RT_MODEL_mirageTracking_T mirageTracking_M_;
RT_MODEL_mirageTracking_T *const mirageTracking_M = &mirageTracking_M_;

/* Forward declaration for local functions */
static void mirageTracking_padarray(const real32_T varargin_1[3686400], real32_T
  b[4243200]);
static void mirageTracking_round(real_T x[8]);
static void mirageTracking_invNxN(const real_T x[16], real_T y[16]);
static void mirageTra_transformToImageSpace(const real_T S_P[16], real_T
  Cam_E_S[16], const real_T K[16], real_T p[8], real_T P[16], real_T M[16]);
static void mirageTracking_getEquations(const real_T Cam_PDes[16], const real_T
  S_PDes[16], const real_T Err[8], const real_T M[16], real_T equations[96],
  real_T constants[8]);
static void mirageTracking_eml_xgetrf(real_T A[144], int32_T ipiv[12]);
static void mirageTracking_solveEquations(const real_T A[192], const real_T B[16],
  real_T params[12]);
static void mirageTracking_visioMotor(real_T L_p[8], real_T R_p[8], const real_T
  S_PDes[16], real_T errParams[12]);
static void mirageTracking_SystemCore_step(visioncodegen_Autothresholder_T *obj,
  const real32_T varargin_1[6561], boolean_T varargout_1[6561]);
static void mirageTrackin_SystemCore_step_l(visioncodegen_Autothresholder_T *obj,
  const real32_T varargin_1[6561], boolean_T varargout_1[6561]);
static void mirageTracki_SystemCore_step_li(visioncodegen_BlobAnalysis_mi_T *obj,
  const boolean_T varargin_1[6561], int32_T varargout_1_data[], int32_T
  varargout_1_sizes[2], real_T varargout_2_data[], int32_T varargout_2_sizes[2],
  int32_T varargout_3_data[], int32_T varargout_3_sizes[2]);
static void mirageTrack_SystemCore_step_liy(visioncodegen_BlobAnalysis_mi_T *obj,
  const boolean_T varargin_1[6561], int32_T varargout_1_data[], int32_T
  varargout_1_sizes[2], real_T varargout_2_data[], int32_T varargout_2_sizes[2],
  int32_T varargout_3_data[], int32_T varargout_3_sizes[2]);
static void mirageTracking_merge(int32_T idx_data[], int32_T x_data[], int32_T
  offset, int32_T np, int32_T nq);
static void mirageTracking_eml_sort_idx(int32_T x_data[], int32_T *x_sizes,
  int32_T idx_data[], int32_T *idx_sizes);
static void mirageTracking_eml_sort_f(int32_T x_data[], int32_T x_sizes[2],
  int32_T dim, int32_T idx_data[], int32_T idx_sizes[2]);
static void mirageTracking_eml_sort(int32_T x_data[], int32_T x_sizes[2],
  int32_T idx_data[], int32_T idx_sizes[2]);
int32_T div_s32(int32_T numerator, int32_T denominator)
{
  int32_T quotient;
  uint32_T tempAbsQuotient;
  if (denominator == 0) {
    quotient = numerator >= 0 ? MAX_int32_T : MIN_int32_T;

    /* Divide by zero handler */
  } else {
    tempAbsQuotient = (uint32_T)(numerator >= 0 ? numerator : -numerator) /
      (denominator >= 0 ? denominator : -denominator);
    quotient = (numerator < 0) != (denominator < 0) ? -(int32_T)tempAbsQuotient :
      (int32_T)tempAbsQuotient;
  }

  return quotient;
}

/* Function for MATLAB Function: '<Root>/Calculate Target Points Version 3' */
static void mirageTracking_padarray(const real32_T varargin_1[3686400], real32_T
  b[4243200])
{
  int32_T k;
  int32_T b_k;
  for (k = 0; k < 3; k++) {
    for (b_k = 0; b_k < 40; b_k++) {
      memset(&b[1040 * b_k + 1414400 * k], 0, 1040U * sizeof(real32_T));
    }

    for (b_k = 0; b_k < 40; b_k++) {
      memset(&b[1040 * (b_k + 1320) + 1414400 * k], 0, 1040U * sizeof(real32_T));
    }

    for (b_k = 0; b_k < 1280; b_k++) {
      memset(&b[1040 * (b_k + 40) + 1414400 * k], 0, 40U * sizeof(real32_T));
    }

    for (b_k = 0; b_k < 1280; b_k++) {
      memset(&b[(1040 * (b_k + 40) + 1414400 * k) + 1000], 0, 40U * sizeof
             (real32_T));
    }
  }

  for (b_k = 0; b_k < 3; b_k++) {
    for (k = 0; k < 1280; k++) {
      memcpy(&b[(1040 * (k + 40) + 1414400 * b_k) + 40], &varargin_1[960 * k +
             1228800 * b_k], 960U * sizeof(real32_T));
    }
  }
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

/* Function for MATLAB Function: '<Root>/Calculate Target Points Version 3' */
static void mirageTracking_round(real_T x[8])
{
  int32_T k;
  for (k = 0; k < 8; k++) {
    x[k] = rt_roundd_snf(x[k]);
  }
}

/* Function for MATLAB Function: '<Root>/Estimate Pose Error' */
static void mirageTracking_invNxN(const real_T x[16], real_T y[16])
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
        jBcol = b_j + jy;
        smax = A[b_j];
        A[b_j] = A[jBcol];
        A[jBcol] = smax;
        kAcol = b_j + 4;
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
static void mirageTracking_getEquations(const real_T Cam_PDes[16], const real_T
  S_PDes[16], const real_T Err[8], const real_T M[16], real_T equations[96],
  real_T constants[8])
{
  real_T n[12];
  real_T j;
  real_T O[12];
  int32_T i;
  int32_T j_0;
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
    n[0] = Cam_PDes[(i << 2) + 2] * M[0] - Cam_PDes[i << 2] * M[2];
    n[3] = Cam_PDes[(i << 2) + 2] * M[4] - Cam_PDes[i << 2] * M[6];
    n[6] = Cam_PDes[(i << 2) + 2] * M[8] - Cam_PDes[i << 2] * M[10];
    n[9] = Cam_PDes[(i << 2) + 2] * M[12] - Cam_PDes[i << 2] * M[14];
    n[1] = Cam_PDes[(i << 2) + 2] * M[1] - Cam_PDes[(i << 2) + 1] * M[2];
    n[4] = Cam_PDes[(i << 2) + 2] * M[5] - Cam_PDes[(i << 2) + 1] * M[6];
    n[7] = Cam_PDes[(i << 2) + 2] * M[9] - Cam_PDes[(i << 2) + 1] * M[10];
    n[10] = Cam_PDes[(i << 2) + 2] * M[13] - Cam_PDes[(i << 2) + 1] * M[14];
    n[2] = Cam_PDes[(i << 2) + 2] * M[2];
    n[5] = Cam_PDes[(i << 2) + 2] * M[6];
    n[8] = Cam_PDes[(i << 2) + 2] * M[10];
    n[11] = Cam_PDes[(i << 2) + 2] * M[14];
    for (j_0 = 0; j_0 < 12; j_0++) {
      O[j_0] = n[j_0] / n[11];
    }

    Vx_idx_0 = O[0] - Err[i << 1] * O[2];
    Vx_idx_1 = O[3] - Err[i << 1] * O[5];
    Vx_idx_2 = O[6] - Err[i << 1] * O[8];
    Vy_idx_0 = O[1] - Err[(i << 1) + 1] * O[2];
    Vy_idx_1 = O[4] - Err[(i << 1) + 1] * O[5];
    Vy_idx_2 = O[7] - Err[(i << 1) + 1] * O[8];
    j_0 = (int32_T)j - 1;
    equations[j_0] = S_PDes[i << 2] * Vx_idx_0;
    equations[j_0 + 8] = S_PDes[i << 2] * Vx_idx_1;
    equations[j_0 + 16] = S_PDes[i << 2] * Vx_idx_2;
    equations[j_0 + 24] = S_PDes[(i << 2) + 1] * Vx_idx_0;
    equations[j_0 + 32] = S_PDes[(i << 2) + 1] * Vx_idx_1;
    equations[j_0 + 40] = S_PDes[(i << 2) + 1] * Vx_idx_2;
    equations[j_0 + 48] = S_PDes[(i << 2) + 2] * Vx_idx_0;
    equations[j_0 + 56] = S_PDes[(i << 2) + 2] * Vx_idx_1;
    equations[j_0 + 64] = S_PDes[(i << 2) + 2] * Vx_idx_2;
    equations[j_0 + 72] = Vx_idx_0;
    equations[j_0 + 80] = Vx_idx_1;
    equations[j_0 + 88] = Vx_idx_2;
    j_0 = (int32_T)(j + 1.0) - 1;
    equations[j_0] = S_PDes[i << 2] * Vy_idx_0;
    equations[j_0 + 8] = S_PDes[i << 2] * Vy_idx_1;
    equations[j_0 + 16] = S_PDes[i << 2] * Vy_idx_2;
    equations[j_0 + 24] = S_PDes[(i << 2) + 1] * Vy_idx_0;
    equations[j_0 + 32] = S_PDes[(i << 2) + 1] * Vy_idx_1;
    equations[j_0 + 40] = S_PDes[(i << 2) + 1] * Vy_idx_2;
    equations[j_0 + 48] = S_PDes[(i << 2) + 2] * Vy_idx_0;
    equations[j_0 + 56] = S_PDes[(i << 2) + 2] * Vy_idx_1;
    equations[j_0 + 64] = S_PDes[(i << 2) + 2] * Vy_idx_2;
    equations[j_0 + 72] = Vy_idx_0;
    equations[j_0 + 80] = Vy_idx_1;
    equations[j_0 + 88] = Vy_idx_2;
    constants[(int32_T)j - 1] = Err[i << 1] - O[9];
    constants[(int32_T)(j + 1.0) - 1] = Err[(i << 1) + 1] - O[10];
    j += 2.0;
  }
}

/* Function for MATLAB Function: '<Root>/Estimate Pose Error' */
static void mirageTracking_eml_xgetrf(real_T A[144], int32_T ipiv[12])
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
static void mirageTracking_solveEquations(const real_T A[192], const real_T B[16],
  real_T params[12])
{
  real_T b_y[144];
  int8_T p[12];
  real_T x[144];
  int32_T ipiv[12];
  int32_T b_k;
  int32_T jBcol;
  int32_T kAcol;
  int32_T c_k;
  int32_T b_i;
  real_T A_0[12];

  /*  This function solves the equations as Ax-B=0. Gets A and B, computes x */
  /*  parameters */
  /*  A :(mxn) Equation Matrix  */
  /*  B :(mx1) Constants Vector  */
  /*  */
  /*  params :(nx1) Parameters Vector */
  memset(&b_y[0], 0, 144U * sizeof(real_T));
  for (b_k = 0; b_k < 12; b_k++) {
    for (jBcol = 0; jBcol < 12; jBcol++) {
      x[b_k + 12 * jBcol] = 0.0;
      for (c_k = 0; c_k < 16; c_k++) {
        x[b_k + 12 * jBcol] += A[(b_k << 4) + c_k] * A[(jBcol << 4) + c_k];
      }
    }
  }

  mirageTracking_eml_xgetrf(x, ipiv);
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
          b_y[kAcol + 12 * jBcol] -= b_y[12 * jBcol + c_k] * x[12 * c_k + kAcol];
        }
      }
    }
  }

  for (b_k = 0; b_k < 12; b_k++) {
    jBcol = 12 * b_k;
    for (c_k = 11; c_k >= 0; c_k += -1) {
      kAcol = 12 * c_k;
      if (b_y[c_k + jBcol] != 0.0) {
        b_y[c_k + jBcol] /= x[c_k + kAcol];
        for (b_i = 0; b_i + 1 <= c_k; b_i++) {
          b_y[b_i + jBcol] -= b_y[c_k + jBcol] * x[b_i + kAcol];
        }
      }
    }
  }

  for (b_k = 0; b_k < 12; b_k++) {
    A_0[b_k] = 0.0;
    for (jBcol = 0; jBcol < 16; jBcol++) {
      A_0[b_k] += A[(b_k << 4) + jBcol] * B[jBcol];
    }
  }

  for (b_k = 0; b_k < 12; b_k++) {
    params[b_k] = 0.0;
    for (jBcol = 0; jBcol < 12; jBcol++) {
      params[b_k] += b_y[12 * jBcol + b_k] * A_0[jBcol];
    }
  }
}

/* Function for MATLAB Function: '<Root>/Estimate Pose Error' */
static void mirageTracking_visioMotor(real_T L_p[8], real_T R_p[8], const real_T
  S_PDes[16], real_T errParams[12])
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
  L_p[1] = 960.0 - L_p[1];
  L_p[3] = 960.0 - L_p[3];
  L_p[5] = 960.0 - L_p[5];
  L_p[7] = 960.0 - L_p[7];
  R_p[1] = 960.0 - R_p[1];
  R_p[3] = 960.0 - R_p[3];
  R_p[5] = 960.0 - R_p[5];
  R_p[7] = 960.0 - R_p[7];

  /*  Pixel Error */
  /*  Left and Right Camera Equations (2 equations for 1 point) */
  for (i = 0; i < 8; i++) {
    L_p_0[i] = L_p[i] - L_pDes1[i];
    R_p_0[i] = R_p[i] - R_pDes1[i];
  }

  mirageTracking_getEquations(L_PDes, S_PDes, L_p_0, L_M, Left_Equations,
    R_pDes1);
  mirageTracking_getEquations(b_0, S_PDes, R_p_0, R_M, Right_Equations, L_pDes1);
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
  mirageTracking_solveEquations(Left_Equations_0, b_0, errParams);
}

real_T rt_atan2d_snf(real_T u0, real_T u1)
{
  real_T y;
  int32_T u0_0;
  int32_T u1_0;
  if (rtIsNaN(u0) || rtIsNaN(u1)) {
    y = (rtNaN);
  } else if (rtIsInf(u0) && rtIsInf(u1)) {
    if (u0 > 0.0) {
      u0_0 = 1;
    } else {
      u0_0 = -1;
    }

    if (u1 > 0.0) {
      u1_0 = 1;
    } else {
      u1_0 = -1;
    }

    y = atan2(u0_0, u1_0);
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

/* Function for MATLAB Function: '<Root>/Calculate Target Points Version 3' */
static void mirageTracking_SystemCore_step(visioncodegen_Autothresholder_T *obj,
  const real32_T varargin_1[6561], boolean_T varargout_1[6561])
{
  vision_Autothresholder_1_mira_T *d_obj;
  int32_T i;
  real32_T scale;
  real32_T cnt;
  int32_T idxMaxVal;
  if (obj->isInitialized != 1) {
    obj->isInitialized = 1;
    obj->NoTuningBeforeLockingCodeGenError = true;
  }

  d_obj = &obj->cSFunObject;

  /* System object Outputs function: vision.Autothresholder */
  for (i = 0; i < 256; i++) {
    d_obj->W0_P_DW[i] = 0.0F;
  }

  for (i = 0; i < 256; i++) {
    d_obj->W1_MU_DW[i] = 0.0F;
  }

  scale = (real32_T)div_s32(255, (int32_T)(obj->cSFunObject.P1_UMAX_RTP -
    obj->cSFunObject.P0_UMIN_RTP));
  for (i = 0; i < 6561; i++) {
    if (varargin_1[i] < d_obj->P0_UMIN_RTP) {
      d_obj->W0_P_DW[0]++;
    } else if (varargin_1[i] > d_obj->P1_UMAX_RTP) {
      d_obj->W0_P_DW[255]++;
    } else {
      d_obj->W0_P_DW[(uint8_T)((varargin_1[i] - d_obj->P0_UMIN_RTP) * scale +
        0.5F)]++;
    }
  }

  for (i = 0; i < 256; i++) {
    d_obj->W0_P_DW[i] /= 6561.0F;
  }

  obj->cSFunObject.W1_MU_DW[0] = obj->cSFunObject.W0_P_DW[0];
  cnt = 2.0F;
  for (i = 0; i < 255; i++) {
    d_obj->W1_MU_DW[i + 1] = d_obj->W0_P_DW[i + 1] * cnt + d_obj->W1_MU_DW[i];
    cnt++;
  }

  cnt = obj->cSFunObject.W1_MU_DW[255];
  for (i = 0; i < 255; i++) {
    d_obj->W0_P_DW[i + 1] += d_obj->W0_P_DW[i];
  }

  for (i = 0; i < 256; i++) {
    d_obj->W1_MU_DW[i] = d_obj->W0_P_DW[i] * cnt - d_obj->W1_MU_DW[i];
    d_obj->W1_MU_DW[i] *= d_obj->W1_MU_DW[i];
    d_obj->W0_P_DW[i] *= 1.0F - d_obj->W0_P_DW[i];
    d_obj->W0_P_DW[i] = d_obj->W1_MU_DW[i] / d_obj->W0_P_DW[i];
  }

  idxMaxVal = 0;
  cnt = 0.0F;
  for (i = 0; i < 256; i++) {
    if (d_obj->W0_P_DW[i] > cnt) {
      cnt = d_obj->W0_P_DW[i];
      idxMaxVal = i;
    }
  }

  cnt = (real32_T)idxMaxVal / scale + obj->cSFunObject.P0_UMIN_RTP;
  for (i = 0; i < 6561; i++) {
    varargout_1[i] = (varargin_1[i] > cnt);
  }
}

/* Function for MATLAB Function: '<Root>/Calculate Target Points Version 3' */
static void mirageTrackin_SystemCore_step_l(visioncodegen_Autothresholder_T *obj,
  const real32_T varargin_1[6561], boolean_T varargout_1[6561])
{
  vision_Autothresholder_1_mira_T *d_obj;
  int32_T i;
  real32_T scale;
  real32_T cnt;
  int32_T idxMaxVal;
  if (obj->isInitialized != 1) {
    obj->isInitialized = 1;
    obj->NoTuningBeforeLockingCodeGenError = true;
  }

  d_obj = &obj->cSFunObject;

  /* System object Outputs function: vision.Autothresholder */
  for (i = 0; i < 256; i++) {
    d_obj->W0_P_DW[i] = 0.0F;
  }

  for (i = 0; i < 256; i++) {
    d_obj->W1_MU_DW[i] = 0.0F;
  }

  scale = (real32_T)div_s32(255, (int32_T)(obj->cSFunObject.P1_UMAX_RTP -
    obj->cSFunObject.P0_UMIN_RTP));
  for (i = 0; i < 6561; i++) {
    if (varargin_1[i] < d_obj->P0_UMIN_RTP) {
      d_obj->W0_P_DW[0]++;
    } else if (varargin_1[i] > d_obj->P1_UMAX_RTP) {
      d_obj->W0_P_DW[255]++;
    } else {
      d_obj->W0_P_DW[(uint8_T)((varargin_1[i] - d_obj->P0_UMIN_RTP) * scale +
        0.5F)]++;
    }
  }

  for (i = 0; i < 256; i++) {
    d_obj->W0_P_DW[i] /= 6561.0F;
  }

  obj->cSFunObject.W1_MU_DW[0] = obj->cSFunObject.W0_P_DW[0];
  cnt = 2.0F;
  for (i = 0; i < 255; i++) {
    d_obj->W1_MU_DW[i + 1] = d_obj->W0_P_DW[i + 1] * cnt + d_obj->W1_MU_DW[i];
    cnt++;
  }

  cnt = obj->cSFunObject.W1_MU_DW[255];
  for (i = 0; i < 255; i++) {
    d_obj->W0_P_DW[i + 1] += d_obj->W0_P_DW[i];
  }

  for (i = 0; i < 256; i++) {
    d_obj->W1_MU_DW[i] = d_obj->W0_P_DW[i] * cnt - d_obj->W1_MU_DW[i];
    d_obj->W1_MU_DW[i] *= d_obj->W1_MU_DW[i];
    d_obj->W0_P_DW[i] *= 1.0F - d_obj->W0_P_DW[i];
    d_obj->W0_P_DW[i] = d_obj->W1_MU_DW[i] / d_obj->W0_P_DW[i];
  }

  idxMaxVal = 0;
  cnt = 0.0F;
  for (i = 0; i < 256; i++) {
    if (d_obj->W0_P_DW[i] > cnt) {
      cnt = d_obj->W0_P_DW[i];
      idxMaxVal = i;
    }
  }

  cnt = (real32_T)idxMaxVal / scale + obj->cSFunObject.P0_UMIN_RTP;
  for (i = 0; i < 6561; i++) {
    varargout_1[i] = (varargin_1[i] > cnt);
  }
}

/* Function for MATLAB Function: '<Root>/Calculate Target Points Version 3' */
static void mirageTracki_SystemCore_step_li(visioncodegen_BlobAnalysis_mi_T *obj,
  const boolean_T varargin_1[6561], int32_T varargout_1_data[], int32_T
  varargout_1_sizes[2], real_T varargout_2_data[], int32_T varargout_2_sizes[2],
  int32_T varargout_3_data[], int32_T varargout_3_sizes[2])
{
  vision_BlobAnalysis_3_mirageT_T *d_obj;
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
    obj->isInitialized = 1;
    obj->NoTuningBeforeLockingCodeGenError = true;
  }

  d_obj = &obj->cSFunObject;

  /* System object Outputs function: vision.BlobAnalysis */
  maxNumBlobsReached = false;
  for (loop = 0; loop < 84; loop++) {
    d_obj->W3_PAD_DW[loop] = 0U;
  }

  currentLabel = 1U;
  loop = 0;
  idx = 84;
  for (n = 0; n < 81; n++) {
    for (maxc = 0; maxc < 81; maxc++) {
      d_obj->W3_PAD_DW[idx] = (uint8_T)(varargin_1[loop] ? 255 : 0);
      loop++;
      idx++;
    }

    d_obj->W3_PAD_DW[idx] = 0U;
    d_obj->W3_PAD_DW[idx + 1] = 0U;
    idx += 2;
  }

  for (loop = 0; loop < 82; loop++) {
    d_obj->W3_PAD_DW[loop + idx] = 0U;
  }

  loop = 1;
  pixIdx = 0U;
  n = 0;
  while (n < 81) {
    maxc = 1;
    idx = loop * 83;
    maxr = 0;
    while (maxr < 81) {
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
                83U) - 1);
              d_obj->W1_M_PIXLIST_DW[pixIdx] = (int16_T)(walkerIdx % 83U - 1U);
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
          n = 81;
          maxr = 81;
        }

        if (maxr < 81) {
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
    n = 81;
    idx = 81;
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

/* Function for MATLAB Function: '<Root>/Calculate Target Points Version 3' */
static void mirageTrack_SystemCore_step_liy(visioncodegen_BlobAnalysis_mi_T *obj,
  const boolean_T varargin_1[6561], int32_T varargout_1_data[], int32_T
  varargout_1_sizes[2], real_T varargout_2_data[], int32_T varargout_2_sizes[2],
  int32_T varargout_3_data[], int32_T varargout_3_sizes[2])
{
  vision_BlobAnalysis_3_mirageT_T *d_obj;
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
    obj->isInitialized = 1;
    obj->NoTuningBeforeLockingCodeGenError = true;
  }

  d_obj = &obj->cSFunObject;

  /* System object Outputs function: vision.BlobAnalysis */
  maxNumBlobsReached = false;
  for (loop = 0; loop < 84; loop++) {
    d_obj->W3_PAD_DW[loop] = 0U;
  }

  currentLabel = 1U;
  loop = 0;
  idx = 84;
  for (n = 0; n < 81; n++) {
    for (maxc = 0; maxc < 81; maxc++) {
      d_obj->W3_PAD_DW[idx] = (uint8_T)(varargin_1[loop] ? 255 : 0);
      loop++;
      idx++;
    }

    d_obj->W3_PAD_DW[idx] = 0U;
    d_obj->W3_PAD_DW[idx + 1] = 0U;
    idx += 2;
  }

  for (loop = 0; loop < 82; loop++) {
    d_obj->W3_PAD_DW[loop + idx] = 0U;
  }

  loop = 1;
  pixIdx = 0U;
  n = 0;
  while (n < 81) {
    maxc = 1;
    idx = loop * 83;
    maxr = 0;
    while (maxr < 81) {
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
                83U) - 1);
              d_obj->W1_M_PIXLIST_DW[pixIdx] = (int16_T)(walkerIdx % 83U - 1U);
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
          n = 81;
          maxr = 81;
        }

        if (maxr < 81) {
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
    n = 81;
    idx = 81;
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

/* Function for MATLAB Function: '<Root>/Calculate Target Points Version 3' */
static void mirageTracking_merge(int32_T idx_data[], int32_T x_data[], int32_T
  offset, int32_T np, int32_T nq)
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

/* Function for MATLAB Function: '<Root>/Calculate Target Points Version 3' */
static void mirageTracking_eml_sort_idx(int32_T x_data[], int32_T *x_sizes,
  int32_T idx_data[], int32_T *idx_sizes)
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
          mirageTracking_merge(idx_data, b_x_data, tailOffset, bLen, nQuartets -
                               bLen);
        }
      }

      nQuartets = bLen << 1;
      nPairs >>= 1;
      for (nLeft = 1; nLeft <= nPairs; nLeft++) {
        mirageTracking_merge(idx_data, b_x_data, (nLeft - 1) * nQuartets, bLen,
                             bLen);
      }

      bLen = nQuartets;
    }

    if (*x_sizes > bLen) {
      mirageTracking_merge(idx_data, b_x_data, 0, bLen, *x_sizes - bLen);
    }
  }

  *x_sizes = b_x_sizes;
  for (nLeft = 0; nLeft < b_x_sizes; nLeft++) {
    x_data[nLeft] = b_x_data[nLeft];
  }
}

/* Function for MATLAB Function: '<Root>/Calculate Target Points Version 3' */
static void mirageTracking_eml_sort_f(int32_T x_data[], int32_T x_sizes[2],
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
  b = x_sizes[dim - 1];
  vwork_sizes = (int8_T)x_sizes[dim - 1];
  idx_sizes[0] = (int8_T)x_sizes[0];
  idx_sizes[1] = (int8_T)x_sizes[1];
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

      mirageTracking_eml_sort_idx(vwork_data, &vwork_sizes, iidx_data, &c_k);
      for (c_k = 0; c_k + 1 <= b; c_k++) {
        x_data[j + c_k * vstride] = vwork_data[c_k];
        idx_data[j + c_k * vstride] = iidx_data[c_k];
      }
    }

    c_k = 2;
  }
}

/* Function for MATLAB Function: '<Root>/Calculate Target Points Version 3' */
static void mirageTracking_eml_sort(int32_T x_data[], int32_T x_sizes[2],
  int32_T idx_data[], int32_T idx_sizes[2])
{
  int32_T dim;
  dim = 2;
  if (x_sizes[0] != 1) {
    dim = 1;
  }

  mirageTracking_eml_sort_f(x_data, x_sizes, dim, idx_data, idx_sizes);
}

/* Model step function */
void mirageTracking_step(void)
{
  /* local block i/o variables */
  real_T rtb_Clock;

  {
    char_T *sErr;
    void *source_R;
    visioncodegen_Autothresholder_T hautoth;
    real_T lwc[8];
    real_T rwc[8];
    real_T lwx[81];
    real_T lwy[81];
    real_T rwx[81];
    real_T rwy[81];
    visioncodegen_Autothresholder_T *obj;
    visioncodegen_BlobAnalysis_mi_T *b_obj;
    static const int8_T b[8] = { -1, 82, 83, 84, 1, -82, -83, -84 };

    real_T world2robotDes[16];
    real_T errParams[12];
    real_T poseErrorMat[16];
    real_T world2robot[16];
    real_T y[12];
    real_T theta;
    static const int8_T b_0[16] = { 1, 0, 0, 0, 0, 1, 0, 0, 0, 0, 1, 0, 0, 0, 0,
      1 };

    int16_T rtb_qr[6];
    int32_T i;
    int32_T i_0;
    static const int16_T tmp[6] = { -395, 2, 45, 0, 0, 0 };

    real_T tmp_0[16];
    int32_T i_1;
    int16_T tmp_1[16];
    real_T tmp_2[16];
    real_T tmp_3[16];
    int32_T i_2;
    int32_T aL_data[50];
    int32_T aL_sizes[2];
    real_T cL_data[100];
    int32_T cL_sizes[2];
    int32_T unusedU0_data[200];
    int32_T unusedU0_sizes[2];
    int32_T aR_data[50];
    int32_T aR_sizes[2];
    real_T cR_data[100];
    int32_T cR_sizes[2];
    int32_T indL_data[50];
    int32_T indR_data[50];
    int32_T iidx_data[50];
    real_T cL_data_0[2];

    /* Clock: '<Root>/Clock' */
    rtb_Clock = mirageTracking_M->Timing.t[0];

    /* MATLAB Function: '<Root>/MATLAB Function' */
    for (i = 0; i < 6; i++) {
      rtb_qr[i] = tmp[i];
    }

    /* End of MATLAB Function: '<Root>/MATLAB Function' */

    /* S-Function (simaqfvd): '<Root>/Left Camera' */
    /* MATLAB Function 'MATLAB Function': '<S5>:1' */
    /*  qr = [xr yr thetar]^T (reference trajectory) */
    /*  ur = [vr omegar]^T (reference control input that generates the reference trajetory in ideal conditions) */
    /*  A = 1; */
    /*  xL = 10; */
    /*  vx = 1; */
    /*  %  */
    /*  % desired x and y  */
    /*  xr = vx*t; */
    /*  yr = A*sin(2*pi*xr/xL); */
    /*   */
    /*  % desired x and y derivatives */
    /*  xrdot = vx; */
    /*  yrdot = A*(2*pi/xL)*cos(2*pi*xr/xL); */
    /*   */
    /*  % desired theta */
    /*  thetar = atan2(yrdot,xrdot); */
    /*   */
    /*  % desired second derivatives */
    /*  xrddot = 0; */
    /*  yrddot = -A*(2*pi/xL)^2*sin(2*pi*xr/xL); */
    /*   */
    /*  % desired theta derivative */
    /*  thetardot = (yrddot - xrddot*tan(thetar))/(xrdot*(1+tan(thetar)^2)); */
    /*   */
    /*  vr     = sqrt(xrdot^2+yrdot^2); */
    /*  omegar = thetardot; */
    /* -------------------------------------------- */
    /*   */
    /*  xr = vx*t - 30; */
    /*  yr = A*sin(2*pi*xr/xL) + 2; */
    /*   */
    /*  % desired x and y derivatives */
    /*  xrdot = vx; */
    /*  yrdot = A*(2*pi/xL)*cos(2*pi*xr/xL); */
    /*   */
    /*  % desired theta */
    /*  thetar = atan2(yrdot,xrdot); */
    /*   */
    /*  % desired second derivatives */
    /*  xrddot = 0; */
    /*  yrddot = -A*(2*pi/xL)^2*sin(2*pi*xr/xL); */
    /*   */
    /*  % desired theta derivative */
    /*  thetardot = (yrddot - xrddot*tan(thetar))/(xrdot*(1+tan(thetar)^2)); */
    /*   */
    /*  vr     = sqrt(xrdot^2+yrdot^2); */
    /*  omegar = thetardot; */
    /* -------------------------------------------- */
    /*  yr = vx*t - 30; */
    /*  xr = A*sin(2*pi*yr/xL) + 2; */
    /*   */
    /*  yrdot = vx; */
    /*  xrdot = A*(2*pi/xL)*cos(2*pi*xr/xL); */
    /*   */
    /*  desired theta */
    /*  thetar = atan2(yrdot,xrdot); */
    /*  thetardot = 0; */
    /*   */
    /*  vr     = sqrt(xrdot^2+yrdot^2); */
    /*  omegar = thetardot; */
    /* -------------------------------------------- */
    /*  xr = vx*t - 30; */
    /*  yr = 5; */
    /*   */
    /*  xrdot = vx; */
    /*  yrdot = 0; */
    /*   */
    /*  thetar = 0; */
    /*  thetardot = 0; */
    /*   */
    /*  vr     = sqrt(xrdot^2+yrdot^2); */
    /*  omegar = thetardot; */
    /*  qr = [xr;yr;thetar]; */
    /*  ur = [vr;omegar]; */
    /* '<S5>:1:86' */
    sErr = GetErrorBuffer(&mirageTracking_DW.LeftCamera_FromVideoDevice[0U]);
    source_R = (void *)&mirageTracking_B.LeftCamera[0U];
    LibOutputs_FVD(&mirageTracking_DW.LeftCamera_FromVideoDevice[0U], source_R,
                   GetNullPointer(), GetNullPointer(), "single", 3686400, 1, 0,
                   GetNullPointer());
    if (*sErr != 0) {
      rtmSetErrorStatus(mirageTracking_M, sErr);
      rtmSetStopRequested(mirageTracking_M, 1);
    }

    /* End of S-Function (simaqfvd): '<Root>/Left Camera' */

    /* S-Function (simaqfvd): '<Root>/Right Camera' */
    sErr = GetErrorBuffer(&mirageTracking_DW.RightCamera_FromVideoDevice[0U]);
    source_R = (void *)&mirageTracking_B.RightCamera[0U];
    LibOutputs_FVD(&mirageTracking_DW.RightCamera_FromVideoDevice[0U], source_R,
                   GetNullPointer(), GetNullPointer(), "single", 3686400, 1, 0,
                   GetNullPointer());
    if (*sErr != 0) {
      rtmSetErrorStatus(mirageTracking_M, sErr);
      rtmSetStopRequested(mirageTracking_M, 1);
    }

    /* End of S-Function (simaqfvd): '<Root>/Right Camera' */

    /* MATLAB Function: '<Root>/Calculate Target Points Version 3' incorporates:
     *  Memory: '<Root>/Left Cam Centers'
     *  Memory: '<Root>/Right Cam Centers'
     */
    /* MATLAB Function 'Calculate Target Points Version 3': '<S3>:1' */
    /* Pad Size */
    /* '<S3>:1:4' */
    memcpy(&mirageTracking_B.fv0[0], &mirageTracking_B.LeftCamera[0], 3686400U *
           sizeof(real32_T));
    mirageTracking_padarray(mirageTracking_B.fv0, mirageTracking_B.leftImage);

    /* '<S3>:1:5' */
    memcpy(&mirageTracking_B.fv0[0], &mirageTracking_B.RightCamera[0], 3686400U *
           sizeof(real32_T));
    mirageTracking_padarray(mirageTracking_B.fv0, mirageTracking_B.rightImage);

    /* '<S3>:1:7' */
    memcpy(&lwc[0], &mirageTracking_DW.LeftCamCenters_PreviousInput[0], sizeof
           (real_T) << 3U);
    mirageTracking_round(lwc);

    /* Center of the left windows */
    /* '<S3>:1:8' */
    for (i_1 = 0; i_1 < 8; i_1++) {
      rwc[i_1] = mirageTracking_DW.RightCamCenters_PreviousInput[i_1];
      lwc[i_1] += 40.0;
    }

    mirageTracking_round(rwc);

    /* Center of the right windows */
    /* '<S3>:1:10' */
    /* '<S3>:1:11' */
    for (i_1 = 0; i_1 < 8; i_1++) {
      mirageTracking_B.L_p[i_1] = 0.0;
      mirageTracking_B.R_p[i_1] = 0.0;
      rwc[i_1] += 40.0;
    }

    /* '<S3>:1:13' */
    /* '<S3>:1:14' */
    for (i = 0; i < 4; i++) {
      /* '<S3>:1:14' */
      /* '<S3>:1:15' */
      /* '<S3>:1:16' */
      /* '<S3>:1:18' */
      /* '<S3>:1:19' */
      for (i_1 = 0; i_1 < 81; i_1++) {
        lwx[i_1] = (-40.0 + (real_T)i_1) + lwc[i << 1];
        lwy[i_1] = lwc[(i << 1) + 1] + (-40.0 + (real_T)i_1);
        rwx[i_1] = (-40.0 + (real_T)i_1) + rwc[i << 1];
        rwy[i_1] = rwc[(i << 1) + 1] + (-40.0 + (real_T)i_1);
      }

      /* '<S3>:1:21' */
      /* '<S3>:1:22' */
      /* '<S3>:1:24' */
      for (i_1 = 0; i_1 < 3; i_1++) {
        for (i_0 = 0; i_0 < 81; i_0++) {
          for (i_2 = 0; i_2 < 81; i_2++) {
            mirageTracking_B.pWinL_m[(i_2 + 81 * i_0) + 6561 * i_1] = 1.0F -
              mirageTracking_B.leftImage[((((int32_T)lwx[i_0] - 1) * 1040 +
              (int32_T)lwy[i_2]) + 1414400 * i_1) - 1];
          }
        }
      }

      /* '<S3>:1:25' */
      for (i_1 = 0; i_1 < 3; i_1++) {
        for (i_0 = 0; i_0 < 81; i_0++) {
          for (i_2 = 0; i_2 < 81; i_2++) {
            mirageTracking_B.pWinR[(i_2 + 81 * i_0) + 6561 * i_1] = 1.0F -
              mirageTracking_B.rightImage[((((int32_T)rwx[i_0] - 1) * 1040 +
              (int32_T)rwy[i_2]) + 1414400 * i_1) - 1];
          }
        }
      }

      /* '<S3>:1:27' */
      /* '<S3>:1:28' */
      /* '<S3>:1:30' */
      /*  imsubtract(rgb1(:,:,3),gray) */
      /* '<S3>:1:31' */
      /*  imsubtract(rgb1(:,:,3),gray)  */
      obj = &hautoth;
      hautoth.isInitialized = 0;
      hautoth.NoTuningBeforeLockingCodeGenError = true;

      /* System object Constructor function: vision.Autothresholder */
      obj->cSFunObject.P0_UMIN_RTP = 0.0F;
      obj->cSFunObject.P1_UMAX_RTP = 1.0F;
      hautoth.NoTuningBeforeLockingCodeGenError = false;

      /* '<S3>:1:34' */
      for (i_1 = 0; i_1 < 81; i_1++) {
        for (i_0 = 0; i_0 < 81; i_0++) {
          mirageTracking_B.pWinL[i_0 + 81 * i_1] = mirageTracking_B.pWinL_m[(81 *
            i_1 + i_0) + 13122] - ((mirageTracking_B.pWinL_m[(81 * i_1 + i_0) +
            6561] * 0.587F + mirageTracking_B.pWinL_m[81 * i_1 + i_0] * 0.2989F)
            + mirageTracking_B.pWinL_m[(81 * i_1 + i_0) + 13122] * 0.114F);
        }
      }

      mirageTracking_SystemCore_step(&hautoth, mirageTracking_B.pWinL,
        mirageTracking_B.pBWL);

      /* '<S3>:1:35' */
      for (i_1 = 0; i_1 < 81; i_1++) {
        for (i_0 = 0; i_0 < 81; i_0++) {
          mirageTracking_B.pWinL[i_0 + 81 * i_1] = mirageTracking_B.pWinR[(81 *
            i_1 + i_0) + 13122] - ((mirageTracking_B.pWinR[(81 * i_1 + i_0) +
            6561] * 0.587F + mirageTracking_B.pWinR[81 * i_1 + i_0] * 0.2989F) +
            mirageTracking_B.pWinR[(81 * i_1 + i_0) + 13122] * 0.114F);
        }
      }

      mirageTrackin_SystemCore_step_l(&hautoth, mirageTracking_B.pWinL,
        mirageTracking_B.pBWR);

      /* finds yellow regions in image using BlobAnalysis. */
      b_obj = &mirageTracking_B.hblob;
      mirageTracking_B.hblob.isInitialized = 0;
      mirageTracking_B.hblob.NoTuningBeforeLockingCodeGenError = true;

      /* System object Constructor function: vision.BlobAnalysis */
      for (i_0 = 0; i_0 < 8; i_0++) {
        b_obj->cSFunObject.P0_WALKER_RTP[i_0] = b[i_0];
      }

      b_obj->cSFunObject.P1_MINAREA_RTP = 0U;
      b_obj->cSFunObject.P2_MAXAREA_RTP = MAX_uint32_T;
      b_obj->cSFunObject.P1_MINAREA_RTP = 0U;
      b_obj->cSFunObject.P2_MAXAREA_RTP = MAX_uint32_T;
      mirageTracking_B.hblob.NoTuningBeforeLockingCodeGenError = false;
      mirageTracki_SystemCore_step_li(&mirageTracking_B.hblob,
        mirageTracking_B.pBWL, aL_data, aL_sizes, cL_data, cL_sizes,
        unusedU0_data, unusedU0_sizes);

      /*  get [x y] is a centroid of region */
      mirageTrack_SystemCore_step_liy(&mirageTracking_B.hblob,
        mirageTracking_B.pBWR, aR_data, aR_sizes, cR_data, cR_sizes,
        unusedU0_data, unusedU0_sizes);

      /*  get [x y] is a centroid of region */
      mirageTracking_eml_sort(aL_data, aL_sizes, iidx_data, unusedU0_sizes);
      i_0 = unusedU0_sizes[0] * unusedU0_sizes[1];
      for (i_1 = 0; i_1 < i_0; i_1++) {
        indL_data[i_1] = iidx_data[i_1];
      }

      mirageTracking_eml_sort(aR_data, aR_sizes, iidx_data, unusedU0_sizes);
      i_0 = unusedU0_sizes[0] * unusedU0_sizes[1];
      for (i_1 = 0; i_1 < i_0; i_1++) {
        indR_data[i_1] = iidx_data[i_1];
      }

      /* '<S3>:1:46' */
      i_0 = cL_sizes[1];
      for (i_1 = 0; i_1 < i_0; i_1++) {
        cL_data_0[i_1] = cL_data[(cL_sizes[0] * i_1 + indL_data[0]) - 1];
      }

      mirageTracking_B.L_p[i << 1] = (cL_data_0[0] + lwx[0]) - 40.0;
      mirageTracking_B.L_p[1 + (i << 1)] = (cL_data_0[1] + lwy[0]) - 40.0;

      /* '<S3>:1:47' */
      i_0 = cR_sizes[1];
      for (i_1 = 0; i_1 < i_0; i_1++) {
        cL_data_0[i_1] = cR_data[(cR_sizes[0] * i_1 + indR_data[0]) - 1];
      }

      mirageTracking_B.R_p[i << 1] = (cL_data_0[0] + rwx[0]) - 40.0;
      mirageTracking_B.R_p[1 + (i << 1)] = (cL_data_0[1] + rwy[0]) - 40.0;

      /* '<S3>:1:14' */
    }

    /* End of MATLAB Function: '<Root>/Calculate Target Points Version 3' */

    /* MATLAB Function: '<Root>/Estimate Pose Error' incorporates:
     *  Constant: '<Root>/Object Model'
     */
    /* MATLAB Function 'Estimate Pose Error': '<S4>:1' */
    /* Desired Robot Pose for current step qr() */
    /* '<S4>:1:5' */
    /* '<S4>:1:6' */
    /* '<S4>:1:7' */
    /* '<S4>:1:8' */
    /* '<S4>:1:9' */
    /* '<S4>:1:10' */
    /* Transform target points from I space into Desired Robot Space */
    /* '<S4>:1:13' */
    tmp_1[0] = 1;
    tmp_1[4] = 0;
    tmp_1[8] = 0;
    tmp_1[12] = rtb_qr[0];
    tmp_1[1] = 0;
    tmp_1[5] = 1;
    tmp_1[9] = 0;
    tmp_1[13] = rtb_qr[1];
    tmp_1[2] = 0;
    tmp_1[6] = 0;
    tmp_1[10] = 1;
    tmp_1[14] = rtb_qr[2];
    tmp_1[3] = 0;
    tmp_1[7] = 0;
    tmp_1[11] = 0;
    tmp_1[15] = 1;
    poseErrorMat[0] = 1.0;
    poseErrorMat[4] = 0.0;
    poseErrorMat[8] = 0.0;
    poseErrorMat[12] = 0.0;
    poseErrorMat[1] = 0.0;
    poseErrorMat[5] = cos(rtb_qr[3]);
    poseErrorMat[9] = -sin(rtb_qr[3]);
    poseErrorMat[13] = 0.0;
    poseErrorMat[2] = 0.0;
    poseErrorMat[6] = sin(rtb_qr[3]);
    poseErrorMat[10] = cos(rtb_qr[3]);
    poseErrorMat[14] = 0.0;
    poseErrorMat[3] = 0.0;
    poseErrorMat[7] = 0.0;
    poseErrorMat[11] = 0.0;
    poseErrorMat[15] = 1.0;
    for (i_1 = 0; i_1 < 4; i_1++) {
      for (i_0 = 0; i_0 < 4; i_0++) {
        world2robotDes[i_1 + (i_0 << 2)] = 0.0;
        world2robotDes[i_1 + (i_0 << 2)] += poseErrorMat[i_0 << 2] * (real_T)
          tmp_1[i_1];
        world2robotDes[i_1 + (i_0 << 2)] += poseErrorMat[(i_0 << 2) + 1] *
          (real_T)tmp_1[i_1 + 4];
        world2robotDes[i_1 + (i_0 << 2)] += poseErrorMat[(i_0 << 2) + 2] *
          (real_T)tmp_1[i_1 + 8];
        world2robotDes[i_1 + (i_0 << 2)] += poseErrorMat[(i_0 << 2) + 3] *
          (real_T)tmp_1[i_1 + 12];
      }
    }

    tmp_2[0] = cos(rtb_qr[4]);
    tmp_2[4] = 0.0;
    tmp_2[8] = sin(rtb_qr[4]);
    tmp_2[12] = 0.0;
    tmp_2[1] = 0.0;
    tmp_2[5] = 1.0;
    tmp_2[9] = 0.0;
    tmp_2[13] = 0.0;
    tmp_2[2] = -sin(rtb_qr[4]);
    tmp_2[6] = 0.0;
    tmp_2[10] = cos(rtb_qr[4]);
    tmp_2[14] = 0.0;
    tmp_2[3] = 0.0;
    tmp_2[7] = 0.0;
    tmp_2[11] = 0.0;
    tmp_2[15] = 1.0;
    for (i_1 = 0; i_1 < 4; i_1++) {
      for (i_0 = 0; i_0 < 4; i_0++) {
        poseErrorMat[i_1 + (i_0 << 2)] = 0.0;
        poseErrorMat[i_1 + (i_0 << 2)] += tmp_2[i_0 << 2] * world2robotDes[i_1];
        poseErrorMat[i_1 + (i_0 << 2)] += tmp_2[(i_0 << 2) + 1] *
          world2robotDes[i_1 + 4];
        poseErrorMat[i_1 + (i_0 << 2)] += tmp_2[(i_0 << 2) + 2] *
          world2robotDes[i_1 + 8];
        poseErrorMat[i_1 + (i_0 << 2)] += tmp_2[(i_0 << 2) + 3] *
          world2robotDes[i_1 + 12];
      }
    }

    tmp_3[0] = cos(rtb_qr[5]);
    tmp_3[4] = -sin(rtb_qr[5]);
    tmp_3[8] = 0.0;
    tmp_3[12] = 0.0;
    tmp_3[1] = sin(rtb_qr[5]);
    tmp_3[5] = cos(rtb_qr[5]);
    tmp_3[9] = 0.0;
    tmp_3[13] = 0.0;
    tmp_3[2] = 0.0;
    tmp_3[6] = 0.0;
    tmp_3[10] = 1.0;
    tmp_3[14] = 0.0;
    tmp_3[3] = 0.0;
    tmp_3[7] = 0.0;
    tmp_3[11] = 0.0;
    tmp_3[15] = 1.0;
    for (i_1 = 0; i_1 < 4; i_1++) {
      for (i_0 = 0; i_0 < 4; i_0++) {
        tmp_0[i_1 + (i_0 << 2)] = 0.0;
        tmp_0[i_1 + (i_0 << 2)] += tmp_3[i_0 << 2] * poseErrorMat[i_1];
        tmp_0[i_1 + (i_0 << 2)] += tmp_3[(i_0 << 2) + 1] * poseErrorMat[i_1 + 4];
        tmp_0[i_1 + (i_0 << 2)] += tmp_3[(i_0 << 2) + 2] * poseErrorMat[i_1 + 8];
        tmp_0[i_1 + (i_0 << 2)] += tmp_3[(i_0 << 2) + 3] * poseErrorMat[i_1 + 12];
      }
    }

    mirageTracking_invNxN(tmp_0, world2robotDes);

    /* '<S4>:1:14' */
    /* Calculate the error parameters for current step */
    /* '<S4>:1:17' */
    for (i_1 = 0; i_1 < 8; i_1++) {
      lwc[i_1] = mirageTracking_B.L_p[i_1];
      rwc[i_1] = mirageTracking_B.R_p[i_1];
    }

    for (i_1 = 0; i_1 < 4; i_1++) {
      for (i_0 = 0; i_0 < 4; i_0++) {
        poseErrorMat[i_1 + (i_0 << 2)] = 0.0;
        poseErrorMat[i_1 + (i_0 << 2)] += mirageTracking_P.ObjectModel_Value[i_0
          << 2] * world2robotDes[i_1];
        poseErrorMat[i_1 + (i_0 << 2)] += mirageTracking_P.ObjectModel_Value
          [(i_0 << 2) + 1] * world2robotDes[i_1 + 4];
        poseErrorMat[i_1 + (i_0 << 2)] += mirageTracking_P.ObjectModel_Value
          [(i_0 << 2) + 2] * world2robotDes[i_1 + 8];
        poseErrorMat[i_1 + (i_0 << 2)] += mirageTracking_P.ObjectModel_Value
          [(i_0 << 2) + 3] * world2robotDes[i_1 + 12];
      }
    }

    mirageTracking_visioMotor(lwc, rwc, poseErrorMat, errParams);

    /* '<S4>:1:19' */
    for (i_1 = 0; i_1 < 16; i_1++) {
      poseErrorMat[i_1] = b_0[i_1];
    }

    /* '<S4>:1:20' */
    memcpy(&y[0], &errParams[0], 12U * sizeof(real_T));
    for (i_1 = 0; i_1 < 4; i_1++) {
      poseErrorMat[i_1 << 2] = y[3 * i_1];
      poseErrorMat[1 + (i_1 << 2)] = y[3 * i_1 + 1];
      poseErrorMat[2 + (i_1 << 2)] = y[3 * i_1 + 2];
    }

    /* '<S4>:1:22' */
    for (i_1 = 0; i_1 < 4; i_1++) {
      for (i_0 = 0; i_0 < 4; i_0++) {
        world2robot[i_1 + (i_0 << 2)] = 0.0;
        world2robot[i_1 + (i_0 << 2)] += world2robotDes[i_0 << 2] *
          poseErrorMat[i_1];
        world2robot[i_1 + (i_0 << 2)] += world2robotDes[(i_0 << 2) + 1] *
          poseErrorMat[i_1 + 4];
        world2robot[i_1 + (i_0 << 2)] += world2robotDes[(i_0 << 2) + 2] *
          poseErrorMat[i_1 + 8];
        world2robot[i_1 + (i_0 << 2)] += world2robotDes[(i_0 << 2) + 3] *
          poseErrorMat[i_1 + 12];
      }
    }

    /* '<S4>:1:24' */
    /*  This function get the rotation matrix and computes the euler angles. */
    /*  rotMat :(3x3) rotation matrix  */
    /*  */
    /*  roll  : angle phi */
    /*  pitch : angle theta */
    /*  yaw   : angle psi */
    theta = rt_atan2d_snf(-world2robot[2], sqrt(world2robot[0] * world2robot[0]
      + world2robot[1] * world2robot[1]));

    /* '<S4>:1:26' */
    mirageTracking_B.qtilde[0] = -world2robot[12] - (real_T)rtb_qr[0];
    mirageTracking_B.qtilde[1] = -world2robot[13] - (real_T)rtb_qr[1];
    mirageTracking_B.qtilde[2] = -world2robot[14] - (real_T)rtb_qr[2];
    mirageTracking_B.qtilde[3] = -rt_atan2d_snf(world2robot[6] / cos(theta),
      world2robot[10] / cos(theta)) - (real_T)rtb_qr[3];
    mirageTracking_B.qtilde[4] = -theta - (real_T)rtb_qr[4];
    mirageTracking_B.qtilde[5] = -rt_atan2d_snf(world2robot[1] / cos(theta),
      world2robot[0] / cos(theta)) - (real_T)rtb_qr[5];

    /* End of MATLAB Function: '<Root>/Estimate Pose Error' */

    /* ToWorkspace: '<Root>/Pose Error' */
    rt_UpdateLogVar((LogVar *)(LogVar*)
                    (mirageTracking_DW.PoseError_PWORK.LoggedData),
                    &mirageTracking_B.qtilde[0], 0);
  }

  /* Matfile logging */
  rt_UpdateTXYLogVars(mirageTracking_M->rtwLogInfo, (mirageTracking_M->Timing.t));

  {
    int32_T i;
    for (i = 0; i < 8; i++) {
      /* Update for Memory: '<Root>/Left Cam Centers' */
      mirageTracking_DW.LeftCamCenters_PreviousInput[i] = mirageTracking_B.L_p[i];

      /* Update for Memory: '<Root>/Right Cam Centers' */
      mirageTracking_DW.RightCamCenters_PreviousInput[i] =
        mirageTracking_B.R_p[i];
    }
  }

  /* signal main to stop simulation */
  {                                    /* Sample time: [0.0s, 0.0s] */
    if ((rtmGetTFinal(mirageTracking_M)!=-1) &&
        !((rtmGetTFinal(mirageTracking_M)-mirageTracking_M->Timing.t[0]) >
          mirageTracking_M->Timing.t[0] * (DBL_EPSILON))) {
      rtmSetErrorStatus(mirageTracking_M, "Simulation finished");
    }
  }

  /* Update absolute time for base rate */
  /* The "clockTick0" counts the number of times the code of this task has
   * been executed. The absolute time is the multiplication of "clockTick0"
   * and "Timing.stepSize0". Size of "clockTick0" ensures timer will not
   * overflow during the application lifespan selected.
   * Timer of this task consists of two 32 bit unsigned integers.
   * The two integers represent the low bits Timing.clockTick0 and the high bits
   * Timing.clockTickH0. When the low bit overflows to 0, the high bits increment.
   */
  if (!(++mirageTracking_M->Timing.clockTick0)) {
    ++mirageTracking_M->Timing.clockTickH0;
  }

  mirageTracking_M->Timing.t[0] = mirageTracking_M->Timing.clockTick0 *
    mirageTracking_M->Timing.stepSize0 + mirageTracking_M->Timing.clockTickH0 *
    mirageTracking_M->Timing.stepSize0 * 4294967296.0;

  {
    /* Update absolute timer for sample time: [0.033333333333333333s, 0.0s] */
    /* The "clockTick1" counts the number of times the code of this task has
     * been executed. The resolution of this integer timer is 0.033333333333333333, which is the step size
     * of the task. Size of "clockTick1" ensures timer will not overflow during the
     * application lifespan selected.
     * Timer of this task consists of two 32 bit unsigned integers.
     * The two integers represent the low bits Timing.clockTick1 and the high bits
     * Timing.clockTickH1. When the low bit overflows to 0, the high bits increment.
     */
    mirageTracking_M->Timing.clockTick1++;
    if (!mirageTracking_M->Timing.clockTick1) {
      mirageTracking_M->Timing.clockTickH1++;
    }
  }
}

/* Model initialize function */
void mirageTracking_initialize(void)
{
  /* Registration code */

  /* initialize non-finites */
  rt_InitInfAndNaN(sizeof(real_T));

  /* initialize real-time model */
  (void) memset((void *)mirageTracking_M, 0,
                sizeof(RT_MODEL_mirageTracking_T));

  {
    /* Setup solver object */
    rtsiSetSimTimeStepPtr(&mirageTracking_M->solverInfo,
                          &mirageTracking_M->Timing.simTimeStep);
    rtsiSetTPtr(&mirageTracking_M->solverInfo, &rtmGetTPtr(mirageTracking_M));
    rtsiSetStepSizePtr(&mirageTracking_M->solverInfo,
                       &mirageTracking_M->Timing.stepSize0);
    rtsiSetErrorStatusPtr(&mirageTracking_M->solverInfo, (&rtmGetErrorStatus
      (mirageTracking_M)));
    rtsiSetRTModelPtr(&mirageTracking_M->solverInfo, mirageTracking_M);
  }

  rtsiSetSimTimeStep(&mirageTracking_M->solverInfo, MAJOR_TIME_STEP);
  rtsiSetSolverName(&mirageTracking_M->solverInfo,"FixedStepDiscrete");
  rtmSetTPtr(mirageTracking_M, &mirageTracking_M->Timing.tArray[0]);
  rtmSetTFinal(mirageTracking_M, 10.0);
  mirageTracking_M->Timing.stepSize0 = 0.033333333333333333;

  /* Setup for data logging */
  {
    static RTWLogInfo rt_DataLoggingInfo;
    mirageTracking_M->rtwLogInfo = &rt_DataLoggingInfo;
  }

  /* Setup for data logging */
  {
    rtliSetLogXSignalInfo(mirageTracking_M->rtwLogInfo, (NULL));
    rtliSetLogXSignalPtrs(mirageTracking_M->rtwLogInfo, (NULL));
    rtliSetLogT(mirageTracking_M->rtwLogInfo, "tout");
    rtliSetLogX(mirageTracking_M->rtwLogInfo, "");
    rtliSetLogXFinal(mirageTracking_M->rtwLogInfo, "");
    rtliSetLogVarNameModifier(mirageTracking_M->rtwLogInfo, "rt_");
    rtliSetLogFormat(mirageTracking_M->rtwLogInfo, 0);
    rtliSetLogMaxRows(mirageTracking_M->rtwLogInfo, 1000);
    rtliSetLogDecimation(mirageTracking_M->rtwLogInfo, 1);
    rtliSetLogY(mirageTracking_M->rtwLogInfo, "");
    rtliSetLogYSignalInfo(mirageTracking_M->rtwLogInfo, (NULL));
    rtliSetLogYSignalPtrs(mirageTracking_M->rtwLogInfo, (NULL));
  }

  /* block I/O */
  (void) memset(((void *) &mirageTracking_B), 0,
                sizeof(B_mirageTracking_T));

  /* states (dwork) */
  (void) memset((void *)&mirageTracking_DW, 0,
                sizeof(DW_mirageTracking_T));

  /* Matfile logging */
  rt_StartDataLoggingWithStartTime(mirageTracking_M->rtwLogInfo, 0.0,
    rtmGetTFinal(mirageTracking_M), mirageTracking_M->Timing.stepSize0,
    (&rtmGetErrorStatus(mirageTracking_M)));

  {
    char_T *sErr;
    char * deviceInfo[7];
    char * triggerInfo[3];
    char * colorSpaceInfo[2];
    int32_T roiArray[4];
    char * propNames[19];
    int32_T propLengths[19];
    void *propValues[19];
    real_T propNumericRef;
    real_T propNumericRef_0;
    real_T propNumericRef_1;
    real_T propNumericRef_2;
    real_T propNumericRef_3;
    real_T propNumericRef_4;
    real_T propNumericRef_5;
    real_T propNumericRef_6;
    real_T propNumericRef_7;
    real_T propNumericRef_8;
    real_T propNumericRef_9;
    char * selectedMetadata;
    char * featureNames[25];
    void *featureValues[25];
    int32_T featureTypes[25];
    real_T currentFeatureRef;
    real_T currentFeatureRef_0;
    real_T currentFeatureRef_1;
    real_T currentFeatureRef_2;
    real_T currentFeatureRef_3;
    real_T currentFeatureRef_4;
    real_T currentFeatureRef_5;
    real_T currentFeatureRef_6;
    real_T currentFeatureRef_7;
    real_T currentFeatureRef_8;
    real_T currentFeatureRef_9;
    real_T currentFeatureRef_a;
    real_T currentFeatureRef_b;
    real_T currentFeatureRef_c;
    real_T currentFeatureRef_d;
    real_T currentFeatureRef_e;
    real_T currentFeatureRef_f;
    real_T currentFeatureRef_g;
    real_T currentFeatureRef_h;
    real_T currentFeatureRef_i;
    real_T currentFeatureRef_j;
    real_T currentFeatureRef_k;
    real_T currentFeatureRef_l;
    real_T currentFeatureRef_m;
    real_T currentFeatureRef_n;
    char * deviceInfo_0[7];
    char * triggerInfo_0[3];
    char * colorSpaceInfo_0[2];
    int32_T roiArray_0[4];
    char * propNames_0[19];
    int32_T propLengths_0[19];
    void *propValues_0[19];
    real_T propNumericRef_a;
    real_T propNumericRef_b;
    real_T propNumericRef_c;
    real_T propNumericRef_d;
    real_T propNumericRef_e;
    real_T propNumericRef_f;
    real_T propNumericRef_g;
    real_T propNumericRef_h;
    real_T propNumericRef_i;
    real_T propNumericRef_j;
    real_T propNumericRef_k;
    char * selectedMetadata_0;
    char * featureNames_0[25];
    void *featureValues_0[25];
    int32_T featureTypes_0[25];
    real_T currentFeatureRef_o;
    real_T currentFeatureRef_p;
    real_T currentFeatureRef_q;
    real_T currentFeatureRef_r;
    real_T currentFeatureRef_s;
    real_T currentFeatureRef_t;
    real_T currentFeatureRef_u;
    real_T currentFeatureRef_v;
    real_T currentFeatureRef_w;
    real_T currentFeatureRef_x;
    real_T currentFeatureRef_y;
    real_T currentFeatureRef_z;
    real_T currentFeatureRef_10;
    real_T currentFeatureRef_11;
    real_T currentFeatureRef_12;
    real_T currentFeatureRef_13;
    real_T currentFeatureRef_14;
    real_T currentFeatureRef_15;
    real_T currentFeatureRef_16;
    real_T currentFeatureRef_17;
    real_T currentFeatureRef_18;
    real_T currentFeatureRef_19;
    real_T currentFeatureRef_1a;
    real_T currentFeatureRef_1b;
    real_T currentFeatureRef_1c;

    /* Start for S-Function (simaqfvd): '<Root>/Left Camera' */
    sErr = GetErrorBuffer(&mirageTracking_DW.LeftCamera_FromVideoDevice[0U]);
    CreateHostLibrary("C:\\Program Files\\MATLAB\\R2015a\\toolbox\\imaq\\imaqblks\\imaqmex\\win64\\imaqmex.mexw64",
                      &mirageTracking_DW.LeftCamera_FromVideoDevice[0U]);
    if (*sErr != 0) {
      CreateHostLibrary("imaqmex.mexw64",
                        &mirageTracking_DW.LeftCamera_FromVideoDevice[0U]);
    }

    if (*sErr == 0) {
      deviceInfo[0U] = "winvideo";
      deviceInfo[1U] =
        "C:\\MATLAB\\SupportPackages\\R2015a\\osgenericvideointerface\\toolbox\\imaq\\supportpackages\\genericvideo\\adaptor\\win64\\mwwinvideoimaq.dll";
      deviceInfo[2U] = "Logitech HD Webcam C525";
      deviceInfo[3U] = "RGB24_1280x960";
      deviceInfo[4U] = "input1";
      deviceInfo[5U] =
        "C:\\Program Files\\MATLAB\\R2015a\\toolbox\\imaq\\imaq\\private\\imaqmex.imdf";
      deviceInfo[6U] =
        "C:\\MATLAB\\SupportPackages\\R2015a\\osgenericvideointerface\\toolbox\\imaq\\supportpackages\\genericvideo\\adaptor\\win64\\mwwinvideoimaq.imdf";
      triggerInfo[0U] = "manual";
      triggerInfo[1U] = "none";
      triggerInfo[2U] = "none";
      colorSpaceInfo[0U] = "rgb";
      colorSpaceInfo[1U] = "grbg";
      roiArray[0U] = 0;
      roiArray[1U] = 0;
      roiArray[2U] = 1280;
      roiArray[3U] = 960;
      propNames[0U] = "BacklightCompensation";
      propLengths[0U] = 2;
      propValues[0U] = (void *)"on";
      propNames[1U] = "Brightness";
      propLengths[1U] = 1;
      propNumericRef = 128.0;
      propValues[1U] = (void *)&propNumericRef;
      propNames[2U] = "Contrast";
      propLengths[2U] = 1;
      propNumericRef_0 = 32.0;
      propValues[2U] = (void *)&propNumericRef_0;
      propNames[3U] = "Exposure";
      propLengths[3U] = 1;
      propNumericRef_1 = -7.0;
      propValues[3U] = (void *)&propNumericRef_1;
      propNames[4U] = "ExposureMode";
      propLengths[4U] = 4;
      propValues[4U] = (void *)"auto";
      propNames[5U] = "Focus";
      propLengths[5U] = 1;
      propNumericRef_2 = 60.0;
      propValues[5U] = (void *)&propNumericRef_2;
      propNames[6U] = "FocusMode";
      propLengths[6U] = 4;
      propValues[6U] = (void *)"auto";
      propNames[7U] = "FrameRate";
      propLengths[7U] = 7;
      propValues[7U] = (void *)"30.0000";
      propNames[8U] = "Gain";
      propLengths[8U] = 1;
      propNumericRef_3 = 64.0;
      propValues[8U] = (void *)&propNumericRef_3;
      propNames[9U] = "HorizontalFlip";
      propLengths[9U] = 3;
      propValues[9U] = (void *)"off";
      propNames[10U] = "Pan";
      propLengths[10U] = 1;
      propNumericRef_4 = 0.0;
      propValues[10U] = (void *)&propNumericRef_4;
      propNames[11U] = "Saturation";
      propLengths[11U] = 1;
      propNumericRef_5 = 32.0;
      propValues[11U] = (void *)&propNumericRef_5;
      propNames[12U] = "Sharpness";
      propLengths[12U] = 1;
      propNumericRef_6 = 22.0;
      propValues[12U] = (void *)&propNumericRef_6;
      propNames[13U] = "Tag";
      propLengths[13U] = 0;
      propValues[13U] = (void *)"";
      propNames[14U] = "Tilt";
      propLengths[14U] = 1;
      propNumericRef_7 = 0.0;
      propValues[14U] = (void *)&propNumericRef_7;
      propNames[15U] = "VerticalFlip";
      propLengths[15U] = 3;
      propValues[15U] = (void *)"off";
      propNames[16U] = "WhiteBalance";
      propLengths[16U] = 1;
      propNumericRef_8 = 5500.0;
      propValues[16U] = (void *)&propNumericRef_8;
      propNames[17U] = "WhiteBalanceMode";
      propLengths[17U] = 4;
      propValues[17U] = (void *)"auto";
      propNames[18U] = "Zoom";
      propLengths[18U] = 1;
      propNumericRef_9 = 1.0;
      propValues[18U] = (void *)&propNumericRef_9;
      selectedMetadata = "";
      featureNames[0U] = "-debug";
      featureTypes[0U] = 1;
      currentFeatureRef = 0.0;
      featureValues[0U] = (void *)&currentFeatureRef;
      featureNames[1U] = "-debuggentlacquisition";
      featureTypes[1U] = 1;
      currentFeatureRef_0 = 0.0;
      featureValues[1U] = (void *)&currentFeatureRef_0;
      featureNames[2U] = "-debuggentldiscovery";
      featureTypes[2U] = 1;
      currentFeatureRef_1 = 0.0;
      featureValues[2U] = (void *)&currentFeatureRef_1;
      featureNames[3U] = "-debuggentloperation";
      featureTypes[3U] = 1;
      currentFeatureRef_2 = 0.0;
      featureValues[3U] = (void *)&currentFeatureRef_2;
      featureNames[4U] = "-debuggigeconnect";
      featureTypes[4U] = 1;
      currentFeatureRef_3 = 0.0;
      featureValues[4U] = (void *)&currentFeatureRef_3;
      featureNames[5U] = "-debuggigediscovery";
      featureTypes[5U] = 1;
      currentFeatureRef_4 = 0.0;
      featureValues[5U] = (void *)&currentFeatureRef_4;
      featureNames[6U] = "-debuggigeframeassembly";
      featureTypes[6U] = 1;
      currentFeatureRef_5 = 0.0;
      featureValues[6U] = (void *)&currentFeatureRef_5;
      featureNames[7U] = "-debuggigegvspreception";
      featureTypes[7U] = 1;
      currentFeatureRef_6 = 0.0;
      featureValues[7U] = (void *)&currentFeatureRef_6;
      featureNames[8U] = "-debuggigeopen";
      featureTypes[8U] = 1;
      currentFeatureRef_7 = 0.0;
      featureValues[8U] = (void *)&currentFeatureRef_7;
      featureNames[9U] = "-debuggigepacketresend";
      featureTypes[9U] = 1;
      currentFeatureRef_8 = 0.0;
      featureValues[9U] = (void *)&currentFeatureRef_8;
      featureNames[10U] = "-genicamcommandsavailable";
      featureTypes[10U] = 1;
      currentFeatureRef_9 = 0.0;
      featureValues[10U] = (void *)&currentFeatureRef_9;
      featureNames[11U] = "-gigedisableforceip";
      featureTypes[11U] = 1;
      currentFeatureRef_a = 0.0;
      featureValues[11U] = (void *)&currentFeatureRef_a;
      featureNames[12U] = "-gigedisablepacketresend";
      featureTypes[12U] = 1;
      currentFeatureRef_b = 0.0;
      featureValues[12U] = (void *)&currentFeatureRef_b;
      featureNames[13U] = "-logallevents";
      featureTypes[13U] = 1;
      currentFeatureRef_c = 0.0;
      featureValues[13U] = (void *)&currentFeatureRef_c;
      featureNames[14U] = "-previewfullbitdepth";
      featureTypes[14U] = 1;
      currentFeatureRef_d = 0.0;
      featureValues[14U] = (void *)&currentFeatureRef_d;
      featureNames[15U] = "-slowpreview";
      featureTypes[15U] = 1;
      currentFeatureRef_e = 0.0;
      featureValues[15U] = (void *)&currentFeatureRef_e;
      featureNames[16U] = "-usedcamlittleendian";
      featureTypes[16U] = 1;
      currentFeatureRef_f = 0.0;
      featureValues[16U] = (void *)&currentFeatureRef_f;
      featureNames[17U] = "-useobsoletepreview";
      featureTypes[17U] = 1;
      currentFeatureRef_g = 0.0;
      featureValues[17U] = (void *)&currentFeatureRef_g;
      featureNames[18U] = "-vfw";
      featureTypes[18U] = 1;
      currentFeatureRef_h = 1.0;
      featureValues[18U] = (void *)&currentFeatureRef_h;
      featureNames[19U] = "-gigecommandpacketretries";
      featureTypes[19U] = 0;
      currentFeatureRef_i = 1.0;
      featureValues[19U] = (void *)&currentFeatureRef_i;
      featureNames[20U] = "-gigeheartbeattimeout";
      featureTypes[20U] = 0;
      currentFeatureRef_j = 1000.0;
      featureValues[20U] = (void *)&currentFeatureRef_j;
      featureNames[21U] = "-gigepacketacktimeout";
      featureTypes[21U] = 0;
      currentFeatureRef_k = 500.0;
      featureValues[21U] = (void *)&currentFeatureRef_k;
      featureNames[22U] = "-macvideoframegrabduringdevicediscoverytimeout";
      featureTypes[22U] = 0;
      currentFeatureRef_l = 10000.0;
      featureValues[22U] = (void *)&currentFeatureRef_l;
      featureNames[23U] = "-pointgreystartdelay";
      featureTypes[23U] = 0;
      currentFeatureRef_m = 500.0;
      featureValues[23U] = (void *)&currentFeatureRef_m;
      featureNames[24U] = "-pointgreystopdelay";
      featureTypes[24U] = 0;
      currentFeatureRef_n = 750.0;
      featureValues[24U] = (void *)&currentFeatureRef_n;
      LibCreate_FVD(&mirageTracking_DW.LeftCamera_FromVideoDevice[0U],
                    &deviceInfo[0U], 1, &triggerInfo[0U], (rtInf), 1.0,
                    &roiArray[0U], &colorSpaceInfo[0U], 19, &propNames[0U],
                    &propLengths[0U], &propValues[0U], 25, &featureNames[0U],
                    &featureTypes[0U], &featureValues[0U], &selectedMetadata);
    }

    if (*sErr == 0) {
      LibStart(&mirageTracking_DW.LeftCamera_FromVideoDevice[0U]);
    }

    if (*sErr != 0) {
      rtmSetErrorStatus(mirageTracking_M, sErr);
      rtmSetStopRequested(mirageTracking_M, 1);
    }

    /* End of Start for S-Function (simaqfvd): '<Root>/Left Camera' */

    /* Start for S-Function (simaqfvd): '<Root>/Right Camera' */
    sErr = GetErrorBuffer(&mirageTracking_DW.RightCamera_FromVideoDevice[0U]);
    CreateHostLibrary("C:\\Program Files\\MATLAB\\R2015a\\toolbox\\imaq\\imaqblks\\imaqmex\\win64\\imaqmex.mexw64",
                      &mirageTracking_DW.RightCamera_FromVideoDevice[0U]);
    if (*sErr != 0) {
      CreateHostLibrary("imaqmex.mexw64",
                        &mirageTracking_DW.RightCamera_FromVideoDevice[0U]);
    }

    if (*sErr == 0) {
      deviceInfo_0[0U] = "winvideo";
      deviceInfo_0[1U] =
        "C:\\MATLAB\\SupportPackages\\R2015a\\osgenericvideointerface\\toolbox\\imaq\\supportpackages\\genericvideo\\adaptor\\win64\\mwwinvideoimaq.dll";
      deviceInfo_0[2U] = "Logitech HD Webcam C525";
      deviceInfo_0[3U] = "RGB24_1280x960";
      deviceInfo_0[4U] = "input1";
      deviceInfo_0[5U] =
        "C:\\Program Files\\MATLAB\\R2015a\\toolbox\\imaq\\imaq\\private\\imaqmex.imdf";
      deviceInfo_0[6U] =
        "C:\\MATLAB\\SupportPackages\\R2015a\\osgenericvideointerface\\toolbox\\imaq\\supportpackages\\genericvideo\\adaptor\\win64\\mwwinvideoimaq.imdf";
      triggerInfo_0[0U] = "manual";
      triggerInfo_0[1U] = "none";
      triggerInfo_0[2U] = "none";
      colorSpaceInfo_0[0U] = "rgb";
      colorSpaceInfo_0[1U] = "grbg";
      roiArray_0[0U] = 0;
      roiArray_0[1U] = 0;
      roiArray_0[2U] = 1280;
      roiArray_0[3U] = 960;
      propNames_0[0U] = "BacklightCompensation";
      propLengths_0[0U] = 2;
      propValues_0[0U] = (void *)"on";
      propNames_0[1U] = "Brightness";
      propLengths_0[1U] = 1;
      propNumericRef_a = 128.0;
      propValues_0[1U] = (void *)&propNumericRef_a;
      propNames_0[2U] = "Contrast";
      propLengths_0[2U] = 1;
      propNumericRef_b = 32.0;
      propValues_0[2U] = (void *)&propNumericRef_b;
      propNames_0[3U] = "Exposure";
      propLengths_0[3U] = 1;
      propNumericRef_c = -7.0;
      propValues_0[3U] = (void *)&propNumericRef_c;
      propNames_0[4U] = "ExposureMode";
      propLengths_0[4U] = 4;
      propValues_0[4U] = (void *)"auto";
      propNames_0[5U] = "Focus";
      propLengths_0[5U] = 1;
      propNumericRef_d = 60.0;
      propValues_0[5U] = (void *)&propNumericRef_d;
      propNames_0[6U] = "FocusMode";
      propLengths_0[6U] = 4;
      propValues_0[6U] = (void *)"auto";
      propNames_0[7U] = "FrameRate";
      propLengths_0[7U] = 7;
      propValues_0[7U] = (void *)"30.0000";
      propNames_0[8U] = "Gain";
      propLengths_0[8U] = 1;
      propNumericRef_e = 64.0;
      propValues_0[8U] = (void *)&propNumericRef_e;
      propNames_0[9U] = "HorizontalFlip";
      propLengths_0[9U] = 3;
      propValues_0[9U] = (void *)"off";
      propNames_0[10U] = "Pan";
      propLengths_0[10U] = 1;
      propNumericRef_f = 0.0;
      propValues_0[10U] = (void *)&propNumericRef_f;
      propNames_0[11U] = "Saturation";
      propLengths_0[11U] = 1;
      propNumericRef_g = 32.0;
      propValues_0[11U] = (void *)&propNumericRef_g;
      propNames_0[12U] = "Sharpness";
      propLengths_0[12U] = 1;
      propNumericRef_h = 22.0;
      propValues_0[12U] = (void *)&propNumericRef_h;
      propNames_0[13U] = "Tag";
      propLengths_0[13U] = 0;
      propValues_0[13U] = (void *)"";
      propNames_0[14U] = "Tilt";
      propLengths_0[14U] = 1;
      propNumericRef_i = 0.0;
      propValues_0[14U] = (void *)&propNumericRef_i;
      propNames_0[15U] = "VerticalFlip";
      propLengths_0[15U] = 3;
      propValues_0[15U] = (void *)"off";
      propNames_0[16U] = "WhiteBalance";
      propLengths_0[16U] = 1;
      propNumericRef_j = 5500.0;
      propValues_0[16U] = (void *)&propNumericRef_j;
      propNames_0[17U] = "WhiteBalanceMode";
      propLengths_0[17U] = 4;
      propValues_0[17U] = (void *)"auto";
      propNames_0[18U] = "Zoom";
      propLengths_0[18U] = 1;
      propNumericRef_k = 1.0;
      propValues_0[18U] = (void *)&propNumericRef_k;
      selectedMetadata_0 = "";
      featureNames_0[0U] = "-debug";
      featureTypes_0[0U] = 1;
      currentFeatureRef_o = 0.0;
      featureValues_0[0U] = (void *)&currentFeatureRef_o;
      featureNames_0[1U] = "-debuggentlacquisition";
      featureTypes_0[1U] = 1;
      currentFeatureRef_p = 0.0;
      featureValues_0[1U] = (void *)&currentFeatureRef_p;
      featureNames_0[2U] = "-debuggentldiscovery";
      featureTypes_0[2U] = 1;
      currentFeatureRef_q = 0.0;
      featureValues_0[2U] = (void *)&currentFeatureRef_q;
      featureNames_0[3U] = "-debuggentloperation";
      featureTypes_0[3U] = 1;
      currentFeatureRef_r = 0.0;
      featureValues_0[3U] = (void *)&currentFeatureRef_r;
      featureNames_0[4U] = "-debuggigeconnect";
      featureTypes_0[4U] = 1;
      currentFeatureRef_s = 0.0;
      featureValues_0[4U] = (void *)&currentFeatureRef_s;
      featureNames_0[5U] = "-debuggigediscovery";
      featureTypes_0[5U] = 1;
      currentFeatureRef_t = 0.0;
      featureValues_0[5U] = (void *)&currentFeatureRef_t;
      featureNames_0[6U] = "-debuggigeframeassembly";
      featureTypes_0[6U] = 1;
      currentFeatureRef_u = 0.0;
      featureValues_0[6U] = (void *)&currentFeatureRef_u;
      featureNames_0[7U] = "-debuggigegvspreception";
      featureTypes_0[7U] = 1;
      currentFeatureRef_v = 0.0;
      featureValues_0[7U] = (void *)&currentFeatureRef_v;
      featureNames_0[8U] = "-debuggigeopen";
      featureTypes_0[8U] = 1;
      currentFeatureRef_w = 0.0;
      featureValues_0[8U] = (void *)&currentFeatureRef_w;
      featureNames_0[9U] = "-debuggigepacketresend";
      featureTypes_0[9U] = 1;
      currentFeatureRef_x = 0.0;
      featureValues_0[9U] = (void *)&currentFeatureRef_x;
      featureNames_0[10U] = "-genicamcommandsavailable";
      featureTypes_0[10U] = 1;
      currentFeatureRef_y = 0.0;
      featureValues_0[10U] = (void *)&currentFeatureRef_y;
      featureNames_0[11U] = "-gigedisableforceip";
      featureTypes_0[11U] = 1;
      currentFeatureRef_z = 0.0;
      featureValues_0[11U] = (void *)&currentFeatureRef_z;
      featureNames_0[12U] = "-gigedisablepacketresend";
      featureTypes_0[12U] = 1;
      currentFeatureRef_10 = 0.0;
      featureValues_0[12U] = (void *)&currentFeatureRef_10;
      featureNames_0[13U] = "-logallevents";
      featureTypes_0[13U] = 1;
      currentFeatureRef_11 = 0.0;
      featureValues_0[13U] = (void *)&currentFeatureRef_11;
      featureNames_0[14U] = "-previewfullbitdepth";
      featureTypes_0[14U] = 1;
      currentFeatureRef_12 = 0.0;
      featureValues_0[14U] = (void *)&currentFeatureRef_12;
      featureNames_0[15U] = "-slowpreview";
      featureTypes_0[15U] = 1;
      currentFeatureRef_13 = 0.0;
      featureValues_0[15U] = (void *)&currentFeatureRef_13;
      featureNames_0[16U] = "-usedcamlittleendian";
      featureTypes_0[16U] = 1;
      currentFeatureRef_14 = 0.0;
      featureValues_0[16U] = (void *)&currentFeatureRef_14;
      featureNames_0[17U] = "-useobsoletepreview";
      featureTypes_0[17U] = 1;
      currentFeatureRef_15 = 0.0;
      featureValues_0[17U] = (void *)&currentFeatureRef_15;
      featureNames_0[18U] = "-vfw";
      featureTypes_0[18U] = 1;
      currentFeatureRef_16 = 1.0;
      featureValues_0[18U] = (void *)&currentFeatureRef_16;
      featureNames_0[19U] = "-gigecommandpacketretries";
      featureTypes_0[19U] = 0;
      currentFeatureRef_17 = 1.0;
      featureValues_0[19U] = (void *)&currentFeatureRef_17;
      featureNames_0[20U] = "-gigeheartbeattimeout";
      featureTypes_0[20U] = 0;
      currentFeatureRef_18 = 1000.0;
      featureValues_0[20U] = (void *)&currentFeatureRef_18;
      featureNames_0[21U] = "-gigepacketacktimeout";
      featureTypes_0[21U] = 0;
      currentFeatureRef_19 = 500.0;
      featureValues_0[21U] = (void *)&currentFeatureRef_19;
      featureNames_0[22U] = "-macvideoframegrabduringdevicediscoverytimeout";
      featureTypes_0[22U] = 0;
      currentFeatureRef_1a = 10000.0;
      featureValues_0[22U] = (void *)&currentFeatureRef_1a;
      featureNames_0[23U] = "-pointgreystartdelay";
      featureTypes_0[23U] = 0;
      currentFeatureRef_1b = 500.0;
      featureValues_0[23U] = (void *)&currentFeatureRef_1b;
      featureNames_0[24U] = "-pointgreystopdelay";
      featureTypes_0[24U] = 0;
      currentFeatureRef_1c = 750.0;
      featureValues_0[24U] = (void *)&currentFeatureRef_1c;
      LibCreate_FVD(&mirageTracking_DW.RightCamera_FromVideoDevice[0U],
                    &deviceInfo_0[0U], 2, &triggerInfo_0[0U], (rtInf), 1.0,
                    &roiArray_0[0U], &colorSpaceInfo_0[0U], 19, &propNames_0[0U],
                    &propLengths_0[0U], &propValues_0[0U], 25, &featureNames_0
                    [0U], &featureTypes_0[0U], &featureValues_0[0U],
                    &selectedMetadata_0);
    }

    if (*sErr == 0) {
      LibStart(&mirageTracking_DW.RightCamera_FromVideoDevice[0U]);
    }

    if (*sErr != 0) {
      rtmSetErrorStatus(mirageTracking_M, sErr);
      rtmSetStopRequested(mirageTracking_M, 1);
    }

    /* End of Start for S-Function (simaqfvd): '<Root>/Right Camera' */
    /* Start for ToWorkspace: '<Root>/Pose Error' */
    {
      int_T dimensions[1] = { 6 };

      mirageTracking_DW.PoseError_PWORK.LoggedData = rt_CreateLogVar(
        mirageTracking_M->rtwLogInfo,
        0.0,
        rtmGetTFinal(mirageTracking_M),
        mirageTracking_M->Timing.stepSize0,
        (&rtmGetErrorStatus(mirageTracking_M)),
        "q",
        SS_DOUBLE,
        0,
        0,
        0,
        6,
        1,
        dimensions,
        NO_LOGVALDIMS,
        (NULL),
        (NULL),
        0,
        1,
        0.033333333333333333,
        1);
      if (mirageTracking_DW.PoseError_PWORK.LoggedData == (NULL))
        return;
    }
  }

  {
    int32_T i;
    for (i = 0; i < 8; i++) {
      /* InitializeConditions for Memory: '<Root>/Left Cam Centers' */
      mirageTracking_DW.LeftCamCenters_PreviousInput[i] =
        mirageTracking_P.LeftCamCenters_X0[i];

      /* InitializeConditions for Memory: '<Root>/Right Cam Centers' */
      mirageTracking_DW.RightCamCenters_PreviousInput[i] =
        mirageTracking_P.RightCamCenters_X0[i];
    }
  }
}

/* Model terminate function */
void mirageTracking_terminate(void)
{
  char_T *sErr;

  /* Terminate for S-Function (simaqfvd): '<Root>/Left Camera' */
  sErr = GetErrorBuffer(&mirageTracking_DW.LeftCamera_FromVideoDevice[0U]);
  LibTerminate(&mirageTracking_DW.LeftCamera_FromVideoDevice[0U]);
  if (*sErr != 0) {
    rtmSetErrorStatus(mirageTracking_M, sErr);
    rtmSetStopRequested(mirageTracking_M, 1);
  }

  LibDestroy(&mirageTracking_DW.LeftCamera_FromVideoDevice[0U], 0);
  DestroyHostLibrary(&mirageTracking_DW.LeftCamera_FromVideoDevice[0U]);

  /* End of Terminate for S-Function (simaqfvd): '<Root>/Left Camera' */

  /* Terminate for S-Function (simaqfvd): '<Root>/Right Camera' */
  sErr = GetErrorBuffer(&mirageTracking_DW.RightCamera_FromVideoDevice[0U]);
  LibTerminate(&mirageTracking_DW.RightCamera_FromVideoDevice[0U]);
  if (*sErr != 0) {
    rtmSetErrorStatus(mirageTracking_M, sErr);
    rtmSetStopRequested(mirageTracking_M, 1);
  }

  LibDestroy(&mirageTracking_DW.RightCamera_FromVideoDevice[0U], 0);
  DestroyHostLibrary(&mirageTracking_DW.RightCamera_FromVideoDevice[0U]);

  /* End of Terminate for S-Function (simaqfvd): '<Root>/Right Camera' */
}
