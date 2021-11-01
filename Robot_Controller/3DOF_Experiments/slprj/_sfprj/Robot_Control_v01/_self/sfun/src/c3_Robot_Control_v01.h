#ifndef __c3_Robot_Control_v01_h__
#define __c3_Robot_Control_v01_h__

/* Include files */
#include "sf_runtime/sfc_sf.h"
#include "sf_runtime/sfc_mex.h"
#include "rtwtypes.h"
#include "multiword_types.h"

/* Type Definitions */
#ifndef struct_sKe46gf3wfNeOrzL8yzt6nF
#define struct_sKe46gf3wfNeOrzL8yzt6nF

struct sKe46gf3wfNeOrzL8yzt6nF
{
  real_T tx;
  real_T ty;
  real_T tz;
  real_T phi;
  real_T theta;
  real_T psi;
};

#endif                                 /*struct_sKe46gf3wfNeOrzL8yzt6nF*/

#ifndef typedef_c3_sKe46gf3wfNeOrzL8yzt6nF
#define typedef_c3_sKe46gf3wfNeOrzL8yzt6nF

typedef struct sKe46gf3wfNeOrzL8yzt6nF c3_sKe46gf3wfNeOrzL8yzt6nF;

#endif                                 /*typedef_c3_sKe46gf3wfNeOrzL8yzt6nF*/

#ifndef struct_smuwpGFfhzM6MfA0WGyOk6E
#define struct_smuwpGFfhzM6MfA0WGyOk6E

struct smuwpGFfhzM6MfA0WGyOk6E
{
  real_T x;
  real_T y;
  real_T z;
  real_T phi;
  real_T theta;
  real_T psi;
};

#endif                                 /*struct_smuwpGFfhzM6MfA0WGyOk6E*/

#ifndef typedef_c3_smuwpGFfhzM6MfA0WGyOk6E
#define typedef_c3_smuwpGFfhzM6MfA0WGyOk6E

typedef struct smuwpGFfhzM6MfA0WGyOk6E c3_smuwpGFfhzM6MfA0WGyOk6E;

#endif                                 /*typedef_c3_smuwpGFfhzM6MfA0WGyOk6E*/

#ifndef struct_s6d9wPYgmS3cwGRsMJd1GAE
#define struct_s6d9wPYgmS3cwGRsMJd1GAE

struct s6d9wPYgmS3cwGRsMJd1GAE
{
  c3_sKe46gf3wfNeOrzL8yzt6nF pose;
  real_T fx;
  real_T fy;
  real_T cx;
  real_T cy;
  real_T s;
  real_T d;
};

#endif                                 /*struct_s6d9wPYgmS3cwGRsMJd1GAE*/

#ifndef typedef_c3_s6d9wPYgmS3cwGRsMJd1GAE
#define typedef_c3_s6d9wPYgmS3cwGRsMJd1GAE

typedef struct s6d9wPYgmS3cwGRsMJd1GAE c3_s6d9wPYgmS3cwGRsMJd1GAE;

#endif                                 /*typedef_c3_s6d9wPYgmS3cwGRsMJd1GAE*/

#ifndef struct_smiE7r3wwCYeiytRpYXSxxE
#define struct_smiE7r3wwCYeiytRpYXSxxE

struct smiE7r3wwCYeiytRpYXSxxE
{
  c3_sKe46gf3wfNeOrzL8yzt6nF pose;
  c3_s6d9wPYgmS3cwGRsMJd1GAE leftCamera;
  c3_s6d9wPYgmS3cwGRsMJd1GAE rightCamera;
};

#endif                                 /*struct_smiE7r3wwCYeiytRpYXSxxE*/

#ifndef typedef_c3_smiE7r3wwCYeiytRpYXSxxE
#define typedef_c3_smiE7r3wwCYeiytRpYXSxxE

typedef struct smiE7r3wwCYeiytRpYXSxxE c3_smiE7r3wwCYeiytRpYXSxxE;

#endif                                 /*typedef_c3_smiE7r3wwCYeiytRpYXSxxE*/

#ifndef struct_sZiCA83jGK4zdmds7wvlXUF
#define struct_sZiCA83jGK4zdmds7wvlXUF

struct sZiCA83jGK4zdmds7wvlXUF
{
  real_T p[8];
  real_T C_P[16];
  real_T M[16];
};

#endif                                 /*struct_sZiCA83jGK4zdmds7wvlXUF*/

#ifndef typedef_c3_sZiCA83jGK4zdmds7wvlXUF
#define typedef_c3_sZiCA83jGK4zdmds7wvlXUF

typedef struct sZiCA83jGK4zdmds7wvlXUF c3_sZiCA83jGK4zdmds7wvlXUF;

#endif                                 /*typedef_c3_sZiCA83jGK4zdmds7wvlXUF*/

#ifndef typedef_SFc3_Robot_Control_v01InstanceStruct
#define typedef_SFc3_Robot_Control_v01InstanceStruct

typedef struct {
  SimStruct *S;
  ChartInfoStruct chartInfo;
  uint32_T chartNumber;
  uint32_T instanceNumber;
  int32_T c3_sfEvent;
  boolean_T c3_isStable;
  boolean_T c3_doneDoubleBufferReInit;
  uint8_T c3_is_active_c3_Robot_Control_v01;
  uint32_T c3_method;
  boolean_T c3_method_not_empty;
  uint32_T c3_state;
  boolean_T c3_state_not_empty;
  uint32_T c3_b_state[2];
  boolean_T c3_b_state_not_empty;
  uint32_T c3_c_state[625];
  boolean_T c3_c_state_not_empty;
  real_T (*c3_q)[3];
  real_T (*c3_qtilde)[3];
  real_T (*c3_qr)[3];
} SFc3_Robot_Control_v01InstanceStruct;

#endif                                 /*typedef_SFc3_Robot_Control_v01InstanceStruct*/

/* Named Constants */

/* Variable Declarations */
extern struct SfDebugInstanceStruct *sfGlobalDebugInstanceStruct;

/* Variable Definitions */

/* Function Declarations */
extern const mxArray *sf_c3_Robot_Control_v01_get_eml_resolved_functions_info
  (void);

/* Function Definitions */
extern void sf_c3_Robot_Control_v01_get_check_sum(mxArray *plhs[]);
extern void c3_Robot_Control_v01_method_dispatcher(SimStruct *S, int_T method,
  void *data);

#endif
