/* Include files */

#include "Robot_Control_v01_sfun.h"
#include "Robot_Control_v01_sfun_debug_macros.h"
#include "c1_Robot_Control_v01.h"
#include "c2_Robot_Control_v01.h"
#include "c3_Robot_Control_v01.h"
#include "c4_Robot_Control_v01.h"

/* Type Definitions */

/* Named Constants */

/* Variable Declarations */

/* Variable Definitions */
uint32_T _Robot_Control_v01MachineNumber_;

/* Function Declarations */

/* Function Definitions */
void Robot_Control_v01_initializer(void)
{
}

void Robot_Control_v01_terminator(void)
{
}

/* SFunction Glue Code */
unsigned int sf_Robot_Control_v01_method_dispatcher(SimStruct *simstructPtr,
  unsigned int chartFileNumber, const char* specsCksum, int_T method, void *data)
{
  if (chartFileNumber==1) {
    c1_Robot_Control_v01_method_dispatcher(simstructPtr, method, data);
    return 1;
  }

  if (chartFileNumber==2) {
    c2_Robot_Control_v01_method_dispatcher(simstructPtr, method, data);
    return 1;
  }

  if (chartFileNumber==3) {
    c3_Robot_Control_v01_method_dispatcher(simstructPtr, method, data);
    return 1;
  }

  if (chartFileNumber==4) {
    c4_Robot_Control_v01_method_dispatcher(simstructPtr, method, data);
    return 1;
  }

  return 0;
}

extern void sf_Robot_Control_v01_uses_exported_functions(int nlhs, mxArray *
  plhs[], int nrhs, const mxArray * prhs[])
{
  plhs[0] = mxCreateLogicalScalar(0);
}

unsigned int sf_Robot_Control_v01_process_check_sum_call( int nlhs, mxArray *
  plhs[], int nrhs, const mxArray * prhs[] )
{

#ifdef MATLAB_MEX_FILE

  char commandName[20];
  if (nrhs<1 || !mxIsChar(prhs[0]) )
    return 0;

  /* Possible call to get the checksum */
  mxGetString(prhs[0], commandName,sizeof(commandName)/sizeof(char));
  commandName[(sizeof(commandName)/sizeof(char)-1)] = '\0';
  if (strcmp(commandName,"sf_get_check_sum"))
    return 0;
  plhs[0] = mxCreateDoubleMatrix( 1,4,mxREAL);
  if (nrhs>1 && mxIsChar(prhs[1])) {
    mxGetString(prhs[1], commandName,sizeof(commandName)/sizeof(char));
    commandName[(sizeof(commandName)/sizeof(char)-1)] = '\0';
    if (!strcmp(commandName,"machine")) {
      ((real_T *)mxGetPr((plhs[0])))[0] = (real_T)(882233260U);
      ((real_T *)mxGetPr((plhs[0])))[1] = (real_T)(1419725848U);
      ((real_T *)mxGetPr((plhs[0])))[2] = (real_T)(2066517105U);
      ((real_T *)mxGetPr((plhs[0])))[3] = (real_T)(231829313U);
    } else if (!strcmp(commandName,"exportedFcn")) {
      ((real_T *)mxGetPr((plhs[0])))[0] = (real_T)(0U);
      ((real_T *)mxGetPr((plhs[0])))[1] = (real_T)(0U);
      ((real_T *)mxGetPr((plhs[0])))[2] = (real_T)(0U);
      ((real_T *)mxGetPr((plhs[0])))[3] = (real_T)(0U);
    } else if (!strcmp(commandName,"makefile")) {
      ((real_T *)mxGetPr((plhs[0])))[0] = (real_T)(4011605029U);
      ((real_T *)mxGetPr((plhs[0])))[1] = (real_T)(3625523369U);
      ((real_T *)mxGetPr((plhs[0])))[2] = (real_T)(3797628295U);
      ((real_T *)mxGetPr((plhs[0])))[3] = (real_T)(2361107479U);
    } else if (nrhs==3 && !strcmp(commandName,"chart")) {
      unsigned int chartFileNumber;
      chartFileNumber = (unsigned int)mxGetScalar(prhs[2]);
      switch (chartFileNumber) {
       case 1:
        {
          extern void sf_c1_Robot_Control_v01_get_check_sum(mxArray *plhs[]);
          sf_c1_Robot_Control_v01_get_check_sum(plhs);
          break;
        }

       case 2:
        {
          extern void sf_c2_Robot_Control_v01_get_check_sum(mxArray *plhs[]);
          sf_c2_Robot_Control_v01_get_check_sum(plhs);
          break;
        }

       case 3:
        {
          extern void sf_c3_Robot_Control_v01_get_check_sum(mxArray *plhs[]);
          sf_c3_Robot_Control_v01_get_check_sum(plhs);
          break;
        }

       case 4:
        {
          extern void sf_c4_Robot_Control_v01_get_check_sum(mxArray *plhs[]);
          sf_c4_Robot_Control_v01_get_check_sum(plhs);
          break;
        }

       default:
        ((real_T *)mxGetPr((plhs[0])))[0] = (real_T)(0.0);
        ((real_T *)mxGetPr((plhs[0])))[1] = (real_T)(0.0);
        ((real_T *)mxGetPr((plhs[0])))[2] = (real_T)(0.0);
        ((real_T *)mxGetPr((plhs[0])))[3] = (real_T)(0.0);
      }
    } else if (!strcmp(commandName,"target")) {
      ((real_T *)mxGetPr((plhs[0])))[0] = (real_T)(3061339410U);
      ((real_T *)mxGetPr((plhs[0])))[1] = (real_T)(1991824845U);
      ((real_T *)mxGetPr((plhs[0])))[2] = (real_T)(3599338742U);
      ((real_T *)mxGetPr((plhs[0])))[3] = (real_T)(2357874978U);
    } else {
      return 0;
    }
  } else {
    ((real_T *)mxGetPr((plhs[0])))[0] = (real_T)(599365115U);
    ((real_T *)mxGetPr((plhs[0])))[1] = (real_T)(3582295624U);
    ((real_T *)mxGetPr((plhs[0])))[2] = (real_T)(1958830029U);
    ((real_T *)mxGetPr((plhs[0])))[3] = (real_T)(303066223U);
  }

  return 1;

#else

  return 0;

#endif

}

unsigned int sf_Robot_Control_v01_autoinheritance_info( int nlhs, mxArray *
  plhs[], int nrhs, const mxArray * prhs[] )
{

#ifdef MATLAB_MEX_FILE

  char commandName[32];
  char aiChksum[64];
  if (nrhs<3 || !mxIsChar(prhs[0]) )
    return 0;

  /* Possible call to get the autoinheritance_info */
  mxGetString(prhs[0], commandName,sizeof(commandName)/sizeof(char));
  commandName[(sizeof(commandName)/sizeof(char)-1)] = '\0';
  if (strcmp(commandName,"get_autoinheritance_info"))
    return 0;
  mxGetString(prhs[2], aiChksum,sizeof(aiChksum)/sizeof(char));
  aiChksum[(sizeof(aiChksum)/sizeof(char)-1)] = '\0';

  {
    unsigned int chartFileNumber;
    chartFileNumber = (unsigned int)mxGetScalar(prhs[1]);
    switch (chartFileNumber) {
     case 1:
      {
        if (strcmp(aiChksum, "oF4zu5LNwo5QGOJLAQMQCC") == 0) {
          extern mxArray *sf_c1_Robot_Control_v01_get_autoinheritance_info(void);
          plhs[0] = sf_c1_Robot_Control_v01_get_autoinheritance_info();
          break;
        }

        plhs[0] = mxCreateDoubleMatrix(0,0,mxREAL);
        break;
      }

     case 2:
      {
        if (strcmp(aiChksum, "GMftGO877Ve5Cak9JupjED") == 0) {
          extern mxArray *sf_c2_Robot_Control_v01_get_autoinheritance_info(void);
          plhs[0] = sf_c2_Robot_Control_v01_get_autoinheritance_info();
          break;
        }

        plhs[0] = mxCreateDoubleMatrix(0,0,mxREAL);
        break;
      }

     case 3:
      {
        if (strcmp(aiChksum, "lPuRJFFHqDAKN2KxUCL7hB") == 0) {
          extern mxArray *sf_c3_Robot_Control_v01_get_autoinheritance_info(void);
          plhs[0] = sf_c3_Robot_Control_v01_get_autoinheritance_info();
          break;
        }

        plhs[0] = mxCreateDoubleMatrix(0,0,mxREAL);
        break;
      }

     case 4:
      {
        if (strcmp(aiChksum, "AwsKLrCMzJCg0Vx6dNm80G") == 0) {
          extern mxArray *sf_c4_Robot_Control_v01_get_autoinheritance_info(void);
          plhs[0] = sf_c4_Robot_Control_v01_get_autoinheritance_info();
          break;
        }

        plhs[0] = mxCreateDoubleMatrix(0,0,mxREAL);
        break;
      }

     default:
      plhs[0] = mxCreateDoubleMatrix(0,0,mxREAL);
    }
  }

  return 1;

#else

  return 0;

#endif

}

unsigned int sf_Robot_Control_v01_get_eml_resolved_functions_info( int nlhs,
  mxArray * plhs[], int nrhs, const mxArray * prhs[] )
{

#ifdef MATLAB_MEX_FILE

  char commandName[64];
  if (nrhs<2 || !mxIsChar(prhs[0]))
    return 0;

  /* Possible call to get the get_eml_resolved_functions_info */
  mxGetString(prhs[0], commandName,sizeof(commandName)/sizeof(char));
  commandName[(sizeof(commandName)/sizeof(char)-1)] = '\0';
  if (strcmp(commandName,"get_eml_resolved_functions_info"))
    return 0;

  {
    unsigned int chartFileNumber;
    chartFileNumber = (unsigned int)mxGetScalar(prhs[1]);
    switch (chartFileNumber) {
     case 1:
      {
        extern const mxArray
          *sf_c1_Robot_Control_v01_get_eml_resolved_functions_info(void);
        mxArray *persistentMxArray = (mxArray *)
          sf_c1_Robot_Control_v01_get_eml_resolved_functions_info();
        plhs[0] = mxDuplicateArray(persistentMxArray);
        mxDestroyArray(persistentMxArray);
        break;
      }

     case 2:
      {
        extern const mxArray
          *sf_c2_Robot_Control_v01_get_eml_resolved_functions_info(void);
        mxArray *persistentMxArray = (mxArray *)
          sf_c2_Robot_Control_v01_get_eml_resolved_functions_info();
        plhs[0] = mxDuplicateArray(persistentMxArray);
        mxDestroyArray(persistentMxArray);
        break;
      }

     case 3:
      {
        extern const mxArray
          *sf_c3_Robot_Control_v01_get_eml_resolved_functions_info(void);
        mxArray *persistentMxArray = (mxArray *)
          sf_c3_Robot_Control_v01_get_eml_resolved_functions_info();
        plhs[0] = mxDuplicateArray(persistentMxArray);
        mxDestroyArray(persistentMxArray);
        break;
      }

     case 4:
      {
        extern const mxArray
          *sf_c4_Robot_Control_v01_get_eml_resolved_functions_info(void);
        mxArray *persistentMxArray = (mxArray *)
          sf_c4_Robot_Control_v01_get_eml_resolved_functions_info();
        plhs[0] = mxDuplicateArray(persistentMxArray);
        mxDestroyArray(persistentMxArray);
        break;
      }

     default:
      plhs[0] = mxCreateDoubleMatrix(0,0,mxREAL);
    }
  }

  return 1;

#else

  return 0;

#endif

}

unsigned int sf_Robot_Control_v01_third_party_uses_info( int nlhs, mxArray *
  plhs[], int nrhs, const mxArray * prhs[] )
{
  char commandName[64];
  char tpChksum[64];
  if (nrhs<3 || !mxIsChar(prhs[0]))
    return 0;

  /* Possible call to get the third_party_uses_info */
  mxGetString(prhs[0], commandName,sizeof(commandName)/sizeof(char));
  commandName[(sizeof(commandName)/sizeof(char)-1)] = '\0';
  mxGetString(prhs[2], tpChksum,sizeof(tpChksum)/sizeof(char));
  tpChksum[(sizeof(tpChksum)/sizeof(char)-1)] = '\0';
  if (strcmp(commandName,"get_third_party_uses_info"))
    return 0;

  {
    unsigned int chartFileNumber;
    chartFileNumber = (unsigned int)mxGetScalar(prhs[1]);
    switch (chartFileNumber) {
     case 1:
      {
        if (strcmp(tpChksum, "IOxMZyAa9D9JarVixgmtlD") == 0) {
          extern mxArray *sf_c1_Robot_Control_v01_third_party_uses_info(void);
          plhs[0] = sf_c1_Robot_Control_v01_third_party_uses_info();
          break;
        }
      }

     case 2:
      {
        if (strcmp(tpChksum, "6IuVuc2tqBqFadNHJabVDB") == 0) {
          extern mxArray *sf_c2_Robot_Control_v01_third_party_uses_info(void);
          plhs[0] = sf_c2_Robot_Control_v01_third_party_uses_info();
          break;
        }
      }

     case 3:
      {
        if (strcmp(tpChksum, "dHXSXKPVPazPj3VhTZAqlH") == 0) {
          extern mxArray *sf_c3_Robot_Control_v01_third_party_uses_info(void);
          plhs[0] = sf_c3_Robot_Control_v01_third_party_uses_info();
          break;
        }
      }

     case 4:
      {
        if (strcmp(tpChksum, "6qosHpwK9FmL01zuSlrQiF") == 0) {
          extern mxArray *sf_c4_Robot_Control_v01_third_party_uses_info(void);
          plhs[0] = sf_c4_Robot_Control_v01_third_party_uses_info();
          break;
        }
      }

     default:
      plhs[0] = mxCreateDoubleMatrix(0,0,mxREAL);
    }
  }

  return 1;
}

unsigned int sf_Robot_Control_v01_jit_fallback_info( int nlhs, mxArray * plhs[],
  int nrhs, const mxArray * prhs[] )
{
  char commandName[64];
  char tpChksum[64];
  if (nrhs<3 || !mxIsChar(prhs[0]))
    return 0;

  /* Possible call to get the jit_fallback_info */
  mxGetString(prhs[0], commandName,sizeof(commandName)/sizeof(char));
  commandName[(sizeof(commandName)/sizeof(char)-1)] = '\0';
  mxGetString(prhs[2], tpChksum,sizeof(tpChksum)/sizeof(char));
  tpChksum[(sizeof(tpChksum)/sizeof(char)-1)] = '\0';
  if (strcmp(commandName,"get_jit_fallback_info"))
    return 0;

  {
    unsigned int chartFileNumber;
    chartFileNumber = (unsigned int)mxGetScalar(prhs[1]);
    switch (chartFileNumber) {
     case 1:
      {
        if (strcmp(tpChksum, "IOxMZyAa9D9JarVixgmtlD") == 0) {
          extern mxArray *sf_c1_Robot_Control_v01_jit_fallback_info(void);
          plhs[0] = sf_c1_Robot_Control_v01_jit_fallback_info();
          break;
        }
      }

     case 2:
      {
        if (strcmp(tpChksum, "6IuVuc2tqBqFadNHJabVDB") == 0) {
          extern mxArray *sf_c2_Robot_Control_v01_jit_fallback_info(void);
          plhs[0] = sf_c2_Robot_Control_v01_jit_fallback_info();
          break;
        }
      }

     case 3:
      {
        if (strcmp(tpChksum, "dHXSXKPVPazPj3VhTZAqlH") == 0) {
          extern mxArray *sf_c3_Robot_Control_v01_jit_fallback_info(void);
          plhs[0] = sf_c3_Robot_Control_v01_jit_fallback_info();
          break;
        }
      }

     case 4:
      {
        if (strcmp(tpChksum, "6qosHpwK9FmL01zuSlrQiF") == 0) {
          extern mxArray *sf_c4_Robot_Control_v01_jit_fallback_info(void);
          plhs[0] = sf_c4_Robot_Control_v01_jit_fallback_info();
          break;
        }
      }

     default:
      plhs[0] = mxCreateDoubleMatrix(0,0,mxREAL);
    }
  }

  return 1;
}

unsigned int sf_Robot_Control_v01_updateBuildInfo_args_info( int nlhs, mxArray *
  plhs[], int nrhs, const mxArray * prhs[] )
{
  char commandName[64];
  char tpChksum[64];
  if (nrhs<3 || !mxIsChar(prhs[0]))
    return 0;

  /* Possible call to get the updateBuildInfo_args_info */
  mxGetString(prhs[0], commandName,sizeof(commandName)/sizeof(char));
  commandName[(sizeof(commandName)/sizeof(char)-1)] = '\0';
  mxGetString(prhs[2], tpChksum,sizeof(tpChksum)/sizeof(char));
  tpChksum[(sizeof(tpChksum)/sizeof(char)-1)] = '\0';
  if (strcmp(commandName,"get_updateBuildInfo_args_info"))
    return 0;

  {
    unsigned int chartFileNumber;
    chartFileNumber = (unsigned int)mxGetScalar(prhs[1]);
    switch (chartFileNumber) {
     case 1:
      {
        if (strcmp(tpChksum, "IOxMZyAa9D9JarVixgmtlD") == 0) {
          extern mxArray *sf_c1_Robot_Control_v01_updateBuildInfo_args_info(void);
          plhs[0] = sf_c1_Robot_Control_v01_updateBuildInfo_args_info();
          break;
        }
      }

     case 2:
      {
        if (strcmp(tpChksum, "6IuVuc2tqBqFadNHJabVDB") == 0) {
          extern mxArray *sf_c2_Robot_Control_v01_updateBuildInfo_args_info(void);
          plhs[0] = sf_c2_Robot_Control_v01_updateBuildInfo_args_info();
          break;
        }
      }

     case 3:
      {
        if (strcmp(tpChksum, "dHXSXKPVPazPj3VhTZAqlH") == 0) {
          extern mxArray *sf_c3_Robot_Control_v01_updateBuildInfo_args_info(void);
          plhs[0] = sf_c3_Robot_Control_v01_updateBuildInfo_args_info();
          break;
        }
      }

     case 4:
      {
        if (strcmp(tpChksum, "6qosHpwK9FmL01zuSlrQiF") == 0) {
          extern mxArray *sf_c4_Robot_Control_v01_updateBuildInfo_args_info(void);
          plhs[0] = sf_c4_Robot_Control_v01_updateBuildInfo_args_info();
          break;
        }
      }

     default:
      plhs[0] = mxCreateDoubleMatrix(0,0,mxREAL);
    }
  }

  return 1;
}

void sf_Robot_Control_v01_get_post_codegen_info( int nlhs, mxArray * plhs[], int
  nrhs, const mxArray * prhs[] )
{
  unsigned int chartFileNumber = (unsigned int) mxGetScalar(prhs[0]);
  char tpChksum[64];
  mxGetString(prhs[1], tpChksum,sizeof(tpChksum)/sizeof(char));
  tpChksum[(sizeof(tpChksum)/sizeof(char)-1)] = '\0';
  switch (chartFileNumber) {
   case 1:
    {
      if (strcmp(tpChksum, "IOxMZyAa9D9JarVixgmtlD") == 0) {
        extern mxArray *sf_c1_Robot_Control_v01_get_post_codegen_info(void);
        plhs[0] = sf_c1_Robot_Control_v01_get_post_codegen_info();
        return;
      }
    }
    break;

   case 2:
    {
      if (strcmp(tpChksum, "6IuVuc2tqBqFadNHJabVDB") == 0) {
        extern mxArray *sf_c2_Robot_Control_v01_get_post_codegen_info(void);
        plhs[0] = sf_c2_Robot_Control_v01_get_post_codegen_info();
        return;
      }
    }
    break;

   case 3:
    {
      if (strcmp(tpChksum, "dHXSXKPVPazPj3VhTZAqlH") == 0) {
        extern mxArray *sf_c3_Robot_Control_v01_get_post_codegen_info(void);
        plhs[0] = sf_c3_Robot_Control_v01_get_post_codegen_info();
        return;
      }
    }
    break;

   case 4:
    {
      if (strcmp(tpChksum, "6qosHpwK9FmL01zuSlrQiF") == 0) {
        extern mxArray *sf_c4_Robot_Control_v01_get_post_codegen_info(void);
        plhs[0] = sf_c4_Robot_Control_v01_get_post_codegen_info();
        return;
      }
    }
    break;

   default:
    break;
  }

  plhs[0] = mxCreateDoubleMatrix(0,0,mxREAL);
}

void Robot_Control_v01_debug_initialize(struct SfDebugInstanceStruct*
  debugInstance)
{
  _Robot_Control_v01MachineNumber_ = sf_debug_initialize_machine(debugInstance,
    "Robot_Control_v01","sfun",0,4,0,0,0);
  sf_debug_set_machine_event_thresholds(debugInstance,
    _Robot_Control_v01MachineNumber_,0,0);
  sf_debug_set_machine_data_thresholds(debugInstance,
    _Robot_Control_v01MachineNumber_,0);
}

void Robot_Control_v01_register_exported_symbols(SimStruct* S)
{
}

static mxArray* sRtwOptimizationInfoStruct= NULL;
mxArray* load_Robot_Control_v01_optimization_info(void)
{
  if (sRtwOptimizationInfoStruct==NULL) {
    sRtwOptimizationInfoStruct = sf_load_rtw_optimization_info(
      "Robot_Control_v01", "Robot_Control_v01");
    mexMakeArrayPersistent(sRtwOptimizationInfoStruct);
  }

  return(sRtwOptimizationInfoStruct);
}

void unload_Robot_Control_v01_optimization_info(void)
{
  if (sRtwOptimizationInfoStruct!=NULL) {
    mxDestroyArray(sRtwOptimizationInfoStruct);
    sRtwOptimizationInfoStruct = NULL;
  }
}
