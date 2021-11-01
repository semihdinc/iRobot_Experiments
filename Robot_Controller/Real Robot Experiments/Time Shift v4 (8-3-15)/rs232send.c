/* $Revision: 1.5.4.6 $ $Date: 2009/12/22 18:55:47 $ */
/* rs232send.c - xPC Target, non-inlined S-function driver for RS-232 send (asynchronious)  */
/* Copyright 1996-2009 The MathWorks, Inc.
*/

#define S_FUNCTION_LEVEL 2
#undef S_FUNCTION_NAME
#define S_FUNCTION_NAME rs232send

#include <stddef.h>
#include <stdlib.h>

#include "tmwtypes.h"
#include "simstruc.h" 

#ifdef MATLAB_MEX_FILE
#include "mex.h"
#else
#include <windows.h>				
#include <string.h>
#include "xpcimports.h"
#endif

/* Input Arguments */
#define NUMBER_OF_ARGS        	(7)
#define PORT_ARG				ssGetSFcnParam(S,0)
#define FORMAT_SEND_ARG        	ssGetSFcnParam(S,1)
#define IN_PORT_ARG				ssGetSFcnParam(S,2)
#define NUM_IN_ARG				ssGetSFcnParam(S,3)
#define FORMAT_SEND_INFO		ssGetSFcnParam(S,4)
#define SAMP_TIME_ARG           ssGetSFcnParam(S,5)
#define WAIT_ARG				ssGetSFcnParam(S,6)

#define NO_I_WORKS              	(0)
#define NO_R_WORKS              	(0)
#define NO_P_WORKS              	(1)

#define tmpsize 					128

static char_T msg[256];

extern int rs232ports[];


static int datacount(char *formatstring)
{
  size_t i;
  int    n;

  n=0;
  for (i=0;i<strlen(formatstring);i++) {
	if (formatstring[i]=='%') n++;
  }
  return(n);
}

#ifndef MATLAB_MEX_FILE
static void build(SimStruct *S, char *formatstring, int port, int inputindex)
{

  int i, fi, j, found, k;
  char tmp[tmpsize];
  char token[tmpsize];
  char tmp_send[tmpsize];
  char tmp3[tmpsize];
  int_T portindex;
  int_T typeChar;

  strcpy (tmp_send,"");

  i=0;
  fi=0;
  k=0;
  j=0;

  while (fi < (int)strlen(formatstring)) {
  	   if (formatstring[fi]=='%') {
  	        // search format token
			typeChar = 0;

	        found=0;
	        j=fi;
			k=0;
	      	while (!found) {
			  portindex = (int)mxGetPr(IN_PORT_ARG)[i+inputindex]-1;
			  switch (formatstring[j]) {
				  case 'c':
				  case 'C':
					typeChar = 1;
				  case 'd':
				  case 'i':
				  case 'o':
				  case 'u':
				  case 'x':
				  case 'X':	{
			   		InputPtrsType uPtrs= ssGetInputPortSignalPtrs(S,portindex); 
                    strncpy(token,formatstring+fi,k+1);
					token[k+1]='\0';
  			        switch (ssGetInputPortDataType(S,portindex)) { 
			          case SS_INT8: {
                        const int8_T *data = uPtrs[0];
				        sprintf(tmp,token,(int)*data);
                        break;
			          }
      		          case SS_UINT8: {
                        const uint8_T *data = uPtrs[0];
				        sprintf(tmp,token,(int)*data);
                        break;
                      }
			          case SS_INT16: {
                        const int16_T *data = uPtrs[0];
				        sprintf(tmp,token,(int)*data);
                        break;
			          }
      		          case SS_UINT16: {
                        const uint16_T *data = uPtrs[0];
				        sprintf(tmp,token,(int)*data);
                        break;
			          }
			          case SS_INT32: {
                        const int32_T *data = uPtrs[0];
				        sprintf(tmp,token,(int)*data);
                        break;
			          }
      		          case SS_UINT32: {
                        const uint32_T *data = uPtrs[0];
				        sprintf(tmp,token,(int)*data);
                        break;
			          }
                      case SS_DOUBLE: {
                        const double *data = uPtrs[0];
				        sprintf(tmp,token,(int)*data);
                        break;
			          }
                      case SS_SINGLE: {
                        const float *data = uPtrs[0];
				        sprintf(tmp,token,(int)*data);
                        break;
			          }
				    }
                    strcat(tmp_send,tmp);
           	        found=1;
					break;
				  }
				  case 'e':
				  case 'E':
				  case 'f':
				  case 'g':
				  case 'G':	{
				    InputPtrsType uPtrs= ssGetInputPortSignalPtrs(S,portindex); 
       	   	        strncpy(token,formatstring+fi,k+1);
					token[k+1]='\0';
  			        switch (ssGetInputPortDataType(S,portindex)) { 
			          case SS_INT8: {
                        const int8_T *data = uPtrs[0];
				        sprintf(tmp,token,(double)*data);
                        break;
			          }
      		          case SS_UINT8: {
                        const uint8_T *data = uPtrs[0];
				        sprintf(tmp,token,(double)*data);
                        break;
                      }
			          case SS_INT16: {
                        const int16_T *data = uPtrs[0];
				        sprintf(tmp,token,(double)*data);
                        break;
			          }
      		          case SS_UINT16: {
                        const uint16_T *data = uPtrs[0];
				        sprintf(tmp,token,(double)*data);
                        break;
			          }
			          case SS_INT32: {
                        const int32_T *data = uPtrs[0];
				        sprintf(tmp,token,(double)*data);
                        break;
			          }
      		          case SS_UINT32: {
                        const uint32_T *data = uPtrs[0];
				        sprintf(tmp,token,(double)*data);
                        break;
			          }
                      case SS_DOUBLE: {
                        const double *data = uPtrs[0];
				        sprintf(tmp,token,(double)*data);
                        break;
			          }
                      case SS_SINGLE: {
                        const float *data = uPtrs[0];
				        sprintf(tmp,token,(double)*data);
                        break;
			          }
				    }
                    strcat(tmp_send,tmp); 
         		    found=1;
					break;
			      }					
				  default: {
				  	k++;
				  	j++;
				  	if (j > (int)strlen(formatstring)-1) {
	              		ssSetErrorStatus(S,"error in format string\n");
						return;
				  	}
				  	break;
				  }
	         	}
	     	}
	      	// set new fi
	      	fi=j+1;
	      	i++;
   		} else {
		  tmp3[0] = formatstring[fi];
          tmp3[1] = '\0';
          strcat(tmp_send,tmp3);
       	  fi++;
   		}
	}
  rl32eSendBlock(port,(void *) tmp_send, strlen(tmp_send));
}
#endif

static void mdlInitializeSizes(SimStruct *S)
{
  int_T i, num_in;

  ssSetNumSFcnParams(S, NUMBER_OF_ARGS);
  if (ssGetNumSFcnParams(S) != ssGetSFcnParamsCount(S)) {
	sprintf(msg,"Wrong number of input arguments passed.\n%d arguments are expected\n",NUMBER_OF_ARGS);
    ssSetErrorStatus(S,msg);
    return;
  }

  num_in = (int_T)mxGetPr(NUM_IN_ARG)[0];

  /* Set-up size information */
  ssSetNumContStates(S, 0);
  ssSetNumDiscStates(S, 0);
  if( !ssSetNumOutputPorts(S, 0) )return;
  
  if( !ssSetNumInputPorts(S, num_in) )return;

  i = 0;
  
  while (i<(int)num_in) {
	ssSetInputPortWidth(S, i, 1);
    ssSetInputPortDirectFeedThrough(S, i, 1);
    ssSetInputPortDataType(S, i, DYNAMICALLY_TYPED);
	i++;
  }

  ssSetSimStateCompliance(S, HAS_NO_SIM_STATE);

  ssSetNumSampleTimes(S,1);
  ssSetNumIWork(S, NO_I_WORKS); 
  ssSetNumRWork(S, NO_R_WORKS);
  ssSetNumPWork(S, NO_P_WORKS);
  ssSetNumModes(         S, 0);
  ssSetNumNonsampledZCs( S, 0);

  for( i = 0 ; i < NUMBER_OF_ARGS; i++ )
  {
    ssSetSFcnParamTunable(S,i,0);  /* None of the parameters are tunable */
  }

  ssSetOptions(S, SS_OPTION_DISALLOW_CONSTANT_SAMPLE_TIME | SS_OPTION_EXCEPTION_FREE_CODE );
}
 
/* Function to initialize sample times */
static void mdlInitializeSampleTimes(SimStruct *S)
{
  ssSetSampleTime(S, 0, mxGetPr(SAMP_TIME_ARG)[0]);
  if (mxGetN((SAMP_TIME_ARG))==1) {
	ssSetOffsetTime(S, 0, 0.0);
  } else {
    ssSetOffsetTime(S, 0, mxGetPr(SAMP_TIME_ARG)[1]);
  }
}
 
#ifdef MATLAB_MEX_FILE

#define MDL_SET_INPUT_PORT_DATA_TYPE
static void mdlSetInputPortDataType(SimStruct *S, int portIndex, DTypeId dType)
{
    switch (dType) {
      case SS_INT8:
      case SS_UINT8:
      case SS_INT16:
      case SS_UINT16:
      case SS_INT32:
      case SS_UINT32:
      case SS_SINGLE:
      case SS_DOUBLE: 
        ssSetInputPortDataType( S, portIndex, dType); 
      break;
      default:
        ssSetErrorStatus(S, "Input signal(s) must be one of the types: int8, uint8, int16, uint16, int32, uint32, double.\n");
  }      
}

#define MDL_SET_OUTPUT_PORT_DATA_TYPE
static void mdlSetOutputPortDataType(SimStruct *S, int portIndex, DTypeId dType)
{
	ssSetOutputPortDataType( S, portIndex, dType);
}
#endif

#define MDL_START  /* Change to #undef to remove function */
#if defined(MDL_START) 
static void mdlStart(SimStruct *S)
{
#ifndef MATLAB_MEX_FILE

  char_T *buffer;

  buffer=(char *)calloc(mxGetN(FORMAT_SEND_ARG)+1,sizeof(char));
  if (buffer==NULL) {
	ssSetErrorStatus(S,"RS232 Send/Rec: could not allocate memory\n");
	return;
  }
  mxGetString(FORMAT_SEND_ARG, buffer, mxGetN(FORMAT_SEND_ARG)+1);
  ssSetPWorkValue(S,0,(void *)buffer);

#endif	
}
#endif

/* Function to compute outputs */
static void mdlOutputs(SimStruct *S, int_T tid)
{
#ifndef MATLAB_MEX_FILE

  int_T port, h, ini, inputindex;
  double delay;
  char *format_send;
  char tempChar;


  port=((int_T)mxGetPr(PORT_ARG)[0])-1;
  if (!rs232ports[port]) {
	ssSetErrorStatus(S,"        RS232 Send/Rec: chosen COM-port not initialized\n");
	return;
  }

  ini = 0;
  inputindex = 0;
  
  format_send=(char *)ssGetPWorkValue(S,0);

  for (h=0;h<mxGetPr(FORMAT_SEND_INFO)[0];h++) {
    if (mxGetPr(FORMAT_SEND_INFO)[h+1]>0) {
	  tempChar = format_send[ini+(int)mxGetPr(FORMAT_SEND_INFO)[h+1]];
      format_send[ini+(int)mxGetPr(FORMAT_SEND_INFO)[h+1]] = '\0';
      build(S, format_send+ini, port,inputindex);
      while (!(rl32eLineStatus(port) & RS232_TX_SHIFT_EMPTY));
	  inputindex = inputindex + datacount(format_send+ini);
      format_send[ini+(int)mxGetPr(FORMAT_SEND_INFO)[h+1]] = tempChar;
	  ini = ini + (int_T)mxGetPr(FORMAT_SEND_INFO)[h+1];
	  delay = mxGetPr(WAIT_ARG)[h];
	  if (delay>0.0) {
	    rl32eWaitDouble(delay);
	  }
	}
  }

#endif
}

/* Function to perform housekeeping at execution termination */
static void mdlTerminate(SimStruct *S)
{
#ifndef MATLAB_MEX_FILE
  char *buffer;

  buffer=(char *)ssGetPWorkValue(S,0);
  if (buffer!=NULL) {
  	free(buffer);
  }
#endif
}

#ifdef	MATLAB_MEX_FILE    /* Is this file being compiled as a MEX-file? */
#include "simulink.c"      /* MEX-file interface mechanism */
#else
#include "cg_sfun.h"       /* Code generation registration function */
#endif
