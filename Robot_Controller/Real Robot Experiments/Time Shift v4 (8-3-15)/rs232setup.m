/* $Revision: 1.4.4.5 $ $Date: 2009/12/05 02:09:57 $ */
/* rs232setup.c - xPC Target, non-inlined S-function driver for setup of RS-232 */
/* Copyright 1996-2009 The MathWorks, Inc.
*/


#define S_FUNCTION_LEVEL 2
#undef  S_FUNCTION_NAME
#define S_FUNCTION_NAME  rs232setup

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
#define NUMBER_OF_ARGS        	(19)
#define PORT_ARG             	ssGetSFcnParam(S,0)
#define BAUDRATE_ARG 	        ssGetSFcnParam(S,1)
#define DATABIT_ARG           	ssGetSFcnParam(S,2)
#define STOPBIT_ARG             ssGetSFcnParam(S,3)
#define PARITY_ARG 	            ssGetSFcnParam(S,4)
#define PROTOCOL_ARG           	ssGetSFcnParam(S,5)
#define SENDBUF_ARG           	ssGetSFcnParam(S,6)
#define RECBUF_ARG           	ssGetSFcnParam(S,7)
#define WAIT_ARG				ssGetSFcnParam(S,8)
#define INIT_STRING				ssGetSFcnParam(S,9)
#define INIT_ACK				ssGetSFcnParam(S,10)
#define INIT_TIMEOUT            ssGetSFcnParam(S,11)
#define TERM_STRING				ssGetSFcnParam(S,12)
#define TERM_ACK				ssGetSFcnParam(S,13)
#define TERM_TIMEOUT            ssGetSFcnParam(S,14)
#define FORMAT_SEND_INFO_INIT	ssGetSFcnParam(S,15)
#define FORMAT_REC_INFO_INIT	ssGetSFcnParam(S,16)
#define FORMAT_SEND_INFO_TERM	ssGetSFcnParam(S,17)
#define FORMAT_REC_INFO_TERM	ssGetSFcnParam(S,18)

#define NO_I_WORKS              	(0)
#define NO_R_WORKS              	(0)

#define RECV_BUF_SIZE               65536

static char_T msg[256];

extern int rs232ports[];
extern int rs232recbufs[];

static void mdlInitializeSizes(SimStruct *S)
{
  int i;

  ssSetNumSFcnParams(S, NUMBER_OF_ARGS);
  if (ssGetNumSFcnParams(S) != ssGetSFcnParamsCount(S)) {
	sprintf(msg,"Wrong number of input arguments passed.\n%d arguments are expected\n",NUMBER_OF_ARGS);
    ssSetErrorStatus(S,msg);
    return;
  }

  ssSetSimStateCompliance(S, HAS_NO_SIM_STATE);
  
  ssSetNumContStates(S, 0);
  ssSetNumDiscStates(S, 0);
  if(!ssSetNumOutputPorts(S, 0)) return;
  if(!ssSetNumInputPorts(S, 0)) return;
  ssSetNumSampleTimes(S, 1);
  ssSetNumIWork(S, NO_I_WORKS); 
  ssSetNumRWork(S, NO_R_WORKS);
  ssSetNumPWork(S, 0);
  ssSetNumModes(S, 0);
  ssSetNumNonsampledZCs(S, 0);

  for( i = 0 ; i < NUMBER_OF_ARGS; i++ )
  {
    ssSetSFcnParamTunable(S,i,0);  /* None of the parameters are tunable */
  }

  ssSetOptions(S, SS_OPTION_DISALLOW_CONSTANT_SAMPLE_TIME | SS_OPTION_EXCEPTION_FREE_CODE );
}

static void mdlInitializeSampleTimes(SimStruct *S)
{
    ssSetSampleTime(S, 0, INHERITED_SAMPLE_TIME);
    ssSetOffsetTime(S, 0, 0.0);

}

#define MDL_START  /* Change to #undef to remove function */
#if defined(MDL_START) 
static void mdlStart(SimStruct *S)
  {

#ifndef MATLAB_MEX_FILE
  uint16_T c;
  char in[128];
  char format_send[128];
  char format_rec[128];
  int h,j,k;
  float delay;
  int baudrates[]={115200,57600,38400,19200,9600,4800,2400,1200,600,300,110};
  int parities[]={0x100,0x200,0x400};
  int baudrate, parity, port;
  double starttime, setupTimeout;
  int lenStr = 0;
  int ini,ini2;
  char tempChar;

  port=((int_T)mxGetPr(PORT_ARG)[0])-1;
  //if (port==PORT) {
  //	printf("        RS232 I/O-Error: choosen COM-port used for host<->target communication\n");
  //	return;
  //}
  baudrate=baudrates[((int_T)mxGetPr(BAUDRATE_ARG)[0])-1];
  parity=parities[((int_T)mxGetPr(PARITY_ARG)[0])-1];

  rl32eInitCOMPort(	port,
				DefaultCOMIOBase[port],
				DefaultCOMIRQ[port],
				baudrate,
				parity,
				(int_T)mxGetPr(STOPBIT_ARG)[0],
				9-((int_T)mxGetPr(DATABIT_ARG)[0]),
				(int_T)mxGetPr(RECBUF_ARG)[0],
				(int_T)mxGetPr(SENDBUF_ARG)[0],
				((int_T)mxGetPr(PROTOCOL_ARG)[0])-1);

  rs232ports[port]=1;

  mxGetString(INIT_STRING, format_send, mxGetN(INIT_STRING)+1);
  mxGetString(INIT_ACK, format_rec, mxGetN(INIT_ACK)+1);
  
  ini = 0;
  ini2 = 0;
  if (strlen(format_send) > 0)
  {
    for (h=0;h<mxGetPr(FORMAT_SEND_INFO_INIT)[0];h++) {
	  if (mxGetPr(FORMAT_SEND_INFO_INIT)[h+1]>0) {
	    tempChar = format_send[ini+(int)mxGetPr(FORMAT_SEND_INFO_INIT)[h+1]];
        format_send[ini+(int)mxGetPr(FORMAT_SEND_INFO_INIT)[h+1]] = '\0';
        rl32eSendBlock(port,format_send , strlen(format_send));
	    while (!(rl32eLineStatus(port) & RS232_TX_SHIFT_EMPTY));
        format_send[ini+(int)mxGetPr(FORMAT_SEND_INFO_INIT)[h+1]] = tempChar;
	    ini = ini + (int)mxGetPr(FORMAT_SEND_INFO_INIT)[h+1];
	    delay = (float)mxGetPr(WAIT_ARG)[h];
	    if (delay>0.0) {
	      rl32eWaitDouble(delay);
	    }  
	  }
      k = 0;
	  if (mxGetPr(FORMAT_REC_INFO_INIT)[h+1] > 0) {
	    tempChar = format_rec[ini2+(int)mxGetPr(FORMAT_REC_INFO_INIT)[h+1]];
        format_rec[ini2+(int)mxGetPr(FORMAT_REC_INFO_INIT)[h+1]] = '\0';
        if (k < RECV_BUF_SIZE) {
	      starttime=rl32eGetTicksDouble();
          setupTimeout = mxGetPr(INIT_TIMEOUT)[h];

	      lenStr = strlen(ini2+format_rec);

		  rl32eWaitDouble(0.01); //wait 

	      while (k<lenStr) {
	        k = rl32eReceiveBufferCount(port);
	        if (rl32eETimeDouble(rl32eGetTicksDouble(),starttime) > setupTimeout) {
		      ssSetErrorStatus(S,"RS232 Setup: Initialization ACK timeout");
		      return;
	        }
	      }
	      //Read excess data out of serial port
	      for (j=0;j<(k-lenStr);j++) {
	        c=rl32eReceiveChar(port);
	      }
	      for (j=0;j<lenStr;j++) {
	        c=rl32eReceiveChar(port);
	        if (( c & 0xff00)!=0) {
	          ssSetErrorStatus(S,"RS232 Setup: Receive error");
	          return;
	        }
	        in[j]=(char)c;
            in[j+1]='\0';
	      }
							  
	      printf("Ack I %s E %s\n",in,ini2+format_rec);
	      if (strcmp(in,ini2+format_rec)!=0) {
            ssSetErrorStatus(S,"RS232 Setup: Initialization ACK failed!");  
          }
        }
        format_rec[ini2+(int)mxGetPr(FORMAT_REC_INFO_INIT)[h+1]] = tempChar;
        ini2 = ini2 + (int)mxGetPr(FORMAT_REC_INFO_INIT)[h+1];
	  }
    }
  }
#endif // Not MATLAB_MEX_FILE  
}
#endif /*  MDL_START */

static void mdlOutputs(SimStruct *S, int_T tid)
{
#ifndef MATLAB_MEX_FILE
#endif
}

static void mdlTerminate(SimStruct *S)
{
#ifndef MATLAB_MEX_FILE
  uint16_T c;
  char in[128];
  char format_send[128];
  char format_rec[128];
  int h,j,k,port;
  double starttime, setupTimeout;
  int lenStr;
  int ini, ini2;
  char tempChar;

  port=((int_T)mxGetPr(PORT_ARG)[0])-1;


  mxGetString(TERM_STRING, format_send, mxGetN(TERM_STRING)+1);
  mxGetString(TERM_ACK, format_rec, mxGetN(TERM_ACK)+1);
  
  ini = 0;
  ini2 = 0;
  
  if (strlen(format_send) > 0)
  {
    for (h=0;h<mxGetPr(FORMAT_SEND_INFO_TERM)[0];h++) {
	  if (mxGetPr(FORMAT_SEND_INFO_TERM)[h+1]>0) {
	    tempChar = format_send[ini+(int)mxGetPr(FORMAT_SEND_INFO_TERM)[h+1]];
        format_send[ini+(int)mxGetPr(FORMAT_SEND_INFO_TERM)[h+1]] = '\0';
        rl32eSendBlock(port,format_send , strlen(format_send));
	    while (!(rl32eLineStatus(port) & RS232_TX_SHIFT_EMPTY));
        format_send[ini+(int)mxGetPr(FORMAT_SEND_INFO_TERM)[h+1]] = tempChar;
	    ini = ini + (int)mxGetPr(FORMAT_SEND_INFO_TERM)[h+1];
  	  }
      k = 0;
	  if (mxGetPr(FORMAT_REC_INFO_TERM)[h+1] > 0) {
	    tempChar = format_rec[ini2+(int)mxGetPr(FORMAT_REC_INFO_TERM)[h+1]];
        format_rec[ini2+(int)mxGetPr(FORMAT_REC_INFO_TERM)[h+1]] = '\0';
        if (k < RECV_BUF_SIZE) {
	      starttime=rl32eGetTicksDouble();
          setupTimeout = mxGetPr(TERM_TIMEOUT)[h];

	      lenStr = strlen(ini2+format_rec);

		  rl32eWaitDouble(0.01); //wait 

	      while (k<lenStr) {
	        k = rl32eReceiveBufferCount(port);
	        if (rl32eETimeDouble(rl32eGetTicksDouble(),starttime) > setupTimeout) {
		      ssSetErrorStatus(S,"RS232 Setup: Termination ACK timeout");
		      return;
	        }
	      }
	      //Read excess data out of serial port
	      for (j=0;j<(k-lenStr);j++) {
	        c=rl32eReceiveChar(port);
	      }
	      for (j=0;j<lenStr;j++) {
	        c=rl32eReceiveChar(port);
	        if (( c & 0xff00)!=0) {
	          ssSetErrorStatus(S,"RS232 Setup: Receive error");
	          return;
	        }
	        in[j]=(char)c;
            in[j+1]='\0';
	      }

          if (strcmp(in,ini2+format_rec)!=0) {
            ssSetErrorStatus(S,"RS232 Setup: Termination ACK failed!");  
          }
        }
        format_rec[ini2+(int)mxGetPr(FORMAT_REC_INFO_TERM)[h+1]] = tempChar;
        ini2 = ini2 + (int)mxGetPr(FORMAT_REC_INFO_TERM)[h+1];
	  }
    }
  }

  rl32eCloseCOMPort(port);
  rs232ports[port]=0;
#endif
}

#ifdef  MATLAB_MEX_FILE    /* Is this file being compiled as a MEX-file? */
#include "simulink.c"      /* MEX-file interface mechanism */
#else
#include "cg_sfun.h"       /* Code generation registration function */
#endif
