/* Copyright 2004-2007 The MathWorks, Inc. */

#ifndef __XPCIMPORTS_H__
#define __XPCIMPORTS_H__

#include "xpctarget.h"
#include "deviceinfo.h"

typedef struct PCIDeviceInfoStruct{
    unsigned long BaseAddress[6];
    unsigned short AddressSpaceIndicator[6];
    unsigned short MemoryType[6];
    unsigned short Prefetchable[6];
    unsigned short InterruptLine;
} PCIDeviceInfo;

#ifndef NOT_EXTERN_FN_POINTERS
#  define EXTERN extern
#else
#  define EXTERN 
#endif

// Pentium counter (finetime_xpcimport)

typedef struct {
   unsigned long TimeLow;
   unsigned long TimeHigh;
} xPCFineTime;

EXTERN void (XPCCALLCONV *xpceFTReadTime) (xPCFineTime *T);
EXTERN void (XPCCALLCONV *xpceFTAdd)      (xPCFineTime *Res, xPCFineTime *T1, xPCFineTime *T2);
EXTERN void (XPCCALLCONV *xpceFTSubtract) (xPCFineTime *Res, xPCFineTime *T1, xPCFineTime *T2);
 
// I/O (io_xpcimport)

#define RT_PG_USERREADWRITE  XPC_RT_PG_USERREADWRITE
#define BASE_FREQUENCY   1193000.0

#define rl32eGetDevicePtr xpcReserveMemoryRegion

#define rl32eInpDW(port)    xpcInpDW((uint16_T)(port))
#define rl32eInpW(port)     xpcInpW((uint16_T)(port))
#define rl32eInpB(port)      (unsigned short)xpcInpB((uint16_T)(port))

#define rl32eOutpDW(port, value) xpcOutpDW((uint16_T)(port), (uint32_T)(value) )
#define rl32eOutpW(port, value)  xpcOutpW( (uint16_T)(port), (uint16_T)(value) ) 
#define rl32eOutpB(port, value)  xpcOutpB( (uint16_T)(port), (uint8_T) (value) ) 

// Utility functions (util_xpcimport)
#define xpceIsModelInit  (int)xpcIsModelInit

// Physical memory allocation for DMA buffers
#define xpceAllocPhysicalMemory xpcAllocPhysicalMemory
#define xpceFreePhysicalMemory  xpcFreePhysicalMemory

// Time (time_xpcimport)
double rl32eGetTicksDouble(void);       /* from xpcPCFunctions.c */
#define rl32eWaitDouble(seconds) xpcBusyWait(seconds)
#define rl32eETimeDouble(stop, start)  (((stop) - (start)) / BASE_FREQUENCY)

EXTERN double (XPCCALLCONV * xpceGetTET)           (void);


// PCI (pci_xpcimport)
EXTERN void (XPCCALLCONV * rl32eShowPCIDev)        (void);
EXTERN void (XPCCALLCONV * rl32eShowPCIInfo)       (PCIDeviceInfo pciinfo);
EXTERN int  (XPCCALLCONV * rl32eGetPCIInfo)        (unsigned short vendor_id, unsigned short device_id, PCIDeviceInfo *pciinfo);
EXTERN int  (XPCCALLCONV * rl32eGetPCIInfoAtSlot)  (unsigned short vendor_id, unsigned short device_id, int slot, PCIDeviceInfo *pciinfo);
EXTERN int  (XPCCALLCONV * xpceAssignPCIInterrupt) (unsigned short vendor_id, unsigned short device_id, unsigned short interruptLine);

// PCI (sigpar_xpcimport)

EXTERN double (XPCCALLCONV *xpceGetSignalValue)(int sigNo);
EXTERN void   (XPCCALLCONV *xpceSetParameter)(int parIdx, double *par);
EXTERN void   (XPCCALLCONV *xpceGetParameter)(int parIdx, double *par);
EXTERN void   (XPCCALLCONV *xpceGetParamDims)(int parIdx, double *dimension);

// RS232 (rs232_xpcimport)
/* Definitions required for RS232 functions */
#define RS232_DATA_READY   0x01     /* not an error               */
#define RS232_OVERRUN      0x02     /* error detected by hardware */
#define RS232_PARITY       0x04     /* error detected by hardware */
#define RS232_FRAME        0x08     /* error detected by hardware */
#define RS232_BREAK        0x10     /* not an error               */
#define RS232_TIMEOUT      0x40     /* error detected by software */
#define RS232_BUFFER_FULL  0x80     /* error detected by software */
#define RS232_TXB_EMPTY    0x20     /* not an error               */
#define RS232_TX_SHIFT_EMPTY 0x40   /* dito                       */
#define RS232_HARD_ERROR   (RS232_OVERRUN | RS232_PARITY | RS232_FRAME | RS232_BREAK)

typedef unsigned short COMData;
typedef unsigned char  Byte;

typedef enum { NoProtocol, XOnXOff, RTSCTS, DTRDSR } Protocol;

extern int DefaultCOMIOBase[];
extern int DefaultCOMIRQ[];

EXTERN void (XPCCALLCONV * rl32eInitCOMPort)(int  Port, int  IOBase, int  IRQ, int  Baudrate, int  Parity, int  StopBits, int  WordLength, int  ReceiveBufferSize, int  SendBufferSize, Protocol Prot);
EXTERN void (XPCCALLCONV * rl32eCloseCOMPort)(int Port);
EXTERN void (XPCCALLCONV * rl32eSendChar)(int Port, Byte Data);
EXTERN COMData (XPCCALLCONV * rl32eReceiveChar)(int Port);
EXTERN int (XPCCALLCONV * rl32eReceiveBufferCount)(int Port);
EXTERN Byte (XPCCALLCONV * rl32eLineStatus)(int Port);
EXTERN void (XPCCALLCONV * rl32eSendBlock)(int Port, void * Data, int Length);


// UDP (udp_xpcimport)

EXTERN int  (XPCCALLCONV * xpceUDPOpenSend)        (const char* Address, int localPort, int remotePort, int Broadcast, int BufferSize);
EXTERN int  (XPCCALLCONV * xpceUDPOpenReceive)     (int Port, const char *From, int bufferSize);
EXTERN int  (XPCCALLCONV * xpceUDPSend)            (int udpDesc,
                                                    unsigned char *data,
                                                    int length);
EXTERN int  (XPCCALLCONV * xpceUDPReceive)         (int udpDesc,
                                                    unsigned char *data,
                                                    int length,
                                                    unsigned long *from,
                                                    int *bytesRecd);

EXTERN void (XPCCALLCONV * xpceUDPClose)           (int udpDesc);

// Utility functions (util_xpcimport)

//EXTERN int (XPCCALLCONV *xpceIsModelInit)          (void);

// xPCscope functions
EXTERN int (XPCCALLCONV * rl32eScopeExists)         (int ScopeNo);
EXTERN int (XPCCALLCONV * rl32eDefScope)            (int ScopeNo, int ScopeType);
EXTERN int (XPCCALLCONV * rl32eGetSignalNo)         (const char *signalName);
EXTERN void (XPCCALLCONV * rl32eAddSignal)          (int ScopeNo, int SignalNo);
EXTERN void (XPCCALLCONV * rl32eSetScope)           (int ScopeNo, int action, double value);
EXTERN void (XPCCALLCONV * xpceFSScopeSet)          (int ScopeNo, const char *filename, int mode, unsigned int writeSize,int dynamicFileOnOff, unsigned int maxWriteFileSize);
EXTERN void (XPCCALLCONV * rl32eSetTargetScope)     (int ScopeNo, int action, double value);
EXTERN void (XPCCALLCONV * rl32eSetTargetScopeSigFt)(int ScopeNo, int SigNo, char *formatstr);
EXTERN void (XPCCALLCONV * rl32eRestartAcquisition) (int ScopeNo);
EXTERN void (XPCCALLCONV * xpceScopeAcqOK)          (int ScopeNo, int *scopeAcqOK);

EXTERN void* (XPCCALLCONV *TBCreateSpin)( void );
EXTERN int (XPCCALLCONV *TBDeleteSpin)( void * p );
EXTERN int (XPCCALLCONV *TBTakeSpinFast)( void * p );
EXTERN int (XPCCALLCONV *TBGiveSpinFast)( void * p );

EXTERN void* (XPCCALLCONV * xpceCreateSemaphore)(unsigned int Type, unsigned int InitialValue, const char * Name );
EXTERN void  (XPCCALLCONV *xpceWaitSemaphore)( void * p );
EXTERN void  (XPCCALLCONV *xpceSignalSemaphore)( void * p );
EXTERN void  (XPCCALLCONV *xpceDeleteSemaphore)( void * p );


EXTERN int (XPCCALLCONV *FindUsbVideoDevice)(int,void*);
EXTERN int (XPCCALLCONV *GetVideoConfigs)(void*,int fmt, int  ctype, void* video_config, int max_config);
EXTERN int (XPCCALLCONV *StartVideo)(void*,unsigned int config_id, unsigned int fwidth, unsigned int fheight, unsigned int finterval, int  fformat, int  ftype);
EXTERN int (XPCCALLCONV *ReadFrameBuffer)(void*,void* data, int* width, int* height);
EXTERN void (XPCCALLCONV *StopVideo)(void*);
EXTERN void (XPCCALLCONV *ConvertUImage)(void* datasrc, void* datadst, int width, int height, int  format, long* colortable);





#endif // __XPCIMPORTS_H__

