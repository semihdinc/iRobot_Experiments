// Declaration of PciDevice, a structure used to pass information about 
// installed CEI boards from the xPC environment to the CEI-x20 API.

//  Copyright 2004-2010 The MathWorks, Inc.

#ifndef __DEVICEINFO_H__
#define __DEVICEINFO_H__
#include <stdio.h>
#define MAX_CEI_BOARD_ID 16

typedef struct PciDeviceStruct {
    unsigned long BaseAddress[6];
    unsigned long VirtAddress[6];
    unsigned long Length[6];
    unsigned short AddressSpaceIndicator[6];
    unsigned short MemoryType[6];
    unsigned short Prefetchable[6];
    unsigned short InterruptLine;
    unsigned short VendorId;
    unsigned short DeviceId;
    unsigned short SubDeviceId;
    unsigned short SubVendorId;
    short          dummy;  // force correct 4 byte alignment for bus
    int            bus;
    int            slot;
} PciDevice;

void ShowPCIDevice( PciDevice *pdev );

int GetPCIDeviceFull( unsigned short vendor_id,
                      unsigned short device_id,
                      unsigned short sub_vendor_id,
                      unsigned short sub_device_id,
                      int bus,
                      int slot,
                      PciDevice *pciDevice );

void ShowPCIDevice( PciDevice *pdev );

#endif // __DEVICEINFO_H__ 
