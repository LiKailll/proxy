/*$$HEADER*/
/******************************************************************************/
/*                                                                            */
/*                    H E A D E R   I N F O R M A T I O N                     */
/*                                                                            */
/******************************************************************************/

// Project Name                   : OpenRISC Debug Proxy
// File Name                      : win_usb_driver_calls.c
// Prepared By                    : jb
// Project Start                  : 2008-10-01

/*$$COPYRIGHT NOTICE*/
/******************************************************************************/
/*                                                                            */
/*                      C O P Y R I G H T   N O T I C E                       */
/*                                                                            */
/******************************************************************************/
/*
  This library is free software; you can redistribute it and/or
  modify it under the terms of the GNU Lesser General Public
  License as published by the Free Software Foundation;
  version 2.1 of the License, a copy of which is available from
  http://www.gnu.org/licenses/old-licenses/lgpl-2.1.txt.

  This library is distributed in the hope that it will be useful,
  but WITHOUT ANY WARRANTY; without even the implied warranty of
  MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the GNU
  Lesser General Public License for more details.

  You should have received a copy of the GNU Lesser General Public
  License along with this library; if not, write to the Free Software
  Foundation, Inc., 51 Franklin Street, Fifth Floor, Boston, MA  02110-1301  USA
*/

/*$$DESCRIPTION*/
/******************************************************************************/
/*                                                                            */
/*                           D E S C R I P T I O N                            */
/*                                                                            */
/******************************************************************************/
//
// Some generically named functions for interfacing to the JTAG driver 
// functions, making the code calling these platform independant. The correct 
// driver calling file (either Cygwin or Linux driver) is included at compile 
// time.
// Also included in this file is the USB-JTAG chip initialisation function.
//


/*$$CHANGE HISTORY*/
/******************************************************************************/
/*                                                                            */
/*                         C H A N G E  H I S T O R Y                         */
/*                                                                            */
/******************************************************************************/

// Date		Version	Description
//------------------------------------------------------------------------
// 081101		First revision           			jb


#include <stdio.h>
#include <assert.h>
#include <ctype.h>
#include <string.h>
#include <stdlib.h>
#include <stdarg.h>
#include <sys/stat.h>
#include <sys/types.h>
#include <windows.h>

#include "win_FTCJTAG.h"

#include "win_FTCJTAG_ptrs.h"	

#include "usb_driver_calls.h"

#include "or_debug_proxy.h"


// Load the pointers and loading functions of the FTCJTAG driver
int getFTDIJTAGFunctions() {

  HINSTANCE hLib;

  /* Load the library */
  hLib = LoadLibrary(TEXT("FTCJTAG.dll"));
  DWORD ldLibErrno = GetLastError();

  if (hLib == NULL) {
    printf("Loading of library file \"FTCJTAG.dll\" failed!\n");
    return -1;
  } else {
    assert(0 == ldLibErrno);
    //printf("Loading of library file \"FTCJTAG.dll\" success.\n");
  }

  /* get the address of the functions */
  jtagGetNumDevices = (jtagGetNumDevicesPtr)GetProcAddress(hLib, "JTAG_GetNumDevices");
  if (jtagGetNumDevices == NULL) {
    printf("GetProcAddress for JTAG_GetNumDevices Failed.\n");
    return -1;
  }

  jtagGetNumHiSpeedDevices = (jtagGetNumHiSpeedDevicesPtr)GetProcAddress(hLib, "JTAG_GetNumHiSpeedDevices");
  if (jtagGetNumHiSpeedDevices == NULL) {
    printf("GetProcAddress for JTAG_GetNumHiSpeedDevices Failed.\n");
    return -1;
  }

  jtagGetDeviceNameLocID = (jtagGetDeviceNameLocIDPtr)GetProcAddress(hLib, "JTAG_GetDeviceNameLocID");
  if (jtagGetDeviceNameLocID == NULL) {
    printf("GetProcAddress for  Failed JTAG_GetDeviceNameLocID.\n");
    return -1;
  }

  jtagGetHiSpeedDeviceNameLocIDChannel = (jtagGetHiSpeedDeviceNameLocIDChannelPtr)GetProcAddress(hLib, "JTAG_GetHiSpeedDeviceNameLocIDChannel");
  if (jtagGetHiSpeedDeviceNameLocIDChannel == NULL) {
    printf("GetProcAddress for  Failed JTAG_GetHiSpeedDeviceNameLocIDChannel.\n");
    return -1;
  }

  jtagOpen = (jtagOpenPtr)GetProcAddress(hLib, "JTAG_Open");
  if (jtagOpen == NULL) {
    printf("GetProcAddress for JTAG_Open  Failed.\n");
    return -1;
  }

  jtagOpenEx = (jtagOpenExPtr)GetProcAddress(hLib, "JTAG_OpenEx");
  if (jtagOpenEx == NULL) {
    printf("GetProcAddress for JTAG_OpenEx  Failed.\n");
    return -1;
  }

  jtagOpenHiSpeedDevice = (jtagOpenHiSpeedDevicePtr)GetProcAddress(hLib, "JTAG_OpenHiSpeedDevice");
  if (jtagOpenHiSpeedDevice == NULL) {
    printf("GetProcAddress for JTAG_OpenHiSpeedDevice  Failed.\n");
    return -1;
  }

  jtagGetHiSpeedDeviceType = (jtagGetHiSpeedDeviceTypePtr)GetProcAddress(hLib, "JTAG_GetHiSpeedDeviceType");
  if (jtagGetHiSpeedDeviceType == NULL) {
    printf("GetProcAddress for JTAG_GetHiSpeedDeviceType  Failed.\n");
    return -1;
  }

  jtagClose = (jtagClosePtr)GetProcAddress(hLib, "JTAG_Close");
  if (jtagClose == NULL) {
    printf("GetProcAddress for JTAG_Close  Failed.\n");
    return -1;
  }

  jtagCloseDevice = (jtagCloseDevicePtr)GetProcAddress(hLib, "JTAG_CloseDevice");
  if (jtagCloseDevice == NULL) {
    printf("GetProcAddress for JTAG_CloseDevice  Failed.\n");
    return -1;
  }

  jtagInitDevice = (jtagInitDevicePtr)GetProcAddress(hLib, "JTAG_InitDevice");
  if (jtagInitDevice == NULL) {
    printf("GetProcAddress for JTAG_InitDevice  Failed.\n");
    return -1;
  }

  jtagTurnOnDivideByFiveClockingHiSpeedDevice = (jtagTurnOnDivideByFiveClockingHiSpeedDevicePtr)GetProcAddress(hLib, "JTAG_TurnOnDivideByFiveClockingHiSpeedDevice");
  if (jtagTurnOnDivideByFiveClockingHiSpeedDevice == NULL) {
    printf("GetProcAddress for JTAG_TurnOnDivideByFiveClockingHiSpeedDevice  Failed.\n");
    return -1;
  }

  jtagTurnOffDivideByFiveClockingHiSpeedDevice = (jtagTurnOffDivideByFiveClockingHiSpeedDevicePtr)GetProcAddress(hLib, "JTAG_TurnOffDivideByFiveClockingHiSpeedDevice");
  if (jtagTurnOffDivideByFiveClockingHiSpeedDevice == NULL) {
    printf("GetProcAddress for JTAG_TurnOffDivideByFiveClockingHiSpeedDevice  Failed.\n");
    return -1;
  }

  jtagTurnOnAdaptiveClockingHiSpeedDevice = (jtagTurnOnAdaptiveClockingHiSpeedDevicePtr)GetProcAddress(hLib, "JTAG_TurnOnAdaptiveClockingHiSpeedDevice");
  if (jtagTurnOnAdaptiveClockingHiSpeedDevice == NULL) {
    printf("GetProcAddress for JTAG_TurnOnAdaptiveClockingHiSpeedDevice  Failed.\n");
    return -1;
  }

  jtagTurnOffAdaptiveClockingHiSpeedDevice = (jtagTurnOffAdaptiveClockingHiSpeedDevicePtr)GetProcAddress(hLib, "JTAG_TurnOffAdaptiveClockingHiSpeedDevice");
  if (jtagTurnOffAdaptiveClockingHiSpeedDevice == NULL) {
    printf("GetProcAddress for JTAG_TurnOffAdaptiveClockingHiSpeedDevice  Failed.\n");
    return -1;
  }

  jtagSetDeviceLatencyTimer = (jtagSetDeviceLatencyTimerPtr)GetProcAddress(hLib, "JTAG_SetDeviceLatencyTimer");
  if (jtagSetDeviceLatencyTimer == NULL) {
    printf("GetProcAddress for JTAG_SetDeviceLatencyTimer  Failed.\n");
    return -1;
  }

  jtagGetDeviceLatencyTimer = (jtagGetDeviceLatencyTimerPtr)GetProcAddress(hLib, "JTAG_GetDeviceLatencyTimer");
  if (jtagGetDeviceLatencyTimer == NULL) {
    printf("GetProcAddress for JTAG_GetDeviceLatencyTimer  Failed.\n");
    return -1;
  }

  jtagGetClock = (jtagGetClockPtr)GetProcAddress(hLib, "JTAG_GetClock");
  if (jtagGetClock == NULL) {
    printf("GetProcAddress for JTAG_GetClock  Failed.\n");
    return -1;
  }

  jtagGetHiSpeedDeviceClock = (jtagGetHiSpeedDeviceClockPtr)GetProcAddress(hLib, "JTAG_GetHiSpeedDeviceClock");
  if (jtagGetHiSpeedDeviceClock == NULL) {
    printf("GetProcAddress for JTAG_GetHiSpeedDeviceClock  Failed.\n");
    return -1;
  }

  jtagSetClock = (jtagSetClockPtr)GetProcAddress(hLib, "JTAG_SetClock");
  if (jtagSetClock == NULL) {
    printf("GetProcAddress for JTAG_SetClock  Failed.\n");
    return -1;
  }

  jtagSetLoopback = (jtagSetLoopbackPtr)GetProcAddress(hLib, "JTAG_SetLoopback");
  if (jtagSetLoopback == NULL) {
    printf("GetProcAddress for JTAG_SetLoopback  Failed.\n");
    return -1;
  }

  jtagSetGPIOs = (jtagSetGPIOsPtr)GetProcAddress(hLib, "JTAG_SetGPIOs");
  if (jtagSetGPIOs == NULL) {
    printf("GetProcAddress for JTAG_SetGPIOs  Failed.\n");
    return -1;
  }

  jtagSetHiSpeedDeviceGPIOs = (jtagSetHiSpeedDeviceGPIOsPtr)GetProcAddress(hLib, "JTAG_SetHiSpeedDeviceGPIOs");
  if (jtagSetHiSpeedDeviceGPIOs == NULL) {
    printf("GetProcAddress for JTAG_SetHiSpeedDeviceGPIOs  Failed.\n");
    return -1;
  }

  jtagGetGPIOs = (jtagGetGPIOsPtr)GetProcAddress(hLib, "JTAG_GetGPIOs");
  if (jtagGetGPIOs == NULL) {
    printf("GetProcAddress for JTAG_GetGPIOs  Failed.\n");
    return -1;
  }

  jtagGetHiSpeedDeviceGPIOs = (jtagGetHiSpeedDeviceGPIOsPtr)GetProcAddress(hLib, "JTAG_GetHiSpeedDeviceGPIOs");
  if (jtagGetHiSpeedDeviceGPIOs == NULL) {
    printf("GetProcAddress for JTAG_GetHiSpeedDeviceGPIOs  Failed.\n");
    return -1;
  }

  jtagWrite = (jtagWritePtr)GetProcAddress(hLib, "JTAG_Write");
  if (jtagWrite == NULL) {
    printf("GetProcAddress for JTAG_Write  Failed.\n");
    return -1;
  }

  jtagRead = (jtagReadPtr)GetProcAddress(hLib, "JTAG_Read");
  if (jtagRead == NULL) {
    printf("GetProcAddress for JTAG_Read  Failed.\n");
    return -1;
  }

  jtagWriteRead = (jtagWriteReadPtr)GetProcAddress(hLib, "JTAG_WriteRead");
  if (jtagWriteRead == NULL) {
    printf("GetProcAddress for JTAG_WriteRead  Failed.\n");
    return -1;
  }

  jtagGenerateClockPulses = (jtagGenerateClockPulsesPtr)GetProcAddress(hLib, "JTAG_GenerateClockPulses");
  if (jtagGenerateClockPulses == NULL) {
    printf("GetProcAddress for JTAG_GenerateClockPulses  Failed.\n");
    return -1;
  }

  jtagGenerateClockPulsesHiSpeedDevice = (jtagGenerateClockPulsesHiSpeedDevicePtr)GetProcAddress(hLib, "JTAG_GenerateClockPulsesHiSpeedDevice");
  if (jtagGenerateClockPulsesHiSpeedDevice == NULL) {
    printf("GetProcAddress for JTAG_GenerateClockPulsesHiSpeedDevice  Failed.\n");
    return -1;
  }

  jtagClearCmdSequence = (jtagClearCmdSequencePtr)GetProcAddress(hLib, "JTAG_ClearCmdSequence");
  if (jtagClearCmdSequence == NULL) {
    printf("GetProcAddress for JTAG_ClearCmdSequence  Failed.\n");
    return -1;
  }

  jtagAddWriteCmd = (jtagAddWriteCmdPtr)GetProcAddress(hLib, "JTAG_AddWriteCmd");
  if (jtagAddWriteCmd == NULL) {
    printf("GetProcAddress for JTAG_AddWriteCmd  Failed.\n");
    return -1;
  }

  jtagAddReadCmd = (jtagAddReadCmdPtr)GetProcAddress(hLib, "JTAG_AddReadCmd");
  if (jtagAddReadCmd == NULL) {
    printf("GetProcAddress for JTAG_AddReadCmd  Failed.\n");
    return -1;
  }

  jtagAddWriteReadCmd = (jtagAddWriteReadCmdPtr)GetProcAddress(hLib, "JTAG_AddWriteReadCmd");
  if (jtagAddWriteReadCmd == NULL) {
    printf("GetProcAddress for JTAG_AddWriteReadCmd  Failed.\n");
    return -1;
  }

  jtagClearDeviceCmdSequence = (jtagClearDeviceCmdSequencePtr)GetProcAddress(hLib, "JTAG_ClearDeviceCmdSequence");
  if (jtagClearDeviceCmdSequence == NULL) {
    printf("GetProcAddress for  Failed.\n JTAG_ClearDeviceCmdSequence");
    return -1;
  }

  jtagAddDeviceWriteReadCmd = (jtagAddDeviceWriteReadCmdPtr)GetProcAddress(hLib, "JTAG_AddDeviceWriteReadCmd");
  if (jtagAddDeviceWriteReadCmd == NULL) {
    printf("GetProcAddress for  Failed.\n JTAG_AddDeviceWriteReadCmd");
    return -1;
  }

  jtagAddDeviceReadCmd = (jtagAddDeviceReadCmdPtr)GetProcAddress(hLib, "JTAG_AddDeviceReadCmd");
  if (jtagAddDeviceReadCmd == NULL) {
    printf("GetProcAddress for  Failed JTAG_AddDeviceReadCmd.\n");
    return -1;
  }

  jtagAddDeviceWriteReadCmd = (jtagAddDeviceWriteReadCmdPtr)GetProcAddress(hLib, "JTAG_AddDeviceWriteReadCmd");
  if (jtagAddDeviceWriteReadCmd == NULL) {
    printf("GetProcAddress for  Failed JTAG_AddDeviceWriteReadCmd.\n");
    return -1;
  }


  jtagExecuteCmdSequence = (jtagExecuteCmdSequencePtr)GetProcAddress(hLib, "JTAG_ExecuteCmdSequence");
  if (jtagExecuteCmdSequence == NULL) {
    printf("GetProcAddress for  Failed JTAG_ExecuteCmdSequence.\n");
    return -1;
  }

  jtagGetDllVersion = (jtagGetDllVersionPtr)GetProcAddress(hLib, "JTAG_GetDllVersion");
  if (jtagGetDllVersion == NULL) {
    printf("GetProcAddress for JTAG_GetDllVersion  Failed.\n");
    return -1;
  }

  jtagGetErrorCodeString = (jtagGetErrorCodeStringPtr)GetProcAddress(hLib, "JTAG_GetErrorCodeString");
  if (jtagGetErrorCodeString == NULL) {
    printf("GetProcAddress for  Failed JTAG_GetErrorCodeString.\n");
    return -1;
  }

  //   = (Ptr) GetProcAddress(hLib, "");  
  //if ( == NULL) {
  //  printf("GetProcAddress for  Failed.\n");
  //  return -1;
  // }

  return 0;

}


// Global USB JTAG device handle
FTC_HANDLE ftHandle;

FTC_STATUS
FT2232_USB_JTAG_WriteDataToExternalDevice(
  //FTC_HANDLE ftHandle, 
  BOOL bInstructionTestData,
  DWORD dwNumBitsToWrite,
  PWriteDataByteBuffer pWriteDataBuffer,
  DWORD dwNumBytesToWrite,
  DWORD dwTapControllerState) {
  // Return the appropritae call to the Windows driver
  return (jtagWrite)(
    ftHandle,
    bInstructionTestData,
    dwNumBitsToWrite,
    pWriteDataBuffer,
    dwNumBytesToWrite,
    dwTapControllerState);

}


// Read data from external device
FTC_STATUS
FT2232_USB_JTAG_ReadDataFromExternalDevice(
  //FTC_HANDLE ftHandle, 
  BOOL bInstructionTestData,
  DWORD dwNumBitsToRead,
  PReadDataByteBuffer pReadDataBuffer,
  LPDWORD lpdwNumBytesReturned,
  DWORD dwTapControllerState)

{
  return (jtagRead)(
    ftHandle,
    bInstructionTestData,
    dwNumBitsToRead,
    pReadDataBuffer,
    lpdwNumBytesReturned,
    dwTapControllerState);

}






// Write Read
FTC_STATUS
FT2232_USB_JTAG_WriteReadDataToFromExternalDevice(
  //FTC_HANDLE ftHandle, 
  BOOL bInstructionTestData,
  DWORD dwNumBitsToWriteRead,
  PWriteDataByteBuffer pWriteDataBuffer,
  DWORD dwNumBytesToWrite,
  PReadDataByteBuffer pReadDataBuffer,
  LPDWORD lpdwNumBytesReturned,
  DWORD dwTapControllerState) {

  return (jtagWriteRead)(
    ftHandle,
    bInstructionTestData,
    dwNumBitsToWriteRead,
    pWriteDataBuffer,
    dwNumBytesToWrite,
    pReadDataBuffer,
    lpdwNumBytesReturned,
    dwTapControllerState);

}



// Close device
FTC_STATUS
FT2232_USB_JTAG_CloseDevice() {
  return (jtagClose)(ftHandle);
}

void
reinit_usb_jtag(void) {
  FTC_STATUS Status = FTC_SUCCESS;
  if (DEBUG_USB_DRVR_FUNCS) printf("win_usb_driver_calls: reset_usb_jtag() - ");
  Status = (jtagInitDevice)(ftHandle, 0);
  if (DEBUG_USB_DRVR_FUNCS) printf("return Status: %d\n", (int)Status);
}

// Set clock frequency
// Frequency = 12Mhz/((1+divisor)*2)
//
int g_usb_jtag_clk_divider = 3;

int init_usb_jtag() {
  FTC_STATUS Status = FTC_SUCCESS;
  DWORD dwNumDevices = 0;
  DWORD dwNumHiSpeedDevices = 0;
  char szDeviceName[100];
  char szChannel[5];
  DWORD dwLocationID = 0;
  DWORD dwHiSpeedDeviceType = 0;
  DWORD dwHiSpeedDeviceIndex = 0;
  char szDeviceDetails[150];
  byte timerValue = 0;
  // Made global: FTC_HANDLE ftHandle;
  DWORD dwClockFrequencyHz = 0;
  FTC_INPUT_OUTPUT_PINS LowInputOutputPinsData;
  FTH_INPUT_OUTPUT_PINS HighInputOutputPinsData;
  FTC_LOW_HIGH_PINS LowPinsInputData;
  FTH_LOW_HIGH_PINS HighPinsInputData;
  FTC_CLOSE_FINAL_STATE_PINS CloseFinalStatePinsData;
  BOOL bPerformCommandSequence = false;
  WriteDataByteBuffer WriteDataBuffer;
  ReadDataByteBuffer ReadDataBuffer;
  ReadCmdSequenceDataByteBuffer ReadCmdSequenceDataBuffer;
  DWORD dwNumBytesReturned = 0;
  DWORD dwLoopCntr = 0;
  char szDllVersion[10];
  char szStatusErrorMessage[100];
  char szErrorMessage[150];
  DWORD dwCharCntr = 0;
  int MsgBoxKeyPressed = 0;

  Status = (jtagGetDllVersion)(szDllVersion, 10);
  if (DEBUG_USB_DRVR_FUNCS)
    printf("JTAG_GetDLLVersion returned Status: 0x%x and version %s\n",
      Status, szDllVersion);

  Status = (jtagGetNumHiSpeedDevices)(&dwNumHiSpeedDevices);
  if (DEBUG_USB_DRVR_FUNCS)
    printf("JTAG_GetNumHiSpeedDevices returned Status: 0x%x and %d devices\n",
      Status, dwNumHiSpeedDevices);

  if (dwNumHiSpeedDevices == 0) {
    printf("Error: USB debug cable not detected\nPlease ensure the device is attached and correctly installed\n\n");
    exit(-1);
  }

  if ((Status == FTC_SUCCESS) && (dwNumHiSpeedDevices > 0)) {
    /* To fit for current interface board FT2232HL Board V2
     * FT2232H has 2 independent channels with target machine, channel A and channel B
     * FT2232HL Board V2 connect channel B with its JTAG socket
     */
    do {
      Status = jtagGetHiSpeedDeviceNameLocIDChannel(dwHiSpeedDeviceIndex, szDeviceName, 100, &dwLocationID, szChannel, 5, &dwHiSpeedDeviceType);
      dwHiSpeedDeviceIndex = dwHiSpeedDeviceIndex + 1;
    } while ((Status == FTC_SUCCESS) && (dwHiSpeedDeviceIndex < dwNumHiSpeedDevices) && (strcmp(szChannel, g_jtag_channel) != 0));
    printf("JTAG_GetHiSpeedDeviceNameLocIDChannel returned:\n          Status: 0x%x, devices name: %s, channel %s\n\n",
      Status, szDeviceName, szChannel);

    if (Status == FTC_SUCCESS) {
      if (strcmp(szChannel, g_jtag_channel) != 0)
        Status = FTC_DEVICE_IN_USE;
    }

    if (Status == FTC_SUCCESS) {
      Status = jtagOpenHiSpeedDevice(szDeviceName, dwLocationID, szChannel, &ftHandle);

      if (Status == FTC_SUCCESS) {
        Status = jtagGetHiSpeedDeviceType(ftHandle, &dwHiSpeedDeviceType);

        if (Status == FTC_SUCCESS) {
          strcpy(szDeviceDetails, "Type = ");

          if (dwHiSpeedDeviceType == FT4232H_DEVICE_TYPE)
            strcat(szDeviceDetails, "FT4232H");
          else if (dwHiSpeedDeviceType == FT2232H_DEVICE_TYPE)
            strcat(szDeviceDetails, "FT2232H");

          strcat(szDeviceDetails, ", Name = ");
          strcat(szDeviceDetails, szDeviceName);
          printf("Hi Speed Device opened: %s\n\n", szDeviceDetails);
        }
      }
    }
  }

  if ((Status == FTC_SUCCESS) && ((ftHandle != 0))) {
    Status = jtagInitDevice(ftHandle, 0);

    if (Status == FTC_SUCCESS) {
      if ((Status = jtagGetHiSpeedDeviceClock(g_usb_jtag_clk_divider, &dwClockFrequencyHz)) == FTC_SUCCESS) {
        Status = jtagSetClock(ftHandle, g_usb_jtag_clk_divider, &dwClockFrequencyHz);
        //printf("USB_JTAG_CLK_DIVIDER = %d, dwClockFrequencyHz = %d Hz\n\n", (int)g_usb_jtag_clk_divider, dwClockFrequencyHz);
        printf("JTAG frequency = %.1f MHz\n\n", dwClockFrequencyHz/5e6);
      }
    }

    if (Status == FTC_SUCCESS) {
      Status = jtagSetLoopback(ftHandle, false);
    }

    if (Status == FTC_SUCCESS) {
      LowInputOutputPinsData.bPin1InputOutputState = true;
      LowInputOutputPinsData.bPin2InputOutputState = true;
      LowInputOutputPinsData.bPin3InputOutputState = true;
      LowInputOutputPinsData.bPin4InputOutputState = true;
      LowInputOutputPinsData.bPin1LowHighState = true;
      LowInputOutputPinsData.bPin2LowHighState = true;
      LowInputOutputPinsData.bPin3LowHighState = true;
      LowInputOutputPinsData.bPin4LowHighState = true;

      if (dwHiSpeedDeviceType == FT2232H_DEVICE_TYPE) {
        HighInputOutputPinsData.bPin1InputOutputState = true;
        HighInputOutputPinsData.bPin2InputOutputState = true;
        HighInputOutputPinsData.bPin3InputOutputState = true;
        HighInputOutputPinsData.bPin4InputOutputState = true;
        HighInputOutputPinsData.bPin5InputOutputState = true;
        HighInputOutputPinsData.bPin6InputOutputState = true;
        HighInputOutputPinsData.bPin7InputOutputState = true;
        HighInputOutputPinsData.bPin8InputOutputState = true;
        HighInputOutputPinsData.bPin1LowHighState = true;
        HighInputOutputPinsData.bPin2LowHighState = false;
        HighInputOutputPinsData.bPin3LowHighState = false;
        HighInputOutputPinsData.bPin4LowHighState = true;
        HighInputOutputPinsData.bPin5LowHighState = true;
        HighInputOutputPinsData.bPin6LowHighState = false;
        HighInputOutputPinsData.bPin7LowHighState = false;
        HighInputOutputPinsData.bPin8LowHighState = true;
        Status = jtagSetHiSpeedDeviceGPIOs(ftHandle, true, &LowInputOutputPinsData, true, &HighInputOutputPinsData);
      } else {
        Status = jtagSetHiSpeedDeviceGPIOs(ftHandle, true, &LowInputOutputPinsData, false, &HighInputOutputPinsData);
      }

#if 0
      if (Status == FTC_SUCCESS) {
        if (dwHiSpeedDeviceType == FT2232H_DEVICE_TYPE)
          Status = jtagGetHiSpeedDeviceGPIOs(ftHandle, true, &LowPinsInputData, true, &HighPinsInputData);
        else
          Status = jtagGetHiSpeedDeviceGPIOs(ftHandle, true, &LowPinsInputData, false, &HighPinsInputData);
      }
#endif
    }
  }

  if (Status == FTC_SUCCESS) {
    return 0;
  } else {
    return 1;
  }
}
