#!/usr/bin/env python3
# Note: copy USB2DaqsB.dll along with this script.
# Run: python readadc.py

import time
from ctypes import (
    CDLL, Structure, byref, CFUNCTYPE, POINTER,
    c_bool, c_ubyte, c_ushort, c_ulong, c_char_p,
    create_string_buffer
)

# Load the USB2DaqsB.dll library
usbadc_dll_path = './USB2DaqsB.dll'
adc_dll = CDLL(usbadc_dll_path)

## define C function prototype

# note for basic data type (WinDef.h)
# Ref. https://learn.microsoft.com/en-us/openspecs/windows_protocols/ms-dtyp/d7edc080-e499-4219-a837-1bc40b64bb04
# typedef unsigned char BYTE;
# typedef unsigned long DWORD, *PDWORD, *LPDWORD;

# ADC_CONFIG structure
# prototype:
# typedef struct 
# {
#    BYTE byADCOptions;  // bit7: 0 - trigger stop, 1 - trigger start
#                        // bit6: reserved
#                        // bit5: sampling unit, 0 - 1us, 1 - 1ms
#                        // bit4: compare trigger setting, 0 - >=, 1 - <=
#                        // bit3~2: trigger event:
#                        //             00 - falling edge
#                        //             01 - rising edge
#                        //             10 - rising and falling edge
#                        // bit1~0: trigger mode:
#                        //             00 - by GPIO, one sample per event
#                        //             01 - period, one sample per period
#                        //             10 - by GPIO then period
#                        //             11 - by voltage compare then period
#    BYTE byGPIO;        // GPIO direction and level
#                        // bit7~4: GPIO4~1 level, 1 - high, 0 - low
#                        // bit3~0: GPIO4~1 direction, 1 - input, 0 - output
#    BYTE byActived;     // channel for compare trigger
#    BYTE byReserved;
#    WORD wTrigVol;      // trigger gating voltage
#    WORD wReserved;
#    DWORD dwPeriod;     // sampling period (interval)
#    DWORD dwCycleCnt;   // counter for sampling cycles
#    DWORD dwMaxCycles;  // max sampling cycles, 0 for no limit
#    DWORD dwReserved;
# } ADC_CONFIG;
# ctypes equivalent:
class ADC_CONFIG(Structure):
    _fields_ = [
        ("byADCOptions", c_ubyte),
        ("byGPIO", c_ubyte),
        ("byActived", c_ubyte),
        ("byReserved", c_ubyte),
        ("wTrigVol", c_ushort),
        ("wReserved", c_ushort),
        ("dwPeriod", c_ulong),
        ("dwCycleCnt", c_ulong),
        ("dwMaxCycles", c_ulong),
        ("dwReserved", c_ulong)
    ]

# Set callback function for USB plug-in/out notification
# prototype: bool M3F20xm_SetUSBNotify(bool bLog, USB_DLL_CALLBACK pUSB_CallBack)
M3F20xm_SetUSBNotify = adc_dll.M3F20xm_SetUSBNotify
# prototype: bool function(BYTE iDevIndex, DWORD iDevStatus)
M3F20xm_USB_READY_CALLBACK = CFUNCTYPE(c_bool, c_ubyte, c_ulong)
M3F20xm_SetUSBNotify.argtypes = [c_bool, M3F20xm_USB_READY_CALLBACK]
M3F20xm_SetUSBNotify.restype = c_bool

# prototype: BYTE M3F20xm_OpenDevice(void)
# Note: return 0xFF if failed to open device
M3F20xm_OpenDevice = adc_dll.M3F20xm_OpenDevice
M3F20xm_OpenDevice.restype = c_ubyte  # Set the return type to BYTE

# Open device by serial number
# prototype: BYTE M3F20xm_OpenDeviceByNumber(char* pSerialString)
# Note: return 0 if no device found, 1 if device not in use, 2 if device in use
M3F20xm_OpenDeviceByNumber = adc_dll.M3F20xm_OpenDeviceByNumber
M3F20xm_OpenDeviceByNumber.argtypes = [c_char_p]
M3F20xm_OpenDeviceByNumber.restype = c_ubyte

# prototype: bool M3F20xm_GetVersion(BYTE byIndex, BYTE byType, char* lpBuffer)
# byType: 0x00: lib version, 0x01: driver version, 0x02: firmware version
M3F20xm_GetVersion = adc_dll.M3F20xm_GetVersion
M3F20xm_GetVersion.argtypes = [c_ubyte, c_ubyte, c_char_p]
M3F20xm_GetVersion.restype = c_bool
M3F20xm_GetVersion_query_dict = {
    0x00: "lib version",
    0x01: "driver version",
    0x02: "firmware version"}

# prototype: bool M3F20xm_CloseDevice(BYTE byIndex)
M3F20xm_CloseDevice = adc_dll.M3F20xm_CloseDevice
M3F20xm_CloseDevice.argtypes = [c_ubyte]
M3F20xm_CloseDevice.restype = c_bool

# prototype: bool M3F20xm_Verify(BYTE byIndex, BYTE* pResult)
M3F20xm_Verify = adc_dll.M3F20xm_Verify
M3F20xm_Verify.argtypes = [c_ubyte, POINTER(c_ubyte)]
M3F20xm_Verify.restype = c_bool

# prototype: BYTE M3F20xm_GetSerialNo(BYTE byIndex, char* lpBuff)
# Note: byIndex is the Usb2ish device number
#       lpBuff should have more than 10 bytes
# return: 0: no device found, 1: device not in use, 2: device in use
M3F20xm_GetSerialNo = adc_dll.M3F20xm_GetSerialNo
M3F20xm_GetSerialNo.argtypes = [c_ubyte, c_char_p]
M3F20xm_GetSerialNo.restype = c_ubyte

# prototype: bool M3F20xm_ADCGetConfig(BYTE byIndex, ADC_CONFIG* psConfig)
M3F20xm_ADCGetConfig = adc_dll.M3F20xm_ADCGetConfig
M3F20xm_ADCGetConfig.argtypes = [c_ubyte, POINTER(ADC_CONFIG)]
M3F20xm_ADCGetConfig.restype = c_bool

# prototype: bool M3F20xm_ADCSetConfig(BYTE byIndex, ADC_CONFIG* psConfig)
M3F20xm_ADCSetConfig = adc_dll.M3F20xm_ADCSetConfig
M3F20xm_ADCSetConfig.argtypes = [c_ubyte, POINTER(ADC_CONFIG)]
M3F20xm_ADCSetConfig.restype = c_bool

# Read ADC value in one turn.
# prototype: bool M3F20xm_ADCRead(BYTE byIndex, WORD* pwValue)
# Note: pwValue buffer for data, must be 8 bytes long
M3F20xm_ADCRead = adc_dll.M3F20xm_ADCRead
M3F20xm_ADCRead.argtypes = [c_ubyte, POINTER(c_ushort)]
M3F20xm_ADCRead.restype = c_bool

# prototype: bool M3F20xm_ADCStart(BYTE byIndex)
M3F20xm_ADCStart = adc_dll.M3F20xm_ADCStart
M3F20xm_ADCStart.argtypes = [c_ubyte]
M3F20xm_ADCStart.restype = c_bool

# prototype: bool M3F20xm_ADCStop(BYTE byIndex)
M3F20xm_ADCStop = adc_dll.M3F20xm_ADCStop
M3F20xm_ADCStop.argtypes = [c_ubyte]
M3F20xm_ADCStop.restype = c_bool

# Clean and initialize internal FIFO buffer
# prototype: bool M3F20xm_InitFIFO(BYTE byIndex)
M3F20xm_InitFIFO = adc_dll.M3F20xm_InitFIFO
M3F20xm_InitFIFO.argtypes = [c_ubyte]
M3F20xm_InitFIFO.restype = c_bool

# Read internal FIFO buffer
# prototype: bool M3F20xm_ReadFIFO(BYTE byIndex, BYTE* lpBuffer, DWORD dwBuffSize, DWORD* pdwRealSize)
# Note: lpBuffer: buffer for data
#       dwBuffSize: requested data length
#       pdwRealSize: actual data length 
M3F20xm_ReadFIFO = adc_dll.M3F20xm_ReadFIFO
M3F20xm_ReadFIFO.argtypes = [c_ubyte, POINTER(c_ushort), c_ulong, POINTER(c_ulong)]
M3F20xm_ReadFIFO.restype = c_bool

# Get unread FIFO length
# prototype: bool M3F20xm_GetFIFOLeft(BYTE byIndex, DWORD* pdwBuffsize)
# Note: pdwBuffsize: unread data length
M3F20xm_GetFIFOLeft = adc_dll.M3F20xm_GetFIFOLeft
M3F20xm_GetFIFOLeft.argtypes = [c_ubyte, POINTER(c_ulong)]
M3F20xm_GetFIFOLeft.restype = c_bool

# Read all register values in the device
# prototype: bool M3F20xm_ReadAllReg(BYTE byIndex, BYTE* pbyValue)
# Note: pbyValue: point the the register values, must be 47 bytes long, 
#       corresponding to registers 0x01~0x2F
M3F20xm_ReadAllReg = adc_dll.M3F20xm_ReadAllReg
M3F20xm_ReadAllReg.argtypes = [c_ubyte, POINTER(c_ubyte)]
M3F20xm_ReadAllReg.restype = c_bool

# Update all register values in the device
# prototype: bool M3F20xm_WriteAllReg(BYTE byIndex, BYTE* pbyValue)
# Note: pbyValue: point the the register values, must be 47 bytes long,
#       corresponding to registers 0x01~0x2F
M3F20xm_WriteAllReg = adc_dll.M3F20xm_WriteAllReg
M3F20xm_WriteAllReg.argtypes = [c_ubyte, POINTER(c_ubyte)]
M3F20xm_WriteAllReg.restype = c_bool

if __name__ == '__main__':

    # Define the callback function
    def usb_ready_callback(iDevIndex, iDevStatus):
        print(f"iDevIndex: {iDevIndex}, iDevStatus: {iDevStatus}")
        if iDevStatus == 0x00:
            print("Device unplugged.")
        elif iDevStatus == 0x80:
            print("Device plugged in.")
        return True

    # Convert the Python callback function to a C callback function
    USB_READY_CALLBACK = M3F20xm_USB_READY_CALLBACK(usb_ready_callback)

    # Call M3F20xm_SetUSBNotify with the callback function
    result = M3F20xm_SetUSBNotify(True, USB_READY_CALLBACK)
    print(f"M3F20xm_SetUSBNotify return {result}.")

    # wait a while for the device to be ready
    # 0.04 second is at the boundary of success and failure, so we choose 0.1 second
    time.sleep(0.1)  

    device_number = M3F20xm_OpenDevice()
    if device_number == 0xFF:
        print("Failed to open device.")
        exit(0)
    else:
        print(f"Device opened successfully. Device Number: {device_number}")

    ver_buffer_size = 256  # >= 50
    ver_buffer = create_string_buffer(ver_buffer_size)

    #time.sleep(0.1)

    # loop to query version information
    for i_type in [0, 1, 2]:
        result = M3F20xm_GetVersion(device_number, i_type, ver_buffer)
        # Check the result
        if result:
            print(f"Query successful. {M3F20xm_GetVersion_query_dict[i_type]}: "
                f"{ver_buffer.value.decode('utf-8')}")
        else:
            print("Error in querying version.")

    # get serial number
    serial_buffer_size = 10
    serial_buffer = create_string_buffer(serial_buffer_size)
    result = M3F20xm_GetSerialNo(device_number, serial_buffer)
    if result == 0:
        print("No device found.")
    elif result == 1:
        print("Device not in use.")
    elif result == 2:
        print(f"Device in use. Serial Number: {serial_buffer.value.decode('utf-8')}")
    else:
        print("Error in querying serial number.")

    # call M3F20xm_Verify
    verify_result = c_ubyte(0)
    result = M3F20xm_Verify(device_number, byref(verify_result))
    print(f"M3F20xm_Verify return {result}. Verify result: {verify_result.value}.")

    # call M3F20xm_ADCGetConfig
    adc_config = ADC_CONFIG()
    result = M3F20xm_ADCGetConfig(device_number, byref(adc_config))
    print(f"M3F20xm_ADCGetConfig return {result}.")
    print("ADC_CONFIG:")
    for field in adc_config._fields_:
        v = getattr(adc_config, field[0])
        print(f"    {field[0]}\t: {v} = ({hex(v)})")
    
    # read a ADC sample data
    SampleFrameType = c_ushort * 8
    values = SampleFrameType()
    result = M3F20xm_ADCRead(device_number, values)
    print(f"M3F20xm_ADCRead return {result}.")
    print("ADC sample data:")
    for v in values:
        print(f"    {v} = ({hex(v)})")
    
    if 0:
        # waiting Ctrl-C in a while loop
        while True:
            time.sleep(10)
            print(time.strftime("%Y-%m-%d %H:%M:%S", time.localtime()))
            pass

    # close device
    result = M3F20xm_CloseDevice(device_number)
    print('M3F20xm_CloseDevice return', result)
