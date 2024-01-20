#!/usr/bin/env python3
# Note: copy USB2DaqsB.dll along with this script.
# Run: python readadc.py

import time
from ctypes import (
    byref, create_string_buffer
)
from M3F20xm import *

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
