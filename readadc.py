#!/usr/bin/env python3
# Note: copy USB2DaqsB.dll along with this script.
# Run: python readadc.py

import time
from ctypes import (
    byref, create_string_buffer
)

from M3F20xm import *

class M3F20xmADC:
    def __init__(self):
        self.device_number = None
        self.serial_number = None
        self.version_string = {}
        self.ready = False
        self.init()

    # Define the callback function
    def usb_ready_callback(iDevIndex, iDevStatus):
        print(f"iDevIndex: {iDevIndex}, iDevStatus: {iDevStatus}")
        if iDevStatus == 0x00:
            print("Device unplugged.")
        elif iDevStatus == 0x80:
            print("Device plugged in.")
        return True
    
    def init(self):
        # Convert the Python callback function to a C callback function
        USB_READY_CALLBACK = M3F20xm_USB_READY_CALLBACK(self.usb_ready_callback)

        # Call M3F20xm_SetUSBNotify with the callback function
        result = M3F20xm_SetUSBNotify(True, USB_READY_CALLBACK)
        print(f"M3F20xm_SetUSBNotify return {result}.")

        # wait a while for the device to be ready
        # 0.04 second is at the boundary of success and failure, so we choose 0.1 second
        time.sleep(0.1)  

        device_number = M3F20xm_OpenDevice()
        if device_number == 0xFF:
            print("Failed to open device.")
            self.ready = False
            return
        else:
            print(f"Device opened successfully. Device Number: {device_number}")
            self.ready = True
            self.device_number = device_number

        ver_buffer_size = 256  # >= 50
        ver_buffer = create_string_buffer(ver_buffer_size)

        # loop to query version information
        for i_type in [0, 1, 2]:
            result = M3F20xm_GetVersion(device_number, i_type, ver_buffer)
            # Check the result
            if result:
                q = M3F20xm_GetVersion_query_dict[i_type]
                v = ver_buffer.value.decode('utf-8')
                print(f"Query successful. {q}: "
                    f"{v}")
                self.version_string[q] = v
            else:
                print("Error in querying version.")
                self.ready = False

        # get serial number
        serial_buffer_size = 10
        serial_buffer = create_string_buffer(serial_buffer_size)
        result = M3F20xm_GetSerialNo(device_number, serial_buffer)
        self.serial_number = serial_buffer.value.decode('utf-8')
        if result == 0:
            print("No device found.")
        elif result == 1:
            print("Device not in use.")
        elif result == 2:
            print(f"Device in use. Serial Number: {self.serial_number}")
        else:
            print("Error in querying serial number.")

        # call M3F20xm_Verify
        verify_result = c_ubyte(0)
        result = M3F20xm_Verify(device_number, byref(verify_result))
        print(f"M3F20xm_Verify return {result}. Verify result: {verify_result.value}.")
        self.verify_result = verify_result.value

        # call M3F20xm_ADCGetConfig
        adc_config = ADC_CONFIG()
        result = M3F20xm_ADCGetConfig(device_number, byref(adc_config))
        print(f"M3F20xm_ADCGetConfig return {result}.")
        print("ADC_CONFIG:")
        for field in adc_config._fields_:
            v = getattr(adc_config, field[0])
            print(f"    {field[0]}\t: {v} = ({hex(v)})")
        self.adc_config = adc_config

    def sampling(self):
        # read a ADC sample data
        SampleFrameType = c_ushort * 8
        values = SampleFrameType()
        result = M3F20xm_ADCRead(self.device_number, values)
        print(f"M3F20xm_ADCRead return {result}.")
        print("ADC sample data:")
        for v in values:
            print(f"    {v} = ({hex(v)})")

    def close(self):
        # close device
        result = M3F20xm_CloseDevice(self.device_number)
        print('M3F20xm_CloseDevice return', result)

if __name__ == '__main__':

    adc = M3F20xmADC()
    adc.sampling()
    
    if 0:
        # waiting Ctrl-C in a while loop
        while True:
            time.sleep(10)
            print(time.strftime("%Y-%m-%d %H:%M:%S", time.localtime()))
            pass

    adc.close()
