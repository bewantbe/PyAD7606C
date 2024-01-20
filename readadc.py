#!/usr/bin/env python3
# Note: copy USB2DaqsB.dll along with this script.
# Run: python readadc.py

import time
import array
import numpy as np
from ctypes import (
    byref, create_string_buffer
)

from M3F20xm import *

class M3F20xmADC:
    # Note: This interface give user a non-ctypes way to use the ADC.
    REG_LIST_LENGTH = 47
    SERIAL_NUMBER_LENGTH = 10   # include \0

    def __init__(self):
        self.device_number = None
        self.serial_number = None
        self.version_string = {}
        self.adc_config = None
        self.reg_list = None
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
            # try to close the device first
            M3F20xm_CloseDevice(0)
            time.sleep(0.1)  
            device_number = M3F20xm_OpenDevice()

        if device_number == 0xFF:
            print("Failed to open device. e.g. device not plugged in or been used by other program.")
            self.ready = False
            # raise an OSError exception
            raise OSError("Failed to open device.")
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

        _, serial_number = self.status()
        self.serial_number = serial_number

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

        reg_list = (c_ubyte * self.REG_LIST_LENGTH)()
        M3F20xm_ReadAllReg(device_number, reg_list)
        self.reg_list = reg_list

    def status(self):
        # get serial number
        serial_buffer = create_string_buffer(self.SERIAL_NUMBER_LENGTH)
        result = M3F20xm_GetSerialNo(self.device_number, serial_buffer)
        serial_number = serial_buffer.value.decode('utf-8')
        if result == 0:
            print("No device found.")
        elif result == 1:
            print("Device not in use.")
        elif result == 2:
            print(f"Device in use. ")
        else:
            print("Error in querying serial number.")
        print(f'Serial Number: {serial_number}')
        return result, serial_number

    def config(self, **kwargs):
        # refer to ADC_CONFIG in M3F20xm.py for kwargs
        adc_config = self.adc_config
        if kwargs:
            for k, v in kwargs.items():
                setattr(adc_config, k, v)
            result = M3F20xm_ADCSetConfig(self.device_number, byref(adc_config))
        result = M3F20xm_ADCGetConfig(self.device_number, byref(adc_config))
        return adc_config

    def get_register(self):
        reg_list = self.reg_list
        ret = M3F20xm_ReadAllReg(self.device_number, reg_list)
        print('M3F20xm_ReadAllReg return', ret)
        return reg_list

    def set_register(self, reg_list):
        # reg_list must be a list of 47 unsigned byte intergers
        assert len(reg_list) == self.REG_LIST_LENGTH
        for i, v in enumerate(reg_list):
            self.reg_list[i] = v
        M3F20xm_WriteAllReg(self.device_number, reg_list)
        return self.get_register()

    def show_config(self):
        # show config in user friendly way
        conf = self.adc_config
        print("ADC_CONFIG:")
        print(f"  byADCOptions = {conf.byADCOptions} ({hex(conf.byADCOptions)})")
        print("    Trigger mode:",
            {
                0: 'by GPIO, one sample per event',
                1: 'period, one sample per period',
                2: 'by GPIO then period',
                3: 'by voltage compare then period'
            }[conf.byADCOptions & 0x03])
        print("    Trigger edge:", 
              {
                  0: 'falling',
                  1: 'rising',
                  2: 'both'
              }[(conf.byADCOptions >> 2) & 0x03])
        print("    Trigger compare:",
              {
                  0: 'greater or equal than',
                  1: 'less or equal than'
              }[(conf.byADCOptions >> 4) & 0x01])
        print("    Sample period unit:",
              {
                  0: '1 us',
                  1: '1 ms'
              }[(conf.byADCOptions >> 5) & 0x01])
        print("    Trigger status", 
              {
                  0: 'stopped',
                  1: 'on'
              }[(conf.byADCOptions >> 7) & 0x01])
        print(f"  byGPIO      = {conf.byGPIO} ({hex(conf.byGPIO)})")
        print(f"    GPIO direction (0=output, 1=input):"
              f"{conf.byGPIO & 0x0F : 04b} (port 4~1)")
        print(f"    GPIO voltage level (0=low, 1=high):"
              f"{(conf.byGPIO >> 4) & 0x0F : 04b} (port 4~1)")
        print(f"  byActived   = {conf.byActived:08b} (channel for trigger)")
        print(f"  wTrigVol    = {conf.wTrigVol} (trigger voltage in mV)")
        print(f"  dwPeriod    = {conf.dwPeriod} (sampling period)")
        print(f"  dwCycleCnt  = {conf.dwCycleCnt} (sampling cycles)")
        print(f"  dwMaxCycles = {conf.dwMaxCycles} (max sampling cycles, 0=no limit)")

    def show_reg(self):
        reg = lambda n: self.reg_list[n-1]
        print("Register list:")
        print(f"  RESET_DETECT : {reg(0x01)>>7 & 0x01}")
        print(f"  DIGITAL_ERROR: {reg(0x01)>>6 & 0x01}")
        print(f"  OPEN_DETECTED: {reg(0x01)>>5 & 0x01}")
        print(f"  CONFIG:")
        print(f"    STATUS_HEADER : {reg(0x02)>>6 & 0x01}")
        print(f"    EXT_OS_CLOCK  : {reg(0x02)>>5 & 0x01}")
        print(f"    DOUT_FORMAT   : {reg(0x02)>>3 & 0x03}")
        print(f"    OPERATION_MODE: {reg(0x02)>>0 & 0x03}")
        print(f"  Input range (0~15):")
        print(f"    CH1: {reg(0x03)>>0 & 0x0F}")
        print(f"    CH2: {reg(0x03)>>4 & 0x0F}")
        print(f"    CH3: {reg(0x04)>>0 & 0x0F}")
        print(f"    CH4: {reg(0x04)>>4 & 0x0F}")
        print(f"    CH5: {reg(0x05)>>0 & 0x0F}")
        print(f"    CH6: {reg(0x05)>>4 & 0x0F}")
        print(f"    CH7: {reg(0x06)>>0 & 0x0F}")
        print(f"    CH8: {reg(0x06)>>4 & 0x0F}")
        print(f"  BANDWIDTH: {reg(0x07):08b} (CH 8~1)")
        print("  Oversampling:")
        print(f"    OS_PAD  : {reg(0x08)>>4 & 0x0F}")
        print(f"    OS_RATIO: {reg(0x08)>>0 & 0x0F}")
        print("  Input gain (0~63):")
        print(f"    CH1: {reg(0x09) & 0x3F}")
        print(f"    CH2: {reg(0x0A) & 0x3F}")
        print(f"    CH3: {reg(0x0B) & 0x3F}")
        print(f"    CH4: {reg(0x0C) & 0x3F}")
        print(f"    CH5: {reg(0x0D) & 0x3F}")
        print(f"    CH6: {reg(0x0E) & 0x3F}")
        print(f"    CH7: {reg(0x0F) & 0x3F}")
        print(f"    CH8: {reg(0x10) & 0x3F}")
        print(f"  Input offset (0~255):")
        print(f"    CH1: {reg(0x11)}")
        print(f"    CH2: {reg(0x12)}")
        print(f"    CH3: {reg(0x13)}")
        print(f"    CH4: {reg(0x14)}")
        print(f"    CH5: {reg(0x15)}")
        print(f"    CH6: {reg(0x16)}")
        print(f"    CH7: {reg(0x17)}")
        print(f"    CH8: {reg(0x18)}")
        print(f"  Input phase correction (0~255):")
        print(f"    CH1: {reg(0x19)}")
        print(f"    CH2: {reg(0x1A)}")
        print(f"    CH3: {reg(0x1B)}")
        print(f"    CH4: {reg(0x1C)}")
        print(f"    CH5: {reg(0x1D)}")
        print(f"    CH6: {reg(0x1E)}")
        print(f"    CH7: {reg(0x1F)}")
        print(f"    CH8: {reg(0x20)}")
        print("  Error flags:")
        print(f"    INTERFACE_CHECK_EN    : {reg(0x21)>>7 & 0x01}")
        print(f"    CLK_FS_OS_COUNTER_EN  : {reg(0x21)>>6 & 0x01}")
        print(f"    BUSY_STUCK_HIGH_ERR_EN: {reg(0x21)>>5 & 0x01}")
        print(f"    SPI_READ_ERR_EN       : {reg(0x21)>>4 & 0x01}")
        print(f"    SPI_WRITE_ERR_EN      : {reg(0x21)>>3 & 0x01}")
        print(f"    INT_CRC_ERR_EN        : {reg(0x21)>>2 & 0x01}")
        print(f"    MM_CRC_ERR_EN         : {reg(0x21)>>1 & 0x01}")
        print(f"    ROM_CRC_ERR_EN        : {reg(0x21)>>0 & 0x01}")
        print(f"    BUSY_STUCK_HIGH_ERR   : {reg(0x22)>>5 & 0x01}")
        print(f"    SPI_READ_ERR          : {reg(0x22)>>4 & 0x01}")
        print(f"    SPI_WRITE_ERR         : {reg(0x22)>>3 & 0x01}")
        print(f"    INT_CRC_ERR           : {reg(0x22)>>2 & 0x01}")
        print(f"    MM_CRC_ERR            : {reg(0x22)>>1 & 0x01}")
        print(f"    ROM_CRC_ERR           : {reg(0x22)>>0 & 0x01}")
        print(f"  Channel open detect enable: {reg(0x23):08b} (CH 8~1)")
        print(f"  Channel open detected     : {reg(0x24):08b} (CH 8~1)")
        # ...
        print(f"  OPEN_DETECT_QUEUE: {reg(0x2C)}")
        print(f"  FS_CLK_COUNTER   : {reg(0x2D)}")
        print(f"  OS_CLK_COUNTER   : {reg(0x2E)}")
        print(f"  ID               : {reg(0x2F) >> 4:X}")
        print(f"  SILICON_REVISION : {reg(0x2F) & 0x0F:X}")

    def sampling(self):
        # read a ADC sample data
        SampleFrameType = c_ushort * 8
        values = SampleFrameType()
        result = M3F20xm_ADCRead(self.device_number, values)
        print(f"M3F20xm_ADCRead return {result}.")
        print("ADC sample data:")
        for v in values:
            print(f"    {v} = ({hex(v)})")
        return array.array('H', values)

    def start(self):
        result = M3F20xm_InitFIFO(self.device_number)
        print('M3F20xm_InitFIFO return', result)
        result = M3F20xm_ADCStart(self.device_number)
        print('M3F20xm_ADCStart return', result)

    def stop(self):
        result = M3F20xm_ADCStop(self.device_number)
        print('M3F20xm_ADCStop return', result)

    def read(self):
        # read all data in FIFO
        # Note: lpBuffer: buffer for data
        #       dwBuffSize: requested data length
        #       pdwRealSize: actual data length 
        n_channel = 8
        dwBuffSize = n_channel * 1024
        lpBuffer = (c_ushort * dwBuffSize)()
        pdwRealSize = c_ulong(0)
        result = M3F20xm_ReadFIFO(self.device_number, lpBuffer, dwBuffSize, byref(pdwRealSize))
        print('M3F20xm_ReadFIFO return', result)
        print(f"pdwRealSize: {pdwRealSize.value}")
        # data left
        pwdBuffSize = c_ulong(0)
        result = M3F20xm_GetFIFOLeft(self.device_number, byref(pwdBuffSize))
        print('M3F20xm_GetFIFOLeft return', result)
        print(f"Data still in buffer: {pwdBuffSize.value}")
        return array.array('H', lpBuffer[:n_channel * pdwRealSize.value])

    def close(self):
        # close device
        result = M3F20xm_CloseDevice(self.device_number)
        print('M3F20xm_CloseDevice return', result)

if __name__ == '__main__':

    adc = M3F20xmADC()
    adc.sampling()

    adc.show_config()
    adc.show_reg()

    if 0:
        adc.start()
        for i in range(10):
            print('------- loop', i)
            v = adc.read()
            print('data size', len(v))
            print('data mean', np.mean(v))
            time.sleep(0.1)
        adc.stop()

        print('------- loop ends')
        v = adc.read()
        print('data size', len(v))
        print('data mean', np.mean(v))

    if 0:
        # waiting Ctrl-C in a while loop
        while True:
            time.sleep(10)
            print(time.strftime("%Y-%m-%d %H:%M:%S", time.localtime()))
            pass

    adc.close()
