# Python wrapper for USB2DaqsB.dll

import time
import array
from ctypes import (
    CDLL, Structure, CFUNCTYPE, POINTER,
    c_bool, c_ubyte, c_ushort, c_ulong, c_char_p,
    byref, create_string_buffer
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

# start sampling trigger
# prototype: bool M3F20xm_ADCStart(BYTE byIndex)
M3F20xm_ADCStart = adc_dll.M3F20xm_ADCStart
M3F20xm_ADCStart.argtypes = [c_ubyte]
M3F20xm_ADCStart.restype = c_bool

# stop sampling trigger
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

#########################################################################
# pythonic interface

debug_level = 3

def dbg_print(level, *p, **keys):
    """
    Used for printing error and debugging information.
    Controlled by global (module) debug_level.
    Higher debug_level will show more information.
        debug_level == 0: show nothing.
        debug_level == 1: show only error.
        debug_level == 2: show warning.
        debug_level == 3: show hint.
        debug_level == 4: show message.
        debug_level == 5: most verbose.
    """
    if level > debug_level:
        return
    level_str = {1:'Error', 2:'Warning', 3:'Hint', 4:'Message', 5:'Verbose'}
    print(level_str[level] + ':', *p, **keys)

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
        dbg_print(3, f"iDevIndex: {iDevIndex}, iDevStatus: {iDevStatus}")
        if iDevStatus == 0x00:
            print(3, "Device unplugged.")
        elif iDevStatus == 0x80:
            print(3, "Device plugged in.")
        return True
    
    def init(self):
        # Convert the Python callback function to a C callback function
        USB_READY_CALLBACK = M3F20xm_USB_READY_CALLBACK(self.usb_ready_callback)

        # Call M3F20xm_SetUSBNotify with the callback function
        result = M3F20xm_SetUSBNotify(True, USB_READY_CALLBACK)
        dbg_print(5, f"M3F20xm_SetUSBNotify return {result}.")

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
            dbg_print(1, "Failed to open device. e.g. device not plugged in or been used by other program.")
            self.ready = False
            # raise an OSError exception
            raise OSError("Failed to open device.")
        else:
            dbg_print(3, f"Device opened successfully. Device Number: {device_number}")
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
                dbg_print(5, f"Query successful. {q}: "
                    f"{v}")
                self.version_string[q] = v
            else:
                dbg_print(1, "Error in querying version.")
                self.ready = False

        _, serial_number = self.status()
        self.serial_number = serial_number

        # call M3F20xm_Verify
        verify_result = c_ubyte(0)
        result = M3F20xm_Verify(device_number, byref(verify_result))
        dbg_print(4, f"M3F20xm_Verify return {result}. Verify result: {verify_result.value}.")
        self.verify_result = verify_result.value

        # call M3F20xm_ADCGetConfig
        adc_config = ADC_CONFIG()
        result = M3F20xm_ADCGetConfig(device_number, byref(adc_config))
        dbg_print(5, f"M3F20xm_ADCGetConfig return {result}.")
        dbg_print(5, "ADC_CONFIG:")
        for field in adc_config._fields_:
            v = getattr(adc_config, field[0])
            dbg_print(5, f"    {field[0]}\t: {v} = ({hex(v)})")
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
        dbg_print(5, 'M3F20xm_ReadAllReg return', ret)
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
              f"{conf.byGPIO & 0x0F :04b} (port 4~1)")
        print(f"    GPIO voltage level (0=low, 1=high):"
              f"{(conf.byGPIO >> 4) & 0x0F :04b} (port 4~1)")
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
        print(f"  Input range (4bits):")
        rginfo = {
            0b0000: "±2.5 V single-ended",
            0b0001: "±5   V single-ended",
            0b0010: "±6.25V single-ended",
            0b0011: "±10  V single-ended",
            0b0100: "±12.5V single-ended",
            0b0101: "0~ 5  V single-ended",
            0b0110: "0~10  V single-ended",
            0b0111: "0~12.5V single-ended",
            0b1000: "±5   V differential",
            0b1001: "±10  V differential",
            0b1010: "±12.5V differential",
            0b1011: "±20  V differential",
            0b1100: "(not defined)",
            0b1101: "(not defined)",
            0b1110: "(not defined)",
            0b1111: "(not defined)",
        }
        print(f"    CH1: {reg(0x03)>>0 & 0x0F:04b} ({rginfo[reg(0x03)>>0 & 0x0F]}))")
        print(f"    CH2: {reg(0x03)>>4 & 0x0F:04b} ({rginfo[reg(0x03)>>4 & 0x0F]}))")
        print(f"    CH3: {reg(0x04)>>0 & 0x0F:04b} ({rginfo[reg(0x04)>>0 & 0x0F]}))")
        print(f"    CH4: {reg(0x04)>>4 & 0x0F:04b} ({rginfo[reg(0x04)>>4 & 0x0F]}))")
        print(f"    CH5: {reg(0x05)>>0 & 0x0F:04b} ({rginfo[reg(0x05)>>0 & 0x0F]}))")
        print(f"    CH6: {reg(0x05)>>4 & 0x0F:04b} ({rginfo[reg(0x05)>>4 & 0x0F]}))")
        print(f"    CH7: {reg(0x06)>>0 & 0x0F:04b} ({rginfo[reg(0x06)>>0 & 0x0F]}))")
        print(f"    CH8: {reg(0x06)>>4 & 0x0F:04b} ({rginfo[reg(0x06)>>4 & 0x0F]}))")
        print(f"  BANDWIDTH: {reg(0x07):08b} (CH 8~1)")
        print("  Oversampling:")
        print(f"    OS_PAD  : {reg(0x08)>>4 & 0x0F}")
        print(f"    OS_RATIO: {reg(0x08)>>0 & 0x0F:04b} (x{1<<(reg(0x08)>>0 & 0x0F)})")
        print("  Gain Register to Remove Gain Error Caused by External R_FILTER (0~63):")
        print(f"    CH1: {reg(0x09) & 0x3F}")
        print(f"    CH2: {reg(0x0A) & 0x3F}")
        print(f"    CH3: {reg(0x0B) & 0x3F}")
        print(f"    CH4: {reg(0x0C) & 0x3F}")
        print(f"    CH5: {reg(0x0D) & 0x3F}")
        print(f"    CH6: {reg(0x0E) & 0x3F}")
        print(f"    CH7: {reg(0x0F) & 0x3F}")
        print(f"    CH8: {reg(0x10) & 0x3F}")
        print(f"  Offset Register to Remove External System Offset Errors (0~255):")
        print(f"    CH1: {reg(0x11)}")
        print(f"    CH2: {reg(0x12)}")
        print(f"    CH3: {reg(0x13)}")
        print(f"    CH4: {reg(0x14)}")
        print(f"    CH5: {reg(0x15)}")
        print(f"    CH6: {reg(0x16)}")
        print(f"    CH7: {reg(0x17)}")
        print(f"    CH8: {reg(0x18)}")
        print(f"  Phase delay from 0 µs to 255 µs in steps of 1 µs. (0~255):")
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
        print("  DIAGNOSTIC_MUX")
        print(f"    CH1: {reg(0x28)>>0 & 0x07}")
        print(f"    CH2: {reg(0x28)>>3 & 0x07}")
        print(f"    CH3: {reg(0x29)>>0 & 0x07}")
        print(f"    CH4: {reg(0x29)>>3 & 0x07}")
        print(f"    CH5: {reg(0x2A)>>0 & 0x07}")
        print(f"    CH6: {reg(0x2A)>>3 & 0x07}")
        print(f"    CH7: {reg(0x2B)>>0 & 0x07}")
        print(f"    CH8: {reg(0x2B)>>3 & 0x07}")
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
        dbg_print(5, f"M3F20xm_ADCRead return {result}.")
        dbg_print(4, "ADC sample data:")
        for v in values:
            dbg_print(4, f"    {v} = ({hex(v)})")
        return array.array('H', values)

    def start(self):
        result = M3F20xm_InitFIFO(self.device_number)
        dbg_print(5, 'M3F20xm_InitFIFO return', result)
        result = M3F20xm_ADCStart(self.device_number)
        dbg_print(5, 'M3F20xm_ADCStart return', result)

    def stop(self):
        result = M3F20xm_ADCStop(self.device_number)
        dbg_print(5, 'M3F20xm_ADCStop return', result)

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
        dbg_print(5, 'M3F20xm_ReadFIFO return', result)
        dbg_print(5, f"pdwRealSize: {pdwRealSize.value}")
        # data left
        pwdBuffSize = c_ulong(0)
        result = M3F20xm_GetFIFOLeft(self.device_number, byref(pwdBuffSize))
        dbg_print(5, 'M3F20xm_GetFIFOLeft return', result)
        dbg_print(4, f"Data still in buffer: {pwdBuffSize.value}")
        return array.array('H', lpBuffer[:n_channel * pdwRealSize.value])

    def close(self):
        # close device
        result = M3F20xm_CloseDevice(self.device_number)
        dbg_print(5, 'M3F20xm_CloseDevice return', result)
