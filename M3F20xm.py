# Python wrapper for USB2DaqsB.dll

import logging
import math
import time
import array
from ctypes import (
    CDLL, Structure, CFUNCTYPE, POINTER,
    c_bool, c_ubyte, c_ushort, c_short, c_int, c_ulong, c_char_p,
    byref, create_string_buffer, cast
)

import os

_cwd_ = os.path.dirname(os.path.abspath(__file__))

# Load the USB2DaqsB.dll library
usbadc_dll_path = os.path.join(_cwd_, './USB2DaqsB.dll')
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
#    WORD wTrigVol;      // trigger gating voltage (mV)
#    WORD wPeriod;       // sampling period (interval)
#    WORD wPreNum;       // pre-sampling number after trigger
#    WORD wReserved;     // not used
#    DWORD dwCycleCnt;   // counter for sampling cycles
#    DWORD dwMaxCycles;  // max sampling cycles, 0 for no limit
#    DWORD dwReserved;
# } ADC_CONFIG;
# ctypes equivalent:
class ADC_CONFIG(Structure):
    _fields_ = [
        ("byADCOptions", c_ubyte),    # default: 0x11
        ("byGPIO"      , c_ubyte),    # default: 0x0F
        ("byActived"   , c_ubyte),    # default: 0
        ("byReserved"  , c_ubyte),    # default: 0
        ("wTrigVol"    , c_ushort),   # default: 0
        ("wPeriod"     , c_ushort),   # default: 10
        ("wPreNum"     , c_ushort),   # default: 0
        ("wReserved"   , c_ushort),   # default: 0
        ("dwCycleCnt"  , c_ulong),    # default: 0
        ("dwMaxCycles" , c_ulong),    # default: 6000000
        ("dwReserved"  , c_ulong)     # default: 0
    ]

# Set callback function for USB plug-in/out notification
# prototype: bool M3F20xm_SetUSBNotify(bool bLog, USB_DLL_CALLBACK pUSB_CallBack)
M3F20xm_SetUSBNotify = adc_dll.M3F20xm_SetUSBNotify
# prototype: bool function(BYTE iDevIndex, DWORD iDevStatus, ???)
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
# Note: return ture if successful, false if failed
M3F20xm_GetVersion = adc_dll.M3F20xm_GetVersion
M3F20xm_GetVersion.argtypes = [c_ubyte, c_ubyte, c_char_p]
M3F20xm_GetVersion.restype = c_bool
M3F20xm_GetVersion_query_dict = {
    0x00: "lib version",
    0x01: "driver version",
    0x02: "firmware version"}

# prototype: bool M3F20xm_CloseDevice(BYTE byIndex)
# Note: return ture if successful, false if failed
M3F20xm_CloseDevice = adc_dll.M3F20xm_CloseDevice
M3F20xm_CloseDevice.argtypes = [c_ubyte]
M3F20xm_CloseDevice.restype = c_bool

# prototype: bool M3F20xm_Verify(BYTE byIndex, BYTE* pResult)
# Note: return ture if successful, false if failed
M3F20xm_Verify = adc_dll.M3F20xm_Verify
# old
#M3F20xm_Verify.argtypes = [c_ubyte, POINTER(c_ubyte)]
#M3F20xm_Verify.restype = c_bool
# new 2024
M3F20xm_Verify.argtypes = [c_ubyte]
M3F20xm_Verify.restype = c_bool

# prototype: BYTE M3F20xm_GetSerialNo(BYTE byIndex, char* lpBuff)
# Note: byIndex is the Usb2ish device number
#       lpBuff should have more than 10 bytes
# return: 0: no device found, 1: device not in use, 2: device in use
M3F20xm_GetSerialNo = adc_dll.M3F20xm_GetSerialNo
M3F20xm_GetSerialNo.argtypes = [c_ubyte, c_char_p]
M3F20xm_GetSerialNo.restype = c_ubyte

# prototype: bool M3F20xm_ADCGetConfig(BYTE byIndex, ADC_CONFIG* psConfig)
# Note: return ture if successful, false if failed
M3F20xm_ADCGetConfig = adc_dll.M3F20xm_ADCGetConfig
M3F20xm_ADCGetConfig.argtypes = [c_ubyte, POINTER(ADC_CONFIG)]
M3F20xm_ADCGetConfig.restype = c_bool

# prototype: bool M3F20xm_ADCSetConfig(BYTE byIndex, ADC_CONFIG* psConfig)
# Note: return ture if successful, false if failed
M3F20xm_ADCSetConfig = adc_dll.M3F20xm_ADCSetConfig
M3F20xm_ADCSetConfig.argtypes = [c_ubyte, POINTER(ADC_CONFIG)]
M3F20xm_ADCSetConfig.restype = c_bool

# Read ADC value in one turn.
# prototype: bool M3F20xm_ADCRead(BYTE byIndex, WORD* pwValue)
# Note: pwValue buffer for data, must be 8 words long
# Note: return ture if successful, false if failed
M3F20xm_ADCRead = adc_dll.M3F20xm_ADCRead
M3F20xm_ADCRead.argtypes = [c_ubyte, POINTER(c_ushort)]
M3F20xm_ADCRead.restype = c_bool

# trigger sampling
# prototype: bool M3F20xm_ADCStart(BYTE byIndex)
# Note: return ture if successful, false if failed
M3F20xm_ADCStart = adc_dll.M3F20xm_ADCStart
M3F20xm_ADCStart.argtypes = [c_ubyte]
M3F20xm_ADCStart.restype = c_bool

# stop sampling trigger
# prototype: bool M3F20xm_ADCStop(BYTE byIndex)
# Note: return ture if successful, false if failed
M3F20xm_ADCStop = adc_dll.M3F20xm_ADCStop
M3F20xm_ADCStop.argtypes = [c_ubyte]
M3F20xm_ADCStop.restype = c_bool

# Initialize and clean internal FIFO buffer
# prototype: bool M3F20xm_InitFIFO(BYTE byIndex)
# Note: return ture if successful, false if failed
M3F20xm_InitFIFO = adc_dll.M3F20xm_InitFIFO
M3F20xm_InitFIFO.argtypes = [c_ubyte]
M3F20xm_InitFIFO.restype = c_bool

# Read internal FIFO buffer
# prototype: bool M3F20xm_ReadFIFO(BYTE byIndex, BYTE* lpBuffer, DWORD dwBuffSize, DWORD* pdwRealSize)
# Note: lpBuffer: buffer for data
#       dwBuffSize: requested data length, type depends on range selection
#       pdwRealSize: actual data length 
# Note: return ture if successful, false if failed
M3F20xm_ReadFIFO = adc_dll.M3F20xm_ReadFIFO
M3F20xm_ReadFIFO.argtypes = [c_ubyte, POINTER(c_ushort), c_ulong, POINTER(c_ulong)]
M3F20xm_ReadFIFO.restype = c_bool

# Get unread FIFO length
# prototype: bool M3F20xm_GetFIFOLeft(BYTE byIndex, DWORD* pdwBuffsize)
# Note: pdwBuffsize: unread data length in byte
# Note: return ture if successful, false if failed
M3F20xm_GetFIFOLeft = adc_dll.M3F20xm_GetFIFOLeft
M3F20xm_GetFIFOLeft.argtypes = [c_ubyte, POINTER(c_ulong)]
M3F20xm_GetFIFOLeft.restype = c_bool

# Read all register values in the device
# prototype: bool M3F20xm_ReadAllReg(BYTE byIndex, BYTE* pbyValue)
# Note: pbyValue: point to the the register values, must be 47 bytes long, 
#       corresponding to registers 0x01~0x2F
# Note: return ture if successful, false if failed
M3F20xm_ReadAllReg = adc_dll.M3F20xm_ReadAllReg
M3F20xm_ReadAllReg.argtypes = [c_ubyte, POINTER(c_ubyte)]
M3F20xm_ReadAllReg.restype = c_bool

# Update all register values in the device
# prototype: bool M3F20xm_WriteAllReg(BYTE byIndex, BYTE* pbyValue)
# Note: pbyValue: point to the the register values, must be 47 bytes long,
#       corresponding to registers 0x01~0x2F
M3F20xm_WriteAllReg = adc_dll.M3F20xm_WriteAllReg
M3F20xm_WriteAllReg.argtypes = [c_ubyte, POINTER(c_ubyte)]
M3F20xm_WriteAllReg.restype = c_bool

#########################################################################
# pythonic interface

debug_level = 5

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

def pretty_num_unit(v, n_prec = 4):
    # print 100000 as 100k, etc.
    if v == 0:
        return '0'
        #return f'%.{n_prec}f'%(v,)
    scale_st = {
        -1:'m', -2:'u', -3:'n', -4:'p', -5:'f', -6:'a',
        0:'', 1:'k', 2:'M', 3:'G', 4:'T', 5:'P', 6:'E'
    }
    sign = -1 if v < 0 else 1
    v = v * sign
    scale = int(math.floor(math.log(v) / math.log(1000.0)))
    if scale > 6:
        scale = 6
    if scale < -6:
        scale = -6
    v = v * 1000.0 ** (-scale)
    st = f'%.{n_prec}g%s'%(v, scale_st[scale])
    return st

def M3F20xm_ShowConfig(conf):
    # show config in an user friendly way
    print("ADC_CONFIG:")
    print(f"  byADCOptions = {conf.byADCOptions} ({hex(conf.byADCOptions)})")
    print("    [1:0] Trigger mode      :",
        {
            0: 'by GPIO, one sample per event',
            1: 'period, one sample per period',
            2: 'by GPIO then period',
            3: 'by voltage compare then period'
        }[conf.byADCOptions & 0x03])
    print("    [3:2] Trigger edge      :", 
            {
                0: 'falling',
                1: 'rising',
                2: 'both'
            }[(conf.byADCOptions >> 2) & 0x03])
    print("    [4]   Trigger compare   :",
            {
                0: 'greater or equal',
                1: 'less or equal'
            }[(conf.byADCOptions >> 4) & 0x01])
    print("    [5]   Sample period unit:",
            {
                0: '1 us',
                1: '1 ms'
            }[(conf.byADCOptions >> 5) & 0x01])
    print("    [7]   Trigger status    :", 
            {
                0: 'stopped',
                1: 'on'
            }[(conf.byADCOptions >> 7) & 0x01])
    t_unit = 1e-3 if (conf.byADCOptions & 0x20) else 1e-6
    t_cnt = conf.wPeriod
    t_intv = t_unit * t_cnt
    print(f"  byGPIO      = {conf.byGPIO} ({hex(conf.byGPIO)})")
    print(f"    [3:0] GPIO direction (0=output, 1=input):"
            f"{conf.byGPIO & 0x0F :04b} (port 4~1)")
    print(f"    [7:4] GPIO voltage level (0=low, 1=high):"
            f"{(conf.byGPIO >> 4) & 0x0F :04b} (port 4~1)")
    print(f"  byActived   = {conf.byActived:08b} (channel for trigger)")
    print(f"  byReserved  = {conf.byReserved:08b} (reserved)")
    print(f"  wTrigVol    = {conf.wTrigVol} (trigger voltage in mV)")
    print(f"  wPeriod     = {conf.wPeriod} (sampling period)")
    print(f"  wPreNum     = {conf.wPreNum} (pre-sampling number after trigger)")
    print(f"  wReserved   = {conf.wReserved} (reserved)")
    print(f"  dwCycleCnt  = {conf.dwCycleCnt} (sampling cycles)")
    print(f"  dwMaxCycles = {conf.dwMaxCycles} (max sampling cycles, 0=no limit)")
    print(f"  dwReserved  = {conf.dwReserved} (reserved)")
    print(f"  Sampling frequency: {pretty_num_unit(1.0/t_intv)}Hz.")
    print(f"  Duration: {t_intv*conf.dwMaxCycles:.6g} s.")

def AD7606C_ShowReg(reg_list, simple = False):
    if simple:
        print("Register list:")
        for ii in range((len(reg_list)-1)//8 + 1):
            for jj in range(min(8, len(reg_list)-ii*8)):
                print(f"0x{reg_list[ii*8+jj]:02X}", end=' ')
            print('')
        print('')
        return

    reg = lambda n: reg_list[n-1]
    # AD7606C-16 Manual, p-p 57, Table 31.
    print("Register list:")
    print(f"  RESET_DETECT : {reg(0x01)>>7 & 0x01}")
    print(f"  DIGITAL_ERROR: {reg(0x01)>>6 & 0x01}")
    print(f"  OPEN_DETECTED: {reg(0x01)>>5 & 0x01}")
    print(f"  CONFIG:")
    print(f"    STATUS_HEADER : {reg(0x02)>>6 & 0x01}")
    print(f"    EXT_OS_CLOCK  : {reg(0x02)>>5 & 0x01}")
    dout_fmt_st = {
        0b00: '1 D_outx',
        0b01: '2 D_outx',
        0b10: '4 D_outx',
        0b11: '8 D_outx'
    }
    print(f"    DOUT_FORMAT   : {reg(0x02)>>3 & 0x03} ({dout_fmt_st[reg(0x02)>>3 & 0x03]}))")
    op_mode_st = {
        0b00: 'normal',
        0b01: 'standby',
        0b10: 'autostandby',
        0b11: 'shutdown'
    }
    print(f"    OPERATION_MODE: {reg(0x02)>>0 & 0x03} ({op_mode_st[reg(0x02)>>0 & 0x03]}))")
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
    # AD7606C-16 Manual, p-p 34, Table 18, Table 19
    print(f"  BANDWIDTH: {reg(0x07):08b} (CH 8~1) (25kHz or 220kHz)")
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
    print('')

def AD7606C_VoltRangeGetReg(volt_range, ends='single'):
    # volt_range: [min, max] in V
    # ends = 'single' or 'diff'
    # TODO: move this to independent function, allow set channel differently
    v_table_bs = [2.5, 5.0, 6.25, 10.0, 12.5]  # bipolar single-ended
    v_table_ps = [5, 10, 12.5]                 # positive single-ended
    v_table_bd = [5, 10, 12.5, 20]             # bipolar differential
    if volt_range[0] > volt_range[1]:
        # swap
        volt_range = [volt_range[1], volt_range[0]]
    if ends == 'single':
        if volt_range[0] < 0:
            # bs type
            vtb = v_table_bs
            reg_offset = 0
        else:
            # ps type
            vtb = v_table_ps
            reg_offset = 5
    elif ends == 'diff':
        # bd type
        vtb = v_table_bd
        reg_offset = 8
    volt_abs_max = max(abs(volt_range[0]), abs(volt_range[1]))
    for i in range(len(vtb)):
        print(f"i = {i}, v = {vtb[i]} vs {volt_abs_max}")
        if volt_abs_max <= vtb[i]:
            break
    if (i >= len(vtb)) or (volt_abs_max > vtb[i]):
        raise ValueError("Requested input voltage range out of ADC's specification.")
    reg_range = reg_offset + i
    # TODO: return true range, and return proper data value type
    return reg_range

_ch_range_type = {
    # ref. AD7606C-16 Manual, p-p 59, Table 34
    # ref. AD7606C-16 Manual, p-p 30, ADC Transfer Function
    0b0000: (c_short , 'h', -2.5,  2.5,  1),   # "±2.5 V single-ended",
    0b0001: (c_short , 'h', -5.0,  5.0,  1),   # "±5   V single-ended",
    0b0010: (c_short , 'h', -6.25, 6.25, 1),   # "±6.25V single-ended",
    0b0011: (c_short , 'h', -10.0, 10.0, 1),   # "±10  V single-ended",
    0b0100: (c_short , 'h', -12.5, 12.5, 1),   # "±12.5V single-ended",
    0b0101: (c_ushort, 'H',  0.0,   5.0, 1),   # "0~ 5  V single-ended",
    0b0110: (c_ushort, 'H',  0.0,  10.0, 1),   # "0~10  V single-ended",
    0b0111: (c_ushort, 'H',  0.0,  12.5, 1),   # "0~12.5V single-ended",
    0b1000: (c_short , 'h', -5.0,   5.0, 2),   # "±5   V differential",
    0b1001: (c_short , 'h', -10.0, 10.0, 2),   # "±10  V differential",
    0b1010: (c_short , 'h', -12.5, 12.5, 2),   # "±12.5V differential",
    0b1011: (c_short , 'h', -20.0, 20.0, 2),   # "±20  V differential",
    0b1100: (c_ushort, 'H', -2.5,  2.5,  0),   # "(not defined)",
    0b1101: (c_ushort, 'H', -2.5,  2.5,  0),   # "(not defined)",
    0b1110: (c_ushort, 'H', -2.5,  2.5,  0),   # "(not defined)",
    0b1111: (c_ushort, 'H', -2.5,  2.5,  0),   # "(not defined)",
}

def AD7606C_ChRangeGetType(ch_range_code):
    return _ch_range_type[ch_range_code]

def AD7606C_ChRangeAdapter(ch_range_code):
    ch_info = AD7606C_ChRangeGetType(ch_range_code)
    # return sampling type for one, group, type code, lambda for conversion
    ty1 = ch_info[0]
    ty8 = ty1 * 8
    typecode = ch_info[1]
    if typecode == 'h':
        fi2v = lambda i: i / 32768.0 * ch_info[3]
    else: # typecode == 'H'
        fi2v = lambda i: i / 65536.0 * ch_info[3]
    return ty1, ty8, typecode, fi2v

class M3F20xmADC:
    """Pythonic interface for M3F20xm interfaced ADC, mainly AD7606C."""
    REG_LIST_LENGTH = 47
    SERIAL_NUMBER_LENGTH = 10   # include \0
    n_channel = 8
    _ty0up = POINTER(c_ushort)

    def __init__(self, reset = False):
        self.device_number = None
        self.serial_number = None
        self.version_string = {}
        self.adc_config = None
        self.reg_list = None
        self.ready = False
        self.need_notify = False
        self.b_adc_started = False
        if not reset:
            self.init()
        else:
            try:
                self.init()
            except ConnectionError as e:
                time.sleep(1)
                self.close()
                time.sleep(1)
                self.init()  # one more try

    def usb_ready_callback(iDevIndex, iDevStatus):
        """callback function for USB plug-in/out notification."""
        dbg_print(3, f"iDevIndex: {iDevIndex}, iDevStatus: {iDevStatus}")
        if iDevStatus == 0x00:
            print(3, "Device unplugged.")
        elif iDevStatus == 0x80:
            print(3, "Device plugged in.")
        return True
    
    def init(self):
        if self.need_notify:
            # Convert the Python callback function to a C callback function
            USB_READY_CALLBACK = M3F20xm_USB_READY_CALLBACK(self.usb_ready_callback)

            # Call M3F20xm_SetUSBNotify with the callback function
            device_log = True
            result = M3F20xm_SetUSBNotify(device_log, USB_READY_CALLBACK)
            dbg_print(5, f"M3F20xm_SetUSBNotify return {result}.")

        # wait a while for the device to be ready (dll initialization)
        # 0.04 second is at the boundary of success and failure, so we choose 0.1 second
        time.sleep(0.1)

        device_number = M3F20xm_OpenDevice()
        if device_number == 0xFF:
            dbg_print(1, "Failed to open device. e.g. device not plugged in or been used by other program.")
            self.ready = False
            self.at_error(ConnectionError("Failed to open device. (return 255)"))
        else:
            dbg_print(3, f"Device opened successfully. Device Number: {device_number}")
            self.ready = True
            self.device_number = device_number

        time.sleep(0.1)

        # call M3F20xm_Verify
        result = M3F20xm_Verify(device_number)
        dbg_print(4, f"M3F20xm_Verify return {result}.")
        self.verify_result = result

        time.sleep(0.1)  # not sure

        ver_buffer_size = 256  # >= 50
        ver_buffer = create_string_buffer(ver_buffer_size)

        # loop to query version information
        for i_type in [0, 1, 2]:
            result = M3F20xm_GetVersion(device_number, i_type, ver_buffer)
            # Check the result
            if result:
                q = M3F20xm_GetVersion_query_dict[i_type]
                v = ver_buffer.value.decode('utf-8')
                dbg_print(5, f"Query successful. {q}: {v}")
                self.version_string[q] = v
            else:
                dbg_print(1, f"Error in querying version. result = {result}.")
                self.ready = False
                self.at_error(ConnectionError(f"Error in querying version. result = {result}."))
            time.sleep(0.1)   # not sure

        # call M3F20xm_GetSerialNo to get serial number
        result, self.serial_number = self.status()
        if result == 0:
            self.ready = False
            self.at_error(ConnectionError("Device not found (GetSerialNo)."))
        elif result == 2:
            self.ready = False
            #self.at_error(ConnectionRefusedError(
            #    "Device in use (GetSerialNo), try to close the device first, or try reset it."))

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

        self.init_ch_range()

    def reset(self):
        # reset the device as hard as possible
        logging.info("Resetting the device.")
        self.stop()
        time.sleep(0.1)
        self.close()
        time.sleep(0.1)
        self.init()
        self.config('reset')
        self.set_register('reset')

    def status(self):
        dbg_print(5, 'serial and status query.')
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
            print(f"Error in querying serial number. result = {result}")
        print(f'Serial Number: {serial_number}')
        return result, serial_number

    def config(self, *set_st, **kwargs):
        # refer to ADC_CONFIG in M3F20xm.py for kwargs
        if (len(set_st) > 0) and (set_st[0] == 'reset'):
            kwargs = {
                'byADCOptions': 0x11,
                'byGPIO'      : 0x0F,
                'byActived'   : 0x00,
                'wTrigVol'    : 0x00,
                'wPeriod'     : 10,
                'wPreNum'     : 0,
                'dwCycleCnt'  : 0,
                'dwMaxCycles' : 6000000,
            }
        if not kwargs:
            return
        adc_config = self.adc_config
        valid_keys = dict(adc_config._fields_).keys()
        for k, v in kwargs.items():
            if k not in valid_keys:
                raise KeyError(f'Invalid config key "{k}".')
            setattr(adc_config, k, v)
        result = M3F20xm_ADCSetConfig(self.device_number, byref(adc_config))

    def get_config(self, update = True):
        adc_config = self.adc_config
        if update:
            result = M3F20xm_ADCGetConfig(self.device_number, byref(adc_config))
        dbg_print(5, f"M3F20xm_ADCGetConfig return {result}.")
        return adc_config

    def get_register(self, update = True):
        reg_list = self.reg_list
        if update:
            ret = M3F20xm_ReadAllReg(self.device_number, reg_list)
        dbg_print(5, 'M3F20xm_ReadAllReg return', ret)
        return reg_list

    def set_register(self, reg_list):
        if isinstance(reg_list, str) and reg_list == "reset":
            # Note: reg 0x2F DEVICE_ID is said to be 0x31 in the manual
            #       but it is actually 0x22 in the device (AD7606C).
            reg_list = [
                0x00, 0x08, 0x33, 0x33, 0x33, 0x33, 0xFF, 0x00,
                0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
                0x80, 0x80, 0x80, 0x80, 0x80, 0x80, 0x80, 0x80,
                0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
                0x01, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
                0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x22
            ]
        # reg_list must be a list of 47 unsigned byte intergers
        assert len(reg_list) == self.REG_LIST_LENGTH
        for i, v in enumerate(reg_list):
            #dbg_print(4, 'i, v =', i, v)
            self.reg_list[i] = v
        M3F20xm_WriteAllReg(self.device_number, self.reg_list)

    def get_sampling_interval(self):
        t_unit = 1e-3 if (self.adc_config.byADCOptions & 0x20) else 1e-6
        t_cnt = self.adc_config.wPeriod
        return t_unit * t_cnt

    def set_sampling_interval(self, t_intv):
        # use second as the unit, will choose a closest value
        adc_opt = self.adc_config.byADCOptions
        if t_intv < 1e-3:
            # set to us mode, i.e. bit 5 to 0
            adc_opt &= 0xDF
            tick = int(t_intv * 1e6 + 0.5)
        else:
            # set to ms mode, i.e. bit 5 to 1
            adc_opt |= 0x20
            tick = int(t_intv * 1e3 + 0.5)
        self.config(byADCOptions = adc_opt, wPeriod = tick)

    def set_sampling_rate(self, sr, t_total = None):
        # sr: sampling rate in Hz
        # t_total: total sampling time in second, None means no limit
        self.set_sampling_interval(1.0 / sr)
        t_intv = self.get_sampling_interval()
        if t_total is not None:
            n_cycle = int(1.0 / t_intv * t_total + 0.5)
            self.config(dwCycleCnt = 0, dwMaxCycles = n_cycle)
        else:
            self.config(dwCycleCnt = 0, dwMaxCycles = 0)

    def init_ch_range(self):
        # assume get ch 1 is enough
        range_code = self.reg_list[0x03-1] & 0x0F
        self.ty1, self.ty8, self.typecode, self.fi2v = \
            AD7606C_ChRangeAdapter(range_code)

    def set_input_range(self, volt_range, ends='single'):
        reg_range = AD7606C_VoltRangeGetReg(volt_range, ends)
        reg = self.get_register()
        # all 8 channels are the same
        reg[0x03-1] = reg_range | (reg_range << 4)
        reg[0x04-1] = reg_range | (reg_range << 4)
        reg[0x05-1] = reg_range | (reg_range << 4)
        reg[0x06-1] = reg_range | (reg_range << 4)
        self.set_register(reg)
        self.init_ch_range()
        # TODO: add calibration input port.

    def show_config(self):
        M3F20xm_ShowConfig(self.adc_config)

    def show_reg(self, simple = False):
        reg_list = list(self.get_register())
        AD7606C_ShowReg(reg_list, simple)

    def one_sample(self):
        # read a ADC sample data
        values = (self.ty8)()
        result = M3F20xm_ADCRead(self.device_number, cast(values, self._ty0up))
        dbg_print(5, f"M3F20xm_ADCRead return {result}.")
        dbg_print(4, "ADC sample data:")
        for v in values:
            dbg_print(4, f"    {v} = ({hex(v)})")
        return array.array(self.typecode, values)

    def start(self, n_max_frames):
        result = M3F20xm_InitFIFO(self.device_number)
        dbg_print(5, 'M3F20xm_InitFIFO return', result)
        self.dwBuffSize = 2 * self.n_channel * n_max_frames
        #self.lpBuffer   = (self.ty1 * (self.dwBuffSize // 2))()
        self.lpBuffer_bytes = bytearray(self.dwBuffSize)
        self.lpBuffer   = (c_ushort * (self.dwBuffSize // 2)) \
                          .from_buffer(self.lpBuffer_bytes)
        self.b_adc_started = True
        result = M3F20xm_ADCStart(self.device_number)
        dbg_print(5, 'M3F20xm_ADCStart return', result)

    def stop(self):
        dbg_print(5, 'trying stop ADC.')
        result = M3F20xm_ADCStop(self.device_number)
        dbg_print(5, 'M3F20xm_ADCStop return', result)
        self.b_adc_started = False

    def read(self, n_max_frames = None):
        if n_max_frames is not None:
            return self.readn(n_max_frames)
        # read data in FIFO, might be a faster version
        pdwRealSize = c_ulong(0)
        result = M3F20xm_ReadFIFO(self.device_number,
                                  self.lpBuffer,
                                  self.dwBuffSize,
                                  byref(pdwRealSize))
        arr = array.array(self.typecode, self.lpBuffer_bytes[:pdwRealSize.value])
        #print('size:', self.dwBuffSize, pdwRealSize.value, len(arr))
        # we have few choices
        #   (1) cast type when calling ReadFIFO, then pass buffer to array directly
        #   (2) no cast when calling ReadFIFO, then pass buffer to array by bytes
        #   (3) use bytearray and ctypes .from_buffer
        return arr

    def readn(self, n_max_frames):
        # read data in FIFO
        # Note: lpBuffer: buffer for data
        #       dwBuffSize: requested data length
        #       pdwRealSize: actual data length 
        dwBuffSize = 2 * self.n_channel * n_max_frames
        lpBuffer = (c_short * (dwBuffSize // 2))()   # TODO: set type by get_register, pre allocate memory, set n_max_frames at start
        pdwRealSize = c_ulong(0)
        result = M3F20xm_ReadFIFO(self.device_number, lpBuffer, dwBuffSize, byref(pdwRealSize))
        #dbg_print(5, 'M3F20xm_ReadFIFO return', result)
        #dbg_print(5, f"pdwRealSize: {pdwRealSize.value}")
        arr = array.array('h', lpBuffer[:(pdwRealSize.value // 2)])
        return arr

    def get_fifo_frames_left(self):
        pwdBuffSize = c_ulong(0)
        result = M3F20xm_GetFIFOLeft(self.device_number, byref(pwdBuffSize))
        #dbg_print(5, 'M3F20xm_GetFIFOLeft return', result)
        #dbg_print(4, f"Data still in buffer: {pwdBuffSize.value}")
        return pwdBuffSize.value // 2 // self.n_channel

    def close(self):
        if self.device_number is None:
            return
        if self.b_adc_started:
            self.stop()
        result = M3F20xm_CloseDevice(self.device_number)
        dbg_print(5, 'M3F20xm_CloseDevice return', result)
        self.device_number = None

    def at_error(self, exception):
        self.close()
        if exception is not None:
            raise exception

    def __del__(self):
        self.close()
