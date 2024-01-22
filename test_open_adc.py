import time
from ctypes import (
    CDLL, windll, Structure, CFUNCTYPE, POINTER,
    c_bool, c_ubyte, c_ushort, c_int, c_ulong, c_char_p,
    byref, create_string_buffer
)

usbadc_dll_path = './USB2DaqsB.dll'
adc_dll = CDLL(usbadc_dll_path)
#adc_dll = windll.LoadLibrary(usbadc_dll_path)

M3F20xm_SetUSBNotify = adc_dll.M3F20xm_SetUSBNotify
M3F20xm_USB_READY_CALLBACK = CFUNCTYPE(c_bool, c_ubyte, c_ulong)
M3F20xm_SetUSBNotify.argtypes = [c_bool, M3F20xm_USB_READY_CALLBACK]
M3F20xm_SetUSBNotify.restype = c_bool

M3F20xm_GetSerialNo = adc_dll.M3F20xm_GetSerialNo
M3F20xm_GetSerialNo.argtypes = [c_ubyte, c_char_p]
M3F20xm_GetSerialNo.restype = c_ubyte

M3F20xm_OpenDevice = adc_dll.M3F20xm_OpenDevice
M3F20xm_OpenDevice.restype = c_ubyte

M3F20xm_CloseDevice = adc_dll.M3F20xm_CloseDevice
M3F20xm_CloseDevice.argtypes = [c_ubyte]
M3F20xm_CloseDevice.restype = c_bool

M3F20xm_Verify = adc_dll.M3F20xm_Verify
#M3F20xm_Verify.argtypes = [c_ubyte, POINTER(c_ubyte)]
M3F20xm_Verify.argtypes = [c_ubyte]
M3F20xm_Verify.restype = c_bool

# Define the callback function
def usb_ready_callback(iDevIndex, iDevStatus):
    print(f"iDevIndex: {iDevIndex}, iDevStatus: {iDevStatus}")
    if iDevStatus == 0x00:
        print("Device unplugged.")
    elif iDevStatus == 0x80:
        print("Device plugged in.")
    return True
    
USB_READY_CALLBACK = M3F20xm_USB_READY_CALLBACK(usb_ready_callback)

# Call M3F20xm_SetUSBNotify with the callback function
result = M3F20xm_SetUSBNotify(True, USB_READY_CALLBACK)
print(f"M3F20xm_SetUSBNotify return {result}.")

# sleep 0.1 second to ensure device works
time.sleep(0.1)

device_number = M3F20xm_OpenDevice()
print('Device number:', device_number)

serial_buffer = create_string_buffer(10)
result = M3F20xm_GetSerialNo(device_number, serial_buffer)
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

#verify_result = c_ubyte(2)
#result = M3F20xm_Verify(device_number, byref(verify_result))
#print(f"M3F20xm_Verify return {result}. Verify result: {verify_result.value}.")

result = M3F20xm_Verify(device_number)
print(f"M3F20xm_Verify return {result}")

if device_number == 0xFF:
    device_number = 0
M3F20xm_CloseDevice(device_number)
