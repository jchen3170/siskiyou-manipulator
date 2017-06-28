# from ctypes import *
import ctypes
import serial

if __name__ == "__main__":
    # port = "COM2"
    # port = "\\\\.\\COM2"
    port = "/dev/ttyUSB0"
    baud = 38400
    ser = serial.Serial(port=port, baudrate=baud, timeout=3)

    print (ser.isOpen())

    s = '2 ST\n\n'
    # arr = (c_byte * len(s)).from_buffer_copy(c_byte(s)
    # arr = c_wchar_p(s)
    arr = ctypes.create_string_buffer(bytes(s.encode('ascii')))
    

    bw = 0;
    ctypes.windll.kernel32.WriteFile(ser, arr, len(arr), bw, None)

    ser.close()