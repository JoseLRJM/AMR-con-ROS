#!/usr/bin/python3
import time
import serial

print("UART Demonstration Program")
print("NVIDIA Jetson Nano Developer Kit")


serial_port = serial.Serial(
    port="/dev/ttyTHS0",
    baudrate=115200,
    bytesize=serial.EIGHTBITS,
    parity=serial.PARITY_NONE,
    stopbits=serial.STOPBITS_ONE,
)
# Wait a second to let the port initialize
time.sleep(1)

try:
    # Send a simple header
    serial_port.write("UART Demonstration Program\r\n".encode())
    serial_port.write("NVIDIA Jetson Nano Developer Kit\r\n".encode())
    while True:
        if serial_port.inWaiting() > 0:
            data = serial_port.read(6)#_until(b'\xff')
            arr= bytearray(data)
            #if arr[0]==161:
                #jetson_check= (arr[0] + arr[1] + arr[2] + arr [3]) & 255
            jetson_check= int(arr[1]<<24 + arr[2]<<16 + arr[3]<<8 + arr[4])
            print("vel_izq "+str(arr[1]))
            print("vel_der "+str(arr[2]))
            print("sentido "+str(arr[3]))
            print("checksum "+str(arr[4]))
            print("jetson_check "+str(jetson_check))

            #print(data)
            #serial_port.write(data)
            #int_val = int.from_bytes(data[1], "big")
            
            if data == "\r".encode():
                # For Windows boxen on the other end
                serial_port.write("\n".encode())


except KeyboardInterrupt:
    print("Exiting Program")

except Exception as exception_error:
    print("Error occurred. Exiting Program")
    print("Error: " + str(exception_error))

finally:
    serial_port.close()
    pass