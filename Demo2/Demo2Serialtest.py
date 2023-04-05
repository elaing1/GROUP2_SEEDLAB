import struct
import serial
import time

# Set the address of the Arduino's serial port
ser = serial.Serial('/dev/ttyACM1', 115200)
time.sleep(3)

# Function to read from the Arduino's serial port
def ReadfromArduino():
    while (ser.in_waiting > 0):
        try:
            line = ser.readline().decode('utf-8').rstrip()
            print("serial output: ", line)
        except:
            print("Communication Error")

# Send a double and a boolean to the Arduino
my_double = 3.14159
angle = 'a'
message = angle + str(my_double)
ser.write(message.encode('utf-8'))

# Wait for the Arduino to respond
time.sleep(2)
ReadfromArduino()
print("Done")