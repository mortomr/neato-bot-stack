import serial

# ser = serial.Serial('/dev/ttyUSB0', baudrate=115200, timeout=1)
ser = serial.Serial('/dev/ttyAMA10', baudrate=115200, timeout=1)

while True:
    data = ser.read(22)
    if data:
        print(" ".join(f"{b:02X}" for b in data))
