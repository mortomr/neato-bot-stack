import serial

ser = serial.Serial('/dev/ttyAMA0', baudrate=115200, timeout=1)

while True:
    data = ser.read(22)  # XV-11 packets are 22 bytes
    print(" ".join(f"{b:02X}" for b in data))

# loopback_test.py
# import serial

# ser = serial.Serial('/dev/ttyAMA0', baudrate=115200, timeout=1)
# ser.write(b'hello uart\n')
# response = ser.readline()
# print("Received:", response)

