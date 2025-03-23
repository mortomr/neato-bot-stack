#!/usr/bin/env python3
import serial
import time

def build_command(motor: str, direction: str, speed: int) -> int:
    motor_bit = 0b10000000 if motor.lower() == "right" else 0
    dir_bit = 0b01000000 if direction.upper() == "CCW" else 0
    return motor_bit | dir_bit | (speed & 0x3F)

def send_command(ser, cmd_byte):
    ser.write(bytes([cmd_byte]))
    print(f"Sent: 0x{cmd_byte:02X} ({cmd_byte})")

def sweep_test(port="/dev/ttyUSB0", baud=9600, step=8, delay=1.5):
    ser = serial.Serial(port, baud, timeout=1)
    print("🌀 Starting motor sweep test...")

    for speed in range(0, 64, step):
        print(f"\n🟡 Speed: {speed}")
        left_cmd = build_command("left", "CW", speed)     # Left motor: CW = forward
        right_cmd = build_command("right", "CCW", speed)  # Right motor: CCW = forward
        send_command(ser, left_cmd)
        send_command(ser, right_cmd)
        time.sleep(delay)

    print("\n🟥 Stopping both motors...")
    send_command(ser, build_command("left", "CW", 0))
    send_command(ser, build_command("right", "CCW", 0))
    ser.close()
    print("✅ Test complete.")

if __name__ == "__main__":
    sweep_test()

