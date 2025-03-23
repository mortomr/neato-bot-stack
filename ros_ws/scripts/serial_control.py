#!/usr/bin/env python3
import serial
import time

def build_command(motor: str, direction: str, speed: int) -> int:
    """
    Build the command byte for Serial Simplified mode.
    :param motor: 'left' or 'right'
    :param direction: 'CW' or 'CCW'
    :param speed: Integer from 0 (stop) to 63 (full speed)
    :return: Command byte as integer.
    """
    if not (0 <= speed <= 63):
        raise ValueError("Speed must be between 0 and 63")

    # Motor selection: Bit 7 (0 for left, 1 for right)
    if motor.lower() == 'left':
        motor_bit = 0
    elif motor.lower() == 'right':
        motor_bit = 1
    else:
        raise ValueError("Motor must be 'left' or 'right'")

    # Direction: Bit 6 (0 for CW, 1 for CCW)
    if direction.upper() == 'CW':
        direction_bit = 0
    elif direction.upper() == 'CCW':
        direction_bit = 1
    else:
        raise ValueError("Direction must be 'CW' or 'CCW'")

    # Construct command byte: [Motor (1 bit)] [Direction (1 bit)] [Speed (6 bits)]
    command_byte = (motor_bit << 7) | (direction_bit << 6) | (speed & 0x3F)
    return command_byte

def send_command(ser, command_byte: int):
    """
    Send a single command byte over the serial connection.
    """
    ser.write(bytes([command_byte]))
    print(f"Sent command: {command_byte} (0x{command_byte:02X})")

def main():
    # Update the serial port as necessary (common ones: /dev/serial0 or /dev/ttyAMA0)
    # serial_port = "/dev/serial0"
    serial_port = "/dev/ttyUSB0"
    baud_rate = 9600  # Must match your DIP switch configuration on the MDDS30
    try:
        ser = serial.Serial(serial_port, baud_rate, timeout=1)
    except Exception as e:
        print(f"Error opening serial port: {e}")
        return

    # Allow some time for the MDDS30 to power up and initialize (about 1 second)
    time.sleep(1)

    # Example 1: Run left motor at full speed CW (command byte 63)
    command_left_full_cw = build_command("left", "CW", 63)
    send_command(ser, command_left_full_cw)
    time.sleep(3)

    # Stop left motor (speed = 0)
    command_left_stop = build_command("left", "CW", 0)
    send_command(ser, command_left_stop)
    time.sleep(1)

    # Example 2: Run right motor at full speed CCW (command byte 255)
    command_right_full_ccw = build_command("right", "CCW", 63)
    send_command(ser, command_right_full_ccw)
    time.sleep(3)

    # Stop right motor
    command_right_stop = build_command("right", "CW", 0)
    send_command(ser, command_right_stop)

    ser.close()

if __name__ == '__main__':
    main()
