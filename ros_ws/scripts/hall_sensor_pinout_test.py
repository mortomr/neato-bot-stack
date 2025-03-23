import RPi.GPIO as GPIO
import time

# Assign test GPIO pins (change if needed)
test_pins = [17, 27, 22]  # Orange, Brown, Blue (in any order)

def pulse_test(vcc, gnd, signal):
    print(f"\n🔌 Testing VCC: GPIO{vcc}, GND: GPIO{gnd}, Signal: GPIO{signal}")

    # Setup roles
    GPIO.setup(vcc, GPIO.OUT)
    GPIO.output(vcc, GPIO.HIGH)

    GPIO.setup(gnd, GPIO.OUT)
    GPIO.output(gnd, GPIO.LOW)

    GPIO.setup(signal, GPIO.IN, pull_up_down=GPIO.PUD_UP)

    input("🌀 Rotate the motor shaft for 5 seconds then press Enter...")

    pulse_count = 0
    timeout = time.time() + 5
    prev = GPIO.input(signal)

    while time.time() < timeout:
        val = GPIO.input(signal)
        if prev == GPIO.HIGH and val == GPIO.LOW:
            pulse_count += 1
        prev = val
        time.sleep(0.001)

    print(f"📈 Pulses detected: {pulse_count}")
    GPIO.cleanup()

def main():
    print("🧪 Starting Hall effect encoder pinout test...")

    for i in range(3):
        for j in range(3):
            if i == j:
                continue
            k = 3 - i - j
            GPIO.setmode(GPIO.BCM)  # Reset mode each time after cleanup
            pulse_test(test_pins[i], test_pins[j], test_pins[k])
            time.sleep(1)

    print("✅ Test complete. Look for the combo with the most pulses.")

if __name__ == "__main__":
    main()

