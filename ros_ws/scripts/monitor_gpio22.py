import RPi.GPIO as GPIO
import time

GPIO.setmode(GPIO.BCM)
GPIO.setup(22, GPIO.IN)

print("📡 Monitoring GPIO22... (press Ctrl+C to stop)")
try:
    while True:
        val = GPIO.input(22)
        print("HIGH" if val else "LOW")
        time.sleep(0.05)
except KeyboardInterrupt:
    GPIO.cleanup()
    print("👋 Done.")

