"""One-motor DRV2605L test on Raspberry Pi 5 (NOT for Pico 2 W).

Pi 3V3 (physical pin 1) -> DRV2605L VIN
Pi GND (physical pin 6) -> DRV2605L GND
Pi GPIO2/SDA (pin 3) -> DRV2605L SDA
Pi GPIO3/SCL (pin 5) -> DRV2605L SCL
Motor + -> DRV2605L Motor+
Motor - -> DRV2605L Motor-
"""
import time

import board
import adafruit_drv2605


def main():
    drv = adafruit_drv2605.DRV2605(board.I2C())
    drv.use_ERM()  # Adafruit mini vibration disc motor is an ERM motor.
    drv.realtime_value = 0
    drv.mode = adafruit_drv2605.MODE_REALTIME
    try:
        print("Motor test: gentle buzz for 0.5 seconds...")
        drv.realtime_value = 65  # Positive, signed 8-bit realtime drive value, range 0..127.
        time.sleep(0.5)
        print("Motor test complete.")
    finally:
        drv.realtime_value = 0
        drv.mode = adafruit_drv2605.MODE_INTTRIG


if __name__ == "__main__":
    main()
