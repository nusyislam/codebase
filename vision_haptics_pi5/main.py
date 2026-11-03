# Point Cloud 9: haptic driver for the Raspberry Pi Pico 2 W (MicroPython)
#
# Save this to the Pico as main.py so it runs on power-up.
# It receives lines like "0,140,255\n" (left, center, right, each 0-255) over the
# USB cable and drives three vibration motors with PWM.
#
# On boot it buzzes left, center, right once each so the wiring can be checked
# without the Pi. If no command arrives for TIMEOUT_MS, all motors turn off,
# so a crashed Pi script can't leave a motor stuck on.
#
# Wiring, per motor (Mashrafi):
#   Pico GPxx -> 220 ohm -> MOSFET gate, plus 10k ohm from gate to GND
#   MOSFET source -> GND
#   MOSFET drain  -> motor (-)
#   motor (+)     -> 5 V rail (Pi 5V pin is fine for a few coin motors)
#   flyback diode across the motor, striped end on the + side
#   Pico GND and the 5 V supply GND must be connected together

from machine import Pin, PWM
import select
import sys
import time

MOTOR_PINS = (16, 17, 18)  # GP16 = left, GP17 = center, GP18 = right
PWM_FREQ = 1000            # Hz
MAX_DUTY = 0.6             # cap: coin motors are often rated ~3 V, 0.6 x 5 V ~= 3 V average
TIMEOUT_MS = 500

motors = []
for p in MOTOR_PINS:
    m = PWM(Pin(p))
    m.freq(PWM_FREQ)
    m.duty_u16(0)
    motors.append(m)

led = Pin("LED", Pin.OUT)


def set_motor(i, v):
    v = max(0, min(255, v))
    motors[i].duty_u16(int(v / 255 * MAX_DUTY * 65535))


def all_off():
    for i in range(len(motors)):
        set_motor(i, 0)


# Boot self-test
for i in range(len(motors)):
    led.on()
    set_motor(i, 255)
    time.sleep_ms(300)
    set_motor(i, 0)
    led.off()
    time.sleep_ms(200)

poll = select.poll()
poll.register(sys.stdin, select.POLLIN)
last_cmd = time.ticks_ms()
stopped = True

while True:
    if poll.poll(10):
        line = sys.stdin.readline().strip()
        parts = line.split(",")
        if len(parts) == 3:
            try:
                vals = [int(x) for x in parts]
            except ValueError:
                continue
            for i, v in enumerate(vals):
                set_motor(i, v)
            last_cmd = time.ticks_ms()
            stopped = False
            led.toggle()  # blinks while commands are coming in

    if not stopped and time.ticks_diff(time.ticks_ms(), last_cmd) > TIMEOUT_MS:
        all_off()
        led.off()
        stopped = True
