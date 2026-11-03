SENIOR DESIGN — Pi 5 + OAK-D PRO + ONE ADAFRUIT DRV2605L MOTOR
================================================================
This is a one-motor adaptation of the uploaded project. The original
main.py/haptic_bridge.py expect a Pico 2W and THREE MOSFET-controlled
motors; they do NOT operate a DRV2605L as-is.

WIRING (with Pi powered OFF)
---------------------------
Raspberry Pi 5 physical pin 1  (3.3V)  -> DRV2605L VIN
Raspberry Pi 5 physical pin 6  (GND)   -> DRV2605L GND
Raspberry Pi 5 physical pin 3  (GPIO2) -> DRV2605L SDA
Raspberry Pi 5 physical pin 5  (GPIO3) -> DRV2605L SCL
Vibration motor positive/red wire      -> DRV2605L Motor+
Vibration motor negative/blue wire     -> DRV2605L Motor-
OAK-D Pro USB-C                        -> Pi 5 USB 3 port (blue)
Pi 5 power                             -> suitable official/recommended USB-C supply
Pico 2 W                               -> not needed for this single-motor test
DO NOT connect the vibration motor directly to Pi GPIO pins.

RUN ON RASPBERRY PI 5 (Raspberry Pi OS)
-------------------------------------
1. In a terminal, enable I2C:
   sudo raspi-config
   -> Interface Options -> I2C -> Enable
   (or: sudo raspi-config nonint do_i2c 0)

2. Install tools and Python dependencies:
   sudo apt update
   sudo apt install -y i2c-tools python3-venv python3-pip libgpiod-dev python3-libgpiod
   cd /path/to/vision_haptics_pi5
   python3 -m venv --system-site-packages .venv
   source .venv/bin/activate
   python -m pip install 'depthai<3' opencv-python numpy adafruit-circuitpython-drv2605 adafruit-blinka

3. Check controller on I2C bus:
   sudo i2cdetect -y 1
   Expect 5a on the address grid. If absent, STOP and check wiring and I2C.

4. Check motor only:
   python test_motor_drv2605.py
   Motor should buzz for 0.5 sec then stop.

5. Test depth processing without using the motor:
   python haptic_bridge_drv2605.py --dry-run

6. Camera + vibration motor:
   python haptic_bridge_drv2605.py
   Or from SSH with no screen:
   python haptic_bridge_drv2605.py --headless
   Press q in the OpenCV window or Ctrl+C in terminal to stop.

HOW IT BEHAVES
--------------
OAK camera computes depth in millimeters. The code examines the nearest
valid obstacle in the left/center/right portions of the middle band.
For one motor, it uses the nearest of all three:
  >= 1.5m -> no motor vibration
  <= 0.4m -> maximum configured test strength (MAX_RTP=95)
  in between -> proportional vibration
Change NEAR_M, FAR_M or MAX_RTP near the top of haptic_bridge_drv2605.py.

NOTES
-----
- This is an adaptation for the named hardware, not a verified physical
  hardware test. Check motor rating and connections before powering up.
- depth_demo.py is preserved from the original project, written for DepthAI 2.
  Actual camera compatibility depends on model/firmware/installed DepthAI.
- 'main.py' belongs on a Pico running MicroPython for the ORIGINAL
  MOSFET+three-motor project; do NOT run it on the Pi for DRV2605L.
- I2C address of DRV2605L is 0x5A.
- Never leave an unattended powered motor connected during debugging.
