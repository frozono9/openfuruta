# ESP32 firmware

The [bring-up sketch](bringup/) reads the pendulum encoder without issuing motion commands. The [closed-loop sketch](closed_loop/) contains the v2 approach controller from the prototype tests. It includes travel limits, a pulse-command lease, encoder checks, shaft tracking and return to center.

## Build and flash

Install [PlatformIO Core](https://docs.platformio.org/en/latest/core/installation/index.html). Both projects target esp32dev and Arduino. From the repository root:

~~~sh
pio run -d firmware/bringup
pio run -d firmware/closed_loop
pio device list
~~~

Replace SERIAL_PORT with the path of your ESP32. Disconnect 12 V motor power while flashing and commissioning the sensors.

~~~sh
pio run -d firmware/bringup -t upload --upload-port SERIAL_PORT
pio device monitor -p SERIAL_PORT -b 115200 --eol LF
~~~

Rotate the pendulum by hand. Check for address 0x36, a smooth angle trace, magnet detected and no strong or weak field flags. Adjust the magnet position or gap as needed. Close the monitor, flash the closed-loop firmware, then reopen the serial monitor:

~~~sh
pio run -d firmware/closed_loop -t upload --upload-port SERIAL_PORT
pio device monitor -p SERIAL_PORT -b 115200 --eol LF
~~~

## Calibrate

1. Let the pendulum hang still. Send **MAGNETREAD** and **SHAFTREAD** on separate lines. The shaft status bits 5:3 should be binary 100. Both angles should track hand movement and roll over smoothly.
2. Send **ZERO**. If it rejects the current reading, take the raw pendulum angle from the third telemetry column while hanging still. If it exceeds 180°, subtract 360°. Send **ZERORECOVER** followed by the signed angle in degrees. Keep the pendulum still until **EVENT,GRAVITY_ZERO_READY**. The ESP32 stores this calibration.
3. With motor power off, align the arm to the mechanical center marked during assembly. Send **SHAFTREAD** and record its reported angle as your home. Use this value for your unit.
4. Under supervision, with the workspace clear, connect 12 V motor power. Send **TMCVERIFY** and expect **EVENT,TMC_UART_VERIFIED**. If verification fails, switch off motor power and check VDD, PDN/UART, the address straps and ground. Verify the expected 16 external microsteps on your carrier.
5. With the pendulum still, send **CENTER** and wait for **EVENT,CENTER_CONFIRMED**. Send **HOMETEST 25** for a roughly 2.8° positive move and return. Confirm the motor-shaft encoder increases with positive counted steps and the arm follows the shaft without slipping. **STOP** disables the driver when the serial link is working. Keep the independent motor power cutoff within reach.

## Run one swing-up

Close the serial monitor. Secure the rig, clear both sweeps and verify the full travel envelope. The runner selects v2 and starts one supervised trial. It checks both encoders, proximity to the measured home, telemetry freshness, travel and shaft tracking. Use the cutoff if you lose the serial link or see mechanical slip.

~~~sh
python -m pip install -r firmware/requirements.txt
python firmware/run_trial.py --port SERIAL_PORT --home-deg YOUR_MEASURED_ANGLE
~~~

The runner uses the tested swing limits of 7.5 rad/s and 65 rad/s². It sends **STOP** on exit to remove holding current. After a fault, inspect the rig and manually return to the measured mechanical center with motor power off before retrying. The published physical trial telemetry came from the original monitoring setup. This portable runner uses the same ESP32 interface and has been exercised with recorded and simulated serial input.

The ESP32 estimates pendulum angle and rate from the hinge AS5600. V2 injects energy while the pendulum swings, checks the measured state before engaging balance, and commands return to center after a hold or normal swing-up timeout. The [native simulation](../v2/hil/) exercises the controller with simulated sensor and motor timing.
