# rail-teleop

Bare-metal C joystick controller for an ELEGOO Nano V3 (16 MHz ATmega328P),
a NEMA17 stepper, and a TB6600 driver configured for **1600 pulses/revolution**.
The Nano drives the motor directly and reports its limited velocity over USB serial.

## Build and flash

Install `avr-gcc`, AVR libc/binutils, `avrdude`, Python 3, and Make. On Ubuntu:

```sh
sudo apt install make gcc-avr avr-libc binutils-avr avrdude python3
```

From this directory:

```sh
make build                          # build/rail-teleop.elf
make build SCALE=0.1                 # default linear scale
make test                           # requires a host C compiler
make flash PORT=/dev/ttyUSB0         # newer Nano bootloader, 115200 baud
make flash PORT=/dev/ttyUSB0 BAUD=57600  # old Nano bootloader
make clean
```

`BAUD` controls bootloader upload speed only. Runtime serial is always 115200
baud, 8 data bits, no parity, one stop bit. Changing `SCALE` rebuilds the firmware.
Use the same `SCALE` on the flash command if you selected a nondefault scale.

## Wiring

| Signal | Nano pin | Connection |
| --- | --- | --- |
| Joystick VRx | A0 | Unused |
| Joystick SW | D12 | Button to GND; internal pull-up enabled |
| Joystick VRy | A1 | Y-axis analog output; velocity control |
| Joystick power | 5V / GND | Module VCC / GND |
| STEP | D9 | TB6600 PUL− |
| DIR | D8 | TB6600 DIR− |
| Enable | D7 | TB6600 ENA− |
| TB6600 common anode | 5V | PUL+, DIR+, ENA+ |

The driver wiring is common anode. STEP pulses are active LOW. ENA LOW disables
this module; ENA HIGH enables it. DIR HIGH commands motion away from the zero
end. USB serial uses the Nano's D0/D1 UART; leave those pins free for USB.
The joystick's X axis is unused.

## Calibration and operation

The firmware starts uncalibrated with the driver disabled. Move the carriage by
hand to the same zero end used by rail-drive, then press the joystick. After
20 ms of stable press, the firmware zeros its pulse count and enables the motor.
It holds zero velocity until the joystick returns to center. Increasing VRy
then commands positive velocity; decreasing VRy commands negative velocity.
The center is ADC count 471 (measured 2.3 V, assuming a 5.0 V AVcc reference),
with a ±25-count deadband (approximately 2.18–2.42 V) and linear scaling to
full velocity at each voltage endpoint. Negative motion is blocked at position
zero until the carriage has moved in the positive direction.

The next button press immediately stops pulse generation and disables the driver,
bypassing jerk and acceleration limits. `sp 0` is prioritized at the next control
cycle. An already-started serial line finishes before that stop line. A stable
20 ms release is required before another enable press. Every enable establishes
a new zero: move the carriage back to the zero end before re-enabling.
A missed control deadline or an attempted pulse beyond a physical position bound
also disables the driver and invalidates calibration.

Position is counted from emitted pulses, without an encoder or limit switches.
Missed steps or manual movement while disabled are not measured.

## Limits and timing

All linear constants from rail-drive are multiplied by the build-time `SCALE`:

| Quantity | Formula | Default (`SCALE=0.1`) |
| --- | --- | --- |
| Lead screw pitch | 4 × scale mm/revolution | 0.4 mm/revolution |
| Position limits | 0–500 × scale mm | 0–50 mm |
| Position margin | 2 × scale mm | 0.2 mm |
| Velocity limit | ±32 × scale mm/s | ±3.2 mm/s |
| Acceleration limit | ±50 × scale mm/s² | ±5 mm/s² |
| Jerk limit | ±50 × scale mm/s³ | ±5 mm/s³ |

The motion loop uses rail-drive's jerk-limited velocity profile and stopping-distance
calculation. It brakes toward the interior envelope (0.2–49.8 mm by default).
Within either margin, only inward commands are accepted. Physical bounds also
prevent outward pulses. Both geometry and velocity scale together, so maximum
pulse rate stays 12.8 kHz and full travel stays 2,000,000 pulses.

Timer1 releases a 40 kHz ISR. An integer phase accumulator schedules pulses and
counts signed position; floating-point motion calculations run outside the ISR
at 1 kHz. Pulses occupy one timer tick, with at least one inactive tick between
pulses and one setup tick after direction changes. Nominal tick duration is 25 µs;
actual edge timing includes interrupt latency. Control overruns disable the driver.

Serial sends newline-terminated `sp <vel>` at 100 Hz. Velocity is the quantized
pulse-scheduler setpoint in mm/s, with three decimal places, for example:

```text
sp 0.000
sp 1.250
sp -3.200
```

Startup and disable also send `sp 0`. No incoming commands or acknowledgements
are required. TX is buffered and nonblocking; a full queue drops whole telemetry
lines, and disable discards queued telemetry after any line already in progress.

To view serial output, install pyserial and run the receive-only monitor:

```sh
python3 -m pip install pyserial
python3 serial_monitor.py /dev/ttyUSB0
```

On macOS, use the Nano's `/dev/cu.usbserial-*` port instead. The monitor uses
115200 baud; press Ctrl+C to stop. Opening the serial port may reset the Nano.

## Verification

`make test` runs host motion simulations at scales 0.1 and 1.0, plus tests against
mocked AVR registers for enable/disable, held presses, re-zeroing, center gating,
pulse counts, direction changes, physical bounds, deadline failure, and serial
formatting/stop priority. These tests do not measure actual AVR execution time.

Before full-travel operation, check on hardware:

1. Startup and a disable press leave ENA LOW and STEP HIGH.
2. An enable press zeros position but produces no pulses until the joystick centers.
3. Positive deflection moves away from the chosen zero end.
4. A logic analyzer shows active-low pulses, minimum pulse/inactive/setup timing,
   and a maximum average rate of 12.8 kHz; verify the 1 kHz control loop stays active
   without deadline-triggered disable, including while serial output is running.
5. Serial is readable at 115200 baud and a disable press promptly produces `sp 0`.
6. Motion brakes before both margins and can move inward afterward.
