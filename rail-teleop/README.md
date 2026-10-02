# rail-teleop

Bare-metal C joystick controller for an ELEGOO Nano V3 (16 MHz ATmega328P),
a NEMA17 stepper, and a TB6600 driver configured for **3200 pulses/revolution**.
The Nano drives the motor directly and reports its limited velocity over USB serial.

## Build and flash

Install `avr-gcc`, AVR libc/binutils, `avrdude`, Python 3, and Make. On Ubuntu:

```sh
sudo apt install make gcc-avr avr-libc binutils-avr avrdude python3
```

From this directory:

```sh
make build                          # build/rail-teleop.elf
make build SCALE=0.2                 # default linear scale
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
The first press also captures the current VRy reading on A1 as joystick center;
leave the joystick at rest while pressing SW. Increasing VRy commands positive
velocity; decreasing VRy commands negative velocity. A ±25-count deadband is
applied around the captured center. Outside it, velocity scales linearly to
full negative at ADC count 0 and full positive at ADC count 1023. Negative
motion is blocked at position zero until the carriage has moved positively.

The first stable 20 ms press starts velocity broadcasting at 100 Hz and enables
operation. Later presses are ignored and do not re-zero position or disable the
driver. Reset the Nano to start over. At a physical position bound, the driver
remains enabled: outward pulses are blocked and inward movement is allowed.
Missed control cycles retain the last pulse rate and leave the driver enabled.
Joystick velocity broadcasting continues independently of track position.

Position is counted from emitted pulses, without an encoder or limit switches.
Missed steps or manual movement while disabled are not measured.

## Limits and timing

Position, velocity, and margin are multiplied by the build-time `SCALE`.
Acceleration and jerk are divided by `SCALE`:

| Quantity | Formula | Default (`SCALE=0.2`) |
| --- | --- | --- |
| Belt travel per revolution | π × 20 mm (unscaled) | 62.83 mm/revolution |
| Position limits | 0–500 × scale mm | 0–100 mm |
| Position margin | 2 × scale mm | 0.4 mm |
| Velocity limit | ±32 × scale mm/s | ±6.4 mm/s |
| Acceleration limit | ±50 ÷ scale mm/s² | ±250 mm/s² |
| Jerk limit | ±50 ÷ scale mm/s³ | ±250 mm/s³ |

The motion loop uses rail-drive's jerk-limited velocity profile and stopping-distance
calculation. It brakes toward the interior envelope (0.4–99.6 mm by default).
Within either margin, only inward commands are accepted. Physical bounds also
prevent outward pulses. The belt gear is assumed to have an effective driving
diameter of 20 mm, giving 62.83 mm/revolution and 50.93 pulses/mm at 3200 pulses/revolution.
Linear speed is pulse frequency × 62.83 / 3200 mm/s. At the default scale,
maximum pulse rate is 325.95 Hz and the travel bound is 5092 pulses (rounded down).

Timer1 releases a 40 kHz ISR. An integer phase accumulator schedules pulses and
counts signed position; floating-point motion calculations run outside the ISR
at 1 kHz. Pulses occupy one timer tick, with at least one inactive tick between
pulses and one setup tick after direction changes. Nominal tick duration is 25 µs;
actual edge timing includes interrupt latency. Missed control ticks coalesce into one pending control cycle.

Serial sends newline-terminated `sp <vel>` at 100 Hz. Velocity is the calibrated
joystick command in mm/s, with three decimal places. Track position, motor
enable state, acceleration, and jerk do not constrain this value:

```text
sp 0.000
sp 1.250
sp -6.400
```

Serial output begins after the first press. No incoming commands or
acknowledgements are required. TX is buffered and nonblocking; a full queue
drops whole telemetry lines.

To view serial output, install pyserial and run the receive-only monitor:

```sh
python3 -m pip install pyserial
python3 serial_monitor.py /dev/ttyUSB0
```

On macOS, use the Nano's `/dev/cu.usbserial-*` port instead. The monitor uses
115200 baud; press Ctrl+C to stop. Opening the serial port may reset the Nano.

## Verification

`make test` runs host motion simulations at scales 0.2 and 1.0, plus tests against
mocked AVR registers for startup enable, held/repeated presses, joystick center calibration,
pulse counts, direction changes, physical bounds, delayed control servicing, and serial
formatting/stop priority. These tests do not measure actual AVR execution time.

Before full-travel operation, check on hardware:

1. Startup leaves ENA LOW and STEP HIGH.
2. The first press zeros position and calibrates joystick center without pulses.
3. Positive deflection moves away from the chosen zero end.
4. A logic analyzer shows active-low pulses, minimum pulse/inactive/setup timing,
   and a maximum average rate of 325.95 Hz at the default scale; verify the 1 kHz control loop stays active
   including while serial output is running.
5. Serial is readable at 115200 baud and later presses leave operation enabled.
6. Motion brakes before both margins and can move inward afterward.
