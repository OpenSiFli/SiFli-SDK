# Motor Encoder Drive Example (RT-Thread)

[English](README_EN.md) | [Simplified Chinese](README.md)

Source path: example/motor

## Overview
This example demonstrates the complete flow to drive and control a pair of DC
gear motors on RT-Thread. It includes:

- Reading motor speed/position using pulse encoders.
- Controlling motor duty cycle and direction using PWM outputs.
- Implementing PID control for speed and position loops (including an
  anti-windup variant).
- Providing a finsh Finsh command (`motor`) for real-time control and debugging.

### Brief on the PID algorithm
This example uses classic PID controllers for both the speed and position loops.
The PID controller produces a control signal based on the error between the
setpoint and the measured value to drive the PWM duty. The continuous-time PID
expression is:

$$ u(t)=K_P e(t) + K_I \int_0^{t} e(\tau)\,d\tau + K_D \frac{d e(t)}{d t} $$

where $e(t)=r(t)-y(t)$ is the error between setpoint $r(t)$ and process variable
$y(t)$, and $K_P,K_I,K_D$ are the proportional, integral and derivative gains
respectively.

In embedded sampled implementations, a discrete form is commonly used (with
sampling period $T_s$ and sample index $k$):

$$ u[k]=K_P e[k] + K_I T_s \sum_{i=0}^{k} e[i] + K_D \frac{e[k]-e[k-1]}{T_s} $$

or in recursive form:

$$ u[k]=K_P e[k] + I[k] + K_D \frac{e[k]-e[k-1]}{T_s},\qquad I[k]=I[k-1]+K_I T_s
e[k] $$

Anti-windup strategy: when the drive signal $u[k]$ exceeds the actuator limits
(e.g. PWM duty range 0~100%), the output should be saturated and the integral
accumulation stopped or adjusted to prevent further growth of the integral term
which could cause overshoot. A common simple method is integral clamping:

1) Compute the unsaturated control (including all terms);
2) Clip the control to the allowed range;
3) If clipping occurred, do not update or conditionally update the integral term
   (only accumulate when not saturated or when the error direction helps reduce
   saturation).

Tuning tips: start with $K_I=0, K_D=0$ and increase $K_P$ until acceptable
response speed; then add $K_I$ to eliminate steady-state error; finally tune
$K_D$ to reduce overshoot and oscillation. Speed loop sampling is typically
higher (tens to hundreds of Hz), while position loop sampling can be lower; the
position loop usually acts as an outer loop around the speed loop.

### Quadrature encoder

This example uses incremental quadrature encoders (A/B two channels) for
measuring motor speed and position. The encoder outputs two square waves
approximately 90° out of phase; counters count A/B pulses to obtain incremental
counts.

Wiring

- Use `ENC1A_PIN` / `ENC1B_PIN` for encoder 1 A/B channels; use `ENC2A_PIN` /
  `ENC2B_PIN` for encoder 2 (see the hardware connections table below).

Sampling window and counting

- The code reads and resets the encoder counters in the main loop at a fixed
  window (the main loop uses wake_time += 20), the window is about 20 ms (0.02
  s).
- Therefore the input `pulse` to `pulse_to_rpm(int32_t pulse)` is the pulse
  increment measured during this 20 ms window.

Pulse to RPM conversion

- Convert pulses in the window to pulses per second: pulses_per_second = pulse /
  0.02 = pulse * 50
- Convert pulses per second to rotational speed using pulses per revolution
  (PPR, accounting for encoder resolution, gearbox ratio and quadrature x4
  counting):

  rpm = pulses_per_second / PPR * 60 = pulse * 50 / PPR * 60

- The inverse conversion (converting a target RPM to pulses within the time
  window, used for mapping) is:

  pulse = (int)(rpm / 60 * PPR / 50)

PPR origin

- In this example, total PPR is determined by three parts: encoder's PPR,
  gearbox ratio and quadrature x4 counting. The formula is:

  total_ppr = encoder_ppr * gearbox_ratio * 4

- With example hardware parameters: encoder_ppr = 13, gearbox_ratio = 20.409, we
  get:

  total_ppr = 13 * 20.409 * 4 = 1061.268

Example

- For a target speed of 120 rpm, the expected pulse count in a 20 ms window is:

  pulse ≈ 120 / 60 * 1061.268 / 50 ≈ 42

  If detected pulse = 42, then `pulse_to_rpm` yields:

  rpm ≈ 42 * 50 / 1061.268 * 60 ≈ 118.7 rpm

Notes

- Pulse direction: the code uses signed pulse values to represent direction
  (positive/negative). The command interface accepts a `dir` parameter to set
  the target direction (sign).
- Replace the PPR values according to your encoder and gearbox. For precision,
  consult the encoder datasheet or measure it practically.
- If the signal is noisy or bouncy, consider hardware filtering or software
  debouncing, moving average or low-pass filtering to improve speed calculation
  stability.

## Usage
After the program starts, control the motors via the serial console. The finsh
command format is: motor dir spd num Parameters: direction, speed, number of
rotations. Direction (dir): 0 = forward, 1 = reverse Speed (spd): in rpm
(revolutions per minute) Number of rotations (num): total rotations; set to 0
for continuous run Examples:
- `motor 0 120 10` means forward, 120 rpm, 10 rotations
- `motor 1 180 12` means reverse, 180 rpm, 12 rotations

### Supported boards
This example can run on the following development boards:
* sf32lb52-lcd_n16r8

### Hardware reference

- Motor: MG310 with gearbox; gearbox ratio 1:20.409; no-load speed ≈ 300 rpm;
  rated voltage 7.4V.
- Encoder: Hall-effect quadrature encoder, PPR = 13.
- Driver: TB6612FNG (dual low-voltage driver, common TB6612 modules) — small
  size and simple drive; rated continuous current typically ≤ 1~2 A (higher
  peak), suitable for small gear motors.
- Others: jumper wires (Dupont), regulator/power module capable of supplying
  sufficient stall current, oscilloscope/multimeter for debugging when needed.

### Hardware connections

Development board to driver:
| Function                     |   Macro    | Pin # | Pin name |
| ---------------------------- | :--------: | :---: | :------: |
| PWM A output                 | `PWMA_PIN` |  36   |   PA36   |
| PWM B output                 | `PWMB_PIN` |  41   |   PA41   |
| AIN1 (left motor direction)  | `AIN1_PIN` |  44   |   PA44   |
| AIN2 (left motor direction)  | `AIN2_PIN` |  35   |   PA35   |
| STBY (enable)                | `STBY_PIN` |  43   |   PA43   |
| BIN1 (right motor direction) | `BIN1_PIN` |  42   |   PA42   |
| BIN2 (right motor direction) | `BIN2_PIN` |  40   |   PA40   |

Development board to encoders:
| Function   |    Macro    | Pin # | Pin name |
| ---------- | :---------: | :---: | :------: |
| Encoder1 A | `ENC1A_PIN` |  38   |   PA38   |
| Encoder1 B | `ENC1B_PIN` |  37   |   PA37   |
| Encoder2 A | `ENC2A_PIN` |  39   |   PA39   |
| Encoder2 B | `ENC2B_PIN` |  30   |   PA30   |

Driver to motors:
- Left motor: M+ to driver AO1, M- to AO2
- Right motor: M+ to driver BO2, M- to BO1

**Note**: adjust wiring and the pin macros in the code to match your actual
hardware (for example, pin macros, PWM polarity, enable/direction mapping, power
requirements, etc.).

**Note (configuration and migration guide)**: if you use different motor/driver
models, check and modify the code following these points:

- Pins and macros: update the pin macros at the top of the file (e.g.
  `PWMA_PIN`, `PWMB_PIN`, `AIN1_PIN`, `AIN2_PIN`, `STBY_PIN`, `ENC1A_PIN`,
  `ENC1B_PIN`, `ENC2A_PIN`, `ENC2B_PIN`) to match your wiring.
- PWM polarity and frequency: some drivers require specific PWM polarity
  (active-high or active-low) or frequency. If the output direction is reversed
  from expected, invert duty logic or swap high/low mappings; adjust timer PWM
  frequency if needed.
- Enable/standby and direction signals: confirm the polarity
  (active-high/active-low) of the driver enable and direction signals. If
  different, invert the corresponding control logic or pin outputs in the driver
  code.
- Power and grounding: ensure motor/driver power can supply sufficient stall
  current and share ground with the MCU. Incorrect power or missing common
  ground may cause drive failures or incorrect encoder readings.
- Hall encoder interface: Hall sensors may need pull-ups or filtering; verify
  encoder supply and MCU logic levels (3.3V vs 5V); use level shifting or
  isolation if necessary.
- Protection and cooling: check driver current rating and heat dissipation; add
  heatsinks or current-limiting protection (fuses, current sensing) if needed.

Suggested modification order:
1) Update pin macros and build to confirm no syntax errors;
2) Validate PWM and direction logic with a simple single-direction short test;
3) Connect encoders and confirm pulse counting and direction;
4) Adjust PID parameters and verify closed-loop behavior at low speed.

### Build and flash

Switch to the example `project` directory and run scons to build:
```
scons --board=sf32lb52-lcd_n16r8 -j32
```

Run the uart download batch in the matching build folder and follow the prompts
to select the port:

```
build_sf32lb52-lcd_n16r8_hcpu\uart_download.bat

UART Download

Please input the serial port number: 5
```


## Example output
motor 0 120 10 Motor command set: Direction: Forward Speed: 120 rpm Rotations:
10 msh />left speed: 113.072289 rpm, right speed: 113.072289 rpm. left speed:
118.725899 rpm, right speed: 118.725899 rpm. left speed: 118.725899 rpm, right
speed: 121.552711 rpm. left speed: 115.899094 rpm, right speed: 115.899094 rpm.
left speed: 118.725899 rpm, right speed: 115.899094 rpm. left speed: -5.653615
rpm, right speed: 0.000000 rpm. left speed: 0.000000 rpm, right speed: 0.000000
rpm.

motor 1 180 12 Motor command set: Direction: Reverse Speed: 180 rpm Rotations:
12 msh />left speed: -172.435242 rpm, right speed: -175.262039 rpm. left speed:
-178.088852 rpm, right speed: -178.088852 rpm. left speed: -175.262039 rpm,
right speed: -175.262039 rpm. left speed: -178.088852 rpm, right speed:
-180.915665 rpm. left speed: 0.000000 rpm, right speed: 0.000000 rpm. left
speed: 0.000000 rpm, right speed: 0.000000 rpm.

## Troubleshooting

If the motor does not spin or the speed is abnormal, check the following:
1. Verify power connections, voltage and current meet the motor requirements.
2. Check motor wiring, especially PWM and direction control pins.
3. Confirm encoder wiring and that A/B channels are connected to the correct
   pins.

If the problem persists after these checks, please open an
[issue](https://github.com/OpenSiFli/SiFli-SDK/issues) on GitHub.

## References
- [SiFli-SDK
  Quickstart](https://docs.sifli.com/projects/sdk/latest/sf32lb52x/quickstart/index.html)
- [SiFli-SDK Development
  Guide](https://docs.sifli.com/projects/sdk/latest/sf32lb52x/development/index.html)
