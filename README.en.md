# TungLam_Stepper

**TungLam_Stepper** is a timer-driven STEP/DIR motion library for Arduino AVR robotics and mechanisms.

Author: **Nguyen Khac Tung Lam — Tung Lam Automation**

It is designed to work independently or alongside:

- `TungLam_PS2`
- `TungLam_OmniMecanum_4WD`

Recommended Mega2560 resource ownership:

```text
TungLam_PS2              -> Hardware SPI D50..D53
TungLam_Stepper          -> Timer1
TungLam_OmniMecanum_4WD  -> Timer3 + Timer4
```

## Highlights

- Non-blocking Timer1 Compare STEP pulse engine.
- No `delay()` / `delayMicroseconds()` pulse loops.
- Cached AVR port-register I/O on the hot path.
- Up to four independent axes by default.
- Trapezoidal position moves.
- Constant-speed continuous mode with controlled stop.
- Q24.8 fixed-point interval recurrence.
- Hard MIN/MAX limits and software limits.
- Two-pass non-blocking homing with switch confirmation, backoff and slow re-approach.
- Per-phase homing travel guard to stop safely if a switch fails.
- 32-bit position-overflow protection.
- Configurable STEP pulse width/polarity, direction setup, enable polarity and direction inversion.
- Full-step/rev, microstep, gear-ratio and travel-per-revolution models.
- Relative and absolute commands in steps, revolutions, degrees and linear travel.
- Optional A4988 / DRV8825 microstep GPIO profiles.
- Generic manual/DIP microstep model for TB6600, DM542 and other STEP/DIR drivers.
- Timer1 ownership is fail-closed and restored when the final axis calls `end()`.
- Compact PROGMEM diagnostics with `printState()`, `faultName()` and state names.

## Mega2560 baseline

| Axis | STEP | DIR | EN | MIN/HOME | MAX |
|---|---:|---:|---:|---:|---:|
| 1 | D22 | D23 | D42 | A8 | A9 |
| 2 | D24 | D25 | D43 | A10 | A11 |
| 3 | D26 | D27 | D44 | A12 | A13 |
| 4 | D28 | D29 | D45 | A14 | A15 |

These are recommended pins, not mandatory pins. The generic pin layer caches the selected AVR port register and bit mask during setup.

## Quick start

```cpp
#include <TungLam_Stepper.h>

TungLamStepper axis(22, 23, 42);

void setup() {
  if (!axis.begin()) {
    return;
  }

  axis.setMotorFullStepsPerRevolution(200);
  axis.setMicrosteps(16);

  axis.setMaxSpeed(8000);
  axis.setAcceleration(12000);
  axis.setDeceleration(12000);

  axis.moveRevolutions(1.0f);
}

void loop() {
  // No axis.run() call is required.
}
```

## Driver timing and pulse polarity

```cpp
TungLamStepperDriverConfig driver(
    true,   // EN active LOW
    false,  // DIR not inverted
    4,      // STEP active width [us]
    4,      // DIR setup [us]
    true    // STEP active HIGH; false = active LOW / sinking
);

axis.begin(driver, TungLamStepperMotionConfig());
```

The final parameter is optional, so existing four-argument configuration code remains source-compatible.

## Mechanical model

```cpp
axis.setMotorFullStepsPerRevolution(200);
axis.setMicrosteps(16);
axis.setGearRatio(1.0f);
axis.setTravelPerOutputRevolution(8.0f);  // e.g. 8 mm lead screw
```

Then:

```cpp
axis.moveTravel(50.0f);     // relative +50 mm
axis.moveToTravel(150.0f);  // absolute 150 mm
```

## Homing

```cpp
axis.attachLimits(A8, A9);

TungLamStepperHomingConfig home(
    TungLamStepperDirection::Negative,
    2000,    // fast seek pulse/s
    400,     // slow re-approach pulse/s
    100,     // backoff pulses
    0,       // home position
    3,       // consecutive confirmation samples
    160000   // max pulses per homing phase; 0 disables guard
);

axis.home(home);
```

Homing runs in the Timer1 state machine and does not block the application loop. Software limits are intentionally bypassed during homing because the absolute position is not trusted until home is found.

## Driver microstep modes

Manual/DIP drivers:

```cpp
axis.setMicrosteps(16);
```

A4988:

```cpp
axis.attachMicrostepPins(MS1, MS2, MS3);
axis.setMicrostepMode(16, TungLamMicrostepDriverProfile::A4988);
```

DRV8825:

```cpp
axis.attachMicrostepPins(MODE0, MODE1, MODE2);
axis.setMicrostepMode(32, TungLamMicrostepDriverProfile::DRV8825);
```

## Stop behavior

`stop()` is idempotent. Repeated fail-safe calls do not continuously recalculate the stopping trajectory.

For position moves it never extends the axis beyond the original target. Software limits also constrain the stop envelope.

`emergencyStop()` cancels STEP scheduling immediately without a deceleration profile.

## Diagnostics

```cpp
axis.printState(Serial);
Serial.println(TungLamStepper::faultName(axis.fault()));
```

`maximumStepRate()` reports the timing ceiling derived from the configured STEP pulse width and timer guard. It is a theoretical timing ceiling; practical throughput is lower when multiple axes and other interrupts are active.

## Timer1 ownership

The library only claims Timer1 when it still matches the Arduino AVR default PWM configuration or is unused. If another subsystem has changed Timer1 mode, compare outputs or interrupts, `begin()` fails with `Timer1Conflict`.

The original Timer1 registers are saved and restored when the last Stepper axis calls `end()`.

Do not combine this library with legacy `TungLam_Control_MotorV5::Init_Timer1()` in the same project.

## Examples

The Arduino IDE menu intentionally contains only three project-style templates:

- `PositionTemplate`
- `LinearAxisTemplate`
- `MultiAxisTemplate`

The three-library integration regression is stored under:

```text
extras/integration-examples/ThreeLibraryRobot/
```

## Validation status

Software validation includes architecture regression, motion math tests, Mega2560 compile with warnings enabled, and a combined PS2 + Drive + Stepper integration compile.

Real hardware validation is still required before a hardware-stable v1.0 release.

## Technical docs

- [Architecture](extras/ARCHITECTURE.md)
- [Mega2560 pinout](extras/PINOUT_MEGA2560.md)
- [TungLam ecosystem integration](extras/INTEGRATION_TUNGLAM.md)
- [References](extras/REFERENCES.md)
- [Validation matrix](extras/VALIDATION.md)

## License

MIT.
