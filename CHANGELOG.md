# Changelog

## 0.1.0 - Development

### Added
- Timer1 event-driven non-blocking STEP/DIR engine.
- Cached direct AVR port access for STEP/DIR/ENA/limit pins.
- Configurable STEP pulse width, STEP polarity, DIR setup time, EN polarity and DIR inversion.
- Independent multi-axis registration, default 4 axes.
- Position moves with trapezoidal acceleration/deceleration.
- Continuous constant-speed mode.
- Relative and absolute motion in pulse, revolutions, degrees and user travel units.
- Controlled stop, idempotent repeated stop and emergency stop.
- Hard MIN/MAX limits and atomic software-limit updates.
- Non-blocking two-pass homing with fast seek, confirmed switch hit, backoff, slow re-approach and zeroing.
- Consecutive-sample homing debounce without driving deeper into an active home switch.
- Per-phase homing travel guard (`maxPhaseSteps`) and `HomingTravelExceeded` fault.
- Full-step, microstep, gear-ratio and travel-per-revolution motion model.
- Optional A4988 and DRV8825 software microstep-pin profiles; manual/DIP mode remains generic.
- Q24.8 fixed-point interval recurrence to avoid integer-quantization stall during acceleration.
- Scheduler elapsed-time synchronization when axes are started/stopped asynchronously.
- 32-bit position-overflow detection.
- Driver timing cache so the STEP ISR does not perform microsecond-to-tick 64-bit division.
- `maximumStepRate()` timing ceiling.
- PROGMEM fault/mode/homing names and `printState(Print&)`.
- `end()` and Timer1 register restoration when the final Stepper axis is released.
- Fail-closed Timer1 ownership: only Arduino-default or unused Timer1 state may be reclaimed.
- Mega2560 pin/resource baseline designed to coexist with TungLam PS2 and Omni/Mecanum libraries.
- Three-library integration template under `extras/integration-examples/ThreeLibraryRobot`.
- Vietnamese and English README documentation.

### Changed / hardened
- `runContinuous()` now respects configured max speed and timing ceiling.
- First STEP after any direction change respects configured DIR setup time.
- Homing intentionally bypasses soft limits until the mechanical reference is re-established.
- Controlled stop cannot extend a position move beyond its original target.
- Controlled stop is clipped by the configured soft-limit envelope.
- Mechanical-model setters reject changes while the axis is moving.
- Soft-limit 32-bit values are updated atomically on AVR.
- Position/unit conversion paths reject unsafe 32-bit overflow.
- STEP polarity supports active-HIGH and active-LOW/sinking wiring without changing the hot-path architecture.
- Limit reconfiguration is transactional and clears omitted old pins.
- Emergency stop removes the axis from pending STEP scheduling before shared-timer synchronization.
- Speed-to-interval conversion uses ceiling division so generated frequency never exceeds the requested rate.
- Continuous mode no longer depends on a lifetime uint32 progress counter.

### Validation
- `tools/check_architecture.py`: architecture/static regression.
- `tests/test_motion_math.py`: motion-math and safety regression.
- Arduino CLI 1.5.1 + `arduino:avr` 1.8.8 Mega2560 compile available locally.
- All three main examples compile with `--warnings all`.
- PS2 v0.5.0 + Omni/Mecanum v0.11.0 + Stepper integration compiles together on Mega2560.
- Generic AVR/Arduino Uno smoke compile passes with `--warnings all`.
- Package regression verifies Arduino metadata, exact example surface, UTF-8 hygiene and required files.
- GitHub Actions workflow includes architecture regression, motion regression, API/package audits, Arduino Lint, Mega compile, Uno smoke compile and pinned three-library integration compile.
- Exact local compile sizes and hardware-pending gates are recorded in `extras/VALIDATION.md`.
- Hardware bench validation is still pending before a hardware-stable release.
- Remote GitHub repository has not been created yet, so hosted CI has not run.
