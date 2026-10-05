# Changelog

## 0.1.0 - Development

### Added
- Timer1 event-driven non-blocking STEP/DIR engine.
- Cached direct AVR port access for STEP/DIR/ENA/limit pins.
- Configurable STEP/DIR/ENA driver timing and polarity.
- Independent multi-axis registration, default 4 axes.
- Position moves with trapezoidal acceleration/deceleration.
- Continuous constant-speed mode.
- Controlled stop and emergency stop.
- Hard MIN/MAX limits and software limits.
- Non-blocking two-pass homing with fast seek, confirmed switch hit, backoff, slow re-approach and zeroing.
- Consecutive-sample debounce for homing switch confirmation and switch release.
- Full-step, microstep, gear ratio and travel-per-revolution motion model.
- Optional software microstep pin profiles for A4988 and DRV8825; manual/DIP mode remains generic.
- Q24.8 fixed-point interval recurrence to avoid integer-quantization stall during acceleration.
- Scheduler elapsed-time synchronization when axes are started or stopped asynchronously.
- Mega2560 resource/pin baseline designed to coexist with TungLam PS2 and Omni/Mecanum libraries.

### Validation
- Architecture/static regression available in `tools/check_architecture.py`.
- Motion math regression available in `tests/test_motion_math.py`.
- GitHub Actions workflow prepared for Arduino Lint and Mega2560 compile.
- Hardware bench validation pending.
- Remote GitHub repository has not been created yet, so hosted CI has not run.
