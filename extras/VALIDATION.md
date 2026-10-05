# Validation Matrix

Validation date: **2026-10-05**

This file records the software gates used for the current `0.1.0` development baseline.

## Toolchain

- Arduino CLI: **1.5.1**
- Arduino AVR core: **1.8.8**
- AVR GCC: **7.3.0-atmel3.6.1-arduino7**
- Primary target: **Arduino Mega 2560**
- Generic AVR smoke target: **Arduino Uno**

## Static / regression gates

| Gate | Result |
|---|---|
| `python tools/check_architecture.py` | PASS |
| `python tests/test_motion_math.py` | PASS |
| `python tools/check_api_surface.py` | PASS |
| `python tools/check_package.py` | PASS |
| `git diff --check` | PASS |

The motion/safety regression covers:

- long trapezoidal profile;
- short triangular profile;
- Q24.8 recurrence precision;
- unit conversion;
- timer microsecond ceiling;
- requested-speed ceiling;
- controlled stop target clipping;
- controlled stop soft-limit clipping;
- repeated stop idempotence;
- int32 position boundaries;
- homing debounce;
- homing phase travel guard boundary;
- STEP active-HIGH / active-LOW polarity;
- A4988 microstep table;
- DRV8825 microstep table;
- soft-limit recovery;
- single-move phase-index bound;
- motion configuration phase-distance bound;
- continuous-mode progress-counter independence;
- asynchronous multi-axis scheduler synchronization.

## Arduino compile gates

All commands were compiled with `--warnings all`.

| Target | Sketch | Flash | SRAM | Result |
|---|---|---:|---:|---|
| Mega2560 | PositionTemplate | 15,022 B / 253,952 B | 461 B / 8,192 B | PASS |
| Mega2560 | LinearAxisTemplate | 15,470 B / 253,952 B | 444 B / 8,192 B | PASS |
| Mega2560 | MultiAxisTemplate | 14,540 B / 253,952 B | 1,059 B / 8,192 B | PASS |
| Uno | AVRGenericSmoke | 12,024 B / 32,256 B | 243 B / 2,048 B | PASS |
| Mega2560 | PS2 + Drive + Stepper integration | 25,014 B / 253,952 B | 785 B / 8,192 B | PASS |

The integration build uses the local matching libraries:

- `TungLam_PS2` 0.5.0 baseline
- `TungLam_OmniMecanum_4WD` 0.11.0 baseline
- `TungLam_Stepper` 0.1.0 development baseline

## CI prepared

`.github/workflows/ci.yml` is prepared to run:

1. architecture regression;
2. motion/safety regression;
3. API surface audit;
4. package audit;
5. Arduino Lint strict + Library Manager checks;
6. all three Mega2560 main examples;
7. generic AVR/Uno smoke compile;
8. pinned PS2 + Drive + Stepper integration compile.

Hosted CI history:

- Run `37286973809`: compile/regression jobs PASS; Arduino Lint exposed LD003 because the Uno smoke sketch was under `tests/arduino/`.
- The smoke sketch was moved to `extras/compile-tests/`.
- Run `37287340846`: compile/regression jobs PASS; CI configuration exposed that Arduino Lint expects `library-manager: submit` rather than a boolean.
- Run `37287557189` on commit `e2df02e`: **PASS all jobs**, including Arduino Lint in Library Manager submission mode, Mega examples, Uno smoke compile, regression suite, package audit, and the pinned three-library integration build.

## Hardware gates still required

Software validation is not a substitute for electrical/mechanical validation.

Before a hardware-stable release, verify:

- STEP pulse width/polarity with logic analyzer or oscilloscope;
- DIR setup time around direction reversals;
- active-HIGH driver wiring;
- active-LOW/common-anode sinking wiring if used;
- A4988 microstep selection;
- DRV8825 microstep selection;
- TB6600 or DM542 STEP/DIR operation;
- MIN/MAX limit behavior;
- two-pass homing repeatability;
- broken/disconnected home-switch travel guard;
- emergency stop while STEP pulse scheduling is active;
- 2–4 axes running concurrently;
- PS2 + Mecanum/Omni + Stepper on one Mega2560.

Only after those hardware gates should the library be described as hardware-stable / v1.0-ready.
