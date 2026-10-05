from pathlib import Path
import re
import sys

ROOT = Path(__file__).resolve().parents[1]
errors = []

def check(cond, msg):
    if cond:
        print("PASS:", msg)
    else:
        print("FAIL:", msg)
        errors.append(msg)

def strip_comments(text):
    text = re.sub(r"/\*.*?\*/", "", text, flags=re.S)
    return re.sub(r"//.*", "", text)

header = (ROOT / "src" / "TungLam_Stepper.h").read_text(encoding="utf-8")
cpp = (ROOT / "src" / "TungLam_Stepper.cpp").read_text(encoding="utf-8")
engine_h = (ROOT / "src" / "internal" / "TLTimerEngine.h").read_text(encoding="utf-8")
engine = (ROOT / "src" / "internal" / "TLTimerEngine.cpp").read_text(encoding="utf-8")
fastpin = (ROOT / "src" / "internal" / "TLFastPin.h").read_text(encoding="utf-8")
readme = (ROOT / "README.md").read_text(encoding="utf-8")
readme_en = (ROOT / "README.en.md").read_text(encoding="utf-8")
pinout = (ROOT / "extras" / "PINOUT_MEGA2560.md").read_text(encoding="utf-8")
integration = (ROOT / "extras" / "integration-examples" / "ThreeLibraryRobot" / "ThreeLibraryRobot.ino").read_text(encoding="utf-8")
props = (ROOT / "library.properties").read_text(encoding="utf-8")

all_src = "\n".join([header, cpp, engine_h, engine, fastpin])
code_only = strip_comments(all_src)

print("=== Blocking / hot-path audit ===")
check("delayMicroseconds(" not in code_only,
      "core has no delayMicroseconds() pulse loop")
check(re.search(r"\bdelay\s*\(", code_only) is None,
      "core has no delay()")
check("digitalWrite(" not in code_only,
      "core has no digitalWrite() hot path")
check("portOutputRegister" in fastpin and "portInputRegister" in fastpin,
      "fast pin caches AVR register pointers")

step_match = re.search(
    r"void\s+TungLamStepper::stepRiseFromIsr\s*\(\)\s*\{(.*?)\n\}",
    cpp, flags=re.S)
check(step_match is not None, "STEP ISR implementation is present")
if step_match:
    step_body = strip_comments(step_match.group(1))
    check("microsecondsToTimerTicks" not in step_body,
          "STEP ISR uses cached timing, no us-to-tick division")
    check("pulseWidthTicks_" in step_body,
          "STEP ISR uses cached pulseWidthTicks_")

check("TIMER1_COMPA_vect" in engine and "TIMER1_COMPB_vect" in engine,
      "Timer1 compare A/B ISR engine exists")

print("\n=== Timer1 ownership audit ===")
check("looksArduinoDefault" in engine,
      "Timer1 claim recognizes Arduino AVR default state")
check("savedTccr1a_" in engine and "savedTccr1b_" in engine and
      "savedTimsk1_" in engine,
      "Timer1 registers are saved before claim")
check("releaseTimer1IfUnused" in engine,
      "Timer1 release path exists")
check("TCCR1A = savedTccr1a_" in engine and
      "TCCR1B = savedTccr1b_" in engine,
      "Timer1 configuration is restored after final axis")
check("Timer1Conflict" in header and "Timer1Conflict" in cpp,
      "Timer1 conflict has a public fault")
emergency_match = re.search(
    r"void\s+TungLamStepper::emergencyStop\s*\(\)\s*\{(.*?)\n\}",
    cpp, flags=re.S)
check(emergency_match is not None, "emergencyStop implementation is present")
if emergency_match:
    body = emergency_match.group(1)
    check(body.find("running_ = false") < body.find("TLTimerEngine::synchronizeNow"),
          "emergencyStop hides axis before scheduler synchronization")

print("\n=== Motion / safety audit ===")
for token, label in [
    ("currentIntervalQ8_", "Q24.8 interval recurrence"),
    ("stopping_", "idempotent controlled-stop state"),
    ("PositionOverflow", "position overflow fault"),
    ("HomingTravelExceeded", "homing travel guard fault"),
    ("homingMaxPhaseSteps_", "homing max-phase state"),
    ("nextStepOverflowsPositionFromIsr", "ISR position overflow guard"),
    ("directionSetupTicks_", "cached DIR setup timing"),
    ("pulseWidthTicks_", "cached STEP pulse timing"),
    ("maximumStepRate", "timing ceiling API"),
    ("stepActiveHigh", "configurable STEP polarity"),
]:
    check(token in all_src, label)

check("mode_ != TungLamStepperMode::Homing" in cpp and
      "nextStepViolatesSoftLimitFromIsr" in cpp,
      "soft-limit is bypassed only for homing path")
check("if (!running_ || stopping_) return;" in cpp,
      "repeated stop() calls are idempotent")
check("tlMinU32(allowedStopSteps, remaining)" in cpp,
      "controlled stop cannot extend past position target")
check("AtomicGuard lock;\n  softMin_" in cpp,
      "soft-limit 32-bit updates are atomic")
check("newMin" in cpp and "newMax" in cpp and "limitMin_ = newMin" in cpp,
      "limit reconfiguration is transactional")
check("if (current < softMin_) return directionSign_ < 0;" in cpp or
      "return !(current < softMin_ && directionSign_ > 0);" in cpp,
      "soft-limit permits recovery from outside MIN")
check("if (current > softMax_) return directionSign_ > 0;" in cpp or
      "return !(current > softMax_ && directionSign_ < 0);" in cpp,
      "soft-limit permits recovery from outside MAX")
check("mode_ != TungLamStepperMode::Continuous" in cpp and
      "++completedSteps_" in cpp,
      "continuous mode does not depend on a wrapping progress counter")
check("stepsPerSecond - 1ULL" in cpp,
      "speed-to-interval conversion uses ceil division")
check("magnitude > static_cast<uint64_t>(INT32_MAX)" in cpp,
      "single move is bounded to recurrence phase-index range")
check("2ULL * config.accelerationStepsPerSecond2" in cpp and
      "2ULL * config.decelerationStepsPerSecond2" in cpp,
      "motion validation uses 64-bit phase-distance math")
check("const uint64_t denominator = 2ULL * decel" in cpp,
      "controlled-stop distance uses 64-bit denominator")
check("step_.write(driverConfig_.stepActiveHigh)" in cpp and
      "step_.write(!driverConfig_.stepActiveHigh)" in cpp,
      "STEP active/idle polarity is applied through fast GPIO")

print("\n=== Flexible API audit ===")
for token in [
    "TungLamStepperDriverConfig",
    "TungLamStepperMotionConfig",
    "TungLamStepperHomingConfig",
    "begin(",
    "end(",
    "setMicrosteps",
    "attachMicrostepPins",
    "setMicrostepMode",
    "setGearRatio",
    "setTravelPerOutputRevolution",
    "move(",
    "moveTo(",
    "moveRevolutions",
    "moveDegrees",
    "moveTravel",
    "moveToRevolutions",
    "moveToDegrees",
    "moveToTravel",
    "revolutionsToSteps",
    "degreesToSteps",
    "travelToSteps",
    "runContinuous",
    "home(",
    "isHomed",
    "isStopping",
    "attachLimits",
    "setSoftLimits",
    "maximumStepRate",
    "printState",
    "faultName",
]:
    check(token in header, f"public API contains {token}")

check("TungLamMicrostepDriverProfile::A4988" in cpp and
      "TungLamMicrostepDriverProfile::DRV8825" in cpp,
      "A4988/DRV8825 microstep profiles exist")

print("\n=== Resource contract ===")
check("Timer1" in pinout, "Stepper owns Timer1")
check("Timer3 + Timer4" in pinout, "Drive keeps Timer3 + Timer4")
check("D50..D53" in pinout, "PS2 keeps hardware SPI D50..D53")
check("D22 / PA0" in pinout and "D29 / PA7" in pinout,
      "STEP/DIR baseline spans PORTA D22..D29")
check("D42 / PL7" in pinout and "D45 / PL4" in pinout,
      "ENA baseline uses PORTL D42..D45")
check("A8 / PK0" in pinout and "A15 / PK7" in pinout,
      "limit baseline uses PORTK A8..A15")
check("#include <TungLam_PS2.h>" in integration and
      "#include <TungLam_OmniMecanum_4WD.h>" in integration and
      "#include <TungLam_Stepper.h>" in integration,
      "three-library integration regression exists")

print("\n=== Package / docs ===")
check("version=0.1.2" in props, "development version is 0.1.2")
check("architectures=avr" in props, "platform scope is explicit: AVR")
check("hardware validation" in readme.lower() and "v1.0" in readme.lower(),
      "VI README does not claim hardware-stable v1.0")
check("hardware validation" in readme_en.lower(),
      "EN README keeps hardware validation caveat")
check("TungLamStepper" in (ROOT / "keywords.txt").read_text(encoding="utf-8"),
      "Arduino IDE keywords file exists")

print("\n=== Basic source sanity ===")
for path in [
    ROOT / "src" / "TungLam_Stepper.h",
    ROOT / "src" / "TungLam_Stepper.cpp",
    ROOT / "src" / "internal" / "TLTimerEngine.h",
    ROOT / "src" / "internal" / "TLTimerEngine.cpp",
    ROOT / "src" / "internal" / "TLFastPin.h",
]:
    text = path.read_text(encoding="utf-8")
    check(text.count("{") == text.count("}"),
          f"balanced braces: {path.name}")

check("max<" not in code_only and "min<" not in code_only,
      "no accidental templated Arduino min/max usage")

if errors:
    print(f"\nStepper architecture audit: FAIL ({len(errors)} issues)")
    sys.exit(1)

print("\nStepper architecture audit: PASS")
