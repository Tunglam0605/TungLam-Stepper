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

header = (ROOT / "src" / "TungLam_Stepper.h").read_text(encoding="utf-8")
cpp = (ROOT / "src" / "TungLam_Stepper.cpp").read_text(encoding="utf-8")
engine = (ROOT / "src" / "internal" / "TLTimerEngine.cpp").read_text(encoding="utf-8")
fastpin = (ROOT / "src" / "internal" / "TLFastPin.h").read_text(encoding="utf-8")
readme = (ROOT / "README.md").read_text(encoding="utf-8")
pinout = (ROOT / "extras" / "PINOUT_MEGA2560.md").read_text(encoding="utf-8")
props = (ROOT / "library.properties").read_text(encoding="utf-8")

print("=== Blocking-path audit ===")
all_src = "\n".join([header, cpp, engine, fastpin])
code_only = re.sub(r"/\*.*?\*/", "", all_src, flags=re.S)
code_only = re.sub(r"//.*", "", code_only)
check("delayMicroseconds(" not in code_only, "core has no delayMicroseconds() pulse loop")
check(re.search(r"\bdelay\s*\(", code_only) is None, "core has no delay()")
check("digitalWrite(" not in code_only, "hot-path source has no digitalWrite()")
check("portOutputRegister" in fastpin, "fast pin caches AVR output register")
check("TIMER1_COMPA_vect" in engine and "TIMER1_COMPB_vect" in engine,
      "Timer1 compare A/B ISR engine exists")
check("TungLamStepperMode::Homing" in cpp, "non-blocking homing state is implemented")
check("homingConfirmSamples_" in cpp, "homing debounce state is implemented")
check("TungLamMicrostepDriverProfile::A4988" in cpp and
      "TungLamMicrostepDriverProfile::DRV8825" in cpp,
      "A4988/DRV8825 optional microstep profiles exist")

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

print("\n=== Flexible API ===")
for token in [
    "TungLamStepperDriverConfig",
    "TungLamStepperMotionConfig",
    "setMicrosteps",
    "setGearRatio",
    "setTravelPerOutputRevolution",
    "moveRevolutions",
    "moveDegrees",
    "moveTravel",
    "attachLimits",
    "setSoftLimits",
    "runContinuous",
    "TungLamStepperHomingConfig",
    "home(",
    "isHomed",
    "attachMicrostepPins",
    "setMicrostepMode",
    "TungLamMicrostepDriverProfile",
]:
    check(token in header, f"public API contains {token}")

check("version=0.1.0" in props, "development version is 0.1.0")
check("architectures=avr" in props, "current platform scope is explicit: AVR")
check("development baseline" in readme.lower(), "README does not claim hardware stability")

print("\n=== Basic source sanity ===")
for path in [
    ROOT / "src" / "TungLam_Stepper.h",
    ROOT / "src" / "TungLam_Stepper.cpp",
    ROOT / "src" / "internal" / "TLTimerEngine.h",
    ROOT / "src" / "internal" / "TLTimerEngine.cpp",
    ROOT / "src" / "internal" / "TLFastPin.h",
]:
    text = path.read_text(encoding="utf-8")
    check(text.count("{") == text.count("}"), f"balanced braces: {path.name}")

check("max<" not in code_only and "min<" not in code_only,
      "no accidental templated Arduino min/max usage")

if errors:
    print(f"\nStepper architecture audit: FAIL ({len(errors)} issues)")
    sys.exit(1)

print("\nStepper architecture audit: PASS")
