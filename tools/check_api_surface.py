from pathlib import Path
import re
import sys

ROOT = Path(__file__).resolve().parents[1]
header = (ROOT / "src" / "TungLam_Stepper.h").read_text(encoding="utf-8")
cpp = (ROOT / "src" / "TungLam_Stepper.cpp").read_text(encoding="utf-8")

errors = []

def check(cond, msg):
    if cond:
        print("PASS:", msg)
    else:
        print("FAIL:", msg)
        errors.append(msg)

methods = [
    "begin", "end", "enable", "disable", "enabled",
    "setAutoEnable", "autoEnable",
    "setDriverConfig", "driverConfig",
    "setMotionConfig", "motionConfig",
    "setMaxSpeed", "setAcceleration", "setDeceleration",
    "setMotorFullStepsPerRevolution", "setMicrosteps",
    "attachMicrostepPins", "detachMicrostepPins", "microstepPinsAttached",
    "setMicrostepPinLevels", "setMicrostepMode",
    "setGearRatio", "setTravelPerOutputRevolution",
    "motorFullStepsPerRevolution", "microsteps", "gearRatio",
    "travelPerOutputRevolution", "pulsesPerOutputRevolution",
    "move", "moveTo", "moveRevolutions", "moveDegrees", "moveTravel",
    "moveToRevolutions", "moveToDegrees", "moveToTravel",
    "revolutionsToSteps", "degreesToSteps", "travelToSteps",
    "stepsToRevolutions", "stepsToDegrees", "stepsToTravel",
    "runContinuous", "home", "isHoming", "isHomed", "homingState",
    "clearHomed", "stop", "emergencyStop", "isRunning",
    "isMovingToPosition", "isStopping", "mode",
    "position", "targetPosition", "setCurrentPosition", "distanceToGo",
    "positionRevolutions", "positionDegrees", "positionTravel",
    "attachLimits", "detachLimits", "minLimitActive", "maxLimitActive",
    "setSoftLimits", "clearSoftLimits", "softLimitsEnabled",
    "fault", "clearFault", "faultName", "modeName", "homingStateName",
    "printState", "stepPin", "dirPin", "enablePin", "engineSlot",
    "maximumStepRate", "maxAxes", "timerFrequencyHz",
]

print("=== Public API definition audit ===")
for method in methods:
    declared = re.search(r"\b" + re.escape(method) + r"\s*\(", header) is not None
    defined = re.search(
        r"\bTungLamStepper::" + re.escape(method) + r"\s*\(",
        cpp
    ) is not None
    check(declared, f"declared: {method}")
    check(defined, f"defined: {method}")

print("\n=== Public enum diagnostic audit ===")
for fault in [
    "None", "InvalidPin", "NoAxisSlot", "Timer1Conflict",
    "InvalidConfig", "SoftLimit", "MinLimit", "MaxLimit",
    "PositionOverflow", "HomingTravelExceeded",
]:
    check(f"TungLamStepperFault::{fault}" in cpp,
          f"faultName handles {fault}")

for mode in ["Idle", "Position", "Continuous", "Homing"]:
    check(f"TungLamStepperMode::{mode}" in cpp,
          f"modeName handles {mode}")

for state in ["Idle", "SeekFast", "Backoff", "SeekSlow", "Complete"]:
    check(f"TungLamStepperHomingState::{state}" in cpp,
          f"homingStateName handles {state}")

if errors:
    print(f"\nAPI surface audit: FAIL ({len(errors)} issues)")
    sys.exit(1)

print("\nAPI surface audit: PASS")
