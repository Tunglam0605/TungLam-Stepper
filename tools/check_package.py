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

props_path = ROOT / "library.properties"
props = {}
for raw in props_path.read_text(encoding="utf-8").splitlines():
    if not raw.strip() or raw.lstrip().startswith("#"):
        continue
    if "=" not in raw:
        errors.append(f"invalid library.properties line: {raw}")
        continue
    key, value = raw.split("=", 1)
    props[key.strip()] = value.strip()

print("=== Required Arduino library metadata ===")
for key in [
    "name", "version", "author", "maintainer", "sentence",
    "paragraph", "category", "url", "architectures", "includes",
]:
    check(bool(props.get(key)), f"library.properties has {key}")

check(props.get("name") == "TungLam_Stepper", "library name is stable")
check(re.fullmatch(r"\d+\.\d+\.\d+", props.get("version", "")) is not None,
      "version is strict semver")
check(props.get("version") == "0.1.1", "development package version is 0.1.1")
check(props.get("architectures") == "avr", "architecture scope is avr")
check(props.get("includes") == "TungLam_Stepper.h",
      "Arduino auto-include header is correct")

print("\n=== Required files ===")
for rel in [
    "README.md",
    "README.en.md",
    "LICENSE",
    "CHANGELOG.md",
    "keywords.txt",
    "src/TungLam_Stepper.h",
    "src/TungLam_Stepper.cpp",
    "src/internal/TLFastPin.h",
    "src/internal/TLTimerEngine.h",
    "src/internal/TLTimerEngine.cpp",
]:
    check((ROOT / rel).is_file(), f"exists: {rel}")

print("\n=== Main example surface ===")
expected_examples = {
    "PositionTemplate",
    "LinearAxisTemplate",
    "MultiAxisTemplate",
}
example_root = ROOT / "examples"
actual_examples = {p.name for p in example_root.iterdir() if p.is_dir()}
check(actual_examples == expected_examples,
      "Arduino IDE menu contains exactly the 3 project templates")

for name in sorted(expected_examples):
    sketch = example_root / name / f"{name}.ino"
    check(sketch.is_file(), f"primary sketch matches folder: {name}/{name}.ino")

integration = (
    ROOT / "extras" / "integration-examples" /
    "ThreeLibraryRobot" / "ThreeLibraryRobot.ino"
)
check(integration.is_file(),
      "three-library integration is kept outside Arduino IDE menu")

smoke = ROOT / "extras" / "compile-tests" / "AVRGenericSmoke" / "AVRGenericSmoke.ino"
check(smoke.is_file(), "generic AVR compile smoke sketch exists")

print("\n=== Repository hygiene ===")
for generated in ["build", ".pio", ".vscode/.browse.c_cpp.db"]:
    check(not (ROOT / generated).exists(), f"no generated artifact: {generated}")

check(not (ROOT / ".development").exists(),
      "no .development marker")
check(not (ROOT / ".gitmodules").exists(),
      "no Git submodules")

exe_files = [
    str(path.relative_to(ROOT))
    for path in ROOT.rglob("*")
    if path.is_file() and path.suffix.lower() == ".exe" and ".git" not in path.parts
]
check(not exe_files, "no .exe files in repository")

symlinks = [
    str(path.relative_to(ROOT))
    for path in ROOT.rglob("*")
    if path.is_symlink() and ".git" not in path.parts
]
check(not symlinks, "no symlinks in repository")

bad = []
for path in ROOT.rglob("*"):
    if not path.is_file() or ".git" in path.parts:
        continue
    if path.stat().st_size > 2 * 1024 * 1024:
        bad.append(str(path.relative_to(ROOT)))
check(not bad, "no oversized files in library package")

text_files = []
for pattern in ["*.h", "*.cpp", "*.ino", "*.md", "*.py", "*.yml", "*.txt", "*.properties"]:
    text_files.extend(ROOT.rglob(pattern))

encoding_bad = []
for path in text_files:
    try:
        content = path.read_text(encoding="utf-8")
    except UnicodeDecodeError:
        encoding_bad.append(str(path.relative_to(ROOT)))
        continue
    if "\ufffd" in content:
        encoding_bad.append(str(path.relative_to(ROOT)))
check(not encoding_bad, "UTF-8 text files contain no replacement characters")

if errors:
    print(f"\nPackage audit: FAIL ({len(errors)} issues)")
    for err in errors:
        print(" -", err)
    sys.exit(1)

print("\nPackage audit: PASS")
