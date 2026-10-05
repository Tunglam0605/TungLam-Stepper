#!/usr/bin/env python3
"""Kiểm tra public API có đủ gợi ý Doxygen tiếng Việt cho Arduino IDE."""

from pathlib import Path
import re
import sys

ROOT = Path(__file__).resolve().parents[1]
HEADER = ROOT / "src" / "TungLam_Stepper.h"


def class_public_block(text: str, class_name: str) -> str:
    m = re.search(r"class\s+" + re.escape(class_name) + r"\s*\{(.*?)\n\};", text, re.S)
    if not m:
        raise RuntimeError(f"Không tìm thấy class {class_name}")
    body = m.group(1)
    return body.split("public:", 1)[1].split("private:", 1)[0]


def declarations(block: str, class_name: str):
    pattern = re.compile(
        r"(?m)^\s*(?:[A-Za-z_][A-Za-z0-9_:<>&]*\s+)*"
        r"[A-Za-z_][A-Za-z0-9_]*\s*\([^;{}]*\)\s*(?:const)?\s*;"
    )
    for m in pattern.finditer(block):
        decl = " ".join(m.group(0).split())
        before = block[: m.start()].rstrip()
        comment = ""
        if before.endswith("*/"):
            start = before.rfind("/**")
            if start >= 0:
                comment = before[start:]
        yield decl, comment


def main() -> int:
    text = HEADER.read_text(encoding="utf-8-sig")
    errors = []
    total = 0

    for class_name in ("TungLamStepper",):
        block = class_public_block(text, class_name)
        count = 0

        for decl, comment in declarations(block, class_name):
            total += 1
            count += 1

            if "@brief" not in comment:
                errors.append(f"{class_name}: thiếu @brief: {decl}")

            args = decl[decl.find("(") + 1 : decl.rfind(")")].strip()
            if args and args != "void" and "@param" not in comment:
                errors.append(f"{class_name}: thiếu @param: {decl}")

            is_constructor = decl.startswith(class_name + "(")
            returns_void = decl.startswith("void ")
            if not is_constructor and not returns_void and "@return" not in comment:
                errors.append(f"{class_name}: thiếu @return: {decl}")

        print(f"{class_name}: {count} public functions checked")

    print(f"Total public functions: {total}")

    if errors:
        print("\nAPI documentation: FAIL")
        for error in errors:
            print(" -", error)
        return 1

    print("API documentation: PASS")
    return 0


if __name__ == "__main__":
    sys.exit(main())
