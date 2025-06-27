#!/usr/bin/env python3
"""
mirror_bt_full.py
-----------------
Mirror Eurobot BT XML from yellow side (x=0→3 m) to blue side
(x mirrored about 1.5 m), fix Parallel thresholds, and flip
offset sign for pure dockings along X.

Usage
-----
python mirror_bt_full.py yellow.xml blue.xml
python mirror_bt_full.py yellow.xml -        # stdout
"""

import argparse
import sys
import xml.etree.ElementTree as ET
from pathlib import Path

# ---------- Configurable parameters ----------
FIELD_WIDTH = 3.0
MIRROR_AXIS  = FIELD_WIDTH / 2.0
ORIENT_MAP   = {"0": "2", "2": "0", "1": "1", "3": "3"}
DECIMALS     = 3            # round x to this many decimals
# ---------------------------------------------


# ──────────────────────────────────────────────
# Helper: mirror base attribute
# ──────────────────────────────────────────────
def mirror_base_attr(base: str) -> str:
    """
    Convert 'x, y, dir' string to its mirrored counterpart.
    Leave untouched if format unexpected or placeholder.
    """
    if "{PrevGoal}" in base or "{" in base:
        return base

    parts = [p.strip() for p in base.split(",")]
    if len(parts) != 3:
        return base

    try:
        x_old = float(parts[0])
        y_part = parts[1]            # keep original formatting
        dir_old = parts[2]

        x_new = round(FIELD_WIDTH - x_old, DECIMALS)
        dir_new = ORIENT_MAP.get(dir_old, dir_old)

        return f"{x_new}, {y_part}, {dir_new}"
    except ValueError:
        return base


# ──────────────────────────────────────────────
# Helper: Pure‑docking X offset sign invert
# ──────────────────────────────────────────────
def maybe_flip_offset(elem: ET.Element) -> None:
    if (
        elem.tag == "Docking"
        and elem.get("isPureDocking") == "1"
        and "dock_x" in (elem.get("dock_type") or "")
    ):
        offset_str = elem.get("offset")
        if offset_str is None:
            return
        try:
            val = float(offset_str)
            elem.set("offset", str(-val))
        except ValueError:
            pass


# ──────────────────────────────────────────────
# Helper: clamp Parallel thresholds
# ──────────────────────────────────────────────
def fix_parallel(elem: ET.Element) -> None:
    children = [c for c in elem if isinstance(c.tag, str)]
    n = len(children)
    if n == 0:
        return

    fc_attr = elem.get("failure_count")
    sc_attr = elem.get("success_count")

    def clamp(attr_val: str | None) -> str | None:
        if attr_val is None:
            return None
        try:
            val = int(attr_val)
            return str(min(val, n))
        except ValueError:
            return attr_val

    new_fc = clamp(fc_attr)
    new_sc = clamp(sc_attr)

    if new_fc and new_fc != fc_attr:
        elem.set("failure_count", new_fc)
    if new_sc and new_sc != sc_attr:
        elem.set("success_count", new_sc)


# ──────────────────────────────────────────────
# Main processing
# ──────────────────────────────────────────────
def process_tree(input_path: Path) -> ET.ElementTree:
    tree = ET.parse(input_path)
    root = tree.getroot()

    for elem in root.iter():
        # 1) mirror base attr
        if (base_attr := elem.get("base")) is not None:
            elem.set("base", mirror_base_attr(base_attr))

        # 2) flip offset for pure dock_x
        maybe_flip_offset(elem)

    # 3) clamp Parallel thresholds
    for par in root.iter("Parallel"):
        fix_parallel(par)

    return tree


def main() -> None:
    ap = argparse.ArgumentParser(
        description="Mirror yellow‑side BT XML to blue‑side with safety fixes."
    )
    ap.add_argument("input_xml", type=Path, help="Input BT XML file (yellow side)")
    ap.add_argument(
        "output_xml",
        help='Output file path, or "-" to write to stdout (UTF‑8).',
    )
    args = ap.parse_args()

    tree = process_tree(args.input_xml)

    if args.output_xml == "-":
        tree.write(sys.stdout, encoding="unicode", xml_declaration=True)
    else:
        tree.write(args.output_xml, encoding="utf-8", xml_declaration=True)
        print(f"✔ Mirrored BT saved to {args.output_xml}")


if __name__ == "__main__":
    main()

