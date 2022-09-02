#!/usr/bin/env python
"""
convert objects xml-exported with pinocchio < 2.6.0
into objects xml-importables with pinocchio >= 2.6.0
then import that from multicontact-api
and re-export as binary
"""

from pathlib import Path

import multicontact_api

DEL_BOTH = [
    'c_init',
    'dc_init',
    'ddc_init',
    'L_init',
    'dL_init',
    'c_final',
    'dc_final',
    'ddc_final',
    'L_final',
    'dL_final',
]

DEL_COLS = DEL_BOTH + [
    'q_init',
    'q_final',
]

DEL_ROWS = DEL_BOTH + ['contact_points_positions']


def update_xml(f: Path) -> Path:
    prev = ""
    updated = f.parent / f"updated_{f.name}"
    with f.open() as f_in, updated.open("w") as f_out:
        for line in f_in:
            if line.strip().startswith("<rows>"):
                if prev in DEL_ROWS:
                    continue
            elif line.strip().startswith("<cols>"):
                if prev in DEL_COLS:
                    continue
            else:
                if strip := line.strip(" \t\n<>"):
                    prev = strip.split()[0]
            print(line, end="", file=f_out)
    return updated


def xml_to_bin(f: Path) -> Path:
    updated = f.parent / f"{f.stem}.cs"
    cs = multicontact_api.ContactSequence()
    cs.loadFromXML(str(f), "nimp")
    cs.saveAsBinary(str(updated))
    return updated


if __name__ == '__main__':
    for f in Path().glob("*.xml"):
        print(f"updtaing {f}...")
        try:
            new_xml = update_xml(f)
            new_bin = xml_to_bin(new_xml)
            print(f"{f} updated into {new_bin}")
        except RuntimeError as e:
            print(f"ERROR on {f}: {e}")
