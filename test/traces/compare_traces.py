"""Checks the golden traces of the Skybrush binary show file format.

Usage: python3 compare_traces.py --generator <generate_trace> --traces <dir> [--update]

For each show file (*.skyb) in the traces directory, runs the trace generator
and compares its output with the golden trace stored next to the show file
(<name>.json), using the tolerances in tolerances.json. With --update, the
golden traces are replaced with the output of the generator instead.
"""

import argparse
import json
import math
import shutil
import subprocess
import sys
import tempfile
from collections.abc import Sequence
from pathlib import Path
from typing import Any, Optional

#: Path of a value within a trace, as a sequence of keys and indices
JsonPath = Sequence[Any]

#: Absolute and relative tolerance of a quantity
Tolerance = dict[str, float]

#: Keys that are not compared because they do not describe the behaviour
IGNORED_KEYS = frozenset({"generator"})

#: Maximum number of differences to report per trace
MAX_REPORTED_DIFFERENCES = 20


class TraceComparator:
    """Compares a trace with a golden trace and collects the differences."""

    def __init__(self, tolerances: dict[str, Any]):
        self._units: dict[str, Tolerance] = tolerances["by_unit"]
        self._default: Tolerance = tolerances["default"]
        # Longest unit suffixes first so "_mm_s2" is matched before "_mm_s"
        self._suffixes: list[str] = sorted(self._units, key=len, reverse=True)
        self.differences: list[str] = []

    def unit_of(self, key: Optional[str]) -> Optional[str]:
        if key is None:
            return None
        for unit in self._suffixes:
            if key.endswith("_" + unit):
                return unit
        return None

    def compare(self, expected: Any, actual: Any, path: JsonPath, key: Optional[str], context: Any) -> None:
        if isinstance(expected, dict) and isinstance(actual, dict):
            self._compare_objects(expected, actual, path)
        elif isinstance(expected, list) and isinstance(actual, list):
            self._compare_arrays(expected, actual, path, key, context)
        elif isinstance(expected, bool) or isinstance(actual, bool) or expected is None or actual is None:
            if expected != actual or type(expected) is not type(actual):
                self._report(path, f"expected {expected!r}, got {actual!r}")
        elif isinstance(expected, (int, float)) and isinstance(actual, (int, float)):
            self._compare_numbers(expected, actual, path, key)
        elif expected != actual:
            self._report(path, f"expected {expected!r}, got {actual!r}")

    def _compare_objects(self, expected: dict[str, Any], actual: dict[str, Any], path: JsonPath) -> None:
        expected_keys = set(expected) - IGNORED_KEYS
        actual_keys = set(actual) - IGNORED_KEYS
        for missing in sorted(expected_keys - actual_keys):
            self._report([*path, missing], "missing from the trace")
        for extra in sorted(actual_keys - expected_keys):
            self._report([*path, extra], "not present in the golden trace")
        for key in sorted(expected_keys & actual_keys):
            self.compare(expected[key], actual[key], [*path, key], key, expected)

    def _compare_arrays(self, expected: list[Any], actual: list[Any], path: JsonPath, key: Optional[str], context: Any) -> None:
        if len(expected) != len(actual):
            self._report(path, f"expected {len(expected)} items, got {len(actual)}")
            return

        # Columns of RTH plan entries are named in the "entry_fields" array
        columns: Optional[list[str]] = None
        if key == "entries" and isinstance(context, dict) and "entry_fields" in context:
            columns = context["entry_fields"]

        for index, (expected_item, actual_item) in enumerate(zip(expected, actual)):
            if columns is not None and isinstance(expected_item, list):
                for column, (e, a) in enumerate(zip(expected_item, actual_item)):
                    self.compare(e, a, [*path, index, columns[column]], columns[column], None)
                if len(expected_item) != len(actual_item):
                    self._report([*path, index], f"expected {len(expected_item)} fields, got {len(actual_item)}")
            else:
                # Items of an array inherit the key (and thus the unit) of the array
                self.compare(expected_item, actual_item, [*path, index], key, context)

    def _compare_numbers(self, expected: float, actual: float, path: JsonPath, key: Optional[str]) -> None:
        unit = self.unit_of(key)
        if unit is None and isinstance(expected, int) and isinstance(actual, int):
            if expected != actual:
                self._report(path, f"expected {expected}, got {actual}")
            return

        tolerance = self._units[unit] if unit is not None else self._default
        allowed = tolerance["abs"] + tolerance["rel"] * abs(expected)
        if not math.isclose(expected, actual, rel_tol=0, abs_tol=allowed):
            self._report(path, f"expected {expected!r}, got {actual!r} (allowed difference: {allowed:g})")

    def _report(self, path: JsonPath, message: str) -> None:
        location = "".join(f"[{item}]" if isinstance(item, int) else f".{item}" for item in path)
        self.differences.append(f"{location or '.'}: {message}")


def generate_trace(generator: Path, show_file: Path, output: Path) -> None:
    result = subprocess.run([str(generator), str(show_file), str(output)], capture_output=True, text=True)
    if result.returncode != 0:
        sys.exit(f"{generator} failed on {show_file}:\n{result.stderr}")


def main() -> None:
    parser = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    parser.add_argument("--generator", type=Path, required=True, help="path of the trace generator")
    parser.add_argument("--traces", type=Path, required=True, help="directory of the show files and golden traces")
    parser.add_argument("--update", action="store_true", help="replace the golden traces with the output of the generator")
    options = parser.parse_args()

    with open(options.traces / "tolerances.json") as fp:
        tolerances = json.load(fp)

    show_files = sorted(options.traces.glob("*.skyb"))
    if not show_files:
        sys.exit(f"no show files found in {options.traces}")

    failed = False
    with tempfile.TemporaryDirectory() as tmpdir:
        for show_file in show_files:
            golden = show_file.with_suffix(".json")
            output = Path(tmpdir) / golden.name
            generate_trace(options.generator, show_file, output)

            if options.update:
                shutil.copyfile(output, golden)
                print(f"updated {golden}")
                continue

            if not golden.exists():
                print(f"FAIL {show_file.name}: golden trace {golden.name} is missing")
                failed = True
                continue

            with open(golden) as fp:
                expected = json.load(fp)
            with open(output) as fp:
                actual = json.load(fp)

            comparator = TraceComparator(tolerances)
            comparator.compare(expected, actual, [], None, None)
            if comparator.differences:
                failed = True
                print(f"FAIL {show_file.name}: {len(comparator.differences)} difference(s)")
                for difference in comparator.differences[:MAX_REPORTED_DIFFERENCES]:
                    print(f"  {difference}")
            else:
                print(f"PASS {show_file.name}")

    sys.exit(1 if failed else 0)


if __name__ == "__main__":
    main()
