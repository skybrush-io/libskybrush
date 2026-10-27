"""Checks golden traces generated from Skybrush show files.

Usage: python3 compare_traces.py --generator <generator> --traces <dir>
           [--show-files <file>... | --standalone <name>] [--update]

For each show file, runs the trace generator and compares its output with the
golden trace of the show file (<name>.json in the traces directory), using the
tolerances in tolerances.json in the traces directory. The show files are the
*.skyb files of the traces directory unless they are given explicitly. With
--standalone, the generator does not take a show file, only the path of its
output, and its golden trace is <name>.json in the traces directory. With
--update, the golden traces are replaced with the output of the generator
instead.
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


def generate_trace(generator: Path, show_file: Optional[Path], output: Path) -> None:
    args = [str(generator)] + ([str(show_file)] if show_file else []) + [str(output)]
    result = subprocess.run(args, capture_output=True, text=True)
    if result.returncode != 0:
        sys.exit(f"{generator} failed on {show_file or 'its scenarios'}:\n{result.stderr}")


def main() -> None:
    parser = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    parser.add_argument("--generator", type=Path, required=True, help="path of the trace generator")
    parser.add_argument("--traces", type=Path, required=True, help="directory of the show files and golden traces")
    group = parser.add_mutually_exclusive_group()
    group.add_argument("--show-files", type=Path, nargs="+", help="show files to use instead of the ones in the traces directory")
    group.add_argument("--standalone", metavar="NAME", help="the generator takes no show file; its golden trace is NAME.json")
    parser.add_argument("--update", action="store_true", help="replace the golden traces with the output of the generator")
    options = parser.parse_args()

    with open(options.traces / "tolerances.json") as fp:
        tolerances = json.load(fp)

    # Each run is identified by its show file, or by None for a standalone generator
    runs: list[tuple[str, Optional[Path]]]
    if options.standalone:
        runs = [(options.standalone, None)]
    else:
        show_files = options.show_files or sorted(options.traces.glob("*.skyb"))
        if not show_files:
            sys.exit(f"no show files found in {options.traces}")
        runs = [(show_file.stem, show_file) for show_file in show_files]

    failed = False
    with tempfile.TemporaryDirectory() as tmpdir:
        for name, show_file in runs:
            golden = options.traces / f"{name}.json"
            output = Path(tmpdir) / golden.name
            generate_trace(options.generator, show_file, output)

            if options.update:
                shutil.copyfile(output, golden)
                print(f"updated {golden}")
                continue

            if not golden.exists():
                print(f"FAIL {name}: golden trace {golden.name} is missing")
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
                print(f"FAIL {name}: {len(comparator.differences)} difference(s)")
                for difference in comparator.differences[:MAX_REPORTED_DIFFERENCES]:
                    print(f"  {difference}")
            else:
                print(f"PASS {name}")

    sys.exit(1 if failed else 0)


if __name__ == "__main__":
    main()
