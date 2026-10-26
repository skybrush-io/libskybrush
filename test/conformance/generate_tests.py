"""Generates Unity test functions from the conformance test cases of the
Skybrush binary show file format.

Usage: python3 generate_tests.py <cases.json> <output.inc>

The conformance test cases are maintained together with the specification of
the file format. Each case becomes a test function that calls the assertion
helpers defined in test_conformance.c.
"""

import json
import re
import sys
from collections.abc import Callable, Iterable, Sequence
from typing import Any, Optional

#: Decoded JSON object of the expected results of a single block type
Expectation = dict[str, Any]

#: Decoded JSON object of the entire conformance test case corpus
Corpus = dict[str, Any]

#: A single row of a numeric table in the expected results
Row = Sequence[Any]

#: Function that emits the lines of C code that check a single block type
Emitter = Callable[[list[str], Expectation], None]

BLOCK_TYPES: tuple[str, ...] = (
    "trajectory",
    "light_program",
    "yaw_control",
    "event_list",
    "rth_plan",
    "gcs_light_control_setup",
    "metadata",
)


def c_bool(value: Any) -> str:
    return "1" if value else "0"


def c_float(value: float) -> str:
    return repr(float(value)) + "f"


def c_error(name: str) -> str:
    return "SB_" + name


def c_bytes(data: bytes) -> str:
    # C does not allow empty initializers so we always emit at least one byte;
    # the length of the case is passed separately
    values = list(data) or [0]
    return ", ".join(f"0x{b:02x}" for b in values)


def c_hex_bytes(text: str) -> str:
    return ", ".join(f"0x{b:02x}" for b in bytes.fromhex(text))


def c_rows(rows: Iterable[Row], formatter: Callable[[Any], str]) -> str:
    return ", ".join("{ " + ", ".join(formatter(v) for v in row) + " }" for row in rows)


def emit_table(
    lines: list[str],
    ctype: str,
    name: str,
    width: int,
    rows: Optional[Sequence[Row]],
    formatter: Callable[[Any], str],
) -> tuple[str, str]:
    """Emits a static two-dimensional table and returns the expressions for
    its pointer and its number of rows."""
    if not rows:
        return "NULL", "0"
    lines.append(f"    static const {ctype} {name}[][{width}] = {{ {c_rows(rows, formatter)} }};")
    return name, str(len(rows))


def emit_trajectory(lines: list[str], expect: Expectation) -> None:
    table, count = emit_table(lines, "float", "positions", 4, expect.get("positions"), c_float)
    lines.append(
        f"    expect_trajectory({c_error(expect['result'])}, "
        f"{c_bool('duration_ms' in expect)}, {expect.get('duration_ms', 0)}u, "
        f"{table}, {count});"
    )


def emit_light_program(lines: list[str], expect: Expectation) -> None:
    table, count = emit_table(lines, "int", "colors", 4, expect.get("colors"), str)
    ends = expect.get("ends_at_ms", -2)
    ends = -1 if ends is None else ends
    lines.append(f"    expect_light_program({c_error(expect['result'])}, {table}, {count}, {ends}L);")


def emit_yaw_control(lines: list[str], expect: Expectation) -> None:
    table, count = emit_table(lines, "float", "yaw", 2, expect.get("yaw_deg"), c_float)
    lines.append(
        f"    expect_yaw_control({c_error(expect['result'])}, "
        f"{c_bool('auto_yaw' in expect)}, {c_bool(expect.get('auto_yaw', False))}, "
        f"{c_bool('yaw_offset_ddeg' in expect)}, {expect.get('yaw_offset_ddeg', 0)}, "
        f"{c_bool('num_deltas' in expect)}, {expect.get('num_deltas', 0)}, "
        f"{table}, {count});"
    )


def emit_event_list(lines: list[str], expect: Expectation) -> None:
    events = expect.get("events")
    if events:
        rows = ", ".join(
            f"{{ {t}u, {etype}, {subtype}, {{ {c_hex_bytes(payload)} }} }}"
            for t, etype, subtype, payload in events
        )
        lines.append(f"    static const expected_event_t events[] = {{ {rows} }};")
        table, count = "events", str(len(events))
    else:
        table, count = "NULL", "0"
    lines.append(
        f"    expect_event_list({c_error(expect['result'])}, "
        f"{c_bool('events' in expect)}, {table}, {count});"
    )


def emit_rth_plan(lines: list[str], expect: Expectation) -> None:
    table, count = emit_table(lines, "float", "points", 2, expect.get("points_mm"), c_float)
    lines.append(
        f"    expect_rth_plan({c_error(expect['result'])}, "
        f"{c_bool('points_mm' in expect)}, {table}, {count}, "
        f"{c_bool('num_entries' in expect)}, {expect.get('num_entries', 0)});"
    )


def emit_gcs_light_control_setup(lines: list[str], expect: Expectation) -> None:
    coords = expect.get("coords", [0, 0])
    size = expect.get("matrix_size", [0, 0])
    palette = expect.get("palette")
    if palette:
        rows = ", ".join("{ " + c_hex_bytes(color) + " }" for color in palette)
        lines.append(f"    static const uint8_t palette[][3] = {{ {rows} }};")
        table, count = "palette", str(len(palette))
    else:
        table, count = "NULL", "0"
    lines.append(
        f"    expect_gcs_light_control_setup({c_error(expect['result'])}, "
        f"{c_bool('coords' in expect)}, {coords[0]}, {coords[1]}, "
        f"{c_bool('matrix_size' in expect)}, {size[0]}, {size[1]}, "
        f"{c_bool('palette' in expect)}, {table}, {count});"
    )


def emit_metadata(lines: list[str], expect: Expectation) -> None:
    show_id = expect.get("show_id", "00000000")
    lines.append(f"    static const uint8_t show_id[] = {{ {c_hex_bytes(show_id)} }};")
    lines.append(
        f"    expect_metadata({c_error(expect['result'])}, "
        f"{c_bool('show_id' in expect)}, show_id, "
        f"{c_bool('drone_index' in expect)}, {expect.get('drone_index', 0)}u);"
    )


EMITTERS: dict[str, Emitter] = {
    "trajectory": emit_trajectory,
    "light_program": emit_light_program,
    "yaw_control": emit_yaw_control,
    "event_list": emit_event_list,
    "rth_plan": emit_rth_plan,
    "gcs_light_control_setup": emit_gcs_light_control_setup,
    "metadata": emit_metadata,
}


def main() -> None:
    input_path, output_path = sys.argv[1], sys.argv[2]
    with open(input_path) as fp:
        corpus: Corpus = json.load(fp)

    if corpus.get("version") != 1:
        sys.exit(f"unsupported conformance case format version: {corpus.get('version')}")

    lines: list[str] = [
        "/* Generated by test/conformance/generate_tests.py from the conformance",
        " * test cases of the Skybrush binary show file format. Do not edit. */",
        "",
        f"#define POSITION_TOLERANCE_MM {c_float(corpus['tolerances']['position_mm'])}",
        f"#define YAW_TOLERANCE_DEG {c_float(corpus['tolerances']['yaw_deg'])}",
        "",
    ]
    names: list[str] = []

    for case in corpus["cases"]:
        name = "test_" + re.sub(r"[^a-z0-9]", "_", case["id"])
        data = bytes.fromhex(case["bytes"])
        expect = case["expect"]
        names.append(name)

        lines.append(f"/* {case['id']}: {case['description']} */")
        lines.append(f"static uint8_t {name}_bytes[] = {{ {c_bytes(data)} }};")
        lines.append(f"static void {name}(void)")
        lines.append("{")
        lines.append(f"    begin_case({name}_bytes, {len(data)});")
        lines.append(f"    expect_file({c_error(expect['file'])});")
        if expect["file"] == "SUCCESS":
            for block_type in BLOCK_TYPES:
                lines.append("    {")
                block_lines: list[str] = []
                EMITTERS[block_type](block_lines, expect.get(block_type, {"result": "ENOENT"}))
                lines.extend("    " + line for line in block_lines)
                lines.append("    }")
        lines.append("    end_case();")
        lines.append("}")
        lines.append("")

    lines.append("#define RUN_CONFORMANCE_TESTS() \\")
    lines.append("    do { \\")
    for name in names:
        lines.append(f"        RUN_TEST({name}); \\")
    lines.append("    } while (0)")
    lines.append("")

    with open(output_path, "w") as fp:
        fp.write("\n".join(lines))


if __name__ == "__main__":
    main()
