"""Moves the unreleased changes in CHANGELOG.md under a new version.

Usage: python3 etc/scripts/release_changelog.py <version> [<date>]

Renames the "Unreleased" section of the changelog to the given version and the
given date (today by default), and adds a new, empty "Unreleased" section above
it. Fails without modifying the changelog if the "Unreleased" section is empty
or if the changelog already has a section for the version.

tbump runs this script before committing a new version (see tbump.toml). When
it fails, tbump stops before committing; the files that tbump has already
modified can be restored with `git checkout -- .`.
"""

import sys
from datetime import date
from pathlib import Path

CHANGELOG = Path(__file__).resolve().parents[2] / "CHANGELOG.md"
UNRELEASED = "## [Unreleased]"


def release(text: str, version: str, release_date: str) -> str:
    """Returns the changelog with the unreleased changes moved under the given version."""
    if f"## [{version}]" in text:
        raise ValueError(f"CHANGELOG.md already has a section for {version}")

    start = text.find(UNRELEASED)
    if start < 0:
        raise ValueError(f"CHANGELOG.md has no {UNRELEASED!r} section")

    body_start = start + len(UNRELEASED)
    next_section = text.find("\n## [", body_start)
    body = text[body_start:] if next_section < 0 else text[body_start:next_section]
    if not body.strip():
        raise ValueError("the 'Unreleased' section of CHANGELOG.md is empty; there is nothing to release")

    return f"{text[:body_start]}\n\n## [{version}] - {release_date}{text[body_start:]}"


def main() -> None:
    if len(sys.argv) not in (2, 3):
        sys.exit(f"Usage: {sys.argv[0]} <version> [<date>]")

    version = sys.argv[1]
    release_date = sys.argv[2] if len(sys.argv) == 3 else date.today().isoformat()

    try:
        CHANGELOG.write_text(release(CHANGELOG.read_text(), version, release_date))
    except ValueError as ex:
        sys.exit(f"error: {ex}")

    print(f"Moved the unreleased changes in {CHANGELOG.name} under {version} ({release_date})")


if __name__ == "__main__":
    main()
