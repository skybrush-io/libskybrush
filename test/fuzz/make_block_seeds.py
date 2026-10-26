"""Extracts the blocks of show files as seed inputs of the fuzz_block target.

Each block of each show file given on the command line is written into a
separate file in the output directory. The seed consists of the type of the
block followed by its body, which is the input format of fuzz_block.

Usage: make_block_seeds.py <output directory> <show file>...
"""

import sys
from pathlib import Path


def iter_blocks(data: bytes):
    """Yields the type and the body of each block of a show file."""
    if data[:4] != b"skyb" or len(data) < 5:
        raise ValueError("not a show file")

    version = data[4]
    if version == 1:
        offset = 5
    elif version == 2:
        features = data[5]
        offset = 10 if features & 1 else 6
    else:
        raise ValueError(f"unsupported version: {version}")

    while offset + 3 <= len(data):
        block_type = data[offset]
        length = data[offset + 1] | (data[offset + 2] << 8)
        yield block_type, data[offset + 3 : offset + 3 + length]
        offset += 3 + length


def main() -> None:
    output_dir = Path(sys.argv[1])
    output_dir.mkdir(parents=True, exist_ok=True)

    for path in map(Path, sys.argv[2:]):
        try:
            blocks = list(iter_blocks(path.read_bytes()))
        except ValueError:
            continue
        for index, (block_type, body) in enumerate(blocks):
            seed = output_dir / f"{path.stem}_{index}"
            seed.write_bytes(bytes([block_type]) + body)


if __name__ == "__main__":
    main()
