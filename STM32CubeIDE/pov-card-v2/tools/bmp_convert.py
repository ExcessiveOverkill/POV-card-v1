from PIL import Image
import os
import struct

# ── Configurable constants ───────────────────────────────────────────────────
SINGLE_DISPLAY_CYCLES   = 15
SINGLE_CYCLE_COUNT      = 1
SEQUENCE_DISPLAY_CYCLES = 2
SEQUENCE_CYCLE_COUNT    = 3

# List 1 or 2 BMP files to process. Paths are resolved relative to this script.
INPUT_FILES = [
    "test_sequence_1_bit.bmp",
    "EO_Logo.bmp",
    # "image2_sequence.bmp",
]

# Generated C data is written straight into the firmware include that main.c
# pulls in with `#include "eo_image_data.h"` (after Image_Metadata is defined).
SCRIPT_DIR  = os.path.dirname(os.path.abspath(__file__))
OUTPUT_HEADER = os.path.join(SCRIPT_DIR, "..", "Core", "Inc", "eo_image_data.h")
# ────────────────────────────────────────────────────────────────────────────

MAX_FRAMES = 128


def bmp_bit_depth(path):
    with open(path, 'rb') as f:
        f.seek(28)
        return struct.unpack_from('<H', f.read(2))[0]


def is_alternating_column(pixels, x, h):
    """True if every adjacent pair of pixels in the column differs."""
    for y in range(h - 1):
        if pixels[x, y] == pixels[x, y + 1]:
            return False
    return True


def find_sequence_frames(pixels, w, h):
    """Return list of (start_x, end_x) spans between alternating separator columns."""
    frames = []
    start = 0
    for x in range(w):
        if is_alternating_column(pixels, x, h):
            if x > start:
                frames.append((start, x))
            start = x + 1
    if start < w:
        frames.append((start, w))
    return frames


def encode_column_1bit(pixels, x, h):
    """Pack h pixels into h/8 bytes. bit 0 of byte 0 = bottom row of image."""
    col = []
    for byte_i in range(h // 8):
        b = 0
        for bit in range(8):
            row = h - 1 - (byte_i * 8 + bit)
            if pixels[x, row] >= 128:  # light = on
                b |= (1 << bit)
        col.append(b)
    return col


def encode_column_4bit(pixels, x, h):
    """Pack 2 pixels per byte, top pixel in high nibble, going top-to-bottom."""
    col = []
    for y in range(0, h, 2):
        lo = round(pixels[x, y] * 15 / 255) & 0x0F
        hi = round(pixels[x, y + 1] * 15 / 255) & 0x0F if y + 1 < h else 0
        col.append((hi << 4) | lo)
    return col


def process_bmp(path):
    depth = bmp_bit_depth(path)
    if depth not in (1, 4):
        raise ValueError(f"{path}: bit depth {depth} not supported (must be 1 or 4)")

    img = Image.open(path).convert('L')
    w, h = img.size

    if h not in (8, 16, 32):
        raise ValueError(f"{path}: height {h} must be 8, 16, or 32")

    pixels = img.load()
    is_seq = "sequence" in os.path.basename(path).lower()

    if is_seq:
        frames = find_sequence_frames(pixels, w, h)
        display_cycles = SEQUENCE_DISPLAY_CYCLES
        cycle_count    = SEQUENCE_CYCLE_COUNT
    else:
        frames = [(0, w)]
        display_cycles = SINGLE_DISPLAY_CYCLES
        cycle_count    = SINGLE_CYCLE_COUNT

    if len(frames) > MAX_FRAMES:
        raise ValueError(f"{path}: {len(frames)} frames exceeds maximum of {MAX_FRAMES}")

    if depth == 1:
        encode_col = encode_column_1bit
        mode = "ONE_BIT"
    else:
        encode_col = encode_column_4bit
        mode = "FOUR_BIT"

    data = []
    frame_widths = []
    for start_x, end_x in frames:
        frame_widths.append(end_x - start_x)
        for x in range(start_x, end_x):
            data.extend(encode_col(pixels, x, h))

    return {
        'data': data,
        'display_cycles': display_cycles,
        'cycle_count': cycle_count,
        'column_height': h,
        'max_frame_columns': max(frame_widths),
        'frame_columns': frame_widths,
        'frame_count': len(frames),
        'mode': mode,
        'image_data_length': len(data),
    }


def fmt_frame_columns(widths):
    """Format frame_columns[128] initializer body."""
    padded = (widths + [0] * MAX_FRAMES)[:MAX_FRAMES]
    rows = []
    for i in range(0, MAX_FRAMES, 16):
        rows.append(", ".join(str(v) for v in padded[i:i + 16]))
    inner = ",\n                ".join(rows)
    return "{\n                " + inner + "\n            }"


def fmt_metadata(meta, offset):
    return (
        "{\n"
        f"        .display_cycles = {meta['display_cycles']},\n"
        f"        .cycle_count = {meta['cycle_count']},\n"
        f"        .column_height = {meta['column_height']},\n"
        f"        .max_frame_columns = {meta['max_frame_columns']},\n"
        f"        .frame_columns = {fmt_frame_columns(meta['frame_columns'])},\n"
        f"        .frame_count = {meta['frame_count']},\n"
        f"        .mode = {meta['mode']},\n"
        f"        .image_data = (uint8_t*)&eo_image_data[{offset}],\n"
        f"        .image_data_length = {meta['image_data_length']}\n"
        "    }"
    )


def fmt_zero_metadata():
    return (
        "{\n"
        "        .display_cycles = 0,\n"
        "        .cycle_count = 0,\n"
        "        .column_height = 0,\n"
        "        .max_frame_columns = 0,\n"
        f"        .frame_columns = {fmt_frame_columns([])},\n"
        "        .frame_count = 0,\n"
        "        .mode = ONE_BIT,\n"
        "        .image_data = 0,\n"
        "        .image_data_length = 0\n"
        "    }"
    )


def main():
    if not INPUT_FILES:
        print("Error: no input files set in INPUT_FILES")
        return

    files = INPUT_FILES[:2]
    if len(INPUT_FILES) > 2:
        print("Warning: only the first 2 files will be processed")

    results = []
    for path in files:
        print(f"Processing {path} ...")
        r = process_bmp(os.path.join(SCRIPT_DIR, path))
        results.append(r)
        print(f"  mode={r['mode']}, height={r['column_height']}, "
              f"frames={r['frame_count']}, bytes={r['image_data_length']}")

    # Combine all image data sequentially
    all_data = []
    offsets = []
    for r in results:
        offsets.append(len(all_data))
        all_data.extend(r['data'])

    total = len(all_data)

    # ── Data array ──────────────────────────────────────────────────────
    rows = []
    for i in range(0, total, 16):
        chunk = all_data[i:i + 16]
        rows.append("    " + ", ".join(f"0x{b:02X}" for b in chunk) + ",")
    data_decl = f"const uint8_t eo_image_data[{total}] = {{\n" + "\n".join(rows) + "\n};"

    # ── Metadata array ───────────────────────────────────────────────────
    structs = [fmt_metadata(r, offsets[i]) for i, r in enumerate(results)]
    while len(structs) < 2:
        structs.append(fmt_zero_metadata())

    meta_decl = (
        "const Image_Metadata eo_metadata[2] = {\n"
        f"    {structs[0]},\n"
        f"    {structs[1]}\n"
        "};"
    )

    banner = (
        "/* AUTO-GENERATED by tools/bmp_convert.py -- do not edit by hand.\n"
        "   Regenerate: edit INPUT_FILES in that script and run `python bmp_convert.py`.\n"
        "   This file is #included into main.c *after* Image_Metadata / ONE_BIT are\n"
        "   defined and inside the .eo_image_data section attribute -- it is not a\n"
        "   standalone header (no include guard on purpose). */\n\n"
    )

    output = banner + data_decl + "\n\n" + meta_decl + "\n"

    with open(OUTPUT_HEADER, 'w', newline='\n') as f:
        f.write(output)

    print(f"\nWritten to {os.path.relpath(OUTPUT_HEADER)}  ({total} bytes total)")


if __name__ == "__main__":
    main()
