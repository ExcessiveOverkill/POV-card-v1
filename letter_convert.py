from PIL import Image

INPUT_PNG = "7x5_letters.png"
OUTPUT_TXT = "font_data.txt"
CHAR_HEIGHT = 7

def is_blank_column(pixels, x, height):
    # white (255) = off, black (0) = on; blank column = all white
    return all(pixels[x, y] == 0 for y in range(height))

def encode_column(pixels, x, height, is_last):
    value = 0
    for y in range(height):
        # bottom of letter = bit 0, so row (height-1) → bit 0
        if pixels[x, y] != 0:
            bit = (height - 1) - y
            value |= (1 << bit)
    if is_last:
        value |= 0x80
    return value

def extract_letters(img):
    pixels = img.load()
    width, height = img.size

    # Split image into letter groups separated by all-black columns
    letters = []
    current = []
    i = 0
    while i < width:
        if is_blank_column(pixels, i, height):
            if current:
                letters.append(current)
                current = []
            # skip consecutive blank separator columns
            while i < width and is_blank_column(pixels, i, height):
                i += 1
        else:
            current.append(i)
            i += 1
    if current:
        letters.append(current)

    return letters, pixels, height

def letter_to_bytes(pixels, col_indices, height):
    # Trim leading and trailing blank columns within the letter
    while col_indices and is_blank_column(pixels, col_indices[0], height):
        col_indices = col_indices[1:]
    while col_indices and is_blank_column(pixels, col_indices[-1], height):
        col_indices = col_indices[:-1]

    result = []
    for i, x in enumerate(col_indices):
        is_last = (i == len(col_indices) - 1)
        result.append(encode_column(pixels, x, height, is_last))
    return result

def main():
    img = Image.open(INPUT_PNG).convert("L")  # grayscale
    # black (< 128) = on, white (>= 128) = off
    bw_img = img.point(lambda p: 255 if p < 128 else 0)
    _, height = bw_img.size

    if height != CHAR_HEIGHT:
        print(f"Warning: image height is {height}, expected {CHAR_HEIGHT}")

    letters, pixels, height = extract_letters(bw_img)
    print(f"Found {len(letters)} letters")

    all_bytes = []
    for letter_cols in letters:
        b = letter_to_bytes(pixels, letter_cols, height)
        all_bytes.append(b)

    # Build C array output
    lines = ["const uint8_t letter_pixels[] = {"]
    flat = []
    for letter_bytes in all_bytes:
        flat.extend(letter_bytes)

    # Format 8 per line
    for i in range(0, len(flat), 8):
        chunk = flat[i:i+8]
        hex_vals = ", ".join(f"0x{b:02X}" for b in chunk)
        lines.append(f"    {hex_vals},")

    lines.append("};")
    lines.append(f"\n// Total bytes: {len(flat)}")
    lines.append(f"// Letters found: {len(all_bytes)}")
    lines.append("// Bit layout per byte: bits 6-0 = column pixels (bit 0 = bottom row)")
    lines.append("// Bit 7 = 1 marks the last column of each letter")

    output = "\n".join(lines)
    with open(OUTPUT_TXT, "w") as f:
        f.write(output)

    print(f"Written to {OUTPUT_TXT}")
    print(f"Total bytes: {len(flat)}")

    # Print per-letter summary
    for i, b in enumerate(all_bytes):
        hex_str = " ".join(f"0x{x:02X}" for x in b)
        print(f"  Letter {i:2d}: {hex_str}")

if __name__ == "__main__":
    main()
