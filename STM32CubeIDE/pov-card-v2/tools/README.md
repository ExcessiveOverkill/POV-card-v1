# POV-card-v2 asset tools

Offline code generators that bake image/font data into the firmware. They are **not**
part of the build — run them by hand when the assets change, then rebuild in STM32CubeIDE.

## Prerequisites

```
pip install pillow
```

## bmp_convert.py — built-in POV images

Converts up to **2** BMP files into `../Core/Inc/eo_image_data.h`
(`const uint8_t eo_image_data[]` + `const Image_Metadata eo_metadata[2]`), which `main.c`
pulls in with `#include "eo_image_data.h"` and places in the `.eo_image_data` flash region.

1. Put the BMP next to this script (or give a path relative to it).
2. Edit `INPUT_FILES` at the top of `bmp_convert.py` (1 or 2 entries).
3. `python bmp_convert.py`
4. Rebuild the `Debug` configuration in STM32CubeIDE and flash.

BMP constraints (enforced by the script):

| Property | Allowed |
|---|---|
| Bit depth | 1-bit **or** 4-bit, uncompressed |
| Height | 8, 16, or 32 px |
| Width | any (≤ 128 recommended); columns become POV lines |
| Frames | filename contains `sequence` → auto-split on alternating separator columns; else 1 frame; ≤ 128 |

Per-image timing is set by the `*_DISPLAY_CYCLES` / `*_CYCLE_COUNT` constants at the top
of the script (`SINGLE_*` for normal images, `SEQUENCE_*` for `sequence` animations).

`eo_metadata` is always exactly 2 entries; a missing second image is emitted as a zeroed
(`mode = ONE_BIT`, `frame_count = 0`) placeholder.

## letter_convert.py — A–Z label font

Converts `7x5_letters.png` (a 7-px-tall A–Z strip) into `font_data.txt`, a
`const uint8_t letter_pixels[]` table. That array is currently **pasted by hand** into
`../Core/Src/main.c` (search for `letter_pixels[]`). Bit layout: bits 6–0 = column pixels
(bit 0 = bottom row); bit 7 = last column of the glyph.

```
python letter_convert.py
```
