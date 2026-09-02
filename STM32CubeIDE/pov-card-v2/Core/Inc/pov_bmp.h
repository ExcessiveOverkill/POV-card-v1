#ifndef POV_BMP_H
#define POV_BMP_H

#include <stdint.h>
#include "pov_image.h"

/* -----------------------------------------------------------------------
 * On-device BMP -> Image_Metadata converter.
 *
 * Accepts the same inputs as tools/bmp_convert.py: an uncompressed 1-bit
 * or 4-bit BMP whose height is 8, 16 or 32 px. Produces column-major
 * packed pixel data byte-identical to what bmp_convert.py emits, plus a
 * filled-in Image_Metadata describing it.
 *
 *   ONE_BIT  : h/8 bytes per column, bit 0 of byte 0 = bottom image row,
 *              set bit = LED on.
 *   FOUR_BIT : h/2 bytes per column, 2 px/byte, low nibble of byte 0 = bottom
 *              image row (flipped to match ONE_BIT); nibble 0..15 = LED brightness.
 * ----------------------------------------------------------------------- */

typedef enum {
    BMP_OK = 0,
    BMP_ERR_TOO_SMALL,        /* buffer shorter than the BMP headers      */
    BMP_ERR_SIGNATURE,        /* first two bytes are not 'B','M'          */
    BMP_ERR_BPP,              /* bits-per-pixel is not 1 or 4             */
    BMP_ERR_COMPRESSION,      /* compression != 0 (BI_RGB required)       */
    BMP_ERR_HEIGHT,           /* |height| is not 8, 16 or 32              */
    BMP_ERR_WIDTH,            /* width < 1 or > 255                       */
    BMP_ERR_DATA_OOB,         /* pixel data extends past end of buffer    */
    BMP_ERR_TOO_MANY_FRAMES,  /* sequence split produced > 128 frames     */
    BMP_ERR_OUTPUT_FULL,      /* packed columns exceed out_cap            */
} bmp_result_t;

/* Convert raw BMP bytes (bmp, bmp_len) into packed columns in out[] (capacity
 * out_cap) and fill *meta. If is_sequence is non-zero the image is split into
 * frames on alternating separator columns and SEQUENCE timing is used;
 * otherwise it is one frame with SINGLE timing.
 *
 * meta->image_data is NOT set here (the caller points it at flash after storing).
 * Returns BMP_OK on success, a BMP_ERR_* code otherwise. */
bmp_result_t pov_bmp_convert(const uint8_t *bmp, uint32_t bmp_len,
                             uint8_t *out, uint16_t out_cap,
                             uint8_t is_sequence, Image_Metadata *meta);

/* Short English description of a bmp_result_t code. */
const char *pov_bmp_strerror(bmp_result_t result);

#endif /* POV_BMP_H */
