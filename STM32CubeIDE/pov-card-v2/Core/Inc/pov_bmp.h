#ifndef POV_BMP_H
#define POV_BMP_H

#include <stdint.h>
#include "pov_frame.h"

/* -----------------------------------------------------------------------
 * BMP validation and conversion.
 *
 * Accepts a 4-bit grayscale BMP (128 × 32 pixels).
 * Converts it to the 2-bit-per-pixel column-major format used by
 * pov_frame_write().  The 4-bit pixel value is right-shifted by 2 to
 * produce 4 brightness levels (0–3).
 * ----------------------------------------------------------------------- */

typedef enum {
    BMP_OK                = 0,
    BMP_ERR_TOO_SMALL,        /* buffer shorter than minimum BMP header */
    BMP_ERR_SIGNATURE,        /* first two bytes are not 'B','M'         */
    BMP_ERR_WRONG_WIDTH,      /* width != POV_FRAME_WIDTH (128)          */
    BMP_ERR_WRONG_HEIGHT,     /* |height| != POV_FRAME_HEIGHT (32)       */
    BMP_ERR_WRONG_BPP,        /* bits-per-pixel != 4                     */
    BMP_ERR_COMPRESSION,      /* compression != 0 (BI_RGB required)      */
    BMP_ERR_DATA_OOB,         /* pixel data extends past end of buffer   */
} bmp_result_t;

/* Convert a raw BMP (bmp_data, bmp_len bytes) into a POV_FRAME_SIZE-byte
   column-major 2bpp frame stored in out[].
   Returns BMP_OK on success, a BMP_ERR_* code otherwise. */
bmp_result_t pov_bmp_convert(const uint8_t *bmp_data, uint32_t bmp_len,
                              uint8_t out[POV_FRAME_SIZE]);

/* Return a short English string describing a bmp_result_t error code. */
const char *pov_bmp_strerror(bmp_result_t result);

#endif /* POV_BMP_H */
