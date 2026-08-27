#include "pov_bmp.h"
#include <string.h>

/* -----------------------------------------------------------------------
 * BMP file / info header field offsets (all values little-endian in file)
 * ----------------------------------------------------------------------- */
#define BMP_OFF_SIGNATURE    0U    /* 2 bytes: 'B','M'              */
#define BMP_OFF_PIXEL_OFFSET 10U   /* uint32: byte offset to pixels */
#define BMP_OFF_INFO_SIZE    14U   /* uint32: info header size      */
#define BMP_OFF_WIDTH        18U   /* int32:  image width in pixels */
#define BMP_OFF_HEIGHT       22U   /* int32:  image height          */
#define BMP_OFF_BPP          28U   /* uint16: bits per pixel        */
#define BMP_OFF_COMPRESSION  30U   /* uint32: compression type      */

#define BMP_MIN_HEADER       54U   /* BITMAPFILEHEADER + BITMAPINFOHEADER */

/* -----------------------------------------------------------------------
 * Internal helpers
 * ----------------------------------------------------------------------- */

static uint32_t read_u32(const uint8_t *p) {
    return (uint32_t)p[0]
         | ((uint32_t)p[1] << 8)
         | ((uint32_t)p[2] << 16)
         | ((uint32_t)p[3] << 24);
}

static int32_t read_s32(const uint8_t *p) {
    return (int32_t)read_u32(p);
}

static uint16_t read_u16(const uint8_t *p) {
    return (uint16_t)((uint16_t)p[0] | ((uint16_t)p[1] << 8));
}

/* Return the 4-bit palette index for pixel (col, row) where row 0 is the
   TOP of the image (after correcting for BMP's bottom-up storage). */
static uint8_t get_pixel(const uint8_t *pixel_data, uint32_t row_pitch,
                          int32_t bmp_height, uint32_t col, uint32_t row)
{
    /* BMP row 0 in the file = bottom of image when height > 0. */
    uint32_t bmp_row = (bmp_height > 0)
                       ? ((uint32_t)bmp_height - 1U - row)
                       : row;

    const uint8_t byte = pixel_data[bmp_row * row_pitch + col / 2U];
    /* High nibble = left pixel (even col), low nibble = right pixel (odd). */
    return (col & 1U) ? (byte & 0x0FU) : (byte >> 4);
}

/* -----------------------------------------------------------------------
 * Public API
 * ----------------------------------------------------------------------- */

bmp_result_t pov_bmp_convert(const uint8_t *bmp_data, uint32_t bmp_len,
                              uint8_t out[POV_FRAME_SIZE])
{
    if (bmp_len < BMP_MIN_HEADER)
        return BMP_ERR_TOO_SMALL;

    /* Signature check. */
    if ((bmp_data[0] != 'B' && bmp_data[0] != 'b') || (bmp_data[1] != 'M' && bmp_data[1] != 'm'))
        return BMP_ERR_SIGNATURE;

    /* Geometry. */
    int32_t  width  = read_s32(bmp_data + BMP_OFF_WIDTH);
    int32_t  height = read_s32(bmp_data + BMP_OFF_HEIGHT);
    uint16_t bpp    = read_u16(bmp_data + BMP_OFF_BPP);
    uint32_t comp   = read_u32(bmp_data + BMP_OFF_COMPRESSION);

    if (width != (int32_t)POV_FRAME_WIDTH)
        return BMP_ERR_WRONG_WIDTH;

    /* Accept both bottom-up (positive height) and top-down (negative). */
    int32_t abs_height = (height < 0) ? -height : height;
    if (abs_height != (int32_t)POV_FRAME_HEIGHT)
        return BMP_ERR_WRONG_HEIGHT;

    if (bpp != 4U)
        return BMP_ERR_WRONG_BPP;

    if (comp != 0U)   /* require BI_RGB (uncompressed) */
        return BMP_ERR_COMPRESSION;

    /* Pixel data location. */
    uint32_t pixel_offset = read_u32(bmp_data + BMP_OFF_PIXEL_OFFSET);
    /* Row pitch for 4bpp: each row is (width/2) bytes, padded to 4-byte boundary. */
    uint32_t row_pitch    = ((POV_FRAME_WIDTH / 2U) + 3U) & ~3U;  /* = 64 bytes */
    uint32_t data_bytes   = row_pitch * (uint32_t)abs_height;

    if (pixel_offset + data_bytes > bmp_len)
        return BMP_ERR_DATA_OOB;

    const uint8_t *pixel_data = bmp_data + pixel_offset;

    /* ----------------------------------------------------------------
     * Convert to 2bpp column-major output.
     *
     * Output layout: column c occupies bytes [c*8 .. c*8+7].
     * Within those 8 bytes, row r occupies bits [(r%4)*2 .. (r%4)*2+1]
     * of byte [r/4].  Brightness = 4-bit value >> 2 → 0..3.
     * ---------------------------------------------------------------- */
    memset(out, 0, POV_FRAME_SIZE);

    for (uint32_t c = 0; c < POV_FRAME_WIDTH; c++) {
        uint8_t *col_bytes = out + c * 8U;
        for (uint32_t r = 0; r < POV_FRAME_HEIGHT; r++) {
            uint8_t px4  = get_pixel(pixel_data, row_pitch, height, c, r);
            uint8_t px2  = px4 >> 2;           /* 4 levels: 0-3 */
            uint8_t shift = (uint8_t)((r % 4U) * 2U);
            col_bytes[r / 4U] |= (uint8_t)(px2 << shift);
        }
    }

    return BMP_OK;
}

const char *pov_bmp_strerror(bmp_result_t result)
{
    switch (result) {
    case BMP_OK:               return "OK";
    case BMP_ERR_TOO_SMALL:    return "File too small";
    case BMP_ERR_SIGNATURE:    return "Not a BMP file";
    case BMP_ERR_WRONG_WIDTH:  return "Width must be 128";
    case BMP_ERR_WRONG_HEIGHT: return "Height must be 32";
    case BMP_ERR_WRONG_BPP:    return "Must be 4-bit grayscale";
    case BMP_ERR_COMPRESSION:  return "Must be uncompressed";
    case BMP_ERR_DATA_OOB:     return "Pixel data truncated";
    default:                   return "Unknown error";
    }
}
