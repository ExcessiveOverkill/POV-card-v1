#include "pov_bmp.h"
#include <string.h>

/* -----------------------------------------------------------------------
 * Direct C port of tools/bmp_convert.py. Given the same BMP it produces
 * byte-identical packed column data + equivalent Image_Metadata.
 * ----------------------------------------------------------------------- */

/* Per-image timing -- keep in sync with tools/bmp_convert.py */
#define SINGLE_DISPLAY_CYCLES     15U
#define SINGLE_CYCLE_COUNT         1U
#define SEQUENCE_DISPLAY_CYCLES    2U
#define SEQUENCE_CYCLE_COUNT       3U

/* A single frame (one display) can be at most this many columns wide --
   Image_Metadata.frame_columns[] entries top out here regardless. A
   *sequence* BMP's total width is the sum of its frames plus one separator
   column between each, so it is checked per-frame below rather than
   against a total-width bound. */
#define POV_BMP_MAX_FRAME_WIDTH   128U

/* BMP header field offsets (all values little-endian in the file) */
#define BMP_OFF_PIXEL_OFFSET  10U   /* uint32: byte offset to pixel data      */
#define BMP_OFF_DIB_SIZE      14U   /* uint32: DIB (info) header size         */
#define BMP_OFF_WIDTH         18U   /* int32:  image width                    */
#define BMP_OFF_HEIGHT        22U   /* int32:  image height (<0 = top-down)   */
#define BMP_OFF_BPP           28U   /* uint16: bits per pixel                 */
#define BMP_OFF_COMPRESSION   30U   /* uint32: compression method            */
#define BMP_OFF_CLR_USED      46U   /* uint32: palette entries used (0 = 2^bpp) */
#define BMP_MIN_HEADER        54U   /* BITMAPFILEHEADER + BITMAPINFOHEADER    */

static uint32_t rd_u32(const uint8_t *p) {
    return (uint32_t)p[0] | ((uint32_t)p[1] << 8)
         | ((uint32_t)p[2] << 16) | ((uint32_t)p[3] << 24);
}
static int32_t  rd_s32(const uint8_t *p) { return (int32_t)rd_u32(p); }
static uint16_t rd_u16(const uint8_t *p) {
    return (uint16_t)((uint16_t)p[0] | ((uint16_t)p[1] << 8));
}

typedef struct {
    const uint8_t *pix;        /* first byte of pixel data                 */
    uint32_t       row_pitch;  /* bytes per pixel row (4-byte aligned)     */
    int32_t        height;     /* signed: < 0 means rows stored top-down   */
    uint8_t        bpp;        /* 1 or 4                                   */
    uint8_t        luma[16];   /* palette index -> 0..255 grey (PIL 'L')   */
} bmp_ctx_t;

/* PIL-style luminance of a BGRA palette entry. */
static uint8_t pal_luma(const uint8_t *entry) {
    uint32_t b = entry[0], g = entry[1], r = entry[2];
    return (uint8_t)((r * 299U + g * 587U + b * 114U + 500U) / 1000U);
}

/* Palette index of pixel (col, file_row); file_row 0 = first row in the file. */
static uint32_t raw_index(const bmp_ctx_t *c, uint32_t col, uint32_t file_row) {
    const uint8_t *row = c->pix + file_row * c->row_pitch;
    if (c->bpp == 1U)
        return (uint32_t)((row[col >> 3] >> (7U - (col & 7U))) & 1U);
    /* 4 bpp: high nibble is the even (left) pixel */
    uint8_t byte = row[col >> 1];
    return (col & 1U) ? (uint32_t)(byte & 0x0FU) : (uint32_t)(byte >> 4);
}

/* Grey value of pixel (col, td_row) with td_row 0 = TOP of the image, matching
   PIL's orientation-normalised .convert('L'). */
static uint8_t img_luma(const bmp_ctx_t *c, uint32_t col, uint32_t td_row) {
    uint32_t h = (uint32_t)((c->height < 0) ? -c->height : c->height);
    uint32_t file_row = (c->height > 0) ? (h - 1U - td_row) : td_row;
    return c->luma[raw_index(c, col, file_row)];
}

/* round(grey * 15 / 255). grey*15/255 is never exactly x.5 for grey in 0..255,
   so plain round-half-up is also round-half-to-even here (matches Python round). */
static uint8_t quant4(uint8_t grey) {
    return (uint8_t)(((uint32_t)grey * 15U + 127U) / 255U);
}

/* A separator column: every adjacent vertical pixel pair differs. */
static int is_separator_col(const bmp_ctx_t *c, uint32_t col, uint32_t h) {
    for (uint32_t y = 0; y + 1U < h; y++)
        if (img_luma(c, col, y) == img_luma(c, col, y + 1U))
            return 0;
    return 1;
}

/* Append packed bytes for one column to out[used..]; return count or -1 on overflow. */
static int32_t pack_column(const bmp_ctx_t *c, uint32_t col, uint32_t h,
                           uint8_t *out, uint16_t out_cap, uint16_t used) {
    if (c->bpp == 1U) {
        uint32_t nb = h / 8U;
        if ((uint32_t)used + nb > out_cap) return -1;
        for (uint32_t bi = 0; bi < nb; bi++) {
            uint8_t b = 0;
            for (uint32_t bit = 0; bit < 8U; bit++) {
                uint32_t td_row = h - 1U - (bi * 8U + bit);
                if (img_luma(c, col, td_row) >= 128U) b |= (uint8_t)(1U << bit);
            }
            out[used + bi] = b;
        }
        return (int32_t)nb;
    } else {
        /* 2 px/byte, h/2 bytes. Vertically flipped like the 1-bit path:
           low nibble of byte 0 = bottom image row. */
        uint32_t nb = h / 2U;
        if ((uint32_t)used + nb > out_cap) return -1;
        for (uint32_t bi = 0; bi < nb; bi++) {
            uint32_t lo_row = h - 1U - (bi * 2U);
            uint32_t hi_row = h - 1U - (bi * 2U + 1U);
            uint8_t lo = quant4(img_luma(c, col, lo_row));
            uint8_t hi = (hi_row < h) ? quant4(img_luma(c, col, hi_row)) : 0U;
            out[used + bi] = (uint8_t)((hi << 4) | lo);
        }
        return (int32_t)nb;
    }
}

bmp_result_t pov_bmp_convert(const uint8_t *bmp, uint32_t bmp_len,
                             uint8_t *out, uint16_t out_cap,
                             uint8_t is_sequence, Image_Metadata *meta)
{
    if (bmp_len < BMP_MIN_HEADER)            return BMP_ERR_TOO_SMALL;
    if (bmp[0] != 'B' || bmp[1] != 'M')      return BMP_ERR_SIGNATURE;

    uint32_t data_off = rd_u32(bmp + BMP_OFF_PIXEL_OFFSET);
    uint32_t dib_size = rd_u32(bmp + BMP_OFF_DIB_SIZE);
    int32_t  width    = rd_s32(bmp + BMP_OFF_WIDTH);
    int32_t  height   = rd_s32(bmp + BMP_OFF_HEIGHT);
    uint16_t bpp      = rd_u16(bmp + BMP_OFF_BPP);
    uint32_t comp     = rd_u32(bmp + BMP_OFF_COMPRESSION);
    uint32_t clr_used = rd_u32(bmp + BMP_OFF_CLR_USED);

    if (bpp != 1U && bpp != 4U)              return BMP_ERR_BPP;

    int32_t h = (height < 0) ? -height : height;
    if (h != 8 && h != 16 && h != 32)        return BMP_ERR_HEIGHT;
    if (comp != 0U)                          return BMP_ERR_COMPRESSION;
    /* Total width sanity bound only -- the largest a legitimate sequence can
       be is POV_IMAGE_MAX_FRAMES frames of POV_BMP_MAX_FRAME_WIDTH columns
       each plus a separator between every pair. Each frame's own width is
       checked individually below; a single (non-sequence) image is exactly
       one frame, so that same per-frame check covers it too. */
    if (width < 1 ||
        width > (int32_t)(POV_IMAGE_MAX_FRAMES * (POV_BMP_MAX_FRAME_WIDTH + 1U)))
        return BMP_ERR_WIDTH;

    uint32_t w         = (uint32_t)width;
    uint32_t row_pitch = (((w * bpp) + 31U) / 32U) * 4U;
    uint32_t pal_off   = 14U + dib_size;
    uint32_t pal_count = (clr_used != 0U) ? clr_used : (1U << bpp);
    if (pal_count > 16U) pal_count = 16U;

    if (pal_off + pal_count * 4U > bmp_len)          return BMP_ERR_TOO_SMALL;
    if (data_off + row_pitch * (uint32_t)h > bmp_len) return BMP_ERR_DATA_OOB;

    bmp_ctx_t c;
    c.pix       = bmp + data_off;
    c.row_pitch = row_pitch;
    c.height    = height;
    c.bpp       = (uint8_t)bpp;
    for (uint32_t i = 0; i < 16U; i++)
        c.luma[i] = (i < pal_count) ? pal_luma(bmp + pal_off + i * 4U) : 0U;

    /* ---- frame spans ----
       Column positions, not widths -- a sequence's total width (sum of all
       frames plus their separators) can exceed 255, so these must be wider
       than uint8_t even though each individual frame is capped at
       POV_BMP_MAX_FRAME_WIDTH (128) below. */
    uint16_t starts[POV_IMAGE_MAX_FRAMES];
    uint16_t ends[POV_IMAGE_MAX_FRAMES];
    uint32_t nframes = 0;

    if (is_sequence) {
        uint32_t start = 0;
        for (uint32_t x = 0; x < w; x++) {
            if (is_separator_col(&c, x, (uint32_t)h)) {
                if (x > start) {
                    if (nframes >= POV_IMAGE_MAX_FRAMES) return BMP_ERR_TOO_MANY_FRAMES;
                    starts[nframes] = (uint16_t)start;
                    ends[nframes]   = (uint16_t)x;
                    nframes++;
                }
                start = x + 1U;
            }
        }
        if (start < w) {
            if (nframes >= POV_IMAGE_MAX_FRAMES) return BMP_ERR_TOO_MANY_FRAMES;
            starts[nframes] = (uint16_t)start;
            ends[nframes]   = (uint16_t)w;
            nframes++;
        }
        if (nframes == 0) return BMP_ERR_TOO_MANY_FRAMES;   /* every column a separator */
    } else {
        starts[0] = 0;
        ends[0]   = (uint16_t)w;
        nframes   = 1;
    }

    /* ---- pack ---- */
    memset(meta->frame_columns, 0, sizeof meta->frame_columns);
    uint16_t used  = 0;
    uint8_t  max_w = 0;
    for (uint32_t f = 0; f < nframes; f++) {
        uint32_t fw = (uint32_t)(ends[f] - starts[f]);
        if (fw > POV_BMP_MAX_FRAME_WIDTH) return BMP_ERR_FRAME_WIDTH;
        meta->frame_columns[f] = (uint8_t)fw;
        if (fw > max_w) max_w = (uint8_t)fw;
        for (uint32_t x = starts[f]; x < ends[f]; x++) {
            int32_t n = pack_column(&c, x, (uint32_t)h, out, out_cap, used);
            if (n < 0) return BMP_ERR_OUTPUT_FULL;
            used = (uint16_t)(used + n);
        }
    }

    meta->display_cycles    = is_sequence ? SEQUENCE_DISPLAY_CYCLES : SINGLE_DISPLAY_CYCLES;
    meta->cycle_count       = is_sequence ? SEQUENCE_CYCLE_COUNT    : SINGLE_CYCLE_COUNT;
    meta->column_height     = (uint8_t)h;
    meta->max_frame_columns = max_w;
    meta->frame_count       = (uint8_t)nframes;
    meta->mode              = (bpp == 1U) ? ONE_BIT : FOUR_BIT;
    meta->image_data        = 0;               /* caller sets this after storing */
    meta->image_data_length = used;

    return BMP_OK;
}

const char *pov_bmp_strerror(bmp_result_t result)
{
    switch (result) {
    case BMP_OK:                  return "OK";
    case BMP_ERR_TOO_SMALL:       return "File too small";
    case BMP_ERR_SIGNATURE:       return "Not a BMP file";
    case BMP_ERR_BPP:             return "Must be 1-bit or 4-bit";
    case BMP_ERR_COMPRESSION:     return "Must be uncompressed";
    case BMP_ERR_HEIGHT:          return "Height must be 8, 16 or 32";
    case BMP_ERR_WIDTH:           return "Bad width";
    case BMP_ERR_DATA_OOB:        return "Pixel data truncated";
    case BMP_ERR_TOO_MANY_FRAMES: return "Too many frames";
    case BMP_ERR_FRAME_WIDTH:     return "Frame wider than 128px";
    case BMP_ERR_OUTPUT_FULL:     return "Image too large";
    default:                      return "Unknown error";
    }
}
