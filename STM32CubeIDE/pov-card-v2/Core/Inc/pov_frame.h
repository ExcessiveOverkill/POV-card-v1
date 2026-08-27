#ifndef POV_FRAME_H
#define POV_FRAME_H

#include <stdint.h>

/* -----------------------------------------------------------------------
 * Frame storage layout
 *
 * One frame = 128 columns × 32 rows × 2 bits/pixel = 1024 bytes.
 * Stored column-major: for column c, bytes [c*8 .. c*8+7] hold the 32
 * two-bit brightness values (row 0 in bits [1:0] of byte c*8, row 1 in
 * bits [3:2], …, row 3 in bits [7:6], then row 4 in the next byte, etc.)
 *
 * Flash layout (2 KB pages, page 0 = 0x08000000):
 *   Pages 0–27  : firmware
 *   Page  28    : user frames 3, 4   (0x0800E000)
 *   Page  29    : user frames 5, 6   (0x0800E800)
 *   Page  30    : user frames 7, 8   (0x0800F000)
 *   Page  31    : user frame  9      (0x0800F800)
 *
 * Frames 0–(POV_FRAME_RESERVED-1) are read from const arrays in firmware
 * and cannot be overwritten via USB.
 * ----------------------------------------------------------------------- */

#define POV_FRAME_COUNT       10U     /* single-digit slots: 0–9 */
#define POV_FRAME_RESERVED    3U      /* frames 0,1,2 are built-in */
#define POV_FRAME_WIDTH       128U
#define POV_FRAME_HEIGHT      32U
#define POV_FRAME_SIZE        1024U   /* bytes per stored frame */

#define POV_FRAME_FLASH_BASE  0x0800E000UL   /* first user-frame page */
#define POV_FRAME_FIRST_PAGE  28U

/* Read frame[index] into buf (POV_FRAME_SIZE bytes).
   Reserved frames return hardcoded pattern data. */
void pov_frame_read(uint8_t index, uint8_t *buf);

/* Write buf (POV_FRAME_SIZE bytes) to flash frame[index].
   Returns  0  on success.
   Returns -1  if index is out of range or reserved.
   Returns -2  on erase failure.
   Returns -3  on program failure. */
int pov_frame_write(uint8_t index, const uint8_t *buf);

/* Return 1 if the user frame slot is blank (never programmed), 0 otherwise.
   Always returns 0 for reserved frames. */
int pov_frame_is_blank(uint8_t index);

#endif /* POV_FRAME_H */
