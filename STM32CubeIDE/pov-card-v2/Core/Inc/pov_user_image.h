#ifndef POV_USER_IMAGE_H
#define POV_USER_IMAGE_H

#include <stdint.h>
#include "pov_image.h"

/* -----------------------------------------------------------------------
 * User-uploaded image storage.
 *
 * Two slots, one 2 KB flash page each, inside the linker's
 * FLASH_USER_IMG_DATA region (symbol _suser_image_data). Metadata for the
 * slots lives in Nv_Metadata.user_images[] and is persisted separately by
 * update_metadata() in main.c.
 * ----------------------------------------------------------------------- */

#define POV_USER_IMAGE_SLOTS       2U
#define POV_USER_IMAGE_SLOT_BYTES  2048U   /* one 2 KB flash page per slot */

/* Erase slot's flash page and program `len` bytes of packed column data into it,
 * then set meta->image_data to that flash address.
 *   slot : 0 or 1
 *   len  : <= POV_USER_IMAGE_SLOT_BYTES
 * Returns 0 on success, -1 bad slot/len, -2 erase failed, -3 program failed.
 * The caller persists *meta into Nv_Metadata (e.g. via update_metadata()). */
int pov_user_image_write(uint8_t slot, Image_Metadata *meta,
                         const uint8_t *data, uint16_t len);

#endif /* POV_USER_IMAGE_H */
