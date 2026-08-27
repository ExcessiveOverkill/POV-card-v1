#include "pov_frame.h"
#include "main.h"
#include <string.h>

/* -----------------------------------------------------------------------
 * Reserved frame data (frames 0, 1, 2).
 * Stored in flash as part of the firmware image — cannot be erased by USB.
 * Replace these arrays with actual POV pattern data.
 * Format: 1024 bytes, column-major, 2 bits per pixel.
 * ----------------------------------------------------------------------- */
static const uint8_t s_reserved_frame[POV_FRAME_RESERVED][POV_FRAME_SIZE] = {
    { 0 },   /* frame 0 — placeholder: all off */
    { 0 },   /* frame 1 — placeholder: all off */
    { 0 },   /* frame 2 — placeholder: all off */
};

/* -----------------------------------------------------------------------
 * Internal helpers
 * ----------------------------------------------------------------------- */

static uint32_t user_idx(uint8_t index)
{
    return (uint32_t)index - POV_FRAME_RESERVED;   /* 0-based for user frames */
}

static uint32_t frame_addr(uint8_t index)
{
    return POV_FRAME_FLASH_BASE + user_idx(index) * POV_FRAME_SIZE;
}

static uint32_t frame_page(uint8_t index)
{
    /* Two user frames share one 2 KB page. */
    return POV_FRAME_FIRST_PAGE + user_idx(index) / 2U;
}

/* -----------------------------------------------------------------------
 * Public API
 * ----------------------------------------------------------------------- */

void pov_frame_read(uint8_t index, uint8_t *buf)
{
    if (index < POV_FRAME_RESERVED) {
        memcpy(buf, s_reserved_frame[index], POV_FRAME_SIZE);
    } else if (index < POV_FRAME_COUNT) {
        memcpy(buf, (const void *)frame_addr(index), POV_FRAME_SIZE);
    }
}

int pov_frame_is_blank(uint8_t index)
{
    if (index < POV_FRAME_RESERVED || index >= POV_FRAME_COUNT)
        return 0;

    const uint8_t *p = (const uint8_t *)frame_addr(index);
    for (uint32_t i = 0; i < POV_FRAME_SIZE; i++) {
        if (p[i] != 0xFFU)
            return 0;
    }
    return 1;
}

int pov_frame_write(uint8_t index, const uint8_t *buf)
{
    if (index < POV_FRAME_RESERVED || index >= POV_FRAME_COUNT)
        return -1;

    uint32_t page      = frame_page(index);
    uint32_t page_base = (uint32_t)0x08000000U + page * FLASH_PAGE_SIZE;

    static uint8_t sibling_buf[POV_FRAME_SIZE];
    int     has_sibling = 0;

    uint32_t ui = user_idx(index);
    uint32_t sibling_ui = (ui % 2U == 0U) ? (ui + 1U) : (ui - 1U);
    uint8_t  sibling_index = (uint8_t)(sibling_ui + POV_FRAME_RESERVED);

    if (sibling_index < POV_FRAME_COUNT) {
        memcpy(sibling_buf,
               (const void *)(POV_FRAME_FLASH_BASE + sibling_ui * POV_FRAME_SIZE),
               POV_FRAME_SIZE);
        has_sibling = 1;
    }

    HAL_FLASH_Unlock();

    /* Erase the 2 KB page. */
    FLASH_EraseInitTypeDef erase = {
        .TypeErase = FLASH_TYPEERASE_PAGES,
        .Page      = page,
        .NbPages   = 1U,
    };
    uint32_t page_error = 0U;
    if (HAL_FLASHEx_Erase(&erase, &page_error) != HAL_OK) {
        HAL_FLASH_Lock();
        return -2;
    }

    /* Determine which half of the page each frame goes to. */
    const uint8_t *even_src = (ui % 2U == 0U) ? buf         : sibling_buf;
    const uint8_t *odd_src  = (ui % 2U == 0U) ? sibling_buf : buf;

    /* Write even frame (first 1024 bytes of page), 8 bytes at a time. */
    for (uint32_t i = 0; i < POV_FRAME_SIZE; i += 8U) {
        uint64_t dw;
        memcpy(&dw, even_src + i, 8U);
        if (HAL_FLASH_Program(FLASH_TYPEPROGRAM_DOUBLEWORD,
                              page_base + i, dw) != HAL_OK) {
            HAL_FLASH_Lock();
            return -3;
        }
    }

    /* Write odd frame (second 1024 bytes) only if there is one. */
    if (has_sibling) {
        for (uint32_t i = 0; i < POV_FRAME_SIZE; i += 8U) {
            uint64_t dw;
            memcpy(&dw, odd_src + i, 8U);
            if (HAL_FLASH_Program(FLASH_TYPEPROGRAM_DOUBLEWORD,
                                  page_base + POV_FRAME_SIZE + i, dw) != HAL_OK) {
                HAL_FLASH_Lock();
                return -3;
            }
        }
    }

    HAL_FLASH_Lock();
    return 0;
}
