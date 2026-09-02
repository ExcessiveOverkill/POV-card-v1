#include "pov_user_image.h"
#include "main.h"          /* HAL flash API, FLASH_BASE, FLASH_PAGE_SIZE */
#include <string.h>

/* Start of the FLASH_USER_IMG_DATA region (STM32C071G8UX_FLASH.ld). */
extern uint32_t _suser_image_data;

int pov_user_image_write(uint8_t slot, Image_Metadata *meta,
                         const uint8_t *data, uint16_t len)
{
    if (slot >= POV_USER_IMAGE_SLOTS)     return -1;
    if (len  > POV_USER_IMAGE_SLOT_BYTES) return -1;

    uint32_t slot_base = (uint32_t)&_suser_image_data
                       + (uint32_t)slot * POV_USER_IMAGE_SLOT_BYTES;
    uint32_t page = (slot_base - FLASH_BASE) / FLASH_PAGE_SIZE;

    HAL_FLASH_Unlock();

    FLASH_EraseInitTypeDef erase = {
        .TypeErase = FLASH_TYPEERASE_PAGES,
        .Page      = page,
        .NbPages   = 1U,
    };
    uint32_t page_err = 0;
    if (HAL_FLASHEx_Erase(&erase, &page_err) != HAL_OK) {
        HAL_FLASH_Lock();
        return -2;
    }

    /* Program only the used bytes; the rest of the page stays erased (0xFF). */
    uint32_t prog_bytes = ((uint32_t)len + 7U) & ~7U;
    for (uint32_t i = 0; i < prog_bytes; i += 8U) {
        uint8_t tmp[8];
        for (uint32_t k = 0; k < 8U; k++) {
            uint32_t idx = i + k;
            tmp[k] = (idx < len) ? data[idx] : 0xFFU;
        }
        uint64_t dword;
        memcpy(&dword, tmp, 8);
        if (HAL_FLASH_Program(FLASH_TYPEPROGRAM_DOUBLEWORD,
                              slot_base + i, dword) != HAL_OK) {
            HAL_FLASH_Lock();
            return -3;
        }
    }

    HAL_FLASH_Lock();

    meta->image_data = (uint8_t *)slot_base;
    return 0;
}
