/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file    ux_device_msc.c
  * @author  MCD Application Team
  * @brief   USBX Device applicative file
  ******************************************************************************
   * @attention
  *
  * Copyright (c) 2026 STMicroelectronics.
  * All rights reserved.
  *
  * This software is licensed under terms that can be found in the LICENSE file
  * in the root directory of this software component.
  * If no LICENSE file comes with this software, it is provided AS-IS.
  *
  ******************************************************************************
  */
/* USER CODE END Header */

/* Includes ------------------------------------------------------------------*/
#include "ux_device_msc.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include <string.h>
#include "pov_bmp.h"
#include "pov_frame.h"
extern volatile uint32_t g_usb_diag_events;
extern volatile uint8_t  g_usb_diag_actv_n;
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
#define RAM_DISK_SECTOR_SIZE    512U
#define RAM_DISK_SECTOR_COUNT   20U
#define RAM_DISK_SIZE           (RAM_DISK_SECTOR_SIZE * RAM_DISK_SECTOR_COUNT)

/* Sector numbers */
#define SECTOR_BOOT    0U
#define SECTOR_FAT1    1U
#define SECTOR_FAT2    2U
#define SECTOR_ROOTDIR 3U
#define SECTOR_DATA    4U   /* first data sector = cluster 2 */

/* Root directory entry size and offsets */
#define DIRENT_SIZE    32U
#define DIRENT_ATTR    11U
#define DIRENT_CLUSTER 26U   /* uint16 LE: first cluster */
#define DIRENT_FILESIZE 28U  /* uint32 LE: file size in bytes */

#define ATTR_READONLY  0x01U
#define ATTR_VOLUME_ID 0x08U
/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
/* USER CODE BEGIN PV */
static UCHAR ram_disk[RAM_DISK_SIZE];
static UINT  ram_disk_formatted = 0U;

/* ---- README.txt content baked onto the drive ---- */
static const char readme_text[] =
    "USB PROGRAMMING\r\n"
    "===========================\r\n"
    "Drop a BMP file onto this drive to program an image to display.\r\n"
    "\r\n"
    "File requirements:\r\n"
    "  Format  : 4-bit grayscale BMP (16 shades)\r\n"
    "  Size    : 128 x 32 pixels\r\n"
    "  Filename: must end with a digit 0-9 before .bmp\r\n"
    "            e.g.  frame3.bmp   anim7.bmp   test9.bmp\r\n"
    "\r\n"
    "Frame slots:\r\n"
    "  0, 1, 2  Reserved - built-in patterns (cannot be overwritten)\r\n"
    "  3 to 9   User-programmable\r\n"
    "\r\n"
    "LED indicators:\r\n"
    "  Slow blink  = USB ready, waiting for file\r\n"
    "  Filling up  = File received, writing to flash\r\n"
    "  All bright  = Programming successful\r\n"
    "  Fast blink  = Error (wrong format, size, or reserved slot)\r\n"
    "\r\n"
    "Unplug USB cable to exit programming mode.\r\n";

/* ---- Shared state read by main.c ---- */
volatile uint8_t  g_usb_connected  = 0U;   /* set by USBD_ChangeFunction  */
volatile uint8_t  g_bmp_pending    = 0U;   /* set when a valid .BMP lands */
volatile uint8_t  g_bmp_frame_idx  = 0U;   /* frame slot (3-9)            */
volatile uint8_t  g_bmp_fill_level = 0U;   /* 0-32 for fill animation     */
volatile uint8_t  g_usb_led_state  = 0U;   /* 0=idle 1=ready 2=prog 3=ok 4=err */
volatile uint32_t g_bmp_ram_sector = 0U;   /* first disk sector of BMP data */
volatile uint32_t g_bmp_ram_size   = 0U;   /* byte length of BMP file       */
/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
/* USER CODE BEGIN PFP */
static void ram_disk_format(void);
static void check_for_bmp(void);
/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */

/* ---- FAT12 entry helpers ---- */

/* Write a FAT12 cluster chain: set entry[cluster] to value (12-bit). */
static void fat12_set(uint8_t *fat, uint32_t cluster, uint16_t value)
{
    uint32_t byte_idx = cluster + (cluster / 2U);
    if (cluster % 2U == 0U) {
        fat[byte_idx]     = (uint8_t)(value & 0xFFU);
        fat[byte_idx + 1U] = (uint8_t)((fat[byte_idx + 1U] & 0xF0U)
                              | ((value >> 8U) & 0x0FU));
    } else {
        fat[byte_idx]     = (uint8_t)((fat[byte_idx] & 0x0FU)
                              | ((value & 0x0FU) << 4U));
        fat[byte_idx + 1U] = (uint8_t)((value >> 4U) & 0xFFU);
    }
}

/* Read a FAT12 entry (12-bit). */
static uint16_t fat12_get(const uint8_t *fat, uint32_t cluster)
{
    uint32_t byte_idx = cluster + (cluster / 2U);
    if (cluster % 2U == 0U) {
        return (uint16_t)(fat[byte_idx] | ((fat[byte_idx + 1U] & 0x0FU) << 8U));
    } else {
        return (uint16_t)((fat[byte_idx] >> 4U) | ((uint16_t)fat[byte_idx + 1U] << 4U));
    }
}

static void ram_disk_format(void)
{
    memset(ram_disk, 0, sizeof(ram_disk));

    /* ---- Boot sector (sector 0) ---- */
    uint8_t *bs = ram_disk;

    bs[0] = 0xEB; bs[1] = 0x3C; bs[2] = 0x90;
    memcpy(&bs[3], "MSWIN4.1", 8);

    bs[11] = 0x00; bs[12] = 0x02;               /* bytes/sector: 512  */
    bs[13] = 1;                                  /* sectors/cluster    */
    bs[14] = 1;    bs[15] = 0;                  /* reserved sectors   */
    bs[16] = 2;                                  /* number of FATs     */
    bs[17] = 16;   bs[18] = 0;                  /* root entry count   */
    bs[19] = (uint8_t)RAM_DISK_SECTOR_COUNT;
    bs[20] = 0;                                  /* total sectors (LE) */
    bs[21] = 0xF8;                               /* media type: fixed  */
    bs[22] = 1;    bs[23] = 0;                  /* sectors/FAT        */
    bs[24] = 1;    bs[25] = 0;                  /* sectors/track      */
    bs[26] = 1;    bs[27] = 0;                  /* number of heads    */
    /* hidden sectors [28-31], large sectors [32-35] stay 0 */
    bs[36] = 0x00;                               /* drive number       */
    bs[37] = 0x00;
    bs[38] = 0x29;                               /* extended boot sig  */
    bs[39] = 0x78; bs[40] = 0x56;
    bs[41] = 0x34; bs[42] = 0x12;               /* volume ID          */
    memcpy(&bs[43], "POV CARD   ", 11);          /* volume label       */
    memcpy(&bs[54], "FAT12   ",    8);           /* FS type string     */
    bs[510] = 0x55; bs[511] = 0xAA;

    /* ---- FAT copies ---- */
    /*  Entry 0 = 0xFF8 (media byte), entry 1 = 0xFFF (reserved).
        README.txt occupies clusters 2 and 3 (two sectors, 1024 bytes). */
    uint8_t *fat1 = ram_disk + RAM_DISK_SECTOR_SIZE * SECTOR_FAT1;
    uint8_t *fat2 = ram_disk + RAM_DISK_SECTOR_SIZE * SECTOR_FAT2;

    fat12_set(fat1, 0U, 0xFF8U);
    fat12_set(fat1, 1U, 0xFFFU);
    fat12_set(fat1, 2U, 0x003U);   /* cluster 2 → next cluster 3 */
    fat12_set(fat1, 3U, 0xFFFU);   /* cluster 3 = end of chain   */

    memcpy(fat2, fat1, RAM_DISK_SECTOR_SIZE);

    /* ---- Root directory (sector 3) ---- */
    uint8_t *root = ram_disk + RAM_DISK_SECTOR_SIZE * SECTOR_ROOTDIR;

    /* Entry 0: volume label */
    memcpy(root, "POV CARD   ", 11);
    root[DIRENT_ATTR] = ATTR_VOLUME_ID;

    /* Entry 1: README.TXT (read-only) */
    uint8_t *readme_entry = root + DIRENT_SIZE;
    memcpy(readme_entry, "README  TXT", 11);
    readme_entry[DIRENT_ATTR] = ATTR_READONLY;
    readme_entry[DIRENT_CLUSTER]     = 2U;   /* first cluster */
    readme_entry[DIRENT_CLUSTER + 1] = 0U;
    uint32_t readme_len = (uint32_t)(sizeof(readme_text) - 1U);
    readme_entry[DIRENT_FILESIZE]     = (uint8_t)(readme_len & 0xFFU);
    readme_entry[DIRENT_FILESIZE + 1] = (uint8_t)((readme_len >>  8) & 0xFFU);
    readme_entry[DIRENT_FILESIZE + 2] = (uint8_t)((readme_len >> 16) & 0xFFU);
    readme_entry[DIRENT_FILESIZE + 3] = (uint8_t)((readme_len >> 24) & 0xFFU);

    /* ---- README.TXT data: clusters 2 & 3 = sectors 4 & 5 ---- */
    uint8_t *data = ram_disk + RAM_DISK_SECTOR_SIZE * SECTOR_DATA;
    memcpy(data, readme_text, readme_len);
}

/*
 * Scan the root directory for a completed .BMP directory entry.
 * Called whenever sector 3 (root directory) is written.
 *
 * A "complete" entry has a non-zero file size and a valid starting cluster.
 * We also validate the filename ends with a digit before the '.BMP' extension.
 */
static void check_for_bmp(void)
{
    const uint8_t *root = ram_disk + RAM_DISK_SECTOR_SIZE * SECTOR_ROOTDIR;

    for (uint32_t i = 0; i < 16U; i++) {
        const uint8_t *e = root + i * DIRENT_SIZE;

        /* Skip empty / deleted / volume-label / LFN entries. */
        if (e[0] == 0x00U || e[0] == 0xE5U)
            continue;
        uint8_t attr = e[DIRENT_ATTR];
        if (attr & (ATTR_VOLUME_ID | 0x10U | 0x08U))
            continue;   /* volume ID, directory, or system */

        /* Must have .BMP or .bmp extension (bytes 8–10 in 8.3 name). */
        if ((e[8] != 'B' && e[8] != 'b') || (e[9] != 'M' && e[9] != 'm') || (e[10] != 'P' && e[10] != 'p'))
            continue;

        /* 8.3 name is space-padded; find the last non-space in bytes 0–7
           and check that it is a digit 0–9. */
        int  name_end = -1;
        for (int j = 7; j >= 0; j--) {
            if (e[j] != ' ') { name_end = j; break; }
        }
        if (name_end < 0)
            continue;

        uint8_t last_char = e[name_end];
        if (last_char < '0' || last_char > '9')
            continue;

        uint8_t frame_idx = (uint8_t)(last_char - '0');

        /* Read file size and starting cluster. */
        uint32_t fsize =
            (uint32_t)e[DIRENT_FILESIZE]
          | ((uint32_t)e[DIRENT_FILESIZE + 1] << 8)
          | ((uint32_t)e[DIRENT_FILESIZE + 2] << 16)
          | ((uint32_t)e[DIRENT_FILESIZE + 3] << 24);

        uint16_t first_cluster =
            (uint16_t)e[DIRENT_CLUSTER]
          | ((uint16_t)e[DIRENT_CLUSTER + 1] << 8);

        if (fsize == 0U || first_cluster < 2U)
            continue;

        /* Locate the BMP data in the RAM disk.
           For this small disk every file is contiguous from its first cluster. */
        uint32_t data_sector = SECTOR_DATA + (uint32_t)(first_cluster - 2U);
        if (data_sector >= RAM_DISK_SECTOR_COUNT)
            continue;
        uint32_t max_bytes = (RAM_DISK_SECTOR_COUNT - data_sector) * RAM_DISK_SECTOR_SIZE;
        if (fsize > max_bytes)
            continue;

        /* Verify the BMP magic bytes are present in the data sector.
           Windows can write the directory entry (with non-zero size and valid
           cluster) before flushing the file data, so without this check we
           would signal g_bmp_pending while the data sectors are still zeroed. */
        const uint8_t *bmp_data = ram_disk + data_sector * RAM_DISK_SECTOR_SIZE;
        if (bmp_data[0] != 'B' && bmp_data[0] != 'b')
            continue;
        if (bmp_data[1] != 'M' && bmp_data[1] != 'm')
            continue;
          

        // Signal main loop to process this BMP.
        g_bmp_frame_idx  = frame_idx;
        g_bmp_ram_sector = data_sector;
        g_bmp_ram_size   = fsize;
        g_bmp_pending    = 1U;

        return;
    }
}

/* USER CODE END 0 */

/**
  * @brief  USBD_STORAGE_Activate
  *         This function is called when insertion of a storage device.
  * @param  storage_instance: Pointer to the storage class instance.
  * @retval none
  */
VOID USBD_STORAGE_Activate(VOID *storage_instance)
{
  /* USER CODE BEGIN USBD_STORAGE_Activate */
  g_usb_diag_events |= (1U << 8);
  if (g_usb_diag_actv_n < 255U) g_usb_diag_actv_n++;
  UX_PARAMETER_NOT_USED(storage_instance);
  /* Always reformat so every connection starts with a clean, known-good disk.
     Conditional reformat was unreliable: if the host physically yanked the cable
     (vbus_sensing_enable=DISABLE means no Deactivate fires), ram_disk_formatted
     stayed 1 and the next connection served the stale/partially-written disk. */
  g_bmp_pending = 0U;     /* discard any BMP queued from a previous session */
  ram_disk_format();
  ram_disk_formatted = 1U;
  g_usb_led_state = 1U;
  /* USER CODE END USBD_STORAGE_Activate */

  return;
}

/**
  * @brief  USBD_STORAGE_Deactivate
  *         This function is called when extraction of a storage device.
  * @param  storage_instance: Pointer to the storage class instance.
  * @retval none
  */
VOID USBD_STORAGE_Deactivate(VOID *storage_instance)
{
  /* USER CODE BEGIN USBD_STORAGE_Deactivate  */
  g_usb_diag_events |= (1U << 9);
  UX_PARAMETER_NOT_USED(storage_instance);
  g_usb_led_state = 0U;
  /* Force re-format on next connection so Windows always sees a clean disk.
     Without this, a mid-write disconnect leaves a corrupt FAT that Windows
     refuses to open. */
  ram_disk_formatted = 0U;
  /* USER CODE END USBD_STORAGE_Deactivate */

  return;
}

/**
  * @brief  USBD_STORAGE_Read
  *         This function is invoked to read from media.
  * @param  storage_instance : Pointer to the storage class instance.
  * @param  lun: Logical unit number is the command is directed to.
  * @param  data_pointer: Address of the buffer to be used for reading or writing.
  * @param  number_blocks: number of sectors to read/write.
  * @param  lba: Logical block address is the sector address to read.
  * @param  media_status: should be filled out exactly like the media status
  *                       callback return value.
  * @retval status
  */
UINT USBD_STORAGE_Read(VOID *storage_instance, ULONG lun, UCHAR *data_pointer,
                       ULONG number_blocks, ULONG lba, ULONG *media_status)
{
  UINT status = UX_SUCCESS;

  /* USER CODE BEGIN USBD_STORAGE_Read */
  UX_PARAMETER_NOT_USED(storage_instance);
  UX_PARAMETER_NOT_USED(lun);
  UX_PARAMETER_NOT_USED(media_status);

  /* In USBX 6.2+ standalone mode _ux_device_class_storage_disk_wait interprets
     the return value as a state code: < UX_STATE_NEXT = error, == UX_STATE_NEXT = done.
     UX_SUCCESS = 0 is < UX_STATE_NEXT = 4, so returning UX_SUCCESS triggers a disk
     error and stalls the bulk-IN endpoint, causing DID_ERROR on the host. */
  if (lba + number_blocks > RAM_DISK_SECTOR_COUNT)
    return UX_STATE_EXIT;   /* UX_STATE_EXIT = 1 < UX_STATE_NEXT = 4 → disk error */

  if (data_pointer != UX_NULL)
    memcpy(data_pointer,
           ram_disk + (lba * RAM_DISK_SECTOR_SIZE),
           number_blocks * RAM_DISK_SECTOR_SIZE);

  status = UX_STATE_NEXT;   /* 4 = success sentinel for standalone state machine */
  /* USER CODE END USBD_STORAGE_Read */

  return status;
}

/**
  * @brief  USBD_STORAGE_Write
  *         This function is invoked to write in media.
  * @param  storage_instance : Pointer to the storage class instance.
  * @param  lun: Logical unit number is the command is directed to.
  * @param  data_pointer: Address of the buffer to be used for reading or writing.
  * @param  number_blocks: number of sectors to read/write.
  * @param  lba: Logical block address is the sector address to read.
  * @param  media_status: should be filled out exactly like the media status
  *                       callback return value.
  * @retval status
  */
UINT USBD_STORAGE_Write(VOID *storage_instance, ULONG lun, UCHAR *data_pointer,
                        ULONG number_blocks, ULONG lba, ULONG *media_status)
{
  UINT status = UX_SUCCESS;

  /* USER CODE BEGIN USBD_STORAGE_Write */
  UX_PARAMETER_NOT_USED(storage_instance);
  UX_PARAMETER_NOT_USED(lun);
  UX_PARAMETER_NOT_USED(media_status);

  if (lba + number_blocks > RAM_DISK_SECTOR_COUNT)
    return UX_STATE_EXIT;   /* < UX_STATE_NEXT → disk error, same as Read */

  if (data_pointer != UX_NULL) {
    memcpy(ram_disk + (lba * RAM_DISK_SECTOR_SIZE),
           data_pointer,
           number_blocks * RAM_DISK_SECTOR_SIZE);

    for (ULONG s = lba; s < lba + number_blocks; s++) {
        /* Root directory written: check for a completed .BMP entry.
           Fill animation is intentionally NOT driven from here — OS metadata
           writes (dirty-flag, thumbs.db, etc.) would otherwise trigger it
           immediately on mount. The animation runs from main.c instead. */
        if (s == SECTOR_ROOTDIR) {
            check_for_bmp();
        }
    }
  }

  status = UX_STATE_NEXT;   /* 4 = success for standalone USBX 6.2+ */
  /* USER CODE END USBD_STORAGE_Write */

  return status;
}

/**
  * @brief  USBD_STORAGE_Flush
  *         This function is invoked to flush media.
  * @param  storage_instance : Pointer to the storage class instance.
  * @param  lun: Logical unit number is the command is directed to.
  * @param  number_blocks: number of sectors to read/write.
  * @param  lba: Logical block address is the sector address to read.
  * @param  media_status: should be filled out exactly like the media status
  *                       callback return value.
  * @retval status
  */
UINT USBD_STORAGE_Flush(VOID *storage_instance, ULONG lun, ULONG number_blocks,
                        ULONG lba, ULONG *media_status)
{
  UINT status = UX_SUCCESS;

  /* USER CODE BEGIN USBD_STORAGE_Flush */
  UX_PARAMETER_NOT_USED(storage_instance);
  UX_PARAMETER_NOT_USED(lun);
  UX_PARAMETER_NOT_USED(number_blocks);
  UX_PARAMETER_NOT_USED(lba);
  UX_PARAMETER_NOT_USED(media_status);
  status = UX_STATE_NEXT;   /* SYNCHRONIZE_CACHE also uses disk_wait; same fix */
  /* USER CODE END USBD_STORAGE_Flush */

  return status;
}

/**
  * @brief  USBD_STORAGE_Status
  *         This function is invoked to obtain the status of the device.
  * @param  storage_instance : Pointer to the storage class instance.
  * @param  lun: Logical unit number is the command is directed to.
  * @param  media_id: is not currently used.
  * @param  media_status: should be filled out exactly like the media status
  *                       callback return value.
  * @retval status
  */
UINT USBD_STORAGE_Status(VOID *storage_instance, ULONG lun, ULONG media_id,
                         ULONG *media_status)
{
  UINT status = UX_SUCCESS;

  /* USER CODE BEGIN USBD_STORAGE_Status */
  UX_PARAMETER_NOT_USED(storage_instance);
  UX_PARAMETER_NOT_USED(lun);
  UX_PARAMETER_NOT_USED(media_id);
  /* _ux_device_class_storage_read copies *media_status into
     request_sense_status. If we leave it unwritten (UX_PARAMETER_NOT_USED),
     the host gets a garbage sense code on any REQUEST_SENSE and may retry
     commands indefinitely, causing the slow-mount symptom. */
  if (media_status != UX_NULL) *media_status = 0U;
  /* USER CODE END USBD_STORAGE_Status */

  return status;
}

/**
  * @brief  USBD_STORAGE_Notification
  *         This function is invoked to obtain the notification of the device.
  * @param  storage_instance : Pointer to the storage class instance.
  * @param  lun: Logical unit number is the command is directed to.
  * @param  media_id: is not currently used.
  * @param  notification_class: specifies the class of notification.
  * @param  media_notification: response for the notification.
  * @param  media_notification_length: length of the response buffer.
  * @retval status
  */
UINT USBD_STORAGE_Notification(VOID *storage_instance, ULONG lun, ULONG media_id,
                               ULONG notification_class, UCHAR **media_notification,
                               ULONG *media_notification_length)
{
  UINT status = UX_SUCCESS;

  /* USER CODE BEGIN USBD_STORAGE_Notification */
  UX_PARAMETER_NOT_USED(storage_instance);
  UX_PARAMETER_NOT_USED(lun);
  UX_PARAMETER_NOT_USED(media_id);
  UX_PARAMETER_NOT_USED(notification_class);
  UX_PARAMETER_NOT_USED(media_notification);
  UX_PARAMETER_NOT_USED(media_notification_length);
  /* USER CODE END USBD_STORAGE_Notification */

  return status;
}

/**
  * @brief  USBD_STORAGE_GetMediaLastLba
  *         Get Media last LBA.
  * @param  none
  * @retval last lba
  */
ULONG USBD_STORAGE_GetMediaLastLba(VOID)
{
  ULONG LastLba = 0U;

  /* USER CODE BEGIN USBD_STORAGE_GetMediaLastLba */
  LastLba = RAM_DISK_SECTOR_COUNT - 1U;
  /* USER CODE END USBD_STORAGE_GetMediaLastLba */

  return LastLba;
}

/**
  * @brief  USBD_STORAGE_GetMediaBlocklength
  *         Get Media block length.
  * @param  none.
  * @retval block length.
  */
ULONG USBD_STORAGE_GetMediaBlocklength(VOID)
{
  ULONG MediaBlockLen = 0U;

  /* USER CODE BEGIN USBD_STORAGE_GetMediaBlocklength */
  MediaBlockLen = RAM_DISK_SECTOR_SIZE;
  /* USER CODE END USBD_STORAGE_GetMediaBlocklength */

  return MediaBlockLen;
}

/* USER CODE BEGIN 1 */

/* Provide access to the raw RAM disk for main.c BMP processing. */
const uint8_t *usbd_get_ram_disk_ptr(void) { return ram_disk; }

/* USER CODE END 1 */
