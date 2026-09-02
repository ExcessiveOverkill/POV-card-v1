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

/* Content hash of the last BMP handed to main.c for each slot, so a root-dir
   rewrite (the host touches sector 3 several times per copy) does not re-flash
   the same image. Cleared on every (re)connect in USBD_STORAGE_Activate. */
static uint32_t s_slot_sig[2] = {0U, 0U};

/* ---- README.txt content baked onto the drive ---- */
static const char readme_text[] =
    "POV CARD - USB IMAGE UPLOAD\r\n"
    "===========================\r\n"
    "Copy a BMP file onto this drive to load a custom\r\n"
    "image. View uploads with the DISPLAY CUSTOM mode.\r\n"
    "\r\n"
    "BMP requirements:\r\n"
    "  - uncompressed, 1-bit or 4-bit\r\n"
    "  - height 8, 16 or 32 px\r\n"
    "  - width up to 128 px\r\n"
    "\r\n"
    "Filename:\r\n"
    "  - must end in 1 or 2 (the image slot) before\r\n"
    "    \".bmp\"    e.g.  logo1.bmp -> slot 1\r\n"
    "                    pic2.bmp  -> slot 2\r\n"
    "  - start the name with \"SEQ\" for a multi-frame\r\n"
    "    animation; frames are split on a 1px column\r\n"
    "    that alternates every pixel.   e.g. seq1.bmp\r\n"
    "  - any name works (upper/lower/long)\r\n"
    "\r\n"
    "Re-uploading a slot overwrites it. Slots are kept\r\n"
    "through a power cycle.\r\n"
    "\r\n"
    "LED status:\r\n"
    "  slow pulse    ready, waiting for a file\r\n"
    "  growing bar   writing to flash\r\n"
    "  all bright    success\r\n"
    "  fast blink    error - bad file, wrong slot,\r\n"
    "                or image too large\r\n"
    "\r\n"
    "Wait for the slow pulse to return, then unplug.\r\n";

/* ---- Shared state read by main.c ---- */
volatile uint8_t  g_usb_connected  = 0U;   /* set by USBD_ChangeFunction  */
volatile uint8_t  g_bmp_pending    = 0U;   /* set when a valid .BMP lands */
volatile uint8_t  g_bmp_frame_idx  = 0U;   /* trailing filename digit (0-9) */
volatile uint8_t  g_bmp_is_sequence = 0U;  /* name starts with "SEQ" -> animation */
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

/* Reconstruct a VFAT long filename for the short entry at root index `sfn_idx`
   by walking the LFN entries that physically precede it (stored highest-ordinal
   first). Writes an ASCII copy (non-ASCII -> '?') to name[LFN_NAME_MAX] and
   returns its length, or 0 if the entry has no long name. */
#define LFN_NAME_MAX  78U   /* up to 6 LFN entries x 13 chars */

static uint32_t lfn_read(const uint8_t *root, uint32_t sfn_idx, char *name)
{
    static const uint8_t off[13] = { 1,3,5,7,9, 14,16,18,20,22,24, 28,30 };
    uint32_t max_ord = 0U;

    memset(name, 0, LFN_NAME_MAX);

    for (int32_t j = (int32_t)sfn_idx - 1; j >= 0; j--) {
        const uint8_t *e = root + (uint32_t)j * DIRENT_SIZE;

        if (e[DIRENT_ATTR] != 0x0FU) break;      /* not an LFN entry */
        if (e[0] == 0xE5U)           break;      /* deleted */

        uint32_t ord = (uint32_t)(e[0] & 0x1FU);
        if (ord == 0U || ord > (LFN_NAME_MAX / 13U)) break;

        uint32_t base = (ord - 1U) * 13U;
        for (uint32_t k = 0; k < 13U; k++) {
            uint16_t ch = (uint16_t)e[off[k]] | ((uint16_t)e[off[k] + 1U] << 8);
            name[base + k] = (ch == 0x0000U || ch == 0xFFFFU) ? '\0'
                           : (ch < 0x80U) ? (char)ch : '?';
        }
        if (ord > max_ord) max_ord = ord;

        if (e[0] & 0x40U) break;                 /* last (highest-ordinal) piece */
    }

    if (max_ord == 0U) return 0U;

    uint32_t len = max_ord * 13U;
    for (uint32_t k = 0; k < len; k++) {
        if (name[k] == '\0') { len = k; break; }
    }
    return len;
}

/*
 * Scan the root directory for a completed .BMP directory entry.
 * Called whenever sector 3 (root directory) is written.
 *
 * A "complete" entry has a non-zero file size and a valid starting cluster.
 * The target slot is the last filename character before ".bmp" (1 or 2);
 * a "SEQ" prefix marks a multi-frame animation. The long (VFAT) name is used
 * when present so mixed-case names like "EO_Logo1.bmp" work.
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
            continue;   /* volume ID, directory, or system (also skips LFN 0x0F) */

        /* Must have .BMP / .bmp extension (8.3 name bytes 8-10). */
        if ((e[8] != 'B' && e[8] != 'b') || (e[9] != 'M' && e[9] != 'm') || (e[10] != 'P' && e[10] != 'p'))
            continue;

        /* --- slot digit + sequence flag from the filename --- */
        char     lname[LFN_NAME_MAX];
        uint32_t lnlen = lfn_read(root, i, lname);
        uint8_t  last_char, is_seq;

        if (lnlen >= 5U) {
            /* Long name present: trust it over the (possibly mangled) 8.3 alias.
               Must be "...X.bmp". */
            if (lname[lnlen - 4] != '.' ||
                (lname[lnlen - 3] != 'b' && lname[lnlen - 3] != 'B') ||
                (lname[lnlen - 2] != 'm' && lname[lnlen - 2] != 'M') ||
                (lname[lnlen - 1] != 'p' && lname[lnlen - 1] != 'P'))
                continue;
            last_char = (uint8_t)lname[lnlen - 5];
            is_seq = ((lname[0] == 'S' || lname[0] == 's') &&
                      (lname[1] == 'E' || lname[1] == 'e') &&
                      (lname[2] == 'Q' || lname[2] == 'q')) ? 1U : 0U;
        } else {
            /* No long name: use the 8.3 short name. A '~' with no LFN means a
               mangled alias we can't interpret -- skip it. */
            int has_tilde = 0, name_end = -1;
            for (int j = 0; j < 8; j++) if (e[j] == '~') { has_tilde = 1; break; }
            if (has_tilde)
                continue;
            for (int j = 7; j >= 0; j--) { if (e[j] != ' ') { name_end = j; break; } }
            if (name_end < 0)
                continue;
            last_char = e[name_end];
            is_seq = ((e[0] == 'S' || e[0] == 's') &&
                      (e[1] == 'E' || e[1] == 'e') &&
                      (e[2] == 'Q' || e[2] == 'q')) ? 1U : 0U;
        }

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

        /* Content hash (FNV-1a). If this slot already got these exact bytes,
           the host is just rewriting the directory -- don't re-flash. */
        uint32_t sig = 2166136261U;
        for (uint32_t k = 0; k < fsize; k++) {
            sig ^= bmp_data[k];
            sig *= 16777619U;
        }
        uint32_t sig_slot = (frame_idx == 2U) ? 1U : 0U;
        if (sig == s_slot_sig[sig_slot])
            continue;
        s_slot_sig[sig_slot] = sig;

        // Signal main loop to process this BMP.
        g_bmp_frame_idx   = frame_idx;
        g_bmp_is_sequence = is_seq;
        g_bmp_ram_sector  = data_sector;
        g_bmp_ram_size    = fsize;
        g_bmp_pending     = 1U;

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
  s_slot_sig[0] = 0U;     /* fresh disk -> forget which images we've seen */
  s_slot_sig[1] = 0U;
  ram_disk_format();
  ram_disk_formatted = 1U;
  g_usb_connected = 1U;   /* drive mounted: a host is definitely present */
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
  g_usb_connected = 0U;
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

    /* Re-scan for a completed .BMP after every write, not just root-dir writes:
       the host may flush the directory entry before the file data (or vice
       versa), so we can't assume which write completes the picture.
       check_for_bmp() only signals when BOTH the entry and BM-signed data are
       present, and its per-slot content hash makes repeat calls no-ops. */
    check_for_bmp();
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
