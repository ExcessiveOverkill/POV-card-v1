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

/* ---- Sector layout ----
   Virtual LBA space the host sees:
     0            boot sector          (flash-backed, read-only)
     1            FAT copy 1           (RAM)
     2            FAT copy 2           (RAM)
     3            root directory       (RAM)
     4            README cluster 2     (flash-backed, read-only)
     5            README cluster 3     (flash-backed, read-only)
     6            .fseventsd cluster 4 (flash-backed, read-only)
     7..23        free/mutable data, clusters 5..21 (RAM)

   README, the boot sector, and the ".fseventsd" directory's content never
   change after format, so keeping them in the RAM buffer at all is wasted
   SRAM. They're served straight from flash instead (see flash_read()), and
   every byte that used to cost is handed back to the free/mutable data area
   -- same total RAM footprint as the original 20-sector/10240-byte disk, just
   17 free clusters instead of 13. */
#define SECTOR_BOOT     0U
#define SECTOR_FAT1     1U
#define SECTOR_FAT2     2U
#define SECTOR_ROOTDIR  3U
#define SECTOR_README1  4U
#define SECTOR_README2  5U
#define SECTOR_FSEV_DIR 6U
#define SECTOR_DATA     7U   /* first free/mutable sector = cluster 5 */

#define DATA_CLUSTER_COUNT    17U                             /* free/mutable clusters */
#define RAM_DISK_SECTOR_COUNT (SECTOR_DATA + DATA_CLUSTER_COUNT)  /* 24, host-visible   */
#define RAM_SLOT_COUNT        (3U + DATA_CLUSTER_COUNT)           /* FAT1+FAT2+ROOTDIR+data */
#define RAM_DISK_SIZE         (RAM_DISK_SECTOR_SIZE * RAM_SLOT_COUNT)  /* 10240, unchanged */

/* Root directory entry size and offsets */
#define DIRENT_SIZE    32U
#define DIRENT_ATTR    11U
#define DIRENT_CLUSTER 26U   /* uint16 LE: first cluster */
#define DIRENT_FILESIZE 28U  /* uint32 LE: file size in bytes */

#define ATTR_READONLY  0x01U
#define ATTR_HIDDEN    0x02U
#define ATTR_SYSTEM    0x04U
#define ATTR_VOLUME_ID 0x08U
#define ATTR_DIRECTORY 0x10U
#define ATTR_LFN       0x0FU

/* Number of writable data clusters (clusters 5..5+DATA_CLUSTER_COUNT-1);
   clusters 2-4 (README, .fseventsd) are flash-backed and never reach the
   s_junk_cluster_mask / ram_slot() machinery at all. */
#define MAX_DATA_CLUSTERS DATA_CLUSTER_COUNT

/* Fixed root-directory entries written by ram_disk_format(): 0 = volume label,
   1 = README.TXT, 2-4 = ".metadata_never_index" (2 LFN + short name),
   5-6 = ".fseventsd" (1 LFN + short name). Indices before this are permanent
   fixtures and must never be touched by filter_and_reclaim_junk(). */
#define RESERVED_ENTRY_COUNT 7U
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

/* Bit i set => free-area data cluster (i+5) currently belongs to a
   macOS/Windows housekeeping object (.Trashes, .Spotlight-V100, .DS_Store,
   "._" AppleDouble sidecars, any directory) and its data must not be stored.
   Rebuilt by filter_and_reclaim_junk() after every write. Clusters 2-4
   (README, .fseventsd) are flash-backed and never reach this mask -- writes
   to them are dropped unconditionally by ram_slot() returning NULL. */
static uint16_t s_junk_cluster_mask = 0U;

/* ---- Flash-resident, byte-exact images of the sectors that never change
   after format: the boot sector and the ".fseventsd" directory cluster
   ("." / ".." / "NO_LOG"). Generated from the same field values
   ram_disk_format() used to poke into RAM by hand -- see the comment on
   flash_read() for why these live in flash instead. ---- */
static const uint8_t boot_sector_image[RAM_DISK_SECTOR_SIZE] = {
    0xEB, 0x3C, 0x90, 0x4D, 0x53, 0x57, 0x49, 0x4E, 0x34, 0x2E, 0x31, 0x00, 0x02, 0x01, 0x01, 0x00,
    0x02, 0x10, 0x00, 0x18, 0x00, 0xF8, 0x01, 0x00, 0x01, 0x00, 0x01, 0x00, 0x00, 0x00, 0x00, 0x00,
    0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x29, 0x78, 0x56, 0x34, 0x12, 0x50, 0x4F, 0x56, 0x20, 0x43,
    0x41, 0x52, 0x44, 0x20, 0x20, 0x20, 0x46, 0x41, 0x54, 0x31, 0x32, 0x20, 0x20, 0x20, 0x00, 0x00,
    0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
    0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
    0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
    0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
    0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
    0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
    0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
    0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
    0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
    0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
    0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
    0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
    0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
    0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
    0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
    0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
    0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
    0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
    0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
    0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
    0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
    0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
    0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
    0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
    0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
    0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
    0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
    0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
    0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x55, 0xAA,
};

static const uint8_t fsev_dir_image[RAM_DISK_SECTOR_SIZE] = {
    0x2E, 0x20, 0x20, 0x20, 0x20, 0x20, 0x20, 0x20, 0x20, 0x20, 0x20, 0x10, 0x00, 0x00, 0x00, 0x00,
    0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x04, 0x00, 0x00, 0x00, 0x00, 0x00,
    0x2E, 0x2E, 0x20, 0x20, 0x20, 0x20, 0x20, 0x20, 0x20, 0x20, 0x20, 0x10, 0x00, 0x00, 0x00, 0x00,
    0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
    0x4E, 0x4F, 0x5F, 0x4C, 0x4F, 0x47, 0x20, 0x20, 0x20, 0x20, 0x20, 0x00, 0x00, 0x00, 0x00, 0x00,
    0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
    0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
    0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
    0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
    0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
    0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
    0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
    0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
    0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
    0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
    0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
    0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
    0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
    0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
    0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
    0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
    0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
    0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
    0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
    0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
    0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
    0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
    0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
    0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
    0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
    0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
    0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
};

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
    "Wait for the slow pulse to return, then unplug.\r\n"
    "\r\n"
    "Check github.com/ExcessiveOverkill/POV-card-v1 for latest updates and documentation.\r\n";

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
static void filter_and_reclaim_junk(void);
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

/* Read a FAT12 cluster chain entry (12-bit). */
static uint16_t fat12_get(const uint8_t *fat, uint32_t cluster)
{
    uint32_t byte_idx = cluster + (cluster / 2U);
    if (cluster % 2U == 0U) {
        return (uint16_t)(fat[byte_idx] | ((uint16_t)(fat[byte_idx + 1U] & 0x0FU) << 8));
    } else {
        return (uint16_t)((fat[byte_idx] >> 4) | ((uint16_t)fat[byte_idx + 1U] << 4));
    }
}

/* Standard FAT LFN short-name checksum. */
static uint8_t lfn_checksum(const uint8_t *sfn11)
{
    uint8_t sum = 0U;
    for (uint32_t i = 0; i < 11U; i++)
        sum = (uint8_t)(((sum & 1U) ? 0x80U : 0U) + (sum >> 1) + sfn11[i]);
    return sum;
}

/* Write the VFAT long-name entries for `name` into root[first_idx ..], stored
   highest-ordinal-first as the FAT spec requires (immediately followed by the
   short-name entry the caller writes itself at root[first_idx + nentries]).
   sfn11 is the 11-byte 8.3 alias the entries' checksum must match, or real FAT
   drivers (this firmware's own lfn_read() doesn't check, but macOS/Windows do)
   will ignore the long name and fall back to the short one. */
static void write_lfn(uint8_t *root, uint32_t first_idx, const char *name,
                      const uint8_t *sfn11)
{
    static const uint8_t off[13] = { 1,3,5,7,9, 14,16,18,20,22,24, 28,30 };
    uint32_t len      = (uint32_t)strlen(name);
    uint32_t nentries = (len + 12U) / 13U;
    uint8_t  csum     = lfn_checksum(sfn11);

    for (uint32_t e = 0; e < nentries; e++) {
        uint8_t *ent = root + (first_idx + e) * DIRENT_SIZE;
        uint32_t ord = nentries - e;          /* 1-based; highest ordinal first */
        uint32_t base = (ord - 1U) * 13U;
        uint8_t  seq  = (uint8_t)ord;
        if (e == 0U) seq |= 0x40U;            /* first entry written = last (highest) */

        memset(ent, 0xFF, DIRENT_SIZE);       /* unused char slots pad to 0xFFFF */
        ent[0]           = seq;
        ent[DIRENT_ATTR] = ATTR_LFN;
        ent[12]          = 0U;
        ent[13]          = csum;
        ent[26]          = 0U;
        ent[27]          = 0U;

        for (uint32_t k = 0; k < 13U; k++) {
            uint32_t ci = base + k;
            if (ci > len) continue;           /* leave 0xFFFF padding */
            uint16_t ch = (ci == len) ? 0x0000U : (uint16_t)(uint8_t)name[ci];
            ent[off[k]]      = (uint8_t)(ch & 0xFFU);
            ent[off[k] + 1U] = (uint8_t)(ch >> 8);
        }
    }
}

/* Map a RAM-backed virtual sector (FAT1, FAT2, ROOTDIR, or the free/mutable
   data area) to its offset in ram_disk[]. Returns NULL for the flash-backed
   read-only sectors (boot, README's two clusters, .fseventsd's cluster) --
   callers check flash_read() first, or treat NULL as "ignore this write". */
static uint8_t *ram_slot(uint32_t sector)
{
    if (sector == SECTOR_FAT1)    return ram_disk + 0U * RAM_DISK_SECTOR_SIZE;
    if (sector == SECTOR_FAT2)    return ram_disk + 1U * RAM_DISK_SECTOR_SIZE;
    if (sector == SECTOR_ROOTDIR) return ram_disk + 2U * RAM_DISK_SECTOR_SIZE;
    if (sector >= SECTOR_DATA && sector < RAM_DISK_SECTOR_COUNT)
        return ram_disk + (3U + (sector - SECTOR_DATA)) * RAM_DISK_SECTOR_SIZE;
    return UX_NULL;
}

/* Fill `out` (512 bytes) for a flash-backed read-only sector (boot, README's
   two clusters, or .fseventsd's directory cluster) and return 1, or return 0
   if `sector` isn't one of them (caller then falls back to ram_slot()).
   README's own const array is shorter than 1024 bytes, so its second cluster
   is zero-padded here rather than requiring readme_text to be pre-padded. */
static uint8_t flash_read(uint32_t sector, uint8_t *out)
{
    if (sector == SECTOR_BOOT) {
        memcpy(out, boot_sector_image, RAM_DISK_SECTOR_SIZE);
        return 1U;
    }
    if (sector == SECTOR_README1 || sector == SECTOR_README2) {
        uint32_t readme_len = (uint32_t)(sizeof(readme_text) - 1U);
        uint32_t base  = (sector - SECTOR_README1) * RAM_DISK_SECTOR_SIZE;
        uint32_t avail = (base < readme_len) ? (readme_len - base) : 0U;
        uint32_t n     = (avail > RAM_DISK_SECTOR_SIZE) ? RAM_DISK_SECTOR_SIZE : avail;
        memset(out, 0, RAM_DISK_SECTOR_SIZE);
        if (n != 0U) memcpy(out, readme_text + base, n);
        return 1U;
    }
    if (sector == SECTOR_FSEV_DIR) {
        memcpy(out, fsev_dir_image, RAM_DISK_SECTOR_SIZE);
        return 1U;
    }
    return 0U;
}

static void ram_disk_format(void)
{
    memset(ram_disk, 0, sizeof(ram_disk));

    /* Boot sector, README's data, and the ".fseventsd" directory's data are
       flash-backed (boot_sector_image / readme_text / fsev_dir_image) and
       never touch ram_disk[] at all -- only their FAT chain + directory
       entries (below) need to live in RAM, same as any other file. */

    /* ---- FAT copies ---- */
    /*  Entry 0 = 0xFF8 (media byte), entry 1 = 0xFFF (reserved).
        README.txt occupies clusters 2 and 3 (two sectors, 1024 bytes).
        Cluster 4 holds the ".fseventsd" opt-out directory (one cluster). */
    uint8_t *fat1 = ram_slot(SECTOR_FAT1);
    uint8_t *fat2 = ram_slot(SECTOR_FAT2);

    fat12_set(fat1, 0U, 0xFF8U);
    fat12_set(fat1, 1U, 0xFFFU);
    fat12_set(fat1, 2U, 0x003U);   /* cluster 2 → next cluster 3 */
    fat12_set(fat1, 3U, 0xFFFU);   /* cluster 3 = end of chain   */
    fat12_set(fat1, 4U, 0xFFFU);   /* cluster 4 = end of chain   */

    memcpy(fat2, fat1, RAM_DISK_SECTOR_SIZE);

    /* ---- Root directory (sector 3) ---- */
    uint8_t *root = ram_slot(SECTOR_ROOTDIR);

    /* Entry 0: volume label */
    memcpy(root, "POV CARD   ", 11);
    root[DIRENT_ATTR] = ATTR_VOLUME_ID;

    /* Entry 1: README.TXT (read-only). Its data lives in flash (readme_text),
       served by flash_read() -- only the directory entry itself is in RAM. */
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

    /* Entries 2-4: ".metadata_never_index" -- tells Spotlight/mds to never index
       this volume, so it never creates ".Spotlight-V100". Zero-length file: no
       cluster, no data (matches the "first_cluster < 2 = no data" convention
       already used by check_for_bmp()/filter_and_reclaim_junk()). The name is
       21 chars (needs 2 LFN entries); the short-name alias is never consumed by
       anything real since the LFN entries carry the true name. */
    {
        uint8_t sfn11[11];
        memcpy(sfn11, "NEVERI~1   ", 11);
        write_lfn(root, 2U, ".metadata_never_index", sfn11);
        uint8_t *sfn = root + 4U * DIRENT_SIZE;
        memcpy(sfn, sfn11, 11);
        sfn[DIRENT_ATTR] = ATTR_HIDDEN;
    }

    /* Entries 5-6: ".fseventsd" -- tells fseventsd not to journal this volume,
       so it stops writing its own event-log files here. A real directory:
       cluster 4, "." / ".." / "NO_LOG" inside (flash-backed, fsev_dir_image). */
    {
        uint8_t sfn11[11];
        memcpy(sfn11, "FSEVEN~1   ", 11);
        write_lfn(root, 5U, ".fseventsd", sfn11);
        uint8_t *sfn = root + 6U * DIRENT_SIZE;
        memcpy(sfn, sfn11, 11);
        sfn[DIRENT_ATTR] = ATTR_DIRECTORY | ATTR_HIDDEN;
        sfn[DIRENT_CLUSTER]     = 4U;
        sfn[DIRENT_CLUSTER + 1] = 0U;
    }
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
    const uint8_t *root = ram_slot(SECTOR_ROOTDIR);

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
           For this small disk every file is contiguous from its first cluster.
           A real upload can only live in the free/mutable area (cluster >= 5,
           virtual sector >= SECTOR_DATA) -- clusters 2-4 are permanently
           allocated to README/.fseventsd so the host's own allocator never
           hands them to a new file, but check anyway since those sectors are
           flash-backed and ram_slot() returns NULL for them. */
        uint32_t virtual_sector = SECTOR_README1 + (uint32_t)(first_cluster - 2U);
        if (virtual_sector < SECTOR_DATA || virtual_sector >= RAM_DISK_SECTOR_COUNT)
            continue;
        uint32_t max_bytes = (RAM_DISK_SECTOR_COUNT - virtual_sector) * RAM_DISK_SECTOR_SIZE;
        if (fsize > max_bytes)
            continue;

        /* Verify the BMP magic bytes are present in the data sector.
           Windows can write the directory entry (with non-zero size and valid
           cluster) before flushing the file data, so without this check we
           would signal g_bmp_pending while the data sectors are still zeroed. */
        const uint8_t *bmp_data = ram_slot(virtual_sector);
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
        g_bmp_ram_sector  = virtual_sector - SECTOR_DATA;  /* offset into the
                               free-data area returned by usbd_get_ram_disk_ptr() */
        g_bmp_ram_size    = fsize;
        g_bmp_pending     = 1U;

        return;
    }
}

/*
 * Scan the root directory for host-created housekeeping objects (macOS's
 * .Trashes, .fseventsd, .Spotlight-V100, .DS_Store, "._" AppleDouble resource
 * -fork sidecars -- including one written for the user's own bmp -- plus any
 * directory, since this firmware never expects a real one from the user) and
 * reclaim them: free their FAT12 chain in both copies and delete their
 * directory entry, so the host's own free-space accounting recovers
 * immediately instead of believing the tiny disk is full. s_junk_cluster_mask
 * is rebuilt every call so USBD_STORAGE_Write() can also drop the raw data
 * payload of a not-yet-fully-written junk object (the host may send the
 * directory entry and the file data in separate WRITE commands).
 *
 * Real entries (the user's .bmp) are never classified junk and are left
 * untouched. Entries 0..RESERVED_ENTRY_COUNT-1 (volume label, README.TXT, and
 * the ".metadata_never_index" / ".fseventsd" opt-out fixtures ram_disk_format()
 * writes) are fixed fixtures, not scanned here at all -- otherwise this would
 * misclassify and delete our own ".fseventsd" directory and hidden marker file
 * on the very next write, undoing the opt-out.
 */
static void filter_and_reclaim_junk(void)
{
    uint8_t *root = ram_slot(SECTOR_ROOTDIR);
    uint8_t *fat1 = ram_slot(SECTOR_FAT1);
    uint8_t *fat2 = ram_slot(SECTOR_FAT2);
    uint16_t mask = 0U;

    for (uint32_t i = RESERVED_ENTRY_COUNT; i < 16U; i++) {
        uint8_t *e = root + i * DIRENT_SIZE;

        if (e[0] == 0x00U || e[0] == 0xE5U)
            continue;
        uint8_t attr = e[DIRENT_ATTR];
        if (attr == ATTR_LFN)
            continue;
        if (attr & ATTR_VOLUME_ID)
            continue;

        char     lname[LFN_NAME_MAX];
        uint32_t lnlen = lfn_read(root, i, lname);

        uint8_t junk = (lnlen > 0U)
                     ? (uint8_t)(lname[0] == '.')
                     : (uint8_t)((attr & (ATTR_HIDDEN | ATTR_SYSTEM)) != 0U);
        if (attr & ATTR_DIRECTORY)
            junk = 1U;

        if (!junk)
            continue;

        uint16_t first_cluster =
            (uint16_t)e[DIRENT_CLUSTER] | ((uint16_t)e[DIRENT_CLUSTER + 1] << 8);
        uint32_t fsize =
            (uint32_t)e[DIRENT_FILESIZE]
          | ((uint32_t)e[DIRENT_FILESIZE + 1] << 8)
          | ((uint32_t)e[DIRENT_FILESIZE + 2] << 16)
          | ((uint32_t)e[DIRENT_FILESIZE + 3] << 24);

        /* Clusters 2-4 (README, .fseventsd) are flash-backed and permanently
           allocated -- the host's own allocator never hands them to a new
           file, so a junk entry's first_cluster should never land there, but
           guard anyway: cluster-5 would underflow for cluster < 5. */
        if (first_cluster >= 5U) {
            /* Directories report a size of 0 but still occupy a cluster. */
            uint32_t nclusters = fsize ? (fsize + RAM_DISK_SECTOR_SIZE - 1U) / RAM_DISK_SECTOR_SIZE
                                        : 1U;
            uint16_t cluster = first_cluster;

            for (uint32_t c = 0; c < nclusters && cluster >= 5U; c++) {
                uint32_t bit = cluster - 5U;
                if (bit >= MAX_DATA_CLUSTERS)
                    break;
                mask |= (uint16_t)(1U << bit);

                uint16_t next = fat12_get(fat1, cluster);
                fat12_set(fat1, cluster, 0x000U);
                fat12_set(fat2, cluster, 0x000U);
                cluster = next;
            }
        }

        e[0] = 0xE5U;   /* delete the directory entry */
    }

    s_junk_cluster_mask = mask;
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
  s_junk_cluster_mask = 0U;
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

  if (data_pointer != UX_NULL) {
    /* Sector-by-sector so flash-backed sectors (boot, README, .fseventsd's
       directory cluster -- see flash_read()) can be served straight from
       flash instead of a wasted RAM copy; everything else comes from the
       (now smaller) ram_disk[] via ram_slot(). */
    for (ULONG s = 0; s < number_blocks; s++) {
      ULONG   sector = lba + s;
      uint8_t *out   = data_pointer + (s * RAM_DISK_SECTOR_SIZE);

      if (!flash_read((uint32_t)sector, out)) {
        const uint8_t *src = ram_slot((uint32_t)sector);
        if (src != UX_NULL)
          memcpy(out, src, RAM_DISK_SECTOR_SIZE);
        else
          memset(out, 0, RAM_DISK_SECTOR_SIZE);   /* unreachable given the bounds check above */
      }
    }
  }

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
    /* Copy sector-by-sector (instead of one bulk memcpy) so two kinds of
       writes can be silently dropped while everything else is stored as
       before (the SCSI write still reports success either way):
         - any write to a flash-backed sector (boot, README, .fseventsd's
           directory cluster -- ram_slot() returns NULL for these; they're
           permanently read-only and the host should never need to touch them)
         - a write to a data sector already known to belong to a macOS/Windows
           housekeeping object (s_junk_cluster_mask, see
           filter_and_reclaim_junk()), keeping the free/mutable area clear
           for the real .bmp. */
    for (ULONG s = 0; s < number_blocks; s++) {
      ULONG    sector = lba + s;
      uint8_t *dst    = ram_slot((uint32_t)sector);

      if (dst == UX_NULL)
        continue;   /* flash-backed, read-only sector */

      if (sector >= SECTOR_DATA) {
        uint32_t bit = (uint32_t)(sector - SECTOR_DATA);
        if (bit < MAX_DATA_CLUSTERS && (s_junk_cluster_mask & (1U << bit)))
          continue;
      }

      memcpy(dst, data_pointer + (s * RAM_DISK_SECTOR_SIZE), RAM_DISK_SECTOR_SIZE);
    }

    /* Re-scan for a completed .BMP after every write, not just root-dir writes:
       the host may flush the directory entry before the file data (or vice
       versa), so we can't assume which write completes the picture.
       check_for_bmp() only signals when BOTH the entry and BM-signed data are
       present, and its per-slot content hash makes repeat calls no-ops. */
    check_for_bmp();

    /* Reclaim any macOS/Windows housekeeping object written by this command
       (or drop its data if it arrived first) so the host's free-space view
       recovers immediately instead of seeing the disk as full. */
    filter_and_reclaim_junk();
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
/* main.c indexes this as ptr + g_bmp_ram_sector*512, and check_for_bmp() sets
   g_bmp_ram_sector relative to the start of the free/mutable data area (i.e.
   an offset, not a raw virtual LBA) -- so this must return that area's base,
   not ram_disk itself (which now starts with FAT1, not sector 0). */
const uint8_t *usbd_get_ram_disk_ptr(void) { return ram_slot(SECTOR_DATA); }

/* USER CODE END 1 */
