/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
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
#include "main.h"
#include "app_usbx_device.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include "math.h"
#include "ux_device_msc.h"
#include "pov_bmp.h"
#include "pov_frame.h"
#include <stdlib.h>
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */

#define MAX_POWER_OFF_DELAY 60*4	// how many seconds of being on before shutting off, regardless of activity
#define POWER_OFF_DELAY 20	// how many seconds of no movement before powering down
#define LED_ENABLE_ACCELERATION 6000

#define ACCEL_AVG_SHIFT 8

/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/

SPI_HandleTypeDef hspi1;
SPI_HandleTypeDef hspi2;
DMA_HandleTypeDef hdma_spi1_tx;
DMA_HandleTypeDef hdma_spi1_rx;
DMA_HandleTypeDef hdma_spi2_tx;

TIM_HandleTypeDef htim1;
TIM_HandleTypeDef htim2;
DMA_HandleTypeDef hdma_tim1_up;

PCD_HandleTypeDef hpcd_USB_DRD_FS;

/* USER CODE BEGIN PV */
static uint8_t s_frame_buf[POV_FRAME_SIZE];
static uint8_t s_usb_ready = 0U;
extern volatile uint8_t  g_usb_connected;
extern volatile uint8_t  g_bmp_pending;
extern volatile uint8_t  g_bmp_frame_idx;
extern volatile uint8_t  g_bmp_fill_level;
extern volatile uint8_t  g_usb_led_state;
extern volatile uint32_t g_bmp_ram_sector;
extern volatile uint32_t g_bmp_ram_size;

extern uint32_t _suser_image_data;
/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_DMA_Init(void);
static void MX_USB_PCD_Init(void);
static void MX_SPI1_Init(void);
static void MX_SPI2_Init(void);
static void MX_TIM1_Init(void);
static void MX_DMAMUX_Init(void);
static void MX_TIM2_Init(void);
/* USER CODE BEGIN PFP */
extern UINT _ux_system_tasks_run(void);
/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */

const uint8_t letter_pixels[] = {
    0x3F, 0x48, 0x48, 0x48, 0xBF, 0x7F, 0x49, 0x49,
    0x49, 0xB6, 0x3E, 0x41, 0x41, 0x41, 0xC1, 0x7F,
    0x41, 0x41, 0x41, 0xBE, 0x7F, 0x49, 0x49, 0x41,
    0xC1, 0x7F, 0x48, 0x48, 0x40, 0xC0, 0x3E, 0x41,
    0x41, 0x49, 0xCE, 0x7F, 0x08, 0x08, 0x08, 0xFF,
    0x41, 0x7F, 0xC1, 0x02, 0x01, 0x41, 0x7E, 0xC0,
    0x7F, 0x08, 0x14, 0x22, 0xC1, 0x7F, 0x01, 0x01,
    0x01, 0x81, 0x7F, 0x20, 0x10, 0x20, 0xFF, 0x7F,
    0x30, 0x08, 0x06, 0xFF, 0x3E, 0x41, 0x41, 0x41,
    0xBE, 0x7F, 0x48, 0x48, 0x48, 0xB0, 0x3E, 0x41,
    0x45, 0x43, 0xBE, 0x7F, 0x48, 0x4C, 0x4A, 0xB1,
    0x32, 0x49, 0x49, 0x49, 0xA6, 0x40, 0x40, 0x7F,
    0x40, 0xC0, 0x7E, 0x01, 0x01, 0x01, 0xFE, 0x70,
    0x0E, 0x01, 0x0E, 0xF0, 0x7E, 0x01, 0x06, 0x01,
    0xFE, 0x63, 0x14, 0x08, 0x14, 0xE3, 0x60, 0x10,
    0x0F, 0x10, 0xE0, 0x43, 0x45, 0x49, 0x51, 0xE1,
    0x40, 0xFF, 0x21, 0x43, 0x45, 0x49, 0xB1, 0x22,
    0x41, 0x49, 0x49, 0xB6, 0x78, 0x08, 0x08, 0x08,
    0xFF, 0x7A, 0x49, 0x49, 0x49, 0xC6, 0x3E, 0x49,
    0x49, 0x49, 0xA6, 0x40, 0x40, 0x4F, 0x50, 0xE0,
    0x36, 0x49, 0x49, 0x49, 0xB6, 0x32, 0x49, 0x49,
    0x49, 0xBE, 0x3E, 0x43, 0x5D, 0x61, 0xBE,
};

// Total bytes: 175
// Letters found: 36
// Bit layout per byte: bits 6-0 = column pixels (bit 0 = bottom row)
// Bit 7 = 1 marks the last column of each letter

static uint32_t accel_last_update_time = 0;
static uint32_t accel_update_period = 1000;
static uint8_t new_accel_data = 0;

static const uint8_t accel_run_tx_request[1+2*3] = {(0b11 << 6) | 0x28, 0, 0, 0, 0, 0, 0};
static uint8_t accel_run_rx_data[1+2*3];

static int16_t accel_x_raw = 0;
static int16_t accel_y_raw = 0;
static int16_t accel_z_raw = 0;
static int32_t last_accel_x_raw = 0;
static int32_t last_accel_y_raw = 0;
static int32_t last_accel_z_raw = 0;

static uint32_t plane[16];

typedef enum {
	NONE = 0,
	ONE_BIT,
	FOUR_BIT,
	FOUR_BIT_FANCY
} Image_Display_Mode;

typedef struct {
	uint8_t display_cycles;
	uint8_t cycle_count;
	uint8_t column_height;
	uint8_t max_frame_columns;
	uint8_t frame_columns[128];
	uint8_t frame_count;

	Image_Display_Mode mode;

	uint8_t* image_data;	// points to start of image data in flash
	uint16_t image_data_length;

} Image_Metadata;

typedef struct{
	uint8_t startup_mode;
	int32_t accel_x_zero;
	int32_t accel_y_zero;
	uint8_t first_run;
	Image_Metadata user_images[2];

} Nv_Metadata;

__attribute__((section(".eo_image_data")))
#include "eo_image_data.h"


extern const Nv_Metadata _smeta_data;

static Nv_Metadata saved_metadata;

static uint16_t blank_ccr[16];

static const uint16_t default_blank_ccr[16] = {
	128-1,
	128-1,
	128-1,
	128-2,
	128-2,
	128-4,
	128-5,
	128-10,
	128-20,
	128-30,
	128-40,
	128-50,
	128-60,
	128-80,
	128-127,
	128-127,
};

static const uint16_t ball_blank_ccr[16] = {
	128-1,
	128-1,
	128-1,
	128-2,
	128-2,
	128-4,
	128-5,
	128-6,
	128-6,
	128-6,
	128-6,
	128-6,
	128-6,
	128-6,
	128-6,
	128-6,
};

void EXTI0_1_IRQHandler(void)
{
    __HAL_GPIO_EXTI_CLEAR_IT(ACCEL_INT1_Pin);

    uint32_t tick = TIM2->CNT;
    accel_update_period = tick - accel_last_update_time;
    accel_last_update_time = tick;

    /* Assert CS */
    HAL_GPIO_WritePin(ACCEL_CS_GPIO_Port, ACCEL_CS_Pin, GPIO_PIN_RESET);

    /* Arm both DMA channels — RX first so it's ready before TX starts clocking */
    HAL_DMA_Start_IT(&hdma_spi1_rx,
                     (uint32_t)&hspi1.Instance->DR,
                     (uint32_t)accel_run_rx_data,
                     sizeof(accel_run_tx_request));
    HAL_DMA_Start(&hdma_spi1_tx,
                  (uint32_t)accel_run_tx_request,
                  (uint32_t)&hspi1.Instance->DR,
                  sizeof(accel_run_rx_data));

    /* Enable SPI DMA requests — TX firing first starts the clock */
    SET_BIT(hspi1.Instance->CR2, SPI_CR2_RXDMAEN);
    SET_BIT(hspi1.Instance->CR2, SPI_CR2_TXDMAEN);
}

static void Accel_DMA_RxComplete(DMA_HandleTypeDef *hdma)
{
	/* Deassert CS */
	HAL_GPIO_WritePin(ACCEL_CS_GPIO_Port, ACCEL_CS_Pin, GPIO_PIN_SET);

	/* Disable SPI DMA requests for next transaction */
	CLEAR_BIT(hspi1.Instance->CR2, SPI_CR2_RXDMAEN | SPI_CR2_TXDMAEN);

	/* Reset TX handle state — no TC interrupt on TX channel means HAL
	 * never clears STATE_BUSY automatically after the transfer completes */
	hdma_spi1_tx.State = HAL_DMA_STATE_READY;

	/* Clear SPI overrun flag — set when the last RX byte clocked in
	 * before DMA read it, leaving stale data in DR. If not cleared,
	 * SPI stops generating RX DMA requests on the next transaction. */
	__HAL_SPI_CLEAR_OVRFLAG(&hspi1);


	// save data
	last_accel_x_raw = accel_x_raw;
	last_accel_y_raw = accel_y_raw;
	last_accel_z_raw = accel_z_raw;
	accel_x_raw = accel_run_rx_data[1] | (accel_run_rx_data[2]<<8);
	accel_y_raw = accel_run_rx_data[3] | (accel_run_rx_data[4]<<8);
	accel_z_raw = accel_run_rx_data[5] | (accel_run_rx_data[6]<<8);
	new_accel_data = 1;

}

/* --------------------------------------------------------------------------
 * Start the LED system
 * Call after frame_buf is populated.
 * -------------------------------------------------------------------------- */
void LED_Start(void)
{
    HAL_DMA_Start(&hdma_spi2_tx,
                  (uint32_t)plane,
                  (uint32_t)&hspi2.Instance->DR,
                  sizeof(plane)/2);

    __HAL_SPI_ENABLE(&hspi2);

    HAL_DMA_Start(&hdma_tim1_up,
				  (uint32_t)blank_ccr,
				  (uint32_t)&TIM1->CCR2,
				  sizeof(blank_ccr)/2);



	__HAL_TIM_ENABLE_DMA(&htim1, TIM_DMA_UPDATE);

    HAL_TIM_PWM_Start(&htim1, TIM_CHANNEL_1);	// led latch
    HAL_TIM_PWM_Start(&htim1, TIM_CHANNEL_2);	// led blank
    HAL_TIM_PWM_Start(&htim1, TIM_CHANNEL_4);	// dma trigger

    HAL_TIM_Base_Stop(&htim1);
    TIM1->CNT = 0;

    SET_BIT(TIM1->DIER, TIM_DIER_UDE);
    SET_BIT(hspi2.Instance->CR2, SPI_CR2_TXDMAEN);

    HAL_TIM_Base_Start(&htim1);


}

/* Build from 4-bit brightness values. */
void BCM_BuildFrame(const uint8_t brightness[32])
{
	memset(plane, 0, sizeof(plane));	// zero

    for(int i = 0; i < 32; i++) {
        uint8_t b = brightness[i];

        for(int j = 0; j < b; j++){
        	plane[j] |= (1u << i);
        }
    }
}


uint8_t update_metadata(){

	FLASH_EraseInitTypeDef erase = {
		.TypeErase = FLASH_TYPEERASE_PAGES,
		.Page      = ((uint32_t)&_smeta_data - FLASH_BASE) / FLASH_PAGE_SIZE,
		.NbPages   = 1
	};

	uint32_t page_err = 0;

	HAL_FLASH_Unlock();
	HAL_StatusTypeDef status = HAL_FLASHEx_Erase(&erase, &page_err);
	HAL_FLASH_Lock();

	if(status != HAL_OK) return 1;

	uint8_t buf[sizeof(Nv_Metadata) + 8] = {0xFF};
	memcpy(buf, &saved_metadata, sizeof(Nv_Metadata));

	uint32_t base_addr = (uint32_t)&_smeta_data;

	HAL_FLASH_Unlock();

	for (uint32_t i = 0; i < sizeof(buf); i += 8)
	{
		uint64_t dword;
		memcpy(&dword, &buf[i], 8);

		status = HAL_FLASH_Program(FLASH_TYPEPROGRAM_DOUBLEWORD,
								   base_addr + i,
								   dword);
		if(status != HAL_OK) return 2;
	}

	HAL_FLASH_Lock();

	return 0;
}

void nvm_load_metadata(void) {
    memcpy(&saved_metadata, (const void *)&_smeta_data, sizeof(Nv_Metadata));
}


void enter_USB_bootloader(void)
{
	FLASH_OBProgramInitTypeDef ob = {0};

	HAL_FLASH_Unlock();
	HAL_FLASH_OB_Unlock();

	HAL_FLASHEx_OBGetConfig(&ob);

	ob.OptionType = OPTIONBYTE_USER;
	ob.USERType   = OB_USER_NBOOT_SEL | OB_USER_NBOOT0 | OB_USER_NBOOT1 | OB_USER_BOR_LEV; // BOOT_LOCK not in USERType on C0 HAL, cleared via FLASH_SECR below
	ob.USERConfig = (ob.USERConfig & ~(FLASH_OPTR_nBOOT_SEL | FLASH_OPTR_nBOOT0 |
										FLASH_OPTR_nBOOT1))
					| FLASH_OPTR_nBOOT_SEL | FLASH_OPTR_nBOOT1;   // nBOOT0=0, nBOOT_SEL=1, nBOOT1=1

	HAL_FLASHEx_OBProgram(&ob);

	CLEAR_BIT(FLASH->SECR, FLASH_SECR_BOOT_LOCK);   // BOOT_LOCK=0, separate register on C0

	HAL_FLASH_OB_Lock();
	HAL_FLASH_Lock();

	HAL_FLASH_OB_Launch();
}

#define NUM_LEDS   32
#define MAX_BRIGHT 15

void set_led_brightness(int32_t value, uint8_t leds[NUM_LEDS], const int32_t in_min, const int32_t in_max) {
    if (value < in_min) value = in_min;
    if (value > in_max) value = in_max;

    memset(leds, 0, NUM_LEDS);

    int32_t offset = value - in_min;      // 0 .. 4096
	int32_t range  = in_max - in_min;     // 4096

	int64_t pos_fp = ((int64_t)offset * (NUM_LEDS - 1) << 16) / range; // 16.16 fixed

	int32_t led_low = (int32_t)(pos_fp >> 16);
	int32_t frac    = (int32_t)(pos_fp & 0xFFFF);        // 0 .. 65535

	if (led_low >= NUM_LEDS - 1) {
		leds[NUM_LEDS - 1] = MAX_BRIGHT;
		return;
	}

	int32_t bright_high = (frac * MAX_BRIGHT + 32768) / 65536; // round, not truncate
	int32_t bright_low  = MAX_BRIGHT - bright_high;

	leds[led_low]     = (uint8_t)bright_low;
	leds[led_low + 1] = (uint8_t)bright_high;
}


/* USER CODE END 0 */

/**
  * @brief  The application entry point.
  * @retval int
  */
int main(void)
{

  /* USER CODE BEGIN 1 */

  /* USER CODE END 1 */

  /* MCU Configuration--------------------------------------------------------*/

  /* Reset of all peripherals, Initializes the Flash interface and the Systick. */
  HAL_Init();

  /* USER CODE BEGIN Init */

  /* USER CODE END Init */

  /* Configure the system clock */
  SystemClock_Config();

  /* USER CODE BEGIN SysInit */

  /* USER CODE END SysInit */

  /* Initialize all configured peripherals */
  MX_GPIO_Init();
  MX_DMA_Init();
  MX_USB_PCD_Init();
  MX_SPI1_Init();
  MX_SPI2_Init();
  MX_USBX_Device_Init();
  MX_TIM1_Init();
  MX_DMAMUX_Init();
  MX_TIM2_Init();
  /* USER CODE BEGIN 2 */

  HAL_TIM_Base_Start(&htim2);

  // lock on power
  HAL_GPIO_WritePin(KEEP_POWER_ON_GPIO_Port, KEEP_POWER_ON_Pin, GPIO_PIN_SET);

  uint32_t tick = 0;
  uint32_t last_active_tick = 0;

  int32_t low_cut_integrator_rate = 10; // max accel magnitude to reduce toward zero each cycle. this is used to remove affects of gravity
  int32_t x_low_cut_integrator = 0;

int32_t x_accel = 0;
int32_t x_accel_interpolated = 0;
int32_t x_accel_previous = 0;
int32_t max_dynamic_accel = 0x7FFF; // acceleration needed to reach ends of image, this should adjust based on the measured max acceleration


int32_t x_accel_magnitude_average = 0;

int32_t image_line = 0;

int32_t max_cycle_accel = 0;
int32_t min_cycle_accel = 0;
uint32_t max_cycle_accel_time = 0;
uint32_t min_cycle_accel_time = 0;

uint32_t estimated_next_edge_time = 0;	// when we think we will be at the end of the shake motion again
uint32_t frame_trigger_time = 0xFFFFFFFF;

int32_t frame_length = 0;

uint32_t current_frame_cycle = 0;
uint32_t frame_display_cycles = 0;

uint32_t current_frame = 0;

int32_t avg_x_accel = 0;
int32_t avg_y_accel = 0;
int32_t avg_z_accel = 0;

const int16_t tap_threshold = 8000;
const uint32_t debounce_time = 50e3;
static uint32_t last_tap_time = 0;
static int16_t tap_accel = 0;
static int8_t tap_direction = 0;
static int8_t consecutive_taps = 0;

static uint8_t save_mode_enable = 0;

static uint8_t level_cal_mode_enable = 0;
static uint8_t x_cal_done = 1;
static uint8_t y_cal_done = 1;

static uint8_t save_metadata = 0;


  memcpy(blank_ccr, default_blank_ccr, sizeof(blank_ccr));


  // Configure and start accelerometer
  HAL_DMA_RegisterCallback(&hdma_spi1_rx, HAL_DMA_XFER_CPLT_CB_ID, Accel_DMA_RxComplete);

  uint8_t spi_buf[2] = {0, 0};

  spi_buf[0] = 0x23;
//  spi_buf[1] = 0b10 << 4;	// 8G range
  spi_buf[1] = (0b1 << 3) | (0b10 << 4);	// 8G range, high res mode
  HAL_GPIO_WritePin(ACCEL_CS_GPIO_Port, ACCEL_CS_Pin, GPIO_PIN_RESET);
  HAL_SPI_Transmit(&hspi1, spi_buf, 2, 1);
  HAL_GPIO_WritePin(ACCEL_CS_GPIO_Port, ACCEL_CS_Pin, GPIO_PIN_SET);
  HAL_Delay(1);

  spi_buf[0] = 0x22;
  spi_buf[1] = 0b1 << 4;	// DRDY1 interrupt
  HAL_GPIO_WritePin(ACCEL_CS_GPIO_Port, ACCEL_CS_Pin, GPIO_PIN_RESET);
  HAL_SPI_Transmit(&hspi1, spi_buf, 2, 1);
  HAL_GPIO_WritePin(ACCEL_CS_GPIO_Port, ACCEL_CS_Pin, GPIO_PIN_SET);
  HAL_Delay(1);

  spi_buf[0] = 0x20;
  spi_buf[1] = 0b111 | (0b1001 << 4);	// xyz enable and ODR set to max
//  spi_buf[1] = 0b111 | (0b0110 << 4);	// xyz enable and ODR set to 200Hz
  HAL_GPIO_WritePin(ACCEL_CS_GPIO_Port, ACCEL_CS_Pin, GPIO_PIN_RESET);
  HAL_SPI_Transmit(&hspi1, spi_buf, 2, 1);
  HAL_GPIO_WritePin(ACCEL_CS_GPIO_Port, ACCEL_CS_Pin, GPIO_PIN_SET);

  // enable NVIC edge trigger for accel data ready signal
  HAL_NVIC_SetPriority(EXTI0_1_IRQn, 1, 0);
  HAL_NVIC_EnableIRQ(EXTI0_1_IRQn);
  // enable NVIC dma for accel data
  HAL_NVIC_SetPriority(DMAMUX1_DMA1_CH4_5_IRQn, 1, 0);
  HAL_NVIC_EnableIRQ(DMAMUX1_DMA1_CH4_5_IRQn);

  EXTI0_1_IRQHandler();	// trigger a read


  enum Led_Mode{
    OFF,
	START_UP,
	MODE_SELECT,
    USB_READY,
    PROGRAMMING,
    SUCCESS,
    ERROR_BMP,
	ERROR_USB,
	FLASH_ERROR,
    POV_DISPLAY_BOTH,
	POV_DISPLAY_USER,
	ACCEL_X_RAW_DISPLAY,
	ACCEL_Y_RAW_DISPLAY,
	ACCEL_Z_RAW_DISPLAY,
	LED_DYNAMIC_DIM_DISPLAY,
	LED_DYNAMIC_DISPLAY,
	LED_ALL_ON,
	LEVEL,
	BALL_SIM,
	SAVE_MODE,
	LEVEL_CAL,
	RESET,
	USB_DFU,
	IMAGE_LINE_DISPLAY,
	POWER_OFF,
	INITIAL_SETUP_CAL,
	END
  } led_mode = START_UP;

  enum Mode{
	  MODE_POV_DISPLAY_BOTH,
	  MODE_POV_DISPLAY_USER_ONLY,
	  MODE_LEVEL,
	  MODE_BALL_SIM,
	  MODE_SAVE_MODE,
	  MODE_LEVEL_CAL,
	  MODE_RESET,
	  MODE_IMAGE_LINE_DISPLAY,
	  MODE_ACCEL_X_RAW_DISPLAY,
	  MODE_ACCEL_Y_RAW_DISPLAY,
	  MODE_ACCEL_Z_RAW_DISPLAY,
	  MODE_LED_DYNAMIC_DIM_DISPLAY,
	  MODE_LED_DYNAMIC_DISPLAY,
	  MODE_USB_DFU,
	  MODE_END

  }	mode = 0;

  nvm_load_metadata();

  // reset nvm metadata if it is not initialized
  if(saved_metadata.user_images[0].mode == 255 || saved_metadata.user_images[1].mode == 255){
	  led_mode = RESET;
  }

  // run setup and calibration on first program
  if(saved_metadata.first_run != 0){
	  led_mode = INITIAL_SETUP_CAL;
  }


  /* Guard: only call _ux_system_tasks_run if USBX fully initialised.
     MX_USBX_Device_Init() runs before this USER CODE section; if the
     memory pool was too small the DCD function pointer stays NULL and
     calling tasks_run causes a hard fault. */
  s_usb_ready = (_ux_system_slave != UX_NULL &&
                 _ux_system_slave->ux_system_slave_dcd.ux_slave_dcd_function != UX_NULL)
                ? 1U : 0U;


  if (!s_usb_ready) {
      led_mode = ERROR_USB;
  }


  uint8_t brightness[32];
  memset(brightness, 0, 32);

  uint32_t next_increment_tick = 0;

  BCM_BuildFrame(brightness);

  // start led DMA update
  LED_Start();

  enum Led_Mode default_led_mode = MODE_SELECT;
  enum Mode last_mode = MODE_POV_DISPLAY_BOTH;

  // use saved mode for default
  if(saved_metadata.startup_mode < MODE_END){
	  mode = saved_metadata.startup_mode;
  }

  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
	tick = TIM2->CNT;	// ticks are in microseconds
	tap_direction = 0;

	if(new_accel_data){
		new_accel_data = 0;

		if(x_accel_magnitude_average > LED_ENABLE_ACCELERATION){
			if(x_accel_previous < 0 && x_accel >= 0){	// transitioned from negative to positive acceleration
				max_cycle_accel = 0;	// clear previous max recorded accel
				estimated_next_edge_time = (min_cycle_accel_time-max_cycle_accel_time) + min_cycle_accel_time;
			}

			if(x_accel_previous > 0 && x_accel <= 0){	// transitioned from positive to negative acceleration
				min_cycle_accel = 0;	// clear previous min recorded accel
				estimated_next_edge_time = (max_cycle_accel_time-min_cycle_accel_time) + max_cycle_accel_time;
			}
		}

		if(estimated_next_edge_time > tick + 500000) estimated_next_edge_time = tick;	// limit to sensible range

		x_accel_previous = x_accel;

		// integrate towards zero to cancel constant acceleration (gravity)
		x_accel = accel_x_raw + x_low_cut_integrator;

		// run integrator
		if(x_accel > low_cut_integrator_rate){
			x_low_cut_integrator -= low_cut_integrator_rate;
		}
		else if(x_accel < -low_cut_integrator_rate){
			x_low_cut_integrator += low_cut_integrator_rate;
		}
		else{
			x_low_cut_integrator -= x_accel;
		}

		// estimate average magnitude (used to detect motion)
		x_accel_magnitude_average += (abs(x_accel) - x_accel_magnitude_average) / 200;


		if(x_accel>max_cycle_accel){
			max_cycle_accel = x_accel;
			max_cycle_accel_time = tick;
		}

		if(x_accel<min_cycle_accel){
			min_cycle_accel = x_accel;
			min_cycle_accel_time = tick;
		}

		avg_x_accel += (int32_t)accel_x_raw - (avg_x_accel >> ACCEL_AVG_SHIFT);
		avg_y_accel += (int32_t)accel_y_raw - (avg_y_accel >> ACCEL_AVG_SHIFT);
		avg_z_accel += (int32_t)accel_z_raw - (avg_z_accel >> ACCEL_AVG_SHIFT);

		tap_accel = accel_y_raw - (avg_y_accel >> ACCEL_AVG_SHIFT);

		if(((abs(tap_accel) > tap_threshold) &&  (tick > (last_tap_time + debounce_time)))){

		    if(tap_accel > 0){
		    	tap_direction = 1;
		    }
		    else{
		    	tap_direction = -1;
		    }

		    if(led_mode != MODE_SELECT){
				if(consecutive_taps == 0){
					consecutive_taps = tap_direction;
				}
				else if(consecutive_taps < 0 && tap_direction < 0){
					consecutive_taps--;
				}
				else if(consecutive_taps > 0 && tap_direction > 0){
					consecutive_taps++;
				}
				else{
					consecutive_taps = 0;
				}
			}

		    last_tap_time = tick;
			last_active_tick = tick;
		}

		if(last_tap_time + 400e3 < tick){
			switch(consecutive_taps){
				case 3:
					led_mode = MODE_SELECT;
				break;
				default:
				break;
			}
			consecutive_taps = 0;
		}

	}

	// interpolate acceleration steps to get a smooth change
	x_accel_interpolated = x_accel_previous * (int32_t)accel_update_period + (x_accel / (int32_t)(tick-accel_last_update_time));
	x_accel_interpolated /= (int32_t)accel_update_period;

	// find line of image to display based on current acceleration and image length
	image_line =  ((x_accel_interpolated + max_dynamic_accel/2) / (max_dynamic_accel / frame_length));

    switch (led_mode) {
      default:
      case OFF:
    	  memset(brightness, 0, 32);	// all off
    	  break;

      case START_UP:
      	  {
			  static uint32_t directions = 0xFFFFFFFF;
			  static int8_t start_up_brightness[16] = {-15, -14, -13, -12, -11, -10, -9, -8, -7, -6, -5, -4, -3, -2, -1, 0};
			  static uint8_t done = 0;
			  if(tick > next_increment_tick){
				  next_increment_tick = tick + 20e3;	// 20ms
				  for(uint8_t i = 0; i < 16; i++){
					  if(start_up_brightness[i] >= 0){
						  brightness[i] = start_up_brightness[i];
						  brightness[31-i] = start_up_brightness[i];
						  done = 0;
					  }
					  else{
						  brightness[i] = 0;
						  brightness[31-i] = 0;
					  }
					  if(start_up_brightness[i] == 8){
						  directions &= ~(0b1 << i);
					  }
					  if(directions & (0b1 << i)){
						  start_up_brightness[i]++;
					  }
					  else if(start_up_brightness[i] >= 0){
						  start_up_brightness[i]--;
					  }
				  }
				  done++;
				  if(done == 5){
					  led_mode = default_led_mode;
				  }
			  }
      	  }
		  break;

      case POWER_OFF:
		  {
			  static uint32_t directions = 0xFFFFFFFF;
			  static int8_t start_up_brightness[16] = {-15, -14, -13, -12, -11, -10, -9, -8, -7, -6, -5, -4, -3, -2, -1, 0};
			  static uint8_t done = 0;
			  if(tick > next_increment_tick){
				  next_increment_tick = tick + 20e3;	// 20ms
				  for(uint8_t i = 0; i < 16; i++){
					  if(start_up_brightness[i] >= 0){
						  brightness[15-i] = start_up_brightness[i];
						  brightness[16+i] = start_up_brightness[i];
						  done = 0;
					  }
					  else{
						  brightness[15-i] = 0;
						  brightness[16+i] = 0;
					  }
					  if(start_up_brightness[i] == 8){
						  directions &= ~(0b1 << i);
					  }
					  if(directions & (0b1 << i)){
						  start_up_brightness[i]++;
					  }
					  else if(start_up_brightness[i] >= 0){
						  start_up_brightness[i]--;
					  }
				  }
				  done++;
				  if(done == 5){

					  if(save_mode_enable){
						  saved_metadata.startup_mode = last_mode;
						  save_metadata = 1;
					  }

					  if(save_metadata){
						  if(update_metadata() && led_mode != FLASH_ERROR){
							  led_mode = FLASH_ERROR;
							  continue;
						  }
					  }

					  HAL_Delay(50);
					  HAL_GPIO_WritePin(KEEP_POWER_ON_GPIO_Port, KEEP_POWER_ON_Pin, GPIO_PIN_RESET);	// turn power off
					  HAL_Delay(500);
					  NVIC_SystemReset();
				  }
			  }
		  }
		  break;

      case MODE_SELECT:
      	  {

			  static const uint32_t inactivity_delay = 3e6;	// 3 seconds
			  static const uint32_t verify_step_delay = 750e3;	// 750ms
			  static uint32_t display_buffer[64];
			  static uint8_t first_cycle = 1;
			  static enum Led_Mode next_mode = MODE_POV_DISPLAY_BOTH;
			  static uint32_t* next_blank_ccr = default_blank_ccr;
			  static uint8_t verify_bar_target = 0;
			  static uint8_t verify_bar = 0;
			  static uint32_t verify_step_time = 0;

			  frame_length = 64;

			  if(first_cycle || tap_direction != 0){

				  if(tap_direction > 0 && !first_cycle){
					  mode++;
					  if(mode == MODE_END) mode = 0;
				  }
				  else if(!first_cycle){
					  if(mode == 0) mode = MODE_END;
					  mode--;
				  }

				  memcpy(blank_ccr, default_blank_ccr, sizeof(blank_ccr));

				  memset(display_buffer, 0, sizeof(display_buffer));	// reset display

				  uint8_t letters[12*3];


				  // fill with spaces
				  memset(letters, 0, sizeof(letters));

				  verify_bar_target = 0;	// default to no verification
				  verify_bar = 0;

				  switch(mode){
				  	  case MODE_POV_DISPLAY_BOTH:
				  		  memcpy(letters, "DISPLAY", 7);
				  		  memcpy(letters+12, "BOTH", 4);
				  		  next_mode = POV_DISPLAY_BOTH;
				  		  next_blank_ccr = default_blank_ccr;
					  break;
				  	  case MODE_POV_DISPLAY_USER_ONLY:
				  		  memcpy(letters, "DISPLAY", 7);
						  memcpy(letters+12, "CUSTOM", 6);
						  next_mode = POV_DISPLAY_USER;
						  next_blank_ccr = default_blank_ccr;
					  break;
				  	  case MODE_LEVEL:
				  		  memcpy(letters, "LEVEL", 5);
				  		  next_mode = LEVEL;
				  		  next_blank_ccr = ball_blank_ccr;
					  break;
				  	  case MODE_BALL_SIM:
				  		  memcpy(letters, "BALL", 4);
				  		  memcpy(letters+12, "SIM", 3);
				  		  next_mode = BALL_SIM;
				  		  next_blank_ccr = ball_blank_ccr;
					  break;
				  	  case MODE_SAVE_MODE:
				  		  memcpy(letters, "SAVE", 4);
				  		  memcpy(letters+12, "MODE", 4);
				  		  if(save_mode_enable){
				  			  memcpy(letters+24, "ON", 2);
				  		  }
				  		  else{
				  			verify_bar_target = 8;
				  		  }
				  		  next_mode = SAVE_MODE;
				  		  next_blank_ccr = default_blank_ccr;
					  break;
				  	  case MODE_LEVEL_CAL:
				  		  memcpy(letters, "LEVEL", 5);
				  		  memcpy(letters+12, "CAL", 3);
				  		  if(level_cal_mode_enable){
							  memcpy(letters+24, "ON", 2);
						  }
						  else{
							verify_bar_target = 8;
						  }
				  		  next_mode = LEVEL_CAL;
				  		  next_blank_ccr = ball_blank_ccr;
				  		  verify_bar_target = 8;
					  break;
				  	  case MODE_RESET:
						  memcpy(letters, "RESET", 5);
						  next_mode = RESET;
						  next_blank_ccr = default_blank_ccr;
						  verify_bar_target = 8;
					  break;
				  	  case MODE_IMAGE_LINE_DISPLAY:
						  memcpy(letters, "IMAGE", 5);
						  memcpy(letters+12, "LINE", 4);
						  memcpy(letters+24, "TEST", 4);
						  next_mode = IMAGE_LINE_DISPLAY;
						  next_blank_ccr = default_blank_ccr;
					  break;
				  	  case MODE_ACCEL_X_RAW_DISPLAY:
				  		  memcpy(letters, "X", 1);
						  memcpy(letters+12, "ACCEL", 5);
						  memcpy(letters+24, "TEST", 4);
						  next_mode = ACCEL_X_RAW_DISPLAY;
						  next_blank_ccr = default_blank_ccr;
					  break;
				  	  case MODE_ACCEL_Y_RAW_DISPLAY:
				  		  memcpy(letters, "Y", 1);
						  memcpy(letters+12, "ACCEL", 5);
						  memcpy(letters+24, "TEST", 4);
						  next_mode = ACCEL_Y_RAW_DISPLAY;
						  next_blank_ccr = default_blank_ccr;
					  break;
				  	  case MODE_ACCEL_Z_RAW_DISPLAY:
				  		  memcpy(letters, "Z", 1);
						  memcpy(letters+12, "ACCEL", 5);
						  memcpy(letters+24, "TEST", 4);
						  next_mode = ACCEL_Z_RAW_DISPLAY;
						  next_blank_ccr = default_blank_ccr;
					  break;
				  	  case MODE_LED_DYNAMIC_DIM_DISPLAY:
				  		  memcpy(letters, "LED", 3);
						  memcpy(letters+12, "DIM", 3);
						  memcpy(letters+24, "TEST", 4);
						  next_mode = LED_DYNAMIC_DIM_DISPLAY;
						  next_blank_ccr = default_blank_ccr;
					  break;
				  	  case MODE_LED_DYNAMIC_DISPLAY:
				  		  memcpy(letters, "LED", 3);
						  memcpy(letters+12, "DOT", 3);
						  memcpy(letters+24, "TEST", 4);
						  next_mode = LED_DYNAMIC_DISPLAY;
						  next_blank_ccr = default_blank_ccr;
				  	  break;
				  	  case MODE_USB_DFU:
				  		  memcpy(letters, "USB", 3);
				  		  memcpy(letters+12, "UPDATE", 6);
				  		memcpy(letters+24, "DANGER", 6);
						  next_mode = USB_DFU;
						  next_blank_ccr = default_blank_ccr;
						  verify_bar_target = 8;
					  break;
				  	  case MODE_END:
					  break;
				  }

				  // build rows
				  for(uint8_t row = 0; row < 3; row++){
					  uint8_t row_data[12*5];
					  uint8_t row_data_idx = 0;
					  memset(row_data, 0, sizeof(row_data));

					  for(uint8_t letter_idx = 0; letter_idx < 12; letter_idx++){
						  uint8_t letter = letters[letter_idx + 12*row];

						  if(letter == 0) break;	// end of row

						  // find pixel data for letter
						  if(letter >= 65 && letter <= 90){	// A-Z
							  letter -= 65;
						  }
						  else{
							  letter = 0;	// unknown character
						  }

						  uint8_t current_letter_idx = 0;
						  for(uint8_t pixel_column = 0; pixel_column < sizeof(letter_pixels); pixel_column++){
							  if(current_letter_idx == letter){
								  row_data[row_data_idx] = letter_pixels[pixel_column] & 0x7F;	// only save lower 7 letter bits
								  row_data_idx++;
							  }

							  if(letter_pixels[pixel_column] & (0b1<<7)) current_letter_idx++;	// increment to next character on index bit

							  if(current_letter_idx > letter){
								  row_data[row_data_idx] = 0;	// add a space between letters
								  row_data_idx++;
								  break;
							  };
						  }
					  }

					  // move row data into actual frame buffer
					  uint8_t start_offset = 64/2 - row_data_idx/2;
					  if(start_offset > 31) start_offset = 0;	// limit in the event of an overflow
					  uint8_t end_offset = start_offset+row_data_idx;
					  if(end_offset > 64) end_offset = 64;	// limit in the event of an overflow

					  for(uint8_t col = start_offset; col < end_offset; col++){
						  display_buffer[col] |= row_data[col-start_offset] << (8*(3-row));
					  }
				  }
				  current_frame = 0;

			  }

			  // display image if image_line is valid and motion is detected
			  else if(x_accel_magnitude_average > LED_ENABLE_ACCELERATION){

				  last_tap_time = tick;	// used to force the debounce delay after shaking stops before accepting new taps

				  if(image_line >= 0 && image_line < frame_length){
					  uint32_t column_data = display_buffer[image_line];
					  for(uint8_t i = 0; i < 32; i++){
						  brightness[i] = column_data & (0b1 << i) ? 15 : 0;
					  }
					  if(verify_bar_target != 0){
						  if(verify_bar > 8) verify_bar = 8;
						  for(uint8_t i = 0; i < 8-verify_bar; i++){
							  brightness[i] = 8;
						  }
					  }

				  }
				  else{
					  memset(brightness, 0, 32);
				  }

				  last_active_tick = tick;
			  }
			  else{
				  memset(brightness, 0, 32);	// off
				  brightness[mode] = 5;	// indicate mode
				  verify_step_time = tick + verify_step_delay;
				  verify_bar = 0;
			  }

			  // modes that require verification
			  if(mode == MODE_SAVE_MODE || mode == MODE_RESET || mode == MODE_USB_DFU || mode == MODE_LEVEL_CAL){
				  first_cycle = 0;
				  if(tick > verify_step_time){
					  verify_step_time = tick + verify_step_delay;
					  verify_bar++;
					  if(verify_bar > verify_bar_target){
						  led_mode = next_mode;
						  first_cycle = 1;	// force rebuilding image
						  memset(brightness, 0, 32);	// off
					  }
				  }
			  }
			  else{
				  if(last_tap_time + inactivity_delay < tick || first_cycle){
					  // switch to selected mode
					  led_mode = next_mode;
					  last_mode = mode;
					  memset(brightness, 0, 32);	// off
					  memcpy(blank_ccr, next_blank_ccr, sizeof(blank_ccr));
				  }
				  first_cycle = 0;
			  }

      	  }
    	  break;

      case BALL_SIM:
      	  {
      		  static uint32_t last_run_tick = 0;
      		  static int16_t last_leds = 0;
      		  if(tick > last_run_tick + 2e3){	// far past, reset timing
      			last_run_tick = tick;	// update to now
      		  }
      		  if(tick > last_run_tick + 1e3){
      			  last_run_tick += 1000;
      			  memset(brightness, 0, 32);
      			  uint16_t leds = ball_sim_tick(accel_y_raw);
      			  ball_to_leds(leds, brightness);
      			  if(abs(last_leds - (int16_t)leds) > 16){
      				last_leds = leds;
      				last_active_tick = tick;
      			  }
      		  }
      	  }
    	  break;

      case IMAGE_LINE_DISPLAY:
      	  {
      		frame_length = 64;
      		// display image if image_line is valid and motion is detected
			if(x_accel_magnitude_average > LED_ENABLE_ACCELERATION){
				if(image_line >= 0 && image_line < frame_length){
					memset(brightness, 0, 32);
					brightness[image_line/2] = 8;
				}
				else{
					memset(brightness, 0, 32);
				}

				last_active_tick = tick;
			}
			else{
				memset(brightness, 0, 32);	// off
			}

      	  }
      	  break;

      case ACCEL_X_RAW_DISPLAY:
      case ACCEL_Y_RAW_DISPLAY:
      case ACCEL_Z_RAW_DISPLAY:
      	  {
      		  int8_t led_bar = 0;

      		  if(led_mode == ACCEL_X_RAW_DISPLAY){
      			  led_bar = accel_x_raw / 2048;
      		  }
      		  else if(led_mode == ACCEL_Y_RAW_DISPLAY){
      			  led_bar = accel_y_raw / 2048;
      		  }
      		  else if(led_mode == ACCEL_Z_RAW_DISPLAY){
      			  led_bar = accel_z_raw / 2048;
			  }

      		  memset(brightness, 0, 32);
      		  for(uint8_t i = 0; i < 16; i++){
      			  if(led_bar > i){
      				  brightness[i+16] = 8;
      			  }
      			  else if(led_bar < -i){
      				brightness[15-i] = 8;
      			  }
      		  }
      		  if(abs(led_bar) > 4){
      			  last_active_tick = tick;
      		  }
      	  }
      	  break;

      case LED_DYNAMIC_DISPLAY:
    	  static uint8_t direction = 1;
    	  static uint8_t index_offset = 0;
    	  if(tick > next_increment_tick){
    		  next_increment_tick = tick + 50e3;	// 50ms
    		  if(direction){
    			  index_offset++;
    			  if(index_offset == 31){
    				  direction = 0;
    			  }
    		  }
    		  else{
    			  index_offset--;
				  if(index_offset == 0){
					  direction = 1;
				  }
    		  }
    		  for(uint8_t i = 0; i < 32; i++){
    			  if(i == index_offset){
    				  brightness[i] = 15;
    			  }
    			  else{
    				  brightness[i] = 0;
    			  }
    		  }
    	  }
    	  break;

      case LED_DYNAMIC_DIM_DISPLAY:
      	  {
			  static uint32_t directions = 0xFFFFFFFF;
			  static uint8_t dynamic_dim_brightness[32] = {0, 0, 1, 1, 2, 2, 3, 3, 4, 4, 5, 5, 6, 6, 7, 7, 8, 8, 9, 9, 10, 10, 11, 11, 12, 12, 13, 13, 14, 14, 15, 15};
			  if(tick > next_increment_tick){
				  next_increment_tick = tick + 50e3;	// 50ms
				  memcpy(brightness, dynamic_dim_brightness, 32);
				  for(uint8_t i = 0; i < 32; i++){
					  if(dynamic_dim_brightness[i] == 0){
						  directions |= 0b1 << i;
					  }
					  else if(dynamic_dim_brightness[i] == 15){
						  directions &= ~(0b1 << i);
					  }
					  if(directions & (0b1 << i)){
						  dynamic_dim_brightness[i]++;
					  }
					  else{
						  dynamic_dim_brightness[i]--;
					  }
				  }
			  }
      	  }
    	  break;

      case POV_DISPLAY_BOTH:
      case POV_DISPLAY_USER:
    	  static const Image_Metadata* active_image_metadata;
    	  static uint8_t image_index = 0;
    	  static uint8_t image_cycles = 0;
    	  static uint16_t frame_start_offset = 0;

    	  switch(image_index){
			  case 0:
				  active_image_metadata = &(eo_metadata[0]);
			  break;
			  case 1:
				  active_image_metadata = &(eo_metadata[1]);
			  break;
			  case 2:
				  active_image_metadata = &(saved_metadata.user_images[0]);
			  break;
			  case 3:
				  active_image_metadata = &(saved_metadata.user_images[1]);
			  break;
    	  }

    	  if(frame_display_cycles != active_image_metadata->display_cycles){
    		  frame_display_cycles = active_image_metadata->display_cycles;
    		  current_frame_cycle = frame_display_cycles;
    	  }
    	  frame_length = active_image_metadata->max_frame_columns;

    	  uint8_t valid_image_line = 0;
    	  uint8_t left_offset = (frame_length - active_image_metadata->frame_columns[current_frame]) / 2;
    	  uint8_t right_offset = active_image_metadata->frame_columns[current_frame] + left_offset;
    	  if(image_line >= left_offset && image_line < right_offset){
    		  valid_image_line = 1;
    	  }

    	  // display image if image_line is valid and motion is detected
    	  if(valid_image_line && (x_accel_magnitude_average > LED_ENABLE_ACCELERATION) && active_image_metadata->mode != NONE){

    		  int16_t offset_image_line = image_line - left_offset;

    		  uint8_t* col_ptr;

    		  switch(active_image_metadata->mode){
    		  	  case ONE_BIT:
    		  		  uint32_t column_bits = 0;

    		  		  switch(active_image_metadata->column_height){
    		  		  	  case 32:
    		  		  		  col_ptr = (active_image_metadata->image_data) + frame_start_offset*4 + offset_image_line*4;
    		  		  		  memcpy(&column_bits, col_ptr, 4);
    		  		  		  for(uint8_t i = 0; i < 32; i++){
    		  		  			  brightness[i] = (column_bits & (0b1 << i)) ? 15 : 0;
    		  		  		  }
						  break;

    		  		  	  case 16:
    		  		  		  col_ptr = (active_image_metadata->image_data) + frame_start_offset*2 + offset_image_line*2;
							  memcpy(&column_bits, col_ptr, 2);
							  for(uint8_t i = 0; i < 32; i++){
								  brightness[i] = (column_bits & (0b1 << (i>>1))) ? 15 : 0;
							  }
						  break;

    		  		  	  case 8:
    		  		  		  col_ptr = (active_image_metadata->image_data) + frame_start_offset + offset_image_line;
    		  		  		  memcpy(&column_bits, col_ptr, 1);
    		  		  		  for(uint8_t i = 0; i < 32; i++){
    		  		  			  brightness[i] = (column_bits & (0b1 << (i>>2))) ? 15 : 0;
    		  		  		  }
    		  		  	  break;

    		  		  	  default:
    		  		  	  break;
    		  		  }
				  break;

				  case FOUR_BIT:
					  switch(active_image_metadata->column_height){
					  case 32:
						  col_ptr = (active_image_metadata->image_data) + frame_start_offset*4*4 + offset_image_line*4*4;
						  for(uint8_t i = 0; i < 32; i += 2){
							  uint8_t pixel_pair_brightness = *(col_ptr + (i>>1));

							  brightness[i] = pixel_pair_brightness & 0xF;
							  brightness[i+1] = pixel_pair_brightness >> 4;
						  }

					  case 16:
						  col_ptr = (active_image_metadata->image_data) + frame_start_offset*2*4 + offset_image_line*2*4;
						  for(uint8_t i = 0; i < 32; i += 4){
							  uint8_t pixel_pair_brightness = *(col_ptr + (i>>2));

							  brightness[i] = pixel_pair_brightness & 0xF;
							  brightness[i+1] = pixel_pair_brightness & 0xF;
							  brightness[i+2] = pixel_pair_brightness >> 4;
							  brightness[i+3] = pixel_pair_brightness >> 4;
						  }
					  break;

					  case 8:
						  col_ptr = (active_image_metadata->image_data) + frame_start_offset*1*4 + offset_image_line*1*4;
						  for(uint8_t i = 0; i < 32; i += 8){
							  uint8_t pixel_pair_brightness = *(col_ptr + (i>>3));

							  brightness[i] = pixel_pair_brightness & 0xF;
							  brightness[i+1] = pixel_pair_brightness & 0xF;
							  brightness[i+2] = pixel_pair_brightness & 0xF;
							  brightness[i+3] = pixel_pair_brightness & 0xF;
							  brightness[i+4] = pixel_pair_brightness >> 4;
							  brightness[i+5] = pixel_pair_brightness >> 4;
							  brightness[i+6] = pixel_pair_brightness >> 4;
							  brightness[i+7] = pixel_pair_brightness >> 4;
						  }
					  break;

					  default:
					  break;
					  }
				  break;

				  default:
				  break;
    		  }

    		  last_active_tick = tick;
		  }
		  else{
			  memset(brightness, 0, 32);	// all off
			  if(x_accel_magnitude_average < LED_ENABLE_ACCELERATION){
				  frame_start_offset = 0;
				  current_frame = 0;
				  current_frame_cycle = frame_display_cycles;
				  frame_trigger_time = tick;
			  }
		  }

    	  if(active_image_metadata->mode == NONE){
    		  image_cycles = 0;
			  image_index++;
			  if(image_index > 3){
				  if(led_mode == POV_DISPLAY_BOTH){
					  image_index = 0;
				  }
				  else{
					  image_index = 2;
				  }
			  }
    	  }
    	  // increment frame index
    	  else if(tick >= frame_trigger_time && current_frame_cycle == 0){	// reset counter and increment to next frame index
    		  frame_start_offset += active_image_metadata->frame_columns[current_frame];
    		  current_frame++;
    		  current_frame_cycle = frame_display_cycles;

    		  // handle end of image/animation
    		  if(current_frame >= active_image_metadata->frame_count){
    			  current_frame = 0;		// loop back to first frame once end of image is reached
    			  frame_start_offset = 0;
    			  image_cycles++;

    			  // image/animation complete, switch to next
    			  if(image_cycles >= active_image_metadata->cycle_count){
    				  image_cycles = 0;
    				  image_index++;
    				  if(image_index > 3){
						  if(led_mode == POV_DISPLAY_BOTH){
							  image_index = 0;
						  }
						  else{
							  image_index = 2;
						  }
    				  }
    			  }
    			  current_frame_cycle = frame_display_cycles;
    		  }
    	  }

    	  // decrement frame view cycles
    	  if(tick >= frame_trigger_time && current_frame_cycle != 0 && frame_trigger_time < estimated_next_edge_time){
    		  frame_trigger_time = estimated_next_edge_time;
    		  current_frame_cycle--;
    	  }

    	  if(current_frame_cycle == 0) frame_trigger_time = estimated_next_edge_time;

    	  break;

      case LED_ALL_ON:
    	  memset(brightness, 15, 32);	// all on
    	  break;

      case LEVEL_CAL:
    	  level_cal_mode_enable = 1;
    	  x_cal_done = 0;
    	  y_cal_done = 0;
    	  led_mode = LEVEL;
    	  break;

      case LEVEL:
      	  {

			static int16_t tilt[128];
			static uint8_t tilt_idx = 0;
			static int16_t last_tilt = 0;
			uint8_t x_active = 0;
			if(abs(avg_x_accel) > abs(avg_y_accel)){
			  // assume x is the vertical axis
			  tilt[tilt_idx] = avg_y_accel >> (ACCEL_AVG_SHIFT);
			}
			else{
			  // assume y is the vertical axis
			  tilt[tilt_idx] = avg_x_accel >> (ACCEL_AVG_SHIFT);
			  x_active = 1;
			}

			int32_t sum = 0;
			for(uint8_t i = 0; i < 128; i++){
			  sum += (int32_t)tilt[i];
			}

			tilt_idx++;
			if(tilt_idx == 128){
			  tilt_idx = 0;
			}

			memset(brightness, 0, 32);	// all off

			// calibration mode
			if(level_cal_mode_enable){
				if((tick > last_active_tick + 3e6) && (sum > -15*128) && (sum < 15*128)){	// stable for 3 seconds and near expected zero
					memset(brightness, 1, 32);	// all dim
					if(x_active && (x_cal_done == 0)){
						x_cal_done = 1;
						saved_metadata.accel_x_zero = sum;

					}
					if((x_active == 0) && (y_cal_done == 0)){
						y_cal_done = 1;
						saved_metadata.accel_y_zero = sum;

					}
					if(x_cal_done && y_cal_done){
						level_cal_mode_enable = 0;
						save_metadata = 1;	// save to flash at power off
					}
				}
			}


			if(x_active && x_cal_done){
				sum = sum - saved_metadata.accel_x_zero;
			}
			if((x_active == 0) && y_cal_done){
				sum = sum - saved_metadata.accel_y_zero;
			}


			set_led_brightness(sum, brightness, -2048*16, 2048*16);


			sum = sum / 128;


			if(sum > 15) sum = 15;
			if(sum < -15) sum = -15;

			if(abs(sum - last_tilt) > 2){
			  last_active_tick = tick;	// keep awake while level is moving
			  last_tilt = sum;
			}

      	  }
    	  break;

      case SAVE_MODE:
      	  {
      		  save_mode_enable = 1;
      		  led_mode = default_led_mode;
      	  }
      	  break;

      case RESET:
      	  {
      		// clear metadata except for accel calibration
      		saved_metadata.first_run = 0;
      		saved_metadata.startup_mode = 0;
      		Image_Metadata blank_image_metadata;
      		blank_image_metadata.cycle_count = 0;
      		blank_image_metadata.display_cycles = 0;
      		blank_image_metadata.column_height = 0;
      		blank_image_metadata.max_frame_columns = 0;
      		memset(blank_image_metadata.frame_columns, 0, sizeof(blank_image_metadata.frame_columns));
      		blank_image_metadata.frame_count = 0;
      		blank_image_metadata.image_data = 0;
      		blank_image_metadata.image_data_length = 0;
      		blank_image_metadata.mode = 0;
      		saved_metadata.user_images[0] = blank_image_metadata;
      		saved_metadata.user_images[1] = blank_image_metadata;

      		// clear user image data
      		FLASH_EraseInitTypeDef erase = {
				.TypeErase = FLASH_TYPEERASE_PAGES,
				.Page      = ((uint32_t)&_suser_image_data - FLASH_BASE) / FLASH_PAGE_SIZE,
				.NbPages   = 2
			};

			uint32_t page_err = 0;

			HAL_FLASH_Unlock();
			HAL_StatusTypeDef status = HAL_FLASHEx_Erase(&erase, &page_err);
			HAL_FLASH_Lock();
			if(status != HAL_OK){
				led_mode = FLASH_ERROR;
			}
			else{
				led_mode = POWER_OFF;
			}

			update_metadata();
      	  }
      	  break;

      case USB_DFU:
		  {
			  ux_device_stack_disconnect();          // soft-disconnect, host sees device drop
			  memset(brightness, 2, 32);	// all dim
//			  HAL_Delay(5);                          // let any pending ACK/status stage flush
//			  HAL_PCD_Stop(&hpcd_USB_DRD_FS);
//			  HAL_PCD_DeInit(&hpcd_USB_DRD_FS);

			  led_mode = POWER_OFF;

			  enter_USB_bootloader();
		  }
		  break;

      case INITIAL_SETUP_CAL:
		  {
			  static int8_t xtilt[128];
			  static int8_t ytilt[128];
			  static uint8_t tilt_idx = 0;
			  static uint8_t cal_cycles = 0;

			  ytilt[tilt_idx] = avg_y_accel >> (ACCEL_AVG_SHIFT+4);
			  xtilt[tilt_idx] = avg_x_accel >> (ACCEL_AVG_SHIFT+4);

			  tilt_idx++;
			  if(tilt_idx == 128){
				  tilt_idx = 0;
				  cal_cycles++;
			  }

			  if(cal_cycles == 200){
				  int32_t xsum = 0;
				  int32_t ysum = 0;
				  for(uint8_t i = 0; i < 128; i++){
					  xsum += (int32_t)xtilt[i];
					  ysum += (int32_t)ytilt[i];
				  }

				  saved_metadata.accel_x_zero = xsum;
				  saved_metadata.accel_y_zero = ysum;

				  save_metadata = 1;	// save to flash at power off

				  led_mode = RESET;

			  }

			  memset(brightness, 2, 32);	// all dim

		  }
		  break;
    }


    // reset animation variables when card is no longer being shaken
    if(x_accel_magnitude_average < LED_ENABLE_ACCELERATION){
		current_frame = 0;	// reset frame counter so we start at first frame of animation
	}

    if((last_active_tick + POWER_OFF_DELAY*1e6 < tick) || tick > MAX_POWER_OFF_DELAY*1e6){	// power off after inactive for too long
    	led_mode = POWER_OFF;
    }

    BCM_BuildFrame(brightness);
    /* ----------------------------------------------------------------
     * BMP processing (deferred from write callback so USB stack
     * finishes its current transaction first).
     * ---------------------------------------------------------------- */
    /*if (g_bmp_pending) {
        g_bmp_pending   = 0U;
        hold_start      = now;
        g_usb_led_state = 2U;

        uint8_t  frame_idx   = g_bmp_frame_idx;
        uint32_t data_sector = g_bmp_ram_sector;
        uint32_t bmp_size    = g_bmp_ram_size;

        const uint8_t *disk  = usbd_get_ram_disk_ptr();
        const uint8_t *bmp   = disk + data_sector * 512U;

        bmp_result_t conv = pov_bmp_convert(bmp, bmp_size, s_frame_buf);
        if (conv != BMP_OK) {
            g_usb_led_state = 4U;
        } else if (frame_idx < POV_FRAME_RESERVED) {
            g_usb_led_state = 4U;
        } else {
            int wr = pov_frame_write(frame_idx, s_frame_buf);
            g_usb_led_state = (wr == 0) ? 3U : 4U;
        }
    }
    */


    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */
    if (s_usb_ready)
        _ux_system_tasks_run();
  }
  /* USER CODE END 3 */
}

/**
  * @brief System Clock Configuration
  * @retval None
  */
void SystemClock_Config(void)
{
  RCC_OscInitTypeDef RCC_OscInitStruct = {0};
  RCC_ClkInitTypeDef RCC_ClkInitStruct = {0};

  __HAL_FLASH_SET_LATENCY(FLASH_LATENCY_1);

  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI48;
  RCC_OscInitStruct.HSI48State = RCC_HSI48_ON;

  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }

  /** Initializes the CPU, AHB and APB buses clocks
  */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_HSIUSB48;
  RCC_ClkInitStruct.SYSCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_HCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_APB1_DIV1;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_1) != HAL_OK)
  {
    Error_Handler();
  }
}

/**
  * @brief DMAMUX Initialization Function
  * @param None
  * @retval None
  */
static void MX_DMAMUX_Init(void)
{

  /* USER CODE BEGIN DMAMUX_Init 0 */

  /* USER CODE END DMAMUX_Init 0 */

  /* USER CODE BEGIN DMAMUX_Init 1 */

  /* USER CODE END DMAMUX_Init 1 */
  /* USER CODE BEGIN DMAMUX_Init 2 */
	HAL_DMA_MuxSyncConfigTypeDef sync = {
		.SyncSignalID  = HAL_DMAMUX1_SYNC_EXTI3,
		.SyncPolarity  = HAL_DMAMUX_SYNC_RISING,
		.SyncEnable    = ENABLE,
		.EventEnable   = DISABLE,
		.RequestNumber = 2,
	};
	HAL_DMAEx_ConfigMuxSync(&hdma_spi2_tx, &sync);
  /* USER CODE END DMAMUX_Init 2 */

}

/**
  * @brief SPI1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_SPI1_Init(void)
{

  /* USER CODE BEGIN SPI1_Init 0 */

  /* USER CODE END SPI1_Init 0 */

  /* USER CODE BEGIN SPI1_Init 1 */

  /* USER CODE END SPI1_Init 1 */
  /* SPI1 parameter configuration*/
  hspi1.Instance = SPI1;
  hspi1.Init.Mode = SPI_MODE_MASTER;
  hspi1.Init.Direction = SPI_DIRECTION_2LINES;
  hspi1.Init.DataSize = SPI_DATASIZE_8BIT;
  hspi1.Init.CLKPolarity = SPI_POLARITY_HIGH;
  hspi1.Init.CLKPhase = SPI_PHASE_2EDGE;
  hspi1.Init.NSS = SPI_NSS_SOFT;
  hspi1.Init.BaudRatePrescaler = SPI_BAUDRATEPRESCALER_8;
  hspi1.Init.FirstBit = SPI_FIRSTBIT_MSB;
  hspi1.Init.TIMode = SPI_TIMODE_DISABLE;
  hspi1.Init.CRCCalculation = SPI_CRCCALCULATION_DISABLE;
  hspi1.Init.CRCPolynomial = 7;
  hspi1.Init.CRCLength = SPI_CRC_LENGTH_DATASIZE;
  hspi1.Init.NSSPMode = SPI_NSS_PULSE_DISABLE;
  if (HAL_SPI_Init(&hspi1) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN SPI1_Init 2 */

  /* USER CODE END SPI1_Init 2 */

}

/**
  * @brief SPI2 Initialization Function
  * @param None
  * @retval None
  */
static void MX_SPI2_Init(void)
{

  /* USER CODE BEGIN SPI2_Init 0 */

  /* USER CODE END SPI2_Init 0 */

  /* USER CODE BEGIN SPI2_Init 1 */

  /* USER CODE END SPI2_Init 1 */
  /* SPI2 parameter configuration*/
  hspi2.Instance = SPI2;
  hspi2.Init.Mode = SPI_MODE_MASTER;
  hspi2.Init.Direction = SPI_DIRECTION_2LINES;
  hspi2.Init.DataSize = SPI_DATASIZE_16BIT;
  hspi2.Init.CLKPolarity = SPI_POLARITY_LOW;
  hspi2.Init.CLKPhase = SPI_PHASE_1EDGE;
  hspi2.Init.NSS = SPI_NSS_SOFT;
  hspi2.Init.BaudRatePrescaler = SPI_BAUDRATEPRESCALER_2;
  hspi2.Init.FirstBit = SPI_FIRSTBIT_LSB;
  hspi2.Init.TIMode = SPI_TIMODE_DISABLE;
  hspi2.Init.CRCCalculation = SPI_CRCCALCULATION_DISABLE;
  hspi2.Init.CRCPolynomial = 7;
  hspi2.Init.CRCLength = SPI_CRC_LENGTH_DATASIZE;
  hspi2.Init.NSSPMode = SPI_NSS_PULSE_ENABLE;
  if (HAL_SPI_Init(&hspi2) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN SPI2_Init 2 */

  /* USER CODE END SPI2_Init 2 */

}

/**
  * @brief TIM1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM1_Init(void)
{

  /* USER CODE BEGIN TIM1_Init 0 */

  /* USER CODE END TIM1_Init 0 */

  TIM_ClockConfigTypeDef sClockSourceConfig = {0};
  TIM_MasterConfigTypeDef sMasterConfig = {0};
  TIM_OC_InitTypeDef sConfigOC = {0};
  TIM_BreakDeadTimeConfigTypeDef sBreakDeadTimeConfig = {0};

  /* USER CODE BEGIN TIM1_Init 1 */

  /* USER CODE END TIM1_Init 1 */
  htim1.Instance = TIM1;
  htim1.Init.Prescaler = 0;
  htim1.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim1.Init.Period = 127;
  htim1.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim1.Init.RepetitionCounter = 0;
  htim1.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_ENABLE;
  if (HAL_TIM_Base_Init(&htim1) != HAL_OK)
  {
    Error_Handler();
  }
  sClockSourceConfig.ClockSource = TIM_CLOCKSOURCE_INTERNAL;
  if (HAL_TIM_ConfigClockSource(&htim1, &sClockSourceConfig) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_TIM_PWM_Init(&htim1) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_OC3REF;
  sMasterConfig.MasterOutputTrigger2 = TIM_TRGO2_OC4REF;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim1, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  sConfigOC.OCMode = TIM_OCMODE_PWM1;
  sConfigOC.Pulse = 2;
  sConfigOC.OCPolarity = TIM_OCPOLARITY_HIGH;
  sConfigOC.OCNPolarity = TIM_OCNPOLARITY_HIGH;
  sConfigOC.OCFastMode = TIM_OCFAST_DISABLE;
  sConfigOC.OCIdleState = TIM_OCIDLESTATE_RESET;
  sConfigOC.OCNIdleState = TIM_OCNIDLESTATE_RESET;
  if (HAL_TIM_PWM_ConfigChannel(&htim1, &sConfigOC, TIM_CHANNEL_1) != HAL_OK)
  {
    Error_Handler();
  }
  sConfigOC.Pulse = 100;
  if (HAL_TIM_PWM_ConfigChannel(&htim1, &sConfigOC, TIM_CHANNEL_2) != HAL_OK)
  {
    Error_Handler();
  }
  sConfigOC.OCMode = TIM_OCMODE_PWM2;
  sConfigOC.Pulse = 2;
  if (HAL_TIM_PWM_ConfigChannel(&htim1, &sConfigOC, TIM_CHANNEL_4) != HAL_OK)
  {
    Error_Handler();
  }
  sBreakDeadTimeConfig.OffStateRunMode = TIM_OSSR_DISABLE;
  sBreakDeadTimeConfig.OffStateIDLEMode = TIM_OSSI_DISABLE;
  sBreakDeadTimeConfig.LockLevel = TIM_LOCKLEVEL_OFF;
  sBreakDeadTimeConfig.DeadTime = 0;
  sBreakDeadTimeConfig.BreakState = TIM_BREAK_DISABLE;
  sBreakDeadTimeConfig.BreakPolarity = TIM_BREAKPOLARITY_HIGH;
  sBreakDeadTimeConfig.BreakFilter = 0;
  sBreakDeadTimeConfig.BreakAFMode = TIM_BREAK_AFMODE_INPUT;
  sBreakDeadTimeConfig.Break2State = TIM_BREAK2_DISABLE;
  sBreakDeadTimeConfig.Break2Polarity = TIM_BREAK2POLARITY_HIGH;
  sBreakDeadTimeConfig.Break2Filter = 0;
  sBreakDeadTimeConfig.Break2AFMode = TIM_BREAK_AFMODE_INPUT;
  sBreakDeadTimeConfig.AutomaticOutput = TIM_AUTOMATICOUTPUT_DISABLE;
  if (HAL_TIMEx_ConfigBreakDeadTime(&htim1, &sBreakDeadTimeConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM1_Init 2 */

  /* USER CODE END TIM1_Init 2 */
  HAL_TIM_MspPostInit(&htim1);

}

/**
  * @brief TIM2 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM2_Init(void)
{

  /* USER CODE BEGIN TIM2_Init 0 */

  /* USER CODE END TIM2_Init 0 */

  TIM_ClockConfigTypeDef sClockSourceConfig = {0};
  TIM_MasterConfigTypeDef sMasterConfig = {0};

  /* USER CODE BEGIN TIM2_Init 1 */

  /* USER CODE END TIM2_Init 1 */
  htim2.Instance = TIM2;
  htim2.Init.Prescaler = 47;
  htim2.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim2.Init.Period = 4294967295;
  htim2.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim2.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  if (HAL_TIM_Base_Init(&htim2) != HAL_OK)
  {
    Error_Handler();
  }
  sClockSourceConfig.ClockSource = TIM_CLOCKSOURCE_INTERNAL;
  if (HAL_TIM_ConfigClockSource(&htim2, &sClockSourceConfig) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim2, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM2_Init 2 */

  /* USER CODE END TIM2_Init 2 */

}

/**
  * @brief USB Initialization Function
  * @param None
  * @retval None
  */
static void MX_USB_PCD_Init(void)
{

  /* USER CODE BEGIN USB_Init 0 */

  /* USER CODE END USB_Init 0 */

  /* USER CODE BEGIN USB_Init 1 */

  /* USER CODE END USB_Init 1 */
  hpcd_USB_DRD_FS.Instance = USB_DRD_FS;
  hpcd_USB_DRD_FS.Init.dev_endpoints = 8;
  hpcd_USB_DRD_FS.Init.speed = USBD_FS_SPEED;
  hpcd_USB_DRD_FS.Init.phy_itface = PCD_PHY_EMBEDDED;
  hpcd_USB_DRD_FS.Init.Sof_enable = DISABLE;
  hpcd_USB_DRD_FS.Init.low_power_enable = DISABLE;
  hpcd_USB_DRD_FS.Init.lpm_enable = DISABLE;
  hpcd_USB_DRD_FS.Init.battery_charging_enable = DISABLE;
  hpcd_USB_DRD_FS.Init.vbus_sensing_enable = DISABLE;
  hpcd_USB_DRD_FS.Init.bulk_doublebuffer_enable = DISABLE;
  hpcd_USB_DRD_FS.Init.iso_singlebuffer_enable = DISABLE;
  if (HAL_PCD_Init(&hpcd_USB_DRD_FS) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN USB_Init 2 */
  /* BTable for 8 endpoints occupies PMA 0x00-0x3F (8 * 8 bytes = 64 bytes).
     All data buffers must start at 0x40 or above to avoid overlapping it. */
  HAL_PCDEx_PMAConfig(&hpcd_USB_DRD_FS, 0x00, PCD_SNG_BUF, 0x40);  /* EP0 OUT */
  HAL_PCDEx_PMAConfig(&hpcd_USB_DRD_FS, 0x80, PCD_SNG_BUF, 0x80);  /* EP0 IN  */
  HAL_PCDEx_PMAConfig(&hpcd_USB_DRD_FS, 0x81, PCD_SNG_BUF, 0xC0);  /* EP1 IN  (MSC bulk) */
  HAL_PCDEx_PMAConfig(&hpcd_USB_DRD_FS, 0x02, PCD_SNG_BUF, 0x100); /* EP2 OUT (MSC bulk) */
  /* USER CODE END USB_Init 2 */

}

/**
  * Enable DMA controller clock
  */
static void MX_DMA_Init(void)
{

  /* DMA controller clock enable */
  __HAL_RCC_DMA1_CLK_ENABLE();

  /* DMA interrupt init */
  /* DMA1_Channel1_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(DMA1_Channel1_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(DMA1_Channel1_IRQn);
  /* DMA1_Channel2_3_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(DMA1_Channel2_3_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(DMA1_Channel2_3_IRQn);
  /* DMAMUX1_DMA1_CH4_5_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(DMAMUX1_DMA1_CH4_5_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(DMAMUX1_DMA1_CH4_5_IRQn);

}

/**
  * @brief GPIO Initialization Function
  * @param None
  * @retval None
  */
static void MX_GPIO_Init(void)
{
  GPIO_InitTypeDef GPIO_InitStruct = {0};
  /* USER CODE BEGIN MX_GPIO_Init_1 */

  /* USER CODE END MX_GPIO_Init_1 */

  /* GPIO Ports Clock Enable */
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(KEEP_POWER_ON_GPIO_Port, KEEP_POWER_ON_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(ACCEL_CS_GPIO_Port, ACCEL_CS_Pin, GPIO_PIN_SET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOB, GPIO_PIN_6, GPIO_PIN_RESET);

  /*Configure GPIO pin : KEEP_POWER_ON_Pin */
  GPIO_InitStruct.Pin = KEEP_POWER_ON_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(KEEP_POWER_ON_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pin : ACCEL_CS_Pin */
  GPIO_InitStruct.Pin = ACCEL_CS_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_HIGH;
  HAL_GPIO_Init(ACCEL_CS_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pin : ACCEL_INT1_Pin */
  GPIO_InitStruct.Pin = ACCEL_INT1_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_IT_RISING;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(ACCEL_INT1_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pin : ACCEL_INT2_Pin */
  GPIO_InitStruct.Pin = ACCEL_INT2_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(ACCEL_INT2_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pin : PB6 */
  GPIO_InitStruct.Pin = GPIO_PIN_6;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

  /* USER CODE BEGIN MX_GPIO_Init_2 */
  EXTI->EXTICR[1] = (EXTI->EXTICR[1] & ~EXTI_EXTICR1_EXTI3_Msk) | (0x00UL << EXTI_EXTICR1_EXTI3_Pos); /* Port A */
  EXTI->RTSR1 |= EXTI_RTSR1_RT3;   /* rising edge on line 8 */
  EXTI->EMR1  |= EXTI_EMR1_EM3;    /* event enable, not interrupt */
  /* USER CODE END MX_GPIO_Init_2 */
}

/* USER CODE BEGIN 4 */

/* USER CODE END 4 */

/**
  * @brief  This function is executed in case of error occurrence.
  * @retval None
  */
void Error_Handler(void)
{
  /* USER CODE BEGIN Error_Handler_Debug */
  /* User can add his own implementation to report the HAL error return state */
  __disable_irq();
  while (1)
  {
  }
  /* USER CODE END Error_Handler_Debug */
}
#ifdef USE_FULL_ASSERT
/**
  * @brief  Reports the name of the source file and the source line number
  *         where the assert_param error has occurred.
  * @param  file: pointer to the source file name
  * @param  line: assert_param error line source number
  * @retval None
  */
void assert_failed(uint8_t *file, uint32_t line)
{
  /* USER CODE BEGIN 6 */
  /* User can add his own implementation to report the file name and line number,
     ex: printf("Wrong parameters value: file %s on line %d\r\n", file, line) */
  /* USER CODE END 6 */
}
#endif /* USE_FULL_ASSERT */
