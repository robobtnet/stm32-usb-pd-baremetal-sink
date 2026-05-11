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

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include "pd_bm.h"
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
#define PD_APP_TRACE_QUEUE_LEN 512U
#define PD_APP_TIMER_HZ 1000U
#define PD_APP_TIMER_BASE_HZ 1000000U
/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/

COM_InitTypeDef BspCOMInit;

/* USER CODE BEGIN PV */
typedef struct
{
  const char *event;
  uint32_t tick;
  uint32_t a;
  uint32_t b;
  uint32_t c;
  uint32_t d;
} PD_AppTraceRecord;

static uint16_t pd_app_rx_dma_size;
static PD_AppTraceRecord pd_app_trace_queue[PD_APP_TRACE_QUEUE_LEN];
static volatile uint16_t pd_app_trace_head;
static volatile uint16_t pd_app_trace_tail;
static volatile uint16_t pd_app_trace_lost;
TIM_HandleTypeDef htim16;

/*
 * Idle dwell required in DETACHED before we let the trace flush. Fast
 * hard-reset / re-attach cycles sit in DETACHED only briefly (~50 ms in the
 * 240 W cable log), so this threshold keeps printf out of those windows.
 */
#define PD_APP_DETACH_FLUSH_IDLE_MS 500U

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void SystemPower_Config(void);
static void MX_GPIO_Init(void);
static void MX_GPDMA1_Init(void);
static void MX_ICACHE_Init(void);
static void MX_UCPD1_Init(void);
/* USER CODE BEGIN PFP */
static uint32_t PD_App_GetTick(void *user);
static void PD_App_RxDmaStart(uint8_t *buffer, uint16_t size, void *user);
static void PD_App_RxDmaStop(void *user);
static uint16_t PD_App_RxDmaCount(void *user);
static void PD_App_Trace(const char *event, uint32_t a, uint32_t b, uint32_t c, uint32_t d, void *user);
static void PD_App_TraceFlush(void);
static void PD_App_TIM16_Init(void);
/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */

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

  /* Configure the System Power */
  SystemPower_Config();

  /* Configure the system clock */
  SystemClock_Config();

  /* USER CODE BEGIN SysInit */

  /* USER CODE END SysInit */

  /* Initialize all configured peripherals */
  MX_GPIO_Init();
  MX_GPDMA1_Init();
  MX_ICACHE_Init();
  MX_UCPD1_Init();
  /* USER CODE BEGIN 2 */
  PD_App_TIM16_Init();
  if (HAL_TIM_Base_Start_IT(&htim16) != HAL_OK)
  {
    Error_Handler();
  }

  /* USER CODE END 2 */

  /* Initialize leds */
  BSP_LED_Init(LED_GREEN);
  BSP_LED_Init(LED_BLUE);
  BSP_LED_Init(LED_RED);


  static LL_UCPD_InitTypeDef pd_ucpd_init;
  static PD_BM_Config pd_config;

  LL_UCPD_StructInit(&pd_ucpd_init);

  pd_config.ucpd = UCPD1;
  pd_config.ucpd_init = &pd_ucpd_init;
  pd_config.get_tick_ms = PD_App_GetTick;
  pd_config.rx_dma_start = PD_App_RxDmaStart;
  pd_config.rx_dma_stop = PD_App_RxDmaStop;
  pd_config.rx_dma_count = PD_App_RxDmaCount;
  pd_config.trace = PD_App_Trace;
  pd_config.user = NULL;
  pd_config.get_source_cap_interval_ms = 500U;
  pd_config.attach_debounce_ms = 40U;

  pd_config.profiles[0] = (PD_BM_Profile){ 5000U, 500U, 1U };
  pd_config.profiles[1] = (PD_BM_Profile){ 9000U, 1000U, 1U };
  pd_config.profiles[2] = (PD_BM_Profile){ 12000U, 1000U, 1U };
  pd_config.profiles[3] = (PD_BM_Profile){ 15000U, 1000U, 1U };
  pd_config.profiles[4] = (PD_BM_Profile){ 28000U, 3000U, 1U };

  if (PD_BM_Init(&pd_config) == 0U)
  {
    BSP_LED_On(LED_RED);
    Error_Handler();
  }

  BSP_LED_On(LED_BLUE);   /* یعنی init انجام شد */

  HAL_PWREx_DisableUCPDDeadBattery();

  /* Initialize USER push-button, will be used to trigger an interrupt each time it's pressed.*/
  BSP_PB_Init(BUTTON_USER, BUTTON_MODE_EXTI);

  /* Initialize COM1 port (115200, 8 bits (7-bit data + 1 stop bit), no parity */
  BspCOMInit.BaudRate   = 115200;
  BspCOMInit.WordLength = COM_WORDLENGTH_8B;
  BspCOMInit.StopBits   = COM_STOPBITS_1;
  BspCOMInit.Parity     = COM_PARITY_NONE;
  BspCOMInit.HwFlowCtl  = COM_HWCONTROL_NONE;
  if (BSP_COM_Init(COM1, &BspCOMInit) != BSP_ERROR_NONE)
  {
    Error_Handler();
  }

  printf("STM32U575 USB-PD sink start\r\n");

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {

    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */
	  static uint32_t led_last_tick = 0U;
	      static uint8_t pd_reported_profile = 0xFFU;
	      static uint16_t pd_reported_voltage_mv = 0U;
	      static uint16_t pd_reported_current_ma = 0U;
	      static uint32_t pd_detached_since_tick = 0U;
	      static PD_BM_State pd_prev_state = PD_BM_STATE_DETACHED;
	      PD_BM_State pd_state;
	      uint32_t led_interval = 0U;
	      uint8_t flush_allowed = 0U;

	      if (PD_BM_NeedsService() != 0U)
	      {
	        PD_BM_Task();
	      }

	      pd_state = PD_BM_GetState();

	      /*
	       * Track how long we have been continuously DETACHED. Reset on any
	       * other state so the idle counter is fresh after every reattach.
	       */
	      if (pd_state == PD_BM_STATE_DETACHED)
	      {
	        if (pd_prev_state != PD_BM_STATE_DETACHED)
	        {
	          pd_detached_since_tick = HAL_GetTick();
	          if (pd_detached_since_tick == 0U)
	          {
	            pd_detached_since_tick = 1U;
	          }
	        }
	      }
	      else
	      {
	        pd_detached_since_tick = 0U;
	      }
	      pd_prev_state = pd_state;

	      /*
	       * Trace flush gating. printf at 115200 blocks for several ms per record,
	       * which is well over the PD GoodCRC / SenderResponse / chunk timing
	       * window. Keep traces buffered while a live contract is running:
	       *   - READY:     SPR-only contract — EPR rejected / disabled
	       *   - ERROR:     terminal failure
	       *   - DETACHED:  but only after a debounce window, so the rapid
	       *                hard-reset -> detach -> reattach cycle does not
	       *                trigger printf in the middle of negotiation.
	       *
	       * Note: ATTACHED was previously in this whitelist. It is now
	       * removed because SRC_CAP can arrive any moment after attach and
	       * we cannot afford the ~5 ms blocking.
	       * READY/EPR_READY are also excluded now; EPR_KeepAlive_Ack can
	       * arrive at any time and still needs a fast GoodCRC.
	       */
	      if (PD_BM_NeedsService() == 0U)
	      {
	        switch (pd_state)
	        {
	          case PD_BM_STATE_ERROR:
	            flush_allowed = 1U;
	            break;
	          case PD_BM_STATE_DETACHED:
	            if ((pd_detached_since_tick != 0U)
	                && ((HAL_GetTick() - pd_detached_since_tick)
	                    >= PD_APP_DETACH_FLUSH_IDLE_MS))
	            {
	              flush_allowed = 1U;
	            }
	            break;
	          default:
	            break;
	        }
	      }

	      if (flush_allowed != 0U)
	      {
	        PD_App_TraceFlush();
	      }

	      switch (pd_state)
	      {
	        case PD_BM_STATE_DETACHED:
	          BSP_LED_Off(LED_GREEN);
	          pd_reported_profile = 0xFFU;
	          pd_reported_voltage_mv = 0U;
	          pd_reported_current_ma = 0U;
	          break;

	        case PD_BM_STATE_ATTACHED:
	          BSP_LED_On(LED_GREEN);
	          break;

	        case PD_BM_STATE_RX_ACTIVITY:
	          led_interval = 500U;
	          break;

	        case PD_BM_STATE_SRC_CAP_RX:
	        case PD_BM_STATE_REQUEST_SENT:
	          led_interval = 120U;
	          break;

	        case PD_BM_STATE_ACCEPT_RX:
	          led_interval = 300U;
	          break;

	        case PD_BM_STATE_READY:
	        case PD_BM_STATE_EPR_READY:
	        case PD_BM_STATE_PS_RDY_RX:
	          led_interval = 1000U;
	          break;

	        case PD_BM_STATE_ERROR:
	        default:
	          led_interval = 50U;
	          break;
	      }

	      if ((pd_state == PD_BM_STATE_READY) || (pd_state == PD_BM_STATE_EPR_READY))
	      {
	        uint8_t active_profile = PD_BM_GetActiveProfile();
	        uint16_t voltage_mv = PD_BM_GetRequestedVoltage();
	        uint16_t current_ma = PD_BM_GetRequestedCurrent();

	        if ((active_profile != 0xFFU) && (voltage_mv != 0U) && (current_ma != 0U)
	            && ((active_profile != pd_reported_profile)
	            || (voltage_mv != pd_reported_voltage_mv)
	            || (current_ma != pd_reported_current_ma)))
	        {
	          pd_reported_profile = active_profile;
	          pd_reported_voltage_mv = voltage_mv;
	          pd_reported_current_ma = current_ma;
	          printf("USB-PD ready: profile=%u, voltage=%u mV, current=%u mA\r\n",
	              active_profile, voltage_mv, current_ma);
	        }
	      }

	      if ((led_interval != 0U) && ((HAL_GetTick() - led_last_tick) >= led_interval))
	      {
	        led_last_tick = HAL_GetTick();
	        BSP_LED_Toggle(LED_GREEN);
	      }
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

  /** Configure the main internal regulator output voltage
  */
  if (HAL_PWREx_ControlVoltageScaling(PWR_REGULATOR_VOLTAGE_SCALE4) != HAL_OK)
  {
    Error_Handler();
  }

  /** Initializes the CPU, AHB and APB buses clocks
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI|RCC_OSCILLATORTYPE_MSI;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.HSICalibrationValue = RCC_HSICALIBRATION_DEFAULT;
  RCC_OscInitStruct.MSIState = RCC_MSI_ON;
  RCC_OscInitStruct.MSICalibrationValue = RCC_MSICALIBRATION_DEFAULT;
  RCC_OscInitStruct.MSIClockRange = RCC_MSIRANGE_4;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_NONE;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }

  /** Initializes the CPU, AHB and APB buses clocks
  */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2
                              |RCC_CLOCKTYPE_PCLK3;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_MSI;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV1;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;
  RCC_ClkInitStruct.APB3CLKDivider = RCC_HCLK_DIV1;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_0) != HAL_OK)
  {
    Error_Handler();
  }
}

/**
  * @brief Power Configuration
  * @retval None
  */
static void SystemPower_Config(void)
{

  /*
   * Switch to SMPS regulator instead of LDO
   */
  if (HAL_PWREx_ConfigSupply(PWR_SMPS_SUPPLY) != HAL_OK)
  {
    Error_Handler();
  }
/* USER CODE BEGIN PWR */
/* USER CODE END PWR */
}

/**
  * @brief GPDMA1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_GPDMA1_Init(void)
{

  /* USER CODE BEGIN GPDMA1_Init 0 */

  /* USER CODE END GPDMA1_Init 0 */

  /* Peripheral clock enable */
  __HAL_RCC_GPDMA1_CLK_ENABLE();

  /* GPDMA1 interrupt Init */
    HAL_NVIC_SetPriority(GPDMA1_Channel0_IRQn, 0, 0);
    HAL_NVIC_EnableIRQ(GPDMA1_Channel0_IRQn);
    HAL_NVIC_SetPriority(GPDMA1_Channel1_IRQn, 0, 0);
    HAL_NVIC_EnableIRQ(GPDMA1_Channel1_IRQn);

  /* USER CODE BEGIN GPDMA1_Init 1 */

  /* USER CODE END GPDMA1_Init 1 */
  /* USER CODE BEGIN GPDMA1_Init 2 */

  /* USER CODE END GPDMA1_Init 2 */

}

/**
  * @brief ICACHE Initialization Function
  * @param None
  * @retval None
  */
static void MX_ICACHE_Init(void)
{

  /* USER CODE BEGIN ICACHE_Init 0 */

  /* USER CODE END ICACHE_Init 0 */

  /* USER CODE BEGIN ICACHE_Init 1 */

  /* USER CODE END ICACHE_Init 1 */

  /** Enable instruction cache in 1-way (direct mapped cache)
  */
  if (HAL_ICACHE_ConfigAssociativityMode(ICACHE_1WAY) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_ICACHE_Enable() != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN ICACHE_Init 2 */

  /* USER CODE END ICACHE_Init 2 */

}

/**
  * @brief UCPD1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_UCPD1_Init(void)
{

  /* USER CODE BEGIN UCPD1_Init 0 */

  /* USER CODE END UCPD1_Init 0 */

  LL_GPIO_InitTypeDef GPIO_InitStruct = {0};
  LL_DMA_InitTypeDef DMA_InitStruct = {0};

  /* Peripheral clock enable */
  LL_APB1_GRP2_EnableClock(LL_APB1_GRP2_PERIPH_UCPD1);

  LL_AHB2_GRP1_EnableClock(LL_AHB2_GRP1_PERIPH_GPIOB);
  LL_AHB2_GRP1_EnableClock(LL_AHB2_GRP1_PERIPH_GPIOA);
  /**UCPD1 GPIO Configuration
  PB15   ------> UCPD1_CC2
  PA15 (JTDI)   ------> UCPD1_CC1
  */
  GPIO_InitStruct.Pin = LL_GPIO_PIN_15;
  GPIO_InitStruct.Mode = LL_GPIO_MODE_ANALOG;
  GPIO_InitStruct.Pull = LL_GPIO_PULL_NO;
  LL_GPIO_Init(GPIOB, &GPIO_InitStruct);

  GPIO_InitStruct.Pin = LL_GPIO_PIN_15;
  GPIO_InitStruct.Mode = LL_GPIO_MODE_ANALOG;
  GPIO_InitStruct.Pull = LL_GPIO_PULL_NO;
  LL_GPIO_Init(GPIOA, &GPIO_InitStruct);

  /* UCPD1 DMA Init */

  /* GPDMA1_REQUEST_UCPD1_RX Init */
  DMA_InitStruct.SrcAddress = 0x00000000U;
  DMA_InitStruct.DestAddress = 0x00000000U;
  DMA_InitStruct.Direction = LL_DMA_DIRECTION_PERIPH_TO_MEMORY;
  DMA_InitStruct.BlkHWRequest = LL_DMA_HWREQUEST_SINGLEBURST;
  DMA_InitStruct.DataAlignment = LL_DMA_DATA_ALIGN_ZEROPADD;
  DMA_InitStruct.SrcBurstLength = 1;
  DMA_InitStruct.DestBurstLength = 1;
  DMA_InitStruct.SrcDataWidth = LL_DMA_SRC_DATAWIDTH_BYTE;
  DMA_InitStruct.DestDataWidth = LL_DMA_DEST_DATAWIDTH_BYTE;
  DMA_InitStruct.SrcIncMode = LL_DMA_SRC_FIXED;
  DMA_InitStruct.DestIncMode = LL_DMA_DEST_INCREMENT;
  DMA_InitStruct.Priority = LL_DMA_LOW_PRIORITY_LOW_WEIGHT;
  DMA_InitStruct.BlkDataLength = 0x00000000U;
  DMA_InitStruct.TriggerMode = LL_DMA_TRIGM_BLK_TRANSFER;
  DMA_InitStruct.TriggerPolarity = LL_DMA_TRIG_POLARITY_MASKED;
  DMA_InitStruct.TriggerSelection = 0x00000000U;
  DMA_InitStruct.Request = LL_GPDMA1_REQUEST_UCPD1_RX;
  DMA_InitStruct.TransferEventMode = LL_DMA_TCEM_BLK_TRANSFER;
  DMA_InitStruct.SrcAllocatedPort = LL_DMA_SRC_ALLOCATED_PORT0;
  DMA_InitStruct.DestAllocatedPort = LL_DMA_DEST_ALLOCATED_PORT0;
  DMA_InitStruct.LinkAllocatedPort = LL_DMA_LINK_ALLOCATED_PORT1;
  DMA_InitStruct.LinkStepMode = LL_DMA_LSM_FULL_EXECUTION;
  DMA_InitStruct.LinkedListBaseAddr = 0x00000000U;
  DMA_InitStruct.LinkedListAddrOffset = 0x00000000U;
  LL_DMA_Init(GPDMA1, LL_DMA_CHANNEL_1, &DMA_InitStruct);

  /* GPDMA1_REQUEST_UCPD1_TX Init */
  DMA_InitStruct.SrcAddress = 0x00000000U;
  DMA_InitStruct.DestAddress = 0x00000000U;
  DMA_InitStruct.Direction = LL_DMA_DIRECTION_PERIPH_TO_MEMORY;
  DMA_InitStruct.BlkHWRequest = LL_DMA_HWREQUEST_SINGLEBURST;
  DMA_InitStruct.DataAlignment = LL_DMA_DATA_ALIGN_ZEROPADD;
  DMA_InitStruct.SrcBurstLength = 1;
  DMA_InitStruct.DestBurstLength = 1;
  DMA_InitStruct.SrcDataWidth = LL_DMA_SRC_DATAWIDTH_BYTE;
  DMA_InitStruct.DestDataWidth = LL_DMA_DEST_DATAWIDTH_BYTE;
  DMA_InitStruct.SrcIncMode = LL_DMA_SRC_FIXED;
  DMA_InitStruct.DestIncMode = LL_DMA_DEST_INCREMENT;
  DMA_InitStruct.Priority = LL_DMA_LOW_PRIORITY_LOW_WEIGHT;
  DMA_InitStruct.BlkDataLength = 0x00000000U;
  DMA_InitStruct.TriggerMode = LL_DMA_TRIGM_BLK_TRANSFER;
  DMA_InitStruct.TriggerPolarity = LL_DMA_TRIG_POLARITY_MASKED;
  DMA_InitStruct.TriggerSelection = 0x00000000U;
  DMA_InitStruct.Request = LL_GPDMA1_REQUEST_UCPD1_TX;
  DMA_InitStruct.TransferEventMode = LL_DMA_TCEM_BLK_TRANSFER;
  DMA_InitStruct.SrcAllocatedPort = LL_DMA_SRC_ALLOCATED_PORT0;
  DMA_InitStruct.DestAllocatedPort = LL_DMA_DEST_ALLOCATED_PORT0;
  DMA_InitStruct.LinkAllocatedPort = LL_DMA_LINK_ALLOCATED_PORT1;
  DMA_InitStruct.LinkStepMode = LL_DMA_LSM_FULL_EXECUTION;
  DMA_InitStruct.LinkedListBaseAddr = 0x00000000U;
  DMA_InitStruct.LinkedListAddrOffset = 0x00000000U;
  LL_DMA_Init(GPDMA1, LL_DMA_CHANNEL_0, &DMA_InitStruct);

  /* UCPD1 interrupt Init */
  NVIC_SetPriority(UCPD1_IRQn, NVIC_EncodePriority(NVIC_GetPriorityGrouping(),0, 0));
  NVIC_EnableIRQ(UCPD1_IRQn);

  /* USER CODE BEGIN UCPD1_Init 1 */

  /* USER CODE END UCPD1_Init 1 */
  /* USER CODE BEGIN UCPD1_Init 2 */

  /* USER CODE END UCPD1_Init 2 */

}

/**
  * @brief GPIO Initialization Function
  * @param None
  * @retval None
  */
static void MX_GPIO_Init(void)
{
  /* USER CODE BEGIN MX_GPIO_Init_1 */

  /* USER CODE END MX_GPIO_Init_1 */

  /* GPIO Ports Clock Enable */
  __HAL_RCC_GPIOC_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();

  /* USER CODE BEGIN MX_GPIO_Init_2 */

  /* USER CODE END MX_GPIO_Init_2 */
}

/* USER CODE BEGIN 4 */
static void PD_App_TIM16_Init(void)
{
  RCC_ClkInitTypeDef clock_config;
  uint32_t flash_latency;
  uint32_t tim_clk_hz;
  uint32_t prescaler;
  uint32_t period;

  HAL_RCC_GetClockConfig(&clock_config, &flash_latency);
  (void)flash_latency;

  tim_clk_hz = HAL_RCC_GetPCLK2Freq();
  if (clock_config.APB2CLKDivider != RCC_HCLK_DIV1)
  {
    tim_clk_hz *= 2U;
  }

  prescaler = tim_clk_hz / PD_APP_TIMER_BASE_HZ;
  if (prescaler == 0U)
  {
    prescaler = 1U;
  }

  period = (tim_clk_hz / prescaler) / PD_APP_TIMER_HZ;
  if (period == 0U)
  {
    period = 1U;
  }

  __HAL_RCC_TIM16_CLK_ENABLE();

  htim16.Instance = TIM16;
  htim16.Init.Prescaler = prescaler - 1U;
  htim16.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim16.Init.Period = period - 1U;
  htim16.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim16.Init.RepetitionCounter = 0U;
  htim16.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;

  if (HAL_TIM_Base_Init(&htim16) != HAL_OK)
  {
    Error_Handler();
  }

  HAL_NVIC_SetPriority(TIM16_IRQn, 5, 0);
  HAL_NVIC_EnableIRQ(TIM16_IRQn);
}

static uint32_t PD_App_GetTick(void *user)
{
  (void)user;
  return HAL_GetTick();
}

static void PD_App_RxDmaStart(uint8_t *buffer, uint16_t size, void *user)
{
  (void)user;

  pd_app_rx_dma_size = size;

  LL_DMA_DisableChannel(GPDMA1, LL_DMA_CHANNEL_1);

  LL_DMA_SetSrcAddress(GPDMA1, LL_DMA_CHANNEL_1, (uint32_t)&UCPD1->RXDR);
  LL_DMA_SetDestAddress(GPDMA1, LL_DMA_CHANNEL_1, (uint32_t)buffer);
  LL_DMA_SetBlkDataLength(GPDMA1, LL_DMA_CHANNEL_1, size);

  LL_DMA_EnableChannel(GPDMA1, LL_DMA_CHANNEL_1);
}

static void PD_App_RxDmaStop(void *user)
{
  (void)user;
  LL_DMA_DisableChannel(GPDMA1, LL_DMA_CHANNEL_1);
}

static uint16_t PD_App_RxDmaCount(void *user)
{
  (void)user;

  return (uint16_t)(pd_app_rx_dma_size -
                    LL_DMA_GetBlkDataLength(GPDMA1, LL_DMA_CHANNEL_1));
}

static void PD_App_Trace(const char *event, uint32_t a, uint32_t b, uint32_t c, uint32_t d, void *user)
{
  (void)user;
  uint16_t head = pd_app_trace_head;
  uint16_t next = (uint16_t)((head + 1U) % PD_APP_TRACE_QUEUE_LEN);

  if (next == pd_app_trace_tail)
  {
    pd_app_trace_lost++;
    return;
  }

  pd_app_trace_queue[head].event = event;
  pd_app_trace_queue[head].tick = HAL_GetTick();
  pd_app_trace_queue[head].a = a;
  pd_app_trace_queue[head].b = b;
  pd_app_trace_queue[head].c = c;
  pd_app_trace_queue[head].d = d;
  pd_app_trace_head = next;
}

static void PD_App_TraceFlush(void)
{
  /*
   * Print AT MOST ONE record per call. This keeps each main-loop iteration
   * short so PD_BM_NeedsService() can grab the CPU between printf bursts and
   * service incoming RX / send GoodCRC within the PD timing window.
   */
  if (pd_app_trace_tail != pd_app_trace_head)
  {
    PD_AppTraceRecord record = pd_app_trace_queue[pd_app_trace_tail];
    pd_app_trace_tail = (uint16_t)((pd_app_trace_tail + 1U) % PD_APP_TRACE_QUEUE_LEN);

    printf("[PD] %lu %s a=%lu b=0x%08lX c=%lu d=%lu\r\n",
        (unsigned long)record.tick,
        record.event,
        (unsigned long)record.a,
        (unsigned long)record.b,
        (unsigned long)record.c,
        (unsigned long)record.d);
    return;
  }

  if (pd_app_trace_lost != 0U)
  {
    uint16_t lost = pd_app_trace_lost;
    pd_app_trace_lost = 0U;
    printf("[PD] trace_lost count=%u\r\n", lost);
  }
}
/* USER CODE END 4 */

/**
  * @brief  Period elapsed callback in non blocking mode
  * @note   This function is called  when TIM17 interrupt took place, inside
  * HAL_TIM_IRQHandler(). It makes a direct call to HAL_IncTick() to increment
  * a global variable "uwTick" used as application time base.
  * @param  htim : TIM handle
  * @retval None
  */
void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim)
{
  /* USER CODE BEGIN Callback 0 */

  /* USER CODE END Callback 0 */
  if (htim->Instance == TIM17)
  {
    HAL_IncTick();
  }
  /* USER CODE BEGIN Callback 1 */
  if (htim->Instance == TIM16)
  {
    PD_BM_TimerTickISR();
  }

  /* USER CODE END Callback 1 */
}

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
