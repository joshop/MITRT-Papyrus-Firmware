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

/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */
#define CAN_CMD_ID 501
/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */

/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/

FDCAN_HandleTypeDef hfdcan1;

/* USER CODE BEGIN PV */

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_FDCAN1_Init(void);
/* USER CODE BEGIN PFP */

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */
void CAN_SendAck(uint8_t *data, uint8_t len)
{
  FDCAN_TxHeaderTypeDef txHeader = {0};

  txHeader.Identifier = CAN_CMD_ID;
  txHeader.IdType = FDCAN_STANDARD_ID;
  txHeader.TxFrameType = FDCAN_DATA_FRAME;
  txHeader.DataLength = len;
  txHeader.ErrorStateIndicator = FDCAN_ESI_ACTIVE;
  txHeader.BitRateSwitch = FDCAN_BRS_OFF;
  txHeader.FDFormat = FDCAN_CLASSIC_CAN;
  txHeader.TxEventFifoControl = FDCAN_NO_TX_EVENTS;
  txHeader.MessageMarker = 0;

  while (HAL_FDCAN_GetTxFifoFreeLevel(&hfdcan1) == 0);
  HAL_FDCAN_AddMessageToTxFifoQ(&hfdcan1, &txHeader, data);
}

/**
 * @brief Retrieves the data from the specified HX711 channel and IC via bitbanging.
 * Data should only be retrieved if `DOUT` on the desired IC is set to `LOW`.
 * @param channel (uint8_t) set to 0 (channel A) or 1 (channel B)
 * @param cs (uint8_t) which load cell to read from (0, 1, 2)
 * @param gain (uint8_t) if channel 0, set to 128 (0) or 64 (1)
 * @returns (uint32_t) Data in the lower 6 bytes, in 2's complement format.
 * if `0xFFFFFFFF`, an error has occured
 * if `0x800000`, minimum. if `0x7FFFFF`, maximum.
 */
uint32_t HX711_GetData(uint8_t channel, uint8_t cs, uint8_t gain) {
  // error status codes
  uint32_t ERROR_NOT_READY = 0xFF000001;
  uint32_t ERROR_INVALID_ARG = 0xFF000002;
  uint32_t ERROR_BUFFER = 0xFF000003;

  uint32_t pulses;
  uint32_t buffer;

  switch (channel) {
    case 0:
      if (gain) {
        pulses = 27;
      } else {
        pulses = 25;
      }
      break;
    case 1:
      pulses = 26;
    default:
      return ERROR_INVALID_ARG;
  }

  uint16_t GPIO_PIN_DOUT;
  uint16_t GPIO_PIN_SCK;
  switch (cs) {
    case 0:
      GPIO_PIN_DOUT = GPIO_PIN_6;
      GPIO_PIN_SCK = GPIO_PIN_1;
    case 1:
      GPIO_PIN_DOUT = GPIO_PIN_5;
      GPIO_PIN_SCK = GPIO_PIN_2;
    case 2:
      GPIO_PIN_DOUT = GPIO_PIN_4;
      GPIO_PIN_SCK = GPIO_PIN_3;
    default:
      return ERROR_INVALID_ARG;
  }

  // check that this is set to LOW
  if (HAL_GPIO_ReadPin(GPIOA, GPIO_PIN_DOUT)) {
    return ERROR_NOT_READY;
  }

  // pulse SCK and put data onto the buffer
  // remember to set SCK to LOW (if it somehow isn't)
  HAL_GPIO_WritePin(GPIOA, GPIO_PIN_SCK, GPIO_PIN_RESET);
  pulses <<= 1;
  uint8_t clock_status = 0;
  uint32_t idx = 0;
  while (pulses > 0) {
    HAL_GPIO_TogglePin(GPIOA, GPIO_PIN_SCK);
    clock_status ^= 1;
    if (clock_status) {
      uint32_t data = HAL_GPIO_ReadPin(GPIOA, GPIO_PIN_DOUT);
      buffer |= (data << idx);
      idx++;
    }
    pulses--;
  }
  HAL_GPIO_WritePin(GPIOA, GPIO_PIN_SCK, GPIO_PIN_RESET);
  // should be set to HIGH when done
  if (HAL_GPIO_ReadPin(GPIOA, GPIO_PIN_DOUT)) {
    return buffer;
  }
  return ERROR_BUFFER;
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
  MX_FDCAN1_Init();
  /* USER CODE BEGIN 2 */
  HAL_FDCAN_Start(&hfdcan1);

  FDCAN_RxHeaderTypeDef rxHeader;
  uint8_t rxData[8];
  uint8_t txData[8];

  uint32_t lastTick = 0;
  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
    // simple blink
    if (HAL_GetTick() - lastTick > 1000) {
      HAL_GPIO_TogglePin(GPIOA, GPIO_PIN_0);
      lastTick = HAL_GetTick();
    }
    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */

    // TODO: change boilerplate code from pt
    if (HAL_FDCAN_GetRxFifoFillLevel(&hfdcan1, FDCAN_RX_FIFO0) > 0)
    {
      HAL_FDCAN_GetRxMessage(&hfdcan1, FDCAN_RX_FIFO0, &rxHeader, rxData);

      txData[0] = rxData[0];
      txData[1] = 0x00; // OK

      switch (rxData[0])
      {
        case 0x00:
          txData[1] = 0x04; break;
        case 0x01:
          // *(uint16_t*)(txData+1) = (ch2_mV*2500)/1500;
          CAN_SendAck(txData, 3);
          continue;
        case 0x02:
          // *(uint16_t*)(txData+1) = (ch3_mV*2500)/1500;
          CAN_SendAck(txData, 3);
          continue;
        case 0x03:
          // *(uint16_t*)(txData+1) = (ch4_mV*2500)/1500;
          CAN_SendAck(txData, 3);
          continue;

        case 0xFF:
          CAN_SendAck(txData, 2);
          HAL_Delay(10);
          NVIC_SystemReset();
          break;


        default:
          continue;
      }

      CAN_SendAck(txData, 2);
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

  __HAL_FLASH_SET_LATENCY(FLASH_LATENCY_0);

  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.HSIDiv = RCC_HSI_DIV4;
  RCC_OscInitStruct.HSICalibrationValue = RCC_HSICALIBRATION_DEFAULT;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }

  /** Initializes the CPU, AHB and APB buses clocks
  */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_HSI;
  RCC_ClkInitStruct.SYSCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_HCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_APB1_DIV1;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_0) != HAL_OK)
  {
    Error_Handler();
  }
}

/**
  * @brief FDCAN1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_FDCAN1_Init(void)
{

  /* USER CODE BEGIN FDCAN1_Init 0 */

  /* USER CODE END FDCAN1_Init 0 */

  /* USER CODE BEGIN FDCAN1_Init 1 */

  /* USER CODE END FDCAN1_Init 1 */
  hfdcan1.Instance = FDCAN1;
  hfdcan1.Init.ClockDivider = FDCAN_CLOCK_DIV1;
  hfdcan1.Init.FrameFormat = FDCAN_FRAME_CLASSIC;
  hfdcan1.Init.Mode = FDCAN_MODE_NORMAL;
  hfdcan1.Init.AutoRetransmission = ENABLE;
  hfdcan1.Init.TransmitPause = DISABLE;
  hfdcan1.Init.ProtocolException = DISABLE;
  hfdcan1.Init.NominalPrescaler = 4;
  hfdcan1.Init.NominalSyncJumpWidth = 1;
  hfdcan1.Init.NominalTimeSeg1 = 13;
  hfdcan1.Init.NominalTimeSeg2 = 2;
  hfdcan1.Init.DataPrescaler = 1;
  hfdcan1.Init.DataSyncJumpWidth = 1;
  hfdcan1.Init.DataTimeSeg1 = 1;
  hfdcan1.Init.DataTimeSeg2 = 1;
  hfdcan1.Init.StdFiltersNbr = 1;
  hfdcan1.Init.ExtFiltersNbr = 0;
  hfdcan1.Init.TxFifoQueueMode = FDCAN_TX_FIFO_OPERATION;
  if (HAL_FDCAN_Init(&hfdcan1) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN FDCAN1_Init 2 */
  FDCAN_FilterTypeDef filter = {0};
  filter.IdType = FDCAN_STANDARD_ID;
  filter.FilterIndex = 0;
  filter.FilterType = FDCAN_FILTER_DUAL;
  filter.FilterConfig = FDCAN_FILTER_TO_RXFIFO0;
  filter.FilterID1 = CAN_CMD_ID;
  filter.FilterID2 = 0;
  HAL_FDCAN_ConfigFilter(&hfdcan1, &filter);
  HAL_FDCAN_ConfigGlobalFilter(
    &hfdcan1,
    FDCAN_REJECT,  // Non-matching standard frames
    FDCAN_REJECT,  // Non-matching extended frames
    FDCAN_REJECT,  // Reject remote standard frames
    FDCAN_REJECT   // Reject remote extended frames
  );
  /* USER CODE END FDCAN1_Init 2 */

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
  HAL_GPIO_WritePin(GPIOA, GPIO_PIN_0|GPIO_PIN_1|GPIO_PIN_2|GPIO_PIN_3
                          |GPIO_PIN_7, GPIO_PIN_RESET);

  /*Configure GPIO pins : PA0 PA1 PA2 PA3
                           PA7 */
  GPIO_InitStruct.Pin = GPIO_PIN_0|GPIO_PIN_1|GPIO_PIN_2|GPIO_PIN_3
                          |GPIO_PIN_7;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

  /*Configure GPIO pins : PA4 PA5 PA6 */
  GPIO_InitStruct.Pin = GPIO_PIN_4|GPIO_PIN_5|GPIO_PIN_6;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

  /* USER CODE BEGIN MX_GPIO_Init_2 */

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
