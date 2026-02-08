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

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
#define CAN_CMD_ID     300

/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/

FDCAN_HandleTypeDef hfdcan1;

SPI_HandleTypeDef hspi1;

/* USER CODE BEGIN PV */

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_FDCAN1_Init(void);
static void MX_SPI1_Init(void);
/* USER CODE BEGIN PFP */

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */
typedef struct {
  int16_t thermocouple_c;
  int16_t internal_c;
  uint8_t fault;
  uint8_t fault_scg;
  uint8_t fault_scv;
  uint8_t fault_oc;
} MAX31855_Data;

HAL_StatusTypeDef MAX31855_Read(SPI_HandleTypeDef *hspi,
                                GPIO_TypeDef *cs_port,
                                uint16_t cs_pin,
                                MAX31855_Data *out) {
  uint8_t rx[4];
  uint32_t raw;

  /* CS low */
  HAL_GPIO_WritePin(cs_port, cs_pin, GPIO_PIN_RESET);

  /* Read 32 bits */
  HAL_StatusTypeDef status =
  HAL_SPI_Receive(hspi, rx, 4, HAL_MAX_DELAY);

  /* CS high */
  HAL_GPIO_WritePin(cs_port, cs_pin, GPIO_PIN_SET);

  if (status != HAL_OK) {
    return status;
  }

  /* Assemble big-endian data */
  raw = ((uint32_t)rx[0] << 24) |
  ((uint32_t)rx[1] << 16) |
  ((uint32_t)rx[2] << 8)  |
  ((uint32_t)rx[3]);

  /* Fault flag */
  out->fault = (raw >> 16) & 0x01;

  /* Thermocouple temperature (14-bit signed, 0.25°C/LSB) */
  int16_t tc_raw = (raw >> 18) & 0x3FFF;
  if (tc_raw & 0x2000) {          // sign bit
    tc_raw |= 0xC000;           // sign extend
  }
  out->thermocouple_c = tc_raw;

  /* Internal temperature (12-bit signed, 0.0625°C/LSB) */
  int16_t int_raw = (raw >> 4) & 0x0FFF;
  if (int_raw & 0x0800) {
    int_raw |= 0xF000;
  }
  out->internal_c = int_raw;

  /* Fault details */
  out->fault_scg = raw & 0x01;        // short to GND
  out->fault_scv = (raw >> 1) & 0x01; // short to VCC
  out->fault_oc  = (raw >> 2) & 0x01; // open circuit

  return HAL_OK;
}

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
  MX_SPI1_Init();
  /* USER CODE BEGIN 2 */
  HAL_FDCAN_Start(&hfdcan1);

  FDCAN_RxHeaderTypeDef rxHeader;
  uint8_t rxData[8];
  uint8_t txData[8];
  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */
    if (HAL_FDCAN_GetRxFifoFillLevel(&hfdcan1, FDCAN_RX_FIFO0) > 0)
    {
      HAL_FDCAN_GetRxMessage(&hfdcan1, FDCAN_RX_FIFO0, &rxHeader, rxData);

      txData[0] = rxData[0];
      txData[1] = 0x00; // OK
      MAX31855_Data thermo;

      switch (rxData[0])
      {
        case 0x00:
          txData[1] = 0x03; break;
        case 0x01:
        case 0x02:
        case 0x03:
          if (rxData[0] == 0x01) MAX31855_Read(&hspi1, GPIOB, GPIO_PIN_7, &thermo);
          if (rxData[0] == 0x02) MAX31855_Read(&hspi1, GPIOC, GPIO_PIN_14, &thermo);
          if (rxData[0] == 0x03) MAX31855_Read(&hspi1, GPIOC, GPIO_PIN_15, &thermo);
          if (thermo.fault) {
            txData[1] = 0xFF;
            txData[2] = 0xFF;
          } else {
            *(int16_t*)(txData+1) = thermo.thermocouple_c;
          }
          CAN_SendAck(txData, 3);
          continue;
        case 0x04:
        case 0x05:
        case 0x06:
          if (rxData[0] == 0x04) MAX31855_Read(&hspi1, GPIOB, GPIO_PIN_7, &thermo);
          if (rxData[0] == 0x05) MAX31855_Read(&hspi1, GPIOC, GPIO_PIN_14, &thermo);
          if (rxData[0] == 0x06) MAX31855_Read(&hspi1, GPIOC, GPIO_PIN_15, &thermo);
          txData[1] = 0;
          if (thermo.fault_oc) txData[1] |= 1;
          if (thermo.fault_scg) txData[1] |= 2;
          if (thermo.fault_scv) txData[1] |= 4;
          if (thermo.fault) txData[1] |= 8;
          *(int16_t*)(txData+2) = thermo.internal_c;
          CAN_SendAck(txData, 4);
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
  hspi1.Init.CLKPolarity = SPI_POLARITY_LOW;
  hspi1.Init.CLKPhase = SPI_PHASE_1EDGE;
  hspi1.Init.NSS = SPI_NSS_SOFT;
  hspi1.Init.BaudRatePrescaler = SPI_BAUDRATEPRESCALER_2;
  hspi1.Init.FirstBit = SPI_FIRSTBIT_MSB;
  hspi1.Init.TIMode = SPI_TIMODE_DISABLE;
  hspi1.Init.CRCCalculation = SPI_CRCCALCULATION_DISABLE;
  hspi1.Init.CRCPolynomial = 7;
  hspi1.Init.CRCLength = SPI_CRC_LENGTH_DATASIZE;
  hspi1.Init.NSSPMode = SPI_NSS_PULSE_ENABLE;
  if (HAL_SPI_Init(&hspi1) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN SPI1_Init 2 */

  /* USER CODE END SPI1_Init 2 */

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
  __HAL_RCC_GPIOB_CLK_ENABLE();
  __HAL_RCC_GPIOC_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOB, GPIO_PIN_7, GPIO_PIN_SET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOC, GPIO_PIN_14|GPIO_PIN_15, GPIO_PIN_SET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOA, GPIO_PIN_0|GPIO_PIN_1, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOA, GPIO_PIN_4, GPIO_PIN_SET);

  /*Configure GPIO pin : PB7 */
  GPIO_InitStruct.Pin = GPIO_PIN_7;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

  /*Configure GPIO pins : PC14 PC15 */
  GPIO_InitStruct.Pin = GPIO_PIN_14|GPIO_PIN_15;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOC, &GPIO_InitStruct);

  /*Configure GPIO pins : PA0 PA1 PA4 */
  GPIO_InitStruct.Pin = GPIO_PIN_0|GPIO_PIN_1|GPIO_PIN_4;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
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
