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
#include "app_fatfs.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include <string.h>
#include <stdio.h>
#include <stdlib.h>
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */

#define CAN_CMD_ID     0x124

#define FLASH_USER_PAGE_ADDR  0x0801F800   // Example: last page of 64KB flash
#define FLASH_PAGE_SIZE       2048
/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
ADC_HandleTypeDef hadc1;
DMA_HandleTypeDef hdma_adc1;

FDCAN_HandleTypeDef hfdcan1;

I2C_HandleTypeDef hi2c1;

SPI_HandleTypeDef hspi1;
SPI_HandleTypeDef hspi2;

TIM_HandleTypeDef htim2;

UART_HandleTypeDef huart1;
UART_HandleTypeDef huart2;

/* USER CODE BEGIN PV */

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_DMA_Init(void);
static void MX_ADC1_Init(void);
static void MX_FDCAN1_Init(void);
static void MX_I2C1_Init(void);
static void MX_SPI1_Init(void);
static void MX_SPI2_Init(void);
static void MX_USART1_UART_Init(void);
static void MX_USART2_UART_Init(void);
static void MX_TIM2_Init(void);
/* USER CODE BEGIN PFP */
void CAN_Send(uint8_t cmd);
void UART_Print(char *s);
void UART2_Print(char *s);
void Process_UART_Command(char *cmd);
/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */
int next_can_id;
uint8_t lastRxData[8];

int __io_putchar(int ch)
{
  // Use the HAL function to transmit a character via your configured UART
  HAL_UART_Transmit(&huart2, (uint8_t *)&ch, 1, HAL_MAX_DELAY);
  return ch;
}

int __io_getchar(void)
{
  uint8_t ch = 0;
  // Wait for character reception
  HAL_UART_Receive(&huart2, (uint8_t *)&ch, 1, HAL_MAX_DELAY);
  return ch;
}
HAL_StatusTypeDef Flash_SaveArray(uint32_t pageAddress, uint8_t *data, uint32_t length)
{
  HAL_StatusTypeDef status;
  uint32_t pageError = 0;

  FLASH_EraseInitTypeDef eraseInit;

  if(length > FLASH_PAGE_SIZE)
    return HAL_ERROR;

  HAL_FLASH_Unlock();

  eraseInit.TypeErase = FLASH_TYPEERASE_PAGES;
  eraseInit.Page = (pageAddress - FLASH_BASE) / FLASH_PAGE_SIZE;
  eraseInit.NbPages = 1;

  status = HAL_FLASHEx_Erase(&eraseInit, &pageError);
  if(status != HAL_OK)
  {
    HAL_FLASH_Lock();
    return status;
  }

  uint32_t addr = pageAddress;
  uint64_t writeData;

  for(uint32_t i = 0; i < length; i += 8)
  {
    writeData = 0xFFFFFFFFFFFFFFFF;

    memcpy(&writeData, &data[i], (length - i >= 8) ? 8 : (length - i));

    status = HAL_FLASH_Program(FLASH_TYPEPROGRAM_DOUBLEWORD, addr, writeData);

    if(status != HAL_OK)
    {
      HAL_FLASH_Lock();
      return status;
    }

    addr += 8;
  }

  HAL_FLASH_Lock();
  return HAL_OK;
}

void UART2_Print_Int(int a);
void CAN_Send(uint8_t x);
void Flash_LoadArray(uint32_t pageAddress, uint8_t *buffer, uint32_t length)
{
  memcpy(buffer, (uint8_t*)pageAddress, length);
}
uint8_t emergency_solenoid[18];
int need_restore = 1;
void update_solenoid() {
  UART2_Print("Updating flash record: ");
  for (int i = 0; i < 18; i++) {
    UART2_Print_Int(emergency_solenoid[i]);
    UART2_Print(i == 17 ? "\r\n" : ",");
  }
  Flash_SaveArray(FLASH_USER_PAGE_ADDR, emergency_solenoid, 18);
}
void restore_solenoid() {
  Flash_LoadArray(FLASH_USER_PAGE_ADDR, emergency_solenoid, 18);
  if (!need_restore) return;
  for (int i = 0; i < 18; i++) {
    next_can_id = 105 + i / 3;
    UART2_Print("Restoring ");
    UART2_Print_Int(next_can_id);
    UART2_Print(":");
    UART2_Print_Int(i % 3 + 1);
    UART2_Print("=");
    int command = 1 + (i % 3) * 2 + (emergency_solenoid[i] ? 1 : 0);
    UART2_Print_Int(command);
    UART2_Print("\r\n");
    CAN_Send(command);
    HAL_Delay(250);
  }
}

uint16_t ADC_Read_VREFINT(void)
{
  ADC_ChannelConfTypeDef sConfig = {0};

  sConfig.Channel = ADC_CHANNEL_VREFINT;
  sConfig.Rank = ADC_REGULAR_RANK_1;
  sConfig.SamplingTime = ADC_SAMPLINGTIME_COMMON_1;

  HAL_ADC_ConfigChannel(&hadc1, &sConfig);

  HAL_ADC_Start(&hadc1);
  HAL_ADC_PollForConversion(&hadc1, HAL_MAX_DELAY);
  HAL_ADC_GetValue(&hadc1);

  HAL_ADC_Start(&hadc1);
  HAL_ADC_PollForConversion(&hadc1, HAL_MAX_DELAY);
  uint16_t raw = HAL_ADC_GetValue(&hadc1);
  HAL_ADC_Stop(&hadc1);

  return raw;
}
uint32_t ADC_Get_VDDA_mV(void)
{
  uint16_t vrefint_raw = ADC_Read_VREFINT();
  uint16_t vrefint_cal = *VREFINT_CAL_ADDR;

  // VDDA = 3.0V * VREFINT_CAL / VREFINT_DATA
  // 3000 mV is the calibration reference voltage
  uint32_t vdda = 3000UL * vrefint_cal / vrefint_raw;

  return vdda;
}
int terse = 0;
volatile uint16_t vref_raw = 1;
uint16_t vref_mV;
int just_set_solenoid = 0;
int just_set_id;
int just_set_val;
int just_set_idx;
void HAL_ADC_ConvCpltCallback(ADC_HandleTypeDef* hadc) {
  if (hadc->Instance == ADC1) {
    uint32_t vrefint_cal = *VREFINT_CAL_ADDR;
    vref_mV = (VREFINT_CAL_VREF * vrefint_cal) / vref_raw;
  }
}
void CAN_Rx_Default(FDCAN_RxHeaderTypeDef rxHeader) {
  if(just_set_solenoid) {
    emergency_solenoid[(just_set_id - 105) * 3 + just_set_idx - 1] = just_set_val;
    update_solenoid();
  }
  UART2_Print("OK\r\n");
  just_set_solenoid = 0;
}


void CAN_Rx_TC(FDCAN_RxHeaderTypeDef rxHeader) {
  char buff[64];
  int16_t tc_t = *(int16_t*)(lastRxData+1);
  float tc_tf = ((float)tc_t) / 4.0;
  if (lastRxData[1] == 0xFF && lastRxData[2] == 0xFF) {
    sprintf(buff, terse ? "\r\nErr" : "TC Fault\r\n");
  } else {
    sprintf(buff, terse ? "%.2f\r\n" : "T = %.2f C\r\n", tc_tf);
  }
  UART2_Print(buff);
}
typedef struct PTCfg {
  int initialized;
  uint16_t max_psi;
} PTCfg;
float pt_translate_mv(int id, int16_t pt_p);

void CAN_Rx_PT(FDCAN_RxHeaderTypeDef rxHeader) {
  char buff[64];
  int16_t pt_p = *(int16_t*)(lastRxData+1);
  float pt_v = pt_translate_mv(rxHeader.Identifier, pt_p);
  sprintf(buff, terse ? "%.2f\r\n" : "P = %.2f psi\r\n", pt_v);
  UART2_Print(buff);
}
float last_pt_v;

void CAN_Rx_PT_Quiet(FDCAN_RxHeaderTypeDef rxHeader) {
  int16_t pt_p = *(int16_t*)(lastRxData+1);
  // 0.57965/1.68179 * (PT2 + 359.67442) - 151.83194
  last_pt_v = pt_translate_mv(rxHeader.Identifier, pt_p);//3000.0f * ((float)pt_p - 500.0f) / 4000.0f;
}
void CAN_Rx_Relay(FDCAN_RxHeaderTypeDef rxHeader) {
  if (lastRxData[1]) {
    UART2_Print(terse ? "ON\rn" : "R = ON\r\n");
  } else {
    UART2_Print(terse ? "OFF\rn" : "R = OFF\r\n");
  }
}
void CAN_Rx_TCStat(FDCAN_RxHeaderTypeDef rxHeader) {
  char buff[64];
  uint8_t tc_f = lastRxData[1];
  int16_t tc_t = *(int16_t*)(lastRxData+2);
  float tc_tf = ((float)tc_t) / 16.0;
  sprintf(buff, terse ? "%.2f" : "CJC = %.2f C; ", tc_tf);
  UART2_Print(buff);
  if (tc_f == 0) {
    UART2_Print(terse ? "ok" : "no fault\r\n");
  } else {
    if (tc_f & 1) UART2_Print(terse ? "o" : "open ");
    if (tc_f & 2) UART2_Print(terse ? "g" : "GND-short ");
    if (tc_f & 4) UART2_Print(terse ? "v" : "VCC-short ");
    if (tc_f & 8) UART2_Print(terse ? "f" : "fault ");
    UART2_Print("\r\n");
  }
}

void (*CAN_Rx_Func)(FDCAN_RxHeaderTypeDef) = CAN_Rx_Default;

void CAN_Decode() {
  FDCAN_RxHeaderTypeDef rxHeader;
  uint32_t tick = HAL_GetTick();

  while (HAL_FDCAN_GetRxFifoFillLevel(&hfdcan1, FDCAN_RX_FIFO0) == 0 && HAL_GetTick() - tick < 500);

  if (HAL_FDCAN_GetRxFifoFillLevel(&hfdcan1, FDCAN_RX_FIFO0) == 0) {
    UART2_Print("(NO ACK)\r\n");
    just_set_solenoid = 0;
    return;
  }

  HAL_FDCAN_GetRxMessage(&hfdcan1, FDCAN_RX_FIFO0, &rxHeader, lastRxData);

  CAN_Rx_Func(rxHeader);
  CAN_Rx_Func = CAN_Rx_Default;

}

void CAN_Send_Long(uint8_t *buf, uint8_t len) {
  FDCAN_TxHeaderTypeDef txHeader = {0};
  FDCAN_RxHeaderTypeDef rxHeader;

  txHeader.Identifier = next_can_id;
  txHeader.IdType = FDCAN_STANDARD_ID;
  txHeader.TxFrameType = FDCAN_DATA_FRAME;
  txHeader.DataLength = len;
  txHeader.FDFormat = FDCAN_CLASSIC_CAN;
  txHeader.BitRateSwitch = FDCAN_BRS_OFF;

  HAL_FDCAN_AddMessageToTxFifoQ(&hfdcan1, &txHeader, buf);

  /* Wait for response */

  CAN_Decode();


}

typedef struct {
  GPIO_TypeDef* dout_port;
  uint16_t dout_pin;
  GPIO_TypeDef* sck_port;
  uint16_t sck_pin;
  float scale;
  int32_t offset;
} HX711_HandleTypeDef;


void HX711_Init(HX711_HandleTypeDef *hx);
int32_t HX711_ReadRaw(HX711_HandleTypeDef *hx);
float HX711_ReadWeight(HX711_HandleTypeDef *hx);
void HX711_Tare(HX711_HandleTypeDef *hx, uint16_t samples);
void HX711_SetParams(HX711_HandleTypeDef *hx, float scale, float offs);

static void HX711_Delay(void)
{
  for (volatile int i = 0; i < 50; i++); // small delay (~1us)
}

void HX711_Init(HX711_HandleTypeDef *hx)
{
  HAL_GPIO_WritePin(hx->sck_port, hx->sck_pin, GPIO_PIN_RESET);
  hx->scale = 1.0f;
  hx->offset = 0;
}

int32_t HX711_ReadRaw(HX711_HandleTypeDef *hx)
{
  int32_t data = 0;

  // Wait for DOUT to go low (data ready)
  while (HAL_GPIO_ReadPin(hx->dout_port, hx->dout_pin) == GPIO_PIN_SET);

  for (int i = 0; i < 24; i++)
  {
    HAL_GPIO_WritePin(hx->sck_port, hx->sck_pin, GPIO_PIN_SET);
    HX711_Delay();

    data = data << 1;
    HAL_GPIO_WritePin(hx->sck_port, hx->sck_pin, GPIO_PIN_RESET);
    HX711_Delay();

    if (HAL_GPIO_ReadPin(hx->dout_port, hx->dout_pin))
      data++;
  }

  // 25th pulse → Channel A, Gain 128
  HAL_GPIO_WritePin(hx->sck_port, hx->sck_pin, GPIO_PIN_SET);
  HX711_Delay();
  HAL_GPIO_WritePin(hx->sck_port, hx->sck_pin, GPIO_PIN_RESET);
  HX711_Delay();

  // Sign extend 24-bit value
  if (data & 0x800000)
    data |= 0xFF000000;
  char buf[64];
  return data;
}

float HX711_ReadWeight(HX711_HandleTypeDef *hx)
{
  int32_t raw = HX711_ReadRaw(hx);
  return (raw - hx->offset) / hx->scale;
}

void HX711_Tare(HX711_HandleTypeDef *hx, uint16_t samples)
{
  int64_t sum = 0;
  for (uint16_t i = 0; i < samples; i++)
  {
    sum += HX711_ReadRaw(hx);
  }
  hx->offset = sum / samples;
}

void HX711_SetParams(HX711_HandleTypeDef *hx, float scale, float offs)
{
  hx->offset = offs;
  hx->scale = scale;
}

HX711_HandleTypeDef hx = {
  .dout_port = GPIOD,
  .dout_pin = GPIO_PIN_1,
  .sck_port = GPIOD,
  .sck_pin = GPIO_PIN_0
};





int muted = 0;
void UART_Print(char *s)
{
  if (muted) return;
  //HAL_UART_Transmit(&hlpuart2, (uint8_t *)s, strlen(s), HAL_MAX_DELAY);
}
void UART2_Print_Int(int a) {
  char buf[8];
  if (muted) return;
  sprintf(buf, "%d", a);
  UART2_Print(buf);
}

void UART2_Print(char *s)
{
  if (muted) return;
  HAL_UART_Transmit(&huart2, (uint8_t *)s, strlen(s), HAL_MAX_DELAY);
}
void CAN_Send(uint8_t cmd)
{
  FDCAN_TxHeaderTypeDef txHeader = {0};
  FDCAN_RxHeaderTypeDef rxHeader;
  // UART2_Print("Sending ");
  // UART2_Print_Int(cmd);
  // UART2_Print(" to ");
  // UART2_Print_Int(next_can_id);
  // UART2_Print("\r\n");
  uint8_t txData[8] = {cmd};

  txHeader.Identifier = next_can_id;
  txHeader.IdType = FDCAN_STANDARD_ID;
  txHeader.TxFrameType = FDCAN_DATA_FRAME;
  txHeader.DataLength = FDCAN_DLC_BYTES_1;
  txHeader.FDFormat = FDCAN_CLASSIC_CAN;
  txHeader.BitRateSwitch = FDCAN_BRS_OFF;

  HAL_FDCAN_AddMessageToTxFifoQ(&hfdcan1, &txHeader, txData);

  /* Wait for response */

  CAN_Decode();
}



typedef struct ServoCfg {
  int initialized;
  uint16_t min_angle;
  uint16_t max_angle;
  uint16_t min_us;
  uint16_t max_us;
} ServoCfg;

typedef struct NameMap {
  int can_id;
  int num_names;
  struct {
    char name[15];
    uint8_t index;
  } names[8];
  union {
    ServoCfg servo;
    PTCfg pt;
  } cfg;
} NameMap;

int num_maps = 0;
NameMap name_maps[32];

int add_namemap(int can_id, char *name, uint8_t index) {
  NameMap *m = NULL;
  for (int i = 0; i < num_maps; i++) {
    if (name_maps[i].can_id == can_id) {
        m = &name_maps[i];
      break;
    }
  }
  if (num_maps == 32) return 1;
  if (m == NULL) {
    m = &name_maps[num_maps];
    num_maps++;
  }

  for (int i = 0; i < m->num_names; i++) {
    if (!strcmp(m->names[i].name, name)) return 2;
  }
  if (m->num_names == 8) return 3;
  if (strlen(name) > 14) return 4;
  strcpy(m->names[m->num_names].name, name);
  m->names[m->num_names].index = index;
  m->num_names++;
  m->can_id = can_id;
  memset(&(m->cfg), 0, sizeof(m->cfg));
  return 0;
}
int last_index;

int retrieve_id(char *label) {
  last_index = -1;
  for (int i = 0; i < num_maps; i++) {
    for (int j = 0; j < name_maps[i].num_names; j++) {
      if (!strcmp(name_maps[i].names[j].name, label)) {
        last_index = name_maps[i].names[j].index;
        return name_maps[i].can_id;
      }
    }
  }
  return atoi(label);
}
const char *dev_classes[] = {"System", "Relay", "Servo", "Thermocouple", "PT"};
void bus_list_function() {
  FDCAN_TxHeaderTypeDef txHeader = {0};
  FDCAN_RxHeaderTypeDef rxHeaders[20];
  uint8_t respTypes[20];
  FDCAN_RxHeaderTypeDef rxHeader;
  uint8_t txData[8] = {0x00};
  uint8_t rxData[8];

  txHeader.Identifier = 0;
  txHeader.IdType = FDCAN_STANDARD_ID;
  txHeader.TxFrameType = FDCAN_DATA_FRAME;
  txHeader.DataLength = FDCAN_DLC_BYTES_1;
  txHeader.FDFormat = FDCAN_CLASSIC_CAN;
  txHeader.BitRateSwitch = FDCAN_BRS_OFF;
  if (!terse) UART2_Print("Devices on bus:\r\n");
  HAL_FDCAN_AddMessageToTxFifoQ(&hfdcan1, &txHeader, txData);
  uint32_t tick = HAL_GetTick();


  int num_resp = 0;
  while (HAL_GetTick() - tick < 500) {
    if (HAL_FDCAN_GetRxFifoFillLevel(&hfdcan1, FDCAN_RX_FIFO0) != 0) {
      HAL_FDCAN_GetRxMessage(&hfdcan1, FDCAN_RX_FIFO0, &rxHeaders[num_resp], rxData);
      respTypes[num_resp++] = rxData[1];
    }
  }
  for (uint8_t i = 0; i < num_resp; i++) {
    rxHeader = rxHeaders[i];
    uint16_t resp_id = rxHeader.Identifier;
    UART2_Print_Int(resp_id);
    for (int i = 0; i < num_maps; i++) {
      if (name_maps[i].can_id == resp_id) {
        UART2_Print(" (");
        for (int j = 0; j < name_maps[i].num_names; j++) {
          UART2_Print(name_maps[i].names[j].name);
          if (name_maps[i].names[j].index != 0xFF) {
            UART2_Print("/");
            UART2_Print_Int(name_maps[i].names[j].index);
          }
          if (j != name_maps[i].num_names-1) UART2_Print(", ");
        }
        UART2_Print(")");
      }
    }
    UART2_Print(" - ");
    if (respTypes[i] > 4) {
      UART2_Print("Unknown");
    } else {
      UART2_Print(dev_classes[respTypes[i]]);
    }
    UART2_Print("\r\n");
  }
  if (!terse) UART2_Print("(end of list)\r\n");

}

void next_word(char *word, char **buf) {
  int wlen = strcspn(*buf, " ");
  memcpy(word, *buf, wlen);
  word[wlen] = 0;
  if ((*buf)[wlen]) wlen++;
  *buf += wlen;
}

int all_numeric(char *s) {
  if (!*s) return 0;
  for (int i = 0; i < strlen(s); i++) {
    if (s[i] > '9' || s[i] < '0') return 0;
  }
  return 1;
}

int cmd_invalid = 0;
void command_help(char *rest) {
  cmd_invalid = 1;
  UART2_Print("List of commands:\r\n"
    "!reset: reset STM32 remotely\r\n"
    "name - name CAN devices\r\n"
    "  name <can id> <name> [<index>]: name a device\r\n"
    "  name clear <can id>: clear names\r\n"
    "  name list <can id>: list names\r\n"
    "bus - CAN bus operations\r\n"
    "  bus reset: reset whole CAN bus\r\n"
    "  bus list: list all devices on bus\r\n"
    "sd - SD card setup\r\n"
    "  sd init: initialize microSD\r\n"
    "  sd free: print microSD free space\r\n"
    "  sd deinit: deinitalize microSD\r\n"
    "file - file operations\r\n"
    "  file run <file>: runs a file as commands\r\n"
    "  file program <file>: programs a file. Typed commands are written to the end of the file\r\n"
    "  file endp: stop programming and write commands to file\r\n"
    "  file read <file>: reads all lines in a file\r\n"
    "  file delete <file>: deletes a file\r\n"
    "  file delline <file> <line>: deletes a single line of a file\r\n"
    "  file list: lists files in root directory\r\n"
    "relay - control relays\r\n"
    "  relay off <device>: turns off a relay, supports entire boards\r\n"
    "  relay on <device>: turns on a relay\r\n"
    "  relay reset <device>: reset relay boards\r\n"
    "servo - control servos\r\n"
    "  servo off <device>: disengage servo\r\n"
    "  servo on <device>: engage servo\r\n"
    "  servo set <device> <angle>: set servo to angle\r\n"
    "  servo reset <device>: reset servo boards\r\n"
    "tc - control thermocouples\r\n"
    "  tc get <device>: reads TC temperature\r\n"
    "  tc status <device>: reads TC status flags\r\n"
    "  tc reset <device>: resets TC boards\r\n"
    "pt - control pressure transducers\r\n"
    "  pt get <device>: reads PT pressure\r\n"
    "  pt reset <device>: resets PT boards\r\n"
    "time - timer functions\r\n"
    "  time stamp: get current timestamp\r\n"
    "  time mark: set mark, time stamp shows time since mark\r\n"
    "  time sleep <ms>: sleep for a time in ms\r\n"
    "  time delay <ms> <command...>: run the command after a time in ms\r\n"
    "  time every <ms> <command...>: run the command every interval in ms\r\n"
    "  time list: list timers running\r\n"
    "  time delete [<id>]: delete that timer, default is last added\r\n"
    "  time quiet [<id>]: silence/unsilence messages from that timer\r\n"
    "msg - message functions\r\n"
    "  msg echo <message...>: prints the message out\r\n"
    "log - data logging functions\r\n"
    "  log open <file>: opens file for data logging, append to end\r\n"
    "  log close: close open log file\r\n"
    "  log sync: write new changes back to log file\r\n"
    "  log line <format...>: write a line of data to the log file; specifiers separated by spaces:\r\n"
    "   t -> time stamp in ms\r\n"
    "   m -> time since mark in ms\r\n"
    "   '<TEXT> -> that text\r\n"
    "   P<PT name> -> pressure in psi\r\n"
    "On startup or reset, startup.scr is run automatically.\r\n"
  );
}
void command_bus(char *rest) {
  char word[32];
  next_word(word, &rest);
  if (!strcmp(word, "reset")) {
    UART2_Print("Full bus reset...\r\n");
    NVIC_SystemReset();
  } else if (!strcmp(word, "list")) {
    bus_list_function();
  } else if (!strcmp(word, "vdd")) {
    char buffer[256];
    sprintf(buffer, terse ? "%dm\r\n" : "VDD = %d mV\r\n", vref_mV);
    UART2_Print(buffer);
  } else if (!*word) {
    cmd_invalid = 1;
    UART2_Print(terse ? "Invalid\r\n" : "Option needed for \"bus\".\r\n");
  } else {
    cmd_invalid = 1;
    UART2_Print(terse ? "Invalid" : "Unknown option for \"bus\": ");
    if (!terse) UART2_Print(word);
    UART2_Print("\r\n");
  }
}
void command_name(char *rest) {
  char word[32];
  next_word(word, &rest);
  if (!strcmp(word, "clear")) {
    UART2_Print("Todo\r\n");
  } else if (!strcmp(word, "list")) {
    int id = retrieve_id(rest);
    if (id == 0) {
      cmd_invalid = 1;
      UART2_Print(terse ? "Invalid\r\n" : "Format: name list <device>\r\n");
      return;
    }
    int yes = 0;
    for (int i = 0; i < num_maps; i++) {
      if (name_maps[i].can_id == id) {
        yes = 1;
        if (!terse) {
          UART2_Print("Names for device ");
          UART2_Print_Int(id);
          UART2_Print(": ");
        }
        for (int j = 0; j < name_maps[i].num_names; j++) {
          UART2_Print(name_maps[i].names[j].name);
          if (name_maps[i].names[j].index != 0xFF) {
            UART2_Print("/");
            UART2_Print_Int(name_maps[i].names[j].index);
          }
          if (j != name_maps[i].num_names - 1) UART2_Print(", ");
        }
        UART2_Print("\r\n");
      }
    }
    if (!yes && !terse) {
      UART2_Print("No names for that device.\r\n");
    }
  } else {
    int id = retrieve_id(word);
    if (id == 0) {
      UART2_Print(terse ? "Invalid\r\n" : "Option or device needed for \"name\".\r\n");
      return;
    }
    if (!*rest) {
      cmd_invalid = 1;
      UART2_Print(terse ? "Invalid\r\n" : "Format: name <device> <name> [<index>]\r\n");
    }
    next_word(word, &rest);
    int index = -1;
    if (*rest) {
      if (all_numeric(rest)) {
        index = atoi(rest);
      } else {
        cmd_invalid = 1;
        UART2_Print(terse ? "Invalid\r\n" : "Format: name <device> <name> [<index>]\r\n");
        return;
      }
    }
    int result = add_namemap(id, word, index);
    if (result == 1) {
      UART2_Print(terse ? "Err\r\n" : "Too many devices in map.\r\n");
    } else if (result == 2) {
      UART2_Print(terse ? "Err\r\n" : "Name already registered.\r\n");
    } else if (result == 3) {
      UART2_Print(terse ? "Err\r\n" : "Too many names on device.\r\n");
    } else if (result == 4) {
      UART2_Print(terse ? "Err\r\n" : "Name too long.\r\n");
    } else {
      UART2_Print(terse ? "OK\r\n" : "Name registered.\r\n");
    }
  }
}
void command_relay(char *rest) {
  /* Relay fmt:
   * 1: R1 ON
   * 2: R1 OFF
   * 3: R2 ON
   * 4: R2 OFF
   * 5: R3 ON
   * 6: R3 OFF
   * 16: Get R1
   * 17: Get R2
   * 18: Get R3
   */
  char word[32];
  next_word(word, &rest);
  if (!strcmp(word, "off")) {
    int id = retrieve_id(rest);
    if (id == 0) {
      UART2_Print(terse ? "Invalid\r\n" : "Invalid ID or device name.\r\n");
      return;
    }
    next_can_id = id;
    int cmd;
    if (!terse) {
      UART2_Print(rest);
      UART2_Print(": ");
    }

    just_set_solenoid = 1;
    just_set_id = next_can_id;
    just_set_val = 0;
    just_set_idx = last_index;
    if (last_index == 1) {
      if (!terse) UART2_Print("Turning relay OFF.\r\n");
      CAN_Send(0x02);
    } else if (last_index == 2) {
      if (!terse) UART2_Print("Turning relay OFF.\r\n");
      CAN_Send(0x04);
    } else if (last_index == 3) {
      if (!terse) UART2_Print("Turning relay OFF.\r\n");
      CAN_Send(0x06);
    } else {
      if (!terse) UART2_Print("Turning relays OFF.\r\n");
      CAN_Send(0x02);
      CAN_Send(0x04);
      CAN_Send(0x06);
    }
  } else if (!strcmp(word, "on")) {
    int id = retrieve_id(rest);
    if (id == 0) {
      UART2_Print(terse ? "Invalid\r\n" : "Invalid ID or device name.\r\n");
      return;
    }
    next_can_id = id;
    int cmd;
    if (!terse) {
      UART2_Print(rest);
      UART2_Print(": ");
    }
    just_set_solenoid = 1;
    just_set_id = next_can_id;
    just_set_val = 1;
    just_set_idx = last_index;
    if (last_index == 1) {
      if (!terse) UART2_Print("Turning relay ON.\r\n");
      CAN_Send(0x01);
    } else if (last_index == 2) {
      if (!terse) UART2_Print("Turning relay ON.\r\n");
      CAN_Send(0x03);
    } else if (last_index == 3) {
      if (!terse) UART2_Print("Turning relay ON.\r\n");
      CAN_Send(0x05);
    } else {
      UART2_Print(terse ? "Invalid\r\n" : "Need to specify individual relay.\r\n");
    }
  } else if (!strcmp(word, "get")) {
    int id = retrieve_id(rest);
    if (id == 0) {
      UART2_Print(terse ? "Invalid\r\n" : "Invalid ID or device name.\r\n");
      return;
    }
    if (last_index < 1 || last_index > 3) {
      UART2_Print(terse ? "Invalid\r\n" : "Need to specify individual relay.\r\n");
      return;
    }
    next_can_id = id;
    if (!terse) {
      UART2_Print(rest);
      UART2_Print(": ");
    }
    CAN_Rx_Func = CAN_Rx_Relay;
    CAN_Send(last_index + 0x10);
  } else if (!strcmp(word, "reset")) {
    int id = retrieve_id(rest);
    if (id == 0) {
      UART2_Print(terse ? "Invalid\r\n" : "Invalid ID or device name.\r\n");
      return;
    }
    next_can_id = id;
    if (!terse) {
      UART2_Print(rest);
      UART2_Print(": Reset board.\r\n");
    }
    CAN_Send(0xFF);
  } else if (!*word) {
    cmd_invalid = 1;
    UART2_Print(terse ? "Invalid\r\n" : "Option needed for \"relay\".\r\n");
  } else {
    cmd_invalid = 1;
    UART2_Print(terse ? "Invalid" : "Unknown option for \"relay\": ");
    if (!terse) UART2_Print(word);
    UART2_Print("\r\n");
  }
}


uint16_t servo_get_us(char *label, uint16_t angle) {
  ServoCfg *servo = NULL;
  ServoCfg def = {0};
  for (int i = 0; i < num_maps; i++) {
    if (name_maps[i].can_id == atoi(label)) {
      servo = &name_maps[i].cfg.servo;
      break;
    }
    for (int j = 0; j < name_maps[i].num_names; j++) {
      if (!strcmp(name_maps[i].names[j].name, label)) {
        servo = &name_maps[i].cfg.servo;
        break;
      }
    }
  }
  if (servo == NULL) {
    servo = &def;
  }
  if (!servo->initialized) {
    servo->initialized = 1;
    servo->min_us = 500;
    servo->max_us = 2500;
    servo->min_angle = 0;
    servo->max_angle = 180;
  }
  if (angle < servo->min_angle) return 0;
  if (angle > servo->max_angle) return 0;
  uint16_t delta = angle - servo->min_angle;
  return servo->min_us + ((uint32_t)(delta) * (servo->max_us - servo->min_us)) / (servo->max_angle - servo->min_angle);
}
uint16_t *servo_settings(char *label) {
  ServoCfg *servo = NULL;
  ServoCfg def = {0};
  for (int i = 0; i < num_maps; i++) {
    if (name_maps[i].can_id == atoi(label)) {
      servo = &name_maps[i].cfg.servo;
      break;
    }
    for (int j = 0; j < name_maps[i].num_names; j++) {
      if (!strcmp(name_maps[i].names[j].name, label)) {
        servo = &name_maps[i].cfg.servo;
        break;
      }
    }
  }
  if (servo == NULL) {
    return NULL;
  }
  if (!servo->initialized) {
    servo->initialized = 1;
    servo->min_us = 500;
    servo->max_us = 2500;
    servo->min_angle = 0;
    servo->max_angle = 180;
  }
  return &(servo->min_angle);
}
uint16_t servo_get_us_clamp(int id, uint16_t angle) {
  ServoCfg *servo = NULL;
  ServoCfg def = {0};
  for (int i = 0; i < num_maps; i++) {
    if (name_maps[i].can_id == id) {
      servo = &name_maps[i].cfg.servo;
      break;
    }
  }
  if (servo == NULL) {
    servo = &def;
  }
  if (!servo->initialized) {
    servo->initialized = 1;
    servo->min_us = 500;
    servo->max_us = 2500;
    servo->min_angle = 0;
    servo->max_angle = 180;
  }
  if (angle < servo->min_angle) angle = servo->min_angle;
  if (angle > servo->max_angle) angle = servo->max_angle;
  uint16_t delta = angle - servo->min_angle;
  return servo->min_us + ((uint32_t)(delta) * (servo->max_us - servo->min_us)) / (servo->max_angle - servo->min_angle);
}

void command_servo(char *rest) {
  /* 1: S1 ON
   * 2: S2 On
   * 3: S3 ON
   * 4: S4 On
   * 5: S1 OFF
   * 6: S2 OFF
   * 7: S3 off
   * 8: S4 off
   * 10: set S1 (next 2 bytes pulsewidth us)
   * 11: set S2
   * 12: set S3
   * 13: Set S4
   * 14: get S1
   * 15: get S2
   * 16: get S3
   * 17: get S4
   *
   */

  char word[32];
  next_word(word, &rest);
  if (!strcmp(word, "on")) {
    int id = retrieve_id(rest);
    if (id == 0) {
      UART2_Print(terse ? "Invalid\r\n" : "Invalid ID or device name.\r\n");
      return;
    }
    next_can_id = id;
    int cmd;
    if (!terse) {
      UART2_Print(rest);
      UART2_Print(": ");
    }
    if (last_index == 1) {
      if (!terse) UART2_Print("Turning servo ON.\r\n");
      CAN_Send(0x01);
    } else if (last_index == 2) {
      if (!terse) UART2_Print("Turning servo ON.\r\n");
      CAN_Send(0x02);
    } else if (last_index == 3) {
      if (!terse) UART2_Print("Turning servo ON.\r\n");
      CAN_Send(0x03);
    } else if (last_index == 4) {
      if (!terse) UART2_Print("Turning servo ON.\r\n");
      CAN_Send(0x04);
    } else {
      UART2_Print(terse ? "Invalid\r\n" : "Need to specify individual servo.\r\n");
    }
  } else if (!strcmp(word, "off")) {
    int id = retrieve_id(rest);
    if (id == 0) {
      UART2_Print(terse ? "Invalid\r\n" : "Invalid ID or device name.\r\n");
      return;
    }
    next_can_id = id;
    int cmd;
    if (!terse) {
      UART2_Print(rest);
      UART2_Print(": ");
    }
    if (last_index == 1) {
      if (!terse) UART2_Print("Turning servo OFF.\r\n");
      CAN_Send(0x05);
    } else if (last_index == 2) {
      if (!terse) UART2_Print("Turning servo OFF.\r\n");
      CAN_Send(0x06);
    } else if (last_index == 3) {
      if (!terse) UART2_Print("Turning servo OFF.\r\n");
      CAN_Send(0x07);
    } else if (last_index == 4) {
      if (!terse) UART2_Print("Turning servo OFF.\r\n");
      CAN_Send(0x08);
    } else {
      UART2_Print(terse ? "Invalid" : "Need to specify individual servo.\r\n");
    }
  } else if (!strcmp(word, "cfg")) {
    next_word(word, &rest);
    int id = retrieve_id(word);
    if (id == 0) {
      UART2_Print(terse ? "Invalid\r\n" : "Invalid ID or device name.\r\n");
      return;
    }
    uint16_t *sett_p = servo_settings(word);
    next_word(word, &rest);
    if (!strcmp(word, "minus")) {
      sett_p += 2;
    } else if (!strcmp(word, "maxus")) {
      sett_p += 3;
    } else if (!strcmp(word, "minangle")) {
      //
    } else if (!strcmp(word, "maxangle")) {
      sett_p += 1;
    } else {
      UART2_Print(terse ? "Invalid" : "Invalid setting: ");
      if (!terse) UART2_Print(word);
      UART2_Print("\r\n");
    }
    if (!*rest) {
      if (!terse) {
        UART2_Print("Cur ");
        UART2_Print(word);
        UART2_Print(" = ");
      }
      UART2_Print_Int(*sett_p);
      UART2_Print("\r\n");
    } else {
      uint16_t newval = atoi(rest);
      if (terse) {
        UART2_Print("OK\r\n");
      } else {
        UART2_Print("New ");
        UART2_Print(word);
        UART2_Print(" -> ");
        UART2_Print_Int(newval);
        UART2_Print("\r\n");
      }
      *sett_p = newval;
    }

  } else if (!strcmp(word, "set")) {
    next_word(word, &rest);
    int id = retrieve_id(word);
    if (id == 0) {
      UART2_Print(terse ? "Invalid\r\n" : "Invalid ID or device name.\r\n");
      return;
    }
    uint16_t angle = atoi(rest);
    if (angle == 0 && *rest != '0') {
      UART2_Print(terse ? "Err\r\n" : "Invalid angle.\r\n");
      return;
    }
    uint16_t serv_us = servo_get_us(word, angle);
    if (serv_us == 0) {
      UART2_Print(terse ? "Err\r\n" : "Angle out of range.\r\n");
      return;
    }
    char buf[3];
    if (last_index < 1 || last_index > 4) {
      UART2_Print(terse ? "Invalid\r\n" : "Need to specify individual servo.\r\n");
      return;
    }
    if (!terse) {
      UART2_Print(rest);
      UART2_Print(": Setting servo -> ");
      UART2_Print_Int(angle);
      UART2_Print(" deg (");
      UART2_Print_Int(serv_us);
      UART2_Print(" us).\r\n");
    } else {
      UART2_Print("OK\r\n");
    }
    buf[0] = 0x10 + last_index;
    *(uint16_t*)&buf[1] = serv_us;
    CAN_Send_Long(buf, 3);
  } else if (!strcmp(word, "reset")) {
    int id = retrieve_id(rest);
    if (id == 0) {
      UART2_Print(terse ? "Invalid\r\n" : "Invalid ID or device name.\r\n");
      return;
    }
    next_can_id = id;
    if (!terse) {
      UART2_Print(rest);
      UART2_Print(": Reset board.\r\n");
    } else {
      UART2_Print("OK\r\n");
    }
    CAN_Send(0xFF);
  } else if (!*word) {
    cmd_invalid = 1;
    UART2_Print(terse ? "Invalid\r\n" : "Option needed for \"servo\".\r\n");
  } else {
    cmd_invalid = 1;
    UART2_Print(terse ? "Invalid" : "Unknown option for \"servo\": ");
    if (!terse) UART2_Print(word);
    UART2_Print("\r\n");
  }
}
void command_tc(char *rest) {
  char word[32];
  next_word(word, &rest);
  if (!strcmp(word, "get")) {
    int id = retrieve_id(rest);
    if (id == 0) {
      UART2_Print(terse ? "Invalid\r\n" :"Invalid ID or device name.\r\n");
      return;
    }
    if (last_index < 1 || last_index > 3) {
      UART2_Print(terse ? "Invalid\r\n" :"Need to specify individual TC.\r\n");
      return;
    }
    next_can_id = id;
    UART2_Print(rest);
    if (!terse) UART2_Print(": ");
    CAN_Rx_Func = CAN_Rx_TC;
    CAN_Send(last_index);
  } else if (!strcmp(word, "status")) {
    int id = retrieve_id(rest);
    if (id == 0) {
      UART2_Print(terse ? "Invalid\r\n" :"Invalid ID or device name.\r\n");
      return;
    }
    if (last_index < 1 || last_index > 3) {
      UART2_Print(terse ? "Invalid\r\n" :"Need to specify individual TC.\r\n");
      return;
    }
    next_can_id = id;
    UART2_Print(rest);
    if (!terse) UART2_Print(": ");
    CAN_Rx_Func = CAN_Rx_TCStat;
    CAN_Send(0x03 + last_index);
  } else if (!strcmp(word, "reset")) {
    int id = retrieve_id(rest);
    if (id == 0) {
      UART2_Print(terse ? "Invalid\r\n" :"Invalid ID or device name.\r\n");
      return;
    }
    next_can_id = id;
    if (!terse) {
      UART2_Print(rest);
      UART2_Print(": Reset board.\r\n");
    } else {
      UART2_Print("OK\r\n");
    }
    CAN_Send(0xFF);
  } else if (!*word) {
    cmd_invalid = 1;
    UART2_Print(terse ? "Invalid\r\n" :"Option needed for \"tc\".\r\n");
  } else {
    cmd_invalid = 1;
    UART2_Print(terse ? "Invalid" : "Unknown option for \"tc\": ");
    if (!terse) UART2_Print(word);
    UART2_Print("\r\n");
  }

}
void command_pt(char *rest) {
  char word[32];
  next_word(word, &rest);
  if (!strcmp(word, "get")) {
    int id = retrieve_id(rest);
    if (id == 0) {
      UART2_Print(terse ? "Invalid\r\n" :"Invalid ID or device name.\r\n");
      return;
    }
    if (last_index < 1 || last_index > 3) {
      UART2_Print(terse ? "Invalid\r\n" :"Need to specify individual PT.\r\n");
      return;
    }
    next_can_id = id;
    UART2_Print(rest);
    UART2_Print(": ");
    CAN_Rx_Func = CAN_Rx_PT;
    CAN_Send(last_index);
  } else if (!strcmp(word, "cfg")) {
    next_word(word, &rest);
    int id = retrieve_id(word);
    if (id == 0) {
      UART2_Print(terse ? "Invalid\r\n" : "Invalid ID or device name.\r\n");
      return;
    }
    PTCfg *pt = NULL;
    PTCfg def = {0};
    for (int i = 0; i < num_maps; i++) {
      if (name_maps[i].can_id == id) {
        pt = &name_maps[i].cfg.pt;
        break;
      }
    }
    if (pt == NULL) {
      pt = &def;
    }
    if (!pt->initialized) {
      pt->initialized = 1;
      pt->max_psi = 3000;
    }
    next_word(word, &rest);
    if (!strcmp(word, "max")) {
      if (!*rest) {
        if (!terse) {
          UART2_Print("Cur ");
          UART2_Print(word);
          UART2_Print(" = ");
        }
        UART2_Print_Int(pt->max_psi);
        UART2_Print("\r\n");
      } else {
        pt->max_psi = atoi(rest);
        if (terse) {
          UART2_Print("OK\r\n");
        } else {
          UART2_Print("New ");
          UART2_Print(word);
          UART2_Print(" -> ");
          UART2_Print_Int(pt->max_psi);
          UART2_Print("\r\n");
        }
      }
    }
  } else if (!strcmp(word, "reset")) {
    int id = retrieve_id(rest);
    if (id == 0) {
      UART2_Print(terse ? "Invalid\r\n" :"Invalid ID or device name.\r\n");
      return;
    }
    next_can_id = id;
    if (!terse) {
      UART2_Print(rest);
      UART2_Print(": Reset board.\r\n");
    } else {
      UART2_Print("OK\r\n");
    }
    CAN_Send(0xFF);
  } else if (!*word) {
    cmd_invalid = 1;
    UART2_Print(terse ? "Invalid\r\n" :"Option needed for \"pt\".\r\n");
  } else {
    cmd_invalid = 1;
    UART2_Print(terse ? "Invalid" :"Unknown option for \"pt\": ");
    if (!terse) UART2_Print(word);
    UART2_Print("\r\n");
  }

}

int sd_ok = 0;
void initialize_sd();
void sd_free_space();
void deinit_sd();
void command_sd(char *rest) {
  char word[32];
  next_word(word, &rest);
  if (!strcmp(word, "init")) {
    if (sd_ok) UART2_Print("SD card already initialized.\r\n");
    else initialize_sd();
  } else if (!strcmp(word, "free")) {
    if (sd_ok) sd_free_space();
    else UART2_Print("SD card not initialized.\r\n");
  } else if (!strcmp(word, "deinit")) {
    if (sd_ok) deinit_sd();
    else UART2_Print("SD card not initialized.\r\n");
  } else if (!*word) {
    cmd_invalid = 1;
    UART2_Print(terse ? "Invalid\r\n" :"Option needed for \"sd\".\r\n");
  } else {
    cmd_invalid = 1;
    UART2_Print(terse ? "Invalid" :"Unknown option for \"sd\": ");
    if (!terse) UART2_Print(word);
    UART2_Print("\r\n");
  }
}

FATFS FatFs;
FIL pfil;
int cur_programming = 0;
void list_all_files() {
  FRESULT fres;
  DIR dir;
  FILINFO filinfo;
  fres = f_opendir(&dir, "/");
  char buffer[256];
  if (fres != FR_OK) {
    UART2_Print("Unable to open directory.\r\n");
    return;
  }
  UART2_Print("Files on SD card:\r\n");
  while (1) {
    fres = f_readdir(&dir, &filinfo);
    if (fres != FR_OK || filinfo.fname[0] == 0) break;
    unsigned long size = filinfo.fsize;
    const char *prefix;
    if (size > 10*1073741824) {
      prefix = "GiB";
      size >>= 30;
    } else if (size > 10*1048576) {
      prefix = "MiB";
      size >>= 20;
    } else if (size > 10*1024) {
      prefix = "KiB";
      size >>= 10;
    } else {
      prefix = "B";
    }
    sprintf(buffer, "%-18s %lu %s\r\n", filinfo.fname, size, prefix);
    UART2_Print(buffer);
  }
  f_closedir(&dir);
}
void process_command(char *buf);
void command_file(char *rest) {
  FIL fil;
  FRESULT fres;
  char word[32];
  if (!sd_ok) {
    UART2_Print("SD card must be initialized to use \"file\".\r\n");
    return;
  } else if (cur_programming && strcmp(word, "endp")) {
    UART2_Print("Can't use \"file\" commands while programming.\r\n");
  }
  next_word(word, &rest);
  if (!strcmp(word, "list")) {
    list_all_files();
  } else if (!strcmp(word, "delete")) {
    fres = f_unlink(rest);
    if (fres != FR_OK) {
      UART2_Print("Unable to open file.\r\n");
      return;
    } else {
      UART2_Print("File deleted.\r\n");
    }
  } else if (!strcmp(word, "read")) {
    char cmdbuf[320];
    fres = f_open(&fil, rest, FA_READ);
    if (fres != FR_OK) {
      UART2_Print("Unable to open file.\r\n");
      return;
    }
    UART2_Print("File contents:\r\n");
    char lnbuf[8];
    for (int i = 0; ;i++) {
      TCHAR *rres = f_gets((TCHAR*)cmdbuf, 320, &fil);
      if (cmdbuf[strlen(cmdbuf)-1] == '\n') cmdbuf[strlen(cmdbuf)-1] = 0;
      if (rres == 0) break;
      sprintf(lnbuf, "%04d|", i+1);
      UART2_Print(lnbuf);
      UART2_Print(cmdbuf);
      UART2_Print("\r\n");
    }
    f_close(&fil);
  } else if (!strcmp(word, "readslow")) {
    char cmdbuf[320];
    fres = f_open(&fil, rest, FA_READ);
    if (fres != FR_OK) {
      UART2_Print("Unable to open file.\r\n");
      return;
    }
    UART2_Print("File contents:\r\n");
    char lnbuf[8];
    for (int i = 0; ;i++) {
      TCHAR *rres = f_gets((TCHAR*)cmdbuf, 320, &fil);
      if (cmdbuf[strlen(cmdbuf)-1] == '\n') cmdbuf[strlen(cmdbuf)-1] = 0;
      if (rres == 0) break;
      sprintf(lnbuf, "%04d|", i+1);
      UART2_Print(lnbuf);
      UART2_Print(cmdbuf);
      UART2_Print("\r\n");
      HAL_Delay(20);
    }
    f_close(&fil);
  } else if (!strcmp(word, "delline")) {
    next_word(word, &rest);
    int delline = atoi(rest);
    if (delline == 0) {
      UART2_Print("Format: file delline <file> <linenum>\r\n");
      return;
    }
    fres = f_open(&fil, word, FA_READ);
    if (fres != FR_OK) {
      UART2_Print("Unable to open file.\r\n");
      return;
    }
    FIL fil2;
    fres = f_open(&fil2, "tmp", FA_WRITE | FA_CREATE_ALWAYS);
    if (fres != FR_OK) {
      UART2_Print("Unable to open temporary file.\r\n");
      return;
    }
    char linebuf[320];
    int curline = 1;
    while (f_gets(linebuf, 320, &fil)) {
      if (curline != delline) {
        f_puts(linebuf, &fil2);
      }
      curline++;
    }
    f_close(&fil);
    f_close(&fil2);
    fres = f_unlink(word);
    if (fres != FR_OK) {
      UART2_Print("Unable to delete old file.\r\n");
      return;
    }
    fres = f_rename("tmp", word);
    if (fres != FR_OK) {
      UART2_Print("Unable to rename file; temporary copy saved in \"tmp\".\r\n");
      return;
    }
    UART2_Print("Line deleted.\r\n");
  } else if (!strcmp(word, "run")) {
    if (!*rest) rest = "startup.scr";
    char cmdbuf[320];
    fres = f_open(&fil, rest, FA_READ);
    if (fres != FR_OK) {
      UART2_Print("Unable to open file.\r\n");
      return;
    }
    while (1) {
      TCHAR *rres = f_gets((TCHAR*)cmdbuf, 320, &fil);
      if (cmdbuf[strlen(cmdbuf)-1] == '\n') cmdbuf[strlen(cmdbuf)-1] = 0;
      if (rres == 0) break;
      process_command(cmdbuf);
    }
    f_close(&fil);
  } else if (!strcmp(word, "program")) {
    if (!*rest) rest = "startup.scr";
    fres = f_open(&pfil, rest, FA_WRITE | FA_OPEN_APPEND);
    if (fres != FR_OK) {
      UART2_Print("Unable to open file.\r\n");
      return;
    }
    cur_programming = 1;
  } else if (!*word) {
    cmd_invalid = 1;
    UART2_Print(terse ? "Invalid\r\n" :"Option needed for \"file\".\r\n");
  } else {
    cmd_invalid = 1;
    UART2_Print(terse ? "Invalid" :"Unknown option for \"file\": ");
    if (!terse) UART2_Print(word);
    UART2_Print("\r\n");
  }
}
void command_msg(char *rest) {
  char word[32];
  next_word(word, &rest);
  if (!strcmp(word, "echo")) {
    UART2_Print(rest);
    UART2_Print("\r\n");
  } else if (!strcmp(word, "quiet")) {
    muted |= 2;
  } else if (!strcmp(word, "unquiet")) {
    muted &= ~2;
  } else if (!strcmp(word, "terse")) {
    terse = 1;
  } else if (!strcmp(word, "unterse")) {
    terse = 0;
  } else if (!*word) {
    cmd_invalid = 1;
    UART2_Print(terse ? "Invalid\r\n" :"Option needed for \"msg\".\r\n");
  } else {
    cmd_invalid = 1;
    UART2_Print(terse ? "Invalid" :"Unknown option for \"msg\": ");
    UART2_Print(word);
    UART2_Print("\r\n");
  }
}
struct TimeEntry {
  char cmdbuf[320];
  int reload;
  int timeout;
  int quiet;
  struct TimeEntry *next;
};
typedef struct TimeEntry TimeEntry;
TimeEntry *root = NULL;

void add_time_entry(char *cmd, int reload, int timeout) {
  TimeEntry **t = &root;
  while (*t != NULL) {
    t = &((*t)->next);
  }
  *t = malloc(sizeof(TimeEntry));
  strcpy((*t)->cmdbuf, cmd);
  (*t)->reload = reload;
  (*t)->timeout = timeout;
  (*t)->next = NULL;
  (*t)->quiet = 0;
}
float pt_translate_mv(int id, int16_t pt_p){
  PTCfg *pt = NULL;
  PTCfg def = {0};
  for (int i = 0; i < num_maps; i++) {
    if (name_maps[i].can_id == id) {
      pt = &name_maps[i].cfg.pt;
      break;
    }
  }
  if (pt == NULL) {
    pt = &def;
  }
  if (!pt->initialized) {
    pt->initialized = 1;
    pt->max_psi = 3000;
  }
  return ((float)pt->max_psi) * ((float)pt_p - 500.0f) / 4000.0f;
}
int time_mark = 0;
void main_tick() {
  TimeEntry *t = root;
  while (t != NULL) {
    t->timeout--;
    t = t->next;
  }
}
void command_time(char *rest) {
  char word[32];
  next_word(word, &rest);
  if (!strcmp(word, "stamp")) {
    char buff[64];
    sprintf(buff, terse ? "%dm" : "Timestamp %dms", HAL_GetTick());
    UART2_Print(buff);
    if (time_mark) {
      sprintf(buff, terse ? ",%dm" : ", %dms since mark\r\n", HAL_GetTick() - time_mark);
      UART2_Print(buff);
    } else {
      UART2_Print("\r\n");
    }
  } else if (!strcmp(word, "mark")) {
    char buff[64];
    time_mark = HAL_GetTick();
    sprintf(buff, terse ? "OK" : "Mark set at %dms\r\n", HAL_GetTick());
    UART2_Print(buff);
  } else if (!strcmp(word, "sleep")) {
    int ms = atoi(rest);
    if (ms == 0) {
      UART2_Print(terse ? "Invalid\r\n" :"Format: time sleep <ms>\r\n");
      return;
    }
    HAL_Delay(ms);
  } else if (!strcmp(word, "delay")) {
    next_word(word, &rest);
    int ms = atoi(word);
    if (ms == 0 || !*rest) {
      UART2_Print(terse ? "Invalid\r\n" :"Format: time delay <ms> <command...>\r\n");
      return;
    }
    add_time_entry(rest, 0, ms);
    if (terse) {
      UART2_Print("OK\r\n");
    } else {
      UART2_Print("Running command in ");
      UART2_Print_Int(ms);
      UART2_Print("ms.\r\n");
    }
  } else if (!strcmp(word, "every")) {
    next_word(word, &rest);
    int ms = atoi(word);
    if (ms == 0 || !*rest) {
      UART2_Print(terse ? "Invalid\r\n" :"Format: time every <ms> <command...>\r\n");
      return;
    }
    add_time_entry(rest, ms, ms);
    if (terse) {
      UART2_Print("OK\r\n");
    } else {
      UART2_Print("Running command every ");
      UART2_Print_Int(ms);
      UART2_Print("ms.\r\n");
    }
  } else if (!strcmp(word, "list")) {
    if (root == NULL) {
      UART2_Print(terse ? "None\r\n" :"No timers active.\r\n");
      return;
    }
    TimeEntry *t = root;
    if (!terse) UART2_Print("Timers active:\r\n");
    int i = 0;
    while (t != NULL) {
      UART2_Print_Int(++i);
      if (t->reload) {
        UART2_Print(" Every ");
        UART2_Print_Int(t->reload);
      } else {
        UART2_Print(" In ");
        UART2_Print_Int(t->timeout);
      }
      UART2_Print("ms: ");
      UART2_Print(t->cmdbuf);
      UART2_Print("\r\n");
      t = t->next;
    }
  } else if (!strcmp(word, "delete")) {
    int idx = atoi(rest);
    if (idx == 0 && *rest) {
      UART2_Print(terse ? "Invalid\r\n" :"Format: time delete <index>\r\n");
      return;
    }
    if (root == NULL) {
      UART2_Print(terse ? "None\r\n" :"No timers active.\r\n");
      return;
    }
    TimeEntry *t = root;
    TimeEntry **last = &root;
    int i = 0;
    while (t != NULL) {
      i++;
      if (i == idx || (!idx && t->next == NULL)) {
        *last = t->next;
        free(t);
        break;
      }
      last = &(t->next);
      t = t->next;
    }
    if (t == NULL) {
      UART2_Print(terse ? "Invalid\r\n" :"No timer with that index.\r\n");
    } else {
      UART2_Print(terse ? "OK\r\n" :"Timer deleted.\r\n");
    }
  } else if (!strcmp(word, "quiet")) {
    int idx = atoi(rest);
    if (idx == 0 && *rest) {
      UART2_Print(terse ? "Invalid\r\n" : "Format: time quiet <index>\r\n");
      return;
    }
    if (root == NULL) {
      UART2_Print(terse ? "None" : "No timers active.\r\n");
      return;
    }
    TimeEntry *t = root;
    TimeEntry **last = &root;
    int i = 0;
    while (t != NULL) {
      i++;
      if (i == idx || (!idx && t->next == NULL)) {
        t->quiet ^= 1;
        break;
      }
      last = &(t->next);
      t = t->next;
    }
    if (t == NULL) {
      UART2_Print(terse ? "Invalid\r\n" : "No timer with that index.\r\n");
    } else {
      UART2_Print(t->quiet ? "Timer silenced.\r\n" : "Timer unsilenced.\r\n");
    }
  } else if (!*word) {
    cmd_invalid = 1;
    UART2_Print(terse ? "Invalid\r\n" :"Option needed for \"time\".\r\n");
  } else {
    cmd_invalid = 1;
    UART2_Print(terse ? "Invalid" :"Unknown option for \"time\": ");
    if (!terse) UART2_Print(word);
    UART2_Print("\r\n");
  }
}
float control_last_rd;
float control_last_err;
float control_last_eff;
int control_last_ts = 0;
FIL logfile;
int islogopen = 0;
void command_log(char *rest) {
  FRESULT fres;
  char word[32];
  next_word(word, &rest);
  if (!strcmp(word, "open")) {
    if (islogopen) {
      UART2_Print(terse ? "Err\r\n" :"Log file already open.\r\n");
      return;
    }
    if (!*rest) {
      UART2_Print(terse ? "Invalid\r\n" :"Format: log open <filename>.\r\n");
      return;
    }
    fres = f_open(&logfile, rest, FA_WRITE | FA_OPEN_APPEND);
    if (fres != FR_OK) {
      UART2_Print(terse ? "Err\r\n" :"Unable to open log file.\r\n");
      return;
    }
    islogopen = 1;
  } else if (!strcmp(word, "close")) {
    if (!islogopen) {
      UART2_Print(terse ? "Err\r\n" :"Log file is not open.\r\n");
      return;
    }
    islogopen = 0;
    fres = f_sync(&logfile);
    if (fres != FR_OK) {
      UART2_Print(terse ? "Err\r\n" :"Unable to sync log file.\r\n");
      f_close(&logfile);
      return;
    }
    fres = f_close(&logfile);
    if (fres != FR_OK) {
      UART2_Print(terse ? "Err\r\n" :"Unable to close log file.\r\n");
      return;
    }
  } else if (!strcmp(word, "sync")) {
    if (!islogopen) {
      UART2_Print(terse ? "Err\r\n" :"Log file is not open.\r\n");
      return;
    }
    fres = f_sync(&logfile);
    if (fres != FR_OK) {
      UART2_Print(terse ? "Err\r\n" :"Unable to sync log file.\r\n");
      return;
    }
  } else if (!strcmp(word, "line") || !strcmp(word, "print")) {
    if (!islogopen) {
      UART2_Print(terse ? "Err\r\n" :"Log file is not open.\r\n");
      return;
    }
    char buffer[1024];
    char *bufp = buffer;
    while (*rest) {
      next_word(word, &rest);
      if (word[0] == '\'') {
        strcpy(bufp, word+1);
      } else if (word[0] == 't') {
        sprintf(bufp, "%d", HAL_GetTick());
      } else if (word[0] == 'm') {
        sprintf(bufp, "%d", HAL_GetTick() - time_mark);
      } else if (word[0] == 'P') {
        int id = retrieve_id(word+1);
        if (id == 0) {
          UART2_Print(terse ? "Invalid" :"Invalid PT name: ");
          if (!terse) UART2_Print(word + 1);
          UART2_Print("\r\n");
          return;
        }
        if (last_index < 1 || last_index > 3) {
          UART2_Print(terse ? "Invalid\r\n" :"Need to specify individual PT.\r\n");
          return;
        }
        next_can_id = id;
        last_pt_v = 0;
        CAN_Rx_Func = CAN_Rx_PT_Quiet;
        CAN_Send(last_index);
        sprintf(bufp, "%.2f", last_pt_v);
      } else if (word[0] == 'c') {
        if (word[1] == 's') sprintf(bufp, "%.3f", control_last_rd);
        if (word[1] == 'e') sprintf(bufp, "%.3f", control_last_err);
        if (word[1] == 'a') sprintf(bufp, "%.3f", control_last_eff);
        if (word[1] == 't') sprintf(bufp, "%d", control_last_ts);
      } else if (word[0] == 'L') {
        sprintf(bufp, "%.2f", HX711_ReadWeight(&hx));
      } else {
        UART2_Print(terse ? "Invalid\r\n" :"Invalid line specifier: ");
        if (!terse) UART2_Print(word);
        return;
      }
      int ilen = strlen(bufp);
      bufp[ilen] = *rest ? ',' : '\n';
      bufp[ilen + 1] = 0;
      bufp += ilen + 1;
    }
    if (!strcmp(word, "line")) f_puts(buffer, &logfile);
    if (!terse) UART2_Print("Log line: ");
    UART2_Print(buffer);
  } else if (!*word) {
    cmd_invalid = 1;
    UART2_Print(terse ? "Invalid\r\n" :"Option needed for \"log\".\r\n");
  } else {
    cmd_invalid = 1;
    UART2_Print(terse ? "Invalid" :"Unknown option for \"log\": ");
    if (!terse) UART2_Print(word);
    UART2_Print("\r\n");
  }
}
float control_p;
float control_i;
float control_d;
int control_stype;
int control_sid = 0;
int control_sidx = 0;
int control_atype;
int control_aid = 0;
int control_aidx = 0;
float control_setp;
float control_int;
float control_mina;
float control_maxa;
float control_read() {
  if (control_stype == 0) {
    next_can_id = control_sid;
    last_pt_v = 0;
    CAN_Rx_Func = CAN_Rx_PT_Quiet;
    CAN_Send(control_sidx);
    return last_pt_v;
  } else if (control_stype == 1) {
    next_can_id = control_sid;
    last_pt_v = 0;
    CAN_Rx_Func = CAN_Rx_PT_Quiet;
    CAN_Send(control_sidx);
    float pt1 = last_pt_v;
    CAN_Rx_Func = CAN_Rx_PT_Quiet;
    CAN_Send(control_sidx+1);
    return pt1 - last_pt_v;
  } else {
    return 0;
  }
}
void control_write(float f) {
  if (control_atype == 0) {
    char buf[3];
    uint16_t serv_us = servo_get_us_clamp(control_aid, f);
    buf[0] = 0x10 + control_aidx;
    *(uint16_t*)&buf[1] = serv_us;
    next_can_id = control_aid;
    CAN_Send_Long(buf, 3);
  }
}
void command_loop(char *rest) {
  char buffer[256];
  char word[32];
  next_word(word, &rest);
  if (!strcmp(word, "p")) {
    if (*rest) {
      float val = atof(rest);
      sprintf(buffer, terse ? "OK\r\n" : "New P = %f\r\n", val);
      control_p = val;
    } else {
      sprintf(buffer, terse ? "%f\r\n" : "Current P = %f\r\n", control_p);
    }
    UART2_Print(buffer);
  } else if (!strcmp(word, "i")) {
    if (*rest) {
      float val = atof(rest);
      sprintf(buffer, terse ? "OK\r\n" : "New I = %f\r\n", val);
      control_i = val;
    } else {
      sprintf(buffer, terse ? "%f\r\n" : "Current I = %f\r\n", control_i);
    }
    UART2_Print(buffer);
  } else if (!strcmp(word, "d")) {
    if (*rest) {
      float val = atof(rest);
      sprintf(buffer, terse ? "OK\r\n" : "New D = %f\r\n", val);
      control_d = val;
    } else {
      sprintf(buffer, terse ? "%f\r\n" : "Current D = %f\r\n", control_d);
    }
    UART2_Print(buffer);
  } else if (!strcmp(word, "min")) {
    if (*rest) {
      float val = atof(rest);
      sprintf(buffer, terse ? "OK\r\n" : "New min = %f\r\n", val);
      control_mina = val;
    } else {
      sprintf(buffer, terse ? "%f\r\n" : "Current min = %f\r\n", control_mina);
    }
    UART2_Print(buffer);
  } else if (!strcmp(word, "max")) {
    if (*rest) {
      float val = atof(rest);
      sprintf(buffer, terse ? "OK\r\n" : "New max = %f\r\n", val);
      control_maxa = val;
    } else {
      sprintf(buffer, terse ? "%f\r\n" : "Current max = %f\r\n", control_maxa);
    }
    UART2_Print(buffer);
  } else if (!strcmp(word, "sensor")) {
    next_word(word, &rest);
    if (!strcmp(word, "pt")) {
      control_stype = 0;
    } else if (!strcmp(word, "dpt")) {
      control_stype = 1;
    } else {
      UART2_Print(terse ? "Invalid" :"Invalid sensor type: ");
      if (!terse) UART2_Print(word);
      UART2_Print("\r\n");
      return;
    }
    int id = retrieve_id(rest);
    if (id == 0) {
      UART2_Print(terse ? "Invalid" :"Invalid sensor name: ");
      if (!terse)UART2_Print(rest);
      UART2_Print("\r\n");
      return;
    }
    control_sid = id;
    control_sidx = last_index;
    if (terse) {
      UART2_Print("OK\r\n");
    } else {
      UART2_Print("Now reading from ");
      UART2_Print(rest);
      UART2_Print("\r\n");
    }

  } else if (!strcmp(word, "act")) {
    next_word(word, &rest);
    if (!strcmp(word, "servo")) {
      control_atype = 0;
    } else {
      UART2_Print(terse ? "Invalid" :"Invalid actuator type: ");
      if (!terse) UART2_Print(word);
      UART2_Print("\r\n");
      return;
    }
    int id = retrieve_id(rest);
    if (id == 0) {
      UART2_Print(terse ? "Invalid" :"Invalid actuator name: ");
      if (!terse) UART2_Print(rest);
      UART2_Print("\r\n");
      return;
    }
    control_aid = id;
    control_aidx = last_index;
    if (terse) {
      UART2_Print("OK\r\n");
    } else {
      UART2_Print("Now writing to ");
      UART2_Print(rest);
      UART2_Print("\r\n");
    }
  } else if (!strcmp(word, "setp")) {
    if (*rest) {
      float val = atof(rest);
      sprintf(buffer, terse ? "OK\r\n" : "New setpoint = %f\r\n", val);
      control_setp = val;
    } else {
      sprintf(buffer, terse ? "%f\r\n" : "Current setpoint = %f\r\n", control_setp);
    }
    UART2_Print(buffer);
  } else if (!strcmp(word, "run")) {
    if (!control_sid) {
      UART2_Print(terse ? "Invalid\r\n" :"Control source not configured.\r\n");
      return;
    }
    if (!control_aid) {
      UART2_Print(terse ? "Invalid\r\n" :"Control actuator not configured.\r\n");
      return;
    }
    float dt = 0.001f;
    control_last_ts = HAL_GetTick();
    control_last_rd = control_read();
    float prev = control_last_err;
    control_last_err = control_last_rd - control_setp; // sus
    control_int += control_last_err * dt;
    control_last_eff = control_p * control_last_err
                     + control_i * control_int
                     + control_d * (control_last_err - prev) / dt;
    if (control_last_eff < control_mina) control_last_eff = control_mina;
    if (control_last_eff > control_maxa) control_last_eff = control_maxa;
    control_write(control_last_eff);
  } else if (!strcmp(word, "stat")) {
    if (!control_last_ts) {
      UART2_Print(terse ? "Invalid\r\n" :"Loop hasn't been run.\r\n");
      return;
    }
    sprintf(buffer, terse ? "%dm,%f,%f,%f\r\n" : "Last run at %dms, S=%f Err=%f A=%f\r\n", control_last_ts, control_last_rd, control_last_err, control_last_eff);
    UART2_Print(buffer);
  } else if (!*word) {
    cmd_invalid = 1;
    UART2_Print(terse ? "Invalid\r\n" :"Option needed for \"loop\".\r\n");
  } else {
    cmd_invalid = 1;
    UART2_Print(terse ? "Invalid" :"Unknown option for \"loop\": ");
    if (!terse) UART2_Print(word);
    UART2_Print("\r\n");
  }
}
void command_lc(char *rest) {
  char word[32];
  next_word(word, &rest);
  if (!strcmp(word, "init")) {
    HX711_Init(&hx);
  } else if (!strcmp(word, "setup")) {
    next_word(word, &rest);
    float scale = atof(word);
    float offs = atof(rest);
    HX711_SetParams(&hx, scale, offs);
  } else if (!strcmp(word, "read")) {
    float weight = HX711_ReadWeight(&hx);
    char buffer[256];
    sprintf(buffer, "LC = %.2f lb\r\n", weight);
    UART2_Print(buffer);
  }
}
int prog_lines = 0;
int adventuremain();
void process_command(char *buf) {
  char word[32];
  FRESULT fres;
  // int wlen = strcspn(buf, " ");
  // memcpy(word, buf, wlen);
  // word[wlen] = 0;
  // buf += wlen + 1;
  if (cur_programming) {
    if (!strcmp(buf, "file endp")) {
      char buffer[64];
      sprintf(buffer, terse ? "OK %d\r\n" :"Programmed %d lines.\r\n", prog_lines);
      UART2_Print(buffer);
      prog_lines = 0;
      cur_programming = 0;
      f_close(&pfil);
    } else {
      int len = strlen(buf);
      buf[len] = '\n';
      buf[++len] = 0;
      int olen;
      fres = f_write(&pfil, buf, len, &olen);
      if (fres != FR_OK) {
        UART2_Print(terse ? "Err\r\n" : "Warning, write failed.\r\n");
      } else {
        prog_lines += 1;
      }
      f_sync(&pfil);
    }
    return;
  }
  next_word(word, &buf);
  cmd_invalid = 0;
  if (!strcmp(word, "help")) {
    command_help(buf);
  } else if (!strcmp(word, "bus")) {
    command_bus(buf);
  } else if (!strcmp(word, "name")) {
    command_name(buf);
  } else if (!strcmp(word, "relay")) {
    command_relay(buf);
  } else if (!strcmp(word, "sd")) {
    command_sd(buf);
  } else if (!strcmp(word, "file")) {
    command_file(buf);
  } else if (!strcmp(word, "servo")) {
    command_servo(buf);
  } else if (!strcmp(word, "tc")) {
    command_tc(buf);
  } else if (!strcmp(word, "time")) {
    command_time(buf);
  } else if (!strcmp(word, "msg")) {
    command_msg(buf);
  } else if (!strcmp(word, "pt")) {
    command_pt(buf);
  } else if (!strcmp(word, "log")) {
    command_log(buf);
  } else if (!strcmp(word, "loop")) {
    command_loop(buf);
  } else if (!strcmp(word, "lc")) {
    command_lc(buf);
  } else if (!strcmp(word, "adventure")) {
    //adventuremain();
  } else if (!*word) {
    cmd_invalid = 1;
    UART2_Print(terse ? "Invalid\r\n" : "Enter a command.\r\n");
  } else {
    cmd_invalid = 1;
    UART2_Print(terse ? "Invalid" : "Unknown command: ");
    if (!terse) UART2_Print(word);
    UART2_Print("\r\n");
  }
}
void initialize_sd() {
  FRESULT fres;
  UART2_Print("Mounting SD card...\r\n");
  fres = f_mount(&FatFs, "", 1); //1=mount now
  if (fres != FR_OK) {
    UART2_Print("f_mount error\r\n");
    sd_ok = 0;
    return;
  }


  UART2_Print("SD card setup.\r\n");
  sd_ok = 1;
  return;
}
void deinit_sd() {
  f_mount(NULL, "", 0);
  UART2_Print("SD card removed.\r\n");
  sd_ok = 0;
  return;
}
void sd_free_space() {
  FATFS* getFreeFs;
  FRESULT fres;
  DWORD free_clusters, free_sectors, total_sectors;
  fres = f_getfree("", &free_clusters, &getFreeFs);
  if (fres != FR_OK) {
    UART2_Print("f_getfree error\r\n");
    return;
  }
  //Formula comes from ChaN's documentation
  total_sectors = (getFreeFs->n_fatent - 2) * getFreeFs->csize;
  free_sectors = free_clusters * getFreeFs->csize;
  char buffer[256];
  sprintf(buffer, "SD card space:\r\n%10lu MiB total drive space.\r\n%10lu MiB available.\r\n", total_sectors / 2048, free_sectors / 2048);
  UART2_Print(buffer);
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
  MX_ADC1_Init();
  MX_FDCAN1_Init();
  MX_I2C1_Init();
  MX_SPI1_Init();
  MX_SPI2_Init();
  MX_USART1_UART_Init();
  MX_USART2_UART_Init();
  if (MX_FATFS_Init() != APP_OK) {
    Error_Handler();
  }
  MX_TIM2_Init();
  /* USER CODE BEGIN 2 */
  HAL_Delay(500);
  LL_ADC_SetCommonPathInternalCh(__LL_ADC_COMMON_INSTANCE(ADC1),
                                 LL_ADC_PATH_INTERNAL_VREFINT);

  HAL_ADCEx_Calibration_Start(&hadc1);
  HAL_ADC_Start_DMA(&hadc1, (uint32_t*)&vref_raw, 1);
  HAL_TIM_Base_Start(&htim2);
  HAL_FDCAN_Start(&hfdcan1);

  HAL_ADC_Stop_DMA(&hadc1);

  hadc1.Init.ExternalTrigConv = ADC_SOFTWARE_START;
  hadc1.Init.ExternalTrigConvEdge = ADC_EXTERNALTRIGCONVEDGE_NONE;
  HAL_ADC_Init(&hadc1);

  HAL_ADC_Start(&hadc1);
  HAL_ADC_PollForConversion(&hadc1, 100);
  volatile uint16_t v = HAL_ADC_GetValue(&hadc1);

  HAL_Delay(400); //a short delay is important to let the SD card settle
  restore_solenoid();

  //some variables for FatFs


  //Open the file system
  initialize_sd();


  char rxChar;
  char cmdBuf[320];
  uint8_t idx = 0;
  setvbuf(stdin, NULL, _IONBF, 0);
  UART2_Print("Papyrus reset\r\n");
  muted |= 1;
  process_command("file run startup.scr");
  muted &= ~1;

  HX711_Init(&hx);

  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */
    /* UART command input (blocking, simple) */
    if (HAL_UART_Receive(&huart2, (uint8_t *)&rxChar, 1, 1) == HAL_OK)
    {
      if (rxChar == '\r' || rxChar == '\n')
      {
        cmdBuf[idx] = 0;
        /*UART2_Print("> ");
        UART2_Print(cmdBuf);
        UART2_Print("\r\n");*/
        process_command(cmdBuf);
        idx = 0;
      }
      else {
        cmdBuf[idx++] = rxChar;
        //HAL_UART_Transmit(&huart2, (uint8_t *)&rxChar, 1, 10);
        //UART2_Print("\r\n");
      }
    }
    TimeEntry *t = root;
    TimeEntry **last = &root;
    while (t != NULL) {
      if (t->timeout <= 0) {
        if (t->quiet) muted |= 1;
        process_command(t->cmdbuf);
        muted &= ~1;
        if (t->reload) {
          t->timeout = t->reload;
        } else {
          *last = t->next;
          free(t);
          t = *last;
        }
      } else {
        last = &(t->next);
        t = t->next;
      }
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
  HAL_PWREx_ControlVoltageScaling(PWR_REGULATOR_VOLTAGE_SCALE1);

  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.HSIDiv = RCC_HSI_DIV1;
  RCC_OscInitStruct.HSICalibrationValue = RCC_HSICALIBRATION_DEFAULT;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSI;
  RCC_OscInitStruct.PLL.PLLM = RCC_PLLM_DIV1;
  RCC_OscInitStruct.PLL.PLLN = 8;
  RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV2;
  RCC_OscInitStruct.PLL.PLLQ = RCC_PLLQ_DIV2;
  RCC_OscInitStruct.PLL.PLLR = RCC_PLLR_DIV2;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }

  /** Initializes the CPU, AHB and APB buses clocks
  */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV4;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_2) != HAL_OK)
  {
    Error_Handler();
  }
}

/**
  * @brief ADC1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_ADC1_Init(void)
{

  /* USER CODE BEGIN ADC1_Init 0 */

  /* USER CODE END ADC1_Init 0 */

  ADC_ChannelConfTypeDef sConfig = {0};

  /* USER CODE BEGIN ADC1_Init 1 */

  /* USER CODE END ADC1_Init 1 */

  /** Configure the global features of the ADC (Clock, Resolution, Data Alignment and number of conversion)
  */
  hadc1.Instance = ADC1;
  hadc1.Init.ClockPrescaler = ADC_CLOCK_SYNC_PCLK_DIV4;
  hadc1.Init.Resolution = ADC_RESOLUTION_12B;
  hadc1.Init.DataAlign = ADC_DATAALIGN_RIGHT;
  hadc1.Init.ScanConvMode = ADC_SCAN_SEQ_FIXED;
  hadc1.Init.EOCSelection = ADC_EOC_SEQ_CONV;
  hadc1.Init.LowPowerAutoWait = DISABLE;
  hadc1.Init.LowPowerAutoPowerOff = DISABLE;
  hadc1.Init.ContinuousConvMode = DISABLE;
  hadc1.Init.NbrOfConversion = 1;
  hadc1.Init.DiscontinuousConvMode = DISABLE;
  hadc1.Init.ExternalTrigConv = ADC_EXTERNALTRIG_T2_TRGO;
  hadc1.Init.ExternalTrigConvEdge = ADC_EXTERNALTRIGCONVEDGE_RISING;
  hadc1.Init.DMAContinuousRequests = ENABLE;
  hadc1.Init.Overrun = ADC_OVR_DATA_OVERWRITTEN;
  hadc1.Init.SamplingTimeCommon1 = ADC_SAMPLETIME_160CYCLES_5;
  hadc1.Init.OversamplingMode = DISABLE;
  hadc1.Init.TriggerFrequencyMode = ADC_TRIGGER_FREQ_HIGH;
  if (HAL_ADC_Init(&hadc1) != HAL_OK)
  {
    Error_Handler();
  }

  /** Configure Regular Channel
  */
  sConfig.Channel = ADC_CHANNEL_VREFINT;
  sConfig.Rank = ADC_RANK_CHANNEL_NUMBER;
  if (HAL_ADC_ConfigChannel(&hadc1, &sConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN ADC1_Init 2 */
  /* USER CODE END ADC1_Init 2 */

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
  hfdcan1.Init.AutoRetransmission = DISABLE;
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
  hfdcan1.Init.StdFiltersNbr = 0;
  hfdcan1.Init.ExtFiltersNbr = 0;
  hfdcan1.Init.TxFifoQueueMode = FDCAN_TX_FIFO_OPERATION;
  if (HAL_FDCAN_Init(&hfdcan1) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN FDCAN1_Init 2 */

  // FDCAN_FilterTypeDef filter = {0};
  // filter.IdType = FDCAN_STANDARD_ID;
  // filter.FilterIndex = 0;
  // filter.FilterType = FDCAN_FILTER_MASK;
  // filter.FilterConfig = FDCAN_FILTER_TO_RXFIFO0;
  // filter.FilterID1 = CAN_CMD_ID << 18;
  // filter.FilterID2 = 0x7FF << 18;
  //
  // HAL_FDCAN_ConfigFilter(&hfdcan1, &filter);
  /* USER CODE END FDCAN1_Init 2 */

}

/**
  * @brief I2C1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_I2C1_Init(void)
{

  /* USER CODE BEGIN I2C1_Init 0 */

  /* USER CODE END I2C1_Init 0 */

  /* USER CODE BEGIN I2C1_Init 1 */

  /* USER CODE END I2C1_Init 1 */
  hi2c1.Instance = I2C1;
  hi2c1.Init.Timing = 0x00503D58;
  hi2c1.Init.OwnAddress1 = 0;
  hi2c1.Init.AddressingMode = I2C_ADDRESSINGMODE_7BIT;
  hi2c1.Init.DualAddressMode = I2C_DUALADDRESS_DISABLE;
  hi2c1.Init.OwnAddress2 = 0;
  hi2c1.Init.OwnAddress2Masks = I2C_OA2_NOMASK;
  hi2c1.Init.GeneralCallMode = I2C_GENERALCALL_DISABLE;
  hi2c1.Init.NoStretchMode = I2C_NOSTRETCH_DISABLE;
  if (HAL_I2C_Init(&hi2c1) != HAL_OK)
  {
    Error_Handler();
  }

  /** Configure Analogue filter
  */
  if (HAL_I2CEx_ConfigAnalogFilter(&hi2c1, I2C_ANALOGFILTER_ENABLE) != HAL_OK)
  {
    Error_Handler();
  }

  /** Configure Digital filter
  */
  if (HAL_I2CEx_ConfigDigitalFilter(&hi2c1, 0) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN I2C1_Init 2 */

  /* USER CODE END I2C1_Init 2 */

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
  hspi1.Init.DataSize = SPI_DATASIZE_4BIT;
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
  hspi2.Init.DataSize = SPI_DATASIZE_8BIT;
  hspi2.Init.CLKPolarity = SPI_POLARITY_LOW;
  hspi2.Init.CLKPhase = SPI_PHASE_1EDGE;
  hspi2.Init.NSS = SPI_NSS_SOFT;
  hspi2.Init.BaudRatePrescaler = SPI_BAUDRATEPRESCALER_128;
  hspi2.Init.FirstBit = SPI_FIRSTBIT_MSB;
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
  * @brief TIM2 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM2_Init(void)
{

  /* USER CODE BEGIN TIM2_Init 0 */

  /* USER CODE END TIM2_Init 0 */

  TIM_SlaveConfigTypeDef sSlaveConfig = {0};
  TIM_MasterConfigTypeDef sMasterConfig = {0};

  /* USER CODE BEGIN TIM2_Init 1 */

  /* USER CODE END TIM2_Init 1 */
  htim2.Instance = TIM2;
  htim2.Init.Prescaler = 25;
  htim2.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim2.Init.Period = 49999;
  htim2.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim2.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_ENABLE;
  if (HAL_TIM_Base_Init(&htim2) != HAL_OK)
  {
    Error_Handler();
  }
  sSlaveConfig.SlaveMode = TIM_SLAVEMODE_DISABLE;
  sSlaveConfig.InputTrigger = TIM_TS_ITR0;
  if (HAL_TIM_SlaveConfigSynchro(&htim2, &sSlaveConfig) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_UPDATE;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim2, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM2_Init 2 */

  /* USER CODE END TIM2_Init 2 */

}

/**
  * @brief USART1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_USART1_UART_Init(void)
{

  /* USER CODE BEGIN USART1_Init 0 */

  /* USER CODE END USART1_Init 0 */

  /* USER CODE BEGIN USART1_Init 1 */

  /* USER CODE END USART1_Init 1 */
  huart1.Instance = USART1;
  huart1.Init.BaudRate = 115200;
  huart1.Init.WordLength = UART_WORDLENGTH_8B;
  huart1.Init.StopBits = UART_STOPBITS_1;
  huart1.Init.Parity = UART_PARITY_NONE;
  huart1.Init.Mode = UART_MODE_TX_RX;
  huart1.Init.HwFlowCtl = UART_HWCONTROL_NONE;
  huart1.Init.OverSampling = UART_OVERSAMPLING_16;
  huart1.Init.OneBitSampling = UART_ONE_BIT_SAMPLE_DISABLE;
  huart1.Init.ClockPrescaler = UART_PRESCALER_DIV1;
  huart1.AdvancedInit.AdvFeatureInit = UART_ADVFEATURE_NO_INIT;
  if (HAL_UART_Init(&huart1) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_UARTEx_SetTxFifoThreshold(&huart1, UART_TXFIFO_THRESHOLD_1_8) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_UARTEx_SetRxFifoThreshold(&huart1, UART_RXFIFO_THRESHOLD_1_8) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_UARTEx_DisableFifoMode(&huart1) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN USART1_Init 2 */

  /* USER CODE END USART1_Init 2 */

}

/**
  * @brief USART2 Initialization Function
  * @param None
  * @retval None
  */
static void MX_USART2_UART_Init(void)
{

  /* USER CODE BEGIN USART2_Init 0 */

  /* USER CODE END USART2_Init 0 */

  /* USER CODE BEGIN USART2_Init 1 */

  /* USER CODE END USART2_Init 1 */
  huart2.Instance = USART2;
  huart2.Init.BaudRate = 115200;
  huart2.Init.WordLength = UART_WORDLENGTH_8B;
  huart2.Init.StopBits = UART_STOPBITS_1;
  huart2.Init.Parity = UART_PARITY_NONE;
  huart2.Init.Mode = UART_MODE_TX_RX;
  huart2.Init.HwFlowCtl = UART_HWCONTROL_NONE;
  huart2.Init.OverSampling = UART_OVERSAMPLING_16;
  huart2.Init.OneBitSampling = UART_ONE_BIT_SAMPLE_DISABLE;
  huart2.Init.ClockPrescaler = UART_PRESCALER_DIV1;
  huart2.AdvancedInit.AdvFeatureInit = UART_ADVFEATURE_RXOVERRUNDISABLE_INIT;
  huart2.AdvancedInit.OverrunDisable = UART_ADVFEATURE_OVERRUN_DISABLE;
  if (HAL_UART_Init(&huart2) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_UARTEx_SetTxFifoThreshold(&huart2, UART_TXFIFO_THRESHOLD_1_8) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_UARTEx_SetRxFifoThreshold(&huart2, UART_RXFIFO_THRESHOLD_1_8) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_UARTEx_EnableFifoMode(&huart2) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN USART2_Init 2 */

  /* USER CODE END USART2_Init 2 */

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
  __HAL_RCC_GPIOD_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOB, GPIO_PIN_0|GPIO_PIN_1, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOB, GPIO_PIN_2, GPIO_PIN_SET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOD, GPIO_PIN_0|GPIO_PIN_2|GPIO_PIN_3, GPIO_PIN_RESET);

  /*Configure GPIO pin : PB0 */
  GPIO_InitStruct.Pin = GPIO_PIN_0;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

  /*Configure GPIO pins : PB1 PB2 */
  GPIO_InitStruct.Pin = GPIO_PIN_1|GPIO_PIN_2;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_PULLUP;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

  /*Configure GPIO pins : PD0 PD2 PD3 */
  GPIO_InitStruct.Pin = GPIO_PIN_0|GPIO_PIN_2|GPIO_PIN_3;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOD, &GPIO_InitStruct);

  /*Configure GPIO pin : PD1 */
  GPIO_InitStruct.Pin = GPIO_PIN_1;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(GPIOD, &GPIO_InitStruct);

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
