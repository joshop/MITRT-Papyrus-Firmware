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
/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
ADC_HandleTypeDef hadc1;

FDCAN_HandleTypeDef hfdcan1;

I2C_HandleTypeDef hi2c1;

UART_HandleTypeDef hlpuart2;
UART_HandleTypeDef huart1;
UART_HandleTypeDef huart2;

SPI_HandleTypeDef hspi1;
SPI_HandleTypeDef hspi2;

/* USER CODE BEGIN PV */

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_ADC1_Init(void);
static void MX_FDCAN1_Init(void);
static void MX_I2C1_Init(void);
static void MX_SPI1_Init(void);
static void MX_SPI2_Init(void);
static void MX_USART1_UART_Init(void);
static void MX_USART2_UART_Init(void);
static void MX_LPUART2_UART_Init(void);
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

void CAN_Rx_Default(FDCAN_RxHeaderTypeDef rxHeader) {

  UART2_Print("OK\r\n");
}

void CAN_Rx_TC(FDCAN_RxHeaderTypeDef rxHeader) {
  char buff[64];
  int16_t tc_t = *(int16_t*)(lastRxData+1);
  float tc_tf = ((float)tc_t) / 4.0;
  if (lastRxData[1] == 0xFF && lastRxData[2] == 0xFF) {
    sprintf(buff, "TC Fault\r\n");
  } else {
    sprintf(buff, "T = %.2f C\r\n", tc_tf);
  }
  UART2_Print(buff);
}
void CAN_Rx_PT(FDCAN_RxHeaderTypeDef rxHeader) {
  char buff[64];
  int16_t pt_p = *(int16_t*)(lastRxData+1);
  float pt_v = 3000.0f * ((float)pt_p - 500.0f) / 4000.0f;
  sprintf(buff, "P = %.2f psi\r\n", pt_v);
  UART2_Print(buff);
}
void CAN_Rx_TCStat(FDCAN_RxHeaderTypeDef rxHeader) {
  char buff[64];
  uint8_t tc_f = lastRxData[1];
  int16_t tc_t = *(int16_t*)(lastRxData+2);
  float tc_tf = ((float)tc_t) / 16.0;
  sprintf(buff, "CJC = %.2f C; ", tc_tf);
  UART2_Print(buff);
  if (tc_f == 0) {
    UART2_Print("no fault\r\n");
  } else {
    if (tc_f & 1) UART2_Print("open ");
    if (tc_f & 2) UART2_Print("GND-short ");
    if (tc_f & 4) UART2_Print("VCC-short ");
    if (tc_f & 8) UART2_Print("fault ");
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



void CAN_Send(uint8_t cmd)
{
  FDCAN_TxHeaderTypeDef txHeader = {0};
  FDCAN_RxHeaderTypeDef rxHeader;
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

int muted = 0;
void UART_Print(char *s)
{
  if (muted) return;
  HAL_UART_Transmit(&hlpuart2, (uint8_t *)s, strlen(s), HAL_MAX_DELAY);
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
  FDCAN_RxHeaderTypeDef rxHeader;
  uint8_t txData[8] = {0x00};
  uint8_t rxData[8];

  txHeader.Identifier = 0;
  txHeader.IdType = FDCAN_STANDARD_ID;
  txHeader.TxFrameType = FDCAN_DATA_FRAME;
  txHeader.DataLength = FDCAN_DLC_BYTES_1;
  txHeader.FDFormat = FDCAN_CLASSIC_CAN;
  txHeader.BitRateSwitch = FDCAN_BRS_OFF;
  UART2_Print("Devices on bus:\r\n");
  HAL_FDCAN_AddMessageToTxFifoQ(&hfdcan1, &txHeader, txData);
  uint32_t tick = HAL_GetTick();
  while (HAL_GetTick() - tick < 500) {
    if (HAL_FDCAN_GetRxFifoFillLevel(&hfdcan1, FDCAN_RX_FIFO0) != 0) {
      HAL_FDCAN_GetRxMessage(&hfdcan1, FDCAN_RX_FIFO0, &rxHeader, rxData);
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
      if (rxData[1] > 4) {
        UART2_Print("Unknown");
      } else {
        UART2_Print(dev_classes[rxData[1]]);
      }
      UART2_Print("\r\n");
    }
  }
  UART2_Print("(end of list)\r\n");

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
  UART2_Print("Todo\r\n");
}
void command_bus(char *rest) {
  char word[32];
  next_word(word, &rest);
  if (!strcmp(word, "reset")) {
    UART2_Print("Full bus reset...\r\n");
    NVIC_SystemReset();
  } else if (!strcmp(word, "list")) {
    bus_list_function();
  } else if (!*word) {
    cmd_invalid = 1;
    UART2_Print("Option needed for \"bus\".\r\n");
  } else {
    cmd_invalid = 1;
    UART2_Print("Unknown option for \"bus\": ");
    UART2_Print(word);
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
      UART2_Print("Format: name list <device>\r\n");
      return;
    }
    int yes = 0;
    for (int i = 0; i < num_maps; i++) {
      if (name_maps[i].can_id == id) {
        yes = 1;
        UART2_Print("Names for device ");
        UART2_Print_Int(id);
        UART2_Print(": ");
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
    if (!yes) {
      UART2_Print("No names for that device.\r\n");
    }
  } else {
    int id = retrieve_id(word);
    if (id == 0) {
      UART2_Print("Option or device needed for \"name\".\r\n");
      return;
    }
    if (!*rest) {
      cmd_invalid = 1;
      UART2_Print("Format: name <device> <name> [<index>]\r\n");
    }
    next_word(word, &rest);
    int index = -1;
    if (*rest) {
      if (all_numeric(rest)) {
        index = atoi(rest);
      } else {
        cmd_invalid = 1;
        UART2_Print("Format: name <device> <name> [<index>]\r\n");
        return;
      }
    }
    int result = add_namemap(id, word, index);
    if (result == 1) {
      UART2_Print("Too many devices in map.\r\n");
    } else if (result == 2) {
      UART2_Print("Name already registered.\r\n");
    } else if (result == 3) {
      UART2_Print("Too many names on device.\r\n");
    } else if (result == 4) {
      UART2_Print("Name too long.\r\n");
    } else {
      UART2_Print("Name registered.\r\n");
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
      UART2_Print("Invalid ID or device name.\r\n");
      return;
    }
    next_can_id = id;
    int cmd;
    UART2_Print(rest);
    UART2_Print(": ");
    if (last_index == 1) {
      UART2_Print("Turning relay OFF.\r\n");
      CAN_Send(0x02);
    } else if (last_index == 2) {
      UART2_Print("Turning relay OFF.\r\n");
      CAN_Send(0x04);
    } else if (last_index == 3) {
      UART2_Print("Turning relay OFF.\r\n");
      CAN_Send(0x06);
    } else {
      UART2_Print("Turning relays OFF.\r\n");
      CAN_Send(0x02);
      CAN_Send(0x04);
      CAN_Send(0x06);
    }
  } else if (!strcmp(word, "on")) {
    int id = retrieve_id(rest);
    if (id == 0) {
      UART2_Print("Invalid ID or device name.\r\n");
      return;
    }
    next_can_id = id;
    int cmd;
    UART2_Print(rest);
    UART2_Print(": ");
    if (last_index == 1) {
      UART2_Print("Turning relay ON.\r\n");
      CAN_Send(0x01);
    } else if (last_index == 2) {
      UART2_Print("Turning relay ON.\r\n");
      CAN_Send(0x03);
    } else if (last_index == 3) {
      UART2_Print("Turning relay ON.\r\n");
      CAN_Send(0x05);
    } else {
      UART2_Print("Need to specify individual relay.\r\n");
    }
  } else if (!strcmp(word, "reset")) {
    int id = retrieve_id(rest);
    if (id == 0) {
      UART2_Print("Invalid ID or device name.\r\n");
      return;
    }
    next_can_id = id;
    UART2_Print(rest);
    UART2_Print(": Reset board.\r\n");
    CAN_Send(0xFF);
  } else if (!*word) {
    cmd_invalid = 1;
    UART2_Print("Option needed for \"relay\".\r\n");
  } else {
    cmd_invalid = 1;
    UART2_Print("Unknown option for \"relay\": ");
    UART2_Print(word);
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
    servo->min_us = 1000;
    servo->max_us = 2000;
    servo->min_angle = 0;
    servo->max_angle = 180;
  }
  if (angle < servo->min_angle) return 0;
  if (angle > servo->max_angle) return 0;
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
      UART2_Print("Invalid ID or device name.\r\n");
      return;
    }
    next_can_id = id;
    int cmd;
    UART2_Print(rest);
    UART2_Print(": ");
    if (last_index == 1) {
      UART2_Print("Turning servo ON.\r\n");
      CAN_Send(0x01);
    } else if (last_index == 2) {
      UART2_Print("Turning servo ON.\r\n");
      CAN_Send(0x02);
    } else if (last_index == 3) {
      UART2_Print("Turning servo ON.\r\n");
      CAN_Send(0x03);
    } else if (last_index == 4) {
      UART2_Print("Turning servo ON.\r\n");
      CAN_Send(0x04);
    } else {
      UART2_Print("Need to specify individual servo.\r\n");
    }
  } else if (!strcmp(word, "off")) {
    int id = retrieve_id(rest);
    if (id == 0) {
      UART2_Print("Invalid ID or device name.\r\n");
      return;
    }
    next_can_id = id;
    int cmd;
    UART2_Print(rest);
    UART2_Print(": ");
    if (last_index == 1) {
      UART2_Print("Turning servo OFF.\r\n");
      CAN_Send(0x05);
    } else if (last_index == 2) {
      UART2_Print("Turning servo OFF.\r\n");
      CAN_Send(0x06);
    } else if (last_index == 3) {
      UART2_Print("Turning servo OFF.\r\n");
      CAN_Send(0x07);
    } else if (last_index == 4) {
      UART2_Print("Turning servo OFF.\r\n");
      CAN_Send(0x08);
    } else {
      UART2_Print("Need to specify individual servo.\r\n");
    }
  } else if (!strcmp(word, "set")) {
    next_word(word, &rest);
    int id = retrieve_id(word);
    if (id == 0) {
      UART2_Print("Invalid ID or device name.\r\n");
      return;
    }
    uint16_t angle = atoi(rest);
    if (angle == 0 && *rest != '0') {
      UART2_Print("Invalid angle.\r\n");
      return;
    }
    uint16_t serv_us = servo_get_us(word, angle);
    if (serv_us == 0) {
      UART2_Print("Angle out of range.\r\n");
      return;
    }
    char buf[3];
    if (last_index < 1 || last_index > 4) {
      UART2_Print("Need to specify individual servo.\r\n");
      return;
    }
    UART2_Print(rest);
    UART2_Print(": Setting servo -> ");
    UART2_Print_Int(angle);
    UART2_Print(" deg (");
    UART2_Print_Int(serv_us);
    UART2_Print(" us).\r\n");
    buf[0] = 0x10 + last_index;
    *(uint16_t*)&buf[1] = serv_us;
    CAN_Send_Long(buf, 3);
  } else if (!strcmp(word, "reset")) {
    int id = retrieve_id(rest);
    if (id == 0) {
      UART2_Print("Invalid ID or device name.\r\n");
      return;
    }
    next_can_id = id;
    UART2_Print(rest);
    UART2_Print(": Reset board.\r\n");
    CAN_Send(0xFF);
  } else if (!*word) {
    cmd_invalid = 1;
    UART2_Print("Option needed for \"servo\".\r\n");
  } else {
    cmd_invalid = 1;
    UART2_Print("Unknown option for \"servo\": ");
    UART2_Print(word);
    UART2_Print("\r\n");
  }
}
void command_tc(char *rest) {
  char word[32];
  next_word(word, &rest);
  if (!strcmp(word, "get")) {
    int id = retrieve_id(rest);
    if (id == 0) {
      UART2_Print("Invalid ID or device name.\r\n");
      return;
    }
    if (last_index < 1 || last_index > 3) {
      UART2_Print("Need to specify individual TC.\r\n");
      return;
    }
    next_can_id = id;
    UART2_Print(rest);
    UART2_Print(": ");
    CAN_Rx_Func = CAN_Rx_TC;
    CAN_Send(last_index);
  } else if (!strcmp(word, "status")) {
    int id = retrieve_id(rest);
    if (id == 0) {
      UART2_Print("Invalid ID or device name.\r\n");
      return;
    }
    if (last_index < 1 || last_index > 3) {
      UART2_Print("Need to specify individual TC.\r\n");
      return;
    }
    next_can_id = id;
    UART2_Print(rest);
    UART2_Print(": ");
    CAN_Rx_Func = CAN_Rx_TCStat;
    CAN_Send(0x03 + last_index);
  } else if (!strcmp(word, "reset")) {
    int id = retrieve_id(rest);
    if (id == 0) {
      UART2_Print("Invalid ID or device name.\r\n");
      return;
    }
    next_can_id = id;
    UART2_Print(rest);
    UART2_Print(": Reset board.\r\n");
    CAN_Send(0xFF);
  } else if (!*word) {
    cmd_invalid = 1;
    UART2_Print("Option needed for \"tc\".\r\n");
  } else {
    cmd_invalid = 1;
    UART2_Print("Unknown option for \"tc\": ");
    UART2_Print(word);
    UART2_Print("\r\n");
  }

}
void command_pt(char *rest) {
  char word[32];
  next_word(word, &rest);
  if (!strcmp(word, "get")) {
    int id = retrieve_id(rest);
    if (id == 0) {
      UART2_Print("Invalid ID or device name.\r\n");
      return;
    }
    if (last_index < 1 || last_index > 3) {
      UART2_Print("Need to specify individual PT.\r\n");
      return;
    }
    next_can_id = id;
    UART2_Print(rest);
    UART2_Print(": ");
    CAN_Rx_Func = CAN_Rx_PT;
    CAN_Send(last_index);
  } else if (!strcmp(word, "reset")) {
    int id = retrieve_id(rest);
    if (id == 0) {
      UART2_Print("Invalid ID or device name.\r\n");
      return;
    }
    next_can_id = id;
    UART2_Print(rest);
    UART2_Print(": Reset board.\r\n");
    CAN_Send(0xFF);
  } else if (!*word) {
    cmd_invalid = 1;
    UART2_Print("Option needed for \"pt\".\r\n");
  } else {
    cmd_invalid = 1;
    UART2_Print("Unknown option for \"pt\": ");
    UART2_Print(word);
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
    else UART2_Print("SD Card not initialized.\r\n");
  } else if (!*word) {
    cmd_invalid = 1;
    UART2_Print("Option needed for \"sd\".\r\n");
  } else {
    cmd_invalid = 1;
    UART2_Print("Unknown option for \"sd\": ");
    UART2_Print(word);
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
    UART2_Print("Option needed for \"file\".\r\n");
  } else {
    cmd_invalid = 1;
    UART2_Print("Unknown option for \"file\": ");
    UART2_Print(word);
    UART2_Print("\r\n");
  }
}
void command_msg(char *rest) {
  char word[32];
  next_word(word, &rest);
  if (!strcmp(word, "echo")) {
    UART2_Print(rest);
    UART2_Print("\r\n");
  } else if (!*word) {
    cmd_invalid = 1;
    UART2_Print("Option needed for \"msg\".\r\n");
  } else {
    cmd_invalid = 1;
    UART2_Print("Unknown option for \"msg\": ");
    UART2_Print(word);
    UART2_Print("\r\n");
  }
}
struct TimeEntry {
  char cmdbuf[320];
  int reload;
  int timeout;
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
    sprintf(buff, "Timestamp %dms", HAL_GetTick());
    UART2_Print(buff);
    if (time_mark) {
      sprintf(buff, ", %dms since mark\r\n", HAL_GetTick() - time_mark);
      UART2_Print(buff);
    } else {
      UART2_Print("\r\n");
    }
  } else if (!strcmp(word, "mark")) {
    char buff[64];
    time_mark = HAL_GetTick();
    sprintf(buff, "Mark set at %dms\r\n", HAL_GetTick());
    UART2_Print(buff);
  } else if (!strcmp(word, "sleep")) {
    int ms = atoi(rest);
    if (ms == 0) {
      UART2_Print("Format: time sleep <ms>\r\n");
      return;
    }
    HAL_Delay(ms);
  } else if (!strcmp(word, "delay")) {
    next_word(word, &rest);
    int ms = atoi(word);
    if (ms == 0 || !*rest) {
      UART2_Print("Format: time delay <ms> <command...>\r\n");
      return;
    }
    add_time_entry(rest, 0, ms);
    UART2_Print("Running command in ");
    UART2_Print_Int(ms);
    UART2_Print("ms.\r\n");
  } else if (!strcmp(word, "every")) {
    next_word(word, &rest);
    int ms = atoi(word);
    if (ms == 0 || !*rest) {
      UART2_Print("Format: time every <ms> <command...>\r\n");
      return;
    }
    add_time_entry(rest, ms, ms);
    UART2_Print("Running command every ");
    UART2_Print_Int(ms);
    UART2_Print("ms.\r\n");
  } else if (!strcmp(word, "list")) {
    if (root == NULL) {
      UART2_Print("No timers active.\r\n");
      return;
    }
    TimeEntry *t = root;
    UART2_Print("Timers active:\r\n");
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
      UART2_Print("Format: time delete <index>\r\n");
      return;
    }
    if (root == NULL) {
      UART2_Print("No timers active.\r\n");
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
      UART2_Print("No timer with that index.\r\n");
    } else {
      UART2_Print("Timer deleted.\r\n");
    }
  } else if (!*word) {
    cmd_invalid = 1;
    UART2_Print("Option needed for \"time\".\r\n");
  } else {
    cmd_invalid = 1;
    UART2_Print("Unknown option for \"time\": ");
    UART2_Print(word);
    UART2_Print("\r\n");
  }
}
int prog_lines = 0;
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
      sprintf(buffer, "Programmed %d lines.\r\n", prog_lines);
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
        UART2_Print("Warning, write failed.\r\n");
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
  } else if (!*word) {
    cmd_invalid = 1;
    UART2_Print("Enter a command.\r\n");
  } else {
    cmd_invalid = 1;
    UART2_Print("Unknown command: ");
    UART2_Print(word);
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
  MX_ADC1_Init();
  MX_FDCAN1_Init();
  MX_I2C1_Init();
  MX_SPI1_Init();
  MX_SPI2_Init();
  MX_USART1_UART_Init();
  MX_USART2_UART_Init();
  //MX_LPUART2_UART_Init();
  if (MX_FATFS_Init() != APP_OK) {
    Error_Handler();
  }
  /* USER CODE BEGIN 2 */

  HAL_FDCAN_Start(&hfdcan1);

  HAL_Delay(400); //a short delay is important to let the SD card settle

  //some variables for FatFs


  //Open the file system
  initialize_sd();


  char rxChar;
  char cmdBuf[320];
  uint8_t idx = 0;
  UART2_Print("Papyrus reset\r\n");
  muted = 1;
  process_command("file run startup.scr");
  muted = 0;

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
        process_command(t->cmdbuf);
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
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_NONE;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }

  /** Initializes the CPU, AHB and APB buses clocks
  */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_HSI;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV1;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_0) != HAL_OK)
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
  hadc1.Init.ClockPrescaler = ADC_CLOCK_SYNC_PCLK_DIV2;
  hadc1.Init.Resolution = ADC_RESOLUTION_12B;
  hadc1.Init.DataAlign = ADC_DATAALIGN_RIGHT;
  hadc1.Init.ScanConvMode = ADC_SCAN_DISABLE;
  hadc1.Init.EOCSelection = ADC_EOC_SINGLE_CONV;
  hadc1.Init.LowPowerAutoWait = DISABLE;
  hadc1.Init.LowPowerAutoPowerOff = DISABLE;
  hadc1.Init.ContinuousConvMode = DISABLE;
  hadc1.Init.NbrOfConversion = 1;
  hadc1.Init.DiscontinuousConvMode = DISABLE;
  hadc1.Init.ExternalTrigConv = ADC_SOFTWARE_START;
  hadc1.Init.ExternalTrigConvEdge = ADC_EXTERNALTRIGCONVEDGE_NONE;
  hadc1.Init.DMAContinuousRequests = DISABLE;
  hadc1.Init.Overrun = ADC_OVR_DATA_PRESERVED;
  hadc1.Init.SamplingTimeCommon1 = ADC_SAMPLETIME_1CYCLE_5;
  hadc1.Init.SamplingTimeCommon2 = ADC_SAMPLETIME_1CYCLE_5;
  hadc1.Init.OversamplingMode = DISABLE;
  hadc1.Init.TriggerFrequencyMode = ADC_TRIGGER_FREQ_HIGH;
  if (HAL_ADC_Init(&hadc1) != HAL_OK)
  {
    Error_Handler();
  }

  /** Configure Regular Channel
  */
  sConfig.Channel = ADC_CHANNEL_0;
  sConfig.Rank = ADC_REGULAR_RANK_1;
  sConfig.SamplingTime = ADC_SAMPLINGTIME_COMMON_1;
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
  * @brief LPUART2 Initialization Function
  * @param None
  * @retval None
  */
static void MX_LPUART2_UART_Init(void)
{

  /* USER CODE BEGIN LPUART2_Init 0 */

  /* USER CODE END LPUART2_Init 0 */

  /* USER CODE BEGIN LPUART2_Init 1 */

  /* USER CODE END LPUART2_Init 1 */
  hlpuart2.Instance = LPUART2;
  hlpuart2.Init.BaudRate = 115200;
  hlpuart2.Init.WordLength = UART_WORDLENGTH_8B;
  hlpuart2.Init.StopBits = UART_STOPBITS_1;
  hlpuart2.Init.Parity = UART_PARITY_NONE;
  hlpuart2.Init.Mode = UART_MODE_TX_RX;
  hlpuart2.Init.HwFlowCtl = UART_HWCONTROL_NONE;
  hlpuart2.Init.OneBitSampling = UART_ONE_BIT_SAMPLE_DISABLE;
  hlpuart2.Init.ClockPrescaler = UART_PRESCALER_DIV1;
  hlpuart2.AdvancedInit.AdvFeatureInit = UART_ADVFEATURE_NO_INIT;
  hlpuart2.FifoMode = UART_FIFOMODE_DISABLE;
  if (HAL_UART_Init(&hlpuart2) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_UARTEx_SetTxFifoThreshold(&hlpuart2, UART_TXFIFO_THRESHOLD_1_8) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_UARTEx_SetRxFifoThreshold(&hlpuart2, UART_RXFIFO_THRESHOLD_1_8) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_UARTEx_DisableFifoMode(&hlpuart2) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN LPUART2_Init 2 */

  /* USER CODE END LPUART2_Init 2 */

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
  huart2.AdvancedInit.AdvFeatureInit = UART_ADVFEATURE_NO_INIT;
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
  if (HAL_UARTEx_DisableFifoMode(&huart2) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN USART2_Init 2 */

  /* USER CODE END USART2_Init 2 */

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
  HAL_GPIO_WritePin(GPIOD, GPIO_PIN_0|GPIO_PIN_1|GPIO_PIN_2|GPIO_PIN_3, GPIO_PIN_RESET);

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

  /*Configure GPIO pins : PD0 PD1 PD2 PD3 */
  GPIO_InitStruct.Pin = GPIO_PIN_0|GPIO_PIN_1|GPIO_PIN_2|GPIO_PIN_3;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
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
