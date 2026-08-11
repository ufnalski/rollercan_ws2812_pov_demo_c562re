/*
 * rollercan.c
 *
 *  Created on: Mar 15, 2026
 *      Author: ufnalski
 */

#include "rollercan.h"
#include "main.h"
#include "stm32c5xx_hal.h"

hal_fdcan_handle_t *hfdcan1;
hal_fdcan_tx_header_t tx_header;
uint8_t txData[8];
hal_fdcan_rx_header_t rx_header;
uint8_t rxData[8];
uint8_t rxDataBis[3][8];
hal_fdcan_global_filter_config_t global_filter_config;
hal_fdcan_filter_t filter_config;

uint8_t position_received_flag = 0;
uint8_t speed_received_flag = 0;
uint8_t current_received_flag = 0;
uint8_t temperature_received_flag = 0;

union {
  int16_t int16;
  uint8_t uint8[2];
} int16_to_uint8_converter;

union {
  int32_t int32;
  uint8_t uint8[4];
} int32_to_uint8_converter;

union {
  uint32_t uint32;
  uint8_t uint8[4];
} uint32_to_uint8_converter;

void rollercan_can_config(void) {
  hfdcan1 = mx_fdcan1_gethandle();
  global_filter_config.acceptance_non_matching_ext = HAL_FDCAN_NO_MATCH_REJECT;
  global_filter_config.acceptance_non_matching_std = HAL_FDCAN_NO_MATCH_REJECT;
  global_filter_config.acceptance_remote_ext = HAL_FDCAN_REMOTE_REJECT;
  global_filter_config.acceptance_remote_std = HAL_FDCAN_REMOTE_REJECT;
  if (HAL_FDCAN_SetGlobalFilter(hfdcan1, &global_filter_config) != HAL_OK) {
    FDCAN_Error_Handler();
  };

  filter_config.id_type = HAL_FDCAN_ID_EXTENDED;
  filter_config.filter_index = 0;
  filter_config.filter_type = HAL_FDCAN_FILTER_TYPE_CLASSIC;
  filter_config.filter_config = HAL_FDCAN_FILTER_TO_RX_FIFO_0;
  filter_config.filter_id1 =
      (COMMAND_GET_PARAMETER << 24) |
      ((HOST_CAN_ID & 0xFF) << 8);       // response to get parameter command
  filter_config.filter_id2 = 0x1FFFFF00; // mask
  if (HAL_FDCAN_SetFilter(hfdcan1, &filter_config) != HAL_OK) {
    FDCAN_Error_Handler();
  };

  if (HAL_FDCAN_Start(hfdcan1) != HAL_OK) {
    FDCAN_Error_Handler();
  };
}

void HAL_FDCAN_RxFifo0Callback(hal_fdcan_handle_t *hfdcan,
                               uint32_t rx_fifo0_interrupts) {
  // https://community.st.com/t5/stm32-mcus/understanding-fdcan-interrupts-grouping-in-applicable-stm32-mcus/ta-p/852823
  if (rx_fifo0_interrupts & HAL_FDCAN_IT_GROUP_RX_FIFO_0) {
    if (HAL_FDCAN_GetReceivedMessage(hfdcan1, HAL_FDCAN_RX_FIFO_0, &rx_header,
                                     rxData) == HAL_OK) {
      if ((TRIPOD_LEG_A_CAN_ID == (rx_header.b.identifier & 0xFF)) &&
          (rxData[0] == 0x31) && (rxData[1] == 0x70)) { // position
        memcpy(&rxDataBis[TRIPOD_LEG_A_IDX][0], rxData, 8);
        position_received_flag = TRIPOD_LEG_A_CAN_ID;
      } else if ((TRIPOD_LEG_B_CAN_ID == (rx_header.b.identifier & 0xFF)) &&
                 (rxData[0] == 0x31) && (rxData[1] == 0x70)) {
        memcpy(&rxDataBis[TRIPOD_LEG_B_IDX][0], rxData, 8);
        position_received_flag = TRIPOD_LEG_B_CAN_ID;
      } else if ((TRIPOD_LEG_C_CAN_ID == (rx_header.b.identifier & 0xFF)) &&
                 (rxData[0] == 0x31) && (rxData[1] == 0x70)) {
        memcpy(&rxDataBis[TRIPOD_LEG_C_IDX][0], rxData, 8);
        position_received_flag = TRIPOD_LEG_C_CAN_ID;
      } else if ((TRIPOD_LEG_A_CAN_ID == (rx_header.b.identifier & 0xFF)) &&
                 (rxData[0] == 0x30) && (rxData[1] == 0x70)) { // speed
        memcpy(&rxDataBis[TRIPOD_LEG_A_IDX][0], rxData, 8);
        speed_received_flag = TRIPOD_LEG_A_CAN_ID;
      } else if ((TRIPOD_LEG_B_CAN_ID == (rx_header.b.identifier & 0xFF)) &&
                 (rxData[0] == 0x30) && (rxData[1] == 0x70)) {
        memcpy(&rxDataBis[TRIPOD_LEG_B_IDX][0], rxData, 8);
        speed_received_flag = TRIPOD_LEG_B_CAN_ID;
      } else if ((TRIPOD_LEG_C_CAN_ID == (rx_header.b.identifier & 0xFF)) &&
                 (rxData[0] == 0x30) && (rxData[1] == 0x70)) {
        memcpy(&rxDataBis[TRIPOD_LEG_C_IDX][0], rxData, 8);
        speed_received_flag = TRIPOD_LEG_C_CAN_ID;
      } else if ((TRIPOD_LEG_A_CAN_ID == (rx_header.b.identifier & 0xFF)) &&
                 (rxData[0] == 0x32) && (rxData[1] == 0x70)) { // current
        memcpy(&rxDataBis[TRIPOD_LEG_A_IDX][0], rxData, 8);
        current_received_flag = TRIPOD_LEG_A_CAN_ID;
      } else if ((TRIPOD_LEG_B_CAN_ID == (rx_header.b.identifier & 0xFF)) &&
                 (rxData[0] == 0x32) && (rxData[1] == 0x70)) {
        memcpy(&rxDataBis[TRIPOD_LEG_B_IDX][0], rxData, 8);
        current_received_flag = TRIPOD_LEG_B_CAN_ID;
      } else if ((TRIPOD_LEG_C_CAN_ID == (rx_header.b.identifier & 0xFF)) &&
                 (rxData[0] == 0x32) && (rxData[1] == 0x70)) {
        memcpy(&rxDataBis[TRIPOD_LEG_C_IDX][0], rxData, 8);
        current_received_flag = TRIPOD_LEG_C_CAN_ID;
      } else if ((TRIPOD_LEG_A_CAN_ID == (rx_header.b.identifier & 0xFF)) &&
                 (rxData[0] == 0x35) && (rxData[1] == 0x70)) { // temperature
        memcpy(&rxDataBis[TRIPOD_LEG_A_IDX][0], rxData, 8);
        temperature_received_flag = TRIPOD_LEG_A_CAN_ID;
      } else if ((TRIPOD_LEG_B_CAN_ID == (rx_header.b.identifier & 0xFF)) &&
                 (rxData[0] == 0x35) && (rxData[1] == 0x70)) {
        memcpy(&rxDataBis[TRIPOD_LEG_B_IDX][0], rxData, 8);
        temperature_received_flag = TRIPOD_LEG_B_CAN_ID;
      } else if ((TRIPOD_LEG_C_CAN_ID == (rx_header.b.identifier & 0xFF)) &&
                 (rxData[0] == 0x35) && (rxData[1] == 0x70)) {
        memcpy(&rxDataBis[TRIPOD_LEG_C_IDX][0], rxData, 8);
        temperature_received_flag = TRIPOD_LEG_C_CAN_ID;
      }
    }
  }
}

void ask_for_id(uint8_t _drive_id) {
  hfdcan1 = mx_fdcan1_gethandle();
  tx_header.b.identifier =
      (COMMAND_OBTAIN_ID << 24) | _drive_id | (HOST_CAN_ID << 8);
  tx_header.b.frame_type = HAL_FDCAN_FRAME_DATA;
  tx_header.b.identifier_type = HAL_FDCAN_ID_EXTENDED;
  tx_header.b.error_state_indicator = HAL_FDCAN_ERROR_STATE_IND_ACTIVE;
  tx_header.b.data_length = HAL_FDCAN_DATA_LEN_CAN_FDCAN_8_BYTE;
  tx_header.b.bit_rate_switch = HAL_FDCAN_BIT_RATE_SWITCH_OFF;
  tx_header.b.frame_format = HAL_FDCAN_HEADER_FRAME_FORMAT_CAN;
  tx_header.b.event_fifo_control = HAL_FDCAN_TX_EVENTS_FIFO_DISCARD;
  tx_header.b.message_marker = 0;
  txData[0] = 0x00;
  txData[1] = 0x00;
  txData[2] = 0x00;
  txData[3] = 0x00;
  txData[4] = 0x00;
  txData[5] = 0x00;
  txData[6] = 0x00;
  txData[7] = 0x00;

  if (HAL_FDCAN_ReqTransmitMsgFromFIFOQ(hfdcan1, &tx_header, txData) !=
      HAL_OK) {
    FDCAN_Error_Handler();
  }
  HAL_Delay(CAN_MSG_SPACING);
}

void enable_operation(uint8_t _drive_id) {
  hfdcan1 = mx_fdcan1_gethandle();
  tx_header.b.identifier =
      (COMMAND_ENABLE_OPERATION << 24) | _drive_id | (HOST_CAN_ID << 8);
  tx_header.b.frame_type = HAL_FDCAN_FRAME_DATA;
  tx_header.b.identifier_type = HAL_FDCAN_ID_EXTENDED;
  tx_header.b.error_state_indicator = HAL_FDCAN_ERROR_STATE_IND_ACTIVE;
  tx_header.b.data_length = HAL_FDCAN_DATA_LEN_CAN_FDCAN_8_BYTE;
  tx_header.b.bit_rate_switch = HAL_FDCAN_BIT_RATE_SWITCH_OFF;
  tx_header.b.frame_format = HAL_FDCAN_HEADER_FRAME_FORMAT_CAN;
  tx_header.b.event_fifo_control = HAL_FDCAN_TX_EVENTS_FIFO_DISCARD;
  tx_header.b.message_marker = 0;
  txData[0] = 0x00;
  txData[1] = 0x00;
  txData[2] = 0x00;
  txData[3] = 0x00;
  txData[4] = 0x00;
  txData[5] = 0x00;
  txData[6] = 0x00;
  txData[7] = 0x00;

  if (HAL_FDCAN_ReqTransmitMsgFromFIFOQ(hfdcan1, &tx_header, txData) !=
      HAL_OK) {
    FDCAN_Error_Handler();
  }
  HAL_Delay(CAN_MSG_SPACING);
}

void stop_operation(uint8_t _drive_id) {
  hfdcan1 = mx_fdcan1_gethandle();
  tx_header.b.identifier =
      (COMMAND_STOP_OPERATION << 24) | _drive_id | (HOST_CAN_ID << 8);
  tx_header.b.frame_type = HAL_FDCAN_FRAME_DATA;
  tx_header.b.identifier_type = HAL_FDCAN_ID_EXTENDED;
  tx_header.b.error_state_indicator = HAL_FDCAN_ERROR_STATE_IND_ACTIVE;
  tx_header.b.data_length = HAL_FDCAN_DATA_LEN_CAN_FDCAN_8_BYTE;
  tx_header.b.bit_rate_switch = HAL_FDCAN_BIT_RATE_SWITCH_OFF;
  tx_header.b.frame_format = HAL_FDCAN_HEADER_FRAME_FORMAT_CAN;
  tx_header.b.event_fifo_control = HAL_FDCAN_TX_EVENTS_FIFO_DISCARD;
  tx_header.b.message_marker = 0;
  txData[0] = 0x00;
  txData[1] = 0x00;
  txData[2] = 0x00;
  txData[3] = 0x00;
  txData[4] = 0x00;
  txData[5] = 0x00;
  txData[6] = 0x00;
  txData[7] = 0x00;

  if (HAL_FDCAN_ReqTransmitMsgFromFIFOQ(hfdcan1, &tx_header, txData) !=
      HAL_OK) {
    FDCAN_Error_Handler();
  }
  HAL_Delay(CAN_MSG_SPACING);
}

void set_speed_mode(uint8_t _drive_id) {
  hfdcan1 = mx_fdcan1_gethandle();
  tx_header.b.identifier =
      (COMMAND_SET_PARAMETER << 24) | _drive_id | (HOST_CAN_ID << 8);
  tx_header.b.frame_type = HAL_FDCAN_FRAME_DATA;
  tx_header.b.identifier_type = HAL_FDCAN_ID_EXTENDED;
  tx_header.b.error_state_indicator = HAL_FDCAN_ERROR_STATE_IND_ACTIVE;
  tx_header.b.data_length = HAL_FDCAN_DATA_LEN_CAN_FDCAN_8_BYTE;
  tx_header.b.bit_rate_switch = HAL_FDCAN_BIT_RATE_SWITCH_OFF;
  tx_header.b.frame_format = HAL_FDCAN_HEADER_FRAME_FORMAT_CAN;
  tx_header.b.event_fifo_control = HAL_FDCAN_TX_EVENTS_FIFO_DISCARD;
  tx_header.b.message_marker = 0;
  txData[0] = 0x05;
  txData[1] = 0x70;
  txData[2] = 0x00;
  txData[3] = 0x00;
  txData[4] = 0x01;
  txData[5] = 0x00;
  txData[6] = 0x00;
  txData[7] = 0x00;

  if (HAL_FDCAN_ReqTransmitMsgFromFIFOQ(hfdcan1, &tx_header, txData) !=
      HAL_OK) {
    FDCAN_Error_Handler();
  }
  HAL_Delay(CAN_MSG_SPACING);
}

void set_position_mode(uint8_t _drive_id) {
  hfdcan1 = mx_fdcan1_gethandle();
  tx_header.b.identifier =
      (COMMAND_SET_PARAMETER << 24) | _drive_id | (HOST_CAN_ID << 8);
  tx_header.b.frame_type = HAL_FDCAN_FRAME_DATA;
  tx_header.b.identifier_type = HAL_FDCAN_ID_EXTENDED;
  tx_header.b.error_state_indicator = HAL_FDCAN_ERROR_STATE_IND_ACTIVE;
  tx_header.b.data_length = HAL_FDCAN_DATA_LEN_CAN_FDCAN_8_BYTE;
  tx_header.b.bit_rate_switch = HAL_FDCAN_BIT_RATE_SWITCH_OFF;
  tx_header.b.frame_format = HAL_FDCAN_HEADER_FRAME_FORMAT_CAN;
  tx_header.b.event_fifo_control = HAL_FDCAN_TX_EVENTS_FIFO_DISCARD;
  tx_header.b.message_marker = 0;
  txData[0] = 0x05;
  txData[1] = 0x70;
  txData[2] = 0x00;
  txData[3] = 0x00;
  txData[4] = 0x02;
  txData[5] = 0x00;
  txData[6] = 0x00;
  txData[7] = 0x00;

  if (HAL_FDCAN_ReqTransmitMsgFromFIFOQ(hfdcan1, &tx_header, txData) !=
      HAL_OK) {
    FDCAN_Error_Handler();
  }
  HAL_Delay(CAN_MSG_SPACING);
}

void set_motor_speed(uint8_t _drive_id, float _rpm) {
  int32_to_uint8_converter.int32 = (int32_t)(_rpm * 100.0f);
  hfdcan1 = mx_fdcan1_gethandle();
  tx_header.b.identifier =
      (COMMAND_SET_PARAMETER << 24) | _drive_id | (HOST_CAN_ID << 8);
  tx_header.b.frame_type = HAL_FDCAN_FRAME_DATA;
  tx_header.b.identifier_type = HAL_FDCAN_ID_EXTENDED;
  tx_header.b.error_state_indicator = HAL_FDCAN_ERROR_STATE_IND_ACTIVE;
  tx_header.b.data_length = HAL_FDCAN_DATA_LEN_CAN_FDCAN_8_BYTE;
  tx_header.b.bit_rate_switch = HAL_FDCAN_BIT_RATE_SWITCH_OFF;
  tx_header.b.frame_format = HAL_FDCAN_HEADER_FRAME_FORMAT_CAN;
  tx_header.b.event_fifo_control = HAL_FDCAN_TX_EVENTS_FIFO_DISCARD;
  tx_header.b.message_marker = 0;
  txData[0] = 0x0A;
  txData[1] = 0x70;
  txData[2] = 0x00;
  txData[3] = 0x00;
  txData[4] = int32_to_uint8_converter.uint8[0];
  txData[5] = int32_to_uint8_converter.uint8[1];
  txData[6] = int32_to_uint8_converter.uint8[2];
  txData[7] = int32_to_uint8_converter.uint8[3];

  if (HAL_FDCAN_ReqTransmitMsgFromFIFOQ(hfdcan1, &tx_header, txData) !=
      HAL_OK) {
    FDCAN_Error_Handler();
  }
  HAL_Delay(CAN_MSG_SPACING);
}

void set_motor_position_pid_kp(uint8_t _drive_id, float _kp) {
  uint32_to_uint8_converter.uint32 = (uint32_t)(_kp * 100000.0f);
  hfdcan1 = mx_fdcan1_gethandle();
  tx_header.b.identifier =
      (COMMAND_SET_PARAMETER << 24) | _drive_id | (HOST_CAN_ID << 8);
  tx_header.b.frame_type = HAL_FDCAN_FRAME_DATA;
  tx_header.b.identifier_type = HAL_FDCAN_ID_EXTENDED;
  tx_header.b.error_state_indicator = HAL_FDCAN_ERROR_STATE_IND_ACTIVE;
  tx_header.b.data_length = HAL_FDCAN_DATA_LEN_CAN_FDCAN_8_BYTE;
  tx_header.b.bit_rate_switch = HAL_FDCAN_BIT_RATE_SWITCH_OFF;
  tx_header.b.frame_format = HAL_FDCAN_HEADER_FRAME_FORMAT_CAN;
  tx_header.b.event_fifo_control = HAL_FDCAN_TX_EVENTS_FIFO_DISCARD;
  tx_header.b.message_marker = 0;
  txData[0] = 0x23;
  txData[1] = 0x70;
  txData[2] = 0x00;
  txData[3] = 0x00;
  txData[4] = uint32_to_uint8_converter.uint8[0];
  txData[5] = uint32_to_uint8_converter.uint8[1];
  txData[6] = uint32_to_uint8_converter.uint8[2];
  txData[7] = uint32_to_uint8_converter.uint8[3];

  if (HAL_FDCAN_ReqTransmitMsgFromFIFOQ(hfdcan1, &tx_header, txData) !=
      HAL_OK) {
    FDCAN_Error_Handler();
  }
  HAL_Delay(CAN_MSG_SPACING);
}

void set_motor_position_pid_ki(uint8_t _drive_id, float _ki) {
  uint32_to_uint8_converter.uint32 = (uint32_t)(_ki * 10000000.0f);
  hfdcan1 = mx_fdcan1_gethandle();
  tx_header.b.identifier =
      (COMMAND_SET_PARAMETER << 24) | _drive_id | (HOST_CAN_ID << 8);
  tx_header.b.frame_type = HAL_FDCAN_FRAME_DATA;
  tx_header.b.identifier_type = HAL_FDCAN_ID_EXTENDED;
  tx_header.b.error_state_indicator = HAL_FDCAN_ERROR_STATE_IND_ACTIVE;
  tx_header.b.data_length = HAL_FDCAN_DATA_LEN_CAN_FDCAN_8_BYTE;
  tx_header.b.bit_rate_switch = HAL_FDCAN_BIT_RATE_SWITCH_OFF;
  tx_header.b.frame_format = HAL_FDCAN_HEADER_FRAME_FORMAT_CAN;
  tx_header.b.event_fifo_control = HAL_FDCAN_TX_EVENTS_FIFO_DISCARD;
  tx_header.b.message_marker = 0;
  txData[0] = 0x24;
  txData[1] = 0x70;
  txData[2] = 0x00;
  txData[3] = 0x00;
  txData[4] = uint32_to_uint8_converter.uint8[0];
  txData[5] = uint32_to_uint8_converter.uint8[1];
  txData[6] = uint32_to_uint8_converter.uint8[2];
  txData[7] = uint32_to_uint8_converter.uint8[3];

  if (HAL_FDCAN_ReqTransmitMsgFromFIFOQ(hfdcan1, &tx_header, txData) !=
      HAL_OK) {
    FDCAN_Error_Handler();
  }
  HAL_Delay(CAN_MSG_SPACING);
}

void set_motor_position_pid_kd(uint8_t _drive_id, float _kd) {
  uint32_to_uint8_converter.uint32 = (uint32_t)(_kd * 100000.0f);
  hfdcan1 = mx_fdcan1_gethandle();
  tx_header.b.identifier =
      (COMMAND_SET_PARAMETER << 24) | _drive_id | (HOST_CAN_ID << 8);
  tx_header.b.frame_type = HAL_FDCAN_FRAME_DATA;
  tx_header.b.identifier_type = HAL_FDCAN_ID_EXTENDED;
  tx_header.b.error_state_indicator = HAL_FDCAN_ERROR_STATE_IND_ACTIVE;
  tx_header.b.data_length = HAL_FDCAN_DATA_LEN_CAN_FDCAN_8_BYTE;
  tx_header.b.bit_rate_switch = HAL_FDCAN_BIT_RATE_SWITCH_OFF;
  tx_header.b.frame_format = HAL_FDCAN_HEADER_FRAME_FORMAT_CAN;
  tx_header.b.event_fifo_control = HAL_FDCAN_TX_EVENTS_FIFO_DISCARD;
  tx_header.b.message_marker = 0;
  txData[0] = 0x25;
  txData[1] = 0x70;
  txData[2] = 0x00;
  txData[3] = 0x00;
  txData[4] = uint32_to_uint8_converter.uint8[0];
  txData[5] = uint32_to_uint8_converter.uint8[1];
  txData[6] = uint32_to_uint8_converter.uint8[2];
  txData[7] = uint32_to_uint8_converter.uint8[3];

  if (HAL_FDCAN_ReqTransmitMsgFromFIFOQ(hfdcan1, &tx_header, txData) !=
      HAL_OK) {
    FDCAN_Error_Handler();
  }
  HAL_Delay(CAN_MSG_SPACING);
}

void set_motor_speed_pid_kp(uint8_t _drive_id, float _kp) {
  uint32_to_uint8_converter.uint32 = (uint32_t)(_kp * 100000.0f);
  hfdcan1 = mx_fdcan1_gethandle();
  tx_header.b.identifier =
      (COMMAND_SET_PARAMETER << 24) | _drive_id | (HOST_CAN_ID << 8);
  tx_header.b.frame_type = HAL_FDCAN_FRAME_DATA;
  tx_header.b.identifier_type = HAL_FDCAN_ID_EXTENDED;
  tx_header.b.error_state_indicator = HAL_FDCAN_ERROR_STATE_IND_ACTIVE;
  tx_header.b.data_length = HAL_FDCAN_DATA_LEN_CAN_FDCAN_8_BYTE;
  tx_header.b.bit_rate_switch = HAL_FDCAN_BIT_RATE_SWITCH_OFF;
  tx_header.b.frame_format = HAL_FDCAN_HEADER_FRAME_FORMAT_CAN;
  tx_header.b.event_fifo_control = HAL_FDCAN_TX_EVENTS_FIFO_DISCARD;
  tx_header.b.message_marker = 0;
  txData[0] = 0x20;
  txData[1] = 0x70;
  txData[2] = 0x00;
  txData[3] = 0x00;
  txData[4] = uint32_to_uint8_converter.uint8[0];
  txData[5] = uint32_to_uint8_converter.uint8[1];
  txData[6] = uint32_to_uint8_converter.uint8[2];
  txData[7] = uint32_to_uint8_converter.uint8[3];

  if (HAL_FDCAN_ReqTransmitMsgFromFIFOQ(hfdcan1, &tx_header, txData) !=
      HAL_OK) {
    FDCAN_Error_Handler();
  }
  HAL_Delay(CAN_MSG_SPACING);
}

void set_motor_speed_pid_ki(uint8_t _drive_id, float _ki) {
  uint32_to_uint8_converter.uint32 = (uint32_t)(_ki * 10000000.0f);
  hfdcan1 = mx_fdcan1_gethandle();
  tx_header.b.identifier =
      (COMMAND_SET_PARAMETER << 24) | _drive_id | (HOST_CAN_ID << 8);
  tx_header.b.frame_type = HAL_FDCAN_FRAME_DATA;
  tx_header.b.identifier_type = HAL_FDCAN_ID_EXTENDED;
  tx_header.b.error_state_indicator = HAL_FDCAN_ERROR_STATE_IND_ACTIVE;
  tx_header.b.data_length = HAL_FDCAN_DATA_LEN_CAN_FDCAN_8_BYTE;
  tx_header.b.bit_rate_switch = HAL_FDCAN_BIT_RATE_SWITCH_OFF;
  tx_header.b.frame_format = HAL_FDCAN_HEADER_FRAME_FORMAT_CAN;
  tx_header.b.event_fifo_control = HAL_FDCAN_TX_EVENTS_FIFO_DISCARD;
  tx_header.b.message_marker = 0;
  txData[0] = 0x21;
  txData[1] = 0x70;
  txData[2] = 0x00;
  txData[3] = 0x00;
  txData[4] = uint32_to_uint8_converter.uint8[0];
  txData[5] = uint32_to_uint8_converter.uint8[1];
  txData[6] = uint32_to_uint8_converter.uint8[2];
  txData[7] = uint32_to_uint8_converter.uint8[3];

  if (HAL_FDCAN_ReqTransmitMsgFromFIFOQ(hfdcan1, &tx_header, txData) !=
      HAL_OK) {
    FDCAN_Error_Handler();
  }
  HAL_Delay(CAN_MSG_SPACING);
}

void set_motor_speed_pid_kd(uint8_t _drive_id, float _kd) {
  uint32_to_uint8_converter.uint32 = (uint32_t)(_kd * 100000.0f);
  hfdcan1 = mx_fdcan1_gethandle();
  tx_header.b.identifier =
      (COMMAND_SET_PARAMETER << 24) | _drive_id | (HOST_CAN_ID << 8);
  tx_header.b.frame_type = HAL_FDCAN_FRAME_DATA;
  tx_header.b.identifier_type = HAL_FDCAN_ID_EXTENDED;
  tx_header.b.error_state_indicator = HAL_FDCAN_ERROR_STATE_IND_ACTIVE;
  tx_header.b.data_length = HAL_FDCAN_DATA_LEN_CAN_FDCAN_8_BYTE;
  tx_header.b.bit_rate_switch = HAL_FDCAN_BIT_RATE_SWITCH_OFF;
  tx_header.b.frame_format = HAL_FDCAN_HEADER_FRAME_FORMAT_CAN;
  tx_header.b.event_fifo_control = HAL_FDCAN_TX_EVENTS_FIFO_DISCARD;
  tx_header.b.message_marker = 0;
  txData[0] = 0x22;
  txData[1] = 0x70;
  txData[2] = 0x00;
  txData[3] = 0x00;
  txData[4] = uint32_to_uint8_converter.uint8[0];
  txData[5] = uint32_to_uint8_converter.uint8[1];
  txData[6] = uint32_to_uint8_converter.uint8[2];
  txData[7] = uint32_to_uint8_converter.uint8[3];

  if (HAL_FDCAN_ReqTransmitMsgFromFIFOQ(hfdcan1, &tx_header, txData) !=
      HAL_OK) {
    FDCAN_Error_Handler();
  }
  HAL_Delay(CAN_MSG_SPACING);
}

void set_motor_position(uint8_t _drive_id, float _position) {
  int32_to_uint8_converter.int32 = (int32_t)(_position * 100.0f);
  hfdcan1 = mx_fdcan1_gethandle();
  tx_header.b.identifier =
      (COMMAND_SET_PARAMETER << 24) | _drive_id | (HOST_CAN_ID << 8);
  tx_header.b.frame_type = HAL_FDCAN_FRAME_DATA;
  tx_header.b.identifier_type = HAL_FDCAN_ID_EXTENDED;
  tx_header.b.error_state_indicator = HAL_FDCAN_ERROR_STATE_IND_ACTIVE;
  tx_header.b.data_length = HAL_FDCAN_DATA_LEN_CAN_FDCAN_8_BYTE;
  tx_header.b.bit_rate_switch = HAL_FDCAN_BIT_RATE_SWITCH_OFF;
  tx_header.b.frame_format = HAL_FDCAN_HEADER_FRAME_FORMAT_CAN;
  tx_header.b.event_fifo_control = HAL_FDCAN_TX_EVENTS_FIFO_DISCARD;
  tx_header.b.message_marker = 0;
  txData[0] = 0x16;
  txData[1] = 0x70;
  txData[2] = 0x00;
  txData[3] = 0x00;
  txData[4] = int32_to_uint8_converter.uint8[0];
  txData[5] = int32_to_uint8_converter.uint8[1];
  txData[6] = int32_to_uint8_converter.uint8[2];
  txData[7] = int32_to_uint8_converter.uint8[3];

  if (HAL_FDCAN_ReqTransmitMsgFromFIFOQ(hfdcan1, &tx_header, txData) !=
      HAL_OK) {
    FDCAN_Error_Handler();
  }
  HAL_Delay(CAN_MSG_SPACING);
}

void start_motor(uint8_t _drive_id) {
  hfdcan1 = mx_fdcan1_gethandle();
  tx_header.b.identifier =
      (COMMAND_SET_PARAMETER << 24) | _drive_id | (HOST_CAN_ID << 8);
  tx_header.b.frame_type = HAL_FDCAN_FRAME_DATA;
  tx_header.b.identifier_type = HAL_FDCAN_ID_EXTENDED;
  tx_header.b.error_state_indicator = HAL_FDCAN_ERROR_STATE_IND_ACTIVE;
  tx_header.b.data_length = HAL_FDCAN_DATA_LEN_CAN_FDCAN_8_BYTE;
  tx_header.b.bit_rate_switch = HAL_FDCAN_BIT_RATE_SWITCH_OFF;
  tx_header.b.frame_format = HAL_FDCAN_HEADER_FRAME_FORMAT_CAN;
  tx_header.b.event_fifo_control = HAL_FDCAN_TX_EVENTS_FIFO_DISCARD;
  tx_header.b.message_marker = 0;
  txData[0] = 0x04;
  txData[1] = 0x70;
  txData[2] = 0x00;
  txData[3] = 0x00;
  txData[4] = 0x01;
  txData[5] = 0x00;
  txData[6] = 0x00;
  txData[7] = 0x00;

  if (HAL_FDCAN_ReqTransmitMsgFromFIFOQ(hfdcan1, &tx_header, txData) !=
      HAL_OK) {
    FDCAN_Error_Handler();
  }
  HAL_Delay(CAN_MSG_SPACING);
}

void stop_motor(uint8_t _drive_id) {
  hfdcan1 = mx_fdcan1_gethandle();
  tx_header.b.identifier =
      (COMMAND_SET_PARAMETER << 24) | _drive_id | (HOST_CAN_ID << 8);
  tx_header.b.frame_type = HAL_FDCAN_FRAME_DATA;
  tx_header.b.identifier_type = HAL_FDCAN_ID_EXTENDED;
  tx_header.b.error_state_indicator = HAL_FDCAN_ERROR_STATE_IND_ACTIVE;
  tx_header.b.data_length = HAL_FDCAN_DATA_LEN_CAN_FDCAN_8_BYTE;
  tx_header.b.bit_rate_switch = HAL_FDCAN_BIT_RATE_SWITCH_OFF;
  tx_header.b.frame_format = HAL_FDCAN_HEADER_FRAME_FORMAT_CAN;
  tx_header.b.event_fifo_control = HAL_FDCAN_TX_EVENTS_FIFO_DISCARD;
  tx_header.b.message_marker = 0;
  txData[0] = 0x04;
  txData[1] = 0x70;
  txData[2] = 0x00;
  txData[3] = 0x00;
  txData[4] = 0x00;
  txData[5] = 0x00;
  txData[6] = 0x00;
  txData[7] = 0x00;

  if (HAL_FDCAN_ReqTransmitMsgFromFIFOQ(hfdcan1, &tx_header, txData) !=
      HAL_OK) {
    FDCAN_Error_Handler();
  }
  HAL_Delay(CAN_MSG_SPACING);
}

float get_motor_position(uint8_t _drive_id) { // deg
  hfdcan1 = mx_fdcan1_gethandle();
  tx_header.b.identifier =
      (COMMAND_GET_PARAMETER << 24) | _drive_id | (HOST_CAN_ID << 8);
  tx_header.b.frame_type = HAL_FDCAN_FRAME_DATA;
  tx_header.b.identifier_type = HAL_FDCAN_ID_EXTENDED;
  tx_header.b.error_state_indicator = HAL_FDCAN_ERROR_STATE_IND_ACTIVE;
  tx_header.b.data_length = HAL_FDCAN_DATA_LEN_CAN_FDCAN_8_BYTE;
  tx_header.b.bit_rate_switch = HAL_FDCAN_BIT_RATE_SWITCH_OFF;
  tx_header.b.frame_format = HAL_FDCAN_HEADER_FRAME_FORMAT_CAN;
  tx_header.b.event_fifo_control = HAL_FDCAN_TX_EVENTS_FIFO_DISCARD;
  tx_header.b.message_marker = 0;
  txData[0] = 0x31;
  txData[1] = 0x70;
  txData[2] = 0x00;
  txData[3] = 0x00;
  txData[4] = 0x00;
  txData[5] = 0x00;
  txData[6] = 0x00;
  txData[7] = 0x00;

  if (HAL_FDCAN_ReqTransmitMsgFromFIFOQ(hfdcan1, &tx_header, txData) !=
      HAL_OK) {
    FDCAN_Error_Handler();
  }

  uint32_t timeout_start = HAL_GetTick();
  while ((position_received_flag != _drive_id) &&
         (HAL_GetTick() - timeout_start <
          CAN_RX_TIMEOUT)) { // to do: error raporting
    __NOP();
  }

  switch (_drive_id) {
  case TRIPOD_LEG_A_CAN_ID:
    int32_to_uint8_converter.uint8[0] = rxDataBis[TRIPOD_LEG_A_IDX][4];
    int32_to_uint8_converter.uint8[1] = rxDataBis[TRIPOD_LEG_A_IDX][5];
    int32_to_uint8_converter.uint8[2] = rxDataBis[TRIPOD_LEG_A_IDX][6];
    int32_to_uint8_converter.uint8[3] = rxDataBis[TRIPOD_LEG_A_IDX][7];
    break;
  case TRIPOD_LEG_B_CAN_ID:
    int32_to_uint8_converter.uint8[0] = rxDataBis[TRIPOD_LEG_B_IDX][4];
    int32_to_uint8_converter.uint8[1] = rxDataBis[TRIPOD_LEG_B_IDX][5];
    int32_to_uint8_converter.uint8[2] = rxDataBis[TRIPOD_LEG_B_IDX][6];
    int32_to_uint8_converter.uint8[3] = rxDataBis[TRIPOD_LEG_B_IDX][7];
    break;
  case TRIPOD_LEG_C_CAN_ID:
    int32_to_uint8_converter.uint8[0] = rxDataBis[TRIPOD_LEG_C_IDX][4];
    int32_to_uint8_converter.uint8[1] = rxDataBis[TRIPOD_LEG_C_IDX][5];
    int32_to_uint8_converter.uint8[2] = rxDataBis[TRIPOD_LEG_C_IDX][6];
    int32_to_uint8_converter.uint8[3] = rxDataBis[TRIPOD_LEG_C_IDX][7];
    break;
  default:
    __NOP();
  }

  position_received_flag = 0;

  return ((float)(int32_to_uint8_converter.int32)) / 100.0f; // deg
}

float get_motor_speed(uint8_t _drive_id) { // rpm
  hfdcan1 = mx_fdcan1_gethandle();
  tx_header.b.identifier =
      (COMMAND_GET_PARAMETER << 24) | _drive_id | (HOST_CAN_ID << 8);
  tx_header.b.frame_type = HAL_FDCAN_FRAME_DATA;
  tx_header.b.identifier_type = HAL_FDCAN_ID_EXTENDED;
  tx_header.b.error_state_indicator = HAL_FDCAN_ERROR_STATE_IND_ACTIVE;
  tx_header.b.data_length = HAL_FDCAN_DATA_LEN_CAN_FDCAN_8_BYTE;
  tx_header.b.bit_rate_switch = HAL_FDCAN_BIT_RATE_SWITCH_OFF;
  tx_header.b.frame_format = HAL_FDCAN_HEADER_FRAME_FORMAT_CAN;
  tx_header.b.event_fifo_control = HAL_FDCAN_TX_EVENTS_FIFO_DISCARD;
  tx_header.b.message_marker = 0;
  txData[0] = 0x30;
  txData[1] = 0x70;
  txData[2] = 0x00;
  txData[3] = 0x00;
  txData[4] = 0x00;
  txData[5] = 0x00;
  txData[6] = 0x00;
  txData[7] = 0x00;

  if (HAL_FDCAN_ReqTransmitMsgFromFIFOQ(hfdcan1, &tx_header, txData) !=
      HAL_OK) {
    FDCAN_Error_Handler();
  }

  uint32_t timeout_start = HAL_GetTick();
  while ((speed_received_flag != _drive_id) &&
         (HAL_GetTick() - timeout_start <
          CAN_RX_TIMEOUT)) { // to do: error raporting
    __NOP();
  }

  switch (_drive_id) {
  case TRIPOD_LEG_A_CAN_ID:
    int32_to_uint8_converter.uint8[0] = rxDataBis[TRIPOD_LEG_A_IDX][4];
    int32_to_uint8_converter.uint8[1] = rxDataBis[TRIPOD_LEG_A_IDX][5];
    int32_to_uint8_converter.uint8[2] = rxDataBis[TRIPOD_LEG_A_IDX][6];
    int32_to_uint8_converter.uint8[3] = rxDataBis[TRIPOD_LEG_A_IDX][7];
    break;
  case TRIPOD_LEG_B_CAN_ID:
    int32_to_uint8_converter.uint8[0] = rxDataBis[TRIPOD_LEG_B_IDX][4];
    int32_to_uint8_converter.uint8[1] = rxDataBis[TRIPOD_LEG_B_IDX][5];
    int32_to_uint8_converter.uint8[2] = rxDataBis[TRIPOD_LEG_B_IDX][6];
    int32_to_uint8_converter.uint8[3] = rxDataBis[TRIPOD_LEG_B_IDX][7];
    break;
  case TRIPOD_LEG_C_CAN_ID:
    int32_to_uint8_converter.uint8[0] = rxDataBis[TRIPOD_LEG_C_IDX][4];
    int32_to_uint8_converter.uint8[1] = rxDataBis[TRIPOD_LEG_C_IDX][5];
    int32_to_uint8_converter.uint8[2] = rxDataBis[TRIPOD_LEG_C_IDX][6];
    int32_to_uint8_converter.uint8[3] = rxDataBis[TRIPOD_LEG_C_IDX][7];
    break;
  default:
    __NOP();
  }

  speed_received_flag = 0;

  return ((float)(int32_to_uint8_converter.int32)) / 100.0f; // rpm
}

float get_motor_current(uint8_t _drive_id) { // mA
  hfdcan1 = mx_fdcan1_gethandle();
  tx_header.b.identifier =
      (COMMAND_GET_PARAMETER << 24) | _drive_id | (HOST_CAN_ID << 8);
  tx_header.b.frame_type = HAL_FDCAN_FRAME_DATA;
  tx_header.b.identifier_type = HAL_FDCAN_ID_EXTENDED;
  tx_header.b.error_state_indicator = HAL_FDCAN_ERROR_STATE_IND_ACTIVE;
  tx_header.b.data_length = HAL_FDCAN_DATA_LEN_CAN_FDCAN_8_BYTE;
  tx_header.b.bit_rate_switch = HAL_FDCAN_BIT_RATE_SWITCH_OFF;
  tx_header.b.frame_format = HAL_FDCAN_HEADER_FRAME_FORMAT_CAN;
  tx_header.b.event_fifo_control = HAL_FDCAN_TX_EVENTS_FIFO_DISCARD;
  tx_header.b.message_marker = 0;
  txData[0] = 0x32;
  txData[1] = 0x70;
  txData[2] = 0x00;
  txData[3] = 0x00;
  txData[4] = 0x00;
  txData[5] = 0x00;
  txData[6] = 0x00;
  txData[7] = 0x00;

  if (HAL_FDCAN_ReqTransmitMsgFromFIFOQ(hfdcan1, &tx_header, txData) !=
      HAL_OK) {
    FDCAN_Error_Handler();
  }

  uint32_t timeout_start = HAL_GetTick();
  while ((current_received_flag != _drive_id) &&
         (HAL_GetTick() - timeout_start <
          CAN_RX_TIMEOUT)) { // to do: error raporting
    __NOP();
  }

  switch (_drive_id) {
  case TRIPOD_LEG_A_CAN_ID:
    int32_to_uint8_converter.uint8[0] = rxDataBis[TRIPOD_LEG_A_IDX][4];
    int32_to_uint8_converter.uint8[1] = rxDataBis[TRIPOD_LEG_A_IDX][5];
    int32_to_uint8_converter.uint8[2] = rxDataBis[TRIPOD_LEG_A_IDX][6];
    int32_to_uint8_converter.uint8[3] = rxDataBis[TRIPOD_LEG_A_IDX][7];
    break;
  case TRIPOD_LEG_B_CAN_ID:
    int32_to_uint8_converter.uint8[0] = rxDataBis[TRIPOD_LEG_B_IDX][4];
    int32_to_uint8_converter.uint8[1] = rxDataBis[TRIPOD_LEG_B_IDX][5];
    int32_to_uint8_converter.uint8[2] = rxDataBis[TRIPOD_LEG_B_IDX][6];
    int32_to_uint8_converter.uint8[3] = rxDataBis[TRIPOD_LEG_B_IDX][7];
    break;
  case TRIPOD_LEG_C_CAN_ID:
    int32_to_uint8_converter.uint8[0] = rxDataBis[TRIPOD_LEG_C_IDX][4];
    int32_to_uint8_converter.uint8[1] = rxDataBis[TRIPOD_LEG_C_IDX][5];
    int32_to_uint8_converter.uint8[2] = rxDataBis[TRIPOD_LEG_C_IDX][6];
    int32_to_uint8_converter.uint8[3] = rxDataBis[TRIPOD_LEG_C_IDX][7];
    break;
  default:
    __NOP();
  }

  current_received_flag = 0;

  return ((float)(int32_to_uint8_converter.int32)) / 100.0f; // mA
}

float get_motor_temperature(uint8_t _drive_id) { // deg. C
  hfdcan1 = mx_fdcan1_gethandle();
  tx_header.b.identifier =
      (COMMAND_GET_PARAMETER << 24) | _drive_id | (HOST_CAN_ID << 8);
  tx_header.b.frame_type = HAL_FDCAN_FRAME_DATA;
  tx_header.b.identifier_type = HAL_FDCAN_ID_EXTENDED;
  tx_header.b.error_state_indicator = HAL_FDCAN_ERROR_STATE_IND_ACTIVE;
  tx_header.b.data_length = HAL_FDCAN_DATA_LEN_CAN_FDCAN_8_BYTE;
  tx_header.b.bit_rate_switch = HAL_FDCAN_BIT_RATE_SWITCH_OFF;
  tx_header.b.frame_format = HAL_FDCAN_HEADER_FRAME_FORMAT_CAN;
  tx_header.b.event_fifo_control = HAL_FDCAN_TX_EVENTS_FIFO_DISCARD;
  tx_header.b.message_marker = 0;
  txData[0] = 0x35;
  txData[1] = 0x70;
  txData[2] = 0x00;
  txData[3] = 0x00;
  txData[4] = 0x00;
  txData[5] = 0x00;
  txData[6] = 0x00;
  txData[7] = 0x00;

  if (HAL_FDCAN_ReqTransmitMsgFromFIFOQ(hfdcan1, &tx_header, txData) !=
      HAL_OK) {
    FDCAN_Error_Handler();
  }

  uint32_t timeout_start = HAL_GetTick();
  while ((temperature_received_flag != _drive_id) &&
         (HAL_GetTick() - timeout_start <
          CAN_RX_TIMEOUT)) { // to do: error raporting
    __NOP();
  }

  switch (_drive_id) {
  case TRIPOD_LEG_A_CAN_ID:
    int32_to_uint8_converter.uint8[0] = rxDataBis[TRIPOD_LEG_A_IDX][4];
    int32_to_uint8_converter.uint8[1] = rxDataBis[TRIPOD_LEG_A_IDX][5];
    int32_to_uint8_converter.uint8[2] = rxDataBis[TRIPOD_LEG_A_IDX][6];
    int32_to_uint8_converter.uint8[3] = rxDataBis[TRIPOD_LEG_A_IDX][7];
    break;
  case TRIPOD_LEG_B_CAN_ID:
    int32_to_uint8_converter.uint8[0] = rxDataBis[TRIPOD_LEG_B_IDX][4];
    int32_to_uint8_converter.uint8[1] = rxDataBis[TRIPOD_LEG_B_IDX][5];
    int32_to_uint8_converter.uint8[2] = rxDataBis[TRIPOD_LEG_B_IDX][6];
    int32_to_uint8_converter.uint8[3] = rxDataBis[TRIPOD_LEG_B_IDX][7];
    break;
  case TRIPOD_LEG_C_CAN_ID:
    int32_to_uint8_converter.uint8[0] = rxDataBis[TRIPOD_LEG_C_IDX][4];
    int32_to_uint8_converter.uint8[1] = rxDataBis[TRIPOD_LEG_C_IDX][5];
    int32_to_uint8_converter.uint8[2] = rxDataBis[TRIPOD_LEG_C_IDX][6];
    int32_to_uint8_converter.uint8[3] = rxDataBis[TRIPOD_LEG_C_IDX][7];
    break;
  default:
    __NOP();
  }

  temperature_received_flag = 0;

  return (float)(int32_to_uint8_converter.int32); // deg. C
}

void FDCAN_Error_Handler(void) {
  while (1) {
    __NOP();
    // Choose your favorite way to go into the panic mode :)
  }
}
