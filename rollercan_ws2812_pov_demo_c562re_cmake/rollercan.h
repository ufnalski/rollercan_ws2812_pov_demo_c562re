/*
 * rollercan.h
 *
 *  Created on: Mar 15, 2026
 *      Author: ufnalski
 */

#ifndef INC_ROLLERCAN_H_
#define INC_ROLLERCAN_H_

#include <stdint.h>
#include <string.h>

#define TRIPOD_LEG_A_CAN_ID 0xA7 // violet
#define TRIPOD_LEG_A_IDX 0
#define TRIPOD_LEG_B_CAN_ID 0xA9 // black
#define TRIPOD_LEG_B_IDX 1
#define TRIPOD_LEG_C_CAN_ID 0xA8 // red
#define TRIPOD_LEG_C_IDX 2

#define HOST_CAN_ID 0xBEEF

#define CAN_MSG_SPACING 1
#define CAN_RX_TIMEOUT 4

#define COMMAND_OBTAIN_ID 0x00
#define COMMAND_ENABLE_OPERATION 0x03
#define COMMAND_STOP_OPERATION 0x04
#define COMMAND_GET_PARAMETER 0x11
#define COMMAND_SET_PARAMETER 0x12

void rollercan_can_config(void);

void ask_for_id(uint8_t _drive_id);
void enable_operation(uint8_t _drive_id);
void stop_operation(uint8_t _drive_id);
void set_speed_mode(uint8_t _drive_id);
void set_position_mode(uint8_t _drive_id);

void set_motor_position_pid_kp(uint8_t _drive_id, float _kp);
void set_motor_position_pid_ki(uint8_t _drive_id, float _ki);
void set_motor_position_pid_kd(uint8_t _drive_id, float _kd);

void set_motor_speed_pid_kp(uint8_t _drive_id, float _kp);
void set_motor_speed_pid_ki(uint8_t _drive_id, float _ki);
void set_motor_speed_pid_kd(uint8_t _drive_id, float _kd);

void set_motor_speed(uint8_t _drive_id, float _rpm);
void set_motor_position(uint8_t _drive_id, float _position);
void start_motor(uint8_t _drive_id);
void stop_motor(uint8_t _drive_id);
float get_motor_position(uint8_t _drive_id);
float get_motor_speed(uint8_t _drive_id);
float get_motor_current(uint8_t _drive_id);
float get_motor_temperature(uint8_t _drive_id);

void FDCAN_Error_Handler(void);

#endif /* INC_ROLLERCAN_H_ */
