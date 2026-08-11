/**
 ******************************************************************************
 * file           : main.c
 * brief          : Main program body
 *                  Calls target system initialization then loop in main.
 ******************************************************************************
 *
 * Copyright (c) 2025 STMicroelectronics.
 * All rights reserved.
 *
 * This software is licensed under terms that can be found in the LICENSE file
 * in the root directory of this software component.
 * If no LICENSE file comes with this software, it is provided AS-IS.
 *
 ******************************************************************************
 */
/* Includes ------------------------------------------------------------------*/
#include "main.h"
#include "mx_tim16.h"
#include "rollercan.h"
#include "stm32c5xx_hal.h"
#include "stm32c5xx_hal_exti.h"
#include "stm32c5xx_hal_tim.h"
#include <math.h>
#include <stdio.h>
#include <string.h>

/* Private typedef -----------------------------------------------------------*/

/* Private define ------------------------------------------------------------*/

// #define USE_ENCODER_FOR_SYNCHRONIZATION
// less precise because accessed over the CAN bus (no interrupt)

#define KNOB_CAN_ID TRIPOD_LEG_C_CAN_ID

#define WS2812_LOGIC_0_CCR 1440 / 24 // PWM clock at 144 MHz
#define WS2812_LOGIC_1_CCR 1440 / 12
#define NUMBER_OF_LEDS 8
#define LED_BUFFER_SIZE (3 * NUMBER_OF_LEDS + 8)

// GRB vs. RGB
#define GREEN_IDX 0
#define RED_IDX 1
#define BLUE_IDX 2

#define MESSAGE_SIZE 5

// There are 8 leds and with letters 5x7 one can start from LED number 0 or 1:
#define LED_OFFSET 1

#define PAREMETER_READ_PERIOD 400 // ms

/* Private macro -------------------------------------------------------------*/

/* Private variables ---------------------------------------------------------*/

hal_tim_handle_t *ws2812_tim;
hal_tim_handle_t *delay_us_tim;
uint8_t ws2812_data[LED_BUFFER_SIZE];
uint8_t ws2812_ccr[8 * LED_BUFFER_SIZE]; // 64 * 1.25 us reset code
float motor_position;
float motor_speed;
float motor_current;
float motor_temperature;
uint8_t b1_pressed_flag = 0;
uint8_t b1_toggle_variable = 0;
uint32_t ParameterReadSoftTimer;

// create a glyph
const uint8_t biased_message[MESSAGE_SIZE][8] = { // I :heart: K & J
    {0b00111, 0b00010, 0b00010, 0b00010, 0b00010, 0b10010, 0b01100, 0b00000},
    {0b01100, 0b10010, 0b10100, 0b01000, 0b10101, 0b10010, 0b01101, 0b00000},
    {0b10001, 0b10010, 0b10100, 0b11000, 0b10100, 0b10010, 0b10001, 0b00000},
    {0b00000, 0b00000, 0b01010, 0b10101, 0b10001, 0b01010, 0b00100, 0b00000},
    {0b01110, 0b00100, 0b00100, 0b00100, 0b00100, 0b00100, 0b01110, 0b00000}};
// const uint8_t biased_message[MESSAGE_SIZE][8] = { // I :heart: STM
//     {0b10001, 0b11011, 0b10101, 0b10101, 0b10001, 0b10001, 0b10001, 0b00000},
//     {0b11111, 0b00100, 0b00100, 0b00100, 0b00100, 0b00100, 0b00100, 0b00000},
//     {0b01111, 0b10000, 0b10000, 0b01110, 0b00001, 0b00001, 0b11110, 0b00000},
//     {0b00000, 0b00000, 0b01010, 0b10101, 0b10001, 0b01010, 0b00100, 0b00000},
//     {0b01110, 0b00100, 0b00100, 0b00100, 0b00100, 0b00100, 0b01110,
//     0b00000}};
const uint8_t char_colors[MESSAGE_SIZE] = {GREEN_IDX, GREEN_IDX, GREEN_IDX,
                                           RED_IDX, GREEN_IDX};

/* Private functions prototype -----------------------------------------------*/

void prepare_ws2812_packet(uint8_t *_data, uint8_t *_ccr, uint8_t _num_of_leds);
void delay_us(uint32_t _delay);
void clear_all_ws2812(void);
void show_pov_message(void);

/**
 * brief:  The application entry point.
 * retval: none but we specify int to comply with C99 standard
 */
int main(void) {
  /** System Init: this code placed in targets folder initializes your system.
   * It calls the initialization (and sets the initial configuration) of the
   * peripherals. You can use STM32CubeMX to generate and call this code or not
   * in this project. It also contains the HAL initialization and the initial
   * clock configuration.
   */
  if (mx_system_init() != SYSTEM_OK) {
    return (-1);
  } else {
    /*
     * You can start your application code here
     */
    rollercan_can_config();
    enable_operation(KNOB_CAN_ID);
    set_speed_mode(KNOB_CAN_ID);
    set_motor_speed(KNOB_CAN_ID, 600);
    start_motor(KNOB_CAN_ID);

    ws2812_tim = mx_tim16_gethandle();
    HAL_TIM_Start(ws2812_tim);

    delay_us_tim = mx_tim17_gethandle();
    HAL_TIM_Start(delay_us_tim);

    ParameterReadSoftTimer = HAL_GetTick();

    while (1) {

      if (b1_pressed_flag == 1) {
        b1_pressed_flag = 0;
        b1_toggle_variable ^= 1U;
        if (b1_toggle_variable == 1) {
          stop_motor(KNOB_CAN_ID);
        } else {
          start_motor(KNOB_CAN_ID);
        }
      }

      if (HAL_GetTick() - ParameterReadSoftTimer > PAREMETER_READ_PERIOD) {
        ParameterReadSoftTimer = HAL_GetTick();
        motor_speed = get_motor_speed(KNOB_CAN_ID);
        motor_current /* mA */ = get_motor_current(KNOB_CAN_ID);
        motor_temperature = get_motor_temperature(KNOB_CAN_ID);
      }

#ifdef USE_ENCODER_FOR_SYNCHRONIZATION
      motor_position = get_motor_position(KNOB_CAN_ID);
      if (fabsf(fmodf(motor_position, 360.0f) - 180.0f) < 2.0f) {
        show_pov_message();
      }
#endif

    } /* end while(1)*/
  }
} /* end main */

void prepare_ws2812_packet(uint8_t *_data, uint8_t *_ccr,
                           uint8_t _num_of_leds) {
  for (uint16_t i = 0; i < (8 * 3 * _num_of_leds); i++) {
    _ccr[i] = ((((_data[i / 8]) >> (i % 8)) && 0x01) == 0 ? WS2812_LOGIC_0_CCR
                                                          : WS2812_LOGIC_1_CCR);
  }
  // reset code >= 50 us
  for (uint16_t i = 8 * 3 * _num_of_leds; i < (8 * (3 * _num_of_leds + 8));
       i++) {
    _ccr[i] = 0;
  }
}

void delay_us(uint32_t _delay) {
  uint32_t _start = HAL_TIM_GetCounter(delay_us_tim);
  while (HAL_TIM_GetCounter(delay_us_tim) - _start < _delay) {
    __NOP();
  }
}

void clear_all_ws2812(void) { memset(ws2812_data, 0x00, LED_BUFFER_SIZE); }

void HAL_EXTI_TriggerCallback(hal_exti_handle_t *hexti,
                              hal_exti_trigger_t trigger) {

#ifndef USE_ENCODER_FOR_SYNCHRONIZATION
  if ((hexti->line == HAL_EXTI_LINE_10) &&
      (trigger == HAL_EXTI_TRIGGER_RISING)) {
    show_pov_message();
  }
#endif

  if ((hexti->line == HAL_EXTI_LINE_13) &&
      (trigger == HAL_EXTI_TRIGGER_RISING)) {
    b1_pressed_flag = 1;
  }
}

void show_pov_message(void) {
  for (uint8_t k = 0; k < MESSAGE_SIZE; k++) {
    for (uint8_t j = 0; j < 8;
         j++) { // 8-5 gives additional spacing between characters
      for (uint8_t i = 0; i < 8 - LED_OFFSET; i++) {
        ws2812_data[3 * (i + LED_OFFSET) + char_colors[k]] =
            0xFF * (biased_message[k][i] & (1U << j));
      }
      prepare_ws2812_packet(ws2812_data, ws2812_ccr, NUMBER_OF_LEDS);
      HAL_TIM_OC_StartChannel_DMA(ws2812_tim, HAL_TIM_CHANNEL_1, ws2812_ccr,
                                  8 * LED_BUFFER_SIZE);
      delay_us(350);

      clear_all_ws2812();
      prepare_ws2812_packet(ws2812_data, ws2812_ccr, NUMBER_OF_LEDS);
      HAL_TIM_OC_StartChannel_DMA(ws2812_tim, HAL_TIM_CHANNEL_1, ws2812_ccr,
                                  8 * LED_BUFFER_SIZE);
      delay_us(550);
    }
  }
}
