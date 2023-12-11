/* USER CODE BEGIN Header */
/**
 ******************************************************************************
 * @file           : main.c
 * @brief          : Main program body
 ******************************************************************************
 * @attention
 *
 * Copyright (c) 2023 STMicroelectronics.
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

/* Private includes ---------------------------------
 * -------------------------*/
/* USER CODE BEGIN Includes */
#include "fonts.h"
#include "ssd1306.h"
#include "mpu6050.h"
#include "eeprom.h"
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */

#define leftBaseSpeed 555
#define rightBaseSpeed 625

#define rightEmitterPort GPIOB
#define rightEmitterPin GPIO_PIN_10
#define leftEmitterPort GPIOB
#define leftEmitterPin GPIO_PIN_11
#define front1EmitterPort GPIOB
#define front1EmitterPin GPIO_PIN_15
#define front2EmitterPort GPIOB
#define front2EmitterPin GPIO_PIN_12

#define switchPort GPIOA
#define switchPin GPIO_PIN_1

#define normal_foreward 0
#define normal_foreward_follow 4
#define left_follow 1
#define center_follow 2
#define right_follow 3

#define EEPROM_ADDR 0xA0
/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
ADC_HandleTypeDef hadc1;
//I2C_HandleTypeDef hi2c1;
TIM_HandleTypeDef htim1;
TIM_HandleTypeDef htim2;
TIM_HandleTypeDef htim3;
/* USER CODE BEGIN PV */
ADC_ChannelConfTypeDef sConfigPrivate = { 0 };
MPU6050_t MPU6050;

uint8_t visit[15][15] = { { 200, 200, 200, 200, 200, 200, 200, 200, 200, 200,
		200, 200, 200, 200, 200 }, { 200, 1, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
		0 }, { 200, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0 }, { 200, 0, 0, 0,
		0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0 }, { 200, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
		0, 0, 0, 0 }, { 200, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0 }, { 200,
		0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0 }, { 200, 0, 0, 0, 0, 0, 0, 0,
		0, 0, 0, 0, 0, 0, 0 },
		{ 200, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0 }, { 200, 0, 0, 0, 0, 0,
				0, 0, 0, 0, 0, 0, 0, 0, 0 }, { 200, 0, 0, 0, 0, 0, 0, 0, 0, 0,
				0, 0, 0, 0, 0 },
		{ 200, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0 }, { 200, 0, 0, 0, 0, 0,
				0, 0, 0, 0, 0, 0, 0, 0, 0 }, { 200, 0, 0, 0, 0, 0, 0, 0, 0, 0,
				0, 0, 0, 0, 0 },
		{ 200, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0 }, };
uint8_t *visit_ptr[15][15];

/*
 uint8_t maze_value_initial[15][15] = { { 200, 200, 200, 200, 200, 200, 200, 200,
 200, 200, 200, 200, 200, 200, 200 }, { 200, 12, 11, 10, 9, 8, 7, 6, 6,
 7, 8, 9, 10, 11, 12 }, { 200, 11, 10, 9, 8, 7, 6, 5, 5, 6, 7, 8, 9, 10,
 11 }, { 200, 10, 9, 8, 7, 6, 5, 4, 4, 5, 6, 7, 8, 9, 10 }, { 200, 9, 8,
 7, 6, 5, 4, 3, 3, 4, 5, 6, 7, 8, 9 }, { 200, 8, 7, 6, 5, 4, 3, 2, 2, 3,
 4, 5, 6, 7, 8 }, { 200, 7, 6, 5, 4, 3, 2, 1, 1, 2, 3, 4, 5, 6, 7 }, {
 200, 6, 5, 4, 3, 2, 1, 0, 0, 1, 2, 3, 4, 5, 6 }, { 200, 6, 5, 4, 3, 2,
 1, 0, 0, 1, 2, 3, 4, 5, 6 }, { 200, 7, 6, 5, 4, 3, 2, 1, 1, 2, 3, 4, 5,
 6, 7 }, { 200, 8, 7, 6, 5, 4, 3, 2, 2, 3, 4, 5, 6, 7, 8 }, { 200, 9, 8,
 7, 6, 5, 4, 3, 3, 4, 5, 6, 7, 8, 9 }, { 200, 10, 9, 8, 7, 6, 5, 4, 4, 5,
 6, 7, 8, 9, 10 }, { 200, 11, 10, 9, 8, 7, 6, 5, 5, 6, 7, 8, 9, 10, 11 },
 { 200, 12, 11, 10, 9, 8, 7, 6, 6, 7, 8, 9, 10, 11, 12 } };

 uint8_t *maze_value_initial_ptr[15][15];*/

uint8_t maze_value[15][15] = { { 200, 200, 200, 200, 200, 200, 200, 200, 200, //this is main processing flood values
		200, 200, 200, 200, 200, 200 }, { 200, 12, 11, 10, 9, 8, 7, 6, 6, 7, 8,
		9, 10, 11, 12 }, { 200, 11, 10, 9, 8, 7, 6, 5, 5, 6, 7, 8, 9, 10, 11 },
		{ 200, 10, 9, 8, 7, 6, 5, 4, 4, 5, 6, 7, 8, 9, 10 }, { 200, 9, 8, 7, 6,
				5, 4, 3, 3, 4, 5, 6, 7, 8, 9 }, { 200, 8, 7, 6, 5, 4, 3, 2, 2,
				3, 4, 5, 6, 7, 8 }, { 200, 7, 6, 5, 4, 3, 2, 1, 1, 2, 3, 4, 5,
				6, 7 }, { 200, 6, 5, 4, 3, 2, 1, 0, 0, 1, 2, 3, 4, 5, 6 }, {
				200, 6, 5, 4, 3, 2, 1, 0, 0, 1, 2, 3, 4, 5, 6 }, { 200, 7, 6, 5,
				4, 3, 2, 1, 1, 2, 3, 4, 5, 6, 7 }, { 200, 8, 7, 6, 5, 4, 3, 2,
				2, 3, 4, 5, 6, 7, 8 }, { 200, 9, 8, 7, 6, 5, 4, 3, 3, 4, 5, 6,
				7, 8, 9 }, { 200, 10, 9, 8, 7, 6, 5, 4, 4, 5, 6, 7, 8, 9, 10 },
		{ 200, 11, 10, 9, 8, 7, 6, 5, 5, 6, 7, 8, 9, 10, 11 }, { 200, 12, 11,
				10, 9, 8, 7, 6, 6, 7, 8, 9, 10, 11, 12 } };

uint8_t *maze_value_ptr[15][15];

uint8_t maze_value_back[15][15] = { { 200, 200, 200, 200, 200, 200, 200, 200,
		200, 200, 200, 200, 200, 200, 200 }, { 200, 0, 1, 2, 3, 4, 5, 6, 7, 8,
		9, 10, 11, 12, 13 }, { 200, 1, 2, 3, 4, 5, 6, 7, 8, 9, 10, 11, 12, 13,
		14 }, { 200, 2, 3, 4, 5, 6, 7, 8, 9, 10, 11, 12, 13, 14, 15 }, { 200, 3,
		4, 5, 6, 7, 8, 9, 10, 11, 12, 13, 14, 15, 16 }, { 200, 4, 5, 6, 7, 8, 9,
		10, 11, 12, 13, 14, 15, 16, 17 }, { 200, 5, 6, 7, 8, 9, 10, 11, 12, 13,
		14, 15, 16, 17, 18 }, { 200, 6, 7, 8, 9, 10, 11, 12, 13, 14, 15, 16, 17,
		18, 19 }, { 200, 7, 8, 9, 10, 11, 12, 13, 14, 15, 16, 17, 18, 19, 20 },
		{ 200, 8, 9, 10, 11, 12, 13, 14, 15, 16, 17, 18, 19, 20, 21 }, { 200, 9,
				10, 11, 12, 13, 14, 15, 16, 17, 18, 19, 20, 21, 22 }, { 200, 10,
				11, 12, 13, 14, 15, 16, 17, 18, 19, 20, 21, 22, 23 }, { 200, 11,
				12, 13, 14, 15, 16, 17, 18, 19, 20, 21, 22, 23, 24 }, { 200, 12,
				13, 14, 15, 16, 17, 18, 19, 20, 21, 22, 23, 24, 25 }, { 200, 13,
				14, 15, 16, 17, 18, 19, 20, 21, 22, 23, 24, 25, 26 } };

uint8_t *maze_value_back_ptr[15][15];

uint8_t maze_value_back_0[15][15] = { { 200, 200, 200, 200, 200, 200, 200, 200,
		200, 200, 200, 200, 200, 200, 200 }, { 200, 1, 0, 1, 2, 3, 4, 5, 6, 7,
		8, 9, 10, 11, 12 },
		{ 200, 2, 1, 2, 3, 4, 5, 6, 7, 8, 9, 10, 11, 12, 13 }, { 200, 3, 2, 3,
				4, 5, 6, 7, 8, 9, 10, 11, 12, 13, 14 }, { 200, 4, 3, 4, 5, 6, 7,
				8, 9, 10, 11, 12, 13, 14, 15 }, { 200, 5, 4, 5, 6, 7, 8, 9, 10,
				11, 12, 13, 14, 15, 16 }, { 200, 6, 5, 6, 7, 8, 9, 10, 11, 12,
				13, 14, 15, 16, 17 }, { 200, 7, 6, 7, 8, 9, 10, 11, 12, 13, 14,
				15, 16, 17, 18 }, { 200, 8, 7, 8, 9, 10, 11, 12, 13, 14, 15, 16,
				17, 18, 19 }, { 200, 9, 8, 9, 10, 11, 12, 13, 14, 15, 16, 17,
				18, 19, 20 }, { 200, 10, 9, 10, 11, 12, 13, 14, 15, 16, 17, 18,
				19, 20, 21 }, { 200, 11, 10, 11, 12, 13, 14, 15, 16, 17, 18, 19,
				20, 21, 22 }, { 200, 12, 11, 12, 13, 14, 15, 16, 17, 18, 19, 20,
				21, 22, 23 }, { 200, 13, 12, 13, 14, 15, 16, 17, 18, 19, 20, 21,
				22, 23, 24 }, { 200, 14, 13, 14, 15, 16, 17, 18, 19, 20, 21, 22,
				23, 24, 25 } };

uint8_t *maze_value_back_ptr_0[15][15];

uint8_t maze_value_back_1[15][15] = { { 200, 200, 200, 200, 200, 200, 200, 200,
		200, 200, 200, 200, 200, 200, 200 }, { 200, 1, 2, 3, 4, 5, 6, 7, 8, 9,
		10, 11, 12, 13, 14 }, { 200, 0, 1, 2, 3, 4, 5, 6, 7, 8, 9, 10, 11, 12,
		13 }, { 200, 1, 2, 3, 4, 5, 6, 7, 8, 9, 10, 11, 12, 13, 14 }, { 200, 2,
		3, 4, 5, 6, 7, 8, 9, 10, 11, 12, 13, 14, 15 }, { 200, 3, 4, 5, 6, 7, 8,
		9, 10, 11, 12, 13, 14, 15, 16 }, { 200, 4, 5, 6, 7, 8, 9, 10, 11, 12,
		13, 14, 15, 16, 17 }, { 200, 5, 6, 7, 8, 9, 10, 11, 12, 13, 14, 15, 16,
		17, 18 }, { 200, 6, 7, 8, 9, 10, 11, 12, 13, 14, 15, 16, 17, 18, 19 }, {
		200, 7, 8, 9, 10, 11, 12, 13, 14, 15, 16, 17, 18, 19, 20 }, { 200, 8, 9,
		10, 11, 12, 13, 14, 15, 16, 17, 18, 19, 20, 21 }, { 200, 9, 10, 11, 12,
		13, 14, 15, 16, 17, 18, 19, 20, 21, 22 }, { 200, 10, 11, 12, 13, 14, 15,
		16, 17, 18, 19, 20, 21, 22, 23 }, { 200, 11, 12, 13, 14, 15, 16, 17, 18,
		19, 20, 21, 22, 23, 24 }, { 200, 12, 13, 14, 15, 16, 17, 18, 19, 20, 21,
		22, 23, 24, 25 }

};

uint8_t *maze_value_back_ptr_1[15][15];

uint8_t walls[29][29] = { { 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1,
		1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1 }, { 1, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
		0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 1 },
		{ 1, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
				0, 0, 0, 0, 0, 1 }, { 1, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
				0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 1 }, { 1, 0, 0, 0, 0,
				0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
				0, 0, 1 }, { 1, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
				0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 1 }, { 1, 0, 0, 0, 0, 0, 0, 0,
				0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 1 },
		{ 1, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
				0, 0, 0, 0, 0, 1 }, { 1, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
				0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 1 }, { 1, 0, 0, 0, 0,
				0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
				0, 0, 1 }, { 1, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
				0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 1 }, { 1, 0, 0, 0, 0, 0, 0, 0,
				0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 1 },
		{ 1, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
				0, 0, 0, 0, 0, 1 }, { 1, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
				0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 1 }, { 1, 0, 0, 0, 0,
				0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
				0, 0, 1 }, { 1, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
				0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 1 }, { 1, 0, 0, 0, 0, 0, 0, 0,
				0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 1 },
		{ 1, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
				0, 0, 0, 0, 0, 1 }, { 1, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
				0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 1 }, { 1, 0, 0, 0, 0,
				0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
				0, 0, 1 }, { 1, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
				0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 1 }, { 1, 0, 0, 0, 0, 0, 0, 0,
				0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 1 },
		{ 1, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
				0, 0, 0, 0, 0, 1 }, { 1, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
				0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 1 }, { 1, 0, 0, 0, 0,
				0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
				0, 0, 1 }, { 1, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
				0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 1 }, { 1, 0, 0, 0, 0, 0, 0, 0,
				0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 1 },
		{ 1, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
				0, 0, 0, 0, 0, 1 }, { 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1,
				1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1 } };
uint8_t *walls_ptr[29][29];

int8_t orient = 0;
int8_t x = 1;
int8_t y = 1;

int8_t centerx;
int8_t centery;
int8_t centerorient;

int8_t brake_activate = 1;
int8_t path_block_front = 0;
int8_t path_block_right = 0;
int8_t path_block_left = 0;

#define N 70
uint8_t stack_x[N];
uint8_t stack_y[N];

uint8_t flood_x;
uint8_t flood_y;

uint8_t toWrite_0[14];
uint8_t toWrite_1[14];
uint8_t toWrite_2[14];
uint8_t toWrite_3[14];
uint8_t toWrite_4[14];
uint8_t toWrite_5[14];
uint8_t toWrite_6[14];
uint8_t toWrite_7[14];
uint8_t toWrite_8[14];
uint8_t toWrite_9[14];
uint8_t toWrite_10[14];
uint8_t toWrite_11[14];
uint8_t toWrite_12[14];
uint8_t toWrite_13[14];

uint8_t toRead_0[14];
uint8_t toRead_1[14];
uint8_t toRead_2[14];
uint8_t toRead_3[14];
uint8_t toRead_4[14];
uint8_t toRead_5[14];
uint8_t toRead_6[14];
uint8_t toRead_7[14];
uint8_t toRead_8[14];
uint8_t toRead_9[14];
uint8_t toRead_10[14];
uint8_t toRead_11[14];
uint8_t toRead_12[14];
uint8_t toRead_13[14];

int8_t pass_orient = 0;
/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_ADC1_Init(void);
//static void MX_I2C1_Init(void);
static void MX_TIM1_Init(void);
static void MX_TIM2_Init(void);
static void MX_TIM3_Init(void);

void follow_maze_value_array();
void goToFrontBlock();
void turnToNextBlock();
void advance(int mode, uint16_t dis);
void raedSideSensors(void);
void raedFrontSensors(void);
void update_coordinate();
void update_walls();
void disactivate_brake();
void check_path_blocked();
void flood_maze_values();
void turnToNextBlock_after_flood();
void get_minimum_accesible_neighbour();
void get_minimum_accesible_neighbour();

void followWallLeft(void);
void followWallRight(void);
void followWallCenter(void);

void alignFrontAngle(void);
void alignFrontDistance(void);

void mdrive(int mleft, int mright);
void turnRight(void);
void turnLeft(void);
void turnBack(void);
void brake(int time);

void push(uint8_t x, uint8_t y);
void pop();

void saveMaze();
void readMaze();
void clearMaze();

void virtual_flood();
void turnToNextBlock_after_flood_virtual();
void get_walls_from_wall_array();

void turnToNextBlock_optimized();
void get_minimum_accesible_neighbour_value_for_optimized();

//void turnToNextBlock_optimized();

/* USER CODE BEGIN PFP */

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */
uint16_t front2;
uint16_t right;
uint16_t front1;
uint16_t left;

uint32_t leftStep = 0;
uint32_t rightStep = 0;

uint32_t currentLeftStep = 0;
uint32_t currentLrightStep = 0;

uint8_t leftWall = 0;
uint8_t rightWall = 0;
uint8_t frontWall = 0;

int16_t accesible_1 = 0;
int16_t accesible_2 = 0;
int16_t accesible_3 = 0;
int16_t accesible_4 = 0;

int8_t minimum_accesible_value;

int8_t mode = -1;

/* USER CODE END 0 */

/**
 * @brief  The application entry point.
 * @retval int
 */
int main(void) {
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
	MX_I2C1_Init();
	MX_TIM1_Init();
	MX_TIM2_Init();
	MX_TIM3_Init();
	/* USER CODE BEGIN 2 */

	for (int i = 0; i < 15; i++) {
		for (int j = 0; j < 15; j++) {
			visit_ptr[i][j] = &visit[i][j];
		}
	}
	for (int i = 0; i < 15; i++) {
		for (int j = 0; j < 15; j++) {
			maze_value_ptr[i][j] = &maze_value[i][j];
		}
	}
	for (int i = 0; i < 15; i++) {
		for (int j = 0; j < 15; j++) {
			maze_value_back_ptr[i][j] = &maze_value_back[i][j];
		}
	}
	for (int i = 0; i < 15; i++) {
		for (int j = 0; j < 15; j++) {
			maze_value_back_ptr_0[i][j] = &maze_value_back_0[i][j];
		}
	}
	for (int i = 0; i < 15; i++) {
		for (int j = 0; j < 15; j++) {
			maze_value_back_ptr_1[i][j] = &maze_value_back_1[i][j];
		}
	}

	for (int i = 0; i < 29; i++) {
		for (int j = 0; j < 29; j++) {
			walls_ptr[i][j] = &walls[i][j];
		}
	}

	HAL_TIM_Base_Start_IT(&htim2);
	HAL_TIM_Base_Start_IT(&htim1);
	HAL_ADC_Start(&hadc1);
	SSD1306_Init();

	HAL_TIM_PWM_Start(&htim3, TIM_CHANNEL_1);
	HAL_TIM_PWM_Start(&htim3, TIM_CHANNEL_2);
	HAL_TIM_PWM_Start(&htim3, TIM_CHANNEL_3);
	HAL_TIM_PWM_Start(&htim3, TIM_CHANNEL_4);

	/*while (MPU6050_Init(&hi2c1) == 1)
		;
*/
	HAL_Delay(5000);
	mode=0;
	/* USER CODE END 2 */

	/* Infinite loop */
	/* USER CODE BEGIN WHILE */
	while (1) {

		if (mode == -1) {
			if (HAL_GPIO_ReadPin(switchPort, switchPin) == GPIO_PIN_RESET) {
				SSD1306_GotoXY(0, 0);
				SSD1306_Puts(" speed ", &Font_11x18, 1);
				SSD1306_UpdateScreen();
				HAL_Delay(1000);
				SSD1306_Clear();
				mode = 100;
			} else {
				raedFrontSensors();
				if (front1 > 2000 && front2 > 2000) {
					SSD1306_GotoXY(0, 0);
					SSD1306_Puts("erasing...", &Font_11x18, 1);
					SSD1306_UpdateScreen();
					clearMaze();
					HAL_Delay(1000);
					SSD1306_Clear();
				}
				SSD1306_GotoXY(0, 0);
				SSD1306_Puts(" search ", &Font_11x18, 1);
				SSD1306_UpdateScreen();
				HAL_Delay(1000);
				SSD1306_Clear();
				mode = 0;
			}
		}
		if (mode == 0) {
			raedFrontSensors();
			if (front1 > 2000) {
				SSD1306_GotoXY(0, 0);
				SSD1306_Puts(" left ", &Font_11x18, 1);
				SSD1306_UpdateScreen();
				HAL_Delay(1000);
				SSD1306_Clear();
				readMaze();
				orient = 1;
				pass_orient = orient;
				mode = 1;
			}
			if (front2 > 2000) {
				SSD1306_GotoXY(0, 0);
				SSD1306_Puts(" right ", &Font_11x18, 1);
				SSD1306_UpdateScreen();
				HAL_Delay(1000);
				SSD1306_Clear();
				readMaze();
				orient = 0;
				pass_orient = orient;
				mode = 1;
			}
		}
		if (mode == 1) {                      // first search
			goToFrontBlock();
			turnToNextBlock();
		}
		if (mode == 2) {
			saveMaze();
			for (uint8_t x = 0; x < 15; x++) { //assign back flood array to maze value array
				for (uint8_t y = 0; y < 15; y++) {
					*maze_value_ptr[x][y] = *maze_value_back_ptr[x][y];
				}
			}
			mode = 33;
		}
		if (mode == 33) {
			virtual_flood(-1);
		}
		if (mode == 3) {                       // back search
			goToFrontBlock();
			turnToNextBlock();
		}
		if (mode == 4) {
			turnBack();
			readMaze();
			mode = 5;
		}
		if (mode == 5) {
			virtual_flood(1);
		}

		/////////////////////////////////////////////////////////////////////////////////////////////////////////////
		if (mode == 100) {
			readMaze();
			mode = 101;
		}
		if (mode == 101) {
			raedFrontSensors();
			if (front1 > 2000) {
				SSD1306_GotoXY(0, 0);
				SSD1306_Puts(" left ", &Font_11x18, 1);
				SSD1306_UpdateScreen();
				HAL_Delay(1000);
				SSD1306_Clear();
				orient = 1;
				pass_orient = orient;
				mode = 102;
			}
			if (front2 > 2000) {
				SSD1306_GotoXY(0, 0);
				SSD1306_Puts(" right ", &Font_11x18, 1);
				SSD1306_UpdateScreen();
				HAL_Delay(1000);
				SSD1306_Clear();
				orient = 0;
				pass_orient = orient;
				mode = 102;

			}
		}
		if (mode == 102) {
			goToFrontBlock();
			turnToNextBlock();
		}
		if (mode == 103) {

			for (uint8_t x = 0; x < 15; x++) { //assign back flood array to maze value array
				for (uint8_t y = 0; y < 15; y++) {
					*maze_value_ptr[x][y] = *maze_value_back_ptr[x][y];
				}
			}

			mode = 104;

		}
		if (mode == 104) {                     //back flood
			virtual_flood(-1);
		}
		if (mode == 105) {                       // back search
			goToFrontBlock();
			turnToNextBlock();
		}
		if (mode == 106) {
			turnBack();
			SSD1306_GotoXY(0, 30);
			SSD1306_Puts("finish..", &Font_11x18, 1);
			SSD1306_UpdateScreen();
			HAL_Delay(1000);
			SSD1306_Clear();
			mode = 100;
		}
		/* USER CODE END WHILE */

		/* USER CODE BEGIN 3 */
	}
	/* USER CODE END 3 */
}

/* USER CODE BEGIN 4 */
void clearMaze() {
	for (uint8_t s = 0; s < 15; s++) {
		for (uint8_t i = 15 * s; i < 15 * (s + 1); i++) {
			EEPROM_Write_NUM(i, 0, *maze_value_ptr[s][i - (15 * s)]);
		}
	}
}
void saveMaze() {
	for (uint8_t s = 0; s < 15; s++) {
		for (uint8_t i = 15 * s; i < 15 * (s + 1); i++) {
			EEPROM_Write_NUM(i, 0, *maze_value_ptr[s][i - (15 * s)]);
		}
	}
}
void readMaze() {
	for (uint8_t s = 0; s < 15; s++) {
		for (uint8_t i = 15 * s; i < 15 * (s + 1); i++) {
			*maze_value_ptr[s][i - (15 * s)] = EEPROM_Read_NUM(i, 0);
		}
	}
}

void virtual_flood(int dir) {
	update_coordinate();
	get_walls_from_wall_array();
//	SSD1306_printValue(0, 0, x);
//	SSD1306_printValue(30, 0, y);
//	HAL_Delay(100);
	if (*maze_value_ptr[x][y] == 0) {
		if (dir == 1) {
			x = 1;
			y = 1;
			orient = pass_orient;
		} else {
			x = centerx;
			y = centery;
			orient = centerorient;
		}

		if (mode == 33) {
			mode = 3;
			return;
		}
		if (mode == 5) {
			mode = 0;
			return;
		}
		if (mode == 104) {
			mode = 105;
			return;
		}

		//	SSD1306_Clear();

	}

	if (orient == 0) {

		if (*maze_value_ptr[x][y + 1] + 1 == *maze_value_ptr[x][y]
				&& frontWall == 0) { //foreward

		} else if (*maze_value_ptr[x + 1][y] + 1 == *maze_value_ptr[x][y]
				&& rightWall == 0) { //right
			flood_maze_values();
			orient++;
			if (orient == 4) {
				orient = 0;
			}
		} else if (*maze_value_ptr[x - 1][y] + 1 == *maze_value_ptr[x][y]
				&& leftWall == 0) { //left
			flood_maze_values();

			orient--;
			if (orient == -1) {
				orient = 3;
			}
		} else {
			flood_maze_values();
			turnToNextBlock_after_flood_virtual();
		}

	} else if (orient == 1) {

		if (*maze_value_ptr[x + 1][y] + 1 == *maze_value_ptr[x][y]
				&& frontWall == 0) { //foreward

		} else if (*maze_value_ptr[x][y - 1] + 1 == *maze_value_ptr[x][y]
				&& rightWall == 0) { //right

			orient++;
			if (orient == 4) {
				orient = 0;
			}

		} else if (*maze_value_ptr[x][y + 1] + 1 == *maze_value_ptr[x][y]
				&& leftWall == 0) { //left

			orient--;
			if (orient == -1) {
				orient = 3;
			}
		} else {
			flood_maze_values();
			turnToNextBlock_after_flood_virtual();
		}

	} else if (orient == 2) {

		if (*maze_value_ptr[x][y - 1] + 1 == *maze_value_ptr[x][y]
				&& frontWall == 0) { //foreward

		} else if (*maze_value_ptr[x - 1][y] + 1 == *maze_value_ptr[x][y]
				&& rightWall == 0) { //right
			orient++;
			if (orient == 4) {
				orient = 0;
			}
		} else if (*maze_value_ptr[x + 1][y] + 1 == *maze_value_ptr[x][y]
				&& leftWall == 0) { //left
			orient--;
			if (orient == -1) {
				orient = 3;
			}
		} else {

			flood_maze_values();
			turnToNextBlock_after_flood_virtual();
		}

	} else if (orient == 3) { // orient == 3

		if (*maze_value_ptr[x - 1][y] + 1 == *maze_value_ptr[x][y]
				&& frontWall == 0) { //foreward

		} else if (*maze_value_ptr[x][y + 1] + 1 == *maze_value_ptr[x][y]
				&& rightWall == 0) { //right
			orient++;
			if (orient == 4) {
				orient = 0;
			}
		} else if (*maze_value_ptr[x][y - 1] + 1 == *maze_value_ptr[x][y]
				&& leftWall == 0) { //left
			orient--;
			if (orient == -1) {
				orient = 3;
			}
		} else {
			flood_maze_values();
			turnToNextBlock_after_flood_virtual();
		}
	}

}

# define check_side_wall_advance_distance 43//20
# define check_front_wall_advance_distance 58//80
# define to_block_middle_advance_distance 22

void goToFrontBlock() {
	frontWall = 0;
	rightWall = 0;
	leftWall = 0;
	advance(normal_foreward, check_side_wall_advance_distance);
	raedSideSensors();
	if (left > 300) {
		leftWall = 1;
	}
	if (right > 300) {
		rightWall = 1;
	}


	if (leftWall == 1 && rightWall == 1) {
		advance(center_follow, check_front_wall_advance_distance);
	} else if (leftWall == 1 && rightWall == 0) {
		advance(left_follow, check_front_wall_advance_distance);
	} else if (leftWall == 0 && rightWall == 1) {
		advance(right_follow, check_front_wall_advance_distance);
	} else {
		advance(normal_foreward_follow, check_front_wall_advance_distance);
	}

	raedFrontSensors();
	if ((front1 + front2) / 2 > 400) {
		frontWall = 1;
	}

	update_coordinate();
	*visit_ptr[x][y] = 1;
	update_walls();

	disactivate_brake();

	if (brake_activate == 1) {
		advance(normal_foreward, to_block_middle_advance_distance);
		brake(10);
		HAL_Delay(50);
	}
	if (brake_activate == 0) {
		advance(normal_foreward, to_block_middle_advance_distance + 5);
	}
	//HAL_Delay(40);

}

/*
 void turnToNextBlock_optimized() {

 get_minimum_accesible_neighbour_value_for_optimized();

 if (*maze_value_ptr[x][y] == 0) {
 SSD1306_GotoXY(0, 0);
 SSD1306_Puts(" finishhhhh", &Font_11x18, 1);
 SSD1306_UpdateScreen();
 HAL_Delay(40000);

 mode = 2;

 }

 if (orient == 0) {

 if (*maze_value_ptr[x][y + 1] == minimum_accesible_value) { //foreward

 } else if (*maze_value_ptr[x + 1][y] == minimum_accesible_value) { //right
 turnRight();
 } else if (*maze_value_ptr[x - 1][y] == minimum_accesible_value) { //left
 turnLeft();
 } else {
 turnBack();
 }

 } else if (orient == 1) {

 if (*maze_value_ptr[x + 1][y] == minimum_accesible_value) { //foreward

 } else if (*maze_value_ptr[x][y - 1] == minimum_accesible_value) { //right

 turnRight();

 } else if (*maze_value_ptr[x][y + 1] == minimum_accesible_value) { //left

 turnLeft();

 } else {
 turnBack();

 }
 } else if (orient == 2) {

 if (*maze_value_ptr[x][y - 1] == minimum_accesible_value) { //foreward

 } else if (*maze_value_ptr[x - 1][y] == minimum_accesible_value) { //right
 turnRight();
 } else if (*maze_value_ptr[x + 1][y] == minimum_accesible_value) { //left
 turnLeft();
 } else {
 turnBack();
 }

 } else if (orient == 3) { // orient == 3

 if (*maze_value_ptr[x - 1][y] == minimum_accesible_value) { //foreward

 } else if (*maze_value_ptr[x][y + 1] == minimum_accesible_value) { //right
 turnRight();
 } else if (*maze_value_ptr[x][y - 1] == minimum_accesible_value) { //left
 turnLeft();
 } else {
 turnBack();
 }
 }
 }
 */

void disactivate_brake() {

	if (*maze_value_ptr[x][y] == 0) {
		brake_activate = 1;
	}

	if (orient == 0) {

		if (*maze_value_ptr[x][y + 1] + 1 == *maze_value_ptr[x][y]
				&& frontWall == 0) { //foreward
			brake_activate = 0;
		} else if (*maze_value_ptr[x + 1][y] + 1 == *maze_value_ptr[x][y]
				&& rightWall == 0) { //right
			brake_activate = 1;
		} else if (*maze_value_ptr[x - 1][y] + 1 == *maze_value_ptr[x][y]
				&& leftWall == 0) { //left
			brake_activate = 1;
		} else {
			brake_activate = 1;
			//flood_maze_values();
			//turnToNextBlock_after_flood();
		}

	} else if (orient == 1) {

		if (*maze_value_ptr[x + 1][y] + 1 == *maze_value_ptr[x][y]
				&& frontWall == 0) { //foreward
			brake_activate = 0;
		} else if (*maze_value_ptr[x][y - 1] + 1 == *maze_value_ptr[x][y]
				&& rightWall == 0) { //right

			brake_activate = 1;

		} else if (*maze_value_ptr[x][y + 1] + 1 == *maze_value_ptr[x][y]
				&& leftWall == 0) { //left

			brake_activate = 1;

		} else {
			brake_activate = 1;
			//flood_maze_values();
			//turnToNextBlock_after_flood();
		}

	} else if (orient == 2) {

		if (*maze_value_ptr[x][y - 1] + 1 == *maze_value_ptr[x][y]
				&& frontWall == 0) { //foreward
			brake_activate = 0;
		} else if (*maze_value_ptr[x - 1][y] + 1 == *maze_value_ptr[x][y]
				&& rightWall == 0) { //right
			brake_activate = 1;
		} else if (*maze_value_ptr[x + 1][y] + 1 == *maze_value_ptr[x][y]
				&& leftWall == 0) { //left
			brake_activate = 1;
		} else {
			brake_activate = 1;
			//flood_maze_values();
			//turnToNextBlock_after_flood();
		}

	} else if (orient == 3) { // orient == 3

		if (*maze_value_ptr[x - 1][y] + 1 == *maze_value_ptr[x][y]
				&& frontWall == 0) { //foreward
			brake_activate = 0;
		} else if (*maze_value_ptr[x][y + 1] + 1 == *maze_value_ptr[x][y]
				&& rightWall == 0) { //right
			brake_activate = 1;
		} else if (*maze_value_ptr[x][y - 1] + 1 == *maze_value_ptr[x][y]
				&& leftWall == 0) { //left
			brake_activate = 1;
		} else {
			brake_activate = 1;
			//flood_maze_values();
			//turnToNextBlock_after_flood();
		}
	}
}

void turnToNextBlock() {

	if (*maze_value_ptr[x][y] == 0) {
		brake(10);
		HAL_Delay(50);
		SSD1306_GotoXY(0, 0);
		SSD1306_Puts(" finish", &Font_11x18, 1);
		SSD1306_UpdateScreen();
		HAL_Delay(800);

		centerx = x;
		centery = y;
		centerorient = orient;

		if (mode == 1) {
			mode = 2;
			return;
		}
		if (mode == 3) {
			mode = 4;
			return;
		}
		if (mode == 102) {
			mode = 103;
			return;
		}
		if (mode == 105) {
			mode = 106;
			return;
		}

	}

	if (orient == 0) {

		if (*maze_value_ptr[x][y + 1] + 1 == *maze_value_ptr[x][y]
				&& frontWall == 0) { //foreward

		} else if (*maze_value_ptr[x + 1][y] + 1 == *maze_value_ptr[x][y]
				&& rightWall == 0) { //right
			turnRight();
		} else if (*maze_value_ptr[x - 1][y] + 1 == *maze_value_ptr[x][y]
				&& leftWall == 0) { //left
			turnLeft();
		} else {
			flood_maze_values();
			turnToNextBlock_after_flood();
		}

	} else if (orient == 1) {

		if (*maze_value_ptr[x + 1][y] + 1 == *maze_value_ptr[x][y]
				&& frontWall == 0) { //foreward

		} else if (*maze_value_ptr[x][y - 1] + 1 == *maze_value_ptr[x][y]
				&& rightWall == 0) { //right

			turnRight();

		} else if (*maze_value_ptr[x][y + 1] + 1 == *maze_value_ptr[x][y]
				&& leftWall == 0) { //left

			turnLeft();

		} else {
			flood_maze_values();
			turnToNextBlock_after_flood();
		}

	} else if (orient == 2) {

		if (*maze_value_ptr[x][y - 1] + 1 == *maze_value_ptr[x][y]
				&& frontWall == 0) { //foreward

		} else if (*maze_value_ptr[x - 1][y] + 1 == *maze_value_ptr[x][y]
				&& rightWall == 0) { //right
			turnRight();
		} else if (*maze_value_ptr[x + 1][y] + 1 == *maze_value_ptr[x][y]
				&& leftWall == 0) { //left
			turnLeft();
		} else {

			flood_maze_values();
			turnToNextBlock_after_flood();
		}

	} else if (orient == 3) { // orient == 3

		if (*maze_value_ptr[x - 1][y] + 1 == *maze_value_ptr[x][y]
				&& frontWall == 0) { //foreward

		} else if (*maze_value_ptr[x][y + 1] + 1 == *maze_value_ptr[x][y]
				&& rightWall == 0) { //right
			turnRight();
		} else if (*maze_value_ptr[x][y - 1] + 1 == *maze_value_ptr[x][y]
				&& leftWall == 0) { //left
			turnLeft();
		} else {
			flood_maze_values();
			turnToNextBlock_after_flood();
		}
	}
}

void turnToNextBlock_after_flood() {
//check_path_blocked();
	if (orient == 0) {

		if (*maze_value_ptr[x][y + 1] + 1 == *maze_value_ptr[x][y]
				&& frontWall == 0) { //foreward

		} else if (*maze_value_ptr[x + 1][y] + 1 == *maze_value_ptr[x][y]
				&& rightWall == 0) { //right
			turnRight();
		} else if (*maze_value_ptr[x - 1][y] + 1 == *maze_value_ptr[x][y]
				&& leftWall == 0) { //left
			turnLeft();
		} else {
			turnBack();
		}

	} else if (orient == 1) {

		if (*maze_value_ptr[x + 1][y] + 1 == *maze_value_ptr[x][y]
				&& frontWall == 0) { //foreward

		} else if (*maze_value_ptr[x][y - 1] + 1 == *maze_value_ptr[x][y]
				&& rightWall == 0) { //right

			turnRight();

		} else if (*maze_value_ptr[x][y + 1] + 1 == *maze_value_ptr[x][y]
				&& leftWall == 0) { //left

			turnLeft();

		} else {
			turnBack();
		}

	} else if (orient == 2) {

		if (*maze_value_ptr[x][y - 1] + 1 == *maze_value_ptr[x][y]
				&& frontWall == 0) { //foreward

		} else if (*maze_value_ptr[x - 1][y] + 1 == *maze_value_ptr[x][y]
				&& rightWall == 0) { //right
			turnRight();
		} else if (*maze_value_ptr[x + 1][y] + 1 == *maze_value_ptr[x][y]
				&& leftWall == 0) { //left
			turnLeft();
		} else {
			turnBack();
		}
	} else if (orient == 3) { // orient == 3

		if (*maze_value_ptr[x - 1][y] + 1 == *maze_value_ptr[x][y]
				&& frontWall == 0) { //foreward

		} else if (*maze_value_ptr[x][y + 1] + 1 == *maze_value_ptr[x][y]
				&& rightWall == 0) { //right
			turnRight();
		} else if (*maze_value_ptr[x][y - 1] + 1 == *maze_value_ptr[x][y]
				&& leftWall == 0) { //left
			turnLeft();
		} else {
			turnBack();
		}
	}
}

void turnToNextBlock_after_flood_virtual() {
//check_path_blocked();
	if (orient == 0) {

		if (*maze_value_ptr[x][y + 1] + 1 == *maze_value_ptr[x][y]
				&& frontWall == 0) { //foreward

		} else if (*maze_value_ptr[x + 1][y] + 1 == *maze_value_ptr[x][y]
				&& rightWall == 0) { //right
			orient++;
			if (orient == 4) {
				orient = 0;
			}
		} else if (*maze_value_ptr[x - 1][y] + 1 == *maze_value_ptr[x][y]
				&& leftWall == 0) { //left
			orient--;
			if (orient == -1) {
				orient = 3;
			}
		} else {
			orient++;
			if (orient == 4) {
				orient = 0;
			}
			orient++;
			if (orient == 4) {
				orient = 0;
			}
		}

	} else if (orient == 1) {

		if (*maze_value_ptr[x + 1][y] + 1 == *maze_value_ptr[x][y]
				&& frontWall == 0) { //foreward

		} else if (*maze_value_ptr[x][y - 1] + 1 == *maze_value_ptr[x][y]
				&& rightWall == 0) { //right
			orient++;
			if (orient == 4) {
				orient = 0;
			}
		} else if (*maze_value_ptr[x][y + 1] + 1 == *maze_value_ptr[x][y]
				&& leftWall == 0) { //left
			orient--;
			if (orient == -1) {
				orient = 3;
			}

		} else {
			orient++;
			if (orient == 4) {
				orient = 0;
			}
			orient++;
			if (orient == 4) {
				orient = 0;
			}
		}

	} else if (orient == 2) {

		if (*maze_value_ptr[x][y - 1] + 1 == *maze_value_ptr[x][y]
				&& frontWall == 0) { //foreward

		} else if (*maze_value_ptr[x - 1][y] + 1 == *maze_value_ptr[x][y]
				&& rightWall == 0) { //right
			orient++;
			if (orient == 4) {
				orient = 0;
			}
		} else if (*maze_value_ptr[x + 1][y] + 1 == *maze_value_ptr[x][y]
				&& leftWall == 0) { //left
			orient--;
			if (orient == -1) {
				orient = 3;
			}
		} else {
			orient++;
			if (orient == 4) {
				orient = 0;
			}
			orient++;
			if (orient == 4) {
				orient = 0;
			}
		}
	} else if (orient == 3) { // orient == 3

		if (*maze_value_ptr[x - 1][y] + 1 == *maze_value_ptr[x][y]
				&& frontWall == 0) { //foreward

		} else if (*maze_value_ptr[x][y + 1] + 1 == *maze_value_ptr[x][y]
				&& rightWall == 0) { //right
			orient++;
			if (orient == 4) {
				orient = 0;
			}
		} else if (*maze_value_ptr[x][y - 1] + 1 == *maze_value_ptr[x][y]
				&& leftWall == 0) { //left
			orient--;
			if (orient == -1) {
				orient = 3;
			}
		} else {
			orient++;
			if (orient == 4) {
				orient = 0;
			}
			orient++;
			if (orient == 4) {
				orient = 0;
			}
		}
	}
}

int8_t top = -1;

void flood_maze_values() {
//SSD1306_printValue(0, 0, x);
//SSD1306_printValue(40, 0, y);
	push(x, y);
//HAL_Delay(100);
	while (top > -1) {
		//SSD1306_printValue(70, 20, top);
		//HAL_Delay(100);
		pop();

		get_minimum_accesible_neighbour();

		if (*maze_value_ptr[flood_x][flood_y] <= minimum_accesible_value) {
			*maze_value_ptr[flood_x][flood_y] = minimum_accesible_value + 1;

			if (accesible_1 < 1000) {
				push(flood_x, flood_y + 1);
			}
			if (accesible_2 < 1000) {
				push(flood_x + 1, flood_y);
			}
			if (accesible_3 < 1000) {
				push(flood_x, flood_y - 1);
			}
			if (accesible_4 < 1000) {
				push(flood_x - 1, flood_y);
			}
		}
	}
}

void get_walls_from_wall_array() {
	frontWall = 0;
	rightWall = 0;
	leftWall = 0;

	uint8_t x_walls = ((x - 1) * 2) + 1;
	uint8_t y_walls = ((y - 1) * 2) + 1;

	if (orient == 0) {
		if (*walls_ptr[x_walls][y_walls + 1] == 1) {
			frontWall = 1;
		}
		if (*walls_ptr[x_walls + 1][y_walls] == 1) {
			rightWall = 1;
		}
		if (*walls_ptr[x_walls - 1][y_walls] == 1) {
			leftWall = 1;
		}
	} else if (orient == 1) {
		if (*walls_ptr[x_walls + 1][y_walls] == 1) {
			frontWall = 1;
		}
		if (*walls_ptr[x_walls][y_walls - 1] == 1) {
			rightWall = 1;
		}
		if (*walls_ptr[x_walls][y_walls + 1] == 1) {
			leftWall = 1;
		}
	} else if (orient == 2) {
		if (*walls_ptr[x_walls][y_walls - 1] == 1) {
			frontWall = 1;
		}
		if (*walls_ptr[x_walls - 1][y_walls] == 1) {
			rightWall = 1;
		}
		if (*walls_ptr[x_walls + 1][y_walls] == 1) {
			leftWall = 1;
		}
	} else { // orient == 3
		if (*walls_ptr[x_walls - 1][y_walls] == 1) {
			frontWall = 1;
		}
		if (*walls_ptr[x_walls][y_walls + 1] == 1) {
			rightWall = 1;
		}
		if (*walls_ptr[x_walls][y_walls - 1] == 1) {
			leftWall = 1;
		}
	}

}

void get_minimum_accesible_neighbour() {
	accesible_1 = 1000;
	accesible_2 = 1000;
	accesible_3 = 1000;
	accesible_4 = 1000;

	int8_t x_in_wall_map;
	int8_t y_in_wall_map;
	x_in_wall_map = ((flood_x - 1) * 2) + 1;
	y_in_wall_map = ((flood_y - 1) * 2) + 1;

	if (*walls_ptr[x_in_wall_map][y_in_wall_map + 1] == 0) { //0
		accesible_1 = *maze_value_ptr[flood_x][flood_y + 1];
	}
	if (*walls_ptr[x_in_wall_map + 1][y_in_wall_map] == 0) { //1
		accesible_2 = *maze_value_ptr[flood_x + 1][flood_y];
	}
	if (*walls_ptr[x_in_wall_map][y_in_wall_map - 1] == 0) { //2
		accesible_3 = *maze_value_ptr[flood_x][flood_y - 1];
	}
	if (*walls_ptr[x_in_wall_map - 1][y_in_wall_map] == 0) { //3
		accesible_4 = *maze_value_ptr[flood_x - 1][flood_y];
	}

	if (accesible_1 <= accesible_2 && accesible_1 <= accesible_3
			&& accesible_1 <= accesible_4) {
		minimum_accesible_value = accesible_1;
	} else if (accesible_2 <= accesible_1 && accesible_2 <= accesible_3
			&& accesible_2 <= accesible_4) {
		minimum_accesible_value = accesible_2;
	} else if (accesible_3 <= accesible_1 && accesible_3 <= accesible_2
			&& accesible_3 <= accesible_4) {
		minimum_accesible_value = accesible_3;
	} else if (accesible_4 <= accesible_1 && accesible_4 <= accesible_2
			&& accesible_4 <= accesible_3) {
		minimum_accesible_value = accesible_4;
	}

}

void update_coordinate() {
	if (orient == 0) {
		y++;
	} else if (orient == 1) {
		x++;
	} else if (orient == 2) {
		y--;
	} else { // orient == 3
		x--;
	}
//SSD1306_printValue(0, 0, x);
//SSD1306_printValue(30, 0, y);
}

void update_walls() {
//int8_t x_walls; //x_in_wall_map
//int8_t y_walls; //y_in_wall_map
	uint8_t x_walls = ((x - 1) * 2) + 1;
	uint8_t y_walls = ((y - 1) * 2) + 1;

	if (orient == 0) {
		if (frontWall == 1) {
			*walls_ptr[x_walls][y_walls + 1] = 1;
		}
		if (rightWall == 1) {
			*walls_ptr[x_walls + 1][y_walls] = 1;
		}
		if (leftWall == 1) {
			*walls_ptr[x_walls - 1][y_walls] = 1;
		}
	} else if (orient == 1) {
		if (frontWall == 1) {
			*walls_ptr[x_walls + 1][y_walls] = 1;
		}
		if (rightWall == 1) {
			*walls_ptr[x_walls][y_walls - 1] = 1;
		}
		if (leftWall == 1) {
			*walls_ptr[x_walls][y_walls + 1] = 1;
		}
	} else if (orient == 2) {
		if (frontWall == 1) {
			*walls_ptr[x_walls][y_walls - 1] = 1;
		}
		if (rightWall == 1) {
			*walls_ptr[x_walls - 1][y_walls] = 1;
		}
		if (leftWall == 1) {
			*walls_ptr[x_walls + 1][y_walls] = 1;
		}
	} else { // orient == 3
		if (frontWall == 1) {
			*walls_ptr[x_walls - 1][y_walls] = 1;
		}
		if (rightWall == 1) {
			*walls_ptr[x_walls][y_walls + 1] = 1;
		}
		if (leftWall == 1) {
			*walls_ptr[x_walls][y_walls - 1] = 1;
		}
	}
}

/*
 * 0 normal foreward
 *  4 normal_foreward_follow
 * 1 left follow
 * 2 center follow
 * 3 right follow*/
void advance(int mode, uint16_t dis) {

	if (mode == -1) {
		leftStep = TIM1->CNT;
		rightStep = TIM2->CNT;
		uint16_t Startstep = (leftStep + rightStep) / 2;
		while (((leftStep + rightStep) / 2) - Startstep < dis) {
			leftStep = TIM1->CNT;
			rightStep = TIM2->CNT;
			mdrive(-leftBaseSpeed, -rightBaseSpeed);
		}
		mdrive(0, 0);
	}
	if (mode == 0) {
		leftStep = TIM1->CNT;
		rightStep = TIM2->CNT;
		uint16_t Startstep = (leftStep + rightStep) / 2;
		while (((leftStep + rightStep) / 2) - Startstep < dis) {
			leftStep = TIM1->CNT;
			rightStep = TIM2->CNT;
			mdrive(leftBaseSpeed, rightBaseSpeed);

		}
		mdrive(0, 0);
	}
	if (mode == 4) {
		leftStep = TIM1->CNT;
		rightStep = TIM2->CNT;
		uint16_t Startstep = (leftStep + rightStep) / 2;
		while (((leftStep + rightStep) / 2) - Startstep < dis) {
			leftStep = TIM1->CNT;
			rightStep = TIM2->CNT;

			raedFrontSensors();
			if (front1 < 1000 && front2 < 1000) {
				mdrive(leftBaseSpeed, rightBaseSpeed);
			} else if (front1 > 1000) {
				mdrive(leftBaseSpeed, 0);
			} else if (front2 > 1000) {
				mdrive(0, rightBaseSpeed);
			} else {
				mdrive(leftBaseSpeed, rightBaseSpeed);
			}

			raedSideSensors();
			if (left < 600 && right < 600) {
				mdrive(leftBaseSpeed, rightBaseSpeed);
			} else if (left > 600) {
				followWallLeft();
			} else if (right > 600) {
				followWallRight();
			} else {
				mdrive(leftBaseSpeed, rightBaseSpeed);
			}
		}
		mdrive(0, 0);
	}
	if (mode == 1) {
		leftStep = TIM1->CNT;
		rightStep = TIM2->CNT;
		uint16_t Startstep = (leftStep + rightStep) / 2;
		while (((leftStep + rightStep) / 2) - Startstep < dis) {
			leftStep = TIM1->CNT;
			rightStep = TIM2->CNT;
			followWallLeft();
		}
		mdrive(0, 0);
	}
	if (mode == 2) {
		leftStep = TIM1->CNT;
		rightStep = TIM2->CNT;
		uint16_t Startstep = (leftStep + rightStep) / 2;
		while (((leftStep + rightStep) / 2) - Startstep < dis) {
			leftStep = TIM1->CNT;
			rightStep = TIM2->CNT;
			followWallCenter();
		}
		mdrive(0, 0);
	}
	if (mode == 3) {
		leftStep = TIM1->CNT;
		rightStep = TIM2->CNT;
		uint16_t Startstep = (leftStep + rightStep) / 2;
		while (((leftStep + rightStep) / 2) - Startstep < dis) {
			leftStep = TIM1->CNT;
			rightStep = TIM2->CNT;
			followWallRight();
		}
		mdrive(0, 0);
	}
}

float kp_CFollow = 1.25;
float kd_CFollow = 3;
int lastErrorCFollow;
void followWallCenter(void) {
	raedSideSensors();
	int error = left + 100 - right;
	int P = error;
	int D = error - lastErrorCFollow;
	int output = kp_CFollow * P + kd_CFollow * D;
	lastErrorCFollow = error;
	mdrive(leftBaseSpeed + output, leftBaseSpeed - output);
}

float kp_LFollow = 2.5;
float kd_LFollow = 3;
int lastErrorLFollow;

void followWallLeft(void) {
	raedSideSensors();
	int error = left - 800;
	int P = error;
	int D = error - lastErrorLFollow;
	int output = kp_LFollow * P + kd_LFollow * D;
	lastErrorLFollow = error;
	mdrive(leftBaseSpeed + output, leftBaseSpeed - output);
}

float kp_RFollow = 2.5;
float kd_RFollow = 3;
int lastErrorRFollow;
void followWallRight(void) {
	raedSideSensors();
	int error = right - 1000;
	int P = error;
	int D = error - lastErrorRFollow;
	int output = kp_RFollow * P + kd_RFollow * D;
	lastErrorRFollow = error;
	mdrive(leftBaseSpeed - output, leftBaseSpeed + output);
}

void alignFront(void) {
	for (int i = 0; i < 140; i++) {
		alignFrontAngle();
		alignFrontDistance();
	}
	mdrive(0, 0);
}

float kp_wallC = 20;
float kd_wallC = 4;
int lastErrorAngle;
void alignFrontAngle(void) {
	raedFrontSensors();
	int error = front1 - front2;
	int P = error;
	int D = error - lastErrorAngle;
	int output = kp_wallC * P + kd_wallC * D;
	lastErrorAngle = error;
	mdrive(-output, output);
}

float kp_dis = 15;
float kd_dis = 0.3;
int lastErrorDis;
void alignFrontDistance(void) {
	raedFrontSensors();
	int error = 3790 - ((front1 + front2) / 2);
	int P = error;
	int D = error - lastErrorDis;
	int output = kp_dis * P + kd_dis * D;
	lastErrorDis = error;
	mdrive(output, output);
}

void raedFrontSensors(void) {
	sConfigPrivate.Rank = ADC_REGULAR_RANK_1;
	sConfigPrivate.SamplingTime = ADC_SAMPLETIME_1CYCLE_5;
	HAL_GPIO_WritePin(front1EmitterPort, front1EmitterPin, 1);
	HAL_GPIO_WritePin(front2EmitterPort, front2EmitterPin, 1);
	HAL_Delay(0);
	sConfigPrivate.Channel = ADC_CHANNEL_4;
	HAL_ADC_ConfigChannel(&hadc1, &sConfigPrivate);
	HAL_ADC_Start(&hadc1);
	HAL_ADC_PollForConversion(&hadc1, 10);
	front1 = HAL_ADC_GetValue(&hadc1);
	HAL_ADC_Stop(&hadc1);
	sConfigPrivate.Channel = ADC_CHANNEL_2;
	HAL_ADC_ConfigChannel(&hadc1, &sConfigPrivate);
	HAL_ADC_Start(&hadc1);
	HAL_ADC_PollForConversion(&hadc1, 10);
	front2 = HAL_ADC_GetValue(&hadc1);
	HAL_ADC_Stop(&hadc1);
	HAL_GPIO_WritePin(front1EmitterPort, front1EmitterPin, 0);
	HAL_GPIO_WritePin(front2EmitterPort, front2EmitterPin, 0);

//SSD1306_printValue(83, 24, front2);
//SSD1306_printValue(0, 24, front1);
}

void raedSideSensors(void) {
	sConfigPrivate.Rank = ADC_REGULAR_RANK_1;
	sConfigPrivate.SamplingTime = ADC_SAMPLETIME_1CYCLE_5;
	HAL_GPIO_WritePin(leftEmitterPort, leftEmitterPin, 1);
	HAL_GPIO_WritePin(rightEmitterPort, rightEmitterPin, 1);
	HAL_Delay(0);
	sConfigPrivate.Channel = ADC_CHANNEL_5;
	HAL_ADC_ConfigChannel(&hadc1, &sConfigPrivate);
	HAL_ADC_Start(&hadc1);
	HAL_ADC_PollForConversion(&hadc1, 10);
	left = HAL_ADC_GetValue(&hadc1);
	HAL_ADC_Stop(&hadc1);
	sConfigPrivate.Channel = ADC_CHANNEL_3;
	HAL_ADC_ConfigChannel(&hadc1, &sConfigPrivate);
	HAL_ADC_Start(&hadc1);
	HAL_ADC_PollForConversion(&hadc1, 10);
	right = HAL_ADC_GetValue(&hadc1);
	HAL_ADC_Stop(&hadc1);
	HAL_GPIO_WritePin(leftEmitterPort, leftEmitterPin, 0);
	HAL_GPIO_WritePin(rightEmitterPort, rightEmitterPin, 0);

//SSD1306_printValue(80, 0, right);
//SSD1306_printValue(10, 0, left);
}

int32_t Angle;
int32_t startAngle;
int lastErrorTurn;
float kp_turn_right = 45;
float kd_turn = 0;
#define right_tur_adjust 60
int error_turnRight = right_tur_adjust;

void turnRight(void) {
	raedFrontSensors();
	if ((front1 + front2) / 2 > 750) {
		alignFront();
	} else {
		mdrive(0, 0);
	}
	MPU6050_Read_All(&hi2c1, &MPU6050);
	startAngle = MPU6050.KalmanAngleZ;
	for (int i = 0; i < 1100; i++) {
		MPU6050_Read_All(&hi2c1, &MPU6050);
		Angle = MPU6050.KalmanAngleZ;
		error_turnRight = (startAngle + right_tur_adjust) - Angle;
		int P = error_turnRight;
		int D = error_turnRight - lastErrorTurn;
		int output = kp_turn_right * P + kd_turn * D;
		lastErrorTurn = error_turnRight;
		mdrive(output, -output);
	}
	mdrive(0, 0);

	error_turnRight = right_tur_adjust;
	orient++;
	if (orient == 4) {
		orient = 0;
	}
}
/*
 float kp_turn_right_smooth = 80; //40;//60; //110;
 #define right_tur_adjust_smooth 89//27

 void turnRight_smooth(void) {

 MPU6050_Read_All(&hi2c1, &MPU6050);
 startAngle = MPU6050.KalmanAngleZ;
 for (int i = 0; i < 1900; i++) {
 MPU6050_Read_All(&hi2c1, &MPU6050);
 Angle = MPU6050.KalmanAngleZ;
 error_turnRight = (startAngle + right_tur_adjust_smooth) - Angle;
 int P = error_turnRight;
 int D = error_turnRight - lastErrorTurn;
 int output = kp_turn_right_smooth * P + kd_turn * D;
 lastErrorTurn = error_turnRight;
 mdrive(output, 0);
 }
 mdrive(0, 0);
 error_turnRight = right_tur_adjust_smooth;
 orient++;
 if (orient == 4) {
 orient = 0;
 }
 }*/

float kp_turn_left = 40;
#define left_turn_adjust 60
int error_turnLeft = -left_turn_adjust;

void turnLeft(void) {
	raedFrontSensors();
	if ((front1 + front2) / 2 > 750) {
		alignFront();
	} else {
		mdrive(0, 0);
	}
	MPU6050_Read_All(&hi2c1, &MPU6050);
	startAngle = MPU6050.KalmanAngleZ;
	for (int i = 0; i < 1100; i++) {
		MPU6050_Read_All(&hi2c1, &MPU6050);
		Angle = MPU6050.KalmanAngleZ;
		error_turnLeft = (Angle * -1) + (startAngle - left_turn_adjust);
		int P = error_turnLeft;
		int D = error_turnLeft - lastErrorTurn;
		int output = kp_turn_left * P + kd_turn * D;
		lastErrorTurn = error_turnLeft;
		mdrive(output, -output);
	}
	mdrive(0, 0);
	error_turnLeft = -left_turn_adjust;
	orient--;
	if (orient == -1) {
		orient = 3;
	}
}

/*
 float kp_turn_left_smooth = 50;
 #define left_turn_adjust_smooth 90
 void turnLeft_smooth(void) {
 raedFrontSensors();

 MPU6050_Read_All(&hi2c1, &MPU6050);
 startAngle = MPU6050.KalmanAngleZ;
 for (int i = 0; i < 1800; i++) {
 MPU6050_Read_All(&hi2c1, &MPU6050);
 Angle = MPU6050.KalmanAngleZ;
 error_turnLeft = (Angle * -1) + (startAngle - left_turn_adjust_smooth);
 int P = error_turnLeft;
 int D = error_turnLeft - lastErrorTurn;
 int output = kp_turn_left_smooth * P + kd_turn * D;
 lastErrorTurn = error_turnLeft;
 mdrive(0, -output * 2);
 }
 mdrive(0, 0);
 //advance(normal_foreward, 5);
 error_turnLeft = -left_turn_adjust_smooth;
 orient--;
 if (orient == -1) {
 orient = 3;
 }
 //SSD1306_printValue(80, 0, orient);
 }*/
void turnBack(void) {
	turnRight();
	turnRight();
}
void brake(int time) {
	mdrive(-leftBaseSpeed, -rightBaseSpeed);
	HAL_Delay(time);
	mdrive(0, 0);
}

void mdrive(int mleft, int mright) {
	if (mleft > 0) {
		if (mleft > 625) {
			mleft = 625;
		}
		__HAL_TIM_SET_COMPARE(&htim3, TIM_CHANNEL_4, mleft); // left front
		__HAL_TIM_SET_COMPARE(&htim3, TIM_CHANNEL_1, 0); //left bac
	} else if (mleft < 0) {
		if (mleft < -625) {
			mleft = -625;
		}
		__HAL_TIM_SET_COMPARE(&htim3, TIM_CHANNEL_4, 0); // left front
		__HAL_TIM_SET_COMPARE(&htim3, TIM_CHANNEL_1, -1 * mleft); //left back
	} else {
		__HAL_TIM_SET_COMPARE(&htim3, TIM_CHANNEL_4, 0); // left front
		__HAL_TIM_SET_COMPARE(&htim3, TIM_CHANNEL_1, 0); //left back
	}
	if (mright > 0) {
		if (mright > 625) {
			mright = 625;
		}
		__HAL_TIM_SET_COMPARE(&htim3, TIM_CHANNEL_3, mright); //right front
		__HAL_TIM_SET_COMPARE(&htim3, TIM_CHANNEL_2, 0); //right back
	} else if (mright < 0) {
		if (mright < -625) {
			mright = -625;
		}
		__HAL_TIM_SET_COMPARE(&htim3, TIM_CHANNEL_3, 0); //right front
		__HAL_TIM_SET_COMPARE(&htim3, TIM_CHANNEL_2, -1 * mright); //right back
	} else {
		__HAL_TIM_SET_COMPARE(&htim3, TIM_CHANNEL_3, 0); //right front
		__HAL_TIM_SET_COMPARE(&htim3, TIM_CHANNEL_2, 0); //right back
	}
}

void push(uint8_t x, uint8_t y) {
	top++;
	stack_x[top] = x;
	stack_y[top] = y;
}

void pop() {
	flood_x = stack_x[top];
	flood_y = stack_y[top];
	top--;
}

/**

 * @brief System Clock Configuration
 * @retval None
 */
void SystemClock_Config(void) {
	RCC_OscInitTypeDef RCC_OscInitStruct = { 0 };
	RCC_ClkInitTypeDef RCC_ClkInitStruct = { 0 };
	RCC_PeriphCLKInitTypeDef PeriphClkInit = { 0 };

	/** Initializes the RCC Oscillators according to the specified parameters
	 * in the RCC_OscInitTypeDef structure.
	 */
	RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI;
	RCC_OscInitStruct.HSIState = RCC_HSI_ON;
	RCC_OscInitStruct.HSICalibrationValue = RCC_HSICALIBRATION_DEFAULT;
	RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
	RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSI_DIV2;
	RCC_OscInitStruct.PLL.PLLMUL = RCC_PLL_MUL16;
	if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK) {
		Error_Handler();
	}

	/** Initializes the CPU, AHB and APB buses clocks
	 */
	RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK | RCC_CLOCKTYPE_SYSCLK
			| RCC_CLOCKTYPE_PCLK1 | RCC_CLOCKTYPE_PCLK2;
	RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
	RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
	RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV2;
	RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;

	if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_2) != HAL_OK) {
		Error_Handler();
	}
	PeriphClkInit.PeriphClockSelection = RCC_PERIPHCLK_ADC;
	PeriphClkInit.AdcClockSelection = RCC_ADCPCLK2_DIV6;
	if (HAL_RCCEx_PeriphCLKConfig(&PeriphClkInit) != HAL_OK) {
		Error_Handler();
	}
}

static void MX_ADC1_Init(void) {

	/* USER CODE BEGIN ADC1_Init 0 */

	/* USER CODE END ADC1_Init 0 */

	ADC_ChannelConfTypeDef sConfig = { 0 };

	/* USER CODE BEGIN ADC1_Init 1 */

	/* USER CODE END ADC1_Init 1 */

	/** Common config
	 */
	hadc1.Instance = ADC1;
	hadc1.Init.ScanConvMode = ADC_SCAN_DISABLE;
	hadc1.Init.ContinuousConvMode = ENABLE;
	hadc1.Init.DiscontinuousConvMode = DISABLE;
	hadc1.Init.ExternalTrigConv = ADC_SOFTWARE_START;
	hadc1.Init.DataAlign = ADC_DATAALIGN_RIGHT;
	hadc1.Init.NbrOfConversion = 1;
	if (HAL_ADC_Init(&hadc1) != HAL_OK) {
		Error_Handler();
	}

	/** Configure Regular Channel
	 */
	sConfig.Channel = ADC_CHANNEL_2;
	sConfig.Rank = ADC_REGULAR_RANK_1;
	sConfig.SamplingTime = ADC_SAMPLETIME_1CYCLE_5;
	if (HAL_ADC_ConfigChannel(&hadc1, &sConfig) != HAL_OK) {
		Error_Handler();
	}
	/* USER CODE BEGIN ADC1_Init 2 */

	/* USER CODE END ADC1_Init 2 */

}

static void MX_TIM1_Init(void) {

	/* USER CODE BEGIN TIM1_Init 0 */

	/* USER CODE END TIM1_Init 0 */

	TIM_ClockConfigTypeDef sClockSourceConfig = { 0 };
	TIM_MasterConfigTypeDef sMasterConfig = { 0 };

	/* USER CODE BEGIN TIM1_Init 1 */

	/* USER CODE END TIM1_Init 1 */
	htim1.Instance = TIM1;
	htim1.Init.Prescaler = 0;
	htim1.Init.CounterMode = TIM_COUNTERMODE_UP;
	htim1.Init.Period = 65535;
	htim1.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
	htim1.Init.RepetitionCounter = 0;
	htim1.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
	if (HAL_TIM_Base_Init(&htim1) != HAL_OK) {
		Error_Handler();
	}
	sClockSourceConfig.ClockSource = TIM_CLOCKSOURCE_ETRMODE2;
	sClockSourceConfig.ClockPolarity = TIM_CLOCKPOLARITY_NONINVERTED;
	sClockSourceConfig.ClockPrescaler = TIM_CLOCKPRESCALER_DIV1;
	sClockSourceConfig.ClockFilter = 15;
	if (HAL_TIM_ConfigClockSource(&htim1, &sClockSourceConfig) != HAL_OK) {
		Error_Handler();
	}
	sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
	sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
	if (HAL_TIMEx_MasterConfigSynchronization(&htim1, &sMasterConfig)
			!= HAL_OK) {
		Error_Handler();
	}
	/* USER CODE BEGIN TIM1_Init 2 */

	/* USER CODE END TIM1_Init 2 */

}

static void MX_TIM2_Init(void) {

	/* USER CODE BEGIN TIM2_Init 0 */

	/* USER CODE END TIM2_Init 0 */

	TIM_ClockConfigTypeDef sClockSourceConfig = { 0 };
	TIM_MasterConfigTypeDef sMasterConfig = { 0 };

	/* USER CODE BEGIN TIM2_Init 1 */

	/* USER CODE END TIM2_Init 1 */
	htim2.Instance = TIM2;
	htim2.Init.Prescaler = 0;
	htim2.Init.CounterMode = TIM_COUNTERMODE_UP;
	htim2.Init.Period = 65535;
	htim2.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
	htim2.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
	if (HAL_TIM_Base_Init(&htim2) != HAL_OK) {
		Error_Handler();
	}
	sClockSourceConfig.ClockSource = TIM_CLOCKSOURCE_ETRMODE2;
	sClockSourceConfig.ClockPolarity = TIM_CLOCKPOLARITY_NONINVERTED;
	sClockSourceConfig.ClockPrescaler = TIM_CLOCKPRESCALER_DIV1;
	sClockSourceConfig.ClockFilter = 15;
	if (HAL_TIM_ConfigClockSource(&htim2, &sClockSourceConfig) != HAL_OK) {
		Error_Handler();
	}
	sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
	sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
	if (HAL_TIMEx_MasterConfigSynchronization(&htim2, &sMasterConfig)
			!= HAL_OK) {
		Error_Handler();
	}
	/* USER CODE BEGIN TIM2_Init 2 */

	/* USER CODE END TIM2_Init 2 */

}

static void MX_TIM3_Init(void) {

	/* USER CODE BEGIN TIM3_Init 0 */

	/* USER CODE END TIM3_Init 0 */

	TIM_ClockConfigTypeDef sClockSourceConfig = { 0 };
	TIM_MasterConfigTypeDef sMasterConfig = { 0 };
	TIM_OC_InitTypeDef sConfigOC = { 0 };

	/* USER CODE BEGIN TIM3_Init 1 */

	/* USER CODE END TIM3_Init 1 */
	htim3.Instance = TIM3;
	htim3.Init.Prescaler = 127;
	htim3.Init.CounterMode = TIM_COUNTERMODE_UP;
	htim3.Init.Period = 625;
	htim3.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
	htim3.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
	if (HAL_TIM_Base_Init(&htim3) != HAL_OK) {
		Error_Handler();
	}
	sClockSourceConfig.ClockSource = TIM_CLOCKSOURCE_INTERNAL;
	if (HAL_TIM_ConfigClockSource(&htim3, &sClockSourceConfig) != HAL_OK) {
		Error_Handler();
	}
	if (HAL_TIM_PWM_Init(&htim3) != HAL_OK) {
		Error_Handler();
	}
	sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
	sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
	if (HAL_TIMEx_MasterConfigSynchronization(&htim3, &sMasterConfig)
			!= HAL_OK) {
		Error_Handler();
	}
	sConfigOC.OCMode = TIM_OCMODE_PWM1;
	sConfigOC.Pulse = 0;
	sConfigOC.OCPolarity = TIM_OCPOLARITY_HIGH;
	sConfigOC.OCFastMode = TIM_OCFAST_DISABLE;
	if (HAL_TIM_PWM_ConfigChannel(&htim3, &sConfigOC, TIM_CHANNEL_1)
			!= HAL_OK) {
		Error_Handler();
	}
	if (HAL_TIM_PWM_ConfigChannel(&htim3, &sConfigOC, TIM_CHANNEL_2)
			!= HAL_OK) {
		Error_Handler();
	}
	if (HAL_TIM_PWM_ConfigChannel(&htim3, &sConfigOC, TIM_CHANNEL_3)
			!= HAL_OK) {
		Error_Handler();
	}
	if (HAL_TIM_PWM_ConfigChannel(&htim3, &sConfigOC, TIM_CHANNEL_4)
			!= HAL_OK) {
		Error_Handler();
	}
	/* USER CODE BEGIN TIM3_Init 2 */

	/* USER CODE END TIM3_Init 2 */
	HAL_TIM_MspPostInit(&htim3);

}

static void MX_GPIO_Init(void) {
	GPIO_InitTypeDef GPIO_InitStruct = { 0 };
	/* USER CODE BEGIN MX_GPIO_Init_1 */
	/* USER CODE END MX_GPIO_Init_1 */

	/* GPIO Ports Clock Enable */
	__HAL_RCC_GPIOD_CLK_ENABLE();
	__HAL_RCC_GPIOA_CLK_ENABLE();
	__HAL_RCC_GPIOB_CLK_ENABLE();

	/*Configure GPIO pin Output Level */
	HAL_GPIO_WritePin(GPIOB,
	GPIO_PIN_10 | GPIO_PIN_11 | GPIO_PIN_12 | GPIO_PIN_15, GPIO_PIN_RESET);

	/*Configure GPIO pins : PB10 PB11 PB12 PB15 */
	GPIO_InitStruct.Pin = GPIO_PIN_10 | GPIO_PIN_11 | GPIO_PIN_12 | GPIO_PIN_15;
	GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
	GPIO_InitStruct.Pull = GPIO_NOPULL;
	GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
	HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

	/* USER CODE BEGIN MX_GPIO_Init_2 */
	/* USER CODE END MX_GPIO_Init_2 */
}

/* USER CODE END 4 */

/**
 * @brief  This function is executed in case of error occurrence.
 * @retval None
 */
void Error_Handler(void) {
	/* USER CODE BEGIN Error_Handler_Debug */
	/* User can add his own implementation to report the HAL error return state */
	__disable_irq();
	while (1) {
	}
	/* USER CODE END Error_Handler_Debug */
}

#ifdef  USE_FULL_ASSERT
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
