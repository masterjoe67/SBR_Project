/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
  ******************************************************************************
  * @attention
  *
  * <h2><center>&copy; Copyright (c) 2023 STMicroelectronics.
  * All rights reserved.</center></h2>
  *
  * This software component is licensed by ST under BSD 3-Clause license,
  * the "License"; You may not use this file except in compliance with the
  * License. You may obtain a copy of the License at:
  *                        opensource.org/licenses/BSD-3-Clause
  *
  ******************************************************************************
  */
/* USER CODE END Header */
/* Includes ------------------------------------------------------------------*/
#include "main.h"
#include <string.h>
#include "mpu6050.h"
#include "ssd1306.h"
#include "PID.h"
#include "stdio.h"
#include "EEPROM.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */

#define PPR   800
// #define TICKS_PER_SECOND  40000 // 40kHz
#define TICKS_PER_SECOND  50000 // 50kHz
#define PULSE_WIDTH 1





#define VELOCITY_Kp  0.007
#define VELOCITY_Kd  0.0
#define VELOCITY_Ki  0.0005
#define WARMUP_DELAY_US (5000000UL)
#define DEG_TO_RAD  0.017453292519943295769236907684886
#define RAD_TO_DEG  57.295779513082320876798154814105
#define ANGLE_SET_POINT (2.0 * DEG_TO_RAD)
#define constrain(amt,low,high) ((amt)<(low)?(low):((amt)>(high)?(high):(amt)))

#define MAX_ACCEL (200)
#define MAX_ANGLE (30 * DEG_TO_RAD)
#define ANGLE_Kp  450.0
#define ANGLE_Ki  50.0
#define ANGLE_Kd  10.0


#define MAX_PACKET_SIZE 96
#define DIVISOR 10000.0
#define MOTOR_REVERSE 1

#define REFdebounce 150
uint16_t In1_0, In1_1 = 0;
bool btn1status = 0;

/* Private variables ---------------------------------------------------------*/
CRC_HandleTypeDef hcrc;

I2C_HandleTypeDef hi2c1;
I2C_HandleTypeDef hi2c2;
DMA_HandleTypeDef hdma_i2c2_rx;
DMA_HandleTypeDef hdma_i2c2_tx;
SPI_HandleTypeDef hspi1;
TIM_HandleTypeDef htim3;
UART_HandleTypeDef huart2;
DMA_HandleTypeDef hdma_usart2_tx;
DMA_HandleTypeDef hdma_usart2_rx;

/* USER CODE BEGIN PV */
int16_t acc_x, acc_y, acc_z, gyr_x, gyr_y, gyr_z;
float ax, ay, az, gx, gy, gz, temperature, roll, pitch;
uint8_t buffer[128];

volatile bool i2c2_transmit_complete = false;
volatile bool i2c2_receive_complete = false;

volatile bool mpuInterrupt = false;

uint8_t fifoBuffer[64];

Quaternion q; // [w, x, y, z]         quaternion container
VectorFloat gravity; // [x, y, z]            gravity vector
float ypr[3]; // [yaw, pitch, roll]   yaw/pitch/roll container and gravity vector

PID anglePID(ANGLE_Kp, ANGLE_Kd, ANGLE_Ki, ANGLE_SET_POINT);
PID velocityPID(VELOCITY_Kp, VELOCITY_Kd, VELOCITY_Ki, 0.0);

float pid_settings[6] = {
	ANGLE_Kp,
	ANGLE_Kd,
	ANGLE_Ki,
	VELOCITY_Kp,
	VELOCITY_Kd,
	VELOCITY_Ki
};

volatile unsigned long currentTickLeft = 0UL;
volatile unsigned long currentTickRight = 0UL;
volatile unsigned long ticksPerPulseLeft = (unsigned long)UINT64_MAX;
volatile unsigned long ticksPerPulseRight = (unsigned long)UINT64_MAX;
volatile float accel = 0.0;
volatile float velocity = 0.0;

float joystick[2] = { 0.0f, 0.0f };
float ref_velocity = 0.0f;
float ref_steering = 0.0f;
float steering = 0.0f;

float angle = 0.0;
float targetAngle = ANGLE_SET_POINT;

float targetVelocity = 0.0;

bool isBalancing = false;

char packet[MAX_PACKET_SIZE];
uint8_t packet_size = 0;

uint8_t logMode = 0;

/*uint8_t rxBuffer[64];
uint8_t rxByte;
uint16_t rxIdx;
uint8_t rxFlag;*/

#define CIRC_BUF_SZ       64  /* must be power of two */
static uint8_t rx_dma_circ_buf[CIRC_BUF_SZ];
#define DMA_WRITE_PTR ( (CIRC_BUF_SZ - huart2.hdmarx->Instance->CNDTR) & (CIRC_BUF_SZ - 1) )
static uint32_t rd_ptr;
/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_DMA_Init(void);
static void MX_I2C1_Init(void);
static void MX_SPI1_Init(void);
static void MX_I2C2_Init(void);
static void MX_USART2_UART_Init(void);
static void MX_CRC_Init(void);
static void MX_TIM3_Init(void);

uint32_t getCurrentMicros(void)
{
	uint32_t m0 = HAL_GetTick();
	__IO uint32_t u0 = SysTick->VAL;
	uint32_t m1 = HAL_GetTick();
	__IO uint32_t u1 = SysTick->VAL;
	const uint32_t tms = SysTick->LOAD + 1;

	if (m1 != m0) {
		return (m1 * 1000 + ((tms - u1) * 1000) / tms);
	}
	else {
		return (m0 * 1000 + ((tms - u0) * 1000) / tms);
	}
}

void UART2_SendString(char* s)
{
	HAL_UART_Transmit(&huart2, (uint8_t*)s, strlen(s), 1000);
}

void send_float_array(float *a, uint8_t size) {
	char MSG[40] = { '\0' };
	for (int i = 0; i < size; i++) {
		sprintf(MSG, "%.2ld", (long)(a[i] * DIVISOR));
		//Serial.print((long)(a[i] * DIVISOR));
		UART2_SendString(MSG);
		if (i < size - 1) {
			MSG[0] = ';';
			MSG[1] = '\0';
			UART2_SendString(MSG);
			//Serial.print(';');
		}
	}
	//Serial.print("\r\n");
	MSG[0] = '\r';
	MSG[1] = '\n';
	MSG[2] = '\0';
	UART2_SendString(MSG);
}

void logDisplay(unsigned long nowMicros) { 
	char MSG[40] = { '\0' };
	static unsigned long timestamp = getCurrentMicros();  
	if (nowMicros - timestamp < 10000 /* 100Hz */) {
		return;
	}
	static uint8_t oldLogMode;
	if (logMode != oldLogMode) {
		ssd1306_Clear();
		oldLogMode = logMode;
	}
	
	HAL_GPIO_TogglePin(GPIOE, LD3_Pin);
	
	switch (logMode) {
	case 0:
		sprintf(MSG, "Angle: %.2f", angle * RAD_TO_DEG);
		ssd1306_SetCursor(0, 0);
		ssd1306_WriteString(MSG, Font_7x10);
	
		sprintf(MSG, "TAngle: %.2f", targetAngle * RAD_TO_DEG);
		ssd1306_SetCursor(0, 10);
		ssd1306_WriteString(MSG, Font_7x10);
	
		sprintf(MSG, "Velocity: %.3f", ref_velocity);
		ssd1306_SetCursor(0, 20);
		ssd1306_WriteString(MSG, Font_7x10);
	
		sprintf(MSG, "Steering: %.3f", ref_steering);
		ssd1306_SetCursor(0, 30);
		ssd1306_WriteString(MSG, Font_7x10);
		break;
	case 1:
		sprintf(MSG, "AKp: %.3f", pid_settings[0]);
		ssd1306_SetCursor(0, 0);
		ssd1306_WriteString(MSG, Font_7x10);
	
		sprintf(MSG, "AKi: %0.3f", pid_settings[1]);
		ssd1306_SetCursor(0, 10);
		ssd1306_WriteString(MSG, Font_7x10);
		
		sprintf(MSG, "AKd: %0.3f", pid_settings[2]);
		ssd1306_SetCursor(0, 20);
		ssd1306_WriteString(MSG, Font_7x10);
		
		sprintf(MSG, "VKp: %0.3f", pid_settings[3]);
		ssd1306_SetCursor(0, 30);
		ssd1306_WriteString(MSG, Font_7x10);
		
		sprintf(MSG, "VKi: %0.3f", pid_settings[4]);
		ssd1306_SetCursor(0, 40);
		ssd1306_WriteString(MSG, Font_7x10);
		
		sprintf(MSG, "VKd: %0.3f", pid_settings[5]);
		ssd1306_SetCursor(0, 50);
		ssd1306_WriteString(MSG, Font_7x10);
		
		break;
	}
	
	//ssd1306_Clear();
	
	

	
	
	
	/*ssd1306_UpdateScreen();
	//timestamp = nowMicros;   
	
	printf("accel: (%8.4f, %8.4f, %8.4f)     gyro: (%12.4f, %12.4f, %12.4f)\r\n",
		((s32)ax) * 16.0 / 32768,
		((s32)ay) * 16.0 / 32768,
		((s32)az) * 16.0 / 32768,
		((s32)gx) * 2000.0 / 32768,
		((s32)gy) * 2000.0 / 32768,
		((s32)gz) * 2000.0 / 32768);*/
	
	/*float t = (float)(targetAngle * RAD_TO_DEG);
	sprintf(MSG, "a0: %.4f", t);
	ssd1306_SetCursor(0, 0);
	ssd1306_WriteString(MSG, Font_7x10);
	
	t = (float)(angle * RAD_TO_DEG);
	sprintf(MSG, "ta: %.4f", t);
	ssd1306_SetCursor(0, 10);
	ssd1306_WriteString(MSG, Font_7x10);
	
	sprintf(MSG, "tv: %.4f", velocity);
	ssd1306_SetCursor(0, 20);
	ssd1306_WriteString(MSG, Font_7x10);
	
	sprintf(MSG, "tu: %.4f", accel);
	ssd1306_SetCursor(0, 30);
	ssd1306_WriteString(MSG, Font_7x10);
	ssd1306_UpdateScreen();
	timestamp = nowMicros;*/ 
	timestamp = nowMicros;
	ssd1306_UpdateScreen();
}

void initMPU() {
	const int16_t accel_offset[3] = { -1262, -307, 1897 };
	const int16_t gyro_offset[3] = { 23, -41, 49 };
	
	uint8_t curY = ssd1306_GetCursorY() + 10;

	if (MPU6050_Init(&hi2c2)) {
		//ssd1306_Clear();
		ssd1306_SetCursor(0, curY);
		ssd1306_WriteString("MPU: OK", Font_7x10);
		ssd1306_UpdateScreen();
		
	}
	else {
		ssd1306_Clear();
		ssd1306_SetCursor(0, curY);
		ssd1306_WriteString("MPU: FAULT", Font_7x10);
		ssd1306_UpdateScreen();
		while (1)
		{}
	}
	curY += 10;
	HAL_Delay(1000);
	
	uint8_t count = 0;
	while (1)
	{
	         
		if (!MPU6050_dmpInitialize()) {
			ssd1306_SetCursor(0, curY);
			ssd1306_WriteString("DMP: OK", Font_7x10);
			ssd1306_UpdateScreen();
			break;
		}
		else {
			count++;
			if (count > 4) {
				ssd1306_SetCursor(0, curY);
				ssd1306_WriteString("DMP: FAULT SYSTEM HALTED", Font_7x10);
				ssd1306_UpdateScreen();
				while (1) {}
			}
			ssd1306_SetCursor(0, curY);
			ssd1306_WriteString("DMP: FAULT retry ", Font_7x10);
			ssd1306_UpdateScreen();
			HAL_Delay(1000);
			
		}
	}
	
	HAL_Delay(1000);


	//MPU6050_dmpInitialize();

	/*mpu.setXGyroOffset(gyro_offset[0]);
	mpu.setYGyroOffset(gyro_offset[1]);
	mpu.setZGyroOffset(gyro_offset[2]);
	mpu.setXAccelOffset(accel_offset[0]);
	mpu.setYAccelOffset(accel_offset[1]);
	mpu.setZAccelOffset(accel_offset[2]);*/

	MPU6050_setDMPEnabled(true);
	/* EXTI interrupt init*/
	HAL_NVIC_SetPriority(EXTI9_5_IRQn, 0, 0);
	HAL_NVIC_EnableIRQ(EXTI9_5_IRQn);
}

bool mpuUpdate() {
	if (mpuInterrupt && MPU6050_GetCurrentFIFOPacket(fifoBuffer, dmpPacketSize)) {
		MPU6050_dmpGetQuaternion(&q, fifoBuffer);
		MPU6050_dmpGetGravity(&gravity, &q);
		MPU6050_dmpGetYawPitchRoll(ypr, &q, &gravity);
		mpuInterrupt = false;
		return true;
	}
	return false;
}

unsigned long getTicksPerPulse(float velocity) {
	if (abs(velocity) < 1e-3) {
		// TODO: disable motor
		return (unsigned long)UINT64_MAX ;
	}
	else {
		return (uint64_t)(2.0 * M_PI * TICKS_PER_SECOND / (abs(velocity) * PPR)) - PULSE_WIDTH;
	}
}
void updateVelocity(unsigned long nowMicros) {
	// static unsigned long counter = 0;
	// static unsigned long sum = 0;

	static unsigned long timestamp = getCurrentMicros();
	if (nowMicros - timestamp < 100 /* 10kHz */) {
		return;
	}

	// sum += (nowMicros - timestamp);
	// counter++;
	// if (counter >= 1000) {
	//   Serial.println(((float)(sum)) / counter);
	//   counter = 0;
	//   sum = 0;
	// }

	float dt = ((float)(nowMicros - timestamp)) * 1e-6;
	velocity += accel * dt;
  
	float leftVelocity = velocity - steering;
	float rightVelocity = velocity + steering;
	ticksPerPulseLeft = getTicksPerPulse(leftVelocity);
	ticksPerPulseRight = getTicksPerPulse(rightVelocity);
#if MOTOR_REVERSE  
	if (leftVelocity > 0) {
		//digitalWrite(MOT_A_DIR, HIGH);   
		HAL_GPIO_WritePin(GPIOD, MOT_A_DIR_Pin, GPIO_PIN_SET);
		
	}
	else {
		HAL_GPIO_WritePin(GPIOD, MOT_A_DIR_Pin, GPIO_PIN_RESET);
		//digitalWrite(MOT_A_DIR, LOW);
	}

	if (rightVelocity > 0) {
		HAL_GPIO_WritePin(GPIOD, MOT_B_DIR_Pin, GPIO_PIN_RESET);
		//digitalWrite(MOT_B_DIR, LOW);  
	}
	else {
		HAL_GPIO_WritePin(GPIOD, MOT_B_DIR_Pin, GPIO_PIN_SET);
		//digitalWrite(MOT_B_DIR, HIGH);  
	}
#else
	if (leftVelocity > 0) {
		//digitalWrite(MOT_A_DIR, HIGH);   
		HAL_GPIO_WritePin(GPIOD, MOT_A_DIR_Pin, GPIO_PIN_RESET);
		
	}
	else {
		HAL_GPIO_WritePin(GPIOD, MOT_A_DIR_Pin, GPIO_PIN_SET);
		//digitalWrite(MOT_A_DIR, LOW);
	}

	if (rightVelocity > 0) {
		HAL_GPIO_WritePin(GPIOD, MOT_B_DIR_Pin, GPIO_PIN_SET);
		//digitalWrite(MOT_B_DIR, LOW);  
	}
	else {
		HAL_GPIO_WritePin(GPIOD, MOT_B_DIR_Pin, GPIO_PIN_RESET);
		//digitalWrite(MOT_B_DIR, HIGH);  
	}
#endif


	timestamp = nowMicros;
}

float absF(float value)
{
	return value < 0 ? -value : value;
}



void updateControl(unsigned long nowMicros) {
	/* Wait until IMU filter will settle */
	if (nowMicros < WARMUP_DELAY_US) {
		return;
	}

	static unsigned long timestamp = getCurrentMicros();
	if (nowMicros - timestamp < 1000 /* 1kHz */) {
		return;
	}
	if (!mpuUpdate()) {
		return;
	}
	angle = ypr[1];

	float dt = ((float)(nowMicros - timestamp)) * 1e-6;
	float tmpangle = absF(angle - targetAngle);
	if (tmpangle < M_PI / 18) {
		isBalancing = true;
	}

	if (tmpangle > M_PI / 4) {
		isBalancing = false;
		accel = 0.0;
		velocity = 0.0;    
	}

	if (!isBalancing) {
		return;
	}
	targetAngle = -velocityPID.getControl(velocity, dt);
	//targetAngle = constrain(targetAngle, -MAX_ANGLE, MAX_ANGLE);
	anglePID.setTarget(targetAngle);

	accel = anglePID.getControl(angle, dt);
	accel = constrain(accel, -MAX_ACCEL, MAX_ACCEL);

	timestamp = nowMicros;
}

void initMotors() {

	HAL_GPIO_WritePin(GPIOD, MOT_A_STEP_Pin, GPIO_PIN_RESET);
	HAL_GPIO_WritePin(GPIOD, MOT_B_STEP_Pin, GPIO_PIN_RESET);
	HAL_GPIO_WritePin(GPIOD, MOT_ENABLE_Pin, GPIO_PIN_RESET);
}

void parse_float_array(char *p, uint8_t p_size, float *dest) {
	char buf[16];
	long value;
	uint8_t buf_size = 0;
	uint8_t index = 0;
	for (uint8_t i = 0; i < p_size; i++) {
		if ((p[i] >= '0' && p[i] <= '9') || p[i] == '+' || p[i] == '-') {
			buf[buf_size++] = p[i];
		}
		else if (p[i] == ';') {
			buf[buf_size] = '\0';
			buf_size = 0;
			value = atol(buf);
			dest[index++] = ((float)value) / DIVISOR;
		}
	}
	buf[buf_size] = '\0';
	value = atol(buf);
	dest[index] = ((float)value) / DIVISOR;
}

float parse_float(char *p, uint8_t p_size) {
	char buf[16];
	long value;
	uint8_t buf_size = 0;
	uint8_t index = 0;
	for (uint8_t i = 0; i < p_size; i++) {
		if ((p[i] >= '0' && p[i] <= '9') || p[i] == '+' || p[i] == '-') {
			buf[buf_size++] = p[i];
		}
	}
	buf[buf_size] = '\0';
	value = atol(buf);
	return ((float)value);
	//dest[index] = ((float)value) / DIVISOR;
}

void parse_settings(char *p, uint8_t p_size) {
	parse_float_array(p, p_size, pid_settings);
	anglePID.setSettings(pid_settings[0], pid_settings[1], pid_settings[2]);
	velocityPID.setSettings(pid_settings[3], pid_settings[4], pid_settings[5]);
}

void parse_control(char *p, uint8_t p_size) {
	parse_float_array(p, p_size, joystick);
	ref_velocity = joystick[1];
	ref_steering = joystick[0];
}

void handle_packet(char *p, uint8_t p_size) {
	switch (p[0]) {
	case 'r':
		send_float_array(pid_settings, 6);
		break;
	case 's':
		parse_settings(&p[1], p_size - 1);
		send_float_array(pid_settings, 6);
		break;
	case 'c':
		parse_control(&p[1], p_size - 1);
		//send_float_array(joystick, 2);
		break;
	case 'P':
		pid_settings[0] = parse_float(&p[1], p_size - 1);
		break;
	case 'I':
		pid_settings[1] = parse_float(&p[1], p_size - 1);
		break;
	case 'D':
		pid_settings[2] = parse_float(&p[1], p_size - 1);
		break;
	case 'p':
		pid_settings[3] = parse_float(&p[1], p_size - 1);
		break;
	case 'i':
		pid_settings[4] = parse_float(&p[1], p_size - 1);
		break;
	case 'd':
		pid_settings[5] = parse_float(&p[1], p_size - 1);
		break;
	}
}


static bool msgrx_circ_buf_is_empty(void) {
	if (rd_ptr == DMA_WRITE_PTR) {
		return true;
	}
	return false;
}

static uint8_t msgrx_circ_buf_get(void) {
	uint8_t c = 0;
	if (rd_ptr != DMA_WRITE_PTR) {
		c = rx_dma_circ_buf[rd_ptr++];
		rd_ptr &= (CIRC_BUF_SZ - 1);
	}
	return c;
}

void formatEEPROM() {
	EEPROM_PageErase(0);
	EEPROM_PageErase(1);
	uint8_t index = 0;
	
	EEPROM_Write_NUM(0, 0, EEPROM_VERSION);

	for (int i = 0; i < 6; i++) {
		index = (i * 5);
		EEPROM_Write_NUM(1, index, pid_settings[i]);
		
	}
	//ee24_write(0, data, 1, uint32_t timeout);
}

void readSettigs() {
	uint8_t index = 0;
	for (int i = 0; i < 6; i++) {
		index = (i * 5);
		pid_settings[i] = EEPROM_Read_NUM(1, index);	
	}
}

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
	MX_I2C1_Init();
	MX_SPI1_Init();
	MX_I2C2_Init();
	MX_USART2_UART_Init();
	MX_CRC_Init();
	MX_TIM3_Init();
	/* USER CODE BEGIN 2 */
	ssd1306_Init();
	ssd1306_FlipScreenVertically();
	ssd1306_Clear();
	ssd1306_SetColor(White);
	
	ssd1306_DrawRect(0, 0, ssd1306_GetWidth(), ssd1306_GetHeight());
	ssd1306_SetCursor(8, 20);
	ssd1306_WriteString("MPU6050", Font_16x26);
	ssd1306_UpdateScreen();
	HAL_Delay(2000);
	ssd1306_Clear();
	ssd1306_UpdateScreen();
	ssd1306_Clear();
	ssd1306_SetCursor(0, 0);
	ssd1306_WriteString("* FIRMWARE RESET *", Font_7x10);
	ssd1306_UpdateScreen();
	HAL_Delay(1000);
	ssd1306_Clear();

	//Init EEPROM
	if (EEPROM_isConnected())
	{
		float eeprom_version = EEPROM_Read_NUM(0, 0); 
		
		if (eeprom_version == EEPROM_VERSION) {
			ssd1306_SetCursor(0, 0);
			ssd1306_WriteString("EEPROM: OK  data: OK", Font_7x10);
			ssd1306_UpdateScreen();
			readSettigs();
			HAL_Delay(1000);
		}
		else {
			ssd1306_SetCursor(0, 0);
			ssd1306_WriteString("EEPROM: OK  data: INVALID", Font_7x10);
			ssd1306_SetCursor(0, 10);
			ssd1306_WriteString("EEPROM: formatting", Font_7x10);
			ssd1306_UpdateScreen();
			formatEEPROM();
			HAL_Delay(2000);
			
		}
	}
	else {
		ssd1306_SetCursor(0, 0);
		ssd1306_WriteString("EEPROM: FAULT", Font_7x10);
		ssd1306_UpdateScreen();
		HAL_Delay(2000);
	}
	initMPU();
	initMotors();
	/* USER CODE END 2 */
	HAL_TIM_Base_Start_IT(&htim3);

	HAL_NVIC_SetPriority(USART2_IRQn, 0, 0);
	HAL_NVIC_EnableIRQ(USART2_IRQn);
	HAL_UART_Receive_DMA(&huart2, rx_dma_circ_buf, CIRC_BUF_SZ);

	ssd1306_Clear();
	
	static unsigned long time = getCurrentMicros();
	/* Infinite loop */
	/* USER CODE BEGIN WHILE */
	while (1)
	{
		unsigned long now = getCurrentMicros();
		updateVelocity(now);
		updateControl(now);
		now = getCurrentMicros();  
		if (now - time < 300000 /* 100Hz */) {
			continue;
		}
		else {
			if (btn1status) {
				logMode++;
				if (logMode > 2) {
					logMode = 0;
				}
				HAL_GPIO_TogglePin(GPIOE, LD10_Pin);
				btn1status = 0;
			
			}
			time = now;	
		}
		
		
#ifdef LOGGING_ENABLED
		logDisplay(now);
#endif
		
#ifdef COUNT_LOOP
		static unsigned long last_ts = micros();
		static unsigned long  counter = 0;

		counter++;
		if (now - last_ts >= 1000000) {
			Serial.println(counter);
			counter = 0;
			last_ts = now;
		}
#endif
		
		while (!msgrx_circ_buf_is_empty())
		{
			uint8_t c = msgrx_circ_buf_get();
			//HAL_UART_Receive(&huart2, &c, 1, 1000);

			if (c == '\n') {
				continue;
			}
			else if (c == '\r') {
				handle_packet(packet, packet_size);
				packet_size = 0;
			}
			else {
				packet[packet_size++] = (uint8_t) c;
			}
		}
		static const float a = 0.99;
		targetVelocity = a * targetVelocity + (1.0 - a) * ref_velocity;
		velocityPID.setTarget(targetVelocity);

		steering = a * steering + (1.0 - a) * ref_steering;
	}
	/* USER CODE END 3 */
}

/**
  * @brief System Clock Configuration
  * @retval None
  */
void SystemClock_Config(void)
{
	RCC_OscInitTypeDef RCC_OscInitStruct = { 0 };
	RCC_ClkInitTypeDef RCC_ClkInitStruct = { 0 };
	RCC_PeriphCLKInitTypeDef PeriphClkInit = { 0 };

	/** Initializes the RCC Oscillators according to the specified parameters
	* in the RCC_OscInitTypeDef structure.
	*/
	RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI | RCC_OSCILLATORTYPE_HSE;
	RCC_OscInitStruct.HSEState = RCC_HSE_BYPASS;
	RCC_OscInitStruct.HSEPredivValue = RCC_HSE_PREDIV_DIV1;
	RCC_OscInitStruct.HSIState = RCC_HSI_ON;
	RCC_OscInitStruct.HSICalibrationValue = RCC_HSICALIBRATION_DEFAULT;
	RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
	RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSE;
	RCC_OscInitStruct.PLL.PLLMUL = RCC_PLL_MUL9;
	if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
	{
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

	if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_2) != HAL_OK)
	{
		Error_Handler();
	}
	PeriphClkInit.PeriphClockSelection = RCC_PERIPHCLK_USART2 | RCC_PERIPHCLK_I2C1
	                            | RCC_PERIPHCLK_I2C2;
	PeriphClkInit.Usart2ClockSelection = RCC_USART2CLKSOURCE_PCLK1;
	PeriphClkInit.I2c1ClockSelection = RCC_I2C1CLKSOURCE_HSI;
	PeriphClkInit.I2c2ClockSelection = RCC_I2C2CLKSOURCE_HSI;
	if (HAL_RCCEx_PeriphCLKConfig(&PeriphClkInit) != HAL_OK)
	{
		Error_Handler();
	}
}

/**
  * @brief CRC Initialization Function
  * @param None
  * @retval None
  */
static void MX_CRC_Init(void)
{

	/* USER CODE BEGIN CRC_Init 0 */

	/* USER CODE END CRC_Init 0 */

	/* USER CODE BEGIN CRC_Init 1 */

	/* USER CODE END CRC_Init 1 */
	hcrc.Instance = CRC;
	hcrc.Init.DefaultPolynomialUse = DEFAULT_POLYNOMIAL_ENABLE;
	hcrc.Init.DefaultInitValueUse = DEFAULT_INIT_VALUE_ENABLE;
	hcrc.Init.InputDataInversionMode = CRC_INPUTDATA_INVERSION_NONE;
	hcrc.Init.OutputDataInversionMode = CRC_OUTPUTDATA_INVERSION_DISABLE;
	hcrc.InputDataFormat = CRC_INPUTDATA_FORMAT_BYTES;
	if (HAL_CRC_Init(&hcrc) != HAL_OK)
	{
		Error_Handler();
	}
	/* USER CODE BEGIN CRC_Init 2 */

	/* USER CODE END CRC_Init 2 */

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
	hi2c1.Init.Timing = 0x0000020B;
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
  * @brief I2C2 Initialization Function
  * @param None
  * @retval None
  */
static void MX_I2C2_Init(void)
{

	/* USER CODE BEGIN I2C2_Init 0 */

	/* USER CODE END I2C2_Init 0 */

	/* USER CODE BEGIN I2C2_Init 1 */

	/* USER CODE END I2C2_Init 1 */
	hi2c2.Instance = I2C2;
	hi2c2.Init.Timing = 0x00000001;
	hi2c2.Init.OwnAddress1 = 0;
	hi2c2.Init.AddressingMode = I2C_ADDRESSINGMODE_7BIT;
	hi2c2.Init.DualAddressMode = I2C_DUALADDRESS_DISABLE;
	hi2c2.Init.OwnAddress2 = 0;
	hi2c2.Init.OwnAddress2Masks = I2C_OA2_NOMASK;
	hi2c2.Init.GeneralCallMode = I2C_GENERALCALL_DISABLE;
	hi2c2.Init.NoStretchMode = I2C_NOSTRETCH_DISABLE;
	if (HAL_I2C_Init(&hi2c2) != HAL_OK)
	{
		Error_Handler();
	}
	/** Configure Analogue filter
	*/
	if (HAL_I2CEx_ConfigAnalogFilter(&hi2c2, I2C_ANALOGFILTER_ENABLE) != HAL_OK)
	{
		Error_Handler();
	}
	/** Configure Digital filter
	*/
	if (HAL_I2CEx_ConfigDigitalFilter(&hi2c2, 0) != HAL_OK)
	{
		Error_Handler();
	}
	/** I2C Fast mode Plus enable
	*/
	__HAL_SYSCFG_FASTMODEPLUS_ENABLE(I2C_FASTMODEPLUS_I2C2);
	/* USER CODE BEGIN I2C2_Init 2 */

	/* USER CODE END I2C2_Init 2 */

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
	hspi1.Init.BaudRatePrescaler = SPI_BAUDRATEPRESCALER_4;
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
  * @brief TIM3 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM3_Init(void)
{

	/* USER CODE BEGIN TIM3_Init 0 */

	/* USER CODE END TIM3_Init 0 */

	TIM_ClockConfigTypeDef sClockSourceConfig = { 0 };
	TIM_MasterConfigTypeDef sMasterConfig = { 0 };

	/* USER CODE BEGIN TIM3_Init 1 */

	/* USER CODE END TIM3_Init 1 */
	htim3.Instance = TIM3;
	htim3.Init.Prescaler = 0;
	htim3.Init.CounterMode = TIM_COUNTERMODE_UP;
	htim3.Init.Period = 4000;
	htim3.Init.ClockDivision = TIM_CLOCKDIVISION_DIV2;
	htim3.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
	if (HAL_TIM_Base_Init(&htim3) != HAL_OK)
	{
		Error_Handler();
	}
	sClockSourceConfig.ClockSource = TIM_CLOCKSOURCE_INTERNAL;
	if (HAL_TIM_ConfigClockSource(&htim3, &sClockSourceConfig) != HAL_OK)
	{
		Error_Handler();
	}
	sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
	sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
	if (HAL_TIMEx_MasterConfigSynchronization(&htim3, &sMasterConfig) != HAL_OK)
	{
		Error_Handler();
	}
	/* USER CODE BEGIN TIM3_Init 2 */

	/* USER CODE END TIM3_Init 2 */

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
	huart2.Init.BaudRate = 38400;
	huart2.Init.WordLength = UART_WORDLENGTH_8B;
	huart2.Init.StopBits = UART_STOPBITS_1;
	huart2.Init.Parity = UART_PARITY_NONE;
	huart2.Init.Mode = UART_MODE_TX_RX;
	huart2.Init.HwFlowCtl = UART_HWCONTROL_NONE;
	huart2.Init.OverSampling = UART_OVERSAMPLING_16;
	huart2.Init.OneBitSampling = UART_ONE_BIT_SAMPLE_DISABLE;
	huart2.AdvancedInit.AdvFeatureInit = UART_ADVFEATURE_NO_INIT;
	if (HAL_UART_Init(&huart2) != HAL_OK)
	{
		Error_Handler();
	}
	/* USER CODE BEGIN USART2_Init 2 */
	//HAL_NVIC_SetPriority(USART2_IRQn, 0, 0);
	//HAL_NVIC_EnableIRQ(USART2_IRQn);
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
	/* DMA1_Channel4_IRQn interrupt configuration */
	HAL_NVIC_SetPriority(DMA1_Channel4_IRQn, 0, 0);
	HAL_NVIC_EnableIRQ(DMA1_Channel4_IRQn);
	/* DMA1_Channel5_IRQn interrupt configuration */
	HAL_NVIC_SetPriority(DMA1_Channel5_IRQn, 0, 0);
	HAL_NVIC_EnableIRQ(DMA1_Channel5_IRQn);
	/* DMA1_Channel6_IRQn interrupt configuration */
	HAL_NVIC_SetPriority(DMA1_Channel6_IRQn, 0, 0);
	HAL_NVIC_EnableIRQ(DMA1_Channel6_IRQn);
	/* DMA1_Channel7_IRQn interrupt configuration */
	HAL_NVIC_SetPriority(DMA1_Channel7_IRQn, 0, 0);
	HAL_NVIC_EnableIRQ(DMA1_Channel7_IRQn);

}

/**
  * @brief GPIO Initialization Function
  * @param None
  * @retval None
  */
static void MX_GPIO_Init(void)
{
	GPIO_InitTypeDef GPIO_InitStruct = { 0 };

	/* GPIO Ports Clock Enable */
	__HAL_RCC_GPIOE_CLK_ENABLE();
	__HAL_RCC_GPIOC_CLK_ENABLE();
	__HAL_RCC_GPIOF_CLK_ENABLE();
	__HAL_RCC_GPIOA_CLK_ENABLE();
	__HAL_RCC_GPIOD_CLK_ENABLE();
	__HAL_RCC_GPIOB_CLK_ENABLE();

	/*Configure GPIO pin Output Level */
	HAL_GPIO_WritePin(GPIOE,
		CS_I2C_SPI_Pin|LD4_Pin|LD3_Pin|LD5_Pin
	                        |LD7_Pin|LD9_Pin|LD10_Pin|LD8_Pin
	                        |LD6_Pin,
		GPIO_PIN_RESET);

	/*Configure GPIO pin Output Level */
	HAL_GPIO_WritePin(GPIOD, MOT_B_STEP_Pin | MOT_B_DIR_Pin | MOT_A_STEP_Pin | MOT_A_DIR_Pin, GPIO_PIN_RESET);
	HAL_GPIO_WritePin(GPIOD, MOT_ENABLE_Pin, GPIO_PIN_SET);
	/*Configure GPIO pins : DRDY_Pin MEMS_INT3_Pin MEMS_INT4_Pin MEMS_INT1_Pin
	                         MEMS_INT2_Pin */
	GPIO_InitStruct.Pin = DRDY_Pin | MEMS_INT3_Pin | MEMS_INT4_Pin | MEMS_INT1_Pin
	                        | MEMS_INT2_Pin;
	GPIO_InitStruct.Mode = GPIO_MODE_EVT_RISING;
	GPIO_InitStruct.Pull = GPIO_NOPULL;
	HAL_GPIO_Init(GPIOE, &GPIO_InitStruct);

	/*Configure GPIO pins : CS_I2C_SPI_Pin LD4_Pin LD3_Pin LD5_Pin
	                         LD7_Pin LD9_Pin LD10_Pin LD8_Pin
	                         LD6_Pin */
	GPIO_InitStruct.Pin = CS_I2C_SPI_Pin | LD4_Pin | LD3_Pin | LD5_Pin
	                        | LD7_Pin | LD9_Pin | LD10_Pin | LD8_Pin
	                        | LD6_Pin;
	GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
	GPIO_InitStruct.Pull = GPIO_NOPULL;
	GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
	HAL_GPIO_Init(GPIOE, &GPIO_InitStruct);

	/*Configure GPIO pin : B1_Pin */
	GPIO_InitStruct.Pin = B1_Pin;
	GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
	GPIO_InitStruct.Pull = GPIO_NOPULL;
	HAL_GPIO_Init(B1_GPIO_Port, &GPIO_InitStruct);

	/*Configure GPIO pins : MOT_B_STEP_Pin MOT_B_DIR_Pin MOT_A_STEP_Pin MOT_A_DIR_Pin */
	GPIO_InitStruct.Pin = MOT_B_STEP_Pin | MOT_B_DIR_Pin | MOT_A_STEP_Pin | MOT_A_DIR_Pin | MOT_ENABLE_Pin;
	GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
	GPIO_InitStruct.Pull = GPIO_NOPULL;
	GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
	HAL_GPIO_Init(GPIOD, &GPIO_InitStruct);

	/*Configure GPIO pins : DM_Pin DP_Pin */
	GPIO_InitStruct.Pin = DM_Pin | DP_Pin;
	GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
	GPIO_InitStruct.Pull = GPIO_NOPULL;
	GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_HIGH;
	GPIO_InitStruct.Alternate = GPIO_AF14_USB;
	HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

	/*Configure GPIO pin : MPU_int_Pin */
	GPIO_InitStruct.Pin = MPU_int_Pin;
	GPIO_InitStruct.Mode = GPIO_MODE_IT_RISING;
	GPIO_InitStruct.Pull = GPIO_NOPULL;
	HAL_GPIO_Init(MPU_int_GPIO_Port, &GPIO_InitStruct);

	/* EXTI interrupt init*/
	HAL_NVIC_SetPriority(EXTI9_5_IRQn, 0, 0);
	HAL_NVIC_EnableIRQ(EXTI9_5_IRQn);

}

/* USER CODE BEGIN 4 */

/* USER CODE END 4 */


 /**
  * @brief  Period elapsed callback in non blocking mode
  * @note   This function is called  when TIM1 interrupt took place, inside
  * HAL_TIM_IRQHandler(). It makes a direct call to HAL_IncTick() to increment
  * a global variable "uwTick" used as application time base.
  * @param  htim : TIM handle
  * @retval None
  */
void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim)
{
	/* USER CODE BEGIN Callback 0 */

	/* USER CODE END Callback 0 */
	if (htim->Instance == TIM1) {
		HAL_IncTick();
		if (HAL_GPIO_ReadPin(B1_GPIO_Port, B1_Pin) == 0) {
			In1_0++;
			In1_1 = 0;
			if (In1_0 >= REFdebounce) {
				In1_0 = REFdebounce + 1;
				btn1status = 0;
			}
		}
		else {
			In1_1++;
			In1_0 = 0;
			if (In1_1 >= REFdebounce) {
				In1_1 = REFdebounce + 1;
				btn1status = 1;
			}
		}
	}
	
	
	if (htim->Instance == TIM3) {
		
	
		if (currentTickLeft >= ticksPerPulseLeft) {
			currentTickLeft = 0;
		}

		if (currentTickLeft == 0) {
			HAL_GPIO_WritePin(GPIOD, MOT_B_STEP_Pin, GPIO_PIN_SET);
			//PORTD |= _BV(PD7); // digitalWrite(MOT_B_STEP, HIGH);
			HAL_GPIO_WritePin(GPIOE, LD6_Pin, GPIO_PIN_SET);
		}
		else if (currentTickLeft == PULSE_WIDTH) {
			HAL_GPIO_WritePin(GPIOD, MOT_B_STEP_Pin, GPIO_PIN_RESET);
			//PORTD &= ~_BV(PD7); // digitalWrite(MOT_B_STEP, LOW);
			HAL_GPIO_WritePin(GPIOE, LD6_Pin, GPIO_PIN_RESET);
		}
  
		currentTickLeft++;

		if (currentTickRight >= ticksPerPulseRight) {
			currentTickRight = 0;
		}

		if (currentTickRight == 0) {
			HAL_GPIO_WritePin(GPIOD, MOT_A_STEP_Pin, GPIO_PIN_SET);
			//PORTB |= _BV(PB1); // digitalWrite(MOT_A_STEP, HIGH);
			HAL_GPIO_WritePin(GPIOE, LD7_Pin, GPIO_PIN_SET);
		}
		else if (currentTickRight == PULSE_WIDTH) {
			HAL_GPIO_WritePin(GPIOD, MOT_A_STEP_Pin, GPIO_PIN_RESET);
			//PORTB &= ~_BV(PB1); // digitalWrite(MOT_A_STEP, LOW);    
			HAL_GPIO_WritePin(GPIOE, LD7_Pin, GPIO_PIN_RESET);
		}
  
		currentTickRight++;
	}
	/* USER CODE BEGIN Callback 1 */

	/* USER CODE END Callback 1 */
}

void HAL_I2C_MemTxCpltCallback(I2C_HandleTypeDef *hi2c)
{
	/* Prevent unused argument(s) compilation warning */
	UNUSED(hi2c);
	i2c2_transmit_complete = true;
}

void HAL_I2C_MemRxCpltCallback(I2C_HandleTypeDef *hi2c)
{
	/* Prevent unused argument(s) compilation warning */
	UNUSED(hi2c);
	i2c2_receive_complete = true;	
}

void HAL_GPIO_EXTI_Callback(uint16_t GPIO_Pin)
{

	if (GPIO_Pin == GPIO_PIN_6)
	{
		
		mpuInterrupt = true;
	}
}

void HAL_UART_RxCpltCallback(UART_HandleTypeDef* huart)
{


	//HAL_UART_Receive_DMA(huart, rxBuffer, 64);

	//disable half transfer interrupt; it is enabled in HAL_UARTEx_ReceiveToIdle_DMA()
	//__HAL_DMA_DISABLE_IT(huart2.hdmarx, DMA_IT_HT);
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

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
