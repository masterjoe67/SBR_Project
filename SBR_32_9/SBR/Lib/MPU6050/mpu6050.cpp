/*
 * mpu6050.c
 *
 *  Created on: 08.10.2018
 *  	License: MIT
 *      Author: Mateusz Salamon
 *      Based on:
 *      	 - MPU-6000 and MPU-6050 Product Specification Revision 3.4
 *      	 - MPU-6000 and MPU-6050 Register Map and Descriptions Revision 4.2
 *      	 - i2cdevlib by Jeff Rowberg on MIT license
 *      	 - SparkFun MPU-9250 Digital Motion Processor (DMP) Arduino Library on MIT License
 *
 *		www.msalamon.pl
 *		mateusz@msalamon.pl
 *
 *	Website: https://msalamon.pl/6-stopni-swobody-z-mpu6050-na-stm32/
 *	GitHub: https://github.com/lamik/MPU6050_STM32_HAL
 */

#include "stm32f3xx_hal.h"
#include "main.h"
#include <stdlib.h>
#include <string.h>
#include "mpu6050.h"
#include "math.h"
#include "stdbool.h"


#define I2C_TIMEOUT 10

I2C_HandleTypeDef *i2c;
float Acc_Scale;
float Gyr_Scale;

typedef HAL_StatusTypeDef ret_code_t;

static const uint8_t expected_who_am_i = 0x68U;

uint8_t mbuffer[14];
uint32_t fifoTimeout = MPU6050_FIFO_DEFAULT_TIMEOUT;

uint16_t dmpPacketSize;
uint8_t *dmpPacketBuffer;



//
// CONFIG
//
void MPU6050_SetDlpf(uint8_t Value)
{
	uint8_t tmp;
	mpu6050_register_read(MPU6050_RA_CONFIG, &tmp, 1);
	//HAL_I2C_Mem_Read(i2c, MPU6050_ADDRESS, MPU6050_RA_CONFIG, 1, &tmp, 1, I2C_TIMEOUT);
	tmp &= 0xF8;
	tmp |= (Value & 0x7);
	mpu6050_register_write(MPU6050_RA_CONFIG, tmp);
	//HAL_I2C_Mem_Write(i2c, MPU6050_ADDRESS, MPU6050_RA_CONFIG, 1, &tmp, 1, I2C_TIMEOUT);
}

//
// PWR_MGMT_1
//
void MPU6050_DeviceReset(uint8_t Reset)
{
	uint8_t tmp;
	mpu6050_register_read(MPU6050_RA_PWR_MGMT_1, &tmp, 1);
	//HAL_I2C_Mem_Read(i2c, MPU6050_ADDRESS, MPU6050_RA_PWR_MGMT_1, 1, &tmp, 1, I2C_TIMEOUT);
	tmp &= ~(1 << MPU6050_PWR1_DEVICE_RESET_BIT);
	tmp |= ((Reset & 0x1) << MPU6050_PWR1_DEVICE_RESET_BIT);
	mpu6050_register_write(MPU6050_RA_PWR_MGMT_1, tmp);
	//HAL_I2C_Mem_Write(i2c, MPU6050_ADDRESS, MPU6050_RA_PWR_MGMT_1, 1, &tmp, 1, I2C_TIMEOUT);
}

void MPU6050_SetSleepEnabled(uint8_t Enable)
{
	uint8_t tmp;
	mpu6050_register_read(MPU6050_RA_PWR_MGMT_1, &tmp, 1);
	//HAL_I2C_Mem_Read(i2c, MPU6050_ADDRESS, MPU6050_RA_PWR_MGMT_1, 1, &tmp, 1, I2C_TIMEOUT);
	tmp &= ~(1 << MPU6050_PWR1_SLEEP_BIT);
	tmp |= ((Enable & 0x1) << MPU6050_PWR1_SLEEP_BIT);
	mpu6050_register_write(MPU6050_RA_PWR_MGMT_1, tmp);
	//HAL_I2C_Mem_Write(i2c, MPU6050_ADDRESS, MPU6050_RA_PWR_MGMT_1, 1, &tmp, 1, I2C_TIMEOUT);
}

void MPU6050_SetCycleEnabled(uint8_t Enable)
{
	uint8_t tmp;
	mpu6050_register_read(MPU6050_RA_PWR_MGMT_1, &tmp, 1);
	//HAL_I2C_Mem_Read(i2c, MPU6050_ADDRESS, MPU6050_RA_PWR_MGMT_1, 1, &tmp, 1, I2C_TIMEOUT);
	tmp &= ~(1 << MPU6050_PWR1_CYCLE_BIT);
	tmp |= ((Enable & 0x1) << MPU6050_PWR1_CYCLE_BIT);
	mpu6050_register_write(MPU6050_RA_PWR_MGMT_1, tmp);
	//HAL_I2C_Mem_Write(i2c, MPU6050_ADDRESS, MPU6050_RA_PWR_MGMT_1, 1, &tmp, 1, I2C_TIMEOUT);
}

void MPU6050_SetTemperatureSensorDisbled(uint8_t Disable)
{
	uint8_t tmp;
	mpu6050_register_read(MPU6050_RA_PWR_MGMT_1, &tmp, 1);
	//HAL_I2C_Mem_Read(i2c, MPU6050_ADDRESS, MPU6050_RA_PWR_MGMT_1, 1, &tmp, 1, I2C_TIMEOUT);
	tmp &= ~(1 << MPU6050_PWR1_TEMP_DIS_BIT);
	tmp |= ((Disable & 0x1) << MPU6050_PWR1_TEMP_DIS_BIT);
	mpu6050_register_write(MPU6050_RA_PWR_MGMT_1, tmp);
	//HAL_I2C_Mem_Write(i2c, MPU6050_ADDRESS, MPU6050_RA_PWR_MGMT_1, 1, &tmp, 1, I2C_TIMEOUT);
}

void MPU6050_SetClockSource(uint8_t Source)
{
	uint8_t tmp;
	mpu6050_register_read(MPU6050_RA_PWR_MGMT_1, &tmp, 1);
	//HAL_I2C_Mem_Read(i2c, MPU6050_ADDRESS, MPU6050_RA_PWR_MGMT_1, 1, &tmp, 1, I2C_TIMEOUT);
	tmp &= 0xF8;
	tmp |= (Source & 0x7);
	mpu6050_register_write(MPU6050_RA_PWR_MGMT_1, tmp);
	//HAL_I2C_Mem_Write(i2c, MPU6050_ADDRESS, MPU6050_RA_PWR_MGMT_1, 1, &tmp, 1, I2C_TIMEOUT);
}

//
//	PWR_MGMT_2
//
void MPU6050_SetLowPowerWakeUpFrequency(uint8_t Frequency)
{
	uint8_t tmp;
	mpu6050_register_read(MPU6050_RA_PWR_MGMT_2, &tmp, 1);
	//HAL_I2C_Mem_Read(i2c, MPU6050_ADDRESS, MPU6050_RA_PWR_MGMT_2, 1, &tmp, 1, I2C_TIMEOUT);
	tmp &= 0x3F;
	tmp |= (Frequency & 0x3) << MPU6050_PWR2_LP_WAKE_CTRL_BIT;
	mpu6050_register_write(MPU6050_RA_PWR_MGMT_2, tmp);
	//HAL_I2C_Mem_Write(i2c, MPU6050_ADDRESS, MPU6050_RA_PWR_MGMT_2, 1, &tmp, 1, I2C_TIMEOUT);
}

void MPU6050_AccelerometerAxisStandby(uint8_t XA_Stby, uint8_t YA_Stby, uint8_t ZA_Stby)
{
	uint8_t tmp;
	mpu6050_register_read(MPU6050_RA_PWR_MGMT_2, &tmp, 1);
	//HAL_I2C_Mem_Read(i2c, MPU6050_ADDRESS, MPU6050_RA_PWR_MGMT_2, 1, &tmp, 1, I2C_TIMEOUT);
	tmp &= 0xC7;
	tmp |= ((XA_Stby & 0x1) << MPU6050_PWR2_STBY_XA_BIT) | ((YA_Stby & 0x1) << MPU6050_PWR2_STBY_YA_BIT) | ((ZA_Stby & 0x1) << MPU6050_PWR2_STBY_ZA_BIT);
	mpu6050_register_write(MPU6050_RA_PWR_MGMT_2, tmp);
	//HAL_I2C_Mem_Write(i2c, MPU6050_ADDRESS, MPU6050_RA_PWR_MGMT_2, 1, &tmp, 1, I2C_TIMEOUT);
}

void MPU6050_GyroscopeAxisStandby(uint8_t XG_Stby, uint8_t YG_Stby, uint8_t ZG_Stby)
{
	uint8_t tmp;
	mpu6050_register_read(MPU6050_RA_PWR_MGMT_2, &tmp, 1);
	//HAL_I2C_Mem_Read(i2c, MPU6050_ADDRESS, MPU6050_RA_PWR_MGMT_2, 1, &tmp, 1, I2C_TIMEOUT);
	tmp &= 0xF8;
	tmp |= ((XG_Stby & 0x1) << MPU6050_PWR2_STBY_XG_BIT) | ((YG_Stby & 0x1) << MPU6050_PWR2_STBY_YG_BIT) | ((ZG_Stby & 0x1) << MPU6050_PWR2_STBY_ZG_BIT);
	mpu6050_register_write(MPU6050_RA_PWR_MGMT_2, tmp);
	//HAL_I2C_Mem_Write(i2c, MPU6050_ADDRESS, MPU6050_RA_PWR_MGMT_2, 1, &tmp, 1, I2C_TIMEOUT);
}

//
//	Measurement scale configuration
//
void MPU6050_SetFullScaleGyroRange(uint8_t Range)
{
	uint8_t tmp;
	mpu6050_register_read(MPU6050_RA_GYRO_CONFIG, &tmp, 1);
	//HAL_I2C_Mem_Read(i2c, MPU6050_ADDRESS, MPU6050_RA_GYRO_CONFIG, 1, &tmp, 1, I2C_TIMEOUT);
	tmp &= 0xE7;
	tmp |= ((Range & 0x7) << 3);
	mpu6050_register_write(MPU6050_RA_GYRO_CONFIG, tmp);
	//HAL_I2C_Mem_Write(i2c, MPU6050_ADDRESS, MPU6050_RA_GYRO_CONFIG, 1, &tmp, 1, I2C_TIMEOUT);

	switch (Range)
	{
	case MPU6050_GYRO_FS_250:
		Gyr_Scale = 0.007633;
		break;
	case MPU6050_GYRO_FS_500:
		Gyr_Scale = 0.015267;
		break;
	case MPU6050_GYRO_FS_1000:
		Gyr_Scale = 0.030487;
		break;
	case MPU6050_GYRO_FS_2000:
		Gyr_Scale = 0.060975;
		break;
	default:
		break;
	}
}

void MPU6050_SetFullScaleAccelRange(uint8_t Range)
{
	uint8_t tmp;
	mpu6050_register_read(MPU6050_RA_ACCEL_CONFIG, &tmp, 1);
	//HAL_I2C_Mem_Read(i2c, MPU6050_ADDRESS, MPU6050_RA_ACCEL_CONFIG, 1, &tmp, 1, I2C_TIMEOUT);
	tmp &= 0xE7;
	tmp |= ((Range & 0x7) << 3);
	mpu6050_register_write(MPU6050_RA_ACCEL_CONFIG, tmp);
	//HAL_I2C_Mem_Write(i2c, MPU6050_ADDRESS, MPU6050_RA_ACCEL_CONFIG, 1, &tmp, 1, I2C_TIMEOUT);

	switch (Range)
	{
	case MPU6050_ACCEL_FS_2:
		Acc_Scale = 0.000061;
		break;
	case MPU6050_ACCEL_FS_4:
		Acc_Scale = 0.000122;
		break;
	case MPU6050_ACCEL_FS_8:
		Acc_Scale = 0.000244;
		break;
	case MPU6050_ACCEL_FS_16:
		Acc_Scale = 0.0004882;
		break;
	default:
		break;
	}
}

//
// Reading data
//
int16_t MPU6050_GetTemperatureRAW(void)
{
	uint8_t tmp[2];
	mpu6050_register_read(MPU6050_RA_TEMP_OUT_H, tmp, 2);
	//HAL_I2C_Mem_Read(i2c, MPU6050_ADDRESS, MPU6050_RA_TEMP_OUT_H, 1, tmp, 2, I2C_TIMEOUT);
	return (((int16_t)tmp[0]) << 8) | tmp[1];
}

float MPU6050_GetTemperatureCelsius(void)
{
	int16_t temp;

	temp = MPU6050_GetTemperatureRAW();

	return (float)temp / 340 + 36.53;
}

int16_t MPU6050_GetAccelerationXRAW(void)
{
	uint8_t tmp[2];
	mpu6050_register_read(MPU6050_RA_ACCEL_XOUT_H, tmp, 2);
	//HAL_I2C_Mem_Read(i2c, MPU6050_ADDRESS, MPU6050_RA_ACCEL_XOUT_H, 1, tmp, 2, I2C_TIMEOUT);
	return (((int16_t)tmp[0]) << 8) | tmp[1];
}

int16_t MPU6050_GetAccelerationYRAW(void)
{
	uint8_t tmp[2];
	mpu6050_register_read(MPU6050_RA_ACCEL_YOUT_H, tmp, 2);
	//HAL_I2C_Mem_Read(i2c, MPU6050_ADDRESS, MPU6050_RA_ACCEL_YOUT_H, 1, tmp, 2, I2C_TIMEOUT);
	return (((int16_t)tmp[0]) << 8) | tmp[1];
}

int16_t MPU6050_GetAccelerationZRAW(void)
{
	uint8_t tmp[2];
	mpu6050_register_read(MPU6050_RA_ACCEL_ZOUT_H, tmp, 2);
	//HAL_I2C_Mem_Read(i2c, MPU6050_ADDRESS, MPU6050_RA_ACCEL_ZOUT_H, 1, tmp, 2, I2C_TIMEOUT);
	return (((int16_t)tmp[0]) << 8) | tmp[1];
}

void MPU6050_GetAccelerometerRAW(int16_t *x, int16_t *y, int16_t *z)
{
	uint8_t tmp[6];
	mpu6050_register_read(MPU6050_RA_ACCEL_XOUT_H, tmp, 6);
	//HAL_I2C_Mem_Read(i2c, MPU6050_ADDRESS, MPU6050_RA_ACCEL_XOUT_H, 1, tmp, 6, I2C_TIMEOUT);

	*x = (((int16_t)tmp[0]) << 8) | tmp[1];
	*y = (((int16_t)tmp[2]) << 8) | tmp[3];
	*z = (((int16_t)tmp[4]) << 8) | tmp[5];
}

void MPU6050_GetAccelerometerScaled(float* x, float* y, float* z)
{
	int16_t tmp_x, tmp_y, tmp_z;
	MPU6050_GetAccelerometerRAW(&tmp_x, &tmp_y, &tmp_z);

	*x = (float)tmp_x * Acc_Scale;
	*y = (float)tmp_y * Acc_Scale;
	*z = (float)tmp_z * Acc_Scale;
}

int16_t MPU6050_GetRotationXRAW(void)
{
	uint8_t tmp[2];
	mpu6050_register_read(MPU6050_RA_GYRO_XOUT_H, tmp, 2);
	//HAL_I2C_Mem_Read(i2c, MPU6050_ADDRESS, MPU6050_RA_GYRO_XOUT_H, 1, tmp, 2, I2C_TIMEOUT);
	return (((int16_t)tmp[0]) << 8) | tmp[1];
}

int16_t MPU6050_GetRotationYRAW(void)
{
	uint8_t tmp[2];
	mpu6050_register_read(MPU6050_RA_GYRO_YOUT_H, tmp, 2);
	//HAL_I2C_Mem_Read(i2c, MPU6050_ADDRESS, MPU6050_RA_GYRO_YOUT_H, 1, tmp, 2, I2C_TIMEOUT);
	return (((int16_t)tmp[0]) << 8) | tmp[1];
}

int16_t MPU6050_GetRotationZRAW(void)
{
	uint8_t tmp[2];
	mpu6050_register_read(MPU6050_RA_GYRO_ZOUT_H, tmp, 2);
	//HAL_I2C_Mem_Read(i2c, MPU6050_ADDRESS, MPU6050_RA_GYRO_ZOUT_H, 1, tmp, 2, I2C_TIMEOUT);
	return (((int16_t)tmp[0]) << 8) | tmp[1];
}

void MPU6050_GetGyroscopeRAW(int16_t *x, int16_t *y, int16_t *z)
{
	uint8_t tmp[6];
	mpu6050_register_read(MPU6050_RA_GYRO_XOUT_H, tmp, 6);
	//HAL_I2C_Mem_Read(i2c, MPU6050_ADDRESS, MPU6050_RA_GYRO_XOUT_H, 1, tmp, 6, I2C_TIMEOUT);

	*x = (((int16_t)tmp[0]) << 8) | tmp[1];
	*y = (((int16_t)tmp[2]) << 8) | tmp[3];
	*z = (((int16_t)tmp[4]) << 8) | tmp[5];
}

void MPU6050_GetGyroscopeScaled(float* x, float* y, float* z)
{
	int16_t tmp_x, tmp_y, tmp_z;

	MPU6050_GetGyroscopeRAW(&tmp_x, &tmp_y, &tmp_z);

	*x = (float)tmp_x * Gyr_Scale;
	*y = (float)tmp_y * Gyr_Scale;
	*z = (float)tmp_z * Gyr_Scale;
}

void MPU6050_GetRollPitch(float* Roll, float* Pitch)
{
	float acc_x, acc_y, acc_z;
	MPU6050_GetAccelerometerScaled(&acc_x, &acc_y, &acc_z);

	*Roll = atan2(acc_y, acc_z) * 180.0 / M_PI;
	*Pitch = -(atan2(acc_x, sqrt(acc_y*acc_y + acc_z*acc_z)) * 180.0) / M_PI;
}

//
//	Setting INT pin
//
void MPU6050_SetInterruptMode(uint8_t Mode)
{
	uint8_t tmp;
	mpu6050_register_read(MPU6050_RA_INT_PIN_CFG, &tmp, 1);
	//HAL_I2C_Mem_Read(i2c, MPU6050_ADDRESS, MPU6050_RA_INT_PIN_CFG, 1, &tmp, 1, I2C_TIMEOUT);
	tmp &= ~(1 << MPU6050_INTCFG_INT_LEVEL_BIT);
	tmp |= ((Mode & 0x1) << MPU6050_INTCFG_INT_LEVEL_BIT);
	mpu6050_register_write(MPU6050_RA_INT_PIN_CFG, tmp);
	//HAL_I2C_Mem_Write(i2c, MPU6050_ADDRESS, MPU6050_RA_INT_PIN_CFG, 1, &tmp, 1, I2C_TIMEOUT);
}

void MPU6050_SetInterruptDrive(uint8_t Drive)
{
	uint8_t tmp;
	mpu6050_register_read(MPU6050_RA_INT_PIN_CFG, &tmp, 1);
	//HAL_I2C_Mem_Read(i2c, MPU6050_ADDRESS, MPU6050_RA_INT_PIN_CFG, 1, &tmp, 1, I2C_TIMEOUT);
	tmp &= ~(1 << MPU6050_INTCFG_INT_OPEN_BIT);
	tmp |= ((Drive & 0x1) << MPU6050_INTCFG_INT_OPEN_BIT);
	mpu6050_register_write(MPU6050_RA_INT_PIN_CFG, tmp);
	//HAL_I2C_Mem_Write(i2c, MPU6050_ADDRESS, MPU6050_RA_INT_PIN_CFG, 1, &tmp, 1, I2C_TIMEOUT);
}

void MPU6050_SetInterruptLatch(uint8_t Latch)
{
	uint8_t tmp;
	mpu6050_register_read(MPU6050_RA_INT_PIN_CFG, &tmp, 1);
	//HAL_I2C_Mem_Read(i2c, MPU6050_ADDRESS, MPU6050_RA_INT_PIN_CFG, 1, &tmp, 1, I2C_TIMEOUT);
	tmp &= ~(1 << MPU6050_INTCFG_INT_RD_CLEAR_BIT);
	tmp |= ((Latch & 0x1) << MPU6050_INTCFG_INT_RD_CLEAR_BIT);
	mpu6050_register_write(MPU6050_RA_INT_PIN_CFG, tmp);
	//HAL_I2C_Mem_Write(i2c, MPU6050_ADDRESS, MPU6050_RA_INT_PIN_CFG, 1, &tmp, 1, I2C_TIMEOUT);
}

void MPU6050_SetInterruptLatchClear(uint8_t Clear)
{
	uint8_t tmp;
	mpu6050_register_read(MPU6050_RA_INT_PIN_CFG, &tmp, 1);
	//HAL_I2C_Mem_Read(i2c, MPU6050_ADDRESS, MPU6050_RA_INT_PIN_CFG, 1, &tmp, 1, I2C_TIMEOUT);
	tmp &= ~(1 << MPU6050_INTCFG_LATCH_INT_EN_BIT);
	tmp |= ((Clear & 0x1) << MPU6050_INTCFG_LATCH_INT_EN_BIT);
	mpu6050_register_write(MPU6050_RA_INT_PIN_CFG, tmp);
	//HAL_I2C_Mem_Write(i2c, MPU6050_ADDRESS, MPU6050_RA_INT_PIN_CFG, 1, &tmp, 1, I2C_TIMEOUT);
}

void MPU6050_SetIntEnableRegister(uint8_t Value)
{
	mpu6050_register_write(MPU6050_RA_INT_ENABLE, Value);
	//HAL_I2C_Mem_Write(i2c, MPU6050_ADDRESS, MPU6050_RA_INT_ENABLE, 1, &Value, 1, I2C_TIMEOUT);
}

void MPU6050_SetIntDataReadyEnabled(uint8_t Enable)
{
	uint8_t tmp;
	mpu6050_register_read(MPU6050_RA_INT_ENABLE, &tmp, 1);
	//HAL_I2C_Mem_Read(i2c, MPU6050_ADDRESS, MPU6050_RA_INT_ENABLE, 1, &tmp, 1, I2C_TIMEOUT);
	tmp &= ~(1 << MPU6050_INTERRUPT_DATA_RDY_BIT);
	tmp |= ((Enable & 0x1) << MPU6050_INTERRUPT_DATA_RDY_BIT);
	mpu6050_register_write(MPU6050_RA_INT_ENABLE, tmp);
	//HAL_I2C_Mem_Write(i2c, MPU6050_ADDRESS, MPU6050_RA_INT_ENABLE, 1, &tmp, 1, I2C_TIMEOUT);
}

uint8_t MPU6050_GetIntStatusRegister(void)
{
	uint8_t tmp;
	//mpu6050_register_read(MPU6050_RA_INT_STATUS, &tmp, 1);
	HAL_I2C_Mem_Read(i2c, MPU6050_ADDRESS, MPU6050_RA_INT_STATUS, 1, &tmp, 1, I2C_TIMEOUT);
	return tmp;
}

uint8_t MPU6050_GetDeviceID(void)
{
	uint8_t tmp;
	mpu6050_register_read(MPU6050_RA_WHO_AM_I, &tmp, 1);
	//HAL_I2C_Mem_Read(i2c, MPU6050_ADDRESS, MPU6050_RA_WHO_AM_I, 1, &tmp, 1, I2C_TIMEOUT);
	return tmp << 1;
}

//
//	Motion functions - not included in documentation/register map
//
void MPU6050_SetDHPFMode(uint8_t Dhpf)
{
	uint8_t tmp;
	mpu6050_register_read(MPU6050_RA_ACCEL_CONFIG, &tmp, 1);
	//HAL_I2C_Mem_Read(i2c, MPU6050_ADDRESS, MPU6050_RA_ACCEL_CONFIG, 1, &tmp, 1, I2C_TIMEOUT);
	tmp &= ~(0x07);
	tmp |= Dhpf & 0x7;
	mpu6050_register_write(MPU6050_RA_ACCEL_CONFIG, tmp);
	//HAL_I2C_Mem_Write(i2c, MPU6050_ADDRESS, MPU6050_RA_ACCEL_CONFIG, 1, &tmp, 1, I2C_TIMEOUT);
}

uint8_t MPU6050_GetMotionStatusRegister(void)
{
	uint8_t tmp;
	HAL_I2C_Mem_Read(i2c, MPU6050_ADDRESS, MPU6050_RA_MOT_DETECT_STATUS, 1, &tmp, 1, I2C_TIMEOUT);
	return tmp;
}

void MPU6050_SetIntZeroMotionEnabled(uint8_t Enable)
{
	uint8_t tmp;
	HAL_I2C_Mem_Read(i2c, MPU6050_ADDRESS, MPU6050_RA_INT_ENABLE, 1, &tmp, 1, I2C_TIMEOUT);
	tmp &= ~(1 << MPU6050_INTERRUPT_ZMOT_BIT);
	tmp |= ((Enable & 0x1) << MPU6050_INTERRUPT_ZMOT_BIT);
	HAL_I2C_Mem_Write(i2c, MPU6050_ADDRESS, MPU6050_RA_INT_ENABLE, 1, &tmp, 1, I2C_TIMEOUT);
}

void MPU6050_SetIntMotionEnabled(uint8_t Enable)
{
	uint8_t tmp;
	HAL_I2C_Mem_Read(i2c, MPU6050_ADDRESS, MPU6050_RA_INT_ENABLE, 1, &tmp, 1, I2C_TIMEOUT);
	tmp &= ~(1 << MPU6050_INTERRUPT_MOT_BIT);
	tmp |= ((Enable & 0x1) << MPU6050_INTERRUPT_MOT_BIT);
	HAL_I2C_Mem_Write(i2c, MPU6050_ADDRESS, MPU6050_RA_INT_ENABLE, 1, &tmp, 1, I2C_TIMEOUT);
}

void MPU6050_SetIntFreeFallEnabled(uint8_t Enable)
{
	uint8_t tmp;
	HAL_I2C_Mem_Read(i2c, MPU6050_ADDRESS, MPU6050_RA_INT_ENABLE, 1, &tmp, 1, I2C_TIMEOUT);
	tmp &= ~(1 << MPU6050_INTERRUPT_FF_BIT);
	tmp |= ((Enable & 0x1) << MPU6050_INTERRUPT_FF_BIT);
	HAL_I2C_Mem_Write(i2c, MPU6050_ADDRESS, MPU6050_RA_INT_ENABLE, 1, &tmp, 1, I2C_TIMEOUT);
}

void MPU6050_SetMotionDetectionThreshold(uint8_t Threshold)
{
	HAL_I2C_Mem_Write(i2c, MPU6050_ADDRESS, MPU6050_RA_MOT_THR, 1, &Threshold, 1, I2C_TIMEOUT);
}

void MPU6050_SetMotionDetectionDuration(uint8_t Duration)
{
	HAL_I2C_Mem_Write(i2c, MPU6050_ADDRESS, MPU6050_RA_MOT_DUR, 1, &Duration, 1, I2C_TIMEOUT);
}

void MPU6050_SetZeroMotionDetectionThreshold(uint8_t Threshold)
{
	HAL_I2C_Mem_Write(i2c, MPU6050_ADDRESS, MPU6050_RA_ZRMOT_THR, 1, &Threshold, 1, I2C_TIMEOUT);
}

void MPU6050_SetZeroMotionDetectionDuration(uint8_t Duration)
{
	HAL_I2C_Mem_Write(i2c, MPU6050_ADDRESS, MPU6050_RA_ZRMOT_DUR, 1, &Duration, 1, I2C_TIMEOUT);
}

void MPU6050_SetFreeFallDetectionThreshold(uint8_t Threshold)
{
	HAL_I2C_Mem_Write(i2c, MPU6050_ADDRESS, MPU6050_RA_FF_THR, 1, &Threshold, 1, I2C_TIMEOUT);
}

void MPU6050_SetFreeFallDetectionDuration(uint8_t Duration)
{
	HAL_I2C_Mem_Write(i2c, MPU6050_ADDRESS, MPU6050_RA_FF_DUR, 1, &Duration, 1, I2C_TIMEOUT);
}

// BANK_SEL register

void MPU6050_setMemoryBank(uint8_t bank, bool prefetchEnabled, bool userBank) {
	bank &= 0x1F;
	if (userBank) bank |= 0x20;
	if (prefetchEnabled) bank |= 0x40;
	HAL_I2C_Mem_Write(i2c, MPU6050_ADDRESS, MPU6050_RA_BANK_SEL, 1, &bank, 1, I2C_TIMEOUT);
	
}

// MEM_START_ADDR register

void MPU6050_setMemoryStartAddress(uint8_t address) {
	HAL_I2C_Mem_Write(i2c, MPU6050_ADDRESS, MPU6050_RA_MEM_START_ADDR, 1, &address, 1, I2C_TIMEOUT);
	//I2Cdev::writeByte(devAddr, MPU6050_RA_MEM_START_ADDR, address, wireObj);
}

bool mpu6050_register_read(uint8_t register_address, uint8_t * destination, uint8_t number_of_bytes)
{
	ret_code_t ret;

#if USE_I2C_DMA
	uint32_t start_tick = HAL_GetTick();
	i2c2_receive_complete = false;
	ret = HAL_I2C_Mem_Read_DMA(i2c, MPU6050_ADDRESS, register_address, I2C_MEMADD_SIZE_8BIT, destination, number_of_bytes);
	while (!i2c2_receive_complete) {
		if ((HAL_GetTick() - start_tick) > I2C_TIMEOUT)
			break;
		__WFE();
	}
#else
	ret = HAL_I2C_Mem_Read(i2c, MPU6050_ADDRESS, register_address, I2C_MEMADD_SIZE_8BIT, destination, number_of_bytes, I2C_TIMEOUT);
#endif

	return (ret == HAL_OK);

	// HAL_StatusTypeDef HAL_I2C_Mem_Read_DMA(I2C_HandleTypeDef *hi2c, uint16_t DevAddress, uint16_t MemAddress, uint16_t MemAddSize, uint8_t *pData, uint16_t Size)
}

bool mpu6050_verify_product_id(void)
{
	uint8_t who_am_i;

	if (mpu6050_register_read(MPU6050_RA_WHO_AM_I, &who_am_i, 1)) {
		if (who_am_i != expected_who_am_i) {
			return false;
		}
		else {
			return true;
		}
	}
	else {
		return false;
	}
}

bool mpu6050_register_write(uint8_t register_address, uint8_t value)
{
	ret_code_t ret;

#if USE_I2C_DMA
	uint32_t start_tick = HAL_GetTick();
	i2c2_transmit_complete = false;
	ret = HAL_I2C_Mem_Write_DMA(i2c, MPU6050_ADDRESS, register_address, I2C_MEMADD_SIZE_8BIT, &value, 1);
	while (!i2c2_transmit_complete) {
		if ((HAL_GetTick() - start_tick) > I2C_TIMEOUT)
			break;
		__WFE();
	}
#else
	ret = HAL_I2C_Mem_Write(i2c, MPU6050_ADDRESS, register_address, I2C_MEMADD_SIZE_8BIT, &value, 1, I2C_TIMEOUT);
#endif

	return (ret == HAL_OK);

	// HAL_StatusTypeDef HAL_I2C_Mem_Write_DMA(I2C_HandleTypeDef *hi2c, uint16_t DevAddress, uint16_t MemAddress, uint16_t MemAddSize, uint8_t *pData, uint16_t Size)
}

bool mpu6050_writeBytes(uint8_t mem_address, uint8_t length, uint8_t* data) 
{
	ret_code_t ret;

/*#if USE_I2C_DMA
	uint32_t start_tick = HAL_GetTick();
	i2c2_transmit_complete = false;
	ret = HAL_I2C_Mem_Write_DMA(i2c, MPU6050_ADDRESS, mem_address, I2C_MEMADD_SIZE_8BIT, &value, 1);
	while (!i2c2_transmit_complete) {
		if ((HAL_GetTick() - start_tick) > I2C_TIMEOUT)
			break;
		__WFE();
	}
#else*/
	ret = HAL_I2C_Mem_Write(i2c, MPU6050_ADDRESS, mem_address, I2C_MEMADD_SIZE_8BIT, data, length, I2C_TIMEOUT);
//#endif

	return (ret == HAL_OK);

	// HAL_StatusTypeDef HAL_I2C_Mem_Write_DMA(I2C_HandleTypeDef *hi2c, uint16_t DevAddress, uint16_t MemAddress, uint16_t MemAddSize, uint8_t *pData, uint16_t Size)
}

//
//	Initialization
//
bool MPU6050_Init(I2C_HandleTypeDef *hi2c)
{
	bool transfer_succeeded = true;
	i2c = hi2c;
	//MPU6050_DeviceReset(1);
	
	// Do a reset on signal paths
	uint8_t reset_value = 0x04U | 0x02U | 0x01U; // Resets gyro, accelerometer and temperature sensor signal paths.
	transfer_succeeded &= mpu6050_register_write(MPU6050_RA_SIGNAL_PATH_RESET, reset_value);
	
	// Read and verify product ID
	transfer_succeeded &= mpu6050_verify_product_id();
	
	if (!transfer_succeeded)
		return false;
	
    MPU6050_SetSleepEnabled(0);
    MPU6050_SetClockSource(MPU6050_CLOCK_INTERNAL);
    MPU6050_SetDlpf(MPU6050_DLPF_BW_20);
    MPU6050_SetFullScaleGyroRange(MPU6050_GYRO_FS_250);
    MPU6050_SetFullScaleAccelRange(MPU6050_ACCEL_FS_2);
	return true;
}

void MPU6050_setSlaveAddress(uint8_t num, uint8_t address) {
	if (num > 3) return;
	HAL_I2C_Mem_Write(i2c, MPU6050_ADDRESS, MPU6050_RA_I2C_SLV0_ADDR + num * 3, 1, &address, 1, I2C_TIMEOUT);
}

void MPU6050_setI2CMasterModeEnabled(uint8_t Enable) {
	
	uint8_t tmp;
	HAL_I2C_Mem_Read(i2c, MPU6050_ADDRESS, MPU6050_RA_USER_CTRL, 1, &tmp, 1, I2C_TIMEOUT);
	tmp &= ~(1 << MPU6050_USERCTRL_I2C_MST_EN_BIT);
	tmp |= ((Enable & 0x1) << MPU6050_INTERRUPT_FF_BIT);
	HAL_I2C_Mem_Write(i2c, MPU6050_ADDRESS, MPU6050_RA_USER_CTRL, 1, &tmp, 1, I2C_TIMEOUT);
	
	//I2Cdev::writeBit(devAddr, MPU6050_RA_USER_CTRL, MPU6050_USERCTRL_I2C_MST_EN_BIT, enabled, wireObj);
}

void MPU6050_resetI2CMaster() {
	uint8_t tmp;
	HAL_I2C_Mem_Read(i2c, MPU6050_ADDRESS, MPU6050_RA_USER_CTRL, 1, &tmp, 1, I2C_TIMEOUT);
	tmp &= ~(1 << MPU6050_USERCTRL_I2C_MST_RESET_BIT);
	tmp |= ((0x01 & 0x1) << MPU6050_INTERRUPT_FF_BIT);
	HAL_I2C_Mem_Write(i2c, MPU6050_ADDRESS, MPU6050_RA_USER_CTRL, 1, &tmp, 1, I2C_TIMEOUT);
	
	//I2Cdev::writeBit(devAddr, MPU6050_RA_USER_CTRL, MPU6050_USERCTRL_I2C_MST_RESET_BIT, true, wireObj);
}

/** Set gyroscope sample rate divider.
 * @param rate New sample rate divider
 * @see getRate()
 * @see MPU6050_RA_SMPLRT_DIV
 */
void MPU6050_setRate(uint8_t rate) {
	HAL_I2C_Mem_Write(i2c, MPU6050_ADDRESS, MPU6050_RA_SMPLRT_DIV, 1, &rate, 1, I2C_TIMEOUT);
	//I2Cdev::writeByte(devAddr, MPU6050_RA_SMPLRT_DIV, rate, wireObj);
}

bool MPU6050_writeBits(uint8_t regAddr, uint8_t bitStart, uint8_t length, uint8_t data) {
	//      010 value to write
	// 76543210 bit numbers
	//    xxx   args: bitStart=4, length=3
	// 00011100 mask byte
	// 10101111 original value (sample)
	// 10100011 original & ~mask
	// 10101011 masked | value
	uint8_t b;
	//HAL_I2C_Mem_Read(i2c, MPU6050_ADDRESS, regAddr, 1, &b, 1, I2C_TIMEOUT);
	if (HAL_I2C_Mem_Read(i2c, MPU6050_ADDRESS, regAddr, 1, &b, 1, I2C_TIMEOUT) == HAL_OK) {
		uint8_t mask = ((1 << length) - 1) << (bitStart - length + 1);
		data <<= (bitStart - length + 1); // shift data into correct position
		data &= mask; // zero all non-important bits in data
		b &= ~(mask); // zero all important bits in existing byte
		b |= data; // combine data with existing byte
		return HAL_OK == HAL_I2C_Mem_Write(i2c, MPU6050_ADDRESS, regAddr, 1, &b, 1, I2C_TIMEOUT);
		//return writeByte(devAddr, regAddr, b, wireObj);
	}
	else {
		return false;
	}
}

/** write a single bit in an 8-bit device register.
 * @param devAddr I2C slave device address
 * @param regAddr Register regAddr to write to
 * @param bitNum Bit position to write (0-7)
 * @param value New bit value to write
 * @return Status of operation (true = success)
 */
bool MPU6050_writeBit(uint8_t regAddr, uint8_t bitNum, uint8_t data) {
	uint8_t b;
	HAL_I2C_Mem_Read(i2c, MPU6050_ADDRESS, regAddr, 1, &b, 1, I2C_TIMEOUT);
	//readByte(devAddr, regAddr, &b, I2Cdev::readTimeout, wireObj);
	b = (data != 0) ? (b | (1 << bitNum)) : (b & ~(1 << bitNum));
	return HAL_OK == HAL_I2C_Mem_Write(i2c, MPU6050_ADDRESS, regAddr, 1, &b, 1, I2C_TIMEOUT);
	//return writeByte(devAddr, regAddr, b, wireObj);
}

/** Set external FSYNC configuration.
 * @see getExternalFrameSync()
 * @see MPU6050_RA_CONFIG
 * @param sync New FSYNC configuration value
 */
void MPU6050_setExternalFrameSync(uint8_t sync) {
	MPU6050_writeBits(MPU6050_RA_CONFIG, MPU6050_CFG_EXT_SYNC_SET_BIT, MPU6050_CFG_EXT_SYNC_SET_LENGTH, sync);
}

void MPU6050_setOTPBankValid(bool enabled) {

	//MPU6050_writeBits(MPU6050_RA_XG_OFFS_TC, MPU6050_TC_OTP_BNK_VLD_BIT, 1, 1);
	
	MPU6050_writeBit(MPU6050_RA_XG_OFFS_TC, MPU6050_TC_OTP_BNK_VLD_BIT, enabled);
}

bool MPU6050_writeMemoryBlock(const uint8_t *data, uint16_t dataSize, uint8_t bank, uint8_t address, bool verify) {
	MPU6050_setMemoryBank(bank, false, false);
	MPU6050_setMemoryStartAddress(address);
	uint8_t chunkSize;
	uint8_t *verifyBuffer = 0;
	uint8_t *progBuffer = 0;
	uint16_t i;
	uint8_t j;
	if (verify) verifyBuffer = (uint8_t *)malloc(MPU6050_DMP_MEMORY_CHUNK_SIZE);
	//if (useProgMem) progBuffer = (uint8_t *)malloc(MPU6050_DMP_MEMORY_CHUNK_SIZE);
	for (i = 0; i < dataSize;) {
		// determine correct chunk size according to bank position and data size
		chunkSize = MPU6050_DMP_MEMORY_CHUNK_SIZE;

		// make sure we don't go past the data size
		if (i + chunkSize > dataSize) chunkSize = dataSize - i;

		// make sure this chunk doesn't go past the bank boundary (256 bytes)
		if (chunkSize > 256 - address) chunkSize = 256 - address;
        
		
		// write the chunk of data as specified
		progBuffer = (uint8_t *)data + i;
		
		mpu6050_writeBytes(MPU6050_RA_MEM_R_W, chunkSize, progBuffer);

		//I2Cdev::writeBytes(devAddr, MPU6050_RA_MEM_R_W, chunkSize, progBuffer, wireObj);

		// verify data if needed
		if (verify && verifyBuffer) {
			MPU6050_setMemoryBank(bank, false, false);
			MPU6050_setMemoryStartAddress(address);
			HAL_I2C_Mem_Read(i2c, MPU6050_ADDRESS, MPU6050_RA_MEM_R_W, 1, verifyBuffer, chunkSize, I2C_TIMEOUT);
			//I2Cdev::readBytes(devAddr, MPU6050_RA_MEM_R_W, chunkSize, verifyBuffer, I2Cdev::readTimeout, wireObj);
			if (memcmp(progBuffer, verifyBuffer, chunkSize) != 0) {
				/*Serial.print("Block write verification error, bank ");
				Serial.print(bank, DEC);
				Serial.print(", address ");
				Serial.print(address, DEC);
				Serial.print("!\nExpected:");
				for (j = 0; j < chunkSize; j++) {
				    Serial.print(" 0x");
				    if (progBuffer[j] < 16) Serial.print("0");
				    Serial.print(progBuffer[j], HEX);
				}
				Serial.print("\nReceived:");
				for (uint8_t j = 0; j < chunkSize; j++) {
				    Serial.print(" 0x");
				    if (verifyBuffer[i + j] < 16) Serial.print("0");
				    Serial.print(verifyBuffer[i + j], HEX);
				}
				Serial.print("\n");*/
				free(verifyBuffer);
				
				return false; // uh oh.
			}
		}

		// increase byte index by [chunkSize]
		i += chunkSize;

		// uint8_t automatically wraps to 0 at 256
		address += chunkSize;

		// if we aren't done, update bank (if necessary) and address
		if (i < dataSize) {
			if (address == 0) bank++;
			MPU6050_setMemoryBank(bank, false, false);
			MPU6050_setMemoryStartAddress(address);
		}
	}
	if (verify) free(verifyBuffer);
	return true;
}

bool MPU6050_writeProgMemoryBlock(const uint8_t *data, uint16_t dataSize, uint8_t bank, uint8_t address, bool verify) {
	return MPU6050_writeMemoryBlock(data, dataSize, bank, address, verify);
}

// DMP_CFG_1 register

uint8_t MPU6050_getDMPConfig1() {
	uint8_t tmp;
	HAL_I2C_Mem_Read(i2c, MPU6050_ADDRESS, MPU6050_RA_DMP_CFG_1, 1, &tmp, 1, I2C_TIMEOUT);
	//I2Cdev::readByte(devAddr, MPU6050_RA_DMP_CFG_1, buffer, I2Cdev::readTimeout, wireObj);
	return tmp;
}
void MPU6050_setDMPConfig1(uint8_t config) {
	HAL_I2C_Mem_Write(i2c, MPU6050_ADDRESS, MPU6050_RA_DMP_CFG_1, 1, &config, 1, I2C_TIMEOUT);
	//I2Cdev::writeByte(devAddr, MPU6050_RA_DMP_CFG_1, config, wireObj);
}

// DMP_CFG_2 register

uint8_t MPU6050_getDMPConfig2() {
	uint8_t tmp;
	HAL_I2C_Mem_Read(i2c, MPU6050_ADDRESS, MPU6050_RA_DMP_CFG_2, 1, &tmp, 1, I2C_TIMEOUT);
	//I2Cdev::readByte(devAddr, MPU6050_RA_DMP_CFG_2, buffer, I2Cdev::readTimeout, wireObj);
	return tmp;
}
void MPU6050_setDMPConfig2(uint8_t config) {
	HAL_I2C_Mem_Write(i2c, MPU6050_ADDRESS, MPU6050_RA_DMP_CFG_2, 1, &config, 1, I2C_TIMEOUT);
	//I2Cdev::writeByte(devAddr, MPU6050_RA_DMP_CFG_2, config, wireObj);
}

void MPU6050_setFIFOEnabled(bool enabled) {
	
	MPU6050_writeBits(MPU6050_RA_USER_CTRL, MPU6050_USERCTRL_FIFO_EN_BIT, 1, enabled);
}

void MPU6050_resetDMP() {
	//MPU6050_writeBits(MPU6050_RA_USER_CTRL, MPU6050_USERCTRL_DMP_RESET_BIT, 1, 1);
	MPU6050_writeBit(MPU6050_RA_USER_CTRL, MPU6050_USERCTRL_DMP_RESET_BIT, true);
}

void MPU6050_setDMPEnabled(bool enabled) {
	//MPU6050_writeBits(MPU6050_RA_USER_CTRL, MPU6050_USERCTRL_DMP_EN_BIT, 1, 1);
	MPU6050_writeBit(MPU6050_RA_USER_CTRL, MPU6050_USERCTRL_DMP_EN_BIT, enabled);
}

void MPU6050_resetFIFO() {
	//MPU6050_writeBits(MPU6050_RA_USER_CTRL, MPU6050_USERCTRL_FIFO_RESET_BIT, 1, 1);
	MPU6050_writeBit(MPU6050_RA_USER_CTRL, MPU6050_USERCTRL_FIFO_RESET_BIT, true);
}

uint8_t MPU6050_dmpInitialize() {
	// reset device
	//DEBUG_PRINTLN(F("\n\nResetting MPU6050..."));
	MPU6050_DeviceReset(1);
	HAL_Delay(30); // wait after reset

	// enable sleep mode and wake cycle
	/*Serial.println(F("Enabling sleep mode..."));
	setSleepEnabled(true);
	Serial.println(F("Enabling wake cycle..."));
	setWakeCycleEnabled(true);*/

	// disable sleep mode
	MPU6050_SetSleepEnabled(0);

	// get MPU hardware revision
	MPU6050_setMemoryBank(0x10, true, true);
	MPU6050_setMemoryStartAddress(0x06);
	/*DEBUG_PRINTLN(F("Checking hardware revision..."));
	DEBUG_PRINT(F("Revision @ user[16][6] = "));
	DEBUG_PRINTLN(readMemoryByte());
	DEBUG_PRINTLN(F("Resetting memory bank selection to 0..."));*/
	MPU6050_setMemoryBank(0, false, false);

	// check OTP bank valid
	/*DEBUG_PRINTLN(F("Reading OTP bank valid flag..."));
	DEBUG_PRINT(F("OTP bank is "));
	DEBUG_PRINTLN(getOTPBankValid() ? F("valid!") : F("invalid!"));

	// setup weird slave stuff (?)
	DEBUG_PRINTLN(F("Setting slave 0 address to 0x7F..."));*/
	MPU6050_setSlaveAddress(0, 0x7F);
	//DEBUG_PRINTLN(F("Disabling I2C Master mode..."));
	MPU6050_setI2CMasterModeEnabled(false);
	//DEBUG_PRINTLN(F("Setting slave 0 address to 0x68 (self)..."));
	MPU6050_setSlaveAddress(0, 0x68);
	//DEBUG_PRINTLN(F("Resetting I2C Master control..."));
	MPU6050_resetI2CMaster();
	HAL_Delay(20);
	//DEBUG_PRINTLN(F("Setting clock source to Z Gyro..."));
	MPU6050_SetClockSource(MPU6050_CLOCK_PLL_ZGYRO);

	//DEBUG_PRINTLN(F("Setting DMP and FIFO_OFLOW interrupts enabled..."));
	MPU6050_SetIntEnableRegister(1 << MPU6050_INTERRUPT_FIFO_OFLOW_BIT | 1 << MPU6050_INTERRUPT_DMP_INT_BIT);

	//DEBUG_PRINTLN(F("Setting sample rate to 200Hz..."));
	MPU6050_setRate(4); // 1khz / (1 + 4) = 200 Hz

	//DEBUG_PRINTLN(F("Setting external frame sync to TEMP_OUT_L[0]..."));
	MPU6050_setExternalFrameSync(MPU6050_EXT_SYNC_TEMP_OUT_L);

	//DEBUG_PRINTLN(F("Setting DLPF bandwidth to 42Hz..."));
	MPU6050_SetDlpf(MPU6050_DLPF_BW_42);

	//DEBUG_PRINTLN(F("Setting gyro sensitivity to +/- 2000 deg/sec..."));
	MPU6050_SetFullScaleGyroRange(MPU6050_GYRO_FS_2000);

	// load DMP code into memory banks
	//DEBUG_PRINT(F("Writing DMP code to MPU memory banks ("));
	//DEBUG_PRINT(MPU6050_DMP_CODE_SIZE);
	//DEBUG_PRINTLN(F(" bytes)"));
	if (!MPU6050_writeProgMemoryBlock(dmpMemory, MPU6050_DMP_CODE_SIZE, 0, 0, true)) return 1; // Failed
	//DEBUG_PRINTLN(F("Success! DMP code written and verified."));

	// Set the FIFO Rate Divisor int the DMP Firmware Memory
	unsigned char dmpUpdate[] = { 0x00, MPU6050_DMP_FIFO_RATE_DIVISOR };
	MPU6050_writeMemoryBlock(dmpUpdate, 0x02, 0x02, 0x16, true); // Lets write the dmpUpdate data to the Firmware image, we have 2 bytes to write in bank 0x02 with the Offset 0x16

	//write start address MSB into register
	MPU6050_setDMPConfig1(0x03);
	//write start address LSB into register
	MPU6050_setDMPConfig2(0x00);

	//DEBUG_PRINTLN(F("Clearing OTP Bank flag..."));
	MPU6050_setOTPBankValid(false);

	//DEBUG_PRINTLN(F("Setting motion detection threshold to 2..."));
	MPU6050_SetMotionDetectionThreshold(2);

	//DEBUG_PRINTLN(F("Setting zero-motion detection threshold to 156..."));
	MPU6050_SetZeroMotionDetectionThreshold(156);

	//DEBUG_PRINTLN(F("Setting motion detection duration to 80..."));
	MPU6050_SetMotionDetectionDuration(80);

	//DEBUG_PRINTLN(F("Setting zero-motion detection duration to 0..."));
	MPU6050_SetZeroMotionDetectionDuration(0);
	//DEBUG_PRINTLN(F("Enabling FIFO..."));
	MPU6050_setFIFOEnabled(true);

	//DEBUG_PRINTLN(F("Resetting DMP..."));
	MPU6050_resetDMP();

	//DEBUG_PRINTLN(F("DMP is good to go! Finally."));

	//DEBUG_PRINTLN(F("Disabling DMP (you turn it on later)..."));
	MPU6050_setDMPEnabled(false);

	//DEBUG_PRINTLN(F("Setting up internal 42-byte (default) DMP packet buffer..."));
	dmpPacketSize = 42;

	//DEBUG_PRINTLN(F("Resetting FIFO and clearing INT status one last time..."));
	MPU6050_resetFIFO();
	MPU6050_GetIntStatusRegister();

	return 0; // success
}

/** Get current FIFO buffer size.
 * This value indicates the number of bytes stored in the FIFO buffer. This
 * number is in turn the number of bytes that can be read from the FIFO buffer
 * and it is directly proportional to the number of samples available given the
 * set of sensor data bound to be stored in the FIFO (register 35 and 36).
 * @return Current FIFO buffer size
 */
uint16_t MPU6050_getFIFOCount() {
	
	HAL_I2C_Mem_Read(i2c, MPU6050_ADDRESS, MPU6050_RA_FIFO_COUNTH, 1, mbuffer, 2, I2C_TIMEOUT);
	//I2Cdev::readBytes(devAddr, MPU6050_RA_FIFO_COUNTH, 2, buffer, I2Cdev::readTimeout, wireObj);
	return (((uint16_t)mbuffer[0]) << 8) | mbuffer[1];
}

/** Get timeout to get a packet from FIFO buffer.
 * @return Current timeout to get a packet from FIFO buffer
 * @see MPU6050_FIFO_DEFAULT_TIMEOUT
 */
uint32_t MPU6050_getFIFOTimeout() {
	return fifoTimeout;
}

void MPU6050_getFIFOBytes(uint8_t *data, uint8_t length) {
	if (length > 0) {
		//I2Cdev::readBytes(devAddr, MPU6050_RA_FIFO_R_W, length, data, I2Cdev::readTimeout, wireObj);
		mpu6050_register_read(MPU6050_RA_FIFO_R_W, data, length);
	}
	else {
		*data = 0;
	}
}


/** Get latest byte from FIFO buffer no matter how much time has passed.
 * ===                  GetCurrentFIFOPacket                    ===
 * ================================================================
 * Returns 1) when nothing special was done
 *         2) when recovering from overflow
 *         0) when no valid data is available
 * ================================================================ */
int8_t MPU6050_GetCurrentFIFOPacket(uint8_t *data, uint8_t length) {
	// overflow proof
	int16_t fifoC;
	// This section of code is for when we allowed more than 1 packet to be acquired
	uint32_t BreakTimer = HAL_GetTick();
	bool packetReceived = false;
	do {
		if ((fifoC = MPU6050_getFIFOCount())  > length) {

			if (fifoC > 200) {
				// if you waited to get the FIFO buffer to > 200 bytes it will take longer to get the last packet in the FIFO Buffer than it will take to  reset the buffer and wait for the next to arrive
				MPU6050_resetFIFO(); // Fixes any overflow corruption
				fifoC = 0;
				while (!(fifoC = MPU6050_getFIFOCount()) && ((HAL_GetTick() - BreakTimer) <= (MPU6050_getFIFOTimeout()))) ; // Get Next New Packet
			}
			else {
				//We have more than 1 packet but less than 200 bytes of data in the FIFO Buffer
				uint8_t Trash[I2C_BUFFER_LENGTH];
				while ((fifoC = MPU6050_getFIFOCount()) > length) {
					// Test each time just in case the MPU is writing to the FIFO Buffer
					fifoC = fifoC - length; // Save the last packet
					uint16_t  RemoveBytes;
					while (fifoC) {
						// fifo count will reach zero so this is safe
						RemoveBytes = (fifoC < I2C_BUFFER_LENGTH) ? fifoC : I2C_BUFFER_LENGTH; // Buffer Length is different than the packet length this will efficiently clear the buffer
						MPU6050_getFIFOBytes(Trash, (uint8_t)RemoveBytes);
						fifoC -= RemoveBytes;
					}
				}
			}
		}
		if (!fifoC) return 0; // Called too early no data or we timed out after FIFO Reset
		// We have 1 packet
		packetReceived = fifoC == length;
		if (!packetReceived && (HAL_GetTick() - BreakTimer) > (MPU6050_getFIFOTimeout())) return 0;
	} while (!packetReceived);
	MPU6050_getFIFOBytes(data, length); //Get 1 packet
	return 1;
}

uint8_t MPU6050_dmpGetQuaternion(int32_t *data, const uint8_t* packet) {
	// TODO: accommodate different arrangements of sent data (ONLY default supported now)
	if (packet == 0) packet = dmpPacketBuffer;
	data[0] = (((uint32_t)packet[0] << 24) | ((uint32_t)packet[1] << 16) | ((uint32_t)packet[2] << 8) | packet[3]);
	data[1] = (((uint32_t)packet[4] << 24) | ((uint32_t)packet[5] << 16) | ((uint32_t)packet[6] << 8) | packet[7]);
	data[2] = (((uint32_t)packet[8] << 24) | ((uint32_t)packet[9] << 16) | ((uint32_t)packet[10] << 8) | packet[11]);
	data[3] = (((uint32_t)packet[12] << 24) | ((uint32_t)packet[13] << 16) | ((uint32_t)packet[14] << 8) | packet[15]);
	return 0;
}
uint8_t MPU6050_dmpGetQuaternion(int16_t *data, const uint8_t* packet) {
	// TODO: accommodate different arrangements of sent data (ONLY default supported now)
	if (packet == 0) packet = dmpPacketBuffer;
	data[0] = ((packet[0] << 8) | packet[1]);
	data[1] = ((packet[4] << 8) | packet[5]);
	data[2] = ((packet[8] << 8) | packet[9]);
	data[3] = ((packet[12] << 8) | packet[13]);
	return 0;
}
uint8_t MPU6050_dmpGetQuaternion(Quaternion *q, const uint8_t* packet) {
	// TODO: accommodate different arrangements of sent data (ONLY default supported now)
	int16_t qI[4];
	uint8_t status = MPU6050_dmpGetQuaternion(qI, packet);
	if (status == 0) {
		q->w = (float)qI[0] / 16384.0f;
		q->x = (float)qI[1] / 16384.0f;
		q->y = (float)qI[2] / 16384.0f;
		q->z = (float)qI[3] / 16384.0f;
		return 0;
	}
	return status; // int16 return value, indicates error if this line is reached
}

// uint8_t MPU6050_6Axis_MotionApps20::dmpGetGyroAndAccelSensor(long *data, const uint8_t* packet);
// uint8_t MPU6050_6Axis_MotionApps20::dmpGetGyroSensor(long *data, const uint8_t* packet);
// uint8_t MPU6050_6Axis_MotionApps20::dmpGetControlData(long *data, const uint8_t* packet);
// uint8_t MPU6050_6Axis_MotionApps20::dmpGetTemperature(long *data, const uint8_t* packet);
// uint8_t MPU6050_6Axis_MotionApps20::dmpGetGravity(long *data, const uint8_t* packet);
uint8_t MPU6050_dmpGetGravity(int16_t *data, const uint8_t* packet) {
	/* +1g corresponds to +8192, sensitivity is 2g. */
	int16_t qI[4];
	uint8_t status = MPU6050_dmpGetQuaternion(qI, packet);
	data[0] = ((int32_t)qI[1] * qI[3] - (int32_t)qI[0] * qI[2]) / 16384;
	data[1] = ((int32_t)qI[0] * qI[1] + (int32_t)qI[2] * qI[3]) / 16384;
	data[2] = ((int32_t)qI[0] * qI[0] - (int32_t)qI[1] * qI[1]
		   - (int32_t)qI[2] * qI[2] + (int32_t)qI[3] * qI[3]) / (int32_t)(2 * 16384L);
	return status;
}

uint8_t MPU6050_dmpGetGravity(VectorFloat *v, Quaternion *q) {
	v->x = 2 * (q->x*q->z - q->w*q->y);
	v->y = 2 * (q->w*q->x + q->y*q->z);
	v->z = q->w*q->w - q->x*q->x - q->y*q->y + q->z*q->z;
	return 0;
}

uint8_t MPU6050_dmpGetYawPitchRoll(float *data, Quaternion *q, VectorFloat *gravity) {
	// yaw: (about Z axis)
	data[0] = atan2(2*q->x*q->y - 2*q->w*q->z, 2*q->w*q->w + 2*q->x*q->x - 1);
	// pitch: (nose up/down, about Y axis)
	data[1] = atan2(gravity->x, sqrt(gravity->y*gravity->y + gravity->z*gravity->z));
	// roll: (tilt left/right, about X axis)
	data[2] = atan2(gravity->y, gravity->z);
	if (gravity->z < 0) {
		if (data[1] > 0) {
			data[1] = M_PI - data[1]; 
		}
		else { 
			data[1] = -M_PI - data[1];
		}
	}
	return 0;
}
// Nothing else changed
