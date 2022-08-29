/*
 * mpu6050.c
 *
 *  Created on: 15.07.2021
 *      Author: Moritz
 */

#include <stdint.h>
#include <stdbool.h>
#include "stm32f3xx.h"

#include "i2c.h"
#include "mpu6050.h"
#include "itm.h"


static I2C_TypeDef* i2c_reg;


/*
 * delay loop for 100MHz, compiler optimization flag -Og
 * needs to be reconfigured for different system clocks and optimization flags
 */
static void delay(uint32_t msec)
{
	for (uint32_t j=0; j < 6710UL * msec; j++)
	{
		__NOP();
	}
}

void mpu_write(uint8_t reg_addr, uint8_t* data, uint8_t data_len)
{
	static uint8_t mpu_buffer[TXBUFFER_LEN] = {0};
	mpu_buffer[0] = reg_addr;
	for(int i=1; i<=data_len; i++)
	{
		mpu_buffer[i] = *data;
		data++;
	}
	i2c_start(i2c_reg, MPU_ADDRESS, mpu_buffer, data_len+1, 0, 0);
	while(i2c_busy(i2c_reg));
}

void mpu_read(uint8_t reg_addr, uint8_t* data, uint8_t data_len)
{
	i2c_start(i2c_reg, MPU_ADDRESS, &reg_addr, 1, data, data_len);
	while(i2c_busy(i2c_reg));
}

bool mpu_connect(I2C_TypeDef* reg_ptr)
{
	uint8_t id;
	uint8_t reset_val;
	static uint8_t datac[1] = {0};

	i2c_reg = reg_ptr;
	i2c_init(i2c_reg, true, APB1Clock);		//i2c with fast mode, does nothing if i2c already initialized

	datac[0] = (1<<7);
	mpu_write(MPU_PWR_MGMT_1, datac, 1);		//swrst mpu

	do
	{
		mpu_read(MPU_PWR_MGMT_1, &reset_val, 1);
	}while(reset_val != 0x40);				//wait until mpu is reset

	datac[0] = 0x00;
	mpu_write(MPU_PWR_MGMT_1, datac, 1);

	delay(100);		//wait for gyro startup, needed before calibration

	mpu_read(MPU_WHO_AM_I, &id, 1);
	if(id == 0x68)
		return true;
	else
		return false;
}

/*
 * set accel and gyro full scale
 */
void mpu_init(uint8_t gyro_fs, uint8_t accel_fs)
{
	static uint8_t datafs[1] = {0};

	datafs[0] = (gyro_fs<<3);
	mpu_write(MPU_GYRO_CONFIG, datafs, 1);

	datafs[0] = (accel_fs<<3);
	mpu_write(MPU_ACCEL_CONFIG, datafs, 1);
}

/*
 * calibrate gyroscope, mpu must be stationary
 * Parameter 1 ALWAYS false
 */
void mpu_calibrate(bool calibrateAccel)
{
	//uint8_t tmp = 0;
	uint8_t gyro_fs_temp = 0;
	uint8_t accel_fs_temp = 0;
	static uint8_t data[6] = {0};

	mpu_read(MPU_GYRO_CONFIG, &gyro_fs_temp, 1);	//save current fs settings
	gyro_fs_temp = (gyro_fs_temp & 0b00011000)>>3;

	mpu_read(MPU_ACCEL_CONFIG, &accel_fs_temp, 1);
	accel_fs_temp = (accel_fs_temp & 0b00011000)>>3;

	mpu_init(2, 2);	//fs settings for cal



	uint8_t gyro_data[6];
	int16_t gyro_values[3];
	int32_t gyro_offset[3] = {0,0,0};

	for(int i=0; i<SUM_LEN; i++)
	{
		mpu_read(MPU_GYRO_REG, gyro_data, 6);
		for(int j = 0; j<3; j++)
		{
			gyro_values[j] = (gyro_data[j*2]<<8) | gyro_data[j*2+1];
			gyro_offset[j] += gyro_values[j];
		}
		delay(1);
	}
	for(int j = 0; j<3; j++)
	{
		gyro_offset[j] /= SUM_LEN;
		gyro_offset[j] = -gyro_offset[j];
		data[j*2] = (gyro_offset[j]>>8) & 0xFF;
		data[j*2+1] = gyro_offset[j] & 0xFF;
	}
	mpu_write(MPU_XG_OFFS_USRH, data, 6);

	if(calibrateAccel)
	{

		uint8_t accel_data[6] = {0};
		int16_t accel_values[3] = {0};
		int32_t accel_offset[3] = {0};
		int16_t accel_factory_offset[3] = {0};
		uint8_t mask_bit[3] = {0};

		//mpu_read(MPU_ACCEL_CONFIG, &tmp, 1);

		for(int i=0; i<SUM_LEN; i++)
		{
			mpu_read(MPU_ACCEL_REG, accel_data, 6);
			for(int j = 0; j<3; j++)
			{
				accel_values[j] = (accel_data[j*2]<<8) | accel_data[j*2+1];
				accel_offset[j] += accel_values[j];
			}
			delay(1);
		}
		for(int j = 0; j<3; j++)
		{
			accel_offset[j] /= SUM_LEN;
		}
		if (accel_offset[2] > 0L)
			accel_offset[2] -= 4096;
		else
			accel_offset[2] += 4096;

		ITM_SendInt(accel_offset[0]);
		ITM_SendChar('\n');
		ITM_SendInt(accel_offset[1]);
		ITM_SendChar('\n');
		ITM_SendInt(accel_offset[2]);
		ITM_SendChar('\n');

		mpu_read(MPU_XA_OFFS_USRH, accel_data, 6);
		for(int j = 0; j<3; j++)
		{
			ITM_SendInt(accel_data[j*2]);
			ITM_SendChar('\n');
			ITM_SendInt(accel_data[j*2+1]);
			ITM_SendChar('\n');

			accel_factory_offset[j] = (accel_data[j*2]<<8) | accel_data[j*2+1];

			ITM_SendInt(accel_factory_offset[j]);
			ITM_SendChar('\n');

			if(READ_BIT(accel_data[j*2+1], 0x01))
				mask_bit[j] = 0x01;

			accel_factory_offset[j] -= accel_offset[j];

			ITM_SendInt(accel_factory_offset[j]);
			ITM_SendChar('\n');


			data[j*2] = (accel_factory_offset[j]>>8) & 0xFF;
			data[j*2+1] = (accel_factory_offset[j] & 0xFF);

			if(mask_bit[j])
				SET_BIT(data[j*2+1], 0x01);
			else
				CLEAR_BIT(data[j*2+1], 0x01);

			ITM_SendInt(data[j*2]);
			ITM_SendChar('\n');
			ITM_SendInt(data[j*2+1]);
			ITM_SendChar('\n');
		}

		mpu_write(MPU_XA_OFFS_USRH, data, 6);
	}

	mpu_init(gyro_fs_temp, accel_fs_temp);		//restore previous fs settings
}
