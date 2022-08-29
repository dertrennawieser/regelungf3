/*
 * main.c
 *
 *  Created on: 29.08.2022
 *      Author: Moritz
 */

#include <stdint.h>
#include <math.h>
#include "stm32f3xx.h"

#include "i2c.h"
#include "tim.h"
#include "dac.h"
#include "itm.h"
#include "systeminit.h"

#include "mpu6050.h"

#define GYRO_GEW 0.998f

extern uint32_t SystemCoreClock;
extern uint32_t APB1Clock;

uint32_t deltaT = 0;

uint8_t sendBuffer[2];

typedef struct
{
	float angle;
	float errsum;
	float lasterr;
	float accvalues_deg;
	int16_t gyrovalues;
	int16_t accvalues;

}axis;

axis x, y, z;

void getAcc()
{
	uint8_t values_8bit[6];
	mpu_read(MPU_ACCEL_REG, values_8bit, 6);

	//values-8bit -> values_16bit
	x.accvalues = (values_8bit[0]<<8) | values_8bit[1];
	y.accvalues = (values_8bit[2]<<8) | values_8bit[3];
	z.accvalues = (values_8bit[4]<<8) | values_8bit[5];

}

/*
 * Accel oss=0 !!!!!!!!
 */
void getAcc_calibrated()
{
	getAcc();
	x.accvalues -= ACCEL_X_OFFSET;
	y.accvalues -= ACCEL_Y_OFFSET;
	z.accvalues -= ACCEL_Z_OFFSET;

	/*
	ITM_SendInt(x.accvalues);
	ITM_SendChar('\n');
	ITM_SendInt(y.accvalues);
	ITM_SendChar('\n');
	ITM_SendInt(z.accvalues);
	ITM_SendChar('\n');
	ITM_SendChar('\n');
	*/
}

void getGyro()
{
	uint8_t values_8bit[6];
	mpu_read(MPU_GYRO_REG, values_8bit, 6);

	//values-8bit -> values_16bit
	x.gyrovalues = (values_8bit[0]<<8) | values_8bit[1];
	y.gyrovalues = (values_8bit[2]<<8) | values_8bit[3];
	z.gyrovalues = (values_8bit[4]<<8) | values_8bit[5];

	/*
	ITM_SendInt(x.gyrovalues);
	ITM_SendChar('\n');
	ITM_SendInt(y.gyrovalues);
	ITM_SendChar('\n');
	ITM_SendInt(z.gyrovalues);
	ITM_SendChar('\n');
	ITM_SendChar('\n');
	*/
}

void calculateAngels()
{

	getAcc_calibrated();
	getGyro();

	//Winkel aus Beschleunigungssensorwerten berechnen mit Eulerformel
	/*y.accvalues_deg = atan2f(((float)y.accvalues/16384),sqrtf(((float)x.accvalues/16384)*((float)x.accvalues/16384)
			+((float)z.accvalues/16384)*((float)z.accvalues/16384)))*180/(float)M_PI;

	x.accvalues_deg = atan2f(((float)x.accvalues/16384),sqrtf(((float)y.accvalues/16384)*((float)y.accvalues/16384)
			+((float)z.accvalues/16384)*((float)z.accvalues/16384)))*180/(float)M_PI;*/

	/*ITM_SendString("\n\ndeltaT: ");
					ITM_SendInt(TIM2->CNT);
					ITM_SendChar('\n');*/

	//y.accvalues_deg = atan2f(((float)y.accvalues/16384),(float)z.accvalues/16384)*180/(float)M_PI;

	y.accvalues_deg = atan2f(((float)y.accvalues/16384),sqrtf(((float)x.accvalues/16384)*((float)x.accvalues/16384)
			+((float)z.accvalues/16384)*((float)z.accvalues/16384)))*180/(float)M_PI;

	x.accvalues_deg = -atan2f((-(float)x.accvalues/16384),sqrtf(((float)y.accvalues/16384)*((float)y.accvalues/16384)
			+((float)z.accvalues/16384)*((float)z.accvalues/16384)))*180/(float)M_PI;



	//Absoluten Winkel berechnen mit Komplement�rfilter	x.gyroAngle = 0;
	deltaT = READ_REG(TIM6->CNT);
	x.angle = GYRO_GEW * (x.angle + ((y.gyrovalues)/32.8f)*(((float)deltaT)/1000000)) + (1-GYRO_GEW)*x.accvalues_deg;
	y.angle = GYRO_GEW * (y.angle + ((x.gyrovalues)/32.8f)*(((float)deltaT)/1000000)) + (1-GYRO_GEW)*y.accvalues_deg;

	if(y.angle > 25)
	{
		__NOP();
	}

	//TIM2 (f�r Integration der Gyrowerte)
	CLEAR_REG(TIM6->CNT);


	/*ITM_SendString("\n\ndeltaT: ");
	ITM_SendInt(deltaT);
	ITM_SendChar('\n');*/


	ITM_SendString(" Angle2: ");
	ITM_SendInt((int) y.angle);

	ITM_SendChar(',');
	ITM_SendInt(((int)(fabsf(y.angle)*10000)%10000));

	ITM_SendString("\nAngle1: ");
	ITM_SendInt((int) x.angle);

	ITM_SendChar(',');
	ITM_SendInt(((int)(fabsf(x.angle)*10000)%10000));

}

void init()
{
	tim6_init(100, 0xFFFF);
	tim7_init(100, 0xFFFF);

	tim3_pwminit(100, 20000);
}

int main(void)
{
	SystemCoreClock=72000000;
	APB1Clock=36000000;

	SysTick_Config(SystemCoreClock/1000);

	bool b = false;


	ITM_SendString("h");

	init();
	i2c_init(I2C1, true, APB1Clock);

	SET_BIT(RCC->AHBENR, RCC_AHBENR_GPIOAEN);

	MODIFY_REG(GPIOA->MODER, GPIO_MODER_MODER5,  GPIO_MODER_MODER5_0);
	MODIFY_REG(GPIOA->MODER, GPIO_MODER_MODER6,  GPIO_MODER_MODER6_0);

	/*//init mpu6050
	mpu_sendbuffer[0] = MPU_PWR_MGMT_1;
	mpu_sendbuffer[1] = 0x00;
	i2c_start(I2C1, MPU_ADDRESS, mpu_sendbuffer, 2, 0, 0);
	//wait_ms(1);
	while(i2c_busy(I2C1));
	mpu_sendbuffer[0] = MPU_GYRO_CONFIG;
	mpu_sendbuffer[1] = 0x10;
	i2c_start(I2C1, MPU_ADDRESS, mpu_sendbuffer, 2, 0, 0);
	while(i2c_busy(I2C1));*/

	mpu_connect(I2C1);
	wait_ms(10);

	mpu_init(2,0);
	wait_ms(10);

	mpu_calibrate(false);
	wait_ms(100);

	/*data[0] = (1<<0);
	mpu_write(MPU_USER_CTRL, data, 1);*/
	WRITE_REG(GPIOA->BSRR, GPIO_BSRR_BS_5);

	//Timer enable counter
	SET_BIT(TIM6->CR1, TIM_CR1_CEN);
	SET_BIT(TIM7->CR1, TIM_CR1_CEN);
	while(1)
	{
		calculateAngels();

		TIM3->CCR1 = 10000;
		TIM3->CCR2 = 15000;
		TIM3->CCR3 = 5000;
		TIM3->CCR4 = 500;

		if(READ_BIT(TIM3->SR, TIM_SR_UIF))
		{
			CLEAR_BIT(TIM3->SR, TIM_SR_UIF);
		}





		if(b)
		{
			WRITE_REG(GPIOA->BSRR, GPIO_BSRR_BS_6);
			b=false;
		}
		else
		{
			WRITE_REG(GPIOA->BSRR, GPIO_BSRR_BR_6);
			b=true;
		}
	}
}
