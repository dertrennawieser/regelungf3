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
#include "adc.h"
#include "itm.h"
#include "systeminit.h"

#include "mpu6050.h"

#define CYCLE_TIME 0.001f

#define GYRO_GEW 0.998f

#define CO 1
#define LIMIT 5000
#define GAS_LIMIT 2000

extern uint32_t SystemCoreClock;
extern uint32_t APB1Clock;

uint8_t sendBuffer[2];

float KP = 9; // 3.5
float KD;
float KI;

typedef struct
{
	float angle;
	float errsum;
	float lasterr;
	float accvalues_deg;
	int16_t control;
	int16_t gyrovalues;
	int16_t accvalues;

}axis;

float accmag=0;
float output = 0;

int32_t iErr;
uint32_t speed[4];

int32_t gas = 1500;

axis x, y, z;
void calculateAngels();
int32_t pid(axis*);

void TIM7_IRQHandler()
{
	CLEAR_BIT(TIM7->SR, TIM_SR_UIF);
	iwdg_refresh();

	GPIOA->ODR ^= (GPIO_ODR_6);
	calculateAngels();


	x.control = pid(&x);
	y.control = pid(&y);

	//Berechung der Stellgröße (PID + Gas)
	speed[0] = gas+x.control-y.control;//gas-correctX+correctY;
	speed[1] = gas-x.control+y.control;//gas-correctX-correctY;
	speed[2] = gas+x.control+y.control;
	speed[3] = gas-x.control-y.control;

	/*
	 * M3	 M1     Y<------O
	 * 	\	/	    		|
	 * 	 \_/				|
	 * 	 / \				|
	 *  /   \				V
	 * M0    M2				X
	*/

	//PWM-Signal Begrenzung
	for(int i=0; i<4; i++)
	{
		if(speed[i] < 1000)
			speed[i] = 1000;

		if(speed[i] > GAS_LIMIT)
			speed[i] = GAS_LIMIT;
	}

	TIM3->CCR1 = speed[0];
	TIM3->CCR2 = speed[1];
	TIM3->CCR3 = speed[2];
	TIM3->CCR4 = speed[3];
}

void armMotors()
{
	ITM_SendChar('1');

	TIM3->CCR1 = 1000;
	TIM3->CCR2 = 1000;
	TIM3->CCR3 = 1000;
	TIM3->CCR4 = 1000;
	wait_ms(2000);

	ITM_SendChar('2');

	TIM3->CCR1 = 1300;
	TIM3->CCR2 = 1300;
	TIM3->CCR3 = 1300;
	TIM3->CCR4 = 1300;
	wait_ms(1000);

	ITM_SendChar('3');

	TIM3->CCR1 = 1000;
	TIM3->CCR2 = 1000;
	TIM3->CCR3 = 1000;
	TIM3->CCR4 = 1000;
	wait_ms(2000);

	ITM_SendChar('f');

}

void calibrateMotors()
{
	ITM_SendChar('1');

	TIM3->CCR1 = 2000;
	TIM3->CCR2 = 2000;
	TIM3->CCR3 = 2000;
	TIM3->CCR4 = 2000;
	wait_ms(6000);

	ITM_SendChar('2');

	TIM3->CCR1 = 1000;
	TIM3->CCR2 = 1000;
	TIM3->CCR3 = 1000;
	TIM3->CCR4 = 1000;
	wait_ms(6000);

	ITM_SendChar('f');

}

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
	//SET_BIT(GPIOA->BSRR, GPIO_BSRR_BS_6);
	getAcc_calibrated();
	getGyro();
	//SET_BIT(GPIOA->BSRR, GPIO_BSRR_BR_6);

	//Winkel aus Beschleunigungssensorwerten berechnen mit Eulerformel
	/*y.accvalues_deg = atan2f(((float)y.accvalues/16384),sqrtf(((float)x.accvalues/16384)*((float)x.accvalues/16384)
			+((float)z.accvalues/16384)*((float)z.accvalues/16384)))*180/(float)M_PI;

	x.accvalues_deg = atan2f(((float)x.accvalues/16384),sqrtf(((float)y.accvalues/16384)*((float)y.accvalues/16384)
			+((float)z.accvalues/16384)*((float)z.accvalues/16384)))*180/(float)M_PI;*/

	/*ITM_SendString("\n\ndeltaT: ");
					ITM_SendInt(TIM2->CNT);
					ITM_SendChar('\n');*/

	//y.accvalues_deg = atan2f(((float)y.accvalues/16384),(float)z.accvalues/16384)*180/(float)M_PI;
	accmag = sqrtf(x.accvalues*x.accvalues + y.accvalues*y.accvalues + z.accvalues*z.accvalues)/16384;


	if((accmag >1.1f) || (accmag < 0.9f))
	{
		x.angle = x.angle + ((y.gyrovalues)/32.8f) * CYCLE_TIME;
		y.angle = y.angle + ((x.gyrovalues)/32.8f) * CYCLE_TIME;
	}
	else
	{
		y.accvalues_deg = -atan2f(((float)y.accvalues/16384),sqrtf(((float)x.accvalues/16384)*((float)x.accvalues/16384)
				+((float)z.accvalues/16384)*((float)z.accvalues/16384)))*180/(float)M_PI;

		x.accvalues_deg = -atan2f((-(float)x.accvalues/16384),sqrtf(((float)y.accvalues/16384)*((float)y.accvalues/16384)
				+((float)z.accvalues/16384)*((float)z.accvalues/16384)))*180/(float)M_PI;



		//Absoluten Winkel berechnen mit Komplement�rfilter	x.gyroAngle = 0;
		//deltaT = READ_REG(TIM6->CNT);
		x.angle = GYRO_GEW * (x.angle + ((y.gyrovalues)/32.8f)*CYCLE_TIME) + (1-GYRO_GEW)*x.accvalues_deg;
		y.angle = GYRO_GEW * (y.angle + ((x.gyrovalues)/32.8f)*CYCLE_TIME) + (1-GYRO_GEW)*y.accvalues_deg;
	}



	if(y.angle > 25)
	{
		__NOP();
	}

	//TIM2 (f�r Integration der Gyrowerte)
	//CLEAR_REG(TIM6->CNT);


	/*ITM_SendString("\n\ndeltaT: ");
	ITM_SendInt(deltaT);
	ITM_SendChar('\n');*/


	/*ITM_SendString(" Angle2: ");
	ITM_SendInt((int) y.angle);

	ITM_SendChar(',');
	ITM_SendInt(((int)(fabsf(y.angle)*10000)%10000));

	ITM_SendString("\nAngle1: ");
	ITM_SendInt((int) x.angle);

	ITM_SendChar(',');
	ITM_SendInt(((int)(fabsf(x.angle)*10000)%10000));
*/
	dac_write((uint16_t)(y.angle*11.375f + 2046));

}

int32_t pid(axis *a)
{
	//I-Anteil (ab Regeldifferenz<30°)
	if(a->angle<10 && a->angle>-10 )
	{
		//Fehler
		iErr =  (a->angle)/**deltaT/100000*/;

		//Aufaddierung des Fehlers mit Limitierung (I-Anteil)
		if(a->errsum + iErr < -LIMIT)
			a->errsum = -LIMIT;
		else if(a->errsum + iErr > LIMIT)
			a->errsum = LIMIT;
		else
			a->errsum += iErr;
	}
	//Regeldifferenz>5° => I-Anteil = 0
	else
	{
		if(a->errsum!=0)
			a->errsum = 0;
	}

	//PID:   [...P-Anteil...]   [...I-Anteil..]   [.....................D-Anteil.....................]
	output = KP*(a->angle) + KI*a->errsum + KD*(a->angle-a->lasterr)/CYCLE_TIME;

	//optionaler Koeffizient CO
	output *= CO;

	//Fehler speichern für nächsten Durchlauf (für D-Anteil)
	a->lasterr = (a->angle);

	//Limitierung für PWM
	if(output < -1000)
		return -1000;
	else if(output > 1000)
		return 1000;
	else
		return (int32_t) output;
}

void init()
{
	tim6_init(72, 0xFFFF);
	tim7_init(72, (uint16_t) (CYCLE_TIME*1000000.0f));

	tim3_pwminit(72, 20000);
}

int main(void)
{
	SystemCoreClock=72000000;
	APB1Clock=36000000;

	SysTick_Config(SystemCoreClock/1000);

	NVIC_SetPriority(I2C1_EV_IRQn, 0);
	NVIC_SetPriority(I2C1_ER_IRQn, 0);
	NVIC_SetPriority(TIM7_IRQn, 1);


	ITM_SendString("h");

	init();
	i2c_init(I2C1, true, 8000000);
	adc_init();

	SET_BIT(RCC->AHBENR, RCC_AHBENR_GPIOAEN);
	SET_BIT(RCC->AHBENR, RCC_AHBENR_GPIOBEN);
	SET_BIT(RCC->AHBENR, RCC_AHBENR_GPIOCEN);

	MODIFY_REG(GPIOA->MODER, GPIO_MODER_MODER5,  GPIO_MODER_MODER5_0);
	MODIFY_REG(GPIOA->MODER, GPIO_MODER_MODER6,  GPIO_MODER_MODER6_0);
	MODIFY_REG(GPIOC->MODER, GPIO_MODER_MODER0,  GPIO_MODER_MODER0_0);

	SET_BIT(GPIOC->BSRR, GPIO_BSRR_BS_0);

	//MODIFY_REG(GPIOB->MODER, GPIO_MODER_MODER5,  GPIO_MODER_MODER5_0);
	MODIFY_REG(GPIOB->PUPDR, GPIO_PUPDR_PUPDR0, GPIO_PUPDR_PUPDR0_0);

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

	wait_ms(100);
	mpu_connect(I2C1);
	wait_ms(10);

	mpu_init(2,0);
	wait_ms(10);

	mpu_calibrate(false);
	wait_ms(100);

	dac_init();

	/*data[0] = (1<<0);
	mpu_write(MPU_USER_CTRL, data, 1);*/
	WRITE_REG(GPIOA->BSRR, GPIO_BSRR_BS_5);

	NVIC_EnableIRQ(TIM7_IRQn);

	//Timer enable counter

	SET_BIT(TIM3->CR1, TIM_CR1_CEN);

	//calibrateMotors();
	armMotors(); //blheli 35a esc
	/*TIM3->CCR1 = 1300;
	wait_ms(1000);
	GPIOA->ODR ^= GPIO_ODR_5;
	TIM3->CCR1 = 1000;
	TIM3->CCR2 = 1300;
	wait_ms(1000);
	GPIOA->ODR ^= GPIO_ODR_5;
	TIM3->CCR2 = 1000;
	TIM3->CCR3 = 1300;
	wait_ms(1000);
	GPIOA->ODR ^= GPIO_ODR_5;
	TIM3->CCR3 = 1000;
	TIM3->CCR4 = 1300;
	wait_ms(1000);
	GPIOA->ODR ^= GPIO_ODR_5;
	TIM3->CCR4 = 1000;*/

	SET_BIT(TIM6->CR1, TIM_CR1_CEN);
	SET_BIT(TIM7->CR1, TIM_CR1_CEN);
	iwdg_init();

	//bool pInc = false;
	KP = 25.0f;
	KD = 3.0f;//1.65;
	KI = 0.045f;//0.01;

	while(1)
	{
		//__WFI();
		/*if(READ_BIT(GPIOB->IDR, GPIO_IDR_0) && pInc)
		{
			KI += 0.001;
			pInc = false;
			ITM_SendInt((int32_t)(KD * 1000));
			wait_ms(50);
		}
		else if (!READ_BIT(GPIOB->IDR, GPIO_IDR_0))
		{
			pInc = true;
			wait_ms(50);
		}*/


		while(adc_busy())
		{
			if(READ_BIT(GPIOB->IDR, GPIO_IDR_0))
			{

				CLEAR_BIT(TIM7->CR1, TIM_CR1_CEN);

				while(1)
				{
					if(TIM3->CCR1 > 1000)
						TIM3->CCR1 -= 1;
					if(TIM3->CCR2 > 1000)
						TIM3->CCR2 -= 1;
					if(TIM3->CCR3 > 1000)
						TIM3->CCR3 -= 1;
					if(TIM3->CCR4 > 1000)
						TIM3->CCR4 -= 1;


					iwdg_refresh();
					wait_ms(1);

				}
			}

		}


		gas = 1000 + adc_getnewdata() * 0.24f;
		ITM_SendInt(gas);
		adc_startconversion(2);

		wait_ms(10);

		//GPIOA->ODR ^= (GPIO_ODR_6);
	}
}
