/*
 * mpu6050.h
 *
 *  Created on: 15.07.2021
 *      Author: Moritz
 */

#ifndef MPU6050_H_
#define MPU6050_H_


#define MPU_ADDRESS 0b1101000

#define MPU_XA_OFFS_USRH 0x06
#define MPU_XG_OFFS_USRH 0x13
#define MPU_GYRO_CONFIG 0x1B
#define MPU_ACCEL_CONFIG 0x1C
#define MPU_ACCEL_REG 0x3B
#define MPU_GYRO_REG 0x43
#define MPU_SIGNAL_PATH_RESET 0x68
#define MPU_USER_CTRL 0x6A
#define MPU_PWR_MGMT_1 0x6B
#define MPU_WHO_AM_I 0x75

#define SUM_LEN 200
#define TXBUFFER_LEN 7
#define ACCEL_X_OFFSET 0
#define ACCEL_Y_OFFSET 0
#define ACCEL_Z_OFFSET 0

extern uint32_t APB1Clock;

void mpu_write(uint8_t, uint8_t*, uint8_t);
void mpu_read(uint8_t, uint8_t*, uint8_t);
bool mpu_connect(I2C_TypeDef*);
void mpu_init(uint8_t, uint8_t);
void mpu_calibrate(bool);

typedef struct
{
	int16_t accel, gyro;
	float angle;
	uint16_t dt;

}axis_s;

#endif /* MPU6050_H_ */
