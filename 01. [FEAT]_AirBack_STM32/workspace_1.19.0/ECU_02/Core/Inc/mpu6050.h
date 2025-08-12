#ifndef INC_MPU6050_H_
#define INC_MPU6050_H_

#include "main.h"

#define MPU6050_ADDR 0xD0 // ADDR 핀이 GND에 연결되었을 때 (0x68 << 1)

// MPU6050 Register Addresses
#define WHO_AM_I_REG 0x75
#define PWR_MGMT_1_REG 0x6B
#define SMPLRT_DIV_REG 0x19
#define CONFIG_REG 0x1A
#define GYRO_CONFIG_REG 0x1B
#define ACCEL_CONFIG_REG 0x1C
#define ACCEL_XOUT_H_REG 0x3B

void MPU6050_Init(I2C_HandleTypeDef *hi2c);
void MPU6050_Read_Accel(I2C_HandleTypeDef *hi2c, int16_t *accel_data);

#endif /* INC_MPU6050_H_ */
