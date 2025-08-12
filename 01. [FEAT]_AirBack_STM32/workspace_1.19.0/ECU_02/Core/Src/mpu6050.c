#include "mpu6050.h"

// MPU6050 초기화
void MPU6050_Init(I2C_HandleTypeDef *hi2c) {
    uint8_t check;
    uint8_t data;

    // WHO_AM_I 레지스터 읽기 (통신 확인)
    HAL_I2C_Mem_Read(hi2c, MPU6050_ADDR, WHO_AM_I_REG, 1, &check, 1, 1000);

    if (check == 0x68) { // MPU-6050의 WHO_AM_I 값
        // 전원 관리 레지스터 설정 (슬립 모드 해제)
        data = 0;
        HAL_I2C_Mem_Write(hi2c, MPU6050_ADDR, PWR_MGMT_1_REG, 1, &data, 1, 1000);

        // 샘플 레이트 분주기 설정 (1KHz)
        data = 0x07; // 1KHz / (7+1) = 125Hz
        HAL_I2C_Mem_Write(hi2c, MPU6050_ADDR, SMPLRT_DIV_REG, 1, &data, 1, 1000);

        // 가속도 센서 설정 (+-2g)
        data = 0x00; // FS_SEL_0
        HAL_I2C_Mem_Write(hi2c, MPU6050_ADDR, ACCEL_CONFIG_REG, 1, &data, 1, 1000);
    }
}

// 가속도 데이터 읽기
void MPU6050_Read_Accel(I2C_HandleTypeDef *hi2c, int16_t *accel_data) {
    uint8_t Rx_data[6];

    // X, Y, Z 축 가속도 데이터 읽기 (6바이트)
    HAL_I2C_Mem_Read(hi2c, MPU6050_ADDR, ACCEL_XOUT_H_REG, 1, Rx_data, 6, 1000);

    // 16비트 값으로 결합
    accel_data[0] = (int16_t)(Rx_data[0] << 8 | Rx_data[1]);
    accel_data[1] = (int16_t)(Rx_data[2] << 8 | Rx_data[3]);
    accel_data[2] = (int16_t)(Rx_data[4] << 8 | Rx_data[5]);
}
