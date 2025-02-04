/*
 * imu.c
 *
 *  Created on: Jan 22, 2025
 *      Author: torik
 */

#include "imu.h"

static void active_imu()
{
	HAL_GPIO_WritePin(SPI2_CS_GPIO_Port, SPI2_CS_Pin, RESET);
}
static void deactive_imu()
{
	HAL_GPIO_WritePin(SPI2_CS_GPIO_Port, SPI2_CS_Pin, SET);
}

void mpu9250_read(uint8_t address, uint8_t *data)
 {
 uint8_t temp_data = 0x80|address;
 active_imu();
 HAL_SPI_Transmit(&IMU_SPI, &temp_data , 1, 50);
 HAL_SPI_Receive(&IMU_SPI, data, 1, 50);
 deactive_imu();
 }
void mpu9250_write(uint8_t reg, uint8_t data)
 {
 active_imu();
 HAL_SPI_Transmit(&IMU_SPI, &reg, 1, 50);
 HAL_SPI_Transmit(&IMU_SPI, &data, 1, 50);
 deactive_imu();
 }
void mpu9250_init()
{
	mpu9250_write(MPU9250_REG_PWR_MGMT_1, 0xC1); // Reset registers & going to sleep mode & set available clock
	mpu9250_write(MPU9250_REG_PWR_MGMT_1, 0x01); // set available clock

	mpu9250_write(MPU9250_REG_SMPLRT_DIV, 0x00);
	mpu9250_write(MPU9250_REG_GYRO_CONFIG, 0x8); // set gyro
	mpu9250_write(MPU9250_REG_ACCEL_CONFIG_1, 0x8); // set accelerometer
	mpu9250_write(MPU9250_REG_ACCEL_CONFIG_2, 0xA); // set accelerometer low pass

	mpu9250_write(MPU9250_REG_USER_CTRL, 0x10);

	mpu9250_write(MPU9250_REG_CONFIG, 0x00); // active 250Hz Low pass filter
}
uint8_t mpu9250_read_data(mpu9250* data)
{
static uint8_t data_rx[14];
uint8_t temp_data = 0x80|MPU9250_REG_ACCEL_XOUT_H;
active_imu();
HAL_SPI_Transmit(&IMU_SPI, &temp_data, 1, 1000);
HAL_SPI_Receive(&IMU_SPI, data_rx, 14, 1000);
deactive_imu();
data ->x_accel = ((int16_t)data_rx[0]<<8)| (int16_t)data_rx[1];
data ->y_accel = ((int16_t)data_rx[2]<<8)| (int16_t) data_rx[3];
data ->z_accel = ((int16_t)data_rx[4]<<8)| (int16_t) data_rx[5];
data ->temp = ((int16_t)data_rx[6]<<8) | (int16_t) data_rx[7];
data ->x_gyro = ((int16_t)data_rx[8]<<8) | (int16_t)data_rx[9];
data ->y_gyro = ((int16_t)data_rx[10]<<8)| (int16_t)data_rx[11];
data ->z_gyro = ((int16_t)data_rx[12]<<8)| (int16_t)data_rx[13];
return 0;
}
