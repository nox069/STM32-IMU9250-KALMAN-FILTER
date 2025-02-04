/*
 * imu.h
 *
 *  Created on: Jan 22, 2025
 *      Author: torik
 */

#ifndef INC_IMU_H_
#define INC_IMU_H_
#include <stdint.h>
#include <main.h>


#define GYRO_RANGE_VALUE        _gyro_500dps
#define ACCEL_RANGE_VALUE       _accel_4g
extern SPI_HandleTypeDef   hspi2;
#define IMU_SPI hspi2


#define MPU9250_REG_SELF_TEST_X_GYRO    0x00
#define MPU9250_REG_SELF_TEST_Y_GYRO    0x01
#define MPU9250_REG_SELF_TEST_Z_GYRO    0x02
#define MPU9250_REG_SELF_TEST_X_ACCEL   0x0D
#define MPU9250_REG_SELF_TEST_Y_ACCEL   0x0E
#define MPU9250_REG_SELF_TEST_Z_ACCEL   0x0F

#define MPU9250_REG_XG_OFFSET_H         0x13
#define MPU9250_REG_XG_OFFSET_L         0x14
#define MPU9250_REG_YG_OFFSET_H         0x15
#define MPU9250_REG_YG_OFFSET_L         0x16
#define MPU9250_REG_ZG_OFFSET_H         0x17
#define MPU9250_REG_ZG_OFFSET_L         0x18

#define MPU9250_REG_SMPLRT_DIV          0x19
#define MPU9250_REG_CONFIG              0x1A
#define MPU9250_REG_GYRO_CONFIG         0x1B
#define MPU9250_REG_ACCEL_CONFIG_1      0x1C
#define MPU9250_REG_ACCEL_CONFIG_2      0x1D
#define MPU9250_REG_LP_ACCEL_ODR        0x1E
#define MPU9250_REG_WOM_THR             0x1F
#define MPU9250_REG_FIFO_EN             0x23

#define MPU9250_REG_I2C_MST_CTRL        0x24

#define MPU9250_REG_I2C_SLV0_ADDR       0x25
#define MPU9250_REG_I2C_SLV0_REG        0x26
#define MPU9250_REG_I2C_SLV0_CTRL       0x27

#define MPU9250_REG_I2C_SLV1_ADDR       0x28
#define MPU9250_REG_I2C_SLV1_REG        0x29
#define MPU9250_REG_I2C_SLV1_CTRL       0x2A

#define MPU9250_REG_I2C_SLV2_ADDR       0x2B
#define MPU9250_REG_I2C_SLV2_REG        0x2C
#define MPU9250_REG_I2C_SLV2_CTRL       0x2D

#define MPU9250_REG_I2C_SLV3_ADDR       0x2E
#define MPU9250_REG_I2C_SLV3_REG        0x2F
#define MPU9250_REG_I2C_SLV3_CTRL       0x30

#define MPU9250_REG_I2C_SLV4_ADDR       0x31
#define MPU9250_REG_I2C_SLV4_REG        0x32
#define MPU9250_REG_I2C_SLV4_DO         0x33
#define MPU9250_REG_I2C_SLV4_CTRL       0x34
#define MPU9250_REG_I2C_SLV4_DI         0x35

#define MPU9250_REG_I2C_MST_STATUS      0x36

#define MPU9250_REG_INT_PIN_CFG         0x37
#define MPU9250_REG_INT_ENABLE          0x38
#define MPU9250_REG_INT_STATUS          0x3A

#define MPU9250_REG_ACCEL_XOUT_H        0x3B
#define MPU9250_REG_ACCEL_XOUT_L        0x3C
#define MPU9250_REG_ACCEL_YOUT_H        0x3D
#define MPU9250_REG_ACCEL_YOUT_L        0x3E
#define MPU9250_REG_ACCEL_ZOUT_H        0x3F
#define MPU9250_REG_ACCEL_ZOUT_L        0x40

#define MPU9250_REG_TEMP_OUT_H          0x41
#define MPU9250_REG_TEMP_OUT_L          0x42

#define MPU9250_REG_GYRO_XOUT_H         0x43
#define MPU9250_REG_GYRO_XOUT_L         0x44
#define MPU9250_REG_GYRO_YOUT_H         0x45
#define MPU9250_REG_GYRO_YOUT_L         0x46
#define MPU9250_REG_GYRO_ZOUT_H         0x47
#define MPU9250_REG_GYRO_ZOUT_L         0x48

#define MPU9250_REG_EXT_SENS_DATA_00    0x49
#define MPU9250_REG_EXT_SENS_DATA_01    0x4A
#define MPU9250_REG_EXT_SENS_DATA_02    0x4B
#define MPU9250_REG_EXT_SENS_DATA_03    0x4C
#define MPU9250_REG_EXT_SENS_DATA_04    0x4D
#define MPU9250_REG_EXT_SENS_DATA_05    0x4E
#define MPU9250_REG_EXT_SENS_DATA_06    0x4F
#define MPU9250_REG_EXT_SENS_DATA_07    0x50
#define MPU9250_REG_EXT_SENS_DATA_08    0x51
#define MPU9250_REG_EXT_SENS_DATA_09    0x52
#define MPU9250_REG_EXT_SENS_DATA_10    0x53
#define MPU9250_REG_EXT_SENS_DATA_11    0x54
#define MPU9250_REG_EXT_SENS_DATA_12    0x55
#define MPU9250_REG_EXT_SENS_DATA_13    0x56
#define MPU9250_REG_EXT_SENS_DATA_14    0x57
#define MPU9250_REG_EXT_SENS_DATA_15    0x58
#define MPU9250_REG_EXT_SENS_DATA_16    0x59
#define MPU9250_REG_EXT_SENS_DATA_17    0x5A
#define MPU9250_REG_EXT_SENS_DATA_18    0x5B
#define MPU9250_REG_EXT_SENS_DATA_19    0x5C
#define MPU9250_REG_EXT_SENS_DATA_20    0x5D
#define MPU9250_REG_EXT_SENS_DATA_21    0x5E
#define MPU9250_REG_EXT_SENS_DATA_22    0x5F
#define MPU9250_REG_EXT_SENS_DATA_23    0x60

#define MPU9250_REG_I2C_SLV0_DO         0x63
#define MPU9250_REG_I2C_SLV1_DO         0x64
#define MPU9250_REG_I2C_SLV2_DO         0x65
#define MPU9250_REG_I2C_SLV3_DO         0x66

#define MPU9250_REG_I2C_MST_DELAY_CTRL  0x67
#define MPU9250_REG_SIGNAL_PATH_RESET   0x68
#define MPU9250_REG_MOT_DETECT_CTRL     0x69
#define MPU9250_REG_USER_CTRL           0x6A

#define MPU9250_REG_PWR_MGMT_1          0x6B
#define MPU9250_REG_PWR_MGMT_2          0x6C

#define MPU9250_REG_FIFO_COUNTH         0x72
#define MPU9250_REG_FIFO_COUNTL         0x73
#define MPU9250_REG_FIFO_R_W            0x74

#define MPU9250_REG_WHO_AM_I            0x75

#define MPU9250_REG_XA_OFFSET_H         0x77
#define MPU9250_REG_XA_OFFSET_L         0x78
#define MPU9250_REG_YA_OFFSET_H         0x7A
#define MPU9250_REG_YA_OFFSET_L         0x7B
#define MPU9250_REG_ZA_OFFSET_H         0x7D
#define MPU9250_REG_ZA_OFFSET_L         0x7E

#define MPU9250_COMPASS_REG_WIA         0x00
#define MPU9250_COMPASS_REG_INFO        0x01
#define MPU9250_COMPASS_REG_ST1         0x02
#define MPU9250_COMPASS_REG_HXL         0x03
#define MPU9250_COMPASS_REG_HXH         0x04
#define MPU9250_COMPASS_REG_HYL         0x05
#define MPU9250_COMPASS_REG_HYH         0x06
#define MPU9250_COMPASS_REG_HZL         0x07
#define MPU9250_COMPASS_REG_HZH         0x08
#define MPU9250_COMPASS_REG_ST2         0x09
#define MPU9250_COMPASS_REG_CNTL1       0x0A
#define MPU9250_COMPASS_REG_CNTL2       0x0B
#define MPU9250_COMPASS_REG_ASTC        0x0C
#define MPU9250_COMPASS_REG_TS1         0x0D
#define MPU9250_COMPASS_REG_TS2         0x0E
#define MPU9250_COMPASS_REG_I2CDIS      0x0F
#define MPU9250_COMPASS_REG_ASAX        0x10
#define MPU9250_COMPASS_REG_ASAy        0x11
#define MPU9250_COMPASS_REG_ASAZ        0x12


	 typedef struct
	 {
	 int16_t x_accel;
	 int16_t y_accel;
	 int16_t z_accel;
	 int16_t x_gyro;
	 int16_t y_gyro;
	 int16_t z_gyro;
	 int16_t x_magnet;
	 int16_t y_magnet;
	 int16_t z_magnet;
	 int16_t temp;
	 }mpu9250;

	 typedef enum
	 {
	 _gyro_250dps =  0x00,
	 _gyro_500dps =  0x01,
	 _gyro_1000dps = 0x02,
	 _gyro_2000dps = 0x03
	 } gyro_range;
	 typedef enum
	 {
	 _accel_2g =  0x00,
	 _accel_4g =  0x01,
	 _accel_8g =  0x02,
	 _accel_16g = 0x03
	 } accel_range;

void mpu9250_init();
void mpu9250_write(uint8_t reg, uint8_t data);
void mpu9250_read(uint8_t address, uint8_t *data);
uint8_t mpu9250_read_data(mpu9250* data);

#endif /* INC_IMU_H_ */
