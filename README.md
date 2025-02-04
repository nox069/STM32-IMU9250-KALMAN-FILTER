# STM32-IMU9250-KALMAN-FILTER

0. Add "imu.h" file to your Inc folder, "imu.c" to your Src folder -> then refresh 

1. include library:  #include "imu.h"
	
2. Define:  mpu9250 imu_data;

3. In main loop initiate this function:   mpu9250_init(&imu_data);

4. In the loop function include this:  process_imu_data(&imu_data); 


After that call variables tike this: imu_data.angle_pitch
