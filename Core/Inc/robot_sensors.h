#ifndef ROBOT_SENSORS_H
#define ROBOT_SENSORS_H

#include <stdint.h>

#define ROBOT_SENSOR_INA219_VALID             0x0001U
#define ROBOT_SENSOR_DHT30_VALID              0x0002U
#define ROBOT_SENSOR_MPU6050_VALID            0x0004U
#define ROBOT_SENSOR_DS18B20_VALID            0x0008U

typedef struct
{
  uint16_t valid_flags;
  uint16_t battery_mv;
  int16_t env_temp_c_x100;
  uint16_t env_humi_x100;
  int16_t battery_temp_c_x100;
  int16_t imu_accel_raw[3];
  int16_t imu_gyro_raw[3];
  int16_t imu_temp_c_x100;
} RobotSensorsData_t;

void RobotSensors_Init(void);
void RobotSensors_Update(void);
void RobotSensors_Get(RobotSensorsData_t *out);
uint8_t RobotSensors_ScanI2C(uint8_t *addresses, uint8_t max_count);

#endif
