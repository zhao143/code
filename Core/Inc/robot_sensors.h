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

/*
 * 初始化传感器模块。
 *
 * 会尝试配置 INA219、唤醒 MPU6050，并把 DS18B20 引脚释放为上拉输入状态。
 * 某个传感器初始化失败不会卡死系统，后续状态里的 valid_flags 会告诉你
 * 哪些传感器实际读到了。
 */
void RobotSensors_Init(void);

/*
 * 周期性更新所有传感器。
 *
 * 快速传感器 INA219/MPU6050 每 100ms 更新一次，DHT30/DS18B20 约 1s 更新一次。
 * 这个函数由 sensorTask 循环调用，不要放在中断里。
 */
void RobotSensors_Update(void);

/*
 * 读取最近一次传感器缓存。
 *
 * 参数：
 *   out：输出结构体指针。传 NULL 时函数直接返回。
 */
void RobotSensors_Get(RobotSensorsData_t *out);

/*
 * 扫描 I2C2 总线上的设备地址。
 *
 * 参数：
 *   addresses：用于保存发现地址的数组，可以传 NULL。
 *   max_count：addresses 数组最多能保存几个地址。
 *
 * 返回值：
 *   总线上应答的设备数量。调试 I2C 模块接线时可以临时调用这个函数。
 */
uint8_t RobotSensors_ScanI2C(uint8_t *addresses, uint8_t max_count);

#endif
