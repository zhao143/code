#ifndef ROBOT_CONFIG_H
#define ROBOT_CONFIG_H

#include <stdint.h>

/* Board mapping:
 * Motor A = left wheel, Motor B = right wheel.
 * Change the invert macros after first bench test if a wheel runs backward.
 */
#define ROBOT_PWM_MAX                         1000
#define ROBOT_CONTROL_PERIOD_MS               10U
#define ROBOT_SENSOR_FAST_PERIOD_MS           100U
#define ROBOT_SENSOR_SLOW_PERIOD_MS           1000U
#define ROBOT_TELEMETRY_PERIOD_MS             500U
#define ROBOT_CMD_TIMEOUT_MS                  700U

#define ROBOT_WHEEL_DIAMETER_MM               65
#define ROBOT_ENCODER_COUNTS_PER_REV          1320

#define ROBOT_MOTOR_A_INVERT                  0
#define ROBOT_MOTOR_B_INVERT                  0
#define ROBOT_ENCODER_A_INVERT                0
#define ROBOT_ENCODER_B_INVERT                0

/* Battery is measured by INA219 bus-voltage register only. */
#define ROBOT_BATTERY_LOW_MV                  10000U
#define ROBOT_BATTERY_RECOVER_MV              10800U

#define ROBOT_FAN_ON_C_X100                   4500
#define ROBOT_FAN_OFF_C_X100                  4000
#define ROBOT_TEMP_FAULT_C_X100               7000

/* Speed-mode parameters are intentionally conservative and must be tuned. */
#define ROBOT_SPEED_FF_PWM_PER_MM_S_X100      150
#define ROBOT_SPEED_KP_X100                   80
#define ROBOT_SPEED_KI_X100                   3
#define ROBOT_SPEED_INTEGRAL_LIMIT            30000

/* Keep stall detection off until motor direction and encoder direction are verified. */
#define ROBOT_STALL_DETECT_ENABLE             0
#define ROBOT_STALL_PWM_THRESHOLD             800
#define ROBOT_STALL_DELTA_THRESHOLD           2
#define ROBOT_STALL_TIME_MS                   600U

#define INA219_I2C_ADDR                       0x40U
#define DHT30_I2C_ADDR                        0x44U
#define MPU6050_I2C_ADDR                      0x68U

#endif
