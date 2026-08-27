#ifndef ROBOT_APP_H
#define ROBOT_APP_H

#include <stdint.h>
#include "robot_encoder.h"
#include "robot_sensors.h"

typedef enum
{
  ROBOT_APP_STATE_SAFE_IDLE = 0,
  ROBOT_APP_STATE_READY = 1,
  ROBOT_APP_STATE_RUN = 2,
  ROBOT_APP_STATE_FAULT = 3
} RobotAppState_t;

#define ROBOT_FAULT_COMM_TIMEOUT              0x0001U
#define ROBOT_FAULT_BATTERY_LOW               0x0002U
#define ROBOT_FAULT_OVER_TEMP                 0x0004U
#define ROBOT_FAULT_ESTOP                     0x0008U
#define ROBOT_FAULT_STALL_A                   0x0010U
#define ROBOT_FAULT_STALL_B                   0x0020U

typedef struct
{
  RobotAppState_t state;
  uint16_t faults;
  uint8_t motion_mode;
  int16_t target_a;
  int16_t target_b;
  int16_t pwm_a;
  int16_t pwm_b;
  uint8_t fan_on;
  uint8_t buzzer_on;
  RobotEncoderData_t encoder;
  RobotSensorsData_t sensors;
} RobotAppStatus_t;

/*
 * 板级初始化入口。
 *
 * main.c 在 MX_GPIO_Init、MX_TIMx_Init、MX_USARTx_Init 等 CubeMX 外设初始化
 * 完成后调用它。这个函数会让电机保持关闭、初始化传感器和串口，并准备好
 * 后续 FreeRTOS 任务需要的底层模块。
 */
void RobotApp_BoardInit(void);

/*
 * 创建机器人应用层 FreeRTOS 任务。
 *
 * freertos.c 在 osKernelInitialize 之后、osKernelStart 之前调用。这里创建
 * 控制任务、传感器任务和遥测任务。
 */
void RobotApp_CreateTasks(void);

/*
 * CubeMX 默认任务入口。
 *
 * 当前只保留一个低频空闲循环，方便以后放一些不紧急的后台检查。
 */
void RobotApp_DefaultTask(void *argument);

/*
 * LED 状态任务入口。
 *
 * 根据状态用不同频率闪烁 PC13 LED：故障快闪、运行中等速度闪、空闲慢闪。
 */
void RobotApp_StatusLedTask(void *argument);

/*
 * 获取机器人当前状态快照。
 *
 * 内部会加互斥量，避免读到一半时控制任务正在更新同一份数据。
 */
void RobotApp_GetStatus(RobotAppStatus_t *out);

/*
 * 请求尽快发送一次状态。
 *
 * 正式 KICKPI 模式下用于响应状态请求；调试模式下状态本来会周期打印。
 */
void RobotApp_RequestTelemetry(void);

#endif
