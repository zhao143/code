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

void RobotApp_BoardInit(void);
void RobotApp_CreateTasks(void);
void RobotApp_DefaultTask(void *argument);
void RobotApp_StatusLedTask(void *argument);
void RobotApp_GetStatus(RobotAppStatus_t *out);
void RobotApp_RequestTelemetry(void);

#endif
