#ifndef ROBOT_COMM_H
#define ROBOT_COMM_H

#include <stdint.h>
#include "stm32f1xx_hal.h"

#define ROBOT_COMM_FRAME_HEAD0                0xAAU
#define ROBOT_COMM_FRAME_HEAD1                0x55U
#define ROBOT_COMM_VERSION                    0x01U
#define ROBOT_COMM_MAX_PAYLOAD                64U

#define ROBOT_CMD_SET_MOTION                  0x01U
#define ROBOT_CMD_SET_STATE                   0x02U
#define ROBOT_CMD_GET_STATUS                  0x03U
#define ROBOT_CMD_SET_OUTPUT                  0x04U
#define ROBOT_CMD_ESTOP                       0x05U
#define ROBOT_CMD_STATUS                      0x81U

#define ROBOT_MOTION_MODE_PWM                 0U
#define ROBOT_MOTION_MODE_SPEED               1U

#define ROBOT_STATE_CMD_IDLE                  0U
#define ROBOT_STATE_CMD_ENABLE                1U
#define ROBOT_STATE_CMD_CLEAR_FAULT           2U

typedef struct
{
  uint8_t version;
  uint8_t command;
  uint8_t length;
  uint8_t payload[ROBOT_COMM_MAX_PAYLOAD];
} RobotCommFrame_t;

typedef void (*RobotCommFrameHandler_t)(const RobotCommFrame_t *frame);

void RobotComm_Init(void);
void RobotComm_SetFrameHandler(RobotCommFrameHandler_t handler);
void RobotComm_ProcessRx(void);
HAL_StatusTypeDef RobotComm_SendFrame(uint8_t command, const uint8_t *payload, uint8_t length);
uint32_t RobotComm_GetRxOverflowCount(void);
uint16_t RobotComm_Crc16(const uint8_t *data, uint16_t length, uint16_t seed);

#endif
