#ifndef ROBOT_MOTOR_H
#define ROBOT_MOTOR_H

#include <stdint.h>

void RobotMotor_Init(void);
void RobotMotor_Enable(uint8_t enable);
void RobotMotor_SetPercent(int16_t motor_a, int16_t motor_b);
void RobotMotor_Stop(void);
void RobotMotor_GetPercent(int16_t *motor_a, int16_t *motor_b);

#endif
