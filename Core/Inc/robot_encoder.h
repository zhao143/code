#ifndef ROBOT_ENCODER_H
#define ROBOT_ENCODER_H

#include <stdint.h>

typedef struct
{
  int32_t total_a;
  int32_t total_b;
  int16_t delta_a;
  int16_t delta_b;
  int16_t speed_a_mm_s;
  int16_t speed_b_mm_s;
} RobotEncoderData_t;

void RobotEncoder_Init(void);
void RobotEncoder_Sample(uint32_t period_ms, RobotEncoderData_t *out);
void RobotEncoder_Get(RobotEncoderData_t *out);

#endif
