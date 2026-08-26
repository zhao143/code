#include "robot_encoder.h"
#include "robot_config.h"
#include "tim.h"

static RobotEncoderData_t s_data;
static int16_t s_last_a;
static int16_t s_last_b;

static int16_t apply_dir_a(int16_t value)
{
#if ROBOT_ENCODER_A_INVERT
  return (int16_t)-value;
#else
  return value;
#endif
}

static int16_t apply_dir_b(int16_t value)
{
#if ROBOT_ENCODER_B_INVERT
  return (int16_t)-value;
#else
  return value;
#endif
}

static int16_t counts_to_mm_s(int16_t delta, uint32_t period_ms)
{
  int32_t counts_per_second;
  int32_t circumference_mm;
  int32_t speed;

  if (period_ms == 0U || ROBOT_ENCODER_COUNTS_PER_REV == 0)
  {
    return 0;
  }

  counts_per_second = ((int32_t)delta * 1000) / (int32_t)period_ms;
  circumference_mm = ((int32_t)ROBOT_WHEEL_DIAMETER_MM * 31416) / 10000;
  speed = (counts_per_second * circumference_mm) / ROBOT_ENCODER_COUNTS_PER_REV;

  if (speed > 32767)
  {
    speed = 32767;
  }
  else if (speed < -32768)
  {
    speed = -32768;
  }

  return (int16_t)speed;
}

void RobotEncoder_Init(void)
{
  HAL_TIM_Encoder_Start(&htim2, TIM_CHANNEL_ALL);
  HAL_TIM_Encoder_Start(&htim4, TIM_CHANNEL_ALL);

  __HAL_TIM_SET_COUNTER(&htim2, 0U);
  __HAL_TIM_SET_COUNTER(&htim4, 0U);

  s_last_a = 0;
  s_last_b = 0;
  s_data.total_a = 0;
  s_data.total_b = 0;
}

void RobotEncoder_Sample(uint32_t period_ms, RobotEncoderData_t *out)
{
  int16_t now_a = (int16_t)__HAL_TIM_GET_COUNTER(&htim2);
  int16_t now_b = (int16_t)__HAL_TIM_GET_COUNTER(&htim4);
  int16_t delta_a = apply_dir_a((int16_t)(now_a - s_last_a));
  int16_t delta_b = apply_dir_b((int16_t)(now_b - s_last_b));

  s_last_a = now_a;
  s_last_b = now_b;

  s_data.delta_a = delta_a;
  s_data.delta_b = delta_b;
  s_data.total_a += delta_a;
  s_data.total_b += delta_b;
  s_data.speed_a_mm_s = counts_to_mm_s(delta_a, period_ms);
  s_data.speed_b_mm_s = counts_to_mm_s(delta_b, period_ms);

  if (out != 0)
  {
    *out = s_data;
  }
}

void RobotEncoder_Get(RobotEncoderData_t *out)
{
  if (out != 0)
  {
    *out = s_data;
  }
}
