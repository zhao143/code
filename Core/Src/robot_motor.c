#include "robot_motor.h"
#include "main.h"
#include "robot_config.h"
#include "tim.h"

static int16_t s_motor_a;
static int16_t s_motor_b;

static int16_t clamp_pwm(int16_t value)
{
  if (value > ROBOT_PWM_MAX)
  {
    return ROBOT_PWM_MAX;
  }
  if (value < -ROBOT_PWM_MAX)
  {
    return -ROBOT_PWM_MAX;
  }
  return value;
}

static void set_dir(GPIO_TypeDef *in1_port, uint16_t in1_pin,
                    GPIO_TypeDef *in2_port, uint16_t in2_pin,
                    int16_t value)
{
  if (value > 0)
  {
    HAL_GPIO_WritePin(in1_port, in1_pin, GPIO_PIN_SET);
    HAL_GPIO_WritePin(in2_port, in2_pin, GPIO_PIN_RESET);
  }
  else if (value < 0)
  {
    HAL_GPIO_WritePin(in1_port, in1_pin, GPIO_PIN_RESET);
    HAL_GPIO_WritePin(in2_port, in2_pin, GPIO_PIN_SET);
  }
  else
  {
    HAL_GPIO_WritePin(in1_port, in1_pin, GPIO_PIN_RESET);
    HAL_GPIO_WritePin(in2_port, in2_pin, GPIO_PIN_RESET);
  }
}

static uint32_t duty_from_pwm(int16_t value)
{
  uint32_t period = __HAL_TIM_GET_AUTORELOAD(&htim1) + 1U;
  int16_t abs_value = (value < 0) ? (int16_t)-value : value;

  return ((uint32_t)abs_value * period) / ROBOT_PWM_MAX;
}

void RobotMotor_Init(void)
{
  HAL_TIM_PWM_Start(&htim1, TIM_CHANNEL_1);
  HAL_TIM_PWM_Start(&htim1, TIM_CHANNEL_4);
  __HAL_TIM_MOE_ENABLE(&htim1);

  RobotMotor_Stop();
  RobotMotor_Enable(0U);
}

void RobotMotor_Enable(uint8_t enable)
{
  HAL_GPIO_WritePin(TB6612_STBY_GPIO_Port, TB6612_STBY_Pin,
                    enable ? GPIO_PIN_SET : GPIO_PIN_RESET);
}

void RobotMotor_SetPercent(int16_t motor_a, int16_t motor_b)
{
  motor_a = clamp_pwm(motor_a);
  motor_b = clamp_pwm(motor_b);

#if ROBOT_MOTOR_A_INVERT
  motor_a = (int16_t)-motor_a;
#endif
#if ROBOT_MOTOR_B_INVERT
  motor_b = (int16_t)-motor_b;
#endif

  s_motor_a = motor_a;
  s_motor_b = motor_b;

  RobotMotor_Enable((motor_a != 0) || (motor_b != 0));

  set_dir(AIN1_GPIO_Port, AIN1_Pin, AIN2_GPIO_Port, AIN2_Pin, motor_a);
  set_dir(BIN1_GPIO_Port, BIN1_Pin, BIN2_GPIO_Port, BIN2_Pin, motor_b);

  __HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_4, duty_from_pwm(motor_a));
  __HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_1, duty_from_pwm(motor_b));
}

void RobotMotor_Stop(void)
{
  s_motor_a = 0;
  s_motor_b = 0;

  set_dir(AIN1_GPIO_Port, AIN1_Pin, AIN2_GPIO_Port, AIN2_Pin, 0);
  set_dir(BIN1_GPIO_Port, BIN1_Pin, BIN2_GPIO_Port, BIN2_Pin, 0);

  __HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_4, 0U);
  __HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_1, 0U);
}

void RobotMotor_GetPercent(int16_t *motor_a, int16_t *motor_b)
{
  if (motor_a != 0)
  {
    *motor_a = s_motor_a;
  }
  if (motor_b != 0)
  {
    *motor_b = s_motor_b;
  }
}
