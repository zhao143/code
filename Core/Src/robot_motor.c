#include "robot_motor.h"
#include "main.h"
#include "robot_config.h"
#include "tim.h"

static int16_t s_motor_a;
static int16_t s_motor_b;

/*
 * 把输入 PWM 限制到电机允许的范围内。
 *
 * 这里的范围是 -ROBOT_PWM_MAX 到 ROBOT_PWM_MAX。正负号表示方向，绝对值
 * 表示占空比大小。统一在这里做限幅，避免上层命令把 PWM 推得太大。
 */
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

/*
 * 根据方向和输入值，给 TB6612 的两根方向脚写电平。
 *
 * value > 0：正转
 * value < 0：反转
 * value = 0：双方向脚都拉低，进入停止状态
 *
 * 这个函数只负责方向，不负责 PWM 占空比。
 */
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

/*
 * 把 PWM 百分比换算成定时器比较值。
 *
 * TIM1 的自动重装值决定了 PWM 周期，这里按占空比比例计算比较值。这样上层
 * 只要传 -1000 到 1000 的数，不用关心定时器具体的 ARR 数值。
 */
static uint32_t duty_from_pwm(int16_t value)
{
  uint32_t period = __HAL_TIM_GET_AUTORELOAD(&htim1) + 1U;
  int16_t abs_value = (value < 0) ? (int16_t)-value : value;

  return ((uint32_t)abs_value * period) / ROBOT_PWM_MAX;
}

/*
 * 初始化电机输出。
 *
 * 先启动 TIM1 的两个 PWM 通道，再把 STBY 拉低，保证上电时电机不会乱转。
 * 之后上层需要调用 RobotMotor_Enable(1) 才会真正使能 TB6612。
 */
void RobotMotor_Init(void)
{
  HAL_TIM_PWM_Start(&htim1, TIM_CHANNEL_1);
  HAL_TIM_PWM_Start(&htim1, TIM_CHANNEL_4);
  __HAL_TIM_MOE_ENABLE(&htim1);

  RobotMotor_Stop();
  RobotMotor_Enable(0U);
}

/*
 * 控制 TB6612 的 STBY 引脚。
 *
 * 使能时把 STBY 拉高，驱动芯片工作；关闭时把 STBY 拉低，所有电机输出
 * 都停止。这个动作比只改 PWM 更安全，所以停机时会一起调用。
 */
void RobotMotor_Enable(uint8_t enable)
{
  HAL_GPIO_WritePin(TB6612_STBY_GPIO_Port, TB6612_STBY_Pin,
                    enable ? GPIO_PIN_SET : GPIO_PIN_RESET);
}

/*
 * 设置两个电机的开环 PWM。
 *
 * motor_a 对应左轮，motor_b 对应右轮。程序内部会自动做方向脚切换和 PWM
 * 比较值换算，因此上层只需要给一个带符号的目标值即可。
 */
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

/*
 * 立即停止两个电机。
 *
 * 这个函数会把方向脚、PWM 比较值都清零，确保驱动输出进入最安全的状态。
 */
void RobotMotor_Stop(void)
{
  s_motor_a = 0;
  s_motor_b = 0;

  set_dir(AIN1_GPIO_Port, AIN1_Pin, AIN2_GPIO_Port, AIN2_Pin, 0);
  set_dir(BIN1_GPIO_Port, BIN1_Pin, BIN2_GPIO_Port, BIN2_Pin, 0);

  __HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_4, 0U);
  __HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_1, 0U);
}

/*
 * 读取最近一次设置的电机 PWM 值。
 *
 * 这个函数不读取寄存器，只返回软件缓存。它主要给状态打印和调试界面用。
 */
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
