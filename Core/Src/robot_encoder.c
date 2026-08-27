#include "robot_encoder.h"
#include "robot_config.h"
#include "tim.h"

static RobotEncoderData_t s_data;
static int16_t s_last_a;
static int16_t s_last_b;

/*
 * 根据配置决定编码器 A 的方向。
 *
 * 如果机械安装方向和代码定义相反，就把 ROBOT_ENCODER_A_INVERT 置 1。
 * 这里集中处理，避免其他地方到处写负号。
 */
static int16_t apply_dir_a(int16_t value)
{
#if ROBOT_ENCODER_A_INVERT
  return (int16_t)-value;
#else
  return value;
#endif
}

/*
 * 根据配置决定编码器 B 的方向。
 *
 * 这个函数和 apply_dir_a 的作用一样，只是针对右轮编码器。
 */
static int16_t apply_dir_b(int16_t value)
{
#if ROBOT_ENCODER_B_INVERT
  return (int16_t)-value;
#else
  return value;
#endif
}

/*
 * 把编码器增量换算成轮边线速度。
 *
 * period_ms 是采样周期。函数先把本周期计数换算成每秒计数，再结合轮径和
 * 每圈计数，估算出 mm/s 速度。这个值是给调试和简单闭环用的。
 */
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

/*
 * 启动两个编码器定时器并清零计数器。
 *
 * TIM2 和 TIM4 都要工作在 Encoder Mode = TI1 and TI2。这里只做运行时启动
 * 和缓存清零，CubeMX 里的通道和模式要先配好。
 */
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

/*
 * 采样两个编码器并更新内部速度缓存。
 *
 * 控制任务固定每 10ms 调一次这个函数，所以 period_ms 传 10 即可。函数会
 * 记录本次增量、累计值和速度，供状态输出和闭环控制使用。
 */
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

/*
 * 读取最近一次编码器缓存。
 *
 * 这个函数不直接触碰定时器，只返回 RobotEncoder_Sample 已经更新好的数据。
 */
void RobotEncoder_Get(RobotEncoderData_t *out)
{
  if (out != 0)
  {
    *out = s_data;
  }
}
