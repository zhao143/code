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

/*
 * 初始化两个硬件编码器定时器。
 *
 * TIM2 对应电机 A 编码器，TIM4 对应电机 B 编码器。CubeMX 中必须选择
 * Encoder Mode = TI1 and TI2，否则只能用单通道计数，方向和分辨率都会不对。
 */
void RobotEncoder_Init(void);

/*
 * 采样编码器并计算速度。
 *
 * 参数：
 *   period_ms：两次采样之间的周期，当前控制任务固定为 10ms。
 *   out：可选输出结构体，传 NULL 时只更新内部缓存。
 *
 * 说明：
 *   total 是累计计数，delta 是本周期增量，speed_x_mm_s 是根据轮径和每圈计数
 *   估算出的轮边线速度，单位 mm/s。
 */
void RobotEncoder_Sample(uint32_t period_ms, RobotEncoderData_t *out);

/*
 * 获取最近一次编码器采样结果。
 *
 * 这个函数不会主动读取定时器，只返回 RobotEncoder_Sample 已经计算好的缓存。
 */
void RobotEncoder_Get(RobotEncoderData_t *out);

#endif
