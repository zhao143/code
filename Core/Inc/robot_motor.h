#ifndef ROBOT_MOTOR_H
#define ROBOT_MOTOR_H

#include <stdint.h>

/*
 * 初始化 TB6612 电机输出。
 *
 * 会启动 TIM1 的两个 PWM 通道，并让电机保持停止、STBY 关闭。上电默认不允许
 * 电机乱动，后续只有控制任务收到命令后才会打开 STBY。
 */
void RobotMotor_Init(void);

/*
 * 控制 TB6612 的 STBY 引脚。
 *
 * enable=0：TB6612 待机，电机驱动输出关闭。
 * enable=1：TB6612 使能，方向脚和 PWM 才会真正驱动电机。
 */
void RobotMotor_Enable(uint8_t enable);

/*
 * 设置两个电机的开环 PWM 百分比。
 *
 * 参数：
 *   motor_a：电机 A，范围 -1000 到 1000。正负号表示方向，绝对值表示占空比。
 *   motor_b：电机 B，范围 -1000 到 1000。
 *
 * 注意：
 *   函数内部会限幅，所以串口命令给超过范围的值也不会让 PWM 超出安全范围。
 */
void RobotMotor_SetPercent(int16_t motor_a, int16_t motor_b);

/*
 * 立即停止两个电机。
 *
 * 会把方向脚清零、PWM 比较值清零，但不会单独改变故障状态。故障状态由上层
 * robot_app.c 管理。
 */
void RobotMotor_Stop(void);

/*
 * 读取当前保存的 PWM 输出值。
 *
 * 参数允许传 NULL，只读取自己关心的那一路。这个函数主要用于调试输出或状态
 * 回传，实际电机控制仍以 TIM1 寄存器输出为准。
 */
void RobotMotor_GetPercent(int16_t *motor_a, int16_t *motor_b);

#endif
