#ifndef ROBOT_DELAY_H
#define ROBOT_DELAY_H

#include <stdint.h>

/*
 * 初始化 DWT 微秒计数器。
 *
 * DS18B20 单总线需要几十微秒级别的时序，普通 HAL_Delay 只能做到毫秒级，
 * 所以这里打开 Cortex-M3 的 DWT CYCCNT 计数器，用 CPU 周期数来做微秒延时。
 * 这个函数在 main.c 初始化外设后、启动 FreeRTOS 前调用一次即可。
 */
void RobotDelay_Init(void);

/*
 * 微秒级阻塞延时。
 *
 * 参数：
 *   us：需要等待的微秒数。
 *
 * 注意：
 *   这是忙等待，会占用 CPU；因此只用于 DS18B20 这类必须严格卡时序的短延时，
 *   不要拿它做普通任务里的长时间等待。
 */
void RobotDelay_Us(uint32_t us);

#endif
