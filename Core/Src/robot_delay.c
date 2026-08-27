#include "robot_delay.h"
#include "stm32f1xx_hal.h"

/*
 * 初始化 DWT 微秒计数器。
 *
 * 这个函数只需要在系统启动时调用一次。它打开 Cortex-M3 的 DWT 计数器，
 * 这样后面才能用 CPU 周期数实现非常精确的微秒延时。
 */
void RobotDelay_Init(void)
{
  CoreDebug->DEMCR |= CoreDebug_DEMCR_TRCENA_Msk;
  DWT->CYCCNT = 0U;
  DWT->CTRL |= DWT_CTRL_CYCCNTENA_Msk;
}

/*
 * 微秒级阻塞延时。
 *
 * 参数 us 表示要等待的微秒数。这个函数是忙等待，期间 CPU 会一直空转，
 * 所以只适合 DS18B20 这类必须卡住时序的短延时，不适合普通任务里的长等待。
 */
void RobotDelay_Us(uint32_t us)
{
  uint32_t start = DWT->CYCCNT;
  uint32_t ticks = (HAL_RCC_GetHCLKFreq() / 1000000U) * us;

  while ((uint32_t)(DWT->CYCCNT - start) < ticks)
  {
  }
}
