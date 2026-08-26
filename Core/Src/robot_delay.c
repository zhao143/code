#include "robot_delay.h"
#include "stm32f1xx_hal.h"

void RobotDelay_Init(void)
{
  CoreDebug->DEMCR |= CoreDebug_DEMCR_TRCENA_Msk;
  DWT->CYCCNT = 0U;
  DWT->CTRL |= DWT_CTRL_CYCCNTENA_Msk;
}

void RobotDelay_Us(uint32_t us)
{
  uint32_t start = DWT->CYCCNT;
  uint32_t ticks = (HAL_RCC_GetHCLKFreq() / 1000000U) * us;

  while ((uint32_t)(DWT->CYCCNT - start) < ticks)
  {
  }
}
