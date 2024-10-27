#ifndef __DWT_DELAY_H__
#define __DWT_DELAY_H__

#ifdef __cplusplus
extern "C" {
#endif

#include "main.h"


#define DWT_MS_CYCLE  1000
#define DWT_US_CYCLE  1000000

#define GET_SYSTEM_CLK_FREQ()  HAL_RCC_GetSysClockFreq()


/**
 * @brief  Initializes DWT_Cycle_Count for DWT_Delay_us and DWT_Delay_ms function
 * @return Error DWT counter
 *         1: DWT counter Error
 *         0: DWT counter works
 */
uint32_t DWT_Delay_Init(void);

/**
 * @brief  This function provides a delay (in microseconds)
 * @param  microseconds: delay in microseconds
 */
__STATIC_INLINE void DWT_Delay_us(uint32_t microseconds)
{
  uint32_t clk_cycle_start = DWT->CYCCNT;

  /* Go to number of cycles for system */
  microseconds *= (GET_SYSTEM_CLK_FREQ() / DWT_US_CYCLE);

  /* Delay till end */
  while ((DWT->CYCCNT - clk_cycle_start) < microseconds);
}

/**
 * @brief  This function provides a delay (in milliseconds)
 * @param  milliseconds: delay in milliseconds
 */
__STATIC_INLINE void DWT_Delay_ms(uint32_t milliseconds)
{
  uint32_t clk_cycle_start = DWT->CYCCNT;

  /* Go to number of cycles for system */
  milliseconds *= (GET_SYSTEM_CLK_FREQ() / DWT_MS_CYCLE);

  /* Delay till end */
  while ((DWT->CYCCNT - clk_cycle_start) < milliseconds);
}

#ifdef __cplusplus
}
#endif

#endif


