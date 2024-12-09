/*******************************************************************************
 *
 * TITLE:	Utility Functions
 * AUTHOR:	Greg Berkeley
 * RELEASE:	05/07/2024
 *
 * NOTES
 * 1. None.
 *
 ******************************************************************************/

#include <stdint.h>
#include "stm32l476xx.h"


/**
 * Due to overhead faster delays will be less accurate. From observations on scope with TIM6 = 10 MHz:
 *
 * useconds		us delay
 * --------		--------
 * 	1			~2
 *  2			~2.9
 *  5			~5.9
 *  10			~10.9
 *  50			~50.8
 *  100			~100.6
 *  500			~500.0
 *  1000		~998.0
 */
static inline void DelayUs(TIM_TypeDef *TIMx, uint8_t timerRateMHz, uint16_t useconds)
{
	TIMx->CNT = 0;
	uint16_t duration = useconds * timerRateMHz;
	while(TIMx->CNT < duration);
}


/*******************************************************************************
* END
*******************************************************************************/
