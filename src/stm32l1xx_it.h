/*
 * stm32l1xx_it.h
 *
 *  Created on: 10 Nov. 2019 ã.
 *      Author: Dmitry
 */

#ifndef STM32L1XX_IT_H_
#define STM32L1XX_IT_H_

#ifdef __cplusplus
extern "C" {
#endif

void NMI_Handler(void);
void HardFault_Handler(void);
void MemManage_Handler(void);
void BusFault_Handler(void);
void UsageFault_Handler(void);
void DebugMon_Handler(void);
void SysTick_Handler(void);

#ifdef __cplusplus
}
#endif

#endif /* STM32L1XX_IT_H_ */
