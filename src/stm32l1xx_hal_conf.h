/*
 * stm32l1xx_hal_conf.h
 *
 *  Created on: 10 Nov. 2019 ã.
 *      Author: Dmitry
 */

#ifndef STM32L1XX_HAL_CONF_H_
#define STM32L1XX_HAL_CONF_H_

#ifdef __cplusplus
	extern "C" {
#endif

#define HAL_MODULE_ENABLED
/* #define HAL_ADC_MODULE_ENABLED */
/* #define HAL_COMP_MODULE_ENABLED */
#define HAL_CORTEX_MODULE_ENABLED
/* #define HAL_CRC_MODULE_ENABLED */
/* #define HAL_CRYP_MODULE_ENABLED */
/* #define HAL_DAC_MODULE_ENABLED */
#define HAL_DMA_MODULE_ENABLED
#define HAL_FLASH_MODULE_ENABLED
#define HAL_GPIO_MODULE_ENABLED
/* #define HAL_I2C_MODULE_ENABLED */
/* #define HAL_I2S_MODULE_ENABLED */
/* #define HAL_IRDA_MODULE_ENABLED */
/* #define HAL_IWDG_MODULE_ENABLED */
/* #define HAL_LCD_MODULE_ENABLED */
/* #define HAL_NOR_MODULE_ENABLED */
/* #define HAL_OPAMP_MODULE_ENABLED */
/* #define HAL_PCD_MODULE_ENABLED */
#define HAL_PWR_MODULE_ENABLED
#define HAL_RCC_MODULE_ENABLED
/* #define HAL_RTC_MODULE_ENABLED */
/* #define HAL_SD_MODULE_ENABLED */
/* #define HAL_SMARTCARD_MODULE_ENABLED */
/* #define HAL_SPI_MODULE_ENABLED */
/* #define HAL_SRAM_MODULE_ENABLED */
#define HAL_TIM_MODULE_ENABLED
/* #define HAL_UART_MODULE_ENABLED */
/* #define HAL_USART_MODULE_ENABLED */
/* #define HAL_WWDG_MODULE_ENABLED */

/* Oscillator configuration here */

#if !defined  (HSE_VALUE)
  #define HSE_VALUE    (8000000U) /*!< Value of the External oscillator in Hz */
#endif /* HSE_VALUE */

#if !defined  (HSE_STARTUP_TIMEOUT)
  #define HSE_STARTUP_TIMEOUT    (100U)   /*!< Time out for HSE start up, in ms */
#endif /* HSE_STARTUP_TIMEOUT */

/**
  * @brief Internal Multiple Speed oscillator (MSI) default value.
  *        This value is the default MSI range value after Reset.
  */
#if !defined  (MSI_VALUE)
  #define MSI_VALUE    (2097000U) /*!< Value of the Internal oscillator in Hz*/
#endif /* MSI_VALUE */

/**
  * @brief Internal High Speed oscillator (HSI) value.
  *        This value is used by the RCC HAL module to compute the system frequency
  *        (when HSI is used as system clock source, directly or through the PLL).
  */
#if !defined  (HSI_VALUE)
  #define HSI_VALUE    (16000000U) /*!< Value of the Internal oscillator in Hz*/
#endif /* HSI_VALUE */

/**
  * @brief Internal Low Speed oscillator (LSI) value.
  */
#if !defined  (LSI_VALUE)
  #define LSI_VALUE    (37000U) /*!< LSI Typical Value in Hz*/
#endif /* LSI_VALUE */          /*!< Value of the Internal Low Speed oscillator in Hz
                                     The real value may vary depending on the variations
                                     in voltage and temperature.*/

/**
  * @brief External Low Speed oscillator (LSE) value.
  */
#if !defined  (LSE_VALUE)
  #define LSE_VALUE    (32768U) /*!< Value of the External Low Speed oscillator in Hz*/
#endif /* LSE_VALUE */

/**
  * @brief Time out for LSE start up value in ms.
  */
#if !defined  (LSE_STARTUP_TIMEOUT)
  #define LSE_STARTUP_TIMEOUT    (5000U)   /*!< Time out for LSE start up, in ms */
#endif /* LSE_STARTUP_TIMEOUT */

/* System configuration here */
#define  VDD_VALUE                    (3300U) /*!< Value of VDD in mv */
#define  TICK_INT_PRIORITY            (0x000FU)    /*!< tick interrupt priority */
#define  USE_RTOS                     0U
#define  PREFETCH_ENABLE              1U
#define  INSTRUCTION_CACHE_ENABLE     0U
#define  DATA_CACHE_ENABLE            0U

/* Register callback feature configuration here */
#define USE_HAL_ADC_REGISTER_CALLBACKS        0U
#define USE_HAL_COMP_REGISTER_CALLBACKS       0U
#define USE_HAL_DAC_REGISTER_CALLBACKS        0U
#define USE_HAL_I2C_REGISTER_CALLBACKS        0U
#define USE_HAL_I2S_REGISTER_CALLBACKS        0U
#define USE_HAL_IRDA_REGISTER_CALLBACKS       0U
#define USE_HAL_OPAMP_REGISTER_CALLBACKS      0U
#define USE_HAL_PCD_REGISTER_CALLBACKS        0U
#define USE_HAL_RTC_REGISTER_CALLBACKS        0U
#define USE_HAL_SDMMC_REGISTER_CALLBACKS      0U
#define USE_HAL_SMARTCARD_REGISTER_CALLBACKS  0U
#define USE_HAL_SPI_REGISTER_CALLBACKS        0U
#define USE_HAL_TIM_REGISTER_CALLBACKS        0U
#define USE_HAL_UART_REGISTER_CALLBACKS       0U
#define USE_HAL_USART_REGISTER_CALLBACKS      0U
#define USE_HAL_WWDG_REGISTER_CALLBACKS       0U

/* SPI periph configuration here */
#define USE_SPI_CRC                   1U

/* Includes here */
#ifdef HAL_RCC_MODULE_ENABLED
 #include "stm32l1xx_hal_rcc.h"
#endif /* HAL_RCC_MODULE_ENABLED */

#ifdef HAL_GPIO_MODULE_ENABLED
 #include "stm32l1xx_hal_gpio.h"
#endif /* HAL_GPIO_MODULE_ENABLED */

#ifdef HAL_DMA_MODULE_ENABLED
  #include "stm32l1xx_hal_dma.h"
#endif /* HAL_DMA_MODULE_ENABLED */

#ifdef HAL_CORTEX_MODULE_ENABLED
 #include "stm32l1xx_hal_cortex.h"
#endif /* HAL_CORTEX_MODULE_ENABLED */

#ifdef HAL_ADC_MODULE_ENABLED
 #include "stm32l1xx_hal_adc.h"
#endif /* HAL_ADC_MODULE_ENABLED */

#ifdef HAL_COMP_MODULE_ENABLED
  #include "stm32l1xx_hal_comp.h"
#endif /* HAL_COMP_MODULE_ENABLED */

#ifdef HAL_CRC_MODULE_ENABLED
 #include "stm32l1xx_hal_crc.h"
#endif /* HAL_CRC_MODULE_ENABLED */

#ifdef HAL_CRYP_MODULE_ENABLED
 #include "stm32l1xx_hal_cryp.h"
#endif /* HAL_CRYP_MODULE_ENABLED */

#ifdef HAL_DAC_MODULE_ENABLED
 #include "stm32l1xx_hal_dac.h"
#endif /* HAL_DAC_MODULE_ENABLED */

#ifdef HAL_FLASH_MODULE_ENABLED
 #include "stm32l1xx_hal_flash.h"
#endif /* HAL_FLASH_MODULE_ENABLED */

#ifdef HAL_SRAM_MODULE_ENABLED
 #include "stm32l1xx_hal_sram.h"
#endif /* HAL_SRAM_MODULE_ENABLED */

#ifdef HAL_NOR_MODULE_ENABLED
 #include "stm32l1xx_hal_nor.h"
#endif /* HAL_NOR_MODULE_ENABLED */

#ifdef HAL_I2C_MODULE_ENABLED
 #include "stm32l1xx_hal_i2c.h"
#endif /* HAL_I2C_MODULE_ENABLED */

#ifdef HAL_I2S_MODULE_ENABLED
 #include "stm32l1xx_hal_i2s.h"
#endif /* HAL_I2S_MODULE_ENABLED */

#ifdef HAL_IWDG_MODULE_ENABLED
 #include "stm32l1xx_hal_iwdg.h"
#endif /* HAL_IWDG_MODULE_ENABLED */

#ifdef HAL_LCD_MODULE_ENABLED
 #include "stm32l1xx_hal_lcd.h"
#endif /* HAL_LCD_MODULE_ENABLED */

#ifdef HAL_OPAMP_MODULE_ENABLED
 #include "stm32l1xx_hal_opamp.h"
#endif /* HAL_OPAMP_MODULE_ENABLED */

#ifdef HAL_PWR_MODULE_ENABLED
 #include "stm32l1xx_hal_pwr.h"
#endif /* HAL_PWR_MODULE_ENABLED */

#ifdef HAL_RTC_MODULE_ENABLED
 #include "stm32l1xx_hal_rtc.h"
#endif /* HAL_RTC_MODULE_ENABLED */

#ifdef HAL_SD_MODULE_ENABLED
 #include "stm32l1xx_hal_sd.h"
#endif /* HAL_SD_MODULE_ENABLED */

#ifdef HAL_SPI_MODULE_ENABLED
 #include "stm32l1xx_hal_spi.h"
#endif /* HAL_SPI_MODULE_ENABLED */

#ifdef HAL_TIM_MODULE_ENABLED
 #include "stm32l1xx_hal_tim.h"
#endif /* HAL_TIM_MODULE_ENABLED */

#ifdef HAL_UART_MODULE_ENABLED
 #include "stm32l1xx_hal_uart.h"
#endif /* HAL_UART_MODULE_ENABLED */

#ifdef HAL_USART_MODULE_ENABLED
 #include "stm32l1xx_hal_usart.h"
#endif /* HAL_USART_MODULE_ENABLED */

#ifdef HAL_IRDA_MODULE_ENABLED
 #include "stm32l1xx_hal_irda.h"
#endif /* HAL_IRDA_MODULE_ENABLED */

#ifdef HAL_SMARTCARD_MODULE_ENABLED
 #include "stm32l1xx_hal_smartcard.h"
#endif /* HAL_SMARTCARD_MODULE_ENABLED */

#ifdef HAL_WWDG_MODULE_ENABLED
 #include "stm32l1xx_hal_wwdg.h"
#endif /* HAL_WWDG_MODULE_ENABLED */

#ifdef HAL_PCD_MODULE_ENABLED
 #include "stm32l1xx_hal_pcd.h"
#endif /* HAL_PCD_MODULE_ENABLED */

#ifdef USE_FULL_ASSERT
#define assert_param(expr) ((expr) ? (void)0U : assert_failed((uint8_t *)__FILE__, __LINE__))
	void assert_failed(uint8_t* file, uint32_t line);
#else
#define assert_param(expr) ((void)0U)
#endif

#ifdef __cplusplus
}
#endif

#endif /* STM32L1XX_HAL_CONF_H_ */
