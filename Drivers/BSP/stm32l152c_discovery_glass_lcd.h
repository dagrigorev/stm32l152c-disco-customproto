 /**
  ******************************************************************************
  * @file    stm32l152c_discovery_glass_lcd.h
  * @author  MCD Application Team
  * @brief   Header file for stm32l152c_discovery_glass_lcd.c module.
  ******************************************************************************
  * @attention
  *
  * <h2><center>&copy; Copyright (c) 2017 STMicroelectronics.
  * All rights reserved.</center></h2>
  *
  * This software component is licensed by ST under BSD 3-Clause license,
  * the "License"; You may not use this file except in compliance with the
  * License. You may obtain a copy of the License at:
  *                        opensource.org/licenses/BSD-3-Clause
  *
  ******************************************************************************
  */ 

/** @addtogroup BSP
  * @{
  */
  
/** @addtogroup STM32L152C_DISCOVERY
  * @{
  */

/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef __stm32l152c_discovery_GLASS_LCD_H
#define __stm32l152c_discovery_GLASS_LCD_H

#ifdef __cplusplus
 extern "C" {
#endif

/* Includes ------------------------------------------------------------------*/
#include "stm32l152c_discovery.h"
#include "stm32l1xx_hal_lcd.h"

/** @addtogroup STM32L152C_Discovery_GLASS_LCD
  * @{
  */ 

/** @defgroup STM32L152C-Discovery_GLASS_LCD_Exported_Types Exported Types
  * @{
  */
/**
  * @brief LCD Glass digit position
  */
typedef enum
{
  LCD_DIGIT_POSITION_1 = 1,
  LCD_DIGIT_POSITION_2 = 2,
  LCD_DIGIT_POSITION_3 = 3,
  LCD_DIGIT_POSITION_4 = 4,
  LCD_DIGIT_POSITION_5 = 5,
  LCD_DIGIT_POSITION_6 = 6,
  LCD_DIGIT_MAX_NUMBER = 6,
}DigitPosition_Typedef;

/**
  * @brief LCD Glass point
  * Warning: element values correspond to LCD Glass point.
  */

typedef enum
{
  POINT_OFF = 0,
  POINT_ON = 1
}Point_Typedef;

/**
  * @brief LCD Glass Double point
  * Warning: element values correspond to LCD Glass Double point.
  */
typedef enum
{
  DOUBLEPOINT_OFF = 0,
  DOUBLEPOINT_ON = 1
}DoublePoint_Typedef;

/**
  * @brief LCD Glass Battery Level
  * element values correspond to different LCD Glass battery levels
  */
typedef enum
{
  BATTERYLEVEL_OFF = 0,
  BATTERYLEVEL_1_4 = 1,
  BATTERYLEVEL_1_2 = 2,
  BATTERYLEVEL_3_4 = 3,
  BATTERYLEVEL_FULL = 4
}BatteryLevel_Typedef;

/**
  * @brief LCD Glass Bar Id
  */
typedef enum
{
  LCD_BAR_NONE  = 0,
  LCD_BAR_0     = (1 << 0),
  LCD_BAR_1     = (1 << 1),
  LCD_BAR_2     = (1 << 2),
  LCD_BAR_3     = (1 << 3)
}BarId_Typedef;

/**
  * @}
  */
  
/** @defgroup STM32L152C-Discovery_GLASS_LCD_Exported_Defines Exported Defines
  * @{
  */

/**
  * @brief LCD digit defintion 
  */
#define COM_PER_DIGIT_NB          4/*!< Specifies number of COM to address a digit */
#define SEG_PER_DIGIT_NB          4/*!< Specifies number of SEG to address a digit */

#define LCD_MAP_CHAR_COM0_SEG_1ST_POS   (1 << LCD_MAP_CHAR_COM0_SEG_1ST_SHIFT)
#define LCD_MAP_CHAR_COM0_SEG_2ND_POS   (1 << LCD_MAP_CHAR_COM0_SEG_2ND_SHIFT)
#define LCD_MAP_CHAR_COM0_SEG_3RD_POS   (1 << LCD_MAP_CHAR_COM0_SEG_3RD_SHIFT)
#define LCD_MAP_CHAR_COM0_SEG_4TH_POS   (1 << LCD_MAP_CHAR_COM0_SEG_4TH_SHIFT)
#define LCD_MAP_CHAR_COM1_SEG_1ST_POS   (1 << LCD_MAP_CHAR_COM1_SEG_1ST_SHIFT)
#define LCD_MAP_CHAR_COM1_SEG_2ND_POS   (1 << LCD_MAP_CHAR_COM1_SEG_2ND_SHIFT)
#define LCD_MAP_CHAR_COM1_SEG_3RD_POS   (1 << LCD_MAP_CHAR_COM1_SEG_3RD_SHIFT)
#define LCD_MAP_CHAR_COM1_SEG_4TH_POS   (1 << LCD_MAP_CHAR_COM1_SEG_4TH_SHIFT)
#define LCD_MAP_CHAR_COM2_SEG_1ST_POS   (1 << LCD_MAP_CHAR_COM2_SEG_1ST_SHIFT)
#define LCD_MAP_CHAR_COM2_SEG_2ND_POS   (1 << LCD_MAP_CHAR_COM2_SEG_2ND_SHIFT)
#define LCD_MAP_CHAR_COM2_SEG_3RD_POS   (1 << LCD_MAP_CHAR_COM2_SEG_3RD_SHIFT)
#define LCD_MAP_CHAR_COM2_SEG_4TH_POS   (1 << LCD_MAP_CHAR_COM2_SEG_4TH_SHIFT)
#define LCD_MAP_CHAR_COM3_SEG_1ST_POS   (1 << LCD_MAP_CHAR_COM3_SEG_1ST_SHIFT)
#define LCD_MAP_CHAR_COM3_SEG_2ND_POS   (1 << LCD_MAP_CHAR_COM3_SEG_2ND_SHIFT)
#define LCD_MAP_CHAR_COM3_SEG_3RD_POS   (1 << LCD_MAP_CHAR_COM3_SEG_3RD_SHIFT)
#define LCD_MAP_CHAR_COM3_SEG_4TH_POS   (1 << LCD_MAP_CHAR_COM3_SEG_4TH_SHIFT)
#define LCD_MAP_CHAR_COM0_SEG_1ST_SHIFT 0x00000000
#define LCD_MAP_CHAR_COM0_SEG_2ND_SHIFT 0x00000001
#define LCD_MAP_CHAR_COM0_SEG_3RD_SHIFT 0x00000002
#define LCD_MAP_CHAR_COM0_SEG_4TH_SHIFT 0x00000003
#define LCD_MAP_CHAR_COM1_SEG_1ST_SHIFT 0x00000004
#define LCD_MAP_CHAR_COM1_SEG_2ND_SHIFT 0x00000005
#define LCD_MAP_CHAR_COM1_SEG_3RD_SHIFT 0x00000006
#define LCD_MAP_CHAR_COM1_SEG_4TH_SHIFT 0x00000007
#define LCD_MAP_CHAR_COM2_SEG_1ST_SHIFT 0x00000008
#define LCD_MAP_CHAR_COM2_SEG_2ND_SHIFT 0x00000009
#define LCD_MAP_CHAR_COM2_SEG_3RD_SHIFT 0x00000010
#define LCD_MAP_CHAR_COM2_SEG_4TH_SHIFT 0x00000011
#define LCD_MAP_CHAR_COM3_SEG_1ST_SHIFT 0x00000012
#define LCD_MAP_CHAR_COM3_SEG_2ND_SHIFT 0x00000013
#define LCD_MAP_CHAR_COM3_SEG_3RD_SHIFT 0x00000014
#define LCD_MAP_CHAR_COM3_SEG_4TH_SHIFT 0x00000015

/**
  * @brief LCD Digit defines
  */
#define LCD_DIGIT1_COM0               LCD_COM0
#define LCD_DIGIT1_COM0_SEG_MASK      ~(LCD_SEG0 | LCD_SEG1 | LCD_SEG22 | LCD_SEG23)
#define LCD_DIGIT1_COM1               LCD_COM1
#define LCD_DIGIT1_COM1_SEG_MASK      ~(LCD_SEG0 | LCD_SEG1 | LCD_SEG22 | LCD_SEG23)
#define LCD_DIGIT1_COM2               LCD_COM2
#define LCD_DIGIT1_COM2_SEG_MASK      ~(LCD_SEG0 | LCD_SEG1 | LCD_SEG22 | LCD_SEG23)
#define LCD_DIGIT1_COM3               LCD_COM3
#define LCD_DIGIT1_COM3_SEG_MASK      ~(LCD_SEG0 | LCD_SEG1 | LCD_SEG22 | LCD_SEG23)

#define LCD_DIGIT2_COM0               LCD_COM0
#define LCD_DIGIT2_COM0_SEG_MASK      ~(LCD_SEG2 | LCD_SEG3 | LCD_SEG20 | LCD_SEG21)
#define LCD_DIGIT2_COM1               LCD_COM1
#define LCD_DIGIT2_COM1_SEG_MASK      ~(LCD_SEG2 | LCD_SEG3 | LCD_SEG20 | LCD_SEG21)
#define LCD_DIGIT2_COM2               LCD_COM2
#define LCD_DIGIT2_COM2_SEG_MASK      ~(LCD_SEG2 | LCD_SEG3 | LCD_SEG20 | LCD_SEG21)
#define LCD_DIGIT2_COM3               LCD_COM3
#define LCD_DIGIT2_COM3_SEG_MASK      ~(LCD_SEG2 | LCD_SEG3 | LCD_SEG20 | LCD_SEG21)

#define LCD_DIGIT3_COM0               LCD_COM0
#define LCD_DIGIT3_COM0_SEG_MASK      ~(LCD_SEG4 | LCD_SEG5 | LCD_SEG18 | LCD_SEG19)
#define LCD_DIGIT3_COM1               LCD_COM1
#define LCD_DIGIT3_COM1_SEG_MASK      ~(LCD_SEG4 | LCD_SEG5 | LCD_SEG18 | LCD_SEG19)
#define LCD_DIGIT3_COM2               LCD_COM2
#define LCD_DIGIT3_COM2_SEG_MASK      ~(LCD_SEG4 | LCD_SEG5 | LCD_SEG18 | LCD_SEG19)
#define LCD_DIGIT3_COM3               LCD_COM3
#define LCD_DIGIT3_COM3_SEG_MASK      ~(LCD_SEG4 | LCD_SEG5 | LCD_SEG18 | LCD_SEG19)

#define LCD_DIGIT4_COM0               LCD_COM0
#define LCD_DIGIT4_COM0_SEG_MASK      ~(LCD_SEG6 | LCD_SEG17)
#define LCD_DIGIT4_COM0_1             LCD_COM0_1
#define LCD_DIGIT4_COM0_1_SEG_MASK    ~(LCD_SEG7 | LCD_SEG16)
#define LCD_DIGIT4_COM1               LCD_COM1
#define LCD_DIGIT4_COM1_SEG_MASK      ~(LCD_SEG6 |  LCD_SEG17)
#define LCD_DIGIT4_COM1_1             LCD_COM1_1
#define LCD_DIGIT4_COM1_1_SEG_MASK    ~(LCD_SEG7 | LCD_SEG16)
#define LCD_DIGIT4_COM2               LCD_COM2
#define LCD_DIGIT4_COM2_SEG_MASK      ~(LCD_SEG6 | LCD_SEG17)
#define LCD_DIGIT4_COM2_1             LCD_COM2_1
#define LCD_DIGIT4_COM2_1_SEG_MASK    ~(LCD_SEG7 | LCD_SEG16)
#define LCD_DIGIT4_COM3               LCD_COM3
#define LCD_DIGIT4_COM3_SEG_MASK      ~(LCD_SEG6 | LCD_SEG17)
#define LCD_DIGIT4_COM3_1             LCD_COM3_1
#define LCD_DIGIT4_COM3_1_SEG_MASK    ~(LCD_SEG7 | LCD_SEG16)

#define LCD_DIGIT5_COM0               LCD_COM0
#define LCD_DIGIT5_COM0_SEG_MASK      ~(LCD_SEG9 | LCD_SEG14)
#define LCD_DIGIT5_COM0_1             LCD_COM0_1
#define LCD_DIGIT5_COM0_1_SEG_MASK    ~(LCD_SEG8 | LCD_SEG15)
#define LCD_DIGIT5_COM1               LCD_COM1
#define LCD_DIGIT5_COM1_SEG_MASK      ~(LCD_SEG9 | LCD_SEG14)
#define LCD_DIGIT5_COM1_1             LCD_COM1_1
#define LCD_DIGIT5_COM1_1_SEG_MASK    ~(LCD_SEG8 | LCD_SEG15)
#define LCD_DIGIT5_COM2               LCD_COM2
#define LCD_DIGIT5_COM2_SEG_MASK      ~(LCD_SEG9 | LCD_SEG14)
#define LCD_DIGIT5_COM2_1             LCD_COM2_1
#define LCD_DIGIT5_COM2_1_SEG_MASK    ~(LCD_SEG8 | LCD_SEG15)
#define LCD_DIGIT5_COM3               LCD_COM3
#define LCD_DIGIT5_COM3_SEG_MASK      ~(LCD_SEG9 | LCD_SEG14)
#define LCD_DIGIT5_COM3_1             LCD_COM3_1
#define LCD_DIGIT5_COM3_1_SEG_MASK    ~(LCD_SEG8 | LCD_SEG15)

#define LCD_DIGIT6_COM0               LCD_COM0
#define LCD_DIGIT6_COM0_SEG_MASK      ~(LCD_SEG10 | LCD_SEG11 | LCD_SEG12 | LCD_SEG13)
#define LCD_DIGIT6_COM1               LCD_COM1
#define LCD_DIGIT6_COM1_SEG_MASK      ~(LCD_SEG10 | LCD_SEG11 | LCD_SEG12 | LCD_SEG13)
#define LCD_DIGIT6_COM2               LCD_COM2
#define LCD_DIGIT6_COM2_SEG_MASK      ~(LCD_SEG10 | LCD_SEG11 | LCD_SEG12 | LCD_SEG13)
#define LCD_DIGIT6_COM3               LCD_COM3
#define LCD_DIGIT6_COM3_SEG_MASK      ~(LCD_SEG10 | LCD_SEG11 | LCD_SEG12 | LCD_SEG13)

/**
  * @brief LCD Bar location
  */
#define LCD_BAR0_2_COM            LCD_COM3
#define LCD_BAR1_3_COM            LCD_COM2
#define LCD_BAR0_SEG              LCD_SEG11
#define LCD_BAR1_SEG              LCD_SEG11
#define LCD_BAR2_SEG              LCD_SEG9
#define LCD_BAR3_SEG              LCD_SEG9
#define LCD_BAR0_2_SEG_MASK       ~(LCD_BAR0_SEG | LCD_BAR2_SEG)
#define LCD_BAR1_3_SEG_MASK       ~(LCD_BAR1_SEG | LCD_BAR3_SEG)

/**
  * @brief LCD segments & coms redefinition.
  * LCD component segments & coms are not necessarily link to MCU segmnents & coms output.
  */
#define LCD_COM0          MCU_LCD_COM0
#define LCD_COM0_1        MCU_LCD_COM0
#define LCD_COM1          MCU_LCD_COM1
#define LCD_COM1_1        MCU_LCD_COM1
#define LCD_COM2          MCU_LCD_COM2
#define LCD_COM2_1        MCU_LCD_COM2
#define LCD_COM3          MCU_LCD_COM3
#define LCD_COM3_1        MCU_LCD_COM3
#define LCD_SEG0          MCU_LCD_SEG0
#define LCD_SEG1          MCU_LCD_SEG1
#define LCD_SEG2          MCU_LCD_SEG2
#define LCD_SEG3          MCU_LCD_SEG7
#define LCD_SEG4          MCU_LCD_SEG8
#define LCD_SEG5          MCU_LCD_SEG9
#define LCD_SEG6          MCU_LCD_SEG10
#define LCD_SEG7          MCU_LCD_SEG11
#define LCD_SEG8          MCU_LCD_SEG12
#define LCD_SEG9          MCU_LCD_SEG13
#define LCD_SEG10         MCU_LCD_SEG14
#define LCD_SEG11         MCU_LCD_SEG15
#define LCD_SEG12         MCU_LCD_SEG17
#define LCD_SEG13         MCU_LCD_SEG16
#define LCD_SEG14         MCU_LCD_SEG18
#define LCD_SEG15         MCU_LCD_SEG19
#define LCD_SEG16         MCU_LCD_SEG20
#define LCD_SEG17         MCU_LCD_SEG21
#define LCD_SEG18         MCU_LCD_SEG24
#define LCD_SEG19         MCU_LCD_SEG25
#define LCD_SEG20         MCU_LCD_SEG26
#define LCD_SEG21         MCU_LCD_SEG27
#define LCD_SEG22         MCU_LCD_SEG28
#define LCD_SEG23         MCU_LCD_SEG29
#define LCD_SEG0_SHIFT          MCU_LCD_SEG0_SHIFT
#define LCD_SEG1_SHIFT          MCU_LCD_SEG1_SHIFT
#define LCD_SEG2_SHIFT          MCU_LCD_SEG2_SHIFT
#define LCD_SEG3_SHIFT          MCU_LCD_SEG7_SHIFT
#define LCD_SEG4_SHIFT          MCU_LCD_SEG8_SHIFT
#define LCD_SEG5_SHIFT          MCU_LCD_SEG9_SHIFT
#define LCD_SEG6_SHIFT          MCU_LCD_SEG10_SHIFT
#define LCD_SEG7_SHIFT          MCU_LCD_SEG11_SHIFT
#define LCD_SEG8_SHIFT          MCU_LCD_SEG12_SHIFT
#define LCD_SEG9_SHIFT          MCU_LCD_SEG13_SHIFT
#define LCD_SEG10_SHIFT         MCU_LCD_SEG14_SHIFT
#define LCD_SEG11_SHIFT         MCU_LCD_SEG15_SHIFT
#define LCD_SEG12_SHIFT         MCU_LCD_SEG17_SHIFT
#define LCD_SEG13_SHIFT         MCU_LCD_SEG16_SHIFT
#define LCD_SEG14_SHIFT         MCU_LCD_SEG18_SHIFT
#define LCD_SEG15_SHIFT         MCU_LCD_SEG19_SHIFT
#define LCD_SEG16_SHIFT         MCU_LCD_SEG20_SHIFT
#define LCD_SEG17_SHIFT         MCU_LCD_SEG21_SHIFT
#define LCD_SEG18_SHIFT         MCU_LCD_SEG24_SHIFT
#define LCD_SEG19_SHIFT         MCU_LCD_SEG25_SHIFT
#define LCD_SEG20_SHIFT         MCU_LCD_SEG26_SHIFT
#define LCD_SEG21_SHIFT         MCU_LCD_SEG27_SHIFT
#define LCD_SEG22_SHIFT         MCU_LCD_SEG28_SHIFT
#define LCD_SEG23_SHIFT         MCU_LCD_SEG29_SHIFT

/**
  * @brief STM32 LCD segments & coms definitions.
  */
#define MCU_LCD_COM0          LCD_RAM_REGISTER0
#define MCU_LCD_COM0_1        LCD_RAM_REGISTER1
#define MCU_LCD_COM1          LCD_RAM_REGISTER2
#define MCU_LCD_COM1_1        LCD_RAM_REGISTER3
#define MCU_LCD_COM2          LCD_RAM_REGISTER4
#define MCU_LCD_COM2_1        LCD_RAM_REGISTER5
#define MCU_LCD_COM3          LCD_RAM_REGISTER6
#define MCU_LCD_COM3_1        LCD_RAM_REGISTER7
#define MCU_LCD_COM4          LCD_RAM_REGISTER8
#define MCU_LCD_COM4_1        LCD_RAM_REGISTER9
#define MCU_LCD_COM5          LCD_RAM_REGISTER10
#define MCU_LCD_COM5_1        LCD_RAM_REGISTER11
#define MCU_LCD_COM6          LCD_RAM_REGISTER12
#define MCU_LCD_COM6_1        LCD_RAM_REGISTER13
#define MCU_LCD_COM7          LCD_RAM_REGISTER14
#define MCU_LCD_COM7_1        LCD_RAM_REGISTER15
#define MCU_LCD_SEG0          (0x00000001 << MCU_LCD_SEG0_SHIFT)
#define MCU_LCD_SEG1          (0x00000001 << MCU_LCD_SEG1_SHIFT)
#define MCU_LCD_SEG2          (0x00000001 << MCU_LCD_SEG2_SHIFT)
#define MCU_LCD_SEG3          (0x00000001 << MCU_LCD_SEG3_SHIFT)
#define MCU_LCD_SEG4          (0x00000001 << MCU_LCD_SEG4_SHIFT)
#define MCU_LCD_SEG5          (0x00000001 << MCU_LCD_SEG5_SHIFT)
#define MCU_LCD_SEG6          (0x00000001 << MCU_LCD_SEG6_SHIFT)
#define MCU_LCD_SEG7          (0x00000001 << MCU_LCD_SEG7_SHIFT)
#define MCU_LCD_SEG8          (0x00000001 << MCU_LCD_SEG8_SHIFT)
#define MCU_LCD_SEG9          (0x00000001 << MCU_LCD_SEG9_SHIFT)
#define MCU_LCD_SEG10         (0x00000001 << MCU_LCD_SEG10_SHIFT)
#define MCU_LCD_SEG11         (0x00000001 << MCU_LCD_SEG11_SHIFT)
#define MCU_LCD_SEG12         (0x00000001 << MCU_LCD_SEG12_SHIFT)
#define MCU_LCD_SEG13         (0x00000001 << MCU_LCD_SEG13_SHIFT)
#define MCU_LCD_SEG14         (0x00000001 << MCU_LCD_SEG14_SHIFT)
#define MCU_LCD_SEG15         (0x00000001 << MCU_LCD_SEG15_SHIFT)
#define MCU_LCD_SEG16         (0x00000001 << MCU_LCD_SEG16_SHIFT)
#define MCU_LCD_SEG17         (0x00000001 << MCU_LCD_SEG17_SHIFT)
#define MCU_LCD_SEG18         (0x00000001 << MCU_LCD_SEG18_SHIFT)
#define MCU_LCD_SEG19         (0x00000001 << MCU_LCD_SEG19_SHIFT)
#define MCU_LCD_SEG20         (0x00000001 << MCU_LCD_SEG20_SHIFT)
#define MCU_LCD_SEG21         (0x00000001 << MCU_LCD_SEG21_SHIFT)
#define MCU_LCD_SEG22         (0x00000001 << MCU_LCD_SEG22_SHIFT)
#define MCU_LCD_SEG23         (0x00000001 << MCU_LCD_SEG23_SHIFT)
#define MCU_LCD_SEG24         (0x00000001 << MCU_LCD_SEG24_SHIFT)
#define MCU_LCD_SEG25         (0x00000001 << MCU_LCD_SEG25_SHIFT)
#define MCU_LCD_SEG26         (0x00000001 << MCU_LCD_SEG26_SHIFT)
#define MCU_LCD_SEG27         (0x00000001 << MCU_LCD_SEG27_SHIFT)
#define MCU_LCD_SEG28         (0x00000001 << MCU_LCD_SEG28_SHIFT)
#define MCU_LCD_SEG29         (0x00000001 << MCU_LCD_SEG29_SHIFT)
#define MCU_LCD_SEG30         (0x00000001 << MCU_LCD_SEG30_SHIFT)
#define MCU_LCD_SEG31         (0x00000001 << MCU_LCD_SEG31_SHIFT)
#define MCU_LCD_SEG32         (0x00000001 << MCU_LCD_SEG32_SHIFT)
#define MCU_LCD_SEG33         (0x00000001 << MCU_LCD_SEG33_SHIFT)
#define MCU_LCD_SEG34         (0x00000001 << MCU_LCD_SEG34_SHIFT)
#define MCU_LCD_SEG35         (0x00000001 << MCU_LCD_SEG35_SHIFT)
#define MCU_LCD_SEG36         (0x00000001 << MCU_LCD_SEG36_SHIFT)
#define MCU_LCD_SEG37         (0x00000001 << MCU_LCD_SEG37_SHIFT)
#define MCU_LCD_SEG38         (0x00000001 << MCU_LCD_SEG38_SHIFT)
#define MCU_LCD_SEG0_SHIFT    0
#define MCU_LCD_SEG1_SHIFT    1
#define MCU_LCD_SEG2_SHIFT    2
#define MCU_LCD_SEG3_SHIFT    3
#define MCU_LCD_SEG4_SHIFT    4
#define MCU_LCD_SEG5_SHIFT    5
#define MCU_LCD_SEG6_SHIFT    6
#define MCU_LCD_SEG7_SHIFT    7
#define MCU_LCD_SEG8_SHIFT    8
#define MCU_LCD_SEG9_SHIFT    9
#define MCU_LCD_SEG10_SHIFT   10
#define MCU_LCD_SEG11_SHIFT   11
#define MCU_LCD_SEG12_SHIFT   12
#define MCU_LCD_SEG13_SHIFT   13
#define MCU_LCD_SEG14_SHIFT   14
#define MCU_LCD_SEG15_SHIFT   15
#define MCU_LCD_SEG16_SHIFT   16
#define MCU_LCD_SEG17_SHIFT   17
#define MCU_LCD_SEG18_SHIFT   18
#define MCU_LCD_SEG19_SHIFT   19
#define MCU_LCD_SEG20_SHIFT   20
#define MCU_LCD_SEG21_SHIFT   21
#define MCU_LCD_SEG22_SHIFT   22
#define MCU_LCD_SEG23_SHIFT   23
#define MCU_LCD_SEG24_SHIFT   24
#define MCU_LCD_SEG25_SHIFT   25
#define MCU_LCD_SEG26_SHIFT   26
#define MCU_LCD_SEG27_SHIFT   27
#define MCU_LCD_SEG28_SHIFT   28
#define MCU_LCD_SEG29_SHIFT   29
#define MCU_LCD_SEG30_SHIFT   30
#define MCU_LCD_SEG31_SHIFT   31
#define MCU_LCD_SEG32_SHIFT   0
#define MCU_LCD_SEG33_SHIFT   1
#define MCU_LCD_SEG34_SHIFT   2
#define MCU_LCD_SEG35_SHIFT   3
#define MCU_LCD_SEG36_SHIFT   4
#define MCU_LCD_SEG37_SHIFT   5
#define MCU_LCD_SEG38_SHIFT   6
#define MCU_LCD_SEG39_SHIFT   7
#define MCU_LCD_SEG40_SHIFT   8
#define MCU_LCD_SEG41_SHIFT   9
#define MCU_LCD_SEG42_SHIFT   10
#define MCU_LCD_SEG43_SHIFT   11

/**
  * @brief LCD Pins definition.
  */
#define LCD_GPIO_BANKA_PINS  (GPIO_PIN_1 | GPIO_PIN_2 | GPIO_PIN_3 | GPIO_PIN_8 | GPIO_PIN_9 |\
                             GPIO_PIN_10 |GPIO_PIN_15)
#define LCD_GPIO_BANKB_PINS  (GPIO_PIN_3 | GPIO_PIN_4 | GPIO_PIN_5 | GPIO_PIN_8 | GPIO_PIN_9 | \
                             GPIO_PIN_10 | GPIO_PIN_11 | GPIO_PIN_12 | GPIO_PIN_13 | GPIO_PIN_14 |\
                             GPIO_PIN_15)
#define LCD_GPIO_BANKC_PINS  (GPIO_PIN_0 | GPIO_PIN_1 | GPIO_PIN_2 | GPIO_PIN_3 | GPIO_PIN_6 | \
                             GPIO_PIN_7 | GPIO_PIN_8 | GPIO_PIN_9 | GPIO_PIN_10 |GPIO_PIN_11)

/* Define for scrolling sentences*/
#define SCROLL_SPEED_HIGH     125
#define SCROLL_SPEED_MEDIUM   250
#define SCROLL_SPEED_LOW      500

#define DOT                   ((uint16_t) 0x8000 ) /* for add decimal point in string */
#define DOUBLE_DOT            ((uint16_t) 0x4000) /* for add decimal point in string */

/* code for '(' character */
#define C_OPENPARMAP          ((uint16_t) 0x0028)

/* code for ')' character */
#define C_CLOSEPARMAP         ((uint16_t) 0x0011)

/* code for 'm' character */
#define C_MMAP                ((uint16_t) 0xb210)

/* code for 'n' character */
#define C_NMAP                ((uint16_t) 0x2210)

/* code for '�' character */
#define C_UMAP                ((uint16_t) 0x6084)

/* constant code for '*' character */
#define C_STAR                ((uint16_t) 0xA0DD)

/* constant code for '-' character */
#define C_MINUS               ((uint16_t) 0xA000)

/* constant code for '/' */
#define C_SLATCH              ((uint16_t) 0x00c0)

/* constant code for � */
#define C_PERCENT_1           ((uint16_t) 0xec00)

/* constant code for small o */
#define C_PERCENT_2           ((uint16_t) 0xb300)

#define C_FULL                ((uint16_t) 0xffdd)

/**
  * @}
  */   

/** @addtogroup STM32L152C-Discovery_LCD_Exported_Functions
  * @{
  */
void BSP_LCD_GLASS_Init(void);
void BSP_LCD_GLASS_DeInit(void);
void BSP_LCD_GLASS_BlinkConfig(uint32_t BlinkMode, uint32_t BlinkFrequency);
void BSP_LCD_GLASS_Contrast(uint32_t Contrast);
void BSP_LCD_GLASS_DisplayChar(uint8_t* ch, Point_Typedef Point, DoublePoint_Typedef Column, DigitPosition_Typedef Position);
void BSP_LCD_GLASS_DisplayString(uint8_t* ptr);
void BSP_LCD_GLASS_WriteChar(uint8_t* ch, uint8_t Point, uint8_t Column, uint8_t Position);
void BSP_LCD_GLASS_DisplayStrDeci(uint16_t* ptr);
void BSP_LCD_GLASS_ScrollSentence(uint8_t* ptr, uint16_t nScroll, uint16_t ScrollSpeed);
void BSP_LCD_GLASS_DisplayBar(uint32_t BarId);
void BSP_LCD_GLASS_ClearBar(uint32_t BarId);
void BSP_LCD_GLASS_BarLevelConfig(uint8_t BarLevel);
void BSP_LCD_GLASS_Clear(void);
/**
  * @}
  */

/**
  * @}
  */   

#ifdef __cplusplus
}
#endif

#endif /* __STM32L152C-Discovery_GLASS_LCD_H */

/**
  * @}
  */   

/**
  * @}
  */   

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
