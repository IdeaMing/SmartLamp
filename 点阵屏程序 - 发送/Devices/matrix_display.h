/**
  ******************************************************************************
  * @file    matrix_display.h
  * @author  Liming
  * @version V1.0.0
  * @date    11-November-2017
  * @brief   This file contains all the functions prototypes for the
  *          matrix_display.c driver.
  ******************************************************************************
  * @attention
  *
  * <h2><center>&copy; COPYRIGHT 2017 IdeaMing</center></h2>
  *
  * @Licensed under 
  * @Revision 
  ******************************************************************************
  */

/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef __MATRIX_DISPLAY_H
#define __MATRIX_DISPLAY_H

#ifdef __cplusplus
 extern "C" {
#endif

/* Module---------------------------------------------------------------------*/	 
#ifdef	_MATRIX_MODULE_
#define	MATRIX_EXT
#else
#define	MATRIX_EXT	extern
#endif
	 
/* Includes ------------------------------------------------------------------*/
#include "stm32f0xx.h"
#include "font.h"   
/** @addtogroup Utilities
  * @{
  */ 

/** @addtogroup MATRIX_DISPLAY
  * @{
  */
      
/** @defgroup MATRIX_DISPLAY_LOW_LEVEL 
  * @brief This file provides set of firmware functions to manage Leds and push-button
  *        available on matrix_display from IdeaMing.
  * @{
  */

/** @defgroup MATRIX_DISPLAY_LOW_LEVEL_Exported_Types
  * @{
  */
typedef enum 
{
  LED2 = 0,
  
} Led_TypeDef;

typedef enum 
{  
  Color_Red   = 0,
  Color_Green = 1,
  Color_Yellow= 2
} Color_TypeDef; 

typedef enum 
{  
  BUTTON_USER = 0,
} Button_TypeDef;

typedef enum 
{  
  BUTTON_MODE_GPIO = 0,
  BUTTON_MODE_EXTI = 1
} ButtonMode_TypeDef; 

/* The Joystick is available on adafruit 1.8" TFT shield */     
typedef enum 
{ 
  JOY_NONE = 0,
  JOY_SEL = 1,
  JOY_DOWN = 2,
  JOY_LEFT = 3,
  JOY_RIGHT = 4,
  JOY_UP = 5
} JOYState_TypeDef;


/**
  * @}
  */  

/** @defgroup MATRIX_DISPLAY_LOW_LEVEL_Exported_Constants
  * @{
  */ 



/** @addtogroup MATRIX_DISPLAY_LOW_LEVEL_LED
  * @{
  */
#define MATRIX_COL_PIN            GPIO_Pin_0|GPIO_Pin_1|GPIO_Pin_2|GPIO_Pin_3|GPIO_Pin_4
#define MATRIX_COL_GPIO_PORT      GPIOB
#define MATRIX_COL_CLK            RCC_AHBPeriph_GPIOB 

#define	MATRIX_RowG_CLK					  RCC_AHBPeriph_GPIOB
#define	MATRIX_RowG_PORT				  GPIOB
#define	MATRIX_CCLKG_PIN				  GPIO_Pin_5
#define	MATRIX_DCLKG_PIN				  GPIO_Pin_6
#define	MATRIX_SDING_PIN				  GPIO_Pin_7

#define	MATRIX_RowR_CLK					  RCC_AHBPeriph_GPIOA
#define	MATRIX_RowR_PORT				  GPIOA
#define	MATRIX_CCLKR_PIN				  GPIO_Pin_1
#define	MATRIX_DCLKR_PIN				  GPIO_Pin_2
#define	MATRIX_SDINR_PIN				  GPIO_Pin_3

/**
  * @}
  */ 
/** @addtogroup MATRIX_DISPLAY_LOW_LEVEL_BUTTON
  * @{
  */  
#define BUTTONn                          1  

/**
 * @brief Wakeup push-button
 */
#define BUTTON_PIN                GPIO_Pin_15
#define BUTTON_GPIO_PORT          GPIOA
#define BUTTON_GPIO_CLK           RCC_AHBPeriph_GPIOA
#define BUTTON_EXTI_LINE          EXTI_Line15
#define BUTTON_EXTI_PORT_SOURCE   EXTI_PortSourceGPIOA
#define BUTTON_EXTI_PIN_SOURCE    EXTI_PinSource15
#define BUTTON_EXTI_IRQn          EXTI4_15_IRQn 


/**
  * @}
  */ 
/** @addtogroup MATRIX_DISPLAY_LOW_LEVEL_INTRRUPT
  * @{
  */  

/**
 * @brief 无线模块中断引脚
 */
#define GDO0_PIN                  GPIO_Pin_0      // PA0 GDO0
#define GDO0_GPIO_PORT            GPIOA
#define GDO0_GPIO_CLK             RCC_AHBPeriph_GPIOA
#define GDO0_EXTI_LINE            EXTI_Line0
#define GDO0_EXTI_PORT_SOURCE     EXTI_PortSourceGPIOA
#define GDO0_EXTI_PIN_SOURCE      EXTI_PinSource0
#define GDO0_EXTI_IRQn            EXTI0_1_IRQn 

#define GDO2_PIN                  GPIO_Pin_8     // PA 8 GDO2
#define GDO2_GPIO_PORT            GPIOA
#define GDO2_GPIO_CLK             RCC_AHBPeriph_GPIOA
#define GDO2_EXTI_LINE            EXTI_Line8
#define GDO2_EXTI_PORT_SOURCE     EXTI_PortSourceGPIOA
#define GDO2_EXTI_PIN_SOURCE      EXTI_PinSource8
#define GDO2_EXTI_IRQn            EXTI4_15_IRQn 


/**
  * @}
  */
    
/** @addtogroup MATRIX_DISPLAY_LOW_LEVEL_SPI
  * @{
  */
/**
  * @brief  SPI2 Interface pins
  */
/* Wireless module CC2500 */

#define CC2500_SPI                     SPI2
#define CC2500_SPI_CLK                 RCC_APB1Periph_SPI2

#define CC2500_POWER_PIN               GPIO_Pin_8
#define CC2500_POWER_PIN_PORT          GPIOA
#define CC2500_POWER_PIN_CLK           RCC_AHBPeriph_GPIOA


#define SPI2_NSS_PIN                   GPIO_Pin_12                 /* PB.12 */
#define SPI2_NSS_GPIO_PORT             GPIOB                       /* GPIOB */
#define SPI2_NSS_GPIO_CLK              RCC_AHBPeriph_GPIOB
#define SPI2_NSS_SOURCE                GPIO_PinSource12
#define SPI2_NSS_AF                    GPIO_AF_0

#define SPI2_SCK_PIN                   GPIO_Pin_13                 /* PB.13 */
#define SPI2_SCK_GPIO_PORT             GPIOB                       /* GPIOB */
#define SPI2_SCK_GPIO_CLK              RCC_AHBPeriph_GPIOB
#define SPI2_SCK_SOURCE                GPIO_PinSource13
#define SPI2_SCK_AF                    GPIO_AF_0

#define SPI2_MISO_PIN                  GPIO_Pin_14                 /* PB.14 */
#define SPI2_MISO_GPIO_PORT            GPIOB                       /* GPIOB */
#define SPI2_MISO_GPIO_CLK             RCC_AHBPeriph_GPIOB
#define SPI2_MISO_SOURCE               GPIO_PinSource14
#define SPI2_MISO_AF                   GPIO_AF_0

#define SPI2_MOSI_PIN                  GPIO_Pin_15                 /* PB.15 */
#define SPI2_MOSI_GPIO_PORT            GPIOB                       /* GPIOB */
#define SPI2_MOSI_GPIO_CLK             RCC_AHBPeriph_GPIOB
#define SPI2_MOSI_SOURCE               GPIO_PinSource15
#define SPI2_MOSI_AF                   GPIO_AF_0

/**
  * @brief  SPI1 Interface pins
  */
/* Flash module W25Q64 */

#define W25Q16_SPI                     SPI1
#define W25Q16_SPI_CLK                 RCC_APB2Periph_SPI1

#define SPI1_NSS_PIN                   GPIO_Pin_4                  /* PA.04 */
#define SPI1_NSS_GPIO_PORT             GPIOA                       /* GPIOA */
#define SPI1_NSS_GPIO_CLK              RCC_AHBPeriph_GPIOA
#define SPI1_NSS_SOURCE                GPIO_PinSource4
#define SPI1_NSS_AF                    GPIO_AF_0

#define SPI1_SCK_PIN                   GPIO_Pin_5                  /* PA.05 */
#define SPI1_SCK_GPIO_PORT             GPIOA                       /* GPIOA */
#define SPI1_SCK_GPIO_CLK              RCC_AHBPeriph_GPIOA
#define SPI1_SCK_SOURCE                GPIO_PinSource5
#define SPI1_SCK_AF                    GPIO_AF_0

#define SPI1_MISO_PIN                  GPIO_Pin_6                 /* PB.06 */
#define SPI1_MISO_GPIO_PORT            GPIOA                      /* GPIOA */
#define SPI1_MISO_GPIO_CLK             RCC_AHBPeriph_GPIOA
#define SPI1_MISO_SOURCE               GPIO_PinSource6
#define SPI1_MISO_AF                   GPIO_AF_0

#define SPI1_MOSI_PIN                  GPIO_Pin_7                  /* PA.07 */
#define SPI1_MOSI_GPIO_PORT            GPIOA                       /* GPIOA */
#define SPI1_MOSI_GPIO_CLK             RCC_AHBPeriph_GPIOA
#define SPI1_MOSI_SOURCE               GPIO_PinSource7
#define SPI1_MOSI_AF                   GPIO_AF_0
/**
  * @}
  */  
    

/**
  * @brief  LCD Data/Command pin
  */
#define LCD_DC_PIN                    GPIO_Pin_9                  /* PA.9 */
#define LCD_DC_GPIO_PORT              GPIOA                       /* GPIOA */
#define LCD_DC_GPIO_CLK               RCC_AHBPeriph_GPIOA


/**
  * @brief  ADC Interface pins
  */
/* The ADC is used to detect motion of Joystick available on adafruit 1.8" TFT shield */   
#define ADCx                            ADC1
#define ADC_CLK                         RCC_APB2Periph_ADC1
    
#define ADC_GPIO_PIN                    GPIO_Pin_0                  /* PB.0  */
#define ADC_GPIO_PORT                   GPIOB                       /* GPIOB */
#define ADC_GPIO_CLK                    RCC_AHBPeriph_GPIOB


/**
  * @}
  */

/** @defgroup MATRIX_DISPLAY_LOW_LEVEL_Exported_Macros
  * @{
  */

/**
  * @brief CC2500 W25Q64 line management 
  */
#define W25Q16_NSS_HIGH()   SPI1_NSS_GPIO_PORT->BSRR = SPI1_NSS_PIN
#define W25Q16_NSS_LOW()    SPI1_NSS_GPIO_PORT->BRR  = SPI1_NSS_PIN

#define CC2500_NSS_HIGH()   SPI2_NSS_GPIO_PORT->BSRR = SPI2_NSS_PIN
#define CC2500_NSS_LOW()    SPI2_NSS_GPIO_PORT->BRR  = SPI2_NSS_PIN

#define DCLKG_HIGH()        MATRIX_RowG_PORT->BSRR = MATRIX_DCLKG_PIN   
#define DCLKG_LOW()         MATRIX_RowG_PORT->BRR  = MATRIX_DCLKG_PIN   

#define CCLKG_HIGH()        MATRIX_RowG_PORT->BSRR = MATRIX_CCLKG_PIN
#define CCLKG_LOW()         MATRIX_RowG_PORT->BRR  = MATRIX_CCLKG_PIN

#define DCLKR_HIGH()        MATRIX_RowR_PORT->BSRR = MATRIX_DCLKR_PIN   
#define DCLKR_LOW()         MATRIX_RowR_PORT->BRR  = MATRIX_DCLKR_PIN

#define CCLKR_HIGH()        MATRIX_RowR_PORT->BSRR = MATRIX_CCLKR_PIN
#define CCLKR_LOW()         MATRIX_RowR_PORT->BRR  = MATRIX_CCLKR_PIN 

#define CC2500_ON()         CC2500_POWER_PIN_PORT->BSRR = CC2500_POWER_PIN
#define CC2500_OFF()        CC2500_POWER_PIN_PORT->BRR  = CC2500_POWER_PIN

#define INTERNAL_FLASH_PAGE0        0x08007000
#define INTERNAL_FLASH_PAGE1        0x08007400
#define INTERNAL_FLASH_PAGE2        0x08007800
#define INTERNAL_FLASH_PAGE3        0x08007C00


MATRIX_EXT	uint8_t	Green_Color[32][8];
MATRIX_EXT	uint8_t	Red_Color[32][8];
MATRIX_EXT	uint8_t	CC2500_GETPACKT;
/**
  * @}
  */

/** @defgroup MATRIX_DISPLAY_LOW_LEVEL_Exported_Functions
  * @{
  */
void LED_Init(void);
void Matrix_ColumnInit(void);
void Matrix_RowInit(void);
uint8_t Screen_fill(Color_TypeDef Color, unsigned char Byte);
void Button_Init(ButtonMode_TypeDef Button_Mode);
uint32_t Button_GetState(void);
void TIM_Config(uint32_t Frequence , uint32_t Period);
void MATRIX_Display(void);
void Dis_Demo(void);
void Matrix_FillAscii(Color_TypeDef Color, Pos_TypeDef Pos,uint8_t Ascii);
void Matrix_FillChinese(Color_TypeDef Color,Pos_TypeDef Pos,uint8_t *Chinese);

void CC2500_SPI_Init(void);
void CC2500_INT_Config(void);
void CC2500_InitPowerPin(void);
uint32_t GDO0_GetState(void);
uint32_t GDO2_GetState(void);
uint8_t CC2500_SPI_WriteRead(uint8_t Data);

void W25Q16_Init(void);
uint8_t W25Q16_SPI_WriteRead(uint8_t Data);

void LCD_CtrlLines_Config(void);
void SD_CtrlLines_Config(void);
void STM_ADC_Config(void);
JOYState_TypeDef STM_Get_JOYState(void);

void	Flash_WriteRead(uint8_t	WR_flag,uint32_t FLASH_START_ADDR, uint8_t *Buff,uint16_t Length);

void Flash_WritePage(uint32_t PageNumber,uint8_t *Buff);
void Flash_ReadPage(uint32_t PageNumber,uint8_t *Buff);


#undef	_MATRIX_MODULE_
/**
  * @}
  */

#ifdef __cplusplus
}
#endif

#endif /* __MATRIX_DISPLAY_H */
/**
  * @}
  */ 

/*********************** (C) COPYRIGHT 2017 IdeaMing ******END OF FILE*********/
