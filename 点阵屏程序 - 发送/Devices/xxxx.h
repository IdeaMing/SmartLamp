/**
  ******************************************************************************
  * @file    xxxx.h
  * @author  Liming
  * @version V1.0.0
  * @date    11-November-2017
  * @brief   This file contains all the functions prototypes for the
  *          xxxx.c driver.
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
#ifndef __xxxx_H
#define __xxxx_H

#ifdef __cplusplus
 extern "C" {
#endif

/* Module---------------------------------------------------------------------*/	 
#ifdef	_xxxx_MODULE_
#define	xxxx_EXT
#else
#define	xxxx_EXT	extern
#endif
	 
/* Includes ------------------------------------------------------------------*/
#include "stm32f0xx.h"
/* Exported types ------------------------------------------------------------*/
typedef enum 
{  
  BUTTON_MODE_GPIO = 0,
  BUTTON_MODE_EXTI = 1
} ButtonMode_TypeDef; 

/* Exported constants --------------------------------------------------------*/   
#define BUTTON_PIN                GPIO_Pin_15
/* Exported macro ------------------------------------------------------------*/

/* Exported functions ------------------------------------------------------- */
void Xxxx_Init();











#undef	_xxxx_MODULE_


#ifdef __cplusplus
}
#endif

#endif /* __xxxx_H */


/*********************** (C) COPYRIGHT 2017 IdeaMing ******END OF FILE*********/
