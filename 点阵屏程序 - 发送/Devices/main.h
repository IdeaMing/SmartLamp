/**
  ******************************************************************************
  * @file    main.h 
  * @author  Liming
  * @version V1.1.0
  * @date    11-November-2017
  * @brief   Header for main.c module
  ******************************************************************************
  * @attention
  *
  * <h2><center>&copy; COPYRIGHT 2017 IdeaMing</center></h2>
  *
  * Licensed under 
  *
  ******************************************************************************
  */
  
/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef __MAIN_H
#define __MAIN_H

/* Includes ------------------------------------------------------------------*/
#include "matrix_display.h"
#include "bsp_flash.h"
#include "bsp_wireless.h"
#include "bsp_usart.h"
#include "string.h"

/* Exported types ------------------------------------------------------------*/
/* Exported constants --------------------------------------------------------*/
/* Exported macro ------------------------------------------------------------*/
/* Exported functions ------------------------------------------------------- */
void TimingDelay_Decrement(void);
void Delay(__IO uint32_t nTime);






#endif /* __MAIN_H */

/************************ (C) COPYRIGHT IdeaMing *****END OF FILE****/
