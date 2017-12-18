/**
  ******************************************************************************
  * @file    font.h
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
#ifndef __FONT_H
#define __FONT_H

#ifdef __cplusplus
 extern "C" {
#endif

/* Module---------------------------------------------------------------------*/	 
#ifdef	_FONT_MODULE_
#define	FONT_EXT
#else
#define	FONT_EXT	extern
#endif
	 
/* Includes ------------------------------------------------------------------*/
#include "stm32f0xx.h"

/* Exported types ------------------------------------------------------------*/

typedef enum 
{  
  Pos00   = 0,  // 第0行位置0
  Pos01   = 1,
  Pos02   = 2,
  Pos03   = 3,
  Pos04   = 4,
  Pos05   = 5,
  Pos06   = 6,
  Pos07   = 7, 
  
  Pos10   = 8,  // 第1行位置0
  Pos11   = 9,
  Pos12   = 10,
  Pos13   = 11,
  Pos14   = 12,
  Pos15   = 13,
  Pos16   = 14,
  Pos17   = 15  
} Pos_TypeDef;  // 显示位置


typedef struct 
{
    uint8_t Head[3];    // 索引
    uint8_t Infor[32];  // 形码
}GB2312Type16x16;       // GB2312 16x16



/* Exported constants --------------------------------------------------------*/   

/* Exported macro ------------------------------------------------------------*/
FONT_EXT  unsigned char const AsciiCode16x8[1520];
FONT_EXT  GB2312Type16x16 const GB2312Code16x16[7*35];
/* Exported functions ------------------------------------------------------- */






#undef	_FONT_MODULE_


#ifdef __cplusplus
}
#endif

#endif /* __FONT_H */


/*********************** (C) COPYRIGHT 2017 IdeaMing ******END OF FILE*********/
