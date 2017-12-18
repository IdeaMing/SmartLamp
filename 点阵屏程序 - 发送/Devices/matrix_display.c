/**
  ******************************************************************************
  * @file    matrix_display
  * @author  Liming
  * @version V1.0.0
  * @date    11-November-2017
  * @brief   This file provides set of firmware functions to manage Leds and
  *          push-button available on matrix_display from IdeaMing.
  *          thanks for http://bbs.21ic.com/icview-623956-1-1.html 
  *          landlord mmuuss586
  ******************************************************************************
  * @attention
  *
  * <h2><center>&copy; COPYRIGHT 2017 IdeaMing</center></h2>
  *
  * @Licensed under 
  * @Revision 
  ******************************************************************************
  */  

/* Module---------------------------------------------------------------------*/
#define	_MATRIX_MODULE_

/* Includes ------------------------------------------------------------------*/
#include "matrix_display.h"


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

/* Private typedef -----------------------------------------------------------*/
typedef enum {FAILED = 0, PASSED = !FAILED} TestStatus;
/* Private define ------------------------------------------------------------*/
#define FLASH_PAGE_SIZE         ((uint32_t)0x00000400)   /* FLASH Page Size */
#define FLASH_USER_START_ADDR   ((uint32_t)0x08006000)   /* Start @ of user Flash area */
#define FLASH_USER_END_ADDR     ((uint32_t)0x08007000)   /* End @ of user Flash area */



/* Private macro -------------------------------------------------------------*/
/* Private variables ---------------------------------------------------------*/
/** @defgroup MATRIX_DISPLAY_LOW_LEVEL_Private_Variables
  * @{
  */ 
uint16_t PrescalerValue = 0;
uint16_t    CCR3_Val = 1000;
uint8_t	Green_Color[32][8];
uint8_t	Red_Color[32][8];
uint8_t Data_busy;

uint32_t EraseCounter = 0x00;
uint32_t Data = 0x3210ABCD;
uint32_t NbrOfPage = 0x00;
__IO FLASH_Status FLASHStatus = FLASH_COMPLETE;
__IO TestStatus MemoryProgramStatus = PASSED;

/**
  * @}
  */ 

/* Private function prototypes -----------------------------------------------*/

/** @defgroup MATRIX_DISPLAY_LOW_LEVEL_Private_Functions
  * @{
  */ 


/*******************************************************************************
  * @brief  行控制端口初始化
  * @param  None
  * @retval None
****************************************************************Author:Liming**/
void Matrix_ColumnInit(void)
{
    GPIO_InitTypeDef  GPIO_InitStructure;
    
    /* Enable the GPIO Clock */
    RCC_AHBPeriphClockCmd(MATRIX_COL_CLK, ENABLE);

    /* Configure the GPIO pin */
    GPIO_InitStructure.GPIO_Pin = MATRIX_COL_PIN;
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_OUT;
    GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;
    GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_UP;
    GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
    GPIO_Init(MATRIX_COL_GPIO_PORT, &GPIO_InitStructure);
    GPIO_ResetBits(MATRIX_COL_GPIO_PORT,MATRIX_COL_PIN);
}
/*******************************************************************************
  * @brief  列控制端口初始化
  * @param  None
  * @retval None
****************************************************************Author:Liming**/
void Matrix_RowInit(void)
{
  GPIO_InitTypeDef  GPIO_InitStructure;

  /* Enable the GPIO Clock */
  RCC_AHBPeriphClockCmd(MATRIX_RowR_CLK, ENABLE);
  RCC_AHBPeriphClockCmd(MATRIX_RowG_CLK, ENABLE);

  /* Configure the GPIO pin */
  GPIO_InitStructure.GPIO_Pin	= MATRIX_CCLKR_PIN|MATRIX_DCLKR_PIN|MATRIX_SDINR_PIN;
  GPIO_InitStructure.GPIO_Mode = GPIO_Mode_OUT;
  GPIO_InitStructure.GPIO_OType= GPIO_OType_PP;
  GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_UP;
  GPIO_InitStructure.GPIO_Speed= GPIO_Speed_10MHz;
  GPIO_Init(MATRIX_RowR_PORT, &GPIO_InitStructure); 

  GPIO_InitStructure.GPIO_Pin = MATRIX_CCLKG_PIN|MATRIX_DCLKG_PIN|MATRIX_SDING_PIN;
  GPIO_Init(MATRIX_RowG_PORT, &GPIO_InitStructure);
  CCLKR_LOW();
  CCLKG_LOW();
}

/*******************************************************************************
  * @brief  缓存写入汉字
  * @param  None
  * @retval None
****************************************************************Author:Liming**/
void Matrix_FillChinese(Color_TypeDef Color,Pos_TypeDef Pos,uint8_t *Chinese)
{
  uint16_t Num;
  uint8_t i;

  for(Num = 0; Num < sizeof(GB2312Code16x16)/35; Num++) 
  {
    if((GB2312Code16x16[Num].Head[0] == *Chinese) && GB2312Code16x16[Num].Head[1] == *(Chinese+1))
    {
      if(Color==Color_Red)
      {
        if(Pos/8)
        {
          for(i=16;i<32;i++)
          {
            Red_Color[i][Pos-8]=GB2312Code16x16[Num].Infor[2*(i-16)];
            Red_Color[i][Pos-7]=GB2312Code16x16[Num].Infor[2*(i-16)+1];
            Green_Color[i][Pos-8]=0x00;
            Green_Color[i][Pos-7]=0x00;
          }
        }
        else
        {
          for(i=0;i<16;i++)
          {
            Red_Color[i][Pos]=GB2312Code16x16[Num].Infor[2*i];
            Red_Color[i][Pos+1]=GB2312Code16x16[Num].Infor[2*i+1];
            Green_Color[i][Pos]=0x00;
            Green_Color[i][Pos+1]=0x00;
          } 
        }
      }
      else if(Color==Color_Green)   
      {
        if(Pos/8)
        {
          for(i=16;i<32;i++)
          {
            Red_Color[i][Pos-8]=0x00;
            Red_Color[i][Pos-7]=0x00;
            Green_Color[i][Pos-8]=GB2312Code16x16[Num].Infor[2*(i-16)];
            Green_Color[i][Pos-7]=GB2312Code16x16[Num].Infor[2*(i-16)+1];
          }
        }
        else
        {
          for(i=0;i<16;i++)
          {
            Red_Color[i][Pos]=0x00;
            Red_Color[i][Pos+1]=0x00;
            Green_Color[i][Pos]=GB2312Code16x16[Num].Infor[2*i];
            Green_Color[i][Pos+1]=GB2312Code16x16[Num].Infor[2*i+1];
          } 
        }
      }
      else
      { 
        if(Pos/8)
        {
          for(i=16;i<32;i++)
          {
            Red_Color[i][Pos-8]=GB2312Code16x16[Num].Infor[2*(i-16)];
            Red_Color[i][Pos-7]=GB2312Code16x16[Num].Infor[2*(i-16)+1];
            Green_Color[i][Pos-8]=GB2312Code16x16[Num].Infor[2*(i-16)];
            Green_Color[i][Pos-7]=GB2312Code16x16[Num].Infor[2*(i-16)+1];
          }
        }
        else
        {
          for(i=0;i<16;i++)
          {
            Red_Color[i][Pos]=GB2312Code16x16[Num].Infor[2*i];
            Red_Color[i][Pos+1]=GB2312Code16x16[Num].Infor[2*i+1];
            Green_Color[i][Pos]=GB2312Code16x16[Num].Infor[2*i];
            Green_Color[i][Pos+1]=GB2312Code16x16[Num].Infor[2*i+1];
          } 
        }   
      }     
    }    
  }
}



void Matrix_FillAscii(Color_TypeDef Color, Pos_TypeDef Pos,uint8_t Ascii)
{
  uint16_t x;
  uint8_t i;
  x = (Ascii-0x20)*16;  // 计算ASCII形码位置
  
  if(Color==Color_Red)
  {
    if(Pos/8)
    {
      for(i=16;i<32;i++)
      {
        Red_Color[i][Pos-8]=AsciiCode16x8[x+i-16];
        Green_Color[i][Pos-8]=0x00;
      }
    }
    else
    {
      for(i=0;i<16;i++)
      {
        Red_Color[i][Pos]=AsciiCode16x8[x+i];
        Green_Color[i][Pos]=0x00;
      } 
    }
  }
  else if(Color==Color_Green)   
  {
    if(Pos/8)
    {
      for(i=16;i<32;i++)
      {
        Red_Color[i][Pos-8]=0x00;
        Green_Color[i][Pos-8]=AsciiCode16x8[x+i-16];
      }
    }
    else
    {
      for(i=0;i<16;i++)
      {
        Red_Color[i][Pos]=0x00;
        Green_Color[i][Pos]=AsciiCode16x8[x+i];
      } 
    }
  }
  else
  { 
    if(Pos/8)
    {
      for(i=16;i<32;i++)
      {
        Red_Color[i][Pos-8]=AsciiCode16x8[x+i-16];
        Green_Color[i][Pos-8]=AsciiCode16x8[x+i-16];
      }
    }
    else
    {
      for(i=0;i<16;i++)
      {
        Red_Color[i][Pos]=AsciiCode16x8[x+i];
        Green_Color[i][Pos]=AsciiCode16x8[x+i];
      } 
    }   
  }     
}


/*******************************************************************************
  * @brief  格式化填充
  * @param  None
  * @retval None
****************************************************************Author:Liming**/
uint8_t Screen_fill(Color_TypeDef Color, unsigned char Byte)
{
  unsigned char i,j;
  if(Data_busy==0)
  {     
    Data_busy=1;
    
    if(Color==Color_Red)
    {
      for(i=0;i<32;i++)
      {
        for(j=0;j<8;j++)
        {
          Red_Color[i][j]=Byte;
          Green_Color[i][j]=0x00;
        }
      }
    }
    else if(Color == Color_Green)
    {
      for(i=0;i<32;i++)
      {
        for(j=0;j<8;j++)
        {
          Red_Color[i][j]=0x00;
          Green_Color[i][j]=Byte;
        }
      }        
    }
    else
    {
      for(i=0;i<32;i++)
      {
        for(j=0;j<8;j++)
        {
          Red_Color[i][j]=Byte;
          Green_Color[i][j]=Byte;
        }
      }     
    }
    Data_busy=0; 
    return 1;
  }
  else
  {
    return 0;
  }
}

/*******************************************************************************
  * @brief  屏幕刷新一行显示
  * @param  None
  * @retval None
****************************************************************Author:Liming**/
void MATRIX_Display(void)
{
	static	uint16_t Column;
	uint8_t j,k,tempR,tempG;	

	if(Data_busy==0)
  {
    for(j=0;j<8;j++)
    {
      tempR = Red_Color[Column][7-j];	// 红色行
      tempG = Green_Color[Column][7-j];	// 绿色行
      
      for(k=0;k<8;k++)
      {
        DCLKR_LOW();
        DCLKG_LOW();			
        if(tempR&0x01)
        {
          GPIO_ResetBits(MATRIX_RowR_PORT,MATRIX_SDINR_PIN);          
        }
        else
        {

          GPIO_SetBits(MATRIX_RowR_PORT,MATRIX_SDINR_PIN);
        }
        if(tempG&0x01)
        {
          GPIO_ResetBits(MATRIX_RowG_PORT,MATRIX_SDING_PIN);          
        }
        else
        {

          GPIO_SetBits(MATRIX_RowG_PORT,MATRIX_SDING_PIN);
        }		
        DCLKR_HIGH();	
        DCLKG_HIGH();      
        tempR >>=1;	// LSB
        tempG >>=1;
      }      
    }
    CCLKR_HIGH();   // 更新数据
    CCLKG_HIGH();
    
    if(Column&0x01) // 切换一行的VCC
    { GPIOB->BSRR  = GPIO_Pin_0;}    
    else
    { GPIOB->BRR = GPIO_Pin_0;}
    if(Column&0x02)
    { GPIOB->BSRR  = GPIO_Pin_1;}    
    else
    { GPIOB->BRR = GPIO_Pin_1;}
    if(Column&0x04)
    { GPIOB->BSRR  = GPIO_Pin_2;}    
    else
    { GPIOB->BRR = GPIO_Pin_2;}
    if(Column&0x08)
    { GPIOB->BSRR  = GPIO_Pin_3;}    
    else
    { GPIOB->BRR = GPIO_Pin_3;}
    if(Column&0x10)
    { GPIOB->BSRR  = GPIO_Pin_4;}    
    else
    { GPIOB->BRR = GPIO_Pin_4;}
    
    Column++;
    if(Column>31)
    {
      Column=0;
    }
    CCLKR_LOW();
    CCLKG_LOW();    // 保持数据	    
  }
}


/*******************************************************************************
  * @brief  显示数据组合
  * @param  None
  * @retval None
****************************************************************Author:Liming**/
void Matrix_File(void)
{
  



}



/*******************************************************************************
  * @brief  内部flash读写函数
  * @param  WR_flag:读写功能选择
  *   @arg  1：写
  *   @arg  0: 读
  * @param  Address:要操作的地址
  * @retval None
****************************************************************Author:Liming**/
void	Flash_WriteRead(uint8_t	WR_flag,uint32_t FLASH_START_ADDR, uint8_t *Buff,uint16_t Length)
{
  uint8_t Pages,i;
  uint32_t  Address;
  uint32_t  DATA_32;
  
	if(WR_flag)
	{		 /* Unlock the Flash to enable the flash control register access *************/ 
		FLASH_Unlock();
			
    /* Clear pending flags (if any) */  
		FLASH_ClearFlag(FLASH_FLAG_EOP | FLASH_FLAG_PGERR | FLASH_FLAG_WRPERR); 
		
    /* Erase the user Flash area
			(area defined by FLASH_START_ADDR and length) ***********/

		/* Define the number of page to be erased */
		Pages = Length / FLASH_PAGE_SIZE;

		/* Erase the FLASH pages */
		for(EraseCounter = 0; EraseCounter < Pages; EraseCounter++)
		{
			if (FLASH_ErasePage(FLASH_USER_START_ADDR + (FLASH_PAGE_SIZE * EraseCounter))!= FLASH_COMPLETE)
			{
			 /* Error occurred while sector erase. 
					 User can add here some code to deal with this error  */
				FLASH_Lock();
				break;
			}
		}
		/* Program the user Flash area word by word
			(area defined by FLASH_USER_START_ADDR and FLASH_USER_END_ADDR) ***********/

		Address = FLASH_START_ADDR;   
    Length =Length/4;
		for(i=0;i<Length;i++)
		{     
      DATA_32 = Buff[i*4]+(uint32_t)(Buff[i*4+1]<<8)+(uint32_t)(Buff[i*4+2]<<16)+(uint32_t)(Buff[i*4+3]<<24);
			if (FLASH_ProgramWord(Address,DATA_32) == FLASH_COMPLETE)
			{
				Address = Address + 4;                
			}
			else
			{ 
				/* Error occurred while writing data in Flash memory. 
					 User can add here some code to deal with this error */
				FLASH_Lock();
				break;
			}
		}
		/* Lock the Flash to disable the flash control register access (recommended
			 to protect the FLASH memory against possible unwanted operation) *********/
		FLASH_Lock(); 
	}
	else
	{
    Address = FLASH_START_ADDR;  
    Length = Length/4;
    for(i=0;i<Length;i++)
    {    
      DATA_32 = *(__IO uint32_t *)(Address+i*4);	//read data     
      Buff[i*4]=(uint8_t)DATA_32;
      Buff[i*4+1]=(uint8_t)(DATA_32>>8);
      Buff[i*4+2]=(uint8_t)(DATA_32>>16);
      Buff[i*4+3]=(uint8_t)(DATA_32>>24);
    }        
	}
}


void Flash_WritePage(uint32_t PageNumber,uint8_t *Buff)
{
  uint32_t OneWord;
  uint16_t i;
  FLASH_Unlock();
  FLASH_ClearFlag(FLASH_FLAG_EOP | FLASH_FLAG_PGERR | FLASH_FLAG_WRPERR);
  if(FLASH_ErasePage(PageNumber)!= FLASH_COMPLETE)
  {
    FLASH_Lock();	
  }
  for(i=0;i<0x100;i++)
  {
    OneWord = Buff[i*4]+(uint32_t)(Buff[i*4+1]<<8)+(uint32_t)(Buff[i*4+2]<<16)+(uint32_t)(Buff[i*4+3]<<24);
    if (FLASH_ProgramWord(PageNumber+i*4,OneWord) != FLASH_COMPLETE)
    {
      FLASH_Lock(); 
      break;      
    }  
  }  
}

void Flash_ReadPage(uint32_t PageNumber,uint8_t *Buff)
{
  uint32_t OneWord;
  uint16_t i;
  for(i=0;i<0x100;i++)
  {
    OneWord = *(__IO uint32_t *)(PageNumber+i*4);	//read data  
    Buff[i*4]=OneWord;
    Buff[i*4+1]=OneWord>>8;
    Buff[i*4+2]=OneWord>>16;
    Buff[i*4+3]=OneWord>>24;
  }  
}



/*******************************************************************************
  * @brief  配置按键端口和相关的中端
  * @param  Button_Mode:指定按键模式
  *    @arg BUTTON_MODE_GPIO: Button will be used as simple IO 
  *    @arg BUTTON_MODE_EXTI: Button will be connected to EXTI line with interrupt
  * @retval None
****************************************************************Author:Liming**/
void Button_Init(ButtonMode_TypeDef Button_Mode)
{
	GPIO_InitTypeDef GPIO_InitStructure;
	EXTI_InitTypeDef EXTI_InitStructure;
	NVIC_InitTypeDef NVIC_InitStructure;
	
	/* Enable the GPIO Clock */
  RCC_AHBPeriphClockCmd(BUTTON_GPIO_CLK, ENABLE);
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_SYSCFG, ENABLE);
	
	/* Configure Button pin as input */
  GPIO_InitStructure.GPIO_Pin  = BUTTON_PIN;
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IN;
	GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_UP;
	GPIO_Init(BUTTON_GPIO_PORT, &GPIO_InitStructure);

	if (Button_Mode == BUTTON_MODE_EXTI)
	{
	/* Connect Button EXTI Line to Button GPIO Pin */
	SYSCFG_EXTILineConfig(BUTTON_EXTI_PORT_SOURCE, BUTTON_EXTI_PIN_SOURCE);

	/* Configure Button EXTI line */
	EXTI_InitStructure.EXTI_Line = BUTTON_EXTI_LINE;
	EXTI_InitStructure.EXTI_Mode = EXTI_Mode_Interrupt;
	EXTI_InitStructure.EXTI_Trigger = EXTI_Trigger_Falling;  
	EXTI_InitStructure.EXTI_LineCmd = ENABLE;
	EXTI_Init(&EXTI_InitStructure);

	/* Enable and set Button EXTI Interrupt to the lowest priority */
	NVIC_InitStructure.NVIC_IRQChannel = BUTTON_EXTI_IRQn;
	NVIC_InitStructure.NVIC_IRQChannelPriority= 0x01;
	NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;

	NVIC_Init(&NVIC_InitStructure); 
	}
}

/**
  * @brief  Returns the selected Button state.
  * @param  Button: Specifies the Button to be checked.
  *   This parameter must be: BUTTON_USER  
  * @retval The Button GPIO pin value.
  */
uint32_t Button_GetState(void)
{
  return GPIO_ReadInputDataBit(BUTTON_GPIO_PORT, BUTTON_PIN);
}

/**
  * @brief  Configure the TIM3 .
  * @param  Frequence 设置定时器频率
            Period    设置定时器周期
  * @retval None
  */
void TIM_Config(uint32_t Frequence , uint32_t Period)
{
	TIM_TimeBaseInitTypeDef	TIM_TimeBaseStructure;
	NVIC_InitTypeDef NVIC_InitStructure;

    
	/* TIM3 clock enable */
	RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM3, ENABLE); 

  PrescalerValue = (uint16_t) (SystemCoreClock  / Frequence) - 1;
    
	TIM_TimeBaseStructure.TIM_Period = Period;	//25000
	TIM_TimeBaseStructure.TIM_Prescaler = 0; 
	TIM_TimeBaseStructure.TIM_ClockDivision = 0;
	TIM_TimeBaseStructure.TIM_CounterMode = TIM_CounterMode_Up;
	TIM_TimeBaseInit(TIM3,&TIM_TimeBaseStructure);
	
  TIM_PrescalerConfig(TIM3, PrescalerValue, TIM_PSCReloadMode_Immediate);// 设置分频
  
  TIM_ITConfig(TIM3,TIM_IT_Update,ENABLE);
    
	/* Enable the TIM3 global Interrupt */
	NVIC_InitStructure.NVIC_IRQChannel = TIM3_IRQn;
	NVIC_InitStructure.NVIC_IRQChannelPriority = 2;
	NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
	NVIC_Init(&NVIC_InitStructure);
    
  TIM_Cmd(TIM3,ENABLE);
}


/*******************************************************************************
  * @brief	RGB彩灯控制端口初始化
  * @param  None
  * @retval None
****************************************************************Author:Liming**/
void LED_Init(void)
{
  GPIO_InitTypeDef  GPIO_InitStructure;
  
  /* Enable the GPIO Clock */
  RCC_AHBPeriphClockCmd(RCC_AHBPeriph_GPIOB, ENABLE);

  /* Configure the GPIO pin */
  GPIO_InitStructure.GPIO_Pin = GPIO_Pin_6|GPIO_Pin_7|GPIO_Pin_8|GPIO_Pin_10;
  GPIO_InitStructure.GPIO_Mode = GPIO_Mode_OUT;
  GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;
  GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_UP;
  GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
  GPIO_Init(MATRIX_COL_GPIO_PORT, &GPIO_InitStructure); 
    
  GPIO_SetBits(MATRIX_COL_GPIO_PORT,GPIO_Pin_6|GPIO_Pin_7|GPIO_Pin_8);
}


/*******************************************************************************
  * @brief  None
  * @param  None
  * @retval None
****************************************************************Author:Liming**/
void SystemClock_Config(void)
{             
}





void CC2500_InitPowerPin(void)
{
  GPIO_InitTypeDef  GPIO_InitStructure;
  
  RCC_AHBPeriphClockCmd(CC2500_POWER_PIN_CLK,ENABLE);
  
  GPIO_InitStructure.GPIO_Pin = CC2500_POWER_PIN;
  GPIO_InitStructure.GPIO_Mode = GPIO_Mode_OUT;
  GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
  GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;
  GPIO_InitStructure.GPIO_PuPd  = GPIO_PuPd_UP;
  GPIO_Init(CC2500_POWER_PIN_PORT, &GPIO_InitStructure);
  GPIO_SetBits(CC2500_POWER_PIN_PORT ,CC2500_POWER_PIN);
//  GPIO_ResetBits(CC2500_POWER_PIN_PORT,CC2500_POWER_PIN);  
}


/**
  * @brief  Initializes the SPI Interface used to drive the CC2500 wireless module
  * @note   None    
  * @param  None
  * @retval None
  */
void CC2500_SPI_Init(void)
{
  GPIO_InitTypeDef  GPIO_InitStructure;
  SPI_InitTypeDef   SPI_InitStructure;

  /* Enable GPIOs clock */
  RCC_AHBPeriphClockCmd(SPI2_MOSI_GPIO_CLK | SPI2_MISO_GPIO_CLK | SPI2_SCK_GPIO_CLK|SPI2_NSS_GPIO_CLK, ENABLE);

  /* Enable SPI clock */
  RCC_APB1PeriphClockCmd(CC2500_SPI_CLK, ENABLE); 

    
  /* Configure SPI SCK pin */
  GPIO_InitStructure.GPIO_Pin = SPI2_SCK_PIN;
  GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF;
  GPIO_InitStructure.GPIO_Speed = GPIO_Speed_10MHz;
  GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;
  GPIO_InitStructure.GPIO_PuPd  = GPIO_PuPd_UP;
  GPIO_Init(SPI2_SCK_GPIO_PORT, &GPIO_InitStructure);

  /* Configure SPI MISO pin */
  GPIO_InitStructure.GPIO_Pin = SPI2_MISO_PIN;
  GPIO_Init(SPI2_MISO_GPIO_PORT, &GPIO_InitStructure);

  /* Configure SPI MOSI pin */
  GPIO_InitStructure.GPIO_Pin = SPI2_MOSI_PIN;
  GPIO_Init(SPI2_MOSI_GPIO_PORT, &GPIO_InitStructure);
  
  /* Configure SPI NSS pin */
  GPIO_InitStructure.GPIO_Pin = SPI2_NSS_PIN;
  GPIO_InitStructure.GPIO_Mode = GPIO_Mode_OUT;
  GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_UP;
  GPIO_Init(SPI2_NSS_GPIO_PORT,&GPIO_InitStructure);
  
  /* Connect SCK, MISO and MOSI pins to SPI alternate */
  GPIO_PinAFConfig(SPI2_SCK_GPIO_PORT, SPI2_SCK_SOURCE, SPI2_SCK_AF);
  GPIO_PinAFConfig(SPI2_MISO_GPIO_PORT, SPI2_MISO_SOURCE, SPI2_MISO_AF);
  GPIO_PinAFConfig(SPI2_MOSI_GPIO_PORT, SPI2_MOSI_SOURCE, SPI2_MOSI_AF); 
  
  /* Configure SPI */
  SPI_InitStructure.SPI_Direction = SPI_Direction_2Lines_FullDuplex;
  SPI_InitStructure.SPI_Mode = SPI_Mode_Master;
  SPI_InitStructure.SPI_DataSize = SPI_DataSize_8b;
  SPI_InitStructure.SPI_CPOL = SPI_CPOL_High;
  SPI_InitStructure.SPI_CPHA = SPI_CPHA_2Edge;
  SPI_InitStructure.SPI_NSS = SPI_NSS_Soft;
      /* SPI baudrate is set to 9 MHz maximum (PCLK1/SPI_BaudRatePrescaler = 36/4 = 9 MHz) 
       to verify these constraints:
          - ST7735R LCD SPI interface max baudrate is 15MHz for write and 6.66MHz for read
            Since the provided driver doesn't use read capability from LCD, only constraint 
            on write baudrate is considered.
          - SD card SPI interface max baudrate is 25MHz for write/read
          - PCLK1 max frequency is 36 MHz 
       */
  SPI_InitStructure.SPI_BaudRatePrescaler = SPI_BaudRatePrescaler_8;
  SPI_InitStructure.SPI_FirstBit = SPI_FirstBit_MSB;
  SPI_InitStructure.SPI_CRCPolynomial = 7;
  SPI_Init(CC2500_SPI, &SPI_InitStructure);
  
  /* Configure RXFIFO to return its state each Quarter Full buffer (8its) */
  SPI_RxFIFOThresholdConfig(CC2500_SPI, SPI_RxFIFOThreshold_QF);
  
  /* Enable SPI */
  SPI_Cmd(CC2500_SPI, ENABLE);
}


/**
  * @brief  无线模块中断脚配置
  * @note   None  
  * @param  None
  * @retval None
  */
void CC2500_INT_Config(void)
{
	GPIO_InitTypeDef GPIO_InitStructure;
	EXTI_InitTypeDef EXTI_InitStructure;
	NVIC_InitTypeDef NVIC_InitStructure;
	
	/* Enable the GPIO Clock */
  RCC_AHBPeriphClockCmd(GDO0_GPIO_CLK|GDO2_GPIO_CLK, ENABLE);
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_SYSCFG, ENABLE);
	
	/* Configure GDO0 and GDO2 pin as input */
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IN;
	GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_DOWN;
  GPIO_InitStructure.GPIO_OType= GPIO_OType_PP;
	GPIO_InitStructure.GPIO_Pin = GDO0_PIN;
	GPIO_Init(GDO0_GPIO_PORT, &GPIO_InitStructure);
    
  GPIO_InitStructure.GPIO_Pin = GDO2_PIN;
	GPIO_Init(GDO2_GPIO_PORT, &GPIO_InitStructure);

	/* Connect GDO0 and GDO2 EXTI Line to Button GPIO Pin */
	SYSCFG_EXTILineConfig(GDO0_EXTI_PORT_SOURCE, GDO0_EXTI_PIN_SOURCE);
  SYSCFG_EXTILineConfig(GDO2_EXTI_PORT_SOURCE, GDO2_EXTI_PIN_SOURCE);
    
	/* Configure GDO0 EXTI line */
	EXTI_InitStructure.EXTI_Line = GDO0_EXTI_LINE;
	EXTI_InitStructure.EXTI_Mode = EXTI_Mode_Interrupt;
	EXTI_InitStructure.EXTI_Trigger = EXTI_Trigger_Falling;  
	EXTI_InitStructure.EXTI_LineCmd = ENABLE;
	EXTI_Init(&EXTI_InitStructure);

    /* Configure GDO2 EXTI line */
	EXTI_InitStructure.EXTI_Line = GDO2_EXTI_LINE; 
	EXTI_InitStructure.EXTI_Mode = EXTI_Mode_Interrupt;
	EXTI_InitStructure.EXTI_Trigger = EXTI_Trigger_Rising;  
  
	EXTI_InitStructure.EXTI_LineCmd = ENABLE;
	EXTI_Init(&EXTI_InitStructure);
  
  EXTI_ClearITPendingBit(GDO0_EXTI_LINE);
  EXTI_ClearITPendingBit(GDO2_EXTI_LINE);
	/* Enable and set Button EXTI Interrupt to the lowest priority */
	NVIC_InitStructure.NVIC_IRQChannel = GDO0_EXTI_IRQn;
	NVIC_InitStructure.NVIC_IRQChannelPriority= 0x03;
	NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
  NVIC_Init(&NVIC_InitStructure); 

  NVIC_InitStructure.NVIC_IRQChannel = GDO2_EXTI_IRQn;
	NVIC_InitStructure.NVIC_IRQChannelPriority= 0x03;
	NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;	
  NVIC_Init(&NVIC_InitStructure);

}

uint32_t GDO0_GetState(void)
{
  return GPIO_ReadInputDataBit(GDO0_GPIO_PORT, GDO0_PIN);
}

uint32_t GDO2_GetState(void)
{
  return GPIO_ReadInputDataBit(GDO2_GPIO_PORT, GDO2_PIN);
}





/**
  * @brief  Initializes the SPI Interface used to drive the W25Q64
  * @note   None  
  * @param  None
  * @retval None
  */
void W25Q16_Init(void)
{
  GPIO_InitTypeDef  GPIO_InitStructure;
  SPI_InitTypeDef   SPI_InitStructure;

  /* Enable GPIOs clock */
  RCC_AHBPeriphClockCmd(SPI1_MOSI_GPIO_CLK | SPI1_MISO_GPIO_CLK | SPI1_SCK_GPIO_CLK|SPI1_NSS_GPIO_CLK, ENABLE);

  /* Enable SPI clock */
  RCC_APB2PeriphClockCmd(W25Q16_SPI_CLK, ENABLE); 

    
  /* Configure SPI SCK pin */
  GPIO_InitStructure.GPIO_Pin = SPI1_SCK_PIN;
  GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF;
  GPIO_InitStructure.GPIO_Speed = GPIO_Speed_10MHz;
  GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;
  GPIO_InitStructure.GPIO_PuPd  = GPIO_PuPd_DOWN;
  GPIO_Init(SPI1_SCK_GPIO_PORT, &GPIO_InitStructure);

  /* Configure SPI MOSI pin */
  GPIO_InitStructure.GPIO_Pin = SPI1_MOSI_PIN;
  GPIO_Init(SPI1_MOSI_GPIO_PORT, &GPIO_InitStructure);
  
    /* Configure SPI MISO pin */
  GPIO_InitStructure.GPIO_Pin = SPI1_MISO_PIN;
//  GPIO_InitStructure.GPIO_OType = GPIO_OType_OD;  
  GPIO_Init(SPI1_MISO_GPIO_PORT, &GPIO_InitStructure);
  
  /* Configure SPI NSS pin */
  GPIO_InitStructure.GPIO_Pin = SPI1_NSS_PIN;
  GPIO_InitStructure.GPIO_Mode = GPIO_Mode_OUT;
  GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_UP;
  GPIO_Init(SPI1_NSS_GPIO_PORT,&GPIO_InitStructure);
  
  W25Q16_NSS_HIGH();
  
  /* Connect SCK, MISO and MOSI pins to SPI alternate */
  GPIO_PinAFConfig(SPI1_SCK_GPIO_PORT, SPI1_SCK_SOURCE, SPI1_SCK_AF);
  GPIO_PinAFConfig(SPI1_MISO_GPIO_PORT, SPI1_MISO_SOURCE, SPI1_MISO_AF);
  GPIO_PinAFConfig(SPI1_MOSI_GPIO_PORT, SPI1_MOSI_SOURCE, SPI1_MOSI_AF); 
  
  /* Configure SPI */
  SPI_InitStructure.SPI_Direction = SPI_Direction_2Lines_FullDuplex;
  SPI_InitStructure.SPI_Mode = SPI_Mode_Master;
  SPI_InitStructure.SPI_DataSize = SPI_DataSize_8b;
  SPI_InitStructure.SPI_CPOL = SPI_CPOL_High;
  SPI_InitStructure.SPI_CPHA = SPI_CPHA_2Edge;
  SPI_InitStructure.SPI_NSS = SPI_NSS_Soft;
  /* SPI baudrate is set to 9 MHz maximum (PCLK1/SPI_BaudRatePrescaler = 36/4 = 9 MHz) 
   to verify these constraints:
      - ST7735R LCD SPI interface max baudrate is 15MHz for write and 6.66MHz for read
        Since the provided driver doesn't use read capability from LCD, only constraint 
        on write baudrate is considered.
      - SD card SPI interface max baudrate is 25MHz for write/read
      - PCLK1 max frequency is 36 MHz 
   */
  SPI_InitStructure.SPI_BaudRatePrescaler = SPI_BaudRatePrescaler_8;
  SPI_InitStructure.SPI_FirstBit = SPI_FirstBit_MSB;
  SPI_InitStructure.SPI_CRCPolynomial = 7;
  SPI_Init(W25Q16_SPI, &SPI_InitStructure);
  
  /* Configure RXFIFO to return its state each Quarter Full buffer (8its) */
  SPI_RxFIFOThresholdConfig(W25Q16_SPI, SPI_RxFIFOThreshold_QF);
  
  /* Enable SPI */
  SPI_Cmd(W25Q16_SPI, ENABLE);
}


/**
  * @brief  Sends a byte through the SPI interface and return the byte received 
  *         from the SPI bus.
  * @param  Data: byte send.
  * @retval The received byte value
  * @retval None
  */
uint8_t CC2500_SPI_WriteRead(uint8_t Data)
{
  uint8_t tmp = 0x00;
  
  /* Wait until the transmit buffer is empty */ 
  while(SPI_I2S_GetFlagStatus(CC2500_SPI, SPI_I2S_FLAG_TXE) != SET)
  {
  }  
  /* Send the byte */
  SPI_SendData8(CC2500_SPI, Data);
  
  /* Wait to receive a byte */ 
  while(SPI_I2S_GetFlagStatus(CC2500_SPI, SPI_I2S_FLAG_RXNE) != SET)
  {
  }
  /* Return the byte read from the SPI bus */    
  tmp = SPI_ReceiveData8(CC2500_SPI); 

  /* Wait until the BSY flag is set */   
  while(SPI_I2S_GetFlagStatus(CC2500_SPI, SPI_I2S_FLAG_BSY) != RESET)
  {
  }  
 
  /* Return read Data */
  return tmp;
}

/**
  * @brief  Sends a byte through the SPI interface and return the byte received 
  *         from the SPI bus.
  * @param  Data: byte send.
  * @retval The received byte value
  * @retval None
  */
uint8_t W25Q16_SPI_WriteRead(uint8_t Data)
{
  uint8_t tmp = 0x00;
  
  /* Wait until the transmit buffer is empty */ 
  while(SPI_I2S_GetFlagStatus(W25Q16_SPI, SPI_I2S_FLAG_TXE) != SET)
  {
  }  
  /* Send the byte */
  SPI_SendData8(W25Q16_SPI, Data);
  
  /* Wait to receive a byte */ 
  while(SPI_I2S_GetFlagStatus(W25Q16_SPI, SPI_I2S_FLAG_RXNE) != SET)
  {
  }
  /* Return the byte read from the SPI bus */    
  tmp = SPI_ReceiveData8(W25Q16_SPI); 

  /* Wait until the BSY flag is set */   
  while(SPI_I2S_GetFlagStatus(W25Q16_SPI, SPI_I2S_FLAG_BSY) != RESET)
  {
  }  
 
  /* Return read Data */
  return tmp;
}

/**
  * @brief  Initializes ADC, used to detect motion of Joystick available on 
  *         adafruit 1.8" TFT shield.
  * @param  None
  * @retval None
  */
void STM_ADC_Config(void)
{
  GPIO_InitTypeDef      GPIO_InitStructure;
  ADC_InitTypeDef       ADC_InitStructure;
      
  /* Enable ADC clock */
  RCC_APB2PeriphClockCmd(ADC_CLK, ENABLE);

  /* Enable GPIO clock */
  RCC_AHBPeriphClockCmd(ADC_GPIO_CLK, ENABLE);
      
  /* Configure ADC1 Channel 8 as analog input */
  GPIO_InitStructure.GPIO_Pin = ADC_GPIO_PIN ;
  GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AN;
  GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_NOPULL ;
  GPIO_Init(ADC_GPIO_PORT, &GPIO_InitStructure);  
  ADC_StructInit(&ADC_InitStructure);
  
  /* ADC1 DeInit */  
  ADC_DeInit(ADC1);
  
  /* Initialize ADC structure */
  ADC_StructInit(&ADC_InitStructure);
  
  /* Configure the ADC1 in continous mode withe a resolutuion equal to 12 bits  */
  ADC_InitStructure.ADC_Resolution = ADC_Resolution_12b;
  ADC_InitStructure.ADC_ContinuousConvMode = ENABLE; 
  ADC_InitStructure.ADC_ExternalTrigConvEdge = ADC_ExternalTrigConvEdge_None;
  ADC_InitStructure.ADC_DataAlign = ADC_DataAlign_Right;
  ADC_InitStructure.ADC_ScanDirection = ADC_ScanDirection_Upward;
  ADC_Init(ADC1, &ADC_InitStructure); 
  
  /* Convert the ADC1 Channel 8 with 239.5 Cycles as sampling time */ 
  ADC_ChannelConfig(ADC1, ADC_Channel_8 , ADC_SampleTime_239_5Cycles);   

  /* ADC Calibration */
  ADC_GetCalibrationFactor(ADC1);
  
  /* Enable ADCperipheral[PerIdx] */
  ADC_Cmd(ADC1, ENABLE);     
  
  /* Wait the ADRDY falg */
  while(!ADC_GetFlagStatus(ADC1, ADC_FLAG_ADRDY)); 
  
  /* ADC1 regular Software Start Conv */ 
  ADC_StartOfConversion(ADC1);

}

/**
  * @brief  Returns the Joystick key pressed.
  * @note   To know which Joystick key is pressed we need to detect the voltage
  *         level on each key output
  *           - SEL   : 1.055 V / 1308
  *           - RIGHT : 0.595 V / 737
  *           - LEFT  : 3.0 V / 3720 
  *           - UP    : 1.65 V / 2046
  *           - DOWN  : 0.71 V / 88
  *           - None  : 3.3 V / 4095
  * @retval Code of the Joystick key pressed.
  *          This code can be one of the following values:
  *            @arg  JOY_NONE
  *            @arg  JOY_SEL
  *            @arg  JOY_DOWN
  *            @arg  JOY_LEFT
  *            @arg  JOY_RIGHT
  *            @arg  JOY_UP  
  */
JOYState_TypeDef STM_Get_JOYState(void)
{
  JOYState_TypeDef state = JOY_NONE;
  uint16_t  KeyConvertedValue = 0; 
  KeyConvertedValue = ADC_GetConversionValue(ADCx);
  
  if((KeyConvertedValue > 2010) && (KeyConvertedValue < 2090))
  {
    state = JOY_UP;
  }
  else if((KeyConvertedValue > 680) && (KeyConvertedValue < 780))
  {
    state = JOY_RIGHT;
  }
  else if((KeyConvertedValue > 1270) && (KeyConvertedValue < 1350))
  {
    state = JOY_SEL;
  }
  else if((KeyConvertedValue > 50) && (KeyConvertedValue < 130))
  {
    state = JOY_DOWN;
  }
  else if((KeyConvertedValue > 3680) && (KeyConvertedValue < 3760))
  {
    state = JOY_LEFT;
  }
  else
  {
    state = JOY_NONE;
  }
  /* Loop while a key is pressed */
  if(state != JOY_NONE)
  { 
    while(KeyConvertedValue < 4000)
    {
      KeyConvertedValue = ADC_GetConversionValue(ADCx);
    }      
  }
  /* Return the code of the Joystick key pressed*/
  return state;
}

/**
  * @}
  */ 

/**
  * @}
  */ 

/**
  * @}
  */   

/**
  * @}
  */ 
      
/*********************** (C) COPYRIGHT 2017 IdeaMing ******END OF FILE*********/
