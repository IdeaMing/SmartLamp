/**
  ******************************************************************************
  * @file    main.c 
  * @author  Liming
  * @version V1.1.0
  * @date    11-November-2017
  * @brief   Main program body
  ******************************************************************************
  * @attention
  *
  * <h2><center>&copy; COPYRIGHT 2017 IdeaMing</center></h2>
  *
  * @Licensed None
  * @Revision 
  ******************************************************************************
  */

/* Includes ------------------------------------------------------------------*/
#include "main.h"

/** @addtogroup IO_Toggle
  * @{
  */ 

/* Private typedef -----------------------------------------------------------*/
/* Private define ------------------------------------------------------------*/
/* Private macro -------------------------------------------------------------*/
/* Private variables ---------------------------------------------------------*/

static __IO uint32_t TimingDelay; // ϵͳʱ�Ӷ�ʱ��
static __IO uint32_t Flash_ID;    // ��ȡ��W25Q16ID
uint8_t FlashBuff[100];          // ����ȡFlashʹ�õĻ���
uint16_t PrintCount;              // Debug�������
uint8_t temmp,i,j,k;              // ������
uint8_t Buffer[0x40]={"ABCDEFGHIJKLMNOPQRSTUVWXYZabcdefghijklmnopqrstuvwxyz"};               // ������
uint8_t DATACNT;                  // ������
//ϵͳ����
struct STRUPARAM
{
  unsigned short ID;
  unsigned char Channel;
  unsigned char Sleep;  
}Param;

//�������ݰ�
struct STRUPACKET
{
  unsigned char Index;
  enum PRO_PACKET_VALUE Value;         //����ֵ
  unsigned char Param[0x08];
  unsigned long Time;
}Packet;


//���ͻ�����
struct STRUSENDBUFFER
{
  unsigned char Buffer[SENDPACKETLEN];    //���ͻ�����
  unsigned char SendOverTime;             //��ʱ��ʱ����
  unsigned char Mode;                     //00:ֹͣ����,01:����,02:�������
}SendBuffer;

//��ʱ���
struct STRUOVERTIME
{
  unsigned short IdleOverTime;        //���г�ʱ
  unsigned short SignalOverTime;      //�źŶ�ʧ��ʱ
  unsigned short WakeupOverTime;      //���ѳ�ʱ
  unsigned short AdDispOverTime;      //AD��ʾ��ʱ
  unsigned short AdReadOverTime;      //AD��ȡ��ʱ
}OverTime;

//�ŵ�ɨ��
struct STRUSCANCHANNEL
{
  unsigned char ScanTag;              //ɨ����,0:��ɨ��,1:�ֶ����,2:�Զ�ɨ��,3:ǿ�����
  unsigned char ScanLoop;             //ɨ�����
  unsigned short OverTime;            //ɨ�賬ʱ
  unsigned short ScanOverTime;        //ɨ�賬ʱ
  unsigned char Channel;              //��ǰ�ŵ�
}ScanChannel;

//RSSIֵ
struct STRURSSIVALUE
{
  unsigned short Value;
  unsigned char Count;
  unsigned char Rssi;
}RssiValue;

/* Private function prototypes -----------------------------------------------*/
RCC_ClocksTypeDef RCC_Clocks;
/* Private functions ---------------------------------------------------------*/
void FlashTest(void);
void InternalFlashTest(void);
/**
  * @brief   Main program
  * @param  None
  * @retval None
  */
int main(void)
{
  /*!< At this stage the microcontroller clock setting is already configured, 
       this is done through SystemInit() function which is called from startup
       file (startup_stm32f030.s) before to branch to application main.
       To reconfigure the default setting of SystemInit() function, refer to
       system_stm32f0xx.c file
     */ 
  
  /* SysTick end of count event each 1ms */
  RCC_GetClocksFreq(&RCC_Clocks);
  SysTick_Config(RCC_Clocks.HCLK_Frequency / 1000);  

  Matrix_ColumnInit();	
  Matrix_RowInit();  
  TIM_Config(48000000,25000);

  CC2500_SPI_Init();
  if(!CC2500_Init(0,0)) // ��ʼ��ʧ��
  {
    Matrix_FillAscii(Color_Red,Pos00,'2');
    Matrix_FillAscii(Color_Red,Pos01,'5');
    Matrix_FillAscii(Color_Red,Pos02,'0');
    Matrix_FillAscii(Color_Red,Pos03,'0');
    Matrix_FillAscii(Color_Red,Pos05,'E');
    Matrix_FillAscii(Color_Red,Pos06,'r');
    Matrix_FillAscii(Color_Red,Pos07,'r');    
  }   
  else 
  {
    Matrix_FillAscii(Color_Red,Pos00,'2');
    Matrix_FillAscii(Color_Red,Pos01,'5');
    Matrix_FillAscii(Color_Red,Pos02,'0');
    Matrix_FillAscii(Color_Red,Pos03,'0');

    Matrix_FillAscii(Color_Red,Pos06,'O');
    Matrix_FillAscii(Color_Red,Pos07,'K'); 
  }
  CC2500_INT_Config();
  Delay(500);
  Screen_fill(Color_Yellow,0x00);  

  /* Infinite loop */
  while (1)
  {                                 
#ifdef  DEBUG     
    printf ("%d\r\n",CC2500_GETPACKT);
#endif
        
    Delay(600);                
    ProPacketUpload(0x0001,0x0000,0,0,Buffer,SendBuffer.Buffer);
    DATACNT++;        
    if(DATACNT>99)DATACNT=0;
    Buffer[2]=DATACNT/10+'0';
    Buffer[3]=DATACNT%10+'0';
    if(CC2500_GETPACKT==0)
    { 
      CC2500_GETPACKT=1;
      CC2500_Send(SendBuffer.Buffer); 
    }                                  
  }
}

/*******************************************************************************
  * @brief  �ⲿflash����
  * @param  None
  * @retval None
****************************************************************Author:Liming**/
void FlashTest(void)
{
  unsigned char i;
  for(i=0;i<250;i++)
  {
    FlashBuff[i]=i;    
  }  
  SPI_FLASH_SectorErase(4096);
  SPI_FLASH_BufferWrite(FlashBuff,4096,100);
  for(i=0;i<250;i++)
  {
    FlashBuff[i]=0;        
  }  
  SPI_FLASH_BufferRead(FlashBuff,4096,100);
}

/*******************************************************************************
  * @brief  �ڲ�flash����
  * @param  None
  * @retval None
****************************************************************Author:Liming**/
void InternalFlashTest(void)
{
  uint16_t i;
  
  FlashBuff[0]=0x5a;
  FlashBuff[1]=0xcd;
  FlashBuff[2]=0x1d;
  FlashBuff[3]=0x2e;
  for(i=4;i<0x100;i++)
  {
    FlashBuff[i]=0x5a;
  }
  for(i=0x100;i<0x200;i++)
  {
    FlashBuff[i]=0x3a;
  }
  for(i=0x200;i<0x300;i++)
  {
    FlashBuff[i]=0x1a;
  }
  for(i=0x300;i<0x400;i++)
  {
    FlashBuff[i]=0x0a;
  }
  Flash_WritePage(INTERNAL_FLASH_PAGE0,FlashBuff);
  for(i=0;i<0x400;i++)
  {
    FlashBuff[i]=0x05;
  }  
  Flash_ReadPage(INTERNAL_FLASH_PAGE0,FlashBuff);  
}



/**
* @brief  Inserts a delay time.
* @param  nTime: specifies the delay time length, in 1 ms.
* @retval None
*/
void Delay(__IO uint32_t nTime)
{
  TimingDelay = nTime;
 
  while(TimingDelay != 0);
}

/**
* @brief  Decrements the TimingDelay variable.
* @param  None
* @retval None
*/
void TimingDelay_Decrement(void)
{
  if (TimingDelay != 0x00)
  { 
    TimingDelay--;
  }
}


#ifdef  USE_FULL_ASSERT

/**
* @brief  Reports the name of the source file and the source line number
*         where the assert_param error has occurred.
* @param  file: pointer to the source file name
* @param  line: assert_param error line source number
* @retval None
*/
void assert_failed(uint8_t* file, uint32_t line)
{ 
  /* User can add his own implementation to report the file name and line number,
  ex: printf("Wrong parameters value: file %s on line %d\r\n", file, line) */
  
  /* Infinite loop */
  while (1)
  {
  }
}
#endif

/**
* @}
*/


/*********************** (C) COPYRIGHT 2017 IdeaMing ******END OF FILE*********/
