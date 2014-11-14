/*************************************************************************
 *
 *    Used with ICCARM and AARM.
 *
 *    (c) Copyright IAR Systems 2007
 *
 *    File name   : main.c
 *    Description : Define main module
 *
 *    History :
 *    1. Date        : 19, July 2006
 *       Author      : Stanimir Bonev
 *       Description : Create
 *
 *  This example project shows how to use the IAR Embedded Workbench
 * for ARM to develop code for the IAR STM32-SK board. It implements USB
 * HID mouse. When host install needed driver a mouse's cursor begin moved
 * in rectangle shape. The WAKE-UP button is used for USB resume when device
 * is suspended.
 *
 *  Controls:
 *   WAKE-UP - USB resume, when device is suspended
 *
 *  Jumpers:
 *   PWR_SEL - depending of power source
 *
 *    $Revision: 19278 $
 **************************************************************************/
#include "includes.h"

#define LOOP_DLY_100US  450
#define NSAMPLE  16

// MMA -------------------------------------------------------------------------

#define WHO_AM_I          0x0F
#define OFFSET_X          0x16
#define OFFSET_Y          0x17
#define OFFSET_Z          0x18
#define GAIN_X            0x19
#define GAIN_Y            0x1A
#define GAIN_Z            0x1B
#define CTRL_REG1         0x20
#define CTRL_REG2         0x21
#define CTRL_REG3         0x22
#define HP_FILTER_RESET   0x23
#define STATUS_REG        0x27
#define OUTX_L            0x28
#define OUTX_H            0x29
#define OUTY_L            0x2A
#define OUTY_H            0x2B
#define OUTZ_L            0x2C
#define OUTZ_H            0x2D
#define FF_WU_CFG         0x30
#define FF_WU_SRC         0x31
#define FF_WU_ACK         0x32
#define FF_WU_THS_L       0x34
#define FF_WU_THS_H       0x35
#define FF_WU_DURATION    0x36
#define DD_CFG            0x38
#define DD_SRC            0x39
#define DD_ACK            0x3A
#define DD_THSI_L         0x3C
#define DD_THSI_H         0x3D
#define DD_THSE_L         0x3E
#define DD_THSE_H         0x3F
#define LIS_ADDRESS       0x3A


volatile Boolean Update = TRUE;
Int32U CriticalSecCntr;

extern GPIO_InitTypeDef GPIO_InitStructure;
volatile unsigned int dly;
Int16U ADCValue;
unsigned char jjj=0;


unsigned char value_h;
unsigned char value_l;
unsigned int  value;
unsigned char val;

int tmp, g;
union acc_data
{ 
  unsigned char data[2];
  signed short result;
}
;
union acc_data  data_x,data_y;
signed short offset_x,offset_y; 
signed short my_x,my_y;
int x,y;
int base_x, base_y;
int Coordinates[3];

int row=2,col=6;
unsigned char sign_x,sign_y;


void Delay_ (unsigned long a) { while (--a!=0); }


/*************************************************************************
 * Function Name: Timer1IntrHandler
 * Parameters: none
 *
 * Return: none
 *
 * Description: Timer 1 interrupt handler
 *
 *************************************************************************/
void Timer1IntrHandler (void)
{
  // Clear update interrupt bit
  TIM1_ClearITPendingBit(TIM1_FLAG_Update);
  Update = TRUE;
}


/*************************************************************************
 * Function Name: Clk_Init
 * Parameters: Int32U Frequency
 * Return: Int32U
 *
 * Description: Init clock system
 *
 *************************************************************************/
void Clk_Init (void)
{
  // 1. Clocking the controller from internal HSI RC (8 MHz)
  RCC_HSICmd(ENABLE);
  // wait until the HSI is ready
  while(RCC_GetFlagStatus(RCC_FLAG_HSIRDY) == RESET);
  RCC_SYSCLKConfig(RCC_SYSCLKSource_HSI);
  // 2. Enable ext. high frequency OSC
  RCC_HSEConfig(RCC_HSE_ON);
  // wait until the HSE is ready
  while(RCC_GetFlagStatus(RCC_FLAG_HSERDY) == RESET);
  // 3. Init PLL
  RCC_PLLConfig(RCC_PLLSource_HSE_Div1,RCC_PLLMul_9); // 72MHz
  RCC_PLLCmd(ENABLE);
  // wait until the PLL is ready
  while(RCC_GetFlagStatus(RCC_FLAG_PLLRDY) == RESET);
  // 4. Set system clock dividers
  RCC_USBCLKConfig(RCC_USBCLKSource_PLLCLK_1Div5);
  RCC_ADCCLKConfig(RCC_PCLK2_Div8);
  RCC_PCLK2Config(RCC_HCLK_Div1);
  RCC_PCLK1Config(RCC_HCLK_Div2);
  RCC_HCLKConfig(RCC_SYSCLK_Div1);
#ifdef EMB_FLASH
  // 5. Init Embedded Flash
  // Zero wait state, if 0 < HCLK 24 MHz
  // One wait state, if 24 MHz < HCLK 56 MHz
  // Two wait states, if 56 MHz < HCLK 72 MHz
  // Flash wait state
  FLASH_SetLatency(FLASH_Latency_2);
  // Half cycle access
  FLASH_HalfCycleAccessCmd(FLASH_HalfCycleAccess_Disable);
  // Prefetch buffer
  FLASH_PrefetchBufferCmd(FLASH_PrefetchBuffer_Enable);
#endif // EMB_FLASH
  // 5. Clock system from PLL
  RCC_SYSCLKConfig(RCC_SYSCLKSource_PLLCLK);
}


void InitMMA(void) {

   // GPIOB Periph clock enable
  RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOB, ENABLE);

  // Configure PB5 as input - SENS_INT
  GPIO_InitStructure.GPIO_Pin =  GPIO_Pin_5;
  GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IPD;
  GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
  GPIO_Init(GPIOB, &GPIO_InitStructure);

  Hrd_I2C_Init();

  // Who Am I
  ReadReg(&val, WHO_AM_I);    // have to be 0x3A

  // Set CTRL_REG1
  // WriteReg(0xC7, CTRL_REG1);
  WriteReg(0xC7, CTRL_REG1);
  //ReadReg(&val, CTRL_REG1);

  // Set CTRL_REG2
  // WriteReg(0xC5, CTRL_REG2);
  WriteReg(0x54, CTRL_REG2);  //12bit signed result
  //ReadReg(&val, CTRL_REG2);

  // Set CTRL_REG3
  WriteReg(0x00, CTRL_REG3);
  //ReadReg(&val, CTRL_REG3);

}


/****************************************************************************/
/*  Get MMA coordinates (X,Y,Z)                                             */
/*  Function : GetCoordinates                                               */
/****************************************************************************/
//*************************************************************************
void GetCoordinates (void) {

  unsigned char n;
  volatile unsigned int i;

  Coordinates[0] = 0;
  Coordinates[1] = 0;
  Coordinates[2] = 0;

  for (n=0; n < NSAMPLE; n++)
  {
    // X
      value = value_h = value_l = 0;
      ReadReg(&value_l, OUTX_L);
      data_x.data[0]=value_l;
      ReadReg(&value_h, OUTX_H);
      data_x.data[1]=value_h;
      value = (value_h<<8)|value_l;
//      Coordinates[0] += value;
      Coordinates[0] +=data_x.result;
     
      // Y
      value = value_h = value_l = 0;
      ReadReg(&value_l, OUTY_L);
      data_y.data[0]=value_l;
      ReadReg(&value_h, OUTY_H);
      data_y.data[1]=value_h;
      value = (value_h<<8)|value_l;
//      Coordinates[1] += value;
      Coordinates[1] +=data_y.result;
      // Z
      value = value_h = value_l = 0;
      ReadReg(&value_l, OUTZ_L);
      ReadReg(&value_h, OUTZ_H);
      value = (value_h<<8)|value_l;
      Coordinates[2] += value;
  }

  Coordinates[0] /= NSAMPLE;
  Coordinates[1] /= NSAMPLE;
  Coordinates[2] /= NSAMPLE;

}


void WriteBall(unsigned char row, unsigned char col) {

    LCDClear();
  LCDChrXY2(row, col);
   LCDUpdate();
}


void TestMMA(void) {


  while(1) {

    // mma and square ------------------------------------------------------------

    //Initialization------------------------------------------------------------
      col=6;
      row=2;

      //      get coordinates
    
      GetCoordinates();

      //get number of column
    col= col - Coordinates[0] / 100;
    if(col<0)     col=0;
    if(col>13)     col=13;
    //get number of row   
    row = row - Coordinates[1] / 150;
    if(row>5)     row=5;
    if(row<0)     row=0;
    //print square on screen 
    WriteBall(col, row);
    // -------------------------------------------------------------------------
  }

}
/*************************************************************************
 * Function Name: Dly100us
 * Parameters: Int32U Dly
 *
 * Return: none
 *
 * Description: Delay Dly * 100us
 *
 *************************************************************************/
void Dly100us(void *arg)
{
Int32U Dly = (Int32U)arg;
  while(Dly--)
  {
    for(volatile int i = LOOP_DLY_100US; i; i--);
  }
}

/*************************************************************************
 * Function Name: main
 * Parameters: none
 *
 * Return: none
 *
 * Description: main
 *
 *************************************************************************/
void main(void)
{
NVIC_InitTypeDef NVIC_InitStructure;
TIM1_TimeBaseInitTypeDef TIM1_TimeBaseInitStruct;
GPIO_InitTypeDef GPIO_InitStructure;


#ifdef DEBUG
 debug();
#endif

  ENTR_CRT_SECTION();

  // Init clock system
  Clk_Init();

  // NVIC init
#ifndef  EMB_FLASH
  /* Set the Vector Table base location at 0x20000000 */
  NVIC_SetVectorTable(NVIC_VectTab_RAM, 0x0);
#else  /* VECT_TAB_FLASH  */
  /* Set the Vector Table base location at 0x08000000 */
  NVIC_SetVectorTable(NVIC_VectTab_FLASH, 0x0);
#endif
  NVIC_PriorityGroupConfig(NVIC_PriorityGroup_4);

  // Init USB wakeup button (WAKE_UP (PA0))
  // Enable GPIO clock and release reset
  RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOA,
                         ENABLE);
  RCC_APB2PeriphResetCmd(RCC_APB2Periph_GPIOA,
                         DISABLE);

  GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
  GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IN_FLOATING;
  GPIO_InitStructure.GPIO_Pin = GPIO_Pin_0;
  GPIO_Init(GPIOA,&GPIO_InitStructure);

 /* // Timer1 Init
  // Enable Timer1 clock and release reset
  RCC_APB2PeriphClockCmd(RCC_APB2Periph_TIM1,ENABLE);
  RCC_APB2PeriphResetCmd(RCC_APB2Periph_TIM1,DISABLE);

  // Set timer period 0.01 sec
  TIM1_TimeBaseInitStruct.TIM1_Prescaler = 720;  // 10us resolution
  TIM1_TimeBaseInitStruct.TIM1_CounterMode = TIM1_CounterMode_Up;
  TIM1_TimeBaseInitStruct.TIM1_Period = 1000;  // 10 ms
  TIM1_TimeBaseInitStruct.TIM1_ClockDivision = TIM1_CKD_DIV1;
  TIM1_TimeBaseInitStruct.TIM1_RepetitionCounter = 0;
  TIM1_TimeBaseInit(&TIM1_TimeBaseInitStruct);

  // Clear update interrupt bit
  TIM1_ClearITPendingBit(TIM1_FLAG_Update);
  // Enable update interrupt
  TIM1_ITConfig(TIM1_FLAG_Update,ENABLE);

  NVIC_InitStructure.NVIC_IRQChannel = TIM1_UP_IRQChannel;
  NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 7;
  NVIC_InitStructure.NVIC_IRQChannelSubPriority = 0;
  NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
  NVIC_Init(&NVIC_InitStructure);

  // Enable timer counting
  TIM1_Cmd(ENABLE);*/

  // HID USB
 // HidInit();
  // Soft connection enable
  //USB_ConnectRes(TRUE);

  EXT_CRT_SECTION();

  // LCD Init
// LCD init
  LCDInit();
  LCDContrast(0x45);

  LCDClear();
  LCDStr ( 0, "   Welcome to ", 0 );
  LCDStr ( 1, " STM32F103-STK", 0 );
  LCDStr ( 2, "  development ", 0 );
  LCDStr ( 3, "     board    ", 0 );
  LCDStr ( 5, "www.olimex.com", 1 );

  LCDUpdate();

  // Delay :-)
  for(dly=0; dly<5000000; dly++);


  // Test MMA--------------------------------------------------------
    InitMMA();
   
    while(GPIO_ReadInputDataBit(GPIOC, GPIO_Pin_13) == Bit_RESET) {
      
       TestMMA();
      // for(dly=0; dly<1000000; dly++);
    }
}


#ifdef  DEBUG
/*******************************************************************************
* Function Name  : assert_failed
* Description    : Reports the name of the source file and the source line number
*                  where the assert_param error has occurred.
* Input          : - file: pointer to the source file name
*                  - line: assert_param error line source number
* Output         : None
* Return         : None
*******************************************************************************/
void assert_failed(u8* file, u32 line)
{ 
  /* User can add his own implementation to report the file name and line number,
     ex: printf("Wrong parameters value: file %s on line %d\r\n", file, line) */

  /* Infinite loop */
  while (1)
  {
  }
}
#endif
