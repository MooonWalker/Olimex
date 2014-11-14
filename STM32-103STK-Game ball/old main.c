/*************************************************************************
 *
 *    Used with ICCARM and AARM.
 *
 *    (c) Copyright IAR Systems 2007
 *
 *    File name   : main.c
 *    Description : CAN test
 *
 *    History :
 *    1. Date        : April 28, 2007
 *       Author      : Stanimir Bonev
 *       Description : Create
 *
 *    $Revision: 1.4 $
 **************************************************************************/
// #include <stdio.h>

#include "stm32f10x_lib.h"
#include "arm_comm.h"
#include "lcd.h"
#include "stm32f10x_map.h"
#include "adc.h"
#include "bits.h"
#include "const.h"
#include "i2c.h"
#include "drv_lisxxx.h"

#define NSAMPLE  8

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


//__root __no_init volatile CAN_TypeDef     ttt @ CAN_BASE;
__root __no_init volatile RCC_TypeDef     ttt1 @ RCC_BASE;
__root __no_init volatile GPIO_TypeDef    ttt2 @ GPIOB_BASE;
__root __no_init volatile AFIO_TypeDef    ttt3 @ AFIO_BASE;
__root __no_init volatile I2C_TypeDef     ttt4 @ I2C1_BASE;

extern unsigned char status;
extern GPIO_InitTypeDef GPIO_InitStructure;
I2C_InitTypeDef I2C_InitStructure;
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
 * Function Name: Clk_Init
 * Parameters: Int32U Frequency
 * Return: Int32U
 *
 * Description: Init clock system
 *
 *************************************************************************/
void Clk_Init(void)
{
  // 1. Cloking the controller from internal HSI RC (8 MHz)
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
//  RCC_PLLConfig(RCC_PLLSource_HSE_Div2,RCC_PLLMul_9); // 72MHz
  RCC_PLLCmd(ENABLE);
  // wait until the PLL is ready
  while(RCC_GetFlagStatus(RCC_FLAG_PLLRDY) == RESET);
  // 4. Set system clock divders
  RCC_USBCLKConfig(RCC_USBCLKSource_PLLCLK_1Div5);
  RCC_ADCCLKConfig(RCC_PCLK2_Div8);
  RCC_PCLK2Config(RCC_HCLK_Div1);
  RCC_PCLK1Config(RCC_HCLK_Div2);
  RCC_HCLKConfig(RCC_SYSCLK_Div1);
  /* Flash 1 wait state */
  *(vu32 *)0x40022000 = 0x01;
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
    col= col - Coordinates[0] / 2000;
    if(col<0)     col=0;
    if(col>13)     col=13;
    //get number of row   
    row = row - Coordinates[1] / 3000;
    if(row>5)     row=5;
    if(row<0)     row=0;
    //print square on screen 
    WriteBall(col, row);
    // -------------------------------------------------------------------------
  }

}


/*************************************************************************
 * Function Name: main
 * Parameters: none
 * Return: Int32U
 *
 * Description: The main subroutine
 *
 *************************************************************************/
int main(void)
{

  #ifdef DEBUG
    debug();
  #endif

  // Init clock system
  Clk_Init();

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

/******************* (C) COPYRIGHT 2007 STMicroelectronics *****END OF FILE****/
