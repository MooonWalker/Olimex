#include "stm32f10x.h"
#include "Olimexino-STM32.h"
#include "lcd3310.h"

#define LED2PIN GPIO_Pin_1   /* yellow LED PA1 0x0002 */
#define HIGH Bit_SET
#define LOW Bit_RESET

volatile uint32_t msTicks;
volatile uint32_t nowTicks;

GPIO_InitTypeDef GPIO_InitStructure;



void SysTick_Handler(void)
{
  msTicks++;
}

void exactDelay (uint32_t dlyTicks)
{
  uint32_t curTicks;

  curTicks = msTicks;
  while ((msTicks - curTicks) < dlyTicks);
}

int main(void)
{
	 int Inverse=0;

	 SystemInit();
	 SysTick_Config(SystemCoreClock/ 1000); /* Configure SysTick to generate an interrupt every millisecond */

	 RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOA, ENABLE);

	 GPIO_InitStructure.GPIO_Mode = GPIO_Mode_Out_PP;
	 GPIO_InitStructure.GPIO_Pin = LED2PIN;
	 GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
	 GPIO_Init(GPIOA, &GPIO_InitStructure);


	// GPIO_WriteBit(GPIOA, LED2PIN, Bit_SET);

 	LCDInit();
 	LCDContrast(0x70);
    LCDStr(0, (unsigned char *)"*** OLIMEX ***", Inverse);
    LCDStr(1, (unsigned char *)"  STM32-P103  ", !Inverse);
    LCDStr(2, (unsigned char *)"  demo  for   ", Inverse);
    LCDStr(3, (unsigned char *)"  MOD-LCD3310 ", !Inverse);
    LCDStr(4, (unsigned char *)"  using SPI   ", Inverse);
    LCDStr(5, (unsigned char *)"  ATI   ", !Inverse);

    while(1)  //Loop.......
    {
    	exactDelay(1500);

    	    	GPIO_WriteBit(GPIOA, LED2PIN, HIGH);

    	exactDelay(100);

    	    	GPIO_WriteBit(GPIOA, LED2PIN, LOW);

    }
}
