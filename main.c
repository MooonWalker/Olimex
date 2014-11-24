#include "stm32f10x_conf.h"

#define LED2PIN GPIO_Pin_1
#define HIGH Bit_SET
#define LOW Bit_RESET

volatile uint32_t msTicks;
volatile uint32_t nowTicks;


void SysTick_Handler(void)
{
  msTicks++;
}

void Delay (uint32_t dlyTicks)
{
  uint32_t curTicks;

  curTicks = msTicks;
  while ((msTicks - curTicks) < dlyTicks);
}

int main(void)
{
	 GPIO_InitTypeDef GPIO_InitStructure;

	 SystemInit();
	 SysTick_Config(SystemCoreClock/ 1000); /* Configure SysTick to generate an interrupt every millisecond */

	 RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOA, ENABLE);

	 GPIO_InitStructure.GPIO_Mode = GPIO_Mode_Out_PP;
	 GPIO_InitStructure.GPIO_Pin = LED2PIN;
	 GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
	 GPIO_Init(GPIOA, &GPIO_InitStructure);

	 GPIO_WriteBit(GPIOA, LED2PIN, Bit_SET);

    while(1)  //Loop.......
    {
    	Delay(1500);

    	    	GPIO_WriteBit(GPIOA, LED2PIN, HIGH);

    	    	Delay(100);

    	    	GPIO_WriteBit(GPIOA, LED2PIN, LOW);

    }
}
