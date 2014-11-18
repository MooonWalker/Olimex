#include "stm32f10x.h"
#include "Olimexino-STM32.h"

GPIO_InitTypeDef GPIO_InitStructure;

int main(void)
{

    while(1)
    {
    	SystemInit();
    	SysTick_Config(SystemCoreClock/ 1000); /* Configure SysTick to generate an interrupt every millisecond */


    }
}
