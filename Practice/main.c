#include "stm32f10x.h"
#include "Olimexino-STM32.h"
// PA1 sárga, PA5 zõd ledek
// User button: PC9

GPIO_InitTypeDef GPIO_InitStructure;
EXTI_InitTypeDef EXTI_InitStructure;
NVIC_InitTypeDef NVIC_InitStructure;

volatile uint32_t msTicks;
volatile uint32_t nowTicks;
volatile int startstop=0;


int isButtonPressed(void);

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

void EXTI9_5_IRQHandler(void)
{
	if (EXTI_GetITStatus(EXTI_Line9) != RESET)
	{
		startstop=1;
		//isButtonPressed();
	}
	EXTI_ClearITPendingBit(EXTI_Line9);  /* Clear the EXTI line 9 pending bit */
}

void setup()
{
		 SystemInit();
	    SysTick_Config(SystemCoreClock/ 1000); /* Configure SysTick to generate an interrupt every millisecond */

	   	RCC->APB2ENR |= RCC_APB2ENR_IOPAEN; /* enable APB2 for GPIOA */
	   	RCC->APB2ENR |= RCC_APB2ENR_IOPCEN; /* enable APB2 for GPIOC */
	   	RCC->APB2ENR |= RCC_APB2ENR_AFIOEN; /* enable APB2 for AFIO */

	   	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_Out_PP;
	 	   GPIO_InitStructure.GPIO_Pin =  GPIO_Pin_1 | GPIO_Pin_5;
	   	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_2MHz;
	   	GPIO_Init(GPIOA, &GPIO_InitStructure);

	   	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IN_FLOATING;
	   	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_9;
	   	GPIO_InitStructure.GPIO_Speed =GPIO_Speed_50MHz;
	   	GPIO_Init(GPIOC, &GPIO_InitStructure);

	   	//
	   	    // Configure one bit for preemption priority
	   	    //
	   	NVIC_PriorityGroupConfig(NVIC_PriorityGroup_1);
	   	NVIC_InitStructure.NVIC_IRQChannel = EXTI9_5_IRQn;
	      NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 0x01;
	   	NVIC_InitStructure.NVIC_IRQChannelSubPriority =0x01;
	   	NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
	   	NVIC_Init(&NVIC_InitStructure);

	   	GPIO_EXTILineConfig(GPIO_PortSourceGPIOC, GPIO_PinSource9);

	   	EXTI_InitStructure.EXTI_Line = EXTI_Line9;
	   	EXTI_InitStructure.EXTI_Mode = EXTI_Mode_Interrupt;
	   	EXTI_InitStructure.EXTI_Trigger = EXTI_Trigger_Rising;
	   	EXTI_InitStructure.EXTI_LineCmd = ENABLE;
	   	EXTI_Init(&EXTI_InitStructure);
}

int main(void)
{
  setup();

  while(1)
  {
	 if (startstop==1)
	 {
	   GPIO_WriteBit(GPIOA, GPIO_Pin_5, Bit_RESET);
	   GPIO_WriteBit(GPIOA, GPIO_Pin_1, Bit_SET);
   	exactDelay(1000);

   	GPIO_WriteBit(GPIOA, GPIO_Pin_1, Bit_RESET);
   	GPIO_WriteBit(GPIOA, GPIO_Pin_5, Bit_SET);
   	exactDelay(1000);
	 }
  }

  return 0;
}

int isButtonPressed()
{
	int result =0;
	if (GPIO_ReadInputDataBit(GPIOC, GPIO_Pin_9)>0)
	{
		exactDelay(2);
		if (GPIO_ReadInputDataBit(GPIOC, GPIO_Pin_9)>0)
			{
				result=1;
				GPIO_WriteBit(GPIOA, GPIO_Pin_1, Bit_SET);
				GPIO_WriteBit(GPIOA, GPIO_Pin_5, Bit_SET);
				while(GPIO_ReadInputDataBit(GPIOC, GPIO_Pin_9)){}  // only return if the button released
			}
	}
	return result;
}

