// ADC.c
#include "stm32f10x_lib.h"
#include "adc.h"


ADC_InitTypeDef   ADC_InitStructure;
GPIO_InitTypeDef GPIO_InitStructure;

void ADCInit(void) {

  // ADC init
  // ADC Deinit
  RCC_APB2PeriphClockCmd(RCC_APB2Periph_ADC1|RCC_APB2Periph_ADC2|RCC_APB2Periph_GPIOA, ENABLE);
  ADC_DeInit(ADC1);

  // RA1 - analog input
  GPIO_InitStructure.GPIO_Pin = GPIO_Pin_1;
  GPIO_InitStructure.GPIO_Speed = 0;
  GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AIN;
  GPIO_Init (GPIOA, &GPIO_InitStructure);

  // ADC Structure Initialization
  ADC_StructInit(&ADC_InitStructure);

  // Preinit
  ADC_InitStructure.ADC_Mode = ADC_Mode_Independent;
  ADC_InitStructure.ADC_ScanConvMode = DISABLE;
  ADC_InitStructure.ADC_ContinuousConvMode = DISABLE;
  ADC_InitStructure.ADC_ExternalTrigConv = ADC_ExternalTrigConv_None;
  ADC_InitStructure.ADC_DataAlign = ADC_DataAlign_Right;
  ADC_InitStructure.ADC_NbrOfChannel = 1;
  ADC_Init(ADC1, &ADC_InitStructure);

  // Enable the ADC
  ADC_Cmd(ADC1, ENABLE);
}

Int16U GetADCChanel(Int8U chanel) {

  // Configure chanel
  ADC_RegularChannelConfig(ADC1, chanel, 1, ADC_SampleTime_55Cycles5);

  // Start the conversion
  ADC_SoftwareStartConvCmd(ADC1, ENABLE);

  // Wait until conversion completion
  while(ADC_GetFlagStatus(ADC1, ADC_FLAG_EOC) == RESET);

  // Get the conversion value
  return ADC_GetConversionValue(ADC1);

}

