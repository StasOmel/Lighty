#include "adc.h"


uint32_t Read_ADC1(uint8_t channel)
{

  ADC_RegularChannelConfig(ADC1, channel, 1, ADC_SampleTime_239Cycles5);

  ADC_SoftwareStartConvCmd(ADC1, ENABLE);

  while(ADC_GetFlagStatus(ADC1, ADC_FLAG_EOC) == RESET);

  return ADC_GetConversionValue(ADC1);
}
