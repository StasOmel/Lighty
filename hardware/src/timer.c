#include "timer.h"

volatile uint32_t TimingDelay;
volatile uint32_t Cnt10ms;

void Delay(uint32_t nTime)
{
  TimingDelay = nTime;

  while(TimingDelay != 0);
}

void timerDelayDecrement(void)
{
  if (TimingDelay != 0x00)
    {
      TimingDelay--;
    }
  Cnt10ms++;

}

void timerInit(void)
{
  Cnt10ms=0;
}

uint32_t timer10msTest(void)
{
  if (Cnt10ms >= 10)
    {
      Cnt10ms = 0;
      return 1;
    }
  return 0;
}


void timerDelay_ms(unsigned int delay)
{
  TIM2->PSC = 24000;	//������������� ������������
  TIM2->ARR = (delay*2)-1;	 //������������� �������� ������������ �������, � ������ � �������� ��� ������� ������������ ������� ����������
  TIM2->EGR |= TIM_EGR_UG;	//���������� ������� ���������� ��� ������ ������ � �������� PSC � ARR
  TIM2->CR1 |= TIM_CR1_CEN|TIM_CR1_OPM;	//��������� ������ ������� ���� CEN � ������������� ����� ������ ������� ���������� ���� OPM
  while ((TIM2->CR1&TIM_CR1_CEN)!=0);
}

void timerDelay_us(unsigned int delay)
{
  TIM2->PSC = 23;	//������������� ������������
  TIM2->ARR = (delay*2)-1; //������������� �������� ������������ �������, � ������ � �������� ��� ������� ������������ ������� ����������
  TIM2->EGR |= TIM_EGR_UG;	//���������� ������� ���������� ��� ������ ������ � �������� PSC � ARR
  TIM2->CR1 |= TIM_CR1_CEN|TIM_CR1_OPM;	//��������� ������ ������� ���� CEN � ������������� ����� ������ ������� ���������� ���� OPM
  while ((TIM2->CR1&TIM_CR1_CEN)!=0);
}
