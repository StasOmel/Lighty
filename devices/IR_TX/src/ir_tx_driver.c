#include "ir_tx_driver.h"

#define PulseON   TIM_CtrlPWMOutputs(TIM1, ENABLE); TIM_Cmd(TIM1, ENABLE)
#define PulseOFF  TIM_CtrlPWMOutputs(TIM1, DISABLE); TIM_Cmd(TIM1, DISABLE)

void Gen_NEC_Byte(unsigned char dat);


void IR_TX_Gen_NEC_Code(unsigned char adr, unsigned char cmd)
{
 PulseON;
 timerDelay_us(9000);
 PulseOFF;
 timerDelay_us(4500);
 Gen_NEC_Byte(adr);
 Gen_NEC_Byte(~adr);
 Gen_NEC_Byte(cmd);
 Gen_NEC_Byte(~cmd);
 PulseON;
 timerDelay_us(560);
 PulseOFF;
}


void Gen_NEC_Byte(unsigned char dat)
{
 char i;
 for(i=0;i<8;i++)
  {
  PulseON;
  timerDelay_us(560);
  PulseOFF;
  if(dat&0x80){timerDelay_us(1690);}
  else {timerDelay_us(560);}
  dat= dat << 1;
  }
}
