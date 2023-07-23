#include "spi.h"


void SPI_write_16(uint16_t data)
{

  SPI_Cmd(SPI1, ENABLE);
  CS_LOW;

  timerDelay_us(200);


  while (SPI_I2S_GetFlagStatus(SPI1, SPI_I2S_FLAG_TXE) == RESET);
  SPI_I2S_SendData (SPI1, data);
  while (SPI_I2S_GetFlagStatus(SPI1, SPI_I2S_FLAG_TXE) == RESET);

  timerDelay_us(200);

  CS_HIGH;
  SPI_Cmd(SPI1, DISABLE);
}

