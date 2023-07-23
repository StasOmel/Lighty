#ifndef __SPI_H
#define __SPI_H

#include "system.h"

#define CS_PIN      GPIO_Pin_4
#define CS_PORT     GPIOA

#define CS_LOW      GPIO_ResetBits( CS_PORT, CS_PIN)
#define CS_HIGH     GPIO_SetBits( CS_PORT, CS_PIN)

void SPI_write_16(uint16_t data);


#endif /* __SPI_H */
