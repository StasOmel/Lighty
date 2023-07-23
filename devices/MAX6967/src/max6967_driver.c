#include "max6967_driver.h"
#include "spi.h"

void MAX6967_write(uint8_t addr, uint8_t data);

void MAX6967_PowerUp(void)
{
  MAX6967_write(CONFIG_A, CONFIG_RUMP_UP_RUN_MODE);
}

void MAX6967_Shutdown(void)
{
  MAX6967_write(CONFIG_A, CONFIG_SHUTDOWN_MODE);
}

void MAX6967_OutPWMWrite(uint32_t OUTn, uint32_t value)
{
  if (value == OUT_VALUE_LED_OFF) value = OUT_PWM_VALUE_MAX;
  if (value == 0x00) value = OUT_VALUE_LED_OFF;
  if (value == OUT_LOGIC_LOW 	||
      value == OUT_LOGIC_HIGH ||
      value == OUT_CONSTANT_CURRENT) value = OUT_PWM_VALUE_MIN;

  MAX6967_write(OUTn, value);
}

void MAX6967_write(uint8_t addr, uint8_t data)
{
  SPI_write_16((addr << 8) + data);
}
