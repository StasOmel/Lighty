#include "outputs.h"
#include "MAX6967_driver.h"
#include "ir_tx_driver.h"

ColorRGB_t RGB;
ColorHSV_t hsv, hsv_old, Led1_hsv, Led2_hsv, Led3_hsv;
ColorRGB_t rgb, Led1_rgb, Led2_rgb, Led3_rgb;
ColorRGB_t rgb_old;

void MAX6967_Init(void);
void MAX6967_DeInit(void);

void outputsInit(void)
{
  MAX6967_Init();

  outputsRGBTest(10);
  outputsRGBLedSet(RGB_ALL, 0, 0, 0);
}

void outputsDeInit(void)
{
  MAX6967_DeInit();
}

void outputsRGBLedSet(RGBLedn_t Ledn, uint32_t R, uint32_t G, uint32_t B)
{
  switch (Ledn)
  {
  case RGB_LED1:
    MAX6967_OutPWMWrite(LED1_RED_CHANNEL, R);
    MAX6967_OutPWMWrite(LED1_GREEN_CHANNEL, G);
    MAX6967_OutPWMWrite(LED1_BLUE_CHANNEL, B);
    break;

  case RGB_LED2:
    MAX6967_OutPWMWrite(LED2_RED_CHANNEL, R);
    MAX6967_OutPWMWrite(LED2_GREEN_CHANNEL, G);
    MAX6967_OutPWMWrite(LED2_BLUE_CHANNEL, B);
    break;

  case RGB_LED3:
    MAX6967_OutPWMWrite(LED3_RED_CHANNEL, R);
    MAX6967_OutPWMWrite(LED3_GREEN_CHANNEL, G);
    MAX6967_OutPWMWrite(LED3_BLUE_CHANNEL, B);
    break;

  case RGB_ALL:
    MAX6967_OutPWMWrite(LED1_RED_CHANNEL, R);
    MAX6967_OutPWMWrite(LED1_GREEN_CHANNEL, G);
    MAX6967_OutPWMWrite(LED1_BLUE_CHANNEL, B);

    MAX6967_OutPWMWrite(LED2_RED_CHANNEL, R);
    MAX6967_OutPWMWrite(LED2_GREEN_CHANNEL, G);
    MAX6967_OutPWMWrite(LED2_BLUE_CHANNEL, B);

    MAX6967_OutPWMWrite(LED3_RED_CHANNEL, R);
    MAX6967_OutPWMWrite(LED3_GREEN_CHANNEL, G);
    MAX6967_OutPWMWrite(LED3_BLUE_CHANNEL, B);
    break;

  default:
    break;
  }
}


void MAX6967_Init(void)
{
  MAX6967_Shutdown();
  timerDelay_ms(100);
  MAX6967_PowerUp();
  outputsRGBLedSet(RGB_ALL, 0, 0, 0);
}

void MAX6967_DeInit(void)
{
  MAX6967_Shutdown();
}

void outputsFadeIn(RGBLedn_t Led1, RGBLedn_t Led2, RGBLedn_t Led3, uint32_t time)
{

  rgb_old.r = rgb.r;
  rgb_old.g = rgb.g;
  rgb_old.b = rgb.b;

  while (1)
    {
      rgb.r += 10;
      rgb.g += 10;
      rgb.b += 10;

      if (rgb.r >= 255) rgb.r = 255;
      if (rgb.g >= 255) rgb.g = 255;
      if (rgb.b >= 255) rgb.b = 255;

      outputsRGBLedSet(Led1, rgb.r, rgb.g, rgb.b);
      outputsRGBLedSet(Led2, rgb.r, rgb.g, rgb.b);
      outputsRGBLedSet(Led3, rgb.r, rgb.g, rgb.b);

      timerDelay_ms(time);

      if (rgb.r == 255 && rgb.g == 255 && rgb.b == 255) break;
    }
}

void outputsFadeOut(RGBLedn_t Led1, RGBLedn_t Led2, RGBLedn_t Led3, uint32_t time)
{
  while (1)
    {
      rgb.r -= 10;
      rgb.g -= 10;
      rgb.b -= 10;

      if (rgb.r < rgb_old.r) rgb.r = rgb_old.r;
      if (rgb.g < rgb_old.g) rgb.g = rgb_old.g;
      if (rgb.b < rgb_old.b) rgb.b = rgb_old.b;

      outputsRGBLedSet(Led1, rgb.r, rgb.g, rgb.b);
      outputsRGBLedSet(Led2, rgb.r, rgb.g, rgb.b);
      outputsRGBLedSet(Led3, rgb.r, rgb.g, rgb.b);

      timerDelay_ms(time);

      if (rgb.r == rgb_old.r && rgb.g == rgb_old.g && rgb.b == rgb_old.b) break;
    }
}

void outputsFadeInOut(void)
{

  rgb_old.r = rgb.r;
  rgb_old.g = rgb.g;
  rgb_old.b = rgb.b;

  while (1)
    {
      rgb.r += 10;
      rgb.g += 10;
      rgb.b += 10;

      if (rgb.r >= 255) rgb.r = 255;
      if (rgb.g >= 255) rgb.g = 255;
      if (rgb.b >= 255) rgb.b = 255;

      outputsRGBLedSet(RGB_ALL, rgb.r, rgb.g, rgb.b);

      timerDelay_ms(3);

      if (rgb.r == 255 && rgb.g == 255 && rgb.b == 255) break;
    }

  while (1)
    {
      rgb.r -= 10;
      rgb.g -= 10;
      rgb.b -= 10;

      if (rgb.r < rgb_old.r) rgb.r = rgb_old.r;
      if (rgb.g < rgb_old.g) rgb.g = rgb_old.g;
      if (rgb.b < rgb_old.b) rgb.b = rgb_old.b;

      outputsRGBLedSet(RGB_ALL, rgb.r, rgb.g, rgb.b);

      timerDelay_ms(3);

      if (rgb.r == rgb_old.r && rgb.g == rgb_old.g && rgb.b == rgb_old.b) break;
    }
}

void outputsFadeInOutQueue(void)
{

  rgb_old.r = rgb.r;
  rgb_old.g = rgb.g;
  rgb_old.b = rgb.b;

  while (1)
    {
      rgb.r += 10;


      if (rgb.r >= 255) rgb.r = 255;

      outputsRGBLedSet(RGB_ALL, rgb.r, rgb.g, rgb.b);

      timerDelay_ms(1);

      if (rgb.r == 255) break;
    }

  timerDelay_ms(1);

  while (1)
    {
      rgb.g += 10;

      if (rgb.g >= 255) rgb.g = 255;

      outputsRGBLedSet(RGB_ALL, rgb.r, rgb.g, rgb.b);

      timerDelay_ms(1);

      if (rgb.g == 255) break;
    }

  timerDelay_ms(1);

  while (1)
    {
      rgb.b += 10;

      if (rgb.b >= 255) rgb.b = 255;

      outputsRGBLedSet(RGB_ALL, rgb.r, rgb.g, rgb.b);

      timerDelay_ms(1);

      if (rgb.b == 255) break;
    }

  timerDelay_ms(3);

  while (1)
    {
      rgb.b -= 10;

      if (rgb.b < rgb_old.b) rgb.b = rgb_old.b;

      outputsRGBLedSet(RGB_ALL, rgb.r, rgb.g, rgb.b);

      timerDelay_ms(1);

      if (rgb.b == rgb_old.b) break;
    }

  timerDelay_ms(1);

  while (1)
    {
      rgb.g -= 10;

      if (rgb.g < rgb_old.g) rgb.g = rgb_old.g;

      outputsRGBLedSet(RGB_ALL, rgb.r, rgb.g, rgb.b);

      timerDelay_ms(1);

      if (rgb.g == rgb_old.g) break;
    }

  timerDelay_ms(1);

  while (1)
    {

      rgb.r -= 10;


      if (rgb.r < rgb_old.r) rgb.r = rgb_old.r;

      outputsRGBLedSet(RGB_ALL, rgb.r, rgb.g, rgb.b);

      timerDelay_ms(1);

      if (rgb.r == rgb_old.r) break;
    }

}


void outputsRGBTest(uint32_t delay)
{
  outputsRGBLedSet(RGB_LED1, 20, 0, 0);
  timerDelay_ms(delay);
  outputsRGBLedSet(RGB_LED1, 0, 20, 0);
  timerDelay_ms(delay);
  outputsRGBLedSet(RGB_LED1, 0, 0, 20);
  timerDelay_ms(delay);

  outputsRGBLedSet(RGB_LED1, 0, 0, 0);
  timerDelay_ms(delay/10);

  outputsRGBLedSet(RGB_LED2, 20, 0, 0);
  timerDelay_ms(delay);
  outputsRGBLedSet(RGB_LED2, 0, 20, 0);
  timerDelay_ms(delay);
  outputsRGBLedSet(RGB_LED2, 0, 0, 20);
  timerDelay_ms(delay);

  outputsRGBLedSet(RGB_LED2, 0, 0, 0);
  timerDelay_ms(delay/10);


  outputsRGBLedSet(RGB_LED3, 20, 0, 0);
  timerDelay_ms(delay);
  outputsRGBLedSet(RGB_LED3, 0, 20, 0);
  timerDelay_ms(delay);
  outputsRGBLedSet(RGB_LED3, 0, 0, 20);
  timerDelay_ms(delay);

  outputsRGBLedSet(RGB_LED3, 0, 0, 0);
  timerDelay_ms(delay/10);

}

void outputsIRLedCommand(uint32_t adr, uint32_t cmd)
{
  IR_TX_Gen_NEC_Code(adr, cmd);
}
