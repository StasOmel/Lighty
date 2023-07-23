#include "modes.h"

#include <stdio.h>
#include <stdlib.h>
#include "semihosting.h"

#include "table_fft.h"
#include "stm32_dsp.h"

DeviceMode_t DeviceMode;
DeviceSubMode_t DeviceSubMode;

DeviceMode_t DeviceMode_old;
DeviceSubMode_t DeviceSubMode_old;

POSITION_t position;

volatile uint32_t selectedMode = 0, selectionDone = 0;

extern ColorHSV_t hsv, hsv_old, Led1_hsv, Led2_hsv, Led3_hsv;

ColorHSV_t Led1_target_hsv, Led2_target_hsv, Led3_target_hsv;


extern ColorRGB_t rgb, Led1_rgb, Led2_rgb, Led3_rgb, rgb_old;

extern SysSensorsData_TypeDef SysSensorsData;

static uint32_t LedFreqBindArray[9], SpectrumAverageCount = 4;

static uint32_t hue_cnt = 0, value_cnt = 0, hue_change_timer = 5, value_change_timer = 7;

extern uint8_t Receive_Buffer[64];
extern uint8_t Send_Buffer[64];

uint32_t wasRTCWakeUp;

void modesColorSelector(void);
void modesColorMusic10ms(void);
void modesColorMusicSuperLoop(void);
void modesRandomColor(void);
void modesRemote(void);
void modes5(void);
void modes6(void);
void modesSelector(void);
void modesIdle(void);

void modesInit(void)
{
  DeviceMode = selectedMode = MODE_COLOR_SELECTOR;
  DeviceSubMode = 0;
  position = POSITION_UP;

  DeviceMode_old = MODE_REMOTE;

  LedFreqBindArray[0] = 0b00000000000000000000000000000010;	// Led1 R
  LedFreqBindArray[1] = 0b00000000000000000000000010000000;	// Led1 G
  LedFreqBindArray[2] = 0b00000000000000000100000000000000;	// Led1 B

  LedFreqBindArray[3] = 0b00000000000000000000000100100000;	// Led2 R
  LedFreqBindArray[4] = 0b00000000000000000000000000000100;	// Led2 G
  LedFreqBindArray[5] = 0b00000000000000000000100000000000;	// Led2 B

  LedFreqBindArray[6] = 0b00000000000000000000000001000000;	// Led3 R
  LedFreqBindArray[7] = 0b00000000000000000000010000000000;	// Led3 G
  LedFreqBindArray[8] = 0b00000000000000000000000000001000;	// Led3 B


  hsv.h = 0;
  hsv.s = 100;
  hsv.v = 0;

  Led1_hsv.h = 0;
  Led1_hsv.s = 100;
  Led1_hsv.v = 100;

  Led2_hsv.h = 120;
  Led2_hsv.s = 100;
  Led2_hsv.v = 100;

  Led3_hsv.h = 240;
  Led3_hsv.s = 100;
  Led3_hsv.v = 100;


  Led1_target_hsv.h = 31;
  Led1_target_hsv.s = 100;
  Led1_target_hsv.v = 100;

  Led2_target_hsv.h = 194;
  Led2_target_hsv.s = 100;
  Led2_target_hsv.v = 100;

  Led3_target_hsv.h = 93;
  Led3_target_hsv.s = 100;
  Led3_target_hsv.v = 100;

  wasRTCWakeUp=0;
}

void modesManage(void)
{
  if (sensorsCheckOverturnChange(&position))
    {
      if (position == POSITION_DOWN)
        {
          outputsRGBLedSet(RGB_ALL, 10, 10, 10);
          modesSelectionDoneClear();
          modesModeSet(MODE_SELECTOR);
          hsv.v = 100;
        }
      else
        {
          outputsRGBLedSet(RGB_ALL, 0, 0, 0);
          modesModeSet(modesGetSelectedMode());
        }
    }

  if (usbCheckModeChangeCommand())
    {
      if (modesModeSet(Receive_Buffer[1]))
        {

          //			modesGetModeColor(DeviceMode);
          //			hsv.s=100;
          //			hsv.v=30;
          //			toolsHSVtoRGB(&hsv, &rgb);
          //			outputsRGBLedSet(RGB_ALL, rgb.r, rgb.g, rgb.b);
          //			//TODO: FIX!
          //			timerDelay_ms(30);
          //			outputsRGBLedSet(RGB_ALL, 0, 0, 0);
        }

      modesSubModeSet(Receive_Buffer[2]);
    }


  if (SysSensorsData.Voltage <= VBAT_DISCHARGE_HIGH_THRESHOLD && modesModeGet() != MODE_IDLE &&
      modesSubModeGet() != SUB_MODE_IDLE_BATLOW_INDICATING &&
      usbCheckPowerStatus() == USB_UNPOWERED)
    {
      DeviceMode_old = modesModeGet();
      DeviceSubMode_old = modesSubModeGet();
      modesModeSet(MODE_IDLE);
      modesSubModeSet(SUB_MODE_IDLE_BATLOW_INDICATING);
    }

  if (usbCheckPowerStatus() == USB_POWERED && modesModeGet() == MODE_IDLE &&
      modesSubModeGet() == SUB_MODE_IDLE_BATLOW_INDICATING)
    {
      modesModeSet(DeviceMode_old);
      modesSubModeSet(DeviceSubMode_old);
    }


}


void modes10msManage(void)
{
  switch (DeviceMode)
  {
  case MODE_COLOR_SELECTOR:
    modesColorSelector();
    break;

  case MODE_COLOR_MUSIC:
    modesColorMusic10ms();
    break;

  case MODE_RANDOM_COLOR:
    modesRandomColor();
    break;

  case MODE_REMOTE:
    modesRemote();
    break;

  case MODE_5:
    //modes5();
    break;

  case MODE_6:
    //modes6();
    break;

  case MODE_IDLE:
    modesIdle();
    break;

  case MODE_SELECTOR:
    modesSelector();
    break;

  default:
    break;
  }
}

void modesSuperLoop(void)
{
  switch (DeviceMode)
  {
  case MODE_COLOR_SELECTOR:
    //modesColorSelector();
    break;

  case MODE_COLOR_MUSIC:
    modesColorMusicSuperLoop();
    break;

  case MODE_RANDOM_COLOR:
    //modes3();
    break;

  case MODE_REMOTE:
    //modes4();
    break;

  case MODE_5:
    //modes5();
    break;

  case MODE_6:
    //modes6();
    break;

  case MODE_IDLE:
    //outputsRGBLedSet(RGB_ALL, 0, 0, 0);
    break;

  case MODE_SELECTOR:
    //modesSelector();
    break;

  default:
    break;
  }
}

uint8_t modesModeSet(DeviceMode_t newMode)
{
  if (DeviceMode != newMode) 	{DeviceMode = newMode; return 1;}
  else {return 0;}
}

uint8_t modesSubModeSet(DeviceSubMode_t newSubMode)
{
  if (DeviceSubMode != newSubMode) 	{DeviceSubMode = newSubMode; return 1;}
  else {return 0;}
}

DeviceMode_t modesModeGet(void)
{
  return DeviceMode;
}

DeviceSubMode_t modesSubModeGet(void)
{
  return DeviceSubMode;
}

void modesColorSelector(void)
{
  static uint32_t MaxBrightnessFlashLatch, SelectTime, SelectedSaturationFlashLatch, wasLight = 0;
  static int32_t angle, angle_old;
  uint32_t temp=0;
  //SysSensorsData.CoverPercents = 0;


  angle = SysSensorsData.Inclination;

  if (angle > 77)
    {
      hsv.h = SysSensorsData.Heading;
      if (SelectedSaturationFlashLatch || SelectTime != 0)
        {
          SelectedSaturationFlashLatch=0;
          //angle_old=0;
          SelectTime=0;
        }
    }


  if (SysSensorsData.CoverPercents < 27 && angle <= 77)
    {
      if (SelectedSaturationFlashLatch != 1)
        {
          temp = angle;
          temp = temp / 10;
          temp = 10-temp;
          temp = temp * 6;
          hsv.s = ((angle) + 45) - temp;
          if (hsv.s < 0) hsv.s = 0;
          if (hsv.s >= 100) hsv.s = 100;
        }

      if (angle >= angle_old)
        {
          if ((angle - angle_old) < 10) SelectTime++;
          else SelectTime=0;
        }
      else
        {
          if ((angle_old - angle) < 10) SelectTime++;
          else SelectTime=0;
        }

      angle_old = angle;

      if (SelectTime > 200 && SelectedSaturationFlashLatch != 1)
        {
          SelectedSaturationFlashLatch = 1;
          //TODO: fix
          outputsFadeInOutQueue();
        }
    }
  else if (SysSensorsData.CoverPercents > 33 && SysSensorsData.CoverPercents < 70)
    {
      hsv.v ++;

      if (hsv.v > 10) wasLight = 1;

      if (hsv.v > 100)
        {
          hsv.v = 100;
          if (!MaxBrightnessFlashLatch)
            {
              //TODO: fix
              outputsFadeInOutQueue();

              //timerDelay_ms(5);
              MaxBrightnessFlashLatch = 1;
            }
        }
    }
  else if (SysSensorsData.CoverPercents > 75)
    {
      hsv.v --;
      if (hsv.v <= 0) hsv.v = 0;
      if (hsv.v < 90 && MaxBrightnessFlashLatch) MaxBrightnessFlashLatch = 0;
    }


  if (DeviceSubMode == SUB_MODE_COLOR_SELECTOR_PC_DEF)
    {
      if (usbCheckNewDataPacket())
        {
          outputsRGBLedSet(RGB_LED1, Receive_Buffer[1], Receive_Buffer[2], Receive_Buffer[3]);
          outputsRGBLedSet(RGB_LED2, Receive_Buffer[4], Receive_Buffer[5], Receive_Buffer[6]);
          outputsRGBLedSet(RGB_LED3, Receive_Buffer[7], Receive_Buffer[8], Receive_Buffer[9]);

          Led1_rgb.r = Receive_Buffer[1];
          Led1_rgb.g = Receive_Buffer[2];
          Led1_rgb.b = Receive_Buffer[3];

          Led2_rgb.r = Receive_Buffer[4];
          Led2_rgb.g = Receive_Buffer[5];
          Led2_rgb.b = Receive_Buffer[6];

          Led3_rgb.r = Receive_Buffer[7];
          Led3_rgb.g = Receive_Buffer[8];
          Led3_rgb.b = Receive_Buffer[9];
        }
    }
  else
    {
      if (hsv_old.h != hsv.h || hsv_old.s != hsv.s || hsv_old.v != hsv.v)
        {
          toolsHSVtoRGB(&hsv, &rgb);

          if (hsv.v < 5) {
              rgb.r = 0;
              rgb.g = 0;
              rgb.b = 0;
          }

          Led1_rgb.r = rgb.r;
          Led1_rgb.g = rgb.g;
          Led1_rgb.b = rgb.b;

          Led2_rgb.r = rgb.r;
          Led2_rgb.g = rgb.g;
          Led2_rgb.b = rgb.b;

          Led3_rgb.r = rgb.r;
          Led3_rgb.g = rgb.g;
          Led3_rgb.b = rgb.b;

          outputsRGBLedSet(RGB_ALL, rgb.r, rgb.g, rgb.b);

          if (hsv.v == 0 && wasLight) {
              timerDelay_ms(1000);
              wasLight = 0;
          }
        }

      hsv_old.h = hsv.h;
      hsv_old.s = hsv.s;
      hsv_old.v = hsv.v;
    }

}



void modesSelector(void)
{
  static uint32_t SelectTime=0, mode=0, old_mode=0;

  if (SysSensorsData.Inclination < SELECTOR_MODE_DOWN_INCLINATION_THRESHOLD)
    {
      mode = SysSensorsData.Heading / 60;
      hsv.s = 100;
      modesGetModeColor(mode);

      if (mode == old_mode && !selectionDone)
        {
          SelectTime++;

          if (SelectTime % 2 == 0) hsv.v --;

          if (hsv.v <= 0) hsv.v = 0;
        }
      else
        {
          SelectTime = 0;
          hsv.v = 100;
        }

      old_mode = mode;

      if (SelectTime > 200)
        {
          selectedMode = mode;
          selectionDone = 1;
          outputsRGBLedSet(RGB_ALL, 0, 0, 0);
        }
      else
        {
          if (!selectionDone)
            {
              toolsHSVtoRGB(&hsv, &rgb);
              outputsRGBLedSet(RGB_ALL, rgb.r, rgb.g, rgb.b);
            }
        }
    }
  else
    {
      if (!selectionDone) outputsRGBLedSet(RGB_ALL, 10, 10, 10);
      SelectTime = 0;
      old_mode = 0;
    }

}

void modesSelectionDoneClear(void)
{
  selectionDone = 0;
}

DeviceMode_t modesGetSelectedMode(void)
{
  switch (selectedMode)
  {
  case MODE_COLOR_SELECTOR:
    MAX9814_SHDN_ON;
    ADC_Off();
    break;

  case MODE_COLOR_MUSIC:
    MAX9814_SHDN_OFF;
    ADC_On();
    break;

  case MODE_RANDOM_COLOR:
    MAX9814_SHDN_OFF;
    ADC_On();
    break;

  case MODE_REMOTE:
    MAX9814_SHDN_ON;
    ADC_Cmd(ADC1, DISABLE);
    hsv.h = 0;
    hsv.s = 100;
    hsv.v = 0;
    break;

  case MODE_5: break;
  case MODE_6: break;
  default:
    break;
  }
  return selectedMode;
}


void modesGetModeColor(DeviceMode_t mode)
{
  switch (mode)
  {
  case MODE_COLOR_SELECTOR: hsv.h = 0; break;
  case MODE_COLOR_MUSIC: hsv.h = 60; break;
  case MODE_RANDOM_COLOR: hsv.h = 120; break;
  case MODE_REMOTE: hsv.h = 180; break;
  case MODE_5: hsv.h = 240; break;
  case MODE_6: hsv.h = 300; break;
  default:
    break;
  }
}

#define PI2  6.28318530717959

void modesColorMusic10ms(void)
{
  uint32_t i, temp = 0, value = 0, j = 1;
  if (usbCheckNewDataPacket())
    {
      for (i = 0; i < 9; i++)
        {
          value = 0;

          temp = Receive_Buffer[j++];
          value = temp << 24;

          temp = Receive_Buffer[j++];
          value += temp << 16;

          temp = Receive_Buffer[j++];
          value += temp << 8;

          temp = Receive_Buffer[j++];
          value += temp;

          LedFreqBindArray[i] = value;
        }

      SpectrumAverageCount = Receive_Buffer[37];
    }

}


void modesColorMusicSuperLoop(void)
{
  //	DEBUG_HIGH;
  static uint32_t i, j;
  static int32_t RawADC, CmpxBufIn[FFT_SIZE], CmpxBufOut[FFT_SIZE], MagBuf[FFT_SIZE];
  static uint8_t LEDs_Values[9];
  static uint32_t ReadySpectrum[FFT_SIZE / 2], AverageSpectrum[FFT_SIZE / 2][5];

  for (i=0; i< FFT_SIZE; i++)
    {
      RawADC = sensorsGetAudioSample();
      /* to complex number with real and imaginary part (imaginary zero) */
      CmpxBufIn[i] = (((int16_t)RawADC) - 2048) << 4 ;
    }

  //Radix-4 complex FFT for STM32
  cr4_fft_64_stm32(CmpxBufOut, CmpxBufIn, FFT_SIZE);

  int32_t lX,lY;

  for (i=0; i < FFT_SIZE; i++)
    {
      lX= (CmpxBufOut[i]<<16)>>16; // sine_cosine --> cos
      lY= (CmpxBufOut[i] >> 16);   //sine_cosine --> sin
      MagBuf[i] = sqrt(lX*lX + lY*lY);
    }

  // sliding: prepare for averaging
  for (i=0; i < SpectrumAverageCount; i++)
    {
      for (j=0; j < FFT_SIZE / 2; j++)
        {
          AverageSpectrum[j][i] = AverageSpectrum[j][i+1];
        }
    }
  for (j=0; j < FFT_SIZE / 2; j++)
    {
      AverageSpectrum[j][SpectrumAverageCount-1] = MagBuf[j]>>2; // spectrum is always divided by two
    }

  // averaging
  for (i=0; i < 32; i++)
    {
      ReadySpectrum[i] = 0;
      for (j=0; j < SpectrumAverageCount; j++)
        {
          ReadySpectrum[i] += AverageSpectrum[i][j];
        }
      ReadySpectrum[i] = ReadySpectrum[i] / SpectrumAverageCount;
    }

  // for USB
  for (i=0; i < FFT_SIZE / 2; i++)
    {
      Send_Buffer[i+32] = ReadySpectrum[i];
    }

  // assign spectrum to leds according to LedFreqBindArray
  for (i = 0; i < 9; i++)
    {
      LEDs_Values[i] = 0;
      for (j = 0; j < 32; j++)
        {
          if (LedFreqBindArray[i] & (1 << j)) LEDs_Values[i] += ReadySpectrum[j];
        }
      if (LEDs_Values[i] < 5) LEDs_Values[i]=0;
    }

  Led1_rgb.r = LEDs_Values[0];
  Led1_rgb.g = LEDs_Values[1];
  Led1_rgb.b = LEDs_Values[2];

  Led2_rgb.r = LEDs_Values[3];
  Led2_rgb.g = LEDs_Values[4];
  Led2_rgb.b = LEDs_Values[5];

  Led3_rgb.r = LEDs_Values[6];
  Led3_rgb.g = LEDs_Values[7];
  Led3_rgb.b = LEDs_Values[8];


  outputsRGBLedSet(RGB_LED1, LEDs_Values[0], LEDs_Values[1], LEDs_Values[2]);
  outputsRGBLedSet(RGB_LED2, LEDs_Values[3], LEDs_Values[4], LEDs_Values[5]);
  outputsRGBLedSet(RGB_LED3, LEDs_Values[6], LEDs_Values[7], LEDs_Values[8]);
  //      DEBUG_LOW;

}


void modesIdle(void)
{
  static uint32_t BatLowIndicatorLedMaxLatch, BatLowIndicatorLedsMinLatch;

  if (modesSubModeGet() == SUB_MODE_IDLE_BATLOW_INDICATING)
    {

      /* Wait till RTC Second event occurs */
      RTC_ClearFlag(RTC_FLAG_SEC);
      while(RTC_GetFlagStatus(RTC_FLAG_SEC) == RESET);

      /* Alarm in 3 second */
      RTC_SetAlarm(RTC_GetCounter() + 300);
      /* Wait until last write operation on RTC registers has finished */
      RTC_WaitForLastTask();

      ResetInt1Latch();

      EXTI_ClearITPendingBit(LSM303_INT1_EXTI_LINE);
      EXTI_ClearITPendingBit(EXTI_Line17);

      /* Request to enter STOP mode with regulator in low power mode*/
      PWR_EnterSTOPMode(PWR_Regulator_LowPower, PWR_STOPEntry_WFI);

      //	    /* Configures system clock after wake-up from STOP: enable HSE, PLL and select
      //	       PLL as system clock source (HSE and PLL are disabled in STOP mode) */
      SYSCLKConfig_STOP();

      outputsRGBLedSet(RGB_LED2, 100, 0, 0);
      timerDelay_ms(3000);
      outputsRGBLedSet(RGB_LED2, 100, 0, 0);

      /*
		if ((rgb.r != 100 || rgb.g != 100 || rgb.b != 100) && !BatLowIndicatorLedMaxLatch)
		{
		rgb.r++;
		if (rgb.r > 100) rgb.r = 100;
		rgb.g++;
		if (rgb.g > 100) rgb.g = 100;
		rgb.b++;
		if (rgb.b > 100) rgb.b = 100;

		if (rgb.r == 100 && rgb.g == 100 && rgb.b == 100) {BatLowIndicatorLedMaxLatch = 1; BatLowIndicatorLedsMinLatch = 0;}
		}
		else if ((rgb.r != 0 || rgb.g != 0 || rgb.b != 0) && !BatLowIndicatorLedsMinLatch)
		{
		rgb.r--;
		if (rgb.r < 0) rgb.r = 0;
		rgb.g--;
		if (rgb.g < 0) rgb.g = 0;
		rgb.b--;
		if (rgb.b < 0) rgb.b = 0;

		if (rgb.r == 0 && rgb.g == 0 && rgb.b == 0) {BatLowIndicatorLedMaxLatch = 0; BatLowIndicatorLedsMinLatch = 1;}
		}
       */




    }
}

void modesRandomColor(void)
{

  static uint32_t Led1_hue_direction, Led2_hue_direction, Led3_hue_direction;
  //Led1_value_direction, Led2_value_direction, Led3_value_direction;

  if (hue_cnt >= hue_change_timer)
    {
      hue_cnt = 0;

      hue_change_timer = SysSensorsData.Heading / 30;
      if ((rand() % 10) % 2 == 0 ) hue_change_timer++;
      else hue_change_timer--;

      if (hue_change_timer > 12) hue_change_timer = 12;
      if (hue_change_timer == 0) hue_change_timer = 1;

      if (Led1_hsv.h == Led1_target_hsv.h)
        {
          srand(sensorsGetAudioSample());
          Led1_target_hsv.h = rand() % 360;

          if (sensorsGetAudioSample() % 2 == 0) Led1_hue_direction = 1;
          else Led1_hue_direction = 0;
        }
      if (Led2_hsv.h == Led2_target_hsv.h)
        {
          srand(sensorsGetAudioSample());
          Led2_target_hsv.h = rand() % 360;

          if (sensorsGetAudioSample() % 2 == 0) Led2_hue_direction = 1;
          else Led2_hue_direction = 0;
        }
      if (Led3_hsv.h == Led3_target_hsv.h)
        {
          srand(sensorsGetAudioSample());
          Led3_target_hsv.h = rand() % 360;

          if (sensorsGetAudioSample() % 2 == 0) Led3_hue_direction = 1;
          else Led3_hue_direction = 0;
        }

      if (Led1_hue_direction) Led1_hsv.h++;
      else Led1_hsv.h--;
      if (Led2_hue_direction) Led2_hsv.h++;
      else Led2_hsv.h--;
      if (Led3_hue_direction) Led3_hsv.h++;
      else Led3_hsv.h--;


      if (Led1_hsv.h == 361 ) Led1_hsv.h = 0;
      else if (Led1_hsv.h == -1) Led1_hsv.h = 360;
      if (Led2_hsv.h == 361 ) Led2_hsv.h = 0;
      else if (Led2_hsv.h == -1) Led2_hsv.h = 360;
      if (Led3_hsv.h == 361 ) Led3_hsv.h = 0;
      else if (Led3_hsv.h == -1) Led3_hsv.h = 360;



      toolsHSVtoRGB(&Led1_hsv, &rgb);
      outputsRGBLedSet(RGB_LED1, rgb.r, rgb.g, rgb.b);

      toolsHSVtoRGB(&Led2_hsv, &rgb);
      outputsRGBLedSet(RGB_LED2, rgb.r, rgb.g, rgb.b);

      toolsHSVtoRGB(&Led3_hsv, &rgb);
      outputsRGBLedSet(RGB_LED3, rgb.r, rgb.g, rgb.b);
    }

  if (value_cnt >= value_change_timer)
    {
      value_cnt = 0;


      value_change_timer = SysSensorsData.Heading / 30;
      if ((rand() % 10) % 2 == 0 ) value_change_timer++;
      else value_change_timer--;

      if (value_change_timer > 12) value_change_timer = 12;
      if (value_change_timer == 0) value_change_timer = 1;


      if (Led1_hsv.v == Led1_target_hsv.v)
        {
          srand(sensorsGetAudioSample());
          Led1_target_hsv.v = rand() % 100;
        }
      if (Led2_hsv.v == Led2_target_hsv.v)
        {
          srand(sensorsGetAudioSample());
          Led2_target_hsv.v = rand() % 100;
        }
      if (Led3_hsv.v == Led3_target_hsv.v)
        {
          srand(sensorsGetAudioSample());
          Led3_target_hsv.v = rand() % 100;
        }


      if (Led1_hsv.v > Led1_target_hsv.v) Led1_hsv.v--;
      else Led1_hsv.v++;

      if (Led2_hsv.v > Led2_target_hsv.v) Led2_hsv.v--;
      else Led2_hsv.v++;

      if (Led3_hsv.v > Led3_target_hsv.v) Led3_hsv.v--;
      else Led3_hsv.v++;



      toolsHSVtoRGB(&Led1_hsv, &rgb);
      outputsRGBLedSet(RGB_LED1, rgb.r, rgb.g, rgb.b);

      toolsHSVtoRGB(&Led2_hsv, &rgb);
      outputsRGBLedSet(RGB_LED2, rgb.r, rgb.g, rgb.b);

      toolsHSVtoRGB(&Led3_hsv, &rgb);
      outputsRGBLedSet(RGB_LED3, rgb.r, rgb.g, rgb.b);
    }

  hue_cnt++;
  value_cnt++;
}

static int32_t HeadingOld=10, HeadingCurrent=30, AwayTime=0, STOPCNT=0;

void modesRemote(void)
{

  static uint32_t HoldTime, PressedLatch;

  static int32_t HeadingChange;

  if (SysSensorsData.Inclination < 70) return;

  HeadingCurrent = SysSensorsData.Heading;

  hsv.h = HeadingCurrent;

  HeadingChange = HeadingCurrent - HeadingOld;

  if (abs(HeadingChange) > 180)
    {
      if (HeadingChange > 0)
        {
          HeadingChange = (360 - HeadingChange)*-1;
        }
      else
        {
          HeadingChange = (360 - abs(HeadingChange));
        }
    }




  if (abs(HeadingChange) > 10)
    //if (abs(HeadingChange) > 7 && SysSensorsData.CoverPercents > 7)
    {
      //outputsRGBLedSet(RGB_LED2, 50, 50, 50);
      //outputsRGBLedSet(RGB_LED3, 50, 50, 50);

      hsv.v=100;
      toolsHSVtoRGB(&hsv, &rgb);

      Led2_rgb.r = rgb.r;
      Led2_rgb.g = rgb.g;
      Led2_rgb.b = rgb.b;

      Led3_rgb.r = rgb.r;
      Led3_rgb.g = rgb.g;
      Led3_rgb.b = rgb.b;

      outputsRGBLedSet(RGB_LED2, rgb.r, rgb.g, rgb.b);
      outputsRGBLedSet(RGB_LED3, rgb.r, rgb.g, rgb.b);

      timerDelay_ms(1);
      if (HeadingChange > 0)
        {
          outputsIRLedCommand(0x40, 0xF8);
        }
      else
        {
          outputsIRLedCommand(0x40, 0x78);
        }
      outputsRGBLedSet(RGB_LED2, 0, 0, 0);
      outputsRGBLedSet(RGB_LED3, 0, 0, 0);
      HeadingOld = HeadingCurrent;
    }

  //Send_Buffer[27] = HeadingChange >> 8;
  //Send_Buffer[28] = HeadingChange;




  if (SysSensorsData.CoverPercents > 25)//  && SysSensorsData.CoverPercents < 75)
    {
      if (hsv.v < 95 && !PressedLatch) hsv.v+=3;

      if (hsv.v >= 95 && !PressedLatch)
        {
          HoldTime++;
          if (HoldTime >= 25)
            {
              PressedLatch=1;
              outputsFadeIn(0, RGB_LED2, RGB_LED3, 2);
              outputsIRLedCommand(0x40, 0x58);
              outputsFadeOut(0, RGB_LED2, RGB_LED3, 2);
              AwayTime=0;
            }
        }
    }
  else
    {
      HoldTime=0;
      if (hsv.v > 0) hsv.v-=3;
      if (hsv.v < 0) hsv.v=0;
      if (PressedLatch && hsv.v == 0)
        {
          AwayTime++;
          if (AwayTime >= 15)
            {
              AwayTime=0;
              PressedLatch=0;
              HoldTime=0;
            }
        }
    }

  toolsHSVtoRGB(&hsv, &rgb);

  Led2_rgb.r = rgb.r;
  Led2_rgb.g = rgb.g;
  Led2_rgb.b = rgb.b;

  Led3_rgb.r = rgb.r;
  Led3_rgb.g = rgb.g;
  Led3_rgb.b = rgb.b;

  outputsRGBLedSet(RGB_LED2, rgb.r, rgb.g, rgb.b);
  outputsRGBLedSet(RGB_LED3, rgb.r, rgb.g, rgb.b);

  if (rgb.r == 0 && rgb.g ==0 && rgb.b==0 && AwayTime == 0)
    {
      STOPCNT++;
      if (STOPCNT >= 30 /*|| wasRTCWakeUp == 1*/)
        {
          //wasRTCWakeUp=0;

          /* Wait till RTC Second event occurs */
          //RTC_ClearFlag(RTC_FLAG_SEC);
          //while(RTC_GetFlagStatus(RTC_FLAG_SEC) == RESET);

          /* Alarm in 3 second */
          //RTC_SetAlarm(RTC_GetCounter() + 10);
          /* Wait until last write operation on RTC registers has finished */
          //RTC_WaitForLastTask();

          ResetInt1Latch();

          EXTI_ClearITPendingBit(LSM303_INT1_EXTI_LINE);
          //EXTI_ClearITPendingBit(EXTI_Line17);

          /* Request to enter STOP mode with regulator in low power mode*/
          PWR_EnterSTOPMode(PWR_Regulator_LowPower, PWR_STOPEntry_WFI);

          //	    /* Configures system clock after wake-up from STOP: enable HSE, PLL and select
          //	       PLL as system clock source (HSE and PLL are disabled in STOP mode) */
          SYSCLKConfig_STOP();
          STOPCNT=0;

        }
    }
  else
    {
      STOPCNT=0;
    }

}

void modes5(void)
{
  static uint32_t ccc, fl, max;
  if (ccc > max)
    {
      max = SysSensorsData.Heading / 30;
      if (fl)
        {
          outputsRGBLedSet(RGB_ALL, 255, 255, 255);
          fl=0;
        }
      else
        {
          outputsRGBLedSet(RGB_ALL, 0, 0, 0);
          fl=1;
        }
      ccc = 0;

    }
  ccc++;
}

void modes6(void)
{
  outputsRGBTest(50);
}
