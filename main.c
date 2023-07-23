#include "main.h"
#include "modes.h"


int main(void)
{
  Set_System();

  USB_Interrupts_Config();
  Set_USBClock();
  USB_Init();

  sensorsInit();
  outputsInit();
  modesInit();
  timerInit();
  memoryInit();
  usbInit();



  sensorsMotionDetectInit();
  /*
	while(1)
	{


	RTC_ClearFlag(RTC_FLAG_SEC);
	while(RTC_GetFlagStatus(RTC_FLAG_SEC) == RESET);


	RTC_SetAlarm(RTC_GetCounter() + 10);

	RTC_WaitForLastTask();

	ResetInt1Latch();

	EXTI_ClearITPendingBit(LSM303_INT1_EXTI_LINE);

	PWR_EnterSTOPMode(PWR_Regulator_LowPower, PWR_STOPEntry_WFI);

	SYSCLKConfig_STOP();

	outputsRGBLedSet(RGB_ALL, 50, 0, 0);
	timerDelay_ms(1000);
	ResetInt1Latch();
	outputsRGBLedSet(RGB_ALL, 0, 0, 0);

	}
   */


  //	     while (1)
  //	{
  //		/* Request to enter STOP mode with regulator in low power mode*/
  //		PWR_EnterSTOPMode(PWR_Regulator_LowPower, PWR_STOPEntry_WFI);
  //
  //	    /* Configures system clock after wake-up from STOP: enable HSE, PLL and select
  //	       PLL as system clock source (HSE and PLL are disabled in STOP mode) */
  //	    SYSCLKConfig_STOP();
  //
  //		outputsRGBLedSet(RGB_ALL, 50,  0, 0);
  //		timerDelay_ms(1000);
  //		ResetInt1Latch();
  //		outputsRGBLedSet(RGB_ALL, 0, 0, 0);
  //	}




  //	  //Allow access to BKP Domain
  //	  PWR_BackupAccessCmd(ENABLE);
  //
  //	//Check if the StandBy flag is set
  //	  if(PWR_GetFlagStatus(PWR_FLAG_SB) != RESET)
  //	  {
  //	  // System resumed from STANDBY mode
  //
  //		  outputsRGBLedSet(RGB_ALL, 10, 0, 0);
  //
  //	   // Clear StandBy flag
  //	  PWR_ClearFlag(PWR_FLAG_SB);
  //
  //	  // Wait for RTC APB registers synchronisation
  //	  RTC_WaitForSynchro();
  //	  }
  //	  else
  //	  {
  //		  outputsRGBLedSet(RGB_ALL, 0, 10, 0);
  //
  //	  // RTC clock source configuration ----------------------------------------
  //	  // Reset Backup Domain
  //
  //
  //	  // Enable LSI OSC
  //	  RCC_LSICmd(ENABLE);
  //	  // Wait till LSI is ready
  //	  while(RCC_GetFlagStatus(RCC_FLAG_LSIRDY) == RESET);
  //
  //	  // Select the RTC Clock Source
  //	  RCC_RTCCLKConfig(RCC_RTCCLKSource_LSI);
  //	  // Enable the RTC Clock
  //	  RCC_RTCCLKCmd(ENABLE);
  //
  //	  // RTC configuration -----------------------------------------------------
  //	  // Wait for RTC APB registers synchronisation
  //	  RTC_WaitForSynchro();
  //
  //	  // Set the RTC time base to 1s
  //	  RTC_SetPrescaler(37000);
  //
  //	  // Wait until last write operation on RTC registers has finished
  //	  RTC_WaitForLastTask();
  //	  }
  //
  //
  //	  timerDelay_ms(3000);
  //
  //
  //
  //	  outputsRGBLedSet(RGB_ALL, 0, 0, 0);
  //
  //	  sensorsDeInit();
  //	  outputsDeInit();
  //	  All_PINS_TO_Input();
  //
  //	    RTC_ClearFlag(RTC_FLAG_SEC);
  //	    while(RTC_GetFlagStatus(RTC_FLAG_SEC) == RESET);
  //
  //	    RTC_SetAlarm(RTC_GetCounter()+10);
  //	    RTC_WaitForLastTask();
  //	    PWR_EnterSTANDBYMode();

  /*
	outputsRGBLedSet(RGB_ALL, 10, 10, 10);
	timerDelay_ms(1000);
	outputsRGBLedSet(RGB_ALL, 0, 0, 0);
   */

  if (memoryCheckCalibrationDone())
    {
      memoryLoadCalibration(0);
      sensorsGetAll();
      sensorsGetAll();
      if (sensorsInclinationGet(AXIS_Z) < -50)
        {
          sensorsCompassCalibration();
          timerDelay_ms(10000);
          sensorsProximityCalibration();
          timerDelay_ms(1000);
          memorySaveCalibration();
        }
    }
  else
    {
      sensorsCompassCalibration();
      timerDelay_ms(10000);
      sensorsProximityCalibration();
      timerDelay_ms(1000);
      memorySaveCalibration();
    }

  outputsRGBLedSet(RGB_ALL, 0, 0, 0);

  while(1)
    {
      if (timer10msTest())
        {
          //DEBUG_HIGH;
          sensorsGetAll();
          modesManage();
          modes10msManage();
          usb10msManage();
          //DEBUG_LOW;
        }
      modesSuperLoop();
    }

}

