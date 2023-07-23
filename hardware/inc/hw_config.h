/******************** (C) COPYRIGHT 2011 STMicroelectronics ********************
* File Name          : hw_config.h
* Author             : MCD Application Team
* Version            : V3.3.0
* Date               : 21-March-2011
* Description        : Hardware Configuration & Setup
********************************************************************************
* THE PRESENT FIRMWARE WHICH IS FOR GUIDANCE ONLY AIMS AT PROVIDING CUSTOMERS
* WITH CODING INFORMATION REGARDING THEIR PRODUCTS IN ORDER FOR THEM TO SAVE TIME.
* AS A RESULT, STMICROELECTRONICS SHALL NOT BE HELD LIABLE FOR ANY DIRECT,
* INDIRECT OR CONSEQUENTIAL DAMAGES WITH RESPECT TO ANY CLAIMS ARISING FROM THE
* CONTENT OF SUCH FIRMWARE AND/OR THE USE MADE BY CUSTOMERS OF THE CODING
* INFORMATION CONTAINED HEREIN IN CONNECTION WITH THEIR PRODUCTS.
*******************************************************************************/

/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef __HW_CONFIG_H
#define __HW_CONFIG_H


/* Includes ------------------------------------------------------------------*/
#include "system.h"

#define I2C_Speed              200000
//#define I2C_Speed              100000 //the real freq is 200KHz! (????)
//#define I2C_Speed              200000 //the real freq is KHz! (????)
//#define I2C_Speed              400000
#define I2C_SLAVE_ADDRESS7     0xA0
#define I2C_TIMEOUT             100

#define USB_DISCONNECT                      GPIOB
#define USB_DISCONNECT_PIN                  GPIO_Pin_3
#define APB2Periph_GPIO_DISCONNECT      	RCC_APB2Periph_GPIOB

#define DEBUG_PORT                      GPIOA
#define DEBUG_PIN                  GPIO_Pin_10

#define IR_LED_PORT                      GPIOA
#define IR_LED_PIN                  GPIO_Pin_9

#define IR_LED_LOW	GPIO_ResetBits(IR_LED_PORT, IR_LED_PIN);
#define IR_LED_HIGH 	GPIO_SetBits(IR_LED_PORT, IR_LED_PIN);


#define MAX9814_SHDN_PORT				GPIOB
#define MAX9814_SHDN_PIN                GPIO_Pin_5

#define MAX9814_SHDN_ON		GPIO_ResetBits(MAX9814_SHDN_PORT, MAX9814_SHDN_PIN);
#define MAX9814_SHDN_OFF	GPIO_SetBits(MAX9814_SHDN_PORT, MAX9814_SHDN_PIN);


#define LSM303_INT1_PORT				GPIOA
#define LSM303_INT1_PIN                GPIO_Pin_0

#define LSM303_INT1_IS_ON	GPIO_ReadInputDataBit(LSM303_INT1_PORT, LSM303_INT1_PIN)

#define KEY_BUTTON_PIN                   GPIO_Pin_9
#define KEY_BUTTON_GPIO_PORT             GPIOB
#define KEY_BUTTON_GPIO_CLK              RCC_APB2Periph_GPIOB
#define LSM303_INT1_EXTI_LINE             EXTI_Line0
#define LSM303_INT1_EXTI_PORT_SOURCE     GPIO_PortSourceGPIOA
#define LSM303_INT1_EXTI_PIN_SOURCE      GPIO_PinSource0
#define LSM303_INT1_EXTI_IRQn             EXTI0_IRQn


#define DEBUG_LOW	GPIO_ResetBits(DEBUG_PORT, DEBUG_PIN);
#define DEBUG_HIGH 	GPIO_SetBits(DEBUG_PORT, DEBUG_PIN);


#define USB_POWER_DETECT_PORT GPIOA
#define USB_POWER_DETECT_PIN  GPIO_Pin_15

#define USB_POWER_PRESENT     1
#define USB_POWER_NOT_PRESENT 0


/* Exported types ------------------------------------------------------------*/
/* Exported constants --------------------------------------------------------*/
/* Exported macro ------------------------------------------------------------*/
/* Exported define -----------------------------------------------------------*/
/* Exported functions ------------------------------------------------------- */
void Set_System(void);
void Set_USBClock(void);
void Enter_LowPowerMode(void);
void Leave_LowPowerMode(void);
void USB_Interrupts_Config(void);
void Get_SerialNum(void);
void USB_Cable_Config (FunctionalState NewState);
void All_PINS_TO_Input(void);
void SYSCLKConfig_STOP(void);
uint32_t GetADCState(void);
void ADC_On(void);
void ADC_Off(void);


#endif  /*__HW_CONFIG_H*/

/******************* (C) COPYRIGHT 2011 STMicroelectronics *****END OF FILE****/
