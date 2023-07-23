/******************** (C) COPYRIGHT 2009 STMicroelectronics ********************
* File Name          : I2C1.c
* Author             : MSH Application Team
* Author             : andrea labombarda
* Version            : V1.1
* Date               : 04/05/2010
* Description        : This file provides a set of functions needed to manage the
*                     communication between I2C peripheral and sensor
* HISTORY:
* Date        | Modification                              | Author
* 04/05/2010    | Initial Revision                          | Andrea Labombarda
* 09/11/2010    | New ReadBuffer function                   | Andrea Labombarda
* 28/05/2011    | I2C SA0 CS pin support                    | Andrea Labombarda

********************************************************************************
* THE PRESENT FIRMWARE WHICH IS FOR GUIDANCE ONLY AIMS AT PROVIDING CUSTOMERS
* WITH CODING INFORMATION REGARDING THEIR PRODUCTS IN ORDER FOR THEM TO SAVE TIME.
* AS A RESULT, STMICROELECTRONICS SHALL NOT BE HELD LIABLE FOR ANY DIRECT,
* INDIRECT OR CONSEQUENTIAL DAMAGES WITH RESPECT TO ANY CLAIMS ARISING FROM THE
* CONTENT OF SUCH FIRMWARE AND/OR THE USE MADE BY CUSTOMERS OF THE CODING
* INFORMATION CONTAINED HEREIN IN CONNECTION WITH THEIR PRODUCTS.
*
* THIS SOFTWARE IS SPECIFICALLY DESIGNED FOR EXCLUSIVE USE WITH ST PARTS.
*
*******************************************************************************/

/* Includes ------------------------------------------------------------------*/
#include "i2c.h"


/* Private typedef -----------------------------------------------------------*/
/* Private define ------------------------------------------------------------*/



/* I2C STOP mask */
#define CR1_STOP_Set            ((uint16_t)0x0200)
#define CR1_STOP_Reset          ((uint16_t)0xFDFF)

/* I2C ACK mask */
#define CR1_ACK_Set             ((uint16_t)0x0400)
#define CR1_ACK_Reset           ((uint16_t)0xFBFF)

/* I2C POS mask */
#define CR1_POS_Set             ((uint16_t)0x0800)
#define CR1_POS_Reset           ((uint16_t)0xF7FF)


/* Private macro -------------------------------------------------------------*/
/* Private variables ---------------------------------------------------------*/
/* Private function prototypes -----------------------------------------------*/
/* Private functions ---------------------------------------------------------*/




/*******************************************************************************
* Function Name  : I2C_BufferRead
* Description    : read a numByteToRead bytes from I2C Bus
* Input          : deviceAddr is the I2C address of the device
*                  readAddr is the register address you want to read from
*                  numByteToRead is the number of bytes to read
* Output         : pBuffer is the buffer that contains bytes read
* Return         : None
*******************************************************************************/
uint8_t I2C_BufferRead(uint8_t* pBuffer, uint8_t deviceAddr, uint8_t readAddr, uint16_t numByteToRead) {
  //uint8_t i2cTimeOut = 0;
  //uint8_t i2cTimeOut2 = 0;
    __IO uint32_t temp = 0;
    
    if(numByteToRead > 1) {
      readAddr |= 0x80;
    }
  
  // /* While the bus is busy * /
  while(I2C_GetFlagStatus(I2C1, I2C_FLAG_BUSY));

  // * Send START condition * /
  I2C_GenerateSTART(I2C1, ENABLE);

  // / * Test on EV5 and clear it * /
  while(!I2C_CheckEvent(I2C1, I2C_EVENT_MASTER_MODE_SELECT));

  // / * Send EEPROM address for write  * /
  I2C_Send7bitAddress(I2C1, deviceAddr, I2C_Direction_Transmitter);

  // / * Test on EV6 and clear it * /
  //while(!I2C_CheckEvent(I2C1, I2C_EVENT_MASTER_TRANSMITTER_MODE_SELECTED));
  while(!I2C_CheckEvent(I2C1, I2C_EVENT_MASTER_TRANSMITTER_MODE_SELECTED)) ;

  // / * Send the EEPROM's internal address to read from: Only one byte address  * /
  I2C_SendData(I2C1, readAddr);  

  /// * Test on EV8 and clear it * /
  while(!I2C_CheckEvent(I2C1, I2C_EVENT_MASTER_BYTE_TRANSMITTED));
    
  /// * Send STRAT condition a second time * /  
  I2C_GenerateSTART(I2C1, ENABLE);

  /// * Test on EV5 and clear it * /
  while(!I2C_CheckEvent(I2C1, I2C_EVENT_MASTER_MODE_SELECT));
  
    // * Send EEPROM address for read * /
  I2C_Send7bitAddress(I2C1, deviceAddr, I2C_Direction_Receiver);  

  if (numByteToRead == 1)  {
    /* Wait until ADDR is set */
    while ((I2C1->SR1&0x0002) != 0x0002);
   /* Clear ACK bit */
    I2C1->CR1 &= CR1_ACK_Reset;
    /* Disable all active IRQs around ADDR clearing and STOP programming because the EV6_3
    software sequence must complete before the current byte end of transfer */
    __disable_irq();
    /* Clear ADDR flag */
    temp = I2C1->SR2;
    /* Program the STOP */
   I2C_GenerateSTOP(I2C1, ENABLE);
    /* Re-enable IRQs */
    __enable_irq();
    /* Wait until a data is received in DR register (RXNE = 1) EV7 */
     while ((I2C1->SR1 & 0x00040) != 0x000040);
    /* Read the data */
      *pBuffer = I2C1->DR;
  
  }
  else if (numByteToRead == 2) {
                  
    /* Set POS bit */
    I2C1->CR1 |= CR1_POS_Set;
    /* Wait until ADDR is set: EV6 */
    while ((I2C1->SR1&0x0002) != 0x0002);
    /* EV6_1: The acknowledge disable should be done just after EV6,
    that is after ADDR is cleared, so disable all active IRQs around ADDR clearing and 
    ACK clearing */
    __disable_irq();
    /* Clear ADDR by reading SR2 register  */
    temp = I2C1->SR2;
    /* Clear ACK */
    I2C1->CR1 &= CR1_ACK_Reset;
    /*Re-enable IRQs */
    __enable_irq();
    /* Wait until BTF is set */
    while ((I2C1->SR1 & 0x00004) != 0x000004);
    /* Disable IRQs around STOP programming and data reading */
    __disable_irq();
    /* Program the STOP */
    I2C_GenerateSTOP(I2C1, ENABLE);
    /* Read first data */
    *pBuffer = I2C1->DR;
    /* Re-enable IRQs */
    __enable_irq();
    /**/
    pBuffer++;
    /* Read second data */
    *pBuffer = I2C1->DR;
    /* Clear POS bit */
    I2C1->CR1  &= CR1_POS_Reset;
  }
  

  else { //numByteToRead > 2 
    // * Test on EV6 and clear it * /
    while(!I2C_CheckEvent(I2C1, I2C_EVENT_MASTER_RECEIVER_MODE_SELECTED));
    // * While there is data to be read * /
    while(numByteToRead)   {
      /* Receive bytes from first byte until byte N-3 */
      if (numByteToRead != 3) {
        /* Poll on BTF to receive data because in polling mode we can not guarantee the
        EV7 software sequence is managed before the current byte transfer completes */
        while ((I2C1->SR1 & 0x00004) != 0x000004);
        /* Read data */
        *pBuffer = I2C1->DR;
         pBuffer++;
        /* Decrement the read bytes counter */
        numByteToRead--;
      }
      
      /* it remains to read three data: data N-2, data N-1, Data N */
      if (numByteToRead == 3) {
        /* Wait until BTF is set: Data N-2 in DR and data N -1 in shift register */
        while ((I2C1->SR1 & 0x00004) != 0x000004);
        /* Clear ACK */
        I2C1->CR1 &= CR1_ACK_Reset;
    
        /* Disable IRQs around data reading and STOP programming */
        __disable_irq();
        /* Read Data N-2 */
        *pBuffer = I2C1->DR;
        /* Increment */
        pBuffer++;
        /* Program the STOP */
        I2C1->CR1 |= CR1_STOP_Set;
        /* Read DataN-1 */
        *pBuffer = I2C1->DR;
        /* Re-enable IRQs */
        __enable_irq();
        /* Increment */
        pBuffer++;
        /* Wait until RXNE is set (DR contains the last data) */
        while ((I2C1->SR1 & 0x00040) != 0x000040);
        /* Read DataN */
        *pBuffer = I2C1->DR;
        /* Reset the number of bytes to be read by master */
        numByteToRead = 0;
      }
    }
  }
 
  /* Make sure that the STOP bit is cleared by Hardware before CR1 write access */
  while ((I2C1->CR1&0x200) == 0x200);

  // * Enable Acknowledgement to be ready for another reception * /
  I2C_AcknowledgeConfig(I2C1, ENABLE);
  
  return 1;
    
}


/*******************************************************************************
* Function Name  : I2C_ByteWrite
* Description    : write a Byte to I2C Bus
* Input          : deviceAddr is the I2C address of the device
*                  WriteAddr is the register address you want to write to
*                  pBuffer contains bytes to write
* Output         : None
* Return         : None
*******************************************************************************/
void I2C_ByteWrite(uint8_t* pBuffer, uint8_t deviceAddress, uint8_t WriteAddr) {    

  /* Send STRAT condition */
  I2C_GenerateSTART(I2C1, ENABLE);

  /* Test on EV5 and clear it */
  while(!I2C_CheckEvent(I2C1, I2C_EVENT_MASTER_MODE_SELECT));  

  /* Send EEPROM address for write */
  I2C_Send7bitAddress(I2C1, deviceAddress, I2C_Direction_Transmitter);
  
  /* Test on EV6 and clear it */
  while(!I2C_CheckEvent(I2C1, I2C_EVENT_MASTER_TRANSMITTER_MODE_SELECTED));

  
  /* Send the EEPROM's internal address to write to : only one byte Address */
  I2C_SendData(I2C1, WriteAddr); 
  
  /* Test on EV8 and clear it */
  while(!I2C_CheckEvent(I2C1, I2C_EVENT_MASTER_BYTE_TRANSMITTED));

  /* Send the byte to be written */
  I2C_SendData(I2C1, *pBuffer); 
   
  /* Test on EV8 and clear it */
  while(!I2C_CheckEvent(I2C1, I2C_EVENT_MASTER_BYTE_TRANSMITTED));
  
  /* Send STOP condition */
  I2C_GenerateSTOP(I2C1, ENABLE);
}


/******************* (C) COPYRIGHT 2009 STMicroelectronics *****END OF FILE****/
