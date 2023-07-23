/******************** (C) COPYRIGHT 2009 STMicroelectronics ********************
* File Name          : i2c_mems.h
* Author             : MSH Application Team
* Version            : V1.1
* Date               : 04/05/2010
* Description        : Descriptor Header for i2c_mems file
* HISTORY:
* Date          | Modification                              | Author
* 04/05/2010    | Initial Revision                          | Andrea Labombarda
* 09/11/2010    | new readBuffer Signature                  | Andrea Labombarda
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

/* Define to prevent recursive inclusion ------------------------------------ */
#ifndef __I2C_H
#define __I2C_H

/* Includes ------------------------------------------------------------------*/
#include "system.h"

/* Exported types ------------------------------------------------------------*/
/* Exported constants --------------------------------------------------------*/
/* Defines for the GPIO pins used for the I2C communication */





/* Exported macro ------------------------------------------------------------*/
/* Exported functions ------------------------------------------------------- */
uint8_t I2C_BufferRead(uint8_t* pBuffer, uint8_t deviceAddr, uint8_t readAddr, uint16_t numByteToRead);
void I2C_ByteWrite(uint8_t* pBuffer, uint8_t deviceAddress, uint8_t WriteAddr);


#endif /* __I2C_H */

/******************* (C) COPYRIGHT 2009 STMicroelectronics *****END OF FILE****/

