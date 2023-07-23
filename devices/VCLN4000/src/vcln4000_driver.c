#include "vcln4000_driver.h"
#include "i2c.h"
#include "timer.h"

/*******************************************************************************
 * Function Name		: ReadReg
 * Description		: Generic Reading function. It must be fullfilled with either
 *			: I2C or SPI reading functions
 * Input			: Register Address
 * Output		: Data REad
 * Return		: None
 *******************************************************************************/
static uint8_t ReadReg(uint8_t Reg, uint8_t* Data) {

  //To be completed with either I2c or SPI reading function
  //*Data = SPI_Mems_Read_Reg( Reg );
  if(!I2C_BufferRead(Data, VCLN4000_I2C_ADDRESS, Reg, 1))
    return VCLN4000_ERROR;
  else
    return VCLN4000_SUCCESS;
}

/*******************************************************************************
 * Function Name		: WriteReg
 * Description		: Generic Writing function. It must be fullfilled with either
 *			: I2C or SPI writing function
 * Input			: Register Address, Data to be written
 * Output		: None
 * Return		: None
 *******************************************************************************/
static uint8_t WriteReg(uint8_t WriteAddr, uint8_t Data) {

  //To be completed with either I2c or SPI writing function
  //SPI_Mems_Write_Reg(Reg, Data);
  I2C_ByteWrite(&Data,  VCLN4000_I2C_ADDRESS,  WriteAddr);
  return 1;
}


void vcln4000_driverInit(void)
{
  //	uint8_t value;
  //	ReadReg(PROD_REV_ID_REG, &value);

  /* Set recommended datasheet value	*/
  WriteReg(PROX_MODULATOR_TIMING_ADJ_REG, PROX_MODULATOR_TIMING_ADJ_SAMPLE_VALUE);

  /* Set continues conversation mode for light sense	*/
  //WriteReg(AMBIENT_LIGHT_PARAM_REG, CONT_CONV_MODE);

  vcln4000_driverSetLEDCurrent(20);	// Set IR led current = 200 mA
}

void vcln4000_driverSetLEDCurrent(uint8_t current)
{
  WriteReg(LED_CURRENT_REG, current);
}

uint32_t vcln4000_driverPerformProximityMeasurment(void)
{
  uint32_t proximity_value = 0xFFFF, cnt = 0;
  uint8_t byte;

  //ReadReg(PROX_MEASUREMENT_RESULT_HIGH_REG, &byte);	/* DUMMY reading for */
  //ReadReg(PROX_MEASUREMENT_RESULT_LOW_REG, &byte);	/* PROX_DATA_RDY flag clear	*/

  ReadReg(COMMAND_REG, &byte);
  byte |= 1 << PROX_OD;
  WriteReg(COMMAND_REG, byte);
  //Delay(1);
  timerDelay_us(300);

  for (cnt = 0; cnt < 1000; cnt++)
    {
      byte=0;
      ReadReg(COMMAND_REG, &byte);
      if (byte & (1 << PROX_DATA_RDY))
        {
          ReadReg(PROX_MEASUREMENT_RESULT_HIGH_REG, &byte);
          proximity_value = byte;
          proximity_value = proximity_value << 8;
          ReadReg(PROX_MEASUREMENT_RESULT_LOW_REG, &byte);
          proximity_value += byte;
          break;
        }
    }

  return proximity_value;
}

void vcln4000_driverStartLightMeasurment(void)
{

  //ReadReg(AMBIENT_LIGHT_RESULT_HIGH_REG, &byte);	/* DUMMY reading for */
  //ReadReg(AMBIENT_LIGHT_RESULT_LOW_REG, &byte);	/* PROX_DATA_RDY flag clear	*/

  WriteReg(COMMAND_REG, 1 << ALS_OD);

}


uint32_t vcln4000_driverGetLightMeasurmentResult(void)
{
  uint32_t light_value = 0xFFF1, cnt = 0;
  uint8_t byte;


  for (cnt = 0; cnt < 10; cnt++)
    {
      byte=0;
      ReadReg(COMMAND_REG, &byte);
      if (byte & (1 << ALS_DATA_RDY))
        {
          ReadReg(AMBIENT_LIGHT_RESULT_HIGH_REG, &byte);
          light_value = byte;
          light_value = light_value << 8;
          ReadReg(AMBIENT_LIGHT_RESULT_LOW_REG, &byte);
          light_value += byte;
          break;
        }
    }

  return light_value;
}


uint32_t vcln4000_driverPerformLightMeasurment(void)
{
  uint32_t light_value = 0xFFFF, cnt = 0;
  uint8_t byte;

  //ReadReg(AMBIENT_LIGHT_RESULT_HIGH_REG, &byte);	/* DUMMY reading for */
  //ReadReg(AMBIENT_LIGHT_RESULT_LOW_REG, &byte);	/* PROX_DATA_RDY flag clear	*/

  WriteReg(COMMAND_REG, 1 << ALS_OD);
  //Delay(1);


  for (cnt = 0; cnt < 10000; cnt++)
    {
      byte=0;
      ReadReg(COMMAND_REG, &byte);
      if (byte & (1 << ALS_DATA_RDY))
        {
          ReadReg(AMBIENT_LIGHT_RESULT_HIGH_REG, &byte);
          light_value = byte;
          light_value = light_value << 8;
          ReadReg(AMBIENT_LIGHT_RESULT_LOW_REG, &byte);
          light_value += byte;
          break;
        }
    }

  return light_value;
}


