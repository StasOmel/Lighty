#include "memory.h"

#include "eeprom.h"

extern uint32_t ProximityOffset;
extern float ProximityMaxCoverCoeff;

extern vector m_max;
extern vector m_min;

uint16_t VirtAddVarTab[NumbOfVar] =
    {
        CALIBRATION_SIGNATURE_ADDRESS,

        CALIBRATION_PROXIMITY_OFFSET,
        CALIBRATION_PROXIMITY_MAX_COVER_COEF_HI,
        CALIBRATION_PROXIMITY_MAX_COVER_COEF_LOW,

        CALIBRATION_MAG_MIN_X,
        CALIBRATION_MAG_MIN_Y,
        CALIBRATION_MAG_MIN_Z,
        CALIBRATION_MAG_MAX_X,
        CALIBRATION_MAG_MAX_Y,
        CALIBRATION_MAG_MAX_Z
    };


void memoryInit(void)
{
  FLASH_Unlock();
  EE_Init();
}

uint32_t memoryCheckCalibrationDone(void)
{
  uint16_t calibration_sig = 0;

  EE_ReadVariable(VirtAddVarTab[CALIBRATION_SIGNATURE_ADDRESS], &calibration_sig);

  if (calibration_sig == CORRECT_CALIBRATION_SIGNATURE) return 1;
  else return 0;
}

void memorySaveCalibration(void)
{
  uint32_t uCoeff;
  memcpy(&uCoeff, &ProximityMaxCoverCoeff, sizeof uCoeff);

  EE_WriteVariable(VirtAddVarTab[CALIBRATION_PROXIMITY_OFFSET], (uint16_t)ProximityOffset);

  EE_WriteVariable(VirtAddVarTab[CALIBRATION_PROXIMITY_MAX_COVER_COEF_HI], (uint16_t)(uCoeff>>16));
  EE_WriteVariable(VirtAddVarTab[CALIBRATION_PROXIMITY_MAX_COVER_COEF_LOW], (uint16_t)uCoeff);

  EE_WriteVariable(VirtAddVarTab[CALIBRATION_MAG_MIN_X], (int16_t)m_min.x);
  EE_WriteVariable(VirtAddVarTab[CALIBRATION_MAG_MIN_Y], (int16_t)m_min.y);
  EE_WriteVariable(VirtAddVarTab[CALIBRATION_MAG_MIN_Z], (int16_t)m_min.z);

  EE_WriteVariable(VirtAddVarTab[CALIBRATION_MAG_MAX_X], (int16_t)m_max.x);
  EE_WriteVariable(VirtAddVarTab[CALIBRATION_MAG_MAX_Y], (int16_t)m_max.y);
  EE_WriteVariable(VirtAddVarTab[CALIBRATION_MAG_MAX_Z], (int16_t)m_max.z);


  EE_WriteVariable(VirtAddVarTab[CALIBRATION_SIGNATURE_ADDRESS], CORRECT_CALIBRATION_SIGNATURE);
}


void memoryLoadCalibration(uint32_t load_defaults)
{
  uint16_t u16_temp;
  uint32_t u32_temp;
  int16_t s16_temp;

  if (load_defaults)
    {
      ProximityOffset = 2713;
      ProximityMaxCoverCoeff = 1.34;

      m_min.x = -566;
      m_min.y = -510;
      m_min.z = -641;

      m_max.x = 597;
      m_max.y = 555;
      m_max.z = 428;
    }
  else
    {
      EE_ReadVariable(VirtAddVarTab[CALIBRATION_PROXIMITY_OFFSET], &u16_temp);
      ProximityOffset = u16_temp;

      EE_ReadVariable(VirtAddVarTab[CALIBRATION_PROXIMITY_MAX_COVER_COEF_HI], &u16_temp);
      u32_temp = u16_temp;
      u32_temp = u32_temp << 16;

      EE_ReadVariable(VirtAddVarTab[CALIBRATION_PROXIMITY_MAX_COVER_COEF_LOW], &u16_temp);
      u32_temp += u16_temp;

      memcpy(&ProximityMaxCoverCoeff, &u32_temp, sizeof ProximityMaxCoverCoeff);

      EE_ReadVariable(VirtAddVarTab[CALIBRATION_MAG_MIN_X], &s16_temp);
      m_min.x = s16_temp;
      EE_ReadVariable(VirtAddVarTab[CALIBRATION_MAG_MIN_Y], &s16_temp);
      m_min.y = s16_temp;
      EE_ReadVariable(VirtAddVarTab[CALIBRATION_MAG_MIN_Z], &s16_temp);
      m_min.z = s16_temp;

      EE_ReadVariable(VirtAddVarTab[CALIBRATION_MAG_MAX_X], &s16_temp);
      m_max.x = s16_temp;
      EE_ReadVariable(VirtAddVarTab[CALIBRATION_MAG_MAX_Y], &s16_temp);
      m_max.y = s16_temp;
      EE_ReadVariable(VirtAddVarTab[CALIBRATION_MAG_MAX_Z], &s16_temp);
      m_max.z = s16_temp;
    }
}

