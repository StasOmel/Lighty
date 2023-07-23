#ifndef __SENSORS__H
#define __SENSORS__H

#include "system.h"


typedef enum
{
  AXIS_X,
  AXIS_Y,
  AXIS_Z
}AXIS_t;

typedef enum {
  POSITION_UP,
  POSITION_DOWN
} POSITION_t;


typedef struct
{
  int8_t Inclination;
  uint16_t Heading;

  uint16_t Proximity;
  uint8_t CoverPercents;
  uint16_t LightIntensity;

  uint16_t Voltage;

  int16_t Temperature;
} SysSensorsData_TypeDef;



#define PROXIMITY_TOUCH_THRESHOLD_COUNTS 50

#define HEADING_MEAN_SIZE	4
#define PROXIMITY_MEAN_SIZE	4

#define PROXIMITY_RAW_DATA_MEAN_SIZE	1
#define LIGHT_RAW_DATA_MEAN_SIZE	1

#define ACC_MAG_RAW_DATA_MEAN_SIZE	1

void sensorsInit(void);
void sensorsDeInit(void);

void sensorsGetAll(void);

uint32_t sensorsGetAudioSample(void);
uint16_t sensorsCompassGetHeading(void);
uint32_t sensorsCompassCalibrate(void);
uint8_t sensorsGet6DPosition(void);
int32_t sensorsInclinationGet(AXIS_t axis);
uint32_t sensorsCheckOverturnChange(POSITION_t *positionActual);
void sensorsProximityCalibration(void);
uint32_t sensorsProximityGetCover(void);
void sensorsCompassCalibration(void);
int32_t sensorsTemperatureGet(void);
uint32_t sensorsSystemGetVoltage(void);
void sensorsMotionDetectInit(void);
#endif /* __SENSORS_H */
