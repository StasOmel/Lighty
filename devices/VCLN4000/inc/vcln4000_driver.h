#ifndef __VCLN4000_DRIVER__H
#define __VCLN4000_DRIVER__H

#include "system.h"

typedef enum {
  VCLN4000_SUCCESS         =		0x01,
  VCLN4000_ERROR			=		0x00
} vcln4000_status_t;

/* Exported constants --------------------------------------------------------*/

#define VCLN4000_I2C_ADDRESS			0x26
#define PROD_REV_ID_SUPPORTED_VALUES	0x11
#define PROX_MODULATOR_TIMING_ADJ_SAMPLE_VALUE	0x81

#define BIT(x) ( (x) )

//Register Definition

// COMMAND REGISTER 0
#define COMMAND_REG				0x80
#define CONFIG_LOCK				BIT(7)
#define ALS_DATA_RDY			BIT(6)
#define PROX_DATA_RDY			BIT(5)
#define ALS_OD					BIT(4)
#define PROX_OD					BIT(3)

// PRODUCT and REVISION ID REGISTER 1
#define PROD_REV_ID_REG			0x81

// LED CURRENT SETTING REGISTER 3
#define LED_CURRENT_REG			0x83

// AMBIENT LIGHT PARAMETER REGISTER 4
#define AMBIENT_LIGHT_PARAM_REG		0x84
#define CONT_CONV_MODE				BIT(7)
#define AUTO_OFFSET_COMPENSATION	BIT(3)

// AMBIENT LIGHT RESULT REGISTER 5 (HIGH)
#define AMBIENT_LIGHT_RESULT_HIGH_REG	0x85

// AMBIENT LIGHT RESULT REGISTER 6 (LOW)
#define AMBIENT_LIGHT_RESULT_LOW_REG	0x86

// PROXIMITY LIGHT RESULT REGISTER 7 (HIGH)
#define PROX_MEASUREMENT_RESULT_HIGH_REG	0x87

// PROXIMITY LIGHT RESULT REGISTER 8 (LOW)
#define PROX_MEASUREMENT_RESULT_LOW_REG	0x88

// PROXIMITY MEASUREMENT FREQUENCY REGISTER 9
#define PROX_MEASUREMENT_FREQ_REG			0x89

// PROXIMITY MODULATOR TIMING ADJUSTMENT REGISTER 10
#define PROX_MODULATOR_TIMING_ADJ_REG		0x8A


#define PROX_MEASUREMENT_FREQ_3_125_MHZ		0x00
#define PROX_MEASUREMENT_FREQ_1_5625_MHZ	0x01
#define PROX_MEASUREMENT_FREQ_781_25_KHZ	0x02
#define PROX_MEASUREMENT_FREQ_390_525_KHZ	0x03


void vcln4000_driverInit(void);
void vcln4000_driverSetLEDCurrent(uint8_t current);
uint32_t vcln4000_driverPerformProximityMeasurment(void);
uint32_t vcln4000_driverPerformLightMeasurment(void);
uint32_t vcln4000_driverGetLightMeasurmentResult(void);
void vcln4000_driverStartLightMeasurment(void);
#endif /* __VCLN4000_DRIVER_H */
