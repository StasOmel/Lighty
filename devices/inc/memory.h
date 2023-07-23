#ifndef _MEMORY_H
#define _MEMORY_H

#include "system.h"

#define CALIBRATION_SIGNATURE_ADDRESS			0x0000

#define CALIBRATION_PROXIMITY_OFFSET			0x0001
#define CALIBRATION_PROXIMITY_MAX_COVER_COEF_HI		0x0002
#define CALIBRATION_PROXIMITY_MAX_COVER_COEF_LOW	0x0003

#define CALIBRATION_MAG_MIN_X				0x0004
#define CALIBRATION_MAG_MIN_Y				0x0005
#define CALIBRATION_MAG_MIN_Z				0x0006
#define CALIBRATION_MAG_MAX_X				0x0007
#define CALIBRATION_MAG_MAX_Y				0x0008
#define CALIBRATION_MAG_MAX_Z				0x0009

#define CORRECT_CALIBRATION_SIGNATURE			0xA5A5


void memoryInit(void);
uint32_t memoryCheckCalibrationDone(void);
void memorySaveCalibration(void);
void memoryLoadCalibration(uint32_t load_defaults);

#endif /* _MEMORY_H */
