#ifndef __MODES_H
#define __MODES_H

#include "system.h"


#define VBAT_DISCHARGE_LOW_THRESHOLD 360 //3.5V
#define VBAT_DISCHARGE_HIGH_THRESHOLD 365 //3.6V

#define SELECTOR_MODE_DOWN_INCLINATION_THRESHOLD	-65

#define FFT_SIZE	64

typedef enum {
  MODE_COLOR_SELECTOR = 0x00,     //red
  MODE_COLOR_MUSIC = 0x01,        //yellow
  MODE_RANDOM_COLOR = 0x02,       //green
  MODE_REMOTE = 0x03,             //light blue
  MODE_5 = 0x04,
  MODE_6 = 0x05,
  MODE_IDLE = 0x06,
  MODE_STANDBY = 0x07,
  MODE_SELECTOR = 0x08,
} DeviceMode_t;


typedef enum {
  SUB_MODE_COLOR_SELECTOR_PC_DEF = 0x10,
  SUB_MODE_COLOR_SELECTOR_MOTION_DEF = 0x20,
  SUB_MODE_IDLE_BATLOW_INDICATING = 0x16,
} DeviceSubMode_t;



void modesInit(void);
uint8_t modesModeSet(DeviceMode_t newMode);
uint8_t modesSubModeSet(DeviceSubMode_t newSubMode);
DeviceSubMode_t modesSubModeGet(void);
DeviceMode_t modesGetSelectedMode(void);
void modesManage(void);
void modes10msManage(void);
void modesSuperLoop(void);
void modesSelectionDoneClear(void);
void modesGetModeColor(DeviceMode_t mode);
DeviceMode_t modesModeGet(void);

#endif /* __MODES_H */
