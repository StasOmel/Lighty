#ifndef __OUTPUTS__H
#define __OUTPUTS__H

#include "system.h"


typedef enum {
  RGB_LED0, // dummy
  RGB_LED1,
  RGB_LED2,
  RGB_LED3,
  RGB_ALL
} RGBLedn_t;

#define LED1_RED_CHANNEL	MAX6967_P2_OUT_LEVEL_A
#define LED1_GREEN_CHANNEL	MAX6967_P0_OUT_LEVEL_A
#define LED1_BLUE_CHANNEL	MAX6967_P1_OUT_LEVEL_A

#define LED2_RED_CHANNEL	MAX6967_P3_OUT_LEVEL_A
#define LED2_GREEN_CHANNEL	MAX6967_P4_OUT_LEVEL_A
#define LED2_BLUE_CHANNEL	MAX6967_P5_OUT_LEVEL_A

#define LED3_RED_CHANNEL	MAX6967_P6_OUT_LEVEL_A
#define LED3_GREEN_CHANNEL	MAX6967_P7_OUT_LEVEL_A
#define LED3_BLUE_CHANNEL	MAX6967_P8_OUT_LEVEL_A

void outputsInit(void);
void outputsDeInit(void);
void outputsRGBLedSet(RGBLedn_t Ledn, uint32_t R, uint32_t G, uint32_t B);
void outputsRGBTest(uint32_t delay);
void outputsFadeInOut(void);
void outputsFadeIn(RGBLedn_t Led1, RGBLedn_t Led2, RGBLedn_t Led3, uint32_t time);
void outputsFadeOut(RGBLedn_t Led1, RGBLedn_t Led2, RGBLedn_t Led3, uint32_t time);
void outputsFadeInOutQueue(void);

void outputsIRLedCommand(uint32_t adr, uint32_t cmd);
#endif /* __OUTPUTS_H */
