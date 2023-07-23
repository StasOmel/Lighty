#ifndef __USB_H
#define __USB_H

#include "main.h"
#include "system.h"
#include "usb_lib.h"
#include "usb_pwr.h"

typedef enum {
	USB_CONNECTED,
	USB_DISCONNECTED
} USB_CONNECTION_STATUS_t;

typedef enum {
	USB_POWERED,
	USB_UNPOWERED
} USB_POWER_STATUS_t;

enum USB_PACKET_ID
{
USB_PACKET_ID_MODE_CHANGE = 0xAA,
USB_PACKET_ID_DATA = 0xBB
};

uint8_t usbPowerDetect(void);
USB_POWER_STATUS_t usbCheckPowerStatus(void);
void usbSendCommonData(void);
uint32_t usbCheckModeChangeCommand(void);
uint32_t usbCheckNewDataPacket(void);
void usbInit(void);
void usb10msManage(void);
USB_CONNECTION_STATUS_t usbCheckConnectionStatus(void);
#endif /* __USB_H */
