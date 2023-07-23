#include "usb.h"
#include "usb_lib.h"
#include "modes.h"


volatile uint32_t USB_ModeChangeCommand, USB_DataPacket;
uint8_t Receive_Buffer[64];
uint8_t Send_Buffer[64];

extern ColorHSV_t hsv;
extern ColorRGB_t rgb, Led1_rgb, Led2_rgb, Led3_rgb;

static uint32_t CheckStatusTimeout;

extern SysSensorsData_TypeDef SysSensorsData;

static USB_CONNECTION_STATUS_t UsbConnected;
USB_POWER_STATUS_t UsbPowered;

uint8_t usbPowerDetect(void)
{
  uint8_t status = USB_POWER_PRESENT;

  /*!< Check GPIO to detect SD */
  if (GPIO_ReadInputDataBit(USB_POWER_DETECT_PORT, USB_POWER_DETECT_PIN) == Bit_RESET)
    {
      status = USB_POWER_NOT_PRESENT;
    }
  return status;
}

void usbInit(void)
{
  CheckStatusTimeout = 100;
  UsbConnected = USB_DISCONNECTED;
  UsbPowered = USB_UNPOWERED;
  USB_ModeChangeCommand = 0;
  USB_DataPacket = 0;
}

USB_POWER_STATUS_t usbCheckPowerStatus(void)
{
  return UsbPowered;
}

USB_CONNECTION_STATUS_t usbCheckConnectionStatus(void)
{
  return UsbConnected;
}

uint32_t usbCheckModeChangeCommand(void)
{
  if (USB_ModeChangeCommand)
    {
      USB_ModeChangeCommand = 0;
      return 1;
    }
  return 0;
}

uint32_t usbCheckNewDataPacket(void)
{
  if (USB_DataPacket)
    {
      USB_DataPacket = 0;
      return 1;
    }
  return 0;
}

void usb10msManage(void)
{

  if (!CheckStatusTimeout)
    {
      if (usbPowerDetect()) UsbPowered = USB_POWERED;
      else UsbPowered = USB_UNPOWERED;

      if (UsbPowered == USB_POWERED && bDeviceState == CONFIGURED)
        {
          UsbConnected = USB_CONNECTED;
        }
      else
        {
          UsbConnected = USB_DISCONNECTED;
        }
      CheckStatusTimeout=100;
    }
  CheckStatusTimeout--;

  if (UsbConnected == USB_CONNECTED)
    {
      usbSendCommonData();
    }
}


void usbSendCommonData(void)
{
  if ( GetEPTxStatus( ENDP1 ) == EP_TX_NAK )  // Is the endpoint empty?
    {
      Send_Buffer[0] = modesModeGet();

      Send_Buffer[2] = Led1_rgb.r;
      Send_Buffer[3] = Led1_rgb.g;
      Send_Buffer[4] = Led1_rgb.b;

      Send_Buffer[5] = Led2_rgb.r;
      Send_Buffer[6] = Led2_rgb.g;
      Send_Buffer[7] = Led2_rgb.b;

      Send_Buffer[8] = Led3_rgb.r;
      Send_Buffer[9] = Led3_rgb.g;
      Send_Buffer[10] = Led3_rgb.b;

      Send_Buffer[11] = hsv.h >> 8;
      Send_Buffer[12] = hsv.h;
      Send_Buffer[13] = hsv.s;
      Send_Buffer[14] = hsv.v;

      Send_Buffer[15] = SysSensorsData.Heading >> 8;
      Send_Buffer[16] = SysSensorsData.Heading;

      Send_Buffer[17] = SysSensorsData.Inclination;

      Send_Buffer[18] = SysSensorsData.Proximity >> 8;
      Send_Buffer[19] = SysSensorsData.Proximity;

      Send_Buffer[20] = SysSensorsData.CoverPercents;

      Send_Buffer[21] = SysSensorsData.LightIntensity >> 8;
      Send_Buffer[22] = SysSensorsData.LightIntensity;

      Send_Buffer[23] = SysSensorsData.Voltage >> 8;
      Send_Buffer[24] = SysSensorsData.Voltage;

      Send_Buffer[25] = SysSensorsData.Temperature >> 8;
      Send_Buffer[26] = SysSensorsData.Temperature;

      USB_SIL_Write(EP1_IN, (uint8_t*) Send_Buffer, 64);
      SetEPTxValid(ENDP1);
    }
}
