#include "usb_device.h"
#include "usbd_core.h"
#include "usbd_cdc.h"
#include "usbd_cdc_if.h"
#include "usbd_desc.h"

USBD_HandleTypeDef hUsbDeviceFS;

void MX_USB_DEVICE_Init(void)
{
  if (USBD_Init(&hUsbDeviceFS, &VCP_Desc, 0U) != USBD_OK) return;
  if (USBD_RegisterClass(&hUsbDeviceFS, &USBD_CDC) != USBD_OK) return;
  if (USBD_CDC_RegisterInterface(&hUsbDeviceFS, &USBD_CDC_fops) != USBD_OK) return;
  (void)USBD_Start(&hUsbDeviceFS);
}
