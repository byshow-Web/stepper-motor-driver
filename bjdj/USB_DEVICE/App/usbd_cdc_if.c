#include "usbd_cdc_if.h"
#include "usb_device.h"
#include "main.h"

extern USBD_HandleTypeDef hUsbDeviceFS;

#define APP_RX_DATA_SIZE  64U
#define APP_TX_DATA_SIZE  64U

static uint8_t UserRxBuffer[APP_RX_DATA_SIZE];
static uint8_t UserTxBuffer[APP_TX_DATA_SIZE];
static USBD_CDC_LineCodingTypeDef LineCoding = {115200U, 0U, 0U, 8U};

static int8_t CDC_Init_FS(void)
{
  USBD_CDC_SetTxBuffer(&hUsbDeviceFS, UserTxBuffer, 0U);
  USBD_CDC_SetRxBuffer(&hUsbDeviceFS, UserRxBuffer);
  return (int8_t)USBD_OK;
}

static int8_t CDC_DeInit_FS(void)
{
  return (int8_t)USBD_OK;
}

static int8_t CDC_Control_FS(uint8_t cmd, uint8_t *pbuf, uint16_t length)
{
  (void)length;
  switch (cmd)
  {
    case CDC_SET_LINE_CODING:
      LineCoding.bitrate = (uint32_t)pbuf[0] | ((uint32_t)pbuf[1] << 8) |
                           ((uint32_t)pbuf[2] << 16) | ((uint32_t)pbuf[3] << 24);
      LineCoding.format = pbuf[4];
      LineCoding.paritytype = pbuf[5];
      LineCoding.datatype = pbuf[6];
      break;
    case CDC_GET_LINE_CODING:
      pbuf[0] = (uint8_t)LineCoding.bitrate;
      pbuf[1] = (uint8_t)(LineCoding.bitrate >> 8);
      pbuf[2] = (uint8_t)(LineCoding.bitrate >> 16);
      pbuf[3] = (uint8_t)(LineCoding.bitrate >> 24);
      pbuf[4] = LineCoding.format;
      pbuf[5] = LineCoding.paritytype;
      pbuf[6] = LineCoding.datatype;
      break;
    default:
      break;
  }
  return (int8_t)USBD_OK;
}

static int8_t CDC_Receive_FS(uint8_t *buf, uint32_t *len)
{
  uint32_t i;
  for (i = 0U; i < *len; i++) Protocol_ReceiveByte(buf[i]);
  USBD_CDC_SetRxBuffer(&hUsbDeviceFS, UserRxBuffer);
  USBD_CDC_ReceivePacket(&hUsbDeviceFS);
  return (int8_t)USBD_OK;
}

USBD_CDC_ItfTypeDef USBD_CDC_fops = {
  CDC_Init_FS,
  CDC_DeInit_FS,
  CDC_Control_FS,
  CDC_Receive_FS
};

uint8_t CDC_Transmit_FS(uint8_t *buf, uint16_t len)
{
  USBD_CDC_HandleTypeDef *hcdc;
  hcdc = (USBD_CDC_HandleTypeDef *)hUsbDeviceFS.pClassData;
  if ((hcdc == 0) || (hcdc->TxState != 0U)) return 1U;
  if (len > APP_TX_DATA_SIZE) return 1U;
  /* USB transmission is asynchronous: copy before the caller's stack frame ends. */
  memcpy(UserTxBuffer, buf, len);
  USBD_CDC_SetTxBuffer(&hUsbDeviceFS, UserTxBuffer, len);
  return (uint8_t)USBD_CDC_TransmitPacket(&hUsbDeviceFS);
}
