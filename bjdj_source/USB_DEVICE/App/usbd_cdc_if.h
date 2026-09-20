#ifndef __USBD_CDC_IF_H
#define __USBD_CDC_IF_H

#include "usbd_cdc.h"

extern USBD_CDC_ItfTypeDef USBD_CDC_fops;
uint8_t CDC_Transmit_FS(uint8_t *buf, uint16_t len);

#endif
