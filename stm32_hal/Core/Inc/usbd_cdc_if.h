#ifndef USBD_CDC_IF_H
#define USBD_CDC_IF_H

#include <stdint.h>

#include "usbd_cdc.h"

#define APP_RX_DATA_SIZE 512U
#define APP_TX_DATA_SIZE 512U

extern USBD_CDC_ItfTypeDef USBD_Interface_fops_FS;

uint8_t CDC_Transmit_FS(uint8_t *buf, uint16_t len);
int usbd_cdc_read_byte(uint8_t *byte);

#endif
