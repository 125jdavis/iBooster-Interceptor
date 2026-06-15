#include "usbd_cdc_if.h"

#include <stdbool.h>

#include "usb_device.h"

static int8_t CDC_Init_FS(void);
static int8_t CDC_DeInit_FS(void);
static int8_t CDC_Control_FS(uint8_t cmd, uint8_t *pbuf, uint16_t length);
static int8_t CDC_Receive_FS(uint8_t *pbuf, uint32_t *len);
static int8_t CDC_TransmitCplt_FS(uint8_t *pbuf, uint32_t *len, uint8_t epnum);

USBD_CDC_ItfTypeDef USBD_Interface_fops_FS = {
    CDC_Init_FS,
    CDC_DeInit_FS,
    CDC_Control_FS,
    CDC_Receive_FS,
    CDC_TransmitCplt_FS,
};

static uint8_t UserRxBufferFS[APP_RX_DATA_SIZE];
static uint8_t UserTxBufferFS[APP_TX_DATA_SIZE];

static uint8_t rx_ring[APP_RX_DATA_SIZE];
static volatile uint16_t rx_head;
static volatile uint16_t rx_tail;

static USBD_CDC_LineCodingTypeDef linecoding = {
    115200,
    0x00,
    0x00,
    0x08,
};

static int8_t CDC_Init_FS(void)
{
    rx_head = 0U;
    rx_tail = 0U;

    USBD_CDC_SetTxBuffer(&hUsbDeviceFS, UserTxBufferFS, 0U);
    USBD_CDC_SetRxBuffer(&hUsbDeviceFS, UserRxBufferFS);
    return (USBD_CDC_ReceivePacket(&hUsbDeviceFS) == USBD_OK) ? USBD_OK : USBD_FAIL;
}

static int8_t CDC_DeInit_FS(void)
{
    return USBD_OK;
}

static int8_t CDC_Control_FS(uint8_t cmd, uint8_t *pbuf, uint16_t length)
{
    UNUSED(length);

    switch (cmd) {
    case CDC_SET_LINE_CODING:
        linecoding.bitrate = (uint32_t)(pbuf[0] | (pbuf[1] << 8) | (pbuf[2] << 16) | (pbuf[3] << 24));
        linecoding.format = pbuf[4];
        linecoding.paritytype = pbuf[5];
        linecoding.datatype = pbuf[6];
        break;

    case CDC_GET_LINE_CODING:
        pbuf[0] = (uint8_t)(linecoding.bitrate);
        pbuf[1] = (uint8_t)(linecoding.bitrate >> 8);
        pbuf[2] = (uint8_t)(linecoding.bitrate >> 16);
        pbuf[3] = (uint8_t)(linecoding.bitrate >> 24);
        pbuf[4] = linecoding.format;
        pbuf[5] = linecoding.paritytype;
        pbuf[6] = linecoding.datatype;
        break;

    case CDC_SET_CONTROL_LINE_STATE:
    case CDC_SEND_BREAK:
    case CDC_SEND_ENCAPSULATED_COMMAND:
    case CDC_GET_ENCAPSULATED_RESPONSE:
    case CDC_SET_COMM_FEATURE:
    case CDC_GET_COMM_FEATURE:
    case CDC_CLEAR_COMM_FEATURE:
    default:
        break;
    }

    return USBD_OK;
}

static int8_t CDC_Receive_FS(uint8_t *pbuf, uint32_t *len)
{
    for (uint32_t idx = 0; idx < *len; idx++) {
        uint16_t next = (uint16_t)((rx_head + 1U) % APP_RX_DATA_SIZE);
        if (next == rx_tail) {
            break;
        }
        rx_ring[rx_head] = pbuf[idx];
        rx_head = next;
    }

    USBD_CDC_SetRxBuffer(&hUsbDeviceFS, UserRxBufferFS);
    return (USBD_CDC_ReceivePacket(&hUsbDeviceFS) == USBD_OK) ? USBD_OK : USBD_FAIL;
}

static int8_t CDC_TransmitCplt_FS(uint8_t *pbuf, uint32_t *len, uint8_t epnum)
{
    UNUSED(pbuf);
    UNUSED(len);
    UNUSED(epnum);
    return USBD_OK;
}

uint8_t CDC_Transmit_FS(uint8_t *buf, uint16_t len)
{
    USBD_CDC_HandleTypeDef *cdc = (USBD_CDC_HandleTypeDef *)hUsbDeviceFS.pClassData;

    if ((hUsbDeviceFS.pClassData == NULL) || (hUsbDeviceFS.dev_state != USBD_STATE_CONFIGURED)) {
        return USBD_FAIL;
    }

    if (cdc->TxState != 0U) {
        return USBD_BUSY;
    }

    USBD_CDC_SetTxBuffer(&hUsbDeviceFS, buf, len);
    return USBD_CDC_TransmitPacket(&hUsbDeviceFS);
}

int usbd_cdc_read_byte(uint8_t *byte)
{
    int has_data = 0;

    __disable_irq();
    if (rx_tail != rx_head) {
        *byte = rx_ring[rx_tail];
        rx_tail = (uint16_t)((rx_tail + 1U) % APP_RX_DATA_SIZE);
        has_data = 1;
    }
    __enable_irq();

    return has_data;
}
