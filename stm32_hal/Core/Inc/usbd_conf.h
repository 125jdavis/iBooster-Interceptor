#ifndef USBD_CONF_H
#define USBD_CONF_H

#ifdef __cplusplus
extern "C" {
#endif

#include "stm32f4xx.h"

#include <string.h>

#define USBD_MAX_NUM_INTERFACES 1U
#define USBD_MAX_NUM_CONFIGURATION 1U
#define USBD_MAX_STR_DESC_SIZ 0x100U
#define USBD_SELF_POWERED 1U
#define USBD_DEBUG_LEVEL 0U

#define USBD_CDC_INTERVAL 2000U

#define USBD_malloc (void *)USBD_static_malloc
#define USBD_free USBD_static_free
#define USBD_memset memset
#define USBD_memcpy memcpy
#define USBD_Delay HAL_Delay

#if (USBD_DEBUG_LEVEL > 0U)
#include <stdio.h>
#define USBD_UsrLog(...)     \
    do {                     \
        printf(__VA_ARGS__); \
        printf("\\n");      \
    } while (0)
#else
#define USBD_UsrLog(...) \
    do {                 \
    } while (0)
#endif

#if (USBD_DEBUG_LEVEL > 1U)
#include <stdio.h>
#define USBD_ErrLog(...)      \
    do {                      \
        printf("ERROR: ");   \
        printf(__VA_ARGS__);  \
        printf("\\n");       \
    } while (0)
#else
#define USBD_ErrLog(...) \
    do {                 \
    } while (0)
#endif

#if (USBD_DEBUG_LEVEL > 2U)
#include <stdio.h>
#define USBD_DbgLog(...)      \
    do {                      \
        printf("DEBUG : ");  \
        printf(__VA_ARGS__);  \
        printf("\\n");       \
    } while (0)
#else
#define USBD_DbgLog(...) \
    do {                 \
    } while (0)
#endif

void *USBD_static_malloc(uint32_t size);
void USBD_static_free(void *p);

#ifdef __cplusplus
}
#endif

#endif
