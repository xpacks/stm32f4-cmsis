/* -----------------------------------------------------------------------------
 * Copyright (c) 2013-2014 ARM Ltd.
 *
 * This software is provided 'as-is', without any express or implied warranty.
 * In no event will the authors be held liable for any damages arising from
 * the use of this software. Permission is granted to anyone to use this
 * software for any purpose, including commercial applications, and to alter
 * it and redistribute it freely, subject to the following restrictions:
 *
 * 1. The origin of this software must not be misrepresented; you must not
 *    claim that you wrote the original software. If you use this software in
 *    a product, an acknowledgment in the product documentation would be
 *    appreciated but is not required.
 *
 * 2. Altered source versions must be plainly marked as such, and must not be
 *    misrepresented as being the original software.
 *
 * 3. This notice may not be removed or altered from any source distribution.
 *
 *
 * $Date:        10. September 2014
 * $Revision:    V2.00
 *
 * Project:      MCI Driver Definitions for ST STM32F4xx
 * ---------------------------------------------------------------------------*/

#ifndef __MCI_STM32F4XX_H
#define __MCI_STM32F4XX_H

#include "Driver_MCI.h"
#include "stm32f4xx_hal.h"

#include "RTE_Components.h"
#ifdef    RTE_DEVICE_FRAMEWORK_CLASSIC
#include "RTE_Device.h"
#else
#include "MX_Device.h"
#endif

#ifdef RTE_DEVICE_FRAMEWORK_CLASSIC
#if (defined(RTE_Drivers_MCI0) && !RTE_SDIO)
  #error "SDIO not configured in RTE_Device.h!"
#endif

#if (!RTE_SDIO_DMA)
  #error "SDIO requires DMA! Enable DMA in RTE_Device.h!"
#endif

/* Macro definitions */
#define _GPIO_PIN_x(x)                   GPIO_PIN_##x
#define  GPIO_PIN_x(x)                  _GPIO_PIN_x(x)

#define _DMA_CHANNEL_x(x)                DMA_CHANNEL_##x
#define  DMA_CHANNEL_x(x)               _DMA_CHANNEL_x(x)

#define _DMAx_STREAMy(x, y)              DMA##x##_Stream##y
#define  DMAx_STREAMy(x, y)             _DMAx_STREAMy(x, y)

#define _DMAx_STREAMy_IRQ(x, y)          DMA##x##_Stream##y##_IRQHandler
#define  DMAx_STREAMy_IRQ(x, y)         _DMAx_STREAMy_IRQ(x, y)

#define _DMAx_STREAMy_IRQn(x, y)         DMA##x##_Stream##y##_IRQn
#define  DMAx_STREAMy_IRQn(x, y)        _DMAx_STREAMy_IRQn(x, y)

#if (RTE_SDIO_BUS_WIDTH_4)
  #define MX_SDIO_D1_Pin                1
  #define MX_SDIO_D2_Pin                1
  #define MX_SDIO_D3_Pin                1
#endif
#if (RTE_SDIO_BUS_WIDTH_8)
  #define MX_SDIO_D4_Pin                1
  #define MX_SDIO_D5_Pin                1
  #define MX_SDIO_D6_Pin                1
  #define MX_SDIO_D7_Pin                1
#endif

#if (RTE_SDIO_CD_PIN_EN)
  #define MX_MemoryCard_CD_Pin          1
  #define MX_MemoryCard_CD_GPIOx        RTE_SDIO_CD_PORT
  #define MX_MemoryCard_CD_GPIO_Pin     GPIO_PIN_x(RTE_SDIO_CD_PIN)
  #define MX_MemoryCard_CD_GPIO_PuPd    GPIO_NOPULL
  #define MemoryCard_CD_Pin_Active      ((RTE_SDIO_CD_ACTIVE == 0) ? GPIO_PIN_RESET : GPIO_PIN_SET)
#endif

#if (RTE_SDIO_WP_PIN_EN)
  #define MX_MemoryCard_WP_Pin          1
  #define MX_MemoryCard_WP_GPIOx        RTE_SDIO_WP_PORT
  #define MX_MemoryCard_WP_GPIO_Pin     GPIO_PIN_x(RTE_SDIO_WP_PIN)
  #define MX_MemoryCard_WP_GPIO_PuPd    GPIO_NOPULL
  #define MemoryCard_CD_Pin_Active      ((RTE_SDIO_WP_ACTIVE == 0) ? GPIO_PIN_RESET : GPIO_PIN_SET)
#endif

#if (RTE_SDIO_DMA)
#define MX_SDIO_DMA_Instance            DMAx_STREAMy (RTE_SDIO_DMA_NUMBER, RTE_SDIO_DMA_STREAM)
#define MX_SDIO_DMA_Channel             DMA_CHANNEL_x (RTE_SDIO_DMA_CHANNEL)
#define MX_SDIO_DMA_Priority            DMA_PRIORITY_HIGH
#define SDIO_DMA_Handler                DMAx_STREAMy_IRQ(RTE_SDIO_DMA_NUMBER, RTE_SDIO_DMA_STREAM)
#define SDIO_DMA_IRQn                   DMAx_STREAMy_IRQn(RTE_SDIO_DMA_NUMBER, RTE_SDIO_DMA_STREAM)
#endif

#else /* MX_Device.h */

#define DMAx_STREAMy_IRQ(x)             DMA2_Stream3_IRQHandler
#define DMAx_STREAMy_IRQn(x)            DMA2_Stream3_IRQn

#if defined(MX_SDIO_DMA_Instance)
#define SDIO_DMA_Handler                DMAx_STREAMy_IRQ(MX_SDIO_DMA_Instance)
#define SDIO_DMA_IRQn                   DMAx_STREAMy_IRQn(MX_SDIO_DMA_Instance)
#endif

#endif /* RTE_DEVICE_FRAMEWORK_CLASSIC */

/* Define 4-bit data bus width */
#if defined(MX_SDIO_D1_Pin) && defined(MX_SDIO_D2_Pin) && defined(MX_SDIO_D3_Pin)
  #define SDIO_BUS_WIDTH_4 1
#else
  #define SDIO_BUS_WIDTH_4 0
#endif

/* Define 8-bit data bus width */
#if defined(MX_SDIO_D4_Pin) && defined(MX_SDIO_D5_Pin) && defined(MX_SDIO_D6_Pin) && defined(MX_SDIO_D7_Pin)
  #define SDIO_BUS_WIDTH_8 1
#else
  #define SDIO_BUS_WIDTH_8 0
#endif

/* Define Card Detect pin existence */
#if defined(MX_MemoryCard_CD_Pin)
  #define SDIO_CD_PIN 1
#else
  #define SDIO_CD_PIN 0
#endif

/* Define Write Protect pin existence */
#if defined(MX_MemoryCard_WP_Pin)
  #define SDIO_WP_PIN 1
#else
  #define SDIO_WP_PIN 0
#endif

/* SDIO Adapter Clock definition */
#define SDIOCLK            48000000     /* SDIO adapter clock */

/* Interrupt clear Mask */
#define SDIO_ICR_BIT_Msk        SDIO_ICR_CCRCFAILC | \
                                SDIO_ICR_DCRCFAILC | \
                                SDIO_ICR_CTIMEOUTC | \
                                SDIO_ICR_DTIMEOUTC | \
                                SDIO_ICR_TXUNDERRC | \
                                SDIO_ICR_RXOVERRC  | \
                                SDIO_ICR_CMDRENDC  | \
                                SDIO_ICR_CMDSENTC  | \
                                SDIO_ICR_DATAENDC  | \
                                SDIO_ICR_STBITERRC | \
                                SDIO_ICR_DBCKENDC  | \
                                SDIO_ICR_SDIOITC   | \
                                SDIO_ICR_CEATAENDC

/* Driver flag definitions */
#define MCI_INIT            (1 << 0)    /* MCI initialized           */
#define MCI_POWER           (1 << 1)    /* MCI powered on            */
#define MCI_SETUP           (1 << 2)    /* MCI configured            */
#define MCI_RESP_LONG       (1 << 3)    /* Long response expected    */
#define MCI_RESP_CRC        (1 << 4)    /* Check response CRC error  */
#define MCI_DATA_XFER       (1 << 5)    /* Transfer data             */
#define MCI_DATA_READ       (1 << 6)    /* Read transfer             */
#define MCI_READ_WAIT       (1 << 7)    /* Read wait operation start */

#define MCI_RESPONSE_EXPECTED_Msk (ARM_MCI_RESPONSE_SHORT      | \
                                   ARM_MCI_RESPONSE_SHORT_BUSY | \
                                   ARM_MCI_RESPONSE_LONG)

/* MCI Transfer Information Definition */
typedef struct _MCI_XFER {
  uint8_t *buf;                         /* Data buffer                        */
  uint32_t cnt;                         /* Data bytes to transfer             */
} MCI_XFER;

/* MCI Driver State Definition */
typedef struct _MCI_INFO {
  ARM_MCI_SignalEvent_t cb_event;       /* Driver event callback function     */
  ARM_MCI_STATUS        status;         /* Driver status                      */
  uint32_t             *response;       /* Pointer to response buffer         */
  MCI_XFER              xfer;           /* Data transfer description          */
  uint8_t               flags;          /* Driver state flags                 */
  uint32_t dctrl;
  uint32_t dlen;
  uint32_t dtimer;
  uint32_t rd_timeout;
  uint32_t wr_timeout;
} MCI_INFO;

#endif /* __MCI_STM32F4XX_H */
