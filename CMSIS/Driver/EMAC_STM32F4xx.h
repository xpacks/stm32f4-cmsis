/* -----------------------------------------------------------------------------
 * Copyright (c) 2013 - 2014 ARM Ltd.
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
 * $Date:        19. September 2014
 * $Revision:    V2.00
 *  
 * Project:      Ethernet Media Access (MAC) Definitions for STM32F4xx
 * --------------------------------------------------------------------------*/

#ifndef __EMAC_STM32F4XX_H
#define __EMAC_STM32F4XX_H

#include <stdint.h>

#include "Driver_ETH_MAC.h"
#include "stm32f4xx_hal.h"

#include "RTE_Components.h"
#ifdef    RTE_DEVICE_FRAMEWORK_CLASSIC
#include "RTE_Device.h"
#else
#include "MX_Device.h"
#endif

#ifdef RTE_DEVICE_FRAMEWORK_CLASSIC
  #if (defined(RTE_Drivers_ETH_MAC0) && !RTE_ETH)
  #error "Ethernet not configured in RTE_Device.h!"
  #endif

  #if (RTE_ETH_MII && RTE_ETH_RMII)
  #error "Ethernet interface configuration in RTE_Device.h is invalid!"
  #endif

  #define MX_ETH_MDC_GPIOx        RTE_ETH_MDI_MDC_PORT
  #define MX_ETH_MDC_GPIO_Pin     (1U << RTE_ETH_MDI_MDC_PIN)
  #define MX_ETH_MDIO_GPIOx       RTE_ETH_MDI_MDIO_PORT
  #define MX_ETH_MDIO_GPIO_Pin    (1U << RTE_ETH_MDI_MDIO_PIN)

  #if (RTE_ETH_MII)
    #define ETH_MII                 1

    #define MX_ETH_TXD0_GPIOx       RTE_ETH_MII_TXD0_PORT
    #define MX_ETH_TXD0_GPIO_Pin    (1U << RTE_ETH_MII_TXD0_PIN)
    #define MX_ETH_TXD1_GPIOx       RTE_ETH_MII_TXD1_PORT
    #define MX_ETH_TXD1_GPIO_Pin    (1U << RTE_ETH_MII_TXD1_PIN)
    #define MX_ETH_TXD2_GPIOx       RTE_ETH_MII_TXD2_PORT
    #define MX_ETH_TXD2_GPIO_Pin    (1U << RTE_ETH_MII_TXD2_PIN)
    #define MX_ETH_TXD3_GPIOx       RTE_ETH_MII_TXD3_PORT
    #define MX_ETH_TXD3_GPIO_Pin    (1U << RTE_ETH_MII_TXD3_PIN)
    #define MX_ETH_RXD0_GPIOx       RTE_ETH_MII_RXD0_PORT
    #define MX_ETH_RXD0_GPIO_Pin    (1U << RTE_ETH_MII_RXD0_PIN)
    #define MX_ETH_RXD1_GPIOx       RTE_ETH_MII_RXD1_PORT
    #define MX_ETH_RXD1_GPIO_Pin    (1U << RTE_ETH_MII_RXD1_PIN)
    #define MX_ETH_RXD2_GPIOx       RTE_ETH_MII_RXD2_PORT
    #define MX_ETH_RXD2_GPIO_Pin    (1U << RTE_ETH_MII_RXD2_PIN)
    #define MX_ETH_RXD3_GPIOx       RTE_ETH_MII_RXD3_PORT
    #define MX_ETH_RXD3_GPIO_Pin    (1U << RTE_ETH_MII_RXD3_PIN)
    #define MX_ETH_TX_EN_GPIOx      RTE_ETH_MII_TX_EN_PORT
    #define MX_ETH_TX_EN_GPIO_Pin   (1U << RTE_ETH_MII_TX_EN_PIN)
    #define MX_ETH_TX_CLK_GPIOx     RTE_ETH_MII_TX_CLK_PORT
    #define MX_ETH_TX_CLK_GPIO_Pin  (1U << RTE_ETH_MII_TX_CLK_PIN)
    #define MX_ETH_RX_CLK_GPIOx     RTE_ETH_MII_RX_CLK_PORT
    #define MX_ETH_RX_CLK_GPIO_Pin  (1U << RTE_ETH_MII_RX_CLK_PIN)
    #define MX_ETH_CRS_GPIOx        RTE_ETH_MII_CRS_PORT
    #define MX_ETH_CRS_GPIO_Pin     (1U << RTE_ETH_MII_CRS_PIN)
    #define MX_ETH_COL_GPIOx        RTE_ETH_MII_COL_PORT
    #define MX_ETH_COL_GPIO_Pin     (1U << RTE_ETH_MII_COL_PIN)
    #define MX_ETH_RX_DV_GPIOx      RTE_ETH_MII_RX_DV_PORT
    #define MX_ETH_RX_DV_GPIO_Pin   (1U << RTE_ETH_MII_RX_DV_PIN)
    #define MX_ETH_RX_ER_GPIOx      RTE_ETH_MII_RX_ER_PORT
    #define MX_ETH_RX_ER_GPIO_Pin   (1U << RTE_ETH_MII_RX_ER_PIN)

  #else
    #define ETH_MII                 0

    #define MX_ETH_TXD0_GPIOx       RTE_ETH_RMII_TXD0_PORT
    #define MX_ETH_TXD0_GPIO_Pin    (1U << RTE_ETH_RMII_TXD0_PIN)
    #define MX_ETH_TXD1_GPIOx       RTE_ETH_RMII_TXD1_PORT
    #define MX_ETH_TXD1_GPIO_Pin    (1U << RTE_ETH_RMII_TXD1_PIN)
    #define MX_ETH_RXD0_GPIOx       RTE_ETH_RMII_RXD0_PORT
    #define MX_ETH_RXD0_GPIO_Pin    (1U << RTE_ETH_RMII_RXD0_PIN)
    #define MX_ETH_RXD1_GPIOx       RTE_ETH_RMII_RXD1_PORT
    #define MX_ETH_RXD1_GPIO_Pin    (1U << RTE_ETH_RMII_RXD1_PIN)
    #define MX_ETH_TX_EN_GPIOx      RTE_ETH_RMII_TX_EN_PORT
    #define MX_ETH_TX_EN_GPIO_Pin   (1U << RTE_ETH_RMII_TX_EN_PIN)
    #define MX_ETH_REF_CLK_GPIOx    RTE_ETH_RMII_REF_CLK_PORT
    #define MX_ETH_REF_CLK_GPIO_Pin (1U << RTE_ETH_RMII_REF_CLK_PIN)
    #define MX_ETH_CRS_DV_GPIOx     RTE_ETH_RMII_CRS_DV_PORT
    #define MX_ETH_CRS_DV_GPIO_Pin  (1U << RTE_ETH_RMII_CRS_DV_PIN)

#endif /* RTE_ETH_RMII */

#else /* MX_Device.h */
  #if defined(MX_ETH_TXD2_Pin)   && defined(MX_ETH_TXD3_Pin)   && \
      defined(MX_ETH_RXD2_Pin)   && defined(MX_ETH_RXD3_Pin)   && \
      defined(MX_ETH_TX_CLK_Pin) && defined(MX_ETH_RX_CLK_Pin) && \
      defined(MX_ETH_CRS_Pin)    && defined(MX_ETH_COL_Pin)    && \
      defined(MX_ETH_RX_DV_Pin)  && defined(MX_ETH_RX_ER_Pin)
    #define ETH_MII             1
  #else
    #define ETH_MII             0
  #endif
#endif

/* PTP subsecond increment value */
#define PTPSSIR_Val(hclk)     ((0x7FFFFFFFU + (hclk)/2) / (hclk))

/* TDES0 - DMA Descriptor TX Packet Control/Status */
#define DMA_TX_OWN      0x80000000      // Own bit 1=DMA,0=CPU
#define DMA_TX_IC       0x40000000      // Interrupt on completition
#define DMA_TX_LS       0x20000000      // Last segment
#define DMA_TX_FS       0x10000000      // First segment
#define DMA_TX_DC       0x08000000      // Disable CRC
#define DMA_TX_DP       0x04000000      // Disable pad
#define DMA_TX_TTSE     0x02000000      // Transmit time stamp enable
#define DMA_TX_CIC      0x00C00000      // Checksum insertion control
#define DMA_TX_TER      0x00200000      // Transmit end of ring
#define DMA_TX_TCH      0x00100000      // Second address chained
#define DMA_TX_TTSS     0x00020000      // Transmit time stamp status
#define DMA_TX_IHE      0x00010000      // IP header error status
#define DMA_TX_ES       0x00008000      // Error summary
#define DMA_TX_JT       0x00004000      // Jabber timeout
#define DMA_TX_FF       0x00002000      // Frame flushed
#define DMA_TX_IPE      0x00001000      // IP payload error
#define DMA_TX_LC       0x00000800      // Loss of carrier
#define DMA_TX_NC       0x00000400      // No carrier
#define DMA_TX_LCOL     0x00000200      // Late collision
#define DMA_TX_EC       0x00000100      // Excessive collision
#define DMA_TX_VF       0x00000080      // VLAN frame
#define DMA_TX_CC       0x00000078      // Collision count
#define DMA_TX_ED       0x00000004      // Excessive deferral
#define DMA_TX_UF       0x00000002      // Underflow error
#define DMA_TX_DB       0x00000001      // Deferred bit

/* TDES1 - DMA Descriptor TX Packet Control */
#define DMA_RX_TBS2     0x1FFF0000      // Transmit buffer 2 size
#define DMA_RX_TBS1     0x00001FFF      // Transmit buffer 1 size

/* RDES0 - DMA Descriptor RX Packet Status */
#define DMA_RX_OWN      0x80000000      // Own bit 1=DMA,0=CPU
#define DMA_RX_AFM      0x40000000      // Destination address filter fail
#define DMA_RX_FL       0x3FFF0000      // Frame length mask
#define DMA_RX_ES       0x00008000      // Error summary
#define DMA_RX_DE       0x00004000      // Descriptor error
#define DMA_RX_SAF      0x00002000      // Source address filter fail
#define DMA_RX_LE       0x00001000      // Length error
#define DMA_RX_OE       0x00000800      // Overflow error
#define DMA_RX_VLAN     0x00000400      // VLAN tag
#define DMA_RX_FS       0x00000200      // First descriptor
#define DMA_RX_LS       0x00000100      // Last descriptor
#define DMA_RX_IPHCE    0x00000080      // IPv4 header checksum error
#define DMA_RX_LC       0x00000040      // late collision
#define DMA_RX_FT       0x00000020      // Frame type
#define DMA_RX_RWT      0x00000010      // Receive watchdog timeout
#define DMA_RX_RE       0x00000008      // Receive error
#define DMA_RX_DRE      0x00000004      // Dribble bit error
#define DMA_RX_CE       0x00000002      // CRC error
#define DMA_RX_RMAM     0x00000001      // Rx MAC adr.match/payload cks.error

/* RDES1 - DMA Descriptor RX Packet Control */
#define DMA_RX_DIC      0x80000000      // Disable interrupt on completition
#define DMA_RX_RBS2     0x1FFF0000      // Receive buffer 2 size
#define DMA_RX_RER      0x00008000      // Receove end of ring
#define DMA_RX_RCH      0x00004000      // Second address chained
#define DMA_RX_RBS1     0x00001FFF      // Receive buffer 1 size

typedef struct _ETH_PIN {
  GPIO_TypeDef *port;
  uint16_t      pin;
} ETH_PIN;

#endif /* __EMAC_STM32F4XX_H */
