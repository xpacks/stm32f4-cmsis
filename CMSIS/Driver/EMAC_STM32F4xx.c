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
 * $Date:        10. November 2014
 * $Revision:    V2.01
 *  
 * Driver:       Driver_ETH_MAC0
 * Configured:   via RTE_Device.h configuration file 
 * Project:      Ethernet Media Access (MAC) Driver for STM32F4xx
 * ---------------------------------------------------------------------- 
 * Use the following configuration settings in the middleware component
 * to connect to this driver.
 * 
 *   Configuration Setting                 Value
 *   ---------------------                 -----
 *   Connect to hardware via Driver_ETH_MAC# = 0
 * -------------------------------------------------------------------- */

/* History:
 *  Version 2.01
 *    STM32CubeMX generated code can also be used to configure the driver
 *  Version 2.00
 *    Updated to the CMSIS Driver API V2.01
 *  Version 1.10
 *    Based on API V2.00
 *  Version 1.02
 *    Based on API V1.10 (namespace prefix ARM_ added)
 *  Version 1.01
 *    Added checksum offload
 *    Added multicast MAC address filtering 
 *  Version 1.00
 *    Initial release
 */

/* STM32CubeMX configuration:
 *
 * Pinout tab:
 *   - Select ETH peripheral and select MII or RMII mode
 * Clock Configuration tab:
 *   - Ensure that HCLK is at least 25MHz or above
 *   - Ensure thet SYSCLK is at least 50MHz or above if Ethernet PTP is required
 * Configuration tab:
 *   - Select ETH under Connectivity section which opens ETH Configuration window:
 *       - Parameter Settings tab: settings are unused by this driver
 *       - NVIC Settings: enable Ethernet global interrupt
 *       - GPIO Settings: configure as needed
 */

#include "EMAC_STM32F4xx.h"

#define ARM_ETH_MAC_DRV_VERSION ARM_DRIVER_VERSION_MAJOR_MINOR(2,01) /* driver version */


/* Receive/transmit Checksum offload enable */
#ifndef EMAC_CHECKSUM_OFFLOAD
#define EMAC_CHECKSUM_OFFLOAD   1
#endif

/* IEEE 1588 time stamping enable (PTP) */
#ifndef EMAC_TIME_STAMP
#define EMAC_TIME_STAMP         0
#endif

/* Timeouts */
#define PHY_TIMEOUT         2U          /* PHY Register access timeout in ms  */

/* ETH Memory Buffer configuration */
#define NUM_RX_BUF          4U          /* 0x1800 for Rx (4*1536=6K)          */
#define NUM_TX_BUF          2U          /* 0x0C00 for Tx (2*1536=3K)          */
#define ETH_BUF_SIZE        1536U       /* ETH Receive/Transmit buffer size   */

/* Interrupt Handler Prototype */
void ETH_IRQHandler (void);

#if defined(RTE_DEVICE_FRAMEWORK_CUBE_MX)
extern ETH_HandleTypeDef heth;
#endif

/* DMA RX Descriptor */
typedef struct rx_desc {
  uint32_t volatile Stat;
  uint32_t          Ctrl;
  uint8_t  const   *Addr;
  struct rx_desc   *Next;
#if ((EMAC_CHECKSUM_OFFLOAD != 0) || (EMAC_TIME_STAMP != 0))
  uint32_t          ExtStat;
  uint32_t          Reserved[1];
  uint32_t          TimeLo;
  uint32_t          TimeHi;
#endif
} RX_Desc;

/* DMA TX Descriptor */
typedef struct tx_desc {
  uint32_t volatile CtrlStat;
  uint32_t          Size;
  uint8_t          *Addr;
  struct tx_desc   *Next;
#if ((EMAC_CHECKSUM_OFFLOAD != 0) || (EMAC_TIME_STAMP != 0))
  uint32_t          Reserved[2];
  uint32_t          TimeLo;
  uint32_t          TimeHi;
#endif
} TX_Desc;

/* Ethernet Pin definitions */
static const ETH_PIN eth_pins[] = {
  { MX_ETH_MDC_GPIOx,  MX_ETH_MDC_GPIO_Pin  },        /* MII, RMII */
  { MX_ETH_MDIO_GPIOx, MX_ETH_MDIO_GPIO_Pin },        /* MII, RMII */
  { MX_ETH_TXD0_GPIOx, MX_ETH_TXD0_GPIO_Pin },        /* MII, RMII */
  { MX_ETH_TXD1_GPIOx, MX_ETH_TXD1_GPIO_Pin },        /* MII, RMII */
  { MX_ETH_RXD0_GPIOx, MX_ETH_RXD0_GPIO_Pin },        /* MII, RMII */
  { MX_ETH_RXD1_GPIOx, MX_ETH_RXD1_GPIO_Pin },        /* MII, RMII */
  { MX_ETH_TX_EN_GPIOx, MX_ETH_TX_EN_GPIO_Pin },      /* MII, RMII */
#if (ETH_MII)
  { MX_ETH_TXD2_GPIOx, MX_ETH_TXD2_GPIO_Pin },        /* MII, ---- */
  { MX_ETH_TXD3_GPIOx, MX_ETH_TXD3_GPIO_Pin },        /* MII, ---- */
  { MX_ETH_RXD2_GPIOx, MX_ETH_RXD2_GPIO_Pin },        /* MII, ---- */
  { MX_ETH_RXD3_GPIOx, MX_ETH_RXD3_GPIO_Pin },        /* MII, ---- */
  { MX_ETH_TX_CLK_GPIOx, MX_ETH_TX_CLK_GPIO_Pin },    /* MII, ---- */
  { MX_ETH_RX_CLK_GPIOx, MX_ETH_RX_CLK_GPIO_Pin },    /* MII, ---- */ 
  { MX_ETH_CRS_GPIOx,   MX_ETH_CRS_GPIO_Pin },        /* MII, ---- */
  { MX_ETH_COL_GPIOx, MX_ETH_COL_GPIO_Pin },          /* MII, ---- */
  { MX_ETH_RX_DV_GPIOx, MX_ETH_RX_DV_GPIO_Pin },      /* MII, ---- */ 
  { MX_ETH_RX_ER_GPIOx, MX_ETH_RX_ER_GPIO_Pin },      /* MII, ---- */
#else
  { MX_ETH_CRS_DV_GPIOx, MX_ETH_CRS_DV_GPIO_Pin },    /* ---, RMII */
  { MX_ETH_REF_CLK_GPIOx, MX_ETH_REF_CLK_GPIO_Pin },  /* ---, RMII */
#endif
};

/* Driver Version */
static const ARM_DRIVER_VERSION DriverVersion = {
  ARM_ETH_MAC_API_VERSION,
  ARM_ETH_MAC_DRV_VERSION
};

/* Driver Capabilities */
static const ARM_ETH_MAC_CAPABILITIES DriverCapabilities = {
  (EMAC_CHECKSUM_OFFLOAD != 0) ? 1U : 0U,   /* checksum_offload_rx_ip4  */
  (EMAC_CHECKSUM_OFFLOAD != 0) ? 1U : 0U,   /* checksum_offload_rx_ip6  */
  (EMAC_CHECKSUM_OFFLOAD != 0) ? 1U : 0U,   /* checksum_offload_rx_udp  */
  (EMAC_CHECKSUM_OFFLOAD != 0) ? 1U : 0U,   /* checksum_offload_rx_tcp  */
  (EMAC_CHECKSUM_OFFLOAD != 0) ? 1U : 0U,   /* checksum_offload_rx_icmp */
  (EMAC_CHECKSUM_OFFLOAD != 0) ? 1U : 0U,   /* checksum_offload_tx_ip4  */
  (EMAC_CHECKSUM_OFFLOAD != 0) ? 1U : 0U,   /* checksum_offload_tx_ip6  */
  (EMAC_CHECKSUM_OFFLOAD != 0) ? 1U : 0U,   /* checksum_offload_tx_udp  */
  (EMAC_CHECKSUM_OFFLOAD != 0) ? 1U : 0U,   /* checksum_offload_tx_tcp  */
  (EMAC_CHECKSUM_OFFLOAD != 0) ? 1U : 0U,   /* checksum_offload_tx_icmp */
  (ETH_MII != 0) ?
  ARM_ETH_INTERFACE_MII :
  ARM_ETH_INTERFACE_RMII,                   /* media_interface          */
  0U,                                       /* mac_address              */
  1U,                                       /* event_rx_frame           */
  1U,                                       /* event_tx_frame           */
  1U,                                       /* event_wakeup             */
  (EMAC_TIME_STAMP != 0) ? 1U : 0U          /* precision_timer          */
};

/* Local variables */
static ARM_ETH_MAC_SignalEvent_t cb_Event;
static uint8_t                   tx_index;
static uint8_t                   rx_index;
#if (EMAC_CHECKSUM_OFFLOAD)
  static bool                    tx_cks_offload;
#endif
#if (EMAC_TIME_STAMP)
  static uint8_t                 tx_ts_index;
#endif
static RX_Desc                   rx_desc[NUM_RX_BUF];
static TX_Desc                   tx_desc[NUM_TX_BUF];
static uint32_t                  rx_buf [NUM_RX_BUF][ETH_BUF_SIZE>>2];
static uint32_t                  tx_buf [NUM_TX_BUF][ETH_BUF_SIZE>>2];


#if defined(RTE_DEVICE_FRAMEWORK_CLASSIC)
/**
  \fn          void Enable_GPIO_Clock (const GPIO_TypeDef *port)
  \brief       Enable GPIO clock
*/
static void Enable_GPIO_Clock (const GPIO_TypeDef *GPIOx) {
  if      (GPIOx == GPIOA) { __GPIOA_CLK_ENABLE(); }
  else if (GPIOx == GPIOB) { __GPIOB_CLK_ENABLE(); }
  else if (GPIOx == GPIOC) { __GPIOC_CLK_ENABLE(); }
  else if (GPIOx == GPIOD) { __GPIOD_CLK_ENABLE(); }
#if (!defined(STM32F401xC)) && (!defined(STM32F401xE)) && (!defined(STM32F411xE))
  else if (GPIOx == GPIOF) { __GPIOF_CLK_ENABLE(); }
  else if (GPIOx == GPIOG) { __GPIOG_CLK_ENABLE(); }
#endif
  else if (GPIOx == GPIOH) { __GPIOH_CLK_ENABLE(); }
  else if (GPIOx == GPIOI) { __GPIOI_CLK_ENABLE(); }
#if defined(STM32F427xx) || defined(STM32F437xx) || defined(STM32F429xx) || defined(STM32F439xx)
  else if (GPIOx == GPIOJ) { __GPIOJ_CLK_ENABLE(); }
  else if (GPIOx == GPIOK) { __GPIOK_CLK_ENABLE(); }
#endif
  else /* GPIOx == GPIOE */{ __GPIOE_CLK_ENABLE(); }
}
#endif

/**
  \fn          void init_rx_desc (void)
  \brief       Initialize Rx DMA descriptors.
  \return      none.
*/
static void init_rx_desc (void) {
  uint32_t i,next;

  for (i = 0U; i < NUM_RX_BUF; i++) {
    rx_desc[i].Stat = DMA_RX_OWN;
    rx_desc[i].Ctrl = DMA_RX_RCH | ETH_BUF_SIZE;
    rx_desc[i].Addr = (uint8_t *)&rx_buf[i];
    next = i + 1U;
    if (next == NUM_RX_BUF) { next = 0U; }
    rx_desc[i].Next = &rx_desc[next];
  }

  ETH->DMARDLAR = (uint32_t)&rx_desc[0];
  rx_index      = 0U;
}

/**
  \fn          void init_tx_desc (void)
  \brief       Initialize Tx DMA descriptors.
  \return      none.
*/
static void init_tx_desc (void) {
  uint32_t i,next;

  for (i = 0; i < NUM_TX_BUF; i++) {
    tx_desc[i].CtrlStat = DMA_TX_TCH | DMA_TX_LS | DMA_TX_FS;
    tx_desc[i].Addr     = (uint8_t *)&tx_buf[i];
    next = i + 1U;
    if (next == NUM_TX_BUF) { next = 0U; }
    tx_desc[i].Next     = &tx_desc[next];
  }
  ETH->DMATDLAR = (uint32_t)&tx_desc[0];
  tx_index      = 0U;
}

/**
  \fn          uint32_t crc32_8bit_rev (uint32_t crc32, uint8_t val)
  \brief       Calculate 32-bit CRC (Polynom: 0x04C11DB7, data bit-reversed).
  \param[in]   crc32  CRC initial value
  \param[in]   val    Input value
  \return      Calculated CRC value
*/
static uint32_t crc32_8bit_rev (uint32_t crc32, uint8_t val) {
  uint32_t n;

  crc32 ^= __RBIT (val);
  for (n = 8; n; n--) {
    if (crc32 & 0x80000000U) {
      crc32 <<= 1;
      crc32  ^= 0x04C11DB7U;
    } else {
      crc32 <<= 1;
    }
  }
  return (crc32);
}

/**
  \fn          uint32_t crc32_data (const uint8_t *data, uint32_t len)
  \brief       Calculate standard 32-bit Ethernet CRC.
  \param[in]   data  Pointer to buffer containing the data
  \param[in]   len   Data length in bytes
  \return      Calculated CRC value
*/
static uint32_t crc32_data (const uint8_t *data, uint32_t len) {
  uint32_t cnt, crc;

  crc = 0xFFFFFFFFU;
  for (cnt = len; cnt; cnt--) {
    crc = crc32_8bit_rev (crc, *data++);
  }
  return (crc ^ 0xFFFFFFFFU);
}


/* Ethernet Driver functions */

/**
  \fn          ARM_DRIVER_VERSION ARM_ETH_MAC_GetVersion (void)
  \brief       Get driver version.
  \return      \ref ARM_DRIVER_VERSION
*/
static ARM_DRIVER_VERSION GetVersion (void) {
  return DriverVersion;
}


/**
  \fn          ARM_ETH_MAC_CAPABILITIES GetCapabilities (void)
  \brief       Get driver capabilities.
  \return      \ref ARM_ETH_MAC_CAPABILITIES
*/
static ARM_ETH_MAC_CAPABILITIES GetCapabilities (void) {
  return DriverCapabilities;
}


/**
  \fn          int32_t Initialize (ARM_ETH_MAC_SignalEvent_t cb_event)
  \brief       Initialize Ethernet MAC Device.
  \param[in]   cb_event  Pointer to \ref ARM_ETH_MAC_SignalEvent
  \return      \ref execution_status
*/
static int32_t Initialize (ARM_ETH_MAC_SignalEvent_t cb_event) {
#if defined(RTE_DEVICE_FRAMEWORK_CLASSIC)
  GPIO_InitTypeDef GPIO_InitStruct;
  const ETH_PIN *io;
#endif

  /* Register driver callback function */
  cb_Event = cb_event;

  /* Enable SYSCFG clock */
  RCC->APB2ENR |= RCC_APB2ENR_SYSCFGEN;

  #if (ETH_MII == 0)
  SYSCFG->PMC |=  SYSCFG_PMC_MII_RMII_SEL;
  #else
  SYSCFG->PMC &= ~SYSCFG_PMC_MII_RMII_SEL;
  #endif

  #if defined(RTE_DEVICE_FRAMEWORK_CLASSIC)
    /* Configure Ethernet pins */
    GPIO_InitStruct.Mode      = GPIO_MODE_AF_PP;
    GPIO_InitStruct.Pull      = GPIO_NOPULL;
    GPIO_InitStruct.Speed     = GPIO_SPEED_HIGH;
    GPIO_InitStruct.Alternate = GPIO_AF11_ETH;

    for (io = eth_pins; io != &eth_pins[sizeof(eth_pins)/sizeof(ETH_PIN)]; io++) {
      Enable_GPIO_Clock (io->port);
      GPIO_InitStruct.Pin = io->pin;
      HAL_GPIO_Init (io->port, &GPIO_InitStruct);
    }
    NVIC_EnableIRQ(ETH_IRQn);
  #else
    heth.Instance = ETH;

    HAL_ETH_MspInit (&heth);
  #endif

  return ARM_DRIVER_OK;
}

/**
  \fn          int32_t Uninitialize (void)
  \brief       De-initialize Ethernet MAC Device.
  \return      \ref execution_status
*/
static int32_t Uninitialize (void) {
  const ETH_PIN *io;

  /* Disable ethernet interrupts */
  ETH->DMAIER  = 0U;
  ETH->PTPTSCR = 0U;
  NVIC_DisableIRQ(ETH_IRQn);

  /* Unconfigure ethernet pins */
  for (io = eth_pins; io != &eth_pins[sizeof(eth_pins)/sizeof(ETH_PIN)]; io++) {
    HAL_GPIO_DeInit(io->port, io->pin);

  }

  return ARM_DRIVER_OK;
}

/**
  \fn          int32_t PowerControl (ARM_POWER_STATE state)
  \brief       Control Ethernet MAC Device Power.
  \param[in]   state  Power state
  \return      \ref execution_status
*/
static int32_t PowerControl (ARM_POWER_STATE state) {
  uint32_t hclk, clkdiv;

  switch (state) {
    case ARM_POWER_OFF:
      /* Reset Ethernet MAC peripheral */
      __ETHMAC_FORCE_RESET();
      __NOP(); __NOP(); __NOP(); __NOP();
      __ETHMAC_RELEASE_RESET();

      /* Disable Ethernet clocks */
      __ETHMAC_CLK_DISABLE();
      __ETHMACTX_CLK_DISABLE();
      __ETHMACRX_CLK_DISABLE();
      #if (EMAC_TIME_STAMP)
      __ETHMACPTP_CLK_DISABLE();
      #endif

      break;

    case ARM_POWER_LOW:
      return ARM_DRIVER_ERROR_UNSUPPORTED;

    case ARM_POWER_FULL:
      /* Enable Ethernet clocks */
      __ETHMAC_CLK_ENABLE();
      __ETHMACTX_CLK_ENABLE();
      __ETHMACRX_CLK_ENABLE();
      #if (EMAC_TIME_STAMP)
      __ETHMACPTP_CLK_ENABLE();
      #endif

      /* Soft reset MAC DMA controller */
      ETH->DMABMR |= ETH_DMABMR_SR;
      while (ETH->DMABMR & ETH_DMABMR_SR) {
        ; /* Wait until software reset completed */
      }

      #if ((EMAC_CHECKSUM_OFFLOAD != 0) || (EMAC_TIME_STAMP != 0))
      /* Enhanced descriptor enable */
      ETH->DMABMR |= ETH_DMABMR_EDE;
      #endif

      /* MDC clock range selection */
      hclk = HAL_RCC_GetHCLKFreq();
      
      if (hclk >= 150000000U) {
        clkdiv = ETH_MACMIIAR_CR_Div102;
      } else if (hclk >= 100000000U) {
        clkdiv = ETH_MACMIIAR_CR_Div62;
      } else if (hclk >= 60000000U) {
        clkdiv = ETH_MACMIIAR_CR_Div42;
      } else if (hclk >= 35000000U) {
        clkdiv = ETH_MACMIIAR_CR_Div26;
      } else if (hclk >= 25000000U) {
        clkdiv = ETH_MACMIIAR_CR_Div16;
      } else {
        /* HCLK is too slow for Ethernet */
        return (ARM_DRIVER_ERROR);
      }
      ETH->MACMIIAR = clkdiv;

      /* Initialize MAC configuration */
      ETH->MACCR   = ETH_MACCR_ROD | 0x00008000U;

      /* Initialize Filter registers */
      ETH->MACFFR  = ETH_MACFFR_BFD;
      ETH->MACFCR  = ETH_MACFCR_ZQPD;

      /* Initialize Address registers */
      ETH->MACA0HR = 0U; ETH->MACA0LR = 0U;
      ETH->MACA1HR = 0U; ETH->MACA1LR = 0U;
      ETH->MACA2HR = 0U; ETH->MACA2LR = 0U;
      ETH->MACA3HR = 0U; ETH->MACA3LR = 0U;

      /* Initialize DMA Descriptors */
      init_rx_desc ();
      init_tx_desc ();

      /* Enable Rx interrupts */
      ETH->DMASR  = 0xFFFFFFFFU;
      ETH->DMAIER = ETH_DMAIER_NISE | ETH_DMAIER_RIE | ETH_DMAIER_TIE;

      #if (EMAC_TIME_STAMP)
      ETH->PTPTSCR = ETH_PTPTSSR_TSSIPV4FE | ETH_PTPTSSR_TSSIPV6FE |
                     ETH_PTPTSSR_TSSSR     | ETH_PTPTSCR_TSE;
      ETH->PTPSSIR = PTPSSIR_Val(HAL_RCC_GetHCLKFreq());
      tx_ts_index  = 0U;
      #endif
      break;

    default:
      return ARM_DRIVER_ERROR_UNSUPPORTED;
  }
  
  return ARM_DRIVER_OK;
}

/**
  \fn          int32_t GetMacAddress (ARM_ETH_MAC_ADDR *ptr_addr)
  \brief       Get Ethernet MAC Address.
  \param[in]   ptr_addr  Pointer to address
  \return      \ref execution_status
*/
static int32_t GetMacAddress (ARM_ETH_MAC_ADDR *ptr_addr) {
  uint32_t val;

  if (ptr_addr == NULL) {
    return ARM_DRIVER_ERROR_PARAMETER;
  }

  val = ETH->MACA0HR;
  ptr_addr->b[5] = (uint8_t)(val >>  8);
  ptr_addr->b[4] = (uint8_t)(val);
  val = ETH->MACA0LR;
  ptr_addr->b[3] = (uint8_t)(val >> 24);
  ptr_addr->b[2] = (uint8_t)(val >> 16);
  ptr_addr->b[1] = (uint8_t)(val >>  8);
  ptr_addr->b[0] = (uint8_t)(val);

  return ARM_DRIVER_OK;
}

/**
  \fn          int32_t SetMacAddress (const ARM_ETH_MAC_ADDR *ptr_addr)
  \brief       Set Ethernet MAC Address.
  \param[in]   ptr_addr  Pointer to address
  \return      \ref execution_status
*/
static int32_t SetMacAddress (const ARM_ETH_MAC_ADDR *ptr_addr) {

  if (ptr_addr == NULL) {
    return ARM_DRIVER_ERROR_PARAMETER;
  }

  /* Set Ethernet MAC Address registers */
  ETH->MACA0HR = ((uint32_t)ptr_addr->b[5] <<  8) |  (uint32_t)ptr_addr->b[4];
  ETH->MACA0LR = ((uint32_t)ptr_addr->b[3] << 24) | ((uint32_t)ptr_addr->b[2] << 16) |
                 ((uint32_t)ptr_addr->b[1] <<  8) |  (uint32_t)ptr_addr->b[0];

  return ARM_DRIVER_OK;
}

/**
  \fn          int32_t SetAddressFilter (const ARM_ETH_MAC_ADDR *ptr_addr,
                                               uint32_t          num_addr)
  \brief       Configure Address Filter.
  \param[in]   ptr_addr  Pointer to addresses
  \param[in]   num_addr  Number of addresses to configure
  \return      \ref execution_status
*/
static int32_t SetAddressFilter (const ARM_ETH_MAC_ADDR *ptr_addr, uint32_t num_addr) {
  uint32_t crc;

  if ((ptr_addr == NULL) && (num_addr != 0)) {
    return ARM_DRIVER_ERROR_PARAMETER;
  }

  /* Use unicast address filtering for first 3 MAC addresses */
  ETH->MACFFR &= ~(ETH_MACFFR_HPF | ETH_MACFFR_HM);
  ETH->MACHTHR = 0U; ETH->MACHTLR = 0U;

  if (num_addr == 0U) {
    ETH->MACA1HR = 0U; ETH->MACA1LR = 0U;
    ETH->MACA2HR = 0U; ETH->MACA2LR = 0U;
    ETH->MACA3HR = 0U; ETH->MACA3LR = 0U;
    return ARM_DRIVER_OK;
  }

  ETH->MACA1HR = ((uint32_t)ptr_addr->b[5] <<  8) |  (uint32_t)ptr_addr->b[4] | ETH_MACA1HR_AE;
  ETH->MACA1LR = ((uint32_t)ptr_addr->b[3] << 24) | ((uint32_t)ptr_addr->b[2] << 16) |
                 ((uint32_t)ptr_addr->b[1] <<  8) |  (uint32_t)ptr_addr->b[0];
  num_addr--;
  if (num_addr == 0U) {
    ETH->MACA2HR = 0U; ETH->MACA2LR = 0U;
    ETH->MACA3HR = 0U; ETH->MACA3LR = 0U;
    return ARM_DRIVER_OK;
  }
  ptr_addr++;

  ETH->MACA2HR = ((uint32_t)ptr_addr->b[5] <<  8) |  (uint32_t)ptr_addr->b[4] | ETH_MACA2HR_AE;
  ETH->MACA2LR = ((uint32_t)ptr_addr->b[3] << 24) | ((uint32_t)ptr_addr->b[2] << 16) |
                 ((uint32_t)ptr_addr->b[1] <<  8) |  (uint32_t)ptr_addr->b[0];
  num_addr--;
  if (num_addr == 0U) {
    ETH->MACA3HR = 0U; ETH->MACA3LR = 0U;
    return ARM_DRIVER_OK;
  }
  ptr_addr++;

  ETH->MACA3HR = ((uint32_t)ptr_addr->b[5] <<  8) |  (uint32_t)ptr_addr->b[4] | ETH_MACA3HR_AE;
  ETH->MACA3LR = ((uint32_t)ptr_addr->b[3] << 24) | ((uint32_t)ptr_addr->b[2] << 16) |
                 ((uint32_t)ptr_addr->b[1] <<  8) |  (uint32_t)ptr_addr->b[0];
  num_addr--;
  if (num_addr == 0U) {
    return ARM_DRIVER_OK;
  }
  ptr_addr++;

  /* Calculate 64-bit Hash table for remaining MAC addresses */
  for ( ; num_addr; ptr_addr++, num_addr--) {
    crc = crc32_data (&ptr_addr->b[0], 6U) >> 26;
    if (crc & 0x20U) {
      ETH->MACHTHR |= (1U << (crc & 0x1FU));
    }
    else {
      ETH->MACHTLR |= (1U << crc);
    }
  }
  /* Enable both, unicast and hash address filtering */
  ETH->MACFFR |= ETH_MACFFR_HPF | ETH_MACFFR_HM;

  return ARM_DRIVER_OK;
}

/**
  \fn          int32_t SendFrame (const uint8_t *frame, uint32_t len, uint32_t flags)
  \brief       Send Ethernet frame.
  \param[in]   frame  Pointer to frame buffer with data to send
  \param[in]   len    Frame buffer length in bytes
  \param[in]   flags  Frame transmit flags (see ARM_ETH_MAC_TX_FRAME_...)
  \return      \ref execution_status
*/
static int32_t SendFrame (const uint8_t *frame, uint32_t len, uint32_t flags) {
  static uint8_t *frame_end = NULL;
  uint8_t *dst = frame_end;
  uint32_t ctrl;

  if ((frame == NULL) || (len == 0U)) {
    return ARM_DRIVER_ERROR_PARAMETER;
  }

  if (dst == NULL) {
    /* Start of a new transmit frame */
    if (tx_desc[tx_index].CtrlStat & DMA_TX_OWN) {
      /* Transmitter is busy, wait */
      return ARM_DRIVER_ERROR_BUSY;
    }
    dst = tx_desc[tx_index].Addr;
    tx_desc[tx_index].Size = len;
  }
  else {
    /* Sending data fragments in progress */
    tx_desc[tx_index].Size += len;
  }
  /* Fast-copy data fragments to ETH-DMA buffer */
  for ( ; len > 7U; dst += 8, frame += 8, len -= 8U) {
    ((__packed uint32_t *)dst)[0] = ((__packed uint32_t *)frame)[0];
    ((__packed uint32_t *)dst)[1] = ((__packed uint32_t *)frame)[1];
  }
  /* Copy remaining 7 bytes */
  for ( ; len > 1U; dst += 2, frame += 2, len -= 2U) {
    ((__packed uint16_t *)dst)[0] = ((__packed uint16_t *)frame)[0];
  }
  if (len > 0U) { dst++[0] = frame++[0]; }

  if (flags & ARM_ETH_MAC_TX_FRAME_FRAGMENT) {
    /* More data to come, remember current write position */
    frame_end = dst;
    return ARM_DRIVER_OK;
  }

  /* Frame is now ready, send it to DMA */
  ctrl = tx_desc[tx_index].CtrlStat & ~DMA_TX_CIC;
#if (EMAC_CHECKSUM_OFFLOAD != 0)
  if (tx_cks_offload) { ctrl |= DMA_TX_CIC; }
#endif
  ctrl &= ~(DMA_TX_IC | DMA_TX_TTSE);
  if (flags & ARM_ETH_MAC_TX_FRAME_EVENT)     { ctrl |= DMA_TX_IC; }
#if (EMAC_TIME_STAMP != 0)
  if (flags & ARM_ETH_MAC_TX_FRAME_TIMESTAMP) { ctrl |= DMA_TX_TTSE; }
  tx_ts_index = tx_index;
#endif
  tx_desc[tx_index].CtrlStat = ctrl | DMA_TX_OWN;

  tx_index++;
  if (tx_index == NUM_TX_BUF) { tx_index = 0U; }
  frame_end = NULL;

  /* Start frame transmission */
  ETH->DMASR   = ETH_DMASR_TPSS;
  ETH->DMATPDR = 0U;

  return ARM_DRIVER_OK;
}

/**
  \fn          int32_t ReadFrame (uint8_t *frame, uint32_t len)
  \brief       Read data of received Ethernet frame.
  \param[in]   frame  Pointer to frame buffer for data to read into
  \param[in]   len    Frame buffer length in bytes
  \return      number of data bytes read or execution status
                 - value >= 0: number of data bytes read
                 - value < 0: error occurred, value is execution status as defined with \ref execution_status 
*/
static int32_t ReadFrame (uint8_t *frame, uint32_t len) {
  uint8_t const *src = rx_desc[rx_index].Addr;

  /* Fast-copy data to frame buffer */
  for ( ; len > 7U; frame += 8, src += 8, len -= 8U) {
    ((__packed uint32_t *)frame)[0] = ((uint32_t *)src)[0];
    ((__packed uint32_t *)frame)[1] = ((uint32_t *)src)[1];
  }
  /* Copy remaining 7 bytes */
  for ( ; len > 1U; frame += 2, src += 2, len -= 2U) {
    ((__packed uint16_t *)frame)[0] = ((uint16_t *)src)[0];
  }
  if (len > 0U) { frame[0] = src[0]; }

  /* Return this block back to ETH-DMA */
  rx_desc[rx_index].Stat = DMA_RX_OWN;

  rx_index++;
  if (rx_index == NUM_RX_BUF) { rx_index = 0; }

  if (ETH->DMASR & ETH_DMASR_RBUS) {
    /* Receive buffer unavailable, resume DMA */
    ETH->DMASR   = ETH_DMASR_RBUS;
    ETH->DMARPDR = 0;
  }
  return ((int32_t)len);
}

/**
  \fn          uint32_t GetRxFrameSize (void)
  \brief       Get size of received Ethernet frame.
  \return      number of bytes in received frame
*/
static uint32_t GetRxFrameSize (void) {
  uint32_t stat = rx_desc[rx_index].Stat;

  if (stat & DMA_RX_OWN) {
    /* Owned by DMA */
    return (0U);
  }
  if (((stat & DMA_RX_ES) != 0) ||
      ((stat & DMA_RX_FS) == 0) ||
      ((stat & DMA_RX_LS) == 0)) {
    /* Error, this block is invalid */
    return (0xFFFFFFFFU);
  }
  return (((stat & DMA_RX_FL) >> 16) - 4U);
}

/* Ethernet IRQ Handler */
void ETH_IRQHandler (void) {
  uint32_t dmasr, macsr, event = 0;

  dmasr = ETH->DMASR;
  ETH->DMASR = dmasr & (ETH_DMASR_NIS | ETH_DMASR_RS | ETH_DMASR_TS);
  if (dmasr & ETH_DMASR_TS)   { event |= ARM_ETH_MAC_EVENT_TX_FRAME; }
  if (dmasr & ETH_DMASR_RS)   { event |= ARM_ETH_MAC_EVENT_RX_FRAME; }
  macsr = ETH->MACSR;
#if (EMAC_TIME_STAMP != 0)
  if (macsr & ETH_MACSR_TSTS) { event |= ARM_ETH_MAC_EVENT_TIMER_ALARM; }
#endif
  if (macsr & ETH_MACSR_PMTS) {
    ETH->MACPMTCSR;
    event |= ARM_ETH_MAC_EVENT_WAKEUP;
  }
  cb_Event (event);
}

/**
  \fn          int32_t GetRxFrameTime (ARM_ETH_MAC_TIME *time)
  \brief       Get time of received Ethernet frame.
  \param[in]   time  Pointer to time structure for data to read into
  \return      \ref execution_status
*/
static int32_t GetRxFrameTime (ARM_ETH_MAC_TIME *time) {
#if (EMAC_TIME_STAMP)
  RX_Desc *rxd = &rx_desc[rx_index];

  if (rxd->Stat & DMA_RX_OWN) {
    /* Owned by DMA */
    return ARM_DRIVER_ERROR_BUSY;
  }
  time->ns  = rxd->TimeLo;
  time->sec = rxd->TimeHi;

  return ARM_DRIVER_OK;
#else
  return ARM_DRIVER_ERROR;
#endif
}

/**
  \fn          int32_t GetTxFrameTime (ARM_ETH_MAC_TIME *time)
  \brief       Get time of transmitted Ethernet frame.
  \param[in]   time  Pointer to time structure for data to read into
  \return      \ref execution_status
*/
static int32_t GetTxFrameTime (ARM_ETH_MAC_TIME *time) {
#if (EMAC_TIME_STAMP)
  TX_Desc *txd = &tx_desc[tx_ts_index];

  if (txd->CtrlStat & DMA_RX_OWN) {
    /* Owned by DMA */
    return ARM_DRIVER_ERROR_BUSY;
  }
  if ((txd->CtrlStat & DMA_TX_TTSS) == 0) {
    /* No transmit time stamp available */
    return ARM_DRIVER_ERROR;
  }
  time->ns  = txd->TimeLo;
  time->sec = txd->TimeHi;
  return ARM_DRIVER_OK;
#else
  return ARM_DRIVER_ERROR;
#endif
}

/**
  \fn          int32_t Control (uint32_t control, uint32_t arg)
  \brief       Control Ethernet Interface.
  \param[in]   control  operation
  \param[in]   arg      argument of operation (optional)
  \return      \ref execution_status
*/
static int32_t Control (uint32_t control, uint32_t arg) {
  uint32_t maccr;
  uint32_t dmaomr;
  uint32_t macffr;

  switch (control) {
    case ARM_ETH_MAC_CONFIGURE:
      maccr = ETH->MACCR & ~(ETH_MACCR_FES | ETH_MACCR_DM   |
                             ETH_MACCR_LM  | ETH_MACCR_IPCO);

      /* Configure 100MBit/10MBit mode */
      switch (arg & ARM_ETH_MAC_SPEED_Msk) {
        case ARM_ETH_MAC_SPEED_10M:
#if (ETH_MII == 0)
          /* RMII Half Duplex Colision detection does not work */
          maccr |= ETH_MACCR_DM;
#endif
          break;
        case ARM_ETH_SPEED_100M:
          maccr |= ETH_MACCR_FES;
          break;
        default:
          return ARM_DRIVER_ERROR_UNSUPPORTED;
      }

      /* Confige Half/Full duplex mode */
      switch (arg & ARM_ETH_MAC_DUPLEX_Msk) {
        case ARM_ETH_MAC_DUPLEX_FULL:
          maccr |= ETH_MACCR_DM;
          break;
        case ARM_ETH_MAC_DUPLEX_HALF:
          break;
        default:
          return ARM_DRIVER_ERROR;
      }

      /* Configure loopback mode */
      if (arg & ARM_ETH_MAC_LOOPBACK) {
        maccr |= ETH_MACCR_LM;
      }

      dmaomr = ETH->DMAOMR & ~(ETH_DMAOMR_RSF| ETH_DMAOMR_TSF);
#if (EMAC_CHECKSUM_OFFLOAD != 0)
      /* Enable rx checksum verification */
      if (arg & ARM_ETH_MAC_CHECKSUM_OFFLOAD_RX) {
        maccr  |= ETH_MACCR_IPCO;
        dmaomr |= ETH_DMAOMR_RSF;
      }

      /* Enable tx checksum generation */
      tx_cks_offload = false;
      if (arg & ARM_ETH_MAC_CHECKSUM_OFFLOAD_TX) {
        dmaomr |= ETH_DMAOMR_TSF;
        tx_cks_offload = true;
      }
#else
      if ((arg & ARM_ETH_MAC_CHECKSUM_OFFLOAD_RX) ||
          (arg & ARM_ETH_MAC_CHECKSUM_OFFLOAD_TX)) {
        /* Checksum offload is disabled in the driver */
        return ARM_DRIVER_ERROR;
      }
#endif
      ETH->DMAOMR = dmaomr;
      ETH->MACCR  = maccr;

      macffr = ETH->MACFFR & ~(ETH_MACFFR_PM | ETH_MACFFR_PAM | ETH_MACFFR_BFD);
      /* Enable broadcast frame receive */
      if ((arg & ARM_ETH_MAC_ADDRESS_BROADCAST) == 0) {
        macffr |= ETH_MACFFR_BFD;
      }

      /* Enable all multicast frame receive */
      if (arg & ARM_ETH_MAC_ADDRESS_MULTICAST) {
        macffr |= ETH_MACFFR_PAM;
      }

      /* Enable promiscuous mode (no filtering) */
      if (arg & ARM_ETH_MAC_ADDRESS_ALL) {
        macffr |= ETH_MACFFR_PM;
      }
      ETH->MACFFR = macffr;
      break;

    case ARM_ETH_MAC_CONTROL_TX:
      /* Enable/disable MAC transmitter */
      maccr  = ETH->MACCR  & ~ETH_MACCR_TE;
      dmaomr = ETH->DMAOMR & ~ETH_DMAOMR_ST;
      if (arg != 0) {
        maccr  |= ETH_MACCR_TE;
        dmaomr |= ETH_DMAOMR_ST;
      }
      ETH->MACCR  = maccr;
      ETH->DMAOMR = dmaomr;
      break;

    case ARM_ETH_MAC_CONTROL_RX:
      /* Enable/disable MAC receiver */
      maccr  = ETH->MACCR  & ~ETH_MACCR_RE;
      dmaomr = ETH->DMAOMR & ~ETH_DMAOMR_SR;
      if (arg != 0) {
        maccr  |= ETH_MACCR_RE;
        dmaomr |= ETH_DMAOMR_SR;
      }
      ETH->MACCR  = maccr;
      ETH->DMAOMR = dmaomr;
      break;

    case ARM_ETH_MAC_FLUSH:
      /* Flush tx and rx buffers */
      if (arg & ARM_ETH_MAC_FLUSH_RX) {
      }
      if (arg & ARM_ETH_MAC_FLUSH_TX) {
        ETH->DMAOMR |= ETH_DMAOMR_FTF;
      }
      break;

    case ARM_ETH_MAC_VLAN_FILTER:
      /* Configure VLAN filter */
      ETH->MACVLANTR = arg;
      break;

    default:
      return ARM_DRIVER_ERROR_UNSUPPORTED;
  }
  return ARM_DRIVER_OK;
}

/**
  \fn          int32_t ControlTimer (uint32_t control, ARM_ETH_MAC_TIME *time)
  \brief       Control Precision Timer.
  \param[in]   control  operation
  \param[in]   time     Pointer to time structure
  \return      \ref execution_status
*/
static int32_t ControlTimer (uint32_t control, ARM_ETH_MAC_TIME *time) {
#if (EMAC_TIME_STAMP != 0)
  switch (control) {
    case ARM_ETH_MAC_TIMER_GET_TIME:
      /* Get current time */
      time->sec = ETH->PTPTSHR;
      time->ns  = ETH->PTPTSLR;
      break;

    case ARM_ETH_MAC_TIMER_SET_TIME:
      /* Set new time */
      ETH->PTPTSHUR = time->sec;
      ETH->PTPTSLUR = time->ns;
      /* Initialize TS time */
      ETH->PTPTSCR |= ETH_PTPTSCR_TSSTI;
      break;

    case ARM_ETH_MAC_TIMER_INC_TIME:
      /* Increment current time */
      ETH->PTPTSHUR = time->sec;
      ETH->PTPTSLUR = time->ns;
      /* Coarse TS clock update */
      ETH->PTPTSCR &= ~ETH_PTPTSCR_TSFCU;
      ETH->PTPTSCR |= ETH_PTPTSCR_TSSTI;
      break;

    case ARM_ETH_MAC_TIMER_DEC_TIME:
      /* Decrement current time */
      ETH->PTPTSHUR = time->sec;
      ETH->PTPTSLUR = time->ns | 0x80000000U;
      /* Coarse TS clock update */
      ETH->PTPTSCR &= ~ETH_PTPTSCR_TSFCU;
      ETH->PTPTSCR |=  ETH_PTPTSCR_TSSTI;
      break;

    case ARM_ETH_MAC_TIMER_SET_ALARM:
      /* Set alarm time */
      ETH->PTPTTHR  = time->sec;
      ETH->PTPTTLR  = time->ns;
      /* Enable also PTP interrupts */
      ETH->PTPTSCR |= ETH_PTPTSCR_TSITE;
      break;

    case ARM_ETH_MAC_TIMER_ADJUST_CLOCK:
      /* Adjust current time, fine correction */
      ETH->PTPTSAR = time->ns;
      /* Fine TS clock correction */
      ETH->PTPTSCR |= (ETH_PTPTSCR_TSARU | ETH_PTPTSCR_TSFCU);
      break;

    default:
      return ARM_DRIVER_ERROR_PARAMETER;
  }
  return ARM_DRIVER_OK;
#else
  return ARM_DRIVER_ERROR;
#endif
}

/**
  \fn          int32_t PHY_Read (uint8_t phy_addr, uint8_t reg_addr, uint16_t *data)
  \brief       Read Ethernet PHY Register through Management Interface.
  \param[in]   phy_addr  5-bit device address
  \param[in]   reg_addr  5-bit register address
  \param[out]  data      Pointer where the result is written to
  \return      \ref execution_status
*/
static int32_t PHY_Read (uint8_t phy_addr, uint8_t reg_addr, uint16_t *data) {
  uint32_t val, tick;

  val = ETH->MACMIIAR & ETH_MACMIIAR_CR;

  ETH->MACMIIAR = val | ETH_MACMIIAR_MB | ((uint32_t)phy_addr << 11) |
                                          ((uint32_t)reg_addr <<  6) ;

  /* Wait until operation completed */
  tick = HAL_GetTick();
  do {
    if ((ETH->MACMIIAR & ETH_MACMIIAR_MB) == 0U) { break; }
  } while ((HAL_GetTick() - tick) < PHY_TIMEOUT);

  if ((ETH->MACMIIAR & ETH_MACMIIAR_MB) == 0U) {
    *data = ETH->MACMIIDR & ETH_MACMIIDR_MD;
    return ARM_DRIVER_OK;
  }

  return ARM_DRIVER_ERROR_TIMEOUT;
}

/**
  \fn          int32_t PHY_Write (uint8_t phy_addr, uint8_t reg_addr, uint16_t data)
  \brief       Write Ethernet PHY Register through Management Interface.
  \param[in]   phy_addr  5-bit device address
  \param[in]   reg_addr  5-bit register address
  \param[in]   data      16-bit data to write
  \return      \ref execution_status
*/
static int32_t PHY_Write (uint8_t phy_addr, uint8_t reg_addr, uint16_t data) {
  uint32_t val, tick;

  ETH->MACMIIDR = data;
  val = ETH->MACMIIAR & ETH_MACMIIAR_CR;
  ETH->MACMIIAR = val | ETH_MACMIIAR_MB | ETH_MACMIIAR_MW | ((uint32_t)phy_addr << 11) |
                                                            ((uint32_t)reg_addr <<  6) ;

  /* Wait until operation completed */
  tick = HAL_GetTick();
  do {
    if ((ETH->MACMIIAR & ETH_MACMIIAR_MB) == 0U) { break; }
  } while ((HAL_GetTick() - tick) < PHY_TIMEOUT);

  if ((ETH->MACMIIAR & ETH_MACMIIAR_MB) == 0U) {
    return ARM_DRIVER_OK;
  }

  return ARM_DRIVER_ERROR_TIMEOUT;
}


/* MAC Driver Control Block */
ARM_DRIVER_ETH_MAC Driver_ETH_MAC0 = {
  GetVersion,
  GetCapabilities,
  Initialize,
  Uninitialize,
  PowerControl,
  GetMacAddress,
  SetMacAddress,
  SetAddressFilter,
  SendFrame,
  ReadFrame,
  GetRxFrameSize,
  GetRxFrameTime,
  GetTxFrameTime,
  ControlTimer,
  Control,
  PHY_Read,
  PHY_Write
};
