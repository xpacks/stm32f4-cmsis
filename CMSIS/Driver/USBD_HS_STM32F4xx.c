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
 * $Date:        27. November 2014
 * $Revision:    V2.04
 *
 * Driver:       Driver_USBD0
 * Configured:   via RTE_Device.h configuration file
 * Project:      USB High-Speed Device Driver for ST STM32F4xx
 * --------------------------------------------------------------------------
 * Use the following configuration settings in the middleware component
 * to connect to this driver.
 *
 *   Configuration Setting                Value
 *   ---------------------                -----
 *   Connect to hardware via Driver_USBD# = 1
 * -------------------------------------------------------------------------- */

/* History:
 *  Version 2.04
 *    Reorganized, common endpoint structure for IN and OUT endpoints
 *  Version 2.03
 *    Send data on first NAK to prevent simultaneous write to FIFO from library
 *    and from IRQ routine
 *  Version 2.02
 *    Removed unnecessary FIFO __packed attribute
 *  Version 2.01
 *    Update for USB Device CMSIS Driver API v2.01
 *  Version 2.00
 *    Updated to 2.00 API
 *  Version 1.04
 *    Multiple packet read
 *  Version 1.03
 *    Based on API V1.10 (namespace prefix ARM_ added)
 *  Version 1.02
 *    Removed include of rl_usb.h header
 *  Version 1.00
 *    Initial release
 */
 
  /* STM32CubeMX configuration:
 *
 * Pinout tab:
 *   - Select USB_OTG_HS peripheral and enable mode Device mode for proper PHY
 * Clock Configuration tab:
 *   - Configure clock
 * Configuration tab:
 *   - Select USB_HS under Connectivity section which opens USB_HS Configuration window:
 *       - Parameter Settings tab: settings are unused by this driver
 *       - NVIC Settings: enable USB on The GO HS global interrupt
 *       - GPIO Settings: configure as needed
 */

#include <stdint.h>
#include <string.h>
#include "cmsis_os.h"

#include "Driver_USBD.h"

#include "OTG_HS_STM32F4xx.h"

extern uint8_t otg_hs_role;
extern uint8_t otg_hs_state;

extern bool OTG_HS_PinsConfigure   (uint8_t pins_mask);
extern bool OTG_HS_PinsUnconfigure (uint8_t pins_mask);

#define OTG                         OTG_HS


// USBD Driver *****************************************************************

#define ARM_USBD_DRV_VERSION ARM_DRIVER_VERSION_MAJOR_MINOR(2,4) // USBD driver version

// Driver Version
static const ARM_DRIVER_VERSION usbd_driver_version = { ARM_USBD_API_VERSION, ARM_USBD_DRV_VERSION };

// Driver Capabilities
static const ARM_USBD_CAPABILITIES usbd_driver_capabilities = {
#ifdef ARM_USBD_VBUS_DETECT
  1U,  // vbus_detection
  1U,  // event_vbus_on
  1U,  // event_vbus_off
#else
  0U,  // vbus_detection
  0U,  // event_vbus_on
  0U,  // event_vbus_off
#endif
};

// Number of Endpoints (excluding Endpoint 0, max = 5)
#ifndef USBD_EP_NUM
#define USBD_EP_NUM             5U
#endif

#define OTG_RX_FIFO_SIZE        1152U
#define OTG_TX0_FIFO_SIZE       384U
#define OTG_TX1_FIFO_SIZE       512U
#define OTG_TX2_FIFO_SIZE       512U
#define OTG_TX3_FIFO_SIZE       512U
#define OTG_TX4_FIFO_SIZE       512U
#define OTG_TX5_FIFO_SIZE       512U

#define OTG_TX_FIFO(n)          *((volatile uint32_t*)(OTG_HS_BASE + 0x1000U + n*0x1000U))
#define OTG_RX_FIFO             *((volatile uint32_t*)(OTG_HS_BASE + 0x1000U))

#define OTG_DIEPTSIZ(EPNum)     *(&OTG->DIEPTSIZ0 + EPNum * 8U)
#define OTG_DIEPCTL(EPNum)      *(&OTG->DIEPCTL0  + EPNum * 8U)
#define OTG_DTXFSTS(EPNum)      *(&OTG->DTXFSTS0  + EPNum * 8U)
#define OTG_DOEPTSIZ(EPNum)     *(&OTG->DOEPTSIZ0 + EPNum * 8U)
#define OTG_DOEPCTL(EPNum)      *(&OTG->DOEPCTL0  + EPNum * 8U)
#define OTG_DIEPINT(EPNum)      *(&OTG->DIEPINT0  + EPNum * 8U)
#define OTG_DOEPINT(EPNum)      *(&OTG->DOEPINT0  + EPNum * 8U)

#define OTG_EP_IN_TYPE(num)      ((OTG_DIEPCTL(num) >> 18U) & 3U)
#define OTG_EP_OUT_TYPE(num)     ((OTG_DOEPCTL(num) >> 18U) & 3U)

// USB Device Endpoint flags
#define USBD_EP_FLAG_CONFIGURED  (1U)
#define USBD_EP_FLAG_BUSY        (1U << 1)

// Endpoint number and index
#define EP_NUM(ep_addr)           (ep_addr & ARM_USB_ENDPOINT_NUMBER_MASK)
#define EP_ID(ep_addr)            (EP_NUM(ep_addr) * 2U + ((ep_addr >> 7U) & 1U))

// Endpoint structure
typedef struct {
  uint8_t  *buffer;
  uint32_t  bufferIndex;
  uint32_t  dataSize;
  uint32_t  maxPacketSize;
  uint8_t   packetCount;
  uint8_t   in_NAK;
  uint8_t   flags;
} ENDPOINT;

// Local variables and structures
static ARM_USBD_SignalDeviceEvent_t   cbDeviceEvent;
static ARM_USBD_SignalEndpointEvent_t cbEndpointEvent;

static ARM_USBD_STATE usbd_state = {0};

         static uint8_t  setup_buf[8];
volatile static uint8_t  setup_flag = 0U;

volatile static ENDPOINT ep[(USBD_EP_NUM + 1U)*2U];

// Function prototypes
static uint16_t  USBD_GetFrameNumber (void);


// Local functions

/**
  \fn          void USBD_FlushInEpFifo (uint8_t ep_addr)
  \brief       Flush IN Endpoint FIFO
  \param[in]   ep_addr specifies Endpoint Address
                 ep_addr.0..3: Address
                 ep_addr.7:    Direction
*/
static void USBD_FlushInEpFifo (uint8_t ep_addr) {
  ep_addr &= ARM_USB_ENDPOINT_NUMBER_MASK;

  // Flush transmit FIFO
  OTG->GRSTCTL = (OTG->GRSTCTL & ~OTG_HS_GRSTCTL_TXFNUM_MSK) |
                  OTG_HS_GRSTCTL_TXFNUM(ep_addr)             |
                  OTG_HS_GRSTCTL_TXFFLSH;
}

/**
  \fn          void USBD_Reset (void)
  \brief       Called after usbd reset interrupt to reset configuration
*/
static void USBD_Reset (void) {
  uint32_t i;

  // Clear Endpoint mask registers
  OTG->DOEPMSK = 0U;
  OTG->DIEPMSK = 0U;

  for (i = 1U; i <= USBD_EP_NUM; i++) {
    if (OTG_DOEPCTL(i) & OTG_HS_DOEPCTLx_EPENA) {
      OTG_DOEPCTL(i)   = OTG_HS_DOEPCTLx_EPDIS |    // Endpoint disable
                         OTG_HS_DOEPCTLx_SNAK;      // Endpoint set NAK
    }
    if (OTG_DIEPCTL(i) & OTG_HS_DIEPCTLx_EPENA) {
      OTG_DIEPCTL(i)   = OTG_HS_DIEPCTLx_EPDIS |    // Endpoint disable
                         OTG_HS_DIEPCTLx_SNAK;      // Endpoint set NAK
    }
    USBD_FlushInEpFifo(i);

    // Reset Endpoint resources
    memset((void *)(&ep[i*2U  ]),  0, sizeof (ENDPOINT));
    memset((void *)(&ep[i*2U+1U]), 0, sizeof (ENDPOINT));

    // Clear IN Endpoint interrupts
    OTG_DIEPINT(i) = OTG_HS_DIEPINTx_XFCR    |
                     OTG_HS_DIEPINTx_EPDISD  |
                     OTG_HS_DIEPINTx_TOC     |
                     OTG_HS_DIEPINTx_ITTXFE  |
                     OTG_HS_DIEPINTx_INEPNE  |
                     OTG_HS_DIEPINTx_TXFE    ;

    // Clear OUT Endpoint interrupts
    OTG_DOEPINT(i) = OTG_HS_DOEPINTx_XFCR    |
                     OTG_HS_DOEPINTx_EPDISD  |
                     OTG_HS_DOEPINTx_STUP    |
                     OTG_HS_DOEPINTx_OTEPDIS |
                     OTG_HS_DOEPINTx_B2BSTUP ;
  }

  // Set device address to 0
  OTG->DCFG       = (OTG->DCFG & ~OTG_HS_DCFG_DAD_MSK) | (0 << OTG_HS_DCFG_DAD_POS);
  OTG->DAINTMSK   =  OTG_HS_DAINT_IEPINT(0U) |  // Enable IN Endpoint0 interrupt
                     OTG_HS_DAINT_OEPINT(0U);   // Enable OUT Endpoint0 interrupt

  // Enable Setup phase done, OUT Endpoint disabled and OUT transfer complete interrupt
  OTG->DOEPMSK    =  OTG_HS_DOEPMSK_STUPM    |
                     OTG_HS_DOEPMSK_EPDM     |
                     OTG_HS_DOEPMSK_XFRCM;

  // Enable In Endpoint disable and IN transfer complete interrupt
  OTG->DIEPMSK    =  OTG_HS_DIEPMSK_EPDM     |
                     OTG_HS_DIEPMSK_XFRCM;

  // Configure FIFOs
  OTG->GRXFSIZ    =  OTG_RX_FIFO_SIZE   / 4U;
  OTG->TX0FSIZ    = (OTG_RX_FIFO_SIZE   / 4U) |
                   ((OTG_TX0_FIFO_SIZE  / 4U) << OTG_HS_DIEPTXFx_INEPTXFD_POS);

  OTG->DIEPTXF1   = ((OTG_RX_FIFO_SIZE + OTG_TX0_FIFO_SIZE) / 4U) |
                    ((OTG_TX1_FIFO_SIZE / 4U) << OTG_HS_DIEPTXFx_INEPTXFD_POS);

  OTG->DIEPTXF2   = ((OTG_RX_FIFO_SIZE + OTG_TX0_FIFO_SIZE + OTG_TX1_FIFO_SIZE) / 4U) |
                    ((OTG_TX2_FIFO_SIZE / 4U) << OTG_HS_DIEPTXFx_INEPTXFD_POS);

  OTG->DIEPTXF3   = ((OTG_RX_FIFO_SIZE + OTG_TX0_FIFO_SIZE + OTG_TX1_FIFO_SIZE +
                      OTG_TX2_FIFO_SIZE) / 4U) | ((OTG_TX3_FIFO_SIZE / 4U) << OTG_HS_DIEPTXFx_INEPTXFD_POS);

  OTG->DIEPTXF4   = ((OTG_RX_FIFO_SIZE + OTG_TX0_FIFO_SIZE + OTG_TX1_FIFO_SIZE +
                      OTG_TX2_FIFO_SIZE + OTG_TX3_FIFO_SIZE) / 4U) |
                    ((OTG_TX4_FIFO_SIZE / 4U) << OTG_HS_DIEPTXFx_INEPTXFD_POS);

  OTG->DIEPTXF5   = ((OTG_RX_FIFO_SIZE + OTG_TX0_FIFO_SIZE + OTG_TX1_FIFO_SIZE +
                      OTG_TX2_FIFO_SIZE + OTG_TX3_FIFO_SIZE + OTG_TX4_FIFO_SIZE) / 4U) |
                    ((OTG_TX5_FIFO_SIZE / 4U) << OTG_HS_DIEPTXFx_INEPTXFD_POS);
}

/**
  \fn          void USBD_EndpointReadSet (uint8_t ep_addr)
  \brief       Set Endpoint for next read.
  \param[in]   ep_addr specifies Endpoint Address
                 ep_addr.0..3: Address
                 ep_addr.7:    Direction
*/
static void USBD_EndpointReadSet (uint8_t ep_addr) {
  volatile ENDPOINT *ptr_ep;
  uint32_t           sz;
  uint8_t            ep_num;

  ptr_ep = &ep[EP_ID(ep_addr)];
  ep_num = EP_NUM(ep_addr);

  // Set packet count and transfer size
  if  (ptr_ep->dataSize > ptr_ep->maxPacketSize) { sz = ptr_ep->maxPacketSize; }
  else                                           { sz = ptr_ep->dataSize; }

  if (ep_num) {
    OTG_DOEPTSIZ(ep_num) = (ptr_ep->packetCount << OTG_HS_DOEPTSIZx_PKTCNT_POS ) |
                            sz;
  } else {
    OTG_DOEPTSIZ(0U)     = (ptr_ep->packetCount << OTG_HS_DOEPTSIZx_PKTCNT_POS ) |
                           (3U                  << OTG_HS_DOEPTSIZ0_STUPCNT_POS) |
                            sz;
  }
  // Set correct frame for Isochronous Endpoint
  if (OTG_EP_OUT_TYPE(ep_num) == ARM_USB_ENDPOINT_ISOCHRONOUS) {
    if ((USBD_GetFrameNumber() & 1U)) { OTG_DOEPCTL(ep_num) |= OTG_HS_DOEPCTLx_SEVNFRM; }
    else                              { OTG_DOEPCTL(ep_num) |= OTG_HS_DOEPCTLx_SODDFRM; }
  }

  // Clear NAK and enable Endpoint
  OTG_DOEPCTL(ep_num) |= OTG_HS_DOEPCTLx_EPENA | OTG_HS_DOEPCTLx_CNAK;
}

/**
  \fn          int32_t USBD_ReadFromFifo (uint8_t ep_addr, uint32_t sz)
  \brief       Read data from USB Endpoint.
  \param[in]   ep_addr specifies Endpoint Address
                 ep_addr.0..3: Address
                 ep_addr.7:    Direction
  \param[in]   sz specifies data size to be read from FIFO
  \return      number of data bytes read
*/
static int32_t USBD_ReadFromFifo (uint8_t ep_addr, uint32_t sz) {
  volatile ENDPOINT *ptr_ep;
  uint32_t           i, residue, val;
  uint8_t           *ptr_dest_8;
  __packed uint32_t *ptr_dest_32;
  volatile uint32_t *ptr_src;
  uint8_t            ep_num;
  uint8_t            tmp_buf[4];

  ptr_ep = &ep[EP_ID(ep_addr)];
  ep_num = EP_NUM(ep_addr);

  // Check if Endpoint is activated and buffer available
  if (!(OTG_DOEPCTL(ep_num) & OTG_HS_DOEPCTLx_USBAEP)) { return 0; }
  if (!ptr_ep->buffer)                                 { return 0; }

  if (sz > ptr_ep->dataSize) {
    sz = ptr_ep->dataSize;
  }

  // If Isochronous Endpoint
  if (OTG_EP_OUT_TYPE(ep_num) == ARM_USB_ENDPOINT_ISOCHRONOUS) {
    val =  ptr_ep->packetCount -
          ((OTG_DOEPTSIZ(ep_num) & OTG_HS_DOEPTSIZx_PKTCNT_MSK) >> OTG_HS_DOEPTSIZx_PKTCNT_POS);

    switch ((OTG_DOEPTSIZ(ep_num) & OTG_HS_DOEPTSIZx_RXDPID_MSK) >> OTG_HS_DOEPTSIZx_RXDPID_POS) {
      case 0:                           // DATA0
        if (val != 1U) sz = 0U;
        break;
      case 2:                           // DATA1
        if (val != 2U) sz = 0U;
        break;
      case 1:                           // DATA2
        if (val != 3U) sz = 0U;
        break;
      default: break;
    }
  }

  // Copy data from FIFO
  ptr_src     = (volatile uint32_t *)(OTG_HS_BASE + 0x1000U);
  ptr_dest_32 = (__packed uint32_t *)(ptr_ep->buffer + ptr_ep->bufferIndex);
  i           = sz / 4U;
  while (i--) {
    *ptr_dest_32++ = *ptr_src;
  }
  ptr_ep->bufferIndex += (sz / 4U) * 4U;

  // If data size is not equal n*4
  residue = sz % 4U;
  if (residue) {
    ptr_dest_8 = (uint8_t *)(ptr_dest_32);
    *((__packed uint32_t *)tmp_buf) = OTG_RX_FIFO;
    for (i = 0U; i < residue; i++) {
      *ptr_dest_8++ = tmp_buf[i];
      ptr_ep->bufferIndex ++;
    }
  }

  if (sz != ptr_ep->maxPacketSize) { ptr_ep->dataSize  = 0U; }
  else                             { ptr_ep->dataSize -= sz; }

  return sz;
}

/**
  \fn          void USBD_WriteToFifo (uint8_t ep_addr)
  \brief       Write data to Endpoint FIFO.
  \param[in]   ep_addr specifies Endpoint Address
                 ep_addr.0..3: Address
                 ep_addr.7:    Direction
*/
static void USBD_WriteToFifo (uint8_t ep_addr) {
  volatile ENDPOINT *ptr_ep;
  uint8_t            ep_num;
  uint32_t           sz, i;
  volatile uint32_t *ptr_dest;
  __packed uint32_t *ptr_src;

  ptr_ep = &ep[EP_ID(ep_addr)];
  ep_num = EP_NUM(ep_addr);

  if (ptr_ep->dataSize > ptr_ep->maxPacketSize) { sz = ptr_ep->maxPacketSize; }
  else                                          { sz = ptr_ep->dataSize; }

  // Check if enough space in FIFO
  if ((OTG_DTXFSTS(ep_num) * 4U) < sz) { return; }

  // Set transfer size and packet count
  OTG_DIEPTSIZ(ep_num) = (ptr_ep->packetCount << OTG_HS_DIEPTSIZx_PKTCNT_POS) |
                         (ptr_ep->packetCount << OTG_HS_DIEPTSIZx_MCNT_POS)   |
                          sz;

  // Set correct frame for Isochronous Endpoint
  if (OTG_EP_IN_TYPE(ep_num) == ARM_USB_ENDPOINT_ISOCHRONOUS) {
    if (USBD_GetFrameNumber() & 1U) { OTG_DIEPCTL(ep_num) |= OTG_HS_DIEPCTLx_SEVNFRM; }
    else                            { OTG_DIEPCTL(ep_num) |= OTG_HS_DIEPCTLx_SODDFRM; }
  }

  // Enable Endpoint and clear NAK
  OTG_DIEPCTL(ep_num) |= OTG_HS_DIEPCTLx_EPENA | OTG_HS_DIEPCTLx_CNAK;

  ptr_src  = (__packed uint32_t *)(ptr_ep->buffer + ptr_ep->bufferIndex);
  ptr_dest = (volatile uint32_t *)(OTG_HS_BASE + 0x1000U + ep_num*0x1000U);

  ptr_ep->bufferIndex += sz;
  ptr_ep->dataSize    -= sz;
  i = (sz+3U)>>2U;
  // Copy data to FIFO
  while (i--) {
    *ptr_dest = *ptr_src++;
  }
}


// USB DEVICE DRIVER FUNCTIONS

/**
  \fn          ARM_DRIVER_VERSION USBD_GetVersion (void)
  \brief       Get driver version.
  \return      \ref ARM_DRIVER_VERSION
*/
static ARM_DRIVER_VERSION USBD_GetVersion (void) { return usbd_driver_version; }

/**
  \fn          ARM_USBD_CAPABILITIES USBD_GetCapabilities (void)
  \brief       Get driver capabilities.
  \return      \ref ARM_USBD_CAPABILITIES
*/
static ARM_USBD_CAPABILITIES USBD_GetCapabilities (void) { return usbd_driver_capabilities; }

/**
  \fn          int32_t USBD_Initialize (ARM_USBD_SignalDeviceEvent_t   cb_device_event,
                                        ARM_USBD_SignalEndpointEvent_t cb_endpoint_event)
  \brief       Initialize USB Device Interface.
  \param[in]   cb_device_event    Pointer to \ref ARM_USBD_SignalDeviceEvent
  \param[in]   cb_endpoint_event  Pointer to \ref ARM_USBD_SignalEndpointEvent
  \return      \ref execution_status
*/
static int32_t USBD_Initialize (ARM_USBD_SignalDeviceEvent_t   cb_device_event,
                                ARM_USBD_SignalEndpointEvent_t cb_endpoint_event) {

  if (otg_hs_state & OTG_HS_USBD_DRIVER_INITIALIZED) { return ARM_DRIVER_OK; }
  if (otg_hs_state)                                  { return ARM_DRIVER_ERROR; }

  cbDeviceEvent   = cb_device_event;
  cbEndpointEvent = cb_endpoint_event;

  otg_hs_role     = ARM_USB_ROLE_DEVICE;
  OTG_HS_PinsConfigure (ARM_USB_PIN_DP | ARM_USB_PIN_DM);

  otg_hs_state    = OTG_HS_USBD_DRIVER_INITIALIZED;

  return ARM_DRIVER_OK;
}

/**
  \fn          int32_t USBD_Uninitialize (void)
  \brief       De-initialize USB Device Interface.
  \return      \ref execution_status
*/
static int32_t USBD_Uninitialize (void) {

  if (!(otg_hs_state & OTG_HS_USBD_DRIVER_INITIALIZED)) { return ARM_DRIVER_OK; }
  if (  otg_hs_state & OTG_HS_USBD_DRIVER_POWERED     ) { return ARM_DRIVER_ERROR; }

  OTG_HS_PinsUnconfigure (ARM_USB_PIN_DP | ARM_USB_PIN_DM);
  otg_hs_role  = ARM_USB_ROLE_NONE;

  otg_hs_state = 0U;

  return ARM_DRIVER_OK;
}

/**
  \fn          int32_t USBD_PowerControl (ARM_POWER_STATE state)
  \brief       Control USB Device Interface Power.
  \param[in]   state  Power state
  \return      \ref execution_status
*/
static int32_t USBD_PowerControl (ARM_POWER_STATE state) {

  if (!(otg_hs_state & OTG_HS_USBD_DRIVER_INITIALIZED)) { return ARM_DRIVER_ERROR; }

  switch (state) {
    case ARM_POWER_OFF:
      if (!(otg_hs_state & OTG_HS_USBD_DRIVER_POWERED)) { return ARM_DRIVER_OK; }
      otg_hs_state  &= ~OTG_HS_USBD_DRIVER_POWERED;
      NVIC_DisableIRQ(OTG_HS_IRQn);                     // Disable interrupt
      OTG->GAHBCFG  &= ~OTG_HS_GAHBCFG_GINT;            // Disable global interrupt mask

      OTG->DCTL     |=  OTG_HS_DCTL_SDIS;               // Soft disconnect enabled
      OTG->GCCFG    &= ~(OTG_HS_GCCFG_VBUSBSEN |        // Disable VBUS sensing device "B"
                         OTG_HS_GCCFG_PWRDWN);          // Power down activated

#ifdef MX_USB_OTG_HS_ULPI_D7_Pin
      // External ULPI High-speed PHY
      RCC->AHB1ENR  &= ~RCC_AHB1ENR_OTGHSULPIEN;        // Disable OTG HS ULPI clock
#endif

      RCC->AHB1ENR  &= ~RCC_AHB1ENR_OTGHSEN;            // Disable OTG HS Clock
      break;

    case ARM_POWER_FULL:
      if (otg_hs_state & OTG_HS_USBD_DRIVER_POWERED) { return ARM_DRIVER_OK; }
      RCC->AHB1ENR  |=  RCC_AHB1ENR_OTGHSEN;            // OTG HS clock enable
      RCC->AHB1RSTR |=  RCC_AHB1RSTR_OTGHRST;           // Reset OTG HS clock
      osDelay(1U);
      RCC->AHB1RSTR &= ~RCC_AHB1RSTR_OTGHRST;           // Clear reset OTG HS clock
      RCC->AHB1ENR  |=  RCC_AHB1ENR_OTGHSEN;            // Enable OTG HS clock

#ifdef MX_USB_OTG_HS_ULPI_D7_Pin
      // External ULPI High-speed PHY
      RCC->AHB1ENR  |=  RCC_AHB1ENR_OTGHSULPIEN;        // Enable OTG HS ULPI clock
#else
      // On-chip Full-speed PHY
      OTG->GUSBCFG   |=  OTG_HS_GUSBCFG_PHSEL  |        // Full-speed transceiver
                         OTG_HS_GUSBCFG_PHYLPCS;        // 48 MHz external clock
#endif

      // Wait until AHB Master state machine is in the idle condition
      while (!(OTG->GRSTCTL & OTG_HS_GRSTCTL_AHBIDL));

      osDelay(2U);

      // Core soft reset
      OTG->GRSTCTL  |=  OTG_HS_GRSTCTL_CSRST;
      while (OTG->GRSTCTL & OTG_HS_GRSTCTL_CSRST);

      osDelay(10U);

      OTG->GAHBCFG  &= ~OTG_HS_GAHBCFG_GINT;            // Disable global interrupt mask
      OTG->GCCFG    |=  OTG_HS_GCCFG_VBUSBSEN;          // Enable VBUS sensing device "B"
#ifdef MX_USB_OTG_HS_ULPI_D7_Pin
      // External ULPI High-speed PHY
      OTG->GCCFG    &= ~OTG_HS_GCCFG_NOVBUSSENS;        // Disable VBUS sense
#endif
      OTG->DCTL     |=  OTG_HS_DCTL_SDIS;               // Soft disconnect enabled

      // Set turnaround time and force device mode
      OTG->GUSBCFG   = (OTG->GUSBCFG & ~OTG_HS_GUSBCFG_TRDT_MSK) |
                        OTG_HS_GUSBCFG_TRDT(5U)                  |
                        OTG_HS_GUSBCFG_FDMOD;

      osDelay(100U);

#ifdef MX_USB_OTG_HS_ULPI_D7_Pin
      // External ULPI High-speed PHY
      OTG->DCFG     &= ~OTG_HS_DCFG_DSPD_MSK;           // High speed
#else
      // On-chip Full-speed PHY
      OTG->DCFG     |=  OTG_HS_DCFG_DSPD_MSK;           // Full Speed
#endif

      // Clear interrupts
      OTG->GINTSTS   =  OTG_HS_GINTSTS_MMIS     |
                        OTG_HS_GINTSTS_SOF      |
                        OTG_HS_GINTSTS_ESUSP    |
                        OTG_HS_GINTSTS_USBSUSP  |
                        OTG_HS_GINTSTS_USBRST   |
                        OTG_HS_GINTSTS_ENUMDNE  |
                        OTG_HS_GINTSTS_ISOODRP  |
                        OTG_HS_GINTSTS_EOPF     |
                        OTG_HS_GINTSTS_IISOIXFR |
                        OTG_HS_GINTSTS_IPXFR    |
                        OTG_HS_GINTSTS_CIDSCHG  |
                        OTG_HS_GINTSTS_DISCINT  |
                        OTG_HS_GINTSTS_SRQINT   |
                        OTG_HS_GINTSTS_WKUPINT;

      // Unmask interrupts
      OTG->GINTMSK   =  OTG_HS_GINTMSK_USBSUSPM |
                        OTG_HS_GINTMSK_USBRST   |
                        OTG_HS_GINTMSK_ENUMDNEM |
                        OTG_HS_GINTMSK_RXFLVLM  |
                        OTG_HS_GINTMSK_IEPINT   |
                        OTG_HS_GINTMSK_OEPINT   |
#ifdef ARM_USBD_VBUS_DETECT
                        OTG_HS_GINTMSK_SRQIM    |
                        OTG_HS_GINTMSK_OTGINT   |
#endif
                        OTG_HS_GINTMSK_WUIM;

#ifdef ARM_USBD_VBUS_DETECT
      if (OTG->GOTGCTL & OTG_HS_GOTGCTL_BSVLD) {        // If B-session valid
        cbDeviceEvent(ARM_USBD_EVENT_VBUS_ON);
      } else {
        cbDeviceEvent(ARM_USBD_EVENT_VBUS_OFF);
      }
#endif
      otg_hs_state  |=  OTG_HS_USBD_DRIVER_POWERED;

      NVIC_EnableIRQ(OTG_HS_IRQn);                      // Enable interrupts
      OTG->GAHBCFG  |=  OTG_HS_GAHBCFG_GINT    |
                        OTG_HS_GAHBCFG_TXFELVL ;
      break;

    default:
      return ARM_DRIVER_ERROR_UNSUPPORTED;
  }

  return ARM_DRIVER_OK;
}

/**
  \fn          int32_t USBD_DeviceConnect (void)
  \brief       Connect USB Device.
  \return      \ref execution_status
*/
static int32_t USBD_DeviceConnect (void) {

  if (!(otg_hs_state & OTG_HS_USBD_DRIVER_POWERED) ) { return ARM_DRIVER_ERROR; }
  if (  otg_hs_state & OTG_HS_USBD_DRIVER_CONNECTED) { return ARM_DRIVER_OK; }

  OTG->DCTL    &= ~OTG_HS_DCTL_SDIS;    // Soft disconnect disabled
  OTG->GCCFG   |=  OTG_HS_GCCFG_PWRDWN;

  otg_hs_state |=  OTG_HS_USBD_DRIVER_CONNECTED;

  return ARM_DRIVER_OK;
}

/**
  \fn          int32_t USBD_DeviceDisconnect (void)
  \brief       Disconnect USB Device.
  \return      \ref execution_status
*/
static int32_t USBD_DeviceDisconnect (void) {

  if (!(otg_hs_state & OTG_HS_USBD_DRIVER_POWERED)  ) { return ARM_DRIVER_ERROR; }
  if (!(otg_hs_state & OTG_HS_USBD_DRIVER_CONNECTED)) { return ARM_DRIVER_OK; }

  OTG->DCTL  |=  OTG_HS_DCTL_SDIS;      // Soft disconnect enabled
  OTG->GCCFG &= ~OTG_HS_GCCFG_PWRDWN;   // Power down activated

  usbd_state.active = false;

  otg_hs_state &= ~OTG_HS_USBD_DRIVER_CONNECTED;

  return ARM_DRIVER_OK;
}

/**
  \fn          ARM_USBD_STATE USBD_DeviceGetState (void)
  \brief       Get current USB Device State.
  \return      Device State \ref ARM_USBD_STATE
*/
static ARM_USBD_STATE USBD_DeviceGetState (void) {

#ifdef ARM_USBD_VBUS_DETECT
  if (OTG->GOTGCTL & OTG_HS_GOTGCTL_BSVLD) {            // If B-session valid
    usbd_state.vbus = 1U;
  } else {
    usbd_state.vbus = 0U;
  }
#else
  usbd_state.vbus = 0U;
#endif

  return usbd_state;
}

/**
  \fn          int32_t USBD_DeviceRemoteWakeup (void)
  \brief       Trigger USB Remote Wakeup.
  \return      \ref execution_status
*/
static int32_t USBD_DeviceRemoteWakeup (void) {

  if (!(otg_hs_state & OTG_HS_USBD_DRIVER_CONNECTED)) { return ARM_DRIVER_ERROR; }

  OTG->DCTL |=   OTG_HS_DCTL_RWUSIG;    // Remote wakeup signalling
  osDelay(5U);
  OTG->DCTL &=  ~OTG_HS_DCTL_RWUSIG;

  return ARM_DRIVER_OK;
}

/**
  \fn          int32_t USBD_DeviceSetAddress (uint8_t dev_addr)
  \brief       Set USB Device Address.
  \param[in]   dev_addr  Device Address
  \return      \ref execution_status
*/
static int32_t USBD_DeviceSetAddress (uint8_t dev_addr) {

  if (!(otg_hs_state & OTG_HS_USBD_DRIVER_CONNECTED)) { return ARM_DRIVER_ERROR; }

  OTG->DCFG = (OTG->DCFG & ~OTG_HS_DCFG_DAD_MSK) |
               OTG_HS_DCFG_DAD(dev_addr)         ;

  return ARM_DRIVER_OK;
}

/**
  \fn          int32_t USBD_ReadSetupPacket (uint8_t *setup)
  \brief       Read setup packet received over Control Endpoint.
  \param[out]  setup  Pointer to buffer for setup packet
  \return      \ref execution_status
*/
static int32_t USBD_ReadSetupPacket (uint8_t *setup) {

  if (!(otg_hs_state & OTG_HS_USBD_DRIVER_CONNECTED)) { return ARM_DRIVER_ERROR; }
  if (!setup_flag)                                    { return ARM_DRIVER_ERROR; }

  setup_flag = 0U;
  memcpy(setup, setup_buf, 8U);

  if (setup_flag) {
    // New setup packet was received while this was being read
    return ARM_DRIVER_ERROR;
  }

  return ARM_DRIVER_OK;
}

/**
  \fn          int32_t USBD_EndpointConfigure (uint8_t  ep_addr,
                                               uint8_t  ep_type,
                                               uint16_t ep_max_packet_size)
  \brief       Configure USB Endpoint.
  \param[in]   ep_addr  Endpoint Address
                - ep_addr.0..3: Address
                - ep_addr.7:    Direction
  \param[in]   ep_type  Endpoint Type (ARM_USB_ENDPOINT_xxx)
  \param[in]   ep_max_packet_size Endpoint Maximum Packet Size
  \return      \ref execution_status
*/
static int32_t USBD_EndpointConfigure (uint8_t  ep_addr,
                                       uint8_t  ep_type,
                                       uint16_t ep_max_packet_size) {
  volatile ENDPOINT *ptr_ep;
  uint32_t           ep_num, ep_dir, ep_mps;

  ptr_ep = &ep[EP_ID(ep_addr)];

  if (!(otg_hs_state & OTG_HS_USBD_DRIVER_CONNECTED)) { return ARM_DRIVER_ERROR; }
  if (ptr_ep->flags & USBD_EP_FLAG_BUSY)              { return ARM_DRIVER_ERROR_BUSY; }
  if (ptr_ep->flags & USBD_EP_FLAG_CONFIGURED)        { return ARM_DRIVER_OK; }

  ep_num =  EP_NUM(ep_addr);
  ep_dir = (ep_addr & ARM_USB_ENDPOINT_DIRECTION_MASK) == ARM_USB_ENDPOINT_DIRECTION_MASK;
  ep_mps =  ep_max_packet_size & ARM_USB_ENDPOINT_MAX_PACKET_SIZE_MASK;

  // Check if Endpoint is enabled
  if (ep_num > USBD_EP_NUM) return ARM_DRIVER_ERROR;

  // Set Endpoint resources
  ptr_ep->buffer        = NULL;
  ptr_ep->dataSize      = 0U;
  ptr_ep->maxPacketSize = ep_mps;

  if (ep_dir) {                         // IN Endpoint
    ptr_ep->in_NAK = 0U;                // Clear IN Endpoint NAK flag

    if (OTG_EP_IN_TYPE(ep_num) == ARM_USB_ENDPOINT_ISOCHRONOUS) {
      ptr_ep->packetCount = (ep_mps &
                             ARM_USB_ENDPOINT_MICROFRAME_TRANSACTIONS_MASK) >> 11U;
    } else {
      ptr_ep->packetCount = 1U;
    }

    // Configure IN Endpoint
    OTG_DIEPCTL(ep_num) = (ep_num  <<  OTG_HS_DIEPCTLx_TXFNUM_POS) |    // FIFO Number
                          (ep_type <<  OTG_HS_DIEPCTLx_EPTYP_POS ) |    // Endpoint Type
                           ep_mps;                                      // Max Packet Size

    // Set DATA0 PID for Interrupt or Bulk Endpoint
    if (ep_type >= ARM_USB_ENDPOINT_BULK) {
      OTG_DIEPCTL(ep_num) |= OTG_HS_DIEPCTLx_SD0PID;
    }

    OTG_DIEPCTL(ep_num)   |= OTG_HS_DIEPCTLx_USBAEP;    // Activate Endpoint

    if (OTG_DIEPCTL(ep_num) & OTG_HS_DIEPCTLx_EPENA) {
      OTG_DIEPCTL(ep_num) |= OTG_HS_DIEPCTLx_EPDIS;     // Disable Endpoint
    }

    USBD_FlushInEpFifo (ep_addr);

    // Isochronous IN Endpoint Configuration
    if (OTG_EP_IN_TYPE(ep_num) == ARM_USB_ENDPOINT_ISOCHRONOUS) {
      OTG->GINTMSK |= OTG_HS_GINTMSK_IISOIXFRM;         // Enable IISOIXFR

      // Regarding FrameNumber, set Frame
      if (USBD_GetFrameNumber() & 1U) { OTG_DIEPCTL(ep_num) |= OTG_HS_DIEPCTLx_SEVNFRM; }
      else                            { OTG_DIEPCTL(ep_num) |= OTG_HS_DIEPCTLx_SODDFRM; }

      // Enable Endpoint and Clear NAK
      OTG_DIEPCTL(ep_num) |= OTG_HS_DIEPCTLx_EPENA | OTG_HS_DIEPCTLx_CNAK;
    }
  } else {                              // OUT Endpoint
    if (OTG_EP_OUT_TYPE(ep_num) == ARM_USB_ENDPOINT_ISOCHRONOUS) {
      ptr_ep->packetCount = (ep_mps &
                             ARM_USB_ENDPOINT_MICROFRAME_TRANSACTIONS_MASK) >> 11U;
    } else {
      ptr_ep->packetCount = 1U;
    }

    // Configure OUT Endpoint
    OTG_DOEPCTL(ep_num) = (ep_type <<  OTG_HS_DOEPCTLx_EPTYP_POS) |     // Endpoint Type
                           OTG_HS_DOEPCTLx_SNAK                   |     // Set NAK
                           ep_mps;                                      // Max Packet Size

    // Set DATA0 PID for Interrupt or Bulk Endpoint
    if (ep_type >= ARM_USB_ENDPOINT_BULK) {
      OTG_DOEPCTL(ep_num) |= OTG_HS_DOEPCTLx_SD0PID;
    }

    // Isochronous OUT Endpoint Configuration
    if (OTG_EP_OUT_TYPE(ep_num) == ARM_USB_ENDPOINT_ISOCHRONOUS) {
      OTG->GINTMSK |= OTG_HS_GINTMSK_EOPFM;             // Enable End of Periodic Frame Interrupt
    }

    OTG_DOEPCTL(ep_num) |= OTG_HS_DOEPCTLx_USBAEP;      // Activate Endpoint
  }

  ptr_ep->flags |= USBD_EP_FLAG_CONFIGURED;             // Set Endpoint configured flag

  OTG->DAINTMSK |= (1U << (ep_num + ((ep_dir ^ 1U) << 4U)));            // Enable Endpoint interrupt

  return ARM_DRIVER_OK;
}

/**
  \fn          int32_t USBD_EndpointUnconfigure (uint8_t ep_addr)
  \brief       Unconfigure USB Endpoint.
  \param[in]   ep_addr  Endpoint Address
                - ep_addr.0..3: Address
                - ep_addr.7:    Direction
  \return      \ref execution_status
*/
static int32_t USBD_EndpointUnconfigure (uint8_t ep_addr) {
  volatile ENDPOINT *ptr_ep;
  uint32_t           ep_num, ep_dir, IsoEpEnCnt, num;

  ptr_ep = &ep[EP_ID(ep_addr)];

  if (!(otg_hs_state & OTG_HS_USBD_DRIVER_CONNECTED)) { return ARM_DRIVER_ERROR; }
  if (  ptr_ep->flags & USBD_EP_FLAG_BUSY)            { return ARM_DRIVER_ERROR_BUSY; }
  if (!(ptr_ep->flags & USBD_EP_FLAG_CONFIGURED))     { return ARM_DRIVER_OK; }

  IsoEpEnCnt = 0U;
  ep_num =  EP_NUM(ep_addr);
  ep_dir = (ep_addr & ARM_USB_ENDPOINT_DIRECTION_MASK) == ARM_USB_ENDPOINT_DIRECTION_MASK;

  OTG->DAINTMSK &= ~(1U << (ep_num + ((ep_dir ^ 1U) << 4U)));           // Disable Endpoint interrupt

  ptr_ep->flags &= ~USBD_EP_FLAG_CONFIGURED;            // Clear Endpoint configured flag

  memset((void *)(ptr_ep), 0, sizeof (ENDPOINT));       // Reset Endpoint parameters

  if (ep_dir) {                         // IN Endpoint
    // Count Active Isochronous IN Endpoints
    if (OTG_EP_IN_TYPE(ep_num) == ARM_USB_ENDPOINT_ISOCHRONOUS) {
      for (num = 1U; num <= USBD_EP_NUM; num++) {
        if (OTG_DIEPCTL(num) & OTG_HS_DIEPCTLx_USBAEP) {
          if (OTG_EP_IN_TYPE(num) == ARM_USB_ENDPOINT_ISOCHRONOUS) {
            IsoEpEnCnt++;
          }
        }
      }
      // If Last Active Isochronous IN Endpoint, Disable IISOIXFR
      if (IsoEpEnCnt == 1U) { OTG->GINTMSK &= ~OTG_HS_GINTMSK_IISOIXFRM; }
    }

    if (OTG_DIEPCTL(ep_num) & OTG_HS_DIEPCTLx_EPENA) {
      OTG_DIEPCTL(ep_num)  |=  OTG_HS_DIEPCTLx_EPDIS;   // Disable Endpoint
    }

    OTG_DIEPCTL(ep_num)    |=  OTG_HS_DIEPCTLx_SNAK;    // Set Endpoint NAK

    if (ep_num) { OTG_DIEPCTL(ep_num)  &= ~OTG_HS_DIEPCTLx_USBAEP; }     // Deactivate Endpoint
  } else {                              // OUT Endpoint
    // Count Active Isochronous OUT Endpoints
    if (OTG_EP_OUT_TYPE(ep_num) == ARM_USB_ENDPOINT_ISOCHRONOUS) {
      for (num = 1U; num <= USBD_EP_NUM; num++) {
        if (OTG_DOEPCTL(num) & OTG_HS_DOEPCTLx_USBAEP) {
          if (OTG_EP_OUT_TYPE(num) == ARM_USB_ENDPOINT_ISOCHRONOUS) {
            IsoEpEnCnt++;
          }
        }
      }
      // If Last Active Isochronous OUT Endpoint, Disable EOPF
      if (IsoEpEnCnt == 1U) { OTG->GINTMSK &= ~OTG_HS_GINTMSK_EOPFM; }
    }

    OTG->DCTL |= OTG_HS_DCTL_SGONAK;                    // Set Global OUT NAK
    while (!(OTG->GINTSTS & OTG_HS_GINTSTS_GONAKEFF));

    OTG_DOEPCTL(ep_num) |= OTG_HS_DOEPCTLx_SNAK;        // Set Endpoint NAK

    if (ep_num) {
      // Disable OUT Endpoint
      if (OTG_DOEPCTL(ep_num) & OTG_HS_DOEPCTLx_EPENA) {// If Endpoint is Enabled
        OTG_DOEPCTL(ep_num)  |=  OTG_HS_DOEPCTLx_EPDIS; // Disable Endpoint

        while (!(OTG_DOEPINT(ep_num) & OTG_HS_DOEPINTx_EPDISD));
      }
      OTG_DOEPCTL(ep_num) &= ~OTG_HS_DOEPCTLx_USBAEP;   // Deactivate Endpoint
    }

    OTG->DCTL |= OTG_HS_DCTL_CGONAK;                    // Clear Global OUT NAK
  }

  return ARM_DRIVER_OK;
}

/**
  \fn          int32_t USBD_EndpointStall (uint8_t ep_addr, bool stall)
  \brief       Set/Clear Stall for USB Endpoint.
  \param[in]   ep_addr  Endpoint Address
                - ep_addr.0..3: Address
                - ep_addr.7:    Direction
  \param[in]   stall  Operation
                - \b false Clear
                - \b true Set
  \return      \ref execution_status
*/
static int32_t USBD_EndpointStall (uint8_t ep_addr, bool stall) {
  volatile ENDPOINT *ptr_ep;
  uint32_t           ep_num, ep_dir;

  ptr_ep = &ep[EP_ID(ep_addr)];

  if (!(otg_hs_state & OTG_HS_USBD_DRIVER_CONNECTED)) { return ARM_DRIVER_ERROR; }
  if (!(ptr_ep->flags & USBD_EP_FLAG_CONFIGURED))     { return ARM_DRIVER_ERROR; }
  if (  ptr_ep->flags & USBD_EP_FLAG_BUSY       )     { return ARM_DRIVER_ERROR_BUSY; }

  ep_num =  EP_NUM(ep_addr);
  ep_dir = (ep_addr & ARM_USB_ENDPOINT_DIRECTION_MASK) == ARM_USB_ENDPOINT_DIRECTION_MASK;

  if (stall) {                          // Activate STALL
    ptr_ep->buffer      = NULL;
    ptr_ep->dataSize    = 0U;
    ptr_ep->bufferIndex = 0U;

    if (ep_dir) {                       // IN Endpoint
      if (OTG_DIEPCTL(ep_num) & OTG_HS_DIEPCTLx_EPENA) {
        OTG_DIEPCTL(ep_num)  |= OTG_HS_DIEPCTLx_STALL | OTG_HS_DIEPCTLx_EPDIS;
      } else {
        OTG_DIEPCTL(ep_num)  |= OTG_HS_DIEPCTLx_STALL;
      }

      USBD_FlushInEpFifo (ep_addr);
    } else {                            // OUT Endpoint
      OTG->DCTL |= OTG_HS_DCTL_SGONAK;                  // Set Global OUT NAK
      while (!(OTG->GINTSTS & OTG_HS_GINTSTS_GONAKEFF));

      // Stall OUT Endpoint
      if (OTG_DOEPCTL(ep_num) & OTG_HS_DOEPCTLx_EPENA) {
        OTG_DOEPCTL(ep_num)  |= OTG_HS_DOEPCTLx_STALL | OTG_HS_DOEPCTLx_EPDIS;
      } else {
        OTG_DOEPCTL(ep_num)  |= OTG_HS_DOEPCTLx_STALL;
      }

      OTG->DCTL |= OTG_HS_DCTL_CGONAK;                  // Clear global NAK
    }
  } else {                              // Clear STALL
    if (ep_dir) {                       // IN Endpoint
      if (OTG_DIEPCTL(ep_num) &  OTG_HS_DIEPCTLx_EPENA) { // If Endpoint enabled
        OTG_DIEPCTL(ep_num)   |= OTG_HS_DIEPCTLx_EPDIS;   // Disable Endpoint
      }

      USBD_FlushInEpFifo (ep_addr);

      // Set DATA0 PID for Interrupt and Bulk Endpoint
      if (((OTG_DIEPCTL(ep_num) & OTG_HS_DIEPCTLx_EPTYP_MSK) >> OTG_HS_DIEPCTLx_EPTYP_POS) > 1U) {
        OTG_DIEPCTL(ep_num) |= OTG_HS_DIEPCTLx_SD0PID;
      }

      OTG_DIEPCTL(ep_num) &= ~OTG_HS_DIEPCTLx_STALL;    // Clear Stall
    } else {                            // Clear OUT Endpoint stall
      // Set DATA0 PID for Interrupt and Bulk Endpoint
      if (((OTG_DOEPCTL(ep_num) & OTG_HS_DOEPCTLx_EPTYP_MSK) >> OTG_HS_DOEPCTLx_EPTYP_POS) > 1U) {
        OTG_DOEPCTL(ep_num) |= OTG_HS_DOEPCTLx_SD0PID;
      }
      OTG_DOEPCTL(ep_num) &= ~OTG_HS_DOEPCTLx_STALL;    // Clear Stall
    }
  }

  return ARM_DRIVER_OK;
}

/**
  \fn          int32_t USBD_EndpointTransfer (uint8_t ep_addr, uint8_t *data, uint32_t num)
  \brief       Read data from or Write data to USB Endpoint.
  \param[in]   ep_addr  Endpoint Address
                - ep_addr.0..3: Address
                - ep_addr.7:    Direction
  \param[out]  data Pointer to buffer for data to read or with data to write
  \param[in]   num  Number of data bytes to transfer
  \return      \ref execution_status
*/
static int32_t USBD_EndpointTransfer (uint8_t ep_addr, uint8_t *data, uint32_t num) {
  volatile ENDPOINT *ptr_ep;
  uint32_t           ep_num, ep_dir;

  ptr_ep = &ep[EP_ID(ep_addr)];

  if (!(otg_hs_state & OTG_HS_USBD_DRIVER_CONNECTED)) { return ARM_DRIVER_ERROR; }
  if (!(ptr_ep->flags & USBD_EP_FLAG_CONFIGURED))     { return ARM_DRIVER_ERROR; }
  if (  ptr_ep->flags & USBD_EP_FLAG_BUSY       )     { return ARM_DRIVER_ERROR_BUSY; }

  ep_dir = (ep_addr & ARM_USB_ENDPOINT_DIRECTION_MASK) == ARM_USB_ENDPOINT_DIRECTION_MASK;

  ptr_ep->flags      |= USBD_EP_FLAG_BUSY;              // Set Endpoint busy

  ptr_ep->bufferIndex = 0U;
  ptr_ep->buffer      = data;
  ptr_ep->dataSize    = num;

  if (ep_dir) {                         // IN Endpoint
    ep_num = EP_NUM(ep_addr);

    ptr_ep->in_NAK       =  1U;                         // Set IN Endpoint NAK flag
    OTG_DIEPCTL(ep_num) |=  OTG_HS_DIEPCTLx_CNAK;       // Clear NAK
    OTG_DIEPCTL(ep_num) |=  OTG_HS_DIEPCTLx_SNAK;       // Set NAK
    OTG->DIEPMSK        |=  OTG_HS_DIEPMSK_INEPNEM;     // Enable NAK effective interrupt
  } else {                              // OUT Endpoint
    USBD_EndpointReadSet(ep_addr);
  }

  return ARM_DRIVER_OK;
}

/**
  \fn          uint32_t USBD_EndpointTransferGetResult (uint8_t ep_addr)
  \brief       Get result of USB Endpoint transfer.
  \param[in]   ep_addr  Endpoint Address
                - ep_addr.0..3: Address
                - ep_addr.7:    Direction
  \return      number of successfully transfered data bytes
*/
static uint32_t USBD_EndpointTransferGetResult (uint8_t ep_addr) {
  return ep[EP_ID(ep_addr)].bufferIndex;
}

/**
  \fn          int32_t USBD_EndpointTransferAbort (uint8_t ep_addr)
  \brief       Abort current USB Endpoint transfer.
  \param[in]   ep_addr  Endpoint Address
                - ep_addr.0..3: Address
                - ep_addr.7:    Direction
  \return      \ref execution_status
*/
static int32_t USBD_EndpointTransferAbort (uint8_t ep_addr) {
  volatile ENDPOINT *ptr_ep;
  uint32_t           ep_num;

  ptr_ep = &ep[EP_ID(ep_addr)];

  if (!(otg_hs_state & OTG_HS_USBD_DRIVER_CONNECTED)) { return ARM_DRIVER_ERROR; }
  if (!(ptr_ep->flags & USBD_EP_FLAG_CONFIGURED))     { return ARM_DRIVER_ERROR; }

  ep_num = EP_NUM(ep_addr);

  ptr_ep->buffer      = NULL;
  ptr_ep->dataSize    = 0U;
  ptr_ep->bufferIndex = 0U;

  if (ep_addr & ARM_USB_ENDPOINT_DIRECTION_MASK) {
    if (OTG_DIEPCTL(ep_num) &  OTG_HS_DIEPCTLx_EPENA) { // If Endpoint enabled
      OTG_DIEPCTL(ep_num)   |= OTG_HS_DIEPCTLx_EPDIS;   // Disable Endpoint
    }

    OTG_DIEPCTL(ep_num)     |= OTG_HS_DIEPCTLx_SNAK;    // Set NAK

    USBD_FlushInEpFifo (ep_addr);
  } else {
    if (OTG_DOEPCTL(ep_num) &  OTG_HS_DOEPCTLx_EPENA) { // If Endpoint enabled
      OTG_DOEPCTL(ep_num)   |= OTG_HS_DOEPCTLx_EPDIS;   // Disable Endpoint
    }

    OTG_DOEPCTL(ep_num)     |= OTG_HS_DOEPCTLx_SNAK;    // Set NAK
  }

  ptr_ep->flags &= ~USBD_EP_FLAG_BUSY;                  // Clear Endpoint busy

  return ARM_DRIVER_OK;
}

/**
  \fn          uint16_t USBD_GetFrameNumber (void)
  \brief       Get current USB Frame Number.
  \return      Frame Number
*/
static uint16_t USBD_GetFrameNumber (void) {
  return ((OTG->DSTS & OTG_HS_DSTS_FNSOF_MSK) >> OTG_HS_DSTS_FNSOF_POS);
}

/**
  \fn          void USBD_HS_IRQ (uint32_t gintsts)
  \brief       USB Device Interrupt Routine (IRQ).
*/
void USBD_HS_IRQ (uint32_t gintsts) {
  volatile ENDPOINT *ptr_ep, *ptr_ep_in;
  uint32_t           val, num, msk, sz, ep_int, i;
  static uint32_t    IsoInIncomplete = 0U;

  // Reset interrupt
  if (gintsts & OTG_HS_GINTSTS_USBRST) {
    USBD_Reset();
    cbDeviceEvent(ARM_USBD_EVENT_RESET);
    OTG->GINTSTS = OTG_HS_GINTSTS_USBRST;
  }

  // Suspend interrupt
  if (gintsts & OTG_HS_GINTSTS_USBSUSP) {
    usbd_state.active = false;
    cbDeviceEvent(ARM_USBD_EVENT_SUSPEND);
    OTG->PCGCCTL |=  1U;
    OTG->GINTSTS  =  OTG_HS_GINTSTS_USBSUSP;
  }

  // Resume interrupt
  if (gintsts & OTG_HS_GINTSTS_WKUPINT) {
    usbd_state.active = true;
    cbDeviceEvent(ARM_USBD_EVENT_RESUME);
    OTG->PCGCCTL &= ~1U;
    OTG->GINTSTS  =  OTG_HS_GINTSTS_WKUPINT;
  }

  // Speed enumeration completed
  if (gintsts & OTG_HS_GINTSTS_ENUMDNE) {
    switch ((OTG->DSTS & OTG_HS_DSTS_ENUMSPD_MSK) >> OTG_HS_DSTS_ENUMSPD_POS) {
      case 0:
        usbd_state.speed  = ARM_USB_SPEED_HIGH;
        usbd_state.active = true;
        cbDeviceEvent(ARM_USBD_EVENT_HIGH_SPEED);
        break;
      case 3:
        usbd_state.speed  = ARM_USB_SPEED_FULL;
        usbd_state.active = true;
        break;
      default: break;
    }

    OTG->DCTL    |= OTG_HS_DCTL_CGINAK;     // Clear global IN NAK
    OTG->DCTL    |= OTG_HS_DCTL_CGONAK;     // Clear global OUT NAK
    OTG->GINTSTS  = OTG_HS_GINTSTS_ENUMDNE;
  }

  if (gintsts & OTG_HS_GINTSTS_RXFLVL) {
    OTG->GINTMSK &= ~OTG_HS_GINTMSK_RXFLVLM;

    val =  OTG->GRXSTSP;
    num =  val & 0x0FU;
    sz  = (val >> 4U) & 0x7FFU;

    switch ((val >> 17U) & 0x0FU) {
      // Setup packet
      case 6:
        // Read setup packet
        *(__packed uint32_t *)(setup_buf)      = OTG_RX_FIFO;
        *(__packed uint32_t *)(setup_buf + 4U) = OTG_RX_FIFO;

        // Analyze Setup packet for SetAddress
        if (setup_buf[0] == 0U) {
          if (setup_buf[1] == 5U)
            USBD_DeviceSetAddress(setup_buf[2]);
        }
        setup_flag = 1U;
      break;

      // OUT packet
      case 2:
        USBD_ReadFromFifo(num, sz);
        break;

      // Global OUT NAK
      case 1:
        break;

      // OUT transfer completed
      case 3:
        break;

      // SETUP transaction completed
      case 4:
        break;

      default:
        break;
    }
    OTG->GINTMSK |= OTG_HS_GINTMSK_RXFLVLM;
  }

  // OUT Packet
  if (gintsts & OTG_HS_GINTSTS_OEPINT) {
    msk = (((OTG->DAINT & OTG->DAINTMSK) >> 16U) & 0xFFFFU);
    num = 0U;

    do {
      if ((msk >> num) & 1U) {
        ep_int = OTG_DOEPINT(num) & OTG->DOEPMSK;
        ptr_ep = &ep[EP_ID(num)];
        // Endpoint disabled
        if (ep_int & OTG_HS_DOEPINTx_EPDISD) {
          if (OTG_EP_OUT_TYPE(num) == ARM_USB_ENDPOINT_ISOCHRONOUS) {
            // Set packet count and transfer size
            OTG_DOEPTSIZ(num) = (ptr_ep->packetCount << OTG_HS_DOEPTSIZx_PKTCNT_POS) |
                                (ptr_ep->maxPacketSize);

            // Set correct frame
            if ((USBD_GetFrameNumber() & 1U)) { OTG_DOEPCTL(num) |= OTG_HS_DOEPCTLx_SEVNFRM; }
            else                              { OTG_DOEPCTL(num) |= OTG_HS_DOEPCTLx_SODDFRM; }

            OTG_DOEPCTL(num) |= OTG_HS_DOEPCTLx_EPENA | OTG_HS_DOEPCTLx_CNAK;
          }
          OTG_DOEPINT(num) = OTG_HS_DOEPINTx_EPDISD;
        }

        // Setup phase done interrupt
        if (ep_int & OTG_HS_DOEPINTx_STUP) {
          ptr_ep->dataSize = 0U;
          OTG_DOEPINT(num) = OTG_HS_DOEPINTx_STUP;
          cbEndpointEvent(num, ARM_USBD_EVENT_SETUP);
        }

        // Transfer complete interrupt
        if (ep_int & OTG_HS_DOEPINTx_XFCR) {
          OTG_DOEPINT(num) = OTG_HS_DOEPINTx_XFCR;
          if (OTG_EP_OUT_TYPE(num) != ARM_USB_ENDPOINT_ISOCHRONOUS) {
            if (ptr_ep->dataSize != 0U)
              USBD_EndpointReadSet(num);
            else {
              ptr_ep->flags &= ~USBD_EP_FLAG_BUSY;
              cbEndpointEvent(num, ARM_USBD_EVENT_OUT);
            }
          }
        }
      }
      num++;
    } while (msk >> num);
  }

  // IN Packet
  if (gintsts & OTG_HS_GINTSTS_IEPINT) {
    msk = (OTG->DAINT & OTG->DAINTMSK & 0xFFFFU);
    num = 0U;

    do {
      if ((msk >> num) & 1U) {
        ep_int = OTG_DIEPINT(num) & OTG->DIEPMSK;
        ptr_ep = &ep[EP_ID(num | ARM_USB_ENDPOINT_DIRECTION_MASK)];
        // Endpoint Disabled
        if (ep_int & OTG_HS_DIEPINTx_EPDISD) {
          OTG_DIEPINT(num) = OTG_HS_DIEPINTx_EPDISD;

          if (OTG_EP_IN_TYPE(num) == ARM_USB_ENDPOINT_ISOCHRONOUS) {
            if ((IsoInIncomplete & (1U << num)) != 0U) {
              // Flush IN Endpoint FIFO and write new data if available
              USBD_FlushInEpFifo(num | ARM_USB_ENDPOINT_DIRECTION_MASK);
              if (ptr_ep->dataSize) { USBD_WriteToFifo(num | ARM_USB_ENDPOINT_DIRECTION_MASK); }
              IsoInIncomplete &= ~(1U << num);
            }
          }
        }

        // IN Endpoint NAK effective
        if (ep_int & OTG_HS_DIEPINTx_INEPNE) {
          if (ptr_ep->in_NAK) {
            ptr_ep->in_NAK = 0U;

            val = 0U;
            ptr_ep_in = &ep[0];
            for (i=0U; i < USBD_EP_NUM; i++) {
              val |= ptr_ep_in->in_NAK;
              ptr_ep_in += 2U;
            }

            // if no more forced NAKs, disable IN NAK effective interrupt
            if (!val) { OTG->DIEPMSK &= ~OTG_HS_DIEPMSK_INEPNEM; }

            // If Data available, write Data
            if ( ptr_ep->dataSize || (!ptr_ep->buffer && !ptr_ep->dataSize)) {
              USBD_WriteToFifo(num | ARM_USB_ENDPOINT_DIRECTION_MASK);
            }
          }
          OTG_DIEPINT(num) = OTG_HS_DIEPINTx_INEPNE;
        }

        // Transmit completed
        if (ep_int & OTG_HS_DIEPINTx_XFCR) {
          OTG_DIEPINT(num) = OTG_HS_DIEPINTx_XFCR;
          if (ptr_ep->dataSize == 0U) {
            ptr_ep->buffer =  NULL;
            ptr_ep->flags &= ~USBD_EP_FLAG_BUSY;
            cbEndpointEvent(num | ARM_USB_ENDPOINT_DIRECTION_MASK, ARM_USBD_EVENT_IN);
          } else {
            USBD_WriteToFifo(num | ARM_USB_ENDPOINT_DIRECTION_MASK);
          }
        }
      }
      num++;
    } while (msk >> num);
  }

  // End of periodic frame
  if (gintsts & OTG_HS_GINTSTS_EOPF) {
    for (num = 1U; num <= USBD_EP_NUM; num++) {

      if (OTG_EP_OUT_TYPE(num) != ARM_USB_ENDPOINT_ISOCHRONOUS) { continue; }
      if ((OTG_DOEPCTL(num) & OTG_HS_DOEPCTLx_USBAEP) == 0)     { continue; }

      // Incomplete Isochronous OUT transfer
      if (OTG->GINTSTS & OTG_HS_GINTSTS_IPXFR) {
        if ((USBD_GetFrameNumber() & 1U) == ((OTG_DOEPCTL(num) >> OTG_HS_DOEPCTLx_EONUM_POS) & 1U)) {
          if (OTG_DOEPCTL(num) & OTG_HS_DOEPCTLx_EPENA) {
            OTG_DOEPCTL(num) |= OTG_HS_DOEPCTLx_EPDIS;
          }
        }

      // Isochronous out transfer completed
      } else {
        if (ep[EP_ID(num)].dataSize != 0U) {
          USBD_EndpointReadSet(num);
        } else {
          ep[EP_ID(num)].flags &= ~USBD_EP_FLAG_BUSY;
          cbEndpointEvent(num, ARM_USBD_EVENT_OUT);
        }
      }
    }
    OTG->GINTSTS = OTG_HS_GINTSTS_EOPF | OTG_HS_GINTSTS_IPXFR;
  }

  // Incomplete isochronous IN transfer
  if (gintsts & OTG_HS_GINTSTS_IISOIXFR) {
    OTG->GINTSTS = OTG_HS_GINTSTS_IISOIXFR;
    for (num = 1U; num <= USBD_EP_NUM; num++) {

      if (OTG_EP_IN_TYPE(num) != ARM_USB_ENDPOINT_ISOCHRONOUS)  { continue; }
      if ((OTG_DIEPCTL(num)   &  OTG_HS_DIEPCTLx_USBAEP) == 0U) { continue; }

      if (OTG_DIEPCTL(num) & OTG_HS_DIEPCTLx_EPENA) {
        if ((USBD_GetFrameNumber() & 1U) == ((OTG_DIEPCTL(num) >> OTG_HS_DIEPCTLx_EONUM_POS) & 1U)) {

          IsoInIncomplete |= (1U << num);
          OTG_DIEPCTL(num)|= OTG_HS_DIEPCTLx_EPDIS | OTG_HS_DIEPCTLx_SNAK;
        }
      }
    }
  }
#ifdef ARM_USBD_VBUS_DETECT
  if (gintsts & OTG_HS_GINTSTS_SRQINT) {
    cbDeviceEvent(ARM_USBD_EVENT_VBUS_ON);
    OTG->GINTSTS = OTG_HS_GINTSTS_SRQINT;
  }

  if (gintsts & OTG_HS_GINTSTS_OTGINT) {
    if (OTG->GOTGINT & OTG_HS_GOTGINT_SEDET) {
      cbDeviceEvent(ARM_USBD_EVENT_VBUS_OFF);
    }
    OTG->GOTGINT = OTG->GOTGINT;
  }
#endif
}

ARM_DRIVER_USBD Driver_USBD1 = {
  USBD_GetVersion,
  USBD_GetCapabilities,
  USBD_Initialize,
  USBD_Uninitialize,
  USBD_PowerControl,
  USBD_DeviceConnect,
  USBD_DeviceDisconnect,
  USBD_DeviceGetState,
  USBD_DeviceRemoteWakeup,
  USBD_DeviceSetAddress,
  USBD_ReadSetupPacket,
  USBD_EndpointConfigure,
  USBD_EndpointUnconfigure,
  USBD_EndpointStall,
  USBD_EndpointTransfer,
  USBD_EndpointTransferGetResult,
  USBD_EndpointTransferAbort,
  USBD_GetFrameNumber
};
