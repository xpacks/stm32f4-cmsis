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
 * $Date:        14. September 2014
 * $Revision:    V2.02
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

#include <stdint.h>
#include <string.h>
#include "cmsis_os.h"

#include "OTG_HS_STM32F4xx.h"

#include "Driver_USBD.h"

extern uint8_t otg_hs_role;
extern uint8_t otg_hs_state;

extern bool OTG_HS_PinsConfigure   (uint8_t pins_mask);
extern bool OTG_HS_PinsUnconfigure (uint8_t pins_mask);
extern bool OTG_HS_PinVbusOnOff    (bool state);

#define OTG                         OTG_HS


/* USBD Driver ****************************************************************/

#define ARM_USBD_DRV_VERSION ARM_DRIVER_VERSION_MAJOR_MINOR(2,2) /* USBD driver version */

/* Driver Version */
static const ARM_DRIVER_VERSION usbd_driver_version = { ARM_USBD_API_VERSION, ARM_USBD_DRV_VERSION };

/* Driver Capabilities */
static const ARM_USBD_CAPABILITIES usbd_driver_capabilities = {
#ifdef ARM_USBD_VBUS_DETECT
  1,  // vbus_detection
  1,  // event_vbus_on
  1,  // event_vbus_off
#else
  0,  // vbus_detection
  0,  // event_vbus_on
  0,  // event_vbus_off
#endif
};

// Number of endpoints
#ifndef USBD_EP_NUM
#define USBD_EP_NUM         5
#endif

#define OTG_RX_FIFO_SIZE        1024
#define OTG_TX0_FIFO_SIZE       64
#define OTG_TX1_FIFO_SIZE       512
#define OTG_TX2_FIFO_SIZE       512
#define OTG_TX3_FIFO_SIZE       512
#define OTG_TX4_FIFO_SIZE       512
#define OTG_TX5_FIFO_SIZE       512

#define OTG_TX_FIFO(n)          *((volatile uint32_t*)(OTG_HS_BASE + 0x1000 + n*0x1000))
#define OTG_RX_FIFO             *((volatile uint32_t*)(OTG_HS_BASE + 0x1000))

#define OTG_DIEPTSIZ(EPNum)     *(&OTG->DIEPTSIZ0 + EPNum * 8)
#define OTG_DIEPCTL(EPNum)      *(&OTG->DIEPCTL0  + EPNum * 8)
#define OTG_DTXFSTS(EPNum)      *(&OTG->DTXFSTS0  + EPNum * 8)
#define OTG_DOEPTSIZ(EPNum)     *(&OTG->DOEPTSIZ0 + EPNum * 8)
#define OTG_DOEPCTL(EPNum)      *(&OTG->DOEPCTL0  + EPNum * 8)
#define OTG_DIEPINT(EPNum)      *(&OTG->DIEPINT0  + EPNum * 8)
#define OTG_DOEPINT(EPNum)      *(&OTG->DOEPINT0  + EPNum * 8)

#define OTG_EP_IN_TYPE(num)      ((OTG_DIEPCTL(num) >> 18) & 3)
#define OTG_EP_OUT_TYPE(num)     ((OTG_DOEPCTL(num) >> 18) & 3)

/* USB Device Endpoint Flags */
#define USBD_HS_EP_FLAG_CONFIGURED  (1 << 0)
#define USBD_HS_EP_FLAG_BUSY        (1 << 1)

/* Endpoint structure */
typedef struct {
  uint8_t  *buffer;
  uint32_t  bufferIndex;
  uint32_t  dataSize;
  uint32_t  maxPacketSize;
  uint8_t   packetCount;
  uint8_t   in_NAK;
  uint8_t   flags;
} ENDPOINT;

/* Local variables and structures */
static ARM_USBD_SignalDeviceEvent_t   cbDeviceEvent;
static ARM_USBD_SignalEndpointEvent_t cbEndpointEvent;

static ARM_USBD_STATE UsbdState = {0};

         static uint8_t  setup_buf[8];
volatile static uint8_t  setup_flag = 0;

volatile static ENDPOINT OutEndpoint[USBD_EP_NUM + 1];
volatile static ENDPOINT InEndpoint [USBD_EP_NUM + 1];

/* Function Prototypes */
static uint16_t  ARM_USBD_GetFrameNumber (void);


/* Local Functions */

/**
  \fn          bool USBD_FlushInEpFifo (uint8_t ep_addr)
  \brief       Flush IN endpoint fifo
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

  // Clear endpoint mask registers
  OTG->DOEPMSK = 0;
  OTG->DIEPMSK = 0;

  for (i = 1; i <= USBD_EP_NUM; i++) {
    if (OTG_DOEPCTL(i) & OTG_HS_DOEPCTLx_EPENA) {
      OTG_DOEPCTL(i)   = OTG_HS_DOEPCTLx_EPDIS | // Endpoint disable
                         OTG_HS_DOEPCTLx_SNAK;   // Endpoint set NAK
    }
    if (OTG_DIEPCTL(i) & OTG_HS_DIEPCTLx_EPENA) {
      OTG_DIEPCTL(i)   = OTG_HS_DIEPCTLx_EPDIS | // Endpoint disable
                         OTG_HS_DIEPCTLx_SNAK;   // Endpoint set NAK
    }
    USBD_FlushInEpFifo(i);

    // Reset endpoint resources
    OutEndpoint[i].buffer         = NULL;
    OutEndpoint[i].dataSize       = 0;
    OutEndpoint[i].maxPacketSize  = 0;
    OutEndpoint[i].flags          = 0;
    InEndpoint[i].buffer          = NULL;
    InEndpoint[i].dataSize        = 0;
    InEndpoint[i].maxPacketSize   = 0;
    InEndpoint[i].flags           = 0;

    // Clear IN endpoint interrupts
    OTG_DIEPINT(i) = OTG_HS_DIEPINTx_XFCR    |
                     OTG_HS_DIEPINTx_EPDISD  |
                     OTG_HS_DIEPINTx_TOC     |
                     OTG_HS_DIEPINTx_ITTXFE  |
                     OTG_HS_DIEPINTx_INEPNE  |
                     OTG_HS_DIEPINTx_TXFE;

    // Clear OUT endpoint interrupts
    OTG_DOEPINT(i) = OTG_HS_DOEPINTx_XFCR    |
                     OTG_HS_DOEPINTx_EPDISD  |
                     OTG_HS_DOEPINTx_STUP    |
                     OTG_HS_DOEPINTx_OTEPDIS |
                     OTG_HS_DOEPINTx_B2BSTUP;
  }

  // Set device address to 0
  OTG->DCFG       = (OTG->DCFG & ~OTG_HS_DCFG_DAD_MSK) | (0 << OTG_HS_DCFG_DAD_POS);
  OTG->DAINTMSK   =  OTG_HS_DAINT_IEPINT(0) | // Enable IN endpoint0 interrupt
                     OTG_HS_DAINT_OEPINT(0);  // Enable OUT endpoint0 interrupt

  // Enable Setup phase done, Out endpoint disabled and Out transfer
  // complete interrupt
  OTG->DOEPMSK    =  OTG_HS_DOEPMSK_STUPM    |
                     OTG_HS_DOEPMSK_EPDM     |
                     OTG_HS_DOEPMSK_XFRCM;

  // Enable In endpoint disable and In transfer complete interrupt
  OTG->DIEPMSK    =  OTG_HS_DIEPMSK_EPDM     |
                     OTG_HS_DIEPMSK_XFRCM;

  // Configure FIFOs
  OTG->GRXFSIZ    =  OTG_RX_FIFO_SIZE   / 4;
  OTG->TX0FSIZ    = (OTG_RX_FIFO_SIZE   / 4) |
                   ((OTG_TX0_FIFO_SIZE  / 4) << OTG_HS_DIEPTXFx_INEPTXFD_POS);

  OTG->DIEPTXF1   = ((OTG_RX_FIFO_SIZE + OTG_TX0_FIFO_SIZE) / 4) |
                    ((OTG_TX1_FIFO_SIZE / 4) << OTG_HS_DIEPTXFx_INEPTXFD_POS);

  OTG->DIEPTXF2   = ((OTG_RX_FIFO_SIZE + OTG_TX0_FIFO_SIZE + OTG_TX1_FIFO_SIZE) / 4) |
                    ((OTG_TX2_FIFO_SIZE / 4) << OTG_HS_DIEPTXFx_INEPTXFD_POS);

  OTG->DIEPTXF3   = ((OTG_RX_FIFO_SIZE + OTG_TX0_FIFO_SIZE + OTG_TX1_FIFO_SIZE +
                      OTG_TX2_FIFO_SIZE) / 4) | ((OTG_TX3_FIFO_SIZE / 4) << OTG_HS_DIEPTXFx_INEPTXFD_POS);

  OTG->DIEPTXF4   = ((OTG_RX_FIFO_SIZE + OTG_TX0_FIFO_SIZE + OTG_TX1_FIFO_SIZE +
                      OTG_TX2_FIFO_SIZE + OTG_TX3_FIFO_SIZE) / 4) |
                    ((OTG_TX4_FIFO_SIZE / 4) << OTG_HS_DIEPTXFx_INEPTXFD_POS);

  OTG->DIEPTXF5   = ((OTG_RX_FIFO_SIZE + OTG_TX0_FIFO_SIZE + OTG_TX1_FIFO_SIZE +
                      OTG_TX2_FIFO_SIZE + OTG_TX3_FIFO_SIZE + OTG_TX4_FIFO_SIZE) / 4) |
                    ((OTG_TX5_FIFO_SIZE / 4) << OTG_HS_DIEPTXFx_INEPTXFD_POS);
}

/**
\fn          void USBD_EndpointReadSet (uint8_t ep_addr)
\brief       Set Endpoint for next read.
\param[in]   ep_addr specifies Endpoint Address
              ep_addr.0..3: Address
              ep_addr.7:    Direction
*/
static void USBD_EndpointReadSet (uint8_t ep_addr) {
  uint32_t sz;

  // Set packet count and transfer size
  if  (OutEndpoint[ep_addr].dataSize > OutEndpoint[ep_addr].maxPacketSize) sz = OutEndpoint[ep_addr].maxPacketSize;
  else                                                                     sz = OutEndpoint[ep_addr].dataSize;
  if (ep_addr != 0) {
    OTG_DOEPTSIZ(ep_addr) = (OutEndpoint[ep_addr].packetCount << OTG_HS_DOEPTSIZx_PKTCNT_POS) |
                             sz;
  } else {
    OTG_DOEPTSIZ(ep_addr) = (OutEndpoint[ep_addr].packetCount << OTG_HS_DOEPTSIZx_PKTCNT_POS) |
                            (3 << OTG_HS_DOEPTSIZ0_STUPCNT_POS)|
                             sz;
  }
  // Set correct frame for isochronous endpoint
  if (OTG_EP_OUT_TYPE(ep_addr) == ARM_USB_ENDPOINT_ISOCHRONOUS) {
    if ((ARM_USBD_GetFrameNumber() & 1)) OTG_DOEPCTL(ep_addr) |= OTG_HS_DOEPCTLx_SEVNFRM;
    else                                 OTG_DOEPCTL(ep_addr) |= OTG_HS_DOEPCTLx_SODDFRM;
  }
  // Clear NAK  and enable endpoint
  OTG_DOEPCTL(ep_addr) |= OTG_HS_DOEPCTLx_EPENA | OTG_HS_DOEPCTLx_CNAK;
}

/**
  \fn          static int32_t USBD_ReadFromFifo (uint8_t ep_addr)
  \brief       Read data from USB Endpoint.
  \param[in]   ep_addr specifies Endpoint Address
                ep_addr.0..3: Address
                ep_addr.7:    Direction
  \param[in]   sz specifies data size to be read from fifo
  \return      number of data bytes read, error code if negative
*/
static int32_t USBD_ReadFromFifo (uint8_t ep_addr, uint32_t sz) {
  uint32_t i, val, residue;
  uint8_t  tmpBuf[4];

  // Check if Endpoint is activated and buffer available
  if ((OTG_DOEPCTL(ep_addr) & OTG_HS_DOEPCTLx_USBAEP) == 0) return 0;
  if (OutEndpoint[ep_addr].buffer == NULL)                  return 0;

  if (sz > OutEndpoint[ep_addr].dataSize)
    sz = OutEndpoint[ep_addr].dataSize;

  // If Isochronous Endpoint
  if (OTG_EP_OUT_TYPE(ep_addr) == ARM_USB_ENDPOINT_ISOCHRONOUS) {
    val =  OutEndpoint[ep_addr].packetCount -
          ((OTG_DOEPTSIZ(ep_addr) & OTG_HS_DOEPTSIZx_PKTCNT_MSK) >> OTG_HS_DOEPTSIZx_PKTCNT_POS);

    switch ((OTG_DOEPTSIZ(ep_addr) & OTG_HS_DOEPTSIZx_RXDPID_MSK) >> OTG_HS_DOEPTSIZx_RXDPID_POS) {
      // Data0
      case 0:
        if (val != 1) sz = 0; break;
       // Data1
      case 2:
        if (val != 2) sz = 0; break;
      // Data2
      case 1:
        if (val != 3) sz = 0; break;
    }
  }

  // Copy data from fifo
  for (i = 0; i < (uint32_t)(sz / 4); i++) {
    *((__packed uint32_t *)(OutEndpoint[ep_addr].buffer + OutEndpoint[ep_addr].bufferIndex)) = OTG_RX_FIFO;
    OutEndpoint[ep_addr].bufferIndex += 4;
  }
  // if data size is not equal n*4
  residue = sz % 4;
  if (residue != 0) {
    *((__packed uint32_t *)tmpBuf) = OTG_RX_FIFO;
    for (i = 0; i < residue; i++) {
      OutEndpoint[ep_addr].buffer[OutEndpoint[ep_addr].bufferIndex++] = tmpBuf[i];
    }
  }

  if (sz != OutEndpoint[ep_addr].maxPacketSize) OutEndpoint[ep_addr].dataSize  = 0;
  else                                          OutEndpoint[ep_addr].dataSize -= sz;

  return (sz);
}

/**
  \fn          void USBD_WriteToFifo (uint8_t ep_addr)
  \brief       Write data to endpoint fifo.
  \param[in]   ep_addr specifies Endpoint Address
                ep_addr.0..3: Address
                ep_addr.7:    Direction
*/
static void USBD_WriteToFifo (uint8_t ep_addr) {
  uint8_t  ep_num;
  uint32_t sz, i;

  ep_num = ep_addr & ARM_USB_ENDPOINT_NUMBER_MASK;

  if (InEndpoint[ep_num].dataSize > InEndpoint[ep_num].maxPacketSize)
    sz = InEndpoint[ep_num].maxPacketSize;
  else
    sz = InEndpoint[ep_num].dataSize;

  // Check if enough space in fifo
  if ((OTG_DTXFSTS(ep_num) * 4) < sz) return;

  // Set transfer size and packet count
  OTG_DIEPTSIZ(ep_num) = (InEndpoint[ep_num].packetCount << OTG_HS_DIEPTSIZx_PKTCNT_POS) |
                         (InEndpoint[ep_num].packetCount << OTG_HS_DIEPTSIZx_MCNT_POS)   |
                          sz;

  // Set correct frame for isochronous endpoint
  if (OTG_EP_IN_TYPE(ep_num) == ARM_USB_ENDPOINT_ISOCHRONOUS) {
    if (ARM_USBD_GetFrameNumber() & 1) OTG_DIEPCTL(ep_num) |= OTG_HS_DIEPCTLx_SEVNFRM;
    else OTG_DIEPCTL(ep_num)                               |= OTG_HS_DIEPCTLx_SODDFRM;
  }

  // Enable endpoint and clear NAK
  OTG_DIEPCTL(ep_num) |= OTG_HS_DIEPCTLx_EPENA | OTG_HS_DIEPCTLx_CNAK;

  // Copy data to fifo
  for (i = 0; i < (uint32_t)((sz+3)/4); i++) {
    OTG_TX_FIFO(ep_num) = *(__packed uint32_t *)(InEndpoint[ep_num].buffer + InEndpoint[ep_num].bufferIndex + 4*i);
  }

  InEndpoint[ep_num].bufferIndex += sz;
  InEndpoint[ep_num].dataSize    -= sz;
}


/* USB Device Driver Functions */

/**
  \fn          ARM_DRIVER_VERSION ARM_USBD_GetVersion (void)
  \brief       Get driver version.
  \return      \ref ARM_DRIVER_VERSION
*/
static ARM_DRIVER_VERSION ARM_USBD_GetVersion (void) { return usbd_driver_version; }

/**
  \fn          ARM_USBD_CAPABILITIES ARM_USBD_GetCapabilities (void)
  \brief       Get driver capabilities.
  \return      \ref ARM_USBD_CAPABILITIES
*/
static ARM_USBD_CAPABILITIES ARM_USBD_GetCapabilities (void) { return usbd_driver_capabilities; }

/**
  \fn          int32_t ARM_USBD_Initialize (ARM_USBD_SignalDeviceEvent_t   cb_device_event,
                                            ARM_USBD_SignalEndpointEvent_t cb_endpoint_event)
  \brief       Initialize USB Device Interface.
  \param[in]   cb_device_event    Pointer to \ref ARM_USBD_SignalDeviceEvent
  \param[in]   cb_endpoint_event  Pointer to \ref ARM_USBD_SignalEndpointEvent
  \return      \ref execution_status
*/
static int32_t ARM_USBD_Initialize (ARM_USBD_SignalDeviceEvent_t   cb_device_event,
                                    ARM_USBD_SignalEndpointEvent_t cb_endpoint_event) {

  if (otg_hs_state & OTG_HS_USBD_DRIVER_INITIALIZED) return ARM_DRIVER_OK;
  if (otg_hs_state)                                  return ARM_DRIVER_ERROR;

  cbDeviceEvent   = cb_device_event;
  cbEndpointEvent = cb_endpoint_event;

  otg_hs_role     = ARM_USB_ROLE_DEVICE;
  OTG_HS_PinsConfigure (ARM_USB_PIN_DP | ARM_USB_PIN_DM);

  otg_hs_state |= OTG_HS_USBD_DRIVER_INITIALIZED;

  return ARM_DRIVER_OK;
}

/**
  \fn          int32_t ARM_USBD_Uninitialize (void)
  \brief       De-initialize USB Device Interface.
  \return      \ref execution_status
*/
static int32_t ARM_USBD_Uninitialize (void) {

  if (!(otg_hs_state & OTG_HS_USBD_DRIVER_INITIALIZED)) return ARM_DRIVER_OK;
  if (  otg_hs_state & OTG_HS_USBD_DRIVER_POWERED     ) return ARM_DRIVER_ERROR;

  OTG_HS_PinsUnconfigure (ARM_USB_PIN_DP | ARM_USB_PIN_DM);
  otg_hs_role   =  ARM_USB_ROLE_NONE;

  otg_hs_state &= ~OTG_HS_USBD_DRIVER_INITIALIZED;

  return ARM_DRIVER_OK;
}

/**
  \fn          int32_t ARM_USBD_PowerControl (ARM_POWER_STATE state)
  \brief       Control USB Device Interface Power.
  \param[in]   state  Power state
  \return      \ref execution_status
*/
static int32_t ARM_USBD_PowerControl (ARM_POWER_STATE state) {
  uint32_t tick;

  if (!(otg_hs_state & OTG_HS_USBD_DRIVER_INITIALIZED)) return ARM_DRIVER_ERROR;

  switch (state) {
    case ARM_POWER_OFF:
      if (!(otg_hs_state & OTG_HS_USBD_DRIVER_POWERED)) return ARM_DRIVER_OK;
      otg_hs_state  &= ~OTG_HS_USBD_DRIVER_POWERED;
      NVIC_DisableIRQ(OTG_HS_IRQn);             // Disable OTG interrupt

      RCC->AHB1ENR  |=  RCC_AHB1ENR_OTGHSEN;    // Enable OTG HS clock
      osDelay(1);                               // Wait 1 ms

      OTG->DCTL     |=  OTG_HS_DCTL_SDIS;       // Soft disconnect enabled
      OTG->GCCFG    &= ~(OTG_HS_GCCFG_VBUSBSEN |// Disable VBUS sensing device "B"
                         OTG_HS_GCCFG_PWRDWN);  // Power down activated

      OTG->GAHBCFG  &= ~OTG_HS_GAHBCFG_GINT;

      // Core soft reset
      OTG->GRSTCTL  |=  OTG_HS_GRSTCTL_CSRST;

      tick = osKernelSysTick();
      do {
        if ((OTG->GRSTCTL & OTG_HS_GRSTCTL_CSRST) == 0) break;
      } while ((osKernelSysTick() - tick) < osKernelSysTickMicroSec(10000));

      if (OTG->GRSTCTL & OTG_HS_GRSTCTL_CSRST) return ARM_DRIVER_ERROR;

#ifdef MX_USB_OTG_HS_ULPI_D7_Pin
      /* External ULPI High-speed PHY */
      RCC->AHB1ENR  &= ~RCC_AHB1ENR_OTGHSULPIEN;// Disable OTG HS ULPI clock
#endif

      RCC->AHB1RSTR |=  RCC_AHB1ENR_OTGHSEN;    // Reset OTG HS clock
      osDelay(1);                               // Wait 1 ms
      RCC->AHB1RSTR &= ~RCC_AHB1ENR_OTGHSEN;
      RCC->AHB1ENR  &= ~RCC_AHB1ENR_OTGHSEN;    // OTG HS clock disable

      break;

    case ARM_POWER_LOW:
      return ARM_DRIVER_ERROR_UNSUPPORTED;

    case ARM_POWER_FULL:
      if (  otg_hs_state & OTG_HS_USBD_DRIVER_POWERED ) return ARM_DRIVER_OK;
      RCC->AHB1ENR  |=  RCC_AHB1ENR_OTGHSEN;    // OTG HS clock enable
      RCC->AHB1RSTR |=  RCC_AHB1RSTR_OTGHRST;   // Reset OTG HS clock
      osDelay(1);                               // Wait 1 ms
      RCC->AHB1RSTR &= ~RCC_AHB1RSTR_OTGHRST;   // Clear reset OTG HS clock
      RCC->AHB1ENR  |=  RCC_AHB1ENR_OTGHSEN;    // Enable OTG HS clock

#ifdef MX_USB_OTG_HS_ULPI_D7_Pin
      /* External ULPI High-speed PHY */
      RCC->AHB1ENR  |=  RCC_AHB1ENR_OTGHSULPIEN;// Enable OTG HS ULPI clock
#else
      /* On-chip Full-speed PHY */
      OTG->GUSBCFG   |=  OTG_HS_GUSBCFG_PHSEL  |    // Full-speed transceiver
                         OTG_HS_GUSBCFG_PHYLPCS;    // 48 MHz external clock
#endif

      // Wait until AHB Master state machine is in the idle condition
      tick = osKernelSysTick();
      do {
        if (OTG->GRSTCTL & OTG_HS_GRSTCTL_AHBIDL) break;
      } while ((osKernelSysTick() - tick) < osKernelSysTickMicroSec(100000));

      if ((OTG->GRSTCTL & OTG_HS_GRSTCTL_AHBIDL) == 0) return ARM_DRIVER_ERROR;

      // Core soft reset
      OTG->GRSTCTL  |=  OTG_HS_GRSTCTL_CSRST;

      tick = osKernelSysTick();
      do {
        if ((OTG->GRSTCTL & OTG_HS_GRSTCTL_CSRST) == 0) break;
      } while ((osKernelSysTick() - tick) < osKernelSysTickMicroSec(10000));

      if (OTG->GRSTCTL & OTG_HS_GRSTCTL_CSRST) return ARM_DRIVER_ERROR;

      osDelay(3);                               // Wait 3 ms

      OTG->GAHBCFG  &= ~OTG_HS_GAHBCFG_GINT;    // Disable interrupts
      OTG->GCCFG    |=  OTG_HS_GCCFG_VBUSBSEN;  // Enable VBUS sensing device "B"
#ifdef MX_USB_OTG_HS_ULPI_D7_Pin
      /* External ULPI High-speed PHY */
      OTG->GCCFG    &= ~OTG_HS_GCCFG_NOVBUSSENS;// Disable VBUS sense
#endif
      OTG->DCTL     |=  OTG_HS_DCTL_SDIS;       // Soft disconnect enabled

      // Set turnaround time and force device mode
      OTG->GUSBCFG   = (OTG->GUSBCFG & ~OTG_HS_GUSBCFG_TRDT_MSK) |
                        OTG_HS_GUSBCFG_TRDT(5)                   |
                        OTG_HS_GUSBCFG_FDMOD;

      osDelay(100);

#ifdef MX_USB_OTG_HS_ULPI_D7_Pin
      /* External ULPI High-speed PHY */
      OTG->DCFG     &= ~OTG_HS_DCFG_DSPD_MSK;    // High speed
#else
      /* On-chip Full-speed PHY */
      OTG->DCFG     |=  OTG_HS_DCFG_DSPD_MSK;    // Full Speed
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
      if (OTG->GOTGCTL & OTG_HS_GOTGCTL_BSVLD)  // If B-session valid
        cbDeviceEvent(ARM_USBD_EVENT_VBUS_ON);
      else
        cbDeviceEvent(ARM_USBD_EVENT_VBUS_OFF);
#endif
      otg_hs_state  |=  OTG_HS_USBD_DRIVER_POWERED;

      // Enable interrupts
      NVIC_EnableIRQ(OTG_HS_IRQn);
      OTG->GAHBCFG  |=  OTG_HS_GAHBCFG_GINT     |
                        OTG_HS_GAHBCFG_TXFELVL;

      break;
  }
  return ARM_DRIVER_OK;
}

/**
  \fn          int32_t ARM_USBD_DeviceConnect (void)
  \brief       Connect USB Device.
  \return      \ref execution_status
*/
static int32_t ARM_USBD_DeviceConnect (void) {

  if (!(otg_hs_state & OTG_HS_USBD_DRIVER_POWERED) ) return ARM_DRIVER_ERROR;
  if (  otg_hs_state & OTG_HS_USBD_DRIVER_CONNECTED) return ARM_DRIVER_OK;

  OTG->DCTL    &= ~OTG_HS_DCTL_SDIS;         // Soft disconnect disabled
  OTG->GCCFG   |=  OTG_HS_GCCFG_PWRDWN;

  otg_hs_state |=  OTG_HS_USBD_DRIVER_CONNECTED;

  return ARM_DRIVER_OK;
}

/**
  \fn          int32_t ARM_USBD_DeviceDisconnect (void)
  \brief       Disconnect USB Device.
  \return      \ref execution_status
*/
static int32_t ARM_USBD_DeviceDisconnect (void) {

  if (!(otg_hs_state & OTG_HS_USBD_DRIVER_POWERED)  ) return ARM_DRIVER_ERROR;
  if (!(otg_hs_state & OTG_HS_USBD_DRIVER_CONNECTED)) return ARM_DRIVER_OK;

  OTG->DCTL    |=   OTG_HS_DCTL_SDIS;        // Soft disconnect enabled
  OTG->GCCFG   &= ~ OTG_HS_GCCFG_PWRDWN;     // Power down activated

  UsbdState.active = 0;

  otg_hs_state &= ~OTG_HS_USBD_DRIVER_CONNECTED;

  return ARM_DRIVER_OK;
}

/**
  \fn          ARM_USBD_STATE ARM_USBD_DeviceGetState (void)
  \brief       Get current USB Device State.
  \return      Device State \ref ARM_USBD_STATE
*/
static ARM_USBD_STATE ARM_USBD_DeviceGetState (void) {
  ARM_USBD_STATE UsbdState0 = {0};

  if (!(otg_hs_state & OTG_HS_USBD_DRIVER_CONNECTED)) return UsbdState0;

#ifdef ARM_USBD_VBUS_DETECT
  if (OTG->GOTGCTL & OTG_HS_GOTGCTL_BSVLD)  // If B-session valid
    UsbdState.vbus = 1;
  else
    UsbdState.vbus = 0;
#else
  UsbdState.vbus = 0;
#endif

  return UsbdState;
}

/**
  \fn          int32_t ARM_USBD_DeviceRemoteWakeup (void)
  \brief       Trigger USB Remote Wakeup.
  \return      \ref execution_status
*/
static int32_t ARM_USBD_DeviceRemoteWakeup (void) {

  if (!(otg_hs_state & OTG_HS_USBD_DRIVER_CONNECTED)) return ARM_DRIVER_ERROR;

  OTG->DCTL   |=   OTG_HS_DCTL_RWUSIG;      // Remote wakeup signalling
  osDelay(5);
  OTG->DCTL   &=  ~OTG_HS_DCTL_RWUSIG;
  return ARM_DRIVER_OK;
}

/**
  \fn          int32_t ARM_USBD_DeviceSetAddress (uint8_t dev_addr)
  \brief       Set USB Device Address.
  \param[in]   dev_addr  Device Address
  \return      \ref execution_status
*/
static int32_t ARM_USBD_DeviceSetAddress (uint8_t dev_addr) {

  if (!(otg_hs_state & OTG_HS_USBD_DRIVER_CONNECTED)) return ARM_DRIVER_ERROR;

  OTG->DCFG = (OTG->DCFG & ~OTG_HS_DCFG_DAD_MSK) |
               OTG_HS_DCFG_DAD(dev_addr);
  return ARM_DRIVER_OK;
}

/**
  \fn          int32_t ARM_USBD_ReadSetupPacket (uint8_t *setup)
  \brief       Read setup packet received over Control Endpoint.
  \param[out]  setup  Pointer to buffer for setup packet
  \return      \ref execution_status
*/
static int32_t ARM_USBD_ReadSetupPacket (uint8_t *setup) {

  if (!(otg_hs_state & OTG_HS_USBD_DRIVER_CONNECTED)) return ARM_DRIVER_ERROR;

  if (!setup_flag)                                    return ARM_DRIVER_ERROR;

  setup_flag = 0;
  memcpy(setup, setup_buf, 8);

  if (setup_flag) {
    // Interrupted with new setup packet
    return ARM_DRIVER_ERROR;
  }

  return ARM_DRIVER_OK;
}

/**
  \fn          int32_t ARM_USBD_EndpointConfigure (uint8_t  ep_addr,
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
static int32_t ARM_USBD_EndpointConfigure (uint8_t  ep_addr,
                                           uint8_t  ep_type,
                                           uint16_t ep_max_packet_size) {
  uint32_t val, dir;
  uint8_t num;

  if (!(otg_hs_state & OTG_HS_USBD_DRIVER_CONNECTED)) return ARM_DRIVER_ERROR;
  if (ep_addr & ARM_USB_ENDPOINT_DIRECTION_MASK) {
    if (InEndpoint[ep_addr & ARM_USB_ENDPOINT_NUMBER_MASK].flags  & USBD_HS_EP_FLAG_CONFIGURED) return ARM_DRIVER_OK;
    if (InEndpoint[ep_addr & ARM_USB_ENDPOINT_NUMBER_MASK].flags  & USBD_HS_EP_FLAG_BUSY      ) return ARM_DRIVER_ERROR_BUSY;
  } else {
    if (OutEndpoint[ep_addr].flags & USBD_HS_EP_FLAG_CONFIGURED) return ARM_DRIVER_OK;
    if (OutEndpoint[ep_addr].flags & USBD_HS_EP_FLAG_BUSY      ) return ARM_DRIVER_ERROR_BUSY;
  }

  num  = ep_addr & ARM_USB_ENDPOINT_NUMBER_MASK;
  val  = ep_max_packet_size & ARM_USB_ENDPOINT_MAX_PACKET_SIZE_MASK;

  if (ep_addr & ARM_USB_ENDPOINT_DIRECTION_MASK) dir = 16;
  else                                           dir = 0;

  // Check if endpoint is enabled
  if (num > USBD_EP_NUM) return ARM_DRIVER_ERROR;

  // IN Endpoint Configuration
  if (dir) {
    // Set IN Endpoint resources
    InEndpoint[num].buffer        = NULL;
    InEndpoint[num].dataSize      = 0;
    InEndpoint[num].maxPacketSize = val;

    if (OTG_EP_IN_TYPE(num) == ARM_USB_ENDPOINT_ISOCHRONOUS) {
      InEndpoint[num].packetCount = (ep_max_packet_size &
                                     ARM_USB_ENDPOINT_MICROFRAME_TRANSACTIONS_MASK) >> 11;
    } else InEndpoint[num].packetCount = 1;

    InEndpoint[num].in_NAK = 0;

    // Configure IN endpoint
    OTG_DIEPCTL(num) = (num     <<  OTG_HS_DIEPCTLx_TXFNUM_POS) |   // Fifo Number
                       (ep_type <<  OTG_HS_DIEPCTLx_EPTYP_POS)  |   // Endpoint Type
                       (val);                                       // Max Packet Size

    // Set DATA0 PID for Interrupt or Bulk Endpoint
    if (ep_type >= ARM_USB_ENDPOINT_BULK) {
      OTG_DIEPCTL(num) |= OTG_HS_DIEPCTLx_SD0PID;
    }

    OTG_DIEPCTL(num)   |= OTG_HS_DIEPCTLx_USBAEP;     // Activate Endpoint

    if (OTG_DIEPCTL(num) & OTG_HS_DIEPCTLx_EPENA) {
      OTG_DIEPCTL(num) |= OTG_HS_DIEPCTLx_EPDIS;      // Disable endpoint
    }
    OTG_DIEPCTL(num)   |= OTG_HS_DIEPCTLx_SNAK;       // Set Endpoint NAK

    USBD_FlushInEpFifo (num | ARM_USB_ENDPOINT_DIRECTION_MASK);


    // Isochronous IN Endpoint Configuration
    if (OTG_EP_IN_TYPE(num) == ARM_USB_ENDPOINT_ISOCHRONOUS) {
      OTG->GINTMSK |= OTG_HS_GINTMSK_IISOIXFRM;       // Enable IISOIXFR

      // Regarding FrameNumber, set Frame
      if (ARM_USBD_GetFrameNumber() & 1) OTG_DIEPCTL(num) |= OTG_HS_DIEPCTLx_SEVNFRM;
      else                               OTG_DIEPCTL(num) |= OTG_HS_DIEPCTLx_SODDFRM;

      // Enable Endpoint and Clear NAK
      OTG_DIEPCTL(num) |= OTG_HS_DIEPCTLx_EPENA | OTG_HS_DIEPCTLx_CNAK;
    }

    InEndpoint[num].flags |= USBD_HS_EP_FLAG_CONFIGURED;

    // Enable IN endpoint interrupt
    OTG->DAINTMSK  |= OTG_HS_DAINTMSK_IEPM(num);


  // OUT Endpoint Configuration
  } else {
    // Set OUT Endpoint resources
    OutEndpoint[num].buffer         = NULL;
    OutEndpoint[num].dataSize       = 0;
    OutEndpoint[num].maxPacketSize  = val;

    if (OTG_EP_OUT_TYPE(num) == ARM_USB_ENDPOINT_ISOCHRONOUS) {
      OutEndpoint[num].packetCount = (ep_max_packet_size &
                                      ARM_USB_ENDPOINT_MICROFRAME_TRANSACTIONS_MASK) >> 11;
    } else OutEndpoint[num].packetCount = 1;

    // Configure OUT endpoint
    OTG_DOEPCTL(num) = (ep_type <<  OTG_HS_DOEPCTLx_EPTYP_POS)| // Endpoint Type
                        OTG_HS_DOEPCTLx_SNAK                  | // Set NAK
                        val;                                    // Max Packet Size

    // Set DATA0 PID for Interrupt or Bulk Endpoint
    if (ep_type >= ARM_USB_ENDPOINT_BULK) {
      OTG_DOEPCTL(num) |= OTG_HS_DOEPCTLx_SD0PID;
    }

    // Isochronous OUT Endpoint Configuration
    if (OTG_EP_OUT_TYPE(num) == ARM_USB_ENDPOINT_ISOCHRONOUS) {
      OTG->GINTMSK |= OTG_HS_GINTMSK_EOPFM;           // Enable End of Periodic Frame Interrupt
    }

    OTG_DOEPCTL(num) |= OTG_HS_DOEPCTLx_USBAEP;       // Activate Endpoint

    OutEndpoint[num].flags |= USBD_HS_EP_FLAG_CONFIGURED;

    // Enable OUT endpoint interrupt
    OTG->DAINTMSK     |= OTG_HS_DAINTMSK_OEPM(num);
  }
  return ARM_DRIVER_OK;
}

/**
  \fn          int32_t ARM_USBD_EndpointUnconfigure (uint8_t ep_addr)
  \brief       Unconfigure USB Endpoint.
  \param[in]   ep_addr  Endpoint Address
                - ep_addr.0..3: Address
                - ep_addr.7:    Direction
  \return      \ref execution_status
*/
static int32_t ARM_USBD_EndpointUnconfigure (uint8_t ep_addr) {
  uint32_t num, IsoEpEnCnt, tick;

  if (!(otg_hs_state & OTG_HS_USBD_DRIVER_CONNECTED)) return ARM_DRIVER_ERROR;
  if (ep_addr & ARM_USB_ENDPOINT_DIRECTION_MASK) {
    if (!(InEndpoint[ep_addr & ARM_USB_ENDPOINT_NUMBER_MASK].flags  & USBD_HS_EP_FLAG_CONFIGURED)) return ARM_DRIVER_OK;
    if (  InEndpoint[ep_addr & ARM_USB_ENDPOINT_NUMBER_MASK].flags  & USBD_HS_EP_FLAG_BUSY       ) return ARM_DRIVER_ERROR_BUSY;
  } else {
    if (!(OutEndpoint[ep_addr].flags & USBD_HS_EP_FLAG_CONFIGURED)) return ARM_DRIVER_OK;
    if (  OutEndpoint[ep_addr].flags & USBD_HS_EP_FLAG_BUSY       ) return ARM_DRIVER_ERROR_BUSY;
  }

  IsoEpEnCnt = 0;

  // Unconfigure IN Endpoint
  if (ep_addr & ARM_USB_ENDPOINT_DIRECTION_MASK) {
    ep_addr &= ARM_USB_ENDPOINT_NUMBER_MASK;

    OTG->DAINTMSK &= ~OTG_HS_DAINTMSK_IEPM(ep_addr);      // Disable IN EP interrupt
    // Count Active Isochronous IN Endpoints
    if (OTG_EP_IN_TYPE(ep_addr) == ARM_USB_ENDPOINT_ISOCHRONOUS) {
      for (num = 1; num <= USBD_EP_NUM; num++) {
        if (OTG_DIEPCTL(num) & OTG_HS_DIEPCTLx_USBAEP) {
          if (OTG_EP_IN_TYPE(num) == ARM_USB_ENDPOINT_ISOCHRONOUS) {
            IsoEpEnCnt++;
          }
        }
      }
      // If Last Active Isochronous IN Endpoint, Disable IISOIXFR
      if (IsoEpEnCnt == 1) OTG->GINTMSK &= ~OTG_HS_GINTMSK_IISOIXFRM;
    }

    if (OTG_DIEPCTL(ep_addr) & OTG_HS_DIEPCTLx_EPENA)
      OTG_DIEPCTL(ep_addr)  |=  OTG_HS_DIEPCTLx_EPDIS;    // Disable Endpoint
    OTG_DIEPCTL(ep_addr)    |=  OTG_HS_DIEPCTLx_SNAK;     // Set Endpoint NAK

    if (ep_addr)
      OTG_DIEPCTL(ep_addr)  &= ~OTG_HS_DIEPCTLx_USBAEP;   // Deactivate Endpoint

    // Reset IN endpoint resources
    InEndpoint[ep_addr].buffer        = NULL;
    InEndpoint[ep_addr].dataSize      = 0;
    InEndpoint[ep_addr].maxPacketSize = 0;
    InEndpoint[ep_addr].packetCount   = 0;
    InEndpoint[ep_addr].in_NAK        = 0;
    InEndpoint[ep_addr].flags        &=~USBD_HS_EP_FLAG_CONFIGURED;

  // Unconfigure OUT Endpoint
  } else {
    OTG->DAINTMSK &= ~OTG_HS_DAINTMSK_OEPM(ep_addr);      // Disable IN EP interrupt

    // Count Active Isochronous OUT Endpoints
    if (OTG_EP_OUT_TYPE(ep_addr) == ARM_USB_ENDPOINT_ISOCHRONOUS) {
      for (num = 1; num <= USBD_EP_NUM; num++) {
        if (OTG_DOEPCTL(num) & OTG_HS_DOEPCTLx_USBAEP) {
          if (OTG_EP_OUT_TYPE(num) == ARM_USB_ENDPOINT_ISOCHRONOUS) {
            IsoEpEnCnt++;
          }
        }
      }
      // If Last Active Isochronous OUT Endpoint, Disable EOPF
      if (IsoEpEnCnt == 1) OTG->GINTMSK &= ~OTG_HS_GINTMSK_EOPFM;
    }

    // Set Global Out Nak
    OTG->DCTL |= OTG_HS_DCTL_SGONAK;
    tick = osKernelSysTick();
    do {
      if (OTG->GINTSTS & OTG_HS_GINTSTS_GONAKEFF) break;
    } while ((osKernelSysTick() - tick) < osKernelSysTickMicroSec(10000));

    if ((OTG->GINTSTS & OTG_HS_GINTSTS_GONAKEFF) == 0) return ARM_DRIVER_ERROR;

    OTG_DOEPCTL(ep_addr) |= OTG_HS_DOEPCTLx_SNAK;         // Set Endpoint NAK

    if (ep_addr) {
      // Disable OUT endpoint
      if (OTG_DOEPCTL(ep_addr) & OTG_HS_DOEPCTLx_EPENA) { // If Endpoint is Enabled
        OTG_DOEPCTL(ep_addr)  |=  OTG_HS_DOEPCTLx_EPDIS;  // Disable Endpoint

        tick = osKernelSysTick();
        do {
          if (OTG_DOEPINT(ep_addr) & OTG_HS_DOEPINTx_EPDISD) break;
        } while ((osKernelSysTick() - tick) < osKernelSysTickMicroSec(10000));

        if ((OTG_DOEPINT(ep_addr) & OTG_HS_DOEPINTx_EPDISD) == 0) return ARM_DRIVER_ERROR;
      }

      OTG_DOEPCTL(ep_addr) &= ~OTG_HS_DOEPCTLx_USBAEP;    // Deactivate Endpoint
    }

    OTG_DOEPCTL(ep_addr) &= ~OTG_HS_DOEPCTLx_USBAEP;      // Deactivate Endpoint

    OTG->DCTL |= OTG_HS_DCTL_CGONAK;                      // Clear Global OUT NAK

    // Reset OUT endpoint resources
    OutEndpoint[ep_addr].buffer        = NULL;
    OutEndpoint[ep_addr].dataSize      = 0;
    OutEndpoint[ep_addr].maxPacketSize = 0;
    OutEndpoint[ep_addr].packetCount   = 0;
    OutEndpoint[ep_addr].flags        &=~USBD_HS_EP_FLAG_CONFIGURED;
  }
  return ARM_DRIVER_OK;
}

/**
  \fn          int32_t ARM_USBD_EndpointStall (uint8_t ep_addr, bool stall)
  \brief       Set/Clear Stall for USB Endpoint.
  \param[in]   ep_addr  Endpoint Address
                - ep_addr.0..3: Address
                - ep_addr.7:    Direction
  \param[in]   stall  Operation
                - \b false Clear
                - \b true Set
  \return      \ref execution_status
*/
static int32_t ARM_USBD_EndpointStall (uint8_t ep_addr, bool stall) {
  uint32_t tick;

  if (!(otg_hs_state & OTG_HS_USBD_DRIVER_CONNECTED)) return ARM_DRIVER_ERROR;
  if (ep_addr & ARM_USB_ENDPOINT_DIRECTION_MASK) {
    if (!(InEndpoint[ep_addr & ARM_USB_ENDPOINT_NUMBER_MASK].flags  & USBD_HS_EP_FLAG_CONFIGURED)) return ARM_DRIVER_ERROR;
    if (  InEndpoint[ep_addr & ARM_USB_ENDPOINT_NUMBER_MASK].flags  & USBD_HS_EP_FLAG_BUSY       ) return ARM_DRIVER_ERROR_BUSY;
  } else {
    if (!(OutEndpoint[ep_addr].flags & USBD_HS_EP_FLAG_CONFIGURED)) return ARM_DRIVER_ERROR;
    if (  OutEndpoint[ep_addr].flags & USBD_HS_EP_FLAG_BUSY       ) return ARM_DRIVER_ERROR_BUSY;
  }

  if (stall) {
    // IN Endpoint stall
    if (ep_addr & ARM_USB_ENDPOINT_DIRECTION_MASK) {
      ep_addr &= ARM_USB_ENDPOINT_NUMBER_MASK;

      InEndpoint[ep_addr].buffer      = NULL;
      InEndpoint[ep_addr].dataSize    = 0;
      InEndpoint[ep_addr].bufferIndex = 0;

      if (OTG_DIEPCTL(ep_addr) & OTG_HS_DIEPCTLx_EPENA)
        OTG_DIEPCTL(ep_addr)  |= OTG_HS_DIEPCTLx_STALL | OTG_HS_DIEPCTLx_EPDIS;
      else
        OTG_DIEPCTL(ep_addr)  |= OTG_HS_DIEPCTLx_STALL;

      USBD_FlushInEpFifo (ep_addr | ARM_USB_ENDPOINT_DIRECTION_MASK);

    // OUT Endpoint Stall
    } else {
      OutEndpoint[ep_addr].buffer      = NULL;
      OutEndpoint[ep_addr].dataSize    = 0;
      OutEndpoint[ep_addr].bufferIndex = 0;

      OTG->DCTL |= OTG_HS_DCTL_SGONAK;                // Set Global OUT NAK
      tick = osKernelSysTick();
      do {
        if (OTG->GINTSTS & OTG_HS_GINTSTS_GONAKEFF) break;
      } while ((osKernelSysTick() - tick) < osKernelSysTickMicroSec(10000));

      if ((OTG->GINTSTS & OTG_HS_GINTSTS_GONAKEFF) == 0) return ARM_DRIVER_ERROR;

      // Stall Out endpoint
      if (OTG_DOEPCTL(ep_addr) & OTG_HS_DOEPCTLx_EPENA)
        OTG_DOEPCTL(ep_addr)  |= OTG_HS_DOEPCTLx_STALL | OTG_HS_DOEPCTLx_EPDIS;
      else
        OTG_DOEPCTL(ep_addr)  |= OTG_HS_DOEPCTLx_STALL;

      OTG->DCTL |= OTG_HS_DCTL_CGONAK;                // Clear global NAK
    }
  } else {
    // Clear IN Endpoint stall
    if (ep_addr & 0x80) {
      ep_addr &= ~0x80;

      if (OTG_DIEPCTL(ep_addr) &  OTG_HS_DIEPCTLx_EPENA)  // If Endpoint enabled
        OTG_DIEPCTL(ep_addr)   |= OTG_HS_DIEPCTLx_EPDIS;  // Disable Endpoint

      USBD_FlushInEpFifo (ep_addr | ARM_USB_ENDPOINT_DIRECTION_MASK);

      // Set DATA0 pid for interrupt and bulk endpoint
      if (((OTG_DIEPCTL(ep_addr) & OTG_HS_DIEPCTLx_EPTYP_MSK) >> OTG_HS_DIEPCTLx_EPTYP_POS) > 1)
        OTG_DIEPCTL(ep_addr) |= OTG_HS_DIEPCTLx_SD0PID;

      OTG_DIEPCTL(ep_addr) &= ~OTG_HS_DIEPCTLx_STALL;     // Clear Stall

    // Clear OUT Endpoint stall
    } else {
      // Set DATA0 pid for interrupt and bulk endpoint
      if (((OTG_DOEPCTL(ep_addr) & OTG_HS_DOEPCTLx_EPTYP_MSK) >> OTG_HS_DOEPCTLx_EPTYP_POS) > 1) {
        OTG_DOEPCTL(ep_addr) |= OTG_HS_DOEPCTLx_SD0PID;
      }
      OTG_DOEPCTL(ep_addr) &= ~OTG_HS_DOEPCTLx_STALL;     // Clear stall
    }
  }
  return ARM_DRIVER_OK;
}

/**
  \fn          int32_t ARM_USBD_EndpointTransfer (uint8_t ep_addr, uint8_t *data, uint32_t num)
  \brief       Read data from or Write data to USB Endpoint.
  \param[in]   ep_addr  Endpoint Address
                - ep_addr.0..3: Address
                - ep_addr.7:    Direction
  \param[out]  data Pointer to buffer for data to read or with data to write
  \param[in]   num  Number of data bytes to transfer
  \return      \ref execution_status
*/
static int32_t ARM_USBD_EndpointTransfer (uint8_t ep_addr, uint8_t *data, uint32_t num) {
  uint8_t ep_num;

  if (!(otg_hs_state & OTG_HS_USBD_DRIVER_CONNECTED)) return ARM_DRIVER_ERROR;
  if (ep_addr & ARM_USB_ENDPOINT_DIRECTION_MASK) {
    if (!(InEndpoint[ep_addr & ARM_USB_ENDPOINT_NUMBER_MASK].flags  & USBD_HS_EP_FLAG_CONFIGURED)) return ARM_DRIVER_ERROR;
    if (  InEndpoint[ep_addr & ARM_USB_ENDPOINT_NUMBER_MASK].flags  & USBD_HS_EP_FLAG_BUSY       ) return ARM_DRIVER_ERROR_BUSY;
  } else {
    if (!(OutEndpoint[ep_addr].flags & USBD_HS_EP_FLAG_CONFIGURED)) return ARM_DRIVER_ERROR;
    if (  OutEndpoint[ep_addr].flags & USBD_HS_EP_FLAG_BUSY       ) return ARM_DRIVER_ERROR_BUSY;
  }

  if (ep_addr & ARM_USB_ENDPOINT_DIRECTION_MASK) {
    ep_num = ep_addr & ARM_USB_ENDPOINT_NUMBER_MASK;

    // If endpoint is disabled, return 0
    if ((OTG_DIEPCTL(ep_num) & OTG_HS_DIEPCTLx_USBAEP) == 0) return (0);

    InEndpoint[ep_num].flags      |= USBD_HS_EP_FLAG_BUSY;
    InEndpoint[ep_num].bufferIndex = 0;
    InEndpoint[ep_num].buffer      = (uint8_t *)data;
    InEndpoint[ep_num].dataSize    = num;

    if (OTG_EP_IN_TYPE(ep_num) == ARM_USB_ENDPOINT_INTERRUPT) {
      InEndpoint[ep_num].in_NAK    = 1;
      OTG_DIEPCTL(ep_num) |=  OTG_HS_DIEPCTLx_SNAK;     // Set NAK
      OTG->DIEPMSK        |=  OTG_HS_DIEPMSK_INEPNEM;   // Enable NAK effective interrupt
    } else
    {
      USBD_WriteToFifo(ep_addr);
    }

  } else {
    if (OutEndpoint[ep_addr].maxPacketSize == 0)
      return ARM_DRIVER_ERROR;

    OutEndpoint[ep_addr].flags      |= USBD_HS_EP_FLAG_BUSY;
    OutEndpoint[ep_addr].bufferIndex = 0;
    OutEndpoint[ep_addr].buffer      = data;
    OutEndpoint[ep_addr].dataSize    = num;

    USBD_EndpointReadSet(ep_addr);
  }

  return ARM_DRIVER_OK;
}

/**
  \fn          uint32_t ARM_USBD_EndpointTransferGetResult (uint8_t ep_addr)
  \brief       Get result of USB Endpoint transfer.
  \param[in]   ep_addr  Endpoint Address
                - ep_addr.0..3: Address
                - ep_addr.7:    Direction
  \return      number of successfully transfered data bytes
*/
static uint32_t ARM_USBD_EndpointTransferGetResult (uint8_t ep_addr) {
  if (ep_addr & ARM_USB_ENDPOINT_DIRECTION_MASK) {
    return InEndpoint[ep_addr & ARM_USB_ENDPOINT_NUMBER_MASK].bufferIndex;
  } else {
    return OutEndpoint[ep_addr].bufferIndex;
  }
}

/**
  \fn          int32_t ARM_USBD_EndpointTransferAbort (uint8_t ep_addr)
  \brief       Abort current USB Endpoint transfer.
  \param[in]   ep_addr  Endpoint Address
                - ep_addr.0..3: Address
                - ep_addr.7:    Direction
  \return      \ref execution_status
*/
static int32_t ARM_USBD_EndpointTransferAbort (uint8_t ep_addr) {

  if (!(otg_hs_state & OTG_HS_USBD_DRIVER_CONNECTED)) return ARM_DRIVER_ERROR;
  if (ep_addr & ARM_USB_ENDPOINT_DIRECTION_MASK) {
    if (!(InEndpoint[ep_addr & ARM_USB_ENDPOINT_NUMBER_MASK].flags  & USBD_HS_EP_FLAG_CONFIGURED)) return ARM_DRIVER_ERROR;
  } else {
    if (!(OutEndpoint[ep_addr].flags & USBD_HS_EP_FLAG_CONFIGURED)) return ARM_DRIVER_ERROR;
  }

  if (ep_addr & ARM_USB_ENDPOINT_DIRECTION_MASK) {
    ep_addr &= ARM_USB_ENDPOINT_NUMBER_MASK;

    InEndpoint[ep_addr].buffer      = NULL;
    InEndpoint[ep_addr].dataSize    = 0;
    InEndpoint[ep_addr].bufferIndex = 0;

    if (OTG_DIEPCTL(ep_addr) &  OTG_HS_DIEPCTLx_EPENA)    // If endpoint enabled
      OTG_DIEPCTL(ep_addr)   |= OTG_HS_DIEPCTLx_EPDIS;    // disable endpoint
    OTG_DIEPCTL(ep_addr)     |= OTG_HS_DIEPCTLx_SNAK;     // Set NAK

    USBD_FlushInEpFifo (ep_addr | ARM_USB_ENDPOINT_DIRECTION_MASK);

    InEndpoint[ep_addr].flags      &= ~USBD_HS_EP_FLAG_BUSY;
  } else {
    OutEndpoint[ep_addr].buffer      = NULL;
    OutEndpoint[ep_addr].dataSize    = 0;
    OutEndpoint[ep_addr].bufferIndex = 0;

    if (OTG_DOEPCTL(ep_addr) &  OTG_HS_DOEPCTLx_EPENA)    // If endpoint enabled
      OTG_DOEPCTL(ep_addr)   |= OTG_HS_DOEPCTLx_EPDIS;    // Disable endpoint
    OTG_DOEPCTL(ep_addr)     |= OTG_HS_DOEPCTLx_SNAK;     // Set NAK

    OutEndpoint[ep_addr].flags      &= ~USBD_HS_EP_FLAG_BUSY;
  }
  return ARM_DRIVER_OK;
}

/**
  \fn          uint16_t ARM_USBD_GetFrameNumber (void)
  \brief       Get current USB Frame Number.
  \return      Frame Number
*/
static uint16_t ARM_USBD_GetFrameNumber (void) {
  return ((OTG->DSTS & OTG_HS_DSTS_FNSOF_MSK) >> OTG_HS_DSTS_FNSOF_POS);
}

/**
  \fn          void USBD_HS_IRQ (uint32_t gintsts)
  \brief       USB Device Interrupt Routine (IRQ).
*/
void USBD_HS_IRQ (uint32_t gintsts) {
  uint32_t val, num, msk, sz, ep_int, i;
  static uint32_t IsoInIncomplete = 0;

// Reset interrupt
  if (gintsts & OTG_HS_GINTSTS_USBRST) {
    USBD_Reset();
    cbDeviceEvent(ARM_USBD_EVENT_RESET);
    OTG->GINTSTS = OTG_HS_GINTSTS_USBRST;
  }

// Suspend interrupt
  if (gintsts & OTG_HS_GINTSTS_USBSUSP) {
    UsbdState.active = 0;
    OTG->PCGCCTL    |= 1;
    cbDeviceEvent(ARM_USBD_EVENT_SUSPEND);
    OTG->GINTSTS = OTG_HS_GINTSTS_USBSUSP;
  }

// Resume interrupt
  if (gintsts & OTG_HS_GINTSTS_WKUPINT) {
    UsbdState.active = 1;
    OTG->PCGCCTL    &= ~1;
    cbDeviceEvent(ARM_USBD_EVENT_RESUME);
    OTG->GINTSTS = OTG_HS_GINTSTS_WKUPINT;
  }

// Speed enumeration completed
  if (gintsts & OTG_HS_GINTSTS_ENUMDNE) {
    switch ((OTG->DSTS & OTG_HS_DSTS_ENUMSPD_MSK) >> OTG_HS_DSTS_ENUMSPD_POS) {
      case 0:
        UsbdState.speed     = ARM_USB_SPEED_HIGH;
        UsbdState.active    = 1;
        cbDeviceEvent(ARM_USBD_EVENT_HIGH_SPEED);
        break;
      case 3:
        UsbdState.speed     = ARM_USB_SPEED_FULL;
        UsbdState.active    = 1;
        break;
    }

    OTG->DCTL    |= OTG_HS_DCTL_CGINAK;     // Clear global IN NAK
    OTG->DCTL    |= OTG_HS_DCTL_CGONAK ;    // clear global OUT NAK
    OTG->GINTSTS  = OTG_HS_GINTSTS_ENUMDNE;
  }

  if (gintsts & OTG_HS_GINTSTS_RXFLVL) {
    OTG->GINTMSK &= ~OTG_HS_GINTMSK_RXFLVLM;

    val =  OTG->GRXSTSP;
    num =  val & 0x0F;
    sz  = (val >> 4) & 0x7FF;

    switch ((val >> 17) & 0x0F) {
      // Setup packet
      case 6:
        // Read setup packet
        *(__packed uint32_t *)(setup_buf)     = OTG_RX_FIFO;
        *(__packed uint32_t *)(setup_buf + 4) = OTG_RX_FIFO;

        // Analyze Setup packet for SetAddress
        if (setup_buf[0] == 0) {
          if (setup_buf[1] == 5)
            ARM_USBD_DeviceSetAddress(setup_buf[2]);
        }

        setup_flag = 1;
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
    msk = (((OTG->DAINT & OTG->DAINTMSK) >> 16) & 0xFFFF);
    num = 0;

    do {
      if ((msk >> num) & 1) {
        ep_int = OTG_DOEPINT(num) & OTG->DOEPMSK;
        // Endpoint disabled
        if (ep_int & OTG_HS_DOEPINTx_EPDISD) {
          if (OTG_EP_OUT_TYPE(num) == ARM_USB_ENDPOINT_ISOCHRONOUS) {
            // Set packet count and transfer size
            OTG_DOEPTSIZ(num) = (OutEndpoint[num].packetCount << OTG_HS_DOEPTSIZx_PKTCNT_POS) |
                                (OutEndpoint[num].maxPacketSize);

            // Set correct frame
            if ((ARM_USBD_GetFrameNumber() & 1)) OTG_DOEPCTL(num) |= OTG_HS_DOEPCTLx_SEVNFRM;
            else                                 OTG_DOEPCTL(num) |= OTG_HS_DOEPCTLx_SODDFRM;

            OTG_DOEPCTL(num) |= OTG_HS_DOEPCTLx_EPENA | OTG_HS_DOEPCTLx_CNAK;
          }
          OTG_DOEPINT(num) = OTG_HS_DOEPINTx_EPDISD;
        }

        // Setup phase done interrupt
        if (ep_int & OTG_HS_DOEPINTx_STUP) {
          OutEndpoint[num].dataSize = 0;
          OTG_DOEPINT(num) = OTG_HS_DOEPINTx_STUP;
          cbEndpointEvent(num, ARM_USBD_EVENT_SETUP);
        }

        // Transfer complete interrupt
        if (ep_int & OTG_HS_DOEPINTx_XFCR) {
          OTG_DOEPINT(num) = OTG_HS_DOEPINTx_XFCR;
          if (OTG_EP_OUT_TYPE(num) != ARM_USB_ENDPOINT_ISOCHRONOUS) {
            if (OutEndpoint[num].dataSize != 0)
              USBD_EndpointReadSet(num);
            else {
              OutEndpoint[num].flags &= ~USBD_HS_EP_FLAG_BUSY;
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
    msk = (OTG->DAINT & OTG->DAINTMSK & 0xFFFF);
    num = 0;

    do {
      if ((msk >> num) & 1) {
        ep_int = OTG_DIEPINT(num) & OTG->DIEPMSK;
        // Endpoint Disabled
        if (ep_int & OTG_HS_DIEPINTx_EPDISD) {
          OTG_DIEPINT(num) = OTG_HS_DIEPINTx_EPDISD;

          if (OTG_EP_IN_TYPE(num) == ARM_USB_ENDPOINT_ISOCHRONOUS) {
            if ((IsoInIncomplete & (1 << num)) != 0) {
              // Flush IN endpoint fifo and write write new data if available
              USBD_FlushInEpFifo(num | ARM_USB_ENDPOINT_DIRECTION_MASK);
              if (InEndpoint[num].dataSize) USBD_WriteToFifo(num);
              IsoInIncomplete &= ~(1 << num);
            }
          }
        }

        // IN endpoint NAK effective
        if (ep_int & OTG_HS_DIEPINTx_INEPNE) {
          if (OTG_EP_IN_TYPE(num) == ARM_USB_ENDPOINT_INTERRUPT) {
            if (InEndpoint[num].in_NAK) {
              InEndpoint[num].in_NAK = 0;

              val = 0;
              for (i=0; i < USBD_EP_NUM; i++) val |= InEndpoint[num].in_NAK;
  
              // if no more forced NAKs, disable IN NAK effective interrupt
              if (!val) OTG->DIEPMSK &= ~OTG_HS_DIEPMSK_INEPNEM;

              // If Data available, write Data
              if (InEndpoint[num].dataSize) USBD_WriteToFifo(num);
            }
          }
          OTG_DIEPINT(num) = OTG_HS_DIEPINTx_INEPNE;
        }

        // Transmit completed
        if (ep_int & OTG_HS_DIEPINTx_XFCR) {
          OTG_DIEPINT(num) = OTG_HS_DIEPINTx_XFCR;
          if (InEndpoint[num].dataSize == 0) {
            InEndpoint[num].buffer =  NULL;
            InEndpoint[num].flags &= ~USBD_HS_EP_FLAG_BUSY;
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
    for (num = 1; num <= USBD_EP_NUM; num++) {

      if (OTG_EP_OUT_TYPE(num) != ARM_USB_ENDPOINT_ISOCHRONOUS) continue;
      if ((OTG_DOEPCTL(num) & OTG_HS_DOEPCTLx_USBAEP) == 0)     continue;

      // Incomplete Isochronous out transfer
      if (OTG->GINTSTS & OTG_HS_GINTSTS_IPXFR) {
        if ((ARM_USBD_GetFrameNumber() & 1) == ((OTG_DOEPCTL(num) >> OTG_HS_DOEPCTLx_EONUM_POS) & 1)) {
          if (OTG_DOEPCTL(num) & OTG_HS_DOEPCTLx_EPENA) {
            OTG_DOEPCTL(num) |= OTG_HS_DOEPCTLx_EPDIS;
          }
        }

      // Isochronous out transfer completed
      } else {
        if (OutEndpoint[num].dataSize != 0)
          USBD_EndpointReadSet(num);
        else {
          OutEndpoint[num].flags &= ~USBD_HS_EP_FLAG_BUSY;
          cbEndpointEvent(num, ARM_USBD_EVENT_OUT);
        }
      }
    }
    OTG->GINTSTS = OTG_HS_GINTSTS_EOPF | OTG_HS_GINTSTS_IPXFR;
  }

// Incomplete isochronous IN transfer
  if (gintsts & OTG_HS_GINTSTS_IISOIXFR) {
    OTG->GINTSTS = OTG_HS_GINTSTS_IISOIXFR;
    for (num = 1; num <= USBD_EP_NUM; num++) {

      if (OTG_EP_IN_TYPE(num) != ARM_USB_ENDPOINT_ISOCHRONOUS) continue;
      if ((OTG_DIEPCTL(num)   &  OTG_HS_DIEPCTLx_USBAEP) == 0) continue;

      if (OTG_DIEPCTL(num) & OTG_HS_DIEPCTLx_EPENA) {
        if ((ARM_USBD_GetFrameNumber() & 1) == ((OTG_DIEPCTL(num) >> OTG_HS_DIEPCTLx_EONUM_POS) & 1)) {

          IsoInIncomplete |= (1 << num);
          OTG_DIEPCTL(num)    |= OTG_HS_DIEPCTLx_EPDIS | OTG_HS_DIEPCTLx_SNAK;
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
  ARM_USBD_GetVersion,
  ARM_USBD_GetCapabilities,
  ARM_USBD_Initialize,
  ARM_USBD_Uninitialize,
  ARM_USBD_PowerControl,
  ARM_USBD_DeviceConnect,
  ARM_USBD_DeviceDisconnect,
  ARM_USBD_DeviceGetState,
  ARM_USBD_DeviceRemoteWakeup,
  ARM_USBD_DeviceSetAddress,
  ARM_USBD_ReadSetupPacket,
  ARM_USBD_EndpointConfigure,
  ARM_USBD_EndpointUnconfigure,
  ARM_USBD_EndpointStall,
  ARM_USBD_EndpointTransfer,
  ARM_USBD_EndpointTransferGetResult,
  ARM_USBD_EndpointTransferAbort,
  ARM_USBD_GetFrameNumber
};
