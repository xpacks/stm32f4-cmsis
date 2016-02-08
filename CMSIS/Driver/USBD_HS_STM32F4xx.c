/* -----------------------------------------------------------------------------
 * Copyright (c) 2013-2015 ARM Ltd.
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
 * $Date:        24. November 2015
 * $Revision:    V2.13
 *
 * Driver:       Driver_USBD0
 * Configured:   via RTE_Device.h configuration file
 * Project:      USB High-Speed Device Driver for ST STM32F4xx
 * --------------------------------------------------------------------------
 * Use the following configuration settings in the middleware component
 * to connect to this driver.
 *
 *   Configuration Setting                  Value
 *   ---------------------                  -----
 *   Connect to hardware via Driver_USBD# = 1
 * --------------------------------------------------------------------------
 * Defines used for driver configuration (at compile time):
 *
 *   USBD_MAX_ENDPOINT_NUM:  defines maximum number of IN/OUT Endpoint pairs 
 *                           that driver will support with Control Endpoint 0
 *                           not included, this value impacts driver memory
 *                           requirements
 *     - default value: 5
 *     - maximum value: 5
 *   USBD_VBUS_DETECT:       defines if driver supports VBUS detection
 *     - default value: 0   (disabled as MCBSTM32F400 board can not detect
 *                           VBUS change because of combination of B340A and
 *                           USBLC6-4 prevents VBUS to go low when board
 *                           is externally powered and USB is not connected)
 * -------------------------------------------------------------------------- */

/* History:
 *  Version 2.13
 *    Updated Isochronous transfer
 *  Version 2.12
 *    Updated IN Endpoint FIFO flush procedure
 *  Version 2.11
 *    Corrected PowerControl function for:
 *      - Unconditional Power Off
 *      - Conditional Power full (driver must be initialized)
 *  Version 2.10
 *    STM32CubeMX generated code can also be used to configure the driver.
 *  Version 2.9
 *    Removed global variable otg_hs_state
 *  Version 2.8
 *    PowerControl for Power OFF and Uninitialize functions made unconditional
 *  Version 2.7
 *    Corrected transferred size during transfer
 *  Version 2.6
 *    Corrected IN ZLP sending
 *  Version 2.5
 *    VBUS sensing disabled if USBD_VBUS_DETECT is not enabled
 *  Version 2.4
 *    Reorganized, common endpoint structure for IN and OUT endpoints
 *  Version 2.3
 *    Send data on first NAK to prevent simultaneous write to FIFO from library
 *    and from IRQ routine
 *  Version 2.2
 *    Removed unnecessary FIFO __packed attribute
 *  Version 2.1
 *    Update for USB Device CMSIS Driver API v2.01
 *  Version 2.0
 *    Updated to 2.00 API
 *  Version 1.4
 *    Multiple packet read
 *  Version 1.3
 *    Based on API V1.10 (namespace prefix ARM_ added)
 *  Version 1.2
 *    Removed include of rl_usb.h header
 *  Version 1.0
 *    Initial release
 */

/*! \page stm32f4_usbd_hs CMSIS-Driver USBD_HS Setup

The CMSIS-Driver USBD_HS requires:
  - Setup of USB clk to 48MHz (if internal Full-speed Phy is used)
  - Configuration of USB_OTG_HS
 
Valid settings for various evaluation boards are listed in the table below:

Peripheral Resource | MCBSTM32F400                  | STM32F4-Discovery | 32F401C-Discovery | 32F429I-Discovery
:-------------------|:------------------------------|:------------------|:------------------|:------------------
USB_OTG_HS Mode     |External Phy:<b>Device_only</b>| n/a               | n/a               | n/a

For different boards, refer to the hardware schematics to reflect correct setup values.
 
The STM32CubeMX configuration for MCBSTM32F400 with steps for Pinout, Clock, and System Configuration are 
listed below. Enter the values that are marked \b bold.
 
Pinout tab
----------
  1. Configure USBD mode
     - Peripherals \b USB_OTG_HS: External Phy: Mode=<b>Device_Only</b>
 
Clock Configuration tab
-----------------------
  1. AHB frequency should be higher than 30 MHz
 
Configuration tab
-----------------
  1. Under Connectivity open \b USB_OTG_HS Configuration:
     - DMA Settings: not used
     - <b>GPIO Settings</b>: review settings, no changes required
          Pin Name | Signal on Pin        | GPIO mode | GPIO Pull-up/Pull..| Maximum out | User Label
          :--------|:--------------       |:----------|:-------------------|:------------|:----------
          PA5      | USB_OTG_DS_ULPI_CK   | Alternate | No pull-up and no..| High        |.
          PA3      | USB_OTG_DS_ULPI_D0   | Alternate | No pull-up and no..| High        |.
          PB0      | USB_OTG_DS_ULPI_D1   | Alternate | No pull-up and no..| High        |.
          PB1      | USB_OTG_DS_ULPI_D2   | Alternate | No pull-up and no..| High        |.
          PB10     | USB_OTG_DS_ULPI_D3   | Alternate | No pull-up and no..| High        |.
          PB11     | USB_OTG_DS_ULPI_D4   | Alternate | No pull-up and no..| High        |.
          PB12     | USB_OTG_DS_ULPI_D5   | Alternate | No pull-up and no..| High        |.
          PB13     | USB_OTG_DS_ULPI_D6   | Alternate | No pull-up and no..| High        |.
          PB5      | USB_OTG_DS_ULPI_D7   | Alternate | No pull-up and no..| High        |.
          PI11     | USB_OTG_DS_ULPI_DIR  | Alternate | No pull-up and no..| High        |.
          PH4      | USB_OTG_DS_ULPI_NXT  | Alternate | No pull-up and no..| High        |.
          PC0      | USB_OTG_DS_ULPI_STP  | Alternate | No pull-up and no..| High        |.
     - <b>NVIC Settings</b>: enable interrupts
          Interrupt Table                      | Enable | Preemption Priority | Sub Priority
          :------------------------------------|:-------|:--------------------|:--------------
          USB On The Go HS global interrupt    |\b ON   | 0                   | 0
     - Parameter Settings: not used
     - User Constants: not used
     - Click \b OK to close the USB_OTG_HS Configuration dialog
*/

/*! \cond */

#include <stdint.h>
#include <string.h>
#include "cmsis_os.h"

#include "Driver_USBD.h"

#include "OTG_HS_STM32F4xx.h"

#ifndef USBD_MAX_ENDPOINT_NUM
#define USBD_MAX_ENDPOINT_NUM           5U
#endif
#if    (USBD_MAX_ENDPOINT_NUM > 5)
#error  Too many Endpoints, maximum IN/OUT Endpoint pairs that this driver supports is 5 !!!
#endif

#ifndef USBD_VBUS_DETECT
#define USBD_VBUS_DETECT                0U
#endif

extern uint8_t otg_hs_role;

extern void OTG_HS_PinsConfigure   (uint8_t pins_mask);
extern void OTG_HS_PinsUnconfigure (uint8_t pins_mask);

#ifdef RTE_DEVICE_FRAMEWORK_CUBE_MX
#ifdef MX_USB_OTG_HS_DEVICE
extern PCD_HandleTypeDef hpcd_USB_OTG_HS;
#endif
#endif


// USBD Driver *****************************************************************

#define ARM_USBD_DRV_VERSION ARM_DRIVER_VERSION_MAJOR_MINOR(2,13)

// Driver Version
static const ARM_DRIVER_VERSION usbd_driver_version = { ARM_USBD_API_VERSION, ARM_USBD_DRV_VERSION };

// Driver Capabilities
static const ARM_USBD_CAPABILITIES usbd_driver_capabilities = {
#if (USBD_VBUS_DETECT == 1)
  1U,   // VBUS Detection
  1U,   // Event VBUS On
  1U,   // Event VBUS Off
#else
  0U,   // VBUS Detection
  0U,   // Event VBUS On
  0U    // Event VBUS Off
#endif
};

#define OTG                     OTG_HS

#define EP_NUM(ep_addr)         ((ep_addr) & ARM_USB_ENDPOINT_NUMBER_MASK)
#define EP_ID(ep_addr)          ((EP_NUM(ep_addr) * 2U) + (((ep_addr) >> 7) & 1U))

// FIFO sizes in bytes (total available memory for FIFOs is 4 kB)
#ifndef OTG_RX_FIFO_SIZE
#define OTG_RX_FIFO_SIZE        1152U
#endif
#ifndef OTG_TX0_FIFO_SIZE
#define OTG_TX0_FIFO_SIZE        384U
#endif
#ifndef OTG_TX1_FIFO_SIZE
#define OTG_TX1_FIFO_SIZE        512U
#endif
#ifndef OTG_TX2_FIFO_SIZE
#define OTG_TX2_FIFO_SIZE        512U
#endif
#ifndef OTG_TX3_FIFO_SIZE
#define OTG_TX3_FIFO_SIZE        512U
#endif
#ifndef OTG_TX4_FIFO_SIZE
#define OTG_TX4_FIFO_SIZE        512U
#endif
#ifndef OTG_TX5_FIFO_SIZE
#define OTG_TX5_FIFO_SIZE        512U
#endif

#define OTG_TX_FIFO(n)          *((volatile uint32_t *)(OTG_HS_BASE + 0x1000U + (n * 0x1000U)))
#define OTG_RX_FIFO             *((volatile uint32_t *)(OTG_HS_BASE + 0x1000U))

#define OTG_DIEPTSIZ(ep_num)    *((volatile uint32_t *)(&OTG->DIEPTSIZ0 + (ep_num * 8U)))
#define OTG_DIEPCTL(ep_num)     *((volatile uint32_t *)(&OTG->DIEPCTL0  + (ep_num * 8U)))
#define OTG_DTXFSTS(ep_num)     *((volatile uint32_t *)(&OTG->DTXFSTS0  + (ep_num * 8U)))
#define OTG_DOEPTSIZ(ep_num)    *((volatile uint32_t *)(&OTG->DOEPTSIZ0 + (ep_num * 8U)))
#define OTG_DOEPCTL(ep_num)     *((volatile uint32_t *)(&OTG->DOEPCTL0  + (ep_num * 8U)))
#define OTG_DIEPINT(ep_num)     *((volatile uint32_t *)(&OTG->DIEPINT0  + (ep_num * 8U)))
#define OTG_DOEPINT(ep_num)     *((volatile uint32_t *)(&OTG->DOEPINT0  + (ep_num * 8U)))

#define OTG_EP_IN_TYPE(ep_num)  ((OTG_DIEPCTL(ep_num) >> 18) & 3U)
#define OTG_EP_OUT_TYPE(ep_num) ((OTG_DOEPCTL(ep_num) >> 18) & 3U)

typedef struct {                        // Endpoint structure definition
  uint8_t  *data;
  uint32_t  num;
  uint32_t  num_transferred_total;
  uint16_t  num_transferring;
  uint16_t  max_packet_size;
  uint8_t   active;
  uint8_t   packet_count;
  uint8_t   in_nak;
  uint8_t   in_zlp;
  uint8_t   in_flush;
} ENDPOINT_t;

static ARM_USBD_SignalDeviceEvent_t   SignalDeviceEvent;
static ARM_USBD_SignalEndpointEvent_t SignalEndpointEvent;

static bool                hw_powered     = false;
static bool                hw_initialized = false;
static ARM_USBD_STATE      usbd_state;
static uint32_t            setup_packet[2];     // Setup packet data
static volatile uint8_t    setup_received;      // Setup packet received

// Endpoints runtime information
static volatile ENDPOINT_t ep[(USBD_MAX_ENDPOINT_NUM + 1U) * 2U];

// Function prototypes
static uint16_t USBD_GetFrameNumber (void);


// Auxiliary functions

/**
  \fn          void USBD_FlushInEpFifo (uint8_t FIFO_num)
  \brief       Flush IN Endpoint FIFO
  \param[in]   FIFO_num  IN Endpoint FIFO number
                - FIFO_num.0..3: IN Endpoint FIFO to Flush
                - FIFO_num.4:    All IN Endpoint FIFOs to Flush
*/
static void USBD_FlushInEpFifo (uint8_t FIFO_num) {

  while ((OTG->GRSTCTL & OTG_HS_GRSTCTL_TXFFLSH) != 0U);
  OTG->GRSTCTL  = (OTG->GRSTCTL & ~OTG_HS_GRSTCTL_TXFNUM_MSK) | OTG_HS_GRSTCTL_TXFNUM(FIFO_num);
  OTG->GRSTCTL |= OTG_HS_GRSTCTL_TXFFLSH;
  while ((OTG->GRSTCTL & OTG_HS_GRSTCTL_TXFFLSH) != 0U);

}

/**
  \fn          void USBD_Reset (void)
  \brief       Reset USB Endpoint settings and variables.
*/
static void USBD_Reset (void) {
  uint8_t  i;
  uint32_t epctl;

  // Reset global variables
  setup_packet[0] = 0U;
  setup_packet[1] = 0U;
  setup_received  = 0U;
  memset((void *)&usbd_state, 0, sizeof(usbd_state));
  memset((void *)ep,          0, sizeof(ep));

  // Clear Endpoint mask registers
  OTG->DOEPMSK = 0U;
  OTG->DIEPMSK = 0U;

  for (i = 1U; i <= USBD_MAX_ENDPOINT_NUM; i++) {
    // Endpoint set NAK
    epctl = OTG_HS_DOEPCTLx_SNAK;
    if ((OTG_DOEPCTL(i) & OTG_HS_DOEPCTLx_EPENA) != 0U) {
      // Disable enabled Endpoint
      epctl |= OTG_HS_DOEPCTLx_EPDIS;
    }
    OTG_DOEPCTL(i) = epctl;

    // Endpoint set NAK
    epctl = OTG_HS_DIEPCTLx_SNAK;
    if ((OTG_DIEPCTL(i) & OTG_HS_DIEPCTLx_EPENA) != 0U) {
      // Disable enabled Endpoint
      epctl |= OTG_HS_DIEPCTLx_EPDIS;
    }
    OTG_DIEPCTL(i) = epctl;

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

  // Flush all IN Endpoint FIFOs
  USBD_FlushInEpFifo (0x10U);

  // Set device address to 0
  OTG->DCFG       = (OTG->DCFG & ~OTG_HS_DCFG_DAD_MSK);
  OTG->DAINTMSK   =  OTG_HS_DAINT_IEPINT(0U) |  // Enable IN Endpoint0 interrupt
                     OTG_HS_DAINT_OEPINT(0U);   // Enable OUT Endpoint0 interrupt

  // Enable Setup phase done, OUT Endpoint disabled and OUT transfer complete interrupt
  OTG->DOEPMSK    =  OTG_HS_DOEPMSK_STUPM    |
                     OTG_HS_DOEPMSK_EPDM     |
                     OTG_HS_DOEPMSK_XFRCM    ;

  // Enable In Endpoint disable and IN transfer complete interrupt
  OTG->DIEPMSK    =  OTG_HS_DIEPMSK_EPDM     |
                     OTG_HS_DIEPMSK_XFRCM    ;

  // Configure FIFOs
  OTG->GRXFSIZ    =   OTG_RX_FIFO_SIZE  / 4U;
  OTG->TX0FSIZ    =  (OTG_RX_FIFO_SIZE  / 4U) |
                    ((OTG_TX0_FIFO_SIZE / 4U) << OTG_HS_DIEPTXFx_INEPTXFD_POS);

  OTG->DIEPTXF1   = ((OTG_RX_FIFO_SIZE  + OTG_TX0_FIFO_SIZE) / 4U) |
                    ((OTG_TX1_FIFO_SIZE / 4U) << OTG_HS_DIEPTXFx_INEPTXFD_POS);

  OTG->DIEPTXF2   = ((OTG_RX_FIFO_SIZE  + OTG_TX0_FIFO_SIZE + OTG_TX1_FIFO_SIZE) / 4U) |
                    ((OTG_TX2_FIFO_SIZE / 4U) << OTG_HS_DIEPTXFx_INEPTXFD_POS);

  OTG->DIEPTXF3   = ((OTG_RX_FIFO_SIZE  + OTG_TX0_FIFO_SIZE + OTG_TX1_FIFO_SIZE +
                      OTG_TX2_FIFO_SIZE)/ 4U) |
                    ((OTG_TX3_FIFO_SIZE / 4U) << OTG_HS_DIEPTXFx_INEPTXFD_POS);

  OTG->DIEPTXF4   = ((OTG_RX_FIFO_SIZE  + OTG_TX0_FIFO_SIZE + OTG_TX1_FIFO_SIZE +
                      OTG_TX2_FIFO_SIZE + OTG_TX3_FIFO_SIZE)/ 4U) |
                    ((OTG_TX4_FIFO_SIZE / 4U) << OTG_HS_DIEPTXFx_INEPTXFD_POS);

  OTG->DIEPTXF5   = ((OTG_RX_FIFO_SIZE  + OTG_TX0_FIFO_SIZE + OTG_TX1_FIFO_SIZE +
                      OTG_TX2_FIFO_SIZE + OTG_TX3_FIFO_SIZE + OTG_TX4_FIFO_SIZE)/ 4U) |
                    ((OTG_TX5_FIFO_SIZE / 4U) << OTG_HS_DIEPTXFx_INEPTXFD_POS);
}

/**
  \fn          void USBD_EndpointReadSet (uint8_t ep_addr)
  \brief       Set Endpoint for next read.
  \param[in]   ep_addr  Endpoint Address
                - ep_addr.0..3: Address
                - ep_addr.7:    Direction
*/
static void USBD_EndpointReadSet (uint8_t ep_addr) {
  volatile ENDPOINT_t *ptr_ep;
  uint16_t             num;
  uint8_t              ep_num;

  ptr_ep = &ep[EP_ID(ep_addr)];
  ep_num = EP_NUM(ep_addr);

  // Set packet count and transfer size
  if  (ptr_ep->num > ptr_ep->max_packet_size) { num = ptr_ep->max_packet_size; }
  else                                        { num = ptr_ep->num;             }

  if (ep_num != 0U) {
    OTG_DOEPTSIZ(ep_num) = (ptr_ep->packet_count << OTG_HS_DOEPTSIZx_PKTCNT_POS ) |
                            num                                                   ;
  } else {
    OTG_DOEPTSIZ(0U)     = (ptr_ep->packet_count << OTG_HS_DOEPTSIZx_PKTCNT_POS ) |
                           (3U                   << OTG_HS_DOEPTSIZ0_STUPCNT_POS) |
                            num                                                   ;
  }
  // Set correct frame for Isochronous Endpoint
  if (OTG_EP_OUT_TYPE(ep_num) == ARM_USB_ENDPOINT_ISOCHRONOUS) {
    if ((USBD_GetFrameNumber() & 1U) != 0U) { OTG_DOEPCTL(ep_num) |= OTG_HS_DOEPCTLx_SEVNFRM; }
    else                                    { OTG_DOEPCTL(ep_num) |= OTG_HS_DOEPCTLx_SODDFRM; }
  }

  // Clear NAK and enable Endpoint
  OTG_DOEPCTL(ep_num) |= OTG_HS_DOEPCTLx_EPENA | OTG_HS_DOEPCTLx_CNAK;
}

/**
  \fn          int32_t USBD_ReadFromFifo (uint8_t ep_addr, uint16_t num)
  \brief       Read data from USB Endpoint.
  \param[in]   ep_addr  Endpoint Address
                - ep_addr.0..3: Address
                - ep_addr.7:    Direction
  \param[in]   num      number of data bytes to read
  \return      number of data bytes read
*/
static int32_t USBD_ReadFromFifo (uint8_t ep_addr, uint16_t num) {
  volatile ENDPOINT_t *ptr_ep;
  uint32_t             i, residue, val;
  uint8_t             *ptr_dest_8;

  // [LNP]
#if defined ( __CC_ARM )
  __packed uint32_t   *ptr_dest_32;
#else
  uint32_t            *ptr_dest_32;
#endif

  volatile uint32_t   *ptr_src;
  uint8_t              ep_num;
  uint8_t              tmp_buf[4];

  ptr_ep = &ep[EP_ID(ep_addr)];
  ep_num = EP_NUM(ep_addr);

  // Check if Endpoint is activated and buffer available
  if ((OTG_DOEPCTL(ep_num) & OTG_HS_DOEPCTLx_USBAEP) == 0U) { return 0U; }
  if (ptr_ep->data == 0U)                                   { return 0U; }

  if (num > ptr_ep->num) { num = ptr_ep->num; }

  // If Isochronous Endpoint
  if (OTG_EP_OUT_TYPE(ep_num) == ARM_USB_ENDPOINT_ISOCHRONOUS) {
    val =  ptr_ep->packet_count -
           ((OTG_DOEPTSIZ(ep_num) & OTG_HS_DOEPTSIZx_PKTCNT_MSK) >> OTG_HS_DOEPTSIZx_PKTCNT_POS);

    switch ((OTG_DOEPTSIZ(ep_num) & OTG_HS_DOEPTSIZx_RXDPID_MSK) >> OTG_HS_DOEPTSIZx_RXDPID_POS) {
      case 0:                           // DATA0
        if (val != 1U) { num = 0U; }
        break;
      case 2:                           // DATA1
        if (val != 2U) { num = 0U; }
        break;
      case 1:                           // DATA2
        if (val != 3U) { num = 0U; }
        break;
      default:
        break;
    }
  }

  // Copy data from FIFO
  ptr_src     = (volatile uint32_t *)(OTG_HS_BASE + 0x1000U);
  ptr_dest_32 = (__packed uint32_t *)(ptr_ep->data + ptr_ep->num_transferred_total);
  i           = num / 4U;
  while (i != 0U) {
    *ptr_dest_32++ = *ptr_src;
    i--;
  }
  ptr_ep->num_transferred_total += num;

  // If data size is not equal n*4
  residue = num % 4U;
  if (residue != 0U) {
    ptr_dest_8 = (uint8_t *)(ptr_dest_32);
    *((__packed uint32_t *)tmp_buf) = OTG_RX_FIFO;
    for (i = 0U; i < residue; i++) {
      *ptr_dest_8++ = tmp_buf[i];
    }
  }

  if (num != ptr_ep->max_packet_size) { ptr_ep->num  = 0U;  }
  else                                { ptr_ep->num -= num; }

  return num;
}

/**
  \fn          void USBD_WriteToFifo (uint8_t ep_addr)
  \brief       Write data to Endpoint FIFO.
  \param[in]   ep_addr  Endpoint Address
                - ep_addr.0..3: Address
                - ep_addr.7:    Direction
*/
static void USBD_WriteToFifo (uint8_t ep_addr) {
  volatile ENDPOINT_t *ptr_ep;
  uint8_t              ep_num;
  uint16_t             num, i;
  volatile uint32_t   *ptr_dest;

  // [LNP]
#if defined ( __CC_ARM )
  __packed uint32_t   *ptr_src;
#else
  uint32_t            *ptr_src;
#endif

  ptr_ep = &ep[EP_ID(ep_addr)];
  ep_num = EP_NUM(ep_addr);

  if (ptr_ep->num > ptr_ep->max_packet_size) { num = ptr_ep->max_packet_size; }
  else                                       { num = ptr_ep->num;             }

  // Check if enough space in FIFO
  if ((OTG_DTXFSTS(ep_num) * 4U) < num) { return; }

  // Set transfer size and packet count
  OTG_DIEPTSIZ(ep_num) = (ptr_ep->packet_count << OTG_HS_DIEPTSIZx_PKTCNT_POS) |
                         (ptr_ep->packet_count << OTG_HS_DIEPTSIZx_MCNT_POS)   |
                          num                                                  ;

  // Set correct frame for Isochronous Endpoint
  if (OTG_EP_IN_TYPE(ep_num) == ARM_USB_ENDPOINT_ISOCHRONOUS) {
    if ((USBD_GetFrameNumber() & 1U) != 0U) { OTG_DIEPCTL(ep_num) |= OTG_HS_DIEPCTLx_SEVNFRM; }
    else                                    { OTG_DIEPCTL(ep_num) |= OTG_HS_DIEPCTLx_SODDFRM; }
  }

  // Enable Endpoint and clear NAK
  OTG_DIEPCTL(ep_num) |= OTG_HS_DIEPCTLx_EPENA | OTG_HS_DIEPCTLx_CNAK;

  ptr_src  = (__packed uint32_t *)(ptr_ep->data + ptr_ep->num_transferred_total);
  ptr_dest = (volatile uint32_t *)(OTG_HS_BASE + 0x1000U + (ep_num * 0x1000U));

  ptr_ep->num_transferring  = num;
  ptr_ep->num              -= num;
  ptr_ep->in_zlp            = 0U;

  // Copy data to FIFO
  i = (num + 3U) >> 2U;
  while (i != 0U) {
    *ptr_dest = *ptr_src++;
    i--;
  }
}


// USBD Driver functions

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

  if (hw_initialized == true) {
    return ARM_DRIVER_OK;
  }

  SignalDeviceEvent   = cb_device_event;
  SignalEndpointEvent = cb_endpoint_event;

  otg_hs_role = ARM_USB_ROLE_DEVICE;
#ifdef RTE_DEVICE_FRAMEWORK_CLASSIC
  OTG_HS_PinsConfigure (ARM_USB_PIN_DP | ARM_USB_PIN_DM);
#endif

#ifdef RTE_DEVICE_FRAMEWORK_CUBE_MX
  hpcd_USB_OTG_HS.Instance = USB_OTG_HS;
#endif

  hw_initialized = true;

  return ARM_DRIVER_OK;
}

/**
  \fn          int32_t USBD_Uninitialize (void)
  \brief       De-initialize USB Device Interface.
  \return      \ref execution_status
*/
static int32_t USBD_Uninitialize (void) {

#ifdef RTE_DEVICE_FRAMEWORK_CLASSIC
  OTG_HS_PinsUnconfigure (ARM_USB_PIN_DP | ARM_USB_PIN_DM);
#else
  hpcd_USB_OTG_HS.Instance = NULL;
#endif
  otg_hs_role = ARM_USB_ROLE_NONE;

  hw_initialized = false;

  return ARM_DRIVER_OK;
}

/**
  \fn          int32_t USBD_PowerControl (ARM_POWER_STATE state)
  \brief       Control USB Device Interface Power.
  \param[in]   state  Power state
  \return      \ref execution_status
*/
static int32_t USBD_PowerControl (ARM_POWER_STATE state) {

  switch (state) {
    case ARM_POWER_OFF:
      RCC->AHB1ENR  |=  RCC_AHB1ENR_OTGHSEN;            // OTG HS clock enable
#ifdef RTE_DEVICE_FRAMEWORK_CLASSIC
      NVIC_DisableIRQ      (OTG_HS_IRQn);               // Disable interrupt
      NVIC_ClearPendingIRQ (OTG_HS_IRQn);               // Clear pending interrupt
#endif
      hw_powered     = false;                           // Clear powered flag
      OTG->GAHBCFG  &= ~OTG_HS_GAHBCFG_GINT;            // Disable USB interrupts
      RCC->AHB1RSTR |=  RCC_AHB1RSTR_OTGHRST;           // Reset OTG HS module
                                                        // Reset variables
      setup_received =  0U;
      memset((void *)&usbd_state, 0, sizeof(usbd_state));
      memset((void *)ep,          0, sizeof(ep));

#ifdef MX_USB_OTG_HS_ULPI_D7_Pin
      // External ULPI High-speed PHY
      RCC->AHB1ENR  &= ~RCC_AHB1ENR_OTGHSULPIEN;        // OTG HS ULPI clock disable
#else
      // On-chip Full-speed PHY
      OTG->GCCFG    &= ~OTG_HS_GCCFG_PWRDWN;            // Enable PHY power down
#endif
      OTG->PCGCCTL  |=  OTG_HS_PCGCCTL_STPPCLK;         // Stop PHY clock
      OTG->DCTL     |=  OTG_HS_DCTL_SDIS;               // Soft disconnect enabled
      OTG->GCCFG     =  0U;                             // Reset core configuration

#ifdef RTE_DEVICE_FRAMEWORK_CLASSIC
      RCC->AHB1ENR  &= ~RCC_AHB1ENR_OTGHSEN;            // Disable OTG HS clock

#else
      if (hpcd_USB_OTG_HS.Instance != NULL) {
        HAL_PCD_MspDeInit(&hpcd_USB_OTG_HS);
      }
#endif
      break;

    case ARM_POWER_FULL:
      if (hw_initialized == false) {
        return ARM_DRIVER_ERROR;
      }
      if (hw_powered     == true) {
        return ARM_DRIVER_OK;
      }
#ifdef RTE_DEVICE_FRAMEWORK_CLASSIC
      RCC->AHB1ENR  |=  RCC_AHB1ENR_OTGHSEN;            // OTG HS clock enable
#else
      HAL_PCD_MspInit(&hpcd_USB_OTG_HS);
#endif
      RCC->AHB1RSTR |=  RCC_AHB1RSTR_OTGHRST;           // Reset OTG HS module
      osDelay(1U);
      RCC->AHB1RSTR &= ~RCC_AHB1RSTR_OTGHRST;           // Clear reset of OTG HS module
      osDelay(1U);

#ifdef MX_USB_OTG_HS_ULPI_D7_Pin
      // External ULPI High-speed PHY
      RCC->AHB1ENR  |=  RCC_AHB1ENR_OTGHSULPIEN;        // OTG HS ULPI clock enable
#else
      // On-chip Full-speed PHY
      OTG->PCGCCTL  &= ~OTG_HS_PCGCCTL_STPPCLK;         // Start PHY clock
      OTG->GCCFG    |=  OTG_HS_GCCFG_PWRDWN;            // Disable power down
      OTG->GUSBCFG  |=  OTG_HS_GUSBCFG_PHSEL  |         // Full-speed transceiver
                        OTG_HS_GUSBCFG_PHYLPCS;         // 48 MHz external clock
#endif

      // Wait until AHB Master state machine is in the idle condition
      while ((OTG->GRSTCTL & OTG_HS_GRSTCTL_AHBIDL) == 0U);

      OTG->GRSTCTL  |=  OTG_HS_GRSTCTL_CSRST;           // Core soft reset
      while ((OTG->GRSTCTL & OTG_HS_GRSTCTL_CSRST) != 0U);
      osDelay (1U);

      // Wait until AHB Master state machine is in the idle condition
      while ((OTG->GRSTCTL & OTG_HS_GRSTCTL_AHBIDL) == 0U);

      USBD_Reset ();                                    // Reset variables and endpoint settings

#if ((USBD_VBUS_DETECT == 1) && defined(MX_USB_OTG_HS_VBUS_Pin))
      OTG->GCCFG    |=  OTG_HS_GCCFG_VBUSBSEN;          // Enable  VBUS sensing device "B"
#else
      OTG->GCCFG    |=  OTG_HS_GCCFG_NOVBUSSENS;        // Disable VBUS sensing
#endif
      OTG->DCTL     |=  OTG_HS_DCTL_SDIS;               // Soft disconnect enabled

      // Set turnaround time
      OTG->GUSBCFG   = ((OTG->GUSBCFG & ~OTG_HS_GUSBCFG_TRDT_MSK) |
                         OTG_HS_GUSBCFG_TRDT(5U))                 ;
      if (((OTG->GUSBCFG & OTG_HS_GUSBCFG_FDMOD) == 0U) || ((OTG->GUSBCFG & OTG_HS_GUSBCFG_FHMOD) != 0U)) {
        OTG->GUSBCFG &= ~OTG_HS_GUSBCFG_FHMOD;          // Clear force host mode
        OTG->GUSBCFG |=  OTG_HS_GUSBCFG_FDMOD;          // Force device mode
        osDelay (100U);
      }

#ifdef MX_USB_OTG_HS_ULPI_D7_Pin
      // External ULPI High-speed PHY
      OTG->DCFG     &= ~OTG_HS_DCFG_DSPD_MSK;           // High speed
#else
      // On-chip Full-speed PHY
      OTG->DCFG     |=  OTG_HS_DCFG_DSPD_MSK;           // Full Speed
#endif

      OTG->GINTMSK   = (OTG_HS_GINTMSK_USBSUSPM |       // Unmask interrupts
                        OTG_HS_GINTMSK_USBRST   |
                        OTG_HS_GINTMSK_ENUMDNEM |
                        OTG_HS_GINTMSK_RXFLVLM  |
                        OTG_HS_GINTMSK_IEPINT   |
                        OTG_HS_GINTMSK_OEPINT   |
#if (USBD_VBUS_DETECT == 1)
                        OTG_HS_GINTMSK_SRQIM    |
                        OTG_HS_GINTMSK_OTGINT   |
#endif
                        OTG_HS_GINTMSK_WUIM)    ;

      OTG->GAHBCFG  |= (OTG_HS_GAHBCFG_GINT     |       // Enable interrupts
                        OTG_HS_GAHBCFG_TXFELVL) ;

      hw_powered     = true;                            // Set powered flag
#ifdef RTE_DEVICE_FRAMEWORK_CLASSIC
      NVIC_EnableIRQ   (OTG_HS_IRQn);                   // Enable interrupt
#endif
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

  if (hw_powered == false) { return ARM_DRIVER_ERROR; }

  OTG->DCTL    &= ~OTG_HS_DCTL_SDIS;    // Soft disconnect disabled
#ifndef MX_USB_OTG_HS_ULPI_D7_Pin
  OTG->GCCFG   |=  OTG_HS_GCCFG_PWRDWN; // Disable power down
#endif

  return ARM_DRIVER_OK;
}

/**
  \fn          int32_t USBD_DeviceDisconnect (void)
  \brief       Disconnect USB Device.
  \return      \ref execution_status
*/
static int32_t USBD_DeviceDisconnect (void) {

  if (hw_powered == false) { return ARM_DRIVER_ERROR; }

  OTG->DCTL  |=  OTG_HS_DCTL_SDIS;      // Soft disconnect enabled
#ifndef MX_USB_OTG_HS_ULPI_D7_Pin
  OTG->GCCFG &= ~OTG_HS_GCCFG_PWRDWN;   // Enable power down
#endif

  return ARM_DRIVER_OK;
}

/**
  \fn          ARM_USBD_STATE USBD_DeviceGetState (void)
  \brief       Get current USB Device State.
  \return      Device State \ref ARM_USBD_STATE
*/
static ARM_USBD_STATE USBD_DeviceGetState (void) {
  return usbd_state;
}

/**
  \fn          int32_t USBD_DeviceRemoteWakeup (void)
  \brief       Trigger USB Remote Wakeup.
  \return      \ref execution_status
*/
static int32_t USBD_DeviceRemoteWakeup (void) {

  if (hw_powered == false) { return ARM_DRIVER_ERROR; }

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

  if (hw_powered == false) { return ARM_DRIVER_ERROR; }

  OTG->DCFG = (OTG->DCFG & ~OTG_HS_DCFG_DAD_MSK) | OTG_HS_DCFG_DAD(dev_addr);

  return ARM_DRIVER_OK;
}

/**
  \fn          int32_t USBD_ReadSetupPacket (uint8_t *setup)
  \brief       Read setup packet received over Control Endpoint.
  \param[out]  setup  Pointer to buffer for setup packet
  \return      \ref execution_status
*/
static int32_t USBD_ReadSetupPacket (uint8_t *setup) {

  if (hw_powered == false)  { return ARM_DRIVER_ERROR; }
  if (setup_received == 0U) { return ARM_DRIVER_ERROR; }

  setup_received = 0U;
  memcpy(setup, setup_packet, 8);

  if (setup_received != 0U) {           // If new setup packet was received while this was being read
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
  volatile ENDPOINT_t *ptr_ep;
  uint8_t              ep_num;
  uint16_t             ep_mps;
  bool                 ep_dir;

  ep_num = EP_NUM(ep_addr);
  if (ep_num > USBD_MAX_ENDPOINT_NUM) { return ARM_DRIVER_ERROR; }
  if (hw_powered == false)            { return ARM_DRIVER_ERROR; }

  ptr_ep = &ep[EP_ID(ep_addr)];
  if (ptr_ep->active != 0U)           { return ARM_DRIVER_ERROR_BUSY; }

  ep_dir = (ep_addr & ARM_USB_ENDPOINT_DIRECTION_MASK) == ARM_USB_ENDPOINT_DIRECTION_MASK;
  ep_mps =  ep_max_packet_size & ARM_USB_ENDPOINT_MAX_PACKET_SIZE_MASK;

  // Clear Endpoint transfer and configuration information
  memset((void *)(ptr_ep), 0, sizeof (ENDPOINT_t));

  // Set maximum packet size to requested
  ptr_ep->max_packet_size = ep_mps;

  // IN Endpoint
  if (ep_dir != 0U) {
    ptr_ep->in_nak = 0U;                                // Clear IN Endpoint NAK flag

    if (OTG_EP_IN_TYPE(ep_num) == ARM_USB_ENDPOINT_ISOCHRONOUS) {
      ptr_ep->packet_count = (ep_max_packet_size & ARM_USB_ENDPOINT_MICROFRAME_TRANSACTIONS_MASK) >> 11U;
    } else {
      ptr_ep->packet_count = 1U;
    }

    // Configure IN Endpoint
    OTG_DIEPCTL(ep_num) = (ep_num  <<  OTG_HS_DIEPCTLx_TXFNUM_POS) |    // FIFO Number
                          (ep_type <<  OTG_HS_DIEPCTLx_EPTYP_POS ) |    // Endpoint Type
                           ep_mps                                  ;    // Max Packet Size

    // Set DATA0 PID for Interrupt or Bulk Endpoint
    if (ep_type >= ARM_USB_ENDPOINT_BULK) {
      OTG_DIEPCTL(ep_num) |= OTG_HS_DIEPCTLx_SD0PID;
    }

    OTG_DIEPCTL(ep_num)   |= OTG_HS_DIEPCTLx_USBAEP;    // Activate Endpoint

    if ((OTG_DIEPCTL(ep_num) & OTG_HS_DIEPCTLx_EPENA) != 0U) {
      OTG_DIEPCTL(ep_num) |= OTG_HS_DIEPCTLx_EPDIS;     // Disable Endpoint
    }

    // Isochronous IN Endpoint Configuration
    if (OTG_EP_IN_TYPE(ep_num) == ARM_USB_ENDPOINT_ISOCHRONOUS) {
      OTG->GINTMSK |= OTG_HS_GINTMSK_EOPFM;             // Enable End of Periodic Frame Interrupt
    }

    OTG->DAINTMSK |= (1U << ep_num);                    // Enable Endpoint interrupt

  } else {
  // OUT Endpoint
    if (OTG_EP_OUT_TYPE(ep_num) == ARM_USB_ENDPOINT_ISOCHRONOUS) {
      ptr_ep->packet_count = (ep_max_packet_size & ARM_USB_ENDPOINT_MICROFRAME_TRANSACTIONS_MASK) >> 11U;
    } else {
      ptr_ep->packet_count = 1U;
    }

    // Configure OUT Endpoint
    OTG_DOEPCTL(ep_num) = (ep_type <<  OTG_HS_DOEPCTLx_EPTYP_POS) |     // Endpoint Type
                           OTG_HS_DOEPCTLx_SNAK                   |     // Set NAK
                           ep_mps                                 ;     // Max Packet Size

    // Set DATA0 PID for Interrupt or Bulk Endpoint
    if (ep_type >= ARM_USB_ENDPOINT_BULK) {
      OTG_DOEPCTL(ep_num) |= OTG_HS_DOEPCTLx_SD0PID;
    }

    // Isochronous OUT Endpoint Configuration
    if (OTG_EP_OUT_TYPE(ep_num) == ARM_USB_ENDPOINT_ISOCHRONOUS) {
      OTG->GINTMSK |= OTG_HS_GINTMSK_EOPFM;             // Enable End of Periodic Frame Interrupt
    }

    OTG_DOEPCTL(ep_num) |= OTG_HS_DOEPCTLx_USBAEP;      // Activate Endpoint

    OTG->DAINTMSK |= (1U << (ep_num + 16));             // Enable Endpoint interrupt
  }

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
  volatile ENDPOINT_t *ptr_ep;
  uint8_t              ep_num, i, IsoEpEnCnt;
  bool                 ep_dir;

  ep_num = EP_NUM(ep_addr);
  if (ep_num > USBD_MAX_ENDPOINT_NUM) { return ARM_DRIVER_ERROR; }
  if (hw_powered == false)            { return ARM_DRIVER_ERROR; }

  ptr_ep = &ep[EP_ID(ep_addr)];
  if (ptr_ep->active != 0U)           { return ARM_DRIVER_ERROR_BUSY; }

  ep_dir = (ep_addr & ARM_USB_ENDPOINT_DIRECTION_MASK) == ARM_USB_ENDPOINT_DIRECTION_MASK;

  OTG->DAINTMSK &= ~(1U << (ep_num + ((ep_dir ^ 1U) << 4)));    // Disable Endpoint interrupt

  // Clear Endpoint transfer and configuration information
  memset((void *)(ptr_ep), 0, sizeof (ENDPOINT_t));

  IsoEpEnCnt = 0U;

  // Count Active Isochronous OUT and IN Endpoints
  if ((OTG_EP_OUT_TYPE(ep_num) == ARM_USB_ENDPOINT_ISOCHRONOUS) ||
      (OTG_EP_IN_TYPE (ep_num) == ARM_USB_ENDPOINT_ISOCHRONOUS)) {
    for (i = 1U; i <= USBD_MAX_ENDPOINT_NUM; i++) {
      if ((OTG_DOEPCTL(i) & OTG_HS_DOEPCTLx_USBAEP) != 0U) {
        if (OTG_EP_OUT_TYPE(i) == ARM_USB_ENDPOINT_ISOCHRONOUS) {
          IsoEpEnCnt++;
        }
      }
      if ((OTG_DIEPCTL(i) & OTG_HS_DIEPCTLx_USBAEP) != 0U) {
        if (OTG_EP_IN_TYPE(i) == ARM_USB_ENDPOINT_ISOCHRONOUS) {
          IsoEpEnCnt++;
        }
      }
    }

    // If Last Active Isochronous OUT or IN Endpoint, Disable EOPF
    if (IsoEpEnCnt == 1U) { OTG->GINTMSK &= ~OTG_HS_GINTMSK_EOPFM; }
  }

  // IN Endpoint
  if (ep_dir != 0U) {
    // Disable Enabled IN Endpoint
    if ((OTG_DIEPCTL(ep_num) & OTG_HS_DIEPCTLx_EPENA) != 0U) {
      OTG_DIEPCTL(ep_num)  |=  OTG_HS_DIEPCTLx_EPDIS;
    }

    OTG_DIEPCTL(ep_num)    |=  OTG_HS_DIEPCTLx_SNAK;    // Set Endpoint NAK

    // Deactivate Endpoint
    if (ep_num != 0U) { OTG_DIEPCTL(ep_num)  &= ~OTG_HS_DIEPCTLx_USBAEP; }

  } else {
  // OUT Endpoint


    OTG->DCTL |= OTG_HS_DCTL_SGONAK;                    // Set Global OUT NAK
    while ((OTG->GINTSTS & OTG_HS_GINTSTS_GONAKEFF) == 0U);

    OTG_DOEPCTL(ep_num) |= OTG_HS_DOEPCTLx_SNAK;        // Set Endpoint NAK

    if (ep_num != 0U) {
      // Disable Enabled OUT Endpoint
      if ((OTG_DOEPCTL(ep_num) & OTG_HS_DOEPCTLx_EPENA) != 0U) {
        OTG_DOEPCTL(ep_num)  |=  OTG_HS_DOEPCTLx_EPDIS;
        while ((OTG_DOEPINT(ep_num) & OTG_HS_DOEPINTx_EPDISD) == 0U);
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
  volatile ENDPOINT_t *ptr_ep;
  uint8_t              ep_num;
  bool                 ep_dir;

  ep_num = EP_NUM(ep_addr);
  if (ep_num > USBD_MAX_ENDPOINT_NUM) { return ARM_DRIVER_ERROR; }
  if (hw_powered == false)            { return ARM_DRIVER_ERROR; }

  ptr_ep = &ep[EP_ID(ep_addr)];
  if (ptr_ep->active != 0U)           { return ARM_DRIVER_ERROR_BUSY; }

  ep_dir = (ep_addr & ARM_USB_ENDPOINT_DIRECTION_MASK) == ARM_USB_ENDPOINT_DIRECTION_MASK;

  if (stall != 0U) {                                    // Activate STALL
    if (ep_dir != 0U) {                                 // IN Endpoint
      if ((OTG_DIEPCTL(ep_num) & OTG_HS_DIEPCTLx_EPENA) != 0U) {
        // Set flush flag to Flush IN FIFO in Endpoint disabled interrupt
        ptr_ep->in_flush = 1U;
        OTG_DIEPCTL(ep_num)  |= OTG_HS_DIEPCTLx_STALL | OTG_HS_DIEPCTLx_EPDIS;
      } else {
        OTG_DIEPCTL(ep_num)  |= OTG_HS_DIEPCTLx_STALL;
        USBD_FlushInEpFifo (ep_num);
      }
    } else {                                            // OUT Endpoint
      OTG->DCTL |= OTG_HS_DCTL_SGONAK;                  // Set Global OUT NAK
      while ((OTG->GINTSTS & OTG_HS_GINTSTS_GONAKEFF) == 0U);

      // Stall OUT Endpoint
      if ((OTG_DOEPCTL(ep_num) & OTG_HS_DOEPCTLx_EPENA) != 0U) {
        OTG_DOEPCTL(ep_num)  |= OTG_HS_DOEPCTLx_STALL | OTG_HS_DOEPCTLx_EPDIS;
      } else {
        OTG_DOEPCTL(ep_num)  |= OTG_HS_DOEPCTLx_STALL;
      }

      OTG->DCTL |= OTG_HS_DCTL_CGONAK;                  // Clear global NAK
    }
  } else {                                              // Clear STALL
    ptr_ep->in_nak = 0U;
    ptr_ep->in_zlp = 0U;
    if (ep_dir != 0U) {                                 // IN Endpoint
      if ((OTG_DIEPCTL(ep_num) & OTG_HS_DIEPCTLx_EPENA) != 0U) {  // If Endpoint enabled
        OTG_DIEPCTL(ep_num)   |= OTG_HS_DIEPCTLx_EPDIS; // Disable Endpoint
      }

      // Set DATA0 PID for Interrupt and Bulk Endpoint
      if (((OTG_DIEPCTL(ep_num) & OTG_HS_DIEPCTLx_EPTYP_MSK) >> OTG_HS_DIEPCTLx_EPTYP_POS) > 1U) {
        OTG_DIEPCTL(ep_num) |= OTG_HS_DIEPCTLx_SD0PID;
      }

      OTG_DIEPCTL(ep_num) &= ~OTG_HS_DIEPCTLx_STALL;    // Clear Stall
    } else {                                            // Clear OUT Endpoint stall
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
  volatile ENDPOINT_t *ptr_ep;
  uint8_t              ep_num;
  bool                 ep_dir;

  ep_num = EP_NUM(ep_addr);
  if (ep_num > USBD_MAX_ENDPOINT_NUM) { return ARM_DRIVER_ERROR; }
  if (hw_powered == false)            { return ARM_DRIVER_ERROR; }

  ptr_ep = &ep[EP_ID(ep_addr)];
  if (ptr_ep->active != 0U)           { return ARM_DRIVER_ERROR_BUSY; }

  ep_dir = (ep_addr & ARM_USB_ENDPOINT_DIRECTION_MASK) == ARM_USB_ENDPOINT_DIRECTION_MASK;

  ptr_ep->active = 1U;

  ptr_ep->data                  = data;
  ptr_ep->num                   = num;
  ptr_ep->num_transferred_total = 0U;
  ptr_ep->num_transferring      = 0U;

  if (ep_dir != 0U) {                                     // IN Endpoint
    if (OTG_EP_IN_TYPE(ep_num) != ARM_USB_ENDPOINT_ISOCHRONOUS) {
      if (num == 0U) {
        ptr_ep->in_zlp     =  1U;                         // Send IN ZLP requested
      }
      ptr_ep->in_nak       =  1U;                         // Set IN Endpoint NAK flag
      OTG_DIEPCTL(ep_num) |=  OTG_HS_DIEPCTLx_CNAK;       // Clear NAK
      OTG_DIEPCTL(ep_num) |=  OTG_HS_DIEPCTLx_SNAK;       // Set NAK
      OTG->DIEPMSK        |=  OTG_HS_DIEPMSK_INEPNEM;     // Enable NAK effective interrupt
    }
  } else {                                                // OUT Endpoint
    if (OTG_EP_OUT_TYPE(ep_num) != ARM_USB_ENDPOINT_ISOCHRONOUS) {
      USBD_EndpointReadSet(ep_addr);
    }
  }

  return ARM_DRIVER_OK;
}

/**
  \fn          uint32_t USBD_EndpointTransferGetResult (uint8_t ep_addr)
  \brief       Get result of USB Endpoint transfer.
  \param[in]   ep_addr  Endpoint Address
                - ep_addr.0..3: Address
                - ep_addr.7:    Direction
  \return      number of successfully transferred data bytes
*/
static uint32_t USBD_EndpointTransferGetResult (uint8_t ep_addr) {

  if (EP_NUM(ep_addr) > USBD_MAX_ENDPOINT_NUM) { return 0U; }

  return (ep[EP_ID(ep_addr)].num_transferred_total);
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
  volatile ENDPOINT_t *ptr_ep;
  uint8_t              ep_num;

  ep_num = EP_NUM(ep_addr);
  if (ep_num > USBD_MAX_ENDPOINT_NUM) { return ARM_DRIVER_ERROR; }
  if (hw_powered == false)            { return ARM_DRIVER_ERROR; }

  ptr_ep = &ep[EP_ID(ep_addr)];

  ptr_ep->num    = 0U;
  ptr_ep->in_nak = 0U;
  ptr_ep->in_zlp = 0U;

  if ((ep_addr & ARM_USB_ENDPOINT_DIRECTION_MASK) != 0U) {
    if ((OTG_DIEPCTL(ep_num) & OTG_HS_DIEPCTLx_EPENA) != 0U) {
      // Set flush flag to Flush IN FIFO in Endpoint disabled interrupt
      ptr_ep->in_flush = 1U;

      // Disable enabled Endpoint and set NAK
      OTG_DIEPCTL(ep_num) |= OTG_HS_DIEPCTLx_EPDIS | OTG_HS_DIEPCTLx_SNAK;
    } else {
      // Endpoint is already disabled. Set NAK
      OTG_DIEPCTL(ep_num) |= OTG_HS_DIEPCTLx_SNAK;

      // Flush IN EP FIFO
      USBD_FlushInEpFifo (ep_num);
    }

  } else {
    if ((OTG_DOEPCTL(ep_num) & OTG_HS_DOEPCTLx_EPENA) != 0U) {
      // Disable enabled Endpoint and set NAK
      OTG_DOEPCTL(ep_num) |= OTG_HS_DOEPCTLx_EPDIS | OTG_HS_DOEPCTLx_SNAK;
    } else {
      // Endpoint is already disabled. Set NAK
      OTG_DOEPCTL(ep_num) |= OTG_HS_DOEPCTLx_SNAK;
    }
  }

  ptr_ep->active = 0U;

  return ARM_DRIVER_OK;
}

/**
  \fn          uint16_t USBD_GetFrameNumber (void)
  \brief       Get current USB Frame Number.
  \return      Frame Number
*/
static uint16_t USBD_GetFrameNumber (void) {

  if (hw_powered == false) { return 0U; }

  return ((OTG->DSTS & OTG_HS_DSTS_FNSOF_MSK) >> OTG_HS_DSTS_FNSOF_POS);
}

/**
  \fn          void USBD_HS_IRQ (uint32_t gintsts)
  \brief       USB Device Interrupt Routine (IRQ).
*/
void USBD_HS_IRQ (uint32_t gintsts) {
  volatile ENDPOINT_t *ptr_ep, *ptr_ep_in;
  uint32_t             val, msk, ep_int;
  uint16_t             num;
  uint8_t              ep_num, i;
  static uint32_t      IsoInIncomplete = 0U;

  if ((gintsts & OTG_HS_GINTSTS_USBRST) != 0U) {        // Reset interrupt
    USBD_Reset();
    SignalDeviceEvent(ARM_USBD_EVENT_RESET);
    OTG->GINTSTS = OTG_HS_GINTSTS_USBRST;
  }

  if ((gintsts & OTG_HS_GINTSTS_USBSUSP) != 0U) {       // Suspend interrupt
    usbd_state.active = 0U;
    SignalDeviceEvent(ARM_USBD_EVENT_SUSPEND);
    OTG->PCGCCTL |=  OTG_HS_PCGCCTL_STPPCLK;            // Stop PHY clock
    OTG->GINTSTS  =  OTG_HS_GINTSTS_USBSUSP;
  }

  if ((gintsts & OTG_HS_GINTSTS_WKUPINT) != 0U) {       // Resume interrupt
    usbd_state.active = 1U;
    SignalDeviceEvent(ARM_USBD_EVENT_RESUME);
    OTG->PCGCCTL &= ~OTG_HS_PCGCCTL_STPPCLK;            // Start PHY clock
    OTG->GINTSTS  =  OTG_HS_GINTSTS_WKUPINT;
  }

  if ((gintsts & OTG_HS_GINTSTS_ENUMDNE) != 0U) {       // Speed enumeration completed
    switch ((OTG->DSTS & OTG_HS_DSTS_ENUMSPD_MSK) >> OTG_HS_DSTS_ENUMSPD_POS) {
      case 0:
        usbd_state.speed  = ARM_USB_SPEED_HIGH;
        usbd_state.active = 1U;
        SignalDeviceEvent(ARM_USBD_EVENT_HIGH_SPEED);
        break;
      case 3:
        usbd_state.speed  = ARM_USB_SPEED_FULL;
        usbd_state.active = 1U;
        break;
      default:
        break;
    }

    OTG->DCTL    |= OTG_HS_DCTL_CGINAK;                 // Clear global IN NAK
    OTG->DCTL    |= OTG_HS_DCTL_CGONAK;                 // Clear global OUT NAK
    OTG->GINTSTS  = OTG_HS_GINTSTS_ENUMDNE;
  }

  if ((gintsts & OTG_HS_GINTSTS_RXFLVL) != 0U) {        // Receive FIFO interrupt
    val    =  OTG->GRXSTSP;
    ep_num =  val & 0x0FU;
    num    = (val >> 4U) & 0x7FFU;
    switch ((val >> 17U) & 0x0FU) {
      case 6:                                           // Setup packet
        // Read setup packet
        setup_packet[0] = OTG_RX_FIFO;
        setup_packet[1] = OTG_RX_FIFO;

        // Analyze Setup packet for SetAddress
        if ((setup_packet[0] & 0xFFFFU) == 0x0500U) {
          USBD_DeviceSetAddress((setup_packet[0] >> 16) & 0xFFU);
        }
        setup_received = 1U;
        break;
      case 2:                                           // OUT packet
        USBD_ReadFromFifo(ep_num, num);
        break;
      case 1:                                           // Global OUT NAK
      case 3:                                           // OUT transfer completed
      case 4:                                           // SETUP transaction completed
      default:
        break;
    }
  }

  // OUT Packet
  if ((gintsts & OTG_HS_GINTSTS_OEPINT) != 0U) {
    msk    = (((OTG->DAINT & OTG->DAINTMSK) >> 16) & 0xFFFFU);
    ep_num = 0U;
    do {
      if (((msk >> ep_num) & 1U) != 0U) {
        ep_int = OTG_DOEPINT(ep_num) & OTG->DOEPMSK;
        ptr_ep = &ep[EP_ID(ep_num)];
        if ((ep_int & OTG_HS_DOEPINTx_EPDISD) != 0U) {  // If Endpoint disabled
          if (OTG_EP_OUT_TYPE(ep_num) == ARM_USB_ENDPOINT_ISOCHRONOUS) {
            // Isochronous OUT Endpoint
            // Set packet count and transfer size
            OTG_DOEPTSIZ(ep_num) = (ptr_ep->packet_count << OTG_HS_DOEPTSIZx_PKTCNT_POS) |
                                   (ptr_ep->max_packet_size);

            // Set correct frame
            if ((USBD_GetFrameNumber() & 1U) != 0U) { OTG_DOEPCTL(ep_num) |= OTG_HS_DOEPCTLx_SEVNFRM; }
            else                                    { OTG_DOEPCTL(ep_num) |= OTG_HS_DOEPCTLx_SODDFRM; }

            OTG_DOEPCTL(ep_num) |= OTG_HS_DOEPCTLx_EPENA | OTG_HS_DOEPCTLx_CNAK;
          }
          OTG_DOEPINT(ep_num) = OTG_HS_DOEPINTx_EPDISD;
        }

        // Setup phase done interrupt
        if ((ep_int & OTG_HS_DOEPINTx_STUP) != 0U) {
          ptr_ep->num = 0U;
          OTG_DOEPINT(ep_num) = OTG_HS_DOEPINTx_STUP;
          SignalEndpointEvent(ep_num, ARM_USBD_EVENT_SETUP);
        }

        // Transfer complete interrupt
        if ((ep_int & OTG_HS_DOEPINTx_XFCR) != 0U) {
          OTG_DOEPINT(ep_num) = OTG_HS_DOEPINTx_XFCR;
          if (ptr_ep->num != 0U) {
            if (OTG_EP_OUT_TYPE(ep_num) != ARM_USB_ENDPOINT_ISOCHRONOUS) {
              USBD_EndpointReadSet(ep_num);
            }
          } else {
            ptr_ep->active = 0U;
            SignalEndpointEvent(ep_num, ARM_USBD_EVENT_OUT);
          }
        }
      }
      ep_num++;
    } while ((msk >> ep_num) != 0U);
  }

  // IN Packet
  if ((gintsts & OTG_HS_GINTSTS_IEPINT) != 0U) {
    msk    = (OTG->DAINT & OTG->DAINTMSK & 0xFFFFU);
    ep_num = 0U;
    do {
      if (((msk >> ep_num) & 1U) != 0U) {
        ep_int = OTG_DIEPINT(ep_num) & OTG->DIEPMSK;
        ptr_ep = &ep[EP_ID(ep_num | ARM_USB_ENDPOINT_DIRECTION_MASK)];

        if ((ep_int & OTG_HS_DIEPINTx_EPDISD) != 0U) {  // If Endpoint disabled
          OTG_DIEPINT(ep_num) = OTG_HS_DIEPINTx_EPDISD;
          if (ptr_ep->in_flush == 1U) {
            // Clear flush flag
            ptr_ep->in_flush = 0U;  
            // Flush IN Endpoint FIFO
            USBD_FlushInEpFifo(ep_num);
          } else if (OTG_EP_IN_TYPE(ep_num) == ARM_USB_ENDPOINT_ISOCHRONOUS) {
            if ((IsoInIncomplete & (1U << ep_num)) != 0U) {
              // Flush IN Endpoint FIFO and write new data if available
              USBD_FlushInEpFifo(ep_num);
              if (ptr_ep->data != NULL) { 
                ptr_ep->num += ptr_ep->num_transferring;
                USBD_WriteToFifo(ep_num | ARM_USB_ENDPOINT_DIRECTION_MASK);
              }
              IsoInIncomplete &= ~(1U << ep_num);
            }
          }
        }

        // IN Endpoint NAK effective
        if ((ep_int & OTG_HS_DIEPINTx_INEPNE) != 0U) {
          if (ptr_ep->in_nak != 0U) {
            ptr_ep->in_nak = 0U;
            val = 0U;
            ptr_ep_in = &ep[0];
            for (i = 0U; i < USBD_MAX_ENDPOINT_NUM; i++) {
              val |= ptr_ep_in->in_nak;
              ptr_ep_in += 2U;
            }

            // If no more forced NAKs, disable IN NAK effective interrupt
            if (val == 0U) { OTG->DIEPMSK &= ~OTG_HS_DIEPMSK_INEPNEM; }

            // If Data available, write Data
            if ((ptr_ep->num != 0U) || (ptr_ep->in_zlp != 0U)) {
              USBD_WriteToFifo(ep_num | ARM_USB_ENDPOINT_DIRECTION_MASK);
            }
          }
          OTG_DIEPINT(ep_num) = OTG_HS_DIEPINTx_INEPNE;
        }

        // Transmit completed
        if ((ep_int & OTG_HS_DIEPINTx_XFCR) != 0U) {
          OTG_DIEPINT(ep_num) = OTG_HS_DIEPINTx_XFCR;
          ptr_ep->num_transferred_total += ptr_ep->num_transferring;
          if (ptr_ep->num == 0U) {
            ptr_ep->data   = NULL;
            ptr_ep->active = 0U;
            SignalEndpointEvent(ep_num | ARM_USB_ENDPOINT_DIRECTION_MASK, ARM_USBD_EVENT_IN);
          } else {
            if (OTG_EP_IN_TYPE(ep_num) != ARM_USB_ENDPOINT_ISOCHRONOUS) {
              USBD_WriteToFifo(ep_num | ARM_USB_ENDPOINT_DIRECTION_MASK);
            }
          }
        }
      }
      ep_num++;
    } while ((msk >> ep_num != 0U));
  }

  // End of periodic frame
  if ((gintsts & OTG_HS_GINTSTS_EOPF) != 0U) {

    // Clear interrupt flags
    OTG->GINTSTS = OTG_HS_GINTSTS_EOPF | OTG_HS_GINTSTS_IPXFR | OTG_HS_GINTSTS_IISOIXFR;

    // Check enabled isochronous OUT Endpoints
    for (ep_num = 1U; ep_num <= USBD_MAX_ENDPOINT_NUM; ep_num++) {
      if (OTG_EP_OUT_TYPE(ep_num) != ARM_USB_ENDPOINT_ISOCHRONOUS) { continue; }
      if ((OTG_DOEPCTL(ep_num) & OTG_HS_DOEPCTLx_USBAEP) == 0U)    { continue; }

      if ((gintsts & OTG_HS_GINTSTS_IPXFR) != 0U) {
        // Incomplete Isochronous OUT transfer
        if ((USBD_GetFrameNumber() & 1U) == ((OTG_DOEPCTL(ep_num) >> OTG_HS_DOEPCTLx_EONUM_POS) & 1U)) {
          if ((OTG_DOEPCTL(ep_num) & OTG_HS_DOEPCTLx_EPENA) != 0U) {
            OTG_DOEPCTL(ep_num) |= OTG_HS_DOEPCTLx_EPDIS;
          }
        }
      } else {
        // Isochronous OUT transfer completed
        if (ep[EP_ID(ep_num)].num != 0U) {
          USBD_EndpointReadSet(ep_num);
        }
      }
    }

    // Check enabled isochronous IN Endpoints
    for (ep_num = 1U; ep_num <= USBD_MAX_ENDPOINT_NUM; ep_num++) {
      if (OTG_EP_IN_TYPE(ep_num) != ARM_USB_ENDPOINT_ISOCHRONOUS)  { continue; }
      if ((OTG_DIEPCTL(ep_num)   &  OTG_HS_DIEPCTLx_USBAEP) == 0U) { continue; }

      if ((gintsts & OTG_HS_GINTSTS_IISOIXFR) != 0U) {
        if ((OTG_DIEPCTL(ep_num) & OTG_HS_DIEPCTLx_EPENA) != 0U) {
          if ((USBD_GetFrameNumber() & 1U) == ((OTG_DIEPCTL(ep_num) >> OTG_HS_DIEPCTLx_EONUM_POS) & 1U)) {
            IsoInIncomplete  |= (1U << ep_num);
            OTG_DIEPCTL(ep_num) |= OTG_HS_DIEPCTLx_EPDIS | OTG_HS_DIEPCTLx_SNAK;
          }
        }
      } else {
        if (ep[EP_ID(ep_num | ARM_USB_ENDPOINT_DIRECTION_MASK)].num != 0U) {
          USBD_WriteToFifo (ep_num | ARM_USB_ENDPOINT_DIRECTION_MASK);
        }
      }
    }
  }
#if (USBD_VBUS_DETECT == 1)
  if ((gintsts & OTG_HS_GINTSTS_SRQINT) != 0U) {
    usbd_state.vbus = true;
    SignalDeviceEvent(ARM_USBD_EVENT_VBUS_ON);
    OTG->GINTSTS = OTG_HS_GINTSTS_SRQINT;
  }

  if ((gintsts & OTG_HS_GINTSTS_OTGINT) != 0U) {
    if ((OTG->GOTGINT & OTG_HS_GOTGINT_SEDET) != 0U) {
      usbd_state.vbus = false;
      SignalDeviceEvent(ARM_USBD_EVENT_VBUS_OFF);
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

/*! \endcond */
