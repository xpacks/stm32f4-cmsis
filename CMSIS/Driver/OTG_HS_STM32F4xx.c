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
 * $Date:        11. September 2014
 * $Revision:    V2.00
 *
 * Project:      OTG High-Speed Common Driver for ST STM32F4xx
 * Configured:   via RTE_Device.h configuration file
 * -------------------------------------------------------------------------- */

/* History:
 *  Version 2.00
 *    Integrated with Cube
 *  Version 1.04
 *    Use of ST Standard peripheral library
 *  Version 1.03
 *    Based on API V1.10 (namespace prefix ARM_ added)
 *  Version 1.02
 *    Removed include of rl_usb.h header
 *  Version 1.00
 *    Initial release
 */

#include <stdint.h>

#include "stm32f4xx_hal.h"

#include "Driver_USBH.h"
#include "Driver_USBD.h"

#include "RTE_Components.h"

#include "OTG_HS_STM32F4xx.h"


extern void USBH_HS_IRQ (uint32_t gintsts);
extern void USBD_HS_IRQ (uint32_t gintsts);

static uint8_t pins_cfg_mask = 0;
       uint8_t otg_hs_role   = ARM_USB_ROLE_NONE;
       uint8_t otg_hs_state  = 0;


/* Common IRQ Routine *********************************************************/

/**
  \fn          void OTG_HS_IRQHandler (void)
  \brief       USB Interrupt Routine (IRQ).
*/
void OTG_HS_IRQHandler (void) {
  uint32_t gintsts;

  gintsts = USB_OTG_HS->GINTSTS & USB_OTG_HS->GINTMSK;

  switch (otg_hs_role) {
#ifdef RTE_Drivers_USBH1
    case ARM_USB_ROLE_HOST:
      USBH_HS_IRQ (gintsts);
      break;
#endif
#ifdef RTE_Drivers_USBD1
    case ARM_USB_ROLE_DEVICE:
      USBD_HS_IRQ (gintsts);
      break;
#endif
  }
}


/* Public Functions ***********************************************************/

/**
  \fn          void OTG_HS_PinsConfigure (uint8_t pins_mask)
  \brief       Configure single or multiple USB Pin(s).
  \param[in]   Mask of pins to be configured (possible masking values:
               USB_PIN_DP, USB_PIN_DM, USB_PIN_VBUS, USB_PIN_OC, USB_PIN_ID)
*/
void OTG_HS_PinsConfigure (uint8_t pins_mask) {
  GPIO_InitTypeDef GPIO_InitStruct;

  if (pins_mask & (ARM_USB_PIN_DP | ARM_USB_PIN_DM)) {
    /* External ULPI High-speed PHY pins */
#ifdef MX_USB_OTG_HS_ULPI_DIR_Pin
    __GPIOx_CLK_ENABLE            (MX_USB_OTG_HS_ULPI_DIR_GPIOx);
    GPIO_InitStruct.Pin         =  MX_USB_OTG_HS_ULPI_DIR_GPIO_Pin;
    GPIO_InitStruct.Mode        =  MX_USB_OTG_HS_ULPI_DIR_GPIO_Mode;
    GPIO_InitStruct.Pull        =  MX_USB_OTG_HS_ULPI_DIR_GPIO_PuPd;
    GPIO_InitStruct.Speed       =  MX_USB_OTG_HS_ULPI_DIR_GPIO_Speed;
    GPIO_InitStruct.Alternate   =  MX_USB_OTG_HS_ULPI_DIR_GPIO_AF;
    HAL_GPIO_Init                 (MX_USB_OTG_HS_ULPI_DIR_GPIOx, &GPIO_InitStruct);
#endif
#ifdef MX_USB_OTG_HS_ULPI_CK_Pin
    __GPIOx_CLK_ENABLE            (MX_USB_OTG_HS_ULPI_CK_GPIOx);
    GPIO_InitStruct.Pin         =  MX_USB_OTG_HS_ULPI_CK_GPIO_Pin;
    GPIO_InitStruct.Mode        =  MX_USB_OTG_HS_ULPI_CK_GPIO_Mode;
    GPIO_InitStruct.Pull        =  MX_USB_OTG_HS_ULPI_CK_GPIO_PuPd;
    GPIO_InitStruct.Speed       =  MX_USB_OTG_HS_ULPI_CK_GPIO_Speed;
    GPIO_InitStruct.Alternate   =  MX_USB_OTG_HS_ULPI_CK_GPIO_AF;
    HAL_GPIO_Init                 (MX_USB_OTG_HS_ULPI_CK_GPIOx, &GPIO_InitStruct);
#endif
#ifdef MX_USB_OTG_HS_ULPI_NXT_Pin
    __GPIOx_CLK_ENABLE            (MX_USB_OTG_HS_ULPI_NXT_GPIOx);
    GPIO_InitStruct.Pin         =  MX_USB_OTG_HS_ULPI_NXT_GPIO_Pin;
    GPIO_InitStruct.Mode        =  MX_USB_OTG_HS_ULPI_NXT_GPIO_Mode;
    GPIO_InitStruct.Pull        =  MX_USB_OTG_HS_ULPI_NXT_GPIO_PuPd;
    GPIO_InitStruct.Speed       =  MX_USB_OTG_HS_ULPI_NXT_GPIO_Speed;
    GPIO_InitStruct.Alternate   =  MX_USB_OTG_HS_ULPI_NXT_GPIO_AF;
    HAL_GPIO_Init                 (MX_USB_OTG_HS_ULPI_NXT_GPIOx, &GPIO_InitStruct);
#endif
#ifdef MX_USB_OTG_HS_ULPI_STP_Pin
    __GPIOx_CLK_ENABLE            (MX_USB_OTG_HS_ULPI_STP_GPIOx);
    GPIO_InitStruct.Pin         =  MX_USB_OTG_HS_ULPI_STP_GPIO_Pin;
    GPIO_InitStruct.Mode        =  MX_USB_OTG_HS_ULPI_STP_GPIO_Mode;
    GPIO_InitStruct.Pull        =  MX_USB_OTG_HS_ULPI_STP_GPIO_PuPd;
    GPIO_InitStruct.Speed       =  MX_USB_OTG_HS_ULPI_STP_GPIO_Speed;
    GPIO_InitStruct.Alternate   =  MX_USB_OTG_HS_ULPI_STP_GPIO_AF;
    HAL_GPIO_Init                 (MX_USB_OTG_HS_ULPI_STP_GPIOx, &GPIO_InitStruct);
#endif
#ifdef MX_USB_OTG_HS_ULPI_D0_Pin
    __GPIOx_CLK_ENABLE            (MX_USB_OTG_HS_ULPI_D0_GPIOx);
    GPIO_InitStruct.Pin         =  MX_USB_OTG_HS_ULPI_D0_GPIO_Pin;
    GPIO_InitStruct.Mode        =  MX_USB_OTG_HS_ULPI_D0_GPIO_Mode;
    GPIO_InitStruct.Pull        =  MX_USB_OTG_HS_ULPI_D0_GPIO_PuPd;
    GPIO_InitStruct.Speed       =  MX_USB_OTG_HS_ULPI_D0_GPIO_Speed;
    GPIO_InitStruct.Alternate   =  MX_USB_OTG_HS_ULPI_D0_GPIO_AF;
    HAL_GPIO_Init                 (MX_USB_OTG_HS_ULPI_D0_GPIOx, &GPIO_InitStruct);
#endif
#ifdef MX_USB_OTG_HS_ULPI_D1_Pin
    __GPIOx_CLK_ENABLE            (MX_USB_OTG_HS_ULPI_D1_GPIOx);
    GPIO_InitStruct.Pin         =  MX_USB_OTG_HS_ULPI_D1_GPIO_Pin;
    GPIO_InitStruct.Mode        =  MX_USB_OTG_HS_ULPI_D1_GPIO_Mode;
    GPIO_InitStruct.Pull        =  MX_USB_OTG_HS_ULPI_D1_GPIO_PuPd;
    GPIO_InitStruct.Speed       =  MX_USB_OTG_HS_ULPI_D1_GPIO_Speed;
    GPIO_InitStruct.Alternate   =  MX_USB_OTG_HS_ULPI_D1_GPIO_AF;
    HAL_GPIO_Init                 (MX_USB_OTG_HS_ULPI_D1_GPIOx, &GPIO_InitStruct);
#endif
#ifdef MX_USB_OTG_HS_ULPI_D2_Pin
    __GPIOx_CLK_ENABLE            (MX_USB_OTG_HS_ULPI_D2_GPIOx);
    GPIO_InitStruct.Pin         =  MX_USB_OTG_HS_ULPI_D2_GPIO_Pin;
    GPIO_InitStruct.Mode        =  MX_USB_OTG_HS_ULPI_D2_GPIO_Mode;
    GPIO_InitStruct.Pull        =  MX_USB_OTG_HS_ULPI_D2_GPIO_PuPd;
    GPIO_InitStruct.Speed       =  MX_USB_OTG_HS_ULPI_D2_GPIO_Speed;
    GPIO_InitStruct.Alternate   =  MX_USB_OTG_HS_ULPI_D2_GPIO_AF;
    HAL_GPIO_Init                 (MX_USB_OTG_HS_ULPI_D2_GPIOx, &GPIO_InitStruct);
#endif
#ifdef MX_USB_OTG_HS_ULPI_D3_Pin
    __GPIOx_CLK_ENABLE            (MX_USB_OTG_HS_ULPI_D3_GPIOx);
    GPIO_InitStruct.Pin         =  MX_USB_OTG_HS_ULPI_D3_GPIO_Pin;
    GPIO_InitStruct.Mode        =  MX_USB_OTG_HS_ULPI_D3_GPIO_Mode;
    GPIO_InitStruct.Pull        =  MX_USB_OTG_HS_ULPI_D3_GPIO_PuPd;
    GPIO_InitStruct.Speed       =  MX_USB_OTG_HS_ULPI_D3_GPIO_Speed;
    GPIO_InitStruct.Alternate   =  MX_USB_OTG_HS_ULPI_D3_GPIO_AF;
    HAL_GPIO_Init                 (MX_USB_OTG_HS_ULPI_D3_GPIOx, &GPIO_InitStruct);
#endif
#ifdef MX_USB_OTG_HS_ULPI_D4_Pin
    __GPIOx_CLK_ENABLE            (MX_USB_OTG_HS_ULPI_D4_GPIOx);
    GPIO_InitStruct.Pin         =  MX_USB_OTG_HS_ULPI_D4_GPIO_Pin;
    GPIO_InitStruct.Mode        =  MX_USB_OTG_HS_ULPI_D4_GPIO_Mode;
    GPIO_InitStruct.Pull        =  MX_USB_OTG_HS_ULPI_D4_GPIO_PuPd;
    GPIO_InitStruct.Speed       =  MX_USB_OTG_HS_ULPI_D4_GPIO_Speed;
    GPIO_InitStruct.Alternate   =  MX_USB_OTG_HS_ULPI_D4_GPIO_AF;
    HAL_GPIO_Init                 (MX_USB_OTG_HS_ULPI_D4_GPIOx, &GPIO_InitStruct);
#endif
#ifdef MX_USB_OTG_HS_ULPI_D5_Pin
    __GPIOx_CLK_ENABLE            (MX_USB_OTG_HS_ULPI_D5_GPIOx);
    GPIO_InitStruct.Pin         =  MX_USB_OTG_HS_ULPI_D5_GPIO_Pin;
    GPIO_InitStruct.Mode        =  MX_USB_OTG_HS_ULPI_D5_GPIO_Mode;
    GPIO_InitStruct.Pull        =  MX_USB_OTG_HS_ULPI_D5_GPIO_PuPd;
    GPIO_InitStruct.Speed       =  MX_USB_OTG_HS_ULPI_D5_GPIO_Speed;
    GPIO_InitStruct.Alternate   =  MX_USB_OTG_HS_ULPI_D5_GPIO_AF;
    HAL_GPIO_Init                 (MX_USB_OTG_HS_ULPI_D5_GPIOx, &GPIO_InitStruct);
#endif
#ifdef MX_USB_OTG_HS_ULPI_D6_Pin
    __GPIOx_CLK_ENABLE            (MX_USB_OTG_HS_ULPI_D6_GPIOx);
    GPIO_InitStruct.Pin         =  MX_USB_OTG_HS_ULPI_D6_GPIO_Pin;
    GPIO_InitStruct.Mode        =  MX_USB_OTG_HS_ULPI_D6_GPIO_Mode;
    GPIO_InitStruct.Pull        =  MX_USB_OTG_HS_ULPI_D6_GPIO_PuPd;
    GPIO_InitStruct.Speed       =  MX_USB_OTG_HS_ULPI_D6_GPIO_Speed;
    GPIO_InitStruct.Alternate   =  MX_USB_OTG_HS_ULPI_D6_GPIO_AF;
    HAL_GPIO_Init                 (MX_USB_OTG_HS_ULPI_D6_GPIOx, &GPIO_InitStruct);
#endif
#ifdef MX_USB_OTG_HS_ULPI_D7_Pin
    __GPIOx_CLK_ENABLE            (MX_USB_OTG_HS_ULPI_D7_GPIOx);
    GPIO_InitStruct.Pin         =  MX_USB_OTG_HS_ULPI_D7_GPIO_Pin;
    GPIO_InitStruct.Mode        =  MX_USB_OTG_HS_ULPI_D7_GPIO_Mode;
    GPIO_InitStruct.Pull        =  MX_USB_OTG_HS_ULPI_D7_GPIO_PuPd;
    GPIO_InitStruct.Speed       =  MX_USB_OTG_HS_ULPI_D7_GPIO_Speed;
    GPIO_InitStruct.Alternate   =  MX_USB_OTG_HS_ULPI_D7_GPIO_AF;
    HAL_GPIO_Init                 (MX_USB_OTG_HS_ULPI_D7_GPIOx, &GPIO_InitStruct);
    pins_cfg_mask |= ARM_USB_PIN_DP | ARM_USB_PIN_DM;
#endif
    /* On-chip Full-speed PHY pins */
#ifdef MX_USB_OTG_HS_DP_Pin
    if (pins_mask & ARM_USB_PIN_DP) {
      __GPIOx_CLK_ENABLE          (MX_USB_OTG_HS_DP_GPIOx);
      GPIO_InitStruct.Pin       =  MX_USB_OTG_HS_DP_GPIO_Pin;
      GPIO_InitStruct.Mode      =  MX_USB_OTG_HS_DP_GPIO_Mode;
      GPIO_InitStruct.Pull      =  MX_USB_OTG_HS_DP_GPIO_PuPd;
      GPIO_InitStruct.Speed     =  MX_USB_OTG_HS_DP_GPIO_Speed;
      GPIO_InitStruct.Alternate =  MX_USB_OTG_HS_DP_GPIO_AF;
      HAL_GPIO_Init               (MX_USB_OTG_HS_DP_GPIOx, &GPIO_InitStruct);
      pins_cfg_mask |= ARM_USB_PIN_DP;
    }
#endif
#ifdef MX_USB_OTG_HS_DM_Pin
    if (pins_mask & ARM_USB_PIN_DM) {
      __GPIOx_CLK_ENABLE          (MX_USB_OTG_HS_DM_GPIOx);
      GPIO_InitStruct.Pin       =  MX_USB_OTG_HS_DM_GPIO_Pin;
      GPIO_InitStruct.Mode      =  MX_USB_OTG_HS_DM_GPIO_Mode;
      GPIO_InitStruct.Pull      =  MX_USB_OTG_HS_DM_GPIO_PuPd;
      GPIO_InitStruct.Speed     =  MX_USB_OTG_HS_DM_GPIO_Speed;
      GPIO_InitStruct.Alternate =  MX_USB_OTG_HS_DM_GPIO_AF;
      HAL_GPIO_Init               (MX_USB_OTG_HS_DM_GPIOx, &GPIO_InitStruct);
      pins_cfg_mask |= ARM_USB_PIN_DM;
    }
#endif
  }
#ifdef MX_USB_OTG_HS_ID_Pin
  if (pins_mask & ARM_USB_PIN_ID) {
    __GPIOx_CLK_ENABLE            (MX_USB_OTG_HS_ID_GPIOx);
    GPIO_InitStruct.Pin         =  MX_USB_OTG_HS_ID_GPIO_Pin;
    GPIO_InitStruct.Mode        =  MX_USB_OTG_HS_ID_GPIO_Mode;
    GPIO_InitStruct.Pull        =  MX_USB_OTG_HS_ID_GPIO_PuPd;
    GPIO_InitStruct.Speed       =  MX_USB_OTG_HS_ID_GPIO_Speed;
    GPIO_InitStruct.Alternate   =  MX_USB_OTG_HS_ID_GPIO_AF;
    HAL_GPIO_Init                 (MX_USB_OTG_HS_ID_GPIOx, &GPIO_InitStruct);
    pins_cfg_mask |= ARM_USB_PIN_ID;
  }
#endif
#ifdef MX_USB_OTG_HS_VBUS_Pin           // Device VBUS sensing pin (input)
  if (pins_mask & ARM_USB_PIN_VBUS) {
    if (otg_hs_role == ARM_USB_ROLE_DEVICE) {
      __GPIOx_CLK_ENABLE          (MX_USB_OTG_HS_VBUS_GPIOx);
      GPIO_InitStruct.Pin       =  MX_USB_OTG_HS_VBUS_GPIO_Pin;
      GPIO_InitStruct.Mode      =  MX_USB_OTG_HS_VBUS_GPIO_Mode;
      GPIO_InitStruct.Pull      =  MX_USB_OTG_HS_VBUS_GPIO_PuPd;
      GPIO_InitStruct.Speed     =  0;
      GPIO_InitStruct.Alternate =  0;
      HAL_GPIO_Init               (MX_USB_OTG_HS_VBUS_GPIOx, &GPIO_InitStruct);
      pins_cfg_mask |= ARM_USB_PIN_VBUS;
    }
  }
#endif
#ifdef MX_USB_OTG_HS_VBUS_Power_Pin     // Host VBUS power driving pin (output)
  if (pins_mask & ARM_USB_PIN_VBUS) {
    if (otg_hs_role == ARM_USB_ROLE_HOST) {
      __GPIOx_CLK_ENABLE          (MX_USB_OTG_HS_VBUS_Power_GPIOx);
      GPIO_InitStruct.Pin       =  MX_USB_OTG_HS_VBUS_Power_GPIO_Pin;
      GPIO_InitStruct.Mode      =  MX_USB_OTG_HS_VBUS_Power_GPIO_Mode;
      GPIO_InitStruct.Pull      =  MX_USB_OTG_HS_VBUS_Power_GPIO_PuPd;
      GPIO_InitStruct.Speed     =  0;
      GPIO_InitStruct.Alternate =  0;
      HAL_GPIO_Init               (MX_USB_OTG_HS_VBUS_Power_GPIOx, &GPIO_InitStruct);
      pins_cfg_mask |= ARM_USB_PIN_VBUS;
    }
  }
#endif
#ifdef MX_USB_OTG_HS_Overrcurrent_Pin   // Host overcurrent sensing pin (input)
  if (pins_mask & ARM_USB_PIN_OC) {
    if (otg_hs_role == ARM_USB_ROLE_HOST) {
      __GPIOx_CLK_ENABLE          (MX_USB_OTG_HS_Overcurrent_GPIOx);
      GPIO_InitStruct.Pin       =  MX_USB_OTG_HS_Overcurrent_GPIO_Pin;
      GPIO_InitStruct.Mode      =  MX_USB_OTG_HS_Overcurrent_GPIO_Mode;
      GPIO_InitStruct.Pull      =  MX_USB_OTG_HS_Overcurrent_GPIO_PuPd;
      GPIO_InitStruct.Speed     =  0;
      GPIO_InitStruct.Alternate =  0;
      HAL_GPIO_Init               (MX_USB_OTG_HS_Overcurrent_GPIOx, &GPIO_InitStruct);
      pins_cfg_mask |= ARM_USB_PIN_OC;
    }
  }
#endif
}

/**
  \fn          void OTG_HS_PinsUnconfigure (uint8_t pins_mask)
  \brief       De-configure to reset settings single or multiple USB Pin(s).
  \param[in]   Mask of pins to be de-configured (possible masking values:
               USB_PIN_DP, USB_PIN_DM, USB_PIN_VBUS, USB_PIN_OC, USB_PIN_ID)
*/
void OTG_HS_PinsUnconfigure (uint8_t pins_mask) {

  if ((pins_cfg_mask & pins_mask) & (ARM_USB_PIN_DP | ARM_USB_PIN_DM)) {
    /* External ULPI High-speed PHY pins */
#ifdef MX_USB_OTG_HS_ULPI_DIR_Pin
    HAL_GPIO_DeInit (MX_USB_OTG_HS_ULPI_DIR_GPIOx, MX_USB_OTG_HS_ULPI_DIR_GPIO_Pin);
#endif
#ifdef MX_USB_OTG_HS_ULPI_CK_Pin
    HAL_GPIO_DeInit (MX_USB_OTG_HS_ULPI_CK_GPIOx,  MX_USB_OTG_HS_ULPI_CK_GPIO_Pin);
#endif
#ifdef MX_USB_OTG_HS_ULPI_NXT_Pin
    HAL_GPIO_DeInit (MX_USB_OTG_HS_ULPI_NXT_GPIOx, MX_USB_OTG_HS_ULPI_NXT_GPIO_Pin);
#endif
#ifdef MX_USB_OTG_HS_ULPI_STP_Pin
    HAL_GPIO_DeInit (MX_USB_OTG_HS_ULPI_STP_GPIOx, MX_USB_OTG_HS_ULPI_STP_GPIO_Pin);
#endif
#ifdef MX_USB_OTG_HS_ULPI_D0_Pin
    HAL_GPIO_DeInit (MX_USB_OTG_HS_ULPI_D0_GPIOx,  MX_USB_OTG_HS_ULPI_D0_GPIO_Pin);
#endif
#ifdef MX_USB_OTG_HS_ULPI_D1_Pin
    HAL_GPIO_DeInit (MX_USB_OTG_HS_ULPI_D1_GPIOx,  MX_USB_OTG_HS_ULPI_D1_GPIO_Pin);
#endif
#ifdef MX_USB_OTG_HS_ULPI_D2_Pin
    HAL_GPIO_DeInit (MX_USB_OTG_HS_ULPI_D2_GPIOx,  MX_USB_OTG_HS_ULPI_D2_GPIO_Pin);
#endif
#ifdef MX_USB_OTG_HS_ULPI_D3_Pin
    HAL_GPIO_DeInit (MX_USB_OTG_HS_ULPI_D3_GPIOx,  MX_USB_OTG_HS_ULPI_D3_GPIO_Pin);
#endif
#ifdef MX_USB_OTG_HS_ULPI_D4_Pin
    HAL_GPIO_DeInit (MX_USB_OTG_HS_ULPI_D4_GPIOx,  MX_USB_OTG_HS_ULPI_D4_GPIO_Pin);
#endif
#ifdef MX_USB_OTG_HS_ULPI_D5_Pin
    HAL_GPIO_DeInit (MX_USB_OTG_HS_ULPI_D5_GPIOx,  MX_USB_OTG_HS_ULPI_D5_GPIO_Pin);
#endif
#ifdef MX_USB_OTG_HS_ULPI_D6_Pin
    HAL_GPIO_DeInit (MX_USB_OTG_HS_ULPI_D6_GPIOx,  MX_USB_OTG_HS_ULPI_D6_GPIO_Pin);
#endif
#ifdef MX_USB_OTG_HS_ULPI_D7_Pin
    HAL_GPIO_DeInit (MX_USB_OTG_HS_ULPI_D7_GPIOx,  MX_USB_OTG_HS_ULPI_D7_GPIO_Pin);
    pins_cfg_mask &= ~(ARM_USB_PIN_DP | ARM_USB_PIN_DM);
#endif
    /* On-chip Full-speed PHY pins */
#ifdef MX_USB_OTG_HS_DP_Pin
    if ((pins_cfg_mask & pins_mask) & ARM_USB_PIN_DP) {
      HAL_GPIO_DeInit (MX_USB_OTG_HS_DP_GPIOx, MX_USB_OTG_HS_DP_GPIO_Pin);
      pins_cfg_mask &= ~ARM_USB_PIN_DP;
    }
#endif
#ifdef MX_USB_OTG_HS_DM_Pin
    if ((pins_cfg_mask & pins_mask) & ARM_USB_PIN_DM) {
      HAL_GPIO_DeInit (MX_USB_OTG_HS_DM_GPIOx, MX_USB_OTG_HS_DM_GPIO_Pin);
      pins_cfg_mask &= ~ARM_USB_PIN_DM;
    }
#endif
  }
#ifdef MX_USB_OTG_HS_ID_Pin
  if ((pins_cfg_mask & pins_mask) & ARM_USB_PIN_ID) {
    HAL_GPIO_DeInit (MX_USB_OTG_HS_ID_GPIOx, MX_USB_OTG_HS_ID_GPIO_Pin);
    pins_cfg_mask &= ~ARM_USB_PIN_ID;
  }
#endif
#ifdef MX_USB_OTG_HS_VBUS_Pin
  if ((pins_cfg_mask & pins_mask) & ARM_USB_PIN_VBUS) {
    if (otg_hs_role == ARM_USB_ROLE_DEVICE) {
      HAL_GPIO_DeInit (MX_USB_OTG_HS_VBUS_GPIOx, MX_USB_OTG_HS_VBUS_GPIO_Pin);
      pins_cfg_mask &= ~ARM_USB_PIN_VBUS;
    }
  }
#endif
#ifdef MX_USB_OTG_HS_VBUS_Power_Pin
  if ((pins_cfg_mask & pins_mask) & ARM_USB_PIN_VBUS) {
    if (otg_hs_role == ARM_USB_ROLE_HOST) {
      HAL_GPIO_DeInit (MX_USB_OTG_HS_VBUS_Power_GPIOx, MX_USB_OTG_HS_VBUS_Power_GPIO_Pin);
      pins_cfg_mask &= ~ARM_USB_PIN_VBUS;
    }
  }
#endif
#ifdef MX_USB_OTG_HS_Overrcurrent_Pin
  if ((pins_cfg_mask & pins_mask) & ARM_USB_PIN_OC) {
    if (otg_hs_role == ARM_USB_ROLE_HOST) {
      HAL_GPIO_DeInit (MX_USB_OTG_HS_Overcurrent_GPIOx, MX_USB_OTG_HS_Overcurrent_GPIO_Pin);
      pins_cfg_mask &= ~ARM_USB_PIN_OC;
    }
  }
#endif
}

/**
  \fn          void OTG_HS_PinVbusOnOff (bool state)
  \brief       Drive VBUS Pin On/Off.
  \param[in]   state    State On/Off (true = On, false = Off)
*/
void OTG_HS_PinVbusOnOff (bool state) {

#ifdef MX_USB_OTG_HS_VBUS_Power_Pin
  if (otg_hs_role == ARM_USB_ROLE_HOST) {
    HAL_GPIO_WritePin (MX_USB_OTG_HS_VBUS_Power_GPIOx, MX_USB_OTG_HS_VBUS_Power_GPIO_Pin, ((state == true) ? GPIO_PIN_RESET : GPIO_PIN_SET));
  }
#endif
}
