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
 * $Date:        9. December 2014
 * $Revision:    V2.01
 *
 * Project:      OTG Full/Low-Speed Common Driver for ST STM32F4xx
 * Configured:   via RTE_Device.h configuration file
 * -------------------------------------------------------------------------- */

/* History:
 *  Version 2.01
 *    VBUS Power pin active high/low functionality added
 *    Overcurrent state functionality (without event) added
 *  Version 2.00
 *    Integrated with STM32CubeMX
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

#include "OTG_FS_STM32F4xx.h"


extern void USBH_FS_IRQ (uint32_t gintsts);
extern void USBD_FS_IRQ (uint32_t gintsts);

static uint8_t pins_cfg_mask = 0;
       uint8_t otg_fs_role   = ARM_USB_ROLE_NONE;
       uint8_t otg_fs_state  = 0;


/* Common IRQ Routine *********************************************************/

/**
  \fn          void OTG_FS_IRQHandler (void)
  \brief       USB Interrupt Routine (IRQ).
*/
void OTG_FS_IRQHandler (void) {
  uint32_t gintsts;

  gintsts = USB_OTG_FS->GINTSTS & USB_OTG_FS->GINTMSK;

  switch (otg_fs_role) {
#ifdef RTE_Drivers_USBH0
    case ARM_USB_ROLE_HOST:
      USBH_FS_IRQ (gintsts);
      break;
#endif
#ifdef RTE_Drivers_USBD0
    case ARM_USB_ROLE_DEVICE:
      USBD_FS_IRQ (gintsts);
      break;
#endif
  }
}


/* Public Functions ***********************************************************/

/**
  \fn          void OTG_FS_PinsConfigure (uint8_t pins_mask)
  \brief       Configure single or multiple USB Pin(s).
  \param[in]   Mask of pins to be configured (possible masking values:
               ARM_USB_PIN_DP, ARM_USB_PIN_DM, ARM_USB_PIN_VBUS,
               ARM_USB_PIN_OC, ARM_USB_PIN_ID)
*/
void OTG_FS_PinsConfigure (uint8_t pins_mask) {
  GPIO_InitTypeDef GPIO_InitStruct;

  if (pins_mask & ARM_USB_PIN_DP) {
    __GPIOx_CLK_ENABLE            (MX_USB_OTG_FS_DP_GPIOx);
    GPIO_InitStruct.Pin         =  MX_USB_OTG_FS_DP_GPIO_Pin;
    GPIO_InitStruct.Mode        =  MX_USB_OTG_FS_DP_GPIO_Mode;
    GPIO_InitStruct.Pull        =  MX_USB_OTG_FS_DP_GPIO_PuPd;
    GPIO_InitStruct.Speed       =  MX_USB_OTG_FS_DP_GPIO_Speed;
    GPIO_InitStruct.Alternate   =  MX_USB_OTG_FS_DP_GPIO_AF;
    HAL_GPIO_Init                 (MX_USB_OTG_FS_DP_GPIOx, &GPIO_InitStruct);
    pins_cfg_mask |= ARM_USB_PIN_DP;
  }
  if (pins_mask & ARM_USB_PIN_DM) {
    __GPIOx_CLK_ENABLE            (MX_USB_OTG_FS_DM_GPIOx);
    GPIO_InitStruct.Pin         =  MX_USB_OTG_FS_DM_GPIO_Pin;
    GPIO_InitStruct.Mode        =  MX_USB_OTG_FS_DM_GPIO_Mode;
    GPIO_InitStruct.Pull        =  MX_USB_OTG_FS_DM_GPIO_PuPd;
    GPIO_InitStruct.Speed       =  MX_USB_OTG_FS_DM_GPIO_Speed;
    GPIO_InitStruct.Alternate   =  MX_USB_OTG_FS_DM_GPIO_AF;
    HAL_GPIO_Init               (MX_USB_OTG_FS_DM_GPIOx, &GPIO_InitStruct);
    pins_cfg_mask |= ARM_USB_PIN_DM;
  }
#ifdef MX_USB_OTG_FS_ID_Pin
  if (pins_mask & ARM_USB_PIN_ID) {
    __GPIOx_CLK_ENABLE            (MX_USB_OTG_FS_ID_GPIOx);
    GPIO_InitStruct.Pin         =  MX_USB_OTG_FS_ID_GPIO_Pin;
    GPIO_InitStruct.Mode        =  MX_USB_OTG_FS_ID_GPIO_Mode;
    GPIO_InitStruct.Pull        =  MX_USB_OTG_FS_ID_GPIO_PuPd;
    GPIO_InitStruct.Speed       =  MX_USB_OTG_FS_ID_GPIO_Speed;
    GPIO_InitStruct.Alternate   =  MX_USB_OTG_FS_ID_GPIO_AF;
    HAL_GPIO_Init                 (MX_USB_OTG_FS_ID_GPIOx, &GPIO_InitStruct);
    pins_cfg_mask |= ARM_USB_PIN_ID;
  }
#endif
#ifdef MX_USB_OTG_FS_VBUS_Pin           // Device VBUS sensing pin (input)
  if (pins_mask & ARM_USB_PIN_VBUS) {
    if (otg_fs_role == ARM_USB_ROLE_DEVICE) {
      __GPIOx_CLK_ENABLE          (MX_USB_OTG_FS_VBUS_GPIOx);
      GPIO_InitStruct.Pin       =  MX_USB_OTG_FS_VBUS_GPIO_Pin;
      GPIO_InitStruct.Mode      =  MX_USB_OTG_FS_VBUS_GPIO_Mode;
      GPIO_InitStruct.Pull      =  MX_USB_OTG_FS_VBUS_GPIO_PuPd;
      GPIO_InitStruct.Speed     =  0;
      GPIO_InitStruct.Alternate =  0;
      HAL_GPIO_Init               (MX_USB_OTG_FS_VBUS_GPIOx, &GPIO_InitStruct);
      pins_cfg_mask |= ARM_USB_PIN_VBUS;
    }
  }
#endif
#ifdef MX_USB_OTG_FS_VBUS_Power_Pin     // Host VBUS power driving pin (output)
  if (pins_mask & ARM_USB_PIN_VBUS) {
    if (otg_fs_role == ARM_USB_ROLE_HOST) {
      __GPIOx_CLK_ENABLE          (MX_USB_OTG_FS_VBUS_Power_GPIOx);

      // Initial Host VBUS Power Off
#if  (USB_OTG_FS_VBUS_Power_Pin_Active == 0)
      HAL_GPIO_WritePin (MX_USB_OTG_FS_VBUS_Power_GPIOx, MX_USB_OTG_FS_VBUS_Power_GPIO_Pin, GPIO_PIN_SET);
#else
      HAL_GPIO_WritePin (MX_USB_OTG_FS_VBUS_Power_GPIOx, MX_USB_OTG_FS_VBUS_Power_GPIO_Pin, GPIO_PIN_RESET );
#endif

      GPIO_InitStruct.Pin       =  MX_USB_OTG_FS_VBUS_Power_GPIO_Pin;
      GPIO_InitStruct.Mode      =  MX_USB_OTG_FS_VBUS_Power_GPIO_Mode;
      GPIO_InitStruct.Pull      =  MX_USB_OTG_FS_VBUS_Power_GPIO_PuPd;
      GPIO_InitStruct.Speed     =  0;
      GPIO_InitStruct.Alternate =  0;
      HAL_GPIO_Init               (MX_USB_OTG_FS_VBUS_Power_GPIOx, &GPIO_InitStruct);
      pins_cfg_mask |= ARM_USB_PIN_VBUS;
    }
  }
#endif
#ifdef MX_USB_OTG_FS_Overrcurrent_Pin   // Host overcurrent sensing pin (input)
  if (pins_mask & ARM_USB_PIN_OC) {
    if (otg_fs_role == ARM_USB_ROLE_HOST) {
      __GPIOx_CLK_ENABLE          (MX_USB_OTG_FS_Overcurrent_GPIOx);
      GPIO_InitStruct.Pin       =  MX_USB_OTG_FS_Overcurrent_GPIO_Pin;
      GPIO_InitStruct.Mode      =  MX_USB_OTG_FS_Overcurrent_GPIO_Mode;
      GPIO_InitStruct.Pull      =  MX_USB_OTG_FS_Overcurrent_GPIO_PuPd;
      GPIO_InitStruct.Speed     =  0;
      GPIO_InitStruct.Alternate =  0;
      HAL_GPIO_Init               (MX_USB_OTG_FS_Overcurrent_GPIOx, &GPIO_InitStruct);
      pins_cfg_mask |= ARM_USB_PIN_OC;
    }
  }
#endif
}

/**
  \fn          void OTG_FS_PinsUnconfigure (uint8_t pins_mask)
  \brief       De-configure to reset settings single or multiple USB Pin(s).
  \param[in]   Mask of pins to be de-configured (possible masking values:
               ARM_USB_PIN_DP, ARM_USB_PIN_DM, ARM_USB_PIN_VBUS,
               ARM_USB_PIN_OC, ARM_USB_PIN_ID)
*/
void OTG_FS_PinsUnconfigure (uint8_t pins_mask) {

  if ((pins_cfg_mask & pins_mask) & ARM_USB_PIN_DP) {
    HAL_GPIO_DeInit (MX_USB_OTG_FS_DP_GPIOx, MX_USB_OTG_FS_DP_GPIO_Pin);
    pins_cfg_mask &= ~ARM_USB_PIN_DP;
  }
  if ((pins_cfg_mask & pins_mask) & ARM_USB_PIN_DM) {
    HAL_GPIO_DeInit (MX_USB_OTG_FS_DM_GPIOx, MX_USB_OTG_FS_DM_GPIO_Pin);
    pins_cfg_mask &= ~ARM_USB_PIN_DM;
  }
#ifdef MX_USB_OTG_FS_ID_Pin
  if ((pins_cfg_mask & pins_mask) & ARM_USB_PIN_ID) {
    HAL_GPIO_DeInit (MX_USB_OTG_FS_ID_GPIOx, MX_USB_OTG_FS_ID_GPIO_Pin);
    pins_cfg_mask &= ~ARM_USB_PIN_ID;
  }
#endif
#ifdef MX_USB_OTG_FS_VBUS_Pin
  if ((pins_cfg_mask & pins_mask) & ARM_USB_PIN_VBUS) {
    if (otg_fs_role == ARM_USB_ROLE_DEVICE) {
      HAL_GPIO_DeInit (MX_USB_OTG_FS_VBUS_GPIOx, MX_USB_OTG_FS_VBUS_GPIO_Pin);
      pins_cfg_mask &= ~ARM_USB_PIN_VBUS;
    }
  }
#endif
#ifdef MX_USB_OTG_FS_VBUS_Power_Pin
  if ((pins_cfg_mask & pins_mask) & ARM_USB_PIN_VBUS) {
    if (otg_fs_role == ARM_USB_ROLE_HOST) {
      HAL_GPIO_DeInit (MX_USB_OTG_FS_VBUS_Power_GPIOx, MX_USB_OTG_FS_VBUS_Power_GPIO_Pin);
      pins_cfg_mask &= ~ARM_USB_PIN_VBUS;
    }
  }
#endif
#ifdef MX_USB_OTG_FS_Overrcurrent_Pin
  if ((pins_cfg_mask & pins_mask) & ARM_USB_PIN_OC) {
    if (otg_fs_role == ARM_USB_ROLE_HOST) {
      HAL_GPIO_DeInit (MX_USB_OTG_FS_Overcurrent_GPIOx, MX_USB_OTG_FS_Overcurrent_GPIO_Pin);
      pins_cfg_mask &= ~ARM_USB_PIN_OC;
    }
  }
#endif
}

/**
  \fn          void OTG_FS_PinVbusOnOff (bool state)
  \brief       Drive VBUS Pin On/Off.
  \param[in]   state    State On/Off (true = On, false = Off)
*/
void OTG_FS_PinVbusOnOff (bool state) {

#ifdef MX_USB_OTG_FS_VBUS_Power_Pin
  if (otg_fs_role == ARM_USB_ROLE_HOST) {
#if (USB_OTG_FS_VBUS_Power_Pin_Active == 0)
    HAL_GPIO_WritePin (MX_USB_OTG_FS_VBUS_Power_GPIOx, MX_USB_OTG_FS_VBUS_Power_GPIO_Pin, ((state == true) ? GPIO_PIN_RESET : GPIO_PIN_SET));
#else
    HAL_GPIO_WritePin (MX_USB_OTG_FS_VBUS_Power_GPIOx, MX_USB_OTG_FS_VBUS_Power_GPIO_Pin, ((state == true) ? GPIO_PIN_SET   : GPIO_PIN_RESET));
#endif
  }
#endif
}

/**
  \fn          bool OTG_FS_PinGetOC (void)
  \brief       Get state of OverCurrent Pin.
  \return      overcurrent state (true = Overcurrent active, false = No overcurrent)
*/
bool OTG_FS_PinGetOC (void) {

#ifdef MX_USB_OTG_FS_Overcurrent_Pin
  if (otg_fs_role == ARM_USB_ROLE_HOST) {
#if   (USB_OTG_FS_Overcurrent_Pin_Active == 0)
    return ((HAL_GPIO_ReadPin (MX_USB_OTG_FS_Overcurrent_GPIOx, MX_USB_OTG_FS_Overcurrent_GPIO_Pin) == GPIO_PIN_RESET) ? true : false);
#else
    return ((HAL_GPIO_ReadPin (MX_USB_OTG_FS_Overcurrent_GPIOx, MX_USB_OTG_FS_Overcurrent_GPIO_Pin) == GPIO_PIN_SET)   ? true : false);
#endif
  }
  return false;
#else
  return false;
#endif
}
