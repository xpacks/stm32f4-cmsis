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
 * $Date:        02. June 2015
 * $Revision:    V2.3
 *
 * Driver:       Driver_MCI0
 * Configured:   via RTE_Device.h configuration file
 * Project:      MCI Driver for ST STM32F4xx
 * --------------------------------------------------------------------------
 * Use the following configuration settings in the middleware component
 * to connect to this driver.
 *
 *   Configuration Setting                 Value
 *   ---------------------                 -----
 *   Connect to hardware via Driver_MCI# = 0
 * -------------------------------------------------------------------------- */

/* History:
 *  Version 2.3
 *    Updated initialization, uninitialization and power procedures
 *    Corrected data timeout handling, which is now set only with
 *    ARM_MCI_DATA_TIMEOUT control code.
 *    Added support for STM32F446xx
 *  Version 2.2
 *    Added support for SD high speed bus mode, check device errata sheet
 *    and define MemoryCard_Bus_Mode_HS_Enable for this module to enable
 *    high speed bus mode.
 *  Version 2.1
 *    STM32CubeMX generated code can also be used to configure the driver
 *    Sepatare DMA streams used for receive and transmit
 *  Version 2.0
 *    Updated to CMSIS Driver API V2.02
 *  Version 1.2
 *    ST StdPeriph Drivers used for GPIO and DMA
 *  Version 1.1
 *    Based on API V1.10 (namespace prefix ARM_ added)
 *  Version 1.0
 *    Initial release
 */

/* STM32CubeMX configuration:
 *
 * Pinout tab:
 *   - Select SDIO peripheral and select mode
 *   - To configure Card Detect and Write Protect pins:
 *       - Select desired pins and select GPIO_Input mode
 * Clock Configuration tab:
 *   - Ensure that PLLQ output is at 48MHz
 * Configuration tab:
 *   - Select SDIO under Connectivity section which opens SDIO Configuration window:
 *       - Parameter Settings tab: settings are unused by this driver
 *       - NVIC Settings: enable SDIO global interrupt
 *       - GPIO Settings: configure as needed
 *       - DMA Settings:  Rx and Tx DMA transfers are mandatory by this driver:
 *           - add SDIO_RX and SDIO_TX DMA Request
 *           - select Normal DMA mode (for RX and TX)
 *           - deselect Peripheral Increment Address (for RX and TX)
 *           - select Memory Increment Address (for RX and TX)
 *           - select Use Fifo option (for RX and TX)
 *           - select Full Threshold
 *           - select Word Data Width (for RX and TX)
 *           - select 4 Increment Burst Size (for RX and TX)
 *           - go to NVIC Settings tab and enable RX and TX stream global interrupt
 *
 *   - Select GPIO under System section to configure Card Detect and Write Protect pins:
 *       - Select pin used for Card Detect and add User Label: MemoryCard_CD
 *       - Select pin used for Write Protect and add User Label: MemoryCard_WP
 */

#include "MCI_STM32F4xx.h"

#define ARM_MCI_DRV_VERSION ARM_DRIVER_VERSION_MAJOR_MINOR(2,3)  /* driver version */

/* Enable High Speed bus mode */
#if defined(MemoryCard_Bus_Mode_HS_Enable)
  #define SDIO_BUS_MODE_HS    1U
#else
  #define SDIO_BUS_MODE_HS    0U
#endif

/* Define Card Detect pin active state */
#if !defined(MemoryCard_CD_Pin_Active)
  #define MemoryCard_CD_Pin_Active GPIO_PIN_RESET
#endif

/* Define Write Protect pin active state */
#if !defined(MemoryCard_WP_Pin_Active)
  #define MemoryCard_WP_Pin_Active GPIO_PIN_SET
#endif

#if defined(RTE_DEVICE_FRAMEWORK_CUBE_MX)
extern SD_HandleTypeDef hsd;
#endif

#if defined(RTE_DEVICE_FRAMEWORK_CUBE_MX)
extern
#endif
DMA_HandleTypeDef hdma_sdio_rx;

#if defined(RTE_DEVICE_FRAMEWORK_CUBE_MX)
extern
#endif
DMA_HandleTypeDef hdma_sdio_tx;

static MCI_INFO MCI;

/* DMA callback function */
void RX_DMA_Complete(struct __DMA_HandleTypeDef *hdma);
/* IRQ Handler prototype */
void SDIO_IRQHandler (void);


/* Driver Version */
static const ARM_DRIVER_VERSION DriverVersion = {
  ARM_MCI_API_VERSION,
  ARM_MCI_DRV_VERSION
};

/* Driver Capabilities */
static const ARM_MCI_CAPABILITIES DriverCapabilities = {
  SDIO_CD_PIN,                                    /* cd_state          */
  0U,                                             /* cd_event          */
  SDIO_WP_PIN,                                    /* wp_state          */
  0U,                                             /* vdd               */
  0U,                                             /* vdd_1v8           */
  0U,                                             /* vccq              */
  0U,                                             /* vccq_1v8          */
  0U,                                             /* vccq_1v2          */
  SDIO_BUS_WIDTH_4,                               /* data_width_4      */
  SDIO_BUS_WIDTH_8,                               /* data_width_8      */
  0U,                                             /* data_width_4_ddr  */
  0U,                                             /* data_width_8_ddr  */
  SDIO_BUS_MODE_HS,                               /* high_speed        */
  0U,                                             /* uhs_signaling     */
  0U,                                             /* uhs_tuning        */
  0U,                                             /* uhs_sdr50         */
  0U,                                             /* uhs_sdr104        */
  0U,                                             /* uhs_ddr50         */
  0U,                                             /* uhs_driver_type_a */
  0U,                                             /* uhs_driver_type_c */
  0U,                                             /* uhs_driver_type_d */
  1U,                                             /* sdio_interrupt    */
  1U,                                             /* read_wait         */
  0U,                                             /* suspend_resume    */
  0U,                                             /* mmc_interrupt     */
  0U,                                             /* mmc_boot          */
  0U,                                             /* rst_n             */
  0U,                                             /* ccs               */
  0U                                              /* ccs_timeout       */
};


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
  else if (GPIOx == GPIOE) { __GPIOE_CLK_ENABLE(); }
#if defined(GPIOF)
  else if (GPIOx == GPIOF) { __GPIOF_CLK_ENABLE(); }
#endif
#if defined(GPIOG)
  else if (GPIOx == GPIOG) { __GPIOG_CLK_ENABLE(); }
#endif
#if defined(GPIOH)
  else if (GPIOx == GPIOH) { __GPIOH_CLK_ENABLE(); }
#endif
#if defined(GPIOI)
  else if (GPIOx == GPIOI) { __GPIOI_CLK_ENABLE(); }
#endif
#if defined(GPIOJ)
  else if (GPIOx == GPIOJ) { __GPIOJ_CLK_ENABLE(); }
#endif
#if defined(GPIOK)
  else if (GPIOx == GPIOK) { __GPIOK_CLK_ENABLE(); }
#endif
}
#endif


/**
  \fn          ARM_DRV_VERSION GetVersion (void)
  \brief       Get driver version.
  \return      \ref ARM_DRV_VERSION
*/
static ARM_DRIVER_VERSION GetVersion (void) {
  return DriverVersion;
}


/**
  \fn          ARM_MCI_CAPABILITIES MCI_GetCapabilities (void)
  \brief       Get driver capabilities.
  \return      \ref ARM_MCI_CAPABILITIES
*/
static ARM_MCI_CAPABILITIES GetCapabilities (void) {
  return DriverCapabilities;
}


/**
  \fn            int32_t Initialize (ARM_MCI_SignalEvent_t cb_event)
  \brief         Initialize the Memory Card Interface
  \param[in]     cb_event  Pointer to \ref ARM_MCI_SignalEvent
  \return        \ref execution_status
*/
static int32_t Initialize (ARM_MCI_SignalEvent_t cb_event) {
#if defined(RTE_DEVICE_FRAMEWORK_CLASSIC)
  GPIO_InitTypeDef GPIO_InitStruct;
#endif

  if (MCI.flags & MCI_INIT) { return ARM_DRIVER_OK; }

  #if defined(RTE_DEVICE_FRAMEWORK_CLASSIC)
    /* GPIO Ports Clock Enable */
    Enable_GPIO_Clock (GPIOC);
    Enable_GPIO_Clock (GPIOD);
    
    /* Configure CMD, CK and D0 pins */
    GPIO_InitStruct.Pin = GPIO_PIN_2;
    GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
    GPIO_InitStruct.Pull = GPIO_NOPULL;
    GPIO_InitStruct.Speed = GPIO_SPEED_HIGH;
    GPIO_InitStruct.Alternate = GPIO_AF12_SDIO;
    HAL_GPIO_Init(GPIOD, &GPIO_InitStruct);

    GPIO_InitStruct.Pin = GPIO_PIN_12;
    GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
    GPIO_InitStruct.Pull = GPIO_NOPULL;
    GPIO_InitStruct.Speed = GPIO_SPEED_HIGH;
    GPIO_InitStruct.Alternate = GPIO_AF12_SDIO;
    HAL_GPIO_Init(GPIOC, &GPIO_InitStruct);

    GPIO_InitStruct.Pin = GPIO_PIN_8;
    GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
    GPIO_InitStruct.Pull = GPIO_PULLUP;
    GPIO_InitStruct.Speed = GPIO_SPEED_HIGH;
    GPIO_InitStruct.Alternate = GPIO_AF12_SDIO;
    HAL_GPIO_Init(GPIOC, &GPIO_InitStruct);

    #if (SDIO_BUS_WIDTH_4)
      GPIO_InitStruct.Pin = GPIO_PIN_11|GPIO_PIN_10|GPIO_PIN_9;
      GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
      GPIO_InitStruct.Pull = GPIO_PULLUP;
      GPIO_InitStruct.Speed = GPIO_SPEED_HIGH;
      GPIO_InitStruct.Alternate = GPIO_AF12_SDIO;
      HAL_GPIO_Init(GPIOC, &GPIO_InitStruct);
    #endif

    #if (SDIO_BUS_WIDTH_8)
      Enable_GPIO_Clock (GPIOB);

      GPIO_InitStruct.Pin = GPIO_PIN_8|GPIO_PIN_9;
      GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
      GPIO_InitStruct.Pull = GPIO_PULLUP;
      GPIO_InitStruct.Speed = GPIO_SPEED_HIGH;
      GPIO_InitStruct.Alternate = GPIO_AF12_SDIO;
      HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

      GPIO_InitStruct.Pin = GPIO_PIN_7|GPIO_PIN_6;
      GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
      GPIO_InitStruct.Pull = GPIO_PULLUP;
      GPIO_InitStruct.Speed = GPIO_SPEED_HIGH;
      GPIO_InitStruct.Alternate = GPIO_AF12_SDIO;
      HAL_GPIO_Init(GPIOC, &GPIO_InitStruct);
    #endif

    /* Configure CD (Card Detect) Pin */
    #if defined (MX_MemoryCard_CD_Pin)
      Enable_GPIO_Clock (MX_MemoryCard_CD_GPIOx);

      GPIO_InitStruct.Pin  = MX_MemoryCard_CD_GPIO_Pin;
      GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
      GPIO_InitStruct.Pull = MX_MemoryCard_CD_GPIO_PuPd;
      HAL_GPIO_Init(MX_MemoryCard_CD_GPIOx, &GPIO_InitStruct);
    #endif

    /* Configure WP (Write Protect) Pin */
    #if defined (MX_MemoryCard_WP_Pin)
      Enable_GPIO_Clock (MX_MemoryCard_WP_GPIOx);

      GPIO_InitStruct.Pin  = MX_MemoryCard_WP_GPIO_Pin;
      GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
      GPIO_InitStruct.Pull = MX_MemoryCard_WP_GPIO_PuPd;
      HAL_GPIO_Init(MX_MemoryCard_WP_GPIOx, &GPIO_InitStruct);
    #endif

    /* Enable IO compensation cell */
    RCC->APB2ENR  |=  RCC_APB2ENR_SYSCFGEN;
    SYSCFG->CMPCR |= SYSCFG_CMPCR_CMP_PD;
    while ((SYSCFG->CMPCR & SYSCFG_CMPCR_READY) == 0U) {
      ; /* Wait until compensation cell ready */
    }

    /* Enable DMA peripheral clock */
    __DMA2_CLK_ENABLE();
    
    /* Configure DMA receive stream */
    hdma_sdio_rx.Instance                 = MX_SDIO_RX_DMA_Instance;
    hdma_sdio_rx.Init.Channel             = MX_SDIO_RX_DMA_Channel;
    hdma_sdio_rx.Init.Direction           = DMA_PERIPH_TO_MEMORY;
    hdma_sdio_rx.Init.PeriphInc           = DMA_PINC_DISABLE;
    hdma_sdio_rx.Init.MemInc              = DMA_MINC_ENABLE;
    hdma_sdio_rx.Init.PeriphDataAlignment = DMA_PDATAALIGN_WORD;
    hdma_sdio_rx.Init.MemDataAlignment    = DMA_MDATAALIGN_WORD;
    hdma_sdio_rx.Init.Mode                = DMA_PFCTRL;
    hdma_sdio_rx.Init.Priority            = MX_SDIO_RX_DMA_Priority;
    hdma_sdio_rx.Init.FIFOMode            = DMA_FIFOMODE_ENABLE;
    hdma_sdio_rx.Init.FIFOThreshold       = DMA_FIFO_THRESHOLD_FULL;
    hdma_sdio_rx.Init.MemBurst            = DMA_MBURST_INC4;
    hdma_sdio_rx.Init.PeriphBurst         = DMA_PBURST_INC4;

    if (HAL_DMA_Init(&hdma_sdio_rx) != HAL_OK) {
      return ARM_DRIVER_ERROR;
    }

    /* Configure DMA transmit stream */
    hdma_sdio_tx.Instance                 = MX_SDIO_TX_DMA_Instance;
    hdma_sdio_tx.Init.Channel             = MX_SDIO_TX_DMA_Channel;
    hdma_sdio_tx.Init.Direction           = DMA_MEMORY_TO_PERIPH;
    hdma_sdio_tx.Init.PeriphInc           = DMA_PINC_DISABLE;
    hdma_sdio_tx.Init.MemInc              = DMA_MINC_ENABLE;
    hdma_sdio_tx.Init.PeriphDataAlignment = DMA_PDATAALIGN_WORD;
    hdma_sdio_tx.Init.MemDataAlignment    = DMA_MDATAALIGN_WORD;
    hdma_sdio_tx.Init.Mode                = DMA_PFCTRL;
    hdma_sdio_tx.Init.Priority            = MX_SDIO_TX_DMA_Priority;
    hdma_sdio_tx.Init.FIFOMode            = DMA_FIFOMODE_ENABLE;
    hdma_sdio_tx.Init.FIFOThreshold       = DMA_FIFO_THRESHOLD_FULL;
    hdma_sdio_tx.Init.MemBurst            = DMA_MBURST_INC4;
    hdma_sdio_tx.Init.PeriphBurst         = DMA_PBURST_INC4;

    if (HAL_DMA_Init(&hdma_sdio_tx) != HAL_OK) {
      return ARM_DRIVER_ERROR;
    }
  #else
    hsd.Instance = SDIO;
  #endif

  /* Set DMA callback function */
  hdma_sdio_rx.XferCpltCallback  = &RX_DMA_Complete;

  /* Clear control structure */
  memset (&MCI, 0, sizeof (MCI_INFO));

  MCI.cb_event = cb_event;
  MCI.flags    = MCI_INIT;

  return ARM_DRIVER_OK;
}


/**
  \fn            int32_t Uninitialize (void)
  \brief         De-initialize Memory Card Interface.
  \return        \ref execution_status
*/
static int32_t Uninitialize (void) {

  MCI.flags = 0U;

  #if defined(RTE_DEVICE_FRAMEWORK_CLASSIC)
    /* SDIO_CMD, SDIO_CK and SDIO_D0 pins */
    HAL_GPIO_DeInit(GPIOD, GPIO_PIN_2);
    HAL_GPIO_DeInit(GPIOC, GPIO_PIN_12|GPIO_PIN_8);

    #if (SDIO_BUS_WIDTH_4)
      /* SDIO_D[3..1] */
      HAL_GPIO_DeInit(GPIOC, GPIO_PIN_11|GPIO_PIN_10|GPIO_PIN_9);
    #endif

    #if (SDIO_BUS_WIDTH_8)
      /* SDIO_D[7..4] */
      HAL_GPIO_DeInit(GPIOB, GPIO_PIN_8|GPIO_PIN_9);
      HAL_GPIO_DeInit(GPIOC, GPIO_PIN_7|GPIO_PIN_6);
    #endif
  #endif /* RTE_DEVICE_FRAMEWORK_CLASSIC */

  #if defined (MX_MemoryCard_CD_Pin)
    /* Unconfigure CD (Card Detect) Pin */
    HAL_GPIO_DeInit (MX_MemoryCard_CD_GPIOx, MX_MemoryCard_CD_GPIO_Pin);
  #endif

  #if defined (MX_MemoryCard_WP_Pin)
    /* Unconfigure WP (Write Protect) Pin */
    HAL_GPIO_DeInit (MX_MemoryCard_WP_GPIOx, MX_MemoryCard_WP_GPIO_Pin);
  #endif

  return ARM_DRIVER_OK;
}


/**
  \fn            int32_t PowerControl (ARM_POWER_STATE state)
  \brief         Control Memory Card Interface Power.
  \param[in]     state   Power state \ref ARM_POWER_STATE
  \return        \ref execution_status
*/
static int32_t PowerControl (ARM_POWER_STATE state) {
  int32_t status;

  status = ARM_DRIVER_OK;

  switch (state) {
    case ARM_POWER_OFF:
      #if defined(RTE_DEVICE_FRAMEWORK_CLASSIC)
        /* Disable SDIO interrupts in NVIC */
        HAL_NVIC_DisableIRQ (SDIO_IRQn);

        /* Disable DMA stream interrupts in NVIC */
        HAL_NVIC_DisableIRQ (SDIO_RX_DMA_IRQn);
        HAL_NVIC_DisableIRQ (SDIO_TX_DMA_IRQn);

        /* Disable DMA stream */
        if (HAL_DMA_DeInit (&hdma_sdio_rx) != HAL_OK) {
          status = ARM_DRIVER_ERROR;
        }
        if (HAL_DMA_DeInit (&hdma_sdio_tx) != HAL_OK) {
          status = ARM_DRIVER_ERROR;
        }
      #else
        HAL_SD_MspDeInit (&hsd);
      #endif

      MCI.flags = MCI_INIT;

      /* Clear status */
      MCI.status.command_active   = 0U;
      MCI.status.command_timeout  = 0U;
      MCI.status.command_error    = 0U;
      MCI.status.transfer_active  = 0U;
      MCI.status.transfer_timeout = 0U;
      MCI.status.transfer_error   = 0U;
      MCI.status.sdio_interrupt   = 0U;
      MCI.status.ccs              = 0U;

      /* Reset/Dereset SDIO peripheral */
      __SDIO_FORCE_RESET();
      __NOP(); __NOP(); __NOP(); __NOP(); 
      __SDIO_RELEASE_RESET();

      /* SDIO peripheral clock disable */
      __SDIO_CLK_DISABLE();
      break;

    case ARM_POWER_FULL:
      if ((MCI.flags & MCI_POWER) == 0) {
        #if defined(RTE_DEVICE_FRAMEWORK_CUBE_MX)
          HAL_SD_MspInit (&hsd);
        #endif

        /* Clear response and transfer variables */
        MCI.response = NULL;
        MCI.xfer.cnt = 0U;

        /* Enable SDIO peripheral clock */
        __SDIO_CLK_ENABLE();

        /* Enable SDIO peripheral interrupts */
        SDIO->MASK = SDIO_MASK_DATAENDIE  |
                     SDIO_MASK_STBITERRIE |
                     SDIO_MASK_CMDSENTIE  |
                     SDIO_MASK_CMDRENDIE  |
                     SDIO_MASK_DTIMEOUTIE |
                     SDIO_MASK_CTIMEOUTIE |
                     SDIO_MASK_DCRCFAILIE |
                     SDIO_MASK_CCRCFAILIE ;

        /* Set max data timeout */
        SDIO->DTIMER = 0xFFFFFFFF;

        /* Enable clock to the card (SDIO_CK) */
        SDIO->POWER = SDIO_POWER_PWRCTRL_1 | SDIO_POWER_PWRCTRL_0;

        /* Enable DMA stream interrupts in NVIC */
        HAL_NVIC_EnableIRQ(SDIO_RX_DMA_IRQn);
        HAL_NVIC_EnableIRQ(SDIO_TX_DMA_IRQn);

        HAL_NVIC_ClearPendingIRQ(SDIO_IRQn);
        HAL_NVIC_EnableIRQ(SDIO_IRQn);

        MCI.flags |= MCI_POWER;
      }
      break;

    case ARM_POWER_LOW:
    default:
      return ARM_DRIVER_ERROR_UNSUPPORTED;
  }
  return status;
}


/**
  \fn            int32_t CardPower (uint32_t voltage)
  \brief         Set Memory Card supply voltage.
  \param[in]     voltage  Memory Card supply voltage
  \return        \ref execution_status
*/
static int32_t CardPower (uint32_t voltage) {

  if ((MCI.flags & MCI_POWER) == 0U) { return ARM_DRIVER_ERROR; }
  return ARM_DRIVER_ERROR_UNSUPPORTED;
}


/**
  \fn            int32_t ReadCD (void)
  \brief         Read Card Detect (CD) state.
  \return        1:card detected, 0:card not detected, or error
*/
static int32_t ReadCD (void) {

  if ((MCI.flags & MCI_POWER) == 0U) { return ARM_DRIVER_ERROR; }

  /* Read CD (Card Detect) Pin */
  #if defined (MX_MemoryCard_CD_Pin)
  if (HAL_GPIO_ReadPin (MX_MemoryCard_CD_GPIOx, MX_MemoryCard_CD_GPIO_Pin) == MemoryCard_CD_Pin_Active) {
    /* Card Detect switch is active */
    return (1);
  }
  #endif
  return (0);
}


/**
  \fn            int32_t ReadWP (void)
  \brief         Read Write Protect (WP) state.
  \return        1:write protected, 0:not write protected, or error
*/
static int32_t ReadWP (void) {

  if ((MCI.flags & MCI_POWER) == 0U) { return ARM_DRIVER_ERROR; }

  /* Read WP (Write Protect) Pin */
  #if defined (MX_MemoryCard_WP_Pin)
  if (HAL_GPIO_ReadPin (MX_MemoryCard_WP_GPIOx, MX_MemoryCard_WP_GPIO_Pin) == MemoryCard_WP_Pin_Active) {
    /* Write protect switch is active */
    return (1);
  }
  #endif
  return (0);
}


/**
  \fn            int32_t SendCommand (uint32_t  cmd,
                                      uint32_t  arg,
                                      uint32_t  flags,
                                      uint32_t *response)
  \brief         Send Command to card and get the response.
  \param[in]     cmd       Memory Card command
  \param[in]     arg       Command argument
  \param[in]     flags     Command flags
  \param[out]    response  Pointer to buffer for response
  \return        \ref execution_status
*/
static int32_t SendCommand (uint32_t cmd, uint32_t arg, uint32_t flags, uint32_t *response) {
  uint32_t i, clkcr;

  if (((flags & MCI_RESPONSE_EXPECTED_Msk) != 0U) && (response == NULL)) {
    return ARM_DRIVER_ERROR_PARAMETER;
  }
  if ((MCI.flags & MCI_SETUP) == 0U) {
    return ARM_DRIVER_ERROR;
  }
  if (MCI.status.command_active) {
    return ARM_DRIVER_ERROR_BUSY;
  }
  MCI.status.command_active   = 1U;
  MCI.status.command_timeout  = 0U;
  MCI.status.command_error    = 0U;
  MCI.status.transfer_timeout = 0U;
  MCI.status.transfer_error   = 0U;
  MCI.status.ccs              = 0U;

  if (flags & ARM_MCI_CARD_INITIALIZE) {
    clkcr = SDIO->CLKCR;
    if (((clkcr & SDIO_CLKCR_CLKEN) == 0) || ((clkcr & SDIO_CLKCR_PWRSAV) != 0)) {
      SDIO->CLKCR = (SDIO->CLKCR & ~SDIO_CLKCR_PWRSAV) | SDIO_CLKCR_CLKEN;

      i = HAL_RCC_GetHCLKFreq();
      for (i = (i/5000000U)*1000U; i; i--) {
        ; /* Wait for approximate 1000us */
      }
      SDIO->CLKCR = clkcr;
    }
  }

  /* Set command register value */
  cmd = SDIO_CMD_CPSMEN | (cmd & 0xFFU);

  MCI.response = response;
  MCI.flags   &= ~(MCI_RESP_CRC | MCI_RESP_LONG);

  switch (flags & ARM_MCI_RESPONSE_Msk) {
    case ARM_MCI_RESPONSE_NONE:
      /* No response expected (wait CMDSENT) */
      break;

    case ARM_MCI_RESPONSE_SHORT:
    case ARM_MCI_RESPONSE_SHORT_BUSY:
      /* Short response expected (wait CMDREND or CCRCFAIL) */
      cmd |= SDIO_CMD_WAITRESP_0;
      break;

    case ARM_MCI_RESPONSE_LONG:
      MCI.flags |= MCI_RESP_LONG;
      /* Long response expected (wait CMDREND or CCRCFAIL) */
      cmd |= SDIO_CMD_WAITRESP_1 | SDIO_CMD_WAITRESP_0;
      break;

    default:
      return ARM_DRIVER_ERROR;
  }
  if (flags & ARM_MCI_RESPONSE_CRC) {
    MCI.flags |= MCI_RESP_CRC;
  }
  if (flags & ARM_MCI_TRANSFER_DATA) {
    MCI.flags |= MCI_DATA_XFER;
  }

  /* Clear all interrupt flags */
  SDIO->ICR = SDIO_ICR_BIT_Msk;

  /* Send the command */
  SDIO->ARG = arg;
  SDIO->CMD = cmd;

  return ARM_DRIVER_OK;
}


/**
  \fn            int32_t SetupTransfer (uint8_t *data,
                                        uint32_t block_count,
                                        uint32_t block_size,
                                        uint32_t mode)
  \brief         Setup read or write transfer operation.
  \param[in,out] data         Pointer to data block(s) to be written or read
  \param[in]     block_count  Number of blocks
  \param[in]     block_size   Size of a block in bytes
  \param[in]     mode         Transfer mode
  \return        \ref execution_status
*/
static int32_t SetupTransfer (uint8_t *data, uint32_t block_count, uint32_t block_size, uint32_t mode) {
  uint32_t sz, cnt, dctrl;

  if ((data == NULL) || (block_count == 0U) || (block_size == 0U)) { return ARM_DRIVER_ERROR_PARAMETER; }

  if ((MCI.flags & MCI_SETUP) == 0U) {
    return ARM_DRIVER_ERROR;
  }
  if (MCI.status.transfer_active) {
    return ARM_DRIVER_ERROR_BUSY;
  }

  MCI.xfer.buf = data;
  MCI.xfer.cnt = block_count * block_size;

  cnt = MCI.xfer.cnt;
  if (cnt > 0xFFFFU) {
    cnt = 0xFFFFU;
  }

  MCI.xfer.cnt -= cnt;
  MCI.xfer.buf += cnt;

  dctrl = 0U;

  if ((mode & ARM_MCI_TRANSFER_WRITE) == 0) {
    /* Direction: From card to controller */
    MCI.flags |= MCI_DATA_READ;
    dctrl |= SDIO_DCTRL_DTDIR;
  }
  else {
    MCI.flags &= ~MCI_DATA_READ;
  }

  if (mode & ARM_MCI_TRANSFER_STREAM) {
    /* Stream or SDIO multibyte data transfer enable */
    dctrl |= SDIO_DCTRL_DTMODE;
  }
  
  /* Set data block size */
  if (block_size == 512U) {
    sz = 9U;
  }
  else {
    if (block_size > 16384U) {
      return ARM_DRIVER_ERROR_UNSUPPORTED;
    }
    for (sz = 0U; sz < 14U; sz++) {
      if (block_size & (1UL << sz)) {
        break;
      }
    }
  }

  if (mode & ARM_MCI_TRANSFER_WRITE) {
    /* Enable TX DMA stream */
    if (HAL_DMA_Start_IT (&hdma_sdio_tx, (uint32_t)data, (uint32_t)&(SDIO->FIFO), cnt) != HAL_OK) {
      return ARM_DRIVER_ERROR;
    }
  }
  else {
    /* Enable RX DMA stream */
    if (HAL_DMA_Start_IT (&hdma_sdio_rx, (uint32_t)&(SDIO->FIFO), (uint32_t)data, cnt) != HAL_OK) {
      return ARM_DRIVER_ERROR;
    }
  }

  MCI.dlen   = cnt;
  MCI.dctrl  = dctrl | (sz << 4) | SDIO_DCTRL_DMAEN;

  return (ARM_DRIVER_OK);
}


/**
  \fn            int32_t AbortTransfer (void)
  \brief         Abort current read/write data transfer.
  \return        \ref execution_status
*/
static int32_t AbortTransfer (void) {
  int32_t  status;
  uint32_t mask;

  if ((MCI.flags & MCI_SETUP) == 0U) { return ARM_DRIVER_ERROR; }

  status = ARM_DRIVER_OK;

  /* Disable SDIO interrupts */
  mask = SDIO->MASK;
  SDIO->MASK = 0U;

  /* Disable DMA and clear data transfer bit */
  SDIO->DCTRL &= ~(SDIO_DCTRL_DMAEN | SDIO_DCTRL_DTEN);

  if (HAL_DMA_Abort (&hdma_sdio_rx) != HAL_OK) {
    status = ARM_DRIVER_ERROR;
  }
  if (HAL_DMA_Abort (&hdma_sdio_tx) != HAL_OK) {
    status = ARM_DRIVER_ERROR;
  }

  /* Clear SDIO FIFO */
  while (SDIO->FIFOCNT) {
    SDIO->FIFO;
  }

  MCI.status.command_active  = 0U;
  MCI.status.transfer_active = 0U;
  MCI.status.sdio_interrupt  = 0U;
  MCI.status.ccs             = 0U;

  /* Clear pending SDIO interrupts */
  SDIO->ICR = SDIO_ICR_BIT_Msk;

  /* Enable SDIO interrupts */
  SDIO->MASK = mask;

  return status;
}


/**
  \fn            int32_t Control (uint32_t control, uint32_t arg)
  \brief         Control MCI Interface.
  \param[in]     control  Operation
  \param[in]     arg      Argument of operation (optional)
  \return        \ref execution_status
*/
static int32_t Control (uint32_t control, uint32_t arg) {
  GPIO_InitTypeDef GPIO_InitStruct;
  uint32_t val, clkdiv, bps;

  if ((MCI.flags & MCI_POWER) == 0U) { return ARM_DRIVER_ERROR; }

  switch (control) {
    case ARM_MCI_BUS_SPEED:
      /* Determine clock divider and set bus speed */
      bps = arg;

      if ((bps < SDIOCLK) || (SDIO_BUS_MODE_HS == 0U)) {
        /* bps = SDIOCLK / (clkdiv + 2) */
        clkdiv = (SDIOCLK + bps - 1U) / bps;

        if (clkdiv < 2) { clkdiv  = 0U; }
        else            { clkdiv -= 2U; }

        if (clkdiv > SDIO_CLKCR_CLKDIV) {
          clkdiv  = SDIO_CLKCR_CLKDIV;
        }

        SDIO->CLKCR = (SDIO->CLKCR & ~SDIO_CLKCR_CLKDIV)   |
                      SDIO_CLKCR_PWRSAV | SDIO_CLKCR_CLKEN | clkdiv;
        bps = SDIOCLK / (clkdiv + 2U);
      }
      else {
        /* Max output clock is SDIOCLK */
        SDIO->CLKCR |= SDIO_CLKCR_BYPASS | SDIO_CLKCR_PWRSAV | SDIO_CLKCR_CLKEN;

        bps = SDIOCLK;
      }

      for (val = (SDIOCLK/5000000U)*20U; val; val--) {
        ; /* Wait a bit to get stable clock */
      }

      /* Bus speed configured */
      MCI.flags |= MCI_SETUP;
      return ((int32_t)bps);

    case ARM_MCI_BUS_SPEED_MODE:
      switch (arg) {
        case ARM_MCI_BUS_DEFAULT_SPEED:
          /* Speed mode up to 25MHz */
          SDIO->CLKCR &= ~SDIO_CLKCR_NEGEDGE;
          break;
        case ARM_MCI_BUS_HIGH_SPEED:
          /* Speed mode up to 50MHz */
          /* Errata: configuration with the NEGEDGE bit set should not be used. */
          break;
        default: return ARM_DRIVER_ERROR_UNSUPPORTED;
      }
      break;

    case ARM_MCI_BUS_CMD_MODE:
      switch (arg) {
        case ARM_MCI_BUS_CMD_OPEN_DRAIN:
          /* Configure command line in open-drain mode */
          val = GPIO_MODE_AF_OD;
          break;
        case ARM_MCI_BUS_CMD_PUSH_PULL:
          /* Configure command line in push-pull mode */
          val = GPIO_MODE_AF_PP;
          break;
        default:
          return ARM_DRIVER_ERROR_UNSUPPORTED;
      }

      GPIO_InitStruct.Pin = GPIO_PIN_2;
      GPIO_InitStruct.Mode = val;
      GPIO_InitStruct.Pull = GPIO_PULLUP;
      GPIO_InitStruct.Speed = GPIO_SPEED_HIGH;
      GPIO_InitStruct.Alternate = GPIO_AF12_SDIO;
      HAL_GPIO_Init(GPIOD, &GPIO_InitStruct);
      break;

    case ARM_MCI_BUS_DATA_WIDTH:
      switch (arg) {
        case ARM_MCI_BUS_DATA_WIDTH_1:
          SDIO->CLKCR &= ~SDIO_CLKCR_WIDBUS;
          break;
        case ARM_MCI_BUS_DATA_WIDTH_4:
          SDIO->CLKCR = (SDIO->CLKCR & ~SDIO_CLKCR_WIDBUS) | SDIO_CLKCR_WIDBUS_0;
          break;
        case ARM_MCI_BUS_DATA_WIDTH_8:
          SDIO->CLKCR = (SDIO->CLKCR & ~SDIO_CLKCR_WIDBUS) | SDIO_CLKCR_WIDBUS_1;
          break;
        default:
          return ARM_DRIVER_ERROR_UNSUPPORTED;
      }
      break;

    case ARM_MCI_CONTROL_CLOCK_IDLE:
      if (arg) {
        /* Clock generation enabled when idle */
        SDIO->CLKCR &= ~SDIO_CLKCR_PWRSAV;
      }
      else {
        /* Clock generation disabled when idle */
        SDIO->CLKCR |= SDIO_CLKCR_PWRSAV;
      }
      break;

    case ARM_MCI_DATA_TIMEOUT:
      SDIO->DTIMER = arg;
      break;

    case ARM_MCI_MONITOR_SDIO_INTERRUPT:
      MCI.status.sdio_interrupt = 0U;
      SDIO->MASK |= SDIO_MASK_SDIOITIE;
      break;

    case ARM_MCI_CONTROL_READ_WAIT:
      if (arg) {
        /* Assert read wait */
        MCI.flags |= MCI_READ_WAIT;
      }
      else {
        /* Clear read wait */
        MCI.flags &= ~MCI_READ_WAIT;
        SDIO->DCTRL &= ~SDIO_DCTRL_RWSTOP;
      }
      break;

    default: return ARM_DRIVER_ERROR_UNSUPPORTED;
  }

  return ARM_DRIVER_OK;
}


/**
  \fn            ARM_MCI_STATUS GetStatus (void)
  \brief         Get MCI status.
  \return        MCI status \ref ARM_MCI_STATUS
*/
static ARM_MCI_STATUS GetStatus (void) {
  return MCI.status;
}


/* SDIO IRQ Handler */
void SDIO_IRQHandler (void) {
  uint32_t sta, event, mask;

  event = 0U;

  /* Read SDIO interrupt status */
  sta = SDIO->STA;

  if (sta & SDIO_STA_CCRCFAIL) {
    SDIO->ICR = SDIO_ICR_CCRCFAILC;
    /* Command response CRC check failed */
    if (MCI.flags & MCI_RESP_CRC) {
      MCI.status.command_error = 1;

      event |= ARM_MCI_EVENT_COMMAND_ERROR;
    }
    else {
      /* Ignore CRC error and read the response */
      sta |= SDIO_STA_CMDREND;
    }
  }
  if (sta & SDIO_STA_DCRCFAIL) {
    SDIO->ICR = SDIO_ICR_DCRCFAILC;
    /* Data block CRC check failed */
    MCI.status.transfer_error = 1;

    event |= ARM_MCI_EVENT_TRANSFER_ERROR;
  }
  if (sta & SDIO_STA_CTIMEOUT) {
    SDIO->ICR = SDIO_ICR_CTIMEOUTC;
    /* Command response timeout */
    MCI.status.command_timeout = 1;

    event |= ARM_MCI_EVENT_COMMAND_TIMEOUT;
  }
  if (sta & SDIO_STA_DTIMEOUT) {
    SDIO->ICR = SDIO_ICR_DTIMEOUTC;
    /* Data timeout */
    MCI.status.transfer_timeout = 1;

    event |= ARM_MCI_EVENT_TRANSFER_TIMEOUT;
  }
  if (sta & SDIO_STA_CMDREND) {
    SDIO->ICR = SDIO_ICR_CMDRENDC;
    /* Command response received */
    event |= ARM_MCI_EVENT_COMMAND_COMPLETE;

    if (MCI.response) {
      /* Read response registers */
      if (MCI.flags & MCI_RESP_LONG) {
        MCI.response[0] = SDIO->RESP4;
        MCI.response[1] = SDIO->RESP3;
        MCI.response[2] = SDIO->RESP2;
        MCI.response[3] = SDIO->RESP1;
      }
      else {
        MCI.response[0] = SDIO->RESP1;
      }
    }
    if (MCI.flags & MCI_DATA_XFER) {
      MCI.flags &= ~MCI_DATA_XFER;

      if (MCI.flags & MCI_READ_WAIT) {
        MCI.dctrl |= SDIO_DCTRL_RWSTART;
      }

      /* Start data transfer */
      SDIO->DLEN   = MCI.dlen;
      SDIO->DCTRL  = MCI.dctrl | SDIO_DCTRL_DTEN;

      MCI.status.transfer_active = 1U;
    }
  }
  if (sta & SDIO_STA_CMDSENT) {
    SDIO->ICR = SDIO_ICR_CMDSENTC;
    /* Command sent (no response required) */
    event |= ARM_MCI_EVENT_COMMAND_COMPLETE;
  }
  if (sta & SDIO_STA_DATAEND) {
    SDIO->ICR = SDIO_ICR_DATAENDC;
    /* Data end (DCOUNT is zero) */
    if ((MCI.flags & MCI_DATA_READ) == 0) {
    /* Write transfer */
      SDIO->MASK |= SDIO_MASK_DBCKENDIE;
    }
  }
  if (sta & SDIO_STA_STBITERR) {
    SDIO->ICR = SDIO_ICR_STBITERRC;
    /* Start bit not detected on all data signals */
    event |= ARM_MCI_EVENT_TRANSFER_ERROR;
  }
  if (sta & SDIO_STA_DBCKEND) {
    SDIO->ICR = SDIO_ICR_DBCKENDC;
    /* Data block sent/received (CRC check passed) */
    if ((MCI.flags & MCI_DATA_READ) == 0) {
      /* Write transfer */
      if (MCI.xfer.cnt == 0) {
        event |= ARM_MCI_EVENT_TRANSFER_COMPLETE;
      }
    }
    SDIO->MASK &= ~SDIO_MASK_DBCKENDIE;
  }
  if (sta & SDIO_STA_SDIOIT) {
    SDIO->ICR = SDIO_ICR_SDIOITC;
    /* Disable interrupt (must be re-enabled using Control) */
    SDIO->MASK &= SDIO_MASK_SDIOITIE;

    event |= ARM_MCI_EVENT_SDIO_INTERRUPT;
  }

  if (event) {
    /* Check for transfer events */
    mask = ARM_MCI_EVENT_TRANSFER_ERROR   |
           ARM_MCI_EVENT_TRANSFER_TIMEOUT |
           ARM_MCI_EVENT_TRANSFER_COMPLETE;
    if (event & mask) {
      MCI.status.transfer_active = 0U;

      if (MCI.cb_event) {
        if (event & ARM_MCI_EVENT_TRANSFER_ERROR) {
          (MCI.cb_event)(ARM_MCI_EVENT_TRANSFER_ERROR);
        }
        else if (event & ARM_MCI_EVENT_TRANSFER_TIMEOUT) {
          (MCI.cb_event)(ARM_MCI_EVENT_TRANSFER_TIMEOUT);
        }
        else {
          (MCI.cb_event)(ARM_MCI_EVENT_TRANSFER_COMPLETE);
        }
      }
    }
    /* Check for command events */
    mask = ARM_MCI_EVENT_COMMAND_ERROR   |
           ARM_MCI_EVENT_COMMAND_TIMEOUT |
           ARM_MCI_EVENT_COMMAND_COMPLETE;
    if (event & mask) {
      MCI.status.command_active = 0U;

      if (MCI.cb_event) {
        if (event & ARM_MCI_EVENT_COMMAND_ERROR) {
          (MCI.cb_event)(ARM_MCI_EVENT_COMMAND_ERROR);
        }
        else if (event & ARM_MCI_EVENT_COMMAND_TIMEOUT) {
          (MCI.cb_event)(ARM_MCI_EVENT_COMMAND_TIMEOUT);
        }
        else {
          (MCI.cb_event)(ARM_MCI_EVENT_COMMAND_COMPLETE);
        }
      }
    }
    /* Check for SDIO INT event */
    if (event & ARM_MCI_EVENT_SDIO_INTERRUPT) {
      MCI.status.sdio_interrupt = 1U;

      if (MCI.cb_event) {
        (MCI.cb_event)(ARM_MCI_EVENT_SDIO_INTERRUPT);
      }
    }
  }
}


/* Rx DMA Callback */
void RX_DMA_Complete(struct __DMA_HandleTypeDef *hdma) {

  MCI.status.transfer_active = 0U;

  if (MCI.cb_event) {
    (MCI.cb_event)(ARM_MCI_EVENT_TRANSFER_COMPLETE);
  }
}

#if defined(RTE_DEVICE_FRAMEWORK_CLASSIC)
/* RX DMA Stream IRQ Handler */
void SDIO_RX_DMA_Handler (void) {

  HAL_NVIC_ClearPendingIRQ(SDIO_RX_DMA_IRQn);
  HAL_DMA_IRQHandler(&hdma_sdio_rx);
}

/* TX DMA Stream IRQ Handler */
void SDIO_TX_DMA_Handler (void) {

  HAL_NVIC_ClearPendingIRQ(SDIO_TX_DMA_IRQn);
  HAL_DMA_IRQHandler(&hdma_sdio_tx);
}
#endif


/* MCI Driver Control Block */
ARM_DRIVER_MCI Driver_MCI0 = {
  GetVersion,
  GetCapabilities,
  Initialize,
  Uninitialize,
  PowerControl,
  CardPower,
  ReadCD,
  ReadWP,
  SendCommand,
  SetupTransfer,
  AbortTransfer,
  Control,
  GetStatus
};
