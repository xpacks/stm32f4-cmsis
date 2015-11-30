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
 * $Date:        10. September 2014
 * $Revision:    V2.00
 *  
 * Driver:       Driver_MCI0
 * Configured:   via RTE_Device.h configuration file 
 * Project:      MCI Driver for ST STM32F4xx
 * ---------------------------------------------------------------------- 
 * Use the following configuration settings in the middleware component
 * to connect to this driver.
 * 
 *   Configuration Setting                 Value
 *   ---------------------                 -----
 *   Connect to hardware via Driver_MCI# = 0
 * -------------------------------------------------------------------- */

/* History:
 *  Version 2.00
 *    Updated to CMSIS Driver API V2.02
 *  Version 1.02
 *    ST StdPeriph Drivers used for GPIO and DMA
 *  Version 1.01
 *    Based on API V1.10 (namespace prefix ARM_ added)
 *  Version 1.00
 *    Initial release
 */ 

#include "MCI_STM32F4xx.h"

#define ARM_MCI_DRV_VERSION ARM_DRIVER_VERSION_MAJOR_MINOR(2,00)  /* driver version */

/* Define Card Detect pin active state */
#if !defined(MemoryCard_CD_Pin_Active)
  #define MemoryCard_CD_Pin_Active GPIO_PIN_RESET
#endif

/* Define Write Protect pin active state */
#if !defined(MemoryCard_WP_Pin_Active)
  #define MemoryCard_WP_Pin_Active GPIO_PIN_SET
#endif


/* DMA callback function */
void DMA_TransferComplete(struct __DMA_HandleTypeDef *hdma);

static DMA_HandleTypeDef hdma_sdio;
static MCI_INFO          MCI;


/* Driver Version */
static const ARM_DRIVER_VERSION DriverVersion = {
  ARM_MCI_API_VERSION,
  ARM_MCI_DRV_VERSION
};

/* Driver Capabilities */
static const ARM_MCI_CAPABILITIES DriverCapabilities = {
  SDIO_CD_PIN,                                    /* cd_state          */
  0,                                              /* cd_event          */
  SDIO_WP_PIN,                                    /* wp_state          */
  0,                                              /* vdd               */
  0,                                              /* vdd_1v8           */
  0,                                              /* vccq              */
  0,                                              /* vccq_1v8          */
  0,                                              /* vccq_1v2          */
  SDIO_BUS_WIDTH_4,                               /* data_width_4      */
  SDIO_BUS_WIDTH_4 && SDIO_BUS_WIDTH_8,           /* data_width_8      */
  0,                                              /* data_width_4_ddr  */
  0,                                              /* data_width_8_ddr  */
  0,                                              /* high_speed        */
  0,                                              /* uhs_signaling     */
  0,                                              /* uhs_tuning        */
  0,                                              /* uhs_sdr50         */
  0,                                              /* uhs_sdr104        */
  0,                                              /* uhs_ddr50         */
  0,                                              /* uhs_driver_type_a */
  0,                                              /* uhs_driver_type_c */
  0,                                              /* uhs_driver_type_d */
  1,                                              /* sdio_interrupt    */
  1,                                              /* read_wait         */
  0,                                              /* suspend_resume    */
  0,                                              /* mmc_interrupt     */
  0,                                              /* mmc_boot          */
  0,                                              /* rst_n             */
  0,                                              /* ccs               */
  0                                               /* ccs_timeout       */
};


/**
  \fn          void Enable_GPIO_Clock (GPIO_TypeDef *port)
  \brief       Enable GPIO clock
*/
static void Enable_GPIO_Clock (GPIO_TypeDef *GPIOx) {
  if      (GPIOx == GPIOA) { __GPIOA_CLK_ENABLE(); }
  else if (GPIOx == GPIOB) { __GPIOB_CLK_ENABLE(); }
  else if (GPIOx == GPIOC) { __GPIOC_CLK_ENABLE(); }
  else if (GPIOx == GPIOD) { __GPIOD_CLK_ENABLE(); }
  else if (GPIOx == GPIOE) { __GPIOE_CLK_ENABLE(); }
#if !defined(STM32F401xC) && !defined(STM32F401xE) && ! defined(STM32F411xE)
  else if (GPIOx == GPIOF) { __GPIOF_CLK_ENABLE(); }
  else if (GPIOx == GPIOG) { __GPIOG_CLK_ENABLE(); }
#endif
  else if (GPIOx == GPIOH) { __GPIOH_CLK_ENABLE(); }
  else if (GPIOx == GPIOI) { __GPIOI_CLK_ENABLE(); }
#if defined(STM32F427xx) || defined(STM32F437xx) || defined(STM32F429xx) || defined(STM32F439xx)
  else if (GPIOx == GPIOJ) { __GPIOJ_CLK_ENABLE(); }
  else                     { __GPIOK_CLK_ENABLE(); }
#endif
}


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
  GPIO_InitTypeDef GPIO_InitStruct;

  if (MCI.flags & MCI_POWER) { return ARM_DRIVER_ERROR; }
  if (MCI.flags & MCI_INIT)  { return ARM_DRIVER_OK;    }

  MCI.cb_event = cb_event;

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
    GPIO_InitStruct.Pull = MX_MemoryCard_CD_GPIO_PuPd;
    HAL_GPIO_Init(MX_MemoryCard_WP_GPIOx, &GPIO_InitStruct);
  #endif

  /* Enable IO compensation cell */
  RCC->APB2ENR  |=  RCC_APB2ENR_SYSCFGEN;
  SYSCFG->CMPCR |= SYSCFG_CMPCR_CMP_PD;
  while (!(SYSCFG->CMPCR & SYSCFG_CMPCR_READY));
  
  /* Set DMA callback function */
  hdma_sdio.XferCpltCallback  = DMA_TransferComplete;

  /* Enable DMA peripheral clock */
  __DMA2_CLK_ENABLE();

  /* Enable DMA stream interrupts in NVIC */
  HAL_NVIC_EnableIRQ(SDIO_DMA_IRQn);

  HAL_NVIC_ClearPendingIRQ(SDIO_IRQn);
  HAL_NVIC_EnableIRQ(SDIO_IRQn);

  /* Clear status */
  MCI.status.command_active  = 0;
  MCI.status.transfer_active = 0;
  MCI.status.sdio_interrupt  = 0;
  MCI.status.ccs             = 0;
  
  MCI.flags = MCI_INIT;

  return ARM_DRIVER_OK;
}


/**
  \fn            int32_t Uninitialize (void)
  \brief         De-initialize Memory Card Interface.
  \return        \ref execution_status
*/
static int32_t Uninitialize (void) {

  if (MCI.flags & MCI_POWER) { return ARM_DRIVER_ERROR; }

  if (MCI.flags & MCI_INIT) {
    MCI.flags = 0;

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

  #if defined (MX_MemoryCard_CD_Pin)
    /* Unconfigure CD (Card Detect) Pin */
    HAL_GPIO_DeInit (MX_MemoryCard_CD_GPIOx, MX_MemoryCard_CD_GPIO_Pin);
  #endif

  #if defined (MX_MemoryCard_WP_Pin)
    /* Unconfigure WP (Write Protect) Pin */
    HAL_GPIO_DeInit (MX_MemoryCard_WP_GPIOx, MX_MemoryCard_WP_GPIO_Pin);
  #endif
  }
  return ARM_DRIVER_OK;
}


/**
  \fn            int32_t PowerControl (ARM_POWER_STATE state)
  \brief         Control Memory Card Interface Power.
  \param[in]     state   Power state \ref ARM_POWER_STATE
  \return        \ref execution_status
*/
static int32_t PowerControl (ARM_POWER_STATE state) {

  if (!(MCI.flags & MCI_INIT)) return ARM_DRIVER_ERROR;

  switch (state) {
    case ARM_POWER_OFF:
      if (MCI.flags & MCI_POWER) {
        MCI.flags &= ~(MCI_POWER | MCI_SETUP);

        /* Disable DMA stream interrupts in NVIC */
        HAL_NVIC_DisableIRQ (SDIO_DMA_IRQn);
        
        /* Disable DMA stream */
        HAL_DMA_DeInit (&hdma_sdio);

        /* Disable SDIO interrupts */
        HAL_NVIC_DisableIRQ(SDIO_IRQn);

        /* Reset/Dereset SDIO peripheral */
        __SDIO_FORCE_RESET();
        __NOP(); __NOP(); __NOP(); __NOP(); 
        __SDIO_RELEASE_RESET();

        /* SDIO peripheral clock disable */
        __SDIO_CLK_DISABLE();
      }
      break;

    case ARM_POWER_FULL:
      if (!(MCI.flags & MCI_POWER)) {
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

        /* Enable SDIO peripheral interrupts in NVIC */
        HAL_NVIC_ClearPendingIRQ(SDIO_IRQn);
        HAL_NVIC_EnableIRQ(SDIO_IRQn);

        /* Enable clock to the card (SDIO_CK) */
        SDIO->POWER = SDIO_POWER_PWRCTRL_1 | SDIO_POWER_PWRCTRL_0;

        MCI.flags |= MCI_POWER;
      }
      break;

    default:
      return ARM_DRIVER_ERROR_UNSUPPORTED;
  }
  return ARM_DRIVER_OK;
}


/**
  \fn            int32_t CardPower (uint32_t voltage)
  \brief         Set Memory Card supply voltage.
  \param[in]     voltage  Memory Card supply voltage
  \return        \ref execution_status
*/
static int32_t CardPower (uint32_t voltage) {

  if (!(MCI.flags & MCI_POWER)) { return ARM_DRIVER_ERROR; }
  return ARM_DRIVER_ERROR_UNSUPPORTED;
}


/**
  \fn            int32_t ReadCD (void)
  \brief         Read Card Detect (CD) state.
  \return        1:card detected, 0:card not detected, or error
*/
static int32_t ReadCD (void) {

  if (!(MCI.flags & MCI_POWER)) { return ARM_DRIVER_ERROR; }

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

  if (!(MCI.flags & MCI_POWER)) { return ARM_DRIVER_ERROR; }

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

  if ((flags & MCI_RESPONSE_EXPECTED_Msk) && (response == NULL)) {
    return ARM_DRIVER_ERROR_PARAMETER;
  }
  if (!(MCI.flags & MCI_SETUP)) {
    return ARM_DRIVER_ERROR;
  }
  if (MCI.status.command_active) {
    return ARM_DRIVER_ERROR_BUSY;
  }
  MCI.status.command_active   = 1;
  MCI.status.command_timeout  = 0;
  MCI.status.command_error    = 0;
  MCI.status.transfer_timeout = 0;
  MCI.status.transfer_error   = 0;
  MCI.status.ccs              = 0;

  if (flags & ARM_MCI_CARD_INITIALIZE) {
    if (!(SDIO->CLKCR & SDIO_CLKCR_CLKEN) || (SDIO->CLKCR & SDIO_CLKCR_PWRSAV)) {
      clkcr = SDIO->CLKCR;
      SDIO->CLKCR = (SDIO->CLKCR & ~SDIO_CLKCR_PWRSAV) | SDIO_CLKCR_CLKEN;
      /* Wait for approximate 1000us */
      i = HAL_RCC_GetHCLKFreq();
      for (i = (i/5000000)*1000; i; i--);
      SDIO->CLKCR = clkcr;
    }
  }

  /* Set command register value */
  cmd = SDIO_CMD_CPSMEN | (cmd & 0xFF);

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
  uint32_t sz, src, dst, cnt, dctrl, tim;

  if ((data == NULL) || (block_count == 0) || (block_size == 0)) return ARM_DRIVER_ERROR_PARAMETER;

  if (!(MCI.flags & MCI_SETUP)) {
    return ARM_DRIVER_ERROR;
  }
  if (MCI.status.transfer_active) {
    return ARM_DRIVER_ERROR_BUSY;
  }

  MCI.xfer.buf = data;
  MCI.xfer.cnt = block_count * block_size;

  cnt = MCI.xfer.cnt;
  if (cnt > 0xFFFF) {
    cnt = 0xFFFF;
  }

  MCI.xfer.cnt -= cnt;
  MCI.xfer.buf += cnt;

  dctrl = 0;

  if (!(mode & ARM_MCI_TRANSFER_WRITE)) {
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
  if (block_size == 512) {
    sz = 9;
  }
  else {
    if (block_size > 16384) {
      return ARM_DRIVER_ERROR_UNSUPPORTED;
    }
    for (sz = 0; sz < 14; sz++) {
      if (block_size & (1 << sz)) {
        break;
      }
    }
  }

  /* Setup DMA stream */
  hdma_sdio.Instance                 = MX_SDIO_DMA_Instance;
  hdma_sdio.Init.Channel             = MX_SDIO_DMA_Channel;
  hdma_sdio.Init.PeriphInc           = DMA_PINC_DISABLE;
  hdma_sdio.Init.MemInc              = DMA_MINC_ENABLE;
  hdma_sdio.Init.PeriphDataAlignment = DMA_PDATAALIGN_WORD;
  hdma_sdio.Init.MemDataAlignment    = DMA_MDATAALIGN_WORD;
  hdma_sdio.Init.Mode                = DMA_PFCTRL;
  hdma_sdio.Init.Priority            = MX_SDIO_DMA_Priority;
  hdma_sdio.Init.FIFOMode            = DMA_FIFOMODE_ENABLE;
  hdma_sdio.Init.FIFOThreshold       = DMA_FIFO_THRESHOLD_FULL;
  hdma_sdio.Init.MemBurst            = DMA_MBURST_INC4;
  hdma_sdio.Init.PeriphBurst         = DMA_PBURST_INC4;

  if (mode & ARM_MCI_TRANSFER_WRITE) {
    hdma_sdio.Init.Direction         = DMA_MEMORY_TO_PERIPH;
    src = (uint32_t)data;
    dst = (uint32_t)&(SDIO->FIFO);
    tim = MCI.wr_timeout;
  }
  else {
    hdma_sdio.Init.Direction         = DMA_PERIPH_TO_MEMORY;
    src = (uint32_t)&(SDIO->FIFO);
    dst = (uint32_t)data;
    tim = MCI.rd_timeout;
  }
  /* Configure stream */
  HAL_DMA_Init(&hdma_sdio);

  /* Enable stream */
  HAL_DMA_Start_IT (&hdma_sdio, src, dst, cnt);

  MCI.dtimer = tim;
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
  uint32_t mask;

  if (!(MCI.flags & MCI_SETUP)) { return ARM_DRIVER_ERROR; }

  /* Disable SDIO interrupts */
  mask = SDIO->MASK;
  SDIO->MASK = 0;

  /* Disable DMA and clear data transfer bit */
  SDIO->DCTRL &= ~(SDIO_DCTRL_DMAEN | SDIO_DCTRL_DTEN);

  HAL_DMA_Abort (&hdma_sdio);

  /* Clear SDIO FIFO */
  while (SDIO->FIFOCNT) {
    SDIO->FIFO;
  }

  MCI.status.command_active  = 0;
  MCI.status.transfer_active = 0;
  MCI.status.sdio_interrupt  = 0;
  MCI.status.ccs             = 0;

  /* Clear pending SDIO interrupts */
  SDIO->ICR = SDIO_ICR_BIT_Msk;

  /* Enable SDIO interrupts */
  SDIO->MASK = mask;

  return ARM_DRIVER_OK;
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
  uint32_t val, div, bps;
  
  if (!(MCI.flags & MCI_POWER)) { return ARM_DRIVER_ERROR; }

  switch (control) {
    case ARM_MCI_BUS_SPEED:
      /* Determine clock divider and set bus speed */
      bps = arg;

      /* bps = SDIOCLK / (div + 2) */
      div = (SDIOCLK + bps - 1) / bps;
      if (div < 2)                 div  = 0;
      else                         div -= 2;
      if (div > SDIO_CLKCR_CLKDIV) div  = SDIO_CLKCR_CLKDIV;
      SDIO->CLKCR = (SDIO->CLKCR & ~SDIO_CLKCR_CLKDIV)   |
                    SDIO_CLKCR_PWRSAV | SDIO_CLKCR_CLKEN | div;
      /* Wait a bit to get stable clock */
      for (val = (SDIOCLK/5000000)*20; val; val--);
      bps = SDIOCLK / (div + 2);

      MCI.rd_timeout = 100 * (bps / 1000);       /* 100ms */
      MCI.wr_timeout = 250 * (bps / 1000);       /* 250ms */

      /* Bus speed configured */
      MCI.flags |= MCI_SETUP;
      return (bps);

    case ARM_MCI_BUS_SPEED_MODE:
      switch (arg) {
        case ARM_MCI_BUS_DEFAULT_SPEED:
          /* Speed mode up to 25MHz */
          SDIO->CLKCR &= ~SDIO_CLKCR_NEGEDGE;
          break;
        case ARM_MCI_BUS_HIGH_SPEED:
          /* Speed mode up to 50MHz */
          SDIO->CLKCR |= SDIO_CLKCR_NEGEDGE;
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
      SDIO->CLKCR &= ~SDIO_CLKCR_WIDBUS;

      switch (arg) {
        case ARM_MCI_BUS_DATA_WIDTH_1:
          break;
        case ARM_MCI_BUS_DATA_WIDTH_4:
          SDIO->CLKCR |= SDIO_CLKCR_WIDBUS_0;
          break;
        case ARM_MCI_BUS_DATA_WIDTH_8:
          SDIO->CLKCR |= SDIO_CLKCR_WIDBUS_1;
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
      MCI.status.sdio_interrupt = 0;
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


/* DMA Stream IRQ Handler */
void SDIO_DMA_Handler (void) {

  HAL_NVIC_ClearPendingIRQ(SDIO_DMA_IRQn);
  HAL_DMA_IRQHandler(&hdma_sdio);
}

/* DMA Callbacks */
void DMA_TransferComplete(struct __DMA_HandleTypeDef *hdma) {
  if (MCI.flags & MCI_DATA_READ) {
    /* Read transfer */
    MCI.status.transfer_active = 0;

    if (MCI.cb_event) {
      (MCI.cb_event)(ARM_MCI_EVENT_TRANSFER_COMPLETE);
    }
  }
}


/* SDIO IRQ Handler */
void SDIO_IRQHandler (void) {
  uint32_t sta, event, mask;

  event = 0;

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
      SDIO->DTIMER = MCI.dtimer;
      SDIO->DLEN   = MCI.dlen;
      SDIO->DCTRL  = MCI.dctrl | SDIO_DCTRL_DTEN;

      MCI.status.transfer_active = 1;
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
    if (!(MCI.flags & MCI_DATA_READ)) {
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
    if (!(MCI.flags & MCI_DATA_READ)) {
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
      MCI.status.transfer_active = 0;
      
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
      MCI.status.command_active = 0;

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
      MCI.status.sdio_interrupt = 1;
      
      if (MCI.cb_event) {
        (MCI.cb_event)(ARM_MCI_EVENT_SDIO_INTERRUPT);
      }
    }
  }
}


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
