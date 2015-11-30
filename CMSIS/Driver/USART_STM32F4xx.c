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
 * $Date:        25. September 2014
 * $Revision:    V2.00
 *  
 * Driver:       Driver_USART1, Driver_USART2, Driver_USART3, Driver_USART4,
 *               Driver_USART5, Driver_USART6, Driver_USART7, Driver_USART8,
 * Configured:   via RTE_Device.h or MX_Device.h configuration file 
 * Project:      USART Driver for ST STM32F4xx
 * ---------------------------------------------------------------------- 
 * Use the following configuration settings in the middleware component
 * to connect to this driver.
 * 
 *   Configuration Setting                Value     UART Interface
 *   ---------------------                -----     --------------
 *   Connect to hardware via Driver_USART# = 1       use USART1
 *   Connect to hardware via Driver_USART# = 2       use USART2
 *   Connect to hardware via Driver_USART# = 3       use USART3
 *   Connect to hardware via Driver_USART# = 4       use UART4
 *   Connect to hardware via Driver_USART# = 5       use UART5
 *   Connect to hardware via Driver_USART# = 6       use USART6
 *   Connect to hardware via Driver_USART# = 7       use UART7
 *   Connect to hardware via Driver_USART# = 8       use UART8
 * -------------------------------------------------------------------- */

/* History:
 *  Version 2.00
 *    - Updated to CMSIS Driver API V2.00
 *  Version 1.01
 *    Based on API V1.10 (namespace prefix ARM_ added)
 *  Version 1.00
 *    Initial release
 */ 
#include "USART_STM32F4xx.h"

#include "Driver_USART.h"

#define ARM_USART_DRV_VERSION ARM_DRIVER_VERSION_MAJOR_MINOR(2,00)

// Driver Version
static const ARM_DRIVER_VERSION usart_driver_version = { ARM_USART_API_VERSION, ARM_USART_DRV_VERSION };

// USART1
#ifdef MX_USART1

// USART1 Run-Time Information
static USART_INFO          USART1_Info = {0};
static USART_TRANSFER_INFO USART1_TransferInfo = {0};

#ifdef MX_USART1_TX_Pin
  static USART_PIN USART1_tx = {MX_USART1_TX_GPIOx,  MX_USART1_TX_GPIO_Pin,  MX_USART1_TX_GPIO_AF};
#endif
#ifdef MX_USART1_RX_Pin
  static USART_PIN USART1_rx = {MX_USART1_RX_GPIOx,  MX_USART1_RX_GPIO_Pin,  MX_USART1_RX_GPIO_AF};
#endif
#ifdef MX_USART1_CK_Pin
  static USART_PIN USART1_ck = {MX_USART1_CK_GPIOx,  MX_USART1_CK_GPIO_Pin,  MX_USART1_CK_GPIO_AF};
#endif
#ifdef MX_USART1_RTS_Pin
  static USART_PIN USART1_rts = {MX_USART1_RTS_GPIOx, MX_USART1_RTS_GPIO_Pin, MX_USART1_RTS_GPIO_AF};
#endif
#ifdef MX_USART1_CTS_Pin
  static USART_PIN USART1_cts = {MX_USART1_CTS_GPIOx, MX_USART1_CTS_GPIO_Pin, MX_USART1_CTS_GPIO_AF};
#endif

#ifdef MX_USART1_TX_DMA_Instance
  void USART1_TX_DMA_Complete (DMA_HandleTypeDef *hdma);

  static DMA_HandleTypeDef hdma_usart1_tx;
  static USART_DMA USART1_DMA_Tx = {&hdma_usart1_tx, MX_USART1_TX_DMA_Instance, USART1_TX_DMA_Complete, MX_USART1_TX_DMA_Channel, MX_USART1_TX_DMA_Priority, MX_USART1_TX_DMA_IRQn};
#endif
#ifdef MX_USART1_RX_DMA_Instance
  void USART1_RX_DMA_Complete (DMA_HandleTypeDef *hdma);

  static DMA_HandleTypeDef  hdma_usart1_rx;
  static USART_DMA USART1_DMA_Rx = {&hdma_usart1_rx, MX_USART1_RX_DMA_Instance, USART1_RX_DMA_Complete, MX_USART1_RX_DMA_Channel, MX_USART1_RX_DMA_Priority, MX_USART1_RX_DMA_IRQn};
#endif

// USART1 Resources
static USART_RESOURCES USART1_Resources = {
  {     // Capabilities
    1,  // supports UART (Asynchronous) mode
#ifdef MX_USART1_CK_Pin
    1,  // supports Synchronous Master mode
#else
    0,  // supports Synchronous Master mode
#endif
    0,  // supports Synchronous Slave mode
    1,  // supports UART Single-wire mode
    1,  // supports UART IrDA mode
    1,  // supports UART Smart Card mode
    0,  // Smart Card Clock generator
#ifdef MX_USART1_RTS_Pin
    1,  // RTS Flow Control available
#else
    0,  // RTS Flow Control available
#endif
#ifdef MX_USART1_CTS_Pin
    1,  // CTS Flow Control available
#else
    0,  // CTS Flow Control available
#endif
    1,  // Transmit completed event: \ref ARM_USART_EVENT_TX_COMPLETE
    0,  // Signal receive character timeout event: \ref ARM_USART_EVENT_RX_TIMEOUT
#ifdef MX_USART1_RTS_Pin
    1,  // RTS Line: 0=not available, 1=available
#else
    0,  // RTS Line: 0=not available, 1=available
#endif
#ifdef MX_USART1_CTS_Pin
    1,  // CTS Line: 0=not available, 1=available
#else
    0,  // CTS Line: 0=not available, 1=available
#endif
    0,  // DTR Line: 0=not available, 1=available
    0,  // DSR Line: 0=not available, 1=available
    0,  // DCD Line: 0=not available, 1=available
    0,  // RI Line: 0=not available, 1=available
    0,  // Signal CTS change event: \ref ARM_USART_EVENT_CTS
    0,  // Signal DSR change event: \ref ARM_USART_EVENT_DSR
    0,  // Signal DCD change event: \ref ARM_USART_EVENT_DCD
    0,  // Signal RI change event: \ref ARM_USART_EVENT_RI
  },

    USART1,
    HAL_RCC_GetPCLK2Freq,

  // PINS
  {
#ifdef MX_USART1_TX_Pin
    &USART1_tx,
#else
    NULL,
#endif
#ifdef MX_USART1_RX_Pin
    &USART1_rx,
#else
    NULL,
#endif
#ifdef MX_USART1_CK_Pin
    &USART1_ck,
#else
    NULL,
#endif
#ifdef MX_USART1_RTS_Pin 
    &USART1_rts,
#else
    NULL,
#endif
#ifdef MX_USART1_CTS_Pin 
    &USART1_cts,
#else
    NULL,
#endif
  },

    USART1_IRQn,

#ifdef MX_USART1_TX_DMA_Instance
  &USART1_DMA_Tx,
#else
  NULL,
#endif
#ifdef MX_USART1_RX_DMA_Instance
  &USART1_DMA_Rx,
#else
  NULL,
#endif

  &USART1_Info,
  &USART1_TransferInfo
};
#endif

// USART2
#ifdef MX_USART2

// USART2 Run-Time Information
static USART_INFO          USART2_Info = {0};
static USART_TRANSFER_INFO USART2_TransferInfo = {0};

#ifdef MX_USART2_TX_Pin
  static USART_PIN USART2_tx = {MX_USART2_TX_GPIOx,  MX_USART2_TX_GPIO_Pin,  MX_USART2_TX_GPIO_AF};
#endif
#ifdef MX_USART2_RX_Pin
  static USART_PIN USART2_rx = {MX_USART2_RX_GPIOx,  MX_USART2_RX_GPIO_Pin,  MX_USART2_RX_GPIO_AF};
#endif
#ifdef MX_USART2_CK_Pin
  static USART_PIN USART2_ck = {MX_USART2_CK_GPIOx,  MX_USART2_CK_GPIO_Pin,  MX_USART2_CK_GPIO_AF};
#endif
#ifdef MX_USART2_RTS_Pin
  static USART_PIN USART2_rts = {MX_USART2_RTS_GPIOx, MX_USART2_RTS_GPIO_Pin, MX_USART2_RTS_GPIO_AF};
#endif
#ifdef MX_USART2_CTS_Pin
  static USART_PIN USART2_cts = {MX_USART2_CTS_GPIOx, MX_USART2_CTS_GPIO_Pin, MX_USART2_CTS_GPIO_AF};
#endif

#ifdef MX_USART2_TX_DMA_Instance
  void USART2_TX_DMA_Complete (DMA_HandleTypeDef *hdma);

  static DMA_HandleTypeDef hdma_usart2_tx;
  static USART_DMA USART2_DMA_Tx = {&hdma_usart2_tx, MX_USART2_TX_DMA_Instance, USART2_TX_DMA_Complete, MX_USART2_TX_DMA_Channel, MX_USART2_TX_DMA_Priority, MX_USART2_TX_DMA_IRQn};
#endif
#ifdef MX_USART2_RX_DMA_Instance
  void USART2_RX_DMA_Complete (DMA_HandleTypeDef *hdma);

  static DMA_HandleTypeDef  hdma_usart2_rx;
  static USART_DMA USART2_DMA_Rx = {&hdma_usart2_rx, MX_USART2_RX_DMA_Instance, USART2_RX_DMA_Complete, MX_USART2_RX_DMA_Channel, MX_USART2_RX_DMA_Priority, MX_USART2_RX_DMA_IRQn};
#endif

// USART2 Resources
static USART_RESOURCES USART2_Resources = {
  {     // Capabilities
    1,  // supports UART (Asynchronous) mode
#ifdef MX_USART2_CK_Pin
    1,  // supports Synchronous Master mode
#else
    0,  // supports Synchronous Master mode
#endif
    0,  // supports Synchronous Slave mode
    1,  // supports UART Single-wire mode
    1,  // supports UART IrDA mode
    1,  // supports UART Smart Card mode
    0,  // Smart Card Clock generator
#ifdef MX_USART2_RTS_Pin
    1,  // RTS Flow Control available
#else
    0,  // RTS Flow Control available
#endif
#ifdef MX_USART2_CTS_Pin
    1,  // CTS Flow Control available
#else
    0,  // CTS Flow Control available
#endif
    1,  // Transmit completed event: \ref ARM_USART_EVENT_TX_COMPLETE
    0,  // Signal receive character timeout event: \ref ARM_USART_EVENT_RX_TIMEOUT
#ifdef MX_USART2_RTS_Pin
    1,  // RTS Line: 0=not available, 1=available
#else
    0,  // RTS Line: 0=not available, 1=available
#endif
#ifdef MX_USART2_CTS_Pin
    1,  // CTS Line: 0=not available, 1=available
#else
    0,  // CTS Line: 0=not available, 1=available
#endif
    0,  // DTR Line: 0=not available, 1=available
    0,  // DSR Line: 0=not available, 1=available
    0,  // DCD Line: 0=not available, 1=available
    0,  // RI Line: 0=not available, 1=available
    0,  // Signal CTS change event: \ref ARM_USART_EVENT_CTS
    0,  // Signal DSR change event: \ref ARM_USART_EVENT_DSR
    0,  // Signal DCD change event: \ref ARM_USART_EVENT_DCD
    0,  // Signal RI change event: \ref ARM_USART_EVENT_RI
  },

    USART2,
    HAL_RCC_GetPCLK1Freq,

  // PINS
  {
#ifdef MX_USART2_TX_Pin
    &USART2_tx,
#else
    NULL,
#endif
#ifdef MX_USART2_RX_Pin
    &USART2_rx,
#else
    NULL,
#endif
#ifdef MX_USART2_CK_Pin
    &USART2_ck,
#else
    NULL,
#endif
#ifdef MX_USART2_RTS_Pin 
    &USART2_rts,
#else
    NULL,
#endif
#ifdef MX_USART2_CTS_Pin 
    &USART2_cts,
#else
    NULL,
#endif
  },

    USART2_IRQn,

#ifdef MX_USART2_TX_DMA_Instance
  &USART2_DMA_Tx,
#else
  NULL,
#endif
#ifdef MX_USART2_RX_DMA_Instance
  &USART2_DMA_Rx,
#else
  NULL,
#endif

  &USART2_Info,
  &USART2_TransferInfo
};
#endif

// USART3
#ifdef MX_USART3

// USART3 Run-Time Information
static USART_INFO          USART3_Info = {0};
static USART_TRANSFER_INFO USART3_TransferInfo = {0};

#ifdef MX_USART3_TX_Pin
  static USART_PIN USART3_tx = {MX_USART3_TX_GPIOx,  MX_USART3_TX_GPIO_Pin,  MX_USART3_TX_GPIO_AF};
#endif
#ifdef MX_USART3_RX_Pin
  static USART_PIN USART3_rx = {MX_USART3_RX_GPIOx,  MX_USART3_RX_GPIO_Pin,  MX_USART3_RX_GPIO_AF};
#endif
#ifdef MX_USART3_CK_Pin
  static USART_PIN USART3_ck = {MX_USART3_CK_GPIOx,  MX_USART3_CK_GPIO_Pin,  MX_USART3_CK_GPIO_AF};
#endif
#ifdef MX_USART3_RTS_Pin
  static USART_PIN USART3_rts = {MX_USART3_RTS_GPIOx, MX_USART3_RTS_GPIO_Pin, MX_USART3_RTS_GPIO_AF};
#endif
#ifdef MX_USART3_CTS_Pin
  static USART_PIN USART3_cts = {MX_USART3_CTS_GPIOx, MX_USART3_CTS_GPIO_Pin, MX_USART3_CTS_GPIO_AF};
#endif

#ifdef MX_USART3_TX_DMA_Instance
  void USART3_TX_DMA_Complete (DMA_HandleTypeDef *hdma);

  static DMA_HandleTypeDef hdma_usart3_tx;
  static USART_DMA USART3_DMA_Tx = {&hdma_usart3_tx, MX_USART3_TX_DMA_Instance, USART3_TX_DMA_Complete, MX_USART3_TX_DMA_Channel, MX_USART3_TX_DMA_Priority, MX_USART3_TX_DMA_IRQn};
#endif
#ifdef MX_USART3_RX_DMA_Instance
  void USART3_RX_DMA_Complete (DMA_HandleTypeDef *hdma);

  static DMA_HandleTypeDef  hdma_usart3_rx;
  static USART_DMA USART3_DMA_Rx = {&hdma_usart3_rx, MX_USART3_RX_DMA_Instance, USART3_RX_DMA_Complete, MX_USART3_RX_DMA_Channel, MX_USART3_RX_DMA_Priority, MX_USART3_RX_DMA_IRQn};
#endif

// USART3 Resources
static USART_RESOURCES USART3_Resources = {
  {     // Capabilities
    1,  // supports UART (Asynchronous) mode
#ifdef MX_USART3_CK_Pin
    1,  // supports Synchronous Master mode
#else
    0,  // supports Synchronous Master mode
#endif
    0,  // supports Synchronous Slave mode
    1,  // supports UART Single-wire mode
    1,  // supports UART IrDA mode
    1,  // supports UART Smart Card mode
    0,  // Smart Card Clock generator
#ifdef MX_USART3_RTS_Pin
    1,  // RTS Flow Control available
#else
    0,  // RTS Flow Control available
#endif
#ifdef MX_USART3_CTS_Pin
    1,  // CTS Flow Control available
#else
    0,  // CTS Flow Control available
#endif
    1,  // Transmit completed event: \ref ARM_USART_EVENT_TX_COMPLETE
    0,  // Signal receive character timeout event: \ref ARM_USART_EVENT_RX_TIMEOUT
#ifdef MX_USART3_RTS_Pin
    1,  // RTS Line: 0=not available, 1=available
#else
    0,  // RTS Line: 0=not available, 1=available
#endif
#ifdef MX_USART3_CTS_Pin
    1,  // CTS Line: 0=not available, 1=available
#else
    0,  // CTS Line: 0=not available, 1=available
#endif
    0,  // DTR Line: 0=not available, 1=available
    0,  // DSR Line: 0=not available, 1=available
    0,  // DCD Line: 0=not available, 1=available
    0,  // RI Line: 0=not available, 1=available
    0,  // Signal CTS change event: \ref ARM_USART_EVENT_CTS
    0,  // Signal DSR change event: \ref ARM_USART_EVENT_DSR
    0,  // Signal DCD change event: \ref ARM_USART_EVENT_DCD
    0,  // Signal RI change event: \ref ARM_USART_EVENT_RI
  },

    USART3,
    HAL_RCC_GetPCLK1Freq,

  // PINS
  {
#ifdef MX_USART3_TX_Pin
    &USART3_tx,
#else
    NULL,
#endif
#ifdef MX_USART3_RX_Pin
    &USART3_rx,
#else
    NULL,
#endif
#ifdef MX_USART3_CK_Pin
    &USART3_ck,
#else
    NULL,
#endif
#ifdef MX_USART3_RTS_Pin 
    &USART3_rts,
#else
    NULL,
#endif
#ifdef MX_USART3_CTS_Pin 
    &USART3_cts,
#else
    NULL,
#endif
  },

    USART3_IRQn,

#ifdef MX_USART3_TX_DMA_Instance
  &USART3_DMA_Tx,
#else
  NULL,
#endif
#ifdef MX_USART3_RX_DMA_Instance
  &USART3_DMA_Rx,
#else
  NULL,
#endif

  &USART3_Info,
  &USART3_TransferInfo
};
#endif

// UART4
#ifdef MX_UART4

// UART4 Run-Time Information
static USART_INFO          UART4_Info = {0};
static USART_TRANSFER_INFO UART4_TransferInfo = {0};

#ifdef MX_UART4_TX_Pin
  static USART_PIN UART4_tx = {MX_UART4_TX_GPIOx,  MX_UART4_TX_GPIO_Pin,  MX_UART4_TX_GPIO_AF};
#endif
#ifdef MX_UART4_RX_Pin
  static USART_PIN UART4_rx = {MX_UART4_RX_GPIOx,  MX_UART4_RX_GPIO_Pin,  MX_UART4_RX_GPIO_AF};
#endif

#ifdef MX_UART4_TX_DMA_Instance
  void UART4_TX_DMA_Complete (DMA_HandleTypeDef *hdma);

  static DMA_HandleTypeDef hdma_uart4_tx;
  static USART_DMA UART4_DMA_Tx = {&hdma_uart4_tx, MX_UART4_TX_DMA_Instance, UART4_TX_DMA_Complete, MX_UART4_TX_DMA_Channel, MX_UART4_TX_DMA_Priority, MX_UART4_TX_DMA_IRQn};
#endif
#ifdef MX_UART4_RX_DMA_Instance
  void UART4_RX_DMA_Complete (DMA_HandleTypeDef *hdma);

  static DMA_HandleTypeDef  hdma_uart4_rx;
  static USART_DMA UART4_DMA_Rx = {&hdma_uart4_rx, MX_UART4_RX_DMA_Instance, UART4_RX_DMA_Complete, MX_UART4_RX_DMA_Channel, MX_UART4_RX_DMA_Priority, MX_UART4_RX_DMA_IRQn};
#endif

// UART4 Resources
static USART_RESOURCES USART4_Resources = {
  {     // Capabilities
    1,  // supports UART (Asynchronous) mode
    0,  // supports Synchronous Master mode
    0,  // supports Synchronous Slave mode
    1,  // supports UART Single-wire mode
    1,  // supports UART IrDA mode
    0,  // supports UART Smart Card mode
    0,  // Smart Card Clock generator
    0,  // RTS Flow Control available
    0,  // CTS Flow Control available
    1,  // Transmit completed event: \ref ARM_USART_EVENT_TX_COMPLETE
    0,  // Signal receive character timeout event: \ref ARM_USART_EVENT_RX_TIMEOUT
    0,  // RTS Line: 0=not available, 1=available
    0,  // CTS Line: 0=not available, 1=available
    0,  // DTR Line: 0=not available, 1=available
    0,  // DSR Line: 0=not available, 1=available
    0,  // DCD Line: 0=not available, 1=available
    0,  // RI Line: 0=not available, 1=available
    0,  // Signal CTS change event: \ref ARM_USART_EVENT_CTS
    0,  // Signal DSR change event: \ref ARM_USART_EVENT_DSR
    0,  // Signal DCD change event: \ref ARM_USART_EVENT_DCD
    0,  // Signal RI change event: \ref ARM_USART_EVENT_RI
  },

    UART4,
    HAL_RCC_GetPCLK1Freq,

  // PINS
  {
#ifdef MX_UART4_TX_Pin
    &UART4_tx,
#else
    NULL,
#endif
#ifdef MX_UART4_RX_Pin
    &UART4_rx,
#else
    NULL,
#endif
    NULL,
    NULL,
    NULL,
  },

    UART4_IRQn,

#ifdef MX_UART4_TX_DMA_Instance
  &UART4_DMA_Tx,
#else
  NULL,
#endif
#ifdef MX_UART4_RX_DMA_Instance
  &UART4_DMA_Rx,
#else
  NULL,
#endif

  &UART4_Info,
  &UART4_TransferInfo
};
#endif

// UART5
#ifdef MX_UART5

// UART5 Run-Time Information
static USART_INFO          UART5_Info = {0};
static USART_TRANSFER_INFO UART5_TransferInfo = {0};

#ifdef MX_UART5_TX_Pin
  static USART_PIN UART5_tx = {MX_UART5_TX_GPIOx,  MX_UART5_TX_GPIO_Pin,  MX_UART5_TX_GPIO_AF};
#endif
#ifdef MX_UART5_RX_Pin
  static USART_PIN UART5_rx = {MX_UART5_RX_GPIOx,  MX_UART5_RX_GPIO_Pin,  MX_UART5_RX_GPIO_AF};
#endif

#ifdef MX_UART5_TX_DMA_Instance
  void UART5_TX_DMA_Complete (DMA_HandleTypeDef *hdma);

  static DMA_HandleTypeDef hdma_uart5_tx;
  static USART_DMA UART5_DMA_Tx = {&hdma_uart5_tx, MX_UART5_TX_DMA_Instance, UART5_TX_DMA_Complete, MX_UART5_TX_DMA_Channel, MX_UART5_TX_DMA_Priority, MX_UART5_TX_DMA_IRQn};
#endif
#ifdef MX_UART5_RX_DMA_Instance
  void UART5_RX_DMA_Complete (DMA_HandleTypeDef *hdma);

  static DMA_HandleTypeDef  hdma_uart5_rx;
  static USART_DMA UART5_DMA_Rx = {&hdma_uart5_rx, MX_UART5_RX_DMA_Instance, UART5_RX_DMA_Complete, MX_UART5_RX_DMA_Channel, MX_UART5_RX_DMA_Priority, MX_UART5_RX_DMA_IRQn};
#endif

// UART5 Resources
static USART_RESOURCES USART5_Resources = {
  {     // Capabilities
    1,  // supports UART (Asynchronous) mode
    0,  // supports Synchronous Master mode
    0,  // supports Synchronous Slave mode
    1,  // supports UART Single-wire mode
    1,  // supports UART IrDA mode
    0,  // supports UART Smart Card mode
    0,  // Smart Card Clock generator
    0,  // RTS Flow Control available
    0,  // CTS Flow Control available
    1,  // Transmit completed event: \ref ARM_USART_EVENT_TX_COMPLETE
    0,  // Signal receive character timeout event: \ref ARM_USART_EVENT_RX_TIMEOUT
    0,  // RTS Line: 0=not available, 1=available
    0,  // CTS Line: 0=not available, 1=available
    0,  // DTR Line: 0=not available, 1=available
    0,  // DSR Line: 0=not available, 1=available
    0,  // DCD Line: 0=not available, 1=available
    0,  // RI Line: 0=not available, 1=available
    0,  // Signal CTS change event: \ref ARM_USART_EVENT_CTS
    0,  // Signal DSR change event: \ref ARM_USART_EVENT_DSR
    0,  // Signal DCD change event: \ref ARM_USART_EVENT_DCD
    0,  // Signal RI change event: \ref ARM_USART_EVENT_RI
  },

    UART5,
    HAL_RCC_GetPCLK1Freq,

  // PINS
  {
#ifdef MX_UART5_TX_Pin
    &UART5_tx,
#else
    NULL,
#endif
#ifdef MX_UART5_RX_Pin
    &UART5_rx,
#else
    NULL,
#endif
    NULL,
    NULL,
    NULL,
  },

    UART5_IRQn,

#ifdef MX_UART5_TX_DMA_Instance
  &UART5_DMA_Tx,
#else
  NULL,
#endif
#ifdef MX_UART5_RX_DMA_Instance
  &UART5_DMA_Rx,
#else
  NULL,
#endif

  &UART5_Info,
  &UART5_TransferInfo
};
#endif

// USART6
#ifdef MX_USART6

// USART6 Run-Time Information
static USART_INFO          USART6_Info = {0};
static USART_TRANSFER_INFO USART6_TransferInfo = {0};

#ifdef MX_USART6_TX_Pin
  static USART_PIN USART6_tx = {MX_USART6_TX_GPIOx,  MX_USART6_TX_GPIO_Pin,  MX_USART6_TX_GPIO_AF};
#endif
#ifdef MX_USART6_RX_Pin
  static USART_PIN USART6_rx = {MX_USART6_RX_GPIOx,  MX_USART6_RX_GPIO_Pin,  MX_USART6_RX_GPIO_AF};
#endif
#ifdef MX_USART6_CK_Pin
  static USART_PIN USART6_ck = {MX_USART6_CK_GPIOx,  MX_USART6_CK_GPIO_Pin,  MX_USART6_CK_GPIO_AF};
#endif
#ifdef MX_USART6_RTS_Pin
  static USART_PIN USART6_rts = {MX_USART6_RTS_GPIOx, MX_USART6_RTS_GPIO_Pin, MX_USART6_RTS_GPIO_AF};
#endif
#ifdef MX_USART6_CTS_Pin
  static USART_PIN USART6_cts = {MX_USART6_CTS_GPIOx, MX_USART6_CTS_GPIO_Pin, MX_USART6_CTS_GPIO_AF};
#endif

#ifdef MX_USART6_TX_DMA_Instance
  void USART6_TX_DMA_Complete (DMA_HandleTypeDef *hdma);

  static DMA_HandleTypeDef hdma_usart6_tx;
  static USART_DMA USART6_DMA_Tx = {&hdma_usart6_tx, MX_USART6_TX_DMA_Instance, USART6_TX_DMA_Complete, MX_USART6_TX_DMA_Channel, MX_USART6_TX_DMA_Priority, MX_USART6_TX_DMA_IRQn};
#endif
#ifdef MX_USART6_RX_DMA_Instance
  void USART6_RX_DMA_Complete (DMA_HandleTypeDef *hdma);

  static DMA_HandleTypeDef  hdma_usart6_rx;
  static USART_DMA USART6_DMA_Rx = {&hdma_usart6_rx, MX_USART6_RX_DMA_Instance, USART6_RX_DMA_Complete, MX_USART6_RX_DMA_Channel, MX_USART6_RX_DMA_Priority, MX_USART6_RX_DMA_IRQn};
#endif

// USART6 Resources
static USART_RESOURCES USART6_Resources = {
  {     // Capabilities
    1,  // supports UART (Asynchronous) mode
#ifdef MX_USART6_CK_Pin
    1,  // supports Synchronous Master mode
#else
    0,  // supports Synchronous Master mode
#endif
    0,  // supports Synchronous Slave mode
    1,  // supports UART Single-wire mode
    1,  // supports UART IrDA mode
    1,  // supports UART Smart Card mode
    0,  // Smart Card Clock generator
#ifdef MX_USART6_RTS_Pin
    1,  // RTS Flow Control available
#else
    0,  // RTS Flow Control available
#endif
#ifdef MX_USART6_CTS_Pin
    1,  // CTS Flow Control available
#else
    0,  // CTS Flow Control available
#endif
    1,  // Transmit completed event: \ref ARM_USART_EVENT_TX_COMPLETE
    0,  // Signal receive character timeout event: \ref ARM_USART_EVENT_RX_TIMEOUT
#ifdef MX_USART6_RTS_Pin
    1,  // RTS Line: 0=not available, 1=available
#else
    0,  // RTS Line: 0=not available, 1=available
#endif
#ifdef MX_USART6_CTS_Pin
    1,  // CTS Line: 0=not available, 1=available
#else
    0,  // CTS Line: 0=not available, 1=available
#endif
    0,  // DTR Line: 0=not available, 1=available
    0,  // DSR Line: 0=not available, 1=available
    0,  // DCD Line: 0=not available, 1=available
    0,  // RI Line: 0=not available, 1=available
    0,  // Signal CTS change event: \ref ARM_USART_EVENT_CTS
    0,  // Signal DSR change event: \ref ARM_USART_EVENT_DSR
    0,  // Signal DCD change event: \ref ARM_USART_EVENT_DCD
    0,  // Signal RI change event: \ref ARM_USART_EVENT_RI
  },

    USART6,
    HAL_RCC_GetPCLK2Freq,

  // PINS
  {
#ifdef MX_USART6_TX_Pin
    &USART6_tx,
#else
    NULL,
#endif
#ifdef MX_USART6_RX_Pin
    &USART6_rx,
#else
    NULL,
#endif
#ifdef MX_USART6_CK_Pin
    &USART6_ck,
#else
    NULL,
#endif
#ifdef MX_USART6_RTS_Pin 
    &USART6_rts,
#else
    NULL,
#endif
#ifdef MX_USART6_CTS_Pin 
    &USART6_cts,
#else
    NULL,
#endif
  },

    USART6_IRQn,

#ifdef MX_USART6_TX_DMA_Instance
  &USART6_DMA_Tx,
#else
  NULL,
#endif
#ifdef MX_USART6_RX_DMA_Instance
  &USART6_DMA_Rx,
#else
  NULL,
#endif

  &USART6_Info,
  &USART6_TransferInfo
};
#endif

// UART7
#ifdef MX_UART7

// UART7 Run-Time Information
static USART_INFO          UART7_Info = {0};
static USART_TRANSFER_INFO UART7_TransferInfo = {0};

#ifdef MX_UART7_TX_Pin
  static USART_PIN UART7_tx = {MX_UART7_TX_GPIOx,  MX_UART7_TX_GPIO_Pin,  MX_UART7_TX_GPIO_AF};
#endif
#ifdef MX_UART7_RX_Pin
  static USART_PIN UART7_rx = {MX_UART7_RX_GPIOx,  MX_UART7_RX_GPIO_Pin,  MX_UART7_RX_GPIO_AF};
#endif

#ifdef MX_UART7_TX_DMA_Instance
  void UART7_TX_DMA_Complete (DMA_HandleTypeDef *hdma);

  static DMA_HandleTypeDef hdma_uart7_tx;
  static USART_DMA UART7_DMA_Tx = {&hdma_uart7_tx, MX_UART7_TX_DMA_Instance, UART7_TX_DMA_Complete, MX_UART7_TX_DMA_Channel, MX_UART7_TX_DMA_Priority, MX_UART7_TX_DMA_IRQn};
#endif
#ifdef MX_UART7_RX_DMA_Instance
  void UART7_RX_DMA_Complete (DMA_HandleTypeDef *hdma);

  static DMA_HandleTypeDef  hdma_uart7_rx;
  static USART_DMA UART7_DMA_Rx = {&hdma_uart7_rx, MX_UART7_RX_DMA_Instance, UART7_RX_DMA_Complete, MX_UART7_RX_DMA_Channel, MX_UART7_RX_DMA_Priority, MX_UART7_RX_DMA_IRQn};
#endif

// UART7 Resources
static USART_RESOURCES USART7_Resources = {
  {     // Capabilities
    1,  // supports UART (Asynchronous) mode
    0,  // supports Synchronous Master mode
    0,  // supports Synchronous Slave mode
    1,  // supports UART Single-wire mode
    1,  // supports UART IrDA mode
    0,  // supports UART Smart Card mode
    0,  // Smart Card Clock generator
    0,  // RTS Flow Control available
    0,  // CTS Flow Control available
    1,  // Transmit completed event: \ref ARM_USART_EVENT_TX_COMPLETE
    0,  // Signal receive character timeout event: \ref ARM_USART_EVENT_RX_TIMEOUT
    0,  // RTS Line: 0=not available, 1=available
    0,  // CTS Line: 0=not available, 1=available
    0,  // DTR Line: 0=not available, 1=available
    0,  // DSR Line: 0=not available, 1=available
    0,  // DCD Line: 0=not available, 1=available
    0,  // RI Line: 0=not available, 1=available
    0,  // Signal CTS change event: \ref ARM_USART_EVENT_CTS
    0,  // Signal DSR change event: \ref ARM_USART_EVENT_DSR
    0,  // Signal DCD change event: \ref ARM_USART_EVENT_DCD
    0,  // Signal RI change event: \ref ARM_USART_EVENT_RI
  },

    UART7,
    HAL_RCC_GetPCLK1Freq,

  // PINS
  {
#ifdef MX_UART7_TX_Pin
    &UART7_tx,
#else
    NULL,
#endif
#ifdef MX_UART7_RX_Pin
    &UART7_rx,
#else
    NULL,
#endif
    NULL,
    NULL,
    NULL,
  },

    UART7_IRQn,

#ifdef MX_UART7_TX_DMA_Instance
  &UART7_DMA_Tx,
#else
  NULL,
#endif
#ifdef MX_UART7_RX_DMA_Instance
  &UART7_DMA_Rx,
#else
  NULL,
#endif

  &UART7_Info,
  &UART7_TransferInfo
};
#endif

// UART8
#ifdef MX_UART8

// UART8 Run-Time Information
static USART_INFO          UART8_Info = {0};
static USART_TRANSFER_INFO UART8_TransferInfo = {0};

#ifdef MX_UART8_TX_Pin
  static USART_PIN UART8_tx = {MX_UART8_TX_GPIOx,  MX_UART8_TX_GPIO_Pin,  MX_UART8_TX_GPIO_AF};
#endif
#ifdef MX_UART8_RX_Pin
  static USART_PIN UART8_rx = {MX_UART8_RX_GPIOx,  MX_UART8_RX_GPIO_Pin,  MX_UART8_RX_GPIO_AF};
#endif

#ifdef MX_UART8_TX_DMA_Instance
  void UART8_TX_DMA_Complete (DMA_HandleTypeDef *hdma);

  static DMA_HandleTypeDef hdma_uart8_tx;
  static USART_DMA UART8_DMA_Tx = {&hdma_uart8_tx, MX_UART8_TX_DMA_Instance, UART8_TX_DMA_Complete, MX_UART8_TX_DMA_Channel, MX_UART8_TX_DMA_Priority, MX_UART8_TX_DMA_IRQn};
#endif
#ifdef MX_UART8_RX_DMA_Instance
  void UART8_RX_DMA_Complete (DMA_HandleTypeDef *hdma);

  static DMA_HandleTypeDef  hdma_uart8_rx;
  static USART_DMA UART8_DMA_Rx = {&hdma_uart8_rx, MX_UART8_RX_DMA_Instance, UART8_RX_DMA_Complete, MX_UART8_RX_DMA_Channel, MX_UART8_RX_DMA_Priority, MX_UART8_RX_DMA_IRQn};
#endif

// UART8 Resources
static USART_RESOURCES USART8_Resources = {
  {     // Capabilities
    1,  // supports UART (Asynchronous) mode
    0,  // supports Synchronous Master mode
    0,  // supports Synchronous Slave mode
    1,  // supports UART Single-wire mode
    1,  // supports UART IrDA mode
    0,  // supports UART Smart Card mode
    0,  // Smart Card Clock generator
    0,  // RTS Flow Control available
    0,  // CTS Flow Control available
    1,  // Transmit completed event: \ref ARM_USART_EVENT_TX_COMPLETE
    0,  // Signal receive character timeout event: \ref ARM_USART_EVENT_RX_TIMEOUT
    0,  // RTS Line: 0=not available, 1=available
    0,  // CTS Line: 0=not available, 1=available
    0,  // DTR Line: 0=not available, 1=available
    0,  // DSR Line: 0=not available, 1=available
    0,  // DCD Line: 0=not available, 1=available
    0,  // RI Line: 0=not available, 1=available
    0,  // Signal CTS change event: \ref ARM_USART_EVENT_CTS
    0,  // Signal DSR change event: \ref ARM_USART_EVENT_DSR
    0,  // Signal DCD change event: \ref ARM_USART_EVENT_DCD
    0,  // Signal RI change event: \ref ARM_USART_EVENT_RI
  },

    UART8,
    HAL_RCC_GetPCLK1Freq,

  // PINS
  {
#ifdef MX_UART8_TX_Pin
    &UART8_tx,
#else
    NULL,
#endif
#ifdef MX_UART8_RX_Pin
    &UART8_rx,
#else
    NULL,
#endif
    NULL,
    NULL,
    NULL,
  },

    UART8_IRQn,

#ifdef MX_UART8_TX_DMA_Instance
  &UART8_DMA_Tx,
#else
  NULL,
#endif
#ifdef MX_UART8_RX_DMA_Instance
  &UART8_DMA_Rx,
#else
  NULL,
#endif

  &UART8_Info,
  &UART8_TransferInfo
};
#endif

// Function prototypes
static int32_t USART_Receive (void            *data,
                              uint32_t         num,
                              USART_RESOURCES *usart);

/**
  \fn          void Enable_GPIO_Clock (GPIO_TypeDef *port)
  \brief       Enable GPIO clock
*/
static void Enable_GPIO_Clock (GPIO_TypeDef *GPIOx) {
#ifdef GPIOA
  if (GPIOx == GPIOA) __GPIOA_CLK_ENABLE();
#endif 
#ifdef GPIOB
  if (GPIOx == GPIOB) __GPIOB_CLK_ENABLE();
#endif
#ifdef GPIOC
  if (GPIOx == GPIOC) __GPIOC_CLK_ENABLE();
#endif
#ifdef GPIOD
  if (GPIOx == GPIOD) __GPIOD_CLK_ENABLE();
#endif
#ifdef GPIOE
  if (GPIOx == GPIOE) __GPIOE_CLK_ENABLE();
#endif
#ifdef GPIOF
  if (GPIOx == GPIOF) __GPIOF_CLK_ENABLE();
#endif
#ifdef GPIOG
  if (GPIOx == GPIOG) __GPIOG_CLK_ENABLE();
#endif
#ifdef GPIOH
  if (GPIOx == GPIOH) __GPIOH_CLK_ENABLE();
#endif
#ifdef GPIOI
  if (GPIOx == GPIOI) __GPIOI_CLK_ENABLE();
#endif
}


// USART Driver functions

/**
  \fn          ARM_DRIVER_VERSION USARTx_GetVersion (void)
  \brief       Get driver version.
  \return      \ref ARM_DRIVER_VERSION
*/
static ARM_DRIVER_VERSION USARTx_GetVersion (void) {
  return usart_driver_version;
}

/**
  \fn          ARM_USART_CAPABILITIES USART_GetCapabilities (USART_RESOURCES *usart)
  \brief       Get driver capabilities
  \param[in]   usart     Pointer to USART resources
  \return      \ref ARM_USART_CAPABILITIES
*/
static ARM_USART_CAPABILITIES USART_GetCapabilities (USART_RESOURCES *usart) {
  return usart->capabilities;
}

/**
  \fn          int32_t USART_Initialize (ARM_USART_SignalEvent_t  cb_event
                                         USART_RESOURCES         *usart)
  \brief       Initialize USART Interface.
  \param[in]   cb_event  Pointer to \ref ARM_USART_SignalEvent
  \param[in]   usart     Pointer to USART resources
  \return      \ref execution_status
*/
static int32_t USART_Initialize (ARM_USART_SignalEvent_t  cb_event,
                                 USART_RESOURCES         *usart) {

  if (usart->info->flags & USART_FLAG_POWERED) {
    // Device is powered - could not be re-initialized
    return ARM_DRIVER_ERROR;
  }

  if (usart->info->flags & USART_FLAG_INITIALIZED) {
    // Driver is already initialized
    return ARM_DRIVER_OK;
  }

  // Initialize USART Run-time Resources
  usart->info->cb_event = cb_event;

  usart->info->status.tx_busy          = 0;
  usart->info->status.rx_busy          = 0;
  usart->info->status.tx_underflow     = 0;
  usart->info->status.rx_overflow      = 0;
  usart->info->status.rx_break         = 0;
  usart->info->status.rx_framing_error = 0;
  usart->info->status.rx_parity_error  = 0;

  usart->info->mode = 0;

  // Clear transfer information
  memset(usart->xfer, 0, sizeof(USART_TRANSFER_INFO));

  // Enable TX pin port clock
  Enable_GPIO_Clock (usart->io.tx->port);

  // Enable RX pin port clock
  Enable_GPIO_Clock (usart->io.rx->port);

  // Enable CLK pin port clock
  if (usart->io.ck) {
    Enable_GPIO_Clock (usart->io.ck->port);
  }

  // Configure RTS pin
  if (usart->io.rts) {
    Enable_GPIO_Clock (usart->io.rts->port);
  }

  // Configure CTS pin
  if (usart->io.cts) {
    Enable_GPIO_Clock (usart->io.cts->port);
  }

#ifdef __USART_DMA
  if (usart->dma_rx) {
    // Initialize USART RX DMA Resources
    usart->dma_rx->hdma->Instance             = usart->dma_rx->stream;
    usart->dma_rx->hdma->Init.Channel         = usart->dma_rx->channel;
    usart->dma_rx->hdma->Init.Direction       = DMA_PERIPH_TO_MEMORY;
    usart->dma_rx->hdma->Init.Mode            = DMA_NORMAL;
    usart->dma_rx->hdma->Init.Priority        = usart->dma_rx->priority;
    usart->dma_rx->hdma->Init.FIFOMode        = DMA_FIFOMODE_DISABLE;
    usart->dma_rx->hdma->Init.FIFOThreshold   = DMA_FIFO_THRESHOLD_FULL;
    usart->dma_rx->hdma->Init.MemBurst        = DMA_MBURST_SINGLE;
    usart->dma_rx->hdma->Init.PeriphBurst     = DMA_PBURST_SINGLE;
    usart->dma_rx->hdma->Parent               = NULL;
    usart->dma_rx->hdma->XferCpltCallback     = usart->dma_rx->cb_complete;
    usart->dma_rx->hdma->XferHalfCpltCallback = NULL;
    usart->dma_rx->hdma->XferM1CpltCallback   = NULL;
    usart->dma_rx->hdma->XferErrorCallback    = NULL;

    // Enable DMA IRQ in NVIC
    HAL_NVIC_EnableIRQ (usart->dma_rx->irq_num);
  }

  if (usart->dma_tx) {
    // Initialize USART TX DMA Resources
    usart->dma_tx->hdma->Instance             = usart->dma_tx->stream;
    usart->dma_tx->hdma->Init.Channel         = usart->dma_tx->channel;
    usart->dma_tx->hdma->Init.Direction       = DMA_MEMORY_TO_PERIPH;
    usart->dma_tx->hdma->Init.Mode            = DMA_NORMAL;
    usart->dma_tx->hdma->Init.Priority        = usart->dma_tx->priority;
    usart->dma_tx->hdma->Init.FIFOMode        = DMA_FIFOMODE_DISABLE;
    usart->dma_tx->hdma->Init.FIFOThreshold   = DMA_FIFO_THRESHOLD_FULL;
    usart->dma_tx->hdma->Init.MemBurst        = DMA_MBURST_SINGLE;
    usart->dma_tx->hdma->Init.PeriphBurst     = DMA_PBURST_SINGLE;
    usart->dma_tx->hdma->Parent               = NULL;
    usart->dma_tx->hdma->XferCpltCallback     = usart->dma_tx->cb_complete;
    usart->dma_tx->hdma->XferHalfCpltCallback = NULL;
    usart->dma_tx->hdma->XferM1CpltCallback   = NULL;
    usart->dma_tx->hdma->XferErrorCallback    = NULL;

    // Enable DMA IRQ in NVIC
    HAL_NVIC_EnableIRQ (usart->dma_tx->irq_num);
  }

  // Enable DMA clock
  if ((usart->reg == USART1) || (usart->reg == USART6)) {
    // DMA2 used for USART1 and USART6
    __DMA2_CLK_ENABLE();
  } else {
    __DMA1_CLK_ENABLE();
  }
#endif

  usart->info->flags = USART_FLAG_INITIALIZED;

  return ARM_DRIVER_OK;
}

/**
  \fn          int32_t USART_Uninitialize (USART_RESOURCES *usart)
  \brief       De-initialize USART Interface.
  \param[in]   usart     Pointer to USART resources
  \return      \ref execution_status
*/
static int32_t USART_Uninitialize (USART_RESOURCES *usart) {

  if (usart->info->flags & USART_FLAG_POWERED) {
    // Driver is powered - could not be uninitialized
    return ARM_DRIVER_ERROR;
  }

  if (usart->info->flags == 0) {
    // Driver not initialized
    return ARM_DRIVER_OK;
  }

  // Unconfigure USART pins
  if (usart->io.tx)  HAL_GPIO_DeInit(usart->io.tx->port,  usart->io.tx->pin);
  if (usart->io.rx)  HAL_GPIO_DeInit(usart->io.rx->port,  usart->io.rx->pin);
  if (usart->io.ck)  HAL_GPIO_DeInit(usart->io.ck->port,  usart->io.ck->pin);
  if (usart->io.rts) HAL_GPIO_DeInit(usart->io.rts->port, usart->io.rts->pin);
  if (usart->io.cts) HAL_GPIO_DeInit(usart->io.cts->port, usart->io.cts->pin);

#ifdef __USART_DMA
  if (usart->dma_rx) {
    // Disable DMA IRQ in NVIC
    HAL_NVIC_DisableIRQ (usart->dma_rx->irq_num);
    // Deinitialize DMA
    HAL_DMA_DeInit (usart->dma_rx->hdma);
  }

  if (usart->dma_tx) {
    // Disable DMA IRQ in NVIC
    HAL_NVIC_DisableIRQ (usart->dma_tx->irq_num);
    // Deinitialize DMA
    HAL_DMA_DeInit (usart->dma_tx->hdma);
  }
#endif

  // Reset USART status flags
  usart->info->flags = 0;

  return ARM_DRIVER_OK;
}

/**
  \fn          int32_t USART_PowerControl (ARM_POWER_STATE state)
  \brief       Control USART Interface Power.
  \param[in]   state  Power state
  \param[in]   usart  Pointer to USART resources
  \return      \ref execution_status
*/
static int32_t USART_PowerControl (ARM_POWER_STATE  state,
                                   USART_RESOURCES *usart) {

  if ((usart->info->flags & USART_FLAG_INITIALIZED) == 0) {
    // Return error, if USART is not initialized
    return ARM_DRIVER_ERROR;
  }

  if (usart->info->status.rx_busy == 1) {
    // Receive busy
    return ARM_DRIVER_ERROR_BUSY;
  }

  if (usart->info->flags & USART_FLAG_SEND_ACTIVE) {
    // Transmit busy
    return ARM_DRIVER_ERROR_BUSY;
  }

  if (usart->info->flags & USART_FLAG_POWERED) {
    if ((usart->reg->SR & USART_SR_TC) == 0) {
      // Transmission is not complete
      return ARM_DRIVER_ERROR_BUSY;
    }
  }

  switch (state) {
    case ARM_POWER_OFF:
      if ((usart->info->flags & USART_FLAG_POWERED) == 0)
        return ARM_DRIVER_OK;

      // Disable USART IRQ
      NVIC_DisableIRQ(usart->irq_num);

      // Disable USART clock
      __USARTx_CLK_DISABLE(usart->reg);

      usart->info->flags = USART_FLAG_INITIALIZED;
      break;

    case ARM_POWER_LOW:
      return ARM_DRIVER_ERROR_UNSUPPORTED;

    case ARM_POWER_FULL:
      if (usart->info->flags & USART_FLAG_POWERED)
        return ARM_DRIVER_OK;

      // Enable USART clock
      __USARTx_CLK_ENABLE(usart->reg);

      // Reset USART control registers
      usart->reg->CR1 = 0;
      usart->reg->CR2 = 0;
      usart->reg->CR3 = 0;

      usart->info->flags = USART_FLAG_POWERED | USART_FLAG_INITIALIZED;

      // Clear and Enable USART IRQ
      NVIC_ClearPendingIRQ(usart->irq_num);
      NVIC_EnableIRQ(usart->irq_num);

      break;

    default: return ARM_DRIVER_ERROR_UNSUPPORTED;
  }
  return ARM_DRIVER_OK;
}

/**
  \fn          int32_t USART_Send (const void            *data,
                                         uint32_t         num,
                                         USART_RESOURCES *usart)
  \brief       Start sending data to USART transmitter.
  \param[in]   data  Pointer to buffer with data to send to USART transmitter
  \param[in]   num   Number of data items to send
  \param[in]   usart Pointer to USART resources
  \return      \ref execution_status
*/
static int32_t USART_Send (const void            *data,
                                 uint32_t         num,
                                 USART_RESOURCES *usart) {
  int32_t stat;

#ifdef __USART_DMA_TX
  uint32_t source_inc = DMA_MINC_ENABLE;
#endif

  if ((data == NULL) || (num == 0)) {
    // Invalid parameters
    return ARM_DRIVER_ERROR_PARAMETER;
  }

  if ((usart->info->flags & USART_FLAG_CONFIGURED) == 0) {
    // USART is not configured (mode not selected)
    return ARM_DRIVER_ERROR;
  }

  if (usart->info->flags & USART_FLAG_SEND_ACTIVE) {
    // Send is not completed yet
    return ARM_DRIVER_ERROR_BUSY;
  }

  // Set Send active flag
  usart->info->flags |= USART_FLAG_SEND_ACTIVE;

  // Save transmit buffer info
  usart->xfer->tx_buf = (uint8_t *)data;
  usart->xfer->tx_num = num;
  usart->xfer->tx_cnt = 0;

  // Synchronous mode
  if (usart->info->mode == ARM_USART_MODE_SYNCHRONOUS_MASTER) {
    if (usart->xfer->sync_mode == 0) {
      usart->xfer->sync_mode = USART_SYNC_MODE_TX;
      // Start dummy reads
      stat = USART_Receive (&usart->xfer->dump_val, num, usart);
      if (stat == ARM_DRIVER_ERROR_BUSY) return ARM_DRIVER_ERROR_BUSY;

#ifdef __USART_DMA_TX
    } else if (usart->xfer->sync_mode == USART_SYNC_MODE_RX) {
      // Dummy DMA writes (do not increment source address)
      source_inc = DMA_MINC_DISABLE;
#endif
    }
  }

#ifdef __USART_DMA_TX
  // DMA mode
  if (usart->dma_tx) {
    // Prepare DMA to send TX data
    usart->dma_tx->hdma->Init.PeriphInc             = DMA_PINC_DISABLE;
    usart->dma_tx->hdma->Init.MemInc                = source_inc;

    if ((usart->reg->CR1 & USART_CR1_M) && ((usart->reg->CR1 & USART_CR1_PCE) == 0)) {
      // 9-bit data frame, no parity
      usart->dma_tx->hdma->Init.PeriphDataAlignment = DMA_PDATAALIGN_HALFWORD;
      usart->dma_tx->hdma->Init.MemDataAlignment    = DMA_PDATAALIGN_HALFWORD;
    } else {
      // 8-bit data frame
      usart->dma_tx->hdma->Init.PeriphDataAlignment = DMA_PDATAALIGN_BYTE;
      usart->dma_tx->hdma->Init.MemDataAlignment    = DMA_PDATAALIGN_BYTE;
    }

    // Initialize and start USART TX DMA Stream
    if (HAL_DMA_Init     (usart->dma_tx->hdma) != HAL_OK) return ARM_DRIVER_ERROR;
    if (HAL_DMA_Start_IT (usart->dma_tx->hdma, (uint32_t)usart->xfer->tx_buf, (uint32_t)(&usart->reg->DR), num) != HAL_OK)
      return ARM_DRIVER_ERROR;

    // DMA Enable transmitter
    usart->reg->CR3 |= USART_CR3_DMAT;
  } else
#endif
  // Interrupt mode
  {
    // TXE interrupt enable
    usart->reg->CR1 |= USART_CR1_TXEIE;
  }

  return ARM_DRIVER_OK;
}

/**
  \fn          int32_t USART_Receive (void            *data,
                                      uint32_t         num,
                                      USART_RESOURCES *usart)
  \brief       Start receiving data from USART receiver.
  \param[out]  data  Pointer to buffer for data to receive from USART receiver
  \param[in]   num   Number of data items to receive
  \param[in]   usart Pointer to USART resources
  \return      \ref execution_status
*/
static int32_t USART_Receive (void            *data,
                              uint32_t         num,
                              USART_RESOURCES *usart) {

  int32_t stat;
#ifdef __USART_DMA_RX
  uint32_t dest_inc = DMA_MINC_ENABLE;
#endif

  if ((data == NULL) || (num == 0)) {
    // Invalid parameters
    return ARM_DRIVER_ERROR_PARAMETER;
  }

  if ((usart->info->flags & USART_FLAG_CONFIGURED) == 0) {
    // USART is not configured (mode not selected)
    return ARM_DRIVER_ERROR;
  }

  // Check if receiver is busy
  if (usart->info->status.rx_busy == 1) 
    return ARM_DRIVER_ERROR_BUSY;

  // Save number of data to be received
  usart->xfer->rx_num = num;

  // Clear RX statuses
  usart->info->status.rx_break          = 0;
  usart->info->status.rx_framing_error  = 0;
  usart->info->status.rx_overflow       = 0;
  usart->info->status.rx_parity_error   = 0;

  // Save receive buffer info
  usart->xfer->rx_buf = (uint8_t *)data;
  usart->xfer->rx_cnt =            0;

  // Set RX busy flag
  usart->info->status.rx_busy = 1;

#ifdef __USART_DMA_RX

  // Synchronous mode
  if (usart->info->mode == ARM_USART_MODE_SYNCHRONOUS_MASTER) {
    if (usart->xfer->sync_mode == USART_SYNC_MODE_TX) {
      // Dummy DMA reads (do not increment destination address)
      dest_inc = DMA_MINC_DISABLE;
    }
  }

  // DMA mode
  if (usart->dma_rx) {
    // Disable RXNE Interrupt
    usart->reg->CR1 &= ~USART_CR1_RXNEIE;

    // Prepare DMA to send RX data
    usart->dma_rx->hdma->Init.PeriphInc             = DMA_PINC_DISABLE;
    usart->dma_rx->hdma->Init.MemInc                = dest_inc;

    if ((usart->reg->CR1 & USART_CR1_M) && ((usart->reg->CR1 & USART_CR1_PCE) == 0)) {
      // 9-bit data frame, no parity
      usart->dma_rx->hdma->Init.PeriphDataAlignment = DMA_PDATAALIGN_HALFWORD;
      usart->dma_rx->hdma->Init.MemDataAlignment    = DMA_PDATAALIGN_HALFWORD;
    } else {
      // 8 - bit data frame
      usart->dma_rx->hdma->Init.PeriphDataAlignment = DMA_PDATAALIGN_BYTE;
      usart->dma_rx->hdma->Init.MemDataAlignment    = DMA_PDATAALIGN_BYTE;
    }

    // Initialize and start USART RX DMA Stream
    if (HAL_DMA_Init     (usart->dma_rx->hdma) != HAL_OK) return ARM_DRIVER_ERROR;
    if (HAL_DMA_Start_IT (usart->dma_rx->hdma, (uint32_t)(&usart->reg->DR), (uint32_t)usart->xfer->rx_buf, num) != HAL_OK)
      return ARM_DRIVER_ERROR;
  }
#endif

  // Synchronous mode
  if (usart->info->mode == ARM_USART_MODE_SYNCHRONOUS_MASTER) {
    if (usart->xfer->sync_mode == 0) {
      usart->xfer->sync_mode = USART_SYNC_MODE_RX;
      // Send dummy data
      stat = USART_Send (&usart->xfer->def_val, num, usart);
      if (stat == ARM_DRIVER_ERROR_BUSY) return ARM_DRIVER_ERROR_BUSY;
    }
  }

  return ARM_DRIVER_OK;
}

/**
  \fn          int32_t USART_Transfer (const void             *data_out,
                                             void             *data_in,
                                             uint32_t          num,
                                             USART_RESOURCES  *usart)
  \brief       Start sending/receiving data to/from USART transmitter/receiver.
  \param[in]   data_out  Pointer to buffer with data to send to USART transmitter
  \param[out]  data_in   Pointer to buffer for data to receive from USART receiver
  \param[in]   num       Number of data items to transfer
  \param[in]   usart     Pointer to USART resources
  \return      \ref execution_status
*/
static int32_t USART_Transfer (const void             *data_out,
                                     void             *data_in,
                                     uint32_t          num,
                                     USART_RESOURCES  *usart) {
  int32_t status;

  if ((data_out == NULL) || (data_in == NULL) || (num == 0)) {
    // Invalid parameters
    return ARM_DRIVER_ERROR_PARAMETER;
  }

  if ((usart->info->flags & USART_FLAG_CONFIGURED) == 0) {
    // USART is not configured
    return ARM_DRIVER_ERROR;
  }

  if (usart->info->mode == ARM_USART_MODE_SYNCHRONOUS_MASTER) {

    // Set xfer mode
    usart->xfer->sync_mode = USART_SYNC_MODE_TX_RX;

    // Receive
    status = USART_Receive (data_in, num, usart);
    if (status != ARM_DRIVER_OK) return status;

    // Send
    status = USART_Send (data_out, num, usart);
    if (status != ARM_DRIVER_OK) return status;

  } else {
    // Only in synchronous mode
    return ARM_DRIVER_ERROR;
  }
  return ARM_DRIVER_OK;
}

/**
  \fn          uint32_t USART_GetTxCount (USART_RESOURCES *usart)
  \brief       Get transmitted data count.
  \param[in]   usart     Pointer to USART resources
  \return      number of data items transmitted
*/
static uint32_t USART_GetTxCount (USART_RESOURCES *usart) {

#ifdef __USART_DMA_TX
  if (usart->dma_tx) {
    return (usart->xfer->tx_num - __HAL_DMA_GET_COUNTER(usart->dma_tx->hdma));
  } else
#endif
  {
    return usart->xfer->tx_cnt;
  }
}

/**
  \fn          uint32_t USART_GetRxCount (USART_RESOURCES *usart)
  \brief       Get received data count.
  \param[in]   usart     Pointer to USART resources
  \return      number of data items received
*/
static uint32_t USART_GetRxCount (USART_RESOURCES *usart) {

#ifdef __USART_DMA_RX
  if (usart->dma_rx) {
    return (usart->xfer->rx_num - __HAL_DMA_GET_COUNTER(usart->dma_rx->hdma));
  } else
#endif
  {
    return usart->xfer->rx_cnt;
  }
}

/**
  \fn          int32_t USART_Control (uint32_t          control,
                                      uint32_t          arg,
                                      USART_RESOURCES  *usart)
  \brief       Control USART Interface.
  \param[in]   control  Operation
  \param[in]   arg      Argument of operation (optional)
  \param[in]   usart    Pointer to USART resources
  \return      common \ref execution_status and driver specific \ref usart_execution_status
*/
static int32_t USART_Control (uint32_t          control,
                              uint32_t          arg,
                              USART_RESOURCES  *usart) {
  uint32_t val, mode, flow_control, br, i;
  uint32_t cr1, cr2, cr3;
  GPIO_InitTypeDef GPIO_InitStruct;

  if ((usart->info->flags & USART_FLAG_POWERED) == 0) {
    // USART not powered
    return ARM_DRIVER_ERROR;
  }

  cr1  = 0;
  cr2  = 0;
  cr3  = 0;

  switch (control & ARM_USART_CONTROL_Msk) {
     // Control break
    case ARM_USART_CONTROL_BREAK:
      if (arg) {
        if (usart->info->flags & USART_FLAG_SEND_ACTIVE) return ARM_DRIVER_ERROR_BUSY;

        // Set Send active and Break flag
        usart->info->flags      |= USART_FLAG_SEND_ACTIVE;
        usart->xfer->break_flag  = 1;

        // Enable TX interrupt and send break
        usart->reg->CR1 |=  USART_CR1_TXEIE | USART_CR1_SBK;
      }
      else if (usart->xfer->break_flag){
        // Disable TX interrupt
        usart->reg->CR1 &= ~USART_CR1_TXEIE;

        // Clear break and Send Active flag
        usart->xfer->break_flag = 0;
        usart->info->flags     |= USART_FLAG_SEND_ACTIVE;
      }
      return ARM_DRIVER_OK;

    // Abort Send
    case ARM_USART_ABORT_SEND:
      // Disable TX and TC interrupt
      usart->reg->CR1 &= ~(USART_CR1_TXEIE | USART_CR1_TCIE);

      // If DMA mode - disable DMA channel
      if ((usart->dma_tx) && (usart->info->flags & USART_FLAG_SEND_ACTIVE)) {
        // DMA disable transmitter
        usart->reg->CR3 &= ~USART_CR3_DMAT;

        // Abort TX DMA transfer
        HAL_DMA_Abort (usart->dma_tx->hdma);
      }

      // Clear break flag
      usart->xfer->break_flag = 0;

      // Clear Send active flag
      usart->info->flags &= ~USART_FLAG_SEND_ACTIVE;
      return ARM_DRIVER_OK;

    // Abort receive
    case ARM_USART_ABORT_RECEIVE:
      // Disable RX interrupt
      usart->reg->CR1 &= ~USART_CR1_RXNEIE;

      // If DMA mode - disable DMA channel
      if ((usart->dma_rx) && (usart->info->status.rx_busy)) {
        // DMA disable Receiver
        usart->reg->CR3 &= ~USART_CR3_DMAR;

        // Abort RX DMA transfer
        HAL_DMA_Abort (usart->dma_rx->hdma);
      }

      // Clear RX busy status
      usart->info->status.rx_busy = 0;
      return ARM_DRIVER_OK;

    // Abort transfer
    case ARM_USART_ABORT_TRANSFER:
      // Disable TX, TC and RX interrupt
      usart->reg->CR1 &= ~(USART_CR1_TXEIE | USART_CR1_TCIE | USART_CR1_RXNEIE);

      // If DMA mode - disable DMA channel
      if ((usart->dma_tx) && (usart->info->flags & USART_FLAG_SEND_ACTIVE)) {
        // DMA disable transmitter
        usart->reg->CR3 &= ~USART_CR3_DMAT;

        // Abort TX DMA transfer
        HAL_DMA_Abort (usart->dma_tx->hdma);
      }

      // If DMA mode - disable DMA channel
      if ((usart->dma_rx) && (usart->info->status.rx_busy)) {
        // DMA disable Receiver
        usart->reg->CR3 &= ~USART_CR3_DMAR;

        // Abort RX DMA transfer
        HAL_DMA_Abort (usart->dma_rx->hdma);
      }

      // Clear busy statuses
      usart->info->status.rx_busy = 0;
      usart->info->flags &= ~USART_FLAG_SEND_ACTIVE;     
      return ARM_DRIVER_OK;

    // Control TX
    case ARM_USART_CONTROL_TX:
      // Check if TX pin available
      if (usart->io.tx == NULL) return ARM_DRIVER_ERROR;
      if (arg) {
        if (usart->info->mode != ARM_USART_MODE_SMART_CARD) {
          // USART TX pin function selected
          GPIO_InitStruct.Pin       = usart->io.tx->pin;
          GPIO_InitStruct.Mode      = GPIO_MODE_AF_PP;
          GPIO_InitStruct.Pull      = GPIO_NOPULL;
          GPIO_InitStruct.Speed     = GPIO_SPEED_LOW;
          GPIO_InitStruct.Alternate = usart->io.tx->af;
          HAL_GPIO_Init(usart->io.tx->port, &GPIO_InitStruct);
        }
        usart->info->flags |= USART_FLAG_TX_ENABLED;

        // Transmitter enable
        usart->reg->CR1 |= USART_CR1_TE;
      } else {
        usart->info->flags &= ~USART_FLAG_TX_ENABLED;
        
        // Transmitter disable
        usart->reg->CR1 &= ~USART_CR1_TE;
        if (usart->info->mode != ARM_USART_MODE_SMART_CARD) {
          // GPIO pin function selected
          HAL_GPIO_DeInit (usart->io.tx->port, usart->io.tx->pin);
        }
      }
      return ARM_DRIVER_OK;

    // Control RX
    case ARM_USART_CONTROL_RX:
      // Check if RX line available
      if (usart->io.rx == NULL) return ARM_DRIVER_ERROR;
      if (arg) {
        if ((usart->info->mode != ARM_USART_MODE_SMART_CARD)   &&
            (usart->info->mode != ARM_USART_MODE_SINGLE_WIRE )) {
          // USART RX pin function selected
          GPIO_InitStruct.Pin       = usart->io.rx->pin;
          GPIO_InitStruct.Mode      = GPIO_MODE_AF_PP;
          GPIO_InitStruct.Pull      = GPIO_NOPULL;
          GPIO_InitStruct.Speed     = GPIO_SPEED_LOW;
          GPIO_InitStruct.Alternate = usart->io.rx->af;
          HAL_GPIO_Init(usart->io.rx->port, &GPIO_InitStruct);
        }
        usart->info->flags |= USART_FLAG_RX_ENABLED;

        // Enable Error interrupt,
        usart->reg->CR3 |= USART_CR3_EIE;

        // Break detection interrupt enable
        usart->reg->CR2 |= USART_CR2_LBDIE;

        if (!(usart->info->status.rx_busy && usart->dma_rx)) {
          // Enable RXNE interrupt
          usart->reg->CR1 |= USART_CR1_RXNEIE;
        }

        // Enable DMA receiver 
        usart->reg->CR3 |= USART_CR3_DMAR;

        // Receiver enable
        usart->reg->CR1 |= USART_CR1_RE;

      } else {
        usart->info->flags &= ~USART_FLAG_RX_ENABLED;

        // Receiver disable
        usart->reg->CR1 &= ~USART_CR1_RE;

        // Disable DMA receiver
        usart->reg->CR3 &= ~USART_CR3_DMAR;

        // Disable RXNE interrupt
        usart->reg->CR1 &= ~USART_CR1_RXNEIE;

        // Disable Error interrupt,
        usart->reg->CR3 &= ~USART_CR3_EIE;

        // Break detection interrupt disable
        usart->reg->CR2 &= ~USART_CR2_LBDIE;

        if ((usart->info->mode != ARM_USART_MODE_SMART_CARD)   &&
            (usart->info->mode != ARM_USART_MODE_SINGLE_WIRE )) {
          // GPIO pin function selected
          HAL_GPIO_DeInit (usart->io.rx->port, usart->io.rx->pin);
        }
      }
      return ARM_DRIVER_OK;
  }

  // Check if busy
  if ((usart->info->status.rx_busy) || (!(usart->reg->SR & USART_SR_TC)) ||
      (usart->info->flags & USART_FLAG_SEND_ACTIVE)) {
    return ARM_DRIVER_ERROR_BUSY;
  }

  switch (control & ARM_USART_CONTROL_Msk) {
    case ARM_USART_MODE_ASYNCHRONOUS:
      mode = ARM_USART_MODE_ASYNCHRONOUS;
      break;
    case ARM_USART_MODE_SYNCHRONOUS_MASTER:
      if (usart->capabilities.synchronous_master) {
          // Enable Clock pin
          cr2 |= USART_CR2_CLKEN;

          // Enable last bit clock pulse
          cr2 |= USART_CR2_LBCL;
      } else return ARM_USART_ERROR_MODE;
      mode = ARM_USART_MODE_SYNCHRONOUS_MASTER;
      break;
    case ARM_USART_MODE_SYNCHRONOUS_SLAVE:
      return ARM_USART_ERROR_MODE;
    case ARM_USART_MODE_SINGLE_WIRE:
      // Enable Half duplex
      cr3 |= USART_CR3_HDSEL;
      mode = ARM_USART_MODE_SINGLE_WIRE;
      break;
    case ARM_USART_MODE_IRDA:
      // Enable IrDA mode
      cr3 |= USART_CR3_IREN;
      mode = ARM_USART_MODE_IRDA;
      break;
    case ARM_USART_MODE_SMART_CARD:
      if (usart->capabilities.smart_card) {
        // Enable Smart card mode
        cr3 |= USART_CR3_SCEN;
      } else return ARM_USART_ERROR_MODE;
      mode = ARM_USART_MODE_SMART_CARD;
      break;

    // Default TX value
    case ARM_USART_SET_DEFAULT_TX_VALUE:
      usart->xfer->def_val = arg;
      return ARM_DRIVER_OK;

    // IrDA pulse
    case ARM_USART_SET_IRDA_PULSE:
      if (usart->info->mode != ARM_USART_MODE_IRDA) {
        if (arg != 0) {
          // IrDa low-power
          usart->reg->CR3 |= USART_CR3_IRLP;

          // Get clock
          val = usart->periph_clock();

          // Calculate period in ns
          val = 1000000000 / val;
          for (i = 1; i < 0x100; i++) {
            if (val * i > arg) break;
          }
          if (i == 0x100) return ARM_DRIVER_ERROR;
          usart->reg->GTPR = (usart->reg->GTPR & ~USART_GTPR_PSC) | i;
        }
      } else return ARM_DRIVER_ERROR;
      return ARM_DRIVER_OK;

    // SmartCard guard time
    case ARM_USART_SET_SMART_CARD_GUARD_TIME:
      if (usart->info->mode == ARM_USART_MODE_SMART_CARD) {
        if (arg > 0xFF) return ARM_DRIVER_ERROR;

        usart->reg->GTPR = (usart->reg->GTPR & ~USART_GTPR_GT) | arg;
      } else return ARM_DRIVER_ERROR;
      return ARM_DRIVER_OK;

    // SmartCard clock
    case ARM_USART_SET_SMART_CARD_CLOCK:
      if (usart->info->mode == ARM_USART_MODE_SMART_CARD) {
        // Get clock
        val = usart->periph_clock();

        // Calculate period in ns
        val = 1000000000 / val;
        for (i = 1; i < 0x40; i++) {
          // if in +-2% tolerance
          if (((val * i * 2 * 100) < (arg * 102)) &&
              ((val * i * 2 * 100) > (arg * 98))    ) {
            break;
          }
        }
        if (i == 0x40) return ARM_DRIVER_ERROR;

        usart->reg->GTPR = (usart->reg->GTPR & ~USART_GTPR_PSC) | i;
      } else return ARM_DRIVER_ERROR;
      return ARM_DRIVER_OK;

    // SmartCard NACK
    case ARM_USART_CONTROL_SMART_CARD_NACK:
      if (usart->info->mode == ARM_USART_MODE_SMART_CARD) {
        // SmartCard NACK Enable
        if (arg) usart->reg->CR3 |= USART_CR3_NACK;
      } else return ARM_DRIVER_ERROR;
      return ARM_DRIVER_OK;

    // Unsupported command
    default: return ARM_DRIVER_ERROR_UNSUPPORTED;
  }

  // USART Data bits
  switch (control & ARM_USART_DATA_BITS_Msk) {
    case ARM_USART_DATA_BITS_7:
      if ((control & ARM_USART_PARITY_Msk) == ARM_USART_PARITY_NONE)
        return ARM_USART_ERROR_DATA_BITS;

      // 7 data bits, 8. data bit is parity bit
      break;
    case ARM_USART_DATA_BITS_8:
      if ((control & ARM_USART_PARITY_Msk) == ARM_USART_PARITY_NONE) {
        // 8-data bits, no parity
      } else {
        // 11-bit break detection
        cr2 |= USART_CR2_LBDL;

        // 8-data bits, 9. bit is parity bit
        cr1 |= USART_CR1_M;
      }
      break;
    case ARM_USART_DATA_BITS_9:
      if ((control & ARM_USART_PARITY_Msk) != ARM_USART_PARITY_NONE)
        return ARM_USART_ERROR_DATA_BITS;

      // 11-bit break detection
      cr2 |= USART_CR2_LBDL;

      // 9-data bits, no parity
      cr1 |= USART_CR1_M;
      break;
    default: return ARM_USART_ERROR_DATA_BITS;
  }

  // USART Parity
  switch (control & ARM_USART_PARITY_Msk) {
    case ARM_USART_PARITY_NONE:                              break;
    case ARM_USART_PARITY_EVEN:   cr1 |=  USART_CR1_PCE;     break;
    case ARM_USART_PARITY_ODD:    cr1 |= (USART_CR1_PCE | 
                                          USART_CR1_PS);     break;
    default: return ARM_USART_ERROR_PARITY;
  }

  // USART Stop bits
  switch (control & ARM_USART_STOP_BITS_Msk) {
    case ARM_USART_STOP_BITS_1:                              break;
    case ARM_USART_STOP_BITS_2:   cr2 |= USART_CR2_STOP_1;   break;
    case ARM_USART_STOP_BITS_1_5: cr2 |= USART_CR2_STOP_0 |
                                         USART_CR2_STOP_1;   break;
    case ARM_USART_STOP_BITS_0_5: cr2 |= USART_CR2_STOP_0;   break;
    default: return ARM_USART_ERROR_STOP_BITS;
  }

  // USART Flow control
  switch (control & ARM_USART_FLOW_CONTROL_Msk) {
    case ARM_USART_FLOW_CONTROL_NONE:
      flow_control = ARM_USART_FLOW_CONTROL_NONE;
      break;
    case ARM_USART_FLOW_CONTROL_RTS:
      if (usart->capabilities.flow_control_rts) {
        flow_control = ARM_USART_FLOW_CONTROL_RTS;
        // RTS Enable
        cr3 |= USART_CR3_RTSE;
      }
      else return ARM_USART_ERROR_FLOW_CONTROL;
      break;
    case ARM_USART_FLOW_CONTROL_CTS:
      if (usart->capabilities.flow_control_rts) {
        flow_control = ARM_USART_FLOW_CONTROL_CTS;
        // CTS Enable
        cr3 |= USART_CR3_CTSE;
      }
      else return ARM_USART_ERROR_FLOW_CONTROL;
      break;
    case ARM_USART_FLOW_CONTROL_RTS_CTS:
      if (usart->capabilities.flow_control_rts && 
          usart->capabilities.flow_control_cts) {
        flow_control = ARM_USART_FLOW_CONTROL_RTS_CTS;
        // RTS and CTS Enable, CTS interrupt enable
        cr3 |= (USART_CR3_RTSE | USART_CR3_CTSE | USART_CR3_CTSIE);
      } else return ARM_USART_ERROR_FLOW_CONTROL;
      break;
    default: return ARM_USART_ERROR_FLOW_CONTROL;
  }

  // Clock setting for synchronous mode
  if (mode == ARM_USART_MODE_SYNCHRONOUS_MASTER) {

    // Polarity
    switch (control & ARM_USART_CPOL_Msk) {
      case ARM_USART_CPOL0:
        break;
      case ARM_USART_CPOL1:
        cr2 |= USART_CR2_CPOL;
        break;
      default: return ARM_USART_ERROR_CPOL;
    }

    // Phase
    switch (control & ARM_USART_CPHA_Msk) {
      case ARM_USART_CPHA0:
        break;
      case ARM_USART_CPHA1:
        cr2 |= USART_CR2_CPHA;
        break;
      default: return ARM_USART_ERROR_CPHA;
    }
  }

  // USART Baudrate
  val = __USART_BRR(usart->periph_clock(), arg);
  br = ((usart->periph_clock() << 4) / (val & 0xFFFF)) >> 4;
  // If inside +/- 2% tolerance, baud rate configured correctly
  if (!(((br * 100) < (arg * 102)) && ((br * 100) > (arg * 98))))
    return ARM_USART_ERROR_BAUDRATE;

  // USART Disable
  usart->reg->CR1 &= ~USART_CR1_UE;

  // Configure Baud rate register
  usart->reg->BRR = val;

  // Configuration is OK - Mode is valid
  usart->info->mode = mode;

  // Save flow control mode
  usart->info->flow_control = flow_control;

  // Configure TX pin regarding mode and transmitter state
  switch (usart->info->mode) {
    case ARM_USART_MODE_SMART_CARD:
      // USART TX pin function selected
      GPIO_InitStruct.Pin       = usart->io.tx->pin;
      GPIO_InitStruct.Mode      = GPIO_MODE_AF_PP;
      GPIO_InitStruct.Pull      = GPIO_NOPULL;
      GPIO_InitStruct.Speed     = GPIO_SPEED_LOW;
      GPIO_InitStruct.Alternate = usart->io.tx->af;
      HAL_GPIO_Init(usart->io.tx->port, &GPIO_InitStruct);
      break;
    default:
      // Synchronous master/slave, asynchronous, single-wire and IrDA mode
      if (usart->info->flags & USART_FLAG_TX_ENABLED) {
        // USART TX pin function selected
        GPIO_InitStruct.Pin       = usart->io.tx->pin;
        GPIO_InitStruct.Mode      = GPIO_MODE_AF_PP;
        GPIO_InitStruct.Pull      = GPIO_NOPULL;
        GPIO_InitStruct.Speed     = GPIO_SPEED_LOW;
        GPIO_InitStruct.Alternate = usart->io.tx->af;
        HAL_GPIO_Init(usart->io.tx->port, &GPIO_InitStruct);
      } else {
        // GPIO pin function selected
        HAL_GPIO_DeInit (usart->io.tx->port, usart->io.tx->pin);
      }
  }

  // Configure RX pin regarding mode and receiver state
  switch (usart->info->mode) {
    case ARM_USART_MODE_SINGLE_WIRE:
    case ARM_USART_MODE_SMART_CARD:
      // GPIO pin function selected
      HAL_GPIO_DeInit (usart->io.rx->port, usart->io.rx->pin);
      break;
    default:
      // Synchronous master/slave, asynchronous and  IrDA mode
      if (usart->info->flags & USART_FLAG_RX_ENABLED) {
        // USART RX pin function selected
        GPIO_InitStruct.Pin       = usart->io.rx->pin;
        GPIO_InitStruct.Mode      = GPIO_MODE_AF_PP;
        GPIO_InitStruct.Pull      = GPIO_NOPULL;
        GPIO_InitStruct.Speed     = GPIO_SPEED_LOW;
        GPIO_InitStruct.Alternate = usart->io.rx->af;
        HAL_GPIO_Init(usart->io.rx->port, &GPIO_InitStruct);
      } else {
       // GPIO pin function selected
       HAL_GPIO_DeInit (usart->io.rx->port, usart->io.rx->pin);
      }
      break;
  }

  // Configure CLK pin regarding mode
  if (usart->io.ck) {
    switch (usart->info->mode) {
      case ARM_USART_MODE_SMART_CARD:
      case ARM_USART_MODE_SYNCHRONOUS_MASTER:
        // USART CK pin function selected
        GPIO_InitStruct.Pin       = usart->io.ck->pin;
        GPIO_InitStruct.Mode      = GPIO_MODE_AF_PP;
        GPIO_InitStruct.Pull      = GPIO_NOPULL;
        GPIO_InitStruct.Speed     = GPIO_SPEED_LOW;
        GPIO_InitStruct.Alternate = usart->io.ck->af;
        HAL_GPIO_Init(usart->io.ck->port, &GPIO_InitStruct);
        break;
      default:
        // Asynchronous, Single-wire and IrDA mode
        // GPIO pin function selected
        HAL_GPIO_DeInit (usart->io.ck->port, usart->io.ck->pin);
    }
  }

  // Configure RTS pin regarding Flow control configuration
  if (usart->io.rts) {
    if ((flow_control == ARM_USART_FLOW_CONTROL_RTS) ||
        (flow_control == ARM_USART_FLOW_CONTROL_RTS_CTS)) {
      // USART RTS Alternate function
      GPIO_InitStruct.Pin       = usart->io.rts->pin;
      GPIO_InitStruct.Mode      = GPIO_MODE_AF_PP;
      GPIO_InitStruct.Pull      = GPIO_NOPULL;
      GPIO_InitStruct.Speed     = GPIO_SPEED_LOW;
      GPIO_InitStruct.Alternate = usart->io.rts->af;
      HAL_GPIO_Init(usart->io.rts->port, &GPIO_InitStruct);
    } else {
      // GPIO output
      GPIO_InitStruct.Pin       = usart->io.rts->pin;
      GPIO_InitStruct.Mode      = GPIO_MODE_OUTPUT_PP;
      GPIO_InitStruct.Pull      = GPIO_NOPULL;
      GPIO_InitStruct.Speed     = GPIO_SPEED_LOW;
      HAL_GPIO_Init(usart->io.rts->port, &GPIO_InitStruct);
    }
  }

  // Configure CTS pin regarding Flow control configuration
  if (usart->io.cts) {
    if ((flow_control == ARM_USART_FLOW_CONTROL_CTS) ||
        (flow_control == ARM_USART_FLOW_CONTROL_RTS_CTS)) {
      // USART CTS Alternate function
      GPIO_InitStruct.Pin       = usart->io.cts->pin;
      GPIO_InitStruct.Mode      = GPIO_MODE_AF_PP;
      GPIO_InitStruct.Pull      = GPIO_NOPULL;
      GPIO_InitStruct.Speed     = GPIO_SPEED_LOW;
      GPIO_InitStruct.Alternate = usart->io.cts->af;
      HAL_GPIO_Init(usart->io.cts->port, &GPIO_InitStruct);
    } else {
      // GPIO input
      GPIO_InitStruct.Pin       = usart->io.cts->pin;
      GPIO_InitStruct.Mode      = GPIO_MODE_INPUT;
      GPIO_InitStruct.Pull      = GPIO_NOPULL;
      GPIO_InitStruct.Speed     = GPIO_SPEED_LOW;
      HAL_GPIO_Init(usart->io.cts->port, &GPIO_InitStruct);
    }
  }

  // Configure USART control registers
  usart->reg->CR1 = cr1;
  usart->reg->CR2 = cr2;
  usart->reg->CR3 = cr3;

  // USART Enable
  usart->reg->CR1 |= USART_CR1_UE;

  // Set configured flag
  usart->info->flags |= USART_FLAG_CONFIGURED;

  return ARM_DRIVER_OK;
}

/**
  \fn          ARM_USART_STATUS USART_GetStatus (USART_RESOURCES *usart)
  \brief       Get USART status.
  \param[in]   usart     Pointer to USART resources
  \return      USART status \ref ARM_USART_STATUS
*/
static ARM_USART_STATUS USART_GetStatus (USART_RESOURCES *usart) {
  usart->info->status.tx_busy = ((usart->reg->SR & USART_SR_TC) ? (0) : (1));
  return usart->info->status;
}

/**
  \fn          int32_t USART_SetModemControl (ARM_USART_MODEM_CONTROL  control,
                                              USART_RESOURCES         *usart)
  \brief       Set USART Modem Control line state.
  \param[in]   control   \ref ARM_USART_MODEM_CONTROL
  \param[in]   usart     Pointer to USART resources
  \return      \ref execution_status
*/
static int32_t USART_SetModemControl (ARM_USART_MODEM_CONTROL  control,
                                      USART_RESOURCES         *usart) {

  switch (control) {
    case ARM_USART_RTS_CLEAR:
      if ((usart->info->flow_control == ARM_USART_FLOW_CONTROL_NONE) ||
          (usart->info->flow_control == ARM_USART_FLOW_CONTROL_CTS)) {
        HAL_GPIO_WritePin (usart->io.rts->port, usart->io.rts->pin, GPIO_PIN_SET);
      } else {
        // Hardware RTS
        return ARM_DRIVER_ERROR;
      }
      break;
    case ARM_USART_RTS_SET:
      if ((usart->info->flow_control == ARM_USART_FLOW_CONTROL_NONE) ||
          (usart->info->flow_control == ARM_USART_FLOW_CONTROL_CTS)) {
        HAL_GPIO_WritePin (usart->io.rts->port, usart->io.rts->pin, GPIO_PIN_RESET);
      } else {
        // Hardware RTS
        return ARM_DRIVER_ERROR;
      }
      break;
    default: return ARM_DRIVER_ERROR;
  }

  return ARM_DRIVER_OK;
}

/**
  \fn          ARM_USART_MODEM_STATUS USART_GetModemStatus (USART_RESOURCES *usart)
  \brief       Get USART Modem Status lines state.
  \param[in]   usart     Pointer to USART resources
  \return      modem status \ref ARM_USART_MODEM_STATUS
*/
static ARM_USART_MODEM_STATUS USART_GetModemStatus (USART_RESOURCES *usart) {
  ARM_USART_MODEM_STATUS modem_status;

  if ((usart->info->flow_control == ARM_USART_FLOW_CONTROL_NONE) ||
      (usart->info->flow_control == ARM_USART_FLOW_CONTROL_RTS)) {
    modem_status.cts = !(HAL_GPIO_ReadPin (usart->io.cts->port, usart->io.cts->pin));
  } else {
    // Hardware CTS
    modem_status.cts = 0;
  }
  modem_status.dsr = 0;
  modem_status.ri  = 0;
  modem_status.dcd = 0;

  return modem_status;
}


/**
  \fn          void USART_IRQHandler (USART_RESOURCES *usart)
  \brief       USART Interrupt handler.
  \param[in]   usart     Pointer to USART resources
*/
void USART_IRQHandler (USART_RESOURCES *usart) {
  uint32_t val, sr, event;
  uint16_t data;

  // Read USART status register
  sr = usart->reg->SR;

  event = 0;
  
  // Transmit data register empty
  if (sr & USART_SR_TXE & usart->reg->CR1) {

    // Break handling
    if (usart->xfer->break_flag) {
      // Send break
      usart->reg->CR1 |= USART_CR1_SBK;
    } else {
      if(usart->xfer->tx_num != usart->xfer->tx_cnt) {
        if ((usart->info->mode == ARM_USART_MODE_SYNCHRONOUS_MASTER) &&
             (usart->xfer->sync_mode == USART_SYNC_MODE_RX)) {
          // Dummy write in synchronous receive only mode
          data = usart->xfer->def_val;
        } else {
          // Write data to TX FIFO
          data = *(usart->xfer->tx_buf++);

          // If nine bit data, no parity
          if (((usart->reg->CR1 & USART_CR1_PCE) == 0) &&
               (usart->reg->CR1 &USART_CR1_M)) {
            data |= *(usart->xfer->tx_buf++) << 8;
          }
        }
      }
      usart->xfer->tx_cnt++;

      // Write to data register
      usart->reg->DR = data;

      // Check if all data is transmitted
      if (usart->xfer->tx_num == usart->xfer->tx_cnt) {
        // Disable TXE interrupt
        usart->reg->CR1 &= ~USART_CR1_TXEIE;

        // Enable TC interrupt
        usart->reg->CR1 |= USART_CR1_TCIE;

        usart->info->flags &= ~USART_FLAG_SEND_ACTIVE;

        // Set send complete event
        if (usart->info->mode == ARM_USART_MODE_SYNCHRONOUS_MASTER) {
          if ((usart->xfer->sync_mode == USART_SYNC_MODE_TX)    &&
              ((usart->info->flags & USART_FLAG_RX_ENABLED) == 0)) {
            event |= ARM_USART_EVENT_SEND_COMPLETE;
          }
        } else {
          event |= ARM_USART_EVENT_SEND_COMPLETE;
        }
      }
    }
  }

  // Transmission complete
  if (sr & USART_SR_TC & usart->reg->CR1) {
    // Disable transmission complete interrupt
    usart->reg->CR1 &= ~USART_CR1_TCIE;
    event |= ARM_USART_EVENT_TX_COMPLETE;
  }

  // Read Data register not empty
  if (sr & USART_SR_RXNE & usart->reg->CR1) {

    // Check for RX overflow
    if (usart->info->status.rx_busy == 0) {
      // New receive has not been started
      // Dump RX data
      usart->reg->DR;
      usart->info->status.rx_overflow = 1;
      event |= ARM_USART_EVENT_RX_OVERFLOW;
    } else {
      if ((usart->info->mode == ARM_USART_MODE_SYNCHRONOUS_MASTER)  &&
          (usart->xfer->sync_mode == USART_SYNC_MODE_TX)) {
        // Dummy read in synchronous transmit only mode
        usart->reg->DR;
      } else {
        // Read data from RX FIFO into receive buffer
        data = usart->reg->DR;
      }

      *(usart->xfer->rx_buf++) = (uint8_t)data;

      // If nine bit data, no parity
      if (((usart->reg->CR1 & USART_CR1_PCE) == 0) &&
           (usart->reg->CR1 &USART_CR1_M)) {
        *(usart->xfer->rx_buf++) = (uint8_t)(data >> 8);
      }
      usart->xfer->rx_cnt++;

      // Check if requested amount of data is received
      if (usart->xfer->rx_cnt == usart->xfer->rx_num) {

        // Clear RX busy flag and set receive transfer complete event
        usart->info->status.rx_busy = 0;
        if (usart->info->mode == ARM_USART_MODE_SYNCHRONOUS_MASTER) {
          val = usart->xfer->sync_mode;
          usart->xfer->sync_mode = 0;
          switch (val) {
            case USART_SYNC_MODE_TX:
              event |= ARM_USART_EVENT_SEND_COMPLETE;
              break;
            case USART_SYNC_MODE_RX:
              event |= ARM_USART_EVENT_RECEIVE_COMPLETE;
              break;
            case USART_SYNC_MODE_TX_RX:
              event |= ARM_USART_EVENT_TRANSFER_COMPLETE;
              break;
          }
        } else {
          event |= ARM_USART_EVENT_RECEIVE_COMPLETE;
        }
      }
    }
  }

  // RX Overrun
  if (sr & USART_SR_ORE) {
    // Shift register has been overwritten
    // Dummy data read to clear the ORE flag
    usart->reg->DR;
    usart->info->status.rx_overflow = 1;
    event |= ARM_USART_EVENT_RX_OVERFLOW;
  }

  // Framing error
  if (sr & USART_SR_FE) {
    // Dummy data read to clear the FE flag
    usart->reg->DR;
    usart->info->status.rx_framing_error = 1;
    event |= ARM_USART_EVENT_RX_FRAMING_ERROR;
  }

  // Parity error
  if (sr & USART_SR_PE) {
    // Dummy data read to clear the PE flag
    usart->reg->DR;
    usart->info->status.rx_parity_error = 1;
    event |= ARM_USART_EVENT_RX_PARITY_ERROR;
  }

  // Break Detection
  if (sr & USART_SR_LBD) {
    // Clear Break detection flag
    usart->reg->SR &= ~USART_SR_LBD;

    usart->info->status.rx_break = 1;
    event |= ARM_USART_EVENT_RX_BREAK; 
  }

  // Send Event
  if (event && usart->info->cb_event) {
    usart->info->cb_event (event);
  }
}

#ifdef __USART_DMA_TX
void USART_TX_DMA_Complete(USART_RESOURCES *usart) {

  usart->xfer->tx_cnt = usart->xfer->tx_num;
  // Clear TX busy flag
  usart->info->flags &= ~USART_FLAG_SEND_ACTIVE;

  // TC interrupt enable
  usart->reg->CR1 |= USART_CR1_TCIE; 

  // Set Send Complete event for asynchronous transfers
  if (usart->info->mode != ARM_USART_MODE_SYNCHRONOUS_MASTER) {
    if (usart->info->cb_event)
      usart->info->cb_event (ARM_USART_EVENT_SEND_COMPLETE);
  }
}
#endif

#ifdef __USART_DMA_RX
void USART_RX_DMA_Complete(USART_RESOURCES *usart) {
  uint32_t val, event; 

  if (usart->info->mode == ARM_USART_MODE_SYNCHRONOUS_MASTER) {
    val = usart->xfer->sync_mode;
    usart->xfer->sync_mode = 0;
    switch (val) {
      case USART_SYNC_MODE_TX:
        event |= ARM_USART_EVENT_SEND_COMPLETE;
        break;
      case USART_SYNC_MODE_RX:
        event |= ARM_USART_EVENT_RECEIVE_COMPLETE;
        break;
      case USART_SYNC_MODE_TX_RX:
        event |= ARM_USART_EVENT_TRANSFER_COMPLETE;
         break;
    }
  } else {
    event |= ARM_USART_EVENT_RECEIVE_COMPLETE;
  }

  usart->xfer->rx_cnt = usart->xfer->rx_num; 
  usart->info->status.rx_busy = 0;

  // Enable RXNE interrupt to detect RX overrun
  usart->reg->CR1 |= USART_CR1_RXNEIE;

  if (usart->info->cb_event && event) usart->info->cb_event (event);
}
#endif



#ifdef MX_USART1
// USART1 Driver Wrapper functions
static ARM_USART_CAPABILITIES  USART1_GetCapabilities (void)                                                { return USART_GetCapabilities (&USART1_Resources); }
static int32_t                 USART1_Initialize      (ARM_USART_SignalEvent_t cb_event)                    { return USART_Initialize (cb_event, &USART1_Resources); }
static int32_t                 USART1_Uninitialize    (void)                                                { return USART_Uninitialize (&USART1_Resources); }
static int32_t                 USART1_PowerControl    (ARM_POWER_STATE state)                               { return USART_PowerControl (state, &USART1_Resources); }
static int32_t                 USART1_Send            (const void *data, uint32_t num)                      { return USART_Send (data, num, &USART1_Resources); }
static int32_t                 USART1_Receive         (void *data, uint32_t num)                            { return USART_Receive (data, num, &USART1_Resources); }
static int32_t                 USART1_Transfer        (const void *data_out, void *data_in, uint32_t num)   { return USART_Transfer (data_out, data_in, num, &USART1_Resources); }
static uint32_t                USART1_GetTxCount      (void)                                                { return USART_GetTxCount (&USART1_Resources); }
static uint32_t                USART1_GetRxCount      (void)                                                { return USART_GetRxCount (&USART1_Resources); }
static int32_t                 USART1_Control         (uint32_t control, uint32_t arg)                      { return USART_Control (control, arg, &USART1_Resources); }
static ARM_USART_STATUS        USART1_GetStatus       (void)                                                { return USART_GetStatus (&USART1_Resources); }
static int32_t                 USART1_SetModemControl (ARM_USART_MODEM_CONTROL control)                     { return USART_SetModemControl (control, &USART1_Resources); }
static ARM_USART_MODEM_STATUS  USART1_GetModemStatus  (void)                                                { return USART_GetModemStatus (&USART1_Resources); }
       void                    USART1_IRQHandler      (void)                                                {        USART_IRQHandler (&USART1_Resources); }

#ifdef MX_USART1_TX_DMA_Instance
      void                     USART1_TX_DMA_Complete (DMA_HandleTypeDef *hdma)                             {        USART_TX_DMA_Complete(&USART1_Resources); }

void USART1_TX_DMA_Handler (void) {
  HAL_NVIC_ClearPendingIRQ(MX_USART1_TX_DMA_IRQn);
  HAL_DMA_IRQHandler(&hdma_usart1_tx);
}
#endif
#ifdef MX_USART1_RX_DMA_Instance
      void                     USART1_RX_DMA_Complete (DMA_HandleTypeDef *hdma)                             {        USART_RX_DMA_Complete(&USART1_Resources); }

void USART1_RX_DMA_Handler (void) {
  HAL_NVIC_ClearPendingIRQ(MX_USART1_RX_DMA_IRQn);
  HAL_DMA_IRQHandler(&hdma_usart1_rx);
}
#endif

// USART1 Driver Control Block
ARM_DRIVER_USART Driver_USART1 = {
    USARTx_GetVersion,
    USART1_GetCapabilities,
    USART1_Initialize,
    USART1_Uninitialize,
    USART1_PowerControl,
    USART1_Send, 
    USART1_Receive,
    USART1_Transfer,
    USART1_GetTxCount,
    USART1_GetRxCount,
    USART1_Control,
    USART1_GetStatus,
    USART1_SetModemControl,
    USART1_GetModemStatus
};
#endif

#ifdef MX_USART2
// USART2 Driver Wrapper functions
static ARM_USART_CAPABILITIES  USART2_GetCapabilities (void)                                                { return USART_GetCapabilities (&USART2_Resources); }
static int32_t                 USART2_Initialize      (ARM_USART_SignalEvent_t cb_event)                    { return USART_Initialize (cb_event, &USART2_Resources); }
static int32_t                 USART2_Uninitialize    (void)                                                { return USART_Uninitialize (&USART2_Resources); }
static int32_t                 USART2_PowerControl    (ARM_POWER_STATE state)                               { return USART_PowerControl (state, &USART2_Resources); }
static int32_t                 USART2_Send            (const void *data, uint32_t num)                      { return USART_Send (data, num, &USART2_Resources); }
static int32_t                 USART2_Receive         (void *data, uint32_t num)                            { return USART_Receive (data, num, &USART2_Resources); }
static int32_t                 USART2_Transfer        (const void *data_out, void *data_in, uint32_t num)   { return USART_Transfer (data_out, data_in, num, &USART2_Resources); }
static uint32_t                USART2_GetTxCount      (void)                                                { return USART_GetTxCount (&USART2_Resources); }
static uint32_t                USART2_GetRxCount      (void)                                                { return USART_GetRxCount (&USART2_Resources); }
static int32_t                 USART2_Control         (uint32_t control, uint32_t arg)                      { return USART_Control (control, arg, &USART2_Resources); }
static ARM_USART_STATUS        USART2_GetStatus       (void)                                                { return USART_GetStatus (&USART2_Resources); }
static int32_t                 USART2_SetModemControl (ARM_USART_MODEM_CONTROL control)                     { return USART_SetModemControl (control, &USART2_Resources); }
static ARM_USART_MODEM_STATUS  USART2_GetModemStatus  (void)                                                { return USART_GetModemStatus (&USART2_Resources); }
       void                    USART2_IRQHandler      (void)                                                {        USART_IRQHandler (&USART2_Resources); }

#ifdef MX_USART2_TX_DMA_Instance
      void                     USART2_TX_DMA_Complete (DMA_HandleTypeDef *hdma)                             {        USART_TX_DMA_Complete(&USART2_Resources); }

void USART2_TX_DMA_Handler (void) {
  HAL_NVIC_ClearPendingIRQ(MX_USART2_TX_DMA_IRQn);
  HAL_DMA_IRQHandler(&hdma_usart2_tx);
}
#endif
#ifdef MX_USART2_RX_DMA_Instance
      void                     USART2_RX_DMA_Complete (DMA_HandleTypeDef *hdma)                             {        USART_RX_DMA_Complete(&USART2_Resources); }

void USART2_RX_DMA_Handler (void) {
  HAL_NVIC_ClearPendingIRQ(MX_USART2_RX_DMA_IRQn);
  HAL_DMA_IRQHandler(&hdma_usart2_rx);
}
#endif

// USART2 Driver Control Block
ARM_DRIVER_USART Driver_USART2 = {
    USARTx_GetVersion,
    USART2_GetCapabilities,
    USART2_Initialize,
    USART2_Uninitialize,
    USART2_PowerControl,
    USART2_Send, 
    USART2_Receive,
    USART2_Transfer,
    USART2_GetTxCount,
    USART2_GetRxCount,
    USART2_Control,
    USART2_GetStatus,
    USART2_SetModemControl,
    USART2_GetModemStatus
};
#endif

#ifdef MX_USART3
// USART3 Driver Wrapper functions
static ARM_USART_CAPABILITIES  USART3_GetCapabilities (void)                                                { return USART_GetCapabilities (&USART3_Resources); }
static int32_t                 USART3_Initialize      (ARM_USART_SignalEvent_t cb_event)                    { return USART_Initialize (cb_event, &USART3_Resources); }
static int32_t                 USART3_Uninitialize    (void)                                                { return USART_Uninitialize (&USART3_Resources); }
static int32_t                 USART3_PowerControl    (ARM_POWER_STATE state)                               { return USART_PowerControl (state, &USART3_Resources); }
static int32_t                 USART3_Send            (const void *data, uint32_t num)                      { return USART_Send (data, num, &USART3_Resources); }
static int32_t                 USART3_Receive         (void *data, uint32_t num)                            { return USART_Receive (data, num, &USART3_Resources); }
static int32_t                 USART3_Transfer        (const void *data_out, void *data_in, uint32_t num)   { return USART_Transfer (data_out, data_in, num, &USART3_Resources); }
static uint32_t                USART3_GetTxCount      (void)                                                { return USART_GetTxCount (&USART3_Resources); }
static uint32_t                USART3_GetRxCount      (void)                                                { return USART_GetRxCount (&USART3_Resources); }
static int32_t                 USART3_Control         (uint32_t control, uint32_t arg)                      { return USART_Control (control, arg, &USART3_Resources); }
static ARM_USART_STATUS        USART3_GetStatus       (void)                                                { return USART_GetStatus (&USART3_Resources); }
static int32_t                 USART3_SetModemControl (ARM_USART_MODEM_CONTROL control)                     { return USART_SetModemControl (control, &USART3_Resources); }
static ARM_USART_MODEM_STATUS  USART3_GetModemStatus  (void)                                                { return USART_GetModemStatus (&USART3_Resources); }
       void                    USART3_IRQHandler      (void)                                                {        USART_IRQHandler (&USART3_Resources); }

#ifdef MX_USART3_TX_DMA_Instance
      void                     USART3_TX_DMA_Complete (DMA_HandleTypeDef *hdma)                             {        USART_TX_DMA_Complete(&USART3_Resources); }

void USART3_TX_DMA_Handler (void) {
  HAL_NVIC_ClearPendingIRQ(MX_USART3_TX_DMA_IRQn);
  HAL_DMA_IRQHandler(&hdma_usart3_tx);
}
#endif
#ifdef MX_USART3_RX_DMA_Instance
      void                     USART3_RX_DMA_Complete (DMA_HandleTypeDef *hdma)                             {        USART_RX_DMA_Complete(&USART3_Resources); }

void USART3_RX_DMA_Handler (void) {
  HAL_NVIC_ClearPendingIRQ(MX_USART3_RX_DMA_IRQn);
  HAL_DMA_IRQHandler(&hdma_usart3_rx);
}
#endif

// USART3 Driver Control Block
ARM_DRIVER_USART Driver_USART3 = {
    USARTx_GetVersion,
    USART3_GetCapabilities,
    USART3_Initialize,
    USART3_Uninitialize,
    USART3_PowerControl,
    USART3_Send, 
    USART3_Receive,
    USART3_Transfer,
    USART3_GetTxCount,
    USART3_GetRxCount,
    USART3_Control,
    USART3_GetStatus,
    USART3_SetModemControl,
    USART3_GetModemStatus
};
#endif

#ifdef MX_UART4
// USART4 Driver Wrapper functions
static ARM_USART_CAPABILITIES  USART4_GetCapabilities (void)                                                { return USART_GetCapabilities (&USART4_Resources); }
static int32_t                 USART4_Initialize      (ARM_USART_SignalEvent_t cb_event)                    { return USART_Initialize (cb_event, &USART4_Resources); }
static int32_t                 USART4_Uninitialize    (void)                                                { return USART_Uninitialize (&USART4_Resources); }
static int32_t                 USART4_PowerControl    (ARM_POWER_STATE state)                               { return USART_PowerControl (state, &USART4_Resources); }
static int32_t                 USART4_Send            (const void *data, uint32_t num)                      { return USART_Send (data, num, &USART4_Resources); }
static int32_t                 USART4_Receive         (void *data, uint32_t num)                            { return USART_Receive (data, num, &USART4_Resources); }
static int32_t                 USART4_Transfer        (const void *data_out, void *data_in, uint32_t num)   { return USART_Transfer (data_out, data_in, num, &USART4_Resources); }
static uint32_t                USART4_GetTxCount      (void)                                                { return USART_GetTxCount (&USART4_Resources); }
static uint32_t                USART4_GetRxCount      (void)                                                { return USART_GetRxCount (&USART4_Resources); }
static int32_t                 USART4_Control         (uint32_t control, uint32_t arg)                      { return USART_Control (control, arg, &USART4_Resources); }
static ARM_USART_STATUS        USART4_GetStatus       (void)                                                { return USART_GetStatus (&USART4_Resources); }
static int32_t                 USART4_SetModemControl (ARM_USART_MODEM_CONTROL control)                     { return USART_SetModemControl (control, &USART4_Resources); }
static ARM_USART_MODEM_STATUS  USART4_GetModemStatus  (void)                                                { return USART_GetModemStatus (&USART4_Resources); }
       void                    UART4_IRQHandler       (void)                                                {        USART_IRQHandler (&USART4_Resources); }

#ifdef MX_UART4_TX_DMA_Instance
      void                     UART4_TX_DMA_Complete (DMA_HandleTypeDef *hdma)                             {        USART_TX_DMA_Complete(&USART4_Resources); }

void UART4_TX_DMA_Handler (void) {
  HAL_NVIC_ClearPendingIRQ(MX_UART4_TX_DMA_IRQn);
  HAL_DMA_IRQHandler(&hdma_uart4_tx);
}
#endif
#ifdef MX_UART4_RX_DMA_Instance
      void                     UART4_RX_DMA_Complete (DMA_HandleTypeDef *hdma)                             {        USART_RX_DMA_Complete(&USART4_Resources); }

void UART4_RX_DMA_Handler (void) {
  HAL_NVIC_ClearPendingIRQ(MX_UART4_RX_DMA_IRQn);
  HAL_DMA_IRQHandler(&hdma_uart4_rx);
}
#endif

// USART4 Driver Control Block
ARM_DRIVER_USART Driver_USART4 = {
    USARTx_GetVersion,
    USART4_GetCapabilities,
    USART4_Initialize,
    USART4_Uninitialize,
    USART4_PowerControl,
    USART4_Send, 
    USART4_Receive,
    USART4_Transfer,
    USART4_GetTxCount,
    USART4_GetRxCount,
    USART4_Control,
    USART4_GetStatus,
    USART4_SetModemControl,
    USART4_GetModemStatus
};
#endif

#ifdef MX_UART5
// USART5 Driver Wrapper functions
static ARM_USART_CAPABILITIES  USART5_GetCapabilities (void)                                                { return USART_GetCapabilities (&USART5_Resources); }
static int32_t                 USART5_Initialize      (ARM_USART_SignalEvent_t cb_event)                    { return USART_Initialize (cb_event, &USART5_Resources); }
static int32_t                 USART5_Uninitialize    (void)                                                { return USART_Uninitialize (&USART5_Resources); }
static int32_t                 USART5_PowerControl    (ARM_POWER_STATE state)                               { return USART_PowerControl (state, &USART5_Resources); }
static int32_t                 USART5_Send            (const void *data, uint32_t num)                      { return USART_Send (data, num, &USART5_Resources); }
static int32_t                 USART5_Receive         (void *data, uint32_t num)                            { return USART_Receive (data, num, &USART5_Resources); }
static int32_t                 USART5_Transfer        (const void *data_out, void *data_in, uint32_t num)   { return USART_Transfer (data_out, data_in, num, &USART5_Resources); }
static uint32_t                USART5_GetTxCount      (void)                                                { return USART_GetTxCount (&USART5_Resources); }
static uint32_t                USART5_GetRxCount      (void)                                                { return USART_GetRxCount (&USART5_Resources); }
static int32_t                 USART5_Control         (uint32_t control, uint32_t arg)                      { return USART_Control (control, arg, &USART5_Resources); }
static ARM_USART_STATUS        USART5_GetStatus       (void)                                                { return USART_GetStatus (&USART5_Resources); }
static int32_t                 USART5_SetModemControl (ARM_USART_MODEM_CONTROL control)                     { return USART_SetModemControl (control, &USART5_Resources); }
static ARM_USART_MODEM_STATUS  USART5_GetModemStatus  (void)                                                { return USART_GetModemStatus (&USART5_Resources); }
       void                    UART5_IRQHandler       (void)                                                {        USART_IRQHandler (&USART5_Resources); }

#ifdef MX_UART5_TX_DMA_Instance
      void                     UART5_TX_DMA_Complete (DMA_HandleTypeDef *hdma)                             {        USART_TX_DMA_Complete(&USART5_Resources); }

void UART5_TX_DMA_Handler (void) {
  HAL_NVIC_ClearPendingIRQ(MX_UART5_TX_DMA_IRQn);
  HAL_DMA_IRQHandler(&hdma_uart5_tx);
}
#endif
#ifdef MX_UART5_RX_DMA_Instance
      void                     UART5_RX_DMA_Complete (DMA_HandleTypeDef *hdma)                             {        USART_RX_DMA_Complete(&USART5_Resources); }

void UART5_RX_DMA_Handler (void) {
  HAL_NVIC_ClearPendingIRQ(MX_UART5_RX_DMA_IRQn);
  HAL_DMA_IRQHandler(&hdma_uart5_rx);
}
#endif

// USART5 Driver Control Block
ARM_DRIVER_USART Driver_USART5 = {
    USARTx_GetVersion,
    USART5_GetCapabilities,
    USART5_Initialize,
    USART5_Uninitialize,
    USART5_PowerControl,
    USART5_Send, 
    USART5_Receive,
    USART5_Transfer,
    USART5_GetTxCount,
    USART5_GetRxCount,
    USART5_Control,
    USART5_GetStatus,
    USART5_SetModemControl,
    USART5_GetModemStatus
};
#endif

#ifdef MX_USART6
// USART6 Driver Wrapper functions
static ARM_USART_CAPABILITIES  USART6_GetCapabilities (void)                                                { return USART_GetCapabilities (&USART6_Resources); }
static int32_t                 USART6_Initialize      (ARM_USART_SignalEvent_t cb_event)                    { return USART_Initialize (cb_event, &USART6_Resources); }
static int32_t                 USART6_Uninitialize    (void)                                                { return USART_Uninitialize (&USART6_Resources); }
static int32_t                 USART6_PowerControl    (ARM_POWER_STATE state)                               { return USART_PowerControl (state, &USART6_Resources); }
static int32_t                 USART6_Send            (const void *data, uint32_t num)                      { return USART_Send (data, num, &USART6_Resources); }
static int32_t                 USART6_Receive         (void *data, uint32_t num)                            { return USART_Receive (data, num, &USART6_Resources); }
static int32_t                 USART6_Transfer        (const void *data_out, void *data_in, uint32_t num)   { return USART_Transfer (data_out, data_in, num, &USART6_Resources); }
static uint32_t                USART6_GetTxCount      (void)                                                { return USART_GetTxCount (&USART6_Resources); }
static uint32_t                USART6_GetRxCount      (void)                                                { return USART_GetRxCount (&USART6_Resources); }
static int32_t                 USART6_Control         (uint32_t control, uint32_t arg)                      { return USART_Control (control, arg, &USART6_Resources); }
static ARM_USART_STATUS        USART6_GetStatus       (void)                                                { return USART_GetStatus (&USART6_Resources); }
static int32_t                 USART6_SetModemControl (ARM_USART_MODEM_CONTROL control)                     { return USART_SetModemControl (control, &USART6_Resources); }
static ARM_USART_MODEM_STATUS  USART6_GetModemStatus  (void)                                                { return USART_GetModemStatus (&USART6_Resources); }
       void                    USART6_IRQHandler      (void)                                                {        USART_IRQHandler (&USART6_Resources); }

#ifdef MX_USART6_TX_DMA_Instance
      void                     USART6_TX_DMA_Complete (DMA_HandleTypeDef *hdma)                             {        USART_TX_DMA_Complete(&USART6_Resources); }

void USART6_TX_DMA_Handler (void) {
  HAL_NVIC_ClearPendingIRQ(MX_USART6_TX_DMA_IRQn);
  HAL_DMA_IRQHandler(&hdma_usart6_tx);
}
#endif
#ifdef MX_USART6_RX_DMA_Instance
      void                     USART6_RX_DMA_Complete (DMA_HandleTypeDef *hdma)                             {        USART_RX_DMA_Complete(&USART6_Resources); }

void USART6_RX_DMA_Handler (void) {
  HAL_NVIC_ClearPendingIRQ(MX_USART6_RX_DMA_IRQn);
  HAL_DMA_IRQHandler(&hdma_usart6_rx);
}
#endif

// USART6 Driver Control Block
ARM_DRIVER_USART Driver_USART6 = {
    USARTx_GetVersion,
    USART6_GetCapabilities,
    USART6_Initialize,
    USART6_Uninitialize,
    USART6_PowerControl,
    USART6_Send, 
    USART6_Receive,
    USART6_Transfer,
    USART6_GetTxCount,
    USART6_GetRxCount,
    USART6_Control,
    USART6_GetStatus,
    USART6_SetModemControl,
    USART6_GetModemStatus
};
#endif

#ifdef MX_UART7
// USART7 Driver Wrapper functions
static ARM_USART_CAPABILITIES  USART7_GetCapabilities (void)                                                { return USART_GetCapabilities (&USART7_Resources); }
static int32_t                 USART7_Initialize      (ARM_USART_SignalEvent_t cb_event)                    { return USART_Initialize (cb_event, &USART7_Resources); }
static int32_t                 USART7_Uninitialize    (void)                                                { return USART_Uninitialize (&USART7_Resources); }
static int32_t                 USART7_PowerControl    (ARM_POWER_STATE state)                               { return USART_PowerControl (state, &USART7_Resources); }
static int32_t                 USART7_Send            (const void *data, uint32_t num)                      { return USART_Send (data, num, &USART7_Resources); }
static int32_t                 USART7_Receive         (void *data, uint32_t num)                            { return USART_Receive (data, num, &USART7_Resources); }
static int32_t                 USART7_Transfer        (const void *data_out, void *data_in, uint32_t num)   { return USART_Transfer (data_out, data_in, num, &USART7_Resources); }
static uint32_t                USART7_GetTxCount      (void)                                                { return USART_GetTxCount (&USART7_Resources); }
static uint32_t                USART7_GetRxCount      (void)                                                { return USART_GetRxCount (&USART7_Resources); }
static int32_t                 USART7_Control         (uint32_t control, uint32_t arg)                      { return USART_Control (control, arg, &USART7_Resources); }
static ARM_USART_STATUS        USART7_GetStatus       (void)                                                { return USART_GetStatus (&USART7_Resources); }
static int32_t                 USART7_SetModemControl (ARM_USART_MODEM_CONTROL control)                     { return USART_SetModemControl (control, &USART7_Resources); }
static ARM_USART_MODEM_STATUS  USART7_GetModemStatus  (void)                                                { return USART_GetModemStatus (&USART7_Resources); }
       void                    UART7_IRQHandler       (void)                                                {        USART_IRQHandler (&USART7_Resources); }

#ifdef MX_UART7_TX_DMA_Instance
      void                     UART7_TX_DMA_Complete (DMA_HandleTypeDef *hdma)                             {        USART_TX_DMA_Complete(&USART7_Resources); }

void UART7_TX_DMA_Handler (void) {
  HAL_NVIC_ClearPendingIRQ(MX_UART7_TX_DMA_IRQn);
  HAL_DMA_IRQHandler(&hdma_uart7_tx);
}
#endif
#ifdef MX_UART7_RX_DMA_Instance
      void                     UART7_RX_DMA_Complete (DMA_HandleTypeDef *hdma)                             {        USART_RX_DMA_Complete(&USART7_Resources); }

void UART7_RX_DMA_Handler (void) {
  HAL_NVIC_ClearPendingIRQ(MX_UART7_RX_DMA_IRQn);
  HAL_DMA_IRQHandler(&hdma_uart7_rx);
}
#endif

// USART7 Driver Control Block
ARM_DRIVER_USART Driver_USART7 = {
    USARTx_GetVersion,
    USART7_GetCapabilities,
    USART7_Initialize,
    USART7_Uninitialize,
    USART7_PowerControl,
    USART7_Send, 
    USART7_Receive,
    USART7_Transfer,
    USART7_GetTxCount,
    USART7_GetRxCount,
    USART7_Control,
    USART7_GetStatus,
    USART7_SetModemControl,
    USART7_GetModemStatus
};
#endif

#ifdef MX_UART8
// USART8 Driver Wrapper functions
static ARM_USART_CAPABILITIES  USART8_GetCapabilities (void)                                                { return USART_GetCapabilities (&USART8_Resources); }
static int32_t                 USART8_Initialize      (ARM_USART_SignalEvent_t cb_event)                    { return USART_Initialize (cb_event, &USART8_Resources); }
static int32_t                 USART8_Uninitialize    (void)                                                { return USART_Uninitialize (&USART8_Resources); }
static int32_t                 USART8_PowerControl    (ARM_POWER_STATE state)                               { return USART_PowerControl (state, &USART8_Resources); }
static int32_t                 USART8_Send            (const void *data, uint32_t num)                      { return USART_Send (data, num, &USART8_Resources); }
static int32_t                 USART8_Receive         (void *data, uint32_t num)                            { return USART_Receive (data, num, &USART8_Resources); }
static int32_t                 USART8_Transfer        (const void *data_out, void *data_in, uint32_t num)   { return USART_Transfer (data_out, data_in, num, &USART8_Resources); }
static uint32_t                USART8_GetTxCount      (void)                                                { return USART_GetTxCount (&USART8_Resources); }
static uint32_t                USART8_GetRxCount      (void)                                                { return USART_GetRxCount (&USART8_Resources); }
static int32_t                 USART8_Control         (uint32_t control, uint32_t arg)                      { return USART_Control (control, arg, &USART8_Resources); }
static ARM_USART_STATUS        USART8_GetStatus       (void)                                                { return USART_GetStatus (&USART8_Resources); }
static int32_t                 USART8_SetModemControl (ARM_USART_MODEM_CONTROL control)                     { return USART_SetModemControl (control, &USART8_Resources); }
static ARM_USART_MODEM_STATUS  USART8_GetModemStatus  (void)                                                { return USART_GetModemStatus (&USART8_Resources); }
       void                    UART8_IRQHandler       (void)                                                {        USART_IRQHandler (&USART8_Resources); }

#ifdef MX_UART8_TX_DMA_Instance
      void                     UART8_TX_DMA_Complete (DMA_HandleTypeDef *hdma)                             {        USART_TX_DMA_Complete(&USART8_Resources); }

void UART8_TX_DMA_Handler (void) {
  HAL_NVIC_ClearPendingIRQ(MX_UART8_TX_DMA_IRQn);
  HAL_DMA_IRQHandler(&hdma_uart8_tx);
}
#endif
#ifdef MX_UART8_RX_DMA_Instance
      void                     UART8_RX_DMA_Complete (DMA_HandleTypeDef *hdma)                             {        USART_RX_DMA_Complete(&USART8_Resources); }

void UART8_RX_DMA_Handler (void) {
  HAL_NVIC_ClearPendingIRQ(MX_UART8_RX_DMA_IRQn);
  HAL_DMA_IRQHandler(&hdma_uart8_rx);
}
#endif

// USART8 Driver Control Block
ARM_DRIVER_USART Driver_USART8 = {
    USARTx_GetVersion,
    USART8_GetCapabilities,
    USART8_Initialize,
    USART8_Uninitialize,
    USART8_PowerControl,
    USART8_Send, 
    USART8_Receive,
    USART8_Transfer,
    USART8_GetTxCount,
    USART8_GetRxCount,
    USART8_Control,
    USART8_GetStatus,
    USART8_SetModemControl,
    USART8_GetModemStatus
};
#endif
