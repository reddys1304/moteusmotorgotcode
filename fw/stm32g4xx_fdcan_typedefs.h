  /******************************************************************************
  * @attention
  *
  * <h2><center>&copy; Copyright (c) 2019 STMicroelectronics.
  * All rights reserved.</center></h2>
  *
  * This software component is licensed by ST under BSD 3-Clause license,
  * the "License"; You may not use this file except in compliance with the
  * License. You may obtain a copy of the License at:
  *                        opensource.org/licenses/BSD-3-Clause
  *
  ******************************************************************************
  */
This header file defines a series of constants and macros related to the Flexible Data-Rate Controller Area Network (FDCAN) protocol on the STM32 series of microcontrollers by STMicroelectronics. FDCAN is an advanced version of the traditional CAN protocol, supporting higher data rates and more flexible data handling.

### Key Components:

1. **Event and Error Masks**: Defines bit masks for various FDCAN events and errors such as transmission events, reception events, and different types of errors.

2. **FDCAN Element Masks**: Defines bit masks for extracting or setting different fields in an FDCAN message element, such as standard and extended identifiers, remote transmission request, error state indicator, and others.

3. **SRAMCAN Constants**: Specifies the number and size of various elements in the FDCAN's message RAM, such as filter lists, FIFO elements, and event FIFO elements. It also defines the start addresses for different sections of the message RAM.

4. **FDCAN Message RAM Layout**: The constants provide a layout for the message RAM used by the FDCAN module. This includes start addresses for filter lists, FIFOs, and queues, as well as the overall size of the message RAM.

### Usage and Implications:

- **FDCAN Configuration and Handling**: These definitions are crucial for configuring the FDCAN peripheral on STM32 microcontrollers, particularly for setting up message filters, handling interrupts, and managing message buffers.
  
- **Message Parsing and Construction**: The element masks are used for parsing incoming messages and constructing outgoing messages, extracting or inserting fields like IDs, control bits, and data lengths.

- **Error and Event Management**: The error and event masks are used in interrupt handling to identify and respond to specific events and errors in FDCAN communication.

- **Memory Management**: The constants related to the message RAM provide a clear map of how the FDCAN peripheral uses its allocated SRAM, which is crucial for avoiding conflicts and ensuring efficient memory usage.

### General Observations:

- This file is part of a larger framework or library designed for embedded applications using STM32 microcontrollers, with a specific focus on FDCAN communication.
  
- The use of detailed macros and constants suggests a low-level implementation, where direct control and management of hardware resources are required.
  
- The content is highly specific to the STM32 microcontroller architecture and FDCAN module, indicating its use in specialized applications where FDCAN communication is a key requirement.
#pragma once

#define FDCAN_TX_EVENT_FIFO_MASK (FDCAN_IR_TEFL | FDCAN_IR_TEFF | FDCAN_IR_TEFN)
#define FDCAN_RX_FIFO0_MASK (FDCAN_IR_RF0L | FDCAN_IR_RF0F | FDCAN_IR_RF0N)
#define FDCAN_RX_FIFO1_MASK (FDCAN_IR_RF1L | FDCAN_IR_RF1F | FDCAN_IR_RF1N)
#define FDCAN_ERROR_MASK (FDCAN_IR_ELO | FDCAN_IR_WDI | FDCAN_IR_PEA | FDCAN_IR_PED | FDCAN_IR_ARA)
#define FDCAN_ERROR_STATUS_MASK (FDCAN_IR_EP | FDCAN_IR_EW | FDCAN_IR_BO)

#define FDCAN_ELEMENT_MASK_STDID ((uint32_t)0x1FFC0000U) /* Standard Identifier         */
#define FDCAN_ELEMENT_MASK_EXTID ((uint32_t)0x1FFFFFFFU) /* Extended Identifier         */
#define FDCAN_ELEMENT_MASK_RTR   ((uint32_t)0x20000000U) /* Remote Transmission Request */
#define FDCAN_ELEMENT_MASK_XTD   ((uint32_t)0x40000000U) /* Extended Identifier         */
#define FDCAN_ELEMENT_MASK_ESI   ((uint32_t)0x80000000U) /* Error State Indicator       */
#define FDCAN_ELEMENT_MASK_TS    ((uint32_t)0x0000FFFFU) /* Timestamp                   */
#define FDCAN_ELEMENT_MASK_DLC   ((uint32_t)0x000F0000U) /* Data Length Code            */
#define FDCAN_ELEMENT_MASK_BRS   ((uint32_t)0x00100000U) /* Bit Rate Switch             */
#define FDCAN_ELEMENT_MASK_FDF   ((uint32_t)0x00200000U) /* FD Format                   */
#define FDCAN_ELEMENT_MASK_EFC   ((uint32_t)0x00800000U) /* Event FIFO Control          */
#define FDCAN_ELEMENT_MASK_MM    ((uint32_t)0xFF000000U) /* Message Marker              */
#define FDCAN_ELEMENT_MASK_FIDX  ((uint32_t)0x7F000000U) /* Filter Index                */
#define FDCAN_ELEMENT_MASK_ANMF  ((uint32_t)0x80000000U) /* Accepted Non-matching Frame */
#define FDCAN_ELEMENT_MASK_ET    ((uint32_t)0x00C00000U) /* Event type                  */

#define SRAMCAN_FLS_NBR                  (28U)         /* Max. Filter List Standard Number      */
#define SRAMCAN_FLE_NBR                  ( 8U)         /* Max. Filter List Extended Number      */
#define SRAMCAN_RF0_NBR                  ( 3U)         /* RX FIFO 0 Elements Number             */
#define SRAMCAN_RF1_NBR                  ( 3U)         /* RX FIFO 1 Elements Number             */
#define SRAMCAN_TEF_NBR                  ( 3U)         /* TX Event FIFO Elements Number         */
#define SRAMCAN_TFQ_NBR                  ( 3U)         /* TX FIFO/Queue Elements Number         */

#define SRAMCAN_FLS_SIZE            ( 1U * 4U)         /* Filter Standard Element Size in bytes */
#define SRAMCAN_FLE_SIZE            ( 2U * 4U)         /* Filter Extended Element Size in bytes */
#define SRAMCAN_RF0_SIZE            (18U * 4U)         /* RX FIFO 0 Elements Size in bytes      */
#define SRAMCAN_RF1_SIZE            (18U * 4U)         /* RX FIFO 1 Elements Size in bytes      */
#define SRAMCAN_TEF_SIZE            ( 2U * 4U)         /* TX Event FIFO Elements Size in bytes  */
#define SRAMCAN_TFQ_SIZE            (18U * 4U)         /* TX FIFO/Queue Elements Size in bytes  */

#define SRAMCAN_FLSSA ((uint32_t)0)                                                      /* Filter List Standard Start Address */
#define SRAMCAN_FLESA ((uint32_t)(SRAMCAN_FLSSA + (SRAMCAN_FLS_NBR * SRAMCAN_FLS_SIZE))) /* Filter List Extended Start Address */
#define SRAMCAN_RF0SA ((uint32_t)(SRAMCAN_FLESA + (SRAMCAN_FLE_NBR * SRAMCAN_FLE_SIZE))) /* Rx FIFO 0 Start Address            */
#define SRAMCAN_RF1SA ((uint32_t)(SRAMCAN_RF0SA + (SRAMCAN_RF0_NBR * SRAMCAN_RF0_SIZE))) /* Rx FIFO 1 Start Address            */
#define SRAMCAN_TEFSA ((uint32_t)(SRAMCAN_RF1SA + (SRAMCAN_RF1_NBR * SRAMCAN_RF1_SIZE))) /* Tx Event FIFO Start Address        */
#define SRAMCAN_TFQSA ((uint32_t)(SRAMCAN_TEFSA + (SRAMCAN_TEF_NBR * SRAMCAN_TEF_SIZE))) /* Tx FIFO/Queue Start Address        */
#define SRAMCAN_SIZE  ((uint32_t)(SRAMCAN_TFQSA + (SRAMCAN_TFQ_NBR * SRAMCAN_TFQ_SIZE))) /* Message RAM size                   */
