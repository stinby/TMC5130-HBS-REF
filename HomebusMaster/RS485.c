/** \file RS485.c ***********************************************************
 *
 *             Project: Homebus Reference Design
 *            Filename: RS485.c
 *         Description: RS485 interface for TMCL
 *                      (Trinamic Motion Control Language)
 *
 *    Revision History:
 *                    2021_01_26    Rev 1.00    Olav Kahlbaum   File created
 *
 *  -------------------------------------------------------------------- */

/*******************************************************************************
* Copyright (C) Maxim Integrated Products, Inc., All rights Reserved.
*
* This software is protected by copyright laws of the United States and
* of foreign countries. This material may also be protected by patent laws
* and technology transfer regulations of the United States and of foreign
* countries. This software is furnished under a license agreement and/or a
* nondisclosure agreement and may only be used or reproduced in accordance
* with the terms of those agreements. Dissemination of this information to
* any party or parties not specified in the license agreement and/or
* nondisclosure agreement is expressly prohibited.
*
* The above copyright notice and this permission notice shall be included
* in all copies or substantial portions of the Software.
*
* THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS
* OR IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF
* MERCHANTABILITY, FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT.
* IN NO EVENT SHALL MAXIM INTEGRATED BE LIABLE FOR ANY CLAIM, DAMAGES
* OR OTHER LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE,
* ARISING FROM, OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR
* OTHER DEALINGS IN THE SOFTWARE.
*
* Except as contained in this notice, the name of Maxim Integrated
* Products, Inc. shall not be used except as stated in the Maxim Integrated
* Products, Inc. Branding Policy.
*
* The mere transfer of this software does not imply any licenses
* of trade secrets, proprietary technology, copyrights, patents,
* trademarks, maskwork rights, or any other form of intellectual
* property whatsoever. Maxim Integrated Products, Inc. retains all
* ownership rights.
*******************************************************************************
*/

#include <stdlib.h>
#include "bits.h"
#include "mxc_sys.h"
#include "uart.h"
#include "gpio.h"

#define TMCL_COMMAND_LENGTH      9      //<! Length of a TMCL command: 9 bytes
#define TMCL_RX_THRESHOLD        3      //<! Rx FIFO threshold for TMCL. TMCL_COMMAND_LENGTH must be dividable by TMCL_RX_THRESHOLD.

static const sys_cfg_uart_t sys_uart1_cfg = {
        MAP_A,
        UART_FLOW_DISABLE,
};

static gpio_cfg_t RS485DirPin;                         //!< Pin for switching the data direction
static volatile uint8_t Command[TMCL_COMMAND_LENGTH];  //!< Buffer for one TMCL command
static volatile uint8_t CommandCount;                  //!< Counter for filling the command buffer

/***************************************************************//**
   \fn UART1_IRQHandler
   \brief Interrupt handler for UART1.

   This function handles all UART1 interrupts.
********************************************************************/
void UART1_IRQHandler(void)
{
  uint32_t irq_flags;
  uint8_t i;

  irq_flags=MXC_UART1->int_fl;

  //Receive threshold interrupt
  if(irq_flags & MXC_F_UART_INT_FL_RX_FIFO_THRESH)
  {
    //The configured number of bytes have been received by the UART Rx FIFO.
    //Copy these bytes from the FIFO into our RS485 data buffer.
    for(i=0; i<TMCL_RX_THRESHOLD; i++)
    {
      if(CommandCount<TMCL_COMMAND_LENGTH) Command[CommandCount++]=MXC_UART1->fifo;
    }
    MXC_UART1->int_fl=MXC_F_UART_INT_FL_RX_FIFO_THRESH;
  }

  //Receive timeout interrupt
  if(irq_flags & MXC_F_UART_INT_FL_RX_TIMEOUT)
  {
    //A timeout has occured while receiving => discard everything.
    MXC_UART1->int_fl=MXC_F_UART_INT_FL_RX_TIMEOUT;
    MXC_UART1->ctrl |= MXC_F_UART_CTRL_RX_FLUSH;
    CommandCount=0;
  }

  //Receive overrun interrupt
  if(irq_flags & MXC_F_UART_INT_FL_RX_OVERRUN)
  {
    //An Rx overrun interrupt has occured (hardly possible) => discard everything.
    MXC_UART1->int_fl=MXC_F_UART_INT_FL_RX_OVERRUN;
    MXC_UART1->ctrl |= MXC_F_UART_CTRL_RX_FLUSH;
    CommandCount=0;
  }

  //Transmit interrupt
  if(irq_flags & (MXC_F_UART_INT_FL_TX_FIFO_THRESH|MXC_F_UART_INT_FL_TX_FIFO_ALMOST_EMPTY))
  {
    //Has the very last bit been sent out?
    if((MXC_UART1->status & MXC_F_UART_STATUS_TX_EMPTY) && !(MXC_UART1->status & MXC_F_UART_STATUS_TX_BUSY))
    {
      //Switch back transceiver to receive mode
      GPIO_OutClr(&RS485DirPin);

      //Reset and disable the transmit interrupts
      MXC_UART1->int_en&= ~(MXC_F_UART_INT_EN_TX_FIFO_THRESH|MXC_F_UART_INT_EN_TX_FIFO_ALMOST_EMPTY);
      MXC_UART1->int_fl|=MXC_F_UART_INT_FL_TX_FIFO_THRESH|MXC_F_UART_INT_FL_TX_FIFO_ALMOST_EMPTY;

      //Switch on the UART Rx pin again
      MXC_GPIO0->en&= ~BIT11;
      MXC_GPIO0->en1|=BIT11;
    }
  }
}


/***************************************************************//**
   \fn InitRS485()
   \param Baudrate: Baud rate to be used (mostly 9600)
   \brief Initalize everything for RS485 communication

   Initialize the RS485 communication.
********************************************************************/
void InitRS485(uint32_t Baudrate)
{
  uart_cfg_t cfg;
  gpio_cfg_t rxIn;

  //Initialize the P0.12 for switching the RS485 transceiver between
  //transmit and receive mode.
  RS485DirPin.port = PORT_0;
  RS485DirPin.mask = PIN_12;
  RS485DirPin.pad = GPIO_PAD_NONE;
  RS485DirPin.func = GPIO_FUNC_OUT;
  GPIO_Config(&RS485DirPin);
  GPIO_OutClr(&RS485DirPin);

  //Prepare the UART1 RxD pin as input with pull-up
  //for switching off UART RX later (to suppress the echo from the bus).
  rxIn.port = PORT_0;
  rxIn.mask = PIN_11;
  rxIn.pad = GPIO_PAD_PULL_UP;
  rxIn.func = GPIO_FUNC_IN;
  GPIO_Config(&rxIn);

  //Initialize UART1
  cfg.parity = UART_PARITY_DISABLE;
  cfg.size = UART_DATA_SIZE_8_BITS;
  cfg.stop = UART_STOP_1;
  cfg.flow = UART_FLOW_CTRL_DIS;
  cfg.pol = UART_FLOW_POL_EN;
  cfg.baud = Baudrate;
  UART_Init(MXC_UART1, &cfg, &sys_uart1_cfg);
  MXC_UART1->ctrl|=(5<<16);  //Timeout: 5 Frames


  //Configure UART1 interrupts
  MXC_UART1->int_fl=0xffffffff;
  NVIC_ClearPendingIRQ(UART1_IRQn);
  NVIC_DisableIRQ(UART1_IRQn);
  NVIC_SetPriority(UART1_IRQn, 1);
  NVIC_EnableIRQ(UART1_IRQn);
  MXC_UART1->thresh_ctrl=TMCL_RX_THRESHOLD;
  MXC_UART1->int_en=MXC_F_UART_INT_EN_RX_FIFO_THRESH|MXC_F_UART_INT_EN_RX_OVERRUN|MXC_F_UART_INT_EN_RX_TIMEOUT;
  MXC_UART1->ctrl|=MXC_F_UART_CTRL_TX_FLUSH|MXC_F_UART_CTRL_RX_FLUSH;
}


/***************************************************************//**
   \fn SendRS485Reply()
   \param *data: pointer to TMCL reply (9 bytes)
   \brief Send a TMCL reply via RS485

   Sends a TMCL reply (consisting of 9 bytes) via RS485.
   The echo will be suppressed by switching off the UART Rx
   pin as long as the sending goes.
   Switching back to receive mode is done in the interrupt handler.
********************************************************************/
void SendRS485Reply(uint8_t *data)
{
  uint8_t i;

  //Switch off UART1 Rx pin (to suppress the echo)
  MXC_GPIO0->en|=BIT11;
  MXC_GPIO0->en1&= ~BIT11;

  //Switch transceiver to transmit mode
  GPIO_OutSet(&RS485DirPin);

  //Activate transmit interrupts
  MXC_UART1->int_fl=MXC_F_UART_INT_FL_TX_FIFO_ALMOST_EMPTY;
  MXC_UART1->int_en|=MXC_F_UART_INT_EN_TX_FIFO_ALMOST_EMPTY;

  //Send out the data
  for(i=0; i<TMCL_COMMAND_LENGTH; i++)
  {
    while(MXC_UART1->status & MXC_F_UART_STATUS_TX_FULL);
    MXC_UART1->fifo=data[i];
  }
}

/***************************************************************//**
   \fn GetRS485Command()
   \param *data: pointer to array for received data (9 bytes)

   \return  TRUE if data could be read\n
            FALSE if no data available.

   \brief Get a TMCL command from RS485

   Read a TMCL command from the RS485 interface.
********************************************************************/
uint8_t GetRS485Command(uint8_t *data)
{
  uint8_t i;

  if(CommandCount==TMCL_COMMAND_LENGTH)
  {
    __disable_irq();
    for(i=0; i<9; i++) data[i]=Command[i];
    CommandCount=0;
    __enable_irq();

    return TRUE;
  }

  return FALSE;
}
