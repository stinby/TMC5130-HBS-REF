/** \file HomebusMaster.c ******************************************************
 *
 *             Project: Homebus Reference Design
 *            Filename: HomebusMaster.c
 *         Description: Main program for the Homebus Master
 *
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
#include "max32660.h"
#include "gpio.h"
#include "SysTick.h"
#include "RS485.h"
#include "TMCL.h"
#include "HomebusMaster.h"
#include "Homebus.h"

const char VersionString[]="0025V100";  //!< Version information for the TMCL-IDE

static gpio_cfg_t led_out;     //!< Pin for LED (P0.13)
static uint32_t Delay;         //!< Delay for LED blinking

/***************************************************************//**
   \fn InitIO()
   \brief I/O intialization

   Initialize I/O ports.
********************************************************************/
void InitIO(void)
{
  led_out.port = PORT_0;
  led_out.mask = PIN_13;
  led_out.pad = GPIO_PAD_NONE;
  led_out.func = GPIO_FUNC_OUT;
  GPIO_Config(&led_out);
}

//Main program
void main()
{
  InitSysTick();
  InitIO();
  InitRS485(9600);
  HomebusInit(230400);

  for(;;)
  {
    if(abs(GetSysTimer()-Delay)>1000)
    {
      GPIO_OutToggle(&led_out);
      Delay=GetSysTimer();
    }

    ProcessCommand();
  }
}
