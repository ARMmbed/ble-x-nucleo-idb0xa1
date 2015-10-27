/**
  ******************************************************************************
  * @file    x_nucleo_idb0xa1_targets.h
  * @author  AST / EST
  * @version V0.0.1
  * @date    24-July-2015
  * @brief   This header file is intended to manage the differences between 
  *          the different supported base-boards which might mount the
  *          X_NUCLEO_IDB0XA1 BlueNRG BLE Expansion Board.
  ******************************************************************************
  * @attention
  *
  * <h2><center>&copy; COPYRIGHT(c) 2015 STMicroelectronics</center></h2>
  *
  * Redistribution and use in source and binary forms, with or without modification,
  * are permitted provided that the following conditions are met:
  *   1. Redistributions of source code must retain the above copyright notice,
  *      this list of conditions and the following disclaimer.
  *   2. Redistributions in binary form must reproduce the above copyright notice,
  *      this list of conditions and the following disclaimer in the documentation
  *      and/or other materials provided with the distribution.
  *   3. Neither the name of STMicroelectronics nor the names of its contributors
  *      may be used to endorse or promote products derived from this software
  *      without specific prior written permission.
  *
  * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
  * AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
  * IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
  * DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE
  * FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
  * DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
  * SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
  * CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY,
  * OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
  * OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
  *
  ******************************************************************************
  */ 
 
/* Define to prevent from recursive inclusion --------------------------------*/
#ifndef _X_NUCLEO_IDB0XA1_TARGETS_H_
#define _X_NUCLEO_IDB0XA1_TARGETS_H_
 
/*** SPI ***/
/* Use Arduino I2C Connectors */
#define IDB0XA1_PIN_SPI_MOSI   (D11)
#define IDB0XA1_PIN_SPI_MISO   (D12)
#define IDB0XA1_PIN_SPI_nCS    (A1)
#define IDB0XA1_PIN_SPI_RESET  (D7)
#define IDB0XA1_PIN_SPI_IRQ    (A0)

/* NOTE: Define macro 'IDB0XA1_D13_PATCH' if you want to compile for a specifically
         modified version of the X_NUCLEO_IDB0XA1 expansion board in
         which pin 'D13' (rather than the standard pin 'D3') is used 
         in order to provide the SPI serial clock.
	 Expansion boards modified in this way allow to be used on almost
	 any Arduino-compliant base board.
*/
#if defined(IDB0XA1_D13_PATCH)
#define IDB0XA1_PIN_SPI_SCK    (D13)
#else // !defined(IDB0XA1_D13_PATCH)
#define IDB0XA1_PIN_SPI_SCK    (D3)
#endif // !defined(IDB0XA1_D13_PATCH)

#endif // _X_NUCLEO_IDB0XA1_TARGETS_H_
