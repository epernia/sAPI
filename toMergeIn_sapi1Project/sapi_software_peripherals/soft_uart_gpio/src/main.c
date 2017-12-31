/* Copyright 2017, Agustin Bassi.
 * Copyright 2015-2016, Eric Pernia.
 * All rights reserved.
 *
 * This file is part sAPI library for microcontrollers.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions are met:
 *
 * 1. Redistributions of source code must retain the above copyright notice,
 *    this list of conditions and the following disclaimer.
 *
 * 2. Redistributions in binary form must reproduce the above copyright notice,
 *    this list of conditions and the following disclaimer in the documentation
 *    and/or other materials provided with the distribution.
 *
 * 3. Neither the name of the copyright holder nor the names of its
 *    contributors may be used to endorse or promote products derived from this
 *    software without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
 * AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
 * IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
 * ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE
 * LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
 * CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
 * SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
 * INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
 * CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
 * ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 * POSSIBILITY OF SUCH DAMAGE.
 *
 */

/*==================[inclusions]=============================================*/

#include "sapi.h"     // <= sAPI header
#include "sapi_softUart.h"

/*==================[macros and definitions]=================================*/

/*==================[internal data declaration]==============================*/

/*==================[internal functions declaration]=========================*/

/*==================[internal data definition]===============================*/

/*==================[external data definition]===============================*/

/*==================[internal functions definition]==========================*/

/*==================[external functions definition]==========================*/

int main(void){
uint8_t dataSoftUart;

	// Inicializar la placa
	boardConfig();

	// Configura la UART 232 a 9600 Baudios.
	uartConfig(UART_USB, 115200);
	softUartConfig(SOFT_UART_1, GPIO1, GPIO2, 9600);

	// Escribe un mensaje de bienvenida
	uartWriteString(UART_USB, "Hola mundo UART por soft!\n\r");
	softUartWriteString(SOFT_UART_1, "Hola mundo UART por soft!\n\r");

	while(1) {
		// Se queda esperando que llegue informacion por la UART por software
		if (softUartReadByte(SOFT_UART_1, &dataSoftUart)){
			if (dataSoftUart == 'h'){
				gpioWrite(LEDB, HIGH);

				uartWriteString(UART_USB, "LEDB encendido.\n\r");
				softUartWriteString(SOFT_UART_1, "LEDB encendido.\n\r");
			} else if (dataSoftUart == 'l'){
				gpioWrite(LEDB, LOW);

				uartWriteString(UART_USB, "LEDB apagado.\n\r");
				softUartWriteString(SOFT_UART_1, "LEDB apagado.\n\r");
			}
			delay (1000);
		}
	}
	return 0 ;
}

/*==================[end of file]============================================*/
