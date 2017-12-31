/* Copyright 2017, Agustin Bassi.
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

/* Date: 2017-30-10 */

/*==================[inclusions]=============================================*/

#include "sapi_softUart.h"

/*==================[macros and definitions]=================================*/

#define IN_BUF_SIZE     			256

#define SOFT_UART_TIMER				TIMER0

// MACROS COMENTADAS. HACE LA BIBLIOTECA MAS DEPENDIENTE DE sAPI
// A FIN QUE SEA MAS FACIL DE UTILIZAR.

// Interface routines required:
// 1. get_rx_pin_status()
//    Returns 0 or 1 dependent on whether the receive pin is high or low.
//#define get_rx_pin_status()		gpioRead(GPIO_RX);
// 2. set_tx_pin_high()
//    Sets the transmit pin to the high state.
//#define set_tx_pin_high()		gpioWrite(GPIO_TX, HIGH)
// 3. set_tx_pin_low()
//    Sets the transmit pin to the low state.
//#define set_tx_pin_low()		gpioWrite(GPIO_TX, LOW)
// 4. idle()
//#define idle					sleepUntilNextInterrupt
//    Background functions to execute while waiting for input.
// 5. timer_set( BAUD_RATE )
//    Sets the timer to 3 times the baud rate.
//#define timer_set( BAUD_RATE )
// 6. set_timer_interrupt( timer_isr )
//    Enables the timer interrupt.
//#define set_timer_interrupt( timer_isr )

/*==================[internal data declaration]==============================*/

static unsigned char inbuf[IN_BUF_SIZE];
static unsigned char qin = 0;
static unsigned char qout = 0;

static char flag_rx_waiting_for_stop_bit;
static char flag_rx_off;
static char rx_mask;
static char flag_rx_ready;
static char flag_tx_ready;
static char timer_rx_ctr;
static char timer_tx_ctr;
static char bits_left_in_rx;
static char bits_left_in_tx;
static char rx_num_of_bits;
static char tx_num_of_bits;
static char internal_rx_buffer;
static char user_tx_buffer;
static uint16_t internal_tx_buffer;

/*==================[internal functions declaration]=========================*/

static void 	TimerCallback		( void );
static char 	_getchar			( void );
static void 	_putchar			( char ch );
static void 	flush_input_buffer	( void );
static char 	kbhit				( void );
static void 	turn_rx_on			( void );
static void 	turn_rx_off			( void );

/*==================[internal data definition]===============================*/

static gpioMap_t PinRx;
static gpioMap_t PinTx;

/*==================[external data definition]===============================*/

/*==================[internal functions definition]==========================*/

// Estas funciones que originalmente eran MACROS, se definieron como funciones
// con el fin de darle flexibilidad en los pines y las llamadas a funciones
// Se pueden cometar estas funciones y habilitar las MACROS arriba comentadas.
bool_t get_rx_pin_status(){
	return gpioRead(PinRx);
}

void set_tx_pin_high(){
	gpioWrite(PinTx, HIGH);
}

void set_tx_pin_low(){
	gpioWrite(PinTx, LOW);
}

void idle () {
	sleepUntilNextInterrupt();
}

static void TimerCallback		(void)
{
	char mask, start_bit, flag_in;

	// Transmitter Section
	if ( flag_tx_ready )
	{
		if ( --timer_tx_ctr<=0 )
		{
			// se fija cuanto es el LSB de internal_tx_buf
			mask = internal_tx_buffer&1;
			// Hace un shitf a la derecha (descarta el LSB)
			internal_tx_buffer >>= 1;
			if ( mask )
			{
				set_tx_pin_high();
			}
			else
			{
				set_tx_pin_low();
			}
			timer_tx_ctr = 3;
			// Si se mandaron todos los bits pone el flag en false
			if ( --bits_left_in_tx<=0 )
			{
				flag_tx_ready = FALSE;
			}
		}
	}
	// Receiver Section
	if ( flag_rx_off==FALSE )
	{
		if ( flag_rx_waiting_for_stop_bit )
		{
			if ( --timer_rx_ctr<=0 )
			{
				flag_rx_waiting_for_stop_bit = FALSE;
				flag_rx_ready = FALSE;
				internal_rx_buffer &= 0xFF;
				if ( internal_rx_buffer!=0xC2 )
				{
					inbuf[qin] = internal_rx_buffer;
					if ( ++qin>=IN_BUF_SIZE )
					{
						qin = 0;
					}
				}
			}
		}
		else        // rx_test_busy
		{
			if ( flag_rx_ready==FALSE )
			{
				start_bit = get_rx_pin_status();
				// Test for Start Bit
				if ( start_bit==0 )
				{
					flag_rx_ready = TRUE;
					internal_rx_buffer = 0;
					timer_rx_ctr = 4;
					bits_left_in_rx = rx_num_of_bits;
					rx_mask = 1;
				}
			}
			else    // rx_busy
			{
				if ( --timer_rx_ctr<=0 )
				{               // rcv
					timer_rx_ctr = 3;
					flag_in = get_rx_pin_status();
					if ( flag_in )
					{
						internal_rx_buffer |= rx_mask;
					}
					rx_mask <<= 1;
					if ( --bits_left_in_rx<=0 )
					{
						flag_rx_waiting_for_stop_bit = TRUE;
					}
				}
			}
		}
	}
}

static char _getchar ( void )
{
	char ch;

	do
	{
		while ( qout==qin )
		{
			idle();
		}
		ch = inbuf[qout] & 0xFF;
		if ( ++qout>=IN_BUF_SIZE )
		{
			qout = 0;
		}
	}
	while ( ch==0x0A || ch==0xC2 );
	return( ch );
}

static void _putchar ( char ch )
{
	// Se queda esperando que se mande el byte anterior
	while ( flag_tx_ready );
	user_tx_buffer = ch;
	// invoke_UART_transmit
	timer_tx_ctr = 3;

	// En la inicializacion se llena tx_num_of_bits con el valor 10 (hay que ver si anda esto)
	bits_left_in_tx = tx_num_of_bits;

	// Desplaza el caracter que se quiere enviar hacia la derecha
	// todo investigar para que sirve el 0x200

	// Pone en el buffer de salida el bit de start y de stop
	internal_tx_buffer = (user_tx_buffer<<1) | 0x200;

	flag_tx_ready = TRUE;
}

static void flush_input_buffer	( void )
{
	qin = 0;
	qout = 0;
}

static char kbhit				( void )
{
	return( qin!=qout );
}

static void turn_rx_on			( void )
{
	flag_rx_off = FALSE;
}

static void turn_rx_off			( void )
{
	flag_rx_off = TRUE;
}

/*==================[external functions definition]==========================*/

/**
 * Configura la UART por software
 * @param softUart periferico de la uart por software.
 * @param pinRx pin de recepcion.
 * @param pinTx pin de transmision.
 * @param baudRate velocidad de comunicacion. Debe ser 9600
 * @return TRUE
 */
bool_t  softUartConfig 		(softUartMap_t softUart, gpioMap_t pinRx, gpioMap_t pinTx, uint32_t baudRate)
{
	switch (softUart){
	case SOFT_UART_1:
		// En esta version el unico baudrate soportado es 9600
		if (baudRate != 9600){
			baudRate = 9600;
		}

		PinRx = pinRx;
		PinTx = pinTx;

		// Configura el pin TX como salida y RX como entrada
		gpioConfig(PinTx, GPIO_OUTPUT);//gpioConfig(pinTx, GPIO_OUTPUT);
		gpioConfig(PinRx, GPIO_INPUT);//gpioConfig(pinRx, GPIO_INPUT);

		// Inicializa los flags de comunicacion
		flag_tx_ready = FALSE;
		flag_rx_ready = FALSE;
		flag_rx_waiting_for_stop_bit = FALSE;
		flag_rx_off = FALSE;
		rx_num_of_bits = 10;
		tx_num_of_bits = 10;

		set_tx_pin_low();

		// todo: Calcular 35 como BAUDRATE*3
		// Para poder leer y escribir correctamente, se debe setear el periodo
		// de interrupcion de un timer a una velocidad 3 veces superior al baudrate.
		// En este caso, con baudrate a 9600 es: 1seg/(9600 * 3) = 35 uS aprox
		Timer_Init(SOFT_UART_TIMER, Timer_microsecondsToTicks(35), TimerCallback);
//		Timer_Init(SOFT_UART_TIMER, Timer_microsecondsToTicks(100000/(baudRate*3)), TimerCallback);
		break;
	}

	// Envia caracteres para limpiar la terminal
	_putchar(' ');
	_putchar(' ');
	_putchar(' ');
	_putchar('\n');
	_putchar('\r');

	return TRUE;
}

/**
 * Lee un byte de la uart por software
 * @param softUart periferico de la uart por software.
 * @param receivedByte puntero donde se va a guardar el caracter recibido.
 * @return TRUE si se leyo un dato, FALSE caso contrario.
 */
bool_t 	softUartReadByte 	(softUartMap_t softUart, uint8_t* receivedByte ){

	switch (softUart){
		case SOFT_UART_1:
			if (kbhit()){
				turn_rx_off();		//ver si anda bien
				*receivedByte = _getchar();
				flush_input_buffer();
				turn_rx_on();		//ver si anda bien

				return TRUE;
			} else {
				return FALSE;
			}
			break;
	}
}

/**
 * Envia un byte de la uart por software
 * @param softUart periferico de la uart por software.
 * @param byte dato a enviar por la uart por software.
 */
void 	softUartWriteByte	(softUartMap_t softUart, uint8_t byte ){
	switch (softUart){
		case SOFT_UART_1:
			_putchar(byte);
			break;
	}
}

/**
 *
 * Envia un string por la uart por software
 * @param softUart periferico de la uart por software.
 * @param str puntero a string a enviar.
 */
void 	softUartWriteString	(softUartMap_t softUart, char* str ){
	switch (softUart){
		case SOFT_UART_1:
			while(*str != 0){
				softUartWriteByte( softUart, (uint8_t)*str );
				str++;
			}
		break;
	}
}

/**
 *
 * Envia un printf (string formateado) por la uart por software
 * @param softUart periferico de la uart por software.
 * @param fmt string formateado como printf.
 */
void 	softUartPrintf 		(softUartMap_t softUart, const char *fmt, ...){
	register int *varg = (int *)(&fmt);
	char softUartBuffer [100];

		switch (softUart){
				case SOFT_UART_1:
					// Todo: Esta funcion esta en desarrollo

//					stdioSprintf(softUartBuffer, varg);
//					softUartWriteString(SOFT_UART_1, softUartBuffer);
					// Ejemplo de llamada en stdio
//					return print(0, varg);
				break;
			}
}

/*==================[end of file]============================================*/
