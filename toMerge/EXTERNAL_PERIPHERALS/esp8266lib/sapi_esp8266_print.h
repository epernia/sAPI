/* Copyright 2017, Eric Pernia.
 * All rights reserved.
 *
 * Author: Eng. Eric Pernia.
 * Date: 2017-04-17
 */

#ifndef _esp8266__PRINT_H_
#define _esp8266__PRINT_H_

/*==================[inclusions]=============================================*/

#include "sapi_print.h"

/*==================[cplusplus]==============================================*/

#ifdef __cplusplus
extern "C" {
#endif

/*==================[macros]=================================================*/

// Initialize
#define esp8266_PrintSetUart(uart);                 printSetUart(&(esp8266_Print),(uart));
#define esp8266_PrintConfigUart(uart,baudRate);     printConfigUart(&(esp8266_Print),(uart),(baudRate));

// Print String
#define esp8266_PrintString(string);                printString((esp8266_Print),(string));
#define esp8266_PrintEnter();                       printEnter(esp8266_Print);
#define esp8266_PrintlnString(string);              printlnString((esp8266_Print),(string));

// Print Integer
#define esp8266_PrintIntFormat(number,format);      printIntFormat((esp8266_Print),(number),(format));
#define esp8266_PrintUIntFormat(number,format);     printUIntFormat((esp8266_Print),(number),(format));
#define esp8266_PrintlnIntFormat(number,format);    printlnIntFormat((esp8266_Print),(number),(format));
#define esp8266_PrintlnUIntFormat(number,format);   printlnUIntFormat((esp8266_Print),(number),(format));
#define esp8266_PrintInt(number);                   printInt((esp8266_Print),(number));
#define esp8266_PrintUInt(number);                  printUInt((esp8266_Print),(number));
#define esp8266_PrintlnInt(number);                 printlnInt((esp8266_Print),(number));
#define esp8266_PrintlnUInt(number);                printlnUInt((esp8266_Print),(number));
#define esp8266_PrintHex(number,bitSize);           printHex((esp8266_Print),(number),(bitSize));
#define esp8266_PrintlnHex(number,bitSize);         printlnHex((esp8266_Print),(number),(bitSize));

/*==================[typedef]================================================*/

/*==================[external data declaration]==============================*/

/*==================[external functions declaration]=========================*/

/*==================[cplusplus]==============================================*/

#ifdef __cplusplus
}
#endif

/*==================[end of file]============================================*/
#endif /* #ifndef _esp8266__PRINT_H_ */
