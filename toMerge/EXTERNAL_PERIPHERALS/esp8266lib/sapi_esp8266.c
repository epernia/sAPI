/**
 * @file ESP8266.cpp
 * @brief The implementation of class ESP8266. 
 * @author Wu Pengfei<pengfei.wu@itead.cc> 
 * @date 2015.02
 * 
 * @par Copyright:
 * Copyright (c) 2015 ITEAD Intelligent Systems Co., Ltd. \n\n
 * This program is free software; you can redistribute it and/or
 * modify it under the terms of the GNU General Public License as
 * published by the Free Software Foundation; either version 2 of
 * the License, or (at your option) any later version. \n\n
 * THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
 * IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
 * FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
 * AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
 * LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
 * OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN
 * THE SOFTWARE.
 */
#include "sapi_esp8266.h"
#include "sapi_esp8266_print.h"
#include "sapi_delay.h"
#include "sapi_tick.h"

#include <string.h>   // <= Biblioteca de manejo de Strings, ver:
// https://es.wikipedia.org/wiki/String.h
// http://www.alciro.org/alciro/Programacion-cpp-Builder_12/funciones-cadenas-caracteres-string.h_448.htm


/*
#define LOG_OUTPUT_DEBUG            (1)
#define LOG_OUTPUT_DEBUG_PREFIX     (1)

#define logDebug(arg)\
   do {\
      if (LOG_OUTPUT_DEBUG)\
      {\
         if (LOG_OUTPUT_DEBUG_PREFIX)\
         {\
            Serial.print("[LOG Debug: ");\
            Serial.print((const char*)__FILE__);\
            Serial.print(",");\
            Serial.print((unsigned int)__LINE__);\
            Serial.print(",");\
            Serial.print((const char*)__FUNCTION__);\
            Serial.print("] ");\
         }\
         Serial.print(arg);\
      }\
   } while(0)
*/

// ESP8266 Printer
static print_t esp8266_Print;


// ------------------------------ Public methods ------------------------------

void esp8266_UartSet( uartMap_t uart, uint32_t baudRate )
{
   esp8266_PrintSetUart( uart );
   esp8266_PrintConfigUart( uart, baudRate );
   esp8266_RxEmpty();
}

// STATIC INLINE 


// Verify ESP8266 whether live or not.
// Actually, this method will send command "AT" to ESP8266 and waiting for "OK".
bool_t esp8266_IsAlive( void )
{
   return esp8266_AT_Expect();
}

bool_t esp8266_Restart( void )
{
   tick_t start = 0;
   if( esp8266_ATRST_Expect() ){
      delay( 2000 );
      start = tickRead();
      while( tickRead() - start < 3000 ){
         if( esp8266_AT_Expect() ){
            delay( 1500 ); // Waiting for stable
            return TRUE;
         }
         delay( 100 );
      }
   }
   return FALSE;
}

void esp8266_GetVersion( char* version )
{
   esp8266_ATGMR_Expect( version );
}

bool_t esp8266_SetOprToStation( void )
{
   uint8_t mode;
   if( !esp8266_ATCWMODE_Query( &mode ) ){
      return FALSE;
   }
   if( mode == 1 ){
      return TRUE;
   } else{
      if( esp8266_ATCWMODE_Send(1) && esp8266_Restart() ){
         return TRUE;
      } else{
         return FALSE;
      }
   }
}

bool_t esp8266_SetOprToSoftAP( void )
{
   uint8_t mode;
   if( !esp8266_ATCWMODE_Query( &mode ) ){
      return FALSE;
   }
   if( mode == 2 ){
      return TRUE;
   } else {
      if( esp8266_ATCWMODE_Send(2) && esp8266_Restart() ){
         return TRUE;
      } else{
         return FALSE;
      }
   }
}

bool_t esp8266_SetOprToStationSoftAP( void )
{
   uint8_t mode;
   if( !esp8266_ATCWMODE_Query( &mode ) ){
      return FALSE;
   }
   if( mode == 3 ){
      return TRUE;
   } else {
      if( esp8266_ATCWMODE_Send(3) && esp8266_Restart() ){
         return TRUE;
      } else {
         return FALSE;
      }
   }
}

void esp8266_GetAPList( char* list )
{
   esp8266_ATCWLAP_Expect( list );
}

bool_t esp8266_JoinAP( char* ssid, char* pwd )
{
   return esp8266_ATCWJAP_Send( ssid, pwd );
}

bool_t esp8266_EnableClientDHCP( uint8_t mode, bool_t enabled )
{
   return esp8266_ATCWDHCP_Send( mode, enabled );
}

bool_t esp8266_LeaveAP( void )
{
   return esp8266_ATCWQAP_Expect();
}

bool_t esp8266_SetSoftAPParam( char* ssid, char* pwd, uint8_t chl, uint8_t ecn )
{
   return esp8266_ATCWSAP_Send( ssid, pwd, chl, ecn );
}

void esp8266_GetJoinedDeviceIP( char* list )
{
   esp8266_ATCWLIF_Expect( list );
}

void esp8266_GetIPStatus( char* list )
{
   esp8266_ATCIPSTATUS_Expect( list );
}

void esp8266_GetLocalIP( char* list )
{
   esp8266_ATCIFSR_Expect( list );
}

bool_t esp8266_EnableMUX( void )
{
   return esp8266_ATCIPMUX_Send( 1 );
}

bool_t esp8266_DisableMUX( void )
{
   return esp8266_ATCIPMUX_Send( 0 );
}

bool_t esp8266_CreateTCP( char* addr, uint32_t port )
{
   return esp8266_ATCIPSTART_SendSingle( "TCP", addr, port );
}

bool_t esp8266_ReleaseTCP( void )
{
   return esp8266_ATCIPCLOSE_ExpectSingle();
}

bool_t esp8266_RegisterUDP( char* addr, uint32_t port )
{
   return esp8266_ATCIPSTART_SendSingle( "UDP", addr, port );
}

bool_t esp8266_UnregisterUDP( void )
{
   return esp8266_ATCIPCLOSE_ExpectSingle();
}

bool_t esp8266_CreateTCP( uint8_t mux_id, char* addr, uint32_t port )
{
   return esp8266_ATCIPSTART_SendMultiple( mux_id, "TCP", addr, port );
}

bool_t esp8266_ReleaseTCP( uint8_t mux_id )
{
   return esp8266_ATCIPCLOSE_SendMulitple( mux_id );
}

bool_t esp8266_RegisterUDP( uint8_t mux_id, char* addr, uint32_t port )
{
   return esp8266_ATCIPSTART_SendMultiple( mux_id, "UDP", addr, port );
}

bool_t esp8266_UnregisterUDP( uint8_t mux_id )
{
   return esp8266_ATCIPCLOSE_SendMulitple( mux_id );
}

bool_t esp8266_SetTCPServerTimeout( uint32_t timeout )
{
   return esp8266_ATCIPSTO_Send( timeout );
}

bool_t esp8266_StartTCPServer( uint32_t port )
{
   // TODO: Ver si solo habría que poner:
   // return esp8266_ATCIPSERVER_Send( 1, port );
   if( esp8266_ATCIPSERVER_Send( 1, port ) ){ 
      return TRUE;
   }
   return FALSE;
}

bool_t esp8266_StopTCPServer( void )
{
   esp8266_ATCIPSERVER_Send( 0 );
   esp8266_Restart();
   return FALSE;
}

bool_t esp8266_StartServer( uint32_t port )
{
   return esp8266_StartTCPServer( port );
}

bool_t esp8266_StopServer (void )
{
   return esp8266_StopTCPServer();
}

   bool_t esp8266_Send( const uint8_t *buffer, uint32_t len )
   {
      return esp8266_ATCIPSEND_SendSingle( buffer, len );
   }

   bool_t esp8266_Send( uint8_t mux_id, const uint8_t *buffer, uint32_t len )
   {
      return esp8266_ATCIPSEND_SendMultiple( mux_id, buffer, len );
   }

   uint32_t esp8266_Recv( uint8_t *buffer, uint32_t buffer_size, uint32_t timeout )
   {
      return esp8266_RecvPkg( buffer, buffer_size, NULL, timeout, NULL );
   }

   uint32_t esp8266_Recv( uint8_t mux_id, uint8_t *buffer, uint32_t buffer_size, 
                          uint32_t timeout )
   {
      uint8_t id;
      uint32_t ret;
      ret = esp8266_RecvPkg( buffer, buffer_size, NULL, timeout, &id );
      if( ret > 0 && id == mux_id ){
         return ret;
      }
      return 0;
   }

   uint32_t esp8266_Recv( uint8_t *coming_mux_id, uint8_t *buffer, 
                          uint32_t buffer_size, uint32_t timeout )
   {
      return esp8266_RecvPkg( buffer, buffer_size, NULL, timeout, coming_mux_id );
   }

/*----------------------------------------------------------------------------*/
/* +IPD,<id>,<len>:<data> */
/* +IPD,<len>:<data> */

   uint32_t esp8266_RecvPkg( uint8_t *buffer, uint32_t buffer_size, 
                             uint32_t *data_len, uint32_t timeout, 
                             uint8_t *coming_mux_id )
   {
      char* data;
      char a;
      int32_t index_PIPDcomma = -1;
      int32_t index_colon = -1; /* : */
      int32_t index_comma = -1; /* , */
      int32_t len = -1;
      int8_t id = -1;
      bool_t has_data = FALSE;
      uint32_t ret;
      tick_t start;
      uint32_t i;

      if( buffer == NULL ){
         return 0;
      }

      start = tickRead();
      while( tickRead() - start < timeout ){
         if( m_puart->available() > 0 ){
            a = m_puart->read();
            data += a;
         }

         index_PIPDcomma = data.indexOf( "+IPD," );
         if( index_PIPDcomma != -1 ){
            index_colon = data.indexOf( ':', index_PIPDcomma + 5 );
            if( index_colon != -1 ){
               index_comma = data.indexOf( ',', index_PIPDcomma + 5 );
               /* +IPD,id,len:data */
               if( index_comma != -1 && index_comma < index_colon ){ 
                  id = data.substring( index_PIPDcomma + 5, index_comma ).toInt();
                  if(id < 0 || id > 4){
                     return 0;
                  }
                  len = data.substring( index_comma + 1, index_colon ).toInt();
                  if( len <= 0 ){
                     return 0;
                  }
               } else{ /* +IPD,len:data */
                  len = data.substring( index_PIPDcomma + 5, index_colon ).toInt();
                  if( len <= 0 ){
                     return 0;
                  }
               }
               has_data = TRUE;
               break;
            }
         }
      }

      if( has_data ){
         i = 0;
         ret = len > buffer_size ? buffer_size : len;
         start = tickRead();
         while( tickRead() - start < 3000 ){
            while( m_puart->available() > 0 && i < ret ){
               a = m_puart->read();
               buffer[i++] = a;
            }
            if (i == ret) {
               esp8266_RxEmpty();
               if( data_len ){
                  *data_len = len;    
               }
               if( index_comma != -1 && coming_mux_id ){
                  *coming_mux_id = id;
               }
               return ret;
            }
         }
      }
      return 0;
   }


// ----------------------------- Private methods ------------------------------

static void esp8266_RxEmpty( void ) 
{
   // Hace flush de la UART
   uint8_t receiveByte;      
   while( uartReadByte( UART_232, &receiveByte ) );
}

   static char* esp8266_RecvString( char* target, uint32_t timeout )
   {
      char* data;
      char a;
      tick_t start = tickRead();
      while( tickRead() - start < timeout ){
         while( m_puart->available() > 0 ){
            a = m_puart->read();
            if( a == '\0' ) continue;
            data += a;
         }
         if( data.indexOf(target) != -1 ){
            break;
         }   
      }
      return data;
   }

   static char* esp8266_RecvString(char* target1, char* target2, uint32_t timeout)
   {
      char* data;
      char a;
      tick_t start = tickRead();
      while( tickRead() - start < timeout ){
         while( m_puart->available() > 0 ){
            a = m_puart->read();
            if(a == '\0') continue;
            data += a;
         }
         if( data.indexOf( target1 ) != -1 ){
            break;
         } else if( data.indexOf( target2 ) != -1 ){
            break;
         }
      }
      return data;
   }

   static char* esp8266_RecvString( char* target1, char* target2, char* target3,
                                    uint32_t timeout )
   {
      char* data;
      char a;
      tick_t start = tickRead();
      while( tickRead() - start < timeout ){
         while( m_puart->available() > 0 ){
            a = m_puart->read();
            if( a == '\0') continue;
            data += a;
         }
         if( data.indexOf(target1) != -1 ){
            break;
         } else if( data.indexOf(target2) != -1 ){
            break;
         } else if( data.indexOf(target3) != -1 ){
            break;
         }
      }
      return data;
   }

   static bool_t esp8266_RecvFind(char* target, uint32_t timeout)
   {
      char* data_tmp;
      data_tmp = esp8266_RecvString( target, timeout );
      if( data_tmp.indexOf(target) != -1 ){
         return TRUE;
      }
      return FALSE;
   }

   static bool_t esp8266_RecvFindAndFilter( char* target, char* begin, char* end,
                                            char* data, uint32_t timeout )
   {
      char* data_tmp;
      data_tmp = esp8266_RecvString( target, timeout );
      if( data_tmp.indexOf( target ) != -1 ){
         int32_t index1 = data_tmp.indexOf( begin );
         int32_t index2 = data_tmp.indexOf( end) ;
         if( index1 != -1 && index2 != -1 ){
            index1 += begin.length();
            data = data_tmp.substring( index1, index2 );
            return TRUE;
         }
      }
      data = "";
      return FALSE;
   }


// ---------- AT Commands ----------

static bool_t esp8266_AT_Expect( void )
{
   esp8266_RxEmpty();
   esp8266_PrintlnString( "AT" );
   return esp8266_RecvFind( "OK" );
}

static bool_t esp8266_ATRST_Expect( void ) 
{
   esp8266_RxEmpty();
   esp8266_PrintlnString( "AT+RST" );
   return esp8266_RecvFind( "OK" );
}

static bool_t esp8266_ATGMR_Expect( char* version )
{
   esp8266_RxEmpty();
   esp8266_PrintlnString( "AT+GMR" );
   return esp8266_RecvFindAndFilter( "OK", "\r\r\n", "\r\n\r\nOK", version ); 
}

static bool_t esp8266_ATCWMODE_Query( uint8_t *mode ) 
{
   char str_mode[3] = { 0, 0 }; // TODO: Ver el tamaño, al parecer con 2 basta (vale de 1 a 3 + enter)
   bool_t ret;
   if( !mode ){
      return FALSE;
   }
   esp8266_RxEmpty();
   esp8266_PrintlnString( "AT+CWMODE?" );
   ret = esp8266_RecvFindAndFilter( "OK", "+CWMODE:", "\r\n\r\nOK", str_mode ); 
   if(ret) {
      *mode = (uint8_t)( atoi(str_mode) );
      return TRUE;
   } else{
      return FALSE;
   }
}

   static bool_t esp8266_ATCWMODE_Send( uint8_t mode )
   {
      char* data;
      esp8266_RxEmpty();
      esp8266_PrintString( "AT+CWMODE=" );
      esp8266_PrintlnInt( mode );

      data = esp8266_RecvString( "OK", "no change" );
      if( data.indexOf( "OK" ) != -1 || data.indexOf( "no change" ) != -1 ){
         return TRUE;
      }
      return FALSE;
   }

   static bool_t esp8266_ATCWJAP_Send( char* ssid, char* pwd )
   {
      char* data;
      esp8266_RxEmpty();
      esp8266_PrintString( "AT+CWJAP=\"" );
      esp8266_PrintString( ssid );
      esp8266_PrintString( "\",\"" );
      esp8266_PrintString( pwd );
      esp8266_PrintlnString( "\"" );

      data = esp8266_RecvString( "OK", "FAIL", 10000 );
      if( data.indexOf( "OK" ) != -1 ){
         return TRUE;
      }
      return FALSE;
   }

   static bool_t esp8266_ATCWDHCP_Send( uint8_t mode, bool_t enabled )
   {
      char* strEn = "0";
      if( enabled ){
         strEn = "1";
      }
      char* data;
      esp8266_RxEmpty();
      esp8266_PrintString( "AT+CWDHCP=" );
      esp8266_PrintString( strEn );
      esp8266_PrintString( "," );
      esp8266_PrintlnInt( mode );

      data = esp8266_RecvString( "OK", "FAIL", 10000 );
      if( data.indexOf( "OK" ) != -1 ){
         return TRUE;
      }
      return FALSE;
   }

static bool_t esp8266_ATCWLAP_Expect( char* list )
{
   esp8266_RxEmpty();
   esp8266_PrintlnString( "AT+CWLAP" );
   return esp8266_RecvFindAndFilter( "OK", "\r\r\n", "\r\n\r\nOK", list, 10000 );
}

static bool_t esp8266_ATCWQAP_Expect(void )
{
   esp8266_RxEmpty();
   esp8266_PrintlnString( "AT+CWQAP" );
   return esp8266_RecvFind( "OK" );
}

   static bool_t esp8266_ATCWSAP_Send( char* ssid, char* pwd, uint8_t chl, uint8_t ecn )
   {
      char* data;
      esp8266_RxEmpty();
      esp8266_PrintString( "AT+CWSAP=\"" );
      esp8266_PrintString( ssid );
      esp8266_PrintString( "\",\"" );
      esp8266_PrintString( pwd );
      esp8266_PrintString( "\"," );
      esp8266_PrintInt( chl );
      esp8266_PrintString( "," );
      esp8266_PrintlnInt( ecn );

      data = esp8266_RecvString( "OK", "ERROR", 5000 );
      if( data.indexOf( "OK" ) != -1 ){
         return TRUE;
      }
      return FALSE;
   }

static bool_t esp8266_ATCWLIF_Expect( char* list )
{
   esp8266_RxEmpty();
   esp8266_PrintlnString( "AT+CWLIF" );
   return esp8266_RecvFindAndFilter( "OK", "\r\r\n", "\r\n\r\nOK", list );
}

static bool_t esp8266_ATCIPSTATUS_Expect( char* list )
{
   delay(100);
   esp8266_RxEmpty();
   esp8266_PrintlnString( "AT+CIPSTATUS" );
   return esp8266_RecvFindAndFilter( "OK", "\r\r\n", "\r\n\r\nOK", list );
}

   static bool_t esp8266_ATCIPSTART_SendSingle( char* type, char* addr, uint32_t port )
   {
      char* data;
      esp8266_RxEmpty();
      esp8266_PrintString( "AT+CIPSTART=\"" );
      esp8266_PrintString( type );
      esp8266_PrintString( "\",\"" );
      esp8266_PrintString( addr );
      esp8266_PrintString( "\"," );
      esp8266_PrintlnInt( port );

      data = esp8266_RecvString( "OK", "ERROR", "ALREADY CONNECT", 10000 );
      if( data.indexOf( "OK" ) != -1 || data.indexOf( "ALREADY CONNECT" ) != -1 ){
         return TRUE;
      }
      return FALSE;
   }

   static bool_t esp8266_ATCIPSTART_SendMultiple( uint8_t mux_id, char* type, 
                                                  char* addr, uint32_t port )
   {
      char* data;
      esp8266_RxEmpty();
      esp8266_PrintString( "AT+CIPSTART=" );
      esp8266_PrintInt( mux_id );
      esp8266_PrintString( ",\"" );
      esp8266_PrintString( type );
      esp8266_PrintString( "\",\"" );
      esp8266_PrintString( addr );
      esp8266_PrintString( "\"," );
      esp8266_PrintlnInt( port );

      data = esp8266_RecvString( "OK", "ERROR", "ALREADY CONNECT", 10000 );
      if( data.indexOf( "OK" ) != -1 || data.indexOf( "ALREADY CONNECT" ) != -1 ){
         return TRUE;
      }
      return FALSE;
   }

   static bool_t esp8266_ATCIPSEND_SendSingle( const uint8_t *buffer, uint32_t len )
   {
      esp8266_RxEmpty();
      esp8266_PrintString( "AT+CIPSEND=" ) ;
      esp8266_PrintlnInt( len );
      if( esp8266_RecvFind( ">", 5000 ) ){
         esp8266_RxEmpty();
         for( uint32_t i = 0; i < len; i++ ){
            m_puart->write( buffer[i] );
         }
         return esp8266_RecvFind( "SEND OK", 10000 );
      }
      return FALSE;
   }

   static bool_t esp8266_ATCIPSEND_SendMultiple( uint8_t mux_id, const uint8_t *buffer, uint32_t len )
   {
      esp8266_RxEmpty();
      esp8266_PrintString( "AT+CIPSEND=" );
      esp8266_PrintInt(mux_id);
      esp8266_PrintString(",");
      esp8266_PrintlnInt(len);
      if( esp8266_RecvFind( ">", 5000 ) ) {
         esp8266_RxEmpty();
         for( uint32_t i = 0; i < len; i++ ){
            m_puart->write( buffer[i] );
         }
         return esp8266_RecvFind( "SEND OK", 10000 );
      }
      return FALSE;
   }

   static bool_t esp8266_ATCIPCLOSE_SendMulitple( uint8_t mux_id )
   {
      char* data;
      esp8266_RxEmpty();
      esp8266_PrintString( "AT+CIPCLOSE=" );
      esp8266_PrintlnInt( mux_id );

      data = esp8266_RecvString( "OK", "link is not", 5000 );
      if ( data.indexOf( "OK" ) != -1 || data.indexOf( "link is not" ) != -1 ){
         return TRUE;
      }
      return FALSE;
   }

static bool_t esp8266_ATCIPCLOSE_ExpectSingle( void )
{
   esp8266_RxEmpty();
   esp8266_PrintlnString( "AT+CIPCLOSE" );
   return esp8266_RecvFind( "OK", 5000 );
}

static bool_t esp8266_ATCIFSR_Expect( char* list )
{
   esp8266_RxEmpty();
   esp8266_PrintlnString( "AT+CIFSR" );
   return esp8266_RecvFindAndFilter( "OK", "\r\r\n", "\r\n\r\nOK", list );
}

   static bool_t esp8266_ATCIPMUX_Send( uint8_t mode )
   {
      char* data;
      esp8266_RxEmpty();
      esp8266_PrintString( "AT+CIPMUX=" );
      esp8266_PrintlnInt( mode );

      data = esp8266_RecvString( "OK", "Link is builded" );
      if( data.indexOf( "OK" ) != -1 ){
         return TRUE;
      }
      return FALSE;
   }

   static bool_t esp8266_ATCIPSERVER_Send( uint8_t mode, uint32_t port )
   {
      char* data;
      if(mode){
         esp8266_RxEmpty();
         esp8266_PrintString( "AT+CIPSERVER=1," );
         esp8266_PrintlnInt( port );

         data = esp8266_RecvString( "OK", "no change" );
         if( data.indexOf("OK") != -1 || data.indexOf("no change") != -1 ){
            return TRUE;
         }
         return FALSE;
      } else{
         esp8266_RxEmpty();
         esp8266_PrintlnString( "AT+CIPSERVER=0" );
         return esp8266_RecvFind( "\r\r\n" );
      }
   }

static bool_t esp8266_ATCIPSTO_Send( uint32_t timeout )
{
   esp8266_RxEmpty();
   esp8266_PrintString( "AT+CIPSTO=" );
   esp8266_PrintlnInt( timeout );
   return esp8266_RecvFind( "OK" );
}

