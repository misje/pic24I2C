/* 
 * Copyright (C) 2013 Andreas Misje
 *
 * This program is free software: you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation, either version 3 of the License, or
 * (at your option) any later version.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this program. If not, see <http://www.gnu.org/licenses/>.
 */

#include <p24FJ256DA210.h>
#include "i2cDriver.h"
#include <string.h>

#define PASTE( a, b ) a ## b
#define CONCAT( a, b ) PASTE( a, b )

#define I2CCONbits ( CONCAT( I2C, CONCAT( I2C_NO, CONbits ) ) )
#define I2CSTATbits ( CONCAT( I2C, CONCAT( I2C_NO, STATbits ) ) )
#define I2CBRG ( CONCAT( I2C, CONCAT( I2C_NO, BRG ) ) )
#define I2CRCV ( CONCAT( I2C, CONCAT( I2C_NO, RCV ) ) )
#define I2CTRN ( CONCAT( I2C, CONCAT( I2C_NO, TRN ) ) )
#define I2CInterruptRoutine() ( \
      CONCAT( _MI2C, CONCAT( I2C_NO, Interrupt() ) ) )

#if I2C_NO == 1
#define enableI2CInterrupt() ( IEC1bits.MI2C1IE = true )
#define disableI2CInterrupt() ( IEC1bits.MI2C1IE = false )
#define setI2CInterruptPriority( pri ) ( IPC4bits.MI2C1IP = ( pri ) )
#define getI2CInterruptPriority() ( IPC4bits.MI2C1IP )
#define resetI2CInterruptFlag() ( IFS1bits.MI2C1IF = false )
#elif I2C_NO == 2
#define enableI2CInterrupt() ( IEC3bits.MI2C2IE = true )
#define disableI2CInterrupt() ( IEC3bits.MI2C2IE = false )
#define setI2CInterruptPriority( pri ) ( IPC12bits.MI2C2IP = ( pri ) )
#define getI2CInterruptPriority() ( IPC12bits.MI2C2IP )
#define resetI2CInterruptFlag() ( IFS3bits.MI2C2IF = false )
#elif I2C_NO == 3
#define enableI2CInterrupt() ( IEC5bits.MI2C3IE = true )
#define disableI2CInterrupt() ( IEC5bits.MI2C3IE = false )
#define setI2CInterruptPriority( pri ) ( IPC21bits.MI2C3IP = ( pri ) )
#define getI2CInterruptPriority() ( IPC21bits.MI2C3IP )
#define resetI2CInterruptFlag() ( IFS5bits.MI2C3IF = false )
#else
#error "Erroneous I2C_NO value"
#endif

volatile enum i2c_states i2c_state;
volatile enum i2c_errors i2c_error;
volatile bool i2c_stayInErrorState = true;

static volatile unsigned char TRXBuffer[ I2C_TRX_BUFFER_SIZE ];
static volatile size_t TRXBufferCurrPos;
static volatile size_t TRXBufferLen;

static volatile unsigned int numRXBytes;


#define waitUntilIdle() while ( \
      I2CCONbits.ACKEN || \
      I2CCONbits.RCEN  || \
      I2CCONbits.PEN   || \
      I2CCONbits.RSEN  || \
      I2CCONbits.SEN )

#define i2c_write( data ) do { \
   waitUntilIdle(); \
   I2CTRN = data; \
} while ( 0 )

#define sendStartCondition() do { \
   waitUntilIdle(); \
   I2CCONbits.SEN = true; \
   i2c_state = I2C_STATE_sendingStart; \
} while ( 0 )

#define sendReStartCondition() do { \
   waitUntilIdle(); \
   I2CCONbits.RSEN = true; \
   i2c_state = I2C_STATE_sendingRestart; \
} while ( 0 )

#define sendStopCondition() do { \
   waitUntilIdle(); \
   I2CCONbits.PEN = true; \
   i2c_state = I2C_STATE_sendingStop; \
} while ( 0 )

#define sendAck() do { \
   waitUntilIdle(); \
   I2CCONbits.ACKEN = true; \
   i2c_state = I2C_STATE_ack; \
} while ( 0 )

#define stopDueToError( error ) do { \
   i2c_error = error; \
   if ( i2c_stayInErrorState ) \
   i2c_state = I2C_STATE_error; \
   else \
      sendStopCondition(); \
} while ( 0 )


void i2c_reset()
{
   i2c_state = I2C_STATE_idle;
   i2c_error = I2C_ERR_noError;
   I2CSTATbits.BCL = 0;
}

bool i2c_busy()
{
   return i2c_state != I2C_STATE_idle && i2c_state != I2C_STATE_error;
}

void i2c_init( int brg, bool enableSlewRateControl, int priority )
{
   /* Set the baud rate: */
   I2CBRG = brg;
   /* Disable the I²C module: */
   I2CCONbits.I2CEN = false;
   I2CCONbits.DISSLW = !enableSlewRateControl;

   /* Reset interrupt flag: */
   resetI2CInterruptFlag();
   /* Enable interrupt: */
   enableI2CInterrupt();
   /* Set interrupt priority: */
   setI2CInterruptPriority( priority >= 0 && priority <= 7 ? priority : 1 );

   /* Set acknowledge data bit to ACK: */
   I2CCONbits.ACKDT = 0;
   /* Enable I²C module: */
   I2CCONbits.I2CEN = true;
   
   /* Empty RX hardware buffer: */
   int dummy = I2CRCV;
   (void)dummy;

   i2c_reset();
}

void i2c_disable()
{
   disableI2CInterrupt();
   I2CCONbits.I2CEN = false;
   i2c_state = I2C_STATE_disabled;
}

void i2c_enable()
{
   i2c_init( I2CBRG, !I2CCONbits.DISSLW, getI2CInterruptPriority() );
}

int i2c_putc( unsigned char address, unsigned char reg, unsigned char data )
{
   if ( i2c_error )
      return I2C_ERR_inErrorState;

   if ( i2c_state == I2C_STATE_disabled )
      return i2c_error = I2C_ERR_disabled;

   if ( i2c_state != I2C_STATE_idle )
      return i2c_error = I2C_ERR_busy;

   i2c_state = I2C_STATE_error;

   if ( I2C_TRX_BUFFER_SIZE < 3 )
      return i2c_error = I2C_ERR_TXBufferOverflow;

   TRXBufferLen = 3;
   TRXBufferCurrPos = 0;

   TRXBuffer[ 0 ] = ( address << 1 ) & 0xfe;
   TRXBuffer[ 1 ] = reg;
   TRXBuffer[ 2 ] = data;

   numRXBytes = 0;

   sendStartCondition();

   return I2C_ERR_noError;
}

int i2c_puts( unsigned char address, unsigned char reg, unsigned char *data,
      size_t len )
{
   if ( i2c_error )
      return I2C_ERR_inErrorState;

   if ( i2c_state == I2C_STATE_disabled )
      return i2c_error = I2C_ERR_disabled;

   if ( i2c_state != I2C_STATE_idle )
      return i2c_error = I2C_ERR_busy;

   i2c_state = I2C_STATE_error;

   if ( I2C_TRX_BUFFER_SIZE < 2 + len )
      return i2c_error = I2C_ERR_TXBufferOverflow;

   if ( !len )
      len = strlen( (char *)data );

   TRXBufferLen = 2 + len;
   TRXBufferCurrPos = 0;

   TRXBuffer[ 0 ] = ( address << 1 ) & 0xfe;
   TRXBuffer[ 1 ] = reg;

   size_t i;
   for ( i = 0; i < len; ++i )
      TRXBuffer[ i + 2 ] = data[ i ];

   numRXBytes = 0;

   sendStartCondition();

   return I2C_ERR_noError;
}

int i2c_getc( unsigned char address, unsigned char reg )
{
   if ( i2c_error )
      return I2C_ERR_inErrorState;

   if ( i2c_state == I2C_STATE_disabled )
      return i2c_error = I2C_ERR_disabled;

   if ( i2c_state != I2C_STATE_idle )
      return i2c_error = I2C_ERR_busy;

   i2c_state = I2C_STATE_error;

   if ( I2C_TRX_BUFFER_SIZE < 2 )
      return i2c_error = I2C_ERR_TXBufferOverflow;

   TRXBufferLen = 2;
   TRXBufferCurrPos = 0;

   TRXBuffer[ 0 ] = ( address << 1 ) & 0xfe;
   TRXBuffer[ 1 ] = reg;

   numRXBytes = 1;

   sendStartCondition();

   return I2C_ERR_noError;
}


int i2c_gets( unsigned char address, unsigned char reg, size_t len )
{
   if ( i2c_error )
      return I2C_ERR_inErrorState;

   if ( i2c_state == I2C_STATE_disabled )
      return i2c_error = I2C_ERR_disabled;

   if ( i2c_state != I2C_STATE_idle )
      return i2c_error = I2C_ERR_busy;

   i2c_state = I2C_STATE_error;

   if ( I2C_TRX_BUFFER_SIZE < 2 )
      return i2c_error = I2C_ERR_TXBufferOverflow;

   if ( I2C_TRX_BUFFER_SIZE < 1 + len )
      return i2c_error = I2C_ERR_RXBufferOverflow;

   TRXBufferLen = 2;
   TRXBufferCurrPos = 0;

   TRXBuffer[ 0 ] = ( address << 1 ) & 0xfe;
   TRXBuffer[ 1 ] = reg;

   numRXBytes = len;

   sendStartCondition();

   return I2C_ERR_noError;
}

int i2c_getData( unsigned char *data, size_t len )
{
   if ( i2c_error )
      return I2C_ERR_inErrorState;

   if ( i2c_state == I2C_STATE_disabled )
      return i2c_error = I2C_ERR_disabled;

   if ( i2c_state != I2C_STATE_idle )
      return i2c_error = I2C_ERR_busy;

   if ( !TRXBufferCurrPos )
      return i2c_error = I2C_ERR_nothingReceived;

   if ( len < TRXBufferCurrPos )
      return i2c_error = I2C_ERR_RXBufferOverflow;

   size_t i;
   for ( i = 0; i < len; ++i )
      data[ i ] = TRXBuffer[ i ];

   return I2C_ERR_noError;
}



void __attribute__(( interrupt, auto_psv )) I2CInterruptRoutine()
{
   if ( I2CSTATbits.BCL )
   {
      i2c_state = I2C_STATE_error;
      i2c_error = I2C_ERR_collisionDetected;

      resetI2CInterruptFlag();
      return;
   }

   switch ( i2c_state )
   {
      case I2C_STATE_idle:
         i2c_error = I2C_ERR_internal;
         i2c_state = I2C_STATE_error;
         break;

      case I2C_STATE_sendingStart:
      case I2C_STATE_sendingRestart:
         if ( TRXBufferCurrPos == TRXBufferLen )
         {
            stopDueToError( I2C_ERR_internal );
            break;
         }

         i2c_state = I2C_STATE_dataTX;
         i2c_write( TRXBuffer[ TRXBufferCurrPos++ ] );
         break;

      case I2C_STATE_dataTX:
         /* If slave did not acknowledge, abort: */
         if ( I2CSTATbits.ACKSTAT )
         {
            stopDueToError( I2C_ERR_slaveNACK );
            break;
         }

         if ( TRXBufferCurrPos == TRXBufferLen )
         {
            if ( numRXBytes )
            {
               /* If last operation was read: */
               if ( TRXBuffer[ 0 ] & 0x01 )
               {
                  TRXBufferCurrPos = 0;
                  i2c_state = I2C_STATE_dataRX;
                  waitUntilIdle();
                  I2CCONbits.RCEN = true;
                  break;
               }

               /* Re-use address in buffer, but set read bit: */
               TRXBufferCurrPos = 0;
               TRXBuffer[ 0 ] |= 0x01;
               /* Do not transmit register byte: */
               TRXBufferLen = 1;
               sendReStartCondition();
            }
            else
               sendStopCondition();
         }
         else
            i2c_write( TRXBuffer[ TRXBufferCurrPos++ ] );
         break;

      case I2C_STATE_sendingStop:
         i2c_state = I2C_STATE_idle;
         break;

      case I2C_STATE_dataRX:
         TRXBuffer[ TRXBufferCurrPos++ ] = I2CRCV;

         if ( TRXBufferCurrPos == numRXBytes )
            sendStopCondition();
         else
            sendAck();
         break;

      case I2C_STATE_ack:
         i2c_state = I2C_STATE_dataRX;
         waitUntilIdle();
         I2CCONbits.RCEN = true;
         break;

      case I2C_STATE_error:
      case I2C_STATE_disabled:
         break;
   }

   resetI2CInterruptFlag();
}
