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

/**
 * \defgroup I2CDriver I²C driver
 *
 * \author Andreas Misje
 * \date 21.02.13
 *
 * \brief A simple I²C driver
 *
 * This is yet another I²C driver. It was written because Microchip's I²C
 * driver left most of the low-level work to the user. This driver attempts to
 * eliminate as much busy-waiting as possible. A state machine is implemented
 * in the I²C interrupt routine. Data transmission and reception is performed
 * in this state machine using a shared TX/RX data buffer.
 *
 * \anchor i2cAddrNote \note All slave device addresses will be left
 * bit-shifted inside this driver and the eight read/write bit will be
 * set/unset automatically. Please provide an unshifted address.
 */

/**
 * \addtogroup I2CDriver
 * @{
 *
 * \file
 */

#ifndef I2CDRIVER_H
#define I2CDRIVER_H

#include <stdbool.h>
#include <stddef.h>

/**
 * \brief I²C hardware module number
 *
 * For many PIC24s this number must be between 1 and 3.
 */
#define I2C_NO 1

/**
 * Software TX/RX buffer (including address/register bytes)
 */
#define I2C_TRX_BUFFER_SIZE 32

/**
 * \brief I²C driver states
 */
enum i2c_states
{
   /** Driver is idle */
   I2C_STATE_idle,
   /** Driver is sending start condition */
   I2C_STATE_sendingStart,
   /** Driver is sending data from the TX/RX buffer */
   I2C_STATE_dataTX,
   /** Driver is sending repeated start condition */
   I2C_STATE_sendingRestart,
   /** Driver is sending stop condition */
   I2C_STATE_sendingStop,
   /** Driver is receiving data and writing to the TX/RX buffer */
   I2C_STATE_dataRX,
   /** Driver is acknowledging reception */
   I2C_STATE_ack,
   /**
    * Driver is in an error state (see #i2c_error). The driver enters this
    * state if any of the following occurs:
    *    - Slave did not acknowledge
    *    - An unexpected I²C interrupt occurred
    *    - A bus collision was detected
    *    - TX/RX buffer could not fit provided data
    *
    * If #i2c_stayInErrorState is true, the I²C driver must be reset manually
    * by calling i2c_reset() if it has entered the error state.
    */
   I2C_STATE_error,
   /** Driver is temporarily disabled (by i2c_disable()) */
   I2C_STATE_disabled,
};

/**
 * \brief The current I²C driver state
 *
 * \sa i2c_states
 */
extern volatile enum i2c_states i2c_state;

/**
 * \brief Decides whether the I²C driver should remain in the error state once
 * it has entered it
 *
 * If the I²C driver enters the error state, it will stay in the error state
 * until reset by using i2c_reset() if this parameter is true. If not, the I²C
 * driver will immediately go to the idle state. The error will be stored in
 * i2c_error in either case.
 *
 * \sa i2c_states
 */
extern volatile bool i2c_stayInErrorState;

/** 
 * \brief I²C driver errors
 */
enum i2c_errors
{
   /** No error occurred */
   I2C_ERR_noError            =  0,
   /** An internal error occurred */
   I2C_ERR_internal           =  1,
   /** A function was called while the driver was in an error state */
   I2C_ERR_inErrorState       =  2,
   /** The driver is busy */
   I2C_ERR_busy               =  3,
   /** The TX/RX buffer cannot fit the provided data */
   I2C_ERR_TXBufferOverflow   =  4,
   /** The provided buffer cannot fit the received data waiting in the TX/RX
    * buffer */
   I2C_ERR_RXBufferOverflow   =  5,
   /** The slave did not acknowledge */
   I2C_ERR_slaveNACK          =  6,
   /** No bytes has been received */
   I2C_ERR_nothingReceived    =  7,
   /** A bus collision was detected */
   I2C_ERR_collisionDetected  =  8,
   /** The driver is currently disabled (by i2c_disable()) */
   I2C_ERR_disabled           =  9,
};

/**
 * \brief Contains the type of error that occured since last reset.
 *
 * \sa i2c_errors
 */
extern volatile enum i2c_errors i2c_error;

/**
 * \brief Enter idle state and reset error variable
 */
void i2c_reset();

/**
 * \brief Check if I²C driver is busy.
 *
 * \return true if I²C driver is not busy and not in an error state;
 */
bool i2c_busy();

/**
 * \brief Initialise the I²C driver
 *
 * \param brg baud rate generator reload value
 * \param enableSlewRateControl enable slew rate control
 * \param priority interrupt prioriy level. Must be between 0 and 7.
 */
void i2c_init( int brg, bool enableSlewRateControl, int priority );

/**
 * \brief Temporarily disable the I²C driver
 *
 * I²C interrupt will be disabled. All pending data transmission and reception
 * will be aborted.
 */
void i2c_disable();

/**
 * \brief Re-enable the I²C driver
 *
 * I²C interrupt will be re-enabled. The driver will start in the
 * ::I2C_STATE_idle state. Any previous data transmission or reception will
 * not be continued.
 */
void i2c_enable();

/**
 * \brief Sends a byte to a device on the I²C bus.
 *
 * \param address slave address (see
 * \ref i2cAddrNote "note about I²C addresses")
 * \param reg register in slave device
 * \param data data to write
 *
 * \retval I2C_ERR_noError no errors occurred. Start condition has been sent
 * and transmission has started
 * \retval I2C_ERR_inErrorState the I²C driver is in an error state
 * \retval I2C_ERR_busy the I²C driver is busy
 * \retval I2C_ERR_TXBufferOverflow the TX/RX buffer is not large enought to
 * contain \pn{address}, \pn{register} and \pn{data}
 * \retval I2C_ERR_disabled the I²C driver is disabled
 */
int i2c_putc( unsigned char address, unsigned char reg, unsigned char data );

/**
 * \brief Sends a series of bytes to a device on the I²C bus.
 *
 * \param address slave address (see
 * \ref i2cAddrNote "note about I²C addresses")
 * \param reg register in slave device
 * \param data data to write
 * \param len number of bytes to write. If 0, the data is assumed to be a
 * null-terminated string, and the length of the string is used instead.
 *
 * \retval I2C_ERR_noError no errors occurred. Start condition has been sent
 * and transmission has started
 * \retval I2C_ERR_inErrorState the I²C driver is in an error state
 * \retval I2C_ERR_busy the I²C driver is busy
 * \retval I2C_ERR_TXBufferOverflow the TX/RX buffer is not large enought to
 * contain \pn{address}, \pn{register} and \pn{data}
 * \retval I2C_ERR_disabled the I²C driver is disabled
 */
int i2c_puts( unsigned char address, unsigned char reg, unsigned char *data,
      size_t len );

/**
 * \brief Query a byte from a device on the I²C bus. If the reception
 * succeeded, the I²C driver will be in the idle state when the operation is
 * done.
 *
 * \param address slave address (see
 * \ref i2cAddrNote "note about I²C addresses")
 * \param reg register in slave device
 *
 * \retval I2C_ERR_noError no errors occurred. Start condition has been sent
 * \retval I2C_ERR_inErrorState the I²C driver is in an error state
 * \retval I2C_ERR_busy the I²C driver is busy
 * \retval I2C_ERR_RXBufferOverflow the TX/RX buffer is not large enough to
 * contain \pn{address}, \pn{register}
 * \retval I2C_ERR_disabled the I²C driver is disabled
 */
int i2c_getc( unsigned char address, unsigned char reg );

/**
 * \brief Query a series of bytes from a device on the I²C bus. If the
 * reception succeeded, the I²C driver will be in the idle state when the
 * operation is done.
 *
 * \param address slave address (see
 * \ref i2cAddrNote "note about I²C addresses")
 * \param reg register in slave device
 * \param len the number of bytes aniticipated. \a address must be large
 * enough to contain \a len bytes.
 *
 * \retval I2C_ERR_noError no errors occurred. Start condition has been sent
 * \retval I2C_ERR_inErrorState the I²C driver is in an error state
 * \retval I2C_ERR_busy the I²C driver is busy
 * \retval I2C_ERR_RXBufferOverflow the TX/RX buffer is not large enough to
 * contain \pn{address}, \pn{register} and number of expected bytes to receive
 * \retval I2C_ERR_disabled the I²C driver is disabled
 */
int i2c_gets( unsigned char address, unsigned char reg, size_t len );

/**
 * \brief Retrieve data captured from a previous i2c_getc()/i2c_gets() call.
 *
 * The I²C driver has to be in the idle state.
 *
 * \param data buffer to copy retrieved data into
 * \param len size of the data array. It must be large enough to hold the
 * number of bytes previously requested, otherwise I2C_ERR_RXBufferOverflow
 * will be returned.
 *
 * \retval I2C_ERR_noError no errors occurred. Start condition has been sent
 * \retval I2C_ERR_inErrorState the I²C driver is in an error state
 * \retval I2C_ERR_busy the I²C driver is busy
 * \retval I2C_ERR_nothingReceived no bytes was received
 * \retval I2C_ERR_RXBufferOverflow the provided \pn{data} buffer is not large
 * enough to contain the number of bytes received in the TX/RX buffer
 * \retval I2C_ERR_disabled the I²C driver is disabled
 */
int i2c_getData( unsigned char *data, size_t len );


#endif // I2CDRIVER_H

/**
 * @}
 */
