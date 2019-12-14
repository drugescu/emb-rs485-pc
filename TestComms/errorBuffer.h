/*!
 * \brief A small stack for buffering error codes.
 *
 * Part of libAvrUtils - Helper routines for Atmel 8-bit AVR controllers.<p/>
 *
 * These routines and macros are meant for in-target testing of small units of
 * code.
 * The framework uses the UART (UART0 in case the AVR has multiple UARTs) for
 * "printing" the unit test results.
 * <p/>
 * Microcontroller (MCU) applications typically make use of interrupt service
 * routines (ISR).
 * At the same time the MCU executes an user program in parallel to the ISRs.
 * As a consequence there are multiple sources of errors.
 * Additionally user program and ISRs may produce same/different errors
 * repeatedly (that means multiple times before you check the error status).
 * Therefore I decided not to store the latest error in a scalar variable where
 * it can be overwritten and error information can be lost.
 * Instead I am using a small buffer so that no errors get lost.
 * Only if the buffer is full no more error codes can be "remembered".
 * This keeps the routines simple. Furthermore the idea is that earlier errors
 * are more interesting than later errors because typically the later errors
 * are just follow-up errors where as the early errors are the real root cause.
 * So we keep the old ones and forget the new once in case the buffer is full.
 * <p/>
 *
 * \version {
 * 1.0.0 - 2012-07-17 - Initial version.
 * }
 *
 * \author Holger Zahnleiter
 *
 * \copyright {
 * (c) 2012 Holger Zahnleiter
 * This program comes with ABSOLUTELY NO WARRANTY. 
 * This is free software, and you are welcome to redistribute it under certain
 * conditions.
 * The program and its source code are published under the GNU General Public
 * License (GPL).
 * See http://www.gnu.org/licenses/gpl-3.0.txt for details.
 * }
 *
 */

#ifndef ERROR_BUFFER_H
#define ERROR_BUFFER_H


#include <avr/io.h>
#include <stdbool.h>


#ifndef	MAX_ERR_ENTRIES
#warning "MAX_ERR_ENTRIES not defined, assuming 8."
/*!
 * \brief Buffers no more than 8 entries.
 */
#define MAX_ERR_ENTRIES 8
#endif


/*!
 * \brief Struct for storing error entries.
 */
typedef struct {
	uint8_t data[MAX_ERR_ENTRIES];
	uint8_t curr;
	bool overflow;
} ErrorBuffer;

/*!
 * \brief Initialize the buffer.
 * \param p_buffer Buffer to work with.
 */
void init( ErrorBuffer* p_buffer );

/*!
 * \brief Store an entry in the buffer.
 * No more entries are stored once the buffer is full.
 * Buffer is potentially used in ISRs.
 * Therefore interrupts are disabled to prevent corrucption of the buffer.
 * \param p_buffer Buffer to work with.
 * \param p_error Entry to store in the buffer.
 * \returns true if entry was stored, false else.
 */
bool push( ErrorBuffer* p_buffer, const uint8_t p_error );

/*!
 * \brief Remove the most current entry from the buffer and return it.
 * Buffer is potentially used in ISRs.
 * Therefore interrupts are disabled to prevent corrucption of the buffer.
 * \param p_buffer Buffer to work with.
 * \returns Most current entry, 0 (OK) when the buffer is empty.
 */
uint8_t pop( ErrorBuffer* p_buffer );

/*!
 * \brief Return the most current entry, nothing removed from buffer.
 * \param p_buffer Buffer to work with.
 * \returns Most current entry, 0 (OK) when the buffer is empty.
 */
uint8_t peek( const ErrorBuffer* p_buffer );


/*!
 * \brief Chech whether to many errors occurred.
 * \returns true if to many errors had occurred, false else.
 */
bool overflow( const ErrorBuffer* p_buffer );


#endif