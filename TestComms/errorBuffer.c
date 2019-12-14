/*
 * (c) 2012 Holger Zahnleiter
 * This program comes with ABSOLUTELY NO WARRANTY. 
 * This is free software, and you are welcome to redistribute it under certain
 * conditions.
 * The program and its source code are published under the GNU General Public
 * License (GPL).
 * See http://www.gnu.org/licenses/gpl-3.0.txt for details.
 *
 */

#include "errorBuffer.h"

#include <avr/interrupt.h>


void init( ErrorBuffer* p_buffer )
{
	p_buffer->curr = 0;
	p_buffer->overflow = false;
}


bool push( ErrorBuffer* p_buffer, const uint8_t p_error )
{
	cli();
	if ( p_buffer->curr >= MAX_ERR_ENTRIES ) {
		p_buffer->overflow = true;
		sei();
		return false;
	}
	p_buffer->data[p_buffer->curr++] = p_error;
	sei();
	return true;
}


uint8_t pop( ErrorBuffer* p_buffer )
{
	if ( 0 == p_buffer->curr ) return 0;
	cli();
	p_buffer->overflow = false;
	uint8_t l_lastErr = p_buffer->data[--p_buffer->curr];
	sei();
	return l_lastErr;
}


uint8_t peek( const ErrorBuffer* p_buffer )
{
	if ( 0 == p_buffer->curr ) return 0;
	return p_buffer->data[p_buffer->curr - 1];
}


bool overflow( const ErrorBuffer* p_buffer )
{
	return p_buffer->overflow;
}