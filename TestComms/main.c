/*!
 * \brief Test program for rs485slaveLibAVR.
 * \version {
 * 1.0.0 - 2012-05-29 - Initial version.
 * 2.0.0 - 2012-07-11 - Now processing the RESP flag in the address byte.
 * }
 * \author Holger Zahnleiter, holger@zahnleiter.org
 * \copyright {
 * (c) 2012 Holger Zahnleiter
 * This program comes with ABSOLUTELY NO WARRANTY.
 * This is free software, and you are welcome to redistribute it under certain conditions.
 * The program and its source code are published under the GNU General Public License (GPL).
 * See http://www.gnu.org/licenses/gpl-3.0.txt for details.
 * }
 */

#include "rs485slaveLibAVR.h"
#include <stddef.h>
#include <util/delay.h>
#include <avr/wdt.h>
#include <avr/interrupt.h>


void initialize_hardware( void )
{
	//---- Stop the watch-dog timer firing -------------------------------------
	wdt_disable();
	
	//---- Port B (LED at PB0 can be toggled via RS485) ------------------------
	//Set all pins of PORTB to output.
	DDRB = _BV(DDB0)|_BV(DDB1)|_BV(DDB2)|_BV(DDB3)|_BV(DDB4)|_BV(DDB5);
	//All pins low.
	PORTB = 0x00;
	
	//---- Initialize ADC ------------------------------------------------------
	// channel 0, left-justified result, use AVcc as voltage reference
	ADMUX = _BV(REFS0) | _BV(ADLAR); 
	// Enable ADC, use a prescaler of 16 (that makes 500k conversions @8MHz)
	// According to the datasheet this should be 50k to 200k, however, it works
	// fine in my experiments
	ADCSRA = _BV(ADEN) | _BV(ADPS2);

	//---- PWM (dimmable LED at PB1) -------------------------------------------
	//DDRB |= (1<<OC1A); // Port OC1A is output (connected with LED to be dimmed)
	TCCR1A = (1<<WGM10) | (1<<COM1A1); // PWM, phase correct, 8 bit
	TCCR1B = (1<<CS11) | (1<<CS10); // Prescaler 64 = Enable counter
	OCR1A = 127; // Duty cycle 50%
	
	//---- Hardware ready, allow interrupts ------------------------------------
	sei();
}


int main(void)
{
	initialize_hardware();
    initializeRS485( 1 );
	
	RS485RequestMessage l_request;
	RS485ResponseMessage l_response;
	while( true )
	{

		if ( rs485MessageAvailable() )
		{
			consumeRS485Request( &l_request );
			if ( 0 == l_request.address ) 
			{
				//---- Broadcast ----
				if ( 77 == l_request.command ) PORTB ^= (1<<PORTB0);
			}
			else 
			{
				//---- Directly addressed ----
				if ( 66 == l_request.command ) 
				{
					// Start conversion
					ADCSRA |= _BV(ADSC);
					// Wait while conversion in progress
					while( !(ADCSRA & _BV(ADIF)) );
					// Read analog value
					const uint8_t l_adcValue = ADCH;
					// Clear interrupt flag of ADC
					ADCSRA |= _BV(ADIF);
					l_response.numberOfReturnValues = 1;
					l_response.returnValue[0] = l_adcValue;
				}
				else if ( 77 == l_request.command ) 
				{
					if ( 1 == l_request.parameter[0] ) {
						if( 255 - OCR1A >= l_request.parameter[2] ) 
							OCR1A += l_request.parameter[2];
						else
							OCR1A = 255;
					}
					else if ( 2 == l_request.parameter[0] ) {
						if( OCR1A >= l_request.parameter[2] ) 
							OCR1A -= l_request.parameter[2];
						else 
							OCR1A = 0;
					}
					l_response.numberOfReturnValues = 0;
				}
				else 
				{
					l_response.numberOfReturnValues = 0;
				}
				if ( isResponseExpected( &l_request )  ) respondViaRS485( &l_response );
			}
		}
		
		AvrRs485Error l_lastError = lastRS485Error();
		if ( OK != l_lastError )
		{
			for ( uint8_t i = 0; i < l_lastError; i++ )
			{
				//Set pin 0 of port B to high.
				PORTB |= (1<<PORTB0);
				//Wait
				_delay_ms( 200 );
				//Set pin 0 of port B to low.
				PORTB &= ~(1<<PORTB0);
				//Wait
				_delay_ms( 200 );
			}
			//Wait
			_delay_ms( 1300 );
		}
		
	}
}
