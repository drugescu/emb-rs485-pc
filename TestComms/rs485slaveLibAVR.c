/*
 * <b>Technical details</b><br/>
 * A simple RS485 slave library for Atmel ARV controllers (ATmega8).<p/>
 *
 * The software builds a finite state machine. To describe it herein is quite
 * tedious. A picture is doing better that a thousand words here. Nevertheless
 * a typical application goes to the states the following way:
 * <pre>
 * NEED_INIT
 * |
 * | initializeRS485()
 * \/
 * AWAIT_REQUEST_FETCH_ADDRESS (ISR for receiving) <--------+
 * |                                                        |
 * | Address received                                       |
 * \/                                                       |
 * AWAIT_REQUEST_FETCH_COMMAND (ISR for receiving)          |
 * |                                                        |
 * | Command received                                       |
 * \/                                                       |
 * AWAIT_REQUEST_FETCH_NBPARAMS (ISR for receiving)         |
 * |                                                        |
 * | # of parameters received                               |
 * \/                                                       |
 * AWAIT_REQUEST_FETCH_PARAMS (ISR for receiving) <----+    |
 * |           parameter received                      |    |
 * +---------------------------------------------------+    |
 * | last parameter received                                |
 * \/                                                       |
 * DEMAND_PROCESSING                                        |
 * |                                                        |
 * | consumeRS485Request()                                  |
 * \/                                                       |
 * EXPECT_RESPONSE                                          |
 * |                                                        |
 * | respondViaRS485()                                      |
 * \/                                                       |
 * RETURN_SUBSEQUENT_RETVALS (ISR for receiving) <----+     |
 * |           return value sent                      |     |
 * +--------------------------------------------------+     |
 * | last return value sent                                 |
 * +--------------------------------------------------------+
 * </pre>
 *  
 * <b>Copyright statement</b><br/>
 * This software was created by Holger Zahnleiter on 2012-05-29.<br/>
 * Copyright 2012 Holger Zahnleiter<br/>
 * This program comes with ABSOLUTELY NO WARRANTY. 
 * This is free software, and you are welcome to redistribute it under certain conditions.
 * The program and its source code are published under the GNU General Public License (GPL).
 * See http://www.gnu.org/licenses/gpl-3.0.txt for details.
 */

#include "rs485slaveLibAVR.h"


// The makefile defines RS485_SWITCHING_DELAY (or at least it should).
#ifndef RS485_SWITCHING_DELAY
#warning "RS485_SWITCHING_DELAY not defined. Assuming 5 ms."
#define RS485_SWITCHING_DELAY 5
#endif

#include <stddef.h>
#include <avr/interrupt.h>

// The makefile defines F_CPU and BAUD (or at least should). (I typically use internal 8 MHz oszillator.)
#ifndef F_CPU
#warning "F_CPU not defined. Assuming 1 MHz."
#define F_CPU 1000000UL // Factory default for ATmega8 (should never apply because defined in makefile).
#endif
#ifndef BAUD
#warning "BAUD not defined. Assuming 38,400 Baud."
#define BAUD 38400UL // Highest possible baudrate at 8MHz without applying an external crystal.
#endif
#include <util/setbaud.h> // Calculates UBRR/USE_2X according to given F_CPU and BAUD.

#include <util/delay.h>

#include "errorBuffer.h"


/******************************************************************************
 * Port/pin for controlling RS-485 transceiver direction.
 ******************************************************************************/

/*!
 * \brief By default use Port D for controlling RS-485 transceiver direction.
 */
#ifndef RS485_DIR_DDR
#warning "RS485_DIR_DDR not set. Assuming DDRD."
#define RS485_DIR_DDR DDRD
#endif

#ifndef RS485_DIR_DDPIN
#warning "RS485_DIR_DDPIN not set. Assuming DDD2."
#define RS485_DIR_DDPIN DDD2
#endif

#ifndef RS485_DIR_PORT
#warning "RS485_DIR_PORT not set. Assuming PORTD."
#define RS485_DIR_PORT PORTD
#endif

#ifndef RS485_DIR_PIN
#warning "RS485_DIR_PIN not set. Assuming PD2."
#define RS485_DIR_PIN PD2
#endif


/******************************************************************************
 * Support for various AVR variants.
 ******************************************************************************/

// Further models with dual UART might be added here.
#if		__AVR_ATmega162__ \
	||	__AVR_ATmega128__ \
	||	__AVR_ATmega164__ || __AVR_ATmega324__ || __AVR_ATmega644__ || __AVR_ATmega1284P__ \
	||	__AVR_ATmega640__ || __AVR_ATmega1280__ || __AVR_ATmega2560__ \
	||	__AVR_ATmega1281__ || __AVR_ATmega2561__

#if RS485_USE_UART == 1
#define RS485_UBRRH UBRR1H
#define RS485_UBRRL UBRR1L
#define RS485_U2X U2X1
#define RS485_UCSRA UCSR1A
#define RS485_UCSRB UCSR1B
#define RS485_UCSRC UCSR1C
#define RS485_TXEN TXEN1
#define RS485_RXEN RXEN1
#define RS485_UCSZ2 UCSZ12
#define RS485_UCSZ1 UCSZ11
#define RS485_UCSZ0 UCSZ10
#define RS485_UDRE UDRE1
#define RS485_UDR UDR1
#define RS485_MPCM MPCM1
#define RS485_TXC TXC1
#define RS485_RXCIE RXCIE1
#define RS485_TXCIE TXCIE1
#define RS485_TXB8 TXB81
#define RS485_RXB8 RXB81
#if __AVR_ATmega162__
#define RS485_RXC_ISR_VECT USART1_RXC_vect
#define RS485_TXC_ISR_VECT USART1_TXC_vect
#else
#define RS485_RXC_ISR_VECT USART1_RX_vect
#define RS485_TXC_ISR_VECT USART1_TX_vect
#endif
#else // RS485_USE_UART == 0 (default)
#define RS485_UBRRH UBRR0H
#define RS485_UBRRL UBRR0L
#define RS485_U2X U2X0
#define RS485_UCSRA UCSR0A
#define RS485_UCSRB UCSR0B
#define RS485_UCSRC UCSR0C
#define RS485_TXEN TXEN0
#define RS485_RXEN RXEN0
#define RS485_UCSZ2 UCSZ02
#define RS485_UCSZ1 UCSZ01
#define RS485_UCSZ0 UCSZ00
#define RS485_UDRE UDRE0
#define RS485_UDR UDR0
#define RS485_MPCM MPCM0
#define RS485_TXC TXC0
#define RS485_RXCIE RXCIE0
#define RS485_TXCIE TXCIE0
#define RS485_TXB8 TXB80
#define RS485_RXB8 RXB80
#if __AVR_ATmega162__
#define RS485_RXC_ISR_VECT USART0_RXC_vect
#define RS485_TXC_ISR_VECT USART0_TXC_vect
#else
#define RS485_RXC_ISR_VECT USART0_RX_vect
#define RS485_TXC_ISR_VECT USART0_TX_vect
#endif
#endif

#else

#define RS485_UBRRH UBRRH
#define RS485_UBRRL UBRRL
#define RS485_U2X U2X
#define RS485_UCSRA UCSRA
#define RS485_UCSRB UCSRB
#define RS485_UCSRC UCSRC
#define RS485_TXEN TXEN
#define RS485_RXEN RXEN
#define RS485_URSEL URSEL
#define RS485_UCSZ2 UCSZ2
#define RS485_UCSZ1 UCSZ1
#define RS485_UCSZ0 UCSZ0
#define RS485_UDRE UDRE
#define RS485_UDR UDR
#define RS485_MPCM MPCM
#define RS485_TXC TXC
#define RS485_RXCIE RXCIE
#define RS485_TXCIE TXCIE
#define RS485_TXB8 TXB8
#define RS485_RXB8 RXB8
#if __AVR_ATmega16__ || __AVR_ATmega32__ || __AVR_ATmega323__ || __AVR_ATmega8__
#define RS485_RXC_ISR_VECT USART_RXC_vect
#define RS485_TXC_ISR_VECT USART_TXC_vect
#else
#define RS485_RXC_ISR_VECT USART_RX_vect
#define RS485_TXC_ISR_VECT USART_TX_vect
#endif

#endif


/******************************************************************************
 * Global variables and program status.
 ******************************************************************************/

typedef enum {
	NEED_INIT = 0,					// Component not initialised yet.
	AWAIT_REQUEST_FETCH_ADDRESS,	// First "real" state, RS485 slave waiting for request from master.
	AWAIT_REQUEST_FETCH_COMMAND,    // After the address we expect the parameter bytes. First one is "command".
	AWAIT_REQUEST_FETCH_NBPARAMS,	// Now read the number of parameters (might be 0).
	AWAIT_REQUEST_FETCH_PARAMS,		// Finally read as many parameter bytes as signaled by the master.
	DEMAND_PROCESSING,				// Now the request has to be processed.
	EXPECT_RESPONSE,    			// Request from master received. Master expects response.
	RETURN_SUBSEQUENT_RETVALS		// Subsequent response bytes.
} State;

/*
 * The state machines sate is helt here. There are two exceptions. In the states
 * AWAIT_REQUEST_FETCH_PARAMS and RETURN_SUBSEQUENT_RETVALS further state
 * variables are used (for holding a sub-state). These are s_currReqByteIndex
 * and s_currRespByteIndex. Both variables are used as byte counters. One counts
 * the incomming request parameters the other the outgouing return values.
 * The state machine stays in the states AWAIT_REQUEST_FETCH_PARAMS and 
 * RETURN_SUBSEQUENT_RETVALS while counting and making sure that the next state
 * is achieved when the right number of bytes has been received or sent
 * respectively.
 */
static State s_state = NEED_INIT;


static RS485RequestMessage s_receivedMessage;


static uint8_t s_myAddress = 0;
static uint8_t s_currReqByteIndex = 0;


static ErrorBuffer s_lastError;

static RS485ResponseMessage s_responseMessage;
static uint8_t s_currRespByteIndex = 0;


/*
 * Call this method to report an error.
 */
AvrRs485Error status( const AvrRs485Error p_error )
{
	if( OK != p_error ) push( &s_lastError, p_error );
	return p_error;
}


AvrRs485Error initializeRS485( const uint8_t p_myAddress )
{
	if ( 0 == p_myAddress ) return status( ZERO_ADDRESS_NOT_ALLOWED );
	if ( RESPONSE_IS_EXPECTED <=  p_myAddress ) return status( ADDRESS_TO_BIG );
	
	cli(); // Don't allow interrupts during initialization.
	
	//---- Port D is used to select RS485 transceiver direction ----------------
	// Set pin PD2 of PORTD to output. (This is the default and can be
	// overridden by defining the following macro names accordingly.
	RS485_DIR_DDR |= _BV(RS485_DIR_DDPIN);
	RS485_DIR_PORT &= ~(1<<RS485_DIR_PIN); // Set RS485 transceiver to receive.
	
	//---- Initialize UART for 8N1 @9600 baud ----------------------------------
	RS485_UBRRH = UBRRH_VALUE;
	RS485_UBRRL = UBRRL_VALUE;
#if USE_2X
	RS485_UCSRA |= (1<<RS485_U2X); // U2X mode required to achieve improved accuracy
#else
	RS485_UCSRA &= ~(1<<RS485_U2X); // No U2X mode required
#endif
	// This statement is translated into a read-modify-write instruction.
	// This might cause problems according to the ATmega8 data sheet (section
	// "Muti-processor Communication Mode").
	// It is stated that using CLI or SBI on MCPM might accidentally clear
	// the RS485_C flag since it shares the same I/O location as MPCM.
	// 	
	// In this case we want to clear RS485_C anyway. But in other places in this
	// code this might cause trouble and need to be considered. Use the compiler
	// switch "-save-temps" to produce an assembler file. In that file you can
	// verify that nor CLI/SBI has been generated.
	RS485_UCSRA |= (1<<RS485_TXC); // Initially clear TXC, no send has been done so far. (Clearing is achieved by setting it to 1 according to datasheet!!!)	
	
	RS485_UCSRA |= (1<<RS485_MPCM); // Multiprocessor mode, bit 9 determines address byte.
	RS485_UCSRB = (1<<RS485_TXEN)  | (1<<RS485_RXEN)  | (1<<RS485_RXCIE) | (1<<RS485_TXCIE) | (1<<RS485_UCSZ2);  // UART TX and RX on = send and receive, receive and transmit triggering interrupts
	RS485_UCSRB &= ~(1<<RS485_TXB8); // Clear 9th data bit. We only return data no address.
	// Asynchronus 9N1.
#if		__AVR_ATmega162__ \
	||	__AVR_ATmega128__ \
	||	__AVR_ATmega164__ || __AVR_ATmega324__ || __AVR_ATmega644__ || __AVR_ATmega1284P__ \
	||	__AVR_ATmega640__ || __AVR_ATmega1280__ || __AVR_ATmega2560__ \
	||	__AVR_ATmega1281__ || __AVR_ATmega2561__
	RS485_UCSRC = (1<<RS485_UCSZ1) | (1<<RS485_UCSZ0); 
#else
	// On ATmega8 etc. UCSRC shares the same memory location as UBRRH.
	// Setting URSEL indicates that we want to access UCSRC.
	RS485_UCSRC = (1<<RS485_URSEL) | (1<<RS485_UCSZ1) | (1<<RS485_UCSZ0); 
#endif
		
	//---- Initialize remaining component state variables ----------------------
	s_myAddress = p_myAddress;
	s_state = AWAIT_REQUEST_FETCH_ADDRESS;
	init( &s_lastError );
	
	sei(); // System initialized, allow interrupts.
	
	return OK;
}


bool rs485MessageAvailable()
{
	return DEMAND_PROCESSING == s_state;
}


AvrRs485Error consumeRS485Request( RS485RequestMessage* p_request )
{
	if ( !rs485MessageAvailable() ) return status( NO_REQUEST_AVAILABLE );
	if ( NULL == p_request ) return status( NULL_REQUEST_BUFFER );
	p_request->address = s_receivedMessage.address;
	p_request->command = s_receivedMessage.command;
	p_request->numberOfParameters = s_receivedMessage.numberOfParameters;
	for ( uint8_t i = 0; i < s_receivedMessage.numberOfParameters; i++ )
	{
		p_request->parameter[i] = s_receivedMessage.parameter[i];
	}
	s_state = isResponseExpected( &s_receivedMessage ) ? EXPECT_RESPONSE : AWAIT_REQUEST_FETCH_ADDRESS;
	
	return OK;
}


/*
 * I have reproducibly experienced that the master (Windows PC) has gone into
 * timeout because it was not able to read the slaves response. Therefore the
 * slave waits a very short ammount of time (5ms) to allow the master to 
 * switch to receiving mode before the slave himself turns the bus around.
 *
 * I have the following explanation from which I don't know if it's true but I
 * have no better idea:
 * - Master and slaves independently switch their bus transceivers into send
 *   and receiving mode respectively. Their is no coordination between master
 *   and slaves.
 * - For the slaves this is not relevant I think. They are (in my case)
 *   microcontrollers. That means that the only program running is the user
 *   program. Especially there is no operating system and no multiple
 *   processes/threads. Interupt service routines (ISR) are typically short
 *   and fast. The whole system can verry quickly switch from receive to
 *   transmit and vice versa.
 * - The master is (in my case) a Windows PC. Appart from the user program manny
 *   additional programs (services/deamons) are executed concurrently. That
 *   means that once the master has done sending other processes/threads are
 *   likely to be executed in between. So switching to transmit mode might be
 *   deferred for some time.
 * - I don't think that the RS485 transceivers need to much time to switch as
 *   you can read in some fora and web sites on the internet.
 * - This may also be an explanation for why the PC software does not need a
 *   delay and does not need to wait for the MCU.
 * - 5ms may not work in all cases. This is not really deterministic since we
 *   just make an assumtion but do not really coordinate switching of directions
 *   of all involved systems. A possibility would be to use the RTS signal from
 *   the master and to send that singnal (inverted of course) to all slaves.
 *   That would be deterministic but that would cause more effort and it would
 *   make the RS485 a three wire bus. So far I can live with the 5ms delay.
 */
AvrRs485Error respondViaRS485( const RS485ResponseMessage* p_response )
{
	if ( EXPECT_RESPONSE != s_state ) return status( NO_RESPONSE_EXPECTED );
	_delay_ms( RS485_SWITCHING_DELAY ); // Give the master a chance to switch from send to receive before we turn the bus around.
	RS485_DIR_PORT |= (1<<RS485_DIR_PIN); // Set RS485 transceiver to send.
	s_responseMessage.numberOfReturnValues = p_response->numberOfReturnValues;
	for ( uint8_t i = 0; i < s_responseMessage.numberOfReturnValues; i++ ) 
	{
		s_responseMessage.returnValue[i] = p_response->returnValue[i];
	}
	s_state = RETURN_SUBSEQUENT_RETVALS;
	s_currRespByteIndex = 0;
	while( !(RS485_UCSRA & (1<<RS485_UDRE)) ); // Wait until hardware send buffer ready.	
	RS485_UDR = s_responseMessage.numberOfReturnValues; // Send first byte via RS232. Subsequent bytes are sent by ISR (when needed).
	
	return OK;
}


AvrRs485Error lastRS485Error()
{
	return pop( &s_lastError );
}

bool toManyErrors()
{
	return overflow( &s_lastError );
}


ISR(RS485_RXC_ISR_VECT)
{
	RS485_UCSRB &= ~(1<<RS485_RXCIE); // Temporarily disallow interrupts on receive.

	// According to the ATmega8 datasheet the ninth bit needs to be read before
	// UDR.
	// We check the ninth bit to see if the data just received is an address
	// byte. If so, the state machine goes into the state that handels the
	// address byte. This is to ensure that the state machine assumes a defined
	// state should it had gone wild. Cause for assuming a wrong state could be
	// an error in this library or erroneous data sent (error in the RS485
	// master code). If the state machine is without error then the state will
	// be this one anyway.
	if( (AWAIT_REQUEST_FETCH_ADDRESS != s_state) && (RS485_UCSRB & (1<<RS485_RXB8)) )
	{
		s_state = AWAIT_REQUEST_FETCH_ADDRESS;
		status( STATE_MACHINE_RESET );
	}
	
	const uint8_t l_currByte = RS485_UDR; // Read data from hardware receive buffer.
	
	switch ( s_state ) {
		case AWAIT_REQUEST_FETCH_ADDRESS:
			//if ( 0 != l_currByte && s_myAddress != l_currByte ) break;
			if ( 0 != l_currByte && s_myAddress != (l_currByte & ~RESPONSE_IS_EXPECTED) ) break;
			RS485_UCSRA &= ~(1<<RS485_MPCM); // Multi-processor mode off.
			s_receivedMessage.address = l_currByte;
			s_state = AWAIT_REQUEST_FETCH_COMMAND;
			break;
		case AWAIT_REQUEST_FETCH_COMMAND:
			s_receivedMessage.command = l_currByte;
			s_state = AWAIT_REQUEST_FETCH_NBPARAMS;
			break;
		case AWAIT_REQUEST_FETCH_NBPARAMS:
			s_receivedMessage.numberOfParameters = l_currByte;
			if( 0 == l_currByte ) {
				s_state = DEMAND_PROCESSING;
				RS485_UCSRA |= (1<<RS485_MPCM); // Multi-processor mode on.
			}
			else {
				s_currReqByteIndex = 0;
				s_state = AWAIT_REQUEST_FETCH_PARAMS;
			}
			break;
		case AWAIT_REQUEST_FETCH_PARAMS:
			s_receivedMessage.parameter[s_currReqByteIndex++] = l_currByte;
			if ( s_currReqByteIndex < s_receivedMessage.numberOfParameters ) break;
			s_state = DEMAND_PROCESSING;
			RS485_UCSRA |= (1<<RS485_MPCM); // Multi-processor mode on.
			break;
		case DEMAND_PROCESSING:
			status( REQUEST_DROPPED );
			break;
		default:
			RS485_UCSRA |= (1<<RS485_MPCM); // Multi-processor mode on.
			s_state = AWAIT_REQUEST_FETCH_ADDRESS;
			status( INVALID_STATE_WHEN_RECEIVING );
			break;
	}
	
	RS485_UCSRB |= (1<<RS485_RXCIE); // Allow interrupts on receive again.
}


ISR(RS485_TXC_ISR_VECT)
{
	RS485_UCSRB &=~ (1<<RS485_TXCIE); // Temporarily disallow interrupts on send.

	switch ( s_state ) {
		case RETURN_SUBSEQUENT_RETVALS:
			if ( s_currRespByteIndex < s_responseMessage.numberOfReturnValues ) {
				while( !(RS485_UCSRA & (1<<RS485_UDRE)) ); // Wait until hardware send buffer ready.	
				RS485_UDR = s_responseMessage.returnValue[s_currRespByteIndex++]; // Send subsequent byte via RS485.
			}
			else {
				RS485_DIR_PORT &= ~(1<<RS485_DIR_PIN); // Set RS485 transceiver to receive.
				s_state = AWAIT_REQUEST_FETCH_ADDRESS;
			}
			break;
		default:
			RS485_DIR_PORT &= ~(1<<RS485_DIR_PIN); // Set RS485 transceiver to receive.
			RS485_UCSRA |= (1<<RS485_MPCM); // Multi-processor mode on.
			s_state = AWAIT_REQUEST_FETCH_ADDRESS;
			status( INVALID_STATE_WHEN_SENDING );
			break;
	}

	RS485_UCSRB |= (1<<RS485_TXCIE); // Allow interrupts on send again.
}

