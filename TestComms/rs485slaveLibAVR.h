/*!
 * \brief A simple RS485 slave library for Atmel ARV controllers (ATmega8).
 *
 * This software can be included into user programs and it will handle a simple
 * RS485 protocoll. It allows up to 128 participants in a bus system to
 * communicate in master/slave style. All communication is following the
 * request/response pattern except for broadcast messages which are fire and
 * forget. This software provides only the slave side of the bus system.
 * It is intended for systems where a PC (Windows or whatever) acts as a master
 * and the AVRs assume the roles of the slaves.</p>
 *
 * <b>Communication format</b><br/>
 * The communication takes place in the 9N1 style that is nine data bits, no
 * parity and one stop bit. The ninth bit is never used (0) by the slave. The
 * master makes use of it (1) to indicate an address byte. Data is sent by the
 * master with bit nine set to zero (0).<p/>
 *
 * <b>Hardware use</b><br/>
 * Appart from ATmeag8's UART the port bit PD2 is used to switch the direction
 * of the RS485 transceiver. PD0 and PD1 are used by the UART already, where
 * PD0 is RXD and PD1 is TXD.</p>
 * Furthermore the multi-processor communication mode is (MPCM) is used to free
 * the slaves from filtering data bytes. The only receive address bytes which
 * the decide on whether they are addressed or not.<p/>
 * 
 * <b>About the software</b><br/>
 * Sending and receiving is done with interrut service routines (ISR).<p/>
 * Everything is built as a finite state machine. I hope this makes the whole
 * RS485 stack robust and reliable.<p/>
 * 
 * The state variables of the software are global/static. That means you cannot
 * run two instances of this software at the same time. (This would be possible
 * for example on a ATmega162 because this AVR has two UARTS. However my target
 * the ATmega8 has only one UART anyway.)<p/>
 *
 * <b>Request/response messages</b><br/>
 * See <code>../../common/rs485messages.h</code> and
 * <code>../../common/rs485messages.cpp</code> for details.<p/>
 *
 * <b>Makefile entries</b>
 * The following parameters should be defined in the makefile. Default values
 * are used if they are not defined. Those defaults may not match your
 * specifications therefore I recommend to define your parameters:
 * <table border="0">
 * <tr><th>Name</th><th>Explanation</th><th>Default value</th></tr>
 * <tr><td>F_CPU</td><td>Clock frequency in Hz.</td><td>8000000</td></tr>
 * <tr><td>BAUD</td><td>Baudrate, typical RS232 baud rates are ..., 9600, 19200,
 * 38400, ... Not all baud rates can be produced depending on the given CPU
 * clock. Use a baud crystal (e.g. 14.7456 MHz) to produce all standard baud
 * rates with no error. <em>The compiler will issue a warning in case CPU clock
 * and baud rate do not match (error rate to high).</td><td>38400</td></tr>
 * <tr><td>RS485_SWITCHING_DELAY</td><td>Delay time in miliseconds. Short delay
 * woll be performed after receiving the last byte from the request and
 * switching the bus transceiver to send mode.</td><td>5</td></tr>
 * </table>
 *
 * <b>Example Use</b><br/>
 * A typical program would look something like this:
 * <pre>
 * include "rs485libAVR.h"
 * ...Initialize microcontroller hardware (except for UART)...
 * initializeRS485( ...my address... );
 * while( true )
 * {
 *     if ( rs485MessageAvailable() )
 *     {
 *         RS485RequestMessage* l_request;
 *         consumeRS485Request( l_request );
 *         ....Do the requested processing...
 *         if( isResponseRequired( &l_request ) )
 *         {
 *            // Only done for request & response messages, that is
 *            // non-broadcast messages with RESP=1.
 *            RS485RequestMessage l_response;
 *            ....Populate the response message...
 *            respondViaRS485( &l_response );
 *         }
 *     }
 *     ... Do other (not RS485 related) things (but don't get stuck in infinte loop)...
 * }
 * </pre>
 *
 * \version {
 * 1.0.0 - 2012-05-29 - Initial version.
 * 2.0.0 - 2012-07-11 - Now processing the RESP flag in the address byte. Using common sources shared with Windows counter part.
 * 2.1.0 - 2012-07-21 - Uses error buffer from libAvrUtils.
 * 2.2.0 - 2012-09-16 - Now supports dual UARTs and allows for choosing the transceiver enable pin.
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

#ifndef RS485LIBAVR_H
#define RS485LIBAVR_H


#include "../../common/rs485messages.h"
#include "../../common/avrErrors.h"


/*!
 * \brief Initialize RS485 component.
 * Initializes the RS485 component by setting internal state variables and
 * configuring the UART hardware. This function is to be called first before
 * any other function of the RS485 component.
 * \param p_myAddress The address this AVR should listen to.
 *			0 < p_myAddress <= 128. 0 is not allowed because it is the broadcast
 *			address. Since typical maximum number of transceivers on the bus is
 *			32 and 128 we further limit the address range from 1 to 128.
 */
AvrRs485Error initializeRS485( const uint8_t p_myAddress );


/*!
 * \brief Check whether the master has sent a request message.
 * \returns true if the master has sent a request, false else.
 */
bool rs485MessageAvailable();


/*!
 * \brief Access the request sent by the master.
 * \param p_request A buffer provided by the user program. Request message data
 *			will be copied from the receive buffer into that out-parameter.
 *			Must not be NULL.
 * \returns OK if everything went fine, error code else.
 */
AvrRs485Error consumeRS485Request( RS485RequestMessage* p_request );


/*!
 * \brief Return this message to the master.
 * If the request message was no broadcast message then the user program has to
 * return a response.
 * \param p_response Response to be sent back to the master. Must not be NULL.
 * \returns OK if everything went fine, error code else.
 */
AvrRs485Error respondViaRS485( const RS485ResponseMessage* p_response );


/*!
 * \brief Check the last error that has occurred.
 * This function may return an error code != OK even though the functions
 * called by the user program returned OK. This is because an error may happen
 * in one of the ISRs. The ISRs set this error flag in case of failure.
 * \returns Error that has occurred most recently. (OK hopfully.)
 */
AvrRs485Error lastRS485Error();


/*!
 * \brief Check whether there were to many errors.
 * \returns true if to many errors had occurred, false else. Will be set to
 *		false once you start querying for errors with <code>lastRS485Error()</code>.
 */
bool toManyErrors();



#endif