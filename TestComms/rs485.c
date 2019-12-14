/*
 * rs485.c
 *
 * Created: 04.10.2019 22:12:29
 *  Author: Asus
 */ 

#define F_CPU 16000000UL
#include <avr/io.h>
#include <avr/interrupt.h>
#include <util/delay.h>
#include <stdlib.h>
#include "rs485.h"

#define BAUD  4800UL


void
rs485_init(long int baudrate)
{
	/* Set baud rate*/
	int ubr = (16000000UL * 1.0) / (16.0 * baudrate) - 1;

	UBRRH = (ubr >> 8);
	UBRRL = ubr;
	
	UCSRB  = (1 <<  RXEN) | (1 << TXEN);    // Enable Receiver and Transmitter
	UCSRB |= (1 << RXCIE);					// Enable RECV Interrupts
	// UCSRB |= (1 << TXCIE);			    // Enable TRANSMIT Interrupts
	UCSRC  = (1 << URSEL) | (1 << UCSZ1) | (1 << UCSZ0);   // Async-mode-8 bits
	
	/*
	>> Asynchronous mode
	>> No Parity
	>> 1 StopBit (USBS = 0 default0

	>> char size 8
	*/
}



char 
USARTReadChar()
{
	// Wait until a data is available
	while ((UCSRA & (1 << RXC)) == 0) {
	}

	// Now USART has data from host
	return UDR;
}



void 
USARTWriteChar(char data)
{
	// Wait until the transmitter is ready
	while(!(UCSRA & (1<<UDRE))) { /*Do nothing*/ }

	//Now write the data to USART buffer
	UDR = data;
	
	// Wait to transmit byte
	while(!( UCSRA & (1 << TXC)));	
}



void
rs485_send(char *p)
{
	cli();
	if (*p == '\0')
	return;
	
	while (*p++ != '\0') {
		USARTWriteChar(*p);
	}
	sei();
}



int
rs485_read(char *buf)
{
	int n = 0;
	char k = 0;

	do {
			k = USARTReadChar();
			buf[n++] = k;
	} while (k != '\0');
	
	buf[n] = '\0';

	return n;
}
