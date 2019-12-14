/*
 * TestComms.c
 *
 * Created: 29.09.2019 17:24:23
 *  Author: Dragos Rugescu
 */ 

// ATMEGA 8A - 16 mhz external osc with 22 pF caps - 0xFF and 0xC9 fuses
#define F_CPU 16000000UL // 16 MHz clock speed with external oscillator
#define BAUD 38400UL

#define BUTTON1   PB1 // button switch connected to port B pin 1
#define LED1    PINB0 // Led1 connected to port B pin 0
#define DE      PIND7 // DE for direction of RS485 transmission

#include <avr/io.h>
#include <stdio.h>
#include <string.h>
#include <util/delay.h>
#include <stdlib.h>
#include "rs485.h"
#include <stddef.h>
#include <avr/wdt.h>
#include <avr/interrupt.h>

// RS485 - https://datasheets.maximintegrated.com/en/ds/MAX3483-MAX3491.pdf
// RE~ and DE both 0 - Receiving
// RE~ and DE both 1 - (or more precisely DE high and RE dont care - Transmitting
// RE~ is 1 and DE is 0 - low power mode

enum ports {
	PORT_A = 1,
	PORT_B,
	PORT_C,
	PORT_D,
	PORT_E,
	PORT_F,
	PORT_G
};

int DDE = 0;
int PRT = 0;
int current_dir = 0;
const int usdelayt = 5000;
const int usdelayr = 100;

void set_pins(int Port, int Dir) {
	PRT = Port;
	DDE = Dir;
}

void set_transmit() {
	_delay_us(usdelayt);
	if (PRT == PORT_D) {
		PORTD |= (1 << DDE);
	}
	current_dir = 1;
	_delay_us(usdelayt);
}

void set_receive() {
	if (PRT == PORT_D) {
		PORTD &= ~(1 << DDE);
	}
	_delay_us(usdelayr);
	current_dir = 0;
}

// Interrupt for receiving data
char recv_buff[255] = { 0 };
volatile int n = 0;

// Code to be executed when the USART receives a byte here
ISR(USART_RXC_vect) {
	// Assume we are in receive mode - disable global interrupts
	cli();
	
	// Receive Char
	char received_byte = USARTReadChar();

	// Reset buffer on margin
	if (n >= 254) { n = 0; memset(recv_buff, 0, 255); }

	// On newline reset buffer
	if ((received_byte == '\n') || (received_byte == '\r')) { n = 0; memset(recv_buff, 0, 255); }
		
	// Accept proper ASCII or newline
	if ((received_byte >= 32) || (received_byte == '\r') || (received_byte == '\n')) {
		// Add to buffer
		if (received_byte >= 32)
			recv_buff[n++] = received_byte;

		// Set to transmit
		PORTD |= (1 << DE);
		_delay_us(100); // for baud 250000 this seems to be needed - oh well, ISR with delays
		
		// Retransmit
		USARTWriteChar(received_byte);
		_delay_us(100);
		
		// Transmit debugging information
		rs485_send("\r\r\nsent = ");
		if (n >= 10) {
			USARTWriteChar((n/10)+48);
			USARTWriteChar((n%10)+48);
		}
		else
			USARTWriteChar((char)(n+48));
		rs485_send("\r\r\n");
		_delay_us(100); // 0.3 ms total delay + instructions
	}
	
	// Re-enable global interrupts
	sei();
}

int main(void)
{
	/* Initialize buttons and LED */
	DDRB |=  (1 << LED1);	 // LED  as output
	DDRB &= ~(1 << BUTTON1); // BTN1 as input
	DDRD |=  (1 << DE);      // RS485 direction pin, as output
	
	set_pins(PORT_D, DE);
	
	/* Initialize RS-485 driver*/
	// rs485_init(4800);
	rs485_init(250000); // putty seems to work with XON/XOFF
	sei();

	set_receive();
	_delay_us(500) ;
	
	char str[10];
	char buffer[275];
	int i = 0;
	long int j = 0;

    while(1)
    {
		// ISR reception active
		if (j < 5000) j++; else j = 0;
		
		//    1/5000 transmission
		// 4999/5000 wait for input

		if (j == 0) { 
			// Send message and switch LED - interrupt atomic region
			PORTB &= ~(1 << LED1);
			
			_delay_us(100);
			
			memset(buffer, 0, 275);
			memset(str, 0, 10);
			sprintf(str, "%d", i);
			strcpy(buffer, " [ i = ");
			strcat(buffer, str);
			strcat(buffer, " ] >> ");
			
			// Disable global interrupts while handling the global buffer
			cli();
			strcat(buffer, recv_buff);
			strcat(buffer, "\r\n");
			
			_delay_us(100);
			set_transmit();
			_delay_us(100);
			
			// Send the standard message to Putty
			rs485_send(buffer);
			_delay_us(100);
			
			// Switch back to receive
			set_receive();
			
			// Re-enable global interrupts
			sei();
			
			// Increment message count
			i++;
		}
		else {
			PORTB |= (1 << LED1);

			// Must continually set pin?
			set_receive();
		}
    }
}
