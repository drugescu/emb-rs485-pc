/*
 * rs485.h
 *
 * Created: 04.10.2019 22:12:06
 *  Author: Asus
 */ 


// Half-duplex 2wire RS485 with Atmel AVR and a RS485 driver.

// This code includes functions to initialize internal buffers, send a string as null terminated ASCII characters and receive a LF terminated line and return it with null terminator.
// Note that the UART setting for 16MHz CPU Clock, 8databit, none parity and 1stopbit are hard-coded.
// And the code assumes using AVR-GCC and ATMega8a-328p-etc. Edit the code as tools and AVR you use.

// PD0(RxD), PD1(TxD) and PD2 should be conencted to RO, DI and DE of RS485 driver respectively.
// PD0(RxD) should be pulled up externally.
// DE should be shortened to \overline{RE}


#ifndef RS485_H_
#define RS485_H_

#define F_CPU 16000000UL

#define RS485_RXC_ISR_VECT USART_RXC_vect
#define RS485_TXC_ISR_VECT USART_TXC_vect

void rs485_init(long int baudrate);
void rs485_send(char *p);
int rs485_read(char *buf);
int rs485_readln(char *buf, int size);
void USARTWriteChar(char data);
char USARTReadChar();

#endif /* RS485_H_ */