# emb-rs485-pc
Small ATMega8 to PC serial communication (RS485) with part of ADC done. Communication with Putty solved. Currently sending at a hardwired 250k baud successfully. Should be set to function with cables of at least a few miles.

## Parts used
* ATMega8A-PU with 16MHz Xtal.
* Max3483CPA for RS485 Serial communication
* Noname (probably Chinese) PC USB-to-DB9 cable
* Noname (probably Chinese) RS485 (terminal block pins Differential signal + GND) to RS232 (DB9)
* Several LEDs and pushbuttons
* Atmel-ICE AVR Programmer (JTAG)
* Power Jack

## Functionality
Stays in listening mode for keyboard ASCII commands (only text ASCII and return). When receiving, echoes the received byte and records it in a circular buffer which is emptied on return. 1/5000 cycles is spent echoing a standard keeo-alive message with the contents of the buffer. The ADC is prepared with a library to receive input but is not wired yet. The goal is to make this into a very low cost oscilloscope capable of about 1k acquisitions per second, transmitting them through serial comms to the PC.
