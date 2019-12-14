/*
 * adc.h
 *
 * Created: 07.12.2019 16:36:08
 *  Author: Asus
 */ 


#ifndef ADC_H_
#define ADC_H_


#ifdef __cplusplus
extern "C" {
#endif

#include <avr/io.h>
#include <avr/interrupt.h>

#define ADC_PRESCALER_2 	0
#define ADC_PRESCALER_4 	2
#define ADC_PRESCALER_16 	4
#define ADC_PRESCALER_32 	5
#define ADC_PRESCALER_64 	6
#define ADC_PRESCALER_128 	7

#define ADC_VREF_AREF 	0
#define ADC_VREF_AVCC 	64
#define ADC_VREF_MISC1 	128
#define ADC_VREF_MISC2 	192


uint16_t adc_read(uint8_t prescaler, uint8_t vref, uint8_t pin);
void adc_start(uint8_t prescaler, uint8_t vref, uint8_t pin_qty, void (*handler)(uint8_t, uint16_t));
void adc_stop();



#ifdef __cplusplus
}
#endif

#endif /* ADC_H_ */