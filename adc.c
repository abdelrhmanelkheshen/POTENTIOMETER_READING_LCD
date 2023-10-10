/*
 * adc.c
 *
 *  Created on: Apr 29, 2023
 *      Author: Abdelrhman Hesham
 */
#include"avr/io.h"
#include"common_macros.h"
#include"adc.h"
#include "standard_types_defined.h"

void ADC_init(ADC_config * ptr)
{
	SET_BIT(ADCSRA,ADEN);
	ADMUX = (ADMUX & 0x3F) | (ptr->volt << 6);
	ADCSRA = (ADCSRA & 0xF8) | (ptr->scale);
}

uint16 ADC_readChannel(uint8 ch_num)
{
	ADMUX = (ADMUX & 0xE0) | (ch_num & 0x07);
	SET_BIT(ADCSRA,ADSC);
	while(BIT_IS_CLEAR(ADCSRA,ADIF))
	{

	}
	SET_BIT(ADCSRA,ADIF);
	return ADC;
}
