/*
 * adc.h
 *
 *  Created on: Apr 29, 2023
 *      Author: Abdelrhman Hesham
 */

#ifndef ADC_H_
#define ADC_H_
#include"standard_types_defined.h"

#define MAX_VOLT 2.56
#define MAX_DIGITAL_VALUE 1023

typedef enum
{
	INTERNAL_VREF0 ,AVCC,RESERVED,INTERNAL_VREF2
}VOLTAGE;

typedef enum
{
	FCPU_2=1,FCPU_4,FCPU_8,FCPU_16,FCPU_32,FCPU_64,FCPU_128
}PRESCALER;

typedef struct
{
	VOLTAGE volt;
	PRESCALER scale;
}ADC_config;

void ADC_init(ADC_config * ptr);
uint16 ADC_readChannel(uint8 ch_num);

#endif /* ADC_H_ */
