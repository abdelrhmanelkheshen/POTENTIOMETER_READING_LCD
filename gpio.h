/*
 * gpio.h
 *
 *  Created on: Apr 27, 2023
 *      Author: Abdelrhman Hesham
 */

#ifndef GPIO_H_
#define GPIO_H_

#include "standard_types_defined.h"

#define GPIO_PORTS_NUMBER 4
#define GPIO_PINS_NUMBER_PER_PORT 8

/*PORTS ID*/

#define PORTA_ID 0
#define PORTB_ID 1
#define PORTC_ID 2
#define PORTD_ID 3

/*PINS ID*/
#define PIN0_ID 0
#define PIN1_ID 1
#define PIN2_ID 2
#define PIN3_ID 3
#define PIN4_ID 4
#define PIN5_ID 5
#define PIN6_ID 6
#define PIN7_ID 7

/*ENUMS USED*/
typedef enum
{
	PIN_INPUT,PIN_OUTPUT
}GPIO_pinDirectionType;

typedef enum
{
	PORT_INPUT,PORT_OUTPUT = 0xFF
}GPIO_portDirectionType;

typedef union
{
	uint8 DATA;
	struct
	{
		uint8 BIT0 : 1; /*BIT 0 SIZE IS 1 BIT*/
		uint8 BIT1 : 1;
		uint8 BIT2 : 1;
		uint8 BIT3 : 1;
		uint8 BIT4 : 1;
		uint8 BIT5 : 1;
		uint8 BIT6 : 1;
		uint8 BIT7 : 1;
	}BITS;
}REG_8BITS;


/*PORTS ADDRESS*/
/*PORTA REGISTERS*/
/*REGISTER PORTX TO WRITE*/
#define PORTA  ( *(volatile REG_8BITS*)0x3B )
/*REGISTER PINX TO READ*/
#define PINA  ( *(volatile REG_8BITS*)0x39 )
/*REGISTER DDRX TO DETERMINE WHETHER IT'S INPUT OR OUTPUT*/
#define DDRA  ( *(volatile REG_8BITS*)0x3A )
/*PORTB REGISTERS*/
/*REGISTER PORTX TO WRITE*/
#define PORTB  ( *(volatile REG_8BITS*)0x38 )
/*REGISTER PINX TO READ*/
#define PINB  ( *(volatile REG_8BITS*)0x36 )
/*REGISTER DDRX TO DETERMINE WHETHER IT'S INPUT OR OUTPUT*/
#define DDRB  ( *(volatile REG_8BITS*)0x37 )
/*PORTC REGISTERS*/
/*REGISTER PORTX TO WRITE*/
#define PORTC  ( *(volatile REG_8BITS*)0x35 )
/*REGISTER PINX TO READ*/
#define PINC  ( *(volatile REG_8BITS*)0x33 )
/*REGISTER DDRX TO DETERMINE WHETHER IT'S INPUT OR OUTPUT*/
#define DDRC  ( *(volatile REG_8BITS*)0x34 )
/*PORTD REGISTERS*/
/*REGISTER PORTX TO WRITE*/
#define PORTD  ( *(volatile REG_8BITS*)0x32 )
/*REGISTER PINX TO READ*/
#define PIND  ( *(volatile REG_8BITS*)0x30 )
/*REGISTER DDRX TO DETERMINE WHETHER IT'S INPUT OR OUTPUT*/
#define DDRD  ( *(volatile REG_8BITS*)0x31 )

/*FUNCTIONS PROTOTYPES*/
void GPIO_setupPinDirection(uint8 port_num, uint8 pin_num,GPIO_pinDirectionType direction);

void GPIO_writePin(uint8 port_num, uint8 pin_num, uint8 value);

uint8 GPIO_readPin(uint8 port_num, uint8 pin_num);

void GPIO_setupPortDirection(uint8 port_num, GPIO_portDirectionType direction);

void GPIO_writePort(uint8 port_num, uint8 value);

uint8 GPIO_readPort(uint8 port_num);

#endif /* GPIO_H_ */
