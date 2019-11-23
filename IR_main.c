/*
 * cs122_project_ir_test.c
 *
 * Created: 11/8/2019 7:09:31 PM
 * Author : Wes
 */ 

#include <avr/io.h>


int main(void)
{
	DDRA = 0x00; PORTA = 0xFF;
	DDRB = 0xFF; PORTB = 0x00;
	unsigned char val;                                                                         
    /* Replace with your application code */
    while (1) 
    {
		val = PINA;
		
		PORTB = val;
    }
}

