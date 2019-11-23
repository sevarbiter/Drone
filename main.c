// #include <avr/io.h>
// #include "timer2.h"
// 
// int main(void)
// {
// 	DDRB |= (1 << DDB1) |(1 << DDB2);
// 	// PB1 and PB2 is now an output
// 
// 	ICR1 = 2499;
// 	// set TOP to 16bit
// 
// 	OCR1A = 0;
// 	// set PWM for 25% duty cycle @ 16bit
// 
// 	OCR1B = 65;
// 	// set PWM for 75% duty cycle @ 16bit
// 
// 	TCCR1A |= (1 << COM1A1)|(1 << COM1B1);
// 	// set none-inverting mode
// 
// 	TCCR1A |= (1 << WGM11);
// 	TCCR1B |= (1 << WGM12)|(1 << WGM13);
// 	// set Fast PWM mode using ICR1 as TOP
// 	
// 	TCCR1B |= (1 << CS10);
// 	// START the timer with no prescaler
// 	OCR1B = 65;
// 	
// 	
// 	
// 
// 	
// 		while(!TimerFlag){
// 			
// 		}
// 		TimerFlag = 0;
// 		//OCR1B = 0x005;
// 
// 		
// 
// 		
// 	
// 
// 	while (1);
// 	{
// 		
// 			
// 		}
// 
// }
#include <avr/io.h>
#include <avr/interrupt.h>
#include <avr/delay.h>
#include <pwmodulation.h>
#include "i2c.h"
#include "io.h"

#define F_CPU 8000000UL		/* Define CPU Frequency e.g. here its 8MHz */
#include <avr/io.h>		/* Include AVR std. library file */
#include <stdio.h>		/* Include std. library file */
#include <util/delay.h>		/* Include Delay header file */
#include "timer2.h"

unsigned char data[10] = {'x',':',' ','+','0','0','0','0','0','\0'};


#include <avr/io.h>
void move_servo(unsigned char  x, unsigned char servo_pin ){
	unsigned char s_value;
	switch(servo_pin){
		case 0:
		s_value = 1;
		break;
		case 1:
		s_value = 2;
		break;
		case 2:
		s_value = 4;
		break;
		case 3:
		s_value = 8;
		break;
		case 4:
		s_value = 0x10;
		break;
		case 5:
		s_value = 0x20;
		break;
		case 6:
		s_value = 0x40;
		break;
		default:
		s_value = 1;
		break;
	}
	PORTD = s_value;;
	_delay_ms(x);
	PORTD = 0x00;
	_delay_ms(20-x);
	
}
void findnum(unsigned short t){
	signed short temp = (signed)(t);
	if(temp < 0){
		data[3] = '-';
		temp = temp * (-1);
	}
	else
		data[3] = '+';
	
	data[4] = temp/10000 + 48;
	temp = temp % 10000;
	data[5] = temp/1000 + 48;
	temp = temp % 1000;
	data[6] = temp/100 + 48;
	temp = temp % 100;
	data[7] - temp/10 + 48;
	temp = temp % 10;
	data[8] = temp + 48;
}

int main(void)
{
	DDRD = 0xFF;
	DDRA = 0xFF;
/*	DDRA = 0x01; PORTA = 0;*/
// 	DDRD |= (1<<PD5);	/* Make OC1A pin as output */
// 	TCNT1 = 0;		/* Set timer1 count zero */
// 	ICR1 = 2499;		/* Set TOP count for timer1 in ICR1 register */
// 
// 	/* Set Fast PWM, TOP in ICR1, Clear OC1A on compare match, clk/64 */
// 	TCCR1A = (1<<WGM11)|(1<<COM1A1);
// 	TCCR1B = (1<<WGM12)|(1<<WGM13)|(1<<CS10)|(1<<CS11);
// 	TimerSet(1000);
// 	TimerOn();
// 	
// 	OCR1A = 65;


	

	
	TimerSet(5000);
	TimerOn();
	unsigned char j = 0;
	unsigned char i = 1;
	LCD_init();
	LCD_DisplayString(1, data);
	
	//Initialize gyro
	i2cStart(0x69);
	i2cSend(0x7E);
	i2cSend(0x15);
	_delay_ms(100);
	i2cStop();
	
	
	
	//Read gyro:
	
	
	
	
	
	
	while(1)
	{
		//Read Gyro:
			i2cStart(0x69);
			i2cSend(0x0C);
			i2cStop();
			_delay_ms(100);
			
			i2cStartRead(0x69);
			unsigned char dta1 =  i2cReadAck();
			unsigned char dta2 = i2cReadNoAck();
			
			i2cStop();
			
			unsigned short n = (short)dta2 << 8 | (short)dta1;
			findnum(n);
			LCD_DisplayString(1,data);
			_delay_ms(1000);
			
			

// 
// 
// 		PORTA = (PORTA == 0x00)? 0x01: 0x00;
// 		while(!TimerFlag){
// 					move_servo(j,3);
// 					move_servo(j,4);
// 					move_servo(j,5);
// 					move_servo(j,6);
// 					
// 		}
// 		TimerFlag = 0;
// 		j +=1;
// 		if(j == 15){
// 			j = 0;
// 		}



	}
}

