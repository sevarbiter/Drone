
#include <avr/io.h>
#include <avr/interrupt.h>
#include <avr/delay.h>
#include <pwmodulation.h>
#include "i2c.h"

#define F_CPU 8000000UL		/* Define CPU Frequency e.g. here its 8MHz */
#include <avr/io.h>		/* Include AVR std. library file */
#include <stdio.h>		/* Include std. library file */
#include <util/delay.h>		/* Include Delay header file */
#include <math.h>
#include "timer2.h"



#define M1 1
#define M2 4
#define M3 0
#define M4 3
unsigned char PortD_Status = 0;
double	M1_Throtle = 8000;
double  M2_Throtle = 8000;
double	M3_Throtle = 8000;
double	M4_Throtle = 8000;
double  Max_trotle = 14500;
unsigned char Emergency_Stop = 0;

unsigned char Sensitity = 1;
unsigned char adjustment_speed = 1;
unsigned char adjustment_speed1 = 5;
int x_angle = 0;
int y_angle = 0;

volatile short Acc_rawX, Acc_rawY, Acc_rawZ,Gyr_rawX, Gyr_rawY, Gyr_rawZ;

int Acceleration_angle[2];
int Gyro_angle[2];
int Total_angle[2];
int rad_to_deg = 180/3.141592654;
int elapsedTime, time, timePrev;

float PID, pwmLeft, pwmRight, errors, previous_error;
float pid_p=0;
float pid_i=0;
float pid_d=0;
/////////////////PID CONSTANTS/////////////////
double kp=3.55;//3.55
double ki=0.005;//0.003
double kd=2.05;//2.05
///////////////////////////////////////////////

double throttle=1300; //initial value of throttle to the motors
float desired_angle = 0; //This is the angle in which we whant the
//balance to stay steady

unsigned char data[10] = {'x',':',' ','+','0','0','0','0','0','\0'};



	

void SetModulation(short t)
{
	OCR1A = ICR1 - t; //18000
}
#include <avr/io.h>
void pwm_init(short t)
{
	TCCR1A |= 1<<WGM11 | 1<<COM1A1 | 1<<COM1A0;
	TCCR1B |= 1<<WGM13 | 1<<WGM12 | 1<<CS10;
	ICR1 = 2999;
	OCR1A = ICR1 - t;
	//RANGE 850 -
	//SetModulation(t);

}
/*----------------------------------------------------------------------------------------------
*move_motor(double  x, unsigned char servo_pin )
*Input: value for which pulse with modulation will modulate and the pinout desired for the modulation
*Output: none
*Description: This function will take a certain value between 8000 - 15000.
*			  This value is then used to create a pulse with modulation
*			  which is required for creating a PWM Signal.
------------------------------------------------------------------------------------------------*/
void move_motor(double  x, unsigned char servo_pin ){
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
	_delay_us(x);
	PORTD = 0x00;
	_delay_us(20000-x);
	
}
/*----------------------------------------------------------------------------------------------
*move_motor_calibrate(double x)
*Input: value for which pulse with modulation will modulate at
*Output: none
*Description: This function will take a certain value between 0 - 20. 
*			  This value is then used to create a pulse with modulation 
*			  which is required for creating a PWM Signal.
------------------------------------------------------------------------------------------------*/
void move_motor_calibrate(double x){

		
		PORTD = 0xFF;
		_delay_ms(x);
		PORTD = 0x00;
		_delay_ms(20-x);
		
}
/*----------------------------------------------------------------------------------------------
*findnum(int t)
*Input: int t
*Output: none
*Description: This function takes an integer and updates an array for outputing to LCD. used
			  for testing gyro.
------------------------------------------------------------------------------------------------*/
void findnum(int t){
	 int temp = t;
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
	data[7] = temp/10 + 48;
	temp = temp % 10;
	data[8] = temp + 48;

}

/*----------------------------------------------------------------------------------------------
*CalibrateESC()
*Input: none
*Output: none
*Description: This function sends a set of values to move_motor_calibrate(), which in turn
*			  proceeds to calibrate the ESC with the proper PWM values.
------------------------------------------------------------------------------------------------*/
void CalibrateESC()
{ 
	
	for(int i = 5; i < 8; i++ ){
			while(!TimerFlag){
				move_motor_calibrate(i);
			}
			TimerFlag = 0;
			

			
		}

}
/*----------------------------------------------------------------------------------------------
*GyroAcel_init()
*Input: none
*Output: none
*Description: This function initializes the gyro/accelerometer module used to control the flight
*			  of the drone, it turns on the gyro and accelerometer. 
------------------------------------------------------------------------------------------------*/
void GyroAcel_init()
{
	i2cStart(0x69);
	i2cSend(0x7E);
	i2cSend(0x00);
	_delay_ms(300);
	i2cStop();
	
	i2cStart(0x69);
	i2cSend(0x7E);
	i2cSend(0x03);
	_delay_ms(300);
	
	i2cStart(0x69);
	i2cSend(0x7E);
	i2cSend(0x15);
	_delay_ms(300);
	
	i2cStart(0x69);
	i2cSend(0x7E);
	i2cSend(0x12);
	_delay_ms(300);
	i2cStop();
}
/*----------------------------------------------------------------------------------------------
*move_motor_calibrate(double x)
*Input: value for which pulse with modulation will modulate at
*Output: none
*Description: This function will take a certain value between 0 - 20.
*			  This value is then used to create a pulse with modulation
*			  which is required for creating a PWM Signal.
------------------------------------------------------------------------------------------------*/
//Performs two reads. The lower 7 bits and upper 7 bits. 
short GyroAcel_read(uint8_t registers)
{ 
	

		i2cStart(0x69);
		i2cSend(registers);
		i2cStop();
		_delay_us(50); //DO NOT CHANGE THIS
		i2cStartRead(0x69);
		char dta1 =  i2cReadAck();
		char dta2 = i2cReadNoAck();
		i2cStop();
		return (short)dta2 << 8 | (short)dta1;

	
	//findnum(variable);
}

/*----------------------------------------------------------------------------------------------
*Update_Angles()
*Input: none
*Output: none
*Description: This function updates the values held in Acceleration_angle[] matrix for both 
			  the x-axis and the y-axis. It performs this update by calling on the gyro/accelerometer
			  module and taking a reading. 
------------------------------------------------------------------------------------------------*/
void Update_Angles()
{
	Gyr_rawX = GyroAcel_read(0x0C);
	 			Gyr_rawY =  GyroAcel_read(0x0E);
	 			Gyr_rawZ = GyroAcel_read(0x10);
	  			Acc_rawX =  GyroAcel_read(0x12);
	Acc_rawY = GyroAcel_read(0x14);
	  			Acc_rawZ = GyroAcel_read(0x16);
	  
	//-90 -> 90: on X, away from atmega negative
	Acceleration_angle[0] = (atan((Acc_rawY/16384.0)/sqrt(pow((Acc_rawX/16384.0),2) + pow((Acc_rawZ/16384.0),2)))*rad_to_deg);
	Acceleration_angle[1] = atan(-1*(Acc_rawX/16384.0)/sqrt(pow((Acc_rawY/16384.0),2) + pow((Acc_rawZ/16384.0),2)))*rad_to_deg;
	findnum(Acceleration_angle[0]);
			    
}

/*----------------------------------------------------------------------------------------------
*TiltTowardsNegativeY()
*Input: none
*Output: none
*Description: This function compares the desired degree variable y_angle with the degree value 
			  held in the Acceleration_angle[] array and if it is found that the drone is leaning
			  towards the -Y axis then a correction on alternating motors' speed is performed till 
			  desired value is reached. 
------------------------------------------------------------------------------------------------*/
void TiltTowardsNegativeY()
{
	if((Acceleration_angle[1] < y_angle - Sensitity)){
	

		if(M1_Throtle > 8000){
			M1_Throtle -= adjustment_speed;
		}
		if(M3_Throtle > 8000){
			M3_Throtle -= adjustment_speed;
		}
		
		//increase weak motor
		if(M2_Throtle < Max_trotle){
			M2_Throtle += adjustment_speed1;
		}
		if (M4_Throtle < Max_trotle)
		{
			M4_Throtle += adjustment_speed1;
		}
		


	}
}
/*----------------------------------------------------------------------------------------------
*TiltTowardsPositiveY()
*Input: none
*Output: none
*Description: This function compares the desired degree variable y_angle with the degree value 
			  held in the Acceleration_angle[] array and if it is found that the drone is leaning
			  towards the Y axis then a correction on alternating motors' speed is performed till 
			  desired value is reached. 
------------------------------------------------------------------------------------------------*/
void TiltTowardsPositiveY()
{
	if((Acceleration_angle[1] > y_angle + Sensitity) ){
		if(M2_Throtle > 8000){
			M2_Throtle -= adjustment_speed;
		}
		if (M4_Throtle > 8000)
		{
			M4_Throtle -= adjustment_speed;
		}
		

		//increase weak motor
		if(M3_Throtle < Max_trotle){
			M3_Throtle += adjustment_speed1;
		}
		if (M1_Throtle < Max_trotle)
		{
			M1_Throtle += adjustment_speed1;
		}
		
	}
}
/*----------------------------------------------------------------------------------------------
*TiltTowardsNegativex()
*Input: none
*Output: none
*Description: This function compares the desired degree variable x_angle with the degree value 
			  held in the Acceleration_angle[] array and if it is found that the drone is leaning
			  towards the -x axis then a correction on alternating motors' speed is performed till 
			  the desired value is reached. 
------------------------------------------------------------------------------------------------*/
void TiltTowardsNegativeX()
{
	if((Acceleration_angle[0] < x_angle - Sensitity) ){
		
		
		if(M2_Throtle > 8000){
			M2_Throtle -= adjustment_speed;
		}
		if(M1_Throtle > 8000){
			M1_Throtle -= adjustment_speed;
		}
		
		//increase weak motors
		if(M3_Throtle < Max_trotle ){
			M3_Throtle += adjustment_speed1;
		}
		if(M4_Throtle < Max_trotle){
			M4_Throtle += adjustment_speed1;
		}

	 		}
}
/*----------------------------------------------------------------------------------------------
*TiltTowardPositiveX()
*Input: none
*Output: none
*Description: This function compares the desired degree variable x_angle with the degree value 
			  held in the Acceleration_angle[] array and if it is found that the drone is leaning
			  towards the X axis then a correction on alternating motors' speed is performed till 
			  desired value is reached. 
------------------------------------------------------------------------------------------------*/
void TiltTowardsPositiveX()
{
	if((Acceleration_angle[0] > x_angle + Sensitity) ){

		if(M3_Throtle > 8000){
			M3_Throtle -= adjustment_speed;
		}
		if(M4_Throtle > 8000){
			M4_Throtle -= adjustment_speed;
		}
		
		
		//increase weak motor
		if(M2_Throtle < Max_trotle){
			M2_Throtle += adjustment_speed1;
		}
		if (M1_Throtle < Max_trotle)
		{
			M1_Throtle += adjustment_speed1; 
		}
	}
}

 int main(void)
 {
	DDRD = 0xFF; 
	DDRC = 0xFF ;PORTC = 0xFF; 
	DDRA = 0xFF; PORTA = 0x00;;
	DDRB = 0x00; PORTB = 0xFF;

	TimerSet(4900);
	TimerOn();
	
// 	LCD_init();
// 	LCD_DisplayString(1,"HELLO");
// 	GyroAcel_init();
// 	while(1){
// 		Update_Angles();
// 		while(!TimerFlag){
// 			
// 		}
// 		TimerFlag = 0;
// 		findnum(Acceleration_angle[0]);
// 		LCD_DisplayString(1,data);
// 		
// 	}
	CalibrateESC();
	
	
	//Initialize gyro
	GyroAcel_init();
	TimerSet(25);
	Update_Angles();

	
	y_angle = -1;
	x_angle = 0;
	
	
	unsigned char bInput = 0;
	
		x_angle = Acceleration_angle[0] = (atan((Acc_rawY/16384.0)/sqrt(pow((Acc_rawX/16384.0),2) + pow((Acc_rawZ/16384.0),2)))*rad_to_deg);
		y_angle = Acceleration_angle[1] = atan(-1*(Acc_rawX/16384.0)/sqrt(pow((Acc_rawY/16384.0),2) + pow((Acc_rawZ/16384.0),2)))*rad_to_deg;
	int i = 0;
	while(1)
	{
		while(!TimerFlag){
								
								move_motor(M1_Throtle,M1);
								move_motor(M4_Throtle,M4);
								move_motor(M3_Throtle,M3);
								move_motor(M2_Throtle,M2);
		}
		TimerFlag = 0;
		
		//PORTD = PORTD | 0x80;
		if(i < 1000){
			M1_Throtle +=3;
			M2_Throtle +=3;
			M3_Throtle +=3;
			M4_Throtle +=3;
			i++;
		}
		else{
			M1_Throtle = M2_Throtle = M3_Throtle = M4_Throtle = 8000;
		}
		
		
		
		//A TILT TOWARDS -Y WHEN Y is not within desired degree(y_angle)
		TiltTowardsNegativeY();

		//A TILT TOWARDS Y WHEN Y is not within desired degree(y_angle)
		TiltTowardsPositiveY();

		//A TILT TOWARDS -X WHEN X is not within desired degree(x_angle)
		TiltTowardsNegativeX();
		
		//A Tilt TOWARDS X WHEN X is not within desired degree(x_angle)
		TiltTowardsPositiveX();

			

	

		Update_Angles();
		bInput = ~PORTB;
	if(bInput < 15 && bInput > 0) {
		if(bInput == 0x01) {
			x_angle+=5;
		} else if(bInput == 0x02) {
			x_angle-=5;
		} else if(bInput == 0x04) {
			y_angle-=5;
		} else if(bInput == 0x08) {
			y_angle+=5;
		}
	} else {
		x_angle = 0;
		y_angle = 0;
	}

	}


	

}




