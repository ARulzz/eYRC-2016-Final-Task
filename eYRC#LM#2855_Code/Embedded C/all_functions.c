/*
* Team Id: eYRC-LM#2855
* Author List: Aarya R. Shankar
* Filename: all_functions.c
* Theme: Launch A Module
* Functions:  motion_pin_config(),buzzer_pin_config(),buzzer_on(),buzzer_off(),init_timer5(),,lcd_port_config(),left_encoder_pin_config(),right_encoder_pin_config(),right_position_encoder_interrupt_init(),left_position_encoder_interrupt_init()
			  motion_set(char),forward(),back(),left(),right(),soft_left(),soft_right(),angle_rotate(int),linear_distance_mm(int),forward_mm(int),back_mm(int),left_degrees(int),right_degrees(int),soft_right_degrees(int),soft_left_degrees(int),
			  velocity(int,int),port_init(),init_devices()
* Global Variables: ShaftCountLeft,ShaftCountRight,Degrees,DistanceShaft
*/

#define F_CPU 14745600
#include <avr/io.h>
#include <avr/interrupt.h>
#include <util/delay.h>
#include "lcd.c"
#include <stdio.h>
#include <stdlib.h>
#include <math.h>

unsigned int value;

volatile unsigned long int ShaftCountLeft = 0; 
//ShaftCountLeft : to keep track of left position encoder 
volatile unsigned long int ShaftCountRight = 0; 
//ShaftCountRight: to keep track of right position encoder
volatile unsigned int Degrees; 
//Degrees: to accept angle in degrees for turning
volatile unsigned long int distanceShaft = 0;
//distanceShaft : To 

/*
* Function Name:	motipn_pin_config(void)
* Input:			None
* Output:			Initialize the required registers
* Logic:			Initialization : Function to configure ports to enable robot's motion (Port A and Port L here where the motors are connected and where pwm is generated)
* Example Call:		motion_pin_config()
*/

void motion_pin_config (void) 
{
 DDRA = DDRA | 0x0F;
 PORTA = PORTA & 0xF0;
 DDRL = DDRL | 0x18;   //Setting PL3 and PL4 pins as output for PWM generation
 PORTL = PORTL | 0x18; //PL3 and PL4 pins are for velocity control using PWM.
}

/*
* Function Name:	buzzer_pin_config(void)
* Input:			None
* Output:			Initialize the required registers
* Logic :			Initialization : Function to configure ports to use buzzer pin
* Example Call:		buzzer_on()
*/

void buzzer_pin_config(void)
{
	DDRC=DDRC | 0x08; 		// Port C pin 3 as output
	PORTC=PORTC & 0xF7;

}

void servo1_pin_config (void)
{
	DDRB  = DDRB | 0x20;  //making PORTB 5 pin output
	PORTB = PORTB | 0x20; //setting PORTB 5 pin to logic 1
}

//Configure PORTB 6 pin for servo motor 2 operation
void servo2_pin_config (void)
{
	DDRB  = DDRB | 0x40;  //making PORTB 6 pin output
	PORTB = PORTB | 0x40; //setting PORTB 6 pin to logic 1
}

/*
* Function Name:	buzzer_on(void)
* Input:			None
* Output:			Initialize the required registers
* Logic :			Turns the pin connected to the buzzer to high state so that sound is produced
* Example Call:		buzzer_on()
*/

void buzzer_on(void)
{
	
	PORTC= 0x08;   // pin 3 to high 0000 1000
}

/*
* Function Name:	buzzer_off(void)
* Input:			None
* Output:			Initialize the required registers
* Logic :			Turns the pin connected to the buzzer to low state so buzzer produces no sound
* Example Call:		buzzer_off()
*/

void buzzer_off(void)
{
	PORTC= 0x00;
}

//TIMER1 initialization in 10 bit fast PWM mode  
//prescale:256
// WGM: 7) PWM 10bit fast, TOP=0x03FF
// actual value: 52.25Hz 
void timer1_init(void)
{
 TCCR1B = 0x00; //stop
 TCNT1H = 0xFC; //Counter high value to which OCR1xH value is to be compared with
 TCNT1L = 0x01;	//Counter low value to which OCR1xH value is to be compared with
 OCR1AH = 0x03;	//Output compare Register high value for servo 1
 OCR1AL = 0xFF;	//Output Compare Register low Value For servo 1
 OCR1BH = 0x03;	//Output compare Register high value for servo 2
 OCR1BL = 0xFF;	//Output Compare Register low Value For servo 2
 OCR1CH = 0x03;	//Output compare Register high value for servo 3
 OCR1CL = 0xFF;	//Output Compare Register low Value For servo 3
 ICR1H  = 0x03;	
 ICR1L  = 0xFF;
 TCCR1A = 0xAB; /*{COM1A1=1, COM1A0=0; COM1B1=1, COM1B0=0; COM1C1=1 COM1C0=0}
 					For Overriding normal port functionality to OCRnA outputs.
				  {WGM11=1, WGM10=1} Along With WGM12 in TCCR1B for Selecting FAST PWM Mode*/
 TCCR1C = 0x00;
 TCCR1B = 0x0C; //WGM12=1; CS12=1, CS11=0, CS10=0 (Prescaler=256)
}

/*
* Function Name:	init_timer5(void)
* Input:			None
* Output:			Initialize the required registers
* Logic :			Configuring the timer5 registers in Atmega2560 for using PWM functionalities
* Example Call:		init_timer5()
*/

void init_timer5(void)	//Timer For PWM
{
	TCCR5B = 0x00;	//stop
	TCNT5H = 0xFF;	//counter higher 8 bit value to which OCRxH is compared with
	TCNT5L = 0x01;	//counter higher 8 bit value to which OCRxH is compared with
	OCR5AH = 0x00;	//Output compare register high for left motor
	OCR5AL = 0xFF;	//Output compare register low for left motor
	OCR5BH = 0x00;	//Output compare register high for right motor	
	OCR5BL = 0xFF;	//Output compare register high for right motor
	OCR5C  = 0xFF;	//Motor C1
	OCR5CH = 0x00;	//Motor C1
	OCR5CL = 0xFF;
	TCCR5A = 0xA9;

/* COM5A1=1,COM5A0=0,COM5B1=1,COM5B0=0,COM5C1=1,COM5C0=0
For Overriding normal port functionality to OCRnA ouputs
WGM51=0,WGM50=1 along with WGM52 in TCCRB for selecting fast PWM 8 bit mode
*/

	TCCR5B = 0x0B;	//WGM12=1,CS12=0,CS11=1,CS10=1	(Prescaler=64)

}

/*
* Function Name:	lcd_port_config
* Input:			None
* Output:			Initialize the required registers
* Logic :			Configuring the LCD Pins to function
* Example Call:		lcd_port_config()
*/

void lcd_port_config (void)
{
 DDRC = DDRC | 0xF7;    //all the LCD pin's direction set as output
 PORTC = PORTC & 0x80;  // all the LCD pins are set to logic 0 except PORTC 7
}

/*
* Function Name:	left_encoder_pin_config
* Input:			None
* Output:			Initialize the required registers
* Logic :			Function to configure INT4 (PORTE 4) pin as input for the left position encoder
* Example Call:		left_encoder_pin_config()
*/

void left_encoder_pin_config (void)
{
 DDRE  = DDRE & 0xEF;  //Set the direction of the PORTE 4 pin as input
 PORTE = PORTE | 0x10; //Enable internal pull-up for PORTE 4 pin
}

/*
* Function Name:	right_encoder_pin_config
* Input:			None
* Output:			Initialize the required registers
* Logic :			Function to configure INT5 (PORTE 5) pin as input for the right position encoder
* Example Call:		right_encoder_pin_config()
*/

void right_encoder_pin_config (void)
{
 DDRE  = DDRE & 0xDF;  //Set the direction of the PORTE 4 pin as input
 PORTE = PORTE | 0x20; //Enable internal pull-up for PORTE 4 pin
}

/*
* Function Name:	left_position_encoder_interrupt_init
* Input:			None
* Output:			Initialize the required registers
* Logic :			Initializing the interrput pins where left encoder is connected
* Example Call:		left_position_encoder_interrupt_init()
*/

void left_position_encoder_interrupt_init (void) //Interrupt 4 enable
{
 cli(); //Clears the global interrupt
 EICRB = EICRB | 0x02; // INT4 is set to trigger with falling edge
 EIMSK = EIMSK | 0x10; // Enable Interrupt INT4 for left position encoder
 sei();   // Enables the global interrupt 
}

/*
* Function Name:	right_position_encoder_interrupt_init
* Input:			None
* Output:			Initialize the required registers
* Logic :			Initializing the interrput pins where right encoder is connected
* Example Call:		right_position_encoder_interrupt_init()
*/

void right_position_encoder_interrupt_init (void) //Interrupt 5 enable
{
 cli(); //Clears the global interrupt
 EICRB = EICRB | 0x08; // INT5 is set to trigger with falling edge
 EIMSK = EIMSK | 0x20; // Enable Interrupt INT5 for right position encoder
 sei();   // Enables the global interrupt 
}

/*
* Function Name:	ISR(Interrupt Service Routine)
* Input:			None
* Output:			Initialize the required registers
* Logic :			ISR for right position encoder. Here counting the number of times the circle cuts the encoder
* Example Call:		called by the microcontroller when the interrupt receives a signal
*/

ISR(INT5_vect)  
{
 distanceShaft++;
 ShaftCountRight++;  //increment right shaft position count
}

/*
* Function Name:	ISR(Interrupt Service Routine)
* Input:			None
* Output:			Initialize the required registers
* Logic :			ISR for left position encoder. Here counting the number of times the circle cuts the encoder
* Example Call:		called by the microcontroller when the interrupt receives a signal
*/

ISR(INT4_vect)
{
 distanceShaft++;
 ShaftCountLeft++;  //increment left shaft position count
}

/*
* Function Name:	motion_set
* Input:			Direction - (character-Hexadecimal equivalent of the motor port configuration)
* Output:			Initialize the required registers
* Logic :			Function used for setting motor's direction. Robot will start moving in the specified direction once motion_set is called
* Example Call:		motion_set(0x00)
*/

void motion_set (unsigned char Direction)
{
 unsigned char PortARestore = 0;
 Direction &= 0x0F; 		// removing upper nibbel for the protection
 PortARestore = PORTA; 		// reading the PORTA original status
 PortARestore &= 0xF0; 		// making lower direction nibbel to 0
 PortARestore |= Direction; // adding lower nibbel for forward command and restoring the PORTA status
 PORTA = PortARestore; 		// executing the command
}

/*
* Function Name:	forward
* Input:			None
* Output:			Initialize the required registers
* Logic :			Calls the motion_set function with the hexadecimal value of the configuration which moves the motors forward
* Example Call:		forward()
*/

void forward (void) //both wheels forward
{
  motion_set(0x06);
}

/*
* Function Name:	back
* Input:			None
* Output:			Initialize the required registers
* Logic :			Calls the motion_set function with the hexadecimal value of the configuration which moves the motors backward
* Example Call:		back()
*/

void back (void) //both wheels backward
{
  motion_set(0x09);
}

/*
* Function Name:	left
* Input:			None
* Output:			Initialize the required registers
* Logic :			Turn the robot to left direction about the center(differential):Calls the motion_set function with the hexadecimal value of the configuration which moves the Left wheel backward, Right wheel forward
* Example Call:		left()
*/

void left (void)	//Left wheel backward, Right wheel forward
{
  motion_set(0x05);
}

/*
* Function Name:	right
* Input:			None
* Output:			Initialize the required registers
* Logic :			Turn the robot to right direction about the center(differential):Calls the motion_set function with the hexadecimal value of the configuration which moves the Left wheel forward, Right wheel backward
* Example Call:		right()
*/

void right (void) //Left wheel forward, Right wheel backward
{
  motion_set(0x0A);
}

/*
* Function Name:	soft_left
* Input:			None
* Output:			Initialize the required registers
* Logic :			Turn the robot to left:Calls the motion_set function with the hexadecimal value of the configuration which moves the Left wheel Stationary, Right wheel forward
* Example Call:		soft_left()
*/

void soft_left (void) //Left wheel stationary, Right wheel forward
{
 motion_set(0x04);
}

/*
* Function Name:	soft_right
* Input:			None
* Output:			Initialize the required registers
* Logic :			Turn the robot to right:Calls the motion_set function with the hexadecimal value of the configuration which moves the Left wheel Forward, Right wheel Stationary
* Example Call:		soft_right()
*/

void soft_right (void) //Left wheel forward, Right wheel is stationary
{
 motion_set(0x02);
}

/*
* Function Name:	stop
* Input:			None
* Output:			Initialize the required registers
* Logic:			Stop the motors. Set all motor states to low by calling the motion_set function with 0 value
* Example Call:		stop()
*/

void stop (void)
{
  motion_set(0x00);
}

/*
* Function Name:	velocity
* Input:			left,right(Integers storing left and right motor speeds (values 0-255))
* Output:			Velocity for motors are set
* Logic:			Changing the pulse width using registers
* Example Call:		velocity(255,255)
*/

void velocity(unsigned char left,unsigned char right)	//Set PWM Velocity
{
	//lcd_print(1,1,OCR5AL,3);
	//lcd_print(1,1,OCR5BL,3);
	OCR5AL = (unsigned char) left;
	OCR5BL = (unsigned char) right;
}

/*
* Function Name:	angle_rotate
* Input:			Degrees(Integer storing the Angle in Degrees)
* Output:			Initialize the required registers
* Logic :			For turning robot by specified degrees using encoders. When the shaftCount varibles reach the threshold, the motion is stopped
* Example Call:		angle_rotate(30)
*/

void angle_rotate(unsigned int Degrees)
{
 float ReqdShaftCount = 0;
 unsigned long int ReqdShaftCountInt = 0;
 ReqdShaftCount = (float) Degrees/ 4.090; // division by resolution to get shaft count
 ReqdShaftCountInt = (unsigned long int) ReqdShaftCount;
 ShaftCountRight = 0; 
 ShaftCountLeft = 0; 
 while (1)
 {
  //lcd_print(1,8,ShaftCountLeft,5);
  //lcd_print(2,8,ShaftCountRight,5);
  //lcd_print(2,1,ReqdShaftCountInt,3);
  if((ShaftCountRight >= ReqdShaftCountInt) | (ShaftCountLeft >= ReqdShaftCountInt))
  			break;
 }
 stop(); //Stop robot
}

/*
* Function Name:	linear_distance_mm
* Input:			DistanceInMM(Integer storing the Distance in mm)
* Output:			Initialize the required registers
* Logic :			For moving robot by specified distance using encoders. When the shaftCount varibles reach the threshold, the motion is stopped
* Example Call:		linear_distance_mm(100)
*/

void linear_distance_mm(unsigned int DistanceInMM)
{
 float ReqdShaftCount = 0;
 unsigned long int ReqdShaftCountInt = 0;
 ReqdShaftCount =(float) DistanceInMM / 5.338; // division by resolution to get shaft count
 ReqdShaftCountInt = (unsigned long int) ReqdShaftCount;
 ShaftCountLeft = ShaftCountRight = 0;
 while(1)
 {
  lcd_print(1,1,ShaftCountLeft,5);
  lcd_print(2,1,ShaftCountRight,5);
  //lcd_print(2,7,ReqdShaftCountInt,5);
  
  if((ShaftCountLeft > ReqdShaftCountInt) | (ShaftCountRight > ReqdShaftCountInt))
  {
  	break;
  }
 } 
 stop(); //Stop robot
}

/*
* Function Name:	forward_mm
* Input:			DistanceInMM(Integer storing the Distance in mm)
* Output:			Initialize the required registers
* Logic:			Function for moving the robot forward by specified distance. Robot starts moving forward and the linear_distance_mm function is called. Motion is terminated once the terminating condition is reached in the function when the encoder wheel makes the requried number of cuts for the distance to be covered
* Example Call:		forward_mm(100)
*/

void forward_mm(unsigned int DistanceInMM)
{
 forward();
 linear_distance_mm(DistanceInMM);
}

/*
* Function Name:	back_mm
* Input:			DistanceInMM(Integer storing the Distance in mm)
* Output:			Initialize the required registers
* Logic:			Function for moving the robot backward by specified distance. Robot starts moving backward and the linear_distance_mm function is called. Motion is terminated once the terminating condition is reached in the function when the encoder wheel makes the requried number of cuts for the distance to be covered
* Example Call:		back_mm(100)
*/

void back_mm(unsigned int DistanceInMM)
{
 back();
 linear_distance_mm(DistanceInMM);
}

/*
* Function Name:	left_degrees
* Input:			Degrees(Integer storing the Angle in Degrees)
* Output:			Initialize the required registers
* Logic:			Function for rotating the robot towards left by specified angle. Robot starts rotating left and the angle_rotate function is called. Motion is terminated once the terminating condition is reached in the function when the encoder wheel makes the requried number of cuts for the angle to be rotated
* Example Call:		left_degrees(100)
*/

void left_degrees(unsigned int Degrees) 
{
// 88 pulses for 360 degrees rotation 4.090 degrees per count
 left(); //Turn left
 angle_rotate(Degrees);
}

/*
* Function Name:	right_degrees
* Input:			Degrees(Integer storing the Angle in Degrees)
* Output:			Initialize the required registers
* Logic:			Function for rotating the robot towards right by specified angle. Robot starts rotating right and the angle_rotate function is called. Motion is terminated once the terminating condition is reached in the function when the encoder wheel makes the requried number of cuts for the angle to be rotated
* Example Call:		right_degrees(100)
*/

void right_degrees(unsigned int Degrees)
{
// 88 pulses for 360 degrees rotation 4.090 degrees per count
 right(); //Turn right
 angle_rotate(Degrees);
}

/*
* Function Name:	soft_left_degrees
* Input:			Degrees(Integer storing the Angle in Degrees)
* Output:			Initialize the required registers
* Logic:			Function for rotating the robot towards left by specified angle. Robot starts rotating left and the angle_rotate function is called. Motion is terminated once the terminating condition is reached in the function when the encoder wheel makes the requried number of cuts for the angle to be rotated. Only right wheel moves here
* Example Call:		soft_left_degrees(100)
*/

void soft_left_degrees(unsigned int Degrees)
{
 // 176 pulses for 360 degrees rotation 2.045 degrees per count
 soft_left(); //Turn soft left
 Degrees=Degrees*2;
 angle_rotate(Degrees);
}

/*
* Function Name:	soft_right_degrees
* Input:			Degrees(Integer storing the Angle in Degrees)
* Output:			Initialize the required registers
* Logic:			Function for rotating the robot towards right by specified angle. Robot starts rotating right and the angle_rotate function is called. Motion is terminated once the terminating condition is reached in the function when the encoder wheel makes the requried number of cuts for the angle to be rotated. Only left wheel moves here
* Example Call:		soft_right_degrees(100)
*/

void soft_right_degrees(unsigned int Degrees)
{
 // 176 pulses for 360 degrees rotation 2.045 degrees per count
 soft_right();  //Turn soft right
 Degrees=Degrees*2;
 angle_rotate(Degrees);
}

/*
* Function Name:	port_init
* Input:			None
* Output:			Initialize the required registers
* Logic:			Initialization : Initializing all the ports
* Example Call:		port_init()
*/

void port_init()
{
 motion_pin_config(); //robot motion pins config
 left_encoder_pin_config(); //left encoder pin config
 right_encoder_pin_config(); //right encoder pin config	
 buzzer_pin_config();//Buzzer Pin
 servo1_pin_config(); //Configure PORTB 5 pin for servo motor 1 operation
 servo2_pin_config(); //Configure PORTB 6 pin for servo motor 2 operation
}

/*
* Function Name:	init_devices
* Input:			None
* Output:			Intialize the required registers
* Logic:			Initialization : Initializing all the devices connected to the ports and other pins
* Example Call:		init_devices()
*/

void init_devices()
{
 cli(); //Clears the global interrupt
 port_init();  //Initializes all the ports
 lcd_port_config();
 adc_pin_config();
 init_timer5();
 timer1_init();
 left_position_encoder_interrupt_init();
 right_position_encoder_interrupt_init();
 adc_init();
 sei();   // Enables the global interrupt 
}

//Function to rotate Servo 1 by a specified angle in the multiples of 1.86 degrees
void servo_1(unsigned char degrees)
{
	float PositionPanServo = 0;
	PositionPanServo = ((float)degrees / 1.86) + 35.0;
	OCR1AH = 0x00;
	OCR1AL = (unsigned char) PositionPanServo;
}


//Function to rotate Servo 2 by a specified angle in the multiples of 1.86 degrees
void servo_2(unsigned char degrees)
{
	float PositionTiltServo = 0;
	PositionTiltServo = ((float)degrees / 1.86) + 35.0;
	OCR1BH = 0x00;
	OCR1BL = (unsigned char) PositionTiltServo;
}

//servo_free functions unlocks the servo motors from the any angle
//and make them free by giving 100% duty cycle at the PWM. This function can be used to
//reduce the power consumption of the motor if it is holding load against the gravity.

void servo_1_free (void) //makes servo 1 free rotating
{
	OCR1AH = 0x03;
	OCR1AL = 0xFF; //Servo 1 off
}

void servo_2_free (void) //makes servo 2 free rotating
{
	OCR1BH = 0x03;
	OCR1BL = 0xFF; //Servo 2 off
}
