/*
* Team Id: eYRC-LM#2855
* Author List: Aarya R. Shankar
* Filename: all_functions.c
* Theme: Launch A Module
* Functions:  uart0_init(void), pick(), place(), main() 
* Global Variables: data
*/

#include "all_functions.c"

unsigned char data;

void uart0_init(void)
{
	UCSR0B = 0x00; //disable while setting baud rate
	UCSR0A = 0x00;
	UCSR0C = 0x06;
	UBRR0L = 0x5F; //set baud rate lo
	UBRR0H = 0x00; //set baud rate hi
	UCSR0B = 0x98;
}

SIGNAL(USART0_RX_vect) 		// ISR for receive complete interrupt
{
	data = UDR0; 				//making copy of data from UDR0 in 'data' variable
	lcd_wr_char(data);		//echo data back to PC
	
}

 // Function Name:	pick(void)
 // Input:			None
 // Output:			An object in the front cell is picked and robot moves into that cell
 // Logic:			Robot moves a bit into the cell, servo 1 comes down, servo 2 closes the arm grabbing the object, servo 2 comes back up picking the object, the robot is moved forward into the cell
 // Example Call:	pick()


void pick(void)
{
	forward_mm(40);
	_delay_ms(30);
	servo_1(0);
	_delay_ms(500);
	servo_2(22);
	_delay_ms(250);
	servo_1(60);
	_delay_ms(30);
	forward_mm(140);
	_delay_ms(1000);
	
}

 // Function Name:	place(void)
 // Input:			None
 // Output:			An object is placed in the front cell
 // Logic:			Robot moves a bit into the cell, servo 1 comes down, servo 2 open up the arms releasing the object, servo 2 comes back up without the object, the robot is moved back to its original position
 // Example Call:	place()

void place(void)
{
	forward_mm(10);
	_delay_ms(1000);
	servo_1(0);
	_delay_ms(250);
	servo_2(60);
	_delay_ms(100);
	servo_1(60);
	_delay_ms(30);
	back_mm(10);
	_delay_ms(1000);
	
}

int main(void)
{
	init_devices();
	uart0_init();
	lcd_set_4bit();
	lcd_init();
	servo_1(90);
	_delay_ms(30);
	servo_2(60);
	_delay_ms(30);
	velocity(255,251);
	//forward_mm(720);
	
	//forward_mm(1800);
	while(1) {
		if(data=='f')
		{
			velocity(255,251);
			forward_mm(160);
			data='x';
			UDR0 = '!';
		}		
		else if(data=='r')
		{
			//velocity(255,255);
			//right_degrees(84);
			velocity(255,255);
			soft_right_degrees(87);
			back_mm(120);
			data='x';
			UDR0 = '!';
		}		
		else if(data=='l')
		{
			//velocity(255,255);
			//left_degrees(85);
			velocity(255,255);
			soft_left_degrees(87);
			back_mm(120);
			data='x';
			UDR0 = '!';
		}	
		else if(data=='b')
		{
			//velocity(255,255);
			left_degrees(178);
			back_mm(60);
			data='x';
			UDR0 = '!';
		}	
		else if(data=='1')
		{
			velocity(255,251);
			pick();
			data='x';
			UDR0 = '!';
			
		}
		else if(data=='0')
		{
			velocity(255,251);
			place();
			data='x';
			UDR0 = '!';
		}
		else if(data=='.')
		{
			buzzer_on();
			_delay_ms(5000);
			buzzer_off();
			data='x';
			UDR0 = '!';
		}
		
	}	
		_delay_ms(2000);
		servo_1_free();
		servo_2_free();
	/*
	forward_mm(102);
	_delay_ms(1000);
	right_degrees(90);
	_delay_ms(1000);
	left_degrees(90);
	_delay_ms(1000);*/
	
}