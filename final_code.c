/*
* Team Id: eYRC-LM#2855
* Author List: Aarya R. Shankar
* Filename: all_functions.c
* Theme: Launch A Module
* Functions:  motion_pin_config(),buzzer_pin_config(),buzzer_on(),buzzer_off(),init_timer5(),,lcd_port_config(),left_encoder_pin_config(),right_encoder_pin_config(),right_position_encoder_interrupt_init(),left_position_encoder_interrupt_init()
			  motion_set(char),forward(),back(),left(),right(),soft_left(),soft_right(),angle_rotate(int),linear_distance_mm(int),forward_mm(int),back_mm(int),left_degrees(int),right_degrees(int),soft_right_degrees(int),soft_left_degrees(int),adc_pin_config()
			  adc_init(),velocity(int,int),port_init(),init_devices(),ADC_Conversion(char),print_sensor(char,char,int)
* Global Variables: ADC_Value,Left_white_line,Right_white_line,Center_white_line,ShaftCountLeft,ShaftCountRight,Degrees,DistanceShaft
*/


#include "all_functions.c"

unsigned char data;
unsigned int start;
unsigned char move[4];

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

void pick(void)
{
	forward_mm(40);
	_delay_ms(1000);
	servo_1(0);
	_delay_ms(250);
	servo_2(0);
	_delay_ms(250);
	servo_1(60);
	_delay_ms(30);
	forward_mm(160);
	_delay_ms(1000);
	
}

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
	servo_1(60);
	_delay_ms(30);
	servo_2(60);
	_delay_ms(30);
	while(1) {
	if (data=='*')
	{	start = 1;
	}
	else if (data=='#')
	{	start = 0;
		UDR0 = '!';
	}
	if (start == 1)
	{	
		if(data=='f')
		{
			forward_mm(203);
			data='x';
			UDR0 = '!';
		}		
		else if(data=='r')
		{
			right_degrees(100);
			data='x';
			UDR0 = '!';
		}		
		else if(data=='l')
		{
			left_degrees(100);
			data='x';
			UDR0 = '!';
		}		
		else if(data=='1')
		{
			pick();
			data='x';
			UDR0 = '!';
			
		}
		else if(data=='0')
		{
			place();
			data='x';
			UDR0 = '!';
		}
		else if(data=='!')
		{
			buzzer_on();
			_delay_ms(5000);
			buzzer_off();
			data='x';
			UDR0 = '!';
		}
		
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