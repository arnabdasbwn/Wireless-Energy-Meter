/*
Green House Project Receiver
*/

#include<stdio.h>
#include<stdlib.h>
#include<string.h>
#include<avr/io.h>
#include<avr/interrupt.h>
#define F_CPU 16000000UL
#include<util/delay.h>
#include<avr/LCD_mega16.h>

//include nrf24l01
#include<nrf24l01/nrf24l01.h>

//role definitions
#define ROLETX 1
#define ROLERX 0

//main here
int main(void)
{
	DDRD=0xff;
	PORTD=0x00;
	uint8_t txrxrole = 0; // 1 transmitter 0 receiver
	uint8_t i = 0;
	
	int l1=0;
	int l2=0;
	int l3=0;

	//nrf24l01 variables
	uint8_t bufferin[NRF24L01_PAYLOAD];

	lcd_init();
	
	//init nrf24l01
	nrf24l01_init();

	//init interrupt
	sei();

	txrxrole = ROLERX;         /////////////////////////////////////////////////
	
	//main loop
	for(;;)
	{
		uint8_t pipe = 0;
		if(nrf24l01_readready(&pipe))
		{ 
			//if data is ready
			//read buffer
			nrf24l01_read(bufferin);

			if (pipe==0)
			{
				lcd_init();
				lcd_gotoxy1(0);
				lcd_string("Meter 1 Reading");
				lcd_gotoxy2(0);
				lcd_string(bufferin);
				lcd_string(" KWhr");
			}
			if (pipe==1)
			{
				lcd_init();
				lcd_gotoxy1(0);
				lcd_string("Meter 1 Load");
				lcd_gotoxy2(0);
				lcd_string(bufferin);
				lcd_string(" W");
				l1=atoi(bufferin);
			}
			if (pipe==2)
			{
				lcd_init();
				lcd_gotoxy1(0);
				lcd_string("Meter 2 Reading");
				lcd_gotoxy2(0);
				lcd_string(bufferin);
				lcd_string(" KWhr");
			}
			if (pipe==3)
			{
				lcd_init();
				lcd_gotoxy1(0);
				lcd_string("Meter 2 Load");
				lcd_gotoxy2(0);
				lcd_string(bufferin);
				lcd_string(" W");
				l2=atoi(bufferin);
			}
			if (pipe==4)
			{
				lcd_init();
				lcd_gotoxy1(0);
				lcd_string("Meter 3 Reading");
				lcd_gotoxy2(0);
				lcd_string(bufferin);
				lcd_string(" KWhr");
			}
			if (pipe==5)
			{
				lcd_init();
				lcd_gotoxy1(0);
				lcd_string("Meter 3 Load");
				lcd_gotoxy2(0);
				lcd_string(bufferin);
				lcd_string(" W");
				l3=atoi(bufferin);
			}
			for(i=0; i<sizeof(bufferin); i++)
			bufferin[i] = 0;
			if(l1>1400 || l2>1400 || l3>1400)
			{
				PORTD=0xff;
			}
			
		}
	_delay_ms(1500);

	}
}





