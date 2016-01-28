/*
 * meter_project.c
 *
 * Created: love u all 6:01:52 PM
 *  Author: Arnab Kumar Das
 */ 


#include<avr/io.h>                 
#include<avr/interrupt.h>
#include<stdlib.h>
#include<avr/LCD_mega16.h>
#define F_CPU 16000000UL
#include<util/delay.h>
#include<avr/eeprom.h>
#include<math.h>

//include nrf24l01
#include<nrf24l01/nrf24l01.h>

//role definitions
#define ROLETX 1
#define ROLERX 0

uint8_t nrf24l01_writestr(char *data)
{
	uint8_t i = 0;
	uint8_t ret = 0;

	//set tx mode
	nrf24l01_setTX();

	//write data
	nrf24l01_CSNlo; //low CSN
	spi_writereadbyte(NRF24L01_CMD_W_TX_PAYLOAD);
	for (i=0; i<NRF24L01_PAYLOAD; i++)
	spi_writereadbyte(data[i]);
	nrf24l01_CSNhi; //high CSN

	//start transmission
	nrf24l01_CEhi; //high CE
	_delay_us(15);
	nrf24l01_CElo; //low CE

	//stop if max_retries reached or send is ok
	do {
		_delay_us(10);
	}
	while( !(nrf24l01_getstatus() & (1<<NRF24L01_REG_MAX_RT | 1<<NRF24L01_REG_TX_DS)) );

	if(nrf24l01_getstatus() & 1<<NRF24L01_REG_TX_DS)
	ret = 1;

	//reset PLOS_CNT
	nrf24l01_writeregister(NRF24L01_REG_RF_CH, NRF24L01_CH);

	//power down
	nrf24l01_writeregister(NRF24L01_REG_CONFIG, nrf24l01_readregister(NRF24L01_REG_CONFIG) & ~(1<<NRF24L01_REG_PWR_UP));

	//set rx mode
	nrf24l01_setRX();

	return ret;
}
	
void init_interrupt();

long get_counts_m1();
long get_counts_m2();
long get_counts_m3();
long get_time();

volatile long global_counts_m1;
volatile long global_counts_m2;
volatile long global_counts_m3;

long EEMEM count1 = 1392;	//0.435
long EEMEM count2 = 1568;   //0.44
long EEMEM count3 = 13773;	//4.273


volatile int ms;
volatile int seconds;
volatile int minutes;
volatile int hours;

volatile char tag0;
volatile char tag1;
volatile char tag2;

volatile unsigned int load0;
volatile unsigned int load1;
volatile unsigned int load2;

volatile unsigned long time00;
volatile unsigned long time01;
volatile unsigned int timecalc0;

volatile unsigned long time10;
volatile unsigned long time11;
volatile unsigned int timecalc1;

volatile unsigned long time20;
volatile unsigned long time21;
volatile unsigned int timecalc2;

char a[10];
char b[10];
char c[10];

char m1[10];
char m2[10];
char m3[10];

uint8_t writeret;

unsigned long t1;
unsigned long t2;
unsigned long t3;

int main()
{	
	DDRC=0xFF;
	uint8_t txrxrole = 0; // 1 transmitter 0 receiver
	
	init_interrupt();
	lcd_init();

	//init nrf24l01
	nrf24l01_init();

	//init interrupt
	sei();

	txrxrole = ROLETX;         /////////////////////////////////////////////////
	
	//sending buffer addresses
	uint8_t addrtx0[NRF24L01_ADDRSIZE] = NRF24L01_ADDRP0;
	uint8_t addrtx1[NRF24L01_ADDRSIZE] = NRF24L01_ADDRP1;
	uint8_t addrtx2[NRF24L01_ADDRSIZE] = NRF24L01_ADDRP2;
	uint8_t addrtx3[NRF24L01_ADDRSIZE] = NRF24L01_ADDRP3;
	uint8_t addrtx4[NRF24L01_ADDRSIZE] = NRF24L01_ADDRP4;
	uint8_t addrtx5[NRF24L01_ADDRSIZE] = NRF24L01_ADDRP5;

		
	while(1)
	{
			cli();
	float m11=(get_counts_m1()*pow(3200,-1));
	float m22=(get_counts_m2()*pow(3200,-1));
	float m33=(get_counts_m3()*pow(3200,-1));
			
	dtostrf(m11,8,3,m1);
	dtostrf(m22,8,3,m2);
	dtostrf(m33,8,3,m3);
	sei();
	////////////////////////////////////////////////////////////////////////////////////
	
	nrf24l01_settxaddr(addrtx0);
	writeret = nrf24l01_writestr(m1);    //meter 1 Reading
	lcd_gotoxy1(0);
	lcd_string("Pipe 0");
	_delay_ms(1500);
	
	////////////////////////////////////////////////////////////////////////////////////
	
	nrf24l01_settxaddr(addrtx1);
	writeret = nrf24l01_writestr(a);    //meter 1 Load
	lcd_gotoxy1(0);
	lcd_string("Pipe 1");
	_delay_ms(1500);

	////////////////////////////////////////////////////////////////////////////////////
	
	nrf24l01_settxaddr(addrtx2);
	writeret = nrf24l01_writestr(m2);    //meter 2 Reading
	lcd_gotoxy1(0);
	lcd_string("Pipe 2");
	_delay_ms(1500);
	
	////////////////////////////////////////////////////////////////////////////////////
	
	nrf24l01_settxaddr(addrtx3);
	writeret = nrf24l01_writestr(b);    //meter 2 Load
	lcd_gotoxy1(0);
	lcd_string("Pipe 3");
	_delay_ms(1500);
	
	////////////////////////////////////////////////////////////////////////////////////
	
	nrf24l01_settxaddr(addrtx4);
	writeret = nrf24l01_writestr(m3);    //meter 3 Reading
	lcd_gotoxy1(0);
	lcd_string("Pipe 4");
	_delay_ms(1500);
	
	////////////////////////////////////////////////////////////////////////////////////
	
	nrf24l01_settxaddr(addrtx5);
	writeret = nrf24l01_writestr(c);    //meter 3 Load
	lcd_gotoxy1(0);
	lcd_string("Pipe 5");
	_delay_ms(1500);
	
	////////////////////////////////////////////////////////////////////////////////////
	if (tag0=0xff)
	{
		t1=get_time();
		if (t1>time00)
		{
			ltoa(0,a,10);
		}
	}
	if (tag1=0xff)
	{
		t2=get_time();
		if (t2>time10)
		{
			ltoa(0,b,10);
		}
	}
	if (tag2=0xff)
	{
		t3=get_time();
		if (t3>time20)
		{
			ltoa(0,c,10);
		}
	}
		
	}
	
}

ISR(INT0_vect)
{
	global_counts_m1 += 1;
	eeprom_update_dword(&count1,global_counts_m1);
	
	if (tag0==0x00)
	{
		time00=get_time();
		
	}

	if (tag0==0xff)
	{
		time01=get_time();
		timecalc0=time01-time00;
		load0=1125000/timecalc0;
		ltoa(load0,a,10);
	}
	tag0=~tag0;
}
ISR(INT1_vect)
{
	global_counts_m2 += 1;
	eeprom_update_dword(&count2,global_counts_m2);
	
	if (tag1==0x00)
	{
		time10=get_time();
	}

	if (tag1==0xff)
	{
		time11=get_time();
	 	timecalc1=time11-time10;
		load1=1125000/timecalc1;
		ltoa(load1,b,10);
	}
	tag1=~tag1;	
}
ISR(INT2_vect)
{
	global_counts_m3 += 1;
	eeprom_update_dword(&count3,global_counts_m3);
	
	if (tag2==0x00)
	{
		time20=get_time();
	}

	if (tag2==0xff)
	{
		time21=get_time();
		timecalc2=time21-time20;
		load2=1125000/timecalc2;
		ltoa(load2,c,10);
	}
	tag2=~tag2;
}

/*Timer Counter 1 Compare Match A Interrupt Service Routine/Interrupt Handler*/
ISR(TIMER1_COMPA_vect)
{	
	ms++;
	
	if(ms == 1000)
	{
		ms=0;
		seconds++;
	}
	if(seconds == 60)
	{
		seconds = 0;
		minutes++;
	}
	if(minutes == 60)
	{
		minutes = 0;
		hours++;
	}
	if(hours > 23)
	hours = 0;
}

void init_interrupt()
{


	// disable interrupts while initializing
	cli();
	// initialize the global state
	global_counts_m1 = eeprom_read_dword(&count1);
	global_counts_m2 = eeprom_read_dword(&count2);
	global_counts_m3 = eeprom_read_dword(&count3);
	
	
	ms=0;
	seconds=0;
	minutes=0;
	hours=0;
	
	tag0=0x00;
	tag1=0x00;
	tag2=0x00;
	
	 //timer is in "CTC" mode
	TCCR1B=(1<<CS10)|(1<<CS11)|(1<<WGM12);	   // Set up prescaler for CLK/64
	OCR1A = 250;
	TIMSK = 1<<OCIE1A;


	GICR=(1<<INT0)|(1<<INT1)|(1<<INT2);
	MCUCR = (1 << ISC11) | (1 << ISC10) | (1 << ISC01) | (1 << ISC00);
	MCUCSR=1<<ISC2;
			
	sei();
}

long get_counts_m1()
{
	cli();
	long tmp = global_counts_m1;
	sei();
	return tmp;
}
long get_counts_m2()
{
	cli();
	long tmp1 = global_counts_m2;
	sei();
	return tmp1;
}
long get_counts_m3()
{
	cli();
	long tmp2 = global_counts_m3;
	sei();
	return tmp2;
}
long get_time()
{
	cli();
	long time = ms + 1000*seconds + (long)1000*60*minutes + (long)1000*60*60*hours; 
	sei();
	return time;	
}
int get_load0()
{
	cli();
	int tmp7 = load0;
	sei();
	return tmp7;	
}
int get_load1()
{
	cli();
	int tmp8 = load1;
	sei();
	return tmp8;
}
int get_load2()
{
	cli();
	int tmp9 = load2;
	sei();
	return tmp9;
}
