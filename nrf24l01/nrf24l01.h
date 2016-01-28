/*
nrf24l01 lib 0x02

copyright (c) Davide Gironi, 2012

References:
  -  This library is based upon nRF24L01 avr lib by Stefan Engelke
     http://www.tinkerer.eu/AVRLib/nRF24L01
  -  and arduino library 2011 by J. Coliz
     http://maniacbug.github.com/RF24

Released under GPLv3.
Please refer to LICENSE file for licensing information.
*/

#include <avr/io.h>
#include <spi/spi.h> //spi lib path
#include <avr/io.h>
#include <avr/interrupt.h>
#include <avr/delay.h>
#include <string.h>
#include <stdio.h>


#define NRF24L01_REG_CONFIG      0x00
#define NRF24L01_REG_EN_AA       0x01
#define NRF24L01_REG_EN_RXADDR   0x02
#define NRF24L01_REG_SETUP_AW    0x03
#define NRF24L01_REG_SETUP_RETR  0x04
#define NRF24L01_REG_RF_CH       0x05
#define NRF24L01_REG_RF_SETUP    0x06
#define NRF24L01_REG_STATUS      0x07
#define NRF24L01_REG_OBSERVE_TX  0x08
#define NRF24L01_REG_CD          0x09
#define NRF24L01_REG_RX_ADDR_P0  0x0A
#define NRF24L01_REG_RX_ADDR_P1  0x0B
#define NRF24L01_REG_RX_ADDR_P2  0x0C
#define NRF24L01_REG_RX_ADDR_P3  0x0D
#define NRF24L01_REG_RX_ADDR_P4  0x0E
#define NRF24L01_REG_RX_ADDR_P5  0x0F
#define NRF24L01_REG_TX_ADDR     0x10
#define NRF24L01_REG_RX_PW_P0    0x11
#define NRF24L01_REG_RX_PW_P1    0x12
#define NRF24L01_REG_RX_PW_P2    0x13
#define NRF24L01_REG_RX_PW_P3    0x14
#define NRF24L01_REG_RX_PW_P4    0x15
#define NRF24L01_REG_RX_PW_P5    0x16
#define NRF24L01_REG_FIFO_STATUS 0x17
#define NRF24L01_REG_FEATURE     0x1D
#define NRF24L01_REG_DYNPD	     0x1C

/* Bit Mnemonics */
#define NRF24L01_REG_MASK_RX_DR  6
#define NRF24L01_REG_MASK_TX_DS  5
#define NRF24L01_REG_MASK_MAX_RT 4
#define NRF24L01_REG_EN_CRC      3
#define NRF24L01_REG_CRCO        2
#define NRF24L01_REG_PWR_UP      1
#define NRF24L01_REG_PRIM_RX     0
#define NRF24L01_REG_ENAA_P5     5
#define NRF24L01_REG_ENAA_P4     4
#define NRF24L01_REG_ENAA_P3     3
#define NRF24L01_REG_ENAA_P2     2
#define NRF24L01_REG_ENAA_P1     1
#define NRF24L01_REG_ENAA_P0     0
#define NRF24L01_REG_ERX_P5      5
#define NRF24L01_REG_ERX_P4      4
#define NRF24L01_REG_ERX_P3      3
#define NRF24L01_REG_ERX_P2      2
#define NRF24L01_REG_ERX_P1      1
#define NRF24L01_REG_ERX_P0      0
#define NRF24L01_REG_AW          0
#define NRF24L01_REG_ARD         4
#define NRF24L01_REG_ARC         0
#define NRF24L01_REG_PLL_LOCK    4
#define NRF24L01_REG_RF_DR       3
#define NRF24L01_REG_RF_PWR      1
#define NRF24L01_REG_LNA_HCURR   0
#define NRF24L01_REG_RX_DR       6
#define NRF24L01_REG_TX_DS       5
#define NRF24L01_REG_MAX_RT      4
#define NRF24L01_REG_RX_P_NO     1
#define NRF24L01_REG_TX_FULL     0
#define NRF24L01_REG_PLOS_CNT    4
#define NRF24L01_REG_ARC_CNT     0
#define NRF24L01_REG_TX_REUSE    6
#define NRF24L01_REG_FIFO_FULL   5
#define NRF24L01_REG_TX_EMPTY    4
#define NRF24L01_REG_RX_FULL     1
#define NRF24L01_REG_RX_EMPTY    0
#define NRF24L01_REG_RPD         0x09
#define NRF24L01_REG_RF_DR_LOW   5
#define NRF24L01_REG_RF_DR_HIGH  3
#define NRF24L01_REG_RF_PWR_LOW  1
#define NRF24L01_REG_RF_PWR_HIGH 2

/* Instruction Mnemonics */
#define NRF24L01_CMD_R_REGISTER    0x00
#define NRF24L01_CMD_W_REGISTER    0x20
#define NRF24L01_CMD_REGISTER_MASK 0x1F
#define NRF24L01_CMD_R_RX_PAYLOAD  0x61
#define NRF24L01_CMD_W_TX_PAYLOAD  0xA0
#define NRF24L01_CMD_FLUSH_TX      0xE1
#define NRF24L01_CMD_FLUSH_RX      0xE2
#define NRF24L01_CMD_REUSE_TX_PL   0xE3
#define NRF24L01_CMD_NOP           0xFF

//CE and CSN port definitions
#define NRF24L01_DDR DDRB
#define NRF24L01_PORT PORTB
#define NRF24L01_CE PB0
#define NRF24L01_CSN PB1


//CE and CSN functions
#define nrf24l01_CSNhi NRF24L01_PORT |= (1<<NRF24L01_CSN);
#define nrf24l01_CSNlo NRF24L01_PORT &= ~(1<<NRF24L01_CSN);
#define nrf24l01_CEhi NRF24L01_PORT |=  (1<<NRF24L01_CE);
#define nrf24l01_CElo NRF24L01_PORT &= ~(1<<NRF24L01_CE);

//power setup
#define NRF24L01_RF24_PA_MIN 1
#define NRF24L01_RF24_PA_LOW 2
#define NRF24L01_RF24_PA_HIGH 3
#define NRF24L01_RF24_PA_MAX 4
#define NRF24L01_RF24_PA NRF24L01_RF24_PA_MAX

//speed setup
#define NRF24L01_RF24_SPEED_250KBPS 1
#define NRF24L01_RF24_SPEED_1MBPS 2
#define NRF24L01_RF24_SPEED_2MBPS 3
#define NRF24L01_RF24_SPEED NRF24L01_RF24_SPEED_1MBPS

//crc setup
#define NRF24L01_RF24_CRC_DISABLED 1
#define NRF24L01_RF24_CRC_8 2
#define NRF24L01_RF24_CRC_16 3
#define NRF24L01_RF24_CRC NRF24L01_RF24_CRC_16

//transmission channel
#define NRF24L01_CH 76

//payload lenght
#define NRF24L01_PAYLOAD 16

//auto ack enabled
#define NRF24L01_ACK 1

//auto retransmit delay and count
#define NRF24L01_RETR (0b0100 << NRF24L01_REG_ARD) | (0b0111 << NRF24L01_REG_ARC) //1500uS, 15 times

//enable / disable pipe
#define NRF24L01_ENABLEDP0 1 //pipe 0
#define NRF24L01_ENABLEDP1 1 //pipe 1
#define NRF24L01_ENABLEDP2 1 //pipe 2
#define NRF24L01_ENABLEDP3 1 //pipe 3
#define NRF24L01_ENABLEDP4 1 //pipe 4
#define NRF24L01_ENABLEDP5 1 //pipe 5

//address size
#define NRF24L01_ADDRSIZE 5

//pipe address
#define NRF24L01_ADDRP0 {0xE8, 0xE8, 0xF0, 0xF0, 0xE2} //pipe 0, 5 byte address
#define NRF24L01_ADDRP1 {0xC1, 0xC2, 0xC2, 0xC2, 0xC2} //pipe 1, 5 byte address
#define NRF24L01_ADDRP2 {0xC1, 0xC2, 0xC2, 0xC2, 0xC3} //pipe 2, 5 byte address
#define NRF24L01_ADDRP3 {0xC1, 0xC2, 0xC2, 0xC2, 0xC4} //pipe 3, 5 byte address
#define NRF24L01_ADDRP4 {0xC1, 0xC2, 0xC2, 0xC2, 0xC5} //pipe 4, 5 byte address
#define NRF24L01_ADDRP5 {0xC1, 0xC2, 0xC2, 0xC2, 0xC6} //pipe 5, 5 byte address
#define NRF24L01_ADDRTX {0xE8, 0xE8, 0xF0, 0xF0, 0xE2} //tx default address*/

 //enable print info function
#define NRF24L01_PRINTENABLE 1

extern void nrf24l01_init();
extern uint8_t nrf24l01_getstatus();
extern uint8_t nrf24l01_readready();
extern void nrf24l01_read(uint8_t *data);
extern uint8_t nrf24l01_write(uint8_t *data);
extern uint8_t nrf24l01_writestr(char *data);
extern void nrf24l01_setrxaddr(uint8_t channel, uint8_t *addr);
extern void nrf24l01_settxaddr(uint8_t *addr);
#if NRF24L01_PRINTENABLE == 1
extern void nrf24l01_printinfo(void(*prints)(const char *), void(*printc)(unsigned char data));
#endif

//address variables
static uint8_t nrf24l01_addr0[NRF24L01_ADDRSIZE] = NRF24L01_ADDRP0;
static uint8_t nrf24l01_addr1[NRF24L01_ADDRSIZE] = NRF24L01_ADDRP1;
static uint8_t nrf24l01_addr2[NRF24L01_ADDRSIZE] = NRF24L01_ADDRP2;
static uint8_t nrf24l01_addr3[NRF24L01_ADDRSIZE] = NRF24L01_ADDRP3;
static uint8_t nrf24l01_addr4[NRF24L01_ADDRSIZE] = NRF24L01_ADDRP4;
static uint8_t nrf24l01_addr5[NRF24L01_ADDRSIZE] = NRF24L01_ADDRP5;
static uint8_t nrf24l01_addrtx[NRF24L01_ADDRSIZE] = NRF24L01_ADDRTX;

/*
 * read one register
 */
uint8_t nrf24l01_readregister(uint8_t reg) {
	nrf24l01_CSNlo; //low CSN
	spi_writereadbyte(NRF24L01_CMD_R_REGISTER | (NRF24L01_CMD_REGISTER_MASK & reg));
    uint8_t result = spi_writereadbyte(NRF24L01_CMD_NOP); //read write register
    nrf24l01_CSNhi; //high CSN
    return result;
}

/*
 * read many registers
 */
void nrf24l01_readregisters(uint8_t reg, uint8_t *value, uint8_t len) {
	uint8_t i = 0;
	nrf24l01_CSNlo; //low CSN
	spi_writereadbyte(NRF24L01_CMD_R_REGISTER | (NRF24L01_CMD_REGISTER_MASK & reg));
	for(i=0; i<len; i++)
		value[i] = spi_writereadbyte(NRF24L01_CMD_NOP); //read write register
	nrf24l01_CSNhi; //high CSN
}

/*
 * write one register
 */
void nrf24l01_writeregister(uint8_t reg, uint8_t value) {
	nrf24l01_CSNlo; //low CSN
	spi_writereadbyte(NRF24L01_CMD_W_REGISTER | (NRF24L01_CMD_REGISTER_MASK & reg));
	spi_writereadbyte(value); //write register
	nrf24l01_CSNhi; //high CSN
}

/*
 * write many registers
 */
void nrf24l01_writeregisters(uint8_t reg, uint8_t *value, uint8_t len) {
	uint8_t i = 0;
	nrf24l01_CSNlo; //low CSN
    spi_writereadbyte(NRF24L01_CMD_W_REGISTER | (NRF24L01_CMD_REGISTER_MASK & reg));
	for(i=0; i<len; i++)
		 spi_writereadbyte(value[i]); //write register
	nrf24l01_CSNhi; //high CSN
}

/*
 * reverse an array, NRF24L01 expects LSB first
 */
void nrf24l01_revaddress(uint8_t *addr, uint8_t *addrrev) {
	//reverse address
	uint8_t i = 0;
	for(i=0; i<NRF24L01_ADDRSIZE; i++)
		memcpy(&addrrev[i], &addr[NRF24L01_ADDRSIZE-1-i], 1);
}

/*
 * set rx address
 */
void nrf24l01_setrxaddr(uint8_t pipe, uint8_t *addr) {
	if(pipe == 0) {
		memcpy(&nrf24l01_addr0, addr, NRF24L01_ADDRSIZE); //cache address
		uint8_t addrrev[NRF24L01_ADDRSIZE];
		nrf24l01_revaddress(addr, (uint8_t *)addrrev);
    	nrf24l01_writeregisters(NRF24L01_REG_RX_ADDR_P0, addrrev, NRF24L01_ADDRSIZE);
	} else if(pipe == 1) {
		memcpy(&nrf24l01_addr1, addr, NRF24L01_ADDRSIZE); //cache address
		uint8_t addrrev[NRF24L01_ADDRSIZE];
		nrf24l01_revaddress(addr, (uint8_t *)addrrev);
    	nrf24l01_writeregisters(NRF24L01_REG_RX_ADDR_P1, addrrev, NRF24L01_ADDRSIZE);
	} else if(pipe == 2) {
		memcpy(&nrf24l01_addr2, addr, NRF24L01_ADDRSIZE); //cache address
		nrf24l01_writeregister(NRF24L01_REG_RX_ADDR_P2, addr[NRF24L01_ADDRSIZE-1]); //write only LSB MSBytes are equal to RX_ADDR_P
	} else if(pipe == 3) {
		memcpy(&nrf24l01_addr3, addr, NRF24L01_ADDRSIZE); //cache address
		nrf24l01_writeregister(NRF24L01_REG_RX_ADDR_P3, addr[NRF24L01_ADDRSIZE-1]); //write only LSB MSBytes are equal to RX_ADDR_P
	} else if(pipe == 4) {
		memcpy(&nrf24l01_addr4, addr, NRF24L01_ADDRSIZE); //cache address
		nrf24l01_writeregister(NRF24L01_REG_RX_ADDR_P4, addr[NRF24L01_ADDRSIZE-1]); //write only LSB MSBytes are equal to RX_ADDR_P
	} else if(pipe == 5) {
		memcpy(&nrf24l01_addr5, addr, NRF24L01_ADDRSIZE); //cache address
		nrf24l01_writeregister(NRF24L01_REG_RX_ADDR_P5, addr[NRF24L01_ADDRSIZE-1]); //write only LSB MSBytes are equal to RX_ADDR_P
	}
}

/*
 * set tx address
 */
void nrf24l01_settxaddr(uint8_t *addr) {
	memcpy(&nrf24l01_addrtx, addr, NRF24L01_ADDRSIZE); //cache address
	uint8_t addrrev[NRF24L01_ADDRSIZE];
	nrf24l01_revaddress(addr, (uint8_t *)addrrev);
	nrf24l01_writeregisters(NRF24L01_REG_RX_ADDR_P0, addrrev, NRF24L01_ADDRSIZE); //set rx address for ack on pipe 0
	nrf24l01_writeregisters(NRF24L01_REG_TX_ADDR, addrrev, NRF24L01_ADDRSIZE); //set tx address
}

/*
 * flush RX fifo
 */
void nrf24l01_flushRXfifo() {
	nrf24l01_CSNlo; //low CSN
	spi_writereadbyte(NRF24L01_CMD_FLUSH_RX);
	nrf24l01_CSNhi; //high CSN
}

/*
 * flush RX fifo
 */
void nrf24l01_flushTXfifo() {
	nrf24l01_CSNlo; //low CSN
	spi_writereadbyte(NRF24L01_CMD_FLUSH_TX);
	nrf24l01_CSNhi; //high CSN
}

/*
 * set chip as RX
 */
void nrf24l01_setRX() {
	nrf24l01_setrxaddr(0, nrf24l01_addr0); //restore pipe 0 address
	nrf24l01_writeregister(NRF24L01_REG_CONFIG, nrf24l01_readregister(NRF24L01_REG_CONFIG) | (1<<NRF24L01_REG_PRIM_RX)); //prx mode
	nrf24l01_writeregister(NRF24L01_REG_CONFIG, nrf24l01_readregister(NRF24L01_REG_CONFIG) | (1<<NRF24L01_REG_PWR_UP)); //power up
	nrf24l01_writeregister(NRF24L01_REG_STATUS, (1<<NRF24L01_REG_RX_DR) | (1<<NRF24L01_REG_TX_DS) | (1<<NRF24L01_REG_MAX_RT)); //reset status
	nrf24l01_flushRXfifo(); //flush rx fifo
	nrf24l01_flushTXfifo(); //flush tx fifo
	nrf24l01_CEhi; //start listening
	_delay_us(150); //wait for the radio to power up
}

/*
 * set chip as TX
 */
void nrf24l01_setTX() {
	nrf24l01_CElo; //stop listening
	nrf24l01_writeregister(NRF24L01_REG_CONFIG, nrf24l01_readregister(NRF24L01_REG_CONFIG) & ~(1<<NRF24L01_REG_PRIM_RX)); //ptx mode
	nrf24l01_writeregister(NRF24L01_REG_CONFIG, nrf24l01_readregister(NRF24L01_REG_CONFIG) | (1<<NRF24L01_REG_PWR_UP)); //power up
	nrf24l01_writeregister(NRF24L01_REG_STATUS, (1<<NRF24L01_REG_RX_DR) | (1<<NRF24L01_REG_TX_DS) | (1<<NRF24L01_REG_MAX_RT)); //reset status
	nrf24l01_flushTXfifo(); //flush tx fifo
	_delay_us(150); //wait for the radio to power up
}

#if NRF24L01_PRINTENABLE == 1
/*
 * print info
 */
void nrf24l01_printinfo(void(*prints)(const char *), void(*printc)(unsigned char data)) {
	char buff[100];
	prints("info\r\n");
	sprintf(buff,"STATUS: %02X\r\n", nrf24l01_getstatus()); prints(buff);
	sprintf(buff,"CONFIG: %02X\r\n", nrf24l01_readregister(NRF24L01_REG_CONFIG)); prints(buff);
	sprintf(buff,"RF_CH: %02X\r\n", nrf24l01_readregister(NRF24L01_REG_RF_CH)); prints(buff);
	sprintf(buff,"RF_SETUP: %02X\r\n", nrf24l01_readregister(NRF24L01_REG_RF_SETUP)); prints(buff);
	sprintf(buff,"EN_AA: %02X\r\n", nrf24l01_readregister(NRF24L01_REG_EN_AA)); prints(buff);
	sprintf(buff,"EN_RXADDR: %02X\r\n", nrf24l01_readregister(NRF24L01_REG_EN_RXADDR)); prints(buff);
	sprintf(buff,"OBSERVE_TX: %02X\r\n", nrf24l01_readregister(NRF24L01_REG_OBSERVE_TX)); prints(buff);
	prints("\r\n");
}
#endif


/*
 * get status register
 */
uint8_t nrf24l01_getstatus() {
	uint8_t status = 0;
	nrf24l01_CSNlo; //low CSN
	status = spi_writereadbyte(NRF24L01_CMD_NOP); //get status, send NOP request
	nrf24l01_CSNhi; //high CSN
	return status;
}

/*
 * check if there is data ready
 */
uint8_t nrf24l01_readready(uint8_t* pipe) {
    uint8_t status = nrf24l01_getstatus();
    uint8_t ret = status & (1<<NRF24L01_REG_RX_DR);
    if(ret) {
		//get the pipe number
		if(pipe)
			*pipe = (status >> NRF24L01_REG_RX_P_NO) & 0b111;
    }
    return ret;
}

/*
 * get data
 */
void nrf24l01_read(uint8_t *data) {
	uint8_t i = 0;
	//read rx register
	nrf24l01_CSNlo; //low CSN
    spi_writereadbyte(NRF24L01_CMD_R_RX_PAYLOAD);
    for(i=0; i<NRF24L01_PAYLOAD; i++)
    	data[i] = spi_writereadbyte(NRF24L01_CMD_NOP);
    nrf24l01_CSNhi; //high CSN
    //reset register
    nrf24l01_writeregister(NRF24L01_REG_STATUS, (1<<NRF24L01_REG_RX_DR));
    //handle ack payload receipt
	if (nrf24l01_getstatus() & (1<<NRF24L01_REG_TX_DS))
		nrf24l01_writeregister(NRF24L01_REG_STATUS, (1<<NRF24L01_REG_TX_DS));
}

/*
 * put data
 */
uint8_t nrf24l01_write(uint8_t *data) {
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

/*
 * set power level
 */
void nrf24l01_setpalevel() {
  uint8_t setup = nrf24l01_readregister(NRF24L01_REG_RF_SETUP);
  setup &= ~((1<<NRF24L01_REG_RF_PWR_LOW) | (1<<NRF24L01_REG_RF_PWR_HIGH));

  if (NRF24L01_RF24_PA == NRF24L01_RF24_PA_MAX) {
	  setup |= (1<<NRF24L01_REG_RF_PWR_LOW) | (1<<NRF24L01_REG_RF_PWR_HIGH);
  } else if (NRF24L01_RF24_PA == NRF24L01_RF24_PA_HIGH) {
	  setup |= (1<<NRF24L01_REG_RF_PWR_HIGH) ;
  } else if (NRF24L01_RF24_PA == NRF24L01_RF24_PA_LOW) {
	  setup |= (1<<NRF24L01_REG_RF_PWR_LOW);
  } else if (NRF24L01_RF24_PA == NRF24L01_RF24_PA_MIN) {
  } else {
	  //default is max power
	  setup |= (1<<NRF24L01_REG_RF_PWR_LOW) | (1<<NRF24L01_REG_RF_PWR_HIGH);
  }

  nrf24l01_writeregister(NRF24L01_REG_RF_SETUP, setup);
}

/*
 * set datarate
 */
void nrf24l01_setdatarate() {
  uint8_t setup = nrf24l01_readregister(NRF24L01_REG_RF_SETUP) ;

  setup &= ~((1<<NRF24L01_REG_RF_DR_LOW) | (1<<NRF24L01_REG_RF_DR_HIGH));
  if(NRF24L01_RF24_SPEED == NRF24L01_RF24_SPEED_250KBPS) {
    setup |= (1<<NRF24L01_REG_RF_DR_LOW);
  } else {
    if (NRF24L01_RF24_SPEED == NRF24L01_RF24_SPEED_2MBPS) {
    	setup |= (1<<NRF24L01_REG_RF_DR_HIGH);
    } else if (NRF24L01_RF24_SPEED == NRF24L01_RF24_SPEED_2MBPS) {
    } else {
    	//default is 1Mbps
    }
  }

  nrf24l01_writeregister(NRF24L01_REG_RF_SETUP, setup);
}

/*
 * set crc length
 */
void nrf24l01_setcrclength() {
  uint8_t config = nrf24l01_readregister(NRF24L01_REG_CONFIG) & ~((1<<NRF24L01_REG_CRCO) | (1<<NRF24L01_REG_EN_CRC));

  if (NRF24L01_RF24_CRC == NRF24L01_RF24_CRC_DISABLED) {
	  //nothing
  } else if (NRF24L01_RF24_CRC == NRF24L01_RF24_CRC_8) {
	  config |= (1<<NRF24L01_REG_EN_CRC);
  } else if (NRF24L01_RF24_CRC == NRF24L01_RF24_CRC_16) {
	  config |= (1<<NRF24L01_REG_EN_CRC);
	  config |= (1<<NRF24L01_REG_CRCO);
  } else {
	  //default is disabled
  }

  nrf24l01_writeregister(NRF24L01_REG_CONFIG, config);
}
//write string to nrf
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

/*
 * init nrf24l01
 */
void nrf24l01_init() {
	//setup port
	NRF24L01_DDR |= (1<<NRF24L01_CSN); //output
	NRF24L01_DDR |= (1<<NRF24L01_CE); //output

    spi_init(); //init spi

    nrf24l01_CElo; //low CE
    nrf24l01_CSNhi; //high CSN

    _delay_ms(5); //wait for the radio to init

    nrf24l01_setpalevel(); //set power level
    nrf24l01_setdatarate(); //set data rate
    nrf24l01_setcrclength(); //set crc length
    nrf24l01_writeregister(NRF24L01_REG_SETUP_RETR, NRF24L01_RETR); // set retries
    nrf24l01_writeregister(NRF24L01_REG_DYNPD, 0); //disable dynamic payloads
    nrf24l01_writeregister(NRF24L01_REG_RF_CH, NRF24L01_CH); //set RF channel

	//payload size
	#if NRF24L01_ENABLEDP0 == 1
		nrf24l01_writeregister(NRF24L01_REG_RX_PW_P0, NRF24L01_PAYLOAD);
	#endif
	#if NRF24L01_ENABLEDP1 == 1
		nrf24l01_writeregister(NRF24L01_REG_RX_PW_P1, NRF24L01_PAYLOAD);
	#endif
	#if NRF24L01_ENABLEDP2 == 1
		nrf24l01_writeregister(NRF24L01_REG_RX_PW_P2, NRF24L01_PAYLOAD);
	#endif
	#if NRF24L01_ENABLEDP3 == 1
		nrf24l01_writeregister(NRF24L01_REG_RX_PW_P3, NRF24L01_PAYLOAD);
	#endif
	#if NRF24L01_ENABLEDP4 == 1
		nrf24l01_writeregister(NRF24L01_REG_RX_PW_P4, NRF24L01_PAYLOAD);
	#endif
	#if NRF24L01_ENABLEDP5 == 1
		nrf24l01_writeregister(NRF24L01_REG_RX_PW_P5, NRF24L01_PAYLOAD);
	#endif

	//enable pipe
	nrf24l01_writeregister(NRF24L01_REG_EN_RXADDR, 0);
	#if NRF24L01_ENABLEDP0 == 1
		nrf24l01_writeregister(NRF24L01_REG_EN_RXADDR, nrf24l01_readregister(NRF24L01_REG_EN_RXADDR) | (1<<NRF24L01_REG_ERX_P0));
	#endif
	#if NRF24L01_ENABLEDP1 == 1
		nrf24l01_writeregister(NRF24L01_REG_EN_RXADDR, nrf24l01_readregister(NRF24L01_REG_EN_RXADDR) | (1<<NRF24L01_REG_ERX_P1));
	#endif
	#if NRF24L01_ENABLEDP2 == 1
		nrf24l01_writeregister(NRF24L01_REG_EN_RXADDR, nrf24l01_readregister(NRF24L01_REG_EN_RXADDR) | (1<<NRF24L01_REG_ERX_P2));
	#endif
	#if NRF24L01_ENABLEDP3 == 1
		nrf24l01_writeregister(NRF24L01_REG_EN_RXADDR, nrf24l01_readregister(NRF24L01_REG_EN_RXADDR) | (1<<NRF24L01_REG_ERX_P3));
	#endif
	#if NRF24L01_ENABLEDP4 == 1
		nrf24l01_writeregister(NRF24L01_REG_EN_RXADDR, nrf24l01_readregister(NRF24L01_REG_EN_RXADDR) | (1<<NRF24L01_REG_ERX_P4));
	#endif
	#if NRF24L01_ENABLEDP5 == 1
		nrf24l01_writeregister(NRF24L01_REG_EN_RXADDR, nrf24l01_readregister(NRF24L01_REG_EN_RXADDR) | (1<<NRF24L01_REG_ERX_P5));
	#endif

	//auto ack
	#if NRF24L01_ACK == 1
		nrf24l01_writeregister(NRF24L01_REG_EN_AA, nrf24l01_readregister(NRF24L01_REG_EN_AA) | (1<<NRF24L01_REG_ENAA_P0));
		nrf24l01_writeregister(NRF24L01_REG_EN_AA, nrf24l01_readregister(NRF24L01_REG_EN_AA) | (1<<NRF24L01_REG_ENAA_P1));
		nrf24l01_writeregister(NRF24L01_REG_EN_AA, nrf24l01_readregister(NRF24L01_REG_EN_AA) | (1<<NRF24L01_REG_ENAA_P2));
		nrf24l01_writeregister(NRF24L01_REG_EN_AA, nrf24l01_readregister(NRF24L01_REG_EN_AA) | (1<<NRF24L01_REG_ENAA_P3));
		nrf24l01_writeregister(NRF24L01_REG_EN_AA, nrf24l01_readregister(NRF24L01_REG_EN_AA) | (1<<NRF24L01_REG_ENAA_P4));
		nrf24l01_writeregister(NRF24L01_REG_EN_AA, nrf24l01_readregister(NRF24L01_REG_EN_AA) | (1<<NRF24L01_REG_ENAA_P5));
	#else
		nrf24l01_writeregister(NRF24L01_REG_EN_AA, nrf24l01_readregister(NRF24L01_REG_EN_AA) & ~(1<<NRF24L01_REG_ENAA_P0));
		nrf24l01_writeregister(NRF24L01_REG_EN_AA, nrf24l01_readregister(NRF24L01_REG_EN_AA) & ~(1<<NRF24L01_REG_ENAA_P1));
		nrf24l01_writeregister(NRF24L01_REG_EN_AA, nrf24l01_readregister(NRF24L01_REG_EN_AA) & ~(1<<NRF24L01_REG_ENAA_P2));
		nrf24l01_writeregister(NRF24L01_REG_EN_AA, nrf24l01_readregister(NRF24L01_REG_EN_AA) & ~(1<<NRF24L01_REG_ENAA_P3));
		nrf24l01_writeregister(NRF24L01_REG_EN_AA, nrf24l01_readregister(NRF24L01_REG_EN_AA) & ~(1<<NRF24L01_REG_ENAA_P4));
		nrf24l01_writeregister(NRF24L01_REG_EN_AA, nrf24l01_readregister(NRF24L01_REG_EN_AA) & ~(1<<NRF24L01_REG_ENAA_P5));
	#endif

	//rx address
	nrf24l01_setrxaddr(0, nrf24l01_addr0);
	nrf24l01_setrxaddr(1, nrf24l01_addr1);
	nrf24l01_setrxaddr(2, nrf24l01_addr2);
	nrf24l01_setrxaddr(3, nrf24l01_addr3);
	nrf24l01_setrxaddr(4, nrf24l01_addr4);
	nrf24l01_setrxaddr(5, nrf24l01_addr5);

	//tx address
	nrf24l01_settxaddr(nrf24l01_addrtx);

	//set rx mode
	nrf24l01_setRX();
}
