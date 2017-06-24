#include <avr/io.h>
#include <avr/interrupt.h>
#include <util/twi.h>
#include <util/delay.h>
#include <stdlib.h>
#include "hd44780.h"


// CO2 sensor I2C address
#define CO2_ADDR	0x15

// Function codes
#define READ_REG 	0x04
#define WRITE_COIL	0x05
#define WRITE_REG	0x06

// TWI definitions
#define TWCR_START  0xA5 //send START 
#define TWCR_SEND   0x85 //poke TWINT flag to send another byte 
#define TWCR_RACK   0xC5 //receive byte and return ACK to slave  
#define TWCR_RNACK  0x85 //receive byte and return NACK to slave
#define TWCR_RST    0x04 //reset TWI
#define TWCR_STOP   0x94 //send STOP,interrupt off, signals completion
#define TWI_BUFFER_SIZE 17  //SLA+RW (1 byte) +  16 data bytes (message size)
#define TWI_TWBR 72  //100khz TWI clock

// Build command transmit packets
// {slave address, function code, reg addr (MSB), reg addr (LSB), xx, xx}
// xx = (MSB), (LSB) of number of bytes to read if read command is sent, or
// 		output value if write command is sent.
uint8_t get_firmwr_pkt[5] = {READ_REG, 0x13, 0x89, 0x00, 0x01};
uint8_t get_status_pkt[5] = {READ_REG, 0x13, 0x8A, 0x00, 0x01};
uint8_t get_co2_pkt[5] = {READ_REG, 0x13, 0x8B, 0x00, 0x01};
uint8_t reset_pkt[5] = {WRITE_COIL, 0x03, 0xE8, 0xFF, 0x00};
uint8_t sp_calib_pkt[5] = {WRITE_COIL, 0x03, 0xEC, 0xFF, 0x00};
uint8_t stop_sp_calib_pkt[5] = {WRITE_COIL, 0x03, 0xEC, 0x00, 0x00};
uint8_t change_slave_addr[5] = {WRITE_REG, 0x0F, 0xA5, 0x00};	// the last byte is the new addr (1-247)
uint8_t enbl_abc_logic[5] = {WRITE_COIL, 0x03, 0xEE, 0xFF, 0x00};
uint8_t disbl_abc_logic[5] = {WRITE_COIL, 0x03, 0xEE, 0x00, 0x00};

uint16_t co2_ppm;
char ppm_str[5];

char lcd_buf[32] = "CO2 PPM:        ";
uint8_t twi_recv_buf[5];

volatile uint8_t  *twi_buf;      //pointer to the buffer we are xferred from/to
volatile uint8_t  twi_msg_size;  //number of bytes to be xferred
volatile uint8_t  twi_bus_addr;
volatile uint8_t  twi_state;     //status of transaction  


//******************************************************************************
//                            init_twi                               
//
//Uses PD1 as SDA and PD0 as SCL
//10K pullups are present on the board
//for alarm clock an additional 4.7K resistor is also there for pullup
//******************************************************************************

void twi_init(){
  TWDR = 0xFF;     //release SDA, default contents
  TWSR = 0x00;     //prescaler value = 1
  TWBR = TWI_TWBR; //defined in twi_master.h 
}


//******************************************************************************
//                            SPI_init                               
// Initialize the SPI bus. The LCD screen runs on SPI.

//******************************************************************************

void SPI_init() {
	// Set data direction registers for SPI
	DDRB = 0xF7;
	PINB = (1 << PB3);
	// set up SPI (master mode, clk low on idle, leading edge sample)
	SPCR = (1 << SPE) | (1 << MSTR) | (0 << CPOL) | (0 << CPHA);
	SPSR = (1 << SPI2X);
}//SPI_init


//*****************************************************************************
//Call this function to test if the TWI unit is busy transferring data. The TWI
//code uses the the interrupt enable bit (TWIE) to indicate if the TWI unit
//is busy or not.  This protocol must be maintained for correct operation.
//*****************************************************************************
uint8_t twi_busy(void){
  return (bit_is_set(TWCR,TWIE)); //if interrupt is enabled, twi is busy
}


//****************************************************************************
//Initiates a write transfer. Loads global variables. Sends START. ISR handles
//the rest.
//****************************************************************************
void twi_start_wr(uint8_t *twi_data, uint8_t byte_cnt){

  while(twi_busy());                    //wait till TWI rdy for next xfer
  twi_bus_addr = ((CO2_ADDR<<1) & ~TW_READ); //set twi bus address, mark as write 
  twi_buf = twi_data;                   //load pointer to write buffer
  twi_msg_size = byte_cnt;              //load size of xfer 
  TWCR = TWCR_START;                    //itemp_textnitiate START
}

//****************************************************************************
//Initiates a read transfer. Loads global variables. Sends START. ISR handles
//the rest.
//****************************************************************************
void twi_start_rd(uint8_t *twi_data, uint8_t byte_cnt){

  while(twi_busy());                   //wait till TWI rdy for next xfer
  twi_bus_addr = ((CO2_ADDR<<1) | TW_READ); //set twi bus address, mark as read  
  twi_buf = twi_data;                  //load pointer to write buffer
  twi_msg_size = byte_cnt;             //load size of xfer 
  TWCR = TWCR_START;                   //initiate START
}


//****************************************************************************
//This is the TWI ISR. Different actions are taken depending upon the value
//of the TWI status register TWSR.
//****************************************************************************/
ISR(TWI_vect){
  static uint8_t twi_buf_ptr;  //index into the buffer being used 

  switch (TWSR) {
    case TW_START:          //START has been xmitted, fall thorough
    case TW_REP_START:      //Repeated START was xmitted
      TWDR = twi_bus_addr;  //load up the twi bus address
      twi_buf_ptr = 0;      //initalize buffer pointer 
      TWCR = TWCR_SEND;     //send SLA+RW
      break;
    case TW_MT_SLA_ACK:     //SLA+W was xmitted and ACK rcvd, fall through 
    case TW_MT_DATA_ACK:                //Data byte was xmitted and ACK rcvd
      if (twi_buf_ptr < twi_msg_size){  //send data till done
        TWDR = twi_buf[twi_buf_ptr++];  //load next and postincrement index
        TWCR = TWCR_SEND;               //send next byte 
      }
      else{TWCR = TWCR_STOP;}           //last byte sent, send STOP 
      break;
    case TW_MR_DATA_ACK:                //Data byte has been rcvd, ACK xmitted, fall through
      twi_buf[twi_buf_ptr++] = TWDR;    //fill buffer with rcvd data
    case TW_MR_SLA_ACK:                 //SLA+R xmitted and ACK rcvd
      if (twi_buf_ptr < (twi_msg_size-1)){TWCR = TWCR_RACK;}  //ACK each byte
      else                               {TWCR = TWCR_RNACK;} //NACK last byte 
      break; 
    case TW_MR_DATA_NACK: //Data byte was rcvd and NACK xmitted
      twi_buf[twi_buf_ptr] = TWDR;      //save last byte to buffer
      TWCR = TWCR_STOP;                 //initiate a STOP
      break;      
    case TW_MT_ARB_LOST:                //Arbitration lost 
      TWCR = TWCR_START;                //initiate RESTART 
      break;
    default:                            //Error occured, save TWSR 
    	PORTB ^= PB7;
      twi_state = TWSR;         
      TWCR = TWCR_RST;                  //Reset TWI, disable interupts 
  }//switch
}//TWI_isr
//****************************************************************************



int main() {

	int j = 0;

	SPI_init();
	lcd_init();
	clear_display();
	twi_init();
	sei();

	while(1) {

		clear_display();
		twi_start_wr(get_co2_pkt, 5);
		_delay_ms(10);
		twi_start_rd(twi_recv_buf, 4);

		co2_ppm = (twi_recv_buf[2] << 8) | twi_recv_buf[3];
		itoa(co2_ppm, ppm_str, 10);

	for(j=0; j<5; j++) {
		lcd_buf[9+j] = ppm_str[j];
	}

		string2lcd(lcd_buf);
		_delay_ms(1000);


	}//while
	return 0;
}//main