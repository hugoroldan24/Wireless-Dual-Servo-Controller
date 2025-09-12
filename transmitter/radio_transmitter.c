/*
 * Wireless Dual-Servo Controller
 * (c) 2025 Hugo Roldán López
 * This file is part of the project licensed under the MIT License.
 * See the LICENSE file in the root of the repository for full license text.
 */


/***********************************************************************************************
 * radio.c
 *
 * Brief Description:
 * This module provides low-level routines to configure and operate the nRF24L01+ RF transceiver
 * on the ATmega328P in transmitter mode. It includes:
 *   - Functions to write/read RF registers, send commands, set pipe addresses.
 *   - Initialization of the nRF24L01+ as a transmitter.
 *   - A function to send payloads without ACK. Since we don't need to receive any information from
       the receiver and the system is paquet-loss resilient, we can avoid the use of ACKs.
 *
 * Functions:
 *   - void writeRegister(uint8_t writeCommand, uint8_t conf);
 *       Writes a single byte configuration value to the specified nRF24L01+ register.
 *
 *   - uint8_t readRegister(uint8_t reg);
 *       Reads and returns the value of the specified nRF24L01+ register.
 *
 *   - void sendCommand(uint8_t command);
 *       Sends a single-byte command to the nRF24L01+ (e.g., FLUSH_TX).
 *
 *   - void writeAddress(uint8_t pipe, uint8_t *addr, uint8_t size);
 *       Writes an address (multiple bytes) to the given RX or TX address pipe.
 *
 *   - void RF_Transmitter_Init(void);
 *       Configures the nRF24L01+ as a transmitter: channel, data rate, address width,
 *       feature activation, config register, and flushes TX FIFO.
 *
 *   - void sendPaquet(uint8_t *data, uint8_t size);
 *       Sends a payload of specified size without auto-ACK. Asserts CE for the required pulse.
 *
 ***********************************************************************************************/
 
 
#include "spi.h"
#include "common.h"


/**
 * @brief  Write a configuration value to a specific nRF24L01+ register.
 * @param  writeCommand  The SPI write command register address.
 * @param  conf          The configuration byte to write into the register.
 */
void writeRegister(uint8_t writeCommand,uint8_t conf)
{
   PORTD &= ~(1 << SS_PIN);            	
   SPI_Send_Data(writeCommand);
   SPI_Send_Data(conf); 			
   PORTD |= (1 << SS_PIN);  
}


/**
 * @brief  Read and return a single-byte value from the specified nRF24L01+ register.
 * @param  reg  The SPI command/register address (R_REGISTER | register).
 * @return uint8_t data The byte read from that register.
 */
uint8_t readRegister(uint8_t reg)
{
   uint8_t data;
   PORTD &= ~(1 << SS_PIN);            	
   SPI_Send_Data(reg);
   SPI_Receive_Data(NOP,&data); 			
   PORTD |= (1 << SS_PIN);
   return data;   
}


/**
 * @brief  Send a single-byte command to the nRF24L01+ module.
 * @param  command  The command you want to send to the module (e.g., FLUSH_TX, NOP).
 */
void sendCommand(uint8_t command)
{
   PORTD &= ~(1 << SS_PIN);            
   SPI_Send_Data(command); 			
   PORTD |= (1 << SS_PIN);
}


/**
 * @brief  Write a multi-byte address into the specified pipe register.
 * @param  pipe   The SPI command for the target address register (e.g., W_TX_ADDR).
 * @param  addr   Pointer to the array containing the address bytes.
 * @param  size   Number of address bytes to send.
 */
void writeAddress(uint8_t pipe,uint8_t *addr,uint8_t size)
{
  PORTD &= ~(1 << SS_PIN);		
  SPI_Send_Data(pipe);
  for(int8_t l=0;l<size;l++){
     SPI_Send_Data(addr[l]);
  }
  PORTD |= (1 << SS_PIN); 
}


/**
 * @brief  Initialize the nRF24L01+ as a wireless transmitter.
 *         Configures CE pin, sets channel, data rate, address width,
 *         disables retransmit/auto-ack, activates TX_NO_ACK feature, and
 *         programs the CONFIG register for TX mode. Flushes TX FIFO.
 */
void RF_Transmitter_Init()
{
  uint8_t tx_address[] = {0xE7,0xE7,0xE7,0xE7,0xE7};
  
  DDRB |= (1 << DD_CE);		          		/* Configure CE pin as output */
  PORTB &= ~(1 << CE_PIN); 		  		       /* Ensure CE = 0 */
  
  _delay_us(10300); 					       /* Wait ~10.3 ms for power-down stabilization */
  writeRegister(W_RF_CH,0x04);         	        	/* Set RF channel to 2.404 GHz (avoid Wi-Fi) */          	    
  writeRegister(W_RF_SETUP,0x0F);			       /* 2 Mbps data rate, 0 dBm, enable LNA gain */
  writeRegister(W_SETUP_AW,0x03);			       /* Address width = 5 bytes */
  writeRegister(W_SETUP_RETR,0x00);	       	/* Disable auto-retransmit */
  writeRegister(W_STATUS,0x3E);				/* Clear TX_DS and MAX_RT flags */
  writeRegister(W_EN_AA,0x00);				/* Disable auto-acknowledgment on all pipes */
  writeAddress(W_TX_ADDR,tx_address,ADDRESS_WIDTH); 	/* Set TX address */   
  writeRegister(ACTIVATE,ACTIVATION_KEY); 	       /* Activate features (enable TX_NO_ACK) */
  writeRegister(W_FEATURE,0x01);	 		       /* Enable TX payload no-ACK feature */
  writeRegister(W_CONFIG,0x5A); 	 		       /* PWR_UP=1, PRIM_RX=0, enable CRC, mask interrupts */
  _delay_us(1500);                           	       /* Power-up delay (~1.5 ms) */  
 						              /* After configuration, module is in Standby-I until CE=1 */
  sendCommand(FLUSH_TX);                      		/* Flush TX FIFO */
}
  
  				  
/**
 * @brief  Send a data payload to the nRF24L01+ with W_TX_PAYLOAD_NOACK command.
 *         Does not wait for auto-ACK. Pulses CE for ~15 µs to transmit.
 * @param  joystick Union that contains both axis joystick payload
 */
void sendPaquet(JoystickData joystick)
{
   PORTD &= ~(1 << SS_PIN);        /* Chip Select ON */
   
   SPI_Send_Data(W_TX_NO_ACK);   	/* Write payload in TX FIFO without ACK */   	         	       
   SPI_Send_Data(joystick.x_axis);	/* Send joystick x_axis data */
   SPI_Send_Data(joystick.y_axis);	/* Send joystick y_axis data */
   
   PORTD |= (1 << SS_PIN); 	       /* Chip Select OFF */ 
   
   PORTB |= (1 << CE_PIN);         /* Set CE HIGH to transmit */
   _delay_us(15);			/* 15 µs CE pulse to trigger TX */
   PORTB &= ~(1 << CE_PIN);        /* Set CE LOW to return to Standby-I after packet is transmitted */               	 	
}  
