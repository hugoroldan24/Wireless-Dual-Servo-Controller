/*
 * Wireless Dual-Servo Controller
 * (c) 2025 Hugo Roldán López
 * This file is part of the project licensed under the MIT License.
 * See the LICENSE file in the root of the repository for full license text.
 */
 
 
 /***********************************************************************************************************
 * main.c
 *
 * Brief Description:
 * Entry point for the wireless dual-servo transmitter. Initializes all required peripherals
 * (SPI, ADC auto-trigger, ADC, and nRF24L01+ as transmitter) and enters a loop that waits
 * for ADC conversions to complete. Once both joystick channels are sampled, it packages the
 * data and sends it via RF.
 *
 * Workflow:
 *   1. transmitter_config() initialice ADC, ADC Auto-trigger, SPI, RF module as TX and starts the conversions
 *   2. Timer1 triggers ADC conversions; each completed conversion sets sendData flag.
 *   3. ISR(ADC_vect) stores the new ADC sample in obtainedData and records lastChannel.
 *   4. In main loop, once both channels are read (lastChannel == NUM_ELEMENTS-1), call
 *      sendPaquet() to transmit the data packet via nRF24L01+.
 *
 ************************************************************************************************************/
 
 
#include "common.h"
#include "transmitter.h"
#include "radio_transmitter.h"
#include "adc.h"


/**
 * @brief  Main function for the transmitter firmware.
 *         Initializes all transmitter-side peripherals and enters an infinite loop that:
 *           - Waits for ADC conversion to complete (sendData flag).
 *           - Stores the obtained sample in the joystick.axis array at index lastChannel.
 *           - When both joystick channels are sampled (lastChannel == NUM_ELEMENTS-1),
 *             calls sendPaquet() to send the full packet via the RF module.
 * @return int  Always returns 0.
 */
int main(void)
{
  int8_t lastChannel;
  JoystickData joystick;	           /* Declares the union that contains the joystick data (defined in common.h file) */
  
  transmitter_config();	   	        /* Initialices all the transmitter features*/
  while(1){				       
     while(!sendData);              /* Wait until an ADC conversion is complete */
     sendData = 0;
       
     lastChannel = getLastChannel();
     joystick.axis[lastChannel] = readADC();  /* Save the converted value in the axis array at the lastChannel index */
    
     if(lastChannel == NUM_ELEMENTS-1){
        sendPaquet(joystick);    	           /* Sends the joystick union containing the converted data from both axis */  
     }
     
  }  
  
  return 0;
}
