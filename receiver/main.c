/*
 * Wireless Dual-Servo Controller
 * (c) 2025 Hugo Roldán López
 * This file is part of the project licensed under the MIT License.
 * See the LICENSE file in the root of the repository for full license text.
 */
 
 
/***********************************************************************************************
 * main.c
 *
 * Brief Description:
 * Entry point for the wireless dual-servo receiver. Initializes all required peripherals (SPI,
 * PWM timers, RF receiver) and enters a loop that waits for external interrupts from the nRF24L01+
 * indicating a received packet. Upon receiving data, it clears the RX_DR flag, reads joystick
 * values via SPI, and converts them to servo pulse widths which are applied on the next
 * Timer0 interrupt for synchronized PWM output.
 *
 ***********************************************************************************************/
 
 	
#include "spi.h"
#include "pwm.h"
#include "receiver.h"
#include "radio_receiver.h"
#include "common.h"

	
/**
 * @brief  Main function for the receiver firmware.
 *         1. Calls receiver_config() to enable interrupts and initialize SPI, PWM, and RF.
 *         2. Enters an infinite loop waiting for availableData flag set by RF_ISR.
 *         3. Clears the RX_DR status flag on the nRF24L01+.
 *         4. Reads joystick bytes from RX FIFO via SPI.
 *         5. Resets availableData flag.
 *         6. Converts raw X/Y joystick values into OCR1A/B pulse widths using linear interpolation.
 *         7. Waits for the next Timer0 ISR to update OCR1A/B for PWM output.
 * @return int  Always returns 0.
 */
int main(void)
{
  JoystickData joystick;    				 /* Declares the union joystick (defined in common.h file) */
  
  receiver_config(); 				          /* Initialize SPI, PWM timers, RF receiver, and enable listening */
  while(1){
  
     while(!availableData);					  /* Wait until nRF24L01+ external interrupt sets availableData */
     
     availableData = 0;  				 	  /* Reset flag */
     
     writeRegister(W_STATUS,(1<<RX_DS));      /* Clear RX_DS flag on nRF24L01+ */
     
     get_Received_Data(&joystick); 			  /* Read two bytes from RX_FIFO and saves it in the joystick variable */ 
     
     Convert_Value_PWM(joystick);   		  /* Converts raw X/Y joystick values into OCR1A/B pulse widths using linear interpolation */
     
     /* Servo pulse widths are updated in TIMER0_COMPA_vect ISR on next sync */
   }
     
  return 0;	
}
