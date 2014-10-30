//*****************************************************************************
//**
//**  PROGRAM:  PWM_Example
//**  AUTHOR:   Cody Eager, 1/c
//**
//**  MODULE:   PWM_Example.ino
//**  DATE:     28 October 2014
//**
//**  DESCRIPTION:  Example of PWM using pins 3 and 11 on Arduino Pro-Mini
//**                running an AtMega-328. The following table shows the
//**                register command to pulse width relationship:
//**
//**                ARDUINO CX | PULSE WIDTH  
//**                --------------------------
//**                   XXX     |     1000us   
//**                   XXX     |     1500us   
//**                   XXX     |     2000us   
//**                --------------------------
//**
//**  NOTE: The following table represents the associated frequency for the
//**  configuration of the TCCR2B register for timer 2:
//**
//**                SETTING    |     DIVISOR  |  Frequency  
//**                --------------------------------------
//**                   0x01    |      1       |   31372Hz  
//**                   0x02    |      8       |   3921Hz
//**                   0x03    |      32      |   980Hz
//**                   0x04    |      64      |   490Hz
//**                   0x05    |      128     |   245Hz
//**                   0x06    |      256     |   122Hz
//**                   0x07    |      1024    |   30Hz
//**                --------------------------------------
//** 
//**               TCCR2B = TCCR2B & 0b11111000 | <setting>;
//**
//*****************************************************************************


// AVR GCC libraries for more information see:
//     http://www.nongnu.org/avr-libc/user-manual/modules.html
//     https://www.gnu.org/software/libc/manual/

#include <avr/io.h>
#include <avr/interrupt.h>
#include <stdint.h>
#include <string.h>
#include <ctype.h>

// Arduino libraries: see http://arduino.cc/en/Reference/Libraries

//#include <Servo.h> // Arduino Library will not operate with use of ISRs
                     // Recommend using analogWrite to use PWM signals for motors

// Project specific headers

#include "configuration.h"
#include "USART.h"

// Global variables

//NONE


void setup(){ 
  
  // Initialize and Configure USART (serial monitor communications)
  
  USART_init(F_CLK, BAUD_RATE);
  USART_set_terminator(LINE_TERMINATOR);
  
  // Initialize PWM pins
  
  pinMode(S1_PIN, OUTPUT);
  pinMode(S2_PIN, OUTPUT);
  
  // Set all output pins
  
  digitalWrite(S1_PIN, LOW);
  digitalWrite(S2_PIN, LOW);
  
  // Configure registers for PWM
  
  TCCR2A = _BV(COM2A1) | _BV(COM2B1) | _BV(WGM21) | _BV(WGM20);
  TCCR2B = _BV(CS22); //This register controls PWM frequency
  TCCR2B = TCCR2B & 0b11111000 | 0x06; // PWM Frequency
  OCR2A = 0; //Set PWM of S2_PIN (11)
  OCR2B = 0; //Set PWM of S1_PIN (3)
  
  // USART Monitor Banner
  
  USART_puts("//*************************************************************************************\n");
  USART_puts("//**\n");
  USART_puts("//**  PROGRAM:  PWM_Example\n//**  AUTHOR:   Cody Eager, 1/c\n");
  USART_puts("//**\n");
  USART_puts("//**  MODULE:   PWM_Example.ino\n//**  DATE:     28 October 2014\n");
  USART_puts("//**\n");
  USART_puts("//*************************************************************************************\n\n");
  
  USART_puts("Specify Pin and Pulse Width...\n\n");
  USART_puts("** Pin:  ");
}


/*********************************************************************************
 *
 *  INTERUPT SERVICE ROUTINES
 *
 ********************************************************************************/


ISR(USART_RX_vect){

  /**
   * @note This Interrupt Service Routine is called when a new character is received by the USART.
   * Idealy it would have been placed in the USART.cpp file but there is a error "multiple definition 
   * of vector_18".  Apparently Arduino detects when an ISR is in the main sketch.  If you place it 
   * somewhere else it is missed and replaced with the Arduino handler.  This is the source of the 
   * multiple definitions error -  * see discussion @ http://forum.arduino.cc/index.php?topic=42153.0
   */
  USART_handle_ISR();
}


/*********************************************************************************
 *
 *  LOOP
 *
 ********************************************************************************/

void loop(){
  
  char buf[50];
  uint8_t red_f = 0;
  uint8_t green_f = 0;
  
  if(USART_is_string()){
    USART_gets(buf);      // Get user's specified pin
    if (!strcmp(buf,"3") || !strcmp(buf,"RED") || !strcmp(buf, "red") || !strcmp(buf, "Red")){
      red_f = 1;
    }
    else if (!strcmp(buf,"11") || !strcmp(buf,"GREEN") || !strcmp(buf, "green") || !strcmp(buf, "Green")){
      green_f = 1;
    }
    else {
      USART_puts("Warning!  Invalid pin selection.  Please select either Pin 3 or Pin 11.");
    }
    if (red_f){
      USART_puts(strcat(buf, "\n** Width: "));
      while(!USART_is_string());
      USART_gets(buf);
      OCR2B = atoi(buf);   // Set Pin 3
      red_f = 0;
    }
    else if (green_f){
      USART_puts(strcat(buf, "\n** Width: "));
      while(!USART_is_string());
      USART_gets(buf);
      OCR2A = atoi(buf);  // Set Pin 11
      green_f = 0;
    }
    USART_puts(strcat(buf,"\n\n** Pin:  ")); 
  }
}
