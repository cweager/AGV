//*****************************************************************************
//**
//**  PROGRAM:  AGV_FSM
//**  AUTHOR:   Cody Eager, 1/c
//**
//**  MODULE:   AGV_FSM.0.ino
//**  DATE:     21 October 2014
//**
//**  DESCRIPTION:  AGV_FSM is the software control algorithm used to control
//**                AGV movement.  The Arduino on-board AGV receives navigation
//**                commands from the on-board processor (MATLAB) in the form
//**                of integer numbers.  Each integer (0-9) represents a state
//**                that MATLAB commands the Arduino to enter.  The following
//**                table represents the MATLAB integer command and the
//**                commanded state:
//**
//**                MATLAB CX  |  ARDUINO STATE
//**                ---------------------------
//**                    1      |      STOP
//**                    2      |      FWD
//**                    3      |      FWD, LFT
//**                    4      |      FWD, RGT
//**                    5      |      REV
//**                    6      |      REV, LFT
//**                    7      |      REV, RGT
//**                    8      |      SPIN LFT
//**                    9      |      SPIN RGT
//**                    0      |      N/A
//**                ---------------------------
//**
//**                *Note: Prototype includes state control using serial
//**                commands from the serial monitor.
//**
//**                *Note: The use of ISR prevents us from using Arduino the
//**                Servo library.  This algorithm uses PWM signals via compare register
//**                TCCR2B to move the AGV.  The following table relates
//**                the pulse width to motor response:
//**
//**                ARDUINO CX | PULSE WIDTH  | SABERTOOTH STATE
//**                --------------------------------------------
//**                   62      |     1008us   |   REV, FULL
//**                   93      |     1504us   |   ALL STOP
//**                   124     |     2000us   |   FWD, FULL
//**                --------------------------------------------
//**
//**                NOTE: The following table represents the associated frequency for the
//**                configuration of the TCCR2B register for timer 2:
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


/*
 * Autonomous Ground Vehicle FSM Algorithm
 *
 * Copyright 2014 Cody W. Eager 
 *
 *     cody.w.eager@gmail.com
 *
 * This program is free software: you can redistribute it and/or modify it under the terms
 * of the GNU General Public License as published by the Free Software Foundation, either
 * version 3 of the License, or (at your option) any later version.
 *
 * This program is distributed in the hope that it will be useful,but WITHOUT ANY WARRANTY;
 * without even the implied warranty of MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.
 * See the GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License along with this program.
 * If not, see <http://www.gnu.org/licenses/>.
 */

/*
* @Warning - there is a bug between the timer ISR and the Arduino analogWrite
 * function.  If you need to use the PWM from within the ISR then use:
 *
 * http://arduino.cc/en/Tutorial/SecretsOfArduinoPWM
 * 
 *    void setup(){
 *        .
 *        .
 *        .
 *        pinMode(3, OUTPUT);
 *        pinMode(11, OUTPUT);
 *        TCCR2A = _BV(COM2A1) | _BV(COM2B1) | _BV(WGM21) | _BV(WGM20);
 *        TCCR2B = _BV(CS22);                                               // This register controls PWM frequency
 *        OCR2A = 0;                                                        // Arduino I/O pin # 11 duty cycle
 *        OCR2B = 0;                                                        // Arduino I/O pin # 3 duty cycle
 *        .
 *        .
 *        .
 *    }
 *
 *    void loop(){
 *        .
 *        .
 *        .
 *        OCR2A = pin_11_PWM_DC;
 *        OCR2B = pin_3_PWM_DC;
 *        .
 *        .
 *        .
 *    }
 */


// AVR GCC libraries for more information see:
//     http://www.nongnu.org/avr-libc/user-manual/modules.html
//     https://www.gnu.org/software/libc/manual/

#include <avr/io.h>
#include <avr/interrupt.h>
#include <stdint.h>
#include <string.h>
#include <ctype.h>

// Arduino libraries: see http://arduino.cc/en/Reference/Libraries

//#include <Servo.h> // Arduino Library will not operate with use of ISR on Timer 2
                     // Must reconfigure compare register TCCR2B to perform manual PWM operations

// Project specific headers

#include "configuration.h"
#include "USART.h"

// Global variables

typedef enum{ 

  initial, all_stop, fwd, fwd_lft, fwd_rgt, rev, rev_lft, rev_rgt, lft_spin, rgt_spin

}
e_states;

volatile static e_states state = initial;  //Initialize enumerated type

// State Flags

/*
volatile uint8_t all_stop_f = 0;
volatile uint8_t fwd_f = 0;
volatile uint8_t fwd_lft_f = 0;
volatile uint8_t fwd_rgt_f = 0;
volatile uint8_t rev_f = 0;
volatile uint8_t rev_lft_f = 0;
volatile uint8_t rev_rgt_f = 0;
volatile uint8_t lft_spin_f = 0;
volatile uint8_t rgt_spin_f = 0;
*/

volatile uint8_t command_f = 0;


/*********************************************************************************
 *
 *  SET UP
 *
 ********************************************************************************/


void setup(){ 
  
  // Initialize and Configure USART (serial monitor communications)
  
  USART_init(F_CLK, BAUD_RATE);
  USART_set_terminator(LINE_TERMINATOR);
  
  // Initialize ISR timer
  
  init_timer_1_CTC(100);
  
  // Initialize motor control pins
  
  pinMode(S1_PIN, OUTPUT);
  pinMode(S2_PIN, OUTPUT);
  
  // Initialize Arduino LED pin
  
  pinMode(LED_PIN, OUTPUT);
  
  // Set all output pins
  
  digitalWrite(S1_PIN, LOW);
  digitalWrite(S2_PIN, LOW);
  digitalWrite(LED_PIN, LOW);
  
  // Configure registers for PWM
  
  TCCR2A = _BV(COM2A1) | _BV(COM2B1) | _BV(WGM21) | _BV(WGM20);
  TCCR2B = _BV(CS22); //This register controls PWM frequency
  TCCR2B = TCCR2B & 0b11111000 | 0x06; // PWM Frequency
  OCR2A = 0; //Set PWM of S2_PIN (11)
  OCR2B = 0; //Set PWM of S1_PIN (3)
  
  // USART Monitor Banner
  
  USART_puts("//*************************************************************************************\n");
  USART_puts("//**\n");
  USART_puts("//**  PROGRAM:  AGV_FSM\n//**  AUTHOR:   Cody Eager, 1/c\n");
  USART_puts("//**\n");
  USART_puts("//**  MODULE:   AGV_FSM.0.ino\n//**  DATE:     20 October 2014\n");
  USART_puts("//**\n");
  USART_puts("//*************************************************************************************\n\n");
  
  USART_puts ("**  AGV States Initializing...\n");  
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




ISR(TIMER1_COMPA_vect){

  /** @brief This Interrupt Service Routine (ISR) serves as the 100 Hz heartbeat 
   * for the Arduino.  See the companion function init_timer_1_CTC for additional 
   * information.
   *
   * @ Note:
   *    1) Compiler generated code pushes status register and any used registers to stack.
   *    2) Calling a subroutine from the ISR causes compiler to save all 32 registers; a 
   *       slow operation (fact check). 
   *    3) Status and used registers are popped by compiler generated code.
   */
  
  static uint8_t LED_count = 0;
  LED_count++;

  if (LED_count <= 49){
    digitalWrite(LED_PIN, HIGH);
  }
  else{
    digitalWrite(LED_PIN, LOW);
    if (LED_count == 99){
      LED_count = 0;
    }
  }
  
  static uint16_t state_time = 0; // Time in state based on 100Hz incrementer

  switch(state){

  case initial:
    OCR2B = 95; //Set PWM of S1_PIN to 1500us
    OCR2A = 95; //Set PWM of S2_PIN to 1500us
    // Automatically enter All Stop state after 2.5 seconds
    if (++state_time > 250){
      state = all_stop;
      state_time = 0;
    }
    break;

  case all_stop:
    OCR2B = 95; //Set PWM of S1_PIN to 1500us
    OCR2A = 95; //Set PWM of S2_PIN to 1500us
    break;

  case fwd:
    if (command_f){
      state_time = 0;
      command_f = 0;
    }
    else if (++state_time > 500 && command_f == 0){
      state = all_stop;
      state_time = 0;
    }
    
    OCR2B = 128; //Set PWM of S1_PIN to 2000us
    OCR2A = 95; //Set PWM of S2_PIN to 1500us
    break;

  case fwd_lft:
    if (command_f){
      state_time = 0;
      command_f = 0;
    }
    else if (++state_time > 500 && command_f == 0){
      state = all_stop;
      state_time = 0;
    }
    OCR2B = 110; //Set PWM of S1_PIN to 1650us
    OCR2A = 100; //Set PWM of S2_PIN to 1600us
    break;

  case fwd_rgt:
    if (command_f){
      state_time = 0;
      command_f = 0;
    }
    else if (++state_time > 500 && command_f == 0){
      state = all_stop;
      state_time = 0;
    }
    OCR2B = 110; //Set PWM of S1_PIN to 1650us
    OCR2A = 80; //Set PWM of S2_PIN to 1600us
    break;

  case rev:
    if (command_f){
      state_time = 0;
      command_f = 0;
    }
    else if (++state_time > 500 && command_f == 0){
      state = all_stop;
      state_time = 0;
    }
    OCR2B = 60; //Set PWM of S1_PIN to 1000us
    OCR2A = 95; //Set PWM of S2_PIN to 1500us
    break;

  case rev_lft:
    if (command_f){
      state_time = 0;
      command_f = 0;
    }
    else if (++state_time > 500 && command_f == 0){
      state = all_stop;
      state_time = 0;
    }
    OCR2B = 75; //Set PWM of S1_PIN to 1450us
    OCR2A = 85; //Set PWM of S2_PIN to 1400us
    break;

  case rev_rgt:
    if (command_f){
      state_time = 0;
      command_f = 0;
    }
    else if (++state_time > 500 && command_f == 0){
      state = all_stop;
      state_time = 0;
    }
    OCR2B = 75;  //Set PWM of S1_PIN to 1550us
    OCR2A = 100; //Set PWM of S2_PIN to 1600us
    break;

  case lft_spin:
    if (command_f){
      state_time = 0;
      command_f = 0;
    }
    else if (++state_time > 500 && command_f == 0){
      state = all_stop;
      state_time = 0;
    }
    OCR2B = 95; //Set PWM of S1_PIN to 1500us
    OCR2A = 111; //Set PWM of S2_PIN to 1750
    break;

  case rgt_spin:
    if (command_f){
      state_time = 0;
      command_f = 0;
    }
    else if (++state_time > 500 && command_f == 0){
      state = all_stop;
      state_time = 0;
    }
    OCR2B = 95; //Set PWM of S1_PIN to 1500us
    OCR2A = 68; //Set PWM of S2_PIN to 1250
    break;

  default:
    state = all_stop;
    break; 
  }

}


/*********************************************************************************
 *
 *  LOOP
 *
 ********************************************************************************/

void loop(){

  char buf[50];
  static uint8_t last_state = !initial;
  
  //int8_t state_CX = -1;
  //state_CX = Serial.read();  // NOTE:  Command uses "vector_18" - Compiler error
                               // Experiment with analogRead(inputPin)

  /*  **********************************
   *
   *  This section of code is executed
   *  ONCE per state change.
   *
   **********************************  */

  if(state != last_state){

    switch(state){

    case initial:
      USART_puts ("**  AGV States Initialized...\n\n");
      break;

    case all_stop:
      if (last_state != all_stop && last_state != initial && command_f == 0){
        USART_puts("COMMAND TIME OUT\n\n");
      }
      USART_puts ("**  State:    All Stop\n");
      USART_puts ("**  Command:  ");
      break;

    case fwd:
      USART_puts ("**  State:    Forward\n");
      USART_puts ("**  Command:  ");
      break;

    case fwd_lft:
      USART_puts ("**  State:    Forward and Left\n");
      USART_puts ("**  Command:  ");
      break;

    case fwd_rgt:
      USART_puts ("**  State:    Forward and Right\n");
      USART_puts ("**  Command:  ");
      break;

    case rev:
      USART_puts ("**  State:    Reverse\n");
      USART_puts ("**  Command:  ");
      break;

    case rev_lft:
      USART_puts ("**  State:    Reverse and Left\n");
      USART_puts ("**  Command:  ");
      break;

    case rev_rgt:
      USART_puts ("**  State:    Reverse and Right\n");
      USART_puts ("**  Command:  ");
      break;

    case lft_spin:
      USART_puts ("**  State:    Spin Left\n");
      USART_puts ("**  Command:  ");
      break;

    case rgt_spin:
      USART_puts ("**  State:    Spin Right\n");
      USART_puts ("**  Command:  ");
      break;

    default:
      break; 
    }

    last_state = state;

  }

  /*  **********************************
   *
   *  This section of code is executed
   *  EVERY loop iteration.
   *
   *  This section is useful to receive
   *  state commands quickly.
   *
   **********************************  */
   
  
  /*  **********************************
   *
   *  This section of code handles
   *  user commands from the serial
   *  monitor.
   *
   **********************************  */
 
   
  //  Quickly Receive State Commands from Serial Monitor via USART
    //Only enter if statement if a command is entered into USART, else skip
  
  //if(USART_is_string()){
    USART_gets(buf);  
    if (!strcmp(buf,"1") || !strcmp(buf, "STOP")){
      state = all_stop;
      command_f = 1;
      USART_puts(strcat(buf, "\n\n"));
    }
    else if(!strcmp(buf,"2") || !strcmp(buf, "FWD")){
      state = fwd;
      command_f = 1;
      USART_puts(strcat(buf, "\n\n"));
    }
    else if(!strcmp(buf,"3") || !strcmp(buf,"FWD_LFT")){
      state = fwd_lft;
      command_f = 1;
      USART_puts(strcat(buf, "\n\n"));
    }
    else if(!strcmp(buf,"4") || !strcmp(buf, "FWD_RGT")){
      state = fwd_rgt;
      command_f = 1;
      USART_puts(strcat(buf, "\n\n"));
    }
    else if(!strcmp(buf,"5") || !strcmp(buf, "REV")){
      state = rev;
      command_f = 1;
      USART_puts(strcat(buf, "\n\n"));
    }
    else if(!strcmp(buf,"6") || !strcmp(buf, "REV_LFT")){
      state = rev_lft;
      command_f = 1;
      USART_puts(strcat(buf, "\n\n"));
    }
    else if(!strcmp(buf,"7") || !strcmp(buf, "REV_RGT")){
      state = rev_rgt;
      command_f = 1;
      USART_puts(strcat(buf, "\n\n"));
    }
    else if(!strcmp(buf,"8") || !strcmp(buf, "LFT_SPIN")){
      state = lft_spin;
      command_f = 1;
      USART_puts(strcat(buf, "\n\n"));
    }
    else if(!strcmp(buf,"9") || !strcmp(buf, "RGT_SPIN")){
      state = rgt_spin;
      command_f = 1;
      USART_puts(strcat(buf, "\n\n"));
    }
    else {
      USART_puts(buf);
      USART_puts("\n** Invalid command. Please type an integer value from 1-9.\n\n");
      USART_puts("** Command:  ");
    }
  //}
  
  /*
  /*  **********************************
   *
   *  This section of code handles
   *  AGV commands from the MATLAB
   *
   **********************************  /
   
  
  //  Quickly Receive State Commands from MATLAB
    //Only enter if statement if a command is supplied by MATLAB, else skip
    
  if (state_CX != -1){   
    if (state_CX == 1){
      state = all_stop;
      command_f = 1;
    }
    else if(state_CX == 2){
      state = fwd;
      command_f = 1;
    }
    else if(state_CX == 3){
      state = fwd_lft;
      command_f = 1;
    }
    else if(state_CX == 4){
      state = fwd_rgt;
      command_f = 1;
    }
    else if(state_CX == 5){
      state = rev;
      command_f = 1;
    }
    else if(state_CX == 6){
      state = rev_lft;
      command_f = 1;
    }
    else if(state_CX == 7){
      state = rev_rgt;
      command_f = 1;
    }
    else if(state_CX == 8){
      state = lft_spin;
      command_f = 1;
    }
    else if(state_CX == 9){
      state = rgt_spin;
      command_f = 1;
    }
  }
  */
  /*
  switch(state){

  case initial:
    break;

  case all_stop:
    break;

  case fwd:
    break;

  case fwd_lft:
    break;

  case fwd_rgt:
    break;

  case rev:
    break;

  case rev_lft:
    break;

  case rev_rgt:
    break;

  case lft_spin:
    break;

  case rgt_spin:
    break;

  default:
    break; 
  }
  */
}


/*********************************************************************************
 *
 *  FUNCTION DEFNS
 *
 ********************************************************************************/


void init_timer_1_CTC(long desired_ISR_freq){
  /**
   * @brief Configure timer #1 to operate in Clear Timer on Capture Match (CTC Mode)
   *
   *      desired_ISR_freq = (F_CLK / prescale value) /  Output Compare Registers
   *
   *   For example:
   *        Given an Arduino Uno: F_clk = 16 MHz
   *        let prescale                = 64
   *        let desired ISR heartbeat   = 100 Hz
   *
   *        it follows that OCR1A = 2500
   *
   * @param desired_ISR_freq is the desired operating frequency of the ISR
   * @param F_CLK must be defined globally e.g., #define F_CLK 16000000L
   *
   * @return void
   *
   * @note The prescale value is set manually in this function.  Refer to ATMEL ATmega48A/PA/88A/PA/168A/PA/328/P datasheet for specific settings.
   *
   * @warning There are no checks on the desired_ISR_freq parameter.  Use this function with caution.
   *
   * @warning Use of this code will break the Arduino Servo() library.
   */
  cli();                                          // Disable global
  TCCR1A = 0;                                     // Clear timer counter control registers.  The initial value is zero but it
  TCCR1B = 0;                                     // appears Arduino code modifies them on startup...
  TCCR1B |= (1 << WGM12);                         // Timer #1 using CTC mode
  TIMSK1 |= (1 << OCIE1A);                        // Enable CTC interrupt
  TCCR1B |= (1 << CS10)|(1 << CS11);              // Prescale: divide by F_CLK by XXXXL.  Note SC12 already cleared
  OCR1A = (F_CLK / 64L) / desired_ISR_freq;       // Interrupt when TCNT1 equals the top value of the counter specified by OCR
  sei();                                          // Enable global
}
