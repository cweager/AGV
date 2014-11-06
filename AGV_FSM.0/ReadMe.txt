//*****************************************************************************
//**
//**  PROGRAM:  AGV_FSM
//**  AUTHOR:   Cody Eager, 1/c
//**
//**  MODULE:   AGV_FSM.ino
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
//**                *Note: The use of ISR prevents us from using Arduino the
//**                Servo library.  This algorithm uses PWM signals via Arduino
//**                analogWrite to move the AGV.  The following table relates
//**                the pulse width to motor response:
//**
//**                ARDUINO CX | SABERTOOTH STATE
//**                ----------------------------
//**                  1000us   |   REV, FULL
//**                  1500us   |   ALL STOP
//**                  2000us   |   FWD, FULL
//**                ----------------------------
//**
//**		    *Note: For Arduino CX to PWM Pulse Width mapping, see
//**		    PWM_values.xlsx.
//**
//*****************************************************************************

/*

L184 - CONFIGURING REGISTER TCCR2B TO MANIPULATE PWM FREQUENCY TO 4KHZ. DEBUGGING.
NOTE:	CONFIGURATION COMPLETE.

*/

*/

CURRENTLY WORKS FOR SERIAL INPUT.  USING AGV_ARDUINOMATLAB0.TXT FOR SERIAL INPUT FROM MATLAB.  ALSO, SEE PWM_EXAMPLE.INO FOR PWM DATA ACQUISITION.

NOTES:	Serial input pins are digital pins 0 (RX) and 1 (TX).  RX - Receive, TX - Transmit.  Connects USB (computer) to Pro-Mini Chip.

*/