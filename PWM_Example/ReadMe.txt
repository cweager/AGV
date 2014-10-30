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
//*****************************************************************************

**PWM_Example is an example program that allows the user to set the pulse width
of the PWM pin 11 and 3 on an Arduino Pro-Mini.