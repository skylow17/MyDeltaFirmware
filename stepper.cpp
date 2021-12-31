/*
 * Author : E.Chancerel
 * Date : 31.12.2021
 * Hardware : Ender 3 Creality3D V1.1.4
 * Target Board : Sanguino
 * Target MCU : ATmega1284P (16MHz)
 */

#include "stepper.h"
#include "delta.h"
#include "pins.h"

void enable_axis()
{
  digitalWrite(X_Y_E0_ENABLE, MOT_ENABLE); 
  digitalWrite(Z_ENABLE, MOT_ENABLE);  
}
void disable_axis()
{
  digitalWrite(X_Y_E0_ENABLE, MOT_DISABLE);
  digitalWrite(Z_ENABLE, MOT_DISABLE);
}


/*
 * Motor : motor number. 0 = X / 1 = Y / 2 = Z
 * pulses  : number of pulses
 * dspeed : dspeed in pulse per second
 * dir : direction (DIR_INWARD or DIR_OUTWARD)
 */

void move_pulse(int Motor, long pulses, long dspeed , bool dir)
{
  int dir_pin, step_pin;

  //determine which motor to move
  switch(Motor)
  {
    case 0 : //X Motor
    dir_pin = X_DIR_PIN;
    step_pin = X_STEP_PIN;
    break;
    case 1 : //Y Motor
    dir_pin = Y_DIR_PIN;
    step_pin = Y_STEP_PIN;
    break;
    case 2 : //Z Motor
    dir_pin = Z_DIR_PIN;
    step_pin = Z_STEP_PIN;
    break;
    default : return -1;
    
  }
  digitalWrite(dir_pin, dir); //Set direction pin
  
  enable_axis();

  dspeed = 500000 / dspeed; //set time in microseconds and for half a period

  //generate pulse (to check and optimise with fastio)
  for(long i = 0; i <= pulses - 1; i++)
  {
    digitalWrite(step_pin, HIGH);
    delayMicroseconds(dspeed);
    digitalWrite(step_pin, LOW);
    delayMicroseconds(dspeed); 
  }
  
}

void move_theta(int Motor, double theta,long dspeed, bool dir)
{
  move_pulse(Motor, (theta * double(MICROSTEPPING) * double(PULLEY_RATIO) * double(MOT_STEP_PER_REV)) / 360.0, dspeed, dir);
}
