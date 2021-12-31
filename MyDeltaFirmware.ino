/*
 * Author : E.Chancerel
 * Date : 31.12.2021
 * Hardware : Ender 3 Creality3D V1.1.4
 * Target Board : Sanguino
 * Target MCU : ATmega1284P (16MHz)
 */

#include "delta.h"
#include "stepper.h" 
#include "gcode.h"
#include "pins.h"

S_DELTA MyDelta;

gcode commands;

int CalcStatus = 0;
bool dir1, dir2, dir3 = 0;

void setup() {
  init_Delta(); //init pins paramters
  
  commands.begin("OK");
  MyDelta.X = 0;
  MyDelta.Y = 0;
  MyDelta.Z = 0;

  //Set initial arm angle, not accurate => to change
  MyDelta.currentTheta1 = 80; 
  MyDelta.currentTheta2 = 80;
  MyDelta.currentTheta3 = 80;
  
}

void loop() {

  //check if there is A G command in the buffer
  if(commands.available())
  {
    //check if there is a X coordinate available and store it
    if(commands.availableValue('X'))
    {
      MyDelta.X  = commands.GetValue('X');
    }
    //check if there is a Y coordinate available and store it
    if(commands.availableValue('Y'))
    {
      MyDelta.Y = commands.GetValue('Y');
      digitalWrite(LED_PIN, HIGH);  //for test purpose
    }
    
    //check if there is a Z coordinate available and store it
    if(commands.availableValue('Z'))
    {
      MyDelta.Z = commands.GetValue('Z');
    }

    //compute delta kinematics for each axis
    CalcStatus = delta_calcInverse(MyDelta.X, MyDelta.Y, MyDelta.Z, &MyDelta.NewTheta1, &MyDelta.NewTheta2, &MyDelta.NewTheta3);

    
    if(CalcStatus != 0) //handle kinematics computation error
    {
      commands.comment("can't calculate\n");
    }
    else
    {
      //determine motors direction based on current value and new value weight
      dir1 = (MyDelta.NewTheta1 > MyDelta.currentTheta1) ? (DIR_OUTWARD) : (DIR_INWARD);
      dir2 = (MyDelta.NewTheta2 > MyDelta.currentTheta2) ? (DIR_OUTWARD) : (DIR_INWARD);
      dir3 = (MyDelta.NewTheta3 > MyDelta.currentTheta3) ? (DIR_OUTWARD) : (DIR_INWARD);
      //calculate how much motor should move from current angle
      MyDelta.dTheta1 = MyDelta.currentTheta1-MyDelta.NewTheta1;
      MyDelta.dTheta2 = MyDelta.currentTheta2-MyDelta.NewTheta2;
      MyDelta.dTheta3 = MyDelta.currentTheta3-MyDelta.NewTheta3;
      //actuate motors
      //get abs value of delta Theta because direction is managed via the dirx parameter
      move_theta(0, abs(MyDelta.dTheta1), 6400, dir1);
      move_theta(1, abs(MyDelta.dTheta2), 6400, dir2);
      move_theta(2, abs(MyDelta.dTheta3), 6400, dir3);
      //disable_axis(); //only for test purpose
      //refresh current position for the next calculation
      MyDelta.currentTheta1 = MyDelta.NewTheta1;
      MyDelta.currentTheta2 = MyDelta.NewTheta2;
      MyDelta.currentTheta3 = MyDelta.NewTheta3;
        
    }
  }
     
}
