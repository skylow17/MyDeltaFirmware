/*
 * Author : E.Chancerel
 * Date : 31.12.2021
 * Hardware : Ender 3 Creality3D V1.1.4
 * Target Board : Sanguino
 * Target MCU : ATmega1284P (16MHz)
 */

#include <Arduino.h>
#include "delta.h"
#include "stepper.h"
#include "pins.h"


void init_Delta()
{
  pinMode(LED_PIN, OUTPUT);
  pinMode(X_STEP_PIN, OUTPUT);
  pinMode(X_DIR_PIN, OUTPUT);
  pinMode(Y_STEP_PIN, OUTPUT);
  pinMode(Y_DIR_PIN, OUTPUT);
  pinMode(Z_STEP_PIN, OUTPUT);
  pinMode(Z_DIR_PIN, OUTPUT);
  pinMode(X_Y_E0_ENABLE, OUTPUT);
  pinMode(Z_ENABLE, OUTPUT);
  

  //init motor coils
  enable_axis();
  disable_axis();
}


/*
 * compute 1 axis angle based on x,y,z single coordinate
 * from https://hypertriangle.com/~alex/delta-robot-tutorial/
 */
int delta_calcAngleYZ(double x0, double y0, double z0, double *theta) {
     double y1 = -0.5 * 0.57735 * f; // f/2 * tg 30
     y0 -= 0.5 * 0.57735    * e;    // shift center to edge
     // z = a + b*y
     double a = (x0*x0 + y0*y0 + z0*z0 +rf*rf - re*re - y1*y1)/(2*z0);
     double b = (y1-y0)/z0;
     // discriminant
     double d = -(a+b*y1)*(a+b*y1)+rf*(b*b*rf+rf); 
     if (d < 0) return -1; // non-existing point
     double yj = (y1 - a*b - sqrt(d))/(b*b + 1); // choosing outer point
     double zj = a + b*yj;
     *theta = 180.0*atan(-zj/(y1 - yj))/pi/* + ((yj>y1)?180.0:0.0)*/ ;
     return 0;
 }

/*
 * compute 3 axis angle based on x,y,z single coordinate
 * from https://hypertriangle.com/~alex/delta-robot-tutorial/
 */
 int delta_calcInverse(double x0, double y0, double z0, double *theta1, double *theta2, double *theta3) {
     *theta1 = *theta2 = *theta3 = 0;
     int status = delta_calcAngleYZ(x0, y0, z0, theta1);
     if (status == 0) status = delta_calcAngleYZ(x0*cos120 + y0*sin120, y0*cos120-x0*sin120, z0, theta2);  // rotate coords to +120 deg
     if (status == 0) status = delta_calcAngleYZ(x0*cos120 - y0*sin120, y0*cos120+x0*sin120, z0, theta3);  // rotate coords to -120 deg
     return status;
 }
