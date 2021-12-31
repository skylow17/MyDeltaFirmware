#ifndef DELTA_H_
#define DELTA_H_

#include <Arduino.h>
#include <math.h>


#define MICROSTEPPING 16
#define MOT_STEP_PER_REV 200

#define LITTLE_PULLEY_TEETH 16
#define BIG_PULLEY_TEETH 150
#define PULLEY_RATIO double(BIG_PULLEY_TEETH/LITTLE_PULLEY_TEETH)


const double e = 173.;     // end effector
const double f = 190.5;     // base
const double re = 230.5;
const double rf = 90.0; 

 // trigonometric constants
const float sqrt3 = sqrt(3.0);
const float pi = 3.141592653;    // PI
const float sin120 = sqrt3/2.0;   
const float cos120 = -0.5;        
const float tan60 = sqrt3;
const float sin30 = 0.5;
const float tan30 = 1/sqrt3;

//structure to manage machine dynamic values
typedef struct S_DELTA {
  double X;
  double Y;
  double Z;
  double currentTheta1;
  double currentTheta2;
  double currentTheta3;
  double NewTheta1;
  double NewTheta2;
  double NewTheta3;
  double dTheta1;
  double dTheta2;
  double dTheta3;
} S_DELTA;



void init_Delta(void);

int delta_calcAngleYZ(double x0, double y0, double z0, double *theta);

int delta_calcInverse(double x0, double y0, double z0, double *theta1, double *theta2, double *theta3);

 void test(void);

#endif
