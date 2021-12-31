#ifndef STEPPER_H_
#define STEPPER_H_

void enable_axis();

void disable_axis();

void move_pulse(int Motor, long pulses, long dspeed , bool dir);

void move_theta(int Motor, double theta,long dspeed, bool dir);

#endif
