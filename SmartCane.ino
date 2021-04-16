#include <NewTone.h>
#include <SoftPWM.h>
#include <SoftPWM_timer.h>
#include <avr/power.h>
#include <math.h>
#include "notes.h"
volatile long durationOneWay_1, durationOneWay_2, durationOneWay_3, duration_1, duration_2, duration_3;
//========================================================//
int melody[] = {NOTE_E7, NOTE_E7, 0, NOTE_E7, 0, NOTE_C7, NOTE_E7, 0, NOTE_G7, 0, 0, 0, NOTE_G6, 0, 0, 0};
int noteDurations[] = {12, 12, 12, 12, 12, 12, 12, 12, 12, 12, 12, 12, 12, 12, 12, 12, 12, 12, 12, 12, 12, 12, 12, 12, 12, 12};
//=============================================================//
const int pin_trig_down = 6;                                     // Trig Pin Down sensor
const int pin_echo_down = 3;                                     // Echo Pin Down sensor
const int pin_trig_middle = 4;                                   // Trig Pin middle sensor
const int pin_echo_middle = 2;                                   // Echo Pin Middle sensor
const int pin_trig_up = 5;                                       // Trig Pin Up sensor
const int pin_echo_up = 20;                                      // Echo Pin Up sensor
int dutyCycle = 0;                                               //  for calculating The duty cycle
float alpha = 0.5;                                               // exponential smoothing constant factor
int DistanceCm = 0;
int ScaledFactor = 0;

void setup(){
}

void loop (){
}
