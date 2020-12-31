#include <NewTone.h>
#include <SoftPWM.h>
#include <SoftPWM_timer.h>
#include <avr/power.h>
#include <math.h>
#include "notes.h"
volatile long durationOneWay_1, durationOneWay_2, durationOneWay_3, duration_1, duration_2, duration_3;
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

//====================================================//

volatile long echo_middle_start = 0;                             //   Start counting the echo signal
volatile long echo_middle_end = 0;                               //   end of counting the echo signal
int distance_middle = 0;                                         //   The length of the echo signal

volatile long echo_down_start = 0;                               //   Start counting the echo signal
volatile long echo_down_end = 0;                                 //   end of counting the echo signal
int distance_down = 0;                                           //   The length of the echo signal

volatile long echo_up_start = 0;                                 //   Start counting the echo signal
volatile long echo_up_end = 0;                                   //   end of counting the echo signal
int distance_up = 0;                                             //   The length of the echo signal


//==================================================//

int last_update = 0;
int playing = 0;
int ticks = 0;
int enable_feedback = 0;
int enable_motor = 0;
int enable_sound = 0;
int low_level_time_ms = 0;

//==================================================//

void music_init() {
  for (int thisNote = 0; thisNote < 16; thisNote++) {
    int noteDuration = 1000 / noteDurations[thisNote];
    NewTone(Tone_pin, melody[thisNote], noteDuration);
    int pauseBetweenNotes = noteDuration * 1.30;
    delay(pauseBetweenNotes);
  }
  noNewTone(Tone_pin);                                      // Turn off Piezo pin
}
//============================================================//

void setup() {
  Serial.begin(115200);                                     // For debug only, We set Serial Baud rate at 115200
  music_init();                                          // Play music at the initinal of the system.
  SoftPWMBegin();                                           // Setting Library SoftPWM to init
  SoftPWMSet(7, 0);                                         // Setting Pin 7 as PWM output
  SoftPWMSet(8, 0);                                         // Setting Pin 8 as PWM output
  SoftPWMSetFadeTime(7, 0, 100);
  SoftPWMSetFadeTime(8, 0, 100);
  // cli();
  pinMode(pin_trig_down, OUTPUT);                           //   Set sensor pin trig as output
  pinMode(pin_echo_down, INPUT);                            //   Set sensor echo pin as input
  pinMode(pin_trig_middle, OUTPUT);                         //   Set sensor pin trig as output
  pinMode(pin_echo_middle, INPUT);                          //   Set sensor echo pin as input
  pinMode(pin_trig_up, OUTPUT);                             //   Set sensor pin trig as output
  pinMode(pin_echo_up, INPUT);                              //   Set sensor echo pin as input
  //  pinMode(pin_mot_middle, OUTPUT);                          //   Set vibrations motor_1 pin as output
  //  pinMode(pin_mot_up, OUTPUT);                              //   Set vibration motor_2 pin as output
  //  pinMode(pin_mot_down, OUTPUT);                            //   Set vibration motor_3 pin as output
  pinMode(Tone_pin, OUTPUT);                                //   Set Piezo pin as output
  // digitalWrite(7, 0);                           //   Turning off the vibrator by writing value = 0
  //digitalWrite(8, 0);                               //   Turning off the vibrator by writing value = 0
  //  analogWrite(pin_mot_down, 0);                             //   Turning off the vibrator by writing value = 0
  noNewTone(Tone_pin);                                      //   Turning off the Piezo by writting value = 0
  attachInterrupt(digitalPinToInterrupt(pin_echo_down), echo_interrupt_down, CHANGE);
  attachInterrupt(digitalPinToInterrupt(pin_echo_up), echo_interrupt_up, CHANGE);
  attachInterrupt(digitalPinToInterrupt(pin_echo_middle), echo_interrupt_middle, CHANGE);

}
//========================================================//

void loop() {
  effect();                                               //  calling for effect function
  delay(50);

  digitalWrite(pin_trig_down, LOW ) ;                     // turn off the trigger
  delayMicroseconds(2);
  digitalWrite(pin_trig_down, HIGH );                    // Sending high pulse ( trigger )
  delayMicroseconds(10);                       // wait for 10us (module sends signal only, if trigger had a HIGH signal for at least 10 us)
  digitalWrite(pin_trig_down, LOW );               // module sends signal now

  /******************************************************************/
  digitalWrite(pin_trig_up, LOW )   ;             // turn off the trigger
  delayMicroseconds(2);
  digitalWrite(pin_trig_up, HIGH );             // Sending high pulse ( trigger )
  delayMicroseconds(10);                      // wait for 10us (module sends signal only, if trigger had a HIGH signal for at least 10 us)
  digitalWrite(pin_trig_up, LOW);

  /*********************************************************************/
  digitalWrite(pin_trig_middle, LOW);             // turn off the trigger
  delayMicroseconds(2);
  digitalWrite(pin_trig_middle, HIGH);              // Sending high pulse ( trigger )
  delayMicroseconds(10);                       // wait for 10us (module sends signal only, if trigger had a HIGH signal for at least 10 us)
  digitalWrite(pin_trig_middle, LOW);                 // module sends signal low

  /*****************************************************************/

  Serial.print("down:  ");  Serial.print(distance_down);  Serial.print(" cm\t");       // for printing on terminal
  Serial.print("up: ");  Serial.print(distance_up);  Serial.print(" cm\t");             //
  Serial.print("middle: ");  Serial.print(distance_middle);  Serial.println(" cm\t");     //

}
//=========================================================//

void echo_interrupt_down() {
  switch (digitalRead(pin_echo_down)) {
    case HIGH:
      echo_down_end = 0;
      echo_down_start = micros();
      break;
    case LOW:
      if (echo_down_end == 0) {
        echo_down_end = micros();
        duration_1 = echo_down_end - echo_down_start;
        durationOneWay_1 = duration_1 / 2;                 // divided by two, since duration is a roundtrip signal
        // sound speed in air at a temperature of 20°C => ~343.5 m/s  => 0.03435 cm/us
        //Calculation of the distance in centimeters
        distance_down = durationOneWay_1 * 0.03435; // distance in cm
      }
      break;
  }
}

//===================================================//

void echo_interrupt_up() {
  switch (digitalRead(pin_echo_up)) {
    case HIGH:
      echo_up_end = 0;
      echo_up_start = micros();
      break;
    case LOW:
      if (echo_up_end == 0) {
        echo_up_end = micros();
        duration_2 = echo_up_end - echo_up_start;
        durationOneWay_2 = duration_2 / 2;              // divided by two, since duration is a roundtrip signal
        // sound speed in air  at a temperature of 20°C => ~343.5 m/s
        //Calculation of the distance in centimeters
        distance_up = durationOneWay_2 * 0.03435; // distance in cm
      }
      break;
  }
}
//=====================================================//

void echo_interrupt_middle() {
  switch (digitalRead(pin_echo_middle)) {
    case HIGH:
      echo_middle_end = 0;
      echo_middle_start = micros();
      break;
    case LOW:
      if (echo_middle_end == 0) {
        echo_middle_end = micros();
        duration_3 = echo_middle_end - echo_middle_start;
        durationOneWay_3 = duration_3 / 2;                  // divided by two, since duration is a roundtrip signal
        // sound speed in air  at a temperature of 20°C => ~343.5 m/s  => 0.03435 cm/us
        //Calculation of the distance in centimeters
        distance_middle = durationOneWay_3 * 0.03435;         // distance in cm
      }
      break;
  }
}
//=======================================================//

void effect() {

  if (distance_middle > 50 && distance_middle < 120) {     // gjennomsnitt lengde av et steg = 65cm
    SoftPWMSetPercent(7, 95);
    delay(50);
    SoftPWMSetPercent(7, 95);
    delay(50);
    SoftPWMSetPercent(7, 0);
    delay(100);

    // }
    // else {
    //  SoftPWMSetPercent(7, 0);
    //   delay(100);
    // }

    if (distance_middle > 125 && distance_middle < 185   ) {
      SoftPWMSetPercent(7, 75);
      delay(100);
      SoftPWMSetPercent(7, 75);
      delay(150);
      SoftPWMSetPercent(7, 0);
      delay(100);
    }
    // else {
    //    SoftPWMSetPercent(7, 0);
    //   delay(100);
//    /
  }
  if (distance_middle > 190 && distance_middle < 250)
  {

    SoftPWMSetPercent(7, 35);
    delay(250);
    SoftPWMSetPercent(7, 35);
    delay(250);
    SoftPWMSetPercent(7, 0);
    delay(100);
  }
  else {
    SoftPWMSetPercent(7, 0);
    delay(100);
  }
  if ((abs(distance_up > 20)) && (abs(distance_up < 50))) {
    NewTone(Tone_pin, 1000 );
    delay(100);

  }
  else {
    noNewTone(Tone_pin);
    SoftPWMSetPercent(8, 0);
    delay(100);
  }
  if ((abs(distance_up > 140)) && (abs(distance_up < 210))) {
    NewTone(Tone_pin, 2000);
    delay(250);
    //SoftPWMSetPercent(8, 35);
    // delay(100);
    // analogWrite(Tone_pin, 0);
    //delay(100);
  }
  else {
    noNewTone(Tone_pin);
    delay(100);
  }
  if (distance_down > 50 && distance_down < 100 ) {
    SoftPWMSetPercent(8, 95);
    delay(100);
    SoftPWMSetPercent(8, 95);
    delay(350);
  }
  else {
    //  noNewTone(A4);
    SoftPWMSetPercent(8, 0);
    delay(100);
  }
  if (distance_down > 105 && distance_down < 165) {

    SoftPWMSetPercent(8, 50);
    delay(100);
    SoftPWMSetPercent(8, 10);
    delay(100);
    SoftPWMSetPercent(8, 50);
    delay(100);
  }
  else {
    SoftPWMSetPercent(8, 0);
    delay(100);
  }

}
