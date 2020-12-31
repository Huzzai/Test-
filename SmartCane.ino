#include <NewPing.h>

#define 2 trigpin_1
#define 3 echopin_1


void setup(){
Serial.begin(115200);
pinMode(trigpin_1, OUTPUT);
pinMode(echopin_1, INPUT);
}
void loop(){
}

