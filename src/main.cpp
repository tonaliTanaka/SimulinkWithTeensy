#include <Arduino.h>
#include <model.h>

unsigned long preTime = 0;
unsigned long nowTime = 0;
unsigned long interval = 10000; //Step size setted in Simulink

void setup() {
  // put your setup code here, to run once:
  model_initialize(); // initialize fcn created by simulink
  preTime = micros();
}

void loop() {
  // put your main code here, to run repeatedly:
  nowTime = micros();
  if (nowTime - preTime > interval){
    model_step();  //step fcn created by simulink
    preTime = nowTime;
  }
}