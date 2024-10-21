//BIBLIOTECAS
#include <stdio.h>
#include <ESP32Encoder.h>
#include <ArduinoJson.h>


//globals
long prevT = 0;
int posPrev = 0;
volatile int pos_i = 0;
volatile float velocity_i = 0;
volatile long prevT_i = 0;



// Velocity PI control
float Kp = 1;
float Ki - 0;
float erro = 0;
float erroInt = 0;



void CalcSpeed(){
    pos = .getCount() //
    long currT = micros();
    float deltaT = ((float) (currT - prevT))/1.0e6
    float velocity = (pos - posPrev)/deltaT;
    posPrev = pos;
    prevT = currT; 

    clearEncoder();
}

void SetMotorVelocity(int direction, int pwmVal, int PWM, int MotorIdx){

}

void setup(){
    Serial.begin(9600);

}

void loop(){
    int pwr = 100/3.0*micros()/1.0e6; 

}