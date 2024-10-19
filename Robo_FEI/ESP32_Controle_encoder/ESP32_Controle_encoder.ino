//BIBLIOTECAS
#include <stdio.h>
#include "BluetoothSerial.h"
#include <ESP32Encoder.h>

#if !defined(CONFIG_BT_ENABLED) || !defined(CONFIG_BLUEDROID_ENABLED)
#error Bluetooth is not enabled! Please run `make menuconfig` to and enable it
#endif

BluetoothSerial SerialBT;

//60 RPM max

// Define os pinos utilizados para a leitura dos sinais do encoder
int motor_1 = 13;

#define Pino_A 4//2
#define Pino_B 5//15


ESP32Encoder encoder;

//definição de variáveis

String mensagem;
int pwm=128;
float w=0;
unsigned long ms0 = 0;
double rpm = 0;
double rpm_desejado=0;
bool contando=0;
bool ler=0;
int tempo_contagem=250;
bool sentido_movimento = 0; //0-trás. 1-Frente

float get_rpm(){
  //return ((myEncoder.read()-w)*60000)/(tempo_contagem*100000);
  return ((encoder.getCount()-w)*60000)/(tempo_contagem*50000);
}

void message_handler(){
  mensagem = SerialBT.readString(); // a string mensagem armazena o que foi enviado pelo HM-10 para o arduino
  rpm_desejado=((double)mensagem[1]-48)*10+((double)mensagem[2]-48);
  if(mensagem[0]=='f'){
    sentido_movimento=1;
    if(contando==0){
      ler=1;
    }
  }
  if(mensagem[0]=='t'){
    rpm_desejado=rpm_desejado*(-1);
    sentido_movimento=0; 
  }
  if(mensagem[0]=='p'){
    rpm_desejado=0;
    pwm=128;
    ledcWrite(0, pwm);    
  }
}

void setup() {
  // Inicializa o pino D2 como saída
  Serial.begin(9600);
  pinMode(Pino_A, INPUT_PULLUP);
  pinMode(Pino_B, INPUT_PULLUP);
  encoder.attachHalfQuad(Pino_A, Pino_B);
  pinMode(motor_1, OUTPUT);
  SerialBT.begin(9600); //Bluetooth device name
  ledcSetup(0, 500, 8);//Atribuimos ao canal 0 a frequencia de 1000Hz com resolucao de 10bits.
  ledcAttachPin(motor_1, 0);//Atribuimos o pino 2 ao canal 0.
  ledcWrite(0, pwm);
}

// A função de loop é executada repetidamente para sempre
void loop() {
  if (SerialBT.available()){
    Serial.println("PWM: ");
    Serial.println(pwm);
    Serial.println("RPM: ");
    Serial.println(rpm);
    Serial.println("RPM Desejado: ");
    Serial.println(rpm_desejado);
    Serial.println("Contando=");
    Serial.println(contando);
    Serial.println("Ler=");
    Serial.println(ler);
    message_handler();
  }
  if(contando==1 && millis()-ms0 > tempo_contagem){
    rpm = get_rpm();
    SerialBT.print("RPM: ");
    SerialBT.println(rpm);
    SerialBT.println("");
    contando=0;
  }

  if (rpm_desejado>rpm && contando==0 && sentido_movimento==1){
    SerialBT.println("Arrumando");
    SerialBT.print("RPM Desejado: ");
    SerialBT.println(rpm_desejado);
    SerialBT.print("pwm: ");
    SerialBT.println(pwm);
    SerialBT.println("");
    
    pwm=pwm-1;
    ler=1;
  }
  if (rpm_desejado<rpm && contando==0 && sentido_movimento==0){
    SerialBT.println("Arrumando");
    SerialBT.print("RPM Desejado: ");
    SerialBT.println(rpm_desejado);
    SerialBT.print("pwm: ");
    SerialBT.println(pwm);
    SerialBT.println("");
    
    pwm=pwm+1;
    ler=1;
  }
  if(ler==1 && contando==0){
    contando=1;
    w=encoder.getCount();
    ms0=millis();    
    ler=0; 
  }
  ledcWrite(0, pwm);
}
