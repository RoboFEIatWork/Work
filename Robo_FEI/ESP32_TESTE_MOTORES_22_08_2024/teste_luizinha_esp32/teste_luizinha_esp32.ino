//BIBLIOTECAS
#include <stdio.h>
#include "BluetoothSerial.h"
#include <ESP32Encoder.h>

#if !defined(CONFIG_BT_ENABLED) || !defined(CONFIG_BLUEDROID_ENABLED)
#error Bluetooth is not enabled! Please run `make menuconfig` to and enable it
#endif

//definindo os pinos do encoder do motor trás direito
#define Pino_TD_A 18
#define Pino_TD_B 19

BluetoothSerial SerialBT;

ESP32Encoder encoder_TD;//Define o encoder do motor trás direito

//definição de variáveis

String mensagem;
//int ledAzul = 2;
float encoder_posicao[4]={0,0,0,0};//float w=0;
double rpm[4]={0,0,0,0};
double rpm_desejado[4]={0,0,0,0};
unsigned long msegundos_ini = 0;//unsigned long ms0 = 0;
bool contando=0;
bool ler=0;
int tempo_contagem=15;

float get_rpm(){
  rpm[3] = ((encoder_TD.getCount()-encoder_posicao[3])*60000)/(tempo_contagem*50000);
  return rpm[0],rpm[1],rpm[2],rpm[3];
}


//void message_handler(){
//  mensagem = SerialBT.readString(); // a string mensagem armazena o que foi enviado pelo bluetooth para o arduino
  //mover para frente
//  if(mensagem[0]=='2'){
//    digitalWrite(ledAzul,HIGH);
//    SerialBT.print("Ligado ");
//    }
//  else if(mensagem[0]=='3'){
//    digitalWrite(ledAzul, LOW);
//    SerialBT.print("Desligado ");
//    } 
//}

void setup() {
  // Inicializa o pino D2 como saída
  Serial.begin(9600);
  SerialBT.begin(9600); //Bluetooth device name
  //pinMode(ledAzul, OUTPUT);
  //encoder
  pinMode(Pino_TD_A, INPUT_PULLUP);
  pinMode(Pino_TD_B, INPUT_PULLUP);
  encoder_TD.attachHalfQuad(Pino_TD_A, Pino_TD_B);
}

void loop() {
  //if (SerialBT.available()){
  //  message_handler();
  //}
 if(millis()-msegundos_ini > tempo_contagem){
    rpm[0],rpm[1],rpm[2],rpm[3]=get_rpm();
    SerialBT.print("RPM Desejado: ");
    SerialBT.print(rpm_desejado[0]); SerialBT.print(" | "); SerialBT.print(rpm_desejado[1]); SerialBT.print(" | "); SerialBT.print(rpm_desejado[2]); SerialBT.print(" | "); SerialBT.print(rpm_desejado[3]);
    SerialBT.println("");
    SerialBT.print("RPM: ");
    SerialBT.print(rpm[0]); SerialBT.print(" | "); SerialBT.print(rpm[1]); SerialBT.print(" | "); SerialBT.print(rpm[2]); SerialBT.print(" | "); SerialBT.print(rpm[3]);
    SerialBT.println("");
    contando=0;
  }
 
}
// mac esp32 78:21:84:7d:bc:4c
