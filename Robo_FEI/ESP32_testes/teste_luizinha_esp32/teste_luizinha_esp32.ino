#include <ESP32Encoder.h>

//BIBLIOTECAS
#include <stdio.h>
#include "BluetoothSerial.h"
#include <ESP32Encoder.h>

#if !defined(CONFIG_BT_ENABLED) || !defined(CONFIG_BLUEDROID_ENABLED)
#error Bluetooth is not enabled! Please run `make menuconfig` to and enable it
#endif

BluetoothSerial SerialBT;

//definição de variáveis

String mensagem;
int ledAzul = 2;
int ledVermelho = 14;


void message_handler(){
  mensagem = SerialBT.readString(); // a string mensagem armazena o que foi enviado pelo bluetooth para o arduino
  //mover para frente
  if(mensagem[0]=='1'){
    digitalWrite(ledVermelho,HIGH);
    }
  else if(mensagem[0]=='0'){
    digitalWrite(ledVermelho,LOW);
    }
  else if(mensagem[0]=='2'){
    digitalWrite(ledAzul,HIGH);
    }
  if(mensagem[0]=='3'){
    digitalWrite(ledAzul, LOW);
    } 
}

void setup() {
  // Inicializa o pino D2 como saída
  Serial.begin(9600);
  SerialBT.begin(9600); //Bluetooth device name
  pinMode(ledVermelho, OUTPUT);
  pinMode(ledAzul, OUTPUT);
}

void loop() {
  if (SerialBT.available()){
    message_handler();
  }
 
}
// mac esp32 78:21:84:7d:bc:4c
