//BIBLIOTECAS
#include <stdio.h>
#include "BluetoothSerial.h"
#include <ESP32Encoder.h>

#if !defined(CONFIG_BT_ENABLED) || !defined(CONFIG_BLUEDROID_ENABLED)
#error Bluetooth is not enabled! Please run `make menuconfig` to and enable it
#endif

BluetoothSerial SerialBT;

//50 RPM max
//direita ir para frente tem que ser rpm +, pwm +
//esquerda ir para frente tem que ser rpm -, pwm -

// Define os pinos utilizados para a leitura dos sinais do encoder
//motor superior esquerdo
#define Pino_FE_A 33
#define Pino_FE_B 25
#define Pino_FE_PWM 32
//motor superior direito
#define Pino_FD_A 23
#define Pino_FD_B 22
#define Pino_FD_PWM 21
//motor inferior esquerdo
#define Pino_TE_A 26
#define Pino_TE_B 27
#define Pino_TE_PWM 13
//motor inferior direito
#define Pino_TD_A 18
#define Pino_TD_B 19
#define Pino_TD_PWM 2

//definição dos encoders
ESP32Encoder encoder_FE;//frontal esquerdo(0)
ESP32Encoder encoder_FD;//frontal direito(1)
ESP32Encoder encoder_TE;//trás esquerdo(2)
ESP32Encoder encoder_TD;//trás direito(3)

//definição de variáveis

String mensagem;
int pwm[4]={512,512,512,512};
float encoder_posicao[4]={0,0,0,0};//float w=0;
float encoder_posicao_anterior[4]={0,0,0,0};//float w=0;
unsigned long msegundos_ini = 0;//unsigned long ms0 = 0;
double rpm[4]={0,0,0,0};
double rpm_desejado[4]={0,0,0,0};
int tempo_contagem=15;
int tempo_subida = 15;//ms 5 - 3, 10 - 5.3, 15 - 8.1, 20 - 10.4
unsigned long msegundos_ini_subida = 0;//unsigned long ms0 = 0;
//bool sentido_movimento[4]={0,0,0,0}; //0-trás. 1-Frente

void get_encoder(){
  // encoders vao de 0-1023
  encoder_posicao_anterior[0] = encoder_posicao[0];
  encoder_posicao_anterior[1] = encoder_posicao[1];
  encoder_posicao_anterior[2] = encoder_posicao[2];
  encoder_posicao_anterior[3] = encoder_posicao[3];

  encoder_posicao[0] = encoder_FE.getCount();
  encoder_posicao[1] = encoder_FD.getCount();
  encoder_posicao[2] = encoder_TE.getCount();
  encoder_posicao[3] = encoder_TD.getCount();
}

void get_rpm(){
  grt_encoder();
  rpm[0] = ((encoder_posicao[0]-encoder_posicao_anterior[0])*60000)/(tempo_contagem*50000);
  rpm[1] = ((encoder_posicao[1]-encoder_posicao_anterior[1])*60000)/(tempo_contagem*50000);
  rpm[2] = ((encoder_posicao[2]-encoder_posicao_anterior[2])*60000)/(tempo_contagem*50000);
  rpm[3] = ((encoder_posicao[3]-encoder_posicao_anterior[3])*60000)/(tempo_contagem*50000);
}

void send_info_BT(){
  get_rpm();
  SerialBT.print("RPM_DESEJADO: ");
  SerialBT.print(rpm_desejado[0]); 
  SerialBT.println("");
  SerialBT.print("RPM: ");
  SerialBT.print(rpm[0]);
  SerialBT.println("");
  SerialBT.print("encoder_posicao: ");
  SerialBT.print(encoder_posicao[0]); 
  SerialBT.println("");
  SerialBT.print("pwm: ");
  SerialBT.print(pwm[0]); 
  SerialBT.println("");
}

void message_handler(){
  mensagem = SerialBT.readString(); // a string mensagem armazena o que foi enviado pelo bluetooth para o arduino
  //mover para frente
  if(mensagem[0]=='f'){
    rpm_desejado[0]=-1*(((double)mensagem[1]-48)*10+((double)mensagem[2]-48));
    rpm_desejado[1]=-1*rpm_desejado[0];
    rpm_desejado[2]=rpm_desejado[0];
    rpm_desejado[3]=-1*rpm_desejado[0];

  }
  //mover para trás
  if(mensagem[0]=='t'){
    rpm_desejado[0]=(((double)mensagem[1]-48)*10+((double)mensagem[2]-48));
    rpm_desejado[1]=-1*rpm_desejado[0];
    rpm_desejado[2]=rpm_desejado[0];
    rpm_desejado[3]=-1*rpm_desejado[0];

  }
  //girar direita
  if(mensagem[0]=='w'){
    rpm_desejado[0]=(((double)mensagem[1]-48)*10+((double)mensagem[2]-48));
    rpm_desejado[2]=0;
    rpm_desejado[1]=rpm_desejado[0];
    rpm_desejado[3]=0;

  }
  //girar esquerda
  if(mensagem[0]=='q'){
    rpm_desejado[0]=-1*(((double)mensagem[1]-48)*10+((double)mensagem[2]-48));
    rpm_desejado[2]=0;
    rpm_desejado[1]=rpm_desejado[0];
    rpm_desejado[3]=0;
  }
  //mover para direita
  if(mensagem[0]=='d'){
    rpm_desejado[0]=(((double)mensagem[1]-48)*10+((double)mensagem[2]-48));
    rpm_desejado[2]=-1*rpm_desejado[0];
    rpm_desejado[1]=rpm_desejado[0];
    rpm_desejado[3]=-1*rpm_desejado[0];

  }
  //mover para esquerda
  if(mensagem[0]=='e'){
    rpm_desejado[0]=-1*(((double)mensagem[1]-48)*10+((double)mensagem[2]-48));
    rpm_desejado[2]=-1*rpm_desejado[0];
    rpm_desejado[1]=rpm_desejado[0];
    rpm_desejado[3]=-1*rpm_desejado[0];

  }
  if(mensagem[0]=='p'){
    rpm_desejado[0]=0;rpm_desejado[1]=0;rpm_desejado[2]=0;rpm_desejado[3]=0;
    pwm[0]=512;pwm[1]=512;pwm[2]=512;pwm[3]=512;
    rpm[0]=0;rpm[1]=0;rpm[2]=0;rpm[3]=0;
    ledcWrite(Pino_FE_PWM, pwm[0]);
    ledcWrite(Pino_FD_PWM, pwm[1]);
    ledcWrite(Pino_TE_PWM, pwm[2]);
    ledcWrite(Pino_TD_PWM, pwm[3]); 
  }
}

void setup() {
  Serial.begin(9600);
  SerialBT.begin(9600); //Bluetooth device name

  //encoder
  pinMode(Pino_FE_A, INPUT_PULLUP);
  pinMode(Pino_FE_B, INPUT_PULLUP);
  encoder_FE.attachHalfQuad(Pino_FE_A, Pino_FE_B);
  //pwm
  pinMode(Pino_FE_PWM, OUTPUT);
  ledcAttachChannel(Pino_FE_PWM, 500, 10, 0);
  ledcWrite(Pino_FE_PWM, pwm[0]);

  //encoder
  pinMode(Pino_FD_A, INPUT_PULLUP);
  pinMode(Pino_FD_B, INPUT_PULLUP);
  encoder_FD.attachHalfQuad(Pino_FD_A, Pino_FD_B);
  //pwm
  pinMode(Pino_FD_PWM, OUTPUT);
  ledcAttachChannel(Pino_FD_PWM, 500, 10, 1);
  ledcWrite(Pino_FD_PWM, pwm[1]);

  //encoder
  pinMode(Pino_TE_A, INPUT_PULLUP);
  pinMode(Pino_TE_B, INPUT_PULLUP);
  encoder_TE.attachHalfQuad(Pino_TE_A, Pino_TE_B);
  //pwm
  pinMode(Pino_TE_PWM, OUTPUT);
  ledcAttachChannel(Pino_TE_PWM, 500, 10, 2);
  ledcWrite(Pino_TE_PWM, pwm[2]);

  //encoder
  pinMode(Pino_TD_A, INPUT_PULLUP);
  pinMode(Pino_TD_B, INPUT_PULLUP);
  encoder_TD.attachHalfQuad(Pino_TD_A, Pino_TD_B);
  //pwm
  pinMode(Pino_TD_PWM, OUTPUT);
  ledcAttachChannel(Pino_TD_PWM, 500, 10, 3);
  ledcWrite(Pino_TD_PWM, pwm[3]);
}


// A função de loop é executada repetidamente para sempre*
void loop() {
  if (SerialBT.available()){
    message_handler();
  }
  if(millis()-msegundos_ini > tempo_contagem){
    send_info_BT();
    msegundos_ini=millis();  
  }

  if (millis()-msegundos_ini_subida > tempo_subida){
    for (int i=0; i<4; i++){
      if (rpm_desejado[i]>rpm[i]){
        pwm[i]=pwm[i]+1;
      }
      if(rpm_desejado[i]<rpm[i]){
        pwm[i]=pwm[i]-1;
      }   
      if (rpm_desejado[i]==0){pwm[i]=512;}
    } 
    msegundos_ini_subida = millis();
  }

  ledcWrite(Pino_FE_PWM, pwm[0]);
  ledcWrite(Pino_FD_PWM, pwm[1]);
  ledcWrite(Pino_TE_PWM, pwm[2]);
  ledcWrite(Pino_TD_PWM, pwm[3]);
}