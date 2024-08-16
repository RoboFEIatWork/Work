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
unsigned long msegundos_ini = 0;//unsigned long ms0 = 0;
double rpm[4]={0,0,0,0};
double rpm_desejado[4]={0,0,0,0};
bool contando=0;
bool ler=0;
int tempo_contagem=15;
//bool sentido_movimento[4]={0,0,0,0}; //0-trás. 1-Frente


float get_rpm(){
  rpm[0] = ((encoder_FE.getCount()-encoder_posicao[0])*60000)/(tempo_contagem*50000);
  rpm[1] = ((encoder_FD.getCount()-encoder_posicao[1])*60000)/(tempo_contagem*50000);
  rpm[2] = ((encoder_TE.getCount()-encoder_posicao[2])*60000)/(tempo_contagem*50000);
  rpm[3] = ((encoder_TD.getCount()-encoder_posicao[3])*60000)/(tempo_contagem*50000);
  return rpm[0],rpm[1],rpm[2],rpm[3];
}

void message_handler(){
  mensagem = SerialBT.readString(); // a string mensagem armazena o que foi enviado pelo bluetooth para o arduino
  //mover para frente
  if(mensagem[0]=='f'){
    rpm_desejado[0]=-1*(((double)mensagem[1]-48)*10+((double)mensagem[2]-48));
    rpm_desejado[1]=-1*rpm_desejado[0];
    rpm_desejado[2]=rpm_desejado[0];
    rpm_desejado[3]=-1*rpm_desejado[0];
    //sentido_movimento[0]=0;
    //sentido_movimento[2]=0;
    //sentido_movimento[1]=1;
    //sentido_movimento[3]=1;    
    if(contando==0){
      ler=1;
    }
  }
  //mover para trás
  if(mensagem[0]=='t'){
    rpm_desejado[0]=(((double)mensagem[1]-48)*10+((double)mensagem[2]-48));
    rpm_desejado[1]=-1*rpm_desejado[0];
    rpm_desejado[2]=rpm_desejado[0];
    rpm_desejado[3]=-1*rpm_desejado[0];
    //sentido_movimento[0]=1;
    //sentido_movimento[2]=1;
    //sentido_movimento[1]=0;
    //sentido_movimento[3]=0; 
    if(contando==0){
      ler=1;
    }
  }
  //girar direita
  if(mensagem[0]=='w'){
    rpm_desejado[0]=(((double)mensagem[1]-48)*10+((double)mensagem[2]-48));
    rpm_desejado[2]=0;
    rpm_desejado[1]=rpm_desejado[0];
    rpm_desejado[3]=0;
    //sentido_movimento[0]=1;
    //sentido_movimento[2]=1;
    //sentido_movimento[1]=1;
    //sentido_movimento[3]=1; 
    if(contando==0){
      ler=1;
    }
  }
  //girar esquerda
  if(mensagem[0]=='q'){
    rpm_desejado[0]=-1*(((double)mensagem[1]-48)*10+((double)mensagem[2]-48));
    rpm_desejado[2]=0;
    rpm_desejado[1]=rpm_desejado[0];
    rpm_desejado[3]=0;
    //sentido_movimento[0]=0;
    //sentido_movimento[2]=0;
    //sentido_movimento[1]=0;
    //sentido_movimento[3]=0; 
    if(contando==0){
      ler=1;
    }
  }
  //mover para direita
  if(mensagem[0]=='d'){
    rpm_desejado[0]=(((double)mensagem[1]-48)*10+((double)mensagem[2]-48));
    rpm_desejado[2]=-1*rpm_desejado[0];
    rpm_desejado[1]=rpm_desejado[0];
    rpm_desejado[3]=-1*rpm_desejado[0];
    //sentido_movimento[0]=1;
    //sentido_movimento[2]=0;
    //sentido_movimento[1]=1;
    //sentido_movimento[3]=0; 
    if(contando==0){
      ler=1;
    }
  }
  //mover para esquerda
  if(mensagem[0]=='e'){
    rpm_desejado[0]=-1*(((double)mensagem[1]-48)*10+((double)mensagem[2]-48));
    rpm_desejado[2]=-1*rpm_desejado[0];
    rpm_desejado[1]=rpm_desejado[0];
    rpm_desejado[3]=-1*rpm_desejado[0];
    //sentido_movimento[0]=0;
    //sentido_movimento[2]=1;
    //sentido_movimento[1]=0;
    //sentido_movimento[3]=1; 
    if(contando==0){
      ler=1;
    }
  }
  if(mensagem[0]=='p'){
    rpm_desejado[0]=0;rpm_desejado[1]=0;rpm_desejado[2]=0;rpm_desejado[3]=0;
    pwm[0]=512;pwm[1]=512;pwm[2]=512;pwm[3]=512;
    rpm[0]=0;rpm[1]=0;rpm[2]=0;rpm[3]=0;
    ledcWrite(0, pwm[0]);ledcWrite(1, pwm[1]);ledcWrite(2, pwm[2]);ledcWrite(3, pwm[3]);    
  }
}

void setup() {
  // Inicializa o pino D2 como saída
  Serial.begin(9600);
  SerialBT.begin(9600); //Bluetooth device name
  //encoder
  pinMode(Pino_FE_A, INPUT_PULLUP);
  pinMode(Pino_FE_B, INPUT_PULLUP);
  encoder_FE.attachHalfQuad(Pino_FE_A, Pino_FE_B);
  //pwm
  pinMode(Pino_FE_PWM, OUTPUT);
  //ledcSetup(0, 500, 10);//Atribuimos ao canal 0 a frequencia de 1000Hz com resolucao de 10bits.
  //ledcAttachPin(Pino_FE_PWM, 0);//Atribuimos o pino 2 ao canal 0.
  ledcAttachChannel(Pino_FE_PWM, 500, 10, 0);
  ledcWrite(0, pwm[0]);

  //encoder
  pinMode(Pino_FD_A, INPUT_PULLUP);
  pinMode(Pino_FD_B, INPUT_PULLUP);
  encoder_FD.attachHalfQuad(Pino_FD_A, Pino_FD_B);
  //pwm
  pinMode(Pino_FD_PWM, OUTPUT);
  //ledcSetup(1, 500, 10);//Atribuimos ao canal 0 a frequencia de 1000Hz com resolucao de 10bits.
  //ledcAttachPin(Pino_FD_PWM, 1);//Atribuimos o pino 2 ao canal 0.
  ledcAttachChannel(Pino_FD_PWM, 500, 10, 1);
  ledcWrite(1, pwm[1]);

  //encoder
  pinMode(Pino_TE_A, INPUT_PULLUP);
  pinMode(Pino_TE_B, INPUT_PULLUP);
  encoder_TE.attachHalfQuad(Pino_TE_A, Pino_TE_B);
  //pwm
  pinMode(Pino_TE_PWM, OUTPUT);
  //ledcSetup(2, 500, 10);//Atribuimos ao canal 0 a frequencia de 1000Hz com resolucao de 10bits.
  //ledcAttachPin(Pino_TE_PWM, 2);//Atribuimos o pino 2 ao canal 0.
  ledcAttachChannel(Pino_TE_PWM, 500, 10, 2);
  ledcWrite(2, pwm[2]);

  //encoder
  pinMode(Pino_TD_A, INPUT_PULLUP);
  pinMode(Pino_TD_B, INPUT_PULLUP);
  encoder_TD.attachHalfQuad(Pino_TD_A, Pino_TD_B);
  //pwm
  pinMode(Pino_TD_PWM, OUTPUT);
  //ledcSetup(3, 500, 10);//Atribuimos ao canal 0 a frequencia de 1000Hz com resolucao de 10bits.
  //ledcAttachPin(Pino_TD_PWM, 3);//Atribuimos o pino 2 ao canal 0.
  ledcAttachChannel(Pino_TD_PWM, 500, 10, 3);
  ledcWrite(3, pwm[3]);
}

// A função de loop é executada repetidamente para sempre
void loop() {
 
  if(contando==1 && millis()-msegundos_ini > tempo_contagem){
    rpm[0],rpm[1],rpm[2],rpm[3]=get_rpm();
    SerialBT.print("RPM Desejado: ");
    SerialBT.print(rpm_desejado[0]); SerialBT.print(" | "); SerialBT.print(rpm_desejado[1]); SerialBT.print(" | "); SerialBT.print(rpm_desejado[2]); SerialBT.print(" | "); SerialBT.print(rpm_desejado[3]);
    SerialBT.println("");
    SerialBT.print("RPM: ");
    SerialBT.print(rpm[0]); SerialBT.print(" | "); SerialBT.print(rpm[1]); SerialBT.print(" | "); SerialBT.print(rpm[2]); SerialBT.print(" | "); SerialBT.print(rpm[3]);
    SerialBT.println("");
    contando=0;
  }

  for (int i=0; i<4; i++){
    if (rpm_desejado[i]>rpm[i] and contando==0){
      pwm[i]=pwm[i]+1;
      ler=1;
    }
    if(rpm_desejado[i]<rpm[i] and contando==0){
      pwm[i]=pwm[i]-1;
      ler=1;
    }   
    if (rpm_desejado[i]==0){pwm[i]=512;}
  } 
  if(ler==1 && contando==0){
    contando=1;
    encoder_posicao[0]=encoder_FE.getCount();
    encoder_posicao[1]=encoder_FD.getCount();
    encoder_posicao[2]=encoder_TE.getCount();
    encoder_posicao[3]=encoder_TD.getCount();
    msegundos_ini=millis();    
    ler=0; 
  }
  ledcWrite(0, pwm[0]);
  ledcWrite(1, pwm[1]);
  ledcWrite(2, pwm[2]);
  ledcWrite(3, pwm[3]);
}