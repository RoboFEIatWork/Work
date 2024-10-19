//BIBLIOTECAS
#include <stdio.h>
#include "BluetoothSerial.h"
#include <ESP32Encoder.h>

#if !defined(CONFIG_BT_ENABLED) || !defined(CONFIG_BLUEDROID_ENABLED)
#error Bluetooth is not enabled! Please run `make menuconfig` to and enable it
#endif

BluetoothSerial SerialBT;

//50 RPM max

// Define os pinos utilizados para a leitura dos sinais do encoder
//motor superior direito
#define Pino_FD_A 23
#define Pino_FD_B 22
#define Pino_FD_PWM 21
//motor superior esquerdo
#define Pino_FE_A 33
#define Pino_FE_B 25
#define Pino_FE_PWM 32
//motor inferior direito
#define Pino_TD_A 18
#define Pino_TD_B 19
#define Pino_TD_PWM 2
//motor inferior esquerdo
#define Pino_TE_A 26
#define Pino_TE_B 27
#define Pino_TE_PWM 13

//definição dos encoders
ESP32Encoder encoder_FD;//frontal direito
ESP32Encoder encoder_FE;//frontal esquerdo
ESP32Encoder encoder_TD;//tras direito
ESP32Encoder encoder_TE;//tras esquerdo

//definição de variáveis

//[fe, fd, te, td]
String mensagem;
int pwm[4]={128,128,128,128};
float encoder_pos_ini[4]={0,0,0,0};
unsigned long msegundos_ini = 0;
double rpm[4]={0,0,0,0};
double rpm_desejado[4]={0,0,0,0};
bool contando=0;
bool ler=0;
int tempo_contagem=100;


float get_rpm(){
  rpm[0] = ((encoder_FE.getCount()-encoder_pos_ini[0])*60000)/(tempo_contagem*50000);
  rpm[1] = ((encoder_FD.getCount()-encoder_pos_ini[1])*60000)/(tempo_contagem*50000);
  rpm[2] = ((encoder_TE.getCount()-encoder_pos_ini[2])*60000)/(tempo_contagem*50000);
  rpm[3] = ((encoder_TD.getCount()-encoder_pos_ini[3])*60000)/(tempo_contagem*50000);
}

void message_handler(){
  mensagem = SerialBT.readString(); // a string mensagem armazena o que foi enviado pelo HM-10 para o arduino
  if(mensagem[0]=='f'){
    rpm_desejado[0]=((double)mensagem[1]-48)*10+((double)mensagem[2]-48);
    rpm_desejado[1]=((double)mensagem[1]-48)*10+((double)mensagem[2]-48);
    rpm_desejado[2]=((double)mensagem[1]-48)*10+((double)mensagem[2]-48);
    rpm_desejado[3]=((double)mensagem[1]-48)*10+((double)mensagem[2]-48);
    if(contando==0){
      ler=1;
    }
  }
  if(mensagem[0]=='t'){
    rpm_desejado[0]=-1*(((double)mensagem[1]-48)*10+((double)mensagem[2]-48));
    rpm_desejado[1]=-1*(((double)mensagem[1]-48)*10+((double)mensagem[2]-48));
    rpm_desejado[2]=-1*(((double)mensagem[1]-48)*10+((double)mensagem[2]-48));
    rpm_desejado[3]=-1*(((double)mensagem[1]-48)*10+((double)mensagem[2]-48));
    if(contando==0){
      ler=1;
    }
  }
  if(mensagem[0]=='e'){
    rpm_desejado[0]=(((double)mensagem[1]-48)*10+((double)mensagem[2]-48));
    rpm_desejado[1]=-1*(((double)mensagem[1]-48)*10+((double)mensagem[2]-48));
    rpm_desejado[2]=-1*(((double)mensagem[1]-48)*10+((double)mensagem[2]-48));
    rpm_desejado[3]=(((double)mensagem[1]-48)*10+((double)mensagem[2]-48));
    if(contando==0){
      ler=1;
    }
  }
  if(mensagem[0]=='d'){
    rpm_desejado[0]=-1*(((double)mensagem[1]-48)*10+((double)mensagem[2]-48));
    rpm_desejado[1]=(((double)mensagem[1]-48)*10+((double)mensagem[2]-48));
    rpm_desejado[2]=(((double)mensagem[1]-48)*10+((double)mensagem[2]-48));
    rpm_desejado[3]=-1*(((double)mensagem[1]-48)*10+((double)mensagem[2]-48));
    if(contando==0){
      ler=1;
    }
  }

  if(mensagem[0]=='p'){
    rpm_desejado[0]=0;
    pwm[0]=128;
    rpm_desejado[1]=0;
    pwm[1]=128;
    rpm_desejado[2]=0;
    pwm[2]=128;
    rpm_desejado[3]=0;
    pwm[3]=128;
    ledcWrite(0, pwm[0]);
    ledcWrite(1, pwm[1]);
    ledcWrite(2, pwm[2]);
    ledcWrite(3, pwm[3]);
  }
}

void setup() {
  Serial.begin(9600);
  SerialBT.begin(9600); //Bluetooth device name

  pinMode(Pino_FE_A, INPUT_PULLUP);
  pinMode(Pino_FE_B, INPUT_PULLUP);
  encoder_FE.attachHalfQuad(Pino_FE_A, Pino_FE_B);
  pinMode(Pino_FE_PWM, OUTPUT);
  ledcSetup(0, 500, 8);//Atribuimos ao canal 0 a frequencia de 1000Hz com resolucao de 10bits.
  ledcAttachPin(Pino_FE_PWM, 0);//Atribuimos o pino PWM ao canal 0.
  ledcWrite(Pino_FE_PWM, 0);

  pinMode(Pino_FD_A, INPUT_PULLUP);
  pinMode(Pino_FD_B, INPUT_PULLUP);
  encoder_FD.attachHalfQuad(Pino_FD_A, Pino_FD_B);
  pinMode(Pino_FD_PWM, OUTPUT);
  ledcSetup(1, 500, 8);//Atribuimos ao canal 1 a frequencia de 1000Hz com resolucao de 10bits.
  ledcAttachPin(Pino_FD_PWM, 1);//Atribuimos o pino PWM ao canal 1.
  ledcWrite(Pino_FD_PWM, 0);

  pinMode(Pino_TE_A, INPUT_PULLUP);
  pinMode(Pino_TE_B, INPUT_PULLUP);
  encoder_TE.attachHalfQuad(Pino_TE_A, Pino_TE_B);
  pinMode(Pino_TE_PWM, OUTPUT);
  ledcSetup(2, 500, 8);//Atribuimos ao canal 0 a frequencia de 1000Hz com resolucao de 10bits.
  ledcAttachPin(Pino_TE_PWM, 2);//Atribuimos o pino PWM ao canal 0.
  ledcWrite(Pino_TE_PWM, 0);

  pinMode(Pino_TD_A, INPUT_PULLUP);
  pinMode(Pino_TD_B, INPUT_PULLUP);
  encoder_TD.attachHalfQuad(Pino_TD_A, Pino_TD_B);
  pinMode(Pino_TD_PWM, OUTPUT);
  ledcSetup(3, 500, 8);//Atribuimos ao canal 0 a frequencia de 1000Hz com resolucao de 10bits.
  ledcAttachPin(Pino_FE_PWM, 3);//Atribuimos o pino PWM ao canal 0.
  ledcWrite(Pino_TD_PWM, 0);
}

// A função de loop é executada repetidamente para sempre
void loop() {
  if (SerialBT.available()){
    message_handler();
  }
  if(contando==1 && millis()-msegundos_ini > tempo_contagem){
    get_rpm();
    SerialBT.println("RPM:");
    SerialBT.print(rpm[0]); SerialBT.print(" | "); SerialBT.print(rpm[1]); SerialBT.print(" | "); SerialBT.print(rpm[2]); SerialBT.print(" | "); SerialBT.print(rpm[3]);
    SerialBT.println("");
    contando=0;
  }

  if (!(rpm_desejado+2<rpm && rpm<rpm_desejado-2) && contando==0){
    SerialBT.println("Arrumando");
    SerialBT.print("RPM Desejado: ");
    SerialBT.print(rpm_desejado[0]); SerialBT.print(" | "); SerialBT.print(rpm_desejado[1]); SerialBT.print(" | "); SerialBT.print(rpm_desejado[2]); SerialBT.print(" | "); SerialBT.print(rpm_desejado[3]);
    SerialBT.println("");
    
    pwm[0]=pwm[0]-((rpm_desejado[0]-rpm[0])/abs(rpm_desejado[0]-rpm[0]));
    pwm[1]=pwm[1]-((rpm_desejado[1]-rpm[1])/abs(rpm_desejado[1]-rpm[1]));
    pwm[2]=pwm[2]-((rpm_desejado[2]-rpm[2])/abs(rpm_desejado[2]-rpm[2]));
    pwm[3]=pwm[3]-((rpm_desejado[3]-rpm[3])/abs(rpm_desejado[3]-rpm[3]));
    ler=1;
  }
  //if (rpm_desejado<rpm && contando==0 && sentido_movimento==0){SerialBT.println("Arrumando");SerialBT.print("RPM Desejado: ");SerialBT.println(rpm_desejado);SerialBT.println("");pwm=pwm+1;ler=1;}
  if(ler==1 && contando==0){
    contando=1;
    encoder_pos_ini[0]=encoder_FE.getCount();
    encoder_pos_ini[1]=encoder_FD.getCount();
    encoder_pos_ini[2]=encoder_TE.getCount();
    encoder_pos_ini[3]=encoder_TD.getCount();
    msegundos_ini=millis();    
    ler=0; 
  }
    ledcWrite(0, pwm[0]);
    ledcWrite(1, pwm[1]);
    ledcWrite(2, pwm[2]);
    ledcWrite(3, pwm[3]);
}
