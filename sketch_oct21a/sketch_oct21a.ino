//bibliotrcsa
#include <stdio.h>
#include <ESP32Encoder.h>
#include <ArduinoJson.h>

#define ENCODER_PERIOD 100 //amostragem de tempo

#define PULSES_PER_ROTATION 1024
#define MOTOR_REDUCTION 28

// Motor Superior Esquerdo
#define Pino_FE_A 18 // (RR - 18)   (RL - 26)  (FR - )   (FL - )
#define Pino_FE_B 19 // (RR - 19)   (RL - 27)  (FR - )   (FL - )
#define Pino_FE_PWM 2

// Definaçao dos Encoders
ESP32Encoder encoder_FE;

//globais
long prevT = 0;
int posPrev = 0;

float encoder_time = 0;



//Velocity PI control
float Kp = 1;
float Ki = 0;
float erro = 0;
float erroInt = 0; //erro integral


//variáveis para o filtro
float rpm_filtrado = 0;
float rpm_previo = 0;


void clearEncoder() {
  encoder_FE.clearCount();
}

void setMotor(int dir, int pwmVal,int motor_index){

}

// Função para calcular a velocidade em RPM
void CalcSpeed() { 
    // Obtém a posição atual do encoder (em pulsos)
    int pos = encoder_FE.getCount();
    
    // Calcula a diferença de posição entre a leitura atual e a leitura anterior
    int deltaPos = pos - posPrev;
    
    // Converte a diferença de posição para rotações (dividido pelos pulsos por rotação e a redução do motor)
    float rotations = (float)deltaPos / (PULSES_PER_ROTATION * 2 * MOTOR_REDUCTION);
    
    // Calcula a velocidade em rotações por segundo (RPS) com base no período de amostragem
    float rps = rotations / (ENCODER_PERIOD / 1000.0); // ENCODER_PERIOD está em ms, então dividimos por 1000 para converter para segundos
    
    // Converte RPS para RPM (multiplicando por 60)
    float rpm = rps * 60.0;

    //Filtro_passa_baixa 25HZ #Mudar depois-----------------------------
    rpm_filtrado = 0.854*rpm_filtrado +0.0728*rpm + 0.0728*rpm_previo;
    rpm_previo = rpm;
    
    // Atualiza a posição anterior
    posPrev = pos;  

    //Sinal de controle u(s)
    erro = target_rpm-rpm_filtrado;
    erroInt = erroInt + erro*ENCODER_PERIOD;
    float u_s = kp*erro + ki*erroInt;

    //seta a velocidade e direção do motor
    int dir = 1;
    if(u_s<0){
      dir = -1;
    }
    int pwr = fabs(u_s);
    if(pwr>512){
      pwr=512;
    }
    // TODO: setMotor(dir,pwr)




    // Imprime a posição e a velocidade em RPM
    Serial.println(pos);
    Serial.println(rpm);
    Serial.println(rpm_filtrado);
}

void setup() {
  // put your setup code here, to run once:
  Serial.begin(9600);

  //
  pinMode(Pino_FE_A, INPUT);
  pinMode(Pino_FE_B, INPUT);
  pinMode(Pino_FE_PWM, OUTPUT);
  ledcAttachChannel(Pino_FE_PWM, 500, 10, 0);
  ledcWrite(Pino_FE_PWM, 720);

  encoder_FE.attachHalfQuad(Pino_FE_A, Pino_FE_B);

  clearEncoder();

}

void loop() {
  // put your main code here, to run repeatedly:

  if(millis() - encoder_time >= ENCODER_PERIOD){
    
    CalcSpeed();

    encoder_time = millis();
  }
  

}
