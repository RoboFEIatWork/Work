//BIBLIOTECAS
#include <stdio.h>
#include <ESP32Encoder.h>
#include <ArduinoJson.h>

#define BUFFER_SIZE_RECEIVER 300
#define END_OF_JSON_CHAR '}'

char buffer[BUFFER_SIZE_RECEIVER]; // Buffer para armazenar a mensagem
int buffer_index = 0;              // Indicador de posição no buffer

#define ENCODER_PERIOD 15 // para deixar em 10Hz (ros)

unsigned long encoder_timer = 0;

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

struct Vel
{
  struct Linear
  {
    float x;
    float y;
    float z;
  } linear;
  struct Angular
  {
    float x;
    float y;
    float z;
  } angular;
};

// Cria uma variável do tipo Vel
Vel vel;

// Esta função será chamada sempre que dados estiverem disponíveis na porta serial
void serialEvent(){
  while (Serial.available()){
    char inChar = (char)Serial.read();
    buffer[buffer_index] = inChar;
    buffer_index++;
    // Verifica se a mensagem está completa
    if (inChar == END_OF_JSON_CHAR){
      buffer[buffer_index] = '\0'; // Adiciona o caractere de terminação nulo para tornar o buffer uma string válida
      processMessage();
      buffer_index = 0; // Reseta o buffer
    }
  }
}

// Esta função deserializa a mensagem JSON e processa os dados
void processMessage() {
  StaticJsonDocument<BUFFER_SIZE_RECEIVER> doc_subscription_cmd_vel;
  DeserializationError error = deserializeJson(doc_subscription_cmd_vel, buffer);
  if (error) {
  }
  else {
    if (doc_subscription_cmd_vel.containsKey("linear_x") && doc_subscription_cmd_vel.containsKey("angular_z")) {
      float linear_x = doc_subscription_cmd_vel["linear_x"];
      float linear_y = doc_subscription_cmd_vel["linear_y"];
      float angular_z = doc_subscription_cmd_vel["angular_z"];

      if (linear_x < 20.0 && linear_x > -20.0 && linear_y < 20.0 && linear_y > -20.0 && angular_z < 20.0 && angular_z > -20.0) {
        // Define os valores
        vel.linear.x = linear_x;
        vel.linear.y = linear_y;
        vel.linear.z = 0.0;
        vel.angular.x = 0.0;
        vel.angular.y = 0.0;
        vel.angular.z = angular_z;
      }
    }
  }
}

// Funçao para limpar os encoders
void ClearEncoder() {
  encoder_FE.clearCount();
  encoder_FD.clearCount();
  encoder_TE.clearCount();
  encoder_TD.clearCount();
}

//definição de variáveis

int pwm[4]={512,512,512,512};
float encoderAtual[4]={0,0,0,0};//float w=0;
float encoderAnterior[4]={0,0,0,0};//float w=0;
int RPM[4] = {512,512,512,512};
float velocidade[4]={0,0,0,0};
float velocidade_desejado[4]={0,0,0,0};
float WHEEL_RADIUS = 0.05;
float ROBOT_RADIUS = 0.16;
//bool sentido_movimento[4]={0,0,0,0}; //0-trás. 1-Frente


void get_wheelVelocity(){
  
// Cálculo da velocidade em metros por segundo para cada roda
  encoderAtual[0] = encoder_FE.getCount();
  encoderAtual[1] = encoder_FD.getCount();
  encoderAtual[2] = encoder_TE.getCount();
  encoderAtual[3] = encoder_TD.getCount();

  velocidade[0] = 2 * 3.1416 * WHEEL_RADIUS * ((encoderAtual[0] - encoderAnterior[0]) / (1024 * ENCODER_PERIOD / 1000.0)) / 60;
  velocidade[1] = 2 * 3.1416 * WHEEL_RADIUS * ((encoderAtual[1] - encoderAnterior[1]) / (1024 * ENCODER_PERIOD / 1000.0)) / 60;
  velocidade[2] = 2 * 3.1416 * WHEEL_RADIUS * ((encoderAtual[2] - encoderAnterior[2]) / (1024 * ENCODER_PERIOD / 1000.0)) / 60;
  velocidade[3] = 2 * 3.1416 * WHEEL_RADIUS * ((encoderAtual[3] - encoderAnterior[3]) / (1024 * ENCODER_PERIOD / 1000.0)) / 60;

  encoderAnterior[0] = encoderAtual[0];
  encoderAnterior[1] = encoderAtual[1];
  encoderAnterior[2] = encoderAtual[2];
  encoderAnterior[3] = encoderAtual[3];
}

void velocity(){
  // Calculate desired wheel speeds based on linear and angular velocity
  velocidade_desejado[0] = (vel.linear.x - vel.linear.y - vel.angular.z * ROBOT_RADIUS);
  velocidade_desejado[1] = (vel.linear.x + vel.linear.y + vel.angular.z * ROBOT_RADIUS) / WHEEL_RADIUS;
  velocidade_desejado[2] = (vel.linear.x + vel.linear.y - vel.angular.z * ROBOT_RADIUS) / WHEEL_RADIUS;
  velocidade_desejado[3] = (vel.linear.x - vel.linear.y + vel.angular.z * ROBOT_RADIUS) / WHEEL_RADIUS;
}


void setup() {
  Serial.begin(9600);

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

  // ClearEncoder();
  velocidade_desejado[0] = 0.5;
}

// A função de loop é executada repetidamente para sempre
void loop() {
  if(millis() - encoder_timer >=  ENCODER_PERIOD){
    // velocity();

    get_wheelVelocity();

    Serial.print("velocidade Desejado: ");
    Serial.println(velocidade_desejado[0]); 
    Serial.print("velocidade atual: ");
    Serial.println(velocidade[0]);
    Serial.print("PWM: ");
    Serial.println(pwm[0]);
    Serial.print("Encoder FE Count: ");
    Serial.println(encoder_FE.getCount());

    for (int i=0; i<4; i++){
      if (velocidade_desejado[i]>velocidade[i]){
        pwm[i]=pwm[i]+1;
      }
      if(velocidade_desejado[i]<velocidade[i]){
        pwm[i]=pwm[i]-1;
      }   
      if (velocidade_desejado[i]==0){pwm[i]=512;}
    }

    ledcWrite(Pino_FE_PWM, pwm[0]);
    ledcWrite(Pino_FD_PWM, pwm[1]);
    ledcWrite(Pino_TE_PWM, pwm[2]);
    ledcWrite(Pino_TD_PWM, pwm[3]);
    encoder_timer = millis();
  }
}