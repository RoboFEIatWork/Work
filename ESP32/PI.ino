//BIBLIOTECAS
#include <stdio.h>
#include <ESP32Encoder.h>
#include <ArduinoJson.h>

//* 50 RPM max
//* direita ir para frente tem que ser rpm +, pwm +
//* esquerda ir para frente tem que ser rpm -, pwm -

#define BUFFER_SIZE_RECEIVER 300
#define END_OF_JSON_CHAR '}'

#define ENCODER_PERIOD 100

char buffer[BUFFER_SIZE_RECEIVER]; // Buffer para armazenar a mensagem
int buffer_index = 0;              // Indicador de posição no buffer

long encoder_timer = 0;

// definição dos encoders
ESP32Encoder encoder_FE; // frontal esquerdo(0)
ESP32Encoder encoder_FD; // frontal direito(1)
ESP32Encoder encoder_TE; // trás esquerdo(2)
ESP32Encoder encoder_TD; // trás direito(3)

// Usado para as velocidades dos motores
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

#define indicador_PI 14
#define Kp 1
#define Ki 0.25

#define WHEEL_DIAMETER 0.1
#define ROBOT_RADIUS 0.16

//definição de variáveis
String mensagem;
int pwm[4]={512,512,512,512};
float encoder_posicao[4] = {0,0,0,0};//float w=0;
float encoder_posicao_anterior[4] = {0,0,0,0};//float w=0;
float desiredSpeeds[4] = {0,0,0,0};
float SpeedErro[4] = {0,0,0,0};
unsigned long msegundos_ini = 0;//unsigned long ms0 = 0;
double rpm[4]={0,0,0,0};
double rpm_desejado[4]={0,0,0,0};
bool contando=0;
int tempo_contagem=20;
int goal_rpm = 0;
//bool sentido_movimento[4]={0,0,0,0}; //0-trás. 1-Frente

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



void get_rpm(){
  get_encoder();
  msegundos_ini = millis()
  while (millis()-msegundos_ini < tempo_contagem) {
  }
  get_encoder();
  rpm[0] = ((encoder_posicao[0]-encoder_posicao_anterior[0])/tempo_contagem)/245760000;
  rpm[1] = ((encoder_posicao[1]-encoder_posicao_anterior[1])/tempo_contagem)/245760000;
  rpm[2] = ((encoder_posicao[2]-encoder_posicao_anterior[2])/tempo_contagem)/245760000;
  rpm[3] = ((encoder_posicao[3]-encoder_posicao_anterior[3])/tempo_contagem)/245760000;
}

void MotorVelocity(){
  // Calculate desired wheel speeds based on linear and angular velocity
  float deriredSpeed[0] = (vel.linear.x - vel.linear.y - vel.angular.z * ROBOT_RADIUS) / WHEEL_RADIUS;
  float deriredSpeed[1] = (vel.linear.x + vel.linear.y + vel.angular.z * ROBOT_RADIUS) / WHEEL_RADIUS;
  float deriredSpeed[2] = (vel.linear.x + vel.linear.y - vel.angular.z * ROBOT_RADIUS) / WHEEL_RADIUS;
  float deriredSpeed[3] = (vel.linear.x - vel.linear.y + vel.angular.z * ROBOT_RADIUS) / WHEEL_RADIUS;

  //Measure the actual speed of the wheels from the encoders
  float FL_WheelSpeed = encoder_FE.getCount() / (ENCODER_PERIOD / 1000.0); // Speed in encoder units per second
  float FR_WheelSpeed = encoder_FD.getCount() / (ENCODER_PERIOD / 1000.0); // Speed in encoder units per second
  float BL_WheelSpeed = encoder_TE.getCount() / (ENCODER_PERIOD / 1000.0); // Speed in encoder units per second
  float BR_WheelSpeed = encoder_TD.getCount() / (ENCODER_PERIOD / 1000.0); // Speed in encoder units per second

  Serial.println("wheel speeds:");
  Serial.println(FL_WheelSpeed);
  Serial.println(FR_WheelSpeed);
  Serial.println(BL_WheelSpeed);
  Serial.println(BR_WheelSpeed);

  // Calculate the error between desired and actual speeds
  float FL_Error = desiredSpeedFL - FL_WheelSpeed;
  float FR_Error = desiredSpeedFR - FR_WheelSpeed;
  float BL_Error = desiredSpeedBL - BL_WheelSpeed;
  float BR_Error = desiredSpeedBR - BR_WheelSpeed;

  // Apply PI control for each motor velocity control
  float FL_PWM = Kp * FL_Error + Ki * errorSumFL / (sampleTime / 1000.0);
  float FR_PWM = Kp * FR_Error + Ki * errorSumFR / (sampleTime / 1000.0);
  float BL_PWM = Kp * BL_Error + Ki * errorSumBL / (sampleTime / 1000.0);
  float BR_PWM = Kp * BR_Error + Ki * errorSumBR / (sampleTime / 1000.0);

  Serial.println("PWM values:");
  Serial.println(FL_PWM);
  Serial.println(FR_PWM);
  Serial.println(BL_PWM);
  Serial.println(BR_PWM);

  // limita os valores do PWM entre 0 a 1023 sendo 512 o valor de parada
  pwm[0] = constrain(FL_PWM + 512, -512, 512);
  pwm[1] = constrain(FR_PWM + 512, -512, 512);
  pwm[2] = constrain(BL_PWM + 512, -512, 512);
  pwm[3] = constrain(BR_PWM + 512, -512, 512);

  Serial.println("PWM values after constrain:");
  Serial.println(pwm[0]);
  Serial.println(pwm[1]);
  Serial.println(pwm[2]);
  Serial.println(pwm[3]);

  //
  ledcWrite(Pino_FE_PWM, pwm[0]);
  ledcWrite(Pino_FD_PWM, pwm[1]);
  ledcWrite(Pino_TE_PWM, pwm[2]);
  ledcWrite(Pino_TD_PWM, pwm[3]);

  // Update PID control variables
  errorSumFL += FL_Error * (sampleTime / 1000.0);
  errorSumFR += FR_Error * (sampleTime / 1000.0);
  errorSumBL += BL_Error * (sampleTime / 1000.0);
  errorSumBR += BR_Error * (sampleTime / 1000.0);
  lastErrorFL = FL_Error;
  lastErrorFR = FR_Error;
  lastErrorBL = BL_Error;
  lastErrorBR = BR_Error;

}

//!-----------------------------------------------NAO MUDAR ACIMA DA LINHA-----------------------------------------------!

void setup() {
  Serial.begin(9600);

  //encoder frontal esquerdo
  pinMode(Pino_FE_A, INPUT_PULLUP);
  pinMode(Pino_FE_B, INPUT_PULLUP);
  encoder_FE.attachHalfQuad(Pino_FE_A, Pino_FE_B);
  //pwm frontal esquerdo
  pinMode(Pino_FE_PWM, OUTPUT);
  ledcAttachChannel(Pino_FE_PWM, 500, 10, 0); // 500 Hz, 10-bit resolution
  ledcWrite(Pino_FE_PWM, pwm[0]);

  //encoder frontal direito
  pinMode(Pino_FD_A, INPUT_PULLUP);
  pinMode(Pino_FD_B, INPUT_PULLUP);
  encoder_FD.attachHalfQuad(Pino_FD_A, Pino_FD_B);
  //pwm frontal direito
  pinMode(Pino_FD_PWM, OUTPUT);
  ledcAttachChannel(Pino_FD_PWM, 500, 10, 1);
  ledcWrite(Pino_FD_PWM, pwm[1]);

  //encoder traseiro esquerdo
  pinMode(Pino_TE_A, INPUT_PULLUP);
  pinMode(Pino_TE_B, INPUT_PULLUP);
  encoder_TE.attachHalfQuad(Pino_TE_A, Pino_TE_B);
  //pwm traseito esquerdo
  pinMode(Pino_TE_PWM, OUTPUT);
  ledcAttachChannel(Pino_TE_PWM, 500, 10, 2);
  ledcWrite(Pino_TE_PWM, pwm[2]);

  //encoder traseiro direito
  pinMode(Pino_TD_A, INPUT_PULLUP);
  pinMode(Pino_TD_B, INPUT_PULLUP);
  encoder_TD.attachHalfQuad(Pino_TD_A, Pino_TD_B);
  //pwm traseiro direito
  pinMode(Pino_TD_PWM, OUTPUT);
  ledcAttachChannel(Pino_TD_PWM, 500, 10, 3);
  ledcWrite(Pino_TD_PWM, pwm[3]);

  clearEncoders();

  // Define os valores
  vel.linear.x = 0.0;
  vel.linear.y = 0.0;
  vel.linear.z = 0.0;
  vel.angular.x = 0.0;
  vel.angular.y = 0.0;
  vel.angular.z = 0.0;
}

// A função de loop é executada repetidamente para sempre
void loop() {

  // Verifica se existem novos dados na porta serial
  if (Serial.available() > 0) {
    serialEvent();
  }

  if(millis() - encoder_timer >=  ENCODER_PERIOD){

    ControleMotores();

    encoder_timer = millis();

    StaticJsonDocument<300> doc; // Cria um documento JSON

    // --- Leitura dos Encoders------------------------------------------------------
    JsonArray array_encoders = doc.createNestedArray("encoders"); // Cria um array JSON
    // Adiciona valores ao array
    array_encoders.add( Encoder1.getCount());
    array_encoders.add( Encoder2.getCount());
    //-------------------------------------------------------------------------------

    // Serializa e envia o documento JSON
    serializeJson(doc, Serial);
    Serial.println(); // Adiciona uma nova linha
  }

}