//BIBLIOTECAS
#include <stdio.h>
#include <ESP32Encoder.h>
#include <ArduinoJson.h>

#define BUFFER_SIZE_RECEIVER 300
#define END_OF_JSON_CHAR '}'

#define ENCODER_PERIOD 100 //amostragem de tempo

#define PULSOS_POR_REVOLUCAO 1024

char buffer[BUFFER_SIZE_RECEIVER]; // Buffer para armazenar a mensagem
int buffer_index = 0;              // Indicador de posição no buffer

long encoder_timer = 0;

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

#define Kp 1
#define Ki 0.01

#define WHEEL_RADIUS 0.05
#define ROBOT_RADIUS 0.16

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

//!------------------------------------------------------ NAO MEXER ASIMA DESSA LINHA-------------------------------------------------!//


float rpmDesejado[4] = {0, 0, 0, 0};
float rpmAtual[4] = {0, 0, 0, 0};


void ConvertToRPM() {
    // Velocidade linear de cada roda
    float v_FE = vel.linear.x - vel.linear.y - vel.angular.z * ROBOT_RADIUS;
    float v_FD = vel.linear.x + vel.linear.y + vel.angular.z * ROBOT_RADIUS;
    float v_TE = vel.linear.x + vel.linear.y - vel.angular.z * ROBOT_RADIUS;
    float v_TD = vel.linear.x - vel.linear.y + vel.angular.z * ROBOT_RADIUS;

    // Converter velocidade linear para RPM
    rpmDesejado[0] = (v_FE / (2 * PI * WHEEL_RADIUS)) * 60;
    rpmDesejado[1] = (v_FD / (2 * PI * WHEEL_RADIUS)) * 60;
    rpmDesejado[2] = (v_TE / (2 * PI * WHEEL_RADIUS)) * 60;
    rpmDesejado[3] = (v_TD / (2 * PI * WHEEL_RADIUS)) * 60;
}


void get_rpmAtual(){
    //calcula o RMP atual
    rpmAtual[0] = (encoder_FE.getCount() * 60000) / (PULSOS_POR_REVOLUCAO * ENCODER_PERIOD);
    rpmAtual[1] = (encoder_FD.getCount() * 60000) / (PULSOS_POR_REVOLUCAO * ENCODER_PERIOD);
    rpmAtual[2] = (encoder_TE.getCount() * 60000) / (PULSOS_POR_REVOLUCAO * ENCODER_PERIOD);
    rpmAtual[3] = (encoder_TD.getCount() * 60000) / (PULSOS_POR_REVOLUCAO * ENCODER_PERIOD);

    //limpa os encoders apos leitura.
    ClearEncoder();
}

// Variáveis para armazenar os erros acumulados e os erros instantâneos
float erroAcumulado[4] = {0, 0, 0, 0};
float erroInstantaneo[4] = {0, 0, 0, 0};

// Função de controle PI
void ControlPI() {
    for (int i = 0; i < 4; i++) {
        // Calcular o erro atual
        erroInstantaneo[i] = rpmDesejado[i] - rpmAtual[i];

        // Atualizar o erro acumulado
        erroAcumulado[i] += erroInstantaneo[i];

        // Calcular a saída do controlador PI
        float output = Kp * erroInstantaneo[i] + Ki * erroAcumulado[i];

        // Ajustar a velocidade do motor com base na saída do controlador PI
        setMotorSpeed(i, output);
    }
}

// Função para ajustar a velocidade do motor
void setMotorSpeed(int motorIndex, float speed) {
    // Implementar a lógica para ajustar a velocidade do motor
    // Isso pode variar dependendo do hardware e da biblioteca que você está usando
    // Exemplo:
    // motor[motorIndex].setSpeed(speed);
}







//!-------------------------------------------------------


void setup(){

}

void loop(){
    if(millis() - encoder_timer >=  ENCODER_PERIOD){
        encoder_timer = millis();
        get_rpmAtual();
        ControlPI();
    }

    // Processar e Atuzlizar as velociade
    // processMessage();

    // Converte a velocidade do robo para velociade das rodas.
    ConvertToRPM();

    // Aqui você pode adicionar código para usar os valores de RPM calculados
}
