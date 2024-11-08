//bibliotrcsa
#include <stdio.h>
#include <ESP32Encoder.h>
#include <ArduinoJson.h>

#define BUFFER_SIZE_RECEIVER 300
#define END_OF_JSON_CHAR '}'

char buffer[BUFFER_SIZE_RECEIVER]; // Buffer para armazenar a mensagem
int buffer_index = 0; // Indicador de posição no buffer

#define ENCODER_PERIOD 100 //amostragem de tempo

//Dados do robô
#define PULSES_PER_ROTATION 1024
#define MOTOR_REDUCTION 28
#define WHEEL_RADIUS 0.05  //metros
#define lx 0.2355           //metros tem que medir -----------------------------------
#define ly 0.15             //metros tem que medir -----------------------------------


//Definição dos pinos do Motor Superior Esquerdo
#define Pino_FL_A 33
#define Pino_FL_B 25
#define Pino_FL_PWM 32

//Definição dos pinos do Motor Superior Direito
#define Pino_FR_A 23
#define Pino_FR_B 22
#define Pino_FR_PWM 21

//Definição dos pinos do Motor Inferior Direito
#define Pino_RR_A 18
#define Pino_RR_B 19
#define Pino_RR_PWM 2

//Definição dos pinos do Motor Inferior Esquerdo
#define Pino_RL_A 26
#define Pino_RL_B 27
#define Pino_RL_PWM 13

// Definaçao dos Encoders
ESP32Encoder encoder_RR;
ESP32Encoder encoder_RL;
ESP32Encoder encoder_FR;
ESP32Encoder encoder_FL;

//Definição dos zeros dos motores, em pwm
int pwmFR = 520;
int pwmFL = 520;
int pwmRR = 520;
int pwmRL = 535;

//globais para os cálculos do PI para cada motor
long prevTRR = 0;
long prevTRL = 0;
long prevTFR = 0;
long prevTFL = 0;
int posPrevRR = 0;
int posPrevRL = 0;
int posPrevFR = 0;
int posPrevFL = 0;

float encoder_time = 0;

//Velocity PI control
float Kp = 1;
float Ki = 0.2;

float erroRR = 0;
float erroRL = 0;
float erroFR = 0;
float erroFL = 0;
float erroIntRR = 0; //erro integral
float erroIntRL = 0; //erro integral
float erroIntFR = 0; //erro integral
float erroIntFL = 0; //erro integral


//variáveis para o filtro motor Rear Right
float rpm_filtradoRR = 0;
float rpm_previoRR = 0;

//variáveis para o filtro motor Rear Left
float rpm_filtradoRL = 0;
float rpm_previoRL = 0;

//variáveis para o filtro motor Front Right
float rpm_filtradoFR = 0;
float rpm_previoFR = 0;

//variáveis para o filtro motor Front Left
float rpm_filtradoFL = 0;
float rpm_previoFL = 0;

//variáveis para o rpm desejado para cada roda
float target_rpm_RR = 0;
float target_rpm_RL = 0;
float target_rpm_FR = 0;
float target_rpm_FL = 0;


float rotationsRR = 0.0;
float rotationsRL = 0.0;
float rotationsFR = 0.0;
float rotationsFL = 0.0;

float rpmFL = 0.0;

//Formato da mensagem que a ESP vai receber pela serial, correspondendo ao cmd_vel_caramelo
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
void serialEvent() {
  while (Serial.available()) {
    char inChar = (char)Serial.read();
    buffer[buffer_index] = inChar;
    buffer_index++;
    // Verifica se a mensagem está completa
    if (inChar == END_OF_JSON_CHAR) {
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

  } else {
    if (doc_subscription_cmd_vel.containsKey("linear_x") && doc_subscription_cmd_vel.containsKey("linear_y") && doc_subscription_cmd_vel.containsKey("angular_z")){
      float linear_x = doc_subscription_cmd_vel["linear_x"];
      float linear_y = doc_subscription_cmd_vel["linear_y"];
      float angular_z = doc_subscription_cmd_vel["angular_z"];

      if(linear_x < 20.0 && linear_x > -20.0 && angular_z < 20.0 && angular_z > -20.0) {
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

void clearEncoder() {
  encoder_RR.clearCount();
  encoder_RL.clearCount();
  encoder_FR.clearCount();
  encoder_FL.clearCount();
}


float CalcControlSignal(float rpm, float& rpm_previo, float& rpm_filtrado, float target_rpm,float& erro, float& erroInt, float Kp, float Ki) {
  // Filtro passa-baixa de 25Hz
  rpm_filtrado = 0.854 * rpm_filtrado + 0.0728 * rpm + 0.0728 * rpm_previo;
  rpm_previo = rpm;

  // Cálculo do erro
  erro = target_rpm - rpm;
  erroInt += erro;

  // Calcula o sinal de controle u(s)
  float u_s = Kp * erro + Ki * erroInt;

  return u_s;
}

// Função para calcular a velocidade em RPM
void CalcSpeed() {
  // Pega a posicao atual dos encoders
  int posRR = encoder_RR.getCount();
  int posRL = encoder_RL.getCount();
  int posFR = encoder_FR.getCount();
  int posFL = encoder_FL.getCount();

  // Calcula a diferença de posição entre a leitura atual e a leitura anterior
  int deltaPosRR = posRR - posPrevRR;
  int deltaPosRL = posRL - posPrevRL;
  int deltaPosFR = posFR - posPrevFR;
  int deltaPosFL = posFL - posPrevFL;

  // Converte a diferença de posição para rotações (dividido pelos pulsos por rotação e a redução do motor)
  rotationsRR = (float)deltaPosRR / (PULSES_PER_ROTATION * 2 * MOTOR_REDUCTION);
  rotationsRL = (float)deltaPosRL / (PULSES_PER_ROTATION * 2 * MOTOR_REDUCTION);
  rotationsFR = (float)deltaPosFR / (PULSES_PER_ROTATION * 2 * MOTOR_REDUCTION);
  rotationsFL = (float)deltaPosFL / (PULSES_PER_ROTATION * 2 * MOTOR_REDUCTION);

  // Calcula a velocidade em rotações por minuto (RPM) com base no período de amostragem
  float rpmRR = (rotationsRR / (ENCODER_PERIOD / 1000.0)) * 60; // ENCODER_PERIOD está em ms, então dividimos por 1000 para converter para segundos
  float rpmRL = (rotationsRL / (ENCODER_PERIOD / 1000.0)) * 60; // ENCODER_PERIOD está em ms, então dividimos por 1000 para converter para segundos
  float rpmFR = (rotationsFR / (ENCODER_PERIOD / 1000.0)) * 60; // ENCODER_PERIOD está em ms, então dividimos por 1000 para converter para segundos
  rpmFL = (rotationsFL / (ENCODER_PERIOD / 1000.0)) * 60; // ENCODER_PERIOD está em ms, então dividimos por 1000 para converter para segundos

  // Aplica cinemática inversa para o cálculo das velocidades angulares (rad/s),
  //para cada roda e em seguida aplica uma constante para transformar o valor para rpm.
  target_rpm_FL = (vel.linear.x - vel.linear.y - ((lx + ly) * vel.angular.z)) / WHEEL_RADIUS * 9.5493;
  target_rpm_FR = (vel.linear.x + vel.linear.y + ((lx + ly) * vel.angular.z)) / WHEEL_RADIUS * 9.5493;
  target_rpm_RL = (vel.linear.x + vel.linear.y - ((lx + ly) * vel.angular.z)) / WHEEL_RADIUS * 9.5493;
  target_rpm_RR = (vel.linear.x - vel.linear.y + ((lx + ly) * vel.angular.z)) / WHEEL_RADIUS * 9.5493;

  // Chama a função para calcular o sinal de controle e filtrar o RPM
  float u_s_RR = CalcControlSignal(rpmRR, rpm_previoRR, rpm_filtradoRR, -target_rpm_RR, erroRR, erroIntRR, Kp, Ki);
  float u_s_RL = CalcControlSignal(rpmRL, rpm_previoRL, rpm_filtradoRL, target_rpm_RL, erroRL, erroIntRL, Kp, Ki);
  float u_s_FR = CalcControlSignal(rpmFR, rpm_previoFR, rpm_filtradoFR, -target_rpm_FR, erroFR, erroIntFR, Kp, Ki);
  float u_s_FL = CalcControlSignal(rpmFL, rpm_previoFL, rpm_filtradoFL, target_rpm_FL, erroFL, erroIntFL, Kp, Ki);

  // Atualiza a posição anterior
  posPrevRR = posRR;
  posPrevRL = posRL;
  posPrevFR = posFR;
  posPrevFL = posFL;

  float pwmVal[4] = {Pino_RR_PWM,Pino_RL_PWM,Pino_FR_PWM,Pino_FL_PWM};
  float pwm[4] = {pwmRR,pwmRL,pwmFR,pwmFL};
  float u_s[4] = {u_s_RR,u_s_RL,u_s_FR,u_s_FL};
  float zero_pwm[4] = {512,546,512,512};

  for(int i = 0; i < 4; i++){
    pwm[i] = ((u_s[i] / 144) * 512) + 512;

    if(pwm[i] < 0){
      pwm[i] = 0;
    }
    if(pwm[i] > 1024){
      pwm[i] = 1024;
    }
    ledcWrite(pwmVal[i], pwm[i]);
  }

}


void setup() {
  // put your setup code here, to run once:
  Serial.begin(9600);

  // Configuração dos pinos de entrada (sensores de encoders)
  pinMode(Pino_RR_A, INPUT);
  pinMode(Pino_RR_B, INPUT);

  pinMode(Pino_RL_A, INPUT);
  pinMode(Pino_RL_B, INPUT);

  pinMode(Pino_FR_A, INPUT);
  pinMode(Pino_FR_B, INPUT);

  pinMode(Pino_FL_A, INPUT);
  pinMode(Pino_FL_B, INPUT);

  // Configuração dos pinos de saída (PWM para controle dos motores)
  pinMode(Pino_RR_PWM, OUTPUT);
  pinMode(Pino_RL_PWM, OUTPUT);
  pinMode(Pino_FR_PWM, OUTPUT);
  pinMode(Pino_FL_PWM, OUTPUT);

  // Configura os canais PWM para os motores
  ////ledcSetup(canal_RR_PWM, frequencia_PWM, resolucao_PWM); // Configura o canal RR_PWM
  ledcAttachChannel(Pino_RR_PWM, 500, 10, 0); // Associa o pino PWM ao canal RR

  /////ledcSetup(canal_RL_PWM, frequencia_PWM, resolucao_PWM); // Configura o canal RL_PWM
  ledcAttachChannel(Pino_RL_PWM, 500, 10 ,1); // Associa o pino PWM ao canal RL

  /////ledcSetup(canal_RL_PWM, frequencia_PWM, resolucao_PWM); // Configura o canal RL_PWM
  ledcAttachChannel(Pino_FR_PWM, 500, 10 ,2); // Associa o pino PWM ao canal FR

  /////ledcSetup(canal_RL_PWM, frequencia_PWM, resolucao_PWM); // Configura o canal RL_PWM
  ledcAttachChannel(Pino_FL_PWM, 500, 10 ,3); // Associa o pino PWM ao canal FL

  //
  ledcWrite(Pino_RR_PWM, pwmRR);
  ledcWrite(Pino_RL_PWM, pwmRL);
  ledcWrite(Pino_FR_PWM, pwmFR);
  ledcWrite(Pino_FL_PWM, pwmFL);

  //
  encoder_RR.attachHalfQuad(Pino_RR_A, Pino_RR_B);
  encoder_RL.attachHalfQuad(Pino_RL_A, Pino_RL_B);
  encoder_FR.attachHalfQuad(Pino_FR_A, Pino_FR_B);
  encoder_FL.attachHalfQuad(Pino_FL_A, Pino_FL_B);

  clearEncoder();

  while(!Serial) {
  }

}

void loop() {

  // Processar e Atuzlizar as velociade
  if (Serial.available() > 0) {
    serialEvent();
  }

  //Calcula as velocidades das rodas e faz o PI
  if(millis() - encoder_time >= ENCODER_PERIOD){

    CalcSpeed();

    encoder_time = millis();

    StaticJsonDocument<500> doc; // Cria um documento JSON

      // --- Leitura dos Encoders------------------------------------------------------
    JsonArray array_encoders = doc.createNestedArray("encoders"); // Cria um array JSON
    // Adiciona valores ao array
    // array_encoders.add( posFL );
    // array_encoders.add( posFR );
    // array_encoders.add( posRL );
    // array_encoders.add( posRR );
    array_encoders.add( rotationsFL );
    array_encoders.add( rotationsFR );
    array_encoders.add( rotationsRL );
    array_encoders.add( rotationsRR );
    array_encoders.add( rpmFL );
    //-------------------------------------------------------------------------------

      // Serializa e envia o documento JSON
    serializeJson(doc, Serial);
    Serial.println(); // Adiciona uma nova linha
  }

  //Envia a leitura dos encoders pela serial para o PC, no formato Json, para fazer o SLAM
  //Rotations mostra a diferença do quanto de volta a roda deu desde a última leitura. Ou seja, 0.3 voltas, por exemplo.
  // Serial.print("{\"MotorFL\": ");
  // Serial.print(Pino_FL_PWM, 2);  // Imprime o valor do encoder do motor1 com 2 casas decimais
  // Serial.print(", \"MotorFR\": ");
  // Serial.print(Pino_FR_PWM, 2);  // Imprime o valor do encoder do motor2 com 2 casas decimais
  // Serial.print(", \"MotorRL\": ");
  // Serial.print(Pino_RL_PWM, 2);  // Imprime o valor do encoder do motor3 com 2 casas decimais
  // Serial.print(", \"MotorRR\": ");
  // Serial.print(Pino_RR_PWM, 2);  // Imprime o valor do encoder do motor4 com 2 casas decimais
  // Serial.println("}");       // Finaliza a estrutura JSON e quebra a linha
  // Serial.flush();

}
