//bibliotecas
#include <stdio.h>
#include <ESP32Encoder.h>
#include <ArduinoJson.h>


#define ENCODER_PERIOD 100 //amostragem de tempo

//Dados do robô
#define PULSES_PER_ROTATION 1024
#define MOTOR_REDUCTION 28
#define WHEEL_RADIUS 0.05  //metros
#define lx 0.16            //metros tem que medir -----------------------------------
#define ly 0.1             //metros tem que medir -----------------------------------


#define BUFFER_SIZE_RECEIVER 300


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
int pwmRL = 550;

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
float Ki = 0.5;

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

void processMessage() {
  int     size_ = 0;
  String  payload;
  while ( !Serial.available()  ){}
  if ( Serial.available() )
    payload = Serial.readStringUntil( '\n' );

  StaticJsonDocument<BUFFER_SIZE_RECEIVER> doc_subscription_cmd_vel;
  DeserializationError   error = deserializeJson(doc_subscription_cmd_vel, payload);

  if (error) {
    Serial.println(error.c_str());
    return;
  }
  if (doc_subscription_cmd_vel.containsKey("linear_x") && doc_subscription_cmd_vel.containsKey("angular_z")) {
      float linear_x = doc_subscription_cmd_vel["linear_x"];
      float linear_y = doc_subscription_cmd_vel["linear_y"];
      float angular_z = doc_subscription_cmd_vel["angular_z"];

      // Define os valores
      vel.linear.x = linear_x;
      vel.linear.y = linear_y;
      vel.linear.z = 0.0;
      vel.angular.x = 0.0;
      vel.angular.y = 0.0;
      vel.angular.z = angular_z;
  }
}

void clearEncoder() {
  encoder_RR.clearCount();
  encoder_RL.clearCount();
  encoder_FR.clearCount();
  encoder_FL.clearCount();
}

void target_rpm(){
  // Aplica cinemática inversa para o cálculo das velocidades angulares (rad/s),
  //para cada roda e em seguida aplica uma constante para transformar o valor para rpm.
  target_rpm_FL = (vel.linear.x -vel.linear.y -(lx + ly)*vel.angular.z)/WHEEL_RADIUS * 9.5493;
  target_rpm_FR = (vel.linear.x +vel.linear.y +(lx + ly)*vel.angular.z)/WHEEL_RADIUS * 9.5493;
  target_rpm_RL = (vel.linear.x +vel.linear.y -(lx + ly)*vel.angular.z)/WHEEL_RADIUS * 9.5493;
  target_rpm_RR = (vel.linear.x -vel.linear.y +(lx + ly)*vel.angular.z)/WHEEL_RADIUS * 9.5493;
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
  target_rpm();
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
  processMessage();

  //Calcula as velocidades das rodas e faz o PI
  if(millis() - encoder_time >= ENCODER_PERIOD){

    CalcSpeed();

    encoder_time = millis();
  }

  //Envia a leitura dos encoders pela serial para o PC, no formato Json, para fazer o SLAM
  //Rotations mostra a diferença do quanto de volta a roda deu desde a última leitura. Ou seja, 0.3 voltas, por exemplo.
  Serial.print("{\"target_rpm_FL\": ");
  Serial.print(target_rpm_FL, 2);  // Imprime o valor do encoder do motor1 com 2 casas decimais
  Serial.print(", \"target_rpm_FR\": ");
  Serial.print(target_rpm_FR, 2);  // Imprime o valor do encoder do motor2 com 2 casas decimais
  Serial.print(", \"target_rpm_RL\": ");
  Serial.print(target_rpm_RL, 2);  // Imprime o valor do encoder do motor3 com 2 casas decimais
  Serial.print(", \"target_rpm_RR\": ");
  Serial.print(target_rpm_RR, 2);  // Imprime o valor do encoder do motor4 com 2 casas decimais
  Serial.println("}");       // Finaliza a estrutura JSON e quebra a linha
  Serial.flush();
}
