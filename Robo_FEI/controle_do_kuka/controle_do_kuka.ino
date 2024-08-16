
#include <SoftwareSerial.h>
SoftwareSerial mySerial(10, 11); // RX, TX  #mySerial é o móudlo HM-10, do bluetooth
int mse = 2; //mse motor superior esquerdo
int msd = 3; ;
// os pares são da esquerda e os ipares da direita
int mid = 4; //mid motor infeior direito
int mie = 5;  // os motroes da direita tem movimentação inversa. Vão para frente com decrécimos. E os da direita são normais, aumentam com o mais
  //PWM
  //0-128 trás   pwm diferente para os motores que tendem para um lado
  //128-255 frente
int pwm[4]={128,128,128,128};//{pwm_mse,pwm_msd,pwm_mie,pwm_mid}

void setup()   
{  
  //Inicia a serial  
  Serial.begin(9600);  //115200
  mySerial.println("Digite os comandos AT :");  
  //Inicia a serial configurada nas portas 10 e 11
  mySerial.begin(9600);  
  pinMode(mse, OUTPUT);
  pinMode(msd, OUTPUT);
  pinMode(mid, OUTPUT);
  pinMode(mie, OUTPUT);
  analogWrite(mse,pwm[0]);
  analogWrite(msd,pwm[1]);
  analogWrite(mie,pwm[2]);
  analogWrite(mid,pwm[3]);   
}  
    
void mov_frente(){
  //for (int n=0;n<4;n++){pwm[n]=pwm[n]+10;}; 
  pwm[0]=pwm[0]+10;pwm[2]=pwm[2]+10;
  pwm[1]=pwm[1]-10;pwm[3]=pwm[3]-10;  
  analogWrite(mse,pwm[0]);
  analogWrite(msd,pwm[1]);
  analogWrite(mie,pwm[2]);
  analogWrite(mid,pwm[3]); 
  delay(500);
}
void mov_tras(){
  //for (int n=0;n<4;n++){pwm[n]=pwm[n]-10;}; 
  pwm[0]=pwm[0]-10;pwm[2]=pwm[2]-10;
  pwm[1]=pwm[1]+10;pwm[3]=pwm[3]+10;  
  analogWrite(mse,pwm[0]);
  analogWrite(msd,pwm[1]);
  analogWrite(mie,pwm[2]);
  analogWrite(mid,pwm[3]);
  delay(500);
}
void mov_parada(){//Força o robô a parar de se mover
  while (1){  
    if(110<=pwm[0] && pwm[0]<=140){
      for (int n=0;n<4;n++){pwm[n]=128;}; 
      analogWrite(mse,pwm[0]);
      analogWrite(msd,pwm[1]);
      analogWrite(mie,pwm[2]);
      analogWrite(mid,pwm[3]);        
      break;}      
    for (int n=0;pwm[n]<128;n++){pwm[n]=pwm[n]+10;}; 
    for (int n=0;pwm[n]>128;n++){pwm[n]=pwm[n]-10;};
    analogWrite(mse,pwm[0]);
    analogWrite(msd,pwm[1]);
    analogWrite(mie,pwm[2]);
    analogWrite(mid,pwm[3]);
    delay(500);
  }  
}
void mov_direita(){//Força o robô a parar de se mover
  pwm[0]=pwm[0]-10;pwm[3]=pwm[3]+10;pwm[1]=pwm[1]-10;pwm[2]=pwm[2]+10;
  analogWrite(mse,pwm[0]);
  analogWrite(msd,pwm[1]);
  analogWrite(mie,pwm[2]);
  analogWrite(mid,pwm[3]);
  delay(500);  
}
void mov_linear_direita(){//Força o robô a parar de se mover
  pwm[0]=pwm[0]-10;pwm[3]=pwm[3]+10;pwm[1]=pwm[1]-10;pwm[2]=pwm[2]-10;
  analogWrite(mse,pwm[0]);
  analogWrite(msd,pwm[1]);
  analogWrite(mie,pwm[2]);
  analogWrite(mid,pwm[3]);
  delay(500);  
}
void mov_linear_esquerda(){//Força o robô a parar de se mover
  pwm[0]=pwm[0]+10;pwm[3]=pwm[3]-10;pwm[1]=pwm[1]+10;pwm[2]=pwm[2]+10;
  analogWrite(mse,pwm[0]);
  analogWrite(msd,pwm[1]);
  analogWrite(mie,pwm[2]);
  analogWrite(mid,pwm[3]);
  delay(500);  
}
void mov_esquerda(){//Força o robô a parar de se mover
  pwm[0]=pwm[0]+10;pwm[3]=pwm[3]-10;pwm[1]=pwm[1]+10;pwm[2]=pwm[2]-10;
  //for (int n=0;n<4;n=n+3){pwm[n]=pwm[n]+10;}; 
  //for (int n=1;n<3;n++){pwm[n]=pwm[n]-10;};
  analogWrite(mse,pwm[0]);
  analogWrite(msd,pwm[1]);
  analogWrite(mie,pwm[2]);
  analogWrite(mid,pwm[3]);
  delay(500);
}
void loop()  
{  
  // Read device output if available.   
  String mensagem;
  if (mySerial.available()) //Verifica se o HM-10 está disponíve para então poder receber os dados.
  {  
    mensagem = mySerial.readString(); // a string mensagem armazena o que foi enviado pelo HM-10 para o arduino
    mySerial.println(mensagem[0]);    
    if(mensagem[0]=='f'){
      //Função que manda o robô para frente.
      Serial.println('o');  //imprime no terminal no arduino
      mySerial.println((char)'O robô vai para frente');  // imprime no monitor serial do HM-10  
      mov_frente();        
    }
    if(mensagem[0]=='t'){
      //Função que manda o robô para trás.
      mySerial.println((char)'O robô vai para trás');  
      mov_tras();       
    }
    if(mensagem[0]=='d'){
      //Função que manda o robô para direita.
      mySerial.println((char)'O robô vai para direita');  
      mov_direita();       
    }
    if(mensagem[0]=='e'){
      //Função que manda o robô para esquerda.
      mySerial.println((char)'O robô vai para esquerda');  
      mov_esquerda();       
    }
    if(mensagem[0]=='p'){
      //Função que manda o robô para trás.
      mySerial.println((char)'O robô vai parar');   
      mov_parada();      
    }
    if(mensagem[0]=='q'){
      //Função que manda o robô para trás.
      mySerial.println((char)'O robô vai parar');   
      mov_linear_direita();      
    }
    if(mensagem[0]=='w'){
      //Função que manda o robô para trás.
      mySerial.println((char)'O robô vai parar');   
      mov_linear_esquerda();      
    }
  }  
  // Lê dados pela usb e envia para o bluetooth.  
  if (Serial.available())
  {  
    delay(10); // Aguarda 10ms 
    mySerial.write(Serial.read());  
  } 
}
