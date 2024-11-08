import time
import json
import serial
import random
import math
from datetime import datetime

class Serial_pc():

    # Contrutor do nó
    def __init__(self):

        print('Criando o subscriber para cmd_vel')

        self.ser  = serial.Serial("COM5", baudrate= 9600,
           timeout=2.5,
           parity=serial.PARITY_NONE,
           bytesize=serial.EIGHTBITS,
           stopbits=serial.STOPBITS_ONE
        ) #cria a variável da serial

        # Parâmetros do robô
        self.wheel_radius = 0.05  # Raio da roda em metros
        self.lx = 0.5  # Distância entre rodas no eixo X #Tem que medir ---------------------------------------
        self.ly = 0.4  # Distância entre rodas no eixo Y #Tem que medir ---------------------------------------

        # Estado inicial do robô
        self.x = 0.0
        self.y = 0.0
        self.theta = 0.0  # Orientação do robô (ângulo)

        # Distâncias percorridas pelas rodas
        self.encoder_left_front = 0
        self.encoder_right_front = 0
        self.encoder_left_rear = 0
        self.encoder_right_rear = 0

        # Armazenando o tempo da última atualização
        self.last_time = datetime.now()

    # Aqui o nó é executado no ROS
    def run(self):
        self.envia()

        if self.ser.isOpen():
            try:
                incoming = self.ser.readline().decode("utf-8")
                data = json.loads(incoming)
                print("Recebido: ",data) #Testar para ver o que ele está recebendo
                self.publish_odometry(data)
            except Exception as e:
                print(e)
                pass
        else:
            print('opening error')
        # Executa uma iteração do loop de processamento de mensagens.


    def publish_odometry(self,data):

        print(f'Target RPM calculado pela ESP: FL={data["target_rpm_FL"]}, FR={data["target_rpm_FR"]}, RL={data["target_rpm_RL"]}, RR={data["target_rpm_RR"]}')

    # função de callback que lê a mensagem
    def envia(self):
        data = {}
        data["linear_x"] = 0.0   #m/s movimento no eixo x
        data["linear_y"] = 0.0   #m/s movimento no eixo y
        data["linear_z"] = 0.0   #m/s movimento no eixo z
        data["angular_x"] = 0.0 #rad/s de rotação no eixo x
        data["angular_y"] = 0.0 #rad/s de rotação no eixo y
        data["angular_z"] = 0.0 #rad/s de rotação no eixo z
        data=json.dumps(data)
        if self.ser.isOpen():
            self.ser.write(data.encode('ascii'))
            self.ser.flush()
            print("Enviado com sucesso")
        else:
            print('opening error')



# Função principal
def main(args=None):
    node = Serial_pc()
    while (True):     
        node.run()


if __name__ == '__main__':
    main()