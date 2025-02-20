import time
import json
import serial
import random
import math

class Serial_pc():

    # Contrutor do nó
    def __init__(self):

        print('Criando o subscriber para cmd_vel')

        self.ser  = serial.Serial("COM6", baudrate= 9600,
           timeout=2.5,
           parity=serial.PARITY_NONE,
           bytesize=serial.EIGHTBITS,
           stopbits=serial.STOPBITS_ONE
        ) #cria a variável da serial

        # Parâmetros do robô
        self.wheel_radius = 0.05  # Raio da roda em metros
        self.lx = 0.2355  # Distância entre rodas no eixo X #Tem que medir ---------------------------------------
        self.ly = 0.15  # Distância entre rodas no eixo Y #Tem que medir ---------------------------------------

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
        self.last_time = time.perf_counter()

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


    def get_encoder_values(self,data):
        # Aqui você deve pegar os valores reais dos encoders. Para exemplo, vamos incrementar um valor fictício.
        self.encoder_left_front  +=  data["encoders"][0]  # Incrementando para simulação
        self.encoder_right_front -=  data["encoders"][1]
        self.encoder_left_rear   +=  data["encoders"][2]
        self.encoder_right_rear  -=  data["encoders"][3]
        return (self.encoder_left_front, self.encoder_right_front,
                self.encoder_left_rear, self.encoder_right_rear)

    def compute_odometry(self, encoders, delta_time):
        # Calcular a distância percorrida por cada roda
        wheel_distances = []
        for encoder in encoders:
            distance = (encoder) * (2 * math.pi * self.wheel_radius)  # Distância por roda
            wheel_distances.append(distance)

        # # Calculando as velocidades lineares e angulares
        # vx = ( wheel_distances[0] + wheel_distances[1] + wheel_distances[2] + wheel_distances[3]) / ( 4.0 * delta_time)
        # vy = (-wheel_distances[0] + wheel_distances[1] + wheel_distances[2] - wheel_distances[3]) / ( 4.0 * delta_time ) # Se não houver movimento lateral, você pode manter `vy` como 0
        # wz = (-wheel_distances[0] + wheel_distances[1] - wheel_distances[2] + wheel_distances[3]) / (4 * (self.lx + self.ly) * delta_time)
        # # wz /= delta_time  # Dividir pela diferença de tempo para obter rad/s
        
        print(wheel_distances[0])
        print()

        # Error Correction
        if abs(vx) < 0.015: vx = 0
        if abs(vy) < 0.015: vy = 0
        if abs(wz) < 0.015: wz = 0

        # Atualizando a posição e orientação do robô
        self.x += (((vx * math.cos(self.theta)) - ( vy * math.sin(self.theta))) * delta_time)  # Movimento no eixo X
        self.y += (((vx * math.sin(self.theta)) + ( vy * math.cos(self.theta))) * delta_time)  # Movimento no eixo Y
        self.theta += (wz * delta_time)  # Mudança de orientação

        return vx, vy, wz

    def publish_odometry(self,data):

        # Obter os valores dos encoders
        encoders = self.get_encoder_values(data)
        current_time = time.perf_counter()
        # Calcular o tempo decorrido desde a última atualização
        #current_time = time.time()
        delta_time = (current_time - self.last_time)  # Em segundos

        # Calcular a odometria
        vx, vy, wz = self.compute_odometry(encoders, delta_time)

        # Atualizar o timestamp para a próxima iteração
        self.last_time = current_time

        print(f'Publicando Odometry: x={self.x:.2f}, y={self.y:.2f}, theta={wz:.2f}, target_rpm={data["encoders"][4]}')

    # função de callback que lê a mensagem
    def envia(self):
        data = {}
        data["linear_x"] = 0.1  #m/s movimento no eixo x
        data["linear_y"] = 0.0 #m/s movimento no eixo y
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