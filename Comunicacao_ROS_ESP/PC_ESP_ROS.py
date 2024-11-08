import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, QoSReliabilityPolicy
from geometry_msgs.msg import Pose,Twist  # Importa a mensagem Twist
from nav_msgs.msg import Odometry
import time
import json
import serial
import random
from std_msgs.msg import Header
import math

class Serial_pc(Node):

    # Contrutor do nó
    def __init__(self):

        # Aqui é definido o nome do nó
        super().__init__('Serial_pc')
        qos_profile = QoSProfile(depth=10, reliability = QoSReliabilityPolicy.BEST_EFFORT)

        self.get_logger().info('Criando o subscriber para caramelo_cmd_vel')
        self.subscription = self.create_subscription(Twist,'/caramelo_cmd_vel',self.listener_callback,qos_profile )

        self.get_logger().info('Criando o publisher para caramelo_odom')
        self.publisher = self.create_publisher(Odometry, '/caramelo_odom', 10)

        self.ser  = serial.Serial("/dev/ttyUSB0", baudrate= 9600,
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
        self.last_time = self.get_clock().now()

    # Aqui o nó é executado no ROS
    def run(self):
        if self.ser.isOpen():
            try:
                incoming = self.ser.readline().decode("utf-8")
                data = json.loads(incoming)
                self.get_logger().info(data) #Testar para ver o que ele está recebendo
                self.publish_odometry(data)
            except Exception as e:
                self.get_logger().info(e)
                pass
        # Executa uma iteração do loop de processamento de mensagens.
        rclpy.spin(self)

    def get_encoder_values(self,data):
        # Aqui você deve pegar os valores reais dos encoders. Para exemplo, vamos incrementar um valor fictício.
        self.encoder_left_front += data["MotorFL"]  # Incrementando para simulação
        self.encoder_right_front += data["MotorFR"]
        self.encoder_left_rear += data["MotorRL"]
        self.encoder_right_rear += data["MotorRR"]
        return (self.encoder_left_front, self.encoder_right_front,
                self.encoder_left_rear, self.encoder_right_rear)

    def compute_odometry(self, encoders, delta_time):
        # Calcular a distância percorrida por cada roda
        wheel_distances = []
        for encoder in encoders:
            distance = (encoder) * (2 * math.pi * self.wheel_radius)  # Distância por roda
            wheel_distances.append(distance)

        # Calculando as velocidades lineares e angulares
        vx = sum(wheel_distances) / 4.0 / delta_time
        vy = (-wheel_distances[0] + wheel_distances[1] + wheel_distances[2] - wheel_distances[3]) / 4.0 / delta_time  # Se não houver movimento lateral, você pode manter `vy` como 0
        wz = (-wheel_distances[0] + wheel_distances[1] - wheel_distances[2] + wheel_distances[3]) / (4 * (self.lx + self.ly))
        wz /= delta_time  # Dividir pela diferença de tempo para obter rad/s

        # Atualizando a posição e orientação do robô
        self.x += ((vx * cos(self.theta) - vy * sin(self.theta)) * delta_time)  # Movimento no eixo X
        self.y += ((vx * sin(self.theta) + vy * cos(self.theta)) * delta_time)  # Movimento no eixo Y
        self.theta += (wz * delta_time)  # Mudança de orientação

        return vx, vy, wz

    def publish_odometry(self,data):
        # Obter os valores dos encoders
        encoders = self.get_encoder_values(data)

        # Calcular o tempo decorrido desde a última atualização
        current_time = self.get_clock().now()
        delta_time = (current_time - self.last_time).nanoseconds / 1e9  # Em segundos

        # Calcular a odometria
        vx, vy, wz = self.compute_odometry(encoders, delta_time)

        # Atualizar o timestamp para a próxima iteração
        self.last_time = current_time

        # Criar a mensagem Odometry
        odom_msg = Odometry()

        # Preencher o cabeçalho
        odom_msg.header.stamp = self.get_clock().now().to_msg()
        odom_msg.header.frame_id = 'odom'
        odom_msg.child_frame_id = 'base_link'

        # Preencher a posição (pose)
        odom_msg.pose.pose.position.x = self.x
        odom_msg.pose.pose.position.y = self.y
        odom_msg.pose.pose.position.z = 0.0

        # Orientação (usando um quaternion para representar a rotação)
        odom_msg.pose.pose.orientation.x = 0.0
        odom_msg.pose.pose.orientation.y = 0.0
        odom_msg.pose.pose.orientation.z = math.sin(self.theta / 2)
        odom_msg.pose.pose.orientation.w = math.cos(self.theta / 2)

        # Preencher a velocidade (twist)
        odom_msg.twist.twist.linear.x = vx
        odom_msg.twist.twist.linear.y = vy
        odom_msg.twist.twist.angular.z = wz

        # Publicar a mensagem
        self.publisher_.publish(odom_msg)
        self.get_logger().info(f'Publicando Odometry: x={self.x:.2f}, y={self.y:.2f}, theta={self.theta:.2f}')

    # função de callback que lê a mensagem
    def listener_callback(self, msg):
        data = {}
        data["linear_x"] = msg.linear.x   #m/s movimento no eixo x
        data["linear_y"] = msg.linear.y   #m/s movimento no eixo y
        data["linear_z"] = msg.linear.z   #m/s movimento no eixo z
        data["angular_x"] = msg.angular.x #rad/s de rotação no eixo x
        data["angular_y"] = msg.angular.y #rad/s de rotação no eixo y
        data["angular_z"] = msg.angular.z #rad/s de rotação no eixo z
        data=json.dumps(data)
        if ser.isOpen():
            self.ser.write(data.encode('ascii'))
            self.ser.flush()
            #self.ser.close() #-> Não sei se a gente tem que ficar abrindo ou fechando direto a porta serial
        else:
            self.get_logger().info('opening error')

    # Destrutor do nó
    def __del__(self):
        self.get_logger().info('Finalizando o nó! Tchau, tchau...')


# Função principal
def main(args=None):
    rclpy.init(args=args)
    node = Serial_pc()
    try:
        node.run()
        node.destroy_node()
        rclpy.shutdown()
    except KeyboardInterrupt:
        pass

if __name__ == '__main__':
    main()