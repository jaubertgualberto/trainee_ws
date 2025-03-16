#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from sensor_msgs.msg import LaserScan
import numpy as np

class ObstacleAvoidanceNode(Node):
    def __init__(self):
        super().__init__('obstacle_avoidance_node')

        # Publisher para movimentação do robô
        self.publisher_ = self.create_publisher(Twist, '/cmd_vel', 10)

        # Subscriber para o sensor LiDAR
        self.subscription = self.create_subscription(
            LaserScan,
            '/scan',
            self.lidar_callback,
            10)

        # Inicializa a velocidade padrão
        self.twist = Twist()
        self.twist.linear.x = 0.5  # Velocidade inicial para frente
        self.twist.angular.z = 0.0  # Sem rotação inicial

        # Timer para publicar a movimentação periodicamente
        self.timer = self.create_timer(0.1, self.publish_movement)

    def lidar_callback(self, msg):
        """Callback do LiDAR, ajusta a velocidade com base na detecção de obstáculos."""

        # Filtra apenas as leituras do centro do campo de visão (±10 graus)
        num_readings = len(msg.ranges)
        center_index = num_readings // 2  # Índice central
        window_size = num_readings // 10  # 10% das leituras ao redor do centro

        front_distances = np.array(msg.ranges[center_index - window_size:center_index + window_size])

        # Removendo valores inválidos (0 ou inf)
        front_distances = front_distances[np.isfinite(front_distances) & (front_distances > 0)]

        if len(front_distances) == 0:
            return  # Evita erro caso não haja leituras válidas

        min_distance = np.min(front_distances)  # Menor distância na frente do robô
        self.get_logger().info(f'Distância mínima frontal: {min_distance:.2f} metros')

        safety_distance = 0.9  # Distância mínima aceitável antes de desviar

        if min_distance < safety_distance:
            self.get_logger().info("Obstáculo detectado! Desviando...")
            self.twist.linear.x = 0.0  # Para o movimento para frente

            # Se obstáculo for mais à esquerda, gire para direita e vice-versa
            if np.mean(front_distances[:len(front_distances)//2]) < np.mean(front_distances[len(front_distances)//2:]):
                self.twist.linear.x = 0.0 
                self.twist.angular.z = 0.8  # Gira mais devagar para evitar exageros
                
            else:
                self.twist.linear.x = 0.0 
                self.twist.angular.z = -0.8  # Gira no sentido oposto
        else:
            self.twist.linear.x = 0.5  # Continua andando para frente
            self.twist.angular.z = 0.0  # Para de girar quando o caminho está livre

    def publish_movement(self):
        """Publica a velocidade calculada."""
        self.publisher_.publish(self.twist)
        self.get_logger().info(f'Publicando comando de movimento: {self.twist}')

def main(args=None):
    rclpy.init(args=args)
    node = ObstacleAvoidanceNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
