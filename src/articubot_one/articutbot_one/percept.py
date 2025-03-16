#!/usr/bin/env python3


import rclpy
from rclpy.node import Node
from sensor_msgs.msg import LaserScan

class LaserScanListener(Node):
    def __init__(self):
        super().__init__('laser_scan_listener')
        self.subscription = self.create_subscription(
            LaserScan,
            '/scan',
            self.callback,
            10)
        self.subscription  # Evita warning de variável não usada
        
    def callback(self, dados_scan):
        pass
        # self.get_logger().info("Recebendo dados do sensor laser:")
        # self.get_logger().info(f"Ângulo mínimo: {dados_scan.angle_min:.2f}")
        # self.get_logger().info(f"Ângulo máximo: {dados_scan.angle_max:.2f}")
        # self.get_logger().info(f"Número de amostras: {len(dados_scan.ranges)}")
        # self.get_logger().info(f"Primeira leitura de distância: {dados_scan.ranges[0]:.2f} metros")

def main(args=None):
    rclpy.init(args=args)
    node = LaserScanListener()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()