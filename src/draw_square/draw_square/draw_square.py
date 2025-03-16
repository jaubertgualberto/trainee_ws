import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
import time

class DrawSquare(Node):
    def __init__(self):
        super().__init__('draw_square')
        self.publisher_ = self.create_publisher(Twist, '/turtle1/cmd_vel', 10)
        self.state = 0
        self.timer_ = self.create_timer(1.0, self.move_turtle)
        self.i = 0

    def move_turtle(self):
        msg = Twist()

        if self.i < 4:
            if self.state % 2 == 0:
                msg.linear.x = 2.0  # Move para frente
            else:
                msg.angular.z = 1.57  # Gira 90 graus
                self.i += 1


        self.publisher_.publish(msg)
        self.get_logger().info(f'Enviando comando: linear.x={msg.linear.x}, angular.z={msg.angular.z}')
        self.state += 1

def main(args=None):
    rclpy.init(args=args)
    node = DrawSquare()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()