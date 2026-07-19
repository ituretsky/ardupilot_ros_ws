#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import PoseStamped
from rclpy.qos import QoSProfile, ReliabilityPolicy, DurabilityPolicy
from mavros_msgs.srv import CommandTOL
import time

class DroneController(Node):
    def __init__(self):
        super().__init__('drone_controller')
        
        # Профиль QoS, совместимый с ArduPilot DDS
        qos_profile = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            durability=DurabilityPolicy.VOLATILE,
            depth=10
        )
        
        # Подписка на топик с позицией дрона
        self.subscription = self.create_subscription(
            PoseStamped,
            '/ap/pose/filtered',
            self.pose_callback,
            qos_profile
        )
        
        # Клиент для сервиса взлёта (MAVROS)
        self.takeoff_client = self.create_client(CommandTOL, '/mavros/cmd/takeoff')
        while not self.takeoff_client.wait_for_service(timeout_sec=1.0):
            self.get_logger().info('Ожидание сервиса /mavros/cmd/takeoff...')

        self.get_logger().info('Дрон-контроллер запущен. Ожидание данных...')

        # Флаг, чтобы взлёт выполнился только один раз
        self.takeoff_sent = False

    def pose_callback(self, msg):
        # Извлекаем координаты
        x = msg.pose.position.x
        y = msg.pose.position.y
        z = msg.pose.position.z
                
        # Выводим в консоль
        self.get_logger().info(
            f'Позиция: x={x:.2f}, y={y:.2f}, z={z:.2f}'
        )

        # Взлетаем, когда получили координаты и ещё не взлетали
        if not self.takeoff_sent:
            self.takeoff_sent = True
            self.get_logger().info('Отправка команды на взлёт...')
            takeoff_req = CommandTOL.Request()
            takeoff_req.altitude = 5.0  # высота в метрах
            self.takeoff_client.call_async(takeoff_req)

def main(args=None):
    rclpy.init(args=args)
    node = DroneController()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
