#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import PoseStamped
from rclpy.qos import QoSProfile, ReliabilityPolicy, DurabilityPolicy
from dronekit import connect, VehicleMode
import time

class DroneController(Node):
    def __init__(self):
        super().__init__('drone_controller')
        
        # --- DDS-подписка (телеметрия) ---
        qos_profile = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            durability=DurabilityPolicy.VOLATILE,
            depth=10
        )
        
        self.subscription = self.create_subscription(
            PoseStamped,
            '/ap/pose/filtered',
            self.pose_callback,
            qos_profile
        )
        
        # --- Подключение через DroneKit (управление) ---
        self.get_logger().info('Подключение к MAVProxy через DroneKit...')
        self.vehicle = connect('127.0.0.1:14550', wait_ready=True)
        self.get_logger().info('Подключено к MAVProxy через DroneKit!')
        
        # Флаг для однократного взлёта
        self.takeoff_sent = False

    def pose_callback(self, msg):
        # Выводим телеметрию из DDS
        x = msg.pose.position.x
        y = msg.pose.position.y
        z = msg.pose.position.z
        self.get_logger().info(f'Позиция (DDS): x={x:.2f}, y={y:.2f}, z={z:.2f}')
        
        # Отправляем команду взлёта один раз
        if not self.takeoff_sent:
            self.takeoff_sent = True
            self.get_logger().info('Отправка команды взлёта через DroneKit...')
            self.takeoff(5.0)

    def takeoff(self, altitude):
        """Взлёт через DroneKit."""
        try:
            self.vehicle.mode = VehicleMode("GUIDED")
            self.vehicle.armed = True
            self.vehicle.simple_takeoff(altitude)
            self.get_logger().info(f'Команда взлёта на {altitude} м отправлена через DroneKit.')
        except Exception as e:
            self.get_logger().error(f'Ошибка взлёта: {e}')

def main(args=None):
    rclpy.init(args=args)
    node = DroneController()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()