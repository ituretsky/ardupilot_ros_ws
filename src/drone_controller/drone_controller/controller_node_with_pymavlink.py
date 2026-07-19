#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import PoseStamped
from rclpy.qos import QoSProfile, ReliabilityPolicy, DurabilityPolicy
from pymavlink import mavutil
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
        
        # Подключение к MAVProxy через pymavlink
        self.get_logger().info('Подключение к MAVProxy...')
        self.master = mavutil.mavlink_connection('udp:127.0.0.1:14550')
        self.master.wait_heartbeat()
        self.get_logger().info('Подключено к MAVProxy!')

        # Флаг для однократной отправки взлёта
        self.takeoff_sent = False

    def pose_callback(self, msg):
        # Извлекаем координаты
        x = msg.pose.position.x
        y = msg.pose.position.y
        z = msg.pose.position.z
                
        # Выводим в консоль
        # self.get_logger().info(f'Позиция: x={x:.2f}, y={y:.2f}, z={z:.2f}')

        # Отправляем взлёт один раз
        if not self.takeoff_sent:
            self.takeoff_sent = True
            self.get_logger().info('Отправка команды на взлёт...')
            self.takeoff(5.0)

    def takeoff(self, altitude):
        """Отправляет команду взлёта через MAVLink."""
        # 1. Переключение в режим GUIDED
        self.master.mav.command_long_send(
            self.master.target_system,
            self.master.target_component,
            mavutil.mavlink.MAV_CMD_DO_SET_MODE,
            0,
            1,  # 1 = GUIDED
            0, 0, 0, 0, 0, 0
        )
        time.sleep(0.5)
        
        # 2. Арминг
        self.master.mav.command_long_send(
            self.master.target_system,
            self.master.target_component,
            mavutil.mavlink.MAV_CMD_COMPONENT_ARM_DISARM,
            0,
            1,  # 1 = ARM
            0, 0, 0, 0, 0, 0
        )
        time.sleep(0.5)

        # 3. Взлёт
        self.master.mav.command_long_send(
            self.master.target_system,
            self.master.target_component,
            mavutil.mavlink.MAV_CMD_NAV_TAKEOFF,
            0,
            0, 0, 0, 0,
            altitude,
            0, 0
        )
        self.get_logger().info(f'Команда взлёта на {altitude} м отправлена.')



def main(args=None):
    rclpy.init(args=args)
    node = DroneController()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
