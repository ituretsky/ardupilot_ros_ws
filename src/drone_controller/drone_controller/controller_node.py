#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import PoseStamped
from rclpy.qos import QoSProfile, ReliabilityPolicy, DurabilityPolicy
from mavros_msgs.srv import CommandTOL, CommandBool, SetMode
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
        
        # --- Клиент для установки режима GUIDED ---
        self.set_mode_client = self.create_client(SetMode, '/mavros/set_mode')
        while not self.set_mode_client.wait_for_service(timeout_sec=1.0):
            self.get_logger().info('Ожидание сервиса /mavros/set_mode...')
        self.get_logger().info('Сервис /mavros/set_mode доступен.')
        
        # --- Клиент для ARM ---
        self.arming_client = self.create_client(CommandBool, '/mavros/cmd/arming')
        while not self.arming_client.wait_for_service(timeout_sec=1.0):
            self.get_logger().info('Ожидание сервиса /mavros/cmd/arming...')
        self.get_logger().info('Сервис /mavros/cmd/arming доступен.')
        
        # --- Клиент для взлёта ---
        self.takeoff_client = self.create_client(CommandTOL, '/mavros/cmd/takeoff')
        while not self.takeoff_client.wait_for_service(timeout_sec=1.0):
            self.get_logger().info('Ожидание сервиса /mavros/cmd/takeoff...')
        self.get_logger().info('Сервис /mavros/cmd/takeoff доступен.')
        
        self.takeoff_sent = False

    def pose_callback(self, msg):
        x = msg.pose.position.x
        y = msg.pose.position.y
        z = msg.pose.position.z
        self.get_logger().info(f'Позиция (DDS): x={x:.2f}, y={y:.2f}, z={z:.2f}')
        
        if not self.takeoff_sent:
            self.takeoff_sent = True
            self.get_logger().info('Отправка команды взлёта через MAVROS...')
            self.arm_and_takeoff(5.0)

    def arm_and_takeoff(self, altitude):
        """Армит дрон и сразу отправляет взлёт."""
        # 1. Переключение в режим GUIDED
        mode_req = SetMode.Request()
        mode_req.custom_mode = 'GUIDED'
        future_mode = self.set_mode_client.call_async(mode_req)
        rclpy.spin_once(self, timeout_sec=0.5)
        if future_mode.done():
            self.get_logger().info('Режим GUIDED установлен.')
        
        # 2. ARM
        arm_req = CommandBool.Request()
        arm_req.value = True
        future_arm = self.arming_client.call_async(arm_req)
        rclpy.spin_once(self, timeout_sec=0.5)
        if future_arm.done():
            result = future_arm.result()
            if result.success:
                self.get_logger().info('Дрон заармлен.')
            else:
                self.get_logger().error('Ошибка ARM!')
                return
        
        # 3. Взлёт сразу после ARM
        takeoff_req = CommandTOL.Request()
        takeoff_req.altitude = altitude
        future_takeoff = self.takeoff_client.call_async(takeoff_req)
        rclpy.spin_once(self, timeout_sec=0.5)
        if future_takeoff.done():
            result = future_takeoff.result()
            if result.success:
                self.get_logger().info('Взлёт успешен!')
            else:
                self.get_logger().error(f'Ошибка взлёта: {result.result}')

def main(args=None):
    rclpy.init(args=args)
    node = DroneController()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()