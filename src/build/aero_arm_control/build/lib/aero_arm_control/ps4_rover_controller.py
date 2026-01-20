#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Joy
from geometry_msgs.msg import Twist
from std_msgs.msg import Float64MultiArray
import math


class PS4RoverController(Node):
    """
    PS4 Kolu ile Rover Kontrolü
    
    Buton/Eksen Mapping (PS4 Controller):
    - Axes[4] (R2): Gaz (ileri/geri)
    - Axes[0] (Sol Analog X): Direksiyon (sağa/sola)
    - Axes[2] (Sağ Analog X): Robotik kol yaw
    - Axes[3] (Sağ Analog Y): Robotik kol pitch
    - Buttons[4] (L1): Kol gripper aç
    - Buttons[5] (R1): Kol gripper kapat
    """
    
    def __init__(self):
        super().__init__('ps4_rover_controller')
        
        # Parametreler
        self.declare_parameter('max_linear_speed', 2.0)
        self.declare_parameter('max_angular_speed', 2.0)
        self.declare_parameter('deadzone', 0.1)
        self.declare_parameter('arm_speed_scale', 1.0)
        
        self.max_linear = self.get_parameter('max_linear_speed').value
        self.max_angular = self.get_parameter('max_angular_speed').value
        self.deadzone = self.get_parameter('deadzone').value
        self.arm_speed = self.get_parameter('arm_speed_scale').value
        
        # Publishers
        self.cmd_vel_pub = self.create_publisher(
            Twist,
            '/diff_drive_controller/cmd_vel_unstamped',
            10
        )
        
        self.arm_pub = self.create_publisher(
            Float64MultiArray,
            '/arm_controller/commands',
            10
        )
        
        # Subscriber
        self.joy_sub = self.create_subscription(
            Joy,
            '/joy',
            self.joy_callback,
            10
        )
        
        # Kol pozisyonları
        self.arm_positions = [0.0, 0.0, 0.0, 0.0]
        
        self.get_logger().info('PS4 Rover Controller başlatıldı!')
        self.get_logger().info('Kontroller:')
        self.get_logger().info('  R2: Gaz (ileri)')
        self.get_logger().info('  Sol Analog: Direksiyon')
        self.get_logger().info('  Sağ Analog: Robotik Kol')
        self.get_logger().info('  L1/R1: Gripper aç/kapat')
    
    def apply_deadzone(self, value):
        """Deadzone uygula"""
        if abs(value) < self.deadzone:
            return 0.0
        return value
    
    def joy_callback(self, msg):
        """PS4 kolu verilerini işle"""
        
        # === ROVER HAREKETİ ===
        # R2 trigger - Gaz
        r2_value = msg.axes[4] if len(msg.axes) > 4 else 1.0
        gas = (1.0 - r2_value) / 2.0
        
        # L2 trigger - Geri
        l2_value = msg.axes[5] if len(msg.axes) > 5 else 1.0
        brake = (1.0 - l2_value) / 2.0
        
        # Net lineer hız
        linear_x = (gas - brake) * self.max_linear
        
        # Sol analog X - Direksiyon
        steering = msg.axes[0] if len(msg.axes) > 0 else 0.0
        steering = self.apply_deadzone(steering)
        angular_z = -steering * self.max_angular
        
        # Twist mesajı
        cmd = Twist()
        cmd.linear.x = linear_x
        cmd.angular.z = angular_z
        self.cmd_vel_pub.publish(cmd)
        
        # === ROBOTİK KOL ===
        # Sağ analog
        right_x = msg.axes[2] if len(msg.axes) > 2 else 0.0
        right_y = msg.axes[3] if len(msg.axes) > 3 else 0.0
        
        right_x = self.apply_deadzone(right_x)
        right_y = self.apply_deadzone(right_y)
        
        # Kol eksenlerini güncelle
        dt = 0.05
        self.arm_positions[0] += right_x * self.arm_speed * dt
        self.arm_positions[1] += right_y * self.arm_speed * dt
        
        # Gripper kontrolü
        l1_pressed = msg.buttons[4] if len(msg.buttons) > 4 else 0
        r1_pressed = msg.buttons[5] if len(msg.buttons) > 5 else 0
        
        if l1_pressed:
            self.arm_positions[3] = 0.0
        elif r1_pressed:
            self.arm_positions[3] = 1.0
        
        # Pozisyonları sınırla
        self.arm_positions[0] = max(-3.14, min(3.14, self.arm_positions[0]))
        self.arm_positions[1] = max(-1.57, min(1.57, self.arm_positions[1]))
        self.arm_positions[3] = max(0.0, min(1.0, self.arm_positions[3]))
        
        # Kol komutları
        arm_cmd = Float64MultiArray()
        arm_cmd.data = self.arm_positions
        self.arm_pub.publish(arm_cmd)
        
        # Debug
        if abs(linear_x) > 0.01 or abs(angular_z) > 0.01:
            self.get_logger().info(
                f'Rover - Linear: {linear_x:.2f}, Angular: {angular_z:.2f}',
                throttle_duration_sec=1.0
            )


def main(args=None):
    rclpy.init(args=args)
    node = PS4RoverController()
    
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
