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
        self.declare_parameter('max_linear_speed', 10.0)  # Artırıldı: 2.0 → 10.0
        self.declare_parameter('max_angular_speed', 3.0)   # Artırıldı: 2.0 → 3.0
        self.declare_parameter('deadzone', 0.1)
        self.declare_parameter('arm_speed_scale', 1.0)
        
        self.max_linear = self.get_parameter('max_linear_speed').value
        self.max_angular = self.get_parameter('max_angular_speed').value
        self.deadzone = self.get_parameter('deadzone').value
        self.arm_speed = self.get_parameter('arm_speed_scale').value
        
        # Publishers
        self.cmd_vel_pub = self.create_publisher(
            Twist,
            '/cmd_vel',
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
        # DEBUGGING: Tüm axes değerlerini logla - daha fazla ondalık basamak
        if len(msg.axes) >= 6:
            self.get_logger().info(
                f'JOY axes: [0]={msg.axes[0]:.4f} [1]={msg.axes[1]:.4f} [2]={msg.axes[2]:.4f} [3]={msg.axes[3]:.4f} [4]={msg.axes[4]:.4f} [5]={msg.axes[5]:.4f}',
                throttle_duration_sec=0.5
            )
        
        # R2 trigger - axes[5] kullan (PS4'te: basılmamış=1.0, tam basılı=-1.0)
        r2_value = msg.axes[5] if len(msg.axes) > 5 else 1.0
        
        # R2: 1.0 (basılmamış) → -1.0 (tam basılı)
        # Gas: 0.0 (basılmamış) → 1.0 (tam basılı)
        gas = (1.0 - r2_value) / 2.0  # 1.0→0.0, -1.0→1.0
        
        # Deadzone uygula
        if gas < 0.05:
            gas = 0.0
        
        # Karesel scaling - daha yumuşak kontrol
        gas = pow(gas, 0.7)  # 0.7 power: yumuşak hızlanma
        
        # Net lineer hız
        linear_x = gas * self.max_linear
        
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
        
        # Debug - her zaman göster
        self.get_logger().info(
            f'R2={r2_value:.4f} → gas={gas:.4f} → Linear={linear_x:.4f}, Angular={angular_z:.4f}',
            throttle_duration_sec=0.5
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
