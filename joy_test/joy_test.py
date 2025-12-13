#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Joy
from std_msgs.msg import Int32
import math

class WheelControlIntNode(Node):
    def __init__(self):
        super().__init__('wheel_controller')
        
        self.subscription = self.create_subscription(Joy, 'joy', self.joy_callback, 10)
        
        self.fl_speed_pub = self.create_publisher(Int32, '/wheel_front_left_speed', 10)
        self.fl_angle_pub = self.create_publisher(Int32, '/wheel_front_left_angle', 10)
        
        self.fr_speed_pub = self.create_publisher(Int32, '/wheel_front_right_speed', 10)
        self.fr_angle_pub = self.create_publisher(Int32, '/wheel_front_right_angle', 10)
        
        self.rl_speed_pub = self.create_publisher(Int32, '/wheel_rear_left_speed', 10)
        self.rl_angle_pub = self.create_publisher(Int32, '/wheel_rear_left_angle', 10)
        
        self.rr_speed_pub = self.create_publisher(Int32, '/wheel_rear_right_speed', 10)
        self.rr_angle_pub = self.create_publisher(Int32, '/wheel_rear_right_angle', 10)

        self.get_logger().info("Tekerlek Kontrolcüsü Başlatıldı (Merkezi Dönüş Modu)...")
        
        self.axis_speed_index = 1 
        self.axis_angle_index = 0
        
        self.W = 84.0 
        self.L = 113.0 
        self.max_steer_angle = 45.0 

    def joy_callback(self, msg):
        raw_speed = msg.axes[self.axis_speed_index] * 63.0
        raw_turn  = msg.axes[self.axis_angle_index]
        
        if abs(raw_speed) < 2.0: raw_speed = 0.0
        if abs(raw_turn) < 0.05: raw_turn = 0.0

        fl_ang = fr_ang = rl_ang = rr_ang = 180
        fl_spd = fr_spd = rl_spd = rr_spd = int(raw_speed)

        if abs(raw_turn) > 0.001:
            steer_angle_rad = abs(raw_turn) * math.radians(self.max_steer_angle)
            radius = (self.L / 2.0) / math.tan(steer_angle_rad)
            
            inner_angle = math.degrees(math.atan((self.L / 2.0) / (radius - self.W / 2.0)))
            outer_angle = math.degrees(math.atan((self.L / 2.0) / (radius + self.W / 2.0)))
            
            
            dist_inner = math.sqrt((radius - self.W/2.0)**2 + (self.L/2.0)**2)
            dist_outer = math.sqrt((radius + self.W/2.0)**2 + (self.L/2.0)**2)
            dist_center = math.sqrt(radius**2) 
            ratio_inner = dist_inner / dist_center
            ratio_outer = dist_outer / dist_center
            
            vel_inner = raw_speed * ratio_inner
            vel_outer = raw_speed * ratio_outer
            
            if raw_turn > 0:
                fl_ang = int(180 - inner_angle)
                fr_ang = int(180 - outer_angle)
                rl_ang = int(180 + inner_angle)
                rr_ang = int(180 + outer_angle)
                fl_spd = int(vel_inner)
                rl_spd = int(vel_inner)
                fr_spd = int(vel_outer)
                rr_spd = int(vel_outer)
                
            else:
                fl_ang = int(180 + outer_angle)
                fr_ang = int(180 + inner_angle)
                rl_ang = int(180 - outer_angle)
                rr_ang = int(180 - inner_angle)
                fr_spd = int(vel_inner)
                rr_spd = int(vel_inner)
                fl_spd = int(vel_outer)
                rl_spd = int(vel_outer)

        self.publish_int32(self.fl_speed_pub, fl_spd)
        self.fr_speed_pub.publish(self.create_int(fr_spd))
        self.rl_speed_pub.publish(self.create_int(rl_spd))
        self.rr_speed_pub.publish(self.create_int(rr_spd))
        
        self.fl_angle_pub.publish(self.create_int(fl_ang))
        self.fr_angle_pub.publish(self.create_int(fr_ang))
        self.rl_angle_pub.publish(self.create_int(rl_ang))
        self.rr_angle_pub.publish(self.create_int(rr_ang))

    def create_int(self, val):
        msg = Int32()
        msg.data = int(val)
        return msg
        
    def publish_int32(self, publisher, value):
        msg = Int32()
        msg.data = int(value)
        publisher.publish(msg)

def main(args=None):
    rclpy.init(args=args)
    node = WheelControlIntNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()