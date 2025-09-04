#!/usr/bin/env python3
import math
import rclpy
from rclpy.node import Node
from std_msgs.msg import Float32
from geometry_msgs.msg import Twist

class VehicleInterface(Node):
    """
    입력:
      - /vehicle/steering_cmd (Float32, rad) : 조향 명령(기호=좌우, 한계=±steer_limit)
      - /vehicle/speed_cmd    (Float32, m/s) : 속도 명령(기호=전후)
    출력:
      - /cmd_vel (Twist) : ackermann 근사 yaw_rate = v/L * tan(delta)
    """
    def __init__(self):
        super().__init__('vehicle_interface')

        # 파라미터
        self.declare_parameter('wheel_base_m', 0.71)       # URDF와 일치
        self.declare_parameter('steer_limit_rad', 0.628)     # URDF와 일치
        self.declare_parameter('speed_limit_mps', 1.94)
        self.declare_parameter('accel_limit_mps2', 3.0)    # 가감속 제한
        self.declare_parameter('rate_hz', 20.0)

        self.L   = float(self.get_parameter('wheel_base_m').value)
        self.dlim= float(self.get_parameter('steer_limit_rad').value)
        self.vlim= float(self.get_parameter('speed_limit_mps').value)
        self.alim= float(self.get_parameter('accel_limit_mps2').value)
        self.dt  = 1.0/float(self.get_parameter('rate_hz').value)

        # 상태
        self._d_cmd = 0.0  # 목표 조향(rad)
        self._v_cmd = 0.0  # 목표 속도(m/s)
        self._v     = 0.0  # 현재 속도 추정(가속 제한 적용)

        # IO
        self.pub_twist = self.create_publisher(Twist, '/cmd_vel', 10)
        self.sub_delta = self.create_subscription(Float32, '/vehicle/steering_cmd', self._on_delta, 10)
        self.sub_speed = self.create_subscription(Float32, '/vehicle/speed_cmd',    self._on_speed, 10)

        self.timer = self.create_timer(self.dt, self._on_timer)
        self.get_logger().info(
            f'L={self.L:.3f} m, steer_lim=±{self.dlim:.3f} rad, v_lim=±{self.vlim:.2f} m/s, a_lim={self.alim:.2f} m/s^2'
        )

    def _clamp(self, x, xmin, xmax): return xmin if x < xmin else xmax if x > xmax else x

    def _on_delta(self, msg: Float32):
        self._d_cmd = self._clamp(float(msg.data), -self.dlim, self.dlim)

    def _on_speed(self, msg: Float32):
        self._v_cmd = self._clamp(float(msg.data), -self.vlim, self.vlim)

    def _on_timer(self):
        # 가감속 제한
        dv = self._v_cmd - self._v
        max_step = self.alim * self.dt
        if dv >  max_step: dv =  max_step
        if dv < -max_step: dv = -max_step
        self._v += dv

        # Ackermann 근사: yaw_rate = v/L * tan(delta)
        yaw_rate = 0.0
        if abs(self.L) > 1e-6:
            yaw_rate = self._v / self.L * math.tan(self._d_cmd)

        # 퍼블리시
        tw = Twist()
        tw.linear.x  = self._v
        tw.angular.z = yaw_rate
        self.pub_twist.publish(tw)

def main():
    rclpy.init()
    rclpy.spin(VehicleInterface())
    rclpy.shutdown()

if __name__ == '__main__':
    main()
