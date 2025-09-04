#!/usr/bin/env python3
import math, rclpy
from rclpy.node import Node
from rclpy.clock import Clock, ClockType
from geometry_msgs.msg import Twist

class TwistToAckermannFixed(Node):
    """
    입력:  /cmd_vel_teleop (Twist)  # v[m/s], w[yaw rate rad/s]
    출력:  /cmd_vel        (Twist)  # v[m/s], delta[rad]
    고정 주기(rate_hz)로 퍼블리시하여 명령 주기 불규칙성 제거.
    """
    def __init__(self):
        super().__init__('twist_to_ackermann_fixed')
        self.declare_parameter('wheel_base_m', 0.71)
        self.declare_parameter('steer_limit_rad', 0.6)
        self.declare_parameter('eps_speed', 0.1)
        self.declare_parameter('alpha', 0.4)     # 0~1, 클수록 반응 빠름
        self.declare_parameter('rate_hz', 60.0)  # 고정 퍼블리시 주기

        self.L = float(self.get_parameter('wheel_base_m').value)
        self.dlim = float(self.get_parameter('steer_limit_rad').value)
        self.eps = float(self.get_parameter('eps_speed').value)
        self.alpha = float(self.get_parameter('alpha').value)
        self.dt = 1.0 / float(self.get_parameter('rate_hz').value)

        self._v_in = 0.0
        self._w_in = 0.0
        self._delta = 0.0  # LPF 상태

        self.sub = self.create_subscription(Twist, '/cmd_vel_teleop', self.cb, 10)
        self.pub = self.create_publisher(Twist, '/cmd_vel', 10)
        # 벽시계(시뮬 시간 흔들림 영향 제거)
        self.timer = self.create_timer(self.dt, self.on_timer,
                                       clock=Clock(clock_type=ClockType.STEADY_TIME))

        self.get_logger().info(f'L={self.L}m, steer_lim=±{self.dlim}rad, eps={self.eps}, '
                               f'alpha={self.alpha}, rate={1.0/self.dt:.1f}Hz')

    def clamp(self, x, lo, hi):
        return lo if x < lo else hi if x > hi else x

    def cb(self, tw: Twist):
        self._v_in = float(tw.linear.x)
        self._w_in = float(tw.angular.z)

    def on_timer(self):
        v = self._v_in
        w = self._w_in
        if abs(v) < self.eps:
            delta_cmd = self.clamp(w, -self.dlim, self.dlim)   # 정지상태는 조향각으로 해석
        else:
            delta_cmd = math.atan(self.L * w / v)
            delta_cmd = self.clamp(delta_cmd, -self.dlim, self.dlim)

        # 1차 저역통과
        self._delta += self.alpha * (delta_cmd - self._delta)

        out = Twist()
        out.linear.x = v
        out.angular.z = self._delta
        self.pub.publish(out)

def main():
    rclpy.init()
    rclpy.spin(TwistToAckermannFixed())
    rclpy.shutdown()

if __name__ == '__main__':
    main()
