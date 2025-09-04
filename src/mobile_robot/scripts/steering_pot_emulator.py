#!/usr/bin/env python3
import math
import rclpy
from rclpy.node import Node
from std_msgs.msg import Float32

class SteeringPotEmulator(Node):
    """
    - 구독:  /vehicle/steering_cmd   (Float32, 단위: rad)
    - 발행:  /vehicle/steering_angle (Float32, 단위: rad)  ← 포텐셔미터가 읽은 '현재 조향각' 가정
             /vehicle/pot_voltage    (Float32, 단위: V, 선택적 모니터링)
    - 모델:  1차 지연 시스템(τ) + 포텐셔미터 선형 맵핑, 각도 제한 포함
    """
    def __init__(self):
        super().__init__('steering_pot_emulator')

        # 파라미터
        self.declare_parameter('rate_hz', 50.0)            # 발행 주기
        self.declare_parameter('tau_sec', 0.20)            # 1차 지연 시정수
        self.declare_parameter('steer_limit_rad', 0.6)     # 물리적 조향 한계(URDF와 맞춤)
        self.declare_parameter('pot_min_v', 0.5)           # 포텐셔미터 하한 전압
        self.declare_parameter('pot_max_v', 4.5)           # 포텐셔미터 상한 전압

        self.rate_hz = float(self.get_parameter('rate_hz').value)
        self.tau = float(self.get_parameter('tau_sec').value)
        self.steer_lim = float(self.get_parameter('steer_limit_rad').value)
        self.vmin = float(self.get_parameter('pot_min_v').value)
        self.vmax = float(self.get_parameter('pot_max_v').value)

        # 상태
        self.cmd = 0.0     # 목표 조향각 [rad]
        self.ang = 0.0     # 현재 조향각 [rad]

        # I/O
        self.sub_cmd = self.create_subscription(Float32, '/vehicle/steering_cmd', self.cb_cmd, 10)
        self.pub_ang = self.create_publisher(Float32, '/vehicle/steering_angle', 10)
        self.pub_pot = self.create_publisher(Float32, '/vehicle/pot_voltage', 10)

        # 타이머
        self.dt = 1.0 / self.rate_hz
        self.timer = self.create_timer(self.dt, self.on_timer)

        self.get_logger().info(
            f'rate={self.rate_hz}Hz, tau={self.tau}s, limit=±{self.steer_lim}rad, pot=[{self.vmin},{self.vmax}]V'
        )

    def cb_cmd(self, msg: Float32):
        # 입력 명령 제한
        self.cmd = max(-self.steer_lim, min(self.steer_lim, float(msg.data)))

    def on_timer(self):
        # 1차 지연(연속계 근사): dang/dt = (cmd - ang)/tau
        # 이산화: ang += (cmd - ang) * dt/tau
        if self.tau > 1e-6:
            self.ang += (self.cmd - self.ang) * (self.dt / self.tau)
        else:
            self.ang = self.cmd

        # 포텐셔미터 전압으로 선형 맵: [-lim, +lim] → [vmin, vmax]
        # ratio = (ang + lim)/(2*lim)
        ratio = (self.ang + self.steer_lim) / (2.0 * self.steer_lim)
        ratio = max(0.0, min(1.0, ratio))
        v = self.vmin + (self.vmax - self.vmin) * ratio

        # 발행
        self.pub_ang.publish(Float32(data=self.ang))
        self.pub_pot.publish(Float32(data=v))

def main():
    rclpy.init()
    rclpy.spin(SteeringPotEmulator())
    rclpy.shutdown()

if __name__ == '__main__':
    main()

