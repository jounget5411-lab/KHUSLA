#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Imu

class ImuCovInjector(Node):
    def __init__(self):
        super().__init__('imu_cov_injector')

        # 파라미터(필요시 튜닝)
        # 표준편차^2 = 분산(공분산 대각성분)
        # orientation: 약 1.7°(0.03rad) 정도 → 0.03^2 ≈ 9.0e-4
        self.declare_parameter('ori_var', 9.0e-4)
        # angular velocity: 0.02 rad/s → 4.0e-4
        self.declare_parameter('gyro_var', 4.0e-4)
        # linear accel: 0.05 m/s^2 → 2.5e-3
        self.declare_parameter('acc_var', 2.5e-3)

        ori_var  = float(self.get_parameter('ori_var').value)
        gyro_var = float(self.get_parameter('gyro_var').value)
        acc_var  = float(self.get_parameter('acc_var').value)

        self._ori_cov  = [0.0]*9
        self._gyro_cov = [0.0]*9
        self._acc_cov  = [0.0]*9
        self._ori_cov[0] = self._ori_cov[4] = self._ori_cov[8] = ori_var
        self._gyro_cov[0] = self._gyro_cov[4] = self._gyro_cov[8] = gyro_var
        self._acc_cov[0] = self._acc_cov[4] = self._acc_cov[8] = acc_var

        # 토픽 이름(필요하면 변경 가능)
        self.declare_parameter('in_topic',  '/imu')
        self.declare_parameter('out_topic', '/imu/with_cov')

        in_topic  = self.get_parameter('in_topic').value
        out_topic = self.get_parameter('out_topic').value

        self.sub = self.create_subscription(Imu, in_topic, self._cb, 50)
        self.pub = self.create_publisher(Imu, out_topic, 50)

        self.get_logger().info(
            f"ImuCovInjector: in='{in_topic}' → out='{out_topic}', "
            f"diag vars: ori={ori_var}, gyro={gyro_var}, acc={acc_var}"
        )

    def _is_zero_cov(self, cov):
        return len(cov) == 9 and all(c == 0.0 for c in cov)

    def _cb(self, msg: Imu):
        out = Imu()
        out.header = msg.header
        out.orientation = msg.orientation
        out.angular_velocity = msg.angular_velocity
        out.linear_acceleration = msg.linear_acceleration

        # 0(=완벽)로 오면 우리가 채움. (-1은 “없음” 의미라면 그대로 둘 수도 있음)
        out.orientation_covariance = msg.orientation_covariance[:] if not self._is_zero_cov(msg.orientation_covariance) else self._ori_cov[:]
        out.angular_velocity_covariance = msg.angular_velocity_covariance[:] if not self._is_zero_cov(msg.angular_velocity_covariance) else self._gyro_cov[:]
        out.linear_acceleration_covariance = msg.linear_acceleration_covariance[:] if not self._is_zero_cov(msg.linear_acceleration_covariance) else self._acc_cov[:]

        self.pub.publish(out)

def main():
    rclpy.init()
    rclpy.spin(ImuCovInjector())
    rclpy.shutdown()

if __name__ == "__main__":
    main()

