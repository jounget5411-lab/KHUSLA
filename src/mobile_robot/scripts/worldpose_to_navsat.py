#!/usr/bin/env python3
import math, rclpy
from rclpy.node import Node
from geometry_msgs.msg import Pose
from sensor_msgs.msg import NavSatFix, NavSatStatus

def meters_per_deg(lat_deg):
    lat = math.radians(lat_deg)
    m_per_deg_lat = 111132.92 - 559.82*math.cos(2*lat) + 1.175*math.cos(4*lat)
    m_per_deg_lon = 111412.84*math.cos(lat) - 93.5*math.cos(3*lat)
    return m_per_deg_lat, m_per_deg_lon

class WorldPoseToNavSat(Node):
    def __init__(self):
        super().__init__('worldpose_to_navsat')
        # world.sdf의 spherical_coordinates와 반드시 일치
        self.declare_parameter('lat0_deg', 37.2889339)
        self.declare_parameter('lon0_deg', 127.1076245)
        self.declare_parameter('alt0_m', 114.193)
        self.declare_parameter('heading_deg', 0.0)  # <spherical_coordinates><heading_deg>
        self.declare_parameter('frame_id', 'gps_link')

        # ★ 공분산(표준편차) 파라미터 추가: 시뮬 기준 보수/공격적으로 조정 가능
        self.declare_parameter('std_xy_m', 0.15)  # 15 cm
        self.declare_parameter('std_z_m', 0.30)   # 30 cm

        self.lat0 = float(self.get_parameter('lat0_deg').value)
        self.lon0 = float(self.get_parameter('lon0_deg').value)
        self.alt0 = float(self.get_parameter('alt0_m').value)
        hdeg     = float(self.get_parameter('heading_deg').value)
        self.frame_id = str(self.get_parameter('frame_id').value)

        # 공분산용 표준편차
        self.std_xy = float(self.get_parameter('std_xy_m').value)
        self.std_z  = float(self.get_parameter('std_z_m').value)

        # Gazebo(동→+X, CCW+) 기준 회전: EN→World
        self.c = math.cos(math.radians(hdeg))
        self.s = math.sin(math.radians(hdeg))
        self.mlat, self.mlon = meters_per_deg(self.lat0)

        self.sub = self.create_subscription(Pose, '/henes_t870/world_pose', self.cb, 10)
        self.pub = self.create_publisher(NavSatFix, '/gps/fix', 10)

        self.get_logger().info(
            f'anchor=({self.lat0},{self.lon0},{self.alt0}), heading={hdeg}°, '
            f'std=({self.std_xy} m horiz, {self.std_z} m vert)'
        )

    def cb(self, p: Pose):
        xw = p.position.x; yw = p.position.y; zw = p.position.z
        # World → ENU (EN→World의 역행렬)
        x_e =  self.c*xw + self.s*yw
        y_n = -self.s*xw + self.c*yw

        lat = self.lat0 + (y_n / self.mlat)
        lon = self.lon0 + (x_e / self.mlon)

        out = NavSatFix()
        out.header.stamp = self.get_clock().now().to_msg()
        out.header.frame_id = self.frame_id
        out.status.status = NavSatStatus.STATUS_FIX
        out.status.service = NavSatStatus.SERVICE_GPS
        out.latitude  = lat
        out.longitude = lon
        out.altitude  = self.alt0 + zw

        # ★ 공분산 채우기 (DIAGONAL_KNOWN=2)
        cxx = self.std_xy**2
        cyy = self.std_xy**2
        czz = self.std_z**2
        out.position_covariance = [cxx, 0.0, 0.0,
                                   0.0, cyy, 0.0,
                                   0.0, 0.0, czz]
        out.position_covariance_type = 2  # DIAGONAL_KNOWN

        self.pub.publish(out)

def main():
    rclpy.init()
    rclpy.spin(WorldPoseToNavSat())
    rclpy.shutdown()

if __name__ == '__main__':
    main()
