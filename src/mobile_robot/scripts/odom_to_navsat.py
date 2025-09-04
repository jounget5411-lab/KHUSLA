 	#!/usr/bin/env python3
import math
import rclpy
from rclpy.node import Node
from nav_msgs.msg import Odometry
from sensor_msgs.msg import NavSatFix, NavSatStatus

def meters_per_deg(lat_deg: float):
    lat = math.radians(lat_deg)
    m_per_deg_lat = 111132.92 - 559.82 * math.cos(2*lat) + 1.175 * math.cos(4*lat)
    m_per_deg_lon = 111412.84 * math.cos(lat) - 93.5 * math.cos(3*lat)
    return m_per_deg_lat, m_per_deg_lon

class OdomToNavSat(Node):
    def __init__(self):
        super().__init__('odom_to_navsat')
        # Parameters
        self.declare_parameter('lat0_deg', 37.2889339)
        self.declare_parameter('lon0_deg', 127.1076245)
        self.declare_parameter('alt0_m', 114.193)
        self.declare_parameter('map_yaw_deg', 0.0)
        self.declare_parameter('x0_m', 0.0)
        self.declare_parameter('y0_m', 0.0)
        self.declare_parameter('std_xy_m', 0.0)
        self.declare_parameter('std_z_m', 0.0)
        self.declare_parameter('frame_id', 'gps_link')

        self.lat0 = float(self.get_parameter('lat0_deg').value)
        self.lon0 = float(self.get_parameter('lon0_deg').value)
        self.alt0 = float(self.get_parameter('alt0_m').value)
        self.yaw  = math.radians(float(self.get_parameter('map_yaw_deg').value))
        self.x0   = float(self.get_parameter('x0_m').value)
        self.y0   = float(self.get_parameter('y0_m').value)
        self.std_xy = float(self.get_parameter('std_xy_m').value)
        self.std_z  = float(self.get_parameter('std_z_m').value)
        self.frame_id = str(self.get_parameter('frame_id').value)

        self.m_per_deg_lat, self.m_per_deg_lon = meters_per_deg(self.lat0)

        self.pub = self.create_publisher(NavSatFix, '/gps/fix', 10)
        self.sub = self.create_subscription(Odometry, '/odom', self.cb_odom, 10)

        self.get_logger().info(
            f"anchor=({self.lat0}, {self.lon0}, {self.alt0}), yaw={math.degrees(self.yaw):.2f}°, "
            f"offset=({self.x0},{self.y0}) m, std=({self.std_xy},{self.std_z}) m"
        )

    def cb_odom(self, msg: Odometry):
        x = msg.pose.pose.position.x - self.x0
        y = msg.pose.pose.position.y - self.y0
        z = msg.pose.pose.position.z

        c, s = math.cos(self.yaw), math.sin(self.yaw)
        x_e =  c*x + s*y
        y_n = -s*x + c*y

        dlat =  y_n / self.m_per_deg_lat
        dlon =  x_e / self.m_per_deg_lon

        out = NavSatFix()
        out.header.stamp = msg.header.stamp
        out.header.frame_id = self.frame_id
        out.status.status  = NavSatStatus.STATUS_FIX
        out.status.service = NavSatStatus.SERVICE_GPS
        out.latitude  = self.lat0 + dlat
        out.longitude = self.lon0 + dlon
        out.altitude  = self.alt0 + z

        cxx = self.std_xy**2; cyy = self.std_xy**2; czz = self.std_z**2
        out.position_covariance = [cxx,0.0,0.0, 0.0,cyy,0.0, 0.0,0.0,czz]
        out.position_covariance_type = 2  # DIAGONAL_KNOWN
        self.pub.publish(out)

def main():
    rclpy.init()
    rclpy.spin(OdomToNavSat())
    rclpy.shutdown()

if __name__ == '__main__':
    main()

