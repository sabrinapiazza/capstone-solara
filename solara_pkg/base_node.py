"""
base_node: the only bridge between ROS and the Pico.

  /cmd_vel (Twist)  ->  wheel speeds  ->  "V <left> <right>\\n" over USB serial
  "E <left_ticks> <right_ticks>\\n"   ->  /odom (Odometry) + odom -> base_link TF

Serial protocol (owned by the solara-firmware repo, see PROTOCOL.md there):
  Pi  -> Pico : V <left_rad_s> <right_rad_s>     target wheel speeds
  Pico -> Pi  : E <left_ticks> <right_ticks>     running encoder totals, ~50 Hz
  Pico -> Pi  : FW <version>                     printed once on startup

Skid-steer note: each side is treated as one wheel. Tune `track_width` with the
spin test (command a 360 deg turn, adjust until the robot ends where it began).
"""

import math

import rclpy
import serial
from geometry_msgs.msg import TransformStamped, Twist
from nav_msgs.msg import Odometry
from rclpy.node import Node
from tf2_ros import TransformBroadcaster


def yaw_to_quaternion(yaw):
    return (0.0, 0.0, math.sin(yaw / 2.0), math.cos(yaw / 2.0))


class BaseNode(Node):
    def __init__(self):
        super().__init__('base_node')

        # --- parameters (override in config/base_params.yaml) ---
        self.port = self.declare_parameter('serial_port', '/dev/ttyACM0').value
        self.wheel_radius = self.declare_parameter('wheel_radius', 0.05).value   # m
        self.track_width = self.declare_parameter('track_width', 0.30).value     # m, tune with spin test
        self.ticks_per_rev = self.declare_parameter('ticks_per_rev', 1440.0).value
        self.publish_tf = self.declare_parameter('publish_tf', True).value       # False if an EKF takes over
        self.cmd_timeout = self.declare_parameter('cmd_timeout', 0.5).value      # s, stop if no cmd_vel
        self.odom_frame = self.declare_parameter('odom_frame', 'odom').value
        self.base_frame = self.declare_parameter('base_frame', 'base_link').value

        # --- serial ---
        self.ser = serial.Serial(self.port, 115200, timeout=0)
        self.rx_buffer = ''
        self.get_logger().info(f'Opened {self.port}')

        # --- ROS interfaces ---
        self.create_subscription(Twist, 'cmd_vel', self.on_cmd_vel, 10)
        self.odom_pub = self.create_publisher(Odometry, 'odom', 10)
        self.tf_broadcaster = TransformBroadcaster(self)

        # --- state ---
        self.target = (0.0, 0.0)                    # left, right rad/s
        self.last_cmd_time = self.get_clock().now()
        self.last_ticks = None
        self.last_odom_time = None
        self.x = self.y = self.yaw = 0.0

        self.create_timer(0.05, self.send_command)  # 20 Hz to the Pico
        self.create_timer(0.01, self.read_serial)   # poll for encoder lines

    # ---------- commands: ROS -> Pico ----------
    def on_cmd_vel(self, msg: Twist):
        v, w = msg.linear.x, msg.angular.z
        left = (v - w * self.track_width / 2.0) / self.wheel_radius
        right = (v + w * self.track_width / 2.0) / self.wheel_radius
        self.target = (left, right)
        self.last_cmd_time = self.get_clock().now()

    def send_command(self):
        age = (self.get_clock().now() - self.last_cmd_time).nanoseconds * 1e-9
        left, right = self.target if age < self.cmd_timeout else (0.0, 0.0)
        self.ser.write(f'V {left:.3f} {right:.3f}\n'.encode())

    # ---------- feedback: Pico -> ROS ----------
    def read_serial(self):
        data = self.ser.read(self.ser.in_waiting or 1)
        if not data:
            return
        self.rx_buffer += data.decode(errors='ignore')
        while '\n' in self.rx_buffer:
            line, self.rx_buffer = self.rx_buffer.split('\n', 1)
            self.handle_line(line.strip())

    def handle_line(self, line):
        parts = line.split()
        if not parts:
            return
        if parts[0] == 'FW':
            self.get_logger().info(f'Pico firmware {" ".join(parts[1:])}')
        elif parts[0] == 'E' and len(parts) == 3:
            try:
                self.update_odometry(int(parts[1]), int(parts[2]))
            except ValueError:
                self.get_logger().warn(f'Bad encoder line: {line!r}')

    def update_odometry(self, left_ticks, right_ticks):
        now = self.get_clock().now()
        if self.last_ticks is None:
            self.last_ticks, self.last_odom_time = (left_ticks, right_ticks), now
            return

        dt = (now - self.last_odom_time).nanoseconds * 1e-9
        if dt <= 0.0:
            return
        meters_per_tick = 2.0 * math.pi * self.wheel_radius / self.ticks_per_rev
        d_left = (left_ticks - self.last_ticks[0]) * meters_per_tick
        d_right = (right_ticks - self.last_ticks[1]) * meters_per_tick
        self.last_ticks, self.last_odom_time = (left_ticks, right_ticks), now

        d = (d_left + d_right) / 2.0
        d_yaw = (d_right - d_left) / self.track_width
        self.x += d * math.cos(self.yaw + d_yaw / 2.0)
        self.y += d * math.sin(self.yaw + d_yaw / 2.0)
        self.yaw += d_yaw
        self.publish_odometry(now, d / dt, d_yaw / dt)

    def publish_odometry(self, stamp, v, w):
        q = yaw_to_quaternion(self.yaw)

        odom = Odometry()
        odom.header.stamp = stamp.to_msg()
        odom.header.frame_id = self.odom_frame
        odom.child_frame_id = self.base_frame
        odom.pose.pose.position.x = self.x
        odom.pose.pose.position.y = self.y
        (odom.pose.pose.orientation.x, odom.pose.pose.orientation.y,
         odom.pose.pose.orientation.z, odom.pose.pose.orientation.w) = q
        odom.twist.twist.linear.x = v
        odom.twist.twist.angular.z = w
        self.odom_pub.publish(odom)

        if self.publish_tf:
            t = TransformStamped()
            t.header = odom.header
            t.child_frame_id = self.base_frame
            t.transform.translation.x = self.x
            t.transform.translation.y = self.y
            (t.transform.rotation.x, t.transform.rotation.y,
             t.transform.rotation.z, t.transform.rotation.w) = q
            self.tf_broadcaster.sendTransform(t)

    def stop(self):
        self.ser.write(b'V 0 0\n')
        self.ser.close()


def main():
    rclpy.init()
    node = BaseNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.stop()
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
