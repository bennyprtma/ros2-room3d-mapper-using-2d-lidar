#!/usr/bin/env python3
import math
import threading
import time

import rclpy
import serial
from geometry_msgs.msg import PoseStamped, Quaternion, TransformStamped, Twist
from nav_msgs.msg import Odometry, Path
from rclpy.node import Node
from std_msgs.msg import Bool, Empty, Float32
from tf2_ros import TransformBroadcaster


def yaw_to_quat(yaw: float) -> Quaternion:
    q = Quaternion()
    q.z = math.sin(yaw * 0.5)
    q.w = math.cos(yaw * 0.5)
    return q


class OdomNode(Node):
    def __init__(self):
        super().__init__('odom_node')

        # ---------- parameter ----------
        d = self.declare_parameter
        d('port', '/dev/ttyUSB0')
        d('baud', 115200)
        d('serial_settle', 2.5)
        d('sweep_resend', 2.0)

        d('wheel_radius', 0.03)
        d('wheel_separation', 0.135)
        d('ticks_per_rev', 1152)
        d('invert_left', False)
        d('invert_right', False)

        # frame
        d('odom_frame', 'odom')
        d('base_frame', 'base_link')
        d('laser_frame', 'laser')

        d('publish_tf', True)
        d('publish_laser_tf', True)
        d('publish_path', True)
        d('path_min_dist', 0.02)
        d('path_max_poses', 3000)
        d('cmd_timeout', 0.5)


        d('pivot_x', 0.05)
        d('pivot_z', 0.20)
        d('arm_x', 0.0)
        d('arm_z', 0.07)
        d('tilt_sign', 1.0)
        d('tilt_offset_deg', 0.0)
        d('tilt_tf_rate', 100.0)
        d('start_sweep', True)

        g = lambda n: self.get_parameter(n).value
        self.wheel_radius = float(g('wheel_radius'))
        self.wheel_sep = float(g('wheel_separation'))
        self.tpr = int(g('ticks_per_rev'))
        self.inv_l = bool(g('invert_left'))
        self.inv_r = bool(g('invert_right'))
        self.odom_frame = g('odom_frame')
        self.base_frame = g('base_frame')
        self.laser_frame = g('laser_frame')
        self.pub_tf = bool(g('publish_tf'))
        self.pub_laser_tf = bool(g('publish_laser_tf'))
        self.pub_path = bool(g('publish_path'))
        self.path_min_dist = float(g('path_min_dist'))
        self.path_max = int(g('path_max_poses'))
        self.cmd_timeout = float(g('cmd_timeout'))

        self.pivot_x = float(g('pivot_x'))
        self.pivot_z = float(g('pivot_z'))
        self.arm_x = float(g('arm_x'))
        self.arm_z = float(g('arm_z'))
        self.tilt_sign = float(g('tilt_sign'))
        self.tilt_offset = math.radians(float(g('tilt_offset_deg')))

        self.ticks_per_m = self.tpr / (2.0 * math.pi * self.wheel_radius)

        self.x = 0.0
        self.y = 0.0
        self.th = 0.0
        self.vx = 0.0
        self.wz = 0.0
        self.prev_l = None
        self.prev_r = None
        self.prev_t = None
        self.tilt = 0.0
        self.last_cmd_time = self.get_clock().now()
        self.lock = threading.Lock()
        self._alive = True

        port = g('port')
        baud = int(g('baud'))
        try:
            self.ser = serial.Serial(port, baud, timeout=0.2)
        except Exception as exc:
            self.get_logger().error(f'Gagal membuka port {port}: {exc}')
            raise SystemExit(1)
        self.get_logger().info(f'Port serial {port} @ {baud} terbuka')
        settle = float(g('serial_settle'))
        if settle > 0.0:
            self.get_logger().info(f'Menunggu Arduino siap {settle:.1f} s ...')
            time.sleep(settle)
            try:
                self.ser.reset_input_buffer()
            except Exception:
                pass

        self.pub_odom = self.create_publisher(Odometry, 'odom', 20)
        self.pub_servo = self.create_publisher(Float32, 'servo_angle', 20)
        self.pub_pathmsg = self.create_publisher(Path, 'odom_path', 5)
        self.br = TransformBroadcaster(self)

        self.create_subscription(Twist, 'cmd_vel', self.on_cmd, 10)
        self.create_subscription(Empty, 'reset_odom', self.on_reset, 10)
        self.create_subscription(Bool, 'tilt_enable', self.on_tilt_enable, 10)

        self.path = Path()
        self.path.header.frame_id = self.odom_frame

        self.create_timer(0.1, self.watchdog)
        self.create_timer(2.0, self.report)
        rate = max(10.0, float(g('tilt_tf_rate')))
        if self.pub_laser_tf:
            self.create_timer(1.0 / rate, self.publish_laser_tf)

        self.thread = threading.Thread(target=self._serial_loop, daemon=True)
        self.thread.start()

        self.sweep_want = bool(g('start_sweep'))
        self.set_sweep(self.sweep_want)
        resend = float(g('sweep_resend'))
        if resend > 0.0:
            self.create_timer(resend, lambda: self.set_sweep(self.sweep_want, quiet=True))

    # ================= perintah =================
    def on_cmd(self, msg: Twist):
        v = float(msg.linear.x)
        w = float(msg.angular.z)
        half = 0.5 * self.wheel_sep * w
        self._write_motor(v - half, v + half)
        self.last_cmd_time = self.get_clock().now()

    def _write_motor(self, vl: float, vr: float):
        if not self._alive:
            return
        try:
            self.ser.write(f'M {vl:.4f} {vr:.4f}\n'.encode())
        except Exception as exc:
            self.get_logger().warn(f'Gagal kirim motor: {exc}')

    def set_sweep(self, on: bool, quiet: bool = False):
        try:
            self.ser.write(f'S {1 if on else 0}\n'.encode())
            if not quiet:
                self.get_logger().info(f'Sweep servo: {"ON" if on else "PARKIR"}')
        except Exception as exc:
            self.get_logger().warn(f'Gagal kirim sweep: {exc}')

    def on_tilt_enable(self, msg: Bool):
        self.sweep_want = bool(msg.data)
        self.set_sweep(self.sweep_want)

    def watchdog(self):
        dt = (self.get_clock().now() - self.last_cmd_time).nanoseconds * 1e-9
        if dt > self.cmd_timeout:
            self._write_motor(0.0, 0.0)

    def on_reset(self, _msg: Empty):
        with self.lock:
            self.x = self.y = self.th = 0.0
            self.prev_l = self.prev_r = None
            self.path.poses.clear()
        self.get_logger().info('Odometri direset ke nol')

    def _serial_loop(self):
        buf = b''
        while self._alive and rclpy.ok():
            try:
                chunk = self.ser.read(256)
                if chunk:
                    buf += chunk
                    while b'\n' in buf:
                        raw, buf = buf.split(b'\n', 1)
                        line = raw.decode('ascii', 'ignore').strip()
                        if line:
                            self._process(line)
            except Exception as exc:
                if self._alive and rclpy.ok():
                    self.get_logger().warn(f'Serial: {exc}')
                return

    def _process(self, line: str):
        if not line.startswith('E'):
            return
        parts = line.split()
        if len(parts) < 3:
            return
        try:
            tl = int(parts[1])
            tr = int(parts[2])
            tilt_deg = float(parts[3]) / 100.0 if len(parts) >= 4 else 0.0
        except ValueError:
            return

        if self.inv_l:
            tl = -tl
        if self.inv_r:
            tr = -tr

        self.tilt = self.tilt_sign * math.radians(tilt_deg) + self.tilt_offset

        now = self.get_clock().now()
        if self.prev_l is None:
            self.prev_l, self.prev_r, self.prev_t = tl, tr, now
            return

        dt = (now - self.prev_t).nanoseconds * 1e-9
        if dt <= 0.0:
            return

        dl = (tl - self.prev_l) / self.ticks_per_m
        dr = (tr - self.prev_r) / self.ticks_per_m
        self.prev_l, self.prev_r, self.prev_t = tl, tr, now

        dc = 0.5 * (dl + dr)
        dth = (dr - dl) / self.wheel_sep

        with self.lock:
            # integrasi titik tengah: jauh lebih akurat saat berputar
            self.x += dc * math.cos(self.th + 0.5 * dth)
            self.y += dc * math.sin(self.th + 0.5 * dth)
            self.th = math.atan2(math.sin(self.th + dth), math.cos(self.th + dth))
            self.vx = dc / dt
            self.wz = dth / dt

        self.publish(now)

        msg = Float32()
        msg.data = math.degrees(self.tilt)
        self.pub_servo.publish(msg)

    def publish(self, stamp):
        with self.lock:
            x, y, th, vx, wz = self.x, self.y, self.th, self.vx, self.wz

        q = yaw_to_quat(th)
        st = stamp.to_msg()

        odom = Odometry()
        odom.header.stamp = st
        odom.header.frame_id = self.odom_frame
        odom.child_frame_id = self.base_frame
        odom.pose.pose.position.x = x
        odom.pose.pose.position.y = y
        odom.pose.pose.orientation = q
        odom.twist.twist.linear.x = vx
        odom.twist.twist.angular.z = wz
        odom.pose.covariance[0] = 0.002
        odom.pose.covariance[7] = 0.002
        odom.pose.covariance[35] = 0.005
        self.pub_odom.publish(odom)

        if self.pub_tf:
            t = TransformStamped()
            t.header.stamp = st
            t.header.frame_id = self.odom_frame
            t.child_frame_id = self.base_frame
            t.transform.translation.x = x
            t.transform.translation.y = y
            t.transform.rotation = q
            self.br.sendTransform(t)

        if self.pub_path:
            add = True
            if self.path.poses:
                last = self.path.poses[-1].pose.position
                if math.hypot(x - last.x, y - last.y) < self.path_min_dist:
                    add = False
            if add:
                ps = PoseStamped()
                ps.header.stamp = st
                ps.header.frame_id = self.odom_frame
                ps.pose.position.x = x
                ps.pose.position.y = y
                ps.pose.orientation = q
                self.path.poses.append(ps)
                if len(self.path.poses) > self.path_max:
                    del self.path.poses[0]
                self.path.header.stamp = st
                self.pub_pathmsg.publish(self.path)

    def publish_laser_tf(self):
        """TF base_link -> laser dengan model lengan pivot servo.

        Titik lidar = pivot + Rx(tilt) * lengan.
        Tanpa model lengan ini, dinding akan tampak melengkung saat servo
        mengayun karena lidar sebenarnya BERPINDAH, bukan hanya berputar.
        """
        if not rclpy.ok():
            return
        tilt = self.tilt
        s, c = math.sin(tilt), math.cos(tilt)

        t = TransformStamped()
        t.header.stamp = self.get_clock().now().to_msg()
        t.header.frame_id = self.base_frame
        t.child_frame_id = self.laser_frame
        t.transform.translation.x = self.pivot_x + self.arm_x
        t.transform.translation.y = -self.arm_z * s
        t.transform.translation.z = self.pivot_z + self.arm_z * c
        t.transform.rotation.x = math.sin(tilt * 0.5)
        t.transform.rotation.w = math.cos(tilt * 0.5)
        try:
            self.br.sendTransform(t)
        except Exception:
            pass

    def report(self):
        with self.lock:
            self.get_logger().info(
                f'pose x={self.x:+.3f} y={self.y:+.3f} yaw={math.degrees(self.th):+7.2f} deg '
                f'| tilt={math.degrees(self.tilt):+6.2f} deg'
            )

    def stop(self):
        self._alive = False
        try:
            self.ser.write(b'M 0 0\n')
            self.ser.write(b'S 0\n')
        except Exception:
            pass
        if self.thread.is_alive():
            self.thread.join(timeout=1.0)
        try:
            self.ser.close()
        except Exception:
            pass


def main():
    rclpy.init()
    node = OdomNode()
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
