#!/usr/bin/env python3
import math
import select
import sys
import termios
import tty

import rclpy
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
from rclpy.node import Node
from std_msgs.msg import Bool, Empty


def norm_angle(a: float) -> float:
    return math.atan2(math.sin(a), math.cos(a))


class ArrowTeleop(Node):
    def __init__(self):
        super().__init__('arrow_teleop')

        d = self.declare_parameter
        d('linear_speed', 0.15)
        d('angular_speed', 0.8)
        d('min_angular', 0.35)      
        d('turn_step_deg', 30.0)
        d('tolerance_deg', 1.5)
        d('kp', 2.5)
        d('control_period', 0.02)
        d('turn_timeout', 8.0)
        d('use_odom', True)
        d('odom_timeout', 1.0)
        d('ignore_key_while_turning', True)

        g = lambda n: self.get_parameter(n).value
        self.lin = float(g('linear_speed'))
        self.ang = float(g('angular_speed'))
        self.min_ang = float(g('min_angular'))
        self.step = math.radians(float(g('turn_step_deg')))
        self.tol = math.radians(float(g('tolerance_deg')))
        self.kp = float(g('kp'))
        self.timeout = float(g('turn_timeout'))
        self.use_odom = bool(g('use_odom'))
        self.odom_timeout = float(g('odom_timeout'))
        self.ignore_while_turning = bool(g('ignore_key_while_turning'))

        self.pub = self.create_publisher(Twist, 'cmd_vel', 10)
        self.pub_reset = self.create_publisher(Empty, 'reset_odom', 5)
        self.pub_clear = self.create_publisher(Empty, 'clear_cloud', 5)
        self.pub_save = self.create_publisher(Empty, 'save_cloud', 5)
        self.pub_tilt = self.create_publisher(Bool, 'tilt_enable', 5)
        self.create_subscription(Odometry, 'odom', self.on_odom, 20)

        self.yaw = None
        self.last_odom = None
        self.turning = False
        self.target = 0.0
        self.direction = 0.0
        self.turn_start = None
        self.drive = 0.0
        self.sweep_on = True

        self.create_timer(float(g('control_period')), self.control)

    # ---------- odom ----------
    def on_odom(self, msg: Odometry):
        q = msg.pose.pose.orientation
        siny = 2.0 * (q.w * q.z + q.x * q.y)
        cosy = 1.0 - 2.0 * (q.y * q.y + q.z * q.z)
        self.yaw = math.atan2(siny, cosy)
        self.last_odom = self.get_clock().now()

    def odom_fresh(self) -> bool:
        if not self.use_odom or self.last_odom is None or self.yaw is None:
            return False
        age = (self.get_clock().now() - self.last_odom).nanoseconds * 1e-9
        return age <= self.odom_timeout

    # ---------- putaran ----------
    def start_turn(self, direction: float):
        # guard: klik baru diabaikan selama putaran belum selesai,
        # supaya auto-repeat keyboard tidak menumpuk perintah
        if self.turning and self.ignore_while_turning:
            self.get_logger().info('Masih berputar, klik diabaikan',
                                   throttle_duration_sec=1.0)
            return

        self.direction = direction
        self.turning = True
        self.turn_start = self.get_clock().now()

        if self.odom_fresh():
            self.target = norm_angle(self.yaw + direction * self.step)
            self.mode = 'odom'
        else:
            # cadangan open-loop bila /odom belum ada
            self.mode = 'time'
            self.open_loop_end = self.get_clock().now().nanoseconds * 1e-9 + \
                self.step / max(0.05, self.ang)
            self.get_logger().warn('/odom tidak tersedia -> mode waktu (kurang akurat)')

        arah = 'KIRI (CCW)' if direction > 0 else 'KANAN (CW)'
        self.get_logger().info(f'Putar {math.degrees(self.step):.0f} deg ke {arah}')

    def control(self):
        tw = Twist()

        if self.turning:
            elapsed = (self.get_clock().now() - self.turn_start).nanoseconds * 1e-9
            if elapsed > self.timeout:
                self.get_logger().warn('Putaran timeout, dihentikan')
                self.turning = False
                self.pub.publish(Twist())
                return

            if self.mode == 'odom' and self.odom_fresh():
                err = norm_angle(self.target - self.yaw)
                if abs(err) <= self.tol:
                    self.turning = False
                    self.pub.publish(Twist())
                    self.get_logger().info(
                        f'Selesai, sisa galat {math.degrees(err):+.2f} deg')
                    return
                w = self.kp * err
                w = max(-self.ang, min(self.ang, w))
                if abs(w) < self.min_ang:
                    w = math.copysign(self.min_ang, w)
                tw.angular.z = w
            else:
                now = self.get_clock().now().nanoseconds * 1e-9
                if now >= self.open_loop_end:
                    self.turning = False
                    self.pub.publish(Twist())
                    return
                tw.angular.z = self.direction * self.ang
        else:
            tw.linear.x = self.drive * self.lin

        self.pub.publish(tw)

    # ---------- keyboard ----------
    def on_key(self, key: str) -> bool:
        if key == 'A':          # panah atas
            self.drive = 1.0
        elif key == 'B':        # panah bawah
            self.drive = -1.0
        elif key == 'D':        # panah kiri
            self.drive = 0.0
            self.start_turn(+1.0)
        elif key == 'C':        # panah kanan
            self.drive = 0.0
            self.start_turn(-1.0)
        elif key == ' ':
            self.drive = 0.0
            self.turning = False
            self.pub.publish(Twist())
            self.get_logger().info('STOP')
        elif key in ('r', 'R'):
            self.pub_reset.publish(Empty())
            self.get_logger().info('reset odometri')
        elif key in ('c', 'C') and key == 'c':
            self.pub_clear.publish(Empty())
            self.get_logger().info('cloud dibersihkan')
        elif key in ('v', 'V'):
            self.pub_save.publish(Empty())
            self.get_logger().info('cloud disimpan ke file')
        elif key in ('t', 'T'):
            self.sweep_on = not self.sweep_on
            m = Bool()
            m.data = self.sweep_on
            self.pub_tilt.publish(m)
            self.get_logger().info(f'sweep servo {"ON" if self.sweep_on else "OFF"}')
        elif key in ('+', '='):
            self.lin = min(0.5, self.lin + 0.02)
            self.get_logger().info(f'kecepatan maju {self.lin:.2f} m/s')
        elif key in ('-', '_'):
            self.lin = max(0.02, self.lin - 0.02)
            self.get_logger().info(f'kecepatan maju {self.lin:.2f} m/s')
        elif key in ('q', 'Q', '\x03'):
            return False
        return True


def main():
    if not sys.stdin.isatty():
        print('ERROR: arrow_teleop butuh terminal interaktif.')
        print('Jalankan dengan:  ros2 run room3d arrow_teleop')
        print('JANGAN lewat ros2 launch - stdin tidak diteruskan.')
        return

    rclpy.init()
    node = ArrowTeleop()
    print(__doc__)

    fd = sys.stdin.fileno()
    old = termios.tcgetattr(fd)
    try:
        tty.setraw(fd)
        running = True
        idle = 0
        while running and rclpy.ok():
            rclpy.spin_once(node, timeout_sec=0.01)

            ready, _, _ = select.select([sys.stdin], [], [], 0.0)
            if ready:
                ch = sys.stdin.read(1)
                if ch == '\x1b':                     # awalan tombol panah
                    seq = sys.stdin.read(2) if select.select([sys.stdin], [], [], 0.01)[0] else ''
                    if len(seq) == 2 and seq[0] == '[':
                        running = node.on_key(seq[1])
                else:
                    running = node.on_key(ch)
                idle = 0
            else:
                idle += 1
                # lepas tombol maju/mundur bila tidak ada penekanan lagi
                if idle > 15 and node.drive != 0.0:
                    node.drive = 0.0
    except KeyboardInterrupt:
        pass
    finally:
        termios.tcsetattr(fd, termios.TCSADRAIN, old)
        try:
            node.pub.publish(Twist())
        except Exception:
            pass
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
