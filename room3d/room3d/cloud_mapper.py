#!/usr/bin/env python3
import math
import os
import struct

import rclpy
from builtin_interfaces.msg import Time as TimeMsg
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
from rclpy.duration import Duration
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy
from rclpy.time import Time
from sensor_msgs.msg import LaserScan, PointCloud2, PointField
from std_msgs.msg import Bool, Empty
from tf2_ros import Buffer, TransformListener


def quat_to_matrix(qx, qy, qz, qw):
    """Kuaternion -> matriks rotasi 3x3 (tuple of tuple)."""
    xx, yy, zz = qx * qx, qy * qy, qz * qz
    xy, xz, yz = qx * qy, qx * qz, qy * qz
    wx, wy, wz = qw * qx, qw * qy, qw * qz
    return (
        (1 - 2 * (yy + zz), 2 * (xy - wz), 2 * (xz + wy)),
        (2 * (xy + wz), 1 - 2 * (xx + zz), 2 * (yz - wx)),
        (2 * (xz - wy), 2 * (yz + wx), 1 - 2 * (xx + yy)),
    )


class CloudMapper(Node):
    def __init__(self):
        super().__init__('cloud_mapper')

        d = self.declare_parameter
        d('fixed_frame', 'odom')
        d('scan_topic', 'scan')
        d('cloud_topic', 'cloud')
        d('publish_period', 0.3)

        # penyaringan jangkauan & ketinggian
        d('range_min', 0.15)
        d('range_max', 4.0)
        d('min_z', -0.30)
        d('max_z', 2.50)

        d('voxel_size', 0.02)
        d('max_points', 800000)
        d('deskew_chunks', 12)

    
        d('gate_on_motion', True)      
        d('lin_thresh', 0.010)         
        d('ang_thresh', 0.030)        
        d('settle_time', 0.6)         
        d('cmd_hold_time', 0.4)
        d('odom_timeout', 1.0)
        d('require_odom', True)
        d('publish_while_moving', False)

        d('save_path', '~/room3d_scan.pcd')

        g = lambda n: self.get_parameter(n).value
        self.fixed_frame = g('fixed_frame')
        self.period = float(g('publish_period'))
        self.rmin = float(g('range_min'))
        self.rmax = float(g('range_max'))
        self.min_z = float(g('min_z'))
        self.max_z = float(g('max_z'))
        self.voxel = max(0.001, float(g('voxel_size')))
        self.max_points = int(g('max_points'))
        self.chunks = max(1, int(g('deskew_chunks')))
        self.gate = bool(g('gate_on_motion'))
        self.lin_thresh = float(g('lin_thresh'))
        self.ang_thresh = float(g('ang_thresh'))
        self.settle = float(g('settle_time'))
        self.cmd_hold = float(g('cmd_hold_time'))
        self.odom_timeout = float(g('odom_timeout'))
        self.require_odom = bool(g('require_odom'))
        self.pub_while_moving = bool(g('publish_while_moving'))
        self.save_path = os.path.expanduser(str(g('save_path')))

        
        self.voxels = {} 
        self.dirty = False
        self.capturing = False
        self.moving = True 
        self.stopped_since = None
        self.last_odom = None
        self.last_cmd_time = None
        self.dropped = 0
        self.kept = 0

        self.tf_buffer = Buffer(cache_time=Duration(seconds=10.0))
        self.tf_listener = TransformListener(self.tf_buffer, self)

        sensor_qos = QoSProfile(depth=5)
        sensor_qos.reliability = ReliabilityPolicy.BEST_EFFORT

        self.create_subscription(LaserScan, g('scan_topic'), self.on_scan, sensor_qos)
        self.create_subscription(Odometry, 'odom', self.on_odom, 20)
        self.create_subscription(Twist, 'cmd_vel', self.on_cmd, 10)
        self.create_subscription(Empty, 'clear_cloud', self.on_clear, 10)
        self.create_subscription(Empty, 'save_cloud', self.on_save, 10)

        self.pub_cloud = self.create_publisher(PointCloud2, g('cloud_topic'), 1)
        self.pub_capturing = self.create_publisher(Bool, 'capturing', 5)

        self.create_timer(self.period, self.publish_cloud)
        self.create_timer(0.1, self.update_gate)
        self.create_timer(3.0, self.report)

        self.get_logger().info(
            f'cloud_mapper siap | frame={self.fixed_frame} voxel={self.voxel} m '
            f'gating={"AKTIF" if self.gate else "nonaktif"}'
        )

    def on_odom(self, msg: Odometry):
        self.last_odom = self.get_clock().now()
        v = abs(msg.twist.twist.linear.x)
        w = abs(msg.twist.twist.angular.z)
        self.odom_moving = (v > self.lin_thresh) or (w > self.ang_thresh)

    def on_cmd(self, msg: Twist):
        if abs(msg.linear.x) > 1e-4 or abs(msg.angular.z) > 1e-4:
            self.last_cmd_time = self.get_clock().now()

    def update_gate(self):
        """Tentukan capturing = robot benar-benar diam dan sudah tenang."""
        if not self.gate:
            self.capturing = True
            return

        now = self.get_clock().now()
        moving = False
        reason = ''


        if self.last_odom is None:
            if self.require_odom:
                moving = True
                reason = 'menunggu /odom'
        else:
            age = (now - self.last_odom).nanoseconds * 1e-9
            if age > self.odom_timeout:
                if self.require_odom:
                    moving = True
                    reason = '/odom kedaluwarsa'
            elif getattr(self, 'odom_moving', False):
                moving = True
                reason = 'roda berputar'


        if self.last_cmd_time is not None:
            since = (now - self.last_cmd_time).nanoseconds * 1e-9
            if since < self.cmd_hold:
                moving = True
                reason = reason or 'cmd_vel aktif'

        if moving:
            if self.capturing:
                self.get_logger().info(f'BERGERAK ({reason}) -> perekaman DIJEDA')
            self.capturing = False
            self.stopped_since = None
        else:
            if self.stopped_since is None:
                self.stopped_since = now
            quiet = (now - self.stopped_since).nanoseconds * 1e-9
            if quiet >= self.settle and not self.capturing:
                self.capturing = True
                self.get_logger().info(f'DIAM {quiet:.1f}s -> perekaman LANJUT')

        self.moving = moving
        msg = Bool()
        msg.data = self.capturing
        self.pub_capturing.publish(msg)

    # ================= scan =================
    def on_scan(self, scan: LaserScan):
        if not self.capturing:
            self.dropped += 1
            return

        n = len(scan.ranges)
        if n == 0:
            return

        per_chunk = max(1, n // self.chunks)
        scan_time = scan.scan_time if scan.scan_time > 0.0 else (n * scan.time_increment)
        base_time = Time.from_msg(scan.header.stamp)
        has_int = len(scan.intensities) == n

        added = 0
        for c0 in range(0, n, per_chunk):
            c1 = min(n, c0 + per_chunk)
            frac = (c0 + 0.5 * (c1 - c0)) / float(n)
            t_chunk = base_time + Duration(seconds=frac * scan_time)

            tf = self.lookup(scan.header.frame_id, t_chunk)
            if tf is None:
                continue
            (r0, r1, r2), (tx, ty, tz) = tf

            for i in range(c0, c1):
                rng = scan.ranges[i]
                if not (self.rmin < rng < self.rmax) or math.isinf(rng) or math.isnan(rng):
                    continue
                a = scan.angle_min + i * scan.angle_increment
                lx = rng * math.cos(a)
                ly = rng * math.sin(a)

                x = r0[0] * lx + r0[1] * ly + tx
                y = r1[0] * lx + r1[1] * ly + ty
                z = r2[0] * lx + r2[1] * ly + tz

                if not (self.min_z <= z <= self.max_z):
                    continue

                key = (int(x / self.voxel), int(y / self.voxel), int(z / self.voxel))
                if key in self.voxels:
                    continue
                if len(self.voxels) >= self.max_points:
                    continue
                inten = float(scan.intensities[i]) if has_int else 0.0
                self.voxels[key] = (x, y, z, inten)
                added += 1

        if added:
            self.kept += added
            self.dirty = True

    def lookup(self, source_frame, stamp):
        """TF fixed_frame <- source_frame pada waktu stamp, dengan fallback."""
        for target_time in (stamp, Time()):
            try:
                tr = self.tf_buffer.lookup_transform(
                    self.fixed_frame, source_frame, target_time,
                    timeout=Duration(seconds=0.05))
            except Exception:
                continue
            t = tr.transform.translation
            q = tr.transform.rotation
            return quat_to_matrix(q.x, q.y, q.z, q.w), (t.x, t.y, t.z)
        return None


    def publish_cloud(self):
        if not self.dirty or not self.voxels:
            return
        if self.gate and not self.capturing and not self.pub_while_moving:
            # robot bergerak: JANGAN unggah titik ke RViz
            return

        data = bytearray()
        for (x, y, z, inten) in self.voxels.values():
            data += struct.pack('<ffff', x, y, z, inten)

        msg = PointCloud2()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = self.fixed_frame
        msg.height = 1
        msg.width = len(self.voxels)
        msg.fields = [
            PointField(name='x', offset=0, datatype=PointField.FLOAT32, count=1),
            PointField(name='y', offset=4, datatype=PointField.FLOAT32, count=1),
            PointField(name='z', offset=8, datatype=PointField.FLOAT32, count=1),
            PointField(name='intensity', offset=12, datatype=PointField.FLOAT32, count=1),
        ]
        msg.is_bigendian = False
        msg.point_step = 16
        msg.row_step = msg.point_step * msg.width
        msg.is_dense = True
        msg.data = bytes(data)
        self.pub_cloud.publish(msg)
        self.dirty = False

    def on_clear(self, _msg: Empty):
        self.voxels.clear()
        self.kept = 0
        self.dirty = True
        self.get_logger().info('Cloud dibersihkan')
        self.publish_cloud()

    def on_save(self, _msg: Empty):
        try:
            with open(self.save_path, 'w') as f:
                n = len(self.voxels)
                f.write('# .PCD v0.7 - Point Cloud Data file format\n')
                f.write('VERSION 0.7\n')
                f.write('FIELDS x y z intensity\n')
                f.write('SIZE 4 4 4 4\n')
                f.write('TYPE F F F F\n')
                f.write('COUNT 1 1 1 1\n')
                f.write(f'WIDTH {n}\nHEIGHT 1\n')
                f.write('VIEWPOINT 0 0 0 1 0 0 0\n')
                f.write(f'POINTS {n}\nDATA ascii\n')
                for (x, y, z, inten) in self.voxels.values():
                    f.write(f'{x:.4f} {y:.4f} {z:.4f} {inten:.1f}\n')
            self.get_logger().info(f'Cloud disimpan: {self.save_path} ({len(self.voxels)} titik)')
        except Exception as exc:
            self.get_logger().error(f'Gagal menyimpan: {exc}')

    def report(self):
        self.get_logger().info(
            f'{"MEREKAM " if self.capturing else "DIJEDA  "} '
            f'titik={len(self.voxels)} | scan dibuang saat bergerak={self.dropped}'
        )


def main():
    rclpy.init()
    node = CloudMapper()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()