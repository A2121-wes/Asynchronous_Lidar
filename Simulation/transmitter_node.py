import rclpy
from rclpy.node import Node
import numpy as np
from geometry_msgs.msg import PoseStamped
from std_msgs.msg import Float64MultiArray, MultiArrayDimension
from visualization_msgs.msg import Marker, MarkerArray
import math


def rotation_matrix_from_euler(roll, pitch, yaw):
    cx, sx = math.cos(roll),  math.sin(roll)
    cy, sy = math.cos(pitch), math.sin(pitch)
    cz, sz = math.cos(yaw),   math.sin(yaw)
    return np.array([
        [cy*cz,  cy*sz, -sy],
        [sx*sy*cz - cx*sz, sx*sy*sz + cx*cz,  sx*cy],
        [cx*sy*cz + sx*sz, cx*sy*sz - sx*cz,  cx*cy]
    ])


def make_beam_unit_vectors(scan_angles_x_deg, scan_angles_y_deg):
    vectors = []
    for tx in scan_angles_x_deg:
        for ty in scan_angles_y_deg:
            tx_r = math.radians(tx)
            ty_r = math.radians(ty)
            ux =  math.sin(tx_r)
            uy =  math.sin(ty_r) * math.cos(tx_r)
            uz = -math.cos(tx_r) * math.cos(ty_r)
            norm = math.sqrt(ux**2 + uy**2 + uz**2)
            vectors.append([ux/norm, uy/norm, uz/norm])
    return np.array(vectors)


class TransmitterNode(Node):

    def __init__(self):
        super().__init__('transmitter_node')

        self.declare_parameter('flight_altitude',    100.0)
        self.declare_parameter('flight_azimuth_deg',  45.0)
        self.declare_parameter('flight_speed',          0.0)
        self.declare_parameter('gnss_sigma_xy',        0.02)
        self.declare_parameter('gnss_sigma_z',         0.04)
        self.declare_parameter('imu_sigma_rp_deg',    0.005)
        self.declare_parameter('imu_sigma_h_deg',     0.010)
        self.declare_parameter('scan_sigma_deg',     0.0086)
        self.declare_parameter('publish_rate_hz',      10.0)

        self.alt       = self.get_parameter('flight_altitude').value
        self.az_deg    = self.get_parameter('flight_azimuth_deg').value
        self.speed     = self.get_parameter('flight_speed').value
        self.sig_xy    = self.get_parameter('gnss_sigma_xy').value
        self.sig_z     = self.get_parameter('gnss_sigma_z').value
        self.sig_rp    = math.radians(self.get_parameter('imu_sigma_rp_deg').value)
        self.sig_h     = math.radians(self.get_parameter('imu_sigma_h_deg').value)
        self.sig_sc    = math.radians(self.get_parameter('scan_sigma_deg').value)
        rate           = self.get_parameter('publish_rate_hz').value

        az_r = math.radians(self.az_deg)
        self.true_pos   = np.array([0.0, 0.0, self.alt])
        self.velocity   = np.array([self.speed * math.cos(az_r),
                                    self.speed * math.sin(az_r), 0.0])
        self.true_roll  = 0.0
        self.true_pitch = 0.0
        self.true_yaw   = az_r

        # 10x10 grid (-10 to +10 degrees, 10 steps each) + nadir center = 101 beams
        angles = np.linspace(-10, 10, 10).tolist()
        self.beam_vectors_body = make_beam_unit_vectors(angles, angles)
        nadir = np.array([[0.0, 0.0, -1.0]])
        self.beam_vectors_body = np.vstack([self.beam_vectors_body, nadir])

        self.get_logger().info(
            f"Transmitter ready — altitude={self.alt}m  beams={len(self.beam_vectors_body)}"
        )

        self.pose_pub    = self.create_publisher(PoseStamped,      '/transmitter/pose',    10)
        self.beams_pub   = self.create_publisher(Float64MultiArray, '/transmitter/beams',   10)
        self.marker_pub  = self.create_publisher(MarkerArray,       '/transmitter/markers', 10)

        self.t0    = self.get_clock().now().nanoseconds * 1e-9
        self.timer = self.create_timer(1.0 / rate, self.timer_callback)

    def timer_callback(self):
        now_s    = self.get_clock().now().nanoseconds * 1e-9
        dt       = now_s - self.t0
        true_pos = self.true_pos + self.velocity * dt

        noisy_pos = true_pos + np.array([
            np.random.normal(0, self.sig_xy),
            np.random.normal(0, self.sig_xy),
            np.random.normal(0, self.sig_z),
        ])

        noisy_roll  = self.true_roll  + np.random.normal(0, self.sig_rp)
        noisy_pitch = self.true_pitch + np.random.normal(0, self.sig_rp)
        noisy_yaw   = self.true_yaw   + np.random.normal(0, self.sig_h)

        R   = rotation_matrix_from_euler(noisy_roll, noisy_pitch, noisy_yaw)
        A_t = (R @ self.beam_vectors_body.T).T
        A_t[:, 2] = -np.abs(A_t[:, 2])

        noise     = np.random.normal(0, self.sig_sc, A_t.shape)
        A_t_noisy = A_t + noise
        norms     = np.linalg.norm(A_t_noisy, axis=1, keepdims=True)
        A_t_noisy = A_t_noisy / np.maximum(norms, 1e-9)

        stamp = self.get_clock().now().to_msg()

        # --- Pose ---
        pose_msg = PoseStamped()
        pose_msg.header.stamp    = stamp
        pose_msg.header.frame_id = 'map'
        pose_msg.pose.position.x = float(noisy_pos[0])
        pose_msg.pose.position.y = float(noisy_pos[1])
        pose_msg.pose.position.z = float(noisy_pos[2])
        pose_msg.pose.orientation.w = 1.0
        self.pose_pub.publish(pose_msg)

        # --- Beams data ---
        data = []
        for i, a in enumerate(A_t_noisy):
            data.extend([float(i),
                         float(a[0]), float(a[1]), float(a[2]),
                         float(noisy_pos[0]), float(noisy_pos[1]), float(noisy_pos[2])])

        beam_msg = Float64MultiArray()
        beam_msg.layout.dim.append(MultiArrayDimension(
            label='beams', size=len(A_t_noisy), stride=7))
        beam_msg.data = data
        self.beams_pub.publish(beam_msg)

        # --- VISUAL MARKERS ---
        markers = MarkerArray()

        # 1. Big RED sphere = transmitter drone position
        drone = Marker()
        drone.header.frame_id = 'map'
        drone.header.stamp    = stamp
        drone.ns     = 'transmitter'
        drone.id     = 0
        drone.type   = Marker.SPHERE
        drone.action = Marker.ADD
        drone.pose.position.x = float(noisy_pos[0])
        drone.pose.position.y = float(noisy_pos[1])
        drone.pose.position.z = float(noisy_pos[2])
        drone.pose.orientation.w = 1.0
        drone.scale.x = drone.scale.y = drone.scale.z = 5.0
        drone.color.r = 1.0
        drone.color.g = 0.2
        drone.color.b = 0.0
        drone.color.a = 1.0
        markers.markers.append(drone)

        # 2. YELLOW lines = laser beams from drone to ground
        for i, a in enumerate(A_t_noisy):
            if abs(a[2]) < 1e-9:
                continue
            d_t = (0.0 - noisy_pos[2]) / a[2]
            if d_t <= 0:
                continue
            gx = noisy_pos[0] + a[0] * d_t
            gy = noisy_pos[1] + a[1] * d_t
            gz = 0.0

            line = Marker()
            line.header.frame_id = 'map'
            line.header.stamp    = stamp
            line.ns     = 'beams'
            line.id     = i + 1
            line.type   = Marker.LINE_STRIP
            line.action = Marker.ADD
            line.scale.x = 0.3

            from geometry_msgs.msg import Point
            p1 = Point()
            p1.x = float(noisy_pos[0])
            p1.y = float(noisy_pos[1])
            p1.z = float(noisy_pos[2])
            p2 = Point()
            p2.x = float(gx)
            p2.y = float(gy)
            p2.z = float(gz)
            line.points = [p1, p2]
            line.color.r = 1.0
            line.color.g = 1.0
            line.color.b = 0.0
            line.color.a = 0.4
            line.lifetime.sec = 1
            markers.markers.append(line)

        self.marker_pub.publish(markers)


def main(args=None):
    rclpy.init(args=args)
    node = TransmitterNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
