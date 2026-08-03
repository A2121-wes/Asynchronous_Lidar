import rclpy
from rclpy.node import Node
import numpy as np
from geometry_msgs.msg import PoseStamped, Point
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


def perturb_unit_vector(u, sigma_angle):
    rand = np.random.randn(3)
    axis = np.cross(u, rand)
    norm = np.linalg.norm(axis)
    if norm < 1e-9:
        return u.copy()
    axis /= norm
    angle = np.random.normal(0, sigma_angle)
    cos_a, sin_a = math.cos(angle), math.sin(angle)
    u_p = cos_a * u + sin_a * np.cross(axis, u) + (1 - cos_a) * np.dot(axis, u) * axis
    n = np.linalg.norm(u_p)
    return u_p / n if n > 1e-9 else u.copy()


# Receiver colors for RViz2 (one per receiver)
COLORS = [
    (0.0, 1.0, 0.0),   # receiver 0 = GREEN
    (0.0, 0.5, 1.0),   # receiver 1 = BLUE
    (1.0, 0.0, 1.0),   # receiver 2 = MAGENTA
    (1.0, 0.5, 0.0),   # receiver 3 = ORANGE
]


class ReceiverNode(Node):

    def __init__(self):
        super().__init__('receiver_node')

        self.declare_parameter('receiver_id',          0)
        self.declare_parameter('start_x',          275.0)
        self.declare_parameter('start_y',            0.0)
        self.declare_parameter('flight_altitude',   50.0)
        self.declare_parameter('flight_azimuth_deg', 45.0)
        self.declare_parameter('flight_speed',        0.0)
        self.declare_parameter('gnss_sigma_xy',      0.02)
        self.declare_parameter('gnss_sigma_z',       0.04)
        self.declare_parameter('imu_sigma_rp_deg',  0.005)
        self.declare_parameter('imu_sigma_h_deg',   0.010)
        self.declare_parameter('publish_rate_hz',   10.0)

        self.rx_id   = self.get_parameter('receiver_id').value
        start_x      = self.get_parameter('start_x').value
        start_y      = self.get_parameter('start_y').value
        self.alt     = self.get_parameter('flight_altitude').value
        self.az_deg  = self.get_parameter('flight_azimuth_deg').value
        self.speed   = self.get_parameter('flight_speed').value
        self.sig_xy  = self.get_parameter('gnss_sigma_xy').value
        self.sig_z   = self.get_parameter('gnss_sigma_z').value
        self.sig_rp  = math.radians(self.get_parameter('imu_sigma_rp_deg').value)
        self.sig_h   = math.radians(self.get_parameter('imu_sigma_h_deg').value)
        rate         = self.get_parameter('publish_rate_hz').value

        az_r = math.radians(self.az_deg)
        self.true_pos   = np.array([start_x, start_y, self.alt])
        self.velocity   = np.array([self.speed * math.cos(az_r),
                                    self.speed * math.sin(az_r), 0.0])
        self.true_roll  = 0.0
        self.true_pitch = 0.0
        self.true_yaw   = az_r

        self._latest_beams = {}

        self.get_logger().info(
            f"Receiver {self.rx_id} ready — "
            f"pos=({start_x},{start_y},{self.alt})m"
        )

        self.obs_pub    = self.create_publisher(Float64MultiArray,
                            f'/receiver_{self.rx_id}/obs',   10)
        self.pose_pub   = self.create_publisher(PoseStamped,
                            f'/receiver_{self.rx_id}/pose',  10)
        self.marker_pub = self.create_publisher(MarkerArray,
                            f'/receiver_{self.rx_id}/markers', 10)

        self.create_subscription(Float64MultiArray, '/transmitter/beams',
                                 self.beams_callback, 10)

        self.t0    = self.get_clock().now().nanoseconds * 1e-9
        self.timer = self.create_timer(1.0 / rate, self.timer_callback)

    def beams_callback(self, msg):
        d = msg.data
        parsed = {}
        for i in range(len(d) // 7):
            b = i * 7
            parsed[int(d[b])] = (
                np.array([d[b+1], d[b+2], d[b+3]]),
                np.array([d[b+4], d[b+5], d[b+6]])
            )
        self._latest_beams = parsed

    def timer_callback(self):
        if not self._latest_beams:
            return

        now_s    = self.get_clock().now().nanoseconds * 1e-9
        dt       = now_s - self.t0
        true_pos = self.true_pos + self.velocity * dt

        noisy_pos = true_pos + np.array([
            np.random.normal(0, self.sig_xy),
            np.random.normal(0, self.sig_xy),
            np.random.normal(0, self.sig_z),
        ])

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

        obs_data = []
        markers  = MarkerArray()

        # Colored sphere = this receiver drone
        col = COLORS[self.rx_id % len(COLORS)]
        drone = Marker()
        drone.header.frame_id = 'map'
        drone.header.stamp    = stamp
        drone.ns     = f'receiver_{self.rx_id}'
        drone.id     = 0
        drone.type   = Marker.SPHERE
        drone.action = Marker.ADD
        drone.pose.position.x = float(noisy_pos[0])
        drone.pose.position.y = float(noisy_pos[1])
        drone.pose.position.z = float(noisy_pos[2])
        drone.pose.orientation.w = 1.0
        drone.scale.x = drone.scale.y = drone.scale.z = 4.0
        drone.color.r = col[0]
        drone.color.g = col[1]
        drone.color.b = col[2]
        drone.color.a = 1.0
        markers.markers.append(drone)

        # Label showing receiver ID
        label = Marker()
        label.header.frame_id = 'map'
        label.header.stamp    = stamp
        label.ns     = f'receiver_{self.rx_id}_label'
        label.id     = 1
        label.type   = Marker.TEXT_VIEW_FACING
        label.action = Marker.ADD
        label.pose.position.x = float(noisy_pos[0])
        label.pose.position.y = float(noisy_pos[1])
        label.pose.position.z = float(noisy_pos[2]) + 8.0
        label.pose.orientation.w = 1.0
        label.scale.z = 6.0
        label.color.r = col[0]
        label.color.g = col[1]
        label.color.b = col[2]
        label.color.a = 1.0
        label.text = f'RX{self.rx_id}'
        markers.markers.append(label)

        sig_att = (2 * self.sig_rp + self.sig_h) / 3

        for beam_id, (A_t, r_t) in self._latest_beams.items():
            if abs(A_t[2]) < 1e-9:
                continue
            d_t = (0.0 - r_t[2]) / A_t[2]
            if d_t <= 0:
                continue
            r_g_true = r_t + A_t * d_t

            r_g_noisy = r_g_true + np.array([
                np.random.normal(0, d_t * math.radians(0.01)),
                np.random.normal(0, d_t * math.radians(0.01)),
                0.0
            ])

            diff = r_g_noisy - noisy_pos
            dist = np.linalg.norm(diff)
            if dist < 1e-6:
                continue
            u_r = diff / dist
            A_r = perturb_unit_vector(u_r, sig_att)

            obs_data.extend([
                float(beam_id),
                float(noisy_pos[0]), float(noisy_pos[1]), float(noisy_pos[2]),
                float(A_r[0]),       float(A_r[1]),       float(A_r[2]),
            ])

            # Thin colored line from receiver to ground point
            line = Marker()
            line.header.frame_id = 'map'
            line.header.stamp    = stamp
            line.ns     = f'rx{self.rx_id}_lines'
            line.id     = beam_id + 100
            line.type   = Marker.LINE_STRIP
            line.action = Marker.ADD
            line.scale.x = 0.15
            p1 = Point()
            p1.x = float(noisy_pos[0])
            p1.y = float(noisy_pos[1])
            p1.z = float(noisy_pos[2])
            p2 = Point()
            p2.x = float(r_g_true[0])
            p2.y = float(r_g_true[1])
            p2.z = 0.0
            line.points = [p1, p2]
            line.color.r = col[0]
            line.color.g = col[1]
            line.color.b = col[2]
            line.color.a = 0.25
            line.lifetime.sec = 1
            markers.markers.append(line)

        self.marker_pub.publish(markers)

        if obs_data:
            obs_msg = Float64MultiArray()
            obs_msg.layout.dim.append(MultiArrayDimension(
                label='observations', size=len(obs_data) // 7, stride=7))
            obs_msg.data = obs_data
            self.obs_pub.publish(obs_msg)


def main(args=None):
    rclpy.init(args=args)
    node = ReceiverNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
