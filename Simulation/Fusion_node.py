import rclpy
from rclpy.node import Node
import numpy as np
from sensor_msgs.msg import PointCloud2
import sensor_msgs_py.point_cloud2 as pc2
from std_msgs.msg import Float64MultiArray, MultiArrayDimension, Header
from visualization_msgs.msg import Marker, MarkerArray
from geometry_msgs.msg import Point
import math

from async_lidar.async_lidar_math import solve_ground_point, compute_dop


class FusionNode(Node):

    def __init__(self):
        super().__init__('fusion_node')

        self.declare_parameter('n_receivers',          3)
        self.declare_parameter('ground_z_approx',    0.0)
        self.declare_parameter('max_pdop_threshold', 5.0)
        self.declare_parameter('publish_rate_hz',    5.0)

        self.n_rx        = self.get_parameter('n_receivers').value
        self.ground_z    = self.get_parameter('ground_z_approx').value
        self.pdop_thresh = self.get_parameter('max_pdop_threshold').value
        rate             = self.get_parameter('publish_rate_hz').value

        self._tx_beams = {}
        self._rx_obs   = {i: {} for i in range(self.n_rx)}

        self.get_logger().info(
            f"Fusion node ready — {self.n_rx} receivers, PDOP threshold={self.pdop_thresh}"
        )

        self.pts_pub    = self.create_publisher(PointCloud2,       '/ground_points',       10)
        self.dop_pub    = self.create_publisher(Float64MultiArray,  '/dop_values',          10)
        self.unc_pub    = self.create_publisher(Float64MultiArray,  '/point_uncertainty',   10)
        self.marker_pub = self.create_publisher(MarkerArray,        '/fusion/markers',      10)

        self.create_subscription(Float64MultiArray, '/transmitter/beams',
                                 self._tx_callback, 10)

        for rx_id in range(self.n_rx):
            self.create_subscription(Float64MultiArray,
                                     f'/receiver_{rx_id}/obs',
                                     self._make_rx_callback(rx_id), 10)
            self.get_logger().info(f"  Subscribed to /receiver_{rx_id}/obs")

        self.timer = self.create_timer(1.0 / rate, self.timer_callback)

    def _make_rx_callback(self, rx_id):
        def callback(msg):
            d = msg.data
            parsed = {}
            for i in range(len(d) // 7):
                b = i * 7
                parsed[int(d[b])] = (
                    np.array([d[b+1], d[b+2], d[b+3]]),
                    np.array([d[b+4], d[b+5], d[b+6]])
                )
            self._rx_obs[rx_id] = parsed
        return callback

    def _tx_callback(self, msg):
        d = msg.data
        parsed = {}
        for i in range(len(d) // 7):
            b = i * 7
            parsed[int(d[b])] = (
                np.array([d[b+1], d[b+2], d[b+3]]),
                np.array([d[b+4], d[b+5], d[b+6]])
            )
        self._tx_beams = parsed

    def timer_callback(self):
        if not self._tx_beams:
            return

        active_rxs = [i for i in range(self.n_rx) if self._rx_obs.get(i)]
        if len(active_rxs) < 2:
            return

        solved_points = []
        dop_records   = []
        unc_records   = []
        markers       = MarkerArray()
        stamp         = self.get_clock().now().to_msg()

        # Clear old ground point markers
        clear = Marker()
        clear.header.frame_id = 'map'
        clear.header.stamp    = stamp
        clear.action = Marker.DELETEALL
        markers.markers.append(clear)

        for beam_id, (A_t, r_t) in self._tx_beams.items():
            receivers_for_beam = []
            for rx_id in active_rxs:
                if beam_id in self._rx_obs[rx_id]:
                    receivers_for_beam.append(self._rx_obs[rx_id][beam_id])

            if len(receivers_for_beam) < 2:
                continue

            r_g, cov_diag, converged = solve_ground_point(
                r_t, A_t, receivers_for_beam, ground_z_approx=self.ground_z)

            if not converged or r_g is None or cov_diag is None:
                continue

            hdop, vdop, pdop = compute_dop(cov_diag)
            if pdop > self.pdop_thresh or not np.isfinite(pdop):
                continue

            sigma_total = math.sqrt(sum(max(c, 0) for c in cov_diag))
            solved_points.append([float(r_g[0]), float(r_g[1]),
                                   float(r_g[2]), float(sigma_total)])
            dop_records.append([float(beam_id), hdop, vdop, pdop])
            unc_records.append([float(beam_id),
                                 math.sqrt(max(cov_diag[0], 0)),
                                 math.sqrt(max(cov_diag[1], 0)),
                                 math.sqrt(max(cov_diag[2], 0))])

            # WHITE sphere at each solved ground point
            pt_marker = Marker()
            pt_marker.header.frame_id = 'map'
            pt_marker.header.stamp    = stamp
            pt_marker.ns     = 'ground_points'
            pt_marker.id     = int(beam_id) + 200
            pt_marker.type   = Marker.SPHERE
            pt_marker.action = Marker.ADD
            pt_marker.pose.position.x = float(r_g[0])
            pt_marker.pose.position.y = float(r_g[1])
            pt_marker.pose.position.z = float(r_g[2])
            pt_marker.pose.orientation.w = 1.0
            pt_marker.scale.x = pt_marker.scale.y = pt_marker.scale.z = 1.5

            # Color by PDOP: green=good, red=bad
            t = min(pdop / self.pdop_thresh, 1.0)
            pt_marker.color.r = t
            pt_marker.color.g = 1.0 - t
            pt_marker.color.b = 0.0
            pt_marker.color.a = 1.0
            pt_marker.lifetime.sec = 2
            markers.markers.append(pt_marker)

        self.marker_pub.publish(markers)

        if not solved_points:
            return

        # PointCloud2
        header = Header()
        header.stamp    = self.get_clock().now().to_msg()
        header.frame_id = 'map'
        fields = [
            pc2.PointField(name='x',         offset=0,  datatype=pc2.PointField.FLOAT32, count=1),
            pc2.PointField(name='y',         offset=4,  datatype=pc2.PointField.FLOAT32, count=1),
            pc2.PointField(name='z',         offset=8,  datatype=pc2.PointField.FLOAT32, count=1),
            pc2.PointField(name='intensity', offset=12, datatype=pc2.PointField.FLOAT32, count=1),
        ]
        self.pts_pub.publish(pc2.create_cloud(header, fields, solved_points))

        # DOP
        dop_msg = Float64MultiArray()
        dop_msg.layout.dim.append(MultiArrayDimension(
            label='dop', size=len(dop_records), stride=4))
        dop_msg.data = [v for r in dop_records for v in r]
        self.dop_pub.publish(dop_msg)

        # Uncertainty
        unc_msg = Float64MultiArray()
        unc_msg.layout.dim.append(MultiArrayDimension(
            label='uncertainty', size=len(unc_records), stride=4))
        unc_msg.data = [v for r in unc_records for v in r]
        self.unc_pub.publish(unc_msg)

        n = len(solved_points)
        mean_pdop = float(np.mean([r[3] for r in dop_records]))
        self.get_logger().info(
            f"Published {n} ground points | mean PDOP={mean_pdop:.3f}",
            throttle_duration_sec=2.0)


def main(args=None):
    rclpy.init(args=args)
    node = FusionNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
