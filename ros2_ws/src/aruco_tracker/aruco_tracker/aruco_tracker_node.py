import cv2
import numpy as np
import rclpy
from cv_bridge import CvBridge
from geometry_msgs.msg import Twist
from rclpy.node import Node
from rclpy.qos import HistoryPolicy, QoSProfile, ReliabilityPolicy
from sensor_msgs.msg import CameraInfo, Image
from std_msgs.msg import Float32

WINDOW_NAME = 'ArUco tracker'

# (family, size) -> ('5x5', 1000) -> cv2.aruco.DICT_5X5_1000
ARUCO_DICTS = {
    (family, size): getattr(cv2.aruco, f'DICT_{family.upper()}_{size}')
    for family in ('4x4', '5x5', '6x6', '7x7')
    for size in (50, 100, 250, 1000)
}


def make_detector(dict_id):
    """Return detect(gray) -> (corners, ids)."""
    new_api = hasattr(cv2.aruco, 'ArucoDetector')
    if new_api:
        params = cv2.aruco.DetectorParameters()
        dictionary = cv2.aruco.getPredefinedDictionary(dict_id)
    else:
        params = cv2.aruco.DetectorParameters_create()
        dictionary = cv2.aruco.Dictionary_get(dict_id)

    params.cornerRefinementMethod = cv2.aruco.CORNER_REFINE_SUBPIX
    params.minMarkerPerimeterRate = 0.02

    if new_api:
        detector = cv2.aruco.ArucoDetector(dictionary, params)

        def detect(gray):
            corners, ids, _ = detector.detectMarkers(gray)
            return corners, ids
    else:
        def detect(gray):
            corners, ids, _ = cv2.aruco.detectMarkers(gray, dictionary, parameters=params)
            return corners, ids

    return detect


def put_text(img, text, org, color=(0, 255, 0), scale=0.7):
    cv2.putText(img, text, org, cv2.FONT_HERSHEY_SIMPLEX, scale, (0, 0, 0), 4, cv2.LINE_AA)
    cv2.putText(img, text, org, cv2.FONT_HERSHEY_SIMPLEX, scale, color, 2, cv2.LINE_AA)


class Pid:
    """PID with deadband, output clamp and integral anti-windup."""

    def __init__(self, kp, ki, kd, limit, deadband):
        self.kp, self.ki, self.kd = kp, ki, kd
        self.limit, self.deadband = limit, deadband
        self.reset()

    def reset(self):
        self.integral = 0.0
        self.prev_error = None

    def step(self, error, dt):
        if abs(error) < self.deadband:
            error = 0.0
        derivative = 0.0
        if self.prev_error is not None and dt > 0.0:
            derivative = (error - self.prev_error) / dt
        self.prev_error = error
        unsaturated = self.kp * error + self.ki * self.integral + self.kd * derivative
        output = max(-self.limit, min(self.limit, unsaturated))
        # Only integrate while not saturated (anti-windup).
        if self.ki and output == unsaturated and dt > 0.0:
            self.integral += error * dt
        return output


class MarkerTrack:
    """Temporal filter for one marker id: confirmation, outlier gating and smoothing."""

    def __init__(self, marker_id, now, tvec, rvec):
        self.id = marker_id
        self.first_seen = now
        self.last_seen = now
        self.hits = 1            # consecutive frames seen
        self.rejects = 0         # consecutive frames rejected as outliers
        self.tvec = tvec
        self.rvec = rvec
        self.confirmed = False

    def update(self, now, tvec, rvec, alpha, max_jump, min_hits, reject_reset):
        """Fold in a new measurement. Returns False if it was rejected as an outlier."""
        if np.linalg.norm(tvec - self.tvec) > max_jump:
            self.rejects += 1
            if self.rejects >= reject_reset:
                # The marker really moved (or the old track was wrong): start over.
                self.__init__(self.id, now, tvec, rvec)
            return False
        self.rejects = 0
        self.last_seen = now
        self.hits += 1
        self.tvec = alpha * tvec + (1.0 - alpha) * self.tvec
        self.rvec = rvec
        if self.hits >= min_hits:
            self.confirmed = True
        return True


class ArucoTrackerNode(Node):

    def __init__(self):
        super().__init__('aruco_tracker_node')

        self.declare_parameter('dictionary_family', '5x5')
        self.declare_parameter('dictionary', 1000)
        self.declare_parameter('marker_size', 0.1) # meters
        self.declare_parameter('image_topic', '/camera/camera/color/image_raw')
        self.declare_parameter('camera_info_topic', '/camera/camera/color/camera_info')
        self.declare_parameter('show_window', True)
        self.declare_parameter('qos_reliability', 'reliable')  # or 'best_effort'

        # Regulator: errors are marker centre minus image centre, in metres at the marker's depth.
        self.declare_parameter('cmd_vel_topic', '/cmd_vel')
        self.declare_parameter('kp', 0.8)            # (m/s) per metre of error
        self.declare_parameter('ki', 0.0)
        self.declare_parameter('kd', 0.1)
        self.declare_parameter('max_velocity', 0.3)  # m/s clamp on linear.y / linear.z
        self.declare_parameter('deadband_m', 0.02)   # no command inside this error

        # Sign convention of the output: Drone FRD (Front, Right, Down), so a marker
        # to the right of the crosshair gives positive y.
        self.declare_parameter('invert_y', True)
        self.declare_parameter('invert_z', True)
        # Temporal filtering
        self.declare_parameter('min_consecutive_frames', 5)  # frames in a row before a marker counts
        self.declare_parameter('lost_timeout_sec', 0.5)      # confirmed marker dropped after this
        self.declare_parameter('smoothing_alpha', 0.4)       # 1.0 = no smoothing, lower = smoother
        self.declare_parameter('max_jump_m', 0.30)           # per-frame position jump treated as outlier

        family = self.get_parameter('dictionary_family').value
        size = int(self.get_parameter('dictionary').value)
        if (family, size) not in ARUCO_DICTS:
            self.get_logger().warn(f'Unknown dictionary {family}_{size}, falling back to 5x5_1000')
            family, size = '5x5', 1000
        self.marker_size = float(self.get_parameter('marker_size').value)
        self.window_ok = bool(self.get_parameter('show_window').value)
        self.get_logger().info(f'Using DICT_{family.upper()}_{size}, marker size {self.marker_size} m')

        self.detect = make_detector(ARUCO_DICTS[(family, size)])
        # Marker corners in the marker frame (OpenCV order: TL, TR, BR, BL), Z=0 plane.
        h = self.marker_size / 2.0
        self.object_points = np.array(
            [[-h, h, 0], [h, h, 0], [h, -h, 0], [-h, -h, 0]], dtype=np.float32)

        self.bridge = CvBridge()
        self.camera_matrix = None
        self.dist_coeffs = None
        self.min_hits = int(self.get_parameter('min_consecutive_frames').value)
        self.lost_timeout = float(self.get_parameter('lost_timeout_sec').value)
        self.alpha = float(self.get_parameter('smoothing_alpha').value)
        self.max_jump = float(self.get_parameter('max_jump_m').value)
        gains = [float(self.get_parameter(n).value) for n in ('kp', 'ki', 'kd')]
        limit = float(self.get_parameter('max_velocity').value)
        deadband = float(self.get_parameter('deadband_m').value)
        self.pid_y = Pid(*gains, limit, deadband)
        self.pid_z = Pid(*gains, limit, deadband)
        self.sign_y = 1.0 if self.get_parameter('invert_y').value else -1.0
        self.sign_z = 1.0 if self.get_parameter('invert_z').value else -1.0
        self.last_stamp = None
        self.tracks = {}
        self.primary_id = None

        # Use 'best_effort' for sensor-data publishers.
        reliability = (ReliabilityPolicy.BEST_EFFORT
                       if self.get_parameter('qos_reliability').value == 'best_effort'
                       else ReliabilityPolicy.RELIABLE)
        sensor_qos = QoSProfile(reliability=reliability,
                                history=HistoryPolicy.KEEP_LAST, depth=1)
        self.create_subscription(
            CameraInfo, self.get_parameter('camera_info_topic').value,
            self._camera_info_callback, sensor_qos)
        self.create_subscription(
            Image, self.get_parameter('image_topic').value, self._image_callback, sensor_qos)

        self.image_pub = self.create_publisher(Image, '~/image', 5)
        self.distance_pub = self.create_publisher(Float32, '~/distance', 10)
        self.cmd_vel_pub = self.create_publisher(
            Twist, self.get_parameter('cmd_vel_topic').value, 10)

    def _camera_info_callback(self, msg: CameraInfo):
        if self.camera_matrix is not None:
            return
        self.camera_matrix = np.array(msg.k, dtype=np.float64).reshape(3, 3)
        self.dist_coeffs = np.array(msg.d, dtype=np.float64) if len(msg.d) else np.zeros(5)
        self.get_logger().info('Camera info received')

    def _image_callback(self, msg: Image):
        if self.camera_matrix is None:
            self.get_logger().warn('Waiting for camera_info...', throttle_duration_sec=2.0)
            return

        img = self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8').copy()
        corners, ids = self.detect(cv2.cvtColor(img, cv2.COLOR_BGR2GRAY))

        now = self.get_clock().now().nanoseconds / 1e9
        seen = set()
        if ids is not None and len(ids) > 0:
            for marker_corners, marker_id in zip(corners, ids.flatten()):
                marker_id = int(marker_id)
                ok, rvec, tvec = cv2.solvePnP(
                    self.object_points, marker_corners.reshape(4, 2).astype(np.float32),
                    self.camera_matrix, self.dist_coeffs, flags=cv2.SOLVEPNP_IPPE_SQUARE)
                if not ok:
                    continue
                seen.add(marker_id)
                track = self.tracks.get(marker_id)
                if track is None:
                    self.tracks[marker_id] = MarkerTrack(marker_id, now, tvec, rvec)
                elif not track.update(now, tvec, rvec, self.alpha, self.max_jump,
                                      self.min_hits, reject_reset=self.min_hits):
                    seen.discard(marker_id)
                    if marker_id in self.tracks:  # not reset: grey out until it settles
                        self._draw_pending(img, marker_corners)
                    continue
                track = self.tracks[marker_id]
                if track.confirmed:
                    cv2.aruco.drawDetectedMarkers(img, [marker_corners], np.array([[marker_id]]))
                    cv2.drawFrameAxes(img, self.camera_matrix, self.dist_coeffs,
                                      track.rvec, track.tvec, self.marker_size * 0.5)
                    top_left = marker_corners.reshape(4, 2)[0].astype(int)
                    put_text(img, f'id {marker_id}: {float(np.linalg.norm(track.tvec)):.2f} m',
                             (int(top_left[0]), max(int(top_left[1]) - 10, 20)))
                else:
                    self._draw_pending(img, marker_corners)

        # Unconfirmed tracks must be seen in consecutive frames; confirmed ones may
        # miss frames until lost_timeout_sec has passed.
        for marker_id in list(self.tracks):
            track = self.tracks[marker_id]
            if marker_id in seen:
                continue
            if not track.confirmed or now - track.last_seen > self.lost_timeout:
                del self.tracks[marker_id]

        height, width = img.shape[:2]
        center = (width // 2, height // 2)
        self._draw_crosshair(img, center)

        stamp = msg.header.stamp.sec + msg.header.stamp.nanosec * 1e-9
        dt = 0.0 if self.last_stamp is None else max(stamp - self.last_stamp, 0.0)
        self.last_stamp = stamp

        cmd = Twist()  # stays zero when no marker is tracked
        primary = self._select_primary()
        if primary is not None:
            x, y, z = primary.tvec.flatten()
            # Marker centre in pixels, then its offset from the image centre in metres at depth z.
            u = self.camera_matrix[0, 0] * x / z + self.camera_matrix[0, 2]
            v = self.camera_matrix[1, 1] * y / z + self.camera_matrix[1, 2]
            err_y = (u - center[0]) * z / self.camera_matrix[0, 0]  # +right
            err_z = (v - center[1]) * z / self.camera_matrix[1, 1]  # +down
            cmd.linear.y = self.sign_y * self.pid_y.step(err_y, dt)
            cmd.linear.z = self.sign_z * self.pid_z.step(err_z, dt)

            distance = float(np.linalg.norm(primary.tvec))
            self.distance_pub.publish(Float32(data=distance))
            cv2.line(img, center, (int(u), int(v)), (0, 255, 255), 2, cv2.LINE_AA)
            put_text(img, f'Tracking id {primary.id}: {distance:.2f} m', (10, 30))
            put_text(img, f'err y={err_y:+.2f} z={err_z:+.2f} m', (10, 60))
            put_text(img, f'cmd y={cmd.linear.y:+.2f} z={cmd.linear.z:+.2f} m/s', (10, 90))
        else:
            self.pid_y.reset()
            self.pid_z.reset()
            put_text(img, 'No marker', (10, 30), color=(0, 0, 255))
        self.cmd_vel_pub.publish(cmd)

        out_msg = self.bridge.cv2_to_imgmsg(img, encoding='bgr8')
        out_msg.header = msg.header
        self.image_pub.publish(out_msg)
        self._show(img)

    def _select_primary(self):
        """Keep the current target while it is still confirmed, else the longest-tracked one."""
        confirmed = [t for t in self.tracks.values() if t.confirmed]
        if not confirmed:
            self.primary_id = None
            return None
        current = self.tracks.get(self.primary_id)
        if current is None or not current.confirmed:
            current = min(confirmed, key=lambda t: t.first_seen)
            self.primary_id = current.id
        return current

    @staticmethod
    def _draw_crosshair(img, center, arm=25):
        cx, cy = center
        for color, thickness in (((0, 0, 0), 4), ((255, 255, 255), 2)):
            cv2.line(img, (cx - arm, cy), (cx + arm, cy), color, thickness, cv2.LINE_AA)
            cv2.line(img, (cx, cy - arm), (cx, cy + arm), color, thickness, cv2.LINE_AA)

    @staticmethod
    def _draw_pending(img, marker_corners):
        """Grey outline for a detection that is not (yet) confirmed."""
        cv2.polylines(img, [marker_corners.reshape(4, 2).astype(np.int32)], True,
                      (160, 160, 160), 1, cv2.LINE_AA)

    def _show(self, img):
        if not self.window_ok:
            return
        try:
            cv2.imshow(WINDOW_NAME, img)
            cv2.waitKey(1)
        except cv2.error as e:
            self.window_ok = False
            self.get_logger().warn(f'Cannot open OpenCV window ({e}); continuing without it')


def main(args=None):
    rclpy.init(args=args)
    node = ArucoTrackerNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        cv2.destroyAllWindows()
        if rclpy.ok():
            node.destroy_node()
            rclpy.shutdown()


if __name__ == '__main__':
    main()
