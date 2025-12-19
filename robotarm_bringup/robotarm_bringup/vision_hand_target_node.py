import math
import cv2
import numpy as np
import pyrealsense2 as rs
import mediapipe as mp
assert hasattr(mp, "solutions"), "mediapipe broken (solutions missing)"

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import PointStamped

def rt_to_T(rvec, tvec):
    R, _ = cv2.Rodrigues(rvec.reshape(3, 1))
    t = tvec.reshape(3, 1)
    T = np.eye(4, dtype=np.float64)
    T[:3, :3] = R
    T[:3, 3:4] = t
    return T

def T_to_rt(T):
    R = T[:3, :3]
    t = T[:3, 3].reshape(3, 1)
    rvec, _ = cv2.Rodrigues(R)
    return rvec.reshape(3,), t.reshape(3,)

class VisionHandTarget(Node):
    def __init__(self):
        super().__init__("vision_hand_target")

        self.declare_parameter("marker_id", 0)
        self.declare_parameter("marker_length", 0.096)     # m
        self.declare_parameter("tbm_path", "T_B_M.npz")
        self.declare_parameter("topic", "/hand_target_point")
        self.declare_parameter("safe_z_offset_mm", 120.0)
        self.declare_parameter("publish_hz", 10.0)
        self.declare_parameter("alpha", 0.25)
        self.declare_parameter("show", True)

        self.declare_parameter("x_min_m", 0.150)
        self.declare_parameter("x_max_m", 0.500)
        self.declare_parameter("y_min_m", -0.300)
        self.declare_parameter("y_max_m", 0.300)
        self.declare_parameter("z_min_m", 0.050)
        self.declare_parameter("z_max_m", 0.600)

        self.marker_id = int(self.get_parameter("marker_id").value)
        self.marker_len = float(self.get_parameter("marker_length").value)
        self.tbm_path = str(self.get_parameter("tbm_path").value)
        self.topic = str(self.get_parameter("topic").value)
        self.safe_z = float(self.get_parameter("safe_z_offset_mm").value) / 1000.0
        self.alpha = float(self.get_parameter("alpha").value)
        self.show = bool(self.get_parameter("show").value)

        self.x_min = float(self.get_parameter("x_min_m").value)
        self.x_max = float(self.get_parameter("x_max_m").value)
        self.y_min = float(self.get_parameter("y_min_m").value)
        self.y_max = float(self.get_parameter("y_max_m").value)
        self.z_min = float(self.get_parameter("z_min_m").value)
        self.z_max = float(self.get_parameter("z_max_m").value)

        T_B_M = np.load(self.tbm_path)["T_B_M"]
        self.T_M_B = np.linalg.inv(T_B_M)
        self.get_logger().info(f"Loaded {self.tbm_path} and computed inverse.")

        self.pub = self.create_publisher(PointStamped, self.topic, 10)

        aruco_dict = cv2.aruco.getPredefinedDictionary(cv2.aruco.DICT_4X4_250)
        params = cv2.aruco.DetectorParameters()
        self.use_new_api = hasattr(cv2.aruco, "ArucoDetector")
        self.detector = cv2.aruco.ArucoDetector(aruco_dict, params) if self.use_new_api else None
        self.aruco_dict = aruco_dict
        self.aruco_params = params

        self.mp_hands = mp.solutions.hands
        self.hands = self.mp_hands.Hands(
            static_image_mode=False,
            max_num_hands=1,
            model_complexity=1,
            min_detection_confidence=0.6,
            min_tracking_confidence=0.6
        )

        self.pipeline = rs.pipeline()
        config = rs.config()
        config.enable_stream(rs.stream.color, 640, 480, rs.format.bgr8, 30)
        config.enable_stream(rs.stream.depth, 640, 480, rs.format.z16, 30)
        profile = self.pipeline.start(config)

        self.align = rs.align(rs.stream.color)
        self.color_intr = profile.get_stream(rs.stream.color).as_video_stream_profile().get_intrinsics()

        self.mtx = np.array([[self.color_intr.fx, 0, self.color_intr.ppx],
                             [0, self.color_intr.fy, self.color_intr.ppy],
                             [0, 0, 1]], dtype=np.float64)
        self.dist = np.zeros((1, 5), dtype=np.float64)

        self.prev_target = None

        hz = float(self.get_parameter("publish_hz").value)
        self.timer = self.create_timer(1.0 / hz, self.tick)

        self.get_logger().info("vision_hand_target node started.")

    def clamp(self, p):
        p[0] = max(self.x_min, min(self.x_max, p[0]))
        p[1] = max(self.y_min, min(self.y_max, p[1]))
        p[2] = max(self.z_min, min(self.z_max, p[2]))
        return p

    def tick(self):
        frames = self.pipeline.wait_for_frames()
        frames = self.align.process(frames)
        cf = frames.get_color_frame()
        df = frames.get_depth_frame()
        if not cf or not df:
            return

        frame = np.asanyarray(cf.get_data())
        gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)

        if self.use_new_api:
            corners, ids, _ = self.detector.detectMarkers(gray)
        else:
            corners, ids, _ = cv2.aruco.detectMarkers(gray, self.aruco_dict, parameters=self.aruco_params)

        if ids is None or len(ids) == 0:
            if self.show:
                cv2.imshow("vision_hand_target", frame)
                cv2.waitKey(1)
            return

        ids_flat = ids.flatten().tolist()
        rvecs, tvecs, _ = cv2.aruco.estimatePoseSingleMarkers(corners, self.marker_len, self.mtx, self.dist)

        if self.marker_id not in ids_flat:
            if self.show:
                cv2.imshow("vision_hand_target", frame)
                cv2.waitKey(1)
            return

        idx = ids_flat.index(self.marker_id)
        T_C_M = rt_to_T(rvecs[idx].reshape(3,), tvecs[idx].reshape(3,))
        T_C_B = T_C_M @ self.T_M_B
        T_B_C = np.linalg.inv(T_C_B)

        rgb = cv2.cvtColor(frame, cv2.COLOR_BGR2RGB)
        res = self.hands.process(rgb)
        if not res.multi_hand_landmarks:
            if self.show:
                cv2.drawFrameAxes(frame, self.mtx, self.dist, rvecs[idx], tvecs[idx], self.marker_len)
                cv2.imshow("vision_hand_target", frame)
                cv2.waitKey(1)
            return

        lm = res.multi_hand_landmarks[0].landmark

        u = int(lm[9].x * frame.shape[1])
        v = int(lm[9].y * frame.shape[0])
        u = max(0, min(frame.shape[1]-1, u))
        v = max(0, min(frame.shape[0]-1, v))

        depth = df.get_distance(u, v)
        if depth <= 0.0:
            return

        X, Y, Z = rs.rs2_deproject_pixel_to_point(self.color_intr, [u, v], depth)
        p_C = np.array([X, Y, Z, 1.0], dtype=np.float64).reshape(4, 1)

        p_B = (T_B_C @ p_C).reshape(-1)
        target = p_B[:3].copy()
        target[2] += self.safe_z
        target = self.clamp(target)

        if self.prev_target is None:
            smooth = target
        else:
            smooth = self.alpha * target + (1.0 - self.alpha) * self.prev_target
        self.prev_target = smooth

        msg = PointStamped()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = "base"
        msg.point.x = float(smooth[0])
        msg.point.y = float(smooth[1])
        msg.point.z = float(smooth[2])
        self.pub.publish(msg)

        if self.show:
            cv2.aruco.drawDetectedMarkers(frame, corners, ids)
            cv2.drawFrameAxes(frame, self.mtx, self.dist, rvecs[idx], tvecs[idx], self.marker_len)

            rvecB, tvecB = T_to_rt(T_C_B)
            cv2.drawFrameAxes(frame, self.mtx, self.dist, rvecB, tvecB, 0.10)

            cv2.circle(frame, (u, v), 6, (0, 255, 0), -1)

            mm = smooth * 1000.0
            cv2.putText(frame, f"target_B(mm)=[{mm[0]:.1f},{mm[1]:.1f},{mm[2]:.1f}]",
                        (10, 30), cv2.FONT_HERSHEY_SIMPLEX, 0.65, (0,255,0), 2)

            cv2.imshow("vision_hand_target", frame)
            cv2.waitKey(1)

def main():
    rclpy.init()
    node = VisionHandTarget()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    node.destroy_node()
    rclpy.shutdown()

if __name__ == "__main__":
    main()
