#!/usr/bin/env python3
import os, json, math, time
from dataclasses import dataclass
from typing import Optional, Tuple

import numpy as np
import cv2

import rclpy
from rclpy.node import Node

from std_msgs.msg import Int32
from sensor_msgs.msg import Image, CameraInfo

from cv_bridge import CvBridge


def ensure_dir(p: str):
    os.makedirs(p, exist_ok=True)


def wrap_deg(deg: float) -> float:
    # [-180, 180]
    while deg > 180.0:
        deg -= 360.0
    while deg < -180.0:
        deg += 360.0
    return deg


def poly_area(corners_xy: np.ndarray) -> float:
    # corners_xy: (4,2)
    x = corners_xy[:, 0]
    y = corners_xy[:, 1]
    return 0.5 * abs(np.dot(x, np.roll(y, -1)) - np.dot(y, np.roll(x, -1)))


def edge_lengths(corners_xy: np.ndarray) -> Tuple[float, float]:
    # returns approx width_px, height_px (average of opposite edges)
    d01 = np.linalg.norm(corners_xy[1] - corners_xy[0])
    d12 = np.linalg.norm(corners_xy[2] - corners_xy[1])
    d23 = np.linalg.norm(corners_xy[3] - corners_xy[2])
    d30 = np.linalg.norm(corners_xy[0] - corners_xy[3])
    width = 0.5 * (d01 + d23)
    height = 0.5 * (d12 + d30)
    return float(width), float(height)


def marker_angle_deg(corners_xy: np.ndarray) -> float:
    # angle of edge (corner0 -> corner1) in image plane
    v = corners_xy[1] - corners_xy[0]
    return math.degrees(math.atan2(float(v[1]), float(v[0])))


def sharpness_lap_var(gray: np.ndarray, corners_xy: np.ndarray) -> float:
    # ROI bounding box around marker (clamped)
    h, w = gray.shape[:2]
    xmin = int(max(0, np.floor(np.min(corners_xy[:, 0]) - 5)))
    xmax = int(min(w - 1, np.ceil(np.max(corners_xy[:, 0]) + 5)))
    ymin = int(max(0, np.floor(np.min(corners_xy[:, 1]) - 5)))
    ymax = int(min(h - 1, np.ceil(np.max(corners_xy[:, 1]) + 5)))
    if xmax <= xmin or ymax <= ymin:
        return 0.0
    roi = gray[ymin:ymax, xmin:xmax]
    return float(cv2.Laplacian(roi, cv2.CV_64F).var())


@dataclass
class Detection:
    marker_id: int
    corners: np.ndarray  # (4,2) float32
    center: Tuple[float, float]
    area: float
    width_px: float
    height_px: float
    angle_deg: float
    sharpness: float
    score: float
    stamp_sec: float


class ArucoBaselineNode(Node):
    def __init__(self):
        super().__init__('ess_aruco_baseline')

        # --- Parameters ---
        self.declare_parameter('mode', 'focus')  # focus | calibrate | run
        self.declare_parameter('use_trigger', True)  # focus에서는 False 추천
        self.declare_parameter('marker_id', 0)
        self.declare_parameter('dictionary', 'DICT_4X4_50')
        self.declare_parameter('baseline_path', '/var/lib/ess_aruco')
        self.declare_parameter('snaps_dir', '/var/lib/ess_aruco/snaps')
        self.declare_parameter('required_samples', 12)
        self.declare_parameter('min_area_norm', 0.0008)  # 화면 대비 최소 면적 비율
        self.declare_parameter('print_every', 1)  # focus에서 N번마다 출력

        # (선택) pose 추정용
        self.declare_parameter('use_pose', False)
        self.declare_parameter('marker_size_m', 0.05)  # 5cm 기본

        self.mode = str(self.get_parameter('mode').value).strip().lower()
        self.use_trigger = bool(self.get_parameter('use_trigger').value)
        self.marker_id = int(self.get_parameter('marker_id').value)
        self.baseline_path = str(self.get_parameter('baseline_path').value)
        self.snaps_dir = str(self.get_parameter('snaps_dir').value)
        self.required_samples = int(self.get_parameter('required_samples').value)
        self.min_area_norm = float(self.get_parameter('min_area_norm').value)
        self.print_every = int(self.get_parameter('print_every').value)

        self.use_pose = bool(self.get_parameter('use_pose').value)
        self.marker_size_m = float(self.get_parameter('marker_size_m').value)

        ensure_dir(self.snaps_dir)

        # --- ROS I/O ---
        self.bridge = CvBridge()
        self.latest_img_msg: Optional[Image] = None
        self.latest_bgr: Optional[np.ndarray] = None
        self.latest_stamp_sec: float = 0.0

        self.cam_info: Optional[CameraInfo] = None
        self.K: Optional[np.ndarray] = None
        self.D: Optional[np.ndarray] = None

        self.sub_img = self.create_subscription(Image, '/camera/image_raw', self.cb_img, 10)
        self.sub_info = self.create_subscription(CameraInfo, '/camera/camera_info', self.cb_info, 10)
        self.sub_ack = self.create_subscription(Int32, '/ess/thermal/ack', self.cb_ack, 10)

        self.focus_tick = 0
        self.samples: list[Detection] = []

        self.baseline = None
        if self.mode == 'run':
            self.baseline = self.load_baseline()

        # focus 모드에서 트리거 없이 계속 스코어 뽑아주기
        if self.mode == 'focus' and (not self.use_trigger):
            self.timer = self.create_timer(0.2, self.focus_loop)  # 5Hz
        else:
            self.timer = None

        self.get_logger().info(
            f"mode={self.mode}, use_trigger={self.use_trigger}, marker_id={self.marker_id}, "
            f"baseline={self.baseline_path}, snaps={self.snaps_dir}"
        )

    def cb_info(self, msg: CameraInfo):
        self.cam_info = msg
        # CameraInfo.K is 9 elements row-major
        self.K = np.array(msg.k, dtype=np.float64).reshape(3, 3)
        self.D = np.array(msg.d, dtype=np.float64) if len(msg.d) > 0 else np.zeros((5,), dtype=np.float64)

    def cb_img(self, msg: Image):
        self.latest_img_msg = msg
        self.latest_stamp_sec = float(msg.header.stamp.sec) + float(msg.header.stamp.nanosec) * 1e-9
        try:
            self.latest_bgr = self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')
        except Exception as e:
            self.latest_bgr = None
            self.get_logger().warn(f"cv_bridge convert failed: {e}")

    def cb_ack(self, msg: Int32):
        if msg.data != 1:
            return
        if self.mode == 'focus' and (not self.use_trigger):
            return  # focus는 timer가 돌고 있음
        self.process_once(triggered=True)

    def focus_loop(self):
        self.process_once(triggered=False)

    def get_dict(self, name: str):
        name = name.strip()
        if not hasattr(cv2.aruco, name):
            self.get_logger().warn(f"Unknown aruco dictionary '{name}', fallback DICT_4X4_50")
            name = 'DICT_4X4_50'
        return cv2.aruco.getPredefinedDictionary(getattr(cv2.aruco, name))

    def detect(self, bgr: np.ndarray, stamp_sec: float) -> Optional[Detection]:
        h, w = bgr.shape[:2]
        gray = cv2.cvtColor(bgr, cv2.COLOR_BGR2GRAY)

        dictionary_name = str(self.get_parameter('dictionary').value)
        dictionary = self.get_dict(dictionary_name)

        params = cv2.aruco.DetectorParameters()
        corners_list, ids, rejected = cv2.aruco.detectMarkers(gray, dictionary, parameters=params)

        if ids is None or len(ids) == 0:
            return None

        ids = ids.flatten().tolist()
        best = None

        for i, mid in enumerate(ids):
            if mid != self.marker_id:
                continue
            corners = corners_list[i].reshape(4, 2).astype(np.float32)

            area = poly_area(corners)
            area_norm = area / float(w * h)
            if area_norm < self.min_area_norm:
                continue

            # corner refine (subpixel)
            term = (cv2.TERM_CRITERIA_EPS + cv2.TERM_CRITERIA_MAX_ITER, 20, 0.001)
            cv2.cornerSubPix(gray, corners, (5, 5), (-1, -1), term)

            cx = float(np.mean(corners[:, 0]))
            cy = float(np.mean(corners[:, 1]))
            width_px, height_px = edge_lengths(corners)
            ang = marker_angle_deg(corners)
            sharp = sharpness_lap_var(gray, corners)

            # score: sharpness * area_norm (초점 + 크기 둘 다 반영)
            score = sharp * area_norm

            det = Detection(
                marker_id=mid,
                corners=corners,
                center=(cx, cy),
                area=float(area),
                width_px=width_px,
                height_px=height_px,
                angle_deg=float(ang),
                sharpness=float(sharp),
                score=float(score),
                stamp_sec=stamp_sec
            )

            if best is None or det.score > best.score:
                best = det

        return best

    def draw_overlay(self, bgr: np.ndarray, det: Detection, text_lines: list[str]) -> np.ndarray:
        out = bgr.copy()
        pts = det.corners.astype(np.int32).reshape(-1, 1, 2)
        cv2.polylines(out, [pts], True, (0, 255, 0), 2)
        for p in det.corners.astype(np.int32):
            cv2.circle(out, (int(p[0]), int(p[1])), 4, (0, 255, 255), -1)
        cv2.circle(out, (int(det.center[0]), int(det.center[1])), 5, (255, 0, 0), -1)

        y = 25
        for line in text_lines:
            cv2.putText(out, line, (10, y), cv2.FONT_HERSHEY_SIMPLEX, 0.6, (255, 255, 255), 2)
            y += 22
        return out

    def save_image(self, img: np.ndarray, tag: str):
        ts = time.strftime("%Y%m%d_%H%M%S")
        path = os.path.join(self.snaps_dir, f"{tag}_{ts}.jpg")
        cv2.imwrite(path, img)
        return path

    def load_baseline(self):
        if not os.path.exists(self.baseline_path):
            self.get_logger().error(f"baseline not found: {self.baseline_path}")
            return None
        with open(self.baseline_path, 'r') as f:
            return json.load(f)

    def save_baseline(self, det: Detection, img_w: int, img_h: int):
        data = {
            "saved_at": time.strftime("%Y-%m-%d %H:%M:%S"),
            "stamp_sec": det.stamp_sec,
            "marker_id": det.marker_id,
            "image_w": img_w,
            "image_h": img_h,
            "corners": det.corners.tolist(),
            "center": [det.center[0], det.center[1]],
            "area": det.area,
            "width_px": det.width_px,
            "height_px": det.height_px,
            "angle_deg": det.angle_deg,
            "sharpness": det.sharpness,
            "score": det.score,
            "marker_size_m": self.marker_size_m,
            "camera_info": None
        }
        if self.cam_info is not None:
            data["camera_info"] = {
                "k": self.cam_info.k,
                "d": self.cam_info.d,
                "p": self.cam_info.p,
                "distortion_model": self.cam_info.distortion_model
            }

        with open(self.baseline_path, 'w') as f:
            json.dump(data, f, indent=2)

        self.get_logger().info(f"baseline saved: {self.baseline_path}")

    def compute_delta(self, base: dict, cur: Detection):
        base_center = np.array(base["center"], dtype=np.float64)
        base_w = float(base["width_px"])
        base_h = float(base["height_px"])
        base_ang = float(base["angle_deg"])
        base_area = float(base["area"])

        dx_px = cur.center[0] - float(base_center[0])
        dy_px = cur.center[1] - float(base_center[1])

        dx_norm = dx_px / max(1.0, base_w)
        dy_norm = dy_px / max(1.0, base_h)

        # scale proxy (area 기반 / width 기반 둘 다 쓸 수 있음)
        scale_w = cur.width_px / max(1.0, base_w)
        scale_a = math.sqrt(cur.area / max(1e-6, base_area))

        dtheta = wrap_deg(cur.angle_deg - base_ang)

        out = {
            "stamp_sec": cur.stamp_sec,
            "marker_id": cur.marker_id,
            "dx_px": dx_px,
            "dy_px": dy_px,
            "dx_norm": dx_norm,
            "dy_norm": dy_norm,
            "scale_w": scale_w,
            "scale_a": scale_a,
            "dtheta_deg": dtheta,
            "sharpness": cur.sharpness,
            "score": cur.score
        }

        # (선택) pose 추정
        if self.use_pose and (self.K is not None) and (self.D is not None):
            try:
                corners = cur.corners.reshape(1, 4, 2)
                rvecs, tvecs, _ = cv2.aruco.estimatePoseSingleMarkers(
                    corners, self.marker_size_m, self.K, self.D
                )
                rvec = rvecs[0, 0, :].tolist()
                tvec = tvecs[0, 0, :].tolist()
                out["pose"] = {"rvec": rvec, "tvec": tvec}
            except Exception as e:
                out["pose_error"] = str(e)

        return out

    def process_once(self, triggered: bool):
        if self.latest_bgr is None:
            if triggered:
                self.get_logger().warn("no image yet")
            return

        bgr = self.latest_bgr
        stamp = self.latest_stamp_sec

        det = self.detect(bgr, stamp)
        if det is None:
            if self.mode == 'focus':
                self.focus_tick += 1
                if self.focus_tick % max(1, self.print_every) == 0:
                    self.get_logger().info("NO marker detected")
            else:
                self.get_logger().warn("NO marker detected (trigger)")
            return

        h, w = bgr.shape[:2]

        if self.mode == 'focus':
            self.focus_tick += 1
            if self.focus_tick % max(1, self.print_every) == 0:
                self.get_logger().info(
                    f"FOCUS id={det.marker_id} center=({det.center[0]:.1f},{det.center[1]:.1f}) "
                    f"w={det.width_px:.1f}px area={det.area:.0f} sharp={det.sharpness:.1f} score={det.score:.4f}"
                )
            return

        if self.mode == 'calibrate':
            self.samples.append(det)
            self.get_logger().info(
                f"CAL sample {len(self.samples)}/{self.required_samples} "
                f"sharp={det.sharpness:.1f} score={det.score:.4f}"
            )

            overlay = self.draw_overlay(
                bgr, det,
                [f"CAL {len(self.samples)}/{self.required_samples}",
                 f"sharp={det.sharpness:.1f} score={det.score:.4f}"]
            )
            self.save_image(overlay, "cal")

            if len(self.samples) >= self.required_samples:
                best = max(self.samples, key=lambda x: x.score)
                self.save_baseline(best, w, h)

                overlay2 = self.draw_overlay(
                    bgr, best,
                    ["BASELINE SAVED",
                     f"sharp={best.sharpness:.1f} score={best.score:.4f}",
                     f"center=({best.center[0]:.1f},{best.center[1]:.1f})"]
                )
                img_path = self.save_image(overlay2, "baseline")
                self.get_logger().info(f"baseline snapshot: {img_path}")

                self.samples.clear()
            return

        if self.mode == 'run':
            if self.baseline is None:
                self.baseline = self.load_baseline()
                if self.baseline is None:
                    return

            delta = self.compute_delta(self.baseline, det)

            out_path = "/tmp/aruco_delta.json"
            with open(out_path, 'w') as f:
                json.dump(delta, f, indent=2)

            self.get_logger().info(
                f"RUN dx_norm={delta['dx_norm']:.3f} dy_norm={delta['dy_norm']:.3f} "
                f"scale_w={delta['scale_w']:.3f} dtheta={delta['dtheta_deg']:.1f}deg"
            )

            overlay = self.draw_overlay(
                bgr, det,
                [f"dx_norm={delta['dx_norm']:.3f} dy_norm={delta['dy_norm']:.3f}",
                 f"scale_w={delta['scale_w']:.3f} dtheta={delta['dtheta_deg']:.1f}deg"]
            )
            self.save_image(overlay, "run")
            return

        self.get_logger().warn(f"unknown mode: {self.mode}")


def main():
    rclpy.init()
    node = ArucoBaselineNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
