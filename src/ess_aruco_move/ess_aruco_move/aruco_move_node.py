#!/usr/bin/env python3
import os
import json
import math
import time
import threading
from typing import Optional, Tuple

import numpy as np
import cv2

import rclpy
from rclpy.node import Node

from std_msgs.msg import Int32, Float32
from sensor_msgs.msg import Image
from geometry_msgs.msg import Twist
from cv_bridge import CvBridge


def ensure_dir(p: str):
    os.makedirs(p, exist_ok=True)


def wrap_rad(rad: float) -> float:
    # [-pi, pi]
    while rad > math.pi:
        rad -= 2.0 * math.pi
    while rad < -math.pi:
        rad += 2.0 * math.pi
    return rad


def edge_lengths(corners_xy: np.ndarray) -> Tuple[float, float]:
    # corners_xy: (4,2)
    d01 = np.linalg.norm(corners_xy[1] - corners_xy[0])
    d12 = np.linalg.norm(corners_xy[2] - corners_xy[1])
    d23 = np.linalg.norm(corners_xy[3] - corners_xy[2])
    d30 = np.linalg.norm(corners_xy[0] - corners_xy[3])
    width = 0.5 * (d01 + d23)
    height = 0.5 * (d12 + d30)
    return float(width), float(height)


def marker_size_px(corners_xy: np.ndarray) -> float:
    w, h = edge_lengths(corners_xy)
    return 0.5 * (w + h)


def marker_angle_rad(corners_xy: np.ndarray) -> float:
    # edge (corner0 -> corner1)
    v = corners_xy[1] - corners_xy[0]
    return math.atan2(float(v[1]), float(v[0]))


class ArucoMoveNode(Node):
    """
    Modes
      - calibrate: baseline 저장 (request=1 한 번)
      - run:
          * continuous:=false  -> 원샷 세션 (DONE 시 ack=1 후 종료)
          * continuous:=true   -> 계속 추적 세션 (request=0으로 종료)

    추가(이번 패치 핵심):
      - search_until_done:=true + continuous:=false
        => timeout/miss로 FAIL 끝내지 않고, 마커를 잃으면 '탐색 회전'으로 계속 찾다가
           hold_need 달성하면 DONE으로 끝(ack=1).
    """

    def __init__(self):
        super().__init__('ess_aruco_move')

        # ===== Parameters =====
        self.declare_parameter('mode', 'run')  # run | calibrate
        self.declare_parameter('dictionary', 'DICT_4X4_50')

        # triggers
        self.declare_parameter('use_aruco_request', True)
        self.declare_parameter('aruco_request_topic', '/ess/aruco/request')

        # (레거시) thermal ack 트리거 (선택)
        self.declare_parameter('use_thermal_trigger', False)
        self.declare_parameter('thermal_ack_topic', '/ess/thermal/ack')
        self.declare_parameter('trigger_value', 9999)
        self.declare_parameter('emit_ack_on_thermal', False)

        # tracking mode
        self.declare_parameter('continuous', False)           # True면 계속 추적(1=시작,0=정지)
        self.declare_parameter('continuous_max_sec', 0.0)     # 0이면 무한, >0이면 자동 종료(안전용)

        # === infinite-search oneshot mode ===
        self.declare_parameter('search_until_done', False)    # True면 oneshot이라도 DONE까지 무한 반복
        self.declare_parameter('search_when_lost', True)      # lost면 STOP(wait) 대신 회전 탐색
        self.declare_parameter('search_yaw', 0.20)            # 탐색 회전 속도(rad/s)
        self.declare_parameter('search_flip_sec', 3.0)        # N초마다 회전 방향 반전(0이면 반전 안 함)
        self.declare_parameter('lost_log_every', 10)          # miss N번마다 lost 로그
        self.declare_parameter('lost_save_every', 0)          # 0이면 저장 안 함, 예: 30이면 miss 30마다 raw 저장

        # marker / paths
        self.declare_parameter('target_aruco_id', 0)
        self.declare_parameter('baseline_path', '/var/lib/ess_aruco/baseline_id0.json')
        self.declare_parameter('save_dir', '/var/lib/ess_aruco/snaps')

        # outputs
        self.declare_parameter('publish_cmd_vel', True)
        self.declare_parameter('cmd_vel_topic', '/cmd_vel')
        self.declare_parameter('cmd_value_topic', '/cmd_value')
        self.declare_parameter('cmd_yaw_topic', '/cmd_yaw')

        # gains
        self.declare_parameter('k_yaw_rad_per_px', 0.0010)
        self.declare_parameter('k_lin_m_per_scale', 0.30)
        self.declare_parameter('scale_deadband', 0.03)

        # control loop
        self.declare_parameter('run_hz', 10.0)
        self.declare_parameter('run_timeout_sec', 0.5)  # oneshot에서만 의미 (search_until_done면 무시)
        self.declare_parameter('miss_limit', 6)

        # yaw-first gate + tolerances
        self.declare_parameter('dx_gate_px', 35.0)
        self.declare_parameter('dx_tol_px', 6.0)
        self.declare_parameter('scale_tol', 0.01)
        self.declare_parameter('dtheta_tol_rad', 0.025)
        self.declare_parameter('done_hold_count', 3)

        # clamps
        self.declare_parameter('max_lin', 0.15)
        self.declare_parameter('max_yaw', 0.25)

        # sign flips (±1)
        self.declare_parameter('yaw_sign', 1.0)
        self.declare_parameter('lin_sign', -1.0)

        # logging / saving
        self.declare_parameter('save_every_n', 5)
        self.declare_parameter('log_every_n', 5)

        # stop publish
        self.declare_parameter('stop_repeat', 6)
        self.declare_parameter('stop_repeat_sleep', 0.05)

        # ===== Internal =====
        self.mode = str(self.get_parameter('mode').value).strip().lower()
        self.bridge = CvBridge()

        self._img_lock = threading.Lock()
        self._latest_bgr: Optional[np.ndarray] = None
        self._latest_stamp = 0.0

        self._state_lock = threading.Lock()
        self._busy = False
        self._worker: Optional[threading.Thread] = None

        # stop/cancel event (oneshot/continuous 공통)
        self._cancel_evt = threading.Event()

        # ensure dirs
        ensure_dir(os.path.dirname(str(self.get_parameter('baseline_path').value)) or '/var/lib/ess_aruco')
        ensure_dir(str(self.get_parameter('save_dir').value))

        # ===== ROS I/O =====
        self.sub_img = self.create_subscription(Image, '/camera/image_raw', self.cb_img, 10)

        # request trigger
        if bool(self.get_parameter('use_aruco_request').value):
            req_topic = str(self.get_parameter('aruco_request_topic').value)
            self.sub_req = self.create_subscription(Int32, req_topic, self.cb_req, 10)
        else:
            self.sub_req = None

        # thermal trigger (optional)
        if bool(self.get_parameter('use_thermal_trigger').value):
            th_topic = str(self.get_parameter('thermal_ack_topic').value)
            self.sub_th = self.create_subscription(Int32, th_topic, self.cb_thermal, 10)
        else:
            self.sub_th = None

        # ack
        self.pub_aruco_ack = self.create_publisher(Int32, '/ess/aruco/ack', 10)

        # commands
        self.pub_cmd_vel = self.create_publisher(Twist, str(self.get_parameter('cmd_vel_topic').value), 10)
        self.pub_cmd_value = self.create_publisher(Float32, str(self.get_parameter('cmd_value_topic').value), 10)
        self.pub_cmd_yaw = self.create_publisher(Float32, str(self.get_parameter('cmd_yaw_topic').value), 10)

        self.get_logger().info(
            f"READY(mode={self.mode}): image=/camera/image_raw, baseline={str(self.get_parameter('baseline_path').value)}, "
            f"continuous={bool(self.get_parameter('continuous').value)}, "
            f"search_until_done={bool(self.get_parameter('search_until_done').value)}, "
            f"search_when_lost={bool(self.get_parameter('search_when_lost').value)}"
        )

    # ===== callbacks =====
    def cb_img(self, msg: Image):
        try:
            bgr = self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')
        except Exception as e:
            self.get_logger().warn(f"cv_bridge convert failed: {e}")
            return

        stamp = float(msg.header.stamp.sec) + float(msg.header.stamp.nanosec) * 1e-9
        with self._img_lock:
            self._latest_bgr = bgr
            self._latest_stamp = stamp

    def cb_req(self, msg: Int32):
        # request protocol:
        #   1 -> start
        #   0 -> stop (cancel)
        if int(msg.data) == 0:
            self._request_stop()
            return
        if int(msg.data) == 1:
            self._start_once(emit_ack=True)
            return

    def cb_thermal(self, msg: Int32):
        trig = int(self.get_parameter('trigger_value').value)
        if int(msg.data) != trig:
            return
        emit = bool(self.get_parameter('emit_ack_on_thermal').value)
        self._start_once(emit_ack=emit)

    # ===== helpers =====
    def _pub_f32(self, pub, v: float):
        m = Float32()
        m.data = float(v)
        pub.publish(m)

    def _publish_twist(self, lin: float, yaw: float):
        t = Twist()
        t.linear.x = float(lin)
        t.angular.z = float(yaw)
        self.pub_cmd_vel.publish(t)

    def _publish_stop(self):
        if bool(self.get_parameter('publish_cmd_vel').value):
            self._publish_twist(0.0, 0.0)
        self._pub_f32(self.pub_cmd_value, 0.0)
        self._pub_f32(self.pub_cmd_yaw, 0.0)

    def _publish_search(self, yaw: float):
        if bool(self.get_parameter('publish_cmd_vel').value):
            self._publish_twist(0.0, float(yaw))
        self._pub_f32(self.pub_cmd_yaw, float(yaw))
        self._pub_f32(self.pub_cmd_value, 0.0)

    def _publish_stop_burst(self):
        n = int(self.get_parameter('stop_repeat').value)
        sl = float(self.get_parameter('stop_repeat_sleep').value)
        for _ in range(max(1, n)):
            self._publish_stop()
            time.sleep(max(0.0, sl))

    def _clamp(self, x: float, lo: float, hi: float) -> float:
        return float(max(lo, min(hi, x)))

    def _get_dict(self, name: str):
        name = name.strip()
        if not hasattr(cv2.aruco, name):
            self.get_logger().warn(f"Unknown aruco dictionary '{name}', fallback DICT_4X4_50")
            name = 'DICT_4X4_50'
        return cv2.aruco.getPredefinedDictionary(getattr(cv2.aruco, name))

    def _detect_marker(self, gray: np.ndarray):
        dic_name = str(self.get_parameter('dictionary').value)
        dictionary = self._get_dict(dic_name)
        try:
            params = cv2.aruco.DetectorParameters()
            corners_list, ids, _ = cv2.aruco.detectMarkers(gray, dictionary, parameters=params)
        except Exception:
            params = cv2.aruco.DetectorParameters_create()
            corners_list, ids, _ = cv2.aruco.detectMarkers(gray, dictionary, parameters=params)

        if ids is None or len(ids) == 0:
            return None, None

        ids = ids.flatten().tolist()
        return corners_list, ids

    def _pick_target(self, corners_list, ids):
        tid = int(self.get_parameter('target_aruco_id').value)
        best = None
        best_area = -1.0
        for i, mid in enumerate(ids):
            if int(mid) != tid:
                continue
            corners = corners_list[i].reshape(4, 2).astype(np.float32)
            s = marker_size_px(corners)
            area_proxy = s * s
            if area_proxy > best_area:
                best_area = area_proxy
                best = corners
        return best, tid

    def _overlay(self, bgr: np.ndarray, corners: np.ndarray, lines):
        out = bgr.copy()
        pts = corners.astype(np.int32).reshape(-1, 1, 2)
        cv2.polylines(out, [pts], True, (0, 255, 0), 2)

        cx = float(np.mean(corners[:, 0]))
        cy = float(np.mean(corners[:, 1]))
        cv2.circle(out, (int(cx), int(cy)), 5, (255, 0, 0), -1)

        y = 25
        for line in lines:
            cv2.putText(out, line, (10, y), cv2.FONT_HERSHEY_SIMPLEX, 0.6, (255, 255, 255), 2)
            y += 22
        return out

    def _save_snap(self, bgr: np.ndarray, dbg: Optional[np.ndarray] = None) -> Tuple[str, Optional[str]]:
        save_dir = str(self.get_parameter('save_dir').value)
        ensure_dir(save_dir)
        ts = time.strftime("%Y%m%d_%H%M%S")
        raw_path = os.path.join(save_dir, f"snap_{ts}.jpg")
        cv2.imwrite(raw_path, bgr)
        dbg_path = None
        if dbg is not None:
            dbg_path = os.path.join(save_dir, f"snap_{ts}_dbg.jpg")
            cv2.imwrite(dbg_path, dbg)
        return raw_path, dbg_path

    def _load_baseline(self) -> Optional[dict]:
        p = str(self.get_parameter('baseline_path').value)
        if not os.path.exists(p):
            return None
        with open(p, 'r') as f:
            return json.load(f)

    def _save_baseline(self, data: dict):
        p = str(self.get_parameter('baseline_path').value)
        ensure_dir(os.path.dirname(p) or '/var/lib/ess_aruco')
        with open(p, 'w') as f:
            json.dump(data, f, indent=2)

    def _emit_ack(self, ok: bool):
        ack = Int32()
        ack.data = 1 if ok else 0
        self.pub_aruco_ack.publish(ack)

    def _request_stop(self):
        with self._state_lock:
            running = self._busy

        if running:
            self._cancel_evt.set()
            self._publish_stop_burst()
            return

        self._publish_stop_burst()
        self._emit_ack(True)

    # ===== main trigger entry =====
    def _start_once(self, emit_ack: bool):
        with self._state_lock:
            if self._busy:
                return
            self._busy = True

        ok = False
        try:
            # 새 세션 시작 시 cancel 해제 (oneshot/continuous 공통)
            self._cancel_evt.clear()

            if self.mode == 'calibrate':
                ok = self._do_calibrate_once()
                return

            base = self._load_baseline()
            if base is None:
                self.get_logger().warn("no baseline found. run with -p mode:=calibrate first.")
                ok = False
                return

            self._worker = threading.Thread(
                target=self._run_session_thread,
                args=(base, emit_ack),
                daemon=True
            )
            self._worker.start()
            return

        finally:
            if self.mode == 'calibrate':
                if emit_ack:
                    self._emit_ack(ok)
                with self._state_lock:
                    self._busy = False

    def _do_calibrate_once(self) -> bool:
        with self._img_lock:
            frame = None if self._latest_bgr is None else self._latest_bgr.copy()

        if frame is None:
            self.get_logger().warn("no frame yet")
            return False

        gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
        corners_list, ids = self._detect_marker(gray)
        if ids is None:
            raw, _ = self._save_snap(frame, None)
            self.get_logger().warn(f"[FAIL] aruco not found (raw={raw})")
            return False

        corners, tid = self._pick_target(corners_list, ids)
        if corners is None:
            raw, _ = self._save_snap(frame, None)
            self.get_logger().warn(f"[FAIL] target id={tid} not found (raw={raw})")
            return False

        cx = float(np.mean(corners[:, 0]))
        cy = float(np.mean(corners[:, 1]))
        size = float(marker_size_px(corners))
        th = float(marker_angle_rad(corners))

        data = {
            "saved_at": time.strftime("%Y-%m-%d %H:%M:%S"),
            "marker_id": int(tid),
            "cx": cx,
            "cy": cy,
            "size_px": size,
            "theta_rad": th
        }

        self._save_baseline(data)
        self.get_logger().info(f"[CALIB] saved baseline -> {str(self.get_parameter('baseline_path').value)}")

        dbg = self._overlay(frame, corners, [
            f"CALIB id={tid}",
            f"cx={cx:.1f} cy={cy:.1f}",
            f"size={size:.1f}px th={th:.2f}rad",
            "BASELINE SAVED"
        ])
        raw, dbg_path = self._save_snap(frame, dbg)
        self.get_logger().info(f"[CALIB] snap raw={raw} dbg={dbg_path}")
        return True

    # ===== run session =====
    def _compute_cmd(self, dx: float, scale: float, dth: float) -> Tuple[float, float]:
        k_yaw = float(self.get_parameter('k_yaw_rad_per_px').value)
        k_lin = float(self.get_parameter('k_lin_m_per_scale').value)

        yaw_sign = float(self.get_parameter('yaw_sign').value)
        lin_sign = float(self.get_parameter('lin_sign').value)

        yaw = yaw_sign * (-dx * k_yaw)
        lin = lin_sign * (-scale * k_lin)

        dx_gate = float(self.get_parameter('dx_gate_px').value)
        if abs(dx) > dx_gate:
            lin = 0.0

        scale_deadband = float(self.get_parameter('scale_deadband').value)
        if abs(scale) < scale_deadband:
            lin = 0.0

        max_lin = float(self.get_parameter('max_lin').value)
        max_yaw = float(self.get_parameter('max_yaw').value)
        lin = self._clamp(lin, -max_lin, +max_lin)
        yaw = self._clamp(yaw, -max_yaw, +max_yaw)
        return lin, yaw

    def _run_session_thread(self, base: dict, emit_ack: bool):
        ok = False
        try:
            ok = self._run_session(base)
        finally:
            # hard stop on exit (covers DONE/FAIL/cancel paths)
            self._publish_stop_burst()
            if emit_ack:
                self._emit_ack(ok)
            with self._state_lock:
                self._busy = False

    def _run_session(self, base: dict) -> bool:
        continuous = bool(self.get_parameter('continuous').value)
        search_until_done = bool(self.get_parameter('search_until_done').value)
        search_when_lost = bool(self.get_parameter('search_when_lost').value)
        search_yaw = float(self.get_parameter('search_yaw').value)
        search_flip_sec = float(self.get_parameter('search_flip_sec').value)
        lost_log_every = int(self.get_parameter('lost_log_every').value)
        lost_save_every = int(self.get_parameter('lost_save_every').value)

        hz = float(self.get_parameter('run_hz').value)
        period = 1.0 / max(1.0, hz)

        timeout_sec = float(self.get_parameter('run_timeout_sec').value)
        start = time.time()

        max_sec = float(self.get_parameter('continuous_max_sec').value)

        miss_limit = int(self.get_parameter('miss_limit').value)
        publish_cmd_vel = bool(self.get_parameter('publish_cmd_vel').value)

        dx_tol = float(self.get_parameter('dx_tol_px').value)
        scale_tol = float(self.get_parameter('scale_tol').value)
        dth_tol = float(self.get_parameter('dtheta_tol_rad').value)
        hold_need = int(self.get_parameter('done_hold_count').value)

        save_every_n = int(self.get_parameter('save_every_n').value)
        log_every_n = int(self.get_parameter('log_every_n').value)

        base_cx = float(base["cx"])
        base_cy = float(base["cy"])
        base_size = float(base["size_px"])
        base_th = float(base["theta_rad"])

        miss = 0
        hold = 0
        i = 0

        search_dir = 1.0
        search_last_flip = time.time()

        while True:
            if self._cancel_evt.is_set():
                self._publish_stop_burst()
                self.get_logger().warn("[STOP] cancelled by request")
                return False

            if continuous and (max_sec > 0.0) and ((time.time() - start) >= max_sec):
                self.get_logger().warn(f"[STOP] continuous_max_sec reached ({max_sec:.1f}s)")
                self._publish_stop_burst()
                return True

            if (not continuous) and (not search_until_done) and (timeout_sec > 0.0) and ((time.time() - start) >= timeout_sec):
                break

            with self._img_lock:
                frame = None if self._latest_bgr is None else self._latest_bgr.copy()

            if frame is None:
                self._publish_stop()
                time.sleep(period)
                continue

            gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
            corners_list, ids = self._detect_marker(gray)

            if ids is None:
                miss += 1
                hold = 0

                if (lost_save_every > 0) and (miss % lost_save_every == 0):
                    raw, _ = self._save_snap(frame, None)
                    self.get_logger().warn(f"[RUN] marker lost miss={miss} (raw={raw})")

                if continuous or search_until_done:
                    if search_when_lost:
                        if (search_flip_sec > 0.0) and ((time.time() - search_last_flip) >= search_flip_sec):
                            search_dir *= -1.0
                            search_last_flip = time.time()
                        yaw = search_dir * search_yaw
                        self._publish_search(yaw)
                        if (lost_log_every > 0) and (miss % lost_log_every == 0):
                            self.get_logger().warn(f"[RUN] marker lost (miss={miss}) -> SEARCH yaw={yaw:+.3f}")
                    else:
                        self._publish_stop()
                        if (lost_log_every > 0) and (miss % lost_log_every == 0):
                            self.get_logger().warn(f"[RUN] marker lost (miss={miss}) -> STOP(wait)")
                    time.sleep(period)
                    i += 1
                    continue

                if miss >= miss_limit:
                    self.get_logger().warn(f"[RUN] marker lost {miss}/{miss_limit}")
                    break

                time.sleep(period)
                i += 1
                continue

            corners, tid = self._pick_target(corners_list, ids)

            if corners is None:
                miss += 1
                hold = 0

                if continuous or search_until_done:
                    if search_when_lost:
                        if (search_flip_sec > 0.0) and ((time.time() - search_last_flip) >= search_flip_sec):
                            search_dir *= -1.0
                            search_last_flip = time.time()
                        yaw = search_dir * search_yaw
                        self._publish_search(yaw)
                        if (lost_log_every > 0) and (miss % lost_log_every == 0):
                            self.get_logger().warn(f"[RUN] target id lost (miss={miss}) -> SEARCH yaw={yaw:+.3f}")
                    else:
                        self._publish_stop()
                        if (lost_log_every > 0) and (miss % lost_log_every == 0):
                            self.get_logger().warn(f"[RUN] target id lost (miss={miss}) -> STOP(wait)")
                    time.sleep(period)
                    i += 1
                    continue

                if miss >= miss_limit:
                    self.get_logger().warn(
                        f"[RUN] target id={int(self.get_parameter('target_aruco_id').value)} lost {miss}/{miss_limit}"
                    )
                    break

                time.sleep(period)
                i += 1
                continue

            miss = 0

            cx = float(np.mean(corners[:, 0]))
            cy = float(np.mean(corners[:, 1]))
            size = float(marker_size_px(corners))
            th = float(marker_angle_rad(corners))

            dx = cx - base_cx
            dy = cy - base_cy
            scale = (size / max(1e-6, base_size)) - 1.0
            dth = wrap_rad(th - base_th)

            in_tol = (abs(dx) < dx_tol) and (abs(scale) < scale_tol) and (abs(dth) < dth_tol)
            if in_tol:
                hold += 1
            else:
                hold = 0

            if continuous and (hold >= hold_need):
                self._publish_stop()
                lin, yaw = 0.0, 0.0
            else:
                lin, yaw = self._compute_cmd(dx, scale, dth)
                if publish_cmd_vel:
                    self._publish_twist(lin, yaw)
                self._pub_f32(self.pub_cmd_yaw, yaw)
                self._pub_f32(self.pub_cmd_value, lin)

            if (log_every_n > 0) and (i % log_every_n == 0):
                self.get_logger().info(
                    f"[RUN] dx={dx:+.1f}px dy={dy:+.1f}px scale={scale:+.3f} dth={dth:+.2f} "
                    f"-> cmd_yaw={yaw:+.3f}rad cmd_value={lin:+.3f} (hold={hold}/{hold_need})"
                )

            if (save_every_n > 0) and (i % save_every_n == 0):
                dbg = self._overlay(frame, corners, [
                    f"id={tid}",
                    f"dx={dx:+.1f}px scale={scale:+.3f} dth={dth:+.2f}",
                    f"v={lin:+.3f} w={yaw:+.3f}",
                    f"hold={hold}/{hold_need}",
                    "CONTINUOUS" if continuous else ("ONESHOT-INF" if search_until_done else "ONESHOT")
                ])
                self._save_snap(frame, dbg)

            if (not continuous) and (hold >= hold_need):
                self._publish_stop_burst()
                dbg = self._overlay(frame, corners, ["DONE", f"id={tid}"])
                raw, dbg_path = self._save_snap(frame, dbg)

                # ensure last cmd_vel is 0 even after IO/log delay
                self._publish_stop_burst()

                self.get_logger().info(f"[DONE] stop (raw={raw})")
                return True

            i += 1
            time.sleep(period)

        self._publish_stop_burst()
        with self._img_lock:
            frame = None if self._latest_bgr is None else self._latest_bgr.copy()

        if frame is not None:
            raw, _ = self._save_snap(frame, None)
            # ensure last cmd_vel is 0 even after IO/log delay
            self._publish_stop_burst()
            self.get_logger().warn(f"[FAIL] timeout or lost (raw={raw})")
        else:
            self.get_logger().warn("[FAIL] timeout or lost (no frame)")

        return False


def main():
    rclpy.init()
    node = ArucoMoveNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.try_shutdown()


if __name__ == '__main__':
    main()
