#!/usr/bin/env python3
"""
tracker5.py – YOLO detection + EKF tracking + detection_viewer-style fixed map.

Changes vs tracker4.py:
1) Do not draw stale active tracks on the video overlay if they have not been
   detected for more than STALE_BOX_DRAW_FRAMES consecutive frames. The track is
   kept alive internally; only the rectangle overlay is suppressed.
2) Add height-ratio based measurement gating using trusted box height:
   - ratio >= HEIGHT_RATIO_NORMAL_THRESHOLD      -> accept normally
   - HEIGHT_RATIO_SUSPICIOUS_THRESHOLD <= ratio < HEIGHT_RATIO_NORMAL_THRESHOLD
       -> accept, inflate measurement covariance, do NOT update trusted height
   - ratio < HEIGHT_RATIO_SUSPICIOUS_THRESHOLD   -> hard reject the pair

Usage:
    python3 tracker5.py [OPTIONS]
"""

import argparse
import math
import json
import os
from collections import deque
from dataclasses import dataclass, field
from typing import List, Optional, Tuple

import cv2
import numpy as np
import pandas as pd
import torch
from scipy.optimize import linear_sum_assignment
from scipy.spatial.transform import Rotation
from ultralytics import YOLO

from tracker3 import draw_raw_det


# ── Default paths ──────────────────────────────────────────────────────────────
DEFAULT_VIDEO_PATH       = "flight_004/001.avi"
DEFAULT_CSV_PATH         = "flight_004/001_data.csv"
DEFAULT_MODEL_PATH       = "yolov8s.pt"
DEFAULT_CALIBRATION_JSON = "calibration_640_480.json"

# ── Camera / geometry ──────────────────────────────────────────────────────────
CAMERA_HEIGHT_FALLBACK_M    = 5.0
CAMERA_MOUNT_PITCH_DOWN_DEG = 11.5
HORIZONTAL_FOV_DEG          = 55.8
VERTICAL_FOV_DEG            = 43.3

# ── Detector ───────────────────────────────────────────────────────────────────
DETECTION_CONFIDENCE = 0.50
DETECTOR_IMGSZ       = 640

# ── Tracker – base noise ───────────────────────────────────────────────────────
MAX_ASSOCIATION_DISTANCE_M = 5.0
MAHALANOBIS_GATE           = 12.25
MAX_MISSED_FRAMES          = 20
MIN_HITS_TO_CONFIRM        = 6
PROCESS_NOISE_STD_POS      = 0.75
PROCESS_NOISE_STD_VEL      = 2.0
BASE_MEAS_NOISE_M          = 2.0

PIXEL_STD_X = 5.0
PIXEL_STD_Y = 15.0

# ── Adaptive noise ─────────────────────────────────────────────────────────────
ANGULAR_VEL_LOW_DEG_S  = 5.0
ANGULAR_VEL_HIGH_DEG_S = 30.0
ANGULAR_VEL_MAX_SCALE  = 15.0

# ── Duplicate-track suppression ────────────────────────────────────────────────
NEW_TRACK_MIN_DIST_CALM_M  = 0.5
NEW_TRACK_MIN_DIST_NOISY_M = 2.0

# ── Overlay / trusted-height gating ────────────────────────────────────────────
STALE_BOX_DRAW_FRAMES             = 3
HEIGHT_RATIO_NORMAL_THRESHOLD     = 0.8
HEIGHT_RATIO_SUSPICIOUS_THRESHOLD = 0.2
HEIGHT_RATIO_R_INFLATION          = 9.0
TRUSTED_HEIGHT_EMA_ALPHA          = 0.5
MIN_VALID_BOX_HEIGHT_PX           = 10.0

# ── Map colours ────────────────────────────────────────────────────────────────
MAP_BG           = (30,  30,  30)
GRID_COL         = (60,  60,  60)
AXIS_COL         = (110, 110, 110)
TEXT_COL         = (180, 180, 180)
DRONE_COL        = (255, 160,   0)
ARROW_COL        = (255, 255,   0)
DRONE_TRAIL_COL  = (180,  80,   0)
TRK_TRAIL_OLD    = (0,   80, 200)
TRK_TRAIL_NEW    = (0,  220, 255)
TRK_DOT_CUR      = (50, 255,  50)
RAW_TRAIL_OLD    = (0,    0, 120)
RAW_TRAIL_NEW    = (0,  140, 255)
RAW_DOT_CUR      = (0,  200, 255)

MAP_TRAIL_LEN    = 200
MAP_MARGIN_PX    = 12


# ─────────────────────────────────────────────────────────────────────────────
# Utility
# ─────────────────────────────────────────────────────────────────────────────

def get_torch_device() -> str:
    if torch.backends.mps.is_available():
        return "mps"
    if torch.cuda.is_available():
        return "cuda"
    return "cpu"


def quat_to_rotmat_xyzw(qx, qy, qz, qw) -> np.ndarray:
    return Rotation.from_quat([qx, qy, qz, qw]).as_matrix()


def rotmat_to_euler_xyz_deg(R: np.ndarray) -> np.ndarray:
    return Rotation.from_matrix(R).as_euler("xyz", degrees=True)


def euler_xyz_deg_to_rotmat(e: np.ndarray) -> np.ndarray:
    return Rotation.from_euler("xyz", e, degrees=True).as_matrix()


def latlon_to_local_xy_m(
    lat_deg: np.ndarray, lon_deg: np.ndarray
) -> Tuple[np.ndarray, np.ndarray]:
    lat0 = np.deg2rad(lat_deg[0])
    lon0 = np.deg2rad(lon_deg[0])
    lat  = np.deg2rad(lat_deg)
    lon  = np.deg2rad(lon_deg)
    R    = 6_378_137.0
    return (lon - lon0) * math.cos(lat0) * R, (lat - lat0) * R


def quat_to_yaw_flu(qx: float, qy: float, qz: float, qw: float) -> float:
    return math.atan2(2.0 * (qw * qz + qx * qy),
                      1.0 - 2.0 * (qy * qy + qz * qz))


def color_for_id(tid: int) -> Tuple[int, int, int]:
    c = np.random.default_rng(tid).integers(64, 255, 3).tolist()
    return int(c[0]), int(c[1]), int(c[2])


def angular_vel_to_noise_scale(omega_deg_s: float) -> float:
    lo, hi = ANGULAR_VEL_LOW_DEG_S, ANGULAR_VEL_HIGH_DEG_S
    if omega_deg_s <= lo:
        return 1.0
    if omega_deg_s >= hi:
        return ANGULAR_VEL_MAX_SCALE
    t = (omega_deg_s - lo) / (hi - lo)
    return 1.0 + t * (ANGULAR_VEL_MAX_SCALE - 1.0)


def box_height(box: Optional[Tuple[float, float, float, float]]) -> Optional[float]:
    if box is None:
        return None
    return max(0.0, float(box[3] - box[1]))


# ─────────────────────────────────────────────────────────────────────────────
# Angular velocity source
# ─────────────────────────────────────────────────────────────────────────────

class CsvAngularVelocityReader:
    def __init__(self, gimbal: bool):
        self.angular_vel_deg_s: float = 0.0
        self.gimbal = gimbal

    def update(self, row: pd.Series, dt: float) -> float:
        if self.gimbal:
            wz = float(row["ang_vel_z"])
            self.angular_vel_deg_s = math.degrees(abs(wz))
        else:
            wx = float(row["ang_vel_x"])
            wy = float(row["ang_vel_y"])
            wz = float(row["ang_vel_z"])
            self.angular_vel_deg_s = math.degrees(math.sqrt(wx*wx + wy*wy + wz*wz))
        return angular_vel_to_noise_scale(self.angular_vel_deg_s)


# ─────────────────────────────────────────────────────────────────────────────
# Attitude smoother
# ─────────────────────────────────────────────────────────────────────────────

class AttitudeSmoother:
    def __init__(self, alpha: float = 0.25):
        self.alpha = alpha
        self.prev_euler: Optional[np.ndarray] = None

    def smooth(self, R_world_body: np.ndarray) -> np.ndarray:
        e = rotmat_to_euler_xyz_deg(R_world_body)
        if self.prev_euler is None:
            self.prev_euler = e
            return R_world_body
        e_adj = e.copy()
        for i in range(3):
            while e_adj[i] - self.prev_euler[i] > 180:
                e_adj[i] -= 360
            while e_adj[i] - self.prev_euler[i] < -180:
                e_adj[i] += 360
        e_s = self.alpha * e_adj + (1.0 - self.alpha) * self.prev_euler
        self.prev_euler = e_s
        return euler_xyz_deg_to_rotmat(e_s)


# ─────────────────────────────────────────────────────────────────────────────
# Camera calibration & undistortion
# ─────────────────────────────────────────────────────────────────────────────

def load_calibration(
    json_path: str, width: int, height: int,
    hfov_deg: float, vfov_deg: float,
) -> Tuple[np.ndarray, Optional[np.ndarray]]:
    if os.path.isfile(json_path):
        with open(json_path) as f:
            cal = json.load(f)
        K_data = (cal.get("camera_matrix") or cal.get("K")
                  or cal.get("intrinsic_matrix"))
        if K_data is None:
            raise KeyError(f"{json_path} missing camera matrix")
        K = np.array(K_data, dtype=np.float64)
        if K.shape != (3, 3):
            raise ValueError(f"camera_matrix in {json_path} must be 3×3")
        dist_data = (cal.get("dist_coeffs") or cal.get("distCoeffs")
                     or cal.get("distortion_coefficients")
                     or cal.get("distortion") or cal.get("dist"))
        dist = None if dist_data is None else np.array(dist_data, dtype=np.float64).ravel()
        print(f"[cal] loaded {json_path}\n[cal] K=\n{K}")
        if dist is None:
            print("[cal] no distortion coefficients; using pinhole")
        else:
            print(f"[cal] dist={np.round(dist, 5)}")
        return K, dist

    print(f"[cal] {json_path} not found – FOV pinhole fallback")
    fx = (width / 2.0) / math.tan(math.radians(hfov_deg) / 2.0)
    fy = (height / 2.0) / math.tan(math.radians(vfov_deg) / 2.0)
    K = np.array([[fx, 0, width / 2.0], [0, fy, height / 2.0], [0, 0, 1]], dtype=np.float64)
    return K, None


def build_undistort_maps(
    K: np.ndarray, dist: Optional[np.ndarray], width: int, height: int,
) -> Tuple[Optional[np.ndarray], Optional[np.ndarray], np.ndarray]:
    if dist is None or np.allclose(dist, 0):
        return None, None, K
    K_new, _ = cv2.getOptimalNewCameraMatrix(K, dist, (width, height), 0, (width, height))
    m1, m2 = cv2.initUndistortRectifyMap(K, dist, None, K_new, (width, height), cv2.CV_32FC1)
    print(f"[cal] undistort maps ready  K_new fx={K_new[0,0]:.1f} fy={K_new[1,1]:.1f}")
    return m1, m2, K_new


# ─────────────────────────────────────────────────────────────────────────────
# Camera-body geometry
# ─────────────────────────────────────────────────────────────────────────────

def camera_to_body_rotation() -> np.ndarray:
    R_base = np.array([[0, 0, 1], [-1, 0, 0], [0, -1, 0]], dtype=np.float64)
    R_pitch = Rotation.from_euler("y", math.radians(CAMERA_MOUNT_PITCH_DOWN_DEG)).as_matrix()
    return R_pitch @ R_base


# ─────────────────────────────────────────────────────────────────────────────
# Ground projection & measurement covariance
# ─────────────────────────────────────────────────────────────────────────────

def project_pixel_to_ground(
    u: float, v: float,
    K: np.ndarray,
    R_world_body: np.ndarray,
    cam_world_pos: np.ndarray,
    ground_z: float = 0.0,
) -> Optional[np.ndarray]:
    ray_cam = np.linalg.inv(K) @ np.array([u, v, 1.0], dtype=np.float64)
    ray_cam /= np.linalg.norm(ray_cam)
    ray_body = camera_to_body_rotation() @ ray_cam
    ray_world = R_world_body @ ray_body
    dz = ray_world[2]
    if abs(dz) < 1e-9:
        return None
    t = (ground_z - cam_world_pos[2]) / dz
    if t <= 0:
        return None
    return (cam_world_pos + t * ray_world)[:2]


def estimate_measurement_covariance(
    box: Tuple[float, float, float, float],
    K: np.ndarray,
    R_world_body: np.ndarray,
    cam_world_pos: np.ndarray,
    noise_scale: float = 1.0,
    pixel_std_x: float = PIXEL_STD_X,
    pixel_std_y: float = PIXEL_STD_Y,
) -> np.ndarray:
    x1, y1, x2, y2 = box
    u, v = 0.5 * (x1 + x2), y2
    p0 = project_pixel_to_ground(u, v, K, R_world_body, cam_world_pos)
    if p0 is None:
        return np.eye(2) * 100.0 * noise_scale
    pu = project_pixel_to_ground(u + pixel_std_x, v, K, R_world_body, cam_world_pos)
    pv = project_pixel_to_ground(u, v + pixel_std_y, K, R_world_body, cam_world_pos)
    if pu is None or pv is None:
        return np.eye(2) * 25.0 * noise_scale
    J = np.column_stack([(pu - p0) / pixel_std_x, (pv - p0) / pixel_std_y])
    S = J @ np.diag([pixel_std_x**2, pixel_std_y**2]) @ J.T
    S += np.eye(2) * BASE_MEAS_NOISE_M**2
    return S * noise_scale


# ─────────────────────────────────────────────────────────────────────────────
# Data classes
# ─────────────────────────────────────────────────────────────────────────────

@dataclass
class Detection2DGround:
    world_xy: np.ndarray
    meas_cov: np.ndarray
    conf: float
    image_box: Tuple[float, float, float, float]


@dataclass
class Track:
    track_id: int
    x: np.ndarray
    P: np.ndarray
    age: int = 0
    hits: int = 0
    missed: int = 0
    confirmed: bool = False
    last_box: Optional[Tuple[float, float, float, float]] = None
    trusted_box: Optional[Tuple[float, float, float, float]] = None
    trusted_height_px: Optional[float] = None
    history: List[np.ndarray] = field(default_factory=list)
    raw_history: List[np.ndarray] = field(default_factory=list)

    def predict(self, dt: float, noise_scale: float = 1.0) -> None:
        F = np.array([[1,0,dt,0], [0,1,0,dt], [0,0,1,0], [0,0,0,1]], dtype=np.float64)
        q_pos = PROCESS_NOISE_STD_POS ** 2
        q_vel = PROCESS_NOISE_STD_VEL ** 2
        Q = np.diag([q_pos*dt*dt, q_pos*dt*dt, q_vel*dt, q_vel*dt]).astype(np.float64)
        self.x = F @ self.x
        self.P = F @ self.P @ F.T + Q
        self.age += 1
        self.history.append(self.x[:2].copy())
        if len(self.history) > 300:
            self.history = self.history[-300:]

    def _H(self):
        return np.array([[1,0,0,0], [0,1,0,0]], dtype=np.float64)

    def innovation(self, z, R):
        H = self._H()
        return z - H @ self.x, H @ self.P @ H.T + R, H

    def mahalanobis_distance(self, z, R) -> float:
        y, S, _ = self.innovation(z, R)
        try:
            return float(y.T @ np.linalg.inv(S) @ y)
        except Exception:
            return 1e9

    def euclidean_distance(self, z) -> float:
        return float(np.linalg.norm(self.x[:2] - z))

    def update(self, det: Detection2DGround, R: Optional[np.ndarray] = None,
               update_trusted_height: bool = True) -> None:
        R_use = det.meas_cov if R is None else R
        y, S, H = self.innovation(det.world_xy, R_use)
        Kk = self.P @ H.T @ np.linalg.inv(S)
        self.x = self.x + Kk @ y
        self.P = (np.eye(4) - Kk @ H) @ self.P
        self.hits += 1
        self.missed = 0
        self.last_box = det.image_box
        if self.hits >= MIN_HITS_TO_CONFIRM:
            self.confirmed = True

        h_det = box_height(det.image_box)
        if update_trusted_height and h_det is not None and h_det >= MIN_VALID_BOX_HEIGHT_PX:
            self.trusted_box = det.image_box
            if self.trusted_height_px is None:
                self.trusted_height_px = h_det
            else:
                a = TRUSTED_HEIGHT_EMA_ALPHA
                self.trusted_height_px = (1.0 - a) * self.trusted_height_px + a * h_det

        self.raw_history.append(det.world_xy.copy())
        if len(self.raw_history) > 300:
            self.raw_history = self.raw_history[-300:]

    def mark_missed(self):
        self.missed += 1

    def should_draw_box(self, stale_after_frames: int = STALE_BOX_DRAW_FRAMES) -> bool:
        return self.confirmed and self.last_box is not None and self.missed <= stale_after_frames


# ─────────────────────────────────────────────────────────────────────────────
# Multi-object tracker
# ─────────────────────────────────────────────────────────────────────────────

class MultiObjectTracker:
    def __init__(self):
        self.tracks: List[Track] = []
        self.next_id: int = 1

    def _near_confirmed(self, world_xy: np.ndarray, min_dist: float) -> bool:
        for tr in self.tracks:
            if tr.confirmed and np.linalg.norm(tr.x[:2] - world_xy) < min_dist:
                return True
        return False

    def _spawn(self, det: Detection2DGround, min_dist: float) -> None:
        if self._near_confirmed(det.world_xy, min_dist):
            return
        x0 = np.array([det.world_xy[0], det.world_xy[1], 0.0, 0.0])
        P0 = np.diag([4.0, 4.0, 25.0, 25.0]).astype(np.float64)
        h0 = box_height(det.image_box)
        self.tracks.append(Track(
            track_id=self.next_id,
            x=x0,
            P=P0,
            age=1,
            hits=1,
            confirmed=(MIN_HITS_TO_CONFIRM <= 1),
            last_box=det.image_box,
            trusted_box=det.image_box if h0 is not None and h0 >= MIN_VALID_BOX_HEIGHT_PX else None,
            trusted_height_px=h0 if h0 is not None and h0 >= MIN_VALID_BOX_HEIGHT_PX else None,
        ))
        self.next_id += 1

    def _height_ratio_action(self, tr: Track, det: Detection2DGround) -> Tuple[str, np.ndarray]:
        R = det.meas_cov.copy()
        h_det = box_height(det.image_box)
        h_ref = tr.trusted_height_px

        if h_det is None or h_det < MIN_VALID_BOX_HEIGHT_PX or h_ref is None or h_ref < MIN_VALID_BOX_HEIGHT_PX:
            return "normal", R

        ratio = h_det / max(h_ref, 1e-6)
        if ratio >= HEIGHT_RATIO_NORMAL_THRESHOLD:
            return "normal", R
        if ratio >= HEIGHT_RATIO_SUSPICIOUS_THRESHOLD:
            return "suspicious", R * HEIGHT_RATIO_R_INFLATION
        return "reject", R

    def step(
        self,
        detections: List[Detection2DGround],
        dt: float,
        noise_scale: float = 1.0,
    ) -> None:
        for tr in self.tracks:
            tr.predict(dt, noise_scale=noise_scale)

        dup_dist = NEW_TRACK_MIN_DIST_NOISY_M if noise_scale > 1.5 else NEW_TRACK_MIN_DIST_CALM_M

        if not self.tracks:
            for det in detections:
                self._spawn(det, min_dist=0.0)
            return

        if not detections:
            for tr in self.tracks:
                tr.mark_missed()
            self._prune()
            return

        cost = np.full((len(self.tracks), len(detections)), 1e6)
        pair_actions = {}
        pair_R = {}

        for i, tr in enumerate(self.tracks):
            for j, det in enumerate(detections):
                action, R_use = self._height_ratio_action(tr, det)
                pair_actions[(i, j)] = action
                pair_R[(i, j)] = R_use
                if action == "reject":
                    continue

                md = tr.mahalanobis_distance(det.world_xy, R_use)
                ed = tr.euclidean_distance(det.world_xy)
                if md <= MAHALANOBIS_GATE and ed <= MAX_ASSOCIATION_DISTANCE_M:
                    cost[i, j] = md

        rows, cols = linear_sum_assignment(cost)
        matched_t, matched_d = set(), set()
        for r, c in zip(rows, cols):
            if cost[r, c] >= 1e5:
                continue
            action = pair_actions[(r, c)]
            R_use = pair_R[(r, c)]
            update_trusted_height = (action == "normal")
            self.tracks[r].update(detections[c], R=R_use,
                                  update_trusted_height=update_trusted_height)
            matched_t.add(r)
            matched_d.add(c)

        for i, tr in enumerate(self.tracks):
            if i not in matched_t:
                tr.mark_missed()

        for j, det in enumerate(detections):
            if j not in matched_d:
                self._spawn(det, min_dist=dup_dist)

        self._prune()

    def _prune(self):
        self.tracks = [t for t in self.tracks if t.missed <= MAX_MISSED_FRAMES]


# ─────────────────────────────────────────────────────────────────────────────
# Detector
# ─────────────────────────────────────────────────────────────────────────────

class PersonDetector:
    def __init__(self, model_path: str):
        self.device = get_torch_device()
        self.model = YOLO(model_path)
        self.model.to(self.device)
        print(f"[detector] device={self.device}  model={model_path}")

    def detect(self, frame) -> List[Tuple[float, float, float, float, float]]:
        results = self.model(
            frame, conf=DETECTION_CONFIDENCE, imgsz=DETECTOR_IMGSZ,
            device=self.device, verbose=False,
        )
        out = []
        for r in results:
            if r.boxes is None:
                continue
            for box, conf, cls in zip(r.boxes.xyxy.cpu().numpy(),
                                      r.boxes.conf.cpu().numpy(),
                                      r.boxes.cls.cpu().numpy()):
                if int(cls) != 0:
                    continue
                out.append((*box.astype(float).tolist(), float(conf)))
        return out


# ─────────────────────────────────────────────────────────────────────────────
# Telemetry
# ─────────────────────────────────────────────────────────────────────────────

def load_telemetry(csv_path: str) -> pd.DataFrame:
    df = pd.read_csv(csv_path)
    df["timestamp"] = df["stamp_sec"].astype(np.float64) + df["stamp_nanosec"].astype(np.float64) * 1e-9
    lat = df["lat_deg_e7"].astype(np.float64).values / 1e7
    lon = df["lon_deg_e7"].astype(np.float64).values / 1e7
    df["gps_x_m"], df["gps_y_m"] = latlon_to_local_xy_m(lat, lon)
    return df


def get_camera_world_pose(row: pd.Series, gimbal: bool) -> Tuple[np.ndarray, np.ndarray]:
    R_raw = quat_to_rotmat_xyzw(
        float(row["quat_x"]), float(row["quat_y"]),
        float(row["quat_z"]), float(row["quat_w"]),
    )

    if gimbal:
        yaw = quat_to_yaw_flu(
            float(row["quat_x"]), float(row["quat_y"]),
            float(row["quat_z"]), float(row["quat_w"]),
        )
        R = Rotation.from_euler("z", yaw).as_matrix()
    else:
        R = R_raw

    if int(row["odom_valid"]) == 1:
        x, y = float(row["pos_x"]), float(row["pos_y"])
        z = float(row["agl_m"])
    else:
        x, y = float(row["gps_x_m"]), float(row["gps_y_m"])
        z = float(row["agl_m"]) if float(row["agl_m"]) > 0.1 else CAMERA_HEIGHT_FALLBACK_M

    if abs(z) < 1e-6:
        z = CAMERA_HEIGHT_FALLBACK_M

    return np.array([x, y, z]), R


# ─────────────────────────────────────────────────────────────────────────────
# Video drawing helpers
# ─────────────────────────────────────────────────────────────────────────────

def draw_track(frame, tr: Track):
    if tr.last_box is None:
        return
    x1, y1, x2, y2 = map(int, tr.last_box)
    c = color_for_id(tr.track_id)
    cv2.rectangle(frame, (x1, y1), (x2, y2), c, 2)
    cv2.putText(frame, f"ID {tr.track_id}", (x1, max(20, y1 - 26)),
                cv2.FONT_HERSHEY_SIMPLEX, 0.65, c, 2, cv2.LINE_AA)


# ─────────────────────────────────────────────────────────────────────────────
# Map drawing helpers
# ─────────────────────────────────────────────────────────────────────────────

def draw_drone_icon(img, px, py, yaw_rad, radius=6):
    cv2.circle(img, (px, py), radius, DRONE_COL, -1, cv2.LINE_AA)
    cv2.circle(img, (px, py), radius, (255, 220, 100), 1, cv2.LINE_AA)
    img_angle = -yaw_rad
    arrow_len = radius + 14
    ex = int(px + arrow_len * math.cos(img_angle))
    ey = int(py + arrow_len * math.sin(img_angle))
    cv2.arrowedLine(img, (px, py), (ex, ey), ARROW_COL, 2, cv2.LINE_AA, tipLength=0.4)
    cv2.putText(img, "UAV", (px + radius + 2, py - radius),
                cv2.FONT_HERSHEY_SIMPLEX, 0.32, DRONE_COL, 1, cv2.LINE_AA)


def make_map(
    map_size_px: int,
    map_range_m: int,
    drone_pos: Optional[Tuple[float, float]],
    drone_yaw: Optional[float],
    drone_trail: deque,
    track_data: List[Tuple[int, np.ndarray, List[np.ndarray], List[np.ndarray]]],
    raw_current_pts: List[Tuple[float, float]],
    trail_len: int,
) -> np.ndarray:
    img = np.full((map_size_px, map_size_px, 3), MAP_BG, dtype=np.uint8)
    cx = cy = map_size_px // 2
    scale = map_size_px / (2.0 * map_range_m)

    def w2p(wx, wy):
        return (int(cx + wx * scale), int(cy - wy * scale))

    for v in range(-map_range_m, map_range_m + 1, 1):
        gx, _ = w2p(v, 0)
        _, gy = w2p(0, v)
        col = AXIS_COL if v == 0 else GRID_COL
        thick = 2 if v == 0 else 1
        cv2.line(img, (gx, 0), (gx, map_size_px - 1), col, thick)
        cv2.line(img, (0, gy), (map_size_px - 1, gy), col, thick)
        if v != 0 and v % 10 == 0:
            cv2.putText(img, f"{v}m", (gx + 3, cy - 4),
                        cv2.FONT_HERSHEY_SIMPLEX, 0.32, TEXT_COL, 1, cv2.LINE_AA)
            cv2.putText(img, f"{v}m", (cx + 4, gy + 10),
                        cv2.FONT_HERSHEY_SIMPLEX, 0.32, TEXT_COL, 1, cv2.LINE_AA)

    cv2.drawMarker(img, (cx, cy), (90, 90, 90), cv2.MARKER_CROSS, 12, 1, cv2.LINE_AA)

    trail_list = list(drone_trail)
    n = len(trail_list)
    for i in range(1, n):
        t = i / max(n - 1, 1)
        col = (0, int(60 + t * 100), int(180 + t * 75))
        wx0, wy0 = trail_list[i - 1]
        wx1, wy1 = trail_list[i]
        if (abs(wx0) <= map_range_m and abs(wy0) <= map_range_m and
                abs(wx1) <= map_range_m and abs(wy1) <= map_range_m):
            cv2.line(img, w2p(wx0, wy0), w2p(wx1, wy1), col, 1, cv2.LINE_AA)

    for tid, ekf_xy, ekf_hist, raw_hist in track_data:
        raw_list = raw_hist[-trail_len:]
        rn = len(raw_list)
        for i, pt in enumerate(raw_list):
            wx, wy = float(pt[0]), float(pt[1])
            if abs(wx) > map_range_m or abs(wy) > map_range_m:
                continue
            t = i / max(rn - 1, 1)
            col = tuple(int(RAW_TRAIL_OLD[c] + t * (RAW_TRAIL_NEW[c] - RAW_TRAIL_OLD[c])) for c in range(3))
            cv2.circle(img, w2p(wx, wy), max(1, int(1 + t * 2)), col, -1, cv2.LINE_AA)

    for tid, ekf_xy, ekf_hist, raw_hist in track_data:
        hist = ekf_hist[-trail_len:]
        hn = len(hist)
        pts_px = []
        for pt in hist:
            wx, wy = float(pt[0]), float(pt[1])
            pts_px.append(w2p(wx, wy) if abs(wx) <= map_range_m and abs(wy) <= map_range_m else None)
        for i in range(1, len(pts_px)):
            if pts_px[i - 1] is not None and pts_px[i] is not None:
                t = i / max(hn - 1, 1)
                col = tuple(int(TRK_TRAIL_OLD[c] + t * (TRK_TRAIL_NEW[c] - TRK_TRAIL_OLD[c])) for c in range(3))
                cv2.line(img, pts_px[i - 1], pts_px[i], col, 1, cv2.LINE_AA)

    for tid, ekf_xy, ekf_hist, raw_hist in track_data:
        wx, wy = float(ekf_xy[0]), float(ekf_xy[1])
        if abs(wx) > map_range_m or abs(wy) > map_range_m:
            continue
        c = color_for_id(tid)
        px, py = w2p(wx, wy)
        cv2.circle(img, (px, py), 5, c, -1, cv2.LINE_AA)
        cv2.circle(img, (px, py), 5, (255, 255, 255), 1, cv2.LINE_AA)
        cv2.putText(img, f"#{tid}", (px + 6, py - 5),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.38, c, 1, cv2.LINE_AA)

    """for wx, wy in raw_current_pts:
        if abs(wx) > map_range_m or abs(wy) > map_range_m:
            continue
        px, py = w2p(wx, wy)
        cv2.circle(img, (px, py), 3, RAW_DOT_CUR, -1, cv2.LINE_AA)
        cv2.circle(img, (px, py), 3, (255, 255, 255), 1, cv2.LINE_AA)
    """ # Optional: draw current raw detections as dots

    if drone_pos is not None:
        dx, dy = drone_pos
        if abs(dx) <= map_range_m and abs(dy) <= map_range_m:
            draw_drone_icon(img, *w2p(dx, dy), drone_yaw if drone_yaw is not None else 0.0)

    cv2.putText(img, "+X (E)", (map_size_px - 54, cy - 5),
                cv2.FONT_HERSHEY_SIMPLEX, 0.35, TEXT_COL, 1)
    cv2.putText(img, "+Y (N)", (cx + 4, 14),
                cv2.FONT_HERSHEY_SIMPLEX, 0.35, TEXT_COL, 1)
    cv2.putText(img, "2D MAP (EKF origin, m)", (6, map_size_px - 8),
                cv2.FONT_HERSHEY_SIMPLEX, 0.38, (110, 110, 110), 1)
    return img


# ─────────────────────────────────────────────────────────────────────────────
# Main
# ─────────────────────────────────────────────────────────────────────────────

def parse_args():
    ap = argparse.ArgumentParser(description="tracker5 – YOLO + EKF + fixed global map")
    ap.add_argument("--video", default=DEFAULT_VIDEO_PATH)
    ap.add_argument("--csv", default=DEFAULT_CSV_PATH)
    ap.add_argument("--model", default=DEFAULT_MODEL_PATH)
    ap.add_argument("--cal", default=DEFAULT_CALIBRATION_JSON)
    ap.add_argument("--out-video", default=None,
                    help="Output video path (default: <stem>_tracked5.mp4)")
    ap.add_argument("--out-csv", default=None,
                    help="Output tracks CSV (default: <stem>_tracks5.csv)")
    ap.add_argument("--map-range", type=int, default=20,
                    help="Map half-size in metres (fixed axes ±N). Default: 20")
    ap.add_argument("--trail", type=int, default=200,
                    help="Trail length for map dots. Default: 200")
    ap.add_argument("--live-preview", action="store_true",
                    help="Display a live OpenCV window while processing.")
    ap.add_argument("--no-gimbal", action="store_true",
                    help="Use raw telemetry pitch+roll instead of zeroing them.")
    return ap.parse_args()


def main():
    args = parse_args()

    gimbal_mode = not args.no_gimbal
    print(f"[config] gimbal={'ON' if gimbal_mode else 'OFF (raw pitch+roll)'}  angular_vel=csv  live_preview={args.live_preview}")
    print(f"[config] map_range=±{args.map_range}m  trail={args.trail}")
    print(f"[config] stale_box_draw_frames={STALE_BOX_DRAW_FRAMES}  height_ratio_normal={HEIGHT_RATIO_NORMAL_THRESHOLD:.2f}  height_ratio_suspicious={HEIGHT_RATIO_SUSPICIOUS_THRESHOLD:.2f}  suspicious_Rx={HEIGHT_RATIO_R_INFLATION:.1f}")

    stem = os.path.splitext(args.video)[0]
    out_video_path = args.out_video or f"{stem}_tracked5.mp4"
    out_csv_path = args.out_csv or f"{stem}_tracks5.csv"

    telemetry = load_telemetry(args.csv)
    print(telemetry[["gps_x_m", "gps_y_m", "pos_x", "pos_y", "pos_z", "odom_valid"]].head(10))

    cap = cv2.VideoCapture(args.video)
    if not cap.isOpened():
        raise RuntimeError(f"Cannot open {args.video}")

    n_vid = int(cap.get(cv2.CAP_PROP_FRAME_COUNT))
    W = int(cap.get(cv2.CAP_PROP_FRAME_WIDTH))
    H = int(cap.get(cv2.CAP_PROP_FRAME_HEIGHT))
    fps = float(cap.get(cv2.CAP_PROP_FPS))
    N = min(n_vid, len(telemetry))
    print(f"[video] {W}×{H}  {fps:.2f} fps  {N} usable frames")

    map_px = H

    K_raw, dist = load_calibration(args.cal, W, H, HORIZONTAL_FOV_DEG, VERTICAL_FOV_DEG)
    map1, map2, K = build_undistort_maps(K_raw, dist, W, H)

    out_w = W + MAP_MARGIN_PX + map_px
    writer = cv2.VideoWriter(out_video_path, cv2.VideoWriter_fourcc(*"mp4v"), fps, (out_w, H))
    print(f"[output] video → {out_video_path}")
    print(f"[output] csv   → {out_csv_path}")

    if args.live_preview:
        cv2.namedWindow("tracker5", cv2.WINDOW_NORMAL)
        cv2.resizeWindow("tracker5", out_w, H)

    detector = PersonDetector(args.model)
    mot = MultiObjectTracker()
    smoother = AttitudeSmoother(alpha=0.25)
    ang_vel_src = CsvAngularVelocityReader(gimbal=gimbal_mode)

    trail_len = args.trail
    drone_trail = deque(maxlen=trail_len)

    output_rows = []
    prev_time = None
    paused = False
    frame_idx = 0
    canvas = None

    while frame_idx < N:
        if not paused:
            ok, frame_raw = cap.read()
            if not ok:
                break

            frame = cv2.remap(frame_raw, map1, map2, cv2.INTER_LINEAR) if map1 is not None else frame_raw

            row = telemetry.iloc[frame_idx]
            timestamp = float(row["timestamp"])
            dt = (1.0 / fps if prev_time is None else max(1e-3, timestamp - prev_time))
            prev_time = timestamp

            noise_scale = ang_vel_src.update(row, dt)
            omega_deg_s = ang_vel_src.angular_vel_deg_s

            cam_pos, R_raw = get_camera_world_pose(row, gimbal=gimbal_mode)
            R_smooth = smoother.smooth(R_raw)

            drone_yaw = quat_to_yaw_flu(
                float(row["quat_x"]), float(row["quat_y"]),
                float(row["quat_z"]), float(row["quat_w"]),
            )

            raw_dets = detector.detect(frame)
            #for x1, y1, x2, y2, cf in raw_dets:
            #    draw_raw_det(frame, (x1, y1, x2, y2), cf)

            ground_dets = []
            raw_current_pts = []
            n_fail = 0
            for x1, y1, x2, y2, cf in raw_dets:
                wxy = project_pixel_to_ground(0.5 * (x1 + x2), y2, K, R_smooth, cam_pos)
                if wxy is None:
                    n_fail += 1
                    continue

                raw_current_pts.append((float(wxy[0]), float(wxy[1])))
                range_m = np.linalg.norm(wxy - cam_pos[:2])
                cv2.putText(frame, f"{range_m:.1f}m", (int(x1), int(y2) + 20),
                            cv2.FONT_HERSHEY_SIMPLEX, 0.65, (0, 255, 255), 2, cv2.LINE_AA)

                cov = estimate_measurement_covariance((x1, y1, x2, y2), K, R_smooth, cam_pos,
                                                      noise_scale=noise_scale)
                ground_dets.append(Detection2DGround(wxy, cov, cf, (x1, y1, x2, y2)))

            mot.step(ground_dets, dt, noise_scale=noise_scale)

            for tr in mot.tracks:
                if tr.should_draw_box(STALE_BOX_DRAW_FRAMES):
                    draw_track(frame, tr)
                if tr.confirmed:
                    output_rows.append({
                        "frame_idx": frame_idx,
                        "timestamp_sec": timestamp,
                        "track_id": tr.track_id,
                        "world_x_m": float(tr.x[0]),
                        "world_y_m": float(tr.x[1]),
                        "world_vx_mps": float(tr.x[2]),
                        "world_vy_mps": float(tr.x[3]),
                        "noise_scale": round(noise_scale, 2),
                        "omega_deg_s": round(omega_deg_s, 2),
                        "cam_x_m": float(cam_pos[0]),
                        "cam_y_m": float(cam_pos[1]),
                        "cam_z_m": float(cam_pos[2]),
                    })

            hud_av = "csv"
            hud = (f"raw={len(raw_dets)} proj={len(ground_dets)} fail={n_fail} trk={len(mot.tracks)}  "
                   f"omega={omega_deg_s:.1f} deg/s [{hud_av}]  QRx{noise_scale:.1f}  "
                   f"gimbal={'ON' if gimbal_mode else 'OFF'}")
            hud_col = (0, 80, 255) if noise_scale > 3 else (0, 255, 255)
            cv2.putText(frame, hud, (12, 28), cv2.FONT_HERSHEY_SIMPLEX, 0.52, hud_col, 2, cv2.LINE_AA)

            cv2.putText(frame, f"frame {frame_idx}  yaw {math.degrees(drone_yaw):.1f}°",
                        (12, 52), cv2.FONT_HERSHEY_SIMPLEX, 0.50, (180, 255, 180), 1, cv2.LINE_AA)
            cv2.putText(frame, f"uav  x={cam_pos[0]:.2f}m  y={cam_pos[1]:.2f}m  z={cam_pos[2]:.2f}m",
                        (12, 74), cv2.FONT_HERSHEY_SIMPLEX, 0.50, (180, 255, 180), 1, cv2.LINE_AA)

            drone_trail.append((float(cam_pos[0]), float(cam_pos[1])))

            track_data = [
                (tr.track_id, tr.x[:2].copy(), list(tr.history), list(tr.raw_history))
                for tr in mot.tracks if tr.confirmed
            ]

            map_img = make_map(
                map_size_px=map_px,
                map_range_m=args.map_range,
                drone_pos=(float(cam_pos[0]), float(cam_pos[1])),
                drone_yaw=drone_yaw,
                drone_trail=drone_trail,
                track_data=track_data,
                raw_current_pts=raw_current_pts,
                trail_len=trail_len,
            )
            cv2.putText(map_img, f"trail {trail_len}", (6, map_px - 22),
                        cv2.FONT_HERSHEY_SIMPLEX, 0.38, (110, 110, 110), 1)

            canvas = np.zeros((H, out_w, 3), dtype=np.uint8)
            canvas[:, :W] = frame
            x0 = W + MAP_MARGIN_PX
            y0 = max(0, (H - map_px) // 2)
            canvas[y0:y0 + map_px, x0:x0 + map_px] = map_img

            writer.write(canvas)

            if frame_idx % 50 == 0:
                print(f"  [{frame_idx:5d}/{N}] raw={len(raw_dets):2d} proj={len(ground_dets):2d} "
                      f"fail={n_fail} trk={len(mot.tracks):2d}  omega={omega_deg_s:6.1f} deg/s [{hud_av}]  "
                      f"noisex={noise_scale:.2f}")

        if args.live_preview and canvas is not None:
            if not paused:
                cv2.imshow("tracker5", canvas)
            delay = 1 if paused else max(1, int(1000 / fps))
            key = cv2.waitKey(delay) & 0xFF

            if key in (ord('q'), 27):
                break
            elif key == ord(' '):
                paused = not paused
            elif key == ord('s'):
                fname = f"tracker5_frame_{frame_idx:06d}.png"
                cv2.imwrite(fname, canvas)
                print(f"Saved {fname}")
            elif key == ord('='):
                trail_len += 50
                drone_trail = deque(drone_trail, maxlen=trail_len)
                print(f"trail = {trail_len}")
            elif key == ord('-'):
                trail_len = max(10, trail_len - 50)
                drone_trail = deque(drone_trail, maxlen=trail_len)
                print(f"trail = {trail_len}")

        if not paused:
            frame_idx += 1

    cap.release()
    writer.release()
    if args.live_preview:
        cv2.destroyAllWindows()

    pd.DataFrame(output_rows).to_csv(out_csv_path, index=False)
    print(f"\nDone.\nSaved video : {out_video_path}\nSaved tracks: {out_csv_path}")


if __name__ == "__main__":
    main()
