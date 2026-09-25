#!/usr/bin/env python3

"""
Gazebo Fossen 3-DOF Recorder with x_pose and y_pose
====================================================

Records for Fossen 3-DOF model validation and trajectory plotting:

- WAM-V pose from Gazebo transport topic
- Local pose relative to first WAM-V pose
- x_pose and y_pose for direct trajectory plotting
- Body-frame velocities:
    u, v, w
    p, q, r
    r_yaw
- Body-frame accelerations from differentiated velocities:
    u_dot, v_dot, r_dot
- IMU acceleration and angular velocity from Gazebo IMU topic
- Thruster commands T_L and T_R from Gazebo thruster topics
- Segment label from ROS topic /usv/current_segment, published by guide.py

Default topics:
    Pose:
        /world/sydney_regatta/pose/info

    IMU:
        /world/sydney_regatta/model/wamv/link/wamv/imu_wamv_link/sensor/imu_wamv_sensor/imu

    Thrusters:
        /wamv/thrusters/left/thrust
        /wamv/thrusters/right/thrust

    Segment:
        /usv/current_segment

Important:
- The WAM-V pose entity is "wamv", not "base_link".
- x_pose and y_pose are set equal to x_local and y_local.
- If you prefer world coordinates, change x_pose/y_pose to global_x/global_y.
"""

import argparse
import csv
import math
import os
import re
import shutil
import subprocess
import threading
import time
import sys


class GzFossenRecorder:
    def __init__(
        self,
        topic="/world/sydney_regatta/pose/info",
        target_entity="wamv",
        imu_topic=(
            "/world/sydney_regatta/model/wamv/link/wamv/"
            "imu_wamv_link/sensor/imu_wamv_sensor/imu"
        ),
        left_thrust_topic="/wamv/thrusters/left/thrust",
        right_thrust_topic="/wamv/thrusters/right/thrust",
        segment_topic="/usv/current_segment",
        velocity_alpha=0.35,
        accel_alpha=0.25,
        min_dt=0.001,
        max_dt=1.0
    ):
        self.topic = topic
        self.target_entity = target_entity
        self.imu_topic = imu_topic
        self.left_thrust_topic = left_thrust_topic
        self.right_thrust_topic = right_thrust_topic
        self.segment_topic = segment_topic

        # ------------------------------------------------------------
        # Base frame / origin
        # ------------------------------------------------------------
        self.base_x = None
        self.base_y = None
        self.base_z = None
        self.base_yaw = None
        self.base_sim_time = None

        # ------------------------------------------------------------
        # Current local state
        # ------------------------------------------------------------
        self.local_x = 0.0
        self.local_y = 0.0
        self.local_z = 0.0
        self.local_yaw = 0.0

        # ------------------------------------------------------------
        # Current global state
        # ------------------------------------------------------------
        self.global_x = 0.0
        self.global_y = 0.0
        self.global_z = 0.0
        self.global_yaw = 0.0

        # Current Gazebo timestamp
        self.current_sim_time = None

        # Warning state
        self.warned_no_target = False
        self.warned_no_imu = False
        self.warned_no_thrust = False
        self.warned_no_segment = False

        # ------------------------------------------------------------
        # Velocity estimation
        # ------------------------------------------------------------
        alpha_vel = float(velocity_alpha)
        if alpha_vel <= 0.0:
            alpha_vel = 1e-6
        self.vel_alpha = min(1.0, alpha_vel)

        self.min_dt = float(min_dt)
        self.max_dt = float(max_dt)

        self.prev_stamp = None
        self.prev_x = None
        self.prev_y = None
        self.prev_z = None
        self.prev_yaw = None
        self.prev_quat = None

        self.dt = None

        # World-frame linear velocity
        self.vx_world = 0.0
        self.vy_world = 0.0
        self.vz_world = 0.0
        self.speed_world = 0.0

        # Body-frame linear velocity: u, v, w
        self.u = 0.0
        self.v = 0.0
        self.w = 0.0

        # Body-frame angular velocity: p, q, r
        self.p = 0.0
        self.q = 0.0
        self.r = 0.0

        # Yaw rate from heading derivative
        self.r_yaw = 0.0

        # Horizontal speed
        self.speed = 0.0

        self.velocity_initialized = False

        # ------------------------------------------------------------
        # Acceleration estimation: u_dot, v_dot, r_dot
        # ------------------------------------------------------------
        alpha_acc = float(accel_alpha)
        if alpha_acc <= 0.0:
            alpha_acc = 1e-6
        self.accel_alpha = min(1.0, alpha_acc)

        self.u_dot = 0.0
        self.v_dot = 0.0
        self.r_dot = 0.0

        self.prev_u_accel = None
        self.prev_v_accel = None
        self.prev_r_accel = None
        self.prev_accel_stamp = None

        self.acceleration_initialized = False

        # ------------------------------------------------------------
        # IMU state
        # ------------------------------------------------------------
        self.imu_lock = threading.Lock()

        self.ax_imu = 0.0
        self.ay_imu = 0.0
        self.az_imu = 0.0

        self.wx_imu = 0.0
        self.wy_imu = 0.0
        self.wz_imu = 0.0

        self.imu_stamp = None
        self.imu_message_count = 0

        # ------------------------------------------------------------
        # Thrust state
        # ------------------------------------------------------------
        self.thrust_lock = threading.Lock()

        self.T_L = 0.0
        self.T_R = 0.0

        self.left_thrust_count = 0
        self.right_thrust_count = 0

        # ------------------------------------------------------------
        # Segment state
        # ------------------------------------------------------------
        self.segment_lock = threading.Lock()

        self.segment_label = "unknown"
        self.segment_message_count = 0

    # ------------------------------------------------------------------
    # Quaternion helpers
    # ------------------------------------------------------------------
    @staticmethod
    def _normalize_quat(x, y, z, w):
        norm = math.sqrt(x * x + y * y + z * z + w * w)
        if norm < 1e-12:
            return 0.0, 0.0, 0.0, 1.0

        return x / norm, y / norm, z / norm, w / norm

    def quat_to_yaw(self, x, y, z, w):
        x, y, z, w = self._normalize_quat(x, y, z, w)

        siny_cosp = 2.0 * (w * z + x * y)
        cosy_cosp = 1.0 - 2.0 * (y * y + z * z)

        return math.atan2(siny_cosp, cosy_cosp)

    @staticmethod
    def _wrap_angle(angle):
        return math.atan2(math.sin(angle), math.cos(angle))

    # ------------------------------------------------------------------
    # Protobuf text parsing helpers
    # ------------------------------------------------------------------
    def _extract_braced_block(self, text, brace_start):
        depth = 0

        for i in range(brace_start, len(text)):
            ch = text[i]

            if ch == '{':
                depth += 1
            elif ch == '}':
                depth -= 1
                if depth == 0:
                    return text[brace_start + 1:i], i

        return None, len(text)

    def _get_block(self, text, block_name):
        if not text:
            return None

        pattern = re.compile(
            r'(?m)(?:^|\W)' + re.escape(block_name) + r'\s*\{'
        )

        match = pattern.search(text)
        if not match:
            return None

        brace_index = text.find('{', match.start())
        if brace_index < 0:
            return None

        content, _ = self._extract_braced_block(text, brace_index)
        return content

    def _iter_pose_blocks(self, msg_text):
        for match in re.finditer(r'(?m)^\s*pose\s*\{', msg_text):
            brace_index = msg_text.find('{', match.start())
            if brace_index < 0:
                continue

            content, _ = self._extract_braced_block(msg_text, brace_index)
            if content is not None:
                yield content

    def _get_name(self, block):
        match = re.search(r'(?m)^\s*name\s*:\s*"([^"]*)"', block)
        if match:
            return match.group(1)
        return None

    def _get_float(self, text, field, default=0.0):
        if not text:
            return default

        pattern = (
            r'(?m)(?:^|\W)' +
            re.escape(field) +
            r'\s*:\s*([-+]?(?:\d+(?:\.\d*)?|\.\d+)(?:[eE][-+]?\d+)?)'
        )

        match = re.search(pattern, text)
        if not match:
            return default

        return float(match.group(1))

    def _get_first_float(self, text, fields, default=0.0):
        """
        Try several possible field names and return the first numeric one.
        Useful for thrust messages where the value may be called
        data, value, or thrust.
        """
        if not text:
            return default

        for field in fields:
            pattern = (
                r'(?m)(?:^|\W)' +
                re.escape(field) +
                r'\s*:\s*([-+]?(?:\d+(?:\.\d*)?|\.\d+)(?:[eE][-+]?\d+)?)'
            )

            match = re.search(pattern, text)
            if match:
                return float(match.group(1))

        return default

    def _get_stamp(self, msg_text):
        header = self._get_block(msg_text, "header")
        if header is None:
            return None

        stamp = self._get_block(header, "stamp")
        if stamp is None:
            return None

        sec = self._get_float(stamp, "sec", 0.0)
        nsec = self._get_float(stamp, "nsec", 0.0)

        return sec + nsec * 1e-9

    def _has_position_fields(self, block):
        position = self._get_block(block, "position")
        if position is None:
            return False

        return bool(re.search(r'(?m)(?:^|\W)[xyz]\s*:', position))

    def _select_pose_block(self, msg_text):
        blocks = list(self._iter_pose_blocks(msg_text))

        if not blocks:
            if (
                re.search(r'(?m)^\s*position\s*\{', msg_text)
                or re.search(r'(?m)^\s*orientation\s*\{', msg_text)
                or re.search(r'(?m)^\s*name\s*:', msg_text)
            ):
                blocks = [msg_text]
            else:
                return None

        exact_match = None
        target_base_link_match = None

        for block in blocks:
            name = self._get_name(block) or ""

            if name == self.target_entity:
                exact_match = block
                break

            if (
                name == f"{self.target_entity}/base_link"
                and self._has_position_fields(block)
            ):
                target_base_link_match = block

        if exact_match is not None:
            return exact_match

        if target_base_link_match is not None:
            return target_base_link_match

        return None

    # ------------------------------------------------------------------
    # Acceleration helpers
    # ------------------------------------------------------------------
    def _low_pass_accel(self, old_value, new_value):
        return (1.0 - self.accel_alpha) * old_value + self.accel_alpha * new_value

    def _reset_acceleration(self):
        self.u_dot = 0.0
        self.v_dot = 0.0
        self.r_dot = 0.0

        self.prev_u_accel = None
        self.prev_v_accel = None
        self.prev_r_accel = None
        self.prev_accel_stamp = None

        self.acceleration_initialized = False

    def _set_prev_accel(self, stamp, u, v, r):
        self.prev_u_accel = u
        self.prev_v_accel = v
        self.prev_r_accel = r
        self.prev_accel_stamp = stamp

    # ------------------------------------------------------------------
    # Velocity helpers
    # ------------------------------------------------------------------
    def _low_pass(self, old_value, new_value):
        return (1.0 - self.vel_alpha) * old_value + self.vel_alpha * new_value

    def _set_prev_pose(self, stamp, x, y, z, yaw, quat):
        self.prev_stamp = stamp
        self.prev_x = x
        self.prev_y = y
        self.prev_z = z
        self.prev_yaw = yaw
        self.prev_quat = quat

    def _reset_velocity(self):
        self.vx_world = 0.0
        self.vy_world = 0.0
        self.vz_world = 0.0
        self.speed_world = 0.0

        self.u = 0.0
        self.v = 0.0
        self.w = 0.0

        self.p = 0.0
        self.q = 0.0
        self.r = 0.0

        self.r_yaw = 0.0
        self.speed = 0.0

        self.dt = None
        self.velocity_initialized = False

        self._reset_acceleration()

    def _world_to_body_velocity(self, vx, vy, vz, qx, qy, qz, qw):
        qx, qy, qz, qw = self._normalize_quat(qx, qy, qz, qw)

        # Rotation matrix R: body -> world
        r00 = 1.0 - 2.0 * (qy * qy + qz * qz)
        r01 = 2.0 * (qx * qy - qw * qz)
        r02 = 2.0 * (qx * qz + qw * qy)

        r10 = 2.0 * (qx * qy + qw * qz)
        r11 = 1.0 - 2.0 * (qx * qx + qz * qz)
        r12 = 2.0 * (qy * qz - qw * qx)

        r20 = 2.0 * (qx * qz - qw * qy)
        r21 = 2.0 * (qy * qz + qw * qx)
        r22 = 1.0 - 2.0 * (qx * qx + qy * qy)

        # World -> body is R^T
        u = r00 * vx + r10 * vy + r20 * vz
        v = r01 * vx + r11 * vy + r21 * vz
        w = r02 * vx + r12 * vy + r22 * vz

        return u, v, w

    def _angular_velocity_body(self, q_prev, q_curr, dt):
        if dt <= 0.0:
            return 0.0, 0.0, 0.0

        x0, y0, z0, w0 = self._normalize_quat(*q_prev)
        x1, y1, z1, w1 = self._normalize_quat(*q_curr)

        # q_prev inverse
        ix = -x0
        iy = -y0
        iz = -z0
        iw = w0

        # q_rel = q_prev^{-1} * q_curr
        rw = iw * w1 - ix * x1 - iy * y1 - iz * z1
        rx = iw * x1 + ix * w1 + iy * z1 - iz * y1
        ry = iw * y1 - ix * z1 + iy * w1 + iz * x1
        rz = iw * z1 + ix * y1 - iy * x1 + iz * w1

        rx, ry, rz, rw = self._normalize_quat(rx, ry, rz, rw)

        if rw < 0.0:
            rx = -rx
            ry = -ry
            rz = -rz
            rw = -rw

        rw = max(-1.0, min(1.0, rw))

        angle = 2.0 * math.acos(rw)
        s = math.sqrt(max(0.0, 1.0 - rw * rw))

        if angle < 1e-6 or s < 1e-6:
            p = 2.0 * rx / dt
            q = 2.0 * ry / dt
            r = 2.0 * rz / dt
            return p, q, r

        axis_x = rx / s
        axis_y = ry / s
        axis_z = rz / s

        p = axis_x * angle / dt
        q = axis_y * angle / dt
        r = axis_z * angle / dt

        return p, q, r

    def _update_velocity(self, stamp, quat):
        if self.prev_stamp is None:
            self._set_prev_pose(
                stamp,
                self.global_x,
                self.global_y,
                self.global_z,
                self.global_yaw,
                quat
            )
            return

        dt = stamp - self.prev_stamp

        if dt < 0.0:
            self._set_prev_pose(
                stamp,
                self.global_x,
                self.global_y,
                self.global_z,
                self.global_yaw,
                quat
            )
            return

        if dt <= 0.0:
            return

        if dt < self.min_dt:
            return

        if dt > self.max_dt:
            self._reset_velocity()
            self._set_prev_pose(
                stamp,
                self.global_x,
                self.global_y,
                self.global_z,
                self.global_yaw,
                quat
            )
            return

        qx, qy, qz, qw = quat

        vx_raw = (self.global_x - self.prev_x) / dt
        vy_raw = (self.global_y - self.prev_y) / dt
        vz_raw = (self.global_z - self.prev_z) / dt

        u_raw, v_raw, w_raw = self._world_to_body_velocity(
            vx_raw,
            vy_raw,
            vz_raw,
            qx,
            qy,
            qz,
            qw
        )

        p_raw, q_raw, r_raw = self._angular_velocity_body(
            self.prev_quat,
            quat,
            dt
        )

        r_yaw_raw = self._wrap_angle(self.global_yaw - self.prev_yaw) / dt

        if not self.velocity_initialized:
            self.vx_world = vx_raw
            self.vy_world = vy_raw
            self.vz_world = vz_raw

            self.u = u_raw
            self.v = v_raw
            self.w = w_raw

            self.p = p_raw
            self.q = q_raw
            self.r = r_raw

            self.r_yaw = r_yaw_raw

            self.velocity_initialized = True
        else:
            self.vx_world = self._low_pass(self.vx_world, vx_raw)
            self.vy_world = self._low_pass(self.vy_world, vy_raw)
            self.vz_world = self._low_pass(self.vz_world, vz_raw)

            self.u = self._low_pass(self.u, u_raw)
            self.v = self._low_pass(self.v, v_raw)
            self.w = self._low_pass(self.w, w_raw)

            self.p = self._low_pass(self.p, p_raw)
            self.q = self._low_pass(self.q, q_raw)
            self.r = self._low_pass(self.r, r_raw)

            self.r_yaw = self._low_pass(self.r_yaw, r_yaw_raw)

        self.speed = math.hypot(self.u, self.v)
        self.speed_world = math.hypot(self.vx_world, self.vy_world)
        self.dt = dt

        # ----------------------------------------------------------
        # Estimate u_dot, v_dot, r_dot from filtered u, v, r
        # ----------------------------------------------------------
        if self.prev_u_accel is None:
            self._set_prev_accel(stamp, self.u, self.v, self.r)
        else:
            dt_acc = stamp - self.prev_accel_stamp

            if dt_acc < 0.0:
                self._set_prev_accel(stamp, self.u, self.v, self.r)

            elif dt_acc <= 0.0 or dt_acc < self.min_dt:
                pass

            elif dt_acc > self.max_dt:
                self._reset_acceleration()
                self._set_prev_accel(stamp, self.u, self.v, self.r)

            else:
                u_dot_raw = (self.u - self.prev_u_accel) / dt_acc
                v_dot_raw = (self.v - self.prev_v_accel) / dt_acc
                r_dot_raw = (self.r - self.prev_r_accel) / dt_acc

                if not self.acceleration_initialized:
                    self.u_dot = u_dot_raw
                    self.v_dot = v_dot_raw
                    self.r_dot = r_dot_raw

                    self.acceleration_initialized = True
                else:
                    self.u_dot = self._low_pass_accel(self.u_dot, u_dot_raw)
                    self.v_dot = self._low_pass_accel(self.v_dot, v_dot_raw)
                    self.r_dot = self._low_pass_accel(self.r_dot, r_dot_raw)

                self._set_prev_accel(stamp, self.u, self.v, self.r)

        self._set_prev_pose(
            stamp,
            self.global_x,
            self.global_y,
            self.global_z,
            self.global_yaw,
            quat
        )

    # ------------------------------------------------------------------
    # IMU listener
    # ------------------------------------------------------------------
    def _parse_imu_message(self, msg_text):
        stamp = self._get_stamp(msg_text)
        if stamp is None:
            stamp = time.time()

        lin_block = self._get_block(msg_text, "linear_acceleration")
        ang_block = self._get_block(msg_text, "angular_velocity")

        if lin_block is None and ang_block is None:
            return False

        ax = self._get_float(lin_block, "x", 0.0) if lin_block else 0.0
        ay = self._get_float(lin_block, "y", 0.0) if lin_block else 0.0
        az = self._get_float(lin_block, "z", 0.0) if lin_block else 0.0

        wx = self._get_float(ang_block, "x", 0.0) if ang_block else 0.0
        wy = self._get_float(ang_block, "y", 0.0) if ang_block else 0.0
        wz = self._get_float(ang_block, "z", 0.0) if ang_block else 0.0

        with self.imu_lock:
            self.ax_imu = ax
            self.ay_imu = ay
            self.az_imu = az

            self.wx_imu = wx
            self.wy_imu = wy
            self.wz_imu = wz

            self.imu_stamp = stamp
            self.imu_message_count += 1

        return True

    def _imu_loop(self, process):
        msg_lines = []
        has_header = False
        has_content = False

        def flush_message():
            nonlocal msg_lines, has_header, has_content

            if not msg_lines or not has_content:
                msg_lines = []
                has_header = False
                has_content = False
                return

            msg_text = "".join(msg_lines)
            self._parse_imu_message(msg_text)

            msg_lines = []
            has_header = False
            has_content = False

        try:
            for line in process.stdout:
                stripped = line.strip()

                if stripped.startswith("header {"):
                    if has_header:
                        flush_message()
                    has_header = True

                msg_lines.append(line)

                if re.search(
                    r"\b(linear_acceleration|angular_velocity|orientation|stamp)\b",
                    stripped
                ):
                    has_content = True

                if stripped == "" and has_header and has_content:
                    flush_message()

            flush_message()

        except Exception as e:
            print(f"[WARN] IMU listener error: {e}", file=sys.stderr)

    def _start_imu_listener(self):
        if not self.imu_topic:
            print("[INFO] IMU listener disabled.")
            return None

        print(f"Starting IMU listener on topic: {self.imu_topic}")

        cmd = ["gz", "topic", "-e", "-t", self.imu_topic]

        if shutil.which("stdbuf"):
            cmd = ["stdbuf", "-oL"] + cmd

        try:
            process = subprocess.Popen(
                cmd,
                stdout=subprocess.PIPE,
                stderr=None,
                text=True
            )
        except FileNotFoundError:
            print(
                "[ERROR] 'gz' command not found for IMU listener.",
                file=sys.stderr
            )
            return None

        thread = threading.Thread(
            target=self._imu_loop,
            args=(process,),
            daemon=True
        )
        thread.start()

        return process

    # ------------------------------------------------------------------
    # Thrust listeners
    # ------------------------------------------------------------------
    def _thrust_loop(self, process, side):
        msg_lines = []
        has_header = False
        has_content = False

        def flush_message():
            nonlocal msg_lines, has_header, has_content

            if not msg_lines or not has_content:
                msg_lines = []
                has_header = False
                has_content = False
                return

            msg_text = "".join(msg_lines)

            # Most thrust messages use a numeric field called data.
            # If the value is zero, protobuf text may omit it, so default to 0.
            value = self._get_first_float(
                msg_text,
                ("data", "value", "thrust"),
                0.0
            )

            with self.thrust_lock:
                if side == "left":
                    self.T_L = value
                    self.left_thrust_count += 1
                else:
                    self.T_R = value
                    self.right_thrust_count += 1

            msg_lines = []
            has_header = False
            has_content = False

        try:
            for line in process.stdout:
                stripped = line.strip()

                value_line = bool(
                    re.search(
                        r"(?:^|\W)(data|value|thrust)\s*:",
                        stripped
                    )
                )

                # If messages are simple and not separated by blank lines,
                # flush before starting a new value line.
                if value_line and has_content and not has_header:
                    flush_message()

                if stripped.startswith("header {"):
                    if has_header and has_content:
                        flush_message()
                    has_header = True

                msg_lines.append(line)

                if re.search(r"\b(data|value|thrust|header)\b", stripped):
                    has_content = True

                if stripped == "" and has_content:
                    flush_message()

            flush_message()

        except Exception as e:
            print(f"[WARN] Thrust listener error ({side}): {e}", file=sys.stderr)

    def _start_thrust_listener(self, topic, side):
        if not topic:
            return None

        print(f"Starting {side} thruster listener on topic: {topic}")

        cmd = ["gz", "topic", "-e", "-t", topic]

        if shutil.which("stdbuf"):
            cmd = ["stdbuf", "-oL"] + cmd

        try:
            process = subprocess.Popen(
                cmd,
                stdout=subprocess.PIPE,
                stderr=None,
                text=True
            )
        except FileNotFoundError:
            print(
                f"[ERROR] 'gz' command not found for {side} thruster listener.",
                file=sys.stderr
            )
            return None

        thread = threading.Thread(
            target=self._thrust_loop,
            args=(process, side),
            daemon=True
        )
        thread.start()

        return process

    # ------------------------------------------------------------------
    # Segment listener using ROS 2 CLI
    # ------------------------------------------------------------------
    def _segment_loop(self, process):
        try:
            for line in process.stdout:
                stripped = line.strip()

                if not stripped:
                    continue

                if stripped == "---":
                    continue

                label = stripped

                # If output is YAML-like:
                #   data: "Exp1_straight_accel"
                match = re.search(r"data:\s*(.*)", stripped)
                if match:
                    label = match.group(1)

                label = label.strip().strip('"').strip("'")

                if label:
                    with self.segment_lock:
                        self.segment_label = label
                        self.segment_message_count += 1

        except Exception as e:
            print(f"[WARN] Segment listener error: {e}", file=sys.stderr)

    def _start_segment_listener(self):
        if not self.segment_topic:
            print("[INFO] Segment listener disabled.")
            return None

        if not shutil.which("ros2"):
            print(
                "[WARN] 'ros2' command not found. "
                "Segment label logging is disabled."
            )
            return None

        print(f"Starting ROS segment listener on topic: {self.segment_topic}")

        cmd = [
            "ros2",
            "topic",
            "echo",
            self.segment_topic,
            "--field",
            "data"
        ]

        if shutil.which("stdbuf"):
            cmd = ["stdbuf", "-oL"] + cmd

        try:
            process = subprocess.Popen(
                cmd,
                stdout=subprocess.PIPE,
                stderr=None,
                text=True
            )
        except FileNotFoundError:
            print(
                "[WARN] Could not start ros2 segment listener.",
                file=sys.stderr
            )
            return None

        thread = threading.Thread(
            target=self._segment_loop,
            args=(process,),
            daemon=True
        )
        thread.start()

        return process

    # ------------------------------------------------------------------
    # Parse one complete Gazebo pose message
    # ------------------------------------------------------------------
    def parse_message(self, msg_text):
        stamp = self._get_stamp(msg_text)

        if stamp is None:
            stamp = time.time()

        self.current_sim_time = stamp

        block = self._select_pose_block(msg_text)

        if block is None:
            if not self.warned_no_target:
                print(
                    f"[WARN] Could not find pose entity named "
                    f"'{self.target_entity}' in Gazebo message. "
                    f"Check the topic and target entity name."
                )
                self.warned_no_target = True
            return False

        position_block = self._get_block(block, "position") or ""
        orientation_block = self._get_block(block, "orientation")

        self.global_x = self._get_float(position_block, "x", 0.0)
        self.global_y = self._get_float(position_block, "y", 0.0)
        self.global_z = self._get_float(position_block, "z", 0.0)

        if orientation_block is None:
            qx = 0.0
            qy = 0.0
            qz = 0.0
            qw = 1.0
        else:
            qx = self._get_float(orientation_block, "x", 0.0)
            qy = self._get_float(orientation_block, "y", 0.0)
            qz = self._get_float(orientation_block, "z", 0.0)
            qw = self._get_float(orientation_block, "w", 0.0)

        self.global_yaw = self.quat_to_yaw(qx, qy, qz, qw)

        quat = (qx, qy, qz, qw)

        # Velocity and acceleration estimation
        self._update_velocity(stamp, quat)

        # Base origin logic
        if self.base_x is None:
            self.base_x = self.global_x
            self.base_y = self.global_y
            self.base_z = self.global_z
            self.base_yaw = self.global_yaw
            self.base_sim_time = stamp

            print(
                f"[INFO] Base origin locked: "
                f"sim_time={self.base_sim_time:.3f}s, "
                f"x={self.base_x:.3f}, "
                f"y={self.base_y:.3f}, "
                f"z={self.base_z:.3f}, "
                f"yaw={math.degrees(self.base_yaw):.2f} deg"
            )

        dx = self.global_x - self.base_x
        dy = self.global_y - self.base_y
        dz = self.global_z - self.base_z

        cos_b = math.cos(-self.base_yaw)
        sin_b = math.sin(-self.base_yaw)

        self.local_x = dx * cos_b - dy * sin_b
        self.local_y = dx * sin_b + dy * cos_b
        self.local_z = dz

        rel_yaw = self.global_yaw - self.base_yaw
        self.local_yaw = math.atan2(math.sin(rel_yaw), math.cos(rel_yaw))

        return True

    # ------------------------------------------------------------------
    # Recording loop
    # ------------------------------------------------------------------
    def record(self, output_csv="gz_fossen_log.csv"):
        print(f"Starting Gazebo pose listener on topic: {self.topic}")
        print(f"Target entity: '{self.target_entity}'")
        print(f"Velocity filter alpha: {self.vel_alpha:.3f}")
        print(f"Acceleration filter alpha: {self.accel_alpha:.3f}")
        print(f"Min dt: {self.min_dt:.4f} s")
        print(f"Max dt: {self.max_dt:.4f} s")
        print("Press Ctrl+C to stop recording.\n")

        cmd = ["gz", "topic", "-e", "-t", self.topic]

        if shutil.which("stdbuf"):
            cmd = ["stdbuf", "-oL"] + cmd

        try:
            pose_process = subprocess.Popen(
                cmd,
                stdout=subprocess.PIPE,
                stderr=None,
                text=True
            )
        except FileNotFoundError:
            print(
                "[ERROR] 'gz' command not found. "
                "Make sure Gazebo Harmonic is installed and sourced.",
                file=sys.stderr
            )
            return

        processes = [pose_process]

        imu_process = self._start_imu_listener()
        if imu_process is not None:
            processes.append(imu_process)

        left_process = self._start_thrust_listener(
            self.left_thrust_topic,
            "left"
        )
        if left_process is not None:
            processes.append(left_process)

        right_process = self._start_thrust_listener(
            self.right_thrust_topic,
            "right"
        )
        if right_process is not None:
            processes.append(right_process)

        segment_process = self._start_segment_listener()
        if segment_process is not None:
            processes.append(segment_process)

        out_dir = os.path.dirname(os.path.abspath(output_csv))
        if out_dir:
            os.makedirs(out_dir, exist_ok=True)

        with open(output_csv, "w", newline="") as f:
            writer = csv.writer(f)
            writer.writerow([
                "sim_time_s",
                "elapsed_sim_s",
                "wall_time_s",
                "dt_s",
                "segment",

                "x_pose",
                "y_pose",

                "x_local",
                "y_local",
                "z_local",
                "yaw_local_deg",

                "x_global",
                "y_global",
                "z_global",
                "yaw_global_deg",

                "u",
                "v",
                "w",

                "p",
                "q",
                "r",
                "r_yaw",

                "u_dot",
                "v_dot",
                "r_dot",

                "vx_world",
                "vy_world",
                "vz_world",

                "speed",
                "speed_world",

                "ax_imu",
                "ay_imu",
                "az_imu",

                "wx_imu",
                "wy_imu",
                "wz_imu",

                "imu_age_s",

                "T_L",
                "T_R",
            ])

            start_wall_time = time.time()

            msg_lines = []
            has_header = False
            has_content = False

            def flush_message():
                nonlocal msg_lines, has_header, has_content

                if not msg_lines or not has_content:
                    msg_lines = []
                    has_header = False
                    has_content = False
                    return

                msg_text = "".join(msg_lines)

                if self.parse_message(msg_text):
                    wall_elapsed = time.time() - start_wall_time

                    if (
                        self.current_sim_time is not None
                        and self.base_sim_time is not None
                    ):
                        sim_elapsed = self.current_sim_time - self.base_sim_time
                    else:
                        sim_elapsed = wall_elapsed

                    dt_str = f"{self.dt:.6f}" if self.dt is not None else ""

                    with self.imu_lock:
                        ax_imu = self.ax_imu
                        ay_imu = self.ay_imu
                        az_imu = self.az_imu

                        wx_imu = self.wx_imu
                        wy_imu = self.wy_imu
                        wz_imu = self.wz_imu

                        imu_stamp = self.imu_stamp
                        imu_count = self.imu_message_count

                    with self.thrust_lock:
                        T_L = self.T_L
                        T_R = self.T_R
                        left_count = self.left_thrust_count
                        right_count = self.right_thrust_count

                    with self.segment_lock:
                        segment_label = self.segment_label
                        segment_count = self.segment_message_count

                    imu_age_str = ""
                    if (
                        imu_stamp is not None
                        and self.current_sim_time is not None
                    ):
                        imu_age = self.current_sim_time - imu_stamp
                        imu_age_str = f"{imu_age:.6f}"

                    if (
                        self.imu_topic
                        and imu_count == 0
                        and wall_elapsed > 5.0
                        and not self.warned_no_imu
                    ):
                        self.warned_no_imu = True
                        print(
                            "[WARN] No IMU messages received yet. "
                            "Check the IMU topic with:\n"
                            "    gz topic -l | grep -i imu"
                        )

                    if (
                        (self.left_thrust_topic or self.right_thrust_topic)
                        and left_count == 0
                        and right_count == 0
                        and wall_elapsed > 5.0
                        and not self.warned_no_thrust
                    ):
                        self.warned_no_thrust = True
                        print(
                            "[WARN] No thruster messages received yet. "
                            "Check the thruster topics with:\n"
                            "    gz topic -l | grep thrust"
                        )

                    if (
                        self.segment_topic
                        and segment_count == 0
                        and wall_elapsed > 10.0
                        and not self.warned_no_segment
                    ):
                        self.warned_no_segment = True
                        print(
                            "[WARN] No segment labels received yet. "
                            "Make sure guide.py is running and publishing on:\n"
                            f"    {self.segment_topic}"
                        )

                    writer.writerow([
                        f"{self.current_sim_time:.6f}",
                        f"{sim_elapsed:.6f}",
                        f"{wall_elapsed:.3f}",
                        dt_str,
                        segment_label,

                        # x_pose and y_pose for trajectory plotting.
                        # Here they are local coordinates, so the trajectory
                        # starts at (0, 0).
                        f"{self.local_x:.4f}",
                        f"{self.local_y:.4f}",

                        f"{self.local_x:.4f}",
                        f"{self.local_y:.4f}",
                        f"{self.local_z:.4f}",
                        f"{math.degrees(self.local_yaw):.3f}",

                        f"{self.global_x:.4f}",
                        f"{self.global_y:.4f}",
                        f"{self.global_z:.4f}",
                        f"{math.degrees(self.global_yaw):.3f}",

                        f"{self.u:.4f}",
                        f"{self.v:.4f}",
                        f"{self.w:.4f}",

                        f"{self.p:.4f}",
                        f"{self.q:.4f}",
                        f"{self.r:.4f}",
                        f"{self.r_yaw:.4f}",

                        f"{self.u_dot:.4f}",
                        f"{self.v_dot:.4f}",
                        f"{self.r_dot:.4f}",

                        f"{self.vx_world:.4f}",
                        f"{self.vy_world:.4f}",
                        f"{self.vz_world:.4f}",

                        f"{self.speed:.4f}",
                        f"{self.speed_world:.4f}",

                        f"{ax_imu:.4f}",
                        f"{ay_imu:.4f}",
                        f"{az_imu:.4f}",

                        f"{wx_imu:.4f}",
                        f"{wy_imu:.4f}",
                        f"{wz_imu:.4f}",

                        imu_age_str,

                        f"{T_L:.3f}",
                        f"{T_R:.3f}",
                    ])
                    f.flush()

                msg_lines = []
                has_header = False
                has_content = False

            try:
                for line in pose_process.stdout:
                    stripped = line.strip()

                    if stripped.startswith("header {"):
                        if has_header:
                            flush_message()
                        has_header = True

                    msg_lines.append(line)

                    if re.search(r"\b(pose|position|orientation|name)\b", stripped):
                        has_content = True

                    if stripped == "" and has_header and has_content:
                        flush_message()

                flush_message()

            except KeyboardInterrupt:
                print("\n[INFO] Stopping recorder...")

            finally:
                for p in processes:
                    if p is None:
                        continue

                    if p.poll() is None:
                        p.terminate()
                        try:
                            p.wait(timeout=2)
                        except subprocess.TimeoutExpired:
                            p.kill()

                print(f"[INFO] Saved data to: {output_csv}")


def main():
    parser = argparse.ArgumentParser(
        description=(
            "Record WAM-V pose, trajectory coordinates, velocity, acceleration, "
            "IMU, thrusters, and segment label for Fossen 3-DOF model validation."
        )
    )

    parser.add_argument(
        "--topic",
        default="/world/sydney_regatta/pose/info",
        help="Gazebo pose topic."
    )

    parser.add_argument(
        "--target",
        default="wamv",
        help="Entity/model name to extract, e.g. 'wamv'."
    )

    parser.add_argument(
        "--imu-topic",
        default=(
            "/world/sydney_regatta/model/wamv/link/wamv/"
            "imu_wamv_link/sensor/imu_wamv_sensor/imu"
        ),
        help="Gazebo IMU topic."
    )

    parser.add_argument(
        "--left-thrust-topic",
        default="/wamv/thrusters/left/thrust",
        help="Gazebo left thruster command topic."
    )

    parser.add_argument(
        "--right-thrust-topic",
        default="/wamv/thrusters/right/thrust",
        help="Gazebo right thruster command topic."
    )

    parser.add_argument(
        "--segment-topic",
        default="/usv/current_segment",
        help="ROS segment label topic published by guide.py."
    )

    parser.add_argument(
        "--output",
        default="gz_fossen_log.csv",
        help="Output CSV file."
    )

    parser.add_argument(
        "--velocity-alpha",
        type=float,
        default=0.35,
        help="Low-pass filter coefficient for velocity. 1.0 = no filtering."
    )

    parser.add_argument(
        "--accel-alpha",
        type=float,
        default=0.25,
        help="Low-pass filter coefficient for u_dot, v_dot, r_dot."
    )

    parser.add_argument(
        "--min-dt",
        type=float,
        default=0.001,
        help="Minimum dt accepted for numerical differentiation."
    )

    parser.add_argument(
        "--max-dt",
        type=float,
        default=1.0,
        help="Maximum dt accepted before resetting velocity estimation."
    )

    parser.add_argument(
        "--no-imu",
        action="store_true",
        help="Disable IMU listener."
    )

    parser.add_argument(
        "--no-thrusters",
        action="store_true",
        help="Disable thruster listeners."
    )

    parser.add_argument(
        "--no-segment",
        action="store_true",
        help="Disable ROS segment listener."
    )

    args = parser.parse_args()

    imu_topic = "" if args.no_imu else args.imu_topic
    left_thrust_topic = "" if args.no_thrusters else args.left_thrust_topic
    right_thrust_topic = "" if args.no_thrusters else args.right_thrust_topic
    segment_topic = "" if args.no_segment else args.segment_topic

    recorder = GzFossenRecorder(
        topic=args.topic,
        target_entity=args.target,
        imu_topic=imu_topic,
        left_thrust_topic=left_thrust_topic,
        right_thrust_topic=right_thrust_topic,
        segment_topic=segment_topic,
        velocity_alpha=args.velocity_alpha,
        accel_alpha=args.accel_alpha,
        min_dt=args.min_dt,
        max_dt=args.max_dt
    )

    recorder.record(args.output)


if __name__ == "__main__":
    main()