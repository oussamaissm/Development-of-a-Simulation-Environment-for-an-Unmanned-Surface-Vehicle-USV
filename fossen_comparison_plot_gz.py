#!/usr/bin/env python3
"""
Compares measured accelerations (IMU + kinematic) with the Fossen 3-DOF
model prediction INCLUDING wind and wave forces, using the CSV produced
by the Gazebo Fossen recorder.

Wind model (VRX paper, eqs. 23-25):
    uw = Vw*cos(beta - psi),  vw = Vw*sin(beta - psi)
    urw = u - uw,             vrw = v - vw
    X = cx*urw*|urw|, Y = cy*vrw*|vrw|, N = -2*cn*urw*vrw

Wave model (practical 3-DOF form):
    mean drift + first-order oscillation at the peak period.

Usage:
    python3 fossen_comparison_plot_gz.py gz_fossen_log.csv \
        --config params.json --out comparison.png
"""

import sys
import csv
import json
import math
import argparse

import numpy as np

import matplotlib
matplotlib.use("Agg")
import matplotlib.pyplot as plt


DEFAULT_PARAMS = dict(
    # rigid body + added mass
    m=0.0, Iz=0.0, xg=0.0,
    Xu_dot=0.0, Yv_dot=0.0, Nr_dot=0.0, Yr_dot=0.0, Nv_dot=0.0,
    # damping
    Xu=0.0, Yv=0.0, Nr=0.0, Xuu=0.0, Yvv=0.0, Nrr=0.0,
    # propulsion
    lever_arm=1.0, thrust_scale=1.0,
    # heading at t=0 (copy from recorder "Base origin locked" print)
    psi0_deg=0.0,
    # wind (from your SDF: coeff_vector .5 .5 .33, dir 240, mean 5 m/s)
    wind_cx=0.5, wind_cy=0.5, wind_cn=0.33,
    wind_mean_velocity=5.0, wind_direction_deg=240.0, wind_sign=1.0,
    # waves (from your SDF: period 10, gain 5, steepness 2)
    wave_period=10.0, wave_gain=5.0, wave_steepness=2.0,
    wave_drift_x=0.0, wave_drift_y=0.0, wave_drift_n=0.0,
    wave_amp_x=0.0, wave_amp_y=0.0, wave_amp_n=0.0,
    wave_phase_x=0.0, wave_phase_y=0.0, wave_phase_n=0.0,
)


# ------------------------------------------------------------------
# Helpers
# ------------------------------------------------------------------
def to_float(value, default=0.0):
    try:
        if value is None:
            return default
        s = str(value).strip()
        if s == "":
            return default
        return float(s)
    except Exception:
        return default


def load_csv(path):
    with open(path, newline="") as f:
        return list(csv.DictReader(f))


def get_array(rows, candidates, required=False, default=0.0):
    if isinstance(candidates, str):
        candidates = [candidates]

    if not rows:
        if required:
            print("CSV is empty.", file=sys.stderr)
            sys.exit(1)
        return None, np.array([])

    available = rows[0].keys()

    for col in candidates:
        if col in available:
            arr = np.array([to_float(r.get(col), default) for r in rows])
            arr = np.nan_to_num(arr, nan=0.0, posinf=0.0, neginf=0.0)
            return col, arr

    if required:
        print(
            f"Missing required column. Looked for: {candidates}\n"
            f"Available columns: {list(available)}",
            file=sys.stderr
        )
        sys.exit(1)

    return None, np.full(len(rows), default)


def numeric_derivative(t, x):
    x = np.asarray(x, dtype=float)
    t = np.asarray(t, dtype=float)
    d = np.zeros_like(x)

    if len(t) < 2:
        return d

    dt = np.diff(t)
    dx = np.diff(x)

    if np.all(dt > 1e-9):
        try:
            return np.gradient(x, t, edge_order=2 if len(t) > 2 else 1)
        except Exception:
            pass

    good = dt > 1e-9
    d[1:][good] = dx[good] / dt[good]
    for i in range(1, len(d)):
        if not good[i - 1]:
            d[i] = d[i - 1]
    if len(d) > 1:
        d[0] = d[1]
    return d


def moving_average(x, window):
    x = np.asarray(x, dtype=float)
    if window <= 1 or len(x) == 0:
        return x
    kernel = np.ones(int(window)) / float(window)
    return np.convolve(x, kernel, mode="same")


# ------------------------------------------------------------------
# Fossen 3-DOF model with wind + waves
# ------------------------------------------------------------------
def build_M(p):
    m, Iz, xg = p["m"], p["Iz"], p["xg"]
    return np.array([
        [m - p["Xu_dot"], 0.0, 0.0],
        [0.0, m - p["Yv_dot"], m * xg - p["Yr_dot"]],
        [0.0, m * xg - p["Nv_dot"], Iz - p["Nr_dot"]],
    ])


def wind_forces(u, v, psi, Vw, beta, p):
    """Paper eqs. (23)-(25), vectorized."""
    uw = Vw * np.cos(beta - psi)
    vw = Vw * np.sin(beta - psi)

    urw = u - uw
    vrw = v - vw

    s = p["wind_sign"]
    X = s * p["wind_cx"] * urw * np.abs(urw)
    Y = s * p["wind_cy"] * vrw * np.abs(vrw)
    N = s * (-2.0 * p["wind_cn"]) * urw * vrw
    return X, Y, N


def wave_forces(t, p):
    """Mean drift + first-order oscillation at peak period."""
    X = np.full_like(t, p["wave_drift_x"])
    Y = np.full_like(t, p["wave_drift_y"])
    N = np.full_like(t, p["wave_drift_n"])

    Tp = p["wave_period"]
    if Tp > 0.0:
        omega = 2.0 * math.pi / Tp
        X = X + p["wave_amp_x"] * np.cos(omega * t + math.radians(p["wave_phase_x"]))
        Y = Y + p["wave_amp_y"] * np.cos(omega * t + math.radians(p["wave_phase_y"]))
        N = N + p["wave_amp_n"] * np.cos(omega * t + math.radians(p["wave_phase_n"]))

    return X, Y, N


def fossen_accel(Minv, u, v, r, tau, p):
    m, xg = p["m"], p["xg"]

    C = np.array([
        [0.0, 0.0, -(m - p["Yv_dot"]) * v - (m * xg - p["Yr_dot"]) * r],
        [0.0, 0.0, (m - p["Xu_dot"]) * u],
        [(m - p["Yv_dot"]) * v + (m * xg - p["Yr_dot"]) * r,
         -(m - p["Xu_dot"]) * u, 0.0],
    ])

    D = np.diag([
        -p["Xu"] - p["Xuu"] * abs(u),
        -p["Yv"] - p["Yvv"] * abs(v),
        -p["Nr"] - p["Nrr"] * abs(r),
    ])

    nu = np.array([u, v, r])
    rhs = tau - C @ nu - D @ nu
    return Minv @ rhs


# ------------------------------------------------------------------
# Main
# ------------------------------------------------------------------
def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("csv_path")
    ap.add_argument("--config", default=None)
    ap.add_argument("--out", default="fossen_comparison.png")
    ap.add_argument("--time-col", default=None)
    ap.add_argument("--r-col", default=None)
    ap.add_argument("--thrust-scale", type=float, default=None)
    ap.add_argument("--lever-arm", type=float, default=None)
    ap.add_argument("--wind-sign", type=float, default=None)
    ap.add_argument("--smooth", type=int, default=0)
    args = ap.parse_args()

    params = dict(DEFAULT_PARAMS)
    if args.config:
        with open(args.config) as f:
            params.update(json.load(f))
    else:
        print("WARNING: no --config provided.", file=sys.stderr)

    if args.thrust_scale is not None:
        params["thrust_scale"] = args.thrust_scale
    if args.lever_arm is not None:
        params["lever_arm"] = args.lever_arm
    if args.wind_sign is not None:
        params["wind_sign"] = args.wind_sign

    rows = load_csv(args.csv_path)
    if not rows:
        print("CSV file is empty.", file=sys.stderr)
        sys.exit(1)

    # ---------------- columns ----------------
    time_candidates = [args.time_col] if args.time_col else \
        ["elapsed_sim_s", "sim_time_s", "time", "wall_time_s"]
    time_name, t = get_array(rows, time_candidates, required=True)

    u_name, u = get_array(rows, ["u", "u_odom"], required=True)
    v_name, v = get_array(rows, ["v", "v_odom"], required=True)

    r_candidates = [args.r_col] if args.r_col else ["r", "r_yaw", "r_odom"]
    r_name, r_ = get_array(rows, r_candidates, required=True)

    T_L_name, T_L = get_array(rows, ["T_L", "thrust_left"], required=True)
    T_R_name, T_R = get_array(rows, ["T_R", "thrust_right"], required=True)

    ax_name, ax_imu = get_array(rows, ["ax_imu", "ax"], required=True)
    ay_name, ay_imu = get_array(rows, ["ay_imu", "ay"], required=True)

    # heading (deg, local) -> psi = psi0 + yaw_local
    _, yaw_local_deg = get_array(rows, ["yaw_local_deg", "yaw_local", "yaw_pose_deg"])

    # wind: use logged values if present, else constants from params
    ws_col, Vw = get_array(
        rows, ["wind_speed"], default=params["wind_mean_velocity"])
    wd_col, Wd = get_array(
        rows, ["wind_dir"], default=params["wind_direction_deg"])

    # measured kinematic accelerations
    u_dot_name, u_dot_meas = get_array(rows, ["u_dot"])
    if u_dot_name is None:
        u_dot_meas = numeric_derivative(t, u)
        u_dot_name = "derived u"

    v_dot_name, v_dot_meas = get_array(rows, ["v_dot"])
    if v_dot_name is None:
        v_dot_meas = numeric_derivative(t, v)
        v_dot_name = "derived v"

    if r_name == "r":
        r_dot_name, r_dot_meas = get_array(rows, ["r_dot"])
        if r_dot_name is None:
            r_dot_meas = numeric_derivative(t, r_)
            r_dot_name = "derived r"
    else:
        r_dot_name, r_dot_meas = get_array(rows, ["r_dot_yaw"])
        if r_dot_name is None:
            r_dot_meas = numeric_derivative(t, r_)
            r_dot_name = f"derived {r_name}"

    segments = [str(r.get("segment", "")) for r in rows]

    if args.smooth > 1:
        ax_imu = moving_average(ax_imu, args.smooth)
        ay_imu = moving_average(ay_imu, args.smooth)
        u_dot_meas = moving_average(u_dot_meas, args.smooth)
        v_dot_meas = moving_average(v_dot_meas, args.smooth)
        r_dot_meas = moving_average(r_dot_meas, args.smooth)

    # ---------------- environment forces ----------------
    psi = math.radians(params["psi0_deg"]) + np.radians(yaw_local_deg)
    beta = np.radians(Wd)

    X_wind, Y_wind, N_wind = wind_forces(u, v, psi, Vw, beta, params)
    X_wav, Y_wav, N_wav = wave_forces(t, params)

    ts = params["thrust_scale"]
    tau_X = ts * (T_L + T_R) + X_wind + X_wav
    tau_Y = Y_wind + Y_wav
    tau_N = ts * (T_R - T_L) * params["lever_arm"] + N_wind + N_wav

    # ---------------- model integration-free prediction ----------------
    M = build_M(params)
    try:
        Minv = np.linalg.inv(M)
    except np.linalg.LinAlgError:
        print("WARNING: M singular, using pseudo-inverse.", file=sys.stderr)
        Minv = np.linalg.pinv(M)

    n = len(t)
    u_dot_f = np.zeros(n)
    v_dot_f = np.zeros(n)
    r_dot_f = np.zeros(n)

    for i in range(n):
        tau = np.array([tau_X[i], tau_Y[i], tau_N[i]])
        nu_dot = fossen_accel(Minv, u[i], v[i], r_[i], tau, params)
        u_dot_f[i], v_dot_f[i], r_dot_f[i] = nu_dot

    # ---------------- what the IMU measures ----------------
    g = 9.81
    theta_imu = np.array([to_float(r.get("theta_imu")) for r in rows]) \
        if "theta_imu" in rows[0] else np.zeros_like(t)
    phi_imu = np.array([to_float(r.get("phi_imu")) for r in rows]) \
        if "phi_imu" in rows[0] else np.zeros_like(t)

    ax_meas = ax_imu - g * np.sin(theta_imu)
    ay_meas = ay_imu + g * np.sin(phi_imu)

    sm = max(args.smooth, 15)
    ax_meas = moving_average(ax_meas, sm)
    ay_meas = moving_average(ay_meas, sm)

    ax_pred = u_dot_f - r_ * v
    ay_pred = v_dot_f + r_ * u

    # trim initialization artifact
    keep = t > (t[0] + 1.5)
    t, ax_meas, ay_meas = t[keep], ax_meas[keep], ay_meas[keep]
    ax_pred, ay_pred = ax_pred[keep], ay_pred[keep]
    u_dot_meas, v_dot_meas, r_dot_meas = \
        u_dot_meas[keep], v_dot_meas[keep], r_dot_meas[keep]
    u_dot_f, v_dot_f, r_dot_f = u_dot_f[keep], v_dot_f[keep], r_dot_f[keep]
    segments = [s for s, k in zip(segments, keep) if k]

    # ---------------- plot ----------------
    fig, axes = plt.subplots(3, 1, figsize=(12, 9), sharex=True)

    axes[0].plot(t, ax_meas, label=f"IMU ({ax_name})", color="tab:red", lw=1.2)
    axes[0].plot(t, u_dot_meas, label=f"Kinematic ({u_dot_name})",
                 color="tab:gray", lw=0.9, alpha=0.75)
    axes[0].plot(t, u_dot_f, label="Fossen + wind + waves",
                 color="tab:blue", lw=1.2, ls="--")
    axes[0].set_ylabel("Surge accel [m/s²]")
    axes[0].legend(fontsize=8, loc="upper right")
    axes[0].set_title("Surge")
    axes[0].grid(alpha=0.3)

    axes[1].plot(t, ay_meas, label=f"IMU ({ay_name})", color="tab:red", lw=1.2)
    axes[1].plot(t, v_dot_meas, label=f"Kinematic ({v_dot_name})",
                 color="tab:gray", lw=0.9, alpha=0.75)
    axes[1].plot(t, ay_pred, label="Fossen + wind + waves",
                 color="tab:blue", lw=1.2, ls="--")
    axes[1].set_ylabel("Sway accel [m/s²]")
    axes[1].legend(fontsize=8, loc="upper right")
    axes[1].set_title("Sway:")
    axes[1].grid(alpha=0.3)

    axes[2].plot(t, r_dot_meas, label=f"Kinematic ({r_dot_name})",
                 color="tab:gray", lw=0.9, alpha=0.75)
    axes[2].plot(t, r_dot_f, label="Fossen + wind + waves",
                 color="tab:blue", lw=1.2, ls="--")
    axes[2].set_ylabel("Yaw accel [rad/s²]")
    axes[2].set_xlabel(f"Time [{time_name}]")
    axes[2].legend(fontsize=8, loc="upper right")
    axes[2].set_title("Yaw")
    axes[2].grid(alpha=0.3)

    prev_seg = None
    for i, seg in enumerate(segments):
        if seg != prev_seg:
            if prev_seg is not None:
                for ax in axes:
                    ax.axvline(t[i], color="black", alpha=0.18, lw=0.8)
            prev_seg = seg

    fig.tight_layout()
    fig.savefig(args.out, dpi=150)
    print(f"Plot saved: {args.out}")
    print(f"Wind source: {'logged columns' if ws_col else 'constant from params'} "
          f"(Vw={params['wind_mean_velocity']}, dir={params['wind_direction_deg']})")


if __name__ == "__main__":
    main()
