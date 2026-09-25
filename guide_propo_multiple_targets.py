#!/usr/bin/env python3
"""
Guidage USV - Version Multi-Waypoints SANS filtre de Kalman
- Position obtenue directement à partir du GPS (projection plane locale).
- Cap obtenu directement à partir du quaternion IMU.
- Suivi séquentiel d'une liste de cibles (waypoints).
- Freinage et passage automatique au waypoint suivant.
- Enregistrement de la trajectoire complète et création du graphique pour le rapport.
"""

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import NavSatFix, Imu
from std_msgs.msg import Float64
import math
import numpy as np
import csv
import os

try:
    import matplotlib.pyplot as plt
    HAS_MATPLOTLIB = True
except ImportError:
    HAS_MATPLOTLIB = False


class SimplePositionEstimator:
    """Estime la position (à partir du GPS) et le cap (à partir de l'IMU),
    sans fusion par filtre de Kalman."""

    def __init__(self):
        self.origin_lat = None
        self.origin_lon = None
        self.x = 0.0
        self.y = 0.0
        self.yaw = 0.0

    def update_gps_position(self, lat, lon):
        x_gps, y_gps = self._latlon_to_xy(lat, lon)
        self.x = x_gps
        self.y = y_gps
        return x_gps, y_gps

    def update_yaw_from_quaternion(self, qx, qy, qz, qw):
        # Conversion quaternion -> yaw (rotation autour de Z)
        siny_cosp = 2.0 * (qw * qz + qx * qy)
        cosy_cosp = 1.0 - 2.0 * (qy * qy + qz * qz)
        self.yaw = math.atan2(siny_cosp, cosy_cosp)

    def _latlon_to_xy(self, lat, lon):
        if self.origin_lat is None:
            self.origin_lat = lat
            self.origin_lon = lon
            return 0.0, 0.0
        R_earth = 6378137.0
        dlat = math.radians(lat - self.origin_lat)
        dlon = math.radians(lon - self.origin_lon)
        lat0 = math.radians(self.origin_lat)
        x = dlon * math.cos(lat0) * R_earth
        y = dlat * R_earth
        return x, y

    def get_position_local(self):
        return np.array([self.x, self.y, 0.0])

    def get_yaw(self):
        return self.yaw


class USV_GuidanceNode(Node):
    def __init__(self):
        super().__init__('usv_guidance_node')

        # LISTE DES WAYPOINTS (Ajoute tes coordonnées X, Y ici)
        self.targets = [
            (-800.0, 300.0),
            (-720.0, 370.0),
            (-700.0, 450.0),
            (-750.0, 530.0),
            (-900.0, 510.0),
            (-800.0, 300.0)
        ]
        self.current_target_idx = 0

        self.declare_parameter('stop_distance', 8.0)
        self.declare_parameter('slow_distance', 25.0)
        self.declare_parameter('initial_x', -800.32)
        self.declare_parameter('initial_y', 300.02)

        self.stop_distance = self.get_parameter('stop_distance').value
        self.slow_distance = self.get_parameter('slow_distance').value
        self.initial_x = self.get_parameter('initial_x').value
        self.initial_y = self.get_parameter('initial_y').value

        self.get_logger().info(f"Nombre total de waypoints : {len(self.targets)}")
        self._log_current_target()

        self.estimator = SimplePositionEstimator()

        self.offset_x = None
        self.offset_y = None
        self.gps_received = False
        self.imu_received = False
        self.all_targets_reached = False

        self.trajectory_log = []
        self.start_time = None

        self.gps_sub = self.create_subscription(NavSatFix, '/wamv/sensors/gps/gps/fix', self.gps_cb, 10)
        self.imu_sub = self.create_subscription(Imu, '/wamv/sensors/imu/imu/data', self.imu_cb, 10)

        self.left_thruster_pub = self.create_publisher(Float64, '/wamv/thrusters/left/thrust', 10)
        self.right_thruster_pub = self.create_publisher(Float64, '/wamv/thrusters/right/thrust', 10)

        self.kp_yaw = 300.0
        self.max_thrust = 600.0

        self.timer = self.create_timer(0.1, self.control_loop)
        self.log_counter = 0

    def _log_current_target(self):
        tx, ty = self.targets[self.current_target_idx]
        self.get_logger().info(
            f"--> Navigation vers Waypoint {self.current_target_idx + 1}/{len(self.targets)} : X={tx:.1f}, Y={ty:.1f}"
        )

    def gps_cb(self, msg):
        if msg.status.status >= 0:
            x_gps_loc, y_gps_loc = self.estimator.update_gps_position(msg.latitude, msg.longitude)
            self.gps_received = True

            if self.offset_x is None:
                pos_local = self.estimator.get_position_local()
                self.offset_x = self.initial_x - pos_local[0]
                self.offset_y = self.initial_y - pos_local[1]
                self.start_time = self.get_clock().now().nanoseconds * 1e-9

            if self.offset_x is not None:
                t = self.get_clock().now().nanoseconds * 1e-9 - self.start_time
                pos_local = self.estimator.get_position_local()
                x_est_world = pos_local[0] + self.offset_x
                y_est_world = pos_local[1] + self.offset_y
                x_gps_world = x_gps_loc + self.offset_x
                y_gps_world = y_gps_loc + self.offset_y

                self.trajectory_log.append([t, x_est_world, y_est_world, x_gps_world, y_gps_world])

    def imu_cb(self, msg):
        self.estimator.update_yaw_from_quaternion(
            msg.orientation.x, msg.orientation.y, msg.orientation.z, msg.orientation.w
        )
        self.imu_received = True

    def control_loop(self):
        self.log_counter += 1

        if not (self.gps_received and self.imu_received and self.offset_x is not None):
            return

        if self.all_targets_reached:
            return

        pos_local = self.estimator.get_position_local()
        yaw = self.estimator.get_yaw()

        x_world = pos_local[0] + self.offset_x
        y_world = pos_local[1] + self.offset_y

        target_x, target_y = self.targets[self.current_target_idx]
        dx = target_x - x_world
        dy = target_y - y_world
        dist = math.hypot(dx, dy)

        # Vérification d'atteinte du waypoint actuel
        if dist <= self.stop_distance:
            self.get_logger().info(f"Waypoint {self.current_target_idx + 1} atteint ! ({dist:.1f}m)")

            # Passage au waypoint suivant
            if self.current_target_idx < len(self.targets) - 1:
                self.current_target_idx += 1
                self._log_current_target()
                return
            else:
                self.get_logger().info("Tous les waypoints ont été atteints ! Arrêt de la mission.")
                self.all_targets_reached = True
                self.publish_thrust(0.0, 0.0)
                self.save_trajectory()
                return

        psi_d = math.atan2(dy, dx)
        e_psi = psi_d - yaw
        e_psi = math.atan2(math.sin(e_psi), math.cos(e_psi))

        # Décélération si proche du dernier waypoint, sinon vitesse soutenue
        if self.current_target_idx == len(self.targets) - 1 and dist < self.slow_distance:
            ratio = (dist - self.stop_distance) / (self.slow_distance - self.stop_distance)
            T_base = 80.0 + ratio * 200.0
        else:
            T_base = self.max_thrust

        T_diff = self.kp_yaw * e_psi

        T_L = max(0.0, min(self.max_thrust, T_base - T_diff))
        T_R = max(0.0, min(self.max_thrust, T_base + T_diff))

        if self.log_counter % 10 == 0:
            self.get_logger().info(
                f"WP {self.current_target_idx + 1} | Dist: {dist:.1f}m | Cap err: {math.degrees(e_psi):.1f}deg"
            )

        self.publish_thrust(T_L, T_R)

    def publish_thrust(self, tl, tr):
        self.left_thruster_pub.publish(Float64(data=tl))
        self.right_thruster_pub.publish(Float64(data=tr))

    def save_trajectory(self):
        if not self.trajectory_log:
            return

        csv_filename = "trajectoire_usv_multi.csv"
        plot_filename = "trajectoire_usv_multi.png"

        with open(csv_filename, mode='w', newline='') as f:
            writer = csv.writer(f)
            writer.writerow(['temps_s', 'x_est', 'y_est', 'x_gps', 'y_gps'])
            writer.writerows(self.trajectory_log)
        self.get_logger().info(f"Données enregistrées dans '{os.path.abspath(csv_filename)}'")

        if HAS_MATPLOTLIB:
            data = np.array(self.trajectory_log)
            x_est, y_est = data[:, 1], data[:, 2]
            x_gps, y_gps = data[:, 3], data[:, 4]

            plt.figure(figsize=(10, 8))
            plt.plot(x_gps, y_gps, 'r.', alpha=0.2, label='GPS brut')
            plt.plot(x_est, y_est, 'b-', linewidth=2, label='Trajectoire estimée')
            plt.plot(self.initial_x, self.initial_y, 'go', markersize=10, label='Départ')

            # Affichage de tous les waypoints
            targets_np = np.array(self.targets)
            plt.plot(targets_np[:, 0], targets_np[:, 1], 'r--', alpha=0.5, label='Parcours prévu')
            for idx, (tx, ty) in enumerate(self.targets):
                plt.plot(tx, ty, 'rx', markersize=10, markeredgewidth=2)
                plt.text(tx + 1, ty + 1, f"WP {idx + 1}", fontsize=10, fontweight='bold')

            plt.title('Suivi de parcours multi-waypoints de l\'USV')
            plt.xlabel('X (mètres)')
            plt.ylabel('Y (mètres)')
            plt.grid(True)
            plt.legend()
            plt.axis('equal')
            plt.savefig(plot_filename, dpi=300)
            plt.close()
            self.get_logger().info(f"Graphique généré : '{os.path.abspath(plot_filename)}'")


def main(args=None):
    rclpy.init(args=args)
    node = USV_GuidanceNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.save_trajectory()
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()