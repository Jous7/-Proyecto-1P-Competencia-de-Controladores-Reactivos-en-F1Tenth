import rclpy
from rclpy.node import Node
from sensor_msgs.msg import LaserScan
from ackermann_msgs.msg import AckermannDriveStamped
from nav_msgs.msg import Odometry
import numpy as np
import time

class FollowTheGap(Node):
    def __init__(self):
        super().__init__('follow_the_gap')

        # Parámetros de velocidad y dirección
        self.speed_base = 5.0
        self.speed_straight = 20.0
        self.speed_min = 1.0
        self.steering_gain = 0.62

        # Subs y pubs
        self.scan_sub = self.create_subscription(LaserScan, '/scan', self.lidar_callback, 10)
        self.odom_sub = self.create_subscription(Odometry, '/ego_racecar/odom', self.odom_callback, 10)
        self.drive_pub = self.create_publisher(AckermannDriveStamped, '/drive', 10)

        # Parámetros de LIDAR y suavizado
        self.max_range = 9.0
        self.bubble_radius = 0.5
        self.angle_window_size = 20
        self.prev_angles = []
        self.last_steering_angle = 0.0
        self.min_angle_change = 0.03
        self.max_delta = 0.3

        # Para vueltas
        self.lap_position = (0.0, 0.0)  # ¡AJUSTA esta posición a tu punto de partida/meta!
        self.lap_radius = 1.5
        self.lap_triggered = False
        self.start_time = time.time()
        self.last_lap_time = self.start_time
        self.lap_count = 0

    def odom_callback(self, msg):
        x = msg.pose.pose.position.x
        y = msg.pose.pose.position.y

        # Medir distancia a la posición de meta
        dx = x - self.lap_position[0]
        dy = y - self.lap_position[1]
        distance = np.sqrt(dx ** 2 + dy ** 2)

        if distance < self.lap_radius:
            if not self.lap_triggered:
                now = time.time()
                lap_time = now - self.last_lap_time
                self.lap_count += 1
                self.last_lap_time = now
                self.lap_triggered = True
                self.get_logger().info(f"🏁 Vuelta {self.lap_count-1} completada en {lap_time:.2f} segundos")
        else:
            self.lap_triggered = False

    def lidar_callback(self, msg):
        ranges = np.array(msg.ranges)
        angle_increment = msg.angle_increment
        angle_min = msg.angle_min

        # Limpiar el rango y dejar solo el frente
        ranges = np.clip(ranges, 0.0, self.max_range)
        total_len = len(ranges)
        center = total_len // 2
        fov_range = int(total_len * 0.18)
        ranges[:center - fov_range] = 0.0
        ranges[center + fov_range:] = 0.0

        valid_ranges = ranges[ranges > 0]
        if len(valid_ranges) == 0:
            return

        # Burbujear el obstáculo más cercano
        min_index = np.argmin(ranges + (ranges == 0) * self.max_range * 2)
        bubble_indices = int(self.bubble_radius / angle_increment)
        start = max(0, min_index - bubble_indices)
        end = min(len(ranges), min_index + bubble_indices)
        ranges[start:end] = 0.0

        # Encontrar el gap más grande
        masked = np.ma.masked_where(ranges == 0.0, ranges)
        slices = np.ma.clump_unmasked(masked)
        if not slices:
            return

        def gap_score(s):
            gap_size = s.stop - s.start
            gap_center = (s.start + s.stop) // 2
            angle = angle_min + gap_center * angle_increment
            return gap_size - abs(angle) * 1.0

        largest_gap = max(slices, key=gap_score)
        best_index = (largest_gap.start + largest_gap.stop) // 2
        target_angle = angle_min + best_index * angle_increment

        # Suavizado
        self.prev_angles.append(target_angle)
        if len(self.prev_angles) > self.angle_window_size:
            self.prev_angles.pop(0)
        smoothed_angle = sum(self.prev_angles) / len(self.prev_angles)

        # Filtro por cambio mínimo
        if abs(smoothed_angle - self.last_steering_angle) < self.min_angle_change:
            smoothed_angle = self.last_steering_angle

        # Delta máximo por frame
        delta = smoothed_angle - self.last_steering_angle
        if delta > self.max_delta:
            smoothed_angle = self.last_steering_angle + self.max_delta
        elif delta < -self.max_delta:
            smoothed_angle = self.last_steering_angle - self.max_delta

        # Suavizado en rectas
        front_slice = ranges[center - 10:center + 10]
        front_min = np.min(front_slice) if len(front_slice) > 0 else self.max_range
        if abs(smoothed_angle) < 0.06 and front_min > 4.5:
            smoothed_angle *= 0.5

        self.last_steering_angle = smoothed_angle

        # Ajustar velocidad
        is_straight = front_min > 4.5 and abs(smoothed_angle) < 0.07
        is_blocked = front_min < 2.0

        if is_straight:
            speed = self.speed_straight
        elif is_blocked:
            speed = self.speed_min
        else:
            speed = max(self.speed_min, self.speed_base - abs(smoothed_angle) * 2.0)

        # Publicar movimiento
        steering_command = self.steering_gain * smoothed_angle
        drive_msg = AckermannDriveStamped()
        drive_msg.drive.speed = speed
        drive_msg.drive.steering_angle = steering_command
        self.drive_pub.publish(drive_msg)

        # Mostrar en consola (opcional)
        # self.get_logger().info(f"Steering: {steering_command:.3f} | Speed: {speed:.2f} | FrontMin: {front_min:.2f}")

def main(args=None):
    rclpy.init(args=args)
    node = FollowTheGap()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()

