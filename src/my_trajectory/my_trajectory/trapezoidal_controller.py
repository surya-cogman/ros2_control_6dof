#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
import numpy as np
from std_msgs.msg import Float64MultiArray


class TrapezoidalController(Node):
    def __init__(self):
        super().__init__('trapezoidal_controller')

        # ---- USER PARAMETERS ----
        self.dt = 0.01                 # control loop 100 Hz
        self.max_vel = 1.0             # rad/s
        self.max_acc = 3.0             # rad/s^2
        self.target_pos = np.array([0.0, -1.57, 1.0, 0.5, 0.0, 0.0])  # rad
        self.joint_count = len(self.target_pos)

        # Internal state
        self.current_pos = np.zeros(self.joint_count)
        self.current_vel = np.zeros(self.joint_count)

        self.publisher = self.create_publisher(
            Float64MultiArray,
            '/forward_command_controller/commands',
            10
        )

        self.timer = self.create_timer(self.dt, self.update)

        self.get_logger().info("Trapezoidal profile publisher started.")

    def update(self):
        new_pos = []

        for i in range(self.joint_count):
            p = self.current_pos[i]
            v = self.current_vel[i]
            pf = self.target_pos[i]

            # remaining distance
            dist = pf - p

            # Deceleration distance: v^2 / (2a)
            decel_dist = (v * v) / (2 * self.max_acc)

            # ---- Acceleration phase ----
            if abs(dist) > decel_dist:
                # accelerate toward target
                direction = np.sign(dist)
                v = v + direction * self.max_acc * self.dt
                v = np.clip(v, -self.max_vel, self.max_vel)
            else:
                # ---- Deceleration phase ----
                direction = np.sign(dist)
                v = v - direction * self.max_acc * self.dt

            # Integrate position
            p = p + v * self.dt

            # Save back
            self.current_pos[i] = p
            self.current_vel[i] = v

            new_pos.append(float(p))

        # Publish command
        msg = Float64MultiArray()
        msg.data = new_pos
        self.publisher.publish(msg)


def main(args=None):
    rclpy.init(args=args)
    node = TrapezoidalController()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
