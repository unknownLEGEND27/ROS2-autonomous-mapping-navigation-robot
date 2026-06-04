"""
pid_node — PID heading correction using IMU gyro-Z feedback.

Subscribes:
    /cmd_vel_raw    (geometry_msgs/Twist)  — from teleop
    /imu/data_raw   (sensor_msgs/Imu)      — from imu_node

Publishes:
    /cmd_vel        (geometry_msgs/Twist)  — corrected, to motor_node

How it works:
    - When driving STRAIGHT (teleop angular.z ≈ 0):
      PID locks the current heading using gyro-Z integration
      and injects a corrective angular.z to counteract drift.
      This fixes the "right motor faster" problem automatically.

    - When TURNING (teleop angular.z ≠ 0):
      PID is bypassed — raw command passes through.
      Heading resets when turning stops.

    - When STOPPED:
      PID is inactive, heading resets.
"""

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from sensor_msgs.msg import Imu
from std_msgs.msg import String


class PIDNode(Node):
    def __init__(self):
        super().__init__('pid_node')

        # ── PID gains ──
        # Start with Kp only, add Kd once straight-line works,
        # add Ki last (keep very small to avoid windup)
        self.declare_parameter('kp', 1.5)
        self.declare_parameter('ki', 0.0)
        self.declare_parameter('kd', 0.3)
        self.declare_parameter('integral_max', 0.5)
        self.declare_parameter('control_rate', 50.0)
        self.declare_parameter('dead_zone', 0.01)       # radians (~0.6°)
        self.declare_parameter('max_correction', 0.8)    # max angular.z output

        self.kp = self.get_parameter('kp').value
        self.ki = self.get_parameter('ki').value
        self.kd = self.get_parameter('kd').value
        self.integral_max = self.get_parameter('integral_max').value
        self.dead_zone = self.get_parameter('dead_zone').value
        self.max_correction = self.get_parameter('max_correction').value

        # ── PID state ──
        self.heading = 0.0           # integrated yaw (radians)
        self.desired_heading = 0.0
        self.integral = 0.0
        self.prev_error = 0.0
        self.last_imu_time = None
        self.pid_active = False
        self.latest_gyro_z = 0.0     # latest gyro reading (rad/s)

        # ── Latest teleop command ──
        self.latest_cmd = Twist()
        self.last_cmd_time = None    # timeout if no new cmd_vel_raw
        self.cmd_timeout = 0.5       # seconds

        # ── ROS interfaces ──
        self.sub_cmd_raw = self.create_subscription(
            Twist, '/cmd_vel_raw', self.cmd_raw_callback, 10
        )
        self.sub_imu = self.create_subscription(
            Imu, '/imu/data_raw', self.imu_callback, 10
        )
        self.pub_cmd = self.create_publisher(Twist, '/cmd_vel', 10)
        self.pub_debug = self.create_publisher(String, '/pid/debug', 10)

        rate = self.get_parameter('control_rate').value
        self.dt = 1.0 / rate
        self.create_timer(self.dt, self.control_loop)

        self.get_logger().info(
            f'pid_node ready — Kp={self.kp} Ki={self.ki} Kd={self.kd}'
        )

    def cmd_raw_callback(self, msg: Twist):
        """Cache latest teleop command. Decide PID mode."""
        self.latest_cmd = msg
        self.last_cmd_time = self.get_clock().now()

        if abs(msg.angular.z) > 0.05:
            # ── Intentional turn — disable PID ──
            if self.pid_active:
                self.get_logger().info('PID OFF — turning')
            self.pid_active = False
            self.integral = 0.0
            self.prev_error = 0.0

        elif abs(msg.linear.x) > 0.05:
            # ── Straight drive — activate PID ──
            if not self.pid_active:
                # Lock current heading as target
                self.desired_heading = self.heading
                self.integral = 0.0
                self.prev_error = 0.0
                self.pid_active = True
                self.get_logger().info(
                    f'PID ON — locked heading {self.desired_heading:.3f} rad'
                )
        else:
            # ── Stopped ──
            if self.pid_active:
                self.get_logger().info('PID OFF — stopped')
            self.pid_active = False
            self.integral = 0.0

    def imu_callback(self, msg: Imu):
        """Integrate gyro-Z to track heading."""
        now = self.get_clock().now()

        # Store latest gyro reading
        self.latest_gyro_z = msg.angular_velocity.z

        if self.last_imu_time is not None:
            dt = (now - self.last_imu_time).nanoseconds / 1e9
            if 0.001 < dt < 0.1:  # sanity: 10Hz to 1000Hz
                self.heading += msg.angular_velocity.z * dt
        self.last_imu_time = now

    def control_loop(self):
        """Run PID and publish corrected cmd_vel at fixed rate."""
        out = Twist()

        # ── Command timeout — stop if no teleop input ──
        if self.last_cmd_time is None:
            # Never received a command — publish zeros
            self.pub_cmd.publish(out)
            return

        elapsed = (self.get_clock().now() - self.last_cmd_time).nanoseconds / 1e9
        if elapsed > self.cmd_timeout:
            # No recent command — stop everything
            self.pid_active = False
            self.integral = 0.0
            self.latest_cmd = Twist()  # zero the cached command
            self.pub_cmd.publish(out)  # publish zeros to motor
            return

        out.linear.x = self.latest_cmd.linear.x

        if self.pid_active:
            # ── Heading error ──
            error = self.desired_heading - self.heading

            # Dead zone — ignore tiny drift
            if abs(error) < self.dead_zone:
                error = 0.0

            # ── PID terms ──
            # Proportional
            p_term = self.kp * error

            # Integral with anti-windup
            self.integral += error * self.dt
            self.integral = max(-self.integral_max,
                                min(self.integral_max, self.integral))
            i_term = self.ki * self.integral

            # Derivative
            d_term = self.kd * (error - self.prev_error) / self.dt
            self.prev_error = error

            # ── Total correction ──
            correction = p_term + i_term + d_term
            correction = max(-self.max_correction,
                             min(self.max_correction, correction))

            out.angular.z = correction

            # ── Debug output ──
            debug = String()
            debug.data = (
                f'hdg={self.heading:.3f} des={self.desired_heading:.3f} '
                f'err={error:.4f} P={p_term:.3f} I={i_term:.3f} '
                f'D={d_term:.3f} corr={correction:.3f} '
                f'gyro_z={self.latest_gyro_z:.4f}'
            )
            self.pub_debug.publish(debug)
        else:
            # ── Pass through raw angular command ──
            out.angular.z = self.latest_cmd.angular.z

        self.pub_cmd.publish(out)


def main(args=None):
    rclpy.init(args=args)
    node = PIDNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()