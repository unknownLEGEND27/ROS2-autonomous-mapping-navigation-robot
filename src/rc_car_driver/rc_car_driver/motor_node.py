"""
motor_node DEBUG version — mirrors the working test script exactly.
Run with: ros2 run rc_car_driver motor_node
"""

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from std_msgs.msg import String

try:
    import lgpio
    GPIO_AVAILABLE = True
    print("[DEBUG] lgpio imported OK")
except ImportError as e:
    GPIO_AVAILABLE = False
    print(f"[DEBUG] lgpio import FAILED: {e}")


class MotorNode(Node):
    def __init__(self):
        super().__init__('motor_node') 

        # ── Hardcoded pins — EXACTLY matching your test script ──
        self.ENA = 18
        self.IN1 = 24
        self.IN2 = 25
        self.ENB = 19
        self.IN3 = 17
        self.IN4 = 27

        self.pwm_freq = 1000
        self.max_pwm = 80           # capped at 60% — matches your test script
        self.turn_pwm = 80          # for j/l pure rotation
        self.min_pwm = 25
        self.max_linear = 0.5       # teleop default linear speed
        self.max_angular = 1.0      # teleop default angular speed
        self.wheel_base = 0.18
        self.watchdog_timeout = 0.5

        # Motor trim — set to 1.0 (no trim) first.
        # If right motor is still faster at 60% PWM, lower this (e.g. 0.90).
        self.right_trim = 1.0

        # ── GPIO setup — same as your test script ──
        self.chip = None
        if GPIO_AVAILABLE:
            try:
                self.chip = lgpio.gpiochip_open(4)
                self.get_logger().info(f'gpiochip_open(4) OK — handle={self.chip}')

                pins = [self.ENA, self.IN1, self.IN2,
                        self.ENB, self.IN3, self.IN4]
                for pin in pins:
                    lgpio.gpio_claim_output(self.chip, pin)
                    self.get_logger().info(f'  claimed pin {pin} OK')

                self.get_logger().info('ALL GPIO READY — hardware mode')

                # ── Quick self-test: pulse motors for 0.3s ──
                self.get_logger().info('Running self-test: forward 0.3s...')
                lgpio.tx_pwm(self.chip, self.ENA, 1000, 40)
                lgpio.tx_pwm(self.chip, self.ENB, 1000, 40)
                lgpio.gpio_write(self.chip, self.IN1, 1)
                lgpio.gpio_write(self.chip, self.IN2, 0)
                lgpio.gpio_write(self.chip, self.IN3, 0)
                lgpio.gpio_write(self.chip, self.IN4, 1)
                import time
                time.sleep(0.3)
                # Stop
                lgpio.gpio_write(self.chip, self.IN1, 0)
                lgpio.gpio_write(self.chip, self.IN2, 0)
                lgpio.gpio_write(self.chip, self.IN3, 0)
                lgpio.gpio_write(self.chip, self.IN4, 0)
                lgpio.tx_pwm(self.chip, self.ENA, 1000, 0)
                lgpio.tx_pwm(self.chip, self.ENB, 1000, 0)
                self.get_logger().info('Self-test done — did wheels twitch?')

            except Exception as e:
                self.get_logger().error(f'GPIO INIT FAILED: {e}')
                import traceback
                traceback.print_exc()
                self.chip = None
        else:
            self.get_logger().warn('NO lgpio — SIMULATION MODE (nothing will move)')

        # ── ROS interfaces ──
        self.sub_cmd = self.create_subscription(
            Twist, '/cmd_vel', self.cmd_vel_callback, 10
        )
        self.pub_status = self.create_publisher(String, '/motor/status', 10)

        # ── Watchdog ──
        self.last_cmd_time = self.get_clock().now()
        self.create_timer(0.1, self.watchdog_check)

        self.get_logger().info('motor_node ready — waiting for /cmd_vel')

    def cmd_vel_callback(self, msg: Twist):
        self.last_cmd_time = self.get_clock().now()

        linear = msg.linear.x
        angular = msg.angular.z

        # Normalise inputs to [-1, 1]
        lin = max(-1.0, min(1.0, linear / self.max_linear))
        ang = max(-1.0, min(1.0, angular / self.max_angular))

        if abs(lin) < 0.05:
            # ── Pure rotation (j / l keys) ──
            # Motor A (ENA) = physical RIGHT, Motor B (ENB) = physical LEFT
            # positive angular = turn left = right forward, left backward
            left_pwm = ang * self.turn_pwm
            right_pwm = -ang * self.turn_pwm
        else:
            # ── Moving forward/backward with turning ──
            # Both sides same direction, turning side slowed down
            base_pwm = lin * self.max_pwm   # signed: +forward, -backward
            slow_factor = 1.0 - abs(ang) * 0.5  # at full turn: 50% speed
            slow_factor = max(slow_factor, 0.5)  # never below 50%

            if ang > 0.05:
                # Turning left → slow down RIGHT side (Motor B)
                left_pwm = base_pwm
                right_pwm = base_pwm * slow_factor
            elif ang < -0.05:
                # Turning right → slow down LEFT side (Motor A)
                left_pwm = base_pwm * slow_factor
                right_pwm = base_pwm
            else:
                # Straight
                left_pwm = base_pwm
                right_pwm = base_pwm

        self.get_logger().info(
            f'CMD: lin={linear:.2f} ang={angular:.2f} '
            f'-> L={left_pwm:.1f}% R={right_pwm:.1f}%'
        )

        self._drive(left_pwm, right_pwm)

    def _drive(self, left_pwm, right_pwm):
        if self.chip is None:
            self.get_logger().warn(f'[SIM] L={left_pwm:.1f}% R={right_pwm:.1f}%')
            return

        # Apply trim — right motor is physically faster
        right_pwm_trimmed = right_pwm * self.right_trim

        # Set speed FIRST (same as test script's set_speed())
        lgpio.tx_pwm(self.chip, self.ENA, self.pwm_freq, abs(left_pwm))
        lgpio.tx_pwm(self.chip, self.ENB, self.pwm_freq, abs(right_pwm_trimmed))

        # Left motor direction
        if left_pwm > 0:       # forward
            lgpio.gpio_write(self.chip, self.IN1, 1)
            lgpio.gpio_write(self.chip, self.IN2, 0)
        elif left_pwm < 0:     # backward
            lgpio.gpio_write(self.chip, self.IN1, 0)
            lgpio.gpio_write(self.chip, self.IN2, 1)
        else:                  # stop
            lgpio.gpio_write(self.chip, self.IN1, 0)
            lgpio.gpio_write(self.chip, self.IN2, 0)

        # Right motor direction — INVERTED (same as test script)
        if right_pwm > 0:      # forward
            lgpio.gpio_write(self.chip, self.IN3, 0)
            lgpio.gpio_write(self.chip, self.IN4, 1)
        elif right_pwm < 0:    # backward
            lgpio.gpio_write(self.chip, self.IN3, 1)
            lgpio.gpio_write(self.chip, self.IN4, 0)
        else:                  # stop
            lgpio.gpio_write(self.chip, self.IN3, 0)
            lgpio.gpio_write(self.chip, self.IN4, 0)

        # Publish status
        status = String()
        status.data = f'L={left_pwm:.1f}% R={right_pwm_trimmed:.1f}% (trim={self.right_trim})'
        self.pub_status.publish(status)

    def _stop_all(self):
        if self.chip is None:
            return
        lgpio.gpio_write(self.chip, self.IN1, 0)
        lgpio.gpio_write(self.chip, self.IN2, 0)
        lgpio.gpio_write(self.chip, self.IN3, 0)
        lgpio.gpio_write(self.chip, self.IN4, 0)
        lgpio.tx_pwm(self.chip, self.ENA, self.pwm_freq, 0)
        lgpio.tx_pwm(self.chip, self.ENB, self.pwm_freq, 0)

    def watchdog_check(self):
        elapsed = (self.get_clock().now() - self.last_cmd_time).nanoseconds / 1e9
        if elapsed > self.watchdog_timeout:
            self._stop_all()

    def destroy_node(self):
        self._stop_all()
        if self.chip is not None:
            lgpio.gpiochip_close(self.chip)
            self.get_logger().info('GPIO released')
        super().destroy_node()


def main(args=None):
    rclpy.init(args=args)
    node = MotorNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()