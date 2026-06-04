"""
imu_node — Reads MPU6500/9250 over I²C on Pi 5, publishes sensor_msgs/Imu.

Publishes: /imu/data_raw (sensor_msgs/Imu)

I2C wiring:
    SDA → GPIO 2 (pin 3)
    SCL → GPIO 3 (pin 5)
    VCC → 3.3V
    GND → GND

Run calibration FIRST:  python3 imu_calibrate.py
Then paste offsets into config/imu_params.yaml
"""

import math
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Imu

try:
    import smbus2
    I2C_AVAILABLE = True
except ImportError:
    I2C_AVAILABLE = False

# MPU registers
REG_PWR_MGMT_1 = 0x6B
REG_GYRO_CONFIG = 0x1B
REG_ACCEL_CONFIG = 0x1C
REG_ACCEL_XOUT_H = 0x3B

GYRO_SCALES = [131.0, 65.5, 32.8, 16.4]       # LSB/(°/s)
ACCEL_SCALES = [16384.0, 8192.0, 4096.0, 2048.0]  # LSB/g
DEG_TO_RAD = math.pi / 180.0
G_TO_MS2 = 9.80665


class ImuNode(Node):
    def __init__(self):
        super().__init__('imu_node')

        # ── Parameters ──
        self.declare_parameter('i2c_bus', 1)
        self.declare_parameter('i2c_address', 0x68)
        self.declare_parameter('publish_rate', 50.0)
        self.declare_parameter('frame_id', 'imu_link')
        self.declare_parameter('gyro_offset_x', 0.0)
        self.declare_parameter('gyro_offset_y', 0.0)
        self.declare_parameter('gyro_offset_z', 0.0)
        self.declare_parameter('accel_offset_x', 0.0)
        self.declare_parameter('accel_offset_y', 0.0)
        self.declare_parameter('accel_offset_z', 0.0)
        self.declare_parameter('gyro_range', 1)
        self.declare_parameter('accel_range', 0)

        self.bus_num = self.get_parameter('i2c_bus').value
        self.address = self.get_parameter('i2c_address').value
        self.frame_id = self.get_parameter('frame_id').value
        gyro_range = self.get_parameter('gyro_range').value
        accel_range = self.get_parameter('accel_range').value

        self.gyro_offset = [
            self.get_parameter('gyro_offset_x').value,
            self.get_parameter('gyro_offset_y').value,
            self.get_parameter('gyro_offset_z').value,
        ]
        self.accel_offset = [
            self.get_parameter('accel_offset_x').value,
            self.get_parameter('accel_offset_y').value,
            self.get_parameter('accel_offset_z').value,
        ]

        self.gyro_scale = GYRO_SCALES[gyro_range]
        self.accel_scale = ACCEL_SCALES[accel_range]

        # ── I2C init ──
        self.bus = None
        if I2C_AVAILABLE:
            try:
                self.bus = smbus2.SMBus(self.bus_num)
                # Wake up MPU
                self.bus.write_byte_data(self.address, REG_PWR_MGMT_1, 0x00)
                # Set ranges
                self.bus.write_byte_data(
                    self.address, REG_GYRO_CONFIG, gyro_range << 3
                )
                self.bus.write_byte_data(
                    self.address, REG_ACCEL_CONFIG, accel_range << 3
                )

                # Verify connection
                who = self.bus.read_byte_data(self.address, 0x75)
                self.get_logger().info(
                    f'MPU on I2C bus {self.bus_num} addr 0x{self.address:02X} '
                    f'WHO_AM_I=0x{who:02X}'
                )

            except Exception as e:
                self.get_logger().error(f'I2C init failed: {e}')
                self.get_logger().error(
                    'Check: sudo i2cdetect -y 1  (should show 68)'
                )
                self.bus = None
        else:
            self.get_logger().warn(
                'smbus2 not installed — simulation mode. '
                'Install: sudo apt install python3-smbus2'
            )

        # ── Publisher ──
        self.pub_imu = self.create_publisher(Imu, '/imu/data_raw', 10)

        # ── Timer ──
        rate = self.get_parameter('publish_rate').value
        self.create_timer(1.0 / rate, self.publish_imu)

        self.read_errors = 0
        self.get_logger().info(f'imu_node running at {rate} Hz')

    def _read_raw(self):
        """Read 14 bytes of accel + temp + gyro from MPU."""
        data = self.bus.read_i2c_block_data(
            self.address, REG_ACCEL_XOUT_H, 14
        )

        def to_int16(h, l):
            val = (h << 8) | l
            return val - 65536 if val > 32767 else val

        ax = to_int16(data[0], data[1])
        ay = to_int16(data[2], data[3])
        az = to_int16(data[4], data[5])
        gx = to_int16(data[8], data[9])
        gy = to_int16(data[10], data[11])
        gz = to_int16(data[12], data[13])

        return ax, ay, az, gx, gy, gz

    def publish_imu(self):
        msg = Imu()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = self.frame_id

        if self.bus is not None:
            try:
                ax, ay, az, gx, gy, gz = self._read_raw()
                self.read_errors = 0
            except Exception as e:
                self.read_errors += 1
                if self.read_errors <= 3:
                    self.get_logger().warn(f'I2C read error: {e}')
                elif self.read_errors == 10:
                    self.get_logger().error('10 consecutive I2C errors — check wiring')
                return
        else:
            ax = ay = gx = gy = gz = 0
            az = int(self.accel_scale)

        # Convert to SI units with calibration
        msg.angular_velocity.x = (gx / self.gyro_scale - self.gyro_offset[0]) * DEG_TO_RAD
        msg.angular_velocity.y = (gy / self.gyro_scale - self.gyro_offset[1]) * DEG_TO_RAD
        msg.angular_velocity.z = (gz / self.gyro_scale - self.gyro_offset[2]) * DEG_TO_RAD

        msg.linear_acceleration.x = (ax / self.accel_scale - self.accel_offset[0]) * G_TO_MS2
        msg.linear_acceleration.y = (ay / self.accel_scale - self.accel_offset[1]) * G_TO_MS2
        msg.linear_acceleration.z = (az / self.accel_scale - self.accel_offset[2]) * G_TO_MS2

        # No orientation estimate — set covariance[0] = -1
        msg.orientation_covariance[0] = -1.0

        self.pub_imu.publish(msg)

    def destroy_node(self):
        if self.bus is not None:
            self.bus.close()
        super().destroy_node()


def main(args=None):
    rclpy.init(args=args)
    node = ImuNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()