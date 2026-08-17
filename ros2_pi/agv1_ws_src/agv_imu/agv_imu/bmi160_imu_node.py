#!/usr/bin/env python3

import math
import time
from typing import Optional

try:
    import smbus
except Exception:
    smbus = None

try:
    import smbus2
except Exception:
    smbus2 = None

import rclpy
from rclpy.node import Node

from sensor_msgs.msg import Imu
from std_msgs.msg import Float32, Bool, String


class BMI160:
    CHIP_ID_REG = 0x00
    CHIP_ID_VALUE = 0xD1

    GYR_X_LSB = 0x0C
    ACC_X_LSB = 0x12

    ACC_CONF = 0x40
    ACC_RANGE = 0x41
    GYR_CONF = 0x42
    GYR_RANGE = 0x43
    CMD = 0x7E

    CMD_ACC_NORMAL = 0x11
    CMD_GYR_NORMAL = 0x15

    def __init__(self, bus_number: int, address: int):
        self.bus_number = bus_number
        self.address = address

        if smbus is not None:
            self.bus = smbus.SMBus(bus_number)
        elif smbus2 is not None:
            self.bus = smbus2.SMBus(bus_number)
        else:
            raise RuntimeError("Neither smbus nor smbus2 is installed")

    def read_u8(self, reg: int) -> int:
        return self.bus.read_byte_data(self.address, reg)

    def write_u8(self, reg: int, value: int):
        self.bus.write_byte_data(self.address, reg, value & 0xFF)

    def read_block(self, reg: int, length: int):
        return self.bus.read_i2c_block_data(self.address, reg, length)

    def read_i16_from_block(self, data, index: int) -> int:
        value = data[index] | (data[index + 1] << 8)

        if value & 0x8000:
            value -= 65536

        return value

    def initialize(self):
        chip_id = self.read_u8(self.CHIP_ID_REG)

        if chip_id != self.CHIP_ID_VALUE:
            raise RuntimeError(
                f"BMI160 chip ID mismatch. Expected 0x{self.CHIP_ID_VALUE:02X}, got 0x{chip_id:02X}"
            )

        # Put accelerometer and gyro into normal mode.
        self.write_u8(self.CMD, self.CMD_ACC_NORMAL)
        time.sleep(0.1)

        self.write_u8(self.CMD, self.CMD_GYR_NORMAL)
        time.sleep(0.1)

        # ODR/bandwidth config. 0x28 is a common stable 100 Hz normal filtering setting.
        self.write_u8(self.ACC_CONF, 0x28)
        time.sleep(0.01)

        self.write_u8(self.GYR_CONF, 0x28)
        time.sleep(0.01)

        # Accelerometer range: +/-2g
        self.write_u8(self.ACC_RANGE, 0x03)
        time.sleep(0.01)

        # Gyro range: +/-250 deg/s
        self.write_u8(self.GYR_RANGE, 0x03)
        time.sleep(0.01)

    def read_gyro_raw(self):
        data = self.read_block(self.GYR_X_LSB, 6)

        gx = self.read_i16_from_block(data, 0)
        gy = self.read_i16_from_block(data, 2)
        gz = self.read_i16_from_block(data, 4)

        return gx, gy, gz

    def read_accel_raw(self):
        data = self.read_block(self.ACC_X_LSB, 6)

        ax = self.read_i16_from_block(data, 0)
        ay = self.read_i16_from_block(data, 2)
        az = self.read_i16_from_block(data, 4)

        return ax, ay, az


class BMI160ImuNode(Node):
    def __init__(self):
        super().__init__("bmi160_imu_node")

        self.declare_parameter("i2c_bus", 1)
        self.declare_parameter("i2c_address", 0x69)
        self.declare_parameter("frame_id", "imu_link")
        self.declare_parameter("publish_rate_hz", 50.0)
        self.declare_parameter("calibration_samples", 300)

        self.i2c_bus = int(self.get_parameter("i2c_bus").value)
        self.i2c_address = int(self.get_parameter("i2c_address").value)
        self.frame_id = str(self.get_parameter("frame_id").value)
        self.publish_rate_hz = float(self.get_parameter("publish_rate_hz").value)
        self.calibration_samples = int(self.get_parameter("calibration_samples").value)

        self.imu: Optional[BMI160] = None

        self.connected = False
        self.gyro_bias_x = 0.0
        self.gyro_bias_y = 0.0
        self.gyro_bias_z = 0.0

        self.yaw_rad = 0.0
        self.last_time = None

        # BMI160 scale settings used in initialize():
        # Gyro +/-250 deg/s => 131.2 LSB per deg/s
        # Accel +/-2g => 16384 LSB per g
        self.gyro_lsb_per_dps = 131.2
        self.accel_lsb_per_g = 16384.0
        self.g = 9.80665

        self.imu_pub = self.create_publisher(Imu, "/agv_1/imu/data_raw", 10)
        self.yaw_pub = self.create_publisher(Float32, "/agv_1/yaw", 10)
        self.yaw_deg_pub = self.create_publisher(Float32, "/agv_1/yaw_deg", 10)
        self.connected_pub = self.create_publisher(Bool, "/agv_1/imu/connected", 10)
        self.status_pub = self.create_publisher(String, "/agv_1/imu/status", 10)

        self.connect_and_initialize()

        period = 1.0 / self.publish_rate_hz
        self.timer = self.create_timer(period, self.timer_callback)

        self.get_logger().info("BMI160 IMU node started")
        self.get_logger().info(f"I2C bus: {self.i2c_bus}")
        self.get_logger().info(f"I2C address: 0x{self.i2c_address:02X}")
        self.get_logger().info(f"Publish rate: {self.publish_rate_hz} Hz")

    def connect_and_initialize(self):
        try:
            self.imu = BMI160(self.i2c_bus, self.i2c_address)
            self.imu.initialize()
            self.connected = True
            self.publish_status("BMI160 connected and initialized")
            self.publish_connected(True)
            self.calibrate_gyro_bias()

        except Exception as exc:
            self.connected = False
            self.imu = None
            self.publish_connected(False)
            self.publish_status(f"BMI160 init failed: {exc}")
            self.get_logger().error(f"BMI160 init failed: {exc}")

    def calibrate_gyro_bias(self):
        if self.imu is None:
            return

        self.get_logger().info("Calibrating gyro bias. Keep robot still.")

        sx = 0.0
        sy = 0.0
        sz = 0.0

        valid_samples = 0

        for _ in range(self.calibration_samples):
            try:
                gx_raw, gy_raw, gz_raw = self.imu.read_gyro_raw()

                sx += self.raw_gyro_to_rad_s(gx_raw)
                sy += self.raw_gyro_to_rad_s(gy_raw)
                sz += self.raw_gyro_to_rad_s(gz_raw)

                valid_samples += 1
                time.sleep(0.005)

            except Exception:
                pass

        if valid_samples > 0:
            self.gyro_bias_x = sx / valid_samples
            self.gyro_bias_y = sy / valid_samples
            self.gyro_bias_z = sz / valid_samples

        self.yaw_rad = 0.0
        self.last_time = time.time()

        self.get_logger().info(
            f"Gyro bias rad/s: x={self.gyro_bias_x:.6f}, y={self.gyro_bias_y:.6f}, z={self.gyro_bias_z:.6f}"
        )

    def timer_callback(self):
        if not self.connected or self.imu is None:
            self.connect_and_initialize()
            return

        try:
            now = time.time()

            gx_raw, gy_raw, gz_raw = self.imu.read_gyro_raw()
            ax_raw, ay_raw, az_raw = self.imu.read_accel_raw()

            gx = self.raw_gyro_to_rad_s(gx_raw) - self.gyro_bias_x
            gy = self.raw_gyro_to_rad_s(gy_raw) - self.gyro_bias_y
            gz = self.raw_gyro_to_rad_s(gz_raw) - self.gyro_bias_z

            ax = self.raw_accel_to_m_s2(ax_raw)
            ay = self.raw_accel_to_m_s2(ay_raw)
            az = self.raw_accel_to_m_s2(az_raw)

            if self.last_time is None:
                dt = 0.0
            else:
                dt = now - self.last_time

            self.last_time = now

            # Integrate yaw from gyro Z.
            # This will drift slowly over time. It is still useful for short 90-degree turns.
            if 0.0 < dt < 0.2:
                self.yaw_rad += gz * dt
                self.yaw_rad = self.wrap_angle(self.yaw_rad)

            self.publish_imu(gx, gy, gz, ax, ay, az)
            self.publish_yaw()

        except Exception as exc:
            self.connected = False
            self.publish_connected(False)
            self.publish_status(f"BMI160 read failed: {exc}")
            self.get_logger().warn(f"BMI160 read failed: {exc}")

    def publish_imu(self, gx, gy, gz, ax, ay, az):
        msg = Imu()

        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = self.frame_id

        # No fused orientation yet.
        msg.orientation_covariance[0] = -1.0

        msg.angular_velocity.x = gx
        msg.angular_velocity.y = gy
        msg.angular_velocity.z = gz

        msg.linear_acceleration.x = ax
        msg.linear_acceleration.y = ay
        msg.linear_acceleration.z = az

        # Basic placeholder covariances.
        msg.angular_velocity_covariance[0] = 0.02
        msg.angular_velocity_covariance[4] = 0.02
        msg.angular_velocity_covariance[8] = 0.02

        msg.linear_acceleration_covariance[0] = 0.2
        msg.linear_acceleration_covariance[4] = 0.2
        msg.linear_acceleration_covariance[8] = 0.2

        self.imu_pub.publish(msg)

    def publish_yaw(self):
        yaw_msg = Float32()
        yaw_msg.data = float(self.yaw_rad)
        self.yaw_pub.publish(yaw_msg)

        yaw_deg_msg = Float32()
        yaw_deg_msg.data = float(math.degrees(self.yaw_rad))
        self.yaw_deg_pub.publish(yaw_deg_msg)

    def publish_connected(self, connected: bool):
        msg = Bool()
        msg.data = bool(connected)
        self.connected_pub.publish(msg)

    def publish_status(self, text: str):
        msg = String()
        msg.data = text
        self.status_pub.publish(msg)

    def raw_gyro_to_rad_s(self, raw_value: int) -> float:
        deg_s = raw_value / self.gyro_lsb_per_dps
        return math.radians(deg_s)

    def raw_accel_to_m_s2(self, raw_value: int) -> float:
        g_value = raw_value / self.accel_lsb_per_g
        return g_value * self.g

    @staticmethod
    def wrap_angle(angle_rad: float) -> float:
        while angle_rad > math.pi:
            angle_rad -= 2.0 * math.pi

        while angle_rad < -math.pi:
            angle_rad += 2.0 * math.pi

        return angle_rad


def main(args=None):
    rclpy.init(args=args)

    node = BMI160ImuNode()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
