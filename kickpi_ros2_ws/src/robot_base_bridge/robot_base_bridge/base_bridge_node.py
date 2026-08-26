import math
import queue
import struct
import threading
import time

import rclpy
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
from rclpy.node import Node
from sensor_msgs.msg import BatteryState, Imu, RelativeHumidity, Temperature
from std_msgs.msg import UInt16

try:
    import serial
except ImportError:  # pragma: no cover
    serial = None


FRAME_HEAD = b"\xAA\x55"
VERSION = 0x01
MAX_PAYLOAD = 64

CMD_SET_MOTION = 0x01
CMD_SET_STATE = 0x02
CMD_GET_STATUS = 0x03
CMD_SET_OUTPUT = 0x04
CMD_ESTOP = 0x05
CMD_STATUS = 0x81

MOTION_MODE_PWM = 0
MOTION_MODE_SPEED = 1

STATE_CMD_IDLE = 0
STATE_CMD_ENABLE = 1
STATE_CMD_CLEAR_FAULT = 2

SENSOR_INA219_VALID = 0x0001
SENSOR_DHT30_VALID = 0x0002
SENSOR_MPU6050_VALID = 0x0004
SENSOR_DS18B20_VALID = 0x0008


def crc16_ccitt(data: bytes, seed: int = 0xFFFF) -> int:
    crc = seed
    for value in data:
        crc ^= value << 8
        for _ in range(8):
            if crc & 0x8000:
                crc = ((crc << 1) ^ 0x1021) & 0xFFFF
            else:
                crc = (crc << 1) & 0xFFFF
    return crc


def make_frame(command: int, payload: bytes = b"") -> bytes:
    if len(payload) > MAX_PAYLOAD:
        raise ValueError("payload too long")
    header = bytes([VERSION, command, len(payload)])
    crc = crc16_ccitt(header + payload)
    return FRAME_HEAD + header + payload + struct.pack("<H", crc)


class FrameParser:
    def __init__(self):
        self.buffer = bytearray()

    def feed(self, data: bytes):
        frames = []
        self.buffer.extend(data)

        while len(self.buffer) >= 7:
            start = self.buffer.find(FRAME_HEAD)
            if start < 0:
                del self.buffer[:]
                break
            if start > 0:
                del self.buffer[:start]

            if len(self.buffer) < 5:
                break

            version = self.buffer[2]
            command = self.buffer[3]
            length = self.buffer[4]
            if version != VERSION or length > MAX_PAYLOAD:
                del self.buffer[0]
                continue

            total = 2 + 3 + length + 2
            if len(self.buffer) < total:
                break

            payload = bytes(self.buffer[5 : 5 + length])
            rx_crc = struct.unpack_from("<H", self.buffer, 5 + length)[0]
            calc_crc = crc16_ccitt(bytes([version, command, length]) + payload)
            if rx_crc == calc_crc:
                frames.append((command, payload))
                del self.buffer[:total]
            else:
                del self.buffer[0]

        return frames


class RobotBaseBridge(Node):
    def __init__(self):
        super().__init__("robot_base_bridge")

        self.declare_parameter("port", "/dev/ttyS1")
        self.declare_parameter("baud", 115200)
        self.declare_parameter("wheel_base_m", 0.16)
        self.declare_parameter("wheel_diameter_m", 0.065)
        self.declare_parameter("encoder_counts_per_rev", 1320)
        self.declare_parameter("command_timeout_s", 0.5)
        self.declare_parameter("use_pwm_mode", False)
        self.declare_parameter("max_pwm", 1000)
        self.declare_parameter("max_wheel_speed_mm_s", 500)
        self.declare_parameter("publish_odom", True)
        self.declare_parameter("frame_id", "odom")
        self.declare_parameter("base_frame_id", "base_link")

        self.port = self.get_parameter("port").value
        self.baud = int(self.get_parameter("baud").value)
        self.wheel_base_m = float(self.get_parameter("wheel_base_m").value)
        self.wheel_diameter_m = float(self.get_parameter("wheel_diameter_m").value)
        self.counts_per_rev = int(self.get_parameter("encoder_counts_per_rev").value)
        self.command_timeout_s = float(self.get_parameter("command_timeout_s").value)
        self.use_pwm_mode = bool(self.get_parameter("use_pwm_mode").value)
        self.max_pwm = int(self.get_parameter("max_pwm").value)
        self.max_wheel_speed_mm_s = int(self.get_parameter("max_wheel_speed_mm_s").value)
        self.publish_odom = bool(self.get_parameter("publish_odom").value)
        self.frame_id = self.get_parameter("frame_id").value
        self.base_frame_id = self.get_parameter("base_frame_id").value

        self.serial_lock = threading.Lock()
        self.serial_port = None
        self.stop_event = threading.Event()
        self.parser = FrameParser()
        self.status_queue = queue.Queue(maxsize=16)

        self.last_cmd_time = 0.0
        self.last_twist = Twist()
        self.last_total_a = None
        self.last_total_b = None
        self.x = 0.0
        self.y = 0.0
        self.yaw = 0.0

        self.cmd_sub = self.create_subscription(Twist, "cmd_vel", self.on_cmd_vel, 10)
        self.odom_pub = self.create_publisher(Odometry, "odom", 10)
        self.battery_pub = self.create_publisher(BatteryState, "battery_state", 10)
        self.imu_pub = self.create_publisher(Imu, "imu/raw", 10)
        self.env_temp_pub = self.create_publisher(Temperature, "env/temperature", 10)
        self.env_humi_pub = self.create_publisher(RelativeHumidity, "env/humidity", 10)
        self.battery_temp_pub = self.create_publisher(Temperature, "battery/temperature", 10)
        self.fault_pub = self.create_publisher(UInt16, "base/faults", 10)

        self.control_timer = self.create_timer(0.05, self.send_control)
        self.status_timer = self.create_timer(0.02, self.process_status_queue)
        self.request_timer = self.create_timer(1.0, self.request_status)

        self.open_serial()
        self.reader_thread = threading.Thread(target=self.reader_loop, daemon=True)
        self.reader_thread.start()

    def destroy_node(self):
        self.stop_event.set()
        if self.reader_thread.is_alive():
            self.reader_thread.join(timeout=1.0)
        with self.serial_lock:
            if self.serial_port is not None:
                self.serial_port.close()
                self.serial_port = None
        super().destroy_node()

    def open_serial(self):
        if serial is None:
            self.get_logger().error("pyserial is missing: install python3-serial")
            return
        try:
            new_port = serial.Serial(self.port, self.baud, timeout=0.05)
            with self.serial_lock:
                self.serial_port = new_port
            self.get_logger().info(f"serial opened: {self.port} @ {self.baud}")
            self.write_frame(CMD_SET_STATE, bytes([STATE_CMD_ENABLE]))
        except serial.SerialException as exc:
            with self.serial_lock:
                self.serial_port = None
            self.get_logger().warning(f"serial open failed: {exc}")

    def reader_loop(self):
        while not self.stop_event.is_set():
            if self.serial_port is None:
                time.sleep(1.0)
                if self.serial_port is None:
                    self.open_serial()
                continue

            try:
                data = self.serial_port.read(64)
                if not data:
                    continue
                for command, payload in self.parser.feed(data):
                    if command == CMD_STATUS:
                        try:
                            self.status_queue.put_nowait(payload)
                        except queue.Full:
                            pass
            except serial.SerialException as exc:
                self.get_logger().warning(f"serial read failed: {exc}")
                with self.serial_lock:
                    if self.serial_port is not None:
                        self.serial_port.close()
                    self.serial_port = None

    def write_frame(self, command: int, payload: bytes = b""):
        frame = make_frame(command, payload)
        with self.serial_lock:
            if self.serial_port is None:
                return
            try:
                self.serial_port.write(frame)
            except serial.SerialException as exc:
                self.get_logger().warning(f"serial write failed: {exc}")
                self.serial_port.close()
                self.serial_port = None

    def on_cmd_vel(self, msg: Twist):
        self.last_twist = msg
        self.last_cmd_time = time.monotonic()

    def send_control(self):
        now = time.monotonic()
        if now - self.last_cmd_time > self.command_timeout_s:
            linear = 0.0
            angular = 0.0
        else:
            linear = float(self.last_twist.linear.x)
            angular = float(self.last_twist.angular.z)

        left_m_s = linear - angular * self.wheel_base_m * 0.5
        right_m_s = linear + angular * self.wheel_base_m * 0.5

        if self.use_pwm_mode:
            left = self.scale_pwm(left_m_s)
            right = self.scale_pwm(right_m_s)
            mode = MOTION_MODE_PWM
        else:
            left = self.clamp_int(round(left_m_s * 1000.0), -self.max_wheel_speed_mm_s, self.max_wheel_speed_mm_s)
            right = self.clamp_int(round(right_m_s * 1000.0), -self.max_wheel_speed_mm_s, self.max_wheel_speed_mm_s)
            mode = MOTION_MODE_SPEED

        payload = struct.pack("<hhB", int(left), int(right), mode)
        self.write_frame(CMD_SET_MOTION, payload)

    def request_status(self):
        self.write_frame(CMD_GET_STATUS)

    def process_status_queue(self):
        processed = 0
        while processed < 4:
            try:
                payload = self.status_queue.get_nowait()
            except queue.Empty:
                break
            status = self.parse_status(payload)
            if status is not None:
                self.publish_status(status)
            processed += 1

    def parse_status(self, payload: bytes):
        if len(payload) < 56:
            return None

        off = 0

        def u8():
            nonlocal off
            value = payload[off]
            off += 1
            return value

        def u16():
            nonlocal off
            value = struct.unpack_from("<H", payload, off)[0]
            off += 2
            return value

        def i16():
            nonlocal off
            value = struct.unpack_from("<h", payload, off)[0]
            off += 2
            return value

        def i32():
            nonlocal off
            value = struct.unpack_from("<i", payload, off)[0]
            off += 4
            return value

        status = {
            "uptime_ms": i32(),
            "state": u8(),
            "faults": u16(),
            "motion_mode": u8(),
            "target_a": i16(),
            "target_b": i16(),
            "pwm_a": i16(),
            "pwm_b": i16(),
            "speed_a_mm_s": i16(),
            "speed_b_mm_s": i16(),
            "total_a": i32(),
            "total_b": i32(),
            "battery_mv": u16(),
            "battery_temp_c_x100": i16(),
            "env_temp_c_x100": i16(),
            "env_humi_x100": u16(),
            "accel": [i16(), i16(), i16()],
            "gyro": [i16(), i16(), i16()],
            "sensor_flags": u16(),
            "rx_overflow": i32(),
            "fan_on": u8(),
            "buzzer_on": u8(),
        }
        return status

    def publish_status(self, status):
        stamp = self.get_clock().now().to_msg()

        fault_msg = UInt16()
        fault_msg.data = int(status["faults"])
        self.fault_pub.publish(fault_msg)

        if status["sensor_flags"] & SENSOR_INA219_VALID:
            msg = BatteryState()
            msg.header.stamp = stamp
            msg.voltage = status["battery_mv"] / 1000.0
            msg.present = True
            self.battery_pub.publish(msg)

        if status["sensor_flags"] & SENSOR_DHT30_VALID:
            temp = Temperature()
            temp.header.stamp = stamp
            temp.temperature = status["env_temp_c_x100"] / 100.0
            self.env_temp_pub.publish(temp)

            humi = RelativeHumidity()
            humi.header.stamp = stamp
            humi.relative_humidity = status["env_humi_x100"] / 10000.0
            self.env_humi_pub.publish(humi)

        if status["sensor_flags"] & SENSOR_DS18B20_VALID:
            temp = Temperature()
            temp.header.stamp = stamp
            temp.temperature = status["battery_temp_c_x100"] / 100.0
            self.battery_temp_pub.publish(temp)

        if status["sensor_flags"] & SENSOR_MPU6050_VALID:
            imu = Imu()
            imu.header.stamp = stamp
            imu.header.frame_id = self.base_frame_id
            imu.linear_acceleration.x = status["accel"][0] / 16384.0 * 9.80665
            imu.linear_acceleration.y = status["accel"][1] / 16384.0 * 9.80665
            imu.linear_acceleration.z = status["accel"][2] / 16384.0 * 9.80665
            imu.angular_velocity.x = status["gyro"][0] / 131.0 * math.pi / 180.0
            imu.angular_velocity.y = status["gyro"][1] / 131.0 * math.pi / 180.0
            imu.angular_velocity.z = status["gyro"][2] / 131.0 * math.pi / 180.0
            imu.orientation_covariance[0] = -1.0
            self.imu_pub.publish(imu)

        if self.publish_odom:
            self.publish_odometry(status, stamp)

    def publish_odometry(self, status, stamp):
        total_a = int(status["total_a"])
        total_b = int(status["total_b"])

        if self.last_total_a is None:
            self.last_total_a = total_a
            self.last_total_b = total_b
            return

        delta_a = total_a - self.last_total_a
        delta_b = total_b - self.last_total_b
        self.last_total_a = total_a
        self.last_total_b = total_b

        meters_per_count = math.pi * self.wheel_diameter_m / float(self.counts_per_rev)
        left_dist = delta_a * meters_per_count
        right_dist = delta_b * meters_per_count
        ds = (left_dist + right_dist) * 0.5
        dtheta = (right_dist - left_dist) / self.wheel_base_m

        heading_mid = self.yaw + dtheta * 0.5
        self.x += ds * math.cos(heading_mid)
        self.y += ds * math.sin(heading_mid)
        self.yaw = self.normalize_angle(self.yaw + dtheta)

        odom = Odometry()
        odom.header.stamp = stamp
        odom.header.frame_id = self.frame_id
        odom.child_frame_id = self.base_frame_id
        odom.pose.pose.position.x = self.x
        odom.pose.pose.position.y = self.y
        odom.pose.pose.orientation.z = math.sin(self.yaw * 0.5)
        odom.pose.pose.orientation.w = math.cos(self.yaw * 0.5)
        odom.twist.twist.linear.x = (status["speed_a_mm_s"] + status["speed_b_mm_s"]) * 0.0005
        odom.twist.twist.angular.z = (
            (status["speed_b_mm_s"] - status["speed_a_mm_s"]) / 1000.0 / self.wheel_base_m
        )
        self.odom_pub.publish(odom)

    def scale_pwm(self, wheel_m_s: float) -> int:
        ratio = wheel_m_s * 1000.0 / float(self.max_wheel_speed_mm_s)
        return self.clamp_int(round(ratio * self.max_pwm), -self.max_pwm, self.max_pwm)

    @staticmethod
    def clamp_int(value, low, high) -> int:
        return max(low, min(high, int(value)))

    @staticmethod
    def normalize_angle(value):
        while value > math.pi:
            value -= 2.0 * math.pi
        while value < -math.pi:
            value += 2.0 * math.pi
        return value


def main(args=None):
    rclpy.init(args=args)
    node = RobotBaseBridge()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.write_frame(CMD_SET_STATE, bytes([STATE_CMD_IDLE]))
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
