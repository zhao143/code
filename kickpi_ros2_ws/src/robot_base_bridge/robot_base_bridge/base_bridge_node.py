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
from std_msgs.msg import String, UInt16

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
    """计算 STM32 二进制协议使用的 CRC16-CCITT 校验值。

    参数：
        data：需要参与校验的字节数据。
        seed：CRC 初值，当前协议固定使用 0xFFFF。

    返回：
        16 位 CRC 结果。发送帧和接收帧校验都必须使用同一个算法。
    """
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
    """把命令号和 payload 打包成 STM32 能识别的二进制帧。

    帧格式与 STM32 端 robot_comm.c 保持一致：AA 55 + 版本 + 命令 + 长度 +
    payload + CRC16。这个函数只在正式 KICKPI 通信模式下使用，底板调试模式
    下不要发送这些二进制数据。
    """
    if len(payload) > MAX_PAYLOAD:
        raise ValueError("payload too long")
    header = bytes([VERSION, command, len(payload)])
    crc = crc16_ccitt(header + payload)
    return FRAME_HEAD + header + payload + struct.pack("<H", crc)


class FrameParser:
    """STM32 二进制状态帧解析器。

    串口读到的数据可能不是完整的一帧，也可能一次读到多帧，所以这里用一个
    bytearray 缓存未处理数据，并从缓存中持续寻找合法帧头和 CRC 正确的帧。
    """

    def __init__(self):
        """初始化解析器缓存。

        buffer 中保存的是“已经收到但还没有组成完整合法帧”的字节。
        """
        self.buffer = bytearray()

    def feed(self, data: bytes):
        """向解析器输入一段串口字节，并返回解析出的完整帧。

        参数：
            data：从串口读到的新数据，可以是半帧、一帧或多帧。

        返回：
            [(command, payload), ...]。只有帧头、长度和 CRC 都正确的帧才会返回。
        """
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
    """KICKPI 上运行的 ROS2 到 STM32 串口桥接节点。

    这个节点订阅 /cmd_vel，把线速度和角速度换算成左右轮目标，再通过 UART1
    发给 STM32；同时解析 STM32 回传状态，发布电池、IMU、温湿度、里程计和
    故障状态话题。
    """

    def __init__(self):
        """初始化 ROS2 参数、话题、定时器和串口读线程。

        注意：使用这个节点前，STM32 固件里的 ROBOT_UART1_DEBUG_ONLY 必须改成 0。
        如果 STM32 仍处于文本调试模式，这个节点会收不到二进制状态帧。
        """
        super().__init__("robot_base_bridge")

        self.declare_parameter("port", "/dev/ttyAS5")
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
        self.state_cmd_sub = self.create_subscription(String, "base/state_cmd", self.on_state_cmd, 10)
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
        """关闭节点前释放串口和后台线程。

        ROS2 退出时会调用这里。先通知读线程停止，再关闭串口，避免程序结束时
        串口设备仍被占用。
        """
        self.stop_event.set()
        if self.reader_thread.is_alive():
            self.reader_thread.join(timeout=1.0)
        with self.serial_lock:
            if self.serial_port is not None:
                self.serial_port.close()
                self.serial_port = None
        super().destroy_node()

    def open_serial(self):
        """打开与 STM32 相连的串口。

        成功打开后会立即发送 ENABLE 状态命令，让底板进入 READY。真正让电机
        转动仍然需要后续 /cmd_vel 命令。
        """
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
        """后台串口接收线程。

        持续从串口读取字节，交给 FrameParser 解析。如果解析到 STM32 状态帧，
        就放入线程安全队列，交给 ROS2 定时器线程发布话题。
        """
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
        """向 STM32 发送一帧二进制命令。

        参数：
            command：协议命令号，例如设置运动、请求状态、急停。
            payload：命令携带的数据。
        """
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
        """接收 ROS2 /cmd_vel 速度命令。

        这里只缓存最新命令和接收时间，不直接写串口。真正发送由 send_control
        定时器完成，这样发送频率更稳定。
        """
        self.last_twist = msg
        self.last_cmd_time = time.monotonic()

    def on_state_cmd(self, msg: String):
        """接收上层发来的底盘状态命令。

        Web 页面、K230 屏幕或调试脚本可以向 /base/state_cmd 发布字符串命令：
        enable 解除空闲并允许运动；stop/idle 进入空闲并停止；clear/clear_fault 清除
        可恢复故障；estop 立即急停。真正执行这些状态变化的仍然是 STM32。
        """
        command = msg.data.strip().lower()

        if command in ("enable", "run", "ready"):
            self.write_frame(CMD_SET_STATE, bytes([STATE_CMD_ENABLE]))
            self.get_logger().info("state command sent: enable")
        elif command in ("stop", "idle"):
            self.last_twist = Twist()
            self.last_cmd_time = 0.0
            self.write_frame(CMD_SET_MOTION, struct.pack("<hhB", 0, 0, MOTION_MODE_SPEED))
            self.write_frame(CMD_SET_STATE, bytes([STATE_CMD_IDLE]))
            self.get_logger().info("state command sent: idle")
        elif command in ("clear", "clear_fault", "reset_fault"):
            self.write_frame(CMD_SET_STATE, bytes([STATE_CMD_CLEAR_FAULT]))
            self.get_logger().info("state command sent: clear_fault")
        elif command in ("estop", "emergency_stop"):
            self.last_twist = Twist()
            self.last_cmd_time = 0.0
            self.write_frame(CMD_ESTOP)
            self.get_logger().warning("state command sent: estop")
        else:
            self.get_logger().warning(f"unknown state command ignored: {msg.data!r}")

    def send_control(self):
        """周期性把最新 /cmd_vel 转换为左右轮命令并发送给 STM32。

        如果超过 command_timeout_s 没收到新的 /cmd_vel，就主动发送 0 速度，
        避免上层导航节点停止发布后底盘继续运动。
        """
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
        """向 STM32 请求一次状态。

        STM32 正式模式下会回传 0x81 状态帧。调试模式下该命令无效，因为调试
        模式不走二进制协议。
        """
        self.write_frame(CMD_GET_STATUS)

    def process_status_queue(self):
        """处理串口线程解析出的状态帧队列。

        串口读线程不直接发布 ROS 话题，而是把 payload 放入队列。本函数在 ROS2
        定时器里运行，负责把队列里的状态解析并发布出去。
        """
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
        """解析 STM32 的 0x81 状态 payload。

        字段顺序必须和 STM32 端 robot_app.c 的正式模式 send_status 完全一致。
        如果 payload 长度不够，说明帧内容不完整，直接返回 None。
        """
        if len(payload) < 56:
            return None

        off = 0

        def u8():
            """从 payload 当前偏移读取 1 个无符号 8 位数。"""
            nonlocal off
            value = payload[off]
            off += 1
            return value

        def u16():
            """从 payload 当前偏移读取 1 个小端无符号 16 位数。"""
            nonlocal off
            value = struct.unpack_from("<H", payload, off)[0]
            off += 2
            return value

        def i16():
            """从 payload 当前偏移读取 1 个小端有符号 16 位数。"""
            nonlocal off
            value = struct.unpack_from("<h", payload, off)[0]
            off += 2
            return value

        def i32():
            """从 payload 当前偏移读取 1 个小端有符号 32 位数。"""
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
        """把 STM32 状态字典发布为 ROS2 标准话题。

        根据 sensor_flags 判断哪些传感器数据有效，只发布有效传感器的话题。
        这样某个模块没接好时，不会把无效数据当正常数据发出去。
        """
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
        """根据左右轮编码器累计值发布简单里程计。

        这里使用差速底盘模型。它依赖轮径、轮距、编码器每圈计数都配置正确；
        第一次收到状态时只记录基准值，不立即发布位移。
        """
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
        """把轮子线速度粗略换算为开环 PWM。

        这个模式主要用于早期测试，不是真正的速度闭环。正式导航建议使用速度
        模式，让 STM32 根据编码器做闭环控制。
        """
        ratio = wheel_m_s * 1000.0 / float(self.max_wheel_speed_mm_s)
        return self.clamp_int(round(ratio * self.max_pwm), -self.max_pwm, self.max_pwm)

    @staticmethod
    def clamp_int(value, low, high) -> int:
        """把整数限制在 low 到 high 范围内。"""
        return max(low, min(high, int(value)))

    @staticmethod
    def normalize_angle(value):
        """把角度归一化到 -pi 到 pi 范围内。"""
        while value > math.pi:
            value -= 2.0 * math.pi
        while value < -math.pi:
            value += 2.0 * math.pi
        return value


def main(args=None):
    """ROS2 节点入口函数。

    启动节点后进入 rclpy.spin。退出时先给 STM32 发 IDLE 命令，再关闭串口和
    ROS2 节点，尽量让底盘停在安全状态。
    """
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
