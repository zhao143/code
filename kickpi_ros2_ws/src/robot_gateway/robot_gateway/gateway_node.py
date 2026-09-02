"""KICKPI 上的 ROS2、HTTP、Web 页面和 SQLite 网关。"""

import json
import math
import os
import sqlite3
import threading
import time
from http.server import BaseHTTPRequestHandler, ThreadingHTTPServer
from urllib.parse import parse_qs, urlparse

try:
    import cv2
except ImportError:  # pragma: no cover - KICKPI provides OpenCV at runtime.
    cv2 = None

import rclpy
from ament_index_python.packages import get_package_share_directory
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
from rclpy.node import Node
from sensor_msgs.msg import BatteryState, Imu, RelativeHumidity, Temperature
from std_msgs.msg import String, UInt16


class RobotGateway(Node):
    """把 KICKPI 上的 ROS2 数据转换为 Web/API，并保存为 SQLite 记录。"""

    def __init__(self):
        """初始化 ROS 话题、HTTP 服务、数据库和控制安全定时器。"""
        super().__init__("robot_gateway")

        self.declare_parameter("host", "0.0.0.0")
        self.declare_parameter("port", 8080)
        self.declare_parameter("db_path", "/root/robot_data/robot.db")
        self.declare_parameter("token", "robot-dev-token")
        self.declare_parameter("control_timeout_s", 0.3)
        self.declare_parameter("telemetry_period_s", 1.0)
        self.declare_parameter("max_linear_x_m_s", 0.20)
        self.declare_parameter("max_angular_z_rad_s", 1.20)
        self.declare_parameter("video_enabled", False)
        self.declare_parameter("video_rtsp_url", "rtsp://192.168.2.3:8554/test")
        self.declare_parameter("video_fps", 8)
        self.declare_parameter("video_width", 640)
        self.declare_parameter("video_jpeg_quality", 75)

        self.host = str(self.get_parameter("host").value)
        self.port = int(self.get_parameter("port").value)
        self.db_path = str(self.get_parameter("db_path").value)
        self.token = str(self.get_parameter("token").value)
        self.control_timeout_s = float(self.get_parameter("control_timeout_s").value)
        self.telemetry_period_s = float(self.get_parameter("telemetry_period_s").value)
        self.max_linear_x_m_s = float(self.get_parameter("max_linear_x_m_s").value)
        self.max_angular_z_rad_s = float(self.get_parameter("max_angular_z_rad_s").value)
        video_enabled_value = self.get_parameter("video_enabled").value
        if isinstance(video_enabled_value, bool):
            self.video_enabled = video_enabled_value
        else:
            self.video_enabled = str(video_enabled_value).strip().lower() in (
                "1",
                "true",
                "yes",
                "on",
            )
        self.video_rtsp_url = str(self.get_parameter("video_rtsp_url").value)
        self.video_fps = max(1, int(self.get_parameter("video_fps").value))
        self.video_width = max(160, int(self.get_parameter("video_width").value))
        self.video_jpeg_quality = min(
            95,
            max(30, int(self.get_parameter("video_jpeg_quality").value)),
        )
        # K230 的 writeback 流在当前固件上相对浏览器坐标逆时针转了 90 度。
        # 在 KICKPI 代理层旋转，既不影响 K230 本地触摸坐标，也不增加 K230
        # 的图像处理负担。若更换 K230 显示固件，可把该参数改为 0。
        # 当前 K230 writeback 输出是 480x800，内容相对正常横屏逆时针转了 90 度。
        # 这里使用 270 表示 OpenCV 顺时针旋转 270 度，也就是逆时针 90 度。
        self.video_rotate_degrees = 270

        self.data_lock = threading.Lock()
        self.db_lock = threading.Lock()
        self.video_lock = threading.Lock()
        self.stop_event = threading.Event()
        self.video_jpeg = None
        self.video_frame_version = 0
        self.video_thread = None
        self.last_control_time = 0.0
        self.last_control_nonzero = False
        self.latest = {
            "timestamp": None,
            "battery_voltage_v": None,
            "battery_temperature_c": None,
            "environment_temperature_c": None,
            "environment_humidity_percent": None,
            "faults": 0,
            "odom": {"x_m": 0.0, "y_m": 0.0, "yaw_rad": 0.0},
            "imu": {"accel_m_s2": [None, None, None], "gyro_rad_s": [None, None, None]},
            "last_command": {"source": None, "button": None, "linear_x_m_s": 0.0, "angular_z_rad_s": 0.0},
            "outputs": {"relay_on": False, "buzzer_on": False},
            "last_vision": {"frame_id": None, "detections": []},
        }

        self.init_database()

        self.cmd_pub = self.create_publisher(Twist, "cmd_vel", 10)
        self.state_pub = self.create_publisher(String, "base/state_cmd", 10)
        self.output_pub = self.create_publisher(String, "base/output_cmd", 10)
        self.vision_pub = self.create_publisher(String, "vision/detections", 10)

        self.create_subscription(BatteryState, "battery_state", self.on_battery, 10)
        self.create_subscription(Temperature, "env/temperature", self.on_environment_temperature, 10)
        self.create_subscription(RelativeHumidity, "env/humidity", self.on_environment_humidity, 10)
        self.create_subscription(Temperature, "battery/temperature", self.on_battery_temperature, 10)
        self.create_subscription(UInt16, "base/faults", self.on_faults, 10)
        self.create_subscription(String, "base/output_state", self.on_output_state, 10)
        self.create_subscription(Imu, "imu/raw", self.on_imu, 10)
        self.create_subscription(Odometry, "odom", self.on_odom, 10)

        self.telemetry_timer = self.create_timer(self.telemetry_period_s, self.record_telemetry)
        self.watchdog_timer = self.create_timer(0.05, self.control_watchdog)

        self.web_root = os.path.join(get_package_share_directory("robot_gateway"), "web")
        self.httpd = ThreadingHTTPServer((self.host, self.port), self.make_handler())
        self.http_thread = threading.Thread(target=self.httpd.serve_forever, daemon=True)
        self.http_thread.start()
        self.get_logger().info(f"gateway listening on http://{self.host}:{self.port}")
        self.get_logger().info(f"sqlite database: {self.db_path}")

        if self.video_enabled and cv2 is not None:
            self.video_thread = threading.Thread(
                target=self.video_capture_loop,
                name="robot-video",
                daemon=True,
            )
            self.video_thread.start()
            self.get_logger().info(
                "video MJPEG enabled: "
                f"{self.video_rtsp_url} -> /video.mjpg at {self.video_fps} FPS"
            )
        elif self.video_enabled:
            self.video_enabled = False
            self.get_logger().warning(
                "video requested but OpenCV is unavailable; /video.mjpg is disabled"
            )

    def init_database(self):
        """创建遥测、视觉和控制事件表，并开启 SQLite WAL 模式。"""
        directory = os.path.dirname(self.db_path)
        if directory:
            os.makedirs(directory, exist_ok=True)

        self.db = sqlite3.connect(self.db_path, check_same_thread=False)
        with self.db_lock:
            self.db.execute("PRAGMA journal_mode=WAL")
            self.db.execute(
                """
                CREATE TABLE IF NOT EXISTS telemetry (
                    id INTEGER PRIMARY KEY AUTOINCREMENT,
                    ts REAL NOT NULL,
                    battery_voltage_v REAL,
                    battery_temperature_c REAL,
                    environment_temperature_c REAL,
                    environment_humidity_percent REAL,
                    faults INTEGER NOT NULL,
                    odom_x_m REAL,
                    odom_y_m REAL,
                    odom_yaw_rad REAL
                )
                """
            )
            self.db.execute(
                """
                CREATE TABLE IF NOT EXISTS vision_events (
                    id INTEGER PRIMARY KEY AUTOINCREMENT,
                    ts REAL NOT NULL,
                    source TEXT NOT NULL,
                    frame_id INTEGER,
                    detection_count INTEGER NOT NULL,
                    raw_json TEXT NOT NULL
                )
                """
            )
            self.db.execute(
                """
                CREATE TABLE IF NOT EXISTS vision_detections (
                    id INTEGER PRIMARY KEY AUTOINCREMENT,
                    event_id INTEGER NOT NULL,
                    label TEXT,
                    score REAL,
                    x REAL,
                    y REAL,
                    w REAL,
                    h REAL,
                    FOREIGN KEY(event_id) REFERENCES vision_events(id)
                )
                """
            )
            self.db.execute(
                """
                CREATE TABLE IF NOT EXISTS control_events (
                    id INTEGER PRIMARY KEY AUTOINCREMENT,
                    ts REAL NOT NULL,
                    source TEXT NOT NULL,
                    command TEXT,
                    linear_x_m_s REAL,
                    angular_z_rad_s REAL,
                    accepted INTEGER NOT NULL,
                    reason TEXT
                )
                """
            )
            self.db.commit()

    def destroy_node(self):
        """停止视频和 HTTP 线程、关闭数据库，并释放 ROS2 节点资源。"""
        self.stop_event.set()
        if hasattr(self, "httpd"):
            self.httpd.shutdown()
            self.httpd.server_close()
        if hasattr(self, "http_thread") and self.http_thread.is_alive():
            self.http_thread.join(timeout=1.0)
        if hasattr(self, "video_thread") and self.video_thread is not None:
            if self.video_thread.is_alive():
                self.video_thread.join(timeout=3.0)
        if hasattr(self, "db"):
            with self.db_lock:
                self.db.commit()
                self.db.close()
        super().destroy_node()

    def video_capture_loop(self):
        """从 K230 RTSP 读取画面，编码成低帧率 JPEG 供浏览器观看。"""
        if cv2 is None:
            return

        frame_interval = 1.0 / float(self.video_fps)
        last_error_log = 0.0

        while not self.stop_event.is_set():
            capture = cv2.VideoCapture()
            try:
                if hasattr(cv2, "CAP_PROP_OPEN_TIMEOUT_MSEC"):
                    capture.set(cv2.CAP_PROP_OPEN_TIMEOUT_MSEC, 3000)
                if hasattr(cv2, "CAP_PROP_READ_TIMEOUT_MSEC"):
                    capture.set(cv2.CAP_PROP_READ_TIMEOUT_MSEC, 3000)
                opened = capture.open(self.video_rtsp_url, cv2.CAP_FFMPEG)
            except Exception as exc:
                opened = False
                now = time.monotonic()
                if now - last_error_log > 5.0:
                    self.get_logger().warning(f"video open exception: {exc}")
                    last_error_log = now

            if not opened:
                capture.release()
                now = time.monotonic()
                if now - last_error_log > 5.0:
                    self.get_logger().warning(
                        f"cannot open K230 RTSP stream: {self.video_rtsp_url}"
                    )
                    last_error_log = now
                self.stop_event.wait(2.0)
                continue

            next_frame_time = 0.0
            while not self.stop_event.is_set():
                ok, frame = capture.read()
                if not ok or frame is None:
                    break

                now = time.monotonic()
                if now < next_frame_time:
                    continue
                next_frame_time = now + frame_interval

                if self.video_rotate_degrees == 90:
                    frame = cv2.rotate(frame, cv2.ROTATE_90_CLOCKWISE)
                elif self.video_rotate_degrees == 180:
                    frame = cv2.rotate(frame, cv2.ROTATE_180)
                elif self.video_rotate_degrees == 270:
                    frame = cv2.rotate(frame, cv2.ROTATE_90_COUNTERCLOCKWISE)

                height, width = frame.shape[:2]
                if width > self.video_width:
                    scaled_height = max(1, int(height * self.video_width / width))
                    frame = cv2.resize(
                        frame,
                        (self.video_width, scaled_height),
                        interpolation=cv2.INTER_AREA,
                    )

                encoded_ok, encoded = cv2.imencode(
                    ".jpg",
                    frame,
                    [int(cv2.IMWRITE_JPEG_QUALITY), self.video_jpeg_quality],
                )
                if not encoded_ok:
                    continue

                with self.video_lock:
                    self.video_jpeg = encoded.tobytes()
                    self.video_frame_version += 1

            capture.release()
            now = time.monotonic()
            if not self.stop_event.is_set() and now - last_error_log > 5.0:
                self.get_logger().warning("K230 RTSP read stopped; reconnecting")
                last_error_log = now
            self.stop_event.wait(1.0)

    def on_battery(self, msg: BatteryState):
        """保存底板发布的电池电压。INA219 当前按电压检测使用。"""
        if math.isfinite(msg.voltage):
            with self.data_lock:
                self.latest["battery_voltage_v"] = float(msg.voltage)

    def on_environment_temperature(self, msg: Temperature):
        """保存 DHT30 发布的环境温度。"""
        if math.isfinite(msg.temperature):
            with self.data_lock:
                self.latest["environment_temperature_c"] = float(msg.temperature)

    def on_environment_humidity(self, msg: RelativeHumidity):
        """保存 DHT30 发布的相对湿度，并转换成百分数。"""
        if math.isfinite(msg.relative_humidity):
            with self.data_lock:
                self.latest["environment_humidity_percent"] = float(msg.relative_humidity) * 100.0

    def on_battery_temperature(self, msg: Temperature):
        """保存 DS18B20 发布的电池温度。"""
        if math.isfinite(msg.temperature):
            with self.data_lock:
                self.latest["battery_temperature_c"] = float(msg.temperature)

    def on_faults(self, msg: UInt16):
        """保存 STM32 当前故障位。"""
        with self.data_lock:
            self.latest["faults"] = int(msg.data)

    def on_output_state(self, msg: String):
        """保存 STM32 实际返回的蜂鸣器和继电器状态。"""
        try:
            value = json.loads(msg.data)
            relay = bool(value.get("relay_on", False))
            buzzer = bool(value.get("buzzer_on", False))
        except (TypeError, ValueError, json.JSONDecodeError):
            return
        with self.data_lock:
            self.latest["outputs"] = {"relay_on": relay, "buzzer_on": buzzer}

    def on_imu(self, msg: Imu):
        """保存 MPU6050 的加速度和角速度。"""
        with self.data_lock:
            self.latest["imu"] = {
                "accel_m_s2": [
                    float(msg.linear_acceleration.x),
                    float(msg.linear_acceleration.y),
                    float(msg.linear_acceleration.z),
                ],
                "gyro_rad_s": [
                    float(msg.angular_velocity.x),
                    float(msg.angular_velocity.y),
                    float(msg.angular_velocity.z),
                ],
            }

    def on_odom(self, msg: Odometry):
        """保存底盘里程计的平面位置和航向角。"""
        q = msg.pose.pose.orientation
        yaw = math.atan2(2.0 * (q.w * q.z + q.x * q.y), 1.0 - 2.0 * (q.y * q.y + q.z * q.z))
        with self.data_lock:
            self.latest["odom"] = {
                "x_m": float(msg.pose.pose.position.x),
                "y_m": float(msg.pose.pose.position.y),
                "yaw_rad": float(yaw),
            }

    def latest_copy(self):
        """在线程安全地复制当前状态，避免 HTTP 线程读到半更新字典。"""
        with self.data_lock:
            return json.loads(json.dumps(self.latest))

    def record_telemetry(self):
        """按固定周期把当前状态快照写入 telemetry 表。"""
        snapshot = self.latest_copy()
        now = time.time()
        snapshot["timestamp"] = now
        with self.data_lock:
            self.latest["timestamp"] = now

        odom = snapshot["odom"]
        with self.db_lock:
            self.db.execute(
                """
                INSERT INTO telemetry (
                    ts, battery_voltage_v, battery_temperature_c,
                    environment_temperature_c, environment_humidity_percent,
                    faults, odom_x_m, odom_y_m, odom_yaw_rad
                ) VALUES (?, ?, ?, ?, ?, ?, ?, ?, ?)
                """,
                (
                    now,
                    snapshot["battery_voltage_v"],
                    snapshot["battery_temperature_c"],
                    snapshot["environment_temperature_c"],
                    snapshot["environment_humidity_percent"],
                    int(snapshot["faults"]),
                    odom["x_m"],
                    odom["y_m"],
                    odom["yaw_rad"],
                ),
            )
            self.db.commit()

    def control_watchdog(self):
        """如果 K230/Web 心跳超时，自动向底盘发布零速度。"""
        if not self.last_control_nonzero:
            return
        if time.monotonic() - self.last_control_time <= self.control_timeout_s:
            return

        self.publish_twist(0.0, 0.0)
        self.last_control_nonzero = False
        self.record_control_event("watchdog", "timeout_stop", 0.0, 0.0, True, "control heartbeat timeout")
        self.get_logger().warning("control heartbeat timeout: published zero velocity")

    def publish_twist(self, linear_x, angular_z):
        """发布 ROS2 Twist 到 robot_base_bridge。"""
        msg = Twist()
        msg.linear.x = float(linear_x)
        msg.angular.z = float(angular_z)
        self.cmd_pub.publish(msg)

    def clamp_float(self, value, low, high):
        """把有限浮点数限制在给定范围内。"""
        try:
            value = float(value)
        except (TypeError, ValueError):
            return 0.0
        if not math.isfinite(value):
            return 0.0
        return max(low, min(high, value))

    def accept_cmd_vel(self, data):
        """校验、限幅并发布 HTTP 速度命令。"""
        linear = self.clamp_float(
            data.get("linear_x_m_s", 0.0),
            -self.max_linear_x_m_s,
            self.max_linear_x_m_s,
        )
        angular = self.clamp_float(
            data.get("angular_z_rad_s", 0.0),
            -self.max_angular_z_rad_s,
            self.max_angular_z_rad_s,
        )
        source = str(data.get("source", "http"))
        button = str(data.get("button", ""))

        with self.data_lock:
            fault_code = int(self.latest["faults"])

        if fault_code and (abs(linear) > 1e-6 or abs(angular) > 1e-6):
            self.publish_twist(0.0, 0.0)
            self.record_control_event(source, button, linear, angular, False, f"fault 0x{fault_code:04X}")
            return False, "fault_active"

        self.publish_twist(linear, angular)
        self.last_control_time = time.monotonic()
        self.last_control_nonzero = abs(linear) > 1e-6 or abs(angular) > 1e-6
        with self.data_lock:
            self.latest["last_command"] = {
                "source": source,
                "button": button,
                "linear_x_m_s": linear,
                "angular_z_rad_s": angular,
            }
        self.record_control_event(source, button, linear, angular, True, "")
        return True, "accepted"

    def accept_state_command(self, data):
        """向 robot_base_bridge 转发底盘状态命令。"""
        command = str(data.get("command", "")).strip().lower()
        allowed = {
            "enable": "enable",
            "stop": "stop",
            "idle": "stop",
            "clear_fault": "clear_fault",
            "clear": "clear_fault",
            "estop": "estop",
        }
        normalized = allowed.get(command)
        if normalized is None:
            return False, "unsupported_state_command"

        if normalized in ("stop", "estop"):
            self.publish_twist(0.0, 0.0)
            self.last_control_nonzero = False

        msg = String()
        msg.data = normalized
        self.state_pub.publish(msg)
        source = str(data.get("source", "http"))
        self.record_control_event(source, normalized, 0.0, 0.0, True, "")
        return True, "accepted"

    def accept_output_command(self, data):
        """校验网页输出开关，并向底盘桥发布蜂鸣器/继电器命令。

        每次请求同时发送两个输出的完整状态，而不是只发送一个变化量，
        这样网页刷新、重复点击或网络重连都不会让另一项输出进入未知状态。
        """
        def as_bool(value):
            if isinstance(value, bool):
                return value
            if isinstance(value, (int, float)):
                return value != 0
            return str(value).strip().lower() in ("1", "true", "yes", "on")

        with self.data_lock:
            current = dict(self.latest["outputs"])

        buzzer = as_bool(data.get("buzzer_on", current["buzzer_on"]))
        relay = as_bool(data.get("relay_on", current["relay_on"]))

        msg = String()
        msg.data = f"buzzer={1 if buzzer else 0} relay={1 if relay else 0}"
        self.output_pub.publish(msg)
        with self.data_lock:
            self.latest["outputs"] = {"relay_on": relay, "buzzer_on": buzzer}

        source = str(data.get("source", "web"))
        self.record_control_event(
            source,
            f"output buzzer={int(buzzer)} relay={int(relay)}",
            0.0,
            0.0,
            True,
            "",
        )
        return True, "accepted"

    def record_control_event(self, source, command, linear, angular, accepted, reason):
        """保存一次来自 Web、K230 或 watchdog 的控制事件。"""
        with self.db_lock:
            self.db.execute(
                """
                INSERT INTO control_events (
                    ts, source, command, linear_x_m_s, angular_z_rad_s, accepted, reason
                ) VALUES (?, ?, ?, ?, ?, ?, ?)
                """,
                (
                    time.time(),
                    str(source),
                    str(command),
                    float(linear),
                    float(angular),
                    1 if accepted else 0,
                    str(reason),
                ),
            )
            self.db.commit()

    def save_vision_event(self, data):
        """保存 K230 识别事件，并向 ROS2 发布完整 JSON。"""
        source = str(data.get("source", "k230"))
        frame_id = data.get("frame_id")
        try:
            frame_id = int(frame_id) if frame_id is not None else None
        except (TypeError, ValueError):
            frame_id = None

        detections = data.get("detections", [])
        if not isinstance(detections, list):
            detections = []

        event_time = time.time()
        raw_json = json.dumps(data, ensure_ascii=False)
        with self.db_lock:
            cursor = self.db.execute(
                """
                INSERT INTO vision_events (ts, source, frame_id, detection_count, raw_json)
                VALUES (?, ?, ?, ?, ?)
                """,
                (event_time, source, frame_id, len(detections), raw_json),
            )
            event_id = cursor.lastrowid
            for item in detections:
                if not isinstance(item, dict):
                    continue
                self.db.execute(
                    """
                    INSERT INTO vision_detections (event_id, label, score, x, y, w, h)
                    VALUES (?, ?, ?, ?, ?, ?, ?)
                    """,
                    (
                        event_id,
                        str(item.get("label", "")),
                        self.number_or_none(item.get("score")),
                        self.number_or_none(item.get("x")),
                        self.number_or_none(item.get("y")),
                        self.number_or_none(item.get("w")),
                        self.number_or_none(item.get("h")),
                    ),
                )
            self.db.commit()

        with self.data_lock:
            self.latest["last_vision"] = {"frame_id": frame_id, "detections": detections}
        msg = String()
        msg.data = raw_json
        self.vision_pub.publish(msg)
        return event_id

    @staticmethod
    def number_or_none(value):
        """把 JSON 数值转换成有限浮点数；非法值保存为 NULL。"""
        try:
            value = float(value)
        except (TypeError, ValueError):
            return None
        return value if math.isfinite(value) else None

    def query_telemetry(self, limit):
        """读取最近的遥测记录。"""
        limit = max(1, min(int(limit), 2000))
        with self.db_lock:
            rows = self.db.execute(
                """
                SELECT ts, battery_voltage_v, battery_temperature_c,
                       environment_temperature_c, environment_humidity_percent,
                       faults, odom_x_m, odom_y_m, odom_yaw_rad
                FROM telemetry ORDER BY id DESC LIMIT ?
                """,
                (limit,),
            ).fetchall()
        fields = (
            "timestamp",
            "battery_voltage_v",
            "battery_temperature_c",
            "environment_temperature_c",
            "environment_humidity_percent",
            "faults",
            "odom_x_m",
            "odom_y_m",
            "odom_yaw_rad",
        )
        return [dict(zip(fields, row)) for row in rows]

    def query_vision_events(self, limit):
        """读取最近的视觉识别事件。"""
        limit = max(1, min(int(limit), 500))
        with self.db_lock:
            rows = self.db.execute(
                """
                SELECT id, ts, source, frame_id, detection_count, raw_json
                FROM vision_events ORDER BY id DESC LIMIT ?
                """,
                (limit,),
            ).fetchall()
        events = []
        for event_id, ts, source, frame_id, count, raw_json in rows:
            try:
                data = json.loads(raw_json)
            except json.JSONDecodeError:
                data = {}
            events.append(
                {
                    "id": event_id,
                    "timestamp": ts,
                    "source": source,
                    "frame_id": frame_id,
                    "detection_count": count,
                    "detections": data.get("detections", []),
                }
            )
        return events

    def make_handler(self):
        """创建绑定当前 ROS2 节点的 HTTP 请求处理器。"""
        node = self

        class Handler(BaseHTTPRequestHandler):
            """处理 Web 页面和机器人 API 请求。"""

            server_version = "RobotGateway/0.1"

            def log_message(self, format_string, *args):
                """把 HTTP 访问日志转发到 ROS2 日志。"""
                node.get_logger().info("http " + (format_string % args))

            def send_json(self, status, payload):
                """向客户端返回 JSON，并允许局域网 Web 页面跨源访问。"""
                body = json.dumps(payload, ensure_ascii=False).encode("utf-8")
                self.send_response(status)
                self.send_header("Content-Type", "application/json; charset=utf-8")
                self.send_header("Content-Length", str(len(body)))
                self.send_header("Access-Control-Allow-Origin", "*")
                self.send_header("Access-Control-Allow-Headers", "Content-Type, X-Device-Token")
                self.send_header("Access-Control-Allow-Methods", "GET, POST, OPTIONS")
                self.end_headers()
                self.wfile.write(body)

            def unauthorized(self):
                """返回统一的 API 认证失败响应。"""
                self.send_json(401, {"ok": False, "error": "invalid_token"})

            def authorized(self):
                """校验 K230/Web 请求头中的简单设备令牌。"""
                return not node.token or self.headers.get("X-Device-Token", "") == node.token

            def read_json(self):
                """读取 POST body 并解析为 JSON 字典。"""
                try:
                    length = int(self.headers.get("Content-Length", "0"))
                    if length <= 0 or length > 65536:
                        return None
                    body = self.rfile.read(length)
                    value = json.loads(body.decode("utf-8"))
                    return value if isinstance(value, dict) else None
                except (ValueError, UnicodeDecodeError, json.JSONDecodeError):
                    return None

            def do_OPTIONS(self):
                """响应浏览器的 CORS 预检请求。"""
                self.send_response(204)
                self.send_header("Access-Control-Allow-Origin", "*")
                self.send_header("Access-Control-Allow-Headers", "Content-Type, X-Device-Token")
                self.send_header("Access-Control-Allow-Methods", "GET, POST, OPTIONS")
                self.end_headers()

            def do_GET(self):
                """处理网页、视频、健康检查、状态和历史数据查询。"""
                parsed = urlparse(self.path)
                if parsed.path in ("/", "/index.html"):
                    return self.serve_index()
                if parsed.path == "/video.mjpg":
                    return self.serve_mjpeg(parsed)
                if parsed.path == "/api/v1/health":
                    return self.send_json(200, {"ok": True, "service": "robot_gateway", "time": time.time()})
                if parsed.path == "/api/v1/latest":
                    return self.send_json(200, {"ok": True, "data": node.latest_copy()})
                if parsed.path == "/api/v1/telemetry":
                    query = parse_qs(parsed.query)
                    limit = query.get("limit", ["300"])[0]
                    try:
                        limit = int(limit)
                    except ValueError:
                        limit = 300
                    return self.send_json(200, {"ok": True, "data": node.query_telemetry(limit)})
                if parsed.path == "/api/v1/vision/events":
                    query = parse_qs(parsed.query)
                    limit = query.get("limit", ["100"])[0]
                    try:
                        limit = int(limit)
                    except ValueError:
                        limit = 100
                    return self.send_json(200, {"ok": True, "data": node.query_vision_events(limit)})
                self.send_json(404, {"ok": False, "error": "not_found"})

            def do_POST(self):
                """处理速度、状态、输出开关和视觉事件写入接口。"""
                if not self.authorized():
                    return self.unauthorized()
                data = self.read_json()
                if data is None:
                    return self.send_json(400, {"ok": False, "error": "invalid_json"})
                parsed = urlparse(self.path)

                if parsed.path == "/api/v1/control/cmd_vel":
                    accepted, reason = node.accept_cmd_vel(data)
                    status = 200 if accepted else 409
                    return self.send_json(status, {"ok": accepted, "result": reason})

                if parsed.path == "/api/v1/control/state":
                    accepted, reason = node.accept_state_command(data)
                    status = 200 if accepted else 400
                    return self.send_json(status, {"ok": accepted, "result": reason})

                if parsed.path == "/api/v1/control/output":
                    accepted, reason = node.accept_output_command(data)
                    status = 200 if accepted else 400
                    return self.send_json(status, {"ok": accepted, "result": reason})

                if parsed.path == "/api/v1/vision/events":
                    event_id = node.save_vision_event(data)
                    return self.send_json(201, {"ok": True, "event_id": event_id})

                self.send_json(404, {"ok": False, "error": "not_found"})

            def serve_mjpeg(self, parsed):
                """以 multipart MJPEG 方式把最新视频帧持续发送给浏览器。"""
                query_token = parse_qs(parsed.query).get("token", [None])[0]
                header_token = self.headers.get("X-Device-Token")
                if query_token != node.token and header_token != node.token:
                    return self.unauthorized()
                if not node.video_enabled:
                    return self.send_json(503, {"ok": False, "error": "video_disabled"})

                self.send_response(200)
                self.send_header("Cache-Control", "no-cache, no-store, must-revalidate")
                self.send_header("Pragma", "no-cache")
                self.send_header("Connection", "close")
                self.send_header("Content-Type", "multipart/x-mixed-replace; boundary=frame")
                self.end_headers()

                last_version = -1
                try:
                    while not node.stop_event.is_set():
                        with node.video_lock:
                            version = node.video_frame_version
                            jpeg = node.video_jpeg

                        if jpeg is None or version == last_version:
                            time.sleep(0.05)
                            continue

                        packet = (
                            b"--frame\r\n"
                            b"Content-Type: image/jpeg\r\n"
                            + f"Content-Length: {len(jpeg)}\r\n\r\n".encode("ascii")
                            + jpeg
                            + b"\r\n"
                        )
                        self.wfile.write(packet)
                        self.wfile.flush()
                        last_version = version
                except (BrokenPipeError, ConnectionResetError, OSError):
                    pass

            def serve_index(self):
                """返回内置 Web 控制和监视页面。"""
                path = os.path.join(node.web_root, "index.html")
                try:
                    with open(path, "rb") as stream:
                        body = stream.read()
                except OSError:
                    return self.send_json(500, {"ok": False, "error": "web_file_missing"})
                self.send_response(200)
                self.send_header("Content-Type", "text/html; charset=utf-8")
                self.send_header("Content-Length", str(len(body)))
                self.end_headers()
                self.wfile.write(body)

        return Handler


def main(args=None):
    """ROS2 网关进程入口。"""
    rclpy.init(args=args)
    node = RobotGateway()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
