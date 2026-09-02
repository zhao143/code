"""CanMV-K230 触摸屏驾驶程序。

运行环境：
    将 k230_canmv_apps 目录中的“内容”复制到 K230 SD 卡根目录 /sdcard/，再运行
    /sdcard/robot_touch_drive.py。这样脚本、libs 和 mp_deployment_source 位于同一级。

功能：
    1. LCD 实时显示摄像头画面。
    2. 触摸屏提供前进、后退、左转、右转、停车、使能、清故障和急停。
    3. 通过以太网向 KICKPI 的 HTTP API 发送速度命令和状态命令。
    4. 方向按钮按住时每 100 ms 发送心跳，松手或移出按钮立即发送停车命令。

安全约定：
    K230 不直接连接 STM32，也不直接控制电机。
    K230 -> 以太网 HTTP -> KICKPI -> UART5 -> STM32 -> TB6612。
"""

import gc
import os
import socket
import sys
import time
import ujson
import network

from machine import TOUCH
from media.media import *
from libs.PlatTasks import DetectionApp
from libs.PipeLine import PipeLine
from libs.Utils import read_json


# --------------------------- 用户需要确认的参数 ---------------------------

# 当前已经进入 KICKPI、K230 和底板联调阶段，打开 K230 -> KICKPI 的 HTTP 链路。
# 如果只想单独测试屏幕和识别，再临时改回 False。
KICKPI_LINK_ENABLE = True

# KICKPI 直连以太网地址。后续联调时，K230 会通过这个地址访问 Web/API。
KICKPI_IP = "192.168.2.2"
KICKPI_HTTP_PORT = 8080
API_TOKEN = "robot-dev-token"

# K230 和 KICKPI 直连时建议使用静态地址，避免没有 DHCP 时等待超时。
USE_DHCP = False
STATIC_IP = "192.168.2.3"
STATIC_MASK = "255.255.255.0"
STATIC_GATEWAY = "192.168.2.2"
STATIC_DNS = "192.168.2.2"

DISPLAY_WIDTH = 800
DISPLAY_HEIGHT = 480
# OV5647 在当前安装方向下画面上下左右相反。这里仅翻转摄像头输入，OSD
# 控件仍按 800x480 正常坐标绘制，触摸坐标也不需要跟着旋转。
CAMERA_HMIRROR = True
CAMERA_VFLIP = True
# 官方检测视频例程使用 640x360 作为 AI 图像通道；你的 Cron 模型输入是 320x320，
# DetectionApp 会按 deploy_config.json 的参数完成补边和缩放。
AI_FRAME_SIZE = [640, 360]
CONTROL_HEARTBEAT_MS = 100
STATUS_REFRESH_MS = 500
HTTP_TIMEOUT_S = 0.35
TOUCH_RELEASE_HOLD_MS = 180

# K230 官方资料提供了 WBCRtsp 推流类，可以把 LCD 的 writeback 画面编码为
# H.264/RTSP。它不会影响本地 LCD 显示，但会增加编码和网络负载。
#
# 浏览器不能直接播放 rtsp:// 地址，所以这个开关打开后还需要在 KICKPI
# 上运行 RTSP -> HLS/WebRTC/MJPEG 的转换服务，Web 页面才能显示。当前先
# 默认关闭，避免第一次联调时视频编码负载干扰触摸、识别和底盘控制。
VIDEO_RTSP_ENABLE = False
VIDEO_RTSP_PORT = 8554

WBCRtsp = None
if VIDEO_RTSP_ENABLE:
    try:
        from libs.WBCRtsp import WBCRtsp
    except BaseException as exc:
        print("RTSP disabled, WBCRtsp import failed:", exc)
        WBCRtsp = None

# K230 上电后 main.py 会很快自动执行，但此时网口 PHY、网线链路、
# KICKPI 网关服务可能还没有完全准备好。下面这些参数用于开机延时和离线重试，
# 避免“手动运行有网、上电自动运行没网”的启动时序问题。
BOOT_NETWORK_DELAY_MS = 3000
LAN_READY_TIMEOUT_MS = 15000
LAN_RETRY_TIMEOUT_MS = 1500
NETWORK_RETRY_MS = 3000

# Cron/Canaan 部署包路径。脚本会按顺序自动查找这些位置，避免你手工挪文件时再改代码。
MODEL_ROOT_CANDIDATES = (
    "/data/Cron/mp_deployment_source",
    "/data/Cron",
    "/sdcard/mp_deployment_source",
    "/sdcard/Cron",
)

# 若 SD 卡没有模型，程序会自动退化为“仅摄像头 + 触摸控制”，不影响底盘控制功能。
ENABLE_VISION = True
VISION_SEND_INTERVAL_MS = 500


# --------------------------- 屏幕布局 ---------------------------

# 摄像头画面占据上半部分；控制区固定在下方，避免按钮挡住主要画面。
BUTTONS = {
    "forward": (120, 292, 120, 54, "前进"),
    "back": (120, 414, 120, 54, "后退"),
    "left": (18, 353, 120, 54, "左转"),
    "stop": (151, 353, 120, 54, "停车"),
    "right": (284, 353, 120, 54, "右转"),
    "enable": (472, 292, 145, 54, "使能"),
    "clear": (630, 292, 145, 54, "清故障"),
    "estop": (472, 366, 303, 54, "急停"),
}

DIRECTION_BUTTONS = ("forward", "back", "left", "right")
ACTION_BUTTONS = ("stop", "enable", "clear", "estop")

# 速度值故意设置得较小，第一次上车测试应先确认轮子悬空。
MOTION = {
    "forward": (0.05, 0.0),
    "back": (-0.04, 0.0),
    "left": (0.0, 0.45),
    "right": (0.0, -0.45),
    "stop": (0.0, 0.0),
}


def connect_lan(timeout_ms=LAN_READY_TIMEOUT_MS):
    """初始化 K230 的有线网卡并等待获得 IP 地址。

    资料中的 network_lan.py 使用 network.LAN() 和 DHCP。
    这里保留静态 IP 分支，方便没有 DHCP 的直连测试。上电自启动时，
    不能只看 ifconfig 是否有地址，还要等网口链路 isconnected() 变为真；
    否则静态 IP 已经写入了，但物理网口还没连上，后续 HTTP 仍然会失败。

    返回：
        (是否成功, 当前 IP 字符串)
    """
    if not KICKPI_LINK_ENABLE:
        print("LAN skipped: local screen/vision test mode")
        return False, "LOCAL"

    lan = network.LAN()

    try:
        if not lan.active():
            lan.active(True)
    except Exception:
        # 某些固件只支持读取 active 状态，不接受 active(True)。
        pass

    try:
        if USE_DHCP:
            lan.ifconfig("dhcp")
        else:
            lan.ifconfig((STATIC_IP, STATIC_MASK, STATIC_GATEWAY, STATIC_DNS))
    except Exception as exc:
        print("LAN config failed:", exc)
        return False, "0.0.0.0"

    try:
        name = lan.netdev_name()
        if name is not None and hasattr(network, "set_default_dev"):
            network.set_default_dev(name)
    except Exception:
        # 没有默认网卡选择 API 时不影响单网口使用。
        pass

    deadline = time.ticks_add(time.ticks_ms(), timeout_ms)
    while time.ticks_diff(deadline, time.ticks_ms()) > 0:
        os.exitpoint()
        try:
            ip = lan.ifconfig()[0]
        except Exception:
            ip = "0.0.0.0"
        try:
            connected = lan.isconnected()
        except Exception:
            connected = True
        if ip != "0.0.0.0" and connected:
            print("LAN ready:", lan.ifconfig())
            return True, ip
        time.sleep_ms(100)

    print("LAN timeout:", lan.ifconfig())
    return False, "0.0.0.0"


def http_request(method, path, payload=None):
    """向 KICKPI API 发一个简单的 HTTP/1.1 请求。

    K230 只需要发送很小的 JSON 命令，因此使用 socket 实现，避免额外安装
    HTTP 客户端库。函数只读取有限长度的响应，防止网络异常时无限等待。

    参数：
        method：GET 或 POST。
        path：例如 /api/v1/control/cmd_vel。
        payload：POST 时传入字典；GET 时传 None。

    返回：
        (成功标志, JSON 响应字典或 None)。
    """
    if not KICKPI_LINK_ENABLE:
        return False, None

    sock = None
    try:
        addr = socket.getaddrinfo(KICKPI_IP, KICKPI_HTTP_PORT)[0][-1]
        sock = socket.socket()
        sock.settimeout(HTTP_TIMEOUT_S)
        sock.connect(addr)

        body = ""
        if payload is not None:
            body = ujson.dumps(payload)
        body_bytes = body.encode()

        request = (
            method
            + " "
            + path
            + " HTTP/1.1\r\n"
            + "Host: "
            + KICKPI_IP
            + "\r\n"
            + "X-Device-Token: "
            + API_TOKEN
            + "\r\n"
            + "Content-Type: application/json\r\n"
            + "Connection: close\r\n"
            + "Content-Length: "
            + str(len(body_bytes))
            + "\r\n\r\n"
        )
        raw_request = request.encode() + body_bytes

        try:
            sock.write(raw_request)
        except AttributeError:
            sock.send(raw_request)

        response = b""
        while len(response) < 4096:
            try:
                chunk = sock.recv(512)
            except Exception:
                break
            if not chunk:
                break
            response += chunk

        first_line = response.split(b"\r\n", 1)[0]
        ok = b" 200 " in first_line or b" 201 " in first_line or b" 204 " in first_line
        response_body = None
        if b"\r\n\r\n" in response:
            response_body = response.split(b"\r\n\r\n", 1)[1]
            try:
                response_body = ujson.loads(response_body)
            except Exception:
                response_body = None
        return ok, response_body
    except Exception as exc:
        print("HTTP failed:", method, path, exc)
        return False, None
    finally:
        if sock is not None:
            try:
                sock.close()
            except Exception:
                pass


def send_motion(button_name):
    """发送一条差速底盘速度命令。

    这里的线速度和角速度单位分别是 m/s 与 rad/s，KICKPI 网关会再根据
    轮距换算成左右轮目标，最终由 STM32 的速度闭环执行。
    """
    linear_x, angular_z = MOTION.get(button_name, (0.0, 0.0))
    return http_request(
        "POST",
        "/api/v1/control/cmd_vel",
        {
            "source": "k230_screen",
            "button": button_name,
            "linear_x_m_s": linear_x,
            "angular_z_rad_s": angular_z,
        },
    )[0]


def send_state(command):
    """发送底盘状态命令：enable、stop、clear_fault 或 estop。"""
    return http_request(
        "POST",
        "/api/v1/control/state",
        {"source": "k230_screen", "command": command},
    )[0]


def get_latest():
    """读取 KICKPI 最新状态，用于刷新屏幕顶部的网络和故障提示。"""
    ok, data = http_request("GET", "/api/v1/latest")
    if ok and isinstance(data, dict):
        # 网关的响应格式是 {"ok": true, "data": {...}}，屏幕只需要 data。
        latest = data.get("data", {})
        if isinstance(latest, dict):
            return True, latest
    return ok, {}


def point_to_button(point):
    """把触摸点转换为按钮名称；不在按钮区域时返回 None。"""
    for name, rect in BUTTONS.items():
        x, y, width, height, _ = rect
        if x <= point.x < x + width and y <= point.y < y + height:
            return name
    return None


def read_pressed_button(touch, now_ms, hold_name, hold_seen_ms):
    """读取当前触摸状态，并对短暂抖动做去抖处理。

    说明：
        某些触摸屏在长按时会在 DOWN / MOVE / UP 之间短暂抖动。
        这里保留最近一次有效触摸按钮一小段时间，避免按钮状态闪烁。
    """
    points = touch.read(1)
    candidate = None

    if len(points):
        for point in points:
            if hasattr(point, "event") and point.event == TOUCH.EVENT_UP:
                continue
            candidate = point_to_button(point)
            if candidate is not None:
                break

    if candidate is not None:
        return candidate, now_ms

    if hold_name is not None and time.ticks_diff(now_ms, hold_seen_ms) <= TOUCH_RELEASE_HOLD_MS:
        return hold_name, hold_seen_ms

    return None, hold_seen_ms


def draw_button(img, name, pressed_name):
    """绘制一个按钮，并对当前按下的按钮使用不同颜色。"""
    x, y, width, height, title = BUTTONS[name]
    if name == "estop":
        color = (255, 200, 35, 35)
    elif name == "stop":
        color = (255, 220, 145, 30)
    elif name == pressed_name:
        color = (255, 35, 155, 95)
    else:
        color = (230, 35, 45, 55)

    img.draw_rectangle(x, y, width, height, color=color, fill=True)
    img.draw_rectangle(x, y, width, height, color=(255, 235, 240, 245), thickness=2)
    text_x = x + max(8, (width - len(title) * 28) // 2)
    text_y = y + 12
    img.draw_string_advanced(text_x, text_y, 26, title, color=(255, 255, 255, 255))


def draw_detections(img, detections):
    """在摄像头区域绘制识别框和类别，避免覆盖底部控制按钮。"""
    for item in detections:
        try:
            x = int(item.get("x", 0))
            y = int(item.get("y", 0))
            width = int(item.get("w", 0))
            height = int(item.get("h", 0))
            label = str(item.get("label", "object"))
            score = float(item.get("score", 0.0))
        except Exception:
            continue

        # 画面下方是触摸控制区；识别框只显示在上方，防止遮挡按钮文字。
        if y >= 282 or width <= 0 or height <= 0:
            continue
        height = min(height, 282 - y)
        img.draw_rectangle(x, y, width, height, color=(255, 40, 220, 90), thickness=3)
        img.draw_string_advanced(x, max(44, y - 28), 20, "%s %.2f" % (label, score), color=(255, 255, 230, 80))


def draw_overlay(img, pressed_name, ip_address, online, latest, detections):
    """绘制顶部状态栏和下方触摸控制区。"""
    img.clear()

    # 半透明状态栏，摄像头画面仍然从视频层显示在背景。
    img.draw_rectangle(0, 0, DISPLAY_WIDTH, 42, color=(190, 10, 15, 20), fill=True)
    if KICKPI_LINK_ENABLE:
        net_text = "KICKPI: ONLINE" if online else "KICKPI: OFFLINE"
        net_color = (255, 80, 220, 110) if online else (255, 255, 70, 70)
        ip_text = "IP " + ip_address
    else:
        net_text = "K230: LOCAL TEST"
        net_color = (255, 80, 220, 110)
        ip_text = "IP LOCAL"
    img.draw_string_advanced(12, 10, 22, net_text, color=net_color)
    img.draw_string_advanced(255, 10, 22, ip_text, color=(255, 230, 230, 230))

    faults = 0
    if isinstance(latest, dict):
        faults = latest.get("faults", 0)
    fault_text = "FAULT 0x%04X" % int(faults)
    fault_color = (255, 255, 80, 80) if faults else (255, 100, 230, 130)
    img.draw_string_advanced(560, 10, 22, fault_text, color=fault_color)

    # 把关键传感器数据放在画面上方，不占用摄像头和底部按钮区域。
    battery = latest.get("battery_voltage_v") if isinstance(latest, dict) else None
    env_temp = latest.get("environment_temperature_c") if isinstance(latest, dict) else None
    env_humi = latest.get("environment_humidity_percent") if isinstance(latest, dict) else None
    battery_text = "BAT --" if battery is None else "BAT %.2fV" % float(battery)
    env_text = "ENV --" if env_temp is None or env_humi is None else "ENV %.1fC %.1f%%" % (float(env_temp), float(env_humi))
    img.draw_string_advanced(12, 112, 20, battery_text, color=(255, 255, 230, 210))
    img.draw_string_advanced(160, 112, 20, env_text, color=(255, 255, 230, 210))

    if detections:
        img.draw_string_advanced(12, 85, 20, "DETECT %d" % len(detections), color=(255, 255, 220, 80))
        draw_detections(img, detections)

    # 半透明控制底板，避免按钮和摄像头内容混在一起。
    img.draw_rectangle(0, 282, DISPLAY_WIDTH, 198, color=(205, 8, 12, 18), fill=True)
    for name in BUTTONS:
        draw_button(img, name, pressed_name)

    if pressed_name in DIRECTION_BUTTONS:
        command_text = "RUN " + pressed_name.upper()
    elif pressed_name in ACTION_BUTTONS:
        command_text = "ACTION " + pressed_name.upper()
    else:
        command_text = "READY"
    img.draw_string_advanced(18, 57, 22, command_text, color=(255, 255, 255, 255))


def send_vision_event(frame_id, detections):
    """把 K230 识别结果发送给 KICKPI。

    你的 Cron 模型当前输出的类别是 rust，上传结果示例是：
        [{"label": "rust", "class_id": 0, "score": 0.91,
          "x": 10, "y": 20, "w": 80, "h": 120}]

    KICKPI 会保存原始识别 JSON，并将每个目标拆分保存到 vision_detections 表。
    """
    return http_request(
        "POST",
        "/api/v1/vision/events",
        {
            "source": "k230",
            "frame_id": int(frame_id),
            "detections": detections,
        },
    )[0]


def init_vision():
    """按照 Cron 部署包初始化 AnchorBaseDet 检测对象。

    Cron 不是普通 YOLOv8 模型，必须使用资料中的
    ``libs.PlatTasks.DetectionApp``，并把 anchors、类别、输入尺寸和阈值从
    ``deploy_config.json`` 读取。模型文件不存在或固件没有 AI 库时返回 None，
    让主程序仍然可以作为普通摄像头触摸驾驶程序运行。

    返回：
        DetectionApp 实例；失败时返回 None。
    """
    if not ENABLE_VISION:
        print("vision disabled by configuration")
        return None

    try:
        deploy_conf = None
        deploy_config_path = None
        for root in MODEL_ROOT_CANDIDATES:
            candidate = root.rstrip("/") + "/deploy_config.json"
            try:
                os.stat(candidate)
                deploy_config_path = candidate
                break
            except Exception:
                continue

        if deploy_config_path is None:
            raise FileNotFoundError("deploy_config.json not found in Cron or sdcard paths")

        deploy_conf = read_json(deploy_config_path)
        model_root = deploy_config_path.rsplit("/", 1)[0]
        model_name = str(deploy_conf["kmodel_path"])
        if model_name.startswith("./"):
            model_name = model_name[2:]
        model_path = model_name if model_name.startswith("/") else model_root.rstrip("/") + "/" + model_name
        os.stat(model_path)

        labels = deploy_conf["categories"]
        model_input_size = deploy_conf["img_size"]
        model_type = deploy_conf["model_type"]
        anchors = []
        if model_type == "AnchorBaseDet":
            # 三个检测尺度的 anchors 按资料 det_video.py 的顺序展平。
            anchors = deploy_conf["anchors"][0] + deploy_conf["anchors"][1] + deploy_conf["anchors"][2]

        det_app = DetectionApp(
            "video",
            model_path,
            labels,
            model_input_size,
            anchors,
            model_type,
            deploy_conf["confidence_threshold"],
            deploy_conf["nms_threshold"],
            AI_FRAME_SIZE,
            [DISPLAY_WIDTH, DISPLAY_HEIGHT],
            debug_mode=0,
        )
        det_app.config_preprocess()
        # 让后面的坐标转换和屏幕标签都使用配置文件中的真实类别。
        det_app.robot_labels = labels
        print("vision model ready:")
        print("  root:", model_root)
        print("  type:", model_type)
        print("  labels:", labels)
        print("  input:", model_input_size)
        print("  model:", model_path)
        return det_app
    except BaseException as exc:
        print("vision disabled, model init failed:", exc)
        return None


def detections_from_result(result, labels=None):
    """把 Cron DetectionApp 的结果字典转成屏幕和网络 JSON 列表。

    DetectionApp 的 boxes 是 ``[x1, y1, x2, y2]``，坐标位于 AI 输入通道
    ``640x360`` 内；这里转换成 ``x/y/w/h``，同时将坐标缩放到 800x480 LCD。
    """
    if not isinstance(result, dict):
        return []

    boxes = result.get("boxes", [])
    class_ids = result.get("idx", [])
    scores = result.get("scores", [])
    labels = labels if labels is not None else ["rust"]
    detections = []
    for index in range(len(boxes)):
        try:
            x1, y1, x2, y2 = boxes[index]
            class_id = int(class_ids[index])
            score = float(scores[index])
            label = labels[class_id] if 0 <= class_id < len(labels) else str(class_id)
            x = float(x1) * DISPLAY_WIDTH / AI_FRAME_SIZE[0]
            y = float(y1) * DISPLAY_HEIGHT / AI_FRAME_SIZE[1]
            width = float(x2 - x1) * DISPLAY_WIDTH / AI_FRAME_SIZE[0]
            height = float(y2 - y1) * DISPLAY_HEIGHT / AI_FRAME_SIZE[1]
            detections.append(
                {
                    "label": label,
                    "class_id": class_id,
                    "score": score,
                    "x": x,
                    "y": y,
                    "w": width,
                    "h": height,
                }
            )
        except Exception:
            # 单个异常框不能影响这一帧其它目标和驾驶控制。
            continue
    return detections


def main():
    """初始化网络、相机、LCD 和触摸屏，然后进入驾驶主循环。"""
    os.exitpoint(os.EXITPOINT_ENABLE)
    if KICKPI_LINK_ENABLE:
        # main.py 上电自动运行时，先给网口 PHY、KICKPI 对端和交换状态一点时间。
        time.sleep_ms(BOOT_NETWORK_DELAY_MS)
        lan_ok, ip_address = connect_lan()
    else:
        lan_ok, ip_address = False, "LOCAL"
    touch = TOUCH(0)

    # PipeLine 会把摄像头视频绑定到 LCD 的 VIDEO1 层，并创建 OSD 图层。
    pipeline = PipeLine(
        rgb888p_size=AI_FRAME_SIZE,
        display_mode="lcd",
        display_size=[DISPLAY_WIDTH, DISPLAY_HEIGHT],
        osd_layer_num=1,
    )
    pipeline.create(
        to_ide=True,
        hmirror=CAMERA_HMIRROR,
        vflip=CAMERA_VFLIP,
    )

    rtsp_started = False
    if VIDEO_RTSP_ENABLE and WBCRtsp is not None:
        try:
            # WBCRtsp 内部从已经初始化的 Display 获取实际 LCD 分辨率，
            # 当前资料版本的端口固定为 8554、会话名固定为 test。
            WBCRtsp.configure(DISPLAY_WIDTH, DISPLAY_HEIGHT)
            WBCRtsp.start()
            print("K230 RTSP:", "rtsp://" + ip_address + ":" + str(VIDEO_RTSP_PORT) + "/test")
            rtsp_started = True
        except BaseException as exc:
            # 视频推流失败时只关闭视频，不影响本地画面、识别和运动控制。
            print("RTSP start failed, continue without video:", exc)

    vision = init_vision()

    pressed_name = None
    pressed_seen_ms = 0
    last_direction = None
    last_action = None
    last_control_ms = time.ticks_ms()
    last_status_ms = time.ticks_ms()
    last_vision_ms = time.ticks_ms()
    last_network_retry_ms = time.ticks_ms()
    frame_id = 0
    online = lan_ok
    latest = {}
    detections = []

    try:
        while True:
            os.exitpoint()
            now = time.ticks_ms()
            current, pressed_seen_ms = read_pressed_button(touch, now, pressed_name, pressed_seen_ms)
            pressed_name = current

            # 只有成功初始化模型时才取 AI 通道；视频层仍由 PipeLine 持续显示。
            if vision is not None:
                frame_id += 1
                try:
                    result = vision.run(pipeline.get_frame())
                    detections = detections_from_result(result, getattr(vision, "robot_labels", None))
                except BaseException as exc:
                    # 模型运行异常时保留相机和触摸控制，避免 AI 问题变成失控问题。
                    print("vision runtime failed, disable vision:", exc)
                    try:
                        vision.deinit()
                    except Exception:
                        pass
                    vision = None
                    detections = []
            if vision is not None and time.ticks_diff(now, last_vision_ms) >= VISION_SEND_INTERVAL_MS:
                send_vision_event(frame_id, detections)
                last_vision_ms = now

            if KICKPI_LINK_ENABLE:
                # 如果开机时网络没起来，或者 KICKPI 网关晚启动，主循环会定期重试。
                # 这样不用手动停止脚本再运行一次，屏幕会在链路恢复后自动变为 ONLINE。
                if not online and time.ticks_diff(now, last_network_retry_ms) >= NETWORK_RETRY_MS:
                    retry_ok, retry_ip = connect_lan(LAN_RETRY_TIMEOUT_MS)
                    last_network_retry_ms = now
                    if retry_ok:
                        ip_address = retry_ip
                        online = True

                # 方向按钮是“按住运行”模式，持续发心跳，避免任何一层失联后继续跑。
                if current in DIRECTION_BUTTONS:
                    if current != last_direction or time.ticks_diff(now, last_control_ms) >= CONTROL_HEARTBEAT_MS:
                        online = send_motion(current) or online
                        last_control_ms = now
                    last_direction = current
                    last_action = None
                else:
                    if last_direction is not None:
                        online = send_motion("stop") or online
                        last_direction = None
                        last_control_ms = now

                    # 动作按钮只在刚按下时触发一次，防止长按重复发送急停/清故障。
                    if current in ACTION_BUTTONS:
                        if current != last_action:
                            command = "stop" if current == "stop" else current
                            if current == "clear":
                                command = "clear_fault"
                            online = send_state(command) or online
                            if current == "stop":
                                online = send_motion("stop") or online
                            last_action = current
                    else:
                        last_action = None

                if time.ticks_diff(now, last_status_ms) >= STATUS_REFRESH_MS:
                    status_ok, status_data = get_latest()
                    online = status_ok
                    if status_ok:
                        latest = status_data
                    elif last_direction is not None:
                        # 网关失联时，先清除本地运行态；网关自身 watchdog 也会停车。
                        send_motion("stop")
                        last_direction = None
                    last_status_ms = now
            else:
                # 本地测试模式只验证屏幕、触摸和识别，不发送任何 KICKPI 网络请求。
                last_direction = current if current in DIRECTION_BUTTONS else None
                last_action = current if current in ACTION_BUTTONS else None

            draw_overlay(pipeline.osd_img, current, ip_address, online, latest, detections)
            pipeline.show_image()
            gc.collect()

    except KeyboardInterrupt:
        print("user stop")
    except BaseException as exc:
        printer = getattr(sys, "print_exception", None)
        if printer is not None:
            printer(exc)
        else:
            print(repr(exc))
    finally:
        # 任何退出路径都先停车，再释放摄像头、显示和触摸相关资源。
        try:
            send_motion("stop")
            send_state("stop")
        except Exception:
            pass
        if vision is not None:
            try:
                vision.deinit()
            except Exception:
                pass
        if rtsp_started and WBCRtsp is not None:
            try:
                WBCRtsp.stop()
            except Exception as exc:
                print("RTSP stop failed:", exc)
        pipeline.destroy()
        os.exitpoint(os.EXITPOINT_ENABLE_SLEEP)
        time.sleep_ms(100)


if __name__ == "__main__":
    main()
