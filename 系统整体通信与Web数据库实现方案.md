# 机器人系统整体通信、Web 可视化与数据库实现方案

更新时间：2026-08-27

## 1. 最终推荐架构

本项目建议采用“两条通信链路”：

```text
                         以太网 / 局域网
K230 摄像头识别  --------------------------------->  KICKPI H618
       |                                             |
       |                                             +-- Web 页面
       |                                             +-- HTTP API
       |                                             +-- SQLite 数据库
       |                                             +-- ROS2 视觉话题
       |                                             |
       |                                      UART5 /dev/ttyAS5
       |                                             |
       +---------------------------------------------+
                                                     |
                                                   STM32
                                                     |
                                      电机、编码器、DHT30、MPU6050、
                                      INA219、DS18B20、继电器
```

实际数据流：

```text
STM32
  -> UART1/USART1
KICKPI /dev/ttyAS5
  -> robot_base_bridge
ROS2 话题
  -> robot_gateway
SQLite 数据库 + HTTP API + 浏览器轮询
  -> 浏览器
```

```text
K230 识别程序
  -> 以太网 HTTP POST JSON
KICKPI robot_gateway
  -> 保存 recognition_event
  -> 发布 /vision/detections
  -> 推送到 Web 页面
```

## 2. 为什么推荐以太网

### 2.1 K230 到 KICKPI 推荐以太网

识别结果使用网络传输，推荐：

```text
K230 作为 HTTP 客户端
KICKPI 作为 HTTP 服务端
```

选择以太网的原因：

- K230 识别结果本质上是结构化数据，不适合和底盘控制帧混在一起。
- JSON 便于调试，可以直接使用浏览器、`curl` 或 Python 发送测试数据。
- K230 识别频率变化时，网络传输比固定波特率串口更容易扩展。
- 将来可以增加图片地址、目标框、置信度、类别、时间戳，而不用重新设计串口帧。
- KICKPI 已经运行 Linux 和 ROS2，直接运行 HTTP 服务、数据库和 Web 服务最方便。
- 以后要增加实时视频，可以另设 RTSP/WebRTC 通道，不影响识别结果和底盘控制。

K230 官方 CanMV 文档提供了有线 LAN、TCP Client、HTTP Client 等网络示例；其
LAN 示例使用 `network.LAN()` 获取网络接口，TCP 示例使用标准 socket 连接服务端。
本项目可以先用同样的网络能力发送识别 JSON。

### 2.2 什么时候才考虑串口

只有以下情况才优先考虑 K230 串口：

- K230 没有可用以太网接口。
- 只发送很低频、很短的状态文本。
- 现场没有交换机、路由器或网线。
- 需要极低延迟且数据量很小。

如果采用串口，K230 必须使用 KICKPI 的另一条独立串口，不能和 KICKPI 到
STM32 的 `/dev/ttyAS5` 混用，也不能把识别数据插入 STM32 的电机控制协议。
因此本项目不推荐串口作为第一方案。

## 3. 当前 KICKPI 与 STM32 链路

KICKPI 当前已经配置：

```text
KICKPI IP：192.168.1.119
ROS2：Humble
STM32 串口：/dev/ttyAS5
波特率：115200
```

KICKPI 与 STM32 使用 KICKPI 40Pin 的普通 UART5：

```text
KICKPI 8脚  UART5_TX -> STM32 PA10 / USART1_RX
KICKPI 10脚 UART5_RX -> STM32 PA9  / USART1_TX
KICKPI 6脚  GND      -> STM32 GND
```

STM32 固件只有在下面宏改成 0 后，才允许启动 ROS2 桥接：

```c
#define ROBOT_UART1_DEBUG_ONLY 0
```

这一条链路只负责：

- `/cmd_vel` 到 STM32 的电机命令
- 编码器回传
- 电池电压
- DHT30 温湿度
- MPU6050
- DS18B20
- 继电器和故障状态

K230 识别结果不要经过这条链路。

## 4. K230 与 KICKPI 的网络规划

推荐先把两个设备接到同一个路由器或交换机：

```text
路由器/交换机
  |-- KICKPI：192.168.1.119
  |-- K230：DHCP 获取地址，或设置为 192.168.1.120
```

如果采用静态地址，建议：

```text
KICKPI：192.168.1.119
K230：  192.168.1.120
子网掩码：255.255.255.0
网关：按现场路由器地址填写
```

K230 端测试：

```text
ping 192.168.1.119
```

KICKPI 端测试：

```bash
ping <K230的IP>
```

如果是两个设备直接用一根网线连接，不经过路由器，也可以使用静态地址：

```text
KICKPI：192.168.1.119/24
K230：  192.168.1.120/24
```

这种方式不提供 DHCP，两个设备必须手动配置地址。

## 5. K230 发送的数据格式

K230 每次识别到一帧结果，就向 KICKPI 发送：

```text
POST http://192.168.1.119:8080/api/v1/vision/events
Content-Type: application/json
X-Device-Token: robot-dev-token
```

请求 JSON：

```json
{
  "source": "k230",
  "frame_id": 1523,
  "timestamp_ms": 1787831400123,
  "image_width": 640,
  "image_height": 480,
  "latency_ms": 38,
  "detections": [
    {
      "label": "rust",
      "class_id": 0,
      "score": 0.93,
      "x": 120,
      "y": 80,
      "w": 190,
      "h": 360
    }
  ]
}
```

字段说明：

| 字段 | 说明 |
| --- | --- |
| `source` | 数据来源，固定填写 `k230` |
| `frame_id` | K230 递增帧号，方便发现丢帧 |
| `timestamp_ms` | K230 产生结果的时间，不能用来替代 KICKPI 入库时间 |
| `image_width/height` | 原图尺寸；当前屏幕程序按 `800x480` 发送 |
| `latency_ms` | K230 从采集到识别完成的耗时 |
| `detections` | 这一帧的所有识别结果 |
| `label` | 类别名称 |
| `class_id` | 模型类别编号 |
| `score` | 置信度，范围 0 到 1；网关按此字段入库 |
| `x/y/w/h` | 目标框左上角坐标、宽度和高度，单位为像素 |

当前 `Cron` 模型的输出不是 `person/bottle` 等通用类别，而是：

- 模型类型：`AnchorBaseDet`；
- 类别：`rust`；
- 类别编号：`0`；
- K230 `DetectionApp` 原始结果为 `boxes`、`scores`、`idx`；
- `robot_touch_drive.py` 会把原始结果转换为上面网关使用的 `label/class_id/score/x/y/w/h` 格式。

不建议一开始把完整图片直接放入数据库。建议：

- 识别 JSON 放数据库。
- 重要图片按日期保存到文件目录。
- 数据库只保存图片路径或图片 URL。
- 后续需要视频时再单独做视频流。

## 6. KICKPI 后端组成

建议在 KICKPI 上增加一个独立的 `robot_gateway` 服务：

```text
/root/kickpi_ros2_ws/src/robot_gateway
```

它包含四个职责：

### 6.1 HTTP API 服务

提供 K230 写入识别结果的接口：

```text
POST /api/v1/vision/events
```

提供 Web 页面读取数据的接口：

```text
GET /api/v1/health
GET /api/v1/latest
GET /api/v1/telemetry?limit=300
GET /api/v1/vision/events?limit=100
GET /api/v1/vision/events/{id}
```

### 6.2 ROS2 视觉话题

收到 K230 数据后，发布：

```text
/vision/detections
```

初期可以使用 `std_msgs/msg/String`，消息内容就是原始 JSON。
后续如果需要正式的目标框消息，再创建自定义 `Detection2DArray` 接口。

### 6.3 传感器数据入库

`robot_gateway` 订阅现有 ROS2 话题：

```text
/battery_state
/env/temperature
/env/humidity
/battery/temperature
/imu/raw
/odom
/base/faults
/vision/detections
```

传感器数据不要由 STM32 直接访问数据库，STM32 只负责实时采集和上报。
数据库写入统一放在 KICKPI，避免数据库操作影响电机控制。

### 6.4 Web 数据刷新

当前已实现版本使用浏览器原生 `fetch` 每 1 秒轮询：

```text
GET /api/v1/latest
GET /api/v1/telemetry?limit=120
GET /api/v1/vision/events?limit=8
```

这样不依赖额外 WebSocket 库，适合先在局域网完成硬件联调。后续需要更低延迟时，
再将读取接口升级成 SSE 或 WebSocket，不影响数据库和控制接口。

## 7. 数据库设计

单台小车建议先使用 SQLite，不需要 PostgreSQL。SQLite 文件建议放在：

```text
/root/robot_data/robot.db
```

开启：

```text
WAL 模式
busy_timeout
定期提交
按日期备份
```

### 7.1 `telemetry` 传感器表

```sql
CREATE TABLE telemetry (
    id INTEGER PRIMARY KEY AUTOINCREMENT,
    recorded_at TEXT NOT NULL,
    recorded_at_ms INTEGER NOT NULL,
    battery_voltage_v REAL,
    environment_temperature_c REAL,
    environment_humidity_rh REAL,
    battery_temperature_c REAL,
    imu_ax REAL,
    imu_ay REAL,
    imu_az REAL,
    imu_gx REAL,
    imu_gy REAL,
    imu_gz REAL,
    odom_x_m REAL,
    odom_y_m REAL,
    odom_yaw_rad REAL,
    fault_code INTEGER NOT NULL DEFAULT 0
);
CREATE INDEX idx_telemetry_recorded_at ON telemetry(recorded_at_ms);
```

不要每个 IMU 消息都立即写一次磁盘。建议先在内存中合并最新数据，默认每
1000ms 写一条汇总记录。这样可以减少 eMMC 写入量。

### 7.2 `vision_events` 识别事件表

```sql
CREATE TABLE vision_events (
    id INTEGER PRIMARY KEY AUTOINCREMENT,
    received_at TEXT NOT NULL,
    received_at_ms INTEGER NOT NULL,
    source TEXT NOT NULL,
    frame_id INTEGER,
    image_width INTEGER,
    image_height INTEGER,
    latency_ms INTEGER,
    payload_json TEXT NOT NULL,
    image_path TEXT
);
CREATE INDEX idx_vision_events_received_at ON vision_events(received_at_ms);
```

### 7.3 `vision_detections` 目标表

```sql
CREATE TABLE vision_detections (
    id INTEGER PRIMARY KEY AUTOINCREMENT,
    event_id INTEGER NOT NULL,
    label TEXT NOT NULL,
    class_id INTEGER,
    confidence REAL NOT NULL,
    x1 REAL,
    y1 REAL,
    x2 REAL,
    y2 REAL,
    FOREIGN KEY(event_id) REFERENCES vision_events(id)
);
CREATE INDEX idx_vision_detections_event_id ON vision_detections(event_id);
```

保留 `payload_json` 是为了兼容后续增加模型字段；拆开的
`vision_detections` 便于 Web 页面按类别和置信度查询。

## 8. Web 页面功能

第一版不要做复杂管理系统，先做一个单页仪表盘：

### 8.1 实时状态区

显示：

- 电池电压
- 环境温度
- 环境湿度
- 电池温度
- IMU 是否在线
- DHT30 是否在线
- INA219 是否在线
- 底盘故障码
- 继电器状态
- 最后一条识别结果

### 8.2 图表区

显示最近一段时间：

- 电池电压曲线
- 环境温度曲线
- 环境湿度曲线
- 电池温度曲线
- 速度或里程计曲线

第一版可以直接使用浏览器原生 Canvas 绘图，不依赖外部 CDN。

### 8.3 识别区

显示：

- 最新识别时间
- 类别
- 置信度
- 目标框坐标
- 每秒识别数量
- 最近识别记录表

如果后续保存了图片，页面再加载图片并绘制目标框。

访问地址计划为：

```text
http://192.168.1.119:8080
```

## 9. K230 端发送逻辑

K230 识别主循环不要因为网络失败而停止摄像头和推理。正确逻辑：

```text
采集图像
  -> AI 推理
  -> 生成 detections
  -> 尝试发送 HTTP
  -> 发送失败时记录错误并丢弃或缓存
  -> 继续下一帧
```

建议：

- 不要每一帧都发送高分辨率图片。
- 识别结果可以限制为每秒 5 到 10 次。
- 目标框没有变化时，可以降低发送频率。
- HTTP 失败时超时时间设置为 200ms 到 500ms，不能无限阻塞识别。
- KICKPI 恢复后，K230 自动重试。
- `frame_id` 必须递增，便于后端判断丢帧。

CanMV MicroPython 端可采用：

```python
import network
import socket
import time
import ujson

KICKPI_IP = "192.168.1.119"
KICKPI_PORT = 8080

def connect_lan():
    """启用 K230 有线网卡并通过 DHCP 获取地址。"""
    lan = network.LAN()
    if not lan.active():
        lan.active(1)
    lan.ifconfig("dhcp")
    while lan.ifconfig()[0] == "0.0.0.0":
        time.sleep_ms(100)
    return lan.ifconfig()[0]

def send_event(event):
    """用最小 HTTP/1.1 POST 把一次识别结果发给 KICKPI。"""
    body = ujson.dumps(event)
    request = (
        "POST /api/v1/vision/events HTTP/1.1\r\n"
        "Host: {}:{}\r\n"
        "Content-Type: application/json\r\n"
        "X-Device-Token: robot-dev-token\r\n"
        "Content-Length: {}\r\n"
        "Connection: close\r\n"
        "\r\n"
        "{}"
    ).format(KICKPI_IP, KICKPI_PORT, len(body), body)

    sock = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
    sock.settimeout(0.5)
    try:
        addr = socket.getaddrinfo(KICKPI_IP, KICKPI_PORT)[0][-1]
        sock.connect(addr)
        sock.send(request.encode())
        response = sock.recv(128)
        return response.startswith(b"HTTP/1.1 2")
    except Exception as exc:
        print("send event failed:", exc)
        return False
    finally:
        sock.close()
```

这段代码是通信测试模板。接入你的实际识别程序时，只需要把识别结果填入
`event["detections"]`，不要把摄像头采集和网络发送写成互相阻塞的死循环。

## 10. 服务部署顺序

KICKPI 上计划运行三个独立服务：

```text
robot-base-bridge.service   ROS2 底盘串口桥接
robot-gateway.service        Web/API/数据库/视觉接收
```

不建议把所有功能都塞进一个 Python 文件。分服务后：

- 底盘串口异常不会直接让 Web 页面退出。
- Web 页面重启不会停止 STM32 串口控制。
- 数据库故障不会直接影响电机控制任务。
- 可以单独查看日志和重启。

启动顺序：

```text
网络
  -> robot-base-bridge.service
  -> robot-gateway.service
```

实际执行顺序：

1. 先确认 STM32 已切换到二进制通信模式。
2. 手动启动 `robot-base-bridge`。
3. 确认 `/battery_state`、`/odom`、`/base/faults` 能看到数据。
4. 启动 `robot-gateway`。
5. 用 KICKPI 本机 `curl` 发送一条假识别数据。
6. 用浏览器打开 `http://192.168.1.119:8080`。
7. 确认数据库有记录。
8. 再把 K230 接入。
9. 最后再设置两个服务开机启动。

## 11. 分阶段实现清单

### 阶段一：网络通信

- K230 和 KICKPI 接入同一个交换机或路由器。
- 确认 K230 可以 ping 通 `192.168.1.119`。
- KICKPI 开放本机 TCP `8080` 监听。
- 用假 JSON 验证 POST 接口。

### 阶段二：后端接口

- 创建 `robot_gateway` ROS2 Python 包。
- 实现 SQLite 初始化和建表。
- 实现 `/api/v1/health`。
- 实现 `/api/v1/vision/events`。
- 实现 `/api/v1/latest`。
- 实现 `/api/v1/telemetry`。
- 实现鉴权 Token。

### 阶段三：接入底盘传感器

- 订阅 `robot_base_bridge` 发布的话题。
- 每秒写一条 telemetry 汇总记录。
- 检查 DHT30、INA219、DS18B20、MPU6050 的有效状态。
- 把故障码一并入库。

### 阶段四：Web 页面

- 做实时状态区。
- 做温度、湿度、电压曲线。
- 做识别结果表。
- 当前使用 1 秒 HTTP 轮询；后续再按需要升级 SSE 或 WebSocket。
- 页面断线后自动重连。

### 阶段五：K230 正式接入

- 使用 CanMV MicroPython 的 `network.LAN()` 初始化有线网卡。
- 使用 `libs.PlatTasks.DetectionApp` 加载 `Cron` 的 `AnchorBaseDet` 模型。
- 从 `deploy_config.json` 读取 `320x320` 输入尺寸、anchors、阈值和 `rust` 类别。
- 将识别结果按本文 JSON 格式发送；当前程序默认每 `500 ms` 上传一次。
- 加入网络失败重试，网络失败时不能停止摄像头和本地触摸控制。
- 用 `frame_id` 验证没有重复或乱序。
- 控制识别发送频率，避免阻塞推理和控制心跳。

### 阶段六：开机启动和安全

- 安装两个 systemd 服务。
- 先手动验证，再 `systemctl enable`。
- Web 接口只监听局域网。
- 更换默认 Token。
- 不把摄像头原图无限保存到 eMMC。
- 数据库定期备份到 SD 卡或外部电脑。

## 12. 识别结果参与避障时的原则

K230 识别结果如果以后用于避障，不要直接操作 STM32 GPIO，也不要让 K230
直接给电机发 PWM。

正确路径：

```text
K230 识别结果
  -> KICKPI /vision/detections
  -> vision_safety_node
  -> 速度限制或零速度
  -> /cmd_vel
  -> robot_base_bridge
  -> STM32
```

这样 STM32 的通信超时、欠压、过温和急停保护仍然有效，K230 只是上层感知源。

## 13. 当前已经确定的 K230 参数

之前需要确认的两个参数现在已经从 `D:\cxdownload\end\K230` 和
`D:\cxdownload\end\Cron` 中确定：

1. 运行环境采用 K230 CanMV MicroPython，不是 Windows Python，也不是普通 Linux Python。
2. 模型采用 `libs.PlatTasks.DetectionApp` 的 `AnchorBaseDet` 分支。
3. 模型类别只有 `rust`，类别编号为 `0`。
4. 模型输入尺寸为 `320x320`，部署配置中的 anchors、置信度阈值 `0.4` 和 NMS 阈值 `0.5` 已经写入 K230 程序的初始化流程。
5. 模型原始结果为 `boxes/scores/idx`，发送给 KICKPI 前转换为 `x/y/w/h` 和 `score`。

因此，KICKPI 网关、数据库和 K230 程序已经可以按真实模型字段联调。剩余工作不是
重新设计接口，而是把程序复制到 K230 后验证摄像头、屏幕、NPU 推理帧率和网络稳定性。

## 14. K230 屏幕本地控制界面

### 14.1 界面目标

K230 屏幕可以同时承担三个功能：

1. 显示摄像头实时画面。
2. 显示识别框和识别类别。
3. 通过触摸按钮控制小车运动和工作状态。

建议使用横屏布局：

```text
+------------------------------------------------+
|                                                |
|              摄像头画面/识别框                 |
|                                                |
+-----------------------------+------------------+
|        前进                  | 电池 12.1V      |
| 左转   停止   右转           | 状态 READY      |
|        后退                  | 温度/湿度/故障  |
+-----------------------------+------------------+
```

下方按钮不要做成“点击一次一直运动”，而应采用按住运动、松手停止：

- 手指按下 `前进`：发送前进速度。
- 手指持续按住：每 100ms 发送一次心跳和速度。
- 手指松开：立即发送零速度。
- 触摸丢失或网络断开：KICKPI 超时后自动发送零速度。

### 14.2 推荐按钮

第一版只放这些按钮：

```text
前进
后退
左转
右转
停止
使能
清故障
```

停止按钮应该比方向按钮更大，并且放在方向按钮中心位置。
`使能`、`清故障` 使用点击动作；方向按钮使用按住动作。

### 14.3 K230 与 KICKPI 的控制接口

K230 控制请求发给 KICKPI：

```text
POST http://192.168.1.119:8080/api/v1/control/cmd_vel
Content-Type: application/json
X-Device-Token: robot-dev-token
```

前进示例：

```json
{
  "source": "k230_screen",
  "linear_x_m_s": 0.05,
  "angular_z_rad_s": 0.0,
  "button": "forward",
  "seq": 1001
}
```

左转示例：

```json
{
  "source": "k230_screen",
  "linear_x_m_s": 0.0,
  "angular_z_rad_s": 0.5,
  "button": "left",
  "seq": 1002
}
```

松手停止：

```json
{
  "source": "k230_screen",
  "linear_x_m_s": 0.0,
  "angular_z_rad_s": 0.0,
  "button": "release",
  "seq": 1003
}
```

KICKPI 收到后做以下动作：

1. 检查 Token。
2. 限制线速度和角速度。
3. 检查当前故障码。
4. 发布 ROS2 `/cmd_vel`。
5. 返回当前底盘状态。
6. 记录控制来源和命令序号。

建议的 KICKPI 限制：

```text
最大线速度：0.20m/s
最大角速度：1.00rad/s
控制心跳：100ms
KICKPI 控制超时：300ms
STM32 串口命令超时：700ms
```

### 14.4 K230 屏幕状态刷新

K230 每 500ms 请求一次：

```text
GET http://192.168.1.119:8080/api/v1/latest
```

页面显示：

- `READY`、`RUN`、`FAULT`
- 故障码
- 电池电压
- 环境温度和湿度
- 电池温度
- 继电器状态
- 最后一条识别类别和置信度
- KICKPI 网络是否在线

网络断开时，界面必须：

- 状态栏显示 `OFFLINE`
- 禁止继续发送运动命令
- 自动发送一次停止
- 网络恢复后再允许控制

### 14.5 CanMV 的实现方式

如果 K230 运行的是 CanMV MicroPython，摄像头显示可以使用官方
`PipeLine`，把 `display_mode` 设置为 `lcd`；触摸可以使用：

```python
from machine import TOUCH

touch = TOUCH(0)
points = touch.read()
```

读取到 `x`、`y` 后判断是否落在按钮矩形区域内。

第一版建议采用“摄像头图像 + 画按钮矩形”的方式，不必一开始就把完整界面
做成 LVGL。原因是：

- 代码更简单。
- 摄像头画面和识别框容易叠加。
- 触摸坐标判断容易调试。
- 内存占用较小。

界面稳定后，再换成 LVGL 控件。CanMV K230 固件已经集成 LVGL，可以创建
按钮、标签、状态栏等 GUI 控件；官方示例也展示了显示初始化、LVGL 初始化
和事件界面的基本组织方式。

### 14.6 K230 主循环建议

不要把识别、画面显示、触摸控制和网络请求互相长时间阻塞。建议分成四个
低耦合步骤：

```text
采集摄像头图像
  -> AI 识别并画框
  -> 读取触摸并更新按钮状态
  -> 非阻塞发送识别结果/控制心跳
  -> 刷新 LCD
```

推荐周期：

```text
摄像头：按实际帧率
识别结果上传：5~10Hz
控制心跳：10Hz
KICKPI 状态刷新：2Hz
```

### 14.7 与自主导航的兼容

以后同时存在 Web 控制、K230 屏幕控制和 ROS2 自主导航时，不要让多个节点
同时直接发布同一个 `/cmd_vel`。

建议使用输入优先级：

```text
急停/故障
  > K230 屏幕手动控制
  > Web 手动控制
  > ROS2 自主导航
```

可以使用 `twist_mux`，例如：

```text
/cmd_vel_k230_manual
/cmd_vel_web_manual
/cmd_vel_nav
        |
        v
     twist_mux
        |
        v
      /cmd_vel
        |
        v
 robot_base_bridge
```

第一版如果只使用 K230 屏幕控制，可以暂时让 `robot_gateway` 直接发布
`/cmd_vel`；加入自主导航前再接入 `twist_mux`。

### 14.8 实施顺序

1. 先在 K230 屏幕上显示摄像头画面。
2. 加入触摸坐标打印，确认触摸方向和坐标没有旋转。
3. 画出前后左右停止按钮。
4. 不接电机，只发送假控制请求到 KICKPI。
5. KICKPI 返回 `READY/RUN/FAULT` 状态。
6. 接入 `robot_gateway` 的 `/api/v1/control/cmd_vel`。
7. 用 ROS2 `topic echo /cmd_vel` 检查速度。
8. 电机悬空时测试极小速度。
9. 验证松手停止、网络断开停止和 STM32 串口超时停止。
10. 接入 K230 的 `Cron` 锈病识别模型，确认屏幕上显示 `rust` 检测框。
11. 检查 KICKPI 的 `vision_events` 和 `vision_detections` 表有数据，再设置服务开机启动。

### 14.9 当前确认结果和仍需实机确认的项目

已经从资料确认：

- 采用 CanMV MicroPython 示例接口；
- 显示例程使用 `800x480` 和 `Display.ST7701`；
- 触摸例程使用 `TOUCH(0)`；
- 摄像头显示使用 `PipeLine`，识别使用 `DetectionApp`；
- 实际模型入口和部署配置来自 `D:\cxdownload\end\Cron`。

仍需在实机确认：

- 你的 K230 实物屏幕是否确实是 `800x480 ST7701`；
- 屏幕触摸坐标是否与画面方向一致；
- 摄像头型号和 CanMV 固件是否支持当前 `PipeLine` 配置；
- K230 NPU 对该 `.kmodel` 的实际帧率和内存占用；
- K230 与 `192.168.1.119` 之间的网线、IP 和 Token 是否正确。
