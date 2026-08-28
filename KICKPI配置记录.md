# KICKPI 配置记录

更新时间：2026-08-28

## 1. 已确认的设备信息

- 地址：`192.168.1.119`
- 系统：Ubuntu `22.04.5 LTS`
- 架构：`aarch64/arm64`
- ROS：ROS 2 `Humble`
- 主机名：`kickpi`
- 工作空间：`/root/kickpi_ros2_ws`

登录密码没有写入本文件，也没有写入工程配置。

## 2. 已完成的配置

KICKPI 上已经完成：

1. 确认 `/opt/ros/humble` 已存在。
2. 确认 `colcon` 已安装。
3. 安装 `python3-serial`，版本为 `3.5`。
4. 部署 `robot_base_bridge` ROS2 Python 包。
5. 使用 `colcon build --symlink-install` 构建成功。
6. 确认 `ros2 pkg executables robot_base_bridge` 能发现：

```text
robot_base_bridge base_bridge
```

7. `/root/.bashrc` 已加入：

```bash
source /opt/ros/humble/setup.bash
source /root/kickpi_ros2_ws/install/setup.bash
```

8. 已安装 systemd 单元：

```text
/etc/systemd/system/robot-base-bridge.service
```

该服务目前没有启动，也没有设置为开机启动。

9. 已部署 `robot_gateway` ROS2 Python 包，包含：
   - HTTP API；
   - Web 控制和状态页面；
   - SQLite 遥测、视觉和控制日志；
   - `cmd_vel`、`base/state_cmd` 和 `vision/detections` ROS2 接口。
10. 已安装 systemd 单元：

```text
/etc/systemd/system/robot-gateway.service
```

该服务已经通过健康检查、错误 JSON 检查和视觉事件入库检查，当前仍然保持停止且不
开机启动，避免在 STM32 正式通信模式和底盘实机验证前误控制小车。

## 3. KICKPI 与 STM32 接线

本 KICKPI 的普通 UART5 使用 `/dev/ttyAS5`。40Pin 接口接线：

```text
KICKPI 8脚  UART5_TX  -> STM32 USART1_RX（PA10）
KICKPI 10脚 UART5_RX  <- STM32 USART1_TX（PA9）
KICKPI 6脚  GND       -> STM32 GND
```

双方串口为 3.3V 逻辑。不要把 5V 接到 UART 信号脚。TX 和 RX 必须交叉，
GND 必须连接。

串口参数：

```text
115200 baud
8 data bits
no parity
1 stop bit
no hardware flow control
```

## 4. 启动前提

启动桥接节点前，必须先把 STM32 固件中的：

```c
#define ROBOT_UART1_DEBUG_ONLY 0
```

重新编译并烧录。当前 STM32 如果还是 UART1 文本调试模式，不能启动 ROS2
桥接节点，否则 KICKPI 会发送二进制帧，串口助手会出现乱码或误解析。

另外，INA219 电池检测必须接好，避免底板因为 `FAULT=0x0002` 欠压保护而拒绝
运行电机。

## 5. 手动启动

SSH 登录 KICKPI 后执行：

```bash
cd /root/kickpi_ros2_ws
source /opt/ros/humble/setup.bash
source install/setup.bash
ros2 launch robot_base_bridge base_bridge.launch.py port:=/dev/ttyAS5 baud:=115200
```

看到 `serial opened: /dev/ttyAS5 @ 115200`，表示 Linux 串口已经打开。

检查 ROS 话题：

```bash
ros2 topic list
ros2 topic echo /base/faults
ros2 topic echo /battery_state
ros2 topic echo /odom
```

## 6. 开机自动启动

确认手动启动和底盘通信没有问题后，再执行：

```bash
systemctl daemon-reload
systemctl enable robot-base-bridge.service
systemctl start robot-base-bridge.service
systemctl status robot-base-bridge.service
```

查看日志：

```bash
journalctl -u robot-base-bridge.service -f
```

停止和取消开机启动：

```bash
systemctl stop robot-base-bridge.service
systemctl disable robot-base-bridge.service
```

## 7. 当前状态

当前状态分为两部分：

- `robot_base_bridge`：已部署并构建，服务未启动；等待 STM32 改为正式二进制通信模式。
- `robot_gateway`：已部署、构建并完成本机 HTTP/SQLite 烟雾测试，服务未启动；等待底盘桥接和 K230 实机接入。
- STM32 当前仍需确认正式模式固件已经烧录，且 `ROBOT_UART1_DEBUG_ONLY` 已改为 `0`。
- INA219 仍需接入真实电池电压，否则底盘可能继续报告欠压故障 `FAULT=0x0002`。

启动顺序应为：先 STM32 正式模式，再手动启动 `robot_base_bridge`，确认 ROS2 传感器
话题正常，再启动 `robot_gateway`，最后接入 K230 和浏览器。全部验证通过后才执行
`systemctl enable`。
