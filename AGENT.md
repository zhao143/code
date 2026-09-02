# 机器人项目跟进记录（AGENT）

> 最后整理：2026-09-02  
> 工程根目录：`D:\cxdownload\end\code`  
> 用途：记录已完成工作、真实测试结果、当前配置、风险和下一步。后续每完成一次烧录、接线、实测或代码修改，应同步更新本文件，避免把“已写代码”和“已实测成功”混为一谈。

## 1. 当前目标

完成一个两轮编码电机机器人底盘，并将视觉识别、ROS 2、Web 可视化、传感器数据保存和底盘运动控制整合起来。

最终通信链路：

```text
K230 摄像头/触摸屏
    | 以太网 HTTP（视觉事件、触摸控制）
    v
KICKPI H618（ROS 2 + Web + SQLite）
    | UART5，二进制串口协议
    v
STM32F103C8T6 最小系统板
    | PWM / GPIO / 编码器 / I2C / 单总线
    v
TB6612 电机模块 + 两个编码电机 + 传感器 + 风扇继电器
```

调试链路：

```text
电脑串口工具 <-> STM32 USART1（PA9/PA10，115200）
电脑浏览器 <-> KICKPI Wi-Fi Web 页面
逻辑分析仪 <-> STM32/KICKPI 串口线
```

## 2. 总体状态

| 子系统 | 状态 | 当前结论 |
| --- | --- | --- |
| 底板原理图与 PCB 规划 | [PARTIAL] | 已在嘉立创 EDA 中完成并进行过截图审查、布线和地过孔补强；仍应保存 DRC 结果、Gerber 和最终原理图版本。 |
| STM32 CubeMX/FreeRTOS 工程 | [PARTIAL] | 已加入 USART2 经典蓝牙 SPP 文本协议和 KICKPI/蓝牙单一控制权仲裁；需要用 Keil/EIDE 重新编译、烧录并实测。 |
| STM32 串口调试 | [DONE] | USART1 调试命令、状态遥测和故障输出已实现并用于现场排查。 |
| TB6612 与电机 | [PARTIAL] | 首个 TB6612 模块的 B 路异常已通过更换模块定位并绕过；仍需最终确认两轮正反转、PWM、编码器和闭环控制。 |
| 传感器软件支持 | [DONE] | INA219、DHT30、MPU6050、DS18B20 的驱动和遥测字段已实现。 |
| 传感器实测 | [PARTIAL] | 继电器实测正常；INA219 供电/电压读数、全部传感器的最终接线和读数需要在当前硬件状态下重新确认。 |
| KICKPI ROS 2 网关 | [DONE] | ROS 2 串口桥、HTTP 网关、Web 页面、SQLite 存储代码和部署说明已完成。 |
| KICKPI <-> STM32 实机控制 | [PARTIAL] | 物理串口线、逻辑分析和启动测试已做；应按当前烧录固件确认二进制协议模式并完成 ROS 实际驱动两轮验证。 |
| K230 显示与摄像头 | [DONE] | OV5647 摄像头画面和屏幕显示已确认可用。 |
| K230 锈蚀识别 | [DONE] | 手机上展示的锈蚀图片可被识别，屏幕已显示识别框。 |
| K230 -> KICKPI 视觉事件 -> Web | [DONE] | K230 已输出 `connect success`，KICKPI Web 页面能够显示识别数量。 |
| K230 触摸按钮 | [PARTIAL] | 按住时反复按下/释放的问题已在代码中处理，当前触控基本可用；触摸控制是否已实际驱动底盘尚未完成闭环确认。 |
| K230 上电自动联网 | [PARTIAL] | 已在源码加入启动延时、联网超时和循环重试；需要确认最终 `/sdcard/main.py` 已部署，并做断电重启实测。 |
| STM32 新版固件构建 | [DONE] | 已用 ARM Compiler 5.06u7 完整构建 `STM32F103C8T6` 版本，生成新的 `build/code/code.hex`；尚未替用户烧录。 |
| Web 继电器/蜂鸣器控制 | [DONE] | 网页开关、ROS2 `/base/output_cmd`、串口桥 `CMD_SET_OUTPUT` 和 STM32 输出逻辑已打通；实际 PB0/PB1 动作仍需烧录新版 HEX 后现场确认。 |

状态含义：

- `[DONE]`：已经完成，并至少有一次实际测试或明确的运行结果。
- `[PARTIAL]`：已设计、已编码或局部测试，但仍需要完整实机确认。
- `[TODO]`：尚未开始或尚未验证。

## 3. 已完成工作记录

### 3.1 需求与系统方案

已确认的系统功能：

- KICKPI H618 作为 ROS 2、Web 后端、数据库和主机通信节点。
- STM32F103C8T6 最小系统板负责实时电机、编码器、传感器和安全逻辑。
- 两个带编码器的直流电机，使用 TB6612FNG 模块驱动。
- 采集电池电压、环境温湿度、电池温度、姿态数据。
- 使用继电器控制 5V 风扇。
- K230 负责摄像头、屏幕、锈蚀识别和触摸交互。
- K230 不直接接入 STM32 底板；K230 经以太网请求 KICKPI，再由 KICKPI 经 UART 控制 STM32。
- 已去掉 Type-C 和 CH340 USB 串口方案。
- 当前不使用按键。

已讨论并形成的关键原则：

- 电机启动和堵转电流可能远大于普通 2.54 mm 排针和 3A 级 INA219 模块的能力。
- INA219 若只用于检测电池电压，不能让主电机大电流长期通过其小电流测量路径。
- 主电源大电流线应与信号线、传感器线分开走。
- 底板 GND 需要完整地平面，并使用合适的 GND 缝合过孔改善回流路径。

### 3.2 嘉立创 EDA 原理图和 PCB

已做事项：

- 学习并连接过嘉立创 EDA 相关 API/Bridge。
- 在原理图中按功能块重新布局过器件，减少跨页和长连线。
- 原理图功能块包括：STM32 最小系统板、TB6612、5V/3.3V 电源、INA219、DHT30、MPU6050、DS18B20、蜂鸣器、继电器和风扇接口。
- PCB 板框按 `99 mm x 99 mm` 规划。
- 已基于截图检查器件放置、固定孔避让、板框边界、布线和 GND 缝合过孔。
- 用户已在 PCB 上完成布线，并补充、移动过部分 GND 缝合过孔。
- 已讨论 2.54 mm 接线座和电源线径限制。

PCB 仍需保留/确认的事项：

- 在嘉立创 EDA 执行最终 DRC，确认无未连接网络、短路、孔径、丝印压焊盘、铜皮孤岛等错误。
- 导出并保存最终原理图 PDF、PCB PDF、Gerber、BOM、坐标文件和 DRC 报告。
- 电机 12V 输入、5V 总输入和 GND 的实际线宽、过孔数量、连接器电流能力需要按最终电机堵转电流复核。
- 若下一版 PCB 仍承担主电机电流，建议改用更大电流连接器或螺丝端子，避免仅依赖 2.54 mm 单针。

### 3.3 电源与接线方案

已确认或采用的结构：

```text
电池
  -> 18AWG 主线
  -> 12V 降压/供电模块
  -> 12V 电机系统
  -> 5V 总电源
  -> KICKPI / K230 / STM32 最小系统板 / 继电器等
```

- STM32 最小系统板带板载 `XC6204B332MR` 3.3V 稳压器，可从 `5V` 引脚供电；不要把外部 3.3V 与板载稳压输出硬并联。
- 风扇是 5V 风扇，使用 5V 电源和继电器开关控制。
- 电池至降压模块已采用 18AWG。
- 后续用户已接好 0.75 平方毫米导线作为板端电源接线。
- 16AWG/18AWG 适合外部主干；板端 2.54 mm 小孔/小插座附近可使用很短的 `0.5 mm2` 或 `0.33 mm2` 过渡尾线，但这不提升 2.54 mm 接头本身的额定电流。
- AWM 2468 26AWG 不适合作为电机主电源或大电流 5V/12V 主干，只适合低电流信号或小负载。

重要硬件风险：

- 2.54 mm、标称 3A 的单针排针不应承担电机启动/堵转主电流。
- INA219 常见模块的分流电阻和走线不适合直接串在高堵转电流电机总电源上。
- 任何电机测试均应先让车轮悬空，并准备可快速断电的电源开关或拔插点。

### 3.4 STM32 CubeMX 配置

工程文件：`D:\cxdownload\end\code\code.ioc`

当前 MCU：`STM32F103C8T6`

已确认配置：

- 系统时钟：HSE，72 MHz。
- FreeRTOS：CMSIS-RTOS V2。
- HAL 时间基准：`TIM3`。
- 调试串口：`USART1`，`PA9/PA10`，115200 bps。
- I2C 总线：`I2C2`，`PB10=SCL`，`PB11=SDA`。
- 电机 A PWM：`TIM1_CH1`，`PA8`。
- 电机 B PWM：`TIM1_CH4`，`PA11`。
- 编码器 A：`TIM2`，`PA0/PA1`，`Encoder Mode = TI1 and TI2`。
- 编码器 B：`TIM4`，`PB6/PB7`，`Encoder Mode = TI1 and TI2`。
- TB6612 待机：`PA6`。
- TB6612 A 方向：`PB12/PB13`。
- TB6612 B 方向：`PB14/PB15`。
- DS18B20：`PA4`。
- 继电器：`PB1`。
- 蜂鸣器：`PB0`。
- 状态 LED：`PC13`。

定时器分配结论：

- `TIM1`：电机 PWM。
- `TIM2`：编码器 A。
- `TIM3`：HAL 时间基准，不能再当作普通 FreeRTOS 定时器随意复用。
- `TIM4`：编码器 B。
- FreeRTOS 不需要单独占用一个普通通用定时器，系统节拍由 SysTick/RTOS 配置负责；当前资源分配可用。

### 3.5 STM32 固件

主要源码位置：

```text
D:\cxdownload\end\code\Core\Inc\robot_config.h
D:\cxdownload\end\code\Core\Src\robot_app.c
D:\cxdownload\end\code\Core\Src\robot_comm.c
D:\cxdownload\end\code\Core\Src\robot_motor.c
D:\cxdownload\end\code\Core\Src\robot_encoder.c
D:\cxdownload\end\code\Core\Src\robot_sensors.c
D:\cxdownload\end\code\Core\Src\robot_delay.c
```

已实现功能：

- FreeRTOS 任务划分：控制、电机、传感器、串口通信、遥测等。
- 电机 PWM、方向、待机控制。
- 双路编码器读取、速度计算和里程计支持。
- UART 文本调试命令。
- UART 二进制帧协议支持，供 KICKPI ROS 2 串口桥使用。
- INA219、DHT30、MPU6050、DS18B20 数据读取与故障处理。
- 电池低压、过温、通信超时等状态/故障管理。
- 周期性遥测输出。
- 源码中已补充详细中文注释。
- 自动电机测试逻辑用于定位 A/B 路、电机接线和驱动器问题。

当前 `robot_config.h` 的关键运行开关：

```c
#define ROBOT_UART1_DEBUG_ONLY            0
#define ROBOT_RELAY_ENABLE                0
#define ROBOT_BLUETOOTH_ENABLE            1
#define ROBOT_BUZZER_ENABLE               0
#define ROBOT_RELAY_TEST_ENABLE           0
#define ROBOT_MOTOR_AUTO_TEST_ENABLE      0
#define ROBOT_BLUETOOTH_OWNER_TIMEOUT_MS  800U
```

这表示当前源码偏向“正式 KICKPI + 蓝牙控制”：

- 蜂鸣器、继电器仍按当前要求禁用。
- `ROBOT_UART1_DEBUG_ONLY = 0` 时，USART1 为 KICKPI 二进制协议，USART2 为蓝牙文本协议。
- `ROBOT_MOTOR_AUTO_TEST_ENABLE = 0`，上电不会自动测试电机。
- 蓝牙运动命令连续超过 800ms 没有刷新会自动停车并释放控制权。
- 不能只根据 `code_kickpi.hex` 的文件名判断固件模式，必须重新编译、烧录并记录实际版本。

现有构建产物：

```text
D:\cxdownload\end\code\build\code_debug.hex
D:\cxdownload\end\code\build\code_kickpi.hex
D:\cxdownload\end\code\code_kickpi.hex
```

### 3.6 电机故障排查记录

已发生并确认的现象：

1. 发送 `pwm 100 100` 时，接在 `AO1/AO2` 的 A 路电机转动，`BO1/BO2` 的 B 路不转。
2. 交换两台电机后，仍是 A 路能转、B 路不转，因此问题不在电机本体。
3. 已建议并使用逻辑分析仪检查 STM32 到 TB6612 的 B 路 PWM、BIN1、BIN2、STBY 信号。
4. 用户更换另一块购买的 TB6612 模块后，模块可以使用。

当前判断：

- 原 TB6612 模块的 B 路或相关接线存在异常，优先怀疑模块损坏/虚焊/通道故障。
- 更换模块后应重新完成以下验证，才可进入装车阶段：
  - A 电机正转、反转、停止。
  - B 电机正转、反转、停止。
  - `pwm 0 0` 能使两路停止。
  - `speed` 闭环命令能生效。
  - 两个编码器计数方向和脉冲数正确。
  - 车轮悬空时先测试，再落地低速测试。

### 3.7 传感器、继电器和蜂鸣器

设计连接：

| 器件 | 接口/地址 | 说明 |
| --- | --- | --- |
| INA219 | I2C2，`0x40` | 当前定位为电池电压监测，不承担电机主电流串联测量。 |
| DHT30 | I2C2，`0x38` | 环境温湿度。 |
| MPU6050 模块 | I2C2，常用 `0x68` | 模块已有上拉电阻，I2C 总线不应重复堆叠过强上拉。 |
| DS18B20 | `PA4` | 电池温度。 |
| 继电器 | `PB1` | 控制 5V 风扇。 |
| 蜂鸣器 | `PB0` | 报警输出。 |

实测记录：

- 继电器已按每 5 秒跳变的测试方式验证，用户确认继电器正常。
- 蜂鸣器硬件确认可用，但因噪声问题目前在软件中禁用。
- INA219 早期未供电；后续电源线已接好，但必须以当前实物重新读取并确认电池电压数值。
- DHT30、MPU6050、DS18B20 的代码路径已经实现；最终上车前需要记录一次稳定读数和异常断线行为。

INA219 电压监测的接线原则：

```text
电池正极 BAT+
  -> INA219 VIN+
  -> INA219 VIN-
  -> 被监测的正电源节点/电压采样节点

电池负极 BAT-
  -> 系统 GND
  -> INA219 GND

INA219 VCC -> 3.3V
INA219 SCL -> PB10
INA219 SDA -> PB11
```

注意：若电机主电流通过 INA219 的 VIN+/VIN-，必须确认模块分流电阻、铜箔和额定电流可承受堵转电流；当前方案不应默认其可以承受。

### 3.8 KICKPI ROS 2、Web 和数据库

KICKPI 硬件：

- 全志 H618。
- 2 GB RAM、8 GB eMMC、64 GB SD 卡。
- 系统环境：Ubuntu 22.04.5 + ROS 2 Humble。

工程位置：

```text
D:\cxdownload\end\code\kickpi_ros2_ws
```

已实现软件：

| 包/组件 | 作用 |
| --- | --- |
| `robot_base_bridge` | KICKPI 与 STM32 串口桥；接收控制命令，发布里程计、传感器和故障信息。 |
| `robot_gateway` | HTTP API、Web 页面、SQLite 数据库、控制审计、视觉事件接收。 |
| `robot-base-bridge.service` | ROS 2 串口桥 systemd 服务。 |
| `robot-gateway.service` | Web/后端 systemd 服务。 |

ROS 2 关键接口：

```text
订阅：
/cmd_vel
/base/state_cmd

发布：
/odom
/battery_state
/imu/raw
/env/temperature
/env/humidity
/battery/temperature
/base/faults
```

Web/HTTP 关键接口：

```text
GET  /api/v1/health
GET  /api/v1/latest
GET  /api/v1/telemetry
GET  /api/v1/vision/events
POST /api/v1/control/cmd_vel
POST /api/v1/control/state
POST /api/v1/vision/events
```

SQLite 数据库：

```text
/root/robot_data/robot.db
```

主要数据表：

```text
telemetry
vision_events
vision_detections
control_events
```

KICKPI <-> STM32 串口：

- KICKPI 使用 `UART5`，设备节点预期为 `/dev/ttyAS5`。
- 连接必须交叉：`KICKPI TX -> STM32 RX`，`KICKPI RX -> STM32 TX`，两端 `GND -> GND`。
- 逻辑分析仪已用于观察 KICKPI 和底板的串口数据。
- KICKPI 与 STM32 的正式 ROS 2 控制需要 STM32 切换到二进制协议模式后再完整验证。

服务维护命令（在 KICKPI 执行）：

```bash
sudo systemctl status robot-base-bridge.service --no-pager
sudo systemctl status robot-gateway.service --no-pager
sudo systemctl restart robot-base-bridge.service
sudo systemctl restart robot-gateway.service
journalctl -u robot-base-bridge.service -f
journalctl -u robot-gateway.service -f
```

ROS 2 调试命令：

```bash
source /opt/ros/humble/setup.bash
source /root/kickpi_ros2_ws/install/setup.bash
ros2 topic list
ros2 topic echo /base/faults
ros2 topic echo /battery_state
ros2 topic pub --once /cmd_vel geometry_msgs/msg/Twist \
  "{linear: {x: 0.0}, angular: {z: 0.0}}"
```

安全规则：

- 任何新连接或新固件的第一条 ROS 命令必须是速度为 0 的停止命令。
- Web/API 的默认开发令牌只适合局域网调试；正式使用前必须更换。
- 不在本文件、代码仓库、截图或日志中保存 SSH 密码、Wi-Fi 密码和真实访问令牌。

### 3.9 K230 视觉、屏幕和触摸

K230 资料和模型位置：

```text
D:\cxdownload\end\Cron
D:\cxdownload\end\Cron\mp_deployment_source
```

模型文件：

```text
best_AnchorBaseDet_can2_5_s_20260827131049.kmodel
deploy_config.json
```

模型配置摘要：

- 检测类别：`rust`。
- 类别编号：`0`。
- 输入尺寸：`320 x 320`。
- 模型类型：AnchorBaseDet。
- 置信度阈值：`0.4`。
- NMS 阈值：`0.5`。

K230 应用源码：

```text
D:\cxdownload\end\code\k230_canmv_apps\robot_touch_drive.py
```

K230 运行环境：

- CanMV K230 v3p0。
- 摄像头：OV5647 CSI2，已识别到 `1920x1080@30`。
- 自动启动入口：`/sdcard/main.py`。
- 常用部署目录：`/data/Cron/` 或 `/sdcard/`。

已完成的 K230 修复：

- 处理模型路径不一致导致的 `ENOENT`：支持从 `/data/Cron/mp_deployment_source`、`/data/Cron`、`/sdcard/mp_deployment_source`、`/sdcard/Cron` 查找模型。
- 处理 `sys.print_exception` 在当前 CanMV 环境不存在导致的二次崩溃，改为兼容的异常输出方式。
- 处理触摸按钮按住时反复触发的问题：增加释放保持时间，忽略不必要的重复事件。
- 为 KICKPI 以太网连接增加启动等待、连接超时和周期性重试：
  - `BOOT_NETWORK_DELAY_MS = 3000`
  - `LAN_READY_TIMEOUT_MS = 15000`
  - `NETWORK_RETRY_MS = 3000`

已验证结果：

- K230 屏幕可以显示摄像头画面。
- 锈蚀检测可识别手机展示的锈蚀图片。
- 屏幕显示锈蚀检测框。
- K230 输出过 `connect success`。
- KICKPI Web 页面可显示识别数量，说明视觉事件 HTTP 上传链路已经跑通。

K230 <-> KICKPI 专用以太网：

```text
KICKPI Ethernet: 192.168.2.2/24
K230 Ethernet:   192.168.2.3/24
K230 网关/DNS:   192.168.2.2
```

同时的热点网络记录：

```text
电脑热点侧 IP：172.19.73.79
KICKPI Wi-Fi IP：172.19.73.173
```

注意：IP 会随热点/DHCP 变化。`192.168.2.2/24` 与 `192.168.2.3/24` 是 K230 与 KICKPI 直连网段，Web 从电脑访问时通常应使用 KICKPI 当前 Wi-Fi IP。

K230 当前需继续确认：

- 断电重启后，`/sdcard/main.py` 是否自动成功联网。
- 自动启动版和手动运行版是否为同一份脚本。
- 若开机仍显示无网络，先检查自动启动脚本是否已包含“启动延时 + 重试”版本。
- 触摸按钮是否可以经 KICKPI 最终驱动 STM32 两轮运动。

### 3.10 最新 K230 自动联网问题

现象：

- K230 上电自动运行 `main.py` 时显示无网络。
- 手动运行同类脚本时网络正常。

已完成修改：

- 在源码 `D:\cxdownload\end\code\k230_canmv_apps\robot_touch_drive.py` 中加入联网启动等待、超时和重试逻辑。
- 本机 CanMV 远程缓存中的 `/sdcard/main.py` 也已出现上述网络参数和重试逻辑。

还需要实机确认：

1. 确认真正由 K230 自动启动的文件就是修改后的 `/sdcard/main.py`。
2. K230 断电、等待至少 10 秒后重新上电。
3. 观察是否先完成网络初始化，再输出 `connect success`。
4. 在 KICKPI 上确认 Web 页面继续收到视觉事件。

## 4. 当前推荐的测试顺序

### P0：先保证人身与硬件安全

1. 电机测试时将车轮悬空。
2. 使用限流电源或准备可快速断开的电源。
3. 检查 12V、5V、GND 极性。
4. 确认 KICKPI、K230、STM32 三者公共地关系正确。
5. 不让主电机电流长期通过小排针或未确认额定电流的 INA219 模块。

### P1：完成 STM32 底板独立验证

1. 当前先烧录/使用调试版固件。
2. 通过 USART1 发送 `status`，确认无异常故障码。
3. 逐个检查 DHT30、MPU6050、DS18B20、INA219 的遥测值。
4. 在车轮悬空状态下，分别测试 A/B 两路：
   - 正转。
   - 反转。
   - 低 PWM。
   - 停止。
5. 验证两个编码器的计数变化和方向。
6. 继电器与蜂鸣器按当前需求保持禁用，不作为本阶段阻塞项。

### P2：完成 KICKPI <-> STM32 串口和 ROS 2 验证

1. 修改 STM32 配置：

```c
ROBOT_UART1_DEBUG_ONLY = 0
ROBOT_MOTOR_AUTO_TEST_ENABLE = 0
```

2. 重新编译、烧录，并明确记录烧录的 `.hex` 文件和编译时间。
3. 启动 KICKPI `robot-base-bridge`。
4. 先发送 0 速度停止命令。
5. 检查 ROS 2 是否持续收到 `/battery_state`、`/base/faults` 等话题。
6. 使用极低速度短时测试 A/B 两轮。
7. 验证失联超时后 STM32 自动停止。

### P3：完成 K230 联动验证

1. 先完成 K230 自动联网重启测试。
2. 确认视觉识别事件稳定出现在 Web 页面和 SQLite 数据库。
3. 在车轮悬空时，触摸 K230 前进/后退/左转/右转按钮。
4. 检查 K230 -> HTTP -> KICKPI -> ROS 2 -> UART -> STM32 的完整链路。
5. 松手、失焦、断网和服务重启时，都必须使底盘停止。

### P4：进入装车和闭环控制

1. 固定电池、降压模块、主控、K230 和线束。
2. 调整两侧电机方向和编码器方向宏。
3. 测量轮径、轮距、编码器每圈计数。
4. 再启用速度 PID 和里程计标定。
5. 增加低压、过温、传感器失联和串口断链的实际故障演练。

## 5. 未完成事项清单

### 硬件

- [TODO] 复核 INA219 是否只用于安全的电压测量路径，而不是承受电机主电流。
- [TODO] 记录电池、12V、5V 实测电压和压降。
- [TODO] 实测电机启动和堵转/近堵转电流，据此确定保险丝、连接器和线径是否足够。
- [TODO] 完整确认全部传感器在最终供电和接线下的读数。
- [TODO] 最终执行嘉立创 EDA DRC，并保存制造文件。

### STM32

- [TODO] 确认实际烧录的固件是调试模式还是 KICKPI 二进制协议模式。
- [TODO] 更换正常 TB6612 后重新验证 A/B 双电机正反转、停止和编码器。
- [TODO] 完成速度闭环 PID 参数调试。
- [TODO] 完成电池低压、温度过高、通信超时的故障演练。

### KICKPI

- [TODO] 确认 systemd 服务是否已设置开机自启，而不是只在测试时手动运行。
- [TODO] 验证 `/dev/ttyAS5` 权限、设备节点和串口波特率。
- [TODO] 确认 ROS 2 到 STM32 的正式二进制帧闭环控制。
- [TODO] 修改默认 API 令牌，并限制 Web 服务暴露范围。
- [TODO] 为数据库增加备份/导出策略。

### K230

- [TODO] 部署并确认最终自动启动的 `/sdcard/main.py`。
- [TODO] 断电重启验证自动联网、视觉识别和 `connect success`。
- [TODO] 验证触摸屏按键到电机实际运动的端到端控制。
- [TODO] 根据现场效果调整模型置信度阈值、NMS 阈值和屏幕显示性能。

## 6. 重要文件索引

### 底板、STM32 和总体文档

```text
D:\cxdownload\end\code\AGENT.md
D:\cxdownload\end\code\README_机器人底盘代码说明.md
D:\cxdownload\end\code\机器人项目总进度与完整实施流程.md
D:\cxdownload\end\code\KICKPI配置记录.md
D:\cxdownload\end\code\系统整体通信与Web数据库实现方案.md
D:\cxdownload\end\code\K230屏幕控制_视觉_通信部署说明.md
D:\cxdownload\end\code\蓝牙与多控制端控制说明.md
D:\cxdownload\end\code\code.ioc
D:\cxdownload\end\code\code_kickpi.hex
```

### K230

```text
D:\cxdownload\end\code\k230_canmv_apps\robot_touch_drive.py
D:\cxdownload\end\Cron\mp_deployment_source\deploy_config.json
D:\cxdownload\end\Cron\mp_deployment_source\best_AnchorBaseDet_can2_5_s_20260827131049.kmodel
```

### 嘉立创 EDA 与生产资料

```text
D:\project\car\生产文档\KICKPI_H618_ROS2_两轮编码底盘全功能实现路线.md
D:\project\car\生产文档\底板保护滤波补充清单.md
D:\project\car\生产文档\串联并联保护滤波速查图.png
D:\project\car\生产文档\ModuleBaseV2_模块化底板设计记录.md
```

## 7. 后续更新格式

每次实测后，在本文件顶部或本节追加一条记录：

```text
日期：
操作：
使用的硬件/固件版本：
观察到的现象：
结果：[DONE] / [PARTIAL] / [FAILED]
下一步：
```

示例：

```text
日期：2026-08-31
操作：K230 断电重启并运行 /sdcard/main.py。
使用的硬件/固件版本：K230 CanMV，main.py 含网络重试逻辑。
观察到的现象：启动后 3 秒开始联网，输出 connect success，Web 收到视觉事件。
结果：[DONE]
下一步：测试触摸前进按钮到两轮电机的完整链路。
```

## 8. 当前最优先下一步

### 8.1 架构变更：切换到 STM32F405RGT6

日期：2026-08-31  
决定：后续底盘控制 MCU 从 `STM32F103C8T6` 迁移到 `STM32F405RGT6`。

结论：

- 这是新建 F405 最小系统/新 PCB 的迁移，不是对当前 F103C8T6 最小系统板的直接替换。
- 当前 F103 工程和已经完成的底板测试必须保留，作为 F405 迁移的功能对照基线。
- 可以保留大部分外部网络名：USART1、I2C2、TIM1 PWM、TIM2/TIM4 编码器、TB6612 GPIO、DS18B20、继电器、蜂鸣器和 PC13 状态灯。
- 必须重新设计 F405 的 3.3V 供电、双 VCAP 电容、PH0/PH1 晶振、BOOT0、NRST、SWD 和 LQFP64 封装。

详细设计和 CubeMX 迁移步骤见：

```text
D:\cxdownload\end\code\STM32F405RGT6_迁移方案.md
```

### 8.2 最新优先顺序

1. 按 `STM32F405RGT6_迁移方案.md` 完成 F405 最小系统原理图，先审核 3.3V 供电、VCAP、晶振和 SWD。
2. 不并联两个 AMS1117；先确认现有 AMS1117 的 3.3V 总负载和温升，优先评估 500mA 至 1A 的单路 3.3V 电源。
3. 打开嘉立创 EDA 并加载 Bridge 扩展，使 `http://127.0.0.1:49620/eda-windows` 出现已连接窗口。
4. 新建独立 F405 CubeMX 工程，完成 LED、SWD、串口、I2C、PWM 和编码器的板级点亮。
5. 迁移并验证 STM32 底盘业务代码，再完成双电机、双编码器和四类传感器的最终实测。
6. 切换 STM32 到 KICKPI 二进制协议模式，完成 ROS 2 实际驱动两轮测试。
7. 部署并验证 K230 自启动联网，最后测试 K230 触摸按钮控制底盘，并保留通信失联自动停止保护。

## 9. 2026-08-31 当前会话更新

- [DONE] 已启动嘉立创 EDA Bridge，端口为 `49620`。
- [DONE] 嘉立创 EDA 扩展已连接，当前检测到 1 个活动 EDA 窗口。
- [PARTIAL] 尚未在本次会话中执行 F405 原理图或 PCB 修改；下一步应先确认活动窗口中的工程、原理图页和 PCB 页。
- [PARTIAL] F405 的电源方案已确定方向：不并联两个 AMS1117；先核对现有 AMS1117 的输入电压、3.3V 总负载和温升，优先考虑单路 500mA 至 1A 的 3.3V 电源。

## 10. 2026-09-02 蓝牙控制与 Android App 更新

本次工作基于当前仍在使用的 `STM32F103` Keil/RVDS 工程完成。它不是
`STM32F405RGT6` 迁移工程；F405 迁移仍需单独建立 CubeMX 工程并重新验证外设映射。

### STM32 固件

- [DONE] 新增 `Core/Inc/robot_bluetooth.h` 和 `Core/Src/robot_bluetooth.c`。
- [DONE] USART2 作为经典蓝牙串口，默认参数为 `9600 8N1`，支持 HC-05、HC-06
  等 SPP 透传模块。
- [DONE] 蓝牙接收采用中断收字节、环形缓冲、控制任务解析，避免在中断里执行
  电机控制和复杂字符串处理。
- [DONE] 增加蓝牙文本命令：`help`、`status`、`enable`、`stop`、`clear`、
  `clear_fault`、`estop`、`pwm A B`、`speed A B`、`forward`、`back`、
  `left`、`right`。
- [DONE] 增加三类运动控制来源仲裁：`KICKPI`、`BLUETOOTH`、`UART1_DEBUG`。
  同一时间只有一个来源可以占用运动控制权；其他来源收到运动命令时返回
  `BUSY owner=...`。
- [DONE] 蓝牙运动命令使用 800ms 租约超时，手机停止刷新命令后底盘自动停车；
  `stop` 为全局停车并释放控制权，`estop` 停车并锁存故障。
- [DONE] 状态输出增加 `OWNER=...`；正式二进制状态帧在原有数据后追加 1 字节
  控制来源，旧版 KICKPI 解析仍可按旧长度工作。
- [DONE] `Core/Src/main.c` 已在 `ROBOT_BLUETOOTH_ENABLE` 开启时初始化 USART2。
- [DONE] `Core/Src/robot_comm.c` 已增加 USART2 接收中断重装和蓝牙接收处理。
- [DONE] `.eide/eide.yml` 已加入 `Core/Src/robot_bluetooth.c`，Keil 工程也已加入
  该源文件。

### KICKPI

- [DONE] `robot_base_bridge/base_bridge_node.py` 已兼容旧版 56 字节状态帧和新版
  带控制来源字节的状态帧。
- [DONE] 新增 ROS 话题 `base/control_owner`，用于显示当前运动控制来源。
- [DONE] 已通过 `python -m py_compile` 语法检查。

### Android App

- [DONE] 空白项目 `D:/cxdownload/end/code/Car_Control` 已加入经典蓝牙 SPP 管理器。
- [DONE] `BluetoothSerialManager.kt` 支持配对设备列表、连接、断开停车、发送命令、
  接收底盘文本日志和按住方向键周期刷新。
- [DONE] `MainActivity.kt` 已完成蓝牙权限、设备连接、速度滑块、前后左右/停止、
  使能、清故障、状态读取和通信日志界面。
- [DONE] Android 12 及以上申请 `BLUETOOTH_CONNECT`，不使用蓝牙扫描，因此不额外
  申请定位权限。
- [DONE] 已修复快速重连时旧连接清理任务误关闭新连接的并发问题。

### 本次验证与待办

- [DONE] `robot_bluetooth.c`、`robot_comm.c` 已用 ARM GCC 做独立语法检查。
- [DONE] KICKPI Python 节点语法检查通过。
- [PARTIAL] STM32 全量构建尚未在本机完成：当前工程使用 Keil/RVDS 的
  FreeRTOS `ARM_CM3` 端口，ARM GCC 不能直接编译其中的 ARMCC 汇编语法。
  应使用 Keil MDK 或 EIDE 的 AC5 工具链生成最终 HEX。
- [PARTIAL] Android Gradle 尚未进入 Kotlin 编译阶段，本机 Gradle 在启动单次守护进程
  时报告 `Unable to establish loopback connection`。源码已写入，需在 Android Studio
  中打开项目并执行 `assembleDebug`；该问题属于本机 Java/Gradle 环境，不是已报告的
  Kotlin 源码错误。
- [TODO] 烧录包含蓝牙功能的 STM32 HEX，给 HC-05/HC-06 配对后用蓝牙助手执行
  `status`、`enable`、`pwm 100 100`、`stop`。
- [TODO] 将 `Car_Control` 构建出的 APK 安装到手机，验证按住按钮、松手停车和
  蓝牙失联自动停车。
- [TODO] 现场确认三路控制源仲裁：蓝牙占用时 KICKPI/K230 运动命令应收到 BUSY，
  蓝牙停止后其他控制源才能重新接管。

## 11. 2026-09-02 卡死排查与 K230 视频链路更新

### STM32 卡死原因与修复

- [DONE] 定位到 `Core/Src/robot_app.c` 的高风险错误：`ctx_lock()` 原来只等待
  20 ms，却忽略 `osMutexAcquire()` 返回值，超时后仍然调用 `osMutexRelease()`。
  这可能导致非互斥量持有者释放锁，触发 FreeRTOS 断言或破坏互斥量状态，
  现象就是 PC13 停止闪烁、遥测停止和任务像卡死一样。
- [DONE] `ctx_lock()` 已改为必须真正获得互斥量后才返回，等待期间让出 CPU。
  控制临界区不包含 I2C 和 UART 阻塞发送，因此正常情况下不会造成新的长阻塞。
- [DONE] `FreeRTOSConfig.h` 已打开 `configCHECK_FOR_STACK_OVERFLOW=2` 和
  `configUSE_MALLOC_FAILED_HOOK=1`，并将 `configASSERT` 接到统一故障处理。
- [DONE] `freertos.c` 已增加任务创建检查、堆栈溢出钩子、heap 申请失败钩子和
  RTOS 断言故障处理。异常时先调用 `RobotMotor_Stop()`、关闭 TB6612 STBY，
  再让 PC13 进入故障闪烁。
- [DONE] `stm32f1xx_it.c` 的 HardFault、MemManage、BusFault、UsageFault 已统一
  关闭 TB6612 STBY 并用 LED 指示，便于区分普通状态闪烁和 CPU 异常。
- [DONE] UART1 和蓝牙接收解析已增加单次调用字节预算：UART1 每个控制周期最多
  处理 64 字节，蓝牙最多处理 32 字节，避免 KICKPI 或蓝牙持续发送运动保持包时
  长期占用高优先级控制任务，导致 PC13 心跳和遥测任务被饿死。
- [DONE] `RobotComm_SendFrame()` 增加了非零长度 payload 的空指针检查，避免异常
  参数导致内存访问错误。
- [DONE] 增加可选 IWDG 接口：`robot_config.h` 中 `ROBOT_IWDG_ENABLE` 默认保持
  为 0；在 CubeMX 生成 IWDG 后改为 1，启动时初始化 IWDG，并只由 10ms 控制
  任务刷新。控制任务卡死、互斥量死锁或调度异常时，芯片会自动复位。
- [PARTIAL] 当前源码工程仍是 `STM32F103C8T6` 的 Keil/RVDS 工程；本机只有
  ARM GCC，不能直接编译工程自带 RVDS `portmacro.h`。最终 HEX 必须用 Keil MDK
  或 EIDE 的 AC5 工具链重新生成，不能只用本机 GCC 的失败结果判断固件。

### LED 与遥测现象

- `RUN` 状态下 LED 使用 200 ms 周期，`IDLE/READY` 使用 500 ms 周期，所以点击
  运动后变快是程序设计的运行状态提示。
- 如果修复后的版本仍然“彻底不闪”，应观察它是持续慢闪、快速闪，还是完全熄灭：
  持续故障闪烁通常意味着 RTOS 断言/堆栈/heap 异常；完全停止且没有故障闪烁，
  再重点查复位、电源、SWD 供电和 HardFault 现场。

### K230 与 Web 视频

- [DONE] `robot_touch_drive.py` 已将当前联调阶段的 `KICKPI_LINK_ENABLE` 改为
  `True`，K230 静态地址仍为 `192.168.2.3`，KICKPI 有线端仍为 `192.168.2.2`，
  HTTP API 端口为 `8080`。
- [DONE] 已加入可选 `VIDEO_RTSP_ENABLE` 和 `WBCRtsp` 启动/停止代码。打开后
  K230 预计提供 `rtsp://192.168.2.3:8554/test`。
- [PARTIAL] 现有 KICKPI Web 页面只显示遥测和识别 JSON，没有 RTSP 转 HLS、
  WebRTC 或 MJPEG 的代理；普通 Chrome/Edge 不能直接播放 RTSP。因此视频要在
  Web 页面显示，还需要在 KICKPI 增加媒体转换服务，并在 HTML 中使用浏览器支持
  的 HLS/WebRTC/MJPEG 地址。
- [DONE] 2026-09-02 通过 KICKPI `10.81.123.173` 实测：`eth0=192.168.2.2`、
  `wlan0=10.81.123.173`；KICKPI 连接 `192.168.2.3:8554` 成功，并返回
  `RTSP/1.0 200 OK`。KICKPI 自身的 `8080` Web/API、ROS2 两个 systemd 服务
  均处于运行状态；OpenCV 已安装并同时支持 FFmpeg/GStreamer。
- [DONE] `robot_gateway` 增加可选 OpenCV 视频线程和 `/video.mjpg` 浏览器流：
  K230 RTSP -> KICKPI OpenCV -> multipart MJPEG -> Web `<img>`。默认视频参数
  现在 launch 默认开启，也可以用 `video_enabled:=false` 关闭；默认地址为
  `rtsp://192.168.2.3:8554/test`，8 FPS、宽度 640、JPEG 质量 75。
- [DONE] 已将更新后的 `robot_gateway` 源码复制到 KICKPI，并完成
  `colcon build --packages-select robot_gateway --symlink-install` 和网关重启。
  实测 `/video.mjpg?token=robot-dev-token` 返回 `200`，5 秒抓取约 1.1 MB，
  抽取出的 JPEG 画面正常显示 K230 摄像头和触控界面；底盘桥接服务仍为
  `active`，Web/API 最新数据正常更新。

## 12. 2026-09-02 编译修复、输出控制和图像方向修复

### STM32 固件

- [DONE] 删除 `freertos.c` 中 CubeMX 生成的空版 `vApplicationStackOverflowHook`，
  保留带安全停车和故障 LED 的正式版本，解决 `#247 function ... already been defined`。
- [DONE] `stm32f1xx_it.c` 的 HardFault、MemManage、BusFault、UsageFault 已真正
  调用 `RobotCpuFaultLoop()`，不再停在没有 LED 指示的空 `while(1)`。
- [DONE] 正式模式下不使用的调试回调改为条件编译，清除了 `s_debug_line_handler`
  未使用警告。
- [DONE] `build/code/builder.params` 补齐 `iwdg.c`、`robot_bluetooth.c` 和
  `stm32f1xx_hal_iwdg.c`，解决链接阶段缺少 IWDG/蓝牙实现的问题。
- [DONE] `robot_config.h` 当前为：`ROBOT_UART1_DEBUG_ONLY=0`、
  `ROBOT_RELAY_ENABLE=1`、`ROBOT_BUZZER_ENABLE=1`、
  `ROBOT_BUZZER_FAULT_ALARM_ENABLE=0`、`ROBOT_IWDG_ENABLE=1`。
  这表示网页可以手动控制继电器和蜂鸣器，但故障不会自动鸣叫；上电默认关闭。
- [DONE] 使用 ARM Compiler 5.06u7 完整重建成功：
  ROM 36.60 KB/64 KB，RAM 14.07 KB/20 KB。新文件为：
  `D:\cxdownload\end\code\build\code\code.hex`。
  该 HEX 仍是 STM32F103C8T6 工程，不能烧录到 STM32F405RGT6。

### KICKPI Web 输出控制

- [DONE] 网页新增“风扇继电器”和“蜂鸣器”开关。
- [DONE] 新增 HTTP 接口 `POST /api/v1/control/output`，请求体示例：
  `{"relay_on":false,"buzzer_on":false}`。
- [DONE] 网关发布 `base/output_cmd`；串口桥把 `buzzer=0 relay=0` 编码为
  `CMD_SET_OUTPUT (0x04)`，payload[0] 为蜂鸣器、payload[1] 为继电器。
- [DONE] 串口桥新增 `base/output_state`，Web 的开关状态优先显示 STM32 状态帧
  返回的 `fan_on/buzzer_on`，避免只显示浏览器本地假状态。
- [DONE] 已部署到 KICKPI `10.81.123.173`，两个 ROS2 systemd 服务重新构建并
  重启后均为 `active`。安全验证只发送了继电器和蜂鸣器全关闭命令；日志已经
  看到 `output command sent`，尚未在新 HEX 烧录后实测 GPIO 动作。

### 视频和 K230 图像方向

- [DONE] KICKPI 网关的实际 RTSP 帧确认是 `480x800` 且整体逆时针旋转 90 度，
  已在 OpenCV 代理中设置 `video_rotate_degrees=270`（等效逆时针 90 度），
  Web 首帧已恢复为正常横屏、文字正向。
- [PARTIAL] K230 本地脚本 `k230_canmv_apps/robot_touch_drive.py` 已加入
  `hmirror=True`、`vflip=True`，用于修正 OV5647 倒像；需要通过 CanMV 将该文件
  更新到实际自动运行的 `/sdcard/main.py` 或其调用的脚本，再重启 K230 实测。
