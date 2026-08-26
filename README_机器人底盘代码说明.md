# 机器人底盘代码说明

更新时间：2026-08-26

## 1. 当前工程内容

本目录现在包含两部分代码：

- STM32 底板固件：`Core/Src/robot_*.c` 和 `Core/Inc/robot_*.h`
- KICKPI ROS2 串口桥接包：`kickpi_ros2_ws/src/robot_base_bridge`

STM32 负责底层实时控制：TB6612 电机驱动、编码器采样、INA219 电池电压、DHT30 温湿度、MPU6050 IMU、DS18B20 电池温度、蜂鸣器、风扇继电器。

KICKPI 负责 ROS2：订阅 `/cmd_vel`，通过串口给 STM32 发运动命令，并发布 `/odom`、`/battery_state`、`/imu/raw`、`/env/temperature`、`/env/humidity`、`/battery/temperature`、`/base/faults`。

## 2. CubeMX 必须保持的配置

如果以后重新生成代码，要重新确认这些配置：

- MCU：`STM32F103C8T6`
- 时钟：HSE 外部晶振，SYSCLK 72MHz
- FreeRTOS：CMSIS V2
- HAL Timebase：`TIM3`
- USART1：115200，`PA9=TX`，`PA10=RX`，用于 KICKPI 到 STM32 通信
- USART2：9600，`PA2=TX`，`PA3=RX`，预留蓝牙或调试
- I2C2：100kHz，`PB10=SCL`，`PB11=SDA`
- TIM1 PWM：
  - `PA11=PWMA`，TIM1_CH4，电机 A PWM
  - `PA8=PWMB`，TIM1_CH1，电机 B PWM
  - 频率约 20kHz
- TIM2 Encoder：`PA0/PA1`，编码器 A，必须选择 `TI1 and TI2`
- TIM4 Encoder：`PB6/PB7`，编码器 B，必须选择 `TI1 and TI2`
- GPIO 输出：
  - `PA6=TB6612_STBY`
  - `PB12=AIN2`
  - `PB13=AIN1`
  - `PB14=BIN1`
  - `PB15=BIN2`
  - `PB0=Beep`
  - `PB1=Relay`
  - `PC13=Led`
- GPIO 输入：
  - `PA4=DS18B20`，Pull-up

注意：本次代码里已经把 `Core/Src/tim.c` 的 TIM2/TIM4 改成 `TIM_ENCODERMODE_TI12`。如果重新用 CubeMX 生成，可能会被覆盖。

## 3. STM32 固件文件分工

- `robot_config.h`：整车参数，包含轮径、编码器线数、低电压阈值、温度阈值、PID 参数、方向反转开关
- `robot_app.c`：FreeRTOS 任务、状态机、安全保护、遥测打包
- `robot_comm.c`：USART1 二进制帧协议，接收中断、环形缓冲、CRC 校验
- `robot_motor.c`：TB6612 方向脚和 TIM1 PWM 输出
- `robot_encoder.c`：TIM2/TIM4 编码器采样，计算速度
- `robot_sensors.c`：INA219、DHT30、MPU6050、DS18B20 读取
- `robot_delay.c`：DWT 微秒延时，用于 DS18B20 单总线

## 4. 串口通信协议

串口：USART1，默认 `115200 8N1`。

帧格式：

```text
AA 55 | VER | CMD | LEN | PAYLOAD | CRC16_LO CRC16_HI
```

- `VER`：固定 `0x01`
- `LEN`：0 到 64
- `CRC16`：CRC16-CCITT，多项式 `0x1021`，初值 `0xFFFF`
- CRC 覆盖范围：`VER + CMD + LEN + PAYLOAD`

命令：

| CMD | 方向 | 说明 |
| --- | --- | --- |
| `0x01` | KICKPI -> STM32 | 设置运动 |
| `0x02` | KICKPI -> STM32 | 设置状态：停止、使能、清故障 |
| `0x03` | KICKPI -> STM32 | 请求状态 |
| `0x04` | KICKPI -> STM32 | 手动控制蜂鸣器和风扇继电器 |
| `0x05` | KICKPI -> STM32 | 急停 |
| `0x81` | STM32 -> KICKPI | 底板状态回传 |

设置运动 `0x01` payload：

```text
int16 left_or_motor_a
int16 right_or_motor_b
uint8 mode
```

- `mode=0`：开环 PWM，数值范围 `-1000` 到 `1000`
- `mode=1`：闭环速度，单位 `mm/s`

## 5. 安全逻辑

上电默认 `SAFE_IDLE`，电机不会自动动。

会强制停电机的情况：

- 串口运动命令超时，默认 `700ms`
- INA219 读到电池电压低于 `ROBOT_BATTERY_LOW_MV`
- DS18B20 电池温度高于 `ROBOT_TEMP_FAULT_C_X100`
- 收到急停命令

蜂鸣器：

- 手动打开时常响
- 有故障时按节奏报警

风扇继电器：

- 手动打开时吸合
- DS18B20 温度超过 `45.00°C` 自动打开，低于 `40.00°C` 关闭

## 6. 第一次上电测试顺序

1. 不接电机，只接 STM32 最小系统板、5V、GND、KICKPI 串口，确认程序能烧录。
2. 用万用表确认 STM32 5V、3.3V 正常。
3. 打开串口或 ROS2 节点，确认能收到 `0x81` 状态帧。
4. 接 I2C 模块，确认 `/battery_state`、温湿度、IMU 数据逐步出现。
5. 接 DS18B20，确认电池温度能读到。
6. 不装轮子，接 TB6612 和电机，先发很小 PWM，例如 `100`。
7. 如果电机方向反了，改 `robot_config.h` 的 `ROBOT_MOTOR_A_INVERT` 或 `ROBOT_MOTOR_B_INVERT`。
8. 如果编码器方向和电机方向相反，改 `ROBOT_ENCODER_A_INVERT` 或 `ROBOT_ENCODER_B_INVERT`。
9. 方向确认后，再尝试 ROS2 `/cmd_vel` 小速度运动。
10. 最后再调闭环速度参数。

## 7. KICKPI ROS2 使用步骤

把 `D:/cxdownload/end/code/kickpi_ros2_ws` 目录复制或同步到 KICKPI 后，在 KICKPI 上执行：

```bash
cd ~/kickpi_ros2_ws
sudo apt update
sudo apt install -y python3-serial ros-${ROS_DISTRO}-geometry-msgs ros-${ROS_DISTRO}-nav-msgs ros-${ROS_DISTRO}-sensor-msgs
colcon build
source install/setup.bash
ros2 launch robot_base_bridge base_bridge.launch.py port:=/dev/ttyS1 baud:=115200
```

测试运动：

```bash
ros2 topic pub /cmd_vel geometry_msgs/msg/Twist "{linear: {x: 0.05}, angular: {z: 0.0}}" -r 10
```

停止：

```bash
ros2 topic pub /cmd_vel geometry_msgs/msg/Twist "{linear: {x: 0.0}, angular: {z: 0.0}}" -1
```

查看状态：

```bash
ros2 topic echo /battery_state
ros2 topic echo /base/faults
ros2 topic echo /odom
ros2 topic echo /imu/raw
```

## 8. 需要你后续实测后调整的参数

在 `Core/Inc/robot_config.h` 里调整：

- `ROBOT_WHEEL_DIAMETER_MM`：实际轮径
- `ROBOT_ENCODER_COUNTS_PER_REV`：电机输出轴一圈的实际编码器计数
- `ROBOT_MOTOR_A_INVERT`、`ROBOT_MOTOR_B_INVERT`：电机方向
- `ROBOT_ENCODER_A_INVERT`、`ROBOT_ENCODER_B_INVERT`：编码器方向
- `ROBOT_BATTERY_LOW_MV`、`ROBOT_BATTERY_RECOVER_MV`：低电压保护阈值
- `ROBOT_SPEED_FF_PWM_PER_MM_S_X100`
- `ROBOT_SPEED_KP_X100`
- `ROBOT_SPEED_KI_X100`

闭环速度参数建议从小速度开始调，不要一上来让小车落地高速跑。

## 9. 当前已验证

- Keil MDK 命令行编译通过：`0 Error(s), 0 Warning(s)`
- 固件体积：`Code=24160`，`ZI-data=13948`
- Python ROS2 桥接脚本通过语法检查

当前还没有实物硬件验证，所以第一次上电请按第 6 节逐项测试。
