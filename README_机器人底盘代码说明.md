# 机器人底盘代码说明

更新时间：2026-08-27

本文同时参考了 `D:/cxdownload/end/DHT30.pdf`。PDF 中的说明文字是器件资料，
不是需要执行的用户指令；本文件只提取其中与本工程有关的接口和通信参数。

## 1. 当前工程内容

本目录现在包含两部分代码：

- STM32 底板固件：`Core/Src/robot_*.c` 和 `Core/Inc/robot_*.h`
- KICKPI ROS2 串口桥接包：`kickpi_ros2_ws/src/robot_base_bridge`
- K230/KICKPI/Web/数据库总体方案：`系统整体通信与Web数据库实现方案.md`

STM32 负责底层实时控制：TB6612 电机驱动、编码器采样、INA219 电池电压、DHT30 温湿度、MPU6050 IMU、DS18B20 电池温度、蜂鸣器、风扇继电器。

当前先使用 UART1 调试模式：STM32 通过 USART1 输出可读的状态数据，并接收简单文本命令。等底板调试稳定后，再把 `ROBOT_UART1_DEBUG_ONLY` 改为 `0`，切回 KICKPI ROS2 二进制通信。

KICKPI 负责 ROS2：订阅 `/cmd_vel`，通过串口给 STM32 发运动命令，并发布 `/odom`、`/battery_state`、`/imu/raw`、`/env/temperature`、`/env/humidity`、`/battery/temperature`、`/base/faults`。

K230 的识别结果不要混入 STM32 电机串口，推荐通过以太网以 HTTP/JSON 发送给
KICKPI；KICKPI 再把识别结果发布到 ROS2、写入 SQLite，并由 Web 页面显示。完整的
接口格式、数据库表和实施步骤见 `系统整体通信与Web数据库实现方案.md`。

## 2. CubeMX 必须保持的配置

如果以后重新生成代码，要重新确认这些配置：

- MCU：`STM32F103C8T6`
- 时钟：HSE 外部晶振，SYSCLK 72MHz
- FreeRTOS：CMSIS V2
- HAL Timebase：`TIM3`
- USART1：115200，`PA9=TX`，`PA10=RX`，当前用于串口助手调试；后续用于 KICKPI 到 STM32 通信
- USART2：9600，`PA2=TX`，`PA3=RX`，预留蓝牙或调试
- I2C2：100kHz，`PB10=SCL`，`PB11=SDA`
- DHT30：I2C 地址 `0x38`，按 AHT30/DHT30 协议读取，测量命令为 `0xAC 0x33 0x00`
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

注意：当前 `code.ioc` 和 `Core/Src/tim.c` 都已确认 TIM2/TIM4 使用 `TIM_ENCODERMODE_TI12`，对应 CubeMX 界面的 `TI1 and TI2`。如果重新用 CubeMX 生成，仍要重新确认一次。

## 3. STM32 固件文件分工

- `robot_config.h`：整车参数，包含 UART1 调试/正式模式、蜂鸣器总开关、轮径、编码器线数、低电压阈值、温度阈值、PID 参数、方向反转开关
- `robot_app.c`：FreeRTOS 任务、状态机、安全保护、遥测打包
- `robot_comm.c`：USART1 接收中断、环形缓冲、调试文本命令、正式模式二进制帧协议、CRC 校验
- `robot_motor.c`：TB6612 方向脚和 TIM1 PWM 输出
- `robot_encoder.c`：TIM2/TIM4 编码器采样，计算速度
- `robot_sensors.c`：INA219、DHT30、MPU6050、DS18B20 读取
- `robot_delay.c`：DWT 微秒延时，用于 DS18B20 单总线

## 4. 当前 UART1 调试模式

当前 `Core/Inc/robot_config.h` 中：

```c
#define ROBOT_UART1_DEBUG_ONLY 1
#define ROBOT_BUZZER_ENABLE    0
#define ROBOT_RELAY_TEST_ENABLE 1
#define ROBOT_RELAY_TEST_PERIOD_MS 5000U
```

含义：

- UART1 只输出可读调试信息，不发送二进制帧。
- UART1 可以接收文本命令，每条命令以回车或换行结束。
- 蜂鸣器被强制关闭，就算出现故障也不会响。
- 继电器测试默认打开：上电约 5 秒后翻转一次，之后每 5 秒翻转一次，便于确认
  PB1、继电器驱动和风扇输出是否正常。
- 这个模式适合你先用 USB 转串口模块调试底板，不急着接 KICKPI。

串口助手配置：

```text
115200 baud
8 data bits
no parity
1 stop bit
newline: CRLF 或 LF 都可以
```

上电后每 500ms 会输出三行状态：

```text
T=时间 STATE=状态 FAULT=故障 MODE=模式 TARGET=目标 PWM=实际输出 RELAY=继电器 BUZZ=蜂鸣器
ENC 编码器累计值/本周期增量/速度
SENS 传感器有效标志/电池电压/温湿度/电池温度/MPU6050/RX溢出计数
```

调试命令：

| 命令 | 说明 |
| --- | --- |
| `help` | 打印命令帮助 |
| `status` | 立即打印一次状态 |
| `enable` | 进入 READY，不会自动转电机 |
| `stop` | 停止电机，回到安全空闲 |
| `clear` | 清除故障并停止电机 |
| `estop` | 急停，进入 FAULT |
| `pwm 100 100` | 电机 A/B 开环 PWM 测试，范围 `-1000..1000` |
| `pwm 0 0` | 停止 PWM 输出 |
| `speed 50 50` | 闭环速度测试，单位 `mm/s`，需要编码器方向先调对 |
| `relay 1` | 打开风扇继电器 |
| `relay 0` | 关闭手动继电器 |
| `beep 1` | 当前会被忽略，因为蜂鸣器已禁用 |

第一次测电机建议从 `pwm 80 80` 或 `pwm 100 100` 开始，而且先让轮子悬空。

## 5. 后续 KICKPI 二进制通信协议

正式连接 KICKPI 前，先把 `Core/Inc/robot_config.h` 改成：

```c
#define ROBOT_UART1_DEBUG_ONLY 0
```

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

## 6. 安全逻辑

上电默认 `SAFE_IDLE`，电机不会自动动。

会强制停电机的情况：

- 串口运动命令超时，默认 `700ms`
- INA219 读到电池电压低于 `ROBOT_BATTERY_LOW_MV`
- DS18B20 电池温度高于 `ROBOT_TEMP_FAULT_C_X100`
- 收到急停命令

蜂鸣器：

- 当前 `ROBOT_BUZZER_ENABLE=0`，所以永远不响。
- 后续需要报警时，把 `ROBOT_BUZZER_ENABLE` 改为 `1`。
- 使能后，手动打开时常响；有故障时按节奏报警。

风扇继电器：

- 手动打开时吸合
- DS18B20 温度超过 `45.00°C` 自动打开，低于 `40.00°C` 关闭
- `ROBOT_RELAY_TEST_ENABLE=1` 时，测试状态每隔 `5000ms` 翻转一次。
- 测试状态、手动打开、温度自动控制和过温保护采用“或”关系；只要其中一个要求
  打开，PB1 就输出打开电平。因此测试翻转不会关闭过温保护。
- 正式装车或不希望继电器周期动作时，把 `ROBOT_RELAY_TEST_ENABLE` 改为 `0`，
  重新编译并烧录。

## 11. DHT30 说明书对应关系

本工程使用的 DHT30 说明书明确给出了下面的连接和通信方式：

### 11.1 模块接线

- `VDD`：接 `3V3`。说明书允许 `2.2V~5.5V`，本工程的 STM32 I2C 电平是 3.3V，
  所以优先使用 3.3V。
- `SDA`：接 `PB11`，也就是 `I2C2_SDA`。
- `GND`：接系统 `GND`。
- `SCL`：接 `PB10`，也就是 `I2C2_SCL`。

MPU6050、INA219 和 DHT30 共用 `PB10/PB11` 是允许的，前提是三个模块的地址不冲突，
并且整个总线只需要合理的上拉。你买的模块如果已经带上拉电阻，不要再无条件重复
并联很多上拉电阻。

### 11.2 地址为什么看到 0x38、0x70 和 0x71

- 程序配置的 `DHT30_I2C_ADDR` 是 7 位地址 `0x38`。
- DHT30 说明书把写地址写成 `0x70`，这是把 7 位地址左移一位并把读写位清零后的
  8 位地址。
- 读地址是 `0x71`，这是同一个地址左移一位后把最低位设为读。
- STM32 HAL 接口要求传入左移后的地址，所以代码使用
  `(uint16_t)0x38 << 1`，发送时会得到写 `0x70`，接收时会自动得到读 `0x71`。

### 11.3 一次温湿度测量的完整流程

`dht30_update()` 每约 1 秒执行一次：

1. 向地址 `0x70` 写入三个命令字节：`0xAC 0x33 0x00`。
2. 等待约 `80ms`，给传感器完成一次测量。
3. 从地址 `0x71` 读取 7 个字节。
4. 检查第 0 字节的 Busy 位。`bit7=1` 表示仍在测量，当前数据不能使用。
5. 用说明书规定的 CRC8 校验前 6 个字节，初值 `0xFF`，多项式 `0x31`，结果必须
   等于第 7 个字节。
6. CRC 正确后，把 20 位湿度和 20 位温度原始值换算成百分比和摄氏度。

7 字节数据格式如下：

```text
buf[0]       状态 Status，bit7 是 Busy
buf[1..3]    20 位湿度原始数据 SRH
buf[3..5]    20 位温度原始数据 ST
buf[6]       CRC8，校验 buf[0] 到 buf[5]
```

换算公式：

```text
湿度(%RH) = SRH / 1048576 * 100
温度(°C)  = ST  / 1048576 * 200 - 50
```

代码中温度以 `0.01°C` 保存，湿度以 `0.01%RH` 保存。例如状态输出中的
`ENV=+25.30C 48.20%` 表示 25.30°C、48.20%RH。

代码使用 64 位中间结果完成温湿度换算，避免 `20 位原始值 * 20000` 在 32 位
整数中溢出。若看到温度变成约 `-52.xx°C`，通常是烧录了旧固件；重新烧录最新
的 `code.hex` 即可。

### 11.4 DHT30 故障判断

`SENS flags` 的 DHT30 有效位是 `0x0002`：

- `0x0002` 已置位：DHT30 最近一次测量成功，ENV 数据可用。
- `0x0002` 未置位：可能是供电、地址、SDA/SCL 接反、Busy 未结束或 CRC 错误。
- MPU6050 已经能读到时，说明 `PB10/PB11` 总线和 STM32 I2C2 初始化大概率正常，
  应优先检查 DHT30 模块的 VCC、GND、地址和模块本身。
- INA219 没有上电时，整体 flags 可能是 `0x000D`；INA219 接好后，DHT30 也正常时
  预期是 `0x000F`。

## 12. 继电器 5 秒翻转测试

继电器控制脚是 `PB1=Relay`。当前配置中：

```c
#define ROBOT_RELAY_TEST_ENABLE    1
#define ROBOT_RELAY_TEST_PERIOD_MS 5000U
```

系统启动后，`relay_test_on` 初始为 0。控制任务每次调用 `apply_outputs()` 时检查
经过的时间；累计达到 5000ms 就把测试状态取反，并把实际输出写到 `Relay_Pin`。
因此现象应当是：继电器先断开，约 5 秒后吸合，再约 5 秒后释放，循环往复。

需要注意：

- 这只是当前硬件验证用的编译期测试功能，不是最终风扇控制逻辑。
- 继电器模块若是高电平有效，当前代码可直接使用；若实测相反，需要增加一个
  “继电器有效电平”宏，再在输出处取反。
- 继电器线圈必须由模块上的三极管/驱动电路供电，STM32 的 PB1 只提供控制信号，
  不能直接给线圈供电。
- 继电器吸合和释放时可能产生干扰，测试时要确认驱动线圈有续流二极管，模块电源
  和 STM32 共地。
- 测试完成后把开关改成 0，避免小车运行时风扇每 5 秒自动切换。

## 13. 自定义函数功能说明

下面按源文件说明每个自定义函数的作用。函数前面的 `static` 表示它只在当前
`.c` 文件内部使用，公共函数则通过对应的 `.h` 文件供其它模块调用。

### 13.1 `robot_app.c`

| 函数 | 功能和作用 |
| --- | --- |
| `clamp_pwm_i32()` | 把任意 32 位 PWM 输入限制到 `-1000..1000`，防止串口或 PID 计算产生越界输出。 |
| `clamp_i32()` | 把积分项、速度目标等整数限制到指定上下限，防止控制量溢出。 |
| `read_i16_le()` | 按正式协议的小端格式读取一个有符号 16 位数。 |
| `put_u8()` | 向状态 payload 写入 1 字节并推进写指针。 |
| `put_u16()` | 按小端格式向 payload 写入无符号 16 位数。 |
| `put_i16()` | 按小端格式向 payload 写入有符号 16 位数。 |
| `put_i32()` | 按小端格式向 payload 写入有符号 32 位数。 |
| `ctx_lock()` | 进入机器人状态互斥区，防止多个 FreeRTOS 任务同时读写状态结构体。调度器未运行时不调用 OS 锁。 |
| `ctx_unlock()` | 释放 `ctx_lock()` 获得的状态互斥量。 |
| `apply_outputs()` | 根据手动风扇、温度自动风扇、继电器测试状态、过温保护和蜂鸣器开关，统一更新 PB1/PB0 输出。 |
| `update_faults()` | 检查串口超时、INA219 欠压、DS18B20 过温和可选堵转，生成故障码并决定是否进入 FAULT。 |
| `speed_pid()` | 用前馈、比例和积分三部分，根据目标速度和编码器实际速度计算 PWM。 |
| `control_step()` | 执行一次 10ms 控制周期：采样编码器、取传感器快照、更新故障、计算 PWM、停止或驱动电机。 |
| `state_name()` | 把内部状态编号转换成 `IDLE/READY/RUN/FAULT` 文字。 |
| `mode_name()` | 把运动模式编号转换成 `PWM` 或 `SPEED` 文字。 |
| `x100_sign()` | 获取以百分之一单位保存的温度符号，用于串口打印。 |
| `x100_abs_whole()` | 获取温度的绝对值整数部分。 |
| `x100_abs_frac()` | 获取温度的绝对值小数部分。 |
| `send_status()` | 调试模式打印三行文本状态；正式模式打包并发送二进制 `0x81` 状态帧。 |
| `handle_set_motion()` | 正式通信模式解析运动命令，保存 A/B 目标和 PWM/速度模式。 |
| `handle_set_state()` | 正式通信模式处理停止、使能、清故障命令。 |
| `handle_set_output()` | 正式通信模式处理蜂鸣器和风扇继电器手动请求。 |
| `RobotApp_OnFrame()` | 正式协议业务分发入口，根据 CMD 调用对应命令处理函数。 |
| `debug_print_help()` | 打印 UART1 文本调试命令帮助，并显示蜂鸣器/继电器测试开关状态。 |
| `debug_set_motion()` | 解析后的文本 PWM 或 SPEED 命令写入运动目标，不直接操作 GPIO。 |
| `debug_stop()` | 清除运动目标和命令标志，回到安全空闲并停止电机。 |
| `debug_clear_fault()` | 清除故障、清零目标并回到安全空闲，不会自动恢复运动。 |
| `debug_set_relay()` | 设置手动风扇继电器请求，并立即刷新一次输出。继电器测试打开时，周期测试仍会继续。 |
| `RobotApp_OnDebugLine()` | 文本命令总入口，识别 `help/status/pwm/speed/relay/stop/clear/estop` 等命令。 |
| `RobotApp_BoardInit()` | 在 FreeRTOS 启动前初始化 DWT、电机、编码器、传感器和 UART1，确保电机上电保持停止。 |
| `RobotApp_CreateTasks()` | 创建状态互斥量、注册通信回调，并创建控制、传感器、遥测三个应用任务。 |
| `RobotApp_DefaultTask()` | CubeMX 默认后台任务，目前只低频休眠，预留给不影响实时控制的后台功能。 |
| `RobotApp_StatusLedTask()` | 按 IDLE、RUN、FAULT 等状态改变 PC13 LED 闪烁速度。 |
| `RobotApp_GetStatus()` | 在互斥保护下复制完整状态快照，供串口输出和 ROS2 状态帧使用。 |
| `RobotApp_RequestTelemetry()` | 设置遥测请求标志，正式 KICKPI 模式可用来请求尽快回传状态。 |
| `RobotApp_ControlTask()` | FreeRTOS 控制任务入口，每 10ms 处理 UART1、控制周期和电机输出。 |
| `RobotApp_SensorTask()` | FreeRTOS 传感器任务入口，每 50ms 调用一次更新函数，由传感器模块内部节流。 |
| `RobotApp_TelemetryTask()` | FreeRTOS 遥测任务入口，每 500ms 输出文本状态或发送正式二进制状态。 |

### 13.2 `robot_comm.c`

| 函数 | 功能和作用 |
| --- | --- |
| `rx_push_from_isr()` | 在 UART 接收中断中把 1 字节放入环形缓冲区；满时只增加溢出计数，不阻塞中断。 |
| `rx_pop()` | 从环形缓冲区取出 1 字节，供任务上下文解析。 |
| `RobotComm_Crc16()` | 计算正式 KICKPI 帧使用的 CRC16-CCITT，初值由调用者传入。 |
| `parser_reset()` | 二进制帧出错后把解析状态机恢复到等待 `AA 55`。 |
| `parser_accept()` | 按帧头、版本、命令、长度、payload、CRC 顺序解析一个字节。 |
| `debug_accept()` | 文本模式按换行组装命令，支持退格删除。 |
| `RobotComm_Init()` | 清空 UART 接收状态并启动 USART1 的 1 字节中断接收。 |
| `RobotComm_SetFrameHandler()` | 注册正式二进制帧的业务回调。 |
| `RobotComm_SetDebugLineHandler()` | 注册调试文本行的业务回调。 |
| `RobotComm_ProcessRx()` | 在控制任务中取出所有待处理字节，并选择文本或二进制解析器。 |
| `RobotComm_SendFrame()` | 正式模式组装 `AA 55...CRC16` 帧并通过 USART1 发送；调试模式故意返回错误，避免乱码。 |
| `RobotComm_DebugPrintf()` | 用 printf 风格阻塞发送调试文本。日志不要过长，避免影响 10ms 控制周期。 |
| `RobotComm_GetRxOverflowCount()` | 返回接收环形缓冲区溢出次数。 |
| `HAL_UART_RxCpltCallback()` | USART1 收到字节后的 HAL 回调，保存字节并重新启动下一字节接收。 |
| `HAL_UART_ErrorCallback()` | USART1 出现错误后重新启动接收，避免链路永久停止。 |

### 13.3 `robot_motor.c`

| 函数 | 功能和作用 |
| --- | --- |
| `clamp_pwm()` | 将电机 PWM 限制到配置的安全范围。 |
| `set_dir()` | 根据正负方向设置 TB6612 的 IN1/IN2 方向脚；零值时两个方向脚都关闭。 |
| `duty_from_pwm()` | 根据 TIM1 自动重装值把 `-1000..1000` 换算成比较寄存器值。 |
| `RobotMotor_Init()` | 启动 TIM1 CH1/CH4 PWM，打开高级定时器主输出，并保持 STBY 关闭。 |
| `RobotMotor_Enable()` | 设置 TB6612 的 STBY；0 为待机，1 为允许驱动。 |
| `RobotMotor_SetPercent()` | 对 A/B 电机做限幅、方向反转处理、方向脚设置和 PWM 输出。 |
| `RobotMotor_Stop()` | 清零两个 PWM 和方向脚，进入安全停止状态。 |
| `RobotMotor_GetPercent()` | 返回软件保存的最近一次 PWM 值，用于状态显示。 |

### 13.4 `robot_encoder.c`

| 函数 | 功能和作用 |
| --- | --- |
| `apply_dir_a()` | 根据配置修正编码器 A 的方向。 |
| `apply_dir_b()` | 根据配置修正编码器 B 的方向。 |
| `counts_to_mm_s()` | 根据采样周期、轮径和每圈计数，把编码器增量换算成 mm/s。 |
| `RobotEncoder_Init()` | 启动 TIM2/TIM4 编码器接口并清零计数器和缓存。 |
| `RobotEncoder_Sample()` | 读取当前计数器、计算增量、累计值和速度，供 PID 和状态输出使用。 |
| `RobotEncoder_Get()` | 返回最近一次采样缓存，不主动读取定时器。 |

### 13.5 `robot_sensors.c`

| 函数 | 功能和作用 |
| --- | --- |
| `sensor_delay_ms()` | DHT30 等待测量时，在 FreeRTOS 运行后使用 `osDelay`，启动前使用 `HAL_Delay`。 |
| `set_valid()` | 设置或清除 INA219、DHT30、MPU6050、DS18B20 的有效标志。 |
| `i2c_read_reg16()` | 从 I2C 设备读取两个大端字节并拼成 16 位数。 |
| `i2c_write_reg8()` | 向 I2C 设备写入一个 8 位寄存器值。 |
| `ina219_init()` | 写 INA219 配置寄存器，准备读取总线电压。 |
| `ina219_update()` | 读取 INA219 bus voltage，换算成毫伏；本工程不使用分流电流测量。 |
| `mpu6050_init()` | 唤醒 MPU6050，设置数字低通和加速度/陀螺仪量程。 |
| `be_i16()` | 把 MPU6050 的两个大端字节转换成有符号 16 位数。 |
| `mpu6050_update()` | 连续读取加速度、内部温度和陀螺仪原始数据。 |
| `aht_crc8()` | 按 DHT30 说明书的初值 `0xFF`、多项式 `0x31` 计算 CRC8。 |
| `dht30_update()` | 发送 `0xAC 0x33 0x00`、等待 80ms、读取 7 字节、检查 Busy/CRC 并换算温湿度。 |
| `ds18b20_output_low()` | 把 PA4 配成开漏输出并主动拉低单总线。 |
| `ds18b20_release()` | 释放 PA4，让上拉电阻把 DS18B20 总线拉高。 |
| `ds18b20_reset()` | 发送 DS18B20 复位脉冲并检查存在脉冲。 |
| `ds18b20_write_bit()` | 按严格单总线时序写入一个 bit。 |
| `ds18b20_read_bit()` | 按严格单总线时序采样一个 bit。 |
| `ds18b20_write_byte()` | 以低位先出的方式写入一个字节。 |
| `ds18b20_read_byte()` | 以低位先出的方式读取一个字节。 |
| `ds18b20_crc8()` | 校验 DS18B20 scratchpad 的前 8 字节。 |
| `ds18b20_start_convert()` | 发送 Skip ROM 和 Convert T 命令，启动一次温度转换。 |
| `ds18b20_read_temperature()` | 读取 9 字节 scratchpad、做 CRC 校验并换算成 0.01°C。 |
| `ds18b20_update()` | 管理 DS18B20 的“先启动转换、延时、再读取”两阶段流程。 |
| `RobotSensors_Init()` | 清空缓存，初始化 INA219/MPU6050，并释放 DS18B20 总线。 |
| `RobotSensors_Update()` | 按 100ms/1s 周期更新快速和慢速传感器。 |
| `RobotSensors_Get()` | 复制最近一次传感器数据快照。 |
| `RobotSensors_ScanI2C()` | 扫描 I2C2 的 7 位地址，便于确认模块是否应答。 |

### 13.6 `robot_delay.c`

| 函数 | 功能和作用 |
| --- | --- |
| `RobotDelay_Init()` | 打开 Cortex-M3 的 DWT 周期计数器，为微秒延时做准备。 |
| `RobotDelay_Us()` | 用 DWT 忙等待指定微秒数，只给 DS18B20 时序使用，不用于普通长延时。 |

## 14. 调试时建议重点观察的字段

- `STATE=FAULT`：表示有保护条件，不要直接强行给大 PWM。
- `FAULT=0x0002`：低电压故障，INA219 未供电或读数不正确时会出现。
- `SENS flags=0x000D`：INA219、MPU6050、DS18B20 有效，DHT30 无效。
- `SENS flags=0x000F`：四种传感器都成功读到数据。
- `RELAY=1/0`：当前软件判断的继电器实际请求状态，包含 5 秒测试状态。
- `RXOV`：UART1 接收溢出次数；持续增加时要降低发送频率或检查串口配置。

## 7. 第一次上电测试顺序

1. 不接电机，只接 STM32 最小系统板、5V、GND、USB 转串口模块，确认程序能烧录。
2. 用万用表确认 STM32 5V、3.3V 正常。
3. 打开串口助手，确认 UART1 每 500ms 输出可读状态行。
4. 接 I2C 模块，确认状态行里的 `SENS flags` 和电池电压、温湿度、IMU 数据逐步出现。
5. 接 DS18B20，确认电池温度能读到。
6. 不装轮子，接 TB6612 和电机，先发很小 PWM，例如 `100`。
7. 如果电机方向反了，改 `robot_config.h` 的 `ROBOT_MOTOR_A_INVERT` 或 `ROBOT_MOTOR_B_INVERT`。
8. 如果编码器方向和电机方向相反，改 `ROBOT_ENCODER_A_INVERT` 或 `ROBOT_ENCODER_B_INVERT`。
9. 方向确认后，再把 `ROBOT_UART1_DEBUG_ONLY` 改为 `0`，连接 KICKPI。
10. 最后再尝试 ROS2 `/cmd_vel` 小速度运动，并调闭环速度参数。

## 8. KICKPI ROS2 使用步骤

注意：这一步要在底板 UART1 调试通过之后再做，并且固件需要设置 `ROBOT_UART1_DEBUG_ONLY=0`。

把 `D:/cxdownload/end/code/kickpi_ros2_ws` 目录复制或同步到 KICKPI 后，在 KICKPI 上执行：

```bash
cd ~/kickpi_ros2_ws
source /opt/ros/humble/setup.bash
sudo apt install -y python3-serial
colcon build
source install/setup.bash
ros2 launch robot_base_bridge base_bridge.launch.py port:=/dev/ttyAS5 baud:=115200
```

本项目这块 KICKPI 的普通 UART5 建议使用 `/dev/ttyAS5`，不是调试串口。40Pin
接线按下面连接：

```text
KICKPI 40Pin 8脚  UART5_TX  -> STM32 USART1_RX（PA10）
KICKPI 40Pin 10脚 UART5_RX  <- STM32 USART1_TX（PA9）
KICKPI 40Pin 6脚  GND       -> STM32 GND
```

KICKPI 和 STM32 的 UART 都是 3.3V 逻辑，不能把 5V 电源直接接到 UART 信号脚。
两块板必须共地，TX/RX 必须交叉连接。

KICKPI 已经存在 `/opt/ros/humble`，登录 root 后需要先 source：

```bash
source /opt/ros/humble/setup.bash
```

如果希望每次 root 登录自动获得 ROS2 环境，可以把上面这一行加入 `/root/.bashrc`。

仓库中还提供了 `kickpi_ros2_ws/robot-base-bridge.service`。安装到 KICKPI 后可以：

```bash
cp robot-base-bridge.service /etc/systemd/system/
systemctl daemon-reload
systemctl enable robot-base-bridge.service
systemctl start robot-base-bridge.service
systemctl status robot-base-bridge.service
```

注意：只有当 STM32 固件已经把 `ROBOT_UART1_DEBUG_ONLY` 改为 `0` 并烧录后，才启动
这个 systemd 服务。STM32 仍处于文本调试模式时不要启动，否则 KICKPI 会向 UART1
发送二进制帧。

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

## 9. 需要你后续实测后调整的参数

在 `Core/Inc/robot_config.h` 里调整：

- `ROBOT_UART1_DEBUG_ONLY`：`1` 为串口调试，`0` 为 KICKPI 正式通信
- `ROBOT_BUZZER_ENABLE`：`0` 为蜂鸣器永不响，`1` 为允许报警
- `ROBOT_WHEEL_DIAMETER_MM`：实际轮径
- `ROBOT_ENCODER_COUNTS_PER_REV`：电机输出轴一圈的实际编码器计数
- `ROBOT_MOTOR_A_INVERT`、`ROBOT_MOTOR_B_INVERT`：电机方向
- `ROBOT_ENCODER_A_INVERT`、`ROBOT_ENCODER_B_INVERT`：编码器方向
- `ROBOT_BATTERY_LOW_MV`、`ROBOT_BATTERY_RECOVER_MV`：低电压保护阈值
- `ROBOT_SPEED_FF_PWM_PER_MM_S_X100`
- `ROBOT_SPEED_KP_X100`
- `ROBOT_SPEED_KI_X100`

闭环速度参数建议从小速度开始调，不要一上来让小车落地高速跑。

## 15. 当前已验证

- Keil MDK 命令行编译通过：`0 Error(s), 0 Warning(s)`
- 固件体积：最近一次构建为 `Code=32656`，`ZI-data=13980`
- Python ROS2 桥接脚本通过语法检查
- KICKPI 已确认运行 Ubuntu 22.04.5、ROS 2 Humble，已安装 `python3-serial`
- KICKPI 工作空间已部署到 `/root/kickpi_ros2_ws` 并成功用 `colcon build --symlink-install`
- KICKPI 普通 UART5 设备为 `/dev/ttyAS5`，桥接包默认已改为该设备
- `/etc/systemd/system/robot-base-bridge.service` 已安装并通过 systemd 校验，目前保持
  `disabled/inactive`，等 STM32 正式通信固件准备好后再启动

当前还没有实物硬件验证，所以第一次上电请按第 6 节逐项测试。
