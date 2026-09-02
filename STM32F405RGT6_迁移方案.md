# STM32F405RGT6 底盘控制器迁移方案

> 创建日期：2026-08-31  
> 旧方案：STM32F103C8T6 最小系统板  
> 新方案：自制 STM32F405RGT6 LQFP64 最小系统板  
> 原则：保留底盘外设的网络名和功能分配；重新设计 MCU 最小系统、供电、时钟、下载和 PCB 封装。

## 1. 结论

`STM32F405RGT6` 适合替代 `STM32F103C8T6`，并且对当前机器人是一次有价值的升级：

- Cortex-M4F，带 FPU 和 DSP，最高 168 MHz。
- 1 MB Flash、192 KB SRAM，远大于 F103C8T6。
- 资源更充裕，可为后续 DMA、CAN、更多编码器、IMU 融合、复杂控制算法和日志保留空间。

但不能直接把 F405 焊到原 F103 底板或原 F103 最小系统板上：

```text
STM32F103C8T6：LQFP48，约 7 x 7 mm
STM32F405RGT6：LQFP64，10 x 10 mm
```

因此必须新画一块 F405 最小系统/底板，或购买一块引脚明确的 F405 最小系统板后重新对应接口。

当前 F103 工程不要删除。保留它作为已经验证过底板接口、驱动逻辑和调试流程的基线；F405 要建立独立工程后再迁移应用代码。

## 2. F405 对当前项目的直接收益

| 项目 | F103C8T6 | F405RGT6 | 对机器人底盘的意义 |
| --- | --- | --- | --- |
| CPU | Cortex-M3，72 MHz | Cortex-M4F，最高 168 MHz | 传感器滤波、编码器计算和闭环控制余量更大。 |
| 浮点运算 | 无硬件 FPU | 单精度 FPU | 速度 PID、姿态融合和里程计计算更轻松。 |
| Flash | 64 KB 级别 | 1 MB | 可加入日志、协议扩展和更复杂故障处理。 |
| RAM | 20 KB 级别 | 192 KB + 4 KB 备份 SRAM | FreeRTOS 任务、缓存和 DMA 缓冲区更有空间。 |
| DMA | 较少 | 双 DMA、16 stream | UART/I2C/ADC 后续可逐步改为 DMA。 |
| 外设 | 基础 | 更多定时器、串口、I2C、CAN、ADC、USB | 后续扩展不容易缺资源。 |

## 3. 不能沿用的部分

### 3.1 不能复用原 F103 PCB

原因：

- LQFP48 与 LQFP64 封装和焊盘数量不同。
- F405 的电源引脚、`VCAP_1`、`VCAP_2`、时钟引脚和 BOOT0 引脚与 F103 不同。
- F103 的 HSE 使用 `PD0/PD1`；F405RGT6 LQFP64 的 HSE 使用 `PH0/PH1`。
- 原 PCB 上的 F103 最小系统双排针机械尺寸也不能直接用于裸 F405。

### 3.2 不能直接使用 F103 最小系统板

如果手上的是带 `XC6204B332MR` 之类 150 mA 3.3V 稳压器的 F103 最小系统板，它不能变成 F405 最小系统板。

F405 在高频、外设开启的情况下供电电流明显更高。新板应使用独立的高质量 3.3V 供电，不建议依赖 150 mA 小型 LDO。

### 3.3 不能直接复制 F103 的 CubeMX 生成文件

以下 F103 自动生成文件不应复制到 F405 工程：

```text
Core/Src/main.c
Core/Src/gpio.c
Core/Src/i2c.c
Core/Src/tim.c
Core/Src/usart.c
Core/Src/stm32f1xx_it.c
Core/Src/stm32f1xx_hal_timebase_tim.c
Core/Src/system_stm32f1xx.c
Core/Inc/main.h
Core/Inc/stm32f1xx_hal_conf.h
Drivers/
startup_stm32f103*.s
```

这些文件必须由针对 `STM32F405RGTx` 的 CubeMX 工程重新生成。

可迁移的核心业务代码：

```text
Core/Src/robot_app.c
Core/Src/robot_comm.c
Core/Src/robot_motor.c
Core/Src/robot_encoder.c
Core/Src/robot_sensors.c
Core/Src/robot_delay.c

Core/Inc/robot_app.h
Core/Inc/robot_comm.h
Core/Inc/robot_motor.h
Core/Inc/robot_encoder.h
Core/Inc/robot_sensors.h
Core/Inc/robot_delay.h
Core/Inc/robot_config.h
```

## 4. 推荐的 F405 引脚分配

这里优先保持目前已布线、已写代码的网络名。这样 TB6612、编码器、传感器和 KICKPI 串口的外部接口逻辑不变，只替换 MCU 最小系统。

| 功能 | F405 引脚 | 外设配置 | 与旧方案关系 |
| --- | --- | --- | --- |
| KICKPI/调试 TX | `PA9` | USART1_TX | 保持不变。 |
| KICKPI/调试 RX | `PA10` | USART1_RX | 保持不变。 |
| I2C SCL | `PB10` | I2C2_SCL | 保持不变。 |
| I2C SDA | `PB11` | I2C2_SDA | 保持不变。 |
| 电机 B PWM | `PA8` | TIM1_CH1 | 保持不变。 |
| 电机 A PWM | `PA11` | TIM1_CH4 | 保持不变。 |
| 编码器 A 相 A/B | `PA0` / `PA1` | TIM2_CH1/CH2，TI1 and TI2 | 保持不变。 |
| 编码器 B 相 A/B | `PB6` / `PB7` | TIM4_CH1/CH2，TI1 and TI2 | 保持不变。 |
| TB6612 STBY | `PA6` | GPIO 输出 | 保持不变。 |
| AIN2 / AIN1 | `PB12` / `PB13` | GPIO 输出 | 保持不变。 |
| BIN1 / BIN2 | `PB14` / `PB15` | GPIO 输出 | 保持不变。 |
| DS18B20 数据 | `PA4` | GPIO 开漏/输入 | 保持不变。 |
| 蜂鸣器 | `PB0` | GPIO 输出 | 保持不变，现阶段软件禁用。 |
| 继电器 | `PB1` | GPIO 输出 | 保持不变，现阶段软件禁用。 |
| 状态 LED | `PC13` | GPIO 输出 | 保持不变。 |
| SWDIO | `PA13` | SYS_SWDIO | 保持不变。 |
| SWCLK | `PA14` | SYS_SWCLK | 保持不变。 |
| HSE 晶振输入 | `PH0` | RCC_OSC_IN | 与 F103 不同，必须重画。 |
| HSE 晶振输出 | `PH1` | RCC_OSC_OUT | 与 F103 不同，必须重画。 |

注意：

- `PA11` 同时是 USB DM 的复用脚，但当前项目将它保留给电机 A PWM；因此本版不使用 USB OTG。
- `PA0/PA1` 作为编码器输入时，先确认编码器输出电平。如果编码器是 5V 推挽输出，必须先做 5V 到 3.3V 电平转换，不能直接假定可接。
- 所有 I2C 模块应以 3.3V 供电，SCL/SDA 上拉电阻也必须拉到 3.3V。多个模块自带上拉时，注意总上拉不能过强。

## 5. F405 最小系统必须新增的电路

### 5.1 3.3V 主电源

推荐电源树：

```text
12V / 5V 系统电源
    -> 独立 3.3V DC/DC 或低噪声稳压器
    -> STM32F405 VDD、VDDA、传感器逻辑电源
```

要求：

- 输出：`3.3V`。
- 建议能力：`500 mA` 连续输出最低，`1 A` 更稳妥。
- 不使用原 F103 最小系统板上的 150 mA 稳压器作为 F405 主电源。
- 现有一个 AMS1117 只有在确认它的输入为 5V、输出为 3.3V、散热铜皮足够，且整条 3.3V 实测电流不高的情况下才可以暂时使用。
- 不要把两个 AMS1117 的 3.3V 输出直接并联。不同芯片的输出电压存在误差，可能造成两个稳压器互相“顶电流”。
- 当前项目没有必要为了 F405 单独再加一个 AMS1117；更推荐使用一个额定 500 mA 以上、最好 1 A 的 3.3V DC/DC 或低噪声稳压器，再给 MCU 和传感器做局部去耦/磁珠隔离。
- 如果仍使用 AMS1117，从 5V 降到 3.3V 时发热功率为 `P=(5-3.3) x I`：100 mA 约 0.17 W，200 mA 约 0.34 W，300 mA 约 0.51 W，500 mA 约 0.85 W。电流越大，SOT-223 封装越容易因温升而限流或掉压。
- 3.3V 稳压器靠近 MCU 的输出端增加 `10 uF + 100 nF`。
- MCU 和电机电源从布局、走线和回流上分区；电机噪声不要直接灌入 MCU 3.3V。

### 5.2 F405 电源去耦

| 网络/位置 | 建议器件 | 连接方式 | 目的 |
| --- | --- | --- | --- |
| 每个 VDD 引脚附近 | `100 nF X7R 0402/0603` | VDD 到 GND | 高频去耦，电容必须靠近电源脚。 |
| MCU 3.3V 入口 | `4.7 uF` 至 `10 uF X7R` | 3.3V 到 GND | 提供局部储能。 |
| VDDA | 磁珠或 `10 ohm` 电阻 + `100 nF + 1 uF` | 3.3V 经滤波后到 VDDA | 降低数字/电机噪声对模拟部分的影响。 |
| VSSA | 直接接 GND 平面 | VSSA 到 GND | 模拟地参考。 |
| VCAP_1 | `2.2 uF X7R` | VCAP_1 到 GND | F405 内核稳压器稳定电容。 |
| VCAP_2 | `2.2 uF X7R` | VCAP_2 到 GND | F405 内核稳压器稳定电容。 |
| VBAT | `0 ohm` 到 3.3V 或备用电池 | VBAT 供 RTC/备份域 | 不用备用电池时接 3.3V，不要悬空。 |

强制规则：

- `VCAP_1`、`VCAP_2` 只能各自通过 2.2 uF 电容接地。
- `VCAP_1`、`VCAP_2` 不能接 3.3V，不能互相短接，不能给外部负载供电。
- 所有 VDD/VSS、VDDA/VSSA 必须连接，不能遗漏。

### 5.3 时钟

建议沿用 8 MHz HSE：

```text
PH0/OSC_IN -- 8 MHz 晶振 -- PH1/OSC_OUT
PH0 -> 晶振负载电容 -> GND
PH1 -> 晶振负载电容 -> GND
```

器件建议：

- 晶振：8 MHz、无源晶体。
- 两个负载电容：先按晶振标称 CL 和 PCB 寄生计算；常见起点是 `12 pF` 到 `18 pF`，不要机械照搬旧板数值。
- 晶振、两个电容和 PH0/PH1 形成的小环路必须紧凑，远离 TB6612、电机线、DC/DC 电感和继电器。

不需要 RTC 时，`PC14/PC15` 的 32.768 kHz 晶振可以不装。

### 5.4 复位、启动和下载

| 网络 | 推荐连接 |
| --- | --- |
| NRST | `10 kOhm` 上拉到 3.3V，`100 nF` 到 GND；预留复位测试点或按钮焊盘。 |
| BOOT0 | `10 kOhm` 下拉到 GND；预留跳帽/焊盘可临时接 3.3V 进入系统 Bootloader。 |
| SWD | 预留 `3.3V、GND、SWDIO(PA13)、SWCLK(PA14)、NRST` 五针或六针接口。 |

本项目即使不需要实体按键，也必须保留 SWD 和 NRST 测试点。裸 MCU 首次上电、救砖和调试都依赖它们。

## 6. 新 PCB 的布局要点

1. F405 放在逻辑区域中心，远离 TB6612、电机端子、继电器触点和降压电感。
2. 8 MHz 晶振放在 PH0/PH1 旁边，走线短、等长要求不苛刻，但要远离噪声源。
3. 每个 VDD 的 100 nF 电容贴近对应电源脚，电容 GND 端用短过孔直入地平面。
4. 两个 VCAP 的 2.2 uF 必须非常靠近 VCAP 引脚。
5. 3.3V 和 GND 用完整平面或宽铜皮，不从电机大电流回路中穿过。
6. STM32 到 TB6612 的 PWM/方向线远离电机输出线；必要时在 PWM/方向线上预留 `22 ohm` 到 `47 ohm` 串联电阻焊盘。
7. 外部编码器和 I2C 接口靠近板边；若电缆较长，预留 ESD/串联电阻位置。
8. SWD 接口安排在板边并标清 `3V3/GND/SWDIO/SWCLK/NRST`。

## 7. CubeMX 新工程配置

### 7.1 新工程创建

不要修改现有 `D:\cxdownload\end\code\code.ioc` 后直接覆盖工程。

建议新建目录：

```text
D:\cxdownload\end\code\stm32f405_robot_base
```

CubeMX 选择：

```text
MCU：STM32F405RGTx
Package：LQFP64
Firmware Package：STM32Cube FW_F4
Toolchain：MDK-ARM 或当前实际使用的工具链
```

### 7.2 时钟建议

第一阶段建议先使用 `84 MHz`，电源和温升更保守，已经比 F103 的 72 MHz 更有余量：

```text
HSE：8 MHz Crystal/Ceramic Resonator
SYSCLK：84 MHz
AHB：84 MHz
APB1：42 MHz
APB2：84 MHz
```

稳定后，再根据实时负载和供电情况升级到 168 MHz：

```text
HSE：8 MHz
SYSCLK：168 MHz
AHB：168 MHz
APB1：42 MHz
APB2：84 MHz
```

### 7.3 外设配置

```text
SYS:
  Debug = Serial Wire

FREERTOS:
  CMSIS-V2

USART1:
  Asynchronous
  PA9 = TX
  PA10 = RX
  115200 bps

I2C2:
  PB10 = SCL
  PB11 = SDA

TIM1:
  PA8  = CH1 PWM
  PA11 = CH4 PWM

TIM2:
  PA0/PA1
  Encoder Mode = TI1 and TI2

TIM4:
  PB6/PB7
  Encoder Mode = TI1 and TI2

GPIO output:
  PA6 = TB6612_STBY
  PB12 = AIN2
  PB13 = AIN1
  PB14 = BIN1
  PB15 = BIN2
  PB0 = BUZZER_GATE
  PB1 = RELAY_GATE
  PC13 = STATUS_LED

GPIO:
  PA4 = DS18B20
```

HAL 时间基准建议：

- FreeRTOS 使用 SysTick。
- HAL Timebase 选择 `TIM6`，避免和 TIM1/TIM2/TIM4 的电机功能冲突。
- 不再沿用 F103 工程中 `TIM3` 作为 HAL Timebase 的生成文件。

PWM 频率建议保持 20 kHz：

```text
84 MHz 时：TIM1 Prescaler = 0，Period = 4199
168 MHz 时：TIM1 Prescaler = 0，Period = 8399
```

现有 `robot_motor.c` 会读取 TIM1 的自动重装值计算占空比，因此 PWM 周期改变后，上层 PWM 百分比逻辑不用重写。

## 8. 软件迁移步骤

1. 在 `stm32f405_robot_base` 生成全新的 F405 CubeMX 工程。
2. 先只点亮 `PC13`、使用 SWD 下载，确认供电、晶振和复位正确。
3. 依次验证 USART1、I2C2、TIM1 PWM、TIM2 编码器、TIM4 编码器和 DS18B20 GPIO。
4. 从旧工程复制 `robot_*.c/.h` 业务代码到新工程。
5. 按 F405 自动生成的 `main.h`、`tim.h`、`usart.h`、`i2c.h` 修复包含关系。
6. 删除任何 F1 专用头文件和 `stm32f1xx_*` 文件引用。
7. 在 F405 工程中启用 Cortex-M4F 的硬浮点设置；让 CubeMX/MDK 自动选择，不手工混用软浮点库。
8. 先将：

```c
#define ROBOT_UART1_DEBUG_ONLY           1
#define ROBOT_MOTOR_AUTO_TEST_ENABLE     0
```

保持为调试模式，完成板级验证。

9. 双电机、双编码器、四类传感器稳定后，才切换为：

```c
#define ROBOT_UART1_DEBUG_ONLY           0
```

并接入 KICKPI ROS 2 正式二进制串口桥。

## 9. 迁移时建议顺手优化的代码

以下不是首次点亮的阻塞项，但 F405 资源更足，建议后续完成：

- 将 UART1 接收改为 DMA + 空闲中断，降低高速遥测时的中断压力。
- 将 I2C 传感器读取逐步改为 DMA 或异步状态机。
- TIM2 是 32 位定时器；当前编码器代码以 16 位增量处理，可在 F405 版本改为 32 位累计计数。
- 使用 FPU 后再评估速度 PID 参数的数据类型和计算精度。
- 预留 CAN 收发器接口，便于将来扩展电机驱动器或底盘总线。
- 预留 ADC 电池分压输入，作为 INA219 之外的电池电压冗余监测。

## 10. 首次上电检查表

- [ ] 所有 VDD/VSS、VDDA/VSSA 已连接。
- [ ] VCAP_1 和 VCAP_2 各有独立 2.2 uF 到 GND，未接到 3.3V。
- [ ] BOOT0 默认被 10 kOhm 下拉到 GND。
- [ ] NRST 有上拉和测试点。
- [ ] SWDIO、SWCLK、NRST、3.3V、GND 都能接到 ST-Link。
- [ ] 3.3V 空载和上电时均稳定，没有被电机启动拉低。
- [ ] MCU 首次只烧录 LED 闪烁程序，不先接电机。
- [ ] 逻辑电源稳定后，再接 I2C、编码器和 TB6612。
- [ ] 最后才接 KICKPI 和整车电机供电。

## 11. 参考资料

- ST `DS8626 Rev 12`：STM32F405xx/STM32F407xx 数据手册。
- ST `RM0090`：STM32F405/407 等系列参考手册。
- ST `AN2834`：STM32 ADC 精度优化应用笔记。

## 12. 2026-08-31 供电和 Bridge 更新

### 12.1 AMS1117 决策

- `F405 + INA219 + DHT30 + MPU6050 + DS18B20` 的逻辑负载原则上可以由一条 3.3V 电源供电，但必须以实测总电流和 AMS1117 温升为准。
- 单独给 F405 再放一个 AMS1117 不是当前首选方案；这会增加电源域、压差、反向供电和调试复杂度。
- 如果后续实测 3.3V 总电流超过约 `200~300 mA`，或 AMS1117 外壳明显发烫，应该换成 3.3V 开关降压模块/高电流低噪声 LDO，而不是继续叠加第二个 AMS1117。
- KICKPI 和 K230 使用 5V 降压模块供电，不应接入 F405 的 3.3V AMS1117 输出。

### 12.2 嘉立创 EDA Bridge

已启动本地 Bridge：

```text
服务：http://127.0.0.1:49620
健康检查：http://127.0.0.1:49620/health
窗口列表：http://127.0.0.1:49620/eda-windows
```

当前健康状态：

```text
service: easyeda-bridge
status: ok
edaConnected: true
edaWindowCount: 1
activeWindowId: e3a57af7-099f-4d54-ae22-121631a1dc33
```

说明：Bridge 服务已经运行，嘉立创 EDA 扩展也已连接到一个活动窗口。后续执行原理图或 PCB API 操作前，仍应先确认当前窗口打开的是目标工程和正确文档。
