#ifndef ROBOT_CONFIG_H
#define ROBOT_CONFIG_H

#include <stdint.h>

/*
 * UART1 工作模式总开关。
 *
 * 1：底板调试模式。
 *    STM32 会通过 USART1 打印人能直接看懂的文本状态，并接收 help、status、
 *    pwm、relay 这类文本命令。这个阶段不要连接 KICKPI，让 USB 转串口工具
 *    先把底板、电机、编码器、传感器都验证清楚。
 *
 * 0：正式 KICKPI 通信模式。
 *    USART1 不再打印文本，而是使用带帧头和 CRC16 的二进制协议。KICKPI 上
 *    的 ROS2 bridge 节点会订阅 /cmd_vel，再把速度命令转发给 STM32。
 *
 * 调板子时保持为 1；硬件确认稳定后，只改这个宏为 0，再重新编译烧录。
 */
#define ROBOT_UART1_DEBUG_ONLY                1

/*
 * 本版本的未使用外设开关。
 *
 * 你的当前目标是先把底板的电机、编码器、传感器和 USART1 调试功能跑通，
 * 暂时不使用蜂鸣器、继电器/风扇和蓝牙。因此这里采用“硬关闭”而不是只
 * 不接线：代码不会主动驱动这些输出，也不会初始化蓝牙串口。
 */
#define ROBOT_RELAY_ENABLE                    0
#define ROBOT_BLUETOOTH_ENABLE                0

/*
 * 蜂鸣器总开关。
 *
 * 0：蜂鸣器永远不响。即使发生低电压、过温、急停、串口超时等故障，程序
 *    也只会在状态行里显示故障码，不会拉高 PB0。
 *
 * 1：允许程序控制蜂鸣器。后续整车装好后，如果需要报警，再打开这个宏。
 *
 * 你现在反馈蜂鸣器太吵，所以默认关掉。
 */
#define ROBOT_BUZZER_ENABLE                   0

/*
 * 继电器调试翻转开关。
 *
 * 1：允许继电器/风扇相关逻辑参与运行。
 * 0：彻底关闭继电器/风扇逻辑，PB1 始终输出低电平。
 *
 * 当 ROBOT_RELAY_ENABLE=0 时，本开关不会生效，PB1 仍然始终保持低电平。
 */
#define ROBOT_RELAY_TEST_ENABLE               0
#define ROBOT_RELAY_TEST_PERIOD_MS            5000U

/*
 * 电机和编码器方向配置。
 *
 * 这里约定 Motor A 是左轮，Motor B 是右轮。实际焊线或电机安装方向可能会
 * 导致“给正 PWM 但轮子反转”，这种情况不用重新焊线，优先改下面的反向宏。
 *
 * MOTOR_x_INVERT：只改变电机输出方向。
 * ENCODER_x_INVERT：只改变编码器计数方向。
 *
 * 调试顺序建议：
 * 1. 先用 pwm 100 0 测 A 电机方向。
 * 2. 再手转轮子，看编码器 total 是否随正方向增加。
 * 3. 电机方向和编码器方向分别调，不要混在一起猜。
 */
#define ROBOT_PWM_MAX                         1000
#define ROBOT_CONTROL_PERIOD_MS               10U
#define ROBOT_SENSOR_FAST_PERIOD_MS           100U
#define ROBOT_SENSOR_SLOW_PERIOD_MS           1000U
#define ROBOT_TELEMETRY_PERIOD_MS             500U
#define ROBOT_CMD_TIMEOUT_MS                  700U

/*
 * 调试模式下是否启用运动命令超时保护。
 *
 * 串口助手发送的 pwm/speed 命令通常是一条一条手动发送的，不会像 KICKPI
 * 那样每隔几十毫秒持续发送。如果调试模式也启用 700ms 超时，单条测试命令
 * 会在不到 1 秒后被误判为通信中断，状态进入 FAULT，后续 pwm 0 0 和 speed
 * 命令看起来就像“没有反应”。因此当前调试版关闭该保护，电机由 stop、
 * pwm 0 0 或 clear 命令停止；正式 KICKPI 版本仍然保留超时保护。
 */
#define ROBOT_DEBUG_MOTION_TIMEOUT_ENABLE     0

/*
 * UART1 调试版上电自动电机测试。
 *
 * 1：上电后自动依次测试 A、B 两个电机通道，并在 UART1 打印编码器反馈；
 * 0：关闭自动测试，恢复完全手动的 pwm/speed 命令调试方式。
 *
 * 自动测试只适合当前底板调试阶段。测试时必须让车轮悬空，避免上电后小车
 * 自己移动。收到任意 pwm、speed、stop 或 clear 命令后，自动测试会取消。
 */
#define ROBOT_MOTOR_AUTO_TEST_ENABLE           1
#define ROBOT_MOTOR_AUTO_TEST_PWM              100
#define ROBOT_MOTOR_AUTO_TEST_START_DELAY_MS   1000U
#define ROBOT_MOTOR_AUTO_TEST_RUN_MS           3000U
#define ROBOT_MOTOR_AUTO_TEST_PAUSE_MS         1000U
#define ROBOT_MOTOR_AUTO_TEST_MIN_ENCODER_DELTA 5

#define ROBOT_WHEEL_DIAMETER_MM               65
#define ROBOT_ENCODER_COUNTS_PER_REV          1320

#define ROBOT_MOTOR_A_INVERT                  0
#define ROBOT_MOTOR_B_INVERT                  0
#define ROBOT_ENCODER_A_INVERT                0
#define ROBOT_ENCODER_B_INVERT                0

/*
 * INA219 当前只测电池电压，不测大电流。
 *
 * LOW：进入低电压故障的阈值。
 * RECOVER：清除低电压故障的恢复阈值。
 *
 * 恢复阈值比故障阈值高，是为了避免电池电压在临界点附近抖动时，故障状态
 * 一会儿出现、一会儿消失。
 */
#define ROBOT_BATTERY_LOW_MV                  10000U
#define ROBOT_BATTERY_RECOVER_MV              10800U

#define ROBOT_FAN_ON_C_X100                   4500
#define ROBOT_FAN_OFF_C_X100                  4000
#define ROBOT_TEMP_FAULT_C_X100               7000

/*
 * 闭环速度模式参数。
 *
 * 这几个值只是能让闭环先跑起来的保守起点，不是最终调参结果。第一次测试
 * 时建议优先使用 pwm 开环命令；确认电机方向、编码器方向、轮径和每圈计数
 * 都正确后，再用 speed 命令慢慢调这些参数。
 */
#define ROBOT_SPEED_FF_PWM_PER_MM_S_X100      150
#define ROBOT_SPEED_KP_X100                   80
#define ROBOT_SPEED_KI_X100                   3
#define ROBOT_SPEED_INTEGRAL_LIMIT            30000

/*
 * 堵转检测。
 *
 * 当前保持关闭。原因是编码器方向、电机方向、减速比、轮子是否悬空都会影响
 * 判断，过早打开容易误报。等基础运动调通后，再根据实际电机电流和编码器
 * 变化决定是否打开。
 */
#define ROBOT_STALL_DETECT_ENABLE             0
#define ROBOT_STALL_PWM_THRESHOLD             800
#define ROBOT_STALL_DELTA_THRESHOLD           2
#define ROBOT_STALL_TIME_MS                   600U

#define INA219_I2C_ADDR                       0x40U
#define DHT30_I2C_ADDR                        0x38U
#define MPU6050_I2C_ADDR                      0x68U

#endif
