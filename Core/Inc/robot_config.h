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
#define ROBOT_UART1_DEBUG_ONLY                0

/*
 * 本版本的附加外设开关。
 *
 * 正式运行版本使用 USART1 接收 KICKPI 的二进制协议，使用 USART2 接收蓝牙
 * 模块的文本协议。网页可以通过 KICKPI 控制继电器和蜂鸣器。
 */
#define ROBOT_RELAY_ENABLE                    1
#define ROBOT_BLUETOOTH_ENABLE                1

/*
 * 独立看门狗开关。
 *
 * 0：当前工程不调用 IWDG。
 * 1：工程必须已经在 CubeMX 中生成 iwdg.c/iwdg.h；程序启动后由
 *    10ms 控制任务定期刷新看门狗。只要控制任务、调度器或关键路径长时间
 *    卡住，约 1 秒内就会自动复位，避免电机长期保持危险输出。
 *
 * 当前工程已经生成 IWDG 文件并打开该开关。开启看门狗后，必须重新编译和
 * 烧录，不能只改 CubeMX 配置不更新固件。
 */
#define ROBOT_IWDG_ENABLE                     1

/*
 * 蜂鸣器总开关。
 *
 * 0：禁止蜂鸣器输出。
 *
 * 1：允许网页、KICKPI 或蓝牙命令手动控制蜂鸣器。上电默认关闭。
 *
 * 这里打开是为了让网页开关真正可用；故障自动报警由下面的独立开关控制，
 * 因此不会因为底板上电或已有故障码而自动鸣叫。
 */
#define ROBOT_BUZZER_ENABLE                   1

/*
 * 蜂鸣器故障自动报警开关。
 *
 * 0：只响应明确的手动开关命令，适合当前联调阶段，默认静音。
 * 1：检测到故障时按 200ms 周期自动间歇鸣叫。
 */
#define ROBOT_BUZZER_FAULT_ALARM_ENABLE       0

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
 * 蓝牙控制租约超时时间。
 *
 * 手机 App 按住方向键时会周期性刷新运动命令。如果蓝牙断开、手机退出 App
 * 或者方向键事件异常结束，超过这个时间没有刷新就自动停车并释放控制权。
 * 当前 App 按 100ms 刷新，因此 800ms 能留出通信余量，同时不会让断线后
 * 的小车继续运动太久。
 */
#define ROBOT_BLUETOOTH_OWNER_TIMEOUT_MS      800U

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
#define ROBOT_MOTOR_AUTO_TEST_ENABLE           0
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
