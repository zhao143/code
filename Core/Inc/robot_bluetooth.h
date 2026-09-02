#ifndef ROBOT_BLUETOOTH_H
#define ROBOT_BLUETOOTH_H

#include <stdint.h>
#include "stm32f1xx_hal.h"

/*
 * 单次控制任务调用最多解析的蓝牙字节数。
 *
 * 蓝牙模块也是中断接收、任务解析。限制每次处理量可以避免持续输入
 * 长时间占用控制任务，保证 LED、遥测和电机控制周期仍能运行。
 */
#define ROBOT_BLUETOOTH_PROCESS_BUDGET 32U

typedef void (*RobotBluetoothLineHandler_t)(const char *line);

/*
 * 初始化蓝牙串口文本接收缓冲区。
 *
 * USART2 的硬件初始化由 CubeMX 生成的 MX_USART2_UART_Init 完成；本模块负责
 * 接收中断后的字节缓存、换行组包和发送蓝牙端的文本响应。
 */
void RobotBluetooth_Init(void);

/*
 * 在 USART2 接收中断回调中保存一个字节。
 *
 * 该函数只做非常短的环形缓冲操作，不能在中断中调用控制、电机或传感器逻辑。
 */
void RobotBluetooth_OnRxByteFromIsr(uint8_t byte);

/*
 * 在 FreeRTOS 控制任务中处理蓝牙接收缓冲区。
 */
void RobotBluetooth_ProcessRx(void);

/*
 * 注册一行文本命令的处理函数。
 */
void RobotBluetooth_SetLineHandler(RobotBluetoothLineHandler_t handler);

/*
 * 向蓝牙模块发送已经格式化好的文本。
 */
HAL_StatusTypeDef RobotBluetooth_SendText(const char *text);

/*
 * 以 printf 风格向蓝牙模块发送一行文本。
 */
HAL_StatusTypeDef RobotBluetooth_Sendf(const char *format, ...);

#endif
