#ifndef ROBOT_COMM_H
#define ROBOT_COMM_H

#include <stdint.h>
#include "stm32f1xx_hal.h"

#define ROBOT_COMM_FRAME_HEAD0                0xAAU
#define ROBOT_COMM_FRAME_HEAD1                0x55U
#define ROBOT_COMM_VERSION                    0x01U
#define ROBOT_COMM_MAX_PAYLOAD                64U

#define ROBOT_CMD_SET_MOTION                  0x01U
#define ROBOT_CMD_SET_STATE                   0x02U
#define ROBOT_CMD_GET_STATUS                  0x03U
#define ROBOT_CMD_SET_OUTPUT                  0x04U
#define ROBOT_CMD_ESTOP                       0x05U
#define ROBOT_CMD_STATUS                      0x81U

#define ROBOT_MOTION_MODE_PWM                 0U
#define ROBOT_MOTION_MODE_SPEED               1U

#define ROBOT_STATE_CMD_IDLE                  0U
#define ROBOT_STATE_CMD_ENABLE                1U
#define ROBOT_STATE_CMD_CLEAR_FAULT           2U

typedef struct
{
  uint8_t version;
  uint8_t command;
  uint8_t length;
  uint8_t payload[ROBOT_COMM_MAX_PAYLOAD];
} RobotCommFrame_t;

typedef void (*RobotCommFrameHandler_t)(const RobotCommFrame_t *frame);
typedef void (*RobotCommDebugLineHandler_t)(const char *line);

/*
 * 初始化 USART1 接收。
 *
 * 这里使用 1 字节中断接收，每收到一个字节就放进环形缓冲区。真正的解析在
 * FreeRTOS 控制任务中完成，避免中断里做复杂逻辑。
 */
void RobotComm_Init(void);

/*
 * 设置二进制协议帧回调。
 *
 * 只在 ROBOT_UART1_DEBUG_ONLY=0 的正式 KICKPI 模式下有效。调试模式下调用
 * 这个函数不会产生实际效果。
 */
void RobotComm_SetFrameHandler(RobotCommFrameHandler_t handler);

/*
 * 设置文本调试命令回调。
 *
 * 只在 ROBOT_UART1_DEBUG_ONLY=1 的底板调试模式下使用。每收到一行完整文本，
 * robot_comm.c 会把这一行交给上层处理。
 */
void RobotComm_SetDebugLineHandler(RobotCommDebugLineHandler_t handler);

/*
 * 处理 UART1 接收缓冲区。
 *
 * 控制任务会周期性调用这个函数。调试模式下它解析一行文本命令；正式模式下
 * 它解析 AA 55 开头的二进制帧并检查 CRC。
 */
void RobotComm_ProcessRx(void);

/*
 * 发送 KICKPI 二进制协议帧。
 *
 * 调试模式下该函数故意返回 HAL_ERROR，不向串口输出二进制数据，防止串口助手
 * 里混入乱码。正式模式下用于发送 0x81 状态帧。
 */
HAL_StatusTypeDef RobotComm_SendFrame(uint8_t command, const uint8_t *payload, uint8_t length);

/*
 * 通过 UART1 打印调试文本。
 *
 * 用法类似 printf。当前用于状态输出和文本命令反馈。注意不要打印太长太频繁，
 * 因为阻塞式 UART 发送会占用控制任务时间。
 */
HAL_StatusTypeDef RobotComm_DebugPrintf(const char *format, ...);

/*
 * 获取 UART1 接收环形缓冲区溢出次数。
 *
 * 如果这个值持续增加，说明串口输入太快或控制任务处理不及时。
 */
uint32_t RobotComm_GetRxOverflowCount(void);

/*
 * 计算 CRC16-CCITT。
 *
 * 参数：
 *   data：待计算的数据指针。
 *   length：数据长度。
 *   seed：初值，协议里使用 0xFFFF。
 */
uint16_t RobotComm_Crc16(const uint8_t *data, uint16_t length, uint16_t seed);

#endif
