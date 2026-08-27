#include "robot_comm.h"
#include "robot_config.h"
#include "usart.h"
#include <stdarg.h>
#include <stdio.h>
#include <string.h>

#define ROBOT_COMM_RX_RING_SIZE 128U
#define ROBOT_DEBUG_LINE_SIZE    96U

/*
 * UART1 通信模块的整体说明。
 *
 * 中断里只做两件事：把收到的字节保存到环形缓冲区，并重新开启下一次接收。
 * 复杂的文本解析或二进制帧解析都放在 FreeRTOS 控制任务里完成，这样可以避免
 * 在中断中做耗时操作，也能让电机控制周期更加稳定。
 */
static uint8_t s_rx_byte;
static volatile uint16_t s_rx_head;
static volatile uint16_t s_rx_tail;
static volatile uint32_t s_rx_overflow;
static uint8_t s_rx_ring[ROBOT_COMM_RX_RING_SIZE];
#if !ROBOT_UART1_DEBUG_ONLY
static RobotCommFrameHandler_t s_frame_handler;
#endif
static RobotCommDebugLineHandler_t s_debug_line_handler;

#if ROBOT_UART1_DEBUG_ONLY
static char s_debug_line[ROBOT_DEBUG_LINE_SIZE];
static uint8_t s_debug_line_pos;
#else
typedef enum
{
  PARSE_WAIT_HEAD0 = 0,
  PARSE_WAIT_HEAD1,
  PARSE_VERSION,
  PARSE_COMMAND,
  PARSE_LENGTH,
  PARSE_PAYLOAD,
  PARSE_CRC_LO,
  PARSE_CRC_HI
} ParserState_t;

static ParserState_t s_parser_state = PARSE_WAIT_HEAD0;
static RobotCommFrame_t s_parser_frame;
static uint8_t s_parser_index;
static uint8_t s_crc_lo;
#endif

/*
 * 在串口中断上下文中保存一个接收到的字节。
 *
 * 参数 byte：
 *   UART1 刚刚接收到的 1 个字节。
 *
 * 如果缓冲区已经满了，本函数不会等待，也不会覆盖旧数据，只增加溢出计数。
 * 这样即使上位机发送过快，也不会把串口中断卡住。
 */
static void rx_push_from_isr(uint8_t byte)
{
  uint16_t next = (uint16_t)((s_rx_head + 1U) % ROBOT_COMM_RX_RING_SIZE);

  if (next == s_rx_tail)
  {
    s_rx_overflow++;
    return;
  }

  s_rx_ring[s_rx_head] = byte;
  s_rx_head = next;
}

/*
 * 从接收环形缓冲区取出 1 个字节。
 *
 * 返回值：
 *   1：成功取出 1 个字节，并写入 byte。
 *   0：缓冲区为空，当前没有待处理数据。
 */
static uint8_t rx_pop(uint8_t *byte)
{
  if (s_rx_tail == s_rx_head)
  {
    return 0U;
  }

  *byte = s_rx_ring[s_rx_tail];
  s_rx_tail = (uint16_t)((s_rx_tail + 1U) % ROBOT_COMM_RX_RING_SIZE);
  return 1U;
}

/*
 * 计算 CRC16-CCITT 校验值。
 *
 * 正式 KICKPI 模式下，发送和接收的二进制帧都使用这个函数。当前虽然处于
 * UART1 文本调试模式，但保留它是为了以后切换回正式协议时不需要重写通信层。
 */
uint16_t RobotComm_Crc16(const uint8_t *data, uint16_t length, uint16_t seed)
{
  uint16_t crc = seed;
  uint16_t i;

  while (length-- != 0U)
  {
    crc ^= (uint16_t)(*data++) << 8;
    for (i = 0U; i < 8U; i++)
    {
      if ((crc & 0x8000U) != 0U)
      {
        crc = (uint16_t)((crc << 1) ^ 0x1021U);
      }
      else
      {
        crc <<= 1;
      }
    }
  }

  return crc;
}

#if !ROBOT_UART1_DEBUG_ONLY
/*
 * 将正式模式的二进制协议解析状态恢复到等待帧头。
 *
 * 当收到错误版本、非法长度或 CRC 校验失败时，解析器会回到这里重新寻找
 * 下一组 AA 55 帧头，避免一帧坏数据影响后面的所有数据。
 */
static void parser_reset(void)
{
  s_parser_state = PARSE_WAIT_HEAD0;
  s_parser_index = 0U;
}

/*
 * 正式 KICKPI 模式下解析 1 个二进制字节。
 *
 * 本函数按照“帧头 -> 版本 -> 命令 -> 长度 -> payload -> CRC”的顺序推进状态机。
 * 当一帧完整且 CRC 正确时，才调用上层注册的 s_frame_handler。
 */
static void parser_accept(uint8_t byte)
{
  uint8_t header[3];
  uint16_t crc_calc;
  uint16_t crc_rx;

  switch (s_parser_state)
  {
    case PARSE_WAIT_HEAD0:
      if (byte == ROBOT_COMM_FRAME_HEAD0)
      {
        s_parser_state = PARSE_WAIT_HEAD1;
      }
      break;

    case PARSE_WAIT_HEAD1:
      if (byte == ROBOT_COMM_FRAME_HEAD1)
      {
        s_parser_state = PARSE_VERSION;
      }
      else
      {
        parser_reset();
      }
      break;

    case PARSE_VERSION:
      s_parser_frame.version = byte;
      if (byte == ROBOT_COMM_VERSION)
      {
        s_parser_state = PARSE_COMMAND;
      }
      else
      {
        parser_reset();
      }
      break;

    case PARSE_COMMAND:
      s_parser_frame.command = byte;
      s_parser_state = PARSE_LENGTH;
      break;

    case PARSE_LENGTH:
      s_parser_frame.length = byte;
      s_parser_index = 0U;
      if (byte > ROBOT_COMM_MAX_PAYLOAD)
      {
        parser_reset();
      }
      else if (byte == 0U)
      {
        s_parser_state = PARSE_CRC_LO;
      }
      else
      {
        s_parser_state = PARSE_PAYLOAD;
      }
      break;

    case PARSE_PAYLOAD:
      s_parser_frame.payload[s_parser_index++] = byte;
      if (s_parser_index >= s_parser_frame.length)
      {
        s_parser_state = PARSE_CRC_LO;
      }
      break;

    case PARSE_CRC_LO:
      s_crc_lo = byte;
      s_parser_state = PARSE_CRC_HI;
      break;

    case PARSE_CRC_HI:
      header[0] = s_parser_frame.version;
      header[1] = s_parser_frame.command;
      header[2] = s_parser_frame.length;
      crc_calc = RobotComm_Crc16(header, sizeof(header), 0xFFFFU);
      crc_calc = RobotComm_Crc16(s_parser_frame.payload, s_parser_frame.length, crc_calc);
      crc_rx = (uint16_t)s_crc_lo | ((uint16_t)byte << 8);

      if (crc_calc == crc_rx && s_frame_handler != 0)
      {
        s_frame_handler(&s_parser_frame);
      }
      parser_reset();
      break;

    default:
      parser_reset();
      break;
  }
}
#endif

#if ROBOT_UART1_DEBUG_ONLY
/*
 * 调试模式下解析一行 ASCII 文本命令。
 *
 * 一行以换行结束，支持退格键删除输入字符。命令长度限制在 95 个字符以内，
 * 足够输入 pwm、speed、relay、status 等底板测试命令。
 */
static void debug_accept(uint8_t byte)
{
  if (byte == '\r')
  {
    return;
  }

  if (byte == '\n')
  {
    s_debug_line[s_debug_line_pos] = '\0';
    if (s_debug_line_pos != 0U && s_debug_line_handler != 0)
    {
      s_debug_line_handler(s_debug_line);
    }
    s_debug_line_pos = 0U;
    s_debug_line[0] = '\0';
    return;
  }

  if (byte == '\b' || byte == 0x7FU)
  {
    if (s_debug_line_pos != 0U)
    {
      s_debug_line_pos--;
      s_debug_line[s_debug_line_pos] = '\0';
    }
    return;
  }

  if (s_debug_line_pos < (ROBOT_DEBUG_LINE_SIZE - 1U))
  {
    s_debug_line[s_debug_line_pos++] = (char)byte;
    s_debug_line[s_debug_line_pos] = '\0';
  }
}
#endif

/*
 * 初始化 UART1 接收链路。
 *
 * 这里清空接收环形缓冲区和解析状态，并启动 1 字节中断接收。以后每收到
 * 一个字节，HAL_UART_RxCpltCallback 会自动把接收重新挂上。
 */
void RobotComm_Init(void)
{
  s_rx_head = 0U;
  s_rx_tail = 0U;
  s_rx_overflow = 0U;
#if ROBOT_UART1_DEBUG_ONLY
  s_debug_line_pos = 0U;
  s_debug_line[0] = '\0';
#else
  parser_reset();
#endif
  HAL_UART_Receive_IT(&huart1, &s_rx_byte, 1U);
}

/*
 * 注册正式模式的二进制帧处理函数。
 *
 * 只有 ROBOT_UART1_DEBUG_ONLY=0 时才有效。调试模式下为了避免误把文本串口
 * 当成 KICKPI 协议，这个函数会忽略传入的回调。
 */
void RobotComm_SetFrameHandler(RobotCommFrameHandler_t handler)
{
#if ROBOT_UART1_DEBUG_ONLY
  (void)handler;
#else
  s_frame_handler = handler;
#endif
}

/*
 * 注册调试模式的文本命令处理函数。
 *
 * UART1 收到换行后，debug_accept 会把完整字符串交给这个回调。robot_app.c
 * 在这里注册命令处理函数，完成 pwm、stop、relay 等实际动作。
 */
void RobotComm_SetDebugLineHandler(RobotCommDebugLineHandler_t handler)
{
  s_debug_line_handler = handler;
}

/*
 * 处理接收缓冲区中的所有待处理字节。
 *
 * 该函数应该由 FreeRTOS 控制任务周期性调用。它不在 UART 中断里直接解析，
 * 这样既能缩短中断时间，也能让命令处理和电机状态更新运行在同一个任务上下文。
 */
void RobotComm_ProcessRx(void)
{
  uint8_t byte;

  while (rx_pop(&byte) != 0U)
  {
#if ROBOT_UART1_DEBUG_ONLY
    debug_accept(byte);
#else
    parser_accept(byte);
#endif
  }
}

/*
 * 发送正式 KICKPI 模式的二进制通信帧。
 *
 * 调试模式下本函数故意返回 HAL_ERROR，不向 UART1 发任何二进制数据，避免
 * 串口助手看到乱码。正式模式下才会拼接帧头、长度、payload 和 CRC16。
 */
HAL_StatusTypeDef RobotComm_SendFrame(uint8_t command, const uint8_t *payload, uint8_t length)
{
#if ROBOT_UART1_DEBUG_ONLY
  /*
   * 调试模式下故意关闭二进制帧发送。上层应当使用 RobotComm_DebugPrintf，
   * 这样串口助手里只会看到可读文本，不会混入二进制乱码。
   */
  (void)command;
  (void)payload;
  (void)length;
  return HAL_ERROR;
#else
  uint8_t frame[2U + 3U + ROBOT_COMM_MAX_PAYLOAD + 2U];
  uint8_t pos = 0U;
  uint8_t header[3];
  uint16_t crc;

  if (length > ROBOT_COMM_MAX_PAYLOAD)
  {
    return HAL_ERROR;
  }

  frame[pos++] = ROBOT_COMM_FRAME_HEAD0;
  frame[pos++] = ROBOT_COMM_FRAME_HEAD1;
  frame[pos++] = ROBOT_COMM_VERSION;
  frame[pos++] = command;
  frame[pos++] = length;

  if (length != 0U && payload != 0)
  {
    memcpy(&frame[pos], payload, length);
    pos = (uint8_t)(pos + length);
  }

  header[0] = ROBOT_COMM_VERSION;
  header[1] = command;
  header[2] = length;
  crc = RobotComm_Crc16(header, sizeof(header), 0xFFFFU);
  crc = RobotComm_Crc16(payload, length, crc);

  frame[pos++] = (uint8_t)(crc & 0xFFU);
  frame[pos++] = (uint8_t)(crc >> 8);

  return HAL_UART_Transmit(&huart1, frame, pos, 50U);
#endif
}

/*
 * 用 printf 风格向 UART1 输出调试文本。
 *
 * 参数 format 和后面的参数与 printf 类似。当前调试状态、传感器数据和命令
 * 执行结果都通过这个函数输出。它使用阻塞式发送，因此单条日志不要写得过长。
 */
HAL_StatusTypeDef RobotComm_DebugPrintf(const char *format, ...)
{
  char buffer[384];
  va_list args;
  int length;

  /*
   * 调试模式下这个函数会被遥测任务和控制任务里的命令处理函数调用。日志行
   * 不要写得过长，否则阻塞式串口发送会占用 10ms 电机控制周期太久。
   */
  va_start(args, format);
  length = vsnprintf(buffer, sizeof(buffer), format, args);
  va_end(args);

  if (length < 0)
  {
    return HAL_ERROR;
  }
  if (length >= (int)sizeof(buffer))
  {
    length = (int)sizeof(buffer) - 1;
  }

  return HAL_UART_Transmit(&huart1, (uint8_t *)buffer, (uint16_t)length, 100U);
}

/*
 * 获取 UART1 接收环形缓冲区的溢出次数。
 *
 * 如果这个数持续增加，说明串口发送端过快，或者控制任务没有及时处理接收
 * 缓冲区。调试时可以在状态输出里观察这个值。
 */
uint32_t RobotComm_GetRxOverflowCount(void)
{
  return s_rx_overflow;
}

/*
 * UART1 单字节接收完成回调。
 *
 * HAL 收到一个字节后进入这里，本函数先把字节放入环形缓冲区，然后马上启动
 * 下一次 1 字节接收。不要在这个中断回调里做传感器读取或电机控制。
 */
void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart)
{
  if (huart->Instance == USART1)
  {
    rx_push_from_isr(s_rx_byte);
    HAL_UART_Receive_IT(&huart1, &s_rx_byte, 1U);
  }
}

/*
 * UART1 错误回调。
 *
 * 发生偶发噪声、帧错误或溢出后，重新启动 1 字节接收，防止 UART1 接收链路
 * 因一次异常而永久停住。
 */
void HAL_UART_ErrorCallback(UART_HandleTypeDef *huart)
{
  if (huart->Instance == USART1)
  {
    HAL_UART_Receive_IT(&huart1, &s_rx_byte, 1U);
  }
}
