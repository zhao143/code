#include "robot_bluetooth.h"
#include "usart.h"
#include <stdarg.h>
#include <stdio.h>
#include <string.h>

#define ROBOT_BLUETOOTH_RX_RING_SIZE 128U
#define ROBOT_BLUETOOTH_LINE_SIZE     96U

/*
 * 蓝牙模块使用 USART2 的 9600 8N1 配置。中断只把字节放入环形缓冲区，
 * 文本解析在控制任务中完成，这样蓝牙数据不会直接打断 10ms 电机控制流程。
 */
static volatile uint16_t s_rx_head;
static volatile uint16_t s_rx_tail;
static volatile uint32_t s_rx_overflow;
static uint8_t s_rx_ring[ROBOT_BLUETOOTH_RX_RING_SIZE];
static char s_line[ROBOT_BLUETOOTH_LINE_SIZE];
static uint8_t s_line_pos;
static RobotBluetoothLineHandler_t s_line_handler;

/*
 * 在中断上下文中压入 1 个字节。
 *
 * 缓冲区满时丢弃新字节并记录溢出次数。控制命令较短，正常使用时不会达到
 * 这个上限；保留计数便于后续出现蓝牙串口异常时排查。
 */
static void rx_push_from_isr(uint8_t byte)
{
  uint16_t next = (uint16_t)((s_rx_head + 1U) % ROBOT_BLUETOOTH_RX_RING_SIZE);

  if (next == s_rx_tail)
  {
    s_rx_overflow++;
    return;
  }

  s_rx_ring[s_rx_head] = byte;
  s_rx_head = next;
}

/*
 * 从环形缓冲区取出 1 个字节。
 */
static uint8_t rx_pop(uint8_t *byte)
{
  if (s_rx_tail == s_rx_head)
  {
    return 0U;
  }

  *byte = s_rx_ring[s_rx_tail];
  s_rx_tail = (uint16_t)((s_rx_tail + 1U) % ROBOT_BLUETOOTH_RX_RING_SIZE);
  return 1U;
}

/*
 * 处理一个蓝牙字节并组装成一行文本。
 *
 * 兼容蓝牙助手常见的 CR、LF、CRLF 发送方式。收到换行后才调用上层回调，
 * 因此上层总能拿到一条完整命令。
 */
static void accept_byte(uint8_t byte)
{
  if (byte == '\r')
  {
    return;
  }

  if (byte == '\n')
  {
    s_line[s_line_pos] = '\0';
    if (s_line_pos != 0U && s_line_handler != 0)
    {
      s_line_handler(s_line);
    }
    s_line_pos = 0U;
    s_line[0] = '\0';
    return;
  }

  if (byte == '\b' || byte == 0x7FU)
  {
    if (s_line_pos != 0U)
    {
      s_line_pos--;
      s_line[s_line_pos] = '\0';
    }
    return;
  }

  if (s_line_pos < (ROBOT_BLUETOOTH_LINE_SIZE - 1U))
  {
    s_line[s_line_pos++] = (char)byte;
    s_line[s_line_pos] = '\0';
  }
}

/*
 * 清空软件接收状态。USART2 的硬件接收中断由 robot_comm.c 统一重新挂接。
 */
void RobotBluetooth_Init(void)
{
  s_rx_head = 0U;
  s_rx_tail = 0U;
  s_rx_overflow = 0U;
  s_line_pos = 0U;
  s_line[0] = '\0';
  s_line_handler = 0;
}

/*
 * USART2 中断回调使用的最小入口。
 */
void RobotBluetooth_OnRxByteFromIsr(uint8_t byte)
{
  rx_push_from_isr(byte);
}

/*
 * 在控制任务中解析一批待处理字节。
 *
 * 每次调用限制处理量，避免蓝牙持续输入时长期占用高优先级控制任务。
 */
void RobotBluetooth_ProcessRx(void)
{
  uint8_t byte;
  uint16_t budget = ROBOT_BLUETOOTH_PROCESS_BUDGET;

  while (budget-- != 0U && rx_pop(&byte) != 0U)
  {
    accept_byte(byte);
  }
}

/*
 * 注册蓝牙文本命令回调。
 */
void RobotBluetooth_SetLineHandler(RobotBluetoothLineHandler_t handler)
{
  s_line_handler = handler;
}

/*
 * 向蓝牙串口发送一段文本。
 *
 * 蓝牙模块通常把收到的字节原样转发到手机，因此这里不添加隐式换行，调用
 * 者可以自行决定反馈是一行还是多行。
 */
HAL_StatusTypeDef RobotBluetooth_SendText(const char *text)
{
  if (text == 0)
  {
    return HAL_ERROR;
  }

  return HAL_UART_Transmit(&huart2,
                           (uint8_t *)text,
                           (uint16_t)strlen(text),
                           100U);
}

/*
 * 以 printf 风格格式化并发送蓝牙反馈。
 */
HAL_StatusTypeDef RobotBluetooth_Sendf(const char *format, ...)
{
  char buffer[256];
  va_list args;
  int length;

  if (format == 0)
  {
    return HAL_ERROR;
  }

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

  return HAL_UART_Transmit(&huart2, (uint8_t *)buffer, (uint16_t)length, 100U);
}
