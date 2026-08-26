#include "robot_comm.h"
#include "usart.h"
#include <string.h>

#define ROBOT_COMM_RX_RING_SIZE 128U

static uint8_t s_rx_byte;
static volatile uint16_t s_rx_head;
static volatile uint16_t s_rx_tail;
static volatile uint32_t s_rx_overflow;
static uint8_t s_rx_ring[ROBOT_COMM_RX_RING_SIZE];
static RobotCommFrameHandler_t s_frame_handler;

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

static void parser_reset(void)
{
  s_parser_state = PARSE_WAIT_HEAD0;
  s_parser_index = 0U;
}

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

void RobotComm_Init(void)
{
  s_rx_head = 0U;
  s_rx_tail = 0U;
  s_rx_overflow = 0U;
  parser_reset();
  HAL_UART_Receive_IT(&huart1, &s_rx_byte, 1U);
}

void RobotComm_SetFrameHandler(RobotCommFrameHandler_t handler)
{
  s_frame_handler = handler;
}

void RobotComm_ProcessRx(void)
{
  uint8_t byte;

  while (rx_pop(&byte) != 0U)
  {
    parser_accept(byte);
  }
}

HAL_StatusTypeDef RobotComm_SendFrame(uint8_t command, const uint8_t *payload, uint8_t length)
{
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
}

uint32_t RobotComm_GetRxOverflowCount(void)
{
  return s_rx_overflow;
}

void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart)
{
  if (huart->Instance == USART1)
  {
    rx_push_from_isr(s_rx_byte);
    HAL_UART_Receive_IT(&huart1, &s_rx_byte, 1U);
  }
}

void HAL_UART_ErrorCallback(UART_HandleTypeDef *huart)
{
  if (huart->Instance == USART1)
  {
    HAL_UART_Receive_IT(&huart1, &s_rx_byte, 1U);
  }
}
