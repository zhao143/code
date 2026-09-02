#include "robot_app.h"
#include "main.h"
#include "robot_bluetooth.h"
#include "robot_comm.h"
#include "robot_config.h"
#include "robot_delay.h"
#include "robot_encoder.h"
#include "robot_motor.h"
#include "robot_sensors.h"
#include "cmsis_os.h"
#if ROBOT_IWDG_ENABLE
#include "iwdg.h"
#endif
#include <stdio.h>
#include <string.h>

/*
 * 机器人底层运行状态的总结构体。
 *
 * 控制任务、传感器任务、遥测任务都可能读写这里的数据，所以任务之间访问
 * 时使用 s_ctx_mutex 互斥保护。串口接收中断只负责收字节，不直接访问这个
 * 结构体，避免中断和任务同时改数据。
 */
typedef struct
{
  RobotAppState_t state;
  uint16_t faults;
  uint8_t motion_mode;
  RobotControlSource_t control_owner;
  int16_t target_a;
  int16_t target_b;
  int16_t pwm_a;
  int16_t pwm_b;
  int32_t integ_a;
  int32_t integ_b;
  uint32_t last_cmd_ms;
  uint32_t last_motion_cmd_ms;
  uint32_t stall_a_start_ms;
  uint32_t stall_b_start_ms;
  uint8_t command_seen;
  uint8_t telemetry_request;
  uint8_t manual_buzzer;
  uint8_t manual_fan;
  uint8_t auto_fan;
  uint8_t relay_test_on;
  uint32_t relay_test_last_ms;
  uint8_t buzzer_on;
#if ROBOT_UART1_DEBUG_ONLY && ROBOT_MOTOR_AUTO_TEST_ENABLE
  uint8_t auto_test_active;
  uint8_t auto_test_stage;
  uint8_t auto_test_a_pass;
  uint8_t auto_test_b_pass;
  uint32_t auto_test_stage_start_ms;
  int32_t auto_test_encoder_baseline_a;
  int32_t auto_test_encoder_baseline_b;
#endif
  RobotEncoderData_t encoder;
  RobotSensorsData_t sensors;
} RobotAppContext_t;

static RobotAppContext_t s_ctx;
static osMutexId_t s_ctx_mutex;

static const osThreadAttr_t s_control_attr =
{
  .name = "ctrlTask",
  .stack_size = 384 * 4,
  .priority = (osPriority_t)osPriorityAboveNormal,
};

static const osThreadAttr_t s_sensor_attr =
{
  .name = "sensorTask",
  .stack_size = 384 * 4,
  .priority = (osPriority_t)osPriorityNormal,
};

static const osThreadAttr_t s_telemetry_attr =
{
  .name = "telemetryTask",
  .stack_size = 384 * 4,
  .priority = (osPriority_t)osPriorityBelowNormal,
};

static const osMutexAttr_t s_ctx_mutex_attr =
{
  .name = "robotCtx"
};

static void RobotApp_ControlTask(void *argument);
static void RobotApp_SensorTask(void *argument);
static void RobotApp_TelemetryTask(void *argument);
static void send_status(void);
#if ROBOT_UART1_DEBUG_ONLY && ROBOT_MOTOR_AUTO_TEST_ENABLE
static void auto_test_cancel(void);
static void auto_test_step(uint32_t now);
#endif
#if ROBOT_UART1_DEBUG_ONLY
static void RobotApp_OnDebugLine(const char *line);
#else
static void RobotApp_OnFrame(const RobotCommFrame_t *frame);
#endif

/*
 * 将一个带符号的 PWM 命令限制在允许范围内。
 *
 * 这里使用 -1000 到 1000 表示 -100% 到 100%。这个函数既用于串口调试命令，
 * 也用于闭环控制输出，防止异常参数把 PWM 推出预定范围。
 */
static int16_t clamp_pwm_i32(int32_t value)
{
  if (value > ROBOT_PWM_MAX)
  {
    return ROBOT_PWM_MAX;
  }
  if (value < -ROBOT_PWM_MAX)
  {
    return -ROBOT_PWM_MAX;
  }
  return (int16_t)value;
}

/*
 * 将一个 32 位整数限制在指定的最小值和最大值之间。
 *
 * 主要用于积分项、速度目标等不适合直接使用 PWM 限幅的数值，避免控制器
 * 因为异常输入或长时间误差而发生整数溢出。
 */
static int32_t clamp_i32(int32_t value, int32_t min_value, int32_t max_value)
{
  if (value > max_value)
  {
    return max_value;
  }
  if (value < min_value)
  {
    return min_value;
  }
  return value;
}

#if ROBOT_UART1_DEBUG_ONLY && ROBOT_MOTOR_AUTO_TEST_ENABLE
/*
 * 上电自动测试的阶段编号。
 *
 * DELAY 给调试人员留出确认车轮悬空的时间；A_RUN 和 B_RUN 只驱动一个电机
 * 通道；两个 PAUSE 阶段在切换通道前彻底停机；DONE 表示测试完成，之后恢复
 * 正常的手动串口控制。
 */
typedef enum
{
  ROBOT_AUTO_TEST_DELAY = 0,
  ROBOT_AUTO_TEST_A_RUN,
  ROBOT_AUTO_TEST_A_PAUSE,
  ROBOT_AUTO_TEST_B_RUN,
  ROBOT_AUTO_TEST_B_PAUSE,
  ROBOT_AUTO_TEST_DONE
} RobotAutoTestStage_t;

/*
 * 返回 32 位有符号数的绝对值。
 *
 * 自动测试只运行几秒，编码器累计值不会接近 int32_t 的边界，因此这里只用
 * 简单的绝对值来比较测试前后的计数变化。
 */
static int32_t auto_test_abs_i32(int32_t value)
{
  return (value < 0) ? -value : value;
}

/*
 * 进入自动测试的新阶段，并准备本阶段的目标值和编码器基准值。
 *
 * 目标仍然写入 s_ctx，真正的 GPIO/PWM 输出继续由 control_step() 统一完成，
 * 这样自动测试和手动命令走的是同一套电机控制路径。
 */
static void auto_test_enter_stage(RobotAutoTestStage_t stage, uint32_t now)
{
  int32_t delta;

  s_ctx.auto_test_stage = (uint8_t)stage;
  s_ctx.auto_test_stage_start_ms = now;

  if (stage == ROBOT_AUTO_TEST_DELAY)
  {
    s_ctx.target_a = 0;
    s_ctx.target_b = 0;
    s_ctx.state = ROBOT_APP_STATE_READY;
    return;
  }

  if (stage == ROBOT_AUTO_TEST_A_RUN)
  {
    s_ctx.auto_test_encoder_baseline_a = s_ctx.encoder.total_a;
    s_ctx.target_a = ROBOT_MOTOR_AUTO_TEST_PWM;
    s_ctx.target_b = 0;
    s_ctx.motion_mode = ROBOT_MOTION_MODE_PWM;
    s_ctx.command_seen = 0U;
    s_ctx.state = (s_ctx.faults == 0U) ? ROBOT_APP_STATE_RUN : ROBOT_APP_STATE_FAULT;
    RobotComm_DebugPrintf("AUTO_TEST A_RUN TARGET[A=%d B=0] duration=%lums\r\n",
                          ROBOT_MOTOR_AUTO_TEST_PWM,
                          (unsigned long)ROBOT_MOTOR_AUTO_TEST_RUN_MS);
    return;
  }

  if (stage == ROBOT_AUTO_TEST_A_PAUSE)
  {
    delta = auto_test_abs_i32(s_ctx.encoder.total_a - s_ctx.auto_test_encoder_baseline_a);
    s_ctx.auto_test_a_pass = (delta >= ROBOT_MOTOR_AUTO_TEST_MIN_ENCODER_DELTA) ? 1U : 0U;
    s_ctx.target_a = 0;
    s_ctx.target_b = 0;
    s_ctx.state = ROBOT_APP_STATE_READY;
    RobotComm_DebugPrintf("AUTO_TEST A_RESULT ENC_DELTA=%ld RESULT=%s\r\n",
                          (long)delta,
                          s_ctx.auto_test_a_pass ? "PASS" : "FAIL");
    return;
  }

  if (stage == ROBOT_AUTO_TEST_B_RUN)
  {
    s_ctx.auto_test_encoder_baseline_b = s_ctx.encoder.total_b;
    s_ctx.target_a = 0;
    s_ctx.target_b = ROBOT_MOTOR_AUTO_TEST_PWM;
    s_ctx.motion_mode = ROBOT_MOTION_MODE_PWM;
    s_ctx.command_seen = 0U;
    s_ctx.state = (s_ctx.faults == 0U) ? ROBOT_APP_STATE_RUN : ROBOT_APP_STATE_FAULT;
    RobotComm_DebugPrintf("AUTO_TEST B_RUN TARGET[A=0 B=%d] duration=%lums\r\n",
                          ROBOT_MOTOR_AUTO_TEST_PWM,
                          (unsigned long)ROBOT_MOTOR_AUTO_TEST_RUN_MS);
    return;
  }

  if (stage == ROBOT_AUTO_TEST_B_PAUSE)
  {
    delta = auto_test_abs_i32(s_ctx.encoder.total_b - s_ctx.auto_test_encoder_baseline_b);
    s_ctx.auto_test_b_pass = (delta >= ROBOT_MOTOR_AUTO_TEST_MIN_ENCODER_DELTA) ? 1U : 0U;
    s_ctx.target_a = 0;
    s_ctx.target_b = 0;
    s_ctx.state = ROBOT_APP_STATE_READY;
    RobotComm_DebugPrintf("AUTO_TEST B_RESULT ENC_DELTA=%ld RESULT=%s\r\n",
                          (long)delta,
                          s_ctx.auto_test_b_pass ? "PASS" : "FAIL");
    return;
  }

  s_ctx.target_a = 0;
  s_ctx.target_b = 0;
  s_ctx.state = ROBOT_APP_STATE_READY;
  s_ctx.auto_test_active = 0U;
  RobotComm_DebugPrintf("AUTO_TEST DONE A=%s B=%s\r\n",
                        s_ctx.auto_test_a_pass ? "PASS" : "FAIL",
                        s_ctx.auto_test_b_pass ? "PASS" : "FAIL");
}

/*
 * 取消上电自动测试。
 *
 * 手动发送 pwm、speed、stop 或 clear 时调用，防止自动测试下一周期又把手动
 * 指令覆盖掉。取消后不主动清除已有故障，只把电机目标清零。
 */
static void auto_test_cancel(void)
{
  if (s_ctx.auto_test_active != 0U)
  {
    s_ctx.auto_test_active = 0U;
    s_ctx.auto_test_stage = ROBOT_AUTO_TEST_DONE;
    s_ctx.target_a = 0;
    s_ctx.target_b = 0;
    s_ctx.state = ROBOT_APP_STATE_SAFE_IDLE;
    RobotComm_DebugPrintf("AUTO_TEST CANCELED\r\n");
  }
}

/*
 * 每 10ms 推进一次自动测试状态机。
 *
 * 该函数只改变阶段和目标值，不直接写定时器寄存器；实际输出仍由后面的
 * control_step() 统一完成。编码器增量由阶段开始和结束时的累计值之差得到。
 */
static void auto_test_step(uint32_t now)
{
  uint32_t elapsed;

  if (s_ctx.auto_test_active == 0U)
  {
    return;
  }

  elapsed = (uint32_t)(now - s_ctx.auto_test_stage_start_ms);

  switch ((RobotAutoTestStage_t)s_ctx.auto_test_stage)
  {
    case ROBOT_AUTO_TEST_DELAY:
      if (elapsed >= ROBOT_MOTOR_AUTO_TEST_START_DELAY_MS)
      {
        auto_test_enter_stage(ROBOT_AUTO_TEST_A_RUN, now);
      }
      break;

    case ROBOT_AUTO_TEST_A_RUN:
      if (elapsed >= ROBOT_MOTOR_AUTO_TEST_RUN_MS)
      {
        auto_test_enter_stage(ROBOT_AUTO_TEST_A_PAUSE, now);
      }
      break;

    case ROBOT_AUTO_TEST_A_PAUSE:
      if (elapsed >= ROBOT_MOTOR_AUTO_TEST_PAUSE_MS)
      {
        auto_test_enter_stage(ROBOT_AUTO_TEST_B_RUN, now);
      }
      break;

    case ROBOT_AUTO_TEST_B_RUN:
      if (elapsed >= ROBOT_MOTOR_AUTO_TEST_RUN_MS)
      {
        auto_test_enter_stage(ROBOT_AUTO_TEST_B_PAUSE, now);
      }
      break;

    case ROBOT_AUTO_TEST_B_PAUSE:
      if (elapsed >= ROBOT_MOTOR_AUTO_TEST_PAUSE_MS)
      {
        auto_test_enter_stage(ROBOT_AUTO_TEST_DONE, now);
      }
      break;

    default:
      s_ctx.auto_test_active = 0U;
      break;
  }
}
#endif

#if !ROBOT_UART1_DEBUG_ONLY
/*
 * 从小端字节序读取一个有符号 16 位整数。
 *
 * 正式 KICKPI 协议约定低字节在前。这个函数只在正式二进制通信模式编译进去。
 */
static int16_t read_i16_le(const uint8_t *data)
{
  return (int16_t)((uint16_t)data[0] | ((uint16_t)data[1] << 8));
}

/*
 * 向状态 payload 写入一个无符号 8 位数。
 *
 * pos 是当前写入位置，写完后自动向后移动 1 字节。
 */
static void put_u8(uint8_t *payload, uint8_t *pos, uint8_t value)
{
  payload[(*pos)++] = value;
}

/*
 * 向状态 payload 写入一个小端无符号 16 位数。
 */
static void put_u16(uint8_t *payload, uint8_t *pos, uint16_t value)
{
  payload[(*pos)++] = (uint8_t)(value & 0xFFU);
  payload[(*pos)++] = (uint8_t)(value >> 8);
}

/*
 * 向状态 payload 写入一个小端有符号 16 位数。
 */
static void put_i16(uint8_t *payload, uint8_t *pos, int16_t value)
{
  put_u16(payload, pos, (uint16_t)value);
}

/*
 * 向状态 payload 写入一个小端有符号 32 位数。
 */
static void put_i32(uint8_t *payload, uint8_t *pos, int32_t value)
{
  payload[(*pos)++] = (uint8_t)((uint32_t)value & 0xFFU);
  payload[(*pos)++] = (uint8_t)(((uint32_t)value >> 8) & 0xFFU);
  payload[(*pos)++] = (uint8_t)(((uint32_t)value >> 16) & 0xFFU);
  payload[(*pos)++] = (uint8_t)(((uint32_t)value >> 24) & 0xFFU);
}
#endif

/*
 * 获取应用状态互斥量。
 *
 * 只有调度器已经运行且互斥量已经创建时才会真正加锁。这样 BoardInit 阶段
 * 也可以安全调用一些共用函数，不会在 FreeRTOS 尚未启动时调用 OS API。
 */
static void ctx_lock(void)
{
  if (s_ctx_mutex != 0 && osKernelGetState() == osKernelRunning)
  {
    /*
     * 必须真正拿到互斥量后才能在 ctx_unlock 中释放它。
     *
     * 旧代码使用 20 ms 超时却忽略返回值：如果其它任务暂时占用互斥量，
     * 当前任务没有拿到锁，随后仍然调用 osMutexRelease，会造成“非持有者
     * 释放互斥量”。在开启 FreeRTOS 断言时，这会直接停在断言死循环，
     * 于是出现 LED 停止、串口遥测停止和底盘看起来卡死。
     *
     * 这里的临界区只复制状态或更新少量控制变量，不包含传感器 I2C 和
     * UART 阻塞发送，因此等待真正的锁不会影响正常运行。
     */
    while (osMutexAcquire(s_ctx_mutex, osWaitForever) != osOK)
    {
      /*
       * 正常情况下不会进入这里。保留让出 CPU 的重试，避免异常返回时
       * 忙等占满控制任务的时间片。
       */
      osThreadYield();
    }
  }
}

/*
 * 释放应用状态互斥量。
 *
 * 它与 ctx_lock 成对使用。只在成功进入任务运行阶段后才调用 OS 互斥量接口。
 */
static void ctx_unlock(void)
{
  if (s_ctx_mutex != 0 && osKernelGetState() == osKernelRunning)
  {
    (void)osMutexRelease(s_ctx_mutex);
  }
}

/*
 * 把控制来源枚举转换成短文本。
 *
 * 这个函数同时服务蓝牙反馈和状态显示。KICKPI 名称覆盖 ROS2、Web 以及
 * K230 经 KICKPI 转发的运动命令。
 */
const char *RobotApp_ControlSourceName(RobotControlSource_t source)
{
  if (source == ROBOT_CONTROL_SOURCE_KICKPI)
  {
    return "KICKPI";
  }
  if (source == ROBOT_CONTROL_SOURCE_BLUETOOTH)
  {
    return "BLUETOOTH";
  }
  if (source == ROBOT_CONTROL_SOURCE_UART1_DEBUG)
  {
    return "UART1_DEBUG";
  }
  return "NONE";
}

/*
 * 向底盘申请一组电机目标。
 *
 * 非零目标会申请或刷新控制租约；如果另一个来源已经拥有租约，则拒绝本次
 * 命令，不修改当前目标。零目标是该来源的正常停车命令：只有当前来源或无
 * 来源时才执行，这样 KICKPI 的周期性零刷新不会打断蓝牙正在控制的小车。
 */
uint8_t RobotApp_RequestMotion(RobotControlSource_t source,
                               int32_t motor_a,
                               int32_t motor_b,
                               uint8_t mode)
{
  uint32_t now = HAL_GetTick();
  uint8_t is_zero = (uint8_t)((motor_a == 0) && (motor_b == 0));
  uint8_t accepted = 0U;

  if (source == ROBOT_CONTROL_SOURCE_NONE)
  {
    return 0U;
  }

  ctx_lock();

#if ROBOT_UART1_DEBUG_ONLY && ROBOT_MOTOR_AUTO_TEST_ENABLE
  auto_test_cancel();
#endif

  if (is_zero != 0U)
  {
    if (s_ctx.control_owner == ROBOT_CONTROL_SOURCE_NONE ||
        s_ctx.control_owner == source)
    {
      s_ctx.target_a = 0;
      s_ctx.target_b = 0;
      s_ctx.command_seen = 0U;
      s_ctx.last_motion_cmd_ms = 0U;
      s_ctx.control_owner = ROBOT_CONTROL_SOURCE_NONE;
      if (s_ctx.faults == 0U)
      {
        s_ctx.state = ROBOT_APP_STATE_READY;
      }
      accepted = 1U;
    }
  }
  else if (s_ctx.faults == 0U &&
           (s_ctx.control_owner == ROBOT_CONTROL_SOURCE_NONE ||
            s_ctx.control_owner == source))
  {
    if (mode != ROBOT_MOTION_MODE_SPEED)
    {
      mode = ROBOT_MOTION_MODE_PWM;
    }

    if (mode == ROBOT_MOTION_MODE_PWM)
    {
      s_ctx.target_a = clamp_pwm_i32(motor_a);
      s_ctx.target_b = clamp_pwm_i32(motor_b);
    }
    else
    {
      s_ctx.target_a = (int16_t)clamp_i32(motor_a, -1500, 1500);
      s_ctx.target_b = (int16_t)clamp_i32(motor_b, -1500, 1500);
    }

    s_ctx.motion_mode = mode;
    s_ctx.last_cmd_ms = now;
    s_ctx.last_motion_cmd_ms = now;
    s_ctx.command_seen = 1U;
    s_ctx.control_owner = source;
    s_ctx.state = ROBOT_APP_STATE_RUN;
    accepted = 1U;
  }

  ctx_unlock();
  return accepted;
}

/*
 * 执行全局普通停车。
 *
 * stop 明确表示操作者要求立即停止，因此不受当前控制来源限制，并且会释放
 * 租约，让另一个控制端可以重新申请运动控制权。
 */
void RobotApp_RequestStop(RobotControlSource_t source)
{
  (void)source;

  ctx_lock();

#if ROBOT_UART1_DEBUG_ONLY && ROBOT_MOTOR_AUTO_TEST_ENABLE
  auto_test_cancel();
#endif

  s_ctx.target_a = 0;
  s_ctx.target_b = 0;
  s_ctx.command_seen = 0U;
  s_ctx.last_motion_cmd_ms = 0U;
  s_ctx.control_owner = ROBOT_CONTROL_SOURCE_NONE;
  s_ctx.state = ROBOT_APP_STATE_SAFE_IDLE;

  ctx_unlock();
}

/*
 * 处理底盘状态命令。
 *
 * 状态命令不抢占控制租约；idle 和 clear_fault 会停车并释放租约，enable
 * 只把无故障底盘置为 READY。
 */
uint8_t RobotApp_RequestState(uint8_t command)
{
  uint8_t accepted = 1U;

  ctx_lock();

#if ROBOT_UART1_DEBUG_ONLY && ROBOT_MOTOR_AUTO_TEST_ENABLE
  if (command == ROBOT_STATE_CMD_IDLE || command == ROBOT_STATE_CMD_CLEAR_FAULT)
  {
    auto_test_cancel();
  }
#endif

  if (command == ROBOT_STATE_CMD_IDLE)
  {
    s_ctx.target_a = 0;
    s_ctx.target_b = 0;
    s_ctx.command_seen = 0U;
    s_ctx.last_motion_cmd_ms = 0U;
    s_ctx.control_owner = ROBOT_CONTROL_SOURCE_NONE;
    s_ctx.state = ROBOT_APP_STATE_SAFE_IDLE;
  }
  else if (command == ROBOT_STATE_CMD_ENABLE)
  {
    if (s_ctx.faults == 0U)
    {
      s_ctx.state = ROBOT_APP_STATE_READY;
    }
    else
    {
      accepted = 0U;
    }
  }
  else if (command == ROBOT_STATE_CMD_CLEAR_FAULT)
  {
    s_ctx.faults = 0U;
    s_ctx.target_a = 0;
    s_ctx.target_b = 0;
    s_ctx.command_seen = 0U;
    s_ctx.last_motion_cmd_ms = 0U;
    s_ctx.control_owner = ROBOT_CONTROL_SOURCE_NONE;
    s_ctx.state = ROBOT_APP_STATE_SAFE_IDLE;
  }
  else
  {
    accepted = 0U;
  }

  ctx_unlock();
  return accepted;
}

/*
 * 立即急停。
 *
 * 急停会保留故障锁存，必须先 clear_fault，再重新 enable 和发送运动命令。
 */
void RobotApp_RequestEstop(void)
{
  ctx_lock();

#if ROBOT_UART1_DEBUG_ONLY && ROBOT_MOTOR_AUTO_TEST_ENABLE
  auto_test_cancel();
#endif

  s_ctx.target_a = 0;
  s_ctx.target_b = 0;
  s_ctx.command_seen = 0U;
  s_ctx.last_motion_cmd_ms = 0U;
  s_ctx.control_owner = ROBOT_CONTROL_SOURCE_NONE;
  s_ctx.faults |= ROBOT_FAULT_ESTOP;
  s_ctx.state = ROBOT_APP_STATE_FAULT;

  ctx_unlock();
}

/*
 * 根据传感器、故障和手动命令更新附加输出。
 *
 * 继电器和蜂鸣器都由 robot_config.h 控制。网页发送的 SET_OUTPUT 帧使用两个
 * 字节：payload[0] 为蜂鸣器，payload[1] 为继电器。两者上电均为关闭状态。
 * 继电器保留过温自动打开逻辑；蜂鸣器的故障自动报警另有独立宏控制，当前默认
 * 关闭，所以不会因为故障状态而持续鸣叫。
 */
static void apply_outputs(uint32_t now)
{
  uint8_t buzzer_on = 0U;
  uint8_t fan_on;

#if ROBOT_RELAY_ENABLE
#if ROBOT_RELAY_TEST_ENABLE
  /*
   * 继电器调试模式：按固定周期翻转测试状态。这个状态只作为一个“请求打开”
   * 条件参与最终输出，过温自动风扇和手动打开请求仍然可以把继电器保持吸合。
   */
  if ((uint32_t)(now - s_ctx.relay_test_last_ms) >= ROBOT_RELAY_TEST_PERIOD_MS)
  {
    s_ctx.relay_test_last_ms = now;
    s_ctx.relay_test_on ^= 1U;
  }
#endif

  /*
   * 风扇在调试阶段也允许单独测试，因为它只控制继电器，不会直接驱动电机。
   * DS18B20 使用上下两个温度阈值形成滞回：达到开启温度后，必须降到较低的
   * 关闭温度以下才会释放继电器，避免继电器在临界温度附近频繁吸合。
   */
  if ((s_ctx.sensors.valid_flags & ROBOT_SENSOR_DS18B20_VALID) != 0U)
  {
    if (s_ctx.sensors.battery_temp_c_x100 >= ROBOT_FAN_ON_C_X100)
    {
      s_ctx.auto_fan = 1U;
    }
    else if (s_ctx.sensors.battery_temp_c_x100 <= ROBOT_FAN_OFF_C_X100)
    {
      s_ctx.auto_fan = 0U;
    }
  }

  fan_on = (uint8_t)((s_ctx.manual_fan != 0U) || (s_ctx.auto_fan != 0U) ||
                     (s_ctx.relay_test_on != 0U) ||
                     ((s_ctx.faults & ROBOT_FAULT_OVER_TEMP) != 0U));
#else
  /* 当前不使用继电器，手动风扇、自动风扇和测试状态全部强制清零。 */
  (void)now;
  s_ctx.manual_fan = 0U;
  s_ctx.auto_fan = 0U;
  s_ctx.relay_test_on = 0U;
  fan_on = 0U;
#endif

#if ROBOT_BUZZER_ENABLE
  /*
   * 蜂鸣器由编译期宏统一控制。当前允许网页/协议手动控制，但故障自动报警
   * 由 ROBOT_BUZZER_FAULT_ALARM_ENABLE 单独决定，默认保持静音。
   */
  if (s_ctx.manual_buzzer != 0U)
  {
    buzzer_on = 1U;
  }
#if ROBOT_BUZZER_FAULT_ALARM_ENABLE
  else if (s_ctx.faults != 0U)
  {
    buzzer_on = (((now / 200U) & 0x01U) != 0U) ? 1U : 0U;
  }
#endif
#else
  (void)now;
  buzzer_on = 0U;
#endif

  s_ctx.buzzer_on = buzzer_on;
  HAL_GPIO_WritePin(Beep_GPIO_Port, Beep_Pin, buzzer_on ? GPIO_PIN_SET : GPIO_PIN_RESET);
  HAL_GPIO_WritePin(Relay_GPIO_Port, Relay_Pin, fan_on ? GPIO_PIN_SET : GPIO_PIN_RESET);
}

/*
 * 检查通信、电池、温度和可选堵转保护。
 *
 * 只要出现需要立即停机的故障，就把应用状态切换为 FAULT。控制任务看到 FAULT
 * 后会清零 PWM，保证串口断线、欠压或过温时电机不会继续运行。
 */
static void update_faults(uint32_t now)
{
  /*
   * 正式 KICKPI 通信时，机器人处于运行状态必须持续收到运动命令。如果 USB
   * 转串口线松动、串口终端关闭，或者 KICKPI/ROS2 节点崩溃，超时后就进入
   * FAULT 并停止电机。UART1 文本调试模式默认关闭这项检查，因为串口助手
   * 的 pwm/speed 命令是手动单次发送的，不能要求用户持续重复发送。
   */
#if ROBOT_DEBUG_MOTION_TIMEOUT_ENABLE
  if (s_ctx.state == ROBOT_APP_STATE_RUN && s_ctx.command_seen != 0U &&
      (uint32_t)(now - s_ctx.last_cmd_ms) > ROBOT_CMD_TIMEOUT_MS)
  {
    s_ctx.faults |= ROBOT_FAULT_COMM_TIMEOUT;
  }
#elif !ROBOT_UART1_DEBUG_ONLY
  /*
   * 正式 KICKPI 模式只对 KICKPI 的运动租约执行故障型通信超时。蓝牙链路
   * 使用下面独立的租约超时自动停车，不能因为蓝牙断开把底盘锁成 KICKPI
   * 通信故障，否则手机退出后 KICKPI 还需要人工清故障才能接管。
   */
  if (s_ctx.state == ROBOT_APP_STATE_RUN &&
      s_ctx.command_seen != 0U &&
      s_ctx.control_owner == ROBOT_CONTROL_SOURCE_KICKPI &&
      (uint32_t)(now - s_ctx.last_cmd_ms) > ROBOT_CMD_TIMEOUT_MS)
  {
    s_ctx.faults |= ROBOT_FAULT_COMM_TIMEOUT;
  }
#endif

  /*
   * INA219 当前只作为电池电压监测器使用。低电压故障只有在电压升到恢复阈值
   * 以上才清除，避免电池电压在临界点附近变化时导致故障状态反复跳变。
   */
  if ((s_ctx.sensors.valid_flags & ROBOT_SENSOR_INA219_VALID) != 0U)
  {
    if (s_ctx.sensors.battery_mv < ROBOT_BATTERY_LOW_MV)
    {
      s_ctx.faults |= ROBOT_FAULT_BATTERY_LOW;
    }
    else if (s_ctx.sensors.battery_mv > ROBOT_BATTERY_RECOVER_MV)
    {
      s_ctx.faults &= (uint16_t)~ROBOT_FAULT_BATTERY_LOW;
    }
  }

  /* 电池温度过高属于硬保护，电机必须停止，但继电器和风扇仍然可以工作。 */
  if ((s_ctx.sensors.valid_flags & ROBOT_SENSOR_DS18B20_VALID) != 0U &&
      s_ctx.sensors.battery_temp_c_x100 >= ROBOT_TEMP_FAULT_C_X100)
  {
    s_ctx.faults |= ROBOT_FAULT_OVER_TEMP;
  }

#if ROBOT_STALL_DETECT_ENABLE
  if (s_ctx.state == ROBOT_APP_STATE_RUN &&
      s_ctx.pwm_a > ROBOT_STALL_PWM_THRESHOLD &&
      s_ctx.encoder.delta_a < ROBOT_STALL_DELTA_THRESHOLD)
  {
    if (s_ctx.stall_a_start_ms == 0U)
    {
      s_ctx.stall_a_start_ms = now;
    }
    else if ((uint32_t)(now - s_ctx.stall_a_start_ms) > ROBOT_STALL_TIME_MS)
    {
      s_ctx.faults |= ROBOT_FAULT_STALL_A;
    }
  }
  else
  {
    s_ctx.stall_a_start_ms = 0U;
  }

  if (s_ctx.state == ROBOT_APP_STATE_RUN &&
      s_ctx.pwm_b > ROBOT_STALL_PWM_THRESHOLD &&
      s_ctx.encoder.delta_b < ROBOT_STALL_DELTA_THRESHOLD)
  {
    if (s_ctx.stall_b_start_ms == 0U)
    {
      s_ctx.stall_b_start_ms = now;
    }
    else if ((uint32_t)(now - s_ctx.stall_b_start_ms) > ROBOT_STALL_TIME_MS)
    {
      s_ctx.faults |= ROBOT_FAULT_STALL_B;
    }
  }
  else
  {
    s_ctx.stall_b_start_ms = 0U;
  }
#else
  (void)now;
  s_ctx.stall_a_start_ms = 0U;
  s_ctx.stall_b_start_ms = 0U;
#endif

  if (s_ctx.faults != 0U)
  {
    s_ctx.state = ROBOT_APP_STATE_FAULT;
  }
}

/*
 * 根据目标速度和实际速度计算一个保守的闭环 PWM。
 *
 * 这是一个简化的“前馈 + 比例 + 积分”控制器。首次调试建议使用 pwm 开环命令，
 * 只有确认轮径、编码器每圈计数和方向都正确后，再使用 speed 命令调这里的参数。
 */
static int16_t speed_pid(int16_t target_mm_s, int16_t actual_mm_s, int32_t *integral)
{
  int32_t error = (int32_t)target_mm_s - actual_mm_s;
  int32_t output;

  *integral = clamp_i32(*integral + error,
                        -ROBOT_SPEED_INTEGRAL_LIMIT,
                        ROBOT_SPEED_INTEGRAL_LIMIT);

  output = ((int32_t)target_mm_s * ROBOT_SPEED_FF_PWM_PER_MM_S_X100) / 100;
  output += (error * ROBOT_SPEED_KP_X100) / 100;
  output += ((*integral) * ROBOT_SPEED_KI_X100) / 100;

  return clamp_pwm_i32(output);
}

/*
 * 执行一次 10ms 电机控制周期。
 *
 * 这个函数只由控制任务调用，统一完成编码器采样、传感器快照、故障判断、PWM
 * 计算和输出。把这些动作集中在一个任务里，可以避免多个任务同时改电机状态。
 */
static void control_step(void)
{
  uint32_t now = HAL_GetTick();

  /*
   * 只有控制任务负责采样编码器和写入电机 PWM。这样可以保证时间行为相对固定，
   * 不会因为慢速传感器读取或串口命令解析而让多个任务同时修改电机输出。
   */
  RobotEncoder_Sample(ROBOT_CONTROL_PERIOD_MS, &s_ctx.encoder);
  RobotSensors_Get(&s_ctx.sensors);

#if ROBOT_UART1_DEBUG_ONLY && ROBOT_MOTOR_AUTO_TEST_ENABLE
  auto_test_step(now);
#endif

  /*
   * 蓝牙运动命令使用独立租约。蓝牙模块掉线或手机 App 停止刷新时，自动
   * 清零目标并释放来源，保证电机不会因为最后一条命令停留在运行状态。
   */
  if (s_ctx.control_owner == ROBOT_CONTROL_SOURCE_BLUETOOTH &&
      s_ctx.last_motion_cmd_ms != 0U &&
      (uint32_t)(now - s_ctx.last_motion_cmd_ms) >
      ROBOT_BLUETOOTH_OWNER_TIMEOUT_MS)
  {
    s_ctx.target_a = 0;
    s_ctx.target_b = 0;
    s_ctx.command_seen = 0U;
    s_ctx.last_motion_cmd_ms = 0U;
    s_ctx.control_owner = ROBOT_CONTROL_SOURCE_NONE;
    if (s_ctx.faults == 0U)
    {
      s_ctx.state = ROBOT_APP_STATE_READY;
    }
  }

  update_faults(now);

  if (s_ctx.state != ROBOT_APP_STATE_RUN)
  {
    s_ctx.pwm_a = 0;
    s_ctx.pwm_b = 0;
    s_ctx.integ_a = 0;
    s_ctx.integ_b = 0;
    RobotMotor_Stop();
    RobotMotor_Enable(0U);
    apply_outputs(now);
    return;
  }

  if (s_ctx.motion_mode == ROBOT_MOTION_MODE_SPEED)
  {
    s_ctx.pwm_a = speed_pid(s_ctx.target_a, s_ctx.encoder.speed_a_mm_s, &s_ctx.integ_a);
    s_ctx.pwm_b = speed_pid(s_ctx.target_b, s_ctx.encoder.speed_b_mm_s, &s_ctx.integ_b);
  }
  else
  {
    s_ctx.pwm_a = clamp_pwm_i32(s_ctx.target_a);
    s_ctx.pwm_b = clamp_pwm_i32(s_ctx.target_b);
    s_ctx.integ_a = 0;
    s_ctx.integ_b = 0;
  }

  RobotMotor_SetPercent(s_ctx.pwm_a, s_ctx.pwm_b);
  apply_outputs(now);
}

#if ROBOT_UART1_DEBUG_ONLY
/*
 * 将内部状态枚举转换为串口打印用的文字。
 *
 * 这样状态行里会显示 IDLE、READY、RUN、FAULT，而不是让你对着 0、1、2、3
 * 去猜当前系统处于哪个阶段。
 */
static const char *state_name(RobotAppState_t state)
{
  if (state == ROBOT_APP_STATE_SAFE_IDLE)
  {
    return "IDLE";
  }
  if (state == ROBOT_APP_STATE_READY)
  {
    return "READY";
  }
  if (state == ROBOT_APP_STATE_RUN)
  {
    return "RUN";
  }
  return "FAULT";
}

/*
 * 将运动模式转换为串口打印用的文字。
 *
 * mode=0 显示 PWM，mode=1 显示 SPEED，便于确认当前执行的是开环还是闭环。
 */
static const char *mode_name(uint8_t mode)
{
  return (mode == ROBOT_MOTION_MODE_SPEED) ? "SPEED" : "PWM";
}

/*
 * 返回带小数温度值的正负号。
 *
 * 温度内部以 0.01°C 保存。状态打印时需要单独打印符号、整数部分和小数部分，
 * 所以把这一步单独封装。
 */
static char x100_sign(int16_t value)
{
  return (value < 0) ? '-' : '+';
}

/*
 * 取出 0.01°C 数值的绝对值整数部分。
 *
 * 例如 -1234 会返回 12，用于打印成 -12.34°C。
 */
static uint16_t x100_abs_whole(int16_t value)
{
  int32_t v = value;

  if (v < 0)
  {
    v = -v;
  }
  return (uint16_t)(v / 100);
}

/*
 * 取出 0.01°C 数值的绝对值小数部分。
 *
 * 例如 -1234 会返回 34，用于打印成 -12.34°C。
 */
static uint16_t x100_abs_frac(int16_t value)
{
  int32_t v = value;

  if (v < 0)
  {
    v = -v;
  }
  return (uint16_t)(v % 100);
}

/*
 * 调试模式下输出一组完整的状态信息。
 *
 * 这里分成三行：第一行是系统状态和电机输出，第二行是编码器，第三行是各类
 * 传感器。每 500ms 由遥测任务调用一次，也可以通过 status 命令立即调用。
 */
static void send_status(void)
{
  RobotAppStatus_t status;

  RobotApp_GetStatus(&status);

  RobotComm_DebugPrintf(
      "T=%lu STATE=%s FAULT=0x%04X MODE=%s OWNER=%s TARGET[A=%d B=%d] PWM[A=%d B=%d] RELAY=%u BUZZ=%u\r\n",
      HAL_GetTick(),
      state_name(status.state),
      status.faults,
      mode_name(status.motion_mode),
      RobotApp_ControlSourceName(status.control_owner),
      status.target_a,
      status.target_b,
      status.pwm_a,
      status.pwm_b,
      status.fan_on,
      status.buzzer_on);

  RobotComm_DebugPrintf(
      "ENC A[total=%ld delta=%d speed=%dmm/s] B[total=%ld delta=%d speed=%dmm/s]\r\n",
      status.encoder.total_a,
      status.encoder.delta_a,
      status.encoder.speed_a_mm_s,
      status.encoder.total_b,
      status.encoder.delta_b,
      status.encoder.speed_b_mm_s);

  RobotComm_DebugPrintf(
      "SENS flags=0x%04X BAT=%umV ENV=%c%u.%02uC %u.%02u%% DS=%c%u.%02uC MPU acc=%d,%d,%d gyro=%d,%d,%d RXOV=%lu\r\n",
      status.sensors.valid_flags,
      status.sensors.battery_mv,
      x100_sign(status.sensors.env_temp_c_x100),
      x100_abs_whole(status.sensors.env_temp_c_x100),
      x100_abs_frac(status.sensors.env_temp_c_x100),
      status.sensors.env_humi_x100 / 100U,
      status.sensors.env_humi_x100 % 100U,
      x100_sign(status.sensors.battery_temp_c_x100),
      x100_abs_whole(status.sensors.battery_temp_c_x100),
      x100_abs_frac(status.sensors.battery_temp_c_x100),
      status.sensors.imu_accel_raw[0],
      status.sensors.imu_accel_raw[1],
      status.sensors.imu_accel_raw[2],
      status.sensors.imu_gyro_raw[0],
      status.sensors.imu_gyro_raw[1],
      status.sensors.imu_gyro_raw[2],
      RobotComm_GetRxOverflowCount());
}
#else
/*
 * 正式 KICKPI 模式下打包并发送二进制状态帧。
 *
 * 字段顺序必须和 KICKPI 端的解析代码一致。这里使用小端格式，把系统状态、
 * 编码器、传感器和故障信息集中回传。
 */
static void send_status(void)
{
  uint8_t payload[64];
  uint8_t pos = 0U;
  RobotAppStatus_t status;

  RobotApp_GetStatus(&status);

  put_i32(payload, &pos, (int32_t)HAL_GetTick());
  put_u8(payload, &pos, (uint8_t)status.state);
  put_u16(payload, &pos, status.faults);
  put_u8(payload, &pos, status.motion_mode);
  put_i16(payload, &pos, status.target_a);
  put_i16(payload, &pos, status.target_b);
  put_i16(payload, &pos, status.pwm_a);
  put_i16(payload, &pos, status.pwm_b);
  put_i16(payload, &pos, status.encoder.speed_a_mm_s);
  put_i16(payload, &pos, status.encoder.speed_b_mm_s);
  put_i32(payload, &pos, status.encoder.total_a);
  put_i32(payload, &pos, status.encoder.total_b);
  put_u16(payload, &pos, status.sensors.battery_mv);
  put_i16(payload, &pos, status.sensors.battery_temp_c_x100);
  put_i16(payload, &pos, status.sensors.env_temp_c_x100);
  put_u16(payload, &pos, status.sensors.env_humi_x100);
  put_i16(payload, &pos, status.sensors.imu_accel_raw[0]);
  put_i16(payload, &pos, status.sensors.imu_accel_raw[1]);
  put_i16(payload, &pos, status.sensors.imu_accel_raw[2]);
  put_i16(payload, &pos, status.sensors.imu_gyro_raw[0]);
  put_i16(payload, &pos, status.sensors.imu_gyro_raw[1]);
  put_i16(payload, &pos, status.sensors.imu_gyro_raw[2]);
  put_u16(payload, &pos, status.sensors.valid_flags);
  put_i32(payload, &pos, (int32_t)RobotComm_GetRxOverflowCount());
  put_u8(payload, &pos, status.fan_on);
  put_u8(payload, &pos, status.buzzer_on);
  put_u8(payload, &pos, (uint8_t)status.control_owner);

  (void)RobotComm_SendFrame(ROBOT_CMD_STATUS, payload, pos);
}
#endif

#if !ROBOT_UART1_DEBUG_ONLY
/*
 * 处理正式模式下的运动命令。
 *
 * payload 格式为 int16 A、int16 B、uint8 mode。目标为 0 时进入 READY；目标
 * 非 0 时进入 RUN。若已有故障，则只保存目标，不允许真正驱动电机。
 */
static void handle_set_motion(const RobotCommFrame_t *frame)
{
  if (frame->length < 5U)
  {
    return;
  }

  /*
   * UART1 的正式帧统一走控制租约接口。这样 KICKPI、蓝牙和以后增加的
   * 控制端不会各自维护一套“谁能改目标”的判断。
   */
  (void)RobotApp_RequestMotion(ROBOT_CONTROL_SOURCE_KICKPI,
                               read_i16_le(&frame->payload[0]),
                               read_i16_le(&frame->payload[2]),
                               frame->payload[4]);
}

/*
 * 处理正式模式下的状态命令。
 *
 * IDLE 会停止并回到安全空闲，ENABLE 只进入 READY，CLEAR_FAULT 会清故障并
 * 停止电机。清故障后必须重新发送运动命令，避免电机突然恢复运行。
 */
static void handle_set_state(const RobotCommFrame_t *frame)
{
  if (frame->length < 1U)
  {
    return;
  }

  (void)RobotApp_RequestState(frame->payload[0]);
}

/*
 * 处理正式模式下的附加输出命令。
 *
 * payload[0] 控制蜂鸣器，payload[1] 控制继电器。每次只接收两个字节，
 * 且统一归一化为 0/1，避免上层传入任意数值造成不确定输出。
 */
static void handle_set_output(const RobotCommFrame_t *frame)
{
  ctx_lock();

  if (frame->length < 2U)
  {
    ctx_unlock();
    return;
  }

#if ROBOT_BUZZER_ENABLE
  s_ctx.manual_buzzer = frame->payload[0] ? 1U : 0U;
#else
  s_ctx.manual_buzzer = 0U;
#endif

#if ROBOT_RELAY_ENABLE
  s_ctx.manual_fan = frame->payload[1] ? 1U : 0U;
#else
  s_ctx.manual_fan = 0U;
#endif

  ctx_unlock();
}

/*
 * 正式模式的协议帧业务分发函数。
 *
 * 串口层只负责把 CRC 正确的帧交给这里；这里再根据命令号调用具体业务处理
 * 函数，并用状态互斥量保护共享上下文。
 */
static void RobotApp_OnFrame(const RobotCommFrame_t *frame)
{
  if (frame->command == ROBOT_CMD_SET_MOTION)
  {
    handle_set_motion(frame);
  }
  else if (frame->command == ROBOT_CMD_SET_STATE)
  {
    handle_set_state(frame);
  }
  else if (frame->command == ROBOT_CMD_GET_STATUS)
  {
    RobotApp_RequestTelemetry();
  }
  else if (frame->command == ROBOT_CMD_SET_OUTPUT)
  {
    handle_set_output(frame);
  }
  else if (frame->command == ROBOT_CMD_ESTOP)
  {
    RobotApp_RequestEstop();
  }
}
#endif

#if ROBOT_UART1_DEBUG_ONLY
/*
 * 打印底板串口调试命令帮助。
 *
 * 这份帮助会在上电时打印一次，也可以输入 help 再打印。命令设计得尽量简单，
 * 方便你用普通串口助手直接测试，不依赖 KICKPI 或 ROS2。
 */
static void debug_print_help(void)
{
  RobotComm_DebugPrintf("\r\nRobot base UART1 debug mode\r\n");
  RobotComm_DebugPrintf("Commands: help | status | enable | stop | clear | estop\r\n");
  RobotComm_DebugPrintf("          pwm <A -1000..1000> <B -1000..1000>\r\n");
  RobotComm_DebugPrintf("          speed <A mm/s> <B mm/s>\r\n");
#if ROBOT_BUZZER_ENABLE
  RobotComm_DebugPrintf("Buzzer manual control is enabled; fault alarm is %s.\r\n",
#if ROBOT_BUZZER_FAULT_ALARM_ENABLE
                        "enabled");
#else
                        "disabled");
#endif
#else
  RobotComm_DebugPrintf("Buzzer is disabled by ROBOT_BUZZER_ENABLE=0.\r\n");
#endif
#if ROBOT_RELAY_ENABLE
#if ROBOT_RELAY_TEST_ENABLE
  RobotComm_DebugPrintf("Relay test is enabled: toggle every %lu ms.\r\n",
                        (unsigned long)ROBOT_RELAY_TEST_PERIOD_MS);
#else
  RobotComm_DebugPrintf("Relay test is disabled.\r\n");
#endif
#else
  RobotComm_DebugPrintf("Relay/fan output is disabled by ROBOT_RELAY_ENABLE=0.\r\n");
#endif
  RobotComm_DebugPrintf("\r\n");
}

/*
 * 设置调试阶段的电机目标。
 *
 * mode=PWM 时，motor_a/motor_b 是 -1000 到 1000 的开环 PWM。
 * mode=SPEED 时，motor_a/motor_b 是 mm/s 速度目标。
 *
 * 函数只修改目标值，真正的方向脚和 PWM 输出由 10ms 控制任务统一执行。
 */
static void debug_set_motion(int32_t motor_a, int32_t motor_b, uint8_t mode)
{
  uint8_t accepted = RobotApp_RequestMotion(ROBOT_CONTROL_SOURCE_UART1_DEBUG,
                                            motor_a,
                                            motor_b,
                                            mode);

  if (accepted != 0U)
  {
    RobotComm_DebugPrintf("OK motion source=UART1_DEBUG mode=%s A=%ld B=%ld\r\n",
                          mode_name(mode), (long)motor_a, (long)motor_b);
  }
  else
  {
    RobotComm_DebugPrintf("BUSY motion owner=%s\r\n",
                          RobotApp_ControlSourceName(s_ctx.control_owner));
  }
}

/*
 * 停止电机并回到安全空闲状态。
 *
 * 清除 command_seen 可以防止停止后继续触发通信超时故障；控制任务下一周期
 * 会看到 SAFE_IDLE，并再次把电机 PWM 清零。
 */
static void debug_stop(void)
{
  RobotApp_RequestStop(ROBOT_CONTROL_SOURCE_UART1_DEBUG);
  RobotComm_DebugPrintf("OK stop\r\n");
}

/*
 * 清除当前故障并停止电机。
 *
 * 清故障不是自动恢复运动，而是回到 SAFE_IDLE。这样清故障之后还要重新输入
 * pwm 或 speed 命令，避免设备在故障刚消失时突然转动。
 */
static void debug_clear_fault(void)
{
  (void)RobotApp_RequestState(ROBOT_STATE_CMD_CLEAR_FAULT);
  RobotComm_DebugPrintf("OK clear faults\r\n");
}

#if ROBOT_RELAY_ENABLE
/*
 * 手动控制风扇继电器。
 *
 * value=0 关闭手动继电器，value 非 0 打开手动继电器。温度自动控制仍然有效，
 * 因此即使手动关掉，过温保护也可以重新打开风扇。
 */
static void debug_set_relay(int32_t value)
{
  ctx_lock();
  s_ctx.manual_fan = (value != 0) ? 1U : 0U;
  apply_outputs(HAL_GetTick());
  ctx_unlock();

  RobotComm_DebugPrintf("OK relay=%d\r\n", (value != 0) ? 1 : 0);
}
#endif

/*
 * 处理一整行底板调试命令。
 *
 * 这些命令只服务于当前硬件调试阶段，不是 ROS 控制接口。底板确认无误后，把
 * ROBOT_UART1_DEBUG_ONLY 改为 0，正式由 KICKPI 的二进制协议接管 UART1。
 */
static void RobotApp_OnDebugLine(const char *line)
{
  int motor_a;
  int motor_b;
  int value;

  if (strcmp(line, "help") == 0)
  {
    debug_print_help();
  }
  else if (strcmp(line, "status") == 0)
  {
    send_status();
  }
  else if (strcmp(line, "enable") == 0)
  {
    if (RobotApp_RequestState(ROBOT_STATE_CMD_ENABLE) != 0U)
    {
      RobotComm_DebugPrintf("OK enable\r\n");
    }
    else
    {
      RobotComm_DebugPrintf("ERR enable fault=0x%04X\r\n", s_ctx.faults);
    }
  }
  else if (strcmp(line, "stop") == 0)
  {
    debug_stop();
  }
  else if (strcmp(line, "clear") == 0)
  {
    debug_clear_fault();
  }
  else if (strcmp(line, "estop") == 0)
  {
    RobotApp_RequestEstop();
    RobotComm_DebugPrintf("OK estop\r\n");
  }
  else if (sscanf(line, "pwm %d %d", &motor_a, &motor_b) == 2)
  {
    debug_set_motion(motor_a, motor_b, ROBOT_MOTION_MODE_PWM);
  }
  else if (sscanf(line, "speed %d %d", &motor_a, &motor_b) == 2)
  {
    debug_set_motion(motor_a, motor_b, ROBOT_MOTION_MODE_SPEED);
  }
#if ROBOT_RELAY_ENABLE
  else if (sscanf(line, "relay %d", &value) == 1)
  {
    debug_set_relay(value);
  }
#else
  else if (sscanf(line, "relay %d", &value) == 1)
  {
    (void)value;
    RobotComm_DebugPrintf("IGNORED relay disabled by ROBOT_RELAY_ENABLE=0\r\n");
  }
#endif
  else if (sscanf(line, "beep %d", &value) == 1)
  {
    (void)value;
    RobotComm_DebugPrintf("IGNORED buzzer disabled by ROBOT_BUZZER_ENABLE=0\r\n");
  }
  else
  {
    RobotComm_DebugPrintf("ERR unknown command: %s\r\n", line);
    debug_print_help();
  }
}
#endif

/*
 * 蓝牙端帮助文本。
 *
 * 蓝牙模块采用最简单的“每条命令一行”协议，手机 App 和普通蓝牙助手都能
 * 使用。方向命令默认使用 PWM=300，也可以显式填写 0 到 1000 的幅值。
 */
static void bluetooth_print_help(void)
{
  RobotBluetooth_SendText(
      "Commands: help status enable stop clear estop\r\n"
      "          pwm <A -1000..1000> <B -1000..1000>\r\n"
      "          speed <A mm/s> <B mm/s>\r\n"
      "          forward [PWM] back [PWM] left [PWM] right [PWM]\r\n");
}

/*
 * 通过蓝牙返回一行简要状态。
 *
 * 详细传感器数据仍然由 KICKPI 的二进制状态帧上送 ROS2；手机端这里主要需要
 * 知道连接是否有效、当前控制权属于谁、目标值和故障码。
 */
static void bluetooth_send_status(void)
{
  RobotAppStatus_t status;

  RobotApp_GetStatus(&status);
  RobotBluetooth_Sendf(
      "STATUS state=%u fault=0x%04X owner=%s target=%d,%d pwm=%d,%d bat=%umV\r\n",
      (unsigned int)status.state,
      status.faults,
      RobotApp_ControlSourceName(status.control_owner),
      status.target_a,
      status.target_b,
      status.pwm_a,
      status.pwm_b,
      (unsigned int)status.sensors.battery_mv);
}

/*
 * 返回蓝牙运动命令的处理结果。
 *
 * 命令被其他控制端占用时反馈 BUSY；传感器欠压、急停等故障存在时反馈
 * FAULT。这样手机 App 不需要猜测按钮为什么没有让电机运动。
 */
static void bluetooth_report_motion_result(uint8_t accepted,
                                           int32_t motor_a,
                                           int32_t motor_b,
                                           uint8_t mode)
{
  RobotAppStatus_t status;

  RobotApp_GetStatus(&status);
  if (accepted != 0U)
  {
    RobotBluetooth_Sendf("OK motion mode=%u target=%ld,%ld owner=%s\r\n",
                         (unsigned int)mode,
                         (long)motor_a,
                         (long)motor_b,
                         RobotApp_ControlSourceName(status.control_owner));
  }
  else if (status.faults != 0U)
  {
    RobotBluetooth_Sendf("FAULT code=0x%04X\r\n", status.faults);
  }
  else
  {
    RobotBluetooth_Sendf("BUSY owner=%s\r\n",
                         RobotApp_ControlSourceName(status.control_owner));
  }
}

/*
 * 处理 USART2 蓝牙模块收到的一整行命令。
 *
 * 这条路径既供手机 App 使用，也供普通蓝牙串口助手手工测试。所有运动命令
 * 最终都调用 RobotApp_RequestMotion，因此和 KICKPI 共享同一套控制权规则。
 */
void RobotApp_OnBluetoothLine(const char *line)
{
  int motor_a;
  int motor_b;
  int value;
  uint8_t accepted;

  if (line == 0)
  {
    return;
  }

  if (strcmp(line, "help") == 0)
  {
    bluetooth_print_help();
  }
  else if (strcmp(line, "status") == 0)
  {
    bluetooth_send_status();
  }
  else if (strcmp(line, "enable") == 0)
  {
    if (RobotApp_RequestState(ROBOT_STATE_CMD_ENABLE) != 0U)
    {
      RobotBluetooth_SendText("OK enable\r\n");
    }
    else
    {
      bluetooth_send_status();
    }
  }
  else if (strcmp(line, "stop") == 0)
  {
    RobotApp_RequestStop(ROBOT_CONTROL_SOURCE_BLUETOOTH);
    RobotBluetooth_SendText("OK stop\r\n");
  }
  else if (strcmp(line, "clear") == 0 || strcmp(line, "clear_fault") == 0)
  {
    (void)RobotApp_RequestState(ROBOT_STATE_CMD_CLEAR_FAULT);
    RobotBluetooth_SendText("OK clear\r\n");
  }
  else if (strcmp(line, "estop") == 0)
  {
    RobotApp_RequestEstop();
    RobotBluetooth_SendText("OK estop\r\n");
  }
  else if (sscanf(line, "pwm %d %d", &motor_a, &motor_b) == 2)
  {
    accepted = RobotApp_RequestMotion(ROBOT_CONTROL_SOURCE_BLUETOOTH,
                                      motor_a,
                                      motor_b,
                                      ROBOT_MOTION_MODE_PWM);
    bluetooth_report_motion_result(accepted,
                                   motor_a,
                                   motor_b,
                                   ROBOT_MOTION_MODE_PWM);
  }
  else if (sscanf(line, "speed %d %d", &motor_a, &motor_b) == 2)
  {
    accepted = RobotApp_RequestMotion(ROBOT_CONTROL_SOURCE_BLUETOOTH,
                                      motor_a,
                                      motor_b,
                                      ROBOT_MOTION_MODE_SPEED);
    bluetooth_report_motion_result(accepted,
                                   motor_a,
                                   motor_b,
                                   ROBOT_MOTION_MODE_SPEED);
  }
  else if (strcmp(line, "forward") == 0 ||
           sscanf(line, "forward %d", &value) == 1)
  {
    if (strcmp(line, "forward") == 0)
    {
      value = 300;
    }
    accepted = RobotApp_RequestMotion(ROBOT_CONTROL_SOURCE_BLUETOOTH,
                                      value,
                                      value,
                                      ROBOT_MOTION_MODE_PWM);
    bluetooth_report_motion_result(accepted, value, value, ROBOT_MOTION_MODE_PWM);
  }
  else if (strcmp(line, "back") == 0 ||
           sscanf(line, "back %d", &value) == 1)
  {
    if (strcmp(line, "back") == 0)
    {
      value = 300;
    }
    accepted = RobotApp_RequestMotion(ROBOT_CONTROL_SOURCE_BLUETOOTH,
                                      -value,
                                      -value,
                                      ROBOT_MOTION_MODE_PWM);
    bluetooth_report_motion_result(accepted, -value, -value, ROBOT_MOTION_MODE_PWM);
  }
  else if (strcmp(line, "left") == 0 ||
           sscanf(line, "left %d", &value) == 1)
  {
    if (strcmp(line, "left") == 0)
    {
      value = 300;
    }
    accepted = RobotApp_RequestMotion(ROBOT_CONTROL_SOURCE_BLUETOOTH,
                                      -value,
                                      value,
                                      ROBOT_MOTION_MODE_PWM);
    bluetooth_report_motion_result(accepted, -value, value, ROBOT_MOTION_MODE_PWM);
  }
  else if (strcmp(line, "right") == 0 ||
           sscanf(line, "right %d", &value) == 1)
  {
    if (strcmp(line, "right") == 0)
    {
      value = 300;
    }
    accepted = RobotApp_RequestMotion(ROBOT_CONTROL_SOURCE_BLUETOOTH,
                                      value,
                                      -value,
                                      ROBOT_MOTION_MODE_PWM);
    bluetooth_report_motion_result(accepted, value, -value, ROBOT_MOTION_MODE_PWM);
  }
  else
  {
    RobotBluetooth_Sendf("ERR unknown command: %s\r\n", line);
    bluetooth_print_help();
  }
}

/*
 * 初始化机器人底板应用层。
 *
 * 调用时机：main.c 已经完成 HAL、GPIO、I2C、TIM1、TIM2、TIM4 和 USART1
 * 初始化之后，但 FreeRTOS 还没有启动。USART2 只有在蓝牙开关打开时才初始化。
 *
 * 初始化顺序：
 * 1. 打开 DWT 微秒延时；
 * 2. 启动 PWM 并保持 TB6612 待机；
 * 3. 启动两个硬件编码器；
 * 4. 初始化 INA219、MPU6050 和 DS18B20；
 * 5. 开启 UART1 接收中断。
 *
 * 这个函数结束时，电机仍然是安全停止状态。
 */
void RobotApp_BoardInit(void)
{
  memset(&s_ctx, 0, sizeof(s_ctx));
  s_ctx.state = ROBOT_APP_STATE_SAFE_IDLE;
  s_ctx.motion_mode = ROBOT_MOTION_MODE_PWM;

#if ROBOT_UART1_DEBUG_ONLY && ROBOT_MOTOR_AUTO_TEST_ENABLE
  /*
   * 调试版上电自动测试从安全等待阶段开始。此时目标 PWM 为 0，给操作者
   * 留出一秒钟确认车轮悬空；进入控制任务后才会真正驱动 A/B 通道。
   */
  s_ctx.auto_test_active = 1U;
  s_ctx.auto_test_stage = ROBOT_AUTO_TEST_DELAY;
  s_ctx.auto_test_stage_start_ms = HAL_GetTick();
#endif

  RobotDelay_Init();
  RobotMotor_Init();
  RobotEncoder_Init();
  RobotSensors_Init();
  RobotComm_Init();

#if ROBOT_UART1_DEBUG_ONLY
  debug_print_help();
#if ROBOT_MOTOR_AUTO_TEST_ENABLE
  RobotComm_DebugPrintf("AUTO_TEST scheduled: A then B, wheel must be lifted.\r\n");
#endif
#endif
}

/*
 * 创建机器人应用层的 FreeRTOS 对象和任务。
 *
 * 创建的任务包括：
 * - 控制任务：10ms 周期，处理串口命令、编码器和电机输出；
 * - 传感器任务：周期读取 I2C 传感器和 DS18B20；
 * - 遥测任务：每 500ms 输出调试状态或发送正式状态帧。
 *
 * 函数应在 osKernelInitialize 之后、osKernelStart 之前调用。
 */
void RobotApp_CreateTasks(void)
{
  s_ctx_mutex = osMutexNew(&s_ctx_mutex_attr);
  /*
   * 这些对象全部来自 FreeRTOS 动态 heap。任何一个创建失败都意味着后续
   * 控制、传感器或遥测功能不完整，不能继续以“看起来正常”的方式运行。
   * configASSERT 会进入统一的电机安全停车和 LED 故障指示。
   */
  configASSERT(s_ctx_mutex != 0);
#if ROBOT_UART1_DEBUG_ONLY
  RobotComm_SetDebugLineHandler(RobotApp_OnDebugLine);
#else
  RobotComm_SetFrameHandler(RobotApp_OnFrame);
#endif
#if ROBOT_BLUETOOTH_ENABLE
  RobotBluetooth_SetLineHandler(RobotApp_OnBluetoothLine);
#endif

  configASSERT(osThreadNew(RobotApp_ControlTask, 0, &s_control_attr) != 0);
  configASSERT(osThreadNew(RobotApp_SensorTask, 0, &s_sensor_attr) != 0);
  configASSERT(osThreadNew(RobotApp_TelemetryTask, 0, &s_telemetry_attr) != 0);
}

/*
 * CubeMX 默认任务的入口。
 *
 * 当前没有把非实时业务放进这里，只保留一个低频循环。以后如果增加日志保存、
 * 参数管理等不影响电机实时性的功能，可以放到这个任务中。
 */
void RobotApp_DefaultTask(void *argument)
{
  (void)argument;

  for (;;)
  {
    osDelay(1000U);
  }
}

/*
 * LED 状态指示任务。
 *
 * LED 闪烁速度：
 * - FAULT：快速闪烁；
 * - RUN：中速闪烁；
 * - IDLE/READY：慢速闪烁。
 *
 * 它只显示状态，不参与电机控制，因此 LED 异常不会影响电机输出。
 */
void RobotApp_StatusLedTask(void *argument)
{
  uint32_t delay_ms;

  (void)argument;

  for (;;)
  {
    ctx_lock();
    if (s_ctx.state == ROBOT_APP_STATE_FAULT)
    {
      delay_ms = 100U;
    }
    else if (s_ctx.state == ROBOT_APP_STATE_RUN)
    {
      delay_ms = 200U;
    }
    else
    {
      delay_ms = 500U;
    }
    ctx_unlock();

    HAL_GPIO_TogglePin(Led_GPIO_Port, Led_Pin);
    osDelay(delay_ms);
  }
}

/*
 * 获取一份线程安全的机器人状态快照。
 *
 * 由于控制任务可能正在更新 PWM、编码器和故障状态，本函数先获取互斥量，
 * 再把完整数据复制到 out，最后释放互斥量。调用者拿到的是同一时刻的快照。
 */
void RobotApp_GetStatus(RobotAppStatus_t *out)
{
  if (out == 0)
  {
    return;
  }

  ctx_lock();
  out->state = s_ctx.state;
  out->faults = s_ctx.faults;
  out->motion_mode = s_ctx.motion_mode;
  out->control_owner = s_ctx.control_owner;
  out->target_a = s_ctx.target_a;
  out->target_b = s_ctx.target_b;
  out->pwm_a = s_ctx.pwm_a;
  out->pwm_b = s_ctx.pwm_b;
#if ROBOT_RELAY_ENABLE
  out->fan_on = (uint8_t)((s_ctx.manual_fan != 0U) || (s_ctx.auto_fan != 0U) ||
                          (s_ctx.relay_test_on != 0U) ||
                          ((s_ctx.faults & ROBOT_FAULT_OVER_TEMP) != 0U));
#else
  out->fan_on = 0U;
#endif
  out->buzzer_on = s_ctx.buzzer_on;
  out->encoder = s_ctx.encoder;
  out->sensors = s_ctx.sensors;
  ctx_unlock();
}

/*
 * 请求遥测任务尽快发送一次状态。
 *
 * 正式 KICKPI 模式下可用于响应 GET_STATUS 命令。当前文本调试模式会固定每
 * 500ms 打印一次，所以这个标志在调试模式下不会改变打印周期。
 */
void RobotApp_RequestTelemetry(void)
{
  ctx_lock();
  s_ctx.telemetry_request = 1U;
  ctx_unlock();
}

/*
 * 机器人 10ms 控制任务。
 *
 * 每个周期的顺序是：
 * 1. 在互斥量外解析 UART1 接收数据，避免命令回调再次加锁造成死锁；
 * 2. 加锁后采样编码器、读取传感器快照、检查故障；
 * 3. 如果允许运行，计算 PWM 或闭环速度输出；
 * 4. 写入 TB6612 方向脚、PWM、风扇继电器和蜂鸣器；
 * 5. 使用 osDelayUntil 尽量维持稳定的 10ms 周期。
 */
static void RobotApp_ControlTask(void *argument)
{
  uint32_t wake_tick = osKernelGetTickCount();

  (void)argument;

  for (;;)
  {
    RobotComm_ProcessRx();
#if ROBOT_BLUETOOTH_ENABLE
    RobotComm_ProcessBluetoothRx();
#endif

    ctx_lock();
    control_step();
    ctx_unlock();

#if ROBOT_IWDG_ENABLE
    /*
     * 只由 10ms 控制任务刷新硬件看门狗。
     *
     * 如果控制任务因为死循环、互斥量死锁、异常或调度器异常而不能继续
     * 执行到这里，IWDG 就不会被刷新，芯片会在设定时间后自动复位。传感器
     * 任务和遥测任务不能代替控制任务刷新，这样才能真正检测到底盘控制链路。
     */
    (void)HAL_IWDG_Refresh(&hiwdg);
#endif

    wake_tick += ROBOT_CONTROL_PERIOD_MS;
    (void)osDelayUntil(wake_tick);
  }
}

/*
 * 传感器任务。
 *
 * RobotSensors_Update 内部已经按照不同传感器的实际转换速度做了节流，所以
 * 任务每 50ms 调一次即可。DHT30 和 DS18B20 的较慢读取不会阻塞电机控制任务。
 */
static void RobotApp_SensorTask(void *argument)
{
  (void)argument;

  for (;;)
  {
    RobotSensors_Update();
    osDelay(50U);
  }
}

/*
 * 遥测任务。
 *
 * 调试模式下每 500ms 通过 UART1 输出三行可读文本；正式模式下发送 CRC16
 * 二进制状态帧。它被设置为较低优先级，避免串口打印影响 10ms 控制任务。
 */
static void RobotApp_TelemetryTask(void *argument)
{
#if !ROBOT_UART1_DEBUG_ONLY
  uint8_t send_now;
#endif

  (void)argument;

  for (;;)
  {
    osDelay(ROBOT_TELEMETRY_PERIOD_MS);

#if ROBOT_UART1_DEBUG_ONLY
    send_status();
#else
    ctx_lock();
    send_now = s_ctx.telemetry_request;
    s_ctx.telemetry_request = 0U;
    ctx_unlock();

    if (send_now != 0U)
    {
      send_status();
    }
    else
    {
      send_status();
    }
#endif
  }
}
