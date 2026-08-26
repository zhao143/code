#include "robot_app.h"
#include "main.h"
#include "robot_comm.h"
#include "robot_config.h"
#include "robot_delay.h"
#include "robot_encoder.h"
#include "robot_motor.h"
#include "robot_sensors.h"
#include "cmsis_os.h"
#include <string.h>

typedef struct
{
  RobotAppState_t state;
  uint16_t faults;
  uint8_t motion_mode;
  int16_t target_a;
  int16_t target_b;
  int16_t pwm_a;
  int16_t pwm_b;
  int32_t integ_a;
  int32_t integ_b;
  uint32_t last_cmd_ms;
  uint32_t stall_a_start_ms;
  uint32_t stall_b_start_ms;
  uint8_t command_seen;
  uint8_t telemetry_request;
  uint8_t manual_buzzer;
  uint8_t manual_fan;
  uint8_t auto_fan;
  uint8_t buzzer_on;
  RobotEncoderData_t encoder;
  RobotSensorsData_t sensors;
} RobotAppContext_t;

static RobotAppContext_t s_ctx;
static osMutexId_t s_ctx_mutex;

static const osThreadAttr_t s_control_attr =
{
  .name = "ctrlTask",
  .stack_size = 256 * 4,
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
static void RobotApp_OnFrame(const RobotCommFrame_t *frame);

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

static int16_t read_i16_le(const uint8_t *data)
{
  return (int16_t)((uint16_t)data[0] | ((uint16_t)data[1] << 8));
}

static void put_u8(uint8_t *payload, uint8_t *pos, uint8_t value)
{
  payload[(*pos)++] = value;
}

static void put_u16(uint8_t *payload, uint8_t *pos, uint16_t value)
{
  payload[(*pos)++] = (uint8_t)(value & 0xFFU);
  payload[(*pos)++] = (uint8_t)(value >> 8);
}

static void put_i16(uint8_t *payload, uint8_t *pos, int16_t value)
{
  put_u16(payload, pos, (uint16_t)value);
}

static void put_i32(uint8_t *payload, uint8_t *pos, int32_t value)
{
  payload[(*pos)++] = (uint8_t)((uint32_t)value & 0xFFU);
  payload[(*pos)++] = (uint8_t)(((uint32_t)value >> 8) & 0xFFU);
  payload[(*pos)++] = (uint8_t)(((uint32_t)value >> 16) & 0xFFU);
  payload[(*pos)++] = (uint8_t)(((uint32_t)value >> 24) & 0xFFU);
}

static void ctx_lock(void)
{
  if (s_ctx_mutex != 0 && osKernelGetState() == osKernelRunning)
  {
    (void)osMutexAcquire(s_ctx_mutex, 20U);
  }
}

static void ctx_unlock(void)
{
  if (s_ctx_mutex != 0 && osKernelGetState() == osKernelRunning)
  {
    (void)osMutexRelease(s_ctx_mutex);
  }
}

static void apply_outputs(uint32_t now)
{
  uint8_t buzzer_on = 0U;
  uint8_t fan_on;

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
                     ((s_ctx.faults & ROBOT_FAULT_OVER_TEMP) != 0U));

  if (s_ctx.manual_buzzer != 0U)
  {
    buzzer_on = 1U;
  }
  else if (s_ctx.faults != 0U)
  {
    buzzer_on = (((now / 200U) & 0x01U) != 0U) ? 1U : 0U;
  }

  s_ctx.buzzer_on = buzzer_on;
  HAL_GPIO_WritePin(Beep_GPIO_Port, Beep_Pin, buzzer_on ? GPIO_PIN_SET : GPIO_PIN_RESET);
  HAL_GPIO_WritePin(Relay_GPIO_Port, Relay_Pin, fan_on ? GPIO_PIN_SET : GPIO_PIN_RESET);
}

static void update_faults(uint32_t now)
{
  if (s_ctx.state == ROBOT_APP_STATE_RUN && s_ctx.command_seen != 0U &&
      (uint32_t)(now - s_ctx.last_cmd_ms) > ROBOT_CMD_TIMEOUT_MS)
  {
    s_ctx.faults |= ROBOT_FAULT_COMM_TIMEOUT;
  }

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

static void control_step(void)
{
  uint32_t now = HAL_GetTick();

  RobotEncoder_Sample(ROBOT_CONTROL_PERIOD_MS, &s_ctx.encoder);
  RobotSensors_Get(&s_ctx.sensors);

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

  (void)RobotComm_SendFrame(ROBOT_CMD_STATUS, payload, pos);
}

static void handle_set_motion(const RobotCommFrame_t *frame)
{
  if (frame->length < 5U)
  {
    return;
  }

  s_ctx.target_a = read_i16_le(&frame->payload[0]);
  s_ctx.target_b = read_i16_le(&frame->payload[2]);
  s_ctx.motion_mode = frame->payload[4];
  if (s_ctx.motion_mode != ROBOT_MOTION_MODE_SPEED)
  {
    s_ctx.motion_mode = ROBOT_MOTION_MODE_PWM;
  }

  s_ctx.last_cmd_ms = HAL_GetTick();
  s_ctx.command_seen = 1U;

  if (s_ctx.faults == 0U)
  {
    if (s_ctx.target_a == 0 && s_ctx.target_b == 0)
    {
      s_ctx.state = ROBOT_APP_STATE_READY;
    }
    else
    {
      s_ctx.state = ROBOT_APP_STATE_RUN;
    }
  }
}

static void handle_set_state(const RobotCommFrame_t *frame)
{
  uint8_t cmd;

  if (frame->length < 1U)
  {
    return;
  }

  cmd = frame->payload[0];

  if (cmd == ROBOT_STATE_CMD_IDLE)
  {
    s_ctx.target_a = 0;
    s_ctx.target_b = 0;
    s_ctx.command_seen = 0U;
    s_ctx.state = ROBOT_APP_STATE_SAFE_IDLE;
  }
  else if (cmd == ROBOT_STATE_CMD_ENABLE)
  {
    if (s_ctx.faults == 0U)
    {
      s_ctx.state = ROBOT_APP_STATE_READY;
      s_ctx.last_cmd_ms = HAL_GetTick();
    }
  }
  else if (cmd == ROBOT_STATE_CMD_CLEAR_FAULT)
  {
    s_ctx.faults = 0U;
    s_ctx.target_a = 0;
    s_ctx.target_b = 0;
    s_ctx.command_seen = 0U;
    s_ctx.state = ROBOT_APP_STATE_SAFE_IDLE;
  }
}

static void handle_set_output(const RobotCommFrame_t *frame)
{
  if (frame->length < 2U)
  {
    return;
  }

  s_ctx.manual_buzzer = frame->payload[0] ? 1U : 0U;
  s_ctx.manual_fan = frame->payload[1] ? 1U : 0U;
}

static void RobotApp_OnFrame(const RobotCommFrame_t *frame)
{
  ctx_lock();

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
    s_ctx.telemetry_request = 1U;
  }
  else if (frame->command == ROBOT_CMD_SET_OUTPUT)
  {
    handle_set_output(frame);
  }
  else if (frame->command == ROBOT_CMD_ESTOP)
  {
    s_ctx.faults |= ROBOT_FAULT_ESTOP;
    s_ctx.state = ROBOT_APP_STATE_FAULT;
  }

  ctx_unlock();
}

void RobotApp_BoardInit(void)
{
  memset(&s_ctx, 0, sizeof(s_ctx));
  s_ctx.state = ROBOT_APP_STATE_SAFE_IDLE;
  s_ctx.motion_mode = ROBOT_MOTION_MODE_PWM;

  RobotDelay_Init();
  RobotMotor_Init();
  RobotEncoder_Init();
  RobotSensors_Init();
  RobotComm_Init();
}

void RobotApp_CreateTasks(void)
{
  s_ctx_mutex = osMutexNew(&s_ctx_mutex_attr);
  RobotComm_SetFrameHandler(RobotApp_OnFrame);

  (void)osThreadNew(RobotApp_ControlTask, 0, &s_control_attr);
  (void)osThreadNew(RobotApp_SensorTask, 0, &s_sensor_attr);
  (void)osThreadNew(RobotApp_TelemetryTask, 0, &s_telemetry_attr);
}

void RobotApp_DefaultTask(void *argument)
{
  (void)argument;

  for (;;)
  {
    osDelay(1000U);
  }
}

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
  out->target_a = s_ctx.target_a;
  out->target_b = s_ctx.target_b;
  out->pwm_a = s_ctx.pwm_a;
  out->pwm_b = s_ctx.pwm_b;
  out->fan_on = (uint8_t)((s_ctx.manual_fan != 0U) || (s_ctx.auto_fan != 0U));
  out->buzzer_on = s_ctx.buzzer_on;
  out->encoder = s_ctx.encoder;
  out->sensors = s_ctx.sensors;
  ctx_unlock();
}

void RobotApp_RequestTelemetry(void)
{
  ctx_lock();
  s_ctx.telemetry_request = 1U;
  ctx_unlock();
}

static void RobotApp_ControlTask(void *argument)
{
  uint32_t wake_tick = osKernelGetTickCount();

  (void)argument;

  for (;;)
  {
    RobotComm_ProcessRx();

    ctx_lock();
    control_step();
    ctx_unlock();

    wake_tick += ROBOT_CONTROL_PERIOD_MS;
    (void)osDelayUntil(wake_tick);
  }
}

static void RobotApp_SensorTask(void *argument)
{
  (void)argument;

  for (;;)
  {
    RobotSensors_Update();
    osDelay(50U);
  }
}

static void RobotApp_TelemetryTask(void *argument)
{
  uint8_t send_now;

  (void)argument;

  for (;;)
  {
    osDelay(ROBOT_TELEMETRY_PERIOD_MS);

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
  }
}
