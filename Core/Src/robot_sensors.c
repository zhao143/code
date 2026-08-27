#include "robot_sensors.h"
#include "i2c.h"
#include "main.h"
#include "robot_config.h"
#include "robot_delay.h"
#include "cmsis_os.h"
#include <string.h>

static RobotSensorsData_t s_data;
static uint32_t s_last_fast_ms;
static uint32_t s_last_env_ms;
static uint32_t s_last_ds_start_ms;
static uint8_t s_ds_pending;

/*
 * 在 FreeRTOS 已启动和未启动两种情况下都可用的毫秒延时包装。
 *
 * DHT30 需要等测量转换完成，FreeRTOS 运行时优先用 osDelay 让出 CPU；如果
 * 还没进调度器，就退回到 HAL_Delay。
 */
static void sensor_delay_ms(uint32_t ms)
{
  if (osKernelGetState() == osKernelRunning)
  {
    osDelay(ms);
  }
  else
  {
    HAL_Delay(ms);
  }
}

/*
 * 设置或清除某个传感器的有效标志位。
 *
 * valid=1 表示这次读到了可信数据；valid=0 表示本轮读取失败。上层通过
 * valid_flags 就能知道哪些传感器当前是可用的。
 */
static void set_valid(uint16_t flag, uint8_t valid)
{
  if (valid != 0U)
  {
    s_data.valid_flags |= flag;
  }
  else
  {
    s_data.valid_flags &= (uint16_t)~flag;
  }
}

/*
 * 读取 I2C 寄存器中的 16 位大端数据。
 *
 * 大多数传感器寄存器都是高字节在前，所以这里统一封装一下，避免每个传感器
 * 都自己处理字节顺序。
 */
static uint8_t i2c_read_reg16(uint8_t addr, uint8_t reg, uint16_t *value)
{
  uint8_t buf[2];

  if (HAL_I2C_Mem_Read(&hi2c2, (uint16_t)addr << 1, reg, I2C_MEMADD_SIZE_8BIT,
                       buf, sizeof(buf), 50U) != HAL_OK)
  {
    return 0U;
  }

  *value = ((uint16_t)buf[0] << 8) | buf[1];
  return 1U;
}

/*
 * 向 I2C 设备写入 8 位寄存器值。
 *
 * 这是 MPU6050 初始化时最常用的写寄存器封装。
 */
static uint8_t i2c_write_reg8(uint8_t addr, uint8_t reg, uint8_t value)
{
  return (HAL_I2C_Mem_Write(&hi2c2, (uint16_t)addr << 1, reg, I2C_MEMADD_SIZE_8BIT,
                            &value, 1U, 50U) == HAL_OK) ? 1U : 0U;
}

/*
 * 初始化 INA219。
 *
 * 这里主要是给它写一个默认配置，让后续可以稳定读取 bus voltage。
 * 本项目当前只把 INA219 当电压表使用，不涉及电流和分流电阻标定。
 */
static uint8_t ina219_init(void)
{
  uint8_t cfg[2] = {0x39U, 0x9FU};

  return (HAL_I2C_Mem_Write(&hi2c2, (uint16_t)INA219_I2C_ADDR << 1, 0x00U,
                            I2C_MEMADD_SIZE_8BIT, cfg, sizeof(cfg), 50U) == HAL_OK) ? 1U : 0U;
}

/*
 * 更新 INA219 电池电压。
 *
 * 读取 bus voltage 寄存器后换算成毫伏。这里不做电流检测，所以只保留电池
 * 电压监控这一项，足够做欠压保护。
 */
static void ina219_update(void)
{
  uint16_t raw;

  if (i2c_read_reg16(INA219_I2C_ADDR, 0x02U, &raw) == 0U)
  {
    set_valid(ROBOT_SENSOR_INA219_VALID, 0U);
    return;
  }

  s_data.battery_mv = (uint16_t)(((raw >> 3) & 0x1FFFU) * 4U);
  set_valid(ROBOT_SENSOR_INA219_VALID, 1U);
}

/*
 * 唤醒 MPU6050 并配置几个常用量程。
 *
 * 这里使用比较保守的默认配置，先保证能读到加速度、角速度和温度，后续如果
 * 想换量程，只改这个函数即可。
 */
static uint8_t mpu6050_init(void)
{
  if (i2c_write_reg8(MPU6050_I2C_ADDR, 0x6BU, 0x00U) == 0U)
  {
    return 0U;
  }
  i2c_write_reg8(MPU6050_I2C_ADDR, 0x1AU, 0x03U);
  i2c_write_reg8(MPU6050_I2C_ADDR, 0x1BU, 0x00U);
  i2c_write_reg8(MPU6050_I2C_ADDR, 0x1CU, 0x00U);
  return 1U;
}

/*
 * 把大端字节数组转成有符号 16 位数。
 *
 * MPU6050 的寄存器数据就是这种格式，所以这个函数用来把原始字节拼成数值。
 */
static int16_t be_i16(const uint8_t *buf)
{
  return (int16_t)(((uint16_t)buf[0] << 8) | buf[1]);
}

/*
 * 读取 MPU6050 的加速度、角速度和内部温度。
 *
 * 失败时只清除有效标志，不会把系统卡死。这样即使模块没接好，底板也还能
 * 继续跑电机和其它传感器。
 */
static void mpu6050_update(void)
{
  uint8_t buf[14];
  int16_t temp_raw;

  if (HAL_I2C_Mem_Read(&hi2c2, (uint16_t)MPU6050_I2C_ADDR << 1, 0x3BU,
                       I2C_MEMADD_SIZE_8BIT, buf, sizeof(buf), 50U) != HAL_OK)
  {
    set_valid(ROBOT_SENSOR_MPU6050_VALID, 0U);
    return;
  }

  s_data.imu_accel_raw[0] = be_i16(&buf[0]);
  s_data.imu_accel_raw[1] = be_i16(&buf[2]);
  s_data.imu_accel_raw[2] = be_i16(&buf[4]);
  temp_raw = be_i16(&buf[6]);
  s_data.imu_temp_c_x100 = (int16_t)(((int32_t)temp_raw * 100) / 340 + 3653);
  s_data.imu_gyro_raw[0] = be_i16(&buf[8]);
  s_data.imu_gyro_raw[1] = be_i16(&buf[10]);
  s_data.imu_gyro_raw[2] = be_i16(&buf[12]);
  set_valid(ROBOT_SENSOR_MPU6050_VALID, 1U);
}

/*
 * 计算 AHT30 / DHT30 数据包的 CRC8。
 *
 * AHT30/DHT30 常见模块地址是 0x38，测量命令是 0xAC 0x33 0x00。它的数据包
 * 最后一字节是 CRC8，算法仍然使用多项式 0x31、初值 0xFF。
 */
static uint8_t aht_crc8(const uint8_t *data, uint8_t len)
{
  uint8_t crc = 0xFFU;
  uint8_t i;

  while (len-- != 0U)
  {
    crc ^= *data++;
    for (i = 0U; i < 8U; i++)
    {
      crc = (crc & 0x80U) ? (uint8_t)((crc << 1) ^ 0x31U) : (uint8_t)(crc << 1);
    }
  }

  return crc;
}

/*
 * 触发 DHT30/AHT30 一次测量并读回温湿度。
 *
 * 你的模块地址是 0x38，更像 AHT30/DHT30 协议，而不是 SHT30 的 0x44/0x45
 * 协议。这里发送 0xAC 0x33 0x00，等待约 80ms，然后读取 7 字节：
 *
 *   buf[0]      ：状态字节，bit7=1 表示传感器忙；
 *   buf[1..3]   ：20 位湿度原始值；
 *   buf[3..5]   ：20 位温度原始值；
 *   buf[6]      ：CRC8。
 *
 * 温度输出单位是 0.01°C，湿度输出单位是 0.01%RH，方便状态行直接打印。
 */
static void dht30_update(void)
{
  uint8_t cmd[3] = {0xACU, 0x33U, 0x00U};
  uint8_t buf[7];
  uint32_t raw_t;
  uint32_t raw_h;
  uint8_t crc_ok;

  if (HAL_I2C_Master_Transmit(&hi2c2, (uint16_t)DHT30_I2C_ADDR << 1,
                              cmd, sizeof(cmd), 50U) != HAL_OK)
  {
    set_valid(ROBOT_SENSOR_DHT30_VALID, 0U);
    return;
  }

  sensor_delay_ms(80U);

  if (HAL_I2C_Master_Receive(&hi2c2, (uint16_t)DHT30_I2C_ADDR << 1,
                             buf, sizeof(buf), 50U) != HAL_OK)
  {
    set_valid(ROBOT_SENSOR_DHT30_VALID, 0U);
    return;
  }

  if ((buf[0] & 0x80U) != 0U)
  {
    set_valid(ROBOT_SENSOR_DHT30_VALID, 0U);
    return;
  }

  /*
   * 按 DHT30 说明书校验 Status、湿度 20 位数据和温度 20 位数据，也就是
   * buf[0] 到 buf[5] 共 6 个字节。CRC 初值为 0xFF，多项式为 0x31。
   */
  crc_ok = (aht_crc8(buf, 6U) == buf[6]) ? 1U : 0U;
  if (crc_ok == 0U)
  {
    set_valid(ROBOT_SENSOR_DHT30_VALID, 0U);
    return;
  }

  raw_h = ((uint32_t)buf[1] << 12) | ((uint32_t)buf[2] << 4) | ((uint32_t)buf[3] >> 4);
  raw_t = (((uint32_t)buf[3] & 0x0FU) << 16) | ((uint32_t)buf[4] << 8) | buf[5];

  s_data.env_humi_x100 = (uint16_t)(((uint64_t)raw_h * 10000ULL) / 1048576ULL);

  /*
   * 温度换算中的 raw_t * 20000 可能超过 32 位有符号整数范围。若直接用
   * int32_t 相乘，室温附近就会发生溢出，串口可能显示成 -52.xx°C。这里使用
   * 64 位中间结果，最后再保存为 0.01°C 单位的 int16_t。
   */
  s_data.env_temp_c_x100 = (int16_t)(((uint64_t)raw_t * 20000ULL) / 1048576ULL - 5000ULL);
  set_valid(ROBOT_SENSOR_DHT30_VALID, 1U);
}

/*
 * 把 DS18B20 引脚切换成开漏输出并拉低。
 *
 * 单总线时序里，主机拉低总线的动作都要通过这个函数完成。
 */
static void ds18b20_output_low(void)
{
  GPIO_InitTypeDef GPIO_InitStruct;

  HAL_GPIO_WritePin(DS18B20_GPIO_Port, DS18B20_Pin, GPIO_PIN_RESET);

  GPIO_InitStruct.Pin = DS18B20_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_OD;
  GPIO_InitStruct.Pull = GPIO_PULLUP;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_HIGH;
  HAL_GPIO_Init(DS18B20_GPIO_Port, &GPIO_InitStruct);
}

/*
 * 释放 DS18B20 总线。
 *
 * 释放后总线由上拉电阻拉高，或者由从设备拉低表示应答/数据位。
 */
static void ds18b20_release(void)
{
  GPIO_InitTypeDef GPIO_InitStruct;

  HAL_GPIO_WritePin(DS18B20_GPIO_Port, DS18B20_Pin, GPIO_PIN_SET);

  GPIO_InitStruct.Pin = DS18B20_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_PULLUP;
  HAL_GPIO_Init(DS18B20_GPIO_Port, &GPIO_InitStruct);
}

/*
 * 对 DS18B20 发复位脉冲并检测存在脉冲。
 *
 * 返回 1 表示器件存在，返回 0 表示总线上没有检测到器件。
 */
static uint8_t ds18b20_reset(void)
{
  uint8_t presence;

  ds18b20_output_low();
  RobotDelay_Us(480U);
  ds18b20_release();
  RobotDelay_Us(70U);
  presence = (HAL_GPIO_ReadPin(DS18B20_GPIO_Port, DS18B20_Pin) == GPIO_PIN_RESET) ? 1U : 0U;
  RobotDelay_Us(410U);

  return presence;
}

/*
 * 向 DS18B20 写一个单比特。
 *
 * 0 和 1 的时序不同，所以需要严格按协议控制拉低和释放的时间。
 */
static void ds18b20_write_bit(uint8_t bit)
{
  ds18b20_output_low();
  if (bit != 0U)
  {
    RobotDelay_Us(6U);
    ds18b20_release();
    RobotDelay_Us(64U);
  }
  else
  {
    RobotDelay_Us(60U);
    ds18b20_release();
    RobotDelay_Us(10U);
  }
}

/*
 * 从 DS18B20 读一个单比特。
 *
 * 先短暂拉低总线，再释放总线并在合适的采样窗口读取电平。
 */
static uint8_t ds18b20_read_bit(void)
{
  uint8_t bit;

  ds18b20_output_low();
  RobotDelay_Us(6U);
  ds18b20_release();
  RobotDelay_Us(9U);
  bit = (HAL_GPIO_ReadPin(DS18B20_GPIO_Port, DS18B20_Pin) == GPIO_PIN_SET) ? 1U : 0U;
  RobotDelay_Us(55U);

  return bit;
}

/*
 * 向 DS18B20 写一个字节。
 *
 * 单总线协议里，字节就是由 8 个低位先出的 bit 组成。
 */
static void ds18b20_write_byte(uint8_t value)
{
  uint8_t i;

  for (i = 0U; i < 8U; i++)
  {
    ds18b20_write_bit((uint8_t)(value & 0x01U));
    value >>= 1;
  }
}

/*
 * 从 DS18B20 读一个字节。
 *
 * 由 8 次读 bit 组成，读出来的低位在前。
 */
static uint8_t ds18b20_read_byte(void)
{
  uint8_t i;
  uint8_t value = 0U;

  for (i = 0U; i < 8U; i++)
  {
    value |= (uint8_t)(ds18b20_read_bit() << i);
  }

  return value;
}

/*
 * 计算 DS18B20 scratchpad 的 CRC8。
 *
 * 读温度前先做校验，避免总线干扰导致脏数据直接进入状态输出。
 */
static uint8_t ds18b20_crc8(const uint8_t *data, uint8_t len)
{
  uint8_t crc = 0U;
  uint8_t i;

  while (len-- != 0U)
  {
    crc ^= *data++;
    for (i = 0U; i < 8U; i++)
    {
      crc = (crc & 0x01U) ? (uint8_t)((crc >> 1) ^ 0x8CU) : (uint8_t)(crc >> 1);
    }
  }

  return crc;
}

/*
 * 触发 DS18B20 转换。
 *
 * 这里只做“开始转换”，并不马上读温度。真正读数据要等转换时间过去以后再
 * 调 ds18b20_read_temperature。
 */
static uint8_t ds18b20_start_convert(void)
{
  if (ds18b20_reset() == 0U)
  {
    return 0U;
  }

  ds18b20_write_byte(0xCCU);
  ds18b20_write_byte(0x44U);
  return 1U;
}

/*
 * 读取 DS18B20 当前温度。
 *
 * 返回值为 1 表示成功，0 表示失败。温度输出单位是 0.01°C，便于上层直接
 * 做阈值判断和文本打印。
 */
static uint8_t ds18b20_read_temperature(int16_t *temp_c_x100)
{
  uint8_t scratch[9];
  int16_t raw;
  uint8_t i;

  if (ds18b20_reset() == 0U)
  {
    return 0U;
  }

  ds18b20_write_byte(0xCCU);
  ds18b20_write_byte(0xBEU);

  for (i = 0U; i < sizeof(scratch); i++)
  {
    scratch[i] = ds18b20_read_byte();
  }

  if (ds18b20_crc8(scratch, 8U) != scratch[8])
  {
    return 0U;
  }

  raw = (int16_t)(((uint16_t)scratch[1] << 8) | scratch[0]);
  *temp_c_x100 = (int16_t)(((int32_t)raw * 100) / 16);
  return 1U;
}

/*
 * 管理 DS18B20 的“先触发、后读取”两阶段流程。
 *
 * 因为 DS18B20 转换温度需要时间，所以这里先启动一次转换，过一段时间后再
 * 读取结果。这样不会在任务里长时间阻塞。
 */
static void ds18b20_update(uint32_t now)
{
  int16_t temp;

  if (s_ds_pending != 0U)
  {
    if ((uint32_t)(now - s_last_ds_start_ms) < 800U)
    {
      return;
    }

    if (ds18b20_read_temperature(&temp) != 0U)
    {
      s_data.battery_temp_c_x100 = temp;
      set_valid(ROBOT_SENSOR_DS18B20_VALID, 1U);
    }
    else
    {
      set_valid(ROBOT_SENSOR_DS18B20_VALID, 0U);
    }

    s_ds_pending = 0U;
    return;
  }

  if ((uint32_t)(now - s_last_ds_start_ms) >= ROBOT_SENSOR_SLOW_PERIOD_MS)
  {
    if (ds18b20_start_convert() != 0U)
    {
      s_ds_pending = 1U;
      s_last_ds_start_ms = now;
    }
    else
    {
      set_valid(ROBOT_SENSOR_DS18B20_VALID, 0U);
      s_last_ds_start_ms = now;
    }
  }
}

/*
 * 初始化所有传感器相关缓存和外设。
 *
 * 这个函数不会因为某个模块失败就退出，它的目标是“尽量让能工作的先工作”，
 * 后续再通过 valid_flags 判断哪些模块当前可用。
 */
void RobotSensors_Init(void)
{
  memset(&s_data, 0, sizeof(s_data));
  ina219_init();
  mpu6050_init();
  ds18b20_release();
}

/*
 * 周期性更新全部传感器。
 *
 * 快速项：INA219 和 MPU6050，大约 100ms 采样一次。
 * 慢速项：DHT30 和 DS18B20，大约 1s 采样一次。
 */
void RobotSensors_Update(void)
{
  uint32_t now = HAL_GetTick();

  if ((uint32_t)(now - s_last_fast_ms) >= ROBOT_SENSOR_FAST_PERIOD_MS)
  {
    s_last_fast_ms = now;
    ina219_update();
    mpu6050_update();
  }

  if ((uint32_t)(now - s_last_env_ms) >= ROBOT_SENSOR_SLOW_PERIOD_MS)
  {
    s_last_env_ms = now;
    dht30_update();
  }

  ds18b20_update(now);
}

/*
 * 拷贝最近一次传感器缓存。
 *
 * 这是给上层状态输出用的快照接口。调用时如果 out 为 NULL，就什么也不做。
 */
void RobotSensors_Get(RobotSensorsData_t *out)
{
  if (out != 0)
  {
    *out = s_data;
  }
}

/*
 * 扫描 I2C2 总线上的设备。
 *
 * 这个函数主要给你调试接线用。比如你不确定 DHT30、MPU6050、INA219 到底
 * 有没有挂好，就可以临时调用这个函数看看地址列表。
 */
uint8_t RobotSensors_ScanI2C(uint8_t *addresses, uint8_t max_count)
{
  uint8_t addr;
  uint8_t count = 0U;

  for (addr = 1U; addr < 0x7FU; addr++)
  {
    if (HAL_I2C_IsDeviceReady(&hi2c2, (uint16_t)addr << 1, 1U, 5U) == HAL_OK)
    {
      if (addresses != 0 && count < max_count)
      {
        addresses[count] = addr;
      }
      count++;
    }
  }

  return count;
}
