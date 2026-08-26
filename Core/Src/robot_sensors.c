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

static uint8_t i2c_write_reg8(uint8_t addr, uint8_t reg, uint8_t value)
{
  return (HAL_I2C_Mem_Write(&hi2c2, (uint16_t)addr << 1, reg, I2C_MEMADD_SIZE_8BIT,
                            &value, 1U, 50U) == HAL_OK) ? 1U : 0U;
}

static uint8_t ina219_init(void)
{
  uint8_t cfg[2] = {0x39U, 0x9FU};

  return (HAL_I2C_Mem_Write(&hi2c2, (uint16_t)INA219_I2C_ADDR << 1, 0x00U,
                            I2C_MEMADD_SIZE_8BIT, cfg, sizeof(cfg), 50U) == HAL_OK) ? 1U : 0U;
}

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

static int16_t be_i16(const uint8_t *buf)
{
  return (int16_t)(((uint16_t)buf[0] << 8) | buf[1]);
}

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

static uint8_t sht_crc8(const uint8_t *data, uint8_t len)
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

static void dht30_update(void)
{
  uint8_t cmd[2] = {0x2CU, 0x06U};
  uint8_t buf[6];
  uint16_t raw_t;
  uint16_t raw_h;

  if (HAL_I2C_Master_Transmit(&hi2c2, (uint16_t)DHT30_I2C_ADDR << 1,
                              cmd, sizeof(cmd), 50U) != HAL_OK)
  {
    set_valid(ROBOT_SENSOR_DHT30_VALID, 0U);
    return;
  }

  sensor_delay_ms(20U);

  if (HAL_I2C_Master_Receive(&hi2c2, (uint16_t)DHT30_I2C_ADDR << 1,
                             buf, sizeof(buf), 50U) != HAL_OK)
  {
    set_valid(ROBOT_SENSOR_DHT30_VALID, 0U);
    return;
  }

  if (sht_crc8(&buf[0], 2U) != buf[2] || sht_crc8(&buf[3], 2U) != buf[5])
  {
    set_valid(ROBOT_SENSOR_DHT30_VALID, 0U);
    return;
  }

  raw_t = ((uint16_t)buf[0] << 8) | buf[1];
  raw_h = ((uint16_t)buf[3] << 8) | buf[4];
  s_data.env_temp_c_x100 = (int16_t)(-4500 + ((int32_t)17500 * raw_t) / 65535);
  s_data.env_humi_x100 = (uint16_t)(((uint32_t)10000U * raw_h) / 65535U);
  set_valid(ROBOT_SENSOR_DHT30_VALID, 1U);
}

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

static void ds18b20_release(void)
{
  GPIO_InitTypeDef GPIO_InitStruct;

  HAL_GPIO_WritePin(DS18B20_GPIO_Port, DS18B20_Pin, GPIO_PIN_SET);

  GPIO_InitStruct.Pin = DS18B20_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_PULLUP;
  HAL_GPIO_Init(DS18B20_GPIO_Port, &GPIO_InitStruct);
}

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

static void ds18b20_write_byte(uint8_t value)
{
  uint8_t i;

  for (i = 0U; i < 8U; i++)
  {
    ds18b20_write_bit((uint8_t)(value & 0x01U));
    value >>= 1;
  }
}

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

void RobotSensors_Init(void)
{
  memset(&s_data, 0, sizeof(s_data));
  ina219_init();
  mpu6050_init();
  ds18b20_release();
}

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

void RobotSensors_Get(RobotSensorsData_t *out)
{
  if (out != 0)
  {
    *out = s_data;
  }
}

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
