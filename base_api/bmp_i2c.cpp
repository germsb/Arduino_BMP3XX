#include "../bmp_i2c.h"
TwoWire *_BMP3_i2c;

static int8_t i2c_write(uint8_t dev_id, uint8_t reg_addr, uint8_t *reg_data, uint16_t len);
static int8_t i2c_read(uint8_t dev_id, uint8_t reg_addr, uint8_t *reg_data, uint16_t len);
static void delay_msec(uint32_t ms);

///constucteur

BMP3_I2C::BMP3_I2C(uint8_t addr, TwoWire *theWire)
{
  printf("helllo la terre de bmp_i2c");
  _BMP3_i2c = theWire;
  i2c_addr = addr;
}

/// public

bool BMP3_I2C::init()
{
  _BMP3_i2c->begin();
  the_sensor.dev_id = i2c_addr;
  the_sensor.intf = BMP3_I2C_INTF;
  the_sensor.read = &i2c_read;
  the_sensor.write = &i2c_write;
  return initSensor();
}

bool BMP3_I2C::setSensorInSleepMode(void)
{
  uint8_t rslt = BMP3_OK;
  _forcedModeEnabled = false;
  rslt = bmp3_soft_reset(&the_sensor);
  return rslt == BMP3_OK;
}

bool BMP3_I2C::getSensorData(bmp_data &sensorData, bool computeAltitude = false)
{
  int8_t rslt;

  /* Variable used to select the sensor component */
  uint8_t sensor_comp;

  /* if forced mode, set power mode of the sensor at each reading*/
  if (_forcedModeEnabled)
  {
    rslt = bmp3_set_op_mode(&the_sensor);
    if (rslt != BMP3_OK)
    {
      printf("error set power mode \n");
      if (rslt == BMP3_E_INVALID_ODR_OSR_SETTINGS)
        printf("BMP3_E_INVALID_ODR_OSR_SETTINGS \n");
      return false;
    }
  }

  /* Variable used to store the compensated data */
  struct bmp3_data data;

  /* Sensor component selection */
  sensor_comp = BMP3_PRESS | BMP3_TEMP;

  /* Temperature and Pressure data are read and stored in the bmp3_data instance */
  rslt = bmp3_get_sensor_data(sensor_comp, &data, &the_sensor);
  if (rslt != BMP3_OK)
    return false;

  /* Save the temperature and pressure data */
  sensorData.temperature = data.temperature;
  sensorData.pressure = data.pressure;
  if (computeAltitude)
  {
    sensorData.altitude = calcAltitude(sensorData.pressure, sensorData.seaLevelPressure);
  }
  return true;
}

bool BMP3_I2C::calcSeaLevelPressure(bmp_data &sensorData, double YourActualAltitude)
{
  if (sensorData.pressure != -99999.0)
  {
    sensorData.seaLevelPressure = sensorData.pressure / pow(1.0 - (YourActualAltitude / 44330.0), 5.255);
    return true;
  }
  return false;
}

/// protected

bool BMP3_I2C::initSensor()
{
  _forcedModeEnabled = false;
  the_sensor.delay_ms = delay_msec;
  int8_t rslt = BMP3_OK;
  rslt = bmp3_init(&the_sensor);
#ifdef BMP3XX_DEBUG
  Serial.print("Result: ");
  Serial.println(rslt);
#endif

  if (rslt != BMP3_OK)
    return false;

#ifdef BMP3XX_DEBUG
  Serial.print("T1 = ");
  Serial.println(the_sensor.calib_data.reg_calib_data.par_t1);
  Serial.print("T2 = ");
  Serial.println(the_sensor.calib_data.reg_calib_data.par_t2);
  Serial.print("T3 = ");
  Serial.println(the_sensor.calib_data.reg_calib_data.par_t3);
  Serial.print("P1 = ");
  Serial.println(the_sensor.calib_data.reg_calib_data.par_p1);
  Serial.print("P2 = ");
  Serial.println(the_sensor.calib_data.reg_calib_data.par_p2);
  Serial.print("P3 = ");
  Serial.println(the_sensor.calib_data.reg_calib_data.par_p3);
  Serial.print("P4 = ");
  Serial.println(the_sensor.calib_data.reg_calib_data.par_p4);
  Serial.print("P5 = ");
  Serial.println(the_sensor.calib_data.reg_calib_data.par_p5);
  Serial.print("P6 = ");
  Serial.println(the_sensor.calib_data.reg_calib_data.par_p6);
  Serial.print("P7 = ");
  Serial.println(the_sensor.calib_data.reg_calib_data.par_p7);
  Serial.print("P8 = ");
  Serial.println(the_sensor.calib_data.reg_calib_data.par_p8);
  Serial.print("P9 = ");
  Serial.println(the_sensor.calib_data.reg_calib_data.par_p9);
  Serial.print("P10 = ");
  Serial.println(the_sensor.calib_data.reg_calib_data.par_p10);
  Serial.print("P11 = ");
  Serial.println(the_sensor.calib_data.reg_calib_data.par_p11);
  //Serial.print("T lin = "); Serial.println(the_sensor.calib_data.reg_calib_data.t_lin);
#endif

  return true;
}

/// private

bool BMP3_I2C::setConfig(uint8_t TemperatureOversampling, uint8_t PressureOversampling, uint8_t IIRFilter, uint8_t PowerMode, uint8_t OutputDataRate, bool DataReadyInterrupt)
{
  int8_t rslt = BMP3_OK;
  uint16_t settings_sel = 0;

  /* Select the pressure and temperature sensor to be enabled */
  the_sensor.settings.press_en = BMP3_ENABLE;
  the_sensor.settings.temp_en = BMP3_ENABLE;
  settings_sel |= BMP3_PRESS_EN_SEL | BMP3_TEMP_EN_SEL;

  /* set pressure settings */
  if (PressureOversampling > BMP3_OVERSAMPLING_32X)
  {
    printf("error pressure oversampling \n");
    return false;
  }
  the_sensor.settings.odr_filter.press_os = PressureOversampling;
  settings_sel |= BMP3_PRESS_OS_SEL;

  /* set temperature settings */
  if (TemperatureOversampling > BMP3_OVERSAMPLING_32X)
  {
    printf("error temp oversampling \n");
    return false;
  }
  the_sensor.settings.odr_filter.temp_os = TemperatureOversampling;
  settings_sel |= BMP3_TEMP_OS_SEL;

  /* set iirFilter settings */
  if (IIRFilter > BMP3_IIR_FILTER_COEFF_127)
  {
    printf("error iir filter \n");
    return false;
  }
  the_sensor.settings.odr_filter.iir_filter = IIRFilter;
  settings_sel |= BMP3_IIR_FILTER_SEL;

  /* set output data rate */
  if (PowerMode == BMP3_NORMAL_MODE)
  {
    if (OutputDataRate > BMP3_ODR_0_001_HZ)
    {
      printf("error data rate \n");
      return false;
    }
    the_sensor.settings.odr_filter.odr = OutputDataRate;
    settings_sel |= BMP3_ODR_SEL;
  }

  /* set interrupt settings */
  //TODO check if interrupt settings are ok
  the_sensor.settings.int_settings.drdy_en =
      (PowerMode == BMP3_NORMAL_MODE && DataReadyInterrupt) ? BMP3_ENABLE : BMP3_DISABLE;
  //the_sensor.settings.int_settings.latch = BMP3_INT_PIN_NON_LATCH; //BMP3_INT_PIN_LATCH
  //the_sensor.settings.int_settings.level = BMP3_INT_PIN_ACTIVE_HIGH; //BMP3_INT_PIN_ACTIVE_LOW
  //the_sensor.settings.int_settings.output_mode = BMP3_INT_PIN_PUSH_PULL; //BMP3_INT_PIN_OPEN_DRAIN
  settings_sel |= BMP3_DRDY_EN_SEL; // | BMP3_LEVEL_SEL | BMP3_LATCH_SEL | BMP3_OUTPUT_MODE_SEL;

  /* set sensor settings */
  rslt = bmp3_set_sensor_settings(settings_sel, &the_sensor);
  if (rslt != BMP3_OK)
  {
    printf("error set sensor settings \n");
    return false;
  }

#ifdef BMP3XX_DEBUG
  bmp3_get_sensor_settings(&the_sensor);
  Serial.printf("temp en: %u, temp os: %u, press en: %u, press os: %u, odr: %u", the_sensor.settings.temp_en, the_sensor.settings.odr_filter.temp_os, the_sensor.settings.press_en, the_sensor.settings.odr_filter.press_os, the_sensor.settings.odr_filter.odr);
#endif

  /* set power mode */
  if (PowerMode == BMP3_NORMAL_MODE || PowerMode == BMP3_FORCED_MODE || PowerMode == BMP3_SLEEP_MODE)
  {
    _forcedModeEnabled = PowerMode == BMP3_FORCED_MODE;
    the_sensor.settings.op_mode = PowerMode;

    rslt = bmp3_set_op_mode(&the_sensor);
    if (rslt != BMP3_OK)
    {
      printf("error set power mode \n");
      if (rslt == BMP3_E_INVALID_ODR_OSR_SETTINGS)
        printf("BMP3_E_INVALID_ODR_OSR_SETTINGS \n");
      return false;
    }
  }
  else
    return false;

  return true;
}

double BMP3_I2C::calcAltitude(double atmospheriquePressure, double seaLevelPressure)
{
  return 44330.0 * (1.0 - pow(atmospheriquePressure / seaLevelPressure, 0.1902949));
}

/// static

/**************************************************************************/
/*!
    @brief  Reads 8 bit values over I2C
*/
/**************************************************************************/
int8_t i2c_read(uint8_t dev_id, uint8_t reg_addr, uint8_t *reg_data, uint16_t len)
{
#ifdef BMP3XX_DEBUG
  Serial.print("\tI2C $");
  Serial.print(reg_addr, HEX);
  Serial.print(" => ");
#endif

  _BMP3_i2c->beginTransmission((uint8_t)dev_id);
  _BMP3_i2c->write((uint8_t)reg_addr);
  _BMP3_i2c->endTransmission();
  if (len != _BMP3_i2c->requestFrom((uint8_t)dev_id, (byte)len))
  {
#ifdef BMP3XX_DEBUG
    Serial.print("Failed to read ");
    Serial.print(len);
    Serial.print(" bytes from ");
    Serial.println(dev_id, HEX);
#endif
    return 1;
  }
  while (len--)
  {
    *reg_data = (uint8_t)_BMP3_i2c->read();
#ifdef BMP3XX_DEBUG
    Serial.print("0x");
    Serial.print(*reg_data, HEX);
    Serial.print(", ");
#endif
    reg_data++;
  }
#ifdef BMP3XX_DEBUG
  Serial.println("");
#endif
  return 0;
}

/**************************************************************************/
/*!
    @brief  Writes 8 bit values over I2C
*/
/**************************************************************************/
int8_t i2c_write(uint8_t dev_id, uint8_t reg_addr, uint8_t *reg_data, uint16_t len)
{
#ifdef BMP3XX_DEBUG
  Serial.print("\tI2C $");
  Serial.print(reg_addr, HEX);
  Serial.print(" <= ");
#endif
  _BMP3_i2c->beginTransmission((uint8_t)dev_id);
  _BMP3_i2c->write((uint8_t)reg_addr);
  while (len--)
  {
    _BMP3_i2c->write(*reg_data);
#ifdef BMP3XX_DEBUG
    Serial.print("0x");
    Serial.print(*reg_data, HEX);
    Serial.print(", ");
#endif
    reg_data++;
  }
  _BMP3_i2c->endTransmission();
#ifdef BMP3XX_DEBUG
  Serial.println("");
#endif
  return 0;
}

static void delay_msec(uint32_t ms)
{
  delay(ms);
}
