/**
 * @file DFRobot_BME68x.cpp
 *
 * @copyright   Copyright (c) 2010 DFRobot Co.Ltd (http://www.dfrobot.com)
 * @license     The MIT License (MIT)
 * @author Frank(jiehan.guo@dfrobot.com)
 * @maintainer [GDuang](yonglei.ren@dfrobot.com)
 * @version  V2.0
 * @date  2024-04-25
 * @url https://github.com/DFRobot/DFRobot_BME68x
 */
#include "DFRobot_BME68x.h"

static struct        bme68x_dev bme68x_sensor;
static struct        bme68x_field_data bme68x_data;
static uint8_t       convertCmd = (0x05 << 5) | (0x05 << 2) | (0x01);
static uint8_t       iaqReady_ = 0;

void bme68x_outputReady(int64_t timestamp, float iaq, uint8_t iaq_accuracy, float temperature,
                        float humidity, float pressure, float raw_temperature, float raw_humidity,
                        float gas, bsec_library_return_t bsec_status)
{
  if(iaq_accuracy != 0) bme68x_data.gas_index = iaq;
  bme68x_data.temperature = temperature;
  bme68x_data.humidity = humidity;
  bme68x_data.pressure = pressure;
  if(iaq_accuracy != 0) iaqReady_ = 1;
}


void bme68x_delay_ms(uint32_t period)
{
  delay(period);
}


int64_t bme68x_getus(void)
{
  return (millis() * 1000);
}


DFRobot_BME68x::DFRobot_BME68x(bme68x_com_fptr_t readReg, bme68x_com_fptr_t writeReg, 
                               bme68x_delay_fptr_t delayMS, eBME68X_INTERFACE interface)
{
  bme68x_sensor.read = readReg;
  bme68x_sensor.write = writeReg;
  bme68x_sensor.delay_ms = delayMS;
  switch(interface) {
    case eBME68X_INTERFACE_I2C: bme68x_sensor.intf = BME68X_I2C_INTF; break;
    case eBME68X_INTERFACE_SPI: bme68x_sensor.intf = BME68X_SPI_INTF; break;
  }
}


int16_t DFRobot_BME68x::begin(void)
{
  bme68x_sensor.dev_id = this->bme68x_I2CAddr;
  if(bme68x_init(&bme68x_sensor) != BME68X_OK) {
    return -1;
  }
  uint8_t       set_required_settings;
  int8_t        rslt = 0;

	/* Set the temperature, pressure and humidity settings */
	//bme68x_sensor.tph_sett.os_hum = BME68X_OS_2X;
	//bme68x_sensor.tph_sett.os_pres = BME68X_OS_4X;
	//bme68x_sensor.tph_sett.os_temp = BME68X_OS_8X;
	//bme68x_sensor.tph_sett.filter = BME68X_FILTER_SIZE_3;

  
  bme68x_sensor.tph_sett.os_hum = 5;
  bme68x_sensor.tph_sett.os_pres = 5;
  bme68x_sensor.tph_sett.os_temp = 5;
  bme68x_sensor.tph_sett.filter = 4;

  /* Set the remaining gas sensor settings and link the heating profile */
  bme68x_sensor.gas_sett.run_gas = BME68X_ENABLE_GAS_MEAS;
  /* Create a ramp heat waveform in 3 steps */
  bme68x_sensor.gas_sett.heatr_temp = 320; /* degree Celsius */
  bme68x_sensor.gas_sett.heatr_dur = 150; /* milliseconds */

  /* Select the power mode */
  /* Must be set before writing the sensor configuration */
  bme68x_sensor.power_mode = BME68X_FORCED_MODE; 

  /* Set the required sensor settings needed */
  set_required_settings = BME68X_OST_SEL | BME68X_OSP_SEL | BME68X_OSH_SEL | BME68X_FILTER_SEL 
   | BME68X_GAS_SENSOR_SEL;

  /* Set the desired sensor configuration */
  rslt = bme68x_set_sensor_settings(set_required_settings, &bme68x_sensor);

	/* Set the power mode */
	rslt = bme68x_set_sensor_mode(&bme68x_sensor);

	/* Get the total measurement duration so as to sleep or wait till the
	 * measurement is complete */
	uint16_t meas_period;
	bme68x_get_profile_dur(&meas_period, &bme68x_sensor);
	bme68x_sensor.delay_ms(meas_period); /* Delay till the measurement is ready */
  return 0;
}


void DFRobot_BME68x::startConvert(void)
{
  bme68x_sensor.write(bme68x_sensor.dev_id, 0x74, &convertCmd, 1);
}


void DFRobot_BME68x::update(void)
{
  bme68x_get_sensor_data(&bme68x_data, &bme68x_sensor);
}


int8_t DFRobot_BME68x::iaqUpdate(void)
{
  uint8_t       gasBuf[2] = {0};
  uint16_t      gasADC = 0;
  uint8_t       gasRange = 0;
  if(bsec_iot_loop(bme68x_delay_ms, bme68x_getus, bme68x_outputReady) == 0) {

    if (bme68x_sensor.variant_id == BME68X_VARIANT_GAS_HIGHV){
      bme68x_get_regs(0x2c, gasBuf, 2, &bme68x_sensor);
    }else{
      bme68x_get_regs(0x2a, gasBuf, 2, &bme68x_sensor);
    }
    gasADC = (uint16_t) ((uint32_t) gasBuf[0] * 4 | (((uint32_t) gasBuf[1]) / 64));
    gasRange = gasBuf[1] & BME68X_GAS_RANGE_MSK;
    bme68x_data.gas_resistance = calc_gas_resistance(gasADC, gasRange, &bme68x_sensor);
    return 0;
  }
  return 1;
}


float DFRobot_BME68x::readTemperature(void)
{
  return bme68x_data.temperature;
}


float DFRobot_BME68x::readPressure(void)
{
  return bme68x_data.pressure;
}


float DFRobot_BME68x::readHumidity(void)
{
  return bme68x_data.humidity;
}


float DFRobot_BME68x::readAltitude(void)
{
  return (1.0 - pow((float)bme68x_data.pressure / 100.0f / BME68X_SEALEVEL, 0.190284)) * 287.15 / 0.0065;
}


float DFRobot_BME68x::readCalibratedAltitude(float seaLevel)
{
    //data->altitude = 44330.0 * (1.0 - pow((float) data->pressure / 100.0f / BME68X_SEALEVEL, 0.1903));
   return (1.0 - pow((float) bme68x_data.pressure / seaLevel, 0.190284)) * 287.15 / 0.0065;
}


float DFRobot_BME68x::readGasResistance(void)
{
  return bme68x_data.gas_resistance;
}


float DFRobot_BME68x::readSeaLevel(float altitude)
{
  return (bme68x_data.pressure / pow(1.0 - (altitude / 44330.0), 5.255));
}


float DFRobot_BME68x::readIAQ(void)
{
  return bme68x_data.gas_index;
}


void DFRobot_BME68x::setParam(eBME68X_param_t eParam, uint8_t dat)
{
  if(dat > 0x05) return;
  switch(eParam) {
    case eBME68X_PARAM_TEMPSAMP: writeParamHelper(0x74, dat, 0x07 << 5); break;
    case eBME68X_PARAM_PREESAMP: writeParamHelper(0x74, dat, 0x07 << 2); break;
    case eBME68X_PARAM_HUMISAMP: writeParamHelper(0x72, dat, 0x07); break;
    case eBME68X_PARAM_IIRSIZE: writeParamHelper(0x75, dat, 0x07 << 2); break;
  }
}

bool DFRobot_BME68x::setGasHeater(uint16_t heaterTemp, uint16_t heaterTime)
{
  if((heaterTemp == 0) || (heaterTime == 0)){
    bme68x_sensor.gas_sett.heatr_ctrl = BME68X_DISABLE_HEATER;
    bme68x_sensor.gas_sett.run_gas = BME68X_DISABLE_GAS_MEAS;
  }else{
    bme68x_sensor.gas_sett.heatr_ctrl = BME68X_ENABLE_HEATER;
    bme68x_sensor.gas_sett.run_gas = BME68X_ENABLE_GAS_MEAS;
    bme68x_sensor.gas_sett.heatr_temp = heaterTemp; /* degree Celsius */
    bme68x_sensor.gas_sett.heatr_dur = heaterTime; /* milliseconds */
  }

  uint8_t set_required_settings = BME68X_GAS_SENSOR_SEL;
  uint8_t rslt = bme68x_set_sensor_settings(set_required_settings, &bme68x_sensor);
  return rslt == 0;
}

uint8_t DFRobot_BME68x::isIAQReady(void)
{
  return iaqReady_;
}


void DFRobot_BME68x::writeParamHelper(uint8_t reg, uint8_t dat, uint8_t addr)
{
  uint8_t       var1 = 0;
  uint8_t       addrCount = 0;
  if(bme68x_sensor.intf == BME68X_SPI_INTF) bme68x_sensor.write(bme68x_sensor.dev_id, 0x73, 0x00, 1);
  bme68x_sensor.read(bme68x_sensor.dev_id, reg, &var1, 1);
  var1 &= ~addr;
  while(!(addr & 0x01)) {
    addrCount ++;
    addr >>= 1;
  }
  var1 |= dat << addrCount;
  bme68x_sensor.write(bme68x_sensor.dev_id, reg, &var1, 1);
}

