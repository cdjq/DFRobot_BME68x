/**
 * @file DFRobot_BME68x_I2C.cpp
 *
 * @copyright   Copyright (c) 2010 DFRobot Co.Ltd (http://www.dfrobot.com)
 * @license     The MIT License (MIT)
 * @author Frank(jiehan.guo@dfrobot.com)
 * @maintainer [GDuang](yonglei.ren@dfrobot.com)
 * @version  V2.0
 * @date  2024-04-25
 * @url https://github.com/DFRobot/DFRobot_BME68x
 */

#include "DFRobot_BME68x_I2C.h"


static int8_t bme68x_i2c_read(uint8_t dev_id, uint8_t reg_addr, uint8_t *data, uint16_t len)
{
  Wire.begin();
  Wire.beginTransmission(dev_id);
  Wire.write(reg_addr);
  Wire.endTransmission();
  Wire.requestFrom(dev_id, (uint8_t)len);
  while(Wire.available()) {
    *data = Wire.read();
    data ++;
  }
  return 0;
}


static int8_t bme68x_i2c_write(uint8_t dev_id, uint8_t reg_addr, uint8_t *data, uint16_t len)
{
  Wire.begin();
  Wire.beginTransmission(dev_id);
  Wire.write(reg_addr);
  while(len --) {
    Wire.write(*data);
    data ++;
  }
  Wire.endTransmission();
  return 0;
}


DFRobot_BME68x_I2C::DFRobot_BME68x_I2C(uint8_t I2CAddr_) :
                    DFRobot_BME68x(bme68x_i2c_read, bme68x_i2c_write, bme68x_delay_ms, eBME68X_INTERFACE_I2C)
{
  bme68x_I2CAddr = I2CAddr_;
}


void DFRobot_BME68x_I2C::setConvertAndUpdate()
{
  bsec_iot_init(0.33333f, 0, bme68x_i2c_write, bme68x_i2c_read, bme68x_delay_ms, bme68x_I2CAddr, BME68X_I2C_INTF);
}

