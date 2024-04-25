/**
 * @file DFRobot_BME68x_SPI.cpp
 *
 * @copyright   Copyright (c) 2010 DFRobot Co.Ltd (http://www.dfrobot.com)
 * @license     The MIT License (MIT)
 * @author Frank(jiehan.guo@dfrobot.com)
 * @maintainer [GDuang](yonglei.ren@dfrobot.com)
 * @version  V2.0
 * @date  2024-04-25
 * @url https://github.com/DFRobot/DFRobot_BME68x
 */
#include "DFRobot_BME68x_SPI.h"

static uint8_t bme68x_cs = 0;


static int8_t bme68x_spi_read(uint8_t dev_id, uint8_t reg_addr, uint8_t *data, uint16_t len)
{
  SPI.begin();
  SPI.beginTransaction(SPISettings(1000000, MSBFIRST, SPI_MODE0));
  digitalWrite(bme68x_cs, 0);
  SPI.transfer(reg_addr | 0x80);
  while(len --) {
    *data = SPI.transfer(0x00);
    data ++;
  }
  digitalWrite(bme68x_cs, 1);
  SPI.endTransaction();
  return 0;
}


static int8_t bme68x_spi_write(uint8_t dev_id, uint8_t reg_addr, uint8_t *data, uint16_t len)
{
  SPI.begin();
  SPI.beginTransaction(SPISettings(1000000, MSBFIRST, SPI_MODE0));
  digitalWrite(bme68x_cs, 0);
  SPI.transfer(reg_addr);
  while(len --) {
    SPI.transfer(*data);
    data ++;
  }
  digitalWrite(bme68x_cs, 1);
  SPI.endTransaction();
  return 0;
}


DFRobot_BME68x_SPI::DFRobot_BME68x_SPI(uint8_t pin_cs) :
                    DFRobot_BME68x(bme68x_spi_read, bme68x_spi_write, bme68x_delay_ms, eBME68X_INTERFACE_SPI)
{
  bme68x_cs = pin_cs;
  pinMode(bme68x_cs, OUTPUT);
  digitalWrite(bme68x_cs, 1);
}


void DFRobot_BME68x_SPI::setConvertAndUpdate()
{
  bsec_iot_init(0.33333f, 0, bme68x_spi_write, bme68x_spi_read, bme68x_delay_ms, 0x77, BME68X_SPI_INTF);
}

