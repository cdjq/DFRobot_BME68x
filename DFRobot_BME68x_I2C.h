/**
 * @file DFRobot_BME68x_I2C.h
 *
 * @copyright   Copyright (c) 2010 DFRobot Co.Ltd (http://www.dfrobot.com)
 * @license     The MIT License (MIT)
 * @author Frank(jiehan.guo@dfrobot.com)
 * @maintainer [GDuang](yonglei.ren@dfrobot.com)
 * @version  V2.0
 * @date  2024-04-25
 * @url https://github.com/DFRobot/DFRobot_BME68x
 */

#ifndef DFRobot_BME68x_I2C_H
#define DFRobot_BME68x_I2C_H

#include "DFRobot_BME68x.h"


class DFRobot_BME68x_I2C : public DFRobot_BME68x
{
  public:
    DFRobot_BME68x_I2C(uint8_t I2CAddr);

    void        setConvertAndUpdate(void);
};

#endif