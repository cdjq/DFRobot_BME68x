/*!
 * @file DFRobot_BME68x_SPI.ino
 * @brief Connect bme68x 4 wires SPI interface with your board (please reference board compatibility).
 * @n BME68x cs pin connect to D3 on esp8266 and esp32 board, on AVR board is IO 3.
 * @n Temprature, Humidity, pressure, altitude, calibrate altitude and gas resistance data will print on serial window.
 *
 * @copyright   Copyright (c) 2010 DFRobot Co.Ltd (http://www.dfrobot.com)
 * @license     The MIT License (MIT)
 * @author [Frank](jiehan.guo@dfrobot.com)
 * @maintainer [GDuang](yonglei.ren@dfrobot.com)
 * @version  V2.0
 * @date  2024-04-25
 * @url https://github.com/DFRobot/DFRobot_BME68x
 */

#include "DFRobot_BME68x_SPI.h"
#include "SPI.h"

#ifdef __AVR__
const uint8_t bme_cs = 3;
#elif ((defined ESP_PLATFORM) || (defined __ets__))
const uint8_t bme_cs = D3;
#else
  #error unknow board
#endif

/*use an accurate altitude to calibrate sea level air pressure*/
#define CALIBRATE_PRESSURE

DFRobot_BME68x_SPI bme(bme_cs);

float seaLevel; 
void setup()
{
  uint8_t rslt = 1;
  Serial.begin(9600);
  while(!Serial);
  delay(5000);
  Serial.println();
  while(rslt != 0) {
    rslt = bme.begin();
    if(rslt != 0) {
      Serial.println("bme begin failure");
      delay(2000);
    }
  }
  Serial.println("bme begin successful");
  #ifdef CALIBRATE_PRESSURE
  bme.startConvert();
  delay(1000);
  bme.update();
  /*You can use an accurate altitude to calibrate sea level air pressure. 
   *And then use this calibrated sea level pressure as a reference to obtain the calibrated altitude.
   *In this case,525.0m is chendu accurate altitude.
   */
  seaLevel = bme.readSeaLevel(525.0);
  Serial.print("seaLevel :");
  Serial.println(seaLevel);
  #endif

  // At initialization, the default heating layer target temperature is 320 and the duration is 150ms
  bool res = bme.setGasHeater(360, 100);
  
  Serial.print("Set the target temperature of the heating layer and the heating time: ");
  if(res == true){
    Serial.println("set successful!");
  }else{
    Serial.println("set failure!");
  }
}

void loop()
{
  bme.startConvert();
  delay(1000);
  bme.update();
  Serial.println();
  Serial.print("temperature(C) :");
  Serial.println(bme.readTemperature() / 100, 2);
  Serial.print("pressure(Pa) :");
  Serial.println(bme.readPressure());
  Serial.print("humidity(%rh) :");
  Serial.println(bme.readHumidity() / 1000, 2);
  Serial.print("gas resistance(ohm) :");
  Serial.println(bme.readGasResistance());
  Serial.print("altitude(m) :");
  Serial.println(bme.readAltitude());
  #ifdef CALIBRATE_PRESSURE
  Serial.print("calibrated altitude(m) :");
  Serial.println(bme.readCalibratedAltitude(seaLevel));
  #endif
}
