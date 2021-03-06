# ESP8266WeatherCAT3D
ESP8266 weather station with a BME 280, SI1145 and Acurite 3in1 anemometer.

**Uses (or used) the following hardware for development:**
* Adafruit ESP8266 HUZZAH breakbout board: https://www.adafruit.com/products/2471
* Adafuit SI1145 Digital UV Index / IR / Visible Light Sensor: https://www.adafruit.com/product/1777
* Adafruit BME280 breakout board for humidity, temperature & pressure: https://www.adafruit.com/product/2652
  
**Deployed prototype uses:**
* Generic EBay BME280 breakout board for humidity, temperature & pressure.
* Generic EBay SI1145 Digital UV Index / IR / Visible Light Sensor
* DS18B20s ommited in this version
  
 **Version history:**
* 20160206 - V2.0 based on DHT-22 version
* 20160207 - V2.1 added F and C temp feeds, BME280 sensor
* 20160221 - V2.2 added two DS18S20 sensors
* 20170901 - V3.0 new hardware for UV, visible and IR detection.
* 20171007 - V3.1 added MAX17043 LiPo fuel gauge monitor. DS18B20s removed.
* 20171203 - V3.2 Added Wire.begin in setup. Code works without, but best to use it.

 **I2C addresses:**
* 0x36 : LiPo fuel gauge 
* 0x60 : SI1145 
* 0x76 :  BME 280
                 
 **This software uses the following code:**
 LiPo fuel gauge by lucadentella http://www.lucadentella.it/en/
 Anemometer code by sbiermann from: https://github.com/sbiermann/esp8266-anemometer
 The Adafruit BME280 library,designed specifically to work with the Adafruit BME280 Breakout
 ESP8266 Arduino from: https://github.com/esp8266/Arduino
 Adafruit.io code written by Tony DiCola for Adafruit Industries.
 
 This is a library for the Si1145 UV/IR/Visible Light Sensor
 Designed specifically to work with the Si1145 sensor in the
 adafruit shop ----> https://www.adafruit.com/products/1777
 Written by Limor Fried/Ladyada for Adafruit Industries. 


* reddit post: https://www.reddit.com/r/esp8266/comments/79dmxj/esp8266_weather_station_in_an_acurite_3_in_1/
* imgur album: https://imgur.com/a/ICvYv

**2017** [ClearAirTurbulence3D](ClearAirTurbulence3D@gmail.com)
