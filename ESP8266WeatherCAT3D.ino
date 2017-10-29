/***************************************************************************
 ESP8266WeatherCAT3D
 Based on ESP8266BME280 with io.adafruit MQTT support
 ClearAirTurbulence3D ClearAirTurbulence3D @gmail.com
  
 Uses or used the following hardware for development:
  Adafruit ESP8266 HUZZAH breakbout board: https://www.adafruit.com/products/2471
  Adafuit SI1145 Digital UV Index / IR / Visible Light Sensor: https://www.adafruit.com/product/1777
  Adafruit BME280 breakout board for humidity, temperature & pressure. https://www.adafruit.com/product/2652
  Deployed prototype uses:
  Generic EBay BME280 breakout board for humidity, temperature & pressure.
  Generic EBay SI1145 Digital UV Index / IR / Visible Light Sensor
  DS18B20s ommited in this version
  
 I2C is used to communicate with these boards.
 20160206 - V2.0 based on DHT-22 version
                      20160207 - V2.1 added F and C temp feeds, BME280 sensor
                      20160221 - V2.2 added two DS18S20 sensors
                      20170901 - V3.0 new hardware for UV, visible and IR detection.
                      20171007 - V3.1 added MAX17043 LiPo fuel gauge monitor. DS18B20s removed.
  I2C addresses:
  0x36 : LiPo fuel gauge 
  0x60 : SI1145 
  0x76 :  BNE 280
                 
 This software uses the following code:
 LiuPo fuel gauge by lucadentella http://www.lucadentella.it/en/
 Anemometer code by sbiermann from: https://github.com/sbiermann/esp8266-anemometer
 The Adafruit BME280 library,designed specifically to work with the Adafruit BME280 Breakout
 ESP8266 Arduino from: https://github.com/esp8266/Arduino
 Adafruit.io code written by Tony DiCola for Adafruit Industries.

 This is a library for the Si1145 UV/IR/Visible Light Sensor

 Designed specifically to work with the Si1145 sensor in the
 adafruit shop
 ----> https://www.adafruit.com/products/1777
 Written by Limor Fried/Ladyada for Adafruit Industries. 

 Adafruit invests time and resources providing this open source code,
 please support Adafruit andopen-source hardware by purchasing products
 from Adafruit!

 Written by Limor Fried & Kevin Townsend for Adafruit Industries.
 BSD license, all text above must be included in any redistribution
***************************************************************************/
#include <Wire.h>
#include <Adafruit_Sensor.h>
#include <Adafruit_BME280.h>
#include "Adafruit_SI1145.h"
#include "MAX17043.h"

#include "config.h" // AdafuitIO & wireless info

//I2C pins for ESP8266 Can use other pins. The defaults are I2C SDA = GPIO #4 & I2C SCL = GPIO#5 
//uncomment below and define other pins for I2C. In the code, call Wire.begin(ESP8266_SDI, ESP8266_SCK);
//#define ESP8266_SDI  
//#define ESP8266_SCK

#define SEALEVELPRESSURE_HPA (1025.1)

Adafruit_BME280 bme; // I2C

MAX17043 batteryMonitor;
Adafruit_SI1145 uvl = Adafruit_SI1145();

// Program variables
float BME280_hum, BME280_temp, BME280_temp_f, BME280_hPa;  // Values read from BME280
float SI1145_vis, SI1145_ir, SI1145_uv;  // Values read from SI1145
float cellVoltage, stateOfCharge; //LiPo voltage, state of charge (%)
float WindSpeed, WindRev;

String webString="";     // String to display

int input_pin = 12; // GPIO12 for anemometer
unsigned long  next_timestamp = 0;
float last_wind = 0;
int count = 0;
volatile unsigned long last_micros;
long debouncing_time = 5; //in millis
volatile unsigned long IntCount = 0;
char charBuffer[32];

// Functions
void connect();

/****************************** Feeds ***************************************/
/* temperature : BME280 temperature in C                                    */
/* ----- temperaturef : BME280 temperature in F (calculated) not in free    */
/* pressure : BME280 pressure in hPA                                        */
/* humidity : BME280 humidity in %                                          */
/* ----- Dev1 : 1820 device 1 temperature (deleted)                         */
/* ----- Dev2 : 1820 device 2 temperature (deleted)                         */
/* LiPoV : LiPo battery voltage                                             */
/* LiPoC : LiPo battery charge %                                            */
/* VIS : Si1145 visible light value                                         */
/* IR : Si1145 IR light value                                               */
/* UV :  Si1145 UV light value                                              */
/* WindSpeed windspeed in km/hr                                             */
/* WindRev windspeed in revs per sec                                        */
/*                                                                          */
/* Setup format: const char CAPNAM_FEED and reference AIO /feed/feedname    */
/* Adafruit_MQTT_Publish feedname                                           */
/****************************************************************************/

// set up the feeds
AdafruitIO_Feed *temperature = io.feed("temperature");
AdafruitIO_Feed *humidity = io.feed("humidity");
AdafruitIO_Feed *pressure = io.feed("pressure");
AdafruitIO_Feed *lipov = io.feed("lipov");
AdafruitIO_Feed *lipoc = io.feed("lipoc");
AdafruitIO_Feed *vis = io.feed("vis");
AdafruitIO_Feed *ir = io.feed("ir");
AdafruitIO_Feed *uv = io.feed("uv");
AdafruitIO_Feed *windspeed = io.feed("windspeed");
AdafruitIO_Feed *windrev = io.feed("windrev");


void WindInterrupt()
{
  if((long)(micros() - last_micros) >= debouncing_time * 1000) 
  {
    IntCount++;
    last_micros = micros();
  }
} //WindInterrupt

void setup() 
{
  Serial.begin(115200); 
  batteryMonitor.reset();
  batteryMonitor.quickStart();
  Serial.println(F("ESP8266 Weather Station 3.0"));
  pinMode(12, INPUT_PULLUP); //enable internal pullup for reed switch
  if (!bme.begin(0x76)) //0x76 is the EBAY device address
    {
     Serial.println("Could not find a valid BME280 sensor, check wiring!");
     while (1);
    } //BME280 check
    
  if (! uvl.begin()) 
    {
     Serial.println("Didn't find a Si1145");
     while (1);
    }//Si1145 check

  // connect to io.adafruit.com
  Serial.print("Connecting to Adafruit IO");
  io.connect();
 // wait for a connection
  while(io.status() < AIO_CONNECTED)
   {
    Serial.print(".");
    delay(500);
   }
  // we are connected
  Serial.println();
  Serial.println(io.statusText());
       
  attachInterrupt(input_pin,WindInterrupt,RISING);
}//setup

void loop() 
{
  // io.run(); is required for all sketches.
  // it should always be present at the top of your loop
  // function. it keeps the client connected to
  // io.adafruit.com, and processes any incoming data.
 io.run();

 GetBME280Data(); 
 yield();
 GetSi1145Data();
 yield();
 GetWind();
 yield();
 GetLiPo();
 yield();
 
 // Publish data
 temperature->save(BME280_temp);
 pressure->save(BME280_hPa);
 humidity->save(BME280_hum);
 lipov->save(cellVoltage);
 lipoc->save(stateOfCharge);
 vis->save(SI1145_vis);
 ir->save(SI1145_ir);
 uv->save(SI1145_uv);
 windspeed->save(WindSpeed);
 windrev->save(WindRev);
 

// Commented out code is a test to increase the sleep delay if the battery falls below a certain voltage 
//   if(cellVoltage >= 3.5) // Repeat every 120 seconds if battery voltage is good

     ESP.deepSleep(120000000,WAKE_RF_DEFAULT); //deep sleep for 120 sec.
     
//   else
//     ESP.deepSleep(600000000,WAKE_RF_DEFAULT); // deep sleep for 10 minutes if battery voltage is low 

     
 delay(1000); //wait for deep sleep to complete
}//loop



// Get BME280 temperature, pressure & humidity as well as ESP8266 AD0 average value
void GetBME280Data() 
 {
  BME280_temp = bme.readTemperature(); // Read temperature as Celcius
//  BME280_temp_f = (BME280_temp * 1.8)+32.0; // Read temperature as Farenheit
  BME280_hPa = bme.readPressure()/100.0F; // Read pressure as hPa
  BME280_hum = bme.readHumidity();     // Read humidity (percent)

/*  Serial.print(BME280_temp);
  Serial.println(" C");
  Serial.print(BME280_temp_f);
  Serial.println(" F");
  Serial.print(BME280_hPa);
  Serial.println(" hPa");
  Serial.print(BME280_hum);
  Serial.println(" %"); 
  */  
 }// GetBME280Data

void GetSi1145Data()
{
 SI1145_vis = uvl.readVisible();
 SI1145_ir = uvl.readIR();

/* Serial.println("===================");
 Serial.print("Vis: "); 
 Serial.println(uv.readVisible());
 Serial.print("IR: "); 
 Serial.println(uv.readIR()); 
 */
   SI1145_uv = uvl.readUV();
  // the index is multiplied by 100 so to get the
  // integer index, divide by 100!
  SI1145_uv /= 100.0;
    
  //Serial.print("UV: ");  
  //Serial.println(UVindex); 
  
} //GetSi1145Data

void GetWind()
{
  if (millis() > next_timestamp )    
  { 
    detachInterrupt(input_pin);
    count++; 
    WindRev = IntCount/2; //computing rounds per second 
    if(IntCount == 0)
      WindSpeed = 0.0;
    else
      WindSpeed = 3.436 / (1 + WindRev) + 3.013 * WindRev;// found here: https://www.amazon.de/gp/customer-reviews/R3C68WVOLJ7ZTO/ref=cm_cr_getr_d_rvw_ttl?ie=UTF8&ASIN=B0018LBFG8 (in German)

    count = 0;
    IntCount = 0;
    last_wind = WindSpeed;
    //Serial.println(WindSpeed);
    //Serial.println(WindRev);
    next_timestamp  = millis()+1000; //intervall is 1s
    attachInterrupt(input_pin,WindInterrupt,RISING);
   }
  } //GetWind
  
void GetLiPo()
{ 
  cellVoltage = batteryMonitor.getVCell();
/*  Serial.print("Voltage:\t\t");
  Serial.print(cellVoltage, 4);
  Serial.println("V");
*/
  stateOfCharge = batteryMonitor.getSoC();
/*  Serial.print("State of charge:\t");
  Serial.print(stateOfCharge);
  Serial.println("%");  
*/
}//GetLiPo


