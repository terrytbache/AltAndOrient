#include <SD.h>
#include <gfxfont.h>
#include <Adafruit_GFX.h>
#include <Adafruit_SPITFT.h>
#include <Adafruit_SPITFT_Macros.h>
#include <Adafruit_SSD1351.h>
#include <RTClib.h>
#include <Wire.h>
#include <Adafruit_Sensor.h>
#include <Adafruit_BNO055.h>
#include <utility/imumaths.h>
#include <Adafruit_BMP280.h>

#define BMP_SCK 13
#define BMP_MISO 12
#define BMP_MOSI 11 
#define BMP_CS 10
#define sclk 2
#define mosi 3
#define dc   4
#define cs   5
#define rst  6

// Color definitions
#define  BLACK           0x0000
#define BLUE            0x001F
#define RED             0xF800
#define GREEN           0x07E0
#define CYAN            0x07FF
#define MAGENTA         0xF81F
#define YELLOW          0xFFE0  
#define WHITE           0xFFFF

Adafruit_BNO055 bno = Adafruit_BNO055();
Adafruit_BMP280 bmp; // I2C
Adafruit_SSD1351 tft = Adafruit_SSD1351(cs, dc, mosi, sclk, rst); 

RTC_DS3231 rtc;
char daysOfTheWeek[7][12] = {"Sunday", "Monday", "Tuesday", "Wednesday", "Thursday", "Friday", "Saturday"};

File myFile;

int readDelay;
int BNO055_SAMPLERATE_DELAY_MS = 1000;
int TICK = 32760;

/**************************************************************************SETUP************************************************************************************/

void setup(void)
{
  tft.begin();
  Serial.begin(9600);
  pinMode(10,OUTPUT); /*must be left as an output or the SD library functions will not work.

  /* Initialise the sensor */
  if(!bno.begin())
  {
    Serial.print("Ooops, no BNO055 detected ... Check your wiring or I2C ADDR!");
    while(1);
  }
  
if (!bmp.begin()) {  
    Serial.println(F("Could not find a valid BMP280 sensor, check wiring!"));
    while (1);
}
  bno.setExtCrystalUse(true);
  //Serial.println("Calibration status values: 0=uncalibrated, 3=fully calibrated");
  /*
  tft.fillScreen(BLACK);
  delay(5);
  lcdTestPattern();
  delay(50);
  tft.fillScreen(BLACK);   
*/
  if (!SD.begin(10)){
    Serial.println("initialization failed!");
    return;
  }

  
}


/**************************************************************************SETUP************************************************************************************/
void loop(void)
{


//RTC 
DateTime now = rtc.now();
    
    Serial.print(now.year(), DEC);
    Serial.print('/');
    Serial.print(now.month(), DEC);
    Serial.print('/');
    Serial.print(now.day(), DEC);
    Serial.print(" (");
    Serial.print(daysOfTheWeek[now.dayOfTheWeek()]);
    Serial.print(") ");
    Serial.print(now.hour(), DEC);
    Serial.print(':');
    Serial.print(now.minute(), DEC);
    Serial.print(':');
    Serial.print(now.second(), DEC);
    Serial.println();

    

/* char szTemp[8];
    dtostrf(now.second(), 1, 1, szTemp);
    testdrawtext(szTemp, WHITE);
    
/**************************************************************************/
/*
*/
/**************************************************************************/
 

  
  // Possible vector values can be:
  // - VECTOR_ACCELEROMETER - m/s^2
  // - VECTOR_MAGNETOMETER  - uT
  // - VECTOR_GYROSCOPE     - rad/s
  // - VECTOR_EULER         - degrees
  // - VECTOR_LINEARACCEL   - m/s^2
  // - VECTOR_GRAVITY       - m/s^2
  imu::Vector<3> euler = bno.getVector(Adafruit_BNO055::VECTOR_EULER);

  /* BNO55 - Display the floating point data */
  
 /*
  Serial.print("X: ");
  Serial.print(euler.x());
  Serial.print(" Y: ");
  Serial.print(euler.y());
  Serial.print(" Z: ");
  Serial.print(euler.z());
  Serial.print("\t\t");
  /* BNO55 - Quaternion data */
  imu::Quaternion quat = bno.getQuat();
 /*
  Serial.print("qW: ");
  Serial.print(quat.w(), 4);
  Serial.print(" qX: ");
  Serial.print(quat.y(), 4);
  Serial.print(" qY: ");
  Serial.print(quat.x(), 4);
  Serial.print(" qZ: ");
  Serial.print(quat.z(), 4);
  Serial.print("\t\t");
  /* BNO55 - Display calibration status for each sensor. */
  uint8_t system, gyro, accel, mag = 0;
  /*
  bno.getCalibration(&system, &gyro, &accel, &mag);
  Serial.print("CALIBRATION: Sys=");
  Serial.print(system, DEC);
  Serial.print(" Gyro=");
  Serial.print(gyro, DEC);
  Serial.print(" Accel=");
  Serial.print(accel, DEC);
  Serial.print(" Mag=");
  Serial.println(mag, DEC);
   /* BMP280 - Display status for each sensor. */
 
 Serial.print(F("Temperature = "));
  Serial.print(bmp.readTemperature());
  Serial.println(" *C");
  Serial.print(F("Pressure = "));
  Serial.print(bmp.readPressure());
  Serial.println(" Pa");
  Serial.print(F("Approx altitude = "));
  Serial.print(bmp.readAltitude(1013.25)); // this should be adjusted to your local forcase
  Serial.println(" m");
  Serial.println();

//For the SD card...

myFile = SD.open("testRTC.txt", FILE_WRITE);
if (myFile) {
    myFile.print(now.year(), DEC);
    myFile.print('/');
    myFile.print(now.month(), DEC);
    myFile.print('/');
    myFile.print(now.day(), DEC);
    myFile.print(" (");
    myFile.print(daysOfTheWeek[now.dayOfTheWeek()]);
    myFile.print(") ");
    myFile.print(now.hour(), DEC);
    myFile.print(':');
    myFile.print(now.minute(), DEC);
    myFile.print(':');
    myFile.print(now.second(), DEC);
    myFile.println();
     
    myFile.print(F("Temperature = "));
    myFile.print(bmp.readTemperature());
    myFile.println(" *C");
    myFile.print(F("Pressure = "));
    myFile.print(bmp.readPressure());
    myFile.println(" Pa");
    myFile.print(F("Approx altitude = "));
    myFile.print(bmp.readAltitude(1013.25)); // this should be adjusted to your local forcase
    myFile.println(" m");
    myFile.println();
    myFile.close();
} else {
  Serial.println("error in opening testRTC.txt");
}


 //********************************************************************************************** ***************************************   
 //*************************************** *************************************** *************************************** *************************************** 
  //              READ THE SERIAL MONITOR
 //********************************************************************************************** ***************************************   
 //*************************************** *************************************** *************************************** *************************************** 

 if (Serial.available() > 0)  {
  readDelay=Serial.parseInt();
  Serial.print ("Print delay (ms) Changed to ");
  Serial.print (readDelay);
  Serial.println ("...");
  BNO055_SAMPLERATE_DELAY_MS = readDelay;
  }
 
  delay(BNO055_SAMPLERATE_DELAY_MS);
  Serial.print("Print delay (ms) = ");
  Serial.println (BNO055_SAMPLERATE_DELAY_MS);
  TICK++;
  Serial.println (TICK);
 
   tft.fillScreen(BLACK);
    delay(50);
   
   char TICKDisp[0];
   Serial.println (TICKDisp);
    dtostrf(TICK, 1, 1, TICKDisp);
    testdrawtext(TICKDisp, WHITE);

  
}
   //********************************************************************************************** ***************************************   
 //*************************************** *************************************** *************************************** *************************************** 


//function definitions
void testdrawtext(char *text, uint16_t color) {
  tft.setCursor(0,0);
  tft.setTextColor(color);
  tft.print(text);
}


void lcdTestPattern(void)
{
  uint32_t i,j;
  tft.goTo(0, 0);
  
  for(i=0;i<128;i++)
  {
    for(j=0;j<128;j++)
    {
      if(i<16){
        tft.writeData(RED>>8); tft.writeData(RED);
      }
      else if(i<32) {
        tft.writeData(YELLOW>>8);tft.writeData(YELLOW);
      }
      else if(i<48){tft.writeData(GREEN>>8);tft.writeData(GREEN);}
      else if(i<64){tft.writeData(CYAN>>8);tft.writeData(CYAN);}
      else if(i<80){tft.writeData(BLUE>>8);tft.writeData(BLUE);}
      else if(i<96){tft.writeData(MAGENTA>>8);tft.writeData(MAGENTA);}
      else if(i<112){tft.writeData(BLACK>>8);tft.writeData(BLACK);}
      else {
        tft.writeData(WHITE>>8);      
        tft.writeData(WHITE);
       }
    }
  }
  
}

