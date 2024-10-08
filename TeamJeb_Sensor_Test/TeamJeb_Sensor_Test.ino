#include <Wire.h>
#include <string>
#include <SPI.h>
#include <Adafruit_Sensor.h>
#include "Adafruit_BMP3XX.h"
#include <Adafruit_BNO055.h>
#include <utility/imumaths.h>
#include <SoftwareSerial.h>


#define BMP_SCK 13
#define BMP_MISO 12
#define BMP_MOSI 11
#define BMP_CS 10

#define SEALEVELPRESSURE_HPA (1013.25)

Adafruit_BMP3XX bmp; //Pressure Sensor
Adafruit_BNO055 bno = Adafruit_BNO055(55); // Absolute Orientation Sensor
SoftwareSerial OpenLog(0, 1); //SD Logger


// REQUIRED TELEMETRY
String team_ID = "Team Jeb";
float mission_Time = 0, UTC_Time = 0, packet_Count = 0, altitude = 0, temp = 0;
float SW_State = 0; //SW_State  1 = Ascent, 2 = Stabilization, 3 = Descent, 4 = Landing, 5 = Landed
float acc_X = 0, acc_Y = 0, acc_Z = 0; // In meters per second squared
float gyro_X = 0, gyro_Y = 0, gyro_Z = 0; // Degrees per second
float gps_Lat = 0, gps_Lon = 0, gps_Alt = 0; // Latitude and Longitude in Degrees and Alt in Meters above Sea Level

//ADDITIONAL TELEMETRY
float maxAlt = 0;
int lowAlt = 0;
float pressure = 0;
bool descent = false;

void logData();

void setup() {
  // put your setup code here, to run once:

  Serial.begin(9600);
  OpenLog.begin(9600);
  new 

  if (!bmp.begin_I2C()) {
    Serial.println("Could not find a valid BMP3 sensor, check wiring!");
    while (1);
  }
  if(!bno.begin())
  {
    Serial.print("Ooops, no BNO055 detected ... Check your wiring or I2C ADDR!");
    while(1);
  }
    

  

  bmp.setTemperatureOversampling(BMP3_OVERSAMPLING_8X);
  bmp.setPressureOversampling(BMP3_OVERSAMPLING_4X);
  bmp.setIIRFilterCoeff(BMP3_IIR_FILTER_COEFF_3);
  bmp.setOutputDataRate(BMP3_ODR_50_HZ);
  
  bno.setExtCrystalUse(true);


  //Code for CSV File Writing
  OpenLog.print("Temp");
  OpenLog.print(",");
  OpenLog.print("Pressure");
  OpenLog.print(",");
  OpenLog.println("Altitude");


}

void loop() {

  temp = bmp.temperature;
  pressure = bmp.pressure/100;
  altitude = bmp.readAltitude(SEALEVELPRESSURE_HPA);
  


  //Getting input from sensors will be above this ^^^
  
  delay(1000);
  if (altitude>maxAlt && altitude!=1654.
  )
   {
    maxAlt=altitude;
   }

  


  //Software State Decider
  //if (altitude > 5000 && altitude < 16500 && SW_State<1)
  //{
    //SW_State = 1;
  //}


  //Testing Purposes
  //digitalWrite(4, HIGH);
  //delay(1500);
  //digitalWrite(4, LOW);
  //delay(1500);



  sensors_event_t event; 
  bno.getEvent(&event);
  Serial.print("X: ");
  Serial.print(event.orientation.x, 4);
  Serial.print("\tY: ");
  Serial.print(event.orientation.y, 4);
  Serial.print("\tZ: ");
  Serial.print(event.orientation.z, 4);
  Serial.println("");
  Serial.print("Temperature:");
  Serial.print(temp);
  Serial.print(" Pressure:");
  Serial.print(pressure);
  Serial.print(" Altitude:");
  Serial.print(altitude);
  Serial.print(" Max Altitude:");
  Serial.println(maxAlt);

  logData();




}

void logData() // Logs data to sd card
{
  OpenLog.print(temp);
  OpenLog.print(",");
  OpenLog.print(pressure);
  OpenLog.print(",");
  OpenLog.println(altitude);
  return;
}
 

//   Stabilization Algorithm in progress   //
//void stabilize() {
  //if (gyro_X > 10 || gyro_X < -10)
  //{
    //if (gyro_X > 5)
    //{
      //turn on counter clockwise solenoid
      //delay
      //turn off counter clockwise solenoid
    //}
    //if (gyro_X < -5)
    //{
      //turn on clockwise solenoid
      //delay
      //turn off clockwise solenoid
    //}
    
  //}
//}

