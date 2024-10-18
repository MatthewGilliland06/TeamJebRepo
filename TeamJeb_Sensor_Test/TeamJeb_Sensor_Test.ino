#include <Wire.h>
#include <string>
#include <SPI.h>
#include <UnixTime.h>
#include <Adafruit_Sensor.h>
#include "Adafruit_BMP3XX.h"
#include <Adafruit_BNO055.h>
#include <utility/imumaths.h>
#include <SoftwareSerial.h>
#include "SparkFun_Qwiic_OpenLog_Arduino_Library.h"
#include <SparkFun_u-blox_GNSS_v3.h>
SFE_UBLOX_GNSS myGNSS;
sensors_event_t event; 

UnixTime stamp(0); // 0 for UTC Time

#define SEALEVELPRESSURE_HPA (1013.25)

Adafruit_BMP3XX bmp; //Pressure Sensor
Adafruit_BNO055 bno = Adafruit_BNO055(55); // Absolute Orientation Sensor
SoftwareSerial OpenLog(0, 1); //SD Logger


// REQUIRED TELEMETRY
String team_ID = "Team Jeb";
unsigned int mission_Time = 0;
uint32_t UTC_Time = 0;
float packet_Count = 0, altitude = 0, temp = 0;
float SW_State = 0; //SW_State  1 = Ascent, 2 = Stabilization, 3 = Descent, 4 = Landing, 5 = Landed
/*
enum SW_State{
  Ascent,
  Stabilization,
  Descent,
  Landing, 
  Landed
};
*/

float acc_X = 0, acc_Y = 0, acc_Z = 0; // In meters per second squared
float gyro_X = 0, gyro_Y = 0, gyro_Z = 0; // Degrees per second
float gps_Lat = 0, gps_Long = 0, gps_Alt = 0; // Latitude and Longitude in Degrees and Alt in Meters above Sea Level

//ADDITIONAL TELEMETRY
int orient_X = 0, orient_Y = 0, orient_Z = 0;
int angle_X = 0, angle_Y = 0, angle_Z = 0;
float maxAlt = 0;
int lowAlt = 0;
float pressure = 0;
bool descent = false;
int Pi = 3.1415926535897932384626433832795;
int led_Time = 0;
int solenoid_Time = 0;
int GPS_Time = 0;
int timePeriod = 500; // Milliseconds
bool ledOn = false;
imu::Vector<3> orientation;
imu::Vector<3> acceleration;
imu::Vector<3> gyro;

void logData();
void recieveData();
void stabilize();

void setup() {
  // put your setup code here, to run once:

  Serial.begin(19200);
  OpenLog.begin(19200);

  if (!bmp.begin_I2C()) {
    Serial.println("Could not find BMP388 sensor, check wiring!");
    while (1);
  }
  if(!bno.begin())
  {
    Serial.print("Ooops, no BNO055 detected ... Check your wiring or I2C ADDR!");
    while(1);
  }
  while (myGNSS.begin() == false) //Connect to the u-blox module using Wire port
  {
    Serial.println(F("u-blox GNSS not detected at default I2C address. Retrying..."));
    delay (1000);
  }

  myGNSS.setI2COutput(COM_TYPE_UBX);
  myGNSS.setNavigationFrequency(5);
    
  bmp.setTemperatureOversampling(BMP3_OVERSAMPLING_8X);
  bmp.setPressureOversampling(BMP3_OVERSAMPLING_4X);
  bmp.setIIRFilterCoeff(BMP3_IIR_FILTER_COEFF_3);
  bmp.setOutputDataRate(BMP3_ODR_50_HZ);
  
  bno.setExtCrystalUse(true);

  


  //Code for CSV File Writing
  OpenLog.print("TEAM_ID");
  OpenLog.print(",");
  OpenLog.print("MISSION_TIME");
  OpenLog.print(",");
  OpenLog.print("UTC_TIME");
  OpenLog.print(",");
  OpenLog.print("PACKET_COUNT");
  OpenLog.print(",");
  OpenLog.print("SW_STATE");
  OpenLog.print(",");
  OpenLog.print("ALTITUDE");
  OpenLog.print(",");
  OpenLog.print("TEMP");
  OpenLog.print(",");
  OpenLog.print("ACC_X");
  OpenLog.print(",");
  OpenLog.print("ACC_Y");
  OpenLog.print(",");
  OpenLog.print("ACC_Z");
  OpenLog.print(",");
  OpenLog.print("GYRO_X");
  OpenLog.print(",");
  OpenLog.print("GYRO_Y");
  OpenLog.print(",");
  OpenLog.print("GRYO_Z");
  OpenLog.print(",");
  OpenLog.print("GPS_LAT");
  OpenLog.print(",");
  OpenLog.print("GPS_LONG");
  OpenLog.print(",");
  OpenLog.print("GPS_ALT");
  OpenLog.print(",");
  OpenLog.println("PRESSURE");  

  stamp.setDateTime(2024, 10, 14, 20, 32, 30);   
  UTC_Time = stamp.getUnix();

}

void loop() {
  delay(100);
  mission_Time = millis();
  recieveData();
  digitalWrite(4,LOW);
  

  //Getting input from sensors will be above this ^^^
    
  //Software State Decider
  //if (altitude > 5000 && altitude < 16500 && SW_State<1)
  //{
    //SW_State = 1;
  //}

  //Testing Purposes
  
  Serial.println("");
  Serial.print("Temperature:");
  Serial.print(temp);
  Serial.print(" Pressure:");
  Serial.print(pressure);
  Serial.print(" Altitude:");
  Serial.print(altitude);
  Serial.print(" Max Altitude:");
  Serial.println(maxAlt);
  Serial.print(orient_X);
  Serial.print(",");
  Serial.print(orient_Y);
  Serial.print(",");
  Serial.println(orient_Z);
  Serial.print(acc_X);
  Serial.print(",");
  Serial.print(acc_Y);
  Serial.print(",");
  Serial.println(acc_Z);
  Serial.print(gyro_X);
  Serial.print(",");
  Serial.print(gyro_Y);
  Serial.print(",");
  Serial.println(gyro_Z);
  Serial.println(mission_Time);
  digitalWrite(11,HIGH);

  Serial.print(F("Lat: "));
  Serial.print(gps_Lat);
  Serial.print(F(" Long: "));
  Serial.print(gps_Long);
  Serial.print(F(" Alt: "));
  Serial.print(gps_Alt);
  Serial.println(F(" (m)"));
  
  //Serial.println(UTC_Time);

  if (mission_Time - solenoid_Time >= timePeriod)
  {
  //logData(); // Logs data
  stabilize(); // Testing stabilization
  solenoid_Time = mission_Time;
  }

  if (mission_Time - led_Time >= timePeriod)
  {
    if (ledOn==false)
    {
      digitalWrite(4,HIGH);
      led_Time = mission_Time;
      ledOn = true;
    }
    else if (ledOn == true)
    {
      digitalWrite(4,LOW);
      led_Time = mission_Time;
      ledOn = false;
    }
  }
  
  

}

void logData() // Logs data to sd card
{
  OpenLog.print(team_ID);
  OpenLog.print(",");
  OpenLog.print(mission_Time);
  OpenLog.print(",");
  OpenLog.print(UTC_Time);
  OpenLog.print(",");
  OpenLog.print(packet_Count);
  OpenLog.print(",");
  OpenLog.print(SW_State);
  OpenLog.print(",");
  OpenLog.print(altitude);
  OpenLog.print(",");
  OpenLog.print(temp);
  OpenLog.print(",");
  OpenLog.print(acc_X);
  OpenLog.print(",");
  OpenLog.print(acc_Y);
  OpenLog.print(",");
  OpenLog.print(acc_Z);
  OpenLog.print(",");
  OpenLog.print(gyro_X);
  OpenLog.print(",");
  OpenLog.print(gyro_Y);
  OpenLog.print(",");
  OpenLog.print(gyro_Z);
  OpenLog.print(",");
  OpenLog.print(gps_Lat);
  OpenLog.print(",");
  OpenLog.print(gps_Long);
  OpenLog.print(",");
  OpenLog.print(gps_Alt);  
  OpenLog.print(",");
  OpenLog.println(pressure);  
  return;
}

void recieveData()
{
  
  
  UTC_Time++;
  temp = bmp.temperature;
  pressure = bmp.pressure/100;
  altitude = bmp.readAltitude(SEALEVELPRESSURE_HPA);
  if ( mission_Time - GPS_Time >= 100)
  {
  gps_Lat = myGNSS.getLatitude();
  gps_Long = myGNSS.getLongitude();
  gps_Alt = myGNSS.getAltitudeMSL() / 1000; // Altitude above Mean Sea Level
  GPS_Time = mission_Time;
  }
  bno.getEvent(&event);
  orientation = bno.getVector(Adafruit_BNO055::VECTOR_EULER);
  acceleration = bno.getVector(Adafruit_BNO055::VECTOR_ACCELEROMETER);
  gyro = bno.getVector(Adafruit_BNO055::VECTOR_GYROSCOPE);
  acc_X = acceleration.x();
  acc_Y = acceleration.y();
  acc_Z = acceleration.z();
  gyro_X = (gyro.x())*(180/Pi);
  gyro_Y = (gyro.y())*(180/Pi);
  gyro_Z = (gyro.z())*(180/Pi);
  orient_X = orientation.x();
  orient_Y = orientation.y();
  orient_Z = orientation.z();
  packet_Count++;
}


 

//   Stabilization Algorithm in progress   //
void stabilize() {
  if (gyro_X > 15 || gyro_X < -15)
  {
    if (gyro_X > 10)
    {
      digitalWrite(2,HIGH);
      //turn on counter clockwise solenoid
      //wait for a set amount of time (Don't use delay)
      //turn off counter clockwise solenoid
      digitalWrite(3,LOW);
      
    }
    if (gyro_X < -10)
    {
      digitalWrite(3,HIGH);
      //turn on clockwise solenoid
      //wait for a set amount of time (Don't use delay)
      //turn off clockwise solenoid
      digitalWrite(2,LOW);
    }
  
  }
  else
  {
    digitalWrite(2,LOW);
    digitalWrite(3,LOW);
  }
}


/*
void softwareState(){
  if (gps_Alt>15000)
  {
    SW_State =
  }

}
*/

