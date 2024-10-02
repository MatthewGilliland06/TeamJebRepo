#include <Wire.h>
#include <string>

// REQUIRED TELEMETRY
String team_ID = "Team Jeb";
int mission_Time = 0, UTC_Time = 0, packet_Count = 0, altitude = 0, temp = 0;
int SW_State = 0; //SW_State  1 = Ascent, 2 = Stabilization, 3 = Descent, 4 = Landing, 5 = Landed
int acc_X = 0, acc_Y = 0, acc_Z = 0; // In meters per second squared
int gyro_X = 0, gyro_Y = 0, gyro_Z = 0; // Degrees per second
int gps_Lat = 0, gps_Lon = 0, gps_Alt = 0; // Latitude and Longitude in Degrees and Alt in Meters above Sea Level

//ADDITIONAL TELEMETRY
int maxAlt = 0;
int lowAlt = 0;
bool descent = false;


void setup() {
  // put your setup code here, to run once:
  pinMode(4, OUTPUT);

  Serial.begin(9600);

}

void loop() {


  //Getting input from sensors will be above this ^^^
  
  
  if (altitude>maxAlt)
   {
     maxAlt=altitude;
   }

  


  //Software State Decider
  if (altitude > 5000 && altitude < 16500 && SW_State<1)
  {
    SW_State = 1;
  }


  //Testing Purposes
  digitalWrite(4, HIGH);
  Serial.println("LED On");
  delay(1000);
  digitalWrite(4, LOW);
  Serial.println("LED Off");
  delay(1000);


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

