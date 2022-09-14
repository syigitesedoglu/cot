#include <Wire.h>
#include "RTClib.h" //important
#include <MPU6050.h> 
#include <time.h>
RTC_DS1307 rtc;
#define DS1307_ADDRESS 0x68 ///< I2C address for DS1307
#define MPU6050_ADDRESS 0x69//AD0 PIN (on the mpu6050 sensor)
MPU6050 mpu;
//define RTC.
double M,Y,D,MN,H,S;  //  14h 15m 39.7s, 19° 10′ 56″ Arcturus => 213.9154167    19.1822222  // 05h 55m 10.3053s, +07° 24′ 25.426″ Betelgeuse => 88.7929388    7.4070628
double A,B;
double location = 41.0251;//my longtitude.
double location2 = 29.0195;//my latitude.
double LST_degrees;//variable to store local side real time(LST) in degrees.
double LST_hours;//variable to store local side real time(LST) in decimal hours.
unsigned long timer = 0;
float timeStep = 0.01;
// Pitch and Yaw values
double pitch = 0; //Pitch ==> DEC value in degrees
double yaw = 0; // Yaw ==> RA value in degrees
double val = 0;//variable to store the user input DEC
double val2 = 0;//variable to store the user input RA
double temp = val2;//temporary value to store val2
const int stahp=7,stahp2=10;
const int cw=8,cw2=11;
const int ccw=6,ccw2=9;


void setup() {
      Serial.begin(115200);
    Serial.print("setup begin!");
  //set date-time according to (seconds, minutes, hours, day of the week, day of the month, month, year)
  if (! rtc.isrunning()) {
    Serial.println("RTC is NOT running!");
    Serial.println("Initialize MPU6050");
     //following line sets the RTC to the date & time this sketch was compiled
     //rtc.adjust(DateTime(F(_DATE_), F(_TIME_)));
     rtc.adjust(DateTime(__DATE__, __TIME__));
     //This line sets the RTC with an explicit date & time, for example to set
     //January 21, 2014 at 3am you would call:
     rtc.adjust(DateTime(2021, 12, 1, 11, 20, 0));
  }
    //rtc.adjust(DateTime(F(_DATE_), F(_TIME_)));
    rtc.adjust(DateTime(__DATE__, __TIME__));
    rtc.now();

    pinMode(stahp, OUTPUT); // base motor stop
    pinMode(cw, OUTPUT); //base motor clockwise
    pinMode(ccw,OUTPUT); // base motor counter clockwise
    pinMode(stahp2,OUTPUT); // DEC motor stop
    pinMode(cw2,OUTPUT); // DEC motor clockwise
    pinMode(ccw2,OUTPUT); // DEC motor counter clockwise
    delay(5000);//wait before starting 
    while(!mpu.begin(MPU6050_SCALE_2000DPS, MPU6050_RANGE_2G))
    {
    }
    mpu.calibrateGyro();
    mpu.setThreshold(3);
    Serial.print("setup done!");
}
void loop() 
{
    DateTime now = rtc.now();
    // DEC = declination
    Serial.print(now.year(), DEC);
    Serial.print('/');
    Serial.print(now.month(), DEC);
    Serial.print('/');
    Serial.print(now.day(), DEC);
    Serial.print(" (");
    Serial.print(now.dayOfTheWeek());
    Serial.print(") ");
    Serial.print(now.hour(), DEC);
    Serial.print(':');
    Serial.print(now.minute(), DEC);
    Serial.print(':');
    Serial.print(now.second(), DEC);
    Serial.println();
 //this will update the RA degrees with sidereal time 1degree at a time
    //this way the object or star on the sky is tracked.
  if(location2>0){//for the northern hemisphere. 
    if( floor(LST_degrees)==LST_degrees ){ 
      if (LST_degrees>180){
        val2 = temp+(360-LST_degrees);
        }else{
        val2 = temp-LST_degrees; //use val2 = temp+LST_degrees; if you are located in the southern hemisphere.
        }
    }
  }else{//for the southern hemisphere.
   if( floor(LST_degrees)==LST_degrees ){ 
      if (LST_degrees>180){
        val2 = temp-(360-LST_degrees);
        }else{
        val2 = temp+LST_degrees; //use val2 = temp+LST_degrees; if you are located in the southern hemisphere.
        }
    }
}
  rtc.now();
  LST_time();
  recvdata();
  pitch_check();
  yaw_check();
  timer = millis();
  Vector norm = mpu.readNormalizeGyro();
  //I've put the sensor with a 90 degree angle on the setup due to
    //cable connection problems. Because of that the data values from the mpu6050 chip are
    //different in this case:
    //roll data(X-axis) is pitch.
    //pitch data(Y-axis) is yaw.
    yaw = yaw + norm.YAxis * timeStep;
    pitch = pitch + norm.XAxis * timeStep;
    Serial.print(" Yaw = ");
    Serial.print(yaw);
    Serial.print(" Pitch = ");
    Serial.print(pitch);
    Serial.print(" LST_d = ");
    Serial.print(LST_degrees);
    Serial.print(" LST_h = ");
    Serial.println(LST_hours);//local sidereal time in decimal hours.
    delay((timeStep*1000) - (millis() - timer));//timer for the gyro.
    
}
void recvdata(){
  //This function receives data from serial as (0.00,0.00)
  //splits it to strings by the comma ","
  //than converts them to doubles 
  if(Serial.available() > 0){
   String a= Serial.readString();
   String value1, value2;
   //For loop which will seperate the String in parts
   //and assign them the variables we declare
   for (int i = 0; i < a.length(); i++ ){
       if (a.substring(i, i+1) == ",") {
           value2 = a.substring(0, i);
           value1= a.substring(i+1);
           break;
            }
        }
        val=90-value1.toFloat();
        val2=value2.toFloat();
        temp = val2;
    }
}
void pitch_check(){
    //check if pitch is high, low or equal to the user input
    //send commands to slave-module to start and stop motors
    if(floor(pitch*100)/100==floor(val*100)/100){
        digitalWrite(stahp,HIGH);
        }else{
        digitalWrite(stahp,LOW);
    }
    if(floor(pitch*100)<floor(val*100)){
        digitalWrite(cw,HIGH);
        }else{
        digitalWrite(cw,LOW);
    }
    if(floor(pitch*100)>floor(val*100)){
        digitalWrite(ccw,HIGH);
        }else{
        digitalWrite(ccw,LOW);
    }
}
void yaw_check(){
    //check if yaw is high, low or equal to the user input
    //send commands to slave-module to start and stop motors
    if(floor(yaw*100)==floor(val2*100)){
        digitalWrite(stahp2,HIGH);
        }else{
        digitalWrite(stahp2,LOW);
    }
    if(floor(yaw*100)<floor(val2*100)){
        digitalWrite(cw2,HIGH);
        }else{
        digitalWrite(cw2,LOW);
    }
    if(floor(yaw*100)>floor(val2*100)){
        digitalWrite(ccw2,HIGH);
        }else{
        digitalWrite(ccw2,LOW);
    }
}

void LST_time(){
  //http://www.stargazing.net/kepler/altaz.html

  DateTime now = rtc.now();

  M = (double) now.month();
  Y = (double) now.year();
  D = (double) now.dayOfTheWeek();
  MN = (double) now.minute();
  H = (double) now.hour();
  S = (double) now.second();
  A = (double) (Y-2000)*365.242199;
  B = (double) (M-1)*30.4368499;
  double JDN200=A+B+(D-1)+now.hour()/24;
  double decimal_time = H+(MN/60)+(S/3600);
  double LST = 100.46 + 0.985647 * JDN200 + location + 15*decimal_time;
  LST_degrees = (LST-(floor((LST/360)*360))) ;
  LST_hours = LST_degrees/15;
}
