#include <Wire.h>
#include <RTClib.h>
#include <SoftwareSerial.h>
#include <Servo.h>

Servo horizontal;  // horizontal servo
int servoh = 180;
int servohLimitHigh = 175;
int servohLimitLow = 5;
// 65 degrees MAX

Servo vertical;  // vertical servo
int servov = 45;
int servovLimitHigh = 100;
int servovLimitLow = 1;

// LDR pin connections
// name = analogpin;
int ldrlt = A1;  //LDR top left - BOTTOM LEFT <--- BDG
int ldrrt = A2;  //LDR top right - BOTTOM RIGHT
int ldrld = A3;  //LDR down left - TOP LEFT
int ldrrd = A4;  //ldr down right - TOP RIGHT
int ldrmt = A5;

const int button1 = 9;
const int button2 = 10;
const int motorA = 7;
const int motorB = 8;
const int pvPin = 1; // +V from PV is connected to analog pin 1 
int buttonStateA = 0;
int buttonStateB = 0;
int angle = 0;
int angle_ref = 110; // angle of horizontal array; facing the sky
int pos = 0;
int pos2 = 0;
int oldvalue;
int oldvalue2;

// Define BT variables
long previousMillis = 0; // will store last time data was sent
long interval = 8000; // interval at which to send data (ms)

// Define your location
float LATITUDE = 15.3317;
float LONGITUDE = -120.9675;

RTC_DS3231 rtc;
SoftwareSerial BTSerial(2, 3);  // RX, TX
//Servo motorA;
//Servo motorB;          // motorA and motorB??

void setup() {
  angle = angle_ref;
  horizontal.attach(7);
  vertical.attach(8);
  horizontal.write(180);
  vertical.write(0);  //45? //0?
  
  pinMode(motorA, OUTPUT);
  pinMode(motorB, OUTPUT);
  pinMode(button1, INPUT);
  pinMode(button1, INPUT);
  pinMode(pvPin, INPUT);
  delay(2500);
  Serial.begin(9600);
  Wire.begin();
  rtc.begin();
  //if (!rtc.isrunning()) {
  //Serial.println("Couldn't find RTC");
  //while (1);

  //Set the RTC time
  //rtc.adjust(DateTime(F(12, 3, 2023), F(10, 00, 00)));  // day, month, year // hour, minute, second
  rtc.adjust(DateTime(2023, 3, 13, 15, 30, 0));  // set time to March 13, 2023, 3:30 PM
  BTSerial.begin(9600);
}

void loop() {
  if (BTSerial.available()) {
  // handle Bluetooth commands here
  //Calculate the runnning average
  total[3] -= readings[index][3];    
  readings[index][3] = analogRead(pvPin);             
  total[3] += readings[index][3];      
  float average_pv = total[3] / numReadings;     // return the runnning average of batt
  
  //Update PV voltage
  float voltage_pv = average_pv * ref_voltage * 2.0 / 1024.0; //    /(R2/(R1 + R2)) 
  float power_pv = voltage_pv * 0.150 * 5;
    
    // Correct angle for display
  float angle_corr = angle * (148 - 110) / 45 - 100 * (148 - 110) / 45;
      
    // Datalogging over bluetooth every X seconds
  unsigned long currentMillis = millis();
  unsigned long currentTime = millis();    // 1000.0;
  if(currentMillis - previousMillis > interval)
  {
   previousMillis = currentMillis;   
    Serial.print("Time = "); Serial.print(millis()/1000.0); Serial.print(" s       |  ");
    Serial.print("Array angle = "); Serial.print(angle_corr); Serial.println(" deg");
    //Serial.print("Temperature = "); Serial.print(temperatureC); Serial.println(" deg C");
    //Serial.print("Battery SOC = "); Serial.print( (int) battery_capa); Serial.print("%  |  ");
    //Serial.print("V batt = "); Serial.print(voltage_batt); Serial.println(" V");
    Serial.print("V pv = "); Serial.print(voltage_pv); Serial.print(" V        |  ");
    Serial.print("P pv (estimate) = "); Serial.print(power_pv); Serial.println(" W");
    Serial.print("Vcc = "); Serial.print(ref_voltage); Serial.println(" V");
    Serial.println();      
  }
  DateTime now = rtc.now();  // Get the current time from the RTC
  // calculate the position of the sun based on current time and location
  float altitude, azimuth;
  calculateSunPosition(now, LATITUDE, LONGITUDE, altitude, azimuth);
  Serial.print("Altitude: ");
  Serial.print(altitude);
  Serial.print(", Azimuth: ");
  Serial.println(azimuth);
  delay(1000);

  int year = now.year();
  int month = now.month();
  int day = now.day();
  int hour = now.hour();
  int minute = now.minute();
  int second = now.second();

  // Calculate the current day of the year
  int N = now.day();

  // Calculate the decimal hour
  float h = now.hour() + now.minute() / 60.0 + now.second() / 3600.0;

  // Calculate the solar declination angle
  float delta = 23.45 * sin(2 * PI * (N - 81) / 365);

  // Calculate the solar hour angle
  float H = 15 * (h - 12) + LONGITUDE;

  // Calculate the solar altitude angle
  float sinAlt = sin(radians(LATITUDE)) * sin(radians(delta)) + cos(radians(LATITUDE)) * cos(radians(delta)) * cos(radians(H));
  float alt = degrees(asin(sinAlt));

  // Calculate the solar azimuth angle
  float cosAz = (sin(radians(delta)) - sin(radians(LATITUDE)) * sin(radians(alt))) / (cos(radians(LATITUDE)) * cos(radians(alt)));
  float az = degrees(acos(cosAz));
  if (H > 0) 
    az = 360 - az;

    // control the servo motors to align the solar panel with the sun
    // implement feedback mechanism to continuously adjust the servo motors
  }

// Calculate the equation of time for a given date
float equationOfTime(int year, int month, int day) {
    float B = 360 / 365.24 * (day - 81);
    float EOT = B + 9.87 * sin(radians(2 * B)) - 7.53 * cos(radians(B)) - 1.5 * sin(radians(B));
    return EOT;
}
void calculateSunPosition(DateTime now, float latitude, float longitude, float &altitude, float &azimuth) {
    // Calculate the sun's position for the given date and time
    int year = now.year();
    int month = now.month();
    int day = now.day();
    int hour = now.hour();
    int minute = now.minute();
    int second = now.second();

    //Calculate the quationof time
    //float eqTime = equationOFTime(year, month, day);

    // Convert latitude and longitude to radians
    float latRad = radians(latitude);
    float longRad = radians(longitude);

    // Calculate the declination angle
    float sinDeclination = sin(radians(23.45)) * sin(radians(360 / 365.24 * (day - 81)));//sin(radians(altitude));
    float cosDeclination = sqrt(1 - pow(sinDeclination, 2));
    float declination = degrees(atan2(sinDeclination, cosDeclination));

    // Calculate the hour angle
    float solarTime = hour + minute / 60.0 + second / 3600.0;
    float solarNoon = 12.0 - (longitude / 15.0) - (4 * (longitude - 15 * equationOfTime(year, month, day)) / 60.0); 
    float hourAngle = 15.0 * (solarTime - solarNoon);

    // Calculate the altitude angle
    float sinAltitude = sin(latRad) * sin(radians(declination)) + cos(latRad) * cos(radians(declination)) * cos(radians(hourAngle));
    altitude = degrees(asin(sinAltitude));

    // Calculate the azimuth angle
    float cosAzimuth = (sin(radians(declination)) - sin(latRad) * sin(radians(altitude))) / (cos(latRad) * cos(radians(altitude)));
    azimuth = degrees(acos(cosAzimuth));
    if (hourAngle > 0) {
        azimuth = 360 - azimuth;
    }
}

void controlServoMotors(float altitude, float azimuth) {
    // convert altitude and azimuth angles to servo motor positions
    // control the servo motors to move to the desired positions

  int servohNew = map(azimuth, 0, 360, servohLimitLow, servohLimitHigh);
  int servovNew = map(altitude, 0, 90, servovLimitLow, servovLimitHigh);

    // move the servo motors to the desired positions
  if (abs(servohNew - servoh) > 2) {
    if (servohNew > servoh) {
      for (pos = servoh; pos <= servohNew; pos += 1) {
        horizontal.write(pos);
        delay(10);  //maybe adjusted (mm)
      }
    } else {
      for (pos = servoh; pos >= servohNew; pos -= 1) {
        horizontal.write(pos);
          delay(10);  //maybe adjusted (mm)
      }
    }
    servoh = servohNew;
  }

  if (abs(servovNew - servov) > 2) {
    if (servovNew > servov) {
      for (pos2 = servov; pos2 <= servovNew; pos2 += 1) {
        vertical.write(pos2);
        delay(10);  //maybe adjusted (mm)
      }
    } else {
      for (pos2 = servov; pos2 >= servovNew; pos2 -= 1) {
        vertical.write(pos2);
        delay(10);  //maybe adjusted (mm)
      }
    }
    servov = servovNew;
  }
}

void feedbackMechanism() {
  int ldrStatus = analogRead(ldrmt);
  if (ldrStatus > 30) {
    buttonStateA = digitalRead(button1);
    if (buttonStateA == LOW) {

      digitalWrite(motorA, HIGH);  //COUNTER clockwise
      digitalWrite(motorB, LOW);
    } else {
      digitalWrite(motorA, LOW);
      digitalWrite(motorB, LOW);
    }

    int lt = analogRead(ldrlt);  // top left
    int rt = analogRead(ldrrt);  // top right
    int ld = analogRead(ldrld);  // down left
    int rd = analogRead(ldrrd);  // down right
    int dtime = 10;
    int tol = 90;             // dtime=diffirence time, tol=toleransi
    int avt = (lt + rt) / 2;  // average value top
    int avd = (ld + rd) / 2;  // average value down
    int avl = (lt + ld) / 2;  // average value left
    int avr = (rt + rd) / 2;  // average value right
    int dvert = avt - avd;    // check the diffirence of up and down
    int dhoriz = avl - avr;   // check the diffirence og left and rigt

      //if(lt>90){
      //if(Switch_a==LOW){
      // digitalWrite(9==HIGH);
      // digitalWrite(10==LOW);
      // delay(1000);
      //}}

    if (-1 * tol > dvert || dvert > tol) {
      if (avt > avd) {
        servov = ++servov;
        if (servov > servovLimitHigh) {
          servov = servovLimitHigh;
        }
      } else if (avt < avd) {
        servov = --servov;
        if (servov < servovLimitLow) {
          servov = servovLimitLow;
        }
      }
      vertical.write(servov);
    }
    if (-1 * tol > dhoriz || dhoriz > tol)  // check if the diffirence is in the tolerance else change horizontal angle
    {
      if (avl > avr) {
        servoh = --servoh;
        if (servoh < servohLimitLow) {
          servoh = servohLimitLow;
        }
      } else if (avl < avr) {
        servoh = ++servoh;
        if (servoh > servohLimitHigh) {
          servoh = servohLimitHigh;
        }
      } else if (avl = avr) {
        delay(10);
      }
      horizontal.write(servoh);
    }

    delay(dtime);

  } else {
    oldvalue = horizontal.read();
    oldvalue2 = vertical.read();

    for (pos = oldvalue; pos <= 180; pos += 1) {  // goes from 0 degrees to 180 degrees
        // in steps of 1 degree
      horizontal.write(pos);
      delay(15);
    }
    for (pos2 = oldvalue2; pos2 <= 0; pos2 += 1) {  // goes from 0 degrees to 180 degrees
      // in steps of 1 degree

      vertical.write(pos2);  // tell servo to go to position in variable 'pos'
      delay(15);
    }
    buttonStateB = digitalRead(button2);
    if (buttonStateB == LOW) {

      digitalWrite(motorA, LOW);  //clockwise
      digitalWrite(motorB, HIGH);
    } else {
      digitalWrite(motorA, LOW);
      digitalWrite(motorB, LOW);
    }
  }
}
