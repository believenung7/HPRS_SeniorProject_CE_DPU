#include <SoftwareSerial.h>
#include <Wire.h>
#include "Kalman.h"
#include <ArduinoSort.h>

SoftwareSerial bluetooth(12, 13); // RX, TX

#define Motor_Left_Forward      52
#define Motor_Left_Backward     51
#define Motor_Right_Forward     53
#define Motor_Right_Backward    50
#define Motor_Body_Forward      48
#define Motor_Body_Backward     49
#define PWM_Left                2
#define PWM_Right               3
#define PWM_Body                4

#define LimitForward            22
#define LimitBackward           23

#define sw_control              19

int ModeGyro = 0; //0 = Manual, 1=Auto
int ModePicture = 0;
int NewCountForGyro;
int LastCountForGyro;
int CurrentCountForGyro;


int ValueLimitForward = 0;
int ValueLimitBackward = 0;
String contentFromAndroid = "";
String contentFromJoy = "";
String CheckAndroid = "";
String CheckJoy = "";
String WheelLeft = "00";
String speedmode = "0";
int modecontrol;
String contentjoy = "";
String cmd = "";
String string = "";
int limitForward;
int limitBackward;
int checkbody;
int checkbodyForward;
int checkbodyBackward;
int SWcontrol = 0;
int Speed_Left = 245;
int Speed_Right = 245;
int Speed_Body = 105;
int speedmodechange;
int spchange;

String RT = "RT" ;
String GY = "GY" ;
String WH = "WH" ;
String SW = "SW";
String batt = "24.7";

unsigned long previousMillis = 0;

int forwardStep;
int backwardStep;

float voltage;
float sensorVC = 0;

int sensorPin = A1;
float sensorValue = 0.0f;
String voltageString = "0.0";
int stringLength = 0;

float vout = 0.0;
float vin = 0.0;
float R1 = 30000.0;
float R2 = 7500.0;

//For Gyro GY521
Kalman kalmanX; // Create the Kalman instance
/* IMU Data */
int16_t accX, accY, accZ;
int16_t tempRaw;
int16_t gyroX, gyroY, gyroZ;

float accXangle;//, accYangle; // Angle calculate using the accelerometer
float gyroXangle;//, gyroYangle; // Angle calculate using the gyro
float kalAngleX;//, kalAngleY; // Calculate the angle using a Kalman filter

float accYangle;//, accYangle; // Angle calculate using the accelerometer
float gyroYangle;//, gyroYangle; // Angle calculate using the gyro
float kalAngleY;//, kalAngleY; // Calculate the angle using a Kalman filter

unsigned long timer;
uint8_t i2cData[14]; // Buffer for I2C data
float CurrentAngle;
float LastCurrentAngle4;
float LastCurrentAngle3;
float LastCurrentAngle2;
float LastCurrentAngle;
float NewCurrentAngle;

int speed;
int speedtemp = 0;
int move = 6;

// PID
const float Kp = 4; //6
const float Ki = 1; //1
const float Kd = 1; //1
float pTerm, iTerm, dTerm, integrated_error, last_error, error;
const float K = 1.9 * 1.12;
//const float K = 2;

//#define   GUARD_GAIN   10.0 //40 //20

#define runEvery(t) for (static typeof(t) _lasttime;(typeof(t))((typeof(t))millis() - _lasttime) > (t);_lasttime += (t))

void OpenTimer()
{
  noInterrupts();           // disable all interrupts
  TCCR1A = 0;
  TCCR1B = 0;
  TCNT1  = 0;

  OCR1A = 7812;     //3000  15624     // compare match register 16MHz/256/2Hz
  TCCR1B |= (1 << WGM12);   // CTC mode
  TCCR1B |= (1 << CS12);    // 256 prescaler
  TIMSK1 |= (1 << OCIE1A);  // enable timer compare interrupt
  interrupts();             // enable all interrupts

}


void setup() {
  // put your setup code here, to run once:
  Serial.begin(9600);
  Serial3.begin(9600);
  bluetooth.begin(9600);
  Setup_Gyro();
  Setup_LowLevel();
  OpenTimer();
  Serial.print("Mode Gyro = ");
  Serial.println(ModeGyro);
  pinMode(LimitForward, INPUT_PULLUP);
  pinMode(LimitBackward, INPUT_PULLUP);
  pinMode(sw_control, INPUT_PULLUP);
  Serial.println("Connecting");

  //Serial.println("Timer On");

  Serial.println("Setup OK");
}

void loop()
{
  //delay(1000);
  //checkLimit();
  //Test();
  //GGG();
  //RobotBackward();
  //Android();
  //CheckVoltage();
  RunGyro();
  runMode();
}

void runMode()
{
  if (ModePicture == 1)
  {
    ModeGyro = 0;
    Speed_Left = 230;
    Speed_Right = 230;
    speedmodechange = 5;
    Serial.println("sq");
    DrawSQ();

  }
  else if (ModePicture == 2)
  {
    ModeGyro = 0;
    Speed_Left = 230;
    Speed_Right = 230;
    speedmodechange = 5;
    Serial.println("tri");
    DrawTRI();
  }
  else if (ModePicture == 3)
  {
    Speed_Left = 230;
    Speed_Right = 230;
    speedmodechange = 5;
    ModeGyro = 0;
    Serial.println("rec");
    DrawREC();
  }
}



ISR(TIMER1_COMPA_vect)          // timer compare interrupt service routine
{
  limitForward = digitalRead(LimitForward);
  limitBackward = digitalRead(LimitBackward);
  sensorValue = analogRead(sensorPin);
  SWcontrol = digitalRead(sw_control);

  //  bluetooth.print("RT24.7");
  //  //bluetooth.println(readVoltage());
  //  //VC_Volt_Sensor();
  //  bluetooth.print("GY");
  //  bluetooth.print(NewCurrentAngle);
  //  bluetooth.print("WH");
  //  bluetooth.print(WheelLeft);
  //  bluetooth.println();

  if (limitForward == HIGH)
  {
    ValueLimitForward = 1;
  }
  else
  {
    ValueLimitForward = 0;
  }
  if (limitBackward == HIGH)
  {
    ValueLimitBackward = 1;
  }
  else
  {
    ValueLimitBackward = 0;
  }

  if (SWcontrol == HIGH)
  {
    /////////////Android////////////////////
    SWcontrol = 1;
    String content = "";
    char character;
    String pass = "";
    if (ModeGyro == 0)
    {
      if (checkbodyForward == 1 && ValueLimitForward == 0 )
      {
        if (ValueLimitBackward == 0)
        {
          BodyForward();
          checkbodyForward = 1;
          checkbodyBackward = 0;
          //Serial.println("222");
        }
        else
        {
          BodyForward();
          checkbodyForward = 1;
          checkbodyBackward = 0;
          //Serial.println("222");
        }
      }
      else if (checkbodyBackward == 1 && ValueLimitBackward == 0)
      {
        if (ValueLimitForward == 0)
        {
          BodyBackward();
          checkbodyForward = 0;
          checkbodyBackward = 1;
        }
        else
        {
          BodyBackward();
          checkbodyForward = 0;
          checkbodyBackward = 1;
        }
        //Serial.println("333");
      }
      else
      {
        StopBody();
        checkbodyForward = 0;
        checkbodyBackward = 0;
        //Serial.println("111");
      }
    }
    /*if (speedmodechange == 0)
      {
       Speed_Left = 230;
       Speed_Right = 230;
       //Serial.println("sp 0");
      }
      else if (speedmodechange == 1)
      {
       Speed_Left = 220;
       Speed_Right = 220;
       //Serial.println("sp 1");
      }
      else if (speedmodechange == 2)
      {
       Speed_Left = 210;
       Speed_Right = 210;
       //Serial.println("sp 2");
      }
      else if (speedmodechange == 3)
      {
       Speed_Left = 200;
       Speed_Right = 200;
       //Serial.println("sp 3");
      }
      else if (speedmodechange == 4)
      {
       Speed_Left = 100;
       Speed_Right = 100;
       //Serial.println("sp 4");
      }*/
    if (bluetooth.available() > 0 )
    {

    }
    else
    {
      String SentBT = RT + batt + GY + NewCurrentAngle + WH + WheelLeft + SW + modecontrol;
      delay(25);
      //Serial.println(SentBT);
      bluetooth.println(SentBT);
    }


    while (bluetooth.available())
    {
      limitForward = digitalRead(LimitForward);
      limitBackward = digitalRead(LimitBackward);

      if (limitForward == HIGH)
      {
        ValueLimitForward = 1;
      }
      else
      {
        ValueLimitForward = 0;
      }
      if (limitBackward == HIGH)
      {
        ValueLimitBackward = 1;
      }
      else
      {
        ValueLimitBackward = 0;
      }

      character = bluetooth.read();
      Serial.print(character);
      //      if (character == '*')
      //      {
      //        content = "";
      //        //content.concat(character);
      //      }
      //      else
      if (character == '#')
      {
        //String Data_Sub = content.substring(0, 1);
        //Serial.println(Data_Sub);
        //C = Control
        //        if (Data_Sub == "R")
        //        {
        //          Serial.print(content);
        //          Serial.print("222");
        if (ModePicture == 0)
        {
          if (content == "RTF")
          {
            CheckAndroid = "rtf";
            //Serial.print(CheckAndroid);
            RobotForward();
            spchange = 1;
          }
          if (content == "RTB")
          {
            CheckAndroid = "RTB";
            //Serial.println("buttom");
            //Serial.print(CheckAndroid);
            RobotBackward();
            spchange = 1;
          }
          if (content == "RTR")
          {
            CheckAndroid = "RTR";
            //Serial.println("right");
            //Serial.print(CheckAndroid);
            RobotTrunRight();
            spchange = 1;
          }
          if (content == "RTL")
          {
            CheckAndroid = "RTL";
            //Serial.print(CheckAndroid);
            RobotTrunLeft();
            spchange = 1;
            //Serial.println("left");
          }
          if (content == "RTU" && ModeGyro == 0 && limitForward == LOW)
          {
            CheckAndroid = "RTU";
            // Serial.print(CheckAndroid);
            checkbody = 1;
            checkbodyForward = 1;
            checkbodyBackward = 0;
            BodyForward();
            //while(limitForward == LOW);
            // Serial.println("FUUUUUUUUUUUUUUUUU");
          }
          else if (content == "RTD" && ModeGyro == 0 && limitBackward == LOW)
          {
            CheckAndroid = "RTD";
            //Serial.print(CheckAndroid);
            checkbodyForward = 0;
            checkbodyBackward = 1;
            checkbody = 0;
            BodyBackward();
            //Serial.println("FDDDDDDDDDDDDDDDDDDD");
          }
          else
          {
            //checkbody = 2;
            checkbodyForward = 0;
            checkbodyBackward = 0;
            StopBody();
            //Serial.println("AAAAAAAAAAAAAAAAAAAA");
          }
          if (content == "RTS")
          {
            CheckAndroid = "rts";
            //Serial.print(CheckAndroid);
            StopRobot();//JoyControl ();
            //Serial.println(" + stop");
            //state=0;
            //break;
            spchange = 0;
          }
          //        }
          //        else if (Data_Sub == "S")
          //        {
          if (content == "SP0")
          {
            Speed_Left = 245;
            Speed_Right = 245;
            speedmodechange = 0;
            //Serial.print("Speed Mode 0");
          }
          else if (content == "SP1")
          {
            Speed_Left = 240;
            Speed_Right = 240;
            speedmodechange = 1;
            //Serial.print("Speed Mode 1");
          }
          else if (content == "SP2")
          {
            Speed_Left = 235;
            Speed_Right = 235;
            speedmodechange = 2;
            //Serial.print("Speed Mode 2");
          }
          else if (content == "SP3")
          {
            Speed_Left = 220;
            Speed_Right = 220;
            speedmodechange = 3;
            //Serial.print("Speed Mode 3");
          }
          else if (content == "SP4")
          {
            Speed_Left = 230;
            Speed_Right = 230;
            speedmodechange = 4;
            //Serial.print("Speed Mode 4");
          }
          else if (content == "SP5")
          {
            Speed_Left = 245;
            Speed_Right = 245;
            speedmodechange = 5;
            //Serial.print("Speed Mode 4");
          }
          //        }
          //        else if (Data_Sub == "G")
          //        {
          if (content == "GRM")
          {
            //Serial.print("Gyro Manual");
            ModeGyro = 0;
            StopBody();
          }
          else if (content == "GRA")
          {
            //Serial.print("Gyro Auto");
            ModeGyro = 1;
          }
          //        }
          //        else if (Data_Sub == "M")
          //        {
          if (content == "MPT")
          {
            Serial.print("Mode Triangle");
            ModePicture = 2;

          }
          else if (content == "MPR")
          {
            Serial.print("Mode Rectangle");
            ModePicture = 3;
          }
          else if (content == "MPS")
          {
            Serial.print("Mode Square");
            //DrawSQ();
            ModePicture = 1;
          }
        }

        //        }

        //        String SentBT = RT + batt + GY + NewCurrentAngle + WH + WheelLeft;
        //        delay(5);
        //        bluetooth.println(SentBT);
      }
      else
      {
        if ((character > 96 && character < 123 ) || (character > 64 && character < 91 ) || (character > 47 && character < 58))// a-z
        {
          content.concat(character);
        }
        //        if (character > 34)
        //        {
        //          content.concat(character);
        //        }
        else
        {

        }
        delay(10);
      }
    }
  }
  else
  {
    /////////////////////JOYSTICK/////////////////////////////
    //String contentjoy = "";
    SWcontrol  = 0;
    char character;
    String pass = "";
    while (Serial3.available())
    {
      character = Serial3.read();
      //character = Serial3.read();
      //Serial.print(character);
      if (character == '#')
      {
        String Data_Sub = contentjoy.substring(0, 1);
        //Serial.println(Data_Sub);
        //C = Control
        if (Data_Sub == "C")
        {
          if (contentjoy == "CF")
          {
            CheckJoy = "cf";
            //Serial.print(CheckJoy);
            RobotForward();
          }
          if (contentjoy == "CB")
          {
            CheckJoy = "cb";
            //Serial.println("buttom");
            //Serial.print(CheckJoy);
            RobotBackward();
          }
          if (contentjoy == "CR")
          {
            CheckJoy = "cr";
            //Serial.println("right");
            /////Serial.print(CheckJoy);
            RobotTrunRight();
          }
          if (contentjoy == "CL")
          {
            CheckJoy = "cl";
            //Serial.print(CheckJoy);
            RobotTrunLeft();
            //Serial.println("left");
          }
          if (contentjoy == "CU" && ModeGyro == 0)
          {
            CheckJoy = "cu";
            //Serial.print(CheckJoy);
            BodyForward();
            //Serial.println("left");
          }
          if (contentjoy == "CD" && ModeGyro == 0)
          {
            CheckJoy = "cd";
            //Serial.print(CheckJoy);
            BodyBackward();
            //Serial.println("left");
          }
          if (contentjoy == "CP")
          {
            CheckJoy = "cp";
            //Serial.print(CheckJoy);
            StopBody();
          }
          if (contentjoy == "CS")
          {
            CheckJoy = "cs";
            //Serial.print(CheckJoy);
            StopRobot();
          }
        }
        else if (Data_Sub == "S")
        {
          if (contentjoy == "SP0")
          {
            Speed_Left = 245;
            Speed_Right = 245;
            speedmodechange = 0;
            //Serial.println("Speed Mode 0");
          }
          else if (contentjoy == "SP1")
          {
            Speed_Left = 240;
            Speed_Right = 240;
            speedmodechange = 1;
            ///Serial.println("Speed Mode 1");
          }
          else if (contentjoy == "SP2")
          {
            Speed_Left = 210;
            Speed_Right = 210;
            speedmodechange = 2;
            // Serial.println("Speed Mode 2");
          }
          else if (contentjoy == "SP3")
          {
            Speed_Left = 200;
            Speed_Right = 200;
            speedmodechange = 3;
            //Serial.println("Speed Mode 3");
          }
          else if (contentjoy == "SP4")
          {
            Speed_Left = 230;
            Speed_Right = 230;
            speedmodechange = 4;
            //Serial.println("Speed Mode 4");
          }
        }
        else if (Data_Sub == "G")
        {
          if (contentjoy == "GRM")
          {
            // Serial.print("Gyro Manual");
            ModeGyro = 0;
          }
          else if (contentjoy == "GRA")
          {
            // Serial.print("Gyro Auto");
            ModeGyro = 1;
          }
        }

        contentjoy = "";
      }
      else
      {
        if (character > 34)
        {
          contentjoy.concat(character);
        }
        else
        {

        }

        delay (10);
      }
    }
  }
}







void RunGyro()
{
  UpdateGyro();
  NewCurrentAngle =  (int)FindMedian(LastCurrentAngle4, LastCurrentAngle3, LastCurrentAngle2, LastCurrentAngle, CurrentAngle);

  LastCurrentAngle4 = LastCurrentAngle3;
  LastCurrentAngle3 = LastCurrentAngle2;
  LastCurrentAngle2 = LastCurrentAngle;
  LastCurrentAngle = CurrentAngle;
  //Serial.println(NewCurrentAngle);

  if (ModeGyro == 1) //Auto
  {
    if (NewCurrentAngle >= 88 && NewCurrentAngle <= 93) // 178 183
    {
      //delay(50);
      //      if (NewCurrentAngle >= 89 && NewCurrentAngle <= 94)
      //      {
      //      Serial.print("stop ");
      //      Serial.println(NewCurrentAngle);
      StopBody();
      //      delay(50);
      //      StopBody();

      //delay(100);
      //Serial.println("BodyStop");
      //      }
    }
    else
    {
      if (NewCurrentAngle > 93) // 182
      {
        //delay(50);
        if (NewCurrentAngle > 93)
        {
          //Pid();
          //      Motors();
          //Serial.println("BodyForward");
          limitForward = digitalRead(LimitForward);
          if (limitForward == 0)
          {
            //if ( NewCurrentAngle > 100 )
            //              BodyForward(100);
            //              targetSpeed = 100;
            //else
            //              BodyForward(110);
            //              targetSpeed = 110;
            //            Pid();
            //            ControlBody();
            BodyForwardAuto( (NewCurrentAngle > 100 ? 100 : 108) );
            //BodyForward();
            //StopBody();
          }
          else
          {
            StopBody();
          }
        }
      }
      else if (NewCurrentAngle < 88) // 178
      {
        //delay(50);
        if (NewCurrentAngle < 88)
        {
          //Serial.println("BodyBackward");
          //Pid();
          limitBackward = digitalRead(LimitBackward);
          if (limitBackward == 0)
          {
            //            Pid();
            //            ControlBody();
            BodyBackwardAuto( (NewCurrentAngle > 100 ? 100 : 108) );
            // BodyBackward();
          }
          else
          {
            StopBody();
          }
        }
      }
    }
  }
  else
  {

  }

  //String SentBT = RT + batt + GY + NewCurrentAngle + WH + WheelLeft + speedmodechange;
  // Serial.println(NewCurrentAngle);

  //  if (bluetooth.available() > 0)
  //  {
  //    // bluetooth.print(SentBT);
  //  }
  //  else
  //  {
  //Serial.println(SentBT);
  // bluetooth.println(SentBT);
  //  }
  if (SWcontrol == LOW)
  {
    modecontrol = 0;
    String SentBT = RT + batt + GY + NewCurrentAngle + WH + WheelLeft + SW + modecontrol;
    delay(5);
    //Serial.println(SentBT);
    bluetooth.println(SentBT);
  }
  else 
  {
    modecontrol = 1;
  }
  delay(50);
}

void Setup_Gyro()
{
  Wire.begin();
  i2cData[0] = 7; // Set the sample rate to 1000Hz - 8kHz/(7+1) = 1000Hz
  i2cData[1] = 0x00; // Disable FSYNC and set 260 Hz Acc filtering, 256 Hz Gyro filtering, 8 KHz sampling
  i2cData[2] = 0x00; // Set Gyro Full Scale Range to ±250deg/s
  i2cData[3] = 0x00; // Set Accelerometer Full Scale Range to ±2g
  while (i2cWrite(0x19, i2cData, 4, false)); // Write to all four registers at once
  while (i2cWrite(0x6B, 0x01, true)); // PLL with X axis gyroscope reference and disable sleep mode
  while (i2cRead(0x75, i2cData, 1));
  if (i2cData[0] != 0x68) { // Read "WHO_AM_I" register
    Serial.print(F("Error reading sensor"));
    while (1);
  }
  delay(100); // Wait for sensor to stabilize

  /* Set kalman and gyro starting angle */
  while (i2cRead(0x3B, i2cData, 6));
  accX = ((i2cData[0] << 8) | i2cData[1]);
  accY = ((i2cData[2] << 8) | i2cData[3]);
  accZ = ((i2cData[4] << 8) | i2cData[5]);
  //เอาค่าแกนY
  accYangle = (atan2(accX, accZ) + PI) * RAD_TO_DEG;

  kalmanX.setAngle(accYangle); // Set starting angle
  gyroYangle = accYangle;

  //เอาค่าแกนX
  //  accXangle = (atan2(accY, accZ) + PI) * RAD_TO_DEG;
  //
  //  kalmanX.setAngle(accXangle); // Set starting angle
  //  gyroXangle = accXangle;
  timer = micros();
}
float FindMedian(float myarr, float myarr1, float myarr2, float myarr3, float myarr4)
{
  float myArray[5] = {myarr, myarr1, myarr2, myarr3, myarr4} ;
  sortArray(myArray, 5);
  //  printArray("sort: ",myArray);
  //delay(10);
  return myArray[2];
}
void printArray(String msg, float* myArray) {
  Serial.print(msg);
  Serial.print(" ");
  Serial.print(myArray[0]);
  Serial.print(" ");
  Serial.print(myArray[1]);
  Serial.print(" ");
  Serial.print(myArray[2]);
  Serial.print(" ");
  Serial.print(myArray[3]);
  Serial.print(" ");
  Serial.print(myArray[4]);
  Serial.print(" ");
}

void Pid() {
  //  error = 180 - CurrentAngle;  // 180 = level
  error = 180 - NewCurrentAngle;
  pTerm = Kp * error; //P
  integrated_error += error;
  //iTerm = Ki * constrain(integrated_error, -GUARD_GAIN, GUARD_GAIN);//I
  //iTerm += error * Kp;
  dTerm = Kd * (error - last_error);
  last_error = error;
  //speed = constrain(K*(pTerm + iTerm + dTerm), -200, 200);
  speed = constrain(K * (pTerm + dTerm), -150, 150);
  //Speed = (K*(pTerm + iTerm + dTerm))/255;
  if (speed > 0)
  {
    Speed_Body = (-speed); //+ 46; //60,85,48
    if (Speed_Body > 110)
    {
      Speed_Body = 110;
    }
    else if (Speed_Body < 100)
    {
      Speed_Body = 100;
    }
    //BodyForward();
  }
  else
  {
    Speed_Body = speed; //+ 50; //60,60
    if (Speed_Body > 110)
    {
      Speed_Body = 110;
    }
    else if (Speed_Body < 100)
    {
      Speed_Body = 100;
    }
  }
}

void UpdateGyro()
{
  while (i2cRead(0x3B, i2cData, 14));
  accX = ((i2cData[0] << 8) | i2cData[1]);
  accY = ((i2cData[2] << 8) | i2cData[3]);
  accZ = ((i2cData[4] << 8) | i2cData[5]);
  tempRaw = ((i2cData[6] << 8) | i2cData[7]);
  gyroX = ((i2cData[8] << 8) | i2cData[9]);
  gyroY = ((i2cData[10] << 8) | i2cData[11]);
  gyroZ = ((i2cData[12] << 8) | i2cData[13]);
  //เอาค่าแกนX
  //  accXangle = (atan2(accY, accZ) + PI) * RAD_TO_DEG;
  //  double gyroXrate = (double)gyroX / 131.0;
  //เอาค่าแกนY
  accYangle = (atan2(accX, accZ) + PI) * RAD_TO_DEG;
  double gyroYrate = (double)gyroY / 131.0;

  //เอาค่าแกนY
  CurrentAngle = kalmanX.getAngle(accYangle, gyroYrate, (double)(micros() - timer) / 1000000);
  //เอาค่าแกนX
  //  CurrentAngle = kalmanX.getAngle(accXangle, gyroXrate, (double)(micros() - timer) / 1000000);
  timer = micros();
  //CurrentAngle = (int)CurrentAngle;
}

void Setup_LowLevel()
{
  pinMode(Motor_Left_Forward, OUTPUT);
  pinMode(Motor_Left_Backward, OUTPUT);
  pinMode(PWM_Left, OUTPUT);

  pinMode(Motor_Right_Forward, OUTPUT);
  pinMode(Motor_Right_Backward, OUTPUT);
  pinMode(PWM_Right, OUTPUT);

  pinMode(Motor_Body_Forward, OUTPUT);
  pinMode(Motor_Body_Backward, OUTPUT);
  pinMode(PWM_Body, OUTPUT);

}


void StopRobot()
{
  digitalWrite(Motor_Left_Forward, LOW);
  digitalWrite(Motor_Left_Backward, LOW);
  analogWrite(PWM_Left, 200);
  digitalWrite(Motor_Right_Forward, LOW);
  digitalWrite(Motor_Right_Backward, LOW);
  analogWrite(PWM_Right, 200);
  WheelLeft = "00";

  //Serial.println("STOP");
}



void RobotForward()
{
  int Speed_RightT = Speed_Right;
  int Speed_LeftT = Speed_Left - 5;
  digitalWrite(Motor_Left_Forward, HIGH);
  digitalWrite(Motor_Left_Backward, LOW);
  analogWrite(PWM_Left, Speed_Left);
  WheelLeft = "01";
  digitalWrite(Motor_Right_Forward, HIGH);
  digitalWrite(Motor_Right_Backward, LOW);
  analogWrite(PWM_Right, Speed_Right);

}

void RobotBackward()
{
  int Speed_RightT = Speed_Right;
  int Speed_LeftT = Speed_Left - 3;
  digitalWrite(Motor_Left_Forward, LOW);
  digitalWrite(Motor_Left_Backward, HIGH);
  analogWrite(PWM_Left, Speed_Left);
  WheelLeft = "10";
  digitalWrite(Motor_Right_Forward, LOW);
  digitalWrite(Motor_Right_Backward, HIGH);
  analogWrite(PWM_Right, Speed_Right);
}


void RobotTrunLeft()
{
  int spL = Speed_Left - 5;
  int spR = Speed_Right - 5;
  digitalWrite(Motor_Left_Forward, LOW);
  digitalWrite(Motor_Left_Backward, HIGH);
  analogWrite(PWM_Left, spL);
  WheelLeft = "10";
  digitalWrite(Motor_Right_Forward, HIGH);
  digitalWrite(Motor_Right_Backward, LOW);
  analogWrite(PWM_Right, spR);
}


void RobotTrunRight()
{
  int spL = Speed_Left - 5;
  int spR = Speed_Right - 5;
  digitalWrite(Motor_Left_Forward, HIGH);
  digitalWrite(Motor_Left_Backward, LOW);
  analogWrite(PWM_Left, spL);
  WheelLeft = "01";
  digitalWrite(Motor_Right_Forward, LOW);
  digitalWrite(Motor_Right_Backward, HIGH);
  analogWrite(PWM_Right, spR);
}




void StopBody()
{
  digitalWrite(Motor_Body_Forward, LOW);
  digitalWrite(Motor_Body_Backward, LOW);
  analogWrite(PWM_Body, 250);
}

void ControlBody()
{
  if (speed > 0)
  {
    //forward
    int SpeedBody = ((Speed_Body - 255) * (-1));
    BodyForwardAuto(Speed_Body);
    //Serial.print(Speed_Body);
    Serial.print("    SpeedF : ");    Serial.println(Speed_Body);
  }

  else if (speed < 0)
  {
    // backward
    //Speed = map(Speed,0,-255,0,255);
    int SpeedBody = ((Speed_Body - 255) * (-1));
    BodyBackwardAuto(Speed_Body);
    //Serial.print(Speed_Body);
    Serial.print("    SpeedB : ");    Serial.println(Speed_Body);
  }
  else
  {
    StopBody();

    Serial.print("    SpeedS : ");    Serial.println(Speed_Body);
  }
}


void BodyForwardAuto(int sp)
{
  checkbody = 1;
  limitForward = digitalRead(LimitForward);
  if (limitForward == HIGH || ValueLimitForward == 1)
  {
    StopBody();
    //Serial.println("STOP");
  }
  else
  {
    digitalWrite(Motor_Body_Forward, HIGH);
    digitalWrite(Motor_Body_Backward, LOW);
    analogWrite(PWM_Body, sp);
    //Serial.println("Motor ON");
  }

}



void BodyForward()
{
  checkbody = 1;
  limitForward = digitalRead(LimitForward);
  if (limitForward == HIGH || ValueLimitForward == 1)
  {
    StopBody();
    //Serial.println("STOP");
  }
  else
  {
    digitalWrite(Motor_Body_Forward, HIGH);
    digitalWrite(Motor_Body_Backward, LOW);
    analogWrite(PWM_Body, Speed_Body);
    //Serial.println("Motor ON");
  }

}


void BodyBackward()
{
  checkbody = 0;
  limitBackward = digitalRead(LimitBackward);
  if (limitBackward == HIGH)
  {
    StopBody();
    // Serial.println("KKKKK");
  }
  else
  {
    digitalWrite(Motor_Body_Forward, LOW);
    digitalWrite(Motor_Body_Backward, HIGH);
    analogWrite(PWM_Body, Speed_Body);
  }
}


void BodyBackwardAuto(int sp)
{
  checkbody = 0;
  limitBackward = digitalRead(LimitBackward);
  if (limitBackward == HIGH)
  {
    StopBody();
    // Serial.println("KKKKK");
  }
  else
  {
    digitalWrite(Motor_Body_Forward, LOW);
    digitalWrite(Motor_Body_Backward, HIGH);
    analogWrite(PWM_Body, sp);
  }
}



float readVoltage()
{
  sensorValue = analogRead(sensorPin);
  vout = ((sensorValue * 5.0) / 1024.0);
  vin = vout / (R2 / (R1 + R2));
  return vin;
}


void GGG()
{
  string = "";
  //  cmd = "";
  while (bluetooth.available())
  {
    char cmd = ((char)bluetooth.read());

    if (cmd == '#') {
      break;
    } else {
      string += cmd;
    }
    delay(1);
  }
  Serial.print(string);
  if (string == "GRM")
  {
    Serial.print("grm");
  }
  else if (string == "SP0")
  {
    Serial.print("sp0");
  }
  if (string == "RTF")
  {
    Serial.print("rtf");
  }
  else if (string == "RTS")
  {
    Serial.print("rts");
  }
}



void Test()
{
  //Read from bluetooth and write to usb serial
  if (bluetooth.available())
  {
    char toSend = (char)bluetooth.read();
    //Serial.print("T");
    Serial.print(toSend);
    //Serial3.print(toSend);

  }


  //Read from usb serial to bluetooth
  if (Serial.available())
  {
    char toRec = (char)Serial.read();
    bluetooth.print(toRec);
    //Serial3.print(toRec);
    //Serial.print("R");
    //Serial.print(toRec);
  }

  // delay(50);
  if (Serial3.available())
  {
    char toSend = (char)Serial3.read();
    //Serial.print("t");

    Serial.print(toSend);
  }
}




void VC_Volt_Sensor()
{
  voltage = ((sensorValue * (5.045 / 1023.0)) * 5); //5.72 .045
  Serial.println(voltage);
}

void DrawSQ ()
{
  RobotForward();
  delay(1000);
  StopRobot();
  delay(2000);
  RobotTrunRight();
  delay(485);
  StopRobot();
  delay(2000);

  RobotForward();
  delay(1000);
  StopRobot();
  delay(2000);
  RobotTrunRight();
  delay(485);
  StopRobot();
  delay(2000);

  RobotForward();
  delay(1000);
  StopRobot();
  delay(2000);
  RobotTrunRight();
  delay(485);
  StopRobot();
  delay(2000);

  RobotForward();
  delay(1000);
  StopRobot();
  delay(2000);
  RobotTrunRight();
  delay(485);
  StopRobot();
  ModePicture = 0;
}


void DrawTRI ()
{
  RobotForward();
  delay(1500);
  StopRobot();
  delay(2000);
  RobotTrunRight();
  delay(650);
  StopRobot();
  delay(2000);

  RobotForward();
  delay(1500);
  StopRobot();
  delay(2000);
  RobotTrunRight();
  delay(600);
  StopRobot();
  delay(2000);

  RobotForward();
  delay(1500);
  StopRobot();
  delay(2000);
  RobotTrunRight();
  delay(600);
  StopRobot();
  delay(2000);

  ModePicture = 0;
}



void DrawREC ()
{
  RobotForward();
  delay(1000);
  StopRobot();
  delay(2000);
  RobotTrunRight();
  delay(490);
  StopRobot();
  delay(2000);

  RobotForward();
  delay(2000);
  StopRobot();
  delay(2000);
  RobotTrunRight();
  delay(495);
  StopRobot();
  delay(2000);

  RobotForward();
  delay(1000);
  StopRobot();
  delay(2000);
  RobotTrunRight();
  delay(510);
  StopRobot();
  delay(2000);

  RobotForward();
  delay(2000);
  StopRobot();
  delay(2000);
  RobotTrunRight();
  delay(490);
  StopRobot();
  ModePicture = 0;
}




