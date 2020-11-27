#include <Arduino.h>
#include <MPU6050_tockn.h>
#include <Wire.h>

// Serial CODE init
String inputString1 = "";
String inputString2 = "";
String inputString3 = "";
String sendStrig = "";

//HardwareSerial Serial1(USART1);
HardwareSerial Serial2(USART2); // or HardWareSerial Serial2 (PA3, PA2);
HardwareSerial Serial3(USART3);

#define HOVER_SERIAL_BAUD 115200     // [-] Baud rate for HoverSerial (used to communicate with the hoverboard)
#define RASPBERRY_SERIAL_BAUD 115200 // [-] Baud rate for HoverSerial (used to communicate with the hoverboard)
#define BLUETOOTH_SERIAL_BAUD 115200 // [-] Baud rate for HoverSerial (used to communicate with the hoverboard)

void initSerial()
{

  inputString1.reserve(64);
  inputString2.reserve(64);
  inputString3.reserve(64);

  Serial.begin(9600);
  Serial1.begin(RASPBERRY_SERIAL_BAUD); // on PA9 PA10
  Serial2.begin(HOVER_SERIAL_BAUD);     // on PA3 PA2
  Serial3.begin(BLUETOOTH_SERIAL_BAUD); // on PB11 PB10

  Serial1.println("Serial1:");
  Serial2.println("Serial2:");
  Serial3.println("Serial3:");
}

int status;
float pitch;
float roll;
float yaw;
int timerIMU;
bool existIMU;
MPU6050 IMU(Wire);
void initIMU()
{
  delay(100);
  IMU.begin();
  delay(100);
  existIMU = true;

  IMU.calcGyroOffsets(false, 3000, 1000); //auto calibration
  delay(500);
  IMU.update();
  float angle = abs(IMU.getAngleZ());
  delay(200);
  IMU.update();
  if (abs(angle - abs(IMU.getGyroZ())) > 0.5)
  {
    IMU.setGyroOffsets(-2.18, -1.89, 1.25); //Manual calibration
  }
}
#define G 9.81;
float covertAccel(float accel_mss)
{
  return accel_mss / G;
}

float covertGyro(float gyro_rad)
{
  return gyro_rad * RAD_TO_DEG;
}
//Madgwick filter;

void imuData()
{

  if (existIMU)
  {

    IMU.update();

    yaw = IMU.getAngleZ() * (-1.0);
    pitch = IMU.getAngleY();
    roll = IMU.getAngleX();
  }
}

#include <VL53L0X.h> // SENZOR CODE
VL53L0X Sensor1;
VL53L0X Sensor2;
#define SENZOR1_ADDRESS 0x30
#define SENZOR2_ADDRESS 0x34
// set the pins to shutdown
#define SHT_SENZOR1 PB8
#define SHT_SENZOR2 PB9
int senzor1;
int senzor2;
bool existSenzor1;
bool existSenzor2;

void initSenzor()
{

  pinMode(SHT_SENZOR1, OUTPUT);
  pinMode(SHT_SENZOR2, OUTPUT);
  // all reset
  digitalWrite(SHT_SENZOR1, LOW);
  digitalWrite(SHT_SENZOR2, LOW);
  delay(10);

  // activating LOX1 and reseting LOX2
  digitalWrite(SHT_SENZOR1, HIGH);
  digitalWrite(SHT_SENZOR2, LOW);
  delay(10);
  Sensor1.setAddress(SENZOR1_ADDRESS);
  Sensor1.setTimeout(200);
  if (!Sensor1.init())
  {
    Serial.println("ERROR VL53L0X Sensor1 !");
    // while (1) {}
    existSenzor1 = false;
  }
  else
  {
    existSenzor1 = true;
    /* // lower the return signal rate limit (default is 0.25 MCPS)
    Sensor1.setSignalRateLimit(0.1);
    //increase laser pulse periods (defaults are 14 and 10 PCLKs)
    Sensor1.setVcselPulsePeriod(VL53L0X::VcselPeriodPreRange, 18);
    Sensor1.setVcselPulsePeriod(VL53L0X::VcselPeriodFinalRange, 14);
    //Serial.print(Sensor1.getAddress());
    //Serial.println("Senzor1 I2C adresa");*/
    Sensor1.startContinuous();
  }

  delay(10);
  // activating LOX2 and reseting LOX1
  digitalWrite(SHT_SENZOR1, HIGH);
  digitalWrite(SHT_SENZOR2, HIGH);
  delay(10);
  Sensor2.setAddress(SENZOR2_ADDRESS);
  Sensor2.setTimeout(200);
  if (!Sensor2.init())
  {
    Serial.println("ERROR VL53L0X Sensor2 !");
    //while (1) {}
    existSenzor2 = false;
  }
  else
  {
    existSenzor2 = true;

    /*// lower the return signal rate limit (default is 0.25 MCPS)
    Sensor2.setSignalRateLimit(0.1);
    // increase laser pulse periods (defaults are 14 and 10 PCLKs)
    Sensor2.setVcselPulsePeriod(VL53L0X::VcselPeriodPreRange, 18);
    Sensor2.setVcselPulsePeriod(VL53L0X::VcselPeriodFinalRange, 14);
    // Serial.print(Sensor2.getAddress());
    // Serial.println("Senzor2 I2C adresa");*/

    Sensor2.startContinuous();
  }
}

#define SERIAL_BAUD 115200 // [-] Baud rate for built-in Serial (used for the Serial Monitor)
#define START_FRAME 0xABCD // [-] Start frme definition for reliable serial communication
#define TIME_SEND 100      // [ms] Sending time interval
#define SPEED_MAX_TEST 300 // [-] Maximum speed for testing
//#define DEBUG_RX                        // [-] Debug received data. Prints all bytes to serial (comment-out to disable)

//HardwareSerial HoverSerial(USART2);   // or HardWareSerial Serial2 (PA3, PA2);

// Global variables
long last_send = 0;

// Global variables
uint8_t idx = 0;        // Index for new data pointer
uint16_t bufStartFrame; // Buffer Start Frame
byte *p;                // Pointer declaration for the new received data
byte incomingByte;
byte incomingBytePrev;

typedef struct
{
  uint16_t start;
  int16_t speedLeft;
  int16_t speedRight;
  uint16_t checksum;
} SerialCommand;
SerialCommand Command;

typedef struct
{
  uint16_t start;
  int16_t cmd1;
  int16_t cmd2;
  int16_t speedR_meas;
  int16_t speedL_meas;
  int16_t batVoltage;
  int16_t boardTemp;
  uint16_t cmdLed;
  uint16_t checksum;
} SerialFeedback;
SerialFeedback Feedback;
SerialFeedback NewFeedback;

#include "Ticker.h" // Timer code
void powerButons();
void mySerialEvent(); //Def timer funkcion
void imuData();
void senzorData();
void driveMotor();
//void sendData();

Ticker timer0(powerButons, 100, 0, MILLIS);
Ticker timer1(mySerialEvent, 10, 0, MILLIS);
Ticker timer2(imuData, 50, 0, MILLIS);
Ticker timer3(senzorData, 35, 0, MILLIS);
Ticker timer4(driveMotor, 100, 0, MILLIS);
//Ticker timer5(sendData, 150, 0, MILLIS);

#include <PID_v1.h>
//PID regulators
double yawPidSet, yawPidInp, yawPidOut;
//Specify the links and initial tuning parameters
double yawPidKp = 0.5, yawPidKi = 0.0, yawPidKd = 0.1;
PID yawPID(&yawPidInp, &yawPidOut, &yawPidSet, yawPidKp, yawPidKi, yawPidKd, DIRECT);

#define POWERHOLD PB0
#define CENTRALSTOP PA0
#define POWERON PB1

void setup()
{ // Start inicializacion Program
  Wire.begin();
  initSerial();
  initIMU();

  initSenzor();

  pinMode(POWERHOLD, OUTPUT);
  digitalWrite(POWERHOLD, HIGH);      // Hold power MCU 5V
  pinMode(CENTRALSTOP, INPUT_ANALOG); // central stop buton
  pinMode(POWERON, INPUT_ANALOG);     // power buton on cca 1V on
  analogReadResolution(12);           //12b 4095
  imuData();                          //read YAW from IMU
  //tell the PID to range between 0 and the full window size
  yawPID.SetOutputLimits(-30, 30);
  yawPID.SetSampleTime(100);
  //turn the PID on
  yawPID.SetMode(AUTOMATIC);
  yawPidSet = yaw;

  timer0.start();
  timer1.start();
  timer2.start();
  timer3.start();
  timer4.start();
  //timer5.start();
}

bool centralStop = true;
float batteryVolt;
float batteryPercent;

void powerButons()
{
#define TREESHOT 500
#define ADMAX 4095
  //const float ADREF = 3.30;
  const float BATTERYMAX = 42.0;
  const float BATTERYMIN = 31.0;
  const float BATTERYCALIBR = 0.01939;
  const float POWEROFF = 30.0;

  static int adStopBtnOld = 0;
  static float adOnBtnOndSum = 0;
  static int adOnBtnOndInc = 0;
  //static int PowerFuncion = 0;

  int adStopBtn = analogRead(CENTRALSTOP);

  if (adStopBtn > TREESHOT && adStopBtnOld > TREESHOT) // central stop buton
  {
    centralStop = false;
  }
  else
  {
    centralStop = true;
  }
  adStopBtnOld = adStopBtn;

  adOnBtnOndSum += analogRead(POWERON);
  adOnBtnOndInc++;
  if (adOnBtnOndInc >= 10)
  {
    batteryVolt = (adOnBtnOndSum / adOnBtnOndInc) * BATTERYCALIBR;
    batteryPercent = map(batteryVolt, BATTERYMIN, BATTERYMAX, 0, 100);
    batteryPercent = constrain(batteryPercent, 0, 100);
    adOnBtnOndSum = 0;
    adOnBtnOndInc = 0;
    if (batteryVolt < POWEROFF)
    {
      digitalWrite(POWERHOLD, LOW); // OFF power MCU 5V
      centralStop = true;
    }
  }
}

// ########################## SEND ##########################
void SendSerial2(int16_t uSpeedLeft, int16_t uSpeedRight)
{
  // Create command
  Command.start = (uint16_t)START_FRAME;
  Command.speedLeft = (int16_t)uSpeedLeft;
  Command.speedRight = (int16_t)uSpeedRight;
  Command.checksum = (uint16_t)(Command.start ^ Command.speedLeft ^ Command.speedRight);

  // Write to Serial
  Serial2.write((uint8_t *)&Command, sizeof(Command));
}

void ReceiveSerial2()
{
  // Check for new data availability in the Serial buffer
  if (Serial2.available())
  {
    incomingByte = Serial2.read();                                      // Read the incoming byte
    bufStartFrame = ((uint16_t)(incomingByte) << 8) | incomingBytePrev; // Construct the start frame
  }
  else
  {
    return;
  }

  // Copy received data
  if (bufStartFrame == START_FRAME)
  { // Initialize if new data is detected
    p = (byte *)&NewFeedback;
    *p++ = incomingBytePrev;
    *p++ = incomingByte;
    idx = 2;
  }
  else if (idx >= 2 && idx < sizeof(SerialFeedback))
  { // Save the new received data
    *p++ = incomingByte;
    idx++;
  }

  // Check if we reached the end of the package
  if (idx == sizeof(SerialFeedback))
  {
    uint16_t checksum;
    checksum = (uint16_t)(NewFeedback.start ^ NewFeedback.cmd1 ^ NewFeedback.cmd2 ^ NewFeedback.speedR_meas ^ NewFeedback.speedL_meas ^ NewFeedback.batVoltage ^ NewFeedback.boardTemp ^ NewFeedback.cmdLed);

    // Check validity of the new data
    if (NewFeedback.start == START_FRAME && checksum == NewFeedback.checksum)
    {
      // Copy the new data
      memcpy(&Feedback, &NewFeedback, sizeof(SerialFeedback));
    }
    else
    {
      //Serial3.println("Non-valid data skipped");
    }
    idx = 0; // Reset the index (it prevents to enter in this if condition in the next cycle)
  }

  // Update previous states
  incomingBytePrev = incomingByte;
}

int motorL = 0;
int motorR = 0;
int robAngle = 0;
int motPoc = 5000; //off

void parametrsMotor(String input)
{
  if (input.length() > 4)
  {

    int pos1 = input.indexOf('L') + 1;
    int pos2 = input.indexOf(',');

    if (pos1 < pos2 && pos1 > 0)
    {
      motorL = (input.substring(pos1, pos2)).toInt();
      pos1 = input.indexOf('R') + 1;
      pos2 = input.length();
      if (pos1 < pos2 && pos1 > 0)
      {
        motorR = (input.substring(pos1, pos2)).toInt();
        motPoc = 0;
      }
    }
  }
}

char questionHead = ' ';
String questionBody = "";
char commandHead = ' ';
String commandBody = "";
char hashtagHead = ' ';
String hashtagHeadBody = "";
float messagePar[4];
void messageBody(String body)
{
  int pos1 = 0, pos2 = 0;
  bool end = false;
  if (body.length() == 0)
    end = true;
  if (body.charAt(0) == ';')
    pos1 = 1;

  for (int i = 0; i < 4; i++)
  {
    if (end)
    {
      messagePar[i] = 0;
    }
    else
    {
      pos2 = body.indexOf(';', pos1);
      if (pos2 > 0)
      {

        messagePar[i] = (body.substring(pos1, pos2)).toFloat();
        pos1 = pos2 + 1;
      }
      else
      {
        messagePar[i] = (body.substring(pos1, body.length())).toFloat();
        end = true;
      }
    }
  }
}

String messageF3(char question, float a, float b, float c)
{
  String message = "";
  message = "?" + String(question) + String(a, 2) + ";" + String(b, 2) + ";" + String(c, 2);
  return message;
}

String messageI3(char question, int a, int b, int c)
{
  String message = "";
  message = "?" + String(question) + String(a, DEC) + ";" + String(b, DEC) + ";" + String(c, DEC);
  return message;
}

void sendMessage(String message, int port)
{
  switch (port)
  {
  case 0:
    Serial.println(message);
    break;
  case 1:
    Serial1.println(message);
    break;
  case 2:
    Serial2.println(message);
    break;
  case 3:
    Serial3.println(message);
    break;

  default:
    break;
  }
}

void sendData(char question, int port)
{

  switch (question)
  {
  case 'M':
    sendMessage(messageI3(question, Feedback.speedR_meas, Feedback.speedL_meas, Feedback.boardTemp), port);
    break;

  case 'A':
    sendMessage(messageF3(question, IMU.getAccAngleX(), IMU.getAccAngleY(), 0.0), port);
    break;

  case 'O':
    sendMessage(messageF3(question, IMU.getGyroXoffset(), IMU.getGyroYoffset(), IMU.getGyroZoffset()), port);
    break;

  case 'G':
    sendMessage(messageF3(question, IMU.getGyroAngleX(), IMU.getGyroAngleY(), IMU.getGyroAngleZ()), port);
    break;

  case 'C':
    sendMessage(messageI3(question, centralStop, 0, 0), port);
    break;

  case 'I':
    sendMessage(messageF3(question, pitch, roll, yaw), port);
    break;

  case 'B':
    sendMessage(messageF3(question, batteryVolt, batteryPercent, 0.0), port);
    break;

  case 'S':
    sendMessage(messageI3(question, senzor1, senzor2, 0), port);
    break;

  case '?':
    sendMessage(" Robot;0.1", port);
    break;

  case ' ':
    break;

  default:
    sendMessage(" COMAND ERROR", port);
    break;
  }
}

int senzorStop = 500; //50cm auto stop
int autoBrake = 10;   //10=1s

void setParamters()
{
  messageBody(hashtagHeadBody);
  switch (hashtagHead)
  {

  case 'A':
    robAngle = constrain(messagePar[0], -360.0, 360.0);
    break;
  case 'B':
    autoBrake = constrain(messagePar[0], 0.0, 100.0);
    break;

  case 'S':
    senzorStop = constrain(messagePar[0], 0.0, 5000.0);
    break;

  case 'Y':
    yawPidKp = constrain(messagePar[0], 0.0, 10.0);
    yawPidKi = constrain(messagePar[1], 0.0, 10.0);
    yawPidKd = constrain(messagePar[2], 0.0, 10.0);
    yawPID.SetTunings(yawPidKp, yawPidKi, yawPidKd);
    break;

  case ' ':
    // statements
    break;
  default:
    // statements
    break;
  }
  hashtagHead = ' ';
  hashtagHeadBody = "";
}

void setCommand()
{
  messageBody(commandBody);
  switch (commandHead)
  {

  case 'A':
    robAngle = constrain(messagePar[0], -360.0, 360.0);
    break;
  case 'D':
    motorL = constrain(messagePar[0], -500.0, 500.0);
    motorR = constrain(messagePar[1], -500.0, 500.0);
    motPoc = 0;
    break;

  case 'C':
    robAngle = constrain(messagePar[0], -360.0, 360.0);
    motorL = motorR = constrain(messagePar[1], -500.0, 500.0);
    motPoc = 0;
    break;

  case ' ':
    // statements
    break;
  default:
    // statements
    break;
  }
  commandHead = ' ';
  commandBody = "";
}

void message(String input, int port)
{
  input.trim();
  input.toUpperCase();
  int mLength = input.length();
  if (mLength > 1)
  {

    switch (input.charAt(0))
    {
    case 'L':
      parametrsMotor(input);
      break;

    case '?':
      questionHead = input.charAt(1);
      if (mLength > 2)
      {
        questionBody = input.substring(2);
        questionBody.trim();
      }
      else
      {
        questionBody = "";
      }
      sendData(questionHead, port);
      break;

    case '!':
      commandHead = input.charAt(1);
      if (mLength > 2)
      {
        commandBody = input.substring(2);
        commandBody.trim();
        setCommand();
      }
      else
      {
        commandBody = "";
      }
      break;

    case '#':
      hashtagHead = input.charAt(1);
      if (mLength > 2)
      {
        hashtagHeadBody = input.substring(2);
        hashtagHeadBody.trim();
        setParamters();
      }
      else
      {
        hashtagHeadBody = "";
      }
      break;
    default:

      break;
    }
  }
}

void mySerialEvent1()
{
  while (Serial1.available() > 0)
  {
    char inChar = (char)Serial1.read();
    if (inChar == '\n' || inChar == '\r' || inputString1.length() > 32)
    {
      if (inputString1.length() > 0)
      {
        //Serial2.println(inputString1);
        message(inputString1, 1);
        inputString1 = "";
      }
    }
    else
    {
      inputString1 += inChar;
    }
  }
}

void mySerialEvent2()
{
  while (Serial2.available() > 0)
  {
    char inChar = (char)Serial2.read();
    if (inChar == '\n' || inChar == '\r')
    {
      if (inputString2.length() > 0)
      {
        //Serial1.println(inputString2);
        //Serial3.println(inputString2);
        inputString2 = "";
      }
    }
    else
    {
      inputString2 += inChar;
    }
  }
}

void mySerialEvent3()
{
  while (Serial3.available() > 0)
  {
    char inChar = (char)Serial3.read();
    if (inChar == '\n' || inChar == '\r')
    {
      if (inputString3.length() > 0)
      {
        //Serial2.println(inputString3);
        message(inputString3, 3);
        sendStrig = inputString3;
        inputString3 = "";
      }
    }
    else
    {
      inputString3 += inChar;
    }
  }
}

void mySerialEvent()
{
  mySerialEvent1();

  //mySerialEvent2();
  mySerialEvent3();
}

void senzorData()
{
  if (existSenzor1)
  {
    senzor1 = Sensor1.readRangeContinuousMillimeters();
    if (Sensor1.timeoutOccurred())
    {
      Serial.print("ERROR Senzor1");
      // existSenzor1 = false;
    }
  }

  if (existSenzor2)
  {
    senzor2 = Sensor2.readRangeContinuousMillimeters();
    if (Sensor2.timeoutOccurred())
    {
      Serial.print("ERROR Senzor2");
      //existSenzor2 = false;
    }
  }
}

#define AkcelMin 1 //
#define AkcelMAX 10
#define AkcelBrake 50
#define AkcelAngle 1
#define AkcelDiv 20
#define AutoBrakeYaw 80 //50=5s
void driveMotor()
{
  static int akMotL = 0, akMotR = 0, akcel = 0;

  bool brakeSenzor = false;
  if (akMotL > 0 && akMotR > 0) //drive front
  {
    if (existSenzor1 && senzor1 < senzorStop)
    {
      brakeSenzor = true;
    }
    if (existSenzor2 && senzor2 < senzorStop)
    {
      brakeSenzor = true;
    }
  }

  if (motPoc == 0)
  {
    akcel = (abs(motorL) + abs(motorR)) / AkcelDiv;
    if (akcel < AkcelMin)
      akcel = AkcelMin;
    if (akcel > AkcelMAX)
      akcel = AkcelMAX;
  }

  if (motPoc > autoBrake || brakeSenzor) // auto brake 1s
  {
    motorL = 0;
    motorR = 0;
  }

  if (motPoc < 5000)
  {
    motPoc++;
  }

  if (motorL == 0 && motorR == 0)
    akcel = AkcelBrake;

  if (akMotL < motorL)
  {
    akMotL = akMotL + akcel;
    if (akMotL > motorL)
      akMotL = motorL;
  }

  if (akMotR < motorR)
  {
    akMotR = akMotR + akcel;
    if (akMotR > motorR)
      akMotR = motorR;
  }

  if (akMotL > motorL)
  {
    akMotL = akMotL - akcel;
    if (akMotL < motorL)
      akMotL = motorL;
  }

  if (akMotR > motorR)
  {
    akMotR = akMotR - akcel;
    if (akMotR < motorR)
      akMotR = motorR;
  }

  if (robAngle != 0)
  {
    motPoc = 0;
    yawPidSet = yaw + robAngle;
    robAngle = 0;
  }

  int regMotL = 0, regMotR = 0;

  yawPidInp = yaw;
  yawPID.Compute();

  yawPidInp = yaw;
  yawPID.Compute();
  if ((abs(akMotL - (akMotR)) < 10) && !brakeSenzor && motPoc < AutoBrakeYaw)
  {
    regMotL = akMotL + int(yawPidOut);
    regMotR = akMotR - int(yawPidOut);
  }
  else
  {
    yawPidSet = yaw;
    regMotL = akMotL;
    regMotR = akMotR;
  }

  if (centralStop)
  {
    SendSerial2(0, 0);
  }
  else
  {
    SendSerial2(regMotL, regMotR);
  }
}

void loop()
{ // Repeat loop forever
  timer0.update();
  timer1.update();
  ReceiveSerial2();
  timer2.update();
  timer3.update();
  timer4.update();
  ReceiveSerial2();
}