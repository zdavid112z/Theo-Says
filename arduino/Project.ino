#include <NewToneLib.h>
#include <Servo.h>
#include <HardwareSerial.h>
#include <AccelStepper.h>
// #include <Stepper.h>

#define FULLSTEP 4

#define bt Serial3

MyTone t(false);

const char* msgMB_MobileOn = "mobile_on";
const char* msgMB_CalibrationDone = "calibration_done";
const char* msgMB_PositionStart = "p ";
const char* msgMB_DirectionStart = "d ";
const char* msgMB_PositionFormat = "p %d %d";
const char* msgMB_DirectionFormat = "d %d";
const int msgPositionMultiplier = 10000;

const char* msgBM_Calibrate = "calibrate";
const char* msgBM_LogOff = "log_off";
const char* msgBM_LogPosition = "log_position";
const char* msgBM_LogDirection = "log_direction";

// tx 8 rx 7
const int startButtonPin = 22;
const int motorXPin = 7;
const int motorYPin = 6;
const int laserPin = 24;
const int buzzerPin = 5;
const int lightPin = 3;
const int stepperIN1 = 44;
const int stepperIN2 = 46;
const int stepperIN3 = 48;
const int stepperIN4 = 50;

const int dirInvalid = -1;
const int dirL  = 0;
const int dirTL = 1;
const int dirT  = 2;
const int dirTR = 3;
const int dirR  = 4;
const int dirBR = 5;
const int dirB  = 6;
const int dirBL = 7;

const int cornerTL = 0;
const int cornerTR = 1;
const int cornerBR = 2;
const int cornerBL = 3;

const int lightGoPosition = 0;
const int lightSadPosition = -510;
const int lightHappyPosition = -1019;

const int cornersToDir[4][4] = {
  { -1, dirR, dirBR, dirB },
  { dirL, -1, dirB, dirBL },
  { dirTL, dirT, -1, dirL },
  { dirT, dirTR, dirR, -1 }
};

const float cornerToPositionOffset = 0.25f;
const int cornerToPosition[4][2] = {
  { msgPositionMultiplier * cornerToPositionOffset, msgPositionMultiplier * cornerToPositionOffset },
  { msgPositionMultiplier * (1.f - cornerToPositionOffset), msgPositionMultiplier * cornerToPositionOffset },
  { msgPositionMultiplier * (1.f - cornerToPositionOffset), msgPositionMultiplier * (1.f - cornerToPositionOffset) },
  { msgPositionMultiplier * cornerToPositionOffset, msgPositionMultiplier * (1.f - cornerToPositionOffset) }
};
// 1230, 1420
float lastServoPositions[4][2] = {
//  { 1281, 1444 },
//  { 1093, 1454 },
//  { 1080, 1380 },
//  { 1267, 1367 },
  { 1380, 1460 },
  { 1200, 1460 },
  { 1200, 1400 },
  { 1380, 1400 },
};

const long maxServoError = 1000000;
const int minTimeBelowServoMaxError = 2000;
const int servoSmoothFactor = 10;
const int servoSmoothTime = 25;
const float servoWidth = 183;
const float servoHeight = 68;
const float servoKpAtten = 1.0f;
const float servoKpx = servoKpAtten * servoWidth / float(msgPositionMultiplier);
const float servoKix = 0.0f / 1000.f;
const float servoKdx = 0.f;
const float servoKpy = servoKpAtten * servoHeight / float(msgPositionMultiplier);
const float servoKiy = 0.0f / 1000.f;
const float servoKdy = 0.f;

const int servoMinVal = 1000;
const int servoMaxVal = 2000;

Servo servoX;
Servo servoY;

// Define number of steps per rotation:

AccelStepper stepper(FULLSTEP, stepperIN1, stepperIN3, stepperIN2, stepperIN4);

int corner = 0;
//int corners[4][2] = {{78, 91}, {56, 91}, {78, 78}, {56, 78}};
//int center[2] = {(75 + 53) / 2, (88 + 75) / 2};
// int center[2] = {85, 67};
int center[2] = { 1230, 1420 };
// int center[2] = { 1430, 1500 };
int corners[100];
int numCorners = 0;
int numCorrectDirections = 0;
float lastServoX;
float lastServoY;
char btBuffer[128];
int btBufferLen = 0;

void myTone(int pin, int freq, int duration) {
  unsigned long startTime = millis();
  unsigned long halfPeriod = 500000L / freq;
  while (millis() - startTime < duration)
  {
    digitalWrite(pin, HIGH);
    delayMicroseconds(halfPeriod);
    digitalWrite(pin, LOW);
    delayMicroseconds(halfPeriod);
  }
}

void getCurrentCamPosition(int& sx, int& sy) {
  waitForMessageStartingWith(msgMB_PositionStart);
  sscanf(btBuffer, msgMB_PositionFormat, &sx, &sy);
}

void goToCamCorner(int x, int y) {
  int sx, sy;
  getCurrentCamPosition(sx, sy);
  for (int i = 0; i < 5; i++) {
    getCurrentCamPosition(sx, sy);
    Serial.print("Detected at ");
    Serial.print(sx);
    Serial.print(" , ");
    Serial.println(sy);
    
  }
}

void playStartGameSound() {
  t.tone(buzzerPin, 1100, 500);
  delay(500);
  t.tone(buzzerPin, 1100, 500);
  delay(500);
  t.tone(buzzerPin, 1100, 500);
  delay(500);
//  myTone(buzzerPin, 1200, 500);
//  delay(500);
//  myTone(buzzerPin, 1200, 500);
//  delay(500);
//  myTone(buzzerPin, 1200, 500);
//  delay(500);
}

void playGoGameSound() {
  t.tone(buzzerPin, 1400, 750);
  delay(500);
//  myTone(buzzerPin, 1600, 750);
//  delay(500);
}

void playCorrectSound() {
  t.tone(buzzerPin, 1200, 200);
  // delay(200);
  t.tone(buzzerPin, 1400, 400);
  delay(500);
//  myTone(buzzerPin, 1200, 200);
//  myTone(buzzerPin, 1600, 400);
//  delay(500);
}

void playGameoverSound() {
  t.tone(buzzerPin, 1500, 300);
  // delay(300);
  t.tone(buzzerPin, 1300, 300);
  // delay(300);
  t.tone(buzzerPin, 1100, 300);
  delay(500);
//  myTone(buzzerPin, 1400, 300);
//  myTone(buzzerPin, 1200, 300);
//  myTone(buzzerPin, 1000, 300);
//  delay(500);
}

void moveServosSlowAbs(float x, float y) {
  for (int i = 1; i <= servoSmoothFactor; i++) {
    servoX.writeMicroseconds(lastServoX + float(i) * (x - lastServoX) / servoSmoothFactor);
    servoY.writeMicroseconds(lastServoY + float(i) * (y - lastServoY) / servoSmoothFactor);
    delay(servoSmoothTime);
  }
}

void moveServosNowAbs(float x, float y) {
  x = min(max(servoMinVal, x), servoMaxVal);
  y = min(max(servoMinVal, y), servoMaxVal);
  moveServosSlowAbs(x, y);
  lastServoX = x;
  lastServoY = y;
}

void moveServosNowRel(float x, float y) {
  moveServosNowAbs(lastServoX + x, lastServoY + y);
}

bool waitForMessageStartingWithTimed(const char *msg, int timeoutMillis) {
  int mlen = strlen(msg);
  int ch;
  long now = millis();
  do {
    while ((ch = bt.read()) != '\n') {
      if (ch != -1)
        btBuffer[btBufferLen++] = ch;
      if (timeoutMillis <= millis() - now)
        return false;
    }
    btBuffer[btBufferLen] = 0;
    btBufferLen = 0;
    Serial.print("Received from mobile: '");
    Serial.print(btBuffer);
    Serial.println("'");
  } while (memcmp(btBuffer, msg, mlen) != 0);
  return true;
}

void waitForMessageStartingWith(const char *msg) {
  int mlen = strlen(msg);
  int ch;
  do {
    while ((ch = bt.read()) != '\n') {
      if (ch != -1)
        btBuffer[btBufferLen++] = ch;
    }
    btBuffer[btBufferLen] = 0;
    btBufferLen = 0;
    Serial.print("Received from mobile: '");
    Serial.print(btBuffer);
    Serial.println("'");
  } while (memcmp(btBuffer, msg, mlen) != 0);
}

void addRandomCorner()
{
  if (numCorners == 0) {
    numCorners = 1;
    corners[0] = random(4);
  } else {
    int corner = random(3);
    if (corner >= corners[numCorners - 1])
      corner++;
    corners[numCorners++] = corner;
  }
}

void validateDirection(int dir)
{
  if (numCorrectDirections < 0 || numCorrectDirections == 0 && dir == dirInvalid)
    return;
  if (numCorrectDirections + 1 >= numCorners) {
    numCorrectDirections = -1;
    return;
  }
  int c0 = corners[numCorrectDirections];
  int c1 = corners[numCorrectDirections + 1];
  int correctDir = cornersToDir[c0][c1];
  Serial.println("Inside dir validator");
  Serial.println(c0);
  Serial.println(c1);
  Serial.println(correctDir);
  Serial.println(dir);
  if (dir == correctDir) {
    numCorrectDirections++;
  } else {
    numCorrectDirections = -1;
  }
}

void rotateServosToPosition(int x, int y) {
  long error = 0;
  long timeBelowError = 0;
  long lastTime = millis(), nowTime, dt;
  int prevErrorX = 0;
  int prevErrorY = 0;
  long intErrorX = 0;
  long intErrorY = 0;

  Serial.print("Moving at ");
  Serial.print(x);
  Serial.print(" , ");
  Serial.println(y);
  do {
    waitForMessageStartingWith(msgMB_PositionStart);
    nowTime = millis();
    dt = nowTime - lastTime;
    lastTime = nowTime;
    int sx, sy;
    sscanf(btBuffer, msgMB_PositionFormat, &sx, &sy);
    Serial.print("Detected at ");
    Serial.print(sx);
    Serial.print(" , ");
    Serial.println(sy);
    if (sx == -1 || sy == -1) {
      moveServosNowAbs(center[0], center[1]);
      intErrorX = 0;
      intErrorY = 0;
    } else {
      int ex = sx - x;
      int ey = sy - y;
      intErrorX += ex * dt;
      intErrorY += ey * dt;
      // TODO: derivative
      float outX = servoKpx * float(ex) + servoKix * float(intErrorX);
      float outY = servoKpy * float(ey) + servoKiy * float(intErrorY);
      Serial.print("Errors ");
      Serial.print(ex);
      Serial.print(" , ");
      Serial.println(ey);
      Serial.print("Moving with ");
      Serial.print(outX);
      Serial.print(" , ");
      Serial.println(outY);
      moveServosNowRel(outX, outY);
      Serial.print("Currently at ");
      Serial.print(lastServoX);
      Serial.print(" , ");
      Serial.println(lastServoY);
  
      prevErrorX = ex;
      prevErrorY = ey;
  
      error = ex * ex + ey * ey;
      if (error < maxServoError)
        timeBelowError += dt;
      else timeBelowError = 0;
    }

  } while (timeBelowError <= minTimeBelowServoMaxError);
}

void calibrateLastServoPos()
{
  bt.println(msgBM_LogPosition);

  rotateServosToPosition(
    cornerToPosition[cornerTL][0],
    cornerToPosition[cornerTL][1]);

  float left = lastServoX;
  float top = lastServoY;
  
  rotateServosToPosition(
    cornerToPosition[cornerBR][0],
    cornerToPosition[cornerBR][1]);

  float right = lastServoX;
  float bottom = lastServoY;

  Serial.println("Calibrated:");
  Serial.println(left);
  Serial.println(top);
  Serial.println(right);
  Serial.println(bottom);
  
  bt.println(msgBM_LogOff);

  lastServoPositions[cornerTL][0] = left;
  lastServoPositions[cornerTL][1] = top;
  lastServoPositions[cornerTR][0] = right;
  lastServoPositions[cornerTR][1] = top;
  lastServoPositions[cornerBL][0] = left;
  lastServoPositions[cornerBL][1] = bottom;
  lastServoPositions[cornerBR][0] = right;
  lastServoPositions[cornerBR][1] = bottom;
}

bool validateDirections() {

  while (bt.available())
    bt.read();

  numCorrectDirections = 0;
  while (numCorrectDirections >= 0 && numCorrectDirections < numCorners - 1) {
    if (!waitForMessageStartingWithTimed(msgMB_DirectionStart, 4000)) {
      return false;
    }
    int dir;
    sscanf(btBuffer, msgMB_DirectionFormat, &dir);
    validateDirection(dir);
    Serial.print("Received direction ");
    Serial.println(dir);
    Serial.print("Correct dirs = ");
    Serial.println(numCorrectDirections);
  }
  return numCorrectDirections == numCorners - 1;
}

void rotateServosToCorner(int corner) {
  moveServosNowAbs(
    lastServoPositions[corner][0],
    lastServoPositions[corner][1]);
  delay(300);
//  rotateServosToPosition(
//      cornerToPosition[corner][0],
//      cornerToPosition[corner][1]);
  lastServoPositions[corner][0] = lastServoX;
  lastServoPositions[corner][1] = lastServoY;
}

bool playRound()
{
  Serial.println("Starting round");
  // bt.println(msgBM_LogPosition);
  bt.println(msgBM_LogOff);
  rotateServosToCorner(corners[0]);
  digitalWrite(laserPin, HIGH);
  for (int i = 1; i < numCorners; i++) {
    Serial.println("Moving to next corner");
    rotateServosToCorner(corners[i]);
  }
  Serial.println("Finished moving to corners");
  digitalWrite(laserPin, LOW);
  bt.println(msgBM_LogDirection);
  Serial.println("Waiting for player to copy");
  bool ret = validateDirections();
  bt.println(msgBM_LogOff);
  Serial.print("Finished round with result ");
  Serial.println(ret);
  return ret;
}

void setup() {
  randomSeed(analogRead(0));
  Serial.begin(9600);
  bt.begin(9600);
  Serial.println("Starting...");

  pinMode(startButtonPin, INPUT_PULLUP);
  pinMode(laserPin, OUTPUT);
  pinMode(buzzerPin, OUTPUT);
  pinMode(lightPin, OUTPUT);
  
  stepper.setMaxSpeed(350);
  stepper.setAcceleration(100);
  // stepper.moveTo(lightGoPosition);
  
  servoX.attach(motorXPin);
  servoY.attach(motorYPin);
  digitalWrite(laserPin, HIGH);
  digitalWrite(buzzerPin, HIGH);
  digitalWrite(lightPin, LOW);
  moveServosNowAbs(center[0], center[1]);
  Serial.println("Servo centered. Waiting for button press...");  
  // waitForMessage(msgMB_MobileOn);
  while (digitalRead(startButtonPin) != 0);
  Serial.println("Sending calibration message");
  bt.println("garbage");
  bt.println(msgBM_Calibrate);
  waitForMessageStartingWith(msgMB_CalibrationDone);
  Serial.println("Phone calibration done. Calibrating servos...");
  calibrateLastServoPos();
  Serial.println("Servos calibration done. Starting game!");
}

void loop() {
  
  Serial.println("Started menu");
  numCorners = 5;
  corners[0] = cornerTR;
  corners[1] = cornerBR;
  corners[2] = cornerBL;
  corners[3] = cornerTL;
  corners[4] = cornerTR;
  while (playRound() == false) {
    Serial.println("Round failed");
  }

  playStartGameSound();
  digitalWrite(lightPin, HIGH);
  playGoGameSound();
  digitalWrite(lightPin, LOW);
  stepper.moveTo(lightHappyPosition);
  stepper.runToPosition();

  bool wonRound;
  numCorners = 0;
  addRandomCorner();
  do {
    addRandomCorner();
    wonRound = playRound();
    if (wonRound) {
      digitalWrite(lightPin, HIGH);
      playCorrectSound();
      digitalWrite(lightPin, LOW);
    }
  } while (wonRound);
  playGameoverSound();
  stepper.moveTo(lightSadPosition);
  stepper.runToPosition();
  digitalWrite(lightPin, HIGH);
  playGameoverSound();
  digitalWrite(lightPin, LOW);
  stepper.moveTo(lightGoPosition);
  stepper.runToPosition();
}
