#include <QTRSensors.h>
#include <AFMotor.h>
#include <SoftwareSerial.h>
#include <Wire.h>
#include <Adafruit_TCS34725softi2c.h>

//Bluetooth
SoftwareSerial mySerial(50, 51);  // RX, TX

//Sensores RGB
const int sdaEsq = 20, sclEsq = 21, sdaDir = 53, sclDir = 52;
Adafruit_TCS34725softi2c rgbDir = Adafruit_TCS34725softi2c(TCS34725_INTEGRATIONTIME_50MS, TCS34725_GAIN_4X, sdaDir, sclDir);
Adafruit_TCS34725softi2c rgbEsq = Adafruit_TCS34725softi2c(TCS34725_INTEGRATIONTIME_50MS, TCS34725_GAIN_4X, sdaEsq, sclEsq);

//Led RGB
const int redPin = A13, greenPin = A9, bluePin = A8;

//Buzzer
const int buzzerPin = 37;

//Motores
AF_DCMotor MEsq(4);
AF_DCMotor MDir(3);

//Sensor Segue Linha
QTRSensors qtr;
const uint8_t SensorCount = 8;
uint16_t sensorValues[SensorCount];

int lastError = 0;
boolean onoff = 0;
int val, cnt = 0, v[3];

const uint16_t threshold = 500;  // adjustable - can take values between 0 and 1000

//the speed can be between 0 and 255 - 0 is LOW and 255 is HIGH. At a high value,
//you may risk burning the motors, because the voltage supplied by the PWM control
//is higher than 6V.
const int maxspeeda = 255;
const int maxspeedb = 255;
const int basespeeda = 240;
const int basespeedb = 240;
/*If your robot can't take tight curves, you can set up the robot
  to revolve around the base (and not around one of the wheels) by
  setting the minspeed to a negative value (-100), so the motors will go 
  forward and backward. Doing this the motors can wear out faster. 
  If you don't want to do this, set the minspeed to 0, so the motors 
  will only go forward.
*/
//const int minspeeda = 0;
//const int minspeedb = 0;
const int minspeeda = -100;
const int minspeedb = -100;

float Kp = 0;
float Ki = 0;
float Kd = 0;
uint8_t multiP = 1;
uint8_t multiI = 1;
uint8_t multiD = 1;
uint8_t Kpfinal;
uint8_t Kifinal;
uint8_t Kdfinal;
int P;
int I;
int D;
float Pvalue;
float Ivalue;
float Dvalue;

void setup() {
  Serial.begin(9600);
  mySerial.begin(9600);
  qtr.setTypeRC();
  qtr.setSensorPins((const uint8_t[]){ 31, 28, 29, 26, 27, 24, 25, 22 }, SensorCount);
  qtr.setEmitterPin(23);

  while (!rgbDir.begin()) {
    Serial.println("O sensor RGB direito não foi encontrado ... verifica as ligações");
    delay(1000);
  }

  while (!rgbEsq.begin()) {
    Serial.println("O sensor RGB esquerdo não foi encontrado ... verifica as ligações");
    delay(1000);
  }

  delay(500);
  pinMode(LED_BUILTIN, OUTPUT);
  calibration();
  moveStop();
}

void calibration() {
  digitalWrite(LED_BUILTIN, HIGH);
  // 2.5 ms RC read timeout (default) * 10 reads per calibrate() call
  // = ~25 ms per calibrate() call.
  // Call calibrate() 400 times to make calibration take about 10 seconds.
  for (uint16_t i = 0; i < 400; i++) {
    qtr.calibrate();
  }
  // print the calibration minimum values measured when emitters were on
  for (uint8_t i = 0; i < SensorCount; i++) {
    Serial.print(qtr.calibrationOn.minimum[i]);
    Serial.print(' ');
  }
  Serial.println();
  // print the calibration maximum values measured when emitters were on
  for (uint8_t i = 0; i < SensorCount; i++) {
    Serial.print(qtr.calibrationOn.maximum[i]);
    Serial.print(' ');
  }
  //Serial.println();
  digitalWrite(LED_BUILTIN, LOW);
}

void loop() {
  if (mySerial.available()) {
    while (mySerial.available() == 0)
      ;
    valuesread();
    processing();
  }
  if (onoff == 1) {
    robot_control();
  }
  if (onoff == 0) {
    moveStop();
  }
}

//This void delimits each instruction.
//The Arduino knows that for each instruction it will receive 2 bytes.
void valuesread() {
  val = mySerial.read();
  cnt++;
  v[cnt] = val;
  if (cnt == 2)
    cnt = 0;
}

//In this void the the 2 read values are assigned.
void processing() {
  int a = v[1];
  if (a == 1) {
    Kp = v[2];
  }
  if (a == 2) {
    multiP = v[2];
  }
  if (a == 3) {
    Ki = v[2];
  }
  if (a == 4) {
    multiI = v[2];
  }
  if (a == 5) {
    Kd = v[2];
  }
  if (a == 6) {
    multiD = v[2];
  }
  if (a == 7) {
    onoff = v[2];
  }
  // Serial.print(Kp);
  // Serial.print("\t");
  // Serial.print(Ki);
  // Serial.print("\t");
  // Serial.println(Kd);
  //Serial.print("\t");
  // Serial.println(onoff);
}

//Make sure that this values are assigned correctly
void moveF(int velA, int velB) {  // -> para o carro andar para a frente
  //Serial.print("ola");
  MEsq.setSpeed(velA);
  MDir.setSpeed(velB);
  MEsq.run(FORWARD);
  MDir.run(FORWARD);
}

void moveL(int velA, int velB) {  // -> para o carro andar para a frente
  MEsq.setSpeed(velA);
  MDir.setSpeed(velB);
  MEsq.run(BACKWARD);
  MDir.run(FORWARD);
}

void moveR(int velA, int velB) {  // -> para o carro andar para a frente
  MEsq.setSpeed(velA);
  MDir.setSpeed(velB);
  MEsq.run(FORWARD);
  MDir.run(BACKWARD);
}

void moveStop() {  // -> para o carro andar para a frente
  MEsq.run(RELEASE);
  MDir.run(RELEASE);
}


void robot_control() {

  checkColor();
  uint16_t position = qtr.readLineBlack(sensorValues);
  int error = 3500 - position;

  int cnt = 0;
  float sum = 0;
  for (int i = 0; i < 8; i++) {
    if (sensorValues[i] >= threshold) {
      cnt++;
      sum = sum + i;
    }
  }


  //I made a case where the robot can cross the line (when the curve is too tight) and the position can be 3500
  //even though it is not on the center of the line. If you don't want your motors to rotate in 2 directions
  //comment the right_brake(100,100) / left_brake(100, 100) and uncomment the forward_brake(0,100) / forward_brake(0,100)


  if (cnt >= 3) {
    int motorspeeda = 0;
    int motorspeedb = 0;
    int val = sum / cnt;
    if (val < 3.5) {
      //turn right
      moveR(100, 100);
      //forward_brake(0,100);
    }
    if (val > 3.5) {
      //turn left
      moveL(100, 100);
      //forward_brake(100,0);
    }
    if (val == 3.5) {
      cnt = cnt / 2;
      uint16_t mini = 1000;
      uint8_t minpos = 0;
      for (int i = 4 - cnt; i <= 3 + cnt; i++) {
        if (mini > sensorValues[i]) {
          mini = sensorValues[i];
          minpos = i;
        }
      }
      if (minpos < 3.5) {
        //turn right
        moveR(100, 100);
        //forward_brake(0,100);
      }
      if (minpos > 3.5) {
        //turn left
        moveL(100, 100);
        //forward_brake(100,0);
      }
    }
  } else {
    PID(error);
  }
}

void PID(int error) {

  int P = error;
  int I = I + error;
  int D = error - lastError;
  lastError = error;
  Pvalue = (Kp / pow(10, multiP)) * P;
  Ivalue = (Ki / pow(10, multiI)) * I;
  Dvalue = (Kd / pow(10, multiD)) * D;

  float motorspeed = Pvalue + Ivalue + Dvalue;

  int motorspeeda = basespeeda + motorspeed;
  int motorspeedb = basespeedb - motorspeed;

  if (motorspeeda > maxspeeda) {
    motorspeeda = maxspeeda;
  }
  if (motorspeedb > maxspeedb) {
    motorspeedb = maxspeedb;
  }
  if (motorspeeda < minspeeda) {
    motorspeeda = minspeeda;
  }
  if (motorspeedb < minspeedb) {
    motorspeedb = minspeedb;
  }
  Serial.print(motorspeeda);
  Serial.print(" ");
  Serial.println(motorspeedb);
  speedcontrol(motorspeeda, motorspeedb);
}

void speedcontrol(int mota, int motb) {
  if (mota >= 0 && motb >= 0) {
    moveF(mota, motb);
  }
  if (mota < 0 && motb >= 0) {
    //dreapta
    mota = 0 - mota;
    moveL(mota, motb);
  }
  if (mota >= 0 && motb < 0) {
    //stanga
    motb = 0 - motb;
    moveR(mota, motb);
  }
}

void turnRed() {
  analogWrite(redPin, 255);
  analogWrite(greenPin, 0);
  analogWrite(bluePin, 0);
}

void turnGreen() {
  analogWrite(redPin, 0);
  analogWrite(greenPin, 255);
  analogWrite(bluePin, 0);
}

void turnBlue() {
  analogWrite(redPin, 0);
  analogWrite(greenPin, 0);
  analogWrite(bluePin, 255);
}

void turnPink() {
  analogWrite(redPin, 255);
  analogWrite(greenPin, 0);
  analogWrite(bluePin, 255);
}

void ledOff() {
  analogWrite(redPin, 0);
  analogWrite(greenPin, 0);
  analogWrite(bluePin, 0);
}

void checkColor() {

  uint16_t clearDir, redDir, greenDir, blueDir;
  uint16_t clearEsq, redEsq, greenEsq, blueEsq;
  rgbDir.setInterrupt(false);  // turn on LED
  rgbEsq.setInterrupt(false);  // turn on LED
  delay(60);                   // takes 50ms to read
  rgbDir.getRawData(&redDir, &greenDir, &blueDir, &clearDir);
  rgbEsq.getRawData(&redEsq, &greenEsq, &blueEsq, &clearEsq);
  rgbDir.setInterrupt(true);  // turn off LED
  rgbEsq.setInterrupt(true);  // turn off LED

  uint32_t sumDir = clearDir;
  float dirR, dirG, dirB;
  dirR = redDir;
  dirR /= sumDir;
  dirG = greenDir;
  dirG /= sumDir;
  dirB = blueDir;
  dirB /= sumDir;
  dirR *= 256;
  dirG *= 256;
  dirB *= 256;

  uint32_t sumEsq = clearEsq;
  float esqR, esqG, esqB;
  esqR = redEsq;
  esqR /= sumEsq;
  esqG = greenEsq;
  esqG /= sumEsq;
  esqB = blueEsq;
  esqB /= sumEsq;
  esqR *= 256;
  esqG *= 256;
  esqB *= 256;

  Serial.print("RGB Direito R: ");
  Serial.print((int)dirR);
  Serial.print(" G: ");
  Serial.print((int)dirG);
  Serial.print(" B: ");
  Serial.print((int)dirB);
  Serial.print("\t");

  Serial.print("RGB Esquerdo R: ");
  Serial.print((int)esqR);
  Serial.print(" G: ");
  Serial.print((int)esqG);
  Serial.print(" B: ");
  Serial.println((int)esqB);


  if ((dirR > 120 && dirG < 100 && dirB < 100) || (esqR > 120 && esqG < 100 && esqB < 100)) {
    analogWrite(redPin, 255);
    analogWrite(greenPin, 0);
    analogWrite(bluePin, 0);
    buzzerOn();

  } else if ((dirR < 100 && dirG > 110 && dirB < 100) || (esqR < 100 && esqG > 110 && esqB < 100)) {
    analogWrite(redPin, 0);
    analogWrite(greenPin, 255);
    analogWrite(bluePin, 0);
    buzzerOn();
  } else if ((dirR < 100 && dirG < 100 && dirB > 130) || (esqR < 100 && esqG < 100 && esqB > 130)) {
    analogWrite(redPin, 0);
    analogWrite(greenPin, 0);
    analogWrite(bluePin, 255);
    buzzerOn();
  } else {
    ledOff();
  }
}

void buzzerOn() {
  moveStop();
  digitalWrite(buzzerPin, HIGH);  //Set PIN 8 feet as HIGH = 5 v
  delay(3000);
  digitalWrite(buzzerPin, LOW);
  ledOff();
}
