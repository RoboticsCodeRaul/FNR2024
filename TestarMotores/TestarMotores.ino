#include <AFMotor.h>

AF_DCMotor motorDir(1);
AF_DCMotor motorEsq(3);


void setup() {
  // put your setup code here, to run once:
  Serial.begin(9600);

  motorDir.run(RELEASE);
  motorEsq.run(RELEASE);
}

void loop() {
  // put your main code here, to run repeatedly:
  moveF(255, 255);
  // delay(5000);
  // moveB(255, 255);
  // delay(5000);
  // moveR(255, 255);
  // delay(5000);
  // moveL(255, 255);
  // delay(5000);
  // moveStop();
}

void moveF(int velA, int velB) {  // -> para o carro andar para a frente

  motorEsq.setSpeed(velA);
  motorDir.setSpeed(velB);

  motorEsq.run(FORWARD);
  motorDir.run(FORWARD);

}

void moveB(int velA, int velB) {  // -> para o carro andar para tras

  motorEsq.setSpeed(velA);
  motorDir.setSpeed(velB);

  motorEsq.run(BACKWARD);
  motorDir.run(BACKWARD);

}

void moveL(int velA, int velB) {  // -> para o carro andar para a esquerda

  motorEsq.setSpeed(velA);
  motorDir.setSpeed(velB);

  motorEsq.run(BACKWARD);
  motorDir.run(FORWARD);
}

void moveR(int velA, int velB) {  // -> para o carro andar para a direita

  motorEsq.setSpeed(velA);
  motorDir.setSpeed(velB);

  motorEsq.run(FORWARD);
  motorDir.run(BACKWARD);

}

void moveStop() {  // -> para o carro parar

  motorEsq.setSpeed(0);
  motorDir.setSpeed(0);

  motorEsq.run(RELEASE);
  motorDir.run(RELEASE);

}