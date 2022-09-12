#include <util/atomic.h>

#define encAR 2
#define encBR 5
#define inAR 11
#define inBR 10
#define PWMR 9

#define encAL 3
#define encBL 4
#define inAL 7
#define inBL 8
#define PWML 6

#define trigR A0
#define trigL A4
#define trigF A1

#define echoR 12
#define echoL A3
#define echoF A2

volatile int posR = 0 , posL = 0 ;
int target;

void setup() {
  pinMode(encAR,INPUT);
  pinMode(encBR,INPUT);
  pinMode(inAR,OUTPUT);
  pinMode(inBR,OUTPUT);
  pinMode(PWMR,OUTPUT);
  attachInterrupt(digitalPinToInterrupt(encAR),readEncoderR,RISING);

  pinMode(encAL,INPUT);
  pinMode(encBL,INPUT);
  pinMode(inAL,OUTPUT);
  pinMode(inBL,OUTPUT);
  pinMode(PWML,OUTPUT);
  attachInterrupt(digitalPinToInterrupt(encAL),readEncoderL,RISING);
  moveDist(30,100);
  moveDist(30,100);
}

void loop() {

}

void moveDist(int dist , int Speed){
  int ppr = 385;
  float tireCirc = 2 * 3.5 * 3.14;
  int currPosR ,currPosL;
  ATOMIC_BLOCK(ATOMIC_RESTORESTATE){
    currPosR = posR;
    currPosL = posL;
    }
  int targetR = -((float(dist)/tireCirc ) * float(ppr)) + currPosR ;
  int targetL = (float(dist)/tireCirc ) * float(ppr)+ currPosL ;
  int eL = targetL - currPosL , eR =  currPosR - targetR;
  int dirR = 1 , dirL = 1;
  if(eR < 0) dirR = -1;
  if(eL < 0) dirL = -1;
  int mn = 0.02*abs(eR);
  float leftDamp = 1.0 , rightDamp = 1.0 ,increment = 0.5;
  int prevDistR = readUltrasonicR() , prevDistL = readUltrasonicL();
  while(abs(eR)> mn){
    if(readUltrasonicR() <= 10){
      if(readUltrasonicR() < prevDistR){
          rightDamp += increment;
          leftDamp -= increment;
        }
      else if(readUltrasonicR() > prevDistR){
          rightDamp -= increment;
          leftDamp += increment;
        }
      }
    if(readUltrasonicL() <= 10){
      if(readUltrasonicL() < prevDistL){
          rightDamp -= increment;
          leftDamp += increment;
        }
      else if(readUltrasonicL() > prevDistL){
          rightDamp += increment;
          leftDamp -= increment;
          
        }
      }
    int errorR = 0.45*eR, errorL = 0.45*eR;
    if(abs(eL) < abs(eR)){
          errorL =  errorR - 0.1 * errorR;
          errorR =  errorR + 0.1 * errorR;
        }
    else if(abs(eL) > abs(eR)){
        errorL =  errorR + 0.1 * errorR;
        errorR =  errorR - 0.1 * errorR;
        }
    int pwmR = rightDamp * errorR * Speed ;
    int pwmL = leftDamp * errorL * Speed ;
    if(pwmR > 150) pwmR = 150;
    else if(pwmR < 40) pwmR = 40;
    if(pwmL > 150) pwmL = 150;
    else if(pwmL < 40) pwmL = 40;
    setMotor(dirR , pwmR , inAR , inBR , PWMR);
    setMotor(dirL , pwmL , inAL , inBL , PWML);
    ATOMIC_BLOCK(ATOMIC_RESTORESTATE){
      currPosR = posR;
      currPosL = posL;
    }
    eL = targetL - currPosL , eR =  currPosR - targetR;
    prevDistR = readUltrasonicR();
    prevDistL = readUltrasonicL();
    if(eR < 0) break;
    if(eL < 0) break;
  }
  setMotor(0,0,inAR,inBR,PWMR);
  setMotor(0,0,inAL,inBL,PWML);
  ATOMIC_BLOCK(ATOMIC_RESTORESTATE){
      posR = 0;
      posL = 0;
    }
}
  
void setMotor(int dir , int pwmValue , int in1 , int in2 , int pwm ){
  analogWrite(pwm , pwmValue);
  if(dir == 1){
    digitalWrite(in1,HIGH);
    digitalWrite(in2,LOW);
    }
  else if(dir == -1){
    digitalWrite(in2,HIGH);
    digitalWrite(in1,LOW);
    }
  else{
    digitalWrite(in1,LOW);
    digitalWrite(in2,LOW);
    }
  }
void readEncoderR(){
  if(digitalRead(encBR)) posR++;
  else posR--;
  }

void readEncoderL(){
  if(digitalRead(encBL)) posL++;
  else posL--;
  }

int readUltrasonicR(){
  digitalWrite(trigR, LOW);
  delayMicroseconds(2);
  digitalWrite(trigR, HIGH);
  delayMicroseconds(10);
  digitalWrite(trigR, LOW);
  int distant = pulseIn(echoR, HIGH);
  int disC = distant * 0.5 * 0.034;
  return disC;
  }

int readUltrasonicL(){
  digitalWrite(trigL, LOW);
  delayMicroseconds(2);
  digitalWrite(trigL, HIGH);
  delayMicroseconds(10);
  digitalWrite(trigL, LOW);
  int distant = pulseIn(echoL, HIGH);
  int disC = distant * 0.5 * 0.034;
  return disC;
  }

int readUltrasonicF(){
  digitalWrite(trigF, LOW);
  delayMicroseconds(2);
  digitalWrite(trigF, HIGH);
  delayMicroseconds(10);
  digitalWrite(trigF, LOW);
  int distant = pulseIn(echoF, HIGH);
  int disC = distant * 0.5 * 0.034;
  return disC;
  }
