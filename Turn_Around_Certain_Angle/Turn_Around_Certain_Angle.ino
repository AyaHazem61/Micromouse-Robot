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

volatile int posR = 0 , posL = 0 ;
long prevT = 0;
float prevER = 0 ,prevEL = 0  ;
float integralER = 0 ,integralEL = 0  ;

void setup() {
  Serial.begin(9600);
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
  
  turnAround(180,100);
  
}

void loop() {
  
}


void turnAround(int degree , int Speed){
  int fullTurn = 920; int target;
  switch(degree){
    case 90 :
      target = 0.25 * fullTurn;
      break;
    case -90 :
      target = -0.25 * fullTurn;
      break;
    case 180 :
      target = 0.52* fullTurn;
      break;
    case -180 :
      target = -0.52 * fullTurn;
      break;
    case 360 :
      target = fullTurn;
      break;
    }
  
  int currPosR , currPosL;
  ATOMIC_BLOCK(ATOMIC_RESTORESTATE){
    currPosR = posR;
    currPosL = posL;
    }
  int targetR = target + currPosR , targetL = target + currPosL;
  int eL = targetL - currPosL , eR =currPosR - targetR;
  int dirR = 1 , dirL = 1;
  if(eR < 0){
     dirR = -1;
  }
  if(eL < 0){
     dirL = -1;
  }
  int mn = 0.05 * abs(eR);
 
  while(abs(eR) > mn){
    int pwmR = float(0.45*abs(eR))*Speed, pwmL;
    if(pwmR > Speed){
       pwmR = Speed;
      }
      dirR = 1 , dirL = 1;
      if(eR < 0){
         dirR = -1;
      }
       if(eL < 0){
         dirL = -1;
      }
      if(abs(eL) < abs(eR)){
          pwmL =  pwmR - 0.15 * pwmR;
          pwmR =  pwmR + 0.15 * pwmR;
        }
      else if(abs(eL) > abs(eR)){
        pwmL =  pwmR + 0.15 * pwmR;
        pwmR =  pwmR - 0.15 * pwmR;
        }
      setMotor(dirR,pwmR,inAR,inBR,PWMR);
      setMotor(dirL,pwmL,inAL,inBL,PWML);
      ATOMIC_BLOCK(ATOMIC_RESTORESTATE){
        currPosR = posR;
        currPosL = posL;
        }
      eL = targetL - currPosL , eR =currPosR - targetR;
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
