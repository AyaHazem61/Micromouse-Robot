#include <util/atomic.h>

#define trig0 A1
#define trig1 8
#define trig2 A3
#define trig3 A5

#define echo0 A0
#define echo1 9
#define echo2 A2
#define echo3 A4

#define encAR 2
#define encBR 5
#define inAR 10
#define inBR 11
#define PWMR 9

#define encAL 3
#define encBL 4
#define inAL 8
#define inBL 7
#define PWML 6

volatile int posR = 0 , posL = 0 ;
long prevT = 0;
float prevER = 0 ,prevEL = 0  ;
float integralER = 0 ,integralEL = 0  ;

int target;

const int cellLength = 30; 

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

  pinMode(trig0,OUTPUT);
  pinMode(echo0,INPUT);

  pinMode(trig1,OUTPUT);
  pinMode(echo1,INPUT);

  pinMode(trig2,OUTPUT);
  pinMode(echo2,INPUT);

  pinMode(trig3,OUTPUT);
  pinMode(echo3,INPUT);
}

void loop() { 
/* N >> N >> N >> N >> E >> E >> N >> N >> w */
  byte path[] = {1,1,1,1,4,4,1,1,8};
  byte currHeading = path[0];
  for(int i = 0 ; i < sizeof(path) ; i++){
    moveTo(path[i],currHeading,200);
    
    currHeading = path[i];
    }
  /*
    N to E
    turn 90  *for x milie soconds* or *until both ultrasonics reads walls*
    continue forward
  */
}
void moveDist(int dist , int Speed){
  int ppr = 385;
  float tireCirc = 2 * 3.25 * 3.14;
  target = (float(dist)/tireCirc ) * float(ppr);
  float kpL = 0.45 , kiL = 0.1, kdL = 0;
  float kpR = 0.45, kiR = 0.1, kdR = 0;
  long currT = micros();
  float deltaT = float(currT - prevT)/1.0e6;
  int currPosR ,currPosL;
  ATOMIC_BLOCK(ATOMIC_RESTORESTATE){
    currPosR = posR;
    currPosL = posL;
    }

  int eL = currPosL  -  target, eR = target + currPosR  ;
  int dedER = (eR- prevER) / deltaT , dedEL = (eL - prevEL) / deltaT ;
  integralER = integralER + eR*deltaT;
  integralEL = integralEL + eL*deltaT;
  
  float uR = kpR*eR + kiR * integralER  + kdR * dedER;
  float uL = kpL*eL + kiL * integralEL  + kdL * dedEL;
  while((uL > 50 || uL < - 50) || (uR > 50 || uR < - 50)){
    int pwmR = (fabs(uR)) * Speed , pwmL = (fabs(uL)) * Speed;
      if(pwmR > Speed){
        pwmR = Speed;
        }
      if(pwmL > Speed){
        pwmL = Speed;
        }
      int dirR = 1 , dirL = 1;
      if(uR < 0){
         dirR = -1;
      }
       if(uL < 0){
         dirL = -1;
      }
      
      setMotor(dirR,pwmR,inAR,inBR,PWMR);
      setMotor(dirL,pwmL,inAL,inBL,PWML);
      
      prevER = eR; prevEL = eL;
      prevT = currT ;
      currT = micros();
      deltaT = float(currT - prevT)/1.0e6;
      ATOMIC_BLOCK(ATOMIC_RESTORESTATE){
        currPosR = posR;
        currPosL = posL;
        }
    
      eL =   target - currPosL, eR =  target + currPosR   ;
      dedER = (eR- prevER) / deltaT , dedEL = (eL - prevEL) / deltaT ;
      integralER = integralER + eR*deltaT;
      integralEL = integralEL + eL*deltaT;
      
      uR = kpR*eR + kiR * integralER  + kdR * dedER;
      uL = kpL*eL + kiL * integralEL  + kdL * dedEL;
       }
      setMotor(0,0,inAR,inBR,PWMR);
      setMotor(0,0,inAL,inBL,PWML); 
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

void turnAround(int degree , int Speed){
  int ppr = 385;
  int targetL = degree * float(ppr) , targetR = degree * float(ppr);
  float kpL = 0.45 , kiL = 0.1, kdL = 0;
  float kpR = 0.45, kiR = 0.1, kdR = 0;
  long currT = micros();
  float deltaT = float(currT - prevT)/1.0e6;
  int currPosR ,currPosL;
  ATOMIC_BLOCK(ATOMIC_RESTORESTATE){
    currPosR = posR;
    currPosL = posL;
    }

  int eL = currPosL  -  targetL, eR = targetR - currPosR  ;
  int dedER = (eR- prevER) / deltaT , dedEL = (eL - prevEL) / deltaT ;
  integralER = integralER + eR*deltaT;
  integralEL = integralEL + eL*deltaT;
  
  float uR = kpR*eR + kiR * integralER  + kdR * dedER;
  float uL = kpL*eL + kiL * integralEL  + kdL * dedEL;
  while((uL > 50 || uL < - 50) || (uR > 50 || uR < - 50)){
    int pwmR = (fabs(uR)) * Speed , pwmL = (fabs(uL)) * Speed;
      if(pwmR > Speed){
        pwmR = Speed;
        }
      if(pwmL > Speed){
        pwmL = Speed;
        }
      int dirR = 1 , dirL = 1;
      if(uR < 0){
         dirR = -1;
      }
       if(uL < 0){
         dirL = -1;
      }
      
      setMotor(dirR,pwmR,inAR,inBR,PWMR);
      setMotor(dirL,pwmL,inAL,inBL,PWML);
      
      prevER = eR; prevEL = eL;
      prevT = currT ;
      currT = micros();
      deltaT = float(currT - prevT)/1.0e6;
      ATOMIC_BLOCK(ATOMIC_RESTORESTATE){
        currPosR = posR;
        currPosL = posL;
        }
    
      eL = currPosL  -  targetL, eR = targetR - currPosR;
      dedER = (eR- prevER) / deltaT , dedEL = (eL - prevEL) / deltaT ;
      integralER = integralER + eR*deltaT;
      integralEL = integralEL + eL*deltaT;
      
      uR = kpR*eR + kiR * integralER  + kdR * dedER;
      uL = kpL*eL + kiL * integralEL  + kdL * dedEL;
       }
      setMotor(0,0,inAR,inBR,PWMR);
      setMotor(0,0,inAL,inBL,PWML); 
  }
 
void moveTo(int heading,int currHeading , int Speed){
  switch (heading) {
    case 1 :
        switch (currHeading) {
          case 1 :
            moveDist(cellLength,Speed);
            break;
          case 2 :
            turnAround(-180,Speed);
            moveDist(cellLength,Speed);
            break;
          case 4 :
            turnAround(-90,Speed);
            moveDist(cellLength,Speed);
            break;
          case 8 :
            turnAround(90,Speed);
            moveDist(cellLength,Speed);
            break;
        break;}
    case 2 : /*N >> S*/
    switch (currHeading) {
          case 1 :
            turnAround(180,Speed);
            moveDist(cellLength,Speed);
            break;
          case 2 :
            moveDist(cellLength,Speed);
            break;
          case 4 :
            turnAround(90,Speed);
            moveDist(cellLength,Speed);
            break;
          case 8 :
            turnAround(-90,Speed);
            moveDist(cellLength,Speed);
            break;
        break;
        }
    case 4 :
        switch (currHeading) {
          case 1 :
            turnAround(-90,Speed);
            moveDist(cellLength,Speed);
            break;
          case 2 :
            turnAround(90,Speed);
            moveDist(cellLength,Speed);
            break;
          case 4 :
            moveDist(cellLength,Speed);
            break;
          case 8 :
            turnAround(180,Speed);
            moveDist(cellLength,Speed);
            break;
         break;}
    case 8 :
        switch (currHeading) {
          case 1 :
            turnAround(90,Speed);
            moveDist(cellLength,Speed);
            break;
          case 2 :
            turnAround(-90,Speed);
            moveDist(cellLength,Speed);
            break;
          case 4 :
            turnAround(180,Speed);
            moveDist(cellLength,Speed);
            break;
          case 8 :
            moveDist(cellLength,Speed);
            break;
      break;
    }
  }
}
