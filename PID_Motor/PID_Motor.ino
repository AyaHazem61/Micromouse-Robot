#include <util/atomic.h>

#define encA 3
#define encB 4
#define inA 8
#define inB 7
#define PWM 6

volatile int pos = 0;
long prevT = 0;
float prevE = 0;
float integralE = 0;
int target = 1200;

void setup() {
  Serial.begin(9600);
  pinMode(encA,INPUT);
  pinMode(encB,INPUT);
  pinMode(inA,OUTPUT);
  pinMode(inB,OUTPUT);
  pinMode(PWM,OUTPUT);
  pinMode(A5,OUTPUT);
  attachInterrupt(digitalPinToInterrupt(encA),readEncoder,RISING);


}

void loop() {
  //Serial.println(int((float(analogRead(A5))/1023)*2240.4));
  target = 500;
  float kp = .75 , ki = 0.25, kd = 0;
  long currT = micros();
  float deltaT = float(currT - prevT)/1.0e6;
  int currPos;
  ATOMIC_BLOCK(ATOMIC_RESTORESTATE){
    currPos = pos;
    }

  int e =  target - currPos;
  int dedE = (e - prevE) / deltaT;
  integralE = integralE + e*integralE;

  float u = kp*e + ki * integralE  + kd * dedE;

  if((u < 40 && u > 0) || (u < 0 && u > - 40)){
    u = 0;
    }
  
  int pwm = (fabs(u)/target) * 255;
  if(pwm > 255){
    pwm = 255;
    }
  int dir = 1;
  if(u < 0){
     dir = -1;
  }

  setMotor(dir,PWM,pwm,inA,inB);

  prevE = e;

  Serial.print(target);
  Serial.print(" ");
  Serial.println(pos);
//  Serial.print(" ");
//  Serial.print(u);
//  Serial.print(" ");
//  Serial.println(pwm);
// setMotor(1,PWM,100,inA,inB);
}

void setMotor(int dir , int pwm , int pwmValue , int in1 , int in2){
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
void readEncoder(){
  if(digitalRead(encB)) pos++;
  else pos--;
  }
