#include <util/atomic.h>

#define IRL A7
#define IRR A6

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

#define trigR 12
#define trigL A3
#define trigF A1

#define echoR A0
#define echoL A4
#define echoF A2

volatile int posR = 0, posL = 0;
long prevT = 0;
float prevER = 0, prevEL = 0;
float integralER = 0, integralEL = 0;
int target;
int cellLength = 30;
int dirs[] = { 1, 2, 4, 8 };
int currDir = 1;
int nextDir;
int X,Y;
 


struct coord {
  int x;
  int y;
};

coord start = { 0, 0 };
coord end = { 2, 2};
coord curr = start;
coord minCoord;
int minDist = 100;
int nextDist;

void setup() {
  Serial.begin(9600);
  pinMode(encAR, INPUT);
  pinMode(encBR, INPUT);
  pinMode(inAR, OUTPUT);
  pinMode(inBR, OUTPUT);
  pinMode(PWMR, OUTPUT);
  attachInterrupt(digitalPinToInterrupt(encAR), readEncoderR, RISING);

  pinMode(encAL, INPUT);
  pinMode(encBL, INPUT);
  pinMode(inAL, OUTPUT);
  pinMode(inBL, OUTPUT);
  pinMode(PWML, OUTPUT);
  attachInterrupt(digitalPinToInterrupt(encAL), readEncoderL, RISING);

  pinMode(trigR, OUTPUT);
  pinMode(echoR, INPUT);
  pinMode(trigL, OUTPUT);
  pinMode(echoL, INPUT);
  pinMode(trigF, OUTPUT);
  pinMode(echoF, INPUT);

  pinMode(IRR, INPUT);
  pinMode(IRL, INPUT);
  
}

void loop() {
  while (curr.x != end.x || curr.y != end.y) {
    if (readUltrasonicF() > 20) {
      coord next ;
      next = calcCoord(nextDir,curr.x,curr.y);
      nextDist = calcDist(next.x , next.y);
      if(nextDist < minDist){
        nextDir =  currDir;
        
        X = next.x; Y = next.y;
        minDist = nextDist;
        minCoord.x = X;
        minCoord.y = Y;
      }
    } 
    if (readUltrasonicL() > 20) {
      coord next ;
      next = calcCoord(nextDir,curr.x,curr.y);
      nextDist = calcDist(next.x , next.y);
      if(nextDist < minDist){
        nextDir = currDir >> 1;
        if (nextDir <= 0) nextDir = 8;
        else if (nextDir >= 16) nextDir = 1;
        minDist = nextDist;
        
        X = next.x; Y = next.y;
        minCoord.x = X;
        minCoord.y = Y;
      }
    }
    if (readUltrasonicR() > 20) {
      coord next ;
      next = calcCoord(nextDir,curr.x,curr.y);
      nextDist = calcDist(next.x , next.y);
      if(nextDist < minDist){
        nextDir = currDir << 1;
        if (nextDir <= 0) nextDir = 8;
        else if (nextDir >= 16) nextDir = 1;
        
        X = next.x; Y = next.y;
        minDist = nextDist;
        minCoord.x = X;
        minCoord.y = Y;
      }
    }
    Serial.println("X : " + String(minCoord.x) + " ,Y : " + String(minCoord.y));
    Serial.println("minDist" + String(minDist));
    moveTo(nextDir,currDir,100);
    currDir = nextDir;
    X = minCoord.x;
    Y = minCoord.y;
    curr.x = X;
    curr.y = Y;
    delay(1000);
    
  }
  setMotor(0, 0, inAR, inBR, PWMR);
  setMotor(0, 0, inAL, inBL, PWML);
 
}

coord calcCoord(int nextDir, int x_, int y_) {
  coord nextCoord;
  nextCoord.y = y_;
  nextCoord.x = x_;
      switch (nextDir) {
        case 1:
          nextCoord.y = y_ + 1;
          break;
        case 2:
          nextCoord.x = x_ + 1;
          break;
        case 4:
          nextCoord.y = y_ - 1;
          break;
        case 8:
          nextCoord.x = x_ - 1;
          break;
      }
  return nextCoord;
}
void moveDist(int dist, int Speed) {
  int ppr = 385;
  float tireCirc = 2 * 3.5 * 3.14;
  int currPosR, currPosL;
  ATOMIC_BLOCK(ATOMIC_RESTORESTATE) {
    currPosR = posR;
    currPosL = posL;
  }
  int targetR = -((float(dist) / tireCirc) * float(ppr)) + currPosR;
  int targetL = (float(dist) / tireCirc) * float(ppr) + currPosL;
  int eL = targetL - currPosL, eR = currPosR - targetR;
  int dirR = 1, dirL = 1;
  if (eR < 0) dirR = -1;
  if (eL < 0) dirL = -1;
  int mn = 0.02 * abs(eR);
  while (abs(eR) > mn) {
    if (readUltrasonicF() < 10) {
      setMotor(0, 0, inAR, inBR, PWMR);
      setMotor(0, 0, inAL, inBL, PWML);
      break;
    }
    ATOMIC_BLOCK(ATOMIC_RESTORESTATE) {
      while (analogRead(IRL) < 100) {
        setMotor(-1, 80, inAR, inBR, PWMR);
        setMotor(1, 80, inAL, inBL, PWML);
      }
      while (analogRead(IRR) < 100) {
        setMotor(1, 80, inAR, inBR, PWMR);
        setMotor(-1, 80, inAL, inBL, PWML);
      }
    }
    int pwmR = float(0.45 * abs(eR)) * Speed, pwmL;
    if (pwmR > Speed) {
      pwmR = Speed;
    }
    dirR = 1, dirL = 1;
    if (eR < 0) {
      break;
    }
    if (eL < 0) {
      break;
    }
    if (abs(eL) < abs(eR)) {
      pwmL = pwmR - 0.15 * pwmR;
      pwmR = pwmR + 0.15 * pwmR;
    } else if (abs(eL) > abs(eR)) {
      pwmL = pwmR + 0.15 * pwmR;
      pwmR = pwmR - 0.15 * pwmR;
    }
    setMotor(dirR, pwmR, inAR, inBR, PWMR);
    setMotor(dirL, pwmL, inAL, inBL, PWML);
    ATOMIC_BLOCK(ATOMIC_RESTORESTATE) {
      currPosR = posR;
      currPosL = posL;
    }
    eL = targetL - currPosL, eR = currPosR - targetR;
  }
  setMotor(0, 0, inAR, inBR, PWMR);
  setMotor(0, 0, inAL, inBL, PWML);
  ATOMIC_BLOCK(ATOMIC_RESTORESTATE) {
    posR = 0;
    posL = 0;
  }
}

bool isValid(int x, int y) {
  return ((x > 4) || (x < 0) || (y > 4) || (y < 0)) ? false : true;
}
void setMotor(int dir, int pwmValue, int in1, int in2, int pwm) {
  analogWrite(pwm, pwmValue);
  if (dir == 1) {
    digitalWrite(in1, HIGH);
    digitalWrite(in2, LOW);
  } else if (dir == -1) {
    digitalWrite(in2, HIGH);
    digitalWrite(in1, LOW);
  } else {
    digitalWrite(in1, LOW);
    digitalWrite(in2, LOW);
  }
}
void readEncoderR() {
  if (digitalRead(encBR)) posR++;
  else posR--;
}

void readEncoderL() {
  if (digitalRead(encBL)) posL++;
  else posL--;
}


void turnAround(int degree, int Speed) {
  int fullTurn = 920;
  int target;
  switch (degree) {
    case 90:
      target = 0.25 * fullTurn;
      break;
    case -90:
      target = -0.25 * fullTurn;
      break;
    case 180:
      target = 0.52 * fullTurn;
      break;
    case -180:
      target = -0.52 * fullTurn;
      break;
    case 360:
      target = fullTurn;
      break;
  }

  int currPosR, currPosL;
  ATOMIC_BLOCK(ATOMIC_RESTORESTATE) {
    currPosR = posR;
    currPosL = posL;
  }
  int targetR = target + currPosR, targetL = target + currPosL;
  int eL = targetL - currPosL, eR = currPosR - targetR;
  int dirR = 1, dirL = 1;
  if (eR < 0) {
    dirR = -1;
  }
  if (eL < 0) {
    dirL = -1;
  }
  int mn = 0.05 * abs(eR);

  while (abs(eR) > mn) {
    int pwmR = float(0.45 * abs(eR)) * Speed, pwmL;
    if (pwmR > Speed) {
      pwmR = Speed;
    }
    // if (eR < 0) {
    //   break;
    // }
    // if (eL < 0) {
    //   break;
    // }
    if (abs(eL) < abs(eR)) {
      pwmL = pwmR - 0.15 * pwmR;
      pwmR = pwmR + 0.15 * pwmR;
    } else if (abs(eL) > abs(eR)) {
      pwmL = pwmR + 0.15 * pwmR;
      pwmR = pwmR - 0.15 * pwmR;
    }
    setMotor(dirR, pwmR, inAR, inBR, PWMR);
    setMotor(dirL, pwmL, inAL, inBL, PWML);
    ATOMIC_BLOCK(ATOMIC_RESTORESTATE) {
      currPosR = posR;
      currPosL = posL;
    }
    eL = targetL - currPosL, eR = currPosR - targetR;
  }
  setMotor(0, 0, inAR, inBR, PWMR);
  setMotor(0, 0, inAL, inBL, PWML);
  ATOMIC_BLOCK(ATOMIC_RESTORESTATE) {
    posR = 0;
    posL = 0;
  }
}


int readUltrasonicR() {
  digitalWrite(trigR, LOW);
  delayMicroseconds(2);
  digitalWrite(trigR, HIGH);
  delayMicroseconds(10);
  digitalWrite(trigR, LOW);
  int distant = pulseIn(echoR, HIGH);
  int disC = distant * 0.5 * 0.034;
  return disC;
}

int readUltrasonicL() {
  digitalWrite(trigL, LOW);
  delayMicroseconds(2);
  digitalWrite(trigL, HIGH);
  delayMicroseconds(10);
  digitalWrite(trigL, LOW);
  int distant = pulseIn(echoL, HIGH);
  int disC = distant * 0.5 * 0.034;
  return disC;
}

int readUltrasonicF() {
  digitalWrite(trigF, LOW);
  delayMicroseconds(2);
  digitalWrite(trigF, HIGH);
  delayMicroseconds(10);
  digitalWrite(trigF, LOW);
  int distant = pulseIn(echoF, HIGH);
  int disC = distant * 0.5 * 0.034;
  return disC;
}
int updateWalls() {
  int newWalls = 15;
  int wallsToAdd = 0;
  int distance;
  distance = readUltrasonicF();
  if (distance < 20) {
    if (newWalls & 1) {
      wallsToAdd += 1;
    }
  }
  distance = readUltrasonicL();
  if (distance < 20) {
    if (newWalls & 8) {
      wallsToAdd += 8;
    }
  }
  distance = readUltrasonicR();
  if (distance < 20) {
    if (newWalls & 2) {
      wallsToAdd += 2;
    }
  }
  newWalls -= wallsToAdd;
  return newWalls;
}

int calcDist(int posX, int posY) {
  int distance = 0;
  distance = abs(2 - posX) + abs(2 - posY);
  return distance;
}
void moveTo(int heading, int currHeading, int Speed) {
  switch (heading) {
    case 1:
      switch (currHeading) {
        case 1:
          moveDist(cellLength, Speed);
          break;
        case 2:
          turnAround(-90, Speed);
          moveDist(cellLength, Speed);
          break;
        case 4:
          turnAround(-180, Speed);
          moveDist(cellLength, Speed);
          break;
        case 8:
          turnAround(90, Speed);
          moveDist(cellLength, Speed);
          break;
      }
      break;
    case 2: /*N >> S*/
      switch (currHeading) {
        case 1:
          turnAround(90, Speed);
          moveDist(cellLength, Speed);
          break;
        case 2:
          moveDist(cellLength, Speed);
          break;
        case 4:
          turnAround(-90, Speed);
          moveDist(cellLength, Speed);
          break;
        case 8:
          turnAround(180, Speed);
          moveDist(cellLength, Speed);
          break;
      }
      break;
    case 4:
      switch (currHeading) {
        case 1:
          turnAround(180, Speed);
          moveDist(cellLength, Speed);
          break;
        case 2:
          turnAround(90, Speed);
          moveDist(cellLength, Speed);
          break;
        case 4:
          moveDist(cellLength, Speed);
          break;
        case 8:
          turnAround(-90, Speed);
          moveDist(cellLength, Speed);
          break;
      }
      break;
    case 8:
      switch (currHeading) {
        case 1:
          turnAround(-90, Speed);
          moveDist(cellLength, Speed);
          break;
        case 2:
          turnAround(180, Speed);
          moveDist(cellLength, Speed);
          break;
        case 4:
          turnAround(90, Speed);
          moveDist(cellLength, Speed);
          break;
        case 8:
          moveDist(cellLength, Speed);
          break;
      }
      break;
  }
}