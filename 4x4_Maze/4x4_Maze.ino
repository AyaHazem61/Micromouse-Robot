#include <util/atomic.h>

#define IRL 0
#define IRR 13

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
void setup() {
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

<<<<<<< HEAD
  pinMode(IRR, INPUT_PULLUP);
  pinMode(IRL, INPUT_PULLUP);
  //________1______
  /*turnAround(90, 80);
  moveDist(30, 100);
  moveDist(30, 100);
  setMotor(0, 0, inAR, inBR, PWMR);
  setMotor(0, 0, inAL, inBL, PWML);*/

  //_______2______
  /*
  moveTo(1,8,100);
  */
||||||| 4bd1bd2
  pinMode(IRR, INPUT);
  pinMode(IRL, INPUT);
  turnAround(90, 80);
  moveDist(30, 100);
  moveDist(30, 100);
=======
  pinMode(IRR, INPUT);
  pinMode(IRL, INPUT);
>>>>>>> 452e3ea518227f45b5c715c36b5b44062034bfc2
}

void loop() {
  //_______3________
  
  /*int currDir = 1;
  int walls = updateWalls();
  while(walls & 15 != 15) {
    if (walls & 1 != 0) {
      moveTo(1 , currDir , 100);
      currDir = 1;
    }
    else if (walls & 2 != 0) {
      moveTo(2 , currDir , 100);
      currDir = 2;
    }
    else if (walls & 4 != 0) {
      moveTo(4 , currDir , 100);
      currDir = 4;
    }
    else if (walls & 8 != 0) {
      moveTo(8 , currDir , 100);
      currDir = 8;
      }
    walls = updateWalls();
    }*/

  //___________4__________
  /*int end_x = 3, end_y = 4;
  int start_x = 2, start_y = 1;
  int curr_x = start_x, curr_y = start_y;
  while ((curr_x != end_x) && (curr_y != end_y)) {
    int min_x = -1, min_y = -1, minDist = 100, minDir , currDir = 1;
    int wall = updateWalls();
    Serial.println(wall);
    int walls = updateWalls();
    if (walls & 1 != 0) {
      int distance = calcDist(curr_x, curr_y + 1);
      if (distance < minDist) {
        if (isValid(curr_x, curr_y + 1)) {
          min_x = curr_x;
          min_y = curr_y + 1;
          minDist = distance;
          minDir = 1;
        }
      }
    }
    if (walls & 2 != 0) {
      int distance = calcDist(curr_x, curr_y - 1);
      if (distance < minDist) {
        if (isValid(curr_x, curr_y - 1)) {
          min_x = curr_x;
          min_y = curr_y - 1;
          minDist = distance;
          minDir = 2;
        }
      }
    }
    if (walls & 4 != 0) {
      int distance = calcDist(curr_x - 1, curr_y);
      if (distance < minDist) {
        if (isValid(curr_x - 1, curr_y)) {
          min_x = curr_x - 1;
          min_y = curr_y;
          minDist = distance;
          minDir = 4;
        }
      }
    }
    if (walls & 8 != 0) {
      int distance = calcDist(curr_x + 1, curr_y);
      if (distance < minDist) {
        if (isValid(curr_x + 1, curr_y)) {
          min_x = curr_x + 1;
          min_y = curr_y;
          minDist = distance;
          minDir = 8;
        }
      }
    }
    if (isValid(min_x, min_y)) {
      moveTo(minDir, currDir, 100);
      curr_x = min_x;
      curr_y = min_y;
      currDir = minDir;
    }
    while (min_x != -1 && min_y != -1) {
      int wall = updateWalls();
      Serial.println(wall);
      min_x = -1;
      min_y = -1;
      minDist = 100;
      minDir;
      int walls = updateWalls();
      if (walls & 1 != 0) {
        int distance = calcDist(curr_x, curr_y + 1);
        if (distance < minDist) {
          min_x = curr_x;
          min_y = curr_y + 1;
          minDist = distance;
          minDir = 1;
        }
      }
      if (walls & 2 != 0) {
        int distance = calcDist(curr_x, curr_y - 1);
        if (distance < minDist) {
          min_x = curr_x;
          min_y = curr_y - 1;
          minDist = distance;
          minDir = 2;
        }
      }
      if (walls & 4 != 0) {
        int distance = calcDist(curr_x - 1, curr_y);
        if (distance < minDist) {
          min_x = curr_x - 1;
          min_y = curr_y;
          minDist = distance;
          minDir = 4;
        }
      }
      if (walls & 8 != 0) {
        int distance = calcDist(curr_x + 1, curr_y);
        if (distance < minDist) {
          min_x = curr_x + 1;
          min_y = curr_y;
          minDist = distance;
          minDir = 8;
        }
      }
      if (isValid(min_x, min_y)) {
        moveTo(minDir, currDir, 100);
        curr_x = min_x;
        curr_y = min_y;
        currDir = minDir;
      }
    }
  }*/
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
      while (!digitalRead(IRL)) {
        setMotor(0, 0, inAR, inBR, PWMR);
        setMotor(1, 80, inAL, inBL, PWML);
      }
      while (!digitalRead(IRR)) {
        setMotor(1, 80, inAR, inBR, PWMR);
        setMotor(0, 0, inAL, inBL, PWML);
      }
    }
    int pwmR = float(0.45 * abs(eR)) * Speed, pwmL;
    if (pwmR > Speed) {
      pwmR = Speed;
    }
    dirR = 1, dirL = 1;
    if (eR < 0) {
      dirR = -1;
    }
    if (eL < 0) {
      dirL = -1;
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
    dirR = 1, dirL = 1;
    if (eR < 0) {
      dirR = -1;
    }
    if (eL < 0) {
      dirL = -1;
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
    if (newWalls & 1 != 0) {
      wallsToAdd += 1;
    }
  }
  distance = readUltrasonicL();
  if (distance < 20) {
    if (newWalls & 8 != 0) {
      wallsToAdd |= 8;
    }
  }
  distance = readUltrasonicR();
  if (distance < 20) {
    if (newWalls & 4 != 0) {
      wallsToAdd += 4;
    }
  }
  newWalls -= wallsToAdd;
  return newWalls;
}

int calcDist(int posX, int posY) {
  int distance = 0;
  distance = abs(3 - posX) + abs(4 - posY);
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
          turnAround(-180, Speed);
          moveDist(cellLength, Speed);
          break;
        case 4:
          turnAround(-90, Speed);
          moveDist(cellLength, Speed);
          break;
        case 8:
          turnAround(90, Speed);
          moveDist(cellLength, Speed);
          break;
          break;
      }
      break;
    case 2: /*N >> S*/
      switch (currHeading) {
        case 1:
          turnAround(180, Speed);
          moveDist(cellLength, Speed);
          break;
        case 2:
          moveDist(cellLength, Speed);
          break;
        case 4:
          turnAround(90, Speed);
          moveDist(cellLength, Speed);
          break;
        case 8:
          turnAround(-90, Speed);
          moveDist(cellLength, Speed);
          break;
          break;
      }
      break;
    case 4:
      switch (currHeading) {
        case 1:
          turnAround(-90, Speed);
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
          turnAround(180, Speed);
          moveDist(cellLength, Speed);
          break;
          break;
      }
      break;
    case 8:
      switch (currHeading) {
        case 1:
          turnAround(90, Speed);
          moveDist(cellLength, Speed);
          break;
        case 2:
          turnAround(-90, Speed);
          moveDist(cellLength, Speed);
          break;
        case 4:
          turnAround(180, Speed);
          moveDist(cellLength, Speed);
          break;
        case 8:
          moveDist(cellLength, Speed);
          break;
          break;
      }
      break;
  }
}
