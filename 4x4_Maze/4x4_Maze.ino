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

int W = 2;
int L = 4;

volatile int posR = 0, posL = 0;
long prevT = 0;
float prevER = 0, prevEL = 0;
float integralER = 0, integralEL = 0;
int target;
int cellLength = 30;
int dirs[] = { 1, 2, 4, 8 };
int currDir = 1, minDir;
int nextDir;
int X, Y;

struct cell {
  int distance;
  /* binary number represent each cell's walls  
      in directions W-S-E-N
      1 ==>> no wall
      0 ==>> wall
  */
  int walls;
};

cell maze[5][5];

struct coord {
  int x;
  int y;
};

coord start = { 1, 1 };
coord end = { 2, 4 };
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
  begin_();
  delay(100);
}

void loop() {
  while (curr.x != end.x || curr.y != end.y) {
    delay(1000);
    updateWalls(curr);
    int WALLS = maze[curr.x][curr.y].walls;
    if (WALLS & 1) {
      coord next;
      next = calcCoord(1, curr.x, curr.y);
      if (isValid(next.x, next.y)) {
        if (maze[next.x][next.y].distance < minDist) {
          X = next.x;
          Y = next.y;
          minDist = maze[next.x][next.y].distance;
          minDir = 1;
          minCoord.x = X;
          minCoord.y = Y;
        }
      }
    }
    if (WALLS & 8) {
      coord next;
      next = calcCoord(8, curr.x, curr.y);
      if (isValid(next.x, next.y)) {
        if (maze[next.x][next.y].distance < minDist) {
          minDist = maze[next.x][next.y].distance;
          minDir = 8;
          X = next.x;
          Y = next.y;
          minCoord.x = X;
          minCoord.y = Y;
        }
      }
    }
    if (WALLS & 2) {
      coord next;
      next = calcCoord(2, curr.x, curr.y);
      if (isValid(next.x, next.y)) {
        if (maze[next.x][next.y].distance < minDist) {
          X = next.x;
          Y = next.y;
          minDist = maze[next.x][next.y].distance;
          minCoord.x = X;
          minCoord.y = Y;
          minDir = 2;
        }
      }
    }
    moveTo(minDir, currDir, 75);
    currDir = minDir;
    X = minCoord.x;
    Y = minCoord.y;
    curr.x = X;
    curr.y = Y;
    Serial.println("X : " + String(curr.x) + " ,Y : " + String(curr.y));
    Serial.println("minDist: " + String(minDist));
  }
  setMotor(0, 0, inAR, inBR, PWMR);
  setMotor(0, 0, inAL, inBL, PWML);
}

/*instantiates a maze with no walls except boundries
   instantiates each cell's distance from the center
*/
void begin_() {
  for (int i = 1; i < 5; i++) {
    for (int j = 1; j < 5; j++) {
      maze[i][j].distance = calcDist(i, j);
      maze[i][j].walls = 15;
    }
  }
}


coord calcCoord(int nextDir, int x_, int y_) {
  coord nextCoord = { 0, 0 };
  switch (nextDir) {
    case 1:
      nextCoord.x = x_;
      nextCoord.y = y_ + 1;
      break;
    case 2:
      nextCoord.x = x_ + 1;
      nextCoord.y = y_;
      break;
    case 4:
      nextCoord.x = x_;
      nextCoord.y = y_ - 1;
      break;
    case 8:
      nextCoord.x = x_ - 1;
      nextCoord.y = y_;
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
  int prevRightDist = readUltrasonicR();
  int prevLeftDist = readUltrasonicL();
  // int dampR = 1, dampL = 1 , incre = 0.5;
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
    if (readUltrasonicL() < 15) {
      if (readUltrasonicL() > prevLeftDist) {
        pwmL = pwmR - 0.15 * pwmR;
        pwmR = pwmR + 0.15 * pwmR;
      } else if (readUltrasonicL() < prevLeftDist) {
        pwmL = pwmR + 0.15 * pwmR;
        pwmR = pwmR - 0.15 * pwmR;
      }
    }
    if (readUltrasonicR() < 15) {
      if (readUltrasonicL() < prevLeftDist) {
        pwmL = pwmR - 0.15 * pwmR;
        pwmR = pwmR + 0.15 * pwmR;
      } else if (readUltrasonicL() > prevLeftDist) {
        pwmL = pwmR + 0.15 * pwmR;
        pwmR = pwmR - 0.15 * pwmR;
      }
    }

    setMotor(dirR, pwmR, inAR, inBR, PWMR);
    setMotor(dirL, pwmL, inAL, inBL, PWML);
    ATOMIC_BLOCK(ATOMIC_RESTORESTATE) {
      currPosR = posR;
      currPosL = posL;
    }
    eL = targetL - currPosL, eR = currPosR - targetR;
    //  Serial.print(-targetR);
    //   Serial.print(" ");
    //   Serial.print(targetL);
    //   Serial.print(" ");
    //   Serial.print(-posR);
    //   Serial.print(" ");
    //   Serial.println(posL);
  }
  setMotor(0, 0, inAR, inBR, PWMR);
  setMotor(0, 0, inAL, inBL, PWML);
  ATOMIC_BLOCK(ATOMIC_RESTORESTATE) {
    posR = 0;
    posL = 0;
  }
}

bool isValid(int x, int y) {
  bool IsValid = true;
  if((x > W) || (x <= 0) || (y > L) || (y <= 0)) IsValid = false;
  return IsValid;
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
void updateWalls(coord currCoord) {
  int wallDir;
  int newWalls = maze[currCoord.x][currCoord.y].walls;
  int wallsToAdd = 0;
  int distance;


  distance = readUltrasonicF();

  if (distance < 20) {
    wallDir = currDir;
    if (newWalls & wallDir) {
      wallsToAdd += wallDir;
    }
  }


  distance = readUltrasonicL();

  if (distance < 20) {
    wallDir = currDir >> 1;

    if (wallDir < 1) wallDir = 8;
    else if (wallDir > 8) wallDir = 1;

    if (newWalls & wallDir) {
      wallsToAdd += wallDir;
    }
  }


  distance = readUltrasonicR();

  if (distance < 20) {
    wallDir = currDir << 1;

    if (wallDir < 1) wallDir = 8;
    else if (wallDir > 8) wallDir = 1;

    if (newWalls & wallDir) {
      wallsToAdd += wallDir;
    }
  }
  newWalls -= wallsToAdd;
  maze[currCoord.x][currCoord.y].walls = newWalls;
}

int calcDist(int posX, int posY) {
  int distance;
  distance = abs(W - posX) + abs(L - posY);
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