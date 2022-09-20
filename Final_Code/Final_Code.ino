#include <QList.h>
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
struct coord {
  byte x;
  byte y;
};
struct cell {
  int distance;
  /* binary number represent each cell's walls  
      in directions W-E-S-N
      1 ==>> no wall
      0 ==>> wall
  */
  byte walls;
  coord parent;
};
#define X 16
#define Y 16

/*directions :
     North 0001 ==>> 1
     South 0010 ==>> 2
     East  0100 ==>> 4
     West  1000 ==>> 8
*/
byte dirs[] = { 1, 2, 4, 8 };

cell maze[X][Y];

/* calculates the distance between cell (posX,posY)
    and the center of the maze{(7,7),(8,7),(7,8),(8,8)}*/
int calcDist(int posX, int posY, int dim) {
  int distance = 0;
  int center = dim / 2;
  if (posX < center) {
    if (posY < center) {
      posX--;
      posY--;
      distance = abs(center - posX) + abs(center - posY);
    } else if (posY > center) {
      posY--;
      distance = abs(center - posX) + abs(center - posY);
    }
  } else if (posX > center) {
    if (posY < center) {
      posX--;
      distance = abs(center - posX) + abs(center - posY);
    } else if (posY > center) {
      distance = abs(center - posX) + abs(center - posY);
    }
  }
  return distance;
}

/*instantiates a maze with no walls except boundries
   instantiates each cell's distance from the center
*/
void begin_() {
  for (int i = 0; i < X; i++) {
    for (int j = 0; j < Y; j++) {
      maze[i][j].distance = calcDist(i, j, X);
      maze[i][j].walls = 15;
      maze[i][j].parent.x = -1;
      maze[i][j].parent.y = -1;
      if (i == 0) {
        maze[i][j].walls = 13;
      } else if (i == X - 1) {
        maze[i][j].walls = 14;
      } else if (j == 0) {
        maze[i][j].walls = 11;
      } else if (j == j - 1) {
        maze[i][j].walls = 7;
      }
    }
  }
  maze[0][0].walls = 9;
  maze[0][Y - 1].walls = 5;
  maze[X - 1][0].walls = 10;
  maze[X - 1][Y - 1].walls = 6;
}

bool checkCoord(coord currCoord) {
  if (currCoord.x < 0 || currCoord.x > X - 1 || currCoord.y < 0 || currCoord.y > Y - 1)
    return false;
  return true;
}

/*takes coordinate of a cell and a direction 
 * outputs next cell in this direction
*/
coord bearingCoord(coord currCoord, int dir) {
  coord nextCoord = { 0, 0 };
  switch (dir) {
    case 1:
      nextCoord.x = currCoord.x + 1;
      nextCoord.y = currCoord.y;
      break;
    case 2:
      nextCoord.x = currCoord.x - 1;
      nextCoord.y = currCoord.y;
      break;
    case 4:
      nextCoord.x = currCoord.x;
      nextCoord.y = currCoord.y - 1;
      break;
    case 8:
      nextCoord.x = currCoord.x;
      nextCoord.y = currCoord.y + 1;
      break;
  }
}

bool isEnd(coord Coord, coord goal[]) {
  bool is_end = false;
  for (int i = 0; i < sizeof(goal); i++) {
    if (goal[i].x == Coord.x && goal[i].y == Coord.y) is_end = true;
  }
  return isEnd;
}

/*
INPUT: Coordindate to update, and a direction representing the wall to add
OUTPUT: Update to coordinate adding the wall provided as an argument
*/

void coordUpdate(coord coordinate, int wallDir) {
  if (checkCoord(coordinate)) {
    /*if there's no wall found in this direction(its value = 1)*/
    if ((maze[coordinate.y][coordinate.x].walls & wallDir) != 0) {
      /*add the wall (make its value 0)*/
      maze[coordinate.y][coordinate.x].walls = maze[coordinate.y][coordinate.x].walls - wallDir;
    }
  }
}

void aStarSearch(coord currCoord, coord goal[]) {
  int currDir = 1;
  QList<coord> closedList;
  QList<coord> openList;
  openList.push_back(currCoord);
  maze[currCoord.x][currCoord.y].walls = updateWalls(currCoord);
  coord minCoord = { 0, 0 };
  int minDist = X * Y, minDir;
  /*searching the 4 directions to find the cell with minimam distance*/
  for (int i = 0; i < sizeof(dirs); i++) {
    int dir = dirs[i];
    /*if there isn't a wall in this direction*/
    if (maze[currCoord.x][currCoord.y].walls & dir != 0) {
      coord nextCoord = bearingCoord(currCoord, dir);
      if (checkCoord(nextCoord) && !isEnd(nextCoord, goal)) {
        if (maze[nextCoord.x][nextCoord.y].distance < minDist) {
          minDist = maze[nextCoord.x][nextCoord.y].distance;
          minCoord = nextCoord;
          minDir = dir;
        } else if (checkCoord(nextCoord) && isEnd(nextCoord, goal)) {
          moveTo(dir, currDir, 200);
          return;
        }
      }
    }

    /*updating the walls of surrounding cells
        if there is a wall in this direction*/

    else if (maze[currCoord.x][currCoord.y].walls & dir == 0) {
      coord workingCoord = { currCoord.x, currCoord.y };
      switch (dir) {
        /*if there is a wall at north*/
        case 1:
          /*add wall at south to next cell*/
          workingCoord.y = workingCoord.y + 1;
          coordUpdate(workingCoord, 2);
          break;
        case 2:
          workingCoord.y = workingCoord.y - 1;
          coordUpdate(workingCoord, 1);
          break;
        case 4:
          workingCoord.x = workingCoord.x - 1;
          coordUpdate(workingCoord, 8);
          break;
        case 8:
          workingCoord.x = workingCoord.x + 1;
          coordUpdate(workingCoord, 4);
          break;
      }
    }
  }
  maze[minCoord.x][minCoord.y].parent.x = currCoord.x;
  maze[minCoord.x][minCoord.y].parent.y = currCoord.y;
  coord temp = openList.back();
  closedList.push_back(temp);
  openList.pop_back();
  openList.push_back(minCoord);

  //motor goes to this cell ==>> moves one cell in the direction of minDir
  moveTo(minDir, currDir, 100);
  currDir = minDir;
  while (openList.size() != 0) {
    temp = openList.back();
    openList.pop_back();
    closedList.push_back(temp);
    maze[temp.x][temp.y].walls = updateWalls(temp);
    minCoord = { 0, 0 };
    minDist = X * Y;
    for (int i = 0; i < sizeof(dirs); i++) {
      int dir = dirs[i];
      if (maze[temp.x][temp.y].walls & dir != 0) {
        coord nextCoord = bearingCoord(temp, dir);
        if (checkCoord(nextCoord) && !isEnd(nextCoord, goal)) {
          if (maze[nextCoord.x][nextCoord.y].distance < minDist) {
            minDist = maze[nextCoord.x][nextCoord.y].distance;
            minCoord = nextCoord;
            minDir = dir;
          } else if (checkCoord(nextCoord) && isEnd(nextCoord, goal)) {
            //move to end
            //stop
            moveTo(dir, currDir, 200);
            return;
          }
        }
      }
      /*updating the walls of surrounding cells
        if there is a wall in this direction*/
      else if (maze[temp.x][temp.y].walls & dir == 0) {
        coord workingCoord = { temp.x, temp.y };
        switch (dir) {
          /*if there is a wall at north*/
          case 1:
            /*add wall at south to next cell*/
            workingCoord.y = workingCoord.y + 1;
            coordUpdate(workingCoord, 2);
            break;
          case 2:
            workingCoord.y = workingCoord.y - 1;
            coordUpdate(workingCoord, 1);
            break;
          case 4:
            workingCoord.x = workingCoord.x - 1;
            coordUpdate(workingCoord, 8);
            break;
          case 8:
            workingCoord.x = workingCoord.x + 1;
            coordUpdate(workingCoord, 4);
            break;
        }
      }
    }
    maze[minCoord.x][minCoord.y].parent.x = temp.x;
    maze[minCoord.x][minCoord.y].parent.y = temp.y;
    openList.push_back(minCoord);
    moveTo(minDir, currDir, 100);
    currDir = minDir;
  }
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
    // if (readUltrasonicF() < 10) {
    //   setMotor(0, 0, inAR, inBR, PWMR);
    //   setMotor(0, 0, inAL, inBL, PWML);
    //   break;
    // }
    ATOMIC_BLOCK(ATOMIC_RESTORESTATE) {
      // int SPEED = (int)0.8 * Speed;
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
int updateWalls(coord currCoord) {
  int newWalls = maze[currCoord.x][currCoord.y].walls;
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

  begin_();
}

void loop() {
  coord goalCoord[] = { (X / 2 - 1, Y / 2 - 1), (X / 2, Y / 2 - 1), (X / 2 - 1, Y / 2), (X / 2, Y / 2) };
  coord startCoord = { 0, 0 };
  coord currCoord = { 0, 0 };
  while (!isEnd(currCoord, goalCoord)) {
    aStarSearch(currCoord, goalCoord);
  }
}