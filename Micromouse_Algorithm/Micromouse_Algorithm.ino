#include <QList.h>

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
    and the center of the maze{(5,5),(6,5),(5,6),(6,6)}*/
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

int updateWalls() {
  //reads from ultrasonic
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
bool isDead(int walls) {
  int noWalls = 0;
  for (int i = 0; i < sizeof(dirs); i++) {
    if (walls & dirs[i] != 0) noWalls++;
  }
  return (noWalls == 1) ? true : false;
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
  QList<coord> closedList;
  QList<coord> openList;
  openList.push_back(currCoord);
  maze[currCoord.x][currCoord.y].walls = updateWalls();
  coord minCoord = { 0, 0 };
  int minDist = X * Y, minDir;
  /*searching the 4 directions to find the cell with minimam distance*/
  for (int i = 0; i < sizeof(dirs); i++) {
    int dir = dirs[i];
    /*if there isn't a wall in this direction*/
    if (maze[currCoord.x][currCoord.y].walls & dir != 0) {
      coord nextCoord = bearingCoord(currCoord, dir);
      if (checkCoord(nextCoord)) {
        if (isEnd(nextCoord, goal)) {
          //move to end-cell
          //stop
          return;
        } else if (maze[nextCoord.x][nextCoord.y].distance < minDist) {
          minDist = maze[nextCoord.x][nextCoord.y].distance;
          minCoord = nextCoord;
          minDir = dir;
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
  coord temp = openList.back();
  closedList.push_back(temp);
  openList.pop_back();
  if (maze[minCoord.x][minCoord.y].distance != (maze[temp.x][temp.y].distance + 1))
    maze[minCoord.x][minCoord.y].distance = maze[temp.x][temp.y].distance + 1;
  openList.push_back(minCoord);
  //motor goes to this cell ==>> moves one cell in the direction of minDir
  while (openList.size() != 0) {
    temp = openList.back();
    openList.pop_back();
    closedList.push_back(temp);
    maze[temp.x][temp.y].walls = updateWalls();
    minCoord = { 0, 0 };
    minDist = X * Y;
    for (int i = 0; i < sizeof(dirs); i++) {
      int dir = dirs[i];
      if (maze[temp.x][temp.y].walls & dir != 0) {
        coord nextCoord = bearingCoord(temp, dir);
        if (checkCoord(nextCoord)) {
          if (isEnd(nextCoord, goal)) {
            //move to end-cell
            //stop
            return;
          } else if (maze[nextCoord.x][nextCoord.y].distance < minDist) {
            minDist = maze[nextCoord.x][nextCoord.y].distance;
            minCoord = nextCoord;
            minDir = dir;
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
    if (maze[minCoord.x][minCoord.y].distance != (maze[temp.x][temp.y].distance + 1))
      maze[minCoord.x][minCoord.y].distance = maze[temp.x][temp.y].distance + 1;
    openList.push_back(minCoord);
  }
}
void setup() {
  begin_();
}

void loop() {
  coord goalCoord[] = { (X / 2 - 1, Y / 2 - 1), (X / 2, Y / 2 - 1), (X / 2 - 1, Y / 2), (X / 2, Y / 2) };
  coord startCoord = { 0, 0 };
  aStarSearch(startCoord, goalCoord);
}