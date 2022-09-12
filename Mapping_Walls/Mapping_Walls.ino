#define trigR A0
#define trigL A4
#define trigF A1

#define echoR 12
#define echoL A3
#define echoF A2

struct coord{
  byte x; byte y;
  };
struct cell{
  int distance;
  /* binary number represent each cell's walls  
      in directions W-E-S-N
      1 ==>> no wall
      0 ==>> wall
  */
  byte walls;
  coord parent;
  };
#define X 12
#define Y 12

/*directions :
     North 0001 ==>> 1
     South 0010 ==>> 2
     East  0100 ==>> 4
     West  1000 ==>> 8
*/
byte dirs[] = {1,2,4,8};

cell maze[X][Y];

void setup() {
//  Serial.begin(9600);
  pinMode(trigR,OUTPUT);
  pinMode(echoR,INPUT);
  pinMode(trigL,OUTPUT);
  pinMode(echoL,INPUT);
  pinMode(trigF,OUTPUT);
  pinMode(echoF,INPUT);
  begin_();
}

void loop() {
//  delay(4000);
  coord startCoord = {0,0};
  int wall = updateWalls(startCoord);
  maze[startCoord.x][startCoord.y].walls = wall;
/*  byte _8 = (wall & 8)? 0:1;
    Serial.print(_8);
    byte _4 = (wall & 4)? 0:1;
    Serial.print(_4);
    byte _2 = (wall & 2)? 0:1;
    Serial.print(_2);
    byte _1 = (wall & 1)? 0:1;
    Serial.println(_1);*/
}

/* calculates the distance between cell (posX,posY)
    and the center of the maze{(7,7),(8,7),(7,8),(8,8)}*/
int calcDist(int posX,int posY,int dim){
  int distance = 0;
  int center = dim/2;
  if(posX < center){
    if(posY < center){
      posX--;
      posY--;
      distance = abs(center - posX) + abs(center - posY);
      }
    else if (posY > center ){
      posY--;
      distance = abs(center - posX) + abs(center - posY);
      }    
      }
  else if(posX >center){
    if(posY < center){
      posX--;
      distance = abs(center - posX) + abs(center - posY);
      }
    else if (posY > center){
      distance = abs(center - posX) + abs(center - posY);
      }    
      }
   return distance;
  }

/*instantiates a maze with no walls except boundries
   instantiates each cell's distance from the center
*/
void begin_(){
  for(int i = 0 ;i< X ; i++){
    for(int j = 0 ;j< Y; j++){
      maze[i][j].distance = calcDist(i,j,X);
      maze[i][j].walls = 15;
      maze[i][j].parent.x = -1;
      maze[i][j].parent.y = -1;
      if(i == 0){
        maze[i][j].walls = 13;
        }
      else if(i == X-1){
        maze[i][j].walls = 14;
        }
      else if(j == 0){
        maze[i][j].walls = 11;
        }
        else if(j == j-1){
        maze[i][j].walls = 7;
        }
     }}
    maze[0][0].walls = 9;
    maze[0][Y-1].walls = 5;
    maze[X-1][0].walls = 10;
    maze[X-1][Y-1].walls = 6;
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

int updateWalls(coord currCoord){
  int newWalls = maze[currCoord.x][currCoord.y].walls; 
  byte wallsToAdd = 0;
  int distance;
  distance = readUltrasonicF();
  if(distance <= 10){
    if(newWalls & 1 != 0){
      wallsToAdd += 1;
      }
  }
  distance = readUltrasonicL();
  if(distance <= 10){
     if(newWalls & 8 != 0){
        wallsToAdd |= 8;
     }
  }    
  distance = readUltrasonicR();
  if(distance <= 20){
    if(newWalls & 4 != 0){
       wallsToAdd += 4;
        }
      }    
    newWalls -= wallsToAdd;
    return newWalls;
    }
