/*
  This is a template for Group 1 NYU Adv. Mechatronics Spring 2022 Project 3.  The goal of the project is to deliver a
  line following robot that will navigate the grid of streets with one way lanes and a potential blockage to reach multiple
  friendlies and enemies along the route. The robot will then proceed to check for the friends and enemies using a camera and 
  image processing and then proceeds to eliminate the enemies once recognized.  
*/
#include "simpletools.h"   // Include simple tools (THIS IS AUTO INCLUDED AND WILL NOT SHOW UP IN THE PROJECT MANAGER)
#include "ping.h"
#include "servo.h"
              
// ---------------------------------------------------------------------------------------------------------------------Header Files (Above)-----------------------------------------------------------------------------------------

// ---------------------------------------------------------------------------------------------------------------------Function Prototypes------------------------------------------------------------------------------------------

// -----------------------------------------------------------------------------------------------------------Localization & Navigation

void determineOptimalPath(void); 
void followOptimalPath(void);
void getToStart(void);
void i1downAroundToA4(void);
void i4upAroundToi1(void);
void updatePos(int nextPos);

// -----------------------------------------------------------------------------------------------------------IR Sensor


void sampleLine(void *par);

// -----------------------------------------------------------------------------------------------------------Ping))

void detectBlockage(void *pingC);
void detectHooman(void *PingLeft);
void detectHoomanOnRight(void *pingRight);

//  -----------------------------------------------------------------------------------------------------------Servo


void crossIntersection(void);
void crossIntersectionFindHooman(void);
void followLine(void);
void followLineToIntersection(void);
void followLineFindHooman(void);
void followLineFindBlockage(void);
void followLineFindBlockageOrHooman(void);
void moveForward(void);
void stop(void);
void turnAround(void);
void turnLeft(void);
void turnRight(void);

// -----------------------------------------------------------------------------------------------------------RPi
void signalPi(void);

// -----------------------------------------------------------------------------------------------------------Sword
void enGarde(void* par);

// -----------------------------------------------------------------------------------------------------------LCD
void setupLCD(void* par);

// -----------------------------------------------------------------------------------------------------------SETUP
void letsAGo(void);

// ---------------------------------------------------------------------------------------------------------------------I/O Interfacing--------------------------------------------------------------------------------------------------
                                              
// -----------------------------------------------------------------------------------------------------------Localization - N/A

// -----------------------------------------------------------------------------------------------------------IR Sensor
int IRSensorHighPin = 6, IRSensorLowPin = 3;

// -----------------------------------------------------------------------------------------------------------Ping)))
int pingL = 1, pingC = 2, pingR = 0;

// -----------------------------------------------------------------------------------------------------------Servo
int servoL = 14, servoR = 15;

// -----------------------------------------------------------------------------------------------------------RPi
int triggerCameraPin = 9, enemyDetectedPin = 10, friendlyDetectedPin = 11;

// -----------------------------------------------------------------------------------------------------------Sword
int swordPin = 17;

// -----------------------------------------------------------------------------------------------------------LCD
int lcdPin = 12;
serial *lcd;


// ---------------------------------------------------------------------------------------------------------------------Global Variables---------------------------------------------------------------------------------------------------

// -----------------------------------------------------------------------------------------------------------Localization
volatile int currHead, prevPos, blockage, path;

// -----------------------------------------------------------------------------------------------------------IR Sensor
volatile int IRLine, intersectionDetected;
unsigned int IRStack[40+40];

// -----------------------------------------------------------------------------------------------------------Ping)))
volatile int blockageDetected = 0, hoomanDetected = 0, aGlobalFlag = 0;;
volatile int friendliesFound[4], friendlyDetected = 0, enemiesFound[4], enemyDetected = 0; 
unsigned int blockageStack[40+25], hoomanStack[40+25], hoomanRightStack[40+25];
int blockPingCog, hoomanPingCog, rightHoomanPingCog;
int pingtocam = 800;

// -----------------------------------------------------------------------------------------------------------Servo - N/A

int camPauseTime = 550;

// -----------------------------------------------------------------------------------------------------------RPi
volatile int enemyDetected, friendlyDetected, checkAfterTurnAround = 0, aFlag = 0;

// -----------------------------------------------------------------------------------------------------------Sword
volatile int swing = 0;
unsigned int swordStack[40+25];

// -----------------------------------------------------------------------------------------------------------LCD
unsigned int LCDStack[40+40];
const int ON = 22;
const int backlight = 17;
const int CLR = 12;
const int CRT = 13;

// ---------------------------------------------------------------------------------------------------------------------MAIN FUNCTION-------------------------------------------------------------------------------------------------------
int main(){  
  letsAGo();
  getToStart();  
  determineOptimalPath();
  followOptimalPath();
  cogstart((void*)setupLCD, NULL, LCDStack, sizeof(LCDStack));
  stop();
}

// ---------------------------------------------------------------------------------------------------------------------Function Definitions------------------------------------------------------------------------------------------------

// -----------------------------------------------------------------------------------------------------------Localization & Navigation                   

void determineOptimalPath(void){
   crossIntersection();
   followLineToIntersection();
   crossIntersection();
   //At i1
   updatePos(12);
   blockPingCog = cogstart((void*)detectBlockage,(void*)pingC,blockageStack,sizeof(blockageStack));
   hoomanPingCog = cogstart((void*)detectHooman,(void*)pingL,hoomanStack,sizeof(hoomanStack));
   while(currHead == 12 || currHead == 13){
    rightHoomanPingCog = cogstart((void*)detectHoomanOnRight,(void*)pingR,hoomanRightStack,sizeof(hoomanRightStack));
    followLineFindBlockageOrHooman();
    if(blockageDetected == 1){
      cogstop(blockPingCog);
      path = 0;
      turnAround();
      updatePos(11);
      followLineToIntersection(); //Falls out of loop just as it reaches i1
    }
    else{
     cogstop(rightHoomanPingCog);
     checkAfterTurnAround=0;
     crossIntersection();
     rightHoomanPingCog = cogstart((void*)detectHoomanOnRight,(void*)pingR,hoomanRightStack,sizeof(hoomanRightStack));
     //At i2
     updatePos(13);
     followLineFindBlockageOrHooman();
     if(blockageDetected == 1){
       cogstop(blockPingCog);
       updatePos(12);
       path = 1;
       turnAround();
       followLineToIntersection();
       crossIntersection();
       //At i2
       updatePos(11);
       followLineFindHooman();
       //Falls out of loop just as it reaches i1
     }
     else{
      path=2;
      cogstop(rightHoomanPingCog);
      checkAfterTurnAround =0;
      crossIntersection();
      //At i3
      updatePos(14);
      //falls out of loop just as it passes i3         
    }          
   }
 }   
}

void followOptimalPath(void){
  if(path == 2){
    followLineFindHooman();
    crossIntersection();
    //At i4
    updatePos(15);
    blockPingCog = cogstart((void*)detectBlockage,(void*)pingC,blockageStack,sizeof(blockageStack));
    followLineFindBlockageOrHooman();
    if(blockageDetected == 1){
      cogstop(blockPingCog);
      updatePos(14);
      turnAround();
      followLineFindHooman();
      //At i4
      crossIntersection();
      updatePos(13);
      followLineFindHooman();
      crossIntersection();
      //At i3
      updatePos(12);
      followLineFindHooman();
      crossIntersection();
      //At i2
      updatePos(11);
      followLineFindHooman();
      //At i1
      updatePos(01);
      turnRight();
      i1downAroundToA4();
      //At A4
      updatePos(14);
      if(hoomanDetected == 1){
        signalPi();
        servo_speed(servoL,40);
        servo_speed(servoR,40);
        pause(900);
      }
      else{
        turnRight();
      }
      followLineToIntersection();
      crossIntersection();
      updatePos(24);
      //Dumps out just above i4
    }
  }
  else{
    //Picking up
    updatePos(01);
    turnRight();
    i1downAroundToA4();
    //At A4
    updatePos(14);
    if(hoomanDetected == 1){
        signalPi();
        servo_speed(servoL,40);
        servo_speed(servoR,40);
        pause(900);
      }
    else{
      turnRight();
    }      
    followLineToIntersection();
    //At i4
    updatePos(15);
    turnLeft();
    followLineFindHooman();
    turnAround();
    //At i5
    updatePos(14);
    followLineFindHooman();
    crossIntersection();
    //At i4
    updatePos(13);
    if(path == 1){
      blockageDetected = 0;
      blockPingCog = cogstart((void*)detectBlockage,(void*)pingC,blockageStack,sizeof(blockageStack));
      followLineFindBlockageOrHooman();
      if(blockageDetected == 1){
        cogstop(blockPingCog);
        updatePos(14);
        turnAround();
      }
    }      
    else if(path ==0){
      followLineFindHooman();
      crossIntersection();
      //At i3
      updatePos(12);
      blockageDetected = 0;
      blockPingCog = cogstart((void*)detectBlockage,(void*)pingC,blockageStack,sizeof(blockageStack));
      followLineFindBlockageOrHooman();
      if(blockageDetected == 1){
        cogstop(blockPingCog);
        turnAround();
        updatePos(13);
      }
      followLineFindHooman();
      crossIntersection();
      updatePos(14);        
    }              
    followLineFindHooman();
    //At i4
    updatePos(24);
    turnRight();
  }
  i4upAroundToi1();
  i1downAroundToA4();
  //---------------------------------------------------------------------------------TEST
  crossIntersectionFindHooman();
  updatePos(15);
  followLineToIntersection();
  crossIntersectionFindHooman();          
}   

void getToStart(void){
  followLineToIntersection();
  moveForward();
  pause(350);
  followLineToIntersection();
  // At i0
  currHead = 11;
  prevPos  = 10;      
}

void i1downAroundToA4(void){    //---------------------------------------------------TEST
  followLineToIntersection();
  turnRight();
  //At A1
  if(hoomanDetected == 1){
    signalPi();
  }
  updatePos(02);
  followLineToIntersection();
  //At A2
  crossIntersectionFindHooman();
  updatePos(03);
  followLineToIntersection(); 
  //At A3
  crossIntersectionFindHooman();
  updatePos(04);
  followLineToIntersection();
}

void i4upAroundToi1(void){     //---------------------------------------------------------TEST
  followLineToIntersection();
  //At B4
  turnRight();
  if(hoomanDetected == 1){
    signalPi();
  }
  updatePos(23);
  followLineToIntersection();
  //At B3
  crossIntersectionFindHooman();
  updatePos(22);
  followLineToIntersection();
  //At B2
  crossIntersectionFindHooman();
  updatePos(21);
  followLineToIntersection();
  //At B1
  if(hoomanDetected == 1){
    signalPi();
    servo_speed(servoL,40);
    servo_speed(servoR,40);
    pause(900);
  }
  else{
    turnRight();
  }  
  followLineToIntersection();
  crossIntersection();
  //At i1
  updatePos(01);
  //Dumps out just below i1
}

void updatePos(int nextPos){
  prevPos = currHead;
  currHead = nextPos;
  }    

// -----------------------------------------------------------------------------------------------------------IR Sensor

void sampleLine(void *par){
  while(1){
  set_directions(IRSensorHighPin,IRSensorLowPin,0b1111);
  set_outputs(IRSensorHighPin,IRSensorLowPin,0b1111);
  pause(10);
  set_directions(IRSensorHighPin,IRSensorLowPin,0b0000);
  pause(1);
  IRLine = get_states(IRSensorHighPin,IRSensorLowPin);
  if (IRLine == 0b1111){
    intersectionDetected = 1;
  }    
  else {
    intersectionDetected = 0;
  }
  pause(60);     
  }  
} 

// -----------------------------------------------------------------------------------------------------------Ping)))

void detectBlockage(void *pingCent){
  int aPin = (int)pingCent;
  int cmDist;
  while(1){
     cmDist = ping_cm(aPin);
     if (cmDist < 9){
       blockageDetected = 1;
       if(aGlobalFlag ==0){
         blockage = currHead;
         aGlobalFlag = 1;
       }       
     }
     else{
       blockageDetected = 0;
     }
     pause(50);              
  }   
}

void detectHooman(void *PingLeft){
  int pingPin = (int)PingLeft;
  int cmDisthoom;
  while(1){
    cmDisthoom = ping_cm(pingPin);      
    if (cmDisthoom < 15){
      hoomanDetected = 1;
      pause(100);
    }
    else{
      hoomanDetected = 0;
    }
    pause(50);      
  }      
}

void detectHoomanOnRight(void *pingRight){  //----------------------------------TESTED
  int anotherPingPin = (int)pingRight;
  int cmDistRight;
  int internalFlag = 0;
  checkAfterTurnAround = 0;
  while(blockageDetected == 0){
    pause(50);
    cmDistRight = ping_cm(anotherPingPin);
    pause(20);
    if(cmDistRight < 15){
      internalFlag = 1;
    }
    pause(50);
  }
  if(internalFlag == 1){
    checkAfterTurnAround = 1;
  }
}

// -----------------------------------------------------------------------------------------------------------Servo

void crossIntersection(void){
  while(intersectionDetected == 1){
    moveForward();
    pause(10);
  } 
}

void crossIntersectionFindHooman(void){//------------------------------------------TESTED
  while(intersectionDetected == 1){
    moveForward();
    if(hoomanDetected == 1){
      signalPi();
      }
      pause(20);
   }
}  
  
void followLine(void){
    switch(IRLine){
      case 0b1111:
      break;
    case 0b1000:
      servo_speed(servoL,10);
      servo_speed(servoR,-40);
      pause(100);
      break;  
    case 0b1100:
      servo_speed(servoL,20);
      servo_speed(servoR,-40);
      pause(100);
      break;
    case 0b1110:
      servo_speed(servoL,30);
      servo_speed(servoR,-40);
      pause(100);
      break;
    case 0b0100:
      servo_speed(servoL,35);
      servo_speed(servoR,-40);
      pause(100);
      break;
    case 0b1101:
      servo_speed(servoL,30);
      servo_speed(servoR,-40);
      pause(100);
      break;
    case 0b0110:
      servo_speed(servoL,40);
      servo_speed(servoR,-40);
      pause(100);
      break;
    case 0b1011:
      servo_speed(servoL,40);
      servo_speed(servoR,-30);
      pause(100);
      break;
    case 0b0010:
      servo_speed(servoL,40);
      servo_speed(servoR,-35);
      pause(100);
      break;
    case 0b0111:
      servo_speed(servoL,30);
      servo_speed(servoR,-20);
      pause(100);
      break;
    case 0b0011: 
      servo_speed(servoL,40);
      servo_speed(servoR,-20);
      pause(100);
      break;
    case 0b0001: 
      servo_speed(servoL,40);
      servo_speed(servoR,-10);
      pause(100);
      break;
    case 0b0000:
      stop();
      break;
    }      
}

void followLineToIntersection(void){
  while(intersectionDetected == 0){
         followLine();
  }
}

void followLineFindHooman(void){
  while(intersectionDetected == 0 && hoomanDetected == 0){
    followLine();
  }
  if(hoomanDetected == 1){
    signalPi();
    followLineToIntersection();
  }
}

void followLineFindBlockageOrHooman(void){
  while(intersectionDetected == 0 &&(hoomanDetected == 0 && blockageDetected == 0)){
    followLine();
  }
  if(hoomanDetected == 1){
    signalPi();
    followLineFindBlockage();
  }      
}

void followLineFindBlockage(void){
  while(intersectionDetected == 0 && blockageDetected == 0){
    followLine();
  }  
}          

void moveForward(void){
  servo_speed(servoL,40);
  servo_speed(servoR,-40);
  pause(20);
}

void stop(void){
  servo_speed(servoL,0);
  servo_speed(servoR,0);
}

void turnAround(void){
  servo_speed(servoL,40);
  servo_speed(servoR,40);
  pause(2200);//---------------------------------------------------------------CONFIRM IN FULL TEST
  if(checkAfterTurnAround == 1){
    aFlag = 1;
    signalPi();
    cogstop(rightHoomanPingCog);
    checkAfterTurnAround = 0;
  }
}  

void turnLeft(void){
  moveForward();
  pause(420);//---------------------------------------------------------------CONFIRM IN FULL TEST
  servo_speed(servoL,-40);
  servo_speed(servoR,-40);
  pause(1150);//---------------------------------------------------------------CONFIRM IN FULL TEST
}  

void turnRight(void){
  moveForward();
  pause(420);//----------------------------------------------------------------CONFIRM IN FULL TEST
  servo_speed(servoL,40);
  servo_speed(servoR,40);
  pause(1000);//-------------------------------------------------------------CONFIRM IN FULL TEST
}  

// -----------------------------------------------------------------------------------------------------------RPi

void signalPi(void){
  moveForward();
    if(aFlag == 0){
      pause(camPauseTime);
    }
    else if(aFlag == 1){
      aFlag = 0;
    }    
  stop();
  pause(500);
  set_output(triggerCameraPin,1);
  pause(600);  
  int flavorOfPie = 0; 
  while(flavorOfPie == 0){
    flavorOfPie = get_states(friendlyDetectedPin,enemyDetectedPin);  
    }
  set_output(triggerCameraPin,0);
  
  if(flavorOfPie == 0b10){
        set_output(27,1);
        if((currHead == friendliesFound[0] ||currHead == friendliesFound[1]) || (currHead == friendliesFound[2] || currHead == friendliesFound[3])){
        }          
        else{
          friendliesFound[friendlyDetected] = currHead;
          friendlyDetected = friendlyDetected + 1;
        }          
        pause(2000);
        set_output(27,0);
        }
   else if(flavorOfPie==0b01){
        swing = 1;
        set_output(26,1);
        enemiesFound[enemyDetected] = currHead;
        enemyDetected = enemyDetected + 1;
        pause(2500);
        set_output(26,0);
  }            
}  

// -----------------------------------------------------------------------------------------------------------Sword

void enGarde(void *par){
  servo_angle(swordPin,300);
  while(1){
    if (swing == 1){
      servo_angle(swordPin, 1700); //swings
      pause(500);
      servo_angle(swordPin,300);
      swing = 0;
    }
    servo_angle(swordPin,300);
    pause(250);
  }        
}  

// -----------------------------------------------------------------------------------------------------------LCD

void setupLCD(void *par){
  lcd = serial_open(lcdPin, lcdPin, 0, 19200);
  writeChar(lcd, ON);
  writeChar(lcd, backlight);
  writeChar(lcd, CLR);
  pause(10);
  dprint(lcd, "CLASSIFIED");
  writeChar(lcd,CRT);
  dprint(lcd, "Survey Report");
  pause(7850);
  writeChar(lcd,CLR);
  dprint(lcd, "%02d Frnd Locd", friendlyDetected);
  writeChar(lcd,CRT);
  pause(1000);
  dprint(lcd, "%02d En Elim", enemyDetected);
  pause(3000); 
  writeChar(lcd,CLR);
  dprint(lcd,"Send help to:");
  writeChar(lcd, CRT);
  dprint(lcd, "%02d, %02d, %02d, %02d", friendliesFound[0], friendliesFound[1], friendliesFound[2], friendliesFound[3]);
  pause(3000);
  writeChar(lcd,CLR);
  dprint(lcd, "Send Cleanup:");
  writeChar(lcd,CRT);
  dprint(lcd, "%02d, %02d, %02d, %02d", enemiesFound[0], enemiesFound[1], enemiesFound[2], enemiesFound[3]);
  pause(3000);
  writeChar(lcd,CLR);
  dprint(lcd, "Blockage at:");
  writeChar(lcd,CRT);
  dprint(lcd, "%02d", blockage);
     
}
 
// -----------------------------------------------------------------------------------------------------------SETUP
void letsAGo(void){
  pause(50);
  set_directions(friendlyDetectedPin,triggerCameraPin,0b001);
  set_output(triggerCameraPin,0);
  set_directions(27,26,0b11);
  cogstart((void*)sampleLine, NULL, IRStack, sizeof(IRStack));
  cogstart((void*)enGarde, NULL, swordStack, sizeof(swordStack));
  }