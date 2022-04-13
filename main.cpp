/*
 * Lab 03
 * @author Daphine Kirubi
 * @date 03/20
 * this code makes a robot adopts a random wander algorithm to 
 * find two desired object (nemo and Dory).It uses wall following 
 * and line following strategies to locate them. It can also avoid 
 * obstacles and continue with the search autonomously
 */ 

#include <Arduino.h>
#include <wpi-32u4-lib.h>

#include <IRdecoder.h>
#include <ir_codes.h>

#include <Rangefinder.h>
#include <Rangefinder2.h>
#include <Chassis.h>

// Sets up the IR receiver/decoder object
const uint8_t IR_DETECTOR_PIN = 1;
IRDecoder decoder(IR_DETECTOR_PIN);

Rangefinder rangefinder(11, 4);     // create instance of sonar echo pin 11, trigger pin 4
Rangefinder2 rangefinder2(2,3);   //ech then trig

Chassis chassis(6.0, 1440, 12.5);   // Declares a chassis object with nominal dimensions

const int LED_PIN_EX1 = 12;
const int LED_PIN_EX2 = 18;  // shown as pin A0 on the Romi board
const float K_p = 0.15 ;
float baseSpeed = 10;        // Defines the speed of the robot
float distance = -1;         // Records the rangefinder distance from obstacles (in cm)
float rangeFinder2Distance = -1 ;  //records the rangefinder2 distance (in cm)
float warnDistance = 30;     // The warning distance (could be adjusted)
float dangDistance = 15;     // The danger distance (could be adjusted)
uint16_t  whiteThreshold = 100;

void setLED(int pin, bool value)
{
  Serial.println("setLED()");
  digitalWrite(pin, value);
}

void readSonar(){                         //read ultrasonic range sensor   
  distance = rangefinder.getDistance();   //distance in cm
  rangeFinder2Distance = rangefinder2.getDistance(); 
}


// Defines the robot states
// TODO3: Define 3 new states ROBOT_WANDER, ROBOT_AVOID, ROBOT_WALL_FOLLOWING
enum ROBOT_STATE {ROBOT_IDLE,ROBOT_WANDER,ROBOT_AVOID,ROBOT_WALL_FOLLOWING, TARGET_FOUND};
ROBOT_STATE robotState = ROBOT_IDLE;

// idle() stops the motors
void idle(void)
{
  Serial.println("idle()");
  setLED(LED_PIN_EX1, LOW);
  setLED(LED_PIN_EX2, LOW);

  chassis.idle();
  //set state to idle
  robotState = ROBOT_IDLE;
  Serial.println("/idle()");
}

/*
 * This is the standard setup function that is called when the board is rebooted
 * It is used to initialize anything that needs to be done once.
 */
void setup() 
{
  // This will initialize the Serial at a baud rate of 115200 for prints
  // Be sure to set your Serial Monitor appropriately
  Serial.begin(115200);
  chassis.init();

  chassis.setMotorPIDcoeffs(5,0.3);

  pinMode(LED_PIN_EX1, OUTPUT);
  pinMode(LED_PIN_EX2, OUTPUT);
  idle();              // make sure the robot is stopped upon startup
  decoder.init();      // Initializes the IR decoder
  rangefinder.init();  // Initializes the rangefinder
  rangefinder2.init();

  Serial.println("/setup()");
}
//this function looks for dory by finding an intersection
 bool findingDory(uint16_t threshold){     
  static bool prevIntersection = false ;
  bool retVal = false;

  int16_t leftADC = analogRead(LEFT_LINE_SENSE);
  int16_t rightADC = analogRead(RIGHT_LINE_SENSE);

  Serial.print(leftADC);
  Serial.print('\t'); //print a TAB character to make the output prettier
  Serial.println(rightADC);

  if(leftADC <= threshold || rightADC <= threshold){
    prevIntersection = true ;
    retVal = true;
  }
  prevIntersection = false ;
  return retVal ;
} 
// TODO5: Modify this function to achieve the following:
//       (1) the process stays in the while loop repeatedly if the motion is NOT completed 
//           or if the rangefinder readings are further than the warnDistance
//       (2) if the rangefinder readings < the warnDistance, change the robot state 
//           ROBOT_AVOID and exit the while loop
// hint: you would want to call readSonar();
void fancyLights (void){

 robotState = TARGET_FOUND;
 Serial.println("FOUND DORY");
       
 for(int a = 1 ;a < 5 ; a++){      //lights flash.  Nemo  is found
    
    setLED(LED_PIN_EX1,LOW); 
    setLED(LED_PIN_EX2,LOW);
    delay(500);

    setLED(LED_PIN_EX1,HIGH); 
    setLED(LED_PIN_EX2,HIGH);
    delay(500);
   }
 robotState = ROBOT_IDLE;
   
}

void waitUntilDoneOrBlocked(){
  bool completed = chassis.checkMotionComplete();   
  readSonar();

  while(!completed && distance > warnDistance && !findingDory(whiteThreshold))
  {
    Serial.println("loop ()");    
    completed = chassis.checkMotionComplete();  
    Serial.println(completed); 
    delay(500);
    readSonar();
    if (distance < warnDistance){
      robotState = ROBOT_AVOID;
      break;
    }

    if(findingDory(whiteThreshold)){
      fancyLights();
      break;
    }
    Serial.println("/loop"); 
  }
}


// TODO6: Call waitUntilDoneOrBlocked after driveFor
void driveUntilBlocked(float dist, float speed) // A helper function to drive a set distance
{
  Serial.println("drive...");
  chassis.driveFor(dist,speed);         // moves for a certain distance
  waitUntilDoneOrBlocked();
}

// TODO7: Call waitUntilDoneOrBlocked after turnFor
void turnUntilBlocked(float ang, float speed) // A helper function to turn a set angle
{
  Serial.println("turn...");
  chassis.turnFor(ang,speed);
  waitUntilDoneOrBlocked();
}



void randomWander(){      //function to move robot random forward and turn
  setLED(LED_PIN_EX1, HIGH);
  setLED(LED_PIN_EX2, HIGH);
  
  Serial.println("randomWander()");
  int randNumber = random(millis());    //generate a random number
  int dist = randNumber % 10;       //limit the values between 0 and 50 cm
  int angle = randNumber % 120;          //limit the values between 0 and 120 degrees
 // int speed = randNumber % 50 ;
  
   Serial.print(dist);
   Serial.print(" cm\t");
   Serial.print(angle);
   Serial.println(" deg\t");
   
  driveUntilBlocked(dist,baseSpeed);
  turnUntilBlocked(angle,baseSpeed);
 
}

void robotAvoidObstacles(void){
  readSonar();

  if (distance < dangDistance){           //when it's too close stop 
        Serial.println("STOP!!!");
        idle();
        }

   if(warnDistance > distance && distance > dangDistance ){    //avoid robot
    Serial.println("obstacle ()");
    float reverseDistance = -10;
    Serial.println(reverseDistance);
   
    chassis.driveFor(reverseDistance,baseSpeed);
    while (!chassis.checkMotionComplete()) {
      delay(50);
    }
    chassis.turnFor(90,baseSpeed);
    while (!chassis.checkMotionComplete()) {
      delay(50);
    }
    if(chassis.checkMotionComplete()) robotState = ROBOT_WANDER ;   //change to listen for keypres  
    } 
     
}
void wallFollowing (void) {
   
   setLED(LED_PIN_EX1,HIGH); 

    readSonar();
   
    float error = (rangeFinder2Distance  - 30);
    float turnEffort = error * K_p;
    
    Serial.print("range 2 :" );
    Serial.println(rangeFinder2Distance);
    delay(100);
    Serial.print("range 1 :" );
    Serial.println(distance);

    while(!findingDory(whiteThreshold)){ 

    float error = (rangeFinder2Distance - 30 );
    turnEffort = error * K_p ;
    readSonar();

    if (findingDory(whiteThreshold)) {                   //break when Dory is found 
        robotState = TARGET_FOUND ;
        break;
      }

    if (rangeFinder2Distance < 10 ){     // heading into wall
      Serial.println("crash ()");
      chassis.turnFor(20,20,true);
      chassis.driveFor(5,20,true);
      break ;
      
    }

    if (distance <= 15 && rangeFinder2Distance <= 30){          // wall infront ; turn left 
        readSonar();
        Serial.println("wall infront ()");
        chassis.turnFor(90,20,true);
        break;
    }

    else if (rangeFinder2Distance > 10 && rangeFinder2Distance <= 20 ){    //desired distance

      Serial.println("straight ()");
      chassis.setTwist(20,turnEffort);
      break ;
    }

    else if (rangeFinder2Distance > 20 && rangeFinder2Distance <= 30){     //away from wall 
      readSonar();
      Serial.println("away from wall()") ;
      chassis.turnFor(-20,20,true);
      chassis.driveFor(5,20,true);
      chassis.turnFor(20,20,true);
      break ;

    }
    else {       
      Serial.println("lost ()");    //lost
      Serial.print("range 2 :" );
      Serial.println(rangeFinder2Distance);     
      delay (100);                       
      
  
    }
  
  } 
}
    

// Handles a key press on the IR remote
void handleKeyPress(int16_t keyPress)
{
  Serial.println("Key: " + String(keyPress));

  if(keyPress == ENTER_SAVE) idle();   // Set up the emergency stop button
 
  switch(robotState)
  {
    case ROBOT_IDLE:
    if(keyPress == NUM_1){         // TODO 9
      robotState = ROBOT_WANDER ;
    }  
    if(keyPress == NUM_2) {
      robotState = ROBOT_WALL_FOLLOWING;
    }
      break;


    // TODO4: respond to speed +/- commands (when in ROBOT_WANDER or ROBOT_WALL_FOLLOWING state)
    case ROBOT_WANDER:
    if(keyPress == VOLplus){
      baseSpeed = baseSpeed + 5;
    }
    if(keyPress == VOLminus){
      baseSpeed = baseSpeed - 5;
    }
      break;

    case ROBOT_WALL_FOLLOWING:
    if(keyPress == VOLplus){
      baseSpeed = baseSpeed + 5;
    }
    if(keyPress == VOLminus){
      baseSpeed = baseSpeed - 5;
    }
    default:
      break;
  }
}
/*
 * The main loop for the program. The loop function is repeatedly called
 * after setup() is complete.
 */
void loop()
{
  // Checks for a key press on the remote
  int16_t keyPress = decoder.getKeyCode();
  if(keyPress >= 0) handleKeyPress(keyPress);
  // A basic state machine
  switch(robotState)
  {

    case ROBOT_WANDER: 
      randomWander();
      break;
    case ROBOT_AVOID :
      setLED(LED_PIN_EX1,LOW);            //turn led off
      robotAvoidObstacles();
      break;
    case ROBOT_WALL_FOLLOWING:
      wallFollowing();
      break;
    default:
      break;
  }
}

