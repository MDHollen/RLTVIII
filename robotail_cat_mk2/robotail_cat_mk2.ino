/**************************************************************************/
/*!
    @file     robotail_cat_mk1
    @author   Calvin Murphy and Matthew Hollenbeck
    @license  BSD (see license.txt)
    This is an example for the Adafruit MMA8451 Accel breakout board
    ----> https://www.adafruit.com/products/2019
    Adafruit invests time and resources providing this open source code,
    please support Adafruit and open-source hardware by purchasing
    products from Adafruit!
    @section  HISTORY
    v1.0  - First release
*/
/**************************************************************************/

#include <Wire.h>
#include "Timer.h"
#include <Adafruit_MMA8451.h>
#include <Adafruit_Sensor.h>

#include <Servo.h>

Servo servo1; //Tip LRC
Servo servo2; //Mid FBC
Servo servo3; //Base LRC
Servo servo4; //Tip FBC

Adafruit_MMA8451 mma = Adafruit_MMA8451();

Timer t;

//define possible servo speeds using enum:
enum servoSpeed {
  maxBR = 0, //max forward/right
  servoStop = 93,
  maxFL = 180 //max backward/left
};

//set up initial accel recording (for calibration)
float initXaccel;
float initYaccel;
float initZaccel;

//record previous acceleration values:
float prevXaccel;
float prevYaccel;
float prevZaccel;

//set up user motion tracking
float userSpeed;
float userFlatSpeed;
float userXvel;
float userYvel;
float userZvel;

//set up loop time tracker (milliseconds)
unsigned long loopTime;
unsigned long prevTime;

//define possible states using enums:
enum stateTypeEnum {
  standing,
  leaning,
  walking,
  sprinting
};

enum stateDirEnum {
  forward,
  right,
  backward,
  left
};

enum stateVertEnum {
  level,
  rising,
  falling
};

stateTypeEnum stateType = standing;
stateDirEnum stateDir = forward;
stateVertEnum stateVert = level;
//NOTE: If the stateType is 'standing', direction will not matter.

//counting intervals between twitches
bool isTwitching = false;
int twitchCount = 0;
int twitchTime;

//Keeping track of tail positions
int tipLRCPos = 2;
int midFBCPos = 2;
int baseLRCPos = 2;
int tipFBCPos = 2;

void setup(void) 
{
  servo1.attach(0); //Tip LRC
  servo2.attach(1); //Mid FBC
  servo3.attach(2); //Base LRC
  servo4.attach(3); //Tip FBC

  //Calibrate Servo Positions
  center_servos();

  //Find accelerometer
  do {
    Serial.println("Searching for accelerometer...");
  } while (!mma.begin());
  Serial.println("MMA8451 found!");
  mma.setRange(MMA8451_RANGE_2_G);

  //Get accelerometer event
  sensors_event_t event;
  mma.getEvent(&event);

  //Set initial acceleration
  prevXaccel = event.acceleration.x;
  prevYaccel = event.acceleration.y;
  prevZaccel = event.acceleration.z;

  //Initialize velocity trackers
  userSpeed = 0;
  userFlatSpeed = 0;
  userXvel = 0;
  userYvel = 0;
  userZvel = 0;

  //Set initial loop time to 0
  loopTime = 0;
  prevTime = millis();

  //Set twitch wait time to somewhere between 5 and 15 seconds.
  twitchTime = 5000 + 10000 * rand();
}

void loop()
{
  /* Get a new sensor event */
  sensors_event_t event;
  mma.getEvent(&event);

  //Store accelerations (m/s^2) from this moment for later use.
  int xAccel = event.acceleration.x;
  int yAccel = event.acceleration.y;
  int zAccel = event.acceleration.z;

  //Determine user speed (m/s):
  userXvel += xAccel * loopTime / 1000;
  userYvel += yAccel * loopTime / 1000;
  userZvel += zAccel * loopTime / 1000;

  userSpeed = sqrt(pow(userXvel, 2) + pow(userYvel, 2) + pow(userZvel, 2));
  userFlatSpeed = sqrt(pow(userXvel, 2) + pow(userYvel, 2));

  //Determine state type
  //State priority: sprinting, walking, leaning, standing
  if (abs(userFlatSpeed) > 2) {
    stateType = sprinting;
  }
  else if (abs(userFlatSpeed) > 1) {
    stateType = walking;
  }
  else if ((abs(xAccel) >= 9 && abs(yAccel) <= 1 && zAccel < 0) || (abs(xAccel) <= 1 && abs(yAccel) >= 9 && zAccel < 0)) {
    stateType = leaning;
  }
  else {
    stateType = standing;
  }

  //Determine state direction
  //(assuming standard cartesian coordinate system (forward/backward is +/- Y, right/left is +/- X)
  if (abs(userXvel) >= abs(userYvel)) { //sideways movement
    if(userXvel >= 0) {
      stateDir = right;
    }
    else {
      stateDir = left;
    }
  }
  else { //forward/backward movement
    if(userYvel >= 0) {
      stateDir = forward;
    }
    else {
      stateDir = backward;
    }
  }

  //Determine vertical state
  //(assuming standard cartesian coordinate system (up is +Z, down is -Z)
  if(abs(userZvel) > 1) { //if user is moving vertically
    if(userZvel >= 0) {
      stateVert = rising;
    }
    else {
      stateVert = falling;
    }
  }
  else { //if user is not moving vertically
    stateVert = level;
  }

  //Perform operations based on user state.

  //STANDING STATE
  if (stateType == standing) {
    switch (stateVert) {
      case level: //swing tail in basic cycle
        if (midFBCPos > 0) {
          move_servo(2, maxBR, 300);
        }
        else {
          move_servo(2, maxFL, 300);
        }
        break;
      case rising: //move tail in jumping position
        move_servo_to(2, -300);
        move_servo_to(4, 200);
        break;
      case falling: //move tail in falling position
        move_servo_to(2, 300);
        move_servo_to(4, -200);
        break;
      default:
        break;
    }
  }

  //LEANING STATE
  if (stateType == leaning) {
    switch (stateDir) {
      case forward:
        move_servo_to(2, 400);
        break;
      case right:
        move_servo_to(3, 300);
        break;
      case left:
        move_servo_to(3, -300);
        break;
      case backward:
        move_servo_to(2, -400);
        break;
      default:
        break;
    }
  }

  //WALKING STATE
  if (stateType == walking) {
    switch (stateDir) {
      case forward: //if user is walking forward
        if(stateVert == rising) {
          move_servo_to(2, 400);
          move_servo_to(4, -500);
        }
        else {
          move_servo_to(2, 300);
          move_servo_to(4, -400);
        }
        move_servo_to(4, 0);
        break;
      case right: //if user is sidestepping right
        if(stateVert == rising) {
          move_servo_to(3, -300);
          move_servo_to(1, -600);
        }
        else {
          move_servo_to(3, -200);
          move_servo_to(1, -400);
        }
        move_servo_to(4, 0);
        break;
      case left: //if user is sidestepping left
        if(stateVert == rising) {
          move_servo_to(3, 300);
          move_servo_to(1, 600);
        }
        else {
          move_servo_to(3, 200);
          move_servo_to(1, 400);
        }
        move_servo_to(4, 0);
        break;
      case backward: //if user is walking backward
        if(stateVert == rising) {
          move_servo_to(2, -300);
          move_servo_to(4, 400);
        }
        else {
          move_servo_to(2, -200);
          move_servo_to(4, 300);
        }
        move_servo_to(4, 0);
        break;
      default:
        break;
    }
  }

  //SPRINTING STATE (Currently just a copy of walking state)
  if (stateType == sprinting) {
    switch (stateDir) {
      case forward: //if user is walking forward
        if(stateVert == rising) {
          move_servo_to(2, 400);
          move_servo_to(4, -500);
        }
        else {
          move_servo_to(2, 300);
          move_servo_to(4, -400);
        }
        move_servo_to(4, 0);
        break;
      case right: //if user is sidestepping right
        if(stateVert == rising) {
          move_servo_to(3, -300);
          move_servo_to(1, -600);
        }
        else {
          move_servo_to(3, -200);
          move_servo_to(1, -400);
        }
        move_servo_to(4, 0);
        break;
      case left: //if user is sidestepping left
        if(stateVert == rising) {
          move_servo_to(3, 300);
          move_servo_to(1, 600);
        }
        else {
          move_servo_to(3, 200);
          move_servo_to(1, 400);
        }
        move_servo_to(4, 0);
        break;
      case backward: //if user is walking backward
        if(stateVert == rising) {
          move_servo_to(2, -300);
          move_servo_to(4, 400);
        }
        else {
          move_servo_to(2, -200);
          move_servo_to(4, 300);
        }
        move_servo_to(4, 0);
        break;
      default:
        break;
    }
  }

  //Activate twitch
  if (twitchCount >= twitchTime) {
    int randy = random(1, 4); //1 is tipLRC twitch left, 2 is tipLRC twitch right, 3 is tipFBC backward, 4 is tipFBC forward.
    int twitchPos = random(400, 600);
    switch (randy) {
      case 1: //tipLRC twitch left
        move_servo_to(1, -twitchPos);
        move_servo_to(1, 0); //centers tail tip after twitch
        break;
      case 2: //tipLRC twitch right
        move_servo_to(1, twitchPos);
        move_servo_to(1, 0); //centers tail tip after twitch
        break;
      case 3: //tipFBC twitch backward
        move_servo_to(4, -twitchPos);
        move_servo_to(4, 0); //centers tail tip after twitch
        break;
      case 4: //tipFBC twitch forward
        move_servo_to(4, 500);
        move_servo_to(4, 0); //centers tail tip after twitch
        break;
    }
    twitchCount = 0;
    twitchTime = 5000 + 10000 * rand();
  }

  //Store values of previous accelerations for future use.
  prevXaccel = event.acceleration.x;
  prevYaccel = event.acceleration.y;
  prevZaccel = event.acceleration.z;

  twitchCount++;
  
  //delay(1500);

  loopTime = millis() - prevTime;
  prevTime = millis();
}

//Generic method to move a particular servo for a certain amount of time, at a certain speed, then stop.
void move_servo(int servoNum, servoSpeed speedSet, int milliseconds) {
  int dir_modifier = 1; //dir_modifier is 1 if moving right, -1 if moving left.
  if (speedSet = maxFL) {
    dir_modifier = -1;
  }
  switch (servoNum) {
    case 1:
      servo1.write(speedSet);
      delay(milliseconds);
      servo1.write(93);
      tipLRCPos += milliseconds * dir_modifier;
      break;
    case 2:
      servo2.write(speedSet);
      delay(milliseconds);
      servo2.write(93);
      midFBCPos += milliseconds * dir_modifier;
      break;
    case 3:
      servo3.write(speedSet);
      delay(milliseconds);
      servo3.write(93);
      baseLRCPos += milliseconds * dir_modifier;
      break;
    case 4:
      servo4.write(speedSet);
      delay(milliseconds);
      servo4.write(93);
      tipFBCPos += milliseconds * dir_modifier;
      break;
    default:
      break;
  }
}

//Generic method to move a particular servo from its current position to a new position.
void move_servo_to(int servoNum, int targetPos) {
  int servoPos;
  int posDiff;
  switch (servoNum) {
    case 1:
      servoPos = tipLRCPos;
      posDiff = targetPos - servoPos;
      if (posDiff > 0) {
        move_servo(servoNum, maxBR, posDiff);
      }
      else if(posDiff < 0) {
        move_servo(servoNum, maxFL, posDiff);
      }
      else {
        move_servo(servoNum, servoStop, 0);
      }
      break;
    case 2:
      servoPos = midFBCPos;
      posDiff = targetPos - servoPos;
      if (posDiff > 0) {
        move_servo(servoNum, maxBR, posDiff);
      }
      else if(posDiff < 0) {
        move_servo(servoNum, maxFL, posDiff);
      }
      else {
        move_servo(servoNum, servoStop, 0);
      }
      break;
    case 3:
      servoPos = baseLRCPos;
      posDiff = targetPos - servoPos;
      if (posDiff > 0) {
        move_servo(servoNum, maxBR, posDiff);
      }
      else if(posDiff < 0) {
        move_servo(servoNum, maxFL, posDiff);
      }
      else {
        move_servo(servoNum, servoStop, 0);
      }
      break;
    case 4:
      servoPos = tipFBCPos;
      posDiff = targetPos - servoPos;
      if (posDiff > 0) {
        move_servo(servoNum, maxBR, posDiff);
      }
      else if(posDiff < 0) {
        move_servo(servoNum, maxFL, posDiff);
      }
      else {
        move_servo(servoNum, servoStop, 0);
      }
      break;
    default:
      break;
  }  
}

//Calibration method moves servos to their limit, then moves them back to the center.
//Consider rewriting this to be more dance-like?
//Alternatively, consider rewriting this to move all servos to limits at the same time.
void center_servos() {
  //Center tip LRC
  move_servo(1, maxBR, 1100);
  move_servo(1, maxFL, 600);
  tipLRCPos = 0; //tipLRC is centered
  delay(500);

  //Center mid FBC
  move_servo(2, maxBR, 800);
  move_servo(2, maxFL, 300);
  midFBCPos = 0; //midFBC is centered
  delay(500);

  //Center base LRC
  move_servo(3, maxBR, 600);
  move_servo(3, maxFL, 300);
  baseLRCPos = 0; //baseLRC is centered
  delay(500);

  //Center tip FBC
  move_servo(4, maxBR, 1100);
  move_servo(4, maxFL, 450);
  tipFBCPos = 0; //tipFBC is centered
  delay(1500);

}
