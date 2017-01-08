/**************************************************************************/
/*!
    @file     Adafruit_MMA8451.h
    @author   K. Townsend (Adafruit Industries)
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
#include <Adafruit_MMA8451.h>
#include <Adafruit_Sensor.h>

#include <Servo.h>

Servo servo1;
Servo servo2;
Servo servo3;
Servo servo4;

Adafruit_MMA8451 mma = Adafruit_MMA8451();

//record previous acceleration values;
float previousXacceleration;
float previousYacceleration;
float previousZacceleration;

//define possible states
int leaningforward;
int movingleft;
int movingright;
int straightforwardmovement;
int walking = false;

//counting intervals between twitches
float checkstilltwitch = 1;
int checkssincetwitch = 0;

//For testing the walk indicator
int counter1 = 0;

//Keeping track of tail positions
int tipLRC;
int midFBM;
int baseLRC;
int tipFBM;

float rando;

void setup(void) 
{
  servo1.attach(0); //Tip LRC
  servo2.attach(1); //Mid FBC
  servo3.attach(2); //Base LRC
  servo4.attach(3); //Tip FRC

  //Center tip LRC
  servo1.write(0); //Move right
  delay(1100);
  servo1.write(180); //Move left
  delay(600);
  servo1.write(93); //Stop moving
  tipLRC = 3;

  delay(500);

  //Center mid FBC
  servo2.write(0); //Move backward
  delay(800);
  servo2.write(180); //Move forwards
  delay(300);
  servo2.write(93); //Stop moving
  midFBM = 3;

  delay(500);

  //Center base LRC
  servo3.write(0); //Move right
  delay(600);
  servo3.write(180); //Move left
  delay(300);
  servo3.write(93); //Stop moving
  baseLRC = 3;

  delay(500);

  //Center tip FBM
  servo4.write(0); //Move fowards
  delay(1100);
  servo4.write(180); //Move backwards
  delay(450);
  servo4.write(93); //Stop moving
  tipFBM = 3;

  delay(1500);
  
  //pinMode(13, OUTPUT); //Activate when you want to monitor movements on the computer.
  
  //Serial.begin(9600); //For testing walk indicator.
  
  Serial.println("Adafruit MMA8451 test!");
  

  if (! mma.begin()) 
  {
    Serial.println("Couldnt start");
    while (1);
  }
  //Serial.println("MMA8451 found!");
  
  mma.setRange(MMA8451_RANGE_2_G);
  
  //Serial.print("Range = "); Serial.print(2 << mma.getRange());  
  //Serial.println("G");

  sensors_event_t event; 
  mma.getEvent(&event);

  previousXacceleration = event.acceleration.x;
  previousYacceleration = event.acceleration.y;
  previousZacceleration = event.acceleration.z;
  
}

void loop()
{

  /* Get a new sensor event */
  sensors_event_t event;
  mma.getEvent(&event);

  //Store accelerations from this moment for later use.

  /* Display the results (acceleration is measured in m/s^2) */
  //Serial.print("X: \t"); Serial.print(event.acceleration.x); Serial.print("\t");
  //Serial.print("Y: \t"); Serial.print(event.acceleration.y); Serial.print("\t");
  //Serial.print("Z: \t"); Serial.print(event.acceleration.z); Serial.print("\t");
  //Serial.println("m/s^2 ");

/*
  //Check to see if user is side-stepping or turning.
  if (event.acceleration.y < -1) //Moving right
  {
    movingleft = false;
    movingright = true;
    straightforwardmovement = false;
    if (baseLRC == 1)
    {
      servo3.write(0); //Move right
      delay(600);
      servo3.write(93); //Stop moving
      baseLRC = 2;
    }
    else if (baseLRC == 3)
    {
      servo3.write(0); //Move right
      delay(300);
      servo3.write(93); //Stop moving
      baseLRC = 2;
    }
  }
  else if (event.acceleration.y > +1) //Moving left
  {
    movingright = false;
    movingleft = true;
    straightforwardmovement = false;
    if (baseLRC == 2)
    {
      servo3.write(180); //Move left
      delay(600);
      servo3.write(93); //Stop moving
      baseLRC = 1;
    }
    else if (baseLRC == 3)
    {
      servo3.write(180); //Move left
      delay(300);
      servo3.write(93); //Stop moving
      baseLRC = 1;
    }
  }
  else if (event.acceleration.y > -1 && event.acceleration.y < +1) //Neither forwards or backwards
  {
    straightforwardmovement = true;
    movingleft = false;
    movingright = false;
    if (baseLRC == 1)
    {
      servo3.write(0); //Move right
      delay(300);
      servo3.write(93); //Stop moving
      baseLRC = 3;
    }
    else if (baseLRC == 2)
    {
      servo3.write(180); //Move left
      delay(300);
      servo3.write(93); //Stop moving
      baseLRC = 3;
    }
  }
*/

  //Check to see if user is walking.
  if ((event.acceleration.x - previousXacceleration) < -1 || (event.acceleration.x - previousXacceleration) > 1)
  {
    walking = true;
    statewalking(); //For testing the walk indicator.
  }
  else if ((event.acceleration.x - previousXacceleration) > -1 || (event.acceleration.x - previousXacceleration) < 1)
  {
    walking = false;
  }

  //Check it see if user is leaning forward.
  if (event.acceleration.x >= -9 && event.acceleration.x <= 0 && event.acceleration.z <0)
  {
    leaningforward = true;
    if (walking == false)
    {
      //Bring mid forward
      if (midFBM == 3) //Currently centered
      {
        servo2.write(180); //Move forwards
        delay(400);
        servo2.write(93); //Stop moving
        midFBM = 1;
      }
      else if (midFBM == 2) //Currently backwards
      {
        servo2.write(180); //Move forwards
        delay(800);
        servo2.write(93); //Stop moving
        midFBM = 1;
      }
    }
  }
  //Check it see if user is not leaning forward.
  else if (!(event.acceleration.x >= -9 && event.acceleration.x <= 0 && event.acceleration.z <0))
  {
    leaningforward = false;
    //Bring mid to middle
    if (walking == false)
    {
      if (midFBM == 1) //Currently forwards
      {
        servo2.write(0); //Move backward
        delay(300);
        servo2.write(93); //Stop moving
        midFBM = 3;
      }
      else if (midFBM == 2) //Currently backwards
      {
        servo2.write(180); //Move forwards
        delay(300);
        servo2.write(93); //Stop moving
        midFBM = 3;
      }
    }
  }
/*
  //Read off all the things the wearer is doing.
  Serial.print("The wearer is ");
  if (leaningforward == true)
  {
    Serial.print("leaning forward, ");
  }
  else if (leaningforward == false)
  {
    Serial.print("not leaning forward, ");
  }
  
  if (movingleft == true)
  {
    Serial.print("moving left, ");
  }
  else if (movingright == true)
  {
    Serial.print("moving right, ");
  }
  else if (straightforwardmovement == true)
  {
    Serial.print("not moving left or right, ");
  }

  if (walking == true)
  {
    Serial.print("and walking.");
  }
  else if (walking == false)
  {
    Serial.print("and not walking.");
  }
*/
  //Time to twitch?
  if (checkssincetwitch > checkstilltwitch)
  {
    Serial.print("Twitch");
    rando = random(100);
    
    if (rando > 50) //LRC or FBM?
    {
      //Twitch tipLRC
      if (tipLRC == 3) //Currently centered
      {
        rando = random(100);
        if (rando >= 50)
        {
          servo1.write(180); //Move left
          delay(500);
          servo1.write(93); //Stop moving
          tipLRC = 1; //Left
        }
        else
        {
          servo1.write(0); //Move right
          delay(500);
          servo1.write(93); //Stop moving
          tipLRC = 2; //Right
        }
      }
      else if (tipLRC == 1) //Currently left
      {
        rando = random(100);
        if (rando >= 50)
        {
          servo1.write(0); //Move right
          delay(1000);
          servo1.write(93); //Stop moving
          tipLRC = 2; //Right
        }
        else
        {
          servo1.write(0); //Move right
          delay(510);
          servo1.write(93); //Stop moving
          tipLRC = 3; //Centered
        }
      }
      else //Currently right
      {
        rando = random(100);
        if (rando >= 50)
        {
          servo1.write(180); //Move left
          delay(1000);
          servo1.write(93); //Stop moving
          tipLRC = 1; //Left
        }
        else
        {
          servo1.write(180); //Move left
          delay(510);
          servo1.write(93); //Stop moving
          tipLRC = 3; //Centered
        }
      }
    }

    else if (rando < 50)
    {
      //Twitch tipFBM
      if (tipFBM == 3) //Currently centered
      {
        rando = random(100);
        if (rando >= 50)
        {
          servo4.write(180); //Move backwards
          delay(500);
          servo4.write(93); //Stop moving
          tipFBM = 2; //Backwards
        }
        else
        {
          servo4.write(0); //Move forwards
          delay(500);
          servo4.write(93); //Stop moving
          tipFBM = 1; //Forwards
        }
      }
      else if (tipFBM == 1) //Currently forward
      {
        rando = random(100);
        if (rando >= 50)
        {
          servo4.write(180); //Move backwards
          delay(1000);
          servo4.write(93); //Stop moving
          tipFBM = 2; //Backwards
        }
        else
        {
          servo4.write(180); //Move backwards
          delay(450);
          servo4.write(93); //Stop moving
          tipFBM = 3; //Centered
        }
      }
      else //Currently backwards
      {
        rando = random(100);
        if (rando >= 50)
        {
          servo4.write(0); //Move forwards
          delay(1000);
          servo4.write(93); //Stop moving
          tipFBM = 1; //Forwards
        }
        else
        {
          servo4.write(0); //Move forwards
          delay(450);
          servo4.write(93); //Stop moving
          tipFBM = 3; //Centered
        }
      }
    }
    
    checkstilltwitch = random(600);
    checkssincetwitch = 0;
  }

  
  Serial.println();

  //Store values of previous accelerations for future use.
  previousXacceleration = event.acceleration.x;
  previousYacceleration = event.acceleration.y;
  previousZacceleration = event.acceleration.z;

  checkssincetwitch++;
  
  //delay(1500);
  
}

void statewalking()
{
  //Bring mid out backwards
  if (midFBM == 1) //Currendly forward
  {
    servo2.write(0); //Move backward
    delay(800);
    servo2.write(93); //Stop moving
    midFBM = 2;
  }
  else if (midFBM == 3) //Currently centered
  {
    servo2.write(0); //Move backwards
    delay(400);
    servo2.write(93); //Stop moving
    midFBM = 2;
  }
  
  //Make tip stick out backwards
  if (tipFBM == 1)
  {
    servo4.write(180); //Move backwards
    delay(1000);
    servo4.write(93); //Stop moving
    tipFBM = 2;
  }
  else if (tipFBM == 3)
  {
    servo4.write(180); //Move backwards
    delay(500);
    servo4.write(93); //Stop moving
    tipFBM = 2;
  }

  //Center tipLRC
  if (tipLRC == 1) //Currently left
  {
    servo1.write(0); //Move right
    delay(300);
    servo1.write(93); //Stop moving
    tipLRC = 3;
  }
  else if (tipLRC == 2) //Currently right
  {
    servo1.write(180); //Move left
    delay(300);
    servo1.write(93); //Stop moving
    tipLRC = 3;
  }
  delay(1500);
  /* testing the walk sensor with blinking light
  while (counter1 < 10)
  {
    digitalWrite(13, HIGH);   // turn the LED on (HIGH is the voltage level)
    delay(100);              // wait for a second
    digitalWrite(13, LOW);    // turn the LED off by making the voltage LOW
    delay(100);              // wait for a second
    counter1++;
  }
  counter1=0;
  */
}

