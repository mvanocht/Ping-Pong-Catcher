// This version has left/right limit switch detect with brake, catch detect and count,
// and when ball caught, move to right until right hit, then run servo until ball released and then move back left until left right is not hit
// Debounce implemented on RLIM and LLIM switches, as well as OPTO input

#include <Servo.h>
#include <LiquidCrystal_I2C.h>
#include <Wire.h>

// Wiring: SDA pin is connected to A4 and SCL pin to A5.
// Connect to LCD via I2C, default address 0x27 (A0-A2 not jumpered)
LiquidCrystal_I2C lcd = LiquidCrystal_I2C(0x27, 16, 2); // Change to (0x27,20,4) for 20x4 LCD.

char mytextupper[] = "";
char mytextlower[] = "";

int sensorValue = 0; //for joystick sensing
int outputValue1 = 0; //for Brushed motor PWM1
int outputValue2 = 0; //for Brushed motor PWM2
int servomotorValue = 0; //for servo motor 980Hz PWM
boolean r_lim = false; //for right limit logic
boolean l_lim = false; //for left limit logic
int joyPosition = 0; //0 = left, 1 = right, 2 = middle
int catchCounter = 0; //counter how many times ball has been caught
boolean catchState = 0; //current state of catch
boolean lastCatchState = 0; //previous state of catch
int servoAngle = 0; //angle command for servo
int servoAngleChangeWait = 10; //wait this ms before incrementing angle
int slowMove = 100; // PWM (0-255) during slow moves
int fastMove = 200; // PWM (0-255) during slow moves
const unsigned long DebounceTime = 10;
const unsigned long OPTODebounceTimeHigh = 1000;
const unsigned long OPTODebounceTimeLow = 10;
unsigned long RLIMButtonStateChangeTime = 0; // Debounce timer
unsigned long LLIMButtonStateChangeTime = 0; // Debounce timer
unsigned long OPTOButtonStateChangeTime = 0; // Debounce timer
boolean RLIMButtonWasPressed = true; //active high
boolean LLIMButtonWasPressed = true; //active high
boolean OPTOButtonWasPressed = false; //active high
boolean RLIMState = false;
boolean LLIMState = false;
boolean OPTOState = false;

Servo myservo; //declare servo object

void setup()
{
    // Initiate the LCD:
  lcd.init();
  lcd.backlight();

  pinMode(A0, INPUT); //for joystick sensing
  pinMode(5, INPUT); //for output of catch sensor (opto detector). High = a ball is in the cup
  pinMode(6, INPUT_PULLUP); //for right limit logic. HIGH = limit detected, LOW = no limit. This pin has a PULLUP (default HIGH)
  pinMode(7, INPUT_PULLUP); //for left limit logic. HIGH = limit detected, LOW = no limit. This pin has a PULLUP (default HIGH)
  pinMode(8, OUTPUT); //for Brushed motor PWM1
  pinMode(9, OUTPUT); //for Brushed motor PWM2

  myservo.attach(10,550,2000); //pin 10 for PWM for the cup tipping servo
  myservo.write(servoAngle); //set servo motor to zero degrees initially

  //Serial.begin(9600);

  lcd.setCursor(5, 0);
  lcd.print("ROHM");
  lcd.setCursor(1, 1);
  lcd.print("SEMICONDUCTOR");
  delay(2000);
  lcd.setCursor(1, 0);
  lcd.print("Catch the Ball !!!  ");
  lcd.setCursor(1, 1);
  lcd.print(String("Counter = ") + String(catchCounter) + String("        "));
}

void checkRLIMButton()
{
  //RLIM detect
  boolean RLIMbuttonIsPressed = digitalRead(6); //Active High
  
  if(RLIMbuttonIsPressed != RLIMButtonWasPressed) { //button state changed from before
    RLIMButtonWasPressed = RLIMbuttonIsPressed;
    RLIMButtonStateChangeTime = millis(); //record the current time when button state changed
  }

  if ((millis() - RLIMButtonStateChangeTime > DebounceTime) && RLIMButtonWasPressed) //if time has passed more than debounce time and Button is High
  {
    RLIMState = true;
  }
  else //going to low does not require any debounce (if want to flip polarity, the debounce has to moved here)
  {
      RLIMState = false;
  }
}

void checkLLIMButton()
{

  //LLIM detect
  boolean LLIMbuttonIsPressed = digitalRead(7); //Active High
  
  if(LLIMbuttonIsPressed != LLIMButtonWasPressed) { //button state changed from before
    LLIMButtonWasPressed = LLIMbuttonIsPressed;
    LLIMButtonStateChangeTime = millis(); //record the current time when button state changed
  }

  if ((millis() - LLIMButtonStateChangeTime > DebounceTime) && LLIMButtonWasPressed) //if time has passed more than debounce time and Button is High
  {
    LLIMState = true;
  }
  else //going to low does not require any debounce (if want to flip polarity, the debounce has to moved here)
  {
      LLIMState = false;
  }

}

void checkOPTOButton()
{

  //OPTO detect
  boolean OPTObuttonIsPressed = digitalRead(5); //Active High
  
  if(OPTObuttonIsPressed != OPTOButtonWasPressed) { //button state changed from before
    OPTOButtonWasPressed = OPTObuttonIsPressed;
    OPTOButtonStateChangeTime = millis(); //record the current time when button state changed
  }

  if ((millis() - OPTOButtonStateChangeTime > OPTODebounceTimeHigh) && OPTOButtonWasPressed) //if time has passed more than debounce time and Button is High
  {
    OPTOState = true;
  }
  else //going to low does not require any debounce (if want to flip polarity, the debounce has to moved here)
  {
    OPTOState = false;
  }
  
}

void loop()
{

  ///////////JOYSTICK OPERATION////////////
  // read the value from the joystick
  sensorValue = analogRead(A0);
  if (sensorValue < 400) {
    joyPosition = 0;
  	}
    else if (sensorValue > 600) {
      joyPosition = 1;
    }
    else {
      joyPosition = 2;
    }

  if(joyPosition != 2) {
  checkLLIMButton();
  checkRLIMButton();
  //Serial.print("RLIMState "); Serial.print(RLIMState); Serial.print('\n');
  //Serial.print("LLIMState "); Serial.print(LLIMState); Serial.print('\n');
  r_lim = RLIMState && joyPosition;
  l_lim = LLIMState && !joyPosition;
  //Serial.print("RLIM "); Serial.print(r_lim); Serial.print('\n');
  //Serial.print("LLIM "); Serial.print(l_lim); Serial.print('\n');
  }

//Sensing of Joystick and PWM duty on pin 8 and pin 9
//sense for right
  outputValue1 = map(sensorValue,600, 1023, 0, 255);
  outputValue1 = constrain(outputValue1,0,!r_lim*255*0.5);
 
 //sense for left
  outputValue2 = map(sensorValue, 0, 423, 255, 0);
  outputValue2 = constrain(outputValue2,0,!l_lim*255*0.5);

//brake condition
  if(r_lim || l_lim) {
    outputValue1 = 255;
    outputValue2 = 255;
  }
///////////JOYSTICK OPERATION////////////

//////////MOTOR OPERATION//////////////
//asign value to PWM pins to motor EVK
  analogWrite(8, outputValue1);
  analogWrite(9, outputValue2);
  
////////// Normal play LCD messages//////////
  if(r_lim && !l_lim) {
      lcd.setCursor(1, 0);
      lcd.print("Right Limit     ");
      lcd.setCursor(1, 1);
      lcd.print("Reached       ");
    }
    else if(l_lim && !r_lim) {
      lcd.setCursor(1, 0);
      lcd.print("Left Limit    ");
      lcd.setCursor(1, 1);
      lcd.print("Reached       ");
    }

  if(!r_lim && !l_lim) {
      lcd.setCursor(1, 0);
      if(!catchCounter) {
        lcd.print("Catch the Ball   "); //when counter = 0 display this messaage
      }
      else {
        lcd.print("Keep Playing      "); //if counter is > 0, display this message instead
      }
      lcd.setCursor(1, 1);
      lcd.print(String("Counter = ") + String(catchCounter) + String("        "));
    }
  


  //Section for ball caught + moving to right + dumping ball out using servo + move back to left
  //Joytstick is ignored the entire duration of this operation
  checkOPTOButton();
  catchState = OPTOState;
  //Serial.print("catchState ");Serial.print(catchState);Serial.print('\n');
  //Serial.print("LastcatchState ");Serial.print(lastCatchState);Serial.print('\n');

  //compare catchState to previous
  //during the below operations, we do not care about the actual Opto output
  if (catchState != lastCatchState) {
    //state has changed, increment counter
    if (catchState) {
      //current state is high means there is a rising edge
      catchCounter++;
      lcd.setCursor(1, 0);
      lcd.print("Ball Caught     ");
      lcd.setCursor(1, 1);
      lcd.print(String("Counter = ") + String(catchCounter) + String("        "));

      //move slowly to the right until right limit is hit, ignore joystick, do this one time
      delay(100);
      while(!r_lim) {
        lcd.setCursor(1, 0);
        lcd.print("Ball Caught     ");
        lcd.setCursor(1, 1);
        lcd.print("Going to Right     ");
        //move right slowly
        //delay(1000);
        outputValue1 = slowMove;
        outputValue2 = 0;
        checkRLIMButton();
        r_lim = RLIMState; //keep checking right limit
        if(r_lim) {
          //brake
          outputValue2 = 255;
          outputValue1 = 255;
        }
        analogWrite(8, outputValue1);
        analogWrite(9, outputValue2);
        
      }
      delay(100);

      //start moving servo to 180 degrees slowly
      while(catchState) {

        //myservo.attach(10,550,2000); //pin 10 for PWM for the cup tipping servo
        //myservo.write(servoAngle); //set servo motor to zero degrees initially

        lcd.setCursor(1, 0);
        lcd.print("Releasing Ball     ");
        lcd.setCursor(1, 1);
        lcd.print(String("Tipping cup      "));
        delay(100);

        //increase angle
        //Serial.print("Starting Servo increase");
        for(servoAngle; servoAngle <= 180; servoAngle++) {
          myservo.write(servoAngle);
          //Serial.print("servo Angle = "); Serial.print(servoAngle); Serial.print('\n');
          // if(!(servoAngle%10)) { //if angle is multiple of 10's, show on LCD screen
          //   lcd.setCursor(1, 0);
          //   lcd.print("Releasing Ball     ");
          //   lcd.setCursor(1, 1);
          //   lcd.print(String("Angle = ") + String(servoAngle) + String("        "));
          // }
          delay(servoAngleChangeWait); // wait this amount every change in angle
        }
        delay(100); //wait 1sec
        //decrease angle
        //Serial.print("Decreasing angle");
        for(servoAngle; servoAngle >= 0; servoAngle--) {
          myservo.write(servoAngle); //
          //Serial.print("servo Angle = "); Serial.print(servoAngle); Serial.print('\n');
          // if(!(servoAngle%10)) { //if angle is multiple of 10's, show on LCD screen
          //   lcd.setCursor(1, 0);
          //   lcd.print("Releasing Ball     ");
          //   lcd.setCursor(1, 1);
          //   lcd.print(String("Angle = ") + String(servoAngle) + String("        "));
          // }
          delay(servoAngleChangeWait); // wait this amount every change in angle
        }
        delay(100);
        catchState = digitalRead(5); //check again catchState;
        //Serial.print("catchState = "); Serial.print(catchState); Serial.print('\n');
        //delay(1000);

      }

      //ball is now released
      delay(1000); //debounce
      lastCatchState = catchState; //update last catch state for next time to detect rising edge again
      lcd.setCursor(1, 0);
      lcd.print("Ball Released     ");
      lcd.setCursor(1, 1);
      lcd.print(String("Counter = ") + String(catchCounter) + String("           "));
      delay(100);

      //move to the left a bit to get out of right limit
      while(r_lim) {
        lcd.setCursor(1, 0);
        lcd.print("Moving cup back....     ");
        lcd.setCursor(1, 1);
        lcd.print("Get ready .....     ");
        outputValue1 = 0;
        outputValue2 = fastMove;
        analogWrite(8, outputValue1);
        analogWrite(9, outputValue2);
        checkRLIMButton();
        r_lim = RLIMState; //check status of right limit
      }
      //delay(1000);
      outputValue2 = 0; //reset to prevent motor from turning
      outputValue1 = 0; //reset to prevent motor from turning
      analogWrite(8, outputValue1);
      analogWrite(9, outputValue2);
      delay(2000);

    }

    else {
    // catchState is low, and we don't care
    }

    
  }

}
