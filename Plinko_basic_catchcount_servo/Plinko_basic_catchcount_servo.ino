// This version has left/right limit switch detect with brake, catch detect and count,
// and when ball caught, move to right until left right hit, then run servo until ball released and then move back left until left right is not hit

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
int r_lim = 0; //for right limit logic
int l_lim = 0; //for left limit logic
int joyPosition = 0; //0 = left, 1 = right
int catchCounter = 0; //counter how many times ball has been caught
int catchState = 0; //current state of catch
int lastCatchState = 0; //previous state of catch
int servoAngle = 0; //angle command for servo
int slowMove = 50; // PWM (0-255) during slow moves

Servo myservo; //declare servo object

void setup()
{
    // Initiate the LCD:
  lcd.init();
  lcd.backlight();

  pinMode(A0, INPUT); //for joystick sensing
  pinMode(5, INPUT); //for output of catch sensor (photo diode). High = a ball is in the cup
  pinMode(6, INPUT_PULLUP); //for right limit logic. HIGH = limit detected, LOW = no limit. This pin has a PULLUP (default HIGH)
  pinMode(7, INPUT_PULLUP); //for left limit logic. HIGH = limit detected, LOW = no limit. This pin has a PULLUP (default HIGH)
  pinMode(8, OUTPUT); //for Brushed motor PWM1
  pinMode(9, OUTPUT); //for Brushed motor PWM2
  myservo.attach(10,1000,2000); //pin 10 for PWM for the cup tipping servo
  myservo.write(servoAngle); //set servo motor to zero degrees initially
  Serial.begin(9600);

  lcd.setCursor(5, 0);
  lcd.print("ROHM");
  lcd.setCursor(1, 1);
  lcd.print("SEMICONDUCTOR");
  delay(2000);
  lcd.setCursor(1, 0);
}
void loop()
{

  // read the value from the sensor
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
  r_lim = digitalRead(6) && joyPosition;
  l_lim = digitalRead(7) && !joyPosition;
  }

//Sensing of Joystick and PWM duty on pin 8 and pin 9
//sense for right
  outputValue1 = map(sensorValue,600, 1023, 0, 255);
  outputValue1 = constrain(outputValue1,0,!r_lim*255);
 
 //sense for left
  outputValue2 = map(sensorValue, 0, 423, 255, 0);
  outputValue2 = constrain(outputValue2,0,!l_lim*255);

//brake condition
  if(r_lim || l_lim) {
    outputValue1 = 255;
    outputValue2 = 255;
  }

//asign value to PWM pins to motor EVK
  analogWrite(8, outputValue1);
  analogWrite(9, outputValue2);
  
// LCD messages
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
        lcd.print("Catch the Ball   ");
      }
      else {
        lcd.print("Keep Playing      ");
      }
      lcd.setCursor(1, 1);
      lcd.print(String("Counter = ") + String(catchCounter) + String("        "));
    }

  catchState = digitalRead(5);
  //compare catchState to previous
  if (catchState != lastCatchState) {
    //state has changed, increment counter
    if (catchState) {
      //current state is high means there is a rising edge
      catchCounter++;
      lcd.setCursor(1, 0);
      lcd.print("Ball Caught     ");
      lcd.setCursor(1, 1);
      lcd.print(String("Counter = ") + String(catchCounter) + String("        "));

      //move slowly to the right until left limit is hit, ignore joystick, do this one time
      while(!r_lim) {
        //move right slowly
        delay(1000);
        outputValue1 = slowMove;
        outputValue2 = 0;
        r_lim = digitalRead(6); //keep checking right limit
        if(r_lim) {
          //brake
          outputValue2 = 255;
          outputValue1 = 255;
        }
        analogWrite(8, outputValue1);
        analogWrite(9, outputValue2);
        lcd.setCursor(1, 0);
        lcd.print("Ball Caught     ");
        lcd.setCursor(1, 1);
        lcd.print("Going to Right     ");;
      }
      delay(2000);

      //start moving servo to 180 degrees slowly
      while(catchState) {

        lcd.setCursor(1, 0);
        lcd.print("Releasing Ball     ");
        lcd.setCursor(1, 1);
        lcd.print(String("Tipping cup      "));
        delay(2000);

        //increase angle
        Serial.print("Starting Servo increase");
        for(servoAngle; servoAngle <= 180; servoAngle++) {
          myservo.write(servoAngle);
          Serial.print("servo Angle = "); Serial.print(servoAngle); Serial.print('\n');
          if(!(servoAngle%10)) { //if angle is multiple of 10's, show on LCD screen
            lcd.setCursor(1, 0);
            lcd.print("Releasing Ball     ");
            lcd.setCursor(1, 1);
            lcd.print(String("Angle = ") + String(servoAngle) + String("        "));
          }
          delay(10); // wait this amount every change in angle
        }
        delay(1000); //wait 1sec
        //decrease angle
        Serial.print("Decreasing angle");
        for(servoAngle; servoAngle >= 0; servoAngle--) {
          myservo.write(servoAngle); //
          Serial.print("servo Angle = "); Serial.print(servoAngle); Serial.print('\n');
          if(!(servoAngle%10)) { //if angle is multiple of 10's, show on LCD screen
            lcd.setCursor(1, 0);
            lcd.print("Releasing Ball     ");
            lcd.setCursor(1, 1);
            lcd.print(String("Angle = ") + String(servoAngle) + String("        "));
          }
          delay(10); // wait this amount every change in angle
        }
        delay(1000);
        catchState = digitalRead(5); //check again catchState;
        Serial.print("catchState = ");
        Serial.print(catchState);
        Serial.print('\n');
        //delay(1000);
      }
    }
    delay(50); //debounce
    lastCatchState = catchState; //update last state for next time
    lcd.setCursor(1, 0);
    lcd.print("Ball Released     ");
    lcd.setCursor(1, 1);
    lcd.print(String("Counter = ") + String(catchCounter) + String("           "));
    delay(1000);

    //move to the left a bit to get out of right limit
    while(r_lim) {
      lcd.setCursor(1, 0);
      lcd.print("Get ready .....     ");
      lcd.setCursor(1, 1);
      lcd.print(String("Counter = ") + String(catchCounter));
      outputValue1 = 0;
      outputValue2 = slowMove;
      analogWrite(8, outputValue1);
      analogWrite(9, outputValue2);
      r_lim = digitalRead(6); //check status of right limit
    }
    //delay(1000);
    outputValue2 = 0;
    outputValue1 = 0;
    analogWrite(8, outputValue1);
    analogWrite(9, outputValue2);

    
  }


  // Serial.print("sensor = ");
  // Serial.print(sensorValue);
  // Serial.print(" joyposition = ");
  // Serial.print(joyPosition);
  // Serial.print("  L_lim = ");
  // Serial.println(l_lim);
  // Serial.print("  R_lim = ");
  // Serial.println(r_lim);
  // Serial.print("  output1 = ");
  // Serial.println(outputValue1);
  // Serial.print("  output2 = ");
  // Serial.println(outputValue2);
  // delay(1000);

}