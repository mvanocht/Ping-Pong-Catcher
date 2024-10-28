// This version has left/right limit switch detect with brake, catch detect and count

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
  // pinMode(4, OUTPUT); //for servo motor 980Hz PWM (pin 13 and pin 4 are 980Hz, others are 490Hz)
  Serial.begin(9600);
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
  
  catchState = digitalRead(5);
  //compare catchState to previous
  if (catchState != lastCatchState) {
    //state has changed, increment counter
    if (catchState) {
      //current state is high means there is a rising edge
      catchCounter++;
    }
    delay(50); //debounce
    lastCatchState = catchState; //update last state for next time
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
      lcd.setCursor(2, 0);
      lcd.print("Right Limit     ");
      lcd.setCursor(2, 1);
      lcd.print("Reached       ");
    }
    else if(l_lim && !r_lim) {
      lcd.setCursor(2, 0);
      lcd.print("Left Limit    ");
      lcd.setCursor(2, 1);
      lcd.print("Reached       ");
    }

if(!r_lim && !l_lim) {
      lcd.setCursor(2, 0);
      lcd.print("Keep Playing  ");
      lcd.setCursor(2, 1);
      lcd.print(String("Counter = ") + String(catchCounter));
    }

  
  Serial.print("sensor = ");
  Serial.print(sensorValue);
  Serial.print(" joyposition = ");
  Serial.print(joyPosition);
  Serial.print("  L_lim = ");
  Serial.println(l_lim);
  Serial.print("  R_lim = ");
  Serial.println(r_lim);
  Serial.print("  output1 = ");
  Serial.println(outputValue1);
  Serial.print("  output2 = ");
  Serial.println(outputValue2);
  //delay(1000);

}