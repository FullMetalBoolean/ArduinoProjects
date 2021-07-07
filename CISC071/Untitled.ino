/*
 * OBJECT AVOIDANCE WITH SERVO
 * 
 * Robot will avoid objects in front.
 * Ultrasonic sensor scans for the best route to take
 * Robot can't detect objects at it's side, best to install 2 IR Detectors
 * *
 */

 // include the library code:
#include <LiquidCrystal.h>
#include "sensorsDRV.h"
#include <avr/wdt.h>
#include "TBMotor.h"
#include <Servo.h>
#include <SPI.h>


OseppTBMotor Motor1(12, 11);
OseppTBMotor Motor2(8, 3);

const int rs = 1, en = 0, d4 = 5, d5 = 7, d6 = 3, d7 = 2;
LiquidCrystal lcd(rs, en, d4, d5, d6, d7);


/////////////////////////////
//VARIABLES & INITIALIZATION
/////////////////////////////

#define leftMotor Motor1
#define rightMotor Motor2


Ultrasonic ults(2, 4);

const int servoPin = 9;
const int servoBais = 85;


Servo sv;
///////////////////////////
//Accelorometer variable setup
//////////////////////////
int leftSpeed = 0;
int rightSpeed = 0;

//the time we give the sensor to calibrate (10-60 secs according to the datasheet)
int calibrationTime = 30;        

//the time when the sensor outputs a low impulse
long unsigned int lowIn;         

//the amount of milliseconds the sensor has to be low 
//before we assume all motion has stopped
long unsigned int pause = 5000;  

boolean lockLow = true;
boolean takeLowTime;   

int pirPin = 6;    //the digital pin connected to the PIR sensor's output
int ledPin = 10;

//Assign the Chip Select signal to pin 10.
int CS=11;

//This is a list of some of the registers available on the ADXL345.
//To learn more about these and the rest of the registers on the ADXL345, read the datasheet!
char POWER_CTL = 0x2D;  //Power Control Register
char DATA_FORMAT = 0x31;
char DATAX0 = 0x32; //X-Axis Data 0
char DATAX1 = 0x33; //X-Axis Data 1
char DATAY0 = 0x34; //Y-Axis Data 0
char DATAY1 = 0x35; //Y-Axis Data 1
char DATAZ0 = 0x36; //Z-Axis Data 0
char DATAZ1 = 0x37; //Z-Axis Data 1

//This buffer will hold values read from the ADXL345 registers.
char values[10];
//These variables will be used to hold the x,y and z axis accelerometer values.
int x,y,z;

/////////////////////////////
//SETUP
/////////////////////////////
void setup(){
  Serial.begin(9600);
  ///////////////////////////
  //Object Avoidance Code
  ///////////////////////////
  Serial.begin(9600);
  Serial.println("Serial Monitor Setup Done");
  //Setup a watchdog
  //When the battery voltage is insufficient / program unexpected
  //Watchdog reset chip
  wdt_enable(WDTO_4S);

  sv.attach(servoPin);
  sv.write(servoBais);
  /////////////////////////
  //Secondary Sensor Code
  ////////////////////////
  pinMode(pirPin, INPUT);
  pinMode(ledPin, OUTPUT);
  digitalWrite(pirPin, LOW);

  //give the sensor some time to calibrate
  lcd.print("calibrating sensor ");
    for(int i = 0; i < calibrationTime; i++){
      lcd.print(".");
      delay(1000);
      }
    lcd.println(" done");
    lcd.println("SENSOR ACTIVE");
    delay(50);

    // set up the LCD's number of columns and rows:
  lcd.begin(16, 2);

  //Initiate an SPI communication instance.
  SPI.begin();
  //Configure the SPI connection for the ADXL345.
  SPI.setDataMode(SPI_MODE3);
  //Create a serial connection to display the data on the terminal.
 
  
  //Set up the Chip Select pin to be an output from the Arduino.
  pinMode(CS, OUTPUT);
  //Before communication starts, the Chip Select pin needs to be set high.
  digitalWrite(CS, HIGH);
  
  //Put the ADXL345 into +/- 4G range by writing the value 0x01 to the DATA_FORMAT register.
  writeRegister(DATA_FORMAT, 0x01);
  //Put the ADXL345 into Measurement Mode by writing 0x08 to the POWER_CTL register.
  writeRegister(POWER_CTL, 0x08);  //Measurement mode  
}

////////////////////////////
//LOOP
void loop(){
  //Object Avoidance "check-n-avoid" Loop
  ////////////////////////////////////////////////////
  const float threshold = 300;

  //If in 4 seconds,The program is not coming back here.
  //Chip will reset
  wdt_reset();
  sv.write(servoBais);
  if (dist_filter() < threshold)
  {
    //If an obstacle is detected
    //stop the car first
    Serial.println("Stopped, object detected");
    leftSpeed = rightSpeed = 0;
    SetMotor();
    if (moreOpenDirection() == 'R')
    {
      //If the right side is more open, turn right.
      leftSpeed = 255;
      rightSpeed = -255;
    } else {
      leftSpeed = -255;
      rightSpeed = 255;
    }
    SetMotor();
    //Keep turning until you avoid it
    Serial.println("Turning to avoid");
    while (dist_filter() < threshold) delay(50);
    //The obstacle is not in the range of ultrasonic sensor
    //It's probably still on the Tank route
    //Continue to turn out for some time, may be able to completely avoid it
    //Time delay depends on the speed of the tank.
    delay(300);
    return;//to the beginning of the loop function.
    Serial.println("End of Sensor Cycle");
  }
  //If there is no problem, keep moving forward.
  leftSpeed = rightSpeed = 255;
  SetMotor();
  Serial.println("moving on");
  ////////////////////////////////////////
  //Secondary Sensor Logs n' Loops
  ////////////////////////////////////////

     if(digitalRead(pirPin) == HIGH){
       digitalWrite(ledPin, HIGH);   //the led visualizes the sensors output pin state
       if(lockLow){  
         //makes sure we wait for a transition to LOW before any further output is made:
         lockLow = false;            
         lcd.println("---");
         lcd.print("motion detected at ");
         Serial.print(millis()/1000);
         lcd.println(" sec"); 
         delay(50);
         }         
         takeLowTime = true;
       }

     if(digitalRead(pirPin) == LOW){       
       digitalWrite(ledPin, LOW);  //the led visualizes the sensors output pin state

       if(takeLowTime){
        lowIn = millis();          //save the time of the transition from high to LOW
        takeLowTime = false;       //make sure this is only done at the start of a LOW phase
        }
       //if the sensor is low for more than the given pause, 
       //we assume that no more motion is going to happen
       if(!lockLow && millis() - lowIn > pause){  
           //makes sure this block of code is only executed again after 
           //a new motion sequence has been detected
           lockLow = true;                        
           lcd.print("motion ended at ");      //output
           Serial.print((millis() - pause)/1000);
           lcd.println(" sec");
           delay(50);
           }
       }
 // set the cursor to column 0, line 1
  // (note: line 1 is the second row, since counting begins with 0):
  lcd.setCursor(0, 1);
  // print the number of seconds since reset:
  Serial.print(millis() / 1000);

  //Reading 6 bytes of data starting at register DATAX0 will retrieve the x,y and z acceleration values from the ADXL345.
  //The results of the read operation will get stored to the values[] buffer.
  readRegister(DATAX0, 6, values);

  //The ADXL345 gives 10-bit acceleration values, but they are stored as bytes (8-bits). To get the full value, two bytes must be combined for each axis.
  //The X value is stored in values[0] and values[1].
  x = ((int)values[1]<<8)|(int)values[0];
  //The Y value is stored in values[2] and values[3].
  y = ((int)values[3]<<8)|(int)values[2];
  //The Z value is stored in values[4] and values[5].
  z = ((int)values[5]<<8)|(int)values[4];
  
  //Print the results to the terminal.
  Serial.print(x, DEC);
  Serial.print(',');
  Serial.print(y, DEC);
  Serial.print(',');
  Serial.println(z, DEC);      
  delay(10); 
}


/////////////////////////////////////////////////
//Member Body Functions
///////////////////////////////////////////////

//////////////////////////////////////////////
//OA Code
//Multiple sampling, take the average, is a good way to resist interference
float dist_filter()
{
  float avgDist = 0;
  int i = 0;
  for (i = 0; i < 10; i++)avgDist += ults.Detect();
  return avgDist / i;
}

void SetMotor()
{
  if (leftSpeed > 255)leftSpeed = 255; else if (leftSpeed < -255)leftSpeed = -255;
  if (rightSpeed > 255)rightSpeed = 255; else if (rightSpeed < -255)rightSpeed = -255;
  //Depending on your connection,
  //if the direction of the motor rotation is not the direction you want,
  //you can change it by changing the Positive/negative sign of the speed
  leftMotor.SetSpeed(leftSpeed);
  rightMotor.SetSpeed(-rightSpeed);
}

char moreOpenDirection()
{
  long scan[90];
  sv.write(servoBais);
  //Empty array
  for (int i = 0; i < 90; i++)scan[i] = 0;
  delay(100);
  //Scan the distance around and into the array
  for (int i = 0; i > -45; i--)
  {
    sv.write(servoBais + i); delay(2);
    scan[45 - i] = ults.Detect();
  }
  for (int i = -45; i < 45; i++)
  {
    sv.write(i + servoBais); delay(2);
    scan[45 - i - 1] += ults.Detect();
  }
  for (int i = 45; i > 0; i--)
  {
    sv.write(i + servoBais); delay(2);
    scan[45 - i] += ults.Detect();
  }
  //note:Every angle has been scanned for two times.
  sv.write(servoBais);

  //Calculate the side of the more open
  //We can simply sum the distance between two sides.
  long left = 0;
  for (int i = 0; i < 45; i++)left += scan[i];
  long right = 0;
  for (int i = 45; i < 90; i++)right += scan[i];
  if (left > right)return 'L'; else return 'R';
}
/////////////////////////////////////////////////////////
//SS Code





//This function will write a value to a register on the ADXL345.
//Parameters:
//  char registerAddress - The register to write a value to
//  char value - The value to be written to the specified register.
void writeRegister(char registerAddress, char value){
  //Set Chip Select pin low to signal the beginning of an SPI packet.
  digitalWrite(CS, LOW);
  //Transfer the register address over SPI.
  SPI.transfer(registerAddress);
  //Transfer the desired register value over SPI.
  SPI.transfer(value);
  //Set the Chip Select pin high to signal the end of an SPI packet.
  digitalWrite(CS, HIGH);
}

//This function will read a certain number of registers starting from a specified address and store their values in a buffer.
//Parameters:
//  char registerAddress - The register addresse to start the read sequence from.
//  int numBytes - The number of registers that should be read.
//  char * values - A pointer to a buffer where the results of the operation should be stored.
void readRegister(char registerAddress, int numBytes, char * values){
  //Since we're performing a read operation, the most significant bit of the register address should be set.
  char address = 0x80 | registerAddress;
  //If we're doing a multi-byte read, bit 6 needs to be set as well.
  if(numBytes > 1)address = address | 0x40;
  
  //Set the Chip select pin low to start an SPI packet.
  digitalWrite(CS, LOW);
  //Transfer the starting register address that needs to be read.
  SPI.transfer(address);
  //Continue to read registers until we've read the number specified, storing the results to the input buffer.
  for(int i=0; i<numBytes; i++){
    values[i] = SPI.transfer(0x00);
  }
  //Set the Chips Select pin high to end the SPI packet.
  digitalWrite(CS, HIGH);
}