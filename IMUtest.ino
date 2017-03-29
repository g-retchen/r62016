#include <Arduino.h>
#include <Wire.h>
#include <Adafruit_Sensor.h>
#include <Adafruit_BNO055.h>
#include <utility/imumaths.h>
#include <string.h>
#include <FastLED.h>


//Function Declartions
void serialEvent(); // Receives data from Pi
void determineAction(char); // Determines which action to preform given an input
int convertChar2Int(char); // converts char to int
char convertInt2Char(int); // converts int to char
void write7Seg(char); // Writes a number to the seven segment display
void ledValues(int, char); // LED matrix
int get_heading(int); // read heading


// IMU Global Variables
Adafruit_BNO055 bno = Adafruit_BNO055(28);
Adafruit_BNO055 IMU = Adafruit_BNO055(28);
int heading;
char headingChar;
char fwd;
char turn;

//7 segment display
const int latchPin = 2; // pin 12
const int dataPin = 3; // pin 14
const int clockPin = 4; // pin 11

//initialize LED Matrix Display
#define LED_PIN     7
#define NUM_LEDS    55
#define BRIGHTNESS  64
#define LED_TYPE    WS2811
#define COLOR_ORDER GRB
#define kMatrixWidth 8
#define kMatrixHeight 8
#define UPDATES_PER_SECOND 100
CRGB leds[kMatrixWidth * kMatrixHeight];
const uint8_t ledSpot[] = {15, 6, 5, 4, 3, 2, 1, 0, 14, 13, 12, 11, 10, 9, 8, 22, 21, 20, 19, 18, 17, 16, 30, 29, 28, 27, 26, 25, 24, 38, 37, 36, 35, 34, 33, 32, 46, 45, 44, 43, 42, 41, 40, 54, 53, 52, 51, 50, 49, 48, 55};


// Global Variables
String inputString = ""; // String read in from the pi
boolean stringComplete = false;  // Flag to determine if the string from pi is complete

void setup(void) {
  Serial.begin(9600);
  delay(3000);
  
  //initialize LED object
  FastLED.addLeds<LED_TYPE, LED_PIN, COLOR_ORDER>(leds, NUM_LEDS);
  FastLED.setBrightness(  96 );
  //pinmodes for 7 segment LED  
  pinMode(latchPin, OUTPUT);
  pinMode(dataPin, OUTPUT);
  pinMode(clockPin, OUTPUT);
  while(1){
    write7Seg('1');
    delay(1000);
    write7Seg('2');
    delay(1000);
    write7Seg('3');
    delay(1000);
    write7Seg('4');
    delay(1000);
    write7Seg('5');
    delay(1000);
    write7Seg('6');
    delay(1000);
  }
  
   // bno055  section testing
  while(IMU.begin()==false)
  {
    Serial.println("Oops, no IMU");
    IMU.setExtCrystalUse(true);
 
    
    while(1);
  }
  
  leds[ledSpot[1]] = CRGB::Yellow;  // turn yellow on home square at init.
  FastLED.show();

  
}

void loop() {
  serialEvent(); // Call the event which reads from the pi
  
  if (stringComplete) {
  // Determines the action sent from the pi
    
    Serial.println(inputString[0]); // Print the recieved data from the PI
    Serial.println("");
    Serial.println("");
    char actionType = inputString[0];
    
    determineAction(actionType);
    stringComplete=false;
  }  
}

void determineAction(char actionType){
  // Determines the action

  char front;
  char right;
  char left;
  String value2Send = "";

  int number = 10*convertChar2Int(inputString[1]) + convertChar2Int(inputString[2]);
  int headerNumber = 100*convertChar2Int(inputString[1]) + 10*convertChar2Int(inputString[2]) + convertChar2Int(inputString[3]);
  char sevSegNumber = inputString[1];
  char gridType = inputString[3];
  
  switch(actionType){
//      case 'a':  front = objInFront(sharp1);//get front IR sensor from PI
//                 value2Send += front;
//                 value2Send += '\n';
//                 Serial.write(front);         
//                 break;
//      case 'b':  right = objInFront(sharp1); //get front, right, and left values from IR sensor
//                 //front = objInFront(sharp2);
//                 //left = objInFront(sharp3);
//                 value2Send += right;
//                 value2Send += front;
//                 value2Send += left;
//                 value2Send += '\n';
//                 //irValue2Send.append(front);
//                 //irValue2send.append()
//                 //Serial.print(right);
//                 //Serial.print(front);
//                 Serial.println(value2Send);
//                 break;
      case 'c': leds[ledSpot[number]] = CRGB::Purple;
                FastLED.show();
                //ledValues(number, gridType);   //updates the matrix
                Serial.println("hi from c");
                Serial.flush();
                break;
      case 'e':                         //used for testing the matrix. remove later
                //Serial.print(typeOfNode[1]);
                //Serial.print(typeOfNode[2]);
                Serial.flush();
                break;
      case 'd': write7Seg(sevSegNumber);
                Serial.println(sevSegNumber);
                Serial.flush();              
                break;
      case 'f': //restartLEDs();
                Serial.flush();
                break;
      case 'h' : 
                heading = get_heading(heading);
                headingChar = char(heading);
                //heading_delta = get_heading_delta(updated_heading);
                //heading = number2String(heading);
                //heading += '\n';
                //fwd = move_fwd(heading_delta);
                Serial.print("heading data section     ");
                Serial.println(heading);
                Serial.println("");
                //Serial.write(fwd);
                Serial.flush();
                break;
      default:
                Serial.write('z');
                Serial.flush();
         //put numbers on grids case 'c': put numbers on grids
    } // end-case
}

int get_heading(int heading){
  // reads heading from IMU
  sensors_event_t event;
  IMU.getEvent(&event);
  heading = event.orientation.x;
  return heading;
}

void serialEvent() {
  // Reads a character at a time from the Pi
  // concatnates the string from the pi in to a global
  while (Serial.available()) {
    // get the new byte:
    char inChar = (char)Serial.read();
    // add it to the inputString:
    inputString += inChar;
    // if the incoming character is a newline, set a flag
    // so the main loop can do something about it:
    if (inChar == '\n') {
      stringComplete = true;
    }
  }
}

void write7Seg(char number){
// JScofield 3-19-17.  
// added extra clock high and low to push data through the storage register.  
// Converted values to hex.   

  switch (number){
    case('2'): 
              digitalWrite(latchPin, HIGH);
              shiftOut(dataPin, clockPin, MSBFIRST, 0x5B);//shiftOut(dataPin, clockPin, MSBFIRST, 091);
              digitalWrite(latchPin, LOW);
              digitalWrite(latchPin, HIGH);
              digitalWrite(latchPin, LOW);
              break;
    case('3'):  
              digitalWrite(latchPin, HIGH);
              shiftOut(dataPin, clockPin, MSBFIRST, 0x4F);//shiftOut(dataPin, clockPin, MSBFIRST, 079);
              digitalWrite(latchPin, LOW);
              digitalWrite(latchPin, HIGH);
              digitalWrite(latchPin, LOW);
              break;
    case('4'):
              digitalWrite(latchPin, HIGH);
              shiftOut(dataPin, clockPin, MSBFIRST, 0x66);//shiftOut(dataPin, clockPin, MSBFIRST, 0102);
              digitalWrite(latchPin, LOW);
              digitalWrite(latchPin, HIGH);
              digitalWrite(latchPin, LOW);
              break;
    case('5'):
              digitalWrite(latchPin, HIGH);
              shiftOut(dataPin, clockPin, MSBFIRST, 0x6D);//shiftOut(dataPin, clockPin, MSBFIRST, 0109);
              digitalWrite(latchPin, LOW);
              digitalWrite(latchPin, HIGH);
              digitalWrite(latchPin, LOW);
              break;
    case('6'):
              digitalWrite(latchPin, HIGH);
              shiftOut(dataPin, clockPin, MSBFIRST, 0x7C);//shiftOut(dataPin, clockPin, MSBFIRST, 0124);
              digitalWrite(latchPin, LOW);
              digitalWrite(latchPin, HIGH);
              digitalWrite(latchPin, LOW);
              break;
    case('1'):
              digitalWrite(latchPin, HIGH);
              shiftOut(dataPin, clockPin, MSBFIRST, 0x06);//shiftOut(dataPin, clockPin, MSBFIRST, 6);
              digitalWrite(latchPin, LOW);
              digitalWrite(latchPin, HIGH);
              digitalWrite(latchPin, LOW);
              break;
    default:
              digitalWrite(latchPin, HIGH);
              shiftOut(dataPin, clockPin, MSBFIRST, 0x00);//shiftOut(dataPin, clockPin, MSBFIRST, 000);
              digitalWrite(latchPin, LOW);
              digitalWrite(latchPin, HIGH);
              digitalWrite(latchPin, LOW);
              break;

  } 
    return;
}

//change color of the LED on the matrix display
//takes strng from the serial communication
//converts string to a number for which LED should display
//and a character for which color the LED should change to
//and changes the colors in the matrix accordingly
void ledValues(int number, char typeOfString)
{
       

  switch (typeOfString){
    case 'r':
        leds[ledSpot[number]] = CRGB::Red;
        break;
    case 'b':
        leds[ledSpot[number]] = CRGB::Blue;
        break;
    case 'g':
        leds[ledSpot[number]] = CRGB::Green;
        break;
    case 'p':
        leds[ledSpot[number]] = CRGB::Purple;
        break;
    case 'y':
        leds[ledSpot[number]] = CRGB::Yellow;        
        break;
    default:
        //Serial.print("Error: No Color Input");
        break;
  }
  //  typeOfNode[number] = gridSpot;
    FastLED.show();
    return;
                                                   
//}
}


int convertChar2Int(char digit){
  // Converts a char to int
  int digitAsInt = 0;
  
  switch(digit){
    case'0': digitAsInt = 0; break;
    case'1': digitAsInt = 1; break;
    case'2': digitAsInt = 2; break;
    case'3': digitAsInt = 3; break;
    case'4': digitAsInt = 4; break;
    case'5': digitAsInt = 5; break;
    case'6': digitAsInt = 6; break;
    case'7': digitAsInt = 7; break;
    case'8': digitAsInt = 8; break;
    case'9': digitAsInt = 9; break;  
    default: digitAsInt = 0;         
  }
  return digitAsInt;

}

String number2String(int number){
  // Converts an int to string

  String send = "";
  
  send += convertInt2Char(number / 100);
  send += convertInt2Char((number % 100) / 10);
  send += convertInt2Char(number % 10);

  return send;
}
