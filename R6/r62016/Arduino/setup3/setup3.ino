#include <FastLED.h>
#include <Arduino.h>
#include <SharpIR.h>




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
const uint8_t ledSpot[] = {14, 6, 5, 4, 3, 2, 1, 0, 14, 13, 12, 11, 10, 9, 8, 22, 21, 20, 19, 18, 17, 16, 30, 29, 28, 27, 26, 25, 24, 38, 37, 36, 35, 34, 33, 32, 46, 45, 44, 43, 42, 41, 40, 54, 53, 52, 51, 50, 49, 48, 55};


//initialize IR values and functions
#define model 20150
#define ir A0    //front sensor
#define ir2 A1
#define ir3 A2
SharpIR sharp1(ir, 25, 93, model);
SharpIR sharp2(ir2, 25, 93, model);
SharpIR sharp3(ir3, 25, 93, model);
boolean ObjInFront(SharpIR);  //get whether or not obstacle is in front of robot


//7 segment display
const int latchPin = 2; // pin 12
const int dataPin = 3; // pin 14
const int clockPin = 4; // pin 11


//LED initializations
//int sevSegValues();
void write7Seg();


//function declarations
//converts a string array of values to an integer  
void ledValues(*char);
int convertStr2Int(char digit);
void restartLEDs(void);
//void ledValues(char *typeOfNode);
//int** convert1Dto2d(int *1dArray);



void setup() {
    Serial.begin(9600);
    delay( 3000 ); // power-up safety delay

    //initialize LED object
    FastLED.addLeds<LED_TYPE, LED_PIN, COLOR_ORDER>(leds, NUM_LEDS);
    FastLED.setBrightness(  96 );
  //pinmodes for 7 segment LED  
  pinMode(latchPin, OUTPUT);
  pinMode(dataPin, OUTPUT);
  pinMode(clockPin, OUTPUT);
  leds[1] = CRGB::Red;
    
}

void loop()
{
  char typeOfNode[50];  //array of values in each LED on the matrix
  
  char front;
  char right;
  char left;
  
  //char ledNumber;
 
  char d;
  
  //wait until Pi is ready to communicate
  if (Serial.available() > 0){

  infoString = Serial.read();//-'0';
  int number = infoString[0];
     
  switch(number){
    case 'a':  front = objInFront(sharp1);//get front IR sensor from PI
               Serial.write(front);         
               break;
    case 'b':  right = objInFront(sharp1); //get front, right, and left values from IR sensor
               front = objInFront(sharp2);
               left = objInFront(sharp3);
               Serial.print(right);
               Serial.print(front);
               Serial.print(left);
               break;
    case 'c': ledValues(infoString);  //send LED values to matrix to display and save values in an array
                                      //updates the matrix
              Serial.flush();
              break;
    case 'e':                         //used for testing the matrix. remove later
              Serial.print(typeOfNode[1]);
              Serial.print(typeOfNode[2]);
              Serial.flush();
              break;
    case 'd': //ledNumber = sevSegValues();
              //ledNumber = Serial.read();
              write7Seg();
              //Serial.print(ledNumber);
              Serial.flush();              
              break;
    case 'f': //restartLEDs();
              Serial.flush();
              break;
    default:
              Serial.write('z');
              Serial.flush();
       //put numbers on grids case 'c': put numbers on grids
  }
  }

}


//this is currently not working
void restartLEDs(){

for (int i = 0; i < 64; i++)
     leds[i] = CRGB::White;
return;
}

//change color of the LED on the matrix display
//takes strng from the serial communication
//converts string to a number for which LED should display
//and a character for which color the LED should change to
//and changes the colors in the matrix accordingly
void ledValues(char typeOfString)
{
  int number;
  String strNumber;
  char gridSpot;
  
 // while (Serial.available() > 0){

  //get grid
  strNumber = Serial.readString();
 
  number = 10*convertStr2Int(strNumber[1]) + convertStr2Int(strNumber[2]);
  gridSpot = strNumber[3];
        

  switch (gridSpot){
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



//used with the ledValues function
//takes a number in the form of a character
//converts the character number to an integer number
//returns the integer number
int convertStr2Int(char digit){
  int digitAsInt = 0;
  
  switch(digit){
    case'0':
      digitAsInt = 0;
      break;
    case'1':
      digitAsInt = 1;
      break;
    case'2':
      digitAsInt = 2;
      break;
    case'3':
      digitAsInt = 3;
      break;
    case'4':
      digitAsInt = 4;
      break;
    case'5':
      digitAsInt = 5;
      break;
    case'6':
      digitAsInt = 6;
      break;
    case'7':
      digitAsInt = 7;
      break;
    case'8':
      digitAsInt = 8;
      break;
    case'9':
      digitAsInt = 9;
      break;  
    default:
      digitAsInt = 0;    
        
  }
  return digitAsInt;

}


//converts a 1d matrix to a 2d matrix
/*
int** convert1Dto2d(int *1dArray){
  int** array2D = 0;
  
        array2D = new int*[height];
  
        for (int h = 0; h < 7; h++)
        {
              array2D[h] = new int[width];
  
              for (int w = 0; w < 7; w++)
              {
                    array2D[h+1][w+1] = 1dArray[h*7 + w];                    
              }
        }
  return 2dArray;
}*/


//checks for obstacle in front of the robot
//I think that this sensor only detects within 10 t0 80 CM
//should be able to check 3 blocks
char objInFront(SharpIR sharp){
  int dis = sharp.distance();
  
  if(dis > 40)
    return '0';  //there is no obstacle in front of sensor
   else 
     return '1';  //there is an obstacle in front of sensor
}


//write values to the 7 segment display
void write7Seg(){
  
  while (!Serial.available()){}
  char number = Serial.read();

// JScofield 3-19-17.  added extra clock high and low to push data through the storage register.  Converted values to hex.   

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



