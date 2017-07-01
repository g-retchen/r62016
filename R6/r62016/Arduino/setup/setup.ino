#include <SharpIR.h>
#include <Arduino.h>


/* PIN definitions */
// built in LED
const byte led_pin = LED_BUILTIN;  
 // Button on pin-2, internal pullup
const byte but_pin = 3;           
// the number of the pushbutton pin
const int buttonPin = 2;     
int buttonState = 0;

/* Volatile variables modified in the ISR */
volatile boolean led_state = LOW;
volatile unsigned int last_but_ptime = 0;

byte number = 0;
int state = 0;

//IR Sensor
#define ir A0
#define ir2 A1
#define ir3 A2
//#define ir A2
#define model 20150
//boolean done=false;
SharpIR sharp1(ir, 25, 93, model);
SharpIR sharp2(ir2, 25, 93, model);
SharpIR sharp3(ir3, 25, 93, model);


boolean isEMF();
boolean ObjInFront();
boolean ObjOnLeft();
boolean ObjOnRight();


void setup()
{
  Serial.begin(9600);
  //tester LED
  pinMode(13, OUTPUT); 
 // pinMode(buttonPin, INPUT);
  pinMode(led_pin, OUTPUT);
  pinMode(but_pin, INPUT_PULLUP);
  pinMode (ir, INPUT);
  
  
  //wait for the button to be pressed (we still need to buy an inverted schmitt trigger)
  while (buttonState != HIGH) {
    //  Serial.write("can't start car until button is pressed!");
      buttonState = digitalRead(buttonPin);
  };
}

/* Show the led_state on the internal LED, it is changed in the ISR */
void loop()
{
  // digitalWrite(led_pin, led_state);    
    
  //look for serial from the pi
  //get the values from the sensors
  //send the values to the raspberry pi (determine beforehand where you are on the track -- are you on a tunnel edge?)
  //update the 7x7 LED display
  
  //wait until Pi is ready to communicate
  while(Serial.available() == 0){}

   number = Serial.read();
   if (number == 'd')
     Serial.write('z');
   //Serial.write(number);
 //  Serial.print("character recieved");
  // Serial.print(number);
//   if (number != 'd')
  //   while(1);
   //get and send values to the pi
   boolean emf_values = isEMF();
   boolean obj_in_front = objInFront();
   
   //do I eventually want to just make this all println()?
   if(isEMF())
     Serial.write('E');
   else
     Serial.write('F');
     
   //checks for object in front of the IR Sensor
   if(objInFront())
     Serial.write('G');
   else
     Serial.write('H');
     
   //checks for object on left of the IR Sensor
   if(objOnLeft())
     Serial.write('G');
   else
     Serial.write('H');
     
   //checks for object on left of the IR Sensor
   if(objOnLeft())
     Serial.write('G');
   else
     Serial.write('H');
     
   //get node number from Pi
   number = Serial.read();
   
   //get node type from Pi
   node_type = Serial.read();
   
   //
   //
   //update the 7x7 grid
}


//determine if there is an EMF signal under the robot. might change this function to color light sensor
boolean isEMF(){
  //if on EMF tunnel, 
  //return true;
  //else
  //return false;
  return true;
}

//checks for obstacle in front of the robot
//I think that this sensor only detects within 10 t0 80 CM
//should be able to check 3 blocks
boolean objInFront(){
  int dis = sharp1.distance();
  
  if(dis > 100)
    return true;
   else 
     return false;
}

////checks for obstacle in front of the robot
boolean objOnLeft(){
  int dis = sharp2.distance();
  
  if(dis > 100)
    return true;
   else 
     return false;
}

//checks for obstacle in front of the robot
/*
boolean objOnRight(){
  int dis = sharp3.distance();
  
  if(dis > 100)
    return true;
   else 
     return false;
}
*/



