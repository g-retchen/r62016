// Gripper - Rigoberto 

// Include --------------------------------------------------
#include <Servo.h>      //Create servo object to control a servo
#include <Servo.h>

// Define -----------------------------------------------------
Servo Servo_w;
Servo Servo_g;
//----------------------------------------------------------------

void setup() 
{
  Servo_w.attach(4);  //Attach wrist servo to pwm/digital pin
  Servo_g.attach(5);  //Attach gripper servo to pwm/digital pin
}

void loop() 
{
  //Wrist Operation 

    Servo_w.write(45);  //tell servo to turn 45 degrees
    delay(1000);        //pause
    Servo_w.write(90);  //tell servo to turn 90 degrees
    delay(1000);        
    Servo_w.write(135); //tell servo to turn 135 degrees
    delay(1000);  
    Servo_w.write(180); //tell servo to turn 180 degrees
    delay(1000);        

// -----------------------------------------------------------------------
//Gripper Operation
 Servo_g.write(50);

//      int position;     
//Tell servo to go 135 degrees, stepping by 2 degrees
//for(position = 0; position < 180; position +=2)
//{
//  Servo_g.write(position);  //move to next position
//  delay(20);                 // pause to allow for picture

//Tell servo to go to 30 degrees, stepping by one degree
//for(position = 30; position >= 0; position -= 1)
//{
//    Servo_g.write(position);  //move to next position
//    delay(500);                //short pause to allow it to move
}
//}
}
