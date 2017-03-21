#include <Wire.h>
#include <Adafruit_Sensor.h>
#include <Adafruit_BNO055.h>
#include <utility/imumaths.h>

// Set the delay between fresh samples
#define BNO055_SAMPLERATE_DELAY_MS (3000)

Adafruit_BNO055 bno = Adafruit_BNO055(55);

/////////////////////////////////////////////////////////////////////////////////////////
// Arduino setup function (automatically called at startup)
/////////////////////////////////////////////////////////////////////////////////////////

void setup(void)
{
  Serial.begin(9600);

  //  Initialise the sensor
  if (!bno.begin())
  {
    //There was a problem detecting the BNO055 ... check your connections
    Serial.print("Ooops, no BNO055 detected ... Check your wiring or I2C ADDR!");
    while (1);
  }

  delay(1000);

  bno.setExtCrystalUse(true);
}

/////////////////////////////////////////////////////////////////////////////////////////
//  Arduino loop function, called once 'setup' is complete (your own code should go here)
/////////////////////////////////////////////////////////////////////////////////////////

void loop(void)
{
  // declare variables
  int updated_heading;
  int heading_delta;
  char fwd;
  char turn;
  char py_signal = 'b';

  // switch cases
  switch (py_signal) {
    // case move forward
    case 'a' : {
        // call functions
        updated_heading = get_updated_heading();

        // display the integer point data
        Serial.print("Updated X: ");
        Serial.print(updated_heading);
        Serial.println("");

        // delay before requesting next data
        delay(BNO055_SAMPLERATE_DELAY_MS);

        heading_delta = get_heading_delta(updated_heading);

        // display the integer point data
        Serial.print("Delta X: ");
        Serial.print(heading_delta);
        Serial.println("");

        // delay before requesting next data
        delay(BNO055_SAMPLERATE_DELAY_MS);

        //  turn = move_turn(heading_delta);
        //
        //  // display the integer point data
        //  Serial.print("Turn X: ");
        //  Serial.print(turn);
        //  Serial.println("");
        //
        //  // delay before requesting next data
        //  delay(BNO055_SAMPLERATE_DELAY_MS);

        fwd = move_fwd(heading_delta);

        // display the integer point data
        Serial.print("Fwd X: ");
        Serial.print(fwd);
        Serial.println("");

        // delay before requesting next data
        delay(BNO055_SAMPLERATE_DELAY_MS);
      }
      break;
    case 'b': {
        // call functions
        updated_heading = get_updated_heading();

        // display the integer point data
        Serial.print("Updated X: ");
        Serial.print(updated_heading);
        Serial.println("");

        // delay before requesting next data
        delay(BNO055_SAMPLERATE_DELAY_MS);

        heading_delta = get_heading_delta(updated_heading);

        // display the integer point data
        Serial.print("Delta X: ");
        Serial.print(heading_delta);
        Serial.println("");

        // delay before requesting next data
        delay(BNO055_SAMPLERATE_DELAY_MS);

        //  turn = move_turn(heading_delta);
        //
        //  // display the integer point data
        //  Serial.print("Turn X: ");
        //  Serial.print(turn);
        //  Serial.println("");
        //
        //  // delay before requesting next data
        //  delay(BNO055_SAMPLERATE_DELAY_MS);

        turn = move_turn(heading_delta);

        // display the integer point data
        Serial.print("Turn X: ");
        Serial.print(turn);
        Serial.println("");

        // delay before requesting next data
        delay(BNO055_SAMPLERATE_DELAY_MS);
      }
      break;
      default:
      break;
  }
}

/////////////////////////////////////////////////////////////////////////////////////////
//  User defined functions
/////////////////////////////////////////////////////////////////////////////////////////

// get the updated heading reading
int get_updated_heading() {
  int updated_heading;
  sensors_event_t event;
  bno.getEvent(&event);
  updated_heading = event.orientation.x;
  return updated_heading;
}

// calculate the heading delta
int get_heading_delta(int updated_heading) {
  int heading_delta;
  if (updated_heading > 180) {
    updated_heading = updated_heading - 360;
    heading_delta = updated_heading;
  }
  else {
    heading_delta = updated_heading;
  }
  return heading_delta;
}

// move forward with l/r compensation
char move_fwd(int heading_delta) {
  char left = 'l';
  char right = 'r';
  char forward = 'n';
  if (heading_delta > 5 && heading_delta < 180) {
    // send command to turn left to correct
    return left;
  }
  if (heading_delta < -5 && heading_delta > -180) {
    // send command to turn right to correct
    return right;
  }
  else {
    // send command to continue forward
    return forward;
  }
}

// turn l/r, indicate when done
char move_turn(int heading_delta) {
  char continue_turn = 'g';
  char end_turn = 'e';
  while (abs(heading_delta) < 90) {
    return continue_turn;
  }
  return end_turn;
}

