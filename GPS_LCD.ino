/*
This is a combination of Adafruit's GPS and LCD library example code to read data
from an Adafruit Ultimate GPS (https://learn.adafruit.com/adafruit-ultimate-gps/overview) and
output values to a 16x2 LCD.

Line1: time
Line2: GPS info, toggled by pressing the right button on the GPS Pi Plate.

Kevin Hooke / @kevinhooke
*/
#include <Wire.h>
#include <Adafruit_MCP23017.h>
#include <Adafruit_RGBLCDShield.h>
#include <Adafruit_GPS.h>
#include <SoftwareSerial.h>

//colors
#define OFF 0x0
#define RED 0x1
#define YELLOW 0x3
#define GREEN 0x2
#define TEAL 0x6
#define BLUE 0x4
#define VIOLET 0x5
#define WHITE 0x7 

SoftwareSerial mySerial(3, 2);

Adafruit_GPS GPS(&mySerial);

// Set GPSECHO to 'false' to turn off echoing the GPS data to the Serial console
// Set to 'true' if you want to debug and listen to the raw GPS sentences.
#define GPSECHO  true

boolean usingInterrupt = false;
void useInterrupt(boolean); // Func prototype keeps Arduino 0023 happy

//lcd init
Adafruit_RGBLCDShield lcd = Adafruit_RGBLCDShield();

//combinations of values to display based on button press
short buttonState = 0;

void setup()
{
  // connect at 115200 so we can read the GPS fast enough and echo without dropping chars
  Serial.begin(115200);
 
  // 9600 NMEA is the default baud rate for Adafruit MTK GPS's- some use 4800
  GPS.begin(9600);

  // uncomment this line to turn on RMC (recommended minimum) and GGA (fix data) including altitude
  GPS.sendCommand(PMTK_SET_NMEA_OUTPUT_RMCGGA);
  // uncomment this line to turn on only the "minimum recommended" data
  //GPS.sendCommand(PMTK_SET_NMEA_OUTPUT_RMCONLY);

  // Set the update rate
  GPS.sendCommand(PMTK_SET_NMEA_UPDATE_1HZ);   // 1 Hz update rate
 
  // Request updates on antenna status, comment out to keep quiet
  GPS.sendCommand(PGCMD_ANTENNA);

  //interrupt for reading gps data
  useInterrupt(true);

  delay(1000);
  // Ask for firmware version
  mySerial.println(PMTK_Q_RELEASE);

  // set up the LCD's number of columns and rows:
  lcd.begin(16, 2);
}


// Interrupt is called once a millisecond, looks for any new GPS data, and stores it
SIGNAL(TIMER0_COMPA_vect) {
  char c = GPS.read();
  // if you want to debug, this is a good time to do it!
#ifdef UDR0
  if (GPSECHO)
    if (c) UDR0 = c;
  // writing direct to UDR0 is much much faster than Serial.print
  // but only one character can be written at a time.
#endif
}

void useInterrupt(boolean v) {
  if (v) {
    // Timer0 is already used for millis() - we'll just interrupt somewhere
    // in the middle and call the "Compare A" function above
    OCR0A = 0xAF;
    TIMSK0 |= _BV(OCIE0A);
    usingInterrupt = true;
  } else {
    // do not call the interrupt function COMPA anymore
    TIMSK0 &= ~_BV(OCIE0A);
    usingInterrupt = false;
  }
}

uint32_t timer = millis();
void loop()                     // run over and over again
{

  // in case you are not using the interrupt above, you'll
  // need to 'hand query' the GPS, not suggested :(
  if (! usingInterrupt) {
    // read data from the GPS in the 'main loop'
    char c = GPS.read();
    // if you want to debug, this is a good time to do it!
    if (GPSECHO)
      if (c) Serial.print(c);
  }

  // if a sentence is received, we can check the checksum, parse it...
  if (GPS.newNMEAreceived()) {
    // a tricky thing here is if we print the NMEA sentence, or data
    // we end up not listening and catching other sentences!
    // so be very wary if using OUTPUT_ALLDATA and trytng to print out data
    //Serial.println(GPS.lastNMEA());   // this also sets the newNMEAreceived() flag to false

    if (!GPS.parse(GPS.lastNMEA()))   // this also sets the newNMEAreceived() flag to false
      return;  // we can fail to parse a sentence in which case we should just wait for another
  }

  // if millis() or timer wraps around, we'll just reset it
  if (timer > millis()){
    timer = millis();
  }

  //check for button press on the LCD Pi Plate
  uint8_t buttons = lcd.readButtons();

  if (buttons) {
    
    if(buttons & BUTTON_UP){
      lcd.setBacklight(WHITE);
    }

    if(buttons & BUTTON_DOWN){
      lcd.setBacklight(OFF);    
    }
    
    //if button pressed, cycle through display combinations
    if (buttons & BUTTON_RIGHT) {
      lcd.clear();
      if (buttonState >= 2) {
        buttonState = 0;
      }
      else {
        buttonState++;
      }
    }
  }
  
  // update lcd every approx 1 sec
  if (millis() - timer > 1000) {
    timer = millis(); // reset the timer

    String line1 = "";
    String line2 = "";
    if (GPS.fix) {
      if (buttonState == 0) {
        line1 = concatTime(line1);
        line2.concat(GPS.latitudeDegrees);
        line2.concat(", ");
        line2.concat(GPS.longitudeDegrees);
      }
      else if (buttonState == 1) {
        line1 = concatTime(line1);
        line2.concat("Speed (knots): ");
        line2.concat(GPS.speed);
      }
      else {
        line1.concat(GPS.latitude);
        line1.concat(GPS.lat);
        line2.concat(GPS.longitude);
        line2.concat(GPS.lon);
      }
    }
    else {
      line2 = "Waiting for fix...";
    }

    //lcd print
    lcd.setCursor(0, 0);
    lcd.print(line1);
    lcd.setCursor(0, 1);
    lcd.print(line2);

    //write available data to serial out for debugging
    Serial.print("\nTime: ");
    Serial.print(GPS.hour, DEC); Serial.print(':');
    Serial.print(GPS.minute, DEC); Serial.print(':');
    Serial.print(GPS.seconds, DEC); Serial.print('.');
    Serial.println(GPS.milliseconds);
    Serial.print("Date: ");
    Serial.print(GPS.day, DEC); Serial.print('/');
    Serial.print(GPS.month, DEC); Serial.print("/20");
    Serial.println(GPS.year, DEC);
    Serial.print("Fix: "); Serial.print((int)GPS.fix);
    Serial.print(" quality: "); Serial.println((int)GPS.fixquality);

    if (GPS.fix) {
      Serial.print("Location: ");
      Serial.print(GPS.latitude, 4); Serial.print(GPS.lat);
      Serial.print(", ");
      Serial.print(GPS.longitude, 4); Serial.println(GPS.lon);
      Serial.print("Location (in degrees, works with Google Maps): ");
      Serial.print(GPS.latitudeDegrees, 4);
      Serial.print(", ");
      Serial.println(GPS.longitudeDegrees, 4);

      Serial.print("Speed (knots): "); Serial.println(GPS.speed);
      Serial.print("Angle: "); Serial.println(GPS.angle);
      Serial.print("Altitude: "); Serial.println(GPS.altitude);
      Serial.print("Satellites: "); Serial.println((int)GPS.satellites);
    }
  }
}

//builds the time line
String concatTime(String time) {
  time.concat(padWithZero(GPS.hour));
  time.concat(':');
  time.concat(padWithZero(GPS.minute));
  time.concat(':');
  time.concat(padWithZero(GPS.seconds));
  time.concat(" UTC");
  return time;
}

//pads value with leading zero if length is 1
String padWithZero(int value){
  String result = String(value);
  if(result.length() == 1){
    result = "0" + result;
  }
  return result;
}
