/*
Quanser QUBE Servo 2 Core Library

Created 2016 by Quanser Inc.
www.quanser.com
*/

#include <Arduino.h>
#include <SPI.h>
#include "QUBEServo2.h"

//Status Bit Masks
const byte stallErrorMask = B00000100;
const byte stallDetectedMask = B00000010;
const byte amplifierFaultMask = B00000001;

//Serial Variables
static String dData;  // string that will be printed to the Arduino Serial Monitor
static boolean dDataReady;  // true when there is a string ready to be printed
static int dDataIndex;  // used to print the string one character at a time

//Create the display class
Display::Display()
{
	this->dData = "";
	this->dDataReady = false;
	this->dDataIndex = 0;

	// reserve 256 bytes for the string that will be printed to the Arduino Serial Monitor
	dData.reserve(256);
}


void Display::buildString(float theta, float alpha, float currentSense, int moduleID, int moduleStatus)
{

	if (this->dDataReady == false) {
		// assemble the string to be printed to the Arduino Serial Monitor
		// (Note that the String() conversion function is time consuming.  If more data
		// needs to displayed than in this example, it may be necessary to assemble the
		// displayData string over multiple sample periods.)

		reset(moduleID, moduleStatus);

		this->dData += "\r\nArm angle (deg): ";
		float thetaDeg = theta * (180.0 / M_PI);
		this->dData += String(round(thetaDeg));

		this->dData += "\r\nPendulum angle (deg): ";
		float alphaDeg = alpha * (180.0 / M_PI);
		this->dData += String(round(alphaDeg));

		this->dData += "\r\nCurrent (mA): ";
		float currentSenseAmps = (3.3 / (4.0 * 4095.0)) * ((currentSense / 2.0) - 4095.0);
		this->dData += String(currentSenseAmps * 1000);
		this->dData += "\r\n\n";

		this->dDataReady = true;  // the string is ready to be printed
		this->dDataIndex = 0;
	}
}

void Display::reset(int moduleID, int moduleStatus)
{
	this->dData = "";  // clear the string

	if (moduleID == -1) {
		this->dData += "Module ID: No module detected";
	}
	else {
		this->dData += "Module ID: ";
		this->dData += String(moduleID);
	}

  if (moduleStatus == 0) {
    this->dData += "\r\nStatus: Good";  // '\r' is carriage return and '\n' is new line
  }
  else if (moduleStatus & amplifierFaultMask) {
		this->dData += "\r\nStatus: Amplifier Fault";
	}
  else if (moduleStatus & stallErrorMask) {
    this->dData += "\r\nStatus: Stall Error";
  }
  else if (moduleStatus & stallDetectedMask) {
    this->dData += "\r\nStatus: Stall Detected";
  }
	else {
		this->dData += "\r\nStatus: ";
		this->dData += String(moduleStatus);
	}
}

// initialize the SPI data to be written
byte mode = 1;                      // normal mode = 1
byte writeMask = B00011111;         // Bxxxxxx11 to enable the motor, Bxxx111xx to enable the LEDs, Bx11xxxxx to enable writes to the encoders
byte LEDRedMSB = 0;                 // red LED command MSB
byte LEDRedLSB = 0;                 // red LED command LSB
byte LEDGreenMSB = 0;               // green LED command MSB
byte LEDGreenLSB = 0;               // green LED command LSB
byte LEDBlueMSB = 0;                // blue LED command MSB
byte LEDBlueLSB = 0;                // blue LED command LSB
byte encoder0ByteSet[3] = {0,0,0};  // encoder0 is set to this value only when writes are enabled with writeMask
byte encoder1ByteSet[3] = {0,0,0};  // encoder1 is set to this value only when writes are enabled with writeMask
byte motorMSB = 0x80;               // motor command MSB must be B1xxxxxxx to enable the amplifier
byte motorLSB = 0;                  // motor command LSB

// initialize the SPI data to be read
byte moduleIDMSB = 0;               // module ID MSB (module ID for the QUBE Servo is 777 decimal)
byte moduleIDLSB = 0;               // module ID LSB
byte encoder0Byte[3] = {0,0,0};     // arm encoder counts
byte encoder1Byte[3] = {0,0,0};     // pendulum encoder counts
byte tach0Byte[3] = {0,0,0};        // arm tachometer
byte moduleStatus = 0;              // module status (the QUBE Servo sends status = 0 when there are no errors)
byte currentSenseMSB = 0;           // motor current sense MSB 
byte currentSenseLSB = 0;           // motor current sense LSB

// global variables for LED intensity (999 is maximum intensity, 0 is off)
//int LEDRed = 0;
//int LEDGreen = 0;
//int LEDBlue = 0;
extern int LEDRed;
extern int LEDGreen;
extern int LEDBlue;
extern float motorPWM;
// This function is used to clear the stall error and reset the encoder values to 0.
// The motor and LEDs are turned off when this function is called.

// set pin 10 as the slave select for the Quanser QUBE
// (Note that if a different pin is used for the slave select, pin 10 should be set as
// an output to prevent accidentally putting the Arduino UNO into slave mode.)
const int slaveSelectPin = 10;

void resetQUBEServo() {
  
  // enable the motor and LEDs, and enable writes to the encoders
  writeMask = B01111111;
  
  // turn off the LEDs
  LEDRedMSB = 0;
  LEDRedLSB = 0;
  LEDGreenMSB = 0;
  LEDGreenLSB = 0;
  LEDBlueMSB = 0;
  LEDBlueLSB = 0;
  
  // reset the encoder values to 0
  encoder0ByteSet[2] = 0;
  encoder0ByteSet[1] = 0;
  encoder0ByteSet[0] = 0;
  encoder1ByteSet[2] = 0;
  encoder1ByteSet[1] = 0;
  encoder1ByteSet[0] = 0;
  
  // turn off the motor, and clear the stall error by disabling the amplifier
  motorMSB = 0;  // motor command MSB is B0xxxxxxx to disable the amplifier
  motorLSB = 0;
  
  // initialize the SPI bus using the defined speed, data order and data mode
  SPI.beginTransaction(SPISettings(1000000, MSBFIRST, SPI_MODE2));
  digitalWrite(slaveSelectPin, HIGH);  // take the slave select pin high to de-select the device
  digitalWrite(slaveSelectPin, LOW);   // take the slave select pin low to select the device
  
  // send and receive the data via SPI
  moduleIDMSB = SPI.transfer(mode);                    // read the module ID MSB, send the mode
  moduleIDLSB = SPI.transfer(0);                       // read the module ID LSB
  encoder0Byte[2] = SPI.transfer(writeMask);           // read encoder0 byte 2, send the write mask
  encoder0Byte[1] = SPI.transfer(LEDRedMSB);           // read encoder0 byte 1, send the red LED MSB
  encoder0Byte[0] = SPI.transfer(LEDRedLSB);           // read encoder0 byte 0, send the red LED LSB
  encoder1Byte[2] = SPI.transfer(LEDGreenMSB);         // read encoder1 byte 2, send the green LED MSB
  encoder1Byte[1] = SPI.transfer(LEDGreenLSB);         // read encoder1 byte 1, send the green LED LSB
  encoder1Byte[0] = SPI.transfer(LEDBlueMSB);          // read encoder1 byte 0, send the blue LED MSB
  tach0Byte[2] = SPI.transfer(LEDBlueLSB);             // read tachometer0 byte 2, send the blue LED LSB
  tach0Byte[1] = SPI.transfer(encoder0ByteSet[2]);     // read tachometer0 byte 1, send encoder0 byte 2
  tach0Byte[0] = SPI.transfer(encoder0ByteSet[1]);     // read tachometer0 byte 0, send encoder0 byte 1
  moduleStatus = SPI.transfer(encoder0ByteSet[0]);     // read the status, send encoder0 byte 0
  currentSenseMSB = SPI.transfer(encoder1ByteSet[2]);  // read the current sense MSB, send encoder1 byte 2
  currentSenseLSB = SPI.transfer(encoder1ByteSet[1]);  // read the current sense LSB, send encoder1 byte 1
  SPI.transfer(encoder1ByteSet[0]);                    // send encoder1 byte 0
  SPI.transfer(motorMSB);                              // send the motor MSB
  SPI.transfer(motorLSB);                              // send the motor LSB
  
  digitalWrite(slaveSelectPin, HIGH);  // take the slave select pin high to de-select the device
  SPI.endTransaction();
  
  writeMask = B00011111;  // enable the motor and LEDs, disable writes to the encoders
  motorMSB = 0x80;  // enable the amplifier
}

float theta = 0;//horizontal angle
float alpha = 0;//pendulum angle
float currentSense = 0;
// set pin 10 as the slave select for the Quanser QUBE
// (Note that if a different pin is used for the slave select, pin 10 should be set as
// an output to prevent accidentally putting the Arduino UNO into slave mode.)
void getQUBEData()
{
   // initialize the SPI bus using the defined speed, data order and data mode
    //SPI.beginTransaction(SPISettings(1000000, MSBFIRST, SPI_MODE2));
    // take the slave select pin low to select the device
    digitalWrite(slaveSelectPin, LOW);
    
    // send and receive the data via SPI (except for the motor command, which is sent after the pendulum control code) 
    moduleIDMSB = SPI.transfer(mode);                    // read the module ID MSB, send the mode
    moduleIDLSB = SPI.transfer(0);                       // read the module ID LSB
    encoder0Byte[2] = SPI.transfer(writeMask);           // read encoder0 byte 2, send the write mask
    encoder0Byte[1] = SPI.transfer(LEDRedMSB);           // read encoder0 byte 1, send the red LED MSB
    encoder0Byte[0] = SPI.transfer(LEDRedLSB);           // read encoder0 byte 0, send the red LED LSB
    encoder1Byte[2] = SPI.transfer(LEDGreenMSB);         // read encoder1 byte 2, send the green LED MSB
    encoder1Byte[1] = SPI.transfer(LEDGreenLSB);         // read encoder1 byte 1, send the green LED LSB
    encoder1Byte[0] = SPI.transfer(LEDBlueMSB);          // read encoder1 byte 0, send the blue LED MSB
    tach0Byte[2] = SPI.transfer(LEDBlueLSB);             // read tachometer0 byte 2, send the blue LED LSB
    tach0Byte[1] = SPI.transfer(encoder0ByteSet[2]);     // read tachometer0 byte 1, send encoder0 byte 2
    tach0Byte[0] = SPI.transfer(encoder0ByteSet[1]);     // read tachometer0 byte 0, send encoder0 byte 1
    moduleStatus = SPI.transfer(encoder0ByteSet[0]);     // read the status, send encoder0 byte 0
    currentSenseMSB = SPI.transfer(encoder1ByteSet[2]);  // read the current sense MSB, send encoder1 byte 2
    currentSenseLSB = SPI.transfer(encoder1ByteSet[1]);  // read the current sense LSB, send encoder1 byte 1
    SPI.transfer(encoder1ByteSet[0]);                    // send encoder1 byte 0
/*Module ID*/
    int moduleID = (moduleIDMSB << 8) | moduleIDLSB;
    
    /*Motor Encoder Counts*/
    long encoder0 = ((long)encoder0Byte[2] << 16) | (long)(encoder0Byte[1] << 8) | encoder0Byte[0];
    if (encoder0 & 0x00800000) {
      encoder0 = encoder0 | 0xFF000000;
    }
    // convert the arm encoder counts to angle theta in radians
    theta = (float)encoder0 * (-2.0 * M_PI / 2048);

    /*Pendulum Encoder Counts*/
    long encoder1 = ((long)encoder1Byte[2] << 16) | (long)(encoder1Byte[1] << 8) | encoder1Byte[0];
    if (encoder1 & 0x00800000) {
      encoder1 = encoder1 | 0xFF000000;
    }
    // wrap the pendulum encoder counts when the pendulum is rotated more than 360 degrees
    encoder1 = encoder1 % 2048;
    if (encoder1 < 0) {
      encoder1 = 2048 + encoder1;
    }    
    // convert the pendulum encoder counts to angle alpha in radians
    alpha = (float)encoder1 * (2.0 * M_PI / 2048) - M_PI;

    /*Current Sense Value*/
    currentSense = (currentSenseMSB << 8) | currentSenseLSB;

  // convert the LED intensities to MSB and LSB
    LEDRedMSB = (byte)(LEDRed >> 8);
    LEDRedLSB = (byte)(LEDRed & 0x00FF);
    LEDGreenMSB = (byte)(LEDGreen >> 8);
    LEDGreenLSB = (byte)(LEDGreen & 0x00FF);
    LEDBlueMSB = (byte)(LEDBlue >> 8);
    LEDBlueLSB = (byte)(LEDBlue & 0x00FF);

      int motor = (int)motorPWM;  // convert float to int (2 bytes)
      motor = motor | 0x8000;  // motor command MSB must be B1xxxxxxx to enable the amplifier
      motorMSB = (byte)(motor >> 8);
      motorLSB = (byte)(motor & 0x00FF);   
    // send the motor data via SPI
    SPI.transfer(motorMSB);
    SPI.transfer(motorLSB);
    
    // take the slave select pin high to de-select the device
    digitalWrite(slaveSelectPin, HIGH);
    SPI.endTransaction();

}
