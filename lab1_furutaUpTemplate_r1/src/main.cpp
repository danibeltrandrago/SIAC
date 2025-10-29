#include <Arduino.h>
#include <SPI.h>// include the SPI library and the math library

//Global variables
float h=0.001;//sampling time in seconds
float r=0.5;//reference
float Nx[4] = {1,0,0,0};
float Nu = 0;
float theta=0;//measured angle for the horizontal link (motor) in radians
float thetaKMinus1=0;
float thetaDot=0;
float thetaDotFiltered=0;
float alpha=0;//measured angle for the vertical link (pendulum) in radians
float alphaKMinus1=0;
float alphaDot=0;
float alphaDotFiltered=0;
float u=0;
float ueq = 0;
float K[4]={-1.7027,-0.7828,19.5906,1.4513};//put your gains here
float x[4]={0,0,0,0};
float xeq[4] = {0,0,0,0};
float motorVoltage=0;
float currentSense=0;
float taskSupervisionCounter=0;

unsigned long old_time = 0;

// initialize the SPI data to be written
const int slaveSelectPin = 10;// set pin 10 as the slave select for the Quanser QUBE (Note that if a different pin is used for the slave select, pin 10 should be set as an output to prevent accidentally putting the Arduino UNO into slave mode.)
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
int LEDRed = 0;
int LEDGreen = 0;
int LEDBlue = 0;

// function prototypes
void resetQUBEServo2(void);
void TaskQUBEServo2Data(void);//Gets data from Inertial Measurement Unit 
void TaskControl(void);//Computes the controller
void TaskSupervison(void);//Sends data to the host PC via serial communication

void setup() 
{
  pinMode(LED_BUILTIN,OUTPUT);
  Serial.begin(115200);// initialize serial communication

  pinMode (slaveSelectPin, OUTPUT);// set the SPI slaveSelectPin as an output
  SPI.begin();// initialize SPI  
  resetQUBEServo2();//init QUBE Servo 2 rotary pendulum
  old_time = millis();
}

void loop() 
{  
	unsigned long initTime=micros();

	TaskQUBEServo2Data();
	TaskControl();
	TaskSupervison();

	delayMicroseconds(h*1000000-(micros()-initTime));
}

void TaskControl(void)
{	
  if(millis() - old_time > 1000) 
  {
    r = r == 0.5 ? -0.5 : 0.5;
    old_time = millis();
  }

  thetaDot=(theta-thetaKMinus1)/0.001;
  alphaDot=(alpha-alphaKMinus1)/0.001;
  thetaDotFiltered=(1.0-0.9512)*thetaDot+0.9512*thetaDotFiltered;
  alphaDotFiltered=(1.0-0.9512)*alphaDot+0.9512*alphaDotFiltered;
  thetaKMinus1=theta;
  alphaKMinus1=alpha;

  x[0]=theta - xeq[0];
  x[1]=thetaDotFiltered - xeq[1];
  x[2]=alpha - xeq[2];
  x[3]=alphaDotFiltered - xeq[3];

  for (int i = 0; i < 4; i++)
  {
    x[i] = Nx[i]*r - x[i];
  }

  if ( (alpha>(-30.0*2.0*PI/360.0)) &&  (alpha<(30.0*2.0*PI/360.0)) )//the pendulum is within the convergence zone to apply control
  {
    u = K[0]*x[0] + K[1]*x[1] + K[2]*x[2] + K[3]*x[3];
    u += r*Nu;
    u += ueq;
    LEDRed = 100;
    LEDGreen = 1000-max(0,fabs(r-theta)*250);
    LEDBlue = min(999,fabs(r-theta)*250); 
  }else{
    u=0; //the pendulum falls down
    LEDRed = 800;
    LEDGreen = 150;
    LEDBlue = 150; 
  }
  
	//check for saturation
	if (u>15.0)
	{
		u=15.0;
	}
	if (u<-15.0)
	{
		u=-15.0;
	}
  motorVoltage=-u; //Note that u>0 means rotating CCW according to the documentation. TODO: double check it!!!-->Done
}

void TaskSupervison(void)
{  
  taskSupervisionCounter++;
  if (taskSupervisionCounter==200)
  {   
    Serial.print((float)r);
    Serial.print(",");
    Serial.print((float)theta);
    Serial.print(",");
    Serial.print((float)alpha);
    Serial.print(",");
    Serial.print((float)u);    
    Serial.println();
    taskSupervisionCounter=0;
  }
}

// This function is used to clear the stall error and reset the encoder values to 0.
// The motor and LEDs are turned off when this function is called.
void resetQUBEServo2(void) 
{
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

//data is both get and sent in this function using spi data communication
void TaskQUBEServo2Data(void)
{
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

    float motorPWM = motorVoltage * (625.0 / 15.0);
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
