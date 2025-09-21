#include <Arduino.h>
#include "MPU6050.h"
#include "Seeed_vl53l0x.h"

//pin mapping
#define pinEncoderA (uint8_t)19//attach yellow wire from the motor to pin 2 of arduino
#define pinEncoderB (uint8_t)18//attach purple wire from the motor to pin 19 of arduino
#define pinDir (uint8_t)12//13
//#define pinDirInv (uint8_t)11//old red keyes L298 boards require complementary value in IN1 and IN2 pins
#define pinPWM (uint8_t)3//11
#define pinBrake (uint8_t)9//8
#define pinCurrent (uint8_t)A0//A1

MPU6050 IMU;//Inertial Measurement Unit
Seeed_vl53l0x VL53L0X; //TimeOfFlight distance sensor
// EthernetUDP Udp;// An EthernetUDP instance to send and receive packets over UDP

//Global variables
float pitch=0;
float roll=0;
int datax=0;
int datay=0;
int dataz=0;

//motor
volatile signed int ISRCounter=0;//used inside the ISR
float angle=0;//used inside the controller
float r=0;//reference angle to move the motor
float u=0;

// function prototypes
void ISREncoderA( void );// one hw interrupt driven isr to read the encoder pulse, the isr resumes the task
void ISREncoderB( void );// another hw interrupt driven isr to read the other encoder pulse, the isr resumes the task
void TaskIMU(void);//Gets data from Inertial Measurement Unit 
void TaskControl(void);//Computes the controller
void TaskSupervison();//Sends data to the host PC via serial communication

void setup() 
{
	pinMode(LED_BUILTIN,OUTPUT);

	pinMode(pinEncoderA, INPUT_PULLUP);
	pinMode(pinEncoderB, INPUT_PULLUP);
	pinMode(pinDir, OUTPUT);
	digitalWrite(pinDir,1);
	pinMode(pinPWM, OUTPUT);
	digitalWrite(pinPWM,0);
	pinMode(pinBrake, OUTPUT);
	digitalWrite(pinBrake,0);
	pinMode(pinCurrent, INPUT);

	Wire.begin();
	IMU.initialize();

	Serial.begin(115200);
 
	attachInterrupt(digitalPinToInterrupt(pinEncoderA), ISREncoderA, CHANGE);
	attachInterrupt(digitalPinToInterrupt(pinEncoderB), ISREncoderB, CHANGE);
}

void loop() 
{
	unsigned long initTime=micros();
	digitalWrite(LED_BUILTIN,digitalRead(LED_BUILTIN)^1);

	TaskIMU();
	TaskControl();
	TaskSupervison();

	delayMicroseconds(10000-(micros()-initTime));
}

//depending on encoder state to check direction, increments/decrements a counter
void ISREncoderA(void)  
{
  if (digitalRead(pinEncoderA) == digitalRead(pinEncoderB))
    {
       ISRCounter++;
    }
    else
    {
       ISRCounter--;
    }
}

//depending on encoder state to check direction, increments/decrements a counter
void ISREncoderB(void )  
{
  if (digitalRead(pinEncoderA) != digitalRead(pinEncoderB))
    {
       ISRCounter++;
    }
    else
    {
       ISRCounter--;
    }
}

void TaskIMU(void)
{
	IMU.getAcceleration(&datax, &datay, &dataz);
	pitch = -360.0/6.2832*atan2(-datax,dataz);
	roll =  360.0/6.2832*atan2(datay,dataz);
}

void TaskControl(void)
{
	// r=?;

	angle = (float)ISRCounter/4.0;//read input

	//PID computation
	// u=P+I+D;//compute PID output

	//check for motor direction
	if (u<=0)
	{
		//turn CW
	}else{
		//turn CCW
	}
	//check for saturation
	if (u>255.0)
	{
		u=255.0;
	}
	if (u<-255.0)
	{
		u=-255.0;
	}

	// analogWrite(pinPWM,?);//update output
}

void TaskSupervison()
{
	Serial.print((float)r);
	Serial.print(",");
	Serial.print((float)roll);
	Serial.println();
}
