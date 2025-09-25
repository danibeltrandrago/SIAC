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
float yaw = 0;
short datax=0;
short datay=0;
short dataz=0;

//motor
volatile signed int ISRCounter=0;//used inside the ISR
float angle=0;//used inside the controller
float r=90;//reference angle to move the motor
float u=0;

//PID Variables
double Kp=10, Ki=0.01, Kd=1;

double error = 0;
double lastError = 0;
double cumulativeError = 0;
double rateError = 0;
double elapsedTime = 0;
double previousTime = 0;

// function prototypes
void ISREncoderA( void );// one hw interrupt driven isr to read the encoder pulse, the isr resumes the task
void ISREncoderB( void );// another hw interrupt driven isr to read the other encoder pulse, the isr resumes the task
void TaskIMU(void);//Gets data from Inertial Measurement Unit 
void TaskControl(void);//Computes the controller
void TaskSupervison();//Sends data to the host PC via serial communication
void computePID();

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
	yaw = 360.0/6.2832*atan2(datax,datay);
	r = roll;
}

void TaskControl(void)
{
	//Aqui conozco el angulo actual del motor
	angle = (float)ISRCounter/4.0;//read input
	long int currentTime = millis();
	elapsedTime = 0.01;//(long int)(currentTime - previousTime);
	previousTime = currentTime;

	error = r - angle;
	cumulativeError += error * elapsedTime;
	rateError = (error - lastError) / elapsedTime;
	u = Kp * error+ Ki* cumulativeError;// + Kd * rateError;
	lastError = error;

	//check for motor direction
	if (u<=0)
	{
		//turn CW
		digitalWrite(pinDir, LOW);
	}else{
		//turn CCW
		digitalWrite(pinDir, HIGH);
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

	Serial.print(u);
	Serial.print(", ");
	Serial.println(angle);
	
	analogWrite(pinPWM,u); //update output
}

void TaskSupervison()
{
}