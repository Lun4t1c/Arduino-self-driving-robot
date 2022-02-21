/*
Arduino self driving robot library

USAGE:
initialize Car instance according to variables in constructor
put Car::setup() in Arduino setup()
put Car::travelLoop() in Arduino loop()

and you are ready to go
*/

#ifndef CarControl_h
#define CarControl_h

#pragma region configuration
#define defMS 3
#define TOP_SPEED 255
#define SERVO_NEUTRAL 100	//position in which servo is considered to be looking straight forward
#define STOP_DISTANCE 70 	//minimal distance, which makes car stop
#define START_DISTANCE 100	//minimal distance worth travelling
#define AREA_SCAN_MIN 1		//position in which servo is maximally turned to the left
#define AREA_SCAN_MAX 179	//position in which servo is maximally turned to the right
#define MAX_ULTRASONIC_DISTANCE 300	//max ultrasonic range in cm
#define TURN_DELAY 1000		//miliseconds required to turn car 90 degrees as exactly as possible
#define PANIC_STOP_THRESHOLD 100	//min forward distance scans difference which activates immediate stop
#define STUCK_CONTROL_THRESHOLD 15 //how many times forward scan has to be same or similiar to assume car is stuck
#pragma endregion

#include "Arduino.h"
#include <Servo.h>
#include <Ultrasonic.h>

class Car {
private:
	int IN1, IN2, ENA,
		IN3, IN4, ENB,
		ServoPin;
	uint8_t UsTrigger, UsEcho, 
		BuzzerPin;

	uint8_t LedRed,
		LedGreen,
		LedBlue;

	Servo servo;
	Ultrasonic *pUltrasonic;

	int stuckControlCounter = 0;


	//data
	int currentSpeed = 0;
	int *pLastAreaScan = nullptr;
	int lastForwardScan;
	enum directions { LEFT, RIGHT, FORWARD, BACKWARD, NULL_SCAN };

	//states	
	enum states { IDLE, TRAVEL, PLAYFUL, DISORIENTED, STARTLED };
	Car::states currentState;

public:
	Car(int in1, int in2, int ena,
		int in3, int in4, int enb,
		int servopin,
		uint8_t ustrigger, uint8_t usecho,
		uint8_t buzzerpin,
		uint8_t ledred = -1, uint8_t ledgreen = -1, uint8_t ledblue = -1);


	#pragma region Basic movement
	void slowStart(int = defMS);
	void slowStop(int = defMS);
	void turnLeft();
	void turnRight();
	void turnAround();
	void start();
	void stop();
	void goBackward();
	void turnLeftSingleTrack();
	void turnRightSingleTrack();
	#pragma endregion


	#pragma region Travel
	void travelForward();
	void travelLeft();
	void travelRight();
	void travelBackward();
	#pragma endregion


	#pragma region Orientation
	int getDistance();

	int peekLeft();
	int peekRight();
	void scanArea();
	void resetServo();
	Car::directions determineDirection();
	#pragma endregion


	#pragma region Interactions
	void switchState(Car::states);
	void recolorLed(Car::states);
	void soundScreech(int ms = 350);
	void soundWoow(int ms = 350);
	#pragma endregion


	#pragma region Utils
	int* getLastAreaScan();

	void setup();	

	void leftTrackForward();
	void leftTrackBackward();
	void leftTrackStop();

	void rightTrackForward();
	void rightTrackBackward();
	void rightTrackStop();

	void playAround();	
	#pragma endregion

	void travelLoop();
	void determineDirectionAndTravel();

	void testFunc();
};

#endif