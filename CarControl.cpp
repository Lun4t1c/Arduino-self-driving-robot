#include "Arduino.h"
#include "CarControl.h"

Car::Car(int in1, int in2, int ena,
		int in3, int in4, int enb,
		int servopin,
		uint8_t ustrigger, uint8_t usecho,
		uint8_t buzzerpin,
		uint8_t ledred, uint8_t ledgreen, uint8_t ledblue) {

	Serial.begin(9600);
	Serial.println("Initializing car instance...");

	IN1 = in1;
	IN2 = in2;
	ENA = ena;
	IN3 = in3;
	IN4 = in4;
	ENB = enb;
	ServoPin = servopin;
	UsTrigger = ustrigger;
	UsEcho = usecho;
	BuzzerPin = buzzerpin;
	

	pinMode(IN1, OUTPUT);
	pinMode(IN2, OUTPUT);
	pinMode(ENA, OUTPUT);

	pinMode(IN3, OUTPUT);
	pinMode(IN4, OUTPUT);
	pinMode(ENB, OUTPUT);

	digitalWrite(IN1, LOW);
	digitalWrite(IN2, LOW);
	digitalWrite(ENA, LOW);
	digitalWrite(IN3, LOW);
	digitalWrite(IN4, LOW);
	digitalWrite(ENB, LOW);

	pinMode(BuzzerPin, OUTPUT);
	digitalWrite(BuzzerPin, LOW);

		
	LedRed = ledred;
	LedGreen = ledgreen;
	LedBlue = ledblue;

	pinMode(LedRed, OUTPUT);
	pinMode(LedGreen, OUTPUT);
	pinMode(LedBlue, OUTPUT);

}

#pragma region Basic movement
//start car slowly
void Car::slowStart(int ms) {
	Serial.println("Slow starting car...");
	digitalWrite(IN1, HIGH);
	digitalWrite(IN3, HIGH);
	for (int i = 0; i <= TOP_SPEED; i++) {
		analogWrite(ENA, i);
		analogWrite(ENB, i);
		currentSpeed = i;
		delay(ms);
	}
}

//stop car slowly
void Car::slowStop(int ms) {
	Serial.println("Slow stopping car...");
	digitalWrite(IN1, HIGH);
	digitalWrite(IN3, HIGH);
	for (int i = currentSpeed; i >= 0; i--) {
		analogWrite(ENA, i);
		analogWrite(ENB, i);
		delay(ms);
	}
	digitalWrite(ENA, LOW);
}

//rotate car left
void Car::turnLeft() {
	Serial.println("Turning left...");
	digitalWrite(IN1, LOW);
	digitalWrite(IN2, HIGH);
	digitalWrite(ENA, HIGH);

	digitalWrite(IN3, HIGH);
	digitalWrite(IN4, LOW);
	digitalWrite(ENB, HIGH);

	delay(TURN_DELAY);
	stop();
}

//rotate car right
void Car::turnRight() {
	Serial.println("Turning right...");
	digitalWrite(IN1, HIGH);
	digitalWrite(IN2, LOW);
	digitalWrite(ENA, HIGH);

	digitalWrite(IN3, LOW);
	digitalWrite(IN4, HIGH);
	digitalWrite(ENB, HIGH);

	delay(TURN_DELAY);
	stop();
}

//rotate car backwards
void Car::turnAround(){
	leftTrackBackward();
	delay(TURN_DELAY);

	stop();

	rightTrackForward();
	delay(TURN_DELAY);
	stop();
}

//start car immediately
void Car::start(){
	Serial.println("Starting car...");
	currentSpeed = 255;
	digitalWrite(IN1, HIGH);
	digitalWrite(IN3, HIGH);

	digitalWrite(ENA, HIGH);
	digitalWrite(ENB, HIGH);
}

//stop car immediately
void Car::stop() {
	Serial.println("Stoping car...");
	digitalWrite(IN1, LOW);
	digitalWrite(IN2, LOW);
	digitalWrite(ENA, LOW);

	digitalWrite(IN3, LOW);
	digitalWrite(IN4, LOW);
	digitalWrite(ENB, LOW);
}

//start car in reverse
void Car::goBackward(){
	Serial.println("Going backward...");
	stop();
	digitalWrite(IN2, HIGH);
	digitalWrite(IN4, HIGH);

	digitalWrite(ENA, HIGH);
	digitalWrite(ENB, HIGH);
}

//turn car left using only left side wheels
void Car::turnLeftSingleTrack(){
	Serial.println("Turning left (single track)...");

	stop();

	rightTrackForward();

	delay(TURN_DELAY);
	stop();
}

//turn car left using only right side wheels
void Car::turnRightSingleTrack(){
	Serial.println("Turning right (single track)...");

	stop();

	leftTrackForward();
	
	delay(TURN_DELAY);
	stop();
}
#pragma endregion

#pragma region Travel
void Car::travelForward(){
	slowStart();
}

void Car::travelLeft(){
	//turnLeft();
	turnLeftSingleTrack();
	slowStart();
}

void Car::travelRight(){
	//turnRight();
	turnRightSingleTrack();
	slowStart();
}

void Car::travelBackward(){
	turnAround();
	scanArea();
	determineDirectionAndTravel();
}
#pragma endregion

#pragma region Orientation
//single distance check
int Car::getDistance(){
	int res = pUltrasonic->read();
	//Serial.println(res);
	return res;
}

//peek left side and return distance
int Car::peekLeft(){
	servo.write(AREA_SCAN_MAX);
	int out = getDistance();
	resetServo();
	return out;
}

//peek right side and return distance
int Car::peekRight(){
	servo.write(AREA_SCAN_MIN);
	int out = getDistance();
	resetServo();
	return out;
}

//scan area for obstacles
void Car::scanArea(){
	switchState(DISORIENTED);

	if (pLastAreaScan != nullptr) delete [] pLastAreaScan;
	pLastAreaScan = new int[AREA_SCAN_MAX + 1];

	Serial.println("Scanning area...");
	for (int i = AREA_SCAN_MIN; i < AREA_SCAN_MAX; i++){
		pLastAreaScan[i] = pUltrasonic->read();
		servo.write(i);
		//delay(20);
	}

	resetServo();
}

//reset servo position to neutral
void Car::resetServo(){
	servo.write(SERVO_NEUTRAL);
}

//determine which way to go using pLastAreaScan, with specifided priority:
// forward -> left -> right -> backward
Car::directions Car::determineDirection(){
	if (pLastAreaScan == nullptr) return NULL_SCAN;

	//check forward
	if (pLastAreaScan[AREA_SCAN_MAX / 2] >= START_DISTANCE)
		return FORWARD;

	//check left
	if (pLastAreaScan[AREA_SCAN_MIN] >= START_DISTANCE)
		return LEFT;
	
	//check right
	if (pLastAreaScan[AREA_SCAN_MAX] >= START_DISTANCE)
		return RIGHT;

	//otherwise turn around
	return BACKWARD;
}
#pragma endregion

#pragma region Interactions
//switch current state
void Car::switchState(Car::states nextState) {
	currentState = nextState;
	recolorLed(nextState);
}

//change RGB LED color according to next state
void Car::recolorLed(Car::states nextState){

	switch(nextState){
		case IDLE:
			digitalWrite(LedRed, HIGH);
			digitalWrite(LedGreen, HIGH);
			digitalWrite(LedBlue, HIGH);
			break;

		case PLAYFUL:
			digitalWrite(LedRed, HIGH);
			digitalWrite(LedGreen, LOW);
			digitalWrite(LedBlue, HIGH);
			break;

		case TRAVEL:
			digitalWrite(LedRed, LOW);
			digitalWrite(LedGreen, HIGH);
			digitalWrite(LedBlue, LOW);
			break;

		case DISORIENTED:
			digitalWrite(LedRed, HIGH);
			digitalWrite(LedGreen, LOW);
			digitalWrite(LedBlue, LOW);
			break;

		case STARTLED:
			digitalWrite(LedRed, HIGH);
			digitalWrite(LedGreen, LOW);
			digitalWrite(LedBlue, LOW);
			break;
	}
}

//buzzer sound
void Car::soundScreech(int ms){
	tone(BuzzerPin, 10750, ms);
}

void Car::soundWoow(int ms){
	for(int i=0; i<255; i++){
		tone(BuzzerPin, i);
		delay(1);
	}

	for(int i=255; i>0; i--){
		tone(BuzzerPin, i);
		delay(1);
	}

	noTone(BuzzerPin);
}
#pragma endregion

#pragma region Utils
//return pointer to array with last performed area scan
int* Car::getLastAreaScan(){
	return pLastAreaScan;
}

//helper function for setting up servo and ultrasonic
//also performs initial area scan
void Car::setup(){
	switchState(IDLE);

	servo.attach(ServoPin);
	pUltrasonic = new Ultrasonic(UsTrigger, UsEcho);
	scanArea();
	lastForwardScan = pUltrasonic->read();

	determineDirectionAndTravel();
}

void Car::leftTrackForward(){
	digitalWrite(IN1, HIGH);
	digitalWrite(IN2, LOW);
	digitalWrite(ENA, HIGH);
}

void Car::leftTrackBackward(){
	digitalWrite(IN1, LOW);
	digitalWrite(IN2, HIGH);
	digitalWrite(ENA, HIGH);
}

void Car::leftTrackStop(){
	digitalWrite(IN1, LOW);
	digitalWrite(IN2, LOW);
	digitalWrite(ENA, LOW);
}

void Car::rightTrackForward(){
	digitalWrite(IN3, HIGH);
	digitalWrite(IN4, LOW);
	digitalWrite(ENB, HIGH);
}

void Car::rightTrackBackward(){
	digitalWrite(IN3, LOW);
	digitalWrite(IN4, HIGH);
	digitalWrite(ENB, HIGH);
}

void Car::rightTrackStop(){
	digitalWrite(IN3, LOW);
	digitalWrite(IN4, LOW);
	digitalWrite(ENA, LOW);
}

void Car::playAround(){
	int ms = 1400;

	Serial.println("Stop");
	stop();

	Serial.println("right forward");
	rightTrackForward();
	delay(ms);
	stop();

	Serial.println("left forward");
	leftTrackForward();
	delay(ms);
	stop();

	Serial.println("right forward");
	rightTrackForward();
	delay(ms);
	stop();

	Serial.println("left forward");
	leftTrackForward();
	delay(ms);
	stop();
}
#pragma endregion


void Car::testFunc(){
	stop();
	Serial.println("leftTrackBackward");
	leftTrackBackward();
	delay(2000);

	stop();
	Serial.println("rightTrackBackward");
	rightTrackBackward();
	delay(2000);
	slowStop();
}


//=================================================
//Main travel loop to call in arduino loop():

void Car::travelLoop(){
	int forwardScan = getDistance();

	if (forwardScan <= STOP_DISTANCE){
		Serial.print("Obstacle detected: ");
		Serial.println(forwardScan);
		//immediately stop car if obstacle appears suddenly (specified by PANIC_STOP_THRESHOLD config)
		//slowly stop car if obstacle was detected from far away
		if ( (lastForwardScan - forwardScan) > PANIC_STOP_THRESHOLD ) {
			stop();
			soundScreech();
		}
		else{
			slowStop();
		}

		scanArea();

		//determine where to go next
		determineDirectionAndTravel();
	}
}

//helper function to avoid code redundancy in setup()
void Car::determineDirectionAndTravel(){
	Car::directions d = determineDirection();
	Serial.print("Determined direction - ");
	Serial.println(d);
	switch (d) {
		case FORWARD:
			travelForward();
			break;

		case LEFT:
			travelLeft();
			break;

		case RIGHT:
			travelRight();
			break;

		case BACKWARD:
			travelBackward();
			break;

		case NULL_SCAN:
			break;

		default:
			break;
	}

	switchState(TRAVEL);
}