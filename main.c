/*
 * MAE3780_FinalProject_RobotCode.c
 *
 * Created: 12/1/2018 9:28:37 PM
 * Author : csc254 yma5 zpc5
 */ 

#define F_CPU 16000000UL
#include <avr/io.h>
#include "serial.h" //Comment out if using mode selection
#include <util/delay.h>
#include <avr/interrupt.h>
#include <stdlib.h>

//Calibrated values
int driveTime1Ft = 1000; //Calibrated time to drive ~1ft (from full stop to full stop)
int turnTime90Deg = 590; //Calibrated time to turn ~90 degrees (from full stop to full stop)
int bluePeriod = 65; //Calibrated mean period of the color sensor reading on blue.
int yellowPeriod = 18; //Calibrated mean period of the color sensor reading on yellow.
int sonarDistance = 15; //Calibrated distance for positive sonar hit

//Drive Servo Control Binary Values
int leftWheelForward = 0b10000000;
int leftWheelBackward = 0b01000000;
int leftWheelStop = 0b00111111;
int rightWheelForward = 0b00100000;
int rightWheelBackward = 0b00010000;
int rightWheelStop = 0b11001111;
int bothWheelsStop = 0b00001111;

//Values used across multiple functions
int crossedCenter = 0; //Stores whether or not the robot thinks it has crossed the center line at least once
int isStartingColorBlue; //Stores whether the robot started on blue or not (ie. yellow).
int colorHistory[10]; //Array of ints to store the last 10 color sensor readings (for averaging purposes).
int colorPeriod; //Average value of last 10 color sensor readings.
int period = 0; //Value passed by the color sensor/sonar interrupt.
int hitEdgeHeadOn = 0; //For determining if the robot has hit an edge straight on (transition between portions of code)
int followRobotBypass = 0; //For possible on-startup sensing a jumper wire to bypass certain loops in the code if they don't work during competition.
int initialBlockClearBypass = 0; //For possible on-startup sensing a jumper wire to bypass certain loops in the code if they don't work during competition.
int startingQTI = 0; //Look, this is completely within the rules
int clearingOurSide = 0; //For making sure we clear blocks from our side
int armsDeployed = 0; //Stores if the arms are deployed or not

//Specify which interrupt reading to perform
int gettingColorReading = 0;
int gettingSonarReading = 0;

ISR(PCINT0_vect) //Color Sensor and Sonar interrupt
{
	if(gettingColorReading){
		if (PINB &= 0b00100000){
			//Interrupt triggered on rising edge, reset TCINT1 (TIMER1) to 0
			TCNT1 = 0;
			} else {
			//Interrupt triggered on falling edge, log time since rising edge as period
			period = TCNT1;
		}
	}	
	if(gettingSonarReading){
		if (PINB &= 0b00011100){ //Triggers if any of the sonar pins are high (may have to rewrite if not working properly)
			//Interrupt triggered on rising edge, reset TCINT1 (TIMER1) to 0
			TCNT1 = 0;
			} else {
			//Interrupt triggered on falling edge, log time since rising edge as period
			period = TCNT1;
		}
	}
}

ISR(INT0_vect) //Right QTI interrupt
{
	EIMSK &= 0b11111100; //Disables QTI interrupts temporarily
	_delay_ms(50); //Waits for 25ms to verify the reading wasn't messed up.
	if ((PIND &= 0b00001100) == 0b00001100){ //Checks if both QTIs are on black
		hitEdgeHeadOn = 1;
		if(clearingOurSide) //Pushes blocks off the edge if we're clearing out side
		{
			drive(1,driveTime1Ft*2);
			drive(0,driveTime1Ft);
			turn(1,turnTime90Deg);
		} else {
			drive(0,driveTime1Ft);
			turn(rand()%2,turnTime90Deg);
		}
	} else if (PIND &= 0b00000100){ //If the QTI sensor is still triggered on black	
		if(clearingOurSide) //Pushes blocks off the edge if we're clearing out side
		{
			drive(1,driveTime1Ft*2);
			drive(0,driveTime1Ft);
			turn(1,turnTime90Deg);
		} else
		{
			//This is the turn around QTI interrupt, replace with more relevant code
			drive(1,driveTime1Ft*2);
			wait(50); //Stops and waits for a moment (to kill inertia)
			drive(0,500); //Drive backwards
			turn(1,turnTime90Deg); //Turns left
		}
	}
	EIMSK |= 0b00000011; //Re-enables QTI interrupts
}

ISR(INT1_vect) //Left QTI interrupt
{
	EIMSK &= 0b11111100; //Disables QTI interrupts temporarily
	_delay_ms(50); //Waits for 25ms to verify the reading wasn't messed up.
	if ((PIND &= 0b00001100) == 0b00001100){ //Checks if both QTIs are on black
		hitEdgeHeadOn = 1;
		if(clearingOurSide) //Pushes blocks off the edge if we're clearing out side
		{
			drive(1,driveTime1Ft*2);
			drive(0,driveTime1Ft);
			turn(1,turnTime90Deg);
		} else {
		drive(0,driveTime1Ft);
		turn(rand()%2,turnTime90Deg);
		}
	} else if (PIND &= 0b00001000){ //If the QTI sensor is still triggered on black
		if(clearingOurSide) //Pushes blocks off the edge if we're clearing out side
		{
			drive(1,driveTime1Ft*2);
			drive(0,driveTime1Ft);
			turn(1,turnTime90Deg);
		} else
		{
			//This is the turn around QTI interrupt, replace with more relevant code
			drive(1,driveTime1Ft*1.2);
			wait(50); //Stops and waits for a moment (to kill inertia)
			drive(0,500); //Drive backwards
			turn(0,turnTime90Deg); //Turns left
		}
	}
	EIMSK |= 0b00000011; //Re-enables QTI interrupts
}

int main(void) //Main program execution
{
	init_uart(); //Initializes serial communication(?)
	init(); //Calls general initialization function
	
	int sensorCheck = 0;
    while (sensorCheck) //Loop that just prints sensor reading to terminal for debugging
    {
		sensorReadings();
		//motorTest();
		_delay_ms(1000);
    }
	
	initialBlockClearBypass = (0 || startingQTI);
	if(!initialBlockClearBypass) //Unless the bypass code is implemented, run the initial bock clearing function. Runs only once.
	{
		initialBlockClear();
	}
	
	followRobotBypass = (0 || startingQTI); 
	int chaseCounter = 0;
	while(!followRobotBypass && (chaseCounter<30)) //Loop to hunt down and beat up the other robot
	{
		followOtherRobot();
		chaseCounter += 1;
	}
	
	//Turns left 90 degrees and clears blocks like a pro
	turn(1,turnTime90Deg);
	EIMSK |= 0b00000011; //Re-enables QTI interrupts if they were disabled during the robot following code
	while(1) //Final default action loop, just infinitely clears blocks from its starting side (bouncing semi-randomly)
	{
		clearBlocks();
	}
}

void init() //General I/O and sensors initialization routine
{
	//Sets in/out on all relevant pins
	DDRC &= 0b11000000; //Sets all the analog pins as inputs (needed for possibly bypassing parts of code if robot breaks)
	DDRD &= 0b11110011; //Sets PortD 2 and PortD 3 to input for the QTI sensors
	DDRD |= 0b11110000; //Sets PortD 4-7 as output for the drive servos
	DDRB |= 0b00000011; //Sets PortB 0 and PortB 1 as output for the arm servos
	DDRB &= 0b11000011; //Sets PortB 2-5 as input for the sonars and color sensor
	//DDRD &= 0b00000011; //Sets PortD 0 and PortD 1 to input for mode selection (commented out when serial is actually being used)
	
	//Color/Sonar interrupt initialization
	sei(); //Enables interrupts globally
	PCICR |= 0b00000001; //Enables interrupts from PCINT7..0 (all 3 sonar and the color sensor)
	//Still requires calls to PSMSK0 to mask for certain interrupts
	TCCR1B = 0b00000001; //Enables TIMER1 with a prescaler of 1
	
	//QTI sensor interrupt initialization
	EICRA = 0b00001111; //A rising edge of INT0 or INT1 (either QTI reaching black) triggers an interrupt.
	//EIMSK |= 0b00000011; //Enables interrupts on INT0 and INT1 (comment out when not driving)
	
	//Establishes an initial color history
	int i, avgColor;
	for(i = 0; i<5; i++){
		colorHistory[i] = getColor();
		avgColor += colorHistory[i];
	}
	
	//Uses the color history to decide if it is on blue or not
	int startingColor = getAvgColor(getColor());
	isStartingColorBlue = ((startingColor >= bluePeriod-15)&&(startingColor <= bluePeriod+15));
	
	//Sneaky shit
	startingQTI = PIND &= 0b00001100;
}

void sensorReadings() //Prints all current sensor readings
{
	int rightQTIReading, leftQTIReading, rightSonarReading, centerSonarReading, leftSonarReading, colorReading, avgColorReading, modeSelection;
	rightQTIReading = PIND &= 0b00000100;
	printf("Right QTI reading: %u \n",rightQTIReading);
	leftQTIReading  = PIND &= 0b00001000;
	printf("Left QTI reading: %u \n",leftQTIReading);
	rightSonarReading = getSonarReading(0);
	_delay_ms(100);
	printf("Right sonar reading: %u in. \n",rightSonarReading);
	centerSonarReading = getSonarReading(1);
	_delay_ms(100);
	printf("Center sonar reading: %u in. \n",centerSonarReading);
	leftSonarReading = getSonarReading(2);
	printf("Left sonar reading: %u in. \n",leftSonarReading);
	colorReading = getColor();
	printf("Current color reading: %u \n",colorReading);
	avgColorReading = getAvgColor(colorReading);
	printf("Average color reading: %u \n",avgColorReading);
	int randomTest = rand()%2;
	printf("Random: %u \n",randomTest);
	
}

void motorTest() //Runs all motor functions briefly
{
	if(!armsDeployed)
	{
		armDeploy();
		_delay_ms(700);
		armStop();
		armsDeployed = 1;
	}
	wait(500);
	if(armsDeployed){
		armRetract();
		_delay_ms(700);
		armStop();
		armsDeployed = 0;
	}
	wait(500);
	drive(1,500);
	wait(500);
	drive(0,500);
	wait(500);
	turn(1,500);
	wait(500);
	turn(0,500);
	wait(500);
}

void initialBlockClear() //Commands run at the beginning of the match before we try to push the other robot off the board
{
	driveForward();
	if(!armsDeployed)
	{
		armDeploy();
		_delay_ms(700); 
		armStop();
		armsDeployed = 1;
	}
	while(!crossedCenter)//Drives forward and updates color reading until the robot has crossed the center
	{
		driveForward();
		int currentColor = getAvgColor(getColor());
		//If the current average color reading is opposite the starting one, the robot has crossed the center.
		if((!isStartingColorBlue && ((currentColor >= bluePeriod-15)&&(currentColor <= bluePeriod+15))) || (isStartingColorBlue && ((currentColor >= yellowPeriod-15)&&(currentColor <= yellowPeriod+15))))
		{
			crossedCenter = 1;
		} 
	}
	EIMSK &= 0b11111100; //Disables QTI Interrupts
	while(!(PIND&=0b00001100)){} //Drives forward onto the opposing side until a QTI hits an edge, test this to make sure it doesn't push blocks off the back edge
	
	//Drive backwards, retracts the arms, then turns around (to go find the other robot)
	driveBackward();
	if(armsDeployed){
		armRetract();
		_delay_ms(700);
		armStop();
		armsDeployed = 0;
	}
	_delay_ms(1000);
	wait(500);
	turn(1,500);
	
}

void dontfollowOtherRobot() //Uses Sonar to find opposing robot and tries to push it off the field
{
	//EIMSK &= 0b11111100; //Disables QTI interrupts temporarily to preserve timing
	
	EIMSK |= 0b00000011;
	wait(250);
	int centerSonar = 250 >= getSonarReading(1);
	_delay_ms(100);
	if (centerSonar)
	{
		driveForward();
		_delay_ms(750);
	} else 
	{
		stopMovement();
		int rightSonar = 250 >= getSonarReading(0);
		_delay_ms(100);
		int leftSonar = 250 >= getSonarReading(2);
		if(rightSonar && !leftSonar)
		{
			turnRight();
		} else if(leftSonar && !rightSonar)
		{
			turnLeft();
		} else 
		{
			turnLeft(); //Default is to turn left after failing to find a target
		}
	}
	_delay_ms(250); //Sonar sweeps every 0.25 sec to give time to actually do the relevant movements
	//pseudoQTI(); //Between each sonar sweep it doesn a quick check for the QTIs being on the black border
	//EIMSK |= 0b00000011; //Re-enables QTI interrupts
}

void followOtherRobot()
{
	EIMSK &= 0b11111100; //Disables QTI interrupts temporarily to preserve timing
	wait(25);
	
	//getting values to use
	int leftSonar = getSonarReading(2);
	_delay_ms(100);
	int centerSonar = getSonarReading(1);
	_delay_ms(100);
	int rightSonar = getSonarReading(0);
	_delay_ms(100);
	
	
	// Arbitrarily high value to denote out of range
	
	if (centerSonar > 24){
		centerSonar = 100;
	}
	if (rightSonar > 24){
		rightSonar = 100;
	}
	if (leftSonar > 24){
		leftSonar = 100;
	}
	
	//ALL ABOUT EQUAL
	int turned = 0;
	if((abs(leftSonar - centerSonar)/(0.5*abs(leftSonar + centerSonar)) < 0.15) && (abs(rightSonar - centerSonar)/(0.5*abs(rightSonar + centerSonar)) < 0.15)){
		if((leftSonar == 100) && (centerSonar == 100) && (rightSonar == 100)){
			turn(rand()%2,500); // Random turn when out of range
			turned = 1;
		}
		else{
			driveForward();
		}
		// Out of Range
	}
	//SEMI INEQUALITIES
	else if((abs(leftSonar - centerSonar)/(0.5*abs(leftSonar + centerSonar))) < 0.15){
		if(rightSonar > centerSonar){
			driveForward();
		}
		else{ //right < center
			turn(0,200);
			turned = 1;
		}
	}
	else if((abs(rightSonar - centerSonar)/(0.5*abs(rightSonar + centerSonar))) < 0.15){
		if(leftSonar > centerSonar){
			driveForward(1000);
		}
		else{ //left > center
			turn(1,200);
			turned = 1;
		}
	}
	else if((abs(rightSonar - leftSonar)/(0.5*abs(rightSonar + leftSonar))) < 0.15){
		driveForward(1000);
	}
	//ABSOLUTE INEQUALITIES
	else if(rightSonar > centerSonar && centerSonar > leftSonar){ // right > center > left
		turn(1,200);
		turned = 1;
	}
	else if(leftSonar > centerSonar && centerSonar > rightSonar){ // left > center > right
		turn(0,200);
		turned = 1;
	}
	else if(centerSonar > rightSonar && rightSonar > leftSonar){ //center > right > left wtf
		turn(rand()%2,200);
		turned = 1;
	}
	else if(leftSonar > rightSonar && rightSonar > centerSonar){ //left > right > center
		driveForward(1000);
	}
	else if(centerSonar > leftSonar && leftSonar > rightSonar){ //center > left > right wtf
		turn(rand()%2,200);
		turned = 1;
	}
	else if(rightSonar > leftSonar && leftSonar > centerSonar){ //right > left > center + other conditions?
		driveForward(1000);
	} else {
		turn(rand()%2,200);
		turned = 1;
	}
	if(!turned)
	{
		_delay_ms(750);
	}
	turned = 0;
	_delay_ms(200);
	pseudoQTI();
}

void clearBlocks() //Bounces around clearing blocks to the edges of our starting side
{
	//Deploys the arms if they aren't currently
	if(!armsDeployed || !(rand()%3))
	{
		armDeploy();
		_delay_ms(700);
		armStop();
		armsDeployed = 1;
	}
	EIMSK |= 0b00000011;
	int currentColor = getAvgColor(getColor());
	if((!isStartingColorBlue && ((currentColor >= bluePeriod-15)&&(currentColor <= bluePeriod+15))) || (isStartingColorBlue && ((currentColor >= yellowPeriod-15)&&(currentColor <= yellowPeriod+15))))
	{
		//Need to add something to make sure it gets back to our side before clearing indefinitely
		int backOnStartSide = 0;
		while(!backOnStartSide)
		{
			currentColor = getAvgColor(getColor());
			backOnStartSide = !((!isStartingColorBlue && ((currentColor >= bluePeriod-15)&&(currentColor <= bluePeriod+15))) || (isStartingColorBlue && ((currentColor >= yellowPeriod-15)&&(currentColor <= yellowPeriod+15))));
			driveForward();
		}

		//Once back on our side, drives farther onto the side then turn around
		drive(1,driveTime1Ft);
		turn(1,turnTime90Deg*2);
	}
	
	for(int i = 0; i<5; i++){
		if(isStartingColorBlue){ colorHistory[i] = bluePeriod; }
		if(!isStartingColorBlue){ colorHistory[i] = yellowPeriod; }
	}
	
	driveForward();
	int clearingOurSide = 1; //Changes this state variable to the QTI interrupt will clear our side
	//While still on our starting side, just drive and update color (hitting black borders is handled by QTIs
	while(!((!isStartingColorBlue && ((currentColor >= bluePeriod-15)&&(currentColor <= bluePeriod+15))) || (isStartingColorBlue && ((currentColor >= yellowPeriod-15)&&(currentColor <= yellowPeriod+15)))))
	{
		driveForward();
		currentColor = getColor();//getAvgColor(getColor());
	}
	
	//If the robot crosses the center, reverse then turn around
	armRetract();
	armsDeployed = 0;
	drive(0,500);
	turn(rand()%2,turnTime90Deg);
	clearingOurSide = 0;
	
	//(and repeat)
}

void pseudoQTI() //QTI Interrupt in function form for when running the interrupt itself could mess up timings
{
	if (PIND &= 0b00000100){ //If the QTI sensor is still triggered on black
		//This is the turn around QTI interrupt, replace with more relevant code
		wait(50); //Stops and waits for a moment (to kill inertia)
		drive(0,500); //Drive backwards
		turn(1,turnTime90Deg*0.5); //Turns left
	} else if (PIND &= 0b00001000) {
		//This is the turn around QTI interrupt, replace with more relevant code
		wait(50); //Stops and waits for a moment (to kill inertia)
		drive(0,500); //Drive backwards
		turn(0,turnTime90Deg*0.5); //Turns right
	}
}

int getColor() //Gets a color period reading from the color sensor
{
	gettingColorReading = 1; //Specifies which interrupt routing to run
	PCMSK0 |= 0b00100000; //Turns on the interrupt on PCINT5
	_delay_ms(2); //Pauses for 10ms to ensure the interrupt is triggered and period is set to a value
	PCMSK0 &= 0b11011111; //Turns off the interrupt on PCINT5
	int periodus =  period*.125; //Converts the period from clock ticks to microseconds
	gettingColorReading = 0; //Returns to non-specific interrupt routine
	return periodus;
}

int getAvgColor(int newReading) //Keeps a running average of the last 5 color sensor readings
{
	//Pushes the array forward by one place to the left
	for(int j = 0; j <= 5; j++){ colorHistory[j] = colorHistory[j+1]; }
	//Sets the new data point as the final value of the array
	colorHistory[4] = newReading;
	//Averages the last 5 readings and returns that value
	int avgColor = 0;
	for(int i = 0; i<5; i++){ avgColor += colorHistory[i]; }
	return (int) avgColor*0.2;
}

int getSonarReading(int whichSonar) //Gets a sonar distance reading, returns 1 if the sonar sees something and 0 otherwise
{
	//int whichSonar specifies which sonar it is getting a reading for; 0 = right, 1 = center, 2 = left
	unsigned int sonarPosition = 0b00000100 << whichSonar; //Bit shifts the position of the 1 to the  position for the given sonar's pin's register
	unsigned int sonarPositionInv = 0b11111011 << whichSonar; //Bit shifts the position of the 0 to the  position for the given sonar's pin's register
	sonarPositionInv |= 0b00000011; //Ensures that the first and second bits are 1 (since they might have been left shifted to 0)
	
	//Sets the sonar's port to an output then to high for a 5 microsecond pulse, then low and back to input
	DDRB |= sonarPosition;
	PORTB &= sonarPositionInv;
	_delay_us(2);
	PORTB |= sonarPosition;
	_delay_us(5);
	PORTB &= sonarPositionInv;
	DDRB &= sonarPositionInv;
	
	//Turns on interrupts on the sonar's pin, waits for long enough for the sonar signal to return, then turns the interrupt off
	gettingSonarReading = 1; //Specifies what interrupt routine to run
	PCMSK0 |= sonarPosition;
	_delay_ms(20); //Max required time is 18.5ms
	PCMSK0 &= sonarPositionInv;
	gettingSonarReading = 0; //Returns to non-specific interrupt routine
	
	//Converts the period reading (stored globally) from clock ticks to time and then from time to inches
	//16 ticks = 1 microsecond, 73.746in/microsecond
	float distance =  period/(73.746*16);
	return (int) distance;// <= 10;
}

void stopMovement() //Self explanatory 
{
	PORTD &= bothWheelsStop;
}

void wait(int duration) //Stops the robot for a specified time
{
	stopMovement();
	while(duration >= 0){
		_delay_ms(10);  //Waits for duration seconds.
		duration -= 10;
	}
}

void driveForward() //Self explanatory 
{
	PORTD &= bothWheelsStop;
	PORTD |= rightWheelForward;
	PORTD |= leftWheelForward;
}

void driveBackward() //Self explanatory 
{
	PORTD &= bothWheelsStop;
	PORTD |= rightWheelBackward;
	PORTD |= leftWheelBackward;
}

void turnLeft() //Self explanatory 
{
	PORTD &= bothWheelsStop;
	PORTD |= rightWheelForward;
	PORTD |= leftWheelBackward;
}

void turnRight() //Self explanatory 
{
	PORTD &= bothWheelsStop;
	PORTD |= rightWheelBackward;
	PORTD |= leftWheelForward;
}

void drive(int direction, int driveTime) //Drive (forward = 1, backwards = 0) for a specified number of milliseconds.
{
	if(!direction) {
		driveBackward();
		while(driveTime*1000 >= 0){
			_delay_us(10);  //Waits for driveTime seconds.
			driveTime -= 10;
		}
		stopMovement();
	} else {
		driveForward();
		while(driveTime*1000 >= 0){
			_delay_us(10);  //Waits for driveTime seconds.
			driveTime -= 10;
		}
		stopMovement();
	}
}

void turn(int direction, int driveTime) //Turn (left = 1, right = 0) for a specified number of milliseconds.
{ 
	if(!direction) 
	{ //If direction is 0, turning direction is right
		turnRight();
		while(driveTime >= 0)
		{
			_delay_ms(10);  //Waits for driveTime seconds.
			driveTime -= 10;
		}
		stopMovement();
	} else { //If direction is 1, turning direction is left
		turnLeft();
		while(driveTime >= 0)
		{
			_delay_ms(10);  //Waits for driveTime seconds.
			driveTime -= 10;
		}
		stopMovement();
	}
}

void armStop() //Stops the arm movement 
{
	PORTB &= 0b11111100;
}

void armDeploy() //Extends the arms, takes ~700ms to deploy fully 
{
	PORTB &= 0b11111100;
	PORTB |= 0b00000010;
}

void armRetract() //Retracts the arms 
{
	PORTB &= 0b11111100;
	PORTB |= 0b00000001;
}