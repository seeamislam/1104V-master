#pragma config(Sensor, in8,    gyro,           sensorGyro)
#pragma config(Sensor, dgtl1,  leftDrive,      sensorNone)
#pragma config(Sensor, dgtl7,  leftDrive,      sensorQuadEncoder)
#pragma config(Sensor, dgtl9,  rightDrive,     sensorQuadEncoder)
#pragma config(Sensor, dgtl11, FwEncoder,      sensorQuadEncoder)
#pragma config(Motor,  port1,           mtr_ldr2,      tmotorVex393TurboSpeed_HBridge, openLoop, reversed)
#pragma config(Motor,  port2,           mtr_rdr1,      tmotorVex393TurboSpeed_MC29, openLoop, reversed)
#pragma config(Motor,  port3,           mtr_indx,      tmotorVex393TurboSpeed_MC29, openLoop, reversed)
#pragma config(Motor,  port4,           mtr_int,       tmotorVex393TurboSpeed_MC29, openLoop, reversed)
#pragma config(Motor,  port5,           Motor_FW1,     tmotorVex393HighSpeed_MC29, openLoop, reversed)
#pragma config(Motor,  port6,           mtr_ldr1,      tmotorVex393TurboSpeed_MC29, openLoop)
#pragma config(Motor,  port7,           mtr_rdr2,      tmotorVex393TurboSpeed_MC29, openLoop)
#pragma config(Motor,  port8,           mtr_ldr3,      tmotorVex393TurboSpeed_MC29, openLoop)
#pragma config(Motor,  port9,           Motor_FW2,     tmotorVex393HighSpeed_MC29, openLoop)
#pragma config(Motor,  port10,          mtr_rdr3,      tmotorVex393TurboSpeed_HBridge, openLoop, reversed)


#pragma platform(VEX)
#pragma competitionControl(Competition)
#pragma autonomousDuration(20)
#pragma userControlDuration(120)
#include "Vex_Competition_Includes.c"


#define RPM_INCREMENT 25

int rightBack;
int leftBack;

word btnU, btnD, held = 0;
int desiredRpm = 0;


int trueSpeed(int power) {
	int tsArray[128] =
	{
		0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
		0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
		5,10,12,13,13,14,15,15,15,15,
		20,20,20,20,20,20,20,20,20,20,
		25, 25, 25, 25, 25, 25, 25, 25, 25, 25,
		35, 35, 35, 35, 35, 35, 35, 35, 35, 35,
		41, 41, 42, 42, 43, 44, 44, 45, 45, 46,
		46, 47, 47, 48, 48, 49, 50, 50, 51, 52,
		52, 53, 54, 55, 56, 57, 57, 58, 59, 60,
		61, 62, 63, 64, 65, 66, 67, 68, 72, 77,
		82, 87, 92, 97, 103, 108, 113, 118, 120, 125,
		126,127,127,127,127,127,127,127,127,127,
		127,127,127,127,127,127,127,127
	};
	return ((power>0)?1:-1)*tsArray[power*((power>0)?1:-1)];
}
bool backDriveBool;

void waitForPress()
{
	while(nLCDButtons == 0){}
	wait1Msec(5);
}

void backDrive(){
	if(vexRT[Btn7L] == 1){
		rightBack = 80;
		leftBack = 80;
		backDriveBool = true;

		}else if(vexRT[Btn7R] == 1){
		rightBack = -80;
		leftBack = -80;
		backDriveBool= true;
	}
}
void waitForRelease()
{
	while(nLCDButtons != 0){}
	wait1Msec(5);
}

/*void straight()
{
if(SensorValue[rightDrive] == SensorValue[leftDrive]) // If rightEncoder has counted the same amount as leftEncoder:
{
// Move Forward
motor[rightBack] = -110;		    // Right Motor is run at power level 80
motor[leftBack]  = -110;

}
else if(SensorValue[rightDrive] > SensorValue[leftDrive])	// If rightEncoder has counted more encoder counts
{
// Turn slightly right
motor[rightBack] = -108;		    // Right Motor is run at power level 60
motor[leftBack]  = -110;		    // Left Motor is run at power level 80
}
else	// Only runs if leftEncoder has counted more encoder counts
{
// Turn slightly left
motor[rightBack] = -110;		    // Right Motor is run at power level 80
motor[leftBack]  = -108;		    // Left Motor is run at power level 60
}
}
*/
//***************


void tank(int left, int right) {
	motor[mtr_ldr1] = left;
	motor[mtr_ldr2] = left;
	motor[mtr_ldr3] = left;
	motor[mtr_rdr1] = right;
	motor[mtr_rdr2] = right;
	motor[mtr_rdr3] = right;
}
/*
word maxDrivePower = 127;
long driveTarget = 0;
long driveKl = 100;  // Drive integral limit.
float driveKp = 0.15;
float driveKi = 0.0;
float driveKd = 2;
float turnRatio = 7;  // 12 Ratio of turnKp to use for driving straight.

word maxTurnPower = 127;
long turnTarget = 0;
long turnKl = 100;  // Turn integral limit.
float turnKp = 0.2;
float turnKi = 0.0;
float turnKd = 2;
float driveRatio = 0;  // Ratio of driveKp to use for turning on a dime.
*/
void RotateAngle(int DesiredAngle,int MaxTime,int PowerLimit) {

	//send a neg. angle to turn right.right rotations go negative, power to rotate right is L+,R-
	int NotDone;int LeftPwr;int RightPwr;int PreviousError;int RotationError;int RotationPwr;
	const float RotationkD=2.0; const float RotationkP=0.3;	const int SuccessThreshold=15; // this is 1.5 degrees
	float Dvalue=0.0;int BrakePwr;
	const float DvalueLimit=30.0;
	int BrakeVal=40;// 16 was good, went to 40 so upped pwer to 24 then 40
	int BrakeTime=60;//in msecs 120 was great but need speed

	PreviousError=0; DesiredAngle*=10;
	if (DesiredAngle > 0) BrakePwr= -BrakeVal;
	else BrakePwr= BrakeVal;
	SensorValue[gyro]= 0;
	// set to Neg of desired angle, then seek zero
	// when TheError is pos, brake fails
	clearTimer(T1);NotDone=1;
	while(((time100[T1]) < MaxTime) && NotDone) {
		RotationError=DesiredAngle - (-SensorValue[gyro]);// TheError is error value
		if (abs(RotationError) < SuccessThreshold) {
			NotDone=0;
			LeftPwr= -BrakePwr; RightPwr= BrakePwr;
			motor[mtr_rdr1]=RightPwr;motor[mtr_ldr1]=LeftPwr;
			motor[mtr_rdr2]=RightPwr;motor[mtr_ldr2]=LeftPwr;
			motor[mtr_rdr3]=RightPwr;motor[mtr_ldr3]=LeftPwr;
			wait1Msec(BrakeTime);
			//testcode
			tank(0,0);
		}
		else {
			Dvalue=(RotationError-PreviousError)*RotationkD;	PreviousError=RotationError;
			//Limit Dval
			if (Dvalue > DvalueLimit) Dvalue=DvalueLimit;
			else if (Dvalue < -DvalueLimit) Dvalue=-DvalueLimit;
			RotationPwr=(int)((RotationError*RotationkP)+Dvalue);
			// Limiter-code
			if (RotationPwr < -PowerLimit) RotationPwr=-PowerLimit;
			else if (RotationPwr > PowerLimit) RotationPwr= PowerLimit;
			LeftPwr= -RotationPwr; RightPwr= RotationPwr;
			motor[mtr_rdr1]=RightPwr;motor[mtr_ldr1]=LeftPwr;
			motor[mtr_rdr2]=RightPwr;motor[mtr_ldr2]=LeftPwr;
			motor[mtr_rdr3]=RightPwr;motor[mtr_ldr3]=LeftPwr;
		}
		wait1Msec(10);// find best val for this. was 2
	}
	tank(0,0);
}


/*
task driveThread() {
long encoderL, encoderR, t = time1[T1], dt, error, turnError, lastError = 0, p, i = 0, power, turnPower;
float d;

while (true) {
encoderR = SensorValue[rightDrive];
encoderL = SensorValue[leftDrive];
dt = (time1[T1] - t);

error = (driveTarget - (encoderR + encoderL));
turnError = (turnTarget - (encoderR - encoderL));

p = error;
i = ((error < driveKl) ? (i + (error * dt)) : 0);
d = ((error - lastError) / ((dt > 0) ? dt : 1));

power = ((driveKp * p) + (driveKi * i) + (driveKd * d));
turnPower = ((turnKp * turnRatio) * turnError);

if (abs(power) > maxDrivePower) {
power = (sgn(power) * maxDrivePower);
}

motor[mtr_rdr1] = motor[mtr_rdr2] = motor[mtr_rdr3] = (power + turnPower);
motor[mtr_ldr1] = motor[mtr_ldr2] = motor[mtr_ldr3] = (power + turnPower);

lastError = error;
t += dt;

sleep(10);
}
}

task turnThread() {
long encoderR, encoderL, t = time1[T1], dt, error, driveError, lastError = 0, p, i = 0, power, drivePower;
float d;

while (true) {
encoderR = SensorValue[rightDrive];
encoderL = SensorValue[leftDrive];
dt = (time1[T1] - t);

error = (turnTarget - (encoderR - encoderL));
driveError = (driveTarget - (encoderR + encoderL));

p = error;
i = ((error < turnKl) ? (i + (error * dt)) : 0);
d = ((error - lastError) / ((dt > 0) ? dt : 1));

power = ((turnKp * p) + (turnKi * i) + (turnKd * d));
drivePower = ((driveKp * driveRatio) * driveError);

if (abs(power) > maxTurnPower) {
power = (sgn(power) * maxTurnPower);
}

motor[mtr_rdr1] = motor[mtr_rdr2] = motor[mtr_rdr3] = (drivePower + power);
motor[mtr_ldr1] = motor[mtr_ldr2] = motor[mtr_ldr3] = (drivePower - power);

lastError = error;
t += dt;

sleep(10);
}
}

void driveInch(long distance, word power = maxDrivePower) {
stopTask(turnThread);
stopTask(driveThread);

int DistInch= ((distance*2)/(4.0*PI))*(360);
maxDrivePower = power;
driveTarget += (DistInch);

startTask(driveThread);

}


void turn(long distance, word power = maxTurnPower) {
stopTask(turnThread);
stopTask(driveThread);

maxTurnPower = power;
turnTarget += distance;

startTask(turnThread);
}
*/





void turn(int rightDegree, int leftDegree, int time){
	motor[rightBack] = rightDegree;		    // Right Motor is run at power level 60
	motor[leftBack]  = leftDegree;		    // Left Motor is run at power level 80

	motor[mtr_rdr1] = rightDegree;
	motor[mtr_rdr2] = rightDegree;
	motor[mtr_rdr3] = rightDegree;
	motor[mtr_ldr1] = leftDegree;
	motor[mtr_ldr2] = leftDegree;
	motor[mtr_ldr3] = leftDegree;

	wait1Msec(time);
}
void turnOp(int rightDegree, int leftDegree, int time){
	motor[rightBack] = rightDegree;		    // Right Motor is run at power level 60
	motor[leftBack]  = leftDegree;		    // Left Motor is run at power level 80

	motor[mtr_rdr1] = leftDegree;
	motor[mtr_rdr2] = leftDegree;
	motor[mtr_rdr3] = leftDegree;
	motor[mtr_ldr1] = rightDegree;
	motor[mtr_ldr2] = rightDegree;

	motor[mtr_ldr3] = rightDegree;

	wait1Msec(time);
}
int count = 0;


void pre_auton() {

	const short leftButton = 1;
	const short centerButton = 2;
	const short rightButton = 4;

	clearLCDLine(0);
	clearLCDLine(1);
	//	bool optAuton;

	//if ( centerButton == 1){
	//	optAuton = true;
	//}else {
	//optAuton = false;
	//}

	//while(optAuton == true){
	while(nLCDButtons != centerButton){

		switch(count){
		case 0:

			displayLCDCenteredString(0, "bHrdShoot");
			displayLCDCenteredString(1, "<		 Enter		>");
			waitForPress();

			if(nLCDButtons == leftButton)
			{
				waitForRelease();
				count = 3;
			}
			else if(nLCDButtons == rightButton)
			{
				waitForRelease();
				count++;
			}
			break;
		case 1:

			displayLCDCenteredString(0, "Default");
			displayLCDCenteredString(1, "<		 Enter		>");
			waitForPress();

			if(nLCDButtons == leftButton)
			{
				waitForRelease();
				count--;
			}
			else if(nLCDButtons == rightButton)
			{
				waitForRelease();
				count++;
			}
			break;
		case 2:

			displayLCDCenteredString(0, "Hoard Red");
			displayLCDCenteredString(1, "<		 Enter		>");
			waitForPress();

			if(nLCDButtons == leftButton)
			{
				waitForRelease();
				count--;
			}
			else if(nLCDButtons == rightButton)
			{
				waitForRelease();
				count++;
			}
			break;
		case 3:

			displayLCDCenteredString(0, "bHordShoot");
			displayLCDCenteredString(1, "<		 Enter		>");
			waitForPress();

			if(nLCDButtons == leftButton)
			{
				waitForRelease();
				count--;
			}
			else if(nLCDButtons == rightButton)
			{
				waitForRelease();
				count = 0;
			}
			break;
		default:
			count = 0;
			break;
		}
	}
	//}
	bStopTasksBetweenModes = true;
}

void straight (int distance, int power)
{


	SensorValue[rightDrive]=0;

	while(SensorValue[rightDrive] < distance)
	{



		motor[mtr_rdr1] = power;
		motor[mtr_rdr2] = power;
		motor[mtr_rdr3] = power;
		motor[mtr_ldr1] = power;
		motor[mtr_ldr2] = power;
		motor[mtr_ldr3] = power;
	}
}

void intakeControl(int intPower, int time){
	motor[mtr_int] = intPower;
	wait1Msec(time);
}

void indexControl(int indxPower, int time){
	motor[mtr_indx] = indxPower;
	wait1Msec(time);
}


/*task ActiveBrake() {

tank(0,0);
wait1Msec(500);
int dLP = SensorValue[leftDrive];
int dRP = SensorValue[rightDrive];
while(true) {
int dL = dLP-SensorValue[leftDrive];
int dR = dRP-SensorValue[rightDrive];
float kP = 0.5;
float left = dL*kP;
float right = dR*kP;
tank(left,right);
wait1Msec(50);
}
}
*/

#define FW_LOOP_SPEED              25

#define FW_MAX_POWER              110

#define MOTOR_TPR_269           240.448
#define MOTOR_TPR_393R          261.333
#define MOTOR_TPR_393S          392
#define MOTOR_TPR_393T          627.2
#define MOTOR_TPR_QUAD          72

typedef struct _fw_controller {
	long            counter;                ///< loop counter used for debug

	float           ticks_per_rev;          ///< encoder ticks per revolution
	long            e_current;              ///< current encoder count
	long            e_last;                 ///< current encoder count
	float           v_current;              ///< current velocity in rpm
	long            v_time;                 ///< Time of last velocity calculation
	long            target;
	long            current;
	long            last;
	float           error;
	float           last_error;
	float           gain;
	float           drive;
	float           drive_at_zero;
	long            first_cross;
	float           drive_approx;
	long            motor_drive;
} fw_controller;

static  fw_controller   flywheel;

void
FwMotorSet( int value )
{
	motor[ Motor_FW1 ] = value;
	motor[ Motor_FW2 ] = value;
}

long
FwMotorEncoderGet()
{
	return( SensorValue[dgtl11] );
}

void
FwVelocitySet( fw_controller *fw, int velocity, float predicted_drive )
{
	// set target velocity (motor rpm)
	fw->target        = velocity;
	// Set error so zero crossing is correctly detected
	fw->error         = fw->target - fw->current;
	fw->last_error    = fw->error;
	// Set predicted open loop drive value
	fw->drive_approx  = predicted_drive;
	// Set flag to detect first zero crossing
	fw->first_cross   = 1;
	// clear tbh variable
	fw->drive_at_zero = 0;
}

void
FwCalculateSpeed( fw_controller *fw )
{
	int     delta_ms;
	int     delta_enc;

	fw->e_current = FwMotorEncoderGet();

	delta_ms   = nSysTime - fw->v_time;
	fw->v_time = nSysTime;

	delta_enc = (fw->e_current - fw->e_last);
	fw->e_last = fw->e_current;

	fw->v_current = (1000.0 / delta_ms) * delta_enc * 60.0 / fw->ticks_per_rev;
}

void
FwControlUpdateVelocityTbh( fw_controller *fw )
{
	fw->error = fw->target - fw->current;

	fw->drive =  fw->drive + (fw->error * fw->gain);
	if( fw->drive > 1 )
		fw->drive = 1;
	if( fw->drive < 0 )
		fw->drive = 0;
	if( sgn(fw->error) != sgn(fw->last_error) ) {

		if( fw->first_cross ) {
			fw->drive = fw->drive_approx;
			fw->first_cross = 0;
		}
		else
			fw->drive = 0.5 * ( fw->drive + fw->drive_at_zero );
		fw->drive_at_zero = fw->drive;
	}

	fw->last_error = fw->error;
}



task
FwControlTask()
{
	fw_controller *fw = &flywheel;
	fw->gain = 0.050;
	fw->ticks_per_rev = MOTOR_TPR_QUAD;
	while(1)
	{

		fw->counter++;

		FwCalculateSpeed( fw );

		fw->current = fw->v_current;
		FwControlUpdateVelocityTbh( fw ) ;

		fw->motor_drive  = (fw->drive * FW_MAX_POWER) + 0.5;

		if( fw->motor_drive >  127 ) fw->motor_drive =  127;
		if( fw->motor_drive < -127 ) fw->motor_drive = -127;

		FwMotorSet( fw->motor_drive );

		wait1Msec( FW_LOOP_SPEED );
	}
}

task autonomous()
{



	clearLCDLine(0);
	clearLCDLine(1);
	switch(count){
	case 0:

		displayLCDCenteredString(0, "Autonomous 1");
		displayLCDCenteredString(1, "is running!");
		wait1Msec(25);

		startTask(FwControlTask);	FwVelocitySet (&flywheel, 2862, 0.7);
		wait1Msec(3500);
		indexControl(127, 1500)	;

		intakeControl(127 , 250);

		intakeControl(0 , 950);

		intakeControl(80 , 250);

		intakeControl(0 , 950);

		intakeControl(80 , 250);

		intakeControl(0 , 950);

		intakeControl(127, 250);

		indexControl(0,120);
		startTask(FwControlTask);	FwVelocitySet (&flywheel, 2520, 0.7);
		RotateAngle(-45,1000,96);
		straight(320,80);
		RotateAngle(-90,1000,96);
		straight(710, 45);
		turn(0,0,80);
		RotateAngle(-63,1000,100);
		indexControl(127, 250);
		indexControl(0, 600);
		indexControl(127,250);

		break;
	case 1:

		displayLCDCenteredString(0, "Autonomous 2");
		displayLCDCenteredString(1, "is running!");
		wait1Msec(100);
		startTask(FwControlTask);	FwVelocitySet (&flywheel, 2862, 0.7);
		wait1Msec(3500);
		indexControl(127, 1500)	;

		intakeControl(127 , 250);

		intakeControl(0 , 3000);

		intakeControl(80 , 250);

		intakeControl(0 , 1400);

		intakeControl(80 , 250);

		intakeControl(0 , 1400);

		intakeControl(127, 4000);
		FwVelocitySet (&flywheel, 0 , 0);




		break;
	case 2:

		displayLCDCenteredString(0, "Autonomous 3");
		displayLCDCenteredString(1, "is running!");
		wait1Msec(2000);

		turnOp(100, 100, 850);
		turnOp(0, 0, 100);
		turnOp(50,-50, 700);
		turnOp(0, 0, 100);
		turnOp(-50,-50, 650);
		turnOp(0, 0, 100);
		turnOp(-50,50, 675);
		turnOp(0, 0, 150);
		turnOp(-80,-48, 1250);
		turnOp(0, 0, 150);
		turnOp(70,40,730);
		turnOp(0, 0, 150);
		turnOp(50,-50, 400);
		turnOp(0, 0, 150);
		turnOp(-60,-60, 575);
		turnOp(-50,50,675);
		turnOp(-77,-45, 875);
		turnOp(0, 0, 150);



		break;
	case 3:

		displayLCDCenteredString(0, "");
		displayLCDCenteredString(1, "is running!");
		wait1Msec(10);
		startTask(FwControlTask);	FwVelocitySet (&flywheel, 2862, 0.7);
		wait1Msec(3680);
		indexControl(127, 1500)	;
		intakeControl(127 , 250);
		intakeControl(0 , 1000);
		intakeControl(80 , 250);
		intakeControl(0 , 1000);
		intakeControl(80 , 250);
		intakeControl(0 , 1000);
		intakeControl(127, 250);
		indexControl(0,120);
		startTask(FwControlTask);	FwVelocitySet (&flywheel, 2506, 0.7);
		RotateAngle(45,1000,96);
		straight(320,80);
		RotateAngle(90,1000,96);
		straight(710, 45);
		turn(0,0,80);
		RotateAngle(63,1000,100);
		indexControl(127, 250);
		indexControl(0, 100);
		indexControl(127,250);
		break;
	default:
		displayLCDCenteredString(0, "No valid choice");
		displayLCDCenteredString(1, "was made!");
		break;



	}

}
task usercontrol()
{


	startTask( FwControlTask );
	bLCDBacklight = true;
	while(1)
	{
		//bool straightBool;
		//while(straightBool == true){


		//}
		backDrive();

		//while(backDriveBool == 1){
		//	straightBool = true;
		//}
		motor[mtr_rdr1] = motor[port10] = motor[port7] = rightBack;
		motor[mtr_ldr1] = motor[mtr_ldr2] = motor[mtr_ldr3] = leftBack;

		rightBack = trueSpeed(vexRT[Ch2]);
		leftBack = trueSpeed(vexRT[Ch3]);
		int intakeP = 20;
		if(vexRT[Btn6U] == 1) {
			intakeP = 127;
		}
		else if(vexRT[Btn6D] == 1) {
			intakeP = -127;
		}
		else
			intakeP = -10;
		motor[mtr_int] = intakeP;

		if(vexRT[Btn5U] == 1){
			motor[mtr_indx] = 127;
			} else if(vexRT[Btn5D] == 1){
			motor[mtr_indx] = -127;
			} else {
			motor[mtr_indx] = 0;
		}

		if (vexRT[Btn8L]) {
			desiredRpm = 2275;
			FwVelocitySet(&flywheel, desiredRpm, 0.70);
			} else if (vexRT[Btn8U]) {
			desiredRpm = 2875;
			FwVelocitySet(&flywheel, desiredRpm, 1.00);
			} else if (vexRT[Btn8R]) {
			desiredRpm = 1810;
			FwVelocitySet(&flywheel, desiredRpm, 0.42);
			} else if (vexRT[Btn8D]) {
			desiredRpm = -500;
			FwVelocitySet(&flywheel, desiredRpm, 0);
			} else if (vexRT[Btn8L]) {
			desiredRpm = 2360;
			FwVelocitySet(&flywheel, desiredRpm, 0.42);
			} else if ((btnU || btnD) && !held) {
			desiredRpm += ((btnU - btnD) * RPM_INCREMENT);
			FwVelocitySet(&flywheel, desiredRpm, 0.50);
		}
		held = (btnU + btnD);

		char  str[32];

		sprintf( str, "%4d %4d  %5.2f", flywheel.target,  flywheel.current, nImmediateBatteryLevel/1000.0 );
		displayLCDString(0, 0, str );
		sprintf( str, "%4.2f %4.2f ", flywheel.drive, flywheel.drive_at_zero );
		displayLCDString(1, 0, str );
		wait1Msec(10);
	}
}
