 #pragma config(Sensor, in1,    pot,            sensorPotentiometer)
#pragma config(Sensor, in2,    gyro,           sensorGyro)
#pragma config(Motor,  port1,           leftTopOut,    tmotorVex393_HBridge, openLoop, reversed)
#pragma config(Motor,  port2,           rightDrive,    tmotorVex393HighSpeed_MC29, openLoop, driveRight)
#pragma config(Motor,  port3,           leftDrive,     tmotorVex393HighSpeed_MC29, openLoop, driveLeft)
#pragma config(Motor,  port4,           rightTopOut,   tmotorVex393_MC29, openLoop)
#pragma config(Motor,  port5,           rightBotIn,    tmotorVex393_MC29, openLoop, reversed)
#pragma config(Motor,  port6,           rightBotOut,   tmotorVex393_MC29, openLoop)
#pragma config(Motor,  port7,           leftBotIn,     tmotorVex393_MC29, openLoop)
#pragma config(Motor,  port8,           leftBotOut,    tmotorVex393_MC29, openLoop, reversed)
#pragma config(Motor,  port9,           leftTopIn,     tmotorVex393_MC29, openLoop)
#pragma config(Motor,  port10,          rightTopIn,    tmotorVex393_HBridge, openLoop, reversed)

int rightBack;
int leftBack;

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
	motor[leftDrive] = left;
	motor[rightDrive] = right;

}
/*
word maxDrivePower = 127;
long driveTarget = 0;
long driveKl = 100;  // Drive integral limit.
float driveKp = 0.15;
float driveKi = 0.0;
float driveKd = 2;
float turnRatio = 7;  // 12 Ratio of turnKp to use for driving straight in a straight line.

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
			motor[driveRight]=RightPwr;motor[driveLeft]=LeftPwr;
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
				motor[driveRight]=RightPwr;motor[driveLeft]=LeftPwr;
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


task main()

{
	while (true)
  {
  //bool straightBool;
		//while(straightBool == true){
		//}
		backDrive();
		//while(backDriveBool == 1){
		//	straightBool = true;
		//}
		 motor[driveRight] = rightBack;
		 motor[driveLeft] = leftBack;

		rightBack = trueSpeed(vexRT[Ch2]);
		leftBack = trueSpeed(vexRT[Ch3]);
  }

}
