#pragma config(Sensor, in1,    Lift_Pot,       sensorPotentiometer)
#pragma config(Sensor, in2,    gyro,           sensorGyro)
#pragma config(Sensor, in4,    Claw_Pot,       sensorPotentiometer)
#pragma config(Sensor, in5,    AutoPot,        sensorPotentiometer)
#pragma config(Sensor, dgtl1,  rightEncoder,   sensorQuadEncoder)
#pragma config(Sensor, dgtl3,  leftEncoder,    sensorQuadEncoder)
#pragma config(Sensor, dgtl5,  Lift_Switch,    sensorTouch)
#pragma config(Motor,  port1,           leftDriveBack, tmotorVex393TurboSpeed_HBridge, openLoop)
#pragma config(Motor,  port2,           rightInside,   tmotorVex393_MC29, openLoop, reversed)
#pragma config(Motor,  port3,           rightOutside,  tmotorVex393_MC29, openLoop)
#pragma config(Motor,  port4,           rightDriveFront, tmotorVex393TurboSpeed_MC29, openLoop, reversed)
#pragma config(Motor,  port5,           clawRight,     tmotorVex393_MC29, openLoop, reversed)
#pragma config(Motor,  port6,           clawLeft,      tmotorVex393_MC29, openLoop)
#pragma config(Motor,  port7,           leftDriveFront, tmotorVex393TurboSpeed_MC29, openLoop)
#pragma config(Motor,  port8,           leftOutside,   tmotorVex393_MC29, openLoop, reversed)
#pragma config(Motor,  port9,           leftInside,    tmotorVex393_MC29, openLoop)
#pragma config(Motor,  port10,          rightDriveBack, tmotorVex393TurboSpeed_HBridge, openLoop, reversed)
//*!!Code automatically generated by 'ROBOTC' configuration wizard               !!*//

// This code is for the VEX cortex platform
#pragma platform(VEX2)

// Select Download method as "competition"
#pragma competitionControl(Competition)

//Main competition background code...do not modify!
#include "Vex_Competition_Includes.c"
task liftcontrol();

volatile int Lift_Position;
int Lift_PositionCount = 1;
volatile int Lift_ControlActive = false;
volatile int Lift_Power;
#define Lift_UpperLimit 1700 //(190)
#define Lift_Pos1 4080 //(2720)
#define Lift_Pos2 3500
#define Lift_Pos3 2200
#define Lift_PosRelease 2700
volatile bool Break;



task clawcontrol(); // Close + Open -
volatile int ClawActive = true;
volatile int Claw_Position;
volatile int Claw_Power;
#define Claw_Open 1510 //(150)
#define Claw_Closed 4000 //(2310)
#define Claw_ClosedCube 3600
#define Claw_Mid 3000
#define Mid 0
#define Closed 1
#define Open 2
volatile int ClawPos = -1;

word maxDrivePower = 127;
long driveTarget = 0;
long driveKl = 100;  // Drive integral limit.
float driveKp = 0.15;
float driveKi = 0.0;
float driveKd = 0;
float turnRatio = 1;  // 12 Ratio of turnKp to use for driving straight.

word maxTurnPower = 127;
long turnTarget = 0;
long turnKl = 100;  // Turn integral limit.
float turnKp = 0.2;
float turnKi = 0.0;
float turnKd = 2;
float driveRatio = 0;  // Ratio of driveKp to use for turning on a dime.



task driveThread() {
	long encoderL, encoderR, t = time1[T1], dt, error, turnError, lastError = 0, p, i = 0, power, turnPower;
	float d;

	while (true) {
		encoderR = SensorValue[rightEncoder];
		encoderL = SensorValue[leftEncoder];
		dt = (time1[T1] - t);

		error = (driveTarget - (encoderR + encoderL));
		turnError = (turnTarget - (encoderR - encoderL));

		p = error;
		i = ((error < driveKl) ? (i + (error * dt)) : 0);
		d = ((error - lastError) / ((dt > 0) ? dt : 1));
		power = ((driveKp * p) + (driveKi * i) + (driveKd * d));
		turnPower = ((turnKp * turnRatio) * turnError);

		if (fabs(power) > maxDrivePower) {
			power = (sgn(power) * maxDrivePower);
		}

		motor[rightDriveFront] = motor[rightDriveFront] = (power + turnPower);
		motor[leftDriveFront]  = motor[leftDriveBack] = (power - turnPower);


		lastError = error;
		t += dt;

		sleep(10);
	}
}

task turnThread() {
	long encoderR, encoderL, t = time1[T1], dt, error, driveError, lastError = 0, p, i = 0, power, drivePower;
	float d;

	while (true) {
		encoderR = SensorValue[rightEncoder];
		encoderL = SensorValue[leftEncoder];
		dt = (time1[T1] - t);

		error = (turnTarget - (encoderR - encoderL));
		driveError = (driveTarget - (encoderR + encoderL));

		p = error;
		i = ((error < turnKl) ? (i + (error * dt)) : 0);
		d = ((error - lastError) / ((dt > 0) ? dt : 1));

		power = ((turnKp * p) + (turnKi * i) + (turnKd * d));
		drivePower = ((driveKp * driveRatio) * driveError);

		if (fabs(power) > maxTurnPower) {
			power = (sgn(power) * maxTurnPower);
		}

		motor[rightDriveFront] = motor[rightDriveBack]  = (drivePower + power);
		motor[leftDriveFront] = motor[leftDriveFront]  = (drivePower - power);

		lastError = error;
		t += dt;

		sleep(10);
	}
}


void drive(long distance, word power = maxDrivePower) {
	stopTask(turnThread);
	stopTask(driveThread);

	maxDrivePower = power;
	driveTarget += (distance*2);

	startTask(driveThread);
}

void driveInch(long distance, word power = maxDrivePower) {
	stopTask(turnThread);
	stopTask(driveThread);

	int DistInch= ((distance*2)/(3.25*PI))*(360);
	maxDrivePower = power;
	driveTarget += (DistInch);

	startTask(driveThread);

}

const unsigned int TrueSpeed[128] =

{

	25,	 25,	25,	 25,	25,	 25,	25,	 25,	25,	 25,

	25, 25, 25, 25, 25, 25, 25, 25, 25, 25,

	25, 25, 26, 26, 26, 27, 28, 28, 28, 29,

	29, 30, 30, 30, 30, 31, 31, 32, 32, 32,

	33, 33, 34, 34, 35, 35, 35, 36, 36, 37,

	37, 37, 37, 38, 38, 39, 39, 39, 40, 40,

	41, 41, 42, 42, 43, 44, 44, 45, 45, 46,

	46, 47, 47, 48, 48, 49, 50, 50, 51, 52,

	52, 53, 54, 55, 56, 57, 57, 58, 59, 60,

	61, 62, 63, 64, 65, 66, 67, 67, 68, 70,

	71, 72, 72, 73, 74, 76, 77, 78, 79, 79,

	80, 81, 83, 84, 84, 86, 86, 87, 87, 88,

	88, 89, 89, 90, 90, 127, 127, 127

};

void SetDrive(int LeftDrivePower, int RightDrivePower){

	LeftDrivePower = LeftDrivePower > 127 ? 127 : LeftDrivePower;

	LeftDrivePower = LeftDrivePower < -127 ? -127 : LeftDrivePower;

	RightDrivePower = RightDrivePower > 127 ? 127 : RightDrivePower;

	RightDrivePower = RightDrivePower < -127 ? -127 : RightDrivePower;

	if(vexRT(Btn7L) == true){

		if(RightDrivePower > 0)

		RightDrivePower = TrueSpeed[RightDrivePower];

		else if(RightDrivePower < 0)

		RightDrivePower = -TrueSpeed[-RightDrivePower];

		else

		RightDrivePower = 0;
		LeftDrivePower = RightDrivePower;

	}

	else{

		if(LeftDrivePower > 0)

		LeftDrivePower = TrueSpeed[LeftDrivePower];

		else if(LeftDrivePower < 0)

		LeftDrivePower = -TrueSpeed[-LeftDrivePower];

		else

		LeftDrivePower = 0;



		// right

		if(RightDrivePower > 0)

		RightDrivePower = TrueSpeed[RightDrivePower];

		else if(RightDrivePower < 0)

		RightDrivePower = -TrueSpeed[-RightDrivePower];

		else

		RightDrivePower = 0;

	}



	motor[rightDriveFront] = RightDrivePower;

	motor[rightDriveBack] = RightDrivePower;

	motor[leftDriveFront] = LeftDrivePower;

	motor[leftDriveBack] = LeftDrivePower;

}


void RotateAngle(int DesiredAngle,int MaxTime,int PowerLimit) {

	//send a neg. angle to turn right.right rotations go negative, power to rotate right is L+,R-
	int NotDone;int LeftPwr;int RightPwr;int PreviousError;int RotationError;int RotationPwr;
	const float RotationkD=2.0; const float RotationkP=0.29; 	const int SuccessThreshold=15; // this is 1.5 degrees //0.29 for 100 speed, 0.25 for 127 speed
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
		RotationError=DesiredAngle - (SensorValue[gyro]);// TheError is error value
		if (abs(RotationError) < SuccessThreshold) {
			NotDone=0;
			LeftPwr= -BrakePwr; RightPwr= BrakePwr;
			motor[rightDriveFront] = RightPwr;

			motor[rightDriveBack] = RightPwr;

			motor[leftDriveFront] = LeftPwr;

			motor[leftDriveBack] = LeftPwr;
			wait1Msec(BrakeTime);
			//testcode
			SetDrive(0,0);
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
			motor[rightDriveFront] = RightPwr;

			motor[rightDriveBack] = RightPwr;

			motor[leftDriveFront] = LeftPwr;

			motor[leftDriveBack] = LeftPwr;
		}
		wait1Msec(10);// find best val for this. was 2
	}
	SetDrive(0,0);
}


task DriveControl();

volatile bool BreakLoop;

volatile int DriveMode;

volatile float DesiredDriveValue;

volatile bool FirstLockCheck = false;

volatile bool UnderLoad;

volatile bool DriveActive;

volatile float LinekP = .25;

#define Line 1

#define Rotation 2

#define Off 3

#define F 1

#define B -1

#define R -1

#define L 1

int LeftTurnConst;

int RightTurnConst;

float NoLoadLeftTurnConst = 7.8;

float NoLoadRightTurnConst = 7.8;

float TurnConst;

void SetDriveControl(int Mode, int Value, int Time){

	DriveMode = Mode;

	SensorValue(rightEncoder) = 0;

	SensorValue(Gyro) = 0;

	TurnConst = (sgn(Value) == -1) ? NoLoadRightTurnConst : NoLoadLeftTurnConst;

	DesiredDriveValue = (Mode == Line) ? (Value*360)/10.205 : (Value * TurnConst);

	clearTimer(T1);

	BreakLoop = false;

	DriveActive = true;

	while(BreakLoop == false && time1[T1] < (Time*1000)){

		wait1Msec(20);

	}

	FirstLockCheck = false;

	BreakLoop = false;

	DriveActive = false;

}

void SetLiftMotors(int Power){

	motor[rightInside] = Power;

	motor[rightOutside] = Power;

	motor[leftOutside] = Power;

	motor[leftInside] = Power;
}

void SetLiftPosition(int Position){

	Lift_Position = Position;

}

void Dump(int LiftPos){

	DriveActive = false;
	SetDrive(-120, -120);
	Lift_ControlActive = false;
	Lift_Power = 127;
	clearTimer(T2);
	while(Break == false && time1[T2] < 1000){
		if(SensorValue(Lift_Pot) < Lift_PosRelease){
			ClawPos = Open;
			Claw_Position = Open;
			wait1Msec(500);
			Break = true;
		}
		wait1Msec(20);
	}
	Break = false;
	Lift_ControlActive = true;
	SetLiftPosition(LiftPos);
	wait1Msec(500);
	SetDrive(0, 0);
	wait1Msec(250);
	DriveActive = true;
}

void Auto(){}


string sLStandard = "Standard Left";
string sLElims_Front = "Elims Front Left";
string sLElims_Back = "Elims Back Left";
string sProgrammingSkills = "Skills";
string sNone = "None";
string sRElims_Back = "Elims Back Right";
string sRElims_Front = "Elims Front Right";
string sRStandard = "Standard Right";
string SelectedAuton;

void DisplayAuto(){
	if(SensorValue(AutoPot) < 500){
		SelectedAuton = sLStandard;
	}
	else if(SensorValue(AutoPot) < 1000){
		SelectedAuton = sLElims_Front;
	}
	else if(SensorValue(AutoPot) < 1500){
		SelectedAuton = sLElims_Back;
	}
	else if(SensorValue(AutoPot) < 2000){
		SelectedAuton = sProgrammingSkills;
	}
	else if(SensorValue(AutoPot) < 2500){
		SelectedAuton = sNone;
	}
	else if(SensorValue(AutoPot) < 3000){
		SelectedAuton = sRElims_Back;
	}
	else if(SensorValue(AutoPot) < 3500){
		SelectedAuton = sRElims_Front;
	}
	else{
		SelectedAuton = sRStandard;
	}

	bLCDBacklight = true;

	clearLCDLine(0); clearLCDLine(1);

	displayLCDString(0, 0, SelectedAuton);

}

/////////////////////////////////////////////////////////////////////////////////////////

//

//													Pre-Autonomous Functions

//

// You may want to perform some actions before the competition starts. Do them in the

// following function.

//

/////////////////////////////////////////////////////////////////////////////////////////



void pre_auton(){

	SensorType(in3) = sensorNone;

	wait1Msec(2000);

	SensorType(in3) = sensorGyro;

	wait1Msec(1000);

	bStopTasksBetweenModes = true;

	SensorValue(rightEncoder) = 0;

	SensorValue[gyro] = 0;

	DisplayAuto();

}

task autonomous(){

	startTask(liftcontrol); startTask(clawcontrol); startTask(DriveControl);
	driveInch(12,80);


}
task usercontrol(){

	startTask(liftcontrol); startTask(clawcontrol);

	bool Lift_Toggle1 = false; bool Lift_Toggle2 = false;

	bool Claw_Toggle = false; ClawPos = -1;

	int LeftDrive; int RightDrive;

	while (true){

		DisplayAuto();

		if(vexRT(Btn8R) == true){

			LeftDrive = 60;

			RightDrive = -60;

		}

		else{

			LeftDrive = abs(vexRT(Ch3)) > 20 ? vexRT(Ch3) : 0;

			RightDrive = abs(vexRT(Ch2)) > 20 ? vexRT(Ch2) : 0;

		}

		SetDrive(LeftDrive, RightDrive);

		if(vexRT(Btn5U) == 1){

			Lift_ControlActive = false;

			Lift_PositionCount = 1;

			Lift_Power = 127;

			if(SensorValue(Lift_Pot) < Lift_PosRelease){

				ClawPos = Open;

			}

		}

		else if (vexRT(Btn7D) == 1){

			Lift_ControlActive = false;

			Lift_PositionCount = 1;

			Lift_Power = -127;

		}

		else{

			Lift_Power = 0;

		}



		if(vexRT(Btn8U) == 1){

			ClawPos = Mid;

		}

		if(vexRT(Btn6U) == 1){

			if(Claw_Toggle == false){

				ClawPos++;

				if(ClawPos > 2){

					ClawPos = 1;

				}

				Claw_Toggle = true;

			}

		}

		else{

			Claw_Toggle = false;

		}



		if(vexRT(Btn5D) == 1){

			if(Lift_Toggle1 == false){

				Lift_PositionCount = Lift_PositionCount - 1;

				if(Lift_PositionCount > 1){

					Lift_PositionCount = 1;

				}

				Lift_ControlActive = true;

			}

			Lift_Toggle1 = true;

		}

		else{

			Lift_Toggle1 = false;

		}



		if(vexRT(Btn6D) == 1){

			if(Lift_Toggle2 == false){

				Lift_PositionCount++;

				if(Lift_PositionCount > 3){

					Lift_PositionCount = 3;

				}

				Lift_ControlActive = true;

			}

			Lift_Toggle2 = true;

		}

		else{

			Lift_Toggle2 = false;

		}



		if(ClawPos == Closed){

			Claw_Position = Claw_Closed;

		}

		else if (ClawPos == Mid){

			Claw_Position = Claw_Mid;

		}

		else if (ClawPos == Open){

			Claw_Position = Claw_Open;

		}



		switch(Lift_PositionCount){

		case 1:

			SetLiftPosition(Lift_Pos1);

			break;



		case 2:

			SetLiftPosition(Lift_Pos2);

			break;



		case 3:

			SetLiftPosition(Lift_Pos3);

			break;

		}

		if(vexRT(Btn8L) == 1){

			startTask(DriveControl);

			Auto();

			stopTask(DriveControl);

		}

	}

}

task clawcontrol(){

	int Claw_Current; float Claw_kP = .2;

	while(true){

		if(ClawActive){

			//Update Values

			Claw_Current = SensorValue(Claw_Pot); //Current Error



			if(ClawPos == Closed){

				Claw_Power = ((Claw_Position - Claw_Current) * Claw_kP);

				Claw_Power = Claw_Power < 0 ? 0 : Claw_Power;

			}

			else if(ClawPos == Open){

				Claw_Power = ((Claw_Position - Claw_Current) * Claw_kP * 1.05);

				Claw_Power = Claw_Power > 0 ? 0 : Claw_Power;

			}

			else if(ClawPos == Mid){

				Claw_Power = ((Claw_Position - Claw_Current) * Claw_kP * 1.05);

			}



			//System Limits

			if(SensorValue(Claw_Pot) > Claw_Closed){

				Claw_Power = Claw_Power > 0 ? 0 : Claw_Power;

			}



			if(SensorValue(Claw_Pot) < Claw_Open){

				Claw_Power = Claw_Power < 0 ? 0 : Claw_Power;

			}

			motor[ClawLeft] = Claw_Power;

			motor[ClawRight] = Claw_Power;

			wait1Msec(20);

		}

	}

}



task liftcontrol(){

	int Lift_Current; float DkP; int Lift_Hold = 10;

	while (true){

		if(Lift_ControlActive){

			Lift_Current = SensorValue(Lift_Pot);

			DkP = ((Lift_Position - Lift_Current) > 0) ? .1 : .35;

			Lift_Power = (-(Lift_Position - Lift_Current) * DkP + Lift_Hold);

			if(Lift_Position == Lift_Pos1){

				if(Lift_Power > 30){

					Lift_Power = 30;

				}

			}

		}



		if(SensorValue(Lift_Switch) == true && SensorValue(Lift_Pot) > 1500){

			if(Lift_Power < 0){

				Lift_Power = 0;

			}

			else{

				Lift_Power = Lift_Power;

			}

		}



		if(SensorValue(Lift_Pot) < Lift_UpperLimit){

			if(Lift_Power > 0){

				Lift_Power = 0;

			}

			else{

				Lift_Power = Lift_Power;

			}

		}

		SetLiftMotors(Lift_Power);

		wait1Msec(20);

	}

}



task DriveControl(){



	int DrivePower; int DriveDirection; float CurrentDriveValue; float DkP; float DkI;

	int LockingThreshold = 15;

	int BrakingPower; int BrakingTime; short BatteryLvl;

	int P; int I; int IVal = 0; int I_Limit = 20;

	while(true){

		if(DriveActive){

			CurrentDriveValue = DriveMode == Line ? SensorValue(rightEncoder) : SensorValue(Gyro);

			DriveDirection = sgn(DrivePower);

			DkP = DriveMode == Line ? LinekP : .2;

			if(DriveMode == Rotation){

				DkP = DesiredDriveValue > (75 * TurnConst) ? .2 : .25;

			}

			DkI = DriveMode == Line ? 0 : .0;

			BrakingPower = DriveMode == Line ? 20 : 80;



			//if(DriveMode == Line){

			BrakingTime = 80;

			/*}
			else if(DriveMode == Rotation){
			BrakingTime = abs(DesiredDriveValue) > 720 ? 150 : 100;
			}*/

			P = (DesiredDriveValue - CurrentDriveValue);

			IVal = IVal + P;

			I = IVal * DkI;

			I = I < I_Limit ? I : I_Limit;

			I = I > -I_Limit ? I : -I_Limit;

			if(abs(P) > LockingThreshold){

				DrivePower = P * DkP + I;

			}

			if(abs(P) < LockingThreshold){

				DrivePower = -DriveDirection*BrakingPower;

				IVal = 0;

				FirstLockCheck = true;

			}

			BatteryLvl = nImmediateBatteryLevel/1000;

			DrivePower = (DrivePower * pow(8.4/BatteryLvl, 2));

			switch(DriveMode){

			case Line: SetDrive(DrivePower, DrivePower);		break;

			case Rotation: SetDrive(-DrivePower, DrivePower); break;

			case Off: SetDrive(0,0); break;

			}

			if(FirstLockCheck == true){

				wait1Msec(BrakingTime);

				SetDrive(0,0);

				wait1Msec(50);

				if(abs(P) < LockingThreshold){

					BreakLoop = true;

				}

				else{

					FirstLockCheck = false;

				}

			}

			wait1Msec(5);

		}

		else{

			wait1Msec(5);

		}

	}

}
