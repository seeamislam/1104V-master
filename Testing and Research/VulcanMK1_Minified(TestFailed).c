#pragma config(Sensor,in3,gyro,sensorGyro)
#pragma config(Sensor,in4,Claw_Pot,sensorPotentiometer)
#pragma config(Sensor,in6,AutoPot,sensorPotentiometer)
#pragma config(Sensor,dgtl1,rightEncoder,sensorQuadEncoder)
#pragma config(Sensor,dgtl3,leftEncoder,sensorQuadEncoder)
#pragma config(Sensor,dgtl5,Lift_Switch,sensorTouch)
#pragma config(Sensor,dgtl6,Lift_Enc,sensorQuadEncoder)
#pragma config(Motor,port1,leftDriveBack,tmotorVex393HighSpeed_HBridge,openLoop)
#pragma config(Motor,port2,rightInside,tmotorVex393_MC29,openLoop,reversed)
#pragma config(Motor,port3,rightOutside,tmotorVex393_MC29,openLoop)
#pragma config(Motor,port4,rightDriveFront,tmotorVex393HighSpeed_MC29,openLoop,reversed)
#pragma config(Motor,port5,clawRight,tmotorVex393_MC29,openLoop,reversed)
#pragma config(Motor,port6,clawLeft,tmotorVex393_MC29,openLoop)
#pragma config(Motor,port7,leftDriveFront,tmotorVex393HighSpeed_MC29,openLoop)
#pragma config(Motor,port8,leftOutside,tmotorVex393_MC29,openLoop,reversed)
#pragma config(Motor,port9,leftInside,tmotorVex393_MC29,openLoop)
#pragma config(Motor,port10,rightDriveBack,tmotorVex393HighSpeed_HBridge,openLoop,reversed)
#pragma platform(VEX2)
#pragma competitionControl(Competition)
#include "Vex_Competition_Includes.c"
task liftcontrol();volatile int Lift_Position;int Lift_PositionCount=1;volatile int Lift_ControlActive=!1;volatile int Lift_Power;#define Lift_UpperLimit 422
#define Lift_Pos1 2700
#define Lift_Pos2 2016
#define Lift_Pos3 420
#define Lift_PosRelease 850
#define Lift_EncoderZero 0
volatile bool Break;task clawcontrol();volatile int ClawActive=!0;volatile int Claw_Position;volatile int Claw_Power;#define Claw_Open 3100
#define Claw_Closed 650
#define Claw_ClosedCube 800
#define Claw_Mid 1800
#define Mid 0
#define Closed 1
#define Open 2
volatile int ClawPos=-1;#define LCD_LEFT 1
#define LCD_CENTER 2
#define LCD_RIGHT 4
#define TPR_SPEED 360
int auto=0;bool picked=!1;void liftZero(){SensorValue[Lift_Enc]=Lift_EncoderZero}
task ProgramChooser(){bLCDBacklight=!0;clearLCDLine(0);clearLCDLine(1);const int max=9;const int min=0;string autoName="";while(!picked){if(nLCDButtons==LCD_LEFT){while(nLCDButtons!=0){}
auto --}else if(nLCDButtons==LCD_RIGHT){while(nLCDButtons!=0){}
auto ++}else if(nLCDButtons==LCD_CENTER){while(nLCDButtons!=0){}
picked=!0}
if(auto<min){auto=max}else if(auto>max){auto=min}
switch(auto){case 0:autoName="[Right Cube]";break;case 1:autoName="[Right Stars]";break;case 2:autoName="[Left Cube]";break;case 3:autoName="[Left Cone]";break;case 4:autoName="[Prog Skills]";break;default:autoName="None";break}
displayLCDCenteredString(0,autoName);displayLCDCenteredString(1,"<            >")}}
const unsigned int TrueSpeed[128]={25,25,25,25,25,25,25,25,25,25,25,25,25,25,25,25,25,25,25,25,25,25,26,26,26,27,28,28,28,29,29,30,30,30,30,31,31,32,32,32,33,33,34,34,35,35,35,36,36,37,37,37,37,38,38,39,39,39,40,40,41,41,42,42,43,44,44,45,45,46,46,47,47,48,48,49,50,50,51,52,52,53,54,55,56,57,57,58,59,60,61,62,63,64,65,66,67,67,68,70,71,72,72,73,74,76,77,78,79,79,80,81,83,84,84,86,86,87,87,88,88,89,89,90,90,127,127,127};void SetDrive(int LeftDrivePower,int RightDrivePower){LeftDrivePower=LeftDrivePower>127?127:LeftDrivePower;LeftDrivePower=LeftDrivePower<-127?-127:LeftDrivePower;RightDrivePower=RightDrivePower>127?127:RightDrivePower;RightDrivePower=RightDrivePower<-127?-127:RightDrivePower;if(vexRT(Btn7L)==!0){if(RightDrivePower>0)
RightDrivePower=TrueSpeed[RightDrivePower];else if(RightDrivePower<0)
RightDrivePower=-TrueSpeed[-RightDrivePower];else RightDrivePower=0;LeftDrivePower=RightDrivePower}
else{if(LeftDrivePower>0)
LeftDrivePower=TrueSpeed[LeftDrivePower];else if(LeftDrivePower<0)
LeftDrivePower=-TrueSpeed[-LeftDrivePower];else LeftDrivePower=0;if(RightDrivePower>0)
RightDrivePower=TrueSpeed[RightDrivePower];else if(RightDrivePower<0)
RightDrivePower=-TrueSpeed[-RightDrivePower];else RightDrivePower=0}
motor[rightDriveFront]=RightDrivePower;motor[rightDriveBack]=RightDrivePower;motor[leftDriveFront]=LeftDrivePower;motor[leftDriveBack]=LeftDrivePower}
void RotateAngle(int DesiredAngle,int MaxTime,int PowerLimit){int NotDone;int LeftPwr;int RightPwr;int PreviousError;int RotationError;int RotationPwr;const float RotationkD=2.0;const float RotationkP=0.29;const int SuccessThreshold=15;float Dvalue=0.0;int BrakePwr;const float DvalueLimit=30.0;int BrakeVal=40;int BrakeTime=60;PreviousError=0;DesiredAngle*=10;if(DesiredAngle>0)BrakePwr=-BrakeVal;else BrakePwr=BrakeVal;SensorValue[gyro]=0;clearTimer(T1);NotDone=1;while(((time100[T1])<MaxTime)&&NotDone){RotationError=DesiredAngle-(SensorValue[gyro]);if(abs(RotationError)<SuccessThreshold){NotDone=0;LeftPwr=-BrakePwr;RightPwr=BrakePwr;motor[rightDriveFront]=RightPwr;motor[rightDriveBack]=RightPwr;motor[leftDriveFront]=LeftPwr;motor[leftDriveBack]=LeftPwr;wait1Msec(BrakeTime);SetDrive(0,0)}
else{Dvalue=(RotationError-PreviousError)*RotationkD;PreviousError=RotationError;if(Dvalue>DvalueLimit)Dvalue=DvalueLimit;else if(Dvalue<-DvalueLimit)Dvalue=-DvalueLimit;RotationPwr=(int)((RotationError*RotationkP)+Dvalue);if(RotationPwr<-PowerLimit)RotationPwr=-PowerLimit;else if(RotationPwr>PowerLimit)RotationPwr=PowerLimit;LeftPwr=-RotationPwr;RightPwr=RotationPwr;motor[rightDriveFront]=RightPwr;motor[rightDriveBack]=RightPwr;motor[leftDriveFront]=LeftPwr;motor[leftDriveBack]=LeftPwr}
wait1Msec(10)}
SetDrive(0,0)}
task DriveControl();volatile bool BreakLoop;volatile int DriveMode;volatile float DesiredDriveValue;volatile bool FirstLockCheck=!1;volatile bool UnderLoad;volatile bool DriveActive;volatile float LinekP=.25;#define Line 1
#define Rotation 2
#define Off 3
#define F 1
#define B-1
#define R-1
#define L 1
int LeftTurnConst;int RightTurnConst;float NoLoadLeftTurnConst=7.8;float NoLoadRightTurnConst=7.8;float TurnConst;void SetDriveControl(int Mode,int Value,int Time){DriveMode=Mode;SensorValue(rightEncoder)=0;SensorValue(gyro)=0;TurnConst=(sgn(Value)==-1)?NoLoadRightTurnConst:NoLoadLeftTurnConst;DesiredDriveValue=(Mode==Line)?(Value*360)/10.205:(Value*TurnConst);clearTimer(T1);BreakLoop=!1;DriveActive=!0;while(BreakLoop==!1&&time1[T1]<(Time*1000)){wait1Msec(20)}
FirstLockCheck=!1;BreakLoop=!1;DriveActive=!1}
void SetLiftMotors(int Power){motor[rightInside]=Power;motor[rightOutside]=Power;motor[leftInside]=Power;motor[leftOutside]=Power}
void SetLiftPosition(int Position){Lift_Position=Position}
void Dump(int LiftPos){DriveActive=!1;SetDrive(-120,-120);Lift_ControlActive=!1;Lift_Power=127;clearTimer(T2);while(Break==!1&&time1[T2]<1000){if(SensorValue(Lift_Enc)<Lift_PosRelease){ClawPos=Open;Claw_Position=Open;wait1Msec(500);Break=!0}
wait1Msec(20)}
Break=!1;Lift_ControlActive=!0;SetLiftPosition(LiftPos);wait1Msec(500);SetDrive(0,0);wait1Msec(250);DriveActive=!0}
void Auto(){}
string sLStandard="Standard Left";string sLElims_Front="Elims Front Left";string sLElims_Back="Elims Back Left";string sProgrammingSkills="Skills";string sNone="None";string sRElims_Back="Elims Back Right";string sRElims_Front="Elims Front Right";string sRStandard="Standard Right";string SelectedAuton;void DisplayAuto(){if(SensorValue(AutoPot)<500){SelectedAuton=sLStandard}
else if(SensorValue(AutoPot)<1000){SelectedAuton=sLElims_Front}
else if(SensorValue(AutoPot)<1500){SelectedAuton=sLElims_Back}
else if(SensorValue(AutoPot)<2000){SelectedAuton=sProgrammingSkills}
else if(SensorValue(AutoPot)<2500){SelectedAuton=sNone}
else if(SensorValue(AutoPot)<3000){SelectedAuton=sRElims_Back}
else if(SensorValue(AutoPot)<3500){SelectedAuton=sRElims_Front}
else{SelectedAuton=sRStandard}
bLCDBacklight=!0;clearLCDLine(0);clearLCDLine(1);displayLCDString(0,0,SelectedAuton)}
void RightCube(){SetLiftPosition(Lift_Pos1);Lift_ControlActive=!0;SetLiftPosition(Lift_Pos2-100);SetDriveControl(Line,16,1);ClawPos=Open;Claw_Position=Claw_Open-100;wait1Msec(750);ClawPos=Closed;Claw_Position=Claw_Closed;wait1Msec(1000);RotateAngle(90,1000,100);wait1Msec(500);ClawPos=Open;Claw_Position=Claw_Open;wait1Msec(300);SetLiftPosition(Lift_Pos1);wait1Msec(500);SetDriveControl(Line,15,1);ClawPos=Closed;Claw_Position=Claw_ClosedCube;wait1Msec(300);SetLiftPosition(Lift_Pos2);wait1Msec(300);SetDriveControl(Line,-28,2);wait1Msec(700);RotateAngle(88,1000,100);SetDriveControl(Line,-17,2);wait1Msec(400);SetLiftPosition(Lift_Pos3+200);wait1Msec(1050);ClawPos=Open;Claw_Position=Claw_Open;sleep(200);SetLiftPosition(Lift_Pos1)}
void rightStrs(){SetDriveControl(Line,-3,1);sleep(250);SetLiftPosition(Lift_Pos1);Lift_ControlActive=!0;ClawPos=Open;Claw_Position=Claw_Mid;wait1Msec(800);SetDriveControl(Line,24.5,1);wait1Msec(1200);ClawPos=Open;Claw_Position=Claw_Closed;wait1Msec(500);SetLiftPosition(Lift_Pos2);wait1Msec(600);SetDriveControl(Line,-25,1);wait1Msec(1500);RotateAngle(75,1000,100);wait1Msec(600);SetDriveControl(Line,-26,1);wait1Msec(2000);SetLiftPosition(Lift_Pos3+150);wait1Msec(1000);ClawPos=Open;Claw_Position=Claw_Open}
void LeftCone(){sleep(6666);SetLiftPosition(Lift_Pos1);Lift_ControlActive=!0;SetLiftPosition(Lift_Pos2-100);SetDriveControl(Line,16,1);ClawPos=Open;Claw_Position=Claw_Open-100;wait1Msec(750);ClawPos=Closed;Claw_Position=Claw_Closed;wait1Msec(1000);RotateAngle(-95,1000,100);wait1Msec(500);ClawPos=Open;Claw_Position=Claw_Open;wait1Msec(300);SetLiftPosition(Lift_Pos1);wait1Msec(500);SetDriveControl(Line,15,1);ClawPos=Closed;Claw_Position=Claw_ClosedCube;wait1Msec(300);SetLiftPosition(Lift_Pos2);wait1Msec(300);SetDriveControl(Line,-28,2);wait1Msec(700);RotateAngle(-88,1000,100);SetDriveControl(Line,-17,2);wait1Msec(400);SetLiftPosition(Lift_Pos3+200);wait1Msec(1050);ClawPos=Open;Claw_Position=Claw_Open;sleep(200);SetLiftPosition(Lift_Pos1)}
void LeftCube(){SetLiftPosition(Lift_Pos1);Lift_ControlActive=!0;SetLiftPosition(Lift_Pos2-100);SetDriveControl(Line,16,1);ClawPos=Open;Claw_Position=Claw_Open-100;wait1Msec(750);ClawPos=Closed;Claw_Position=Claw_Closed;wait1Msec(1000)}
void DumpAuto(int time){wait1Msec(400);SetLiftPosition(Lift_Pos3+200);wait1Msec(time);ClawPos=Open;Claw_Position=Claw_Open;sleep(1000);SetLiftPosition(Lift_Pos1);sleep(2000)}
void PreloadDump(int time,float distance){SetDriveControl(Line,distance,1);sleep(2500);ClawPos=Closed;Claw_Position=Claw_Closed;sleep(1000);SetDriveControl(Line,(-1)*distance,1);sleep(2000);DumpAuto(time)}
void ProgSkill(){SetLiftPosition(Lift_Pos1);Lift_ControlActive=!0;SetLiftPosition(Lift_Pos2-100);SetDriveControl(Line,10,1);ClawPos=Open;Claw_Position=Claw_Open;wait1Msec(750)}
void pre_auton(){SensorType(in3)=sensorNone;wait1Msec(1000);SensorType(in3)=sensorGyro;wait1Msec(1000);liftZero();sleep(500);bStopTasksBetweenModes=!0;SensorValue(rightEncoder)=0;startTask(ProgramChooser)}
task autonomous(){startTask(liftcontrol);startTask(clawcontrol);startTask(DriveControl);stopTask(ProgramChooser);switch(auto){case 0:RightCube();break;case 1:rightStrs();break;case 2:LeftCube();break;case 3:LeftCone();break;case 4:ProgSkill();break}}
task usercontrol(){startTask(liftcontrol);startTask(clawcontrol);bool Lift_Toggle1=!1;bool Lift_Toggle2=!1;bool Claw_Toggle=!1;ClawPos=-1;int LeftDrive;int RightDrive;while(!0){if(vexRT(Btn8R)==!0){LeftDrive=60;RightDrive=-60}
else{LeftDrive=abs(vexRT(Ch3))>20?vexRT(Ch3):0;RightDrive=abs(vexRT(Ch2))>20?vexRT(Ch2):0}
SetDrive(LeftDrive,RightDrive);if(vexRT(Btn5U)==1){Lift_ControlActive=!1;Lift_PositionCount=1;Lift_Power=127;if(SensorValue(Lift_Enc)<Lift_PosRelease){ClawPos=Open}}
else if(vexRT(Btn7D)==1){Lift_ControlActive=!1;Lift_PositionCount=1;Lift_Power=-127}
else{Lift_Power=0}
if(vexRT(Btn8U)==1){ClawPos=Mid}
if(vexRT(Btn6U)==1){if(Claw_Toggle==!1){ClawPos++;if(ClawPos>2){ClawPos=1}
Claw_Toggle=!0}}
else{Claw_Toggle=!1}
if(vexRT(Btn5D)==1){if(Lift_Toggle1==!1){Lift_PositionCount=Lift_PositionCount-1;if(Lift_PositionCount>1){Lift_PositionCount=1}
Lift_ControlActive=!0}
Lift_Toggle1=!0}
else{Lift_Toggle1=!1}
if(vexRT(Btn6D)==1){if(Lift_Toggle2==!1){Lift_PositionCount++;if(Lift_PositionCount>3){Lift_PositionCount=3}
Lift_ControlActive=!0}
Lift_Toggle2=!0}
else{Lift_Toggle2=!1}
if(ClawPos==Closed){Claw_Position=Claw_Closed}
else if(ClawPos==Mid){Claw_Position=Claw_Mid}
else if(ClawPos==Open){Claw_Position=Claw_Open}
switch(Lift_PositionCount){case 1:SetLiftPosition(Lift_Pos1);break;case 2:SetLiftPosition(Lift_Pos2);break;case 3:SetLiftPosition(Lift_Pos3);break}
if(vexRT(Btn8L)==1){startTask(DriveControl);Auto();stopTask(DriveControl)}}}
task clawcontrol(){int Claw_Current;float Claw_kP=0.2;for(;;){if(ClawActive){Claw_Current=SensorValue(Claw_Pot);if(ClawPos==Closed){Claw_Power=((Claw_Position-Claw_Current)*Claw_kP)}
else if(ClawPos==Open){Claw_Power=((Claw_Position-Claw_Current)*Claw_kP*1.05)}
else if(ClawPos==Mid){Claw_Power=((Claw_Position-Claw_Current)*Claw_kP*1.05)}
if(SensorValue(Claw_Pot)>Claw_Closed){}
if(SensorValue(Claw_Pot)<Claw_Open){}
writeDebugStreamLine("int x is: %d",Claw_Power);motor[clawLeft]=Claw_Power;motor[clawRight]=Claw_Power;wait1Msec(20)}}}
task liftcontrol(){int Lift_Current;float DkP;int Lift_Hold=10;while(!0){if(Lift_ControlActive){Lift_Current=SensorValue(Lift_Enc);DkP=((Lift_Position-Lift_Current)>0)?.07:.30;Lift_Power=(-(Lift_Position-Lift_Current)*DkP+Lift_Hold);if(Lift_Position==Lift_Pos1){if(Lift_Power>30){Lift_Power=30}}}
if(SensorValue(Lift_Switch)==!0&&SensorValue(Lift_Enc)>1500){if(Lift_Power<0){Lift_Power=0}
else{Lift_Power=Lift_Power}}
if(SensorValue(Lift_Enc)<Lift_UpperLimit){if(Lift_Power>0){Lift_Power=0}
else{Lift_Power=Lift_Power}}
SetLiftMotors(Lift_Power);wait1Msec(20)}}
task DriveControl(){int DrivePower;int DriveDirection;float CurrentDriveValue;float DkP;float DkI;int LockingThreshold=15;int BrakingPower;int BrakingTime;short BatteryLvl;int P;int I;int IVal=0;int I_Limit=20;for(;;){if(DriveActive){CurrentDriveValue=DriveMode==Line?SensorValue(rightEncoder):SensorValue(gyro);DriveDirection=sgn(DrivePower);DkP=DriveMode==Line?LinekP:.2;if(DriveMode==Rotation){DkP=DesiredDriveValue>(75*TurnConst)?.2:.25}
DkI=DriveMode==Line?0:.0;BrakingPower=DriveMode==Line?20:80;BrakingTime=80;P=(DesiredDriveValue-CurrentDriveValue);IVal=IVal+P;I=IVal*DkI;I=I<I_Limit?I:I_Limit;I=I>-I_Limit?I:-I_Limit;if(abs(P)>LockingThreshold){DrivePower=P*DkP+I}
if(abs(P)<LockingThreshold){DrivePower=-DriveDirection*BrakingPower;IVal=0;FirstLockCheck=!0}
BatteryLvl=nImmediateBatteryLevel/1000;DrivePower=(DrivePower*pow(8.4/BatteryLvl,2));switch(DriveMode){case Line:SetDrive(DrivePower,DrivePower);break;case Rotation:SetDrive(-DrivePower,DrivePower);break;case Off:SetDrive(0,0);break}
if(FirstLockCheck==!0){wait1Msec(BrakingTime);SetDrive(0,0);wait1Msec(50);if(abs(P)<LockingThreshold){BreakLoop=!0}
else{FirstLockCheck=!1}}
wait1Msec(5)}
else{wait1Msec(5)}}}
