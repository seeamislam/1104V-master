#pragma config(Motor,  port1,           l,             tmotorVex393_HBridge, openLoop)
#pragma config(Motor,  port2,           l,             tmotorVex393_MC29, openLoop)
#pragma config(Motor,  port3,           l,             tmotorVex393_MC29, openLoop)
#pragma config(Motor,  port4,           l,             tmotorVex393_MC29, openLoop)
#pragma config(Motor,  port5,           l,             tmotorVex393_MC29, openLoop)
#pragma config(Motor,  port6,           l,             tmotorVex393_MC29, openLoop)
#pragma config(Motor,  port7,           l,             tmotorVex393_MC29, openLoop)
#pragma config(Motor,  port8,           l,             tmotorVex393_MC29, openLoop)
#pragma config(Motor,  port9,           l,             tmotorVex393_MC29, openLoop)
#pragma config(Motor,  port10,          l,             tmotorVex393_HBridge, openLoop)

task main()
{
for(int i = 0; i < 20; i++){
motor[port1] = 127;
wait1Msec(250);
motor[port2] = 127;
wait1Msec(250);
motor[port3] = 127;
wait1Msec(250);
motor[port4] = 127;
wait1Msec(250);
motor[port5] = 127;
wait1Msec(250);
motor[port6] = 127;
wait1Msec(250);
motor[port7] = 127;
wait1Msec(250);
motor[port8] = 127;
wait1Msec(250);
motor[port9] = 127;
wait1Msec(250);
motor[port10] = 127;
wait1Msec(250);
}
}
