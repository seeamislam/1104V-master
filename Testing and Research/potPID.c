
void liftPID(){ 
int target;
int trror;
int drive;
int errorSum;
int lastError;
int derivative

int Kp = 0.5;
int Ki = 0.2;
int Kd = 0.05;
while(1==1){
if ( vexRT[Btn5D] == 1 )
{
Target = 600 ;
}
else if ( vexRT[Btn5D] == 1 )
{
Target = 1000 ;
}
else if ( vexRT[Btn5D] == 1 )
{
Target = 1600 ;
}
liftSensor = SensorValue[pot]; 
Error = (liftSensor - Target); 
drive = (error*Kp)+(errorSum*Ki)+(derivative*Kd); 
errorSum += error;
derivative = error-lastError;
lastError = error;
}
}
