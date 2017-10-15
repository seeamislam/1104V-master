//these values are basically just gains and can change depending on the situation
//TODO tune these values... reuben pls

//FINAL VALUES USING ZIEGLER NICHOLS METHOD
const float Kp = 2;
const float Ki = 0.01;
const float Kd = 0.79;

//this can also be changed
float INTEGRAL_POWER_LIMIT = 50/Ki;
float INTEGRAL_ACTIVE = inchToTicks(3); 

int drivePower;
int proportional;
int derivative;
int preIntegral;
int integral;
int error;
int lastError;

void(int inches){

while(1==1){
error = inchToTicks(inches) - SensorValue[rightEncoder]; 
proportional = error*Kp;

if(abs(error) < INTEGRAL_ACTIVE && error != 0){
preIntegral = preIntegral+error;
}else {
preIntegral = 0;
}

if(preIntegral > INTEGRAL_POWER_LIMIT){
preIntegral = INTEGRAL_POWER_LIMIT;
}
if(preIntegeral < -INTEGRAL_POWER_LIMIT){
preIntegral = -INTEGRAL_POWER_LIMIT;
}

integral = Ki*preIntegral;
derivative = Kd * (error-lastError);
lastError = error;

int drivePower = proportional+integral+derivative;

}
}
