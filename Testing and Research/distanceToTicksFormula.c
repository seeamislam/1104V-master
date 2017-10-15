const int WHEEL_CIRCUMFERENCE = 10.21;
//The circ of the wheel. if we change wheels someone please for the love of God update this

int numOfRevolutions;
//declare those variables boiii
//Made a function. this is how u call/use it: distanceAuto(distance,power); e.g. distance auto(24,127);
void distanceAuto(int distance, int power){
distance = WHEEL_CIRCUMFERENCE * numOfRevolutions;
  // this is the basic eqn for distance tb to the BOE bots
numOfRevolutions = distance/WHEEL_CIRCUMFERENCE;
  //rearrange dat beatch
int ticks = numOfRevolutions*90;
//There are 90 ticks in an encoder therefore a full revolution is 90 ticks
  //This basically sends the value to the encoders
  /* what this basically means is.... until the encoder reaches a certain number of ticks we run the motors and a specified power
  when the encoders reach this certain target the motors will stop running */

}
