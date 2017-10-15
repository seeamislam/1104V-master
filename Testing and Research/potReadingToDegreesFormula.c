
int degreePOT;
//y = mx+ b nigguh
int target = (degreePOT/0.61) + 2247.5;
/* 
aight lemme explain this sheeeeeiitt!
i wanted to change the readings of the potentiometer which is mounted on the lift axle from voltage readings
that ranged from 0 - 4095 to degrees which for the potentiometer is 0 - 250 degrees.
btw for future reference lets refer to the Potentiometer as POT or pot(420 blaze it)
we can model the relationship between the potReading to its actual degrees by using our good ol friend y = mx+b
where y is the degrees, x is the actual voltage readings that the pot is giving us, and m and b are just constants that modify the pot readings to degrees
with a bit of math i found that m is 0.61 approx and b is -2247.5
so we are left with the eqn y = 0.61x - 2247.5
the robot cortex reads stuff in voltage readings which is the x so now we need to reaarange the formula to find x(isolate dat hoe)
the new eqn will be (y/0.61) + 2247.5 = x
on the code above i have named x as the target and y as the degreePOT
*/
