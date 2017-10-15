  error = setpoint - measured_value // calculates the proportional error which is target - actualValue
  integral = integral + error*dt // Determines the history of the error overtime to minimize undershoot
  derivative = (error - previous_error)/dt //Trying to preempt the next error to minimize overshoot
  output = Kp*error + Ki*integral + Kd*derivative // Proportional + Integral + Derivative
  previous_error = error /* sends the value of the last error into the previous_error private 
  variable to be used in the derivative calculation */
  wait(dt) // refresh rate of the PID control loop
  goto start //a while loop with a never ennding condition could be used 
