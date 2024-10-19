class CustomPID:
    def __init__(self, Kp, Ki, Kd, setpoint, output_limits=(-70, 70)):
        self.Kp = Kp  # Proportional gain
        self.Ki = Ki  # Integral gain
        self.Kd = Kd  # Derivative gain
        self.setpoint = setpoint  # Desired value
        self.output_limits = output_limits  # Limits for PID output
        
        self.prev_error = 0  # Store the previous error for derivative calculation
        self.integral = 0  # Store the sum of errors for integral calculation
        self.last_time = None  # To calculate time difference between updates

    def update(self, current_value):
        """ Compute PID output based on the current value and setpoint """
        error = self.setpoint - current_value
        
        # Calculate the time difference since the last update
        current_time = time.time()
        if self.last_time is None:
            self.last_time = current_time
        dt = current_time - self.last_time
        
        # Proportional term
        P = self.Kp * error
        
        # Integral term
        self.integral += error * dt
        I = self.Ki * self.integral
        
        # Derivative term
        derivative = (error - self.prev_error) / dt if dt > 0 else 0
        D = self.Kd * derivative
        
        # Combine the terms
        output = P + I + D
        
        # Apply output limits
        output = max(min(output, self.output_limits[1]), self.output_limits[0])
        
        # Update previous error and time for next iteration
        self.prev_error = error
        self.last_time = current_time
        
        return output
