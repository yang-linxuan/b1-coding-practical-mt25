class PDController:
    def __init__(self, kp=0.15, kd=0.6):
        """
        Initialises the PD controller with given proportional and derivative gains.
        Args:
            kp (float): Proportional gain
            kd (float): Derivative gain
        """
        self.kp = kp
        self.kd = kd 
        self.previous_error = 0  # Initialise previous error to zero

    def compute_control_action(self, reference, output):
        """
        Computes the control action using a PD controller.
        Args:
            reference (float): The desired reference value (r[t])
            output (float): The current output value (y[t])
        """
        # Calculate the current error (e[t] = r[t] - y[t])
        error = reference - output

        # PD control law: u[t] = kp * e[t] + kd * (e[t] - e[t-1])
        control_action = self.kp * error + self.kd * (error - self.previous_error)
        
        # Update previous error for the next iteration
        self.previous_error = error
        
        return control_action