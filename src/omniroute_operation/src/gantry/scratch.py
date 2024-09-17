class GantryOperation:
    def __init__(self):
        # Your existing initialization code...

        # Relay feedback for Ziegler-Nichols (separate for X and Y axes)
        self.relay_output_x = 0.0
        self.relay_output_y = 0.0
        self.last_error_sign_x = 0
        self.last_error_sign_y = 0
        self.oscillations_x = []
        self.oscillations_y = []
        self.last_cross_time_x = None
        self.last_cross_time_y = None
        self.amplitude = 10  # This can be adjusted based on system needs
        
        # PID auto-tuned parameters (separate for X and Y axes)
        self.Ku_x = None
        self.Tu_x = None
        self.Ku_y = None
        self.Tu_y = None

    def loop(self):
        if self.gantry_mode == GantryState.TRACK_HARNESS:
            gantry_to_harness = np.array([self.harness_x - self.gantry_x, self.harness_y - self.gantry_y])
            distance = np.linalg.norm(gantry_to_harness)

            if self.track_method == 'tune':
                if distance > 0.01:
                    # Perform relay feedback to induce oscillations independently for X and Y
                    output_x, output_y = self.relay_feedback_control(gantry_to_harness)

                    # Send movement commands to gantry
                    self.move_gantry_rel(output_x, output_y)
                    
                    # Calculate and apply PID tuning for X axis once enough oscillations have occurred
                    if len(self.oscillations_x) >= 2:
                        self.Kp_x, self.Ki_x, self.Kd_x = self.calculate_pid_parameters(self.oscillations_x)

                    # Calculate and apply PID tuning for Y axis once enough oscillations have occurred
                    if len(self.oscillations_y) >= 2:
                        self.Kp_y, self.Ki_y, self.Kd_y = self.calculate_pid_parameters(self.oscillations_y)

                elif self.movement_in_progress:
                    self.jog_cancel()
                    self.movement_in_progress = False

            # PID tracking code...

    def relay_feedback_control(self, gantry_to_setpoint):
        """
        Relay feedback control to induce oscillations in both X and Y directions independently.
        This method will switch outputs whenever the error crosses the threshold.
        """
        error_x, error_y = gantry_to_setpoint[0], gantry_to_setpoint[1]
        current_sign_x, current_sign_y = np.sign(error_x), np.sign(error_y)
        current_time = time.time()

        # Handle X axis relay feedback
        if current_sign_x != self.last_error_sign_x and abs(error_x) > self.threshold:
            self.relay_output_x = self.amplitude if error_x > 0 else -self.amplitude
            if self.last_cross_time_x:
                oscillation_period_x = current_time - self.last_cross_time_x
                self.oscillations_x.append((oscillation_period_x, abs(self.relay_output_x)))
            self.last_cross_time_x = current_time
        self.last_error_sign_x = current_sign_x

        # Handle Y axis relay feedback
        if current_sign_y != self.last_error_sign_y and abs(error_y) > self.threshold:
            self.relay_output_y = self.amplitude if error_y > 0 else -self.amplitude
            if self.last_cross_time_y:
                oscillation_period_y = current_time - self.last_cross_time_y
                self.oscillations_y.append((oscillation_period_y, abs(self.relay_output_y)))
            self.last_cross_time_y = current_time
        self.last_error_sign_y = current_sign_y

        return self.relay_output_x, self.relay_output_y

    def calculate_pid_parameters(self, oscillations):
        """
        Calculate PID parameters using Ziegler-Nichols tuning rules.
        This is done separately for each axis.
        """
        if len(oscillations) < 2:
            return None, None, None  # Wait until at least 2 oscillations

        # Calculate ultimate period (Tu) and ultimate gain (Ku) from oscillations
        periods, amplitudes = zip(*oscillations[-2:])
        Tu = np.mean(periods)
        Ku = 4 * self.amplitude / (np.pi * Tu)  # From Ziegler-Nichols method

        # Ziegler-Nichols tuning rules for PID
        Kp = 0.6 * Ku
        Ki = 2 * Kp / Tu
        Kd = Kp * Tu / 8

        return Kp, Ki, Kd




def loop(self):
    if self.gantry_mode == GantryState.TRACK_HARNESS:
        gantry_to_harness = np.array([self.harness_x - self.gantry_x, self.harness_y - self.gantry_y])
        distance = np.linalg.norm(gantry_to_harness)

        if self.track_method == 'tune':
            if distance > 0.01:
                # Relay feedback for X axis
                self.relay_output_x, self.last_error_sign_x, self.last_cross_time_x, self.oscillations_x = self.relay_feedback_control(
                    gantry_to_harness[0],  # X error
                    self.relay_output_x, 
                    self.last_error_sign_x, 
                    self.last_cross_time_x, 
                    self.amplitude, 
                    self.oscillations_x, 
                    axis_name="x"
                )

                # Relay feedback for Y axis
                self.relay_output_y, self.last_error_sign_y, self.last_cross_time_y, self.oscillations_y = self.relay_feedback_control(
                    gantry_to_harness[1],  # Y error
                    self.relay_output_y, 
                    self.last_error_sign_y, 
                    self.last_cross_time_y, 
                    self.amplitude, 
                    self.oscillations_y, 
                    axis_name="y"
                )

                # Send movement commands to gantry based on relay feedback
                self.move_gantry_rel(self.relay_output_x, self.relay_output_y)

                # Calculate and apply PID tuning for X axis once enough oscillations have occurred
                if len(self.oscillations_x) >= 2:
                    self.Kp_x, self.Ki_x, self.Kd_x = self.calculate_pid_parameters(self.oscillations_x)

                # Calculate and apply PID tuning for Y axis once enough oscillations have occurred
                if len(self.oscillations_y) >= 2:
                    self.Kp_y, self.Ki_y, self.Kd_y = self.calculate_pid_parameters(self.oscillations_y)

            elif self.movement_in_progress:
                self.jog_cancel()
                self.movement_in_progress = False





