# Default system settings [default][min][max]
self.sysDefaults = {
    # Number of chambers to initialize in maze used for testing
    "n_chamb_init": [3, 1, 9],
    # Number of chambers to move at once
    "n_chamb_move": [1, 1, 3],
    # Number of attempts to move a walls
    "n_move_attempt": [2, 1, 3],
    # PWM duty cycle for wall motors
    "pwm_duty": [255, 0, 255],
    # Timeout for wall movement (ms)
    "move_timeout": [1000, 1, 2000]
}

# Make sure n_chambers_move default equal <= n_chambers_init
if self.sysDefaults["n_chambers_move"][2] > self.sysDefaults["n_chambers_init"][0]:
    self.sysDefaults["n_chambers_move"][2] = self.sysDefaults["n_chambers_init"][0]



             # Update number of init chambers setting based on chambersi2c errors
                cham_cnt = 0
                for i in range(self.EsmaCom.rcvEM.argLen): 
                    if self.EsmaCom.rcvEM.ArgU.ui8[i] == 0:
                        cham_cnt += 1
                self.setParamTxtBox(param_ind=0, arg_val=cham_cnt)