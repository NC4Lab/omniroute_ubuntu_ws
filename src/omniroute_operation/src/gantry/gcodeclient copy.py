import serial
import time

# Inspired from 
# https://kevinponce.com/blog/python/send-gcode-through-serial-to-a-3d-printer-using-python/

# Code from Tic-Tac-Toe Gantry robot
# https://github.com/TonyJacb/TicTacToe-Gantry-Bot.git

class Client:
    def __init__(self, port, baud) -> None:
        '''
        Initialises the serial port and wakes up GRBL with desired settings.
        :param port: specify the port to which your printer is connected.
                    If it is an Arduino CNC shield, check the port from Arduino IDE
        :param baud: specify the baudrate at which GRBL is communicating. 
        '''
        self.ser = serial.Serial(port, baud)
        time.sleep(1)

        #Keeping track of the position in absolute values
        self.value_X = 0.0
        self.value_Y = 0.0

        # Set Units(does not seem to work on ender 5)
        self.__initialise("G21") # millimeters

        # Absolute Mode
        self.__initialise("G90\r\n")

        # Relative Mode
        # self.__initialise("G91\r\n")

        #Feed Rate
        self.__initialise("F25000\r\n")

    def realtime_command(self, cmd):
        '''
        Interfaces the Gcode commands to GRBL
        :param cmd:  A Gcode String.
        '''  
        self.ser.write(str.encode(cmd))
        time.sleep(0.01)


    def raw_command(self, cmd):
        '''
        Interfaces the Gcode commands to GRBL
        :param cmd:  A Gcode String.
        '''

        try:
            cmd = cmd.upper()
            cmd = cmd + "\r\n"
            self.ser.write(str.encode(cmd))
            time.sleep(0.01)

            feedback = self.ser.readline()
            if feedback == b'ok\r\n':
                return True 
            else:
                # print(f'[gcodeclient] Command: {cmd}')
                # print(f'[gcodeclient] Feedback: {feedback}')
                return False

        except TypeError:
            print("Gcode commands must be a string")

    def command(self, cmd):
        '''
        Interfaces the Gcode commands to GRBL
        :param cmd:  A Gcode String.
        '''
        try:
            cmd = cmd.upper()
            subcmds = cmd.split(" ")
            for subcmd in subcmds:
                if subcmd[0] == "X":
                    self.value_X += float(subcmd[1:])
                elif subcmd[0] == "Y":
                    self.value_Y += float(subcmd[1:])
            # print(f'Value of X: {self.value_X}, y:{self.value_Y}')
            cmd = cmd + "\r\n"
            self.ser.write(str.encode(cmd))
            time.sleep(1)
            while True:
                feedback = self.ser.readline()
                if feedback == b'ok\r\n':
                    # print(feedback)
                    break
        except TypeError:
            print("Gcode commands must be a string")


    def __initialise(self, cmd):
        '''
        Same as that of command but for initialisation. Used in the constructor.
        '''
        cmd = cmd + "\r\n"
        self.ser.write(str.encode(cmd))
        time.sleep(1)
        while True:
            feedback = self.ser.readline()
            if feedback == b'ok\r\n':
                # print(feedback)
                break

    def     flush(self):
        '''
        Use this function to close the serial port.
        '''
        time.sleep(2)
        self.ser.close()
        quit()