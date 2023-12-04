#!/usr/bin/env python
import time
import rospy
from omniroute_operation.msg import *

# Importing Gantry library
from three_by_three_interface.gcodeclient import Client as GcodeClient
# from three_by_three_interface.find_port port find_port

class GantryFeeder:
    # @brief Initialize the GantryFeeder class
    def __init__(self):

        # @brief Initialize the publisher for writing to '/Esmacat_write_maze_ard0_ease' topic
        # self.maze_ard0_pub = rospy.Publisher('/Esmacat_write_maze_ard0_ease', ease_registers, queue_size=1)

        # @brief Initialize the subsrciber for reading from '/csv_file_name' topic
        rospy.Subscriber('/gantry_cmd', GantryCmd, self.gantry_cmd_callback, queue_size=1, tcp_nodelay=True)

        # ................ GCode Client Setup ................
        self.gcode_client = GcodeClient('/dev/ttyUSB0', 115200)
        ## TODO: Automatically determine the port

        # Wait for a few secs
        # time.sleep(1)
        self.home()
    
    def home(self):
        self.gcode_client.raw_command("$25=5000")
        self.gcode_client.raw_command("$H")
        self.gcode_client.raw_command("G10 P0 L20 X0 Y0 Z0")
        

    def move_gantry(self, x, y):
        self.gcode_client.raw_command("G0 X{} Y{}".format(x,y))
    
    def run_pump(self, duration):
        self.gcode_client.raw_command("M3 S127")
        time.sleep(duration)
        self.gcode_client.raw_command("M5")

    def gantry_cmd_callback(self, msg):
        
        if msg.cmd == "HOME":
            self.home()
        elif msg.cmd == "MOVE":
            # Move the gantry to the specified location
            self.move_gantry(msg.args[0], msg.args[1])
        elif msg.cmd == "FEED":
            self.run_pump(msg.args[0])

# @brief Main code
if __name__ == '__main__':
    # Initialize the ROS node with name 'wall_controller'
    rospy.init_node('gantry_feeder')
    GantryFeeder()  # Create an instance of the class
    rospy.spin()  # Keep the program running until it is explicitly shutdown