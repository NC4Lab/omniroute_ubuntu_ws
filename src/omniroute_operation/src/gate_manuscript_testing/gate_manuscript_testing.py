#!/usr/bin/env python

# ROS Imports
import rospy
from std_msgs.msg import String
from omniroute_operation.msg import *

# Custom Imports
from shared_utils.maze_debug import MazeDB

class GateManuscriptTesting:
    def __init__(self):
        MazeDB.printMsg('OTHER', "[GateManuscriptTesting] Node Started")

        # Store test mode flags
        self.do_test = "sound" # [sound, ephys]

        # Specify testing prameters
        self.cycle_delay = 5 # time to wait between cycles
        self.n_wall_runs = 2 # number of wall up/down cycles
        self.run_count = 0 # counter for number of runs

        # Specify sound cue
        self.sound_cue = '1KHz'

        # Publishers
        self.test_pub = rospy.Publisher('/gate_testing', String, queue_size=10)
        self.gate_pub = rospy.Publisher('/wall_state_cmd', WallState, queue_size=10)
        self.sound_pub = rospy.Publisher('sound_cmd', String, queue_size=1)

        # Initialize wall state
        self.wall_states = WallState()

        # Subscribe to events
        self.event_sub = rospy.Subscriber('/event', Event, self.check_for_setup_event, queue_size=1, tcp_nodelay=True)

        # Track setup status
        self.setup_complete = False

        # Start loop
        self.rate = rospy.Rate(1)  # 1 Hz
        while not rospy.is_shutdown():
            if self.setup_complete:
                rospy.sleep(5)  # 5-second delay before running
                self.run()
            self.rate.sleep()

    def check_for_setup_event(self, msg):
        """ Check for setup completion event. """
        if msg.event == "walls_initialized":
            MazeDB.printMsg('OTHER', "[GateManuscriptTesting] All Arduinos connected. Starting test.")
            self.setup_complete = True

    def run(self):
        """ Runs the appropriate test a fixed number of times. """
        if self.run_count < self.n_wall_runs:
            if self.do_test == "sound":
                self.run_sound_test()
            elif self.do_test == "ephys":
                self.run_ephys_test()
            
            self.run_count += 1  # Increment counter
        else:
            rospy.signal_shutdown("[GateManuscriptTesting] Test complete.")

    def run_sound_test(self):
        """ Executes the sound test. """
        MazeDB.printMsg('OTHER', "[GateManuscriptTesting] Running Sound Test.")

        # Run walls up
        self.run_walls(1)

        # Run walls down
        self.run_walls(1)

    def run_walls(self, state):
        """ Raises or lowers all walls. """

        # Set center chamber gates to the state position
        self.set_chamber_gates(4, state)

        # Send tone command to time lock to and wait 
        self.publish_message("test_sound_cue")
        self.sound_pub.publish(self.sound_cue)
        rospy.sleep(1)

        # run walls
        self.publish_message("test_walls_%d", state)
        self.send_wall_msg()
        rospy.sleep(self.cycle_delay-1)

    def run_ephys_test(self):
        """ Executes the ephys test (to be defined later). """
        MazeDB.printMsg('OTHER', "[GateManuscriptTesting] Running Ephys Test.")

    def publish_message(self, msg):
        """ Publishes a message to /gate_testing. """
        self.test_pub.publish(msg)
        MazeDB.printMsg('OTHER', f"[GateManuscriptTesting] Published: {msg}")

    def set_all_chambers(self, state):
        """ Sets all gates in all chambers to the specified state. """
        for chamber in range(9):  # Chambers 0-8
            for gate in range(8):  # Gates 0-7
                self.set_gate(chamber, gate, state)

    def set_chamber_gates(self, cham, state):
        """ Sets all gates in a specific chamber to the specified state. """
        for gate in range(8):  # Gates 0-7
            self.set_gate(cham, gate, state)


    def set_gate(self, cham, wall, state):
        """ Sets the specified wall state (raise or lower). """
        self.wall_states.chamber = cham
        self.wall_states.wall = [wall]  # Ensure wall is a list
        self.wall_states.state = state
        self.wall_states.send = False
        self.gate_pub.publish(self.wall_states)

    def send_wall_msg(self):
        self.wall_states.send = True
        self.gate_pub.publish(self.wall_states)


# Main Execution
if __name__ == '__main__':
    rospy.init_node('gate_manuscript_testing')
    GateManuscriptTesting()
    rospy.spin()

