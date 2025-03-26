#!/usr/bin/env python
# roslaunch omniroute_operation gate_manuscript_testing.launch

# ROS Imports
import rospy
from std_msgs.msg import String
from omniroute_operation.msg import *

# Audio Imports
import os
import subprocess
import shutil
import signal

# arecord -D hw:4,0 -f S16_LE -r 384000 -c 1 test.wav -d 5

# Custom Imports
from shared_utils.maze_debug import MazeDB

class GateManuscriptTesting:
    def __init__(self):
        MazeDB.printMsg('OTHER', "[GateManuscriptTesting] Node Started")

        # Specify testing prameters
        self.cycle_delay = 5 # time to wait between cycles
        self.n_wall_runs = 3 # number of wall up/down cycles
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

        # Define data save path
        self.data_save_path = "/home/nc4-lassi/omniroute_ubuntu_ws/src/omniroute_operation/src/gate_manuscript_testing/data"
        
        # Define recording parameters
        self.audio_file_name = "audio_recording.wav"  # Change this as needed
        self.recording_process = None

        # Start recording
        MazeDB.printMsg('OTHER', "[GateManuscriptTesting] Start audio recording: %s.", self.audio_file_name)
        self.start_recording()

        # Wait 
        rospy.sleep(1)

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
            
            # Run walls up
            self.run_walls("up")

            # Set walls down
            self.run_walls("down")
            
            self.run_count += 1  # Increment counter
        else:
            rospy.signal_shutdown("[GateManuscriptTesting] Test complete.")
            # Stop recording
            MazeDB.printMsg('OTHER', "[GateManuscriptTesting] Stop audio recording: %s.", self.audio_file_name)
            self.stop_recording()

    def run_walls(self, state):
        """ Raises or lowers all walls. """

        if state == "up":
            self.set_chamber_gates(4, 1)
        elif state == "down":  
            self.set_chamber_gates(4, 0) 

        # Send tone command to time lock to and wait 
        self.publish_message("test_sound_cue")
        self.sound_pub.publish(self.sound_cue)

        # Wait for sound to play
        rospy.sleep(2)

        # run walls
        self.publish_message(f"test_wall_move_{state}")
        self.send_wall_msg()
        rospy.sleep(self.cycle_delay-1)

    def publish_message(self, msg):
        """ Publishes a message to /gate_testing. """
        self.test_pub.publish(msg)
        MazeDB.printMsg('OTHER', f"[GateManuscriptTesting] Published: {msg}")

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
        self.wall_states.chamber = -1
        self.wall_states.wall = [0]
        self.wall_states.state = True
        self.wall_states.send = True
        self.gate_pub.publish(self.wall_states)

    def start_recording(self):
        """ Start recording from the Pettersson M500-384 microphone with debugging. """
        
        # Construct the full path for the audio file
        audio_file = os.path.join(self.data_save_path, self.audio_file_name)
        
        # Ensure the save directory exists
        if not os.path.exists(self.data_save_path):
            MazeDB.printMsg('ERROR', f"[GateManuscriptTesting] Audio save path does not exist: {self.data_save_path}")
            return

        # Check if arecord is installed and accessible
        if not shutil.which("arecord"):
            MazeDB.printMsg('ERROR', "[GateManuscriptTesting] 'arecord' command not found. Ensure ALSA utilities are installed.")
            return

        # Construct the arecord command
        arecord_cmd = [
            "arecord", "-D", "hw:4,0", "-f", "S16_LE", "-r", "384000", "-c", "1", audio_file
        ]

        MazeDB.printMsg('OTHER', f"[GateManuscriptTesting] Executing arecord command: {' '.join(arecord_cmd)}")

        try:
            # Start the recording process
            self.recording_process = subprocess.Popen(
                arecord_cmd, stdout=subprocess.PIPE, stderr=subprocess.PIPE
            )
            
            MazeDB.printMsg('OTHER', f"[GateManuscriptTesting] Recording started: {audio_file} (PID: {self.recording_process.pid})")

        except Exception as e:
            MazeDB.printMsg('ERROR', f"[GateManuscriptTesting] Failed to start recording: {e}")

    def stop_recording(self):
        """ Stop the recording process more gracefully. """
        
        if self.recording_process is None:
            MazeDB.printMsg('ERROR', "[GateManuscriptTesting] No active recording process to stop.")
            return

        try:
            MazeDB.printMsg('OTHER', f"[GateManuscriptTesting] Stopping recording (PID: {self.recording_process.pid})")
            
            # Send SIGINT instead of terminate (equivalent to CTRL+C)
            self.recording_process.send_signal(signal.SIGINT)
            stdout, stderr = self.recording_process.communicate(timeout=5)  
            self.recording_process.wait()

            # Log any output
            if stdout:
                MazeDB.printMsg('OTHER', f"[GateManuscriptTesting] arecord stdout: {stdout.decode().strip()}")
            if stderr and "Aborted by signal Interrupt" not in stderr.decode():
                MazeDB.printMsg('ERROR', f"[GateManuscriptTesting] arecord stderr: {stderr.decode().strip()}")


            MazeDB.printMsg('OTHER', "[GateManuscriptTesting] Recording stopped successfully.")

        except subprocess.TimeoutExpired:
            MazeDB.printMsg('ERROR', "[GateManuscriptTesting] Timeout while stopping recording. Force killing process.")
            self.recording_process.kill()

        except Exception as e:
            MazeDB.printMsg('ERROR', f"[GateManuscriptTesting] Error stopping recording: {e}")

        finally:
            self.recording_process = None




# Main Execution
if __name__ == '__main__':
    rospy.init_node('gate_manuscript_testing')
    GateManuscriptTesting()
    rospy.spin()

