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
import datetime

# arecord -D hw:4,0 -f S16_LE -r 384000 -c 1 test.wav -d 5

# Custom Imports
from shared_utils.maze_debug import MazeDB

class GateManuscriptTesting:
    def __init__(self):
        MazeDB.printMsg('OTHER', "[GateManuscriptTesting] Node Started")

        # Specify testing prameters
        self.start_delay = 0 # time to wait before starting test
        self.cycle_delay = 5 # time to wait between cycles
        self.n_wall_runs = 20 # number of wall up/down cycles
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

        # Flags
        self.setup_complete = False
        self.recording_started = False

        # Define data save path and file names
        self.data_save_path = "/home/nc4-lassi/omniroute_ubuntu_ws/src/omniroute_operation/src/gate_manuscript_testing/data"
        self.audio_file_name = "audio_recording.wav"  # Audio recording filename
        self.bag_filename = "ros_recording.bag"  # Rosbag recording filename
        self.recording_process = None

        # Start loop
        self.rate = rospy.Rate(1)  # 1 Hz
        while not rospy.is_shutdown():
            if self.setup_complete:
                
                # Start audio and rosbag recordings
                if not self.recording_started:
                    self.recording_started = True

                    # Start recordings
                    self.start_rosbag_recordings()
                    self.start_audio_recording()
                    rospy.sleep(self.start_delay)
                    self.publish_message("gate_test_start")
                
                # Run the test
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

            # Stop audio and rosbag recordings
            self.publish_message("gate_test_end")
            rospy.sleep(1)
            self.stop_rosbag_recordings()
            self.stop_audio_recording()
            

    def run_walls(self, state):
        """ Raises or lowers all walls. """

        if state == "up":
            self.set_chamber_gates(4, 1)
        elif state == "down":  
            self.set_chamber_gates(4, 0) 

        # Send tone command to time lock to and wait 
        self.publish_message("gate_test_sound_cue")
        self.sound_pub.publish(self.sound_cue)

        # Wait for sound to play
        rospy.sleep(2)

        # run walls
        self.publish_message(f"gate_test_wall_move_{state}")
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

    def start_audio_recording(self):
        """ Start recording from the Pettersson M500-384 microphone with debugging. """

        # Construct the full path for the audio file
        timestamp = datetime.datetime.now().strftime("%Y%m%d_%H%M%S")
        self.audio_file_name = f"audio_recording_{timestamp}.wav"
        audio_path = os.path.join(self.data_save_path, self.audio_file_name)
        
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
            "arecord", "-D", "hw:4,0", "-f", "S16_LE", "-r", "384000", "-c", "1", audio_path
        ]

        MazeDB.printMsg('OTHER', f"[GateManuscriptTesting] Executing arecord command: {' '.join(arecord_cmd)}")

        try:
            # Start the recording process
            self.recording_process = subprocess.Popen(
                arecord_cmd, stdout=subprocess.PIPE, stderr=subprocess.PIPE
            )
            
            MazeDB.printMsg('OTHER', f"[GateManuscriptTesting] Recording started: {audio_path} (PID: {self.recording_process.pid})")

        except Exception as e:
            MazeDB.printMsg('ERROR', f"[GateManuscriptTesting] Failed to start recording: {e}")

    def stop_audio_recording(self):
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

    def start_rosbag_recordings(self):
        """Start rosbag recordings with timestamped filenames."""
        
        timestamp = datetime.datetime.now().strftime("%Y%m%d_%H%M%S")
        self.bag_filename = f"ros_session_{timestamp}.bag"
        self.bag_path = os.path.join(self.data_save_path, self.bag_filename)
        self.rosbag_process = subprocess.Popen([
            "rosbag", "record", "-a", "-O", self.bag_path
        ])
        MazeDB.printMsg("OTHER", f"[GateManuscriptTesting] Rosbag recording started: {self.bag_path}")

    def stop_rosbag_recordings(self):
        """Stop rosbag recordings."""

        if self.rosbag_process:
            self.rosbag_process.terminate()
            self.rosbag_process.wait()
            MazeDB.printMsg("OTHER", "[GateManuscriptTesting] Rosbag recording stopped.")

# Main Execution
if __name__ == '__main__':
    rospy.init_node('gate_manuscript_testing')
    GateManuscriptTesting()
    rospy.spin()

