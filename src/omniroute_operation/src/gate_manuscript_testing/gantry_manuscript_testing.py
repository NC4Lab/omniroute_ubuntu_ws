#!/usr/bin/env python
# roslaunch omniroute_operation gantry_manuscript_testing.launch

# ROS Imports
import rospy
from std_msgs.msg import String
from omniroute_operation.msg import GantryCmd, Event

# System Imports
import os
import subprocess
import shutil
import signal
import datetime
import time

# Custom Logging Utility
from shared_utils.maze_debug import MazeDB


class GantryManuscriptTesting:
    def __init__(self):
        MazeDB.printMsg('OTHER', "[GantryManuscriptTesting] Node Started")

        # === Parameters ===
        self.positions = [
            (0.345, 0.420),
            (0.645, 0.720),
            (0.945, 0.420),
            (0.645, 0.120)
        ]
        self.n_moves = 21
        self.move_delay = 5  # seconds
        self.start_delay = 5  # optional delay before first move

        # === ROS Publishers ===
        self.cmd_pub = rospy.Publisher('/gantry_cmd', GantryCmd, queue_size=10)
        self.test_pub = rospy.Publisher('/gantry_testing', String, queue_size=10)

        # === ROS Event Subscriber ===
        self.event_sub = rospy.Subscriber('/event', Event, self.check_for_setup_event, queue_size=1, tcp_nodelay=True)

        # === Recording setup ===
        timestamp = datetime.datetime.now().strftime("%Y%m%d_%H%M%S")
        base_path = "/home/nc4-lassi/omniroute_ubuntu_ws/src/omniroute_operation/src/gate_manuscript_testing/data"
        if not os.path.exists(base_path):
            os.makedirs(base_path)

        self.audio_file = os.path.join(base_path, f"audio_recording_{timestamp}.wav")
        self.bag_file = os.path.join(base_path, f"ros_session_{timestamp}.bag")

        self.recording_started = False
        self.setup_complete = False
        self.audio_process = None
        self.rosbag_process = None

        # === Main Loop ===
        self.rate = rospy.Rate(1)
        self.run_count = 0
        while not rospy.is_shutdown():
            if self.setup_complete and not self.recording_started:
                self.recording_started = True
                self.start_recordings()
                rospy.sleep(self.start_delay)
                self.publish_message("gantry_test_start")
                self.run_test()
                self.publish_message("gantry_test_end")
                self.stop_recordings()
                rospy.signal_shutdown("[GantryManuscriptTesting] Test complete.")
            self.rate.sleep()

    def check_for_setup_event(self, msg):
        if msg.event == "walls_initialized":
            MazeDB.printMsg('OTHER', "[GantryManuscriptTesting] System initialized. Beginning test.")
            self.setup_complete = True

    def run_test(self):
        """Runs n_moves, cycling through position list."""
        pos_index = 0
        for move_num in range(self.n_moves):
            x, y = self.positions[pos_index]
            move_label = f"gantry_test_move_{pos_index + 1}"
            self.publish_message(move_label)
            self.send_gantry_cmd(x, y)
            rospy.sleep(self.move_delay)
            pos_index = (pos_index + 1) % len(self.positions)

    def publish_message(self, msg):
        self.test_pub.publish(msg)
        MazeDB.printMsg('OTHER', f"[GantryManuscriptTesting] Published: {msg}")

    def send_gantry_cmd(self, x_mm, y_mm):
        cmd = GantryCmd()
        cmd.cmd = "move_to_coordinate"
        cmd.args = [x_mm, y_mm]
        self.cmd_pub.publish(cmd)
        MazeDB.printMsg('DEBUG', f"[GantryManuscriptTesting] Sent gantry move to ({x_mm}, {y_mm})")

    def start_recordings(self):
        # --- Start audio ---
        if shutil.which("arecord") is None:
            MazeDB.printMsg('ERROR', "'arecord' not found. Audio recording not started.")
        else:
            arecord_cmd = ["arecord", "-D", "hw:4,0", "-f", "S16_LE", "-r", "384000", "-c", "1", self.audio_file]
            MazeDB.printMsg('OTHER', f"[GantryManuscriptTesting] Starting audio recording: {self.audio_file}")
            try:
                self.audio_process = subprocess.Popen(
                    arecord_cmd, stdout=subprocess.PIPE, stderr=subprocess.PIPE
                )
            except Exception as e:
                MazeDB.printMsg('ERROR', f"Failed to start audio recording: {e}")

        # --- Start rosbag ---
        MazeDB.printMsg('OTHER', f"[GantryManuscriptTesting] Starting rosbag recording: {self.bag_file}")
        self.rosbag_process = subprocess.Popen(["rosbag", "record", "-a", "-O", self.bag_file])

    def stop_recordings(self):
        # --- Stop audio ---
        if self.audio_process:
            MazeDB.printMsg('OTHER', f"[GantryManuscriptTesting] Stopping audio recording (PID: {self.audio_process.pid})")
            try:
                self.audio_process.send_signal(signal.SIGINT)
                stdout, stderr = self.audio_process.communicate(timeout=5)
                if stdout:
                    MazeDB.printMsg('OTHER', f"arecord stdout: {stdout.decode().strip()}")
                if stderr and "Aborted by signal Interrupt" not in stderr.decode():
                    MazeDB.printMsg('ERROR', f"arecord stderr: {stderr.decode().strip()}")
            except subprocess.TimeoutExpired:
                MazeDB.printMsg('ERROR', "Timeout stopping audio, force killing.")
                self.audio_process.kill()
            self.audio_process = None
            MazeDB.printMsg('OTHER', "Audio recording stopped.")

        # --- Stop rosbag ---
        if self.rosbag_process:
            MazeDB.printMsg('OTHER', "Stopping rosbag recording...")
            self.rosbag_process.terminate()
            self.rosbag_process.wait()
            self.rosbag_process = None
            MazeDB.printMsg('OTHER', "Rosbag recording stopped.")


if __name__ == '__main__':
    rospy.init_node('gantry_manuscript_testing')
    GantryManuscriptTesting()
    rospy.spin()
