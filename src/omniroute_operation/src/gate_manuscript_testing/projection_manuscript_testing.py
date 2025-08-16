#!/usr/bin/env python
# File: src/omniroute_operation/src/gate_manuscript_testing/projection_manuscript_testing.py
# Purpose: Alternate projector wall images (index 1 ↔ 2) on the 8 walls of center chamber (4),
#          interleaved with audio playback, matching the gating/timing pattern of the other test scripts.
#
# roslaunch omniroute_operation projection_manuscript_testing.launch

import rospy
import os
import subprocess
import shutil
import signal
import datetime

from std_msgs.msg import String, Int32, Int32MultiArray, MultiArrayDimension
from omniroute_operation.msg import Event, WallState
from shared_utils.maze_debug import MazeDB

class ProjectionManuscriptTesting:
    def __init__(self):
        MazeDB.printMsg('OTHER', "[ProjectionManuscriptTesting] Node started")

        # -------------------------
        # USER PARAMETERS (knobs)
        # -------------------------
        self.start_delay = 2.5     # optional delay before first stimulus after priming (sec)
        self.n_events    = 40      # total events, must be even (e.g., 40 => 20 images + 20 sounds)
        self.dt          = 5.0     # fixed interval between EVERY event (sec)
        self.sound_cue   = '1KHz'  # audio cue token expected by the audio node

        # Data capture path (same as other test scripts)
        self.data_save_path = "/home/nc4-lassi/omniroute_ubuntu_ws/src/omniroute_operation/src/gate_manuscript_testing/data"

        # -------------------------
        # PUBLISHERS / SUBSCRIBERS
        # -------------------------
        self.marker_pub      = rospy.Publisher('/projector_av_testing', String, queue_size=10)
        self.sound_pub       = rospy.Publisher('sound_cmd', String, queue_size=1)
        self.wall_state_pub  = rospy.Publisher('/wall_state_cmd', WallState, queue_size=10)
        self.proj_cmd_pub    = rospy.Publisher('projection_cmd', Int32, queue_size=3)
        self.proj_image_pub  = rospy.Publisher('projection_image', Int32MultiArray, queue_size=1)

        self.event_sub = rospy.Subscriber('/event', Event, self._on_event, queue_size=1, tcp_nodelay=True)

        # -------------------------
        # STATE
        # -------------------------
        self.setup_complete     = False
        self.recording_started  = False
        self.rosbag_process     = None
        self.event_count        = 0
        self.next_image_index   = 1  # will alternate 1 ↔ 2

        # Pre-build a base 9x9 config of -1 (ignored) for safety
        self.num_chambers  = 9
        self.num_surfaces  = 9  # 8 walls + 1 floor
        self.blank_ignored = [[-1 for _ in range(self.num_surfaces)] for _ in range(self.num_chambers)]

        # Main loop
        rate = rospy.Rate(10)
        while not rospy.is_shutdown():
            if self.setup_complete:
                # Start recordings immediately after setup gate — ONCE
                if not self.recording_started:
                    self._start_recordings()
                    self.recording_started = True   # <<< CRITICAL: prevents re-entering init block every spin
                    self._mark("projection_test_recording_started")

                    # Projector window setup (move to projector monitors, fullscreen, focus)
                    self._projector_window_setup()

                    # Raise center chamber walls once before starting stimuli
                    self._raise_center_walls_once()

                    # Prime projector once with image index 1 on center walls
                    self._publish_center_wall_image(1)
                    self._mark("projection_test_prime_image_index_1")
                    self.next_image_index = 2

                    # Optional start delay before first stimulus
                    if self.start_delay > 0:
                        rospy.sleep(self.start_delay)

                    self._mark("projection_test_start")

                # Run the alternating stimulus sequence
                self._run_sequence()

            rate.sleep()

    # ---------------------------------
    # Event handling
    # ---------------------------------
    def _on_event(self, msg: Event):
        if msg.event == "walls_initialized":
            MazeDB.printMsg('OTHER', "[ProjectionManuscriptTesting] Setup complete (walls_initialized).")
            self.setup_complete = True

    # ---------------------------------
    # Sequence logic
    # ---------------------------------
    def _run_sequence(self):
        # Stop condition: after n_events total (even number)
        if self.event_count >= self.n_events:
            self._mark("projection_test_end")
            rospy.sleep(1.0)
            self._stop_recordings()
            rospy.signal_shutdown("[ProjectionManuscriptTesting] Test complete.")
            return

        # Alternate: image -> sound -> image -> sound ...
        if self.event_count % 2 == 0:
            # IMAGE EVENT
            idx = self.next_image_index              # 1 or 2
            self._publish_center_wall_image(idx)
            self._mark("image_evt_%02d_idx_%d" % ((self.event_count // 2) + 1, idx))
            # toggle for next image event
            self.next_image_index = 2 if self.next_image_index == 1 else 1
        else:
            # SOUND EVENT
            self.sound_pub.publish(self.sound_cue)
            self._mark("sound_evt_%02d_%s" % ((self.event_count // 2) + 1, self.sound_cue))

        self.event_count += 1
        rospy.sleep(self.dt)

    # ---------------------------------
    # Projector helpers
    # ---------------------------------
    def _projector_window_setup(self):
        # -1: toggle windows to projector monitors
        self.proj_cmd_pub.publish(Int32(data=-1))
        rospy.sleep(0.1)

        # -2: toggle fullscreen
        self.proj_cmd_pub.publish(Int32(data=-2))
        rospy.sleep(0.1)

        # -3: force window focus (bring to top)
        self.proj_cmd_pub.publish(Int32(data=-3))
        rospy.sleep(0.1)

        self._mark("projection_windows_configured")

    def _publish_center_wall_image(self, image_index: int):
        """
        Publish an Int32MultiArray specifying which images to display
        on each surface of each chamber. Only the walls (0–7) of chamber 4
        are set to the given image_index; all other surfaces remain -1
        so they are ignored by the projection system.
        """
        # Start from a base config where all surfaces are ignored (-1)
        cfg = [row[:] for row in self.blank_ignored]
        chamber = 4  # center chamber index

        # Set image for all 8 wall surfaces (indices 0–7); floor is index 8
        for wall in range(0, 8):
            cfg[chamber][wall] = image_index

        # Flatten 2D chamber/surface grid into row-major 1D array for publishing
        msg = Int32MultiArray()
        msg.layout.dim = self._setup_layout(self.num_chambers, self.num_surfaces)
        msg.data = [cfg[i][j] for i in range(self.num_chambers) for j in range(self.num_surfaces)]

        # Publish the projection image configuration
        self.proj_image_pub.publish(msg)

    @staticmethod
    def _setup_layout(rows, cols):
        dim = []
        d1 = MultiArrayDimension()
        d1.label = "rows"
        d1.size = rows
        d1.stride = rows * cols
        dim.append(d1)

        d2 = MultiArrayDimension()
        d2.label = "columns"
        d2.size = cols
        d2.stride = cols
        dim.append(d2)
        return dim

    # ---------------------------------
    # Walls (center chamber) helpers
    # ---------------------------------
    def _raise_center_walls_once(self):
        # Batch publish (send=False) for walls 0..7 of chamber 4, then single commit (send=True)
        for wall in range(8):
            ws = WallState()
            ws.chamber = 4
            ws.wall    = [wall]
            ws.state   = True   # up
            ws.send    = False
            self.wall_state_pub.publish(ws)
        # Commit
        ws = WallState()
        ws.chamber = -1
        ws.wall    = [0]
        ws.state   = True
        ws.send    = True
        self.wall_state_pub.publish(ws)
        self._mark("center_walls_raised")

    # ---------------------------------
    # Recording helpers
    # ---------------------------------
    def _start_recordings(self):
        if not os.path.exists(self.data_save_path):
            os.makedirs(self.data_save_path)

        # Start rosbag
        ts = datetime.datetime.now().strftime("%Y%m%d_%H%M%S")
        self.bag_filename = f"ros_session_{ts}.bag"
        bag_path = os.path.join(self.data_save_path, self.bag_filename)
        self.rosbag_process = subprocess.Popen(["rosbag", "record", "-a", "-O", bag_path])
        MazeDB.printMsg("OTHER", f"[ProjectionManuscriptTesting] Rosbag recording started: {bag_path}")

    def _stop_recordings(self):
        # Stop rosbag
        if self.rosbag_process:
            self.rosbag_process.terminate()
            self.rosbag_process.wait()
            MazeDB.printMsg("OTHER", "[ProjectionManuscriptTesting] Rosbag recording stopped.")
            self.rosbag_process = None
    # ---------------------------------
    # Markers
    # ---------------------------------
    def _mark(self, s: str):
        self.marker_pub.publish(s)
        MazeDB.printMsg('OTHER', f"[ProjectionManuscriptTesting] Mark: {s}")

# -------------------------
# Main
# -------------------------
if __name__ == '__main__':
    rospy.init_node('projection_manuscript_testing')
    ProjectionManuscriptTesting()
    rospy.spin()
