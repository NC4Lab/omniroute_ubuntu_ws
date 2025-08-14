#!/usr/bin/env python
# File: src/omniroute_operation/src/gate_manuscript_testing/projection_manuscript_testing.py
# Purpose: After setup, raise the center chamber (index 4) walls, then run an alternating
#          sequence of image displays (index 1 ↔ 2) and audio playbacks. Images are shown
#          on the 8 walls of the center chamber only. Audio is published to /sound_cmd.
# Notes:
#   - Walls are surfaces 0..7; floor is surface 8.
#   - ROS bag recording is active; audio .wav capture removed.
#
# Example launch override:
#   <node pkg="omniroute_operation" type="projection_manuscript_testing.py" name="projection_manuscript_testing" output="screen">
#     <param name="n_events" value="40"/>
#     <param name="dt" value="5.0"/>
#     <param name="start_delay" value="0.0"/>
#     <param name="sound_cue" value="1KHz"/>
#   </node>

import rospy
import os
import subprocess
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
        # Allow param overrides from launch; fall back to defaults if unset.
        self.start_delay = rospy.get_param("~start_delay", 0.0)   # seconds before first stimulus after priming
        self.n_events    = int(rospy.get_param("~n_events", 40)) # total events (image/sound alternating)
        self.dt          = float(rospy.get_param("~dt", 5.0))    # interval between events (seconds)
        self.sound_cue   = rospy.get_param("~sound_cue", "1KHz") # string token published to /sound_cmd

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
        self.next_image_index   = 1  # alternates 1 ↔ 2 on image events

        # Pre-build a base 9x9 config of -1 (ignored)
        self.num_chambers  = 9
        self.num_surfaces  = 9  # 8 walls + 1 floor
        self.blank_ignored = [[-1 for _ in range(self.num_surfaces)] for _ in range(self.num_chambers)]

        # Graceful shutdown to stop rosbag if needed
        rospy.on_shutdown(self._on_shutdown)

        # Main loop
        rate = rospy.Rate(10)
        while not rospy.is_shutdown():
            if self.setup_complete:
                # One-time recording + projector/walls prime
                if not self.recording_started:
                    self._start_recordings()
                    self.recording_started = True
                    self._mark("projection_test_recording_started")

                    # Projector window setup (move → fullscreen → focus)
                    self._projector_window_setup()

                    # Raise center chamber walls once before starting stimuli
                    self._raise_center_walls_once()

                    # Prime the projector once with image index 1 on center walls
                    self._publish_center_wall_image(1)
                    self._mark("projection_test_prime_image_index_1")
                    self.next_image_index = 2

                    # Optional start delay before the first stimulus event
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
        # Stop condition after n_events total
        if self.event_count >= self.n_events:
            self._mark("projection_test_end")
            rospy.sleep(1.0)
            self._stop_recordings()
            rospy.signal_shutdown("[ProjectionManuscriptTesting] Test complete.")
            return

        # Alternate: image -> sound -> image -> sound ...
        if self.event_count % 2 == 0:
            # IMAGE EVENT
            idx = self.next_image_index  # 1 or 2
            self._publish_center_wall_image(idx)
            self._mark("image_evt_%02d_idx_%d" % ((self.event_count // 2) + 1, idx))
            # Toggle for next image event
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
        # -1: move windows to projector monitors
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
        Publish a 9x9 (chamber x surface) Int32MultiArray where only chamber 4 walls are set.
        All other entries are -1 (ignored by the display node). Floor (surface 8) is left untouched.
        """
        cfg = [row[:] for row in self.blank_ignored]  # deep copy
        chamber = 4

        # Walls are 0..7; floor is 8. Set only walls.
        for wall in range(0, 8):
            cfg[chamber][wall] = image_index

        # Build and publish
        msg = Int32MultiArray()
        msg.layout.dim = self._setup_layout(self.num_chambers, self.num_surfaces)  # clarity; display ignores layout
        msg.data = [cfg[i][j] for i in range(self.num_chambers) for j in range(self.num_surfaces)]
        self.proj_image_pub.publish(msg)

        # Lightweight marker to confirm on-wire content for chamber 4
        self._mark("push c4 walls[0..7]=" + ",".join(str(cfg[chamber][w]) for w in range(8)) + " floor=" + str(cfg[chamber][8]))

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
        if self.rosbag_process:
            self.rosbag_process.terminate()
            self.rosbag_process.wait()
            MazeDB.printMsg("OTHER", "[ProjectionManuscriptTesting] Rosbag recording stopped.")
            self.rosbag_process = None

    # ---------------------------------
    # Markers & shutdown
    # ---------------------------------
    def _mark(self, s: str):
        self.marker_pub.publish(s)
        MazeDB.printMsg('OTHER', f"[ProjectionManuscriptTesting] Mark: {s}")

    def _on_shutdown(self):
        # Ensure rosbag is terminated if the node dies early
        try:
            self._stop_recordings()
        except Exception:
            pass


# -------------------------
# Main
# -------------------------
if __name__ == '__main__':
    rospy.init_node('projection_manuscript_testing')
    ProjectionManuscriptTesting()
    rospy.spin()
