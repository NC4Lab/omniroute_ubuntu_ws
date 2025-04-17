#!/usr/bin/env python3

import os
import subprocess
import shutil
import signal
import datetime
import time

class RecordCalibrationAudio:
    def __init__(self):
        self.duration_sec = 20  # ← Change this duration as needed
        self.data_save_path = "/home/nc4-lassi/omniroute_ubuntu_ws/src/omniroute_operation/src/gate_manuscript_testing/data"
        self.audio_file_name = None
        self.recording_process = None

        print('OTHER', "[RecordCalibrationAudio] Initialized")

    def start_recording(self):
        timestamp = datetime.datetime.now().strftime("%Y%m%d_%H%M%S")
        self.audio_file_name = f"calibration_audio_{timestamp}.wav"
        audio_path = os.path.join(self.data_save_path, self.audio_file_name)

        if not os.path.exists(self.data_save_path):
            print('ERROR', f"[RecordCalibrationAudio] Save path does not exist: {self.data_save_path}")
            return

        if not shutil.which("arecord"):
            print('ERROR', "[RecordCalibrationAudio] 'arecord' command not found. Is ALSA installed?")
            return

        arecord_cmd = [
            "arecord", "-D", "hw:4,0", "-f", "S16_LE",
            "-r", "384000", "-c", "1", audio_path
        ]

        print('OTHER', f"[RecordCalibrationAudio] Running command: {' '.join(arecord_cmd)}")
        try:
            self.recording_process = subprocess.Popen(
                arecord_cmd, stdout=subprocess.PIPE, stderr=subprocess.PIPE
            )
            print('OTHER', f"[RecordCalibrationAudio] Recording started: {audio_path} (PID: {self.recording_process.pid})")
        except Exception as e:
            print('ERROR', f"[RecordCalibrationAudio] Failed to start recording: {e}")

    def stop_recording(self):
        if self.recording_process is None:
            print('ERROR', "[RecordCalibrationAudio] No active recording process to stop.")
            return

        try:
            print('OTHER', f"[RecordCalibrationAudio] Stopping recording (PID: {self.recording_process.pid})")
            self.recording_process.send_signal(signal.SIGINT)
            stdout, stderr = self.recording_process.communicate(timeout=5)
            if stdout:
                print('OTHER', f"[RecordCalibrationAudio] arecord stdout: {stdout.decode().strip()}")
            if stderr and "Aborted by signal Interrupt" not in stderr.decode():
                print('ERROR', f"[RecordCalibrationAudio] arecord stderr: {stderr.decode().strip()}")
            print('OTHER', "[RecordCalibrationAudio] Recording stopped successfully.")
        except subprocess.TimeoutExpired:
            print('ERROR', "[RecordCalibrationAudio] Timeout—killing process.")
            self.recording_process.kill()
        except Exception as e:
            print('ERROR', f"[RecordCalibrationAudio] Error stopping recording: {e}")
        finally:
            self.recording_process = None

if __name__ == "__main__":
    recorder = RecordCalibrationAudio()
    recorder.start_recording()
    time.sleep(recorder.duration_sec)
    recorder.stop_recording()
