#!/usr/bin/env python
import cv2
import os

# Define the folder containing AVI files
folder_path = r"\\10.34.1.59\big_gulp\nc4_rat_data\Maze_Rats\NC40008\250129\Cam_14" # Change this to your folder path
output_file = os.path.join(folder_path, "merged_video.avi")

# Get list of AVI files in the folder
avi_files = [f for f in os.listdir(folder_path) if f.endswith(".avi")]
avi_files.sort()  # Ensure they are in order

if not avi_files:
    print("No AVI files found in the folder.")
    exit()

# Read the first video to get properties
first_video_path = os.path.join(folder_path, avi_files[0])
cap = cv2.VideoCapture(first_video_path)
frame_width = int(cap.get(3))
frame_height = int(cap.get(4))
fps = int(cap.get(cv2.CAP_PROP_FPS))
cap.release()

# Define the codec and create VideoWriter object
fourcc = cv2.VideoWriter_fourcc(*'XVID')
out = cv2.VideoWriter(output_file, fourcc, fps, (frame_width, frame_height))

# Process each AVI file
for avi_file in avi_files:
    video_path = os.path.join(folder_path, avi_file)
    cap = cv2.VideoCapture(video_path)

    while cap.isOpened():
        ret, frame = cap.read()
        if not ret:
            break
        out.write(frame)

    cap.release()

out.release()
cv2.destroyAllWindows()
print(f"Merged video saved as: {output_file}")
