import os
from dateutil.parser import parse as parsedate
from rosbags.highlevel import AnyReader as RosBagReader
from pathlib import Path

def get_log(rat, date, path=os.environ['DATA_PATH']):
    print(f"Using path: {path}") 
    if isinstance(rat, str):
        rat = int(rat)

    # Look in the specific animal folder based on the number designated to each animal 
    anim_folder = os.path.join(path, 'NC4%04d' % rat)
    print(f"Using path: {path}")
    print(f"Looking in folder: {anim_folder}")

    # Look for a specific date 
    if '-' in date:  # e.g. date = '24-Jul-24' 
        date = parsedate(date).strftime('%y%m%d')

    date_folder = os.path.join(anim_folder, date)

    raw_folder = os.path.join(date_folder, 'Raw')

    ros_folder = os.path.join(raw_folder, 'ROS')

    print(f"Looking in folder: {ros_folder}")
    
    bag_files = []  # To store paths of all bag files found

    # Collect all .bag files in the folder
    for file in os.listdir(ros_folder):
        if file.endswith('.bag'):
            bag_files.append(os.path.join(ros_folder, file))

    if not bag_files:
        raise FileNotFoundError("No .bag files found in the specified folder.")
    

    msg_list = []
    # Create dataframe with messages and timestamps
    for bag_file in bag_files:
        with RosBagReader([Path(bag_file)]) as reader:
            connections = [x for x in reader.connections if x.topic == '/rosout']
            for connection, timestamp, rawdata in reader.messages(connections=connections):
                msg = reader.deserialize(rawdata, connection.msgtype)
                msg_list.append([timestamp, msg.msg])

    return msg_list

    # # Read messages from all .bag files found
    # for bag_file in bag_files:
    #     with RosBagReader([Path(bag_file)]) as reader:
    #         connections = [x for x in reader.connections if x.topic == '/rosout']
    #         for connection, timestamp, rawdata in reader.messages(connections=connections):
    #             msg = reader.deserialize(rawdata, connection.msgtype)
    #             msg_list.append(msg.msg)

    # # Return combined messages and the last .bag file
    # return msg_list, raw_folder #bag_files[-1]