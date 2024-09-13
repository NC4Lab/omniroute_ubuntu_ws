#!/usr/bin/env python
import bagpy
from bagpy import bagreader
import os
import pandas as pd
from dateutil.parser import parse as parsedate
import matplotlib.pyplot as plt
from rosbags.highlevel import AnyReader as RosBagReader
from pathlib import Path
import logging
from dotenv import dotenv_values
from dotenv import load_dotenv
import ast 

config = dotenv_values()
data_path = os.path.normpath(config['DATA_PATH'])
os.environ['DATA_PATH'] = data_path
print(f"DATA_PATH is set to: {os.environ['DATA_PATH']}")

trial_types = {
#Trial Types: ['Start Chamber', 'Left Cue', 'Right Cue', 'Sound Cue']
    1: ['1', 'Triangle', 'No_Cue', 'White_Noise'],
    2: ['1', 'No_Cue', 'Triangle', '5KHz'],
    3: ['1', 'Triangle', 'No_Cue', '5KHz'],
    4: ['1', 'No_Cue', 'Triangle', 'White_Noise'],
    5: ['3', 'Triangle', 'No_Cue', 'White_Noise'],
    6: ['3', 'No_Cue', 'Triangle', '5KHz'],
    7: ['3', 'Triangle', 'No_Cue', '5KHz'],
    8: ['3', 'No_Cue', 'Triangle', 'White_Noise'],
    9: ['5', 'Triangle', 'No_Cue', 'White_Noise'],
    10: ['5', 'No_Cue', 'Triangle', '5KHz'],
    11: ['5', 'Triangle', 'No_Cue', '5KHz'],
    12: ['5', 'No_Cue', 'Triangle', 'White_Noise'],
    13: ['7', 'Triangle', 'No_Cue', 'White_Noise'],
    14: ['7', 'No_Cue', 'Triangle', '5KHz'],
    15: ['7', 'Triangle', 'No_Cue', '5KHz'],
    16: ['7', 'No_Cue', 'Triangle', 'White_Noise']
} 
    
def determine_trial_info(rat, date, path=os.environ['DATA_PATH']):
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

    print(f"Looking in folder: {date_folder}")
    
    bag_files = []  # To store paths of all bag files found

    # Collect all .bag files in the folder
    for file in os.listdir(date_folder):
        if file.endswith('.bag'):
            bag_files.append(os.path.join(date_folder, file))

    if not bag_files:
        raise FileNotFoundError("No .bag files found in the specified folder.")
    
    msg_list = []
    # Read messages from all .bag files found
    for bag_file in bag_files:
        with RosBagReader([Path(bag_file)]) as reader:
            connections = [x for x in reader.connections if x.topic == '/rosout']
            for connection, timestamp, rawdata in reader.messages(connections=connections):
                msg = reader.deserialize(rawdata, connection.msgtype)
                msg_list.append(msg.msg)

    # Return combined messages and the last .bag file
    return msg_list, bag_files[-1]

def filter_start_of_trial_messages(messages):
    keywords = ['Chamber', 'Current trial number', 'START OF TRIAL', 'SUCCESS', 'ERROR', 'Right', 'Left']
    
    # Filter messages that start with any of the keywords
    messages = [msg for msg in messages if any(msg.startswith(keyword) for keyword in keywords)]
    
    # Check if the second message starts with 'Current trial number'
    if not messages[1].startswith('Current trial number'):
        # Find the index of the message that starts with 'Current trial number'
        index = messages.index(next(msg for msg in messages if msg.startswith('Current trial number')))
        
        # Delete all messages before this index
        for i in range(index-1):
            del messages[0]  # Always delete the first item until we reach the correct index
            
    return messages

def remove_specific_message(messages):
    return [msg for msg in messages if msg != 'ERROR_END']

def create_dataframe_from_messages(messages):
    # Calculate the number of rows needed
    num_rows = len(messages) // 5 + (1 if len(messages) % 5 != 0 else 0)
    
    columns =['Number', 'Start Chamber', 'Trial Number', 'Trial', 'Result', 'Choice']

    # Initialize an empty DataFrame
    df = pd.DataFrame(columns=columns)
    
    # Populate the DataFrame with messages
    for i in range(num_rows):
        start_idx = i * 5
        end_idx = start_idx + 5
        row_data = messages[start_idx:end_idx]
        
        # Fill the remaining columns with empty strings if less than 6 messages in the last row
        while len(row_data) < 5:
            row_data.append('')
        
        df.loc[i] = [i + 1] + row_data
    
    return df

def extract_trial_info(trial_str):
    try:
        # Check if the string contains 'None' after 'START OF TRIAL'
        if 'START OF TRIAL None' in trial_str:
            return None, None, None

        # Extract the list from the string
        list_str = trial_str.split('START OF TRIAL ')[1]
        
        # Convert the string representation of the list to an actual list
        trial_list = ast.literal_eval(list_str)
        
        # Ensure the extracted list has at least three elements
        if len(trial_list) < 3:
            return None, None, None  # Or handle it differently based on your needs
        
        # Extract the specific items (first three items)
        item1 = trial_list[0]
        item2 = trial_list[1]
        item3 = trial_list[2]
        
        return item1, item2, item3
    
    except (IndexError, ValueError, SyntaxError) as e:
        # Handle cases where the string is not as expected
        print(f"Error processing: {trial_str} - {e}")
        return None, None, None
    
# Function to determine trial type
def determine_trial_type(row):
    # Extract relevant columns
    trial_info = [row['Start Chamber Number'], row['Left Cue'], row['Right Cue'], row['Sound Cue']]
    
    # Loop through trial types and find a match
    for trial_type, expected_values in trial_types.items():
        if trial_info == expected_values:
            return trial_type
    return None  # In case no match is found

def get_previous_folders(rat, date, path=os.environ['DATA_PATH']):
    if isinstance(rat, str):
        rat = int(rat)

    #look in the specific animal folder based on the number designated to each animal 
    anim_folder = os.path.join(path, 'NC4%04d' % rat)

    # Get list of all folders
    all_folders = sorted([folder for folder in os.listdir(anim_folder) if folder.isdigit()])
    
    # Find index of current_date folder
    try:
        current_index = all_folders.index(date)
    except ValueError:
        raise ValueError(f"The given date {date} does not exist in the folder list.")
    
    # Get the previous seven folders including the current one
    start_index = max(current_index - 7, 0)
    return all_folders[start_index:current_index+1]

def combine_csv_files(rat, date, path=os.environ['DATA_PATH']):
    folders = get_previous_folders(rat, date, path=os.environ['DATA_PATH'])
    if isinstance(rat, str):
        rat = int(rat)

    #look in the specific animal folder based on the number designated to each animal 
    anim_folder = os.path.join(path, 'NC4%04d' % rat)

    combined_df = None
    
    for folder in folders:
        csv_path = os.path.join(anim_folder, folder, 'trial_type_summary.csv')
        if os.path.exists(csv_path):
            df = pd.read_csv(csv_path)
            if combined_df is None:
                combined_df = df.copy()
            else:
                combined_df['Total_Repetitions'] += df['Total_Repetitions']
                combined_df['Error_Count'] += df['Error_Count']
                combined_df['Success_Count'] += df['Success_Count']
    
    # Calculate Error and Success counts as proportions
    combined_df['Error_Count'] = combined_df['Error_Count'] / combined_df['Total_Repetitions']
    combined_df['Success_Count'] = combined_df['Success_Count'] / combined_df['Total_Repetitions']
    
    # Save the combined DataFrame to a new CSV file
    output_path = os.path.join(anim_folder, date, 'Past_seven_days_biases.csv')
    combined_df.to_csv(output_path, index=False)
    print(f"Combined CSV saved to {output_path}")