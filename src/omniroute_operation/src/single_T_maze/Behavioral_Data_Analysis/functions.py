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
import math
import re

config = dotenv_values()
data_path = os.path.normpath(config['DATA_PATH'])
os.environ['DATA_PATH'] = data_path
print(f"DATA_PATH is set to: {os.environ['DATA_PATH']}")

trial_types = {
#Trial Types: ['Start Chamber', 'Left Cue', 'Right Cue', 'Floor Cue']
    1: ['5', 'Triangle', 'No_Cue', 'Black'],
    2: ['5', 'No_Cue', 'Triangle', 'Black'],
    3: ['5', 'Triangle', 'No_Cue', 'Green'],
    4: ['5', 'No_Cue', 'Triangle', 'Green'],
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

    for i, msg in enumerate(messages):
        # Check if the message starts with 'START OF TRIAL'
        if msg.startswith('START OF TRIAL'):
            # Extract the list part from the message using regular expressions
            list_part = re.search(r"\[(.*?)\]", msg)
            if list_part:
                list_str = list_part.group(0)  # Get the list string (with square brackets)
                # Convert list string to a Python list, handling 'nan' values as float('nan')
                try:
                    # Remove 'nan' and keep the original structure
                    trial_list = eval(list_str, {"nan": float("nan")})
                    filtered_list = [x for x in trial_list if not (isinstance(x, float) and math.isnan(x))]
                    messages[i] = f"START OF TRIAL {filtered_list}"
                except Exception as e:
                    print(f"Error processing: {msg} - {e}")

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
    return [msg for msg in messages if msg not in ('ERROR_END', 'ERROR_START')]
    #return [msg for msg in messages if msg != 'ERROR_END']

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
        if len(trial_list) >= 4:
            item4 = trial_list[3]
            return item1, item2, item3, item4
        else:
            return item1, item2, item3
    
    except (IndexError, ValueError, SyntaxError) as e:
        # Handle cases where the string is not as expected
        print(f"Error processing: {trial_str} - {e}")
        return None, None, None
    
# Function to determine trial type
def determine_trial_type(row):
    # Extract relevant columns
    trial_info = [row['Start Chamber Number'], row['Left Cue'], row['Right Cue'], row['Floor Cue']]
    
    # Loop through trial types and find a match
    for trial_type, expected_values in trial_types.items():
        if trial_info == expected_values:
            return trial_type
        else:
            return None  # In case no match is found

#function to delte the trials in which the training mode was forced choice.
def remove_rows_with_value(df, column_name, value_to_remove):
    # Create a new dataframe without rows where the value in the specified column matches 'value_to_remove'
    filtered_df = df[df[column_name] != value_to_remove]
    return filtered_df

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
    
    # Get the previous three folders including the current one
    start_index = max(current_index - 2, 0)
    return all_folders[start_index:current_index+1]

def combine_csv_files(rat, date, path=os.environ['DATA_PATH']):
    folders = get_previous_folders(rat, date, path=os.environ['DATA_PATH'])
    if isinstance(rat, str):
        rat = int(rat)

    #look in the specific animal folder based on the number designated to each animal 
    anim_folder = os.path.join(path, 'NC4%04d' % rat)

    combined_df = None
    daily_success_counts = []
    daily_total_reps = []
    
    for folder in folders:
        csv_path = os.path.join(anim_folder, folder, 'trial_type_summary.csv')
        if os.path.exists(csv_path):
            df = pd.read_csv(csv_path)
             # Track daily success counts and total repetitions for the day
            daily_success_counts.append(df['Success_Count'].sum())
            daily_total_reps.append(df['Total_Repetitions'].sum())
            if combined_df is None:
                combined_df = df.copy()
            else:
                combined_df['Total_Repetitions'] += df['Total_Repetitions']
                combined_df['Error_Count'] += df['Error_Count']
                combined_df['Success_Count'] += df['Success_Count']
    
    # Calculate Error and Success counts as proportions
    combined_df['Error_Count'] = combined_df['Error_Count'] / combined_df['Total_Repetitions']
    combined_df['Success_Count'] = combined_df['Success_Count'] / combined_df['Total_Repetitions']

    overall_success = {
        f'Overall_Success_Day_{i+1}': [daily_success_counts[i] / daily_total_reps[i]]
        for i in range(len(daily_success_counts))
    }
    overall_success_df = pd.DataFrame(overall_success)
    
    # Save the combined DataFrame to a new CSV file
    output_path = os.path.join(anim_folder, date, 'Past_three_days_biases.csv')
    output_path_overall = os.path.join(anim_folder, date, 'Overall_Success.csv')
    combined_df.to_csv(output_path, index=False)
    overall_success_df.to_csv(output_path_overall, index=False)
    print(f"Combined CSV saved to {output_path}")


