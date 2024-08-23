#!/usr/bin/env python
import os,time
import rospy
import numpy as np
import math
import subprocess
import random
from std_msgs.msg import String, Int32, Int8
from geometry_msgs.msg import PoseStamped, PointStamped
from omniroute_operation.msg import *
from omniroute_esmacat_ros.msg import *
from omniroute_controller.interface import MazeDimensions
from dateutil.parser import parse as parsedate

import pandas as pd

trial_dir = '/media/big_gulp/nc4_rat_data/Maze_Rats'

rat = 6
date = '240821'

rat_folder = os.path.join(trial_dir, 'NC4%04d' % rat)

if '-' in date: 
    date = parsedate(date).strftime('%y%m%d')

date_folder = os.path.join(rat_folder, date)

trial_summary_path = os.path.join(date_folder, 'Past_three_days_biases.csv')

df = pd.read_csv(trial_summary_path)

trial_types = {
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

trial_count = {key: 0 for key in trial_types}

def find_start_chamber(id_value, df):
    if id_value == 1:
        trial_types_to_check = [1, 2, 3, 4]
    elif id_value == 3:
        trial_types_to_check = [5, 6, 7, 8]
    elif id_value == 5:
        trial_types_to_check = [9, 10, 11, 12]
    elif id_value == 7:
        trial_types_to_check = [13, 14, 15, 16]
    else:
        raise ValueError("Invalid ID value. It must be 1, 3, 5, or 7.")

    # Filter the DataFrame for the relevant trial types
    section = df[df['Trial Type'].isin(trial_types_to_check)]

    # Find the row with the maximum value in column 'Error_Count'
    max_row = section.loc[section['Error_Count'].idxmax()]

    # Get the trial type from the selected row
    trial_type = max_row['Trial Type']

    # Check if this trial type has been selected 8 times
    if trial_count[trial_type] < 8:
        # If not selected 8 times, return this trial type
        return trial_type
    else:
        # If selected 8 times, find the next trial type with the highest 'Error_Count'
        section = section.drop(section[section['Trial Type'] == trial_type].index)
        if not section.empty:
            next_max_row = section.loc[section['Error_Count'].idxmax()]
            return


# def find_start_chamber(id_value, df):
#     if id_value == 1:
#         start_row, end_row = 1, 4  # Rows 1 to 4 (index 0 to 3)
#     elif id_value == 3:
#         start_row, end_row = 5, 8  # Rows 5 to 8 (index 4 to 7)
#     elif id_value == 5:
#         start_row, end_row = 9, 12 # Rows 9 to 12 (index 8 to 11)
#     elif id_value == 7:
#         start_row, end_row = 13, 16 # Rows 13 to 16 (index 12 to 15)
#     else:
#         raise ValueError("Invalid ID value. It must be 1, 3, 5, or 7.")

#     # Select the specific section of rows
#     section = df.iloc[start_row:end_row]

#     # Find the row with the maximum value in column 'Error_Count'
#     max_row = section.loc[section['Error_Count'].idxmax()]
#     print(f"max_row: {max_row}")

#     # Get the trial type from the selected row
#     trial_type = max_row['Trial Type']
#     print(f"trial_type: {trial_type}")

#     # Check if this trial type has been selected 8 times
    
#     if trial_count[trial_type] < 8:
#         # If not selected 8 times, return this trial type
#         return trial_type
#     else:
#         # If selected 8 times, find the next trial type with the highest 'Error_Count'
#         section = section.drop(section[section['Trial Type'] == trial_type].index)
#         if not section.empty:
#             next_max_row = section.loc[section['Error_Count'].idxmax()]
#             return next_max_row['Trial Type']
#         else:
#             raise ValueError("No alternative trial type available.")

def generate_trial(id_value, df):
    start_chamber = find_start_chamber(id_value, df)
    trial = trial_types[start_chamber]

    # Update the count for the selected trial type
    trial_count[start_chamber] += 1

    return trial

trial = generate_trial(5, df)
print(trial)