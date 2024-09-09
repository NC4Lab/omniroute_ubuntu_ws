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

trial_summary_path = os.path.join(date_folder, 'Past_seven_days_biases.csv')

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
        trial_types_to_check = [2, 3, 4, 5]
    elif id_value == 3:
        trial_types_to_check = [6, 7, 8, 9]
    elif id_value == 5:
        trial_types_to_check = [10, 11, 12, 13]
    elif id_value == 7:
        trial_types_to_check = [14, 15, 16, 17]
    else:
        raise ValueError("Invalid ID value. It must be 1, 3, 5, or 7.")

   # Extract the values from the specified trial types
    subset_df = df[df['Trial Type'].isin(trial_types_to_check)]
    print(subset_df)
    values = subset_df['Error_Count'].to_numpy()
    print(values)
    
    # Normalize the values to create probabilities
    total = np.sum(values)
    probabilities = values / total
    
    # Choose one value based on probabilities
    selected_value = np.random.choice(values, size=1, p=probabilities)
    print(selected_value)
    selected_value_index = values.tolist().index(selected_value)
    print(selected_value_index)

    trial_type = subset_df['Trial Type'].iloc[selected_value_index]
    
    return trial_type


def generate_trial(id_value, df):
    start_chamber = find_start_chamber(id_value, df)
    trial = trial_types[start_chamber]

    return trial

trial = generate_trial(5, df)
print(f'Trial: {trial}')