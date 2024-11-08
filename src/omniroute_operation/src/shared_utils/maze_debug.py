#!/usr/bin/env python
# Class for printing and logging to ROS

from PyQt5.QtWidgets import QGraphicsView, QGraphicsTextItem, QGraphicsItemGroup, QGraphicsLineItem
from PyQt5.QtGui import QPen, QColor, QFont
from colorama import Fore, Style
import rospy
import inspect

# ======================== GLOBAL VARS ========================
DB_VERBOSE = False  # debug verbose flag

class MazeDB(QGraphicsView):
    """ MazeDebug class to plot the maze """

    @classmethod
    def printMsg(cls, level, msg, *args):
        """ Log to ROS in color """

        # Skip debug messages if DB_VERBOSE is false
        if not DB_VERBOSE and level == 'DEBUG':
            return
        
        # Append caller class and function to the message
        caller = inspect.stack()[1]
        caller_class = caller[0].f_locals['self'].__class__.__name__
        caller_func = caller[0].f_code.co_name
        msg = f"[{caller_class}::{caller_func}] {msg}"

        # Log message by type and color to ROS
        if level == 'ATTN':
            # Add '=' header and footer for ATTN condition
            f_msg = cls._frmt_msg(Fore.GREEN, msg, *args)
            n = max(0, 60 - len(f_msg) // 2)
            f_msg = '=' * n + " " + f_msg + " " + '=' * n
            rospy.loginfo(f_msg)
        elif level == 'INFO':
            rospy.loginfo(cls._frmt_msg(Fore.BLUE, msg, *args))
        elif level == 'ERROR':
            rospy.logerr(cls._frmt_msg(Fore.RED, msg, *args))
        elif level == 'WARNING':
            rospy.logwarn(cls._frmt_msg(Fore.YELLOW, msg, *args))
        elif level == 'DEBUG':
            rospy.loginfo(cls._frmt_msg(Fore.CYAN, msg, *args))
        else:
            rospy.loginfo(cls._frmt_msg(Fore.BLACK, msg, *args))

    @classmethod
    def _frmt_msg(cls, color, msg, *args):
        """ Format message with color """

        colored_message = f"{color}{msg}{Style.RESET_ALL}"
        return colored_message % args

    def arrStr(input_list):
        """Converts a list/array to a string"""

        result = "[" + ",".join(map(str, input_list)) + "]"
        return result