import os,sys,time
import roslib
import rospy
import math

from qt_gui.plugin import Plugin
from python_qt_binding.QtCore import *
from python_qt_binding.QtWidgets import *
from python_qt_binding.QtGui import *

from python_qt_binding import loadUi

# import qwt

from std_msgs.msg import *

class Interface(Plugin):
    """Main GUI Class
    
    Defines functions which provide slots for Qt Signals, as well as callbacks for ROS messages to which the GUI subscribes. Available signals from the GUI rqt_dome_interface.ui is used for interface elements. Communication between ROS callbacks and UI elements is done through custom signals.
    """

    """Custom signals"""
    # encoderAngleChanged = Signal(float)

    def __init__(self, context):
        super(Interface, self).__init__(context)

        self._joint_sub = None

        ## Give QObjects reasonable names
        self.setObjectName('Interface')

        ## Process standalone plugin command-line arguments
        from argparse import ArgumentParser
        parser = ArgumentParser()
        # Add argument(s) to the parser.
        parser.add_argument("-q", "--quiet", action="store_true",
                dest="quiet",
                help="Put plugin in silent mode")
        args, unknowns = parser.parse_known_args(context.argv())
        if not args.quiet:
            print('arguments: ', args)
            print('unknowns: ', unknowns)

        # Create QWidget
        self._widget = QWidget()
        # Get path to UI file which is a sibling of this file
        # in this example the .ui and .py file are in the same folder
        ui_file = os.path.join(os.path.dirname(os.path.realpath(__file__)), 'interface.ui')
        # Extend the widget with all attributes and children from UI file
        loadUi(ui_file, self._widget)
        # Give QObjects reasonable names

        rospy.loginfo('Test Interface started')

        self._widget.setObjectName('InterfacePluginUi')
        # Show _widget.windowTitle on left-top of each plugin (when 
        # it's set in _widget). This is useful when you open multiple 
        # plugins at once. Also if you open multiple instances of your 
        # plugin at once, these lines add number to make it easy to 
        # tell from pane to pane.
        if context.serial_number() > 1:
            self._widget.setWindowTitle(self._widget.windowTitle() + (' (%d)' % context.serial_number()))

        # Add widget to the user interface
        context.add_widget(self._widget)

        # Subscriber instances
        # rospy.Subscriber("encoder_angle", Angle, self.ros_encoderCallback)

        # Publisher instances
        # self.quit_pub = rospy.Publisher('quit',DomeEvent,queue_size=1);        

        # Action clients
        # self.feed_client = actionlib.SimpleActionClient('feed', PulseDigitalAction)

        # Signal connections
        # self._widget.xmlLoadBtn.clicked[bool].connect(self._handle_xmlLoadBtn_clicked)
        # self._widget.intensitySlider.valueChanged[int].connect(self._handle_intensitySlider_valueChanged)

        # Custom signal connections
        # self.encoderAngleChanged[float].connect(self._handle_encoderAngleChanged)

        # Default texts
        #self._widget.bagDirEdit.setText(os.path.expanduser(os.path.join('~','experiment','bags')))
        
        # Qt callback handling functions
        # Any GUI updates MUST be made from these callbacks, and not from any other 
        # functions or especially ROS callbacks

    def _handle_intensitySlider_valueChanged(self, intensity):
        pass
        # self._widget.intensityValueEdit.setText("%d"%intensity)

        # reg = [0]*8
        # reg[0] = intensity
        # self.intensity_pub.publish(*reg)
