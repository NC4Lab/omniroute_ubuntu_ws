import signal

class MyApp:
    def __init__(self):
        # Initialize the Qt application and widget
        self.app = QApplication([])
        self._widget = QMainWindow()
        self._ui = Ui_MainWindow()
        self._ui.setupUi(self._widget)

        # Connect the signal to the appropriate method
        self._ui.sysQuiteBtn.clicked.connect(self.qt_callback_sysQuiteBtn_clicked)

        # Handle SIGINT signal to gracefully shut down the application
        signal.signal(signal.SIGINT, self.handle_sigint)

    def handle_sigint(self, signum, frame):
        # Call function to shut down the ROS session
        self.end_ros_session()
        # End the application
        QApplication.quit()

    def end_ros_session(self):
        # Your code to terminate the ROS nodes and clean up goes here
        # ...

    def run(self):
        self._widget.show()
        self.app.exec_()


import signal

# ... (Other imports and code)

class Interface(Plugin):
    # ... (Other class functions and variables)

    def __init__(self, context):
        # ... (Other initialization code)

        # Set up a signal handler for SIGINT (Ctrl+C)
        signal.signal(signal.SIGINT, self.signal_handler)

        # ... (Remaining initialization code)

    def signal_handler(self, signal, frame):
        # Call function to shut down the ROS session
        self.end_ros_session()
        # Close the UI window
        self._widget.close()
        # End the application
        QApplication.quit()
        
    # ... (Other class functions)

# ... (Other functions)
