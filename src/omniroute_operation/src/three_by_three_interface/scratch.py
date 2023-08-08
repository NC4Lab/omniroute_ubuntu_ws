# CLASS: Interface plugin
class Interface(Plugin):

    # Define signals
    signal_Esmacat_read_maze_ard0_ease = Signal()

    def __init__(self, context):
        super(Interface, self).__init__(context)

        self._joint_sub = None

        # Give QObjects reasonable names
        self.setObjectName('Interface')
        rospyLogCol('INFO', "Running Interface setup")

        # Process standalone plugin command-line arguments
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
        # Extend the widget with all attributes and children from UI file
        loadUi(os.path.join(os.path.dirname(
            os.path.realpath(__file__)), 'interface.ui'), self._widget)

        self._widget.setObjectName('InterfacePluginUi')
        # Show _widget.windowTitle on left-top of each plugin (when
        # it's set in _widget). This is useful when you open multiple
        # plugins at once. Also if you open multiple instances of your
        # plugin at once, these lines add number to make it easy to
        # tell from pane to pane.
        if context.serial_number() > 1:
            self._widget.setWindowTitle(
                self._widget.windowTitle() + (' (%d)' % context.serial_number()))

        # Add widget to the user interface
        context.add_widget(self._widget)
        self._widget.plotMazeView.setViewportUpdateMode(
            QGraphicsView.FullViewportUpdate)
        self.scene = QGraphicsScene()
        self._widget.plotMazeView.setScene(self.scene)

        # Set the fixed size of the main window based on the dimensions from the UI file
        main_window_width = self._widget.geometry().width()
        main_window_height = self._widget.geometry().height()
        self._widget.setFixedSize(main_window_width, main_window_height)

        # Set the size hint of the main window to match the size of the _widget
        self._widget.window().setMinimumSize(main_window_width, main_window_height)
        self._widget.window().setMaximumSize(main_window_width, main_window_height)

        # Set the background color of the scene to white
        self._widget.plotMazeView.setBackgroundBrush(QColor(255, 255, 255))
        self._widget.plotMazeView.setViewport(QtOpenGL.QGLWidget())

        # Calculate chamber width and wall line width and offset
        maze_view_size = self._widget.plotMazeView.width()
        chamber_width = self._widget.plotMazeView.width()*0.9/NUM_ROWS_COLS
        wall_width = chamber_width*0.1

        # Create Maze_Plot.Maze and populate walls according to WALL_MAP
        self.maze = Maze_Plot.Maze(num_rows=NUM_ROWS_COLS,
                                  num_cols=NUM_ROWS_COLS,
                                  chamber_width=chamber_width,
                                  wall_width=wall_width)

        # Add chambers and disable walls not connected
        for k, c in enumerate(self.maze.chambers):
            self.scene.addItem(c)
            for j, w in enumerate(c.walls):
                if j not in WALL_MAP[k]:
                    w.setEnabled(False)
                    w.setVisible(False)

        # Add walls - this is a new loop so that they are drawn above the chambers
        for c in self.maze.chambers:
            for w in c.walls:
                self.scene.addItem(w)

        # Initialize file list text and index
        self.current_file_index = 0  # set to zero
        self._widget.fileDirEdit.setText(
            self.getPathConfigDir())  # set to default path

        # Initialize ardListWidget with arduino names.
        # Add 8 arduinos to the list labeled Arduino 0-9
        for i in range(8):
            self._widget.ardListWidget.addItem("Arduino " + str(i))
        # Hide the blue selection bar
        self._widget.ardListWidget.setStyleSheet(
            "QListWidget::item { border-bottom: 1px solid black; }")
        # Change the text color for all items in the list to light gray
        self._widget.ardListWidget.setStyleSheet(
            "QListWidget { color: lightgray; }")

        # QT UI object callback setup
        self._widget.fileListWidget.itemClicked.connect(
            self.qt_callback_fileListWidget_clicked)
        self._widget.fileBrowseBtn.clicked.connect(
            self.qt_callback_fileBrowseBtn_clicked)
        self._widget.filePreviousBtn.clicked.connect(
            self.qt_callback_filePreviousBtn_clicked)
        self._widget.fileNextBtn.clicked.connect(
            self.qt_callback_fileNextBtn_clicked)
        self._widget.plotClearBtn.clicked.connect(
            self.qt_callback_plotClearBtn_clicked)
        self._widget.plotSaveBtn.clicked.connect(
            self.qt_callback_plotSaveBtn_clicked)
        self._widget.plotSendBtn.clicked.connect(
            self.qt_callback_plotSendBtn_clicked)
        self._widget.sysQuiteBtn.clicked.connect(
            self.qt_callback_sysQuiteBtn_clicked)

        # QT timer setup for UI updating
        self.timer_updateUI = QTimer()
        self.timer_updateUI.timeout.connect(self.timer_callback_updateUI_loop)
        self.timer_updateUI.start(20)  # set incriment (ms)

        # QT timer for sending initial handshake message after a delay
        self.timer_sendHandshake = QTimer()
        self.timer_sendHandshake.timeout.connect(
            self.timer_callback_sendHandshake_once)
        self.timer_sendHandshake.setSingleShot(True)  # Run only once
        self.timer_sendHandshake.start(1000)  # (ms)

        # QT timer for checking initial handshake message after a delay
        self.timer_checkHandshake = QTimer()
        self.timer_checkHandshake.timeout.connect(
            self.timer_callback_checkHandshake_once)
        self.timer_checkHandshake.setSingleShot(True)  # Run only once

        # ROS publisher
        # wall_clicked_pub = rospy.Publisher('/wall_state', WallState, queue_size=1)
        self.maze_ard0_pub = rospy.Publisher(
            '/Esmacat_write_maze_ard0_ease', ease_registers, queue_size=1)  # Esmacat write maze ard0 ease

        # ROS subscriber
        rospy.Subscriber('Esmacat_read_maze_ard0_ease', ease_registers,
                         self.ros_callback_Esmacat_read_maze_ard0_ease, tcp_nodelay=True)

        # Signal callback setup
        self.signal_Esmacat_read_maze_ard0_ease.connect(
            self.sig_callback_Esmacat_read_maze_ard0_ease)

    """ CALLBACKS: ROS, QT and Signal """

    def ros_callback_Esmacat_read_maze_ard0_ease(self, msg):
        # Store ethercat message in class variable
        si16_arr = [0]*8
        si16_arr[0] = msg.INT0
        si16_arr[1] = msg.INT1
        si16_arr[2] = msg.INT2
        si16_arr[3] = msg.INT3
        si16_arr[4] = msg.INT4
        si16_arr[5] = msg.INT5
        si16_arr[6] = msg.INT6
        si16_arr[7] = msg.INT7

        # Convert 16 bit signed ints to 16 bit unsigned ints array
        ui16_arr = [0]*8
        for i in range(8):
            ui16_arr[i] = ctypes.c_uint16(si16_arr[i]).value

        # Parse the message and check if its new
        if Ecat_Handler.getEthercatMessage(ui16_arr) != 1:
            return

        # Handle confirmation message
        if Ecat_Handler.rcvEM.msgTp == MessageType.CONFIRMATION:
            # Check if confirmation message is handshake
            if Ecat_Handler.cnfEM.msgTp == MessageType.HANDSHAKE:

                # Stop handshake check timer
                self.timer_checkHandshake.stop()

                # Set the handshake flag
                Ecat_Handler.isHandshakeDone = True

                # Send START_SESSION message
                Ecat_Handler.sendEthercatMessage(MessageType.START_SESSION)

        # Emit signal to update UI as UI should not be updated from a non-main thread
        # TEMP self.signal_Esmacat_read_maze_ard0_ease.emit()

    def sig_callback_Esmacat_read_maze_ard0_ease(self):
        pass

    def timer_callback_sendHandshake_once(self):
        # Send HANDSHAKE message to arduino
        Ecat_Handler.sendEthercatMessage(MessageType.HANDSHAKE)

        # Start timer to check for HANDSHAKE message confirm recieved
        self.timer_checkHandshake.start(500)

    def timer_callback_checkHandshake_once(self):
        # Check for HANDSHAKE message confirm recieved flag
        if Ecat_Handler.isHandshakeDone == False:

            # Give up is more that 3 messages have been sent
            if Ecat_Handler.sndEM.msgID > 3:
                rospyLogCol(
                    'ERROR', "!!ERROR: Handshake failed: msg_id=%d!!", Ecat_Handler.sndEM.msgID)
                return

            # Send HANDSHAKE message to arduino again
            Ecat_Handler.sendEthercatMessage(MessageType.HANDSHAKE)

            # Restart check timer
            self.timer_checkHandshake.start(500)
        else:
            # Send START message to arduino
            Ecat_Handler.sendEthercatMessage(MessageType.START_SESSION)

    def timer_callback_updateUI_loop(self):
        # Update graphics
        self.scene.update()
        self._widget.plotMazeView.update()

    def qt_callback_fileBrowseBtn_clicked(self):
        # Filter only CSV files
        filter = "CSV Files (*.csv)"
        files, _ = QFileDialog.getOpenFileNames(
            None, "Select files to add", self.getPathConfigDir(), filter)

        if files:
            # Clear the list widget to remove any previous selections
            self._widget.fileListWidget.clear()

            # Extract the file names from the full paths and store them in self.files
            self.files = [os.path.basename(file) for file in files]

            # Add the selected file names to the list widget
            self._widget.fileListWidget.addItems(self.files)

            # Enable the "Next" and "Previous" buttons if there is more than one file
            if len(self.files) > 1:
                self._widget.fileNextBtn.setEnabled(True)
                self._widget.filePreviousBtn.setEnabled(True)
            else:
                self._widget.fileNextBtn.setEnabled(False)
                self._widget.filePreviousBtn.setEnabled(False)

            # Update file index and load csv
            self.loadFromCSV(0)

    def qt_callback_fileListWidget_clicked(self, item):
        # Get the index of the clicked item and set it as the current file index
        self.current_file_index = self._widget.fileListWidget.currentRow()

        # Update file index and load csv
        self.loadFromCSV(0)

    def qt_callback_fileNextBtn_clicked(self):
        # Update file index and load csv
        self.loadFromCSV(1)

    def qt_callback_filePreviousBtn_clicked(self):
        # Update file index and load csv
        self.loadFromCSV(-1)

    def qt_callback_plotClearBtn_clicked(self):
        Wall_Config.Reset()  # reset all values in list
        self.maze.updateWalls()  # update walls

    def qt_callback_plotSaveBtn_clicked(self):
        # Open the folder specified by self.getPathConfigDir() in an explorer window
        folder_path = self.getPathConfigDir()
        options = QFileDialog.Options()
        options |= QFileDialog.DontUseNativeDialog
        file_name, _ = QFileDialog.getSaveFileName(
            None, "Save CSV File", folder_path, "CSV Files (*.csv);;All Files (*)", options=options)

        if file_name:
            # Ensure the file name ends with ".csv"
            if not file_name.endswith(".csv"):
                file_name += ".csv"

            # The user has specified a file name, you can perform additional actions here
            rospyLogCol('INFO', "Selected file:", file_name)

            # Call the function to save wall config data to the CSV file with the wall array values converted to bytes
            self.saveToCSV(file_name, Wall_Config.makeWall2ByteList())

    def qt_callback_plotSendBtn_clicked(self):
        # Sort entries
        Wall_Config.sortEntries()

        # Send the wall byte array to the arduino
        Ecat_Handler.sendEthercatMessage(
            MessageType.MOVE_WALLS, 9, Wall_Config.getWall2ByteList())

    def qt_callback_sysQuiteBtn_clicked(self):
        # Call function to shut down the ROS session
        self.endRosSession()
        # End the application
        QApplication.quit()

    ''' FUNCTIONS: CSV File Handling '''

    def getPathConfigDir(self, file_name=None):
        # Get the absolute path of the current script file
        script_dir = os.path.dirname(os.path.abspath(__file__))
        # Create the path to the "config" directory four levels up
        dir_path = os.path.abspath(os.path.join(
            script_dir, '..', '..', '..', '..', 'config', 'paths'))
        # Return file or dir path
        if file_name is not None:
            return os.path.join(dir_path, file_name)
        else:
            return dir_path

    def saveToCSV(self, file_name, wall_config_list):
        try:
            with open(file_name, 'w', newline='') as csvfile:
                csv_writer = csv.writer(csvfile)
                for row in wall_config_list:
                    csv_writer.writerow(row)
            rospyLogCol('INFO', "Data saved to:", file_name)
        except Exception as e:
            rospyLogCol('ERROR', "Error saving data to CSV:", str(e))

    def loadFromCSV(self, list_increment):
        # Update the current file index
        self.current_file_index += list_increment

        # Loop back to the end or start if start or end reached, respectively
        if list_increment < 0 and self.current_file_index < 0:
            self.current_file_index = len(self.files) - 1  # set to end
        elif list_increment > 0 and self.current_file_index >= len(self.files):
            self.current_file_index = 0  # set to start

        # Set the current file in the list widget
        self._widget.fileListWidget.setCurrentRow(self.current_file_index)

        # Get the currently selected file path
        file_name = self.files[self.current_file_index]
        folder_path = self.getPathConfigDir()
        file_path = os.path.join(folder_path, file_name)

        # Load and store CSV data
        try:
            with open(file_path, 'r') as csv_file:
                csv_reader = csv.reader(csv_file)
                wall_byte_config_list = [
                    [int(row[0]), int(row[1])] for row in csv_reader]
                Wall_Config.makeByte2WallList(wall_byte_config_list)
                rospyLogCol('INFO', "Data loaded successfully.")
        except Exception as e:
            rospyLogCol('ERROR', "Error loading data from CSV:", str(e))

        # Update plot walls
        self.maze.updateWalls()

    ''' FUNCTIONS: System Operations '''

    def terminate_ros_node(self, s):
        list_cmd = subprocess.Popen(
            "rosnode list", shell=True, stdout=subprocess.PIPE)
        list_output = list_cmd.stdout.read()
        retcode = list_cmd.wait()
        assert retcode == 0, "List command returned %d" % retcode
        for str in list_output.decode().split("\n"):
            if (str.startswith(s)):
                os.system("rosnode kill " + str)

    def terminate_process_and_children(self, p):
        ps_command = subprocess.Popen(
            "ps -o pid --ppid %d --noheaders" % p.pid, shell=True, stdout=subprocess.PIPE)
        ps_output = ps_command.stdout.read()
        retcode = ps_command.wait()
        assert retcode == 0, "ps command returned %d" % retcode
        for pid_str in ps_output.split("\n")[:-1]:
            os.kill(int(pid_str), signal.SIGINT)
        p.terminate()
        p.kill()

    def endRosSession(self):

        # Send END_SESSION message to arduino
        Ecat_Handler.sendEthercatMessage(MessageType.END_SESSION)

        # Kill self.signal_Esmacat_read_maze_ard0_ease.emit() thread
        # TEMP self.thread_Esmacat_read_maze_ard0_ease.terminate()

        # Kill specific nodes
        self.terminate_ros_node("/Esmacat_application_node")
        self.terminate_ros_node("/interface_test_node")

        # Wait for nodes to shutdown
        time.sleep(1)  # (sec)

        # Kill all nodes (This will also kill this script's node)
        os.system("rosnode kill -a")

        # Process any pending events in the event loop
        QCoreApplication.processEvents()

        # Close the UI window
        self._widget.close()

        # Send a shutdown request to the ROS master
        rospy.signal_shutdown("User requested shutdown")

    # NOT WORKING!!
    def closeEvent(self, event):
        rospyLogCol('INFO', "Closing window...")
        # Call function to shut down the ROS session
        self.endRosSession()
        event.accept()  # let the window close
