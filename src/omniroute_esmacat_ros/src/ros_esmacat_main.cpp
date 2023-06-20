/** @file
 *  @brief This file contains the main node for the EtherCAT Arduino Shield by Esmacat slave
 * example project */
/*****************************************************************************************
 * INCLUDES
 ****************************************************************************************/
# include <iostream>
# include "ros_esmacat_app.h"
# include "ros/ros.h"
# include <string>

/*****************************************************************************************
 * FUNCTIONS
 ****************************************************************************************/
/**
 * @brief Initializes the execution of the Ethercat communication and
 *        primary real-time loop for your application for the desired
 *        slave
 * @return
 */

int main(int argc, char **argv)
{
    //this is defined in ros_esmacat_app.cpp and ros_esmacat_app.h
    static plog::RollingFileAppender<plog::CsvFormatter> fileAppender("esmacat_log.csv", 80000, 10); // Create the 1st appender.
    static plog::ColorConsoleAppender<plog::TxtFormatter> consoleAppender; // Create the 2nd appender.

    plog::init(plog::warning, &fileAppender).addAppender(&consoleAppender); // Initialize the logger with the both appenders.

    // Initialize the ROS Esmacat Node
    ros::init(argc, argv, "Esmacat_node");
    ros::NodeHandle nh;

    ros_esmacat_app app;

    std::string adapter_name;
    if (ros::param::has("/ethercat_adapter_name")) {
        ros::param::get("/ethercat_adapter_name",adapter_name);
        app.set_ethercat_adapter_name((char*)adapter_name.c_str());
    }
    else {
        // If the name is not known, select through the terminal an ethernet adapter (the slave)
        // you'd like to communicate with over EtherCAT
        // adapter_name = "enp7s0";
        app.set_ethercat_adapter_name_through_terminal();
    }
    // app.set_ethercat_adapter_name((char*)adapter_name.c_str());

    // start the esmacat application customized for your slave
    app.start();

    while (app.is_esmacat_master_closed() == false) {
        if(!ros::ok()) {
            app.stop();
        }
    }
    
    // ros::MultiThreadedSpinner spinner(2); // Use 4 threads
    // spinner.spin(); // spin() will not return until the node has been shutdown

    ros::shutdown();   //Shutdown ROS after the EsmaCAT application is complete.    

    return 0;
}
