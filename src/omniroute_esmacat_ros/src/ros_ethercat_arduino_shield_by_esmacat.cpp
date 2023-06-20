/** @file
 * @brief This file contains the function defintions of the class associated with the ROS Related
 * source code to interface the EtherCAT Arduino Shield by Esmacat slave*/

/*****************************************************************************************
 * INCLUDES
 ****************************************************************************************/
# include "ros/ros.h"   // Include the ROS header file to use ROS functionalities
# include "ethercat_arduino_shield_by_esmacat.h"   // Include the corresponding Esmacat library for the slave
# include "ros_ethercat_arduino_shield_by_esmacat.h"   // Include the corresponding ROS library for the slave
# include <boost/thread.hpp>   // To use multithreading

/*****************************************************************************************
 * FUNCTIONS
 ****************************************************************************************/

/**
 * @brief This function updates the shared memory with the current EASE registers from the EtherCAT communication
 *        The values are copied to a temporary variable and updated after loacking the shared memory for update  
 *              only by the read thead
 * @return
 */
void ros_ethercat_arduino_shield_by_esmacat::set_read_registers(esmacat_ethercat_arduino_shield_by_esmacat* ease)
{   
    // Temporary variables to store data from the EASE registers     
    interim_register_value_0 = ease->get_input_variable_0_IN_GEN_INT0();
    interim_register_value_1 = ease->get_input_variable_1_IN_GEN_INT1();
    interim_register_value_2 = ease->get_input_variable_2_IN_GEN_INT2();
    interim_register_value_3 = ease->get_input_variable_3_IN_GEN_INT3();
    interim_register_value_4 = ease->get_input_variable_4_IN_GEN_INT4();
    interim_register_value_5 = ease->get_input_variable_5_IN_GEN_INT5();
    interim_register_value_6 = ease->get_input_variable_6_IN_GEN_INT6();
    interim_register_value_7 = ease->get_input_variable_7_IN_GEN_INT7();

    //apply boost lock
    boost::lock_guard<boost::mutex> lock(mtx_ease_read);
    
    // The READ shared variable is updated with the corresponding values from the temporary variable
    ros_read_registers.INT0 = interim_register_value_0;
    ros_read_registers.INT1 = interim_register_value_1;
    ros_read_registers.INT2 = interim_register_value_2;
    ros_read_registers.INT3 = interim_register_value_3;
    ros_read_registers.INT4 = interim_register_value_4;
    ros_read_registers.INT5 = interim_register_value_5;
    ros_read_registers.INT6 = interim_register_value_6;
    ros_read_registers.INT7 = interim_register_value_7;
}

/**
 * @brief This function returns the current data from the READ shared memory
 * @return
 */
ros_ethercat_arduino_shield_by_esmacat::read ros_ethercat_arduino_shield_by_esmacat::get_read_registers(){ 
    //apply boost lock
    boost::lock_guard<boost::mutex> lock(mtx_ease_read);
    return(ros_read_registers);
}

/**
 * @brief This function updates the shared memory with the values from the ROS communication
 *        The values are updated from the data received from ROS coomunication
 *              after loacking the shared memory for update only by the write thead
 * @return
 */
void ros_ethercat_arduino_shield_by_esmacat::set_write_registers(const ros_ethercat_arduino_shield_by_esmacat::write* msg){
    //apply boost lock
    boost::lock_guard<boost::mutex> lock(mtx_ease_write);
    ros_write_registers = *msg;
};

/**
 * @brief This function returns the current data from the WRITE shared memory
 * @return
 */
ros_ethercat_arduino_shield_by_esmacat::write ros_ethercat_arduino_shield_by_esmacat::get_write_registers(){
    //apply boost lock
    boost::lock_guard<boost::mutex> lock(mtx_ease_write);
    return(ros_write_registers);
}

/**
 * @brief Function definition for the ROS READ Thread which publish data at a specified rate
 * @return
 */
void ros_ethercat_arduino_shield_by_esmacat::ROS_read_esmacat_thread(){
    //Declare a message and setup the publisher for that message

    // Create an object similar to the message type used for the ROS Communication to write data
    omniroute_esmacat_ros::ease_registers data_to_send;

    // Create a handle for the ROS Node that writes data
    ros::NodeHandle n_pub_ecat_read;

    // Specify the frequency that you would like to ROS publishing loop to run at
    ros::Rate loop_rate(100);

    // instantiate a publisher using the node defined and passing the <message_type>("Topic name",Buffer Size) as Input
    ros::Publisher pub_ecat_read = n_pub_ecat_read.advertise<omniroute_esmacat_ros::ease_registers>("Esmacat_read_" + topic_name,1000);

    while (ros::ok()){

        // Update the temporary variables with the READ registers from EtherCAT Communication
        ros_ethercat_arduino_shield_by_esmacat::read interim_data = this->get_read_registers();

        // Copy corresponding data from the temporary variable to publish
        data_to_send.INT0 = interim_data.INT0;
        data_to_send.INT1 = interim_data.INT1;
        data_to_send.INT2 = interim_data.INT2;
        data_to_send.INT3 = interim_data.INT3;
        data_to_send.INT4 = interim_data.INT4;
        data_to_send.INT5 = interim_data.INT5;
        data_to_send.INT6 = interim_data.INT6;
        data_to_send.INT7 = interim_data.INT7;

        //Send data to ROS nodes that are not in the hard real-time loop
        pub_ecat_read.publish(data_to_send);       

        // Turn off ROS communication as per the frequency specified
        loop_rate.sleep();
    }
}

/**
 * @brief Function definition for the callback function for the ROS write thread which updates the write shared memory with the
 *             data received from ROS Communication 
 * @return
 */
void ros_ethercat_arduino_shield_by_esmacat::Esmacat_write_Callback(const omniroute_esmacat_ros::ease_registers::ConstPtr& msg)
{
    // Instantiate a write object from the ros_ethercat_arduino_shield_by_esmacat class to receive ROS communication data 
    //     from other ROS nodes that will be used in the hard real-time loop
    ros_ethercat_arduino_shield_by_esmacat::write data_write_interim;

    // Copy received data from ROS communication to a temporary variable
    data_write_interim.INT0 = msg->INT0;
    data_write_interim.INT1 = msg->INT1;
    data_write_interim.INT2 = msg->INT2;
    data_write_interim.INT3 = msg->INT3;
    data_write_interim.INT4 = msg->INT4;
    data_write_interim.INT5 = msg->INT5;
    data_write_interim.INT6 = msg->INT6;
    data_write_interim.INT7 = msg->INT7;

    // Update the write shared memory from the temporary variable 
    this->set_write_registers(&data_write_interim);
}

/**
 * @brief Function definition for the ROS write thread
 * @return
 */
void ros_ethercat_arduino_shield_by_esmacat::ROS_write_esmacat_threader(){
    
    //Setup a subscriber that will get data from other ROS nodes
    ros::MultiThreadedSpinner spinner(0);

    // Create a handle for the ROS Node that receives the write data
    ros::NodeHandle n_sub_ecat_write;

    // instantiate a subscriber using the node defined and passing the ("Topic name",Buffer Size, callback function, self object) as Input
    ros::Subscriber sub_ecat_write = n_sub_ecat_write.subscribe("Esmacat_write_" + topic_name, 1000, &ros_ethercat_arduino_shield_by_esmacat::Esmacat_write_Callback, this);

    spinner.spin(); //blocking spin call for this thread
}
