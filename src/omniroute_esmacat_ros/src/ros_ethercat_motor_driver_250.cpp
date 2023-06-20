# include "ros/ros.h"   // Include the ROS header file to use ROS functionalities
# include "motordriver.h"   // Include the corresponding Esmacat library for the slave
# include "ros_ethercat_motor_driver_250.h"   // Include the corresponding ROS library for the slave
# include <boost/thread.hpp>   // To use multithreading

// void assign_colleague_sequence(int index)
// {
// //    assign_esmacat_slave_index(ecat_md,index);
    
//    return ; 
// }
// /**
// * @brief 
// *
// */

// void ros_ethercat_motor_driver_250::in_that(esmacat_motor_driver* ecat_md)
// {
//     ecat_md->set_position_control_pid_gain(3.05e-6, 0, 1.08e-4);
//     // // sets other important parameters for motor control
//     ecat_md->set_max_allowable_integrated_error_for_position_control(5e7);
//     ecat_md->set_max_velocity_in_position_control_qc_p_ms(1333 * 25);
//     // ecat_md.set_desired_position(898900);
//     std::cout<<"Is the in_that running?"<<std::endl;
// }

/**
 * @brief Function drives the motor by using Ecat_MD function and gives position provided
 *          through ROS write message
 * @return 
 * 
 */

void ros_ethercat_motor_driver_250::drive_it(esmacat_motor_driver* ecat_md){

    // boost::lock_guard<boost::mutex> lock(mtx_ease_drive_it);
    // boost::lock_guard<boost::mutex> lock2(mtx_ease_read);
    boost::lock_guard<boost::mutex> lock(mtx_ease_write);
    ecat_md->enable_escon(ros_write.escon_enable);
    
    if (ros_write.select == "position")
    {
        ecat_md->set_desired_position((int)ros_write.command);
        // std::cout<<"I am position"<<std::endl;
    }
    else if (ros_write.select == "speed")
    {
        // std::cout<<"I am Speed"<<std::endl;
        if (ros_write.command > 0.6) {
            ros_write.command = 0.6;
        }
        else if (ros_write.command < -0.6) {
            ros_write.command = -0.6;
        }
        ecat_md->set_escon_current_setpoint(ros_write.command);    
    }
    
    
    // ecat_md->set_desired_position(696900);
    
    sensorVal = ecat_md->get_digital_input(0);
    interim_encoder = ecat_md->get_encoder_counter();
    
   
    // std::cout<<"Is the drive_it running? -->"<<ros_write.command<<std::endl;
    // return ;
}
/**
 * @brief Function sets ROS read structs and uses a thread lock before setting any read values.
 *          ROS read ->  Read from motor
 * @return 
 * 
 */


void ros_ethercat_motor_driver_250::set_read(esmacat_motor_driver* ecat_md)
{
    boost::lock_guard<boost::mutex> lock(mtx_ease_write);
    interim_encoder = ecat_md->get_encoder_counter();
    interim_select = ros_write.select;
    interim_command = ros_write.command;
    interim_escon = ros_write.escon_enable;
    sensorVal = ecat_md->get_digital_input(0);

    boost::lock_guard<boost::mutex> lock2(mtx_ease_read);
    ros_read.encoder = interim_encoder;
    ros_read.command = interim_command;
    ros_read.sensor = sensorVal;
    // return ;
}


/**
 * @brief Function give ROS read structs and uses a thread lock before returning any read values.
 *          ROS read ->  Read from motor
 * @return Ros_ethercat_motor_driver_250 - Read
 * 
 */
ros_ethercat_motor_driver_250::read ros_ethercat_motor_driver_250::get_read()
{
    boost::lock_guard<boost::mutex> lock(mtx_ease_read);
    return ros_read;
}

/**
 * @brief Function sets ROS write structs and uses a thread lock before setting any write values.
 *          ROS Write ->  Write to motor
 * @return 
 * 
 */
void ros_ethercat_motor_driver_250::set_write(const ros_ethercat_motor_driver_250::write* msg){
    //apply boost lock
    boost::lock_guard<boost::mutex> lock(mtx_ease_write);
    ros_write.sensor = sensorVal;
    ros_write.encoder = interim_encoder;
    ros_write = *msg;
};

/**
 * @brief Function gives ROS write structs and uses a thread lock before giving any write values.
 *          ROS Write ->  Write to motor
 * @return 
 * 
 */
ros_ethercat_motor_driver_250::write ros_ethercat_motor_driver_250::get_write(){
    //apply boost lock
    boost::lock_guard<boost::mutex> lock(mtx_ease_write);

    return(ros_write);
}




/**
 * @brief Function definition for the ROS READ Thread which publish data at a specified rate
 * @return
 */
void ros_ethercat_motor_driver_250::ROS_read_esmacat_thread(){
    //Declare a message and setup the publisher for that message

    // Create an object similar to the message type used for the ROS Communication to write data
    omniroute_esmacat_ros::real_driver data_to_send;

    // Create a handle for the ROS Node that writes data
    ros::NodeHandle n_pub_ecat_read;

    // Specify the frequency that you would like to ROS publishing loop to run at
    ros::Rate loop_rate(100);

    // instantiate a publisher using the node defined and passing the <message_type>("Topic name",Buffer Size) as Input
    ros::Publisher pub_ecat_read = n_pub_ecat_read.advertise<omniroute_esmacat_ros::real_driver>("Esmacat_read_" + topic_name,1000);

    while (ros::ok()){

        // Update the temporary variables with the READ registers from EtherCAT Communication
        ros_ethercat_motor_driver_250::read interim_data = this->get_read();

        // Copy corresponding data from the temporary variable to publish
        data_to_send.encoder = interim_data.encoder;
        data_to_send.command = interim_data.command;
        data_to_send.sensor = sensorVal;
        data_to_send.escon_enable = interim_data.escon_enable;
        data_to_send.select = interim_data.select;
        // data_to_send.INT2 = interim_data.INT2;
        // data_to_send.INT3 = interim_data.INT3;
        // data_to_send.INT4 = interim_data.INT4;
        // data_to_send.INT5 = interim_data.INT5;
        // data_to_send.INT6 = interim_data.INT6;
        // data_to_send.INT7 = interim_data.INT7;

        //Send data to ROS nodes that are not in the hard real-time loop
        pub_ecat_read.publish(data_to_send);       
        // std::cout<<"Is the read thread running?"<<std::endl;

        // Turn off ROS communication as per the frequency specified
        loop_rate.sleep();
    }
}

/**
 * @brief Function definition for the callback function for the ROS write thread which updates the write shared memory with the
 *             data received from ROS Communication 
 * @return
 */
void ros_ethercat_motor_driver_250::Esmacat_write_Callback(const omniroute_esmacat_ros::real_driver::ConstPtr& msg)
{
    // Instantiate a write object from the ros_ethercat_motor_driver_250 class to receive ROS communication data 
    //     from other ROS nodes that will be used in the hard real-time loop
    ros_ethercat_motor_driver_250::write data_write_interim;

    // Copy received data from ROS communication to a temporary variable
    data_write_interim.encoder = msg->encoder;
    data_write_interim.command = msg->command;
    data_write_interim.sensor = msg->sensor;
    data_write_interim.escon_enable = msg->escon_enable;
    data_write_interim.select = msg->select;
    // data_write_interim.INT2 = msg->INT2;
    // data_write_interim.INT3 = msg->INT3;
    // data_write_interim.INT4 = msg->INT4;
    // data_write_interim.INT5 = msg->INT5;
    // data_write_interim.INT6 = msg->INT6;
    // data_write_interim.INT7 = msg->INT7;
    // std::cout<< msg->command << " <-New messages"<< std::endl;

    // Update the write shared memory from the temporary variable 
    this->set_write(&data_write_interim);
}

/**
 * @brief Function definition for the ROS write thread
 * @return
 */
void ros_ethercat_motor_driver_250::ROS_write_esmacat_thread(){
    
    //Setup a subscriber that will get data from other ROS nodes
    ros::MultiThreadedSpinner spinner(0);
    // std::cout<<"Is the write thread running?"<<std::endl;

    // Create a handle for the ROS Node that receives the write data
    ros::NodeHandle n_sub_ecat_write;

    // instantiate a subscriber using the node defined and passing the ("Topic name",Buffer Size, callback function, self object) as Input
    ros::Subscriber sub_ecat_write = n_sub_ecat_write.subscribe("Esmacat_write_" + topic_name, 1000, &ros_ethercat_motor_driver_250::Esmacat_write_Callback, this);

    spinner.spin(); //blocking spin call for this thread
}


