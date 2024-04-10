/** @file
 * @brief This file contains the definition of the functions associated with the user-defined
 * application for the EtherCAT Arduino Shield by Esmacat slave example project */
/*****************************************************************************************
 * INCLUDES
 ****************************************************************************************/
#include "ros_esmacat_app.h"
// #include "ros/ros.h"
// #include "std_msgs/float.h"
// int32_t encoder;
// _Float64 degree;
// //set position setpoint based on the encoder datasheet
// long divider = 2048*81.37*4*3 // encoder counts = 2048*4 per revolution and the rest is gear ratios
// ros::NodeHandle n;
// ros::Publisher chatter_pub =n.advertise<std_msgs::float>("boomAngle",1)
// std_msgs::float msg;

/*****************************************************************************************
 *
 * FUNCTIONS
 ****************************************************************************************/

/**
 * @brief Identifies the actual Esmacat slave sequence in the EtherCAT communication chain.
 */
void ros_esmacat_app::assign_slave_sequence()
{
    // Assign the syncing arduino slave object to the actual slave in the EtherCAT communication chain
    int sync_ease_slave_index;                                                  // Slave index in the EtherCAT communication chain
    ros::param::param<int>("/sync_ease_slave_index", sync_ease_slave_index, 0); // Use the ROS parameter when available
    assign_esmacat_slave_index(&sync_ease_ecat_as, sync_ease_slave_index);      // Assign the slave object to a position in the chain

    // Assign the maze wall controller slave object to the actual slave in the EtherCAT communication chain
    int maze_ard0_ease_slave_index;
    ros::param::param<int>("/maze_ard0_ease_slave_index", maze_ard0_ease_slave_index, 1);
    assign_esmacat_slave_index(&maze_ard0_ease_ecat_as, maze_ard0_ease_slave_index);

    // Assign the feeder servos slave object to the actual slave in the EtherCAT communication chain
    int feeder_ease_slave_index;
    ros::param::param<int>("/feeder_servos_slave_index", feeder_ease_slave_index, 2);
    assign_esmacat_slave_index(&feeder_ease_ease_ecat_as, feeder_servos_slave_index);
}

/**
 * @brief Configure your Esmacat slave.
 * Link Esmacat slave object with the actual Esmacat slave in the EtherCAT communication chain.
 * Functions beginning with 'configure slave' must only be executed in this function
 */
void ros_esmacat_app::configure_slaves()
{
    // add initialization code here
    // Functions starting with "configure_slave" work only in configure_slave() function
    // lines below are from esmacat_master_software and is to setup I/O for hall sensor
    // dont need the lines below unless needed to run hall sensor
    // motor_driver_ease.equator(&ecat_md);
    // motor_driver_ease.configure_me(&ecat_md);
    // ecat_md.configure_slave_encoder_clear();
    // IO_Direction dio_config[7] = {IO_INPUT, IO_INPUT, IO_INPUT, IO_INPUT, IO_INPUT, IO_INPUT, IO_INPUT};
    // ecat_md.configure_slave_dio_direction(dio_config);
}

/** @brief Initialization that needs to happen on the first iteration of the loop
 */
void ros_esmacat_app::init()
{
    // set gains for the proportional, derivative and integral gains for the control loop
    //  ecat_md.set_position_control_pid_gain(3.05e-6, 0, 1.08e-4);
    //  // // // sets other important parameters for motor control
    //  ecat_md.set_max_velocity_in_position_control_qc_p_ms(1333 * 25);
    //  ecat_md.set_max_allowable_integrated_error_for_position_control(5e7);
    //  ecat_md.set_desired_position(898900);
    //  motor_driver_ease.in_that(&ecat_md);
    //  std::cout<<"Is the init running?"<<std::endl;
}

/**
 * @brief Executes functions at the defined loop rate
 */
void ros_esmacat_app::loop()
{

    //............... Synchronize Esmacat Hardware with ROS Communication for Sync Ease ...............

    // The newly modified write registers from the ROS Communication is returned from the shared memory
    sync_ease_ros_message = sync_ease.get_write_registers();

    // The Esmacat slave object is used to update the corresponding Esmacat slave registers with the
    // updated values received from ROS Nodes.
    sync_ease_ecat_as.set_output_variable_0_OUT_GEN_INT0(sync_ease_ros_message.INT0);

    //............... Synchronize Esmacat Hardware with ROS Communication for Maze ARD0 ...............

    // The ROS object created is used to read the current state of registers from the Esmacat slave object
    // and store it in a shared memory location
    maze_ard0_ease.set_read_registers(&maze_ard0_ease_ecat_as);

    // The newly modified write registers from the ROS Communication is returned from the shared memory
    maze_ard0_ease_ros_message = maze_ard0_ease.get_write_registers();

    // The Esmacat slave object is used to update the corresponding Esmacat slave registers with the
    // updated values received from ROS Nodes.
    maze_ard0_ease_ecat_as.set_output_variable_0_OUT_GEN_INT0(maze_ard0_ease_ros_message.INT0);
    maze_ard0_ease_ecat_as.set_output_variable_1_OUT_GEN_INT1(maze_ard0_ease_ros_message.INT1);
    maze_ard0_ease_ecat_as.set_output_variable_2_OUT_GEN_INT2(maze_ard0_ease_ros_message.INT2);
    maze_ard0_ease_ecat_as.set_output_variable_3_OUT_GEN_INT3(maze_ard0_ease_ros_message.INT3);
    maze_ard0_ease_ecat_as.set_output_variable_4_OUT_GEN_INT4(maze_ard0_ease_ros_message.INT4);
    maze_ard0_ease_ecat_as.set_output_variable_5_OUT_GEN_INT5(maze_ard0_ease_ros_message.INT5);
    maze_ard0_ease_ecat_as.set_output_variable_6_OUT_GEN_INT6(maze_ard0_ease_ros_message.INT6);
    maze_ard0_ease_ecat_as.set_output_variable_7_OUT_GEN_INT7(maze_ard0_ease_ros_message.INT7);

    //............... Synchronize Esmacat Hardware with ROS Communication for Feeder Servo ...............

    // The ROS object created is used to read the current state of registers from the Esmacat slave object
    // and store it in a shared memory location
    feeder_ease_ease.set_read_registers(&feeder_ease_ease_ecat_as);

    // The newly modified write registers from the ROS Communication is returned from the shared memory
    feeder_ease_ease_ros_message = feeder_ease_ease.get_write_registers();

    // The Esmacat slave object is used to update the corresponding Esmacat slave registers with the
    // updated values received from ROS Nodes.
    feeder_ease_ease_ecat_as.set_output_variable_0_OUT_GEN_INT0(feeder_ease_ease_ros_message.INT0);

    //     // // The updated value is logged onto the terminal
    // cout <<"Sync EASE Register 0 = " << feeder_ease_ease_ros_message.INT0 << std :: endl;
}