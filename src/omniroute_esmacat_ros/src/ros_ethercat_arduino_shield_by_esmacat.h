/** @file
 * @brief This file contains the declaration of the class associated with the ROS Related
 * source code to interface the EtherCAT Arduino Shield by Esmacat slave*/

# ifndef ROS_ETHERCAT_ARDUINO_SHIELD_BY_ESMACAT
# define ROS_ETHERCAT_ARDUINO_SHIELD_BY_ESMACAT

/*****************************************************************************************
 * INCLUDES
 ****************************************************************************************/
# include "ros/ros.h"   // Include the ROS header file to use ROS functionalities
# include "ethercat_arduino_shield_by_esmacat.h"    // Include the corresponding Esmacat library for the slave
# include <boost/thread.hpp>   // To use multithreading
# include "omniroute_esmacat_ros/ease_registers.h"   // To use the generated header files from the ROS Message

/*****************************************************************************************
 * CLASSES
 ****************************************************************************************/
/**
 * @brief Description of the ROS Esmacat slave class
 *
 * This class contains the source code for the ROS Esmacat slave class used to communicate with Esmacat Nodes
 *     1) Two separate struct to read and write data are defined in the public class along with their constructor
 *     2) Functions associated with the read & write thread
 * 
 * The private scope contains
 *     1) Two separate objects to read and write data from the structs defined in the public scope
 *     2) Two separate boost threads to read and write data 
 *     3) Two mutex locks for the threads
 *     4) Temporary variables to store data to prevent data override  
 */
class ros_ethercat_arduino_shield_by_esmacat
{
    public:
    // Constructor for the class
    ros_ethercat_arduino_shield_by_esmacat(std::string slave_name): topic_name(std::move(slave_name)),
        interim_register_value_0(-1), interim_register_value_1(-1), 
        interim_register_value_2(-1), interim_register_value_3(-1),
        interim_register_value_4(-1), interim_register_value_5(-1), interim_register_value_6(-1),
        interim_register_value_7(-1), ros_read_registers(), ros_write_registers(),
        ROS_read_thread(boost::thread(&ros_ethercat_arduino_shield_by_esmacat::ROS_read_esmacat_thread, this)),
        ROS_write_thread(boost::thread(&ros_ethercat_arduino_shield_by_esmacat::ROS_write_esmacat_threader, this))
        {
            std :: cout << "ROS EASE object instantiated" << std :: endl;
        }
    
    // destructor for the class
    ~ros_ethercat_arduino_shield_by_esmacat()
    {
        std :: cout << "Joining the ROS EASE threads" << std :: endl;
        ROS_read_thread.join();
        ROS_write_thread.join();
    }

    // The read struct to store the read data
    struct read
    {
        // Since EASE has 8 registers, the struct contains 8 integer values
        int16_t INT0;
        int16_t INT1;
        int16_t INT2;
        int16_t INT3;
        int16_t INT4;
        int16_t INT5;
        int16_t INT6;
        int16_t INT7;

        // Constructor initialised with -1 (junk value) initially
        read(): INT0(-1),INT1(-1),
            INT2(-1),INT3(-1),INT4(-1),
            INT5(-1),INT6(-1),INT7(-1)
            {}

    };

    // The write struct to store the write data
    struct write
    {
        // Since EASE has 8 registers, the struct contains 8 integer values
        int16_t INT0;
        int16_t INT1;
        int16_t INT2;
        int16_t INT3;
        int16_t INT4;
        int16_t INT5;
        int16_t INT6;
        int16_t INT7;

        // Constructor initialised with -1 (junk value) initially
        write() : INT0(-1), INT1(-1),
            INT2(-1), INT3(-1), INT4(-1),
            INT5(-1), INT6(-1), INT7(-1)
            {}
    };

    // Set read registers function updates the shared memory with the current EASE Slave registers value from EtherCAT Communication
    void set_read_registers(esmacat_ethercat_arduino_shield_by_esmacat* ease);
    // Get read registers function returns the value from the read shared memory
    read get_read_registers();

    // Set write registers function updates the shared memory with the message received from ROS communication
    void set_write_registers(const ros_ethercat_arduino_shield_by_esmacat::write* input_message);
    // Get write registers function returns the value from the write shared memory
    write get_write_registers();

    // Read data thread function
    void ROS_read_esmacat_thread();

    // Call back function for the write data thread
    void Esmacat_write_Callback(const omniroute_esmacat_ros::ease_registers::ConstPtr& msg);

    // Write data thread function
    void ROS_write_esmacat_threader();

    private:

    const std::string topic_name;

    // Temporary variables to store data to prevent data override  
    int16_t interim_register_value_0;
    int16_t interim_register_value_1;
    int16_t interim_register_value_2;
    int16_t interim_register_value_3;
    int16_t interim_register_value_4;
    int16_t interim_register_value_5;
    int16_t interim_register_value_6;
    int16_t interim_register_value_7;

    read ros_read_registers;   // instantiate an object for read struct 
    write ros_write_registers;   // instantiate an object for write struct

    boost::thread ROS_read_thread;   // Boost threads instantiation for the read thread
    boost::thread ROS_write_thread;   // Boost threads instantiation for the write thread

    mutable boost::mutex mtx_ease_read;   // Mutex lock for read thread
    mutable boost::mutex mtx_ease_write;   // Mutex lock for write thread
};

# endif // ROS_ETHERCAT_ARDUINO_SHIELD_BY_ESMACAT
