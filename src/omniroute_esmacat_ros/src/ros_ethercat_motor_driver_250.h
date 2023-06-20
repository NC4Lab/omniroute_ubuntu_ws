/** @file
 * @brief This file contains the declaration of the class associated with the ROS Related
 * source code to interface the EtherCAT Arduino Shield by Esmacat slave*/

# ifndef ROS_ETHERCAT_MOTOR_DRIVER_250
# define ROS_ETHERCAT_MOTOR_DRIVER_250

/*****************************************************************************************
 * INCLUDES
 ****************************************************************************************/
# include "ros/ros.h"   // Include the ROS header file to use ROS functionalities
# include "motordriver.h"    // Include the corresponding Esmacat library for the slave
# include <boost/thread.hpp>   // To use multithreading
# include "omniroute_esmacat_ros/real_driver.h"   // To use the generated header files from the ROS Message


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
class ros_ethercat_motor_driver_250
{
public:
	ros_ethercat_motor_driver_250(std::string slave_name) : topic_name(std::move(slave_name)),
	interim_encoder(-1),interim_command(-1),ros_read(), ros_write(), interim_select(""), interim_escon(-1),
	ROS_read_thread(boost::thread(&ros_ethercat_motor_driver_250::ROS_read_esmacat_thread,this)),
	ROS_write_thread(boost::thread(&ros_ethercat_motor_driver_250::ROS_write_esmacat_thread,this))
	{

		std ::cout << "ROS EASE object instantiated" << std ::endl;
	}

	// destructor for the class
	~ros_ethercat_motor_driver_250()
	{
		std ::cout << "Joining the ROS EASE threads" << std ::endl;
		ROS_read_thread.join();
		ROS_write_thread.join();
	}
	// The read struct to store the read data
	struct read
	{
		// Since EASE has 8 registers, the struct contains 8 integer values
		int64_t encoder;
		float_t command;
		int32_t sensor;
		int32_t escon_enable;
		std::string select;
		// int16_t INT
		// int16_t INT3;
		// int16_t INT4;
		// int16_t INT5;
		// int16_t INT6;
		// int16_t INT7;

		// Constructor initialised with -1 (junk value) initially
		read() : encoder(-1), command(-1), sensor(-1), escon_enable(0), select("position")
			//  INT2(-1), INT3(-1), INT4(-1),
			//  INT5(-1), INT6(-1), INT7(-1)
		{}
	};

	// The write struct to store the write data
	struct write
	{
		// Since EASE has 8 registers, the struct contains 8 integer values
		int64_t encoder;
		float_t command;
		int32_t sensor;
		int32_t escon_enable;
		std::string select; 
		// int16_t INT2;
		// int16_t INT3;
		// int16_t INT4;
		// int16_t INT5;
		// int16_t INT6;
		// int16_t INT7;

		// Constructor initialised with -1 (junk value) initially
		write() : encoder(-1), command(-1), sensor(-1), escon_enable(0), select("position")
			//   INT2(-1), INT3(-1), INT4(-1),
			//   INT5(-1), INT6(-1), INT7(-1)
		{
		}
	};

	void drive_it(esmacat_motor_driver* ecat_md);
	
	// boost::thread ROS_trieber_thread;	// Boost threads instantiation for the write thread
	// mutable boost::mutex mtx_ease_drive_it; // Mutex lock for write thread
	
	// void configure_me(esmacat_motor_driver *ecat_md);
	// void in_that(esmacat_motor_driver *ecat_md);
	
	void set_read(esmacat_motor_driver* ecat_md);
	
	read get_read();
	
	void set_write(const ros_ethercat_motor_driver_250::write* msg);
	
	write get_write();

	// Call back function for the write data thread
	void Esmacat_write_Callback(const omniroute_esmacat_ros::real_driver::ConstPtr &msg);

	// Write data thread function
	void ROS_write_esmacat_thread();
	
	// Read data thread function
	void ROS_read_esmacat_thread();

	// esmacat_motor_driver ecat_md;
	// int32_t command;
	// int32_t encoder;
	int32_t interim_escon;
	std::string interim_select;
	int32_t interim_command;
	int64_t interim_encoder;
	int32_t sensorVal;

	// void equator(esmacat_motor_driver* object);

private:
	
	const std::string topic_name;

	read ros_read;   // instantiate an object for read struct 
	write ros_write;   // instantiate an object for write struct

	boost::thread ROS_read_thread;   // Boost threads instantiation for the read thread
	boost::thread ROS_write_thread;   // Boost threads instantiation for the write thread

	mutable boost::mutex mtx_ease_read;   // Mutex lock for read thread
	mutable boost::mutex mtx_ease_write;   // Mutex lock for write thread
};
# endif // ROS_ETHERCAT_ARDUINO_SHIELD_BY_ESMACAT