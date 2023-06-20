/** @file
 * @brief This files contains the definitions for all the functions used by the
 * esmacat_application class
 */

/*****************************************************************************************
 * INCLUDES
 ****************************************************************************************/
#include "application.h"

/*****************************************************************************************
 * FUNCTIONS
 ****************************************************************************************/
/** @brief Assigns the slave indices for each slave in the EtherCAT communication chain
 *
 * Also assigns their respective Vendor ID and Product code. Proivdes error handling
 * @param esmacat_slave* Pointer to the Esmacat slave application.
 * @param slave_index Position/index of the specified slave in the EtherCAT communication chain. Starts at 0.
 * @return Error status of the function. NO_ERR indicates successful completion.
 */

esmacat_err esmacat_application::assign_esmacat_slave_index(esmacat_slave* slave_in_app, int slave_index)
{
    ecat_master->ECAT_slave[slave_index] = slave_in_app;
    ethercat_product_id_list[slave_index] = slave_in_app->get_esmacat_product_id();
    ethercat_vendor_id_list[slave_index] = slave_in_app->get_esmacat_vendor_id();
    return NO_ERR;
}

/** @brief Constructor for the Esmacat application class which initializes all the variables
 *
 * Initializes a master. Sets the initial loop count to 0.
 * Initializes the Vendor ID and Product codes of the maximum possible slaves
 * to default
 */
esmacat_application::esmacat_application()
{
    ecat_master = new esmacat_master;
    loop_cnt = 0;

    for (int i=0;i<MAX_NUMBER_OF_ETHERCAT_SLAVE;i++)
    {
        ethercat_product_id_list[i] = DEFAULT_PRODUCT_VENDOR_ID;
        ethercat_vendor_id_list[i] = DEFAULT_PRODUCT_VENDOR_ID;
    }
}

/** @brief Checks if the EtherCAT master connection is closed.
 *
 * @return True if closed, False if still open
 */
bool esmacat_application::is_esmacat_master_closed()
{
    return ecat_master->is_ec_closed();
}

/** @brief Assigns this application as the application to be executed by the
 * EtherCAT master. Starts the thread on which this application is executed.
 */
void esmacat_application::start()
{
    ecat_master->assign_esmacat_application(this);
    ecat_master->StartInternalThread();
}

/** @brief Sets the given name to the EtherCAT adapter i.e. the name for the master port
 *
 * @param Pointer to the name of the EtherCAT adapter
 */
void esmacat_application::set_ethercat_adapter_name(char* eth_adapter_name)
{
    ecat_master->set_ethernet_adapter_name_for_esmacat(eth_adapter_name);
}

/** @brief Allows for the EtherCAT adapter name to be set via the terminal
 *
 * This function prints out the names of the various adapters that are on the
 * EtherCAT communication chain and their indices . It then accepts from the
 * user, the index of the adapter name that is to be assigned to this adapter.
 * This adapter name is set
 */
void esmacat_application::set_ethercat_adapter_name_through_terminal()
{
    // declare variables for ethercat
    ec_adaptert * adapter = NULL;
    ec_adaptert * adapter_selected = NULL;
    int adapter_index = 0;
    int selected_adapter_index = 0;
    int i=0;
    printf("---------------------------------------------- \n");
    printf("Esmacat Master Software\n");
    printf("---------------------------------------------- \n");
    printf ("List of available Ethernet adapters\n");
    adapter = adapter_selected = ec_find_adapters ();
    while (adapter != NULL)
    {
        printf ("%d: %s\n", adapter_index++, adapter->name);
        adapter = adapter->next;
    }
    printf("---------------------------------------------- \n");
    if (adapter_index == 0){
        printf ("There is no available adapter" );
    }
    else{
        printf ("Please select the adapter for EtherCAT: (0 - %d): ",adapter_index-1 );
        scanf("%d",&selected_adapter_index);
    }
    for (i=0;i<selected_adapter_index;i++) adapter_selected=adapter_selected->next;
    set_ethercat_adapter_name(adapter_selected->name);
}

/** @brief Terminates the thread of the application
 */
void esmacat_application::stop()
{
    ecat_master->stop_thread();
}

/** @brief Returns the esmacat_application loop counter that indicates how many
 * loops of the application have been run
 */
uint64_t esmacat_application::get_loop_cnt()
{
    return loop_cnt;
}

/** @brief Increments the esmacat_application loop counter that indicates how many
 * loops of the application have been run
 */
void esmacat_application::increment_loop_cnt()
{
    loop_cnt++;
}  // increment the loop cnt

/** @brief Virtual function to be overwritten by the user application my_app class*/
void esmacat_application::configure_slaves()
{
    // do something for initial setup
}

/** @brief Virtual function to be overwritten by the user application my_app class*/
void esmacat_application::set_elapsed_time_ms(double elapsed_time_ms_)
{
    elapsed_time_ms = elapsed_time_ms_;
}

/** @brief Virtual function to be overwritten by the user application my_app class*/
void esmacat_application::loop()
{
    // this is a base function, needs to be overwrode by a child class
}

/** @brief Virtual function to be overwritten by the user application my_app class*/
void esmacat_application::assign_slave_sequence()
{
    // this is a base function, needs to be overwrode by a child class
}

/** @brief Virtual function to be overwritten by the user application my_app class*/
void esmacat_application::init()
{
    // this is a base function, needs to be overwrode by a child class
}


uint32 esmacat_application::get_app_error_counter()
{
    return ecat_master->get_error_counter();
}
