/** @file
 * @brief This file contains the main program for the Esmacat Demo Kit example project */
/*****************************************************************************************
 * INCLUDES
 ****************************************************************************************/
#include <iostream>
#include "my_app.h"

using namespace std;

/*****************************************************************************************
 * MAIN FUNCTION
 ****************************************************************************************/
/**
 * @brief Initializes the execution of the Ethercat communication and
 *        primary real-time loop for your application
 * @return
 */

int main()
{
    //this is defined in my_app.cpp and my_app.h
    my_app app;

    // if you already know the ethernet adapter name for EtherCAT, uncomment and use the line below
    //    app.set_ethercat_adapter_name(" WRITE YOUR ETHERNET ADAPTER NAME");

    // If the name is not known, select through the terminal an ethernet adapter (the slave)
    // you'd like to communicate with over EtherCAT
    app.set_ethercat_adapter_name_through_terminal();

    // start the esmacat application customized for your slave
    app.start();

    //the application runs as long as the esmacat master and slave are in communication
    while (app.is_esmacat_master_closed() == FALSE );

    return 0;
}

