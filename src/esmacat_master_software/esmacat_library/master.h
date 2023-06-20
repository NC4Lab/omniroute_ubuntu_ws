/** @file
 * @brief This file contains the declaration for the esmacat master
 */

#ifndef ESMACAT_MASTER_H
#define ESMACAT_MASTER_H


/*****************************************************************************************
 * MACROS
 ****************************************************************************************/
#define MY_PRIORITY (49) /* we use 49 as the PRREMPT_RT use 50
                            as the priority of kernel tasklets
                            and interrupt handler by default */

#define MAX_SAFE_STACK (8*1024) /* The maximum stack size which is
                                   guaranteed safe to access without
                                   faulting */

/** @brief The number of nano-seconds in a sec is 10^9 */
#define NSEC_PER_SEC    (1000000000) /* The number of nsecs per sec. */
/** @brief The number of micro-seconds in a sec is 10^6 */
#define USEC_PER_SEC    (1000000)
/** @brief Timeout value in us*/
#define EC_TIMEOUTMON 500

/** @brief The time period/cycle time of the esmacat application is set to 25000 us = 25 ms */
/** Please change this setting to modify the rate at which the application operates */
#define ESMACAT_TIME_PERIOD_US 25000

/** @brief Maximum Number of Ethercat slaves supported by the master */
#define MAX_NUMBER_OF_ETHERCAT_SLAVE 100

/*****************************************************************************************
 * INCLUDES
 ****************************************************************************************/
#include <stdio.h>
#include <string.h>
#include <inttypes.h>
#include <vector>
#include "ethercat.h"
#include "application.h"
#include "slave.h"

// start of RT include
//#include <sys/mman.h>

/*****************************************************************************************
 * CLASSES
 ****************************************************************************************/
class esmacat_application;


/** @brief Holds all the functions and variables used by the ethercat master */
class esmacat_master
{

private:
    /** @brief Holds the PDO map for CANOpen for all the slaves*/
    char IOmap[4096];   // IO map for EtherCAT, used by SOEM
    /** @brief Expected value of the working counter (part of EtherCAT protocol)- used by SOEM */
    int expectedWKC;    // part of SOEM
    boolean needlf;     // part of SOEM
    /** @brief Value of the work counter - used by SOEM */
    volatile int wkc;   // part of SOEM
    /** @brief Indicates that the master is operational and communicating with the slaves - used by SOEM*/
    boolean inOP;       // part of SOEM
    /** @brief Group of slaves - used by SOEM */
    uint8 currentgroup; // part of SOEM
    /** @brief Indicates that the master connection is closed */
    bool ec_closed;     // part of SOEM
    /** @brief Flag to indicate that the thread must be stopped */
    bool stop_thread_loop;  // part of SOEM
    /** @brief Pointer to the user-defined ethercat application to be used */
    esmacat_application* ECAT_app; // esmacat application will be copied to here and be used in threads
    /** @brief Pointer to the thread for the ethercat process data communication loop */
    OSAL_THREAD_HANDLE _thread_esmacat_master_loop;
    /** @brief Pointer to the thread for the ethercat status check */
    OSAL_THREAD_HANDLE _thread_ecatcheck;
    /** @brief Name of the Ethercat adapter */
    char ifname[MAX_NUMBER_OF_ETHERCAT_SLAVE];   // ethernet adapter name
    /** @brief this is an intermediate function to use pthread */
    static void * InternalThreadEntry_esmacat_master_loop(void * This) {((esmacat_master *)This)->esmacat_master_loop(); return NULL;}
    /** @brief this is an intermediate function to use pthread */
    static void * InternalThreadEntry_ecatcheck(void * This) {((esmacat_master *)This)->ecatcheck(); return NULL;}

    uint32 error_counter;

public:
    esmacat_slave* ECAT_slave[MAX_NUMBER_OF_ETHERCAT_SLAVE];    // array of esmacat slaves

    esmacat_master();
    ~esmacat_master();
    // start the threads for esmacat master, Returns true if the thread was successfully started, false if there was an error starting the thread
    bool StartInternalThread();
     // close the threads. Call this function when you finish the program.
    void WaitForInternalThreadToExit();
    void esmacat_master_loop();                         // main RT loop
    void ecatcheck();                                   // check the status of EtherCAT
    bool is_ec_closed();
    void assign_esmacat_application(esmacat_application* esmacat_app){ECAT_app=esmacat_app;}
    void stop_thread();                                         // stop threads of esmacat master
    void set_ethernet_adapter_name_for_esmacat(char* eth_adapter_name);
    uint32 get_error_counter();
    //void CALLBACK RTthread(UINT uTimerID, UINT uMsg, DWORD_PTR dwUser, DWORD_PTR dw1, DWORD_PTR dw2);
};

#endif // ESMACAT_MASTER_H
