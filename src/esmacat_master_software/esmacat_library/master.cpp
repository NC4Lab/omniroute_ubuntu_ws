/** @file
 * @brief This file defines all the functions used by the Esmacat master class which works
 * for any ethercat master
 */

/*****************************************************************************************
 * INCLUDES
 ****************************************************************************************/
#include "master.h"


/*****************************************************************************************
 * FUNCTIONS
 ****************************************************************************************/
// start of RT function
/** @brief Internal function used to assign stack memory and to ensure that page faults
 *  are not generated during use*/
void stack_prefault(void)
{
    unsigned char dummy[MAX_SAFE_STACK];
    memset(dummy, 0, MAX_SAFE_STACK);
    return;
}
// end of RT function


/** @brief Constructor for the esmacat_master class
 *
 * Resets the values of currentgroup, shows the master connection to
 * be active, and the threads running
 */
esmacat_master::esmacat_master()
{
    currentgroup = 0;
    ec_closed = 0;
    stop_thread_loop = 0;

    error_counter =0;

}

/** @brief Destructor for the esmacat_master class
 *
 * Ends the threads on which the applications are running
 */
esmacat_master::~esmacat_master()
{
    WaitForInternalThreadToExit();  // join the threads.
}

/** @brief Stops the thread on which the master application is running
 *
 * Also sets the stop_thread_loop flag to 1 which flags for the thread to
 * be terminated
 */
void esmacat_master::stop_thread()
{
    stop_thread_loop = 1;
    WaitForInternalThreadToExit();
}

/** @brief Starts the threads that will be running in parallel
 *
 * One of the threads checks the status of the EtherCAT to ensure that all slaves are
 * operational. The other threads runs the master loop that ensures constant communication
 * and real-time exchange of data with the slaves
 *
 * @return Returns 0 if successful, and 1 if there is an error
 */
bool esmacat_master::StartInternalThread()
{
    int ret1, ret2;
    // create a thread for ecatcheck. This is a cross-platform thread-creation.
    ret1 = osal_thread_create(&_thread_ecatcheck, 128000, (void*)&esmacat_master::InternalThreadEntry_ecatcheck, this );
    // create a thread for esmacat_master. This is a cross-platform thread-creation.
    ret2 = osal_thread_create_rt(&_thread_esmacat_master_loop, 128000, (void*)&InternalThreadEntry_esmacat_master_loop, this);
    if (ret1 ==0 && ret2 == 0)
    {
        return 0; // if successful, return 0
    }
    else
    {
        return 1;
    }
}

/** @brief Continuously checks the EtherCAT status
 *
 * If the slave belongs to the current group and the state of the slave is:
 *
 * EC_STATE_SAFE_OP + EC_STATE_ERROR then it is attempted to be set to C_STATE_SAFE_OP + EC_STATE_ACK
 *
 * EC_STATE_SAFE_OP then it is attempted to be set to EC_STATE_OPERATIONAL
 *
 * Any state but EC_STATE_NONE then it is reconfigured
 *
 * If the slave is flagged to be unresponsive, and a state check reveals slave to be in a valid state,
 * then the flag is reset. Otherwise, the slave is attempted to be recovered and flag reset.
 *
 * Once all the slaves have been checked to be in operational state, the thread is suspended for 10 ms
 */
void esmacat_master::ecatcheck()
{ // continuously check the ethercat status
    int slave;
    while(1)
    {
        // if either all slaves are operational and the work counter is less that its expected value
        // or if one of the slaves in the group needs to be checked
        if( inOP && ((wkc < expectedWKC) || ec_group[currentgroup].docheckstate))
        {
            if (needlf)
            {
                needlf = FALSE;
                printf("\n");
            }
            /* one ore more slaves are not responding */
            ec_group[currentgroup].docheckstate = FALSE;
            //read the state of the slave
            ec_readstate();
            for (slave = 1; slave <= ec_slavecount; slave++)
            {
                //if the slave in consideration belongs to the current group and is not Operational
                if ((ec_slave[slave].group == currentgroup) && (ec_slave[slave].state != EC_STATE_OPERATIONAL))
                {
                    //flag that the state of the slave under consideration has to be checked
                    ec_group[currentgroup].docheckstate = TRUE;

                    //if the slave is in SAFE_OP and has had an error
                    if (ec_slave[slave].state == (EC_STATE_SAFE_OP + EC_STATE_ERROR))
                    {
                        printf("ERROR : slave %d is in SAFE_OP + ERROR, attempting ack.\n", slave);
                        ec_slave[slave].state = (EC_STATE_SAFE_OP + EC_STATE_ACK);
                        ec_writestate(slave);
                    }
                    //if the slave is in the SAFE_OP state
                    else if(ec_slave[slave].state == EC_STATE_SAFE_OP)
                    {
                        //set it to the OP state inform the user
                        printf("WARNING : slave %d is in SAFE_OP, change to OPERATIONAL.\n", slave);
                        ec_slave[slave].state = EC_STATE_OPERATIONAL;
                        ec_writestate(slave);
                    }
                    //if slave is in any valid state
                    else if(ec_slave[slave].state > EC_STATE_NONE)
                    {
                        //reconfigure it and indicate that the connection has been regained
                        if (ec_reconfig_slave(slave, EC_TIMEOUTMON))
                        {
                            ec_slave[slave].islost = FALSE;
                            printf("MESSAGE : slave %d reconfigured\n",slave);
                        }
                    }
                    //if slave is responding
                    else if(!ec_slave[slave].islost)
                    {
                        // check if the stae of the slave is Operational
                        ec_statecheck(slave, EC_STATE_OPERATIONAL, EC_TIMEOUTRET);
                        //if the slave is not in a valid state, indicate that the connection has been lost
                        if (ec_slave[slave].state == EC_STATE_NONE)
                        {
                            ec_slave[slave].islost = TRUE;
                            printf("ERROR : slave %d lost\n",slave);
                        }
                    }
                }
                //if slave is not responding (regardless of whether it is in the current group)
                if (ec_slave[slave].islost)
                {   //if slave is not in a valid state
                    if(ec_slave[slave].state == EC_STATE_NONE)
                    {
                        //try to recover the connection with the slave
                        if (ec_recover_slave(slave, EC_TIMEOUTMON))
                        {
                            ec_slave[slave].islost = FALSE;
                            printf("MESSAGE : slave %d recovered\n",slave);
                        }
                    }
                    else
                    { //slave is in a valid state, indicate that the connection has been regained
                        ec_slave[slave].islost = FALSE;
                        printf("MESSAGE : slave %d found\n",slave);
                    }
                }
            }
            // all the slaves' status has been checked
            if(!ec_group[currentgroup].docheckstate)
                printf("OK : all slaves resumed OPERATIONAL.\n");
        }
        //suspend the execution of this thread for 10,000us
        osal_usleep(10000);
        //if the thread has been flagged to stop, then this loop terminates
        if (stop_thread_loop == TRUE)
        {
            break;
        }
    }
}

/** @brief This function supports the exchange of real-time data between
 *  the master and slaves
 */
void esmacat_master::esmacat_master_loop()
{
    ec_timet t;
    ec_timet t_prev;
    ec_timet t_now;
    uint32 interval = ESMACAT_TIME_PERIOD_US;
    long d;

    //pre-fault the stack
    stack_prefault();

    // current time is stored in t which is expressed in the form of secs + usec
    t = osal_current_time();
    t_prev = osal_current_time();
    // Increment t by 1 second. This ensures that there's sufficient time to complete initialization
    t.sec++;
    // end of rt part

    int i, j, oloop, iloop, chk,cnt;
    double time_elapsed = 0;
    i = 0;
    needlf = FALSE;
    inOP = FALSE;
    int sum_of_system_parameter_buffer = 0;

    printf("\nStarting Esmacat Master\n");

    // initialise SOEM, bind socket to ifname (ethercat adapter name provided by user)
    if (ec_init(ifname))
    {
        printf("ec_init on %s succeeded.\n",ifname);

        // if one or more slaves were able to be found and auto-configured
        if ( ec_config_init(FALSE) > 0 )
        {
            printf("%d slaves found and configured.\n",ec_slavecount);

            //map their PDOs (process data objects for CANOpen) to the IOmap
            ec_config_map(&IOmap);
            //Configure all the slaves that support Distributed Clock
            ec_configdc();

            printf("Slaves mapped, state to SAFE_OP.\n");
            // wait for all slaves to reach SAFE_OP state
            ec_statecheck(0, EC_STATE_SAFE_OP,  EC_TIMEOUTSTATE * 4);

            // read indevidual slave state and store in ec_slave[]
            ec_readstate();

            // read slave information and, recognize the slave
            for(cnt = 1; cnt <= ec_slavecount ; cnt++)
            {
                printf("\nSlave Index: %d\n", cnt-1);
                printf("Manufacturer code: %8.8x Name: %s Product ID: %8.8x,  Rev: %8.8x iByte: %d oByte: %d\n", ec_slave[cnt].eep_man, ec_slave[cnt].name, ec_slave[cnt].eep_id, ec_slave[cnt].eep_rev, ec_slave[cnt].Ibytes,ec_slave[cnt].Obytes);
            }

            // Proceed to link each slave with its corresponding object in code
            ECAT_app->assign_slave_sequence();
            // check if the items were correctly connected.
            int error_flag_slave_vendor_product_id = 0;

            //this loop verifies that for each slave connected in the communication chain, the Vendor ID and Product Code are as expected by the application
            for(cnt = 1; cnt <= ec_slavecount ; cnt++)
            {
                if ( ECAT_app->ethercat_vendor_id_list[cnt-1] == DEFAULT_PRODUCT_VENDOR_ID && ECAT_app->ethercat_product_id_list[cnt-1] == DEFAULT_PRODUCT_VENDOR_ID  )
                {
                    error_flag_slave_vendor_product_id = 3; // no corresponding variable exist! application does not use this ethercat slave
                    printf("No corresponding variable for %dth slave exist! Application may not want to use this ethercat slave\n",cnt-1);
                }
                else if ( ec_slave[cnt].eep_man != ECAT_app->ethercat_vendor_id_list[cnt-1] )
                {
                    error_flag_slave_vendor_product_id = 1;
                    printf("Vendor ID of %d th item is not matched with your variable!!\n connected_slave_id: %0x, your_variable_slave_id: %0x\n",cnt-1,ec_slave[cnt].eep_man,ECAT_slave[cnt-1]->get_esmacat_vendor_id());
                    stop_thread();
                }
                else if ( ec_slave[cnt].eep_id != ECAT_app->ethercat_product_id_list[cnt-1])
                {
                    error_flag_slave_vendor_product_id = 2;
                    printf("There is no matched product ID for %d th item!!\n connected_slave_id: %0x, your_variable_slave_id: %0x\n",cnt-1,ec_slave[cnt].eep_id,ECAT_slave[cnt-1]->get_esmacat_product_id());
                    stop_thread();
                }
                else
                {
                    printf("Well matched between the connected product and your variables!!\n connected_slave_id: %0x, your_variable_slave_id: %0x\n",ec_slave[cnt].eep_id,ECAT_slave[cnt-1]->get_esmacat_product_id());
                    error_flag_slave_vendor_product_id  = 0;
                }
            }
            // send the intialization information contained in the application
            ECAT_app->configure_slaves();

            expectedWKC = (ec_group[0].outputsWKC * 2) + ec_group[0].inputsWKC;

            ec_slave[0].state = EC_STATE_OPERATIONAL;
            /* send one valid process data to make outputs in slaves happy*/
            ec_send_processdata();
            ec_receive_processdata(EC_TIMEOUTRET);

            /* request OP state for all slaves */
            ec_writestate(0);
			chk = 40;
            /* wait for all slaves to reach OP state */
            do
            {

                ec_send_processdata();
                ec_receive_processdata(EC_TIMEOUTRET);

                ec_statecheck(0, EC_STATE_OPERATIONAL, 50000);
            }
            while (chk-- && (ec_slave[0].state != EC_STATE_OPERATIONAL));
            //as soon as the slave enters into the Operational state
            if (ec_slave[0].state == EC_STATE_OPERATIONAL )
            {
                printf("\nOperational state reached for all slaves.\n");
                //set master flag to indicate it is in operation

                inOP = TRUE;
				t_prev = osal_current_time();
                //infinite loop
                while(1)
                {
					/*************commented only for windows **********/
                    ec_send_processdata();
                    wkc = ec_receive_processdata(EC_TIMEOUTRET);
					/************end windows *************************/
                    if(wkc >= expectedWKC)
                    {
                        sum_of_system_parameter_buffer = 0;
                        // for each slave, exchange the data enqueued
                        for(cnt = 1; cnt <= ec_slavecount  ; cnt++)
                        {
                            oloop = ec_slave[cnt].Obytes;
                            if ((oloop == 0) && (ec_slave[cnt].Obits > 0)) oloop = 1;
                            iloop = ec_slave[cnt].Ibytes;
                            if ((iloop == 0) && (ec_slave[cnt].Ibits > 0)) iloop = 1;
                            if ( ECAT_app->ethercat_product_id_list[cnt-1] != DEFAULT_PRODUCT_VENDOR_ID){
                                ECAT_slave[cnt-1]->ecat_data_process( (uint8_t*)&(ec_slave[cnt].outputs[0]),oloop,(uint8_t*)&(ec_slave[cnt].inputs[0]),iloop);
                                sum_of_system_parameter_buffer +=  ECAT_slave[cnt-1]->flush_one_set_of_system_parameters();
                            }
                        }
                        //data exchange for the cycle has been completed
                        if (sum_of_system_parameter_buffer == 0)
                        {
                            ECAT_app->set_elapsed_time_ms(time_elapsed);
                            ECAT_app->increment_loop_cnt();
                            if (ECAT_app->get_loop_cnt() == 1)
                            {
                                ECAT_app->init();
                            }
                            else
                            {
                                ECAT_app->loop();
                            }
                        }

                        else
                        {
                            // do nothing.
                        }
                        needlf = TRUE;
                    }

                    // set the target for the start of the next interval
                    t.usec += interval;

                    // if the portion of time under the usec is higher than 1 s, then accordingly adjust the
                    // two variables under it to correctly represent the total seconds and usec
                    while (t.usec >= USEC_PER_SEC)
                    {
                        t.usec -= USEC_PER_SEC;
                        t.sec++;
                    }
                    // since the real-time processing has been completed, the thread is suspended (sleep)
                    // until the target time is reached
					
                    ec_timet sleeptime;
                    osal_time_diff(&t_prev, &t, &sleeptime);
                    osal_usleep((uint32)sleeptime.usec);

                    //after sleep cycle is complete, obtain the current time
                    t_now= osal_current_time();

                    //determine the time that has elapsed between the start of the application's last loop and now (in us)
                    d = t_now.usec - t_prev.usec;

                    if (d<0)
                    {
                        d = d+ USEC_PER_SEC;
                    }

                    if (d> 1.02*interval)
                    {
                        error_counter++;
                    }
                    //prepare for the next interval
                    t_prev.sec = t_now.sec;
                    t_prev.usec = t_now.usec;

                    // compute elapsed time in ms
                    time_elapsed += ((double)d/1000);

                    //if thread has been flagged to be stopped, terminate the thread
                    if (stop_thread_loop == TRUE)
                    {
                        break;
                    }
                }
                // since the thread has been terminated, the master is no longer in operation
                inOP = FALSE;
            }
            // if all slaves are not in operational state, print their status and set them to the INIT state
            else
            {
                printf("Not all slaves reached operational state.\n");
                ec_readstate();
                for(i = 1; i<=ec_slavecount ; i++)
                {
                    if(ec_slave[i].state != EC_STATE_OPERATIONAL)
                    {
                        printf("Slave %d State=0x%2.2x StatusCode=0x%4.4x : %s\n",
                               i, ec_slave[i].state, ec_slave[i].ALstatuscode, ec_ALstatuscode2string(ec_slave[i].ALstatuscode));
                    }
                }
            }
            printf("\nRequest init state for all slaves\n");
            ec_slave[0].state = EC_STATE_INIT;
            /* request INIT state for all slaves */
            ec_writestate(0);
        }
        // if no slaves were found
        else
        {
            printf("No slaves found!\n");
        }
        printf("Esmacat Master, close socket\n");
        /* stop SOEM, close socket */
        ec_close();
    }
    // unable to initialize ethercat adapter name of specified name
    else
    {
        printf("No socket connection on %s\nExcecute as root\n",ifname);
    }
    //flag the connection as closed
    ec_closed=1;
}

/** @brief Returns the status of the ethercat master connection
 * @return Boolean value indicating the status of the ethercat master connection
*/
bool esmacat_master::is_ec_closed()
{
    return ec_closed;
}

/** @brief Sets the ethercat adapter name
 *
 * The input name is passed into the SOEM
 *
 * @param eth_adapter_name Name provided for the ethercat adapter
 */
void esmacat_master::set_ethernet_adapter_name_for_esmacat(char* eth_adapter_name)
{
    strcpy(ifname, eth_adapter_name);
}

/** @brief Ensures that the thread has been closed successfully */
void esmacat_master::WaitForInternalThreadToExit()
{
    //(void) pthread_join(*_thread_esmacat_master_loop, NULL);
}

uint32 esmacat_master::get_error_counter()
{
    return error_counter;
}
