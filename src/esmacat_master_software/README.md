
# Esmacat Master Software Repository

## Folder structure

-   esmacat_applications : Contains the template to create your own application as well as project examples of various Esmacat products
-   esmacat_library : Contains the source/header files for the Esmacat master to communicate with slaves
-   esmacat_slave_drivers : Contains the source/header files for all Esmacat slaves. Can also be used to generate source files to communicate with other EtherCAT slaves.
-   ethercat_driver :  Simple Open-Source EtherCAT Master (SOEM) Open Source library that supports the EtherCAT Master

For the details of the software structure, see [this document](https://harmonicbionics.sharepoint.com/:f:/g/r&d/esmacat/EmEX3sXoNEBAvOpXtQfpgAEBkR4264OSkLfmB5s-T45fRg?e=Iwcg1N). 

## !!Recent Syntax Change of Esmacat
Please refer to the folder "example" to see the recent change of the syntax (functions) from our esmacat-server repository to the esmacat_master_software repository. There are some minor differences from our YouTube tutorials. 

## Getting started

### Windows

1.  Install
    1.  winpcap ([https://www.winpcap.org/install/](https://www.winpcap.org/install/))
    2.  cmake
    3.  git
    4.  MS visual Studio
2.  Download the source file or git clone [https://bitbucket.org/harmonicbionics/esmacat_master_software.git](https://bitbucket.org/harmonicbionics/esmacat_master_software.git)
3.  Open esmacatserver folder with Visual Studio
4.  Build the project

For the details, see [this document](https://harmonicbionics.sharepoint.com/:b:/g/r&d/esmacat/ER4ycYgTFdlHrorrghtO6t4B7wFePqLpQNu4RozpZGvV0g?e=J6bKiW).

### Linux

1.  Install build-essential, git and cmake. (e.g., sudo apt-get install build-essential git cmake)
2.  git clone [https://bitbucket.org/harmonicbionics/esmacat_master_software.git](https://bitbucket.org/harmonicbionics/esmacat_master_software.git)
3.  cd esmacat_master_software
4.  mkdir build
5.  cd build
6.  cmake ..
7.  make

For the details, see [this document](https://harmonicbionics.sharepoint.com/:b:/g/r&d/esmacat/EWbdH6XVaRNLq8c3SfHWBO0B185PGgzWmn7aN5-VwqkI8A?e=bRH702).

## Visit our website for the detailed product information
### [https://esmacat.com](https://esmacat.com)