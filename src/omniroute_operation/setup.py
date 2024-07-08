#!/usr/bin/env python

# Import necessary modules
from distutils.core import setup
from catkin_pkg.python_setup import generate_distutils_setup

# Generate the setup configuration dictionary using the generate_distutils_setup function
d = generate_distutils_setup(
    ## Uncomment the line below to include globally visible scripts (not recommended unless necessary)
    # scripts=['bin/myscript'], 
    
    # Define the packages to be included in the distribution
    packages=['omniroute_controller', 'experiment_controller',
              'gantry', 'rat_detector', 'transformer', 'ts_sync', 'shared_utils'],
    
    # Specify the root directory for the package source files
    package_dir={'': 'src'},
    
    # List of scripts to be installed and their locations
    scripts=['scripts/omniroute_controller', 'scripts/experiment_controller']
)

# Pass the configuration dictionary to the setup function to create the package
setup(**d)