#!/usr/bin/env python

from distutils.core import setup
from catkin_pkg.python_setup import generate_distutils_setup

d = generate_distutils_setup(
    ##  don't do this unless you want a globally visible script
    # scripts=['bin/myscript'], 
    packages=['omniroute_controller', 'experiment_controller', 'gantry', 'shared_classes'],
    package_dir={'': 'src'},
    scripts=['scripts/omniroute_controller', 'scripts/experiment_controller']
)

setup(**d)
