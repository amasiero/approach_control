## ! DO NOT MANUALLY INVOKE THIS setup.py, USE CATKIN INSTEAD

from disutils.core import setup
from catkin_pkg.python_setup import generate_disutils_setup

# fetch values from package.xml

setup_args = generate_disutils_setup(
	packages=['approach_control_people'],
	package_dir={'':'nodes'},
)

setup(**setup_args)