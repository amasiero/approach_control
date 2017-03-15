from distutils.core import setup
from catkin_pkg.python_setup import generate_disutils_setup

setup_args = generate_disutils_setup(
	packages=['approach_control_voice'],
	package_dir = {'': 'nodes'},
)