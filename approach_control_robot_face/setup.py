from distutils.core import setup
from catkin_pkg.python_setup import generate_distutils_setup

setup_args = generate_distutils_setup(
    packages = ['approach_control_robot_face'],
    package_dir = {'' : 'nodes'}
)

setup(**setup_args)