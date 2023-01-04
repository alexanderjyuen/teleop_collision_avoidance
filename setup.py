from distutils.core import setup
from catkin_pkg.python_setup import generate_distutils_setup

d = generate_distutils_setup(
    packages=['teleop_collision_avoidance'],
    scripts=['scripts/teleop_collision_avoidance.py'],
    package_dir={'': 'src'}
)

setup(**d)
