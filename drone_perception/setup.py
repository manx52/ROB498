from distutils.core import setup

from catkin_pkg.python_setup import generate_distutils_setup

# fmt: off
d = generate_distutils_setup(
    packages=['drone_perception'],
    scripts=[],
    package_dir={'': 'src'}
)
# fmt: on

setup(**d)
