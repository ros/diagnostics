#!/usr/bin/env python

from setuptools import setup
from catkin_pkg.python_setup import generate_distutils_setup

d = generate_distutils_setup(
    packages=['diagnostic_updater'],
    package_dir={'': 'src'}
)

setup(**d)
