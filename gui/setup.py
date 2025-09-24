#!/usr/bin/env python3

from distutils.core import setup
from catkin_pkg.python_setup import generate_distutils_setup

d = generate_distutils_setup(
    packages=["mcl_quadruped_gui"],
    package_dir={"mcl_quadruped_gui": "src/mcl_quadruped_gui"},
    scripts=["scripts/mcl_quadruped_gui"],
)

setup(**d)
