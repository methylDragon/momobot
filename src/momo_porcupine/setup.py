#!/usr/bin/env python

from distutils.core import setup
from catkin_pkg.python_setup import generate_distutils_setup

setup_args = generate_distutils_setup(

    # State your package directories within /src here
    packages = ['momo_porcupine'],

    # Script locations
    scripts = ['/src/Porcupine/demo/python/porcupine_demo.py'],

    # root/src, basically
    package_dir = {'': 'src'},

    # Your Python dependencies (eg. 'serial')
    install_requires = ['']
)

setup(**setup_args)
