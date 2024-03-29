#!/usr/bin/env python

import os
from distutils.core import setup

from setuptools import find_packages

folder = os.path.dirname(os.path.realpath(__file__))
requirements_path = os.path.join(folder, 'requirements.txt')
install_requires = []
if os.path.isfile(requirements_path):
    with open(requirements_path) as f:
        install_requires = f.read().splitlines()

setup(name='motion_planner',
      version='0.1',
      description='Installation of motion planner requirements',
      author='WareVision LLC Team',
      author_email='',
      package_dir={},
      packages=find_packages(),
      install_requires=install_requires
      )
