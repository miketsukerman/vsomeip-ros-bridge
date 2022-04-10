#!/usr/bin/env python

from setuptools import setup, find_packages
import os

def read(fname):
    return open(os.path.join(os.path.dirname(__file__), fname)).read()

setup(name='franca2ros',
      version='0.0.1',
      description='Franca IDL to ROS2 translator',
      author='Mikhail Tsukerman',
      packages=find_packages(include=['franca2ros.*','franca2ros.translator.*']),
      author_email='miketsukerman@gmail.com',
      license="Apache2",
      keywords="fidl someip ros2",
      long_description=read('readme.md'),
      install_requires=[
          'jinja2', 
          'pyfranca',
          'argparse'
      ],
      entry_points={
          'console_scripts': ['fidl2ros=franca2ros.scripts.fidl2ros:main']
      }
      )
