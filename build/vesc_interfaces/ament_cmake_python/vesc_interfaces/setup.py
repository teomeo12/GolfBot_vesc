from setuptools import find_packages
from setuptools import setup

setup(
    name='vesc_interfaces',
    version='0.0.1',
    packages=find_packages(
        include=('vesc_interfaces', 'vesc_interfaces.*')),
)
