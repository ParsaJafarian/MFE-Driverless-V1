from setuptools import find_packages
from setuptools import setup

setup(
    name='mfe_msgs',
    version='0.0.0',
    packages=find_packages(
        include=('mfe_msgs', 'mfe_msgs.*')),
)
