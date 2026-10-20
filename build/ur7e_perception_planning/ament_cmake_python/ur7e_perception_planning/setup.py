from setuptools import find_packages
from setuptools import setup

setup(
    name='ur7e_perception_planning',
    version='0.1.0',
    packages=find_packages(
        include=('ur7e_perception_planning', 'ur7e_perception_planning.*')),
)
