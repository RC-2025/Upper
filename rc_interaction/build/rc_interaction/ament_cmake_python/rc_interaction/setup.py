from setuptools import find_packages
from setuptools import setup

setup(
    name='rc_interaction',
    version='0.0.1',
    packages=find_packages(
        include=('rc_interaction', 'rc_interaction.*')),
)
