from setuptools import find_packages, setup

setup(
    name="stunt",
    version="0.0.1",
    author="Gabriele Baldoni",
    description=("A platform for developing autonomous vehicles."),
    long_description=open("../README.md").read(),
    url="",
    keywords=("autonomous vehicles driving python CARLA simulation"),
    packages=find_packages(),
    license="Apache 2.0",
    install_requires=[],
)
