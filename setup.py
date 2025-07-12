"""
Package configuration for IKsolver package.

This setup script handles the package installation and dependencies.
"""

from setuptools import find_packages, setup

# Read the main requirements from requirements.txt
with open("requirements.txt") as f:
    requirements = f.read().splitlines()

# Package configuration
setup(
    # Basic package information
    name="iksolver",
    version="0.1.0",
    author="Hoomaan Moradi",
    # Automatically find all packages in the project
    packages=find_packages(),
    # Main dependencies required for the package to run
    install_requires=requirements,
)
