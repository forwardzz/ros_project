from setuptools import find_packages, setup

setup(
    name="YbImuLib",
    version="1.0.0",
    author="Yahboom Team",
    packages=find_packages(),
    tests_require=["pytest"],
)

# pip3 install -e .
