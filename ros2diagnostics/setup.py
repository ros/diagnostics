from setuptools import find_packages
from setuptools import setup

PACKAGE_NAME = 'ros2diagnostics'

setup(
    name=PACKAGE_NAME,
    version='3.1.2',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + PACKAGE_NAME]),
        ('share/' + PACKAGE_NAME, ['package.xml']),
    ],
    install_requires=['ros2cli'],
    zip_safe=True,
    maintainer='user',
    maintainer_email='robo2020@gmail.com',
    description='diagnostic command for ROS2 command line, parse and show /diagnostics topic',
    license='BSD-3-Clause',
    tests_require=['pytest'],
    entry_points={
        'ros2cli.command': [
            'diagnostics = ros2diagnostics.command.diagnostics:DiagnosticsCommand',
        ],
        'ros2diagnostics.verb': [
            'csv = ros2diagnostics.verb.csv:CSVVerb',
            'list = ros2diagnostics.verb.list:ListVerb',
            'show = ros2diagnostics.verb.show:ShowVerb'
        ]
    },
)
