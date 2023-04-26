from setuptools import setup
from setuptools import find_packages

package_name = 'ros2diagnostics_cli'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
    ],
    install_requires=['ros2cli'],
    zip_safe=True,
    maintainer='user',
    maintainer_email='robo2020@gmail.com',
    description='TODO: Package description',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'ros2cli.command': [
            'diagnostics = ros2diagnostics_cli.command.diagnostics:DiagCommand',
        ],
        'ros2diagnostics_cli.verb': [
            'world = ros2diagnostics_cli.verb.world:WorldVerb',
            'csv = ros2diagnostics_cli.verb.csv:CSVVerb',
            'list = ros2diagnostics_cli.verb.list:ListVerb',
            'show = ros2diagnostics_cli.verb.show:ShowVerb'
        ]
    },
)
