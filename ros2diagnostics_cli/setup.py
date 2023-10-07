from setuptools import setup

package_name = 'ros2diagnostics_cli'

setup(
    name=package_name,
    version='0.0.0',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='user',
    maintainer_email='robo2020@gmail.com',
    description='TODO: Package description',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'ros2cli.command': [
            'hello = ros2diagnostics_cli.command.hello:HelloCommand',
        ],
        'ros2diagnostics_cli.verb': [
            'world = ros2diagnostics_cli.verb.world:WorldVerb',
        ]
    },
)
