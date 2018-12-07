from setuptools import find_packages
from setuptools import setup
package_name = 'diagnostic_common_diagnostics'

setup(
    name=package_name,
    version='0.5.1',
    packages=find_packages(exclude=['test']),
   # data_files=[
   #     ('share/ament_index/resource_index/packages',
   #         ['resource/' + package_name]),
   #     ('share/' + package_name, ['package.xml']),
   # ],
    install_requires=['setuptools'],
    zip_safe=True,
    author='Esteve Fernandez',
    author_email='esteve@osrfoundation.org',
    maintainer='Mikael Arguedas',
    maintainer_email='mikael@osrfoundation.org',
    keywords=['ROS'],
    classifiers=[
        'Intended Audience :: Developers',
        'License :: OSI Approved :: Apache Software License',
        'Programming Language :: Python',
        'Topic :: Software Development',
    ],
    description=(
        'Python nodes which were previously in the ros2/examples repository '
        'but are now just used for demo purposes.'
    ),
    license='Apache License, Version 2.0',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
             'cpu_monitor = diagnostic_common_diagnostics.cpu_monitor:main',
             'hd_monitor = diagnostic_common_diagnostics.hd_monitor:main',
             'tf_monitor = diagnostic_common_diagnostics.tf_monitor:main',
             'ntp_monitor = diagnostic_common_diagnostics.ntp_monitor:main',
             'sensors_monitor = diagnostic_common_diagnostics.sensors_monitor:main',
             #'cpu_monitor_psutil_mock = diagnostic_common_diagnostics.test.cpu_monitor_psutil_mock:main',
             #'test_cpu_monitor = diagnostic_common_diagnostics.test.test_cpu_monitor:main'
        ],
    },
)

