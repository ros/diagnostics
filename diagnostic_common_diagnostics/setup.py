from setuptools import setup

package_name = 'diagnostic_common_diagnostics'

setup(
    name=package_name,
    version='4.0.0',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages',
         [f'resource/{package_name}']),
        ('share/' + package_name, ['package.xml']),
        ('lib/' + package_name, [
            'cpu_monitor.py',
            'ntp_monitor.py',
            'ram_monitor.py',
            'sensors_monitor.py',
            'hd_monitor.py',
        ]),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='root',
    maintainer_email='vladyslav.hrynchak@logivations.com',
    description='Package for diagnostics',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'cpu_monitor = diagnostic_common_diagnostics.cpu_monitor:main',
            'ntp_monitor = diagnostic_common_diagnostics.ntp_monitor:main',
            'ram_monitor = diagnostic_common_diagnostics.ram_monitor:main',
            'sensors_monitor = diagnostic_common_diagnostics.sensors_monitor:main',
            'hd_monitor = diagnostic_common_diagnostics.hd_monitor:main',
        ],
    },
)
