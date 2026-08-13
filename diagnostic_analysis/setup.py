from setuptools import find_packages, setup

package_name = 'diagnostic_analysis'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='wirthra',
    maintainer_email='wirthra@gmail.com',
    description='The diagnostic_analysis package can convert a log of diagnostics data \
    into a series of CSV files. Robot logs are recorded with rosbag, and \
    can be processed offline using the scripts in this package.',
    license='BSD',
    extras_require={
        'test': [
            'pytest',
        ],
    },
    entry_points={
        'console_scripts': [
            'export_csv = diagnostic_analysis.export_csv:main',
            'sparse_csv = diagnostic_analysis.sparse_csv:main',
        ],
    },
)
