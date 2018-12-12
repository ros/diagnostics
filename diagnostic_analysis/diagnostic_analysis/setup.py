## ! DO NOT MANUALLY INVOKE THIS setup.py, USE CATKIN INSTEAD

#from distutils.core import setup
#from catkin_pkg.python_setup import generate_distutils_setup
from setuptools import find_packages
from setuptools import setup

# fetch values from package.xml
#setup_args = generate_distutils_setup(
#    packages=['diagnostic_analysis'],
#    package_dir={'': 'src'})

#setup(**setup_args)
package_name = 'diagnostic_analysis'

setup(
    name=package_name,
    version='0.5.1',
    packages=find_packages(exclude=['test']),
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
             'export_csv = diagnostic_analysis.export_csv:main',
             'sparse_csv = diagnostic_analysis.sparse_csv:main',
             'bag_csv_test = test.bag_csv_test:main'
             #'listener = demo_nodes_py.topics.listener:main',
        ],
    },
)
