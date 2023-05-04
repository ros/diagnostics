from setuptools import setup

package_name = "diagnostics_tutorial"

setup(
    name=package_name,
    version="0.0.0",
    packages=[package_name],
    data_files=[
        ("share/ament_index/resource_index/packages", ["resource/" + package_name]),
        ("share/" + package_name, ["package.xml"]),
    ],
    install_requires=["setuptools"],
    zip_safe=True,
    maintainer="user",
    maintainer_email="robo2020@gmail.com",
    description="TODO: Package description",
    license="TODO: License declaration",
    tests_require=["pytest"],
    entry_points={
        "console_scripts": [
            "wrapper_demo=diagnostics_tutorial.status_wrapper_demo:main",
            "comp=diagnostics_tutorial.composite_task_demo:main",
            "freq=diagnostics_tutorial.freq_diagnostics_demo:main",
            "task=diagnostics_tutorial.task_demo:main"

        ],
    },
)
