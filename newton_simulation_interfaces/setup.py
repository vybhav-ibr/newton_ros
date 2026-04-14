from setuptools import setup, find_packages

package_name = "newton_simulation_interfaces"

setup(
    name=package_name,
    version="1.0.0",
    packages=find_packages(),
    data_files=[
        ("share/ament_index/resource_index/packages", ["resource/" + package_name]),
        ("share/" + package_name, ["package.xml"]),
    ],
    install_requires=["setuptools", "newton_ros_interfaces", "simulation_interfaces"],
    zip_safe=True,
    maintainer="Vybhav Ilindra",
    maintainer_email="ibr.vybhav@gmail.com",
    description="Open Simulation Interfaces (OSI) implementation for Genesis simulator",
    license="MIT",
    tests_require=["pytest"],
    entry_points={
        "console_scripts": [],
    },
)
