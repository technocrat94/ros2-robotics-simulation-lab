from setuptools import find_packages, setup

package_name = "newton_ros_bridge"
setup(
    name=package_name,
    version="0.1.0",
    packages=find_packages(),
    data_files=[
        ("share/ament_index/resource_index/packages", ["resource/" + package_name]),
        ("share/" + package_name,
         ["package.xml", "newton_endpoint.py", "newton_robot_endpoint.py"]),
    ],
    install_requires=["setuptools"],
    zip_safe=True,
    maintainer="yuhao",
    maintainer_email="yuhao@example.com",
    description="Minimal observable bridge between ROS 2 Humble and Newton",
    license="Apache-2.0",
    entry_points={"console_scripts": ["ros_adapter = newton_ros_bridge.ros_adapter:main"]},
)
