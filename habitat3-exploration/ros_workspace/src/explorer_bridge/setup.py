from setuptools import find_packages, setup

package_name = "explorer_bridge"

setup(
    name=package_name,
    version="0.1.0",
    packages=find_packages(exclude=["test"]),
    data_files=[
        ("share/ament_index/resource_index/packages", ["resource/" + package_name]),
        ("share/" + package_name, ["package.xml"]),
        ("share/" + package_name + "/launch", ["launch/habitat_sim.launch.py"]),
    ],
    install_requires=["setuptools"],
    zip_safe=True,
    maintainer="Explorer Team",
    maintainer_email="team@example.com",
    description="ROS 2 bridge for Habitat explorer sim and future hardware",
    license="MIT",
    tests_require=["pytest"],
    entry_points={
        "console_scripts": [
            "explorer_bridge_node = explorer_bridge.explorer_bridge_node:main",
            "habitat_map_node = explorer_bridge.habitat_map_node:main",
            "depth_camera_info_node = explorer_bridge.depth_camera_info_node:main",
            "cmd_vel_to_discrete_node = explorer_bridge.cmd_vel_to_discrete_node:main",
        ],
    },
)
