from setuptools import find_packages, setup

package_name = "explorer_mission"

setup(
    name=package_name,
    version="0.1.0",
    packages=find_packages(exclude=["test"]),
    data_files=[
        ("share/ament_index/resource_index/packages", ["resource/" + package_name]),
        ("share/" + package_name, ["package.xml"]),
        ("share/" + package_name + "/launch", ["launch/exploration.launch.py"]),
    ],
    install_requires=["setuptools", "requests"],
    zip_safe=True,
    maintainer="Explorer Team",
    maintainer_email="team@example.com",
    description="VLM-aided exploration mission nodes (Python)",
    license="MIT",
    tests_require=["pytest"],
    entry_points={
        "console_scripts": [
            "vlm_node = explorer_mission.vlm.vlm_node:main",
            "maprender_node = explorer_mission.maprender_node:main",
        ],
    },
)
