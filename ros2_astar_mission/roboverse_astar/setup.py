from glob import glob
from setuptools import find_packages, setup


package_name = "roboverse_astar"


setup(
    name=package_name,
    version="0.1.0",
    packages=find_packages(exclude=["test"]),
    data_files=[
        ("share/ament_index/resource_index/packages", [f"resource/{package_name}"]),
        (f"share/{package_name}", ["package.xml"]),
        (f"share/{package_name}/launch", glob("launch/*.launch.py")),
        (f"share/{package_name}/config", glob("config/*.yaml")),
    ],
    install_requires=["setuptools"],
    zip_safe=True,
    maintainer="stafford99",
    maintainer_email="stafford99@example.com",
    description="ROS2 A* baseline for RoboVerse PX4 X500 qualifier.",
    license="MIT",
    entry_points={
        "console_scripts": [
            "depth_mapper_node = roboverse_astar.depth_mapper_node:main",
            "frontier_goal_node = roboverse_astar.frontier_goal_node:main",
            "astar_planner_node = roboverse_astar.astar_planner_node:main",
            "fuel_detector_node = roboverse_astar.fuel_detector_node:main",
            "mission_manager_node = roboverse_astar.mission_manager_node:main",
            "px4_offboard_node = roboverse_astar.px4_offboard_node:main",
            "mavsdk_waypoint_follower_node = roboverse_astar.mavsdk_waypoint_follower_node:main",
            "dataset_capture_node = roboverse_astar.dataset_capture_node:main",
        ],
    },
)
