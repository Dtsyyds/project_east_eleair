from setuptools import find_packages, setup

package_name = "tank_vision_ros"

setup(
    name=package_name,
    version="0.1.0",
    packages=find_packages(exclude=["test"]),
    data_files=[
        ("share/ament_index/resource_index/packages", ["resource/" + package_name]),
        ("share/" + package_name, ["package.xml"]),
        ("share/" + package_name + "/launch", ["launch/vision.launch.py", "launch/stable_seam_direct.launch.py", "launch/stable_seam_topic.launch.py", "launch/stable_seam_direct_guided_anchor.launch.py"]),
        (
            "share/" + package_name + "/config",
            ["config/default.yaml", "config/ros_topics.yaml"],
        ),
        ("share/" + package_name + "/config", [
            "../../../configs/s100p_stable_seam_4096x3072.yaml",
            "../../../configs/s100p_guided_anchor_seam_4096x3072.yaml",
            "../../../configs/seam_width_reference_s100p_v1.json",
            "../../../configs/seam_geometry_reference_s100p_live_v1.json",
        ]),
    ],
    install_requires=["setuptools"],
    zip_safe=True,
    maintainer="Tank Vision Team",
    maintainer_email="maintainer@example.com",
    description="ROS 2 wrapper for tank vision inspection",
    license="Apache-2.0",
    entry_points={"console_scripts": [
        "vision_node = tank_vision_ros.vision_node:main",
        "stable_seam_node = tank_vision_ros.stable_seam_node:main",
        "seam_debug_viewer = tank_vision_ros.seam_debug_viewer:main",
    ]},
)
