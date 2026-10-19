from setuptools import find_packages, setup

package_name = "fisheye_camera_driver"

setup(
    name=package_name,
    version="0.1.0",
    packages=find_packages(exclude=["test"]),
    data_files=[
        ("share/ament_index/resource_index/packages", ["resource/" + package_name]),
        ("share/" + package_name, ["package.xml"]),
        ("share/" + package_name + "/config", ["config/fisheye.yaml"]),
        ("share/" + package_name + "/launch", ["launch/fisheye_camera.launch.py"]),
    ],
    install_requires=["setuptools"],
    tests_require=["pytest"],
    zip_safe=True,
    maintainer="Tank Inspection Maintainer",
    maintainer_email="maintainer@example.com",
    description="V4L2 fisheye camera ROS 2 driver.",
    license="Apache-2.0",
    entry_points={"console_scripts": ["fisheye_camera_node = fisheye_camera_driver.node:main"]},
)
