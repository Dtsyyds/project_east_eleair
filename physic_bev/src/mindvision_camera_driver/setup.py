from setuptools import find_packages, setup

package_name = "mindvision_camera_driver"

setup(
    name=package_name,
    version="0.1.0",
    packages=find_packages(exclude=["test"]),
    data_files=[
        ("share/ament_index/resource_index/packages", ["resource/" + package_name]),
        ("share/" + package_name, ["package.xml"]),
        ("share/" + package_name + "/config", [
            "config/mindvision_suf1205gc_s100p.yaml",
            "config/mindvision_sus1207gc_t1v_c.yaml",
        ]),
        ("share/" + package_name + "/launch", ["launch/mindvision_camera.launch.py"]),
    ],
    install_requires=["setuptools"],
    tests_require=["pytest"],
    zip_safe=True,
    maintainer="Tank Inspection Maintainer",
    maintainer_email="maintainer@example.com",
    description="ROS 2 Image/CameraInfo publisher for MindVision mono and color industrial cameras.",
    license="Apache-2.0",
    entry_points={"console_scripts": ["mindvision_camera_node = mindvision_camera_driver.node:main"]},
)
