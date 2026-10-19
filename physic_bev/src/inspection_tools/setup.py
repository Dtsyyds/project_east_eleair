from setuptools import find_packages, setup

package_name = "inspection_tools"

setup(
    name=package_name,
    version="0.1.0",
    packages=find_packages(exclude=["test"]),
    data_files=[
        ("share/ament_index/resource_index/packages", ["resource/" + package_name]),
        ("share/" + package_name, ["package.xml"]),
        ("share/" + package_name + "/config", ["config/stability_config.yaml"]),
    ],
    install_requires=["setuptools"],
    tests_require=["pytest"],
    zip_safe=True,
    maintainer="Tank Inspection Maintainer",
    maintainer_email="maintainer@example.com",
    description="Validation and immutable finalization tools for inspection MCAP bags.",
    license="Apache-2.0",
    entry_points={
        "console_scripts": [
            "validate_bag = inspection_tools.validate_bag:main",
            "finalize_bag = inspection_tools.finalize_bag:main",
            "estimate_storage = inspection_tools.estimate_storage:main",
            "depth_to_pointcloud = inspection_tools.depth_to_pointcloud:main",
            "inspect_latest = inspection_tools.inspect_latest:main",
            "stability_monitor = inspection_tools.stability_monitor:main",
            "timestamp_audit = inspection_tools.timestamp_audit:main",
            "time_sync_analysis = inspection_tools.time_sync_analysis:main",
            "eca_mock = inspection_tools.eca_mock:main",
            "websocket_validation = inspection_tools.websocket_validation:main",
            "livox_continuity = inspection_tools.livox_continuity:main",
            "time_readiness = inspection_tools.time_readiness:main",
            "usb_lifecycle = inspection_tools.usb_lifecycle:main",
        ],
    },
)
