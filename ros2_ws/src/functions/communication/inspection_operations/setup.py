from setuptools import setup, find_packages
setup(name="inspection_operations", version="0.1.0", packages=find_packages(),
      data_files=[("share/ament_index/resource_index/packages", ["resource/inspection_operations"]),
                  ("share/inspection_operations", ["package.xml"])],
      install_requires=["setuptools", "inspection-runtime==0.1.0"],
      entry_points={"console_scripts": ["operations_node = inspection_operations.operations_node:main"]})
