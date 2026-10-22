from setuptools import setup, find_packages
setup(name="inspection-cli", version="0.1.0", packages=find_packages(), install_requires=["inspection-runtime==0.1.0"], entry_points={"console_scripts":["inspection = inspection_cli.main:main"]})
