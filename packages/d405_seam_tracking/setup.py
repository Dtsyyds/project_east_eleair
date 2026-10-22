from setuptools import setup, find_packages
import toml
from pathlib import Path
project = toml.loads((Path(__file__).parent / "pyproject.toml").read_text())["project"]
setup(name=project["name"], version=project["version"], packages=find_packages("src"), package_dir={"": "src"}, python_requires=project["requires-python"], install_requires=project["dependencies"], extras_require=project.get("optional-dependencies", {}), entry_points={"console_scripts": [name + " = " + value for name,value in project["scripts"].items()]})
