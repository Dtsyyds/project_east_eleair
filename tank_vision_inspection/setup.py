"""Compatibility metadata for the board's setuptools 59; pyproject is authoritative."""
from pathlib import Path
from setuptools import find_packages, setup
import toml

root = Path(__file__).parent
config = toml.loads((root / "pyproject.toml").read_text())
project = config["project"]
setup(
    name=project["name"], version=project["version"], description=project["description"],
    long_description=(root / project["readme"]).read_text(), long_description_content_type="text/markdown",
    python_requires=project["requires-python"], install_requires=project["dependencies"],
    extras_require=project.get("optional-dependencies", {}),
    packages=find_packages(include=config["tool"]["setuptools"]["packages"]["find"]["include"]),
    entry_points={"console_scripts": [f"{name} = {target}" for name,target in project.get("scripts", {}).items()]},
)
