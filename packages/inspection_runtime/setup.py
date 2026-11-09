from setuptools import setup, find_packages
setup(name='inspection-runtime', version="0.1.0", packages=find_packages("src"), package_dir={"": "src"}, python_requires=">=3.10", install_requires=['PyYAML>=6.0', 'emat-core==0.1.0'])
