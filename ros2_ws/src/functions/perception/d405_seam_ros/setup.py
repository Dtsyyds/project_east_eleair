from setuptools import setup
setup(name='d405_seam_ros', version='0.1.0', packages=['d405_seam_ros'],
      data_files=[('share/ament_index/resource_index/packages', ['resource/d405_seam_ros']),
                  ('share/d405_seam_ros', ['package.xml']), ('share/d405_seam_ros/launch', ['launch/tracking.launch.py'])],
      install_requires=['setuptools', 'd405-seam-tracking==0.1.0'], zip_safe=True,
      maintainer='sunrise', maintainer_email='sunrise@example.invalid', description='Passive D405 experiment adapter', license='Apache-2.0',
      entry_points={'console_scripts': ['d405_seam_node = d405_seam_ros:main']})
