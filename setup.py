from setuptools import find_packages, setup

package_name = 'peak_cam_py'

setup(
    name=package_name,
    version='0.0.1',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='Sherif Nekkah',
    maintainer_email='sherif.nekkah@gmail.com',
    description='Python ROS 2 node for IDS peak cameras (ids-peak PyPI)',
    license='BSD',
    entry_points={
        'console_scripts': [
            'peak_cam_py_node = peak_cam_py.peak_cam_node:main',
        ],
    },
)
