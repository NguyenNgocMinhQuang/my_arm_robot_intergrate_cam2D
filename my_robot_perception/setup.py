from setuptools import find_packages, setup

package_name = 'my_robot_perception'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='minh_quang',
    maintainer_email='quangwww602@gmail.com',
    description='Perception node: phat hien ArUco marker tu webcam va publish pose 3D + TF',
    license='TODO: License declaration',
    extras_require={
        'test': [
            'pytest',
        ],
    },
    entry_points={
        'console_scripts': [
            'aruco_detector_node = my_robot_perception.aruco_detector_node:main',
        ],
    },
)
