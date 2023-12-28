from setuptools import find_packages, setup

package_name = 'kinect_robot_controller'

setup(
    name=package_name,
    version='1.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='dawidexpompa',
    maintainer_email='xalpol12@gmail.com',
    description='Image detection using Kinect v1 with OpenCV library',
    license='BSD',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'rgbd_image_publisher = kinect_robot_controller.rgbd_image_publisher:main',
            'kinect_image_processor = kinect_robot_controller.kinect_image_processor:main',
            'robot_controller = kinect_robot_controller.robot_controller:main'
        ],
    },
)
