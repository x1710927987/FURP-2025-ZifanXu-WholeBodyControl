from setuptools import find_packages, setup

package_name = 'bme_ros2_tutorials_py'

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
    maintainer='Zifan',
    maintainer_email='zifanxu368@gmail.com',
    description='Practicing ROS2 with Python',
    license='MIT',
    extras_require={
        'test': [
            'pytest',
        ],
    },
    entry_points={
        'console_scripts': [
            "hello_world_py = bme_ros2_tutorials_py.hello_world:main",
            "publisher_py = bme_ros2_tutorials_py.publisher:main",
            "oop_publisher_py = bme_ros2_tutorials_py.oop_publisher:main",
            "subscriber_py = bme_ros2_tutorials_py.subscriber:main",
            "oop_subscriber_py = bme_ros2_tutorials_py.oop_subscriber:main"
        ],
    },
)
