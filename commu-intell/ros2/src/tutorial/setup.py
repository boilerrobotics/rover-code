from setuptools import setup

package_name = 'tutorial'

setup(
    name=package_name,
    version='0.1.0',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='tam',
    maintainer_email='thirawat.tam@gmail.com',
    description='Tutorial for learning how to use ROS2',
    license='MIT License',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'talker = tutorial.publisher:main',
        ],
    },
)
