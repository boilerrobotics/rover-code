from setuptools import find_packages, setup

package_name = 'pos_controller'

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
    maintainer='brc',
    maintainer_email='thirawat.tam@gmail.com',
    description='Node to make rover go to specific pos',
    license='Apache-2.0',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            "pos_controller_node = pos_controller.pos_controller_node:main"
        ],
    },
)
