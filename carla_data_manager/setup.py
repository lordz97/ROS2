from setuptools import find_packages, setup

package_name = 'carla_data_manager'

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
    maintainer='mobilabls',
    maintainer_email='mobilabls@todo.todo',
    description='TODO: Package description',
    license='TODO: License declaration',
    tests_require=['pytest'],
entry_points={
    'console_scripts': [
        'lidar_publisher = carla_data_manager.lidar_publisher:main',
        'lidar_subscriber = carla_data_manager.lidar_subscriber:main',
         'buffer = carla_data_manager.buffer:main',
        
    ],
},
)
