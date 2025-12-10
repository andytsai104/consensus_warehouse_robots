from setuptools import find_packages, setup

package_name = 'fleet_manager'

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
    maintainer='andy',
    maintainer_email='andytsai104@gmail.com',
    description='Core tasks manager for consensus warehouse robots',
    license='TODO: License declaration',
    extras_require={
        'test': [
            'pytest',
        ],
    },
    entry_points={
        'console_scripts': [
            'warehouse_robot_controller = fleet_manager.warehouse_robot_controller:main',
            'auctioneer_node = fleet_manager.auctioneer_node:main',   
            'bidder_node = fleet_manager.bidder_node:main',    
            'data_logger_node = fleet_manager.data_logger_node:main',
        ],
    },
)
