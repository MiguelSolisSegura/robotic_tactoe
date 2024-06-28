from setuptools import setup

package_name = 'image_processing'

setup(
    name=package_name,
    version='0.0.0',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
    ],
    install_requires=['setuptools', 'rclpy', 'sensor_msgs', 'cv_bridge', 'opencv-python', 'moveit_planning'],
    zip_safe=True,
    maintainer='user',
    maintainer_email='miguelsolissegura@outlook.com',
    description='TODO: Package description',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'get_board_state = image_processing.get_board_state:main',
            'compute_best_move = image_processing.compute_best_move:main',
        ],
    },
)
