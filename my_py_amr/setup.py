from setuptools import find_packages, setup

package_name = 'my_py_amr'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (f'share/{package_name}/launch', ['launch/camera_bridge_launch.py']),
        (f'share/{package_name}/params', ['params/nav2_params.yaml']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='shree',
    maintainer_email='shree@todo.todo',
    description='TODO: Package description',
    license='TODO: License declaration',
    extras_require={
        'test': [
            'pytest',
        ],
    },
    entry_points={
        'console_scripts': [
            'camera_bridge = my_py_amr.camera_bridge_node:main',
            'image_to_occupancy = my_py_amr.image_to_occupancy:main',
            'top_down_marker_tf = my_py_amr.top_down_marker_tf:main',
            'republish_cmd_vel = my_py_amr.republish_cmd_vel:main',
            'republish_odometry = my_py_amr.republish_odometry:main',
            'republish_joint_states = my_py_amr.republish_joint_states:main',
        ],
    },
)
