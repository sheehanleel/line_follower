from setuptools import find_packages, setup

package_name = 'line_follower'
data_files = []
data_files.append(('share/ament_index/resource_index/packages', ['resource/' + package_name]))
data_files.append(('share/' + package_name + '/launch', ['launch/line_follower_launch.py']))
data_files.append(('share/' + package_name + '/worlds', ['worlds/e-puck_line_follower.wbt']))
data_files.append(('share/' + package_name + '/resource', ['resource/my_robot.urdf']))
data_files.append(('share/' + package_name + '/config', ['config/line_follower_params.yaml']))
data_files.append(('share/' + package_name, ['package.xml']))

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=data_files,
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='Sheehan',
    maintainer_email='sheehanleel@yahoo.com',
    description='Assessment Task 3 – ROS2 Line Follower',
    license='Apache 2.0',
    extras_require={
        'test': [
            'pytest',
        ],
    },
    entry_points={
        'console_scripts': [
            'epuck_driver = line_follower.epuck_driver:main',
            'line_follower_node = line_follower.line_follower_node:main',
        ],
    },
)
