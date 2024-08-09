from setuptools import find_packages, setup

package_name = 'rrl_spot_nav'

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
    maintainer='max',
    maintainer_email='max@todo.todo',
    description='TODO: Package description',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'poses_to_trajectory = rrl_spot_nav.poses_to_trajectory:main',
            'set_start_marker = rrl_spot_nav.set_start_marker:main',
            'set_goal_marker = rrl_spot_nav.set_goal_marker:main',
            'path_follower = rrl_spot_nav.pathfollower:main',
            'rrt_path_follower = rrl_spot_nav.rrt_path_follower:main'
        ],
    },
)
