import os
from glob import glob
from setuptools import find_packages, setup

package_name = 'moveit2py_humble_example'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, 'launch'), glob('launch/*.launch.py')),
        (os.path.join('share', package_name, 'config'), glob('config/*.yaml')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='kalman',
    maintainer_email='samuele.sandrini@polito.it',
    description='TODO: Package description',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'pose_goal = python_examples.pose_goal:main',
            'joint_goal = python_examples.joint_goal:main',
            'named_goal = python_examples.named_goal:main',
            'pose_goal_linear = python_examples.pose_goal_linear:main',
            'pose_goal_with_objects = python_examples.pose_goal_with_objects:main'
        ],
    },
)
