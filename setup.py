from setuptools import find_packages, setup
import os
from glob import glob

package_name = 'p2-drone-formation-control-simulator'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, 'launch'), glob('launch/*.launch.py')),
        (os.path.join('share', package_name, 'launch'), glob('launch/*.xml')),
        (os.path.join('share', package_name, 'world'), glob('world/*.sdf')),
        (os.path.join('share', package_name, 'parameters'), glob('parameters/*.parm')),
        (os.path.join('share', package_name, 'models/iris1'), glob('models/iris1/*')),
        (os.path.join('share', package_name, 'models/iris2'), glob('models/iris2/*')),
        (os.path.join('share', package_name, 'models/iris3'), glob('models/iris3/*'))

    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='michael',
    maintainer_email='michael@todo.todo',
    description='TODO: Package description',
    license='Apache-2.0',
    #tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'cmd_pub = p2_drone_formation_control_simulator.test:main',
            'cmd_node = p2_drone_formation_control_simulator.cmd_node:main',
            'copter1_cmd_node = p2_drone_formation_control_simulator.copter1_cmd_node:main',
            'copter2_cmd_node = p2_drone_formation_control_simulator.copter2_cmd_node:main',
            'copter3_cmd_node = p2_drone_formation_control_simulator.copter3_cmd_node:main',
            'gui_node = p2_drone_formation_control_simulator.gui_node:main',
        ],
    },
)
