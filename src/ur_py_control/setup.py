from setuptools import setup
from glob import glob

package_name = 'ur_py_control'

setup(
    name=package_name,
    version='0.0.0',
    packages=[package_name],
    data_files=[
        ("share/ament_index/resource_index/packages", ["resource/" + package_name]),
        ("share/" + package_name, ["package.xml"]),
        ("share/" + package_name, glob("launch/*.launch.py")),
        ("share/" + package_name + '/share' "/config", glob("config/*.*"))
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='root',
    maintainer_email='root@todo.todo',
    description='TODO: Package description',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        "console_scripts": [
            "publisher_joint_trajectory_controller = \
                ur_py_control.publisher_joint_trajectory_controller:main",
            'show_joint_states = ur_py_control.show_joint_states:main',
            "publisher_joint_trajectory_controller_IK = \
                ur_py_control.publisher_joint_trajectory_controller_IK:main",
        ],
    },
)
