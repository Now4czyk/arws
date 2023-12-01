from setuptools import find_packages, setup

package_name = 'gripper_control'

setup(
    name=package_name,
    version='0.0.0',
    data_files=[
        ('share/' + package_name, ['package.xml']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='teba_knowaczyk',
    maintainer_email='k.nowaczyk@teb-akademia.pl',
    description='TODO: Package description',
    license='TODO: License declaration',
    entry_points={
        'console_scripts': [
            "gripper_control_node = gripper_control.gripper_control_node:main"
        ],
    },
)
