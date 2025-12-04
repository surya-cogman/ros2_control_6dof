from setuptools import find_packages, setup

package_name = 'my_trajectory'

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
    maintainer='itachi',
    maintainer_email='surya.roboengr@gmail.com',
    description='TODO: Package description',
    license='TODO: License declaration',
    extras_require={
        'test': [
            'pytest',
        ],
    },
    entry_points={
        'console_scripts': [
            'trapezoidal_controller = my_trajectory.trapezoidal_controller:main',
            'arm_gui = my_trajectory.arm_gui:main',
            'trap_gui_node = my_trajectory.trap_gui_node:main',
        ],
    },
)
